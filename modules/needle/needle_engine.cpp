#include "needle_engine.h"

#include "core/io/file_access.h"
#include "core/os/os.h"

// needle.h defaults NEEDLE_API to __attribute__((visibility("default"))), which MSVC
// does not understand. We link the static library directly, so export visibility is
// irrelevant here: define it empty before including the header.
#ifndef NEEDLE_API
#define NEEDLE_API
#endif
#include "needle.h"

#ifdef NEEDLE_WINDOWS_DLL
// Exported by needle.dll (see make_needle_dll.ps1). Sets a variable in the DLL's own
// C runtime, which Godot's set_environment() cannot reach.
extern "C" void needle_shim_putenv(const char *name, const char *value);
#endif

NeedleEngine *NeedleEngine::singleton = nullptr;

namespace {

// Holds a UTF-8 copy and hands out nullptr for empty strings ("use the default").
struct OptionalUtf8 {
	CharString data;
	bool empty;
	explicit OptionalUtf8(const String &p_str) :
			data(p_str.utf8()), empty(p_str.is_empty()) {}
	const char *get() const { return empty ? nullptr : data.get_data(); }
};

String join_keywords(const PackedStringArray &p_keywords) {
	String joined;
	for (int i = 0; i < p_keywords.size(); i++) {
		String k = p_keywords[i].strip_edges();
		if (k.is_empty()) {
			continue;
		}
		if (!joined.is_empty()) {
			joined += "\n";
		}
		joined += k;
	}
	return joined; // newline-separated, as needle.h expects
}

} // namespace

NeedleEngine::NeedleEngine() {
	singleton = this;
	// Telemetry is on by default in the Needle binary. Turn it off for a shipped game.
	// Remove these two lines if you want to keep it.
	OS::get_singleton()->set_environment("NEEDLE_TELEMETRY", "0");
	OS::get_singleton()->set_environment("DO_NOT_TRACK", "1");
#ifdef NEEDLE_WINDOWS_DLL
	needle_shim_putenv("NEEDLE_TELEMETRY", "0");
	needle_shim_putenv("DO_NOT_TRACK", "1");
#endif
}

NeedleEngine::~NeedleEngine() {
	singleton = nullptr;
}

String NeedleEngine::_engine_error() const {
	const char *e = needle_last_error();
	return e ? String::utf8(e) : String("unknown error");
}

// Turns a needle_* return code plus output buffer into a String.
// Note: the return value is a count (tokens, words, ...), NOT the byte length of the
// JSON, so the text is read up to its NUL terminator instead.
String NeedleEngine::_finish(int p_result, LocalVector<char> &p_out, const char *p_fn) {
	if (p_result < 0) {
		String err = _engine_error();
		ERR_FAIL_V_MSG(String(), vformat("Needle: %s failed (code %d): %s", p_fn, p_result, err));
	}
	p_out[p_out.size() - 1] = '\0'; // guarantee termination
	return String::utf8(p_out.ptr());
}

// ---------------------------------------------------------------- models

Error NeedleEngine::load_model(const String &p_path) {
	Error err = OK;
	// Works for res:// (including inside an exported PCK) and user:// paths.
	Vector<uint8_t> bytes = FileAccess::get_file_as_bytes(p_path, &err);
	ERR_FAIL_COND_V_MSG(err != OK, err, "Needle: could not read model file: " + p_path);
	ERR_FAIL_COND_V_MSG(bytes.is_empty(), ERR_FILE_CORRUPT, "Needle: model file is empty: " + p_path);

	MutexLock lock(mutex);
	int r = needle_load(bytes.ptr(), (unsigned long long)bytes.size());
	if (r < 0) {
		String e = _engine_error();
		ERR_FAIL_V_MSG(FAILED, vformat("Needle: needle_load failed for %s (code %d): %s", p_path, r, e));
	}
	model_buffers.push_back(bytes);
	return OK;
}

int NeedleEngine::get_models() {
	MutexLock lock(mutex);
	return needle_models();
}

bool NeedleEngine::has_text_model() {
	return (get_models() & NEEDLE_TEXT) != 0;
}

bool NeedleEngine::has_speech_model() {
	return (get_models() & NEEDLE_SPEECH) != 0;
}

// ---------------------------------------------------------------- tool calling

int NeedleEngine::configure(const String &p_system, const String &p_tools_json, const String &p_tool_index_path) {
	MutexLock lock(mutex);
	OptionalUtf8 sys(p_system), tools(p_tools_json), index(p_tool_index_path);
	int r = needle_init(sys.get(), tools.get(), index.get());
	if (r < 0) {
		// Typical cause: system prompt + tools exceed the context window.
		String e = _engine_error();
		ERR_FAIL_V_MSG(r, vformat("Needle: needle_init failed (code %d): %s", r, e));
	}
	configured = true;
	return r; // tokenized static-prefix length
}

String NeedleEngine::complete(const String &p_input, int p_max_new_tokens) {
	MutexLock lock(mutex);
	CharString in = p_input.utf8();
	LocalVector<char> out;
	out.resize(output_capacity);
	out[0] = '\0';
	int r = needle_complete(in.get_data(), nullptr, 0, p_max_new_tokens, out.ptr(), (int)out.size());
	return _finish(r, out, "needle_complete");
}

String NeedleEngine::complete_audio(const PackedFloat32Array &p_pcm_16k_mono, int p_max_new_tokens) {
	ERR_FAIL_COND_V_MSG(p_pcm_16k_mono.is_empty(), String(), "Needle: complete_audio got an empty clip.");
	MutexLock lock(mutex);
	LocalVector<char> out;
	out.resize(output_capacity);
	out[0] = '\0';
	// Exactly one of input / pcm is non-null. The engine transcribes with the speech model,
	// answers the transcript against the tools and merges audio_* fields into the JSON.
	int r = needle_complete(nullptr, p_pcm_16k_mono.ptr(), p_pcm_16k_mono.size(), p_max_new_tokens,
			out.ptr(), (int)out.size());
	return _finish(r, out, "needle_complete (audio)");
}

void NeedleEngine::reset() {
	MutexLock lock(mutex);
	needle_reset();
}

// ---------------------------------------------------------------- speech

void NeedleEngine::set_audio(const String &p_language, const PackedStringArray &p_keywords, bool p_word_timestamps) {
	MutexLock lock(mutex);
	OptionalUtf8 lang(p_language), kw(join_keywords(p_keywords));
	needle_set_audio(lang.get(), kw.get(), p_word_timestamps ? 1 : 0); // strings are copied by the engine
}

String NeedleEngine::transcribe(const PackedFloat32Array &p_pcm_16k_mono, const String &p_language,
		const PackedStringArray &p_keywords, bool p_word_timestamps) {
	ERR_FAIL_COND_V_MSG(p_pcm_16k_mono.is_empty(), String(), "Needle: transcribe got an empty clip.");
	MutexLock lock(mutex);
	OptionalUtf8 lang(p_language), kw(join_keywords(p_keywords));
	LocalVector<char> out;
	out.resize(output_capacity);
	out[0] = '\0';
	int r = needle_transcribe(p_pcm_16k_mono.ptr(), p_pcm_16k_mono.size(), lang.get(), kw.get(),
			p_word_timestamps ? 1 : 0, out.ptr(), (int)out.size());
	return _finish(r, out, "needle_transcribe");
}

String NeedleEngine::stream_transcribe_process(const PackedFloat32Array &p_pcm_16k_mono, const String &p_language,
		const PackedStringArray &p_keywords) {
	MutexLock lock(mutex);
	OptionalUtf8 lang(p_language), kw(join_keywords(p_keywords));
	LocalVector<char> out;
	out.resize(output_capacity);
	out[0] = '\0';
	int r = needle_stream_transcribe_process(p_pcm_16k_mono.ptr(), p_pcm_16k_mono.size(), lang.get(), kw.get(),
			out.ptr(), (int)out.size());
	return _finish(r, out, "needle_stream_transcribe_process");
}

String NeedleEngine::stream_transcribe_stop() {
	MutexLock lock(mutex);
	LocalVector<char> out;
	out.resize(output_capacity);
	out[0] = '\0';
	int r = needle_stream_transcribe_stop(out.ptr(), (int)out.size());
	return _finish(r, out, "needle_stream_transcribe_stop");
}

// ---------------------------------------------------------------- embeddings

PackedFloat32Array NeedleEngine::embed_text(const String &p_text) {
	MutexLock lock(mutex);
	CharString in = p_text.utf8();
	int count = needle_embed(in.get_data(), nullptr, 0, nullptr, 0); // null output: size query
	if (count < 0) {
		String e = _engine_error();
		ERR_FAIL_V_MSG(PackedFloat32Array(), vformat("Needle: needle_embed size query failed (code %d): %s", count, e));
	}
	PackedFloat32Array result;
	result.resize(count);
	int r = needle_embed(in.get_data(), nullptr, 0, result.ptrw(), count);
	if (r < 0) {
		String e = _engine_error();
		ERR_FAIL_V_MSG(PackedFloat32Array(), vformat("Needle: needle_embed failed (code %d): %s", r, e));
	}
	return result;
}

PackedFloat32Array NeedleEngine::embed_audio(const PackedFloat32Array &p_pcm_16k_mono) {
	ERR_FAIL_COND_V_MSG(p_pcm_16k_mono.is_empty(), PackedFloat32Array(), "Needle: embed_audio got an empty clip.");
	MutexLock lock(mutex);
	int count = needle_embed(nullptr, p_pcm_16k_mono.ptr(), p_pcm_16k_mono.size(), nullptr, 0);
	if (count < 0) {
		String e = _engine_error();
		ERR_FAIL_V_MSG(PackedFloat32Array(), vformat("Needle: needle_embed size query failed (code %d): %s", count, e));
	}
	PackedFloat32Array result;
	result.resize(count); // one row per 80 ms frame, flattened
	int r = needle_embed(nullptr, p_pcm_16k_mono.ptr(), p_pcm_16k_mono.size(), result.ptrw(), count);
	if (r < 0) {
		String e = _engine_error();
		ERR_FAIL_V_MSG(PackedFloat32Array(), vformat("Needle: needle_embed failed (code %d): %s", r, e));
	}
	return result;
}

// ---------------------------------------------------------------- diagnostics

String NeedleEngine::get_last_error() {
	MutexLock lock(mutex);
	return _engine_error();
}

void NeedleEngine::set_output_capacity(int p_bytes) {
	ERR_FAIL_COND_MSG(p_bytes < 1024, "Needle: output capacity must be at least 1024 bytes.");
	MutexLock lock(mutex);
	output_capacity = p_bytes;
}

void NeedleEngine::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_model", "path"), &NeedleEngine::load_model);
	ClassDB::bind_method(D_METHOD("get_models"), &NeedleEngine::get_models);
	ClassDB::bind_method(D_METHOD("has_text_model"), &NeedleEngine::has_text_model);
	ClassDB::bind_method(D_METHOD("has_speech_model"), &NeedleEngine::has_speech_model);

	ClassDB::bind_method(D_METHOD("configure", "system", "tools_json", "tool_index_path"), &NeedleEngine::configure, DEFVAL(String()));
	ClassDB::bind_method(D_METHOD("complete", "input", "max_new_tokens"), &NeedleEngine::complete, DEFVAL(256));
	ClassDB::bind_method(D_METHOD("complete_audio", "pcm_16k_mono", "max_new_tokens"), &NeedleEngine::complete_audio, DEFVAL(256));
	ClassDB::bind_method(D_METHOD("reset"), &NeedleEngine::reset);

	ClassDB::bind_method(D_METHOD("set_audio", "language", "keywords", "word_timestamps"), &NeedleEngine::set_audio);
	ClassDB::bind_method(D_METHOD("transcribe", "pcm_16k_mono", "language", "keywords", "word_timestamps"), &NeedleEngine::transcribe,
			DEFVAL(String()), DEFVAL(PackedStringArray()), DEFVAL(false));
	ClassDB::bind_method(D_METHOD("stream_transcribe_process", "pcm_16k_mono", "language", "keywords"), &NeedleEngine::stream_transcribe_process,
			DEFVAL(String()), DEFVAL(PackedStringArray()));
	ClassDB::bind_method(D_METHOD("stream_transcribe_stop"), &NeedleEngine::stream_transcribe_stop);

	ClassDB::bind_method(D_METHOD("embed_text", "text"), &NeedleEngine::embed_text);
	ClassDB::bind_method(D_METHOD("embed_audio", "pcm_16k_mono"), &NeedleEngine::embed_audio);

	ClassDB::bind_method(D_METHOD("get_last_error"), &NeedleEngine::get_last_error);
	ClassDB::bind_method(D_METHOD("is_configured"), &NeedleEngine::is_configured);
	ClassDB::bind_method(D_METHOD("set_output_capacity", "bytes"), &NeedleEngine::set_output_capacity);
	ClassDB::bind_method(D_METHOD("get_output_capacity"), &NeedleEngine::get_output_capacity);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "output_capacity"), "set_output_capacity", "get_output_capacity");

	BIND_ENUM_CONSTANT(MODEL_TEXT);
	BIND_ENUM_CONSTANT(MODEL_SPEECH);
}
