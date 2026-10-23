#pragma once

#include "core/object/class_db.h"
#include "core/object/object.h"
#include "core/os/mutex.h"
#include "core/templates/local_vector.h"

// Wraps Needle's native engine. The engine keeps one process-global text model and one
// speech model, is NOT thread-safe, and holds one conversation. Every call below takes
// the mutex, so it is safe to call from worker threads (one call runs at a time).
// Registered as an engine singleton: C# sees it as the static class `NeedleEngine`.
class NeedleEngine : public Object {
	GDCLASS(NeedleEngine, Object);

	static NeedleEngine *singleton;

	Mutex mutex;
	// The engine may read the .cact buffer in place, so keep every loaded buffer alive.
	LocalVector<Vector<uint8_t>> model_buffers;
	int output_capacity = 128 * 1024;
	bool configured = false;

	String _engine_error() const;
	String _finish(int p_result, LocalVector<char> &p_out, const char *p_fn);

protected:
	static void _bind_methods();

public:
	enum ModelKind {
		MODEL_TEXT = 1, // NEEDLE_TEXT
		MODEL_SPEECH = 2, // NEEDLE_SPEECH
	};

	static NeedleEngine *get_singleton() { return singleton; }

	// Models
	Error load_model(const String &p_path);
	int get_models();
	bool has_text_model();
	bool has_speech_model();

	// Tool calling
	int configure(const String &p_system, const String &p_tools_json, const String &p_tool_index_path = String());
	String complete(const String &p_input, int p_max_new_tokens = 256);
	String complete_audio(const PackedFloat32Array &p_pcm_16k_mono, int p_max_new_tokens = 256);
	void reset();

	// Speech
	void set_audio(const String &p_language, const PackedStringArray &p_keywords, bool p_word_timestamps);
	String transcribe(const PackedFloat32Array &p_pcm_16k_mono, const String &p_language = String(),
			const PackedStringArray &p_keywords = PackedStringArray(), bool p_word_timestamps = false);
	String stream_transcribe_process(const PackedFloat32Array &p_pcm_16k_mono, const String &p_language = String(),
			const PackedStringArray &p_keywords = PackedStringArray());
	String stream_transcribe_stop();

	// Embeddings
	PackedFloat32Array embed_text(const String &p_text);
	PackedFloat32Array embed_audio(const PackedFloat32Array &p_pcm_16k_mono);

	// Diagnostics / settings
	String get_last_error();
	bool is_configured() const { return configured; }
	void set_output_capacity(int p_bytes);
	int get_output_capacity() const { return output_capacity; }

	NeedleEngine();
	~NeedleEngine();
};

VARIANT_ENUM_CAST(NeedleEngine::ModelKind);
