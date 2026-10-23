def can_build(env, platform):
    # Prebuilt engines exist for linuxbsd, macos (arm64), windows, android and ios.
    return platform in ("linuxbsd", "macos", "windows", "android", "ios")


def configure(env):
    pass


def get_doc_classes():
    return ["NeedleEngine"]


def get_doc_path():
    return "doc_classes"
