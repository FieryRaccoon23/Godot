# Activate venv if not active
if (-not $env:VIRTUAL_ENV) {
    .\venv_Godot\Scripts\Activate.ps1
}

# Recompile C++ Engine
scons platform=windows target=editor module_mono_enabled=yes d3d12=no -j8