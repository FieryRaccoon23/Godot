# Activate venv if not active
if (-not $env:VIRTUAL_ENV) {
    .\venv_Godot\Scripts\Activate.ps1
}

# 1. Recompile C++ Engine
scons platform=windows target=editor module_mono_enabled=yes d3d12=no -j8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 2. Regenerate Glue
.\bin\godot.windows.editor.x86_64.mono.exe --generate-mono-glue modules/mono/glue

# 3. Build C# Assemblies
python modules\mono\build_scripts\build_assemblies.py --godot-output-dir=bin