# Activate venv if not active
if (-not $env:VIRTUAL_ENV) {
    .\venv_Godot\Scripts\Activate.ps1
}

# 1. Recompile C++ Engine
scons platform=windows target=editor module_mono_enabled=yes d3d12=no -j8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 1b. Copy MuJoCo runtime dependency next to the built executable
Copy-Item -Path "MuJoCo\bin\mujoco.dll" -Destination "bin\" -Force

# 2. Regenerate Glue
.\bin\godot.windows.editor.x86_64.mono.exe --generate-mono-glue modules/mono/glue
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 3. Build C# Assemblies
python modules\mono\build_scripts\build_assemblies.py --godot-output-dir=bin
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 4. Clear NuGet caches so stale package/assembly metadata never masks a fresh rebuild
dotnet nuget locals all --clear

Write-Host "Build pipeline complete." -ForegroundColor Green