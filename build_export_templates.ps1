# Activate venv if not active
if (-not $env:VIRTUAL_ENV) {
    .\venv_Godot\Scripts\Activate.ps1
}

# 1. Build Release export template
scons platform=windows target=template_release module_mono_enabled=yes d3d12=no -j8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 2. Build Debug export template
scons platform=windows target=template_debug module_mono_enabled=yes d3d12=no -j8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 3. Copy MuJoCo runtime dependency next to both template binaries
Copy-Item -Path "MuJoCo\bin\mujoco.dll" -Destination "bin\" -Force

# 4. Rebuild C# assemblies so they match the freshly built templates
python modules\mono\build_scripts\build_assemblies.py --godot-output-dir=bin
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 5. Clear NuGet caches so nothing stale gets picked up
dotnet nuget locals all --clear

Write-Host "Export templates built successfully." -ForegroundColor Green
Write-Host "Release template: bin\godot.windows.template_release.x86_64.mono.exe" -ForegroundColor Cyan
Write-Host "Debug template:   bin\godot.windows.template_debug.x86_64.mono.exe" -ForegroundColor Cyan
Write-Host "Remember to copy mujoco.dll next to your exported .exe after exporting the project." -ForegroundColor Yellow
