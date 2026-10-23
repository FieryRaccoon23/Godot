# Usage:
#   .\build_full.ps1             build with Needle
#   .\build_full.ps1 -NoNeedle   build without Needle
param([switch]$NoNeedle)

# Activate venv if not active
if (-not $env:VIRTUAL_ENV) {
    .\venv_Godot\Scripts\Activate.ps1
}

$editor    = ".\bin\godot.windows.editor.x86_64.mono.exe"
$needleDir = "modules\needle\thirdparty\windows-x86_64"
$glueDir   = "modules\mono\glue\GodotSharp\GodotSharp\Generated\GodotObjects"

# 0. Needle on/off
if ($NoNeedle) {
    $needleFlag = "module_needle_enabled=no"
    Write-Host "Building WITHOUT Needle." -ForegroundColor Yellow
} else {
    $needleFlag = "module_needle_enabled=yes"
    foreach ($f in "needle.h", "needle.dll", "needle.lib") {
        if (-not (Test-Path "$needleDir\$f")) {
            Write-Host "Missing $needleDir\$f" -ForegroundColor Red
            Write-Host "Copy needle.dll, needle.lib and needle.h from your Needle build folder (or build with -NoNeedle)." -ForegroundColor Red
            exit 1
        }
    }
    Write-Host "Building WITH Needle." -ForegroundColor Cyan
}

# 1. Recompile C++ Engine
scons platform=windows target=editor module_mono_enabled=yes d3d12=no $needleFlag -j8
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 1b. Copy runtime DLLs next to the built executable (the editor needs them to start)
Copy-Item -Path "MuJoCo\bin\mujoco.dll" -Destination "bin\" -Force
if (-not $NoNeedle) {
    Copy-Item -Path "$needleDir\needle.dll" -Destination "bin\" -Force
}

# 2. Regenerate Glue
# The editor is a GUI program, so PowerShell does not wait for it when called directly.
# Start-Process -Wait makes sure glue generation has finished (and succeeded) before step 3.
Write-Host "Generating C# glue..." -ForegroundColor Cyan
$p = Start-Process -FilePath $editor -ArgumentList "--headless", "--generate-mono-glue", "modules/mono/glue" `
    -Wait -NoNewWindow -PassThru
if ($p.ExitCode -ne 0) {
    Write-Host "Glue generation failed (exit code $($p.ExitCode))." -ForegroundColor Red
    if ($p.ExitCode -eq -1073741515) {
        Write-Host "That code means a DLL is missing next to the editor (mujoco.dll or needle.dll in bin\)." -ForegroundColor Red
    }
    exit 1
}

# 2b. Verify the glue matches the build
$needleGlue = Test-Path "$glueDir\NeedleEngine.cs"
if (-not $NoNeedle -and -not $needleGlue) {
    Write-Host "Glue was generated but has no NeedleEngine.cs - the editor binary does not contain the Needle module." -ForegroundColor Red
    Write-Host "Check that modules\needle exists and that scons did not print 'needle:' errors." -ForegroundColor Red
    exit 1
}
Write-Host ("Glue OK" + $(if ($needleGlue) { " (NeedleEngine included)." } else { "." })) -ForegroundColor Green

# 3. Build C# Assemblies (also rebuilds the nupkgs in bin\GodotSharp\Tools\nupkgs)
python modules\mono\build_scripts\build_assemblies.py --godot-output-dir=bin
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# 3b. Verify the built assembly
if (-not $NoNeedle) {
    $dll = "bin\GodotSharp\Api\Debug\GodotSharp.dll"
    $hasNeedle = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($dll)).Contains("NeedleEngine")
    if (-not $hasNeedle) {
        Write-Host "$dll does not contain NeedleEngine after building assemblies." -ForegroundColor Red
        exit 1
    }
    Write-Host "GodotSharp.dll contains NeedleEngine." -ForegroundColor Green
}

# 4. Clear NuGet caches so stale package/assembly metadata never masks a fresh rebuild
dotnet nuget locals all --clear

if ($NoNeedle) {
    Write-Host "Build pipeline complete (without Needle). Remove NEEDLE from DefineConstants in your .csproj." -ForegroundColor Green
} else {
    Write-Host "Build pipeline complete (with Needle)." -ForegroundColor Green
}
Write-Host "Before reopening your game project, delete its .godot\mono, bin and obj folders." -ForegroundColor Yellow
