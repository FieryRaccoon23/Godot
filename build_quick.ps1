# Usage:
#   .\build_quick.ps1             build with Needle
#   .\build_quick.ps1 -NoNeedle   build without Needle
#
# Recompiles C++ only. Fine for changes inside function bodies.
# If you add, remove or rename bound methods, or switch Needle on/off,
# run build_full.ps1 instead so the C# glue is regenerated.
param([switch]$NoNeedle)

# Activate venv if not active
if (-not $env:VIRTUAL_ENV) {
    .\venv_Godot\Scripts\Activate.ps1
}

if ($NoNeedle) {
    $needleFlag = "module_needle_enabled=no"
} else {
    $needleFlag = "module_needle_enabled=yes"
}

# Recompile C++ Engine
scons platform=windows target=editor module_mono_enabled=yes d3d12=no $needleFlag -j8
