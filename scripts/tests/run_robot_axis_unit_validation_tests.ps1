$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$outputDir = Join-Path $repo 'tmp\RobotAxisUnitValidationTests'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$exe = Join-Path $outputDir 'RobotAxisUnitValidationTests.exe'
$vs = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$compile = 'call "{0}" -arch=x64 -host_arch=x64 >nul 2>nul && cd /d "{1}" && cl /nologo /EHsc /std:c++17 /utf-8 /I"{2}\include" "{2}\scripts\tests\robot_axis_unit_validation_tests.cpp" /Fe:"{3}"' -f $vs,$outputDir,$repo,$exe
& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) { throw 'Axis unit validation tests compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Axis unit validation tests failed' }
