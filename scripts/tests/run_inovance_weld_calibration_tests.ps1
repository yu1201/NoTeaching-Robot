[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$qtRoot = Join-Path $repo 'output\QtWidgetsApplication4-third-party-dependencies-20260819\dependencies\Qt\6.7.3\msvc2022_64'
$eigenRoot = Join-Path $repo 'output\QtWidgetsApplication4-third-party-dependencies-20260819\dependencies\Eigen\3.4.0\eigen-3.4.0'
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$outputDir = Join-Path $repo 'tmp\InovanceWeldCalibrationTests'
$outputExe = Join-Path $outputDir 'InovanceWeldCalibrationTests.exe'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f (Join-Path $qtRoot 'include')),
    ('/I"{0}"' -f (Join-Path $qtRoot 'include\QtCore')),
    ('/I"{0}"' -f $eigenRoot),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\inovance_weld_calibration_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), '/link', ('/LIBPATH:"{0}"' -f (Join-Path $qtRoot 'lib')), 'Qt6Core.lib'
) -join ' '
& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) { throw "Inovance calibration test compilation failed: $LASTEXITCODE" }
$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe
if ($LASTEXITCODE -ne 0) { throw "Inovance calibration tests failed: $LASTEXITCODE" }
