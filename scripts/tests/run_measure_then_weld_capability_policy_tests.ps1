[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$outputDir = Join-Path $repo 'tmp\MeasureThenWeldCapabilityPolicyTests'
$outputExe = Join-Path $outputDir 'MeasureThenWeldCapabilityPolicyTests.exe'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/utf-8', '/W4',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\measure_then_weld_capability_policy_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe)
) -join ' '
& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) { throw "Capability policy test compilation failed: $LASTEXITCODE" }
& $outputExe
if ($LASTEXITCODE -ne 0) { throw "Capability policy tests failed: $LASTEXITCODE" }
