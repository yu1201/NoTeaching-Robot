[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$eigenRoot = Join-Path $repo 'output\QtWidgetsApplication4-third-party-dependencies-20260819\dependencies\Eigen\3.4.0\eigen-3.4.0'
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$outputDir = Join-Path $repo 'tmp\RobotKinematicsModelTests'
$outputExe = Join-Path $outputDir 'RobotKinematicsModelTests.exe'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/permissive-', '/utf-8', '/O2',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $eigenRoot),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_kinematics_model_tests.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotKinematicsModel.cpp')),
    ('/Fe:"{0}"' -f $outputExe)
) -join ' '
& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) { throw "Kinematics model test compilation failed: $LASTEXITCODE" }
& $outputExe
if ($LASTEXITCODE -ne 0) { throw "Kinematics model tests failed: $LASTEXITCODE" }
