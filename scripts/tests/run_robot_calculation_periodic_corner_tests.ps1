[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$outputDir = Join-Path $repo 'tmp\RobotCalculationPeriodicCornerTests'
$outputExe = Join-Path $outputDir 'RobotCalculationPeriodicCornerTests.exe'

foreach ($required in @($vsDevCmd, $qtRoot, $eigenRoot)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required build dependency not found: $required"
    }
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/Gy', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8', '/DNOMINMAX',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $eigenRoot),
    ('/I"{0}"' -f $qtInclude), ('/I"{0}\QtCore"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\Const.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_calculation_periodic_corner_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), ('/Fo{0}\' -f $outputDir),
    '/link', '/OPT:REF', ('/LIBPATH:"{0}"' -f $qtLib), 'Qt6Core.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "RobotCalculation periodic-corner test compilation failed: $LASTEXITCODE"
}
$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe
if ($LASTEXITCODE -ne 0) {
    throw "RobotCalculation periodic-corner tests failed: $LASTEXITCODE"
}
