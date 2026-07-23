[CmdletBinding()]
param(
    [string]$RealStepPath
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$outputDir = Join-Path $repo 'tmp\TheoreticalRobotModelStoreTests'
$outputExe = Join-Path $outputDir 'TheoreticalRobotModelStoreTests.exe'

foreach ($required in @(
    $vsDevCmd,
    (Join-Path $qtRoot 'include\QtCore\QCoreApplication'),
    (Join-Path $qtRoot 'lib\Qt6Core.lib'),
    (Join-Path $repo 'include\TheoreticalRobotModelStore.h'),
    (Join-Path $repo 'src\TheoreticalRobotModelStore.cpp'),
    (Join-Path $repo 'src\AppPaths.cpp'),
    (Join-Path $repo 'scripts\tests\theoretical_robot_model_store_tests.cpp')
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required theoretical robot model store test dependency not found: $required"
    }
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/O2', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWIN32_LEAN_AND_MEAN',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\TheoreticalRobotModelStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\theoretical_robot_model_store_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), 'Qt6Core.lib', 'shell32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "TheoreticalRobotModelStoreTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe --preinit-probe
if ($LASTEXITCODE -ne 0) {
    throw "TheoreticalRobotModelStore pre-initialization probe failed: $LASTEXITCODE"
}
& $outputExe --junction-probe
if ($LASTEXITCODE -ne 0) {
    throw "TheoreticalRobotModelStore junction probe failed: $LASTEXITCODE"
}
& $outputExe
if ($LASTEXITCODE -ne 0) {
    throw "TheoreticalRobotModelStoreTests failed: $LASTEXITCODE"
}
if (-not [string]::IsNullOrWhiteSpace($RealStepPath)) {
    $resolvedRealStep = (Resolve-Path -LiteralPath $RealStepPath -ErrorAction Stop).Path
    & $outputExe --real-step-probe $resolvedRealStep
    if ($LASTEXITCODE -ne 0) {
        throw "TheoreticalRobotModelStore real STEP probe failed: $LASTEXITCODE"
    }
}
