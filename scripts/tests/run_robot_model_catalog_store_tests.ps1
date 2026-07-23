[CmdletBinding()]
param(
    [string]$RealSa10StepPath,
    [string]$RealSa10EnvelopePath
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$outputDir = Join-Path $repo 'tmp\RobotModelCatalogStoreTests'
$outputExe = Join-Path $outputDir 'RobotModelCatalogStoreTests.exe'

foreach ($required in @(
    $vsDevCmd,
    (Join-Path $qtRoot 'include\QtCore\QCoreApplication'),
    (Join-Path $qtRoot 'lib\Qt6Core.lib'),
    (Join-Path $eigenRoot 'Eigen\Core'),
    (Join-Path $repo 'include\RobotModelCatalogStore.h'),
    (Join-Path $repo 'src\RobotModelCatalogStore.cpp'),
    (Join-Path $repo 'src\TheoreticalRobotModelStore.cpp'),
    (Join-Path $repo 'src\RobotCollisionEnvelopeStore.cpp'),
    (Join-Path $repo 'src\AppPaths.cpp'),
    (Join-Path $repo 'scripts\tests\robot_model_catalog_store_tests.cpp')
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required robot model catalog test dependency not found: $required"
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
    '/DROBOT_MODEL_CATALOG_ENABLE_TEST_TRUST',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}"' -f $eigenRoot),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\TheoreticalRobotModelStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotCollisionEnvelopeStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotModelCatalogStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_model_catalog_store_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), 'Qt6Core.lib', 'shell32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelCatalogStoreTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe --preinit-probe
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelCatalogStore preinit probe failed: $LASTEXITCODE"
}
& $outputExe
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelCatalogStoreTests failed: $LASTEXITCODE"
}

if (-not [string]::IsNullOrWhiteSpace($RealSa10StepPath) -or
    -not [string]::IsNullOrWhiteSpace($RealSa10EnvelopePath)) {
    if ([string]::IsNullOrWhiteSpace($RealSa10StepPath) -or
        [string]::IsNullOrWhiteSpace($RealSa10EnvelopePath)) {
        throw 'RealSa10StepPath and RealSa10EnvelopePath must be supplied together'
    }
    $resolvedStep = (Resolve-Path -LiteralPath $RealSa10StepPath -ErrorAction Stop).Path
    $resolvedEnvelope = (Resolve-Path -LiteralPath $RealSa10EnvelopePath -ErrorAction Stop).Path
    & $outputExe --real-sa10-bootstrap $resolvedStep $resolvedEnvelope
    if ($LASTEXITCODE -ne 0) {
        throw "RobotModelCatalogStore real SA10 bootstrap failed: $LASTEXITCODE"
    }
}
