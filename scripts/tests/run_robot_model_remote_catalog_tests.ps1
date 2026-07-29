[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$outputDir = Join-Path $repo 'tmp\RobotModelRemoteCatalogTests'
$outputExe = Join-Path $outputDir 'RobotModelRemoteCatalogTests.exe'

foreach ($required in @(
    $vsDevCmd,
    (Join-Path $qtRoot 'include\QtCore\QCoreApplication'),
    (Join-Path $qtRoot 'include\QtGui\QGuiApplication'),
    (Join-Path $qtRoot 'lib\Qt6Core.lib'),
    (Join-Path $qtRoot 'lib\Qt6Gui.lib'),
    (Join-Path $eigenRoot 'Eigen\Core'),
    (Join-Path $repo 'include\RobotModelRemoteCatalog.h'),
    (Join-Path $repo 'src\RobotModelRemoteCatalog.cpp'),
    (Join-Path $repo 'src\RobotCollisionEnvelopeStore.cpp'),
    (Join-Path $repo 'src\AppPaths.cpp'),
    (Join-Path $repo 'scripts\tests\robot_model_remote_catalog_tests.cpp')
)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "required remote robot model catalog test dependency not found: $required"
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
    ('/I"{0}\QtGui"' -f $qtInclude),
    ('/I"{0}"' -f $eigenRoot),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotCollisionEnvelopeStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\RobotModelRemoteCatalog.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_model_remote_catalog_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe),
    '/link',
    ('/LIBPATH:"{0}"' -f $qtLib),
    'Qt6Core.lib',
    'Qt6Gui.lib',
    'shell32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelRemoteCatalogTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
$env:QT_QPA_PLATFORM_PLUGIN_PATH =
    (Join-Path $qtRoot 'plugins\platforms')
$env:QT_QPA_PLATFORM = 'offscreen'
& $outputExe
if ($LASTEXITCODE -ne 0) {
    throw "RobotModelRemoteCatalogTests failed: $LASTEXITCODE"
}
