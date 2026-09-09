[CmdletBinding()]
param([string]$QtRoot)
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if (-not $QtRoot) { $QtRoot = Join-Path $repo 'output\QtWidgetsApplication4-third-party-dependencies-20260819\dependencies\Qt\6.7.3\msvc2022_64' }
$outputDir = Join-Path $repo 'tmp\RobotAdaptorAcceptanceLayoutTests'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$outputExe = Join-Path $outputDir 'RobotAdaptorAcceptanceLayoutTests.exe'
$qtInclude = Join-Path $QtRoot 'include'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/MD', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude), ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtGui"' -f $qtInclude), ('/I"{0}\QtWidgets"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\WindowStyleHelper.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_adaptor_acceptance_layout_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), '/link', ('/LIBPATH:"{0}"' -f (Join-Path $QtRoot 'lib')),
    'Qt6Core.lib', 'Qt6Gui.lib', 'Qt6Widgets.lib', 'User32.lib', 'Gdi32.lib', 'Dwmapi.lib'
) -join ' '
& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) { throw "Layout test compilation failed: $LASTEXITCODE" }
$env:PATH = (Join-Path $QtRoot 'bin') + ';' + $env:PATH
$env:QT_PLUGIN_PATH = Join-Path $QtRoot 'plugins'
$env:QT_QPA_PLATFORM = if (Test-Path -LiteralPath (Join-Path $QtRoot 'plugins\platforms\qoffscreen.dll')) { 'offscreen' } else { 'windows' }
& $outputExe $outputDir
if ($LASTEXITCODE -ne 0) { throw "Layout tests failed: $LASTEXITCODE" }
