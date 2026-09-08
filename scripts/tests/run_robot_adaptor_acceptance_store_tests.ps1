[CmdletBinding()]
param([string]$QtRoot)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if (-not $QtRoot) {
    $QtRoot = Join-Path $repo 'output\QtWidgetsApplication4-third-party-dependencies-20260819\dependencies\Qt\6.7.3\msvc2022_64'
}
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$outputDir = Join-Path $repo 'tmp\RobotAdaptorAcceptanceStoreTests'
$outputExe = Join-Path $outputDir 'RobotAdaptorAcceptanceStoreTests.exe'
$suiteRoot = Join-Path $outputDir ('Suite-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $suiteRoot -Force | Out-Null
$qtInclude = Join-Path $QtRoot 'include'
$qtLib = Join-Path $QtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/MD', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude), ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtNetwork"' -f $qtInclude), ('/I"{0}\QtSql"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ConfigDatabase.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\CredentialSecurity.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\robot_adaptor_acceptance_store_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), '/link', ('/LIBPATH:"{0}"' -f $qtLib),
    'Qt6Core.lib', 'Qt6Network.lib', 'Qt6Sql.lib', 'Crypt32.lib', 'shell32.lib'
) -join ' '
& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) { throw "Acceptance store tests compilation failed: $LASTEXITCODE" }
$env:PATH = (Join-Path $QtRoot 'bin') + ';' + $env:PATH
$env:QT_PLUGIN_PATH = Join-Path $QtRoot 'plugins'
foreach ($phase in @('seed', 'restart', 'verify')) {
    & $outputExe $suiteRoot $phase
    if ($LASTEXITCODE -ne 0) { throw "Acceptance store tests failed in phase ${phase}: $LASTEXITCODE" }
}
