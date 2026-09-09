[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$settings = Import-PowerShellDataFile (Join-Path $repo 'tools\source_environment\environment.local.psd1')
$qt = $settings.QtRoot
$testOut = Join-Path $repo 'tmp\SystemInterlockConfigTests'
New-Item -ItemType Directory -Path $testOut -Force | Out-Null
$exe = Join-Path $testOut 'SystemInterlockConfigTests.exe'
$vsDev = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$arguments = @('call', ('"{0}"' -f $vsDev), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $testOut), '&&', 'cl', '/nologo', '/EHsc', '/std:c++17', '/Zc:__cplusplus',
    '/permissive-', '/utf-8', '/DUNICODE', '/D_UNICODE', '/DNOMINMAX',
    ('/I"{0}\include"' -f $repo), ('/I"{0}"' -f $settings.EigenRoot), ('/I"{0}\include"' -f $qt),
    ('/I"{0}\include\QtCore"' -f $qt), ('/I"{0}\include\QtGui"' -f $qt),
    ('/I"{0}\include\QtWidgets"' -f $qt), ('/I"{0}\include\QtSql"' -f $qt),
    ('/I"{0}\include\QtNetwork"' -f $qt),
    ('"{0}\src\AppPaths.cpp"' -f $repo), ('"{0}\src\ConfigDatabase.cpp"' -f $repo),
    ('"{0}\src\CredentialSecurity.cpp"' -f $repo), ('"{0}\src\PointCloudProcessingConfig.cpp"' -f $repo),
    ('"{0}\src\ScanSafetyGateDialog.cpp"' -f $repo),
    ('"{0}\scripts\tests\system_interlock_config_tests.cpp"' -f $repo),
    ('/Fe:"{0}"' -f $exe), '/link', ('/LIBPATH:"{0}\lib"' -f $qt),
    'Qt6Core.lib', 'Qt6Gui.lib', 'Qt6Widgets.lib', 'Qt6Sql.lib', 'Qt6Network.lib', 'Crypt32.lib') -join ' '
& cmd.exe /d /s /c $arguments
if ($LASTEXITCODE -ne 0) { throw 'SystemInterlockConfigTests compile failed' }
$env:PATH = (Join-Path $qt 'bin') + ';' + $env:PATH
$env:QT_PLUGIN_PATH = Join-Path $qt 'plugins'
$env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path $qt 'plugins\platforms'
$env:QT_QPA_PLATFORM = 'windows'
Push-Location $testOut
try { & $exe; if ($LASTEXITCODE -ne 0) { throw 'SystemInterlockConfigTests failed' } }
finally { Pop-Location }
