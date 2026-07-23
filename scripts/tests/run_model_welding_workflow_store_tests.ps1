[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$eigenRoot = 'E:\Eigen3.4\eigen-3.4.0'
$outputDir = Join-Path $repo 'tmp\ModelWeldingWorkflowStoreTests'
$outputExe = Join-Path $outputDir 'ModelWeldingWorkflowStoreTests.exe'
$suiteRoot = Join-Path $outputDir 'SuiteRoot'

if (-not (Test-Path -LiteralPath $vsDevCmd -PathType Leaf)) {
    throw "VsDevCmd.bat not found: $vsDevCmd"
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$outputFull = [IO.Path]::GetFullPath($outputDir).TrimEnd('\') + '\'
$suiteFull = [IO.Path]::GetFullPath($suiteRoot)
if (-not $suiteFull.StartsWith($outputFull, [StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to reset suite root outside output directory: $suiteFull"
}
if (Test-Path -LiteralPath $suiteFull) {
    Remove-Item -LiteralPath $suiteFull -Recurse -Force
}
New-Item -ItemType Directory -Path $suiteFull -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $eigenRoot),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtGui"' -f $qtInclude),
    ('/I"{0}\QtNetwork"' -f $qtInclude),
    ('/I"{0}\QtSql"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ConfigDatabase.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\CredentialSecurity.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\Const.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ModelWeldingWorkflow.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\model_welding_workflow_store_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), ('/Fo"{0}\\"' -f $outputDir),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib),
    'Qt6Core.lib', 'Qt6Gui.lib', 'Qt6Network.lib', 'Qt6Sql.lib', 'Crypt32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "ModelWeldingWorkflowStoreTests compilation failed: $LASTEXITCODE"
}
$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe $suiteFull
if ($LASTEXITCODE -ne 0) {
    throw "ModelWeldingWorkflowStoreTests failed: $LASTEXITCODE"
}
