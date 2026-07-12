[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$outputDir = Join-Path $repo 'tmp\WeldResumePlannerTests'
$outputExe = Join-Path $outputDir 'WeldResumePlannerTests.exe'

if (-not (Test-Path -LiteralPath $vsDevCmd -PathType Leaf)) {
    throw "VsDevCmd.bat not found: $vsDevCmd"
}
if (-not (Test-Path -LiteralPath $qtRoot -PathType Container)) {
    throw "Qt runtime not found: $qtRoot"
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8', '/DUNICODE', '/D_UNICODE', '/DNOMINMAX',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude), ('/I"{0}\QtCore"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\WeldResumePlanner.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\weld_resume_planner_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), ('/Fo{0}\' -f $outputDir),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), 'Qt6Core.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "WeldResumePlannerTests compilation failed: $LASTEXITCODE"
}
$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe
if ($LASTEXITCODE -ne 0) {
    throw "WeldResumePlannerTests failed: $LASTEXITCODE"
}
