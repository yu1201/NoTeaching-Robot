[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$outputDir = Join-Path $repo 'tmp\WeldSafetyRecoveryStoreTests'
$outputExe = Join-Path $outputDir 'WeldSafetyRecoveryStoreTests.exe'

if (-not (Test-Path -LiteralPath $vsDevCmd -PathType Leaf)) {
    throw "VsDevCmd.bat not found: $vsDevCmd"
}
if (-not (Test-Path -LiteralPath $qtRoot -PathType Container)) {
    throw "Qt runtime not found: $qtRoot"
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

function Reset-TestRoot([string]$path) {
    $outputFull = [IO.Path]::GetFullPath($outputDir).TrimEnd('\') + '\'
    $targetFull = [IO.Path]::GetFullPath($path)
    if (-not $targetFull.StartsWith($outputFull, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove test root outside output directory: $targetFull"
    }
    if (Test-Path -LiteralPath $targetFull) {
        Remove-Item -LiteralPath $targetFull -Recurse -Force
    }
    New-Item -ItemType Directory -Path $targetFull -Force | Out-Null
}

$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cd', '/d', ('"{0}"' -f $outputDir), '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8',
    '/DUNICODE', '/D_UNICODE', '/DNOMINMAX', '/DWELD_SAFETY_STORE_STORAGE_ONLY_TEST',
    '/DCONFIG_DATABASE_TEST_INJECT_CURSOR_ERROR',
    ('/I"{0}"' -f (Join-Path $repo 'include')),
    ('/I"{0}"' -f $qtInclude),
    ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtNetwork"' -f $qtInclude),
    ('/I"{0}\QtSql"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ConfigDatabase.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\CredentialSecurity.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\WeldResumePlanner.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\WeldSafetyRecoveryStore.cpp')),
    ('"{0}"' -f (Join-Path $repo 'scripts\tests\weld_safety_recovery_store_tests.cpp')),
    ('/Fe:"{0}"' -f $outputExe), ('/Fo"{0}\\"' -f $outputDir),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib),
    'Qt6Core.lib', 'Qt6Network.lib', 'Qt6Sql.lib', 'Crypt32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "WeldSafetyRecoveryStoreTests compilation failed: $LASTEXITCODE"
}

$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
$suiteRoot = Join-Path $outputDir 'SuiteRoot'
Reset-TestRoot $suiteRoot
& $outputExe 'suite' $suiteRoot
if ($LASTEXITCODE -ne 0) {
    throw "WeldSafetyRecoveryStore ConfigStore suite failed: $LASTEXITCODE"
}

$failureRoot = Join-Path $outputDir 'FailureRoot'
Reset-TestRoot $failureRoot
& $outputExe 'seed-read-failure' $failureRoot
if ($LASTEXITCODE -ne 0) {
    throw "Could not seed ConfigStore read-failure fixture: $LASTEXITCODE"
}
$databasePath = Join-Path $failureRoot 'Data\ConfigStore.db'
if (-not (Test-Path -LiteralPath $databasePath -PathType Leaf)) {
    throw "ConfigStore fixture database was not created: $databasePath"
}
foreach ($suffix in @('-wal', '-shm')) {
    $sidecar = $databasePath + $suffix
    if (Test-Path -LiteralPath $sidecar) {
        Remove-Item -LiteralPath $sidecar -Force
    }
}
[IO.File]::WriteAllBytes($databasePath, [Text.Encoding]::ASCII.GetBytes('not-a-sqlite-database'))
& $outputExe 'expect-read-failure' $failureRoot
if ($LASTEXITCODE -ne 0) {
    throw "ConfigStore read failure did not fail closed: $LASTEXITCODE"
}

Write-Host 'PASS: WeldSafetyRecoveryStore dynamic storage tests'
