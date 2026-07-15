[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$vsDevCmd = 'C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat'
$qtRoot = 'E:\workspace\soft\QT\6.7.3\msvc2022_64'
$outputDir = Join-Path $repo 'tmp\CredentialSecurityTests'
$outputExe = Join-Path $outputDir 'CredentialSecurityTests.exe'

if (-not (Test-Path -LiteralPath $vsDevCmd -PathType Leaf)) {
    throw "VsDevCmd.bat not found: $vsDevCmd"
}
if (-not (Test-Path -LiteralPath $qtRoot -PathType Container)) {
    throw "Qt runtime not found: $qtRoot"
}
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null

$source = Join-Path $repo 'src\CredentialSecurity.cpp'
$test = Join-Path $repo 'scripts\tests\credential_security_tests.cpp'
$include = Join-Path $repo 'include'
$qtInclude = Join-Path $qtRoot 'include'
$qtLib = Join-Path $qtRoot 'lib'
$compile = @(
    'call', ('"{0}"' -f $vsDevCmd), '-arch=x64', '-host_arch=x64', '1>nul', '2>nul', '&&',
    'cl', '/nologo', '/EHsc', '/std:c++17', '/Zc:__cplusplus', '/permissive-', '/utf-8', '/DUNICODE', '/D_UNICODE', '/DNOMINMAX',
    ('/I"{0}"' -f $include), ('/I"{0}"' -f $qtInclude), ('/I"{0}\QtCore"' -f $qtInclude),
    ('/I"{0}\QtNetwork"' -f $qtInclude), ('/I"{0}\QtSql"' -f $qtInclude),
    ('"{0}"' -f (Join-Path $repo 'src\AppPaths.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ConfigDatabase.cpp')),
    ('"{0}"' -f (Join-Path $repo 'src\ConfigDatabaseAuthentication.cpp')),
    ('"{0}"' -f $source), ('"{0}"' -f $test),
    ('/Fe:"{0}"' -f $outputExe), ('/Fo:"{0}\\"' -f $outputDir),
    '/link', ('/LIBPATH:"{0}"' -f $qtLib), 'Qt6Core.lib', 'Qt6Network.lib', 'Qt6Sql.lib', 'Crypt32.lib'
) -join ' '

& cmd.exe /d /s /c $compile
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurityTests compilation failed: $LASTEXITCODE"
}
$env:PATH = (Join-Path $qtRoot 'bin') + ';' + $env:PATH
& $outputExe
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurityTests failed: $LASTEXITCODE"
}
& $outputExe --future-schema
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity future-schema test failed: $LASTEXITCODE"
}
& $outputExe --corrupt-dpapi
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity corrupt-DPAPI migration test failed: $LASTEXITCODE"
}
& $outputExe --auth-initialization
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity authentication-initialization test failed: $LASTEXITCODE"
}
& $outputExe --empty-v4-recovery
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity empty-v4 recovery test failed: $LASTEXITCODE"
}
& $outputExe --invalid-auth-metadata
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity invalid-auth-metadata test failed: $LASTEXITCODE"
}
foreach ($scenario in @('missing', 'invalid', 'future')) {
    & $outputExe --auth-semantic-gate $scenario
    if ($LASTEXITCODE -ne 0) {
        throw "CredentialSecurity auth-semantic-gate '$scenario' test failed: $LASTEXITCODE"
    }
}
& $outputExe --legacy-disk-gate
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity legacy-disk-gate test failed: $LASTEXITCODE"
}
& $outputExe --pending-scrub-gate
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity pending-scrub-gate test failed: $LASTEXITCODE"
}
& $outputExe --plaintext-backup-gate
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity plaintext-backup-gate test failed: $LASTEXITCODE"
}
& $outputExe --legacy-timestamp-merge
if ($LASTEXITCODE -ne 0) {
    throw "CredentialSecurity legacy timestamp merge test failed: $LASTEXITCODE"
}
foreach ($scenario in @(
    'valid', 'module-case-shadow', 'no-admin', 'legacy-orphan', 'bad-dpapi',
    'portable-dpapi-prefix', 'portable-legacy-prefix',
    'portable-created-at-encrypted', 'bad-password-changed-at',
    'manifest-without-state', 'bad-complete-manifest', 'complete-scrub',
    'legacy-disabled-login-preferences', 'plaintext-enabled-login-preference',
    'plaintext-autologin-sensitive-zero', 'released-login-preferences',
    'protected-remembered-credential',
    'remembered-credential-invalid-unsafe-key',
    'remembered-credential-invalid-empty',
    'remembered-credential-invalid-type',
    'remembered-credential-invalid-sensitive',
    'remembered-credential-invalid-encrypted',
    'empty-created-at', 'table-name-case', 'legacy-table-case'
)) {
    & $outputExe --current-auth-integrity $scenario
    if ($LASTEXITCODE -ne 0) {
        throw "CredentialSecurity current authentication integrity '$scenario' test failed: $LASTEXITCODE"
    }
}
foreach ($scenario in @(
    'success', 'module-case', 'field-case', 'source-conflict',
    'target-conflict', 'current-settings-orphan'
)) {
    & $outputExe --legacy-login-semantic $scenario
    if ($LASTEXITCODE -ne 0) {
        throw "CredentialSecurity legacy login semantic '$scenario' test failed: $LASTEXITCODE"
    }
}
foreach ($scenario in @(
    'valid', 'arbitrary', 'directory', 'symlink', 'dangling-symlink',
    'existing-connection', 'existing-connection-pending',
    'existing-connection-corrupt', 'noop-absent'
)) {
    & $outputExe --installer-transaction-gate $scenario
    if ($LASTEXITCODE -ne 0) {
        throw "CredentialSecurity installer transaction gate '$scenario' test failed: $LASTEXITCODE"
    }
}
foreach ($scenario in @(
    'source-conflict', 'target-conflict', 'target-protected-portable',
    'target-malformed-password-changed-at',
    'no-admin', 'user-case-conflict',
    'field-case-conflict', 'login-module-case-conflict', 'login-field-case-conflict'
)) {
    & $outputExe --legacy-rollback $scenario
    if ($LASTEXITCODE -ne 0) {
        throw "CredentialSecurity legacy rollback '$scenario' test failed: $LASTEXITCODE"
    }
}
foreach ($scenario in @('unknown-key', 'section', 'nested', 'credential-key', 'symlink')) {
    & $outputExe --rejected-runtime-ini $scenario
    if ($LASTEXITCODE -ne 0) {
        throw "CredentialSecurity rejected runtime INI '$scenario' test failed: $LASTEXITCODE"
    }
}

$interopRoot = Join-Path $outputDir 'PythonInteropRoot'
$resolvedOutput = [System.IO.Path]::GetFullPath($outputDir).TrimEnd('\') + '\'
$resolvedInterop = [System.IO.Path]::GetFullPath($interopRoot)
if (-not $resolvedInterop.StartsWith($resolvedOutput, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to replace test directory outside output root: $resolvedInterop"
}
if (Test-Path -LiteralPath $resolvedInterop) {
    Remove-Item -LiteralPath $resolvedInterop -Recurse -Force
}
$legacyData = Join-Path $resolvedInterop 'LegacyData'
$targetData = Join-Path $resolvedInterop 'RuntimeRoot\Data'
New-Item -ItemType Directory -Path $legacyData -Force | Out-Null
New-Item -ItemType Directory -Path $targetData -Force | Out-Null
$legacyPassword = 'Synthetic-Python-Account-Only'
$legacyHashBytes = [System.Text.Encoding]::UTF8.GetBytes("python_legacy_user`n$legacyPassword")
$sha256 = [System.Security.Cryptography.SHA256]::Create()
try {
    $legacyHash = ([BitConverter]::ToString($sha256.ComputeHash($legacyHashBytes))).Replace('-', '').ToLowerInvariant()
} finally {
    $sha256.Dispose()
}
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText((Join-Path $legacyData 'Accounts.ini'), @"
[Users/python_legacy_user]
PasswordHash=$legacyHash
Role=admin
CreatedAt=2026-01-02T03:04:05
"@, $utf8NoBom)
[System.IO.File]::WriteAllText((Join-Path $legacyData 'OnlineServices.ini'), @"
FtpPassword=Synthetic-Python-DPAPI-Interop-Only
"@, $utf8NoBom)
& python (Join-Path $repo 'tools\migrate_config_to_sqlite.py') `
    --source $legacyData --db (Join-Path $targetData 'ConfigStore.db') --encrypt
if ($LASTEXITCODE -ne 0) {
    throw "Python ConfigMigrate interop fixture failed: $LASTEXITCODE"
}
& $outputExe --python-interop-root (Join-Path $resolvedInterop 'RuntimeRoot')
if ($LASTEXITCODE -ne 0) {
    throw "Python-to-C++ DPAPI interop test failed: $LASTEXITCODE"
}

& python (Join-Path $repo 'scripts\tests\test_config_migrate_schema4_upgrade.py')
if ($LASTEXITCODE -ne 0) {
    throw "ConfigMigrate schema4 upgrade regression failed: $LASTEXITCODE"
}
