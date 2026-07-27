[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$installerPath = Join-Path $repoRoot 'installer\QtWidgetsApplication4.iss'
$installer = Get-Content -LiteralPath $installerPath -Raw -Encoding UTF8

function Assert-True {
    param($Condition, [string]$Message)
    if (-not [bool]$Condition) {
        throw $Message
    }
}

Assert-True ($installer.Contains('Excludes: "Data\*,SDK\STEP\versions\*"')) `
    'installer must preserve the existing Data directory'

foreach ($forbidden in @(
    'ConfigMigrate_PreInstall',
    'PrepareToInstall',
    'DatabaseMigration',
    'MigrateExecutable',
    'MigrationStatusPath',
    'CommitPendingDatabaseTransaction',
    'RollbackPendingDatabaseTransaction',
    'FinalizeDeferredCredentialScrub',
    'NoTeachingRobotRecovery',
    'SameVersionRecovery',
    'GetCustomSetupExitCode',
    'DeinitializeSetup',
    'Check: DatabaseMigrationSucceeded'
)) {
    Assert-True (-not $installer.Contains($forbidden)) `
        "installer must not migrate or repair databases during setup: $forbidden"
}

foreach ($repairTool in @(
    'Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion',
    'Source: "..\dist\tools\ConfigMigrate_Run.cmd"; DestDir: "{app}\tools"; Flags: ignoreversion',
    'Source: "..\dist\tools\ConfigMigrate_Install.ps1"; DestDir: "{app}\tools"; Flags: ignoreversion'
)) {
    Assert-True ($installer.Contains($repairTool)) `
        "application-controlled repair tool is missing: $repairTool"
}

Assert-True ($installer.Contains(
    'Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent'
)) 'application launch must not depend on installer database state'

Write-Host 'PASS: installer preserves Data and defers database repair to the application.'
