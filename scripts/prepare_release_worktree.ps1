param(
    [Parameter(Mandatory = $true)][string]$TrustedSourceRoot,
    [Parameter(Mandatory = $true)][string]$TargetWorktreeRoot,
    [Parameter(Mandatory = $true)][string]$AppVersion,
    [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel,
    [Parameter(Mandatory = $true)][string]$PythonExecutable,
    [Parameter(Mandatory = $true)][string]$PythonSha256
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
. (Join-Path $scriptRoot "release_gate_common.ps1")
Set-ReleasePythonTool -PythonExecutable $PythonExecutable -PythonSha256 $PythonSha256

Assert-ReleaseVersion $AppVersion
$trustedRoot = Resolve-ReleaseGatePath $TrustedSourceRoot
$targetRoot = Resolve-ReleaseGatePath $TargetWorktreeRoot
if (Test-ReleaseGateSamePath $trustedRoot $targetRoot) {
    throw "TrustedSourceRoot and TargetWorktreeRoot must be different; preparation is for a linked release worktree."
}

$reportPath = Join-Path $targetRoot "dist\release-gates\worktree-assets-$Channel-$AppVersion.json"
$preparationRunId = New-ReleaseGatePendingReport `
    -Path $reportPath `
    -Kind "worktree-assets" `
    -AppVersion $AppVersion `
    -Channel $Channel
foreach ($staleGate in @(
    (Get-PackageGateReportPath -RepoRoot $targetRoot -AppVersion $AppVersion -Channel $Channel),
    (Get-InstallerGateReportPath -RepoRoot $targetRoot -AppVersion $AppVersion -Channel $Channel))) {
    if (Test-Path -LiteralPath $staleGate) {
        Remove-Item -LiteralPath $staleGate -Force
    }
}

$trustedState = Assert-GitReleaseState -RepoRoot $trustedRoot -AppVersion $AppVersion -Channel neutral
$targetState = Assert-GitReleaseState -RepoRoot $targetRoot -AppVersion $AppVersion -Channel $Channel
$linkedWorktrees = Assert-LinkedReleaseWorktrees `
    -NeutralRoot $trustedRoot `
    -BrandRoot $targetRoot `
    -NeutralHead $trustedState.head `
    -BrandHead $targetState.head
$trustedManifest = Assert-FanucRuntimeManifest -RepoRoot $trustedRoot -RuntimeRoot $trustedRoot
$targetManifest = Read-FanucRuntimeManifest $targetRoot
if ($trustedManifest.sha256 -cne $targetManifest.sha256) {
    throw "Trusted source and target worktree use different versioned FANUC manifests. Merge the target branch first."
}

$trustedMigrationSource = Join-Path $trustedRoot "tools\migrate_config_to_sqlite.py"
$targetMigrationSource = Join-Path $targetRoot "tools\migrate_config_to_sqlite.py"
if ((Get-ReleaseFileSha256 $trustedMigrationSource) -cne (Get-ReleaseFileSha256 $targetMigrationSource)) {
    throw "ConfigMigrate Python source differs between trusted source and target. Merge the target branch first."
}
$trustedConfigExe = Join-Path $trustedRoot "tools\ConfigMigrate.exe"
$trustedConfigBuilder = Join-Path $trustedRoot "scripts\build_config_migrate.ps1"
& $trustedConfigBuilder `
    -PythonExecutable $PythonExecutable `
    -PythonSha256 $PythonSha256 `
    -OutputPath $trustedConfigExe | Out-Null
$trustedConfig = Assert-ConfigMigrateProvenance -RepoRoot $trustedRoot -ExecutablePath $trustedConfigExe

foreach ($item in @($trustedManifest.manifest.files)) {
    $relative = ([string]$item.path).Replace('/', '\')
    $source = Join-Path $trustedRoot $relative
    $destination = Join-Path $targetRoot $relative
    $destinationParent = Split-Path -Parent $destination
    New-Item -ItemType Directory -Path $destinationParent -Force | Out-Null
    Copy-Item -LiteralPath $source -Destination $destination -Force
}

$targetToolsDir = Join-Path $targetRoot "tools"
$targetInstallerToolsDir = Join-Path $targetRoot "dist\tools"
New-Item -ItemType Directory -Path $targetToolsDir, $targetInstallerToolsDir -Force | Out-Null
$targetConfigExe = Join-Path $targetToolsDir "ConfigMigrate.exe"
$targetDistConfigExe = Join-Path $targetInstallerToolsDir "ConfigMigrate.exe"
Copy-Item -LiteralPath $trustedConfigExe -Destination $targetConfigExe -Force
Copy-Item -LiteralPath $trustedConfigExe -Destination $targetDistConfigExe -Force

$trustedRunCmd = Join-Path $trustedRoot "tools\ConfigMigrate_Run.cmd"
$targetRunCmd = Join-Path $targetRoot "tools\ConfigMigrate_Run.cmd"
$targetDistRunCmd = Join-Path $targetInstallerToolsDir "ConfigMigrate_Run.cmd"
if ((Get-ReleaseFileSha256 $trustedRunCmd) -cne (Get-ReleaseFileSha256 $targetRunCmd)) {
    throw "ConfigMigrate_Run.cmd differs between trusted source and target. Merge the target branch first."
}
Copy-Item -LiteralPath $trustedRunCmd -Destination $targetDistRunCmd -Force

$trustedInstallHelper = Join-Path $trustedRoot "tools\ConfigMigrate_Install.ps1"
$targetInstallHelper = Join-Path $targetRoot "tools\ConfigMigrate_Install.ps1"
$targetDistInstallHelper = Join-Path $targetInstallerToolsDir "ConfigMigrate_Install.ps1"
if ((Get-ReleaseFileSha256 $trustedInstallHelper) -cne (Get-ReleaseFileSha256 $targetInstallHelper)) {
    throw "ConfigMigrate_Install.ps1 differs between trusted source and target. Merge the target branch first."
}
Copy-Item -LiteralPath $trustedInstallHelper -Destination $targetDistInstallHelper -Force

$targetConfig = Assert-ConfigMigrateProvenance -RepoRoot $targetRoot -ExecutablePath $targetConfigExe
$targetDistConfig = Assert-ConfigMigrateProvenance -RepoRoot $targetRoot -ExecutablePath $targetDistConfigExe
if ($trustedConfig.sha256 -cne $targetConfig.sha256 -or $trustedConfig.sha256 -cne $targetDistConfig.sha256) {
    throw "Prepared ConfigMigrate copies do not match the trusted executable bytes."
}
if ((Get-ReleaseFileSha256 $targetRunCmd) -cne (Get-ReleaseFileSha256 $targetDistRunCmd)) {
    throw "Prepared dist/tools/ConfigMigrate_Run.cmd does not match the tracked target command."
}
if ((Get-ReleaseFileSha256 $targetInstallHelper) -cne (Get-ReleaseFileSha256 $targetDistInstallHelper)) {
    throw "Prepared dist/tools/ConfigMigrate_Install.ps1 does not match the tracked target helper."
}

$verifiedManifest = Assert-FanucRuntimeManifest -RepoRoot $targetRoot -RuntimeRoot $targetRoot
$targetStateAfter = Assert-GitReleaseState -RepoRoot $targetRoot -AppVersion $AppVersion -Channel $Channel
if ($targetStateAfter.head -cne $targetState.head) {
    throw "Target HEAD changed during worktree preparation."
}

$report = [ordered]@{
    schemaVersion        = 1
    kind                 = "worktree-assets"
    generatedAtUtc       = [DateTime]::UtcNow.ToString("o")
    status               = "pass"
    runId                = $preparationRunId
    gateScriptSha256     = Get-ReleaseFileSha256 $script:ReleaseGateCommonPath
    producerScriptSha256 = Get-ReleaseFileSha256 $PSCommandPath
    appId                = $script:ReleaseGateExpectedAppId
    version              = $AppVersion
    channel              = $Channel
    head                 = $targetState.head
    trustedSourceRoot    = $trustedRoot
    trustedSourceHead    = $trustedState.head
    targetWorktreeRoot   = $targetRoot
    gitCommonDir         = $linkedWorktrees.commonDir
    trustedGitWorktreeDir = $linkedWorktrees.neutralGitDir
    targetGitWorktreeDir = $linkedWorktrees.brandGitDir
    neutralAncestorHead  = $linkedWorktrees.neutralHead
    fanucManifestSha256  = $verifiedManifest.sha256
    fanucFileCount       = [int]$verifiedManifest.manifest.expectedFileCount
    configMigrate        = $targetDistConfig
    configMigrateRunSha256 = Get-ReleaseFileSha256 $targetDistRunCmd
}
Write-ReleaseGateJson -Value $report -Path $reportPath

Write-Host "Release worktree runtime assets are prepared and verified:"
Write-Host "  $reportPath"
