param(
    [Parameter(Mandatory = $true)][string]$NeutralInstallerGateReport,
    [Parameter(Mandatory = $true)][string]$BrandInstallerGateReport,
    [Parameter(Mandatory = $true)][string]$OutputPath
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
. (Join-Path $scriptRoot "release_gate_common.ps1")
$MaxInstallerSizeDifferenceBytes = [int64](2MB)

function Get-ComparablePackageInventory {
    param(
        [Parameter(Mandatory = $true)]$Inventory,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )

    $spec = Get-ReleaseChannelSpec $Channel
    $result = @()
    foreach ($item in @($Inventory)) {
        $path = ([string]$item.path).Replace('\', '/')
        $lower = $path.ToLowerInvariant()
        $allowedDifference = (
            $path -ceq $spec.ExeName `
            -or $lower.StartsWith("branding/") `
            -or $lower.StartsWith("icons/") `
            -or $lower -eq "build_version.txt" `
            -or $lower -eq "deploy_notes.txt")
        if (-not $allowedDifference) {
            $result += $item
        }
    }
    return @($result | Sort-Object path)
}

# Validate every input and live artifact before creating/replacing any output.
$neutralGatePath = Resolve-ReleaseGatePath $NeutralInstallerGateReport
$brandGatePath = Resolve-ReleaseGatePath $BrandInstallerGateReport
if (Test-ReleaseGateSamePath $neutralGatePath $brandGatePath) {
    throw "Neutral and brand installer gate inputs must be different files."
}
$neutral = Assert-InstallerGateReport $neutralGatePath
$brand = Assert-InstallerGateReport $brandGatePath

if ([string]$neutral.channel -cne "neutral" -or [string]$brand.channel -cne "brand") {
    throw "Release pair must contain exactly one neutral gate and one brand gate."
}
if ([string]$neutral.version -cne [string]$brand.version) {
    throw "Neutral and brand versions differ: $($neutral.version) != $($brand.version)"
}
if ([string]$neutral.appId -cne $script:ReleaseGateExpectedAppId `
    -or [string]$brand.appId -cne $script:ReleaseGateExpectedAppId) {
    throw "Both release channels must retain immutable AppId $($script:ReleaseGateExpectedAppId)."
}

$linkedWorktrees = Assert-LinkedReleaseWorktrees `
    -NeutralRoot ([string]$neutral.repoRoot) `
    -BrandRoot ([string]$brand.repoRoot) `
    -NeutralHead ([string]$neutral.head) `
    -BrandHead ([string]$brand.head)

$neutralPackage = Read-ReleaseGateJson ([string]$neutral.packageGateReport)
$brandPackage = Read-ReleaseGateJson ([string]$brand.packageGateReport)
if (-not (Test-ReleaseGateSamePath ([string]$neutralPackage.gitCommonDir) $linkedWorktrees.commonDir) `
    -or -not (Test-ReleaseGateSamePath ([string]$brandPackage.gitCommonDir) $linkedWorktrees.commonDir)) {
    throw "Package gates are not bound to the shared linked-worktree Git common directory."
}
if ([string]$neutralPackage.fanucManifest.sha256 -cne [string]$brandPackage.fanucManifest.sha256 `
    -or [int]$neutralPackage.fanucManifest.fileCount -ne [int]$brandPackage.fanucManifest.fileCount) {
    throw "Neutral and brand FANUC runtime manifests differ."
}
if ([string]$neutralPackage.configMigrate.sha256 -cne [string]$brandPackage.configMigrate.sha256 `
    -or [string]$neutralPackage.configMigrate.sourceSha256 -cne [string]$brandPackage.configMigrate.sourceSha256 `
    -or [string]$neutralPackage.configMigrate.builderSha256 -cne [string]$brandPackage.configMigrate.builderSha256 `
    -or [string]$neutralPackage.configMigrate.isolatedRebuildSha256 -cne [string]$brandPackage.configMigrate.isolatedRebuildSha256) {
    throw "Neutral and brand ConfigMigrate bytes, source, or builder provenance differ."
}
if ([string]$neutralPackage.configMigrateRun.sha256 -cne [string]$brandPackage.configMigrateRun.sha256) {
    throw "Neutral and brand ConfigMigrate_Run.cmd bytes differ."
}

$neutralComparable = @(Get-ComparablePackageInventory -Inventory @($neutralPackage.packageInventory) -Channel neutral)
$brandComparable = @(Get-ComparablePackageInventory -Inventory @($brandPackage.packageInventory) -Channel brand)
Assert-ReleaseInventoryMatches -Expected $neutralComparable -Actual $brandComparable -Context "neutral/brand common package inventory"

$sizeDifference = [Math]::Abs([int64]$neutral.installer.size - [int64]$brand.installer.size)
if ($sizeDifference -gt $MaxInstallerSizeDifferenceBytes) {
    throw "Neutral/brand installer size difference $sizeDifference bytes exceeds gate $MaxInstallerSizeDifferenceBytes bytes."
}

$canonicalOutput = Get-ReleasePairGateReportPath -RepoRoot ([string]$neutral.repoRoot) -AppVersion ([string]$neutral.version)
$protectedPaths = @(
    $neutralGatePath,
    $brandGatePath,
    [string]$neutral.packageGateReport,
    [string]$brand.packageGateReport,
    [string]$neutral.installer.path,
    [string]$brand.installer.path,
    [string]$neutralPackage.configMigrate.path,
    [string]$brandPackage.configMigrate.path,
    [string]$neutralPackage.configMigrateRun.path,
    [string]$brandPackage.configMigrateRun.path
)
foreach ($item in @($neutralPackage.packageInventory)) {
    $protectedPaths += Join-Path ([string]$neutralPackage.packageDir) (([string]$item.path).Replace('/', '\'))
}
foreach ($item in @($brandPackage.packageInventory)) {
    $protectedPaths += Join-Path ([string]$brandPackage.packageDir) (([string]$item.path).Replace('/', '\'))
}
$validatedOutput = Assert-ReleasePairOutputPath `
    -CandidatePath $OutputPath `
    -CanonicalPath $canonicalOutput `
    -ProtectedPaths $protectedPaths

$pairRunId = [Guid]::NewGuid().ToString("D")
$pending = [ordered]@{
    schemaVersion        = $script:ReleaseGateSchemaVersion
    kind                 = "release-pair"
    status               = "pending"
    runId                = $pairRunId
    startedAtUtc         = [DateTime]::UtcNow.ToString("o")
    gateScriptSha256     = Get-ReleaseFileSha256 $script:ReleaseGateCommonPath
    producerScriptSha256 = Get-ReleaseFileSha256 $PSCommandPath
    version              = [string]$neutral.version
}
Write-ReleaseGateJson -Value $pending -Path $validatedOutput

$pairReport = [ordered]@{
    schemaVersion                    = $script:ReleaseGateSchemaVersion
    kind                             = "release-pair"
    status                           = "pass"
    runId                            = $pairRunId
    generatedAtUtc                   = [DateTime]::UtcNow.ToString("o")
    gateScriptSha256                 = Get-ReleaseFileSha256 $script:ReleaseGateCommonPath
    producerScriptSha256             = Get-ReleaseFileSha256 $PSCommandPath
    appId                            = $script:ReleaseGateExpectedAppId
    version                          = [string]$neutral.version
    gitCommonDir                     = $linkedWorktrees.commonDir
    neutralAncestorHead              = $linkedWorktrees.neutralHead
    brandDescendantHead              = $linkedWorktrees.brandHead
    maxInstallerSizeDifferenceBytes  = [int64]$MaxInstallerSizeDifferenceBytes
    installerSizeDifferenceBytes     = [int64]$sizeDifference
    commonInventoryFileCount         = [int]$neutralComparable.Count
    fanucManifestSha256              = [string]$neutralPackage.fanucManifest.sha256
    configMigrateSha256              = [string]$neutralPackage.configMigrate.sha256
    neutral                          = [ordered]@{
        head                 = [string]$neutral.head
        repoRoot             = [string]$neutral.repoRoot
        gitWorktreeDir       = $linkedWorktrees.neutralGitDir
        installerGateReport = $neutralGatePath
        installerGateSha256 = Get-ReleaseFileSha256 $neutralGatePath
        installerSha256     = [string]$neutral.installer.sha256
        installerSize       = [int64]$neutral.installer.size
    }
    brand                            = [ordered]@{
        head                 = [string]$brand.head
        repoRoot             = [string]$brand.repoRoot
        gitWorktreeDir       = $linkedWorktrees.brandGitDir
        installerGateReport = $brandGatePath
        installerGateSha256 = Get-ReleaseFileSha256 $brandGatePath
        installerSha256     = [string]$brand.installer.sha256
        installerSize       = [int64]$brand.installer.size
    }
}
Write-ReleaseGateJson -Value $pairReport -Path $validatedOutput

Write-Host "PASS: neutral/brand release pair is locally verified."
Write-Host "  $validatedOutput"
