[CmdletBinding(DefaultParameterSetName = "Create")]
param(
    [Parameter(Mandatory = $true, ParameterSetName = "Create")][string]$NeutralInstallerGateReport,
    [Parameter(Mandatory = $true, ParameterSetName = "Create")][string]$BrandInstallerGateReport,
    [Parameter(Mandatory = $true, ParameterSetName = "Create")][string]$OutputPath,

    [Parameter(Mandatory = $true, ParameterSetName = "Attest")][string]$AttestExistingPair,
    [Parameter(Mandatory = $true, ParameterSetName = "Attest")]
    [ValidatePattern('^[0-9a-fA-F]{64}$')][string]$PublishChallenge,
    [Parameter(Mandatory = $true, ParameterSetName = "Attest")]
    [ValidatePattern('^[0-9a-fA-F]{64}$')][string]$ExpectedCandidateSha256,
    [Parameter(Mandatory = $true, ParameterSetName = "Attest")]
    [ValidatePattern('^[0-9a-fA-F]{64}$')][string]$ExpectedPairGateSha256,
    [Parameter(Mandatory = $true, ParameterSetName = "Attest")][string]$AttestationOutputPath,

    [Parameter(Mandatory = $true)][string]$PythonExecutable,
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{64}$')][string]$PythonSha256
)

$ErrorActionPreference = "Stop"
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
. (Join-Path $scriptRoot "release_gate_common.ps1")
Set-ReleasePythonTool -PythonExecutable $PythonExecutable -PythonSha256 $PythonSha256
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

function Get-ReleasePairLiveState {
    param(
        [Parameter(Mandatory = $true)][string]$NeutralGateReport,
        [Parameter(Mandatory = $true)][string]$BrandGateReport
    )

    # Every report and live artifact is re-read by the current trusted gate code.
    $neutralGatePath = Resolve-ReleaseGatePath $NeutralGateReport
    $brandGatePath = Resolve-ReleaseGatePath $BrandGateReport
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

    $neutralPackagePath = Resolve-ReleaseGatePath ([string]$neutral.packageGateReport)
    $brandPackagePath = Resolve-ReleaseGatePath ([string]$brand.packageGateReport)
    $neutralPackage = Read-ReleaseGateJson $neutralPackagePath
    $brandPackage = Read-ReleaseGateJson $brandPackagePath
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

    return [pscustomobject]@{
        neutral               = $neutral
        brand                 = $brand
        neutralGatePath       = $neutralGatePath
        brandGatePath         = $brandGatePath
        neutralPackage        = $neutralPackage
        brandPackage          = $brandPackage
        neutralPackagePath    = $neutralPackagePath
        brandPackagePath      = $brandPackagePath
        linkedWorktrees       = $linkedWorktrees
        neutralComparable     = @($neutralComparable)
        brandComparable       = @($brandComparable)
        installerSizeDifference = [int64]$sizeDifference
    }
}

function Assert-ExactReleaseGateProperties {
    param(
        [Parameter(Mandatory = $true)]$Value,
        [Parameter(Mandatory = $true)][string[]]$Expected,
        [Parameter(Mandatory = $true)][string]$Context
    )

    $actual = @($Value.PSObject.Properties.Name | Sort-Object)
    $wanted = @($Expected | Sort-Object)
    if ($actual.Count -ne $wanted.Count) {
        throw "$Context property set differs from the supported schema."
    }
    for ($i = 0; $i -lt $wanted.Count; ++$i) {
        if ([string]$actual[$i] -cne [string]$wanted[$i]) {
            throw "$Context property set differs from the supported schema."
        }
    }
}

function Assert-ExistingReleasePairMatchesLiveState {
    param(
        [Parameter(Mandatory = $true)][string]$PairPath,
        [Parameter(Mandatory = $true)]$Pair,
        [Parameter(Mandatory = $true)]$Live,
        [Parameter(Mandatory = $true)][string]$ExpectedSha256
    )

    Assert-ExactReleaseGateProperties -Value $Pair -Context "release-pair gate" -Expected @(
        "schemaVersion", "kind", "status", "runId", "generatedAtUtc",
        "gateScriptSha256", "producerScriptSha256", "appId", "version", "gitCommonDir",
        "neutralAncestorHead", "brandDescendantHead", "maxInstallerSizeDifferenceBytes",
        "installerSizeDifferenceBytes", "commonInventoryFileCount", "fanucManifestSha256",
        "configMigrateSha256", "neutral", "brand")
    if ([int]$Pair.schemaVersion -ne $script:ReleaseGateSchemaVersion `
        -or [string]$Pair.kind -cne "release-pair" `
        -or [string]$Pair.status -cne "pass" `
        -or [string]::IsNullOrWhiteSpace([string]$Pair.runId)) {
        throw "Existing release-pair gate is not a supported PASS report: $PairPath"
    }
    $ignoredRunId = [Guid]::Empty
    if (-not [Guid]::TryParse([string]$Pair.runId, [ref]$ignoredRunId)) {
        throw "Existing release-pair gate runId is not a GUID."
    }
    $ignoredGeneratedAt = [DateTimeOffset]::MinValue
    if (-not [DateTimeOffset]::TryParse(
        [string]$Pair.generatedAtUtc,
        [Globalization.CultureInfo]::InvariantCulture,
        [Globalization.DateTimeStyles]::RoundtripKind,
        [ref]$ignoredGeneratedAt)) {
        throw "Existing release-pair gate generatedAtUtc is invalid."
    }

    $actualPairSha = Get-ReleaseFileSha256 $PairPath
    if ($actualPairSha -cne $ExpectedSha256.ToLowerInvariant()) {
        throw "Existing release-pair bytes do not match the candidate-bound pair hash."
    }
    $canonicalPairPath = Get-ReleasePairGateReportPath `
        -RepoRoot ([string]$Live.neutral.repoRoot) `
        -AppVersion ([string]$Live.neutral.version)
    if (-not (Test-ReleaseGateSamePath $PairPath $canonicalPairPath)) {
        throw "Existing release-pair gate path is not canonical."
    }
    if ([string]$Pair.gateScriptSha256 -cne (Get-ReleaseFileSha256 $script:ReleaseGateCommonPath) `
        -or [string]$Pair.producerScriptSha256 -cne (Get-ReleaseFileSha256 $PSCommandPath)) {
        throw "Existing release-pair gate was not produced for the current trusted verifier code."
    }
    if ([string]$Pair.appId -cne $script:ReleaseGateExpectedAppId `
        -or [string]$Pair.version -cne [string]$Live.neutral.version `
        -or -not (Test-ReleaseGateSamePath ([string]$Pair.gitCommonDir) ([string]$Live.linkedWorktrees.commonDir)) `
        -or [string]$Pair.neutralAncestorHead -cne [string]$Live.linkedWorktrees.neutralHead `
        -or [string]$Pair.brandDescendantHead -cne [string]$Live.linkedWorktrees.brandHead) {
        throw "Existing release-pair identity, AppId, or linked-worktree ancestry is stale."
    }
    if ([int64]$Pair.maxInstallerSizeDifferenceBytes -ne $MaxInstallerSizeDifferenceBytes `
        -or [int64]$Pair.installerSizeDifferenceBytes -ne [int64]$Live.installerSizeDifference `
        -or [int]$Pair.commonInventoryFileCount -ne [int]$Live.neutralComparable.Count `
        -or [string]$Pair.fanucManifestSha256 -cne [string]$Live.neutralPackage.fanucManifest.sha256 `
        -or [string]$Pair.configMigrateSha256 -cne [string]$Live.neutralPackage.configMigrate.sha256) {
        throw "Existing release-pair cross-channel evidence is stale."
    }

    foreach ($channel in @("neutral", "brand")) {
        $node = $Pair.PSObject.Properties[$channel].Value
        $gate = $Live.PSObject.Properties[$channel].Value
        $gatePath = [string]$Live.PSObject.Properties[($channel + "GatePath")].Value
        Assert-ExactReleaseGateProperties -Value $node -Context "release-pair $channel" -Expected @(
            "head", "repoRoot", "gitWorktreeDir", "installerGateReport",
            "installerGateSha256", "installerSha256", "installerSize")
        if ([string]$node.head -cne [string]$gate.head `
            -or -not (Test-ReleaseGateSamePath ([string]$node.repoRoot) ([string]$gate.repoRoot)) `
            -or -not (Test-ReleaseGateSamePath ([string]$node.gitWorktreeDir) ([string]$gate.gitWorktreeDir)) `
            -or -not (Test-ReleaseGateSamePath ([string]$node.installerGateReport) $gatePath) `
            -or [string]$node.installerGateSha256 -cne (Get-ReleaseFileSha256 $gatePath) `
            -or [string]$node.installerSha256 -cne [string]$gate.installer.sha256 `
            -or [int64]$node.installerSize -ne [int64]$gate.installer.size) {
            throw "Existing release-pair $channel evidence is stale."
        }
    }
}

function Resolve-PublishAttestationOutputPath {
    param(
        [Parameter(Mandatory = $true)][string]$CandidatePath,
        [Parameter(Mandatory = $true)][string[]]$ForbiddenRepoRoots
    )

    $candidate = [System.IO.Path]::GetFullPath($CandidatePath)
    if ([System.IO.Path]::GetExtension($candidate) -cne ".json") {
        throw "Publish attestation output must be a JSON file."
    }
    if (Test-Path -LiteralPath $candidate) {
        throw "Publish attestation output must be a fresh non-existing path."
    }
    $parent = Split-Path -Parent $candidate
    if ([string]::IsNullOrWhiteSpace($parent) -or -not (Test-Path -LiteralPath $parent -PathType Container)) {
        throw "Publish attestation output parent does not exist."
    }
    $parentItem = Get-Item -LiteralPath $parent -Force
    if (($parentItem.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "Publish attestation output parent must not be a reparse point."
    }
    foreach ($repoRoot in $ForbiddenRepoRoots) {
        $root = Resolve-ReleaseGatePath $repoRoot
        $prefix = $root + [System.IO.Path]::DirectorySeparatorChar
        if ($candidate.Equals($root, [System.StringComparison]::OrdinalIgnoreCase) `
            -or $candidate.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Publish attestation is ephemeral evidence and must be written outside release repositories."
        }
    }
    return $candidate
}

if ($PSCmdlet.ParameterSetName -eq "Attest") {
    $pairPath = Resolve-ReleaseGatePath $AttestExistingPair
    if (-not (Test-Path -LiteralPath $pairPath -PathType Leaf)) {
        throw "Existing release-pair gate does not exist: $pairPath"
    }
    $expectedPairSha = $ExpectedPairGateSha256.ToLowerInvariant()
    if ((Get-ReleaseFileSha256 $pairPath) -cne $expectedPairSha) {
        throw "Existing release-pair changed before trusted publish verification started."
    }
    $pair = Read-ReleaseGateJson $pairPath
    $neutralGatePath = [string]$pair.neutral.installerGateReport
    $brandGatePath = [string]$pair.brand.installerGateReport
    if ([string]::IsNullOrWhiteSpace($neutralGatePath) -or [string]::IsNullOrWhiteSpace($brandGatePath)) {
        throw "Existing release-pair does not reference both installer gate reports."
    }

    $live = Get-ReleasePairLiveState -NeutralGateReport $neutralGatePath -BrandGateReport $brandGatePath
    Assert-ExistingReleasePairMatchesLiveState `
        -PairPath $pairPath `
        -Pair $pair `
        -Live $live `
        -ExpectedSha256 $expectedPairSha

    # Re-read the pair after all expensive live checks so a concurrent report swap cannot be attested.
    if ((Get-ReleaseFileSha256 $pairPath) -cne $expectedPairSha) {
        throw "Existing release-pair changed during trusted publish verification."
    }
    $attestationPath = Resolve-PublishAttestationOutputPath `
        -CandidatePath $AttestationOutputPath `
        -ForbiddenRepoRoots @([string]$live.neutral.repoRoot, [string]$live.brand.repoRoot)
    $challenge = $PublishChallenge.ToLowerInvariant()
    $candidateSha = $ExpectedCandidateSha256.ToLowerInvariant()
    $attestation = [ordered]@{
        schemaVersion        = $script:ReleaseGateSchemaVersion
        kind                 = "publish-attestation"
        status               = "pass"
        challenge            = $challenge
        candidateSha256      = $candidateSha
        generatedAtUtc       = [DateTime]::UtcNow.ToString("o")
        verifierProcessId    = [int]$PID
        verifierScriptSha256 = Get-ReleaseFileSha256 $PSCommandPath
        gateScriptSha256     = Get-ReleaseFileSha256 $script:ReleaseGateCommonPath
        appId                = $script:ReleaseGateExpectedAppId
        version              = [string]$live.neutral.version
        pairGate             = [ordered]@{
            path   = $pairPath
            sha256 = $expectedPairSha
            runId  = [string]$pair.runId
        }
        neutral              = [ordered]@{
            head                    = [string]$live.neutral.head
            installerGateReport     = [string]$live.neutralGatePath
            installerGateSha256     = Get-ReleaseFileSha256 ([string]$live.neutralGatePath)
            packageGateReport       = [string]$live.neutralPackagePath
            packageGateSha256       = Get-ReleaseFileSha256 ([string]$live.neutralPackagePath)
            installerSha256         = [string]$live.neutral.installer.sha256
            installerSize           = [int64]$live.neutral.installer.size
        }
        brand                = [ordered]@{
            head                    = [string]$live.brand.head
            installerGateReport     = [string]$live.brandGatePath
            installerGateSha256     = Get-ReleaseFileSha256 ([string]$live.brandGatePath)
            packageGateReport       = [string]$live.brandPackagePath
            packageGateSha256       = Get-ReleaseFileSha256 ([string]$live.brandPackagePath)
            installerSha256         = [string]$live.brand.installer.sha256
            installerSize           = [int64]$live.brand.installer.size
        }
    }
    Write-ReleaseGateJson -Value $attestation -Path $attestationPath
    Write-Host "PASS: current trusted release verifier freshly revalidated the publish candidate."
    Write-Host "  $attestationPath"
    return
}

# Create mode: validate every input and live artifact before creating/replacing any output.
$live = Get-ReleasePairLiveState `
    -NeutralGateReport $NeutralInstallerGateReport `
    -BrandGateReport $BrandInstallerGateReport
$neutral = $live.neutral
$brand = $live.brand
$neutralGatePath = [string]$live.neutralGatePath
$brandGatePath = [string]$live.brandGatePath
$neutralPackage = $live.neutralPackage
$brandPackage = $live.brandPackage
$linkedWorktrees = $live.linkedWorktrees
$neutralComparable = @($live.neutralComparable)
$sizeDifference = [int64]$live.installerSizeDifference

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
