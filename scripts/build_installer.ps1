param(
    [Parameter(Mandatory = $true)]
    [string]$AppVersion,
    [Parameter(Mandatory = $true)]
    [ValidateSet("neutral", "brand")]
    [string]$Channel,
    [switch]$SkipPackageBuild,
    [string]$PackageGateReport = "",
    [string]$OutputBaseFilename = "",
    [Parameter(Mandatory = $true)][string]$MSBuildExecutable,
    [Parameter(Mandatory = $true)][string]$MSBuildSha256,
    [Parameter(Mandatory = $true)][string]$WinDeployQtExecutable,
    [Parameter(Mandatory = $true)][string]$WinDeployQtSha256,
    [Parameter(Mandatory = $true)][string]$InnoCompilerExecutable,
    [Parameter(Mandatory = $true)][string]$InnoCompilerSha256,
    [Parameter(Mandatory = $true)][string]$PythonExecutable,
    [Parameter(Mandatory = $true)][string]$PythonSha256
)

$ErrorActionPreference = "Stop"

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "..")).Path
$gateCommon = Join-Path $scriptRoot "release_gate_common.ps1"
if (-not (Test-Path -LiteralPath $gateCommon -PathType Leaf)) {
    throw "Release gate helpers were not found: $gateCommon"
}
. $gateCommon

Set-ReleasePythonTool -PythonExecutable $PythonExecutable -PythonSha256 $PythonSha256
$isccPath = Assert-ReleaseExternalTool `
    -Path $InnoCompilerExecutable `
    -ExpectedSha256 $InnoCompilerSha256 `
    -ExpectedFileName "ISCC.exe" `
    -PublisherPattern '(?i)Pyrsys B\.V\.'

Assert-ReleaseVersion $AppVersion
$channelSpec = Get-ReleaseChannelSpec $Channel
$expectedOutputBaseFilename = $channelSpec.OutputPrefix + $AppVersion
if (-not [string]::IsNullOrWhiteSpace($OutputBaseFilename) -and $OutputBaseFilename -cne $expectedOutputBaseFilename) {
    throw "OutputBaseFilename is bound to channel/version and must be exactly $expectedOutputBaseFilename."
}
$OutputBaseFilename = $expectedOutputBaseFilename

$packageScript = Join-Path $scriptRoot "build_release_package.ps1"
$issPath = Join-Path $repoRoot "installer\QtWidgetsApplication4.iss"
$packageDir = Join-Path $repoRoot "dist\QtWidgetsApplication4"
$expectedPackageGateReport = Get-PackageGateReportPath -RepoRoot $repoRoot -AppVersion $AppVersion -Channel $Channel
$installerGateReport = Get-InstallerGateReportPath -RepoRoot $repoRoot -AppVersion $AppVersion -Channel $Channel
$installerGateRunId = New-ReleaseGatePendingReport `
    -Path $installerGateReport `
    -Kind "installer" `
    -AppVersion $AppVersion `
    -Channel $Channel

if (-not $SkipPackageBuild) {
    if (-not [string]::IsNullOrWhiteSpace($PackageGateReport)) {
        throw "-PackageGateReport is only accepted together with -SkipPackageBuild."
    }
    & $packageScript `
        -AppVersion $AppVersion `
        -Channel $Channel `
        -MSBuildExecutable $MSBuildExecutable `
        -MSBuildSha256 $MSBuildSha256 `
        -WinDeployQtExecutable $WinDeployQtExecutable `
        -WinDeployQtSha256 $WinDeployQtSha256 `
        -PythonExecutable $PythonExecutable `
        -PythonSha256 $PythonSha256
    $PackageGateReport = $expectedPackageGateReport
}
else {
    if ([string]::IsNullOrWhiteSpace($PackageGateReport)) {
        throw "-SkipPackageBuild requires -PackageGateReport; a directory or timestamp is never sufficient proof."
    }
    if (-not (Test-ReleaseGateSamePath $PackageGateReport $expectedPackageGateReport)) {
        throw "Package gate report path is not canonical for channel=$Channel version=$AppVersion."
    }
}

$packageReport = Assert-PackageGateReport `
    -ReportPath $PackageGateReport `
    -ExpectedRepoRoot $repoRoot `
    -ExpectedAppVersion $AppVersion `
    -ExpectedChannel $Channel
if (-not (Test-ReleaseGateSamePath ([string]$packageReport.packageDir) $packageDir)) {
    throw "Package gate report points at a non-canonical package directory."
}

$installerPath = Join-Path (Join-Path $repoRoot "dist\installer") "$OutputBaseFilename.exe"
if (Test-Path -LiteralPath $installerPath) {
    Remove-Item -LiteralPath $installerPath -Force
}

Write-Host "Compiling Inno Setup installer..."
$isccPath = Assert-ReleaseExternalTool `
    -Path $isccPath -ExpectedSha256 $InnoCompilerSha256 `
    -ExpectedFileName "ISCC.exe" -PublisherPattern '(?i)Pyrsys B\.V\.'
& $isccPath "/DMyAppVersion=$AppVersion" "/DMyOutputBaseFilename=$OutputBaseFilename" $issPath
if ($LASTEXITCODE -ne 0) {
    throw "Inno Setup compilation failed with exit code $LASTEXITCODE."
}
$isccPath = Assert-ReleaseExternalTool `
    -Path $isccPath -ExpectedSha256 $InnoCompilerSha256 `
    -ExpectedFileName "ISCC.exe" -PublisherPattern '(?i)Pyrsys B\.V\.'

$writtenInstallerGate = New-InstallerGateReport `
    -RepoRoot $repoRoot `
    -PackageGateReportPath $PackageGateReport `
    -InstallerPath $installerPath `
    -AppVersion $AppVersion `
    -Channel $Channel `
    -RunId $installerGateRunId `
    -OutputPath $installerGateReport

Write-Host ""
Write-Host "Installer output:"
Write-Host "  $installerPath"
Write-Host "Installer gate report:"
Write-Host "  $writtenInstallerGate"
