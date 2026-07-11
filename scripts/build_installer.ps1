param(
    [Parameter(Mandatory = $true)]
    [string]$AppVersion,
    [Parameter(Mandatory = $true)]
    [ValidateSet("neutral", "brand")]
    [string]$Channel,
    [switch]$SkipPackageBuild,
    [string]$PackageGateReport = "",
    [string]$OutputBaseFilename = ""
)

$ErrorActionPreference = "Stop"

function Find-FirstExistingPath {
    param([string[]]$Candidates)

    foreach ($candidate in $Candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    return $null
}

function Find-InnoSetupCompiler {
    $standardPaths = @(
        "C:\Program Files (x86)\Inno Setup 6\ISCC.exe",
        "C:\Program Files\Inno Setup 6\ISCC.exe",
        "C:\Program Files (x86)\Inno Setup 5\ISCC.exe"
    )
    $directHit = Find-FirstExistingPath $standardPaths
    if ($directHit) {
        return $directHit
    }

    $uninstallKeys = @(
        "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*",
        "HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*"
    )
    foreach ($key in $uninstallKeys) {
        $installEntries = Get-ItemProperty $key -ErrorAction SilentlyContinue | Where-Object {
            $_.DisplayName -like "*Inno Setup*"
        }
        foreach ($entry in $installEntries) {
            $candidate = Join-Path $entry.InstallLocation "ISCC.exe"
            if (Test-Path -LiteralPath $candidate) {
                return (Resolve-Path -LiteralPath $candidate).Path
            }
        }
    }
    return $null
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "..")).Path
$gateCommon = Join-Path $scriptRoot "release_gate_common.ps1"
if (-not (Test-Path -LiteralPath $gateCommon -PathType Leaf)) {
    throw "Release gate helpers were not found: $gateCommon"
}
. $gateCommon

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
    & $packageScript -AppVersion $AppVersion -Channel $Channel
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

$isccPath = Find-InnoSetupCompiler
if (-not $isccPath) {
    throw "ISCC.exe was not found. Please install Inno Setup 6 first."
}

$installerPath = Join-Path (Join-Path $repoRoot "dist\installer") "$OutputBaseFilename.exe"
if (Test-Path -LiteralPath $installerPath) {
    Remove-Item -LiteralPath $installerPath -Force
}

Write-Host "Compiling Inno Setup installer..."
& $isccPath "/DMyAppVersion=$AppVersion" "/DMyOutputBaseFilename=$OutputBaseFilename" $issPath
if ($LASTEXITCODE -ne 0) {
    throw "Inno Setup compilation failed with exit code $LASTEXITCODE."
}

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
