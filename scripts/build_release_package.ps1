param(
    [Parameter(Mandatory = $true)]
    [string]$AppVersion,
    [Parameter(Mandatory = $true)]
    [ValidateSet("neutral", "brand")]
    [string]$Channel,
    [switch]$SkipBuild,
    [switch]$SkipVcRedistDownload,
    [switch]$SkipFanucCompilerTools,
    [string]$Configuration = "Release",
    [string]$Platform = "x64",
    [Parameter(Mandatory = $true)][string]$MSBuildExecutable,
    [Parameter(Mandatory = $true)][string]$MSBuildSha256,
    [Parameter(Mandatory = $true)][string]$WinDeployQtExecutable,
    [Parameter(Mandatory = $true)][string]$WinDeployQtSha256,
    [Parameter(Mandatory = $true)][string]$QtMsBuildPath,
    [Parameter(Mandatory = $true)][string]$PythonExecutable,
    [Parameter(Mandatory = $true)][string]$PythonSha256
)

$ErrorActionPreference = "Stop"

function Copy-DirectoryContent {
    param(
        [string]$SourceDir,
        [string]$TargetDir
    )

    if (-not (Test-Path -LiteralPath $SourceDir)) {
        return
    }

    New-Item -ItemType Directory -Path $TargetDir -Force | Out-Null
    Copy-Item -Path (Join-Path $SourceDir "*") -Destination $TargetDir -Recurse -Force
}

function Copy-TrackedReleaseFile {
    param(
        [Parameter(Mandatory = $true)][string]$RelativePath,
        [Parameter(Mandatory = $true)][string]$DestinationPath
    )

    $sourcePath = Assert-GitTrackedReleaseFile -RepoRoot $repoRoot -RelativePath $RelativePath
    $parent = Split-Path -Parent $DestinationPath
    if (-not [string]::IsNullOrWhiteSpace($parent)) {
        New-Item -ItemType Directory -Path $parent -Force | Out-Null
    }
    Copy-Item -LiteralPath $sourcePath -Destination $DestinationPath -Force
}

function Download-FileIfNeeded {
    param(
        [string]$Url,
        [string]$TargetPath
    )

    if (Test-Path -LiteralPath $TargetPath) {
        return $true
    }

    $targetParent = Split-Path -Parent $TargetPath
    New-Item -ItemType Directory -Path $targetParent -Force | Out-Null
    $downloadPath = "$TargetPath.download"
    for ($attempt = 1; $attempt -le 3; $attempt++) {
        Remove-Item -LiteralPath $downloadPath -Force -ErrorAction SilentlyContinue
        try {
            Invoke-WebRequest -Uri $Url -OutFile $downloadPath -UseBasicParsing
            if (-not (Test-Path -LiteralPath $downloadPath -PathType Leaf) `
                -or (Get-Item -LiteralPath $downloadPath).Length -le 0) {
                throw "downloaded file is missing or empty"
            }
            Move-Item -LiteralPath $downloadPath -Destination $TargetPath -Force
            return $true
        }
        catch {
            Remove-Item -LiteralPath $downloadPath -Force -ErrorAction SilentlyContinue
            Write-Warning ("Failed to download prerequisite from {0} (attempt {1}/3). {2}" `
                -f $Url, $attempt, $_.Exception.Message)
            if ($attempt -lt 3) {
                Start-Sleep -Seconds (2 * $attempt)
            }
        }
    }
    return $false
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "..")).Path
$gateCommon = Join-Path $scriptRoot "release_gate_common.ps1"
if (-not (Test-Path -LiteralPath $gateCommon -PathType Leaf)) {
    throw "Release gate helpers were not found: $gateCommon"
}
. $gateCommon

$msbuildPath = Assert-ReleaseExternalTool `
    -Path $MSBuildExecutable `
    -ExpectedSha256 $MSBuildSha256 `
    -ExpectedFileName "MSBuild.exe" `
    -PublisherPattern '(?i)Microsoft Corporation'
$windeployqtPath = Assert-ReleaseExternalTool `
    -Path $WinDeployQtExecutable `
    -ExpectedSha256 $WinDeployQtSha256 `
    -ExpectedFileName "windeployqt.exe" `
    -PublisherPattern '(?i)The Qt Company'
$qtMsBuildRoot = (Resolve-Path -LiteralPath $QtMsBuildPath -ErrorAction Stop).Path
$qtMsBuildItem = Get-Item -LiteralPath $qtMsBuildRoot -Force
if (-not $qtMsBuildItem.PSIsContainer `
    -or (($qtMsBuildItem.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0)) {
    throw "QtMsBuildPath must be a non-reparse directory: $qtMsBuildRoot"
}
foreach ($requiredQtMsBuildFile in @("Qt.props", "qt.targets")) {
    $requiredQtMsBuildPath = Join-Path $qtMsBuildRoot $requiredQtMsBuildFile
    if (-not (Test-Path -LiteralPath $requiredQtMsBuildPath -PathType Leaf) `
        -or (((Get-Item -LiteralPath $requiredQtMsBuildPath -Force).Attributes `
            -band [System.IO.FileAttributes]::ReparsePoint) -ne 0)) {
        throw "QtMsBuildPath is missing a non-reparse $requiredQtMsBuildFile file."
    }
}
Set-ReleasePythonTool -PythonExecutable $PythonExecutable -PythonSha256 $PythonSha256

Assert-ReleaseVersion $AppVersion
$packageGateReport = Get-PackageGateReportPath -RepoRoot $repoRoot -AppVersion $AppVersion -Channel $Channel
$packageGateRunId = New-ReleaseGatePendingReport `
    -Path $packageGateReport `
    -Kind "package" `
    -AppVersion $AppVersion `
    -Channel $Channel
if ($SkipBuild) {
    throw "-SkipBuild is forbidden for release packaging because it can reuse a stale executable."
}
if ($SkipVcRedistDownload) {
    throw "-SkipVcRedistDownload is forbidden for release packaging because VC runtime presence is a hard gate."
}
if ($Configuration -cne "Release" -or $Platform -cne "x64") {
    throw "Release packaging is fixed to Configuration=Release and Platform=x64."
}
$gitState = Assert-GitReleaseState -RepoRoot $repoRoot -AppVersion $AppVersion -Channel $Channel
$channelSpec = Get-ReleaseChannelSpec $Channel
Assert-SourceApplicationVersion -RepoRoot $repoRoot -AppVersion $AppVersion | Out-Null

$solutionPath = Join-Path $repoRoot "QtWidgetsApplication4.sln"
$buildDir = Join-Path $repoRoot ("x64\" + $Configuration)
$packageDir = Join-Path $repoRoot "dist\QtWidgetsApplication4"
$canonicalBuildDir = Join-Path $repoRoot "x64\Release"
$controlledIntermediateRoot = Join-Path $repoRoot "tmp\ReleaseBuild"
$intermediateDir = Join-Path $controlledIntermediateRoot "$Channel\$AppVersion\obj"
$controlledUserRoot = Join-Path $controlledIntermediateRoot "$Channel\$AppVersion\empty-user-root"
if (-not (Test-ReleaseGateSamePath $buildDir $canonicalBuildDir)) {
    throw "Release build cleanup escaped the canonical x64/Release directory: $buildDir"
}
$intermediateFull = Resolve-ReleaseGatePath $intermediateDir
$intermediatePrefix = (Resolve-ReleaseGatePath $controlledIntermediateRoot) + [System.IO.Path]::DirectorySeparatorChar
if (-not $intermediateFull.StartsWith($intermediatePrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Release intermediate cleanup escaped the controlled tmp/ReleaseBuild directory: $intermediateDir"
}

if (Test-Path -LiteralPath $buildDir) {
    Remove-Item -LiteralPath $buildDir -Recurse -Force
}
if (Test-Path -LiteralPath $intermediateDir) {
    Remove-Item -LiteralPath $intermediateDir -Recurse -Force
}
if (Test-Path -LiteralPath $buildDir) {
    throw "Controlled Release build directory could not be cleaned: $buildDir"
}
if (Test-Path -LiteralPath $intermediateDir) {
    throw "Controlled Release intermediate directory could not be cleaned: $intermediateDir"
}
New-Item -ItemType Directory -Path $controlledUserRoot -Force | Out-Null
$versionParts = @($AppVersion.Split('.') | ForEach-Object { [int]$_ })
Write-Host "Building Release package with a clean MSBuild Rebuild..."
$msbuildPath = Assert-ReleaseExternalTool `
    -Path $msbuildPath -ExpectedSha256 $MSBuildSha256 `
    -ExpectedFileName "MSBuild.exe" -PublisherPattern '(?i)Microsoft Corporation'
& $msbuildPath $solutionPath /m /t:Rebuild `
    "/p:Configuration=$Configuration" `
    "/p:Platform=$Platform" `
    "/p:OutDir=$buildDir\" `
    "/p:IntDir=$intermediateDir\" `
    "/p:UserRootDir=$controlledUserRoot\" `
    "/p:ImportDirectoryBuildProps=false" `
    "/p:ImportDirectoryBuildTargets=false" `
    "/p:DirectoryBuildPropsPath=" `
    "/p:DirectoryBuildTargetsPath=" `
    "/p:QtMsBuild=$qtMsBuildRoot" `
    "/p:CustomBeforeMicrosoftCommonTargets=" `
    "/p:CustomAfterMicrosoftCommonTargets=" `
    "/p:ForceImportBeforeCppTargets=" `
    "/p:ForceImportAfterCppTargets=" `
    "/p:ReleaseVersionMajor=$($versionParts[0])" `
    "/p:ReleaseVersionMinor=$($versionParts[1])" `
    "/p:ReleaseVersionPatch=$($versionParts[2])" `
    "/p:ReleaseVersionBuild=$($versionParts[3])" `
    "/p:ReleaseVersionString=$AppVersion" `
    "/p:ReleaseProductName=$($channelSpec.AppName)" `
    "/p:ReleaseExeName=$($channelSpec.ExeName)" `
    /v:m
if ($LASTEXITCODE -ne 0) {
    throw "MSBuild failed with exit code $LASTEXITCODE."
}
$msbuildPath = Assert-ReleaseExternalTool `
    -Path $msbuildPath -ExpectedSha256 $MSBuildSha256 `
    -ExpectedFileName "MSBuild.exe" -PublisherPattern '(?i)Microsoft Corporation'

$exePath = Join-Path $buildDir $channelSpec.ExeName
if (-not (Test-Path -LiteralPath $exePath)) {
    throw "Release executable was not found: $exePath"
}
Assert-WindowsPeReleaseVersion `
    -Path $exePath `
    -AppVersion $AppVersion `
    -ExpectedProductName $channelSpec.AppName `
    -ExpectedOriginalFilename $channelSpec.ExeName | Out-Null

Write-Host "Running windeployqt on the Release executable..."
# Trim payload the app never loads: D3D/DXC shader compilers (pure Widgets + QOpenGLWidget,
# no Qt Quick/RHI), PDF plugin chain (no PDF feature), unused image formats (icons are svg/ico,
# png is built into Qt6Gui; keep qjpeg conservatively), unused SQL drivers (only QSQLITE),
# Network-information and generic/TUIO plugins are unused. Keep the TLS plugin type:
# OTA and the server-management API use HTTPS and require qschannelbackend.dll on Windows.
$windeployqtPath = Assert-ReleaseExternalTool `
    -Path $windeployqtPath -ExpectedSha256 $WinDeployQtSha256 `
    -ExpectedFileName "windeployqt.exe" -PublisherPattern '(?i)The Qt Company'
& $windeployqtPath --release --no-translations --no-opengl-sw `
    --no-system-d3d-compiler --no-system-dxc-compiler `
    --exclude-plugins qpdf,qtiff,qtga,qicns,qwbmp,qgif,qsqlodbc,qsqlpsql,qsqlmimer `
    --skip-plugin-types networkinformation,generic `
    $exePath
if ($LASTEXITCODE -ne 0) {
    throw "windeployqt failed with exit code $LASTEXITCODE."
}
$windeployqtPath = Assert-ReleaseExternalTool `
    -Path $windeployqtPath -ExpectedSha256 $WinDeployQtSha256 `
    -ExpectedFileName "windeployqt.exe" -PublisherPattern '(?i)The Qt Company'

# windeployqt only ever adds files. Remove previously deployed payload that the
# options above no longer produce, so stale files in x64\<Config> cannot leak
# back into the package (the same mechanism that once leaked vc_redist.x64.exe).
$staleDeployFiles = @(
    "dxcompiler.dll", "dxil.dll", "D3Dcompiler_47.dll", "Qt6Pdf.dll",
    "imageformats\qpdf.dll", "imageformats\qtiff.dll",
    "imageformats\qtga.dll", "imageformats\qicns.dll", "imageformats\qwbmp.dll",
    "imageformats\qgif.dll",
    "sqldrivers\qsqlodbc.dll", "sqldrivers\qsqlpsql.dll", "sqldrivers\qsqlmimer.dll"
)
foreach ($staleRelative in $staleDeployFiles) {
    $stalePath = Join-Path $buildDir $staleRelative
    if (Test-Path -LiteralPath $stalePath) {
        Remove-Item -LiteralPath $stalePath -Force
    }
}
foreach ($staleDir in @("networkinformation", "generic")) {
    $staleDirPath = Join-Path $buildDir $staleDir
    if (Test-Path -LiteralPath $staleDirPath) {
        Remove-Item -LiteralPath $staleDirPath -Recurse -Force
    }
}

$schannelBackendPath = Join-Path $buildDir "tls\qschannelbackend.dll"
if (-not (Test-Path -LiteralPath $schannelBackendPath -PathType Leaf)) {
    throw "Required Qt Schannel TLS backend was not deployed: $schannelBackendPath"
}

$qtRoot = Split-Path -Parent (Split-Path -Parent $windeployqtPath)
$qtTranslationsDir = Join-Path $qtRoot "translations"
$buildTranslationsDir = Join-Path $buildDir "translations"
if (Test-Path -LiteralPath $buildTranslationsDir) {
    Remove-Item -LiteralPath $buildTranslationsDir -Recurse -Force
}
New-Item -ItemType Directory -Path $buildTranslationsDir -Force | Out-Null
foreach ($translationFile in @("qt_zh_CN.qm", "qtbase_zh_CN.qm")) {
    $sourceTranslation = Join-Path $qtTranslationsDir $translationFile
    if (-not (Test-Path -LiteralPath $sourceTranslation -PathType Leaf)) {
        throw "Required Qt translation file was not found: $sourceTranslation"
    }
    Copy-Item -LiteralPath $sourceTranslation -Destination (Join-Path $buildTranslationsDir $translationFile) -Force
}

# The build directory is shared by both channel builds. Any second top-level application
# executable means the caller did not clean the other channel and packaging must stop.
Assert-ExpectedReleaseExecutable -Directory $buildDir -AppVersion $AppVersion -Channel $Channel | Out-Null

if (Test-Path -LiteralPath $packageDir) {
    Remove-Item -LiteralPath $packageDir -Recurse -Force
}
New-Item -ItemType Directory -Path $packageDir -Force | Out-Null

$ignoredReleaseExtensions = @(".lib", ".exp", ".pdb", ".obj", ".iobj", ".ipdb", ".ilk")
# The redistributable is staged into Prerequisites\ below (the only copy the
# installer actually runs); a stray top-level copy must never leak into the
# package root.
# The unique-executable gate above has already rejected any opposite-channel or
# unrelated top-level exe. Redistributables are staged under Prerequisites only.
$ignoredReleaseFileNames = @("vc_redist.x64.exe", "vc_redist.x86.exe")
Get-ChildItem -LiteralPath $buildDir -File | Where-Object {
    $ignoredReleaseExtensions -notcontains $_.Extension.ToLowerInvariant() -and
    $ignoredReleaseFileNames -notcontains $_.Name.ToLowerInvariant()
} | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $packageDir $_.Name) -Force
}

$runtimeOwnedDirectories = @("data", "log", "result", "temp")
Get-ChildItem -LiteralPath $buildDir -Directory | Where-Object {
    $runtimeOwnedDirectories -notcontains $_.Name.ToLowerInvariant()
} | ForEach-Object {
    Copy-DirectoryContent -SourceDir $_.FullName -TargetDir (Join-Path $packageDir $_.Name)
}

foreach ($runtimeDir in @("Data", "Log", "Result", "Temp")) {
    $runtimePath = Join-Path $packageDir $runtimeDir
    if (Test-Path -LiteralPath $runtimePath) {
        Remove-Item -LiteralPath $runtimePath -Recurse -Force
    }
    New-Item -ItemType Directory -Path $runtimePath -Force | Out-Null
}

Copy-DirectoryContent -SourceDir (Join-Path $repoRoot "icons") -TargetDir (Join-Path $packageDir "icons")

# 品牌覆盖包只分发图标资产；品牌文字与启用状态由 ConfigStore 的 global/Branding 模块管理。
# main 等中性分支不跟踪 branding/，安装包保持纯中性 NoTeaching-Robot。
if ($channelSpec.RequiresBranding) {
    Copy-DirectoryContent -SourceDir (Join-Path $repoRoot "branding") -TargetDir (Join-Path $packageDir "branding")
}

$pointCloudExtractionSourceDir = Join-Path $repoRoot "SDK\PointCloudExtration"
$pointCloudExtractionTargetDir = Join-Path $packageDir "SDK\PointCloudExtration"
# Ship only the runtime dependency closure of PointCloudExtration.dll (verified with
# dumpbin /DEPENDENTS, the DLL does not import LoadLibrary so the closure is complete).
# The vendor directory also contains debug opencv *2413d.dll builds and DLLs outside
# the closure (pcl_surface/pcl_visualization/opencv calib3d 等) — dead weight (~42 MB).
# Do NOT touch the SDK source directory itself; filtering happens only at packaging.
$pointCloudExtractionRuntimeFiles = @(
    "PointCloudExtration.dll",
    "OpenNI2.dll",
    "opencv_core2413.dll", "opencv_highgui2413.dll", "opencv_imgproc2413.dll",
    "pcl_common_release.dll", "pcl_features_release.dll", "pcl_filters_release.dll",
    "pcl_io_release.dll", "pcl_io_ply_release.dll", "pcl_kdtree_release.dll",
    "pcl_ml_release.dll", "pcl_octree_release.dll", "pcl_sample_consensus_release.dll",
    "pcl_search_release.dll", "pcl_segmentation_release.dll"
)
New-Item -ItemType Directory -Path $pointCloudExtractionTargetDir -Force | Out-Null
foreach ($runtimeFile in $pointCloudExtractionRuntimeFiles) {
    $relative = "SDK/PointCloudExtration/$runtimeFile"
    Copy-TrackedReleaseFile -RelativePath $relative -DestinationPath (Join-Path $pointCloudExtractionTargetDir $runtimeFile)
}
# config\ holds the default algorithm INI the app reads to derive *.runtime.ini.
$pointCloudConfigDir = Join-Path $pointCloudExtractionSourceDir "config"
if (-not (Test-Path -LiteralPath $pointCloudConfigDir -PathType Container)) {
    throw "PointCloudExtration config directory was not found: $pointCloudConfigDir"
}
$trackedPointCloudConfig = @(Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("ls-files", "--", "SDK/PointCloudExtration/config"))
if ($trackedPointCloudConfig.Count -eq 0) {
    throw "PointCloudExtration config directory is empty: $pointCloudConfigDir"
}
foreach ($relative in $trackedPointCloudConfig) {
    $normalized = ([string]$relative).Replace('\', '/')
    if (-not $normalized.StartsWith("SDK/PointCloudExtration/config/", [System.StringComparison]::Ordinal)) {
        throw "Unexpected tracked PointCloud config path: $relative"
    }
    $configRelative = $normalized.Substring("SDK/PointCloudExtration/".Length).Replace('/', '\')
    Copy-TrackedReleaseFile -RelativePath $normalized -DestinationPath (Join-Path $pointCloudExtractionTargetDir $configRelative)
}

# SDK\STEP\versions is intentionally NOT shipped at all:
# - The STEP SDK is statically linked into the exe (dumpbin shows no Robot-SDK.dll),
#   so the .lib/.hpp archives are link-time material, useless on field machines.
# - The robot-system upgrade package (SRS_*.zip) stays archived in the source
#   repository only (SDK\STEP\versions\timestamp_*\); per decision it is not
#   bundled into the installer — distribute it to field sites separately.

$configMigrateBuildScript = Join-Path $repoRoot "scripts\build_config_migrate.ps1"
if (-not (Test-Path -LiteralPath $configMigrateBuildScript -PathType Leaf)) {
    throw "Config migration build script was not found: $configMigrateBuildScript"
}
& $configMigrateBuildScript `
    -PythonExecutable $PythonExecutable `
    -PythonSha256 $PythonSha256 `
    -OutputPath (Join-Path $repoRoot "tools\ConfigMigrate.exe") | Out-Null

$diagnosticToolsSourceDir = Join-Path $repoRoot "tools"
$configMigrateExe = Join-Path $diagnosticToolsSourceDir "ConfigMigrate.exe"
$configMigrateSourceProvenance = Assert-ConfigMigrateProvenance -RepoRoot $repoRoot -ExecutablePath $configMigrateExe

$diagnosticToolsTargetDir = Join-Path $packageDir "tools"
if (Test-Path -LiteralPath $diagnosticToolsSourceDir) {
    $trackedTools = @(Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("ls-files", "--", "tools")) | Where-Object {
        [System.IO.Path]::GetExtension([string]$_).ToLowerInvariant() -in @(".ps1", ".cmd", ".bat", ".py", ".txt", ".md")
    }
    foreach ($relative in $trackedTools) {
        $normalized = ([string]$relative).Replace('\', '/')
        if (-not $normalized.StartsWith("tools/", [System.StringComparison]::Ordinal)) {
            throw "Unexpected tracked diagnostic tool path: $relative"
        }
        $toolRelative = $normalized.Substring("tools/".Length).Replace('/', '\')
        Copy-TrackedReleaseFile -RelativePath $normalized -DestinationPath (Join-Path $diagnosticToolsTargetDir $toolRelative)
    }
}

$stepSdkSwitchScript = Join-Path $repoRoot "scripts\switch_step_sdk.ps1"
if (Test-Path -LiteralPath $stepSdkSwitchScript) {
    Copy-TrackedReleaseFile -RelativePath "scripts/switch_step_sdk.ps1" -DestinationPath (Join-Path $diagnosticToolsTargetDir "switch_step_sdk.ps1")
}

$installerToolsDir = Join-Path $repoRoot "dist\tools"
New-Item -ItemType Directory -Path $installerToolsDir -Force | Out-Null
foreach ($toolName in @("ConfigMigrate.exe", "ConfigMigrate_Run.cmd", "ConfigMigrate_Install.ps1")) {
    $toolSource = Join-Path $diagnosticToolsSourceDir $toolName
    if (Test-Path -LiteralPath $toolSource) {
        Copy-Item -LiteralPath $toolSource -Destination (Join-Path $installerToolsDir $toolName) -Force
    }
    else {
        throw "Database migration installer tool was not found: $toolSource"
    }
}
$installerConfigMigrate = Join-Path $installerToolsDir "ConfigMigrate.exe"
$installerConfigProvenance = Assert-ConfigMigrateProvenance -RepoRoot $repoRoot -ExecutablePath $installerConfigMigrate
if ($installerConfigProvenance.sha256 -cne $configMigrateSourceProvenance.sha256) {
    throw "dist/tools/ConfigMigrate.exe differs from the source-provenance-checked tools/ConfigMigrate.exe."
}

$fanucSourceDir = Join-Path $repoRoot "SDK\FANUC"
$fanucTargetDir = Join-Path $packageDir "SDK\FANUC"
New-Item -ItemType Directory -Path $fanucTargetDir -Force | Out-Null
Assert-FanucRuntimeManifest -RepoRoot $repoRoot -RuntimeRoot $repoRoot | Out-Null
# Only the exact versioned manifest is field runtime input. Source .kl/.ls and
# ignored compiler byproducts must never enter an installer by directory scan.
$fanucManifest = Read-FanucRuntimeManifest $repoRoot
foreach ($item in @($fanucManifest.manifest.files)) {
    $relative = ([string]$item.path).Replace('/', '\')
    Copy-Item -LiteralPath (Join-Path $repoRoot $relative) -Destination (Join-Path $fanucTargetDir ([System.IO.Path]::GetFileName($relative))) -Force
}
Assert-FanucRuntimeManifest -RepoRoot $repoRoot -RuntimeRoot $packageDir | Out-Null

# Ship BCPD (self-contained bcpd.exe + MIT license) for the model-alignment / point-cloud
# denoising feature. Runtime locates it at <root>\SDK\BCPD\bcpd.exe.
$bcpdSourceDir = Join-Path $repoRoot "SDK\BCPD"
$bcpdTargetDir = Join-Path $packageDir "SDK\BCPD"
if (Test-Path -LiteralPath $bcpdSourceDir) {
    $trackedBcpd = @(Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("ls-files", "--", "SDK/BCPD"))
    foreach ($relative in $trackedBcpd) {
        $normalized = ([string]$relative).Replace('\', '/')
        if (-not $normalized.StartsWith("SDK/BCPD/", [System.StringComparison]::Ordinal)) {
            throw "Unexpected tracked BCPD path: $relative"
        }
        $bcpdRelative = $normalized.Substring("SDK/BCPD/".Length).Replace('/', '\')
        Copy-TrackedReleaseFile -RelativePath $normalized -DestinationPath (Join-Path $bcpdTargetDir $bcpdRelative)
    }
    foreach ($requiredBcpd in @("bcpd.exe", "LICENSE.md")) {
        if (-not (Test-Path -LiteralPath (Join-Path $bcpdTargetDir $requiredBcpd) -PathType Leaf)) {
            throw "Tracked BCPD release asset is missing: $requiredBcpd"
        }
    }
}

# Machine-local WinOLPC directories have no repository manifest and are therefore
# deliberately excluded from every release, regardless of this legacy switch.

$redistDir = Join-Path $packageDir "Prerequisites"
New-Item -ItemType Directory -Path $redistDir -Force | Out-Null
if (-not $SkipVcRedistDownload) {
    $vcRedistTarget = Join-Path $redistDir "vc_redist.x64.exe"
    $vcRedistOk = $false
    $deployedVcRedist = Join-Path $buildDir "vc_redist.x64.exe"
    if (Test-Path -LiteralPath $deployedVcRedist -PathType Leaf) {
        $deployedVcRedistSignature = Get-AuthenticodeSignature -LiteralPath $deployedVcRedist
        if ((Get-Item -LiteralPath $deployedVcRedist).Length -gt 0 `
            -and $deployedVcRedistSignature.Status -eq [System.Management.Automation.SignatureStatus]::Valid `
            -and $null -ne $deployedVcRedistSignature.SignerCertificate `
            -and $deployedVcRedistSignature.SignerCertificate.Subject -match 'CN=Microsoft Corporation') {
            Copy-Item -LiteralPath $deployedVcRedist -Destination $vcRedistTarget -Force
            $vcRedistOk = $true
        }
    }
    if (-not $vcRedistOk) {
        $vcRedistOk = Download-FileIfNeeded `
            -Url "https://aka.ms/vc14/vc_redist.x64.exe" -TargetPath $vcRedistTarget
    }
    if (-not $vcRedistOk) {
        throw "VC++ runtime installer download failed; an incomplete release package is forbidden."
    }
}
$vcRedistPath = Join-Path $redistDir "vc_redist.x64.exe"
if (-not (Test-Path -LiteralPath $vcRedistPath -PathType Leaf) -or (Get-Item -LiteralPath $vcRedistPath).Length -le 0) {
    throw "VC++ runtime installer is missing or empty: $vcRedistPath"
}
$vcRedistSignature = Get-AuthenticodeSignature -LiteralPath $vcRedistPath
if ($vcRedistSignature.Status -ne [System.Management.Automation.SignatureStatus]::Valid `
    -or $null -eq $vcRedistSignature.SignerCertificate `
    -or $vcRedistSignature.SignerCertificate.Subject -notmatch 'CN=Microsoft Corporation') {
    throw "VC++ runtime installer does not have a valid Microsoft Authenticode signature: $vcRedistPath"
}

$notesPath = Join-Path $packageDir "DEPLOY_NOTES.txt"
$notes = @(
    "NoTeaching-Robot deployment notes",
    "",
    "1. This package was generated from the local Release build output.",
    "2. The application writes logs, results and editable config files next to the executable.",
    "3. Because of that, the installer defaults to a user-writable folder instead of Program Files.",
    "4. The installer bundles the Microsoft Visual C++ 2015-2022 Redistributable x64 installer and can run it automatically.",
    "5. FANUC WinOLPC compiler tools are not bundled because machine-local files have no release manifest.",
    "6. Only the exact versioned FANUC .tp/.pc runtime manifest is shipped.",
    "7. The robot-controller STEP SDK is statically linked; SDK\\STEP\\versions is not shipped because its .lib archives",
    "   are link-time only, while the SRS robot-system upgrade package is archived in source and distributed separately.",
    "8. CAD STEP import uses facilities provided by Open CASCADE Technology 7.9.3 through dynamically linked DLLs.",
    "   Keep those TK*.dll files and licenses\\opencascade\\copyright together with every redistributed package.",
    "9. Rebuilds (incl. legacy SDK switch via switch_step_sdk.ps1) are done from the source repository."
)
$notes | Set-Content -LiteralPath $notesPath -Encoding UTF8

$buildInfoPath = Join-Path $packageDir "BUILD_VERSION.txt"
@(
    $channelSpec.AppName,
    "Version: $AppVersion",
    "Channel: $Channel",
    "Commit: $($gitState.head)",
    "Installer: $($channelSpec.OutputPrefix)$AppVersion.exe",
    "BuiltAtUtc: $([DateTime]::UtcNow.ToString('o'))"
) | Set-Content -LiteralPath $buildInfoPath -Encoding UTF8

Assert-EmptyReleaseRuntimeDirectories $packageDir
$writtenGateReport = New-PackageGateReport `
    -RepoRoot $repoRoot `
    -PackageDir $packageDir `
    -AppVersion $AppVersion `
    -Channel $Channel `
    -ConfigMigratePath $installerConfigMigrate `
    -RunId $packageGateRunId `
    -OutputPath $packageGateReport

Write-Host ""
Write-Host "Release package is ready:"
Write-Host "  $packageDir"
Write-Host "Package gate report:"
Write-Host "  $writtenGateReport"
