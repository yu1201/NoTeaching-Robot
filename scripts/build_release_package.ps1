param(
    [switch]$SkipBuild,
    [switch]$SkipVcRedistDownload,
    [switch]$SkipFanucCompilerTools,
    [string]$Configuration = "Release",
    [string]$Platform = "x64"
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

    try {
        Invoke-WebRequest -Uri $Url -OutFile $TargetPath -UseBasicParsing
        return $true
    }
    catch {
        Write-Warning ("Failed to download prerequisite from {0}. {1}" -f $Url, $_.Exception.Message)
        return $false
    }
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "..")).Path
$solutionPath = Join-Path $repoRoot "QtWidgetsApplication4.sln"
$buildDir = Join-Path $repoRoot ("x64\" + $Configuration)
$packageDir = Join-Path $repoRoot "dist\QtWidgetsApplication4"

$msbuildPath = Find-FirstExistingPath @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\MSBuild.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\MSBuild\Current\Bin\MSBuild.exe"
)

$windeployqtPath = Find-FirstExistingPath @(
    "E:\workspace\soft\QT\6.7.3\msvc2022_64\bin\windeployqt.exe",
    "$env:QTDIR\bin\windeployqt.exe"
)

if (-not $SkipBuild) {
    if (-not $msbuildPath) {
        throw "MSBuild.exe was not found. Please install Visual Studio 2022 Build Tools or fix the path in scripts/build_release_package.ps1."
    }

    Write-Host "Building Release package with MSBuild..."
    & $msbuildPath $solutionPath /m /p:Configuration=$Configuration /p:Platform=$Platform /v:m
    if ($LASTEXITCODE -ne 0) {
        throw "MSBuild failed with exit code $LASTEXITCODE."
    }
}

$exePath = Join-Path $buildDir "HK-Pathlynx-CORPLA.exe"
if (-not (Test-Path -LiteralPath $exePath)) {
    throw "Release executable was not found: $exePath"
}

if ($windeployqtPath) {
    Write-Host "Running windeployqt on the Release executable..."
    # Trim payload the app never loads: D3D/DXC shader compilers (pure Widgets + QOpenGLWidget,
    # no Qt Quick/RHI), PDF plugin chain (no PDF feature), unused image formats (icons are svg/ico,
    # png is built into Qt6Gui; keep qjpeg conservatively), unused SQL drivers (only QSQLITE),
    # TLS backends (plain sockets only; FTP goes through WinINet) and TUIO touch plugin.
    & $windeployqtPath --release --no-translations --no-opengl-sw `
        --no-system-d3d-compiler --no-system-dxc-compiler `
        --exclude-plugins qpdf,qtiff,qtga,qicns,qwbmp,qgif,qsqlodbc,qsqlpsql,qsqlmimer `
        --skip-plugin-types tls,networkinformation,generic `
        $exePath
    if ($LASTEXITCODE -ne 0) {
        throw "windeployqt failed with exit code $LASTEXITCODE."
    }

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
    foreach ($staleDir in @("tls", "networkinformation", "generic")) {
        $staleDirPath = Join-Path $buildDir $staleDir
        if (Test-Path -LiteralPath $staleDirPath) {
            Remove-Item -LiteralPath $staleDirPath -Recurse -Force
        }
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
        if (Test-Path -LiteralPath $sourceTranslation) {
            Copy-Item -LiteralPath $sourceTranslation -Destination (Join-Path $buildTranslationsDir $translationFile) -Force
        }
        else {
            Write-Warning "Qt translation file was not found: $sourceTranslation"
        }
    }
}
else {
    Write-Warning "windeployqt.exe was not found. Qt runtime deployment was skipped."
}

if (Test-Path -LiteralPath $packageDir) {
    Remove-Item -LiteralPath $packageDir -Recurse -Force
}
New-Item -ItemType Directory -Path $packageDir -Force | Out-Null

$ignoredReleaseExtensions = @(".lib", ".exp", ".pdb", ".obj", ".iobj", ".ipdb", ".ilk")
# The redistributable is downloaded into Prerequisites\ below (the only copy the
# installer actually runs); a stray copy manually dropped into x64\Release once
# leaked a duplicate 24 MB vc_redist into the package root.
# 同一 x64\Release 目录反复构建品牌+中性两版会互相留下残留 exe，不排除会让品牌包混入
# 中性 exe。品牌分支产出 HK-Pathlynx-CORPLA.exe，这里排除中性主程序 QtWidgetsApplication4.exe。
$ignoredReleaseFileNames = @("vc_redist.x64.exe", "vc_redist.x86.exe", "qtwidgetsapplication4.exe")
Get-ChildItem -LiteralPath $buildDir -File | Where-Object {
    $ignoredReleaseExtensions -notcontains $_.Extension.ToLowerInvariant() -and
    $ignoredReleaseFileNames -notcontains $_.Name.ToLowerInvariant()
} | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $packageDir $_.Name) -Force
}

Get-ChildItem -LiteralPath $buildDir -Directory | ForEach-Object {
    Copy-DirectoryContent -SourceDir $_.FullName -TargetDir (Join-Path $packageDir $_.Name)
}

$dataSourceDir = Join-Path $repoRoot "Data"
$dataTargetDir = Join-Path $packageDir "Data"
New-Item -ItemType Directory -Path $dataTargetDir -Force | Out-Null
# Data is runtime-owned. Do not ship field robots, templates, INI/TXT defaults,
# or the per-device ConfigStore.db. A fresh install creates an empty database on
# first run; an update must leave the existing Data directory untouched.
Get-ChildItem -LiteralPath $dataTargetDir -Filter "ConfigStore.db*" -File -ErrorAction SilentlyContinue | ForEach-Object {
    Remove-Item -LiteralPath $_.FullName -Force
}

Copy-DirectoryContent -SourceDir (Join-Path $repoRoot "icons") -TargetDir (Join-Path $packageDir "icons")

# 品牌覆盖包：仅当 branding/ 被 git 跟踪（品牌分支）才随包分发；
# main 等中性分支的 branding/ 被 .gitignore、不入包，安装包保持纯中性 NoTeaching-Robot。
if (& git -C $repoRoot ls-files "branding/") {
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
    $runtimeSource = Join-Path $pointCloudExtractionSourceDir $runtimeFile
    if (Test-Path -LiteralPath $runtimeSource) {
        Copy-Item -LiteralPath $runtimeSource -Destination (Join-Path $pointCloudExtractionTargetDir $runtimeFile) -Force
    }
    else {
        Write-Warning "PointCloudExtration runtime file was not found: $runtimeSource"
    }
}
# config\ holds the default algorithm INI the app reads to derive *.runtime.ini.
Copy-DirectoryContent -SourceDir (Join-Path $pointCloudExtractionSourceDir "config") -TargetDir (Join-Path $pointCloudExtractionTargetDir "config")

# SDK\STEP\versions is intentionally NOT shipped at all:
# - The STEP SDK is statically linked into the exe (dumpbin shows no Robot-SDK.dll),
#   so the .lib/.hpp archives are link-time material, useless on field machines.
# - The robot-system upgrade package (SRS_*.zip) stays archived in the source
#   repository only (SDK\STEP\versions\timestamp_*\); per decision it is not
#   bundled into the installer — distribute it to field sites separately.

$diagnosticToolsSourceDir = Join-Path $repoRoot "tools"
$diagnosticToolsTargetDir = Join-Path $packageDir "tools"
if (Test-Path -LiteralPath $diagnosticToolsSourceDir) {
    New-Item -ItemType Directory -Path $diagnosticToolsTargetDir -Force | Out-Null
    Get-ChildItem -LiteralPath $diagnosticToolsSourceDir -File | Where-Object {
        $_.Extension.ToLowerInvariant() -in @(".ps1", ".cmd", ".bat", ".py", ".txt", ".md")
    } | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $diagnosticToolsTargetDir $_.Name) -Force
    }
}

$stepSdkSwitchScript = Join-Path $repoRoot "scripts\switch_step_sdk.ps1"
if (Test-Path -LiteralPath $stepSdkSwitchScript) {
    New-Item -ItemType Directory -Path $diagnosticToolsTargetDir -Force | Out-Null
    Copy-Item -LiteralPath $stepSdkSwitchScript -Destination (Join-Path $diagnosticToolsTargetDir "switch_step_sdk.ps1") -Force
}

$installerToolsDir = Join-Path $repoRoot "dist\tools"
New-Item -ItemType Directory -Path $installerToolsDir -Force | Out-Null
foreach ($toolName in @("ConfigMigrate.exe", "ConfigMigrate_Run.cmd")) {
    $toolSource = Join-Path $diagnosticToolsSourceDir $toolName
    if (Test-Path -LiteralPath $toolSource) {
        Copy-Item -LiteralPath $toolSource -Destination (Join-Path $installerToolsDir $toolName) -Force
    }
    else {
        Write-Warning "Database migration installer tool was not found: $toolSource"
    }
}

$fanucSourceDir = Join-Path $repoRoot "SDK\FANUC"
$fanucTargetDir = Join-Path $packageDir "SDK\FANUC"
New-Item -ItemType Directory -Path $fanucTargetDir -Force | Out-Null
# 只拷 SDK\FANUC 顶层的现场所需脚本(.kl/.ls/.pc/.tp 等)。任何子目录(如明图激光宏 mingtu / laser_macros 的源码与编译产物)
# 都是开发交付件、不进现场安装包——故意用 -File(不递归)；勿改成 -Recurse，否则会把这些子目录带进包。
Get-ChildItem -LiteralPath $fanucSourceDir -File | Where-Object {
    $_.Extension.ToLowerInvariant() -in @(".kl", ".ls", ".pc", ".tp", ".var", ".ini", ".txt")
} | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $fanucTargetDir $_.Name) -Force
}

# Ship BCPD (self-contained bcpd.exe + MIT license) for the model-alignment / point-cloud
# denoising feature. Runtime locates it at <root>\SDK\BCPD\bcpd.exe.
$bcpdSourceDir = Join-Path $repoRoot "SDK\BCPD"
$bcpdTargetDir = Join-Path $packageDir "SDK\BCPD"
if (Test-Path -LiteralPath $bcpdSourceDir) {
    New-Item -ItemType Directory -Path $bcpdTargetDir -Force | Out-Null
    Get-ChildItem -LiteralPath $bcpdSourceDir -File | Where-Object {
        $_.Extension.ToLowerInvariant() -in @(".exe", ".md", ".txt")
    } | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $bcpdTargetDir $_.Name) -Force
    }
}

if (-not $SkipFanucCompilerTools) {
    $fanucCompilerSourceDir = Find-FirstExistingPath @(
        "C:\Program Files (x86)\FANUC\WinOLPC\bin",
        "C:\Program Files\FANUC\WinOLPC\bin"
    )

    if ($fanucCompilerSourceDir) {
        $fanucCompilerTargetDir = Join-Path $packageDir "Tools\FANUC\WinOLPC\bin"
        New-Item -ItemType Directory -Path $fanucCompilerTargetDir -Force | Out-Null
        Get-ChildItem -LiteralPath $fanucCompilerSourceDir -File | Where-Object {
            $_.Extension.ToLowerInvariant() -in @(".exe", ".dll", ".ini")
        } | ForEach-Object {
            Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $fanucCompilerTargetDir $_.Name) -Force
        }
    }
    else {
        Write-Warning "FANUC WinOLPC bin directory was not found. FANUC compile tools were not bundled."
    }
}

$redistDir = Join-Path $packageDir "Prerequisites"
New-Item -ItemType Directory -Path $redistDir -Force | Out-Null
if (-not $SkipVcRedistDownload) {
    $vcRedistTarget = Join-Path $redistDir "vc_redist.x64.exe"
    $vcRedistOk = Download-FileIfNeeded -Url "https://aka.ms/vc14/vc_redist.x64.exe" -TargetPath $vcRedistTarget
    if (-not $vcRedistOk) {
        Write-Warning "VC++ runtime installer was not bundled. The target PC may need a manual runtime install."
    }
}

foreach ($runtimeDir in @("Log", "Result", "Temp")) {
    New-Item -ItemType Directory -Path (Join-Path $packageDir $runtimeDir) -Force | Out-Null
}

$notesPath = Join-Path $packageDir "DEPLOY_NOTES.txt"
$notes = @(
    "NoTeaching-Robot deployment notes",
    "",
    "1. This package was generated from the local Release build output.",
    "2. The application writes logs, results and editable config files next to the executable.",
    "3. Because of that, the installer defaults to a user-writable folder instead of Program Files.",
    "4. The installer bundles the Microsoft Visual C++ 2015-2022 Redistributable x64 installer and can run it automatically.",
    "5. The package also bundles FANUC WinOLPC compile tools when they are available on the build PC.",
    "6. Please make sure your FANUC tool redistribution follows your license agreement.",
    "7. SDK\\STEP\\versions is not shipped: STEP SDK .lib archives are link-time only (the SDK is statically linked),",
    "   and the SRS robot-system upgrade package is archived in the source repository and distributed separately.",
    "8. Rebuilds (incl. legacy SDK switch via switch_step_sdk.ps1) are done from the source repository."
)
$notes | Set-Content -LiteralPath $notesPath -Encoding UTF8

Write-Host ""
Write-Host "Release package is ready:"
Write-Host "  $packageDir"
