[CmdletBinding()]
param(
    [string]$ProjectRoot = "",
    [string]$OutputRoot = "",
    [string]$QtRoot = "E:\workspace\soft\QT\6.7.3\msvc2022_64",
    [string]$QtLicenseRoot = "E:\workspace\soft\QT\Licenses",
    [string]$QtMsBuildRoot = "$env:LOCALAPPDATA\QtMsBuild",
    [string]$OpenCvRoot = "E:\OpenCV4.6.0\build",
    [string]$EigenRoot = "E:\Eigen3.4\eigen-3.4.0",
    [string]$OrocosKdlRoot = "C:\Program Files\orocos_kdl",
    [string]$OcctInstalledRoot = "E:\vcpkg\installed\x64-windows",
    [string]$KdlLicensePath = "",
    [double]$MinimumFreeGiB = 8.0,
    [switch]$ReplaceExistingGeneratedKit
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

$toolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $ProjectRoot = [System.IO.Path]::GetFullPath((Join-Path $toolRoot "..\.."))
} else {
    $ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot -ErrorAction Stop).Path
}
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $ProjectRoot "output"
} else {
    $OutputRoot = [System.IO.Path]::GetFullPath($OutputRoot)
}

$dateStamp = Get-Date -Format "yyyyMMdd"
$kitName = "QtWidgetsApplication4-third-party-dependencies-$dateStamp"
$bundleRoot = Join-Path $OutputRoot $kitName
$archivePath = Join-Path $OutputRoot "$kitName.zip"
$existingTargets = @($bundleRoot, $archivePath) | Where-Object { Test-Path -LiteralPath $_ }
if ($existingTargets.Count -gt 0 -and -not $ReplaceExistingGeneratedKit) {
    throw "Generated kit already exists; refusing to overwrite: $($existingTargets -join ', ')"
}
if ($ReplaceExistingGeneratedKit) {
    $expectedOutputPrefix = [System.IO.Path]::GetFullPath($OutputRoot).TrimEnd('\') + '\'
    foreach ($target in $existingTargets) {
        $fullTarget = [System.IO.Path]::GetFullPath($target)
        if (-not $fullTarget.StartsWith($expectedOutputPrefix, [System.StringComparison]::OrdinalIgnoreCase) -or
            [System.IO.Path]::GetFileNameWithoutExtension($fullTarget) -ne $kitName) {
            throw "Refusing to replace unexpected target: $fullTarget"
        }
        Remove-Item -LiteralPath $fullTarget -Recurse -Force
    }
}

$sources = [ordered]@{
    Qt = $QtRoot
    QtLicenses = $QtLicenseRoot
    QtMsBuild = $QtMsBuildRoot
    OpenCV = $OpenCvRoot
    Eigen = $EigenRoot
    OrocosKDL = $OrocosKdlRoot
    OpenCASCADE = $OcctInstalledRoot
}
foreach ($entry in $sources.GetEnumerator()) {
    if (-not (Test-Path -LiteralPath $entry.Value -PathType Container)) {
        throw "$($entry.Key) source directory is missing: $($entry.Value)"
    }
}
foreach ($relative in @(
    "docs\third-party-offline-package.md",
    "tools\source_environment\dependency-lock.json",
    "tools\source_environment\check_environment.ps1",
    "tools\source_environment\build_local.ps1",
    "tools\source_environment\verify_third_party_bundle.ps1",
    "tools\source_environment\configure_project_from_bundle.ps1"
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $ProjectRoot $relative) -PathType Leaf)) {
        throw "Required kit source file is missing: $relative"
    }
}

[System.IO.Directory]::CreateDirectory($OutputRoot) | Out-Null
$driveRoot = [System.IO.Path]::GetPathRoot($OutputRoot)
$drive = Get-CimInstance Win32_LogicalDisk -Filter "DeviceID='$($driveRoot.TrimEnd('\'))'"
if ($null -eq $drive) { throw "Unable to inspect free space for $driveRoot" }
$freeGiB = [math]::Round(($drive.FreeSpace / 1GB), 2)
if ($freeGiB -lt $MinimumFreeGiB) { throw "Only $freeGiB GiB is free on $driveRoot; at least $MinimumFreeGiB GiB is required." }
Write-Host "[1/8] Preflight OK - $freeGiB GiB free on $driveRoot"

function Copy-Tree([string]$Source, [string]$Destination) {
    [System.IO.Directory]::CreateDirectory($Destination) | Out-Null
    & "$env:SystemRoot\System32\robocopy.exe" $Source $Destination /E /COPY:DAT /DCOPY:DAT /R:2 /W:2 /NFL /NDL /NJH /NJS /NP | Out-Null
    $code = $LASTEXITCODE
    if ($code -gt 7) { throw "robocopy failed with exit code ${code}: $Source -> $Destination" }
}

function Copy-One([string]$SourceRoot, [string]$RelativePath, [string]$DestinationRoot) {
    $source = Join-Path $SourceRoot $RelativePath
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw "Required source file is missing: $source" }
    $destination = Join-Path $DestinationRoot $RelativePath
    [System.IO.Directory]::CreateDirectory((Split-Path -Parent $destination)) | Out-Null
    Copy-Item -LiteralPath $source -Destination $destination
}

function Get-TreeStat([string]$Root) {
    $files = @(Get-ChildItem -LiteralPath $Root -Recurse -File -ErrorAction Stop)
    $bytes = ($files | Measure-Object -Property Length -Sum).Sum
    if ($null -eq $bytes) { $bytes = 0 }
    return [pscustomobject]@{ Files = $files.Count; Bytes = [int64]$bytes }
}

[System.IO.Directory]::CreateDirectory($bundleRoot) | Out-Null
[System.IO.Directory]::CreateDirectory((Join-Path $bundleRoot "config")) | Out-Null
[System.IO.Directory]::CreateDirectory((Join-Path $bundleRoot "scripts")) | Out-Null
[System.IO.Directory]::CreateDirectory((Join-Path $bundleRoot "dependencies")) | Out-Null
Copy-Item -LiteralPath (Join-Path $ProjectRoot "docs\third-party-offline-package.md") -Destination (Join-Path $bundleRoot "README.md")
Copy-Item -LiteralPath (Join-Path $ProjectRoot "tools\source_environment\dependency-lock.json") -Destination (Join-Path $bundleRoot "config\dependency-lock.json")
foreach ($name in @("check_environment.ps1", "build_local.ps1", "verify_third_party_bundle.ps1", "configure_project_from_bundle.ps1")) {
    Copy-Item -LiteralPath (Join-Path $ProjectRoot "tools\source_environment\$name") -Destination (Join-Path $bundleRoot "scripts\$name")
}
Write-Host "[2/8] Kit documentation and scripts copied"

$qtDestination = Join-Path $bundleRoot "dependencies\Qt\6.7.3\msvc2022_64"
foreach ($directory in @("include", "mkspecs", "metatypes", "modules")) {
    Copy-Tree (Join-Path $QtRoot $directory) (Join-Path $qtDestination $directory)
}
$qtBinFiles = @(
    "qt.conf", "qmake.exe", "qmake6.exe", "moc.exe", "rcc.exe", "uic.exe", "windeployqt.exe", "windeployqt6.exe", "qtpaths.exe", "qtpaths6.exe",
    "d3dcompiler_47.dll", "opengl32sw.dll",
    "Qt6Core.dll", "Qt6Cored.dll", "Qt6Gui.dll", "Qt6Guid.dll", "Qt6Widgets.dll", "Qt6Widgetsd.dll",
    "Qt6Network.dll", "Qt6Networkd.dll", "Qt6Sql.dll", "Qt6Sqld.dll", "Qt6OpenGL.dll", "Qt6OpenGLd.dll",
    "Qt6OpenGLWidgets.dll", "Qt6OpenGLWidgetsd.dll", "Qt6Svg.dll", "Qt6Svgd.dll", "Qt6SvgWidgets.dll", "Qt6SvgWidgetsd.dll"
)
foreach ($name in $qtBinFiles) { Copy-One $QtRoot "bin\$name" $qtDestination }
$qtModules = @("Core", "Gui", "Widgets", "Network", "Sql", "OpenGL", "OpenGLWidgets", "Svg", "SvgWidgets", "EntryPoint")
foreach ($module in $qtModules) {
    foreach ($suffix in @("", "d")) {
        foreach ($extension in @("lib", "prl")) {
            $relative = "lib\Qt6$module$suffix.$extension"
            if (Test-Path -LiteralPath (Join-Path $QtRoot $relative) -PathType Leaf) { Copy-One $QtRoot $relative $qtDestination }
        }
    }
}
$qtPlugins = @(
    "iconengines\qsvgicon.dll", "iconengines\qsvgicond.dll",
    "imageformats\qico.dll", "imageformats\qicod.dll", "imageformats\qjpeg.dll", "imageformats\qjpegd.dll",
    "imageformats\qsvg.dll", "imageformats\qsvgd.dll", "imageformats\qwebp.dll", "imageformats\qwebpd.dll",
    "platforms\qwindows.dll", "platforms\qwindowsd.dll", "sqldrivers\qsqlite.dll", "sqldrivers\qsqlited.dll",
    "styles\qmodernwindowsstyle.dll", "styles\qmodernwindowsstyled.dll",
    "tls\qcertonlybackend.dll", "tls\qcertonlybackendd.dll", "tls\qopensslbackend.dll", "tls\qopensslbackendd.dll",
    "tls\qschannelbackend.dll", "tls\qschannelbackendd.dll"
)
foreach ($plugin in $qtPlugins) { Copy-One $QtRoot "plugins\$plugin" $qtDestination }
foreach ($translation in @("catalogs.json", "qt_zh_CN.qm", "qtbase_zh_CN.qm")) { Copy-One $QtRoot "translations\$translation" $qtDestination }
Copy-Tree $QtLicenseRoot (Join-Path $bundleRoot "dependencies\Qt\Licenses")
Write-Host "[3/8] Trimmed Qt 6.7.3 link/deploy closure copied"

Copy-Tree $QtMsBuildRoot (Join-Path $bundleRoot "dependencies\QtMsBuild")
Copy-Tree $OpenCvRoot (Join-Path $bundleRoot "dependencies\OpenCV\4.6.0\build")
Copy-Tree $EigenRoot (Join-Path $bundleRoot "dependencies\Eigen\3.4.0\eigen-3.4.0")
Copy-Tree $OrocosKdlRoot (Join-Path $bundleRoot "dependencies\OrocosKDL\1.5.3\orocos_kdl")
Copy-Tree $OcctInstalledRoot (Join-Path $bundleRoot "dependencies\vcpkg\installed\x64-windows")
Write-Host "[4/8] QtMsBuild, OpenCV, Eigen, KDL and OpenCASCADE trees copied"

$kdlLicenseDestination = Join-Path $bundleRoot "dependencies\OrocosKDL\1.5.3\orocos_kdl\COPYING"
if (-not [string]::IsNullOrWhiteSpace($KdlLicensePath)) {
    Copy-Item -LiteralPath (Resolve-Path -LiteralPath $KdlLicensePath -ErrorAction Stop).Path -Destination $kdlLicenseDestination
} else {
    $licenseUri = "https://raw.githubusercontent.com/orocos/orocos_kinematics_dynamics/master/orocos_kdl/COPYING"
    Invoke-WebRequest -UseBasicParsing -Uri $licenseUri -OutFile $kdlLicenseDestination
}
$kdlLicenseText = Get-Content -LiteralPath $kdlLicenseDestination -Raw
if ($kdlLicenseText -notmatch 'GNU LESSER GENERAL PUBLIC LICENSE' -or $kdlLicenseText -notmatch 'Version 2\.1') {
    throw "Downloaded/provided Orocos KDL COPYING file did not pass the license marker check."
}
Write-Host "[5/8] Third-party license set completed"

$suspicious = @(Get-ChildItem -LiteralPath $bundleRoot -Recurse -File -ErrorAction Stop | Where-Object {
    $_.Name -match '(?i)^(id_rsa|id_ed25519|credentials|secrets?)(\..*)?$|\.(pem|pfx|p12|ppk|key)$|^\.env$'
})
if ($suspicious.Count -gt 0) { throw "Secret-like filenames found in bundle: $($suspicious.FullName -join '; ')" }

$componentSpecs = @(
    @{ Component = "Qt"; Version = "6.7.3"; Source = $QtRoot; Bundle = $qtDestination; Selection = "Project link/deploy closure; WebEngine, QML, unrelated modules and PDB files omitted" },
    @{ Component = "QtMsBuild"; Version = "local verified tree"; Source = $QtMsBuildRoot; Bundle = Join-Path $bundleRoot "dependencies\QtMsBuild"; Selection = "Complete tree" },
    @{ Component = "OpenCV"; Version = "4.6.0"; Source = $OpenCvRoot; Bundle = Join-Path $bundleRoot "dependencies\OpenCV\4.6.0\build"; Selection = "Complete build tree" },
    @{ Component = "Eigen"; Version = "3.4.0"; Source = $EigenRoot; Bundle = Join-Path $bundleRoot "dependencies\Eigen\3.4.0\eigen-3.4.0"; Selection = "Complete tree" },
    @{ Component = "Orocos KDL"; Version = "1.5.3"; Source = $OrocosKdlRoot; Bundle = Join-Path $bundleRoot "dependencies\OrocosKDL\1.5.3\orocos_kdl"; Selection = "Complete installed tree plus official COPYING" },
    @{ Component = "OpenCASCADE/vcpkg installed"; Version = "7.9.3/x64-windows"; Source = $OcctInstalledRoot; Bundle = Join-Path $bundleRoot "dependencies\vcpkg\installed\x64-windows"; Selection = "Complete installed tree" }
)
$componentRecords = @()
foreach ($spec in $componentSpecs) {
    $sourceStat = Get-TreeStat $spec.Source
    $bundleStat = Get-TreeStat $spec.Bundle
    $componentRecords += [ordered]@{
        component = $spec.Component
        version = $spec.Version
        selection = $spec.Selection
        sourceFiles = $sourceStat.Files
        sourceBytes = $sourceStat.Bytes
        bundleFiles = $bundleStat.Files
        bundleBytes = $bundleStat.Bytes
    }
}
$bundleMetadata = [ordered]@{
    schemaVersion = 1
    project = "QtWidgetsApplication4"
    generatedDate = (Get-Date -Format "yyyy-MM-dd")
    platform = "Windows x64"
    containsCredentials = $false
    visualStudioIncluded = $false
    gitIncluded = $false
    components = $componentRecords
}
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
[System.IO.File]::WriteAllText((Join-Path $bundleRoot "bundle-source.json"), ($bundleMetadata | ConvertTo-Json -Depth 6), $utf8NoBom)
Write-Host "[6/8] Audit metadata created; no secret-like filenames found"

$manifestPath = Join-Path $bundleRoot "MANIFEST.sha256"
$files = @(Get-ChildItem -LiteralPath $bundleRoot -Recurse -File -ErrorAction Stop | Where-Object FullName -ne $manifestPath | Sort-Object FullName)
$manifestLines = New-Object System.Collections.Generic.List[string]
$index = 0
foreach ($file in $files) {
    $index++
    $relative = $file.FullName.Substring($bundleRoot.Length + 1).Replace('\', '/')
    $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    $manifestLines.Add("$hash  $($file.Length)  $relative")
    if (($index % 500) -eq 0) { Write-Progress -Activity "Hashing dependency bundle" -Status "$index / $($files.Count)" -PercentComplete (($index * 100.0) / $files.Count) }
}
Write-Progress -Activity "Hashing dependency bundle" -Completed
[System.IO.File]::WriteAllLines($manifestPath, $manifestLines, $utf8NoBom)
Write-Host "[7/8] SHA-256 manifest created for $($files.Count) files"

$tar = Join-Path $env:SystemRoot "System32\tar.exe"
if (-not (Test-Path -LiteralPath $tar -PathType Leaf)) { throw "Windows tar.exe was not found: $tar" }
& $tar -a -cf $archivePath -C $OutputRoot $kitName
if ($LASTEXITCODE -ne 0) { throw "ZIP creation failed with exit code $LASTEXITCODE" }
if (-not (Test-Path -LiteralPath $archivePath -PathType Leaf)) { throw "ZIP was not created: $archivePath" }
$archive = Get-Item -LiteralPath $archivePath
$archiveHash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
Write-Host "[8/8] ZIP created"
Write-Host "THIRD_PARTY_KIT_OK"
Write-Host "Staging : $bundleRoot"
Write-Host "Archive : $archivePath"
Write-Host "Bytes   : $($archive.Length)"
Write-Host "SHA256  : $archiveHash"
