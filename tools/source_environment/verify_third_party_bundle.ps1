[CmdletBinding()]
param(
    [string]$BundleRoot = "",
    [switch]$SkipFullHash
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($BundleRoot)) {
    $BundleRoot = [System.IO.Path]::GetFullPath((Join-Path $scriptRoot ".."))
} else {
    $BundleRoot = (Resolve-Path -LiteralPath $BundleRoot -ErrorAction Stop).Path
}

$results = New-Object System.Collections.Generic.List[object]
function Add-Result([string]$Check, [ValidateSet("PASS", "FAIL")][string]$Status, [string]$Detail) {
    $results.Add([pscustomobject]@{ Check = $Check; Status = $Status; Detail = $Detail })
}

function Test-RequiredFiles([string]$Name, [string]$Root, [string[]]$RelativePaths) {
    $missing = @()
    foreach ($relative in $RelativePaths) {
        if (-not (Test-Path -LiteralPath (Join-Path $Root $relative) -PathType Leaf)) {
            $missing += $relative
        }
    }
    if ($missing.Count -eq 0) {
        Add-Result $Name "PASS" "$($RelativePaths.Count) required files found"
    } else {
        Add-Result $Name "FAIL" "$($missing.Count) missing: $(($missing | Select-Object -First 5) -join ', ')"
    }
}

$dependencyRoot = Join-Path $BundleRoot "dependencies"
$qtRoot = Join-Path $dependencyRoot "Qt\6.7.3\msvc2022_64"
$qtMsBuild = Join-Path $dependencyRoot "QtMsBuild"
$openCvRoot = Join-Path $dependencyRoot "OpenCV\4.6.0\build"
$eigenRoot = Join-Path $dependencyRoot "Eigen\3.4.0\eigen-3.4.0"
$kdlRoot = Join-Path $dependencyRoot "OrocosKDL\1.5.3\orocos_kdl"
$occtRoot = Join-Path $dependencyRoot "vcpkg\installed\x64-windows"

$qtFiles = @(
    "bin\qt.conf", "bin\qmake.exe", "bin\moc.exe", "bin\rcc.exe", "bin\uic.exe", "bin\windeployqt.exe",
    "bin\Qt6Core.dll", "bin\Qt6Cored.dll", "bin\Qt6Gui.dll", "bin\Qt6Guid.dll",
    "bin\Qt6Widgets.dll", "bin\Qt6Widgetsd.dll", "bin\Qt6Network.dll", "bin\Qt6Networkd.dll",
    "bin\Qt6Sql.dll", "bin\Qt6Sqld.dll", "bin\Qt6OpenGL.dll", "bin\Qt6OpenGLd.dll",
    "bin\Qt6OpenGLWidgets.dll", "bin\Qt6OpenGLWidgetsd.dll", "bin\Qt6Svg.dll", "bin\Qt6Svgd.dll",
    "lib\Qt6Core.lib", "lib\Qt6Cored.lib", "lib\Qt6EntryPoint.lib", "lib\Qt6EntryPointd.lib",
    "plugins\iconengines\qsvgicon.dll", "plugins\iconengines\qsvgicond.dll",
    "plugins\imageformats\qico.dll", "plugins\imageformats\qicod.dll",
    "plugins\imageformats\qjpeg.dll", "plugins\imageformats\qjpegd.dll",
    "plugins\imageformats\qsvg.dll", "plugins\imageformats\qsvgd.dll",
    "plugins\imageformats\qwebp.dll", "plugins\imageformats\qwebpd.dll",
    "plugins\platforms\qwindows.dll", "plugins\platforms\qwindowsd.dll",
    "plugins\sqldrivers\qsqlite.dll", "plugins\sqldrivers\qsqlited.dll",
    "plugins\styles\qmodernwindowsstyle.dll", "plugins\styles\qmodernwindowsstyled.dll",
    "plugins\tls\qcertonlybackend.dll", "plugins\tls\qcertonlybackendd.dll",
    "plugins\tls\qopensslbackend.dll", "plugins\tls\qopensslbackendd.dll",
    "plugins\tls\qschannelbackend.dll", "plugins\tls\qschannelbackendd.dll",
    "translations\catalogs.json", "translations\qt_zh_CN.qm", "translations\qtbase_zh_CN.qm"
)
Test-RequiredFiles "Qt 6.7.3 closure" $qtRoot $qtFiles
Test-RequiredFiles "QtMsBuild" $qtMsBuild @("Qt.props", "qt.targets", "qt_defaults.props")
Test-RequiredFiles "OpenCV 4.6.0" $openCvRoot @(
    "include\opencv2\core\version.hpp", "x64\vc15\lib\opencv_world460.lib", "x64\vc15\lib\opencv_world460d.lib",
    "x64\vc15\bin\opencv_world460.dll", "x64\vc15\bin\opencv_world460d.dll", "LICENSE"
)
Test-RequiredFiles "Eigen 3.4.0" $eigenRoot @("Eigen\Core", "Eigen\src\Core\util\Macros.h", "COPYING.MPL2")
Test-RequiredFiles "Orocos KDL 1.5.3" $kdlRoot @("include\kdl\config.h", "lib\orocos-kdl.lib", "lib\orocos-kdld.lib", "COPYING")
Test-RequiredFiles "OpenCASCADE 7.9.3" $occtRoot @(
    "include\opencascade\STEPControl_Reader.hxx", "include\opencascade\Standard_Version.hxx",
    "lib\TKDESTEP.lib", "debug\lib\TKDESTEP.lib", "bin\TKDESTEP.dll", "debug\bin\TKDESTEP.dll",
    "share\opencascade\copyright"
)

if (Test-Path -LiteralPath (Join-Path $qtRoot "bin\qmake.exe") -PathType Leaf) {
    $actualQt = ((& (Join-Path $qtRoot "bin\qmake.exe") -query QT_VERSION 2>$null) -join "").Trim()
    Add-Result "Qt version" $(if ($actualQt -eq "6.7.3") { "PASS" } else { "FAIL" }) "expected 6.7.3, actual $actualQt"
}

$versionChecks = @(
    @{ Name = "OpenCV version"; File = Join-Path $openCvRoot "include\opencv2\core\version.hpp"; Pattern = '(?ms)#define CV_VERSION_MAJOR\s+(\d+).*?#define CV_VERSION_MINOR\s+(\d+).*?#define CV_VERSION_REVISION\s+(\d+)'; Expected = "4.6.0" },
    @{ Name = "Eigen version"; File = Join-Path $eigenRoot "Eigen\src\Core\util\Macros.h"; Pattern = '(?ms)#define EIGEN_WORLD_VERSION\s+(\d+).*?#define EIGEN_MAJOR_VERSION\s+(\d+).*?#define EIGEN_MINOR_VERSION\s+(\d+)'; Expected = "3.4.0" },
    @{ Name = "Orocos KDL version"; File = Join-Path $kdlRoot "include\kdl\config.h"; Pattern = '(?m)^#define KDL_VERSION_STRING\s+"([^"]+)"'; Expected = "1.5.3" },
    @{ Name = "OpenCASCADE version"; File = Join-Path $occtRoot "include\opencascade\Standard_Version.hxx"; Pattern = '(?m)^#define OCC_VERSION_COMPLETE\s+"([^"]+)"'; Expected = "7.9.3" }
)
foreach ($check in $versionChecks) {
    if (-not (Test-Path -LiteralPath $check.File -PathType Leaf)) { continue }
    $content = Get-Content -LiteralPath $check.File -Raw
    if ($content -match $check.Pattern) {
        $actual = if ($Matches.Count -eq 4) { "$($Matches[1]).$($Matches[2]).$($Matches[3])" } else { $Matches[1] }
        Add-Result $check.Name $(if ($actual -eq $check.Expected) { "PASS" } else { "FAIL" }) "expected $($check.Expected), actual $actual"
    } else {
        Add-Result $check.Name "FAIL" "version marker was not found"
    }
}

$suspicious = @(Get-ChildItem -LiteralPath $BundleRoot -Recurse -File -ErrorAction Stop | Where-Object {
    $_.Name -match '(?i)^(id_rsa|id_ed25519|credentials|secrets?)(\..*)?$|\.(pem|pfx|p12|ppk|key)$|^\.env$'
})
Add-Result "Secret-like filenames" $(if ($suspicious.Count -eq 0) { "PASS" } else { "FAIL" }) $(if ($suspicious.Count -eq 0) { "none found" } else { ($suspicious.FullName -join "; ") })

$manifestPath = Join-Path $BundleRoot "MANIFEST.sha256"
if (-not (Test-Path -LiteralPath $manifestPath -PathType Leaf)) {
    Add-Result "SHA-256 manifest" "FAIL" "missing MANIFEST.sha256"
} elseif ($SkipFullHash) {
    Add-Result "SHA-256 manifest" "PASS" "present; full hash verification skipped by request"
} else {
    $manifestLines = @(Get-Content -LiteralPath $manifestPath | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
    $bad = New-Object System.Collections.Generic.List[string]
    $index = 0
    foreach ($line in $manifestLines) {
        $index++
        if ($line -notmatch '^([0-9a-f]{64})  ([0-9]+)  (.+)$') {
            $bad.Add("invalid line $index")
            continue
        }
        $expectedHash = $Matches[1]
        $expectedBytes = [int64]$Matches[2]
        $relative = $Matches[3].Replace('/', [System.IO.Path]::DirectorySeparatorChar)
        $fullPath = Join-Path $BundleRoot $relative
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            $bad.Add("missing $relative")
            continue
        }
        $item = Get-Item -LiteralPath $fullPath
        if ($item.Length -ne $expectedBytes) {
            $bad.Add("size $relative")
            continue
        }
        $actualHash = (Get-FileHash -LiteralPath $fullPath -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($actualHash -ne $expectedHash) { $bad.Add("hash $relative") }
        if (($index % 500) -eq 0) { Write-Progress -Activity "Verifying dependency bundle" -Status "$index / $($manifestLines.Count)" -PercentComplete (($index * 100.0) / $manifestLines.Count) }
    }
    Write-Progress -Activity "Verifying dependency bundle" -Completed
    Add-Result "SHA-256 manifest" $(if ($bad.Count -eq 0) { "PASS" } else { "FAIL" }) $(if ($bad.Count -eq 0) { "$($manifestLines.Count) files verified" } else { "$($bad.Count) errors: $(($bad | Select-Object -First 5) -join ', ')" })
}

$results | Format-Table -AutoSize -Wrap
$failed = @($results | Where-Object Status -eq "FAIL")
Write-Host "Summary: $($results.Count - $failed.Count) PASS, $($failed.Count) FAIL"
if ($failed.Count -gt 0) { exit 1 }
Write-Host "BUNDLE_VERIFY_OK"
