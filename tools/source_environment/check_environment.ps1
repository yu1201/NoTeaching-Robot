[CmdletBinding()]
param(
    [string]$ProjectRoot = "",
    [string]$SettingsPath = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

$toolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $repoCandidate = [System.IO.Path]::GetFullPath((Join-Path $toolRoot "..\.."))
    $workingCandidate = [System.IO.Path]::GetFullPath((Get-Location).Path)
    if (Test-Path -LiteralPath (Join-Path $repoCandidate "QtWidgetsApplication4.sln") -PathType Leaf) {
        $ProjectRoot = $repoCandidate
    } elseif (Test-Path -LiteralPath (Join-Path $workingCandidate "QtWidgetsApplication4.sln") -PathType Leaf) {
        $ProjectRoot = $workingCandidate
    } else {
        throw "ProjectRoot was not found automatically. Pass -ProjectRoot with the QtWidgetsApplication4 checkout path."
    }
} else {
    $ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot -ErrorAction Stop).Path
}

if ([string]::IsNullOrWhiteSpace($SettingsPath)) {
    $localSettings = Join-Path $toolRoot "environment.local.psd1"
    if (Test-Path -LiteralPath $localSettings -PathType Leaf) {
        $SettingsPath = $localSettings
    } elseif (Test-Path -LiteralPath (Join-Path $toolRoot "..\config\environment.local.psd1") -PathType Leaf) {
        $SettingsPath = Join-Path $toolRoot "..\config\environment.local.psd1"
    } elseif (Test-Path -LiteralPath (Join-Path $toolRoot "environment.example.psd1") -PathType Leaf) {
        $SettingsPath = Join-Path $toolRoot "environment.example.psd1"
    } else {
        $SettingsPath = Join-Path $toolRoot "..\config\environment.example.psd1"
    }
}
$SettingsPath = (Resolve-Path -LiteralPath $SettingsPath -ErrorAction Stop).Path
$settings = Import-PowerShellDataFile -LiteralPath $SettingsPath

function Get-Setting {
    param([Parameter(Mandatory = $true)][string]$Name, [string]$Default = "")
    if ($settings.ContainsKey($Name) -and -not [string]::IsNullOrWhiteSpace([string]$settings[$Name])) {
        return [Environment]::ExpandEnvironmentVariables([string]$settings[$Name])
    }
    return $Default
}

function Resolve-ConfiguredPath {
    param([string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) { return "" }
    $expanded = [Environment]::ExpandEnvironmentVariables($Value)
    if ([System.IO.Path]::IsPathRooted($expanded)) {
        return [System.IO.Path]::GetFullPath($expanded)
    }
    return [System.IO.Path]::GetFullPath((Join-Path $ProjectRoot $expanded))
}

$results = New-Object System.Collections.Generic.List[object]
function Add-Result {
    param([string]$Name, [ValidateSet("PASS", "WARN", "FAIL")][string]$Status, [string]$Detail)
    $results.Add([pscustomobject]@{ Check = $Name; Status = $Status; Detail = $Detail })
}

function Add-FileGroupCheck {
    param(
        [string]$Name,
        [string]$Root,
        [string[]]$RelativePaths,
        [switch]$WarningOnly
    )
    if ([string]::IsNullOrWhiteSpace($Root)) {
        Add-Result $Name $(if ($WarningOnly) { "WARN" } else { "FAIL" }) "root path is empty"
        return
    }
    $missing = @()
    foreach ($relative in $RelativePaths) {
        if (-not (Test-Path -LiteralPath (Join-Path $Root $relative) -PathType Leaf)) {
            $missing += $relative
        }
    }
    if ($missing.Count -eq 0) {
        Add-Result $Name "PASS" "$($RelativePaths.Count) required files found under $Root"
    } else {
        $preview = ($missing | Select-Object -First 5) -join ", "
        if ($missing.Count -gt 5) { $preview += ", ..." }
        Add-Result $Name $(if ($WarningOnly) { "WARN" } else { "FAIL" }) "$($missing.Count) missing: $preview"
    }
}

function Find-MSBuild {
    $configured = Resolve-ConfiguredPath (Get-Setting "MSBuildExecutable")
    if (-not [string]::IsNullOrWhiteSpace($configured)) { return $configured }
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) { return "" }
    $found = @(& $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe" 2>$null)
    if ($found.Count -eq 0) { return "" }
    return [string]$found[0]
}

$qtRoot = Resolve-ConfiguredPath (Get-Setting "QtRoot")
$qtVersion = Get-Setting "QtVersion" "6.7.3"
$qtMsBuild = Resolve-ConfiguredPath (Get-Setting "QtMsBuild" (Join-Path $env:LOCALAPPDATA "QtMsBuild"))
$openCvRoot = Resolve-ConfiguredPath (Get-Setting "OpenCvRoot")
$openCvVersion = Get-Setting "OpenCvVersion" "4.6.0"
$openCvSuffix = Get-Setting "OpenCvVersionSuffix" "460"
$openCvToolset = Get-Setting "OpenCvToolsetDir" "vc15"
$eigenRoot = Resolve-ConfiguredPath (Get-Setting "EigenRoot")
$eigenVersion = Get-Setting "EigenVersion" "3.4.0"
$kdlRoot = Resolve-ConfiguredPath (Get-Setting "OrocosKdlRoot")
$kdlVersion = Get-Setting "OrocosKdlVersion" "1.5.3"
$vcpkgRoot = Resolve-ConfiguredPath (Get-Setting "VcpkgRoot")
$occtRoot = Resolve-ConfiguredPath (Get-Setting "OcctRoot")
if ([string]::IsNullOrWhiteSpace($occtRoot) -and -not [string]::IsNullOrWhiteSpace($vcpkgRoot)) {
    $occtRoot = Join-Path $vcpkgRoot "installed\x64-windows"
}
$occtVersion = Get-Setting "OcctVersion" "7.9.3"
$msbuild = Find-MSBuild

if ($PSVersionTable.PSVersion.Major -ge 5) {
    Add-Result "PowerShell" "PASS" $PSVersionTable.PSVersion.ToString()
} else {
    Add-Result "PowerShell" "FAIL" "PowerShell 5.1 or newer is required"
}

$gitCommand = Get-Command git.exe -ErrorAction SilentlyContinue
if ($null -eq $gitCommand) {
    Add-Result "Git" "FAIL" "git.exe was not found on PATH"
} else {
    $gitVersion = (& $gitCommand.Source --version 2>$null) -join " "
    Add-Result "Git" "PASS" $gitVersion
}

if (Test-Path -LiteralPath $msbuild -PathType Leaf) {
    $msbuildVersion = @(& $msbuild -version -nologo 2>$null) | Select-Object -Last 1
    Add-Result "Visual Studio/MSBuild" "PASS" "$msbuildVersion | $msbuild"
} else {
    Add-Result "Visual Studio/MSBuild" "FAIL" "MSBuild.exe was not found through settings or vswhere"
}

$windowsSdkBase = ""
$windowsSdkRegistry = "HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots"
if (Test-Path -LiteralPath $windowsSdkRegistry) {
    $windowsSdkBase = [string](Get-ItemPropertyValue -LiteralPath $windowsSdkRegistry -Name "KitsRoot10" -ErrorAction SilentlyContinue)
}
if ([string]::IsNullOrWhiteSpace($windowsSdkBase)) {
    $windowsSdkBase = "${env:ProgramFiles(x86)}\Windows Kits\10"
}
$windowsSdkRoot = Join-Path $windowsSdkBase "Include"
$windowsSdkVersions = @()
if (Test-Path -LiteralPath $windowsSdkRoot -PathType Container) {
    $windowsSdkVersions = @(Get-ChildItem -LiteralPath $windowsSdkRoot -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending)
}
if ($windowsSdkVersions.Count -gt 0) {
    Add-Result "Windows SDK" "PASS" $windowsSdkVersions[0].Name
} else {
    Add-Result "Windows SDK" "FAIL" "No Windows 10/11 SDK include directory was found"
}

Add-FileGroupCheck "Qt $qtVersion" $qtRoot @(
    "bin\qmake.exe", "bin\windeployqt.exe", "bin\Qt6Core.dll", "bin\Qt6Cored.dll",
    "lib\Qt6Core.lib", "lib\Qt6Cored.lib", "lib\Qt6Network.lib", "lib\Qt6Networkd.lib",
    "lib\Qt6Sql.lib", "lib\Qt6Sqld.lib", "lib\Qt6OpenGL.lib", "lib\Qt6OpenGLd.lib",
    "lib\Qt6OpenGLWidgets.lib", "lib\Qt6OpenGLWidgetsd.lib",
    "plugins\sqldrivers\qsqlite.dll", "plugins\tls\qschannelbackend.dll",
    "translations\qt_zh_CN.qm", "translations\qtbase_zh_CN.qm"
)
if (Test-Path -LiteralPath (Join-Path $qtRoot "bin\qmake.exe") -PathType Leaf) {
    $actualQt = ((& (Join-Path $qtRoot "bin\qmake.exe") -query QT_VERSION 2>$null) -join "").Trim()
    Add-Result "Qt version" $(if ($actualQt -eq $qtVersion) { "PASS" } else { "FAIL" }) "expected $qtVersion, actual $actualQt"
}
Add-FileGroupCheck "QtMsBuild" $qtMsBuild @("Qt.props", "qt.targets", "qt_defaults.props")

$opencvRel = "x64\$openCvToolset"
Add-FileGroupCheck "OpenCV $openCvVersion" $openCvRoot @(
    "include\opencv2\core\version.hpp",
    "$opencvRel\lib\opencv_world$openCvSuffix.lib",
    "$opencvRel\lib\opencv_world$($openCvSuffix)d.lib",
    "$opencvRel\bin\opencv_world$openCvSuffix.dll",
    "$opencvRel\bin\opencv_world$($openCvSuffix)d.dll"
)
$opencvHeader = Join-Path $openCvRoot "include\opencv2\core\version.hpp"
if (Test-Path -LiteralPath $opencvHeader -PathType Leaf) {
    $header = Get-Content -LiteralPath $opencvHeader -Raw
    $actualOpenCv = ""
    if ($header -match '(?m)^#define CV_VERSION_MAJOR\s+(\d+)' ) { $major = $Matches[1] } else { $major = "?" }
    if ($header -match '(?m)^#define CV_VERSION_MINOR\s+(\d+)' ) { $minor = $Matches[1] } else { $minor = "?" }
    if ($header -match '(?m)^#define CV_VERSION_REVISION\s+(\d+)' ) { $patch = $Matches[1] } else { $patch = "?" }
    $actualOpenCv = "$major.$minor.$patch"
    Add-Result "OpenCV version" $(if ($actualOpenCv -eq $openCvVersion) { "PASS" } else { "FAIL" }) "expected $openCvVersion, actual $actualOpenCv"
}

Add-FileGroupCheck "Eigen $eigenVersion" $eigenRoot @("Eigen\Core", "Eigen\src\Core\util\Macros.h")
$eigenHeader = Join-Path $eigenRoot "Eigen\src\Core\util\Macros.h"
if (Test-Path -LiteralPath $eigenHeader -PathType Leaf) {
    $header = Get-Content -LiteralPath $eigenHeader -Raw
    $parts = @()
    foreach ($name in @("WORLD", "MAJOR", "MINOR")) {
        if ($header -match "(?m)^#define EIGEN_${name}_VERSION\s+(\d+)") { $parts += $Matches[1] } else { $parts += "?" }
    }
    $actualEigen = $parts -join "."
    Add-Result "Eigen version" $(if ($actualEigen -eq $eigenVersion) { "PASS" } else { "FAIL" }) "expected $eigenVersion, actual $actualEigen"
}

Add-FileGroupCheck "Orocos KDL $kdlVersion" $kdlRoot @("include\kdl\config.h", "lib\orocos-kdl.lib", "lib\orocos-kdld.lib")
$kdlHeader = Join-Path $kdlRoot "include\kdl\config.h"
if (Test-Path -LiteralPath $kdlHeader -PathType Leaf) {
    $header = Get-Content -LiteralPath $kdlHeader -Raw
    $actualKdl = if ($header -match '(?m)^#define KDL_VERSION_STRING\s+"([^"]+)"') { $Matches[1] } else { "unknown" }
    Add-Result "Orocos KDL version" $(if ($actualKdl -eq $kdlVersion) { "PASS" } else { "FAIL" }) "expected $kdlVersion, actual $actualKdl"
}

$occtImports = @("TKDESTEP", "TKXSBase", "TKXCAF", "TKVCAF", "TKCAF", "TKLCAF", "TKCDF", "TKMesh", "TKBO", "TKBool", "TKShHealing", "TKGeomAlgo", "TKGeomBase", "TKTopAlgo", "TKBRep", "TKG2d", "TKG3d", "TKMath", "TKernel", "TKPrim", "TKService", "TKV3d", "TKOpenGl")
$occtRuntime = @("TKBO", "TKBool", "TKBRep", "TKCAF", "TKCDF", "TKDE", "TKDESTEP", "TKernel", "TKG2d", "TKG3d", "TKGeomAlgo", "TKGeomBase", "TKHLR", "TKLCAF", "TKMath", "TKMesh", "TKOpenGl", "TKPrim", "TKService", "TKShHealing", "TKTopAlgo", "TKV3d", "TKVCAF", "TKXCAF", "TKXSBase")
$occtFiles = @("include\opencascade\STEPControl_Reader.hxx", "include\opencascade\Standard_Version.hxx", "share\opencascade\copyright")
foreach ($name in $occtImports) {
    $occtFiles += "lib\$name.lib"
    $occtFiles += "debug\lib\$name.lib"
}
foreach ($name in $occtRuntime) {
    $occtFiles += "bin\$name.dll"
    $occtFiles += "debug\bin\$name.dll"
}
Add-FileGroupCheck "OpenCASCADE $occtVersion" $occtRoot $occtFiles
$occtHeader = Join-Path $occtRoot "include\opencascade\Standard_Version.hxx"
if (Test-Path -LiteralPath $occtHeader -PathType Leaf) {
    $header = Get-Content -LiteralPath $occtHeader -Raw
    $actualOcct = if ($header -match '(?m)^#define OCC_VERSION_COMPLETE\s+"([^"]+)"') { $Matches[1] } else { "unknown" }
    Add-Result "OpenCASCADE version" $(if ($actualOcct -eq $occtVersion) { "PASS" } else { "FAIL" }) "expected $occtVersion, actual $actualOcct"
}

Add-FileGroupCheck "Project files" $ProjectRoot @("QtWidgetsApplication4.sln", "QtWidgetsApplication4.vcxproj", "QtWidgetsApplication4.qrc")
Add-FileGroupCheck "Tracked project SDK" $ProjectRoot @(
    "SDK\STEP\ControlMsgData.hpp", "SDK\STEP\EasyTcpClient.hpp", "SDK\STEP\RobotCom.hpp",
    "SDK\STEP\Robot-SDK.lib", "SDK\STEP\Robot-SDKd.lib",
    "SDK\PointCloudExtration\PointCloudExtration.dll", "SDK\PointCloudExtration\PointCloudExtration.lib",
    "SDK\PointCloudExtration\PointCloudExtration.h", "SDK\PointCloudExtration\config\CorrugatedSheetPointCloudEctration.ini",
    "SDK\SKJCamera\include\skjcamera.h", "SDK\SKJCamera\lib\x64\Debug\SKJCamera.lib",
    "SDK\SKJCamera\lib\x64\Release\SKJCamera.lib", "SDK\SKJCamera\bin\x64\Debug\SKJCamera.dll",
    "SDK\SKJCamera\bin\x64\Release\SKJCamera.dll", "SDK\BCPD\bcpd.exe", "SDK\BCPD\LICENSE.md"
)

$fanucRoot = Join-Path $ProjectRoot "SDK\FANUC"
$tpCount = @(Get-ChildItem -LiteralPath $fanucRoot -Recurse -File -Filter "*.tp" -ErrorAction SilentlyContinue).Count
$pcCount = @(Get-ChildItem -LiteralPath $fanucRoot -Recurse -File -Filter "*.pc" -ErrorAction SilentlyContinue).Count
if ($tpCount -gt 0 -and $pcCount -gt 0) {
    Add-Result "FANUC release runtime" "PASS" "$tpCount .tp and $pcCount .pc files found; formal release still requires authority/hash gates"
} else {
    Add-Result "FANUC release runtime" "WARN" "ordinary source build is allowed; formal packaging requires authoritative .tp/.pc files"
}

if ($null -ne $gitCommand -and (Test-Path -LiteralPath (Join-Path $ProjectRoot ".git"))) {
    $status = @(& $gitCommand.Source -c "safe.directory=$($ProjectRoot.Replace('\', '/'))" -C $ProjectRoot status --porcelain 2>$null)
    if ($LASTEXITCODE -eq 0) {
        Add-Result "Git worktree" $(if ($status.Count -eq 0) { "PASS" } else { "WARN" }) $(if ($status.Count -eq 0) { "clean" } else { "$($status.Count) changed/untracked entries" })
    } else {
        Add-Result "Git worktree" "WARN" "git status could not be read"
    }
}

Write-Host "ProjectRoot : $ProjectRoot"
Write-Host "Settings    : $SettingsPath"
$results | Format-Table -AutoSize -Wrap
$failures = @($results | Where-Object Status -eq "FAIL").Count
$warnings = @($results | Where-Object Status -eq "WARN").Count
Write-Host "SUMMARY: PASS=$(@($results | Where-Object Status -eq 'PASS').Count) WARN=$warnings FAIL=$failures"
if ($failures -gt 0) { exit 1 }
exit 0
