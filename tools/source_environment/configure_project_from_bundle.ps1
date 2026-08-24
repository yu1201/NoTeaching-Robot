[CmdletBinding()]
param(
    [string]$ProjectRoot = "",
    [string]$BundleRoot = "",
    [switch]$Force
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

if ([string]::IsNullOrWhiteSpace($ProjectRoot)) {
    $candidate = [System.IO.Path]::GetFullPath((Get-Location).Path)
    if (Test-Path -LiteralPath (Join-Path $candidate "QtWidgetsApplication4.sln") -PathType Leaf) {
        $ProjectRoot = $candidate
    } else {
        throw "ProjectRoot was not found automatically. Pass the source checkout with -ProjectRoot."
    }
} else {
    $ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot -ErrorAction Stop).Path
}

if (-not (Test-Path -LiteralPath (Join-Path $ProjectRoot "QtWidgetsApplication4.sln") -PathType Leaf)) {
    throw "Not a QtWidgetsApplication4 source checkout: $ProjectRoot"
}

$dependencyRoot = Join-Path $BundleRoot "dependencies"
$paths = [ordered]@{
    QtRoot        = Join-Path $dependencyRoot "Qt\6.7.3\msvc2022_64"
    QtMsBuild     = Join-Path $dependencyRoot "QtMsBuild"
    OpenCvRoot    = Join-Path $dependencyRoot "OpenCV\4.6.0\build"
    EigenRoot     = Join-Path $dependencyRoot "Eigen\3.4.0\eigen-3.4.0"
    OrocosKdlRoot = Join-Path $dependencyRoot "OrocosKDL\1.5.3\orocos_kdl"
    VcpkgRoot     = Join-Path $dependencyRoot "vcpkg"
    OcctRoot      = Join-Path $dependencyRoot "vcpkg\installed\x64-windows"
}

$required = @(
    (Join-Path $paths.QtRoot "bin\qmake.exe"),
    (Join-Path $paths.QtMsBuild "qt.targets"),
    (Join-Path $paths.OpenCvRoot "x64\vc15\lib\opencv_world460.lib"),
    (Join-Path $paths.EigenRoot "Eigen\Core"),
    (Join-Path $paths.OrocosKdlRoot "lib\orocos-kdl.lib"),
    (Join-Path $paths.OcctRoot "include\opencascade\Standard_Version.hxx")
)
foreach ($item in $required) {
    if (-not (Test-Path -LiteralPath $item -PathType Leaf)) {
        throw "Bundle is incomplete; required file is missing: $item"
    }
}

function ConvertTo-XmlText([string]$Value) {
    return [System.Security.SecurityElement]::Escape($Value)
}

function ConvertTo-PowerShellLiteral([string]$Value) {
    return "'" + $Value.Replace("'", "''") + "'"
}

$propsPath = Join-Path $ProjectRoot "environment.local.props"
$psd1Path = Join-Path $ProjectRoot "tools\source_environment\environment.local.psd1"
foreach ($target in @($propsPath, $psd1Path)) {
    if ((Test-Path -LiteralPath $target) -and -not $Force) {
        throw "Refusing to overwrite existing local configuration: $target. Re-run with -Force after reviewing it."
    }
}

$props = @"
<?xml version="1.0" encoding="utf-8"?>
<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <!-- Generated from the audited offline dependency bundle. Local paths only; do not commit. -->
  <PropertyGroup>
    <QtRoot>$(ConvertTo-XmlText $paths.QtRoot)</QtRoot>
    <QtVersion>6.7.3</QtVersion>
    <QtMsBuild>$(ConvertTo-XmlText $paths.QtMsBuild)</QtMsBuild>
    <QtInstall>`$(QtRoot)</QtInstall>
    <OpenCvRoot>$(ConvertTo-XmlText $paths.OpenCvRoot)</OpenCvRoot>
    <OpenCvVersionSuffix>460</OpenCvVersionSuffix>
    <OpenCvToolsetDir>vc15</OpenCvToolsetDir>
    <EigenRoot>$(ConvertTo-XmlText $paths.EigenRoot)</EigenRoot>
    <OrocosKdlRoot>$(ConvertTo-XmlText $paths.OrocosKdlRoot)</OrocosKdlRoot>
    <VcpkgRoot>$(ConvertTo-XmlText $paths.VcpkgRoot)</VcpkgRoot>
    <OcctRoot>$(ConvertTo-XmlText $paths.OcctRoot)</OcctRoot>
  </PropertyGroup>
</Project>
"@

$psd1 = @"
@{
    # Generated from the audited offline dependency bundle. Local paths only; do not commit.
    MSBuildExecutable    = ''
    QtRoot              = $(ConvertTo-PowerShellLiteral $paths.QtRoot)
    QtVersion           = '6.7.3'
    QtMsBuild           = $(ConvertTo-PowerShellLiteral $paths.QtMsBuild)
    OpenCvRoot          = $(ConvertTo-PowerShellLiteral $paths.OpenCvRoot)
    OpenCvVersion       = '4.6.0'
    OpenCvVersionSuffix = '460'
    OpenCvToolsetDir    = 'vc15'
    EigenRoot           = $(ConvertTo-PowerShellLiteral $paths.EigenRoot)
    EigenVersion        = '3.4.0'
    OrocosKdlRoot       = $(ConvertTo-PowerShellLiteral $paths.OrocosKdlRoot)
    OrocosKdlVersion    = '1.5.3'
    VcpkgRoot           = $(ConvertTo-PowerShellLiteral $paths.VcpkgRoot)
    OcctRoot            = $(ConvertTo-PowerShellLiteral $paths.OcctRoot)
    OcctVersion         = '7.9.3'
}
"@

$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
[System.IO.File]::WriteAllText($propsPath, $props.TrimStart(), $utf8NoBom)
[System.IO.Directory]::CreateDirectory((Split-Path -Parent $psd1Path)) | Out-Null
[System.IO.File]::WriteAllText($psd1Path, $psd1.TrimStart(), $utf8NoBom)

Write-Host "CONFIGURE_OK"
Write-Host "Bundle : $BundleRoot"
Write-Host "Project: $ProjectRoot"
Write-Host "Created: $propsPath"
Write-Host "Created: $psd1Path"
Write-Host "Next   : powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\check_environment.ps1"
