[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")][string]$Configuration = "Debug",
    [ValidateSet("Build", "Rebuild")][string]$Target = "Build",
    [string]$ProjectRoot = "",
    [string]$SettingsPath = "",
    [switch]$SkipEnvironmentCheck
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

if (-not $SkipEnvironmentCheck) {
    $checkScript = Join-Path $toolRoot "check_environment.ps1"
    $windowsPowerShell = Join-Path $env:SystemRoot "System32\WindowsPowerShell\v1.0\powershell.exe"
    & $windowsPowerShell -NoProfile -ExecutionPolicy Bypass -File $checkScript -ProjectRoot $ProjectRoot -SettingsPath $SettingsPath
    if ($LASTEXITCODE -ne 0) {
        throw "Environment check failed with exit code $LASTEXITCODE."
    }
}

$settings = Import-PowerShellDataFile -LiteralPath $SettingsPath
function Get-Setting {
    param([string]$Name, [string]$Default = "")
    if ($settings.ContainsKey($Name) -and -not [string]::IsNullOrWhiteSpace([string]$settings[$Name])) {
        return [Environment]::ExpandEnvironmentVariables([string]$settings[$Name])
    }
    return $Default
}
function Full-Path([string]$Value) {
    if ([System.IO.Path]::IsPathRooted($Value)) { return [System.IO.Path]::GetFullPath($Value) }
    return [System.IO.Path]::GetFullPath((Join-Path $ProjectRoot $Value))
}

$msbuild = Get-Setting "MSBuildExecutable"
if ([string]::IsNullOrWhiteSpace($msbuild)) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $found = @(& $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe")
    if ($found.Count -eq 0) { throw "MSBuild.exe was not found through vswhere." }
    $msbuild = [string]$found[0]
}
$msbuild = Full-Path $msbuild

$qtRoot = Full-Path (Get-Setting "QtRoot")
$qtMsBuild = Get-Setting "QtMsBuild" (Join-Path $env:LOCALAPPDATA "QtMsBuild")
$qtMsBuild = Full-Path $qtMsBuild
$openCvRoot = Full-Path (Get-Setting "OpenCvRoot")
$eigenRoot = Full-Path (Get-Setting "EigenRoot")
$kdlRoot = Full-Path (Get-Setting "OrocosKdlRoot")
$vcpkgRoot = Full-Path (Get-Setting "VcpkgRoot")
$occtRoot = Get-Setting "OcctRoot"
$occtRoot = if ([string]::IsNullOrWhiteSpace($occtRoot)) { Join-Path $vcpkgRoot "installed\x64-windows" } else { Full-Path $occtRoot }
$solution = Join-Path $ProjectRoot "QtWidgetsApplication4.sln"

$arguments = @(
    $solution,
    "/m",
    "/t:$Target",
    "/p:Configuration=$Configuration",
    "/p:Platform=x64",
    "/p:QtRoot=$qtRoot",
    "/p:QtVersion=$(Get-Setting 'QtVersion' '6.7.3')",
    "/p:QtInstall=$qtRoot",
    "/p:QtMsBuild=$qtMsBuild",
    "/p:OpenCvRoot=$openCvRoot",
    "/p:OpenCvVersionSuffix=$(Get-Setting 'OpenCvVersionSuffix' '460')",
    "/p:OpenCvToolsetDir=$(Get-Setting 'OpenCvToolsetDir' 'vc15')",
    "/p:EigenRoot=$eigenRoot",
    "/p:OrocosKdlRoot=$kdlRoot",
    "/p:VcpkgRoot=$vcpkgRoot",
    "/p:OcctRoot=$occtRoot",
    "/nologo",
    "/verbosity:minimal"
)

Write-Host "MSBuild      : $msbuild"
Write-Host "Configuration: $Configuration|x64"
Write-Host "Target       : $Target"
& $msbuild @arguments
if ($LASTEXITCODE -ne 0) {
    throw "MSBuild failed with exit code $LASTEXITCODE."
}

$exe = Join-Path $ProjectRoot "x64\$Configuration\QtWidgetsApplication4.exe"
if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) {
    throw "Build reported success but executable is missing: $exe"
}
$item = Get-Item -LiteralPath $exe
$hash = (Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash.ToLowerInvariant()
Write-Host "BUILD_OK"
Write-Host "Executable: $($item.FullName)"
Write-Host "Bytes     : $($item.Length)"
Write-Host "SHA256    : $hash"
