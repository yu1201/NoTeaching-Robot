[CmdletBinding()]
param(
    [string]$ProjectRoot = "",
    [string]$OutputPath = ""
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

$kitName = "QtWidgetsApplication4-source-environment-kit-20260819"
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $OutputPath = Join-Path $ProjectRoot "output\$kitName.zip"
} elseif (-not [System.IO.Path]::IsPathRooted($OutputPath)) {
    $OutputPath = Join-Path $ProjectRoot $OutputPath
}
$OutputPath = [System.IO.Path]::GetFullPath($OutputPath)
if ([System.IO.Path]::GetExtension($OutputPath) -ne ".zip") {
    throw "OutputPath must end with .zip: $OutputPath"
}

$outputDirectory = Split-Path -Parent $OutputPath
[void](New-Item -ItemType Directory -Path $outputDirectory -Force)

$tempBase = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$tempContainer = Join-Path $tempBase ("source-environment-kit-" + [Guid]::NewGuid().ToString("N"))
$stageRoot = Join-Path $tempContainer $kitName
[void](New-Item -ItemType Directory -Path (Join-Path $stageRoot "scripts") -Force)
[void](New-Item -ItemType Directory -Path (Join-Path $stageRoot "config") -Force)
$sourceToolRoot = Join-Path $ProjectRoot "tools\source_environment"

try {
    Copy-Item -LiteralPath (Join-Path $ProjectRoot "docs\source-environment-setup.md") -Destination (Join-Path $stageRoot "README.md")
    Copy-Item -LiteralPath (Join-Path $sourceToolRoot "PACKAGE-CONTENTS.md") -Destination (Join-Path $stageRoot "PACKAGE-CONTENTS.md")
    foreach ($name in @("check_environment.ps1", "build_local.ps1", "new_environment_kit.ps1")) {
        Copy-Item -LiteralPath (Join-Path $sourceToolRoot $name) -Destination (Join-Path $stageRoot "scripts\$name")
    }
    foreach ($name in @("environment.local.props.example", "environment.example.psd1", "dependency-lock.json")) {
        Copy-Item -LiteralPath (Join-Path $sourceToolRoot $name) -Destination (Join-Path $stageRoot "config\$name")
    }

    $sdkFiles = @(
        "SDK\STEP\Robot-SDK.lib",
        "SDK\STEP\Robot-SDKd.lib",
        "SDK\PointCloudExtration\PointCloudExtration.dll",
        "SDK\PointCloudExtration\PointCloudExtration.lib",
        "SDK\SKJCamera\bin\x64\Debug\SKJCamera.dll",
        "SDK\SKJCamera\bin\x64\Release\SKJCamera.dll",
        "SDK\BCPD\bcpd.exe"
    )
    $sdkManifest = @()
    foreach ($relative in $sdkFiles) {
        $path = Join-Path $ProjectRoot $relative
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Required SDK file is missing: $relative" }
        $item = Get-Item -LiteralPath $path
        $hash = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
        $sdkManifest += "$hash  $($item.Length)  $($relative.Replace('\', '/'))"
    }
    [System.IO.File]::WriteAllLines((Join-Path $stageRoot "project-sdk-sha256.txt"), $sdkManifest, [System.Text.UTF8Encoding]::new($false))

    $gitHead = "unknown"
    $gitChangeCount = -1
    $git = Get-Command git.exe -ErrorAction SilentlyContinue
    if ($null -ne $git -and (Test-Path -LiteralPath (Join-Path $ProjectRoot ".git"))) {
        $safeRoot = $ProjectRoot.Replace('\', '/')
        $gitHead = ((& $git.Source -c "safe.directory=$safeRoot" -C $ProjectRoot rev-parse HEAD 2>$null) -join "").Trim()
        $gitStatus = @(& $git.Source -c "safe.directory=$safeRoot" -C $ProjectRoot status --porcelain 2>$null)
        $gitChangeCount = $gitStatus.Count
    }
    $sourceInfo = [ordered]@{
        schemaVersion = 1
        generatedLocal = (Get-Date).ToString("yyyy-MM-ddTHH:mm:ssK")
        projectHead = $gitHead
        worktreeChangeCount = $gitChangeCount
        note = "The kit contains setup tooling only; it is not a source snapshot or release authorization."
    }
    $sourceInfo | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $stageRoot "package-source.json") -Encoding UTF8

    $manifestLines = @()
    $manifestFiles = @(Get-ChildItem -LiteralPath $stageRoot -Recurse -File | Sort-Object FullName)
    foreach ($file in $manifestFiles) {
        $relative = $file.FullName.Substring($stageRoot.Length + 1).Replace('\', '/')
        $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
        $manifestLines += "$hash  $relative"
    }
    [System.IO.File]::WriteAllLines((Join-Path $stageRoot "MANIFEST.sha256"), $manifestLines, [System.Text.UTF8Encoding]::new($false))

    Compress-Archive -LiteralPath $stageRoot -DestinationPath $OutputPath -CompressionLevel Optimal -Force
} finally {
    $resolvedTemp = [System.IO.Path]::GetFullPath($tempContainer)
    if ($resolvedTemp.StartsWith($tempBase, [System.StringComparison]::OrdinalIgnoreCase) -and (Test-Path -LiteralPath $resolvedTemp)) {
        Remove-Item -LiteralPath $resolvedTemp -Recurse -Force
    }
}

Add-Type -AssemblyName System.IO.Compression.FileSystem
$archive = [System.IO.Compression.ZipFile]::OpenRead($OutputPath)
try {
    $entryNames = @($archive.Entries | ForEach-Object { $_.FullName.Replace('\', '/') })
    foreach ($required in @(
        "$kitName/README.md",
        "$kitName/MANIFEST.sha256",
        "$kitName/scripts/check_environment.ps1",
        "$kitName/scripts/build_local.ps1",
        "$kitName/config/environment.local.props.example"
    )) {
        if ($entryNames -notcontains $required) { throw "ZIP is missing required entry: $required" }
    }
    $entryCount = $archive.Entries.Count
} finally {
    $archive.Dispose()
}

$zip = Get-Item -LiteralPath $OutputPath
$zipHash = (Get-FileHash -LiteralPath $OutputPath -Algorithm SHA256).Hash.ToLowerInvariant()
Write-Host "KIT_OK"
Write-Host "ZIP_PATH : $($zip.FullName)"
Write-Host "ZIP_BYTES: $($zip.Length)"
Write-Host "ZIP_SHA256: $zipHash"
Write-Host "ZIP_ENTRIES: $entryCount"
