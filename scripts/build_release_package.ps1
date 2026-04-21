param(
    [switch]$SkipBuild,
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

$exePath = Join-Path $buildDir "QtWidgetsApplication4.exe"
if (-not (Test-Path -LiteralPath $exePath)) {
    throw "Release executable was not found: $exePath"
}

if ($windeployqtPath) {
    Write-Host "Running windeployqt on the Release executable..."
    & $windeployqtPath --release --no-translations --no-opengl-sw $exePath
    if ($LASTEXITCODE -ne 0) {
        throw "windeployqt failed with exit code $LASTEXITCODE."
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
Get-ChildItem -LiteralPath $buildDir -File | Where-Object {
    $ignoredReleaseExtensions -notcontains $_.Extension.ToLowerInvariant()
} | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $packageDir $_.Name) -Force
}

Get-ChildItem -LiteralPath $buildDir -Directory | ForEach-Object {
    Copy-DirectoryContent -SourceDir $_.FullName -TargetDir (Join-Path $packageDir $_.Name)
}

$dataSourceDir = Join-Path $repoRoot "Data"
$dataTargetDir = Join-Path $packageDir "Data"
New-Item -ItemType Directory -Path $dataTargetDir -Force | Out-Null
Get-ChildItem -LiteralPath $dataSourceDir -Directory -Force | Where-Object {
    $_.Name -notlike "*副本*"
} | ForEach-Object {
    $sourceRoot = $_.FullName
    $targetRoot = Join-Path $dataTargetDir $_.Name
    New-Item -ItemType Directory -Path $targetRoot -Force | Out-Null

    Get-ChildItem -LiteralPath $sourceRoot -Recurse -Force | Where-Object {
        $_.Name -notlike "*副本*"
    } | ForEach-Object {
        $relativePath = $_.FullName.Substring($sourceRoot.Length).TrimStart('\')
        $targetPath = Join-Path $targetRoot $relativePath

        if ($_.PSIsContainer) {
            New-Item -ItemType Directory -Path $targetPath -Force | Out-Null
        }
        else {
            $targetParent = Split-Path -Parent $targetPath
            New-Item -ItemType Directory -Path $targetParent -Force | Out-Null
            Copy-Item -LiteralPath $_.FullName -Destination $targetPath -Force
        }
    }
}

Get-ChildItem -LiteralPath $dataSourceDir -File -Force | Where-Object {
    $_.Name -notlike "*副本*"
} | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $dataTargetDir $_.Name) -Force
}

Copy-DirectoryContent -SourceDir (Join-Path $repoRoot "icons") -TargetDir (Join-Path $packageDir "icons")

$fanucSourceDir = Join-Path $repoRoot "SDK\FANUC"
$fanucTargetDir = Join-Path $packageDir "SDK\FANUC"
New-Item -ItemType Directory -Path $fanucTargetDir -Force | Out-Null
Get-ChildItem -LiteralPath $fanucSourceDir -File | Where-Object {
    $_.Extension.ToLowerInvariant() -in @(".kl", ".ls", ".pc", ".tp", ".var", ".ini", ".txt")
} | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $fanucTargetDir $_.Name) -Force
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
    "4. If STEP functions are required on the target PC, vendor runtime components may still be needed.",
    "5. If the target PC misses VC++ runtime, install Microsoft Visual C++ 2015-2022 Redistributable (x64)."
)
$notes | Set-Content -LiteralPath $notesPath -Encoding UTF8

Write-Host ""
Write-Host "Release package is ready:"
Write-Host "  $packageDir"
