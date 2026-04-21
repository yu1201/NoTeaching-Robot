param(
    [switch]$SkipPackageBuild,
    [string]$AppVersion = (Get-Date -Format "yyyy.MM.dd")
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
$packageScript = Join-Path $scriptRoot "build_release_package.ps1"
$issPath = Join-Path $repoRoot "installer\QtWidgetsApplication4.iss"
$packageDir = Join-Path $repoRoot "dist\QtWidgetsApplication4"

if (-not $SkipPackageBuild) {
    & $packageScript
    if ($LASTEXITCODE -ne 0) {
        throw "Package build failed."
    }
}

if (-not (Test-Path -LiteralPath $packageDir)) {
    throw "Package directory is missing: $packageDir"
}

$isccPath = Find-InnoSetupCompiler

if (-not $isccPath) {
    throw "ISCC.exe was not found. Please install Inno Setup 6 first."
}

Write-Host "Compiling Inno Setup installer..."
& $isccPath "/DMyAppVersion=$AppVersion" $issPath
if ($LASTEXITCODE -ne 0) {
    throw "Inno Setup compilation failed with exit code $LASTEXITCODE."
}

Write-Host ""
Write-Host "Installer output folder:"
Write-Host "  $(Join-Path $repoRoot 'dist\installer')"
