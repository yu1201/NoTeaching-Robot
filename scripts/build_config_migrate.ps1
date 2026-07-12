param(
    [Parameter(Mandatory = $true)][string]$PythonExecutable,
    [Parameter(Mandatory = $true)][string]$PythonSha256,
    [string]$OutputPath = "",
    # Kept for command-line compatibility. Rebuilds are now unconditional.
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$gateCommon = Join-Path $PSScriptRoot "release_gate_common.ps1"
if (-not (Test-Path -LiteralPath $gateCommon -PathType Leaf)) {
    throw "Release gate helpers were not found: $gateCommon"
}
. $gateCommon
Set-ReleasePythonTool -PythonExecutable $PythonExecutable -PythonSha256 $PythonSha256
$pythonTool = Get-ReleasePythonTool
$sourcePath = Join-Path $repoRoot "tools\migrate_config_to_sqlite.py"
$targetPath = if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    Join-Path $repoRoot "tools\ConfigMigrate.exe"
}
else {
    [System.IO.Path]::GetFullPath($OutputPath)
}
$buildRoot = Join-Path $repoRoot ("tmp\ConfigMigrateBuild\" + [Guid]::NewGuid().ToString("N"))
$distPath = Join-Path $buildRoot "dist"
$workPath = Join-Path $buildRoot "work"
$specPath = Join-Path $buildRoot "spec"
$deterministicSourceRoot = Join-Path ([System.IO.Path]::GetTempPath()) "NoTeachingRobotRelease\ConfigMigrateDeterministicSource"
$deterministicSourcePath = Join-Path $deterministicSourceRoot "migrate_config_to_sqlite.py"
$buildMutex = New-Object System.Threading.Mutex($false, "Local\NoTeachingRobot_ConfigMigrateDeterministicBuild_v1")
$mutexAcquired = $false

if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) {
    throw "Migration source not found: $sourcePath"
}

$expectedHash = (Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash.ToLowerInvariant()
$targetParent = Split-Path -Parent $targetPath
if (-not [string]::IsNullOrWhiteSpace($targetParent)) {
    New-Item -ItemType Directory -Path $targetParent -Force | Out-Null
}
$previousPythonHashSeed = $env:PYTHONHASHSEED
$previousSourceDateEpoch = $env:SOURCE_DATE_EPOCH

try {
    try {
        $mutexAcquired = $buildMutex.WaitOne([TimeSpan]::FromMinutes(3))
    }
    catch [System.Threading.AbandonedMutexException] {
        $mutexAcquired = $true
    }
    if (-not $mutexAcquired) {
        throw "Timed out waiting for the deterministic ConfigMigrate build lock."
    }
    if (Test-Path -LiteralPath $deterministicSourceRoot) {
        Remove-Item -LiteralPath $deterministicSourceRoot -Recurse -Force
    }
    New-Item -ItemType Directory -Path $deterministicSourceRoot -Force | Out-Null
    Copy-Item -LiteralPath $sourcePath -Destination $deterministicSourcePath -Force
    if ((Get-FileHash -LiteralPath $deterministicSourcePath -Algorithm SHA256).Hash.ToLowerInvariant() -cne $expectedHash) {
        throw "Deterministic ConfigMigrate staging source differs from the current repository source."
    }
    New-Item -ItemType Directory -Path $distPath, $workPath, $specPath -Force | Out-Null
    $dataArgument = "$deterministicSourcePath;_migrator_source"
    $env:PYTHONHASHSEED = "1"
    $env:SOURCE_DATE_EPOCH = "946684800"
    # CPython -I deliberately ignores PYTHONHASHSEED, while PyInstaller archive
    # ordering must use a fixed hash seed for byte-for-byte provenance rebuilds.
    # A verified -I supervisor therefore launches the exact same hash-pinned
    # interpreter with an explicit minimal environment, disabled user site, safe
    # sys.path, and the sole deterministic PYTHONHASHSEED exception.
    $isolatedBootstrap = @'
import os
import subprocess
import sys

python = sys.argv[1]
arguments = sys.argv[2:]
allowed = {
    'APPDATA', 'COMSPEC', 'HOME', 'LOCALAPPDATA', 'PATH', 'PATHEXT',
    'PROGRAMDATA', 'SYSTEMROOT', 'TEMP', 'TMP', 'USERPROFILE', 'WINDIR',
}
environment = {key: value for key, value in os.environ.items() if key.upper() in allowed}
environment.update({
    'PYTHONHASHSEED': '1',
    'PYTHONDONTWRITEBYTECODE': '1',
    'PYTHONNOUSERSITE': '1',
    'PYTHONSAFEPATH': '1',
    'SOURCE_DATE_EPOCH': '946684800',
})
completed = subprocess.run(
    [python, '-s', '-P', '-B', '-m', 'PyInstaller', *arguments],
    env=environment,
    stdin=subprocess.DEVNULL,
    check=False,
)
raise SystemExit(completed.returncode)
'@
    & $pythonTool.path -I -B -c $isolatedBootstrap $pythonTool.path `
        --noconfirm `
        --clean `
        --log-level WARN `
        --noupx `
        --onefile `
        --name ConfigMigrate `
        --distpath $distPath `
        --workpath $workPath `
        --specpath $specPath `
        --add-data $dataArgument `
        $deterministicSourcePath
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller failed with exit code $LASTEXITCODE"
    }
    Get-ReleasePythonTool | Out-Null

    $builtPath = Join-Path $distPath "ConfigMigrate.exe"
    if (-not (Test-Path -LiteralPath $builtPath -PathType Leaf)) {
        throw "PyInstaller did not create: $builtPath"
    }
    Copy-Item -LiteralPath $builtPath -Destination $targetPath -Force

    # This is a post-build sanity check only. No skip/reuse decision trusts output
    # reported by an existing executable.
    $probeOutput = @(& $targetPath --print-source-sha256 2>&1 | ForEach-Object { [string]$_ })
    if ($LASTEXITCODE -ne 0 `
        -or $probeOutput.Count -ne 1 `
        -or $probeOutput[0].Trim().ToLowerInvariant() -cne $expectedHash) {
        throw "Freshly rebuilt ConfigMigrate.exe does not embed the current migration source hash."
    }
    if ((Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash.ToLowerInvariant() -cne $expectedHash `
        -or (Get-FileHash -LiteralPath $deterministicSourcePath -Algorithm SHA256).Hash.ToLowerInvariant() -cne $expectedHash) {
        throw "ConfigMigrate source changed during deterministic build."
    }
    Write-Host "Built ConfigMigrate.exe from current source: $expectedHash"
    Write-Output $targetPath
}
finally {
    if ($null -eq $previousPythonHashSeed) { Remove-Item Env:PYTHONHASHSEED -ErrorAction SilentlyContinue }
    else { $env:PYTHONHASHSEED = $previousPythonHashSeed }
    if ($null -eq $previousSourceDateEpoch) { Remove-Item Env:SOURCE_DATE_EPOCH -ErrorAction SilentlyContinue }
    else { $env:SOURCE_DATE_EPOCH = $previousSourceDateEpoch }
    if (Test-Path -LiteralPath $buildRoot) {
        Remove-Item -LiteralPath $buildRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
    if ($mutexAcquired -and (Test-Path -LiteralPath $deterministicSourceRoot)) {
        Remove-Item -LiteralPath $deterministicSourceRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
    if ($mutexAcquired) {
        try { $buildMutex.ReleaseMutex() } catch { }
    }
    $buildMutex.Dispose()
}
