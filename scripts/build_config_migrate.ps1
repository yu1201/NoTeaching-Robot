param(
    [string]$PythonExecutable = "python",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$sourcePath = Join-Path $repoRoot "tools\migrate_config_to_sqlite.py"
$targetPath = Join-Path $repoRoot "tools\ConfigMigrate.exe"
$buildRoot = Join-Path $repoRoot "tmp\ConfigMigrateBuild"
$distPath = Join-Path $buildRoot "dist"
$workPath = Join-Path $buildRoot "work"
$specPath = Join-Path $buildRoot "spec"

if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) {
    throw "Migration source not found: $sourcePath"
}

$expectedHash = (Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash.ToLowerInvariant()
if (-not $Force -and (Test-Path -LiteralPath $targetPath -PathType Leaf)) {
    $reportedHash = (& $targetPath --print-source-sha256 2>$null | Select-Object -Last 1)
    if ($LASTEXITCODE -eq 0 -and $null -ne $reportedHash -and $reportedHash.Trim().ToLowerInvariant() -eq $expectedHash) {
        Write-Host "ConfigMigrate.exe is current: $expectedHash"
        return
    }
}

New-Item -ItemType Directory -Path $distPath, $workPath, $specPath -Force | Out-Null
$dataArgument = "$sourcePath;_migrator_source"
& $PythonExecutable -m PyInstaller `
    --noconfirm `
    --clean `
    --noupx `
    --onefile `
    --name ConfigMigrate `
    --distpath $distPath `
    --workpath $workPath `
    --specpath $specPath `
    --add-data $dataArgument `
    $sourcePath
if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller failed with exit code $LASTEXITCODE"
}

$builtPath = Join-Path $distPath "ConfigMigrate.exe"
if (-not (Test-Path -LiteralPath $builtPath -PathType Leaf)) {
    throw "PyInstaller did not create: $builtPath"
}
Copy-Item -LiteralPath $builtPath -Destination $targetPath -Force

$reportedHash = (& $targetPath --print-source-sha256 | Select-Object -Last 1)
if ($LASTEXITCODE -ne 0 -or $null -eq $reportedHash -or $reportedHash.Trim().ToLowerInvariant() -ne $expectedHash) {
    throw "ConfigMigrate.exe source hash does not match current migration source."
}
Write-Host "Built ConfigMigrate.exe from current source: $expectedHash"
