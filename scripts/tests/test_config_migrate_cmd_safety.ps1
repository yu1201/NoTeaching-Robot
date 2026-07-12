[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$scriptPath = Join-Path $repo 'tools\ConfigMigrate_Run.cmd'
$testRoot = Join-Path $repo 'tmp\ConfigMigrateCmdSafety'
$marker = Join-Path $testRoot 'injection-marker.txt'

New-Item -ItemType Directory -Path $testRoot -Force | Out-Null
if (Test-Path -LiteralPath $marker) {
    Remove-Item -LiteralPath $marker -Force
}

# 模拟来自参数的 & 命令分隔符。整个值先作为一个带引号参数进入 cmd；若脚本之后
# 用 echo ... %SOURCE_DIR% 无引号二次展开，就会创建 marker。
$sourceArgument = (Join-Path $testRoot 'missing') + "&echo CONFIG_MIGRATE_INJECTION_CONFIRMED>$marker"
$dataRoot = Join-Path $testRoot 'Data Root & Legitimate'
New-Item -ItemType Directory -Path $dataRoot -Force | Out-Null

& $scriptPath --data-root $dataRoot --source $sourceArgument --no-pause | Out-Null
if ($LASTEXITCODE -eq 0) {
    throw 'ConfigMigrate_Run.cmd unexpectedly accepted a missing synthetic source.'
}
if (Test-Path -LiteralPath $marker) {
    throw 'ConfigMigrate_Run.cmd reinterpreted a path argument as a command.'
}

$text = Get-Content -LiteralPath $scriptPath -Raw
foreach ($required in @(
    'echo Unknown argument: "%~1"',
    'echo Legacy Data directory not found: "%SOURCE_DIR%"',
    'echo Resolved data root: "%DATA_ROOT%"',
    'echo Resolved source Data: "%SOURCE_DIR%"',
    'echo Resolved target database: "%DB_PATH%"'
)) {
    if (-not $text.Contains($required)) {
        throw "Missing quoted cmd expansion: $required"
    }
}

Write-Host 'PASS: ConfigMigrate_Run.cmd does not reinterpret path metacharacters.'
