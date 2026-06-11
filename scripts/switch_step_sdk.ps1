param(
    [ValidateSet("timestamp", "legacy", "new", "old")]
    [string]$Mode = "timestamp"
)

$ErrorActionPreference = "Stop"

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "..")).Path
$stepRoot = Join-Path $repoRoot "SDK\STEP"
$versionsRoot = Join-Path $stepRoot "versions"

$normalizedMode = switch ($Mode) {
    "new" { "timestamp" }
    "old" { "legacy" }
    default { $Mode }
}

if ($normalizedMode -eq "legacy") {
    $sourceDir = Join-Path $versionsRoot "legacy_2.4.1_20250801"
    $hasTimestamp = "0"
    $displayName = "STEP legacy SDK 2.4.1"
}
else {
    $sourceDir = Join-Path $versionsRoot "timestamp_2.4.2_20260302_0603"
    $hasTimestamp = "1"
    $displayName = "STEP timestamp SDK 2.4.2"
}

if (-not (Test-Path -LiteralPath $sourceDir)) {
    throw "STEP SDK source directory was not found: $sourceDir"
}

foreach ($fileName in @("Robot-SDK.lib", "Robot-SDKd.lib", "RobotCom.hpp", "ControlMsgData.hpp", "EasyTcpClient.hpp")) {
    $sourcePath = Join-Path $sourceDir $fileName
    if (-not (Test-Path -LiteralPath $sourcePath)) {
        throw "STEP SDK source file was not found: $sourcePath"
    }
    Copy-Item -LiteralPath $sourcePath -Destination (Join-Path $stepRoot $fileName) -Force
}

$buildConfigPath = Join-Path $repoRoot "include\StepSdkBuildConfig.h"
@(
    "#pragma once",
    "",
    "#ifndef STEP_SDK_HAS_TIMESTAMP",
    "#define STEP_SDK_HAS_TIMESTAMP $hasTimestamp",
    "#endif"
) | Set-Content -LiteralPath $buildConfigPath -Encoding UTF8

Write-Host "Active STEP SDK: $displayName"
Write-Host "STEP_SDK_HAS_TIMESTAMP=$hasTimestamp"
Write-Host "Rebuild Debug/Release after switching."
