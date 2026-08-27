param(
    [string]$Configuration = "Release",
    [string]$WeldPosePath = "",
    [switch]$ExpectFieldReplayReject
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$outputDir = Join-Path $repoRoot "x64\StandaloneTests\PlatformSemanticValidator"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "vswhere.exe not found"
}
$vsRoot = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if ([string]::IsNullOrWhiteSpace($vsRoot)) {
    throw "Visual Studio C++ toolchain not found"
}
$devCmd = Join-Path $vsRoot "Common7\Tools\VsDevCmd.bat"
$source = Join-Path $repoRoot "src\PlatformSemanticValidator.cpp"
$testSource = Join-Path $repoRoot "scripts\tests\platform_semantic_validator_tests.cpp"
$includeDir = Join-Path $repoRoot "include"
$testExe = Join-Path $outputDir "platform_semantic_validator_tests.exe"

$compileCommand = 'call "{0}" -arch=x64 -host_arch=x64 >nul && cl.exe /nologo /std:c++17 /EHsc /W4 /utf-8 /O2 /I"{1}" "{2}" "{3}" /Fe:"{4}"' -f `
    $devCmd, $includeDir, $source, $testSource, $testExe
& $env:ComSpec /d /s /c $compileCommand
if ($LASTEXITCODE -ne 0) {
    throw "standalone semantic validator compilation failed: $LASTEXITCODE"
}

& $testExe
if ($LASTEXITCODE -ne 0) {
    throw "standalone semantic validator tests failed: $LASTEXITCODE"
}

Write-Host "Standalone test executable: $testExe"

if (-not [string]::IsNullOrWhiteSpace($WeldPosePath)) {
    $replaySource = Join-Path $repoRoot "scripts\tests\platform_semantic_weldpose_replay.cpp"
    $replayExe = Join-Path $outputDir "platform_semantic_weldpose_replay.exe"
    $replayCompileCommand = 'call "{0}" -arch=x64 -host_arch=x64 >nul && cl.exe /nologo /std:c++17 /EHsc /W4 /utf-8 /O2 /I"{1}" "{2}" "{3}" /Fe:"{4}"' -f `
        $devCmd, $includeDir, $source, $replaySource, $replayExe
    & $env:ComSpec /d /s /c $replayCompileCommand
    if ($LASTEXITCODE -ne 0) {
        throw "standalone field replay compilation failed: $LASTEXITCODE"
    }
    & $replayExe $WeldPosePath 1 0 0
    $replayExitCode = $LASTEXITCODE
    if ($ExpectFieldReplayReject -and $replayExitCode -eq 0) {
        throw "standalone field replay unexpectedly passed"
    }
    if (-not $ExpectFieldReplayReject -and $replayExitCode -ne 0) {
        throw "standalone field replay failed: $LASTEXITCODE"
    }
    if ($ExpectFieldReplayReject) {
        Write-Host "Field replay rejection matched expectation"
    }
    Write-Host "Standalone field replay executable: $replayExe"
}
