param()

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\..")).Path
. (Join-Path $repoRoot "scripts\release_gate_common.ps1")
$testPythonExecutable = (& python.exe -I -c "import sys; print(sys.executable)" | Select-Object -Last 1).Trim()
$testPythonSha256 = Get-ReleaseFileSha256 $testPythonExecutable
Set-ReleasePythonTool `
    -PythonExecutable $testPythonExecutable `
    -PythonSha256 $testPythonSha256

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) {
        throw "ASSERT FAILED: $Message"
    }
}

function Assert-Throws {
    param([scriptblock]$Action, [string]$Message)
    $threw = $false
    try {
        & $Action
    }
    catch {
        $threw = $true
    }
    if (-not $threw) {
        throw "ASSERT FAILED (expected failure): $Message"
    }
}

function New-TestPeFixture {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Version,
        [Parameter(Mandatory = $true)][string]$ProductName,
        [ValidateSet("x64", "x86")][string]$Platform = "x64"
    )

    $escapedProduct = $ProductName.Replace('\', '\\').Replace('"', '\"')
    $escapedVersion = $Version.Replace('\', '\\').Replace('"', '\"')
    $code = @"
using System;
using System.Reflection;
[assembly: AssemblyTitle("Fixture")]
[assembly: AssemblyProduct("$escapedProduct")]
[assembly: AssemblyFileVersion("$escapedVersion")]
[assembly: AssemblyInformationalVersion("$escapedVersion")]
public static class Program { public static int Main() { return 0; } }
"@
    $parent = Split-Path -Parent $Path
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
    $provider = New-Object Microsoft.CSharp.CSharpCodeProvider
    try {
        $parameters = New-Object System.CodeDom.Compiler.CompilerParameters
        $parameters.CompilerOptions = "/platform:$Platform"
        $parameters.OutputAssembly = $Path
        $parameters.GenerateExecutable = $true
        $result = $provider.CompileAssemblyFromSource($parameters, $code)
        if ($result.Errors.HasErrors) {
            throw "C# PE fixture compilation failed: $($result.Errors | Out-String)"
        }
    }
    finally {
        $provider.Dispose()
    }
}

function New-FakeConfigMigrateProbe {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$ReportedHash
    )

    $code = @"
using System;
public static class Program { public static int Main(string[] args) { Console.WriteLine("$ReportedHash"); return 0; } }
"@
    $provider = New-Object Microsoft.CSharp.CSharpCodeProvider
    try {
        $parameters = New-Object System.CodeDom.Compiler.CompilerParameters
        $parameters.CompilerOptions = "/platform:x64"
        $parameters.OutputAssembly = $Path
        $parameters.GenerateExecutable = $true
        $result = $provider.CompileAssemblyFromSource($parameters, $code)
        if ($result.Errors.HasErrors) {
            throw "Fake ConfigMigrate probe compilation failed: $($result.Errors | Out-String)"
        }
    }
    finally {
        $provider.Dispose()
    }
}

function Invoke-GitFixture {
    param([string]$RepoRoot, [string[]]$Arguments)
    & git -C $RepoRoot @Arguments | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "git fixture command failed in ${RepoRoot}: git $($Arguments -join ' ')"
    }
}

function Copy-ManifestRuntimeFixture {
    param([string]$TargetRoot)
    New-Item -ItemType Directory -Path (Join-Path $TargetRoot "installer"), (Join-Path $TargetRoot "SDK\FANUC") -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $repoRoot "installer\fanuc-runtime-manifest.json") -Destination (Join-Path $TargetRoot "installer\fanuc-runtime-manifest.json")
    $manifest = Get-Content -LiteralPath (Join-Path $repoRoot "installer\fanuc-runtime-manifest.json") -Raw | ConvertFrom-Json
    foreach ($item in @($manifest.files)) {
        $relative = ([string]$item.path).Replace('/', '\')
        Copy-Item -LiteralPath (Join-Path $repoRoot $relative) -Destination (Join-Path $TargetRoot $relative)
    }
    & git -C $TargetRoot init -q
    if ($LASTEXITCODE -ne 0) { throw "git init failed for fixture" }
    & git -C $TargetRoot add -- installer/fanuc-runtime-manifest.json
    if ($LASTEXITCODE -ne 0) { throw "git add failed for fixture manifest" }
}

$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("release-gate-tests-" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null
try {
    # Every production script must remain parseable without running MSBuild/Inno.
    foreach ($relative in @(
        "scripts/release_gate_common.ps1",
        "scripts/build_release_package.ps1",
        "scripts/build_installer.ps1",
        "scripts/prepare_release_worktree.ps1",
        "scripts/verify_release_pair.ps1")) {
        $tokens = $null
        $errors = $null
        [System.Management.Automation.Language.Parser]::ParseFile(
            (Join-Path $repoRoot $relative), [ref]$tokens, [ref]$errors) | Out-Null
        Assert-True ($errors.Count -eq 0) "$relative has PowerShell parse errors"
    }

    Assert-ReleaseVersion "2026.07.12.1200"
    Assert-Throws { Assert-ReleaseVersion "v2026.07.12.1200" } "version with v prefix must fail"
    Assert-Throws { Assert-ReleaseVersion "2026.7.12.1200" } "non-canonical version must fail"
    Assert-Throws { Assert-ReleaseVersion "2026.99.99.9999" } "invalid calendar version must fail"
    $currentInstallerDefinition = Assert-InstallerDefinition -RepoRoot $repoRoot -AppVersion "2026.07.13.2115" -Channel neutral
    Assert-True ([string]$currentInstallerDefinition.appId -ceq $script:ReleaseGateExpectedAppId) "installer AppId hard gate failed"

    $installerFixture = Join-Path $tempRoot "installer-definition"
    New-Item -ItemType Directory -Path (Join-Path $installerFixture "installer") -Force | Out-Null
    $installerFixturePath = Join-Path $installerFixture "installer\QtWidgetsApplication4.iss"
    $installerTextOriginal = Get-Content -LiteralPath (Join-Path $repoRoot "installer\QtWidgetsApplication4.iss") -Raw -Encoding UTF8
    function Assert-InstallerTextRejected {
        param([string]$Text, [string]$Message)
        Assert-True ($Text -cne $installerTextOriginal) "installer attack fixture mutation did not change the source: $Message"
        Set-Content -LiteralPath $installerFixturePath -Value $Text -Encoding UTF8
        Assert-Throws { Assert-InstallerDefinition -RepoRoot $installerFixture -AppVersion "2026.07.13.2115" -Channel neutral } $Message
    }
    $sourceDirDefine = '#define MySourceDir "..\dist\QtWidgetsApplication4"'
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace($sourceDirDefine, ($sourceDirDefine + "`r`n#undef MySourceDir`r`n#define MySourceDir `"C:\Windows`""))) `
        -Message "MySourceDir undef/redefinition to an external directory must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('[Icons]', "Source: `"C:\Windows\win.ini`"; DestDir: `"{app}`"; Flags: ignoreversion`r`n`r`n[Icons]")) `
        -Message "extra external [Files] Source must fail"
    $configRunRecord = 'Source: "..\dist\tools\ConfigMigrate_Run.cmd"; DestDir: "{app}\tools"; Flags: ignoreversion'
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace($configRunRecord, ("#if 0`r`n" + $configRunRecord + "`r`n#endif"))) `
        -Message "preprocessor conditional hiding a required Source must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('[Setup]', "#include `"injected-files.iss`"`r`n`r`n[Setup]")) `
        -Message "preprocessor include capable of injecting Source records must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal -replace '(?m)^Source:.*(?:\r?\n|$)', '') `
        -Message "installer with no [Files] Source records must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('Source: "{#MySourceDir}\*"', 'Source: ""')) `
        -Message "installer with empty Source must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace(' recursesubdirs', '')) `
        -Message "installer recursive payload without recursesubdirs must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion', 'Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion skipifsourcedoesntexist')) `
        -Message "optional ConfigMigrate.exe Source must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion', 'Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion; Check: False')) `
        -Message "conditionally skipped ConfigMigrate.exe Source must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('Excludes: "Data\*,SDK\STEP\versions\*"', 'Excludes: "Data\*,SDK\STEP\versions\*,*.exe"')) `
        -Message "recursive payload with extra executable exclusion must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal -replace '(?m)^Source: "\.\.\\dist\\tools\\ConfigMigrate_Run\.cmd".*(?:\r?\n|$)', '') `
        -Message "missing ConfigMigrate_Run.cmd Source must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('Source: "..\dist\tools\ConfigMigrate_Run.cmd"', 'DestName: "ConfigMigrate_Run.cmd"')) `
        -Message "[Files] record missing Source key must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal -replace '(?m)^Source: "\.\.\\dist\\tools\\ConfigMigrate_Install\.ps1"; DestDir: "\{app\}\\tools".*(?:\r?\n|$)', '') `
        -Message "missing installed ConfigMigrate_Install.ps1 helper must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('DestName: "ConfigMigrate_PreInstall.exe"; Flags: dontcopy', 'DestName: "ConfigMigrate_PreInstall.exe"; Flags: ignoreversion')) `
        -Message "pre-install migrator without dontcopy must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('DestName: "ConfigMigrate_PreInstall.ps1"', 'DestName: "ConfigMigrate_PreInstall-Tampered.ps1"')) `
        -Message "pre-install helper DestName drift must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('function PrepareToInstall(var NeedsRestart: Boolean): string;', 'function PrepareDatabaseLater(var NeedsRestart: Boolean): string;')) `
        -Message "missing PrepareToInstall migration hook must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('[Code]', "[Code]`r`n; AllowNoPendingTransaction")) `
        -Message "installer rollback that permits a missing transaction record must fail"
    Assert-InstallerTextRejected `
        -Text ($installerTextOriginal.Replace('if not CommitPendingDatabaseTransaction(CommitStatus) then', 'if False then')) `
        -Message "missing staged database post-install commit must fail"
    Set-Content -LiteralPath $installerFixturePath -Value $installerTextOriginal -Encoding UTF8
    Assert-InstallerDefinition -RepoRoot $installerFixture -AppVersion "2026.07.13.2115" -Channel neutral | Out-Null

    $fixture = Join-Path $tempRoot "fanuc"
    Copy-ManifestRuntimeFixture $fixture
    $manifestInfo = Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture
    Assert-True ([int]$manifestInfo.manifest.expectedFileCount -eq 21) "FANUC manifest must contain 21 files"

    $firstRuntime = Join-Path $fixture (([string]$manifestInfo.manifest.files[0].path).Replace('/', '\'))
    $savedBytes = [System.IO.File]::ReadAllBytes($firstRuntime)
    Remove-Item -LiteralPath $firstRuntime -Force
    Assert-Throws { Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture } "missing FANUC runtime must fail"
    [System.IO.File]::WriteAllBytes($firstRuntime, $savedBytes)

    $tampered = [byte[]]$savedBytes.Clone()
    $tampered[0] = $tampered[0] -bxor 1
    [System.IO.File]::WriteAllBytes($firstRuntime, $tampered)
    Assert-Throws { Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture } "same-size FANUC tamper must fail"
    [System.IO.File]::WriteAllBytes($firstRuntime, $savedBytes)

    Set-Content -LiteralPath (Join-Path $fixture "SDK\FANUC\EXTRA.tp") -Value "extra" -NoNewline
    Assert-Throws { Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture } "extra FANUC runtime must fail"
    Remove-Item -LiteralPath (Join-Path $fixture "SDK\FANUC\EXTRA.tp") -Force

    $manifestPath = Join-Path $fixture "installer\fanuc-runtime-manifest.json"
    $savedManifest = [System.IO.File]::ReadAllBytes($manifestPath)
    $emptyManifest = [ordered]@{
        schemaVersion = 1; manifestVersion = "bad-empty"; expectedFileCount = 0;
        expectedTpCount = 0; expectedPcCount = 0; files = @()
    }
    Write-ReleaseGateJson -Value $emptyManifest -Path $manifestPath
    Assert-Throws { Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture } "self-consistent empty manifest must fail"
    [System.IO.File]::WriteAllBytes($manifestPath, $savedManifest)

    $badManifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
    $badManifest.files[0].path = "SDK/FANUC/../escape.tp"
    Write-ReleaseGateJson -Value $badManifest -Path $manifestPath
    Assert-Throws { Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture } "manifest traversal must fail"
    [System.IO.File]::WriteAllBytes($manifestPath, $savedManifest)

    $duplicateManifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
    $duplicateManifest.files[1].path = ([string]$duplicateManifest.files[0].path).ToUpperInvariant()
    Write-ReleaseGateJson -Value $duplicateManifest -Path $manifestPath
    Assert-Throws { Assert-FanucRuntimeManifest -RepoRoot $fixture -RuntimeRoot $fixture } "case-colliding manifest paths must fail"
    [System.IO.File]::WriteAllBytes($manifestPath, $savedManifest)

    $package = Join-Path $tempRoot "package"
    New-Item -ItemType Directory -Path $package -Force | Out-Null
    foreach ($name in @("Data", "Log", "Result", "Temp")) {
        New-Item -ItemType Directory -Path (Join-Path $package $name) -Force | Out-Null
    }
    Assert-EmptyReleaseRuntimeDirectories $package
    Set-Content -LiteralPath (Join-Path $package "Data\must-not-ship.ini") -Value "secret" -NoNewline
    Assert-Throws { Assert-EmptyReleaseRuntimeDirectories $package } "runtime-owned Data content must fail"
    Remove-Item -LiteralPath (Join-Path $package "Data\must-not-ship.ini") -Force

    $assetRepo = Join-Path $tempRoot "assets-repo"
    $assetPackage = Join-Path $assetRepo "dist\QtWidgetsApplication4"
    New-Item -ItemType Directory -Path (Join-Path $assetRepo "icons"), (Join-Path $assetPackage "icons") -Force | Out-Null
    Set-Content -LiteralPath (Join-Path $assetRepo "icons\app.ico") -Value "neutral-icon" -NoNewline
    Copy-Item -LiteralPath (Join-Path $assetRepo "icons\app.ico") -Destination (Join-Path $assetPackage "icons\app.ico")
    & git -C $assetRepo init -q
    & git -C $assetRepo add -- icons/app.ico
    Assert-ReleaseChannelAssets -RepoRoot $assetRepo -PackageDir $assetPackage -Channel neutral
    New-Item -ItemType Directory -Path (Join-Path $assetPackage "branding") -Force | Out-Null
    Assert-Throws { Assert-ReleaseChannelAssets -RepoRoot $assetRepo -PackageDir $assetPackage -Channel neutral } "neutral package branding must fail"

    $exeDir = Join-Path $tempRoot "exe"
    New-Item -ItemType Directory -Path $exeDir -Force | Out-Null
    New-TestPeFixture -Path (Join-Path $exeDir "QtWidgetsApplication4.exe") -Version "2026.07.12.1200" -ProductName "NoTeaching-Robot"
    Assert-ExpectedReleaseExecutable -Directory $exeDir -AppVersion "2026.07.12.1200" -Channel neutral | Out-Null
    New-TestPeFixture -Path (Join-Path $exeDir "HK-Pathlynx-CORPLA.exe") -Version "2026.07.12.1200" -ProductName "HK-Pathlynx-CORPLA"
    Assert-Throws { Assert-ExpectedReleaseExecutable -Directory $exeDir -AppVersion "2026.07.12.1200" -Channel neutral } "opposite-channel exe must fail"
    Remove-Item -LiteralPath (Join-Path $exeDir "HK-Pathlynx-CORPLA.exe") -Force
    Assert-Throws { Assert-ExpectedReleaseExecutable -Directory $exeDir -AppVersion "2026.07.12.1201" -Channel neutral } "embedded version mismatch must fail"
    [System.IO.File]::WriteAllBytes(
        (Join-Path $exeDir "QtWidgetsApplication4.exe"),
        [System.Text.Encoding]::ASCII.GetBytes("MZ-fake PE marker 2026.07.12.1200"))
    Assert-Throws { Assert-ExpectedReleaseExecutable -Directory $exeDir -AppVersion "2026.07.12.1200" -Channel neutral } "fake MZ/version-marker exe must fail"
    Remove-Item -LiteralPath (Join-Path $exeDir "QtWidgetsApplication4.exe") -Force
    New-TestPeFixture -Path (Join-Path $exeDir "QtWidgetsApplication4.exe") -Version "2026.07.12.1200" -ProductName "NoTeaching-Robot" -Platform x86
    Assert-Throws { Assert-ExpectedReleaseExecutable -Directory $exeDir -AppVersion "2026.07.12.1200" -Channel neutral } "x86 application exe must fail"

    $installerPe = Join-Path $tempRoot "NoTeaching-Robot-Setup-v2026.07.12.1200.exe"
    New-TestPeFixture -Path $installerPe -Version "2026.07.12.1200" -ProductName "NoTeaching-Robot"
    Assert-InstallerProductVersion -InstallerPath $installerPe -AppVersion "2026.07.12.1200" -Channel neutral
    Assert-Throws {
        Assert-InstallerProductVersion -InstallerPath $installerPe -AppVersion "2026.07.12.1201" -Channel neutral
    } "real installer PE with the wrong requested version must fail"
    [System.IO.File]::WriteAllBytes(
        $installerPe,
        [System.Text.Encoding]::ASCII.GetBytes("MZ-fake installer 2026.07.12.1200"))
    Assert-Throws {
        Assert-InstallerProductVersion -InstallerPath $installerPe -AppVersion "2026.07.12.1200" -Channel neutral
    } "fake MZ/version-marker installer must fail"

    $inventoryRoot = Join-Path $tempRoot "inventory"
    New-Item -ItemType Directory -Path $inventoryRoot -Force | Out-Null
    Set-Content -LiteralPath (Join-Path $inventoryRoot "same.bin") -Value "AAAA" -NoNewline
    $inventory = @(Get-ReleaseFileInventory $inventoryRoot)
    Set-Content -LiteralPath (Join-Path $inventoryRoot "same.bin") -Value "BBBB" -NoNewline
    Assert-Throws { Assert-ReleaseInventoryMatches -Expected $inventory -Actual @(Get-ReleaseFileInventory $inventoryRoot) } "same-size inventory tamper must fail"

    $reportPath = Join-Path $tempRoot "atomic-report.json"
    Write-ReleaseGateJson -Value ([ordered]@{ status = "pass"; old = $true }) -Path $reportPath
    $pendingRun = New-ReleaseGatePendingReport -Path $reportPath -Kind package -AppVersion "2026.07.12.1200" -Channel neutral
    $pending = Read-ReleaseGateJson $reportPath
    Assert-True ([string]$pending.status -ceq "pending") "old pass must be invalidated by pending"
    Assert-True ([string]$pending.runId -ceq $pendingRun) "pending runId must be stable"
    Assert-True (@(Get-ChildItem -LiteralPath $tempRoot -Filter "atomic-report.json.tmp.*").Count -eq 0) "atomic writer left a temp report"
    Assert-True (@(Get-ChildItem -LiteralPath $tempRoot -Filter "atomic-report.json.bak.*").Count -eq 0) "atomic writer left a backup report"

    $configFixture = Join-Path $tempRoot "config"
    New-Item -ItemType Directory -Path (Join-Path $configFixture "tools"), (Join-Path $configFixture "scripts") -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $repoRoot "tools\migrate_config_to_sqlite.py") -Destination (Join-Path $configFixture "tools\migrate_config_to_sqlite.py")
    Copy-Item -LiteralPath (Join-Path $repoRoot "scripts\build_config_migrate.ps1") -Destination (Join-Path $configFixture "scripts\build_config_migrate.ps1")
    Copy-Item -LiteralPath (Join-Path $repoRoot "scripts\release_gate_common.ps1") -Destination (Join-Path $configFixture "scripts\release_gate_common.ps1")
    & (Join-Path $configFixture "scripts\build_config_migrate.ps1") `
        -PythonExecutable $testPythonExecutable `
        -PythonSha256 $testPythonSha256 `
        -OutputPath (Join-Path $configFixture "tools\ConfigMigrate.exe") | Out-Null
    Assert-ConfigMigrateProvenance -RepoRoot $configFixture -ExecutablePath (Join-Path $configFixture "tools\ConfigMigrate.exe") | Out-Null
    $configFixtureOtherRoot = Join-Path $tempRoot "config-other-worktree"
    New-Item -ItemType Directory -Path (Join-Path $configFixtureOtherRoot "tools"), (Join-Path $configFixtureOtherRoot "scripts") -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $repoRoot "tools\migrate_config_to_sqlite.py") -Destination (Join-Path $configFixtureOtherRoot "tools\migrate_config_to_sqlite.py")
    Copy-Item -LiteralPath (Join-Path $repoRoot "scripts\build_config_migrate.ps1") -Destination (Join-Path $configFixtureOtherRoot "scripts\build_config_migrate.ps1")
    Copy-Item -LiteralPath (Join-Path $repoRoot "scripts\release_gate_common.ps1") -Destination (Join-Path $configFixtureOtherRoot "scripts\release_gate_common.ps1")
    & (Join-Path $configFixtureOtherRoot "scripts\build_config_migrate.ps1") `
        -PythonExecutable $testPythonExecutable `
        -PythonSha256 $testPythonSha256 `
        -OutputPath (Join-Path $configFixtureOtherRoot "tools\ConfigMigrate.exe") | Out-Null
    Assert-True `
        ((Get-ReleaseFileSha256 (Join-Path $configFixture "tools\ConfigMigrate.exe")) -ceq (Get-ReleaseFileSha256 (Join-Path $configFixtureOtherRoot "tools\ConfigMigrate.exe"))) `
        "ConfigMigrate rebuild bytes must be deterministic across linked-worktree root paths"
    Add-Content -LiteralPath (Join-Path $configFixture "tools\migrate_config_to_sqlite.py") -Value "# tamper"
    Assert-Throws { Assert-ConfigMigrateProvenance -RepoRoot $configFixture -ExecutablePath (Join-Path $configFixture "tools\ConfigMigrate.exe") } "ConfigMigrate source tamper must fail"
    Copy-Item -LiteralPath (Join-Path $repoRoot "tools\migrate_config_to_sqlite.py") -Destination (Join-Path $configFixture "tools\migrate_config_to_sqlite.py") -Force
    $fakeSourceHash = Get-ReleaseFileSha256 (Join-Path $configFixture "tools\migrate_config_to_sqlite.py")
    $fakeProbePath = Join-Path $configFixture "tools\ConfigMigrate.exe"
    Remove-Item -LiteralPath $fakeProbePath -Force
    New-FakeConfigMigrateProbe -Path $fakeProbePath -ReportedHash $fakeSourceHash
    Assert-Throws { Assert-ConfigMigrateProvenance -RepoRoot $configFixture -ExecutablePath $fakeProbePath } "single-line self-reported fake ConfigMigrate must fail"

    $gitFixture = Join-Path $tempRoot "git-clean"
    New-Item -ItemType Directory -Path (Join-Path $gitFixture "installer"), (Join-Path $gitFixture "icons") -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $repoRoot "installer\QtWidgetsApplication4.iss") -Destination (Join-Path $gitFixture "installer\QtWidgetsApplication4.iss")
    Copy-Item -LiteralPath (Join-Path $repoRoot "icons\app.ico") -Destination (Join-Path $gitFixture "icons\app.ico")
    Invoke-GitFixture $gitFixture @("init", "-q")
    Invoke-GitFixture $gitFixture @("branch", "-M", "main")
    Invoke-GitFixture $gitFixture @("config", "user.email", "release-gate@example.invalid")
    Invoke-GitFixture $gitFixture @("config", "user.name", "Release Gate Test")
    Invoke-GitFixture $gitFixture @("add", "--", "installer/QtWidgetsApplication4.iss", "icons/app.ico")
    Invoke-GitFixture $gitFixture @("commit", "-q", "-m", "fixture")
    Assert-GitReleaseState -RepoRoot $gitFixture -AppVersion "2026.07.13.2115" -Channel neutral | Out-Null
    Set-Content -LiteralPath (Join-Path $gitFixture "untracked-release-asset.dll") -Value "stale" -NoNewline
    Assert-Throws { Assert-GitReleaseState -RepoRoot $gitFixture -AppVersion "2026.07.13.2115" -Channel neutral } "untracked release asset must make Git state ambiguous"

    $neutralWorktree = Join-Path $tempRoot "linked-neutral"
    $brandWorktree = Join-Path $tempRoot "linked-brand"
    New-Item -ItemType Directory -Path $neutralWorktree -Force | Out-Null
    Invoke-GitFixture $neutralWorktree @("init", "-q")
    Invoke-GitFixture $neutralWorktree @("branch", "-M", "main")
    Invoke-GitFixture $neutralWorktree @("config", "user.email", "release-gate@example.invalid")
    Invoke-GitFixture $neutralWorktree @("config", "user.name", "Release Gate Test")
    Set-Content -LiteralPath (Join-Path $neutralWorktree "base.txt") -Value "base" -NoNewline
    Invoke-GitFixture $neutralWorktree @("add", "base.txt")
    Invoke-GitFixture $neutralWorktree @("commit", "-q", "-m", "base")
    $baseHead = (git -C $neutralWorktree rev-parse HEAD).Trim()
    Invoke-GitFixture $neutralWorktree @("worktree", "add", "-q", "-b", "hk-pathlynx-corpla", $brandWorktree, $baseHead)
    Set-Content -LiteralPath (Join-Path $brandWorktree "brand.txt") -Value "brand" -NoNewline
    Invoke-GitFixture $brandWorktree @("add", "brand.txt")
    Invoke-GitFixture $brandWorktree @("commit", "-q", "-m", "brand")
    Add-Content -LiteralPath (Join-Path $neutralWorktree "base.txt") -Value "security-fix"
    Invoke-GitFixture $neutralWorktree @("add", "base.txt")
    Invoke-GitFixture $neutralWorktree @("commit", "-q", "-m", "security fix")
    $neutralHead = (git -C $neutralWorktree rev-parse HEAD).Trim()
    $brandHead = (git -C $brandWorktree rev-parse HEAD).Trim()
    Assert-Throws {
        Assert-LinkedReleaseWorktrees -NeutralRoot $neutralWorktree -BrandRoot $brandWorktree -NeutralHead $neutralHead -BrandHead $brandHead
    } "brand HEAD that lacks neutral security commit must fail"
    Invoke-GitFixture $brandWorktree @("merge", "-q", "--no-edit", "main")
    $brandHead = (git -C $brandWorktree rev-parse HEAD).Trim()
    $linked = Assert-LinkedReleaseWorktrees -NeutralRoot $neutralWorktree -BrandRoot $brandWorktree -NeutralHead $neutralHead -BrandHead $brandHead
    Assert-True (Test-ReleaseGateSamePath $linked.commonDir (Get-ReleaseGitPath -RepoRoot $neutralWorktree -Kind common)) "linked common-dir binding failed"

    $independentClone = Join-Path $tempRoot "independent-clone"
    & git clone -q $neutralWorktree $independentClone
    if ($LASTEXITCODE -ne 0) { throw "independent clone fixture failed" }
    $cloneHead = (git -C $independentClone rev-parse HEAD).Trim()
    Assert-Throws {
        Assert-LinkedReleaseWorktrees -NeutralRoot $neutralWorktree -BrandRoot $independentClone -NeutralHead $neutralHead -BrandHead $cloneHead
    } "two independent clones must fail linked-worktree gate"

    $canonicalPairPath = Get-ReleasePairGateReportPath -RepoRoot $neutralWorktree -AppVersion "2026.07.12.1200"
    Assert-Throws {
        Assert-ReleasePairOutputPath -CandidatePath $canonicalPairPath -CanonicalPath $canonicalPairPath -ProtectedPaths @($canonicalPairPath)
    } "pair output aliasing an input must fail"
    Assert-Throws {
        Assert-ReleasePairOutputPath -CandidatePath (Join-Path $neutralWorktree "pair.json") -CanonicalPath $canonicalPairPath -ProtectedPaths @()
    } "non-canonical pair output must fail"
    Assert-ReleasePairOutputPath -CandidatePath $canonicalPairPath -CanonicalPath $canonicalPairPath -ProtectedPaths @($neutralWorktree) | Out-Null

    $releaseText = Get-Content -LiteralPath (Join-Path $repoRoot "scripts\build_release_package.ps1") -Raw
    $installerText = Get-Content -LiteralPath (Join-Path $repoRoot "scripts\build_installer.ps1") -Raw
    $configBuilderText = Get-Content -LiteralPath (Join-Path $repoRoot "scripts\build_config_migrate.ps1") -Raw
    $pairText = Get-Content -LiteralPath (Join-Path $repoRoot "scripts\verify_release_pair.ps1") -Raw
    $commonText = Get-Content -LiteralPath (Join-Path $repoRoot "scripts\release_gate_common.ps1") -Raw
    foreach ($token in @("AppVersion", "Channel", "Assert-ExpectedReleaseExecutable", "Assert-EmptyReleaseRuntimeDirectories", "Get-AuthenticodeSignature", "New-PackageGateReport", "/t:Rebuild", "ReleaseVersionMajor", "ReleaseVersionBuild")) {
        Assert-True ($releaseText.Contains($token)) "build_release_package missing hard gate token $token"
    }
    Assert-True ($releaseText.Contains("-SkipBuild is forbidden")) "SkipBuild must fail closed"
    foreach ($token in @("MSBuildExecutable", "MSBuildSha256", "WinDeployQtExecutable", "WinDeployQtSha256", "QtMsBuildPath", "/p:QtMsBuild=", "PythonExecutable", "PythonSha256", "Assert-ReleaseExternalTool")) {
        Assert-True ($releaseText.Contains($token)) "release build must explicitly bind trusted tool $token"
    }
    foreach ($token in @("QtMsBuildPath", "InnoCompilerExecutable", "InnoCompilerSha256", "Assert-ReleaseExternalTool")) {
        Assert-True ($installerText.Contains($token)) "installer build must explicitly bind trusted tool $token"
    }
    Assert-True (-not $releaseText.Contains('$env:QTDIR') -and -not $releaseText.Contains('Find-FirstExistingPath')) "Qt/MSBuild path fallback must be absent"
    Assert-True (-not $installerText.Contains('Get-ItemProperty') -and -not $installerText.Contains('Find-InnoSetupCompiler')) "ISCC registry/path discovery must be absent"
    Assert-True ($configBuilderText.Contains('-I -B -c $isolatedBootstrap') `
        -and $configBuilderText.Contains("[python, '-s', '-P', '-B', '-m', 'PyInstaller'") `
        -and $configBuilderText.Contains('PythonSha256')) "ConfigMigrate must use explicit hash-bound isolated Python"
    Assert-True (-not $installerText.Contains("Set-Content -LiteralPath `$buildInfoPath")) "installer must not relabel package metadata after package gate"
    Assert-True ($installerText.Contains("-PackageGateReport") -and $installerText.Contains("Assert-PackageGateReport")) "SkipPackageBuild must consume a gate report"
    Assert-True ($releaseText.Contains('Remove-Item -LiteralPath $buildDir -Recurse -Force')) "release build directory must be cleaned before Rebuild"
    Assert-True ($releaseText.Contains('Read-FanucRuntimeManifest') -and -not $releaseText.Contains('Get-ChildItem -LiteralPath $fanucSourceDir -File')) "FANUC packaging must use only the versioned manifest"
    Assert-True ($commonText.Contains('FileVersionInfo') -and $commonText.Contains('0x8664') -and -not $commonText.Contains('ASCII.GetString')) "main executable gate must use x64 PE VersionInfo, not byte markers"
    Assert-True ($pairText.Contains("2MB") -and $pairText.Contains("Assert-ReleaseInventoryMatches") -and $pairText.Contains("Assert-LinkedReleaseWorktrees")) "pair verifier must enforce size, inventory, and linked ancestry gates"
    Assert-True ($pairText.IndexOf('Assert-InstallerGateReport') -lt $pairText.IndexOf('Write-ReleaseGateJson -Value $pending')) "pair must validate inputs before writing pending"
    foreach ($token in @(
        'ParameterSetName = "Attest"',
        'PublishChallenge',
        'ExpectedCandidateSha256',
        'ExpectedPairGateSha256',
        'Assert-ExistingReleasePairMatchesLiveState',
        'Resolve-PublishAttestationOutputPath',
        'kind                 = "publish-attestation"')) {
        Assert-True ($pairText.Contains($token)) "pair verifier fresh publish-attestation mode is missing $token"
    }

    Write-Host "PASS: release packaging gates reject stale, incomplete, cross-channel, and tampered fixtures."
}
finally {
    $fullTemp = [System.IO.Path]::GetFullPath($tempRoot)
    $safeParent = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
    if ($fullTemp.StartsWith($safeParent, [System.StringComparison]::OrdinalIgnoreCase) `
        -and [System.IO.Path]::GetFileName($fullTemp).StartsWith("release-gate-tests-")) {
        Remove-Item -LiteralPath $fullTemp -Recurse -Force -ErrorAction SilentlyContinue
    }
}
