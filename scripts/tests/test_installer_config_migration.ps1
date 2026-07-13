[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$helper = Join-Path $repoRoot 'tools\ConfigMigrate_Install.ps1'
$installer = Join-Path $repoRoot 'installer\QtWidgetsApplication4.iss'
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) (
    'NoTeaching Robot Installer Staging Tests (' + [Guid]::NewGuid().ToString('N') + ')'
)
$fakeMigrator = Join-Path $tempRoot 'Fake ConfigMigrate.exe'
$transactionName = 'ConfigStore.db.install-transaction-v1'

function Assert-True {
    param($Condition, [string]$Message)
    if ($Condition -is [array]) {
        $caller = (Get-PSCallStack)[1]
        throw "assertion at line $($caller.ScriptLineNumber) produced an object array"
    }
    if (-not [bool]$Condition) {
        throw $Message
    }
}

function New-DatabaseBytes {
    param([Parameter(Mandatory = $true)][string]$State)
    return [System.Text.Encoding]::ASCII.GetBytes("SQLite format 3`0$State")
}

function Write-DatabaseState {
    param([string]$Path, [string]$State)
    [System.IO.Directory]::CreateDirectory((Split-Path -Parent $Path)) | Out-Null
    [System.IO.File]::WriteAllBytes($Path, (New-DatabaseBytes $State))
}

function Get-DatabaseState {
    param([string]$Path)
    $bytes = [System.IO.File]::ReadAllBytes($Path)
    return [System.Text.Encoding]::ASCII.GetString($bytes, 16, $bytes.Length - 16)
}

function Get-Sha256 {
    param([string]$Path)
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Get-TextSha256 {
    param([string]$Value)
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    try {
        return [System.BitConverter]::ToString(
            $sha256.ComputeHash([System.Text.Encoding]::UTF8.GetBytes($Value))
        ).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $sha256.Dispose()
    }
}

function Read-KeyValueFile {
    param([string]$Path)
    $map = @{}
    foreach ($line in @(Get-Content -LiteralPath $Path -ErrorAction Stop)) {
        $separator = $line.IndexOf('=')
        if ($separator -gt 0) {
            $map[$line.Substring(0, $separator)] = $line.Substring($separator + 1)
        }
        elseif (-not $map.ContainsKey('RESULT')) {
            $map.RESULT = $line
        }
    }
    return $map
}

function Invoke-InstallHelper {
    param(
        [Parameter(Mandatory = $true)][string]$Data,
        [Parameter(Mandatory = $true)][string]$Status,
        [string[]]$Extra = @()
    )
    & powershell.exe -NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $helper `
        -DataDirectory $Data -MigrateExecutable $fakeMigrator -StatusFile $Status @Extra
    return $LASTEXITCODE
}

function ConvertTo-ProcessArgument {
    param([string]$Value)
    if ($Value.Contains('"')) {
        throw 'Test process argument contains an unsupported quote.'
    }
    return '"' + $Value + '"'
}

function Start-InstallHelper {
    param([string]$Data, [string]$Status)
    $arguments = @(
        '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $helper,
        '-DataDirectory', $Data,
        '-MigrateExecutable', $fakeMigrator,
        '-StatusFile', $Status
    )
    $start = New-Object System.Diagnostics.ProcessStartInfo
    $start.FileName = 'powershell.exe'
    $start.Arguments = (($arguments | ForEach-Object { ConvertTo-ProcessArgument ([string]$_) }) -join ' ')
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $start
    Assert-True ($process.Start()) 'could not start crash-window helper process'
    return $process
}

function Wait-ForFile {
    param([string]$Path, [int]$TimeoutSeconds = 20)
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        if (Test-Path -LiteralPath $Path -PathType Leaf) {
            return
        }
        Start-Sleep -Milliseconds 50
    }
    throw "Timed out waiting for crash-window signal: $Path"
}

function Stop-CrashWindowTree {
    param(
        [Parameter(Mandatory = $true)][System.Diagnostics.Process]$HelperProcess,
        [Parameter(Mandatory = $true)][string]$SignalPath
    )
    $holderPid = [int]([System.IO.File]::ReadAllText($SignalPath).Trim())
    & taskkill.exe /PID $HelperProcess.Id /T /F *> $null
    try { $HelperProcess.WaitForExit(10000) | Out-Null } catch {}
    $holder = Get-Process -Id $holderPid -ErrorAction SilentlyContinue
    if ($null -ne $holder) {
        & taskkill.exe /PID $holderPid /F *> $null
        try { Wait-Process -Id $holderPid -Timeout 10 -ErrorAction SilentlyContinue } catch {}
    }
    Assert-True ($null -eq (Get-Process -Id $HelperProcess.Id -ErrorAction SilentlyContinue)) `
        'the killed installer helper is still running before rollback'
    Assert-True ($null -eq (Get-Process -Id $holderPid -ErrorAction SilentlyContinue)) `
        'the successful migration child stdout holder is still running before rollback'
    $HelperProcess.Dispose()
}

function Assert-NoTransactionArtifacts {
    param([string]$Data, [string]$Context)
    $artifacts = @(
        Get-ChildItem -LiteralPath $Data -Force -ErrorAction SilentlyContinue |
            Where-Object {
                $_.Name -ceq $transactionName -or
                $_.Name -cmatch '^\.ConfigStore\.db\.install-(create|upgrade)-[0-9a-f]{32}\.tmp(?:-journal|-wal|-shm)?$' -or
                $_.Name -cmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$'
            }
    )
    $artifactNames = @($artifacts | ForEach-Object { $_.Name }) -join ', '
    Assert-True ($artifacts.Count -eq 0) "$Context left transaction artifacts: $artifactNames"
}

$fakeSource = @'
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;

public static class FakeConfigMigrate
{
    private static string GetValue(string[] args, string name)
    {
        for (int i = 0; i + 1 < args.Length; ++i)
            if (String.Equals(args[i], name, StringComparison.Ordinal)) return args[i + 1];
        return "";
    }

    private static bool Has(string[] args, string name)
    {
        return args.Any(a => String.Equals(a, name, StringComparison.Ordinal));
    }

    private static byte[] Current(string state)
    {
        return Encoding.ASCII.GetBytes("SQLite format 3\0" + state);
    }

    private static string State(string db)
    {
        if (!File.Exists(db)) return "MISSING";
        byte[] bytes = File.ReadAllBytes(db);
        return Encoding.ASCII.GetString(bytes, Math.Min(16, bytes.Length), Math.Max(0, bytes.Length - 16));
    }

    private static void WriteBackup(string path, byte[] database)
    {
        string text = "NoTeaching-Robot ConfigStore DPAPI backup v1\n" +
            new string('a', 64) + "\n" + Convert.ToBase64String(database) + "\n";
        File.WriteAllText(path, text, new UTF8Encoding(false));
    }

    private static byte[] ReadBackup(string path)
    {
        string[] lines = File.ReadAllLines(path, Encoding.UTF8);
        if (lines.Length < 3 || lines[0] != "NoTeaching-Robot ConfigStore DPAPI backup v1")
            throw new InvalidDataException("bad backup");
        return Convert.FromBase64String(lines[2]);
    }

    private static bool HasPlaintextCredential(string source)
    {
        if (!Directory.Exists(source)) return false;
        foreach (string path in Directory.GetFiles(source, "*.ini", SearchOption.AllDirectories))
        {
            string text = File.ReadAllText(path);
            if (text.IndexOf("Password=", StringComparison.OrdinalIgnoreCase) >= 0 ||
                text.IndexOf("Token=", StringComparison.OrdinalIgnoreCase) >= 0)
                return true;
        }
        return false;
    }

    private static void ScrubPlaintextCredentials(string source)
    {
        if (!Directory.Exists(source)) return;
        foreach (string path in Directory.GetFiles(source, "*.ini", SearchOption.AllDirectories))
        {
            string[] lines = File.ReadAllLines(path);
            string[] scrubbed = lines.Where(line =>
                line.IndexOf("Password=", StringComparison.OrdinalIgnoreCase) < 0 &&
                line.IndexOf("Token=", StringComparison.OrdinalIgnoreCase) < 0).ToArray();
            File.WriteAllLines(path, scrubbed, new UTF8Encoding(false));
        }
    }

    private static string Quote(string value)
    {
        return "\"" + value.Replace("\"", "\\\"") + "\"";
    }

    private static void ArmCrashWindow(string kind, string db)
    {
        string requested = Environment.GetEnvironmentVariable("FAKE_CRASH_KIND") ?? "";
        if (!String.Equals(requested, kind, StringComparison.Ordinal)) return;
        if (Environment.GetEnvironmentVariable("FAKE_WRITE_STAGING_SIDECAR") == "1")
            File.WriteAllText(db + "-wal", "staging-only-sidecar", Encoding.ASCII);
        string signal = Environment.GetEnvironmentVariable("FAKE_CRASH_SIGNAL") ?? "";
        string release = Environment.GetEnvironmentVariable("FAKE_CRASH_RELEASE") ?? "";
        ProcessStartInfo start = new ProcessStartInfo();
        start.FileName = Environment.GetCommandLineArgs()[0];
        start.Arguments = "--hold-stdout " + Quote(release);
        start.UseShellExecute = false;
        start.CreateNoWindow = true;
        Process holder = Process.Start(start);
        File.WriteAllText(signal, holder.Id.ToString(), Encoding.ASCII);
    }

    public static int Main(string[] args)
    {
        if (args.Length == 2 && args[0] == "--hold-stdout")
        {
            while (!File.Exists(args[1])) Thread.Sleep(50);
            return 0;
        }

        string db = GetValue(args, "--db");
        if (Has(args, "--restore-dpapi-backup"))
        {
            File.WriteAllBytes(db, ReadBackup(GetValue(args, "--restore-dpapi-backup")));
            return 0;
        }
        if (Has(args, "--verify-installer-state"))
        {
            string source = GetValue(args, "--source");
            string state = State(db);
            if (Environment.GetEnvironmentVariable("FAKE_CREATE_FINAL_DURING_VERIFY") == "1" &&
                Path.GetFileName(db).StartsWith(".ConfigStore.db.install-create-", StringComparison.Ordinal))
            {
                string final = Path.Combine(Path.GetDirectoryName(db), "ConfigStore.db");
                File.WriteAllBytes(final, Encoding.ASCII.GetBytes("external-final-race-sentinel"));
            }
            if (Environment.GetEnvironmentVariable("FAKE_VERIFY_WRITES_SIDECAR") == "1")
                File.WriteAllText(db + "-wal", "verify-sidecar", Encoding.ASCII);
            if (state.StartsWith("DEFERRED", StringComparison.Ordinal))
                Console.WriteLine("Verified installer ConfigStore state (scrub=pending): " + db);
            else if (state.StartsWith("CURRENT", StringComparison.Ordinal) && !HasPlaintextCredential(source))
                Console.WriteLine("Verified installer ConfigStore state (scrub=complete): " + db);
            else
                return 18;
            return 0;
        }
        if (Has(args, "--verify-current"))
        {
            string state = State(db);
            if (Environment.GetEnvironmentVariable("FAKE_CREATE_FINAL_DURING_VERIFY") == "1" &&
                Path.GetFileName(db).StartsWith(".ConfigStore.db.install-create-", StringComparison.Ordinal))
            {
                string final = Path.Combine(Path.GetDirectoryName(db), "ConfigStore.db");
                File.WriteAllBytes(final, Encoding.ASCII.GetBytes("external-final-race-sentinel"));
            }
            if (Environment.GetEnvironmentVariable("FAKE_VERIFY_WRITES_SIDECAR") == "1")
                File.WriteAllText(db + "-wal", "verify-sidecar", Encoding.ASCII);
            if (state.StartsWith("DEFERRED", StringComparison.Ordinal))
                Console.WriteLine("Verified current ConfigStore schema v5 (scrub=pending): " + db);
            else if (state.StartsWith("CURRENT", StringComparison.Ordinal))
                Console.WriteLine("Verified current ConfigStore schema v5 (scrub=complete): " + db);
            return (state.StartsWith("CURRENT", StringComparison.Ordinal) ||
                    state.StartsWith("DEFERRED", StringComparison.Ordinal)) ? 0 : 11;
        }
        if (Has(args, "--scrub-legacy-credentials"))
        {
            if (!State(db).StartsWith("DEFERRED", StringComparison.Ordinal)) return 14;
            File.WriteAllBytes(db, Current("CURRENT_FINALIZED"));
            ScrubPlaintextCredentials(GetValue(args, "--source"));
            return 0;
        }
        if (Has(args, "--defer-credential-scrub"))
        {
            if (!Has(args, "--installer-staging")) return 15;
            bool scrubComplete = Environment.GetEnvironmentVariable("FAKE_STAGING_SCRUB_COMPLETE") == "1";
            if (!File.Exists(db))
            {
                File.WriteAllBytes(db, Current(scrubComplete ? "CURRENT_CREATED" : "DEFERRED_CREATED"));
                ArmCrashWindow("create", db);
                return 0;
            }
            byte[] original = File.ReadAllBytes(db);
            if (State(db).StartsWith("CURRENT", StringComparison.Ordinal)) return 19;
            string backup = GetValue(args, "--upgrade-backup");
            if (String.IsNullOrEmpty(backup)) return 16;
            WriteBackup(backup, original);
            File.WriteAllBytes(db, Current(scrubComplete ? "CURRENT_UPGRADED" : "DEFERRED_UPGRADED"));
            Console.WriteLine("Upgraded existing database to schema v5: " + db);
            Console.WriteLine("Created and verified DPAPI-protected database backup: " + backup);
            ArmCrashWindow("upgrade", db);
            return 0;
        }
        return 17;
    }
}
'@

try {
    New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null
    Add-Type -TypeDefinition $fakeSource -Language CSharp -OutputAssembly $fakeMigrator -OutputType ConsoleApplication

    # Every successful no-change preflight still leaves a durable, consumable record.
    $freshData = Join-Path $tempRoot 'fresh Data'
    $freshStatus = Join-Path $tempRoot 'fresh.status'
    $code = Invoke-InstallHelper $freshData $freshStatus
    Assert-True ($code -eq 0) "fresh preflight failed: $code"
    Assert-True ((Get-Content $freshStatus -Raw).Trim() -ceq 'OK:NO_DATABASE') 'fresh status mismatch'
    Assert-True (Test-Path (Join-Path $freshData $transactionName) -PathType Leaf) 'NOOP_ABSENT record missing'
    Assert-True (-not (Test-Path (Join-Path $freshData 'ConfigStore.db'))) 'fresh preflight created a final database'
    $freshCommitStatus = Join-Path $tempRoot 'fresh-commit.status'
    $code = Invoke-InstallHelper $freshData $freshCommitStatus @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Content $freshCommitStatus -Raw).Trim() -ceq 'OK:PENDING_TRANSACTION_COMMITTED') 'fresh no-op commit status failed'
    Assert-NoTransactionArtifacts $freshData 'fresh no-op commit'
    $code = Invoke-InstallHelper $freshData (Join-Path $tempRoot 'no-record-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -ne 0) 'rollback without a pending record was incorrectly accepted'

    $absentDriftData = Join-Path $tempRoot 'absent drift Data'
    $code = Invoke-InstallHelper $absentDriftData (Join-Path $tempRoot 'absent-drift-pre.status')
    Assert-True ($code -eq 0) 'NOOP_ABSENT drift fixture preflight failed'
    $absentDriftIni = Join-Path $absentDriftData 'appeared-after-preflight.ini'
    [System.IO.File]::WriteAllText($absentDriftIni, 'Password=late-absent-secret')
    $code = Invoke-InstallHelper $absentDriftData (Join-Path $tempRoot 'absent-drift-resume.status')
    Assert-True ($code -ne 0) 'NOOP_ABSENT resume accepted a late legacy credential input'
    Assert-True (Test-Path (Join-Path $absentDriftData $transactionName) -PathType Leaf) 'NOOP_ABSENT resume drift deleted its safety record'
    $code = Invoke-InstallHelper $absentDriftData (Join-Path $tempRoot 'absent-drift-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -ne 0) 'NOOP_ABSENT commit accepted a late legacy credential input'
    Assert-True (-not (Test-Path (Join-Path $absentDriftData 'ConfigStore.db'))) 'NOOP_ABSENT drift manufactured a final database'
    Assert-True (Test-Path (Join-Path $absentDriftData $transactionName) -PathType Leaf) 'NOOP_ABSENT commit drift deleted its safety record'
    Remove-Item -LiteralPath $absentDriftIni -Force
    $code = Invoke-InstallHelper $absentDriftData (Join-Path $tempRoot 'absent-drift-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'NOOP_ABSENT drift cleanup rollback failed after restoring bound input state'
    Assert-NoTransactionArtifacts $absentDriftData 'NOOP_ABSENT drift cleanup'

    $forgedAbsentData = Join-Path $tempRoot 'forged absent baseline Data'
    New-Item -ItemType Directory $forgedAbsentData -Force | Out-Null
    $forgedAbsentPayload = @(
        'FORMAT=NoTeaching-Robot-Install-Transaction-v1'
        'MODE=NOOP_ABSENT'
        ('SOURCE_INVENTORY_SHA256=' + ('1' * 64))
        'ORIGINAL_STATE=ABSENT'
    ) -join "`n"
    [System.IO.File]::WriteAllText(
        (Join-Path $forgedAbsentData $transactionName),
        $forgedAbsentPayload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $forgedAbsentPayload) + "`n",
        [System.Text.UTF8Encoding]::new($false)
    )
    $code = Invoke-InstallHelper $forgedAbsentData (Join-Path $tempRoot 'forged-absent.status')
    Assert-True ($code -ne 0) 'NOOP_ABSENT accepted a validly framed non-empty baseline digest'
    Assert-True (Test-Path (Join-Path $forgedAbsentData $transactionName) -PathType Leaf) 'forged NOOP_ABSENT rejection deleted its safety record'
    $filesystemRoot = [System.IO.Path]::GetPathRoot($tempRoot)
    $code = Invoke-InstallHelper $filesystemRoot (Join-Path $tempRoot 'root-data.status')
    Assert-True ($code -ne 0) 'filesystem root was accepted as an elevated Data directory'

    # A scrub=complete database is not sufficient installer proof by itself.
    # Fresh plaintext credentials in Data must fail both first preflight and a
    # resumable NOOP_CURRENT transaction instead of starting the application.
    $lateCredentialData = Join-Path $tempRoot 'late credential Data'
    $lateCredentialDb = Join-Path $lateCredentialData 'ConfigStore.db'
    Write-DatabaseState $lateCredentialDb 'CURRENT_LATE_CREDENTIAL'
    $lateCredentialHash = Get-Sha256 $lateCredentialDb
    [System.IO.File]::WriteAllText(
        (Join-Path $lateCredentialData 'late-credential.ini'),
        "AdminPassword=late-plaintext`r`nAccessToken=late-token"
    )
    $lateCredentialStatus = Join-Path $tempRoot 'late-credential.status'
    $code = Invoke-InstallHelper $lateCredentialData $lateCredentialStatus
    Assert-True ($code -ne 0) 'fresh plaintext credentials beside scrub=complete database were accepted'
    Assert-True ((Get-Sha256 $lateCredentialDb) -ceq $lateCredentialHash) 'rejected late credentials changed the final database'
    $code = Invoke-InstallHelper $lateCredentialData (Join-Path $tempRoot 'late-credential-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'rejected late-credential preflight could not be rolled back'
    Assert-NoTransactionArtifacts $lateCredentialData 'late-credential first preflight rollback'

    $lateResumeData = Join-Path $tempRoot 'late credential resume Data'
    $lateResumeDb = Join-Path $lateResumeData 'ConfigStore.db'
    Write-DatabaseState $lateResumeDb 'CURRENT_LATE_RESUME'
    $lateResumeStatus = Join-Path $tempRoot 'late-resume.status'
    $code = Invoke-InstallHelper $lateResumeData $lateResumeStatus
    Assert-True ($code -eq 0 -and (Get-Content $lateResumeStatus -Raw).Trim() -ceq 'OK:CURRENT_AND_VERIFIED') 'clean scrub=complete NOOP_CURRENT setup failed'
    $lateResumeIni = Join-Path $lateResumeData 'late-resume.ini'
    [System.IO.File]::WriteAllText($lateResumeIni, 'Password=appeared-after-proof')
    $code = Invoke-InstallHelper $lateResumeData (Join-Path $tempRoot 'late-resume-rejected.status')
    Assert-True ($code -ne 0) 'NOOP_CURRENT resume accepted plaintext credentials added after proof'
    Remove-Item -LiteralPath $lateResumeIni -Force
    $code = Invoke-InstallHelper $lateResumeData (Join-Path $tempRoot 'late-resume-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'NOOP_CURRENT late-credential rejection could not be rolled back'
    Assert-NoTransactionArtifacts $lateResumeData 'late-credential NOOP_CURRENT rollback'

    # Legacy create is staged only. A lost status file cannot prevent independent rollback.
    $legacyData = Join-Path $tempRoot 'legacy create Data'
    New-Item -ItemType Directory $legacyData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $legacyData 'OnlineServicesConfig.ini'), 'AdminPassword=legacy-secret')
    $legacyStatus = Join-Path $tempRoot 'legacy.status'
    $code = Invoke-InstallHelper $legacyData $legacyStatus
    Assert-True ($code -eq 0) "legacy staged create failed: $code status=$((Get-Content $legacyStatus -Raw -ErrorAction SilentlyContinue))"
    $legacyMap = Read-KeyValueFile $legacyStatus
    Assert-True ($legacyMap.RESULT -ceq 'OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED') 'legacy staged status mismatch'
    Assert-True ($legacyMap.FINALIZE -ceq '1') 'legacy staged create did not require deferred scrub'
    Assert-True (-not (Test-Path (Join-Path $legacyData 'ConfigStore.db'))) 'preinstall create modified the final database'
    Assert-True (@(Get-ChildItem $legacyData -Force -Filter '.ConfigStore.db.install-create-*.tmp').Count -eq 1) 'create staging database missing'
    Remove-Item $legacyStatus -Force
    $code = Invoke-InstallHelper $legacyData $legacyStatus @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'status-loss create rollback failed'
    Assert-True ((Get-Content $legacyStatus -Raw).Trim() -ceq 'OK:PENDING_CREATE_REMOVED') 'create rollback status mismatch'
    Assert-True (-not (Test-Path (Join-Path $legacyData 'ConfigStore.db'))) 'create rollback manufactured a final database'
    Assert-True ((Get-Content (Join-Path $legacyData 'OnlineServicesConfig.ini') -Raw).Contains('legacy-secret')) 'create rollback scrubbed legacy input'
    Assert-NoTransactionArtifacts $legacyData 'create rollback'

    foreach ($sourceDriftMode in @('resume', 'commit')) {
        $sourceDriftData = Join-Path $tempRoot ("create source $sourceDriftMode drift Data")
        New-Item -ItemType Directory $sourceDriftData -Force | Out-Null
        $sourceDriftIni = Join-Path $sourceDriftData 'Process.ini'
        $sourceDriftOriginal = "[Network]`r`nBaseURL=http://127.0.0.1:8090/original`r`nRobotModel=R2000`r`n"
        [System.IO.File]::WriteAllText($sourceDriftIni, $sourceDriftOriginal)
        $code = Invoke-InstallHelper $sourceDriftData (Join-Path $tempRoot ("create-source-$sourceDriftMode-pre.status"))
        Assert-True ($code -eq 0) "create source-$sourceDriftMode drift staging failed"
        [System.IO.File]::WriteAllText($sourceDriftIni, $sourceDriftOriginal.Replace('/original', '/changed'))
        if ($sourceDriftMode -ceq 'resume') {
            $code = Invoke-InstallHelper $sourceDriftData (Join-Path $tempRoot 'create-source-resume-rejected.status')
        }
        else {
            $code = Invoke-InstallHelper $sourceDriftData (Join-Path $tempRoot 'create-source-commit-rejected.status') @('-CommitPendingTransaction')
        }
        Assert-True ($code -ne 0) "create $sourceDriftMode accepted changed non-credential migration input"
        Assert-True (-not (Test-Path (Join-Path $sourceDriftData 'ConfigStore.db'))) "create $sourceDriftMode drift published a final database"
        Assert-True (Test-Path (Join-Path $sourceDriftData $transactionName) -PathType Leaf) "create $sourceDriftMode drift deleted its safety record"
        [System.IO.File]::WriteAllText($sourceDriftIni, $sourceDriftOriginal)
        $code = Invoke-InstallHelper $sourceDriftData (Join-Path $tempRoot ("create-source-$sourceDriftMode-rollback.status")) @('-RollbackPendingTransaction')
        Assert-True ($code -eq 0) "create source-$sourceDriftMode drift cleanup rollback failed"
        Assert-NoTransactionArtifacts $sourceDriftData "create source-$sourceDriftMode drift cleanup"
    }

    # Post-file commit publishes the verified create atomically; only then may scrub finalize.
    $code = Invoke-InstallHelper $legacyData $legacyStatus
    Assert-True ($code -eq 0) 'second staged create failed'
    $createCommitStatus = Join-Path $tempRoot 'create-commit.status'
    $code = Invoke-InstallHelper $legacyData $createCommitStatus @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Content $createCommitStatus -Raw).Trim() -ceq 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED') 'staged create commit did not require finalize'
    $legacyDb = Join-Path $legacyData 'ConfigStore.db'
    Assert-True ((Get-DatabaseState $legacyDb).StartsWith('DEFERRED')) 'create commit did not publish the staged database'
    Assert-True (Test-Path (Join-Path $legacyData $transactionName) -PathType Leaf) 'pending create commit removed its finalize safety record'
    Assert-True (@(Get-ChildItem $legacyData -Force -Filter '.ConfigStore.db.install-create-*.tmp').Count -eq 0) 'pending create commit left its staging database'
    $code = Invoke-InstallHelper $legacyData (Join-Path $tempRoot 'duplicate-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Test-Path (Join-Path $legacyData $transactionName) -PathType Leaf)) 'idempotent pending create commit did not preserve finalize state'
    $code = Invoke-InstallHelper $legacyData (Join-Path $tempRoot 'create-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and (Get-DatabaseState $legacyDb).StartsWith('CURRENT_FINALIZED')) 'create deferred scrub failed after commit'
    Assert-NoTransactionArtifacts $legacyData 'create finalize'
    $code = Invoke-InstallHelper $legacyData (Join-Path $tempRoot 'post-finalize-duplicate-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -ne 0) 'commit without a pending record was incorrectly accepted after finalize'

    # Upgrade also leaves the original final byte-for-byte intact until commit.
    $upgradeData = Join-Path $tempRoot 'upgrade Data'
    $upgradeDb = Join-Path $upgradeData 'ConfigStore.db'
    Write-DatabaseState $upgradeDb 'OLD_SCHEMA4'
    $upgradeOriginalHash = Get-Sha256 $upgradeDb
    $upgradeStatus = Join-Path $tempRoot 'upgrade.status'
    $code = Invoke-InstallHelper $upgradeData $upgradeStatus
    Assert-True ($code -eq 0) "staged upgrade failed: $code"
    $upgradeMap = Read-KeyValueFile $upgradeStatus
    Assert-True ($upgradeMap.RESULT -ceq 'OK:MIGRATED_AND_VERIFIED' -and $upgradeMap.FINALIZE -ceq '1') 'upgrade status contract mismatch'
    Assert-True ((Get-Sha256 $upgradeDb) -ceq $upgradeOriginalHash) 'preinstall upgrade changed the final database'
    Assert-True (@(Get-ChildItem $upgradeData -Force -Filter '.ConfigStore.db.install-upgrade-*.tmp').Count -eq 1) 'upgrade staging database missing'
    Remove-Item $upgradeStatus -Force
    $code = Invoke-InstallHelper $upgradeData $upgradeStatus @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Sha256 $upgradeDb) -ceq $upgradeOriginalHash) 'status-loss upgrade rollback changed the original final database'
    Assert-NoTransactionArtifacts $upgradeData 'upgrade rollback'

    # Exercise the real upgrade publish and the idempotent post-publish recovery branch.
    $code = Invoke-InstallHelper $upgradeData $upgradeStatus
    Assert-True ($code -eq 0 -and (Get-Sha256 $upgradeDb) -ceq $upgradeOriginalHash) 'upgrade restaging failed'
    $upgradeCommitStatus = Join-Path $tempRoot 'upgrade-commit.status'
    $code = Invoke-InstallHelper $upgradeData $upgradeCommitStatus @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-DatabaseState $upgradeDb).StartsWith('DEFERRED_UPGRADED') -and (Get-Content $upgradeCommitStatus -Raw).Trim() -ceq 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED') 'upgrade atomic publish did not require finalize'
    Assert-True (Test-Path (Join-Path $upgradeData $transactionName) -PathType Leaf) 'pending upgrade commit removed its finalize safety record'
    $code = Invoke-InstallHelper $upgradeData (Join-Path $tempRoot 'upgrade-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path (Join-Path $upgradeData $transactionName))) 'upgrade deferred scrub failed to clear durable finalize state'

    $pendingNoopData = Join-Path $tempRoot 'pending noop Data'
    $pendingNoopDb = Join-Path $pendingNoopData 'ConfigStore.db'
    Write-DatabaseState $pendingNoopDb 'DEFERRED_EXISTING'
    [System.IO.File]::WriteAllText((Join-Path $pendingNoopData 'legacy.ini'), 'Password=pending-noop')
    $pendingNoopStatus = Join-Path $tempRoot 'pending-noop-pre.status'
    $code = Invoke-InstallHelper $pendingNoopData $pendingNoopStatus
    Assert-True ($code -eq 0 -and (Read-KeyValueFile $pendingNoopStatus).FINALIZE -ceq '1') 'pending NOOP_CURRENT preflight failed'
    $pendingNoopCommitStatus = Join-Path $tempRoot 'pending-noop-commit.status'
    $code = Invoke-InstallHelper $pendingNoopData $pendingNoopCommitStatus @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Test-Path (Join-Path $pendingNoopData $transactionName) -PathType Leaf) -and (Get-Content $pendingNoopCommitStatus -Raw).Trim() -ceq 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED') 'pending NOOP_CURRENT commit did not require finalize'
    $code = Invoke-InstallHelper $pendingNoopData (Join-Path $tempRoot 'pending-noop-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path (Join-Path $pendingNoopData $transactionName))) 'pending NOOP_CURRENT finalize failed'

    # Installer staging may legitimately be scrub=complete when no legacy
    # credentials need removal. Those transactions commit without FINALIZE=1.
    $completeCreateData = Join-Path $tempRoot 'complete create Data'
    New-Item -ItemType Directory $completeCreateData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $completeCreateData 'Process.ini'), "[General]`r`nMode=clean`r`n")
    $env:FAKE_STAGING_SCRUB_COMPLETE = '1'
    $completeCreateStatus = Join-Path $tempRoot 'complete-create.status'
    $code = Invoke-InstallHelper $completeCreateData $completeCreateStatus
    Remove-Item Env:FAKE_STAGING_SCRUB_COMPLETE -ErrorAction SilentlyContinue
    $completeCreateMap = Read-KeyValueFile $completeCreateStatus
    Assert-True ($code -eq 0 -and $completeCreateMap.RESULT -ceq 'OK:LEGACY_DATABASE_CREATED_AND_VERIFIED') 'scrub-complete staged create status mismatch'
    Assert-True (-not $completeCreateMap.ContainsKey('FINALIZE')) 'scrub-complete staged create incorrectly requested finalize'
    $completeCreateCommitStatus = Join-Path $tempRoot 'complete-create-commit.status'
    $code = Invoke-InstallHelper $completeCreateData $completeCreateCommitStatus @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-DatabaseState (Join-Path $completeCreateData 'ConfigStore.db')).StartsWith('CURRENT_CREATED') -and (Get-Content $completeCreateCommitStatus -Raw).Trim() -ceq 'OK:PENDING_TRANSACTION_COMMITTED') 'scrub-complete staged create commit status failed'

    $completeCommitRejectData = Join-Path $tempRoot 'complete commit reject Data'
    New-Item -ItemType Directory $completeCommitRejectData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $completeCommitRejectData 'Process.ini'), "[General]`r`nMode=clean`r`n")
    $env:FAKE_STAGING_SCRUB_COMPLETE = '1'
    $code = Invoke-InstallHelper $completeCommitRejectData (Join-Path $tempRoot 'complete-commit-reject-pre.status')
    Remove-Item Env:FAKE_STAGING_SCRUB_COMPLETE -ErrorAction SilentlyContinue
    Assert-True ($code -eq 0) 'scrub-complete commit-rejection staging setup failed'
    $lateCommitIni = Join-Path $completeCommitRejectData 'late-before-commit.ini'
    [System.IO.File]::WriteAllText($lateCommitIni, 'Token=late-before-commit')
    $code = Invoke-InstallHelper $completeCommitRejectData (Join-Path $tempRoot 'complete-commit-rejected.status') @('-CommitPendingTransaction')
    Assert-True ($code -ne 0) 'commit accepted plaintext credentials added after scrub=complete staging proof'
    Assert-True (-not (Test-Path (Join-Path $completeCommitRejectData 'ConfigStore.db'))) 'rejected scrub=complete commit published the final database'
    Remove-Item -LiteralPath $lateCommitIni -Force
    $code = Invoke-InstallHelper $completeCommitRejectData (Join-Path $tempRoot 'complete-commit-reject-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'rejected scrub=complete commit could not be rolled back'
    Assert-NoTransactionArtifacts $completeCommitRejectData 'scrub-complete commit rejection rollback'

    $completeUpgradeData = Join-Path $tempRoot 'complete upgrade Data'
    $completeUpgradeDb = Join-Path $completeUpgradeData 'ConfigStore.db'
    Write-DatabaseState $completeUpgradeDb 'OLD_COMPLETE_UPGRADE'
    $env:FAKE_STAGING_SCRUB_COMPLETE = '1'
    $completeUpgradeStatus = Join-Path $tempRoot 'complete-upgrade.status'
    $code = Invoke-InstallHelper $completeUpgradeData $completeUpgradeStatus
    Remove-Item Env:FAKE_STAGING_SCRUB_COMPLETE -ErrorAction SilentlyContinue
    $completeUpgradeMap = Read-KeyValueFile $completeUpgradeStatus
    Assert-True ($code -eq 0 -and $completeUpgradeMap.RESULT -ceq 'OK:MIGRATED_AND_VERIFIED') 'scrub-complete staged upgrade status mismatch'
    Assert-True (-not $completeUpgradeMap.ContainsKey('FINALIZE')) 'scrub-complete staged upgrade incorrectly requested finalize'
    $code = Invoke-InstallHelper $completeUpgradeData (Join-Path $tempRoot 'complete-upgrade-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-DatabaseState $completeUpgradeDb).StartsWith('CURRENT_UPGRADED')) 'scrub-complete staged upgrade commit failed'

    $idempotentData = Join-Path $tempRoot 'idempotent upgrade Data'
    $idempotentDb = Join-Path $idempotentData 'ConfigStore.db'
    Write-DatabaseState $idempotentDb 'OLD_SCHEMA4_IDEMPOTENT'
    $code = Invoke-InstallHelper $idempotentData (Join-Path $tempRoot 'idempotent-pre.status')
    Assert-True ($code -eq 0) 'idempotent upgrade staging failed'
    $record = Read-KeyValueFile (Join-Path $idempotentData $transactionName)
    $idempotentStaging = Join-Path $idempotentData $record.STAGING_NAME
    $idempotentTestBackup = $idempotentDb + '.test-replace-backup'
    [System.IO.File]::Replace($idempotentStaging, $idempotentDb, $idempotentTestBackup)
    Remove-Item -LiteralPath $idempotentTestBackup -Force
    $idempotentResumeStatus = Join-Path $tempRoot 'idempotent-resume.status'
    $code = Invoke-InstallHelper $idempotentData $idempotentResumeStatus
    Assert-True ($code -eq 0 -and (Read-KeyValueFile $idempotentResumeStatus).RESULT -ceq 'OK:MIGRATED_AND_VERIFIED') 'normal preinstall did not resume an exactly published upgrade'
    Assert-True (Test-Path (Join-Path $idempotentData $transactionName) -PathType Leaf) 'published upgrade resume removed its record before ssPostInstall commit'
    $code = Invoke-InstallHelper $idempotentData (Join-Path $tempRoot 'idempotent-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Test-Path (Join-Path $idempotentData $transactionName) -PathType Leaf)) 'idempotent upgrade commit did not persist finalize state'
    $code = Invoke-InstallHelper $idempotentData (Join-Path $tempRoot 'idempotent-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path (Join-Path $idempotentData $transactionName))) 'idempotent upgrade deferred scrub failed'

    $idempotentCreateData = Join-Path $tempRoot 'idempotent create Data'
    New-Item -ItemType Directory $idempotentCreateData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $idempotentCreateData 'legacy.ini'), 'Password=publish-crash')
    $code = Invoke-InstallHelper $idempotentCreateData (Join-Path $tempRoot 'idempotent-create-pre.status')
    Assert-True ($code -eq 0) 'idempotent create staging failed'
    $createRecord = Read-KeyValueFile (Join-Path $idempotentCreateData $transactionName)
    $idempotentCreateStaging = Join-Path $idempotentCreateData $createRecord.STAGING_NAME
    $idempotentCreateFinal = Join-Path $idempotentCreateData 'ConfigStore.db'
    [System.IO.File]::Move($idempotentCreateStaging, $idempotentCreateFinal)
    $idempotentCreateResumeStatus = Join-Path $tempRoot 'idempotent-create-resume.status'
    $code = Invoke-InstallHelper $idempotentCreateData $idempotentCreateResumeStatus
    Assert-True ($code -eq 0 -and (Read-KeyValueFile $idempotentCreateResumeStatus).RESULT -ceq 'OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED') 'normal preinstall did not resume an exactly published create'
    Assert-True (Test-Path (Join-Path $idempotentCreateData $transactionName) -PathType Leaf) 'published create resume removed its record before ssPostInstall commit'
    $code = Invoke-InstallHelper $idempotentCreateData (Join-Path $tempRoot 'idempotent-create-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Test-Path (Join-Path $idempotentCreateData $transactionName) -PathType Leaf)) 'idempotent create publish-crash recovery did not persist finalize state'
    $code = Invoke-InstallHelper $idempotentCreateData (Join-Path $tempRoot 'idempotent-create-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path (Join-Path $idempotentCreateData $transactionName))) 'idempotent create deferred scrub failed'

    $finalizeCrashData = Join-Path $tempRoot 'finalize crash Data'
    New-Item -ItemType Directory $finalizeCrashData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $finalizeCrashData 'legacy.ini'), 'Password=finalize-crash')
    $code = Invoke-InstallHelper $finalizeCrashData (Join-Path $tempRoot 'finalize-crash-pre.status')
    Assert-True ($code -eq 0) 'finalize-crash staging failed'
    $code = Invoke-InstallHelper $finalizeCrashData (Join-Path $tempRoot 'finalize-crash-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and (Test-Path (Join-Path $finalizeCrashData $transactionName) -PathType Leaf)) 'finalize-crash commit did not persist its record'
    & $fakeMigrator --source $finalizeCrashData --db (Join-Path $finalizeCrashData 'ConfigStore.db') --encrypt --scrub-legacy-credentials
    Assert-True ($LASTEXITCODE -eq 0 -and (Test-Path (Join-Path $finalizeCrashData $transactionName) -PathType Leaf)) 'finalize-crash fixture could not simulate durable scrub before record deletion'
    $finalizeCrashResume = Join-Path $tempRoot 'finalize-crash-resume.status'
    $code = Invoke-InstallHelper $finalizeCrashData $finalizeCrashResume
    Assert-True ($code -eq 0 -and (Read-KeyValueFile $finalizeCrashResume).RESULT -ceq 'OK:CURRENT_AND_VERIFIED') 'completed finalize transaction could not resume after record-deletion crash'
    $code = Invoke-InstallHelper $finalizeCrashData (Join-Path $tempRoot 'finalize-crash-reconcile.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0 -and -not (Test-Path (Join-Path $finalizeCrashData $transactionName))) 'completed finalize transaction record was not safely reconciled'

    # If the retrying installer is cancelled after default preflight resumes an
    # already-published pending record, Deinitialize must preserve both the exact
    # new DB and a durable finalize record so manual launch remains fail-closed.
    foreach ($publishedCancelKind in @('create', 'upgrade')) {
        $publishedCancelData = Join-Path $tempRoot ("published cancel $publishedCancelKind Data")
        $publishedCancelFinal = Join-Path $publishedCancelData 'ConfigStore.db'
        if ($publishedCancelKind -ceq 'create') {
            New-Item -ItemType Directory $publishedCancelData -Force | Out-Null
            [System.IO.File]::WriteAllText((Join-Path $publishedCancelData 'legacy.ini'), 'Password=cancel-published')
        }
        else {
            Write-DatabaseState $publishedCancelFinal 'OLD_PUBLISHED_CANCEL'
        }
        $code = Invoke-InstallHelper $publishedCancelData (Join-Path $tempRoot ("published-cancel-$publishedCancelKind-pre.status"))
        Assert-True ($code -eq 0) "published-cancel $publishedCancelKind staging failed"
        $publishedCancelRecord = Read-KeyValueFile (Join-Path $publishedCancelData $transactionName)
        $publishedCancelStaging = Join-Path $publishedCancelData $publishedCancelRecord.STAGING_NAME
        if ($publishedCancelKind -ceq 'create') {
            [System.IO.File]::Move($publishedCancelStaging, $publishedCancelFinal)
        }
        else {
            $publishedCancelTestBackup = $publishedCancelFinal + '.test-replace-backup'
            [System.IO.File]::Replace($publishedCancelStaging, $publishedCancelFinal, $publishedCancelTestBackup)
            Remove-Item -LiteralPath $publishedCancelTestBackup -Force
        }
        $publishedHash = Get-Sha256 $publishedCancelFinal
        $code = Invoke-InstallHelper $publishedCancelData (Join-Path $tempRoot ("published-cancel-$publishedCancelKind-resume.status"))
        Assert-True ($code -eq 0) "default preflight did not resume published $publishedCancelKind before cancellation"
        $cancelStatus = Join-Path $tempRoot ("published-cancel-$publishedCancelKind-rollback.status")
        $code = Invoke-InstallHelper $publishedCancelData $cancelStatus @('-RollbackPendingTransaction')
        Assert-True ($code -eq 0) "published $publishedCancelKind cancellation reconciliation failed"
        Assert-True ((Get-Content $cancelStatus -Raw).Trim() -ceq 'OK:PENDING_PUBLISHED_PRESERVED') "published $publishedCancelKind cancellation returned wrong status"
        Assert-True ((Get-Sha256 $publishedCancelFinal) -ceq $publishedHash) "published $publishedCancelKind cancellation changed the new final database"
        Assert-True (Test-Path (Join-Path $publishedCancelData $transactionName) -PathType Leaf) "published $publishedCancelKind cancellation removed its finalize safety record"
        $publishedCancelRetry = Join-Path $tempRoot ("published-cancel-$publishedCancelKind-retry.status")
        $code = Invoke-InstallHelper $publishedCancelData $publishedCancelRetry
        Assert-True ($code -eq 0 -and (Read-KeyValueFile $publishedCancelRetry).FINALIZE -ceq '1') "published $publishedCancelKind cancellation could not resume durable finalize"
        $code = Invoke-InstallHelper $publishedCancelData (Join-Path $tempRoot ("published-cancel-$publishedCancelKind-commit.status")) @('-CommitPendingTransaction')
        Assert-True ($code -eq 0 -and (Test-Path (Join-Path $publishedCancelData $transactionName) -PathType Leaf)) "published $publishedCancelKind retry commit removed pending finalize state"
        $code = Invoke-InstallHelper $publishedCancelData (Join-Path $tempRoot ("published-cancel-$publishedCancelKind-finalize.status")) @('-FinalizeDeferredScrub')
        Assert-True ($code -eq 0 -and -not (Test-Path (Join-Path $publishedCancelData $transactionName))) "published $publishedCancelKind cancellation finalize recovery failed"
    }

    $createRaceData = Join-Path $tempRoot 'create publish race Data'
    New-Item -ItemType Directory $createRaceData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $createRaceData 'legacy.ini'), 'Password=create-race')
    $code = Invoke-InstallHelper $createRaceData (Join-Path $tempRoot 'create-race-pre.status')
    Assert-True ($code -eq 0) 'create publish-race staging failed'
    $env:FAKE_CREATE_FINAL_DURING_VERIFY = '1'
    $createRaceCommitStatus = Join-Path $tempRoot 'create-race-commit.status'
    $code = Invoke-InstallHelper $createRaceData $createRaceCommitStatus @('-CommitPendingTransaction')
    Remove-Item Env:FAKE_CREATE_FINAL_DURING_VERIFY -ErrorAction SilentlyContinue
    $createRaceFinal = Join-Path $createRaceData 'ConfigStore.db'
    Assert-True ($code -ne 0) 'create publish overwrote a final database that appeared after precheck'
    Assert-True ([System.Text.Encoding]::ASCII.GetString([System.IO.File]::ReadAllBytes($createRaceFinal)) -ceq 'external-final-race-sentinel') 'create no-replace publish changed the racing final database'
    Assert-True (Test-Path (Join-Path $createRaceData $transactionName) -PathType Leaf) 'create publish race lost its verified transaction record'
    Assert-True (@(Get-ChildItem $createRaceData -Force -Filter '.ConfigStore.db.install-create-*.tmp').Count -eq 1) 'create publish race lost its staging database'
    Remove-Item -LiteralPath $createRaceFinal -Force
    $code = Invoke-InstallHelper $createRaceData (Join-Path $tempRoot 'create-race-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'create publish-race cleanup rollback failed after external final was removed'
    Assert-NoTransactionArtifacts $createRaceData 'create publish-race cleanup'

    # Kill the helper after the migration child has exited successfully but before VERIFIED advancement.
    foreach ($crashKind in @('create', 'upgrade')) {
        $crashData = Join-Path $tempRoot ("forced kill $crashKind Data")
        $crashDb = Join-Path $crashData 'ConfigStore.db'
        if ($crashKind -ceq 'create') {
            New-Item -ItemType Directory $crashData -Force | Out-Null
            [System.IO.File]::WriteAllText((Join-Path $crashData 'legacy.ini'), 'password=must-remain')
            $originalHash = ''
        }
        else {
            Write-DatabaseState $crashDb 'OLD_SCHEMA4_CRASH'
            $originalHash = Get-Sha256 $crashDb
        }
        $signal = Join-Path $tempRoot ("$crashKind-child-success.signal")
        $release = Join-Path $tempRoot ("$crashKind-child-release.signal")
        $env:FAKE_CRASH_KIND = $crashKind
        $env:FAKE_CRASH_SIGNAL = $signal
        $env:FAKE_CRASH_RELEASE = $release
        $env:FAKE_WRITE_STAGING_SIDECAR = '1'
        $crashProcess = Start-InstallHelper $crashData (Join-Path $tempRoot ("$crashKind-crash.status"))
        Wait-ForFile $signal
        if ($crashKind -ceq 'create') {
            Assert-True (-not (Test-Path $crashDb)) 'forced-kill create changed final database before VERIFIED'
        }
        else {
            Assert-True ((Get-Sha256 $crashDb) -ceq $originalHash) 'forced-kill upgrade changed final database before VERIFIED'
        }
        Stop-CrashWindowTree $crashProcess $signal
        Remove-Item Env:FAKE_CRASH_KIND,Env:FAKE_CRASH_SIGNAL,Env:FAKE_CRASH_RELEASE,Env:FAKE_WRITE_STAGING_SIDECAR -ErrorAction SilentlyContinue
        $rollbackCode = Invoke-InstallHelper $crashData (Join-Path $tempRoot ("$crashKind-crash-rollback.status")) @('-RollbackPendingTransaction')
        Assert-True ($rollbackCode -eq 0) "forced-kill $crashKind rollback failed"
        if ($crashKind -ceq 'create') {
            Assert-True (-not (Test-Path $crashDb)) 'forced-kill create rollback changed absent final state'
            Assert-True ((Get-Content (Join-Path $crashData 'legacy.ini') -Raw).Contains('must-remain')) 'forced-kill create scrubbed legacy input'
        }
        else {
            Assert-True ((Get-Sha256 $crashDb) -ceq $originalHash) 'forced-kill upgrade rollback changed original final hash'
        }
        Assert-NoTransactionArtifacts $crashData "forced-kill $crashKind rollback"
    }

    # A final-database WAL/journal/SHM is never replayed across an installer generation.
    $sidecarData = Join-Path $tempRoot 'final sidecar Data'
    $sidecarDb = Join-Path $sidecarData 'ConfigStore.db'
    Write-DatabaseState $sidecarDb 'OLD_WITH_WAL'
    $sidecarHash = Get-Sha256 $sidecarDb
    [System.IO.File]::WriteAllText($sidecarDb + '-wal', 'old-wal')
    $code = Invoke-InstallHelper $sidecarData (Join-Path $tempRoot 'final-wal.status')
    Assert-True ($code -ne 0 -and (Get-Sha256 $sidecarDb) -ceq $sidecarHash) 'final WAL was accepted or replayed'
    Assert-True (-not (Test-Path (Join-Path $sidecarData $transactionName))) 'final WAL rejection created a transaction record'
    $orphanSidecarData = Join-Path $tempRoot 'orphan sidecar Data'
    New-Item -ItemType Directory $orphanSidecarData -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $orphanSidecarData 'ConfigStore.db-journal'), 'orphan')
    $code = Invoke-InstallHelper $orphanSidecarData (Join-Path $tempRoot 'orphan-journal.status')
    Assert-True ($code -ne 0) 'orphan final journal was accepted'

    $verifySidecarData = Join-Path $tempRoot 'verify sidecar Data'
    $verifySidecarDb = Join-Path $verifySidecarData 'ConfigStore.db'
    Write-DatabaseState $verifySidecarDb 'CURRENT_VERIFICATION_SIDECAR'
    $env:FAKE_VERIFY_WRITES_SIDECAR = '1'
    $code = Invoke-InstallHelper $verifySidecarData (Join-Path $tempRoot 'verify-sidecar.status')
    Remove-Item Env:FAKE_VERIFY_WRITES_SIDECAR -ErrorAction SilentlyContinue
    Assert-True ($code -ne 0) 'a sidecar created during read-back verification was accepted'
    Assert-True (-not (Test-Path (Join-Path $verifySidecarData $transactionName))) 'verification-sidecar failure created a no-op transaction record'
    Remove-Item -LiteralPath ($verifySidecarDb + '-wal') -Force

    # Independent record corruption must fail closed without changing the final database.
    $tamperData = Join-Path $tempRoot 'record tamper Data'
    $tamperDb = Join-Path $tamperData 'ConfigStore.db'
    Write-DatabaseState $tamperDb 'OLD_TAMPER'
    $tamperHash = Get-Sha256 $tamperDb
    $code = Invoke-InstallHelper $tamperData (Join-Path $tempRoot 'tamper-pre.status')
    Assert-True ($code -eq 0) 'tamper fixture preflight failed'
    Add-Content -LiteralPath (Join-Path $tamperData $transactionName) -Value 'UNKNOWN=1' -Encoding ASCII
    $code = Invoke-InstallHelper $tamperData (Join-Path $tempRoot 'tamper-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -ne 0 -and (Get-Sha256 $tamperDb) -ceq $tamperHash) 'tampered transaction record was accepted or changed final database'

    # Static installer contract: every helper-started failure rolls back by record; commit precedes scrub.
    $helperText = Get-Content -LiteralPath $helper -Raw
    Assert-True ($helperText.Contains("'--installer-staging'")) 'helper does not use the restricted installer-staging CLI'
    Assert-True ($helperText.Contains("'--verify-installer-state'") -and
        $helperText.Contains("'--source', `$script:DataPath") -and
        (($helperText.Split('Invoke-InstallerStateVerification').Count - 1) -ge 12)) `
        'helper does not use source-aware installer-state verification for every lifecycle readback'
    Assert-True (-not $helperText.Contains("'--verify-current'")) 'helper still uses database-only verification in an installer lifecycle'
    Assert-True ($helperText.Contains('SOURCE_INVENTORY_SHA256=') -and
        $helperText.Contains('Assert-NoopAbsentSourceInventory') -and
        $helperText.Contains('Assert-CreateSourceInventory')) `
        'absent/create transactions do not bind exact Data input inventory'
    Assert-True ($helperText.Contains('MODE=PUBLISHED_FINALIZE_PENDING') -and
        $helperText.Contains('Write-PublishedFinalizePending')) `
        'published pending scrub is not persisted across commit/finalize'
    Assert-True ($helperText.Contains('[System.IO.File]::Copy($script:DatabasePath, $script:UpgradeStagingPath, $false)')) 'upgrade does not copy final database to staging'
    Assert-True ($helperText.Contains('Invoke-AtomicFileReplace $stagingPath $script:DatabasePath')) 'upgrade commit is not an atomic same-directory replace'
    Assert-True ($helperText.Contains('Invoke-AtomicFileMoveNoReplace $stagingPath $script:DatabasePath')) 'create commit can overwrite a racing final database'
    Assert-True (-not $helperText.Contains('[System.IO.File]::Move($temporaryPath, $Path)')) 'initial PREPARED record does not use write-through atomic move'
    Assert-True (-not $helperText.Contains('AllowNoPendingTransaction')) 'helper still permits unproven no-record rollback success'
    Assert-True ($helperText.Contains("'MODE=UPGRADE_PREPARED'") -and $helperText.Contains("'MODE=UPGRADE_VERIFIED'")) 'upgrade write-ahead phases are missing'
    Assert-True ($helperText.Contains("'MODE=CREATE_PREPARED'") -and $helperText.Contains("'MODE=CREATE_VERIFIED'")) 'create write-ahead phases are missing'

    $iss = Get-Content -LiteralPath $installer -Raw
    Assert-True (-not $iss.Contains('AllowNoPendingTransaction')) 'installer still permits no-record rollback success'
    Assert-True ($iss.Contains("RollbackStatus = 'OK:PENDING_PUBLISHED_PRESERVED'")) 'installer cannot safely reconcile a previously published transaction on cancellation'
    Assert-True ($iss.Contains("CommitStatus = 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED'") -and
        $iss.Contains('MigrationNeedsFinalize := True;')) `
        'installer does not let final commit state require deferred cleanup'
    Assert-True (($iss.Split('AppendPreparationRollbackOutcome(Result);').Count - 1) -ge 4) 'not every post-helper preparation failure reconciles the transaction record'
    $postInstall = $iss.IndexOf('if CurStep = ssPostInstall then')
    $commit = $iss.IndexOf('if not CommitPendingDatabaseTransaction(CommitStatus) then', $postInstall)
    $finalize = $iss.IndexOf('not FinalizeDeferredCredentialScrub', $postInstall)
    Assert-True ($postInstall -ge 0 -and $commit -gt $postInstall -and $finalize -gt $commit) 'database publish does not precede deferred scrub in ssPostInstall'
    $deinitialize = $iss.IndexOf('procedure DeinitializeSetup;')
    Assert-True ($iss.IndexOf('if RollbackPendingDatabaseTransaction(RollbackStatus) then', $deinitialize) -gt $deinitialize) 'cancellation rollback depends on parsed status fields'
    Assert-True (-not $iss.Contains("if Pos('OK:', StatusText)")) 'installer accepts arbitrary OK-prefixed status'

    function Test-InstallerStatusContract {
        param([string]$Status)
        if ($Status -cin @('OK:NO_DATABASE', 'OK:CURRENT_AND_VERIFIED', "OK:CURRENT_AND_VERIFIED`nFINALIZE=1")) {
            return $true
        }
        if ($Status -cmatch '^OK:MIGRATED_AND_VERIFIED\nBACKUP_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak\nBACKUP_SHA256=[0-9a-f]{64}(?:\nFINALIZE=1)?$') {
            return $true
        }
        if ($Status -cmatch '^OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED\nDATABASE_CREATED=1\nCREATED_SHA256=[0-9a-f]{64}\nFINALIZE=1$') {
            return $true
        }
        return $Status -cmatch '^OK:LEGACY_DATABASE_CREATED_AND_VERIFIED\nDATABASE_CREATED=1\nCREATED_SHA256=[0-9a-f]{64}$'
    }
    $validUpgradeStatus = "OK:MIGRATED_AND_VERIFIED`nBACKUP_NAME=.ConfigStore.db.install-upgrade-" +
        ('a' * 32) + ".tmp.install-backup.dpapi.bak`nBACKUP_SHA256=" + ('b' * 64) + "`nFINALIZE=1"
    Assert-True (Test-InstallerStatusContract $validUpgradeStatus) 'valid staged upgrade status fixture was rejected'
    Assert-True (Test-InstallerStatusContract $validUpgradeStatus.Replace("`nFINALIZE=1", '')) 'valid scrub-complete upgrade status fixture was rejected'
    Assert-True (Test-InstallerStatusContract ("OK:LEGACY_DATABASE_CREATED_AND_VERIFIED`nDATABASE_CREATED=1`nCREATED_SHA256=" + ('d' * 64))) 'valid scrub-complete create status fixture was rejected'
    foreach ($forgedStatus in @(
        ($validUpgradeStatus + "`nUNKNOWN=1"),
        $validUpgradeStatus.Replace('FINALIZE=1', 'FINALIZE=0'),
        $validUpgradeStatus.Replace(('a' * 32), ('A' * 32)),
        $validUpgradeStatus.Replace('.tmp.install-backup.dpapi.bak', '.tmp..\evil.dpapi.bak'),
        "OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED`nDATABASE_CREATED=1`nCREATED_SHA256=" + ('c' * 64)
    )) {
        Assert-True (-not (Test-InstallerStatusContract $forgedStatus)) "forged installer status unexpectedly matched: $forgedStatus"
    }
    Assert-True ($iss.Contains('function IsCanonicalUpgradeBackupName(const Value: string): Boolean;')) 'installer lacks canonical staged backup-name validation'

    Write-Host 'PASS: installer staging-only prepare, atomic commit, forced-kill rollback, sidecar rejection, and fail-closed status handoff'
}
finally {
    Remove-Item Env:FAKE_CRASH_KIND,Env:FAKE_CRASH_SIGNAL,Env:FAKE_CRASH_RELEASE,Env:FAKE_WRITE_STAGING_SIDECAR,Env:FAKE_VERIFY_WRITES_SIDECAR,Env:FAKE_CREATE_FINAL_DURING_VERIFY,Env:FAKE_STAGING_SCRUB_COMPLETE -ErrorAction SilentlyContinue
    if (Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
}
