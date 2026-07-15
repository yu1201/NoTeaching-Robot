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

function New-TestHardLink {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Target
    )
    New-Item -ItemType HardLink -Path $Path -Target $Target -ErrorAction Stop | Out-Null
    Assert-True (Test-Path -LiteralPath $Path -PathType Leaf) 'test hardlink was not created'
}

function Get-TestReadbackPath {
    param(
        [Parameter(Mandatory = $true)][string]$Data,
        [Parameter(Mandatory = $true)][string]$BackupName
    )
    $match = [regex]::Match(
        $BackupName,
        '^\.ConfigStore\.db\.install-upgrade-([0-9a-f]{32})\.tmp\.install-backup\.dpapi\.bak$',
        [System.Text.RegularExpressions.RegexOptions]::CultureInvariant
    )
    Assert-True $match.Success 'test backup name is not transaction-bound'
    return Join-Path $Data ('.ConfigStore.db.install-readback-' + $match.Groups[1].Value + '.tmp')
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
    param([string]$Data, [string]$Status, [string[]]$Extra = @())
    $arguments = @(
        '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $helper,
        '-DataDirectory', $Data,
        '-MigrateExecutable', $fakeMigrator,
        '-StatusFile', $Status
    )
    $arguments += $Extra
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
    # The crash-window child may finish between the signal write and taskkill.
    # Treat taskkill's "process not found" result as a benign race, then rely on
    # the explicit postcondition checks below to prove both processes are gone.
    $previousErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'SilentlyContinue'
        if (-not $HelperProcess.HasExited) {
            & taskkill.exe /PID $HelperProcess.Id /T /F *> $null
        }
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }
    try { $HelperProcess.WaitForExit(10000) | Out-Null } catch {}
    $holder = Get-Process -Id $holderPid -ErrorAction SilentlyContinue
    if ($null -ne $holder) {
        try {
            $ErrorActionPreference = 'SilentlyContinue'
            & taskkill.exe /PID $holderPid /F *> $null
        }
        finally {
            $ErrorActionPreference = $previousErrorActionPreference
        }
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
                $_.Name -cmatch '^\.ConfigStore\.db\.install-readback-[0-9a-f]{32}\.tmp(?:-journal|-wal|-shm)?$' -or
                $_.Name -cmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-original\.quarantine(?:-journal|-wal|-shm)?$' -or
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
        if (Has(args, "--verify-dpapi-backup-against"))
        {
            string backup = GetValue(args, "--verify-dpapi-backup-against");
            if (!File.Exists(db) || !File.Exists(backup)) return 31;
            byte[] expected = File.ReadAllBytes(db);
            byte[] restored = ReadBackup(backup);
            return expected.SequenceEqual(restored) ? 0 : 32;
        }
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

    # Replacing the staging path after the last ordinary verification must not
    # redirect publication.  The test-only hook attempts the replacement only
    # after the helper holds the identity-bound staging handle.
    $createSwapData = Join-Path $tempRoot 'create publish identity swap Data'
    New-Item -ItemType Directory $createSwapData -Force | Out-Null
    [System.IO.File]::WriteAllText(
        (Join-Path $createSwapData 'OnlineServicesConfig.ini'),
        'AdminPassword=create-publish-swap'
    )
    $createSwapPreStatus = Join-Path $tempRoot 'create-publish-swap-pre.status'
    $code = Invoke-InstallHelper $createSwapData $createSwapPreStatus
    Assert-True ($code -eq 0) 'create publish-swap staging setup failed'
    $createSwapRecordPath = Join-Path $createSwapData $transactionName
    $createSwapRecord = Read-KeyValueFile $createSwapRecordPath
    $createSwapStaging = Join-Path $createSwapData $createSwapRecord.STAGING_NAME
    $createSwapStagingHash = Get-Sha256 $createSwapStaging
    $createSwapAttack = Join-Path $createSwapData '.ConfigStore.db.publish-swap-create.tmp'
    Write-DatabaseState $createSwapAttack 'ATTACK_CREATE_PUBLISH'
    $createSwapCommitStatus = Join-Path $tempRoot 'create-publish-swap-commit.status'
    $code = Invoke-InstallHelper $createSwapData $createSwapCommitStatus @(
        '-CommitPendingTransaction',
        '-TestOnlyStagingReplacementPath', $createSwapAttack
    )
    Assert-True ($code -ne 0) 'create commit accepted an injected staging-path replacement'
    Assert-True ((Get-Content $createSwapCommitStatus -Raw).StartsWith('ERROR:')) 'create publish-swap failure did not remain fail-closed'
    Assert-True (-not (Test-Path (Join-Path $createSwapData 'ConfigStore.db'))) 'create publish-swap attack published a final database'
    Assert-True ((Get-Sha256 $createSwapStaging) -ceq $createSwapStagingHash) 'create publish-swap changed the verified staging identity'
    Assert-True (Test-Path $createSwapRecordPath -PathType Leaf) 'create publish-swap failure deleted its transaction record'
    Assert-True (Test-Path $createSwapAttack -PathType Leaf) 'create publish-swap unexpectedly consumed the attack file'
    Remove-Item -LiteralPath $createSwapAttack -Force
    $code = Invoke-InstallHelper $createSwapData (Join-Path $tempRoot 'create-publish-swap-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'create publish-swap transaction could not be rolled back'
    Assert-NoTransactionArtifacts $createSwapData 'create publish-swap rollback'

    # Post-file commit publishes the verified create atomically; only then may scrub finalize.
    $code = Invoke-InstallHelper $legacyData $legacyStatus
    Assert-True ($code -eq 0) 'second staged create failed'
    $createCommitStatus = Join-Path $tempRoot 'create-commit.status'
    $code = Invoke-InstallHelper $legacyData $createCommitStatus @('-CommitPendingTransaction')
    $createCommitStatusText = (Get-Content $createCommitStatus -Raw -ErrorAction SilentlyContinue).Trim()
    Assert-True ($code -eq 0 -and $createCommitStatusText -ceq 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED') "staged create commit did not require finalize: exit=$code status=$createCommitStatusText"
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

    # A non-empty alias that predates the exact discard handle could point at an
    # unrelated same-content victim.  The elevated helper must not become an
    # arbitrary-file truncate oracle: reject it without changing either link or
    # clearing the transaction record.
    $preexistingAliasData = Join-Path $tempRoot 'preexisting staging alias Data'
    New-Item -ItemType Directory $preexistingAliasData -Force | Out-Null
    [System.IO.File]::WriteAllText(
        (Join-Path $preexistingAliasData 'legacy.ini'),
        'Password=preexisting-alias-secret'
    )
    $code = Invoke-InstallHelper $preexistingAliasData (Join-Path $tempRoot 'preexisting-alias-pre.status')
    Assert-True ($code -eq 0) 'preexisting alias create fixture could not be staged'
    $preexistingAliasRecordPath = Join-Path $preexistingAliasData $transactionName
    $preexistingAliasRecord = Read-KeyValueFile $preexistingAliasRecordPath
    $preexistingAliasStaging = Join-Path $preexistingAliasData $preexistingAliasRecord.STAGING_NAME
    $preexistingAliasPath = Join-Path $preexistingAliasData 'preexisting-staging-alias.bin'
    $preexistingAliasHash = Get-Sha256 $preexistingAliasStaging
    New-TestHardLink $preexistingAliasPath $preexistingAliasStaging
    $code = Invoke-InstallHelper $preexistingAliasData (Join-Path $tempRoot 'preexisting-alias-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -ne 0) 'rollback accepted a non-empty preexisting staging hardlink'
    Assert-True (
        (Get-Sha256 $preexistingAliasStaging) -ceq $preexistingAliasHash -and
        (Get-Sha256 $preexistingAliasPath) -ceq $preexistingAliasHash
    ) 'preexisting staging hardlink rejection changed the canonical or alias payload'
    Assert-True (Test-Path -LiteralPath $preexistingAliasRecordPath -PathType Leaf) 'preexisting staging hardlink rejection cleared the transaction record'
    Remove-Item -LiteralPath $preexistingAliasPath -Force
    $code = Invoke-InstallHelper $preexistingAliasData (Join-Path $tempRoot 'preexisting-alias-cleanup.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'preexisting staging hardlink transaction could not roll back after alias removal'
    Assert-NoTransactionArtifacts $preexistingAliasData 'preexisting staging alias cleanup'

    # Crash after durable staging scrub but before disposition.  The next
    # explicit rollback must recognize the exact empty canonical object,
    # re-scrub it idempotently, dispose it, and clear the CREATE_VERIFIED record.
    $stagingScrubCrashData = Join-Path $tempRoot 'staging scrub crash Data'
    New-Item -ItemType Directory $stagingScrubCrashData -Force | Out-Null
    [System.IO.File]::WriteAllText(
        (Join-Path $stagingScrubCrashData 'legacy.ini'),
        'Password=staging-scrub-crash'
    )
    $code = Invoke-InstallHelper $stagingScrubCrashData (Join-Path $tempRoot 'staging-scrub-pre.status')
    Assert-True ($code -eq 0) 'staging scrub crash fixture could not be staged'
    $stagingScrubRecordPath = Join-Path $stagingScrubCrashData $transactionName
    $stagingScrubRecord = Read-KeyValueFile $stagingScrubRecordPath
    $stagingScrubPath = Join-Path $stagingScrubCrashData $stagingScrubRecord.STAGING_NAME
    $stagingScrubProcess = Start-InstallHelper `
        $stagingScrubCrashData `
        (Join-Path $tempRoot 'staging-scrub-crash.status') `
        @('-RollbackPendingTransaction', '-TestOnlyUpgradeCrashPoint', 'after-staging-discard-scrub')
    Assert-True ($stagingScrubProcess.WaitForExit(30000)) 'staging after-scrub helper did not terminate'
    Assert-True ($stagingScrubProcess.ExitCode -ne 0) 'staging after-scrub crash hook returned success'
    $stagingScrubProcess.Dispose()
    Assert-True (
        (Test-Path -LiteralPath $stagingScrubPath -PathType Leaf) -and
        (Get-Item -LiteralPath $stagingScrubPath).Length -eq 0 -and
        (Test-Path -LiteralPath $stagingScrubRecordPath -PathType Leaf) -and
        -not (Test-Path -LiteralPath (Join-Path $stagingScrubCrashData 'ConfigStore.db'))
    ) 'staging after-scrub crash did not retain an empty canonical object plus its CREATE_VERIFIED record'
    $code = Invoke-InstallHelper $stagingScrubCrashData (Join-Path $tempRoot 'staging-scrub-restart.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'staging after-scrub crash could not be safely rolled back on restart'
    Assert-NoTransactionArtifacts $stagingScrubCrashData 'staging after-scrub restart'

    # A hardlink introduced only after the exact single-link staging bind is
    # scrubbed through the open identity.  Killing immediately after disposition
    # must leave the alias empty and the UPGRADE_VERIFIED record recoverable.
    $stagingDispositionData = Join-Path $tempRoot 'staging disposition alias crash Data'
    $stagingDispositionDb = Join-Path $stagingDispositionData 'ConfigStore.db'
    Write-DatabaseState $stagingDispositionDb 'OLD_STAGING_DISPOSITION'
    $stagingDispositionOldHash = Get-Sha256 $stagingDispositionDb
    $stagingDispositionPreStatus = Join-Path $tempRoot 'staging-disposition-pre.status'
    $code = Invoke-InstallHelper $stagingDispositionData $stagingDispositionPreStatus
    $stagingDispositionPreText = if (Test-Path -LiteralPath $stagingDispositionPreStatus -PathType Leaf) {
        (Get-Content -LiteralPath $stagingDispositionPreStatus -Raw).Trim()
    }
    else { '<missing>' }
    Assert-True ($code -eq 0) "staging disposition crash fixture could not be staged (exit=$code status=$stagingDispositionPreText)"
    $stagingDispositionRecordPath = Join-Path $stagingDispositionData $transactionName
    $stagingDispositionRecord = Read-KeyValueFile $stagingDispositionRecordPath
    $stagingDispositionPath = Join-Path $stagingDispositionData $stagingDispositionRecord.STAGING_NAME
    $stagingDispositionBackup = Join-Path $stagingDispositionData $stagingDispositionRecord.BACKUP_NAME
    $stagingDispositionAlias = Join-Path $stagingDispositionData 'staging-disposition-alias.bin'
    $stagingDispositionProcess = Start-InstallHelper `
        $stagingDispositionData `
        (Join-Path $tempRoot 'staging-disposition-crash.status') `
        @(
            '-RollbackPendingTransaction',
            '-TestOnlyUpgradeAttack', 'staging-discard-hardlink',
            '-TestOnlyUpgradeAttackPath', $stagingDispositionAlias,
            '-TestOnlyUpgradeCrashPoint', 'after-staging-discard-disposition'
        )
    Assert-True ($stagingDispositionProcess.WaitForExit(30000)) 'staging after-disposition helper did not terminate'
    Assert-True ($stagingDispositionProcess.ExitCode -ne 0) 'staging after-disposition crash hook returned success'
    $stagingDispositionProcess.Dispose()
    Assert-True (
        -not (Test-Path -LiteralPath $stagingDispositionPath) -and
        (Test-Path -LiteralPath $stagingDispositionAlias -PathType Leaf) -and
        (Get-Item -LiteralPath $stagingDispositionAlias).Length -eq 0 -and
        (Test-Path -LiteralPath $stagingDispositionBackup -PathType Leaf) -and
        (Test-Path -LiteralPath $stagingDispositionRecordPath -PathType Leaf) -and
        (Get-Sha256 $stagingDispositionDb) -ceq $stagingDispositionOldHash
    ) 'staging after-disposition crash exposed bytes or lost its recoverable transaction state'
    $code = Invoke-InstallHelper $stagingDispositionData (Join-Path $tempRoot 'staging-disposition-restart.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Item -LiteralPath $stagingDispositionAlias).Length -eq 0) 'staging after-disposition crash could not recover safely'
    Remove-Item -LiteralPath $stagingDispositionAlias -Force
    Assert-NoTransactionArtifacts $stagingDispositionData 'staging after-disposition restart'

    # The DPAPI envelope follows the same rule.  A crash after backup scrub
    # retains an empty canonical envelope, which restart must accept only as an
    # idempotent scrub residue before removing the transaction record.
    $backupScrubData = Join-Path $tempRoot 'backup scrub crash Data'
    $backupScrubDb = Join-Path $backupScrubData 'ConfigStore.db'
    Write-DatabaseState $backupScrubDb 'OLD_BACKUP_SCRUB'
    $backupScrubOldHash = Get-Sha256 $backupScrubDb
    $code = Invoke-InstallHelper $backupScrubData (Join-Path $tempRoot 'backup-scrub-pre.status')
    Assert-True ($code -eq 0) 'backup scrub crash fixture could not be staged'
    $backupScrubRecordPath = Join-Path $backupScrubData $transactionName
    $backupScrubRecord = Read-KeyValueFile $backupScrubRecordPath
    $backupScrubStaging = Join-Path $backupScrubData $backupScrubRecord.STAGING_NAME
    $backupScrubPath = Join-Path $backupScrubData $backupScrubRecord.BACKUP_NAME
    $backupScrubProcess = Start-InstallHelper `
        $backupScrubData `
        (Join-Path $tempRoot 'backup-scrub-crash.status') `
        @('-RollbackPendingTransaction', '-TestOnlyUpgradeCrashPoint', 'after-backup-discard-scrub')
    Assert-True ($backupScrubProcess.WaitForExit(30000)) 'backup after-scrub helper did not terminate'
    Assert-True ($backupScrubProcess.ExitCode -ne 0) 'backup after-scrub crash hook returned success'
    $backupScrubProcess.Dispose()
    Assert-True (
        -not (Test-Path -LiteralPath $backupScrubStaging) -and
        (Test-Path -LiteralPath $backupScrubPath -PathType Leaf) -and
        (Get-Item -LiteralPath $backupScrubPath).Length -eq 0 -and
        (Test-Path -LiteralPath $backupScrubRecordPath -PathType Leaf) -and
        (Get-Sha256 $backupScrubDb) -ceq $backupScrubOldHash
    ) 'backup after-scrub crash did not retain an empty envelope plus its record'
    $code = Invoke-InstallHelper $backupScrubData (Join-Path $tempRoot 'backup-scrub-restart.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'backup after-scrub crash could not be safely rolled back on restart'
    Assert-NoTransactionArtifacts $backupScrubData 'backup after-scrub restart'

    $backupDispositionData = Join-Path $tempRoot 'backup disposition alias crash Data'
    $backupDispositionDb = Join-Path $backupDispositionData 'ConfigStore.db'
    Write-DatabaseState $backupDispositionDb 'OLD_BACKUP_DISPOSITION'
    $backupDispositionOldHash = Get-Sha256 $backupDispositionDb
    $code = Invoke-InstallHelper $backupDispositionData (Join-Path $tempRoot 'backup-disposition-pre.status')
    Assert-True ($code -eq 0) 'backup disposition crash fixture could not be staged'
    $backupDispositionRecordPath = Join-Path $backupDispositionData $transactionName
    $backupDispositionRecord = Read-KeyValueFile $backupDispositionRecordPath
    $backupDispositionPath = Join-Path $backupDispositionData $backupDispositionRecord.BACKUP_NAME
    $backupDispositionAlias = Join-Path $backupDispositionData 'backup-disposition-alias.bin'
    $backupDispositionProcess = Start-InstallHelper `
        $backupDispositionData `
        (Join-Path $tempRoot 'backup-disposition-crash.status') `
        @(
            '-RollbackPendingTransaction',
            '-TestOnlyUpgradeAttack', 'backup-discard-hardlink',
            '-TestOnlyUpgradeAttackPath', $backupDispositionAlias,
            '-TestOnlyUpgradeCrashPoint', 'after-backup-discard-disposition'
        )
    Assert-True ($backupDispositionProcess.WaitForExit(30000)) 'backup after-disposition helper did not terminate'
    Assert-True ($backupDispositionProcess.ExitCode -ne 0) 'backup after-disposition crash hook returned success'
    $backupDispositionProcess.Dispose()
    Assert-True (
        -not (Test-Path -LiteralPath $backupDispositionPath) -and
        (Test-Path -LiteralPath $backupDispositionAlias -PathType Leaf) -and
        (Get-Item -LiteralPath $backupDispositionAlias).Length -eq 0 -and
        (Test-Path -LiteralPath $backupDispositionRecordPath -PathType Leaf) -and
        (Get-Sha256 $backupDispositionDb) -ceq $backupDispositionOldHash
    ) 'backup after-disposition crash exposed envelope bytes or lost its record'
    $code = Invoke-InstallHelper $backupDispositionData (Join-Path $tempRoot 'backup-disposition-restart.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Item -LiteralPath $backupDispositionAlias).Length -eq 0) 'backup after-disposition crash could not recover safely'
    Remove-Item -LiteralPath $backupDispositionAlias -Force
    Assert-NoTransactionArtifacts $backupDispositionData 'backup after-disposition restart'

    # Decrypted backup readback uses a deterministic transaction-derived name so
    # an empty crash residue can be found and securely disposed on restart.
    $readbackScrubData = Join-Path $tempRoot 'readback scrub crash Data'
    $readbackScrubDb = Join-Path $readbackScrubData 'ConfigStore.db'
    Write-DatabaseState $readbackScrubDb 'OLD_READBACK_SCRUB'
    $readbackScrubOldHash = Get-Sha256 $readbackScrubDb
    $readbackScrubProcess = Start-InstallHelper `
        $readbackScrubData `
        (Join-Path $tempRoot 'readback-scrub-crash.status') `
        @('-TestOnlyUpgradeCrashPoint', 'after-readback-discard-scrub')
    Assert-True ($readbackScrubProcess.WaitForExit(30000)) 'readback after-scrub helper did not terminate'
    Assert-True ($readbackScrubProcess.ExitCode -ne 0) 'readback after-scrub crash hook returned success'
    $readbackScrubProcess.Dispose()
    $readbackScrubRecordPath = Join-Path $readbackScrubData $transactionName
    $readbackScrubRecord = Read-KeyValueFile $readbackScrubRecordPath
    $readbackScrubPath = Get-TestReadbackPath $readbackScrubData $readbackScrubRecord.BACKUP_NAME
    Assert-True (
        $readbackScrubRecord.MODE -ceq 'UPGRADE_PREPARED' -and
        (Test-Path -LiteralPath $readbackScrubPath -PathType Leaf) -and
        (Get-Item -LiteralPath $readbackScrubPath).Length -eq 0 -and
        (Get-Sha256 $readbackScrubDb) -ceq $readbackScrubOldHash
    ) 'readback after-scrub crash did not retain an empty deterministic temp plus PREPARED record'
    $code = Invoke-InstallHelper $readbackScrubData (Join-Path $tempRoot 'readback-scrub-restart.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'readback after-scrub crash could not be safely rolled back on restart'
    Assert-NoTransactionArtifacts $readbackScrubData 'readback after-scrub restart'

    $readbackDispositionData = Join-Path $tempRoot 'readback disposition alias crash Data'
    $readbackDispositionDb = Join-Path $readbackDispositionData 'ConfigStore.db'
    Write-DatabaseState $readbackDispositionDb 'OLD_READBACK_DISPOSITION'
    $readbackDispositionOldHash = Get-Sha256 $readbackDispositionDb
    $readbackDispositionAlias = Join-Path $readbackDispositionData 'readback-disposition-alias.bin'
    $readbackDispositionProcess = Start-InstallHelper `
        $readbackDispositionData `
        (Join-Path $tempRoot 'readback-disposition-crash.status') `
        @(
            '-TestOnlyUpgradeAttack', 'readback-discard-hardlink',
            '-TestOnlyUpgradeAttackPath', $readbackDispositionAlias,
            '-TestOnlyUpgradeCrashPoint', 'after-readback-discard-disposition'
        )
    Assert-True ($readbackDispositionProcess.WaitForExit(30000)) 'readback after-disposition helper did not terminate'
    Assert-True ($readbackDispositionProcess.ExitCode -ne 0) 'readback after-disposition crash hook returned success'
    $readbackDispositionProcess.Dispose()
    $readbackDispositionRecordPath = Join-Path $readbackDispositionData $transactionName
    $readbackDispositionRecord = Read-KeyValueFile $readbackDispositionRecordPath
    $readbackDispositionPath = Get-TestReadbackPath $readbackDispositionData $readbackDispositionRecord.BACKUP_NAME
    Assert-True (
        $readbackDispositionRecord.MODE -ceq 'UPGRADE_PREPARED' -and
        -not (Test-Path -LiteralPath $readbackDispositionPath) -and
        (Test-Path -LiteralPath $readbackDispositionAlias -PathType Leaf) -and
        (Get-Item -LiteralPath $readbackDispositionAlias).Length -eq 0 -and
        (Get-Sha256 $readbackDispositionDb) -ceq $readbackDispositionOldHash
    ) 'readback after-disposition crash exposed plaintext or lost its PREPARED record'
    $code = Invoke-InstallHelper $readbackDispositionData (Join-Path $tempRoot 'readback-disposition-restart.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Item -LiteralPath $readbackDispositionAlias).Length -eq 0) 'readback after-disposition crash could not recover safely'
    Remove-Item -LiteralPath $readbackDispositionAlias -Force
    Assert-NoTransactionArtifacts $readbackDispositionData 'readback after-disposition restart'

    $upgradeSwapData = Join-Path $tempRoot 'upgrade publish identity swap Data'
    $upgradeSwapDb = Join-Path $upgradeSwapData 'ConfigStore.db'
    Write-DatabaseState $upgradeSwapDb 'OLD_UPGRADE_PUBLISH_SWAP'
    $upgradeSwapOriginalHash = Get-Sha256 $upgradeSwapDb
    $upgradeSwapPreStatus = Join-Path $tempRoot 'upgrade-publish-swap-pre.status'
    $code = Invoke-InstallHelper $upgradeSwapData $upgradeSwapPreStatus
    Assert-True ($code -eq 0) 'upgrade publish-swap staging setup failed'
    $upgradeSwapRecordPath = Join-Path $upgradeSwapData $transactionName
    $upgradeSwapRecord = Read-KeyValueFile $upgradeSwapRecordPath
    $upgradeSwapStaging = Join-Path $upgradeSwapData $upgradeSwapRecord.STAGING_NAME
    $upgradeSwapBackup = Join-Path $upgradeSwapData $upgradeSwapRecord.BACKUP_NAME
    $upgradeSwapStagingHash = Get-Sha256 $upgradeSwapStaging
    $upgradeSwapBackupHash = Get-Sha256 $upgradeSwapBackup
    $upgradeSwapAttack = Join-Path $upgradeSwapData '.ConfigStore.db.publish-swap-upgrade.tmp'
    Write-DatabaseState $upgradeSwapAttack 'ATTACK_UPGRADE_PUBLISH'
    $upgradeSwapCommitStatus = Join-Path $tempRoot 'upgrade-publish-swap-commit.status'
    $code = Invoke-InstallHelper $upgradeSwapData $upgradeSwapCommitStatus @(
        '-CommitPendingTransaction',
        '-TestOnlyStagingReplacementPath', $upgradeSwapAttack
    )
    Assert-True ($code -ne 0) 'upgrade commit accepted an injected staging-path replacement'
    Assert-True ((Get-Content $upgradeSwapCommitStatus -Raw).StartsWith('ERROR:')) 'upgrade publish-swap failure did not remain fail-closed'
    Assert-True ((Get-Sha256 $upgradeSwapDb) -ceq $upgradeSwapOriginalHash) 'upgrade publish-swap lost or replaced the original final database'
    Assert-True ((Get-Sha256 $upgradeSwapStaging) -ceq $upgradeSwapStagingHash) 'upgrade publish-swap changed the verified staging identity'
    Assert-True ((Get-Sha256 $upgradeSwapBackup) -ceq $upgradeSwapBackupHash) 'upgrade publish-swap changed the protected backup'
    Assert-True (Test-Path $upgradeSwapRecordPath -PathType Leaf) 'upgrade publish-swap failure deleted its transaction record'
    Assert-True (Test-Path $upgradeSwapAttack -PathType Leaf) 'upgrade publish-swap unexpectedly consumed the attack file'
    Remove-Item -LiteralPath $upgradeSwapAttack -Force
    $code = Invoke-InstallHelper $upgradeSwapData (Join-Path $tempRoot 'upgrade-publish-swap-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Sha256 $upgradeSwapDb) -ceq $upgradeSwapOriginalHash) 'upgrade publish-swap transaction could not be safely rolled back'
    Assert-NoTransactionArtifacts $upgradeSwapData 'upgrade publish-swap rollback'

    # Accept only the immediately preceding unreleased eight-line VERIFIED
    # record, deriving its quarantine name from the already-bound staging UUID.
    $legacyRecordData = Join-Path $tempRoot 'legacy verified record shape Data'
    $legacyRecordDb = Join-Path $legacyRecordData 'ConfigStore.db'
    Write-DatabaseState $legacyRecordDb 'OLD_LEGACY_VERIFIED_RECORD'
    $legacyRecordOldHash = Get-Sha256 $legacyRecordDb
    $code = Invoke-InstallHelper $legacyRecordData (Join-Path $tempRoot 'legacy-record-pre.status')
    Assert-True ($code -eq 0) 'legacy VERIFIED record compatibility fixture could not be staged'
    $legacyRecordPath = Join-Path $legacyRecordData $transactionName
    $legacyPayloadLines = @(
        Get-Content -LiteralPath $legacyRecordPath |
            Where-Object { -not $_.StartsWith('QUARANTINE_NAME=') -and -not $_.StartsWith('PAYLOAD_SHA256=') }
    )
    $legacyPayload = $legacyPayloadLines -join "`n"
    $legacyRecordText = $legacyPayload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $legacyPayload) + "`n"
    [System.IO.File]::WriteAllText($legacyRecordPath, $legacyRecordText, (New-Object System.Text.UTF8Encoding($false)))
    $code = Invoke-InstallHelper $legacyRecordData (Join-Path $tempRoot 'legacy-record-resume.status')
    Assert-True ($code -eq 0 -and (Get-Sha256 $legacyRecordDb) -ceq $legacyRecordOldHash) 'legacy eight-line VERIFIED record could not be strictly resumed'
    $code = Invoke-InstallHelper $legacyRecordData (Join-Path $tempRoot 'legacy-record-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0 -and (Get-Sha256 $legacyRecordDb) -ceq $legacyRecordOldHash) 'legacy eight-line VERIFIED record could not be rolled back'
    Assert-NoTransactionArtifacts $legacyRecordData 'legacy VERIFIED record rollback'

    # The upgrade publisher must hold both exact file identities and reserve the
    # final SQLite sidecar names for the entire two-rename atomic region.
    foreach ($upgradeAttack in @('writer', 'target-replace', 'target-hardlink', 'staging-hardlink', 'sidecar')) {
        $attackData = Join-Path $tempRoot ("upgrade exact-handle attack $upgradeAttack Data")
        $attackDb = Join-Path $attackData 'ConfigStore.db'
        Write-DatabaseState $attackDb ("OLD_EXACT_" + $upgradeAttack.ToUpperInvariant())
        $attackOriginalHash = Get-Sha256 $attackDb
        $attackStatus = Join-Path $tempRoot ("upgrade-exact-$upgradeAttack-pre.status")
        $code = Invoke-InstallHelper $attackData $attackStatus
        Assert-True ($code -eq 0) "upgrade $upgradeAttack attack fixture could not be staged"
        $attackRecordPath = Join-Path $attackData $transactionName
        $attackRecord = Read-KeyValueFile $attackRecordPath
        $attackStaging = Join-Path $attackData $attackRecord.STAGING_NAME
        $attackQuarantine = Join-Path $attackData $attackRecord.QUARANTINE_NAME
        $attackStagingHash = Get-Sha256 $attackStaging
        Assert-True ($attackRecord.QUARANTINE_NAME -ceq ($attackRecord.STAGING_NAME + '.install-original.quarantine')) 'upgrade record quarantine name is not strictly derived from staging'

        $attackAlias = Join-Path $attackData ("$upgradeAttack-injected.bin")
        $attackExtra = @('-CommitPendingTransaction', '-TestOnlyUpgradeAttack', $upgradeAttack)
        if ($upgradeAttack -in @('target-replace', 'target-hardlink', 'staging-hardlink')) {
            if ($upgradeAttack -ceq 'target-replace') {
                [System.IO.File]::WriteAllText($attackAlias, 'late-target-replacement')
            }
            $attackExtra += @('-TestOnlyUpgradeAttackPath', $attackAlias)
        }
        $code = Invoke-InstallHelper $attackData (Join-Path $tempRoot ("upgrade-exact-$upgradeAttack-commit.status")) $attackExtra
        Assert-True ($code -ne 0) "upgrade publication accepted the injected $upgradeAttack attack"
        Assert-True ((Get-Sha256 $attackDb) -ceq $attackOriginalHash) "upgrade $upgradeAttack attack changed the original canonical database"
        Assert-True (-not (Test-Path -LiteralPath $attackQuarantine)) "upgrade $upgradeAttack attack left a raw quarantine"
        Assert-True (Test-Path -LiteralPath $attackRecordPath -PathType Leaf) "upgrade $upgradeAttack attack deleted its durable transaction record"

        if ($upgradeAttack -ceq 'staging-hardlink') {
            Assert-True (-not (Test-Path -LiteralPath $attackStaging)) 'staging hardlink attack retained the compromised staging name'
            Assert-True ((Test-Path -LiteralPath $attackAlias -PathType Leaf) -and (Get-Item -LiteralPath $attackAlias).Length -eq 0) 'staging hardlink alias was not exact-handle truncated and flushed'
            $retryCode = Invoke-InstallHelper $attackData (Join-Path $tempRoot 'staging-hardlink-retry.status')
            Assert-True ($retryCode -ne 0) 'missing staging after hardlink scrub did not remain fail-closed'
            Remove-Item -LiteralPath $attackAlias -Force
            $scrubbedRollbackStatus = Join-Path $tempRoot 'staging-hardlink-rollback.status'
            $rollbackCode = Invoke-InstallHelper $attackData $scrubbedRollbackStatus @('-RollbackPendingTransaction')
            $scrubbedRollbackText = if (Test-Path -LiteralPath $scrubbedRollbackStatus -PathType Leaf) { (Get-Content $scrubbedRollbackStatus -Raw).Trim() } else { '<missing>' }
            Assert-True ($rollbackCode -eq 0 -and (Get-Sha256 $attackDb) -ceq $attackOriginalHash) "scrubbed-staging abort state could not be explicitly rolled back (exit=$rollbackCode status=$scrubbedRollbackText)"
            Assert-NoTransactionArtifacts $attackData 'scrubbed-staging explicit rollback'
            continue
        }

        Assert-True ((Test-Path -LiteralPath $attackStaging -PathType Leaf) -and (Get-Sha256 $attackStaging) -ceq $attackStagingHash) "upgrade $upgradeAttack attack changed the verified staging database"
        if ($upgradeAttack -ceq 'target-replace') {
            Assert-True ((Test-Path -LiteralPath $attackAlias -PathType Leaf) -and ([System.IO.File]::ReadAllText($attackAlias) -ceq 'late-target-replacement')) 'blocked target replacement consumed or changed the attack file'
            Remove-Item -LiteralPath $attackAlias -Force
        }
        elseif ($upgradeAttack -ceq 'target-hardlink') {
            Assert-True ((Test-Path -LiteralPath $attackAlias -PathType Leaf) -and (Get-Sha256 $attackAlias) -ceq $attackOriginalHash) 'target hardlink injection was not detected against the bound original identity'
            Remove-Item -LiteralPath $attackAlias -Force
        }
        elseif ($upgradeAttack -ceq 'sidecar') {
            Assert-True (-not (Test-Path -LiteralPath ($attackDb + '-wal'))) 'atomic sidecar reservation survived helper exit'
        }
        $code = Invoke-InstallHelper $attackData (Join-Path $tempRoot ("upgrade-exact-$upgradeAttack-rollback.status")) @('-RollbackPendingTransaction')
        Assert-True ($code -eq 0 -and (Get-Sha256 $attackDb) -ceq $attackOriginalHash) "upgrade $upgradeAttack attack transaction could not be rolled back"
        Assert-NoTransactionArtifacts $attackData "upgrade $upgradeAttack attack rollback"
    }

    # A hardlink that appears after old disposition is detected from the exact
    # old handle. Preserve NEW, scrub every alias of OLD, and retain the record
    # until the next strict published-state reconciliation.
    $lateOldData = Join-Path $tempRoot 'upgrade late old hardlink Data'
    $lateOldDb = Join-Path $lateOldData 'ConfigStore.db'
    Write-DatabaseState $lateOldDb 'OLD_LATE_DISPOSITION_LINK'
    $lateOldOriginalHash = Get-Sha256 $lateOldDb
    $code = Invoke-InstallHelper $lateOldData (Join-Path $tempRoot 'late-old-pre.status')
    Assert-True ($code -eq 0) 'late old hardlink fixture could not be staged'
    $lateOldRecord = Read-KeyValueFile (Join-Path $lateOldData $transactionName)
    $lateOldStaging = Join-Path $lateOldData $lateOldRecord.STAGING_NAME
    $lateOldQuarantine = Join-Path $lateOldData $lateOldRecord.QUARANTINE_NAME
    $lateOldAlias = Join-Path $lateOldData 'late-old-alias.bin'
    $code = Invoke-InstallHelper $lateOldData (Join-Path $tempRoot 'late-old-commit.status') @(
        '-CommitPendingTransaction',
        '-TestOnlyUpgradeAttack', 'late-old-hardlink',
        '-TestOnlyUpgradeAttackPath', $lateOldAlias
    )
    Assert-True ($code -ne 0) 'late old hardlink injection was accepted without a durable warning'
    Assert-True ((Get-Sha256 $lateOldDb) -ceq $lateOldRecord.MIGRATED_SHA256 -and -not (Test-Path -LiteralPath $lateOldStaging) -and -not (Test-Path -LiteralPath $lateOldQuarantine)) 'late old hardlink handling did not preserve one verified NEW canonical database'
    Assert-True ((Test-Path -LiteralPath $lateOldAlias -PathType Leaf) -and (Get-Item -LiteralPath $lateOldAlias).Length -eq 0) 'late old hardlink alias did not receive exact-handle scrub'
    Assert-True ($lateOldOriginalHash -cne (Get-Sha256 $lateOldDb) -and (Test-Path -LiteralPath (Join-Path $lateOldData $transactionName))) 'late old hardlink handling lost its durable published-state record'
    Remove-Item -LiteralPath $lateOldAlias -Force
    $code = Invoke-InstallHelper $lateOldData (Join-Path $tempRoot 'late-old-resume.status')
    Assert-True ($code -eq 0) 'late old hardlink published state could not be strictly resumed'
    $code = Invoke-InstallHelper $lateOldData (Join-Path $tempRoot 'late-old-recommit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'late old hardlink published state could not be recommitted'
    $code = Invoke-InstallHelper $lateOldData (Join-Path $tempRoot 'late-old-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath (Join-Path $lateOldData $transactionName))) 'late old hardlink published state could not be finalized'

    # Killing immediately after OLD disposition must be harmless even when one
    # alias appeared at the last possible pre-scrub point.  The quarantine name
    # is gone after restart, so only pre-disposition exact-handle zero+flush can
    # guarantee that the otherwise undiscoverable alias contains no raw OLD data.
    $directDispositionCrashData = Join-Path $tempRoot 'direct disposition crash with alias Data'
    $directDispositionCrashDb = Join-Path $directDispositionCrashData 'ConfigStore.db'
    Write-DatabaseState $directDispositionCrashDb 'OLD_DIRECT_DISPOSITION_CRASH'
    $code = Invoke-InstallHelper $directDispositionCrashData (Join-Path $tempRoot 'direct-disposition-pre.status')
    Assert-True ($code -eq 0) 'direct disposition crash fixture could not be staged'
    $directDispositionRecordPath = Join-Path $directDispositionCrashData $transactionName
    $directDispositionRecord = Read-KeyValueFile $directDispositionRecordPath
    $directDispositionStaging = Join-Path $directDispositionCrashData $directDispositionRecord.STAGING_NAME
    $directDispositionQuarantine = Join-Path $directDispositionCrashData $directDispositionRecord.QUARANTINE_NAME
    $directDispositionAlias = Join-Path $directDispositionCrashData 'direct-disposition-old-alias.bin'
    $directDispositionProcess = Start-InstallHelper `
        $directDispositionCrashData `
        (Join-Path $tempRoot 'direct-disposition-crash.status') `
        @(
            '-CommitPendingTransaction',
            '-TestOnlyUpgradeAttack', 'late-old-hardlink',
            '-TestOnlyUpgradeAttackPath', $directDispositionAlias,
            '-TestOnlyUpgradeCrashPoint', 'after-old-disposition'
        )
    Assert-True ($directDispositionProcess.WaitForExit(30000)) 'direct disposition crash helper did not terminate'
    Assert-True ($directDispositionProcess.ExitCode -ne 0) 'direct disposition crash hook returned success'
    $directDispositionProcess.Dispose()
    Assert-True (
        (Get-Sha256 $directDispositionCrashDb) -ceq $directDispositionRecord.MIGRATED_SHA256 -and
        -not (Test-Path -LiteralPath $directDispositionStaging) -and
        -not (Test-Path -LiteralPath $directDispositionQuarantine) -and
        (Test-Path -LiteralPath $directDispositionRecordPath -PathType Leaf)
    ) 'direct disposition crash did not preserve verified NEW plus its durable record'
    Assert-True (
        (Test-Path -LiteralPath $directDispositionAlias -PathType Leaf) -and
        (Get-Item -LiteralPath $directDispositionAlias).Length -eq 0
    ) 'direct disposition crash exposed raw OLD bytes through the surviving alias'
    $code = Invoke-InstallHelper $directDispositionCrashData (Join-Path $tempRoot 'direct-disposition-resume.status')
    Assert-True ($code -eq 0) 'direct disposition crash did not resume from verified NEW'
    Remove-Item -LiteralPath $directDispositionAlias -Force
    $code = Invoke-InstallHelper $directDispositionCrashData (Join-Path $tempRoot 'direct-disposition-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'direct disposition crash could not commit after safe resume'
    $code = Invoke-InstallHelper $directDispositionCrashData (Join-Path $tempRoot 'direct-disposition-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath $directDispositionRecordPath)) 'direct disposition crash could not finalize'

    # A kill after durable scrub but before disposition leaves NEW plus an empty
    # Q. Recovery must recognize that exact empty object, never restore it as OLD,
    # scrub/delete it again idempotently, and remain fail-closed while an alias is
    # still present.
    $emptyQuarantineData = Join-Path $tempRoot 'empty quarantine restart Data'
    $emptyQuarantineDb = Join-Path $emptyQuarantineData 'ConfigStore.db'
    Write-DatabaseState $emptyQuarantineDb 'OLD_EMPTY_Q_RESTART'
    $code = Invoke-InstallHelper $emptyQuarantineData (Join-Path $tempRoot 'empty-q-pre.status')
    Assert-True ($code -eq 0) 'empty quarantine fixture could not be staged'
    $emptyQuarantineRecordPath = Join-Path $emptyQuarantineData $transactionName
    $emptyQuarantineRecord = Read-KeyValueFile $emptyQuarantineRecordPath
    $emptyQuarantineStaging = Join-Path $emptyQuarantineData $emptyQuarantineRecord.STAGING_NAME
    $emptyQuarantinePath = Join-Path $emptyQuarantineData $emptyQuarantineRecord.QUARANTINE_NAME
    $emptyQuarantineAlias = Join-Path $emptyQuarantineData 'empty-q-old-alias.bin'
    $emptyQuarantineProcess = Start-InstallHelper `
        $emptyQuarantineData `
        (Join-Path $tempRoot 'empty-q-crash.status') `
        @(
            '-CommitPendingTransaction',
            '-TestOnlyUpgradeAttack', 'late-old-hardlink',
            '-TestOnlyUpgradeAttackPath', $emptyQuarantineAlias,
            '-TestOnlyUpgradeCrashPoint', 'after-old-scrub'
        )
    Assert-True ($emptyQuarantineProcess.WaitForExit(30000)) 'empty quarantine crash helper did not terminate'
    Assert-True ($emptyQuarantineProcess.ExitCode -ne 0) 'empty quarantine crash hook returned success'
    $emptyQuarantineProcess.Dispose()
    Assert-True (
        (Get-Sha256 $emptyQuarantineDb) -ceq $emptyQuarantineRecord.MIGRATED_SHA256 -and
        -not (Test-Path -LiteralPath $emptyQuarantineStaging) -and
        (Test-Path -LiteralPath $emptyQuarantinePath -PathType Leaf) -and
        (Get-Item -LiteralPath $emptyQuarantinePath).Length -eq 0 -and
        (Get-Item -LiteralPath $emptyQuarantineAlias).Length -eq 0
    ) 'after-old-scrub crash did not retain verified NEW plus an exact empty OLD object'
    $code = Invoke-InstallHelper $emptyQuarantineData (Join-Path $tempRoot 'empty-q-first-recovery.status')
    Assert-True ($code -ne 0) 'empty quarantine recovery accepted a still-linked OLD alias without a durable warning'
    Assert-True (
        (Get-Sha256 $emptyQuarantineDb) -ceq $emptyQuarantineRecord.MIGRATED_SHA256 -and
        -not (Test-Path -LiteralPath $emptyQuarantinePath) -and
        (Get-Item -LiteralPath $emptyQuarantineAlias).Length -eq 0 -and
        (Test-Path -LiteralPath $emptyQuarantineRecordPath -PathType Leaf)
    ) 'empty quarantine recovery did not preserve NEW and fail closed after safe alias scrub'
    Remove-Item -LiteralPath $emptyQuarantineAlias -Force
    $code = Invoke-InstallHelper $emptyQuarantineData (Join-Path $tempRoot 'empty-q-resume.status')
    Assert-True ($code -eq 0) 'empty quarantine recovery could not resume after alias removal'
    $code = Invoke-InstallHelper $emptyQuarantineData (Join-Path $tempRoot 'empty-q-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'empty quarantine recovery could not commit'
    $code = Invoke-InstallHelper $emptyQuarantineData (Join-Path $tempRoot 'empty-q-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath $emptyQuarantineRecordPath)) 'empty quarantine recovery could not finalize'

    # Reproduce the same one-alias OLD quarantine race during restart recovery,
    # using the hook only to create the link after exact-handle verification. The
    # accept/reject decision still comes solely from the real handle identity,
    # link-count, scrub, and disposition code.
    $recoveryLateOldData = Join-Path $tempRoot 'recovery late old hardlink Data'
    $recoveryLateOldDb = Join-Path $recoveryLateOldData 'ConfigStore.db'
    Write-DatabaseState $recoveryLateOldDb 'OLD_RECOVERY_LATE_LINK'
    $code = Invoke-InstallHelper $recoveryLateOldData (Join-Path $tempRoot 'recovery-late-old-pre.status')
    Assert-True ($code -eq 0) 'recovery late old hardlink fixture could not be staged'
    $recoveryLateOldRecordPath = Join-Path $recoveryLateOldData $transactionName
    $recoveryLateOldRecord = Read-KeyValueFile $recoveryLateOldRecordPath
    $recoveryLateOldStaging = Join-Path $recoveryLateOldData $recoveryLateOldRecord.STAGING_NAME
    $recoveryLateOldQuarantine = Join-Path $recoveryLateOldData $recoveryLateOldRecord.QUARANTINE_NAME
    $recoveryLateOldProcess = Start-InstallHelper `
        $recoveryLateOldData `
        (Join-Path $tempRoot 'recovery-late-old-crash.status') `
        @('-CommitPendingTransaction', '-TestOnlyUpgradeCrashPoint', 'after-new-publish')
    Assert-True ($recoveryLateOldProcess.WaitForExit(30000)) 'recovery late old helper did not terminate at after-new-publish'
    Assert-True ($recoveryLateOldProcess.ExitCode -ne 0) 'recovery late old crash hook returned success'
    $recoveryLateOldProcess.Dispose()
    Assert-True (
        (Get-Sha256 $recoveryLateOldDb) -ceq $recoveryLateOldRecord.MIGRATED_SHA256 -and
        -not (Test-Path -LiteralPath $recoveryLateOldStaging) -and
        (Get-Sha256 $recoveryLateOldQuarantine) -ceq $recoveryLateOldRecord.ORIGINAL_SHA256
    ) 'recovery late old crash did not retain NEW plus the exact OLD quarantine'
    $recoveryLateOldAlias = Join-Path $recoveryLateOldData 'recovery-late-old-alias.bin'
    $code = Invoke-InstallHelper $recoveryLateOldData (Join-Path $tempRoot 'recovery-late-old-rejected.status') @(
        '-TestOnlyUpgradeAttack', 'late-old-hardlink',
        '-TestOnlyUpgradeAttackPath', $recoveryLateOldAlias
    )
    Assert-True ($code -ne 0) 'restart recovery accepted one surviving OLD hardlink alias'
    $recoveryLateOldActualHash = if (Test-Path -LiteralPath $recoveryLateOldDb -PathType Leaf) { Get-Sha256 $recoveryLateOldDb } else { '<missing>' }
    $recoveryLateOldStagingExists = Test-Path -LiteralPath $recoveryLateOldStaging
    $recoveryLateOldQuarantineExists = Test-Path -LiteralPath $recoveryLateOldQuarantine
    $recoveryLateOldRecordExists = Test-Path -LiteralPath $recoveryLateOldRecordPath -PathType Leaf
    Assert-True (
        $recoveryLateOldActualHash -ceq $recoveryLateOldRecord.MIGRATED_SHA256 -and
        -not $recoveryLateOldStagingExists -and
        -not $recoveryLateOldQuarantineExists -and
        $recoveryLateOldRecordExists
    ) ("restart recovery did not preserve one verified NEW canonical database and durable record " +
        "(db=$recoveryLateOldActualHash expected=$($recoveryLateOldRecord.MIGRATED_SHA256) " +
        "staging=$recoveryLateOldStagingExists quarantine=$recoveryLateOldQuarantineExists " +
        "record=$recoveryLateOldRecordExists)")
    Assert-True (
        (Test-Path -LiteralPath $recoveryLateOldAlias -PathType Leaf) -and
        (Get-Item -LiteralPath $recoveryLateOldAlias).Length -eq 0
    ) 'restart recovery did not exact-handle scrub the surviving OLD alias'
    Remove-Item -LiteralPath $recoveryLateOldAlias -Force
    $code = Invoke-InstallHelper $recoveryLateOldData (Join-Path $tempRoot 'recovery-late-old-resume.status')
    Assert-True ($code -eq 0) 'restart recovery could not reconcile after the scrubbed OLD alias was removed'
    $code = Invoke-InstallHelper $recoveryLateOldData (Join-Path $tempRoot 'recovery-late-old-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'restart recovery could not recommit the verified NEW database'
    $code = Invoke-InstallHelper $recoveryLateOldData (Join-Path $tempRoot 'recovery-late-old-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath $recoveryLateOldRecordPath)) 'restart recovery could not finalize after the late OLD alias was removed'

    # Repeat the alias + post-disposition kill from the restart-reconciliation
    # branch itself. On the following process there is no Q name from which the
    # alias could be rediscovered, so its durable zero length is the safety proof.
    $recoveryDispositionCrashData = Join-Path $tempRoot 'recovery disposition crash with alias Data'
    $recoveryDispositionCrashDb = Join-Path $recoveryDispositionCrashData 'ConfigStore.db'
    Write-DatabaseState $recoveryDispositionCrashDb 'OLD_RECOVERY_DISPOSITION_CRASH'
    $code = Invoke-InstallHelper $recoveryDispositionCrashData (Join-Path $tempRoot 'recovery-disposition-pre.status')
    Assert-True ($code -eq 0) 'recovery disposition crash fixture could not be staged'
    $recoveryDispositionRecordPath = Join-Path $recoveryDispositionCrashData $transactionName
    $recoveryDispositionRecord = Read-KeyValueFile $recoveryDispositionRecordPath
    $recoveryDispositionStaging = Join-Path $recoveryDispositionCrashData $recoveryDispositionRecord.STAGING_NAME
    $recoveryDispositionQuarantine = Join-Path $recoveryDispositionCrashData $recoveryDispositionRecord.QUARANTINE_NAME
    $publishForRecoveryProcess = Start-InstallHelper `
        $recoveryDispositionCrashData `
        (Join-Path $tempRoot 'recovery-disposition-publish-crash.status') `
        @('-CommitPendingTransaction', '-TestOnlyUpgradeCrashPoint', 'after-new-publish')
    Assert-True ($publishForRecoveryProcess.WaitForExit(30000)) 'recovery disposition publish fixture did not crash'
    Assert-True ($publishForRecoveryProcess.ExitCode -ne 0) 'recovery disposition publish crash returned success'
    $publishForRecoveryProcess.Dispose()
    Assert-True (
        (Get-Sha256 $recoveryDispositionCrashDb) -ceq $recoveryDispositionRecord.MIGRATED_SHA256 -and
        -not (Test-Path -LiteralPath $recoveryDispositionStaging) -and
        (Get-Sha256 $recoveryDispositionQuarantine) -ceq $recoveryDispositionRecord.ORIGINAL_SHA256
    ) 'recovery disposition fixture did not retain NEW plus raw OLD quarantine'
    $recoveryDispositionAlias = Join-Path $recoveryDispositionCrashData 'recovery-disposition-old-alias.bin'
    $recoveryDispositionProcess = Start-InstallHelper `
        $recoveryDispositionCrashData `
        (Join-Path $tempRoot 'recovery-disposition-crash.status') `
        @(
            '-TestOnlyUpgradeAttack', 'late-old-hardlink',
            '-TestOnlyUpgradeAttackPath', $recoveryDispositionAlias,
            '-TestOnlyUpgradeCrashPoint', 'after-old-disposition'
        )
    Assert-True ($recoveryDispositionProcess.WaitForExit(30000)) 'recovery disposition helper did not terminate'
    Assert-True ($recoveryDispositionProcess.ExitCode -ne 0) 'recovery disposition crash hook returned success'
    $recoveryDispositionProcess.Dispose()
    Assert-True (
        (Get-Sha256 $recoveryDispositionCrashDb) -ceq $recoveryDispositionRecord.MIGRATED_SHA256 -and
        -not (Test-Path -LiteralPath $recoveryDispositionStaging) -and
        -not (Test-Path -LiteralPath $recoveryDispositionQuarantine) -and
        (Test-Path -LiteralPath $recoveryDispositionRecordPath -PathType Leaf)
    ) 'recovery disposition crash did not preserve verified NEW and its record'
    Assert-True (
        (Test-Path -LiteralPath $recoveryDispositionAlias -PathType Leaf) -and
        (Get-Item -LiteralPath $recoveryDispositionAlias).Length -eq 0
    ) 'recovery disposition crash exposed raw OLD bytes through the surviving alias'
    $code = Invoke-InstallHelper $recoveryDispositionCrashData (Join-Path $tempRoot 'recovery-disposition-resume.status')
    Assert-True ($code -eq 0) 'recovery disposition crash did not resume from verified NEW'
    Remove-Item -LiteralPath $recoveryDispositionAlias -Force
    $code = Invoke-InstallHelper $recoveryDispositionCrashData (Join-Path $tempRoot 'recovery-disposition-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'recovery disposition crash could not commit after resume'
    $code = Invoke-InstallHelper $recoveryDispositionCrashData (Join-Path $tempRoot 'recovery-disposition-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath $recoveryDispositionRecordPath)) 'recovery disposition crash could not finalize'

    # Kill at every durable two-rename/disposition boundary. Restart must reduce each legal
    # F/S/Q topology to one canonical database before normal commit continues.
    foreach ($publishCrashPoint in @('after-old-quarantine', 'after-new-publish', 'after-old-scrub', 'after-old-disposition')) {
        $publishCrashData = Join-Path $tempRoot ("upgrade publish crash $publishCrashPoint Data")
        $publishCrashDb = Join-Path $publishCrashData 'ConfigStore.db'
        Write-DatabaseState $publishCrashDb ("OLD_CRASH_" + $publishCrashPoint.ToUpperInvariant())
        $publishCrashOriginalHash = Get-Sha256 $publishCrashDb
        $code = Invoke-InstallHelper $publishCrashData (Join-Path $tempRoot ("upgrade-$publishCrashPoint-pre.status"))
        Assert-True ($code -eq 0) "upgrade $publishCrashPoint crash fixture could not be staged"
        $publishCrashRecord = Read-KeyValueFile (Join-Path $publishCrashData $transactionName)
        $publishCrashStaging = Join-Path $publishCrashData $publishCrashRecord.STAGING_NAME
        $publishCrashQuarantine = Join-Path $publishCrashData $publishCrashRecord.QUARANTINE_NAME
        $publishCrashMigratedHash = $publishCrashRecord.MIGRATED_SHA256
        $crashProcess = Start-InstallHelper `
            $publishCrashData `
            (Join-Path $tempRoot ("upgrade-$publishCrashPoint-crash.status")) `
            @('-CommitPendingTransaction', '-TestOnlyUpgradeCrashPoint', $publishCrashPoint)
        Assert-True ($crashProcess.WaitForExit(30000)) "upgrade $publishCrashPoint helper did not terminate at the crash hook"
        Assert-True ($crashProcess.ExitCode -ne 0) "upgrade $publishCrashPoint crash hook returned success"
        $crashProcess.Dispose()
        Assert-True (-not (Test-Path -LiteralPath ($publishCrashDb + '-wal')) -and
            -not (Test-Path -LiteralPath ($publishCrashDb + '-journal')) -and
            -not (Test-Path -LiteralPath ($publishCrashDb + '-shm'))) "upgrade $publishCrashPoint crash left reserved sidecar sentinels"
        if ($publishCrashPoint -ceq 'after-old-quarantine') {
            Assert-True (-not (Test-Path -LiteralPath $publishCrashDb)) 'old-quarantine crash unexpectedly retained a canonical database'
            Assert-True ((Get-Sha256 $publishCrashQuarantine) -ceq $publishCrashOriginalHash -and (Get-Sha256 $publishCrashStaging) -ceq $publishCrashMigratedHash) 'old-quarantine crash did not retain the exact old/staging identities'
        }
        elseif ($publishCrashPoint -ceq 'after-new-publish') {
            Assert-True ((Get-Sha256 $publishCrashDb) -ceq $publishCrashMigratedHash -and (Get-Sha256 $publishCrashQuarantine) -ceq $publishCrashOriginalHash -and -not (Test-Path -LiteralPath $publishCrashStaging)) 'new-publish crash did not retain the exact new/quarantined-old topology'
        }
        elseif ($publishCrashPoint -ceq 'after-old-scrub') {
            Assert-True ((Get-Sha256 $publishCrashDb) -ceq $publishCrashMigratedHash -and (Test-Path -LiteralPath $publishCrashQuarantine -PathType Leaf) -and (Get-Item -LiteralPath $publishCrashQuarantine).Length -eq 0 -and -not (Test-Path -LiteralPath $publishCrashStaging)) 'old-scrub crash did not retain verified NEW plus an empty quarantine'
        }
        else {
            Assert-True ((Get-Sha256 $publishCrashDb) -ceq $publishCrashMigratedHash -and -not (Test-Path -LiteralPath $publishCrashQuarantine) -and -not (Test-Path -LiteralPath $publishCrashStaging)) 'old-disposition crash did not retain the unique new canonical topology'
        }

        $resumeStatus = Join-Path $tempRoot ("upgrade-$publishCrashPoint-resume.status")
        $code = Invoke-InstallHelper $publishCrashData $resumeStatus
        Assert-True ($code -eq 0) "upgrade $publishCrashPoint crash topology could not be reconciled"
        Assert-True (-not (Test-Path -LiteralPath $publishCrashQuarantine)) "upgrade $publishCrashPoint reconciliation left raw quarantine"
        if ($publishCrashPoint -ceq 'after-old-quarantine') {
            Assert-True ((Get-Sha256 $publishCrashDb) -ceq $publishCrashOriginalHash -and (Get-Sha256 $publishCrashStaging) -ceq $publishCrashMigratedHash) 'old-quarantine reconciliation did not restore the original canonical state'
        }
        else {
            Assert-True ((Get-Sha256 $publishCrashDb) -ceq $publishCrashMigratedHash -and -not (Test-Path -LiteralPath $publishCrashStaging)) 'new-publish reconciliation did not preserve the verified new canonical state'
        }
        $commitStatus = Join-Path $tempRoot ("upgrade-$publishCrashPoint-commit.status")
        $code = Invoke-InstallHelper $publishCrashData $commitStatus @('-CommitPendingTransaction')
        Assert-True ($code -eq 0 -and (Get-Sha256 $publishCrashDb) -ceq $publishCrashMigratedHash -and -not (Test-Path -LiteralPath $publishCrashQuarantine) -and -not (Test-Path -LiteralPath $publishCrashStaging)) "upgrade $publishCrashPoint reconciled transaction could not commit uniquely"
        $code = Invoke-InstallHelper $publishCrashData (Join-Path $tempRoot ("upgrade-$publishCrashPoint-finalize.status")) @('-FinalizeDeferredScrub')
        Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath (Join-Path $publishCrashData $transactionName))) "upgrade $publishCrashPoint reconciled transaction could not finalize"
    }

    # A crash-retained OLD quarantine must never be removed before its protected
    # backup is revalidated against the durable record.
    $backupGateData = Join-Path $tempRoot 'upgrade backup-before-quarantine-delete Data'
    $backupGateDb = Join-Path $backupGateData 'ConfigStore.db'
    Write-DatabaseState $backupGateDb 'OLD_BACKUP_GATE'
    $backupGateOldHash = Get-Sha256 $backupGateDb
    $code = Invoke-InstallHelper $backupGateData (Join-Path $tempRoot 'backup-gate-pre.status')
    Assert-True ($code -eq 0) 'backup-before-quarantine-delete fixture could not be staged'
    $backupGateRecord = Read-KeyValueFile (Join-Path $backupGateData $transactionName)
    $backupGatePath = Join-Path $backupGateData $backupGateRecord.BACKUP_NAME
    $backupGateBytes = [System.IO.File]::ReadAllBytes($backupGatePath)
    $backupGateQuarantine = Join-Path $backupGateData $backupGateRecord.QUARANTINE_NAME
    $backupGateProcess = Start-InstallHelper $backupGateData (Join-Path $tempRoot 'backup-gate-crash.status') @(
        '-CommitPendingTransaction', '-TestOnlyUpgradeCrashPoint', 'after-new-publish'
    )
    Assert-True ($backupGateProcess.WaitForExit(30000)) 'backup-gate crash helper did not terminate'
    $backupGateProcess.Dispose()
    Assert-True ((Get-Sha256 $backupGateQuarantine) -ceq $backupGateOldHash) 'backup-gate crash did not retain exact OLD quarantine'
    [System.IO.File]::WriteAllText($backupGatePath, 'corrupted-protected-backup')
    $code = Invoke-InstallHelper $backupGateData (Join-Path $tempRoot 'backup-gate-rejected.status')
    Assert-True ($code -ne 0 -and (Test-Path -LiteralPath $backupGateQuarantine -PathType Leaf) -and (Get-Sha256 $backupGateQuarantine) -ceq $backupGateOldHash) 'corrupt protected backup was accepted or OLD quarantine was deleted first'
    [System.IO.File]::WriteAllBytes($backupGatePath, $backupGateBytes)
    $code = Invoke-InstallHelper $backupGateData (Join-Path $tempRoot 'backup-gate-resume.status')
    Assert-True ($code -eq 0 -and -not (Test-Path -LiteralPath $backupGateQuarantine)) 'restored protected backup could not reconcile NEW plus OLD quarantine'
    $code = Invoke-InstallHelper $backupGateData (Join-Path $tempRoot 'backup-gate-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'backup-gate reconciled transaction could not commit'
    $code = Invoke-InstallHelper $backupGateData (Join-Path $tempRoot 'backup-gate-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0) 'backup-gate reconciled transaction could not finalize'

    # Raw quarantine files are owned only by their exact UPGRADE_VERIFIED
    # record. No-record, other-mode, and wrong-UUID cases all fail closed.
    $orphanQData = Join-Path $tempRoot 'orphan quarantine no record Data'
    New-Item -ItemType Directory $orphanQData -Force | Out-Null
    $orphanQName = '.ConfigStore.db.install-upgrade-' + ('1' * 32) + '.tmp.install-original.quarantine'
    $orphanQPath = Join-Path $orphanQData $orphanQName
    Write-DatabaseState $orphanQPath 'ORPHAN_NO_RECORD'
    $orphanQHash = Get-Sha256 $orphanQPath
    $code = Invoke-InstallHelper $orphanQData (Join-Path $tempRoot 'orphan-q-no-record.status')
    Assert-True ($code -ne 0 -and (Get-Sha256 $orphanQPath) -ceq $orphanQHash) 'no-record orphan quarantine was accepted or changed'

    $wrongQData = Join-Path $tempRoot 'wrong quarantine uuid Data'
    $wrongQDb = Join-Path $wrongQData 'ConfigStore.db'
    Write-DatabaseState $wrongQDb 'OLD_WRONG_Q_UUID'
    $code = Invoke-InstallHelper $wrongQData (Join-Path $tempRoot 'wrong-q-pre.status')
    Assert-True ($code -eq 0) 'wrong-quarantine UUID fixture could not be staged'
    $wrongQPath = Join-Path $wrongQData ('.ConfigStore.db.install-upgrade-' + ('2' * 32) + '.tmp.install-original.quarantine')
    Write-DatabaseState $wrongQPath 'WRONG_Q_UUID'
    $wrongQHash = Get-Sha256 $wrongQPath
    $code = Invoke-InstallHelper $wrongQData (Join-Path $tempRoot 'wrong-q-rejected.status')
    Assert-True ($code -ne 0 -and (Get-Sha256 $wrongQPath) -ceq $wrongQHash) 'wrong-UUID quarantine was accepted or changed'
    Remove-Item -LiteralPath $wrongQPath -Force
    $code = Invoke-InstallHelper $wrongQData (Join-Path $tempRoot 'wrong-q-rollback.status') @('-RollbackPendingTransaction')
    Assert-True ($code -eq 0) 'wrong-UUID quarantine fixture could not roll back after removal'

    $publishedQData = Join-Path $tempRoot 'published mode orphan quarantine Data'
    $publishedQDb = Join-Path $publishedQData 'ConfigStore.db'
    Write-DatabaseState $publishedQDb 'OLD_PUBLISHED_Q'
    $code = Invoke-InstallHelper $publishedQData (Join-Path $tempRoot 'published-q-pre.status')
    Assert-True ($code -eq 0) 'published quarantine fixture could not be staged'
    $publishedQVerified = Read-KeyValueFile (Join-Path $publishedQData $transactionName)
    $publishedQPath = Join-Path $publishedQData $publishedQVerified.QUARANTINE_NAME
    $code = Invoke-InstallHelper $publishedQData (Join-Path $tempRoot 'published-q-commit.status') @('-CommitPendingTransaction')
    Assert-True ($code -eq 0) 'published quarantine fixture could not commit'
    Write-DatabaseState $publishedQPath 'ORPHAN_AFTER_PUBLISHED'
    $publishedQHash = Get-Sha256 $publishedQPath
    $code = Invoke-InstallHelper $publishedQData (Join-Path $tempRoot 'published-q-rejected.status')
    Assert-True ($code -ne 0 -and (Get-Sha256 $publishedQPath) -ceq $publishedQHash) 'PUBLISHED-mode orphan quarantine was accepted or changed'
    Remove-Item -LiteralPath $publishedQPath -Force
    $code = Invoke-InstallHelper $publishedQData (Join-Path $tempRoot 'published-q-finalize.status') @('-FinalizeDeferredScrub')
    Assert-True ($code -eq 0) 'published quarantine fixture could not finalize after orphan removal'

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
    $pendingNoopCommitText = if (Test-Path -LiteralPath $pendingNoopCommitStatus -PathType Leaf) {
        (Get-Content $pendingNoopCommitStatus -Raw).Trim()
    }
    else { '<missing>' }
    Assert-True ($code -eq 0 -and (Test-Path (Join-Path $pendingNoopData $transactionName) -PathType Leaf) -and $pendingNoopCommitText -ceq 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED') "pending NOOP_CURRENT commit did not require finalize (exit=$code status=$pendingNoopCommitText)"
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
        $preparedCrashRecordPath = Join-Path $crashData $transactionName
        $preparedCrashRecord = Read-KeyValueFile $preparedCrashRecordPath
        $preparedCrashStaging = Join-Path $crashData $preparedCrashRecord.STAGING_NAME
        $preparedCrashSidecar = $preparedCrashStaging + '-wal'
        Assert-True (
            $preparedCrashRecord.MODE -ceq ($crashKind.ToUpperInvariant() + '_PREPARED') -and
            (Test-Path -LiteralPath $preparedCrashSidecar -PathType Leaf)
        ) "forced-kill $crashKind did not retain its reachable PREPARED sidecar topology"
        $rollbackCode = Invoke-InstallHelper $crashData (Join-Path $tempRoot ("$crashKind-crash-rollback.status")) @('-RollbackPendingTransaction')
        Assert-True ($rollbackCode -ne 0) "forced-kill $crashKind rollback silently deleted an unbound staging sidecar"
        Assert-True (
            (Test-Path -LiteralPath $preparedCrashRecordPath -PathType Leaf) -and
            (Test-Path -LiteralPath $preparedCrashSidecar -PathType Leaf)
        ) "forced-kill $crashKind sidecar rejection did not preserve its record and evidence"
        Remove-Item -LiteralPath $preparedCrashSidecar -Force
        $rollbackCode = Invoke-InstallHelper $crashData (Join-Path $tempRoot ("$crashKind-crash-clean-rollback.status")) @('-RollbackPendingTransaction')
        Assert-True ($rollbackCode -eq 0) "forced-kill $crashKind rollback failed after explicit sidecar removal"
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
    Assert-True ($helperText.Contains("'--verify-dpapi-backup-against'") -and
        $helperText.Contains("'--db', `$script:DatabasePath")) `
        'helper does not use logical SQLite snapshot verification for DPAPI backups'
    Assert-True (-not $helperText.Contains(
        '(Get-FileSha256 $stagingPath) -cne $ExpectedDatabaseSha256')) `
        'helper still compares a restored SQLite backup to the original raw file hash'
    Assert-True (-not $helperText.Contains("'--verify-current'")) 'helper still uses database-only verification in an installer lifecycle'
    Assert-True ($helperText.Contains('SOURCE_INVENTORY_SHA256=') -and
        $helperText.Contains('Assert-NoopAbsentSourceInventory') -and
        $helperText.Contains('Assert-CreateSourceInventory')) `
        'absent/create transactions do not bind exact Data input inventory'
    Assert-True ($helperText.Contains('MODE=PUBLISHED_FINALIZE_PENDING') -and
        $helperText.Contains('Write-PublishedFinalizePending')) `
        'published pending scrub is not persisted across commit/finalize'
    Assert-True ($helperText.Contains('[System.IO.File]::Copy($script:DatabasePath, $script:UpgradeStagingPath, $false)')) 'upgrade does not copy final database to staging'
    Assert-True ($helperText.Contains('Invoke-IdentityBoundFilePublish') -and
        $helperText.Contains('SetFileInformationByHandle') -and
        $helperText.Contains('The identity-locked staging database changed after verification.')) `
        'staging publication is not bound to one revalidated open file identity'
    $identityPublishCalls = [regex]::Matches(
        $helperText,
        'Invoke-IdentityBoundFilePublish\s+\x60\s*-Source \$stagingPath'
    ).Count
    Assert-True ($identityPublishCalls -eq 1) 'create no-replace publication is not isolated from upgrade publication'
    Assert-True ($helperText.Contains('PublishUpgradeVerified') -and
        $helperText.Contains('ReconcileUpgradeVerified') -and
        $helperText.Contains('QUARANTINE_NAME=') -and
        $helperText.Contains('The exact original upgrade database')) `
        'upgrade commit does not bind old/staging exact handles to a durable quarantine state machine'
    Assert-True ($helperText.Contains('DiscardVerified') -and
        $helperText.Contains('Invoke-IdentityBoundSecureDiscard') -and
        $helperText.Contains('stream.Flush(true)') -and
        -not $helperText.Contains('Remove-Item -LiteralPath $stagingPath -Force')) `
        'sensitive staging/readback discard is not exact-handle zero+flush before disposition'
    Assert-True ($helperText.Contains('replaceExisting ? 1 : 0') -and
        $helperText.Contains('FileRenameInfo')) `
        'create commit does not use identity-bound no-replace publication'
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
