[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$DataDirectory,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$MigrateExecutable,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$StatusFile,

    [string]$ApplicationExecutable = '',

    [switch]$FinalizeDeferredScrub,

    [switch]$RollbackPendingTransaction,

    [switch]$CommitPendingTransaction,

    [switch]$InstallerHoldsInstanceLease,

    # Deterministic security-test hook.  When set, the identity-bound publish
    # primitive attempts to replace the locked staging path with this file and
    # then fails closed without publishing.  Production installers never pass it.
    [string]$TestOnlyStagingReplacementPath = '',

    # Deterministic upgrade-publication attack/crash hooks. Production
    # installers never pass these switches.
    [ValidateSet('', 'writer', 'target-replace', 'target-hardlink', 'staging-hardlink', 'late-old-hardlink', 'sidecar',
        'staging-discard-hardlink', 'backup-discard-hardlink', 'readback-discard-hardlink')]
    [string]$TestOnlyUpgradeAttack = '',

    [string]$TestOnlyUpgradeAttackPath = '',

    [ValidateSet('', 'after-old-quarantine', 'after-new-publish', 'after-old-scrub', 'after-old-disposition',
        'after-staging-discard-scrub', 'after-staging-discard-disposition',
        'after-backup-discard-scrub', 'after-backup-discard-disposition',
        'after-readback-discard-scrub', 'after-readback-discard-disposition')]
    [string]$TestOnlyUpgradeCrashPoint = ''
)

# This helper is executed by Inno Setup from its verified temporary payload before
# any new application files are installed.  It intentionally has no switch that
# can request a legacy-data overwrite migration.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:StatusFilePath = [System.IO.Path]::GetFullPath($StatusFile)
$script:StatusWritten = $false
$script:MigrationArguments = @()
$script:OperationMode = 'preinstall'
$script:CreateStagingPath = ''
$script:UpgradeStagingPath = ''
$script:ApplicationMutex = $null
$script:TransactionFileName = 'ConfigStore.db.install-transaction-v1'
$script:TransactionPath = ''
$script:TransactionFormat = 'NoTeaching-Robot-Install-Transaction-v1'

function Write-InstallerStatus {
    param([Parameter(Mandatory = $true)][string]$Value)

    $parent = Split-Path -Parent $script:StatusFilePath
    if (-not [string]::IsNullOrWhiteSpace($parent)) {
        [System.IO.Directory]::CreateDirectory($parent) | Out-Null
    }
    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($script:StatusFilePath, $Value + "`r`n", $encoding)
    $script:StatusWritten = $true
}

function Get-FileSha256 {
    param([Parameter(Mandatory = $true)][string]$Path)
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Get-TextSha256 {
    param([Parameter(Mandatory = $true)][string]$Value)

    $encoding = New-Object System.Text.UTF8Encoding($false)
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    try {
        return ([System.BitConverter]::ToString(
            $sha256.ComputeHash($encoding.GetBytes($Value))
        ).Replace('-', '').ToLowerInvariant())
    }
    finally {
        $sha256.Dispose()
    }
}

function Get-DataInputInventorySha256 {
    param([string[]]$ExcludedPaths = @())

    $excluded = [System.Collections.Generic.HashSet[string]]::new(
        [System.StringComparer]::OrdinalIgnoreCase
    )
    foreach ($path in @($ExcludedPaths)) {
        if (-not [string]::IsNullOrWhiteSpace($path)) {
            [void]$excluded.Add([System.IO.Path]::GetFullPath($path))
        }
    }

    $entries = [System.Collections.Generic.List[string]]::new()
    if (Test-Path -LiteralPath $script:DataPath) {
        if (-not (Test-Path -LiteralPath $script:DataPath -PathType Container)) {
            throw 'The Data input inventory root is not a directory.'
        }
        $rootPrefix = $script:DataPath + [System.IO.Path]::DirectorySeparatorChar
        $pending = [System.Collections.Generic.Stack[string]]::new()
        $pending.Push($script:DataPath)
        while ($pending.Count -gt 0) {
            $directory = $pending.Pop()
            foreach ($item in @(Get-ChildItem -LiteralPath $directory -Force -ErrorAction Stop)) {
                $fullPath = [System.IO.Path]::GetFullPath($item.FullName)
                if (-not $fullPath.StartsWith(
                        $rootPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
                    throw 'A Data input inventory entry escaped the fixed Data directory.'
                }
                if (($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
                    throw 'A Data input inventory entry is a reparse point.'
                }
                $relative = $fullPath.Substring($rootPrefix.Length).Replace('\', '/')
                $encodedRelative = [System.Convert]::ToBase64String(
                    [System.Text.Encoding]::UTF8.GetBytes($relative)
                )
                if ($item.PSIsContainer) {
                    if ($excluded.Contains($fullPath)) {
                        throw 'A bound installer output path unexpectedly became a directory.'
                    }
                    $entries.Add('D' + "`t" + $encodedRelative)
                    $pending.Push($fullPath)
                    continue
                }
                if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
                    throw 'A Data input inventory entry is not a regular file.'
                }
                if ($excluded.Contains($fullPath)) {
                    continue
                }

                $stream = [System.IO.File]::Open(
                    $fullPath,
                    [System.IO.FileMode]::Open,
                    [System.IO.FileAccess]::Read,
                    [System.IO.FileShare]::Read
                )
                $sha256 = [System.Security.Cryptography.SHA256]::Create()
                try {
                    $length = $stream.Length
                    $digest = [System.BitConverter]::ToString(
                        $sha256.ComputeHash($stream)
                    ).Replace('-', '').ToLowerInvariant()
                    if ($stream.Length -ne $length) {
                        throw 'A Data input changed while its inventory was being read.'
                    }
                }
                finally {
                    $sha256.Dispose()
                    $stream.Dispose()
                }
                $entries.Add(
                    'F' + "`t" + $encodedRelative + "`t" +
                    ([string]$length) + "`t" + $digest
                )
            }
        }
    }

    $ordered = $entries.ToArray()
    [System.Array]::Sort($ordered, [System.StringComparer]::Ordinal)
    $payload = "NoTeaching-Robot-Data-Input-Inventory-v1`n" +
        ([string]$ordered.Count) + "`n"
    if ($ordered.Count -gt 0) {
        $payload += ($ordered -join "`n") + "`n"
    }
    return Get-TextSha256 $payload
}

function Assert-BoundDataInputInventory {
    param(
        [Parameter(Mandatory = $true)][string]$ExpectedSha256,
        [string[]]$ExcludedPaths = @(),
        [Parameter(Mandatory = $true)][string]$Context
    )

    if ($ExpectedSha256 -cnotmatch '^[0-9a-f]{64}$' -or
        (Get-DataInputInventorySha256 -ExcludedPaths $ExcludedPaths) -cne $ExpectedSha256) {
        throw ($Context + ' changed after installer preflight; publication is refused.')
    }
}

function Assert-NoopAbsentSourceInventory {
    param(
        [Parameter(Mandatory = $true)]$Record,
        [Parameter(Mandatory = $true)][string]$Context
    )

    if ($Record.Mode -cne 'NOOP_ABSENT') {
        throw 'An absent source inventory check received the wrong transaction mode.'
    }
    $emptyInventorySha256 = Get-TextSha256 "NoTeaching-Robot-Data-Input-Inventory-v1`n0`n"
    if ($Record.SourceInventorySha256 -cne $emptyInventorySha256) {
        throw 'An absent no-op transaction is not bound to the canonical empty Data inventory.'
    }
    Assert-BoundDataInputInventory `
        -ExpectedSha256 $Record.SourceInventorySha256 `
        -ExcludedPaths @($script:TransactionPath, $script:DatabasePath) `
        -Context $Context
}

function Assert-CreateSourceInventory {
    param(
        [Parameter(Mandatory = $true)]$Record,
        [Parameter(Mandatory = $true)][string]$Context
    )

    $isPublishedCreate = $Record.Mode -ceq 'PUBLISHED_FINALIZE_PENDING' -and
        $Record.PublishedKind -ceq 'CREATE'
    if ($Record.Mode -notin @('CREATE_PREPARED', 'CREATE_READY', 'CREATE_VERIFIED') -and
        -not $isPublishedCreate) {
        throw 'A create source inventory check received the wrong transaction mode.'
    }
    $stagingPath = Join-Path $script:DataPath $Record.StagingName
    Assert-BoundDataInputInventory `
        -ExpectedSha256 $Record.SourceInventorySha256 `
        -ExcludedPaths @($script:TransactionPath, $script:DatabasePath, $stagingPath) `
        -Context $Context
}

function Initialize-AtomicFileType {
    if (-not ('NoTeachingRobot.InstallerAtomicFile' -as [type])) {
        Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.IO;
using System.Security.Cryptography;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace NoTeachingRobot
{
    public static class InstallerAtomicFile
    {
        private const int MoveFileReplaceExisting = 0x1;
        private const int MoveFileWriteThrough = 0x8;
        private const uint GenericRead = 0x80000000;
        private const uint GenericWrite = 0x40000000;
        private const uint Delete = 0x00010000;
        private const uint FileShareRead = 0x00000001;
        private const uint FileShareWrite = 0x00000002;
        private const uint FileShareDelete = 0x00000004;
        private const uint CreateNew = 1;
        private const uint OpenExisting = 3;
        private const uint FileFlagOpenReparsePoint = 0x00200000;
        private const uint FileFlagDeleteOnClose = 0x04000000;
        private const uint FileFlagBackupSemantics = 0x02000000;
        private const int FileRenameInfo = 3;
        private const int FileDispositionInfo = 4;

        [StructLayout(LayoutKind.Sequential)]
        private struct ByHandleFileInformation
        {
            public uint FileAttributes;
            public System.Runtime.InteropServices.ComTypes.FILETIME CreationTime;
            public System.Runtime.InteropServices.ComTypes.FILETIME LastAccessTime;
            public System.Runtime.InteropServices.ComTypes.FILETIME LastWriteTime;
            public uint VolumeSerialNumber;
            public uint FileSizeHigh;
            public uint FileSizeLow;
            public uint NumberOfLinks;
            public uint FileIndexHigh;
            public uint FileIndexLow;
        }

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern bool MoveFileEx(string existingName, string newName, int flags);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern SafeFileHandle CreateFile(
            string fileName,
            uint desiredAccess,
            uint shareMode,
            IntPtr securityAttributes,
            uint creationDisposition,
            uint flagsAndAttributes,
            IntPtr templateFile);

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool SetFileInformationByHandle(
            SafeFileHandle file,
            int fileInformationClass,
            IntPtr fileInformation,
            uint bufferSize);

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool GetFileInformationByHandle(
            SafeFileHandle file,
            out ByHandleFileInformation fileInformation);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern uint GetFinalPathNameByHandle(
            SafeFileHandle file,
            System.Text.StringBuilder path,
            uint pathLength,
            uint flags);

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool FlushFileBuffers(SafeFileHandle file);

        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern bool CreateHardLink(
            string newFileName,
            string existingFileName,
            IntPtr securityAttributes);

        public static void Replace(string source, string destination)
        {
            if (!MoveFileEx(source, destination, MoveFileReplaceExisting | MoveFileWriteThrough))
                throw new Win32Exception(Marshal.GetLastWin32Error(), "Atomic file replace failed");
        }

        public static void MoveNoReplace(string source, string destination)
        {
            if (!MoveFileEx(source, destination, MoveFileWriteThrough))
                throw new Win32Exception(Marshal.GetLastWin32Error(), "Atomic no-replace file move failed");
        }

        private static void RenameOpenHandle(
            SafeFileHandle sourceHandle,
            string destination,
            bool replaceExisting)
        {
            byte[] destinationBytes = System.Text.Encoding.Unicode.GetBytes(
                Path.GetFullPath(destination));
            int rootDirectoryOffset = IntPtr.Size == 8 ? 8 : 4;
            int nameLengthOffset = rootDirectoryOffset + IntPtr.Size;
            int fileNameOffset = nameLengthOffset + 4;
            // FILE_RENAME_INFO ends in WCHAR FileName[1].  Reserve that complete
            // trailing element (including its NUL) even though FileNameLength
            // itself deliberately excludes the terminator.
            int bufferSize = checked(fileNameOffset + destinationBytes.Length + 2);
            IntPtr buffer = Marshal.AllocHGlobal(bufferSize);
            try
            {
                Marshal.Copy(new byte[bufferSize], 0, buffer, bufferSize);
                Marshal.WriteInt32(buffer, 0, replaceExisting ? 1 : 0);
                Marshal.WriteIntPtr(buffer, rootDirectoryOffset, IntPtr.Zero);
                Marshal.WriteInt32(buffer, nameLengthOffset, destinationBytes.Length);
                Marshal.Copy(destinationBytes, 0, IntPtr.Add(buffer, fileNameOffset), destinationBytes.Length);
                if (!SetFileInformationByHandle(
                        sourceHandle,
                        FileRenameInfo,
                        buffer,
                        checked((uint)bufferSize)))
                {
                    throw new Win32Exception(
                        Marshal.GetLastWin32Error(),
                        "Identity-bound atomic file publish failed");
                }
            }
            finally
            {
                Marshal.FreeHGlobal(buffer);
            }
        }

        private static SafeFileHandle OpenExact(
            string path,
            uint desiredAccess,
            uint shareMode,
            uint extraFlags)
        {
            SafeFileHandle handle = CreateFile(
                Path.GetFullPath(path),
                desiredAccess,
                shareMode,
                IntPtr.Zero,
                OpenExisting,
                FileFlagOpenReparsePoint | extraFlags,
                IntPtr.Zero);
            if (handle.IsInvalid)
            {
                int error = Marshal.GetLastWin32Error();
                handle.Dispose();
                throw new Win32Exception(error, "Exact file identity could not be opened: " + path);
            }
            return handle;
        }

        private static string NormalizeHandlePath(string value)
        {
            if (value.StartsWith(@"\\?\UNC\", StringComparison.OrdinalIgnoreCase))
                value = @"\\" + value.Substring(8);
            else if (value.StartsWith(@"\\?\", StringComparison.OrdinalIgnoreCase))
                value = value.Substring(4);
            return Path.GetFullPath(value).TrimEnd(Path.DirectorySeparatorChar);
        }

        private static string GetHandlePath(SafeFileHandle handle)
        {
            System.Text.StringBuilder path = new System.Text.StringBuilder(32768);
            uint length = GetFinalPathNameByHandle(handle, path, (uint)path.Capacity, 0);
            if (length == 0 || length >= path.Capacity)
                throw new Win32Exception(
                    Marshal.GetLastWin32Error(),
                    "The exact file path could not be read from its handle");
            return NormalizeHandlePath(path.ToString());
        }

        private static ByHandleFileInformation GetIdentity(SafeFileHandle handle)
        {
            ByHandleFileInformation information;
            if (!GetFileInformationByHandle(handle, out information))
                throw new Win32Exception(
                    Marshal.GetLastWin32Error(),
                    "The exact file identity could not be queried");
            return information;
        }

        private static string HashExactHandle(SafeFileHandle handle)
        {
            using (SafeFileHandle borrowed = new SafeFileHandle(handle.DangerousGetHandle(), false))
            using (FileStream stream = new FileStream(borrowed, FileAccess.Read, 4096, false))
            using (SHA256 sha256 = SHA256.Create())
            {
                stream.Position = 0;
                return BitConverter.ToString(sha256.ComputeHash(stream))
                    .Replace("-", "")
                    .ToLowerInvariant();
            }
        }

        private static ByHandleFileInformation VerifyExactRegularFile(
            SafeFileHandle handle,
            string expectedPath,
            string description,
            bool requireSingleLink)
        {
            ByHandleFileInformation information = GetIdentity(handle);
            if ((information.FileAttributes & 0x10) != 0 ||
                (information.FileAttributes & 0x400) != 0 ||
                information.NumberOfLinks == 0 ||
                (requireSingleLink && information.NumberOfLinks != 1))
            {
                throw new InvalidOperationException(
                    description + (requireSingleLink
                        ? " is not a single-link regular non-reparse file."
                        : " is not a linked regular non-reparse file."));
            }
            if (!String.Equals(
                    GetHandlePath(handle),
                    NormalizeHandlePath(expectedPath),
                    StringComparison.OrdinalIgnoreCase))
            {
                throw new InvalidOperationException(
                    description + " is no longer at its exact bound path.");
            }
            return information;
        }

        private static void VerifyExactDatabasePayload(
            SafeFileHandle handle,
            string expectedSha256,
            string description)
        {
            if (!String.Equals(
                    HashExactHandle(handle),
                    expectedSha256,
                    StringComparison.Ordinal))
            {
                throw new InvalidOperationException(
                    description + " changed after transaction verification.");
            }
            using (SafeFileHandle borrowed = new SafeFileHandle(handle.DangerousGetHandle(), false))
            using (FileStream stream = new FileStream(borrowed, FileAccess.Read, 4096, false))
            {
                byte[] header = new byte[16];
                stream.Position = 0;
                if (stream.Read(header, 0, header.Length) != header.Length ||
                    !String.Equals(
                        System.Text.Encoding.ASCII.GetString(header),
                        "SQLite format 3\0",
                        StringComparison.Ordinal))
                {
                    throw new InvalidOperationException(
                    description + " has an invalid SQLite header.");
                }
            }
        }

        private static void VerifyExactDatabase(
            SafeFileHandle handle,
            string expectedPath,
            string expectedSha256,
            string description)
        {
            VerifyExactRegularFile(handle, expectedPath, description, true);
            VerifyExactDatabasePayload(handle, expectedSha256, description);
        }

        private static void VerifyExactDatabaseOrEmptyForScrub(
            SafeFileHandle handle,
            string expectedPath,
            string expectedSha256,
            string description)
        {
            ByHandleFileInformation information = VerifyExactRegularFile(
                handle, expectedPath, description, false);
            if (information.FileSizeHigh == 0 && information.FileSizeLow == 0)
                return;
            VerifyExactDatabasePayload(handle, expectedSha256, description);
        }

        private static void VerifyExactEmptyAfterScrub(
            SafeFileHandle handle,
            string description)
        {
            ByHandleFileInformation information = GetIdentity(handle);
            if ((information.FileAttributes & 0x10) != 0 ||
                (information.FileAttributes & 0x400) != 0 ||
                information.FileSizeHigh != 0 ||
                information.FileSizeLow != 0)
            {
                throw new InvalidOperationException(
                    description + " was not durably reduced to an empty regular file.");
            }
        }

        private static SafeFileHandle OpenAndVerifyParent(string directory)
        {
            SafeFileHandle handle = OpenExact(
                directory,
                GenericRead,
                FileShareRead | FileShareWrite,
                FileFlagBackupSemantics);
            ByHandleFileInformation information = GetIdentity(handle);
            if ((information.FileAttributes & 0x10) == 0 ||
                (information.FileAttributes & 0x400) != 0 ||
                !String.Equals(
                    GetHandlePath(handle),
                    NormalizeHandlePath(directory),
                    StringComparison.OrdinalIgnoreCase))
            {
                handle.Dispose();
                throw new InvalidOperationException(
                    "The upgrade publication parent is not the exact regular directory.");
            }
            return handle;
        }

        private static bool PathEntryExists(string path)
        {
            return File.Exists(path) || Directory.Exists(path);
        }

        private static void DeleteOpenHandle(SafeFileHandle handle)
        {
            IntPtr information = Marshal.AllocHGlobal(1);
            try
            {
                Marshal.WriteByte(information, 1);
                if (!SetFileInformationByHandle(
                        handle,
                        FileDispositionInfo,
                        information,
                        1))
                {
                    throw new Win32Exception(
                        Marshal.GetLastWin32Error(),
                        "Exact-handle deletion failed");
                }
            }
            finally
            {
                Marshal.FreeHGlobal(information);
            }
        }

        private static void ZeroAndDeleteOpenHandle(SafeFileHandle handle)
        {
            using (SafeFileHandle borrowed = new SafeFileHandle(handle.DangerousGetHandle(), false))
            using (FileStream stream = new FileStream(borrowed, FileAccess.ReadWrite, 4096, false))
            {
                stream.Position = 0;
                stream.SetLength(0);
                stream.Flush(true);
            }
            DeleteOpenHandle(handle);
        }

        private static void ZeroOpenHandle(SafeFileHandle handle)
        {
            using (SafeFileHandle borrowed = new SafeFileHandle(handle.DangerousGetHandle(), false))
            using (FileStream stream = new FileStream(borrowed, FileAccess.ReadWrite, 4096, false))
            {
                stream.Position = 0;
                stream.SetLength(0);
                stream.Flush(true);
            }
        }

        private static SafeFileHandle[] ReserveFinalSidecars(string destination)
        {
            string[] suffixes = new string[] { "-journal", "-wal", "-shm" };
            SafeFileHandle[] handles = new SafeFileHandle[suffixes.Length];
            try
            {
                for (int index = 0; index < suffixes.Length; ++index)
                {
                    string path = Path.GetFullPath(destination) + suffixes[index];
                    handles[index] = CreateFile(
                        path,
                        GenericRead | GenericWrite | Delete,
                        0,
                        IntPtr.Zero,
                        CreateNew,
                        FileFlagOpenReparsePoint | FileFlagDeleteOnClose,
                        IntPtr.Zero);
                    if (handles[index].IsInvalid)
                        throw new Win32Exception(
                            Marshal.GetLastWin32Error(),
                            "The final SQLite sidecar name could not be exclusively reserved: " + path);
                }
                return handles;
            }
            catch
            {
                DisposeHandles(handles);
                throw;
            }
        }

        private static void DisposeHandles(SafeFileHandle[] handles)
        {
            if (handles == null)
                return;
            foreach (SafeFileHandle handle in handles)
            {
                if (handle != null)
                    handle.Dispose();
            }
        }

        private static void RequireSameParent(
            string source,
            string destination,
            string quarantine)
        {
            string sourceParent = Path.GetDirectoryName(Path.GetFullPath(source));
            string destinationParent = Path.GetDirectoryName(Path.GetFullPath(destination));
            string quarantineParent = Path.GetDirectoryName(Path.GetFullPath(quarantine));
            if (!String.Equals(sourceParent, destinationParent, StringComparison.OrdinalIgnoreCase) ||
                !String.Equals(sourceParent, quarantineParent, StringComparison.OrdinalIgnoreCase) ||
                !String.Equals(
                    Path.GetFullPath(quarantine),
                    Path.GetFullPath(source) + ".install-original.quarantine",
                    StringComparison.OrdinalIgnoreCase))
            {
                throw new InvalidOperationException(
                    "Upgrade publication paths are not one strictly derived transaction family.");
            }
        }

        public static void DiscardVerified(
            string path,
            string expectedSha256,
            string testOnlyHardlinkPath,
            string testOnlyCrashPoint,
            string crashScope)
        {
            string fullPath = Path.GetFullPath(path);
            if (String.IsNullOrEmpty(expectedSha256) || expectedSha256.Length != 64)
                throw new InvalidOperationException(
                    "Secure discard requires one exact lowercase SHA-256 binding.");
            for (int index = 0; index < expectedSha256.Length; ++index)
            {
                char value = expectedSha256[index];
                if (!((value >= '0' && value <= '9') || (value >= 'a' && value <= 'f')))
                    throw new InvalidOperationException(
                        "Secure discard requires one exact lowercase SHA-256 binding.");
            }
            if (!String.Equals(crashScope, "staging-discard", StringComparison.Ordinal) &&
                !String.Equals(crashScope, "backup-discard", StringComparison.Ordinal) &&
                !String.Equals(crashScope, "readback-discard", StringComparison.Ordinal))
            {
                throw new InvalidOperationException("Secure discard received an invalid test scope.");
            }

            using (SafeFileHandle parent = OpenAndVerifyParent(Path.GetDirectoryName(fullPath)))
            using (SafeFileHandle exact = OpenExact(
                fullPath,
                GenericRead | GenericWrite | Delete,
                FileShareRead,
                0))
            {
                ByHandleFileInformation initial = VerifyExactRegularFile(
                    exact, fullPath, "The securely discarded exact object", false);
                bool alreadyEmpty = initial.FileSizeHigh == 0 && initial.FileSizeLow == 0;
                // A non-empty object with a pre-existing alias is not safe to
                // truncate from an elevated installer: the canonical pathname
                // could have been replaced with a same-content victim hardlink.
                // Only aliases created after this single-link exact binding are
                // eligible for scrub.  Empty crash-retained objects are harmless
                // to re-scrub even when their already-empty aliases survive.
                if (!alreadyEmpty && initial.NumberOfLinks != 1)
                    throw new InvalidOperationException(
                        "The securely discarded non-empty object already has a hardlink alias.");
                if (!alreadyEmpty && !String.Equals(
                        HashExactHandle(exact), expectedSha256, StringComparison.Ordinal))
                {
                    throw new InvalidOperationException(
                        "The securely discarded exact object changed after its hash binding.");
                }
                if (!String.IsNullOrEmpty(testOnlyHardlinkPath))
                {
                    if (!CreateHardLink(
                            Path.GetFullPath(testOnlyHardlinkPath),
                            fullPath,
                            IntPtr.Zero))
                    {
                        throw new Win32Exception(
                            Marshal.GetLastWin32Error(),
                            "The secure-discard hardlink injection could not be created");
                    }
                }

                // Recheck the exact handle after the deterministic race hook.
                // Then make every current/future name for this file identity
                // durably empty before the canonical name can disappear.
                ByHandleFileInformation rechecked = VerifyExactRegularFile(
                    exact, fullPath, "The rechecked securely discarded object", false);
                bool recheckedEmpty = rechecked.FileSizeHigh == 0 && rechecked.FileSizeLow == 0;
                if (!recheckedEmpty && !String.Equals(
                        HashExactHandle(exact), expectedSha256, StringComparison.Ordinal))
                {
                    throw new InvalidOperationException(
                        "The securely discarded exact object changed immediately before scrub.");
                }
                ZeroOpenHandle(exact);
                VerifyExactEmptyAfterScrub(exact, "The securely discarded exact object");
                CrashAt(testOnlyCrashPoint, "after-" + crashScope + "-scrub");
                DeleteOpenHandle(exact);
                CrashAt(testOnlyCrashPoint, "after-" + crashScope + "-disposition");
                if (GetIdentity(exact).NumberOfLinks != 0)
                {
                    VerifyExactEmptyAfterScrub(
                        exact, "A surviving securely discarded hardlink");
                    throw new InvalidOperationException(
                        "A secure-discard hardlink survived disposition; all aliases were scrubbed.");
                }
            }

            if (PathEntryExists(fullPath))
                throw new InvalidOperationException(
                    "The securely discarded canonical path survived disposition.");
        }

        private static void CrashAt(string requestedPoint, string actualPoint)
        {
            if (!String.Equals(requestedPoint, actualPoint, StringComparison.Ordinal))
                return;
            System.Diagnostics.Process.GetCurrentProcess().Kill();
            System.Threading.Thread.Sleep(System.Threading.Timeout.Infinite);
        }

        public static void PublishUpgradeVerified(
            string source,
            string destination,
            string quarantine,
            string expectedSourceSha256,
            string expectedDestinationSha256,
            string testOnlyStagingReplacement,
            string testOnlyAttack,
            string testOnlyAttackPath,
            string testOnlyCrashPoint)
        {
            string sourceFullPath = Path.GetFullPath(source);
            string destinationFullPath = Path.GetFullPath(destination);
            string quarantineFullPath = Path.GetFullPath(quarantine);
            RequireSameParent(sourceFullPath, destinationFullPath, quarantineFullPath);
            string parentPath = Path.GetDirectoryName(destinationFullPath);
            bool oldQuarantined = false;
            bool newPublished = false;
            bool stagingAliasInjected = false;
            bool oldScrubStarted = false;
            SafeFileHandle[] sidecarReservations = ReserveFinalSidecars(destinationFullPath);
            try
            {
                using (SafeFileHandle parent = OpenAndVerifyParent(parentPath))
                using (SafeFileHandle staging = OpenExact(
                    sourceFullPath,
                    GenericRead | GenericWrite | Delete,
                    FileShareRead,
                    0))
                using (SafeFileHandle original = OpenExact(
                    destinationFullPath,
                    GenericRead | GenericWrite | Delete,
                    FileShareRead,
                    0))
                {
                  try
                  {
                    VerifyExactDatabase(
                        staging, sourceFullPath, expectedSourceSha256,
                        "The exact upgrade staging database");
                    VerifyExactDatabase(
                        original, destinationFullPath, expectedDestinationSha256,
                        "The exact original upgrade database");
                    if (PathEntryExists(quarantineFullPath))
                        throw new InvalidOperationException(
                            "The derived upgrade quarantine path is already occupied.");
                    if (!FlushFileBuffers(staging))
                        throw new Win32Exception(
                            Marshal.GetLastWin32Error(),
                            "The exact staging database could not be flushed");

                    if (!String.IsNullOrWhiteSpace(testOnlyStagingReplacement))
                    {
                        bool replaced = MoveFileEx(
                            Path.GetFullPath(testOnlyStagingReplacement),
                            sourceFullPath,
                            MoveFileReplaceExisting | MoveFileWriteThrough);
                        int replacementError = Marshal.GetLastWin32Error();
                        if (replaced)
                            throw new InvalidOperationException(
                                "Injected staging replacement bypassed the upgrade identity lock.");
                        throw new Win32Exception(
                            replacementError,
                            "Injected staging replacement was blocked before upgrade publication");
                    }

                    if (String.Equals(testOnlyAttack, "writer", StringComparison.Ordinal))
                    {
                        using (SafeFileHandle writer = CreateFile(
                            destinationFullPath,
                            GenericWrite,
                            FileShareRead | FileShareWrite | FileShareDelete,
                            IntPtr.Zero,
                            OpenExisting,
                            FileFlagOpenReparsePoint,
                            IntPtr.Zero))
                        {
                            if (!writer.IsInvalid)
                                throw new InvalidOperationException(
                                    "Injected late writer bypassed the locked original target.");
                            throw new Win32Exception(
                                Marshal.GetLastWin32Error(),
                                "Injected late writer was blocked by the original target lock");
                        }
                    }
                    if (String.Equals(testOnlyAttack, "target-replace", StringComparison.Ordinal))
                    {
                        bool replaced = MoveFileEx(
                            Path.GetFullPath(testOnlyAttackPath),
                            destinationFullPath,
                            MoveFileReplaceExisting | MoveFileWriteThrough);
                        int replacementError = Marshal.GetLastWin32Error();
                        if (replaced)
                            throw new InvalidOperationException(
                                "Injected late target replacement bypassed the original lock.");
                        throw new Win32Exception(
                            replacementError,
                            "Injected late target replacement was blocked by the original lock");
                    }
                    if (String.Equals(testOnlyAttack, "target-hardlink", StringComparison.Ordinal))
                    {
                        if (!CreateHardLink(
                                Path.GetFullPath(testOnlyAttackPath),
                                destinationFullPath,
                                IntPtr.Zero))
                            throw new Win32Exception(
                                Marshal.GetLastWin32Error(),
                                "The target hardlink injection could not be created");
                    }
                    if (String.Equals(testOnlyAttack, "staging-hardlink", StringComparison.Ordinal))
                    {
                        if (!CreateHardLink(
                                Path.GetFullPath(testOnlyAttackPath),
                                sourceFullPath,
                                IntPtr.Zero))
                            throw new Win32Exception(
                                Marshal.GetLastWin32Error(),
                                "The staging hardlink injection could not be created");
                        stagingAliasInjected = true;
                    }
                    if (String.Equals(testOnlyAttack, "sidecar", StringComparison.Ordinal))
                    {
                        using (SafeFileHandle sidecar = CreateFile(
                            destinationFullPath + "-wal",
                            GenericRead | GenericWrite | Delete,
                            0,
                            IntPtr.Zero,
                            CreateNew,
                            FileFlagOpenReparsePoint,
                            IntPtr.Zero))
                        {
                            if (!sidecar.IsInvalid)
                                throw new InvalidOperationException(
                                    "Injected SQLite sidecar bypassed the reserved final name.");
                            throw new Win32Exception(
                                Marshal.GetLastWin32Error(),
                                "Injected SQLite sidecar was blocked by the atomic reservation");
                        }
                    }

                    // The attack hooks run after the initial handle binding.  These
                    // rechecks are the last gate before the old exact object moves.
                    VerifyExactDatabase(
                        original, destinationFullPath, expectedDestinationSha256,
                        "The rechecked original upgrade database");
                    VerifyExactDatabase(
                        staging, sourceFullPath, expectedSourceSha256,
                        "The rechecked upgrade staging database");

                    RenameOpenHandle(original, quarantineFullPath, false);
                    oldQuarantined = true;
                    VerifyExactDatabase(
                        original, quarantineFullPath, expectedDestinationSha256,
                        "The quarantined original upgrade database");
                    VerifyExactDatabase(
                        staging, sourceFullPath, expectedSourceSha256,
                        "The staged upgrade database after original quarantine");
                    CrashAt(testOnlyCrashPoint, "after-old-quarantine");

                    RenameOpenHandle(staging, destinationFullPath, false);
                    newPublished = true;
                    VerifyExactDatabase(
                        staging, destinationFullPath, expectedSourceSha256,
                        "The published exact upgrade database");
                    VerifyExactDatabase(
                        original, quarantineFullPath, expectedDestinationSha256,
                        "The retained exact original upgrade database");
                    CrashAt(testOnlyCrashPoint, "after-new-publish");

                    VerifyExactDatabase(
                        original, quarantineFullPath, expectedDestinationSha256,
                        "The final exact original upgrade database");
                    if (String.Equals(testOnlyAttack, "late-old-hardlink", StringComparison.Ordinal))
                    {
                        if (!CreateHardLink(
                                Path.GetFullPath(testOnlyAttackPath),
                                quarantineFullPath,
                                IntPtr.Zero))
                            throw new Win32Exception(
                                Marshal.GetLastWin32Error(),
                                "The late old-quarantine hardlink injection could not be created");
                    }
                    // NEW is already exact-handle published and the PowerShell
                    // caller has revalidated the protected backup.  From this
                    // point onward OLD must never be restored: first make every
                    // hardlink to the exact object durably empty, then discard
                    // the quarantine name.
                    oldScrubStarted = true;
                    ZeroOpenHandle(original);
                    VerifyExactEmptyAfterScrub(
                        original, "The scrubbed original upgrade database");
                    CrashAt(testOnlyCrashPoint, "after-old-scrub");
                    DeleteOpenHandle(original);
                    oldQuarantined = false;
                    CrashAt(testOnlyCrashPoint, "after-old-disposition");
                    uint remainingOldLinks = GetIdentity(original).NumberOfLinks;
                    if (remainingOldLinks != 0)
                    {
                        VerifyExactEmptyAfterScrub(
                            original, "A surviving original-database hardlink");
                        throw new InvalidOperationException(
                            "A late original-database hardlink survived disposition; all old aliases were scrubbed.");
                    }
                  }
                  catch (Exception publishError)
                  {
                    // Once exact-handle OLD scrubbing starts, rollback would
                    // reintroduce a truncated database or overwrite verified NEW.
                    // Preserve NEW plus the durable VERIFIED record for strict
                    // next-run reconciliation, whether Q is raw, empty, or gone.
                    if (oldScrubStarted)
                        throw;
                    Exception rollbackError = null;
                    try
                    {
                        if (newPublished)
                        {
                            RenameOpenHandle(staging, sourceFullPath, false);
                            newPublished = false;
                        }
                        if (stagingAliasInjected || GetIdentity(staging).NumberOfLinks != 1)
                            ZeroAndDeleteOpenHandle(staging);
                        if (oldQuarantined)
                        {
                            RenameOpenHandle(original, destinationFullPath, false);
                            oldQuarantined = false;
                        }
                    }
                    catch (Exception failure)
                    {
                        rollbackError = failure;
                    }
                    if (rollbackError != null)
                        throw new AggregateException(
                            "Upgrade publication and exact rollback both failed.",
                            publishError,
                            rollbackError);
                    throw;
                  }
                }
            }
            finally
            {
                DisposeHandles(sidecarReservations);
            }

            if (!File.Exists(destinationFullPath) ||
                File.Exists(sourceFullPath) ||
                PathEntryExists(quarantineFullPath))
            {
                throw new InvalidOperationException(
                    "The exact upgrade publication did not reach one canonical file.");
            }
        }

        public static string ReconcileUpgradeVerified(
            string source,
            string destination,
            string quarantine,
            string expectedSourceSha256,
            string expectedDestinationSha256,
            string testOnlyAttack,
            string testOnlyAttackPath,
            string testOnlyCrashPoint)
        {
            string sourceFullPath = Path.GetFullPath(source);
            string destinationFullPath = Path.GetFullPath(destination);
            string quarantineFullPath = Path.GetFullPath(quarantine);
            RequireSameParent(sourceFullPath, destinationFullPath, quarantineFullPath);
            SafeFileHandle[] sidecarReservations = ReserveFinalSidecars(destinationFullPath);
            try
            {
              using (SafeFileHandle parent = OpenAndVerifyParent(
                  Path.GetDirectoryName(destinationFullPath)))
              {
                bool sourceExists = PathEntryExists(sourceFullPath);
                bool destinationExists = PathEntryExists(destinationFullPath);
                bool quarantineExists = PathEntryExists(quarantineFullPath);

                if (destinationExists && sourceExists && !quarantineExists)
                {
                    using (SafeFileHandle original = OpenExact(
                        destinationFullPath, GenericRead, FileShareRead, 0))
                    using (SafeFileHandle staging = OpenExact(
                        sourceFullPath, GenericRead, FileShareRead, 0))
                    {
                        VerifyExactDatabase(
                            original, destinationFullPath, expectedDestinationSha256,
                            "The reconciled staged original database");
                        VerifyExactDatabase(
                            staging, sourceFullPath, expectedSourceSha256,
                            "The reconciled staged upgrade database");
                    }
                    return "STAGED";
                }

                if (!destinationExists && sourceExists && quarantineExists)
                {
                    using (SafeFileHandle quarantined = OpenExact(
                        quarantineFullPath,
                        GenericRead | GenericWrite | Delete,
                        FileShareRead,
                        0))
                    using (SafeFileHandle staging = OpenExact(
                        sourceFullPath, GenericRead, FileShareRead, 0))
                    {
                        VerifyExactDatabase(
                            quarantined, quarantineFullPath, expectedDestinationSha256,
                            "The crash-retained original database");
                        VerifyExactDatabase(
                            staging, sourceFullPath, expectedSourceSha256,
                            "The crash-retained staging database");
                        RenameOpenHandle(quarantined, destinationFullPath, false);
                        VerifyExactDatabase(
                            quarantined, destinationFullPath, expectedDestinationSha256,
                            "The crash-restored original database");
                    }
                    if (!File.Exists(destinationFullPath) || PathEntryExists(quarantineFullPath))
                        throw new InvalidOperationException(
                            "Original crash recovery did not restore one canonical database.");
                    return "STAGED";
                }

                if (destinationExists && !sourceExists && quarantineExists)
                {
                    using (SafeFileHandle published = OpenExact(
                        destinationFullPath, GenericRead, FileShareRead, 0))
                    using (SafeFileHandle quarantined = OpenExact(
                        quarantineFullPath,
                        GenericRead | GenericWrite | Delete,
                        FileShareRead,
                        0))
                    {
                        VerifyExactDatabase(
                            published, destinationFullPath, expectedSourceSha256,
                            "The crash-published upgrade database");
                        // A prior process may have died after durably scrubbing
                        // OLD but before removing Q. Accept either the exact raw
                        // OLD database or that exact now-empty object. Multiple
                        // links are allowed only because all of them are about to
                        // be scrubbed through this same writable handle.
                        VerifyExactDatabaseOrEmptyForScrub(
                            quarantined, quarantineFullPath, expectedDestinationSha256,
                            "The crash-retained original-or-empty database");
                        if (String.Equals(testOnlyAttack, "late-old-hardlink", StringComparison.Ordinal))
                        {
                            if (!CreateHardLink(
                                    Path.GetFullPath(testOnlyAttackPath),
                                    quarantineFullPath,
                                    IntPtr.Zero))
                                throw new Win32Exception(
                                    Marshal.GetLastWin32Error(),
                                "The recovery old-quarantine hardlink injection could not be created");
                        }
                        ZeroOpenHandle(quarantined);
                        VerifyExactEmptyAfterScrub(
                            quarantined, "The recovery-scrubbed original database");
                        CrashAt(testOnlyCrashPoint, "after-old-scrub");
                        DeleteOpenHandle(quarantined);
                        CrashAt(testOnlyCrashPoint, "after-old-disposition");
                        if (GetIdentity(quarantined).NumberOfLinks != 0)
                        {
                            VerifyExactEmptyAfterScrub(
                                quarantined, "A surviving crash-quarantine hardlink");
                            throw new InvalidOperationException(
                                "A late crash-quarantine hardlink survived disposition; all old aliases were scrubbed.");
                        }
                    }
                    if (PathEntryExists(quarantineFullPath))
                        throw new InvalidOperationException(
                            "Published crash recovery did not remove the exact quarantine.");
                    return "PUBLISHED";
                }

                if (destinationExists && !sourceExists && !quarantineExists)
                {
                    using (SafeFileHandle published = OpenExact(
                        destinationFullPath, GenericRead, FileShareRead, 0))
                    {
                        string finalSha256 = HashExactHandle(published);
                        if (String.Equals(finalSha256, expectedSourceSha256, StringComparison.Ordinal))
                        {
                            VerifyExactDatabase(
                                published, destinationFullPath, expectedSourceSha256,
                                "The reconciled published upgrade database");
                            return "PUBLISHED";
                        }
                        if (String.Equals(finalSha256, expectedDestinationSha256, StringComparison.Ordinal))
                        {
                            VerifyExactDatabase(
                                published, destinationFullPath, expectedDestinationSha256,
                                "The scrubbed-staging abort original database");
                            return "STAGING_SCRUBBED_ABORT_REQUIRED";
                        }
                        throw new InvalidOperationException(
                            "The staging-absent canonical database matches neither OLD nor NEW.");
                    }
                }
              }

              throw new InvalidOperationException(
                  "The verified upgrade publication is in an unknown crash state.");
            }
            finally
            {
                DisposeHandles(sidecarReservations);
            }
        }

        public static void PublishVerified(
            string source,
            string destination,
            string expectedSha256,
            bool replaceExisting,
            string testOnlyReplacement)
        {
            string sourceFullPath = Path.GetFullPath(source);
            string destinationFullPath = Path.GetFullPath(destination);
            if (!String.Equals(
                    Path.GetDirectoryName(sourceFullPath),
                    Path.GetDirectoryName(destinationFullPath),
                    StringComparison.OrdinalIgnoreCase))
            {
                throw new InvalidOperationException(
                    "Identity-bound publication requires one fixed directory.");
            }

            using (SafeFileHandle handle = CreateFile(
                sourceFullPath,
                GenericRead | GenericWrite | Delete,
                FileShareRead,
                IntPtr.Zero,
                OpenExisting,
                FileFlagOpenReparsePoint,
                IntPtr.Zero))
            {
                if (handle.IsInvalid)
                    throw new Win32Exception(
                        Marshal.GetLastWin32Error(),
                        "The verified staging database could not be identity-locked");

                ByHandleFileInformation information;
                if (!GetFileInformationByHandle(handle, out information))
                    throw new Win32Exception(
                        Marshal.GetLastWin32Error(),
                        "The locked staging identity could not be queried");
                if ((information.FileAttributes & 0x10) != 0 ||
                    (information.FileAttributes & 0x400) != 0 ||
                    information.NumberOfLinks != 1)
                {
                    throw new InvalidOperationException(
                        "The locked staging database is not a single-link regular file.");
                }

                using (FileStream stream = new FileStream(handle, FileAccess.ReadWrite, 4096, false))
                {
                    string actualSha256;
                    using (SHA256 sha256 = SHA256.Create())
                    {
                        stream.Position = 0;
                        actualSha256 = BitConverter.ToString(sha256.ComputeHash(stream))
                            .Replace("-", "")
                            .ToLowerInvariant();
                    }
                    if (!String.Equals(actualSha256, expectedSha256, StringComparison.Ordinal))
                        throw new InvalidOperationException(
                            "The identity-locked staging database changed after verification.");

                    stream.Position = 0;
                    byte[] header = new byte[16];
                    if (stream.Read(header, 0, header.Length) != header.Length ||
                        !String.Equals(
                            System.Text.Encoding.ASCII.GetString(header),
                            "SQLite format 3\0",
                            StringComparison.Ordinal))
                    {
                        throw new InvalidOperationException(
                            "The identity-locked staging database has an invalid SQLite header.");
                    }
                    stream.Flush(true);

                    if (!String.IsNullOrWhiteSpace(testOnlyReplacement))
                    {
                        bool replaced = MoveFileEx(
                            Path.GetFullPath(testOnlyReplacement),
                            sourceFullPath,
                            MoveFileReplaceExisting | MoveFileWriteThrough);
                        int replacementError = Marshal.GetLastWin32Error();
                        if (replaced)
                            throw new InvalidOperationException(
                                "Injected staging replacement unexpectedly bypassed the identity lock.");
                        throw new Win32Exception(
                            replacementError,
                            "Injected staging replacement was blocked before publication");
                    }

                    // Rename the exact verified file object represented by this
                    // handle.  No later source-path lookup participates in the
                    // publication decision.
                    RenameOpenHandle(handle, destinationFullPath, replaceExisting);
                    stream.Flush(true);
                }
            }
        }
    }
}
'@
    }
}

function Invoke-AtomicFileReplace {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination
    )

    Initialize-AtomicFileType
    [NoTeachingRobot.InstallerAtomicFile]::Replace($Source, $Destination)
}

function Invoke-AtomicFileMoveNoReplace {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination
    )

    Initialize-AtomicFileType
    [NoTeachingRobot.InstallerAtomicFile]::MoveNoReplace($Source, $Destination)
}

function Invoke-IdentityBoundFilePublish {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination,
        [Parameter(Mandatory = $true)][string]$ExpectedSha256,
        [switch]$ReplaceExisting
    )

    Initialize-AtomicFileType
    [NoTeachingRobot.InstallerAtomicFile]::PublishVerified(
        $Source,
        $Destination,
        $ExpectedSha256,
        [bool]$ReplaceExisting,
        $TestOnlyStagingReplacementPath
    )
}

function Invoke-IdentityBoundUpgradePublish {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination,
        [Parameter(Mandatory = $true)][string]$Quarantine,
        [Parameter(Mandatory = $true)][string]$ExpectedSourceSha256,
        [Parameter(Mandatory = $true)][string]$ExpectedDestinationSha256
    )

    Initialize-AtomicFileType
    [NoTeachingRobot.InstallerAtomicFile]::PublishUpgradeVerified(
        $Source,
        $Destination,
        $Quarantine,
        $ExpectedSourceSha256,
        $ExpectedDestinationSha256,
        $TestOnlyStagingReplacementPath,
        $TestOnlyUpgradeAttack,
        $TestOnlyUpgradeAttackPath,
        $TestOnlyUpgradeCrashPoint
    )
}

function Invoke-IdentityBoundUpgradeReconciliation {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination,
        [Parameter(Mandatory = $true)][string]$Quarantine,
        [Parameter(Mandatory = $true)][string]$ExpectedSourceSha256,
        [Parameter(Mandatory = $true)][string]$ExpectedDestinationSha256
    )

    Initialize-AtomicFileType
    return [NoTeachingRobot.InstallerAtomicFile]::ReconcileUpgradeVerified(
        $Source,
        $Destination,
        $Quarantine,
        $ExpectedSourceSha256,
        $ExpectedDestinationSha256,
        $TestOnlyUpgradeAttack,
        $TestOnlyUpgradeAttackPath,
        $TestOnlyUpgradeCrashPoint
    )
}

function Invoke-IdentityBoundSecureDiscard {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$ExpectedSha256,
        [Parameter(Mandatory = $true)]
        [ValidateSet('staging-discard', 'backup-discard', 'readback-discard')]
        [string]$Scope
    )

    $testHardlinkPath = ''
    if ($TestOnlyUpgradeAttack -ceq ($Scope + '-hardlink')) {
        if ([string]::IsNullOrWhiteSpace($TestOnlyUpgradeAttackPath)) {
            throw 'The secure-discard hardlink test hook requires an alias path.'
        }
        $testHardlinkPath = $TestOnlyUpgradeAttackPath
    }

    Initialize-AtomicFileType
    [NoTeachingRobot.InstallerAtomicFile]::DiscardVerified(
        $Path,
        $ExpectedSha256,
        $testHardlinkPath,
        $TestOnlyUpgradeCrashPoint,
        $Scope
    )
}

function Write-AtomicUtf8Text {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Value,
        [switch]$ReplaceExisting
    )

    $destinationExists = Test-Path -LiteralPath $Path
    if ($destinationExists -and -not $ReplaceExisting) {
        throw 'A pending installer transaction record already exists.'
    }
    if ($destinationExists -and
        (-not (Test-Path -LiteralPath $Path -PathType Leaf) -or
         ((Get-Item -LiteralPath $Path -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0)) {
        throw 'The pending installer transaction record cannot be atomically replaced.'
    }
    $parent = Split-Path -Parent $Path
    [System.IO.Directory]::CreateDirectory($parent) | Out-Null
    $temporaryPath = Join-Path $parent (
        '.' + [System.IO.Path]::GetFileName($Path) + '.' + [Guid]::NewGuid().ToString('N') + '.tmp'
    )
    $encoding = New-Object System.Text.UTF8Encoding($false)
    $bytes = $encoding.GetBytes($Value)
    $stream = $null
    try {
        $stream = New-Object System.IO.FileStream(
            $temporaryPath,
            [System.IO.FileMode]::CreateNew,
            [System.IO.FileAccess]::Write,
            [System.IO.FileShare]::None,
            4096,
            [System.IO.FileOptions]::WriteThrough
        )
        $stream.Write($bytes, 0, $bytes.Length)
        $stream.Flush($true)
        $stream.Dispose()
        $stream = $null
        # MoveFileEx with WRITE_THROUGH is required for both the first PREPARED
        # write and later phase advancement.  A plain File.Move here would make
        # the most important write-ahead record the least durable one.
        if ($destinationExists) {
            Invoke-AtomicFileReplace $temporaryPath $Path
        }
        else {
            Invoke-AtomicFileMoveNoReplace $temporaryPath $Path
        }
    }
    finally {
        if ($null -ne $stream) {
            $stream.Dispose()
        }
        if (Test-Path -LiteralPath $temporaryPath) {
            Remove-Item -LiteralPath $temporaryPath -Force -ErrorAction SilentlyContinue
        }
    }
}

function Read-PendingTransactionRecord {
    if (-not (Test-Path -LiteralPath $script:TransactionPath)) {
        return $null
    }
    if (-not (Test-Path -LiteralPath $script:TransactionPath -PathType Leaf)) {
        throw 'The pending installer transaction record is not a regular file.'
    }
    if (((Get-Item -LiteralPath $script:TransactionPath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'The pending installer transaction record is a reparse point.'
    }

    $strictUtf8 = New-Object System.Text.UTF8Encoding($false, $true)
    $text = $strictUtf8.GetString([System.IO.File]::ReadAllBytes($script:TransactionPath))
    if (-not $text.EndsWith("`n") -or $text.Contains("`r")) {
        throw 'The pending installer transaction record has invalid line framing.'
    }
    $lines = $text.Substring(0, $text.Length - 1).Split([string[]]@("`n"), [System.StringSplitOptions]::None)
    if ($lines.Count -lt 3 -or $lines[0] -cne ('FORMAT=' + $script:TransactionFormat)) {
        throw 'The pending installer transaction record has an invalid format marker.'
    }

    $payloadLines = $lines[0..($lines.Count - 2)]
    $payload = $payloadLines -join "`n"
    $expectedDigest = Get-TextSha256 $payload
    if ($lines[$lines.Count - 1] -cne ('PAYLOAD_SHA256=' + $expectedDigest)) {
        throw 'The pending installer transaction record failed its payload digest check.'
    }

    if ($lines[1] -ceq 'MODE=UPGRADE_PREPARED') {
        if ($lines.Count -ne 6 -or
            $lines[2] -cnotmatch '^STAGING_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' -or
            $lines[3] -cnotmatch '^BACKUP_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' -or
            $lines[3] -cne ('BACKUP_NAME=' + $lines[2].Substring('STAGING_NAME='.Length) + '.install-backup.dpapi.bak') -or
            $lines[4] -cnotmatch '^ORIGINAL_SHA256=[0-9a-f]{64}$') {
            throw 'The prepared upgrade transaction record has unknown or invalid fields.'
        }
        return [pscustomobject]@{
            Mode = 'UPGRADE_PREPARED'
            StagingName = $lines[2].Substring('STAGING_NAME='.Length)
            BackupName = $lines[3].Substring('BACKUP_NAME='.Length)
            OriginalSha256 = $lines[4].Substring('ORIGINAL_SHA256='.Length)
        }
    }

    if ($lines[1] -ceq 'MODE=UPGRADE_VERIFIED') {
        $stagingName = if ($lines.Count -ge 3) {
            $lines[2].Substring('STAGING_NAME='.Length)
        }
        else { '' }
        $derivedQuarantineName = $stagingName + '.install-original.quarantine'
        # The eight-line shape was written by the immediately preceding,
        # unreleased helper. Accept it only with a strictly derived quarantine
        # name so an interrupted local installation can be resumed safely.
        $legacyShape = $lines.Count -eq 8
        $fieldOffset = if ($legacyShape) { 0 } else { 1 }
        if ($lines.Count -notin @(8, 9) -or
            $lines[2] -cnotmatch '^STAGING_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' -or
            (-not $legacyShape -and
                ($lines[3] -cnotmatch '^QUARANTINE_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-original\.quarantine$' -or
                 $lines[3] -cne ('QUARANTINE_NAME=' + $derivedQuarantineName))) -or
            $lines[3 + $fieldOffset] -cnotmatch '^BACKUP_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' -or
            $lines[3 + $fieldOffset] -cne ('BACKUP_NAME=' + $stagingName + '.install-backup.dpapi.bak') -or
            $lines[4 + $fieldOffset] -cnotmatch '^BACKUP_SHA256=[0-9a-f]{64}$' -or
            $lines[5 + $fieldOffset] -cnotmatch '^ORIGINAL_SHA256=[0-9a-f]{64}$' -or
            $lines[6 + $fieldOffset] -cnotmatch '^MIGRATED_SHA256=[0-9a-f]{64}$') {
            throw 'The pending upgrade transaction record has unknown or invalid fields.'
        }
        return [pscustomobject]@{
            Mode = 'UPGRADE_VERIFIED'
            StagingName = $stagingName
            QuarantineName = $derivedQuarantineName
            BackupName = $lines[3 + $fieldOffset].Substring('BACKUP_NAME='.Length)
            BackupSha256 = $lines[4 + $fieldOffset].Substring('BACKUP_SHA256='.Length)
            OriginalSha256 = $lines[5 + $fieldOffset].Substring('ORIGINAL_SHA256='.Length)
            MigratedSha256 = $lines[6 + $fieldOffset].Substring('MIGRATED_SHA256='.Length)
        }
    }

    if ($lines[1] -ceq 'MODE=CREATE_PREPARED') {
        if ($lines.Count -ne 6 -or
            $lines[2] -cnotmatch '^STAGING_NAME=\.ConfigStore\.db\.install-create-[0-9a-f]{32}\.tmp$' -or
            $lines[3] -cnotmatch '^SOURCE_INVENTORY_SHA256=[0-9a-f]{64}$' -or
            $lines[4] -cne 'ORIGINAL_STATE=ABSENT') {
            throw 'The prepared create transaction record has unknown or invalid fields.'
        }
        return [pscustomobject]@{
            Mode = 'CREATE_PREPARED'
            StagingName = $lines[2].Substring('STAGING_NAME='.Length)
            SourceInventorySha256 = $lines[3].Substring('SOURCE_INVENTORY_SHA256='.Length)
        }
    }

    if ($lines[1] -in @('MODE=CREATE_READY', 'MODE=CREATE_VERIFIED')) {
        if ($lines.Count -ne 7 -or
            $lines[2] -cnotmatch '^STAGING_NAME=\.ConfigStore\.db\.install-create-[0-9a-f]{32}\.tmp$' -or
            $lines[3] -cnotmatch '^SOURCE_INVENTORY_SHA256=[0-9a-f]{64}$' -or
            $lines[4] -cne 'ORIGINAL_STATE=ABSENT' -or
            $lines[5] -cnotmatch '^CREATED_SHA256=[0-9a-f]{64}$') {
            throw 'The pending create transaction record has unknown or invalid fields.'
        }
        return [pscustomobject]@{
            Mode = $lines[1].Substring('MODE='.Length)
            StagingName = $lines[2].Substring('STAGING_NAME='.Length)
            SourceInventorySha256 = $lines[3].Substring('SOURCE_INVENTORY_SHA256='.Length)
            CreatedSha256 = $lines[5].Substring('CREATED_SHA256='.Length)
        }
    }

    if ($lines[1] -ceq 'MODE=NOOP_ABSENT') {
        if ($lines.Count -ne 5 -or
            $lines[2] -cnotmatch '^SOURCE_INVENTORY_SHA256=[0-9a-f]{64}$' -or
            $lines[3] -cne 'ORIGINAL_STATE=ABSENT') {
            throw 'The absent no-op transaction record has unknown or invalid fields.'
        }
        return [pscustomobject]@{
            Mode = 'NOOP_ABSENT'
            SourceInventorySha256 = $lines[2].Substring('SOURCE_INVENTORY_SHA256='.Length)
        }
    }

    if ($lines[1] -ceq 'MODE=NOOP_CURRENT') {
        if ($lines.Count -ne 4 -or $lines[2] -cnotmatch '^CURRENT_SHA256=[0-9a-f]{64}$') {
            throw 'The current no-op transaction record has unknown or invalid fields.'
        }
        return [pscustomobject]@{
            Mode = 'NOOP_CURRENT'
            CurrentSha256 = $lines[2].Substring('CURRENT_SHA256='.Length)
        }
    }

    if ($lines[1] -ceq 'MODE=PUBLISHED_FINALIZE_PENDING') {
        if ($lines.Count -ge 3 -and $lines[2] -ceq 'PUBLISHED_KIND=NOOP_CURRENT') {
            if ($lines.Count -ne 5 -or
                $lines[3] -cnotmatch '^PENDING_SHA256=[0-9a-f]{64}$') {
                throw 'The published no-op finalize transaction has unknown or invalid fields.'
            }
            return [pscustomobject]@{
                Mode = 'PUBLISHED_FINALIZE_PENDING'
                PublishedKind = 'NOOP_CURRENT'
                PendingSha256 = $lines[3].Substring('PENDING_SHA256='.Length)
            }
        }
        if ($lines.Count -ge 3 -and $lines[2] -ceq 'PUBLISHED_KIND=CREATE') {
            if ($lines.Count -ne 7 -or
                $lines[3] -cnotmatch '^STAGING_NAME=\.ConfigStore\.db\.install-create-[0-9a-f]{32}\.tmp$' -or
                $lines[4] -cnotmatch '^PENDING_SHA256=[0-9a-f]{64}$' -or
                $lines[5] -cnotmatch '^SOURCE_INVENTORY_SHA256=[0-9a-f]{64}$') {
                throw 'The published create finalize transaction has unknown or invalid fields.'
            }
            return [pscustomobject]@{
                Mode = 'PUBLISHED_FINALIZE_PENDING'
                PublishedKind = 'CREATE'
                StagingName = $lines[3].Substring('STAGING_NAME='.Length)
                PendingSha256 = $lines[4].Substring('PENDING_SHA256='.Length)
                SourceInventorySha256 = $lines[5].Substring('SOURCE_INVENTORY_SHA256='.Length)
            }
        }
        if ($lines.Count -ge 3 -and $lines[2] -ceq 'PUBLISHED_KIND=UPGRADE') {
            if ($lines.Count -ne 9 -or
                $lines[3] -cnotmatch '^STAGING_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' -or
                $lines[4] -cnotmatch '^BACKUP_NAME=\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' -or
                $lines[4] -cne ('BACKUP_NAME=' + $lines[3].Substring('STAGING_NAME='.Length) + '.install-backup.dpapi.bak') -or
                $lines[5] -cnotmatch '^BACKUP_SHA256=[0-9a-f]{64}$' -or
                $lines[6] -cnotmatch '^ORIGINAL_SHA256=[0-9a-f]{64}$' -or
                $lines[7] -cnotmatch '^PENDING_SHA256=[0-9a-f]{64}$') {
                throw 'The published upgrade finalize transaction has unknown or invalid fields.'
            }
            return [pscustomobject]@{
                Mode = 'PUBLISHED_FINALIZE_PENDING'
                PublishedKind = 'UPGRADE'
                StagingName = $lines[3].Substring('STAGING_NAME='.Length)
                BackupName = $lines[4].Substring('BACKUP_NAME='.Length)
                BackupSha256 = $lines[5].Substring('BACKUP_SHA256='.Length)
                OriginalSha256 = $lines[6].Substring('ORIGINAL_SHA256='.Length)
                PendingSha256 = $lines[7].Substring('PENDING_SHA256='.Length)
            }
        }
        throw 'The published finalize transaction has an unknown kind.'
    }

    throw 'The pending installer transaction record has an unknown mode.'
}

function Write-PendingUpgradePrepared {
    param(
        [Parameter(Mandatory = $true)][string]$StagingName,
        [Parameter(Mandatory = $true)][string]$BackupName,
        [Parameter(Mandatory = $true)][string]$OriginalSha256
    )

    if ($StagingName -cnotmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' -or
        $BackupName -cnotmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' -or
        $BackupName -cne ($StagingName + '.install-backup.dpapi.bak') -or
        $OriginalSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'Cannot persist a non-canonical prepared upgrade transaction.'
    }
    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=UPGRADE_PREPARED'
        'STAGING_NAME=' + $StagingName
        'BACKUP_NAME=' + $BackupName
        'ORIGINAL_SHA256=' + $OriginalSha256
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n")
    $record = Read-PendingTransactionRecord
    if ($null -eq $record -or $record.Mode -cne 'UPGRADE_PREPARED' -or
        $record.StagingName -cne $StagingName -or
        $record.BackupName -cne $BackupName -or
        $record.OriginalSha256 -cne $OriginalSha256) {
        throw 'The prepared upgrade transaction record failed read-back verification.'
    }
}

function Advance-PendingUpgradeVerified {
    param(
        [Parameter(Mandatory = $true)][string]$BackupSha256,
        [Parameter(Mandatory = $true)][string]$MigratedSha256
    )

    $prepared = Read-PendingTransactionRecord
    if ($null -eq $prepared -or $prepared.Mode -cne 'UPGRADE_PREPARED' -or
        $BackupSha256 -cnotmatch '^[0-9a-f]{64}$' -or
        $MigratedSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'Cannot advance a missing or invalid prepared upgrade transaction.'
    }
    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=UPGRADE_VERIFIED'
        'STAGING_NAME=' + $prepared.StagingName
        'QUARANTINE_NAME=' + $prepared.StagingName + '.install-original.quarantine'
        'BACKUP_NAME=' + $prepared.BackupName
        'BACKUP_SHA256=' + $BackupSha256
        'ORIGINAL_SHA256=' + $prepared.OriginalSha256
        'MIGRATED_SHA256=' + $MigratedSha256
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath `
        ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n") -ReplaceExisting
    $verified = Read-PendingTransactionRecord
    if ($null -eq $verified -or $verified.Mode -cne 'UPGRADE_VERIFIED' -or
        $verified.StagingName -cne $prepared.StagingName -or
        $verified.QuarantineName -cne ($prepared.StagingName + '.install-original.quarantine') -or
        $verified.BackupName -cne $prepared.BackupName -or
        $verified.BackupSha256 -cne $BackupSha256 -or
        $verified.OriginalSha256 -cne $prepared.OriginalSha256 -or
        $verified.MigratedSha256 -cne $MigratedSha256) {
        throw 'The verified upgrade transaction record failed read-back verification.'
    }
}

function Write-PendingCreatePrepared {
    param(
        [Parameter(Mandatory = $true)][string]$StagingName,
        [Parameter(Mandatory = $true)][string]$SourceInventorySha256
    )

    if ($StagingName -cnotmatch '^\.ConfigStore\.db\.install-create-[0-9a-f]{32}\.tmp$' -or
        $SourceInventorySha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'Cannot persist a non-canonical prepared create transaction.'
    }
    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=CREATE_PREPARED'
        'STAGING_NAME=' + $StagingName
        'SOURCE_INVENTORY_SHA256=' + $SourceInventorySha256
        'ORIGINAL_STATE=ABSENT'
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n")
    $record = Read-PendingTransactionRecord
    if ($null -eq $record -or $record.Mode -cne 'CREATE_PREPARED' -or
        $record.StagingName -cne $StagingName -or
        $record.SourceInventorySha256 -cne $SourceInventorySha256) {
        throw 'The prepared create transaction record failed read-back verification.'
    }
}

function Advance-PendingCreateReady {
    param([Parameter(Mandatory = $true)][string]$CreatedSha256)

    $prepared = Read-PendingTransactionRecord
    if ($null -eq $prepared -or $prepared.Mode -cne 'CREATE_PREPARED' -or
        $CreatedSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'Cannot advance a missing or invalid prepared create transaction.'
    }
    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=CREATE_READY'
        'STAGING_NAME=' + $prepared.StagingName
        'SOURCE_INVENTORY_SHA256=' + $prepared.SourceInventorySha256
        'ORIGINAL_STATE=ABSENT'
        'CREATED_SHA256=' + $CreatedSha256
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath `
        ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n") -ReplaceExisting
    $record = Read-PendingTransactionRecord
    if ($null -eq $record -or $record.Mode -cne 'CREATE_READY' -or
        $record.StagingName -cne $prepared.StagingName -or
        $record.SourceInventorySha256 -cne $prepared.SourceInventorySha256 -or
        $record.CreatedSha256 -cne $CreatedSha256) {
        throw 'The ready create transaction record failed read-back verification.'
    }
}

function Advance-PendingCreateVerified {
    $ready = Read-PendingTransactionRecord
    if ($null -eq $ready -or $ready.Mode -cne 'CREATE_READY') {
        throw 'Cannot verify a create transaction that is not ready for publication.'
    }
    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=CREATE_VERIFIED'
        'STAGING_NAME=' + $ready.StagingName
        'SOURCE_INVENTORY_SHA256=' + $ready.SourceInventorySha256
        'ORIGINAL_STATE=ABSENT'
        'CREATED_SHA256=' + $ready.CreatedSha256
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath `
        ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n") -ReplaceExisting
    $record = Read-PendingTransactionRecord
    if ($null -eq $record -or $record.Mode -cne 'CREATE_VERIFIED' -or
        $record.StagingName -cne $ready.StagingName -or
        $record.SourceInventorySha256 -cne $ready.SourceInventorySha256 -or
        $record.CreatedSha256 -cne $ready.CreatedSha256) {
        throw 'The verified create transaction record failed read-back verification.'
    }
}

function Write-PendingNoopAbsent {
    param(
        [Parameter(Mandatory = $true)][string]$SourceInventorySha256,
        [switch]$ReplaceExisting
    )

    $emptyInventorySha256 = Get-TextSha256 "NoTeaching-Robot-Data-Input-Inventory-v1`n0`n"
    if ($SourceInventorySha256 -cnotmatch '^[0-9a-f]{64}$' -or
        $SourceInventorySha256 -cne $emptyInventorySha256) {
        throw 'Cannot persist a non-canonical absent no-op transaction.'
    }

    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=NOOP_ABSENT'
        'SOURCE_INVENTORY_SHA256=' + $SourceInventorySha256
        'ORIGINAL_STATE=ABSENT'
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath `
        ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n") `
        -ReplaceExisting:$ReplaceExisting
    $record = Read-PendingTransactionRecord
    if ($null -eq $record -or $record.Mode -cne 'NOOP_ABSENT' -or
        $record.SourceInventorySha256 -cne $SourceInventorySha256) {
        throw 'The absent no-op transaction record failed read-back verification.'
    }
}

function Write-PendingNoopCurrent {
    param(
        [Parameter(Mandatory = $true)][string]$CurrentSha256,
        [switch]$ReplaceExisting
    )

    if ($CurrentSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'Cannot persist a non-canonical current no-op transaction.'
    }
    $payload = @(
        'FORMAT=' + $script:TransactionFormat
        'MODE=NOOP_CURRENT'
        'CURRENT_SHA256=' + $CurrentSha256
    ) -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath `
        ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n") `
        -ReplaceExisting:$ReplaceExisting
    $record = Read-PendingTransactionRecord
    if ($null -eq $record -or $record.Mode -cne 'NOOP_CURRENT' -or
        $record.CurrentSha256 -cne $CurrentSha256) {
        throw 'The current no-op transaction record failed read-back verification.'
    }
}

function Write-PublishedFinalizePending {
    param([Parameter(Mandatory = $true)]$Record)

    if ($Record.Mode -ceq 'NOOP_CURRENT') {
        $payloadLines = @(
            'FORMAT=' + $script:TransactionFormat
            'MODE=PUBLISHED_FINALIZE_PENDING'
            'PUBLISHED_KIND=NOOP_CURRENT'
            'PENDING_SHA256=' + $Record.CurrentSha256
        )
    }
    elseif ($Record.Mode -ceq 'CREATE_VERIFIED') {
        $payloadLines = @(
            'FORMAT=' + $script:TransactionFormat
            'MODE=PUBLISHED_FINALIZE_PENDING'
            'PUBLISHED_KIND=CREATE'
            'STAGING_NAME=' + $Record.StagingName
            'PENDING_SHA256=' + $Record.CreatedSha256
            'SOURCE_INVENTORY_SHA256=' + $Record.SourceInventorySha256
        )
    }
    elseif ($Record.Mode -ceq 'UPGRADE_VERIFIED') {
        $payloadLines = @(
            'FORMAT=' + $script:TransactionFormat
            'MODE=PUBLISHED_FINALIZE_PENDING'
            'PUBLISHED_KIND=UPGRADE'
            'STAGING_NAME=' + $Record.StagingName
            'BACKUP_NAME=' + $Record.BackupName
            'BACKUP_SHA256=' + $Record.BackupSha256
            'ORIGINAL_SHA256=' + $Record.OriginalSha256
            'PENDING_SHA256=' + $Record.MigratedSha256
        )
    }
    else {
        throw 'Only a published pending-scrub transaction can enter finalize-pending state.'
    }

    $payload = $payloadLines -join "`n"
    Write-AtomicUtf8Text $script:TransactionPath `
        ($payload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $payload) + "`n") -ReplaceExisting
    $written = Read-PendingTransactionRecord
    if ($null -eq $written -or
        $written.Mode -cne 'PUBLISHED_FINALIZE_PENDING') {
        throw 'The published finalize-pending transaction failed read-back verification.'
    }
    if (($Record.Mode -ceq 'NOOP_CURRENT' -and
            ($written.PublishedKind -cne 'NOOP_CURRENT' -or
             $written.PendingSha256 -cne $Record.CurrentSha256)) -or
        ($Record.Mode -ceq 'CREATE_VERIFIED' -and
            ($written.PublishedKind -cne 'CREATE' -or
             $written.StagingName -cne $Record.StagingName -or
             $written.PendingSha256 -cne $Record.CreatedSha256 -or
             $written.SourceInventorySha256 -cne $Record.SourceInventorySha256)) -or
        ($Record.Mode -ceq 'UPGRADE_VERIFIED' -and
            ($written.PublishedKind -cne 'UPGRADE' -or
             $written.StagingName -cne $Record.StagingName -or
             $written.BackupName -cne $Record.BackupName -or
             $written.BackupSha256 -cne $Record.BackupSha256 -or
             $written.OriginalSha256 -cne $Record.OriginalSha256 -or
             $written.PendingSha256 -cne $Record.MigratedSha256))) {
        throw 'The published finalize-pending transaction fields failed read-back verification.'
    }
    return $written
}

function Remove-PendingTransactionRecord {
    if (-not (Test-Path -LiteralPath $script:TransactionPath)) {
        return
    }
    if (-not (Test-Path -LiteralPath $script:TransactionPath -PathType Leaf) -or
        ((Get-Item -LiteralPath $script:TransactionPath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'The pending installer transaction record is not a regular file.'
    }
    Remove-Item -LiteralPath $script:TransactionPath -Force
    if (Test-Path -LiteralPath $script:TransactionPath) {
        throw 'The pending installer transaction record could not be removed.'
    }
}

function ConvertTo-NativeQuotedArgument {
    param([Parameter(Mandatory = $true)][string]$Value)

    # Windows file names cannot contain a quote.  Reject it explicitly instead of
    # relying on cmd.exe-like re-parsing.  ProcessStartInfo launches the executable
    # directly, so metacharacters such as &, |, (, and ) remain ordinary path data.
    if ($Value.IndexOf([char]0) -ge 0 -or $Value.Contains('"')) {
        throw 'A native process argument contains an unsupported quote or NUL character.'
    }
    return '"' + $Value + '"'
}

function Invoke-ConfigMigrate {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)

    $start = New-Object System.Diagnostics.ProcessStartInfo
    $start.FileName = $script:MigratePath
    $start.Arguments = (($Arguments | ForEach-Object { ConvertTo-NativeQuotedArgument ([string]$_) }) -join ' ')
    # Never make the user-writable Data directory the DLL/runtime search base of
    # an elevated executable.  The migrator is extracted from the signed installer
    # into its private temporary directory.
    $start.WorkingDirectory = Split-Path -Parent $script:MigratePath
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $start
    $started = $false
    try {
        if (-not $process.Start()) {
            throw 'ConfigMigrate could not be started.'
        }
        $started = $true
        $stdoutTask = $process.StandardOutput.ReadToEndAsync()
        $stderrTask = $process.StandardError.ReadToEndAsync()
        $process.WaitForExit()
        $stdout = $stdoutTask.Result
        $stderr = $stderrTask.Result
        $exitCode = $process.ExitCode

        return [pscustomobject]@{
            ExitCode = [int]$exitCode
            StdOut = [string]$stdout
            StdErr = [string]$stderr
        }
    }
    finally {
        if ($started -and -not $process.HasExited) {
            try {
                $process.Kill()
                $process.WaitForExit()
            }
            catch {
                # The outer transaction remains PREPARED and fail-closed if the
                # signed migrator cannot be synchronously reaped.
            }
        }
        $process.Dispose()
    }
}

function Get-SafeMigrationFailureDetail {
    param([Parameter(Mandatory = $true)]$Migration)

    # Never echo raw migrator output into the installer UI: source paths and
    # malformed configuration text can contain operational or credential data.
    # Map known failure families to a bounded, user-actionable explanation.
    $text = ([string]$Migration.StdOut) + "`n" + ([string]$Migration.StdErr)
    if ($text.IndexOf('DPAPI', [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return 'Windows 当前用户无法解密敏感配置或创建受保护备份。'
    }
    if ($text.IndexOf('Cannot decode', [System.StringComparison]::OrdinalIgnoreCase) -ge 0 -or
        $text.IndexOf('account/Profile', [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return '账号记录格式或保护状态无法安全转换。'
    }
    if ($text.IndexOf('credential scrub', [System.StringComparison]::OrdinalIgnoreCase) -ge 0 -or
        $text.IndexOf('credential source', [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return '旧凭据清理状态或来源校验失败。'
    }
    if ($text.IndexOf('integrity', [System.StringComparison]::OrdinalIgnoreCase) -ge 0 -or
        $text.IndexOf('SQLite', [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return '数据库完整性或 SQLite 事务校验失败。'
    }
    if ($text.IndexOf('sidecar', [System.StringComparison]::OrdinalIgnoreCase) -ge 0 -or
        $text.IndexOf('authority', [System.StringComparison]::OrdinalIgnoreCase) -ge 0 -or
        $text.IndexOf('changed', [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return '升级期间数据库文件状态发生变化，或存在未完成的 SQLite 临时文件。'
    }
    if ($text.IndexOf('schema', [System.StringComparison]::OrdinalIgnoreCase) -ge 0) {
        return '数据库版本或表结构不受当前升级程序支持。'
    }
    return '迁移器拒绝了当前数据；为避免泄露敏感配置，界面未显示原始输出。请保留 Data 目录。'
}

function Test-SqliteHeader {
    param([Parameter(Mandatory = $true)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        return $false
    }
    $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read)
    try {
        if ($stream.Length -lt 16) {
            return $false
        }
        $header = New-Object byte[] 16
        if ($stream.Read($header, 0, $header.Length) -ne $header.Length) {
            return $false
        }
        return [System.Text.Encoding]::ASCII.GetString($header) -ceq "SQLite format 3`0"
    }
    finally {
        $stream.Dispose()
    }
}

function Test-ProtectedBackupHeader {
    param([Parameter(Mandatory = $true)][string]$Path)

    $backupFullPath = [System.IO.Path]::GetFullPath($Path)
    if (-not [System.StringComparer]::OrdinalIgnoreCase.Equals(
            (Split-Path -Parent $backupFullPath), $script:DataPath) -or
        ((Get-Item -LiteralPath $backupFullPath -Force).Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        return $false
    }
    $magic = [System.Text.Encoding]::ASCII.GetBytes("NoTeaching-Robot ConfigStore DPAPI backup v1`n")
    $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read)
    try {
        if ($stream.Length -le ($magic.Length + 64)) {
            return $false
        }
        $prefix = New-Object byte[] $magic.Length
        if ($stream.Read($prefix, 0, $prefix.Length) -ne $prefix.Length) {
            return $false
        }
        for ($i = 0; $i -lt $magic.Length; ++$i) {
            if ($prefix[$i] -ne $magic[$i]) {
                return $false
            }
        }
        return $true
    }
    finally {
        $stream.Dispose()
    }
}

function Restore-ProtectedBackupToStaging {
    param([Parameter(Mandatory = $true)][string]$BackupPath)

    if (-not (Test-ProtectedBackupHeader $BackupPath)) {
        throw 'ConfigMigrate produced an invalid protected-backup envelope.'
    }

    $backupName = [System.IO.Path]::GetFileName($BackupPath)
    if ($backupName -cnotmatch '^\.ConfigStore\.db\.install-upgrade-([0-9a-f]{32})\.tmp\.install-backup\.dpapi\.bak$') {
        throw 'The protected-backup readback name is not transaction-bound.'
    }
    $stagingPath = Join-Path $script:DataPath ('.ConfigStore.db.install-readback-' + $Matches[1] + '.tmp')
    Remove-BoundSensitiveDatabaseArtifact `
        -Path $stagingPath `
        -NamePattern '^\.ConfigStore\.db\.install-readback-[0-9a-f]{32}\.tmp$' `
        -Description 'A retained protected-backup readback database' `
        -Scope 'readback-discard'
    try {
        $restore = Invoke-ConfigMigrate @(
            '--restore-dpapi-backup', $BackupPath,
            '--db', $stagingPath
        )
        if ($restore.ExitCode -ne 0 -or -not (Test-SqliteHeader $stagingPath)) {
            throw ('Protected database backup failed read-back verification (exit code {0}).' -f $restore.ExitCode)
        }
        return $stagingPath
    }
    catch {
        Remove-BoundSensitiveDatabaseArtifact `
            -Path $stagingPath `
            -NamePattern '^\.ConfigStore\.db\.install-readback-[0-9a-f]{32}\.tmp$' `
            -Description 'The failed protected-backup readback database' `
            -Scope 'readback-discard'
        throw
    }
}

function Test-ProtectedBackupReadback {
    param(
        [Parameter(Mandatory = $true)][string]$BackupPath,
        [Parameter(Mandatory = $true)][string]$ExpectedDatabaseSha256,
        [string]$ComparisonDatabasePath = $script:DatabasePath,
        [switch]$AllowOriginalUnavailable
    )

    if ($ExpectedDatabaseSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'The expected original database hash is invalid.'
    }
    $backupSha256Before = Get-FileSha256 $BackupPath
    $stagingPath = Restore-ProtectedBackupToStaging $BackupPath
    try {
        # ConfigMigrate intentionally creates a consistent SQLite backup snapshot.
        # SQLite may serialize that logically identical snapshot with different
        # page bytes than the source file, so a raw restored-file SHA comparison
        # is not a valid equivalence check.  While the original database is still
        # present, require the signed migrator to compare canonical snapshots.
        $originalAvailable = (
            (Test-Path -LiteralPath $ComparisonDatabasePath -PathType Leaf) -and
            (Get-FileSha256 $ComparisonDatabasePath) -ceq $ExpectedDatabaseSha256
        )
        if ($originalAvailable) {
            $logicalVerification = Invoke-ConfigMigrate @(
                '--verify-dpapi-backup-against', $BackupPath,
                '--db', $ComparisonDatabasePath
            )
            if ($logicalVerification.ExitCode -ne 0) {
                throw 'The protected backup does not match the original database logical snapshot.'
            }
            if ((Get-FileSha256 $ComparisonDatabasePath) -cne $ExpectedDatabaseSha256) {
                throw 'The original database changed during protected-backup verification.'
            }
        }
        elseif (-not $AllowOriginalUnavailable) {
            throw 'The original database is unavailable for protected-backup comparison.'
        }
        if ((Get-FileSha256 $BackupPath) -cne $backupSha256Before) {
            throw 'The protected backup changed during logical read-back verification.'
        }
    }
    finally {
        Remove-BoundSensitiveDatabaseArtifact `
            -Path $stagingPath `
            -NamePattern '^\.ConfigStore\.db\.install-readback-[0-9a-f]{32}\.tmp$' `
            -Description 'The protected-backup readback database' `
            -Scope 'readback-discard'
    }
}

function Assert-NoDatabaseSidecars {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Context
    )

    foreach ($suffix in @('-journal', '-wal', '-shm')) {
        if (Test-Path -LiteralPath ($Path + $suffix)) {
            throw ($Context + ' has a SQLite sidecar; replay across the installer transaction is refused.')
        }
    }
}

function Assert-RegularFileInData {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$NamePattern,
        [Parameter(Mandatory = $true)][string]$Description
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if (-not [System.StringComparer]::OrdinalIgnoreCase.Equals(
            (Split-Path -Parent $fullPath), $script:DataPath) -or
        [System.IO.Path]::GetFileName($fullPath) -cnotmatch $NamePattern) {
        throw ($Description + ' is outside the fixed Data scope.')
    }
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf) -or
        ((Get-Item -LiteralPath $fullPath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw ($Description + ' is missing, not a regular file, or a reparse point.')
    }
}

function Remove-BoundSensitiveDatabaseArtifact {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$NamePattern,
        [Parameter(Mandatory = $true)][string]$Description,
        [Parameter(Mandatory = $true)]
        [ValidateSet('staging-discard', 'readback-discard')]
        [string]$Scope,
        [string]$ExpectedSha256 = ''
    )

    # Sidecars are not independently bound in the durable transaction record.
    # Preserve them and the record as evidence instead of deleting by pathname.
    Assert-NoDatabaseSidecars $Path $Description
    if (-not (Test-Path -LiteralPath $Path)) {
        return
    }
    Assert-RegularFileInData $Path $NamePattern $Description
    $boundSha256 = $ExpectedSha256
    if ([string]::IsNullOrWhiteSpace($boundSha256)) {
        $boundSha256 = Get-FileSha256 $Path
    }
    if ($boundSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw ($Description + ' has no valid exact hash binding for secure discard.')
    }
    Invoke-IdentityBoundSecureDiscard $Path $boundSha256 $Scope
    if (Test-Path -LiteralPath $Path) {
        throw ($Description + ' survived exact-handle secure discard.')
    }
}

function Remove-BoundTransactionStaging {
    param(
        [Parameter(Mandatory = $true)][string]$StagingName,
        [string]$ExpectedSha256 = ''
    )

    if ($StagingName -cnotmatch '^\.ConfigStore\.db\.install-(create|upgrade)-[0-9a-f]{32}\.tmp$') {
        throw 'The transaction staging name is not canonical.'
    }
    $stagingPath = Join-Path $script:DataPath $StagingName
    Remove-BoundSensitiveDatabaseArtifact `
        -Path $stagingPath `
        -NamePattern '^\.ConfigStore\.db\.install-(create|upgrade)-[0-9a-f]{32}\.tmp$' `
        -Description 'The bound transaction staging database' `
        -Scope 'staging-discard' `
        -ExpectedSha256 $ExpectedSha256
}

function Remove-BoundUpgradeBackup {
    param(
        [Parameter(Mandatory = $true)][string]$BackupName,
        [string]$ExpectedSha256 = ''
    )

    if ($BackupName -cnotmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$') {
        throw 'The bound upgrade backup name is not canonical.'
    }
    $backupPath = Join-Path $script:DataPath $BackupName
    $readbackId = $BackupName.Substring('.ConfigStore.db.install-upgrade-'.Length, 32)
    $readbackPath = Join-Path $script:DataPath ('.ConfigStore.db.install-readback-' + $readbackId + '.tmp')
    Remove-BoundSensitiveDatabaseArtifact `
        -Path $readbackPath `
        -NamePattern '^\.ConfigStore\.db\.install-readback-[0-9a-f]{32}\.tmp$' `
        -Description 'A retained protected-backup readback database' `
        -Scope 'readback-discard'
    if (-not (Test-Path -LiteralPath $backupPath)) {
        return
    }
    Assert-RegularFileInData $backupPath `
        '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' `
        'The bound protected upgrade backup'
    $boundSha256 = $ExpectedSha256
    if ([string]::IsNullOrWhiteSpace($boundSha256)) {
        $boundSha256 = Get-FileSha256 $backupPath
    }
    if ($boundSha256 -cnotmatch '^[0-9a-f]{64}$') {
        throw 'The protected upgrade backup has no valid exact hash binding for secure discard.'
    }
    Invoke-IdentityBoundSecureDiscard $backupPath $boundSha256 'backup-discard'
    if (Test-Path -LiteralPath $backupPath) {
        throw 'The bound protected upgrade backup survived exact-handle secure discard.'
    }
}

function Flush-TransactionDatabase {
    param([Parameter(Mandatory = $true)][string]$Path)

    $stream = [System.IO.File]::Open(
        $Path,
        [System.IO.FileMode]::Open,
        [System.IO.FileAccess]::ReadWrite,
        [System.IO.FileShare]::None
    )
    try {
        $stream.Flush($true)
    }
    finally {
        $stream.Dispose()
    }
}

function Invoke-VerifiedUpgradePublicationReconciliation {
    param([Parameter(Mandatory = $true)]$Record)

    if ($Record.Mode -cne 'UPGRADE_VERIFIED' -or
        $Record.StagingName -cnotmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' -or
        $Record.QuarantineName -cne ($Record.StagingName + '.install-original.quarantine')) {
        throw 'Upgrade publication reconciliation received a non-canonical verified record.'
    }
    $stagingPath = Join-Path $script:DataPath $Record.StagingName
    $quarantinePath = Join-Path $script:DataPath $Record.QuarantineName
    $backupPath = Join-Path $script:DataPath $Record.BackupName
    Assert-NoDatabaseSidecars $script:DatabasePath 'The upgrade reconciliation canonical database'
    Assert-NoDatabaseSidecars $stagingPath 'The upgrade reconciliation staging database'
    Assert-NoDatabaseSidecars $quarantinePath 'The upgrade reconciliation quarantine database'

    $unexpectedQuarantines = @(
        Get-ChildItem -LiteralPath $script:DataPath -Force -ErrorAction Stop |
            Where-Object {
                $_.Name -cmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-original\.quarantine$' -and
                $_.Name -cne $Record.QuarantineName
            }
    )
    if ($unexpectedQuarantines.Count -ne 0) {
        throw 'An unbound raw upgrade quarantine exists; automatic reconciliation is refused.'
    }

    # Rollback secure discard can be killed after staging disposition and while
    # the protected envelope is already durably empty (or just disposed).  With
    # exact OLD still canonical and no staging/quarantine name, no publication is
    # possible and an empty/missing envelope has nothing left to authenticate.
    # Return the explicit abort state so rollback can idempotently finish secure
    # disposal without trying to decrypt an intentional zero-length residue.
    $backupIsDiscarded = -not (Test-Path -LiteralPath $backupPath)
    if (-not $backupIsDiscarded -and
        (Test-Path -LiteralPath $backupPath -PathType Leaf) -and
        ((Get-Item -LiteralPath $backupPath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -eq 0 -and
        (Get-Item -LiteralPath $backupPath -Force).Length -eq 0) {
        $backupIsDiscarded = $true
    }
    if ($backupIsDiscarded -and
        -not (Test-Path -LiteralPath $stagingPath) -and
        -not (Test-Path -LiteralPath $quarantinePath) -and
        (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -and
        (Get-FileSha256 $script:DatabasePath) -ceq $Record.OriginalSha256) {
        return 'STAGING_SCRUBBED_ABORT_REQUIRED'
    }

    # Never discard the crash-retained OLD object until the protected backup is
    # independently readable and still bound to the durable record.
    Assert-RegularFileInData $backupPath `
        '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' `
        'The reconciliation protected upgrade backup'
    if ((Get-FileSha256 $backupPath) -cne $Record.BackupSha256) {
        throw 'The reconciliation protected upgrade backup no longer matches its bound hash.'
    }
    $comparisonPath = $script:DatabasePath
    if ((Test-Path -LiteralPath $quarantinePath -PathType Leaf) -and
        (Get-FileSha256 $quarantinePath) -ceq $Record.OriginalSha256) {
        $comparisonPath = $quarantinePath
    }
    $comparisonAvailable = (Test-Path -LiteralPath $comparisonPath -PathType Leaf) -and
        (Get-FileSha256 $comparisonPath) -ceq $Record.OriginalSha256
    Test-ProtectedBackupReadback `
        -BackupPath $backupPath `
        -ExpectedDatabaseSha256 $Record.OriginalSha256 `
        -ComparisonDatabasePath $comparisonPath `
        -AllowOriginalUnavailable:(-not $comparisonAvailable)
    if ((Get-FileSha256 $backupPath) -cne $Record.BackupSha256) {
        throw 'The reconciliation protected upgrade backup changed during read-back.'
    }

    return Invoke-IdentityBoundUpgradeReconciliation `
        -Source $stagingPath `
        -Destination $script:DatabasePath `
        -Quarantine $quarantinePath `
        -ExpectedSourceSha256 $Record.MigratedSha256 `
        -ExpectedDestinationSha256 $Record.OriginalSha256
}

function Assert-NoUnboundUpgradeQuarantines {
    param($Record)

    if (-not (Test-Path -LiteralPath $script:DataPath)) {
        if ($null -ne $Record) {
            throw 'A pending transaction cannot exist without its fixed Data directory.'
        }
        return
    }
    if (-not (Test-Path -LiteralPath $script:DataPath -PathType Container)) {
        throw 'The fixed Data path is not a directory during quarantine ownership validation.'
    }
    $boundName = ''
    if ($null -ne $Record -and $Record.Mode -ceq 'UPGRADE_VERIFIED') {
        $boundName = $Record.QuarantineName
        if ($boundName -cne ($Record.StagingName + '.install-original.quarantine')) {
            throw 'The verified transaction quarantine binding is not canonical.'
        }
    }
    $quarantines = @(
        Get-ChildItem -LiteralPath $script:DataPath -Force -ErrorAction Stop |
            Where-Object {
                $_.Name -cmatch '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-original\.quarantine$'
            }
    )
    foreach ($quarantine in $quarantines) {
        if ([string]::IsNullOrWhiteSpace($boundName) -or
            $quarantine.Name -cne $boundName) {
            throw 'An upgrade quarantine is not uniquely bound to the active verified transaction.'
        }
    }
}

function Invoke-PendingTransactionRollback {
    param([switch]$PreserveReconciledRecord)

    $record = Read-PendingTransactionRecord
    if ($null -eq $record) {
        throw 'No pending installer transaction exists; rollback cannot be proven.'
    }

    if ($record.Mode -ceq 'NOOP_ABSENT') {
        if (Test-Path -LiteralPath $script:DatabasePath) {
            throw 'The no-op transaction expected the database to remain absent.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The absent final database'
        Assert-NoopAbsentSourceInventory $record 'The absent no-op Data input inventory'
        if (-not $PreserveReconciledRecord) {
            Remove-PendingTransactionRecord
        }
        return 'PENDING_NOOP_CLEARED'
    }

    if ($record.Mode -ceq 'NOOP_CURRENT') {
        if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
            ((Get-Item -LiteralPath $script:DatabasePath -Force).Attributes -band
                [System.IO.FileAttributes]::ReparsePoint) -ne 0 -or
            (Get-FileSha256 $script:DatabasePath) -cne $record.CurrentSha256) {
            throw 'The no-op transaction database no longer matches its original hash.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The current final database'
        $currentVerification = Invoke-InstallerStateVerification $script:DatabasePath
        if ($currentVerification.ExitCode -ne 0) {
            throw 'The current no-op transaction failed source-aware rollback verification.'
        }
        if ((Get-VerifiedCredentialScrubState $currentVerification) -ceq 'pending') {
            Write-PublishedFinalizePending $record | Out-Null
            return 'PENDING_PUBLISHED_PRESERVED'
        }
        if (-not $PreserveReconciledRecord) {
            Remove-PendingTransactionRecord
        }
        return 'PENDING_NOOP_CLEARED'
    }

    if ($record.Mode -in @('UPGRADE_PREPARED', 'UPGRADE_VERIFIED')) {
        $upgradeReconciliationState = ''
        if ($record.Mode -ceq 'UPGRADE_VERIFIED') {
            $upgradeReconciliationState = Invoke-VerifiedUpgradePublicationReconciliation $record | Select-Object -Last 1
            if ($PreserveReconciledRecord -and
                $upgradeReconciliationState -ceq 'STAGING_SCRUBBED_ABORT_REQUIRED') {
                return 'PENDING_STAGING_SCRUBBED_ABORT_REQUIRED'
            }
        }
        if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
            ((Get-Item -LiteralPath $script:DatabasePath -Force).Attributes -band
                [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
            throw 'The final database is missing or unsafe during staged upgrade rollback.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The original final database'
        $finalSha = Get-FileSha256 $script:DatabasePath
        if ($record.Mode -ceq 'UPGRADE_VERIFIED' -and
            $finalSha -ceq $record.MigratedSha256 -and
            -not (Test-Path -LiteralPath (Join-Path $script:DataPath $record.StagingName))) {
            $publishedVerification = Invoke-InstallerStateVerification $script:DatabasePath
            if ($publishedVerification.ExitCode -ne 0) {
                throw 'The already-published upgrade failed current-schema verification.'
            }
            $publishedState = Get-VerifiedCredentialScrubState $publishedVerification
            Assert-NoDatabaseSidecars $script:DatabasePath 'The already-published upgrade database'
            if ($publishedState -ceq 'pending') {
                Write-PublishedFinalizePending $record | Out-Null
                return 'PENDING_PUBLISHED_PRESERVED'
            }
            if ($PreserveReconciledRecord) {
                Write-PendingNoopCurrent $record.MigratedSha256 -ReplaceExisting
            }
            else {
                Remove-PendingTransactionRecord
            }
            return 'PENDING_PUBLISHED_PRESERVED'
        }
        if ($finalSha -cne $record.OriginalSha256) {
            throw 'The final database is neither the bound original nor an exactly published upgrade.'
        }
        $stagingSha = if ($record.Mode -ceq 'UPGRADE_VERIFIED') { $record.MigratedSha256 } else { '' }
        $backupSha = if ($record.Mode -ceq 'UPGRADE_VERIFIED') { $record.BackupSha256 } else { '' }
        Remove-BoundTransactionStaging $record.StagingName $stagingSha
        Remove-BoundUpgradeBackup $record.BackupName $backupSha
        if ($PreserveReconciledRecord) {
            Write-PendingNoopCurrent $record.OriginalSha256 -ReplaceExisting
        }
        else {
            Remove-PendingTransactionRecord
        }
        return 'PENDING_UPGRADE_ROLLED_BACK'
    }

    if ($record.Mode -in @('CREATE_PREPARED', 'CREATE_READY', 'CREATE_VERIFIED')) {
        if ($record.Mode -ceq 'CREATE_VERIFIED' -and
            (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -and
            (Get-FileSha256 $script:DatabasePath) -ceq $record.CreatedSha256 -and
            -not (Test-Path -LiteralPath (Join-Path $script:DataPath $record.StagingName))) {
            Assert-CreateSourceInventory $record 'The already-published create source inventory during rollback'
            Assert-NoDatabaseSidecars $script:DatabasePath 'The already-published create database'
            $publishedVerification = Invoke-InstallerStateVerification $script:DatabasePath
            if ($publishedVerification.ExitCode -ne 0) {
                throw 'The already-published create failed current-schema verification.'
            }
            $publishedState = Get-VerifiedCredentialScrubState $publishedVerification
            Assert-NoDatabaseSidecars $script:DatabasePath 'The already-published create database'
            if ($publishedState -ceq 'pending') {
                Write-PublishedFinalizePending $record | Out-Null
                return 'PENDING_PUBLISHED_PRESERVED'
            }
            if ($PreserveReconciledRecord) {
                Write-PendingNoopCurrent $record.CreatedSha256 -ReplaceExisting
            }
            else {
                Remove-PendingTransactionRecord
            }
            return 'PENDING_PUBLISHED_PRESERVED'
        }
        if (Test-Path -LiteralPath $script:DatabasePath) {
            throw 'The final database appeared without matching an exactly published create.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The absent final database'
        $stagingSha = if ($record.Mode -in @('CREATE_READY', 'CREATE_VERIFIED')) {
            $record.CreatedSha256
        }
        else { '' }
        Remove-BoundTransactionStaging $record.StagingName $stagingSha
        if (-not $PreserveReconciledRecord) {
            Remove-PendingTransactionRecord
        }
        return 'PENDING_CREATE_REMOVED'
    }

    if ($record.Mode -ceq 'PUBLISHED_FINALIZE_PENDING') {
        $publishedState = Get-PublishedFinalizePendingState $record
        if ($publishedState -ceq 'complete') {
            if ($PreserveReconciledRecord) {
                Write-PendingNoopCurrent (Get-FileSha256 $script:DatabasePath) -ReplaceExisting
            }
            else {
                Remove-PendingTransactionRecord
            }
        }
        return 'PENDING_PUBLISHED_PRESERVED'
    }

    throw 'The pending installer transaction has an unsupported mode.'
}

function Commit-PendingTransaction {
    $record = Read-PendingTransactionRecord
    if ($null -eq $record) {
        throw 'No pending installer transaction exists; commit cannot be proven.'
    }
    if ($record.Mode -ceq 'PUBLISHED_FINALIZE_PENDING') {
        $publishedState = Get-PublishedFinalizePendingState $record
        if ($publishedState -ceq 'complete') {
            Remove-PendingTransactionRecord
            return 'PENDING_TRANSACTION_COMMITTED'
        }
        return 'PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED'
    }
    if ($record.Mode -ceq 'NOOP_ABSENT') {
        if (Test-Path -LiteralPath $script:DatabasePath) {
            throw 'The absent no-op transaction no longer matches the installed database state.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The absent final database'
        Assert-NoopAbsentSourceInventory $record 'The absent no-op Data input inventory at commit'
        Remove-PendingTransactionRecord
        return 'PENDING_TRANSACTION_COMMITTED'
    }
    if ($record.Mode -ceq 'NOOP_CURRENT') {
        if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
            (Get-FileSha256 $script:DatabasePath) -cne $record.CurrentSha256) {
            throw 'The current no-op transaction no longer matches the installed database.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The current final database'
        $verification = Invoke-InstallerStateVerification $script:DatabasePath
        if ($verification.ExitCode -ne 0) {
            throw 'The current no-op transaction failed current-schema verification.'
        }
        $currentState = Get-VerifiedCredentialScrubState $verification
        Assert-NoDatabaseSidecars $script:DatabasePath 'The verified current final database'
        if ($currentState -ceq 'pending') {
            Write-PublishedFinalizePending $record | Out-Null
            return 'PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED'
        }
        Remove-PendingTransactionRecord
        return 'PENDING_TRANSACTION_COMMITTED'
    }
    if ($record.Mode -notin @('UPGRADE_VERIFIED', 'CREATE_VERIFIED')) {
        throw 'A prepared transaction cannot be committed before verified advancement.'
    }
    if ($record.Mode -ceq 'UPGRADE_VERIFIED') {
        Invoke-VerifiedUpgradePublicationReconciliation $record | Out-Null
    }

    $stagingPath = Join-Path $script:DataPath $record.StagingName
    $expectedNewSha256 = if ($record.Mode -ceq 'UPGRADE_VERIFIED') {
        $record.MigratedSha256
    }
    else {
        $record.CreatedSha256
    }
    if ($record.Mode -ceq 'CREATE_VERIFIED') {
        Assert-CreateSourceInventory $record 'The staged create source inventory at commit'
    }
    Assert-NoDatabaseSidecars $script:DatabasePath 'The final database'
    Assert-NoDatabaseSidecars $stagingPath 'The verified staging database'

    $publishRequired = $false
    if ($record.Mode -ceq 'UPGRADE_VERIFIED') {
        if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf)) {
            throw 'The original final database disappeared before staged upgrade publication.'
        }
        $finalHash = Get-FileSha256 $script:DatabasePath
        if ($finalHash -ceq $record.OriginalSha256) {
            $publishRequired = $true
        }
        elseif ($finalHash -cne $expectedNewSha256 -or
            (Test-Path -LiteralPath $stagingPath)) {
            throw 'The final database is neither the bound original nor an idempotently published upgrade.'
        }
        $backupPath = Join-Path $script:DataPath $record.BackupName
        Assert-RegularFileInData $backupPath `
            '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' `
            'The verified protected upgrade backup'
        if ((Get-FileSha256 $backupPath) -cne $record.BackupSha256) {
            throw 'The verified protected upgrade backup no longer matches its bound hash.'
        }
        Test-ProtectedBackupReadback `
            -BackupPath $backupPath `
            -ExpectedDatabaseSha256 $record.OriginalSha256 `
            -AllowOriginalUnavailable:($finalHash -cne $record.OriginalSha256)
        if ((Get-FileSha256 $backupPath) -cne $record.BackupSha256) {
            throw 'The protected upgrade backup changed during read-back verification.'
        }
    }
    else {
        if (-not (Test-Path -LiteralPath $script:DatabasePath)) {
            $publishRequired = $true
        }
        elseif (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
            (Get-FileSha256 $script:DatabasePath) -cne $expectedNewSha256 -or
            (Test-Path -LiteralPath $stagingPath)) {
            throw 'The final database is neither absent nor an idempotently published create.'
        }
    }

    if ($publishRequired) {
        Assert-RegularFileInData $stagingPath `
            '^\.ConfigStore\.db\.install-(create|upgrade)-[0-9a-f]{32}\.tmp$' `
            'The verified staging database'
        if ((Get-FileSha256 $stagingPath) -cne $expectedNewSha256 -or
            -not (Test-SqliteHeader $stagingPath)) {
            throw 'The verified staging database no longer matches its transaction hash.'
        }
        $stagingVerification = Invoke-InstallerStateVerification $stagingPath
        if ($stagingVerification.ExitCode -ne 0) {
            throw 'The verified staging database failed current-schema verification.'
        }
        Get-VerifiedCredentialScrubState $stagingVerification | Out-Null
        Assert-NoDatabaseSidecars $stagingPath 'The verified staging database'
        Flush-TransactionDatabase $stagingPath
        if ($record.Mode -ceq 'UPGRADE_VERIFIED') {
            # Bind the original and staging identities together. The old exact
            # object is quarantined no-replace before the exact staging object
            # is published no-replace; the durable VERIFIED record reconciles
            # either crash boundary on the next installer run.
            Assert-NoDatabaseSidecars $script:DatabasePath 'The final upgrade database before publish'
            if ((Get-FileSha256 $script:DatabasePath) -cne $record.OriginalSha256) {
                throw 'The final upgrade database changed immediately before publication.'
            }
            Invoke-IdentityBoundUpgradePublish `
                -Source $stagingPath `
                -Destination $script:DatabasePath `
                -Quarantine (Join-Path $script:DataPath $record.QuarantineName) `
                -ExpectedSourceSha256 $expectedNewSha256 `
                -ExpectedDestinationSha256 $record.OriginalSha256
        }
        else {
            # Create must never overwrite a database that appeared after the
            # precheck; the identity-bound handle rename keeps ReplaceIfExists off.
            Assert-CreateSourceInventory $record 'The staged create source inventory immediately before publish'
            Invoke-IdentityBoundFilePublish `
                -Source $stagingPath `
                -Destination $script:DatabasePath `
                -ExpectedSha256 $expectedNewSha256
        }
    }

    if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
        (Get-FileSha256 $script:DatabasePath) -cne $expectedNewSha256 -or
        (Test-Path -LiteralPath $stagingPath) -or
        ($record.Mode -ceq 'UPGRADE_VERIFIED' -and
            (Test-Path -LiteralPath (Join-Path $script:DataPath $record.QuarantineName)))) {
        throw 'The staged database publication did not reach its exact committed state.'
    }
    Assert-NoDatabaseSidecars $script:DatabasePath 'The published final database'
    $verification = Invoke-InstallerStateVerification $script:DatabasePath
    if ($verification.ExitCode -ne 0) {
        throw 'The published final database failed current-schema verification.'
    }
    $publishedState = Get-VerifiedCredentialScrubState $verification
    Assert-NoDatabaseSidecars $script:DatabasePath 'The verified published final database'
    if ($record.Mode -ceq 'CREATE_VERIFIED') {
        Assert-CreateSourceInventory $record 'The published create source inventory'
    }
    if ($publishedState -ceq 'pending') {
        Write-PublishedFinalizePending $record | Out-Null
        return 'PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED'
    }
    Remove-PendingTransactionRecord
    return 'PENDING_TRANSACTION_COMMITTED'
}

function Invoke-InstallerStateVerification {
    param([string]$Path = $script:DatabasePath)
    return Invoke-ConfigMigrate @(
        '--verify-installer-state',
        '--source', $script:DataPath,
        '--db', $Path
    )
}

function Get-VerifiedCredentialScrubState {
    param([Parameter(Mandatory = $true)]$Verification)

    if ($Verification.ExitCode -ne 0) {
        throw 'Current-schema verification failed before credential scrub state could be read.'
    }
    $isPending = $Verification.StdOut.Contains('(scrub=pending)')
    $isComplete = $Verification.StdOut.Contains('(scrub=complete)')
    if ($isPending -eq $isComplete) {
        throw 'Current-schema verification returned an ambiguous credential scrub state.'
    }
    if ($isPending) {
        return 'pending'
    }
    return 'complete'
}

function Get-PublishedFinalizePendingState {
    param([Parameter(Mandatory = $true)]$Record)

    if ($Record.Mode -cne 'PUBLISHED_FINALIZE_PENDING') {
        throw 'Finalize verification requires a published finalize-pending transaction.'
    }
    if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
        ((Get-Item -LiteralPath $script:DatabasePath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'The finalize-pending database is missing or unsafe.'
    }
    Assert-NoDatabaseSidecars $script:DatabasePath 'The finalize-pending database'
    if ($Record.PublishedKind -in @('CREATE', 'UPGRADE')) {
        $stagingPath = Join-Path $script:DataPath $Record.StagingName
        if (Test-Path -LiteralPath $stagingPath) {
            throw 'A published finalize-pending transaction still has a staging database.'
        }
        Assert-NoDatabaseSidecars $stagingPath 'The absent published staging database'
    }
    if ($Record.PublishedKind -ceq 'UPGRADE') {
        $backupPath = Join-Path $script:DataPath $Record.BackupName
        Assert-RegularFileInData $backupPath `
            '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' `
            'The finalize-pending protected upgrade backup'
        if ((Get-FileSha256 $backupPath) -cne $Record.BackupSha256) {
            throw 'The finalize-pending protected upgrade backup changed.'
        }
    }

    $verification = Invoke-InstallerStateVerification $script:DatabasePath
    if ($verification.ExitCode -ne 0) {
        throw 'The finalize-pending database failed source-aware verification.'
    }
    $state = Get-VerifiedCredentialScrubState $verification
    if ($state -ceq 'pending') {
        if ((Get-FileSha256 $script:DatabasePath) -cne $Record.PendingSha256) {
            throw 'The finalize-pending database changed before credential cleanup.'
        }
        if ($Record.PublishedKind -ceq 'CREATE') {
            Assert-CreateSourceInventory $Record 'The finalize-pending create source inventory'
        }
    }
    Assert-NoDatabaseSidecars $script:DatabasePath 'The verified finalize-pending database'
    return $state
}

function Assert-ApplicationIsStopped {
    if ([string]::IsNullOrWhiteSpace($ApplicationExecutable)) {
        return
    }
    $expectedPath = [System.IO.Path]::GetFullPath($ApplicationExecutable)
    $processName = [System.IO.Path]::GetFileNameWithoutExtension($expectedPath)
    if ($processName -notmatch '^[A-Za-z0-9_.-]+$') {
        throw 'The application executable name is not safe for process inspection.'
    }
    foreach ($process in @(Get-Process -Name $processName -ErrorAction SilentlyContinue)) {
        $processPath = ''
        try {
            $processPath = $process.Path
        }
        catch {
            throw 'The installed application process path could not be inspected; migration is refused.'
        }
        if ([string]::IsNullOrWhiteSpace($processPath)) {
            throw 'An application process path could not be verified; migration is refused.'
        }
        if ([System.StringComparer]::OrdinalIgnoreCase.Equals(
                [System.IO.Path]::GetFullPath($processPath), $expectedPath)) {
            throw 'The installed application is still running; migration is refused.'
        }
    }
}

function Acquire-ApplicationInstanceLease {
    # Hold the exact machine-wide lease used by ApplicationInstanceGuard for the
    # whole helper process.  This closes the race in which the application could
    # restart after the process/path check but before migration completes.
    $scopeBytes = [System.Text.Encoding]::UTF8.GetBytes(
        'QtWidgetsApplication4/robot-hardware-control/v1'
    )
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    try {
        $scopeHash = [System.BitConverter]::ToString(
            $sha256.ComputeHash($scopeBytes)
        ).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $sha256.Dispose()
    }
    $mutexName = 'Global\NoTeaching-Robot-Hardware-Control-v1-' + $scopeHash
    $createdNew = $false
    try {
        $mutex = [System.Threading.Mutex]::new($false, $mutexName, [ref]$createdNew)
    }
    catch {
        throw 'The machine-wide application instance lease could not be opened; migration is refused.'
    }
    if (-not $createdNew) {
        $mutex.Dispose()
        throw 'Another application instance holds the machine-wide control lease; migration is refused.'
    }
    # Do not acquire thread-affine mutex ownership.  Like the C++ guard, the live
    # handle itself is the lease and Windows releases it if this process terminates.
    $script:ApplicationMutex = $mutex
}

function Assert-DatabaseExclusiveAccess {
    if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf)) {
        return
    }
    $stream = $null
    try {
        $stream = [System.IO.File]::Open(
            $script:DatabasePath,
            [System.IO.FileMode]::Open,
            [System.IO.FileAccess]::ReadWrite,
            [System.IO.FileShare]::None
        )
    }
    catch {
        throw 'The account database is in use or not writable; migration is refused.'
    }
    finally {
        if ($null -ne $stream) {
            $stream.Dispose()
        }
    }
}

function Assert-NoReparsePointInPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    $cursor = [System.IO.DirectoryInfo]::new([System.IO.Path]::GetFullPath($Path))
    while ($null -ne $cursor) {
        if ($cursor.Exists -and
            (($cursor.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0)) {
            throw ('Elevated installer migration refuses a reparse-point path component: ' + $cursor.FullName)
        }
        $cursor = $cursor.Parent
    }
}

try {
    $resolvedDataPath = [System.IO.Path]::GetFullPath($DataDirectory)
    $dataRoot = [System.IO.Path]::GetPathRoot($resolvedDataPath)
    if ([System.StringComparer]::OrdinalIgnoreCase.Equals($resolvedDataPath, $dataRoot)) {
        throw 'The elevated installer refuses to use a filesystem root as its Data directory.'
    }
    $script:DataPath = $resolvedDataPath.TrimEnd(
        [char[]]@([System.IO.Path]::DirectorySeparatorChar, [System.IO.Path]::AltDirectorySeparatorChar)
    )
    $script:MigratePath = [System.IO.Path]::GetFullPath($MigrateExecutable)
    $script:DatabasePath = Join-Path $script:DataPath 'ConfigStore.db'
    $script:TransactionPath = Join-Path $script:DataPath $script:TransactionFileName

    $requestedModeCount = 0
    foreach ($requestedMode in @(
            $FinalizeDeferredScrub,
            $RollbackPendingTransaction, $CommitPendingTransaction)) {
        if ($requestedMode) {
            ++$requestedModeCount
        }
    }
    if ($requestedModeCount -gt 1) {
        $script:OperationMode = 'invalid'
        throw 'Installer migration operation switches are mutually exclusive.'
    }
    if ($RollbackPendingTransaction) {
        $script:OperationMode = 'rollback'
    }
    elseif ($FinalizeDeferredScrub) {
        $script:OperationMode = 'finalize'
    }
    elseif ($CommitPendingTransaction) {
        $script:OperationMode = 'commit'
    }

    if (-not (Test-Path -LiteralPath $script:MigratePath -PathType Leaf)) {
        throw 'The installer migration executable is missing.'
    }
    if (((Get-Item -LiteralPath $script:MigratePath -Force).Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'The installer migration executable is a reparse point.'
    }
    Assert-NoReparsePointInPath $script:DataPath
    Assert-NoReparsePointInPath (Split-Path -Parent $script:MigratePath)
    Assert-ApplicationIsStopped
    if (-not $InstallerHoldsInstanceLease) {
        Acquire-ApplicationInstanceLease
    }
    $entryTransactionRecord = Read-PendingTransactionRecord
    Assert-NoUnboundUpgradeQuarantines $entryTransactionRecord
    Assert-DatabaseExclusiveAccess
    Assert-NoDatabaseSidecars $script:DatabasePath 'The final database'
    if ((Test-Path -LiteralPath $script:DatabasePath) -and
        (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
         ((Get-Item -LiteralPath $script:DatabasePath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0)) {
        throw 'The final database is not a regular non-reparse file.'
    }

    if ($RollbackPendingTransaction) {
        $rollbackResult = Invoke-PendingTransactionRollback
        Write-InstallerStatus ('OK:' + $rollbackResult)
        exit 0
    }

    if ($CommitPendingTransaction) {
        $commitResult = Commit-PendingTransaction
        Write-InstallerStatus ('OK:' + $commitResult)
        exit 0
    }

    if ($FinalizeDeferredScrub) {
        $finalizeRecord = Read-PendingTransactionRecord
        if ($null -eq $finalizeRecord -or
            $finalizeRecord.Mode -cne 'PUBLISHED_FINALIZE_PENDING') {
            throw 'Deferred credential cleanup requires a published finalize-pending transaction.'
        }
        $beforeFinalizeState = Get-PublishedFinalizePendingState $finalizeRecord
        if ($beforeFinalizeState -ceq 'pending') {
            $finalize = Invoke-ConfigMigrate @(
                '--source', $script:DataPath,
                '--db', $script:DatabasePath,
                '--encrypt',
                '--scrub-legacy-credentials'
            )
            if ($finalize.ExitCode -ne 0) {
                Write-InstallerStatus (
                    ('ERROR:DEFERRED_SCRUB_FAILED:{0}' -f $finalize.ExitCode) +
                    "`r`nDETAIL=" + (Get-SafeMigrationFailureDetail $finalize)
                )
                exit 26
            }
        }
        $finalVerification = Invoke-InstallerStateVerification
        if ($finalVerification.ExitCode -ne 0) {
            Write-InstallerStatus (
                ('ERROR:DEFERRED_SCRUB_READBACK_FAILED:{0}' -f $finalVerification.ExitCode) +
                "`r`nDETAIL=" + (Get-SafeMigrationFailureDetail $finalVerification)
            )
            exit 27
        }
        if ((Get-VerifiedCredentialScrubState $finalVerification) -cne 'complete') {
            throw 'Deferred credential cleanup did not reach scrub=complete.'
        }
        Assert-NoDatabaseSidecars $script:DatabasePath 'The finalized database'
        Remove-PendingTransactionRecord
        Write-InstallerStatus 'OK:DEFERRED_SCRUB_FINALIZED_AND_VERIFIED'
        exit 0
    }

    $pendingRecord = Read-PendingTransactionRecord
    if ($null -ne $pendingRecord) {
        if ($pendingRecord.Mode -ceq 'PUBLISHED_FINALIZE_PENDING') {
            $publishedFinalizeState = Get-PublishedFinalizePendingState $pendingRecord
            if ($publishedFinalizeState -ceq 'pending') {
                Write-InstallerStatus "OK:CURRENT_AND_VERIFIED`r`nFINALIZE=1"
            }
            else {
                Write-InstallerStatus 'OK:CURRENT_AND_VERIFIED'
            }
            exit 0
        }
        if ($pendingRecord.Mode -ceq 'NOOP_ABSENT') {
            if (Test-Path -LiteralPath $script:DatabasePath) {
                throw 'A pending absent no-op transaction no longer matches the database state.'
            }
            Assert-NoopAbsentSourceInventory $pendingRecord 'The resumed absent no-op Data input inventory'
            Write-InstallerStatus 'OK:NO_DATABASE'
            exit 0
        }
        if ($pendingRecord.Mode -ceq 'NOOP_CURRENT') {
            if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
                (Get-FileSha256 $script:DatabasePath) -cne $pendingRecord.CurrentSha256) {
                throw 'A pending current no-op transaction no longer matches the database state.'
            }
            $pendingVerification = Invoke-InstallerStateVerification $script:DatabasePath
            if ($pendingVerification.ExitCode -ne 0) {
                throw 'A pending current no-op transaction failed current-schema verification.'
            }
            $pendingCurrentScrubState = Get-VerifiedCredentialScrubState $pendingVerification
            Assert-NoDatabaseSidecars $script:DatabasePath 'The verified current final database'
            if ($pendingCurrentScrubState -ceq 'pending') {
                Write-InstallerStatus "OK:CURRENT_AND_VERIFIED`r`nFINALIZE=1"
            }
            else {
                Write-InstallerStatus 'OK:CURRENT_AND_VERIFIED'
            }
            exit 0
        }
        if ($pendingRecord.Mode -ceq 'UPGRADE_VERIFIED') {
            Invoke-VerifiedUpgradePublicationReconciliation $pendingRecord | Out-Null
            $pendingStaging = Join-Path $script:DataPath $pendingRecord.StagingName
            $pendingBackup = Join-Path $script:DataPath $pendingRecord.BackupName
            $pendingFinalHash = Get-FileSha256 $script:DatabasePath
            if ($pendingFinalHash -ceq $pendingRecord.OriginalSha256) {
                Assert-NoDatabaseSidecars $pendingStaging 'The verified upgrade staging database'
                Assert-RegularFileInData $pendingStaging `
                    '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' `
                    'The verified upgrade staging database'
                if ((Get-FileSha256 $pendingStaging) -cne $pendingRecord.MigratedSha256) {
                    throw 'A verified upgrade staging database no longer matches its bound hash.'
                }
                $pendingVerificationPath = $pendingStaging
            }
            elseif ($pendingFinalHash -ceq $pendingRecord.MigratedSha256 -and
                -not (Test-Path -LiteralPath $pendingStaging)) {
                # Previous ssPostInstall published atomically and then died before
                # deleting the VERIFIED record.  Preserve it until this run reaches
                # the ordinary idempotent commit point after installing files.
                $pendingVerificationPath = $script:DatabasePath
            }
            else {
                throw 'A verified upgrade is neither staged nor exactly published.'
            }
            Assert-RegularFileInData $pendingBackup `
                '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' `
                'The verified protected upgrade backup'
            $pendingVerification = Invoke-InstallerStateVerification $pendingVerificationPath
            if ((Get-FileSha256 $pendingBackup) -cne $pendingRecord.BackupSha256 -or
                $pendingVerification.ExitCode -ne 0) {
                throw 'A verified staged upgrade failed hash or schema read-back.'
            }
            $pendingScrubState = Get-VerifiedCredentialScrubState $pendingVerification
            Assert-NoDatabaseSidecars $pendingVerificationPath 'The verified or published upgrade database'
            Test-ProtectedBackupReadback `
                -BackupPath $pendingBackup `
                -ExpectedDatabaseSha256 $pendingRecord.OriginalSha256 `
                -AllowOriginalUnavailable:($pendingFinalHash -cne $pendingRecord.OriginalSha256)
            if ((Get-FileSha256 $pendingBackup) -cne $pendingRecord.BackupSha256) {
                throw 'The protected upgrade backup changed during resume verification.'
            }
            # The backup read-back can be slow enough for another process to
            # change the final/staging state after the first verification.  Bind
            # the status result to the same exact published-or-staged state one
            # last time before returning OK:MIGRATED_AND_VERIFIED.
            $pendingFinalHashAfterReadback = Get-FileSha256 $script:DatabasePath
            if ($pendingFinalHash -ceq $pendingRecord.OriginalSha256) {
                Assert-NoDatabaseSidecars $pendingStaging 'The rechecked upgrade staging database'
                if ($pendingFinalHashAfterReadback -cne $pendingRecord.OriginalSha256 -or
                    -not (Test-Path -LiteralPath $pendingStaging -PathType Leaf) -or
                    (Get-FileSha256 $pendingStaging) -cne $pendingRecord.MigratedSha256) {
                    throw 'The verified staged upgrade changed during protected-backup read-back.'
                }
            }
            else {
                Assert-NoDatabaseSidecars $script:DatabasePath 'The rechecked published upgrade database'
                if ($pendingFinalHashAfterReadback -cne $pendingRecord.MigratedSha256 -or
                    (Test-Path -LiteralPath $pendingStaging)) {
                    throw 'The published upgrade changed during protected-backup read-back.'
                }
            }
            if ((Get-FileSha256 $pendingVerificationPath) -cne $pendingRecord.MigratedSha256) {
                throw 'The verified upgrade database changed before status publication.'
            }
            $pendingUpgradeStatus =
                "OK:MIGRATED_AND_VERIFIED`r`nBACKUP_NAME=" +
                $pendingRecord.BackupName + "`r`nBACKUP_SHA256=" +
                $pendingRecord.BackupSha256
            if ($pendingScrubState -ceq 'pending') {
                $pendingUpgradeStatus += "`r`nFINALIZE=1"
            }
            Write-InstallerStatus $pendingUpgradeStatus
            exit 0
        }
        if ($pendingRecord.Mode -ceq 'CREATE_VERIFIED') {
            Assert-CreateSourceInventory $pendingRecord 'The resumed verified create source inventory'
            $pendingStaging = Join-Path $script:DataPath $pendingRecord.StagingName
            if (-not (Test-Path -LiteralPath $script:DatabasePath)) {
                Assert-NoDatabaseSidecars $pendingStaging 'The verified create staging database'
                Assert-RegularFileInData $pendingStaging `
                    '^\.ConfigStore\.db\.install-create-[0-9a-f]{32}\.tmp$' `
                    'The verified create staging database'
                if ((Get-FileSha256 $pendingStaging) -cne $pendingRecord.CreatedSha256) {
                    throw 'A verified create staging database no longer matches its bound hash.'
                }
                $pendingVerificationPath = $pendingStaging
            }
            elseif ((Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -and
                (Get-FileSha256 $script:DatabasePath) -ceq $pendingRecord.CreatedSha256 -and
                -not (Test-Path -LiteralPath $pendingStaging)) {
                $pendingVerificationPath = $script:DatabasePath
            }
            else {
                throw 'A verified create is neither staged nor exactly published.'
            }
            $pendingVerification = Invoke-InstallerStateVerification $pendingVerificationPath
            if ($pendingVerification.ExitCode -ne 0) {
                throw 'A verified staged create failed hash or schema read-back.'
            }
            $pendingScrubState = Get-VerifiedCredentialScrubState $pendingVerification
            Assert-NoDatabaseSidecars $pendingVerificationPath 'The verified or published create database'
            Assert-CreateSourceInventory $pendingRecord 'The verified resumed create source inventory'
            if ($pendingScrubState -ceq 'pending') {
                Write-InstallerStatus (
                    "OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED`r`n" +
                    "DATABASE_CREATED=1`r`nCREATED_SHA256=" +
                    $pendingRecord.CreatedSha256 + "`r`nFINALIZE=1"
                )
            }
            else {
                Write-InstallerStatus (
                    "OK:LEGACY_DATABASE_CREATED_AND_VERIFIED`r`n" +
                    "DATABASE_CREATED=1`r`nCREATED_SHA256=" +
                    $pendingRecord.CreatedSha256
                )
            }
            exit 0
        }
        throw 'An incomplete prepared installer transaction must be rolled back before migration can resume.'
    }

    if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf)) {
        if (-not (Test-Path -LiteralPath $script:DataPath -PathType Container) -or
            $null -eq (Get-ChildItem -LiteralPath $script:DataPath -Force -ErrorAction Stop | Select-Object -First 1)) {
            $absentInventorySha256 = Get-DataInputInventorySha256 -ExcludedPaths @(
                $script:TransactionPath, $script:DatabasePath
            )
            Write-PendingNoopAbsent $absentInventorySha256
            $absentRecord = Read-PendingTransactionRecord
            Assert-NoopAbsentSourceInventory $absentRecord 'The prepared absent no-op Data input inventory'
            Write-InstallerStatus 'OK:NO_DATABASE'
            exit 0
        }

        $createId = [Guid]::NewGuid().ToString('N')
        $createStagingName = '.ConfigStore.db.install-create-' + $createId + '.tmp'
        $script:CreateStagingPath = Join-Path $script:DataPath $createStagingName
        if (Test-Path -LiteralPath $script:CreateStagingPath) {
            throw 'The randomly selected create staging path already exists.'
        }
        Assert-NoDatabaseSidecars $script:CreateStagingPath 'The create staging database'
        $createSourceInventorySha256 = Get-DataInputInventorySha256 -ExcludedPaths @(
            $script:TransactionPath, $script:DatabasePath, $script:CreateStagingPath
        )
        Write-PendingCreatePrepared $createStagingName $createSourceInventorySha256
        $preparedCreate = Read-PendingTransactionRecord
        Assert-CreateSourceInventory $preparedCreate 'The prepared create source inventory before migration'
        $create = Invoke-ConfigMigrate @(
            '--source', $script:DataPath,
            '--db', $script:CreateStagingPath,
            '--encrypt',
            '--defer-credential-scrub',
            '--installer-staging'
        )
        if ($create.ExitCode -ne 0) {
            Write-InstallerStatus (
                ('ERROR:LEGACY_DATABASE_CREATE_FAILED:{0}' -f $create.ExitCode) +
                "`r`nDETAIL=" + (Get-SafeMigrationFailureDetail $create)
            )
            exit 20
        }
        Assert-RegularFileInData $script:CreateStagingPath `
            '^\.ConfigStore\.db\.install-create-[0-9a-f]{32}\.tmp$' `
            'The created staging database'
        Assert-NoDatabaseSidecars $script:CreateStagingPath 'The created staging database'
        if (-not (Test-SqliteHeader $script:CreateStagingPath)) {
            throw 'Legacy migration did not create a valid staging SQLite database.'
        }
        $createdVerification = Invoke-InstallerStateVerification $script:CreateStagingPath
        if ($createdVerification.ExitCode -ne 0) {
            Write-InstallerStatus (
                ('ERROR:LEGACY_DATABASE_READBACK_FAILED:{0}' -f $createdVerification.ExitCode) +
                "`r`nDETAIL=" + (Get-SafeMigrationFailureDetail $createdVerification)
            )
            exit 28
        }
        $createdScrubState = Get-VerifiedCredentialScrubState $createdVerification
        Assert-NoDatabaseSidecars $script:CreateStagingPath 'The verified create staging database'
        $preparedCreate = Read-PendingTransactionRecord
        Assert-CreateSourceInventory $preparedCreate 'The staged create source inventory after migration'
        $createdHash = Get-FileSha256 $script:CreateStagingPath
        Advance-PendingCreateReady $createdHash
        Advance-PendingCreateVerified
        if (Test-Path -LiteralPath $script:DatabasePath) {
            throw 'The final database changed before staged create publication.'
        }
        if ($createdScrubState -ceq 'pending') {
            Write-InstallerStatus (
                "OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED`r`n" +
                "DATABASE_CREATED=1`r`nCREATED_SHA256=" +
                $createdHash + "`r`nFINALIZE=1"
            )
        }
        else {
            Write-InstallerStatus (
                "OK:LEGACY_DATABASE_CREATED_AND_VERIFIED`r`n" +
                "DATABASE_CREATED=1`r`nCREATED_SHA256=" + $createdHash
            )
        }
        exit 0
    }
    if (((Get-Item -LiteralPath $script:DatabasePath -Force).Attributes -band
            [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'The account database is a reparse point; unattended migration is refused.'
    }

    $currentVerification = Invoke-InstallerStateVerification $script:DatabasePath
    Assert-NoDatabaseSidecars $script:DatabasePath 'The database after schema verification'
    if ($currentVerification.ExitCode -eq 0) {
        $currentScrubState = Get-VerifiedCredentialScrubState $currentVerification
        $currentHash = Get-FileSha256 $script:DatabasePath
        Write-PendingNoopCurrent $currentHash
        if ($currentScrubState -ceq 'pending') {
            Write-InstallerStatus "OK:CURRENT_AND_VERIFIED`r`nFINALIZE=1"
        }
        else {
            Write-InstallerStatus 'OK:CURRENT_AND_VERIFIED'
        }
        exit 0
    }

    $databaseHashBefore = Get-FileSha256 $script:DatabasePath
    $upgradeId = [Guid]::NewGuid().ToString('N')
    $upgradeStagingName = '.ConfigStore.db.install-upgrade-' + $upgradeId + '.tmp'
    $upgradeBackupName = $upgradeStagingName + '.install-backup.dpapi.bak'
    $script:UpgradeStagingPath = Join-Path $script:DataPath $upgradeStagingName
    $upgradeBackupPath = Join-Path $script:DataPath $upgradeBackupName
    if ((Test-Path -LiteralPath $script:UpgradeStagingPath) -or
        (Test-Path -LiteralPath $upgradeBackupPath)) {
        throw 'A randomly selected upgrade staging or backup path already exists.'
    }
    Assert-NoDatabaseSidecars $script:UpgradeStagingPath 'The upgrade staging database'
    Write-PendingUpgradePrepared $upgradeStagingName $upgradeBackupName $databaseHashBefore

    [System.IO.File]::Copy($script:DatabasePath, $script:UpgradeStagingPath, $false)
    Flush-TransactionDatabase $script:UpgradeStagingPath
    if ((Get-FileSha256 $script:UpgradeStagingPath) -cne $databaseHashBefore) {
        throw 'The upgrade staging copy does not match the untouched final database.'
    }
    $script:MigrationArguments = @(
        '--source', $script:DataPath,
        '--db', $script:UpgradeStagingPath,
        '--encrypt',
        '--defer-credential-scrub',
        '--installer-staging',
        '--upgrade-backup', $upgradeBackupPath
    )
    $migration = Invoke-ConfigMigrate $script:MigrationArguments
    if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
        (Get-FileSha256 $script:DatabasePath) -cne $databaseHashBefore) {
        throw 'The final database changed during staging-only migration.'
    }
    if ($migration.ExitCode -ne 0) {
        Write-InstallerStatus (
            ('ERROR:MIGRATION_FAILED_FINAL_UNCHANGED:{0}' -f $migration.ExitCode) +
            "`r`nDETAIL=" + (Get-SafeMigrationFailureDetail $migration)
        )
        exit 21
    }
    if (-not $migration.StdOut.Contains('Upgraded existing database to schema v5:')) {
        throw 'The staging migrator did not report the required existing-database schema upgrade.'
    }
    Assert-RegularFileInData $script:UpgradeStagingPath `
        '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp$' `
        'The upgraded staging database'
    Assert-NoDatabaseSidecars $script:UpgradeStagingPath 'The upgraded staging database'
    Assert-RegularFileInData $upgradeBackupPath `
        '^\.ConfigStore\.db\.install-upgrade-[0-9a-f]{32}\.tmp\.install-backup\.dpapi\.bak$' `
        'The installer-bound protected upgrade backup'
    $backupSha256 = Get-FileSha256 $upgradeBackupPath
    Test-ProtectedBackupReadback $upgradeBackupPath $databaseHashBefore
    if ((Get-FileSha256 $upgradeBackupPath) -cne $backupSha256) {
        throw 'The protected upgrade backup changed during read-back verification.'
    }
    $verification = Invoke-InstallerStateVerification $script:UpgradeStagingPath
    if ($verification.ExitCode -ne 0) {
        Write-InstallerStatus (
            ('ERROR:STAGING_READBACK_FAILED:{0}' -f $verification.ExitCode) +
            "`r`nDETAIL=" + (Get-SafeMigrationFailureDetail $verification)
        )
        exit 24
    }
    $upgradeScrubState = Get-VerifiedCredentialScrubState $verification
    Assert-NoDatabaseSidecars $script:UpgradeStagingPath 'The verified upgrade staging database'

    $migratedSha256 = Get-FileSha256 $script:UpgradeStagingPath
    Assert-NoDatabaseSidecars $script:DatabasePath 'The final database before prepared-status publication'
    if (-not (Test-Path -LiteralPath $script:DatabasePath -PathType Leaf) -or
        (Get-FileSha256 $script:DatabasePath) -cne $databaseHashBefore -or
        -not (Test-Path -LiteralPath $script:UpgradeStagingPath -PathType Leaf) -or
        (Get-FileSha256 $script:UpgradeStagingPath) -cne $migratedSha256 -or
        (Get-FileSha256 $upgradeBackupPath) -cne $backupSha256) {
        throw 'The prepared upgrade changed before its verified state was persisted.'
    }
    Advance-PendingUpgradeVerified $backupSha256 $migratedSha256
    $upgradeStatus =
        "OK:MIGRATED_AND_VERIFIED`r`nBACKUP_NAME=" +
        $upgradeBackupName + "`r`nBACKUP_SHA256=" +
        $backupSha256
    if ($upgradeScrubState -ceq 'pending') {
        $upgradeStatus += "`r`nFINALIZE=1"
    }
    Write-InstallerStatus $upgradeStatus
    exit 0
}
catch {
    $safeType = $_.Exception.GetType().Name -replace '[^A-Za-z0-9_.-]', '_'
    $compensation = 'NOT_REQUIRED'
    if ($script:OperationMode -eq 'preinstall') {
        try {
            if (-not [string]::IsNullOrWhiteSpace($script:TransactionPath) -and
                (Test-Path -LiteralPath $script:TransactionPath -PathType Leaf)) {
                $compensation = Invoke-PendingTransactionRollback -PreserveReconciledRecord
            }
        }
        catch {
            $compensation = 'ROLLBACK_INCOMPLETE'
        }
    }
    if (-not $script:StatusWritten) {
        try {
            Write-InstallerStatus ('ERROR:INSTALL_MIGRATION_EXCEPTION:{0}:{1}' -f $safeType, $compensation)
        }
        catch {
            # The installer will also reject a missing status file and report the
            # non-zero process exit code.  Never mask the original failure here.
        }
    }
    exit 30
}
