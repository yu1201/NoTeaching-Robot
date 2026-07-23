#!/usr/bin/env python3
"""Static fail-closed wiring checks for scan-data queueing and archive upload."""

from __future__ import annotations

from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    source = (REPO_ROOT / "src" / "ScanDataUploader.cpp").read_text(
        encoding="utf-8", errors="replace"
    )
    header = (REPO_ROOT / "include" / "ScanDataUploader.h").read_text(
        encoding="utf-8", errors="replace"
    )
    config = (REPO_ROOT / "include" / "OnlineServicesConfig.h").read_text(
        encoding="utf-8", errors="replace"
    )

    expected_limits = {
        "kMaximumPendingItems": "512",
        "kMaximumItemsPerWorkerRun": "8",
        "kMaximumArchiveFileCount": "4096",
        "kMaximumTempEntriesScanned": "2048",
        "kMaximumStaleArchivesDeletedPerPass": "128",
        "kMaximumSingleFileBytes": "768LL * 1024 * 1024",
        "kMaximumUncompressedBytes": "2LL * 1024 * 1024 * 1024",
        "kMaximumZipBytes": "2LL * 1024 * 1024 * 1024",
    }
    for name, expression in expected_limits.items():
        require(
            re.search(
                rf"constexpr\s+(?:int|qint64)\s+{name}\s*=\s*{re.escape(expression)}\s*;",
                source,
            )
            is not None,
            f"missing or changed hard limit: {name}",
        )

    for fragment in (
        "CollectArchiveFiles(caseDir, &beforeFiles",
        "CollectArchiveFiles(caseDir, &afterFiles",
        "HashFileExact(beforeFiles[index]",
        "afterDigest != sourceDigests[index]",
        "SameArchiveSnapshot(beforeFiles, afterFiles)",
        "std::filesystem::recursive_directory_iterator",
        "entry.symlink_status(ec)",
        "zipBytes <= 0 || zipBytes > kMaximumZipBytes",
        "QIODevice::WriteOnly | QIODevice::NewOnly",
        "QUuid::createUuid()",
        "m_pending.mid(0, kMaximumItemsPerWorkerRun)",
        "m_pending.size() >= kMaximumPendingItems",
        "doc.array().size() > kMaximumPendingItems",
        "m_pendingStoreBlocked",
        "CleanupStaleUploadArchives",
        "ScanDataUploadPolicy::IsOwnedTempArchiveName",
        "ScanDataUploadPolicy::ShouldDeleteTempArchive",
        "HasArchiveStorageHeadroom(zipPath, beforeFiles",
        "QStorageInfo storage",
        "kMinimumFreeDiskReserveBytes",
        "ArchiveCancelRequested(cancel)",
        "HashFileExact(snapshot, &digest, &readError, cancel)",
        "System32/tar.exe",
        "GetSystemWindowsDirectoryW",
        "FILE_ATTRIBUTE_REPARSE_POINT",
        'QStringLiteral("--format") << QStringLiteral("zip")',
        'QStringLiteral("--options") << QStringLiteral("hdrcharset=UTF-8")',
        'QStringLiteral("--null") << QStringLiteral("--no-recursion")',
        "archiveProcess.waitForFinished(kArchiveProcessPollMs)",
        "archiveProcess.kill()",
        "QZipReader archiveReader(zipPath)",
        "expectedEntries.value(normalizedPath) != entry.size",
        "seenEntries.contains(normalizedPath)",
        "drainProcessOutput()",
        "archiveProcess.readAllStandardError()",
        "4096 - boundedArchiveError.size()",
        "ScanDataUploadPolicy::IsOwnedTempArchiveListName",
        "ScanDataUploadPolicy::ShouldDeleteTempArchiveList",
    ):
        require(fragment in source or fragment in header, f"missing fail-closed wiring: {fragment}")

    require(
        "if (f.open(QIODevice::ReadOnly))" not in source,
        "archive still silently skips files that fail to open",
    )
    require(
        'WritableChildPath(QStringLiteral("Temp/OnlineUpload"), zipName)' not in source,
        "local archive name is still deterministic and collision-prone",
    )
    require(
        source.find("CollectArchiveFiles(caseDir, &beforeFiles")
        < source.find("archiveProcess.start()"),
        "archive is created before traversal succeeds",
    )
    require(
        source.find("QIODevice::WriteOnly | QIODevice::NewOnly")
        < source.find("archiveProcess.start()"),
        "archive list is not exclusively created before compression starts",
    )
    require(
        source.find("HasArchiveStorageHeadroom(zipPath, beforeFiles")
        < source.find("QIODevice::WriteOnly | QIODevice::NewOnly"),
        "free-space gate does not run before exclusive archive creation",
    )
    require(
        "TempArchiveTtlMs = 24LL * 60 * 60 * 1000" in header,
        "stale upload archive cleanup does not have a 24-hour TTL",
    )
    require(
        "QFile::remove(zipPath)" not in source[
            source.find("if (!CollectArchiveFiles(caseDir, &beforeFiles") :
            source.find("const QString listPath = zipPath")
        ],
        "preflight failure can delete a path that this worker never created",
    )
    cancel_branch = source[
        source.find("if (ArchiveCancelRequested(cancel))", source.find("while (archiveProcess.state()")) :
        source.find("const QByteArray archiveError")
    ]
    require(
        cancel_branch.find("archiveProcess.kill()")
        < cancel_branch.find("archiveProcess.waitForFinished")
        < cancel_branch.find("QFile::remove(zipPath)"),
        "cancel path can delete the archive before the compressor is confirmed stopped",
    )

    for fragment in (
        "FullAccessAccount",
        "FtpAccessAccount",
        "UploadOnlyAccount",
        "IsDefaultFtpAccount",
        "AccessLevelForAccount",
    ):
        require(fragment in config, f"fixed online-services role mapping is incomplete: {fragment}")
    require(
        source.count("OnlineServicesConfig::IsDefaultFtpAccount") >= 2
        and "AppPaths::IsSafePathComponent(config.deviceName)" in source,
        "default-account and safe device-directory gates are not enforced before and inside the worker",
    )

    print("PASS: scan uploader has bounded limits and fixed-account/device-directory gates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
