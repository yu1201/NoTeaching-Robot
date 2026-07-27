#!/usr/bin/env python3
"""Static integration gates for owned remote FTP workers and bounded ZIP extraction."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"FAIL: {message}")


dialog_h = (ROOT / "include" / "OnlineServicesDialog.h").read_text(encoding="utf-8")
dialog_cpp = (ROOT / "src" / "OnlineServicesDialog.cpp").read_text(encoding="utf-8")
main_cpp = (ROOT / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")
ftp_h = (ROOT / "include" / "FTPClient.h").read_text(encoding="utf-8")
ftp_cpp = (ROOT / "src" / "FTPClient.cpp").read_text(encoding="utf-8")
archive_h = (ROOT / "include" / "RemoteArchiveSecurity.h").read_text(encoding="utf-8")
archive_cpp = (ROOT / "src" / "RemoteArchiveSecurity.cpp").read_text(encoding="utf-8")
lifecycle_h = (ROOT / "include" / "RemoteWorkerLifecycle.h").read_text(encoding="utf-8")

require("~OnlineServicesDialog() override" in dialog_h, "dialog destructor owns worker shutdown")
require("std::thread m_remoteWorker" in dialog_h, "remote worker is a member")
require("RemoteWorkerLifecycle m_remoteLifecycle" in dialog_h,
        "remote cancellation/generation/permanent shutdown share one lifecycle")
require("std::atomic_bool m_cancel" in lifecycle_h
        and "std::atomic<quint64> m_generation" in lifecycle_h,
        "remote lifecycle has atomic cancellation and callback generations")
require("m_permanentShutdown" in lifecycle_h and "m_cancelCycle" in lifecycle_h,
        "normal close suppression is distinct from permanent application shutdown")
require("IsRemoteOperationBusy() const noexcept" in dialog_h, "main-window busy gate API")
require("CancelRemoteOperationAndWait(bool permanentShutdown = false)" in dialog_h,
        "explicit reversible/permanent cancel+join API")
require(dialog_cpp.count("StartRemoteWorker([this") == 6, "all six remote FTP entries use owned worker")
require(".detach()" not in dialog_cpp, "OnlineServicesDialog has no detached threads")

remote_region = dialog_cpp[dialog_cpp.index("single owned/cancellable/joinable FTP worker"):]
require("invokeMethod(qApp" not in remote_region, "remote worker never posts to qApp")
require(remote_region.count("QMetaObject::invokeMethod(this") >= 5,
        "remote callbacks target the owned dialog receiver")
require("m_remoteLifecycle.BeginCancel" in remote_region
        and "m_remoteLifecycle.IsCancelled" in remote_region,
        "cancel/destruction invalidates already queued callbacks")
require("m_remoteWorker.join()" in dialog_cpp[dialog_cpp.index("OnlineServicesDialog::~OnlineServicesDialog"):],
        "destructor joins worker without event loop")
require("onlinePage->IsRemoteOperationBusy()" in main_cpp
        and "onlinePage->CancelRemoteOperationAndWait(true)" in main_cpp,
        "main closeEvent gates and joins remote work")
require("CancelRemoteOperationAndWait();" in dialog_cpp
        and "m_remoteLifecycle.FinishCancel();" in remote_region,
        "ordinary page close cancels but re-enables the cached dialog")

require("downloadFileBounded" in ftp_h and "FtpOpenFileA" in ftp_cpp,
        "remote archive download is chunked/open-handle based")
bounded = ftp_cpp[ftp_cpp.index("bool FtpClient::downloadFileBounded("):
                  ftp_cpp.index("// 真实文件删除逻辑")]
for token in (
    "FtpGetFileSize",
    "openedBytes != expectedRemoteBytes",
    "InternetReadFile",
    "cancelFlag",
    "CREATE_NEW",
    "maximumBytes",
    "DeleteFileA(localFilePath.c_str())",
    "progressCb(received, expectedRemoteBytes)",
):
    require(token in bounded, f"bounded FTP download missing {token}")
require("size_t maximumEntries = 10000" in ftp_h
        and "files.size() >= maximumEntries" in ftp_cpp
        and "cancelFlag" in ftp_cpp[ftp_cpp.index("bool FtpClient::listFiles("):
                                    ftp_cpp.index("bool FtpClient::downloadFile(")],
        "FTP LIST enumeration must be cancellable and memory bounded")

expected_limits = {
    "MaximumArchiveBytes": "2LL * 1024 * 1024 * 1024",
    "MaximumSingleFileBytes": "768LL * 1024 * 1024",
    "MaximumUncompressedBytes": "2LL * 1024 * 1024 * 1024",
    "MaximumCentralDirectoryEntries": "4096",
    "MinimumFreeDiskReserveBytes": "2LL * 1024 * 1024 * 1024",
    "MaximumDeviceRootEntriesScanned": "4096",
    "MaximumStagingDirectoriesPerAudit": "64",
    "MaximumStagingDirectoriesDeletedPerAudit": "16",
    "StagingCleanupMinimumAgeMs": "24LL * 60 * 60 * 1000",
}
for name, value in expected_limits.items():
    require(f"{name} = {value}" in archive_h, f"archive limit {name} drifted")

parse_archive = archive_cpp[archive_cpp.index("bool ParseArchive("):
                            archive_cpp.index("bool IsLinkLike(")]
require(parse_archive.index("PreflightZipStructure(") < parse_archive.index("QZipReader archive("),
        "EOCD/central bounds run before QZipReader::fileInfoList")
for token in (
    "kMaximumCentralDirectoryBytes",
    "kZip64LocatorSignature",
    "totalEntries > RemoteArchiveSecurity::MaximumCentralDirectoryEntries",
    "centralOffset) + centralSize",
    "flags & ((1u << 0)",
    "unixSymlink",
    "localName != centralName",
    "ExtraFieldIsBoundedAndNonZip64",
    "minimumLocalOffset != 0",
    "toCaseFolded()",
):
    require(token in archive_cpp, f"raw ZIP structural preflight missing {token}")

require("GetSystemDirectoryW" in archive_cpp and "SystemRoot" not in archive_cpp,
        "extractor resolves fixed System32 without spoofable environment variable")
require("QProcess extractor" in archive_cpp and "CREATE_NO_WINDOW" in archive_cpp,
        "ZIP extraction uses controlled hidden child process")
require("extractor.terminate()" in archive_cpp and "extractor.kill()" in archive_cpp,
        "ZIP extraction can be forcibly cancelled")
require("drainDiagnostics" in archive_cpp and "diagnosticTail.right(4096)" in archive_cpp,
        "extractor pipes are continuously drained and bounded")
require("lastResourceScanMs" in archive_cpp
        and "resourceLimitExceeded" in archive_cpp
        and "MaximumUncompressedBytes - totalBytes" in archive_cpp,
        "running extractor is killed if actual output exceeds file/count/byte/disk limits")
require("extractAll(" not in archive_cpp,
        "uncancellable in-process QZipReader extraction is forbidden")
require("VerifyExtractedFiles" in archive_cpp
        and "AtomicRenameDirectoryNoReplace" in archive_cpp
        and "MOVEFILE_REPLACE_EXISTING" not in archive_cpp,
        "staging is fully revalidated then promoted atomically without replacement")
for token in (
    "enum class PromotionStatus",
    "PromotedCleanupFailed",
    "QString promotedCasePath",
    "bool stagingCleanupSucceeded",
    "bool RequiresBatchStop() const noexcept",
    "CleanupStagingTransaction",
    "AuditAndCleanupStaging",
):
    require(token in archive_h, f"explicit promotion/staging API missing {token}")

promotion = archive_cpp[archive_cpp.index("PromotionResult ExtractValidateAndPromote("):]
commit_index = promotion.index("result.status = PromotionStatus::PromotedCleanupFailed")
require(commit_index < promotion.index("CleanupStagingTransaction(", commit_index),
        "committed state is published before post-rename staging cleanup")
require(promotion.index("result.promotedCasePath = finalCase")
        < promotion.index("CleanupStagingTransaction(", commit_index),
        "final path is published before post-rename staging cleanup")
require("result.status = PromotionStatus::Promoted;" in promotion
        and "result.stagingCleanupSucceeded = true;" in promotion,
        "successful cleanup upgrades explicit promotion result")

audit = archive_cpp[archive_cpp.index("bool AuditAndCleanupStaging("):
                    archive_cpp.index("PromotionResult ExtractValidateAndPromote(")]
for token in (
    "QDirIterator::NoIteratorFlags",
    "MaximumDeviceRootEntriesScanned",
    "MaximumStagingDirectoriesPerAudit",
    "MaximumStagingDirectoriesDeletedPerAudit",
    "StagingCleanupMinimumAgeMs",
):
    require(token in audit, f"bounded direct-child staging audit missing {token}")
require("StrictStagingNamePattern" in archive_cpp
        and "remote-staging-[0-9a-f]{32}" in archive_cpp,
        "staging identity requires one strict lowercase-hex direct child name")
require(audit.index("deletionCandidates.size()")
        < audit.index("for (const QString& candidate : deletionCandidates)"),
        "staging audit validates deletion budget before mutating residue")

download_worker = dialog_cpp[dialog_cpp.index("StartRemoteWorker([this, cfg, device, archives, localDir]"):
                             dialog_cpp.index("void OnlineServicesDialog::DeleteSelectedRemoteFiles()")]
require(download_worker.index("AuditAndCleanupStaging(")
        < download_worker.index("for (int archiveIndex = 0; archiveIndex < archives.size(); ++archiveIndex)"),
        "staging residue audit runs before the first batch download")
require("PromotionStatus::Promoted" in download_worker
        and "案例已落地但 staging 清理失败" in download_worker
        and "stoppedForStagingCleanup" in download_worker
        and "break;" in download_worker,
        "caller reports landed cleanup failure and stops the remaining batch")
require("QDir(transactionRoot).removeRecursively()" not in download_worker,
        "caller cannot ignore a second staging cleanup result")
require("ValidateOpenedArchiveSize" in dialog_cpp
        and "item->data(Qt::UserRole + 1)" in dialog_cpp,
        "legacy ZIPs bind LIST size through open/download verification")
require("kMaximumRemoteArchivesPerOperation = 32" in dialog_cpp,
        "each download/delete worker run has a bounded archive count")

print("PASS: remote FTP lifetime and bounded/cancellable ZIP staging gates")
