#pragma once

#include <QString>
#include <QtGlobal>

#include <atomic>

namespace RemoteArchiveSecurity
{
    constexpr qint64 MaximumArchiveBytes = 2LL * 1024 * 1024 * 1024;
    constexpr qint64 MaximumSingleFileBytes = 768LL * 1024 * 1024;
    constexpr qint64 MaximumUncompressedBytes = 2LL * 1024 * 1024 * 1024;
    constexpr int MaximumCentralDirectoryEntries = 4096;
    constexpr qint64 MinimumFreeDiskReserveBytes = 2LL * 1024 * 1024 * 1024;
    constexpr int MaximumDeviceRootEntriesScanned = 4096;
    constexpr int MaximumStagingDirectoriesPerAudit = 64;
    constexpr int MaximumStagingDirectoriesDeletedPerAudit = 16;
    constexpr qint64 StagingCleanupMinimumAgeMs = 24LL * 60 * 60 * 1000;

    struct ArchiveInspection
    {
        QString robotName;
        QString caseName;
        qint64 totalUncompressedBytes = 0;
        int fileCount = 0;
        int centralDirectoryEntryCount = 0;
    };

    enum class PromotionStatus
    {
        Failed,
        Promoted,
        PromotedCleanupFailed,
    };

    struct PromotionResult
    {
        PromotionStatus status = PromotionStatus::Failed;
        QString promotedCasePath;
        QString message;
        bool stagingCleanupSucceeded = false;

        bool IsCommitted() const noexcept
        {
            return status == PromotionStatus::Promoted
                || status == PromotionStatus::PromotedCleanupFailed;
        }

        bool RequiresBatchStop() const noexcept
        {
            return status == PromotionStatus::PromotedCleanupFailed
                || !stagingCleanupSucceeded;
        }
    };

    struct StagingAuditResult
    {
        int scannedEntries = 0;
        int stagingDirectories = 0;
        int deletedDirectories = 0;
    };

    // The server LIST size is mandatory for every archive, including legacy names
    // without an embedded length.  A write-once filename's embedded length must also
    // match, so neither format can bypass the same 2 GiB download ceiling.
    bool ValidateListedArchive(
        const QString& fileName,
        qulonglong listedBytes,
        QString* error = nullptr);
    bool ValidateOpenedArchiveSize(
        const QString& fileName,
        qulonglong listedBytes,
        qulonglong openedBytes,
        QString* error = nullptr);

    bool HasWorkspaceCapacity(
        const QString& pathOnTargetVolume,
        qulonglong additionalBytes,
        QString* error = nullptr);

    // Reads and validates the ZIP central directory without extracting.  Only the
    // uploader's single Result/<robot>/<case>/... layout is accepted.
    bool InspectArchive(
        const QString& archivePath,
        ArchiveInspection* inspection,
        QString* error = nullptr);

    // archivePath must live in a unique remote-staging-* child of deviceRoot.  The
    // complete archive is extracted and revalidated there, then the one case tree is
    // promoted by a same-volume no-replace atomic directory rename.  The result
    // distinguishes a pre-commit failure from a committed case whose staging cleanup
    // failed; a committed final path is never hidden from the caller.
    PromotionResult ExtractValidateAndPromote(
        const QString& archivePath,
        const QString& deviceRoot,
        const std::atomic_bool* cancelFlag);

    // Safe, checked cleanup for one direct remote-staging-<32 lowercase hex> child.
    bool CleanupStagingTransaction(
        const QString& transactionRoot,
        const QString& deviceRoot,
        QString* error = nullptr);

    // Batch-start audit is two-phase: validate every direct staging child and all
    // count/age budgets first, then delete only sufficiently old ordinary directories.
    // Any link, invalid name, young residue, scan overflow, or cleanup failure denies
    // the whole batch without treating the residue as absent.
    bool AuditAndCleanupStaging(
        const QString& deviceRoot,
        StagingAuditResult* result,
        QString* error = nullptr,
        qint64 nowUtcMs = -1);
}
