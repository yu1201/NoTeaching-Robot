#include "RemoteArchiveSecurity.h"
#include "RemoteWorkerLifecycle.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QtCore/private/qzipwriter_p.h>

#include <atomic>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <chrono>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

namespace
{
    void Require(bool condition, const char* message)
    {
        if (!condition)
        {
            std::cerr << "FAIL: " << message << std::endl;
            std::exit(1);
        }
    }

    bool WriteZip(
        const QString& path,
        const QList<QPair<QString, QByteArray>>& files,
        const QPair<QString, QString>* symlink = nullptr)
    {
        QZipWriter writer(path);
        writer.setCompressionPolicy(QZipWriter::AlwaysCompress);
        for (const auto& file : files)
        {
            writer.addFile(file.first, file.second);
        }
        if (symlink != nullptr)
        {
            writer.addSymLink(symlink->first, symlink->second);
        }
        writer.close();
        return writer.status() == QZipWriter::NoError;
    }

    QString NewStage(const QString& deviceRoot, const QString& suffix)
    {
        const QString identity = QString::fromLatin1(QCryptographicHash::hash(
            suffix.toUtf8(), QCryptographicHash::Sha256).toHex().left(32));
        const QString stage = QDir(deviceRoot).filePath(
            QStringLiteral("remote-staging-") + identity);
        Require(QDir().mkpath(stage), "create stage");
        return stage;
    }

#ifdef Q_OS_WIN
    class DeleteBlockingFileLock
    {
    public:
        ~DeleteBlockingFileLock()
        {
            Close();
        }

        bool Open(const QString& path)
        {
            Close();
            m_handle = CreateFileW(
                reinterpret_cast<LPCWSTR>(path.utf16()),
                GENERIC_READ,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                nullptr,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);
            return m_handle != INVALID_HANDLE_VALUE;
        }

        void Close()
        {
            if (m_handle != INVALID_HANDLE_VALUE)
            {
                CloseHandle(m_handle);
                m_handle = INVALID_HANDLE_VALUE;
            }
        }

    private:
        HANDLE m_handle = INVALID_HANDLE_VALUE;
    };
#endif

    int FindSignatureFromEnd(const QByteArray& bytes, quint32 signature)
    {
        for (int index = bytes.size() - 4; index >= 0; --index)
        {
            const auto at = [&](int offset)
                { return static_cast<quint32>(static_cast<unsigned char>(bytes.at(index + offset))); };
            const quint32 value = at(0) | (at(1) << 8) | (at(2) << 16) | (at(3) << 24);
            if (value == signature)
            {
                return index;
            }
        }
        return -1;
    }

    void WriteLe16(QByteArray* bytes, int offset, quint16 value)
    {
        (*bytes)[offset] = static_cast<char>(value & 0xff);
        (*bytes)[offset + 1] = static_cast<char>((value >> 8) & 0xff);
    }
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    Require(temporary.isValid(), "temporary root");
    const QString deviceRoot = QDir(temporary.path()).filePath(QStringLiteral("DeviceA"));
    Require(QDir().mkpath(deviceRoot), "device root");

    RemoteWorkerLifecycle lifecycle;
    const quint64 firstGeneration = lifecycle.BeginWorker();
    Require(firstGeneration != 0 && lifecycle.CanStart(), "first worker generation starts");
    lifecycle.BeginCancel(false);  // cached page close
    Require(lifecycle.IsCancelled(firstGeneration) && !lifecycle.CanStart(),
        "normal close cancels current worker and suppresses auto-restart during join");
    lifecycle.FinishCancel();
    Require(lifecycle.CanStart(), "normal close leaves cached page reusable");
    const quint64 reopenedGeneration = lifecycle.BeginWorker();
    Require(reopenedGeneration != 0
        && reopenedGeneration != firstGeneration
        && !lifecycle.IsCancelled(reopenedGeneration),
        "close then reopen can start a fresh generation");
    lifecycle.BeginCancel(true);   // app exit/destructor
    lifecycle.FinishCancel();
    Require(!lifecycle.CanStart() && lifecycle.BeginWorker() == 0,
        "permanent shutdown cannot restart remote work");

    QString error;
    Require(RemoteArchiveSecurity::ValidateListedArchive(
        QStringLiteral("legacy.zip"), 10, &error), "legacy LIST size accepted when bounded");
    Require(!RemoteArchiveSecurity::ValidateOpenedArchiveSize(
        QStringLiteral("legacy.zip"), 10, 11, &error), "legacy open size mismatch rejected");
    Require(!RemoteArchiveSecurity::ValidateListedArchive(
        QStringLiteral("Robot_Case_20260712T010203004_abcdef123456_9.zip"), 10, &error),
        "write-once embedded size mismatch rejected");
    Require(!RemoteArchiveSecurity::ValidateListedArchive(
        QStringLiteral("legacy.zip"),
        static_cast<qulonglong>(RemoteArchiveSecurity::MaximumArchiveBytes) + 1,
        &error), "2 GiB remote LIST ceiling");

    const QString stage = NewStage(deviceRoot, QStringLiteral("valid"));
    const QString validZip = QDir(stage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(validZip, {
        { QStringLiteral("RobotA/Case001/data.txt"), QByteArray("safe-data") },
        { QStringLiteral("RobotA/Case001/sub/empty.bin"), QByteArray() },
    }), "write valid zip");
    RemoteArchiveSecurity::ArchiveInspection inspection;
    Require(RemoteArchiveSecurity::InspectArchive(validZip, &inspection, &error),
        qPrintable(QStringLiteral("inspect valid zip: ") + error));
    Require(inspection.robotName == QStringLiteral("RobotA")
        && inspection.caseName == QStringLiteral("Case001")
        && inspection.fileCount == 2,
        "valid inspection layout");
    std::atomic_bool cancel{ false };
    const RemoteArchiveSecurity::PromotionResult validPromotion =
        RemoteArchiveSecurity::ExtractValidateAndPromote(
            validZip, deviceRoot, &cancel);
    Require(validPromotion.status == RemoteArchiveSecurity::PromotionStatus::Promoted,
        qPrintable(QStringLiteral("extract/promote valid zip: ") + validPromotion.message));
    const QString promoted = validPromotion.promotedCasePath;
    Require(promoted == QDir(deviceRoot).filePath(QStringLiteral("RobotA/Case001")),
        "promoted path");
    Require(QFile(QDir(promoted).filePath(QStringLiteral("data.txt"))).exists(),
        "promoted file exists");
    Require(!QFileInfo::exists(stage), "success removes transaction stage");

    const QString collisionStage = NewStage(deviceRoot, QStringLiteral("collision"));
    const QString collisionZip = QDir(collisionStage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(collisionZip, {
        { QStringLiteral("RobotA/Case001/data.txt"), QByteArray("replacement") },
    }), "write collision zip");
    const RemoteArchiveSecurity::PromotionResult collisionPromotion =
        RemoteArchiveSecurity::ExtractValidateAndPromote(
            collisionZip, deviceRoot, &cancel);
    Require(collisionPromotion.status == RemoteArchiveSecurity::PromotionStatus::Failed
        && !collisionPromotion.IsCommitted()
        && collisionPromotion.stagingCleanupSucceeded,
        "existing case is never overwritten");
    Require(!QFileInfo::exists(collisionStage), "collision failure removes stage");
    QFile original(QDir(deviceRoot).filePath(QStringLiteral("RobotA/Case001/data.txt")));
    Require(original.open(QIODevice::ReadOnly) && original.readAll() == QByteArray("safe-data"),
        "collision leaves original bytes unchanged");

    const QString cancelStage = NewStage(deviceRoot, QStringLiteral("cancel"));
    const QString cancelZip = QDir(cancelStage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(cancelZip, {
        { QStringLiteral("RobotA/Case002/data.txt"), QByteArray("cancel-me") },
    }), "write cancellation zip");
    cancel.store(true);
    const RemoteArchiveSecurity::PromotionResult cancelledBeforeStart =
        RemoteArchiveSecurity::ExtractValidateAndPromote(
            cancelZip, deviceRoot, &cancel);
    Require(cancelledBeforeStart.status == RemoteArchiveSecurity::PromotionStatus::Failed
        && !cancelledBeforeStart.IsCommitted()
        && cancelledBeforeStart.stagingCleanupSucceeded,
        "pre-cancel rejected");
    Require(!QFileInfo::exists(cancelStage), "cancellation removes stage");
    Require(!QFileInfo::exists(QDir(deviceRoot).filePath(QStringLiteral("RobotA/Case002"))),
        "cancellation promotes nothing");
    cancel.store(false);

    const auto inspectRejected = [&](const QString& suffix,
        const QList<QPair<QString, QByteArray>>& files,
        const QPair<QString, QString>* symlink = nullptr)
        {
            const QString badStage = NewStage(deviceRoot, suffix);
            const QString badZip = QDir(badStage).filePath(QStringLiteral("payload.zip"));
            Require(WriteZip(badZip, files, symlink), "write rejected zip fixture");
            RemoteArchiveSecurity::ArchiveInspection ignored;
            const bool accepted = RemoteArchiveSecurity::InspectArchive(badZip, &ignored, &error);
            Require(RemoteArchiveSecurity::CleanupStagingTransaction(
                badStage, deviceRoot, &error), "clean rejected fixture stage");
            return !accepted;
        };
    Require(inspectRejected(QStringLiteral("traversal"), {
        { QStringLiteral("../escape.txt"), QByteArray("bad") },
    }), "path traversal rejected");
    Require(inspectRejected(QStringLiteral("two-cases"), {
        { QStringLiteral("RobotA/Case002/a.txt"), QByteArray("a") },
        { QStringLiteral("RobotA/Case003/b.txt"), QByteArray("b") },
    }), "multiple cases rejected");
    Require(inspectRejected(QStringLiteral("case-fold"), {
        { QStringLiteral("RobotA/Case004/Data.txt"), QByteArray("a") },
        { QStringLiteral("RobotA/Case004/data.txt"), QByteArray("b") },
    }), "case-fold duplicate rejected");
    const QPair<QString, QString> symlink(
        QStringLiteral("RobotA/Case005/link"), QStringLiteral("target"));
    Require(inspectRejected(QStringLiteral("symlink"), {
        { QStringLiteral("RobotA/Case005/data.txt"), QByteArray("a") },
    }, &symlink), "symlink rejected");

    const QString countStage = NewStage(deviceRoot, QStringLiteral("entry-count"));
    const QString countZip = QDir(countStage).filePath(QStringLiteral("payload.zip"));
    QList<QPair<QString, QByteArray>> tooMany;
    for (int index = 0; index <= RemoteArchiveSecurity::MaximumCentralDirectoryEntries; ++index)
    {
        tooMany.push_back({
            QStringLiteral("RobotA/Case006/%1.txt").arg(index), QByteArray("x") });
    }
    Require(WriteZip(countZip, tooMany), "write central entry bomb fixture");
    Require(!RemoteArchiveSecurity::InspectArchive(countZip, &inspection, &error),
        "EOCD preflight rejects entry count before QZipReader list");
    Require(RemoteArchiveSecurity::CleanupStagingTransaction(
        countStage, deviceRoot, &error), "clean entry-count stage");

    const QString runningCancelStage = NewStage(deviceRoot, QStringLiteral("running-cancel"));
    const QString runningCancelZip = QDir(runningCancelStage).filePath(QStringLiteral("payload.zip"));
    QList<QPair<QString, QByteArray>> maximumAllowedEntries;
    for (int index = 0; index < RemoteArchiveSecurity::MaximumCentralDirectoryEntries; ++index)
    {
        maximumAllowedEntries.push_back({
            QStringLiteral("RobotA/Case008/%1.txt").arg(index), QByteArray("x") });
    }
    Require(WriteZip(runningCancelZip, maximumAllowedEntries),
        "write cancellable extraction fixture");
    cancel.store(false);
    std::thread cancelThread([&cancel]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            cancel.store(true);
        });
    const RemoteArchiveSecurity::PromotionResult cancelledExtraction =
        RemoteArchiveSecurity::ExtractValidateAndPromote(
            runningCancelZip, deviceRoot, &cancel);
    cancelThread.join();
    Require(cancelledExtraction.status == RemoteArchiveSecurity::PromotionStatus::Failed
        && !cancelledExtraction.IsCommitted()
        && cancelledExtraction.message.contains(QStringLiteral("子进程解压已取消")),
        "live System32 tar extraction is terminated by cancellation");
    Require(!QFileInfo::exists(runningCancelStage)
        && !QFileInfo::exists(QDir(deviceRoot).filePath(QStringLiteral("RobotA/Case008"))),
        "live extraction cancellation waits for child then cleans staging without promotion");
    cancel.store(false);

    const QString encryptedStage = NewStage(deviceRoot, QStringLiteral("encrypted-flag"));
    const QString encryptedZip = QDir(encryptedStage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(encryptedZip, {
        { QStringLiteral("RobotA/Case007/data.txt"), QByteArray("x") },
    }), "write encryption flag fixture");
    QFile encryptedFile(encryptedZip);
    Require(encryptedFile.open(QIODevice::ReadWrite), "open encryption fixture");
    QByteArray encryptedBytes = encryptedFile.readAll();
    const int localHeader = FindSignatureFromEnd(encryptedBytes, 0x04034b50u);
    const int centralHeader = FindSignatureFromEnd(encryptedBytes, 0x02014b50u);
    Require(localHeader >= 0 && centralHeader >= 0, "find zip headers");
    WriteLe16(&encryptedBytes, localHeader + 6,
        static_cast<quint16>(static_cast<unsigned char>(encryptedBytes.at(localHeader + 6))) | 1u);
    WriteLe16(&encryptedBytes, centralHeader + 8,
        static_cast<quint16>(static_cast<unsigned char>(encryptedBytes.at(centralHeader + 8))) | 1u);
    encryptedFile.resize(0);
    Require(encryptedFile.write(encryptedBytes) == encryptedBytes.size(), "patch encryption flags");
    encryptedFile.close();
    Require(!RemoteArchiveSecurity::InspectArchive(encryptedZip, &inspection, &error),
        "encrypted flag rejected in raw structure preflight");
    Require(RemoteArchiveSecurity::CleanupStagingTransaction(
        encryptedStage, deviceRoot, &error), "clean encrypted stage");

#ifdef Q_OS_WIN
    // The archive is readable, but a no-share-delete handle prevents payload.zip
    // cleanup after the case tree has crossed the atomic rename commit boundary.
    const QString lockedStage = NewStage(deviceRoot, QStringLiteral("locked-cleanup"));
    const QString lockedZip = QDir(lockedStage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(lockedZip, {
        { QStringLiteral("RobotA/Case009/data.txt"), QByteArray("committed-before-cleanup") },
    }), "write cleanup-lock fixture");
    const QString unattemptedStage = NewStage(deviceRoot, QStringLiteral("batch-second"));
    const QString unattemptedZip =
        QDir(unattemptedStage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(unattemptedZip, {
        { QStringLiteral("RobotA/Case010/data.txt"), QByteArray("must-not-land") },
    }), "write second batch fixture");
    DeleteBlockingFileLock payloadLock;
    Require(payloadLock.Open(lockedZip), "open payload delete-blocking lock");

    RemoteArchiveSecurity::PromotionResult lockedPromotion;
    int attemptedArchives = 0;
    for (const QString& batchZip : QList<QString>{ lockedZip, unattemptedZip })
    {
        ++attemptedArchives;
        const RemoteArchiveSecurity::PromotionResult current =
            RemoteArchiveSecurity::ExtractValidateAndPromote(
                batchZip, deviceRoot, &cancel);
        if (attemptedArchives == 1)
        {
            lockedPromotion = current;
        }
        if (current.RequiresBatchStop())
        {
            break;
        }
    }
    const QString lockedFinal =
        QDir(deviceRoot).filePath(QStringLiteral("RobotA/Case009"));
    Require(lockedPromotion.status
            == RemoteArchiveSecurity::PromotionStatus::PromotedCleanupFailed
        && lockedPromotion.IsCommitted()
        && !lockedPromotion.stagingCleanupSucceeded
        && lockedPromotion.promotedCasePath == lockedFinal
        && QFileInfo::exists(QDir(lockedFinal).filePath(QStringLiteral("data.txt")))
        && QFileInfo::exists(lockedStage),
        "rename commit remains explicit and visible when payload cleanup is locked");
    Require(attemptedArchives == 1
        && !QFileInfo::exists(
            QDir(deviceRoot).filePath(QStringLiteral("RobotA/Case010"))),
        "cleanup ambiguity stops the remaining batch immediately");

    payloadLock.Close();
    Require(RemoteArchiveSecurity::CleanupStagingTransaction(
        lockedStage, deviceRoot, &error), "checked cleanup retry after releasing lock");
    Require(RemoteArchiveSecurity::CleanupStagingTransaction(
        unattemptedStage, deviceRoot, &error), "clean unattempted batch stage");

    const QString retryStage = NewStage(deviceRoot, QStringLiteral("locked-cleanup-retry"));
    const QString retryZip = QDir(retryStage).filePath(QStringLiteral("payload.zip"));
    Require(WriteZip(retryZip, {
        { QStringLiteral("RobotA/Case009/data.txt"), QByteArray("replacement-denied") },
    }), "write retry fixture");
    const RemoteArchiveSecurity::PromotionResult retryPromotion =
        RemoteArchiveSecurity::ExtractValidateAndPromote(
            retryZip, deviceRoot, &cancel);
    Require(retryPromotion.status == RemoteArchiveSecurity::PromotionStatus::Failed
        && !retryPromotion.IsCommitted()
        && retryPromotion.promotedCasePath.isEmpty()
        && retryPromotion.stagingCleanupSucceeded
        && QFileInfo::exists(QDir(lockedFinal).filePath(QStringLiteral("data.txt"))),
        "retry collision is never misreported as a new commit");
#endif

    const QString youngStage = NewStage(deviceRoot, QStringLiteral("young-audit"));
    RemoteArchiveSecurity::StagingAuditResult auditResult;
    Require(!RemoteArchiveSecurity::AuditAndCleanupStaging(
        deviceRoot, &auditResult, &error), "young staging residue denies batch start");
    Require(QFileInfo::exists(youngStage), "young residue is never auto-deleted");
    const qint64 agedNow = QDateTime::currentDateTimeUtc().toMSecsSinceEpoch()
        + RemoteArchiveSecurity::StagingCleanupMinimumAgeMs + 60 * 1000;
    Require(RemoteArchiveSecurity::AuditAndCleanupStaging(
        deviceRoot, &auditResult, &error, agedNow)
        && auditResult.deletedDirectories == 1
        && !QFileInfo::exists(youngStage),
        "one sufficiently old residue is safely audited and deleted");

    QList<QString> overDeleteBudgetStages;
    for (int index = 0;
        index <= RemoteArchiveSecurity::MaximumStagingDirectoriesDeletedPerAudit;
        ++index)
    {
        overDeleteBudgetStages.push_back(NewStage(
            deviceRoot, QStringLiteral("delete-budget-%1").arg(index)));
    }
    Require(!RemoteArchiveSecurity::AuditAndCleanupStaging(
        deviceRoot, &auditResult, &error, agedNow),
        "staging deletion budget denies oversized cleanup set");
    Require(std::all_of(overDeleteBudgetStages.cbegin(), overDeleteBudgetStages.cend(),
        [](const QString& path) { return QFileInfo::exists(path); }),
        "two-phase audit performs no partial deletion on delete-budget overflow");
    for (const QString& overDeleteBudgetStage : overDeleteBudgetStages)
    {
        Require(RemoteArchiveSecurity::CleanupStagingTransaction(
            overDeleteBudgetStage, deviceRoot, &error), "clean delete-budget fixture");
    }

    QList<QString> overLimitStages;
    for (int index = 0;
        index <= RemoteArchiveSecurity::MaximumStagingDirectoriesPerAudit;
        ++index)
    {
        overLimitStages.push_back(NewStage(
            deviceRoot, QStringLiteral("residue-limit-%1").arg(index)));
    }
    Require(!RemoteArchiveSecurity::AuditAndCleanupStaging(
        deviceRoot, &auditResult, &error, agedNow),
        "staging residue count limit denies audit");
    Require(std::all_of(overLimitStages.cbegin(), overLimitStages.cend(),
        [](const QString& path) { return QFileInfo::exists(path); }),
        "two-phase residue audit performs no partial deletion on count overflow");
    for (const QString& overLimitStage : overLimitStages)
    {
        Require(RemoteArchiveSecurity::CleanupStagingTransaction(
            overLimitStage, deviceRoot, &error), "clean residue-limit fixture");
    }

    std::cout << "PASS: explicit promotion commit state, cleanup-lock batch stop, bounded staging audit, live tar cancellation, and atomic case promotion"
        << std::endl;
    return 0;
}
