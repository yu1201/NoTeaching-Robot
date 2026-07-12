#include "PointCloudProofIntegrity.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QTextStream>

#include <cstdlib>
#include <condition_variable>
#include <mutex>
#include <thread>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
void Check(bool condition, const QString& message)
{
    if (!condition)
    {
        QTextStream(stderr) << "FAIL: " << message << Qt::endl;
        std::exit(1);
    }
}

QJsonObject Evidence(const QString& relativePath, const QString& digest)
{
    QJsonObject evidence;
    evidence.insert(QStringLiteral("relativePath"), relativePath);
    evidence.insert(QStringLiteral("size"), 123.0);
    evidence.insert(QStringLiteral("sha256"), digest);
    return evidence;
}

QJsonObject ValidUnsignedProof()
{
    const QString digestA(64, QLatin1Char('a'));
    const QString digestB(64, QLatin1Char('b'));
    const QString digestC(64, QLatin1Char('c'));

    QJsonObject context;
    context.insert(QStringLiteral("origin"), QStringLiteral("liveRobotCameraScan"));
    context.insert(QStringLiteral("scanRunId"),
        QStringLiteral("12345678-1234-4abc-8def-1234567890ab"));
    context.insert(QStringLiteral("robotEndpoint"), QStringLiteral("192.0.2.10:5000"));
    context.insert(QStringLiteral("cameraSection"), QStringLiteral("CameraA"));
    context.insert(QStringLiteral("handEyeSha256"), digestA);

    QJsonArray inputs;
    inputs.push_back(Evidence(QStringLiteral("PreciseLaserPoint.txt"), digestA));
    inputs.push_back(Evidence(QStringLiteral("PreciseLaserPoint_KeyPoints.txt"), digestB));
    inputs.push_back(Evidence(QStringLiteral("PreciseLaserPoint_Classified.txt"), digestC));

    QJsonObject artifacts;
    artifacts.insert(QStringLiteral("authorizedPose"),
        Evidence(QStringLiteral("PreciseLaserPoint_WeldPose_2mm.txt"), digestC));

    QJsonObject root;
    root.insert(QStringLiteral("schemaVersion"), 3);
    root.insert(QStringLiteral("purpose"), QStringLiteral("productionMotionAuthorization"));
    root.insert(QStringLiteral("state"), QStringLiteral("PASS"));
    root.insert(QStringLiteral("createdUtc"), QStringLiteral("2026-07-12T02:03:04.000Z"));
    root.insert(QStringLiteral("caseId"), QStringLiteral("case-a"));
    root.insert(QStringLiteral("productionContext"), context);
    root.insert(QStringLiteral("inputs"), inputs);
    root.insert(QStringLiteral("artifacts"), artifacts);
    return root;
}


void WriteBytes(const QString& path, const QByteArray& payload)
{
    QFile file(path);
    Check(file.open(QIODevice::WriteOnly | QIODevice::Truncate),
        QStringLiteral("open test file for writing: %1").arg(path));
    Check(file.write(payload) == payload.size(),
        QStringLiteral("write complete test payload: %1").arg(path));
    file.close();
}

#ifdef Q_OS_WIN
class ExclusiveFileLock
{
public:
    explicit ExclusiveFileLock(const QString& path)
    {
        const QString native = QDir::toNativeSeparators(path);
        m_handle = CreateFileW(
            reinterpret_cast<LPCWSTR>(native.utf16()),
            GENERIC_READ,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr);
        Check(m_handle != INVALID_HANDLE_VALUE,
            QStringLiteral("take exclusive Windows file lock: %1").arg(path));
    }

    ~ExclusiveFileLock()
    {
        if (m_handle != INVALID_HANDLE_VALUE)
        {
            CloseHandle(m_handle);
        }
    }

private:
    HANDLE m_handle = INVALID_HANDLE_VALUE;
};
#endif
}

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    QString error;
    const QByteArray key = QByteArray::fromHex(
        "000102030405060708090a0b0c0d0e0f"
        "101112131415161718191a1b1c1d1e1f");
    QByteArray wrongKey = key;
    wrongKey[0] = static_cast<char>(wrongKey.at(0) ^ 0x7f);

    QJsonObject proof = ValidUnsignedProof();
    Check(PointCloudProofIntegrity::SignProof(proof, key, error),
        QStringLiteral("sign valid proof: %1").arg(error));
    Check(PointCloudProofIntegrity::VerifyProofMac(proof, key, error),
        QStringLiteral("verify valid proof: %1").arg(error));

    const QJsonObject roundTripped = QJsonDocument::fromJson(
        QJsonDocument(proof).toJson(QJsonDocument::Compact)).object();
    Check(PointCloudProofIntegrity::VerifyProofMac(roundTripped, key, error),
        QStringLiteral("canonical JSON must survive parse/key-order changes"));

    QJsonObject forged = proof;
    forged.insert(QStringLiteral("state"), QStringLiteral("FAIL"));
    Check(!PointCloudProofIntegrity::VerifyProofMac(forged, key, error),
        QStringLiteral("state mutation must invalidate the production MAC"));

    forged = proof;
    QJsonArray forgedInputs = forged.value(QStringLiteral("inputs")).toArray();
    QJsonObject firstInput = forgedInputs.at(0).toObject();
    firstInput.insert(QStringLiteral("sha256"), QString(64, QLatin1Char('d')));
    forgedInputs.replace(0, firstInput);
    forged.insert(QStringLiteral("inputs"), forgedInputs);
    Check(!PointCloudProofIntegrity::VerifyProofMac(forged, key, error),
        QStringLiteral("input-evidence mutation must invalidate the production MAC"));

    Check(!PointCloudProofIntegrity::VerifyProofMac(proof, wrongKey, error),
        QStringLiteral("a different machine/key must not verify the proof"));
    forged = ValidUnsignedProof();
    Check(PointCloudProofIntegrity::SignProof(forged, wrongKey, error),
        QStringLiteral("construct wrong-key attack fixture"));
    Check(!PointCloudProofIntegrity::VerifyProofMac(forged, key, error),
        QStringLiteral("attacker re-signing with another key must fail"));

    forged = proof;
    QJsonObject integrity = forged.value(QStringLiteral("integrity")).toObject();
    integrity.insert(QStringLiteral("attackerField"), true);
    forged.insert(QStringLiteral("integrity"), integrity);
    Check(!PointCloudProofIntegrity::VerifyProofMac(forged, key, error),
        QStringLiteral("non-canonical integrity fields must fail closed"));
    forged = ValidUnsignedProof();
    Check(!PointCloudProofIntegrity::SignProof(forged, QByteArray(31, 'x'), error),
        QStringLiteral("non-256-bit signing key must fail closed"));

    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("create receipt test root"));
    const QString casePath = temp.filePath(QStringLiteral("LaserPoint"));
    Check(QDir().mkpath(casePath), QStringLiteral("create canonical LaserPoint case"));
    const QString canonicalCasePath = QDir(casePath).canonicalPath();
    const QByteArray proofPayload = QJsonDocument(proof).toJson(QJsonDocument::Indented);
    QString scanRunId;
    QString receipt;
    Check(PointCloudProofIntegrity::BuildReceiptRecord(
            proof, canonicalCasePath, proofPayload, scanRunId, receipt, error),
        QStringLiteral("build valid receipt: %1").arg(error));
    Check(scanRunId == QStringLiteral("12345678-1234-4abc-8def-1234567890ab"),
        QStringLiteral("receipt scope must use the live scanRunId"));
    Check(PointCloudProofIntegrity::VerifyReceiptRecord(
            proof, canonicalCasePath, proofPayload, receipt, error),
        QStringLiteral("verify exact persisted receipt: %1").arg(error));
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            proof, canonicalCasePath, proofPayload, QString(), error),
        QStringLiteral("missing DPAPI receipt must fail closed"));
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            proof, temp.path(), proofPayload, receipt, error),
        QStringLiteral("copying a proof to another case path must reject its receipt"));
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            proof, canonicalCasePath, proofPayload + '\n', receipt, error),
        QStringLiteral("proof-byte/whitespace mutation must reject its receipt"));

    forged = proof;
    forgedInputs = forged.value(QStringLiteral("inputs")).toArray();
    firstInput = forgedInputs.at(0).toObject();
    firstInput.insert(QStringLiteral("sha256"), QString(64, QLatin1Char('d')));
    forgedInputs.replace(0, firstInput);
    forged.insert(QStringLiteral("inputs"), forgedInputs);
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            forged, canonicalCasePath, proofPayload, receipt, error),
        QStringLiteral("input-evidence mutation must reject its receipt"));

    forged = proof;
    QJsonObject forgedArtifacts = forged.value(QStringLiteral("artifacts")).toObject();
    QJsonObject forgedPose = forgedArtifacts.value(QStringLiteral("authorizedPose")).toObject();
    forgedPose.insert(QStringLiteral("sha256"), QString(64, QLatin1Char('e')));
    forgedArtifacts.insert(QStringLiteral("authorizedPose"), forgedPose);
    forged.insert(QStringLiteral("artifacts"), forgedArtifacts);
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            forged, canonicalCasePath, proofPayload, receipt, error),
        QStringLiteral("pose-evidence mutation must reject its receipt"));

    forged = proof;
    QJsonObject forgedContext = forged.value(QStringLiteral("productionContext")).toObject();
    forgedContext.insert(QStringLiteral("robotEndpoint"), QStringLiteral("192.0.2.99:5000"));
    forged.insert(QStringLiteral("productionContext"), forgedContext);
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            forged, canonicalCasePath, proofPayload, receipt, error),
        QStringLiteral("production-context mutation must reject its receipt"));

    forged = proof;
    forged.insert(QStringLiteral("createdUtc"), QStringLiteral("2026-07-12T02:03:05.000Z"));
    Check(!PointCloudProofIntegrity::VerifyReceiptRecord(
            forged, canonicalCasePath, proofPayload, receipt, error),
        QStringLiteral("proof-time mutation must reject its receipt"));

    forged = proof;
    forgedContext = forged.value(QStringLiteral("productionContext")).toObject();
    forgedContext.insert(QStringLiteral("scanRunId"), QStringLiteral("not-a-uuid"));
    forged.insert(QStringLiteral("productionContext"), forgedContext);
    Check(!PointCloudProofIntegrity::BuildReceiptRecord(
            forged, canonicalCasePath, proofPayload, scanRunId, receipt, error),
        QStringLiteral("invalid/non-live scanRunId must not create a receipt"));

    // The denial marker is installed before a producer may touch an old proof.
    // Verification rejects it until the producer explicitly completes.
    const QString replacementProof = temp.filePath(QStringLiteral("replacement-proof.json"));
    WriteBytes(replacementProof, proofPayload);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            replacementProof, QStringLiteral("normal replacement test"), error),
        QStringLiteral("begin proof replacement: %1").arg(error));
    Check(PointCloudProofIntegrity::RequireProofReplacementActive(replacementProof, error),
        QStringLiteral("active producer can confirm its denial marker: %1").arg(error));
    Check(!PointCloudProofIntegrity::VerifyProofNotDenied(replacementProof, error),
        QStringLiteral("ordinary verifier must reject an active replacement"));
    WriteBytes(replacementProof, proofPayload + QByteArrayLiteral("\n"));
    Check(PointCloudProofIntegrity::CompleteProofReplacement(replacementProof, error),
        QStringLiteral("complete proof replacement: %1").arg(error));
    Check(PointCloudProofIntegrity::VerifyProofNotDenied(replacementProof, error),
        QStringLiteral("completed replacement should remove and read back the marker"));

    // A production motion read lease and a proof replacement writer are mutually
    // exclusive.  The barrier keeps the reader alive while the writer attempts
    // BeginProofReplacement, making the former verify->START race deterministic.
    const QString leasedProof = temp.filePath(QStringLiteral("leased-proof.json"));
    WriteBytes(leasedProof, proofPayload);
    std::mutex leaseMutex;
    std::condition_variable leaseCv;
    bool readerAcquired = false;
    bool releaseReader = false;
    bool readerOk = false;
    QString readerError;
    std::thread readerThread([&]()
        {
            PointCloudProofIntegrity::ProofUseLease lease;
            readerOk = PointCloudProofIntegrity::AcquireProofUseLease(
                leasedProof, lease, readerError);
            {
                std::lock_guard<std::mutex> lock(leaseMutex);
                readerAcquired = true;
            }
            leaseCv.notify_all();
            std::unique_lock<std::mutex> lock(leaseMutex);
            leaseCv.wait(lock, [&]() { return releaseReader; });
        });
    {
        std::unique_lock<std::mutex> lock(leaseMutex);
        leaseCv.wait(lock, [&]() { return readerAcquired; });
    }
    Check(readerOk, QStringLiteral("acquire motion proof lease: %1").arg(readerError));
    Check(!PointCloudProofIntegrity::BeginProofReplacement(
            leasedProof, QStringLiteral("writer must not pass active motion lease"), error),
        QStringLiteral("replacement writer must fail immediately while motion lease is active"));
    Check(!QFileInfo::exists(PointCloudProofIntegrity::DeniedTombstonePath(leasedProof)),
        QStringLiteral("failed writer must not install a tombstone during active motion"));
    {
        std::lock_guard<std::mutex> lock(leaseMutex);
        releaseReader = true;
    }
    leaseCv.notify_all();
    readerThread.join();
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            leasedProof, QStringLiteral("writer after motion lease release"), error),
        QStringLiteral("writer should acquire after read lease release: %1").arg(error));
    PointCloudProofIntegrity::ProofUseLease writerBlockedReader;
    Check(!PointCloudProofIntegrity::AcquireProofUseLease(
            leasedProof, writerBlockedReader, error),
        QStringLiteral("motion reader must fail immediately while replacement writer is active"));
    Check(PointCloudProofIntegrity::CompleteProofReplacement(leasedProof, error),
        QStringLiteral("complete post-lease writer: %1").arg(error));

    // Size gates reject by metadata before allocating/reading a sparse giant file.
    const QString oversizedProof = temp.filePath(QStringLiteral("oversized-proof.json"));
    {
        QFile giant(oversizedProof);
        Check(giant.open(QIODevice::WriteOnly | QIODevice::Truncate),
            QStringLiteral("create oversized proof fixture"));
        Check(giant.resize(PointCloudProofIntegrity::MaximumProofBytes + 1),
            QStringLiteral("resize oversized proof fixture"));
    }
    QByteArray boundedPayload;
    Check(!PointCloudProofIntegrity::ReadFileBounded(
            oversizedProof,
            PointCloudProofIntegrity::MaximumProofBytes,
            boundedPayload,
            error),
        QStringLiteral("oversized proof must reject before readAll allocation"));
    Check(boundedPayload.isEmpty(),
        QStringLiteral("oversized proof rejection must not return partial bytes"));

    const QString giantEvidence = temp.filePath(QStringLiteral("giant-evidence.txt"));
    {
        QFile giant(giantEvidence);
        Check(giant.open(QIODevice::WriteOnly | QIODevice::Truncate),
            QStringLiteral("create sparse giant evidence fixture"));
        Check(giant.resize(PointCloudProofIntegrity::MaximumEvidenceFileBytes + 1),
            QStringLiteral("resize sparse giant evidence fixture"));
    }
    PointCloudProofIntegrity::BoundedFileDigest giantDigest;
    Check(!PointCloudProofIntegrity::HashFileBounded(
            giantEvidence,
            PointCloudProofIntegrity::MaximumEvidenceFileBytes,
            PointCloudProofIntegrity::MaximumEvidenceLinesPerFile,
            giantDigest,
            error),
        QStringLiteral("sparse giant evidence must reject before streaming"));

    const QString oversizedMarkerProof = temp.filePath(QStringLiteral("oversized-marker-proof.json"));
    WriteBytes(oversizedMarkerProof, proofPayload);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            oversizedMarkerProof, QStringLiteral("oversized tombstone test"), error),
        QStringLiteral("begin oversized tombstone fixture: %1").arg(error));
    {
        QFile marker(PointCloudProofIntegrity::DeniedTombstonePath(oversizedMarkerProof));
        Check(marker.open(QIODevice::WriteOnly | QIODevice::Truncate),
            QStringLiteral("open oversized tombstone fixture"));
        Check(marker.resize(PointCloudProofIntegrity::MaximumDenialTombstoneBytes + 1),
            QStringLiteral("resize oversized tombstone fixture"));
    }
    Check(!PointCloudProofIntegrity::RequireProofReplacementActive(
            oversizedMarkerProof, error),
        QStringLiteral("oversized tombstone must fail closed without readAll"));
    PointCloudProofIntegrity::AbandonProofReplacement(oversizedMarkerProof);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            oversizedMarkerProof, QStringLiteral("repair oversized tombstone fixture"), error),
        QStringLiteral("later writer repairs oversized tombstone: %1").arg(error));
    Check(PointCloudProofIntegrity::CompleteProofReplacement(oversizedMarkerProof, error),
        QStringLiteral("complete oversized tombstone cleanup: %1").arg(error));

    // Producer privilege is bound to the thread that persisted/read back the
    // marker. Another thread cannot use the internal STEP-generation bypass or
    // complete the transaction on the owner's behalf.
    const QString threadBoundProof = temp.filePath(QStringLiteral("thread-bound-proof.json"));
    WriteBytes(threadBoundProof, proofPayload);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            threadBoundProof, QStringLiteral("owner thread binding test"), error),
        QStringLiteral("begin thread-bound replacement: %1").arg(error));
    bool foreignRequire = true;
    bool foreignComplete = true;
    QString foreignRequireError;
    QString foreignCompleteError;
    std::thread foreignThread([&]()
        {
            foreignRequire = PointCloudProofIntegrity::RequireProofReplacementActive(
                threadBoundProof, foreignRequireError);
            foreignComplete = PointCloudProofIntegrity::CompleteProofReplacement(
                threadBoundProof, foreignCompleteError);
        });
    foreignThread.join();
    Check(!foreignRequire,
        QStringLiteral("non-owner thread must not inherit active producer privilege"));
    Check(!foreignComplete,
        QStringLiteral("non-owner thread must not complete another producer transaction"));
    Check(PointCloudProofIntegrity::RequireProofReplacementActive(threadBoundProof, error),
        QStringLiteral("owner must retain active privilege after cross-thread attacks: %1")
            .arg(error));
    Check(PointCloudProofIntegrity::CompleteProofReplacement(threadBoundProof, error),
        QStringLiteral("owner completes thread-bound replacement: %1").arg(error));

    // Marker existence alone is a denial. Corrupt/unparseable marker bytes can
    // never be interpreted as authorization, including after a process restart
    // where there is no in-memory transaction state.
    const QString corruptMarkerProof = temp.filePath(QStringLiteral("corrupt-marker-proof.json"));
    WriteBytes(corruptMarkerProof, proofPayload);
    WriteBytes(PointCloudProofIntegrity::DeniedTombstonePath(corruptMarkerProof),
        QByteArrayLiteral("{not-valid-json"));
    Check(!PointCloudProofIntegrity::VerifyProofNotDenied(corruptMarkerProof, error),
        QStringLiteral("corrupt persisted denial marker must reject proof without memory state"));
    Check(QFile::remove(PointCloudProofIntegrity::DeniedTombstonePath(corruptMarkerProof)),
        QStringLiteral("clean corrupt marker fixture"));

    const QString canceledProof = temp.filePath(QStringLiteral("canceled-proof.json"));
    WriteBytes(canceledProof, proofPayload);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            canceledProof, QStringLiteral("normal cancellation test"), error),
        QStringLiteral("begin canceled replacement: %1").arg(error));
    Check(QFile::remove(canceledProof), QStringLiteral("normal cancellation removes old proof"));
    PointCloudProofIntegrity::AbandonProofReplacement(canceledProof);
    Check(!PointCloudProofIntegrity::VerifyProofNotDenied(canceledProof, error),
        QStringLiteral("normal cancellation must retain a persistent denial tombstone"));
    Check(QFileInfo::exists(PointCloudProofIntegrity::DeniedTombstonePath(canceledProof)),
        QStringLiteral("normal cancellation tombstone must survive producer scope"));
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            canceledProof, QStringLiteral("cleanup canceled replacement test"), error),
        QStringLiteral("a later producer may safely supersede a stale tombstone: %1").arg(error));
    Check(PointCloudProofIntegrity::CompleteProofReplacement(canceledProof, error),
        QStringLiteral("cleanup canceled replacement test: %1").arg(error));

#ifdef Q_OS_WIN
    // Reproduce the production failure: an authorized proof is held with no
    // delete sharing. QFile::remove fails, but the already-persisted tombstone
    // still makes the real verification precondition reject the old proof.
    const QString lockedProof = temp.filePath(QStringLiteral("locked-authorized-proof.json"));
    WriteBytes(lockedProof, proofPayload);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            lockedProof, QStringLiteral("exclusive old proof cancellation test"), error),
        QStringLiteral("begin locked proof replacement: %1").arg(error));
    {
        ExclusiveFileLock lock(lockedProof);
        Check(!QFile::remove(lockedProof),
            QStringLiteral("exclusive old proof fixture must force deletion failure"));
        Check(!PointCloudProofIntegrity::VerifyProofNotDenied(lockedProof, error),
            QStringLiteral("locked old authorized proof must remain rejected"));
    }
    PointCloudProofIntegrity::AbandonProofReplacement(lockedProof);
    Check(QFileInfo::exists(lockedProof),
        QStringLiteral("old proof fixture must still exist after forced deletion failure"));
    Check(!PointCloudProofIntegrity::VerifyProofNotDenied(lockedProof, error),
        QStringLiteral("old proof must remain rejected after producer cancellation"));

    // A marker that cannot be removed must also keep the proof denied. The
    // failed Complete path is then abandoned, dropping producer privilege but
    // retaining both the process and on-disk denial.
    const QString lockedMarkerProof = temp.filePath(QStringLiteral("locked-marker-proof.json"));
    WriteBytes(lockedMarkerProof, proofPayload);
    Check(PointCloudProofIntegrity::BeginProofReplacement(
            lockedMarkerProof, QStringLiteral("exclusive marker test"), error),
        QStringLiteral("begin locked marker replacement: %1").arg(error));
    {
        ExclusiveFileLock markerLock(
            PointCloudProofIntegrity::DeniedTombstonePath(lockedMarkerProof));
        Check(!PointCloudProofIntegrity::CompleteProofReplacement(lockedMarkerProof, error),
            QStringLiteral("exclusive marker must prevent unsafe authorization commit"));
        Check(!PointCloudProofIntegrity::VerifyProofNotDenied(lockedMarkerProof, error),
            QStringLiteral("failed marker removal must keep proof denied"));
    }
    PointCloudProofIntegrity::AbandonProofReplacement(lockedMarkerProof);
    Check(!PointCloudProofIntegrity::VerifyProofNotDenied(lockedMarkerProof, error),
        QStringLiteral("failed commit must remain denied after producer privilege is abandoned"));
#endif

    QTextStream(stdout)
        << "PASS: real C++ point-cloud HMAC, receipt, and denial-tombstone attack gates"
        << Qt::endl;
    return 0;
}
