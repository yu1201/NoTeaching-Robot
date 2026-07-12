#pragma once

#include <QByteArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QString>
#include <QtGlobal>

#include <functional>

namespace PointCloudProofIntegrity
{
inline constexpr qint64 MaximumProofBytes = 256LL * 1024;
inline constexpr qint64 MaximumDenialTombstoneBytes = 16LL * 1024;
inline constexpr qint64 MaximumEvidenceFileBytes = 768LL * 1024 * 1024;
inline constexpr qint64 MaximumEvidenceTotalBytes = 2LL * 1024 * 1024 * 1024;
inline constexpr qint64 MaximumEvidenceLinesPerFile = 10LL * 1000 * 1000;
inline constexpr qint64 MaximumEvidenceTotalLines = 20LL * 1000 * 1000;
inline constexpr qint64 MaximumWeldPoseBytes = 256LL * 1024 * 1024;
inline constexpr qint64 MaximumWeldPoseLines = 2LL * 1000 * 1000;
inline constexpr qint64 MaximumWeldPoseLineBytes = 64LL * 1024;

struct BoundedFileDigest
{
    QString sha256;
    qint64 size = 0;
    qint64 lineCount = 0;
};

bool ReadFileBounded(
    const QString& path,
    qint64 maximumBytes,
    QByteArray& payload,
    QString& error);

bool HashFileBounded(
    const QString& path,
    qint64 maximumBytes,
    qint64 maximumLines,
    BoundedFileDigest& digest,
    QString& error,
    const std::function<bool()>& stopRequested = std::function<bool()>());

QByteArray CanonicalJsonBytes(const QJsonValue& value);

bool ConstantTimeEqual(const QByteArray& left, const QByteArray& right);

bool SignProof(
    QJsonObject& root,
    const QByteArray& key,
    QString& error);

bool VerifyProofMac(
    const QJsonObject& root,
    const QByteArray& key,
    QString& error);

bool BuildReceiptRecord(
    const QJsonObject& root,
    const QString& canonicalCasePath,
    const QByteArray& proofPayload,
    QString& scanRunId,
    QString& record,
    QString& error);

bool VerifyReceiptRecord(
    const QJsonObject& root,
    const QString& canonicalCasePath,
    const QByteArray& proofPayload,
    const QString& persistedRecord,
    QString& error);

QString DeniedTombstonePath(const QString& proofPath);

// A production motion keeps this read lease alive from its first proof check
// through the final terminal/safe-retract callback.  A producer cannot begin a
// replacement (and therefore cannot install a denial tombstone) while a reader
// owns the same proof.
class ProofUseLease
{
public:
    ProofUseLease() noexcept = default;
    ~ProofUseLease();
    ProofUseLease(const ProofUseLease&) = delete;
    ProofUseLease& operator=(const ProofUseLease&) = delete;
    ProofUseLease(ProofUseLease&& other) noexcept;
    ProofUseLease& operator=(ProofUseLease&& other) noexcept;

    bool IsActive() const noexcept { return m_active; }
    void Reset() noexcept;

private:
    friend bool AcquireProofUseLease(
        const QString&, ProofUseLease&, QString&);
    QString m_key;
    bool m_active = false;
};

bool AcquireProofUseLease(
    const QString& proofPath,
    ProofUseLease& lease,
    QString& error);

// A proof replacement is fail-closed: the tombstone is committed and read
// back before the caller is allowed to remove or replace an existing proof.
bool BeginProofReplacement(
    const QString& proofPath,
    const QString& reason,
    QString& error);

// Destructive/write steps use this to prove that the fail-closed tombstone is
// still active in this process and on disk.
bool RequireProofReplacementActive(
    const QString& proofPath,
    QString& error);

// Production verification must call this before trusting proof bytes. Any
// tombstone, access ambiguity, or in-process failed replacement denies use.
bool VerifyProofNotDenied(
    const QString& proofPath,
    QString& error);

// Only a fully successful producer calls this. The in-process denial is
// released only after tombstone removal has been read back as absent.
bool CompleteProofReplacement(
    const QString& proofPath,
    QString& error);

// Failure/cancellation keeps the denial but drops the in-process producer
// privilege so a later operation cannot inherit an abandoned transaction.
void AbandonProofReplacement(const QString& proofPath);
}
