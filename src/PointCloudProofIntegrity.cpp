#include "PointCloudProofIntegrity.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMessageAuthenticationCode>
#include <QMutex>
#include <QMutexLocker>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QStringList>
#include <QThread>
#include <QUuid>

#include <algorithm>
#include <utility>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
constexpr int POINT_CLOUD_PROOF_RECEIPT_VERSION = 1;
constexpr auto POINT_CLOUD_PROOF_HMAC_ALGORITHM = "HMAC-SHA256-DPAPI-CurrentUser-v1";
constexpr auto POINT_CLOUD_PROOF_HMAC_KEY_ID = "pointcloud-proof-hmac-key-v1";
constexpr int POINT_CLOUD_PROOF_DENIAL_VERSION = 1;

QMutex g_proofReplacementMutex;
QSet<QString> g_deniedProofsInProcess;
QHash<QString, Qt::HANDLE> g_activeProofReplacements;
QHash<QString, int> g_activeProofReaders;

bool IsSha256Text(const QString& value)
{
    static const QRegularExpression pattern(QStringLiteral("^[0-9a-fA-F]{64}$"));
    return pattern.match(value).hasMatch();
}

QJsonValue CanonicalizeJson(const QJsonValue& value)
{
    if (value.isArray())
    {
        QJsonArray result;
        for (const QJsonValue& item : value.toArray())
        {
            result.push_back(CanonicalizeJson(item));
        }
        return result;
    }
    if (value.isObject())
    {
        const QJsonObject source = value.toObject();
        QStringList keys = source.keys();
        std::sort(keys.begin(), keys.end(), [](const QString& left, const QString& right)
            {
                return left.compare(right, Qt::CaseSensitive) < 0;
            });
        QJsonObject result;
        for (const QString& key : keys)
        {
            result.insert(key, CanonicalizeJson(source.value(key)));
        }
        return result;
    }
    return value;
}

QByteArray MacPayload(QJsonObject root)
{
    QJsonObject integrity = root.value(QStringLiteral("integrity")).toObject();
    integrity.remove(QStringLiteral("mac"));
    root.insert(QStringLiteral("integrity"), integrity);
    return QByteArrayLiteral("NoTeaching-Robot PointCloud Quality Proof v3\n")
        + PointCloudProofIntegrity::CanonicalJsonBytes(root);
}

QString DigestHex(const QByteArray& bytes)
{
    return QString::fromLatin1(
        QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex()).toLower();
}

QString NormalizedProofPath(const QString& proofPath)
{
    QString normalized = QDir::cleanPath(
        QFileInfo(QDir::fromNativeSeparators(proofPath)).absoluteFilePath());
#ifdef Q_OS_WIN
    normalized = normalized.toLower();
#endif
    return normalized;
}

QString TombstonePathForProof(const QString& proofPath)
{
    return QFileInfo(QDir::fromNativeSeparators(proofPath)).absoluteFilePath()
        + QStringLiteral(".denied");
}

bool ProbePath(const QString& path, bool& exists, QString& error)
{
#ifdef Q_OS_WIN
    const QString nativePath = QDir::toNativeSeparators(path);
    const DWORD attributes = GetFileAttributesW(
        reinterpret_cast<LPCWSTR>(nativePath.utf16()));
    if (attributes != INVALID_FILE_ATTRIBUTES)
    {
        exists = true;
        return true;
    }
    const DWORD code = GetLastError();
    if (code == ERROR_FILE_NOT_FOUND || code == ERROR_PATH_NOT_FOUND)
    {
        exists = false;
        return true;
    }
    error = QStringLiteral("检查点云证明拒绝标记失败（Windows 错误 %1）：%2")
        .arg(code)
        .arg(path);
    return false;
#else
    const QFileInfo info(path);
    exists = info.exists();
    return true;
#endif
}

bool ReadFileBoundedImpl(
    const QString& path,
    qint64 maximumBytes,
    QByteArray& payload,
    QString& error)
{
    payload.clear();
    if (maximumBytes <= 0)
    {
        error = QStringLiteral("受限文件读取上限无效。");
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("无法打开受限文件：%1").arg(path);
        return false;
    }
    const qint64 expectedBytes = file.size();
    if (expectedBytes < 0 || expectedBytes > maximumBytes)
    {
        error = QStringLiteral("文件大小 %1 超过 %2 字节硬上限：%3")
            .arg(expectedBytes).arg(maximumBytes).arg(path);
        return false;
    }
    payload = file.readAll();
    if (file.error() != QFileDevice::NoError
        || payload.size() != expectedBytes
        || file.size() != expectedBytes)
    {
        payload.clear();
        error = QStringLiteral("受限文件读取不完整或读取期间发生变化：%1").arg(path);
        return false;
    }
    return true;
}

void ReleaseProofReader(const QString& key) noexcept
{
    if (key.isEmpty())
    {
        return;
    }
    QMutexLocker locker(&g_proofReplacementMutex);
    const auto it = g_activeProofReaders.find(key);
    if (it == g_activeProofReaders.end())
    {
        return;
    }
    if (it.value() <= 1)
    {
        g_activeProofReaders.erase(it);
    }
    else
    {
        --it.value();
    }
}

QByteArray BuildDenialPayload(const QString& proofPath, const QString& reason)
{
    QJsonObject tombstone;
    tombstone.insert(QStringLiteral("schemaVersion"), POINT_CLOUD_PROOF_DENIAL_VERSION);
    tombstone.insert(QStringLiteral("state"), QStringLiteral("denied"));
    tombstone.insert(QStringLiteral("createdUtc"),
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    tombstone.insert(QStringLiteral("transactionId"),
        QUuid::createUuid().toString(QUuid::WithoutBraces).toLower());
    tombstone.insert(QStringLiteral("proofPathSha256"),
        DigestHex(NormalizedProofPath(proofPath).toUtf8()));
    tombstone.insert(QStringLiteral("reason"), reason.trimmed());
    return QJsonDocument(tombstone).toJson(QJsonDocument::Indented);
}

bool ReadValidDenialTombstone(
    const QString& proofPath,
    QByteArray* payload,
    QString& error)
{
    const QString markerPath = TombstonePathForProof(proofPath);
    QByteArray bytes;
    if (!ReadFileBoundedImpl(
            markerPath,
            PointCloudProofIntegrity::MaximumDenialTombstoneBytes,
            bytes,
            error))
    {
        error = QStringLiteral("点云证明拒绝标记回读失败：") + error;
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    const QJsonObject root = document.object();
    if (parseError.error != QJsonParseError::NoError
        || !document.isObject()
        || root.value(QStringLiteral("schemaVersion")).toInt(-1)
            != POINT_CLOUD_PROOF_DENIAL_VERSION
        || root.value(QStringLiteral("state")).toString() != QStringLiteral("denied")
        || root.value(QStringLiteral("proofPathSha256")).toString()
            != DigestHex(NormalizedProofPath(proofPath).toUtf8())
        || QUuid(root.value(QStringLiteral("transactionId")).toString()).isNull()
        || root.value(QStringLiteral("reason")).toString().trimmed().isEmpty())
    {
        error = QStringLiteral("点云证明拒绝标记损坏或路径绑定不一致，保持拒绝：%1")
            .arg(markerPath);
        return false;
    }
    if (payload != nullptr)
    {
        *payload = bytes;
    }
    return true;
}

bool WriteDenialTombstoneLocked(
    const QString& proofPath,
    const QString& reason,
    QString& error)
{
    const QString markerPath = TombstonePathForProof(proofPath);
    const QByteArray payload = BuildDenialPayload(proofPath, reason);
    if (payload.isEmpty()
        || payload.size() > PointCloudProofIntegrity::MaximumDenialTombstoneBytes)
    {
        error = QStringLiteral("点云证明拒绝标记超过 %1 字节硬上限。")
            .arg(PointCloudProofIntegrity::MaximumDenialTombstoneBytes);
        return false;
    }
    QSaveFile marker(markerPath);
    marker.setDirectWriteFallback(false);
    if (!marker.open(QIODevice::WriteOnly)
        || marker.write(payload) != payload.size()
        || !marker.commit())
    {
        marker.cancelWriting();
        error = QStringLiteral("无法原子提交点云证明拒绝标记：%1").arg(markerPath);
        return false;
    }
    QByteArray persisted;
    if (!ReadValidDenialTombstone(proofPath, &persisted, error)
        || !PointCloudProofIntegrity::ConstantTimeEqual(payload, persisted))
    {
        if (error.isEmpty())
        {
            error = QStringLiteral("点云证明拒绝标记写后回读字节不一致：%1")
                .arg(markerPath);
        }
        return false;
    }
    return true;
}
}

namespace PointCloudProofIntegrity
{
bool ReadFileBounded(
    const QString& path,
    qint64 maximumBytes,
    QByteArray& payload,
    QString& error)
{
    error.clear();
    return ReadFileBoundedImpl(path, maximumBytes, payload, error);
}

bool HashFileBounded(
    const QString& path,
    qint64 maximumBytes,
    qint64 maximumLines,
    BoundedFileDigest& digest,
    QString& error,
    const std::function<bool()>& stopRequested)
{
    digest = BoundedFileDigest{};
    error.clear();
    if (maximumBytes <= 0 || maximumLines <= 0)
    {
        error = QStringLiteral("文件证据字节/行数上限无效。");
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("无法打开文件证据：%1").arg(path);
        return false;
    }
    const qint64 expectedBytes = file.size();
    if (expectedBytes <= 0 || expectedBytes > maximumBytes)
    {
        error = QStringLiteral("文件证据大小 %1 不在 (0,%2] 字节范围：%3")
            .arg(expectedBytes).arg(maximumBytes).arg(path);
        return false;
    }

    constexpr qsizetype chunkBytes = 1024 * 1024;
    QByteArray buffer(chunkBytes, Qt::Uninitialized);
    QCryptographicHash hash(QCryptographicHash::Sha256);
    qint64 readBytes = 0;
    qint64 lineCount = 0;
    bool lastByteWasNewline = false;
    while (true)
    {
        if (stopRequested && stopRequested())
        {
            error = QStringLiteral("文件证据哈希已取消：%1").arg(path);
            return false;
        }
        const qint64 count = file.read(buffer.data(), buffer.size());
        if (count < 0)
        {
            error = QStringLiteral("文件证据读取失败：%1").arg(path);
            return false;
        }
        if (count == 0)
        {
            break;
        }
        if (readBytes > maximumBytes - count)
        {
            error = QStringLiteral("文件证据读取期间超过 %1 字节硬上限：%2")
                .arg(maximumBytes).arg(path);
            return false;
        }
        readBytes += count;
        hash.addData(QByteArrayView(buffer.constData(), static_cast<qsizetype>(count)));
        for (qint64 index = 0; index < count; ++index)
        {
            if (buffer.at(static_cast<qsizetype>(index)) == '\n')
            {
                ++lineCount;
                if (lineCount > maximumLines)
                {
                    error = QStringLiteral("文件证据行数超过 %1 硬上限：%2")
                        .arg(maximumLines).arg(path);
                    return false;
                }
            }
        }
        lastByteWasNewline = buffer.at(static_cast<qsizetype>(count - 1)) == '\n';
    }
    if (readBytes > 0 && !lastByteWasNewline)
    {
        ++lineCount;
    }
    if (file.error() != QFileDevice::NoError
        || readBytes != expectedBytes
        || file.size() != expectedBytes
        || lineCount > maximumLines)
    {
        error = QStringLiteral("文件证据不完整、读取期间变化或行数越界：%1").arg(path);
        return false;
    }
    digest.sha256 = QString::fromLatin1(hash.result().toHex()).toLower();
    digest.size = readBytes;
    digest.lineCount = lineCount;
    return true;
}

QByteArray CanonicalJsonBytes(const QJsonValue& value)
{
    const QJsonValue canonical = CanonicalizeJson(value);
    if (canonical.isObject())
    {
        return QJsonDocument(canonical.toObject()).toJson(QJsonDocument::Compact);
    }
    if (canonical.isArray())
    {
        return QJsonDocument(canonical.toArray()).toJson(QJsonDocument::Compact);
    }
    return {};
}

bool ConstantTimeEqual(const QByteArray& left, const QByteArray& right)
{
    if (left.size() != right.size())
    {
        return false;
    }
    unsigned char difference = 0;
    for (qsizetype index = 0; index < left.size(); ++index)
    {
        difference |= static_cast<unsigned char>(left.at(index))
            ^ static_cast<unsigned char>(right.at(index));
    }
    return difference == 0;
}

bool SignProof(QJsonObject& root, const QByteArray& key, QString& error)
{
    if (key.size() != 32)
    {
        error = QStringLiteral("点云证明 HMAC 密钥长度无效。");
        return false;
    }
    QJsonObject integrity;
    integrity.insert(QStringLiteral("algorithm"),
        QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_ALGORITHM));
    integrity.insert(QStringLiteral("keyId"),
        QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_KEY_ID));
    root.insert(QStringLiteral("integrity"), integrity);
    const QByteArray mac = QMessageAuthenticationCode::hash(
        MacPayload(root), key, QCryptographicHash::Sha256).toHex();
    integrity.insert(QStringLiteral("mac"), QString::fromLatin1(mac));
    root.insert(QStringLiteral("integrity"), integrity);
    error.clear();
    return true;
}

bool VerifyProofMac(const QJsonObject& root, const QByteArray& key, QString& error)
{
    const QJsonObject integrity = root.value(QStringLiteral("integrity")).toObject();
    const QString macText = integrity.value(QStringLiteral("mac")).toString();
    if (key.size() != 32
        || integrity.size() != 3
        || integrity.value(QStringLiteral("algorithm")).toString()
            != QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_ALGORITHM)
        || integrity.value(QStringLiteral("keyId")).toString()
            != QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_KEY_ID)
        || !IsSha256Text(macText))
    {
        error = QStringLiteral("点云质量证明缺少严格的本机 HMAC 完整性字段。");
        return false;
    }
    const QByteArray expected = QMessageAuthenticationCode::hash(
        MacPayload(root), key, QCryptographicHash::Sha256).toHex();
    if (!ConstantTimeEqual(expected, macText.toLatin1()))
    {
        error = QStringLiteral("点云质量证明 HMAC 不匹配；手写、复制或修改的证明禁止授权。");
        return false;
    }
    error.clear();
    return true;
}

bool BuildReceiptRecord(
    const QJsonObject& root,
    const QString& canonicalCasePath,
    const QByteArray& proofPayload,
    QString& scanRunId,
    QString& record,
    QString& error)
{
    const QJsonObject context = root.value(QStringLiteral("productionContext")).toObject();
    scanRunId = context.value(QStringLiteral("scanRunId")).toString().trimmed().toLower();
    const QUuid parsedRunId(scanRunId);
    const QString proofMac = root.value(QStringLiteral("integrity")).toObject()
        .value(QStringLiteral("mac")).toString().toLower();
    if (parsedRunId.isNull()
        || parsedRunId.toString(QUuid::WithoutBraces).compare(scanRunId, Qt::CaseInsensitive) != 0
        || canonicalCasePath.trimmed().isEmpty()
        || proofPayload.isEmpty()
        || !IsSha256Text(proofMac))
    {
        error = QStringLiteral("无法构造点云扫描收据：scanRunId、案例路径或证明摘要无效。");
        return false;
    }

    QJsonObject receipt;
    receipt.insert(QStringLiteral("receiptVersion"), POINT_CLOUD_PROOF_RECEIPT_VERSION);
    receipt.insert(QStringLiteral("scanRunId"), scanRunId);
    receipt.insert(QStringLiteral("createdUtc"), root.value(QStringLiteral("createdUtc")).toString());
    receipt.insert(QStringLiteral("casePathSha256"),
        DigestHex(QDir::cleanPath(canonicalCasePath).toUtf8()));
    receipt.insert(QStringLiteral("contextSha256"), DigestHex(CanonicalJsonBytes(context)));
    receipt.insert(QStringLiteral("inputEvidenceSha256"),
        DigestHex(CanonicalJsonBytes(root.value(QStringLiteral("inputs")))));
    receipt.insert(QStringLiteral("poseEvidenceSha256"),
        DigestHex(CanonicalJsonBytes(root.value(QStringLiteral("artifacts")))));
    receipt.insert(QStringLiteral("proofSha256"), DigestHex(proofPayload));
    receipt.insert(QStringLiteral("proofMac"), proofMac);
    record = QString::fromUtf8(CanonicalJsonBytes(receipt));
    if (record.isEmpty())
    {
        error = QStringLiteral("点云扫描收据规范化失败。");
        return false;
    }
    error.clear();
    return true;
}

bool VerifyReceiptRecord(
    const QJsonObject& root,
    const QString& canonicalCasePath,
    const QByteArray& proofPayload,
    const QString& persistedRecord,
    QString& error)
{
    QString scanRunId;
    QString expected;
    if (!BuildReceiptRecord(
            root, canonicalCasePath, proofPayload, scanRunId, expected, error))
    {
        return false;
    }
    if (persistedRecord.isEmpty())
    {
        error = QStringLiteral("本机不存在该 scanRunId 的 DPAPI 持久收据；手写/异机证明禁止授权。");
        return false;
    }
    if (!ConstantTimeEqual(persistedRecord.toUtf8(), expected.toUtf8()))
    {
        error = QStringLiteral("点云扫描收据与案例路径、输入、姿态或证明摘要不一致。");
        return false;
    }
    error.clear();
    return true;
}

QString DeniedTombstonePath(const QString& proofPath)
{
    return TombstonePathForProof(proofPath);
}

ProofUseLease::~ProofUseLease()
{
    Reset();
}

ProofUseLease::ProofUseLease(ProofUseLease&& other) noexcept
    : m_key(std::move(other.m_key)),
      m_active(std::exchange(other.m_active, false))
{
    other.m_key.clear();
}

ProofUseLease& ProofUseLease::operator=(ProofUseLease&& other) noexcept
{
    if (this != &other)
    {
        Reset();
        m_key = std::move(other.m_key);
        m_active = std::exchange(other.m_active, false);
        other.m_key.clear();
    }
    return *this;
}

void ProofUseLease::Reset() noexcept
{
    if (m_active)
    {
        ReleaseProofReader(m_key);
    }
    m_key.clear();
    m_active = false;
}

bool AcquireProofUseLease(
    const QString& proofPath,
    ProofUseLease& lease,
    QString& error)
{
    error.clear();
    if (proofPath.trimmed().isEmpty() || lease.m_active)
    {
        error = lease.m_active
            ? QStringLiteral("点云证明只读使用租约对象已被占用。")
            : QStringLiteral("点云证明只读使用租约缺少证明路径。");
        return false;
    }
    const QString key = NormalizedProofPath(proofPath);
    QMutexLocker locker(&g_proofReplacementMutex);
    if (g_deniedProofsInProcess.contains(key)
        || g_activeProofReplacements.contains(key))
    {
        error = QStringLiteral("点云证明正处于撤销/替换写事务，禁止取得运动使用租约。");
        return false;
    }
    bool markerExists = false;
    if (!ProbePath(TombstonePathForProof(proofPath), markerExists, error))
    {
        return false;
    }
    if (markerExists)
    {
        error = QStringLiteral("存在持久点云证明拒绝标记，禁止取得运动使用租约：%1")
            .arg(TombstonePathForProof(proofPath));
        return false;
    }
    g_activeProofReaders.insert(key, g_activeProofReaders.value(key, 0) + 1);
    lease.m_key = key;
    lease.m_active = true;
    return true;
}

bool BeginProofReplacement(
    const QString& proofPath,
    const QString& reason,
    QString& error)
{
    error.clear();
    if (proofPath.trimmed().isEmpty() || reason.trimmed().isEmpty())
    {
        error = QStringLiteral("点云证明替换缺少证明路径或拒绝原因。");
        return false;
    }
    const QString key = NormalizedProofPath(proofPath);
    QMutexLocker locker(&g_proofReplacementMutex);
    if (g_activeProofReplacements.contains(key))
    {
        error = QStringLiteral("同一点云证明已有替换事务在运行，拒绝并发覆盖。");
        return false;
    }
    if (g_activeProofReaders.value(key, 0) > 0)
    {
        error = QStringLiteral(
            "点云证明仍被生产运动只读租约占用；禁止在焊接执行期间安装拒绝标记或替换证明。");
        return false;
    }
    // Set this before any I/O. If persistence becomes ambiguous, the running
    // process remains fail-closed and cannot accept the old proof.  The active
    // replacement entry is the exclusive write lease and survives until
    // CompleteProofReplacement/AbandonProofReplacement.
    g_deniedProofsInProcess.insert(key);
    if (!WriteDenialTombstoneLocked(proofPath, reason, error))
    {
        return false;
    }
    g_activeProofReplacements.insert(key, QThread::currentThreadId());
    return true;
}

bool RequireProofReplacementActive(const QString& proofPath, QString& error)
{
    error.clear();
    const QString key = NormalizedProofPath(proofPath);
    QMutexLocker locker(&g_proofReplacementMutex);
    if (!g_activeProofReplacements.contains(key)
        || g_activeProofReplacements.value(key) != QThread::currentThreadId())
    {
        error = QStringLiteral("当前线程没有已回读确认的点云证明替换闭锁，禁止删除或写入证明。");
        return false;
    }
    bool markerExists = false;
    if (!ProbePath(TombstonePathForProof(proofPath), markerExists, error)
        || !markerExists)
    {
        if (error.isEmpty())
        {
            error = QStringLiteral("点云证明替换闭锁标记意外缺失，禁止继续。");
        }
        return false;
    }
    return ReadValidDenialTombstone(proofPath, nullptr, error);
}

bool VerifyProofNotDenied(const QString& proofPath, QString& error)
{
    error.clear();
    const QString key = NormalizedProofPath(proofPath);
    QMutexLocker locker(&g_proofReplacementMutex);
    if (g_deniedProofsInProcess.contains(key))
    {
        error = QStringLiteral("点云质量证明正处于撤销/替换闭锁，禁止下发或执行。");
        return false;
    }
    bool markerExists = false;
    if (!ProbePath(TombstonePathForProof(proofPath), markerExists, error))
    {
        return false;
    }
    if (markerExists)
    {
        error = QStringLiteral("存在持久点云证明拒绝标记，旧授权已失效：%1")
            .arg(TombstonePathForProof(proofPath));
        return false;
    }
    return true;
}

bool CompleteProofReplacement(const QString& proofPath, QString& error)
{
    error.clear();
    const QString key = NormalizedProofPath(proofPath);
    QMutexLocker locker(&g_proofReplacementMutex);
    if (!g_activeProofReplacements.contains(key)
        || g_activeProofReplacements.value(key) != QThread::currentThreadId())
    {
        error = QStringLiteral("点云证明替换闭锁未激活，禁止解除拒绝标记。");
        return false;
    }
    const QString markerPath = TombstonePathForProof(proofPath);
    bool markerExists = false;
    if (!ProbePath(markerPath, markerExists, error)
        || !markerExists
        || !ReadValidDenialTombstone(proofPath, nullptr, error))
    {
        const QString originalError = error.isEmpty()
            ? QStringLiteral("点云证明拒绝标记在解除前意外缺失。")
            : error;
        QString restoreError;
        WriteDenialTombstoneLocked(
            proofPath, QStringLiteral("解除拒绝标记前状态异常，继续保持闭锁。"), restoreError);
        error = restoreError.isEmpty()
            ? originalError
            : originalError + QStringLiteral("；恢复拒绝标记也失败：") + restoreError;
        return false;
    }
    if (!QFile::remove(markerPath))
    {
        const QString originalError = QStringLiteral(
            "无法解除点云证明拒绝标记，证明继续保持闭锁：%1")
            .arg(markerPath);
        QString restoreError;
        WriteDenialTombstoneLocked(
            proofPath, QStringLiteral("拒绝标记解除失败，继续保持闭锁。"), restoreError);
        error = restoreError.isEmpty()
            ? originalError
            : originalError + QStringLiteral("；恢复拒绝标记也失败：") + restoreError;
        return false;
    }
    if (!ProbePath(markerPath, markerExists, error) || markerExists)
    {
        const QString originalError = error.isEmpty()
            ? QStringLiteral("点云证明拒绝标记删除后回读仍存在，继续保持闭锁：%1")
                .arg(markerPath)
            : error;
        QString restoreError;
        WriteDenialTombstoneLocked(
            proofPath, QStringLiteral("拒绝标记解除回读异常，继续保持闭锁。"), restoreError);
        error = restoreError.isEmpty()
            ? originalError
            : originalError + QStringLiteral("；恢复拒绝标记也失败：") + restoreError;
        return false;
    }
    g_activeProofReplacements.remove(key);
    g_deniedProofsInProcess.remove(key);
    return true;
}

void AbandonProofReplacement(const QString& proofPath)
{
    const QString key = NormalizedProofPath(proofPath);
    QMutexLocker locker(&g_proofReplacementMutex);
    if (g_activeProofReplacements.value(key, nullptr) == QThread::currentThreadId())
    {
        g_activeProofReplacements.remove(key);
    }
    // Deliberately retain g_deniedProofsInProcess and the on-disk tombstone.
}
}
