#include "WeldSafetyRecoveryStore.h"

#include "AppPaths.h"
#include "ConfigDatabase.h"
#if !defined(WELD_SAFETY_STORE_STORAGE_ONLY_TEST)
#include "RobotDataHelper.h"
#include "RobotDriverAdaptor.h"
#include "RobotOperationLease.h"
#endif

#include <QDateTime>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QHostAddress>
#include <QRegularExpression>
#include <QUuid>

#include <cmath>
#include <mutex>

namespace
{
constexpr char kRecordKey[] = "RecordV2";
constexpr char kPendingKey[] = "SafeRetreatPending";
constexpr char kRecoveryModule[] = "WeldBreakpoint/Breakpoint";
constexpr char kEndpointScopeType[] = "weld_safety_endpoint";
constexpr char kEndpointModule[] = "RecoveryIndexV1";
constexpr char kEndpointIdentityKey[] = "EndpointIdentity";
constexpr char kEndpointRobotNameKey[] = "RobotName";
constexpr qsizetype kMaxRecordUtf8Bytes = 64 * 1024;
constexpr qsizetype kMaxEndpointIdentityLength = 512;
constexpr qsizetype kMaxRobotNameLength = 255;
constexpr qsizetype kMaxStoredRobotRecoveryRecords = 4096;
std::recursive_mutex g_storeMutex;
QMap<QString, QString> g_activeEndpointRecoveryBindings;

void SetError(QString* error, const QString& text)
{
    if (error != nullptr)
    {
        *error = text;
    }
}

QString StoreLabel(const QString& robotName)
{
    return QStringLiteral("robot/%1/%2").arg(
        robotName.trimmed(), QString::fromLatin1(kRecoveryModule));
}

bool ValidateRobotName(const QString& robotName, QString* error)
{
    const QString normalized = robotName.trimmed();
    if (normalized.isEmpty()
        || normalized.size() > kMaxRobotNameLength
        || !AppPaths::IsSafePathComponent(normalized))
    {
        SetError(error, QStringLiteral("机器人名称为空或包含不安全路径字符，无法访问焊接安全恢复记录。"));
        return false;
    }
    return true;
}

QString NormalizePersistentEndpointIdentity(const QString& endpointIdentity)
{
    const QString endpoint = endpointIdentity.trimmed();
    if (endpoint.isEmpty() || endpoint.size() > kMaxEndpointIdentityLength)
    {
        return {};
    }
    static const QRegularExpression pattern(
        QStringLiteral("^tcp:\\[([^\\[\\]]+)\\]:(\\d{1,5})$"));
    const QRegularExpressionMatch match = pattern.match(endpoint);
    if (!match.hasMatch())
    {
        return {};
    }
    QString host = match.captured(1).trimmed().toCaseFolded();
    while (host.size() > 1 && host.endsWith(QLatin1Char('.')))
    {
        host.chop(1);
    }
    for (const QChar ch : host)
    {
        if (ch.isSpace() || ch.isNull())
        {
            return {};
        }
    }
    QHostAddress address;
    if (address.setAddress(host))
    {
        host = address.toString().toCaseFolded();
    }
    bool portOk = false;
    const int port = match.captured(2).toInt(&portOk);
    if (host.isEmpty() || !portOk || port <= 0 || port > 65535)
    {
        return {};
    }
    return QStringLiteral("tcp:[%1]:%2").arg(host).arg(port);
}

ConfigDatabase::ReadStatus ReadRecoveryValueStatusLocked(
    const QString& robotName,
    const char* key,
    std::string& value,
    QString* error)
{
    value.clear();
    if (!ValidateRobotName(robotName, error))
    {
        return ConfigDatabase::ReadStatus::Error;
    }
    QString stored;
    const ConfigDatabase::ReadStatus status = ConfigDatabase::ReadScopedSettingStatus(
        QStringLiteral("robot"), robotName.trimmed(), QString::fromLatin1(kRecoveryModule),
        QString::fromLatin1(key), &stored);
    if (status == ConfigDatabase::ReadStatus::Found)
    {
        const QByteArray bytes = stored.toUtf8();
        value.assign(bytes.constData(), static_cast<std::size_t>(bytes.size()));
    }
    if (status == ConfigDatabase::ReadStatus::Error)
    {
        SetError(error, QString("读取焊接安全恢复存储失败：%1 [%2]")
            .arg(StoreLabel(robotName), QString::fromLatin1(key)));
    }
    return status;
}

ConfigDatabase::ReadStatus ReadRecordValueStatusLocked(
    const QString& robotName,
    QString& value,
    QString* error)
{
    value.clear();
    std::string bytes;
    const ConfigDatabase::ReadStatus status = ReadRecoveryValueStatusLocked(
        robotName, kRecordKey, bytes, error);
    if (status != ConfigDatabase::ReadStatus::Found)
    {
        return status;
    }
    if (bytes.empty() || bytes.size() > static_cast<std::size_t>(kMaxRecordUtf8Bytes))
    {
        SetError(error, bytes.empty()
            ? QStringLiteral("焊接恢复 RecordV2 为空，已拒绝读取。")
            : QStringLiteral("焊接恢复 RecordV2 超过 64 KiB 安全上限，已拒绝读取。"));
        return ConfigDatabase::ReadStatus::Error;
    }
    const QByteArray utf8(bytes.data(), static_cast<qsizetype>(bytes.size()));
    value = QString::fromUtf8(utf8);
    if (value.toUtf8() != utf8 || value.isEmpty())
    {
        value.clear();
        SetError(error, QStringLiteral("焊接恢复 RecordV2 不是严格 UTF-8，已拒绝读取。"));
        return ConfigDatabase::ReadStatus::Error;
    }
    return ConfigDatabase::ReadStatus::Found;
}

ConfigDatabase::ReadStatus ReadPendingValueStatusLocked(
    const QString& robotName,
    bool& pending,
    QString* error)
{
    pending = false;
    std::string value;
    const ConfigDatabase::ReadStatus status = ReadRecoveryValueStatusLocked(
        robotName, kPendingKey, value, error);
    if (status != ConfigDatabase::ReadStatus::Found)
    {
        return status;
    }
    if (value == "0")
    {
        pending = false;
        return ConfigDatabase::ReadStatus::Found;
    }
    if (value == "1")
    {
        pending = true;
        return ConfigDatabase::ReadStatus::Found;
    }
    SetError(error, QStringLiteral("焊后安全回撤门禁格式无效；只允许精确值 0 或 1。"));
    return ConfigDatabase::ReadStatus::Error;
}

bool ReadPendingLocked(const QString& robotName, bool& pending, QString* error)
{
    const ConfigDatabase::ReadStatus markerStatus = ReadPendingValueStatusLocked(
        robotName, pending, error);
    if (markerStatus == ConfigDatabase::ReadStatus::Found)
    {
        return true;
    }
    if (markerStatus == ConfigDatabase::ReadStatus::Error)
    {
        return false;
    }

    QString recordValue;
    const ConfigDatabase::ReadStatus recordStatus = ReadRecordValueStatusLocked(
        robotName, recordValue, error);
    if (recordStatus == ConfigDatabase::ReadStatus::NotFound)
    {
        pending = false;
        return true; // genuine first-use state: neither marker nor record exists
    }
    if (recordStatus == ConfigDatabase::ReadStatus::Found)
    {
        SetError(error, QStringLiteral(
            "检测到 RecordV2 但 SafeRetreatPending 缺失，无法证明安全终态，已保持失败关闭。"));
    }
    return false;
}

bool WritePendingLocked(const QString& robotName, bool pending, QString* error)
{
    if (!ValidateRobotName(robotName, error)
        || !ConfigDatabase::WriteScopedSetting(
            QStringLiteral("robot"), robotName.trimmed(), QString::fromLatin1(kRecoveryModule),
            QString::fromLatin1(kPendingKey), pending ? QStringLiteral("1") : QStringLiteral("0")))
    {
        SetError(error, QString("写入焊后安全回撤门禁失败：%1").arg(StoreLabel(robotName)));
        return false;
    }
    bool readback = !pending;
    if (ReadPendingValueStatusLocked(robotName, readback, error)
            != ConfigDatabase::ReadStatus::Found
        || readback != pending)
    {
        SetError(error, QStringLiteral("焊后安全回撤门禁写后回读不一致，保持失败关闭。"));
        return false;
    }
    return true;
}

bool WriteRecordLocked(const QString& robotName, const QString& encoded, QString* error)
{
    const QByteArray encodedUtf8 = encoded.toUtf8();
    if (encoded.trimmed().isEmpty()
        || encodedUtf8.isEmpty()
        || encodedUtf8.size() > kMaxRecordUtf8Bytes)
    {
        SetError(error, QStringLiteral("拒绝写入空记录或超过 64 KiB 的焊接恢复记录。"));
        return false;
    }
    if (!ValidateRobotName(robotName, error)
        || !ConfigDatabase::WriteScopedSetting(
            QStringLiteral("robot"), robotName.trimmed(), QString::fromLatin1(kRecoveryModule),
            QString::fromLatin1(kRecordKey), encoded))
    {
        SetError(error, QString("写入焊接恢复记录失败：%1").arg(StoreLabel(robotName)));
        return false;
    }
    QString readback;
    if (ReadRecordValueStatusLocked(robotName, readback, error)
            != ConfigDatabase::ReadStatus::Found
        || readback != encoded)
    {
        SetError(error, QStringLiteral("焊接恢复记录写后回读不一致，保持失败关闭。"));
        return false;
    }
    return true;
}

bool ReadRecordLocked(
    const QString& robotName,
    WeldResumePlanner::CheckpointRecord& record,
    QString* encoded,
    QString* error)
{
    QString value;
    const ConfigDatabase::ReadStatus status = ReadRecordValueStatusLocked(robotName, value, error);
    if (status != ConfigDatabase::ReadStatus::Found)
    {
        if (status == ConfigDatabase::ReadStatus::NotFound)
        {
            SetError(error, QStringLiteral("没有可验证的V2焊接恢复记录。"));
        }
        return false;
    }
    if (value.startsWith(QStringLiteral("invalidated:v2:")))
    {
        SetError(error, QStringLiteral("旧断点已被随后启动的完整焊接失效，禁止继续使用。"));
        return false;
    }
    if (!WeldResumePlanner::DecodeRecord(value, record, error))
    {
        return false;
    }
    if (encoded != nullptr)
    {
        *encoded = value;
    }
    return true;
}

ConfigDatabase::ReadStatus ReadEndpointIndexLocked(
    const QString& endpointIdentity,
    QString& robotName,
    QString* error)
{
    robotName.clear();
    const QString endpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    if (endpoint.isEmpty())
    {
        SetError(error, QStringLiteral("机器人持久端点为空，无法读取安全索引。"));
        return ConfigDatabase::ReadStatus::Error;
    }
    QString storedEndpoint;
    QString storedRobot;
    const ConfigDatabase::ReadStatus endpointStatus = ConfigDatabase::ReadScopedSettingStatus(
        QString::fromLatin1(kEndpointScopeType), endpoint,
        QString::fromLatin1(kEndpointModule), QString::fromLatin1(kEndpointIdentityKey),
        &storedEndpoint);
    const ConfigDatabase::ReadStatus robotStatus = ConfigDatabase::ReadScopedSettingStatus(
        QString::fromLatin1(kEndpointScopeType), endpoint,
        QString::fromLatin1(kEndpointModule), QString::fromLatin1(kEndpointRobotNameKey),
        &storedRobot);
    if (endpointStatus == ConfigDatabase::ReadStatus::NotFound
        && robotStatus == ConfigDatabase::ReadStatus::NotFound)
    {
        return ConfigDatabase::ReadStatus::NotFound;
    }
    if (endpointStatus != ConfigDatabase::ReadStatus::Found
        || robotStatus != ConfigDatabase::ReadStatus::Found
        || storedEndpoint != endpoint
        || storedRobot.trimmed().isEmpty()
        || !AppPaths::IsSafePathComponent(storedRobot.trimmed()))
    {
        SetError(error, QStringLiteral("焊接安全端点索引缺失、不可读或写后身份不一致。"));
        return ConfigDatabase::ReadStatus::Error;
    }
    robotName = storedRobot.trimmed();
    return ConfigDatabase::ReadStatus::Found;
}

bool WriteEndpointIndexLocked(
    const QString& endpointIdentity,
    const QString& robotName,
    QString* error)
{
    const QString endpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    const QString robot = robotName.trimmed();
    if (endpoint.isEmpty()
        || endpoint != endpointIdentity.trimmed()
        || !ValidateRobotName(robot, error))
    {
        SetError(error, QStringLiteral("拒绝写入空端点或无效机器人名称的焊接安全索引。"));
        return false;
    }
    QMap<QString, QString> values;
    values.insert(QString::fromLatin1(kEndpointIdentityKey), endpoint);
    values.insert(QString::fromLatin1(kEndpointRobotNameKey), robot);
    if (!ConfigDatabase::WriteScopedSettings(
            QString::fromLatin1(kEndpointScopeType), endpoint,
            QString::fromLatin1(kEndpointModule), values))
    {
        SetError(error, QStringLiteral("写入焊接安全端点索引失败。"));
        return false;
    }
    QString indexedRobot;
    if (ReadEndpointIndexLocked(endpoint, indexedRobot, error)
            != ConfigDatabase::ReadStatus::Found
        || indexedRobot != robot)
    {
        SetError(error, QStringLiteral("焊接安全端点索引写后回读不一致。"));
        return false;
    }
    return true;
}

bool EnsureEndpointIndexForRecordLocked(
    const QString& robotName,
    const QString& encoded,
    QString* error)
{
    WeldResumePlanner::CheckpointRecord record;
    QString decodeError;
    if (!WeldResumePlanner::DecodeRecord(encoded, record, &decodeError)
        || record.robotName.compare(robotName.trimmed(), Qt::CaseInsensitive) != 0
        || record.robotEndpoint.trimmed().isEmpty())
    {
        SetError(error, decodeError.isEmpty()
            ? QStringLiteral("焊接恢复记录缺少匹配的机器人名称或持久端点。")
            : decodeError);
        return false;
    }
    return WriteEndpointIndexLocked(record.robotEndpoint, robotName, error);
}

bool ProbeRobotAdmissionBlockedLocked(
    const QString& candidateRobotName,
    const QString& endpointIdentity,
    bool requireCandidateState,
    bool requireEndpointMatch,
    QString* reason)
{
    bool pending = false;
    QString markerError;
    const ConfigDatabase::ReadStatus markerStatus = ReadPendingValueStatusLocked(
        candidateRobotName, pending, &markerError);
    if (markerStatus == ConfigDatabase::ReadStatus::Error)
    {
        SetError(reason, QString("无法读取机器人 %1 的焊后安全回撤门禁：%2")
            .arg(candidateRobotName, markerError));
        return true;
    }
    if (markerStatus == ConfigDatabase::ReadStatus::Found && !pending)
    {
        return false; // explicit, strictly parsed safe terminal marker
    }

    QString encoded;
    QString recordReadError;
    const ConfigDatabase::ReadStatus recordStatus = ReadRecordValueStatusLocked(
        candidateRobotName, encoded, &recordReadError);
    if (markerStatus == ConfigDatabase::ReadStatus::NotFound
        && recordStatus == ConfigDatabase::ReadStatus::NotFound)
    {
        if (requireCandidateState)
        {
            SetError(reason, QString("端点索引指向机器人 %1，但对应 marker/RecordV2 均缺失，已失败关闭。")
                .arg(candidateRobotName));
            return true;
        }
        return false;
    }
    if (recordStatus == ConfigDatabase::ReadStatus::Error)
    {
        SetError(reason, QString("机器人 %1 的焊接恢复记录读取失败，已失败关闭：%2")
            .arg(candidateRobotName, recordReadError));
        return true;
    }
    if (recordStatus == ConfigDatabase::ReadStatus::NotFound)
    {
        SetError(reason, QString("机器人 %1 存在 SafeRetreatPending，但 RecordV2 缺失，已失败关闭。")
            .arg(candidateRobotName));
        return true;
    }

    WeldResumePlanner::CheckpointRecord record;
    QString decodeError;
    const bool decoded = !encoded.startsWith(QStringLiteral("invalidated:v2:"))
        && WeldResumePlanner::DecodeRecord(encoded, record, &decodeError);
    const QString recordEndpoint = decoded
        ? NormalizePersistentEndpointIdentity(record.robotEndpoint)
        : QString();
    if (decoded && recordEndpoint.isEmpty())
    {
        SetError(reason, QString("机器人 %1 的 RecordV2 持久端点不可规范化，已失败关闭。")
            .arg(candidateRobotName));
        return true;
    }
    if (markerStatus == ConfigDatabase::ReadStatus::NotFound)
    {
        if (!decoded
            || requireEndpointMatch
            || recordEndpoint == endpointIdentity)
        {
            SetError(reason, QString("机器人 %1 存在 RecordV2 但 SafeRetreatPending 缺失，无法证明安全终态。")
                .arg(candidateRobotName));
            return true;
        }
        return false;
    }

    // marker is present and pending=true from here.
    if (!decoded)
    {
        SetError(reason, QString("机器人 %1 的焊后安全门禁已锁存，但 RecordV2 无法验证：%2")
            .arg(candidateRobotName, decodeError));
        return true;
    }
    if (!requireEndpointMatch && recordEndpoint != endpointIdentity)
    {
        return false;
    }
    const QString endpointDetail = recordEndpoint == endpointIdentity
        ? recordEndpoint
        : QString("Record=%1 Current=%2").arg(recordEndpoint, endpointIdentity);
    SetError(reason,
        QString("焊后安全回撤尚未验证完成，禁止新的机器人操作。Robot=%1 Endpoint=%2 Program=%3 State=%4")
            .arg(candidateRobotName, endpointDetail, record.programName, record.state));
    return true;
}

struct EndpointRecoveryCandidate
{
    QString robotScope;
    QString encoded;
    QString encodedSha256;
    WeldResumePlanner::CheckpointRecord record;
};

QString EncodedRecordSha256(const QString& encoded)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        encoded.toUtf8(), QCryptographicHash::Sha256).toHex()).toLower();
}

bool CollectEndpointRecoveryCandidatesLocked(
    const QString& endpointIdentity,
    QVector<EndpointRecoveryCandidate>& candidates,
    QString* error)
{
    candidates.clear();
    QStringList storedRobots;
    if (!ConfigDatabase::TryListScopedSettingIdsBounded(
            QStringLiteral("robot"), QString::fromLatin1(kRecoveryModule),
            kMaxStoredRobotRecoveryRecords, kMaxRobotNameLength, &storedRobots))
    {
        SetError(error, QStringLiteral("恢复绑定无法有界枚举全部机器人 scope，已失败关闭。"));
        return false;
    }
    for (const QString& storedRobotValue : storedRobots)
    {
        const QString storedRobot = storedRobotValue.trimmed();
        if (storedRobot.isEmpty()
            || storedRobot.size() > kMaxRobotNameLength
            || !AppPaths::IsSafePathComponent(storedRobot))
        {
            SetError(error, QStringLiteral("恢复绑定枚举到无效机器人 scope，已失败关闭。"));
            return false;
        }
        bool pending = false;
        QString markerError;
        const ConfigDatabase::ReadStatus markerStatus = ReadPendingValueStatusLocked(
            storedRobot, pending, &markerError);
        if (markerStatus == ConfigDatabase::ReadStatus::Error)
        {
            SetError(error, QString("恢复绑定无法读取 scope %1 的 pending marker：%2")
                .arg(storedRobot, markerError));
            return false;
        }
        QString encoded;
        QString recordError;
        const ConfigDatabase::ReadStatus recordStatus = ReadRecordValueStatusLocked(
            storedRobot, encoded, &recordError);
        if (recordStatus == ConfigDatabase::ReadStatus::Error)
        {
            SetError(error, QString("恢复绑定无法读取 scope %1 的 RecordV2：%2")
                .arg(storedRobot, recordError));
            return false;
        }
        if (markerStatus == ConfigDatabase::ReadStatus::Found && !pending)
        {
            continue;
        }
        EndpointRecoveryCandidate candidate;
        candidate.robotScope = storedRobot;
        candidate.encoded = encoded;
        QString recordEndpoint;
        if (recordStatus == ConfigDatabase::ReadStatus::Found)
        {
            QString decodeError;
            if (encoded.startsWith(QStringLiteral("invalidated:v2:"))
                || !WeldResumePlanner::DecodeRecord(encoded, candidate.record, &decodeError))
            {
                SetError(error, QString("scope %1 的 RecordV2 无法解析：%2")
                    .arg(storedRobot, decodeError));
                return false;
            }
            recordEndpoint = NormalizePersistentEndpointIdentity(
                candidate.record.robotEndpoint);
            if (recordEndpoint.isEmpty()
                || candidate.record.robotName != storedRobot)
            {
                SetError(error, QString("scope %1 的 RecordV2 机器人 scope/端点身份无效。")
                    .arg(storedRobot));
                return false;
            }
        }

        if (markerStatus == ConfigDatabase::ReadStatus::NotFound)
        {
            if (recordStatus == ConfigDatabase::ReadStatus::Found
                && recordEndpoint == endpointIdentity)
            {
                SetError(error, QString("端点目标 scope %1 存在 RecordV2 但 marker 缺失。")
                    .arg(storedRobot));
                return false;
            }
            continue;
        }
        if (!pending)
        {
            continue;
        }
        if (recordStatus != ConfigDatabase::ReadStatus::Found)
        {
            SetError(error, QString("scope %1 的 pending marker 缺少 RecordV2。")
                .arg(storedRobot));
            return false;
        }
        if (recordEndpoint == endpointIdentity)
        {
            candidate.encodedSha256 = EncodedRecordSha256(encoded);
            candidates.push_back(candidate);
        }
    }
    return true;
}

bool ValidateExclusiveRecoverySnapshotLocked(
    const QString& robotName,
    const QString& endpointIdentity,
    const WeldResumePlanner::CheckpointRecord& expected,
    RobotRecoverySafetyPolicy::RecoveryBindingMode mode,
    EndpointRecoveryCandidate& selected,
    QString* error)
{
    const QString robotScope = robotName.trimmed();
    const QString endpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    if (!ValidateRobotName(robotScope, error)
        || endpoint.isEmpty()
        || endpoint != endpointIdentity.trimmed()
        || expected.robotName != robotScope
        || expected.robotEndpoint != endpoint)
    {
        SetError(error, QStringLiteral("恢复绑定请求的机器人 scope、端点或期望记录身份无效。"));
        return false;
    }

    QVector<EndpointRecoveryCandidate> candidates;
    if (!CollectEndpointRecoveryCandidatesLocked(endpoint, candidates, error))
    {
        return false;
    }
    if (candidates.size() != 1 || candidates.front().robotScope != robotScope)
    {
        SetError(error, QString("物理端点 %1 存在 %2 条 pending 恢复记录或唯一记录不属于精确 scope %3，拒绝恢复。")
            .arg(endpoint).arg(candidates.size()).arg(robotScope));
        return false;
    }

    QString indexedRobot;
    QString indexError;
    const ConfigDatabase::ReadStatus indexStatus = ReadEndpointIndexLocked(
        endpoint, indexedRobot, &indexError);
    if (indexStatus == ConfigDatabase::ReadStatus::Error
        || (indexStatus == ConfigDatabase::ReadStatus::Found
            && indexedRobot != robotScope))
    {
        SetError(error, QStringLiteral("恢复绑定的端点索引不可读或未精确指向唯一机器人 scope：")
            + indexError);
        return false;
    }

    selected = candidates.front();
    QString expectedEncodeError;
    const QString expectedEncoded = WeldResumePlanner::EncodeRecord(expected, &expectedEncodeError);
    if (expectedEncoded.isEmpty()
        || expectedEncoded != selected.encoded
        || EncodedRecordSha256(expectedEncoded) != selected.encodedSha256)
    {
        SetError(error, QStringLiteral("恢复绑定取得时完整 RecordV2 或 SHA256 已变化。"));
        return false;
    }
    if (mode == RobotRecoverySafetyPolicy::RecoveryBindingMode::PausedResume)
    {
        if (selected.record.state != QStringLiteral("paused"))
        {
            SetError(error, QStringLiteral("专用 paused 恢复只接受 state=paused。"));
            return false;
        }
    }
    else if (!WeldResumePlanner::ValidateSafeRetreatRecoveryRecord(selected.record, error))
    {
        return false;
    }
    return true;
}

#if !defined(WELD_SAFETY_STORE_STORAGE_ONLY_TEST)

void ApplyTerminal(
    WeldResumePlanner::CheckpointRecord& record,
    const WeldExecutionTerminalResult& terminal)
{
    record.safetyObservedAtUtc = terminal.observedAtUtc.trimmed().isEmpty()
        ? QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)
        : terminal.observedAtUtc;
    record.safetyReason = terminal.reason.trimmed().isEmpty()
        ? QStringLiteral("焊接安全终态未提供原因。")
        : terminal.reason;
    record.safetyProgramCompleted =
        terminal.state != WeldExecutionTerminalState::Incomplete;
    record.safetyMoveSpeedMmPerMin = terminal.safeMoveSpeedMmPerMin;
    record.safeX = terminal.requiredSafePose.dX;
    record.safeY = terminal.requiredSafePose.dY;
    record.safeZ = terminal.requiredSafePose.dZ;
    record.safeRx = terminal.requiredSafePose.dRX;
    record.safeRy = terminal.requiredSafePose.dRY;
    record.safeRz = terminal.requiredSafePose.dRZ;
    record.safeBx = terminal.requiredSafePose.dBX;
    record.safeBy = terminal.requiredSafePose.dBY;
    record.safeBz = terminal.requiredSafePose.dBZ;
    record.terminalPoseValid = terminal.observedTerminalPoseValid;
    record.terminalX = terminal.observedTerminalPose.dX;
    record.terminalY = terminal.observedTerminalPose.dY;
    record.terminalZ = terminal.observedTerminalPose.dZ;
    record.terminalRx = terminal.observedTerminalPose.dRX;
    record.terminalRy = terminal.observedTerminalPose.dRY;
    record.terminalRz = terminal.observedTerminalPose.dRZ;
    record.terminalBx = terminal.observedTerminalPose.dBX;
    record.terminalBy = terminal.observedTerminalPose.dBY;
    record.terminalBz = terminal.observedTerminalPose.dBZ;
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);
}
#endif
}

QString WeldSafetyRecoveryStore::StorageLabel(const QString& robotName)
{
    return StoreLabel(robotName);
}

bool WeldSafetyRecoveryStore::ReadRecord(
    const QString& robotName,
    WeldResumePlanner::CheckpointRecord& record,
    QString* encoded,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    return ReadRecordLocked(robotName, record, encoded, error);
}

bool WeldSafetyRecoveryStore::WriteRecord(
    const QString& robotName,
    const QString& encoded,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    return WriteRecordLocked(robotName, encoded, error);
}

bool WeldSafetyRecoveryStore::DisableLegacy(const QString& robotName, QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    if (!ValidateRobotName(robotName, error)
        || !ConfigDatabase::WriteScopedSetting(
            QStringLiteral("robot"), robotName.trimmed(), QString::fromLatin1(kRecoveryModule),
            QStringLiteral("Valid"), QStringLiteral("0")))
    {
        SetError(error, QStringLiteral("关闭旧版断点标志失败。"));
        return false;
    }
    QString readback;
    if (ConfigDatabase::ReadScopedSettingStatus(
            QStringLiteral("robot"), robotName.trimmed(), QString::fromLatin1(kRecoveryModule),
            QStringLiteral("Valid"), &readback)
            != ConfigDatabase::ReadStatus::Found
        || readback != QStringLiteral("0"))
    {
        SetError(error, QStringLiteral("旧版断点标志写后回读不一致，保持失败关闭。"));
        return false;
    }
    return true;
}

bool WeldSafetyRecoveryStore::ReadPending(
    const QString& robotName,
    bool& pending,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    return ReadPendingLocked(robotName, pending, error);
}

bool WeldSafetyRecoveryStore::BeginOrUpdatePending(
    const QString& robotName,
    const QString& encoded,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    // 端点索引先落盘并回读；随后 marker 先于 RecordV2。任何中途退出都会留下
    // 可由别名命中的不完整状态，准入逻辑只能失败关闭。
    return EnsureEndpointIndexForRecordLocked(robotName, encoded, error)
        && WritePendingLocked(robotName, true, error)
        && WriteRecordLocked(robotName, encoded, error);
}

bool WeldSafetyRecoveryStore::WriteCompletedAndClearPending(
    const QString& robotName,
    const QString& encoded,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    // 完整终态先写，确认回读后才允许清 marker。
    return EnsureEndpointIndexForRecordLocked(robotName, encoded, error)
        && WriteRecordLocked(robotName, encoded, error)
        && WritePendingLocked(robotName, false, error);
}

bool WeldSafetyRecoveryStore::InvalidateIfNoPending(const QString& robotName, QString& error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    bool pending = false;
    if (!ReadPendingLocked(robotName, pending, &error))
    {
        return false;
    }
    if (pending)
    {
        error = QStringLiteral(
            "该机器人仍有未验证完成的焊后安全回撤。禁止使记录失效、重新扫描、自动运动或再次执行焊缝；"
            "请使用“焊后安全回撤恢复”，确认到达记录绑定的收枪安全位后再继续。");
        return false;
    }
    const QString invalidated = QString("invalidated:v2:%1")
        .arg(QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    if (!WriteRecordLocked(robotName, invalidated, &error)
        || !WritePendingLocked(robotName, false, &error))
    {
        return false;
    }
    return DisableLegacy(robotName, &error);
}

bool WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
    const QString& robotName,
    const QString& endpointIdentity,
    QString* reason)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    const QString normalizedRobot = robotName.trimmed();
    const QString endpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    QString validationError;
    if (!ValidateRobotName(normalizedRobot, &validationError)
        || endpoint.isEmpty()
        || endpoint != endpointIdentity.trimmed())
    {
        SetError(reason, QStringLiteral("机器人名称或持久端点无效，无法验证焊后安全回撤门禁：") + validationError);
        return true;
    }

    QString indexedRobot;
    QString indexError;
    const ConfigDatabase::ReadStatus indexStatus = ReadEndpointIndexLocked(
        endpoint, indexedRobot, &indexError);
    if (indexStatus == ConfigDatabase::ReadStatus::Error)
    {
        SetError(reason, QStringLiteral("无法验证焊接安全端点索引，已拒绝新的机器人操作：") + indexError);
        return true;
    }
    if (indexStatus == ConfigDatabase::ReadStatus::Found
        && ProbeRobotAdmissionBlockedLocked(indexedRobot, endpoint, true, true, reason))
    {
        return true;
    }

    if (indexedRobot.compare(normalizedRobot, Qt::CaseInsensitive) != 0
        && ProbeRobotAdmissionBlockedLocked(normalizedRobot, endpoint, false, true, reason))
    {
        return true;
    }

    QStringList storedRobots;
    if (!ConfigDatabase::TryListScopedSettingIdsBounded(
            QStringLiteral("robot"), QString::fromLatin1(kRecoveryModule),
            kMaxStoredRobotRecoveryRecords, kMaxRobotNameLength, &storedRobots))
    {
        SetError(reason, QStringLiteral("无法枚举焊接恢复记录以核对物理端点，已拒绝新的机器人操作。"));
        return true;
    }
    for (const QString& storedRobot : storedRobots)
    {
        if (!AppPaths::IsSafePathComponent(storedRobot.trimmed()))
        {
            SetError(reason, QStringLiteral("枚举到无效机器人名称的焊接恢复记录，已失败关闭。"));
            return true;
        }
        // ConfigStore scope_id 区分大小写。当前查询 robota 未命中并不代表 RobotA
        // 不存在；这里只能跳过已经按完全相同 scope 实际探测过的条目。
        if (storedRobot == normalizedRobot || storedRobot == indexedRobot)
        {
            continue;
        }
        if (ProbeRobotAdmissionBlockedLocked(storedRobot, endpoint, false, false, reason))
        {
            return true;
        }
    }
    return false;
}

bool WeldSafetyRecoveryStore::ReadPausedResumeBinding(
    const QString& robotName,
    const QString& endpointIdentity,
    const WeldResumePlanner::CheckpointRecord& expected,
    WeldResumePlanner::CheckpointRecord* current,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    const QString normalizedEndpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    if (current == nullptr
        || !ValidateRobotName(robotName, error)
        || normalizedEndpoint.isEmpty()
        || normalizedEndpoint != endpointIdentity.trimmed()
        || expected.robotName.compare(robotName.trimmed(), Qt::CaseInsensitive) != 0
        || expected.robotEndpoint != normalizedEndpoint)
    {
        SetError(error, QStringLiteral("paused 恢复绑定参数无效，拒绝取得恢复租约。"));
        return false;
    }
    bool pending = false;
    if (!ReadPendingLocked(robotName, pending, error) || !pending)
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = QStringLiteral("paused 恢复要求持久 SafeRetreatPending 仍为 1。");
        }
        return false;
    }

    WeldResumePlanner::CheckpointRecord observed;
    QString encoded;
    if (!ReadRecordLocked(robotName, observed, &encoded, error))
    {
        return false;
    }
    const auto identity = [](const WeldResumePlanner::CheckpointRecord& record)
        {
            RobotRecoverySafetyPolicy::PausedResumeIdentity value;
            value.state = record.state;
            value.checkpointId = record.checkpointId;
            value.robotName = record.robotName;
            value.robotEndpoint = record.robotEndpoint;
            value.programName = record.programName;
            value.trajectoryRelativePath = record.trajectoryRelativePath;
            value.trajectorySha256 = record.trajectorySha256;
            value.trajectorySize = record.trajectorySize;
            return value;
        };
    QString expectedEncodeError;
    QString observedEncodeError;
    const QString expectedEncoded = WeldResumePlanner::EncodeRecord(expected, &expectedEncodeError);
    const QString observedEncoded = WeldResumePlanner::EncodeRecord(observed, &observedEncodeError);
    if (observed.robotName.compare(robotName.trimmed(), Qt::CaseInsensitive) != 0
        || observed.robotEndpoint != normalizedEndpoint
        || !RobotRecoverySafetyPolicy::SamePausedResumeIdentity(identity(expected), identity(observed))
        || expectedEncoded.isEmpty()
        || observedEncoded.isEmpty()
        || observedEncoded != expectedEncoded)
    {
        SetError(error, QStringLiteral(
            "paused 恢复租约取得时 RecordV2 的 checkpoint、端点、程序、轨迹或 state 已变化，拒绝恢复。"));
        return false;
    }
    *current = observed;
    if (error != nullptr)
    {
        error->clear();
    }
    return true;
}

bool WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
    const QString& robotName,
    const QString& endpointIdentity,
    const WeldResumePlanner::CheckpointRecord& expected,
    RobotRecoverySafetyPolicy::RecoveryBindingMode mode,
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding* binding,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    if (binding == nullptr)
    {
        SetError(error, QStringLiteral("恢复绑定输出为空。"));
        return false;
    }
    *binding = {};
    const QString endpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    if (endpoint.isEmpty() || endpoint != endpointIdentity.trimmed())
    {
        SetError(error, QStringLiteral("恢复绑定端点无效。"));
        return false;
    }
    if (g_activeEndpointRecoveryBindings.contains(endpoint))
    {
        SetError(error, QStringLiteral("同一物理端点已有持 Store 绑定的恢复流程。"));
        return false;
    }

    EndpointRecoveryCandidate selected;
    if (!ValidateExclusiveRecoverySnapshotLocked(
            robotName, endpoint, expected, mode, selected, error))
    {
        return false;
    }
    const QString token = QUuid::createUuid().toString(QUuid::WithoutBraces);
    g_activeEndpointRecoveryBindings.insert(endpoint, token);
    binding->token = token;
    binding->mode = mode;
    binding->robotScope = selected.robotScope;
    binding->endpointIdentity = endpoint;
    binding->encodedRecord = selected.encoded;
    binding->encodedSha256 = selected.encodedSha256;
    binding->record = selected.record;
    if (error != nullptr)
    {
        error->clear();
    }
    return true;
}

void WeldSafetyRecoveryStore::ReleaseExclusiveRecoveryBinding(
    const QString& endpointIdentity,
    const QString& token)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    const QString endpoint = NormalizePersistentEndpointIdentity(endpointIdentity);
    const auto it = g_activeEndpointRecoveryBindings.find(endpoint);
    if (it != g_activeEndpointRecoveryBindings.end() && it.value() == token)
    {
        g_activeEndpointRecoveryBindings.erase(it);
    }
}

bool WeldSafetyRecoveryStore::RevalidateExclusiveRecoveryBinding(
    const RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding& binding,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    if (!binding.IsValid()
        || g_activeEndpointRecoveryBindings.value(binding.endpointIdentity) != binding.token)
    {
        SetError(error, QStringLiteral("恢复端点独占绑定已失效。"));
        return false;
    }
    EndpointRecoveryCandidate selected;
    if (!ValidateExclusiveRecoverySnapshotLocked(
            binding.robotScope,
            binding.endpointIdentity,
            binding.record,
            binding.mode,
            selected,
            error))
    {
        return false;
    }
    if (selected.encoded != binding.encodedRecord
        || selected.encodedSha256 != binding.encodedSha256)
    {
        SetError(error, QStringLiteral("STOP/Kill 期间完整 RecordV2 或 SHA256 已变化，绑定失效。"));
        return false;
    }
    return true;
}

bool WeldSafetyRecoveryStore::TransitionBoundRecordState(
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding* binding,
    const QString& expectedState,
    const QString& newState,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    if (binding == nullptr
        || !binding->IsValid()
        || g_activeEndpointRecoveryBindings.value(binding->endpointIdentity) != binding->token)
    {
        SetError(error, QStringLiteral("绑定状态迁移缺少有效的端点独占 token。"));
        return false;
    }
    EndpointRecoveryCandidate selected;
    if (!ValidateExclusiveRecoverySnapshotLocked(
            binding->robotScope,
            binding->endpointIdentity,
            binding->record,
            binding->mode,
            selected,
            error)
        || selected.encoded != binding->encodedRecord
        || selected.encodedSha256 != binding->encodedSha256
        || selected.record.state != expectedState
        || binding->record.state != expectedState)
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = QStringLiteral("绑定 RecordV2 完整内容/SHA256/state CAS 失败。");
        }
        return false;
    }

    WeldResumePlanner::CheckpointRecord updated = selected.record;
    updated.state = newState;
    QString encodeError;
    const QString updatedEncoded = WeldResumePlanner::EncodeRecord(updated, &encodeError);
    if (updatedEncoded.isEmpty()
        || !WriteRecordLocked(binding->robotScope, updatedEncoded, error))
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = encodeError;
        }
        return false;
    }
    binding->record = updated;
    binding->encodedRecord = updatedEncoded;
    binding->encodedSha256 = EncodedRecordSha256(updatedEncoded);
    return true;
}

bool WeldSafetyRecoveryStore::TransitionRecordState(
    const QString& robotName,
    const QString& expectedCheckpointId,
    const QString& expectedState,
    const QString& newState,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_storeMutex);
    WeldResumePlanner::CheckpointRecord record;
    if (!ReadRecordLocked(robotName, record, nullptr, error))
    {
        return false;
    }
    if (record.checkpointId != expectedCheckpointId || record.state != expectedState)
    {
        SetError(error, QString("断点状态已变化，拒绝覆盖。Expected=%1/%2 Current=%3/%4")
            .arg(expectedCheckpointId, expectedState, record.checkpointId, record.state));
        return false;
    }
    record.state = newState;
    QString encodeError;
    const QString encoded = WeldResumePlanner::EncodeRecord(record, &encodeError);
    if (encoded.isEmpty() || !WriteRecordLocked(robotName, encoded, error))
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = encodeError;
        }
        return false;
    }
    return true;
}

#if !defined(WELD_SAFETY_STORE_STORAGE_ONLY_TEST)
WeldSafetyRecoverySession::WeldSafetyRecoverySession(
    RobotDriverAdaptor* driver,
    const T_PRECISE_MEASURE_PARAM& param)
    : m_driver(driver), m_param(param)
{
}

bool WeldSafetyRecoverySession::Prepare(
    const MeasureThenWeldService::WeldExecutionIdentity& identity,
    QString& error)
{
    error.clear();
    if (m_driver == nullptr || identity.programName.trimmed().isEmpty())
    {
        error = QStringLiteral("共享焊接安全会话缺少机器人或程序身份。");
        return false;
    }
    WeldResumePlanner::CheckpointRecord record;
    record.state = QStringLiteral("prepared");
    record.checkpointId = QUuid::createUuid().toString(QUuid::WithoutBraces);
    record.createdAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    record.robotName = QString::fromStdString(m_param.sRobotName).trimmed();
    if (record.robotName.isEmpty())
    {
        record.robotName = QString::fromStdString(m_driver->RobotName()).trimmed();
    }
    record.robotType = QString::fromStdString(m_driver->DriverDescriptor().typeName);
    record.robotEndpoint = RobotOperationLease::PersistentEndpointIdentity(m_driver);
    record.paramGroupIndex = m_param.nParamGroupIndex;
    record.paramGroupName = m_param.sParamGroupName;
    record.scanSection = QString::fromStdString(m_param.sSectionName);
    record.weldSection = QString::fromStdString(m_param.sWeldSectionName);
    record.parameterFingerprint = identity.parameterFingerprint;
    record.programName = identity.programName;
    record.weldDirection = m_param.nWeldDirection < 0 ? -1 : 1;
    record.actualWeld = m_param.bDoActualWeld;
    record.finalStepMm = identity.effectiveFinalStepMm;
    record.backtrackMm = m_param.dResumeBacktrackMm;
    const QString projectRoot = RobotDataHelper::FindProjectRootPath();
    bool trajectoryBound = false;
    if (record.robotType != QStringLiteral("UNKNOWN") && !record.robotEndpoint.isEmpty())
    {
        trajectoryBound = WeldResumePlanner::BindTrajectoryIdentity(
            projectRoot, identity.sampledPosePath, record.robotName, record, &error);
        if (!trajectoryBound)
        {
            // VirtualWeldTest 的真实运动输入位于 Result/<robot>/VirtualWeld_*/<FinalSampled>，
            // 不伪装成生产 LaserPoint 结构，但仍由 Service 的不可变快照 SHA/size/pointCount 绑定。
            const QFileInfo sampledInfo(identity.sampledPosePath);
            const QString relative = QDir::cleanPath(QDir(projectRoot).relativeFilePath(
                sampledInfo.canonicalFilePath().isEmpty()
                    ? sampledInfo.absoluteFilePath() : sampledInfo.canonicalFilePath()));
            const QStringList parts = QDir::fromNativeSeparators(relative).split(
                QLatin1Char('/'), Qt::SkipEmptyParts);
            QString hashError;
            const QString actualSha = WeldResumePlanner::ComputeFileSha256(
                sampledInfo.absoluteFilePath(), &hashError);
            if (sampledInfo.isFile()
                && parts.size() == 4
                && parts[0].compare(QStringLiteral("Result"), Qt::CaseInsensitive) == 0
                && parts[1].compare(record.robotName, Qt::CaseInsensitive) == 0
                && parts[2].startsWith(QStringLiteral("VirtualWeld_"), Qt::CaseInsensitive)
                && parts[3].endsWith(QStringLiteral("_FinalSampled.txt"), Qt::CaseInsensitive)
                && actualSha.compare(identity.sampledPoseSha256, Qt::CaseInsensitive) == 0
                && sampledInfo.size() == identity.sampledPoseSize
                && identity.sampledPointCount >= 2)
            {
                record.caseId = parts[2];
                record.caseRelativeDir = parts.mid(0, 3).join(QLatin1Char('/'));
                record.trajectoryRelativePath = parts.join(QLatin1Char('/'));
                record.trajectorySha256 = actualSha.toLower();
                record.trajectorySize = sampledInfo.size();
                record.trajectoryPointCount = identity.sampledPointCount;
                record.trajectoryInExecutionOrder = true;
                trajectoryBound = true;
                error.clear();
            }
        }
    }
    if (!trajectoryBound
        || record.trajectorySha256.compare(identity.sampledPoseSha256, Qt::CaseInsensitive) != 0
        || record.trajectorySize != identity.sampledPoseSize)
    {
        if (error.isEmpty())
        {
            error = QStringLiteral("共享焊接安全会话的端点或实际执行轨迹身份无效。");
        }
        return false;
    }
    record.safetyObservedAtUtc = record.createdAtUtc;
    record.safetyReason = QStringLiteral("共享入口已在START前锁存焊后安全回撤门禁。");
    record.safetyProgramCompleted = false;
    record.safetyMoveSpeedMmPerMin = identity.safeMoveSpeedMmPerMin;
    record.safeX = identity.requiredEndSafePose.dX;
    record.safeY = identity.requiredEndSafePose.dY;
    record.safeZ = identity.requiredEndSafePose.dZ;
    record.safeRx = identity.requiredEndSafePose.dRX;
    record.safeRy = identity.requiredEndSafePose.dRY;
    record.safeRz = identity.requiredEndSafePose.dRZ;
    record.safeBx = identity.requiredEndSafePose.dBX;
    record.safeBy = identity.requiredEndSafePose.dBY;
    record.safeBz = identity.requiredEndSafePose.dBZ;
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);
    const QString encoded = WeldResumePlanner::EncodeRecord(record, &error);
    if (encoded.isEmpty()
        || !WeldSafetyRecoveryStore::BeginOrUpdatePending(record.robotName, encoded, &error)
        || !WeldSafetyRecoveryStore::DisableLegacy(record.robotName, &error))
    {
        return false;
    }
    m_record = record;
    m_prepared = true;
    return true;
}

bool WeldSafetyRecoverySession::Finish(
    const WeldExecutionTerminalResult& terminal,
    QString& error)
{
    error.clear();
    if (!m_prepared)
    {
        error = QStringLiteral("共享焊接安全会话尚未准备，禁止写入终态。");
        return false;
    }
    ApplyTerminal(m_record, terminal);
    if (terminal.state == WeldExecutionTerminalState::Incomplete)
    {
        m_record.state = terminal.programStartAttempted
            ? QStringLiteral("interrupted") : QStringLiteral("superseded");
    }
    else if (terminal.state == WeldExecutionTerminalState::ProgramCompletedUnretracted)
    {
        m_record.state = QStringLiteral("unretracted");
    }
    else
    {
        m_record.state = QStringLiteral("finished");
    }
    const QString encoded = WeldResumePlanner::EncodeRecord(m_record, &error);
    if (encoded.isEmpty())
    {
        return false;
    }
    if (terminal.state == WeldExecutionTerminalState::SafelyRetracted
        || (terminal.state == WeldExecutionTerminalState::Incomplete
            && !terminal.programStartAttempted))
    {
        const bool ok = WeldSafetyRecoveryStore::WriteCompletedAndClearPending(
            m_record.robotName, encoded, &error);
        if (ok)
        {
            m_prepared = false;
        }
        return ok;
    }
    return WeldSafetyRecoveryStore::BeginOrUpdatePending(m_record.robotName, encoded, &error);
}
#endif
