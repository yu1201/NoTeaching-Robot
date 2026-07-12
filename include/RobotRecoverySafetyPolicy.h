#pragma once

#include <QByteArray>
#include <QCryptographicHash>
#include <QString>

#include "WeldResumePlanner.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace RobotRecoverySafetyPolicy
{
enum class ObservedProgramState
{
    Stopped,
    Running,
    Paused,
    Unknown
};

enum class PersistedProgramAction
{
    Reject,
    AlreadyStopped,
    StopThenKill,
    KillLoaded
};

inline PersistedProgramAction ResolveStepPersistedProgramAction(
    const QString& expectedProgram,
    const QString& currentProgram,
    ObservedProgramState state,
    bool emptyProgramStableStopped)
{
    const QString expected = expectedProgram.trimmed();
    const QString current = currentProgram.trimmed();
    if (expected.isEmpty())
    {
        return PersistedProgramAction::Reject;
    }
    if (current.isEmpty())
    {
        return emptyProgramStableStopped
            ? PersistedProgramAction::AlreadyStopped
            : PersistedProgramAction::Reject;
    }
    if (current != expected)
    {
        return PersistedProgramAction::Reject;
    }
    if (state == ObservedProgramState::Running)
    {
        return PersistedProgramAction::StopThenKill;
    }
    if (state == ObservedProgramState::Paused || state == ObservedProgramState::Stopped)
    {
        return PersistedProgramAction::KillLoaded;
    }
    return PersistedProgramAction::Reject;
}

inline bool MayMoveForSafeRetreat(
    const QString& persistedState,
    bool safetyProgramCompleted,
    bool persistedProgramTerminationVerified)
{
    if (persistedState == QStringLiteral("unretracted"))
    {
        return safetyProgramCompleted;
    }
    if (persistedState == QStringLiteral("interrupted"))
    {
        return persistedProgramTerminationVerified;
    }
    return false;
}

struct PausedResumeIdentity
{
    QString state;
    QString checkpointId;
    QString robotName;
    QString robotEndpoint;
    QString programName;
    QString trajectoryRelativePath;
    QString trajectorySha256;
    qint64 trajectorySize = -1;
};

inline bool SamePausedResumeIdentity(
    const PausedResumeIdentity& expected,
    const PausedResumeIdentity& current)
{
    return expected.state == QStringLiteral("paused")
        && current.state == QStringLiteral("paused")
        && !expected.checkpointId.trimmed().isEmpty()
        && current.checkpointId == expected.checkpointId
        && current.robotName.compare(expected.robotName, Qt::CaseInsensitive) == 0
        && !expected.robotEndpoint.trimmed().isEmpty()
        && current.robotEndpoint == expected.robotEndpoint
        && !expected.programName.trimmed().isEmpty()
        && current.programName == expected.programName
        && !expected.trajectoryRelativePath.trimmed().isEmpty()
        && current.trajectoryRelativePath == expected.trajectoryRelativePath
        && expected.trajectorySha256.size() == 64
        && current.trajectorySha256 == expected.trajectorySha256
        && expected.trajectorySize >= 0
        && current.trajectorySize == expected.trajectorySize;
}

struct ProgramContentIdentity
{
    qint64 size = -1;
    QByteArray sha256;

    bool IsValid() const
    {
        return size >= 0 && sha256.size() == 64;
    }
};

inline bool SameProgramContent(
    const ProgramContentIdentity& expected,
    const ProgramContentIdentity& observed)
{
    return expected.IsValid()
        && observed.IsValid()
        && expected.size == observed.size
        && expected.sha256 == observed.sha256;
}

struct ProgramContentWitnessSnapshot
{
    std::uint64_t generation = 0;
    std::string projectName;
    std::string programName;
    std::string remoteProgramPath;
    std::string remoteDataPath;
    ProgramContentIdentity programIdentity;
    ProgramContentIdentity dataIdentity;
    std::string observedProjectName;
    std::string observedProgramName;
    int observedProgramState = -1;

    bool IsValid() const
    {
        return generation != 0
            && !projectName.empty()
            && !programName.empty()
            && !remoteProgramPath.empty()
            && !remoteDataPath.empty()
            && programIdentity.IsValid()
            && dataIdentity.IsValid();
    }
};

inline bool SameProgramContentWitness(
    const ProgramContentWitnessSnapshot& expected,
    const ProgramContentWitnessSnapshot& current)
{
    return expected.IsValid()
        && current.IsValid()
        && expected.generation == current.generation
        && expected.projectName == current.projectName
        && expected.programName == current.programName
        && expected.remoteProgramPath == current.remoteProgramPath
        && expected.remoteDataPath == current.remoteDataPath
        && SameProgramContent(expected.programIdentity, current.programIdentity)
        && SameProgramContent(expected.dataIdentity, current.dataIdentity);
}

// FTP 内容校验不持机器人 SDK mutex。红色 STOP 可通过此独立门立即锁存取消，
// 随后取得 SDK mutex 执行 STOP/Kill；校验线程回锁时还必须复核完整 witness 快照。
class RemoteContentVerificationGate final
{
public:
    using Token = std::shared_ptr<std::atomic<bool>>;

    Token TryBegin()
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (m_active != nullptr)
        {
            return {};
        }
        m_active = std::make_shared<std::atomic<bool>>(false);
        return m_active;
    }

    void End(const Token& token)
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (m_active == token)
        {
            m_active.reset();
        }
    }

    void CancelActive()
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (m_active != nullptr)
        {
            m_active->store(true, std::memory_order_release);
        }
    }

private:
    std::mutex m_mutex;
    Token m_active;
};

enum class RecoveryBindingMode
{
    SafeRetreat,
    PausedResume
};

struct ExclusiveRecoveryBinding
{
    QString token;
    RecoveryBindingMode mode = RecoveryBindingMode::SafeRetreat;
    QString robotScope;
    QString endpointIdentity;
    QString encodedRecord;
    QString encodedSha256;
    WeldResumePlanner::CheckpointRecord record;

    bool IsValid() const
    {
        return !token.isEmpty()
            && !robotScope.isEmpty()
            && !endpointIdentity.isEmpty()
            && !encodedRecord.isEmpty()
            && encodedSha256.size() == 64
            && !record.checkpointId.isEmpty();
    }
};

inline ProgramContentIdentity ContentIdentity(const QByteArray& content)
{
    ProgramContentIdentity identity;
    identity.size = content.size();
    identity.sha256 = QCryptographicHash::hash(content, QCryptographicHash::Sha256).toHex().toLower();
    return identity;
}

inline bool ProgramContentWithinLimit(
    const ProgramContentIdentity& identity,
    qint64 maximumBytes)
{
    return identity.IsValid()
        && identity.size > 0
        && maximumBytes > 0
        && identity.size <= maximumBytes;
}
}
