#pragma once

#include <QDateTime>
#include <QString>
#include <QStringList>
#include <QVector>

class WeldResumePlanner
{
public:
    static constexpr int SchemaVersion = 2;
    static constexpr qint64 MaxResumeCheckpointAgeSeconds = 24 * 60 * 60;
    static constexpr qint64 MaxFutureClockSkewSeconds = 5 * 60;
    static constexpr qint64 MaxExecutionTrajectoryBytes = 256LL * 1024 * 1024;
    static constexpr int MaxExecutionTrajectoryPoints = 2'000'000;
    static constexpr int MaxExecutionTrajectoryLines = MaxExecutionTrajectoryPoints + 2;
    static constexpr int MaxExecutionTrajectoryLineBytes = 64 * 1024;

    struct CheckpointRecord
    {
        int schemaVersion = SchemaVersion;
        QString state = QStringLiteral("prepared");
        QString checkpointId;
        QString createdAtUtc;

        QString robotName;
        QString robotType;
        QString robotEndpoint;

        int paramGroupIndex = -1;
        QString paramGroupName;
        QString scanSection;
        QString weldSection;
        QString parameterFingerprint;

        QString caseId;
        QString caseRelativeDir;
        QString sourceTrajectoryRelativePath;
        QString sourceTrajectorySha256;
        QString trajectoryRelativePath;
        QString trajectorySha256;
        qint64 trajectorySize = -1;
        int trajectoryPointCount = 0;
        bool trajectoryInExecutionOrder = true;

        QString programName;
        int programLine = -1;
        int weldDirection = 1;
        bool actualWeld = false;
        double finalStepMm = 0.0;
        double backtrackMm = 5.0;

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;

        // 焊接 START 前即持久化的安全回撤门禁。程序完成后、收枪尚未验证时 state=unretracted；
        // interrupted 表示 START 已尝试但程序终态未知。两者都禁止重新执行焊缝。
        QString safetyObservedAtUtc;
        QString safetyReason;
        bool safetyProgramCompleted = false;
        double safetyMoveSpeedMmPerMin = 0.0;
        double safeX = 0.0;
        double safeY = 0.0;
        double safeZ = 0.0;
        double safeRx = 0.0;
        double safeRy = 0.0;
        double safeRz = 0.0;
        double safeBx = 0.0;
        double safeBy = 0.0;
        double safeBz = 0.0;
        bool terminalPoseValid = false;
        double terminalX = 0.0;
        double terminalY = 0.0;
        double terminalZ = 0.0;
        double terminalRx = 0.0;
        double terminalRy = 0.0;
        double terminalRz = 0.0;
        double terminalBx = 0.0;
        double terminalBy = 0.0;
        double terminalBz = 0.0;
        QString safetyWitnessSha256;
    };

    struct TrajectoryPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct ResumePlan
    {
        QVector<TrajectoryPoint> points;
        int sourcePointCount = 0;
        int matchedSegmentIndex = -1;
        double matchedSegmentRatio = 0.0;
        double matchDistanceMm = 0.0;
        double pauseArcMm = 0.0;
        double resumeArcMm = 0.0;
        double actualBacktrackMm = 0.0;
        double totalArcMm = 0.0;
    };

    static QString EncodeRecord(const CheckpointRecord& record, QString* error = nullptr);
    static bool DecodeRecord(const QString& encoded, CheckpointRecord& record, QString* error = nullptr);
    // 自动续焊只接受最近 24 小时内形成的断点，并拒绝超出小幅时钟漂移的未来时间。
    // currentUtc 显式传入以便测试可重复；生产调用传 QDateTime::currentDateTimeUtc()。
    static bool ValidateCheckpointTime(
        const CheckpointRecord& record,
        const QDateTime& currentUtc,
        QString* error = nullptr);

    static QString ComputeFileSha256(const QString& filePath, QString* error = nullptr);
    static QString BuildParameterFingerprint(const QStringList& orderedFields);
    static QString BuildSafeRetreatWitness(const CheckpointRecord& record);
    static bool ValidateSafeRetreatRecoveryRecord(
        const CheckpointRecord& record,
        QString* error = nullptr);

    static bool BindTrajectoryIdentity(
        const QString& projectRoot,
        const QString& trajectoryPath,
        const QString& robotName,
        CheckpointRecord& record,
        QString* error = nullptr);

    static bool ResolveBoundTrajectory(
        const QString& projectRoot,
        const QString& expectedRobotName,
        const CheckpointRecord& record,
        QString& trajectoryPath,
        QString* error = nullptr);

    static bool LoadExecutionTrajectory(
        const QString& trajectoryPath,
        QVector<TrajectoryPoint>& points,
        QString* error = nullptr);

    static bool PlanFromPausedPose(
        const QString& trajectoryPath,
        double pauseX,
        double pauseY,
        double pauseZ,
        double backtrackMm,
        ResumePlan& plan,
        QString* error = nullptr);

    // 续焊生产入口必须在同一次有界内存快照上同时校验 SHA/大小/点数并规划，
    // 防止 Resolve、解析和最终执行之间使用不同版本的轨迹文件。
    static bool PlanFromPausedPoseBound(
        const QString& trajectoryPath,
        const CheckpointRecord& expectedIdentity,
        double pauseX,
        double pauseY,
        double pauseZ,
        double backtrackMm,
        ResumePlan& plan,
        QString* error = nullptr);
};
