#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

class WeldResumePlanner
{
public:
    static constexpr int SchemaVersion = 2;

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

    static QString ComputeFileSha256(const QString& filePath, QString* error = nullptr);
    static QString BuildParameterFingerprint(const QStringList& orderedFields);

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
};
