#include "WeldResumePlanner.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QTextStream>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
constexpr double EPSILON = 1e-6;

void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void CheckNear(double actual, double expected, const char* message)
{
    if (std::abs(actual - expected) > EPSILON)
    {
        std::cerr << "FAIL: " << message
            << " (actual=" << actual << ", expected=" << expected << ")\n";
        std::exit(1);
    }
}

QString WriteTrajectory(
    const QString& root,
    const QString& robot,
    const QString& runCase,
    const QString& fileName,
    const QVector<WeldResumePlanner::TrajectoryPoint>& points)
{
    const QString directory = QDir(root).filePath(
        QStringLiteral("Result/%1/%2/LaserPoint").arg(robot, runCase));
    Check(QDir().mkpath(directory), "could not create trajectory directory");

    const QString path = QDir(directory).filePath(fileName);
    QFile file(path);
    Check(file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate),
        "could not create trajectory file");
    QTextStream stream(&file);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(9);
    stream << "# execution trajectory\n";
    stream << "weld_index raw_index X Y Z RX RY RZ speed\n";
    for (int index = 0; index < points.size(); ++index)
    {
        // The deliberately large indices prove that columns 2..4, rather than
        // columns 0..2, are interpreted as XYZ.
        stream << (1000 + index) << ' ' << (9000 + index) << ' '
            << points[index].x << ' ' << points[index].y << ' ' << points[index].z
            << " 180 0 90 12\n";
    }
    file.close();
    return QFileInfo(path).absoluteFilePath();
}

WeldResumePlanner::CheckpointRecord CompleteRecord(
    const QString& root,
    const QString& trajectoryPath)
{
    WeldResumePlanner::CheckpointRecord record;
    record.checkpointId = QStringLiteral("checkpoint-001");
    record.createdAtUtc = QStringLiteral("2026-07-11T01:02:03.000Z");
    record.robotType = QStringLiteral("STEP");
    record.robotEndpoint = QStringLiteral("tcp:[192.168.10.10]:8193");
    record.paramGroupIndex = 2;
    record.paramGroupName = QStringLiteral("Group 3");
    record.scanSection = QStringLiteral("PreciseMeasureParam3");
    record.weldSection = QStringLiteral("PreciseWeldParam3");
    record.parameterFingerprint = WeldResumePlanner::BuildParameterFingerprint(
        { QStringLiteral("robot=RobotA"), QStringLiteral("speed=12.5") });
    record.programName = QStringLiteral("PROGRAM_001");
    record.programLine = 42;
    record.weldDirection = -1;
    record.actualWeld = true;
    record.finalStepMm = 2.0;
    record.backtrackMm = 5.0;
    record.x = 12.0;
    record.y = 3.0;
    record.z = 4.0;
    record.rx = 180.0;
    record.ry = 0.0;
    record.rz = 90.0;

    QString error;
    Check(WeldResumePlanner::BindTrajectoryIdentity(
        root, trajectoryPath, QStringLiteral("RobotA"), record, &error),
        qPrintable(QStringLiteral("BindTrajectoryIdentity failed: %1").arg(error)));
    return record;
}

void TestColumnParsingAndRecordRoundTrip(const QString& root)
{
    const QString path = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseColumns"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 1.25, 2.5, 3.75 }, { 11.25, 12.5, 13.75 } });

    QVector<WeldResumePlanner::TrajectoryPoint> points;
    QString error;
    Check(WeldResumePlanner::LoadExecutionTrajectory(path, points, &error),
        qPrintable(QStringLiteral("LoadExecutionTrajectory failed: %1").arg(error)));
    Check(points.size() == 2, "trajectory parser returned wrong point count");
    CheckNear(points[0].x, 1.25, "trajectory parser used wrong X column");
    CheckNear(points[0].y, 2.5, "trajectory parser used wrong Y column");
    CheckNear(points[0].z, 3.75, "trajectory parser used wrong Z column");

    WeldResumePlanner::CheckpointRecord source = CompleteRecord(root, path);
    const QString encoded = WeldResumePlanner::EncodeRecord(source, &error);
    Check(encoded.startsWith(QStringLiteral("b64:v2:")), "record did not use V2 envelope");

    WeldResumePlanner::CheckpointRecord decoded;
    Check(WeldResumePlanner::DecodeRecord(encoded, decoded, &error),
        qPrintable(QStringLiteral("DecodeRecord failed: %1").arg(error)));
    Check(decoded.schemaVersion == WeldResumePlanner::SchemaVersion,
        "record schema version was not preserved");
    Check(decoded.checkpointId == source.checkpointId,
        "record checkpoint identity was not preserved");
    Check(decoded.robotEndpoint == source.robotEndpoint,
        "record robot endpoint was not preserved");
    Check(decoded.caseId == QStringLiteral("CaseColumns"),
        "bound case identity was not preserved");
    Check(decoded.trajectoryRelativePath == source.trajectoryRelativePath,
        "bound trajectory path was not preserved");
    Check(decoded.trajectorySha256 == source.trajectorySha256,
        "bound trajectory hash was not preserved");
    Check(decoded.trajectoryPointCount == 2,
        "bound trajectory point count was not preserved");
    Check(decoded.weldDirection == -1 && decoded.actualWeld,
        "execution parameters were not preserved");
    CheckNear(decoded.backtrackMm, 5.0, "backtrack distance was not preserved");

    Check(!WeldResumePlanner::DecodeRecord(
        QStringLiteral("b64:v1:not-a-v2-record"), decoded, &error),
        "legacy/non-V2 record was accepted");
    Check(!WeldResumePlanner::DecodeRecord(
        QStringLiteral("b64:v2:not-json"), decoded, &error),
        "malformed V2 record was accepted");
}

void TestPathBindingAndTamperDetection(const QString& root)
{
    const QString validPath = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseBound"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_Resume_abc_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 }, { 20.0, 0.0, 0.0 } });
    WeldResumePlanner::CheckpointRecord record = CompleteRecord(root, validPath);

    QString resolved;
    QString error;
    Check(WeldResumePlanner::ResolveBoundTrajectory(
        root, QStringLiteral("RobotA"), record, resolved, &error),
        qPrintable(QStringLiteral("ResolveBoundTrajectory failed: %1").arg(error)));
    Check(QFileInfo(resolved) == QFileInfo(validPath),
        "resolved trajectory differs from bound trajectory");

    Check(!WeldResumePlanner::ResolveBoundTrajectory(
        root, QStringLiteral("RobotB"), record, resolved, &error),
        "trajectory bound to RobotA resolved for RobotB");

    const QString wrongName = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseWrongName"),
        QStringLiteral("arbitrary_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } });
    WeldResumePlanner::CheckpointRecord wrongRecord;
    Check(!WeldResumePlanner::BindTrajectoryIdentity(
        root, wrongName, QStringLiteral("RobotA"), wrongRecord, &error),
        "unexpected trajectory filename was accepted");

    QTemporaryDir outside;
    Check(outside.isValid(), "could not create outside-path fixture");
    const QString outsidePath = WriteTrajectory(
        outside.path(),
        QStringLiteral("RobotA"),
        QStringLiteral("Outside"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } });
    Check(!WeldResumePlanner::BindTrajectoryIdentity(
        root, outsidePath, QStringLiteral("RobotA"), wrongRecord, &error),
        "trajectory outside project root was accepted");

    QFile tampered(validPath);
    Check(tampered.open(QIODevice::ReadWrite), "could not mutate bound trajectory");
    QByteArray bytes = tampered.readAll();
    const QByteArray originalCoordinate("10.000000000");
    const QByteArray replacementCoordinate("11.000000000");
    const qsizetype coordinateOffset = bytes.indexOf(originalCoordinate);
    Check(coordinateOffset >= 0, "could not locate same-size tamper fixture");
    bytes.replace(coordinateOffset, originalCoordinate.size(), replacementCoordinate);
    Check(bytes.size() == record.trajectorySize,
        "same-size tamper fixture unexpectedly changed file length");
    Check(tampered.seek(0), "could not rewind bound trajectory for mutation");
    Check(tampered.write(bytes) == bytes.size(), "could not mutate bound trajectory bytes");
    tampered.close();
    Check(!WeldResumePlanner::ResolveBoundTrajectory(
        root, QStringLiteral("RobotA"), record, resolved, &error),
        "same-size/same-point-count trajectory mutation bypassed SHA256 binding");
}

void TestTrajectoryResourceBounds(const QString& root)
{
    const QString directory = QDir(root).filePath(
        QStringLiteral("Result/RobotA/CaseBounds/LaserPoint"));
    Check(QDir().mkpath(directory), "could not create trajectory bounds directory");

    const QString longLinePath = QDir(directory).filePath(
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_Long_FinalSampled.txt"));
    QFile longLine(longLinePath);
    Check(longLine.open(QIODevice::WriteOnly | QIODevice::Truncate),
        "could not create overlong trajectory line fixture");
    const QByteArray oversizedLine(WeldResumePlanner::MaxExecutionTrajectoryLineBytes + 1, '1');
    Check(longLine.write(oversizedLine) == oversizedLine.size(),
        "could not write overlong trajectory line fixture");
    longLine.close();

    QVector<WeldResumePlanner::TrajectoryPoint> points;
    QString error;
    Check(!WeldResumePlanner::LoadExecutionTrajectory(longLinePath, points, &error),
        "trajectory with an overlong line was accepted");

    const QString oversizedPath = QDir(directory).filePath(
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_Oversize_FinalSampled.txt"));
    QFile oversized(oversizedPath);
    Check(oversized.open(QIODevice::WriteOnly | QIODevice::Truncate),
        "could not create oversized trajectory fixture");
    Check(oversized.resize(WeldResumePlanner::MaxExecutionTrajectoryBytes + 1),
        "could not resize oversized trajectory fixture");
    oversized.close();
    Check(!WeldResumePlanner::LoadExecutionTrajectory(oversizedPath, points, &error),
        "trajectory exceeding the byte limit was accepted");
}

void TestBoundPlanningSnapshot(const QString& root)
{
    const QString path = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseBoundPlan"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 }, { 20.0, 0.0, 0.0 } });
    const WeldResumePlanner::CheckpointRecord record = CompleteRecord(root, path);

    WeldResumePlanner::ResumePlan plan;
    QString error;
    Check(WeldResumePlanner::PlanFromPausedPoseBound(
        path, record, 12.0, 0.0, 0.0, 5.0, plan, &error),
        qPrintable(QStringLiteral("bound snapshot plan failed: %1").arg(error)));
    CheckNear(plan.pauseArcMm, 12.0, "bound snapshot pause arc is wrong");
    CheckNear(plan.resumeArcMm, 7.0, "bound snapshot resume arc is wrong");

    QFile tampered(path);
    Check(tampered.open(QIODevice::ReadWrite), "could not mutate bound-plan trajectory");
    QByteArray bytes = tampered.readAll();
    const QByteArray originalCoordinate("10.000000000");
    const QByteArray replacementCoordinate("11.000000000");
    const qsizetype coordinateOffset = bytes.indexOf(originalCoordinate);
    Check(coordinateOffset >= 0, "could not locate bound-plan tamper fixture");
    bytes.replace(coordinateOffset, originalCoordinate.size(), replacementCoordinate);
    Check(bytes.size() == record.trajectorySize,
        "bound-plan same-size tamper unexpectedly changed file length");
    Check(tampered.seek(0), "could not rewind bound-plan trajectory");
    Check(tampered.write(bytes) == bytes.size(), "could not write bound-plan tamper");
    tampered.close();
    Check(!WeldResumePlanner::PlanFromPausedPoseBound(
        path, record, 12.0, 0.0, 0.0, 5.0, plan, &error),
        "same-size replacement between Resolve and Plan bypassed bound planning");

    WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseBoundPlan"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 }, { 20.0, 0.0, 0.0 } });

    WeldResumePlanner::CheckpointRecord wrongIdentity = record;
    wrongIdentity.trajectorySha256[0] = wrongIdentity.trajectorySha256[0] == QLatin1Char('0')
        ? QLatin1Char('1')
        : QLatin1Char('0');
    Check(!WeldResumePlanner::PlanFromPausedPoseBound(
        path, wrongIdentity, 12.0, 0.0, 0.0, 5.0, plan, &error),
        "bound planning accepted an incorrect SHA256");

    wrongIdentity = record;
    ++wrongIdentity.trajectorySize;
    Check(!WeldResumePlanner::PlanFromPausedPoseBound(
        path, wrongIdentity, 12.0, 0.0, 0.0, 5.0, plan, &error),
        "bound planning accepted an incorrect file size");

    wrongIdentity = record;
    ++wrongIdentity.trajectoryPointCount;
    Check(!WeldResumePlanner::PlanFromPausedPoseBound(
        path, wrongIdentity, 12.0, 0.0, 0.0, 5.0, plan, &error),
        "bound planning accepted an incorrect point count");

    Check(WeldResumePlanner::PlanFromPausedPoseBound(
        path, record, 12.0, 0.0, 0.0, 5.0, plan, &error),
        qPrintable(QStringLiteral("bound plan did not recover after restoring source: %1")
            .arg(error)));
}

void TestForwardAndReverseArcLength(const QString& root)
{
    const QString forward = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseForward"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 2.0, 0.0, 0.0 },
          { 10.0, 0.0, 0.0 }, { 25.0, 0.0, 0.0 } });
    WeldResumePlanner::ResumePlan plan;
    QString error;
    Check(WeldResumePlanner::PlanFromPausedPose(
        forward, 12.0, 0.0, 0.0, 5.0, plan, &error),
        qPrintable(QStringLiteral("forward resume plan failed: %1").arg(error)));
    CheckNear(plan.pauseArcMm, 12.0, "forward pause arc is not millimetre based");
    CheckNear(plan.resumeArcMm, 7.0, "forward backtrack arc is wrong");
    CheckNear(plan.actualBacktrackMm, 5.0, "forward actual backtrack is wrong");
    CheckNear(plan.totalArcMm, 25.0, "forward total arc is wrong");
    Check(plan.matchedSegmentIndex == 2, "forward pause matched wrong segment");

    // The planner consumes the persisted execution order. Reversing that order
    // must therefore measure arc length from the opposite physical endpoint.
    const QString reverse = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseReverse"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 25.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 },
          { 2.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } });
    Check(WeldResumePlanner::PlanFromPausedPose(
        reverse, 12.0, 0.0, 0.0, 5.0, plan, &error),
        qPrintable(QStringLiteral("reverse resume plan failed: %1").arg(error)));
    CheckNear(plan.pauseArcMm, 13.0, "reverse pause arc ignored execution order");
    CheckNear(plan.resumeArcMm, 8.0, "reverse backtrack arc is wrong");
    CheckNear(plan.totalArcMm, 25.0, "reverse total arc is wrong");
    Check(plan.matchedSegmentIndex == 0, "reverse pause matched wrong segment");
}

void TestNonUniformThreeDimensionalArcAndBounds(const QString& root)
{
    const QString path = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("Case3D"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 3.0, 4.0, 0.0 }, { 3.0, 4.0, 12.0 } });
    WeldResumePlanner::ResumePlan plan;
    QString error;
    Check(WeldResumePlanner::PlanFromPausedPose(
        path, 3.0, 4.0, 7.0, 6.0, plan, &error),
        qPrintable(QStringLiteral("3D resume plan failed: %1").arg(error)));
    CheckNear(plan.pauseArcMm, 12.0, "non-uniform 3D pause arc is wrong");
    CheckNear(plan.resumeArcMm, 6.0, "non-uniform 3D backtrack arc is wrong");
    CheckNear(plan.totalArcMm, 17.0, "non-uniform 3D total arc is wrong");
    CheckNear(plan.matchedSegmentRatio, 7.0 / 12.0,
        "non-uniform 3D projection ratio is wrong");

    Check(WeldResumePlanner::PlanFromPausedPose(
        path, 1.2, 1.6, 0.0, 50.0, plan, &error),
        qPrintable(QStringLiteral("bounded backtrack plan failed: %1").arg(error)));
    CheckNear(plan.pauseArcMm, 2.0, "bounded backtrack pause arc is wrong");
    CheckNear(plan.resumeArcMm, 0.0, "backtrack before trajectory start was not clamped");
    CheckNear(plan.actualBacktrackMm, 2.0,
        "actual backtrack did not report start-boundary clamp");

    Check(!WeldResumePlanner::PlanFromPausedPose(
        path, 1.0, 1.0, 1.0, -0.001, plan, &error),
        "negative backtrack distance was accepted");
}

void TestSelfIntersectionAmbiguity(const QString& root)
{
    const QString bowTie = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseAmbiguous"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { -10.0, -10.0, 0.0 }, { 10.0, 10.0, 0.0 },
          { -10.0, 10.0, 0.0 }, { 10.0, -10.0, 0.0 } });
    WeldResumePlanner::ResumePlan plan;
    QString error;
    Check(!WeldResumePlanner::PlanFromPausedPose(
        bowTie, 0.0, 0.0, 0.0, 5.0, plan, &error),
        "self-intersection pause was guessed instead of rejected");
    Check(error.contains(QStringLiteral("多个近似候选")),
        "self-intersection rejection did not report ambiguity");

    // A loop well below the former 10 mm arc threshold can still place the
    // same TCP pose at two materially distinct execution arcs.
    const QString compactBowTie = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseCompactAmbiguous"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { -1.0, -1.0, 0.0 }, { 1.0, 1.0, 0.0 },
          { -1.0, 1.0, 0.0 }, { 1.0, -1.0, 0.0 } });
    Check(!WeldResumePlanner::PlanFromPausedPose(
        compactBowTie, 0.0, 0.0, 0.0, 1.0, plan, &error),
        "compact self-intersection bypassed ambiguity rejection");

    // A shared vertex appears on both adjacent segments but represents the
    // same arc position, so it must remain resumable.
    const QString corner = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseCorner"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 20.0, 0.0, 0.0 }, { 20.0, 20.0, 0.0 } });
    Check(WeldResumePlanner::PlanFromPausedPose(
        corner, 20.0, 0.0, 0.0, 5.0, plan, &error),
        qPrintable(QStringLiteral("ordinary corner was falsely ambiguous: %1").arg(error)));
    CheckNear(plan.pauseArcMm, 20.0, "corner pause arc is wrong");

    const QString nearVertex = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseNearVertex"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 2.0, 0.0, 0.0 }, { 4.0, 0.0, 0.0 } });
    Check(WeldResumePlanner::PlanFromPausedPose(
        nearVertex, 1.9, 0.0, 0.0, 0.5, plan, &error),
        qPrintable(QStringLiteral("near-vertex local candidate was falsely ambiguous: %1")
            .arg(error)));
    CheckNear(plan.pauseArcMm, 1.9, "near-vertex pause arc is wrong");
    CheckNear(plan.resumeArcMm, 1.4, "near-vertex backtrack arc is wrong");
}

void TestCheckpointTimeGate()
{
    WeldResumePlanner::CheckpointRecord record;
    QString error;
    const QDateTime now = QDateTime::fromString(
        QStringLiteral("2026-07-12T12:00:00.000Z"), Qt::ISODateWithMs);

    record.createdAtUtc = QStringLiteral("2026-07-11T12:00:00.000Z");
    Check(WeldResumePlanner::ValidateCheckpointTime(record, now, &error),
        "checkpoint at the exact 24-hour boundary was rejected");

    record.createdAtUtc = QStringLiteral("2026-07-11T11:59:59.000Z");
    Check(!WeldResumePlanner::ValidateCheckpointTime(record, now, &error),
        "checkpoint older than 24 hours was accepted");

    record.createdAtUtc = QStringLiteral("2026-07-12T12:05:00.000Z");
    Check(WeldResumePlanner::ValidateCheckpointTime(record, now, &error),
        "checkpoint at the allowed future-clock-skew boundary was rejected");

    record.createdAtUtc = QStringLiteral("2026-07-12T12:05:01.000Z");
    Check(!WeldResumePlanner::ValidateCheckpointTime(record, now, &error),
        "checkpoint beyond the future-clock-skew boundary was accepted");

    record.createdAtUtc = QStringLiteral("not-an-iso-time");
    Check(!WeldResumePlanner::ValidateCheckpointTime(record, now, &error),
        "checkpoint with an invalid timestamp was accepted");
}

void TestSafeRetreatRecoveryRecord(const QString& root)
{
    const QString path = WriteTrajectory(
        root,
        QStringLiteral("RobotA"),
        QStringLiteral("CaseSafeRetreat"),
        QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp_FinalSampled.txt"),
        { { 0.0, 0.0, 0.0 }, { 20.0, 0.0, 0.0 } });
    WeldResumePlanner::CheckpointRecord record = CompleteRecord(root, path);
    record.state = QStringLiteral("unretracted");
    record.safetyObservedAtUtc = QStringLiteral("2026-07-12T12:00:00.000Z");
    record.safetyReason = QStringLiteral("user cancelled after completed weld");
    record.safetyProgramCompleted = true;
    record.safetyMoveSpeedMmPerMin = 600.0;
    record.safeX = -70.0;
    record.safeY = 0.0;
    record.safeZ = 10.0;
    record.safeRx = 180.0;
    record.safeRy = 0.0;
    record.safeRz = 90.0;
    record.terminalPoseValid = true;
    record.terminalX = 20.0;
    record.terminalY = 0.0;
    record.terminalZ = 0.0;
    record.terminalRx = 180.0;
    record.terminalRy = 0.0;
    record.terminalRz = 90.0;
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);

    QString error;
    Check(WeldResumePlanner::ValidateSafeRetreatRecoveryRecord(record, &error),
        qPrintable(QStringLiteral("valid unretracted record rejected: %1").arg(error)));
    const QString encoded = WeldResumePlanner::EncodeRecord(record, &error);
    WeldResumePlanner::CheckpointRecord decoded;
    Check(!encoded.isEmpty() && WeldResumePlanner::DecodeRecord(encoded, decoded, &error),
        qPrintable(QStringLiteral("unretracted record round-trip failed: %1").arg(error)));

    decoded.safetyReason += QStringLiteral(" tampered");
    Check(!WeldResumePlanner::ValidateSafeRetreatRecoveryRecord(decoded, &error),
        "tampered safe-retreat reason retained a valid witness");

    record.safetyProgramCompleted = false;
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);
    Check(!WeldResumePlanner::ValidateSafeRetreatRecoveryRecord(record, &error),
        "unretracted state accepted programCompleted=false");

    record.state = QStringLiteral("interrupted");
    record.safetyReason = QStringLiteral("START attempted; completion unknown");
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);
    Check(WeldResumePlanner::ValidateSafeRetreatRecoveryRecord(record, &error),
        qPrintable(QStringLiteral("interrupted recovery record rejected: %1").arg(error)));
}
}

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    QTemporaryDir project;
    Check(project.isValid(), "could not create temporary project root");

    TestColumnParsingAndRecordRoundTrip(project.path());
    TestPathBindingAndTamperDetection(project.path());
    TestTrajectoryResourceBounds(project.path());
    TestBoundPlanningSnapshot(project.path());
    TestForwardAndReverseArcLength(project.path());
    TestNonUniformThreeDimensionalArcAndBounds(project.path());
    TestSelfIntersectionAmbiguity(project.path());
    TestCheckpointTimeGate();
    TestSafeRetreatRecoveryRecord(project.path());

    std::cout << "PASS: V2 record, bound trajectory integrity and planning snapshot, "
        "column parsing, execution-order arc backtrack, bounds, ambiguity, and checkpoint TTL\n";
    return 0;
}
