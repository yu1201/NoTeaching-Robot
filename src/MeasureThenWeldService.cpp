#include "MeasureThenWeldService.h"

#include "CameraFrameCache.h"
#include "ConfigDatabase.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "RobotPoseTransform.h"
#include "STEPRobotDriver.h"
#include "groove/framebuffer.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>
#include <QStringConverter>
#include <QTextStream>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>

namespace
{
constexpr int FANUC_MOTION_STATE_REG = 93;
constexpr double FANUC_WELD_PATH_SPEED_MM_PER_MIN = 400.0;
constexpr double DEFAULT_WELD_SAFE_MOVE_SPEED_MM_PER_MIN = 1000.0;
constexpr double DEFAULT_DRY_RUN_SPEED_MM_PER_MIN = 1000.0;
constexpr double WELD_SAFE_OFFSET_DISTANCE_MM = 70.0;
constexpr double DEFAULT_CAMERA_READ_FPS = 100.0;
constexpr qint64 ROBOT_SAMPLE_INTERVAL_MS = 50;
constexpr qint64 CAMERA_ROBOT_MATCH_TAIL_WAIT_MS = 500;
constexpr qint64 CAMERA_ROBOT_MATCH_TAIL_POLL_MS = 10;
constexpr auto RAW_LASER_FILE_NAME = "PreciseLaserPoint.txt";
constexpr auto WORKPIECE_CLOUD_FILE_NAME = "PreciseLaserPoint_WorkpieceCloud.txt";
constexpr auto PRESERVE_PATH_FILE_NAME = "PreciseLaserPoint_PreservePath_2mm.txt";
constexpr auto KEY_POINTS_FILE_NAME = "PreciseLaserPoint_KeyPoints.txt";
constexpr auto CLASSIFIED_FILE_NAME = "PreciseLaserPoint_Classified.txt";
constexpr auto CLASSIFIED_NOISE_FILE_NAME = "PreciseLaserPoint_Classified_Noise.txt";
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";
constexpr auto WELD_SEGMENT_KIND_DEBUG_FILE_NAME = "PreciseLaserPointSegmentKind.txt";
constexpr auto MATCH_DEBUG_FILE_NAME = "PreciseLaserPoint_MatchDebug.csv";
constexpr int POSE_COMP_MATCH_BY_POSE = 0;
constexpr int POSE_COMP_MATCH_BY_SEGMENT_CODE = 1;
constexpr char POSE_COMP_MATCH_MODE_KEY[] = "PoseCompMatchMode";
constexpr int COMP_SEGMENT_COUNT = 4;
constexpr char POSE_GROUP_COUNT_KEY[] = "PoseCompGroupCount";
constexpr char POSE_ACTIVE_GROUP_INDEX_KEY[] = "ActivePoseCompGroupIndex";
constexpr char SEAM_GROUP_COUNT_KEY[] = "SeamCompGroupCount";
constexpr char SEAM_ACTIVE_GROUP_INDEX_KEY[] = "ActiveSeamCompGroupIndex";

int NormalizePoseCompMatchMode(int mode)
{
    return mode == POSE_COMP_MATCH_BY_SEGMENT_CODE
        ? POSE_COMP_MATCH_BY_SEGMENT_CODE
        : POSE_COMP_MATCH_BY_POSE;
}

QString PoseCompMatchModeDisplayName(int mode)
{
    return NormalizePoseCompMatchMode(mode) == POSE_COMP_MATCH_BY_SEGMENT_CODE
        ? QStringLiteral("按四类段属性")
        : QStringLiteral("按姿态匹配");
}

std::string ToUtf8StdString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

qint64 SteadyNowMs()
{
    return static_cast<qint64>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

qint64 SteadyNowUs()
{
    return static_cast<qint64>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

struct TimestampedCameraPoint
{
    int sampleIndex = 0;
    qint64 rawTimestampUs = 0;
    qint64 rawDeltaUs = 0;
    qint64 timestampUs = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> linePoints;
    QString error;
};

void ResolveCameraSamplesAgainstRobotTimeline(
    const std::vector<TimestampedCameraPoint>& cameraSamples,
    std::size_t& nextPendingCameraIndex,
    const std::vector<RobotCalculation::TimestampedRobotPose>& robotSamples,
    std::vector<TimestampedCameraPoint>& matchedCameraSamples,
    int& droppedHeadCameraCount)
{
    if (robotSamples.empty())
    {
        return;
    }

    const qint64 earliestRobotTimestampUs = robotSamples.front().timestampUs;
    const qint64 latestRobotTimestampUs = robotSamples.back().timestampUs;
    while (nextPendingCameraIndex < cameraSamples.size())
    {
        const TimestampedCameraPoint& sample = cameraSamples[nextPendingCameraIndex];
        if (sample.timestampUs < earliestRobotTimestampUs)
        {
            ++droppedHeadCameraCount;
            ++nextPendingCameraIndex;
            continue;
        }

        if (sample.timestampUs > latestRobotTimestampUs)
        {
            break;
        }

        matchedCameraSamples.push_back(sample);
        ++nextPendingCameraIndex;
    }
}

struct WeldPosePreset
{
    struct PoseCompSlot
    {
        QString name;
        QString segmentKind;
        double poseRx = 0.0;
        double poseRy = 0.0;
        double poseRz = 0.0;
        double compX = 0.0;
        double compY = 0.0;
        double compZ = 0.0;
        bool hasIniReference = false;
        bool generatedReference = false;
        bool validReference = false;
    };

    struct SeamCompSlot
    {
        QString name;
        QString segmentKind;
        double weldZComp = 0.0;
        double weldGunDirComp = 0.0;
        double weldSeamDirComp = 0.0;
    };

    QString weldLineFilePath;
    QString weldLineSectionName;
    QString poseCompFilePath;
    QString seamCompFilePath;
    QString robotParaPath;
    int robotType = ROBOT_TYPE_FANUC;
    QString seamKind = "CorrugatedPlate";
    double rx = 0.0;
    double ry = 0.0;
    double measureReferenceRy = 0.0;
    double measureReferenceRz = 0.0;
    double gunToolBaseRz = 180.0;
    double poseMatchMaxErrorDeg = 5.0;
    int poseCompMatchMode = POSE_COMP_MATCH_BY_POSE;
    double cornerTransitionLeadDistance = 10.0;
    double cornerArcRadiusMm = 0.0;
    double weldStartSkipDistance = 10.0;
    double weldEndSkipDistance = 10.0;
    double weldRzGainDeg = 0.0;
    double slopeRzMinDeg = -20.0;
    double slopeRzMaxDeg = 20.0;
    double stepOverlapRel = 20.0;
    int weldDirection = 1;
    bool weldProcessLoaded = false;
    bool cornerArcRadiusFromWeldProcess = false;
    bool transitionSpeedEnabled = false;
    bool transitionCurrentVoltageEnabled = false;
    bool transitionCurrentVoltageEnableMismatch = false;
    int transitionApplyScope = 2;
    double startArcCurrent = 0.0;
    double startArcVoltage = 0.0;
    double startArcWaitTime = 0.0;
    double weldCurrent = 0.0;
    double weldVoltage = 0.0;
    double weldProcessSpeedMmPerMin = 0.0;
    double stopArcCurrent = 0.0;
    double stopArcVoltage = 0.0;
    double stopArcWaitTime = 0.0;
    double transitionSpeedMmPerMin = 0.0;
    double transitionCurrent = 0.0;
    double transitionVoltage = 0.0;
    std::vector<PoseCompSlot> poseCompSlots;
    std::vector<SeamCompSlot> seamCompSlots;
    bool weldLineFromIni = false;
    bool poseCompFromIni = false;
    bool seamCompFromIni = false;
};

struct MeasurementPoseReference
{
    bool valid = false;
    int count = 0;
    QString source;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

bool IsFiniteCameraPoint(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x())
        && std::isfinite(point.y())
        && std::isfinite(point.z());
}

bool ShouldSkipLaserCalc(const TimestampedCameraPoint& sample)
{
    if (!IsFiniteCameraPoint(sample.point))
    {
        return true;
    }

    constexpr double kZeroPointEps = 1e-9;
    return std::abs(sample.point.x()) <= kZeroPointEps
        && std::abs(sample.point.y()) <= kZeroPointEps
        && std::abs(sample.point.z()) <= kZeroPointEps;
}

QString CsvEscape(const QString& value)
{
    QString escaped = value;
    escaped.replace("\"", "\"\"");
    if (escaped.contains(',') || escaped.contains('"') || escaped.contains('\n') || escaped.contains('\r'))
    {
        escaped = "\"" + escaped + "\"";
    }
    return escaped;
}

QString Vector3CsvFields(const Eigen::Vector3d& point)
{
    return QString("%1,%2,%3")
        .arg(point.x(), 0, 'f', 6)
        .arg(point.y(), 0, 'f', 6)
        .arg(point.z(), 0, 'f', 6);
}

QString RobotPoseCsvFields(const T_ROBOT_COORS& pose)
{
    return QString("%1,%2,%3,%4,%5,%6,%7,%8,%9")
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
}

struct RobotInterpolationWindow
{
    int prevIndex = -1;
    int nextIndex = -1;
    qint64 prevTimestampUs = 0;
    qint64 nextTimestampUs = 0;
    double ratio = 0.0;
};

struct QueuedScanCameraFrame
{
    int sampleIndex = 0;
    qint64 rawTimestampUs = 0;
    qint64 rawDeltaUs = 0;
    qint64 timestampUs = 0;
    udpDataShow frame;
};

struct ProcessedScanWorkpiecePoint
{
    Eigen::Vector3d workpiecePoint = Eigen::Vector3d::Zero();
};

struct ProcessedScanCameraSample
{
    TimestampedCameraPoint sample;
    QString status;
    RobotInterpolationWindow robotWindow;
    bool hasRobotPose = false;
    bool hasLaserPoint = false;
    bool contributedWorkpieceFrame = false;
    T_ROBOT_COORS robotPose;
    Eigen::Vector3d laserPoint = Eigen::Vector3d::Zero();
    std::vector<ProcessedScanWorkpiecePoint> workpiecePoints;
    int skippedWorkpieceCloudPointCount = 0;
};

RobotInterpolationWindow FindRobotInterpolationWindow(
    const std::vector<RobotCalculation::TimestampedRobotPose>& robotSamples,
    qint64 targetTimestampUs)
{
    RobotInterpolationWindow window;
    if (robotSamples.empty())
    {
        return window;
    }

    if (targetTimestampUs <= robotSamples.front().timestampUs)
    {
        window.prevIndex = 1;
        window.nextIndex = robotSamples.size() > 1 ? 2 : 1;
        window.prevTimestampUs = robotSamples.front().timestampUs;
        window.nextTimestampUs = robotSamples[static_cast<std::size_t>(window.nextIndex - 1)].timestampUs;
        return window;
    }

    if (targetTimestampUs >= robotSamples.back().timestampUs)
    {
        window.nextIndex = static_cast<int>(robotSamples.size());
        window.prevIndex = robotSamples.size() > 1 ? window.nextIndex - 1 : window.nextIndex;
        window.prevTimestampUs = robotSamples[static_cast<std::size_t>(window.prevIndex - 1)].timestampUs;
        window.nextTimestampUs = robotSamples.back().timestampUs;
        window.ratio = 1.0;
        return window;
    }

    const auto upper = std::lower_bound(
        robotSamples.begin(),
        robotSamples.end(),
        targetTimestampUs,
        [](const RobotCalculation::TimestampedRobotPose& sample, qint64 timestamp)
        {
            return sample.timestampUs < timestamp;
        });
    const auto lower = upper - 1;
    window.prevIndex = static_cast<int>(std::distance(robotSamples.begin(), lower)) + 1;
    window.nextIndex = static_cast<int>(std::distance(robotSamples.begin(), upper)) + 1;
    window.prevTimestampUs = lower->timestampUs;
    window.nextTimestampUs = upper->timestampUs;
    const qint64 dt = window.nextTimestampUs - window.prevTimestampUs;
    window.ratio = dt == 0 ? 0.0 : static_cast<double>(targetTimestampUs - window.prevTimestampUs) / static_cast<double>(dt);
    return window;
}

RobotCalculation::SampleAxis InferMeasureSampleAxis(const T_PRECISE_MEASURE_PARAM& param)
{
    const double deltaX = std::abs(param.tEndPos.dX - param.tStartPos.dX);
    const double deltaY = std::abs(param.tEndPos.dY - param.tStartPos.dY);
    return deltaX > deltaY
        ? RobotCalculation::SampleAxis::AxisX
        : RobotCalculation::SampleAxis::AxisY;
}

QString SampleAxisName(RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisX ? "X" : "Y";
}

RobotCalculation::LowerWeldFilterParams BuildOriginalTrackFitParams(const T_PRECISE_MEASURE_PARAM& param)
{
    RobotCalculation::LowerWeldFilterParams params;
    params.sampleAxis = InferMeasureSampleAxis(param);
    params.fitMode = RobotCalculation::LowerWeldFitMode::PreservePath;
    params.zThreshold = -230.0;
    params.zJumpThreshold = 3.0;
    params.zContinuityThreshold = 2.0;
    params.segmentBreakDistance = 6.0;
    params.keepLongestSegmentOnly = true;
    params.sampleStep = 2.0;
    params.searchWindow = 8.0;
    params.lineFitTrimCount = 0;
    params.piecewiseFitTolerance = 4.0;
    params.piecewiseMinSegmentPoints = 10;
    params.minPointCount = 4;
    params.smoothRadius = 3;
    return params;
}

QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<RobotCalculation::LowerWeldFilterPoint>& points)
{
    QVector<RobotCalculation::IndexedPoint3D> indexedPoints;
    indexedPoints.reserve(points.size());
    for (const RobotCalculation::LowerWeldFilterPoint& point : points)
    {
        RobotCalculation::IndexedPoint3D indexedPoint;
        indexedPoint.index = point.index;
        indexedPoint.point = point.point;
        indexedPoints.push_back(indexedPoint);
    }
    return indexedPoints;
}

std::vector<QString> BuildFilterOutputLines(const RobotCalculation::LowerWeldFilterResult& result)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(result.points.size()) + 1);
    lines.push_back("index x y z source");
    for (const RobotCalculation::LowerWeldFilterPoint& point : result.points)
    {
        lines.push_back(RobotCalculation::Vector3IndexedSpaceText(point.index, point.point, point.source));
    }
    return lines;
}

std::vector<QString> BuildClassifiedOutputLines(const RobotCalculation::LowerWeldClassificationResult& result)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(result.points.size()) + 2);
    lines.push_back("# index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise");
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : result.points)
    {
        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
            .arg(RobotCalculation::LowerWeldPointTypeName(point.type))
            .arg(point.source));
    }
    return lines;
}

std::vector<QString> BuildKeyPointOutputLines(const QVector<RobotCalculation::LowerWeldClassifiedPoint>& points)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 2);
    lines.push_back("# source_index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner");
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : points)
    {
        if (point.type == RobotCalculation::LowerWeldPointType::Normal
            || point.type == RobotCalculation::LowerWeldPointType::Noise)
        {
            continue;
        }

        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
            .arg(RobotCalculation::LowerWeldPointTypeName(point.type))
            .arg(point.source.isEmpty() ? "-" : point.source));
    }
    return lines;
}

std::vector<QString> BuildNoiseOutputLines(
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const RobotCalculation::LowerWeldFilterResult& fitResult)
{
    std::vector<QString> lines;
    QSet<int> validIndexes;
    validIndexes.reserve(fitResult.points.size());
    for (const RobotCalculation::LowerWeldFilterPoint& point : fitResult.points)
    {
        validIndexes.insert(point.index);
    }

    lines.reserve(static_cast<size_t>(inputPoints.size()) + 2);
    lines.push_back("# index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise");
    for (const RobotCalculation::IndexedPoint3D& point : inputPoints)
    {
        if (validIndexes.contains(point.index))
        {
            continue;
        }
        lines.push_back(QString("%1 %2 %3 %4 6 noise raw")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6));
    }
    return lines;
}

QString FilterResultSummary(
    const QString& phaseName,
    const RobotCalculation::LowerWeldFilterParams& params,
    const RobotCalculation::LowerWeldFilterResult& result,
    const QString& outputPath)
{
    return QString("%1完成：采样主轴=%2，输入点=%3，下层候选点=%4，剔除Z突变=%5，剔除Z连续异常=%6，连续段剔除=%7，拟合段数=%8，输出点=%9，结果=%10")
        .arg(phaseName)
        .arg(SampleAxisName(params.sampleAxis))
        .arg(result.inputPointCount)
        .arg(result.lowerPointCount)
        .arg(result.zJumpRejectedCount)
        .arg(result.zContinuityRejectedCount)
        .arg(result.segmentRejectedCount)
        .arg(result.fitSegmentCount)
        .arg(result.points.size())
        .arg(outputPath);
}

double NormalizeAngleNear(double angleDeg, double referenceDeg)
{
    while ((angleDeg - referenceDeg) > 180.0)
    {
        angleDeg -= 360.0;
    }
    while ((angleDeg - referenceDeg) < -180.0)
    {
        angleDeg += 360.0;
    }
    return angleDeg;
}

double NormalizeAngleToFanucRange(double angleDeg)
{
    while (angleDeg > 180.0)
    {
        angleDeg -= 360.0;
    }
    while (angleDeg <= -180.0)
    {
        angleDeg += 360.0;
    }
    return angleDeg;
}

double NormalizeLineAxisDeviationFromReferenceAxis(double directionDeg, double referenceAxisDeg)
{
    double deviation = directionDeg - referenceAxisDeg;
    while (deviation > 90.0)
    {
        deviation -= 180.0;
    }
    while (deviation <= -90.0)
    {
        deviation += 180.0;
    }
    return deviation;
}

double ProjectLineAxisDeviationToPoseRz(double axisDeviationDeg, double measureReferenceRyDeg)
{
    const double tiltFactor = std::abs(std::sin(measureReferenceRyDeg * M_PI / 180.0));
    if (tiltFactor <= 1e-6)
    {
        return 0.0;
    }

    const double deviationRad = axisDeviationDeg * M_PI / 180.0;
    if (std::abs(std::cos(deviationRad)) <= 1e-6)
    {
        return std::copysign(90.0, axisDeviationDeg);
    }

    return std::atan(std::tan(deviationRad) * tiltFactor) * 180.0 / M_PI;
}

double NormalizeRobotRzOutputRange(double angleDeg)
{
    while (angleDeg > 180.0)
    {
        angleDeg -= 360.0;
    }
    while (angleDeg < -180.0)
    {
        angleDeg += 360.0;
    }
    if (std::abs(angleDeg - 180.0) <= 1e-9)
    {
        return -180.0;
    }
    return angleDeg;
}

double RobotRzFromGunDirectionDeg(double gunDirectionFromXDeg)
{
    // STEP RZ convention used by the weld posture generator:
    // gun pointing to robot X- is 0 deg, clockwise rotation is positive, and
    // the 180 deg direction is written as -180 deg.
    return NormalizeRobotRzOutputRange(180.0 - gunDirectionFromXDeg);
}

Eigen::Vector3d GunDirectionVectorFromRobotRz(double rzDeg)
{
    const double directionDeg = 180.0 - NormalizeRobotRzOutputRange(rzDeg);
    const double directionRad = directionDeg * M_PI / 180.0;
    return Eigen::Vector3d(std::cos(directionRad), std::sin(directionRad), 0.0);
}

double GunDirectionDegFromVector(const Eigen::Vector3d& direction)
{
    const double xyLength = std::hypot(direction.x(), direction.y());
    if (xyLength <= 1e-9)
    {
        return 180.0;
    }
    return std::atan2(direction.y(), direction.x()) * 180.0 / M_PI;
}

double ComputeLineNormalRobotRz(
    const Eigen::Vector3d& lineVector,
    double referenceRyDeg,
    double referenceRzDeg,
    double* chosenGunDirectionDeg = nullptr,
    double* rejectedRzDeg = nullptr,
    double* referenceDistanceDeg = nullptr)
{
    (void)referenceRyDeg;

    // RZ is driven by the weld normal, not by blending between corner poses.
    // A 2D seam direction has two perpendicular gun-normal candidates; the
    // current measurement posture selects the branch that keeps the gun closest
    // to how the camera saw this workpiece.
    Eigen::Vector3d tangent(lineVector.x(), lineVector.y(), 0.0);
    const double tangentLength = tangent.norm();
    if (tangentLength <= 1e-9)
    {
        if (chosenGunDirectionDeg != nullptr)
        {
            *chosenGunDirectionDeg = GunDirectionDegFromVector(
                GunDirectionVectorFromRobotRz(referenceRzDeg));
        }
        if (rejectedRzDeg != nullptr)
        {
            *rejectedRzDeg = NormalizeRobotRzOutputRange(referenceRzDeg + 180.0);
        }
        if (referenceDistanceDeg != nullptr)
        {
            *referenceDistanceDeg = 0.0;
        }
        return NormalizeRobotRzOutputRange(referenceRzDeg);
    }

    tangent /= tangentLength;
    const Eigen::Vector3d normalA(-tangent.y(), tangent.x(), 0.0);
    const Eigen::Vector3d normalB = -normalA;
    const Eigen::Vector3d referenceNormal = GunDirectionVectorFromRobotRz(referenceRzDeg);
    const bool useA = normalA.dot(referenceNormal) >= normalB.dot(referenceNormal);
    const Eigen::Vector3d selectedNormal = useA ? normalA : normalB;
    const Eigen::Vector3d rejectedNormal = useA ? normalB : normalA;

    const double selectedGunDirectionDeg = GunDirectionDegFromVector(selectedNormal);
    const double rejectedGunDirectionDeg = GunDirectionDegFromVector(rejectedNormal);
    const double selectedRz = RobotRzFromGunDirectionDeg(selectedGunDirectionDeg);
    const double rejectedRz = RobotRzFromGunDirectionDeg(rejectedGunDirectionDeg);
    if (chosenGunDirectionDeg != nullptr)
    {
        *chosenGunDirectionDeg = NormalizeAngleToFanucRange(selectedGunDirectionDeg);
    }
    if (rejectedRzDeg != nullptr)
    {
        *rejectedRzDeg = NormalizeRobotRzOutputRange(rejectedRz);
    }
    if (referenceDistanceDeg != nullptr)
    {
        *referenceDistanceDeg = std::abs(
            NormalizeAngleNear(selectedRz, referenceRzDeg) - referenceRzDeg);
    }
    return NormalizeRobotRzOutputRange(selectedRz);
}

double AngleDistanceDeg(double angleDeg, double referenceDeg)
{
    return std::abs(NormalizeAngleNear(angleDeg, referenceDeg) - referenceDeg);
}

bool TryParseFiniteDouble(const QString& text, double& value)
{
    bool ok = false;
    value = text.trimmed().toDouble(&ok);
    return ok && std::isfinite(value);
}

int CsvColumnIndex(const QStringList& header, const QString& name)
{
    for (int index = 0; index < header.size(); ++index)
    {
        if (header[index].trimmed() == name)
        {
            return index;
        }
    }
    return -1;
}

MeasurementPoseReference MeasurementPoseReferenceFromRobotPose(
    const T_ROBOT_COORS& pose,
    const QString& source,
    int count)
{
    MeasurementPoseReference reference;
    reference.valid = true;
    reference.count = std::max(1, count);
    reference.source = source;
    reference.x = pose.dX;
    reference.y = pose.dY;
    reference.z = pose.dZ;
    reference.rx = pose.dRX;
    reference.ry = pose.dRY;
    reference.rz = NormalizeRobotRzOutputRange(pose.dRZ);
    return reference;
}

MeasurementPoseReference FirstMeasurementPoseReferenceFromProcessedSamples(
    const std::vector<ProcessedScanCameraSample>& samples,
    const QString& source)
{
    int validCount = 0;
    MeasurementPoseReference reference;
    for (const ProcessedScanCameraSample& sample : samples)
    {
        if (!sample.hasRobotPose || !sample.hasLaserPoint)
        {
            continue;
        }

        ++validCount;
        if (!reference.valid)
        {
            reference = MeasurementPoseReferenceFromRobotPose(sample.robotPose, source, validCount);
        }
    }

    if (reference.valid)
    {
        reference.count = validCount;
    }
    return reference;
}

MeasurementPoseReference LoadMeasurementPoseReferenceFromMatchDebug(
    const QString& matchDebugPath,
    QString* error)
{
    if (error != nullptr)
    {
        error->clear();
    }

    QFile file(matchDebugPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("打开相机-机器人-激光匹配明细失败：%1").arg(matchDebugPath);
        }
        return {};
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    if (stream.atEnd())
    {
        if (error != nullptr)
        {
            *error = QString("相机-机器人-激光匹配明细为空：%1").arg(matchDebugPath);
        }
        return {};
    }

    const QStringList header = stream.readLine().split(',');
    const int statusCol = CsvColumnIndex(header, "status");
    const int robotXCol = CsvColumnIndex(header, "robot_x");
    const int robotYCol = CsvColumnIndex(header, "robot_y");
    const int robotZCol = CsvColumnIndex(header, "robot_z");
    const int robotRxCol = CsvColumnIndex(header, "robot_rx");
    const int robotRyCol = CsvColumnIndex(header, "robot_ry");
    const int robotRzCol = CsvColumnIndex(header, "robot_rz");
    const int requiredMaxCol = std::max({
        statusCol,
        robotXCol,
        robotYCol,
        robotZCol,
        robotRxCol,
        robotRyCol,
        robotRzCol
    });
    if (statusCol < 0 || robotXCol < 0 || robotYCol < 0 || robotZCol < 0
        || robotRxCol < 0 || robotRyCol < 0 || robotRzCol < 0)
    {
        if (error != nullptr)
        {
            *error = QString("相机-机器人-激光匹配明细缺少机器人姿态列：%1").arg(matchDebugPath);
        }
        return {};
    }

    int validCount = 0;
    MeasurementPoseReference reference;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty())
        {
            continue;
        }

        const QStringList fields = line.split(',');
        if (fields.size() <= requiredMaxCol || fields[statusCol].trimmed() != "laser_ok")
        {
            continue;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
        if (!TryParseFiniteDouble(fields[robotXCol], x)
            || !TryParseFiniteDouble(fields[robotYCol], y)
            || !TryParseFiniteDouble(fields[robotZCol], z)
            || !TryParseFiniteDouble(fields[robotRxCol], rx)
            || !TryParseFiniteDouble(fields[robotRyCol], ry)
            || !TryParseFiniteDouble(fields[robotRzCol], rz))
        {
            continue;
        }

        ++validCount;
        if (!reference.valid)
        {
            reference.valid = true;
            reference.source = matchDebugPath;
            reference.x = x;
            reference.y = y;
            reference.z = z;
            reference.rx = rx;
            reference.ry = ry;
            reference.rz = NormalizeRobotRzOutputRange(rz);
        }
    }

    if (reference.valid)
    {
        reference.count = validCount;
        return reference;
    }

    if (error != nullptr)
    {
        *error = QString("相机-机器人-激光匹配明细中没有可用的 laser_ok 测量姿态：%1").arg(matchDebugPath);
    }
    return {};
}

WeldPosePreset ApplyMeasurementPoseReferenceForCalculation(
    WeldPosePreset preset,
    const MeasurementPoseReference& reference,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (!reference.valid)
    {
        if (appendLog)
        {
            appendLog(QString("未读取到点云测量姿态，本次焊接姿态仍使用参数组扫描起终点姿态作为参考RZ=%1 deg。")
                .arg(preset.measureReferenceRz, 0, 'f', 3));
        }
        return preset;
    }

    // 测量姿态只用于焊道法向正反和RZ参考，不能覆盖参数组里固定的焊接RX/RY。
    preset.measureReferenceRy = reference.ry;
    preset.measureReferenceRz = reference.rz;
    if (appendLog)
    {
        appendLog(QString("本次计算使用点云测量姿态作为焊道法向参考，不覆盖固定焊接RX/RY：X=%1, Y=%2, Z=%3, 参考RX=%4, 参考RY=%5, 参考RZ=%6, 固定焊接RX=%7, 固定焊接RY=%8, 有效匹配点=%9, 来源=%10")
            .arg(reference.x, 0, 'f', 3)
            .arg(reference.y, 0, 'f', 3)
            .arg(reference.z, 0, 'f', 3)
            .arg(reference.rx, 0, 'f', 3)
            .arg(reference.ry, 0, 'f', 3)
            .arg(reference.rz, 0, 'f', 3)
            .arg(preset.rx, 0, 'f', 3)
            .arg(preset.ry, 0, 'f', 3)
            .arg(reference.count)
            .arg(reference.source));
    }
    return preset;
}

double PoseDistanceDeg(
    double rxDeg,
    double ryDeg,
    double rzDeg,
    double referenceRxDeg,
    double referenceRyDeg,
    double referenceRzDeg)
{
    const double deltaRx = rxDeg - referenceRxDeg;
    const double deltaRy = ryDeg - referenceRyDeg;
    const double deltaRz = AngleDistanceDeg(rzDeg, referenceRzDeg);
    return std::sqrt(deltaRx * deltaRx + deltaRy * deltaRy + deltaRz * deltaRz);
}

Eigen::Vector3d HorizontalUnitOrZero(const Eigen::Vector3d& vector)
{
    Eigen::Vector3d horizontal = vector;
    horizontal.z() = 0.0;
    const double norm = horizontal.head<2>().norm();
    if (norm <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }
    horizontal /= norm;
    return horizontal;
}

bool TryReadIniDouble(COPini& ini, const std::string& key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

void NormalizeSlopeRzClamp(double& minDeg, double& maxDeg)
{
    if (!std::isfinite(minDeg))
    {
        minDeg = -20.0;
    }
    if (!std::isfinite(maxDeg))
    {
        maxDeg = 20.0;
    }
    if (minDeg > maxDeg)
    {
        std::swap(minDeg, maxDeg);
    }
}

bool IsSlopeSegmentKind(const QString& segmentKind)
{
    return segmentKind.compare("rising_edge", Qt::CaseInsensitive) == 0
        || segmentKind.compare("falling_edge", Qt::CaseInsensitive) == 0;
}

int DefaultPoseCompSlotIndex(const QString& segmentKind)
{
    if (segmentKind.compare("low_platform", Qt::CaseInsensitive) == 0)
    {
        return 0;
    }
    if (segmentKind.compare("rising_edge", Qt::CaseInsensitive) == 0)
    {
        return 1;
    }
    if (segmentKind.compare("high_platform", Qt::CaseInsensitive) == 0)
    {
        return 2;
    }
    if (segmentKind.compare("falling_edge", Qt::CaseInsensitive) == 0)
    {
        return 3;
    }
    return -1;
}

QString DefaultPoseCompSlotKind(int index)
{
    switch (index)
    {
    case 0: return "low_platform";
    case 1: return "rising_edge";
    case 2: return "high_platform";
    case 3: return "falling_edge";
    default: return QString();
    }
}

void InitializeDefaultPoseCompSlots(std::vector<WeldPosePreset::PoseCompSlot>& poseCompCollection)
{
    for (int index = 0; index < static_cast<int>(poseCompCollection.size()); ++index)
    {
        poseCompCollection[index].name = QString("姿态补偿%1").arg(index + 1);
        poseCompCollection[index].segmentKind = DefaultPoseCompSlotKind(index);
    }
}

void InitializeDefaultSeamCompSlots(std::vector<WeldPosePreset::SeamCompSlot>& seamCompCollection)
{
    for (int index = 0; index < static_cast<int>(seamCompCollection.size()); ++index)
    {
        seamCompCollection[index].name = QString("焊道补偿%1").arg(index + 1);
        seamCompCollection[index].segmentKind = DefaultPoseCompSlotKind(index);
    }
}

bool TryParseWeldProcessDouble(const QStringList& fields, int index, double& value)
{
    if (index < 0 || index >= fields.size())
    {
        return false;
    }

    bool ok = false;
    const double parsed = fields[index].trimmed().toDouble(&ok);
    if (!ok)
    {
        return false;
    }

    value = parsed;
    return true;
}

bool TryParseWeldProcessInt(const QStringList& fields, int index, int& value)
{
    if (index < 0 || index >= fields.size())
    {
        return false;
    }

    bool ok = false;
    const int parsed = fields[index].trimmed().toInt(&ok);
    if (!ok)
    {
        return false;
    }

    value = parsed;
    return true;
}

constexpr int kTransitionScopeArc = 0;
constexpr int kTransitionScopeTransition = 1;
constexpr int kTransitionScopeArcAndTransition = 2;

int NormalizeTransitionApplyScope(int scope)
{
    if (scope < kTransitionScopeArc || scope > kTransitionScopeArcAndTransition)
    {
        return kTransitionScopeArcAndTransition;
    }
    return scope;
}

QString TransitionApplyScopeText(int scope)
{
    switch (NormalizeTransitionApplyScope(scope))
    {
    case kTransitionScopeArc:
        return QStringLiteral("圆弧");
    case kTransitionScopeTransition:
        return QStringLiteral("过渡");
    default:
        return QStringLiteral("圆弧+过渡");
    }
}

bool TryParseWeldProcessRow(const QStringList& fields, T_WELD_PARA& weldPara)
{
    if (fields.size() < 30)
    {
        return false;
    }

    weldPara = {};
    weldPara.strWorkPeace = ToUtf8StdString(fields.value(0).trimmed());
    weldPara.strWeldType = ToUtf8StdString(fields.value(1).trimmed());

    bool ok = TryParseWeldProcessDouble(fields, 4, weldPara.dStartArcCurrent)
        && TryParseWeldProcessDouble(fields, 5, weldPara.dStartArcVoltage)
        && TryParseWeldProcessDouble(fields, 6, weldPara.dStartWaitTime)
        && TryParseWeldProcessDouble(fields, 7, weldPara.dTrackCurrent)
        && TryParseWeldProcessDouble(fields, 8, weldPara.dTrackVoltage)
        && TryParseWeldProcessDouble(fields, 9, weldPara.WeldVelocity)
        && TryParseWeldProcessDouble(fields, 10, weldPara.dStopArcCurrent)
        && TryParseWeldProcessDouble(fields, 11, weldPara.dStopArcVoltage)
        && TryParseWeldProcessDouble(fields, 12, weldPara.dStopWaitTime);
    if (!ok)
    {
        return false;
    }

    const auto parseOptionalDouble = [&](int index, double& value, double fallback) -> bool
        {
            value = fallback;
            if (index >= fields.size())
            {
                return true;
            }
            return TryParseWeldProcessDouble(fields, index, value);
        };
    const auto parseOptionalInt = [&](int index, int& value, int fallback) -> bool
        {
            value = fallback;
            if (index >= fields.size())
            {
                return true;
            }
            return TryParseWeldProcessInt(fields, index, value);
        };

    return parseOptionalDouble(39, weldPara.dCornerArcTransitionRadius, 0.0)
        && parseOptionalDouble(40, weldPara.dCornerArcTransitionSpeed, 0.0)
        && parseOptionalDouble(41, weldPara.dCornerArcTransitionCurrent, 0.0)
        && parseOptionalDouble(42, weldPara.dCornerArcTransitionVoltage, 0.0)
        && parseOptionalInt(43, weldPara.nCornerArcTransitionRadiusEnable, 0)
        && parseOptionalInt(44, weldPara.nCornerArcTransitionSpeedEnable, 0)
        && parseOptionalInt(45, weldPara.nCornerArcTransitionCurrentEnable, 0)
        && parseOptionalInt(46, weldPara.nCornerArcTransitionVoltageEnable, 0)
        && parseOptionalInt(47, weldPara.nCornerArcTransitionApplyScope, kTransitionScopeArcAndTransition);
}

bool TryLoadActiveWeldProcessParam(const QString& robotName, T_WELD_PARA& weldPara)
{
    if (robotName.trimmed().isEmpty())
    {
        return false;
    }

    const QString weldFilePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldPara.txt").arg(robotName.trimmed()));
    std::string content;
    if (!ConfigDatabase::ReadTextFile(weldFilePath.toUtf8().constData(), &content))
    {
        return false;
    }

    int useIndex = 0;
    QVector<QStringList> weldRows;
    QString text = QString::fromUtf8(content.data(), static_cast<int>(content.size()));
    QTextStream in(&text);
    while (!in.atEnd())
    {
        const QString rawLine = in.readLine();
        const QString line = rawLine.trimmed();
        if (line.isEmpty() || line.startsWith('#'))
        {
            continue;
        }

        const QStringList fields = rawLine.split('\t', Qt::KeepEmptyParts);
        if (fields.isEmpty())
        {
            continue;
        }

        const QString tag = fields[0].trimmed();
        if (tag.compare("USE", Qt::CaseInsensitive) == 0)
        {
            if (fields.size() >= 2)
            {
                bool ok = false;
                const int parsed = fields[1].trimmed().toInt(&ok);
                if (ok)
                {
                    useIndex = parsed;
                }
            }
            continue;
        }

        weldRows.push_back(fields);
    }

    if (weldRows.isEmpty())
    {
        return false;
    }

    useIndex = qBound(0, useIndex, weldRows.size() - 1);
    return TryParseWeldProcessRow(weldRows[useIndex], weldPara);
}

T_PRECISE_MEASURE_PARAM BuildMeasureWeldParamShell(const QString& robotName)
{
    T_PRECISE_MEASURE_PARAM param;
    const QString normalizedRobotName = robotName.trimmed().isEmpty()
        ? QStringLiteral("RobotA")
        : robotName.trimmed();
    param.sRobotName = ToUtf8StdString(normalizedRobotName);

    QString ensureError;
    RobotDataHelper::EnsureMeasureWeldParamFile(normalizedRobotName, &ensureError);
    const QString iniPath = RobotDataHelper::MeasureWeldParamPath(normalizedRobotName);
    param.sIniFilePath = ToUtf8StdString(iniPath);
    param.sWeldParamFilePath = param.sIniFilePath;

    int groupIndex = 0;
    COPini ini;
    if (ini.SetFileName(param.sIniFilePath))
    {
        std::string groupName;
        ini.SetSectionName("MeasureWeldGroups");
        ini.ReadString(false, "UseGroupNo", &groupIndex);
        groupIndex = std::max(0, groupIndex);
        ini.ReadString(false, ToUtf8StdString(QString("Group%1Name").arg(groupIndex)), groupName);
        if (!groupName.empty())
        {
            param.sParamGroupName = QString::fromStdString(groupName);
        }
    }
    param.nParamGroupIndex = groupIndex;
    param.sSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldScanSectionName(groupIndex));
    param.sWeldSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldWeldSectionName(groupIndex));
    return param;
}

void ApplyActiveWeldProcessToPreset(const T_PRECISE_MEASURE_PARAM& param, WeldPosePreset& preset)
{
    T_WELD_PARA weldPara = {};
    if (!TryLoadActiveWeldProcessParam(QString::fromStdString(param.sRobotName), weldPara))
    {
        return;
    }

    preset.weldProcessLoaded = true;
    preset.startArcCurrent = weldPara.dStartArcCurrent;
    preset.startArcVoltage = weldPara.dStartArcVoltage;
    preset.startArcWaitTime = weldPara.dStartWaitTime;
    preset.weldCurrent = weldPara.dTrackCurrent;
    preset.weldVoltage = weldPara.dTrackVoltage;
    preset.weldProcessSpeedMmPerMin = weldPara.WeldVelocity;
    preset.stopArcCurrent = weldPara.dStopArcCurrent;
    preset.stopArcVoltage = weldPara.dStopArcVoltage;
    preset.stopArcWaitTime = weldPara.dStopWaitTime;
    preset.transitionApplyScope = NormalizeTransitionApplyScope(weldPara.nCornerArcTransitionApplyScope);

    if (weldPara.nCornerArcTransitionRadiusEnable != 0
        && std::isfinite(weldPara.dCornerArcTransitionRadius))
    {
        preset.cornerArcRadiusMm = std::max(2.0, weldPara.dCornerArcTransitionRadius);
        preset.cornerArcRadiusFromWeldProcess = true;
    }

    if (weldPara.nCornerArcTransitionSpeedEnable != 0
        && std::isfinite(weldPara.dCornerArcTransitionSpeed)
        && weldPara.dCornerArcTransitionSpeed > 0.0)
    {
        preset.transitionSpeedEnabled = true;
        preset.transitionSpeedMmPerMin = weldPara.dCornerArcTransitionSpeed;
    }

    const bool transitionCurrentEnabled = weldPara.nCornerArcTransitionCurrentEnable != 0;
    const bool transitionVoltageEnabled = weldPara.nCornerArcTransitionVoltageEnable != 0;
    preset.transitionCurrentVoltageEnableMismatch = transitionCurrentEnabled != transitionVoltageEnabled;
    if (transitionCurrentEnabled
        && transitionVoltageEnabled
        && std::isfinite(weldPara.dCornerArcTransitionCurrent)
        && std::isfinite(weldPara.dCornerArcTransitionVoltage))
    {
        preset.transitionCurrentVoltageEnabled = true;
        preset.transitionCurrent = weldPara.dCornerArcTransitionCurrent;
        preset.transitionVoltage = weldPara.dCornerArcTransitionVoltage;
    }
}

WeldPosePreset LoadWeldPosePreset(const T_PRECISE_MEASURE_PARAM& param)
{
    WeldPosePreset preset;
    preset.rx = param.tStartPos.dRX;
    preset.ry = param.tStartPos.dRY;
    preset.weldRzGainDeg = param.dWeldRzGainDeg;
    preset.slopeRzMinDeg = param.dSlopeRzMinDeg;
    preset.slopeRzMaxDeg = param.dSlopeRzMaxDeg;
    NormalizeSlopeRzClamp(preset.slopeRzMinDeg, preset.slopeRzMaxDeg);
    preset.stepOverlapRel = std::isfinite(param.dStepOverlapRel) ? std::max(0.0, param.dStepOverlapRel) : 20.0;
    preset.weldDirection = param.nWeldDirection < 0 ? -1 : 1;
    const double startRy = param.tStartPos.dRY;
    const double endRyNearStart = NormalizeAngleNear(param.tEndPos.dRY, startRy);
    preset.measureReferenceRy = (startRy + endRyNearStart) * 0.5;
    const double startRz = NormalizeAngleToFanucRange(param.tStartPos.dRZ);
    const double endRzNearStart = NormalizeAngleNear(param.tEndPos.dRZ, startRz);
    preset.measureReferenceRz = NormalizeAngleToFanucRange((startRz + endRzNearStart) * 0.5);
    preset.weldLineSectionName = param.sWeldSectionName.empty()
        ? QStringLiteral("WeldNormalParam0")
        : QString::fromStdString(param.sWeldSectionName);
    preset.weldLineFilePath = param.sWeldParamFilePath.empty()
        ? QString::fromStdString(param.sIniFilePath)
        : QString::fromStdString(param.sWeldParamFilePath);
    preset.poseCompFilePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldPoseCompParam.ini").arg(QString::fromStdString(param.sRobotName)));
    preset.seamCompFilePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldSeamCompParam.ini").arg(QString::fromStdString(param.sRobotName)));
    preset.robotParaPath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/RobotPara.ini").arg(QString::fromStdString(param.sRobotName)));
    preset.poseCompSlots.resize(4);
    InitializeDefaultPoseCompSlots(preset.poseCompSlots);
    preset.seamCompSlots.resize(4);
    InitializeDefaultSeamCompSlots(preset.seamCompSlots);

    if (!ConfigDatabase::HasIniFile(preset.weldLineFilePath))
    {
        goto load_pose_comp;
    }

    {
        COPini ini;
        if (ini.SetFileName(ToUtf8StdString(preset.weldLineFilePath)))
        {
            ini.SetSectionName(ToUtf8StdString(preset.weldLineSectionName));
            double rx = preset.rx;
            double ry = preset.ry;
            double cornerTransitionLeadDistance = preset.cornerTransitionLeadDistance;
            double weldStartSkipDistance = preset.weldStartSkipDistance;
            double weldEndSkipDistance = preset.weldEndSkipDistance;
            double weldRzGainDeg = preset.weldRzGainDeg;
            double slopeRzMinDeg = preset.slopeRzMinDeg;
            double slopeRzMaxDeg = preset.slopeRzMaxDeg;
            double stepOverlapRel = preset.stepOverlapRel;
            const bool hasNormalRx = TryReadIniDouble(ini, "NormalWeldRx", rx);
            const bool hasNormalRy = TryReadIniDouble(ini, "NormalWeldRy", ry);
            TryReadIniDouble(ini, "CornerTransitionLeadDis", cornerTransitionLeadDistance);
            TryReadIniDouble(ini, "WeldStartSkipDis", weldStartSkipDistance);
            TryReadIniDouble(ini, "WeldEndSkipDis", weldEndSkipDistance);
            TryReadIniDouble(ini, "WeldRzGainDeg", weldRzGainDeg);
            TryReadIniDouble(ini, "SlopeRzMinDeg", slopeRzMinDeg);
            TryReadIniDouble(ini, "SlopeRzMaxDeg", slopeRzMaxDeg);
            TryReadIniDouble(ini, "StepOverlapRel", stepOverlapRel);
            // 焊接顺序以当前测量焊接参数为准，避免旧 ini 字段覆盖界面选择。
            preset.stepOverlapRel = std::isfinite(stepOverlapRel) ? std::max(0.0, stepOverlapRel) : 20.0;
            if (!(hasNormalRx && hasNormalRy))
            {
                rx = preset.rx;
                ry = preset.ry;
                const bool hasFlatRx = TryReadIniDouble(ini, "FlatWeldRx", rx);
                const bool hasFlatRy = TryReadIniDouble(ini, "FlatWeldRy", ry);
                if (!(hasFlatRx && hasFlatRy))
                {
                    goto load_pose_comp;
                }
            }

            preset.rx = rx;
            preset.ry = ry;
            preset.cornerTransitionLeadDistance = std::max(0.0, cornerTransitionLeadDistance);
            preset.weldStartSkipDistance = std::max(0.0, weldStartSkipDistance);
            preset.weldEndSkipDistance = std::max(0.0, weldEndSkipDistance);
            preset.weldRzGainDeg = std::isfinite(weldRzGainDeg) ? weldRzGainDeg : 0.0;
            preset.slopeRzMinDeg = slopeRzMinDeg;
            preset.slopeRzMaxDeg = slopeRzMaxDeg;
            NormalizeSlopeRzClamp(preset.slopeRzMinDeg, preset.slopeRzMaxDeg);
            preset.weldLineFromIni = true;
        }
    }

load_pose_comp:
    ApplyActiveWeldProcessToPreset(param, preset);

    if (ConfigDatabase::HasIniFile(preset.poseCompFilePath))
    {
        COPini poseIni;
        if (poseIni.SetFileName(ToUtf8StdString(preset.poseCompFilePath)))
        {
            int poseCompCount = static_cast<int>(preset.poseCompSlots.size());
            poseIni.SetSectionName("ALLWeldPoseComp");
            poseIni.ReadString(false, "PoseCompCount", &poseCompCount);
            int poseGroupCount = 0;
            const bool hasPoseGroups = poseIni.ReadString(false, POSE_GROUP_COUNT_KEY, &poseGroupCount) > 0;
            int activePoseGroupIndex = 0;
            poseIni.ReadString(false, POSE_ACTIVE_GROUP_INDEX_KEY, &activePoseGroupIndex);
            int poseCompMatchMode = preset.poseCompMatchMode;
            if (poseIni.ReadString(false, POSE_COMP_MATCH_MODE_KEY, &poseCompMatchMode) > 0)
            {
                preset.poseCompMatchMode = NormalizePoseCompMatchMode(poseCompMatchMode);
            }
            TryReadIniDouble(poseIni, "PoseMatchMaxErrorDeg", preset.poseMatchMaxErrorDeg);
            preset.poseMatchMaxErrorDeg = std::max(0.0, preset.poseMatchMaxErrorDeg);

            int sourcePoseOffset = 0;
            int loadedPoseCompCount = std::max(0, poseCompCount);
            if (hasPoseGroups && poseGroupCount > 0)
            {
                activePoseGroupIndex = std::clamp(activePoseGroupIndex, 0, poseGroupCount - 1);
                sourcePoseOffset = activePoseGroupIndex * COMP_SEGMENT_COUNT;
                loadedPoseCompCount = COMP_SEGMENT_COUNT;
                poseIni.SetSectionName(ToUtf8StdString(QString("WeldPoseCompGroup%1").arg(activePoseGroupIndex)));
                int groupPoseCompMatchMode = preset.poseCompMatchMode;
                if (poseIni.ReadString(false, POSE_COMP_MATCH_MODE_KEY, &groupPoseCompMatchMode) > 0)
                {
                    preset.poseCompMatchMode = NormalizePoseCompMatchMode(groupPoseCompMatchMode);
                }
            }
            preset.poseCompSlots.assign(loadedPoseCompCount, WeldPosePreset::PoseCompSlot());
            InitializeDefaultPoseCompSlots(preset.poseCompSlots);
            for (int index = 0; index < static_cast<int>(preset.poseCompSlots.size()); ++index)
            {
                WeldPosePreset::PoseCompSlot& slot = preset.poseCompSlots[index];
                poseIni.SetSectionName(ToUtf8StdString(QString("WeldPoseComp%1").arg(sourcePoseOffset + index)));

                std::string slotName;
                std::string segmentKind;
                poseIni.ReadString(false, "Name", slotName);
                poseIni.ReadString(false, "SegmentKind", segmentKind);
                if (!slotName.empty())
                {
                    slot.name = QString::fromStdString(slotName);
                }
                if (!segmentKind.empty())
                {
                    slot.segmentKind = QString::fromStdString(segmentKind);
                }

                double poseRx = preset.rx;
                double poseRy = preset.ry;
                double poseRz = preset.gunToolBaseRz;

                const bool hasPoseRx = TryReadIniDouble(poseIni, "Rx", poseRx);
                const bool hasPoseRy = TryReadIniDouble(poseIni, "Ry", poseRy);
                const bool hasPoseRz = TryReadIniDouble(poseIni, "Rz", poseRz);
                TryReadIniDouble(poseIni, "CompX", slot.compX);
                TryReadIniDouble(poseIni, "CompY", slot.compY);
                TryReadIniDouble(poseIni, "CompZ", slot.compZ);

                slot.poseRx = poseRx;
                slot.poseRy = poseRy;
                slot.poseRz = NormalizeAngleToFanucRange(poseRz);
                slot.hasIniReference = hasPoseRx || hasPoseRy || hasPoseRz;
                slot.generatedReference = false;
                slot.validReference = slot.hasIniReference;
            }
            preset.poseCompFromIni = true;
        }
    }

    if (ConfigDatabase::HasIniFile(preset.seamCompFilePath))
    {
        COPini seamIni;
        if (seamIni.SetFileName(ToUtf8StdString(preset.seamCompFilePath)))
        {
            int seamCompCount = static_cast<int>(preset.seamCompSlots.size());
            seamIni.SetSectionName("ALLWeldSeamComp");
            seamIni.ReadString(false, "SeamCompCount", &seamCompCount);
            int seamGroupCount = 0;
            const bool hasSeamGroups = seamIni.ReadString(false, SEAM_GROUP_COUNT_KEY, &seamGroupCount) > 0;
            int activeSeamGroupIndex = 0;
            seamIni.ReadString(false, SEAM_ACTIVE_GROUP_INDEX_KEY, &activeSeamGroupIndex);
            int sourceSeamOffset = 0;
            int loadedSeamCompCount = std::max(0, seamCompCount);
            if (hasSeamGroups && seamGroupCount > 0)
            {
                activeSeamGroupIndex = std::clamp(activeSeamGroupIndex, 0, seamGroupCount - 1);
                sourceSeamOffset = activeSeamGroupIndex * COMP_SEGMENT_COUNT;
                loadedSeamCompCount = COMP_SEGMENT_COUNT;
            }
            preset.seamCompSlots.assign(loadedSeamCompCount, WeldPosePreset::SeamCompSlot());
            InitializeDefaultSeamCompSlots(preset.seamCompSlots);
            for (int index = 0; index < static_cast<int>(preset.seamCompSlots.size()); ++index)
            {
                WeldPosePreset::SeamCompSlot& slot = preset.seamCompSlots[index];
                seamIni.SetSectionName(ToUtf8StdString(QString("WeldSeamComp%1").arg(sourceSeamOffset + index)));

                std::string slotName;
                std::string segmentKind;
                seamIni.ReadString(false, "Name", slotName);
                seamIni.ReadString(false, "SegmentKind", segmentKind);
                if (!slotName.empty())
                {
                    slot.name = QString::fromStdString(slotName);
                }
                if (!segmentKind.empty())
                {
                    slot.segmentKind = QString::fromStdString(segmentKind);
                }

                TryReadIniDouble(seamIni, "WeldZComp", slot.weldZComp);
                TryReadIniDouble(seamIni, "WeldGunDirComp", slot.weldGunDirComp);
                TryReadIniDouble(seamIni, "WeldSeamDirComp", slot.weldSeamDirComp);
            }
            preset.seamCompFromIni = true;
        }
    }

    if (ConfigDatabase::HasIniFile(preset.robotParaPath))
    {
        COPini robotIni;
        if (robotIni.SetFileName(ToUtf8StdString(preset.robotParaPath)))
        {
            int robotType = preset.robotType;
            robotIni.SetSectionName("BaseParam");
            robotIni.ReadString(false, "RobotType", &robotType);
            preset.robotType = RobotPoseTransform::NormalizeRobotType(robotType);

            robotIni.SetSectionName("Tool");
            double gunToolBaseRz = preset.gunToolBaseRz;
            if (TryReadIniDouble(robotIni, "GunTool_dRZ", gunToolBaseRz))
            {
                preset.gunToolBaseRz = NormalizeAngleToFanucRange(gunToolBaseRz);
            }
        }
    }

    return preset;
}

double ComputeDirectionAngleDeg(const Eigen::Vector3d& startPoint, const Eigen::Vector3d& endPoint, bool* pValid = nullptr)
{
    const double deltaX = endPoint.x() - startPoint.x();
    const double deltaY = endPoint.y() - startPoint.y();
    const double length = std::hypot(deltaX, deltaY);
    const bool valid = length > 1e-6;
    if (pValid != nullptr)
    {
        *pValid = valid;
    }
    if (!valid)
    {
        return 0.0;
    }
    return std::atan2(deltaY, deltaX) * 180.0 / M_PI;
}

QString RobotPoseIndexedSpaceText(int index, const T_ROBOT_COORS& pose, const QString& extra = QString())
{
    QString line = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10")
        .arg(index)
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += " " + extra;
    }
    return line;
}

QString LowerWeldSegmentKindText(
    RobotCalculation::LowerWeldPointType beginType,
    RobotCalculation::LowerWeldPointType endType)
{
    using PointType = RobotCalculation::LowerWeldPointType;
    if ((beginType == PointType::Start && endType == PointType::InnerCorner)
        || (beginType == PointType::InnerCorner && endType == PointType::InnerCorner))
    {
        return "low_platform";
    }
    if (beginType == PointType::InnerCorner && endType == PointType::OuterCorner)
    {
        return "rising_edge";
    }
    if (beginType == PointType::OuterCorner && endType == PointType::OuterCorner)
    {
        return "high_platform";
    }
    if (beginType == PointType::OuterCorner && endType == PointType::InnerCorner)
    {
        return "falling_edge";
    }
    if (endType == PointType::End)
    {
        return "tail";
    }
    return "segment";
}

QString BuildWeldPoseOutputLine(
    int weldIndex,
    int rawIndex,
    const T_ROBOT_COORS& pose,
    RobotCalculation::LowerWeldPointType pointType,
    const QString& segmentKind,
    bool inTransition)
{
    return QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13")
        .arg(weldIndex)
        .arg(rawIndex)
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6)
        .arg(RobotCalculation::LowerWeldPointTypeName(pointType))
        .arg(inTransition ? (segmentKind + "_transition") : segmentKind);
}

struct WeldPoseFileRecord
{
    int weldIndex = 0;
    int rawIndex = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;
    QString pointType;
    QString segmentKind;
};

QString BuildWeldPoseFileRecordLine(const WeldPoseFileRecord& record)
{
    return QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13")
        .arg(record.weldIndex)
        .arg(record.rawIndex)
        .arg(record.point.x(), 0, 'f', 6)
        .arg(record.point.y(), 0, 'f', 6)
        .arg(record.point.z(), 0, 'f', 6)
        .arg(record.rx, 0, 'f', 6)
        .arg(record.ry, 0, 'f', 6)
        .arg(record.rz, 0, 'f', 6)
        .arg(record.bx, 0, 'f', 6)
        .arg(record.by, 0, 'f', 6)
        .arg(record.bz, 0, 'f', 6)
        .arg(record.pointType)
        .arg(record.segmentKind);
}

bool TryParseWeldPoseFileRecord(const QString& line, WeldPoseFileRecord& record)
{
    QString normalizedLine = line;
    normalizedLine.remove('"');
    const QStringList parts = normalizedLine.contains(',')
        ? normalizedLine.split(',', Qt::SkipEmptyParts)
        : normalizedLine.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if (parts.size() < 13)
    {
        return false;
    }

    bool weldIndexOk = false;
    bool rawIndexOk = false;
    bool xOk = false;
    bool yOk = false;
    bool zOk = false;
    bool rxOk = false;
    bool ryOk = false;
    bool rzOk = false;
    bool bxOk = false;
    bool byOk = false;
    bool bzOk = false;

    record.weldIndex = parts[0].trimmed().toInt(&weldIndexOk);
    record.rawIndex = parts[1].trimmed().toInt(&rawIndexOk);
    const double x = parts[2].trimmed().toDouble(&xOk);
    const double y = parts[3].trimmed().toDouble(&yOk);
    const double z = parts[4].trimmed().toDouble(&zOk);
    record.rx = parts[5].trimmed().toDouble(&rxOk);
    record.ry = parts[6].trimmed().toDouble(&ryOk);
    record.rz = parts[7].trimmed().toDouble(&rzOk);
    record.bx = parts[8].trimmed().toDouble(&bxOk);
    record.by = parts[9].trimmed().toDouble(&byOk);
    record.bz = parts[10].trimmed().toDouble(&bzOk);
    record.pointType = parts[11].trimmed();
    record.segmentKind = parts[12].trimmed();

    if (!(weldIndexOk && rawIndexOk && xOk && yOk && zOk
        && rxOk && ryOk && rzOk && bxOk && byOk && bzOk))
    {
        return false;
    }

    record.point = Eigen::Vector3d(x, y, z);
    return true;
}

void ApplyWeldDirectionToExecutionRecords(const WeldPosePreset& preset, QVector<WeldPoseFileRecord>& records)
{
    if (preset.weldDirection >= 0 || records.size() < 2)
    {
        return;
    }

    std::reverse(records.begin(), records.end());
    for (int index = 0; index < records.size(); ++index)
    {
        records[index].weldIndex = index + 1;
    }
}

QString WeldDirectionText(const WeldPosePreset& preset)
{
    return preset.weldDirection < 0
        ? QStringLiteral("终点到起点")
        : QStringLiteral("起点到终点");
}

bool LoadWeldPoseFileRecords(
    const QString& filePath,
    QVector<WeldPoseFileRecord>& records,
    QString& error)
{
    records.clear();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = "打开焊道姿态文件失败：" + QFileInfo(filePath).absoluteFilePath();
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);

    int lineNumber = 0;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        ++lineNumber;
        if (line.isEmpty() || line.startsWith('#'))
        {
            continue;
        }

        WeldPoseFileRecord record;
        if (!TryParseWeldPoseFileRecord(line, record))
        {
            if (records.isEmpty())
            {
                continue;
            }

            error = QString("解析焊道姿态文件失败，第 %1 行格式无效：%2")
                .arg(lineNumber)
                .arg(line);
            return false;
        }

        records.push_back(record);
    }

    if (records.isEmpty())
    {
        error = "焊道姿态文件中没有读取到有效点：" + QFileInfo(filePath).absoluteFilePath();
        return false;
    }

    return true;
}

const WeldPosePreset::SeamCompSlot* FindSeamCompSlotByKind(
    const WeldPosePreset& preset,
    const QString& segmentKind)
{
    for (const WeldPosePreset::SeamCompSlot& slot : preset.seamCompSlots)
    {
        if (slot.segmentKind.compare(segmentKind, Qt::CaseInsensitive) == 0)
        {
            return &slot;
        }
    }
    return nullptr;
}

QString NormalizeSeamCompSegmentKind(QString segmentKind)
{
    constexpr auto arcSuffix = "_arc";
    if (segmentKind.endsWith(arcSuffix, Qt::CaseInsensitive))
    {
        segmentKind.chop(static_cast<int>(std::strlen(arcSuffix)));
    }

    constexpr auto transitionSuffix = "_transition";
    if (segmentKind.endsWith(transitionSuffix, Qt::CaseInsensitive))
    {
        segmentKind.chop(static_cast<int>(std::strlen(transitionSuffix)));
    }
    return segmentKind;
}

int WeldSegmentKindCode(const QString& segmentKind)
{
    const QString normalized = NormalizeSeamCompSegmentKind(segmentKind).trimmed().toLower();
    if (normalized == "low_platform")
    {
        return 0;
    }
    if (normalized == "rising_edge")
    {
        return 1;
    }
    if (normalized == "high_platform")
    {
        return 2;
    }
    if (normalized == "falling_edge")
    {
        return 3;
    }
    return 4;
}

bool IsWeldPoseTransitionRecord(const WeldPoseFileRecord& record)
{
    return record.segmentKind.contains("_transition", Qt::CaseInsensitive)
        || record.pointType.contains("_transition", Qt::CaseInsensitive);
}

std::vector<QString> BuildWeldSegmentKindDebugLines(const QVector<WeldPoseFileRecord>& records)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(records.size()) + 1);
    lines.push_back("weld_index raw_index x y z segment_code transition arc");
    for (const WeldPoseFileRecord& record : records)
    {
        const int segmentCode = WeldSegmentKindCode(record.segmentKind);
        const int transitionFlag = IsWeldPoseTransitionRecord(record) ? 1 : 0;
        const int arcFlag = record.segmentKind.contains("_arc", Qt::CaseInsensitive)
            || record.pointType.contains("_arc", Qt::CaseInsensitive)
            ? 1
            : 0;
        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7 %8")
            .arg(record.weldIndex)
            .arg(record.rawIndex)
            .arg(record.point.x(), 0, 'f', 6)
            .arg(record.point.y(), 0, 'f', 6)
            .arg(record.point.z(), 0, 'f', 6)
            .arg(segmentCode)
            .arg(transitionFlag)
            .arg(arcFlag));
    }
    return lines;
}

const WeldPosePreset::SeamCompSlot* FindSeamCompSlotForRecord(
    const WeldPosePreset& preset,
    const WeldPoseFileRecord& record)
{
    const WeldPosePreset::SeamCompSlot* slot =
        FindSeamCompSlotByKind(preset, NormalizeSeamCompSegmentKind(record.segmentKind));
    if (slot != nullptr)
    {
        return slot;
    }

    // 未按具体段类型配置时，回退到当前工件类型的通用焊道补偿槽。
    return FindSeamCompSlotByKind(preset, preset.seamKind);
}

Eigen::Vector3d ResolveHorizontalTangentDirection(
    const QVector<Eigen::Vector3d>& points,
    int pointIndex)
{
    if (points.isEmpty() || pointIndex < 0 || pointIndex >= points.size())
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
    if (pointIndex > 0 && pointIndex + 1 < points.size())
    {
        tangent = points[pointIndex + 1] - points[pointIndex - 1];
    }
    else if (pointIndex + 1 < points.size())
    {
        tangent = points[pointIndex + 1] - points[pointIndex];
    }
    else if (pointIndex > 0)
    {
        tangent = points[pointIndex] - points[pointIndex - 1];
    }

    return HorizontalUnitOrZero(tangent);
}

QString RobotCoorsText(const T_ROBOT_COORS& coors)
{
    return QString("X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6 BX=%7 BY=%8 BZ=%9")
        .arg(coors.dX, 0, 'f', 3)
        .arg(coors.dY, 0, 'f', 3)
        .arg(coors.dZ, 0, 'f', 3)
        .arg(coors.dRX, 0, 'f', 3)
        .arg(coors.dRY, 0, 'f', 3)
        .arg(coors.dRZ, 0, 'f', 3)
        .arg(coors.dBX, 0, 'f', 3)
        .arg(coors.dBY, 0, 'f', 3)
        .arg(coors.dBZ, 0, 'f', 3);
}

double DegToRad(double deg)
{
    return deg * M_PI / 180.0;
}

T_ROBOT_COORS BuildScanSafeCoorsFromAnchor(
    const T_ROBOT_COORS& anchor,
    const T_PRECISE_MEASURE_PARAM& param)
{
    const double distance = std::isfinite(param.dScanSafeOffsetDistanceMm) && param.dScanSafeOffsetDistanceMm > 0.0
        ? param.dScanSafeOffsetDistanceMm
        : 150.0;
    const double angleDeg = std::isfinite(param.dScanSafeGunAngleDeg)
        ? param.dScanSafeGunAngleDeg
        : 30.0;
    const double angleRad = DegToRad(angleDeg);
    const double xSign = param.nScanSafeXDirection >= 0 ? 1.0 : -1.0;

    T_ROBOT_COORS safe = anchor;
    safe.dX += xSign * distance * std::sin(angleRad);
    safe.dZ += distance * std::cos(angleRad);
    return safe;
}

double PulseDeltaDeg(long currentPulse, long targetPulse, double pulseUnit)
{
    if (!std::isfinite(pulseUnit) || std::abs(pulseUnit) <= 1e-12)
    {
        return 0.0;
    }
    return std::abs(static_cast<double>(currentPulse - targetPulse) * pulseUnit);
}

double MaxWristDeltaDeg(
    const T_ANGLE_PULSE& currentPulse,
    const T_ANGLE_PULSE& targetPulse,
    const T_AXISUNIT& axisUnit)
{
    const double r = PulseDeltaDeg(currentPulse.nRPulse, targetPulse.nRPulse, axisUnit.dRPulseUnit);
    const double b = PulseDeltaDeg(currentPulse.nBPulse, targetPulse.nBPulse, axisUnit.dBPulseUnit);
    const double t = PulseDeltaDeg(currentPulse.nTPulse, targetPulse.nTPulse, axisUnit.dTPulseUnit);
    return std::max({ r, b, t });
}

bool TryBuildWeldSafeCoors(
    const QVector<WeldPoseFileRecord>& records,
    int pointIndex,
    double safeOffsetDistanceMm,
    int robotType,
    T_ROBOT_COORS& safeCoors,
    QString& error)
{
    error.clear();
    if (records.size() < 2)
    {
        error = "焊接姿态点不足 2 个，无法计算安全位置。";
        return false;
    }
    if (pointIndex < 0 || pointIndex >= records.size())
    {
        error = QString("安全位置锚点越界：%1").arg(pointIndex);
        return false;
    }

    QVector<Eigen::Vector3d> points;
    points.reserve(records.size());
    for (const WeldPoseFileRecord& record : records)
    {
        points.push_back(record.point);
    }

    const WeldPoseFileRecord& anchor = records[pointIndex];
    const Eigen::Vector3d seamDirection = ResolveHorizontalTangentDirection(points, pointIndex);
    if (seamDirection.head<2>().norm() <= 1e-9)
    {
        error = QString("第 %1 个焊点附近焊道方向无效，无法计算安全位置。").arg(pointIndex + 1);
        return false;
    }

    Eigen::Vector3d lateralDirection = HorizontalUnitOrZero(
        Eigen::Vector3d::UnitZ().cross(seamDirection));
    if (lateralDirection.head<2>().norm() <= 1e-9)
    {
        error = QString("第 %1 个焊点附近横向法向无效，无法计算安全位置。").arg(pointIndex + 1);
        return false;
    }

    Eigen::Vector3d safeGunAxis = Eigen::Vector3d::UnitY();
    if (robotType == ROBOT_TYPE_STEP)
    {
        safeGunAxis = -safeGunAxis;
    }
    const Eigen::Vector3d gunDirection = HorizontalUnitOrZero(
        RobotPoseTransform::RotationFromAnglesDeg(anchor.rx, anchor.ry, anchor.rz, robotType)
        * safeGunAxis);
    if (gunDirection.head<2>().norm() > 1e-9
        && lateralDirection.head<2>().dot(gunDirection.head<2>()) < 0.0)
    {
        lateralDirection = -lateralDirection;
    }
    // 焊接下枪/收枪安全位现场约定从世界 X- 侧回撤，避免补偿后枪向判断把安全位推到 X+。
    if (lateralDirection.x() > 0.0)
    {
        lateralDirection = -lateralDirection;
    }

    const double safeOffsetDistance =
        std::isfinite(safeOffsetDistanceMm) && safeOffsetDistanceMm > 0.0
        ? safeOffsetDistanceMm
        : WELD_SAFE_OFFSET_DISTANCE_MM;
    const Eigen::Vector3d safeOffsetDirection =
        (Eigen::Vector3d::UnitZ() + lateralDirection).normalized();
    const Eigen::Vector3d safePoint =
        anchor.point + safeOffsetDirection * safeOffsetDistance;

    safeCoors = T_ROBOT_COORS(
        safePoint.x(),
        safePoint.y(),
        safePoint.z(),
        anchor.rx,
        anchor.ry,
        anchor.rz,
        anchor.bx,
        anchor.by,
        anchor.bz);
    return true;
}

bool BuildWeldPoseMoveInfos(
    const QVector<WeldPoseFileRecord>& records,
    double linearSpeedMmPerSec,
    std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    QString& error,
    const WeldPosePreset* preset = nullptr,
    double transitionLinearSpeed = 0.0,
    bool enableWeldProcess = false)
{
    moveInfos.clear();
    moveInfos.reserve(static_cast<size_t>(records.size()));

    const bool useWeldProcess = enableWeldProcess
        && preset != nullptr
        && preset->weldProcessLoaded;
    if (useWeldProcess && preset->transitionCurrentVoltageEnableMismatch)
    {
        error = "拐点过渡电流和过渡电压必须同时启用或同时关闭，请检查当前焊接工艺参数。";
        return false;
    }

    const int transitionApplyScope = preset != nullptr
        ? NormalizeTransitionApplyScope(preset->transitionApplyScope)
        : kTransitionScopeArcAndTransition;
    const auto recordTag = [](const WeldPoseFileRecord& record) -> QString
        {
            return (record.pointType + " " + record.segmentKind).trimmed().toLower();
        };
    const auto isArcRecord = [&](const WeldPoseFileRecord& record) -> bool
        {
            const QString tag = recordTag(record);
            return tag.contains("_arc") || tag.contains(QStringLiteral("圆弧"));
        };
    const auto isTransitionRecord = [&](const WeldPoseFileRecord& record) -> bool
        {
            const QString tag = recordTag(record);
            return tag.contains("transition") || tag.contains(QStringLiteral("过渡"));
        };
    const auto shouldUseTransitionParam = [&](const WeldPoseFileRecord& record) -> bool
        {
            const bool isArc = isArcRecord(record);
            const bool isTransition = isTransitionRecord(record);
            switch (transitionApplyScope)
            {
            case kTransitionScopeArc:
                return isArc;
            case kTransitionScopeTransition:
                return isTransition;
            default:
                return isArc || isTransition;
            }
        };

    int externalAxisPointCount = 0;
    for (const WeldPoseFileRecord& record : records)
    {
        if (std::abs(record.bx) > 1e-6 || std::abs(record.by) > 1e-6 || std::abs(record.bz) > 1e-6)
        {
            ++externalAxisPointCount;
        }

        const bool useTransitionParam = shouldUseTransitionParam(record);
        const bool useTransitionSpeed = preset != nullptr
            && preset->transitionSpeedEnabled
            && transitionLinearSpeed > 0.0
            && useTransitionParam;
        const bool useTransitionWeldParams = useWeldProcess
            && preset->transitionCurrentVoltageEnabled
            && useTransitionParam;

        T_ROBOT_MOVE_INFO moveInfo;
        moveInfo.nMoveType = MOVL;
        moveInfo.tCoord = T_ROBOT_COORS(
            record.point.x(),
            record.point.y(),
            record.point.z(),
            record.rx,
            record.ry,
            record.rz,
            record.bx,
            record.by,
            record.bz);
        moveInfo.tSpeed = T_ROBOT_MOVE_SPEED(
            useTransitionSpeed ? transitionLinearSpeed : linearSpeedMmPerSec,
            0.0,
            0.0);
        moveInfo.dOverlapRel = preset != nullptr ? preset->stepOverlapRel : 20.0;
        moveInfo.nMoveDevice = 0;
        moveInfo.nTrackNo = 0;
        moveInfo.adBasePosVar[0] = record.bx;
        moveInfo.adBasePosVar[1] = record.by;
        moveInfo.adBasePosVar[2] = record.bz;
        if (useWeldProcess)
        {
            moveInfo.bWeldProcessEnabled = true;
            moveInfo.bUseTransitionWeldParams = useTransitionWeldParams;
            moveInfo.dArcStartCurrent = preset->startArcCurrent;
            moveInfo.dArcStartVoltage = preset->startArcVoltage;
            moveInfo.dArcStartWaitTime = preset->startArcWaitTime;
            moveInfo.dWeldCurrent = useTransitionWeldParams
                ? preset->transitionCurrent
                : preset->weldCurrent;
            moveInfo.dWeldVoltage = useTransitionWeldParams
                ? preset->transitionVoltage
                : preset->weldVoltage;
            moveInfo.dWeldSpeedMmPerMin = (useTransitionSpeed && preset->transitionSpeedMmPerMin > 0.0)
                ? preset->transitionSpeedMmPerMin
                : preset->weldProcessSpeedMmPerMin;
            moveInfo.dArcEndCurrent = preset->stopArcCurrent;
            moveInfo.dArcEndVoltage = preset->stopArcVoltage;
            moveInfo.dArcEndWaitTime = preset->stopArcWaitTime;
        }
        moveInfos.push_back(moveInfo);
    }

    if (externalAxisPointCount > 0)
    {
        error = QString("焊接姿态文件包含 %1 个外部轴点位，但当前多点 TP 下发只支持 GP1 六轴点位，请先确认 BX/BY/BZ 是否应为 0。")
            .arg(externalAxisPointCount);
        moveInfos.clear();
        return false;
    }

    if (useWeldProcess && !moveInfos.empty())
    {
        moveInfos.front().bArcStartBeforeMove = true;
        moveInfos.back().bArcEndAfterMove = true;
    }

    return true;
}

double EstimateMoveInfosPathLengthMm(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
{
    if (moveInfos.size() < 2)
    {
        return 0.0;
    }

    double totalLengthMm = 0.0;
    for (size_t index = 1; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_COORS& prev = moveInfos[index - 1].tCoord;
        const T_ROBOT_COORS& curr = moveInfos[index].tCoord;
        const double dx = curr.dX - prev.dX;
        const double dy = curr.dY - prev.dY;
        const double dz = curr.dZ - prev.dZ;
        totalLengthMm += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return totalLengthMm;
}

struct WeldSeamCompApplyStats
{
    int zAdjustedCount = 0;
    int gunDirAdjustedCount = 0;
    int seamDirAdjustedCount = 0;
    int selfIntersectionTrimCount = 0;
    int selfIntersectionRemovedPointCount = 0;
    QSet<QString> usedSlots;
};

struct WeldPathIntersection
{
    int firstSegmentIndex = -1;
    int secondSegmentIndex = -1;
    double firstRatio = 0.0;
    double secondRatio = 0.0;
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
};

double Cross2d(const Eigen::Vector2d& left, const Eigen::Vector2d& right)
{
    return left.x() * right.y() - left.y() * right.x();
}

bool TryFindFirstWeldPathSelfIntersection(
    const QVector<WeldPoseFileRecord>& records,
    WeldPathIntersection& intersection)
{
    constexpr double kMinSegmentLengthMm = 1e-6;
    constexpr double kIntersectionEpsilon = 1e-6;
    intersection = WeldPathIntersection();

    if (records.size() < 4)
    {
        return false;
    }

    for (int firstIndex = 0; firstIndex + 1 < records.size(); ++firstIndex)
    {
        const Eigen::Vector2d firstBegin(
            records[firstIndex].point.x(),
            records[firstIndex].point.y());
        const Eigen::Vector2d firstEnd(
            records[firstIndex + 1].point.x(),
            records[firstIndex + 1].point.y());
        const Eigen::Vector2d firstDelta = firstEnd - firstBegin;
        if (firstDelta.norm() <= kMinSegmentLengthMm)
        {
            continue;
        }

        for (int secondIndex = firstIndex + 2; secondIndex + 1 < records.size(); ++secondIndex)
        {
            const Eigen::Vector2d secondBegin(
                records[secondIndex].point.x(),
                records[secondIndex].point.y());
            const Eigen::Vector2d secondEnd(
                records[secondIndex + 1].point.x(),
                records[secondIndex + 1].point.y());
            const Eigen::Vector2d secondDelta = secondEnd - secondBegin;
            if (secondDelta.norm() <= kMinSegmentLengthMm)
            {
                continue;
            }

            const double denominator = Cross2d(firstDelta, secondDelta);
            if (std::abs(denominator) <= 1e-9)
            {
                continue;
            }

            const Eigen::Vector2d beginDelta = secondBegin - firstBegin;
            const double firstRatio = Cross2d(beginDelta, secondDelta) / denominator;
            const double secondRatio = Cross2d(beginDelta, firstDelta) / denominator;
            if (firstRatio <= kIntersectionEpsilon
                || firstRatio >= 1.0 - kIntersectionEpsilon
                || secondRatio <= kIntersectionEpsilon
                || secondRatio >= 1.0 - kIntersectionEpsilon)
            {
                continue;
            }

            intersection.firstSegmentIndex = firstIndex;
            intersection.secondSegmentIndex = secondIndex;
            intersection.firstRatio = firstRatio;
            intersection.secondRatio = secondRatio;
            intersection.point = firstBegin + firstDelta * firstRatio;
            return true;
        }
    }

    return false;
}

WeldPoseFileRecord InterpolateWeldPoseRecord(
    const WeldPoseFileRecord& begin,
    const WeldPoseFileRecord& end,
    double ratio)
{
    const double safeRatio = std::clamp(ratio, 0.0, 1.0);
    WeldPoseFileRecord record = begin;
    record.rawIndex = static_cast<int>(std::lround(
        begin.rawIndex + (end.rawIndex - begin.rawIndex) * safeRatio));
    record.point = begin.point + (end.point - begin.point) * safeRatio;
    record.rx = begin.rx + (NormalizeAngleNear(end.rx, begin.rx) - begin.rx) * safeRatio;
    record.ry = begin.ry + (NormalizeAngleNear(end.ry, begin.ry) - begin.ry) * safeRatio;
    record.rz = NormalizeAngleToFanucRange(
        begin.rz + (NormalizeAngleNear(end.rz, begin.rz) - begin.rz) * safeRatio);
    record.bx = begin.bx + (end.bx - begin.bx) * safeRatio;
    record.by = begin.by + (end.by - begin.by) * safeRatio;
    record.bz = begin.bz + (end.bz - begin.bz) * safeRatio;
    if (safeRatio >= 0.5)
    {
        record.pointType = end.pointType;
        record.segmentKind = end.segmentKind;
    }
    return record;
}

void RenumberWeldPoseRecords(QVector<WeldPoseFileRecord>& records)
{
    for (int index = 0; index < records.size(); ++index)
    {
        records[index].weldIndex = index + 1;
    }
}

void TrimWeldPathSelfIntersections(
    QVector<WeldPoseFileRecord>& records,
    WeldSeamCompApplyStats& stats)
{
    constexpr int kMaxTrimIterations = 256;
    int iteration = 0;

    WeldPathIntersection intersection;
    while (iteration < kMaxTrimIterations
        && TryFindFirstWeldPathSelfIntersection(records, intersection))
    {
        const int firstIndex = intersection.firstSegmentIndex;
        const int secondIndex = intersection.secondSegmentIndex;
        if (firstIndex < 0
            || secondIndex <= firstIndex + 1
            || secondIndex + 1 >= records.size())
        {
            break;
        }

        WeldPoseFileRecord junctionRecord = InterpolateWeldPoseRecord(
            records[secondIndex],
            records[secondIndex + 1],
            intersection.secondRatio);
        junctionRecord.point.x() = intersection.point.x();
        junctionRecord.point.y() = intersection.point.y();

        records[firstIndex + 1] = junctionRecord;
        const int eraseBegin = firstIndex + 2;
        const int eraseEndExclusive = secondIndex + 1;
        if (eraseBegin < eraseEndExclusive)
        {
            stats.selfIntersectionRemovedPointCount += eraseEndExclusive - eraseBegin;
            records.erase(records.begin() + eraseBegin, records.begin() + eraseEndExclusive);
        }

        ++stats.selfIntersectionTrimCount;
        ++iteration;
    }

    if (stats.selfIntersectionTrimCount > 0)
    {
        RenumberWeldPoseRecords(records);
    }
}

struct WeldCornerArcApplyStats
{
    int inputPointCount = 0;
    int outputPointCount = 0;
    int roundedCornerCount = 0;
    double radiusMm = 0.0;

    int insertedPointCount() const
    {
        return outputPointCount - inputPointCount;
    }
};

bool IsWeldCornerPointType(const QString& pointType)
{
    const QString normalized = pointType.trimmed().toLower();
    return normalized.contains("corner") || normalized.contains(QStringLiteral("拐"));
}

bool IsWeldSegmentBoundaryAtCorner(
    const WeldPoseFileRecord& prev,
    const WeldPoseFileRecord& corner,
    const WeldPoseFileRecord& next)
{
    const QString prevKind = NormalizeSeamCompSegmentKind(prev.segmentKind).trimmed();
    const QString cornerKind = NormalizeSeamCompSegmentKind(corner.segmentKind).trimmed();
    const QString nextKind = NormalizeSeamCompSegmentKind(next.segmentKind).trimmed();
    if (prevKind.isEmpty() || cornerKind.isEmpty() || nextKind.isEmpty())
    {
        return false;
    }

    const bool startsNewSegment =
        prevKind.compare(cornerKind, Qt::CaseInsensitive) != 0
        && cornerKind.compare(nextKind, Qt::CaseInsensitive) == 0;
    if (startsNewSegment)
    {
        return true;
    }

    // 段切换前的最后一个点不是几何拐点，避免和真正拐点连续圆滑两次。
    const bool endsOldSegment =
        prevKind.compare(cornerKind, Qt::CaseInsensitive) == 0
        && cornerKind.compare(nextKind, Qt::CaseInsensitive) != 0;
    return !endsOldSegment
        && prevKind.compare(nextKind, Qt::CaseInsensitive) != 0;
}

double EstimateWeldPoseStepMm(const QVector<WeldPoseFileRecord>& records)
{
    QVector<double> lengths;
    lengths.reserve(records.size() > 1 ? records.size() - 1 : 0);
    for (int index = 1; index < records.size(); ++index)
    {
        const double length = (records[index].point - records[index - 1].point).norm();
        if (std::isfinite(length) && length > 1e-6)
        {
            lengths.push_back(length);
        }
    }

    if (lengths.isEmpty())
    {
        return 2.0;
    }

    std::sort(lengths.begin(), lengths.end());
    return std::clamp(lengths[lengths.size() / 2], 0.5, 5.0);
}

QString WeldArcSegmentKind(const QString& segmentKind)
{
    QString normalized = NormalizeSeamCompSegmentKind(segmentKind).trimmed();
    if (normalized.isEmpty())
    {
        normalized = "arc";
    }
    if (!normalized.endsWith("_arc", Qt::CaseInsensitive))
    {
        normalized += "_arc";
    }
    return normalized;
}

void AppendWeldPoseRecordIfNotDuplicate(
    QVector<WeldPoseFileRecord>& records,
    const WeldPoseFileRecord& record)
{
    constexpr double kDuplicateDistanceMm = 1e-6;
    if (!records.isEmpty()
        && (records.back().point - record.point).norm() <= kDuplicateDistanceMm)
    {
        records.back() = record;
        return;
    }

    records.push_back(record);
}

double WeldPoseTurnAngleRad(const Eigen::Vector3d& incoming, const Eigen::Vector3d& outgoing)
{
    const double incomingLength = incoming.norm();
    const double outgoingLength = outgoing.norm();
    if (incomingLength <= 1e-9 || outgoingLength <= 1e-9)
    {
        return 0.0;
    }

    const double cosTheta = std::clamp(
        incoming.dot(outgoing) / (incomingLength * outgoingLength),
        -0.999999,
        0.999999);
    return std::acos(cosTheta);
}

void TrimSharpEntryPointBeforeWeldArc(
    QVector<WeldPoseFileRecord>& records,
    const Eigen::Vector3d& tangentIn,
    double maxEntryAngleRad,
    double maxTrimDistanceMm)
{
    int trimmedCount = 0;
    while (records.size() >= 2 && trimmedCount < 3)
    {
        const WeldPoseFileRecord& last = records.back();
        if (last.pointType.contains("_arc", Qt::CaseInsensitive))
        {
            break;
        }

        const Eigen::Vector3d incoming = last.point - records[records.size() - 2].point;
        const Eigen::Vector3d outgoing = tangentIn - last.point;
        const double outgoingDistanceMm = outgoing.norm();
        if (outgoingDistanceMm <= 0.5)
        {
            records.removeLast();
            ++trimmedCount;
            continue;
        }

        if (outgoingDistanceMm > maxTrimDistanceMm)
        {
            break;
        }

        if (WeldPoseTurnAngleRad(incoming, outgoing) <= maxEntryAngleRad)
        {
            break;
        }

        records.removeLast();
        ++trimmedCount;
    }
}

bool IsWeldPoseArcRecord(const WeldPoseFileRecord& record)
{
    return record.pointType.contains(QStringLiteral("_arc"), Qt::CaseInsensitive)
        || record.segmentKind.contains(QStringLiteral("_arc"), Qt::CaseInsensitive);
}

void TrimSharpWeldArcEntryPoints(
    QVector<WeldPoseFileRecord>& records,
    double maxEntryAngleRad,
    double maxTrimDistanceMm)
{
    for (int index = 1; index < records.size(); ++index)
    {
        if (!IsWeldPoseArcRecord(records[index])
            || IsWeldPoseArcRecord(records[index - 1]))
        {
            continue;
        }

        int trimmedCount = 0;
        while (index >= 2 && trimmedCount < 3)
        {
            const int previousIndex = index - 1;
            if (IsWeldPoseArcRecord(records[previousIndex]))
            {
                break;
            }

            const Eigen::Vector3d incoming =
                records[previousIndex].point - records[previousIndex - 1].point;
            const Eigen::Vector3d outgoing =
                records[index].point - records[previousIndex].point;
            const double outgoingDistanceMm = outgoing.norm();

            if (outgoingDistanceMm > maxTrimDistanceMm)
            {
                break;
            }

            // 圆弧入口前如果残留一颗很近的原始点，视觉上会变成硬折角。
            if (outgoingDistanceMm > 0.5
                && WeldPoseTurnAngleRad(incoming, outgoing) <= maxEntryAngleRad)
            {
                break;
            }

            records.removeAt(previousIndex);
            --index;
            ++trimmedCount;
        }
    }
}

struct WeldCornerCandidateInfo
{
    bool isCandidate = false;
    bool markedCorner = false;
    double theta = 0.0;
    Eigen::Vector3d incomingDir = Eigen::Vector3d::Zero();
    Eigen::Vector3d outgoingDir = Eigen::Vector3d::Zero();
};

struct WeldPolylineCutPoint
{
    bool valid = false;
    int segmentBeginIndex = -1;
    int segmentEndIndex = -1;
    double ratio = 0.0;
    WeldPoseFileRecord record;
};

int FindWeldPoseProbeIndexBefore(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    double probeDistanceMm)
{
    double accumulatedMm = 0.0;
    for (int index = cornerIndex; index > 0; --index)
    {
        accumulatedMm += (records[index].point - records[index - 1].point).norm();
        if (accumulatedMm >= probeDistanceMm)
        {
            return index - 1;
        }
    }
    return 0;
}

int FindWeldPoseProbeIndexAfter(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    double probeDistanceMm)
{
    double accumulatedMm = 0.0;
    for (int index = cornerIndex; index + 1 < records.size(); ++index)
    {
        accumulatedMm += (records[index + 1].point - records[index].point).norm();
        if (accumulatedMm >= probeDistanceMm)
        {
            return index + 1;
        }
    }
    return records.size() - 1;
}

WeldCornerCandidateInfo EvaluateWeldCornerCandidate(
    const QVector<WeldPoseFileRecord>& records,
    int index,
    double minMarkedCornerAngleRad,
    double autoCornerAngleRad,
    double maxCornerAngleRad,
    double minSegmentLengthMm,
    double directionProbeDistanceMm)
{
    WeldCornerCandidateInfo info;
    if (index <= 0 || index + 1 >= records.size())
    {
        return info;
    }

    const int prevProbeIndex = FindWeldPoseProbeIndexBefore(
        records,
        index,
        directionProbeDistanceMm);
    const int nextProbeIndex = FindWeldPoseProbeIndexAfter(
        records,
        index,
        directionProbeDistanceMm);
    if (prevProbeIndex >= index || nextProbeIndex <= index)
    {
        return info;
    }

    const WeldPoseFileRecord& prev = records[index - 1];
    const WeldPoseFileRecord& corner = records[index];
    const WeldPoseFileRecord& next = records[index + 1];
    const Eigen::Vector3d incoming = corner.point - records[prevProbeIndex].point;
    const Eigen::Vector3d outgoing = records[nextProbeIndex].point - corner.point;
    const double incomingLength = incoming.norm();
    const double outgoingLength = outgoing.norm();
    if (incomingLength <= minSegmentLengthMm || outgoingLength <= minSegmentLengthMm)
    {
        return info;
    }

    info.incomingDir = incoming / incomingLength;
    info.outgoingDir = outgoing / outgoingLength;
    const double cosTheta = std::clamp(info.incomingDir.dot(info.outgoingDir), -0.999999, 0.999999);
    info.theta = std::acos(cosTheta);
    info.markedCorner = IsWeldCornerPointType(corner.pointType)
        || IsWeldSegmentBoundaryAtCorner(prev, corner, next);

    const double minAngle = info.markedCorner
        ? minMarkedCornerAngleRad
        : autoCornerAngleRad;
    info.isCandidate = info.theta >= minAngle
        && info.theta <= maxCornerAngleRad;
    return info;
}

double AccumulateWeldPoseDistanceBetween(
    const QVector<WeldPoseFileRecord>& records,
    int fromIndex,
    int toIndex)
{
    if (fromIndex == toIndex)
    {
        return 0.0;
    }

    const int beginIndex = std::min(fromIndex, toIndex);
    const int endIndex = std::max(fromIndex, toIndex);
    double distanceMm = 0.0;
    for (int index = beginIndex; index < endIndex; ++index)
    {
        distanceMm += (records[index + 1].point - records[index].point).norm();
    }
    return distanceMm;
}

void KeepBestWeldCornerCandidateRun(
    QVector<WeldCornerCandidateInfo>& candidates,
    const QVector<int>& runIndices)
{
    if (runIndices.size() <= 1)
    {
        return;
    }

    int bestIndex = runIndices.front();
    double bestScore = -1.0;
    for (const int index : runIndices)
    {
        const WeldCornerCandidateInfo& candidate = candidates[index];
        const double score = candidate.theta + (candidate.markedCorner ? 10.0 : 0.0);
        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = index;
        }
    }

    for (const int index : runIndices)
    {
        if (index != bestIndex)
        {
            candidates[index].isCandidate = false;
        }
    }
}

void SuppressShortBridgeWeldCornerCandidates(
    const QVector<WeldPoseFileRecord>& records,
    QVector<WeldCornerCandidateInfo>& candidates,
    double mergeDistanceMm)
{
    QVector<int> runIndices;
    int previousCandidate = -1;
    for (int index = 1; index + 1 < records.size(); ++index)
    {
        if (!candidates[index].isCandidate)
        {
            continue;
        }

        if (previousCandidate >= 0
            && AccumulateWeldPoseDistanceBetween(records, previousCandidate, index) > mergeDistanceMm)
        {
            KeepBestWeldCornerCandidateRun(candidates, runIndices);
            runIndices.clear();
        }

        runIndices.push_back(index);
        previousCandidate = index;
    }

    KeepBestWeldCornerCandidateRun(candidates, runIndices);
}

double AccumulateWeldPoseDistanceBackward(
    const QVector<WeldPoseFileRecord>& records,
    int beginIndex,
    int stopIndex)
{
    double distanceMm = 0.0;
    for (int index = beginIndex; index > stopIndex; --index)
    {
        distanceMm += (records[index].point - records[index - 1].point).norm();
    }
    return distanceMm;
}

double AccumulateWeldPoseDistanceForward(
    const QVector<WeldPoseFileRecord>& records,
    int beginIndex,
    int stopIndex)
{
    double distanceMm = 0.0;
    for (int index = beginIndex; index < stopIndex; ++index)
    {
        distanceMm += (records[index + 1].point - records[index].point).norm();
    }
    return distanceMm;
}

struct WeldEndpointTrimStats
{
    int removedStartCount = 0;
    int removedEndCount = 0;
};

void TrimWeldPoseRecordEndpoints(
    const WeldPosePreset& preset,
    QVector<WeldPoseFileRecord>& records,
    WeldEndpointTrimStats& stats)
{
    stats = {};
    if (records.size() < 2)
    {
        return;
    }

    QVector<double> distanceFromStart(records.size(), 0.0);
    for (int index = 1; index < records.size(); ++index)
    {
        distanceFromStart[index] = distanceFromStart[index - 1]
            + (records[index].point - records[index - 1].point).norm();
    }

    QVector<double> distanceFromEnd(records.size(), 0.0);
    for (int index = records.size() - 2; index >= 0; --index)
    {
        distanceFromEnd[index] = distanceFromEnd[index + 1]
            + (records[index + 1].point - records[index].point).norm();
    }

    int startIndex = 0;
    if (preset.weldStartSkipDistance > 1e-6)
    {
        for (int index = 0; index < records.size(); ++index)
        {
            if (distanceFromStart[index] >= preset.weldStartSkipDistance)
            {
                startIndex = index;
                break;
            }
        }
    }

    // 起点不要落在过渡段内部；跳到过渡段结束后的第一个稳定点。
    while (startIndex + 1 < records.size() && IsWeldPoseTransitionRecord(records[startIndex]))
    {
        ++startIndex;
    }

    int endIndex = records.size() - 1;
    if (preset.weldEndSkipDistance > 1e-6)
    {
        for (int index = records.size() - 1; index >= 0; --index)
        {
            if (distanceFromEnd[index] >= preset.weldEndSkipDistance)
            {
                endIndex = index;
                break;
            }
        }
    }

    // 终点同样避开过渡段内部，退回到过渡段开始前的稳定点。
    while (endIndex > startIndex && IsWeldPoseTransitionRecord(records[endIndex]))
    {
        --endIndex;
    }

    if (startIndex <= 0 && endIndex >= records.size() - 1)
    {
        return;
    }

    if (startIndex > endIndex)
    {
        stats.removedStartCount = records.size();
        records.clear();
        return;
    }

    stats.removedStartCount = startIndex;
    stats.removedEndCount = records.size() - 1 - endIndex;
    records = records.mid(startIndex, endIndex - startIndex + 1);
    RenumberWeldPoseRecords(records);
}

bool TryFindWeldPoseCutBefore(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    int minIndex,
    double targetDistanceMm,
    WeldPolylineCutPoint& cutPoint)
{
    constexpr double kEpsilon = 1e-9;
    if (targetDistanceMm <= kEpsilon || cornerIndex <= minIndex)
    {
        return false;
    }

    double remainingDistanceMm = targetDistanceMm;
    for (int index = cornerIndex; index > minIndex; --index)
    {
        const double segmentLengthMm = (records[index].point - records[index - 1].point).norm();
        if (segmentLengthMm <= kEpsilon)
        {
            continue;
        }

        if (remainingDistanceMm <= segmentLengthMm + kEpsilon)
        {
            const double ratio = std::clamp(
                (segmentLengthMm - remainingDistanceMm) / segmentLengthMm,
                0.0,
                1.0);
            cutPoint.valid = true;
            cutPoint.segmentBeginIndex = index - 1;
            cutPoint.segmentEndIndex = index;
            cutPoint.ratio = ratio;
            cutPoint.record = InterpolateWeldPoseRecord(records[index - 1], records[index], ratio);
            return true;
        }

        remainingDistanceMm -= segmentLengthMm;
    }

    return false;
}

bool TryFindWeldPoseCutAfter(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    int maxIndex,
    double targetDistanceMm,
    WeldPolylineCutPoint& cutPoint)
{
    constexpr double kEpsilon = 1e-9;
    if (targetDistanceMm <= kEpsilon || cornerIndex >= maxIndex)
    {
        return false;
    }

    double remainingDistanceMm = targetDistanceMm;
    for (int index = cornerIndex; index < maxIndex; ++index)
    {
        const double segmentLengthMm = (records[index + 1].point - records[index].point).norm();
        if (segmentLengthMm <= kEpsilon)
        {
            continue;
        }

        if (remainingDistanceMm <= segmentLengthMm + kEpsilon)
        {
            const double ratio = std::clamp(
                remainingDistanceMm / segmentLengthMm,
                0.0,
                1.0);
            cutPoint.valid = true;
            cutPoint.segmentBeginIndex = index;
            cutPoint.segmentEndIndex = index + 1;
            cutPoint.ratio = ratio;
            cutPoint.record = InterpolateWeldPoseRecord(records[index], records[index + 1], ratio);
            return true;
        }

        remainingDistanceMm -= segmentLengthMm;
    }

    return false;
}

WeldCornerArcApplyStats ApplyCornerArcTransitionToWeldPoseRecords(
    const WeldPosePreset& preset,
    QVector<WeldPoseFileRecord>& records)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinMarkedCornerAngleRad = 5.0 * kPi / 180.0;
    constexpr double kAutoCornerAngleRad = 30.0 * kPi / 180.0;
    constexpr double kMaxCornerAngleRad = 165.0 * kPi / 180.0;
    constexpr double kMinEnabledArcRadiusMm = 2.0;
    constexpr double kMinSegmentLengthMm = 1e-6;

    WeldCornerArcApplyStats stats;
    stats.inputPointCount = records.size();
    stats.outputPointCount = records.size();
    const double effectiveRadiusMm = preset.cornerArcRadiusMm > 0.0
        ? std::max(kMinEnabledArcRadiusMm, preset.cornerArcRadiusMm)
        : 0.0;
    stats.radiusMm = effectiveRadiusMm;

    if (records.size() < 3 || effectiveRadiusMm <= 0.0)
    {
        return stats;
    }

    const double sampleStepMm = EstimateWeldPoseStepMm(records);
    const double directionProbeDistanceMm = std::clamp(
        std::max(sampleStepMm * 1.5, effectiveRadiusMm * 0.25),
        1.0,
        5.0);
    const double maxEntryTrimAngleRad = 8.0 * kPi / 180.0;
    const double maxEntryTrimDistanceMm = std::max(
        2.0,
        std::max(sampleStepMm * 2.5, effectiveRadiusMm * 0.35));
    const double shortBridgeMergeDistanceMm = std::max(1.0, sampleStepMm * 0.75);
    QVector<WeldCornerCandidateInfo> candidates(records.size());

    for (int index = 1; index + 1 < records.size(); ++index)
    {
        candidates[index] = EvaluateWeldCornerCandidate(
            records,
            index,
            kMinMarkedCornerAngleRad,
            kAutoCornerAngleRad,
            kMaxCornerAngleRad,
            kMinSegmentLengthMm,
            directionProbeDistanceMm);
    }

    SuppressShortBridgeWeldCornerCandidates(
        records,
        candidates,
        shortBridgeMergeDistanceMm);

    QVector<int> previousCandidateIndex(records.size(), 0);
    QVector<int> nextCandidateIndex(records.size(), records.size() - 1);

    int lastCandidateIndex = 0;
    for (int index = 1; index + 1 < records.size(); ++index)
    {
        previousCandidateIndex[index] = lastCandidateIndex;
        if (candidates[index].isCandidate)
        {
            lastCandidateIndex = index;
        }
    }

    lastCandidateIndex = records.size() - 1;
    for (int index = records.size() - 2; index > 0; --index)
    {
        nextCandidateIndex[index] = lastCandidateIndex;
        if (candidates[index].isCandidate)
        {
            lastCandidateIndex = index;
        }
    }

    QVector<WeldPoseFileRecord> roundedRecords;
    roundedRecords.reserve(records.size() + records.size() / 4);
    int copyIndex = 0;

    for (int index = 1; index + 1 < records.size(); ++index)
    {
        const WeldCornerCandidateInfo& candidate = candidates[index];
        if (!candidate.isCandidate || index <= copyIndex)
        {
            continue;
        }

        const WeldPoseFileRecord& corner = records[index];

        const double tanHalf = std::tan(candidate.theta * 0.5);
        if (!std::isfinite(tanHalf) || std::abs(tanHalf) <= 1e-9)
        {
            continue;
        }

        // theta is the path deflection angle. The fillet tangent distance is
        // r * tan(theta / 2), not r / tan(theta / 2).
        double tangentDistanceMm = effectiveRadiusMm * tanHalf;
        const int incomingLimitIndex = std::max(previousCandidateIndex[index], copyIndex);
        const int outgoingLimitIndex = nextCandidateIndex[index];
        const double incomingAvailableMm = AccumulateWeldPoseDistanceBackward(
            records,
            index,
            incomingLimitIndex);
        const double outgoingAvailableMm = AccumulateWeldPoseDistanceForward(
            records,
            index,
            outgoingLimitIndex);
        const double maxTangentDistanceMm = std::min(incomingAvailableMm, outgoingAvailableMm) * 0.45;
        if (maxTangentDistanceMm <= kMinSegmentLengthMm)
        {
            continue;
        }
        if (tangentDistanceMm > maxTangentDistanceMm)
        {
            tangentDistanceMm = maxTangentDistanceMm;
        }

        const double actualRadiusMm = tangentDistanceMm / tanHalf;
        if (!std::isfinite(actualRadiusMm) || actualRadiusMm < kMinEnabledArcRadiusMm)
        {
            continue;
        }

        WeldPolylineCutPoint tangentInCut;
        WeldPolylineCutPoint tangentOutCut;
        if (!TryFindWeldPoseCutBefore(
                records,
                index,
                incomingLimitIndex,
                tangentDistanceMm,
                tangentInCut)
            || !TryFindWeldPoseCutAfter(
                records,
                index,
                outgoingLimitIndex,
                tangentDistanceMm,
                tangentOutCut)
            || !tangentInCut.valid
            || !tangentOutCut.valid)
        {
            continue;
        }

        const double cosTheta = std::clamp(
            candidate.incomingDir.dot(candidate.outgoingDir),
            -0.999999,
            0.999999);
        const Eigen::Vector3d normalComponent =
            candidate.outgoingDir - candidate.incomingDir * cosTheta;
        const double normalLength = normalComponent.norm();
        if (normalLength <= 1e-9)
        {
            continue;
        }

        const Eigen::Vector3d planeX = candidate.incomingDir;
        const Eigen::Vector3d planeY = normalComponent / normalLength;
        const Eigen::Vector3d tangentIn = tangentInCut.record.point;
        const Eigen::Vector3d tangentOut = tangentOutCut.record.point;
        const Eigen::Vector3d center = tangentIn + planeY * actualRadiusMm;
        const Eigen::Vector3d startVector = tangentIn - center;
        const double startAngle = std::atan2(startVector.dot(planeY), startVector.dot(planeX));
        const double arcLengthMm = actualRadiusMm * candidate.theta;
        const int stepCount = std::max(2, static_cast<int>(std::ceil(arcLengthMm / sampleStepMm)));
        const QString arcSegmentKind = WeldArcSegmentKind(corner.segmentKind);
        const QString arcPointType = corner.pointType.trimmed().isEmpty()
            ? QStringLiteral("arc")
            : (corner.pointType + "_arc");

        while (copyIndex <= tangentInCut.segmentBeginIndex)
        {
            AppendWeldPoseRecordIfNotDuplicate(roundedRecords, records[copyIndex]);
            ++copyIndex;
        }
        // 圆弧切入点前如果残留一个很近的尖角点，会让视觉上仍像没有圆滑过渡。
        TrimSharpEntryPointBeforeWeldArc(
            roundedRecords,
            tangentIn,
            maxEntryTrimAngleRad,
            maxEntryTrimDistanceMm);

        for (int arcIndex = 0; arcIndex <= stepCount; ++arcIndex)
        {
            const double ratio = static_cast<double>(arcIndex) / static_cast<double>(stepCount);
            WeldPoseFileRecord arcRecord = InterpolateWeldPoseRecord(
                tangentInCut.record,
                tangentOutCut.record,
                ratio);
            const double angle = startAngle + candidate.theta * ratio;
            arcRecord.point = center
                + planeX * (std::cos(angle) * actualRadiusMm)
                + planeY * (std::sin(angle) * actualRadiusMm);
            if (arcIndex == stepCount)
            {
                arcRecord.point = tangentOut;
            }
            else if (arcIndex == 0)
            {
                arcRecord.point = tangentIn;
            }
            arcRecord.pointType = arcPointType;
            arcRecord.segmentKind = arcSegmentKind;
            AppendWeldPoseRecordIfNotDuplicate(roundedRecords, arcRecord);
        }

        copyIndex = std::max(copyIndex, tangentOutCut.segmentEndIndex);
        ++stats.roundedCornerCount;
    }

    while (copyIndex < records.size())
    {
        AppendWeldPoseRecordIfNotDuplicate(roundedRecords, records[copyIndex]);
        ++copyIndex;
    }

    TrimSharpWeldArcEntryPoints(
        roundedRecords,
        maxEntryTrimAngleRad,
        maxEntryTrimDistanceMm);

    records = std::move(roundedRecords);
    RenumberWeldPoseRecords(records);
    stats.outputPointCount = records.size();
    return stats;
}

int DensifyWeldPoseRecordsByStep(
    QVector<WeldPoseFileRecord>& records,
    double maxStepMm)
{
    if (records.size() < 2 || !std::isfinite(maxStepMm) || maxStepMm <= 0.0)
    {
        return 0;
    }

    QVector<WeldPoseFileRecord> denseRecords;
    denseRecords.reserve(records.size());
    denseRecords.push_back(records.front());

    int insertedPointCount = 0;
    for (int index = 1; index < records.size(); ++index)
    {
        const WeldPoseFileRecord& prev = records[index - 1];
        const WeldPoseFileRecord& next = records[index];
        const double distanceMm = (next.point - prev.point).norm();
        if (!std::isfinite(distanceMm) || distanceMm <= 1e-6)
        {
            AppendWeldPoseRecordIfNotDuplicate(denseRecords, next);
            continue;
        }

        const int segmentCount = std::max(
            1,
            static_cast<int>(std::ceil(distanceMm / maxStepMm)));
        for (int segmentIndex = 1; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double ratio = static_cast<double>(segmentIndex)
                / static_cast<double>(segmentCount);
            WeldPoseFileRecord fillRecord = InterpolateWeldPoseRecord(prev, next, ratio);
            fillRecord.pointType = next.pointType;
            fillRecord.segmentKind = next.segmentKind;
            AppendWeldPoseRecordIfNotDuplicate(denseRecords, fillRecord);
            ++insertedPointCount;
        }
        AppendWeldPoseRecordIfNotDuplicate(denseRecords, next);
    }

    if (insertedPointCount > 0)
    {
        records = std::move(denseRecords);
        RenumberWeldPoseRecords(records);
    }
    return insertedPointCount;
}

WeldSeamCompApplyStats ApplyWeldSeamCompToWeldPoseRecords(
    const WeldPosePreset& preset,
    QVector<WeldPoseFileRecord>& records)
{
    WeldSeamCompApplyStats stats;
    if (records.isEmpty())
    {
        return stats;
    }

    QVector<Eigen::Vector3d> basePoints;
    basePoints.reserve(records.size());
    for (const WeldPoseFileRecord& record : records)
    {
        basePoints.push_back(record.point);
    }

    Eigen::Vector3d previousGunCompDirection = Eigen::Vector3d::Zero();
    bool hasPreviousGunCompDirection = false;

    for (int index = 0; index < records.size(); ++index)
    {
        WeldPoseFileRecord& record = records[index];
        const WeldPosePreset::SeamCompSlot* seamCompSlot =
            FindSeamCompSlotForRecord(preset, record);
        if (seamCompSlot == nullptr)
        {
            continue;
        }

        stats.usedSlots.insert(seamCompSlot->segmentKind);

        if (std::abs(seamCompSlot->weldZComp) > 1e-6)
        {
            record.point.z() += seamCompSlot->weldZComp;
            ++stats.zAdjustedCount;
        }

        const Eigen::Vector3d seamDirection =
            ResolveHorizontalTangentDirection(basePoints, index);

        if (std::abs(seamCompSlot->weldGunDirComp) > 1e-6)
        {
            // 枪反向补偿直接使用焊道水平法向，避免 RZ 规则和 RZ 增益影响补偿方向。
            Eigen::Vector3d lateralDirection = HorizontalUnitOrZero(
                Eigen::Vector3d::UnitZ().cross(seamDirection));
            if (lateralDirection.head<2>().norm() > 1e-9)
            {
                if (hasPreviousGunCompDirection
                    && lateralDirection.head<2>().dot(previousGunCompDirection.head<2>()) < 0.0)
                {
                    lateralDirection = -lateralDirection;
                }

                record.point += lateralDirection * seamCompSlot->weldGunDirComp;
                previousGunCompDirection = lateralDirection;
                hasPreviousGunCompDirection = true;
                ++stats.gunDirAdjustedCount;
            }
        }

        if (std::abs(seamCompSlot->weldSeamDirComp) > 1e-6
            && seamDirection.head<2>().norm() > 1e-9)
        {
            record.point += seamDirection * seamCompSlot->weldSeamDirComp;
            ++stats.seamDirAdjustedCount;
        }
    }

    return stats;
}

std::vector<QString> BuildSegmentPoseOutputLines(
    const RobotCalculation::LowerWeldClassificationResult& result,
    const T_PRECISE_MEASURE_PARAM& param,
    const WeldPosePreset& preset,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    struct SegmentInfo
    {
        int begin = 0;
        int end = 0;
        int nextBegin = 0;
        int transitionBegin = std::numeric_limits<int>::max();
        double beginDistance = 0.0;
        double endDistance = 0.0;
        double transitionBeginDistance = std::numeric_limits<double>::max();
        RobotCalculation::LowerWeldPointType beginType = RobotCalculation::LowerWeldPointType::Normal;
        RobotCalculation::LowerWeldPointType endMarkerType = RobotCalculation::LowerWeldPointType::Normal;
        QString kind;
        double fixedRz = 0.0;
        double directionDeg = 0.0;
        double baseRz = 0.0;
        double normalReferenceDistance = 0.0;
        double rejectedRz = 0.0;
        double rawRzDeviationFromReference = 0.0;
        double clampedRzDeviationFromReference = 0.0;
        bool slopeRzClamped = false;
        bool directionValid = false;
        QVector<double> distanceToEnd;
    };

    struct PoseCompSlotAccumulator
    {
        bool hasValue = false;
        double totalWeight = 0.0;
        double totalRx = 0.0;
        double totalRy = 0.0;
        double totalRz = 0.0;
        double rzReference = 0.0;

        void Add(double rx, double ry, double rz, double weight)
        {
            const double safeWeight = std::max(1e-6, weight);
            if (!hasValue)
            {
                rzReference = rz;
                hasValue = true;
            }

            totalWeight += safeWeight;
            totalRx += rx * safeWeight;
            totalRy += ry * safeWeight;
            totalRz += NormalizeAngleNear(rz, rzReference) * safeWeight;
        }

        bool Resolve(double& rx, double& ry, double& rz) const
        {
            if (!hasValue || totalWeight <= 1e-6)
            {
                return false;
            }

            rx = totalRx / totalWeight;
            ry = totalRy / totalWeight;
            rz = NormalizeAngleToFanucRange(totalRz / totalWeight);
            return true;
        }
    };

    std::vector<QString> lines;
    if (result.points.isEmpty())
    {
        return lines;
    }

    lines.reserve(static_cast<size_t>(result.points.size()) + 2);
    lines.push_back("weld_index raw_index x y z rx ry rz bx by bz point_type segment_kind");

    std::vector<int> keyPointPositions;
    std::vector<RobotCalculation::LowerWeldPointType> keyPointTypes;
    keyPointPositions.reserve(result.points.size());
    keyPointTypes.reserve(result.points.size());
    for (int index = 0; index < result.points.size(); ++index)
    {
        const RobotCalculation::LowerWeldPointType pointType = result.points[index].type;
        if (pointType == RobotCalculation::LowerWeldPointType::Normal
            || pointType == RobotCalculation::LowerWeldPointType::Noise)
        {
            continue;
        }
        keyPointPositions.push_back(index);
        keyPointTypes.push_back(pointType);
    }

    if (keyPointPositions.empty())
    {
        return lines;
    }
    if (keyPointTypes.front() != RobotCalculation::LowerWeldPointType::Start)
    {
        keyPointPositions.insert(keyPointPositions.begin(), 0);
        keyPointTypes.insert(keyPointTypes.begin(), RobotCalculation::LowerWeldPointType::Start);
    }
    if (keyPointTypes.back() != RobotCalculation::LowerWeldPointType::End)
    {
        keyPointPositions.push_back(result.points.size() - 1);
        keyPointTypes.push_back(RobotCalculation::LowerWeldPointType::End);
    }
    if (keyPointPositions.size() < 2)
    {
        return lines;
    }

    if (appendLog)
    {
        appendLog("焊道RZ按焊道法向生成：RZ=0表示枪尖指向机器人X-，顺时针为正，180输出为-180；每段先计算焊道方向的两个垂直法向，再用本次测量姿态RZ选择唯一法向；坡面段再按夹紧范围限制相对上一段的偏转。");
    }

    QVector<double> distanceFromStart(result.points.size(), 0.0);
    for (int index = 1; index < result.points.size(); ++index)
    {
        distanceFromStart[index] = distanceFromStart[index - 1]
            + (result.points[index].point - result.points[index - 1].point).norm();
    }

    std::vector<SegmentInfo> segments;
    double previousSegmentRz = preset.measureReferenceRz;
    for (size_t segmentIndex = 0; segmentIndex + 1 < keyPointPositions.size(); ++segmentIndex)
    {
        SegmentInfo segment;
        segment.begin = keyPointPositions[segmentIndex];
        segment.beginType = keyPointTypes[segmentIndex];
        segment.endMarkerType = keyPointTypes[segmentIndex + 1];
        segment.nextBegin = keyPointPositions[segmentIndex + 1];
        segment.end = (segmentIndex + 2 < keyPointPositions.size())
            ? std::max(segment.begin, segment.nextBegin - 1)
            : std::max(segment.begin, segment.nextBegin);
        segment.kind = LowerWeldSegmentKindText(segment.beginType, segment.endMarkerType);

        bool segmentValid = false;
        const double segmentDirectionDeg = ComputeDirectionAngleDeg(
            result.points[segment.begin].point,
            result.points[segment.nextBegin].point,
            &segmentValid);
        segment.directionDeg = segmentDirectionDeg;
        segment.directionValid = segmentValid;
        double segmentRz = previousSegmentRz;
        if (segmentValid)
        {
            double rejectedRz = 0.0;
            double normalReferenceDistance = 0.0;
            const Eigen::Vector3d segmentVector =
                result.points[segment.nextBegin].point - result.points[segment.begin].point;
            const double normalRzDeg = ComputeLineNormalRobotRz(
                segmentVector,
                preset.measureReferenceRy,
                preset.measureReferenceRz,
                nullptr,
                &rejectedRz,
                &normalReferenceDistance);
            const double selectedRz = NormalizeAngleNear(normalRzDeg, previousSegmentRz);
            const double rawRzDeviation =
                selectedRz - previousSegmentRz;
            double clampedRzDeviation = rawRzDeviation;
            double baseRz = selectedRz;
            if (IsSlopeSegmentKind(segment.kind))
            {
                clampedRzDeviation = std::clamp(
                    rawRzDeviation,
                    preset.slopeRzMinDeg,
                    preset.slopeRzMaxDeg);
                baseRz = previousSegmentRz + clampedRzDeviation;
                segment.slopeRzClamped = std::abs(clampedRzDeviation - rawRzDeviation) > 1e-6;
                if (segment.slopeRzClamped && appendLog)
                {
                    appendLog(QString("斜面段 %1 RZ夹紧：相对上一段原始变化=%2 deg，夹紧后=%3 deg，范围=[%4, %5] deg")
                        .arg(segment.kind)
                        .arg(rawRzDeviation, 0, 'f', 3)
                        .arg(clampedRzDeviation, 0, 'f', 3)
                        .arg(preset.slopeRzMinDeg, 0, 'f', 3)
                        .arg(preset.slopeRzMaxDeg, 0, 'f', 3));
                }
            }

            segment.rawRzDeviationFromReference = rawRzDeviation;
            segment.clampedRzDeviationFromReference = clampedRzDeviation;
            segment.normalReferenceDistance = normalReferenceDistance;
            segment.rejectedRz = rejectedRz;
            segment.baseRz = NormalizeRobotRzOutputRange(baseRz);
            segmentRz = NormalizeAngleNear(baseRz, previousSegmentRz);
        }

        segment.fixedRz = NormalizeRobotRzOutputRange(segmentRz);
        segment.distanceToEnd.resize(segment.end - segment.begin + 1);
        double accumulatedDistance = 0.0;
        segment.distanceToEnd[segment.end - segment.begin] = 0.0;
        for (int index = segment.end - 1; index >= segment.begin; --index)
        {
            accumulatedDistance += (result.points[index + 1].point - result.points[index].point).norm();
            segment.distanceToEnd[index - segment.begin] = accumulatedDistance;
        }
        segment.beginDistance = distanceFromStart[segment.begin];
        segment.endDistance = distanceFromStart[segment.end];

        if (segmentIndex + 1 < segments.capacity())
        {
            // no-op, just silence accidental warnings in some configurations
        }

        if ((segmentIndex + 1) < (keyPointPositions.size() - 1) && preset.cornerTransitionLeadDistance > 1e-6)
        {
            segment.transitionBegin = segment.end;
            for (int index = segment.end; index >= segment.begin; --index)
            {
                const double remainingDistance = segment.distanceToEnd[index - segment.begin];
                if (remainingDistance >= preset.cornerTransitionLeadDistance)
                {
                    segment.transitionBegin = std::min(segment.end, index + 1);
                    segment.transitionBeginDistance = distanceFromStart[segment.transitionBegin];
                    break;
                }
                segment.transitionBegin = index;
            }
        }

        segments.push_back(segment);
        previousSegmentRz = segmentRz;
    }

    if (segments.empty())
    {
        return lines;
    }

    std::vector<WeldPosePreset::PoseCompSlot> poseCompSlots = preset.poseCompSlots;
    std::vector<PoseCompSlotAccumulator> poseCompAccumulators(poseCompSlots.size());
    for (const SegmentInfo& segment : segments)
    {
        const int slotIndex = DefaultPoseCompSlotIndex(segment.kind);
        if (slotIndex < 0 || slotIndex >= static_cast<int>(poseCompSlots.size()))
        {
            continue;
        }

        WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
        const double segmentLength = !segment.distanceToEnd.isEmpty()
            ? std::max(1.0, segment.distanceToEnd.front())
            : std::max(1.0, static_cast<double>(segment.end - segment.begin + 1));
        poseCompAccumulators[slotIndex].Add(
            preset.rx,
            preset.ry,
            segment.fixedRz,
            segmentLength);
    }

    for (int slotIndex = 0; slotIndex < static_cast<int>(poseCompSlots.size()); ++slotIndex)
    {
        WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
        slot.generatedReference = false;
        if (poseCompAccumulators[slotIndex].Resolve(slot.poseRx, slot.poseRy, slot.poseRz))
        {
            slot.poseRz = NormalizeAngleToFanucRange(slot.poseRz);
            slot.generatedReference = true;
            slot.validReference = true;
        }
        else
        {
            slot.validReference = slot.hasIniReference;
        }

        if (appendLog)
        {
            appendLog(QString("姿态补偿槽 %1 [%2]：姿态 RX=%3, RY=%4, RZ=%5, 补偿 dX=%6, dY=%7, dZ=%8, 参考来源=%9")
                .arg(slot.name)
                .arg(slot.segmentKind.isEmpty() ? QString("unassigned") : slot.segmentKind)
                .arg(slot.poseRx, 0, 'f', 3)
                .arg(slot.poseRy, 0, 'f', 3)
                .arg(slot.poseRz, 0, 'f', 3)
                .arg(slot.compX, 0, 'f', 3)
                .arg(slot.compY, 0, 'f', 3)
                .arg(slot.compZ, 0, 'f', 3)
                .arg(slot.generatedReference
                    ? QString("分段均值")
                    : (slot.hasIniReference ? QString("ini回退") : QString("未生成"))));
        }
    }

    const int lowPlatformSlotIndex = DefaultPoseCompSlotIndex("low_platform");
    const int highPlatformSlotIndex = DefaultPoseCompSlotIndex("high_platform");
    if (lowPlatformSlotIndex >= 0
        && highPlatformSlotIndex >= 0
        && lowPlatformSlotIndex < static_cast<int>(poseCompSlots.size())
        && highPlatformSlotIndex < static_cast<int>(poseCompSlots.size()))
    {
        WeldPosePreset::PoseCompSlot& lowPlatformSlot = poseCompSlots[lowPlatformSlotIndex];
        WeldPosePreset::PoseCompSlot& highPlatformSlot = poseCompSlots[highPlatformSlotIndex];
        if (lowPlatformSlot.validReference && highPlatformSlot.validReference)
        {
            const double lowHighPoseDistance = PoseDistanceDeg(
                highPlatformSlot.poseRx,
                highPlatformSlot.poseRy,
                highPlatformSlot.poseRz,
                lowPlatformSlot.poseRx,
                lowPlatformSlot.poseRy,
                lowPlatformSlot.poseRz);
            if (lowHighPoseDistance <= preset.poseMatchMaxErrorDeg)
            {
                highPlatformSlot.poseRx = lowPlatformSlot.poseRx;
                highPlatformSlot.poseRy = lowPlatformSlot.poseRy;
                highPlatformSlot.poseRz = lowPlatformSlot.poseRz;
                if (appendLog)
                {
                    appendLog(QString("姿态补偿槽 [%1] 与 [%2] 姿态差=%3 deg，小于复用阈值=%4 deg，统一复用 [%1] 的参考姿态。")
                        .arg(lowPlatformSlot.segmentKind)
                        .arg(highPlatformSlot.segmentKind)
                        .arg(lowHighPoseDistance, 0, 'f', 3)
                        .arg(preset.poseMatchMaxErrorDeg, 0, 'f', 3));
                }
            }
        }
    }

    if (appendLog)
    {
        appendLog(QString("姿态补偿匹配方式=%1。")
            .arg(PoseCompMatchModeDisplayName(preset.poseCompMatchMode)));
        if (NormalizePoseCompMatchMode(preset.poseCompMatchMode) == POSE_COMP_MATCH_BY_POSE)
        {
            appendLog(QString("姿态匹配最大误差阈值=%1 deg，超过该阈值则该点不做姿态补偿。")
                .arg(preset.poseMatchMaxErrorDeg, 0, 'f', 3));
        }
        else
        {
            appendLog("按四类段属性时，低平台/上升边/高平台/下降边分别匹配姿态补偿槽0/1/2/3。");
        }
    }

    for (const WeldPosePreset::SeamCompSlot& slot : preset.seamCompSlots)
    {
        if (appendLog)
        {
            appendLog(QString("焊道补偿槽 %1 [%2]：dZ=%3, dGunDir=%4, dSeamDir=%5, 来源=%6")
                .arg(slot.name)
                .arg(slot.segmentKind.isEmpty() ? QString("unassigned") : slot.segmentKind)
                .arg(slot.weldZComp, 0, 'f', 3)
                .arg(slot.weldGunDirComp, 0, 'f', 3)
                .arg(slot.weldSeamDirComp, 0, 'f', 3)
                .arg(preset.seamCompFromIni ? preset.seamCompFilePath : QString("默认值")));
        }
    }

    const int weldBeginCandidate = segments.front().begin;
    const int weldEndCandidate = segments.back().end;

    auto findSegmentIndex = [&segments](int pointIndex) -> int
    {
        for (int segmentIndex = 0; segmentIndex < static_cast<int>(segments.size()); ++segmentIndex)
        {
            if (pointIndex >= segments[segmentIndex].begin && pointIndex <= segments[segmentIndex].end)
            {
                return segmentIndex;
            }
        }
        return -1;
    };

    const int weldStartIndex = weldBeginCandidate;
    const int weldEndIndex = weldEndCandidate;
    if (appendLog
        && (preset.weldStartSkipDistance > 1e-6 || preset.weldEndSkipDistance > 1e-6))
    {
        appendLog(QString("起终点裁剪延后到焊道补偿和姿态补偿之后执行：StartSkip=%1mm, EndSkip=%2mm")
            .arg(preset.weldStartSkipDistance, 0, 'f', 3)
            .arg(preset.weldEndSkipDistance, 0, 'f', 3));
    }

    for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex)
    {
        const SegmentInfo& segment = segments[segmentIndex];
        const WeldPosePreset::SeamCompSlot* seamCompSlot =
            FindSeamCompSlotByKind(preset, segment.kind);
        if (seamCompSlot == nullptr)
        {
            seamCompSlot = FindSeamCompSlotByKind(preset, preset.seamKind);
        }
        const double segmentWeldZComp = seamCompSlot != nullptr ? seamCompSlot->weldZComp : 0.0;
        const double segmentWeldGunDirComp = seamCompSlot != nullptr ? seamCompSlot->weldGunDirComp : 0.0;
        const double segmentWeldSeamDirComp = seamCompSlot != nullptr ? seamCompSlot->weldSeamDirComp : 0.0;
        if (appendLog)
        {
            appendLog(QString("焊道姿态段 %1: 点[%2-%3], 固定RZ=%4 deg, 输出基础RZ=%5 deg, 测量参考RZ=%6 deg, 法向与测量参考夹角=%7 deg, 反向候选RZ=%8 deg, RZ原始偏差=%9 deg, RZ夹紧后=%10 deg, RX=%11 deg, RY=%12 deg, 焊道种类=%13, 过渡起点=%14, 起点跳过=%15 mm, 终点跳过=%16 mm, Z补偿=%17 mm, 枪向补偿=%18 mm, 焊道方向补偿=%19 mm")
                .arg(segment.kind)
                .arg(result.points[segment.begin].index)
                .arg(result.points[segment.end].index)
                .arg(segment.fixedRz, 0, 'f', 3)
                .arg(segment.baseRz, 0, 'f', 3)
                .arg(preset.measureReferenceRz, 0, 'f', 3)
                .arg(segment.normalReferenceDistance, 0, 'f', 3)
                .arg(segment.rejectedRz, 0, 'f', 3)
                .arg(segment.rawRzDeviationFromReference, 0, 'f', 3)
                .arg(segment.clampedRzDeviationFromReference, 0, 'f', 3)
                .arg(preset.rx, 0, 'f', 3)
                .arg(preset.ry, 0, 'f', 3)
                .arg(preset.seamKind)
                .arg(segment.transitionBegin == std::numeric_limits<int>::max()
                    ? QString("none")
                    : QString::number(result.points[segment.transitionBegin].index))
                .arg(preset.weldStartSkipDistance, 0, 'f', 3)
                .arg(preset.weldEndSkipDistance, 0, 'f', 3)
                .arg(segmentWeldZComp, 0, 'f', 3)
                .arg(segmentWeldGunDirComp, 0, 'f', 3)
                .arg(segmentWeldSeamDirComp, 0, 'f', 3));
        }
    }

    auto findNearestPoseCompSlot = [&poseCompSlots, &preset](double rx, double ry, double rz) -> int
    {
        int bestIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        for (int slotIndex = 0; slotIndex < static_cast<int>(poseCompSlots.size()); ++slotIndex)
        {
            const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
            if (!slot.validReference)
            {
                continue;
            }

            const double distance = PoseDistanceDeg(
                rx,
                ry,
                rz,
                slot.poseRx,
                slot.poseRy,
                slot.poseRz);
            if (distance > preset.poseMatchMaxErrorDeg)
            {
                continue;
            }
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = slotIndex;
            }
        }
        return bestIndex;
    };

    auto findPoseCompSlot = [&poseCompSlots, &preset, &findNearestPoseCompSlot](
        double rx,
        double ry,
        double rz,
        const QString& segmentKind) -> int
    {
        if (NormalizePoseCompMatchMode(preset.poseCompMatchMode) == POSE_COMP_MATCH_BY_SEGMENT_CODE)
        {
            const int slotIndex = DefaultPoseCompSlotIndex(segmentKind);
            if (slotIndex >= 0 && slotIndex < static_cast<int>(poseCompSlots.size()))
            {
                return slotIndex;
            }
            return -1;
        }

        return findNearestPoseCompSlot(rx, ry, rz);
    };

    QVector<double> sampleDistances;
    sampleDistances.reserve(static_cast<int>((weldEndIndex - weldStartIndex + 1) * 2));
    constexpr double kExpandedSampleStepMm = 2.0;
    for (double distance = distanceFromStart[weldStartIndex];
         distance <= distanceFromStart[weldEndIndex] + 1e-9;
         distance += kExpandedSampleStepMm)
    {
        sampleDistances.push_back(distance);
    }

    for (int pointIndex = weldStartIndex; pointIndex <= weldEndIndex; ++pointIndex)
    {
        const RobotCalculation::LowerWeldPointType pointType = result.points[pointIndex].type;
        if (pointType == RobotCalculation::LowerWeldPointType::Normal
            || pointType == RobotCalculation::LowerWeldPointType::Noise)
        {
            continue;
        }

        const double pointDistance = distanceFromStart[pointIndex];
        if (pointDistance >= distanceFromStart[weldStartIndex] - 1e-9
            && pointDistance <= distanceFromStart[weldEndIndex] + 1e-9)
        {
            sampleDistances.push_back(pointDistance);
        }
    }

    sampleDistances.push_back(distanceFromStart[weldStartIndex]);
    sampleDistances.push_back(distanceFromStart[weldEndIndex]);
    std::sort(sampleDistances.begin(), sampleDistances.end());
    sampleDistances.erase(std::unique(sampleDistances.begin(), sampleDistances.end(),
        [](double left, double right)
        {
            return std::abs(left - right) <= 1e-6;
        }), sampleDistances.end());

    auto samplePointTypeAtDistance = [&](double sampleDistance, int lowerIndex) -> RobotCalculation::LowerWeldPointType
    {
        for (int pointIndex = weldStartIndex; pointIndex <= weldEndIndex; ++pointIndex)
        {
            const RobotCalculation::LowerWeldPointType pointType = result.points[pointIndex].type;
            if (pointType == RobotCalculation::LowerWeldPointType::Normal
                || pointType == RobotCalculation::LowerWeldPointType::Noise)
            {
                continue;
            }

            if (std::abs(distanceFromStart[pointIndex] - sampleDistance) <= 1e-6)
            {
                return pointType;
            }
        }

        if (lowerIndex >= weldStartIndex && lowerIndex <= weldEndIndex)
        {
            return result.points[lowerIndex].type == RobotCalculation::LowerWeldPointType::Noise
                ? RobotCalculation::LowerWeldPointType::Normal
                : RobotCalculation::LowerWeldPointType::Normal;
        }
        return RobotCalculation::LowerWeldPointType::Normal;
    };

    auto samplePointOnPath = [&](double sampleDistance, int& sourceIndex, int& sourceNextIndex) -> Eigen::Vector3d
    {
        if (sampleDistance <= distanceFromStart[weldStartIndex] + 1e-9)
        {
            sourceIndex = weldStartIndex;
            sourceNextIndex = weldStartIndex;
            return result.points[weldStartIndex].point;
        }
        if (sampleDistance >= distanceFromStart[weldEndIndex] - 1e-9)
        {
            sourceIndex = weldEndIndex;
            sourceNextIndex = weldEndIndex;
            return result.points[weldEndIndex].point;
        }

        auto upperIt = std::upper_bound(
            distanceFromStart.begin() + weldStartIndex,
            distanceFromStart.begin() + weldEndIndex + 1,
            sampleDistance);
        int upperIndex = static_cast<int>(upperIt - distanceFromStart.begin());
        upperIndex = std::clamp(upperIndex, weldStartIndex + 1, weldEndIndex);
        const int lowerIndex = upperIndex - 1;
        const double beginDistance = distanceFromStart[lowerIndex];
        const double endDistance = distanceFromStart[upperIndex];
        sourceIndex = lowerIndex;
        sourceNextIndex = upperIndex;
        if (std::abs(endDistance - beginDistance) <= 1e-9)
        {
            return result.points[lowerIndex].point;
        }

        const double ratio = std::clamp((sampleDistance - beginDistance) / (endDistance - beginDistance), 0.0, 1.0);
        return result.points[lowerIndex].point
            + (result.points[upperIndex].point - result.points[lowerIndex].point) * ratio;
    };

    QVector<WeldPoseFileRecord> records;
    records.reserve(sampleDistances.size());

    int weldIndex = 1;
    for (double sampleDistance : sampleDistances)
    {
        if (sampleDistance < distanceFromStart[weldStartIndex] - 1e-9
            || sampleDistance > distanceFromStart[weldEndIndex] + 1e-9)
        {
            continue;
        }

        int sourceIndex = weldStartIndex;
        int sourceNextIndex = weldStartIndex;
        const Eigen::Vector3d sampledPoint = samplePointOnPath(sampleDistance, sourceIndex, sourceNextIndex);

        int segmentIndex = findSegmentIndex(sourceIndex);
        if (segmentIndex < 0)
        {
            segmentIndex = findSegmentIndex(sourceNextIndex);
        }
        if (segmentIndex < 0)
        {
            continue;
        }

        const SegmentInfo& segment = segments[segmentIndex];
        const bool hasNextSegment = segmentIndex + 1 < static_cast<int>(segments.size());
        const double nextSegmentRz = hasNextSegment
            ? NormalizeAngleNear(segments[segmentIndex + 1].fixedRz, segment.fixedRz)
            : segment.fixedRz;
        const bool inTransition = hasNextSegment
            && segment.transitionBeginDistance < std::numeric_limits<double>::max()
            && sampleDistance >= segment.transitionBeginDistance;

        double pointRz = segment.fixedRz;
        if (inTransition && preset.cornerTransitionLeadDistance > 1e-6)
        {
            const double remainingDistance = std::max(0.0, segment.endDistance - sampleDistance);
            const double transitionRatio = 1.0
                - (remainingDistance / preset.cornerTransitionLeadDistance);
            pointRz = segment.fixedRz
                + (nextSegmentRz - segment.fixedRz) * std::clamp(transitionRatio, 0.0, 1.0);
        }
        // Transition points still change angle. The target RZ at each side of
        // the transition comes from the segment weld normal, while slope
        // segments are clamped before interpolation.
        pointRz = NormalizeRobotRzOutputRange(pointRz + preset.weldRzGainDeg);

        const RobotCalculation::LowerWeldPointType pointType =
            samplePointTypeAtDistance(sampleDistance, sourceIndex);

        double pointRx = preset.rx;
        double pointRy = preset.ry;
        Eigen::Vector3d point = sampledPoint;
        const int poseCompSlotIndex = findPoseCompSlot(pointRx, pointRy, pointRz, segment.kind);
        if (poseCompSlotIndex >= 0)
        {
            const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[poseCompSlotIndex];
            const Eigen::Vector3d poseCompLocal(slot.compX, slot.compY, slot.compZ);
            if (poseCompLocal.norm() > 1e-9)
            {
                point += RobotPoseTransform::RotationFromAnglesDeg(pointRx, pointRy, pointRz, preset.robotType)
                    * poseCompLocal;
            }
        }

        WeldPoseFileRecord record;
        record.weldIndex = weldIndex++;
        record.rawIndex = result.points[sourceIndex].index;
        record.point = point;
        record.rx = pointRx;
        record.ry = pointRy;
        record.rz = pointRz;
        record.bx = param.tStartPos.dBX;
        record.by = param.tStartPos.dBY;
        record.bz = param.tStartPos.dBZ;
        record.pointType = RobotCalculation::LowerWeldPointTypeName(pointType);
        record.segmentKind = inTransition ? (segment.kind + "_transition") : segment.kind;
        records.push_back(record);
    }

    for (const WeldPoseFileRecord& record : records)
    {
        lines.push_back(BuildWeldPoseFileRecordLine(record));
    }

    return lines;
}
}

double MeasureThenWeldService::SafeSpeed(double value, double fallback)
{
    return value > 0.0 ? value : fallback;
}

namespace
{
double FanucLinearSpeedMmPerSecFromConfig(double speedMmPerMin, double fallbackMmPerSec = 1.0)
{
    if (speedMmPerMin <= 0.0)
    {
        return fallbackMmPerSec;
    }

    const double converted = speedMmPerMin / 60.0;
    return converted > 0.0 ? converted : fallbackMmPerSec;
}

double LinearCommandSpeedForRobot(RobotDriverAdaptor* pRobotDriver, double speedMmPerMin, double fallback)
{
    if (pRobotDriver != nullptr && dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr)
    {
        return FanucLinearSpeedMmPerSecFromConfig(speedMmPerMin, fallback);
    }
    return speedMmPerMin > 0.0 ? speedMmPerMin : fallback;
}

QString LinearCommandSpeedUnitText(RobotDriverAdaptor* pRobotDriver)
{
    return (pRobotDriver != nullptr && dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr)
        ? QStringLiteral("mm/sec")
        : QStringLiteral("mm/min");
}

QString RobotMotionStatusText(RobotDriverAdaptor* pRobotDriver)
{
    if (pRobotDriver == nullptr)
    {
        return QString();
    }

    QStringList details;
    const std::string lastError = pRobotDriver->GetLastRobotError();
    if (!lastError.empty())
    {
        details << QString("最近错误：%1").arg(DecodeRobotMessageText(lastError));
    }
    const std::string statusText = pRobotDriver->GetRobotStatusText();
    if (!statusText.empty() && statusText != lastError)
    {
        details << QString("当前状态：%1").arg(DecodeRobotMessageText(statusText));
    }
    return details.join("；");
}

bool WaitRobotMotionDone(
    RobotDriverAdaptor* pRobotDriver,
    const QString& name,
    const MeasureThenWeldService::LogCallback& appendLog,
    int startTimeoutMs = 3000,
    int finishTimeoutMs = 120000,
    int pollDelayMs = 100)
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
    {
        int lastState = 0;
        const bool doneOk = pFanucDriver->WaitStateDone(
            FANUC_MOTION_STATE_REG,
            1,
            10,
            20,
            startTimeoutMs,
            finishTimeoutMs,
            pollDelayMs,
            &lastState);
        if (appendLog)
        {
            appendLog(QString("运动结束：%1, R[%2]=%3, WaitStateDone=%4")
                .arg(name)
                .arg(FANUC_MOTION_STATE_REG)
                .arg(lastState)
                .arg(doneOk ? 1 : 0));
            if (!doneOk)
            {
                const QString detail = RobotMotionStatusText(pRobotDriver);
                if (!detail.isEmpty())
                {
                    appendLog(QString("运动异常：%1，%2").arg(name, detail));
                }
            }
        }
        return doneOk;
    }

    const int done = pRobotDriver->CheckRobotDone(pollDelayMs);
    if (appendLog)
    {
        appendLog(QString("运动结束：%1, CheckRobotDone=%2").arg(name).arg(done));
        if (done <= 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            if (!detail.isEmpty())
            {
                appendLog(QString("运动异常：%1，%2").arg(name, detail));
            }
        }
    }
    return done > 0;
}
}

bool MeasureThenWeldService::LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error) const
{
    if (pRobotDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    param = T_PRECISE_MEASURE_PARAM();
    param.sRobotName = pRobotDriver->m_sRobotName.empty() ? "RobotA" : pRobotDriver->m_sRobotName;

    const QString robotName = QString::fromStdString(param.sRobotName);
    QString ensureError;
    if (!RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &ensureError))
    {
        error = ensureError.isEmpty()
            ? QString("创建或打开测量焊接参数文件失败：%1").arg(RobotDataHelper::MeasureWeldParamPath(robotName))
            : ensureError;
        return false;
    }
    const QString iniPath = RobotDataHelper::MeasureWeldParamPath(robotName);
    if (!ConfigDatabase::HasIniFile(iniPath))
    {
        error = ensureError.isEmpty() ? QString("未找到参数文件：%1").arg(iniPath) : ensureError;
        return false;
    }

    param.sIniFilePath = ToUtf8StdString(iniPath);

    COPini ini;
    if (!ini.SetFileName(param.sIniFilePath))
    {
        error = QString("打开参数文件失败：%1").arg(iniPath);
        return false;
    }

    int useNo = 0;
    std::string groupName;
    ini.SetSectionName("MeasureWeldGroups");
    ini.ReadString(false, "UseGroupNo", &useNo);
    ini.ReadString(false, ToUtf8StdString(QString("Group%1Name").arg(useNo)), groupName);
    param.nParamGroupIndex = std::max(0, useNo);
    param.sParamGroupName = groupName.empty()
        ? QString("参数组%1").arg(param.nParamGroupIndex + 1)
        : QString::fromStdString(groupName);
    param.sSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldScanSectionName(param.nParamGroupIndex));
    param.sWeldSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldWeldSectionName(param.nParamGroupIndex));
    param.sWeldParamFilePath = ToUtf8StdString(iniPath);

    ini.SetSectionName(param.sSectionName);
    ini.ReadString(false, "ScanSpeed", &param.dScanSpeed);
    ini.ReadString(false, "RunSpeed", &param.dRunSpeed);
    ini.ReadString(false, "CameraReadFps", &param.dCameraReadFps);
    ini.ReadString(false, "CameraTimeOffsetMs", &param.dCameraTimeOffsetMs);
    ini.ReadString(false, "dAcc", &param.dAcc);
    ini.ReadString(false, "dDec", &param.dDec);

    COPini weldIni;
    COPini* pWeldIni = &ini;
    const QString weldParamPath = QString::fromStdString(param.sWeldParamFilePath.empty()
        ? param.sIniFilePath
        : param.sWeldParamFilePath);
    if (weldParamPath != iniPath)
    {
        if (!weldIni.SetFileName(ToUtf8StdString(weldParamPath)))
        {
            error = QString("打开焊接参数文件失败：%1").arg(weldParamPath);
            return false;
        }
        pWeldIni = &weldIni;
    }
    pWeldIni->SetSectionName(param.sWeldSectionName);
    int doActualWeld = 1;
    pWeldIni->ReadString(false, "WeldEnable", &doActualWeld);
    pWeldIni->ReadString(false, "WeldSpeedMmPerMin", &param.dWeldSpeedMmPerMin);
    pWeldIni->ReadString(false, "DryRunSpeedMmPerMin", &param.dDryRunSpeedMmPerMin);
    pWeldIni->ReadString(false, "WeldSafeMoveSpeedMmPerMin", &param.dWeldSafeMoveSpeedMmPerMin);
    pWeldIni->ReadString(false, "StepOverlapRel", &param.dStepOverlapRel);
    pWeldIni->ReadString(false, "WeldDirection", &param.nWeldDirection);
    pWeldIni->ReadString(false, "GunDownBackSafeDis", &param.dGunDownBackSafeDis);
    pWeldIni->ReadString(false, "WeldRzGainDeg", &param.dWeldRzGainDeg);
    pWeldIni->ReadString(false, "SlopeRzMinDeg", &param.dSlopeRzMinDeg);
    pWeldIni->ReadString(false, "SlopeRzMaxDeg", &param.dSlopeRzMaxDeg);
    param.bDoActualWeld = (doActualWeld != 0);

    ini.SetSectionName(param.sSectionName);
    int useComputedScanSafe = 1;
    ini.ReadString(false, "UseComputedScanSafe", &useComputedScanSafe);
    param.bUseComputedScanSafe = (useComputedScanSafe != 0);
    ini.ReadString(false, "ScanSafeOffsetDistanceMm", &param.dScanSafeOffsetDistanceMm);
    ini.ReadString(false, "ScanSafeGunAngleDeg", &param.dScanSafeGunAngleDeg);
    ini.ReadString(false, "ScanSafeXDirection", &param.nScanSafeXDirection);
    ini.ReadString(false, "ScanSafeLiftHeightMm", &param.dScanSafeLiftHeightMm);
    ini.ReadString(false, "ScanSafeFlipWarnThresholdDeg", &param.dScanSafeFlipWarnThresholdDeg);

    if (!std::isfinite(param.dCameraReadFps) || param.dCameraReadFps <= 0.0)
    {
        param.dCameraReadFps = DEFAULT_CAMERA_READ_FPS;
    }
    if (!std::isfinite(param.dCameraTimeOffsetMs))
    {
        param.dCameraTimeOffsetMs = 0.0;
    }
    if (!std::isfinite(param.dWeldSpeedMmPerMin) || param.dWeldSpeedMmPerMin <= 0.0)
    {
        param.dWeldSpeedMmPerMin = FANUC_WELD_PATH_SPEED_MM_PER_MIN;
    }
    if (!std::isfinite(param.dDryRunSpeedMmPerMin) || param.dDryRunSpeedMmPerMin <= 0.0)
    {
        param.dDryRunSpeedMmPerMin = DEFAULT_DRY_RUN_SPEED_MM_PER_MIN;
    }
    if (!std::isfinite(param.dWeldSafeMoveSpeedMmPerMin) || param.dWeldSafeMoveSpeedMmPerMin <= 0.0)
    {
        param.dWeldSafeMoveSpeedMmPerMin = DEFAULT_WELD_SAFE_MOVE_SPEED_MM_PER_MIN;
    }
    if (!std::isfinite(param.dStepOverlapRel))
    {
        param.dStepOverlapRel = 20.0;
    }
    param.dStepOverlapRel = std::max(0.0, param.dStepOverlapRel);
    param.nWeldDirection = param.nWeldDirection < 0 ? -1 : 1;
    if (!std::isfinite(param.dGunDownBackSafeDis) || param.dGunDownBackSafeDis <= 0.0)
    {
        param.dGunDownBackSafeDis = WELD_SAFE_OFFSET_DISTANCE_MM;
    }
    if (!std::isfinite(param.dWeldRzGainDeg))
    {
        param.dWeldRzGainDeg = 0.0;
    }
    NormalizeSlopeRzClamp(param.dSlopeRzMinDeg, param.dSlopeRzMaxDeg);
    if (!std::isfinite(param.dScanSafeOffsetDistanceMm) || param.dScanSafeOffsetDistanceMm <= 0.0)
    {
        param.dScanSafeOffsetDistanceMm = 150.0;
    }
    if (!std::isfinite(param.dScanSafeGunAngleDeg))
    {
        param.dScanSafeGunAngleDeg = 30.0;
    }
    if (param.nScanSafeXDirection == 0)
    {
        param.nScanSafeXDirection = -1;
    }
    if (!std::isfinite(param.dScanSafeLiftHeightMm) || param.dScanSafeLiftHeightMm < 0.0)
    {
        param.dScanSafeLiftHeightMm = 150.0;
    }
    if (!std::isfinite(param.dScanSafeFlipWarnThresholdDeg) || param.dScanSafeFlipWarnThresholdDeg <= 0.0)
    {
        param.dScanSafeFlipWarnThresholdDeg = 90.0;
    }

    QString pulseError;
    param.bHasStartPulse = ReadPulse(ini, "StartPulse", param.tStartPulse, pulseError);
    if (!param.bHasStartPulse)
    {
        param.tStartPulse = T_ANGLE_PULSE();
        error.clear();
    }

    if (!ReadCoors(ini, "StartPos", param.tStartPos, error)
        || !ReadCoors(ini, "EndPos", param.tEndPos, error))
    {
        return false;
    }

    if (!param.bUseComputedScanSafe)
    {
        if (!ReadPulseList(ini, "StartSafePulseNum", "StartSafePulse", param.vtStartSafePulse, error)
            || !ReadPulseList(ini, "EndSafePulseNum", "EndSafePulse", param.vtEndSafePulse, error))
        {
            return false;
        }
        if (param.vtStartSafePulse.empty() || param.vtEndSafePulse.empty())
        {
            param.bUseComputedScanSafe = true;
        }
    }

    return true;
}

bool MeasureThenWeldService::ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    int bRtn = 1;
    bRtn = (bRtn && ini.ReadString(prefix + ".nS", &pulse.nSPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nL", &pulse.nLPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nU", &pulse.nUPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nR", &pulse.nRPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nB", &pulse.nBPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nT", &pulse.nTPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".lBX", &pulse.lBXPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".lBY", &pulse.lBYPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".lBZ", &pulse.lBZPulse) > 0) ? 1 : 0;
    if (bRtn == 0)
    {
        error = QString("读取脉冲失败：%1").arg(QString::fromStdString(prefix));
        return false;
    }
    return true;
}

bool MeasureThenWeldService::ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    coors = T_ROBOT_COORS();
    QStringList missingKeys;

    auto readRequired = [&ini, &prefix, &missingKeys](const char* suffix, double& value)
        {
            const std::string key = prefix + "." + suffix;
            if (ini.ReadString(false, key, &value) <= 0)
            {
                missingKeys << QString::fromStdString(key);
            }
        };
    auto readOptional = [&ini, &prefix](const char* suffix, double& value)
        {
            const std::string key = prefix + "." + suffix;
            ini.ReadString(false, key, &value);
        };

    readRequired("X", coors.dX);
    readRequired("Y", coors.dY);
    readRequired("Z", coors.dZ);
    readRequired("RX", coors.dRX);
    readRequired("RY", coors.dRY);
    readRequired("RZ", coors.dRZ);
    readOptional("BX", coors.dBX);
    readOptional("BY", coors.dBY);
    readOptional("BZ", coors.dBZ);

    if (!missingKeys.isEmpty())
    {
        error = QString("读取直角坐标失败：%1，缺少 %2，请在“测量焊接参数”里重新示教并保存扫描起点/终点。")
            .arg(QString::fromStdString(prefix), missingKeys.join(", "));
        return false;
    }
    return true;
}

bool MeasureThenWeldService::ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const
{
    int count = 0;
    ini.ReadString(false, countKey, &count);
    if (count <= 0)
    {
        pulses.clear();
        return true;
    }

    pulses.clear();
    for (int index = 0; index < count; ++index)
    {
        T_ANGLE_PULSE pulse;
        if (!ReadPulse(ini, prefix + std::to_string(index), pulse, error))
        {
            return false;
        }
        pulses.push_back(pulse);
    }
    return true;
}

bool MeasureThenWeldService::MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }
    if (setFlowStep)
    {
        setFlowStep(QString("正在运动：%1").arg(name));
    }
    if (appendLog)
    {
        appendLog(QString("开始运动：%1").arg(name));
    }

    const bool moveOk = pRobotDriver->MoveByJob(pulse, T_ROBOT_MOVE_SPEED(speed, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVJ");
    if (!moveOk)
    {
        if (appendLog)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            appendLog(detail.isEmpty()
                ? QString("运动失败：%1").arg(name)
                : QString("运动失败：%1，%2").arg(name, detail));
        }
        return false;
    }

    return WaitRobotMotionDone(pRobotDriver, name, appendLog);
}

bool MeasureThenWeldService::MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    for (size_t index = 0; index < pulses.size(); ++index)
    {
        if (!MovePulseAndWait(pRobotDriver, pulses[index], speed, QString("%1[%2]").arg(name).arg(index), appendLog, setFlowStep))
        {
            return false;
        }
    }
    return true;
}

bool MeasureThenWeldService::MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }
    const double commandSpeed = LinearCommandSpeedForRobot(pRobotDriver, speed, 1.0);
    const QString commandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    if (setFlowStep)
    {
        setFlowStep(QString("正在直线运动：%1").arg(name));
    }
    if (appendLog)
    {
        appendLog(QString("开始直线运动：%1，配置速度= %2 mm/min，下发速度= %3 %4")
            .arg(name)
            .arg(speed, 0, 'f', 3)
            .arg(commandSpeed, 0, 'f', 3)
            .arg(commandSpeedUnit));
    }

    const bool moveOk = pRobotDriver->MoveByJob(coors, T_ROBOT_MOVE_SPEED(commandSpeed, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVL");
    if (!moveOk)
    {
        if (appendLog)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            appendLog(detail.isEmpty()
                ? QString("直线运动失败：%1").arg(name)
                : QString("直线运动失败：%1，%2").arg(name, detail));
        }
        return false;
    }

    return WaitRobotMotionDone(pRobotDriver, name, appendLog);
}

bool MeasureThenWeldService::MoveScanStartSafeAndWait(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    double speed,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    const CheckpointCallback& checkpoint) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    if (!param.bUseComputedScanSafe && !param.vtStartSafePulse.empty())
    {
        return MovePulseListAndWait(pRobotDriver, param.vtStartSafePulse, speed, "下枪安全姿态", appendLog, setFlowStep);
    }
    if (!param.bUseComputedScanSafe && appendLog)
    {
        appendLog("下枪安全姿态未配置有效脉冲点，自动改用扫描安全位置推算。");
    }

    const T_ROBOT_COORS startSafeCoors = BuildScanSafeCoorsFromAnchor(param.tStartPos, param);
    if (appendLog)
    {
        appendLog(QString("扫描下枪安全位置按配置推算：起点=%1，安全位=%2，偏移=%3mm，枪角=%4deg，X方向=%5")
            .arg(RobotCoorsText(param.tStartPos))
            .arg(RobotCoorsText(startSafeCoors))
            .arg(param.dScanSafeOffsetDistanceMm, 0, 'f', 3)
            .arg(param.dScanSafeGunAngleDeg, 0, 'f', 3)
            .arg(param.nScanSafeXDirection >= 0 ? "X+" : "X-"));
    }

    if (param.bHasStartPulse)
    {
        const T_ANGLE_PULSE currentPulse = pRobotDriver->GetCurrentPulse();
        const double maxWristDeltaDeg = MaxWristDeltaDeg(currentPulse, param.tStartPulse, pRobotDriver->m_tAxisUnit);
        const double warnThresholdDeg = param.dScanSafeFlipWarnThresholdDeg > 0.0
            ? param.dScanSafeFlipWarnThresholdDeg
            : 90.0;
        if (maxWristDeltaDeg >= warnThresholdDeg)
        {
            const QString detail = QString(
                "当前关节姿态和扫描起点示教关节姿态差异较大，最大腕部轴差≈%1°，阈值=%2°。\n"
                "为降低姿态翻转时碰撞风险，流程将先原地抬高 %3mm，再在高位切换到扫描起点姿态，最后移动到扫描下枪安全位置。\n"
                "扫描起点关节：R=%4 B=%5 T=%6\n"
                "当前关节：R=%7 B=%8 T=%9")
                .arg(maxWristDeltaDeg, 0, 'f', 3)
                .arg(warnThresholdDeg, 0, 'f', 3)
                .arg(param.dScanSafeLiftHeightMm, 0, 'f', 3)
                .arg(param.tStartPulse.nRPulse)
                .arg(param.tStartPulse.nBPulse)
                .arg(param.tStartPulse.nTPulse)
                .arg(currentPulse.nRPulse)
                .arg(currentPulse.nBPulse)
                .arg(currentPulse.nTPulse);

            if (checkpoint && !checkpoint("扫描姿态翻转风险提醒", detail))
            {
                if (appendLog)
                {
                    appendLog("用户取消扫描姿态翻转保护流程。");
                }
                return false;
            }

            T_ROBOT_COORS currentCoors = pRobotDriver->GetCurrentPos();
            const double liftHeight = std::max(0.0, param.dScanSafeLiftHeightMm);
            if (liftHeight > 1e-6)
            {
                T_ROBOT_COORS liftCoors = currentCoors;
                liftCoors.dZ += liftHeight;
                if (!MoveCoorsAndWait(pRobotDriver, liftCoors, speed, "扫描姿态切换抬高点", appendLog, setFlowStep))
                {
                    return false;
                }
                currentCoors = liftCoors;
            }

            T_ROBOT_COORS highPoseCoors = currentCoors;
            highPoseCoors.dRX = param.tStartPos.dRX;
            highPoseCoors.dRY = param.tStartPos.dRY;
            highPoseCoors.dRZ = param.tStartPos.dRZ;
            if (!MoveCoorsAndWait(pRobotDriver, highPoseCoors, speed, "高位切换扫描起点姿态", appendLog, setFlowStep))
            {
                return false;
            }
        }
    }
    else if (appendLog)
    {
        appendLog("扫描起点关节脉冲未配置，跳过姿态翻转风险判断，仅按直角位姿推算安全位。");
    }

    return MoveCoorsAndWait(pRobotDriver, startSafeCoors, speed, "扫描下枪安全位置", appendLog, setFlowStep);
}

bool MeasureThenWeldService::MoveScanEndSafeAndWait(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    double speed,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    if (!param.bUseComputedScanSafe && !param.vtEndSafePulse.empty())
    {
        return MovePulseListAndWait(pRobotDriver, param.vtEndSafePulse, speed, "收枪姿态", appendLog, setFlowStep);
    }
    if (!param.bUseComputedScanSafe && appendLog)
    {
        appendLog("收枪姿态未配置有效脉冲点，自动改用扫描安全位置推算。");
    }

    const T_ROBOT_COORS endSafeCoors = BuildScanSafeCoorsFromAnchor(param.tEndPos, param);
    if (appendLog)
    {
        appendLog(QString("扫描收枪安全位置按配置推算：终点=%1，安全位=%2")
            .arg(RobotCoorsText(param.tEndPos))
            .arg(RobotCoorsText(endSafeCoors)));
    }
    return MoveCoorsAndWait(pRobotDriver, endSafeCoors, speed, "扫描收枪安全位置", appendLog, setFlowStep);
}

bool MeasureThenWeldService::ScanMoveAndCollect(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath, const LogCallback& appendLog, const StepCallback& setFlowStep, CameraFrameCache* cameraCache) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }
    if (cameraCache == nullptr)
    {
        appendLog("扫描失败：当前机器人没有可用的专属相机缓存。");
        setFlowStep("扫描失败：相机缓存未初始化");
        return false;
    }
    CameraFrameCache* frameCache = cameraCache;
    FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver);
    const double scanCommandSpeed = LinearCommandSpeedForRobot(pRobotDriver, param.dScanSpeed, 1.0);
    const QString scanCommandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    const double configuredCameraReadFps = (std::isfinite(param.dCameraReadFps) && param.dCameraReadFps > 0.0)
        ? param.dCameraReadFps
        : DEFAULT_CAMERA_READ_FPS;
    const qint64 cameraReadIntervalMs = std::max<qint64>(
        1,
        static_cast<qint64>(std::llround(1000.0 / configuredCameraReadFps)));
    const double actualCameraReadFps = 1000.0 / static_cast<double>(cameraReadIntervalMs);
    const qint64 cameraTimeOffsetUs = static_cast<qint64>(std::llround(param.dCameraTimeOffsetMs * 1000.0));
    if (setFlowStep)
    {
        setFlowStep("扫描运动中，正在采集相机点、机器人位置和激光点");
    }

    HandEyeMatrixConfig calibration = GetDefaultHandEyeMatrixConfig();
    QString calibrationError;
    QString calibrationPath;
    const QString cameraSection = RobotDataHelper::MeasureCameraSection(QString::fromStdString(param.sRobotName));
    if (LoadHandEyeMatrixConfig(QString::fromStdString(param.sRobotName), cameraSection, calibration, &calibrationError, &calibrationPath))
    {
        if (appendLog)
        {
            appendLog(QString("已读取手眼矩阵参数：%1 [%2]").arg(calibrationPath, cameraSection));
        }
    }
    else if (appendLog)
    {
        appendLog(QString("读取手眼矩阵参数失败，已回退默认值：%1 [%2]").arg(calibrationError, cameraSection));
    }

    frameCache->Clear();

    std::vector<RobotCalculation::TimestampedRobotPose> robotSamples;
    robotSamples.reserve(1000);
    std::mutex robotSamplesMutex;
    std::condition_variable robotSamplesCv;

    std::deque<QueuedScanCameraFrame> pendingCameraFrames;
    std::mutex pendingCameraFramesMutex;
    std::condition_variable pendingCameraFramesCv;
    std::atomic_bool cameraEnqueueFinished(false);

    std::vector<ProcessedScanCameraSample> processedCameraSamples;
    std::mutex processedCameraSamplesMutex;

    std::vector<TimestampedCameraPoint> cameraSamples;
    std::vector<TimestampedCameraPoint> matchedCameraSamples;
    QVector<RobotCalculation::IndexedPoint3D> laserFitInput;
    cameraSamples.reserve(10000);
    matchedCameraSamples.reserve(10000);
    laserFitInput.reserve(10000);
    long long lastRobotMonitorMs = std::numeric_limits<long long>::min();
    bool passiveRobotSamplingActive = false;
    int invalidCameraTimestampCount = 0;
    int cameraBeforeRobotTimeBaseCount = 0;
    int cameraTimestampBackwardsCount = 0;
    int cameraTimestampJumpCount = 0;
    int enqueuedCameraSampleCount = 0;
    qint64 lastCameraRawTimestampUs = 0;
    qint64 maxCameraRawDeltaUs = 0;
    const qint64 cameraTimestampJumpWarnUs = std::max<qint64>(50000, cameraReadIntervalMs * 4000);
    bool hasCameraToRobotTimeOffset = false;
    qint64 cameraToRobotTimeOffsetUs = 0;
    qint64 firstCameraRawTimestampUs = 0;
    qint64 firstRobotTimestampUs = 0;
    bool hasCameraTimeBaseRobotTimestamp = false;
    qint64 cameraTimeBaseRobotTimestampUs = 0;
    qint64 latestEnqueuedCameraTimestampUs = 0;
    std::uint64_t scanStartCameraSequence = 0;
    std::uint64_t scanEndCameraSequence = 0;
    std::uint64_t lastPulledCameraSequence = 0;

    auto appendCameraFrame = [
        &pendingCameraFrames,
        &pendingCameraFramesMutex,
        &pendingCameraFramesCv,
        cameraTimeOffsetUs,
        &invalidCameraTimestampCount,
        &cameraBeforeRobotTimeBaseCount,
        &cameraTimestampBackwardsCount,
        &cameraTimestampJumpCount,
        &lastCameraRawTimestampUs,
        &maxCameraRawDeltaUs,
        cameraTimestampJumpWarnUs,
        &hasCameraToRobotTimeOffset,
        &cameraToRobotTimeOffsetUs,
        &firstCameraRawTimestampUs,
        &firstRobotTimestampUs,
        &hasCameraTimeBaseRobotTimestamp,
        &cameraTimeBaseRobotTimestampUs,
        &latestEnqueuedCameraTimestampUs,
        &enqueuedCameraSampleCount](const udpDataShow& frame)
        {
            const qint64 rawTimestampUs = static_cast<qint64>(frame.timestamp);
            if (rawTimestampUs <= 0)
            {
                ++invalidCameraTimestampCount;
                return;
            }
            qint64 rawDeltaUs = 0;
            if (lastCameraRawTimestampUs > 0)
            {
                rawDeltaUs = rawTimestampUs - lastCameraRawTimestampUs;
                if (rawDeltaUs <= 0)
                {
                    ++cameraTimestampBackwardsCount;
                }
                else if (rawDeltaUs > cameraTimestampJumpWarnUs)
                {
                    ++cameraTimestampJumpCount;
                    maxCameraRawDeltaUs = std::max(maxCameraRawDeltaUs, rawDeltaUs);
                }
            }
            lastCameraRawTimestampUs = rawTimestampUs;
            if (!hasCameraToRobotTimeOffset)
            {
                if (!hasCameraTimeBaseRobotTimestamp)
                {
                    ++cameraBeforeRobotTimeBaseCount;
                    return;
                }

                firstCameraRawTimestampUs = rawTimestampUs;
                firstRobotTimestampUs = cameraTimeBaseRobotTimestampUs;
                cameraToRobotTimeOffsetUs = firstRobotTimestampUs - firstCameraRawTimestampUs;
                hasCameraToRobotTimeOffset = true;
            }

            {
                std::lock_guard<std::mutex> locker(pendingCameraFramesMutex);
                QueuedScanCameraFrame queuedFrame;
                queuedFrame.sampleIndex = ++enqueuedCameraSampleCount;
                queuedFrame.rawTimestampUs = rawTimestampUs;
                queuedFrame.rawDeltaUs = rawDeltaUs;
                queuedFrame.timestampUs = rawTimestampUs + cameraToRobotTimeOffsetUs + cameraTimeOffsetUs;
                queuedFrame.frame = frame;
                latestEnqueuedCameraTimestampUs = queuedFrame.timestampUs;
                pendingCameraFrames.push_back(std::move(queuedFrame));
            }
            pendingCameraFramesCv.notify_one();
        };

    auto appendRobotPose = [
        &robotSamples,
        &robotSamplesMutex,
        &robotSamplesCv,
        pRobotDriver,
        &lastRobotMonitorMs,
        &passiveRobotSamplingActive]()
        {
            RobotCalculation::TimestampedRobotPose sample;
            RobotDriverAdaptor::StateSnapshot snapshot;
            if (pRobotDriver->LatestStateSnapshot(snapshot) && snapshot.valid)
            {
                if (passiveRobotSamplingActive && snapshot.robotMs == lastRobotMonitorMs)
                {
                    return false;
                }

                sample.pose = snapshot.pose;
                sample.timestampUs = static_cast<qint64>(snapshot.robotMs) * 1000;
                lastRobotMonitorMs = snapshot.robotMs;
                passiveRobotSamplingActive = true;
            }
            if (sample.timestampUs <= 0)
            {
                sample.pose = pRobotDriver->GetCurrentPos();
                sample.timestampUs = SteadyNowUs();
            }

            {
                std::lock_guard<std::mutex> locker(robotSamplesMutex);
                robotSamples.push_back(sample);
            }
            robotSamplesCv.notify_all();
            return true;
        };

    auto latestRobotTimestampUs = [&robotSamples, &robotSamplesMutex]()
        {
            std::lock_guard<std::mutex> locker(robotSamplesMutex);
            return robotSamples.empty() ? 0 : robotSamples.back().timestampUs;
        };

    auto pullScanCameraFramesTo = [frameCache, &lastPulledCameraSequence, &appendCameraFrame](std::uint64_t targetSequence)
        {
            if (targetSequence <= lastPulledCameraSequence)
            {
                return;
            }

            const std::vector<udpDataShow> frames = frameCache->FramesBetween(
                lastPulledCameraSequence,
                targetSequence);
            for (const udpDataShow& frame : frames)
            {
                appendCameraFrame(frame);
            }
            lastPulledCameraSequence = targetSequence;
        };

    auto pullScanCameraFrames = [frameCache, &pullScanCameraFramesTo]()
        {
            pullScanCameraFramesTo(frameCache->Mark());
        };

    const unsigned int hardwareThreads = std::thread::hardware_concurrency();
    const int processingWorkerCount = std::max(
        1,
        std::min(4, static_cast<int>(hardwareThreads > 1 ? hardwareThreads - 1 : 1)));
    const qint64 processingWallStartMs = SteadyNowMs();
    std::vector<std::thread> processingWorkers;
    processingWorkers.reserve(static_cast<std::size_t>(processingWorkerCount));
    for (int workerIndex = 0; workerIndex < processingWorkerCount; ++workerIndex)
    {
        processingWorkers.emplace_back([&]()
            {
                while (true)
                {
                    QueuedScanCameraFrame queuedFrame;
                    {
                        std::unique_lock<std::mutex> locker(pendingCameraFramesMutex);
                        pendingCameraFramesCv.wait(locker, [&]()
                            {
                                return !pendingCameraFrames.empty() || cameraEnqueueFinished.load();
                            });
                        if (pendingCameraFrames.empty())
                        {
                            if (cameraEnqueueFinished.load())
                            {
                                break;
                            }
                            continue;
                        }

                        queuedFrame = std::move(pendingCameraFrames.front());
                        pendingCameraFrames.pop_front();
                    }

                    ProcessedScanCameraSample processed;
                    processed.sample.sampleIndex = queuedFrame.sampleIndex;
                    processed.sample.rawTimestampUs = queuedFrame.rawTimestampUs;
                    processed.sample.rawDeltaUs = queuedFrame.rawDeltaUs;
                    processed.sample.timestampUs = queuedFrame.timestampUs;
                    processed.sample.point = Eigen::Vector3d(
                        queuedFrame.frame.targetPoint.x,
                        queuedFrame.frame.targetPoint.y,
                        queuedFrame.frame.targetPoint.z);
                    processed.sample.error = queuedFrame.frame.errorMessage;

                    std::vector<RobotCalculation::TimestampedRobotPose> interpolationSamples;
                    {
                        std::unique_lock<std::mutex> locker(robotSamplesMutex);
                        while (true)
                        {
                            if (robotSamples.empty())
                            {
                                if (cameraEnqueueFinished.load())
                                {
                                    processed.status = "unmatched_no_robot_sample";
                                    break;
                                }
                            }
                            else if (queuedFrame.timestampUs < robotSamples.front().timestampUs)
                            {
                                processed.status = "unmatched_before_robot";
                                break;
                            }
                            else if (queuedFrame.timestampUs <= robotSamples.back().timestampUs)
                            {
                                processed.robotWindow = FindRobotInterpolationWindow(robotSamples, queuedFrame.timestampUs);
                                if (processed.robotWindow.prevIndex > 0)
                                {
                                    const int prevZeroIndex = std::clamp(
                                        processed.robotWindow.prevIndex - 1,
                                        0,
                                        static_cast<int>(robotSamples.size()) - 1);
                                    const int nextZeroIndex = std::clamp(
                                        processed.robotWindow.nextIndex - 1,
                                        0,
                                        static_cast<int>(robotSamples.size()) - 1);
                                    interpolationSamples.push_back(robotSamples[static_cast<std::size_t>(prevZeroIndex)]);
                                    if (nextZeroIndex != prevZeroIndex)
                                    {
                                        interpolationSamples.push_back(robotSamples[static_cast<std::size_t>(nextZeroIndex)]);
                                    }
                                }
                                break;
                            }
                            else if (cameraEnqueueFinished.load())
                            {
                                processed.status = "unmatched_after_robot";
                                break;
                            }

                            robotSamplesCv.wait_for(locker, std::chrono::milliseconds(CAMERA_ROBOT_MATCH_TAIL_POLL_MS));
                        }
                    }

                    if (!interpolationSamples.empty())
                    {
                        processed.robotPose = RobotCalculation::InterpolateRobotPose(interpolationSamples, queuedFrame.timestampUs);
                        processed.hasRobotPose = true;
                        processed.contributedWorkpieceFrame = !queuedFrame.frame.allResultPoint.empty();
                        processed.workpiecePoints.reserve(queuedFrame.frame.allResultPoint.size());
                        for (int linePointIndex = 0; linePointIndex < static_cast<int>(queuedFrame.frame.allResultPoint.size()); ++linePointIndex)
                        {
                            const cv::Point3d& sourcePoint = queuedFrame.frame.allResultPoint[static_cast<std::size_t>(linePointIndex)];
                            // allResultPoint 的 Z 轴符号与 targetPoint 相反；生成工件点云前先统一到 targetPoint 使用的相机坐标约定。
                            const Eigen::Vector3d cameraLinePoint(sourcePoint.x, sourcePoint.y, -sourcePoint.z);
                            constexpr double kZeroPointEps = 1e-9;
                            const bool isZeroPoint =
                                std::abs(cameraLinePoint.x()) <= kZeroPointEps
                                && std::abs(cameraLinePoint.y()) <= kZeroPointEps
                                && std::abs(cameraLinePoint.z()) <= kZeroPointEps;
                            if (!IsFiniteCameraPoint(cameraLinePoint) || isZeroPoint)
                            {
                                ++processed.skippedWorkpieceCloudPointCount;
                                continue;
                            }

                            ProcessedScanWorkpiecePoint cloudPoint;
                            cloudPoint.workpiecePoint =
                                RobotCalculation::CalcLaserPointInRobot(processed.robotPose, cameraLinePoint, calibration);
                            processed.workpiecePoints.push_back(cloudPoint);
                        }

                        if (ShouldSkipLaserCalc(processed.sample))
                        {
                            processed.status = "skip_invalid_camera_point";
                        }
                        else
                        {
                            processed.status = "laser_ok";
                            processed.laserPoint =
                                RobotCalculation::CalcLaserPointInRobot(processed.robotPose, processed.sample.point, calibration);
                            processed.hasLaserPoint = true;
                        }
                    }

                    {
                        std::lock_guard<std::mutex> locker(processedCameraSamplesMutex);
                        processedCameraSamples.push_back(std::move(processed));
                    }
                }
            });
    }
    auto finishCameraProcessingWorkers = [&]()
        {
            cameraEnqueueFinished.store(true);
            pendingCameraFramesCv.notify_all();
            robotSamplesCv.notify_all();
            for (std::thread& worker : processingWorkers)
            {
                if (worker.joinable())
                {
                    worker.join();
                }
            }
        };

    if (appendLog)
    {
        appendLog(QString("开始扫描运动：相机帧由当前机器人专属缓存读取，配置相机读取帧率=%1 fps（约 %2 ms/帧，用于时间间隔统计），机器人位姿约 %3 ms 采样；机器人位姿使用被动时间轴（FANUC=机器人端robot_ms，STEP/其他=PC steady ms），相机帧timestamp会在首帧处映射到该时间轴，并叠加相机时间补偿 %4 ms。点云转换使用 %5 个后台处理线程。配置扫描速度= %6 mm/min，下发速度= %7 %8")
            .arg(actualCameraReadFps, 0, 'f', 2)
            .arg(cameraReadIntervalMs)
            .arg(ROBOT_SAMPLE_INTERVAL_MS)
            .arg(param.dCameraTimeOffsetMs, 0, 'f', 3)
            .arg(processingWorkerCount)
            .arg(param.dScanSpeed, 0, 'f', 3)
            .arg(scanCommandSpeed, 0, 'f', 3)
            .arg(scanCommandSpeedUnit));
    }

    const bool moveOk = pRobotDriver->MoveByJob(
        param.tEndPos,
        T_ROBOT_MOVE_SPEED(scanCommandSpeed, 0.0, 0.0),
        pRobotDriver->m_nExternalAxleType,
        "MOVL");
    if (!moveOk)
    {
        if (appendLog)
        {
            appendLog("扫描终点运动启动失败。");
        }
        finishCameraProcessingWorkers();
        return false;
    }

    appendRobotPose();
    int motionState = 0;
    bool motionStarted = false;
    const qint64 motionStartMs = SteadyNowMs();
    qint64 lastRobotPollMs = motionStartMs - ROBOT_SAMPLE_INTERVAL_MS;
    while (true)
    {
        const qint64 nowMs = SteadyNowMs();

        if ((nowMs - lastRobotPollMs) >= ROBOT_SAMPLE_INTERVAL_MS)
        {
            lastRobotPollMs = nowMs;
            motionState = pFanucDriver != nullptr
                ? pFanucDriver->GetIntVar(FANUC_MOTION_STATE_REG)
                : pRobotDriver->CheckDone();
            const bool isRunningState = pFanucDriver != nullptr
                ? (motionState == 10 || motionState == 20 || motionState == 1)
                : (motionState == 0);
            const bool isDoneState = pFanucDriver != nullptr
                ? (motionState == 1)
                : (motionState == 2 || motionState == 1);
            if (isRunningState)
            {
                if (!motionStarted)
                {
                    scanStartCameraSequence = frameCache->Mark();
					lastPulledCameraSequence = scanStartCameraSequence;
					if (appendLog)
					{
						if (pFanucDriver != nullptr)
						{
							appendLog(QString("扫描运动状态寄存器进入运行态：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
						}
						else
						{
							appendLog(QString("扫描运动进入运行态：CheckDone=%1").arg(motionState));
						}
					}
				}
                motionStarted = true;
            }
            if (motionStarted)
            {
                appendRobotPose();
                if (!hasCameraTimeBaseRobotTimestamp)
                {
                    const qint64 latestRobotUs = latestRobotTimestampUs();
                    if (latestRobotUs > 0)
                    {
                        cameraTimeBaseRobotTimestampUs = latestRobotUs;
                        hasCameraTimeBaseRobotTimestamp = true;
                    }
                }
                pullScanCameraFrames();
            }
            if (motionStarted && isDoneState)
            {
                scanEndCameraSequence = frameCache->Mark();
                pullScanCameraFramesTo(scanEndCameraSequence);
                break;
            }

            const qint64 elapsedMs = SteadyNowMs() - motionStartMs;
			if (!motionStarted && elapsedMs > 3000)
			{
				if (appendLog)
				{
					if (pFanucDriver != nullptr)
					{
						appendLog(QString("扫描运动未在 3s 内进入运行态：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
					}
					else
					{
						appendLog(QString("扫描运动未在 3s 内进入运行态：CheckDone=%1").arg(motionState));
					}
				}
				finishCameraProcessingWorkers();
				return false;
            }
			if (motionStarted && elapsedMs > 120000)
			{
				if (appendLog)
				{
					if (pFanucDriver != nullptr)
					{
						appendLog(QString("扫描运动等待完成超时：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
					}
					else
					{
						appendLog(QString("扫描运动等待完成超时：CheckDone=%1").arg(motionState));
					}
				}
                finishCameraProcessingWorkers();
                return false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    appendRobotPose();
    const qint64 scanMotionElapsedMs = SteadyNowMs() - motionStartMs;

    if (scanEndCameraSequence == 0)
    {
        scanEndCameraSequence = frameCache->Mark();
        pullScanCameraFramesTo(scanEndCameraSequence);
    }
    frameCache->Clear();

    bool tailWaitTriggered = false;
    const qint64 tailWaitStartMs = SteadyNowMs();
    while (latestEnqueuedCameraTimestampUs > 0
        && latestRobotTimestampUs() > 0
        && latestEnqueuedCameraTimestampUs > latestRobotTimestampUs()
        && (SteadyNowMs() - tailWaitStartMs) < CAMERA_ROBOT_MATCH_TAIL_WAIT_MS)
    {
        if (!tailWaitTriggered && appendLog)
        {
            appendLog(QString("检测到相机时间戳晚于最新机器人位姿，开始等待机器人监控时间追上（最长 %1 ms）。")
                .arg(CAMERA_ROBOT_MATCH_TAIL_WAIT_MS));
        }
        tailWaitTriggered = true;

        appendRobotPose();
        if (latestEnqueuedCameraTimestampUs <= latestRobotTimestampUs())
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_ROBOT_MATCH_TAIL_POLL_MS));
    }

    const qint64 tailWaitElapsedMs = tailWaitTriggered ? (SteadyNowMs() - tailWaitStartMs) : 0;
    const qint64 processingJoinStartMs = SteadyNowMs();
    finishCameraProcessingWorkers();
    const qint64 postMotionProcessingWaitMs = SteadyNowMs() - processingJoinStartMs;
    const qint64 parallelProcessingElapsedMs = SteadyNowMs() - processingWallStartMs;

    {
        std::lock_guard<std::mutex> locker(processedCameraSamplesMutex);
        std::sort(processedCameraSamples.begin(), processedCameraSamples.end(),
            [](const ProcessedScanCameraSample& left, const ProcessedScanCameraSample& right)
            {
                return left.sample.sampleIndex < right.sample.sampleIndex;
            });
        cameraSamples.reserve(processedCameraSamples.size());
        matchedCameraSamples.reserve(processedCameraSamples.size());
        for (const ProcessedScanCameraSample& processed : processedCameraSamples)
        {
            cameraSamples.push_back(processed.sample);
            if (processed.hasRobotPose)
            {
                matchedCameraSamples.push_back(processed.sample);
            }
        }
    }

    int droppedHeadCameraCount = 0;
    int droppedTailCameraCount = 0;
    for (const ProcessedScanCameraSample& processed : processedCameraSamples)
    {
        if (processed.status == "unmatched_before_robot")
        {
            ++droppedHeadCameraCount;
        }
        else if (processed.status == "unmatched_after_robot")
        {
            ++droppedTailCameraCount;
        }
    }

    const QString resultDir = BuildResultDir(param.sRobotName);
    const QString cameraDir = QDir(resultDir).filePath("CameraPoint");
    const QString robotDir = QDir(resultDir).filePath("RobotPoint");
    const QString laserDir = QDir(resultDir).filePath("LaserPoint");
    QDir().mkpath(cameraDir);
    QDir().mkpath(robotDir);
    QDir().mkpath(laserDir);

    const QString cameraPath = QDir(cameraDir).filePath("PreciseCameraPoint.txt");
    const QString robotPath = QDir(robotDir).filePath("PreciseRobotPoint.txt");
    const QString laserPath = QDir(laserDir).filePath(RAW_LASER_FILE_NAME);
    const QString workpieceCloudPath = QDir(laserDir).filePath(WORKPIECE_CLOUD_FILE_NAME);
    const QString matchDebugPath = QDir(laserDir).filePath(MATCH_DEBUG_FILE_NAME);
    const QString preservePathFitPath = QDir(laserDir).filePath(PRESERVE_PATH_FILE_NAME);
    const QString keyPointsPath = QDir(laserDir).filePath(KEY_POINTS_FILE_NAME);
    const QString classifiedPath = QDir(laserDir).filePath(CLASSIFIED_FILE_NAME);
    const QString classifiedNoisePath = QDir(laserDir).filePath(CLASSIFIED_NOISE_FILE_NAME);
    const QString weldPosePath = QDir(laserDir).filePath(WELD_POSE_FILE_NAME);
    const QString weldPoseSeamCompPath = QDir(laserDir).filePath(WELD_POSE_SEAM_COMP_FILE_NAME);
    savedPath = resultDir;

    const qint64 outputBuildStartMs = SteadyNowMs();
    std::vector<QString> cameraLines;
    std::vector<QString> robotLines;
    std::vector<QString> laserLines;
    std::vector<QString> workpieceCloudLines;
    std::vector<QString> matchDebugLines;
    cameraLines.reserve(cameraSamples.size() + 1);
    robotLines.reserve(matchedCameraSamples.size() + 1);
    laserLines.reserve(matchedCameraSamples.size() + 1);
    workpieceCloudLines.reserve(cameraSamples.size() * 16 + 1);
    matchDebugLines.reserve(cameraSamples.size() + 1);
    cameraLines.push_back("index,x,y,z,error");
    robotLines.push_back("index,x,y,z,rx,ry,rz,bx,by,bz");
    laserLines.push_back("index,x,y,z");
    workpieceCloudLines.push_back("index x y z");
    matchDebugLines.push_back("index,status,camera_raw_timestamp_us,camera_raw_delta_us,mapped_robot_timestamp_us,prev_robot_index,prev_robot_timestamp_us,next_robot_index,next_robot_timestamp_us,interp_ratio,camera_x,camera_y,camera_z,robot_x,robot_y,robot_z,robot_rx,robot_ry,robot_rz,robot_bx,robot_by,robot_bz,laser_x,laser_y,laser_z,error");

    const qint64 cameraLineBuildStartMs = SteadyNowMs();
    for (const TimestampedCameraPoint& sample : cameraSamples)
    {
        cameraLines.push_back(RobotCalculation::Vector3IndexedCsv(sample.sampleIndex, sample.point, sample.error));
    }
    const qint64 cameraLineBuildElapsedMs = SteadyNowMs() - cameraLineBuildStartMs;

    int skippedLaserCount = 0;
    int unmatchedBeforeRobotCount = 0;
    int unmatchedAfterRobotCount = 0;
    int unmatchedUnknownCount = 0;
    int laserIndexGapCount = 0;
    int maxLaserIndexGap = 0;
    int lastLaserIndex = -1;
    int workpieceCloudFrameCount = 0;
    int workpieceCloudPointCount = 0;
    int skippedWorkpieceCloudPointCount = 0;
    int workpieceCloudPointIndex = 1;
    const qint64 pointComputeStartMs = SteadyNowMs();
    for (const ProcessedScanCameraSample& processed : processedCameraSamples)
    {
        const TimestampedCameraPoint& sample = processed.sample;
        const int index = sample.sampleIndex;
        const QString& status = processed.status;
        if (status == "unmatched_before_robot")
        {
            ++unmatchedBeforeRobotCount;
        }
        else if (status == "unmatched_after_robot")
        {
            ++unmatchedAfterRobotCount;
        }
        else if (status == "skip_invalid_camera_point")
        {
            ++skippedLaserCount;
        }
        else if (status != "laser_ok")
        {
            ++unmatchedUnknownCount;
        }

        if (processed.contributedWorkpieceFrame)
        {
            ++workpieceCloudFrameCount;
        }
        skippedWorkpieceCloudPointCount += processed.skippedWorkpieceCloudPointCount;
        if (processed.hasRobotPose)
        {
            for (const ProcessedScanWorkpiecePoint& cloudPoint : processed.workpiecePoints)
            {
                QStringList cloudFields;
                cloudFields
                    << QString::number(workpieceCloudPointIndex++)
                    << QString::number(cloudPoint.workpiecePoint.x(), 'f', 6)
                    << QString::number(cloudPoint.workpiecePoint.y(), 'f', 6)
                    << QString::number(cloudPoint.workpiecePoint.z(), 'f', 6);
                workpieceCloudLines.push_back(cloudFields.join(' '));
                ++workpieceCloudPointCount;
            }
        }

        if (processed.hasLaserPoint)
        {
            if (lastLaserIndex > 0 && index - lastLaserIndex > 1)
            {
                ++laserIndexGapCount;
                maxLaserIndexGap = std::max(maxLaserIndexGap, index - lastLaserIndex - 1);
            }
            lastLaserIndex = index;

            robotLines.push_back(RobotCalculation::RobotPoseIndexedCsv(index, processed.robotPose));
            laserLines.push_back(RobotCalculation::Vector3IndexedCsv(index, processed.laserPoint));

            RobotCalculation::IndexedPoint3D laserFitPoint;
            laserFitPoint.index = index;
            laserFitPoint.point = processed.laserPoint;
            laserFitInput.push_back(laserFitPoint);
        }

        QStringList fields;
        fields
            << QString::number(index)
            << status
            << QString::number(sample.rawTimestampUs)
            << QString::number(sample.rawDeltaUs)
            << QString::number(sample.timestampUs)
            << QString::number(processed.robotWindow.prevIndex)
            << QString::number(processed.robotWindow.prevTimestampUs)
            << QString::number(processed.robotWindow.nextIndex)
            << QString::number(processed.robotWindow.nextTimestampUs)
            << QString::number(processed.robotWindow.ratio, 'f', 6)
            << QString::number(sample.point.x(), 'f', 6)
            << QString::number(sample.point.y(), 'f', 6)
            << QString::number(sample.point.z(), 'f', 6);
        if (processed.hasRobotPose)
        {
            fields
                << QString::number(processed.robotPose.dX, 'f', 6)
                << QString::number(processed.robotPose.dY, 'f', 6)
                << QString::number(processed.robotPose.dZ, 'f', 6)
                << QString::number(processed.robotPose.dRX, 'f', 6)
                << QString::number(processed.robotPose.dRY, 'f', 6)
                << QString::number(processed.robotPose.dRZ, 'f', 6)
                << QString::number(processed.robotPose.dBX, 'f', 6)
                << QString::number(processed.robotPose.dBY, 'f', 6)
                << QString::number(processed.robotPose.dBZ, 'f', 6);
        }
        else
        {
            fields << "" << "" << "" << "" << "" << "" << "" << "" << "";
        }
        if (processed.hasLaserPoint)
        {
            fields
                << QString::number(processed.laserPoint.x(), 'f', 6)
                << QString::number(processed.laserPoint.y(), 'f', 6)
                << QString::number(processed.laserPoint.z(), 'f', 6);
        }
        else
        {
            fields << "" << "" << "";
        }
        fields << CsvEscape(sample.error);
        matchDebugLines.push_back(fields.join(','));
    }
    const qint64 pointComputeElapsedMs = SteadyNowMs() - pointComputeStartMs;
    const qint64 outputBuildElapsedMs = SteadyNowMs() - outputBuildStartMs;

    QString error;
    auto saveTextLinesTimed = [this, &error](const QString& filePath, const std::vector<QString>& lines, qint64& elapsedMs)
        {
            const qint64 saveStartMs = SteadyNowMs();
            const bool ok = SaveTextLines(filePath, lines, error);
            elapsedMs = SteadyNowMs() - saveStartMs;
            return ok;
        };
    qint64 saveCameraElapsedMs = 0;
    qint64 saveRobotElapsedMs = 0;
    qint64 saveLaserElapsedMs = 0;
    qint64 saveWorkpieceCloudElapsedMs = 0;
    qint64 saveMatchDebugElapsedMs = 0;
    if (!saveTextLinesTimed(cameraPath, cameraLines, saveCameraElapsedMs)
        || !saveTextLinesTimed(robotPath, robotLines, saveRobotElapsedMs)
        || !saveTextLinesTimed(laserPath, laserLines, saveLaserElapsedMs)
        || !saveTextLinesTimed(workpieceCloudPath, workpieceCloudLines, saveWorkpieceCloudElapsedMs)
        || !saveTextLinesTimed(matchDebugPath, matchDebugLines, saveMatchDebugElapsedMs))
    {
        if (appendLog)
        {
            appendLog(error);
        }
        return false;
    }
    const qint64 saveAllElapsedMs =
        saveCameraElapsedMs
        + saveRobotElapsedMs
        + saveLaserElapsedMs
        + saveWorkpieceCloudElapsedMs
        + saveMatchDebugElapsedMs;

    if (appendLog)
    {
        const std::uint64_t cacheFrameSpan =
            scanEndCameraSequence >= scanStartCameraSequence
            ? scanEndCameraSequence - scanStartCameraSequence
            : 0;
        const double cacheEffectiveFps =
            scanMotionElapsedMs > 0
            ? static_cast<double>(cacheFrameSpan) * 1000.0 / static_cast<double>(scanMotionElapsedMs)
            : 0.0;
        appendLog(QString("扫描完成，相机点=%1，机器人采样=%2，已匹配相机点=%3，保存目录=%4")
            .arg(static_cast<int>(cameraSamples.size()))
            .arg(static_cast<int>(robotSamples.size()))
            .arg(static_cast<int>(matchedCameraSamples.size()))
            .arg(resultDir));
        appendLog(QString("扫描期间已处理相机帧=%1，缓存序号范围=(%2, %3]，缓存新增=%4，估算缓存FPS=%5")
            .arg(static_cast<int>(cameraSamples.size() + invalidCameraTimestampCount + cameraBeforeRobotTimeBaseCount))
            .arg(scanStartCameraSequence)
            .arg(scanEndCameraSequence)
            .arg(static_cast<qulonglong>(cacheFrameSpan))
            .arg(cacheEffectiveFps, 0, 'f', 2));
        if (hasCameraToRobotTimeOffset)
        {
            appendLog(QString("相机时间轴映射到机器人采样时间轴：首帧相机timestamp=%1 us，对齐机器人时间=%2 us，映射偏移=%3 us，额外补偿=%4 ms。")
                .arg(firstCameraRawTimestampUs)
                .arg(firstRobotTimestampUs)
                .arg(cameraToRobotTimeOffsetUs)
                .arg(param.dCameraTimeOffsetMs, 0, 'f', 3));
        }
        if (droppedHeadCameraCount > 0)
        {
            appendLog(QString("有 %1 个相机点早于首个机器人时间戳，已跳过未参与插值。").arg(droppedHeadCameraCount));
        }
        if (cameraBeforeRobotTimeBaseCount > 0)
        {
            appendLog(QString("有 %1 个相机点早于机器人时间基准建立，已跳过未参与插值。").arg(cameraBeforeRobotTimeBaseCount));
        }
        if (invalidCameraTimestampCount > 0)
        {
            appendLog(QString("有 %1 个相机点timestamp无效，已跳过未参与插值。").arg(invalidCameraTimestampCount));
        }
        appendLog(QString("相机原始timestamp间隔统计：倒退次数=%1，大跳次数=%2，大跳阈值=%3 us，最大间隔=%4 us。")
            .arg(cameraTimestampBackwardsCount)
            .arg(cameraTimestampJumpCount)
            .arg(cameraTimestampJumpWarnUs)
            .arg(maxCameraRawDeltaUs));
        if (droppedTailCameraCount > 0)
        {
            appendLog(QString("有 %1 个相机点晚于最后一个机器人时间戳，等待 %2 ms 后仍未匹配到机器人位姿，已跳过未参与插值。")
                .arg(droppedTailCameraCount)
                .arg(CAMERA_ROBOT_MATCH_TAIL_WAIT_MS));
        }
        appendLog(QString("激光计算有效点=%1，跳过异常相机点=%2")
            .arg(static_cast<int>(laserLines.size()) - 1)
            .arg(skippedLaserCount));
        appendLog(QString("完整工件点云：参与帧=%1，点数=%2，跳过异常线点=%3")
            .arg(workpieceCloudFrameCount)
            .arg(workpieceCloudPointCount)
            .arg(skippedWorkpieceCloudPointCount));
        appendLog(QString("扫描耗时统计：运动等待=%1 ms，尾部匹配等待=%2 ms，并行点云处理墙钟=%3 ms（运动后等待=%4 ms），输出构建=%5 ms（相机文本=%6 ms，明细组装=%7 ms），文件写入=%8 ms（相机=%9 ms，机器人=%10 ms，激光=%11 ms，完整点云=%12 ms，匹配明细=%13 ms）。")
            .arg(scanMotionElapsedMs)
            .arg(tailWaitElapsedMs)
            .arg(parallelProcessingElapsedMs)
            .arg(postMotionProcessingWaitMs)
            .arg(outputBuildElapsedMs)
            .arg(cameraLineBuildElapsedMs)
            .arg(pointComputeElapsedMs)
            .arg(saveAllElapsedMs)
            .arg(saveCameraElapsedMs)
            .arg(saveRobotElapsedMs)
            .arg(saveLaserElapsedMs)
            .arg(saveWorkpieceCloudElapsedMs)
            .arg(saveMatchDebugElapsedMs));
        appendLog(QString("激光点序号断点统计：断点段数=%1，最大连续缺失帧数=%2，匹配前丢弃=%3，匹配后丢弃=%4，未知未匹配=%5")
            .arg(laserIndexGapCount)
            .arg(maxLaserIndexGap)
            .arg(unmatchedBeforeRobotCount)
            .arg(unmatchedAfterRobotCount)
            .arg(unmatchedUnknownCount));
        appendLog(QString("相机点文件：%1").arg(cameraPath));
        appendLog(QString("机器人插值位姿文件：%1").arg(robotPath));
        appendLog(QString("激光点文件：%1").arg(laserPath));
        appendLog(QString("完整工件点云文件：%1").arg(workpieceCloudPath));
        appendLog(QString("相机-机器人-激光匹配明细文件：%1").arg(matchDebugPath));
    }

    if (laserFitInput.size() < 2)
    {
        if (appendLog)
        {
            appendLog(QString("激光有效点过少（%1），跳过 PreservePath 拟合、焊道分类和焊接姿态生成。").arg(laserFitInput.size()));
        }
        return true;
    }

    const RobotCalculation::LowerWeldFilterParams originalFitParams = BuildOriginalTrackFitParams(param);
    if (setFlowStep)
    {
        setFlowStep("扫描完成，正在进行先测后焊特征分析");
    }
    if (appendLog)
    {
        appendLog(QString("开始先测后焊特征分析：采样主轴=%1，重采样步长=%2 mm，拐点拟合容差=%3 mm，每段最少点数=%4")
            .arg(SampleAxisName(originalFitParams.sampleAxis))
            .arg(originalFitParams.sampleStep, 0, 'f', 3)
            .arg(originalFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(originalFitParams.piecewiseMinSegmentPoints));
    }

    const RobotCalculation::MeasureThenWeldAnalysisResult originalAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(laserFitInput, originalFitParams);
    if (!originalAnalysis.ok)
    {
        if (appendLog)
        {
            appendLog(QString("先测后焊特征分析失败：%1").arg(originalAnalysis.error));
            appendLog("已保留原始激光点文件，可先按原始点云继续分析。");
        }
        return true;
    }

    if (!SaveTextLines(preservePathFitPath, BuildFilterOutputLines(originalAnalysis.filterResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存先测后焊特征提取结果失败：%1").arg(error));
        }
        return true;
    }

    if (appendLog)
    {
        appendLog(QString("先测后焊特征提取完成：输入=%1，输出=%2，文件=%3")
            .arg(originalAnalysis.filterResult.inputPointCount)
            .arg(originalAnalysis.filterResult.points.size())
            .arg(preservePathFitPath));
        appendLog(FilterResultSummary("先测后焊特征提取", originalFitParams, originalAnalysis.filterResult, preservePathFitPath));
    }

    if (setFlowStep)
    {
        setFlowStep("先测后焊特征提取完成，正在进行焊道分类");
    }
    if (!SaveTextLines(classifiedPath, BuildClassifiedOutputLines(originalAnalysis.classificationResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存焊道分类结果失败：%1").arg(error));
        }
        return true;
    }

    if (!SaveTextLines(keyPointsPath, BuildKeyPointOutputLines(originalAnalysis.keyPoints), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存起终点/拐点结果失败：%1").arg(error));
        }
        return true;
    }

    if (!SaveTextLines(classifiedNoisePath, BuildNoiseOutputLines(laserFitInput, originalAnalysis.filterResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存焊道杂点结果失败：%1").arg(error));
        }
        return true;
    }

    if (appendLog)
    {
        int startCount = 0;
        int endCount = 0;
        int innerCount = 0;
        int outerCount = 0;
        int normalCount = 0;
        for (const RobotCalculation::LowerWeldClassifiedPoint& point : originalAnalysis.classificationResult.points)
        {
            switch (point.type)
            {
            case RobotCalculation::LowerWeldPointType::Start:
                ++startCount;
                break;
            case RobotCalculation::LowerWeldPointType::End:
                ++endCount;
                break;
            case RobotCalculation::LowerWeldPointType::InnerCorner:
                ++innerCount;
                break;
            case RobotCalculation::LowerWeldPointType::OuterCorner:
                ++outerCount;
                break;
            case RobotCalculation::LowerWeldPointType::Normal:
                ++normalCount;
                break;
            default:
                break;
            }
        }

        appendLog(QString("焊道分类完成：起点=%1，终点=%2，内拐点=%3，外拐点=%4，普通点=%5，分类文件=%6")
            .arg(startCount)
            .arg(endCount)
            .arg(innerCount)
            .arg(outerCount)
            .arg(normalCount)
            .arg(classifiedPath));
        appendLog(QString("起终点/拐点文件：%1").arg(keyPointsPath));
        appendLog(QString("焊道杂点文件：%1").arg(classifiedNoisePath));
        appendLog(QString("先测后焊特征分析摘要：输入=%1，下层候选=%2，输出=%3，剔除Z突变=%4，剔除Z连续异常=%5，连续段剔除=%6")
            .arg(originalAnalysis.filterResult.inputPointCount)
            .arg(originalAnalysis.filterResult.lowerPointCount)
            .arg(originalAnalysis.filterResult.points.size())
            .arg(originalAnalysis.filterResult.zJumpRejectedCount)
            .arg(originalAnalysis.filterResult.zContinuityRejectedCount)
            .arg(originalAnalysis.filterResult.segmentRejectedCount));
    }

    const MeasurementPoseReference measurementPoseReference =
        FirstMeasurementPoseReferenceFromProcessedSamples(processedCameraSamples, matchDebugPath);
    const WeldPosePreset weldPosePreset = ApplyMeasurementPoseReferenceForCalculation(
        LoadWeldPosePreset(param),
        measurementPoseReference,
        appendLog);
    if (appendLog)
    {
        appendLog(QString("焊接姿态参数：RX=%1, RY=%2, RZ增益=%3 deg, 爬坡RZ夹紧=[%4, %5] deg, 拐点前过渡=%6 mm, 起点跳过=%7 mm, 终点跳过=%8 mm, 姿态补偿槽=%9, 焊道补偿槽=%10, 基础参数来源=%11, 姿态补偿来源=%12, 焊道补偿来源=%13")
            .arg(weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.weldRzGainDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMinDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMaxDeg, 0, 'f', 3)
            .arg(weldPosePreset.cornerTransitionLeadDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldStartSkipDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldEndSkipDistance, 0, 'f', 3)
            .arg(static_cast<int>(weldPosePreset.poseCompSlots.size()))
            .arg(static_cast<int>(weldPosePreset.seamCompSlots.size()))
            .arg(weldPosePreset.weldLineFromIni
                ? QString("%1 [%2]").arg(weldPosePreset.weldLineFilePath, weldPosePreset.weldLineSectionName)
                : QString("扫描起点姿态回退"))
            .arg(weldPosePreset.poseCompFromIni ? weldPosePreset.poseCompFilePath : QString("默认值"))
            .arg(weldPosePreset.seamCompFromIni ? weldPosePreset.seamCompFilePath : QString("默认值")));
    }

    if (setFlowStep)
    {
        setFlowStep("焊道分类完成，正在生成分段焊接姿态");
    }

    const std::vector<QString> weldPoseLines =
        BuildSegmentPoseOutputLines(originalAnalysis.classificationResult, param, weldPosePreset, appendLog);
    if (!weldPoseLines.empty())
    {
        if (!SaveTextLines(weldPosePath, weldPoseLines, error))
        {
            if (appendLog)
            {
                appendLog(QString("保存焊接姿态结果失败：%1").arg(error));
            }
            return true;
        }

        if (appendLog)
        {
            appendLog(QString("焊接姿态文件：%1").arg(weldPosePath));
        }

        QString seamCompSummary;
        if (!ApplyWeldSeamCompToPoseFile(
            QString::fromStdString(param.sRobotName),
            weldPosePath,
            weldPoseSeamCompPath,
            seamCompSummary,
            error))
        {
            if (appendLog)
            {
                appendLog(QString("保存焊道补偿后文件失败：%1").arg(error));
                appendLog(QString("已保留姿态文件，可在修正补偿参数后重新生成：%1").arg(weldPosePath));
            }
            return false;
        }

        if (appendLog)
        {
            appendLog(QString("焊道补偿文件：%1").arg(weldPoseSeamCompPath));
            appendLog(QString("焊道补偿摘要：%1").arg(seamCompSummary));
        }
        savedPath = weldPoseSeamCompPath;
    }
    else if (appendLog)
    {
        appendLog("焊接姿态生成结果为空，请检查起终点跳过距离或焊道分类结果。");
    }
    return true;
}

bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(
    const T_PRECISE_MEASURE_PARAM& param,
    const QString& laserDir,
    QString& preservePath,
    QString& weldPosePath,
    QString& seamCompPath,
    QString& summary,
    QString& error,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep) const
{
    preservePath.clear();
    weldPosePath.clear();
    seamCompPath.clear();
    summary.clear();
    error.clear();

    const QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("LaserPoint目录不存在：%1").arg(laserDir);
        return false;
    }

    preservePath = dir.filePath(PRESERVE_PATH_FILE_NAME);
    const QString keyPointsPath = dir.filePath(KEY_POINTS_FILE_NAME);
    const QString classifiedPath = dir.filePath(CLASSIFIED_FILE_NAME);
    const QString classifiedNoisePath = dir.filePath(CLASSIFIED_NOISE_FILE_NAME);
    const QString matchDebugPath = dir.filePath(MATCH_DEBUG_FILE_NAME);
    weldPosePath = dir.filePath(WELD_POSE_FILE_NAME);
    seamCompPath = dir.filePath(WELD_POSE_SEAM_COMP_FILE_NAME);

    QString sourceLaserPath = dir.filePath(RAW_LASER_FILE_NAME);
    if (!QFileInfo::exists(sourceLaserPath))
    {
        if (QFileInfo::exists(preservePath))
        {
            sourceLaserPath = preservePath;
            if (appendLog)
            {
                appendLog(QString("未找到原始激光点文件 %1，临时使用已有 PreservePath 文件作为重建输入。").arg(RAW_LASER_FILE_NAME));
            }
        }
        else
        {
            error = QString("未找到原始激光点文件：%1").arg(dir.filePath(RAW_LASER_FILE_NAME));
            return false;
        }
    }

    QVector<RobotCalculation::IndexedPoint3D> laserFitInput;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(sourceLaserPath, laserFitInput, &error))
    {
        return false;
    }
    if (laserFitInput.size() < 2)
    {
        error = QString("激光有效点过少（%1），无法重建焊接文件。").arg(laserFitInput.size());
        return false;
    }

    const RobotCalculation::LowerWeldFilterParams originalFitParams = BuildOriginalTrackFitParams(param);
    if (setFlowStep)
    {
        setFlowStep("正在重新计算 PreservePath、焊接姿态和焊道补偿文件");
    }
    if (appendLog)
    {
        appendLog(QString("跳过扫描重建输入：%1，点数=%2").arg(sourceLaserPath).arg(laserFitInput.size()));
        appendLog(QString("开始先测后焊特征分析：采样主轴=%1，重采样步长=%2 mm，拐点拟合容差=%3 mm，每段最少点数=%4")
            .arg(SampleAxisName(originalFitParams.sampleAxis))
            .arg(originalFitParams.sampleStep, 0, 'f', 3)
            .arg(originalFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(originalFitParams.piecewiseMinSegmentPoints));
    }

    const RobotCalculation::MeasureThenWeldAnalysisResult originalAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(laserFitInput, originalFitParams);
    if (!originalAnalysis.ok)
    {
        error = QString("先测后焊特征分析失败：%1").arg(originalAnalysis.error);
        return false;
    }

    if (!SaveTextLines(preservePath, BuildFilterOutputLines(originalAnalysis.filterResult), error))
    {
        return false;
    }
    if (!SaveTextLines(classifiedPath, BuildClassifiedOutputLines(originalAnalysis.classificationResult), error))
    {
        return false;
    }
    if (!SaveTextLines(keyPointsPath, BuildKeyPointOutputLines(originalAnalysis.keyPoints), error))
    {
        return false;
    }
    if (!SaveTextLines(classifiedNoisePath, BuildNoiseOutputLines(laserFitInput, originalAnalysis.filterResult), error))
    {
        return false;
    }

    QString measurementPoseError;
    const MeasurementPoseReference measurementPoseReference =
        LoadMeasurementPoseReferenceFromMatchDebug(matchDebugPath, &measurementPoseError);
    if (!measurementPoseReference.valid && appendLog)
    {
        appendLog(QString("读取点云测量姿态失败：%1").arg(measurementPoseError));
    }
    const WeldPosePreset weldPosePreset = ApplyMeasurementPoseReferenceForCalculation(
        LoadWeldPosePreset(param),
        measurementPoseReference,
        appendLog);
    if (appendLog)
    {
        appendLog(QString("先测后焊特征提取完成：输入=%1，输出=%2，文件=%3")
            .arg(originalAnalysis.filterResult.inputPointCount)
            .arg(originalAnalysis.filterResult.points.size())
            .arg(preservePath));
        appendLog(QString("焊道分类文件：%1").arg(classifiedPath));
        appendLog(QString("起终点/拐点文件：%1").arg(keyPointsPath));
        appendLog(QString("焊道杂点文件：%1").arg(classifiedNoisePath));
        appendLog(QString("焊接姿态参数：RX=%1, RY=%2, RZ增益=%3 deg, 爬坡RZ夹紧=[%4, %5] deg, 拐点前过渡=%6 mm, 起点跳过=%7 mm, 终点跳过=%8 mm")
            .arg(weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.weldRzGainDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMinDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMaxDeg, 0, 'f', 3)
            .arg(weldPosePreset.cornerTransitionLeadDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldStartSkipDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldEndSkipDistance, 0, 'f', 3));
    }

    if (setFlowStep)
    {
        setFlowStep("特征分析完成，正在生成焊接姿态");
    }
    const std::vector<QString> weldPoseLines =
        BuildSegmentPoseOutputLines(originalAnalysis.classificationResult, param, weldPosePreset, appendLog);
    if (weldPoseLines.empty())
    {
        error = "焊接姿态生成结果为空，请检查起终点跳过距离或焊道分类结果。";
        return false;
    }
    if (!SaveTextLines(weldPosePath, weldPoseLines, error))
    {
        return false;
    }

    if (setFlowStep)
    {
        setFlowStep("焊接姿态已生成，正在生成焊道补偿文件");
    }
    QString seamCompSummary;
    if (!ApplyWeldSeamCompToPoseFile(
        QString::fromStdString(param.sRobotName),
        weldPosePath,
        seamCompPath,
        seamCompSummary,
        error))
    {
        return false;
    }

    summary = QString("重建完成：PreservePath=%1；WeldPose=%2；SeamComp=%3；%4")
        .arg(preservePath, weldPosePath, seamCompPath, seamCompSummary);
    if (appendLog)
    {
        appendLog(QString("PreservePath文件：%1").arg(preservePath));
        appendLog(QString("焊接姿态文件：%1").arg(weldPosePath));
        appendLog(QString("焊道补偿文件：%1").arg(seamCompPath));
        appendLog(QString("焊道补偿摘要：%1").arg(seamCompSummary));
    }
    return true;
}

QString MeasureThenWeldService::BuildResultDir(const std::string& robotName) const
{
    const QString dateText = QDateTime::currentDateTime().toString("yyyyMMdd");
    const QString baseDir = RobotDataHelper::BuildProjectPath(QString("Result/%1").arg(QString::fromStdString(robotName)));
    QDir dir(baseDir);
    if (!dir.exists())
    {
        dir.mkpath(".");
    }

    int flowNo = 1;
    const QStringList entries = dir.entryList(QStringList() << (dateText + "_*"), QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QString& entry : entries)
    {
        const QString suffix = entry.mid(dateText.length() + 1);
        bool ok = false;
        const int value = suffix.toInt(&ok);
        if (ok && value >= flowNo)
        {
            flowNo = value + 1;
        }
    }

    const QString flowDir = dir.filePath(QString("%1_%2").arg(dateText).arg(flowNo, 3, 10, QChar('0')));
    QDir().mkpath(flowDir);
    return QDir::toNativeSeparators(flowDir);
}

bool MeasureThenWeldService::SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const
{
    const QFileInfo fileInfo(filePath);
    const QDir parentDir = fileInfo.dir();
    if (!parentDir.exists() && !QDir().mkpath(parentDir.absolutePath()))
    {
        error = QString("创建保存目录失败：%1").arg(parentDir.absolutePath());
        return false;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        error = QString("保存数据文件失败：%1").arg(filePath);
        return false;
    }

    QTextStream stream(&file);
    for (const QString& line : lines)
    {
        stream << line << "\n";
    }
    return true;
}

bool MeasureThenWeldService::ApplyWeldSeamCompToPoseFile(
    const QString& robotName,
    const QString& inputPath,
    const QString& outputPath,
    QString& summary,
    QString& error) const
{
    summary.clear();
    error.clear();

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(inputPath, records, error))
    {
        return false;
    }

    T_PRECISE_MEASURE_PARAM param = BuildMeasureWeldParamShell(robotName);
    const WeldPosePreset preset = LoadWeldPosePreset(param);

    WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
    WeldEndpointTrimStats endpointTrimStats;
    TrimWeldPoseRecordEndpoints(preset, records, endpointTrimStats);
    if (records.isEmpty())
    {
        error = "焊道补偿和起终点裁剪后没有有效焊接点。";
        return false;
    }
    TrimWeldPathSelfIntersections(records, compStats);
    if (records.isEmpty())
    {
        error = "焊道自交裁剪后没有有效焊接点。";
        return false;
    }
    const double densifyStepMm = std::min(2.0, EstimateWeldPoseStepMm(records));
    const int densifiedPointCount = DensifyWeldPoseRecordsByStep(records, densifyStepMm);
    const WeldCornerArcApplyStats arcStats = ApplyCornerArcTransitionToWeldPoseRecords(preset, records);
    TrimSharpWeldArcEntryPoints(
        records,
        8.0 * 3.14159265358979323846 / 180.0,
        std::max(2.0, densifyStepMm * 2.5));
    RenumberWeldPoseRecords(records);

    QStringList outputLines;
    outputLines.reserve(records.size() + 1);
    outputLines << "weld_index raw_index x y z rx ry rz bx by bz point_type segment_kind";
    for (const WeldPoseFileRecord& record : records)
    {
        outputLines << BuildWeldPoseFileRecordLine(record);
    }

    if (!RobotDataHelper::SaveTextFileLines(outputPath, outputLines, &error))
    {
        return false;
    }

    const QString segmentKindDebugPath = QFileInfo(outputPath)
        .dir()
        .filePath(WELD_SEGMENT_KIND_DEBUG_FILE_NAME);
    if (!SaveTextLines(segmentKindDebugPath, BuildWeldSegmentKindDebugLines(records), error))
    {
        return false;
    }

    QStringList usedSlots = compStats.usedSlots.values();
    usedSlots.sort();
    summary = QString("焊道补偿完成：点数=%1，使用槽位=%2，Z补偿点数=%3，枪反向补偿点数=%4，焊道方向补偿点数=%5，起点裁剪=%6点，终点裁剪=%7点，自交裁剪=%8次，删除回折点=%9，补齐点=%10，最大步长=%11mm，圆弧过渡=%12处，半径=%13mm，新增点=%14，四类属性=%15，配置=%16")
        .arg(records.size())
        .arg(usedSlots.isEmpty() ? QString("无匹配槽位") : usedSlots.join(","))
        .arg(compStats.zAdjustedCount)
        .arg(compStats.gunDirAdjustedCount)
        .arg(compStats.seamDirAdjustedCount)
        .arg(endpointTrimStats.removedStartCount)
        .arg(endpointTrimStats.removedEndCount)
        .arg(compStats.selfIntersectionTrimCount)
        .arg(compStats.selfIntersectionRemovedPointCount)
        .arg(densifiedPointCount)
        .arg(densifyStepMm, 0, 'f', 3)
        .arg(arcStats.roundedCornerCount)
        .arg(arcStats.radiusMm, 0, 'f', 3)
        .arg(arcStats.insertedPointCount())
        .arg(QDir::toNativeSeparators(segmentKindDebugPath))
        .arg(QDir::toNativeSeparators(preset.seamCompFilePath));
    return true;
}

bool MeasureThenWeldService::GenerateStepWeldProgramFiles(
    const QString& robotName,
    const QString& poseFilePath,
    const QString& outputDir,
    bool actualWeld,
    double weldSpeedMmPerMin,
    QString& programName,
    QString& srpPath,
    QString& srdPath,
    QString& summary,
    QString& error) const
{
    programName.clear();
    srpPath.clear();
    srdPath.clear();
    summary.clear();
    error.clear();

    QFileInfo poseInfo(QDir::fromNativeSeparators(poseFilePath.trimmed()));
    if (!poseInfo.isAbsolute())
    {
        poseInfo = QFileInfo(QDir::current().filePath(poseInfo.filePath()));
    }
    if (!poseInfo.exists() || !poseInfo.isFile())
    {
        error = QString("焊接姿态文件不存在：%1")
            .arg(QDir::toNativeSeparators(poseInfo.absoluteFilePath()));
        return false;
    }

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(poseInfo.absoluteFilePath(), records, error))
    {
        return false;
    }

    T_PRECISE_MEASURE_PARAM param = BuildMeasureWeldParamShell(robotName);
    const WeldPosePreset preset = LoadWeldPosePreset(param);
    ApplyWeldDirectionToExecutionRecords(preset, records);
    const double effectiveWeldSpeedMmPerMin =
        (std::isfinite(weldSpeedMmPerMin) && weldSpeedMmPerMin > 0.0)
        ? weldSpeedMmPerMin
        : ((std::isfinite(preset.weldProcessSpeedMmPerMin) && preset.weldProcessSpeedMmPerMin > 0.0)
            ? preset.weldProcessSpeedMmPerMin
            : FANUC_WELD_PATH_SPEED_MM_PER_MIN);
    const double transitionCommandSpeed =
        (preset.transitionSpeedEnabled && preset.transitionSpeedMmPerMin > 0.0)
        ? preset.transitionSpeedMmPerMin
        : 0.0;

    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    if (!BuildWeldPoseMoveInfos(
        records,
        effectiveWeldSpeedMmPerMin,
        moveInfos,
        error,
        &preset,
        transitionCommandSpeed,
        true))
    {
        return false;
    }

    QString resolvedOutputDir = outputDir.trimmed();
    if (resolvedOutputDir.isEmpty())
    {
        resolvedOutputDir = RobotDataHelper::BuildProjectPath("Job/STEP");
    }
    else
    {
        QFileInfo outputInfo(QDir::fromNativeSeparators(resolvedOutputDir));
        resolvedOutputDir = outputInfo.isAbsolute()
            ? outputInfo.absoluteFilePath()
            : QFileInfo(QDir::current().filePath(outputInfo.filePath())).absoluteFilePath();
    }

    const std::string generatedProgramName = STEPRobotCtrl::MakeTimestampWeldProgramName();
    T_AXISUNIT axisUnit;
    std::string localSrpPath;
    std::string localSrdPath;
    std::string writeError;
    if (!STEPRobotCtrl::WriteContiMoveAnyFiles(
        moveInfos,
        QDir::toNativeSeparators(resolvedOutputDir).toStdString(),
        generatedProgramName,
        axisUnit,
        &localSrpPath,
        &localSrdPath,
        &writeError,
        actualWeld))
    {
        error = QString("生成STEP焊接程序失败：%1").arg(QString::fromStdString(writeError));
        return false;
    }

    programName = QString::fromStdString(generatedProgramName);
    srpPath = QDir::toNativeSeparators(QString::fromStdString(localSrpPath));
    srdPath = QDir::toNativeSeparators(QString::fromStdString(localSrdPath));
    summary = QString("STEP焊接程序生成完成：程序=%1，模式=%2，方向=%3，点数=%4，轨迹速度=%5 mm/min，OVERLAPREL=%6，SRP=%7，SRD=%8")
        .arg(programName)
        .arg(actualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑"))
        .arg(WeldDirectionText(preset))
        .arg(static_cast<int>(moveInfos.size()))
        .arg(effectiveWeldSpeedMmPerMin, 0, 'f', 3)
        .arg(preset.stepOverlapRel, 0, 'f', 3)
        .arg(srpPath)
        .arg(srdPath);
    if (!preset.weldProcessLoaded)
    {
        summary += "；未读取到当前焊接工艺参数，本次文件不包含起弧/停弧工艺语句";
    }
    else
    {
        summary += QString("；工艺=%1A/%2V -> %3A/%4V -> %5A/%6V")
            .arg(preset.startArcCurrent, 0, 'f', 3)
            .arg(preset.startArcVoltage, 0, 'f', 3)
            .arg(preset.weldCurrent, 0, 'f', 3)
            .arg(preset.weldVoltage, 0, 'f', 3)
            .arg(preset.stopArcCurrent, 0, 'f', 3)
            .arg(preset.stopArcVoltage, 0, 'f', 3);
        if (preset.transitionSpeedEnabled)
        {
            summary += QString("；过渡速度=%1 mm/min")
                .arg(preset.transitionSpeedMmPerMin, 0, 'f', 3);
        }
        if (preset.transitionCurrentVoltageEnabled)
        {
            summary += QString("；过渡电流电压=%1A/%2V")
                .arg(preset.transitionCurrent, 0, 'f', 3)
                .arg(preset.transitionVoltage, 0, 'f', 3);
        }
        if (preset.transitionSpeedEnabled || preset.transitionCurrentVoltageEnabled)
        {
            summary += QString("；拐点过渡作用范围=%1")
                .arg(TransitionApplyScopeText(preset.transitionApplyScope));
        }
        if (!actualWeld)
        {
            summary += "；空跑模式不生成ARCON/ARCSET/ARCOFF";
        }
    }
    return true;
}

bool MeasureThenWeldService::DownlinkWeldPoseFile(
    RobotDriverAdaptor* pRobotDriver,
    const QString& poseFilePath,
    double linearSpeedConfigMmPerMin,
    QString& summary,
    QString& error) const
{
    summary.clear();
    error.clear();
    const double selectedSpeedMmPerMin =
        (std::isfinite(linearSpeedConfigMmPerMin) && linearSpeedConfigMmPerMin > 0.0)
        ? linearSpeedConfigMmPerMin
        : FANUC_WELD_PATH_SPEED_MM_PER_MIN;

    if (pRobotDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(poseFilePath, records, error))
    {
        return false;
    }

    const double linearCommandSpeed =
        LinearCommandSpeedForRobot(pRobotDriver, selectedSpeedMmPerMin, 1.0);
    const QString linearCommandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    const T_PRECISE_MEASURE_PARAM param = BuildMeasureWeldParamShell(QString::fromStdString(pRobotDriver->m_sRobotName));
    const WeldPosePreset preset = LoadWeldPosePreset(param);
    ApplyWeldDirectionToExecutionRecords(preset, records);
    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    if (!BuildWeldPoseMoveInfos(records, linearCommandSpeed, moveInfos, error, &preset))
    {
        return false;
    }

    if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
    {
        std::string programName;
        std::string localLsPath;
        std::string remoteTpPath;
        const int downlinkRet = pFanucDriver->UploadMultiPointTpProgram(
            moveInfos,
            &programName,
            &localLsPath,
            &remoteTpPath);
        if (downlinkRet != 0)
        {
            error = QString("下发焊接轨迹失败：ret=%1，姿态文件=%2")
                .arg(downlinkRet)
                .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
            return false;
        }

        summary = QString("点数=%1，轨迹速度=%2 mm/min (下发=%3 %4)，程序=%5，本地LS=%6，远程TP=%7，当前仅下发未自动执行")
            .arg(static_cast<int>(moveInfos.size()))
            .arg(selectedSpeedMmPerMin, 0, 'f', 3)
            .arg(linearCommandSpeed, 0, 'f', 3)
            .arg(linearCommandSpeedUnit)
            .arg(QString::fromStdString(programName))
            .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
            .arg(QString::fromStdString(remoteTpPath));
        return true;
    }

    QString stepProgramNameText = "ContiMoveAny";
    int ret = 0;
    if (STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver))
    {
        const std::string stepProgramName = STEPRobotCtrl::MakeTimestampWeldProgramName();
        stepProgramNameText = QString::fromStdString(stepProgramName);
        ret = pStepDriver->ContiMoveAnyWithProgramName(moveInfos, stepProgramName);
    }
    else
    {
        ret = pRobotDriver->ContiMoveAny(moveInfos);
    }
    if (ret != 0)
    {
        error = QString("STEP焊接轨迹下发/启动失败：ret=%1，姿态文件=%2")
            .arg(ret)
            .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
        return false;
    }
    summary = QString("点数=%1，方向=%2，轨迹速度=%3 mm/min (下发=%4 %5)，STEP使用%6生成、上传并启动程序")
        .arg(static_cast<int>(moveInfos.size()))
        .arg(WeldDirectionText(preset))
        .arg(selectedSpeedMmPerMin, 0, 'f', 3)
        .arg(linearCommandSpeed, 0, 'f', 3)
        .arg(linearCommandSpeedUnit)
        .arg(stepProgramNameText);
    return true;
}

bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(
    RobotDriverAdaptor* pRobotDriver,
    const QString& poseFilePath,
    const T_PRECISE_MEASURE_PARAM& param,
    QString& summary,
    QString& error,
    T_ROBOT_COORS* pStartSafeCoors,
    T_ROBOT_COORS* pEndSafeCoors,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    const CheckpointCallback& checkpoint) const
{
    summary.clear();
    error.clear();

    if (pRobotDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(poseFilePath, records, error))
    {
        return false;
    }

    const WeldPosePreset weldPosePreset = LoadWeldPosePreset(param);
    ApplyWeldDirectionToExecutionRecords(weldPosePreset, records);
    T_ROBOT_COORS startSafeCoors;
    if (!TryBuildWeldSafeCoors(records, 0, param.dGunDownBackSafeDis, weldPosePreset.robotType, startSafeCoors, error))
    {
        return false;
    }

    T_ROBOT_COORS endSafeCoors;
    if (!TryBuildWeldSafeCoors(records, records.size() - 1, param.dGunDownBackSafeDis, weldPosePreset.robotType, endSafeCoors, error))
    {
        return false;
    }

    const bool useWeldProcessTrajectorySpeed = param.bDoActualWeld
        && weldPosePreset.weldProcessLoaded
        && std::isfinite(weldPosePreset.weldProcessSpeedMmPerMin)
        && weldPosePreset.weldProcessSpeedMmPerMin > 0.0;
    const double selectedWeldSpeedMmPerMin = param.bDoActualWeld
        ? (useWeldProcessTrajectorySpeed ? weldPosePreset.weldProcessSpeedMmPerMin : param.dWeldSpeedMmPerMin)
        : param.dDryRunSpeedMmPerMin;
    const double weldSpeedMmPerMin =
        (std::isfinite(selectedWeldSpeedMmPerMin) && selectedWeldSpeedMmPerMin > 0.0)
        ? selectedWeldSpeedMmPerMin
        : (param.bDoActualWeld ? FANUC_WELD_PATH_SPEED_MM_PER_MIN : DEFAULT_DRY_RUN_SPEED_MM_PER_MIN);
    const QString weldSpeedSourceText = param.bDoActualWeld
        ? (useWeldProcessTrajectorySpeed ? QStringLiteral("焊接工艺WeldVelocity") : QStringLiteral("测量焊接参数WeldSpeedMmPerMin"))
        : QStringLiteral("空跑速度DryRunSpeedMmPerMin");
    const double safeMoveSpeedMmPerMin =
        (std::isfinite(param.dWeldSafeMoveSpeedMmPerMin) && param.dWeldSafeMoveSpeedMmPerMin > 0.0)
        ? param.dWeldSafeMoveSpeedMmPerMin
        : DEFAULT_WELD_SAFE_MOVE_SPEED_MM_PER_MIN;
    const QString weldModeText = param.bDoActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑");
    const double weldCommandSpeed =
        LinearCommandSpeedForRobot(pRobotDriver, weldSpeedMmPerMin, 1.0);
    const double transitionCommandSpeed =
        (weldPosePreset.transitionSpeedEnabled && weldPosePreset.transitionSpeedMmPerMin > 0.0)
        ? LinearCommandSpeedForRobot(pRobotDriver, weldPosePreset.transitionSpeedMmPerMin, weldCommandSpeed)
        : 0.0;
    const QString weldCommandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    const double weldEstimateSpeedMmPerSec =
        FanucLinearSpeedMmPerSecFromConfig(weldSpeedMmPerMin, 1.0);
    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    if (!BuildWeldPoseMoveInfos(
        records,
        weldCommandSpeed,
        moveInfos,
        error,
        &weldPosePreset,
        transitionCommandSpeed,
        param.bDoActualWeld))
    {
        return false;
    }

    if (pStartSafeCoors != nullptr)
    {
        *pStartSafeCoors = startSafeCoors;
    }
    if (pEndSafeCoors != nullptr)
    {
        *pEndSafeCoors = endSafeCoors;
    }

    if (appendLog)
    {
        appendLog(QString("焊接安全位：回退距离=%1 mm，横向约束=X-")
            .arg(param.dGunDownBackSafeDis, 0, 'f', 3));
        appendLog(QString("下枪安全位置：%1").arg(RobotCoorsText(startSafeCoors)));
        appendLog(QString("收枪安全位置：%1").arg(RobotCoorsText(endSafeCoors)));
        appendLog(QString("焊接轨迹模式：%1，轨迹速度=%2 mm/min，来源=%3，下发速度=%4 %5")
            .arg(weldModeText)
            .arg(weldSpeedMmPerMin, 0, 'f', 3)
            .arg(weldSpeedSourceText)
            .arg(weldCommandSpeed, 0, 'f', 3)
            .arg(weldCommandSpeedUnit));
        appendLog(QString("焊接方向：%1").arg(WeldDirectionText(weldPosePreset)));
        if (param.bDoActualWeld)
        {
            if (weldPosePreset.weldProcessLoaded)
            {
                appendLog(QString("焊接工艺：起弧=%1A/%2V，焊接=%3A/%4V，停弧=%5A/%6V")
                    .arg(weldPosePreset.startArcCurrent, 0, 'f', 3)
                    .arg(weldPosePreset.startArcVoltage, 0, 'f', 3)
                    .arg(weldPosePreset.weldCurrent, 0, 'f', 3)
                    .arg(weldPosePreset.weldVoltage, 0, 'f', 3)
                    .arg(weldPosePreset.stopArcCurrent, 0, 'f', 3)
                    .arg(weldPosePreset.stopArcVoltage, 0, 'f', 3));
                if (weldPosePreset.cornerArcRadiusFromWeldProcess)
                {
                    appendLog(QString("拐点圆弧半径使用当前工艺参数：%1 mm")
                        .arg(weldPosePreset.cornerArcRadiusMm, 0, 'f', 3));
                }
                if (weldPosePreset.transitionSpeedEnabled)
                {
                    appendLog(QString("拐点过渡速度启用：%1 mm/min (下发=%2 %3)")
                        .arg(weldPosePreset.transitionSpeedMmPerMin, 0, 'f', 3)
                        .arg(transitionCommandSpeed, 0, 'f', 3)
                        .arg(weldCommandSpeedUnit));
                }
                if (weldPosePreset.transitionCurrentVoltageEnabled)
                {
                    appendLog(QString("拐点过渡电流电压启用：%1A/%2V")
                        .arg(weldPosePreset.transitionCurrent, 0, 'f', 3)
                        .arg(weldPosePreset.transitionVoltage, 0, 'f', 3));
                }
                if (weldPosePreset.transitionSpeedEnabled || weldPosePreset.transitionCurrentVoltageEnabled)
                {
                    appendLog(QString("拐点过渡作用范围：%1")
                        .arg(TransitionApplyScopeText(weldPosePreset.transitionApplyScope)));
                }
            }
            else
            {
                appendLog("未读取到当前焊接工艺参数，本次只按轨迹运动执行，不生成起弧/停弧工艺语句。");
            }
        }
    }

    QString downlinkSummary;
    QString programNameText;
    int lastState = 0;
    const double pathLengthMm = EstimateMoveInfosPathLengthMm(moveInfos);
    const double estimatedRunMs = weldEstimateSpeedMmPerSec > 1e-6
        ? (pathLengthMm / weldEstimateSpeedMmPerSec) * 1000.0
        : 0.0;
    const int finishTimeoutMs = static_cast<int>(std::clamp(
        estimatedRunMs * 2.0 + 30000.0,
        120000.0,
        1800000.0));

    if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
    {
        const double safeMoveSpeedMmPerSec =
            FanucLinearSpeedMmPerSecFromConfig(safeMoveSpeedMmPerMin, 1.0);
        if (setFlowStep)
        {
            setFlowStep("正在移动到下枪安全位置，并行下发焊接轨迹");
        }
        if (appendLog)
        {
            appendLog(QString("开始直线运动：下枪安全位置，配置速度= %1 mm/min，下发速度= %2 mm/sec")
                .arg(safeMoveSpeedMmPerMin, 0, 'f', 3)
                .arg(safeMoveSpeedMmPerSec, 0, 'f', 3));
        }

        if (!pFanucDriver->MoveByJob(
            startSafeCoors,
            T_ROBOT_MOVE_SPEED(safeMoveSpeedMmPerSec, 0.0, 0.0),
            pFanucDriver->m_nExternalAxleType,
            "MOVL"))
        {
            error = "启动下枪安全位置直线运动失败。";
            return false;
        }
        if (appendLog)
        {
            appendLog("下枪安全位置运动已启动，开始并行下发焊接轨迹程序。");
        }

        std::string programName;
        std::string localLsPath;
        std::string remoteTpPath;
        const int downlinkRet = pFanucDriver->UploadMultiPointTpProgram(
            moveInfos,
            &programName,
            &localLsPath,
            &remoteTpPath);
        if (downlinkRet != 0)
        {
            error = QString("下发焊接轨迹失败：ret=%1，姿态文件=%2。机器人可能仍在移动到下枪安全位置。")
                .arg(downlinkRet)
                .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
            return false;
        }

        int startSafeLastState = 0;
        const bool startSafeDoneOk = pFanucDriver->WaitStateDone(
            FANUC_MOTION_STATE_REG, 1, 10, 20, 3000, 120000, 100, &startSafeLastState);
        if (appendLog)
        {
            appendLog(QString("直线运动结束：下枪安全位置, R[%1]=%2, WaitStateDone=%3")
                .arg(FANUC_MOTION_STATE_REG)
                .arg(startSafeLastState)
                .arg(startSafeDoneOk ? 1 : 0));
        }
        if (!startSafeDoneOk)
        {
            error = QString("下枪安全位置未完成，R[%1]=%2，取消运行焊接轨迹。")
                .arg(FANUC_MOTION_STATE_REG)
                .arg(startSafeLastState);
            return false;
        }

        if (checkpoint && !checkpoint(
            "焊前确认",
            QString("下枪安全位置已到位，焊接轨迹程序也已下发完成。\n"
                    "运行模式：%1\n"
                    "下枪安全位置：%2\n"
                    "焊接程序：%3\n"
                    "是否开始执行焊道？")
                .arg(weldModeText)
                .arg(RobotCoorsText(startSafeCoors))
                .arg(QString::fromStdString(programName))))
        {
            error = "用户在焊前确认节点取消了流程。";
            return false;
        }

        downlinkSummary =
            QString("点数=%1，模式=%2，轨迹速度=%3 mm/min，来源=%4 (下发=%5 %6)，程序=%7，本地LS=%8，远程TP=%9")
                .arg(static_cast<int>(moveInfos.size()))
                .arg(weldModeText)
                .arg(weldSpeedMmPerMin, 0, 'f', 3)
                .arg(weldSpeedSourceText)
                .arg(weldCommandSpeed, 0, 'f', 3)
                .arg(weldCommandSpeedUnit)
                .arg(QString::fromStdString(programName))
                .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
                .arg(QString::fromStdString(remoteTpPath));
        programNameText = QString::fromStdString(programName);

        if (setFlowStep)
        {
            setFlowStep(QString("正在执行焊接轨迹程序：%1").arg(programNameText));
        }
        if (appendLog)
        {
            appendLog(QString("开始执行焊接轨迹程序：%1，轨迹长度≈%2 mm，预计运行≈%3 s，完成超时=%4 s")
                .arg(programNameText)
                .arg(pathLengthMm, 0, 'f', 3)
                .arg(estimatedRunMs / 1000.0, 0, 'f', 1)
                .arg(finishTimeoutMs / 1000.0));
        }

        if (!pFanucDriver->CallJobAndWaitStateDone(
            programName,
            FANUC_MOTION_STATE_REG,
            1,
            10,
            20,
            5000,
            finishTimeoutMs,
            100,
            &lastState,
            true))
        {
            error = QString("焊接轨迹程序执行失败：%1，R[%2]=%3")
                .arg(programNameText)
                .arg(FANUC_MOTION_STATE_REG)
                .arg(lastState);
            return false;
        }

        if (appendLog)
        {
            appendLog(QString("焊接轨迹程序执行完成：%1，R[%2]=%3")
                .arg(programNameText)
                .arg(FANUC_MOTION_STATE_REG)
                .arg(lastState));
        }
    }
    else
    {
        if (!MoveCoorsAndWait(
            pRobotDriver,
            startSafeCoors,
            safeMoveSpeedMmPerMin,
            "下枪安全位置",
            appendLog,
            setFlowStep))
        {
            error = "移动到下枪安全位置失败。";
            return false;
        }

        if (checkpoint && !checkpoint(
            "焊前确认",
            QString("下枪安全位置已到位，STEP 将生成、上传并启动焊接轨迹。\n"
                    "运行模式：%1\n"
                    "下枪安全位置：%2\n"
                    "点数：%3\n"
                    "是否开始执行焊道？")
                .arg(weldModeText)
                .arg(RobotCoorsText(startSafeCoors))
                .arg(static_cast<int>(moveInfos.size()))))
        {
            error = "用户在焊前确认节点取消了流程。";
            return false;
        }

        std::string stepProgramName = STEPRobotCtrl::MakeTimestampWeldProgramName();
        STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver);
        programNameText = pStepDriver != nullptr
            ? QString::fromStdString(stepProgramName)
            : QStringLiteral("STEP ContiMoveAny");
        downlinkSummary =
            QString("点数=%1，模式=%2，轨迹速度=%3 mm/min，来源=%4 (下发=%5 %6)，STEP使用%7生成、上传并启动程序")
                .arg(static_cast<int>(moveInfos.size()))
                .arg(weldModeText)
                .arg(weldSpeedMmPerMin, 0, 'f', 3)
                .arg(weldSpeedSourceText)
                .arg(weldCommandSpeed, 0, 'f', 3)
                .arg(weldCommandSpeedUnit)
                .arg(programNameText);

        if (setFlowStep)
        {
            setFlowStep(QString("正在执行STEP焊接轨迹程序：%1").arg(programNameText));
        }
        if (appendLog)
        {
            appendLog(QString("开始执行STEP焊接轨迹：程序=%1，轨迹长度≈%2 mm，预计运行≈%3 s，完成超时=%4 s")
                .arg(programNameText)
                .arg(pathLengthMm, 0, 'f', 3)
                .arg(estimatedRunMs / 1000.0, 0, 'f', 1)
                .arg(finishTimeoutMs / 1000.0));
        }

        const int ret = pStepDriver != nullptr
            ? pStepDriver->ContiMoveAnyWithProgramName(moveInfos, stepProgramName)
            : pRobotDriver->ContiMoveAny(moveInfos);
        if (ret != 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            error = detail.isEmpty()
                ? QString("STEP焊接轨迹下发/启动失败：ret=%1").arg(ret)
                : QString("STEP焊接轨迹下发/启动失败：ret=%1，%2").arg(ret).arg(detail);
            return false;
        }

        lastState = pRobotDriver->CheckRobotDone(100);
        if (lastState <= 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            error = detail.isEmpty()
                ? QString("STEP焊接轨迹等待完成失败：CheckRobotDone=%1").arg(lastState)
                : QString("STEP焊接轨迹等待完成失败：CheckRobotDone=%1，%2").arg(lastState).arg(detail);
            return false;
        }
        if (appendLog)
        {
            appendLog(QString("STEP焊接轨迹执行完成：CheckRobotDone=%1").arg(lastState));
        }
    }

    if (checkpoint && !checkpoint(
        "焊后确认",
        QString("焊接轨迹已执行完成。\n程序：%1\n完成状态=%2\n是否继续移动到收枪安全位置？")
            .arg(programNameText)
            .arg(lastState)))
    {
        error = "用户在焊后确认节点取消了流程。";
        return false;
    }

    if (!MoveCoorsAndWait(
        pRobotDriver,
        endSafeCoors,
        safeMoveSpeedMmPerMin,
        "收枪安全位置",
        appendLog,
        setFlowStep))
    {
        error = "移动到收枪安全位置失败。";
        return false;
    }

    summary = QString("%1；安全移动速度=%2 mm/min；起点安全位=%3；终点安全位=%4")
        .arg(downlinkSummary)
        .arg(safeMoveSpeedMmPerMin, 0, 'f', 3)
        .arg(RobotCoorsText(startSafeCoors))
        .arg(RobotCoorsText(endSafeCoors));
    return true;
}
