#include "MeasureThenWeldService.h"

#include "CameraFrameCache.h"
#include "ConfigDatabase.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "MeasureThenWeldRuntimeConfig.h"
#include "OPini.h"
#include "PointCloudExtractionProcessor.h"
#include "PointCloudProcessingConfig.h"
#include "WorkpieceMeshBuilder.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "RobotPoseTransform.h"
#include "STEPRobotDriver.h"
#include "WeldProcessFile.h"
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
constexpr auto CORNER_COMP_KEY_POINTS_FILE_NAME = "PreciseLaserPoint_CornerComp_KeyPoints.txt";
constexpr auto CORNER_COMP_CLASSIFIED_FILE_NAME = "PreciseLaserPoint_CornerComp_Classified.txt";
constexpr auto CLASSIFIED_NOISE_FILE_NAME = "PreciseLaserPoint_Classified_Noise.txt";
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";
constexpr auto WELD_SEGMENT_KIND_DEBUG_FILE_NAME = "PreciseLaserPointSegmentKind.txt";
constexpr auto MATCH_DEBUG_FILE_NAME = "PreciseLaserPoint_MatchDebug.csv";
// 四种处理方法各自的基础焊道文件：处理成功时落盘，文件存在即表示该目录已按该方法完成焊道生成。
constexpr auto METHOD_TRACK_SDK_CLASS_FILE_NAME = "PreciseLaserPoint_SdkClass.txt";   // ①SDK拐点扩充焊道
constexpr auto METHOD_TRACK_SDK_BASE_FILE_NAME = "PreciseLaserPoint_SdkBase.txt";     // ②SDK基础焊道
constexpr auto METHOD_TRACK_POINT_BASE_FILE_NAME = "PreciseLaserPoint_PointBase.txt"; // ③点云投影提取焊道
constexpr auto METHOD_TRACK_POINT_LASER_FILE_NAME = "PreciseLaserPoint_PointLaser.txt"; // ④相机目标点焊道

constexpr auto SDK_POINT_CLOUD_OUTPUT_DIR_NAME = "SdkPointCloud";
constexpr auto SDK_SEAM_EXTRACTED_FILE_NAME = "PreciseLaserPoint_SdkSeamExtracted.txt";
constexpr auto SDK_SEAM_EXTRACTED_2MM_FILE_NAME = "PreciseLaserPoint_SdkSeamExtracted_2mm.txt";
constexpr auto SDK_BASE_WELD_FILE_NAME = "PreciseLaserPoint_SdkBaseWeld.txt";
constexpr auto SDK_SCHEME_COMPARE_DIR_NAME = "SchemeCompare";
constexpr int POSE_COMP_MATCH_BY_POSE = 0;
constexpr int POSE_COMP_MATCH_BY_SEGMENT_CODE = 1;
constexpr char POSE_COMP_MATCH_MODE_KEY[] = "PoseCompMatchMode";
constexpr int COMP_SEGMENT_COUNT = 4;
constexpr char POSE_GROUP_COUNT_KEY[] = "PoseCompGroupCount";
constexpr char POSE_ACTIVE_GROUP_INDEX_KEY[] = "ActivePoseCompGroupIndex";
constexpr char SEAM_GROUP_COUNT_KEY[] = "SeamCompGroupCount";
constexpr char SEAM_ACTIVE_GROUP_INDEX_KEY[] = "ActiveSeamCompGroupIndex";
constexpr char CORNER_COMP_ENABLED_KEY[] = "Enabled";
constexpr char INNER_TO_OUTER_CORNER_COMP_KEY[] = "InnerToOuter";
constexpr char INNER_TO_INNER_CORNER_COMP_KEY[] = "InnerToInner";
constexpr char OUTER_TO_OUTER_CORNER_COMP_KEY[] = "OuterToOuter";
constexpr char OUTER_TO_INNER_CORNER_COMP_KEY[] = "OuterToInner";
constexpr double DEFAULT_FINAL_WELD_TRAJECTORY_SAMPLE_STEP_MM = 4.0;

double NormalizeFinalWeldTrajectorySampleStepMm(double value)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        return DEFAULT_FINAL_WELD_TRAJECTORY_SAMPLE_STEP_MM;
    }
    return std::clamp(value, 0.5, 100.0);
}

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
    double measureReferenceRx = 0.0;
    double measureReferenceRy = 0.0;
    double measureReferenceRz = 0.0;
    double gunToolBaseRz = 180.0;
    double poseMatchMaxErrorDeg = 5.0;
    int poseCompMatchMode = POSE_COMP_MATCH_BY_POSE;
    double cornerTransitionLeadDistance = 10.0;
    double cornerArcRadiusMm = 0.0;
    double finalWeldStepFromProcessMm = 0.0;  // 工艺里的实际焊道点间距(>0 优先于测量参数页的值)
    double weldStartSkipDistance = 10.0;
    double weldEndSkipDistance = 10.0;
    double weldRzGainDeg = 0.0;
    bool useTaughtWeldPose = false;
    double taughtWeldPoseRx = 0.0;
    double taughtWeldPoseRy = 0.0;
    double taughtWeldPoseRz = 0.0;
    double slopeRzMinDeg = -20.0;
    double slopeRzMaxDeg = 20.0;
    double stepOverlapRel = 20.0;
    int weldDirection = 1;
    bool weldProcessLoaded = false;
    QString weldProcessLoadError;
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
    int arcMode = 4;
    double transitionSpeedMmPerMin = 0.0;
    double transitionCurrent = 0.0;
    double transitionVoltage = 0.0;
    bool weaveEnabled = true;
    bool trackEnabled = true;
    T_WeaveDate weaveParam;
    T_TrackData trackParam;
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

QString GeometryStrategyName(RobotCalculation::LowerWeldGeometryStrategy strategy)
{
    switch (strategy)
    {
    case RobotCalculation::LowerWeldGeometryStrategy::WorkpieceProjection:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::WorkpieceProjection);
    case RobotCalculation::LowerWeldGeometryStrategy::SlopeWaveFiltered:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered);
    case RobotCalculation::LowerWeldGeometryStrategy::RobustSegmentedKeys:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys);
    case RobotCalculation::LowerWeldGeometryStrategy::LegacyGeometry:
    default:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::LegacyGeometry);
    }
}

QString PoseCornerGroupModule(int groupIndex)
{
    return QStringLiteral("WeldPoseCompParam/CornerCompensation/Group%1").arg(groupIndex);
}

QString PoseCornerSlotModule(int flatIndex)
{
    return QStringLiteral("WeldPoseCompParam/CornerCompensation/Slot%1").arg(flatIndex);
}

bool ReadRobotScopedSetting(const QString& robotName, const QString& module, const QString& key, QString* value)
{
    if (robotName.trimmed().isEmpty())
    {
        return false;
    }
    return ConfigDatabase::ReadScopedSetting(QStringLiteral("robot"), robotName.trimmed(), module, key, value);
}

int ReadRobotScopedInt(const QString& robotName, const QString& module, const QString& key, int defaultValue)
{
    QString value;
    if (!ReadRobotScopedSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    bool ok = false;
    const int parsed = value.trimmed().toInt(&ok);
    return ok ? parsed : defaultValue;
}

bool ReadRobotScopedBool(const QString& robotName, const QString& module, const QString& key, bool defaultValue)
{
    QString value;
    if (!ReadRobotScopedSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    const QString normalized = value.trimmed().toLower();
    return normalized == QStringLiteral("1")
        || normalized == QStringLiteral("true")
        || normalized == QStringLiteral("yes");
}

double ReadRobotScopedDouble(const QString& robotName, const QString& module, const QString& key, double defaultValue)
{
    QString value;
    if (!ReadRobotScopedSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    bool ok = false;
    const double parsed = value.trimmed().toDouble(&ok);
    return ok ? parsed : defaultValue;
}

RobotCalculation::LowerWeldFilterParams::CornerCompensation ReadCornerCompensationSlot(
    const QString& robotName,
    int flatIndex)
{
    RobotCalculation::LowerWeldFilterParams::CornerCompensation comp;
    const QString module = PoseCornerSlotModule(flatIndex);
    comp.innerToOuterMm = ReadRobotScopedDouble(robotName, module, INNER_TO_OUTER_CORNER_COMP_KEY, 0.0);
    comp.innerToInnerMm = ReadRobotScopedDouble(robotName, module, INNER_TO_INNER_CORNER_COMP_KEY, 0.0);
    comp.outerToOuterMm = ReadRobotScopedDouble(robotName, module, OUTER_TO_OUTER_CORNER_COMP_KEY, 0.0);
    comp.outerToInnerMm = ReadRobotScopedDouble(robotName, module, OUTER_TO_INNER_CORNER_COMP_KEY, 0.0);
    return comp;
}

bool HasCornerCompensationValue(const RobotCalculation::LowerWeldFilterParams::CornerCompensation& comp)
{
    return std::abs(comp.innerToOuterMm) > 1e-9
        || std::abs(comp.innerToInnerMm) > 1e-9
        || std::abs(comp.outerToOuterMm) > 1e-9
        || std::abs(comp.outerToInnerMm) > 1e-9;
}

void LoadActivePoseCornerCompensation(
    const QString& robotName,
    RobotCalculation::LowerWeldFilterParams& params)
{
    const QString allModule = QStringLiteral("WeldPoseCompParam/ALLWeldPoseComp");
    int activeGroupIndex = ReadRobotScopedInt(robotName, allModule, POSE_ACTIVE_GROUP_INDEX_KEY, 0);
    const int groupCount = ReadRobotScopedInt(robotName, allModule, POSE_GROUP_COUNT_KEY, activeGroupIndex + 1);
    if (groupCount > 0)
    {
        activeGroupIndex = std::clamp(activeGroupIndex, 0, groupCount - 1);
    }
    else
    {
        activeGroupIndex = 0;
    }

    if (!ReadRobotScopedBool(robotName, PoseCornerGroupModule(activeGroupIndex), CORNER_COMP_ENABLED_KEY, false))
    {
        params.enableCornerCompensation = false;
        params.risingCornerCompensation = {};
        params.fallingCornerCompensation = {};
        return;
    }

    const int offset = activeGroupIndex * COMP_SEGMENT_COUNT;
    params.risingCornerCompensation = ReadCornerCompensationSlot(robotName, offset + 1);
    params.fallingCornerCompensation = ReadCornerCompensationSlot(robotName, offset + 3);
    params.enableCornerCompensation =
        HasCornerCompensationValue(params.risingCornerCompensation)
        || HasCornerCompensationValue(params.fallingCornerCompensation);
}

RobotCalculation::LowerWeldFilterParams BuildOriginalTrackFitParams(const T_PRECISE_MEASURE_PARAM& param)
{
    // 参数名册唯一来源在 MeasureThenWeldService::BuildTrackFitParamsFromSettings（CLI 共用）。
    RobotCalculation::LowerWeldFilterParams params = MeasureThenWeldService::BuildTrackFitParamsFromSettings(
        PointCloudProcessingConfig::Load(),
        InferMeasureSampleAxis(param));
    LoadActivePoseCornerCompensation(QString::fromStdString(param.sRobotName), params);
    return params;
}

Eigen::Vector3d BuildScanDirection(const T_PRECISE_MEASURE_PARAM& param)
{
    double overrideX = 0.0;
    double overrideY = 0.0;
    double overrideZ = 0.0;
    if (PointCloudProcessingConfig::RuntimeScanDirectionOverride(&overrideX, &overrideY, &overrideZ))
    {
        Eigen::Vector3d overrideDirection(overrideX, overrideY, overrideZ);
        if (overrideDirection.norm() <= std::numeric_limits<double>::epsilon())
        {
            return Eigen::Vector3d::UnitX();
        }
        return overrideDirection.normalized();
    }

    Eigen::Vector3d direction(
        param.tEndPos.dX - param.tStartPos.dX,
        param.tEndPos.dY - param.tStartPos.dY,
        param.tEndPos.dZ - param.tStartPos.dZ);
    if (direction.norm() <= std::numeric_limits<double>::epsilon())
    {
        direction = Eigen::Vector3d::UnitX();
    }
    direction.normalize();
    return direction;
}

QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<RobotCalculation::LowerWeldFilterPoint>& points);
QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points);
std::vector<QString> BuildRawLaserOutputLines(const QVector<RobotCalculation::LowerWeldFilterPoint>& points);

// SDK 已焊起点截断：已焊段从上次焊接的起始端延伸，截掉的一侧由"焊接方向"决定——
// 起点到终点焊（weldFromTrackStart=true）已焊段在轨迹头部，保留交界点及其后段并把首点重置为起点；
// 终点到起点焊已焊段在轨迹尾部，保留头部到交界点的段并把尾点重置为终点。
// 两种方向下执行（含方向反转）都恰好从已焊交界处开始接续焊接。
QVector<PointCloudExtractionProcessor::TrackPoint> TruncateTrackAtWeldedStart(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points,
    const Eigen::Vector3d& weldedStart,
    bool weldFromTrackStart,
    int* removedCount)
{
    if (removedCount != nullptr)
    {
        *removedCount = 0;
    }
    if (points.size() < 2)
    {
        return points;
    }
    int nearestIndex = 0;
    double bestDistance = std::numeric_limits<double>::max();
    for (int index = 0; index < points.size(); ++index)
    {
        const double distance = (points[index].point - weldedStart).norm();
        if (distance < bestDistance)
        {
            bestDistance = distance;
            nearestIndex = index;
        }
    }

    if (weldFromTrackStart)
    {
        // 截掉头部已焊段；截断后至少保留两个点，否则视为截断无效。
        if (nearestIndex <= 0 || points.size() - nearestIndex < 2)
        {
            return points;
        }
        QVector<PointCloudExtractionProcessor::TrackPoint> truncated = points.mid(nearestIndex);
        truncated.first().type = PointCloudExtractionProcessor::TrackPointType::Start;
        if (removedCount != nullptr)
        {
            *removedCount = nearestIndex;
        }
        return truncated;
    }

    // 终点到起点焊：截掉尾部已焊段。
    if (nearestIndex >= points.size() - 1 || nearestIndex < 1)
    {
        return points;
    }
    QVector<PointCloudExtractionProcessor::TrackPoint> truncated = points.mid(0, nearestIndex + 1);
    truncated.last().type = PointCloudExtractionProcessor::TrackPointType::End;
    if (removedCount != nullptr)
    {
        *removedCount = points.size() - truncated.size();
    }
    return truncated;
}

// 方法基础焊道行格式与 PreciseLaserPoint 系列一致：index x y z。
template <typename Container>
std::vector<QString> BuildMethodTrackLines(const Container& points)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()));
    for (const auto& point : points)
    {
        lines.push_back(QString("%1 %2 %3 %4")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6));
    }
    return lines;
}

bool WriteTextLinesToFile(const QString& path, const std::vector<QString>& lines, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("无法写入文件：%1（%2）").arg(path, file.errorString());
        }
        return false;
    }
    QTextStream stream(&file);
    for (const QString& line : lines)
    {
        stream << line << '\n';
    }
    stream.flush();
    file.close();
    return true;
}

// 工件模型缓存：处理成功后用内存中的完整点云直接生成（免二次解析 150MB 文本），
// 缓存已存在则跳过；之后查看器/CloudCompare 直接秒开。失败不影响焊接流程。
void EnsureWorkpieceMeshCacheFromCloud(
    const QString& laserDir,
    const QVector<RobotCalculation::IndexedPoint3D>& cloudPoints,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (laserDir.trimmed().isEmpty() || cloudPoints.size() < 16)
    {
        return;
    }
    const QString cachePath = WorkpieceMeshBuilder::MeshCachePath(laserDir);
    if (WorkpieceMeshBuilder::IsMeshCacheValid(cachePath))
    {
        return;
    }
    QString meshError;
    WorkpieceMeshBuilder::Mesh mesh;
    if (!WorkpieceMeshBuilder::BuildFromScanlineCloud(cloudPoints, mesh, meshError)
        || !WorkpieceMeshBuilder::SaveMeshPly(cachePath, mesh, meshError))
    {
        if (appendLog)
        {
            appendLog(QString("工件模型缓存生成失败（不影响流程）：%1").arg(meshError));
        }
        return;
    }
    if (appendLog)
    {
        appendLog(QString("已生成工件模型缓存：%1（顶点 %2 / 三角形 %3）")
            .arg(cachePath)
            .arg(mesh.vertices.size())
            .arg(mesh.indices.size() / 3));
    }
}

// 处理成功时把"该方法的基础焊道"落盘到 LaserPoint 目录（文件存在=该方法已完成焊道生成）。
void SaveMethodBaseTrackFile(
    const QString& laserDir,
    PointCloudProcessingConfig::Mode mode,
    const std::vector<QString>& lines,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (laserDir.trimmed().isEmpty() || lines.empty())
    {
        return;
    }
    const QString path = QDir(laserDir).filePath(MeasureThenWeldService::MethodBaseTrackFileName(mode));
    QString saveError;
    if (!WriteTextLinesToFile(path, lines, &saveError) && appendLog)
    {
        appendLog(QString("方法基础焊道写入失败（不影响处理）：%1").arg(saveError));
    }
}

RobotCalculation::MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldPointCloud(
    const QVector<RobotCalculation::IndexedPoint3D>& legacyLaserInput,
    const QVector<RobotCalculation::IndexedPoint3D>& fullCloudInput,
    const T_PRECISE_MEASURE_PARAM& param,
    const RobotCalculation::LowerWeldFilterParams& fitParams,
    const QString& sdkBaseWeldOutputPath,
    const QString& methodTrackOutputDir,
    const MeasureThenWeldService::LogCallback& appendLog,
    bool* usedExternalLibrary = nullptr,
    PointCloudExtractionProcessor::ExtractionResult* externalExtraction = nullptr)
{
    if (usedExternalLibrary != nullptr)
    {
        *usedExternalLibrary = false;
    }
    if (externalExtraction != nullptr)
    {
        *externalExtraction = PointCloudExtractionProcessor::ExtractionResult();
    }

    const PointCloudProcessingConfig::Settings settings = PointCloudProcessingConfig::Load();
    if (appendLog)
    {
        appendLog(QString("精测点云处理方式：%1。")
            .arg(PointCloudProcessingConfig::ModeDisplayName(settings.mode)));
        appendLog(QString("特征点拟合方案：%1。")
            .arg(GeometryStrategyName(fitParams.geometryStrategy)));
    }

    const bool sdkMode = settings.mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        || settings.mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit;
    if (sdkMode)
    {
        // ①SDK全处理：SDK 拐点(+2mm扩充)直接转结果，不经拟合、不生成基础焊道文件；
        // ②SDK+拟合：SDK 输出基础焊道（稠密），再喂滤波拟合提取特征点。
        // 失败直接报错，不回退其他方法。
        const bool useBaseWeldFit = settings.mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit;
        if (fullCloudInput.size() < 2)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("SDK点云算法输入点过少（%1），无法调用外部库。")
                .arg(fullCloudInput.size());
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        const PointCloudExtractionProcessor::ExtractionResult extraction =
            PointCloudExtractionProcessor::ExtractCorrugatedSheet(
                fullCloudInput,
                settings,
                BuildScanDirection(param),
                useBaseWeldFit ? sdkBaseWeldOutputPath : QString());
        if (!extraction.ok)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("SDK点云算法处理失败：%1").arg(extraction.error);
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        // 已焊起点截断（开关控制）：SDK 检测到已焊段时，按焊接方向截掉焊道已焊部分只焊剩余段。
        PointCloudExtractionProcessor::ExtractionResult workingExtraction = extraction;
        if (settings.sdkUseWeldedStartTruncation)
        {
            if (extraction.hasWeldedStartPoint)
            {
                const bool weldFromTrackStart = param.nWeldDirection >= 0;
                int removedCount = 0;
                workingExtraction.points = TruncateTrackAtWeldedStart(
                    extraction.points, extraction.weldedStartPoint, weldFromTrackStart, &removedCount);
                if (appendLog)
                {
                    if (removedCount > 0)
                    {
                        appendLog(QString("已焊起点截断：交界点(%1, %2, %3)，焊接方向=%4，截除已焊侧 %5 点，保留 %6 点。")
                            .arg(extraction.weldedStartPoint.x(), 0, 'f', 2)
                            .arg(extraction.weldedStartPoint.y(), 0, 'f', 2)
                            .arg(extraction.weldedStartPoint.z(), 0, 'f', 2)
                            .arg(weldFromTrackStart ? "起点到终点" : "终点到起点")
                            .arg(removedCount)
                            .arg(workingExtraction.points.size()));
                    }
                    else
                    {
                        appendLog("已焊起点截断：交界点位于焊道端部或截断后点数不足，焊道未截断。");
                    }
                }
            }
            else if (appendLog)
            {
                appendLog("已焊起点截断：SDK 未检测到已焊段，焊道不截断。");
            }
        }
        RobotCalculation::MeasureThenWeldAnalysisResult analysis = useBaseWeldFit
            ? RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
                ToIndexedInput(workingExtraction.points), fitParams)
            : PointCloudExtractionProcessor::BuildAnalysisResult(workingExtraction, fitParams);
        if (!analysis.ok)
        {
            if (appendLog)
            {
                appendLog(QString("SDK点云算法结果%1失败：%2")
                    .arg(useBaseWeldFit ? "拟合" : "转换")
                    .arg(analysis.error));
            }
            return analysis;
        }
        if (usedExternalLibrary != nullptr)
        {
            *usedExternalLibrary = true;
        }
        if (externalExtraction != nullptr)
        {
            *externalExtraction = extraction;
        }
        if (appendLog)
        {
            appendLog(QString("SDK点云算法处理完成（%1）：输入局部完整点云=%2，SDK输出点=%3，DLL=%4，配置=%5，Z截断=%6 mm。")
                .arg(useBaseWeldFit ? "基础焊道+拟合" : "拐点直接使用")
                .arg(extraction.inputPointCount)
                .arg(extraction.points.size())
                .arg(extraction.dllPath)
                .arg(extraction.configPath)
                .arg(settings.zTruncationValue, 0, 'f', 3));
            if (extraction.usedBaseWeldFile)
            {
                appendLog(QString("SDK基础焊道来自库输出文件：%1").arg(extraction.baseWeldPath));
            }
        }
        SaveMethodBaseTrackFile(
            methodTrackOutputDir, settings.mode, BuildMethodTrackLines(workingExtraction.points), appendLog);
        return analysis;
    }
    if (settings.mode == PointCloudProcessingConfig::Mode::CloudFit)
    {
        // ③点云算法+拟合：立板投影提取（完整点云 + 相机轨迹种子 → 下层轨迹）→ 滤波拟合。
        // 几何链按单条轨迹设计，面状完整点云必须先经投影提取压成轨迹。失败直接报错，不回退。
        if (fullCloudInput.size() < 3)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("点云算法+拟合输入完整点云点数过少（%1）。").arg(fullCloudInput.size());
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        if (legacyLaserInput.size() < 2)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("点云算法+拟合缺少相机轨迹种子点（%1），无法定位底板候选。")
                .arg(legacyLaserInput.size());
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        const RobotCalculation::LowerWeldFilterResult projectedPath =
            RobotCalculation::ProjectWorkpieceCloudToLowerWeldPath(fullCloudInput, legacyLaserInput, fitParams);
        if (!projectedPath.ok)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("点云投影提取失败：%1").arg(projectedPath.error);
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        if (appendLog)
        {
            appendLog(QString("点云投影提取完成：完整点云=%1，种子点=%2，底板候选点=%3，投影轨迹点=%4。")
                .arg(fullCloudInput.size())
                .arg(legacyLaserInput.size())
                .arg(projectedPath.lowerPointCount)
                .arg(projectedPath.points.size()));
        }
        RobotCalculation::MeasureThenWeldAnalysisResult analysis =
            RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
                ToIndexedInput(projectedPath.points), fitParams);
        if (!analysis.ok)
        {
            if (appendLog)
            {
                appendLog(QString("点云算法+拟合处理失败：%1").arg(analysis.error));
            }
            return analysis;
        }
        SaveMethodBaseTrackFile(
            methodTrackOutputDir,
            PointCloudProcessingConfig::Mode::CloudFit,
            BuildMethodTrackLines(projectedPath.points),
            appendLog);
        return analysis;
    }

    // ④特征点+拟合：相机目标轨迹点 → 滤波拟合。
    if (appendLog)
    {
        appendLog(QString("特征点+拟合输入轨迹点数：%1。")
            .arg(legacyLaserInput.size()));
    }
    RobotCalculation::MeasureThenWeldAnalysisResult legacyAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(legacyLaserInput, fitParams);
    if (legacyAnalysis.ok)
    {
        SaveMethodBaseTrackFile(
            methodTrackOutputDir,
            PointCloudProcessingConfig::Mode::LegacyLaserPath,
            BuildMethodTrackLines(legacyLaserInput),
            appendLog);
    }
    return legacyAnalysis;
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

QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points)
{
    QVector<RobotCalculation::IndexedPoint3D> indexedPoints;
    indexedPoints.reserve(points.size());
    for (const PointCloudExtractionProcessor::TrackPoint& point : points)
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

std::vector<QString> BuildIndexedPointOutputLines(
    const QVector<RobotCalculation::IndexedPoint3D>& points,
    const QString& source)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 1);
    lines.push_back("index x y z source");
    for (const RobotCalculation::IndexedPoint3D& point : points)
    {
        lines.push_back(QString("%1 %2 %3 %4 %5")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(source));
    }
    return lines;
}

std::vector<QString> BuildRawLaserOutputLines(const QVector<RobotCalculation::LowerWeldFilterPoint>& points)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 1);
    lines.push_back("index,x,y,z");
    for (const RobotCalculation::LowerWeldFilterPoint& point : points)
    {
        lines.push_back(RobotCalculation::Vector3IndexedCsv(point.index, point.point));
    }
    return lines;
}

QString SdkTrackPointTypeName(PointCloudExtractionProcessor::TrackPointType type)
{
    using TrackType = PointCloudExtractionProcessor::TrackPointType;
    switch (type)
    {
    case TrackType::Start:
        return "start";
    case TrackType::End:
        return "end";
    case TrackType::Corner:
        return "corner";
    case TrackType::Normal:
    default:
        return "normal";
    }
}

int SdkTrackPointTypeCode(PointCloudExtractionProcessor::TrackPointType type)
{
    using TrackType = PointCloudExtractionProcessor::TrackPointType;
    switch (type)
    {
    case TrackType::Start:
        return 1;
    case TrackType::End:
        return 2;
    case TrackType::Corner:
        return 3;
    case TrackType::Normal:
    default:
        return 5;
    }
}

std::vector<QString> BuildSdkTrackOutputLines(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points,
    const QString& source)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 2);
    lines.push_back("# index x y z sdk_type_code sdk_type_name source");
    lines.push_back("# 1=start 2=end 3=corner 5=normal");
    for (const PointCloudExtractionProcessor::TrackPoint& point : points)
    {
        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(SdkTrackPointTypeCode(point.type))
            .arg(SdkTrackPointTypeName(point.type))
            .arg(source));
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

RobotCalculation::LowerWeldFilterParams BuildSchemeCompareFitParams(
    const RobotCalculation::LowerWeldFilterParams& params)
{
    RobotCalculation::LowerWeldFilterParams compareParams = params;
    compareParams.validationCoverageEnabled = false;
    compareParams.validationContinuityEnabled = false;
    compareParams.validationDenoiseRatioEnabled = false;
    compareParams.validationResidualEnabled = false;
    compareParams.validationKeyPointEnabled = false;
    compareParams.validationOutputEnabled = false;
    return compareParams;
}

std::vector<QString> BuildSchemeCompareSummaryLines(
    const QString& title,
    const QString& inputDescription,
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const RobotCalculation::MeasureThenWeldAnalysisResult& analysis)
{
    std::vector<QString> lines;
    lines.reserve(12);
    lines.push_back(QString("方案=%1").arg(title));
    lines.push_back(QString("输入=%1").arg(inputDescription));
    lines.push_back(QString("输入点数=%1").arg(inputPoints.size()));
    lines.push_back(QString("状态=%1").arg(analysis.ok ? "OK" : "FAIL"));
    if (!analysis.ok)
    {
        lines.push_back(QString("错误=%1").arg(analysis.error));
        return lines;
    }
    lines.push_back(QString("拟合输入点=%1").arg(analysis.filterResult.inputPointCount));
    lines.push_back(QString("保留点=%1").arg(analysis.filterResult.points.size()));
    lines.push_back(QString("关键点=%1").arg(analysis.keyPoints.size()));
    lines.push_back(QString("2mm扩充点=%1").arg(analysis.classificationResult.points.size()));
    lines.push_back(QString("起点=%1").arg(analysis.classificationResult.startCount));
    lines.push_back(QString("终点=%1").arg(analysis.classificationResult.endCount));
    lines.push_back(QString("内拐点=%1").arg(analysis.classificationResult.innerCornerCount));
    lines.push_back(QString("外拐点=%1").arg(analysis.classificationResult.outerCornerCount));
    return lines;
}

bool SaveSchemeAnalysisOutputs(
    const MeasureThenWeldService& service,
    const QString& compareDir,
    const QString& prefix,
    const QString& title,
    const QString& inputDescription,
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const RobotCalculation::MeasureThenWeldAnalysisResult& analysis,
    QString& error)
{
    const QDir dir(compareDir);
    if (!service.SaveTextLines(
            dir.filePath(QString("%1_InputPointCloud.txt").arg(prefix)),
            BuildIndexedPointOutputLines(inputPoints, inputDescription),
            error))
    {
        return false;
    }
    if (!service.SaveTextLines(
            dir.filePath(QString("%1_Summary.txt").arg(prefix)),
            BuildSchemeCompareSummaryLines(title, inputDescription, inputPoints, analysis),
            error))
    {
        return false;
    }
    if (!analysis.ok)
    {
        return true;
    }
    return service.SaveTextLines(
            dir.filePath(QString("%1_PreservePath.txt").arg(prefix)),
            BuildFilterOutputLines(analysis.filterResult),
            error)
        && service.SaveTextLines(
            dir.filePath(QString("%1_KeyPoints.txt").arg(prefix)),
            BuildKeyPointOutputLines(analysis.keyPoints),
            error)
        && service.SaveTextLines(
            dir.filePath(QString("%1_Classified_2mm.txt").arg(prefix)),
            BuildClassifiedOutputLines(analysis.classificationResult),
            error);
}

bool SaveSdkSchemeCompareOutputs(
    const MeasureThenWeldService& service,
    const QString& sdkPointCloudDir,
    const QVector<RobotCalculation::IndexedPoint3D>& originalLaserInput,
    const PointCloudExtractionProcessor::ExtractionResult& extraction,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString& error,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    const QString compareDir = QDir(sdkPointCloudDir).filePath(SDK_SCHEME_COMPARE_DIR_NAME);
    if (!QDir().mkpath(compareDir))
    {
        error = QString("创建SDK方案对比目录失败：%1").arg(compareDir);
        return false;
    }

    const QDir dir(compareDir);
    std::vector<QString> featureSummaryLines;
    featureSummaryLines.reserve(5);
    featureSummaryLines.push_back("方案=特征点方案");
    featureSummaryLines.push_back("输入=sdk_feature_point");
    featureSummaryLines.push_back(QString("关键点=%1").arg(extraction.rawPoints.size()));
    featureSummaryLines.push_back(QString("2mm扩充点=%1").arg(extraction.keyPointExpandedPoints.size()));
    if (!service.SaveTextLines(
            dir.filePath("FeaturePoint_KeyPoints.txt"),
            BuildSdkTrackOutputLines(extraction.rawPoints, "sdk_feature_point"),
            error)
        || !service.SaveTextLines(
            dir.filePath("FeaturePoint_KeyPointExpanded_2mm.txt"),
            BuildSdkTrackOutputLines(extraction.keyPointExpandedPoints, "sdk_feature_point_2mm"),
            error)
        || !service.SaveTextLines(
            dir.filePath("FeaturePoint_Summary.txt"),
            featureSummaryLines,
            error))
    {
        return false;
    }

    const RobotCalculation::LowerWeldFilterParams compareParams =
        BuildSchemeCompareFitParams(params);
    const QVector<RobotCalculation::IndexedPoint3D> baseWeldInput = ToIndexedInput(extraction.points);
    const RobotCalculation::MeasureThenWeldAnalysisResult baseWeldFitAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(baseWeldInput, compareParams);
    if (!SaveSchemeAnalysisOutputs(
            service,
            compareDir,
            "BaseWeldPointCloudFit",
            "点云+拟合方案",
            "sdk_base_weld",
            baseWeldInput,
            baseWeldFitAnalysis,
            error))
    {
        return false;
    }

    const RobotCalculation::MeasureThenWeldAnalysisResult originalPointCloudFitAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(originalLaserInput, compareParams);
    if (!SaveSchemeAnalysisOutputs(
            service,
            compareDir,
            "OriginalPointCloudFit",
            "点云拟合方案",
            "original_laser_point",
            originalLaserInput,
            originalPointCloudFitAnalysis,
            error))
    {
        return false;
    }

    if (appendLog)
    {
        appendLog(QString("SDK三方案对比输出目录：%1；特征点=%2，点云+拟合关键点=%3，点云拟合关键点=%4")
            .arg(compareDir)
            .arg(extraction.rawPoints.size())
            .arg(baseWeldFitAnalysis.ok ? baseWeldFitAnalysis.keyPoints.size() : 0)
            .arg(originalPointCloudFitAnalysis.ok ? originalPointCloudFitAnalysis.keyPoints.size() : 0));
    }
    return true;
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

    // 测量姿态只用于焊道法向正反、平台深度轴和RZ参考，不能覆盖参数组里固定的焊接RX/RY。
    preset.measureReferenceRx = reference.rx;
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

bool IsPlatformSegmentKind(const QString& segmentKind)
{
    return segmentKind.compare("low_platform", Qt::CaseInsensitive) == 0
        || segmentKind.compare("high_platform", Qt::CaseInsensitive) == 0;
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

int NormalizeArcMode(int mode)
{
    return mode >= 0 && mode <= 7 ? mode : 4;
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

bool TryLoadActiveWeldProcessParam(const QString& robotName, T_WELD_PARA& weldPara, QString* error)
{
    if (robotName.trimmed().isEmpty())
    {
        if (error != nullptr)
        {
            *error = "机器人名称为空，无法读取当前工艺。";
        }
        return false;
    }

    // 复用 WeldProcessFile（多键主数据优先 + 文本块回退 + BindWeldToWeave 摆动绑定），
    // 与工艺页同一份读取逻辑，不再维护第二套文本解析器。
    WeldProcessFile processFile(ToUtf8StdString(robotName.trimmed()));
    if (!processFile.Init())
    {
        if (error != nullptr)
        {
            *error = QString::fromUtf8(processFile.GetLastError().c_str());
        }
        return false;
    }

    const T_WELD_PARA* activePara = processFile.GetUseWeldPara();
    if (activePara == nullptr)
    {
        if (error != nullptr)
        {
            *error = QString("工艺数据没有有效条目：%1").arg(robotName.trimmed());
        }
        return false;
    }

    weldPara = *activePara;  // 含 BindWeldToWeave 已灌入的 tWeaveParam
    return true;
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
    if (ini.SetFileName(param.sIniFilePath))
    {
        ini.SetSectionName(param.sWeldSectionName);
        ini.ReadString(false, "FinalWeldTrajectoryStepMm", &param.dFinalWeldTrajectoryStepMm);
    }
    param.dFinalWeldTrajectoryStepMm = NormalizeFinalWeldTrajectorySampleStepMm(param.dFinalWeldTrajectoryStepMm);
    return param;
}

void ApplyActiveWeldProcessToPreset(const T_PRECISE_MEASURE_PARAM& param, WeldPosePreset& preset)
{
    T_WELD_PARA weldPara = {};
    QString loadError;
    if (!TryLoadActiveWeldProcessParam(QString::fromStdString(param.sRobotName), weldPara, &loadError))
    {
        preset.weldProcessLoadError = loadError;
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
    preset.arcMode = NormalizeArcMode(weldPara.nArcMode);
    preset.weaveEnabled = weldPara.nWeaveEnable != 0;
    preset.trackEnabled = weldPara.nTrackEnable != 0;
    preset.weaveParam = weldPara.tWeaveParam;
    preset.trackParam = weldPara.tTrackParam;
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

    // 工艺里的实际焊道点间距（>0 时优先于测量参数页的 FinalWeldTrajectoryStepMm）。
    if (std::isfinite(weldPara.dFinalWeldTrajectoryStepMm)
        && weldPara.dFinalWeldTrajectoryStepMm > 0.0)
    {
        preset.finalWeldStepFromProcessMm = weldPara.dFinalWeldTrajectoryStepMm;
    }
}

WeldPosePreset LoadWeldPosePreset(const T_PRECISE_MEASURE_PARAM& param)
{
    WeldPosePreset preset;
    preset.rx = param.tStartPos.dRX;
    preset.ry = param.tStartPos.dRY;
    preset.weldRzGainDeg = param.dWeldRzGainDeg;
    preset.useTaughtWeldPose = param.bUseTaughtWeldPose;
    preset.taughtWeldPoseRx = std::isfinite(param.dTaughtWeldPoseRxDeg) ? param.dTaughtWeldPoseRxDeg : preset.rx;
    preset.taughtWeldPoseRy = std::isfinite(param.dTaughtWeldPoseRyDeg) ? param.dTaughtWeldPoseRyDeg : preset.ry;
    preset.taughtWeldPoseRz = std::isfinite(param.dTaughtWeldPoseRzDeg) ? param.dTaughtWeldPoseRzDeg : param.tStartPos.dRZ;
    preset.slopeRzMinDeg = param.dSlopeRzMinDeg;
    preset.slopeRzMaxDeg = param.dSlopeRzMaxDeg;
    NormalizeSlopeRzClamp(preset.slopeRzMinDeg, preset.slopeRzMaxDeg);
    preset.stepOverlapRel = std::isfinite(param.dStepOverlapRel) ? std::max(0.0, param.dStepOverlapRel) : 20.0;
    preset.weldDirection = param.nWeldDirection < 0 ? -1 : 1;
    preset.measureReferenceRx = param.tStartPos.dRX;
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
            int useTaughtWeldPose = preset.useTaughtWeldPose ? 1 : 0;
            double taughtWeldPoseRx = preset.taughtWeldPoseRx;
            double taughtWeldPoseRy = preset.taughtWeldPoseRy;
            double taughtWeldPoseRz = preset.taughtWeldPoseRz;
            double slopeRzMinDeg = preset.slopeRzMinDeg;
            double slopeRzMaxDeg = preset.slopeRzMaxDeg;
            double stepOverlapRel = preset.stepOverlapRel;
            const bool hasNormalRx = TryReadIniDouble(ini, "NormalWeldRx", rx);
            const bool hasNormalRy = TryReadIniDouble(ini, "NormalWeldRy", ry);
            ini.ReadString(false, "UseTaughtWeldPose", &useTaughtWeldPose);
            TryReadIniDouble(ini, "TaughtWeldPoseRX", taughtWeldPoseRx);
            TryReadIniDouble(ini, "TaughtWeldPoseRY", taughtWeldPoseRy);
            TryReadIniDouble(ini, "TaughtWeldPoseRZ", taughtWeldPoseRz);
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
                    rx = preset.rx;
                    ry = preset.ry;
                }
            }

            preset.rx = rx;
            preset.ry = ry;
            preset.cornerTransitionLeadDistance = std::max(0.0, cornerTransitionLeadDistance);
            preset.weldStartSkipDistance = std::max(0.0, weldStartSkipDistance);
            preset.weldEndSkipDistance = std::max(0.0, weldEndSkipDistance);
            preset.weldRzGainDeg = std::isfinite(weldRzGainDeg) ? weldRzGainDeg : 0.0;
            preset.useTaughtWeldPose = useTaughtWeldPose != 0;
            preset.taughtWeldPoseRx = std::isfinite(taughtWeldPoseRx) ? taughtWeldPoseRx : preset.rx;
            preset.taughtWeldPoseRy = std::isfinite(taughtWeldPoseRy) ? taughtWeldPoseRy : preset.ry;
            preset.taughtWeldPoseRz = std::isfinite(taughtWeldPoseRz) ? taughtWeldPoseRz : preset.measureReferenceRz;
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
    if ((beginType == PointType::Start && endType == PointType::OuterCorner)
        || (beginType == PointType::OuterCorner && endType == PointType::OuterCorner))
    {
        return "high_platform";
    }
    if (beginType == PointType::InnerCorner && endType == PointType::OuterCorner)
    {
        return "rising_edge";
    }
    if (beginType == PointType::OuterCorner && endType == PointType::InnerCorner)
    {
        return "falling_edge";
    }
    if (endType == PointType::End)
    {
        if (beginType == PointType::OuterCorner)
        {
            return "high_platform";
        }
        if (beginType == PointType::InnerCorner)
        {
            return "low_platform";
        }
    }
    return "segment";
}

Eigen::Vector3d UnitVectorOrZero(const Eigen::Vector3d& vector)
{
    const double norm = vector.norm();
    if (!std::isfinite(norm) || norm <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }
    return vector / norm;
}

Eigen::Vector3d MeasurementGunTipAxis(const WeldPosePreset& preset)
{
    Eigen::Vector3d axis = RobotPoseTransform::RotationFromAnglesDeg(
        preset.measureReferenceRx,
        preset.measureReferenceRy,
        preset.measureReferenceRz,
        preset.robotType) * Eigen::Vector3d(-1.0, 0.0, 0.0);
    axis = UnitVectorOrZero(axis);
    if (axis.norm() > 1e-9)
    {
        return axis;
    }
    return UnitVectorOrZero(GunDirectionVectorFromRobotRz(preset.measureReferenceRz));
}

QString SegmentKindFromDepthSide(bool beginIsLowPlatform, bool endIsLowPlatform)
{
    if (beginIsLowPlatform == endIsLowPlatform)
    {
        return beginIsLowPlatform ? "low_platform" : "high_platform";
    }
    return beginIsLowPlatform ? "rising_edge" : "falling_edge";
}

bool AssignSegmentKindsByMeasurementGunDepth(
    const QVector<RobotCalculation::LowerWeldClassifiedPoint>& points,
    const std::vector<int>& keyPointPositions,
    const WeldPosePreset& preset,
    std::vector<RobotCalculation::LowerWeldPointType>& keyPointTypes,
    QVector<QString>& segmentKinds,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    using PointType = RobotCalculation::LowerWeldPointType;
    segmentKinds.clear();
    if (keyPointPositions.size() < 2 || keyPointTypes.size() != keyPointPositions.size())
    {
        return false;
    }

    const int firstPosition = keyPointPositions.front();
    const int lastPosition = keyPointPositions.back();
    if (firstPosition < 0 || lastPosition < 0
        || firstPosition >= points.size()
        || lastPosition >= points.size())
    {
        return false;
    }

    const Eigen::Vector3d gunAxis = MeasurementGunTipAxis(preset);
    if (gunAxis.norm() <= 1e-9)
    {
        return false;
    }

    const Eigen::Vector3d travelAxis =
        UnitVectorOrZero(points[lastPosition].point - points[firstPosition].point);
    Eigen::Vector3d depthAxis = gunAxis;
    if (travelAxis.norm() > 1e-9)
    {
        depthAxis -= travelAxis * depthAxis.dot(travelAxis);
    }
    depthAxis = UnitVectorOrZero(depthAxis);
    if (depthAxis.norm() <= 1e-9)
    {
        depthAxis = gunAxis;
    }
    if (depthAxis.dot(gunAxis) < 0.0)
    {
        depthAxis = -depthAxis;
    }

    QVector<double> depths;
    depths.reserve(static_cast<int>(keyPointPositions.size()));
    double minDepth = std::numeric_limits<double>::max();
    double maxDepth = std::numeric_limits<double>::lowest();
    for (int keyPosition : keyPointPositions)
    {
        if (keyPosition < 0 || keyPosition >= points.size())
        {
            return false;
        }
        const double depth = points[keyPosition].point.dot(depthAxis);
        if (!std::isfinite(depth))
        {
            return false;
        }
        depths.push_back(depth);
        minDepth = std::min(minDepth, depth);
        maxDepth = std::max(maxDepth, depth);
    }

    const double depthRange = maxDepth - minDepth;
    if (!std::isfinite(depthRange) || depthRange <= 1e-6)
    {
        return false;
    }
    const double depthMidpoint = (minDepth + maxDepth) * 0.5;

    std::vector<bool> keyIsLowPlatform(keyPointPositions.size(), false);
    for (int index = 0; index < depths.size(); ++index)
    {
        keyIsLowPlatform[static_cast<std::size_t>(index)] = depths[index] >= depthMidpoint;
        if (index == 0)
        {
            keyPointTypes[static_cast<std::size_t>(index)] = PointType::Start;
        }
        else if (index == depths.size() - 1)
        {
            keyPointTypes[static_cast<std::size_t>(index)] = PointType::End;
        }
        else
        {
            keyPointTypes[static_cast<std::size_t>(index)] =
                keyIsLowPlatform[static_cast<std::size_t>(index)]
                    ? PointType::InnerCorner
                    : PointType::OuterCorner;
        }
    }

    segmentKinds.reserve(static_cast<int>(keyPointPositions.size()) - 1);
    for (std::size_t index = 0; index + 1 < keyPointPositions.size(); ++index)
    {
        segmentKinds.push_back(SegmentKindFromDepthSide(
            keyIsLowPlatform[index],
            keyIsLowPlatform[index + 1]));
    }

    if (appendLog)
    {
        appendLog(QString("按测量枪姿重判焊道段属性：枪尖方向=(%1,%2,%3)，深度轴=(%4,%5,%6)，深度范围=%7 mm，远侧=低平台，近侧=高平台。")
            .arg(gunAxis.x(), 0, 'f', 3)
            .arg(gunAxis.y(), 0, 'f', 3)
            .arg(gunAxis.z(), 0, 'f', 3)
            .arg(depthAxis.x(), 0, 'f', 3)
            .arg(depthAxis.y(), 0, 'f', 3)
            .arg(depthAxis.z(), 0, 'f', 3)
            .arg(depthRange, 0, 'f', 3));
    }

    return true;
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

QString BuildFinalSampledWeldPosePath(const QString& poseFilePath)
{
    const QFileInfo poseInfo(QDir::fromNativeSeparators(poseFilePath));
    const QString baseName = poseInfo.completeBaseName().isEmpty()
        ? QStringLiteral("WeldPose")
        : poseInfo.completeBaseName();
    return QDir::toNativeSeparators(poseInfo.dir().filePath(baseName + "_FinalSampled.txt"));
}

bool SaveWeldPoseFileRecords(
    const QString& path,
    const QVector<WeldPoseFileRecord>& records,
    QString& error)
{
    error.clear();
    if (records.isEmpty())
    {
        error = "抽样后没有可保存的焊接姿态点。";
        return false;
    }

    const QFileInfo fileInfo(QDir::fromNativeSeparators(path));
    const QDir parentDir = fileInfo.dir();
    if (!parentDir.exists() && !QDir().mkpath(parentDir.absolutePath()))
    {
        error = QString("创建最终抽样轨迹目录失败：%1").arg(parentDir.absolutePath());
        return false;
    }

    QFile file(fileInfo.absoluteFilePath());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        error = QString("保存最终抽样轨迹文件失败：%1").arg(fileInfo.absoluteFilePath());
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    for (const WeldPoseFileRecord& record : records)
    {
        stream << BuildWeldPoseFileRecordLine(record) << "\n";
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

Eigen::Vector3d CanonicalHorizontalWeldAxis(Eigen::Vector3d direction)
{
    direction = HorizontalUnitOrZero(direction);
    if (direction.head<2>().norm() <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }

    const double absX = std::abs(direction.x());
    const double absY = std::abs(direction.y());
    const bool reverse = absY >= absX
        ? direction.y() < 0.0
        : direction.x() < 0.0;
    return reverse ? -direction : direction;
}

Eigen::Vector3d ResolveOverallHorizontalWeldDirection(
    const QVector<Eigen::Vector3d>& points)
{
    if (points.size() < 2)
    {
        return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d startToEnd = HorizontalUnitOrZero(points.back() - points.front());
    if (startToEnd.head<2>().norm() > 1e-9)
    {
        return CanonicalHorizontalWeldAxis(startToEnd);
    }

    double longestSegmentLength = 0.0;
    Eigen::Vector3d longestDirection = Eigen::Vector3d::Zero();
    for (int index = 1; index < points.size(); ++index)
    {
        Eigen::Vector3d segment = points[index] - points[index - 1];
        segment.z() = 0.0;
        const double length = segment.head<2>().norm();
        if (length > longestSegmentLength)
        {
            longestSegmentLength = length;
            longestDirection = segment / length;
        }
    }
    return CanonicalHorizontalWeldAxis(longestDirection);
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

T_ROBOT_COORS BuildWeldPoseCoors(const WeldPoseFileRecord& record)
{
    return T_ROBOT_COORS(
        record.point.x(),
        record.point.y(),
        record.point.z(),
        record.rx,
        record.ry,
        record.rz,
        record.bx,
        record.by,
        record.bz);
}

QString FinalWeldTrajectoryRecordTag(const WeldPoseFileRecord& record)
{
    return (record.pointType + "|" + record.segmentKind).trimmed().toLower();
}

QString FinalWeldTrajectorySegmentKey(const WeldPoseFileRecord& record)
{
    QString key = NormalizeSeamCompSegmentKind(record.segmentKind).trimmed().toLower();
    if (key.isEmpty())
    {
        key = record.segmentKind.trimmed().toLower();
    }
    return key;
}

bool IsFinalWeldTrajectoryAnchor(
    const QVector<WeldPoseFileRecord>& records,
    int index)
{
    if (index <= 0 || index >= records.size() - 1)
    {
        return true;
    }

    const QString pointType = records[index].pointType.trimmed().toLower();
    if (pointType == "start"
        || pointType == "end"
        || (pointType.contains("corner") && !pointType.contains("_arc")))
    {
        return true;
    }

    const QString previousSegment = FinalWeldTrajectorySegmentKey(records[index - 1]);
    const QString currentSegment = FinalWeldTrajectorySegmentKey(records[index]);
    const QString nextSegment = FinalWeldTrajectorySegmentKey(records[index + 1]);
    if (currentSegment.compare(previousSegment, Qt::CaseInsensitive) != 0
        || currentSegment.compare(nextSegment, Qt::CaseInsensitive) != 0)
    {
        return true;
    }

    const QString previousTag = FinalWeldTrajectoryRecordTag(records[index - 1]);
    const QString currentTag = FinalWeldTrajectoryRecordTag(records[index]);
    const QString nextTag = FinalWeldTrajectoryRecordTag(records[index + 1]);
    const auto executionModeTag = [](const QString& tag) -> QString
    {
        const bool arc = tag.contains("_arc") || tag.contains(QStringLiteral("圆弧"));
        const bool transition = tag.contains("transition") || tag.contains(QStringLiteral("过渡"));
        if (arc)
        {
            return "arc";
        }
        if (transition)
        {
            return "transition";
        }
        return "normal";
    };

    const QString previousMode = executionModeTag(previousTag);
    const QString currentMode = executionModeTag(currentTag);
    const QString nextMode = executionModeTag(nextTag);
    return currentMode.compare(previousMode, Qt::CaseInsensitive) != 0
        || currentMode.compare(nextMode, Qt::CaseInsensitive) != 0;
}

QVector<WeldPoseFileRecord> SampleFinalWeldTrajectoryRecords(
    const QVector<WeldPoseFileRecord>& records,
    double sampleStepMm)
{
    if (records.size() <= 2 || !std::isfinite(sampleStepMm) || sampleStepMm <= 0.0)
    {
        return records;
    }

    // 任意相邻输出点的沿线间距不得小于设定点间距：过密的点会干扰摆动动作。
    // 拐点/圆弧/过渡区域的边界锚彼此相距常低于设定值，按"锚点优先于普通采样点、
    // 锚点之间仍不足间距时丢弃后到者、终点最优先"的规则去密，圆弧段只留间距达标的关键点。
    const double minGapMm = sampleStepMm;

    struct KeptPoint
    {
        int index = 0;
        bool anchor = false;
        double arcLengthMm = 0.0;
    };
    QVector<KeptPoint> kept;
    kept.reserve(records.size());
    kept.push_back({ 0, true, 0.0 });

    double arcLengthMm = 0.0;
    for (int index = 1; index < records.size() - 1; ++index)
    {
        const double segmentLengthMm = (records[index].point - records[index - 1].point).norm();
        if (std::isfinite(segmentLengthMm) && segmentLengthMm > 1e-6)
        {
            arcLengthMm += segmentLengthMm;
        }

        if (IsFinalWeldTrajectoryAnchor(records, index))
        {
            // 锚点优先：回溯挤掉与其间距不足的普通采样点。
            while (kept.size() > 1
                && !kept.back().anchor
                && arcLengthMm - kept.back().arcLengthMm < minGapMm)
            {
                kept.pop_back();
            }
            if (arcLengthMm - kept.back().arcLengthMm >= minGapMm)
            {
                kept.push_back({ index, true, arcLengthMm });
            }
            // 与前一锚点仍不足间距时丢弃当前锚，保证输出不含过密点。
        }
        else if (arcLengthMm - kept.back().arcLengthMm >= minGapMm)
        {
            kept.push_back({ index, false, arcLengthMm });
        }
    }

    // 终点必留且最优先：回溯挤掉与终点间距不足的点（首点除外）。
    const double tailSegmentMm =
        (records[records.size() - 1].point - records[records.size() - 2].point).norm();
    if (std::isfinite(tailSegmentMm) && tailSegmentMm > 1e-6)
    {
        arcLengthMm += tailSegmentMm;
    }
    while (kept.size() > 1 && arcLengthMm - kept.back().arcLengthMm < minGapMm)
    {
        kept.pop_back();
    }

    QVector<WeldPoseFileRecord> sampled;
    sampled.reserve(kept.size() + 1);
    for (const KeptPoint& keep : kept)
    {
        sampled.push_back(records[keep.index]);
    }
    constexpr double kDuplicateDistanceMm = 1e-6;
    if ((sampled.back().point - records.back().point).norm() <= kDuplicateDistanceMm)
    {
        sampled.back() = records.back();
    }
    else
    {
        sampled.push_back(records.back());
    }
    for (int index = 0; index < sampled.size(); ++index)
    {
        sampled[index].weldIndex = index + 1;
    }
    return sampled;
}

bool BuildWeldPoseMoveInfos(
    const QVector<WeldPoseFileRecord>& records,
    double linearSpeedMmPerSec,
    std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    QString& error,
    const WeldPosePreset* preset = nullptr,
    double finalTrajectorySampleStepMm = DEFAULT_FINAL_WELD_TRAJECTORY_SAMPLE_STEP_MM,
    double transitionLinearSpeed = 0.0,
    bool enableWeldProcess = false,
    QVector<WeldPoseFileRecord>* executionRecordsOut = nullptr)
{
    moveInfos.clear();
    // 工艺里设置了点间距(>0)时优先用工艺的，否则用测量参数页传入值。
    double effectiveSampleStepMm = finalTrajectorySampleStepMm;
    if (preset != nullptr && preset->finalWeldStepFromProcessMm > 0.0)
    {
        effectiveSampleStepMm = preset->finalWeldStepFromProcessMm;
    }
    const QVector<WeldPoseFileRecord> executionRecords =
        SampleFinalWeldTrajectoryRecords(records, NormalizeFinalWeldTrajectorySampleStepMm(effectiveSampleStepMm));
    if (executionRecordsOut != nullptr)
    {
        *executionRecordsOut = executionRecords;
    }
    moveInfos.reserve(static_cast<size_t>(executionRecords.size()));

    const bool useWeldProcess = enableWeldProcess
        && preset != nullptr
        && preset->weldProcessLoaded;
    if (enableWeldProcess
        && preset != nullptr
        && !preset->weldProcessLoaded
        && !preset->weldProcessLoadError.isEmpty())
    {
        error = preset->weldProcessLoadError;
        return false;
    }
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
    }
    if (externalAxisPointCount > 0)
    {
        error = QString("焊接姿态文件包含 %1 个外部轴点位，但当前多点 TP 下发只支持 GP1 六轴点位，请先确认 BX/BY/BZ 是否应为 0。")
            .arg(externalAxisPointCount);
        return false;
    }

    for (const WeldPoseFileRecord& record : executionRecords)
    {
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
        moveInfo.tCoord = BuildWeldPoseCoors(record);
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
            moveInfo.nArcMode = preset->arcMode;
            moveInfo.bHasWeaveParam = preset->weaveEnabled;
            if (moveInfo.bHasWeaveParam)
            {
                moveInfo.tWeaveParam = preset->weaveParam;
            }
            moveInfo.bHasTrackParam = preset->trackEnabled;
            if (moveInfo.bHasTrackParam)
            {
                moveInfo.tTrackParam = preset->trackParam;
            }
        }
        moveInfos.push_back(moveInfo);
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

void TrimSharpWeldArcExitPoints(
    QVector<WeldPoseFileRecord>& records,
    double maxExitAngleRad,
    double maxTrimDistanceMm)
{
    for (int index = 1; index < records.size(); ++index)
    {
        if (IsWeldPoseArcRecord(records[index])
            || !IsWeldPoseArcRecord(records[index - 1]))
        {
            continue;
        }

        int trimmedCount = 0;
        while (index >= 2 && trimmedCount < 3)
        {
            const int previousIndex = index - 1;
            if (!IsWeldPoseArcRecord(records[previousIndex]))
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

            if (outgoingDistanceMm > 0.5
                && WeldPoseTurnAngleRad(incoming, outgoing) <= maxExitAngleRad)
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
    constexpr double kMaxCornerAngleRad = 178.0 * kPi / 180.0;
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

// 独立调试预览(不参与主流程，不影响任何实际焊接文件)：在几何分析得到分类轨迹(拐点+2mm点)后，
// 用与主流程相同的圆弧过渡算法(焊接工艺圆角半径 cornerArcRadiusMm)对拐角做圆弧过渡，仅生成一份
// CloudCompare 点云：绿=圆弧过渡段、灰=直线段。叠加原始 PreciseLaserPoint.txt 即可核对生成的圆弧
// 是否贴合原本焊道。注意：主流程真正的圆弧过渡仍在补偿之后(_SeamComp)，此处只为提前可视化核对。
std::vector<QString> BuildArcTransitionPreviewCloudLines(
    const RobotCalculation::LowerWeldClassificationResult& classification,
    const WeldPosePreset& preset)
{
    QVector<WeldPoseFileRecord> records;
    records.reserve(classification.points.size());
    int weldIndex = 1;
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : classification.points)
    {
        WeldPoseFileRecord record;
        record.weldIndex = weldIndex++;
        record.rawIndex = point.index;
        record.point = point.point;
        record.pointType = RobotCalculation::LowerWeldPointTypeName(point.type);
        record.segmentKind = point.segmentKindAfter;
        records.push_back(record);
    }

    ApplyCornerArcTransitionToWeldPoseRecords(preset, records);

    std::vector<QString> cloud;
    cloud.push_back(QStringLiteral("// X Y Z R G B  绿=圆弧过渡段 灰=直线段 (叠加 PreciseLaserPoint.txt 对比焊道)"));
    for (const WeldPoseFileRecord& record : records)
    {
        const bool isArc = record.pointType.contains(QStringLiteral("arc"))
            || record.segmentKind.contains(QStringLiteral("_arc"));
        const QString rgb = isArc ? QStringLiteral("0 255 0") : QStringLiteral("120 120 120");
        cloud.push_back(QString("%1 %2 %3 %4")
            .arg(record.point.x(), 0, 'f', 6)
            .arg(record.point.y(), 0, 'f', 6)
            .arg(record.point.z(), 0, 'f', 6)
            .arg(rgb));
    }
    return cloud;
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

int SmoothRemainingUnroundedWeldCorners(
    QVector<WeldPoseFileRecord>& records,
    double maxStepMm)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinSmoothAngleRad = 35.0 * kPi / 180.0;
    if (records.size() < 3)
    {
        return 0;
    }

    const double safeStepMm = std::clamp(maxStepMm, 0.5, 5.0);
    const auto hasNearbyTransitionKind =
        [&records](int centerIndex, int radius) -> bool
    {
        const int beginIndex = std::max(0, centerIndex - radius);
        const int endIndex = std::min(
            static_cast<int>(records.size()) - 1,
            centerIndex + radius);
        for (int index = beginIndex; index <= endIndex; ++index)
        {
            if (records[index].segmentKind.contains(
                    QStringLiteral("_transition"),
                    Qt::CaseInsensitive))
            {
                return true;
            }
        }
        return false;
    };

    int smoothedCount = 0;
    for (int index = 1; index + 1 < records.size(); ++index)
    {
        const bool isMarkedCorner = IsWeldCornerPointType(records[index].pointType);
        const bool isResidualTransitionCorner =
            !isMarkedCorner
            && records[index].pointType.compare(QStringLiteral("normal"), Qt::CaseInsensitive) == 0
            && hasNearbyTransitionKind(index, 2);
        if (IsWeldPoseArcRecord(records[index])
            || (!isMarkedCorner && !isResidualTransitionCorner))
        {
            continue;
        }

        const Eigen::Vector3d incoming = records[index].point - records[index - 1].point;
        const Eigen::Vector3d outgoing = records[index + 1].point - records[index].point;
        if (WeldPoseTurnAngleRad(incoming, outgoing) < kMinSmoothAngleRad)
        {
            continue;
        }

        const WeldPoseFileRecord corner = records[index];
        const WeldPoseFileRecord before = records[index - 1];
        const WeldPoseFileRecord after = records[index + 1];
        const double distanceMm = (after.point - before.point).norm();
        if (!std::isfinite(distanceMm) || distanceMm <= 1e-6 || distanceMm > safeStepMm * 5.0)
        {
            continue;
        }

        const int segmentCount = std::max(
            2,
            static_cast<int>(std::ceil(distanceMm / safeStepMm)));
        QVector<WeldPoseFileRecord> bridgeRecords;
        bridgeRecords.reserve(std::max(0, segmentCount - 1));
        for (int segmentIndex = 1; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double ratio = static_cast<double>(segmentIndex)
                / static_cast<double>(segmentCount);
            WeldPoseFileRecord bridgeRecord = InterpolateWeldPoseRecord(before, after, ratio);
            bridgeRecord.rawIndex = corner.rawIndex;
            bridgeRecord.pointType = isMarkedCorner
                ? (corner.pointType + "_arc")
                : QStringLiteral("normal_arc");
            bridgeRecord.segmentKind = WeldArcSegmentKind(corner.segmentKind);
            bridgeRecords.push_back(bridgeRecord);
        }

        records.erase(records.begin() + index);
        for (int bridgeIndex = 0; bridgeIndex < bridgeRecords.size(); ++bridgeIndex)
        {
            records.insert(index + bridgeIndex, bridgeRecords[bridgeIndex]);
        }
        index += std::max(0, static_cast<int>(bridgeRecords.size()) - 1);
        ++smoothedCount;
    }

    if (smoothedCount > 0)
    {
        RenumberWeldPoseRecords(records);
    }
    return smoothedCount;
}

struct WeldCornerRestoreStats
{
    int missingCornerCount = 0;
    int restoredCornerCount = 0;
    int adjustedCornerCount = 0;
    int skippedCornerCount = 0;
};

QString WeldCornerTypeKey(const QString& pointType)
{
    const QString normalized = pointType.trimmed().toLower();
    if (normalized.contains("inner") || normalized.contains(QStringLiteral("内")))
    {
        return QStringLiteral("inner");
    }
    if (normalized.contains("outer") || normalized.contains(QStringLiteral("外")))
    {
        return QStringLiteral("outer");
    }
    if (IsWeldCornerPointType(pointType))
    {
        return QStringLiteral("corner");
    }
    return QString();
}

bool IsSameWeldCornerType(const QString& lhs, const QString& rhs)
{
    const QString lhsKey = WeldCornerTypeKey(lhs);
    const QString rhsKey = WeldCornerTypeKey(rhs);
    return !lhsKey.isEmpty() && lhsKey == rhsKey;
}

int FindMatchingWeldCornerRecordIndex(
    const QVector<WeldPoseFileRecord>& records,
    const WeldPoseFileRecord& corner)
{
    for (int index = 0; index < records.size(); ++index)
    {
        const WeldPoseFileRecord& record = records[index];
        if (record.rawIndex == corner.rawIndex
            && IsSameWeldCornerType(record.pointType, corner.pointType))
        {
            return index;
        }
    }
    return -1;
}

int FindWeldCornerInsertionIndex(
    const QVector<WeldPoseFileRecord>& records,
    const WeldPoseFileRecord& corner)
{
    for (int index = 0; index < records.size(); ++index)
    {
        if (records[index].rawIndex >= corner.rawIndex)
        {
            return index;
        }
    }
    return records.size();
}

QVector<int> CollectSameKindLineWindowBefore(
    const QVector<WeldPoseFileRecord>& records,
    int endIndex,
    int maxPointCount)
{
    QVector<int> indices;
    if (endIndex < 0 || endIndex >= records.size())
    {
        return indices;
    }

    const QString targetKind = NormalizeSeamCompSegmentKind(records[endIndex].segmentKind)
        .trimmed()
        .toLower();
    for (int index = endIndex; index >= 0 && indices.size() < maxPointCount; --index)
    {
        const QString kind = NormalizeSeamCompSegmentKind(records[index].segmentKind)
            .trimmed()
            .toLower();
        if (!targetKind.isEmpty() && kind != targetKind && indices.size() >= 2)
        {
            break;
        }
        indices.push_front(index);
    }
    return indices;
}

QVector<int> CollectSameKindLineWindowAfter(
    const QVector<WeldPoseFileRecord>& records,
    int beginIndex,
    int maxPointCount)
{
    QVector<int> indices;
    if (beginIndex < 0 || beginIndex >= records.size())
    {
        return indices;
    }

    const QString targetKind = NormalizeSeamCompSegmentKind(records[beginIndex].segmentKind)
        .trimmed()
        .toLower();
    for (int index = beginIndex; index < records.size() && indices.size() < maxPointCount; ++index)
    {
        const QString kind = NormalizeSeamCompSegmentKind(records[index].segmentKind)
            .trimmed()
            .toLower();
        if (!targetKind.isEmpty() && kind != targetKind && indices.size() >= 2)
        {
            break;
        }
        indices.push_back(index);
    }
    return indices;
}

bool TryFitWeldPoseLine2D(
    const QVector<WeldPoseFileRecord>& records,
    const QVector<int>& indices,
    Eigen::Vector2d& point,
    Eigen::Vector2d& direction)
{
    if (indices.size() < 2)
    {
        return false;
    }

    point = Eigen::Vector2d::Zero();
    int validCount = 0;
    for (const int index : indices)
    {
        if (index < 0 || index >= records.size())
        {
            continue;
        }
        point += Eigen::Vector2d(records[index].point.x(), records[index].point.y());
        ++validCount;
    }
    if (validCount < 2)
    {
        return false;
    }
    point /= static_cast<double>(validCount);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const int index : indices)
    {
        if (index < 0 || index >= records.size())
        {
            continue;
        }
        const Eigen::Vector2d delta =
            Eigen::Vector2d(records[index].point.x(), records[index].point.y()) - point;
        covariance += delta * delta.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()(1) <= 1e-8)
    {
        return false;
    }

    direction = solver.eigenvectors().col(1).normalized();
    return direction.norm() > 1e-9;
}

double Cross2D(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
{
    return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

bool TryIntersectWeldPoseLines2D(
    const Eigen::Vector2d& firstPoint,
    const Eigen::Vector2d& firstDirection,
    const Eigen::Vector2d& secondPoint,
    const Eigen::Vector2d& secondDirection,
    Eigen::Vector2d& intersection)
{
    const double denominator = Cross2D(firstDirection, secondDirection);
    if (std::abs(denominator) <= 1e-8)
    {
        return false;
    }

    const Eigen::Vector2d delta = secondPoint - firstPoint;
    const double firstRatio = Cross2D(delta, secondDirection) / denominator;
    intersection = firstPoint + firstDirection * firstRatio;
    return intersection.allFinite();
}

bool TryBuildLineIntersectionWeldCorner(
    const WeldPoseFileRecord& corner,
    const QVector<WeldPoseFileRecord>& records,
    int beforeEndIndex,
    int afterBeginIndex,
    WeldPoseFileRecord& restoredCorner)
{
    if (records.size() < 4)
    {
        return false;
    }

    if (beforeEndIndex < 0
        || beforeEndIndex >= records.size()
        || afterBeginIndex < 0
        || afterBeginIndex >= records.size()
        || beforeEndIndex >= afterBeginIndex)
    {
        return false;
    }

    const QVector<int> beforeIndices = CollectSameKindLineWindowBefore(
        records,
        beforeEndIndex,
        6);
    const QVector<int> afterIndices = CollectSameKindLineWindowAfter(
        records,
        afterBeginIndex,
        6);

    Eigen::Vector2d firstPoint;
    Eigen::Vector2d firstDirection;
    Eigen::Vector2d secondPoint;
    Eigen::Vector2d secondDirection;
    if (!TryFitWeldPoseLine2D(records, beforeIndices, firstPoint, firstDirection)
        || !TryFitWeldPoseLine2D(records, afterIndices, secondPoint, secondDirection))
    {
        return false;
    }

    Eigen::Vector2d intersection;
    if (!TryIntersectWeldPoseLines2D(
            firstPoint,
            firstDirection,
            secondPoint,
            secondDirection,
            intersection))
    {
        return false;
    }

    const WeldPoseFileRecord& before = records[beforeEndIndex];
    const WeldPoseFileRecord& after = records[afterBeginIndex];
    const Eigen::Vector2d beforePoint(before.point.x(), before.point.y());
    const Eigen::Vector2d afterPoint(after.point.x(), after.point.y());
    const Eigen::Vector2d span = afterPoint - beforePoint;
    const double spanLength = span.norm();
    const double distanceToBefore = (intersection - beforePoint).norm();
    const double distanceToAfter = (intersection - afterPoint).norm();
    const double maxAllowedDistance = std::max(20.0, spanLength * 6.0);
    const Eigen::Vector2d originalCornerPoint(corner.point.x(), corner.point.y());
    if (distanceToBefore > maxAllowedDistance
        || distanceToAfter > maxAllowedDistance
        || (intersection - originalCornerPoint).norm() > std::max(25.0, spanLength * 8.0))
    {
        return false;
    }

    double ratio = 0.5;
    if (span.squaredNorm() > 1e-9)
    {
        ratio = std::clamp((intersection - beforePoint).dot(span) / span.squaredNorm(), 0.0, 1.0);
    }

    restoredCorner = InterpolateWeldPoseRecord(before, after, ratio);
    restoredCorner.rawIndex = corner.rawIndex;
    restoredCorner.point.x() = intersection.x();
    restoredCorner.point.y() = intersection.y();
    restoredCorner.pointType = corner.pointType;
    restoredCorner.segmentKind = corner.segmentKind;
    if ((restoredCorner.point - before.point).norm() <= 1e-6
        || (restoredCorner.point - after.point).norm() <= 1e-6)
    {
        return false;
    }

    return true;
}

bool TryInsertRestoredWeldCorner(
    const WeldPoseFileRecord& corner,
    QVector<WeldPoseFileRecord>& records)
{
    const int insertionIndex = FindWeldCornerInsertionIndex(records, corner);
    if (insertionIndex <= 0 || insertionIndex >= records.size())
    {
        return false;
    }

    WeldPoseFileRecord restoredCorner;
    if (!TryBuildLineIntersectionWeldCorner(
            corner,
            records,
            insertionIndex - 1,
            insertionIndex,
            restoredCorner))
    {
        return false;
    }

    records.insert(insertionIndex, restoredCorner);
    return true;
}

bool TryAdjustExistingWeldCornerByIntersection(
    const WeldPoseFileRecord& sourceCorner,
    int cornerIndex,
    QVector<WeldPoseFileRecord>& records)
{
    if (cornerIndex <= 0 || cornerIndex + 1 >= records.size())
    {
        return false;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double currentTurnRad = WeldPoseTurnAngleRad(
        records[cornerIndex].point - records[cornerIndex - 1].point,
        records[cornerIndex + 1].point - records[cornerIndex].point);
    if (currentTurnRad < 35.0 * kPi / 180.0)
    {
        return false;
    }

    WeldPoseFileRecord restoredCorner;
    if (!TryBuildLineIntersectionWeldCorner(
            sourceCorner,
            records,
            cornerIndex - 1,
            cornerIndex + 1,
            restoredCorner))
    {
        return false;
    }

    if ((restoredCorner.point - records[cornerIndex].point).norm() <= 1e-6)
    {
        return false;
    }

    records[cornerIndex] = restoredCorner;
    return true;
}

WeldCornerRestoreStats RestoreTrimmedWeldCornersByLineIntersection(
    const QVector<WeldPoseFileRecord>& recordsBeforeTrim,
    QVector<WeldPoseFileRecord>& records)
{
    WeldCornerRestoreStats stats;
    if (recordsBeforeTrim.isEmpty() || records.size() < 4)
    {
        return stats;
    }

    int minRawIndex = std::numeric_limits<int>::max();
    int maxRawIndex = std::numeric_limits<int>::min();
    for (const WeldPoseFileRecord& record : records)
    {
        minRawIndex = std::min(minRawIndex, record.rawIndex);
        maxRawIndex = std::max(maxRawIndex, record.rawIndex);
    }

    for (const WeldPoseFileRecord& corner : recordsBeforeTrim)
    {
        if (!IsWeldCornerPointType(corner.pointType)
            || corner.rawIndex < minRawIndex
            || corner.rawIndex > maxRawIndex)
        {
            continue;
        }

        const int existingCornerIndex = FindMatchingWeldCornerRecordIndex(records, corner);
        if (existingCornerIndex >= 0)
        {
            if (TryAdjustExistingWeldCornerByIntersection(corner, existingCornerIndex, records))
            {
                ++stats.adjustedCornerCount;
                RenumberWeldPoseRecords(records);
            }
            continue;
        }

        ++stats.missingCornerCount;
        if (TryInsertRestoredWeldCorner(corner, records))
        {
            ++stats.restoredCornerCount;
            RenumberWeldPoseRecords(records);
        }
        else
        {
            ++stats.skippedCornerCount;
        }
    }
    return stats;
}

struct PoseCompSegmentRange
{
    int begin = -1;
    int end = -1;
    QString kind;
};

struct PoseCompJunctionApplyStats
{
    int adjustedJunctionCount = 0;
    int removedPointCount = 0;
};

QVector<PoseCompSegmentRange> CollectPoseCompSegmentRanges(
    const QVector<WeldPoseFileRecord>& records)
{
    QVector<PoseCompSegmentRange> ranges;
    if (records.isEmpty())
    {
        return ranges;
    }

    auto normalizedKindAt = [&records](int index) -> QString
    {
        QString kind = NormalizeSeamCompSegmentKind(records[index].segmentKind)
            .trimmed()
            .toLower();
        if (kind.isEmpty())
        {
            kind = records[index].segmentKind.trimmed().toLower();
        }
        return kind;
    };

    PoseCompSegmentRange current;
    current.begin = 0;
    current.end = 0;
    current.kind = normalizedKindAt(0);
    for (int index = 1; index < records.size(); ++index)
    {
        const QString kind = normalizedKindAt(index);
        if (kind.compare(current.kind, Qt::CaseInsensitive) == 0)
        {
            current.end = index;
            continue;
        }

        ranges.push_back(current);
        current.begin = index;
        current.end = index;
        current.kind = kind;
    }
    ranges.push_back(current);
    return ranges;
}

bool ShouldRebuildPoseCompJunction(const QString& leftKind, const QString& rightKind)
{
    if (leftKind.compare(rightKind, Qt::CaseInsensitive) == 0)
    {
        return false;
    }

    const bool leftPlatform = IsPlatformSegmentKind(leftKind);
    const bool rightPlatform = IsPlatformSegmentKind(rightKind);
    const bool leftSlope = IsSlopeSegmentKind(leftKind);
    const bool rightSlope = IsSlopeSegmentKind(rightKind);
    return (leftPlatform && rightSlope) || (leftSlope && rightPlatform);
}

Eigen::Vector2d WeldPosePoint2D(const WeldPoseFileRecord& record)
{
    return Eigen::Vector2d(record.point.x(), record.point.y());
}

Eigen::Vector2d SegmentTravelDirection2D(
    const QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& range,
    const Eigen::Vector2d& fallbackDirection)
{
    if (range.begin >= 0
        && range.end >= range.begin
        && range.end < records.size())
    {
        const Eigen::Vector2d delta =
            WeldPosePoint2D(records[range.end]) - WeldPosePoint2D(records[range.begin]);
        if (delta.norm() > 1e-9)
        {
            return delta.normalized();
        }
    }

    if (fallbackDirection.norm() > 1e-9)
    {
        return fallbackDirection.normalized();
    }
    return Eigen::Vector2d(1.0, 0.0);
}

double SegmentProjection(
    const QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& range,
    const Eigen::Vector2d& travelDirection,
    const Eigen::Vector2d& point)
{
    if (range.begin < 0 || range.begin >= records.size())
    {
        return 0.0;
    }
    return (point - WeldPosePoint2D(records[range.begin])).dot(travelDirection);
}

WeldPoseFileRecord BuildPoseCompJunctionRecord(
    const QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& ownerRange,
    const Eigen::Vector2d& intersection)
{
    const WeldPoseFileRecord& begin = records[ownerRange.begin];
    const WeldPoseFileRecord& end = records[ownerRange.end];
    const Eigen::Vector2d beginPoint = WeldPosePoint2D(begin);
    const Eigen::Vector2d endPoint = WeldPosePoint2D(end);
    const Eigen::Vector2d span = endPoint - beginPoint;

    double ratio = 0.0;
    if (span.squaredNorm() > 1e-9)
    {
        ratio = (intersection - beginPoint).dot(span) / span.squaredNorm();
    }
    const double clampedRatio = std::clamp(ratio, 0.0, 1.0);
    WeldPoseFileRecord record = InterpolateWeldPoseRecord(begin, end, clampedRatio);
    record.point.x() = intersection.x();
    record.point.y() = intersection.y();
    if (std::isfinite(ratio) && std::abs(ratio) <= 3.0)
    {
        record.point.z() = begin.point.z() + (end.point.z() - begin.point.z()) * ratio;
    }
    record.pointType = begin.pointType;
    record.segmentKind = begin.segmentKind;
    return record;
}

bool TryApplyPoseCompJunctionIntersection(
    QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& leftRange,
    const PoseCompSegmentRange& rightRange,
    PoseCompJunctionApplyStats& stats)
{
    constexpr double kMoveEpsilonMm = 1e-5;
    constexpr double kTrimEpsilonMm = 1e-4;
    if (!ShouldRebuildPoseCompJunction(leftRange.kind, rightRange.kind)
        || leftRange.begin < 0
        || leftRange.end < leftRange.begin
        || rightRange.begin <= leftRange.end
        || rightRange.end < rightRange.begin
        || rightRange.end >= records.size())
    {
        return false;
    }

    const QVector<int> leftFitIndices = CollectSameKindLineWindowBefore(
        records,
        leftRange.end,
        8);
    const QVector<int> rightFitIndices = CollectSameKindLineWindowAfter(
        records,
        rightRange.begin,
        8);

    Eigen::Vector2d leftPoint;
    Eigen::Vector2d leftDirection;
    Eigen::Vector2d rightPoint;
    Eigen::Vector2d rightDirection;
    if (!TryFitWeldPoseLine2D(records, leftFitIndices, leftPoint, leftDirection)
        || !TryFitWeldPoseLine2D(records, rightFitIndices, rightPoint, rightDirection))
    {
        return false;
    }

    Eigen::Vector2d intersection;
    if (!TryIntersectWeldPoseLines2D(
            leftPoint,
            leftDirection,
            rightPoint,
            rightDirection,
            intersection))
    {
        return false;
    }

    const Eigen::Vector2d leftEnd = WeldPosePoint2D(records[leftRange.end]);
    const Eigen::Vector2d rightBegin = WeldPosePoint2D(records[rightRange.begin]);
    const double boundaryGap = (rightBegin - leftEnd).norm();
    const double leftLength = (WeldPosePoint2D(records[leftRange.end])
        - WeldPosePoint2D(records[leftRange.begin])).norm();
    const double rightLength = (WeldPosePoint2D(records[rightRange.end])
        - WeldPosePoint2D(records[rightRange.begin])).norm();
    const double maxAllowedDistance = std::max(
        20.0,
        std::max(boundaryGap * 12.0, std::min(leftLength, rightLength) * 2.0));
    if ((intersection - leftEnd).norm() > maxAllowedDistance
        || (intersection - rightBegin).norm() > maxAllowedDistance)
    {
        return false;
    }

    QVector<int> removeIndices;
    const Eigen::Vector2d leftTravelDirection =
        SegmentTravelDirection2D(records, leftRange, leftDirection);
    const double leftIntersectionProjection =
        SegmentProjection(records, leftRange, leftTravelDirection, intersection);
    for (int index = leftRange.end; index > leftRange.begin; --index)
    {
        const double projection = SegmentProjection(
            records,
            leftRange,
            leftTravelDirection,
            WeldPosePoint2D(records[index]));
        if (projection <= leftIntersectionProjection + kTrimEpsilonMm)
        {
            break;
        }
        removeIndices.push_back(index);
    }

    const Eigen::Vector2d rightTravelDirection =
        SegmentTravelDirection2D(records, rightRange, rightDirection);
    const double rightIntersectionProjection =
        SegmentProjection(records, rightRange, rightTravelDirection, intersection);
    for (int index = rightRange.begin + 1; index < rightRange.end; ++index)
    {
        const double projection = SegmentProjection(
            records,
            rightRange,
            rightTravelDirection,
            WeldPosePoint2D(records[index]));
        if (projection >= rightIntersectionProjection - kTrimEpsilonMm)
        {
            break;
        }
        removeIndices.push_back(index);
    }

    WeldPoseFileRecord junctionRecord = BuildPoseCompJunctionRecord(
        records,
        rightRange,
        intersection);
    const bool moved =
        (records[rightRange.begin].point - junctionRecord.point).norm() > kMoveEpsilonMm;
    if (!moved && removeIndices.isEmpty())
    {
        return false;
    }

    records[rightRange.begin] = junctionRecord;
    std::sort(removeIndices.begin(), removeIndices.end());
    removeIndices.erase(std::unique(removeIndices.begin(), removeIndices.end()), removeIndices.end());
    for (int index = removeIndices.size() - 1; index >= 0; --index)
    {
        records.removeAt(removeIndices[index]);
    }

    ++stats.adjustedJunctionCount;
    stats.removedPointCount += removeIndices.size();
    return true;
}

PoseCompJunctionApplyStats ApplyPoseCompSegmentJunctionIntersections(
    QVector<WeldPoseFileRecord>& records)
{
    PoseCompJunctionApplyStats stats;
    constexpr int kMaxPassCount = 128;
    for (int pass = 0; pass < kMaxPassCount; ++pass)
    {
        const QVector<PoseCompSegmentRange> ranges = CollectPoseCompSegmentRanges(records);
        bool adjusted = false;
        for (int index = 0; index + 1 < ranges.size(); ++index)
        {
            if (TryApplyPoseCompJunctionIntersection(
                    records,
                    ranges[index],
                    ranges[index + 1],
                    stats))
            {
                adjusted = true;
                break;
            }
        }

        if (!adjusted)
        {
            break;
        }
    }

    if (stats.adjustedJunctionCount > 0)
    {
        RenumberWeldPoseRecords(records);
    }
    return stats;
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
    const Eigen::Vector3d overallSeamDirection =
        ResolveOverallHorizontalWeldDirection(basePoints);
    const Eigen::Vector3d overallGunDirection = HorizontalUnitOrZero(
        Eigen::Vector3d::UnitZ().cross(overallSeamDirection));

    QVector<const WeldPosePreset::SeamCompSlot*> recordSeamCompSlots;
    recordSeamCompSlots.resize(records.size());
    for (int index = 0; index < records.size(); ++index)
    {
        recordSeamCompSlots[index] = FindSeamCompSlotForRecord(preset, records[index]);
    }

    for (int index = 0; index < records.size(); ++index)
    {
        WeldPoseFileRecord& record = records[index];
        const WeldPosePreset::SeamCompSlot* seamCompSlot =
            recordSeamCompSlots[index];
        Eigen::Vector3d horizontalComp = Eigen::Vector3d::Zero();
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

        if (std::abs(seamCompSlot->weldGunDirComp) > 1e-6)
        {
            // 焊道补偿按不随采样顺序翻转的焊道轴计算；同一补偿组时整条轨迹只做刚性平移。
            if (overallGunDirection.head<2>().norm() > 1e-9)
            {
                horizontalComp += overallGunDirection * seamCompSlot->weldGunDirComp;
                ++stats.gunDirAdjustedCount;
            }
        }

        if (std::abs(seamCompSlot->weldSeamDirComp) > 1e-6
            && overallSeamDirection.head<2>().norm() > 1e-9)
        {
            horizontalComp += overallSeamDirection * seamCompSlot->weldSeamDirComp;
            ++stats.seamDirAdjustedCount;
        }

        if (horizontalComp.head<2>().norm() > 1e-9)
        {
            record.point += horizontalComp;
        }
    }

    return stats;
}

// 焊缝补偿平移之后的完整后处理（端点裁剪/自交裁剪/拐点恢复/加密/圆弧过渡/锐角裁剪/平滑/重编号）。
// 管线 ApplyWeldSeamCompToPoseFile 与补偿预览共用，保证预览贴近实际下发轨迹（单一事实源）。
struct SeamCompFinalizeStats
{
    WeldEndpointTrimStats endpointTrim;
    WeldCornerRestoreStats cornerRestore;
    WeldCornerArcApplyStats arc;
    int densifiedPointCount = 0;
    int postArcDensifiedPointCount = 0;
    int finalDensifiedPointCount = 0;
    int smoothedRemainingCornerCount = 0;
    double densifyStepMm = 2.0;
    bool emptyAfterEndpointTrim = false;
    bool emptyAfterSelfIntersection = false;
};

SeamCompFinalizeStats FinalizeSeamCompedWeldPoseRecords(
    const WeldPosePreset& preset,
    const QVector<WeldPoseFileRecord>& recordsBeforeTrim,
    QVector<WeldPoseFileRecord>& records,
    WeldSeamCompApplyStats& compStats)
{
    SeamCompFinalizeStats stats;
    constexpr double kSharpArcAngleRad = 8.0 * 3.14159265358979323846 / 180.0;

    TrimWeldPoseRecordEndpoints(preset, records, stats.endpointTrim);
    if (records.isEmpty())
    {
        stats.emptyAfterEndpointTrim = true;
        return stats;
    }
    TrimWeldPathSelfIntersections(records, compStats);
    if (records.isEmpty())
    {
        stats.emptyAfterSelfIntersection = true;
        return stats;
    }
    stats.cornerRestore = RestoreTrimmedWeldCornersByLineIntersection(recordsBeforeTrim, records);
    stats.densifyStepMm = std::min(2.0, EstimateWeldPoseStepMm(records));
    stats.densifiedPointCount = DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    stats.arc = ApplyCornerArcTransitionToWeldPoseRecords(preset, records);
    stats.postArcDensifiedPointCount = DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    TrimSharpWeldArcEntryPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    TrimSharpWeldArcExitPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    stats.smoothedRemainingCornerCount = SmoothRemainingUnroundedWeldCorners(records, stats.densifyStepMm);
    stats.finalDensifiedPointCount = DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    TrimSharpWeldArcEntryPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    TrimSharpWeldArcExitPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    stats.smoothedRemainingCornerCount += SmoothRemainingUnroundedWeldCorners(records, stats.densifyStepMm);
    stats.finalDensifiedPointCount += DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    RenumberWeldPoseRecords(records);
    return stats;
}

// 姿态补偿的「匹配槽位 + 工具系旋转后叠加」单一事实源：
// 管线（BuildSegmentPoseOutputLines）与补偿预览（MeasureThenWeldService::RecomputeCompPreview）共用，
// 确保界面显示的补偿后焊道与实际下发轨迹一致。
Eigen::Vector3d ApplyPoseCompToPoint(
    const std::vector<WeldPosePreset::PoseCompSlot>& poseCompSlots,
    int poseCompMatchMode,
    double poseMatchMaxErrorDeg,
    int robotType,
    const Eigen::Vector3d& point,
    double rx,
    double ry,
    double rz,
    const QString& segmentKind)
{
    int slotIndex = -1;
    if (NormalizePoseCompMatchMode(poseCompMatchMode) == POSE_COMP_MATCH_BY_SEGMENT_CODE)
    {
        // 剥掉 _transition/_arc 后缀再匹配：管线内传入的 segment.kind 本就无后缀（无影响），
        // 预览从 _WeldPose_2mm 文件读回的段类带后缀，不剥会漏补偿过渡点、轨迹出现弯折。
        const int defaultIndex = DefaultPoseCompSlotIndex(NormalizeSeamCompSegmentKind(segmentKind));
        if (defaultIndex >= 0 && defaultIndex < static_cast<int>(poseCompSlots.size()))
        {
            slotIndex = defaultIndex;
        }
    }
    else
    {
        double bestDistance = std::numeric_limits<double>::max();
        for (int index = 0; index < static_cast<int>(poseCompSlots.size()); ++index)
        {
            const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[index];
            if (!slot.validReference)
            {
                continue;
            }
            const double distance = PoseDistanceDeg(rx, ry, rz, slot.poseRx, slot.poseRy, slot.poseRz);
            if (distance > poseMatchMaxErrorDeg)
            {
                continue;
            }
            if (distance < bestDistance)
            {
                bestDistance = distance;
                slotIndex = index;
            }
        }
    }

    if (slotIndex < 0)
    {
        return point;
    }
    const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
    const Eigen::Vector3d poseCompLocal(slot.compX, slot.compY, slot.compZ);
    if (poseCompLocal.norm() <= 1e-9)
    {
        return point;
    }
    return point + RobotPoseTransform::RotationFromAnglesDeg(rx, ry, rz, robotType) * poseCompLocal;
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

    QVector<QString> measurementDepthSegmentKinds;
    const bool hasMeasurementDepthSegmentKinds =
        AssignSegmentKindsByMeasurementGunDepth(
            result.points,
            keyPointPositions,
            preset,
            keyPointTypes,
            measurementDepthSegmentKinds,
            appendLog);

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
        segment.kind = hasMeasurementDepthSegmentKinds
            && static_cast<int>(segmentIndex) < measurementDepthSegmentKinds.size()
                ? measurementDepthSegmentKinds[static_cast<int>(segmentIndex)]
                : QString();
        if (segment.kind.isEmpty())
        {
            segment.kind = result.points[segment.begin].segmentKindAfter.trimmed();
        }
        if (segment.kind.isEmpty())
        {
            segment.kind = LowerWeldSegmentKindText(segment.beginType, segment.endMarkerType);
        }

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

    const bool useTaughtWeldPose = preset.useTaughtWeldPose;
    const double outputPoseRx = useTaughtWeldPose ? preset.taughtWeldPoseRx : preset.rx;
    const double outputPoseRy = useTaughtWeldPose ? preset.taughtWeldPoseRy : preset.ry;
    double taughtPlatformRz = NormalizeRobotRzOutputRange(preset.taughtWeldPoseRz);
    double taughtComputedPlatformRz = 0.0;
    double taughtRzOffset = 0.0;
    bool hasTaughtRzReference = false;
    QString taughtReferenceKind;
    if (useTaughtWeldPose)
    {
        const SegmentInfo* referenceSegment = nullptr;
        for (const SegmentInfo& segment : segments)
        {
            if (IsPlatformSegmentKind(segment.kind))
            {
                referenceSegment = &segment;
                break;
            }
        }
        if (referenceSegment == nullptr)
        {
            referenceSegment = &segments.front();
        }

        taughtComputedPlatformRz = NormalizeAngleNear(referenceSegment->fixedRz, taughtPlatformRz);
        const double taughtRzNearComputed = NormalizeAngleNear(taughtPlatformRz, taughtComputedPlatformRz);
        taughtRzOffset = taughtComputedPlatformRz - taughtRzNearComputed;
        taughtReferenceKind = referenceSegment->kind;
        hasTaughtRzReference = true;
        if (appendLog)
        {
            appendLog(QString("启用示教焊接姿态：RX=%1, RY=%2；参考段=%3，计算平台RZ=%4 deg，示教RZ=%5 deg，差值=%6 deg；平台使用示教RZ，坡道使用计算RZ减差值。")
                .arg(outputPoseRx, 0, 'f', 3)
                .arg(outputPoseRy, 0, 'f', 3)
                .arg(taughtReferenceKind)
                .arg(taughtComputedPlatformRz, 0, 'f', 3)
                .arg(taughtRzNearComputed, 0, 'f', 3)
                .arg(taughtRzOffset, 0, 'f', 3));
        }
    }

    auto taughtAdjustedRzForKind = [&](double calculatedRz, const QString& segmentKind) -> double
    {
        if (!useTaughtWeldPose || !hasTaughtRzReference)
        {
            return NormalizeRobotRzOutputRange(calculatedRz);
        }
        if (IsPlatformSegmentKind(segmentKind))
        {
            return taughtPlatformRz;
        }

        const double calculatedNearReference = NormalizeAngleNear(calculatedRz, taughtComputedPlatformRz);
        return NormalizeRobotRzOutputRange(calculatedNearReference - taughtRzOffset);
    };

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
            outputPoseRx,
            outputPoseRy,
            useTaughtWeldPose ? taughtAdjustedRzForKind(segment.fixedRz, segment.kind) : segment.fixedRz,
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
                .arg(outputPoseRx, 0, 'f', 3)
                .arg(outputPoseRy, 0, 'f', 3)
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

    // 姿态补偿槽位匹配 + 工具系旋转应用已下沉为自由函数 ApplyPoseCompToPoint（见上方），
    // 管线与补偿预览共用同一份数学。

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

    auto keyPointTypeForPointIndex = [&](int pointIndex) -> RobotCalculation::LowerWeldPointType
    {
        for (std::size_t keyIndex = 0; keyIndex < keyPointPositions.size(); ++keyIndex)
        {
            if (keyPointPositions[keyIndex] == pointIndex)
            {
                return keyPointTypes[keyIndex];
            }
        }
        if (pointIndex >= 0 && pointIndex < result.points.size())
        {
            return result.points[pointIndex].type;
        }
        return RobotCalculation::LowerWeldPointType::Normal;
    };

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
                return keyPointTypeForPointIndex(pointIndex);
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
        const QString nextSegmentKind = hasNextSegment
            ? segments[segmentIndex + 1].kind
            : segment.kind;
        const bool inTransition = hasNextSegment
            && segment.transitionBeginDistance < std::numeric_limits<double>::max()
            && sampleDistance >= segment.transitionBeginDistance;

        double pointRz = segment.fixedRz;
        double transitionRatio = 0.0;
        if (inTransition && preset.cornerTransitionLeadDistance > 1e-6)
        {
            const double remainingDistance = std::max(0.0, segment.endDistance - sampleDistance);
            transitionRatio = 1.0 - (remainingDistance / preset.cornerTransitionLeadDistance);
            pointRz = segment.fixedRz
                + (nextSegmentRz - segment.fixedRz) * std::clamp(transitionRatio, 0.0, 1.0);
        }
        // Transition points still change angle. The target RZ at each side of
        // the transition comes from the segment weld normal, while slope
        // segments are clamped before interpolation.
        if (useTaughtWeldPose && hasTaughtRzReference)
        {
            if (inTransition && preset.cornerTransitionLeadDistance > 1e-6)
            {
                const double beginRz = taughtAdjustedRzForKind(segment.fixedRz, segment.kind);
                const double endRz = NormalizeAngleNear(
                    taughtAdjustedRzForKind(nextSegmentRz, nextSegmentKind),
                    beginRz);
                pointRz = beginRz + (endRz - beginRz) * std::clamp(transitionRatio, 0.0, 1.0);
            }
            else
            {
                pointRz = taughtAdjustedRzForKind(segment.fixedRz, segment.kind);
            }
            pointRz = NormalizeRobotRzOutputRange(pointRz);
        }
        else
        {
            pointRz = NormalizeRobotRzOutputRange(pointRz + preset.weldRzGainDeg);
        }

        const RobotCalculation::LowerWeldPointType pointType =
            samplePointTypeAtDistance(sampleDistance, sourceIndex);

        double pointRx = outputPoseRx;
        double pointRy = outputPoseRy;
        Eigen::Vector3d point = sampledPoint;
        point = ApplyPoseCompToPoint(
            poseCompSlots,
            preset.poseCompMatchMode,
            preset.poseMatchMaxErrorDeg,
            preset.robotType,
            point,
            pointRx,
            pointRy,
            pointRz,
            segment.kind);

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

    const PoseCompJunctionApplyStats poseCompJunctionStats =
        ApplyPoseCompSegmentJunctionIntersections(records);
    const int poseCompDensifiedPointCount =
        DensifyWeldPoseRecordsByStep(records, kExpandedSampleStepMm);
    if (appendLog && poseCompJunctionStats.adjustedJunctionCount > 0)
    {
        appendLog(QString("姿态补偿段交点重建：重建平台/坡面交点=%1，裁剪多余采样点=%2。")
            .arg(poseCompJunctionStats.adjustedJunctionCount)
            .arg(poseCompJunctionStats.removedPointCount));
    }
    if (appendLog && poseCompDensifiedPointCount > 0)
    {
        appendLog(QString("姿态补偿段交点重建后补点：新增=%1，步长=%2mm。")
            .arg(poseCompDensifiedPointCount)
            .arg(kExpandedSampleStepMm, 0, 'f', 3));
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
            ? QString("创建或打开测量焊接参数数据失败：%1").arg(RobotDataHelper::MeasureWeldParamPath(robotName))
            : ensureError;
        return false;
    }
    const QString iniPath = RobotDataHelper::MeasureWeldParamPath(robotName);
    if (!ConfigDatabase::HasIniFile(iniPath))
    {
        error = ensureError.isEmpty() ? QString("未找到测量焊接参数数据：%1").arg(iniPath) : ensureError;
        return false;
    }

    param.sIniFilePath = ToUtf8StdString(iniPath);

    COPini ini;
    if (!ini.SetFileName(param.sIniFilePath))
    {
        error = QString("打开测量焊接参数数据失败：%1").arg(iniPath);
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
            error = QString("打开焊接参数数据失败：%1").arg(weldParamPath);
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
    pWeldIni->ReadString(false, "FinalWeldTrajectoryStepMm", &param.dFinalWeldTrajectoryStepMm);
    pWeldIni->ReadString(false, "WeldDirection", &param.nWeldDirection);
    pWeldIni->ReadString(false, "GunDownBackSafeDis", &param.dGunDownBackSafeDis);
    pWeldIni->ReadString(false, "WeldRzGainDeg", &param.dWeldRzGainDeg);
    int useTaughtWeldPose = 0;
    pWeldIni->ReadString(false, "UseTaughtWeldPose", &useTaughtWeldPose);
    pWeldIni->ReadString(false, "TaughtWeldPoseRX", &param.dTaughtWeldPoseRxDeg);
    pWeldIni->ReadString(false, "TaughtWeldPoseRY", &param.dTaughtWeldPoseRyDeg);
    pWeldIni->ReadString(false, "TaughtWeldPoseRZ", &param.dTaughtWeldPoseRzDeg);
    param.bUseTaughtWeldPose = (useTaughtWeldPose != 0);
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
    param.dFinalWeldTrajectoryStepMm = NormalizeFinalWeldTrajectorySampleStepMm(param.dFinalWeldTrajectoryStepMm);
    param.nWeldDirection = param.nWeldDirection < 0 ? -1 : 1;
    // 焊接方向已迁入工艺参数：当前工艺设置过（非0）时优先于测量参数页旧值，
    // 此处统一覆盖，下游（执行反转/已焊起点截断/预览箭头）全部跟随。
    {
        T_WELD_PARA activeWeld = {};
        if (TryLoadActiveWeldProcessParam(robotName, activeWeld, nullptr)
            && activeWeld.nWeldDirection != 0)
        {
            param.nWeldDirection = activeWeld.nWeldDirection < 0 ? -1 : 1;
        }
    }
    if (!std::isfinite(param.dGunDownBackSafeDis) || param.dGunDownBackSafeDis <= 0.0)
    {
        param.dGunDownBackSafeDis = WELD_SAFE_OFFSET_DISTANCE_MM;
    }
    if (!std::isfinite(param.dWeldRzGainDeg))
    {
        param.dWeldRzGainDeg = 0.0;
    }
    if (!std::isfinite(param.dTaughtWeldPoseRxDeg))
    {
        param.dTaughtWeldPoseRxDeg = 0.0;
    }
    if (!std::isfinite(param.dTaughtWeldPoseRyDeg))
    {
        param.dTaughtWeldPoseRyDeg = 0.0;
    }
    if (!std::isfinite(param.dTaughtWeldPoseRzDeg))
    {
        param.dTaughtWeldPoseRzDeg = 0.0;
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
    const MeasureThenWeldRuntimeConfig::ScanTimestampSource scanTimestampSource =
        MeasureThenWeldRuntimeConfig::LoadScanTimestampSource();
    const bool useRobotTimestampForScan =
        scanTimestampSource != MeasureThenWeldRuntimeConfig::ScanTimestampSource::Pc;
    const QString scanTimestampSourceName = MeasureThenWeldRuntimeConfig::DisplayName(scanTimestampSource);
    const QString scanTimestampFieldName = MeasureThenWeldRuntimeConfig::FieldName(scanTimestampSource);
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
    QVector<RobotCalculation::IndexedPoint3D> workpieceCloudInput;
    cameraSamples.reserve(10000);
    matchedCameraSamples.reserve(10000);
    laserFitInput.reserve(10000);
    workpieceCloudInput.reserve(100000);
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
        &passiveRobotSamplingActive,
        useRobotTimestampForScan]()
        {
            RobotCalculation::TimestampedRobotPose sample;
            RobotDriverAdaptor::StateSnapshot snapshot;
            if (pRobotDriver->LatestStateSnapshot(snapshot) && snapshot.valid)
            {
                const long long selectedTimestampMs = useRobotTimestampForScan
                    ? snapshot.robotMs
                    : snapshot.pcRecvMs;
                if (selectedTimestampMs > 0
                    && passiveRobotSamplingActive
                    && selectedTimestampMs == lastRobotMonitorMs)
                {
                    return false;
                }

                if (selectedTimestampMs > 0)
                {
                    sample.pose = snapshot.pose;
                    sample.timestampUs = static_cast<qint64>(selectedTimestampMs) * 1000;
                    lastRobotMonitorMs = selectedTimestampMs;
                    passiveRobotSamplingActive = true;
                }
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
        appendLog(QString("开始扫描运动：相机帧由当前机器人专属缓存读取，配置相机读取帧率=%1 fps（约 %2 ms/帧，用于时间间隔统计），机器人位姿约 %3 ms 采样；扫描匹配时间轴=%4（%5），相机帧timestamp会在首帧处映射到该时间轴，并叠加相机时间补偿 %6 ms。点云转换使用 %7 个后台处理线程。配置扫描速度= %8 mm/min，下发速度= %9 %10")
            .arg(actualCameraReadFps, 0, 'f', 2)
            .arg(cameraReadIntervalMs)
            .arg(ROBOT_SAMPLE_INTERVAL_MS)
            .arg(scanTimestampSourceName)
            .arg(scanTimestampFieldName)
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
    // 扫描完成超时按扫描距离/配置速度动态估计（固定 120s 在慢速测量长焊缝时不够用）：
    // 预计时间×2 + 30s，夹在 [120s, 1800s]，与焊接执行的完成超时算法一致。
    const double scanDistanceMm = std::hypot(
        param.tEndPos.dX - param.tStartPos.dX,
        param.tEndPos.dY - param.tStartPos.dY,
        param.tEndPos.dZ - param.tStartPos.dZ);
    const double scanSpeedMmPerSec = param.dScanSpeed > 1e-6 ? param.dScanSpeed / 60.0 : 0.0;
    const double estimatedScanMs = scanSpeedMmPerSec > 1e-6
        ? (scanDistanceMm / scanSpeedMmPerSec) * 1000.0
        : 0.0;
    const int scanFinishTimeoutMs = static_cast<int>(std::clamp(
        estimatedScanMs * 2.0 + 30000.0,
        120000.0,
        1800000.0));
    if (appendLog)
    {
        appendLog(QString("扫描距离≈%1 mm，预计扫描≈%2 s，完成超时=%3 s。")
            .arg(scanDistanceMm, 0, 'f', 1)
            .arg(estimatedScanMs / 1000.0, 0, 'f', 1)
            .arg(scanFinishTimeoutMs / 1000.0, 0, 'f', 0));
    }
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
			if (motionStarted && elapsedMs > scanFinishTimeoutMs)
			{
				if (appendLog)
				{
					if (pFanucDriver != nullptr)
					{
						appendLog(QString("扫描运动等待完成超时（%1 s）：R[%2]=%3")
							.arg(scanFinishTimeoutMs / 1000.0, 0, 'f', 0)
							.arg(FANUC_MOTION_STATE_REG)
							.arg(motionState));
					}
					else
					{
						appendLog(QString("扫描运动等待完成超时（%1 s）：CheckDone=%2")
							.arg(scanFinishTimeoutMs / 1000.0, 0, 'f', 0)
							.arg(motionState));
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
    const QString sdkPointCloudDir = QDir(laserDir).filePath(SDK_POINT_CLOUD_OUTPUT_DIR_NAME);
    const QString sdkSeamExtractedPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_FILE_NAME);
    const QString sdkSeamExtracted2mmPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_2MM_FILE_NAME);
    const QString sdkBaseWeldPath = QDir(sdkPointCloudDir).filePath(SDK_BASE_WELD_FILE_NAME);
    const QString preservePathFitPath = QDir(laserDir).filePath(PRESERVE_PATH_FILE_NAME);
    const QString keyPointsPath = QDir(laserDir).filePath(KEY_POINTS_FILE_NAME);
    const QString classifiedPath = QDir(laserDir).filePath(CLASSIFIED_FILE_NAME);
    const QString cornerCompKeyPointsPath = QDir(laserDir).filePath(CORNER_COMP_KEY_POINTS_FILE_NAME);
    const QString cornerCompClassifiedPath = QDir(laserDir).filePath(CORNER_COMP_CLASSIFIED_FILE_NAME);
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
                const int cloudIndex = workpieceCloudPointIndex++;
                QStringList cloudFields;
                cloudFields
                    << QString::number(cloudIndex)
                    << QString::number(cloudPoint.workpiecePoint.x(), 'f', 6)
                    << QString::number(cloudPoint.workpiecePoint.y(), 'f', 6)
                    << QString::number(cloudPoint.workpiecePoint.z(), 'f', 6);
                workpieceCloudLines.push_back(cloudFields.join(' '));

                RobotCalculation::IndexedPoint3D cloudInputPoint;
                cloudInputPoint.index = cloudIndex;
                cloudInputPoint.point = cloudPoint.workpiecePoint;
                workpieceCloudInput.push_back(cloudInputPoint);
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
        appendLog(QString("局部完整点云：参与帧=%1，点数=%2，跳过异常线点=%3")
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
        appendLog(QString("局部完整点云文件：%1").arg(workpieceCloudPath));
        appendLog(QString("相机-机器人-激光匹配明细文件：%1").arg(matchDebugPath));
    }

    RobotCalculation::LowerWeldFilterParams originalFitParams = BuildOriginalTrackFitParams(param);
    if (originalFitParams.exportFitDebugCloud && !laserDir.isEmpty())
    {
        // 真机路径把拟合调试点云导出到本次结果的 LaserPoint 目录下（FitDebug 子目录）。
        originalFitParams.fitDebugDir = laserDir;
    }
    const PointCloudProcessingConfig::Settings pointCloudSettings = PointCloudProcessingConfig::Load();
    // ①②③ 三种点云链方法都以完整点云为输入（③另需相机轨迹点做投影种子，④只用激光轨迹点）。
    const bool canUseExternalCloud =
        pointCloudSettings.mode != PointCloudProcessingConfig::Mode::LegacyLaserPath
        && workpieceCloudInput.size() >= 2;
    if (laserFitInput.size() < 2 && !canUseExternalCloud)
    {
        if (appendLog)
        {
            appendLog(QString("激光有效点过少（%1），完整点云有效点=%2，跳过 PreservePath 拟合、焊道分类和焊接姿态生成。")
                .arg(laserFitInput.size())
                .arg(workpieceCloudInput.size()));
        }
        return true;
    }

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

    bool usedExternalLibrary = false;
    PointCloudExtractionProcessor::ExtractionResult externalExtraction;
    const RobotCalculation::MeasureThenWeldAnalysisResult originalAnalysis =
        AnalyzeMeasureThenWeldPointCloud(
            laserFitInput,
            workpieceCloudInput,
            param,
            originalFitParams,
            sdkBaseWeldPath,
            laserDir,
            appendLog,
            &usedExternalLibrary,
            &externalExtraction);
    EnsureWorkpieceMeshCacheFromCloud(laserDir, workpieceCloudInput, appendLog);
    if (!originalAnalysis.ok)
    {
        error = QString("先测后焊特征分析失败：%1").arg(originalAnalysis.error);
        if (appendLog)
        {
            appendLog(error);
            appendLog("已保留原始激光点文件，可先按原始点云继续分析。");
        }
        return false;
    }

    if (usedExternalLibrary)
    {
        QDir().mkpath(sdkPointCloudDir);
        if (!SaveTextLines(sdkSeamExtractedPath, BuildSdkTrackOutputLines(externalExtraction.rawPoints, "sdk_extracted"), error))
        {
            if (appendLog)
            {
                appendLog(QString("保存SDK提取焊道结果失败：%1").arg(error));
            }
            return true;
        }
        if (!SaveTextLines(
                sdkSeamExtracted2mmPath,
                BuildSdkTrackOutputLines(externalExtraction.keyPointExpandedPoints, "sdk_keypoint_2mm"),
                error))
        {
            if (appendLog)
            {
                appendLog(QString("保存SDK提取焊道2mm采样结果失败：%1").arg(error));
            }
            return true;
        }
        if (appendLog)
        {
            appendLog(QString("SDK提取点云焊道文件：%1，点数=%2")
                .arg(sdkSeamExtractedPath)
                .arg(externalExtraction.rawPoints.size()));
            appendLog(QString("SDK提取点云焊道2mm采样文件：%1，点数=%2")
                .arg(sdkSeamExtracted2mmPath)
                .arg(externalExtraction.keyPointExpandedPoints.size()));
            appendLog(QString("SDK基础焊道文件：%1，点数=%2")
                .arg(externalExtraction.baseWeldPath.isEmpty() ? sdkBaseWeldPath : externalExtraction.baseWeldPath)
                .arg(externalExtraction.points.size()));
        }
        QString schemeCompareError;
        if (!SaveSdkSchemeCompareOutputs(
                *this,
                sdkPointCloudDir,
                laserFitInput,
                externalExtraction,
                originalFitParams,
                schemeCompareError,
                appendLog)
            && appendLog)
        {
            appendLog(QString("SDK三方案对比输出失败：%1").arg(schemeCompareError));
        }
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
        if (usedExternalLibrary)
        {
            appendLog("本次 PreservePath 来自新版精测点云库输出。");
        }
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

    // 独立圆弧过渡预览(只输出、不参与主流程)：在分类点之后对拐角做圆弧过渡，导出 CloudCompare 点云，
    // 叠加 PreciseLaserPoint.txt 即可核对生成的圆弧是否贴合原本焊道。受“导出调试点云”开关控制。
    if (PointCloudProcessingConfig::Load().exportFitDebugCloud)
    {
        const QString arcPreviewPath =
            QDir(laserDir).filePath(QStringLiteral("PreciseLaserPoint_ArcTransitionPreview.txt"));
        QString arcPreviewError;
        if (SaveTextLines(
                arcPreviewPath,
                BuildArcTransitionPreviewCloudLines(originalAnalysis.classificationResult, LoadWeldPosePreset(param)),
                arcPreviewError)
            && appendLog)
        {
            appendLog(QString("圆弧过渡预览点云(CloudCompare)：%1").arg(arcPreviewPath));
        }
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

    const bool useCornerCompensatedClassification =
        originalAnalysis.cornerCompensatedClassificationResult.ok
        && !originalAnalysis.cornerCompensatedClassificationResult.points.isEmpty();
    if (useCornerCompensatedClassification)
    {
        if (!SaveTextLines(
                cornerCompClassifiedPath,
                BuildClassifiedOutputLines(originalAnalysis.cornerCompensatedClassificationResult),
                error))
        {
            if (appendLog)
            {
                appendLog(QString("保存拐点补偿后焊道分类结果失败：%1").arg(error));
            }
            return true;
        }
        if (!SaveTextLines(cornerCompKeyPointsPath, BuildKeyPointOutputLines(originalAnalysis.cornerCompensatedKeyPoints), error))
        {
            if (appendLog)
            {
                appendLog(QString("保存拐点补偿后起终点/拐点结果失败：%1").arg(error));
            }
            return true;
        }
    }
    else if (originalFitParams.enableCornerCompensation && appendLog)
    {
        appendLog(QString("拐点补偿未生成：%1").arg(
            originalAnalysis.cornerCompensatedClassificationResult.error.isEmpty()
                ? QStringLiteral("未找到可补偿的上坡/下坡拐点或补偿值为 0。")
                : originalAnalysis.cornerCompensatedClassificationResult.error));
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
        if (useCornerCompensatedClassification)
        {
            appendLog(QString("拐点补偿后分类文件：%1").arg(cornerCompClassifiedPath));
            appendLog(QString("拐点补偿后起终点/拐点文件：%1").arg(cornerCompKeyPointsPath));
            appendLog("焊接姿态将使用拐点补偿后分类点生成。");
        }
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
        appendLog(QString("焊接姿态参数：模式=%1, RX=%2, RY=%3, 示教RZ=%4 deg, RZ增益=%5 deg, 爬坡RZ夹紧=[%6, %7] deg, 拐点前过渡=%8 mm, 起点跳过=%9 mm, 终点跳过=%10 mm, 姿态补偿槽=%11, 焊道补偿槽=%12, 基础参数来源=%13, 姿态补偿来源=%14, 焊道补偿来源=%15")
            .arg(weldPosePreset.useTaughtWeldPose ? QStringLiteral("示教平台姿态") : QStringLiteral("原始固定姿态"))
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRx : weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRy : weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.taughtWeldPoseRz, 0, 'f', 3)
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

    const RobotCalculation::LowerWeldClassificationResult& classificationForWeldPose =
        useCornerCompensatedClassification
            ? originalAnalysis.cornerCompensatedClassificationResult
            : originalAnalysis.classificationResult;
    const std::vector<QString> weldPoseLines =
        BuildSegmentPoseOutputLines(classificationForWeldPose, param, weldPosePreset, appendLog);
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
    const QString workpieceCloudPath = dir.filePath(WORKPIECE_CLOUD_FILE_NAME);
    const QString sdkPointCloudDir = dir.filePath(SDK_POINT_CLOUD_OUTPUT_DIR_NAME);
    const QString sdkSeamExtractedPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_FILE_NAME);
    const QString sdkSeamExtracted2mmPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_2MM_FILE_NAME);
    const QString sdkBaseWeldPath = QDir(sdkPointCloudDir).filePath(SDK_BASE_WELD_FILE_NAME);
    const QString keyPointsPath = dir.filePath(KEY_POINTS_FILE_NAME);
    const QString classifiedPath = dir.filePath(CLASSIFIED_FILE_NAME);
    const QString cornerCompKeyPointsPath = dir.filePath(CORNER_COMP_KEY_POINTS_FILE_NAME);
    const QString cornerCompClassifiedPath = dir.filePath(CORNER_COMP_CLASSIFIED_FILE_NAME);
    const QString classifiedNoisePath = dir.filePath(CLASSIFIED_NOISE_FILE_NAME);
    const QString matchDebugPath = dir.filePath(MATCH_DEBUG_FILE_NAME);
    weldPosePath = dir.filePath(WELD_POSE_FILE_NAME);
    seamCompPath = dir.filePath(WELD_POSE_SEAM_COMP_FILE_NAME);

    const PointCloudProcessingConfig::Settings pointCloudSettings = PointCloudProcessingConfig::Load();
    QString sourceLaserPath = dir.filePath(RAW_LASER_FILE_NAME);
    if (!QFileInfo::exists(sourceLaserPath))
    {
        if (QFileInfo::exists(workpieceCloudPath))
        {
            sourceLaserPath = workpieceCloudPath;
            if (appendLog)
            {
                appendLog(QString("未找到原始激光点文件 %1，临时使用局部完整点云作为回退输入。").arg(RAW_LASER_FILE_NAME));
            }
        }
        else if (QFileInfo::exists(preservePath))
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

    QVector<RobotCalculation::IndexedPoint3D> workpieceCloudInput;
    QString workpieceLoadError;
    if (QFileInfo::exists(workpieceCloudPath)
        && !RobotDataHelper::LoadIndexedPoint3DFile(workpieceCloudPath, workpieceCloudInput, &workpieceLoadError)
        && appendLog)
    {
        appendLog(QString("读取局部完整点云失败：%1（点云链方法将因输入不足报错）").arg(workpieceLoadError));
    }

    const RobotCalculation::LowerWeldFilterParams originalFitParams = BuildOriginalTrackFitParams(param);

    // ①②③ 三种点云链方法都以完整点云为输入（③另需相机轨迹点做投影种子，④只用激光轨迹点）。
    const bool canUseExternalCloud =
        pointCloudSettings.mode != PointCloudProcessingConfig::Mode::LegacyLaserPath
        && workpieceCloudInput.size() >= 2;
    if (laserFitInput.size() < 2 && !canUseExternalCloud)
    {
        error = QString("激光有效点过少（%1），完整点云有效点=%2，无法重建焊接文件。")
            .arg(laserFitInput.size())
            .arg(workpieceCloudInput.size());
        return false;
    }

    if (setFlowStep)
    {
        setFlowStep("正在重新计算 PreservePath、焊接姿态和焊道补偿文件");
    }
    if (appendLog)
    {
        appendLog(QString("跳过扫描重建输入：%1，点数=%2").arg(sourceLaserPath).arg(laserFitInput.size()));
        appendLog(QString("局部完整点云输入：%1，点数=%2").arg(workpieceCloudPath).arg(workpieceCloudInput.size()));
        appendLog(QString("开始先测后焊特征分析：采样主轴=%1，重采样步长=%2 mm，拐点拟合容差=%3 mm，每段最少点数=%4")
            .arg(SampleAxisName(originalFitParams.sampleAxis))
            .arg(originalFitParams.sampleStep, 0, 'f', 3)
            .arg(originalFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(originalFitParams.piecewiseMinSegmentPoints));
    }

    bool usedExternalLibrary = false;
    PointCloudExtractionProcessor::ExtractionResult externalExtraction;
    const RobotCalculation::MeasureThenWeldAnalysisResult originalAnalysis =
        AnalyzeMeasureThenWeldPointCloud(
            laserFitInput,
            workpieceCloudInput,
            param,
            originalFitParams,
            sdkBaseWeldPath,
            laserDir,
            appendLog,
            &usedExternalLibrary,
            &externalExtraction);
    EnsureWorkpieceMeshCacheFromCloud(laserDir, workpieceCloudInput, appendLog);
    if (!originalAnalysis.ok)
    {
        error = QString("先测后焊特征分析失败：%1").arg(originalAnalysis.error);
        return false;
    }

    if (usedExternalLibrary)
    {
        QDir().mkpath(sdkPointCloudDir);
        if (!SaveTextLines(sdkSeamExtractedPath, BuildSdkTrackOutputLines(externalExtraction.rawPoints, "sdk_extracted"), error))
        {
            return false;
        }
        if (!SaveTextLines(
                sdkSeamExtracted2mmPath,
                BuildSdkTrackOutputLines(externalExtraction.keyPointExpandedPoints, "sdk_keypoint_2mm"),
                error))
        {
            return false;
        }
        QString schemeCompareError;
        if (!SaveSdkSchemeCompareOutputs(
                *this,
                sdkPointCloudDir,
                laserFitInput,
                externalExtraction,
                originalFitParams,
                schemeCompareError,
                appendLog)
            && appendLog)
        {
            appendLog(QString("SDK三方案对比输出失败：%1").arg(schemeCompareError));
        }
    }

    if (!SaveTextLines(preservePath, BuildFilterOutputLines(originalAnalysis.filterResult), error))
    {
        return false;
    }
    if (!SaveTextLines(classifiedPath, BuildClassifiedOutputLines(originalAnalysis.classificationResult), error))
    {
        return false;
    }

    // 独立圆弧过渡预览(只输出、不参与主流程)：分类点之后对拐角做圆弧过渡，导出 CloudCompare 点云，
    // 叠加 PreciseLaserPoint.txt 核对圆弧是否贴合焊道。受“导出调试点云”开关控制。
    if (PointCloudProcessingConfig::Load().exportFitDebugCloud)
    {
        const QString arcPreviewPath = QDir(QFileInfo(classifiedPath).absolutePath())
            .filePath(QStringLiteral("PreciseLaserPoint_ArcTransitionPreview.txt"));
        QString arcPreviewError;
        if (SaveTextLines(
                arcPreviewPath,
                BuildArcTransitionPreviewCloudLines(originalAnalysis.classificationResult, LoadWeldPosePreset(param)),
                arcPreviewError)
            && appendLog)
        {
            appendLog(QString("圆弧过渡预览点云(CloudCompare)：%1").arg(arcPreviewPath));
        }
    }

    if (!SaveTextLines(keyPointsPath, BuildKeyPointOutputLines(originalAnalysis.keyPoints), error))
    {
        return false;
    }
    if (!SaveTextLines(classifiedNoisePath, BuildNoiseOutputLines(laserFitInput, originalAnalysis.filterResult), error))
    {
        return false;
    }
    const bool useCornerCompensatedClassification =
        originalAnalysis.cornerCompensatedClassificationResult.ok
        && !originalAnalysis.cornerCompensatedClassificationResult.points.isEmpty();
    if (useCornerCompensatedClassification)
    {
        if (!SaveTextLines(
                cornerCompClassifiedPath,
                BuildClassifiedOutputLines(originalAnalysis.cornerCompensatedClassificationResult),
                error))
        {
            return false;
        }
        if (!SaveTextLines(cornerCompKeyPointsPath, BuildKeyPointOutputLines(originalAnalysis.cornerCompensatedKeyPoints), error))
        {
            return false;
        }
    }
    else if (originalFitParams.enableCornerCompensation && appendLog)
    {
        appendLog(QString("拐点补偿未生成：%1").arg(
            originalAnalysis.cornerCompensatedClassificationResult.error.isEmpty()
                ? QStringLiteral("未找到可补偿的上坡/下坡拐点或补偿值为 0。")
                : originalAnalysis.cornerCompensatedClassificationResult.error));
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
        if (usedExternalLibrary)
        {
            appendLog("本次 PreservePath 来自新版精测点云库输出。");
            appendLog(QString("SDK提取点云焊道文件：%1，点数=%2")
                .arg(sdkSeamExtractedPath)
                .arg(externalExtraction.rawPoints.size()));
            appendLog(QString("SDK提取点云焊道2mm采样文件：%1，点数=%2")
                .arg(sdkSeamExtracted2mmPath)
                .arg(externalExtraction.keyPointExpandedPoints.size()));
            appendLog(QString("SDK基础焊道文件：%1，点数=%2")
                .arg(externalExtraction.baseWeldPath.isEmpty() ? sdkBaseWeldPath : externalExtraction.baseWeldPath)
                .arg(externalExtraction.points.size()));
        }
        appendLog(QString("焊道分类文件：%1").arg(classifiedPath));
        appendLog(QString("起终点/拐点文件：%1").arg(keyPointsPath));
        appendLog(QString("焊道杂点文件：%1").arg(classifiedNoisePath));
        if (useCornerCompensatedClassification)
        {
            appendLog(QString("拐点补偿后分类文件：%1").arg(cornerCompClassifiedPath));
            appendLog(QString("拐点补偿后起终点/拐点文件：%1").arg(cornerCompKeyPointsPath));
            appendLog("焊接姿态将使用拐点补偿后分类点生成。");
        }
        appendLog(QString("焊接姿态参数：模式=%1, RX=%2, RY=%3, 示教RZ=%4 deg, RZ增益=%5 deg, 爬坡RZ夹紧=[%6, %7] deg, 拐点前过渡=%8 mm, 起点跳过=%9 mm, 终点跳过=%10 mm")
            .arg(weldPosePreset.useTaughtWeldPose ? QStringLiteral("示教平台姿态") : QStringLiteral("原始固定姿态"))
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRx : weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRy : weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.taughtWeldPoseRz, 0, 'f', 3)
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
    const RobotCalculation::LowerWeldClassificationResult& classificationForWeldPose =
        useCornerCompensatedClassification
            ? originalAnalysis.cornerCompensatedClassificationResult
            : originalAnalysis.classificationResult;
    const std::vector<QString> weldPoseLines =
        BuildSegmentPoseOutputLines(classificationForWeldPose, param, weldPosePreset, appendLog);
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
    const QVector<WeldPoseFileRecord> recordsBeforeTrim = records;
    const SeamCompFinalizeStats finalizeStats =
        FinalizeSeamCompedWeldPoseRecords(preset, recordsBeforeTrim, records, compStats);
    if (finalizeStats.emptyAfterEndpointTrim)
    {
        error = "焊道补偿和起终点裁剪后没有有效焊接点。";
        return false;
    }
    if (finalizeStats.emptyAfterSelfIntersection)
    {
        error = "焊道自交裁剪后没有有效焊接点。";
        return false;
    }

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
    summary = QString("焊道补偿完成：点数=%1，使用槽位=%2，Z补偿点数=%3，枪反向补偿点数=%4，焊道方向补偿点数=%5，起点裁剪=%6点，终点裁剪=%7点，自交裁剪=%8次，删除回折点=%9，交点校正拐点=%10，丢失拐点=%11，交点恢复=%12，跳过恢复=%13，未过渡拐点补偿=%14，补齐点=%15，过渡后补点=%16，最终补点=%17，最大步长=%18mm，圆弧过渡=%19处，半径=%20mm，新增点=%21，四类属性=%22，配置=%23")
        .arg(records.size())
        .arg(usedSlots.isEmpty() ? QString("无匹配槽位") : usedSlots.join(","))
        .arg(compStats.zAdjustedCount)
        .arg(compStats.gunDirAdjustedCount)
        .arg(compStats.seamDirAdjustedCount)
        .arg(finalizeStats.endpointTrim.removedStartCount)
        .arg(finalizeStats.endpointTrim.removedEndCount)
        .arg(compStats.selfIntersectionTrimCount)
        .arg(compStats.selfIntersectionRemovedPointCount)
        .arg(finalizeStats.cornerRestore.adjustedCornerCount)
        .arg(finalizeStats.cornerRestore.missingCornerCount)
        .arg(finalizeStats.cornerRestore.restoredCornerCount)
        .arg(finalizeStats.cornerRestore.skippedCornerCount)
        .arg(finalizeStats.smoothedRemainingCornerCount)
        .arg(finalizeStats.densifiedPointCount)
        .arg(finalizeStats.postArcDensifiedPointCount)
        .arg(finalizeStats.finalDensifiedPointCount)
        .arg(finalizeStats.densifyStepMm, 0, 'f', 3)
        .arg(finalizeStats.arc.roundedCornerCount)
        .arg(finalizeStats.arc.radiusMm, 0, 'f', 3)
        .arg(finalizeStats.arc.insertedPointCount())
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
    QVector<WeldPoseFileRecord> sampledRecords;
    if (!BuildWeldPoseMoveInfos(
        records,
        effectiveWeldSpeedMmPerMin,
        moveInfos,
        error,
        &preset,
        param.dFinalWeldTrajectoryStepMm,
        transitionCommandSpeed,
        true,
        &sampledRecords))
    {
        return false;
    }
    const QString sampledPosePath = BuildFinalSampledWeldPosePath(poseInfo.absoluteFilePath());
    QString sampledSaveError;
    const bool sampledSaved = SaveWeldPoseFileRecords(sampledPosePath, sampledRecords, sampledSaveError);

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
    summary += QString("；最终轨迹点间距=%1 mm")
        .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3);
    summary += sampledSaved
        ? QString("；抽样轨迹文件=%1").arg(sampledPosePath)
        : QString("；抽样轨迹文件保存失败=%1").arg(sampledSaveError);
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
        summary += QString("；摆动=%1，跟踪=%2")
            .arg(preset.weaveEnabled ? QStringLiteral("启用") : QStringLiteral("NULL"))
            .arg(preset.trackEnabled ? QStringLiteral("启用") : QStringLiteral("NULL"));
        summary += QString("；焊接模式=%1").arg(preset.arcMode);
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
    QVector<WeldPoseFileRecord> sampledRecords;
    if (!BuildWeldPoseMoveInfos(records, linearCommandSpeed, moveInfos, error, &preset, param.dFinalWeldTrajectoryStepMm, 0.0, false, &sampledRecords))
    {
        return false;
    }
    const QString sampledPosePath = BuildFinalSampledWeldPosePath(poseFilePath);
    QString sampledSaveError;
    const bool sampledSaved = SaveWeldPoseFileRecords(sampledPosePath, sampledRecords, sampledSaveError);

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

        summary = QString("点数=%1，最终轨迹点间距=%2 mm，轨迹速度=%3 mm/min (下发=%4 %5)，程序=%6，本地LS=%7，远程TP=%8，抽样轨迹文件=%9，当前仅下发未自动执行")
            .arg(static_cast<int>(moveInfos.size()))
            .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3)
            .arg(selectedSpeedMmPerMin, 0, 'f', 3)
            .arg(linearCommandSpeed, 0, 'f', 3)
            .arg(linearCommandSpeedUnit)
            .arg(QString::fromStdString(programName))
            .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
            .arg(QString::fromStdString(remoteTpPath))
            .arg(sampledSaved ? sampledPosePath : QString("保存失败：%1").arg(sampledSaveError));
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
    summary = QString("点数=%1，最终轨迹点间距=%2 mm，方向=%3，轨迹速度=%4 mm/min (下发=%5 %6)，抽样轨迹文件=%7，STEP使用%8生成、上传并启动程序")
        .arg(static_cast<int>(moveInfos.size()))
        .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3)
        .arg(WeldDirectionText(preset))
        .arg(selectedSpeedMmPerMin, 0, 'f', 3)
        .arg(linearCommandSpeed, 0, 'f', 3)
        .arg(linearCommandSpeedUnit)
        .arg(sampledSaved ? sampledPosePath : QString("保存失败：%1").arg(sampledSaveError))
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
    const T_ROBOT_COORS weldStartCoors = BuildWeldPoseCoors(records.front());

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
    QVector<WeldPoseFileRecord> sampledRecords;
    if (!BuildWeldPoseMoveInfos(
        records,
        weldCommandSpeed,
        moveInfos,
        error,
        &weldPosePreset,
        param.dFinalWeldTrajectoryStepMm,
        transitionCommandSpeed,
        param.bDoActualWeld,
        &sampledRecords))
    {
        return false;
    }
    const QString sampledPosePath = BuildFinalSampledWeldPosePath(poseFilePath);
    QString sampledSaveError;
    const bool sampledSaved = SaveWeldPoseFileRecords(sampledPosePath, sampledRecords, sampledSaveError);

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
        appendLog(QString("焊接起点：%1").arg(RobotCoorsText(weldStartCoors)));
        appendLog(QString("收枪安全位置：%1").arg(RobotCoorsText(endSafeCoors)));
        appendLog(QString("焊接轨迹模式：%1，轨迹速度=%2 mm/min，来源=%3，下发速度=%4 %5")
            .arg(weldModeText)
            .arg(weldSpeedMmPerMin, 0, 'f', 3)
            .arg(weldSpeedSourceText)
            .arg(weldCommandSpeed, 0, 'f', 3)
            .arg(weldCommandSpeedUnit));
        appendLog(QString("最终焊接轨迹点间距：%1 mm，仅在生成/下发运动点时抽样")
            .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3));
        appendLog(sampledSaved
            ? QString("最终抽样轨迹文件：%1").arg(sampledPosePath)
            : QString("最终抽样轨迹文件保存失败：%1").arg(sampledSaveError));
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
                appendLog(QString("工艺附加变量：摆动=%1，跟踪=%2")
                    .arg(weldPosePreset.weaveEnabled ? QStringLiteral("启用") : QStringLiteral("NULL"))
                    .arg(weldPosePreset.trackEnabled ? QStringLiteral("启用") : QStringLiteral("NULL")));
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

        if (!MoveCoorsAndWait(
            pRobotDriver,
            weldStartCoors,
            safeMoveSpeedMmPerMin,
            "焊接起点",
            appendLog,
            setFlowStep))
        {
            error = "移动到焊接起点失败。";
            return false;
        }

        if (checkpoint && !checkpoint(
            "焊前确认",
            QString("下枪安全位置和焊接起点已到位，焊接轨迹程序也已下发完成。\n"
                    "运行模式：%1\n"
                    "下枪安全位置：%2\n"
                    "焊接起点：%3\n"
                    "焊接程序：%4\n"
                    "是否开始执行焊道？")
                .arg(weldModeText)
                .arg(RobotCoorsText(startSafeCoors))
                .arg(RobotCoorsText(weldStartCoors))
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

        if (!MoveCoorsAndWait(
            pRobotDriver,
            weldStartCoors,
            safeMoveSpeedMmPerMin,
            "焊接起点",
            appendLog,
            setFlowStep))
        {
            error = "移动到焊接起点失败。";
            return false;
        }

        if (checkpoint && !checkpoint(
            "焊前确认",
            QString("下枪安全位置和焊接起点已到位，STEP 将生成、上传并启动焊接轨迹。\n"
                    "运行模式：%1\n"
                    "下枪安全位置：%2\n"
                    "焊接起点：%3\n"
                    "点数：%4\n"
                    "是否开始执行焊道？")
                .arg(weldModeText)
                .arg(RobotCoorsText(startSafeCoors))
                .arg(RobotCoorsText(weldStartCoors))
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

    summary = QString("%1；安全移动速度=%2 mm/min；起点安全位=%3；焊接起点=%4；终点安全位=%5")
        .arg(downlinkSummary)
        .arg(safeMoveSpeedMmPerMin, 0, 'f', 3)
        .arg(RobotCoorsText(startSafeCoors))
        .arg(RobotCoorsText(weldStartCoors))
        .arg(RobotCoorsText(endSafeCoors));
    return true;
}

// ===== 补偿前后焊道可视化预览实现 =====
// 这些成员函数位于匿名命名空间之后，可直接复用其内的真实补偿数学
// （ApplyWeldSeamCompToWeldPoseRecords / TryParseWeldPoseFileRecord /
//  ResolveOverallHorizontalWeldDirection 等），保证预览与实际下发轨迹一致。

bool MeasureThenWeldService::LoadCompPreviewBaseline(
    CompPreviewKind kind,
    const QString& laserDir,
    QVector<CompPreviewPoint>& baseline,
    QString& error) const
{
    baseline.clear();
    QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("目录不存在：%1").arg(laserDir);
        return false;
    }

    if (kind == CompPreviewKind::Corner)
    {
        // 拐点补偿的补偿前基准 = 关键点（start/end/inner/outer），用于按几何重算段类并整体补偿。
        const QString keyPath = dir.filePath(KEY_POINTS_FILE_NAME);
        QFile keyFile(keyPath);
        if (!keyFile.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            error = QString("打开拐点文件失败：%1").arg(keyPath);
            return false;
        }
        QTextStream keyStream(&keyFile);
        keyStream.setEncoding(QStringConverter::Utf8);
        while (!keyStream.atEnd())
        {
            const QString lineText = keyStream.readLine().trimmed();
            if (lineText.isEmpty() || lineText.startsWith('#'))
            {
                continue;
            }
            const QStringList parts = lineText.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() < 5)
            {
                continue;
            }
            bool xOk = false, yOk = false, zOk = false, codeOk = false;
            const double x = parts[1].toDouble(&xOk);
            const double y = parts[2].toDouble(&yOk);
            const double z = parts[3].toDouble(&zOk);
            const int code = parts[4].toInt(&codeOk);
            if (!xOk || !yOk || !zOk || !codeOk)
            {
                continue;
            }
            CompPreviewPoint point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.typeCode = code;
            point.pointType = parts.size() > 5 ? parts[5] : QString();
            baseline.push_back(point);
        }
        if (baseline.size() < 2)
        {
            error = QString("未从 %1 解析到足够拐点（至少2个）。").arg(KEY_POINTS_FILE_NAME);
            return false;
        }
        return true;
    }

    // 焊道补偿/姿态补偿的补偿前基准 = 稠密 2mm 焊道姿态文件。
    const QString path = dir.filePath(WELD_POSE_FILE_NAME);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QString("打开焊道文件失败：%1").arg(path);
        return false;
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        WeldPoseFileRecord record;
        if (!TryParseWeldPoseFileRecord(line, record))
        {
            continue;
        }
        CompPreviewPoint point;
        point.x = record.point.x();
        point.y = record.point.y();
        point.z = record.point.z();
        point.rx = record.rx;
        point.ry = record.ry;
        point.rz = record.rz;
        point.bx = record.bx;
        point.by = record.by;
        point.bz = record.bz;
        point.weldIndex = record.weldIndex;
        point.rawIndex = record.rawIndex;
        point.segmentKind = record.segmentKind;
        point.pointType = record.pointType;
        baseline.push_back(point);
    }
    if (baseline.isEmpty())
    {
        error = QString("未从 %1 解析到有效焊道点。").arg(WELD_POSE_FILE_NAME);
        return false;
    }
    return true;
}

bool MeasureThenWeldService::LoadCompPreviewOriginalTrack(
    const QString& laserDir,
    QVector<CompPreviewPoint>& points,
    QString& error) const
{
    points.clear();
    QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("目录不存在：%1").arg(laserDir);
        return false;
    }
    const QString path = dir.filePath(CLASSIFIED_FILE_NAME);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QString("打开原始焊道文件失败：%1").arg(path);
        return false;
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    while (!stream.atEnd())
    {
        const QString lineText = stream.readLine().trimmed();
        if (lineText.isEmpty() || lineText.startsWith('#'))
        {
            continue;
        }
        const QStringList parts = lineText.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() < 4)
        {
            continue;
        }
        bool xOk = false, yOk = false, zOk = false;
        const double x = parts[1].toDouble(&xOk);
        const double y = parts[2].toDouble(&yOk);
        const double z = parts[3].toDouble(&zOk);
        if (!xOk || !yOk || !zOk)
        {
            continue;
        }
        CompPreviewPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        if (parts.size() > 4)
        {
            point.typeCode = parts[4].toInt();
        }
        points.push_back(point);
    }
    if (points.isEmpty())
    {
        error = QString("未从 %1 解析到原始焊道点。").arg(CLASSIFIED_FILE_NAME);
        return false;
    }
    return true;
}

MeasureThenWeldService::CompPreviewResult MeasureThenWeldService::RecomputeCompPreview(
    CompPreviewKind kind,
    const QString& robotName,
    const QVector<CompPreviewPoint>& baseline,
    const CompPreviewEditValues& edits) const
{
    CompPreviewResult result;
    if (baseline.isEmpty())
    {
        result.error = QStringLiteral("没有可用的基准焊道点。");
        return result;
    }
    result.before = baseline;

    struct ArrowBasis { double cx = 0.0, cy = 0.0, cz = 0.0, len = 10.0; };
    const auto computeArrowBasis = [](const QVector<CompPreviewPoint>& points) -> ArrowBasis
    {
        ArrowBasis basis;
        if (points.isEmpty())
        {
            return basis;
        }
        double minv[3] = { 1e300, 1e300, 1e300 };
        double maxv[3] = { -1e300, -1e300, -1e300 };
        double sum[3] = { 0.0, 0.0, 0.0 };
        for (const CompPreviewPoint& point : points)
        {
            const double coord[3] = { point.x, point.y, point.z };
            for (int axis = 0; axis < 3; ++axis)
            {
                sum[axis] += coord[axis];
                minv[axis] = std::min(minv[axis], coord[axis]);
                maxv[axis] = std::max(maxv[axis], coord[axis]);
            }
        }
        const double count = static_cast<double>(points.size());
        basis.cx = sum[0] / count;
        basis.cy = sum[1] / count;
        basis.cz = sum[2] / count;
        const double span = std::max({ maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2], 10.0 });
        basis.len = std::clamp(span * 0.15, 8.0, 80.0);
        return basis;
    };
    const auto addArrow = [&result](double ox, double oy, double oz, double vx, double vy, double vz,
        const QString& label, int colorId, bool doubleHeaded)
    {
        CompPreviewArrow arrow;
        arrow.origin[0] = ox; arrow.origin[1] = oy; arrow.origin[2] = oz;
        arrow.vector[0] = vx; arrow.vector[1] = vy; arrow.vector[2] = vz;
        arrow.label = label;
        arrow.colorId = colorId;
        arrow.doubleHeaded = doubleHeaded;
        result.arrows.push_back(arrow);
    };

    if (kind == CompPreviewKind::Seam)
    {
        QVector<WeldPoseFileRecord> records;
        records.reserve(baseline.size());
        for (int index = 0; index < baseline.size(); ++index)
        {
            WeldPoseFileRecord record;
            record.weldIndex = baseline[index].weldIndex;
            record.rawIndex = baseline[index].rawIndex;
            record.point = Eigen::Vector3d(baseline[index].x, baseline[index].y, baseline[index].z);
            record.rx = baseline[index].rx;
            record.ry = baseline[index].ry;
            record.rz = baseline[index].rz;
            record.bx = baseline[index].bx;
            record.by = baseline[index].by;
            record.bz = baseline[index].bz;
            record.segmentKind = baseline[index].segmentKind;
            record.pointType = baseline[index].pointType;
            records.push_back(record);
        }

        QVector<Eigen::Vector3d> basePoints;
        basePoints.reserve(records.size());
        for (const WeldPoseFileRecord& record : records)
        {
            basePoints.push_back(record.point);
        }
        const Eigen::Vector3d seamDir = ResolveOverallHorizontalWeldDirection(basePoints);
        const Eigen::Vector3d gunDir = HorizontalUnitOrZero(Eigen::Vector3d::UnitZ().cross(seamDir));
        result.seamAxis[0] = seamDir.x();
        result.seamAxis[1] = seamDir.y();
        result.seamAxis[2] = seamDir.z();
        result.gunAxis[0] = gunDir.x();
        result.gunAxis[1] = gunDir.y();
        result.gunAxis[2] = gunDir.z();

        // 用对话框当前编辑值构造 4 段焊道补偿槽（段类硬映射 0..3）。
        // 载入真实焊接预设（含圆弧过渡/裁剪等下发参数），再用对话框当前焊缝补偿值覆盖补偿槽。
        WeldPosePreset preset = LoadWeldPosePreset(BuildMeasureWeldParamShell(robotName));
        static const char* const kSegmentKinds[4] = { "low_platform", "rising_edge", "high_platform", "falling_edge" };
        preset.seamCompSlots.assign(4, WeldPosePreset::SeamCompSlot());
        for (int slotIndex = 0; slotIndex < 4; ++slotIndex)
        {
            // 用配置里真实的 segmentKind（可能是 CorrugatedPlate），为空才回退默认四段类；
            // 配合保留的真实 preset.seamKind，使匹配/回退与下发 FindSeamCompSlotForRecord 完全一致。
            const QString segmentKind = edits.seamSegmentKind[slotIndex].trimmed().isEmpty()
                ? QString::fromLatin1(kSegmentKinds[slotIndex])
                : edits.seamSegmentKind[slotIndex];
            preset.seamCompSlots[slotIndex].segmentKind = segmentKind;
            preset.seamCompSlots[slotIndex].weldZComp = edits.weldZComp[slotIndex];
            preset.seamCompSlots[slotIndex].weldGunDirComp = edits.weldGunDirComp[slotIndex];
            preset.seamCompSlots[slotIndex].weldSeamDirComp = edits.weldSeamDirComp[slotIndex];
        }

        // 复用管线真实焊缝补偿平移 + 完整后处理（端点裁剪/自交/拐点恢复/圆弧过渡/加密），贴近 _SeamComp 下发文件。
        WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
        const QVector<WeldPoseFileRecord> recordsBeforeTrim = records;
        FinalizeSeamCompedWeldPoseRecords(preset, recordsBeforeTrim, records, compStats);
        if (records.isEmpty())
        {
            records = recordsBeforeTrim;  // 后处理裁空则回退显示纯补偿平移结果
        }

        result.after.reserve(records.size());
        for (int index = 0; index < records.size(); ++index)
        {
            CompPreviewPoint point;
            point.x = records[index].point.x();
            point.y = records[index].point.y();
            point.z = records[index].point.z();
            point.segmentKind = records[index].segmentKind;
            point.pointType = records[index].pointType;
            result.after.push_back(point);
        }

        // 方向箭头：质心处 Z向 / 枪反向 / 焊道方向，标正负影响方向。
        const ArrowBasis basis = computeArrowBasis(result.before);
        addArrow(basis.cx, basis.cy, basis.cz, 0.0, 0.0, basis.len, QStringLiteral("Z向+"), 0, true);
        addArrow(basis.cx, basis.cy, basis.cz, gunDir.x() * basis.len, gunDir.y() * basis.len, gunDir.z() * basis.len, QStringLiteral("枪反向+"), 1, true);
        addArrow(basis.cx, basis.cy, basis.cz, seamDir.x() * basis.len, seamDir.y() * basis.len, seamDir.z() * basis.len, QStringLiteral("焊道方向+"), 2, true);
        result.ok = true;
        return result;
    }

    if (kind == CompPreviewKind::Pose)
    {
        // 用对话框当前编辑值构造 4 段姿态补偿槽（段类硬映射 0..3，validReference=true 以参与按姿态匹配）。
        std::vector<WeldPosePreset::PoseCompSlot> poseCompSlots(4);
        static const char* const kSegmentKinds[4] = { "low_platform", "rising_edge", "high_platform", "falling_edge" };
        for (int slotIndex = 0; slotIndex < 4; ++slotIndex)
        {
            poseCompSlots[slotIndex].segmentKind = QString::fromLatin1(kSegmentKinds[slotIndex]);
            poseCompSlots[slotIndex].poseRx = edits.poseRx[slotIndex];
            poseCompSlots[slotIndex].poseRy = edits.poseRy[slotIndex];
            poseCompSlots[slotIndex].poseRz = edits.poseRz[slotIndex];
            poseCompSlots[slotIndex].compX = edits.compX[slotIndex];
            poseCompSlots[slotIndex].compY = edits.compY[slotIndex];
            poseCompSlots[slotIndex].compZ = edits.compZ[slotIndex];
            poseCompSlots[slotIndex].validReference = true;
        }

        QVector<Eigen::Vector3d> basePoints;
        basePoints.reserve(baseline.size());
        result.after.reserve(baseline.size());
        for (const CompPreviewPoint& basePoint : baseline)
        {
            const Eigen::Vector3d before(basePoint.x, basePoint.y, basePoint.z);
            basePoints.push_back(before);
            // 复用管线下沉的姿态补偿（工具系 compX/Y/Z 按该点姿态旋到世界后叠加）。
            const Eigen::Vector3d afterPoint = ApplyPoseCompToPoint(
                poseCompSlots,
                edits.poseMatchMode,
                edits.poseMatchMaxErrorDeg,
                edits.robotType,
                before,
                basePoint.rx,
                basePoint.ry,
                basePoint.rz,
                basePoint.segmentKind);
            CompPreviewPoint afterPreview;
            afterPreview.x = afterPoint.x();
            afterPreview.y = afterPoint.y();
            afterPreview.z = afterPoint.z();
            afterPreview.rx = basePoint.rx;
            afterPreview.ry = basePoint.ry;
            afterPreview.rz = basePoint.rz;
            afterPreview.segmentKind = basePoint.segmentKind;
            afterPreview.pointType = basePoint.pointType;
            result.after.push_back(afterPreview);
        }

        const Eigen::Vector3d seamDir = ResolveOverallHorizontalWeldDirection(basePoints);
        const Eigen::Vector3d gunDir = HorizontalUnitOrZero(Eigen::Vector3d::UnitZ().cross(seamDir));
        result.seamAxis[0] = seamDir.x();
        result.seamAxis[1] = seamDir.y();
        result.seamAxis[2] = seamDir.z();
        result.gunAxis[0] = gunDir.x();
        result.gunAxis[1] = gunDir.y();
        result.gunAxis[2] = gunDir.z();

        // 方向箭头：姿态补偿 compX/Y/Z 在工具系，按代表点姿态旋到世界，画 X/Y/Z 补偿正方向。
        const ArrowBasis basis = computeArrowBasis(result.before);
        int repIndex = 0;
        double bestDistanceSq = std::numeric_limits<double>::max();
        for (int index = 0; index < baseline.size(); ++index)
        {
            const double dx = baseline[index].x - basis.cx;
            const double dy = baseline[index].y - basis.cy;
            const double dz = baseline[index].z - basis.cz;
            const double distanceSq = dx * dx + dy * dy + dz * dz;
            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                repIndex = index;
            }
        }
        const Eigen::Matrix3d rotation = RobotPoseTransform::RotationFromAnglesDeg(
            baseline[repIndex].rx, baseline[repIndex].ry, baseline[repIndex].rz, edits.robotType);
        const Eigen::Vector3d toolX = rotation.col(0) * basis.len;
        const Eigen::Vector3d toolY = rotation.col(1) * basis.len;
        const Eigen::Vector3d toolZ = rotation.col(2) * basis.len;
        addArrow(basis.cx, basis.cy, basis.cz, toolX.x(), toolX.y(), toolX.z(), QStringLiteral("X补偿+"), 3, true);
        addArrow(basis.cx, basis.cy, basis.cz, toolY.x(), toolY.y(), toolY.z(), QStringLiteral("Y补偿+"), 4, true);
        addArrow(basis.cx, basis.cy, basis.cz, toolZ.x(), toolZ.y(), toolZ.z(), QStringLiteral("Z补偿+"), 5, true);
        result.ok = true;
        return result;
    }

    // kind == CompPreviewKind::Corner：拐点补偿（复用公开的 RobotCalculation 拐点补偿，内部按几何重算段类）。
    result.before = baseline;

    const auto typeFromCode = [](int code) -> RobotCalculation::LowerWeldPointType
    {
        switch (code)
        {
        case 1: return RobotCalculation::LowerWeldPointType::Start;
        case 2: return RobotCalculation::LowerWeldPointType::End;
        case 3: return RobotCalculation::LowerWeldPointType::InnerCorner;
        case 4: return RobotCalculation::LowerWeldPointType::OuterCorner;
        case 6: return RobotCalculation::LowerWeldPointType::Noise;
        default: return RobotCalculation::LowerWeldPointType::Normal;
        }
    };

    QVector<RobotCalculation::LowerWeldClassifiedPoint> keyPoints;
    keyPoints.reserve(baseline.size());
    for (int index = 0; index < baseline.size(); ++index)
    {
        RobotCalculation::LowerWeldClassifiedPoint keyPoint;
        keyPoint.index = index;
        keyPoint.point = Eigen::Vector3d(baseline[index].x, baseline[index].y, baseline[index].z);
        keyPoint.type = typeFromCode(baseline[index].typeCode);
        keyPoints.push_back(keyPoint);
    }

    RobotCalculation::LowerWeldFilterParams params;
    params.enableCornerCompensation = edits.cornerEnabled;
    params.risingCornerCompensation.innerToOuterMm = edits.risingInnerToOuter;
    params.risingCornerCompensation.innerToInnerMm = edits.risingInnerToInner;
    params.risingCornerCompensation.outerToOuterMm = edits.risingOuterToOuter;
    params.risingCornerCompensation.outerToInnerMm = edits.risingOuterToInner;
    params.fallingCornerCompensation.innerToOuterMm = edits.fallingInnerToOuter;
    params.fallingCornerCompensation.innerToInnerMm = edits.fallingInnerToInner;
    params.fallingCornerCompensation.outerToOuterMm = edits.fallingOuterToOuter;
    params.fallingCornerCompensation.outerToInnerMm = edits.fallingOuterToInner;

    QVector<RobotCalculation::LowerWeldClassifiedPoint> compensatedKeyPoints;
    RobotCalculation::BuildCornerCompensatedLowerWeldClassification(keyPoints, params, &compensatedKeyPoints);

    if (compensatedKeyPoints.size() == keyPoints.size())
    {
        result.after.reserve(compensatedKeyPoints.size());
        for (int index = 0; index < compensatedKeyPoints.size(); ++index)
        {
            CompPreviewPoint point;
            point.x = compensatedKeyPoints[index].point.x();
            point.y = compensatedKeyPoints[index].point.y();
            point.z = compensatedKeyPoints[index].point.z();
            point.typeCode = baseline[index].typeCode;
            point.pointType = baseline[index].pointType;
            result.after.push_back(point);
        }

        // 方向箭头：每个发生位移的拐点画其移动方向（小位移放大显示）。
        const ArrowBasis basis = computeArrowBasis(result.before);
        for (int index = 0; index < result.before.size() && index < result.after.size(); ++index)
        {
            const double dx = result.after[index].x - result.before[index].x;
            const double dy = result.after[index].y - result.before[index].y;
            const double dz = result.after[index].z - result.before[index].z;
            const double moved = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (moved < 1e-6)
            {
                continue;
            }
            const double shown = std::clamp(moved * 4.0, basis.len * 0.4, basis.len * 1.5);
            const double scale = shown / moved;
            addArrow(result.before[index].x, result.before[index].y, result.before[index].z,
                dx * scale, dy * scale, dz * scale, QStringLiteral("拐点位移"), 6, false);
        }
        result.ok = true;
    }
    else
    {
        // 未启用拐点补偿或补偿值全为零 → 补偿后与补偿前一致。
        result.after = baseline;
        result.ok = true;
        if (!edits.cornerEnabled)
        {
            result.error = QStringLiteral("未启用本组拐点补偿（勾选\"启用本组拐点补偿\"后可见效果）。");
        }
    }
    return result;
}

namespace
{
// 流式抽样加载点文件（index x y z 格式）：按文件大小估算行数定抽样步长，
// 解析点数不超过 maxPoints。用于补偿预览显示完整点云（154MB 级，全量加载会卡死界面）。
bool LoadSampledPointFile(
    const QString& filePath,
    int maxPoints,
    QVector<MeasureThenWeldService::CompPreviewPoint>& points,
    QString& error)
{
    points.clear();
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QString("打开点文件失败：%1").arg(filePath);
        return false;
    }
    // 估算每行约 45 字节，定隔行抽样步长；估偏只影响显示点数，不影响正确性。
    const qint64 estimatedLines = std::max<qint64>(1, file.size() / 45);
    const int stride = static_cast<int>(std::max<qint64>(1, estimatedLines / std::max(1, maxPoints)));

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    qint64 lineIndex = 0;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine();
        if ((lineIndex++ % stride) != 0)
        {
            continue;
        }
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty())
        {
            continue;
        }
        const QStringList parts = trimmed.contains(',')
            ? trimmed.split(',', Qt::SkipEmptyParts)
            : trimmed.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() < 4)
        {
            continue;
        }
        bool xOk = false;
        bool yOk = false;
        bool zOk = false;
        const double x = parts[1].trimmed().toDouble(&xOk);
        const double y = parts[2].trimmed().toDouble(&yOk);
        const double z = parts[3].trimmed().toDouble(&zOk);
        if (!xOk || !yOk || !zOk
            || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            continue;
        }
        MeasureThenWeldService::CompPreviewPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        points.push_back(point);
    }
    return !points.isEmpty();
}
}

RobotCalculation::LowerWeldFilterParams MeasureThenWeldService::BuildTrackFitParamsFromSettings(
    const PointCloudProcessingConfig::Settings& pointCloudSettings,
    RobotCalculation::SampleAxis fallbackSampleAxis)
{
    RobotCalculation::LowerWeldFilterParams params;
    switch (pointCloudSettings.sampleAxisMode)
    {
    case PointCloudProcessingConfig::SampleAxisMode::AxisX:
        params.sampleAxis = RobotCalculation::SampleAxis::AxisX;
        break;
    case PointCloudProcessingConfig::SampleAxisMode::AxisY:
        params.sampleAxis = RobotCalculation::SampleAxis::AxisY;
        break;
    default:
        params.sampleAxis = fallbackSampleAxis;
        break;
    }
    // 方案三（立板投影）已并入方法③做前置提取，不再作为拟合方案映射；旧配置值按旧版几何处理。
    if (pointCloudSettings.featurePointStrategy == PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys)
    {
        params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::RobustSegmentedKeys;
    }
    else if (pointCloudSettings.featurePointStrategy == PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered)
    {
        params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::SlopeWaveFiltered;
    }
    else
    {
        params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::LegacyGeometry;
    }
    params.zThreshold = pointCloudSettings.cloudZThresholdMm;
    params.zJumpThreshold = pointCloudSettings.cloudZJumpThresholdMm;
    params.zContinuityThreshold = pointCloudSettings.cloudZContinuityThresholdMm;
    params.segmentBreakDistance = pointCloudSettings.cloudSegmentBreakDistanceMm;
    params.keepLongestSegmentOnly = pointCloudSettings.cloudKeepLongestSegmentOnly;
    params.sampleStep = pointCloudSettings.fitSampleStepMm;
    params.searchWindow = pointCloudSettings.fitSearchWindowMm;
    params.piecewiseFitTolerance = pointCloudSettings.fitPiecewiseToleranceMm;
    params.piecewiseMinSegmentPoints = pointCloudSettings.fitPiecewiseMinSegmentPoints;
    params.minPointCount = pointCloudSettings.fitMinPointCount;
    params.smoothRadius = pointCloudSettings.fitSmoothRadius;
    params.projectionStationWindowMm = pointCloudSettings.projectionStationWindowMm;
    params.projectionTransverseWindowMm = pointCloudSettings.projectionTransverseWindowMm;
    params.projectionZBandBelowMm = pointCloudSettings.projectionZBandBelowMm;
    params.projectionZBandAboveMm = pointCloudSettings.projectionZBandAboveMm;
    params.projectionMaxCandidatePerSeed = pointCloudSettings.projectionMaxCandidatePerSeed;
    params.projectionLayerLowPercent = pointCloudSettings.projectionLayerLowPercent;
    params.projectionLayerHighPercent = pointCloudSettings.projectionLayerHighPercent;
    params.projectionSmoothRadius = pointCloudSettings.projectionSmoothRadius;
    params.useSlopeConsistentCornerFit = pointCloudSettings.slopeConsistentCornerFit;
    params.exportFitDebugCloud = pointCloudSettings.exportFitDebugCloud;
    params.validationCoverageEnabled = pointCloudSettings.validationCoverageEnabled;
    params.validationMinFinitePointCount = pointCloudSettings.validationMinFinitePointCount;
    params.validationMinProjectedSpanMm = pointCloudSettings.validationMinProjectedSpanMm;
    params.validationContinuityEnabled = pointCloudSettings.validationContinuityEnabled;
    params.validationMinStationCoverageRatio = pointCloudSettings.validationMinStationCoverageRatio;
    params.validationMinLongestContinuousRatio = pointCloudSettings.validationMinLongestContinuousRatio;
    params.validationDenoiseRatioEnabled = pointCloudSettings.validationDenoiseRatioEnabled;
    params.validationMaxRejectedRatio = pointCloudSettings.validationMaxRejectedRatio;
    params.validationResidualEnabled = pointCloudSettings.validationResidualEnabled;
    params.validationMaxMedianResidualMm = pointCloudSettings.validationMaxMedianResidualMm;
    params.validationMaxP95ResidualMm = pointCloudSettings.validationMaxP95ResidualMm;
    params.validationResidualInlierThresholdMm = pointCloudSettings.validationResidualInlierThresholdMm;
    params.validationMinResidualInlierRatio = pointCloudSettings.validationMinResidualInlierRatio;
    params.validationKeyPointEnabled = pointCloudSettings.validationKeyPointEnabled;
    params.validationMinKeyPointCount = pointCloudSettings.validationMinKeyPointCount;
    params.validationMinCornerCount = pointCloudSettings.validationMinCornerCount;
    params.validationMinSegmentLengthMm = pointCloudSettings.validationMinSegmentLengthMm;
    params.validationOutputEnabled = pointCloudSettings.validationOutputEnabled;
    params.validationMinOutputPointCount = pointCloudSettings.validationMinOutputPointCount;
    params.validationMinOutputLengthRatio = pointCloudSettings.validationMinOutputLengthRatio;
    return params;
}

QString MeasureThenWeldService::MethodBaseTrackFileName(PointCloudProcessingConfig::Mode mode)
{
    switch (mode)
    {
    case PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet:
        return QString::fromLatin1(METHOD_TRACK_SDK_CLASS_FILE_NAME);
    case PointCloudProcessingConfig::Mode::SdkBaseWeldFit:
        return QString::fromLatin1(METHOD_TRACK_SDK_BASE_FILE_NAME);
    case PointCloudProcessingConfig::Mode::CloudFit:
        return QString::fromLatin1(METHOD_TRACK_POINT_BASE_FILE_NAME);
    case PointCloudProcessingConfig::Mode::LegacyLaserPath:
    default:
        return QString::fromLatin1(METHOD_TRACK_POINT_LASER_FILE_NAME);
    }
}

bool MeasureThenWeldService::LoadCompPreviewRawCloud(
    const QString& laserDir,
    QVector<CompPreviewPoint>& points,
    QString& error,
    QString* sourceDescription) const
{
    points.clear();
    if (sourceDescription != nullptr)
    {
        sourceDescription->clear();
    }
    QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("目录不存在：%1").arg(laserDir);
        return false;
    }

    // "原始数据"显示当前方法自己的基础焊道文件（处理成功时落盘）；
    // 未生成（该目录还没按当前方法处理过）时回退相机目标点轨迹。
    const PointCloudProcessingConfig::Settings settings = PointCloudProcessingConfig::Load();
    const QString methodFileName = MethodBaseTrackFileName(settings.mode);
    const QString methodFilePath = dir.filePath(methodFileName);
    QString methodError;
    if (QFileInfo::exists(methodFilePath)
        && LoadSampledPointFile(methodFilePath, std::numeric_limits<int>::max(), points, methodError))
    {
        if (sourceDescription != nullptr)
        {
            *sourceDescription = QString("%1（%2）")
                .arg(PointCloudProcessingConfig::ModeDisplayName(settings.mode))
                .arg(methodFileName);
        }
        return true;
    }

    QVector<RobotCalculation::IndexedPoint3D> rawPoints;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(dir.filePath(RAW_LASER_FILE_NAME), rawPoints, &error))
    {
        return false;
    }
    points.reserve(rawPoints.size());
    for (const RobotCalculation::IndexedPoint3D& raw : rawPoints)
    {
        CompPreviewPoint point;
        point.x = raw.point.x();
        point.y = raw.point.y();
        point.z = raw.point.z();
        points.push_back(point);
    }
    if (points.isEmpty())
    {
        error = QString("未从 %1 解析到原始点。").arg(RAW_LASER_FILE_NAME);
        return false;
    }
    if (sourceDescription != nullptr)
    {
        *sourceDescription = QStringLiteral("相机目标点轨迹（该方法基础焊道未生成）");
    }
    return true;
}

MeasureThenWeldService::CompPreviewStages MeasureThenWeldService::ComputeCompPreviewStages(
    const QString& robotName,
    const QVector<CompPreviewPoint>& baseline,
    const CompPreviewEditValues& currentEdits,
    const CompPreviewEditValues& savedEdits,
    bool includePoseArrows) const
{
    CompPreviewStages stages;
    if (baseline.isEmpty())
    {
        stages.error = QStringLiteral("没有可用的基准焊道点。");
        return stages;
    }

    static const char* const kSegmentKinds[4] = { "low_platform", "rising_edge", "high_platform", "falling_edge" };
    const auto buildPoseSlots = [&](const CompPreviewEditValues& edits)
    {
        // 注意：局部变量不能叫 slots（Qt 宏）。
        std::vector<WeldPosePreset::PoseCompSlot> poseSlots(4);
        for (int slotIndex = 0; slotIndex < 4; ++slotIndex)
        {
            poseSlots[slotIndex].segmentKind = QString::fromLatin1(kSegmentKinds[slotIndex]);
            poseSlots[slotIndex].poseRx = edits.poseRx[slotIndex];
            poseSlots[slotIndex].poseRy = edits.poseRy[slotIndex];
            poseSlots[slotIndex].poseRz = edits.poseRz[slotIndex];
            poseSlots[slotIndex].compX = edits.compX[slotIndex];
            poseSlots[slotIndex].compY = edits.compY[slotIndex];
            poseSlots[slotIndex].compZ = edits.compZ[slotIndex];
            poseSlots[slotIndex].validReference = true;
        }
        return poseSlots;
    };
    const std::vector<WeldPosePreset::PoseCompSlot> currentPoseSlots = buildPoseSlots(currentEdits);
    const std::vector<WeldPosePreset::PoseCompSlot> savedPoseSlots = buildPoseSlots(savedEdits);

    // 阶段「姿态补偿」：基准(_WeldPose_2mm)已烘焙扫描时保存的姿态补偿，
    // 按 delta = 当前补偿位移 − 已保存补偿位移 叠加，当前=已保存时与文件一致；
    // 位移纯加性，等价于用当前值重跑姿态生成。
    QVector<WeldPoseFileRecord> records;
    records.reserve(baseline.size());
    for (const CompPreviewPoint& base : baseline)
    {
        WeldPoseFileRecord record;
        record.weldIndex = base.weldIndex;
        record.rawIndex = base.rawIndex;
        record.point = Eigen::Vector3d(base.x, base.y, base.z);
        record.rx = base.rx;
        record.ry = base.ry;
        record.rz = base.rz;
        record.bx = base.bx;
        record.by = base.by;
        record.bz = base.bz;
        record.segmentKind = base.segmentKind;
        record.pointType = base.pointType;

        const Eigen::Vector3d basePoint = record.point;
        const Eigen::Vector3d withCurrent = ApplyPoseCompToPoint(
            currentPoseSlots, currentEdits.poseMatchMode, currentEdits.poseMatchMaxErrorDeg,
            currentEdits.robotType, basePoint, base.rx, base.ry, base.rz, base.segmentKind);
        const Eigen::Vector3d withSaved = ApplyPoseCompToPoint(
            savedPoseSlots, savedEdits.poseMatchMode, savedEdits.poseMatchMaxErrorDeg,
            savedEdits.robotType, basePoint, base.rx, base.ry, base.rz, base.segmentKind);
        record.point = basePoint + (withCurrent - basePoint) - (withSaved - basePoint);
        records.push_back(record);
    }

    const auto snapshotRecords = [](const QVector<WeldPoseFileRecord>& sourceRecords)
    {
        QVector<CompPreviewPoint> out;
        out.reserve(sourceRecords.size());
        for (const WeldPoseFileRecord& record : sourceRecords)
        {
            CompPreviewPoint point;
            point.x = record.point.x();
            point.y = record.point.y();
            point.z = record.point.z();
            point.rx = record.rx;
            point.ry = record.ry;
            point.rz = record.rz;
            point.bx = record.bx;
            point.by = record.by;
            point.bz = record.bz;
            point.weldIndex = record.weldIndex;
            point.rawIndex = record.rawIndex;
            point.segmentKind = record.segmentKind;
            point.pointType = record.pointType;
            out.push_back(point);
        }
        return out;
    };
    stages.poseComp = snapshotRecords(records);

    QVector<Eigen::Vector3d> basePoints;
    basePoints.reserve(records.size());
    for (const WeldPoseFileRecord& record : records)
    {
        basePoints.push_back(record.point);
    }
    const Eigen::Vector3d seamDir = ResolveOverallHorizontalWeldDirection(basePoints);
    const Eigen::Vector3d gunDir = HorizontalUnitOrZero(Eigen::Vector3d::UnitZ().cross(seamDir));

    // 阶段「焊道补偿」：真实预设 + 当前编辑槽位覆盖（槽位段类用配置真实值，匹配/回退与下发一致）。
    const T_PRECISE_MEASURE_PARAM measureParam = BuildMeasureWeldParamShell(robotName);
    WeldPosePreset preset = LoadWeldPosePreset(measureParam);
    // 工艺区域试调覆盖（仅预览联动，不落盘）：圆弧过渡启用/半径 + 实际焊道点间距。
    if (currentEdits.processOverrideValid)
    {
        preset.cornerArcRadiusMm = (currentEdits.arcEnabled && currentEdits.arcRadiusMm > 0.0)
            ? std::max(2.0, currentEdits.arcRadiusMm)
            : 0.0;
        preset.finalWeldStepFromProcessMm = currentEdits.processFinalStepMm;
    }
    preset.seamCompSlots.assign(4, WeldPosePreset::SeamCompSlot());
    for (int slotIndex = 0; slotIndex < 4; ++slotIndex)
    {
        const QString segmentKind = currentEdits.seamSegmentKind[slotIndex].trimmed().isEmpty()
            ? QString::fromLatin1(kSegmentKinds[slotIndex])
            : currentEdits.seamSegmentKind[slotIndex];
        preset.seamCompSlots[slotIndex].segmentKind = segmentKind;
        preset.seamCompSlots[slotIndex].weldZComp = currentEdits.weldZComp[slotIndex];
        preset.seamCompSlots[slotIndex].weldGunDirComp = currentEdits.weldGunDirComp[slotIndex];
        preset.seamCompSlots[slotIndex].weldSeamDirComp = currentEdits.weldSeamDirComp[slotIndex];
    }
    WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
    stages.seamComp = snapshotRecords(records);

    // 阶段「圆弧过渡」：完整后处理（端点/自交裁剪、拐点恢复、加密、圆弧过渡、锐角裁剪、平滑、重编号）。
    const QVector<WeldPoseFileRecord> recordsBeforeTrim = records;
    FinalizeSeamCompedWeldPoseRecords(preset, recordsBeforeTrim, records, compStats);
    if (records.isEmpty())
    {
        records = recordsBeforeTrim;  // 后处理裁空则回退显示纯补偿平移结果
    }
    stages.arc = snapshotRecords(records);

    // 阶段「实际焊道」：按点间距最终抽样（首尾+拐点必留，沿弧长≥间距取点），
    // 复用下发管线同一个抽样函数 = 机器人逐点执行的轨迹。
    const double actualStepMm = preset.finalWeldStepFromProcessMm > 0.0
        ? preset.finalWeldStepFromProcessMm
        : measureParam.dFinalWeldTrajectoryStepMm;
    const QVector<WeldPoseFileRecord> actualRecords =
        SampleFinalWeldTrajectoryRecords(records, NormalizeFinalWeldTrajectorySampleStepMm(actualStepMm));
    stages.actual = snapshotRecords(actualRecords);

    // 方向箭头：质心 + 自适应长度。
    double sum[3] = { 0.0, 0.0, 0.0 };
    double minv[3] = { 1e300, 1e300, 1e300 };
    double maxv[3] = { -1e300, -1e300, -1e300 };
    for (const CompPreviewPoint& point : stages.poseComp)
    {
        const double coord[3] = { point.x, point.y, point.z };
        for (int axis = 0; axis < 3; ++axis)
        {
            sum[axis] += coord[axis];
            minv[axis] = std::min(minv[axis], coord[axis]);
            maxv[axis] = std::max(maxv[axis], coord[axis]);
        }
    }
    const double count = static_cast<double>(stages.poseComp.size());
    const double cx = sum[0] / count;
    const double cy = sum[1] / count;
    const double cz = sum[2] / count;
    const double span = std::max({ maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2], 10.0 });
    const double arrowLen = std::clamp(span * 0.15, 8.0, 80.0);
    const auto addArrow = [&stages](double ox, double oy, double oz, double vx, double vy, double vz,
        const QString& label, int colorId)
    {
        CompPreviewArrow arrow;
        arrow.origin[0] = ox; arrow.origin[1] = oy; arrow.origin[2] = oz;
        arrow.vector[0] = vx; arrow.vector[1] = vy; arrow.vector[2] = vz;
        arrow.label = label;
        arrow.colorId = colorId;
        arrow.doubleHeaded = true;
        stages.arrows.push_back(arrow);
    };
    addArrow(cx, cy, cz, 0.0, 0.0, arrowLen, QStringLiteral("Z向+"), 0);
    addArrow(cx, cy, cz, gunDir.x() * arrowLen, gunDir.y() * arrowLen, gunDir.z() * arrowLen, QStringLiteral("枪反向+"), 1);
    addArrow(cx, cy, cz, seamDir.x() * arrowLen, seamDir.y() * arrowLen, seamDir.z() * arrowLen, QStringLiteral("焊道方向+"), 2);

    if (includePoseArrows)
    {
        int repIndex = 0;
        double bestDistanceSq = std::numeric_limits<double>::max();
        for (int index = 0; index < baseline.size(); ++index)
        {
            const double dx = baseline[index].x - cx;
            const double dy = baseline[index].y - cy;
            const double dz = baseline[index].z - cz;
            const double distanceSq = dx * dx + dy * dy + dz * dz;
            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                repIndex = index;
            }
        }
        const Eigen::Matrix3d rotation = RobotPoseTransform::RotationFromAnglesDeg(
            baseline[repIndex].rx, baseline[repIndex].ry, baseline[repIndex].rz, currentEdits.robotType);
        const Eigen::Vector3d toolX = rotation.col(0) * arrowLen;
        const Eigen::Vector3d toolY = rotation.col(1) * arrowLen;
        const Eigen::Vector3d toolZ = rotation.col(2) * arrowLen;
        addArrow(cx, cy, cz, toolX.x(), toolX.y(), toolX.z(), QStringLiteral("X补偿+"), 3);
        addArrow(cx, cy, cz, toolY.x(), toolY.y(), toolY.z(), QStringLiteral("Y补偿+"), 4);
        addArrow(cx, cy, cz, toolZ.x(), toolZ.y(), toolZ.z(), QStringLiteral("Z补偿+"), 5);
    }

    stages.ok = true;
    return stages;
}
