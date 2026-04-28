#include "MeasureThenWeldService.h"

#include "CameraFrameCache.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "groove/framebuffer.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QStringConverter>
#include <QTextStream>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <thread>

namespace
{
constexpr int FANUC_MOTION_STATE_REG = 93;
constexpr double FANUC_WELD_PATH_SPEED_MM_PER_MIN = 400.0;
constexpr double FANUC_SAFE_MOVE_SPEED_MM_PER_MIN = 1000.0;
constexpr double WELD_SAFE_OFFSET_DISTANCE_MM = 30.0;
constexpr double DEFAULT_CAMERA_READ_FPS = 100.0;
constexpr qint64 ROBOT_SAMPLE_INTERVAL_MS = 50;
constexpr qint64 CAMERA_ROBOT_MATCH_TAIL_WAIT_MS = 500;
constexpr qint64 CAMERA_ROBOT_MATCH_TAIL_POLL_MS = 10;
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";

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
    QString seamKind = "CorrugatedPlate";
    double rx = 0.0;
    double ry = 0.0;
    double gunToolBaseRz = 180.0;
    double poseMatchMaxErrorDeg = 5.0;
    double cornerTransitionLeadDistance = 10.0;
    double weldStartSkipDistance = 10.0;
    double weldEndSkipDistance = 10.0;
    std::vector<PoseCompSlot> poseCompSlots;
    std::vector<SeamCompSlot> seamCompSlots;
    bool weldLineFromIni = false;
    bool poseCompFromIni = false;
    bool seamCompFromIni = false;
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

double NormalizeLineAxisDeviationFromYAxis(double directionDeg)
{
    double deviation = directionDeg - 90.0;
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

double AngleDistanceDeg(double angleDeg, double referenceDeg)
{
    return std::abs(NormalizeAngleNear(angleDeg, referenceDeg) - referenceDeg);
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

Eigen::Matrix3d RotXDeg(double angleDeg)
{
    const double angleRad = angleDeg * M_PI / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    matrix(1, 1) = c;
    matrix(1, 2) = -s;
    matrix(2, 1) = s;
    matrix(2, 2) = c;
    return matrix;
}

Eigen::Matrix3d RotYDeg(double angleDeg)
{
    const double angleRad = angleDeg * M_PI / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    matrix(0, 0) = c;
    matrix(0, 2) = s;
    matrix(2, 0) = -s;
    matrix(2, 2) = c;
    return matrix;
}

Eigen::Matrix3d RotZDeg(double angleDeg)
{
    const double angleRad = angleDeg * M_PI / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    matrix(0, 0) = c;
    matrix(0, 1) = -s;
    matrix(1, 0) = s;
    matrix(1, 1) = c;
    return matrix;
}

Eigen::Matrix3d FanucRotDeg(double rxDeg, double ryDeg, double rzDeg)
{
    return RotZDeg(rzDeg) * RotYDeg(ryDeg) * RotXDeg(rxDeg);
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
        if (index == 0)
        {
            seamCompCollection[index].name = "波纹板";
            seamCompCollection[index].segmentKind = "CorrugatedPlate";
        }
        else
        {
            seamCompCollection[index].name = QString("焊道类型%1").arg(index);
            seamCompCollection[index].segmentKind = QString("SeamType%1").arg(index);
        }
    }
}

WeldPosePreset LoadWeldPosePreset(const T_PRECISE_MEASURE_PARAM& param)
{
    WeldPosePreset preset;
    preset.rx = param.tStartPos.dRX;
    preset.ry = param.tStartPos.dRY;
    preset.weldLineSectionName = "WeldNormalParam0";
    preset.weldLineFilePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldLineParam.ini").arg(QString::fromStdString(param.sRobotName)));
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

    if (!QFileInfo::exists(preset.weldLineFilePath))
    {
        goto load_pose_comp;
    }

    {
        COPini ini;
        if (ini.SetFileName(preset.weldLineFilePath.toStdString()))
        {
            ini.SetSectionName(preset.weldLineSectionName.toStdString());
            double rx = preset.rx;
            double ry = preset.ry;
            double cornerTransitionLeadDistance = preset.cornerTransitionLeadDistance;
            double weldStartSkipDistance = preset.weldStartSkipDistance;
            double weldEndSkipDistance = preset.weldEndSkipDistance;
            const bool hasNormalRx = TryReadIniDouble(ini, "NormalWeldRx", rx);
            const bool hasNormalRy = TryReadIniDouble(ini, "NormalWeldRy", ry);
            TryReadIniDouble(ini, "CornerTransitionLeadDis", cornerTransitionLeadDistance);
            TryReadIniDouble(ini, "WeldStartSkipDis", weldStartSkipDistance);
            TryReadIniDouble(ini, "WeldEndSkipDis", weldEndSkipDistance);
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
            preset.weldLineFromIni = true;
        }
    }

load_pose_comp:
    if (QFileInfo::exists(preset.poseCompFilePath))
    {
        COPini poseIni;
        if (poseIni.SetFileName(preset.poseCompFilePath.toStdString()))
        {
            int poseCompCount = static_cast<int>(preset.poseCompSlots.size());
            poseIni.SetSectionName("ALLWeldPoseComp");
            poseIni.ReadString(false, "PoseCompCount", &poseCompCount);
            TryReadIniDouble(poseIni, "PoseMatchMaxErrorDeg", preset.poseMatchMaxErrorDeg);
            preset.poseMatchMaxErrorDeg = std::max(0.0, preset.poseMatchMaxErrorDeg);
            preset.poseCompSlots.assign(std::max(0, poseCompCount), WeldPosePreset::PoseCompSlot());
            InitializeDefaultPoseCompSlots(preset.poseCompSlots);
            for (int index = 0; index < static_cast<int>(preset.poseCompSlots.size()); ++index)
            {
                WeldPosePreset::PoseCompSlot& slot = preset.poseCompSlots[index];
                poseIni.SetSectionName(QString("WeldPoseComp%1").arg(index).toStdString());

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

    if (QFileInfo::exists(preset.seamCompFilePath))
    {
        COPini seamIni;
        if (seamIni.SetFileName(preset.seamCompFilePath.toStdString()))
        {
            int seamCompCount = static_cast<int>(preset.seamCompSlots.size());
            seamIni.SetSectionName("ALLWeldSeamComp");
            seamIni.ReadString(false, "SeamCompCount", &seamCompCount);
            preset.seamCompSlots.assign(std::max(0, seamCompCount), WeldPosePreset::SeamCompSlot());
            InitializeDefaultSeamCompSlots(preset.seamCompSlots);
            for (int index = 0; index < static_cast<int>(preset.seamCompSlots.size()); ++index)
            {
                WeldPosePreset::SeamCompSlot& slot = preset.seamCompSlots[index];
                seamIni.SetSectionName(QString("WeldSeamComp%1").arg(index).toStdString());

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

    if (QFileInfo::exists(preset.robotParaPath))
    {
        COPini robotIni;
        if (robotIni.SetFileName(preset.robotParaPath.toStdString()))
        {
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
    constexpr auto transitionSuffix = "_transition";
    if (segmentKind.endsWith(transitionSuffix, Qt::CaseInsensitive))
    {
        segmentKind.chop(static_cast<int>(std::strlen(transitionSuffix)));
    }
    return segmentKind;
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

    // Backward compatibility: old configs had a single CorrugatedPlate slot
    // applied to the whole weld seam.
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

bool TryBuildWeldSafeCoors(
    const QVector<WeldPoseFileRecord>& records,
    int pointIndex,
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

    const Eigen::Vector3d gunDirection = HorizontalUnitOrZero(
        FanucRotDeg(anchor.rx, anchor.ry, anchor.rz) * Eigen::Vector3d::UnitY());
    if (gunDirection.head<2>().norm() > 1e-9
        && lateralDirection.head<2>().dot(gunDirection.head<2>()) < 0.0)
    {
        lateralDirection = -lateralDirection;
    }

    const Eigen::Vector3d safeOffsetDirection =
        (Eigen::Vector3d::UnitZ() + lateralDirection).normalized();
    const Eigen::Vector3d safePoint =
        anchor.point + safeOffsetDirection * WELD_SAFE_OFFSET_DISTANCE_MM;

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
    QString& error)
{
    moveInfos.clear();
    moveInfos.reserve(static_cast<size_t>(records.size()));

    int externalAxisPointCount = 0;
    for (const WeldPoseFileRecord& record : records)
    {
        if (std::abs(record.bx) > 1e-6 || std::abs(record.by) > 1e-6 || std::abs(record.bz) > 1e-6)
        {
            ++externalAxisPointCount;
        }

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
        moveInfo.tSpeed = T_ROBOT_MOVE_SPEED(linearSpeedMmPerSec, 0.0, 0.0);
        moveInfo.nMoveDevice = 0;
        moveInfo.nTrackNo = 0;
        moveInfo.adBasePosVar[0] = record.bx;
        moveInfo.adBasePosVar[1] = record.by;
        moveInfo.adBasePosVar[2] = record.bz;
        moveInfos.push_back(moveInfo);
    }

    if (externalAxisPointCount > 0)
    {
        error = QString("焊接姿态文件包含 %1 个外部轴点位，但当前多点 TP 下发只支持 GP1 六轴点位，请先确认 BX/BY/BZ 是否应为 0。")
            .arg(externalAxisPointCount);
        moveInfos.clear();
        return false;
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
    QSet<QString> usedSlots;
};

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
            const Eigen::Vector3d gunDirection = HorizontalUnitOrZero(
                FanucRotDeg(record.rx, record.ry, record.rz) * Eigen::Vector3d::UnitY());
            Eigen::Vector3d lateralDirection = HorizontalUnitOrZero(
                Eigen::Vector3d::UnitZ().cross(seamDirection));
            if (lateralDirection.head<2>().norm() > 1e-9)
            {
                if (gunDirection.head<2>().norm() > 1e-9
                    && lateralDirection.head<2>().dot(gunDirection.head<2>()) < 0.0)
                {
                    lateralDirection = -lateralDirection;
                }

                record.point += lateralDirection * seamCompSlot->weldGunDirComp;
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
        RobotCalculation::LowerWeldPointType beginType = RobotCalculation::LowerWeldPointType::Normal;
        RobotCalculation::LowerWeldPointType endMarkerType = RobotCalculation::LowerWeldPointType::Normal;
        QString kind;
        double fixedRz = 0.0;
        double directionDeg = 0.0;
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

    std::vector<SegmentInfo> segments;
    double previousSegmentRz = preset.gunToolBaseRz;
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
            const double axisDeviationDeg = NormalizeLineAxisDeviationFromYAxis(segmentDirectionDeg);
            const double baseRz = preset.gunToolBaseRz + axisDeviationDeg;
            segmentRz = NormalizeAngleNear(baseRz, previousSegmentRz);
        }

        segment.fixedRz = NormalizeAngleToFanucRange(segmentRz);
        segment.distanceToEnd.resize(segment.end - segment.begin + 1);
        double accumulatedDistance = 0.0;
        segment.distanceToEnd[segment.end - segment.begin] = 0.0;
        for (int index = segment.end - 1; index >= segment.begin; --index)
        {
            accumulatedDistance += (result.points[index + 1].point - result.points[index].point).norm();
            segment.distanceToEnd[index - segment.begin] = accumulatedDistance;
        }

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
        appendLog(QString("姿态匹配最大误差阈值=%1 deg，超过该阈值则该点不做姿态补偿。")
            .arg(preset.poseMatchMaxErrorDeg, 0, 'f', 3));
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
    QVector<double> distanceFromStart(result.points.size(), 0.0);
    for (int index = weldBeginCandidate + 1; index <= weldEndCandidate; ++index)
    {
        distanceFromStart[index] = distanceFromStart[index - 1]
            + (result.points[index].point - result.points[index - 1].point).norm();
    }

    int weldStartIndex = weldBeginCandidate;
    if (preset.weldStartSkipDistance > 1e-6)
    {
        for (int index = weldBeginCandidate; index <= weldEndCandidate; ++index)
        {
            if (distanceFromStart[index] >= preset.weldStartSkipDistance)
            {
                weldStartIndex = index;
                break;
            }
        }
    }

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

    while (true)
    {
        const int segmentIndex = findSegmentIndex(weldStartIndex);
        if (segmentIndex < 0)
        {
            break;
        }
        const SegmentInfo& segment = segments[segmentIndex];
        const bool hasNextSegment = segmentIndex + 1 < static_cast<int>(segments.size());
        if (!hasNextSegment || segment.transitionBegin == std::numeric_limits<int>::max())
        {
            break;
        }
        if (weldStartIndex < segment.transitionBegin)
        {
            break;
        }
        weldStartIndex = segments[segmentIndex + 1].begin;
    }

    QVector<double> distanceFromEnd(result.points.size(), 0.0);
    for (int index = weldEndCandidate - 1; index >= weldBeginCandidate; --index)
    {
        distanceFromEnd[index] = distanceFromEnd[index + 1]
            + (result.points[index + 1].point - result.points[index].point).norm();
    }

    int weldEndIndex = weldEndCandidate;
    if (preset.weldEndSkipDistance > 1e-6)
    {
        for (int index = weldEndCandidate; index >= weldBeginCandidate; --index)
        {
            if (distanceFromEnd[index] >= preset.weldEndSkipDistance)
            {
                weldEndIndex = index;
                break;
            }
        }
    }

    while (true)
    {
        const int segmentIndex = findSegmentIndex(weldEndIndex);
        if (segmentIndex < 0)
        {
            break;
        }
        const SegmentInfo& segment = segments[segmentIndex];
        const bool hasNextSegment = segmentIndex + 1 < static_cast<int>(segments.size());
        if (!hasNextSegment || segment.transitionBegin == std::numeric_limits<int>::max())
        {
            break;
        }
        if (weldEndIndex < segment.transitionBegin)
        {
            break;
        }
        weldEndIndex = segment.transitionBegin - 1;
        if (weldEndIndex >= segment.begin)
        {
            break;
        }
        if (segmentIndex == 0)
        {
            break;
        }
        weldEndIndex = segments[segmentIndex - 1].end;
    }

    if (weldStartIndex > weldEndIndex)
    {
        if (appendLog)
        {
            appendLog(QString("起终点跳过后已无有效焊接点：StartSkip=%1mm, EndSkip=%2mm")
                .arg(preset.weldStartSkipDistance, 0, 'f', 3)
                .arg(preset.weldEndSkipDistance, 0, 'f', 3));
        }
        return lines;
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
            appendLog(QString("焊道姿态段 %1: 点[%2-%3], 固定RZ=%4 deg, RX=%5 deg, RY=%6 deg, 焊道种类=%7, 过渡起点=%8, 起点跳过=%9 mm, 终点跳过=%10 mm, Z补偿=%11 mm, 枪向补偿=%12 mm, 焊道方向补偿=%13 mm")
                .arg(segment.kind)
                .arg(result.points[segment.begin].index)
                .arg(result.points[segment.end].index)
                .arg(segment.fixedRz, 0, 'f', 3)
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

    QVector<WeldPoseFileRecord> records;
    records.reserve(weldEndIndex - weldStartIndex + 1);

    int weldIndex = 1;
    for (int pointIndex = weldStartIndex; pointIndex <= weldEndIndex; ++pointIndex)
    {
        const int segmentIndex = findSegmentIndex(pointIndex);
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
            && segment.transitionBegin != std::numeric_limits<int>::max()
            && pointIndex >= segment.transitionBegin;

        double pointRz = segment.fixedRz;
        if (inTransition && preset.cornerTransitionLeadDistance > 1e-6)
        {
            const double remainingDistance = segment.distanceToEnd[pointIndex - segment.begin];
            const double transitionRatio = 1.0
                - (remainingDistance / preset.cornerTransitionLeadDistance);
            pointRz = segment.fixedRz
                + (nextSegmentRz - segment.fixedRz) * std::clamp(transitionRatio, 0.0, 1.0);
        }
        pointRz = NormalizeAngleToFanucRange(pointRz);

        double pointRx = preset.rx;
        double pointRy = preset.ry;
        Eigen::Vector3d point = result.points[pointIndex].point;
        const int poseCompSlotIndex = findNearestPoseCompSlot(pointRx, pointRy, pointRz);
        if (poseCompSlotIndex >= 0)
        {
            const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[poseCompSlotIndex];
            const Eigen::Vector3d poseCompLocal(slot.compX, slot.compY, slot.compZ);
            if (poseCompLocal.norm() > 1e-9)
            {
                point += FanucRotDeg(pointRx, pointRy, pointRz) * poseCompLocal;
            }
        }

        WeldPoseFileRecord record;
        record.weldIndex = weldIndex++;
        record.rawIndex = result.points[pointIndex].index;
        record.point = point;
        record.rx = pointRx;
        record.ry = pointRy;
        record.rz = pointRz;
        record.bx = param.tStartPos.dBX;
        record.by = param.tStartPos.dBY;
        record.bz = param.tStartPos.dBZ;
        record.pointType = RobotCalculation::LowerWeldPointTypeName(result.points[pointIndex].type);
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
}

bool MeasureThenWeldService::LoadPresetParam(FANUCRobotCtrl* pFanucDriver, T_PRECISE_MEASURE_PARAM& param, QString& error) const
{
    if (pFanucDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    param = T_PRECISE_MEASURE_PARAM();
    param.sRobotName = pFanucDriver->m_sRobotName.empty() ? "RobotA" : pFanucDriver->m_sRobotName;

    const QString robotName = QString::fromStdString(param.sRobotName);
    const QString iniPath = RobotDataHelper::PreciseMeasureParamPath(robotName);
    if (!QFileInfo::exists(iniPath))
    {
        error = QString("未找到参数文件：%1").arg(iniPath);
        return false;
    }

    param.sIniFilePath = iniPath.toLocal8Bit().constData();

    COPini ini;
    if (!ini.SetFileName(param.sIniFilePath))
    {
        error = QString("打开参数文件失败：%1").arg(iniPath);
        return false;
    }

    int useNo = 0;
    ini.SetSectionName("ALLPostion");
    ini.ReadString(false, "UsePostionNo", &useNo);
    param.sSectionName = "Postion" + std::to_string(useNo);

    ini.SetSectionName(param.sSectionName);
    ini.ReadString(false, "ScanSpeed", &param.dScanSpeed);
    ini.ReadString(false, "RunSpeed", &param.dRunSpeed);
    ini.ReadString(false, "CameraReadFps", &param.dCameraReadFps);
    ini.ReadString(false, "CameraTimeOffsetMs", &param.dCameraTimeOffsetMs);
    ini.ReadString(false, "dAcc", &param.dAcc);
    ini.ReadString(false, "dDec", &param.dDec);

    if (!std::isfinite(param.dCameraReadFps) || param.dCameraReadFps <= 0.0)
    {
        param.dCameraReadFps = DEFAULT_CAMERA_READ_FPS;
    }
    if (!std::isfinite(param.dCameraTimeOffsetMs))
    {
        param.dCameraTimeOffsetMs = 0.0;
    }

    if (!ReadPulseList(ini, "StartSafePulseNum", "StartSafePulse", param.vtStartSafePulse, error)
        || !ReadCoors(ini, "StartPos", param.tStartPos, error)
        || !ReadCoors(ini, "EndPos", param.tEndPos, error)
        || !ReadPulseList(ini, "EndSafePulseNum", "EndSafePulse", param.vtEndSafePulse, error))
    {
        return false;
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
    const int ok = ini.ReadString(prefix + ".", "", coors);
    if (ok <= 0)
    {
        error = QString("读取直角坐标失败：%1，请在“精测量数据修改”里重新示教并保存扫描起点/终点。")
            .arg(QString::fromStdString(prefix));
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
        count = 1;
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

bool MeasureThenWeldService::MovePulseAndWait(FANUCRobotCtrl* pFanucDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    if (setFlowStep)
    {
        setFlowStep(QString("正在运动：%1").arg(name));
    }
    if (appendLog)
    {
        appendLog(QString("开始运动：%1").arg(name));
    }

    const bool moveOk = pFanucDriver->MoveByJob(pulse, T_ROBOT_MOVE_SPEED(speed, 0.0, 0.0), pFanucDriver->m_nExternalAxleType, "MOVJ");
    if (!moveOk)
    {
        if (appendLog)
        {
            appendLog(QString("运动失败：%1").arg(name));
        }
        return false;
    }

    int lastState = 0;
    const bool doneOk = pFanucDriver->WaitStateDone(FANUC_MOTION_STATE_REG, 1, 10, 20, 3000, 120000, 100, &lastState);
    if (appendLog)
    {
        appendLog(QString("运动结束：%1, R[%2]=%3, WaitStateDone=%4")
            .arg(name)
            .arg(FANUC_MOTION_STATE_REG)
            .arg(lastState)
            .arg(doneOk ? 1 : 0));
    }
    return doneOk;
}

bool MeasureThenWeldService::MovePulseListAndWait(FANUCRobotCtrl* pFanucDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    for (size_t index = 0; index < pulses.size(); ++index)
    {
        if (!MovePulseAndWait(pFanucDriver, pulses[index], speed, QString("%1[%2]").arg(name).arg(index), appendLog, setFlowStep))
        {
            return false;
        }
    }
    return true;
}

bool MeasureThenWeldService::MoveCoorsAndWait(FANUCRobotCtrl* pFanucDriver, const T_ROBOT_COORS& coors, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    const double fanucSpeed = FanucLinearSpeedMmPerSecFromConfig(speed, 1.0);
    if (setFlowStep)
    {
        setFlowStep(QString("正在直线运动：%1").arg(name));
    }
    if (appendLog)
    {
        appendLog(QString("开始直线运动：%1，配置速度= %2 mm/min，下发速度= %3 mm/sec")
            .arg(name)
            .arg(speed, 0, 'f', 3)
            .arg(fanucSpeed, 0, 'f', 3));
    }

    const bool moveOk = pFanucDriver->MoveByJob(coors, T_ROBOT_MOVE_SPEED(fanucSpeed, 0.0, 0.0), pFanucDriver->m_nExternalAxleType, "MOVL");
    if (!moveOk)
    {
        if (appendLog)
        {
            appendLog(QString("直线运动失败：%1").arg(name));
        }
        return false;
    }

    int lastState = 0;
    const bool doneOk = pFanucDriver->WaitStateDone(FANUC_MOTION_STATE_REG, 1, 10, 20, 3000, 120000, 100, &lastState);
    if (appendLog)
    {
        appendLog(QString("直线运动结束：%1, R[%2]=%3, WaitStateDone=%4")
            .arg(name)
            .arg(FANUC_MOTION_STATE_REG)
            .arg(lastState)
            .arg(doneOk ? 1 : 0));
    }
    return doneOk;
}

bool MeasureThenWeldService::ScanMoveAndCollect(FANUCRobotCtrl* pFanucDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    const double fanucScanSpeed = FanucLinearSpeedMmPerSecFromConfig(param.dScanSpeed, 1.0);
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

    CameraFrameCache::Instance().Start();
    CameraFrameCache::Instance().Clear();

    std::vector<udpDataShow> scanCameraFrames;
    std::vector<TimestampedCameraPoint> cameraSamples;
    std::vector<TimestampedCameraPoint> matchedCameraSamples;
    std::vector<RobotCalculation::TimestampedRobotPose> robotSamples;
    QVector<RobotCalculation::IndexedPoint3D> laserFitInput;
    scanCameraFrames.reserve(10000);
    cameraSamples.reserve(10000);
    matchedCameraSamples.reserve(10000);
    robotSamples.reserve(1000);
    laserFitInput.reserve(10000);
    long long lastRobotMonitorMs = std::numeric_limits<long long>::min();
    bool passiveRobotSamplingActive = false;
    std::size_t nextPendingCameraIndex = 0;
    int droppedHeadCameraCount = 0;
    int invalidCameraTimestampCount = 0;
    int cameraBeforeRobotTimeBaseCount = 0;
    int cameraTimestampBackwardsCount = 0;
    int cameraTimestampJumpCount = 0;
    qint64 lastCameraRawTimestampUs = 0;
    qint64 maxCameraRawDeltaUs = 0;
    const qint64 cameraTimestampJumpWarnUs = std::max<qint64>(50000, cameraReadIntervalMs * 4000);
    bool hasCameraToRobotTimeOffset = false;
    qint64 cameraToRobotTimeOffsetUs = 0;
    qint64 firstCameraRawTimestampUs = 0;
    qint64 firstRobotTimestampUs = 0;
    bool hasCameraTimeBaseRobotTimestamp = false;
    qint64 cameraTimeBaseRobotTimestampUs = 0;
    std::uint64_t scanStartCameraSequence = 0;
    std::uint64_t scanEndCameraSequence = 0;
    std::uint64_t lastPulledCameraSequence = 0;

    auto appendCameraFrame = [
        &cameraSamples,
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
        &cameraTimeBaseRobotTimestampUs](const udpDataShow& frame)
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

            cameraSamples.push_back(TimestampedCameraPoint{
                static_cast<int>(cameraSamples.size()) + 1,
                rawTimestampUs,
                rawDeltaUs,
                rawTimestampUs + cameraToRobotTimeOffsetUs + cameraTimeOffsetUs,
                Eigen::Vector3d(frame.targetPoint.x, frame.targetPoint.y, frame.targetPoint.z),
                frame.errorMessage });
        };

    auto resolveReadyCameraSamples = [&cameraSamples, &nextPendingCameraIndex, &robotSamples, &matchedCameraSamples, &droppedHeadCameraCount]()
        {
            ResolveCameraSamplesAgainstRobotTimeline(
                cameraSamples,
                nextPendingCameraIndex,
                robotSamples,
                matchedCameraSamples,
                droppedHeadCameraCount);
        };

    auto appendRobotPose = [&robotSamples, pFanucDriver, &lastRobotMonitorMs, &passiveRobotSamplingActive, &resolveReadyCameraSamples]()
        {
            RobotCalculation::TimestampedRobotPose sample;
            long long robotMs = 0;
            long long pcRecvMs = 0;
            const T_ROBOT_COORS passivePose = pFanucDriver->GetCurrentPosPassive(&robotMs, &pcRecvMs);
            if (pcRecvMs > 0)
            {
                if (passiveRobotSamplingActive && robotMs == lastRobotMonitorMs)
                {
                    return false;
                }

                sample.pose = passivePose;
                sample.timestampUs = static_cast<qint64>(robotMs) * 1000;
                lastRobotMonitorMs = robotMs;
                passiveRobotSamplingActive = true;
            }
            else
            {
                sample.pose = pFanucDriver->GetCurrentPos();
                sample.timestampUs = SteadyNowUs();
            }

            robotSamples.push_back(sample);
            resolveReadyCameraSamples();
            return true;
        };

    auto pullScanCameraFramesTo = [&scanCameraFrames, &lastPulledCameraSequence](std::uint64_t targetSequence)
        {
            if (targetSequence <= lastPulledCameraSequence)
            {
                return;
            }

            const std::vector<udpDataShow> frames = CameraFrameCache::Instance().FramesBetween(
                lastPulledCameraSequence,
                targetSequence);
            scanCameraFrames.insert(scanCameraFrames.end(), frames.begin(), frames.end());
            lastPulledCameraSequence = targetSequence;
        };

    auto pullScanCameraFrames = [&pullScanCameraFramesTo]()
        {
            pullScanCameraFramesTo(CameraFrameCache::Instance().Mark());
        };

    if (appendLog)
    {
        appendLog(QString("开始扫描运动：相机帧由全局缓存线程独立读取，配置相机读取帧率=%1 fps（约 %2 ms/帧，用于时间间隔统计），机器人位姿约 %3 ms 采样；机器人位姿优先使用机器人端robot_ms，相机帧timestamp会在首帧处映射到robot_ms时间轴，并叠加相机时间补偿 %4 ms。配置扫描速度= %5 mm/min，下发速度= %6 mm/sec")
            .arg(actualCameraReadFps, 0, 'f', 2)
            .arg(cameraReadIntervalMs)
            .arg(ROBOT_SAMPLE_INTERVAL_MS)
            .arg(param.dCameraTimeOffsetMs, 0, 'f', 3)
            .arg(param.dScanSpeed, 0, 'f', 3)
            .arg(fanucScanSpeed, 0, 'f', 3));
    }

    const bool moveOk = pFanucDriver->MoveByJob(
        param.tEndPos,
        T_ROBOT_MOVE_SPEED(fanucScanSpeed, 0.0, 0.0),
        pFanucDriver->m_nExternalAxleType,
        "MOVL");
    if (!moveOk)
    {
        if (appendLog)
        {
            appendLog("扫描终点运动启动失败。");
        }
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
            motionState = pFanucDriver->GetIntVar(FANUC_MOTION_STATE_REG);
            if (motionState == 10 || motionState == 20 || motionState == 1)
            {
                if (!motionStarted)
                {
                    scanStartCameraSequence = CameraFrameCache::Instance().Mark();
                    lastPulledCameraSequence = scanStartCameraSequence;
                    if (appendLog)
                    {
                        appendLog(QString("扫描运动状态寄存器进入运行态：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
                    }
                }
                motionStarted = true;
            }
            if (motionStarted)
            {
                appendRobotPose();
                if (!hasCameraTimeBaseRobotTimestamp && !robotSamples.empty())
                {
                    cameraTimeBaseRobotTimestampUs = robotSamples.back().timestampUs;
                    hasCameraTimeBaseRobotTimestamp = true;
                }
                pullScanCameraFrames();
            }
            if (motionStarted && motionState == 1)
            {
                scanEndCameraSequence = CameraFrameCache::Instance().Mark();
                pullScanCameraFramesTo(scanEndCameraSequence);
                break;
            }

            const qint64 elapsedMs = SteadyNowMs() - motionStartMs;
            if (!motionStarted && elapsedMs > 3000)
            {
                if (appendLog)
                {
                    appendLog(QString("扫描运动未在 3s 内进入运行态：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
                }
                return false;
            }
            if (motionStarted && elapsedMs > 120000)
            {
                if (appendLog)
                {
                    appendLog(QString("扫描运动等待完成超时：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
                }
                return false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    appendRobotPose();

    if (scanEndCameraSequence == 0)
    {
        scanEndCameraSequence = CameraFrameCache::Instance().Mark();
        pullScanCameraFramesTo(scanEndCameraSequence);
    }
    for (const udpDataShow& frame : scanCameraFrames)
    {
        appendCameraFrame(frame);
    }

    resolveReadyCameraSamples();

    bool tailWaitTriggered = false;
    const qint64 tailWaitStartMs = SteadyNowMs();
    while (nextPendingCameraIndex < cameraSamples.size()
        && !robotSamples.empty()
        && cameraSamples[nextPendingCameraIndex].timestampUs > robotSamples.back().timestampUs
        && (SteadyNowMs() - tailWaitStartMs) < CAMERA_ROBOT_MATCH_TAIL_WAIT_MS)
    {
        if (!tailWaitTriggered && appendLog)
        {
            appendLog(QString("检测到 %1 个相机点时间戳晚于最新机器人位姿，开始等待机器人监控时间追上（最长 %2 ms）。")
                .arg(static_cast<int>(cameraSamples.size() - nextPendingCameraIndex))
                .arg(CAMERA_ROBOT_MATCH_TAIL_WAIT_MS));
        }
        tailWaitTriggered = true;

        appendRobotPose();
        if (nextPendingCameraIndex >= cameraSamples.size()
            || cameraSamples[nextPendingCameraIndex].timestampUs <= robotSamples.back().timestampUs)
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_ROBOT_MATCH_TAIL_POLL_MS));
    }

    resolveReadyCameraSamples();
    const int droppedTailCameraCount = static_cast<int>(cameraSamples.size() - nextPendingCameraIndex);

    const QString resultDir = BuildResultDir(param.sRobotName);
    const QString cameraDir = QDir(resultDir).filePath("CameraPoint");
    const QString robotDir = QDir(resultDir).filePath("RobotPoint");
    const QString laserDir = QDir(resultDir).filePath("LaserPoint");
    QDir().mkpath(cameraDir);
    QDir().mkpath(robotDir);
    QDir().mkpath(laserDir);

    const QString cameraPath = QDir(cameraDir).filePath("PreciseCameraPoint.txt");
    const QString robotPath = QDir(robotDir).filePath("PreciseRobotPoint.txt");
    const QString laserPath = QDir(laserDir).filePath("PreciseLaserPoint.txt");
    const QString matchDebugPath = QDir(laserDir).filePath("PreciseLaserPoint_MatchDebug.csv");
    const QString preservePathFitPath = QDir(laserDir).filePath("PreciseLaserPoint_PreservePath_2mm.txt");
    const QString classifiedPath = QDir(laserDir).filePath("PreciseLaserPoint_Classified.txt");
    const QString classifiedNoisePath = QDir(laserDir).filePath("PreciseLaserPoint_Classified_Noise.txt");
    const QString weldPosePath = QDir(laserDir).filePath(WELD_POSE_FILE_NAME);
    const QString weldPoseSeamCompPath = QDir(laserDir).filePath(WELD_POSE_SEAM_COMP_FILE_NAME);
    savedPath = resultDir;

    std::vector<QString> cameraLines;
    std::vector<QString> robotLines;
    std::vector<QString> laserLines;
    std::vector<QString> matchDebugLines;
    cameraLines.reserve(cameraSamples.size() + 1);
    robotLines.reserve(matchedCameraSamples.size() + 1);
    laserLines.reserve(matchedCameraSamples.size() + 1);
    matchDebugLines.reserve(cameraSamples.size() + 1);
    cameraLines.push_back("index,x,y,z,error");
    robotLines.push_back("index,x,y,z,rx,ry,rz,bx,by,bz");
    laserLines.push_back("index,x,y,z");
    matchDebugLines.push_back("index,status,camera_raw_timestamp_us,camera_raw_delta_us,mapped_robot_timestamp_us,prev_robot_index,prev_robot_timestamp_us,next_robot_index,next_robot_timestamp_us,interp_ratio,camera_x,camera_y,camera_z,robot_x,robot_y,robot_z,robot_rx,robot_ry,robot_rz,robot_bx,robot_by,robot_bz,laser_x,laser_y,laser_z,error");

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

    for (const TimestampedCameraPoint& sample : cameraSamples)
    {
        cameraLines.push_back(RobotCalculation::Vector3IndexedCsv(sample.sampleIndex, sample.point, sample.error));
    }

    QSet<int> matchedCameraIndexes;
    for (const TimestampedCameraPoint& sample : matchedCameraSamples)
    {
        matchedCameraIndexes.insert(sample.sampleIndex);
    }

    int skippedLaserCount = 0;
    int unmatchedBeforeRobotCount = 0;
    int unmatchedAfterRobotCount = 0;
    int unmatchedUnknownCount = 0;
    int laserIndexGapCount = 0;
    int maxLaserIndexGap = 0;
    int lastLaserIndex = -1;
    for (const TimestampedCameraPoint& sample : cameraSamples)
    {
        const int index = sample.sampleIndex;
        QString status;
        bool hasRobotPose = false;
        bool hasLaserPoint = false;
        T_ROBOT_COORS interpolatedPose;
        Eigen::Vector3d laserPoint = Eigen::Vector3d::Zero();
        RobotInterpolationWindow robotWindow;
        if (robotSamples.empty())
        {
            status = "unmatched_no_robot_sample";
            ++unmatchedUnknownCount;
        }
        else if (sample.timestampUs < robotSamples.front().timestampUs)
        {
            status = "unmatched_before_robot";
            ++unmatchedBeforeRobotCount;
        }
        else if (sample.timestampUs > robotSamples.back().timestampUs)
        {
            status = "unmatched_after_robot";
            ++unmatchedAfterRobotCount;
        }
        else
        {
            robotWindow = FindRobotInterpolationWindow(robotSamples, sample.timestampUs);
            interpolatedPose = RobotCalculation::InterpolateRobotPose(robotSamples, sample.timestampUs);
            hasRobotPose = true;
            if (!matchedCameraIndexes.contains(index))
            {
                status = "unmatched_not_resolved";
                ++unmatchedUnknownCount;
            }
            else if (ShouldSkipLaserCalc(sample))
            {
                status = "skip_invalid_camera_point";
                ++skippedLaserCount;
            }
            else
            {
                status = "laser_ok";
                laserPoint = RobotCalculation::CalcLaserPointInRobot(interpolatedPose, sample.point, calibration);
                hasLaserPoint = true;
                if (lastLaserIndex > 0 && index - lastLaserIndex > 1)
                {
                    ++laserIndexGapCount;
                    maxLaserIndexGap = std::max(maxLaserIndexGap, index - lastLaserIndex - 1);
                }
                lastLaserIndex = index;

                robotLines.push_back(RobotCalculation::RobotPoseIndexedCsv(index, interpolatedPose));
                laserLines.push_back(RobotCalculation::Vector3IndexedCsv(index, laserPoint));

                RobotCalculation::IndexedPoint3D laserFitPoint;
                laserFitPoint.index = index;
                laserFitPoint.point = laserPoint;
                laserFitInput.push_back(laserFitPoint);
            }
        }

        QStringList fields;
        fields
            << QString::number(index)
            << status
            << QString::number(sample.rawTimestampUs)
            << QString::number(sample.rawDeltaUs)
            << QString::number(sample.timestampUs)
            << QString::number(robotWindow.prevIndex)
            << QString::number(robotWindow.prevTimestampUs)
            << QString::number(robotWindow.nextIndex)
            << QString::number(robotWindow.nextTimestampUs)
            << QString::number(robotWindow.ratio, 'f', 6)
            << QString::number(sample.point.x(), 'f', 6)
            << QString::number(sample.point.y(), 'f', 6)
            << QString::number(sample.point.z(), 'f', 6);
        if (hasRobotPose)
        {
            fields
                << QString::number(interpolatedPose.dX, 'f', 6)
                << QString::number(interpolatedPose.dY, 'f', 6)
                << QString::number(interpolatedPose.dZ, 'f', 6)
                << QString::number(interpolatedPose.dRX, 'f', 6)
                << QString::number(interpolatedPose.dRY, 'f', 6)
                << QString::number(interpolatedPose.dRZ, 'f', 6)
                << QString::number(interpolatedPose.dBX, 'f', 6)
                << QString::number(interpolatedPose.dBY, 'f', 6)
                << QString::number(interpolatedPose.dBZ, 'f', 6);
        }
        else
        {
            fields << "" << "" << "" << "" << "" << "" << "" << "" << "";
        }
        if (hasLaserPoint)
        {
            fields
                << QString::number(laserPoint.x(), 'f', 6)
                << QString::number(laserPoint.y(), 'f', 6)
                << QString::number(laserPoint.z(), 'f', 6);
        }
        else
        {
            fields << "" << "" << "";
        }
        fields << CsvEscape(sample.error);
        matchDebugLines.push_back(fields.join(','));
    }

    QString error;
    if (!SaveTextLines(cameraPath, cameraLines, error)
        || !SaveTextLines(robotPath, robotLines, error)
        || !SaveTextLines(laserPath, laserLines, error)
        || !SaveTextLines(matchDebugPath, matchDebugLines, error))
    {
        if (appendLog)
        {
            appendLog(error);
        }
        return false;
    }

    if (appendLog)
    {
        appendLog(QString("扫描完成，相机点=%1，机器人采样=%2，已匹配相机点=%3，保存目录=%4")
            .arg(static_cast<int>(cameraSamples.size()))
            .arg(static_cast<int>(robotSamples.size()))
            .arg(static_cast<int>(matchedCameraSamples.size()))
            .arg(resultDir));
        appendLog(QString("扫描期间全局缓存帧=%1，缓存序号范围=(%2, %3]")
            .arg(static_cast<int>(scanCameraFrames.size()))
            .arg(scanStartCameraSequence)
            .arg(scanEndCameraSequence));
        if (hasCameraToRobotTimeOffset)
        {
            appendLog(QString("相机时间轴映射到机器人robot_ms：首帧相机timestamp=%1 us，对齐机器人时间=%2 us，映射偏移=%3 us，额外补偿=%4 ms。")
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
        appendLog(QString("激光点序号断点统计：断点段数=%1，最大连续缺失帧数=%2，匹配前丢弃=%3，匹配后丢弃=%4，未知未匹配=%5")
            .arg(laserIndexGapCount)
            .arg(maxLaserIndexGap)
            .arg(unmatchedBeforeRobotCount)
            .arg(unmatchedAfterRobotCount)
            .arg(unmatchedUnknownCount));
        appendLog(QString("相机点文件：%1").arg(cameraPath));
        appendLog(QString("机器人插值位姿文件：%1").arg(robotPath));
        appendLog(QString("激光点文件：%1").arg(laserPath));
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
        setFlowStep("扫描完成，正在进行 PreservePath 拟合");
    }
    if (appendLog)
    {
        appendLog(QString("开始 PreservePath 拟合：采样主轴=%1，步长=%2 mm，搜索窗口=%3 mm，分段容差=%4 mm，每段最少点数=%5")
            .arg(SampleAxisName(originalFitParams.sampleAxis))
            .arg(originalFitParams.sampleStep, 0, 'f', 3)
            .arg(originalFitParams.searchWindow, 0, 'f', 3)
            .arg(originalFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(originalFitParams.piecewiseMinSegmentPoints));
    }

    const RobotCalculation::LowerWeldFilterResult originalFitResult =
        RobotCalculation::FilterLowerWeldPath(laserFitInput, originalFitParams);
    if (!originalFitResult.ok)
    {
        if (appendLog)
        {
            appendLog(QString("PreservePath 拟合失败：%1").arg(originalFitResult.error));
            appendLog("已保留原始激光点文件，可先按原始点云继续分析。");
        }
        return true;
    }

    if (!SaveTextLines(preservePathFitPath, BuildFilterOutputLines(originalFitResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存 PreservePath 拟合结果失败：%1").arg(error));
        }
        return true;
    }

    if (appendLog)
    {
        appendLog(FilterResultSummary("PreservePath 拟合", originalFitParams, originalFitResult, preservePathFitPath));
    }

    if (setFlowStep)
    {
        setFlowStep("PreservePath 拟合完成，正在进行焊道分类");
    }
    const RobotCalculation::LowerWeldClassificationResult classifiedResult =
        RobotCalculation::ClassifyLowerWeldPoints(originalFitResult, originalFitParams.sampleAxis);
    if (!classifiedResult.ok)
    {
        if (appendLog)
        {
            appendLog(QString("焊道分类失败：%1").arg(classifiedResult.error));
            appendLog(QString("已保留 PreservePath 拟合结果：%1").arg(preservePathFitPath));
        }
        return true;
    }

    if (!SaveTextLines(classifiedPath, BuildClassifiedOutputLines(classifiedResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存焊道分类结果失败：%1").arg(error));
        }
        return true;
    }

    if (!SaveTextLines(classifiedNoisePath, BuildNoiseOutputLines(laserFitInput, originalFitResult), error))
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
        for (const RobotCalculation::LowerWeldClassifiedPoint& point : classifiedResult.points)
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
        appendLog(QString("焊道杂点文件：%1").arg(classifiedNoisePath));
    }

    const WeldPosePreset weldPosePreset = LoadWeldPosePreset(param);
    if (appendLog)
    {
        appendLog(QString("焊接姿态参数：RX=%1, RY=%2, 拐点前过渡=%3 mm, 起点跳过=%4 mm, 终点跳过=%5 mm, 姿态补偿槽=%6, 焊道补偿槽=%7, 基础参数来源=%8, 姿态补偿来源=%9, 焊道补偿来源=%10")
            .arg(weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.ry, 0, 'f', 3)
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
        BuildSegmentPoseOutputLines(classifiedResult, param, weldPosePreset, appendLog);
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

    T_PRECISE_MEASURE_PARAM param;
    param.sRobotName = robotName.trimmed().isEmpty()
        ? std::string("RobotA")
        : robotName.trimmed().toStdString();
    const WeldPosePreset preset = LoadWeldPosePreset(param);

    QStringList outputLines;
    outputLines.reserve(records.size() + 1);
    outputLines << "weld_index raw_index x y z rx ry rz bx by bz point_type segment_kind";

    const WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
    for (const WeldPoseFileRecord& record : records)
    {
        outputLines << BuildWeldPoseFileRecordLine(record);
    }

    if (!RobotDataHelper::SaveTextFileLines(outputPath, outputLines, &error))
    {
        return false;
    }

    QStringList usedSlots = compStats.usedSlots.values();
    usedSlots.sort();
    summary = QString("焊道补偿完成：点数=%1，使用槽位=%2，Z补偿点数=%3，枪反向补偿点数=%4，焊道方向补偿点数=%5，配置=%6")
        .arg(records.size())
        .arg(usedSlots.isEmpty() ? QString("无匹配槽位") : usedSlots.join(","))
        .arg(compStats.zAdjustedCount)
        .arg(compStats.gunDirAdjustedCount)
        .arg(compStats.seamDirAdjustedCount)
        .arg(QDir::toNativeSeparators(preset.seamCompFilePath));
    return true;
}

bool MeasureThenWeldService::DownlinkWeldPoseFile(
    FANUCRobotCtrl* pFanucDriver,
    const QString& poseFilePath,
    double linearSpeedConfigMmPerMin,
    QString& summary,
    QString& error) const
{
    summary.clear();
    error.clear();
    (void)linearSpeedConfigMmPerMin;

    if (pFanucDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(poseFilePath, records, error))
    {
        return false;
    }

    const double linearSpeedMmPerSec =
        FanucLinearSpeedMmPerSecFromConfig(FANUC_WELD_PATH_SPEED_MM_PER_MIN, 1.0);
    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    if (!BuildWeldPoseMoveInfos(records, linearSpeedMmPerSec, moveInfos, error))
    {
        return false;
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
        error = QString("下发焊接轨迹失败：ret=%1，姿态文件=%2")
            .arg(downlinkRet)
            .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
        return false;
    }

    summary = QString("点数=%1，焊接速度固定=%2 mm/min (%3 mm/sec)，程序=%4，本地LS=%5，远程TP=%6，当前仅下发未自动执行")
        .arg(static_cast<int>(moveInfos.size()))
        .arg(FANUC_WELD_PATH_SPEED_MM_PER_MIN, 0, 'f', 3)
        .arg(linearSpeedMmPerSec, 0, 'f', 3)
        .arg(QString::fromStdString(programName))
        .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
        .arg(QString::fromStdString(remoteTpPath));
    return true;
}

bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(
    FANUCRobotCtrl* pFanucDriver,
    const QString& poseFilePath,
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

    if (pFanucDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(poseFilePath, records, error))
    {
        return false;
    }

    T_ROBOT_COORS startSafeCoors;
    if (!TryBuildWeldSafeCoors(records, 0, startSafeCoors, error))
    {
        return false;
    }

    T_ROBOT_COORS endSafeCoors;
    if (!TryBuildWeldSafeCoors(records, records.size() - 1, endSafeCoors, error))
    {
        return false;
    }

    const double weldLinearSpeedMmPerSec =
        FanucLinearSpeedMmPerSecFromConfig(FANUC_WELD_PATH_SPEED_MM_PER_MIN, 1.0);
    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    if (!BuildWeldPoseMoveInfos(records, weldLinearSpeedMmPerSec, moveInfos, error))
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
        appendLog(QString("下枪安全位置：%1").arg(RobotCoorsText(startSafeCoors)));
        appendLog(QString("收枪安全位置：%1").arg(RobotCoorsText(endSafeCoors)));
    }

    const double safeMoveSpeedMmPerSec =
        FanucLinearSpeedMmPerSecFromConfig(FANUC_SAFE_MOVE_SPEED_MM_PER_MIN, 1.0);
    if (setFlowStep)
    {
        setFlowStep("正在移动到下枪安全位置，并行下发焊接轨迹");
    }
    if (appendLog)
    {
        appendLog(QString("开始直线运动：下枪安全位置，配置速度= %1 mm/min，下发速度= %2 mm/sec")
            .arg(FANUC_SAFE_MOVE_SPEED_MM_PER_MIN, 0, 'f', 3)
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
                "下枪安全位置：%1\n"
                "焊接程序：%2\n"
                "是否开始执行焊道？")
            .arg(RobotCoorsText(startSafeCoors))
            .arg(QString::fromStdString(programName))))
    {
        error = "用户在焊前确认节点取消了流程。";
        return false;
    }

    const QString downlinkSummary =
        QString("点数=%1，焊接速度固定=%2 mm/min (%3 mm/sec)，程序=%4，本地LS=%5，远程TP=%6")
            .arg(static_cast<int>(moveInfos.size()))
            .arg(FANUC_WELD_PATH_SPEED_MM_PER_MIN, 0, 'f', 3)
            .arg(weldLinearSpeedMmPerSec, 0, 'f', 3)
            .arg(QString::fromStdString(programName))
            .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
            .arg(QString::fromStdString(remoteTpPath));
    const QString programNameText = QString::fromStdString(programName);
    const double pathLengthMm = EstimateMoveInfosPathLengthMm(moveInfos);
    const double estimatedRunMs = weldLinearSpeedMmPerSec > 1e-6
        ? (pathLengthMm / weldLinearSpeedMmPerSec) * 1000.0
        : 0.0;
    const int finishTimeoutMs = static_cast<int>(std::clamp(
        estimatedRunMs * 2.0 + 30000.0,
        120000.0,
        1800000.0));

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

    int lastState = 0;
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

    if (checkpoint && !checkpoint(
        "焊后确认",
        QString("焊接轨迹已执行完成。\n程序：%1\nR[%2]=%3\n是否继续移动到收枪安全位置？")
            .arg(programNameText)
            .arg(FANUC_MOTION_STATE_REG)
            .arg(lastState)))
    {
        error = "用户在焊后确认节点取消了流程。";
        return false;
    }

    if (!MoveCoorsAndWait(
        pFanucDriver,
        endSafeCoors,
        FANUC_SAFE_MOVE_SPEED_MM_PER_MIN,
        "收枪安全位置",
        appendLog,
        setFlowStep))
    {
        error = "移动到收枪安全位置失败。";
        return false;
    }

    summary = QString("%1；安全移动速度固定=%2 mm/min；起点安全位=%3；终点安全位=%4")
        .arg(downlinkSummary)
        .arg(FANUC_SAFE_MOVE_SPEED_MM_PER_MIN, 0, 'f', 3)
        .arg(RobotCoorsText(startSafeCoors))
        .arg(RobotCoorsText(endSafeCoors));
    return true;
}
