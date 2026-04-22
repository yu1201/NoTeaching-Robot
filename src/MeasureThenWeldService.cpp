#include "MeasureThenWeldService.h"

#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "groove/framebuffer.h"
#include "groove/threadsafebuffer.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QThread>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

namespace
{
struct TimestampedCameraPoint
{
    qint64 timestampMs = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    QString error;
};

struct WeldPosePreset
{
    QString filePath;
    QString sectionName;
    double rx = 0.0;
    double ry = 0.0;
    double cornerTransitionLeadDistance = 10.0;
    bool fromIni = false;
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
    params.fitMode = RobotCalculation::LowerWeldFitMode::PiecewiseLineFit;
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

RobotCalculation::LowerWeldFilterParams BuildTrapezoidFitParams(
    const RobotCalculation::LowerWeldFilterParams& originalFitParams)
{
    RobotCalculation::LowerWeldFilterParams params = originalFitParams;
    params.fitMode = RobotCalculation::LowerWeldFitMode::TrapezoidFit;
    params.zThreshold = std::numeric_limits<double>::max();
    params.zJumpThreshold = 0.0;
    params.zContinuityThreshold = 0.0;
    params.keepLongestSegmentOnly = false;
    params.searchWindow = std::max(2.0, originalFitParams.sampleStep);
    params.lineFitTrimCount = 0;
    params.piecewiseFitTolerance = std::max(5.0, originalFitParams.piecewiseFitTolerance);
    params.piecewiseMinSegmentPoints = std::max(12, originalFitParams.piecewiseMinSegmentPoints);
    params.minPointCount = 2;
    params.smoothRadius = 0;
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

bool TryReadIniDouble(COPini& ini, const std::string& key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

WeldPosePreset LoadWeldPosePreset(const T_PRECISE_MEASURE_PARAM& param)
{
    WeldPosePreset preset;
    preset.rx = param.tStartPos.dRX;
    preset.ry = param.tStartPos.dRY;
    preset.sectionName = "WeldNormalParam0";
    preset.filePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldLineParam.ini").arg(QString::fromStdString(param.sRobotName)));

    if (!QFileInfo::exists(preset.filePath))
    {
        return preset;
    }

    COPini ini;
    if (!ini.SetFileName(preset.filePath.toStdString()))
    {
        return preset;
    }

    ini.SetSectionName(preset.sectionName.toStdString());
    double rx = preset.rx;
    double ry = preset.ry;
    double cornerTransitionLeadDistance = preset.cornerTransitionLeadDistance;
    const bool hasNormalRx = TryReadIniDouble(ini, "NormalWeldRx", rx);
    const bool hasNormalRy = TryReadIniDouble(ini, "NormalWeldRy", ry);
    TryReadIniDouble(ini, "CornerTransitionLeadDis", cornerTransitionLeadDistance);
    if (!(hasNormalRx && hasNormalRy))
    {
        rx = preset.rx;
        ry = preset.ry;
        const bool hasFlatRx = TryReadIniDouble(ini, "FlatWeldRx", rx);
        const bool hasFlatRy = TryReadIniDouble(ini, "FlatWeldRy", ry);
        if (!(hasFlatRx && hasFlatRy))
        {
            return preset;
        }
    }

    preset.rx = rx;
    preset.ry = ry;
    preset.cornerTransitionLeadDistance = std::max(0.0, cornerTransitionLeadDistance);
    preset.fromIni = true;
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

std::vector<QString> BuildSegmentPoseOutputLines(
    const RobotCalculation::LowerWeldFilterResult& result,
    const T_PRECISE_MEASURE_PARAM& param,
    const WeldPosePreset& preset,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    struct SegmentInfo
    {
        int begin = 0;
        int end = 0;
        QString source;
        double fixedRz = 0.0;
    };

    std::vector<QString> lines;
    if (result.points.isEmpty())
    {
        return lines;
    }

    lines.reserve(static_cast<size_t>(result.points.size()) + 1);
    lines.push_back("index x y z rx ry rz bx by bz source");

    bool scanDirectionValid = false;
    const Eigen::Vector3d scanStart(param.tStartPos.dX, param.tStartPos.dY, param.tStartPos.dZ);
    const Eigen::Vector3d scanEnd(param.tEndPos.dX, param.tEndPos.dY, param.tEndPos.dZ);
    const double scanDirectionDeg = ComputeDirectionAngleDeg(scanStart, scanEnd, &scanDirectionValid);
    const double rzOffset = scanDirectionValid ? (param.tStartPos.dRZ - scanDirectionDeg) : 0.0;

    std::vector<SegmentInfo> segments;
    int begin = 0;
    double previousSegmentRz = param.tStartPos.dRZ;
    while (begin < result.points.size())
    {
        int end = begin;
        const QString segmentSource = result.points[begin].source;
        while ((end + 1) < result.points.size() && result.points[end + 1].source == segmentSource)
        {
            ++end;
        }

        bool segmentValid = false;
        const double segmentDirectionDeg = ComputeDirectionAngleDeg(
            result.points[begin].point,
            result.points[end].point,
            &segmentValid);
        double segmentRz = previousSegmentRz;
        if (segmentValid)
        {
            const double baseRz = scanDirectionValid
                ? (segmentDirectionDeg + rzOffset)
                : segmentDirectionDeg;
            segmentRz = NormalizeAngleNear(baseRz, param.tStartPos.dRZ);
        }

        segments.push_back({ begin, end, segmentSource, segmentRz });
        previousSegmentRz = segmentRz;
        begin = end + 1;
    }

    for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex)
    {
        const SegmentInfo& segment = segments[segmentIndex];
        if (appendLog)
        {
            appendLog(QString("焊道姿态段 %1: 点[%2-%3], 固定RZ=%4 deg, RX=%5 deg, RY=%6 deg, 拐点前过渡=%7 mm")
                .arg(segment.source.isEmpty() ? "segment" : segment.source)
                .arg(segment.begin + 1)
                .arg(segment.end + 1)
                .arg(segment.fixedRz, 0, 'f', 3)
                .arg(preset.rx, 0, 'f', 3)
                .arg(preset.ry, 0, 'f', 3)
                .arg(preset.cornerTransitionLeadDistance, 0, 'f', 3));
        }

        QVector<double> distanceToSegmentEnd;
        distanceToSegmentEnd.resize(segment.end - segment.begin + 1);
        double accumulatedDistance = 0.0;
        distanceToSegmentEnd[segment.end - segment.begin] = 0.0;
        for (int index = segment.end - 1; index >= segment.begin; --index)
        {
            accumulatedDistance += (result.points[index + 1].point - result.points[index].point).norm();
            distanceToSegmentEnd[index - segment.begin] = accumulatedDistance;
        }

        const bool hasNextSegment = (segmentIndex + 1) < segments.size();
        const double nextSegmentRz = hasNextSegment
            ? NormalizeAngleNear(segments[segmentIndex + 1].fixedRz, segment.fixedRz)
            : segment.fixedRz;

        for (int index = segment.begin; index <= segment.end; ++index)
        {
            const Eigen::Vector3d& point = result.points[index].point;
            double pointRz = segment.fixedRz;
            if (hasNextSegment && preset.cornerTransitionLeadDistance > 1e-6)
            {
                const double remainingDistance = distanceToSegmentEnd[index - segment.begin];
                if (remainingDistance < preset.cornerTransitionLeadDistance)
                {
                    const double transitionRatio = 1.0
                        - (remainingDistance / preset.cornerTransitionLeadDistance);
                    pointRz = segment.fixedRz
                        + (nextSegmentRz - segment.fixedRz) * std::clamp(transitionRatio, 0.0, 1.0);
                }
            }

            T_ROBOT_COORS pose(
                point.x(),
                point.y(),
                point.z(),
                preset.rx,
                preset.ry,
                pointRz,
                param.tStartPos.dBX,
                param.tStartPos.dBY,
                param.tStartPos.dBZ);
            lines.push_back(RobotPoseIndexedSpaceText(index + 1, pose, segment.source));
        }
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
    ini.ReadString(false, "dAcc", &param.dAcc);
    ini.ReadString(false, "dDec", &param.dDec);

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

    const int done = pFanucDriver->CheckRobotDone(100);
    if (appendLog)
    {
        appendLog(QString("运动结束：%1, CheckRobotDone=%2").arg(name).arg(done));
    }
    return done > 0;
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

    const int done = pFanucDriver->CheckRobotDone(100);
    if (appendLog)
    {
        appendLog(QString("直线运动结束：%1, CheckRobotDone=%2").arg(name).arg(done));
    }
    return done > 0;
}

bool MeasureThenWeldService::ScanMoveAndCollect(FANUCRobotCtrl* pFanucDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    const double fanucScanSpeed = FanucLinearSpeedMmPerSecFromConfig(param.dScanSpeed, 1.0);
    if (setFlowStep)
    {
        setFlowStep("扫描运动中，正在采集相机点、机器人位置和激光点");
    }

    ThreadSafeBuffer<udpDataShow>::Instance().setMaxSize(5000);
    ThreadSafeBuffer<udpDataShow>::Instance().clear();

    std::vector<TimestampedCameraPoint> cameraSamples;
    std::vector<RobotCalculation::TimestampedRobotPose> robotSamples;
    QVector<RobotCalculation::IndexedPoint3D> laserFitInput;
    cameraSamples.reserve(10000);
    robotSamples.reserve(1000);
    laserFitInput.reserve(10000);

    auto appendCameraFrame = [&cameraSamples](const udpDataShow& frame)
        {
            cameraSamples.push_back(TimestampedCameraPoint{
                QDateTime::currentMSecsSinceEpoch(),
                Eigen::Vector3d(frame.targetPoint.x, frame.targetPoint.y, frame.targetPoint.z),
                frame.errorMessage });
        };

    auto appendRobotPose = [&robotSamples, pFanucDriver]()
        {
            RobotCalculation::TimestampedRobotPose sample;
            sample.pose = pFanucDriver->GetCurrentPos();
            sample.timestampMs = QDateTime::currentMSecsSinceEpoch();
            robotSamples.push_back(sample);
        };

    if (appendLog)
    {
        appendLog(QString("开始扫描运动：相机点按 10ms 读取，机器人位姿约 50ms 采样；相机时间戳当前使用PC接收时刻模拟。配置扫描速度= %1 mm/min，下发速度= %2 mm/sec")
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
    int loopCount = 0;
    while (true)
    {
        udpDataShow frame;
        while (ThreadSafeBuffer<udpDataShow>::Instance().dequeue(frame, 0))
        {
            appendCameraFrame(frame);
        }

        if ((loopCount % 5) == 0)
        {
            appendRobotPose();
        }

        if (pFanucDriver->CheckDonePassive() > 0)
        {
            break;
        }

        ++loopCount;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    appendRobotPose();

    udpDataShow remainFrame;
    while (ThreadSafeBuffer<udpDataShow>::Instance().dequeue(remainFrame, 0))
    {
        appendCameraFrame(remainFrame);
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
    const QString laserPath = QDir(laserDir).filePath("PreciseLaserPoint.txt");
    const QString originalFitPath = QDir(laserDir).filePath("PreciseLaserPoint_OriginalFit_2mm.txt");
    const QString trapezoidFitPath = QDir(laserDir).filePath("PreciseLaserPoint_TrapezoidFit_2mm.txt");
    const QString trapezoidPosePath = QDir(laserDir).filePath("PreciseLaserPoint_TrapezoidPose_2mm.txt");
    savedPath = resultDir;

    std::vector<QString> cameraLines;
    std::vector<QString> robotLines;
    std::vector<QString> laserLines;
    cameraLines.reserve(cameraSamples.size() + 1);
    robotLines.reserve(cameraSamples.size() + 1);
    laserLines.reserve(cameraSamples.size() + 1);
    cameraLines.push_back("index,x,y,z,error");
    robotLines.push_back("index,x,y,z,rx,ry,rz,bx,by,bz");
    laserLines.push_back("index,x,y,z");

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

    int skippedLaserCount = 0;
    for (size_t i = 0; i < cameraSamples.size(); ++i)
    {
        const TimestampedCameraPoint& sample = cameraSamples[i];
        const int index = static_cast<int>(i) + 1;

        cameraLines.push_back(RobotCalculation::Vector3IndexedCsv(index, sample.point, sample.error));
        if (ShouldSkipLaserCalc(sample))
        {
            ++skippedLaserCount;
            continue;
        }

        const T_ROBOT_COORS interpolatedPose = RobotCalculation::InterpolateRobotPose(robotSamples, sample.timestampMs);
        const Eigen::Vector3d laserPoint = RobotCalculation::CalcLaserPointInRobot(interpolatedPose, sample.point, calibration);
        robotLines.push_back(RobotCalculation::RobotPoseIndexedCsv(index, interpolatedPose));
        laserLines.push_back(RobotCalculation::Vector3IndexedCsv(index, laserPoint));

        RobotCalculation::IndexedPoint3D laserFitPoint;
        laserFitPoint.index = index;
        laserFitPoint.point = laserPoint;
        laserFitInput.push_back(laserFitPoint);
    }

    QString error;
    if (!SaveTextLines(cameraPath, cameraLines, error)
        || !SaveTextLines(robotPath, robotLines, error)
        || !SaveTextLines(laserPath, laserLines, error))
    {
        if (appendLog)
        {
            appendLog(error);
        }
        return false;
    }

    if (appendLog)
    {
        appendLog(QString("扫描完成，相机点=%1，机器人采样=%2，保存目录=%3")
            .arg(static_cast<int>(cameraSamples.size()))
            .arg(static_cast<int>(robotSamples.size()))
            .arg(resultDir));
        appendLog(QString("激光计算有效点=%1，跳过异常相机点=%2")
            .arg(static_cast<int>(laserLines.size()) - 1)
            .arg(skippedLaserCount));
        appendLog(QString("相机点文件：%1").arg(cameraPath));
        appendLog(QString("机器人插值位姿文件：%1").arg(robotPath));
        appendLog(QString("激光点文件：%1").arg(laserPath));
    }

    if (laserFitInput.size() < 2)
    {
        if (appendLog)
        {
            appendLog(QString("激光有效点过少（%1），跳过原始轨迹拟合和梯形模板拟合。").arg(laserFitInput.size()));
        }
        return true;
    }

    const RobotCalculation::LowerWeldFilterParams originalFitParams = BuildOriginalTrackFitParams(param);
    if (setFlowStep)
    {
        setFlowStep("扫描完成，正在进行原始轨迹拟合");
    }
    if (appendLog)
    {
        appendLog(QString("开始原始轨迹拟合：采样主轴=%1，步长=%2 mm，搜索窗口=%3 mm，分段容差=%4 mm，每段最少点数=%5")
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
            appendLog(QString("原始轨迹拟合失败：%1").arg(originalFitResult.error));
            appendLog("已保留原始激光点文件，可先按原始点云继续分析。");
        }
        return true;
    }

    if (!SaveTextLines(originalFitPath, BuildFilterOutputLines(originalFitResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存原始轨迹拟合结果失败：%1").arg(error));
        }
        return true;
    }

    if (appendLog)
    {
        appendLog(FilterResultSummary("原始轨迹拟合", originalFitParams, originalFitResult, originalFitPath));
    }

    const RobotCalculation::LowerWeldFilterParams trapezoidFitParams =
        BuildTrapezoidFitParams(originalFitParams);
    if (setFlowStep)
    {
        setFlowStep("原始轨迹拟合完成，正在进行梯形模板拟合");
    }
    if (appendLog)
    {
        appendLog(QString("开始梯形模板拟合：基于原始拟合结果继续压模板，步长=%1 mm，分段容差=%2 mm，每段最少点数=%3")
            .arg(trapezoidFitParams.sampleStep, 0, 'f', 3)
            .arg(trapezoidFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(trapezoidFitParams.piecewiseMinSegmentPoints));
    }

    const RobotCalculation::LowerWeldFilterResult trapezoidFitResult =
        RobotCalculation::FilterLowerWeldPath(ToIndexedInput(originalFitResult.points), trapezoidFitParams);
    if (!trapezoidFitResult.ok)
    {
        if (appendLog)
        {
            appendLog(QString("梯形模板拟合失败：%1").arg(trapezoidFitResult.error));
            appendLog(QString("已保留原始轨迹拟合结果：%1").arg(originalFitPath));
        }
        return true;
    }

    if (!SaveTextLines(trapezoidFitPath, BuildFilterOutputLines(trapezoidFitResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存梯形模板拟合结果失败：%1").arg(error));
        }
        return true;
    }

    if (appendLog)
    {
        appendLog(FilterResultSummary("梯形模板拟合", trapezoidFitParams, trapezoidFitResult, trapezoidFitPath));
    }

    const WeldPosePreset weldPosePreset = LoadWeldPosePreset(param);
    if (appendLog)
    {
        appendLog(QString("焊接姿态参数：RX=%1, RY=%2, 来源=%3")
            .arg(weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.fromIni
                ? QString("%1 [%2]").arg(weldPosePreset.filePath, weldPosePreset.sectionName)
                : QString("扫描起点姿态回退")));
    }

    const std::vector<QString> trapezoidPoseLines =
        BuildSegmentPoseOutputLines(trapezoidFitResult, param, weldPosePreset, appendLog);
    if (!trapezoidPoseLines.empty())
    {
        if (!SaveTextLines(trapezoidPosePath, trapezoidPoseLines, error))
        {
            if (appendLog)
            {
                appendLog(QString("保存梯形焊道姿态结果失败：%1").arg(error));
            }
            return true;
        }

        if (appendLog)
        {
            appendLog(QString("梯形焊道姿态文件：%1").arg(trapezoidPosePath));
        }
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
