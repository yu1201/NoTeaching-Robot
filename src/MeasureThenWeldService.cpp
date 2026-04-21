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

#include <cmath>
#include <thread>

namespace
{
struct TimestampedCameraPoint
{
    qint64 timestampMs = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    QString error;
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
    cameraSamples.reserve(10000);
    robotSamples.reserve(1000);

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
