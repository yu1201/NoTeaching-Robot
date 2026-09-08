#include "FunctionTestDialog.h"

#include "AppPaths.h"

#include "CameraFrameCache.h"
#include "ConfigDatabase.h"
#include "ConfigSection.h"
#include "HandEyeMatrixConfig.h"
#include "RobotDataHelper.h"
#include "RobotCalculation.h"
#include "RobotDriverAdaptor.h"
#include "RobotAdaptorAcceptanceStore.h"
#include "RobotAdaptorAcceptanceLayout.h"
#include "RobotAdaptorAcceptancePlan.h"
#include "MeasureThenWeldCapabilityPolicy.h"
#include "RobotAcceptanceAlarmPreparation.h"
#include "RobotAcceptanceJointMotion.h"
#include "RobotRegisterRoundTrip.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"
#include "RobotCalibrationDialog.h"
#include "WindowStyleHelper.h"
#include "../portable/LaserFramePoint3DFilter/LaserFramePoint3DFilter.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStringConverter>
#include <QStringList>
#include <QTableWidget>
#include <QTabBar>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTextDocument>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

namespace
{
constexpr int kDhJointCount = 6;
constexpr int kDhParamCount = 24;
constexpr double kDhOrientationResidualWeight = 5.0;
constexpr double kDhRegularizationWeight = 0.1;
constexpr int kDhFitMinSampleCount = 8;
constexpr int kDhFitRecommendedSampleCount = 20;
constexpr int kRobotCameraTimestampCheckDurationMs = 60000;
constexpr int kAdaptorAcceptanceStageCount = RobotAdaptorAcceptancePlan::StageCount;
const auto& kAdaptorAcceptanceStageNames = RobotAdaptorAcceptancePlan::StageNames;

QString AdaptorAcceptanceStateText(const QString& state)
{
    if (state == "pass") { return QStringLiteral("通过"); }
    if (state == "fail") { return QStringLiteral("失败"); }
    if (state == "restricted") { return QStringLiteral("能力受限"); }
    if (state == "skipped") { return QStringLiteral("跳过"); }
    if (state == "running") { return QStringLiteral("执行中"); }
    return QStringLiteral("未测试");
}

QString AdaptorAcceptanceStageDescription(int stage)
{
    return QString::fromUtf8(RobotAdaptorAcceptancePlan::Description(stage));
}

QString FormatAcceptancePose(const T_ROBOT_COORS& pose)
{
    return QStringLiteral("X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6 BX=%7 BY=%8 BZ=%9")
        .arg(pose.dX, 0, 'f', 4).arg(pose.dY, 0, 'f', 4).arg(pose.dZ, 0, 'f', 4)
        .arg(pose.dRX, 0, 'f', 4).arg(pose.dRY, 0, 'f', 4).arg(pose.dRZ, 0, 'f', 4)
        .arg(pose.dBX, 0, 'f', 4).arg(pose.dBY, 0, 'f', 4).arg(pose.dBZ, 0, 'f', 4);
}

template<class T, class Formatter>
QString FormatRegisterRoundTripEvidence(const QString& name,
    const RobotRegisterRoundTrip::Result<T>& result, Formatter format)
{
    const auto status = [](const RobotRegisterRoundTrip::Step& step)
    { return QString::fromUtf8(RobotRegisterRoundTrip::StatusText(step)); };
    const auto match = [](const RobotRegisterRoundTrip::Step& read, bool equal)
    { return !read.ok ? QStringLiteral("未确认") : equal ? QStringLiteral("OK") : QStringLiteral("FAIL"); };
    QStringList evidence;
    evidence << QStringLiteral("%1：初始读取=%2 原值=%3 临时值=%4")
        .arg(name, status(result.initialRead),
            result.initialRead.ok ? format(result.original) : QStringLiteral("未获取"),
            result.write.attempted ? format(result.temporary) : QStringLiteral("未执行"));
    evidence << QStringLiteral("临时写入=%1 临时回读=%2 回读值=%3 临时值一致=%4")
        .arg(status(result.write), status(result.read),
            result.read.ok ? format(result.readback) : QStringLiteral("未获取"),
            match(result.read, result.temporaryMatches));
    evidence << QStringLiteral("原值落盘=%1 现场确认=%2")
        .arg(status(result.backupSaved), status(result.confirmation));
    evidence << QStringLiteral("恢复命令=%1 恢复回读=%2 恢复值=%3 原值一致=%4，结果=%5")
        .arg(status(result.restore), status(result.restoreRead),
            result.restoreRead.ok ? format(result.restored) : QStringLiteral("未获取"),
            match(result.restoreRead, result.originalMatches), result.Passed() ? "OK" : "FAIL");
    for (const auto& step : std::initializer_list<std::pair<const char*, const RobotRegisterRoundTrip::Step*>>{
        {"初始读取", &result.initialRead}, {"原值落盘", &result.backupSaved},
        {"现场确认", &result.confirmation}, {"临时写入", &result.write}, {"临时回读", &result.read},
        {"恢复命令", &result.restore}, {"恢复回读", &result.restoreRead}})
    {
        if (!step.second->error.empty())
        { evidence << QString::fromUtf8(step.first) + "错误：" + DecodeRobotMessageText(step.second->error); }
    }
    if (!result.initialRead.ok)
    { evidence << "初始读取失败：为避免覆盖未知原值，未执行任何写入。"; }
    if (result.write.attempted && !(result.restore.ok && result.restoreRead.ok && result.originalMatches))
    { evidence << "恢复未完整确认：已停止后续寄存器测试，请人工核对上述原值。"; }
    return evidence.join('\n');
}

QString JoinRemotePath(const QString& directory, const QString& name)
{
    QString normalized = directory.trimmed();
    normalized.replace('\\', '/');
    while (normalized.endsWith('/')) { normalized.chop(1); }
    return normalized.isEmpty() ? name : normalized + '/' + name;
}

bool FileSha256(const QString& path, QByteArray& sha256, qint64& size, QString* error)
{
    sha256.clear();
    size = 0;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (error != nullptr) { *error = QStringLiteral("无法读取文件：%1").arg(path); }
        return false;
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while (!file.atEnd())
    {
        const QByteArray chunk = file.read(1024 * 1024);
        if (chunk.isEmpty() && file.error() != QFileDevice::NoError)
        {
            if (error != nullptr) { *error = QStringLiteral("读取文件失败：%1").arg(path); }
            return false;
        }
        hash.addData(chunk);
        size += chunk.size();
    }
    sha256 = hash.result();
    if (error != nullptr) { error->clear(); }
    return true;
}

struct IntervalStats
{
    int count = 0;
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    double median = 0.0;
    double stddev = 0.0;
};

struct KinematicsFitSample
{
    int index = 0;
    T_ANGLE_PULSE pulse;
    T_ROBOT_COORS measuredPose;
};

struct RobotTimestampSample
{
    int index = 0;
    qint64 robotTimestampUs = 0;
    qint64 pcReceiveTimestampUs = 0;
    T_ROBOT_COORS pose;
    int done = -1;
};

struct LaserFramePoint3D
{
    int index = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

QString FindProjectFilePathForFunctionTest(const QString& relativePath)
{
    return QDir::toNativeSeparators(AppPaths::FindResourcePath(relativePath));
}

QPushButton* CreateTestButton(const QString& text)
{
    QPushButton* button = new QPushButton(text);
    button->setMinimumSize(150, 44);
    return button;
}

QString NativeAbsolutePath(const QString& path)
{
    return QDir::toNativeSeparators(QFileInfo(path).absoluteFilePath());
}

QString DefaultRobotName(const RobotDriverAdaptor* pRobotDriverAdaptor)
{
    if (pRobotDriverAdaptor == nullptr || pRobotDriverAdaptor->RobotName().empty())
    {
        return "RobotA";
    }
    return QString::fromStdString(pRobotDriverAdaptor->RobotName());
}

double WrapAngleDeg(double value)
{
    return std::remainder(value, 360.0);
}

QString JoinCsvRow(const QStringList& values)
{
    return values.join(',');
}

bool IsNearlyZero(double value)
{
    return std::abs(value) < 1e-6;
}

QString FormatDouble(double value)
{
    return QString::number(value, 'f', 6);
}

QString CsvEscapeForFunctionTest(const QString& value)
{
    QString escaped = value;
    escaped.replace("\"", "\"\"");
    if (escaped.contains(',') || escaped.contains('"') || escaped.contains('\n') || escaped.contains('\r'))
    {
        escaped = "\"" + escaped + "\"";
    }
    return escaped;
}


IntervalStats CalcIntervalStats(QVector<double> values)
{
    IntervalStats stats;
    stats.count = values.size();
    if (values.isEmpty())
    {
        return stats;
    }

    std::sort(values.begin(), values.end());
    stats.min = values.front();
    stats.max = values.back();
    if ((values.size() % 2) == 0)
    {
        stats.median = (values[values.size() / 2 - 1] + values[values.size() / 2]) * 0.5;
    }
    else
    {
        stats.median = values[values.size() / 2];
    }

    double sum = 0.0;
    for (double value : values)
    {
        sum += value;
    }
    stats.mean = sum / static_cast<double>(values.size());

    double variance = 0.0;
    for (double value : values)
    {
        const double diff = value - stats.mean;
        variance += diff * diff;
    }
    stats.stddev = std::sqrt(variance / static_cast<double>(values.size()));
    return stats;
}

QString FormatStatsLine(const QString& name, const IntervalStats& stats, const QString& unit, int precision = 3)
{
    if (stats.count <= 0)
    {
        return QString("%1：无有效数据").arg(name);
    }

    return QString("%1：N=%2 平均=%3%4 中位=%5%4 最小=%6%4 最大=%7%4 标准差=%8%4")
        .arg(name)
        .arg(stats.count)
        .arg(stats.mean, 0, 'f', precision)
        .arg(unit)
        .arg(stats.median, 0, 'f', precision)
        .arg(stats.min, 0, 'f', precision)
        .arg(stats.max, 0, 'f', precision)
        .arg(stats.stddev, 0, 'f', precision);
}

QString BuildRobotCameraTimestampCheckPath(const QString& robotName)
{
    const QString dirPath = RobotDataHelper::BuildProjectPath(QString("Result/%1/TimestampCheck").arg(robotName));
    QDir().mkpath(dirPath);
    return QDir::toNativeSeparators(QDir(dirPath).filePath(
        QString("RobotCameraTimestampCheck_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"))));
}

QString BuildCameraFramePointFilterTestDir(const QString& robotName)
{
    const QString dirPath = RobotDataHelper::BuildProjectPath(QString("Result/%1/CameraFrameFilterTest").arg(robotName));
    QDir().mkpath(dirPath);
    return QDir::toNativeSeparators(QFileInfo(dirPath).absoluteFilePath());
}

LaserFramePoint3DFilterOptions BuildThreeSegmentCameraFrameFilterOptions()
{
    LaserFramePoint3DFilterOptions options;
    options.enableDominantLineSegmentFilter = true;
    options.dominantLineMinSegmentCount = 2;
    options.dominantLineMaxSegmentCount = 3;
    options.dominantLineTrendRecoverDistanceMinMm = 2.0;
    options.dominantLineTrendRecoverDistanceStepScale = 5.0;
    options.dominantLineTrendRecoverEndpointToleranceMm = 3.0;
    options.dominantLineFastSampleCount = 160;
    options.dominantLineFastCandidateCount = 128;
    options.profileComponentKeepStandalone = true;
    options.profileRunKeepStandalone = true;
    return options;
}


std::vector<LaserFramePoint3D> BuildLaserFramePoint3DList(const std::vector<cv::Point3d>& sourcePoints)
{
    std::vector<LaserFramePoint3D> points;
    points.reserve(sourcePoints.size());
    for (int i = 0; i < static_cast<int>(sourcePoints.size()); ++i)
    {
        const cv::Point3d& point = sourcePoints[i];
        points.push_back({ i + 1, point.x, point.y, point.z });
    }
    return points;
}

bool WriteLaserFramePoint3DFile(const QString& filePath, const std::vector<LaserFramePoint3D>& points, QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (error != nullptr)
        {
            *error = QString("打开文件失败: %1").arg(NativeAbsolutePath(filePath));
        }
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream << "index,x,y,z\n";
    for (const LaserFramePoint3D& point : points)
    {
        stream << point.index << ','
            << QString::number(point.x, 'f', 6) << ','
            << QString::number(point.y, 'f', 6) << ','
            << QString::number(point.z, 'f', 6) << '\n';
    }
    return true;
}

bool HasMeaningfulToolOffset(const T_ROBOT_COORS& tool)
{
    return !(IsNearlyZero(tool.dX) && IsNearlyZero(tool.dY) && IsNearlyZero(tool.dZ)
        && IsNearlyZero(tool.dRX) && IsNearlyZero(tool.dRY) && IsNearlyZero(tool.dRZ));
}

T_ROBOT_COORS EffectiveKinematicsTool(const RobotDriverAdaptor* pRobotDriverAdaptor, QString* nameOut = nullptr)
{
    if (pRobotDriverAdaptor != nullptr && HasMeaningfulToolOffset(pRobotDriverAdaptor->Tools().tGunTool))
    {
        if (nameOut != nullptr)
        {
            *nameOut = "GunTool_d";
        }
        return pRobotDriverAdaptor->Tools().tGunTool;
    }

    if (nameOut != nullptr)
    {
        *nameOut = "ZeroTool";
    }
    return T_ROBOT_COORS();
}

std::array<double, kDhJointCount> PulseToJointDegrees(const T_ANGLE_PULSE& pulse, const T_AXISUNIT& axisUnit)
{
    return
    {
        pulse.nSPulse * axisUnit.dSPulseUnit,
        pulse.nLPulse * axisUnit.dLPulseUnit,
        pulse.nUPulse * axisUnit.dUPulseUnit,
        pulse.nRPulse * axisUnit.dRPulseUnit,
        pulse.nBPulse * axisUnit.dBPulseUnit,
        pulse.nTPulse * axisUnit.dTPulseUnit
    };
}

void CoorsToKdlFrameNoLog(const T_ROBOT_COORS& coors, KDL::Frame& frame)
{
    const double x = coors.dX / 1000.0;
    const double y = coors.dY / 1000.0;
    const double z = coors.dZ / 1000.0;
    const double rx = coors.dRX * M_PI / 180.0;
    const double ry = coors.dRY * M_PI / 180.0;
    const double rz = coors.dRZ * M_PI / 180.0;
    frame = KDL::Frame(KDL::Rotation::RPY(rx, ry, rz), KDL::Vector(x, y, z));
}

std::array<double, kDhParamCount> KinematicsToParamArray(const T_KINEMATICS& kinematics)
{
    return
    {
        kinematics.dA1, kinematics.dAL1, kinematics.dD1, kinematics.dTH1,
        kinematics.dA2, kinematics.dAL2, kinematics.dD2, kinematics.dTH2,
        kinematics.dA3, kinematics.dAL3, kinematics.dD3, kinematics.dTH3,
        kinematics.dA4, kinematics.dAL4, kinematics.dD4, kinematics.dTH4,
        kinematics.dA5, kinematics.dAL5, kinematics.dD5, kinematics.dTH5,
        kinematics.dA6, kinematics.dAL6, kinematics.dD6, kinematics.dTH6
    };
}


double DhRegularizationSigma(int paramIndex)
{
    const int fieldIndex = paramIndex % 4;
    return (fieldIndex == 0 || fieldIndex == 2) ? 50.0 : 10.0;
}

double DhFiniteDifferenceStep(int paramIndex)
{
    const int fieldIndex = paramIndex % 4;
    return (fieldIndex == 0 || fieldIndex == 2) ? 0.5 : 0.05;
}

bool ForwardPoseFromDhParams(
    const std::array<double, kDhParamCount>& params,
    const T_AXISUNIT& axisUnit,
    const T_ANGLE_PULSE& pulse,
    const T_ROBOT_COORS& toolCoors,
    T_ROBOT_COORS& outPose)
{
    KDL::Chain chain;
    for (int jointIndex = 0; jointIndex < kDhJointCount; ++jointIndex)
    {
        const int baseIndex = jointIndex * 4;
        const double aMeter = params[baseIndex] / 1000.0;
        const double alphaRad = params[baseIndex + 1] * M_PI / 180.0;
        const double dMeter = params[baseIndex + 2] / 1000.0;
        const double thetaRad = params[baseIndex + 3] * M_PI / 180.0;
        chain.addSegment(KDL::Segment(
            KDL::Joint(KDL::Joint::RotZ),
            KDL::Frame::DH(aMeter, alphaRad, dMeter, thetaRad)));
    }

    const std::array<double, kDhJointCount> jointDegrees = PulseToJointDegrees(pulse, axisUnit);
    KDL::JntArray joints(kDhJointCount);
    for (int jointIndex = 0; jointIndex < kDhJointCount; ++jointIndex)
    {
        joints(jointIndex) = jointDegrees[jointIndex] * M_PI / 180.0;
    }

    KDL::ChainFkSolverPos_recursive fkSolver(chain);
    KDL::Frame flangeFrame;
    if (fkSolver.JntToCart(joints, flangeFrame) < 0)
    {
        return false;
    }

    KDL::Frame toolFrame;
    CoorsToKdlFrameNoLog(toolCoors, toolFrame);
    const KDL::Frame tcpFrame = flangeFrame * toolFrame;

    outPose = T_ROBOT_COORS();
    outPose.dX = tcpFrame.p.x() * 1000.0;
    outPose.dY = tcpFrame.p.y() * 1000.0;
    outPose.dZ = tcpFrame.p.z() * 1000.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    tcpFrame.M.GetRPY(rx, ry, rz);
    outPose.dRX = rx * 180.0 / M_PI;
    outPose.dRY = ry * 180.0 / M_PI;
    outPose.dRZ = rz * 180.0 / M_PI;
    return true;
}

QString KinematicsCsvHeader()
{
    return "index,timestamp,s_pulse,l_pulse,u_pulse,r_pulse,b_pulse,t_pulse,bx_pulse,by_pulse,bz_pulse,"
           "j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,"
           "robot_x,robot_y,robot_z,robot_rx,robot_ry,robot_rz,"
           "model_x,model_y,model_z,model_rx,model_ry,model_rz,"
           "err_x,err_y,err_z,err_rx,err_ry,err_rz";
}

QString BuildKinematicsCsvRow(
    int index,
    const QString& timestamp,
    const T_ANGLE_PULSE& pulse,
    const T_AXISUNIT& axisUnit,
    const T_ROBOT_COORS& robotPose,
    const T_ROBOT_COORS& modelPose)
{
    const std::array<double, kDhJointCount> jointDegrees = PulseToJointDegrees(pulse, axisUnit);
    const double errX = modelPose.dX - robotPose.dX;
    const double errY = modelPose.dY - robotPose.dY;
    const double errZ = modelPose.dZ - robotPose.dZ;
    const double errRx = WrapAngleDeg(modelPose.dRX - robotPose.dRX);
    const double errRy = WrapAngleDeg(modelPose.dRY - robotPose.dRY);
    const double errRz = WrapAngleDeg(modelPose.dRZ - robotPose.dRZ);

    QStringList row;
    row
        << QString::number(index)
        << timestamp
        << QString::number(pulse.nSPulse)
        << QString::number(pulse.nLPulse)
        << QString::number(pulse.nUPulse)
        << QString::number(pulse.nRPulse)
        << QString::number(pulse.nBPulse)
        << QString::number(pulse.nTPulse)
        << QString::number(pulse.lBXPulse)
        << QString::number(pulse.lBYPulse)
        << QString::number(pulse.lBZPulse);

    for (double jointDegree : jointDegrees)
    {
        row << FormatDouble(jointDegree);
    }

    row
        << FormatDouble(robotPose.dX)
        << FormatDouble(robotPose.dY)
        << FormatDouble(robotPose.dZ)
        << FormatDouble(robotPose.dRX)
        << FormatDouble(robotPose.dRY)
        << FormatDouble(robotPose.dRZ)
        << FormatDouble(modelPose.dX)
        << FormatDouble(modelPose.dY)
        << FormatDouble(modelPose.dZ)
        << FormatDouble(modelPose.dRX)
        << FormatDouble(modelPose.dRY)
        << FormatDouble(modelPose.dRZ)
        << FormatDouble(errX)
        << FormatDouble(errY)
        << FormatDouble(errZ)
        << FormatDouble(errRx)
        << FormatDouble(errRy)
        << FormatDouble(errRz);

    return JoinCsvRow(row);
}

bool LoadKinematicsSamplesFromCsv(const QString& filePath, QVector<KinematicsFitSample>& samples, QString* error)
{
    samples.clear();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = "打开运动学样本文件失败: " + NativeAbsolutePath(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);

    bool isFirstLine = true;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty())
        {
            continue;
        }
        if (isFirstLine)
        {
            isFirstLine = false;
            if (line.startsWith("index,"))
            {
                continue;
            }
        }

        const QStringList parts = line.split(',', Qt::KeepEmptyParts);
        if (parts.size() < 23)
        {
            continue;
        }

        bool ok = false;
        KinematicsFitSample sample;
        sample.index = parts[0].toInt(&ok);
        if (!ok)
        {
            continue;
        }

        sample.pulse = T_ANGLE_PULSE(
            parts[2].toLong(), parts[3].toLong(), parts[4].toLong(),
            parts[5].toLong(), parts[6].toLong(), parts[7].toLong(),
            parts[8].toLong(), parts[9].toLong(), parts[10].toLong());

        sample.measuredPose.dX = parts[17].toDouble();
        sample.measuredPose.dY = parts[18].toDouble();
        sample.measuredPose.dZ = parts[19].toDouble();
        sample.measuredPose.dRX = parts[20].toDouble();
        sample.measuredPose.dRY = parts[21].toDouble();
        sample.measuredPose.dRZ = parts[22].toDouble();
        samples.push_back(sample);
    }

    if (samples.isEmpty())
    {
        if (error != nullptr)
        {
            *error = "样本文件里没有可用的关节/直角数据: " + NativeAbsolutePath(filePath);
        }
        return false;
    }
    return true;
}

double ComputeDhFitCost(
    const QVector<KinematicsFitSample>& samples,
    const std::array<double, kDhParamCount>& current,
    const std::array<double, kDhParamCount>& initial,
    const T_AXISUNIT& axisUnit,
    const T_ROBOT_COORS& toolCoors,
    Eigen::VectorXd* residualOut,
    double* positionRmseOut = nullptr,
    double* rotationRmseOut = nullptr)
{
    const int sampleResidualCount = samples.size() * 6;
    const int totalResidualCount = sampleResidualCount + kDhParamCount;
    if (residualOut != nullptr)
    {
        residualOut->resize(totalResidualCount);
    }

    double positionSquaredSum = 0.0;
    double rotationSquaredSum = 0.0;
    int residualIndex = 0;

    for (const KinematicsFitSample& sample : samples)
    {
        T_ROBOT_COORS predictedPose;
        if (!ForwardPoseFromDhParams(current, axisUnit, sample.pulse, toolCoors, predictedPose))
        {
            if (residualOut != nullptr)
            {
                residualOut->setConstant(totalResidualCount, 1e6);
            }
            if (positionRmseOut != nullptr)
            {
                *positionRmseOut = std::numeric_limits<double>::infinity();
            }
            if (rotationRmseOut != nullptr)
            {
                *rotationRmseOut = std::numeric_limits<double>::infinity();
            }
            return std::numeric_limits<double>::infinity();
        }

        const double dx = predictedPose.dX - sample.measuredPose.dX;
        const double dy = predictedPose.dY - sample.measuredPose.dY;
        const double dz = predictedPose.dZ - sample.measuredPose.dZ;
        const double drx = WrapAngleDeg(predictedPose.dRX - sample.measuredPose.dRX);
        const double dry = WrapAngleDeg(predictedPose.dRY - sample.measuredPose.dRY);
        const double drz = WrapAngleDeg(predictedPose.dRZ - sample.measuredPose.dRZ);

        positionSquaredSum += dx * dx + dy * dy + dz * dz;
        rotationSquaredSum += drx * drx + dry * dry + drz * drz;

        if (residualOut != nullptr)
        {
            (*residualOut)(residualIndex++) = dx;
            (*residualOut)(residualIndex++) = dy;
            (*residualOut)(residualIndex++) = dz;
            (*residualOut)(residualIndex++) = drx * kDhOrientationResidualWeight;
            (*residualOut)(residualIndex++) = dry * kDhOrientationResidualWeight;
            (*residualOut)(residualIndex++) = drz * kDhOrientationResidualWeight;
        }
    }

    const double regularizationScale = std::sqrt(kDhRegularizationWeight);
    for (int paramIndex = 0; paramIndex < kDhParamCount; ++paramIndex)
    {
        const double sigma = DhRegularizationSigma(paramIndex);
        const double value = regularizationScale * ((current[paramIndex] - initial[paramIndex]) / sigma);
        if (residualOut != nullptr)
        {
            (*residualOut)(residualIndex++) = value;
        }
    }

    if (positionRmseOut != nullptr)
    {
        *positionRmseOut = samples.isEmpty()
            ? 0.0
            : std::sqrt(positionSquaredSum / static_cast<double>(samples.size() * 3));
    }
    if (rotationRmseOut != nullptr)
    {
        *rotationRmseOut = samples.isEmpty()
            ? 0.0
            : std::sqrt(rotationSquaredSum / static_cast<double>(samples.size() * 3));
    }

    if (residualOut == nullptr)
    {
        return 0.0;
    }
    return 0.5 * residualOut->squaredNorm();
}

bool FitDhParamsByLeastSquares(
    const QVector<KinematicsFitSample>& samples,
    const T_KINEMATICS& initialKinematics,
    const T_AXISUNIT& axisUnit,
    const T_ROBOT_COORS& toolCoors,
    std::array<double, kDhParamCount>& fittedParams,
    double& beforePositionRmse,
    double& beforeRotationRmse,
    double& afterPositionRmse,
    double& afterRotationRmse)
{
    std::array<double, kDhParamCount> current = KinematicsToParamArray(initialKinematics);
    const std::array<double, kDhParamCount> initial = current;

    Eigen::VectorXd residual;
    double currentCost = ComputeDhFitCost(
        samples, current, initial, axisUnit, toolCoors, &residual, &beforePositionRmse, &beforeRotationRmse);
    if (!std::isfinite(currentCost))
    {
        return false;
    }

    double lambda = 1e-2;
    for (int iteration = 0; iteration < 25; ++iteration)
    {
        Eigen::MatrixXd jacobian(residual.size(), kDhParamCount);
        for (int paramIndex = 0; paramIndex < kDhParamCount; ++paramIndex)
        {
            std::array<double, kDhParamCount> stepped = current;
            stepped[paramIndex] += DhFiniteDifferenceStep(paramIndex);

            Eigen::VectorXd steppedResidual;
            const double steppedCost = ComputeDhFitCost(samples, stepped, initial, axisUnit, toolCoors, &steppedResidual);
            if (!std::isfinite(steppedCost))
            {
                return false;
            }

            jacobian.col(paramIndex) = (steppedResidual - residual) / DhFiniteDifferenceStep(paramIndex);
        }

        Eigen::MatrixXd hessian = jacobian.transpose() * jacobian;
        hessian += lambda * Eigen::MatrixXd::Identity(kDhParamCount, kDhParamCount);
        const Eigen::VectorXd gradient = jacobian.transpose() * residual;
        const Eigen::VectorXd delta = hessian.ldlt().solve(-gradient);
        if (!delta.allFinite())
        {
            return false;
        }
        if (delta.norm() < 1e-6)
        {
            break;
        }

        std::array<double, kDhParamCount> trial = current;
        for (int paramIndex = 0; paramIndex < kDhParamCount; ++paramIndex)
        {
            trial[paramIndex] += delta(paramIndex);
        }

        Eigen::VectorXd trialResidual;
        const double trialCost = ComputeDhFitCost(samples, trial, initial, axisUnit, toolCoors, &trialResidual);
        if (std::isfinite(trialCost) && trialCost < currentCost)
        {
            current = trial;
            residual = trialResidual;
            currentCost = trialCost;
            lambda = std::max(1e-6, lambda * 0.5);
        }
        else
        {
            lambda = std::min(1e6, lambda * 4.0);
        }
    }

    fittedParams = current;
    ComputeDhFitCost(samples, current, initial, axisUnit, toolCoors, nullptr, &afterPositionRmse, &afterRotationRmse);
    return std::isfinite(afterPositionRmse) && std::isfinite(afterRotationRmse);
}

QString BuildDhParameterReport(
    const QVector<KinematicsFitSample>& samples,
    const std::array<double, kDhParamCount>& initial,
    const std::array<double, kDhParamCount>& fitted,
    double beforePositionRmse,
    double beforeRotationRmse,
    double afterPositionRmse,
    double afterRotationRmse)
{
    QString report;
    QTextStream stream(&report);
    stream.setEncoding(QStringConverter::Utf8);
    stream << "DH拟合报告\n";
    stream << "样本数: " << samples.size() << "\n";
    stream << "说明: 本次拟合基于当前采集的关节/直角样本做实验性最小二乘优化，结果仅供校核，不会自动写回配置。\n";
    stream << "说明: 直角位姿使用当前读取接口返回值，工具补偿默认按零工具处理。\n";
    stream << "拟合前 RMSE: 位置=" << QString::number(beforePositionRmse, 'f', 4)
        << " mm, 姿态=" << QString::number(beforeRotationRmse, 'f', 4) << " deg\n";
    stream << "拟合后 RMSE: 位置=" << QString::number(afterPositionRmse, 'f', 4)
        << " mm, 姿态=" << QString::number(afterRotationRmse, 'f', 4) << " deg\n\n";

    stream << "建议参数对比:\n";
    stream << "Joint,Field,Initial,Fitted,Delta\n";
    static const char* fieldNames[4] = { "a", "alpha", "d", "theta" };
    for (int jointIndex = 0; jointIndex < kDhJointCount; ++jointIndex)
    {
        for (int fieldIndex = 0; fieldIndex < 4; ++fieldIndex)
        {
            const int paramIndex = jointIndex * 4 + fieldIndex;
            const double delta = fitted[paramIndex] - initial[paramIndex];
            stream << (jointIndex + 1) << "," << fieldNames[fieldIndex] << ","
                << QString::number(initial[paramIndex], 'f', 6) << ","
                << QString::number(fitted[paramIndex], 'f', 6) << ","
                << QString::number(delta, 'f', 6) << "\n";
        }
    }

    stream << "\n[Kinematics]\n";
    for (int jointIndex = 0; jointIndex < kDhJointCount; ++jointIndex)
    {
        const int baseIndex = jointIndex * 4;
        stream << "dA" << (jointIndex + 1) << "=" << QString::number(fitted[baseIndex], 'f', 6) << "\n";
        stream << "dAL" << (jointIndex + 1) << "=" << QString::number(fitted[baseIndex + 1], 'f', 6) << "\n";
        stream << "dD" << (jointIndex + 1) << "=" << QString::number(fitted[baseIndex + 2], 'f', 6) << "\n";
        stream << "dTH" << (jointIndex + 1) << "=" << QString::number(fitted[baseIndex + 3], 'f', 6) << "\n";
    }
    return report;
}

QDoubleSpinBox* CreateKinematicsValueEditor(
    double minimum,
    double maximum,
    int decimals)
{
    QDoubleSpinBox* editor = new QDoubleSpinBox();
    editor->setRange(minimum, maximum);
    editor->setDecimals(decimals);
    editor->setSingleStep(decimals > 3 ? 0.01 : 0.1);
    editor->setKeyboardTracking(false);
    editor->setAlignment(Qt::AlignRight);
    return editor;
}

class KinematicsCandidateSettings
{
public:
    KinematicsCandidateSettings(const QString& robotName, const QString& candidateId)
        : m_robotName(robotName.trimmed())
        , m_candidateId(candidateId.trimmed())
        , m_module(QStringLiteral("KinematicsCandidates/%1").arg(m_candidateId))
    {
        QString ignoredError;
        ConfigDatabase::ReadScopedModuleSnapshot(
            QStringLiteral("robot"), m_robotName, m_module, m_sections, &ignoredError);
        m_originalSections = m_sections.keys();
    }

    bool HasValues() const
    {
        return !m_sections.isEmpty();
    }

    QVariant value(const QString& path, const QVariant& defaultValue = QVariant()) const
    {
        QString section;
        QString key;
        if (!SplitPath(path, &section, &key))
        {
            return defaultValue;
        }
        const auto sectionIt = m_sections.constFind(section);
        if (sectionIt == m_sections.cend())
        {
            return defaultValue;
        }
        const auto valueIt = sectionIt->constFind(key);
        return valueIt == sectionIt->cend() ? defaultValue : QVariant(*valueIt);
    }

    void setValue(const QString& path, const QVariant& value)
    {
        QString section;
        QString key;
        if (!SplitPath(path, &section, &key))
        {
            m_valid = false;
            return;
        }
        QString text;
        const int typeId = value.metaType().id();
        if (typeId == QMetaType::Bool)
        {
            text = value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
        }
        else if (typeId == QMetaType::Double || typeId == QMetaType::Float)
        {
            text = QString::number(value.toDouble(), 'g', 17);
        }
        else
        {
            text = value.toString();
        }
        m_sections[section].insert(key, text);
    }

    void remove(const QString& path)
    {
        QString section;
        QString key;
        if (!SplitPath(path, &section, &key))
        {
            m_valid = false;
            return;
        }
        auto sectionIt = m_sections.find(section);
        if (sectionIt == m_sections.end())
        {
            return;
        }
        sectionIt->remove(key);
        if (sectionIt->isEmpty())
        {
            m_sections.erase(sectionIt);
        }
    }

    bool sync(QString* error = nullptr)
    {
        if (!m_valid || m_robotName.isEmpty() || m_candidateId.isEmpty())
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("运动学候选数据库身份无效。");
            }
            return false;
        }
        return ConfigDatabase::ReplaceScopedModuleSectionsAtomically(
            QStringLiteral("robot"), m_robotName, m_module,
            m_sections, m_originalSections, error);
    }

private:
    static bool SplitPath(const QString& path, QString* section, QString* key)
    {
        const int slash = path.lastIndexOf(QLatin1Char('/'));
        if (slash <= 0 || slash >= path.size() - 1)
        {
            return false;
        }
        *section = path.left(slash).trimmed();
        *key = path.mid(slash + 1).trimmed();
        return !section->isEmpty() && !key->isEmpty();
    }

    QString m_robotName;
    QString m_candidateId;
    QString m_module;
    QMap<QString, QMap<QString, QString>> m_sections;
    QStringList m_originalSections;
    bool m_valid = true;
};

class KinematicsDraftDialog final : public QDialog
{
public:
    explicit KinematicsDraftDialog(RobotDriverAdaptor* driver, QWidget* parent = nullptr)
        : QDialog(parent)
        , m_driver(driver)
        , m_robotName(DefaultRobotName(driver))
    {
        m_candidateId = FindLatestCandidateId();
        setWindowTitle("机器人运动学参数（候选配置）");
        setModal(true);
        resize(1120, 760);

        QVBoxLayout* rootLayout = new QVBoxLayout(this);

        QLabel* safetyLabel = new QLabel(
            "本页面只把候选参数保存到配置数据库，不修改当前 RobotPara 模块、不重建运行中的KDL链，也不向机器人写入DH。"
            "填写完成后必须经过控制器正解对比和人工确认，才能另行启用。");
        safetyLabel->setWordWrap(true);
        safetyLabel->setStyleSheet(
            "QLabel { background:#fff4ce; color:#6b4f00; border:1px solid #e3c36b; "
            "border-radius:4px; padding:8px; }");
        rootLayout->addWidget(safetyLabel);

        QTabWidget* tabs = new QTabWidget();
        tabs->addTab(CreateDhTab(), "DH/MDH参数");
        tabs->addTab(CreateTransformTab(), "Base/Flange/CAD姿态");
        tabs->addTab(CreateToolTab(), "Tool1枪尖TCP");
        rootLayout->addWidget(tabs, 1);

        m_statusLabel = new QLabel();
        m_statusLabel->setWordWrap(true);
        rootLayout->addWidget(m_statusLabel);

        QDialogButtonBox* buttons = new QDialogButtonBox();
        QPushButton* importButton = buttons->addButton("从当前运行参数导入DH", QDialogButtonBox::ActionRole);
        QPushButton* reloadButton = buttons->addButton("重新加载候选", QDialogButtonBox::ResetRole);
        QPushButton* saveButton = buttons->addButton("保存候选", QDialogButtonBox::AcceptRole);
        buttons->addButton("关闭", QDialogButtonBox::RejectRole);
        rootLayout->addWidget(buttons);

        connect(importButton, &QPushButton::clicked, this, [this]() {
            if (QMessageBox::question(
                    this,
                    "导入当前参数",
                    QString("将只用当前机器人 %1 内存中的标准DH覆盖表格前4列，并将约定切换为Standard DH；"
                            "不会修改限位、速度、Tool1、Base/Flange或CAD姿态，也不会立即写入数据库。是否继续？")
                        .arg(m_robotName))
                == QMessageBox::Yes)
            {
                ImportRuntimeDh();
                SetStatus("已从当前运行参数填充标准DH，其他候选字段保持不变，尚未保存。", false);
            }
        });
        connect(reloadButton, &QPushButton::clicked, this, [this]() { LoadDraftOrDefaults(); });
        connect(saveButton, &QPushButton::clicked, this, [this]() {
            QString error;
            if (!SaveDraft(&error))
            {
                QMessageBox::warning(this, "保存候选运动学参数", error);
                return;
            }
            m_saved = true;
            accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

        LoadDraftOrDefaults();
        UpdateConventionDescription();
    }

    bool WasSaved() const
    {
        return m_saved;
    }

    QString DraftStorageLabel() const
    {
        return QStringLiteral("robot/%1/KinematicsCandidates/%2")
            .arg(m_robotName, m_candidateId);
    }

private:
    QWidget* CreateDhTab()
    {
        QWidget* page = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(page);

        QFormLayout* form = new QFormLayout();
        QLineEdit* robotNameEdit = new QLineEdit(m_robotName);
        robotNameEdit->setReadOnly(true);
        m_modelEdit = new QLineEdit("SA10/2000H");
        m_modelEdit->setReadOnly(true);
        m_conventionCombo = new QComboBox();
        m_conventionCombo->addItem("Standard DH", "StandardDH");
        m_conventionCombo->addItem("Craig Modified DH (MDH)", "CraigMDH");
        m_conditionCombo = new QComboBox();
        m_conditionCombo->addItem("焊接工况", "Welding");
        m_conditionCombo->addItem("搬运工况", "Handling");
        m_sa10ConfirmedCheck = new QCheckBox("我确认当前所选机器人本体为SA10/2000H");
        form->addRow("机器人：", robotNameEdit);
        form->addRow("机器人型号：", m_modelEdit);
        form->addRow("参数约定：", m_conventionCombo);
        form->addRow("关节范围工况：", m_conditionCombo);
        form->addRow("目标确认：", m_sa10ConfirmedCheck);
        layout->addLayout(form);

        m_conventionDescription = new QLabel();
        m_conventionDescription->setWordWrap(true);
        layout->addWidget(m_conventionDescription);

        QLabel* sourceLabel = new QLabel(
            "只需手工填写 a、alpha、d、theta0，并确认厂家采用 Standard DH 还是 Craig MDH。"
            "关节限位和最大速度已按《SA10-2000H焊接机器人使用说明书》预填；方向初始为待验证，"
            "后续通过控制器正解样本自动核对。");
        sourceLabel->setWordWrap(true);
        sourceLabel->setStyleSheet("QLabel { color:#315b7d; }");
        layout->addWidget(sourceLabel);

        m_jointTable = new QTableWidget(kDhJointCount, 8);
        m_jointTable->setVerticalHeaderLabels({ "J1", "J2", "J3", "J4", "J5", "J6" });
        m_jointTable->setHorizontalHeaderLabels(
            { "a (mm)", "alpha (deg)", "d (mm)", "theta0 (deg)", "方向", "最小角 (deg)", "最大角 (deg)", "最大速度 (deg/s)" });
        m_jointTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_jointTable->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        m_jointTable->setAlternatingRowColors(true);
        m_jointTable->setMinimumHeight(330);

        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            m_jointEditors[joint][0] = CreateKinematicsValueEditor(-10000.0, 10000.0, 6);
            m_jointEditors[joint][1] = CreateKinematicsValueEditor(-3600.0, 3600.0, 6);
            m_jointEditors[joint][2] = CreateKinematicsValueEditor(-10000.0, 10000.0, 6);
            m_jointEditors[joint][3] = CreateKinematicsValueEditor(-3600.0, 3600.0, 6);
            m_jointEditors[joint][4] = CreateKinematicsValueEditor(-720.0, 720.0, 3);
            m_jointEditors[joint][5] = CreateKinematicsValueEditor(-720.0, 720.0, 3);
            m_jointEditors[joint][6] = CreateKinematicsValueEditor(0.0, 5000.0, 3);

            for (int column = 0; column < 4; ++column)
            {
                m_jointTable->setCellWidget(joint, column, m_jointEditors[joint][column]);
                connect(m_jointEditors[joint][column], &QDoubleSpinBox::valueChanged, this, [this]() {
                    m_dhSource = "ManualEntry";
                });
            }
            m_directionCombos[joint] = new QComboBox();
            m_directionCombos[joint]->addItem("待验证", 0);
            m_directionCombos[joint]->addItem("+1", 1);
            m_directionCombos[joint]->addItem("-1", -1);
            m_jointTable->setCellWidget(joint, 4, m_directionCombos[joint]);
            m_jointTable->setCellWidget(joint, 5, m_jointEditors[joint][4]);
            m_jointTable->setCellWidget(joint, 6, m_jointEditors[joint][5]);
            m_jointTable->setCellWidget(joint, 7, m_jointEditors[joint][6]);
            connect(m_jointEditors[joint][4], &QDoubleSpinBox::valueChanged, this, [this]() {
                m_limitSource = "ManualEntry";
            });
            connect(m_jointEditors[joint][5], &QDoubleSpinBox::valueChanged, this, [this]() {
                m_limitSource = "ManualEntry";
            });
            connect(m_jointEditors[joint][6], &QDoubleSpinBox::valueChanged, this, [this]() {
                m_speedSource = "ManualEntry";
            });
        }
        layout->addWidget(m_jointTable, 1);

        QLabel* thetaHelp = new QLabel(
            "theta0 是机器人控制器关节角为0°时的固定角度偏置；方向表示控制器关节角进入本地模型前乘以 +1 或 -1。"
            "它们不能与编码器零位或脉冲当量混为一项。");
        thetaHelp->setWordWrap(true);
        layout->addWidget(thetaHelp);

        connect(m_conventionCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
            if (index != m_lastConventionIndex)
            {
                bool hasDhValues = false;
                for (int joint = 0; joint < kDhJointCount && !hasDhValues; ++joint)
                {
                    for (int field = 0; field < 4; ++field)
                    {
                        if (std::abs(m_jointEditors[joint][field]->value()) > 1e-9)
                        {
                            hasDhValues = true;
                            break;
                        }
                    }
                }
                if (hasDhValues
                    && QMessageBox::question(
                           this,
                           "切换DH约定",
                           "Standard DH与Craig MDH的行定义不同。切换后将清空当前24项DH数据，是否继续？")
                        != QMessageBox::Yes)
                {
                    SetConventionIndexSilently(m_lastConventionIndex);
                    return;
                }
                if (hasDhValues)
                {
                    for (int joint = 0; joint < kDhJointCount; ++joint)
                    {
                        for (int field = 0; field < 4; ++field)
                        {
                            m_jointEditors[joint][field]->setValue(0.0);
                        }
                    }
                    m_dhSource = "ManualEntryPendingAfterConventionChange";
                    SetStatus("DH约定已切换，原24项参数已清空，请按新约定重新填写。", false);
                }
                m_lastConventionIndex = index;
            }
            UpdateConventionDescription();
        });
        connect(m_conditionCombo, &QComboBox::currentIndexChanged, this, [this]() {
            ApplySa10ConditionLimits();
        });
        return page;
    }

    QGroupBox* CreatePoseGroup(
        const QString& title,
        QCheckBox*& knownCheck,
        std::array<QDoubleSpinBox*, 6>& editors,
        const QString& knownText)
    {
        QGroupBox* group = new QGroupBox(title);
        QGridLayout* layout = new QGridLayout(group);
        knownCheck = new QCheckBox(knownText);
        layout->addWidget(knownCheck, 0, 0, 1, 6);

        static const char* labels[6] = { "X", "Y", "Z", "RX", "RY", "RZ" };
        for (int index = 0; index < 6; ++index)
        {
            QLabel* label = new QLabel(labels[index]);
            editors[index] = index < 3
                ? CreateKinematicsValueEditor(-10000.0, 10000.0, 6)
                : CreateKinematicsValueEditor(-3600.0, 3600.0, 6);
            layout->addWidget(label, 1, index);
            layout->addWidget(
                CreateExternalUnitEditor(
                    editors[index],
                    index < 3 ? QStringLiteral("mm") : QStringLiteral("deg"),
                    group),
                2,
                index);
        }
        return group;
    }

    QWidget* CreateTransformTab()
    {
        QWidget* page = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(page);

        layout->addWidget(CreatePoseGroup(
            "T_RobotBase_DhBase",
            m_baseKnownCheck,
            m_baseEditors,
            "厂家参数已明确给出RobotBase到DH基坐标的变换"));
        layout->addWidget(CreatePoseGroup(
            "T_DhEnd_Flange",
            m_flangeKnownCheck,
            m_flangeEditors,
            "厂家参数已明确给出最后DH坐标到机械法兰的变换"));

        QGroupBox* cadGroup = new QGroupBox("STEP/CAD导出姿态对应的控制器关节角");
        QGridLayout* cadLayout = new QGridLayout(cadGroup);
        m_cadPoseKnownCheck = new QCheckBox("已确认该组角度就是STEP模型的导出姿态");
        cadLayout->addWidget(m_cadPoseKnownCheck, 0, 0, 1, kDhJointCount);
        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            cadLayout->addWidget(new QLabel(QString("J%1").arg(joint + 1)), 1, joint);
            m_cadPoseEditors[joint] =
                CreateKinematicsValueEditor(-720.0, 720.0, 6);
            cadLayout->addWidget(
                CreateExternalUnitEditor(
                    m_cadPoseEditors[joint], QStringLiteral("deg"), cadGroup),
                2,
                joint);
        }
        layout->addWidget(cadGroup);

        QLabel* help = new QLabel(
            "若厂家DH已经把Base或Flange固定变换包含在表内，对应变换保持全零并不要勾选；"
            "如果STEP模型不是控制器零位姿态，必须填写其导出时J1-J6角度，否则分关节模型无法正确绑定。"
            "RX/RY/RZ目前按本项目RPY约定保存，启用前仍需与STEP控制器ABC定义核对。");
        help->setWordWrap(true);
        layout->addWidget(help);
        layout->addStretch(1);
        return page;
    }

    QWidget* CreateToolTab()
    {
        QWidget* page = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout(page);

        QFormLayout* form = new QFormLayout();
        m_toolNameEdit = new QLineEdit("tool1");
        m_toolNameEdit->setReadOnly(true);
        m_toolNameEdit->setToolTip("本页面固定读取逻辑工具编号1；实际命中的tool1/TOOL1名称仍由后续专用接口验证。");
        m_toolFixedCheck = new QCheckBox("控制器Tool.Fixed（灰色方块表示通用驱动未返回该字段）");
        m_toolFixedCheck->setTristate(true);
        m_toolFixedCheck->setCheckState(Qt::PartiallyChecked);
        m_toolPoseKnownCheck = new QCheckBox("我已人工确认当前显示的Tool1枪尖TCP数值");
        form->addRow("工具变量名：", m_toolNameEdit);
        form->addRow("工具类型：", m_toolFixedCheck);
        form->addRow("TCP确认：", m_toolPoseKnownCheck);
        layout->addLayout(form);

        QGroupBox* poseGroup = new QGroupBox("控制器Tool1枪尖TCP（XYZABC）");
        QGridLayout* poseLayout = new QGridLayout(poseGroup);
        static const char* labels[6] = { "X", "Y", "Z", "A/RX", "B/RY", "C/RZ" };
        for (int index = 0; index < 6; ++index)
        {
            poseLayout->addWidget(new QLabel(labels[index]), 0, index);
            m_toolEditors[index] = index < 3
                ? CreateKinematicsValueEditor(-5000.0, 5000.0, 6)
                : CreateKinematicsValueEditor(-3600.0, 3600.0, 6);
            poseLayout->addWidget(
                CreateExternalUnitEditor(
                    m_toolEditors[index],
                    index < 3 ? QStringLiteral("mm") : QStringLiteral("deg"),
                    poseGroup),
                1,
                index);
            connect(m_toolEditors[index], &QDoubleSpinBox::valueChanged, this, [this]() {
                m_toolPoseSource = "ManualEntry";
                m_toolPoseConvention = "Manual_Unverified";
                if (m_toolPoseKnownCheck != nullptr)
                {
                    m_toolPoseKnownCheck->setChecked(false);
                }
            });
        }
        layout->addWidget(poseGroup);

        QPushButton* readToolButton = new QPushButton("只读获取控制器Tool1");
        readToolButton->setMinimumHeight(40);
        layout->addWidget(readToolButton, 0, Qt::AlignLeft);

        m_toolSourceLabel = new QLabel("来源：尚未读取控制器；当前显示数据库GunTool或候选记录值。");
        m_toolSourceLabel->setWordWrap(true);
        layout->addWidget(m_toolSourceLabel);

        QLabel* help = new QLabel(
            "读取操作不会切换当前工具、不会上使能、不会发送运动；结果只填入本候选页面。"
            "当前通用驱动只返回XYZABC，不返回实际命中的tool1/TOOL1名称和Fixed字段；"
            "因此这两项保持未验证，后续扩展专用只读接口后才能作为放行依据。"
            "Tool1的ABC旋转顺序也必须通过控制器正解验证。");
        help->setWordWrap(true);
        layout->addWidget(help);
        layout->addStretch(1);

        connect(readToolButton, &QPushButton::clicked, this, [this]() { ReadControllerTool1(); });
        return page;
    }

    void PopulateSourceDefaults()
    {
        m_modelEdit->setText("SA10/2000H");
        SetConventionIndexSilently(0);
        m_conditionCombo->setCurrentIndex(0);
        m_sa10ConfirmedCheck->setChecked(false);
        m_toolNameEdit->setText("tool1");

        static const double defaultSpeed[kDhJointCount] = { 160.0, 160.0, 169.0, 300.0, 338.0, 535.0 };

        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            for (int field = 0; field < 4; ++field)
            {
                m_jointEditors[joint][field]->setValue(0.0);
            }
            m_directionCombos[joint]->setCurrentIndex(0);

            m_jointEditors[joint][6]->setValue(defaultSpeed[joint]);
            m_cadPoseEditors[joint]->setValue(0.0);
        }

        m_baseKnownCheck->setChecked(false);
        m_flangeKnownCheck->setChecked(false);
        m_cadPoseKnownCheck->setChecked(false);
        for (QDoubleSpinBox* editor : m_baseEditors) editor->setValue(0.0);
        for (QDoubleSpinBox* editor : m_flangeEditors) editor->setValue(0.0);

        const T_ROBOT_COORS tool = m_driver != nullptr ? m_driver->Tools().tGunTool : T_ROBOT_COORS();
        const bool localToolValid = SetToolEditors(tool);
        m_toolFixedCheck->setCheckState(Qt::PartiallyChecked);
        m_toolPoseKnownCheck->setChecked(false);
        m_toolReadFromController = false;
        m_toolPoseSource = "LocalProjectGunTool";
        m_toolPoseConvention = "Project_RPY_Deg_Unverified";
        m_toolSourceLabel->setText(localToolValid
            ? "来源：当前本地GunTool；尚未只读核对控制器Tool1。"
            : "来源：当前本地GunTool包含非有限或越界值，页面已清零且保持未确认。");
        ApplySa10ConditionLimits();
        m_dhSource = "ManualEntryPending";
        m_speedSource = "SA10Manual";
        UpdateConventionDescription();
    }

    void SetConventionIndexSilently(int index)
    {
        const QSignalBlocker blocker(m_conventionCombo);
        m_conventionCombo->setCurrentIndex(index);
        m_lastConventionIndex = index;
        UpdateConventionDescription();
    }

    void ApplySa10ConditionLimits()
    {
        if (m_conditionCombo == nullptr || m_jointEditors[0][4] == nullptr)
        {
            return;
        }
        const bool handling = m_conditionCombo->currentData().toString() == "Handling";
        const double minimum[kDhJointCount] = {
            -165.0, -80.0, handling ? -165.0 : -80.0, -190.0, -130.0, handling ? -360.0 : -220.0
        };
        static const double maximum[kDhJointCount] = { 165.0, 163.0, 80.0, 190.0, 130.0, 220.0 };
        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            m_jointEditors[joint][4]->setValue(minimum[joint]);
            m_jointEditors[joint][5]->setValue(
                joint == 5 && handling ? 360.0 : maximum[joint]);
        }
        m_limitSource = handling ? "SA10Manual_Handling" : "SA10Manual_Welding";
    }

    void ImportRuntimeDh()
    {
        if (m_driver == nullptr)
        {
            return;
        }

        const std::array<double, kDhParamCount> parameters = KinematicsToParamArray(m_driver->KinematicsParameters());
        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            for (int field = 0; field < 4; ++field)
            {
                const double value = parameters[joint * 4 + field];
                const double limit = field == 0 || field == 2 ? 10000.0 : 3600.0;
                if (!std::isfinite(value) || std::abs(value) > limit)
                {
                    QMessageBox::warning(
                        this,
                        "导入当前DH",
                        QString("当前运行参数J%1第%2项不是有限合理值，已拒绝整次导入。")
                            .arg(joint + 1)
                            .arg(field + 1));
                    return;
                }
            }
        }
        SetConventionIndexSilently(0);
        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            const int baseIndex = joint * 4;
            for (int field = 0; field < 4; ++field)
            {
                m_jointEditors[joint][field]->setValue(parameters[baseIndex + field]);
            }
        }
        m_dhSource = "CurrentRuntimeRobotPara";
        UpdateConventionDescription();
    }

    void LoadDraftOrDefaults()
    {
        PopulateSourceDefaults();
        if (m_candidateId.isEmpty())
        {
            SetStatus(QString(
                "配置库中尚无运动学候选；已载入SA10说明书中的焊接限位和最大速度。"
                "DH区全零表示待填写，不是有效模型。存储范围：robot/%1/KinematicsCandidates")
                .arg(m_robotName), false);
            return;
        }

        KinematicsCandidateSettings settings(m_robotName, m_candidateId);
        if (!settings.HasValues())
        {
            SetStatus(QString("运动学候选记录不存在或为空：robot/%1/KinematicsCandidates/%2")
                .arg(m_robotName, m_candidateId), true);
            return;
        }
        const QString storedModel = settings.value("Model/RobotModel").toString().trimmed();
        const QString storedRobotName = settings.value("Model/RobotName").toString().trimmed();
        if (storedModel.compare("SA10/2000H", Qt::CaseInsensitive) != 0
            || storedRobotName.compare(m_robotName, Qt::CaseInsensitive) != 0)
        {
            SetStatus(QString("候选记录与当前目标不匹配，已拒绝加载：robot/%1/KinematicsCandidates/%2")
                .arg(m_robotName, m_candidateId), true);
            return;
        }
        QString draftError;
        if (!ValidateDraftSettings(settings, &draftError))
        {
            SetStatus(QString("候选记录校验失败：%1；已保留说明书默认页面。记录：robot/%2/KinematicsCandidates/%3")
                .arg(draftError, m_robotName, m_candidateId), true);
            return;
        }
        const QString convention = settings.value("Model/Convention", "StandardDH").toString();
        const int conventionIndex = m_conventionCombo->findData(convention);
        SetConventionIndexSilently(conventionIndex >= 0 ? conventionIndex : 0);
        const QString condition = settings.value("Model/OperatingCondition", "Welding").toString();
        const int conditionIndex = m_conditionCombo->findData(condition);
        m_conditionCombo->setCurrentIndex(conditionIndex >= 0 ? conditionIndex : 0);
        m_sa10ConfirmedCheck->setChecked(false);
        m_toolNameEdit->setText(settings.value("Model/ToolName", "tool1").toString());
        const bool cadPoseKnown = settings.value("CadReferencePose/Known", false).toBool();

        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            const QString prefix = QString("Joint%1/").arg(joint + 1);
            static const char* keys[7] = { "A_mm", "Alpha_deg", "D_mm", "Theta0_deg", "Min_deg", "Max_deg", "MaxSpeed_deg_s" };
            for (int field = 0; field < 7; ++field)
            {
                m_jointEditors[joint][field]->setValue(
                    settings.value(prefix + keys[field], m_jointEditors[joint][field]->value()).toDouble());
            }
            const bool directionKnown = settings.value(prefix + "DirectionKnown", false).toBool();
            const int direction = settings.value(prefix + "Direction", 0).toInt();
            const int directionIndex = directionKnown ? m_directionCombos[joint]->findData(direction) : 0;
            m_directionCombos[joint]->setCurrentIndex(directionIndex >= 0 ? directionIndex : 0);
            if (cadPoseKnown)
            {
                m_cadPoseEditors[joint]->setValue(
                    settings.value(QString("CadReferencePose/J%1_deg").arg(joint + 1)).toDouble());
            }
        }
        m_dhSource = settings.value("Source/Dh", "CandidateDatabase_Unspecified").toString();
        m_limitSource = settings.value("Source/JointLimits", "CandidateDatabase_Unspecified").toString();
        m_speedSource = settings.value("Source/JointSpeed", "CandidateDatabase_Unspecified").toString();

        LoadPose(settings, "Base", m_baseKnownCheck, m_baseEditors);
        LoadPose(settings, "Flange", m_flangeKnownCheck, m_flangeEditors);
        m_cadPoseKnownCheck->setChecked(cadPoseKnown);

        const bool fixedKnown = settings.value("Tool1/FixedKnown", false).toBool();
        m_toolFixedCheck->setCheckState(fixedKnown
            ? (settings.value("Tool1/Fixed", false).toBool() ? Qt::Checked : Qt::Unchecked)
            : Qt::PartiallyChecked);
        const bool toolPoseKnown = settings.value("Tool1/PoseKnown", false).toBool();
        if (toolPoseKnown)
        {
            LoadSixValues(settings, "Tool1", m_toolEditors);
            m_toolReadFromController = settings.value("Tool1/ReadFromController", false).toBool();
            m_toolPoseSource = settings.value("Source/ToolPose", "CandidateDatabase_Unspecified").toString();
            m_toolPoseConvention = settings.value("Tool1/PoseConvention", "CandidateDatabase_Unspecified").toString();
            const QString source = settings.value("Tool1/Source", "候选数据库记录").toString();
            m_toolSourceLabel->setText(QString("来源：%1").arg(source));
        }
        m_toolPoseKnownCheck->setChecked(toolPoseKnown);
        if (!toolPoseKnown)
        {
            m_toolSourceLabel->setText("来源：候选数据库记录没有已人工确认的TCP；当前显示本地GunTool，未作为候选值载入。");
        }

        SetStatus(QString("已加载候选记录：robot/%1/KinematicsCandidates/%2；当前状态仍为未验证，并需重新确认当前机器人本体。")
            .arg(m_robotName, m_candidateId), false);
        UpdateConventionDescription();
    }

    bool ValidateDraftSettings(KinematicsCandidateSettings& settings, QString* error) const
    {
        auto fail = [error](const QString& message) {
            if (error != nullptr)
            {
                *error = message;
            }
            return false;
        };
        auto readValue = [&settings, &fail](
                             const QString& key,
                             const QDoubleSpinBox* editor,
                             double* valueOut) {
            const QVariant raw = settings.value(key);
            bool ok = false;
            const double value = raw.toDouble(&ok);
            if (!raw.isValid() || !ok || !std::isfinite(value)
                || value < editor->minimum() || value > editor->maximum())
            {
                return fail(QString("%1 缺失、非有限或超出允许范围").arg(key));
            }
            if (valueOut != nullptr)
            {
                *valueOut = value;
            }
            return true;
        };

        const QString convention = settings.value("Model/Convention").toString();
        if (m_conventionCombo->findData(convention) < 0)
        {
            return fail("Model/Convention不是支持的StandardDH或CraigMDH");
        }
        const QString condition = settings.value("Model/OperatingCondition").toString();
        if (m_conditionCombo->findData(condition) < 0)
        {
            return fail("Model/OperatingCondition不是支持的焊接或搬运工况");
        }
        if (settings.value("Model/ToolName").toString().compare("tool1", Qt::CaseInsensitive) != 0)
        {
            return fail("Model/ToolName必须是逻辑工具tool1");
        }

        bool hasNonZeroDhValue = false;
        static const char* jointKeys[7] = {
            "A_mm", "Alpha_deg", "D_mm", "Theta0_deg", "Min_deg", "Max_deg", "MaxSpeed_deg_s"
        };
        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            const QString prefix = QString("Joint%1/").arg(joint + 1);
            double values[7]{};
            for (int field = 0; field < 7; ++field)
            {
                if (!readValue(prefix + jointKeys[field], m_jointEditors[joint][field], &values[field]))
                {
                    return false;
                }
                if (field < 4 && std::abs(values[field]) > 1e-9)
                {
                    hasNonZeroDhValue = true;
                }
            }
            if (!(values[4] < values[5]) || !(values[6] > 0.0))
            {
                return fail(QString("J%1的限位或最大速度无效").arg(joint + 1));
            }

            const bool directionKnown = settings.value(prefix + "DirectionKnown", false).toBool();
            if (directionKnown)
            {
                bool ok = false;
                const int direction = settings.value(prefix + "Direction").toInt(&ok);
                if (!ok || (direction != 1 && direction != -1))
                {
                    return fail(QString("J%1的Direction必须为+1或-1").arg(joint + 1));
                }
            }
        }
        if (!hasNonZeroDhValue)
        {
            return fail("24项DH/MDH参数不能全部为0");
        }

        auto validateSix = [&settings, &readValue](
                               const QString& group,
                               const std::array<QDoubleSpinBox*, 6>& editors) {
            static const char* keys[6] = { "X", "Y", "Z", "RX", "RY", "RZ" };
            for (int index = 0; index < 6; ++index)
            {
                if (!readValue(group + "/" + keys[index], editors[index], nullptr))
                {
                    return false;
                }
            }
            return true;
        };
        if (settings.value("Base/Known", false).toBool()
            && !validateSix("Base", m_baseEditors))
        {
            return false;
        }
        if (settings.value("Flange/Known", false).toBool()
            && !validateSix("Flange", m_flangeEditors))
        {
            return false;
        }
        if (settings.value("CadReferencePose/Known", false).toBool())
        {
            for (int joint = 0; joint < kDhJointCount; ++joint)
            {
                if (!readValue(
                        QString("CadReferencePose/J%1_deg").arg(joint + 1),
                        m_cadPoseEditors[joint],
                        nullptr))
                {
                    return false;
                }
            }
        }
        if (settings.value("Tool1/PoseKnown", false).toBool()
            && !validateSix("Tool1", m_toolEditors))
        {
            return false;
        }
        return true;
    }

    static void LoadSixValues(
        KinematicsCandidateSettings& settings,
        const QString& group,
        std::array<QDoubleSpinBox*, 6>& editors)
    {
        static const char* keys[6] = { "X", "Y", "Z", "RX", "RY", "RZ" };
        for (int index = 0; index < 6; ++index)
        {
            editors[index]->setValue(settings.value(group + "/" + keys[index], editors[index]->value()).toDouble());
        }
    }

    static void LoadPose(
        KinematicsCandidateSettings& settings,
        const QString& group,
        QCheckBox* knownCheck,
        std::array<QDoubleSpinBox*, 6>& editors)
    {
        knownCheck->setChecked(settings.value(group + "/Known", false).toBool());
        if (knownCheck->isChecked())
        {
            LoadSixValues(settings, group, editors);
        }
    }

    static void SaveSixValues(
        KinematicsCandidateSettings& settings,
        const QString& group,
        const std::array<QDoubleSpinBox*, 6>& editors)
    {
        static const char* keys[6] = { "X", "Y", "Z", "RX", "RY", "RZ" };
        for (int index = 0; index < 6; ++index)
        {
            settings.setValue(group + "/" + keys[index], editors[index]->value());
        }
    }

    bool SaveDraft(QString* error)
    {
        const QString model = m_modelEdit->text().trimmed();
        const QString toolName = m_toolNameEdit->text().trimmed();
        if (model.isEmpty() || toolName.isEmpty())
        {
            if (error != nullptr) *error = "机器人型号和工具变量名不能为空。";
            return false;
        }
        if (model.compare("SA10/2000H", Qt::CaseInsensitive) != 0)
        {
            if (error != nullptr) *error = "本界面当前只允许建立SA10/2000H候选模型。";
            return false;
        }
        if (toolName.compare("tool1", Qt::CaseInsensitive) != 0)
        {
            if (error != nullptr) *error = "本界面当前固定建立逻辑工具Tool1候选。";
            return false;
        }
        if (!m_sa10ConfirmedCheck->isChecked())
        {
            if (error != nullptr) *error = "请先确认当前所选机器人本体确实为SA10/2000H，避免把说明书限位应用到其他机器人。";
            return false;
        }

        bool hasNonZeroDhValue = false;
        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            for (int field = 0; field < 4; ++field)
            {
                if (std::abs(m_jointEditors[joint][field]->value()) > 1e-9)
                {
                    hasNonZeroDhValue = true;
                }
            }
            const double minimum = m_jointEditors[joint][4]->value();
            const double maximum = m_jointEditors[joint][5]->value();
            const double speed = m_jointEditors[joint][6]->value();
            if (!(minimum < maximum))
            {
                if (error != nullptr) *error = QString("J%1的最小角必须小于最大角。").arg(joint + 1);
                return false;
            }
            if (!(speed > 0.0))
            {
                if (error != nullptr) *error = QString("J%1的最大速度必须大于0。").arg(joint + 1);
                return false;
            }
        }
        if (!hasNonZeroDhValue)
        {
            if (error != nullptr) *error = "DH/MDH参数仍全部为0，请先填写厂家给出的六轴参数。";
            return false;
        }

        const QString candidateId = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
        KinematicsCandidateSettings settings(m_robotName, candidateId);
        settings.setValue("Model/Version", 1);
        settings.setValue("Model/Status", "CandidateUnvalidated");
        settings.setValue("Model/RobotName", m_robotName);
        settings.setValue("Model/RobotModel", model);
        settings.setValue("Model/Convention", m_conventionCombo->currentData().toString());
        settings.setValue("Model/OperatingCondition", m_conditionCombo->currentData().toString());
        settings.setValue("Model/TargetConfirmed", true);
        settings.setValue("Model/TargetConfirmedAt", QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
        settings.setValue("Model/LengthUnit", "mm");
        settings.setValue("Model/AngleUnit", "deg");
        settings.setValue("Model/ToolName", toolName);
        settings.setValue("Model/UpdatedAt", QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
        settings.setValue("Source/Dh", m_dhSource);
        settings.setValue("Source/JointLimits", m_limitSource);
        settings.setValue("Source/JointSpeed", m_speedSource);
        settings.setValue("Source/Sa10Manual", "SA10-2000H焊接机器人使用说明书.pdf");
        settings.setValue("Source/Geometry", "SA10-2000H演示模型/整机STEP及J0-J6零件模型");
        settings.setValue("Source/BaseAndFlangeDimensions", "SA10-2000H-2D模型");
        settings.setValue("Source/Workspace", "SA10-2000H工作空间图");

        for (int joint = 0; joint < kDhJointCount; ++joint)
        {
            const QString prefix = QString("Joint%1/").arg(joint + 1);
            static const char* keys[7] = { "A_mm", "Alpha_deg", "D_mm", "Theta0_deg", "Min_deg", "Max_deg", "MaxSpeed_deg_s" };
            for (int field = 0; field < 7; ++field)
            {
                settings.setValue(prefix + keys[field], m_jointEditors[joint][field]->value());
            }
            const int direction = m_directionCombos[joint]->currentData().toInt();
            settings.setValue(prefix + "DirectionKnown", direction == 1 || direction == -1);
            if (direction == 1 || direction == -1)
            {
                settings.setValue(prefix + "Direction", direction);
            }
            if (m_cadPoseKnownCheck->isChecked())
            {
                settings.setValue(
                    QString("CadReferencePose/J%1_deg").arg(joint + 1),
                    m_cadPoseEditors[joint]->value());
            }
        }

        SaveOptionalPose(settings, "Base", m_baseKnownCheck, m_baseEditors);
        settings.setValue("Base/TransformDirection", "T_RobotBase_DhBase");
        SaveOptionalPose(settings, "Flange", m_flangeKnownCheck, m_flangeEditors);
        settings.setValue("Flange/TransformDirection", "T_DhEnd_Flange");
        settings.setValue("CadReferencePose/Known", m_cadPoseKnownCheck->isChecked());
        settings.setValue("CadReferencePose/AngleUnit", "deg");

        const bool fixedKnown = m_toolFixedCheck->checkState() != Qt::PartiallyChecked;
        settings.setValue("Tool1/FixedKnown", fixedKnown);
        if (fixedKnown)
        {
            settings.setValue("Tool1/Fixed", m_toolFixedCheck->isChecked());
        }
        else
        {
            settings.remove("Tool1/Fixed");
        }
        settings.setValue("Tool1/ResolvedVariableNameKnown", false);
        settings.setValue("Tool1/PoseKnown", m_toolPoseKnownCheck->isChecked());
        settings.setValue("Tool1/PoseConvention", m_toolPoseConvention);
        settings.setValue("Tool1/ReadFromController", m_toolReadFromController);
        settings.setValue("Tool1/Source", m_toolSourceLabel->text().mid(QString("来源：").size()));
        settings.setValue("Source/ToolPose", m_toolPoseSource);
        if (m_toolPoseKnownCheck->isChecked())
        {
            SaveSixValues(settings, "Tool1", m_toolEditors);
        }
        settings.setValue("Validation/Validated", false);
        settings.setValue("Validation/Reason", "AwaitingControllerFkAndModelValidation");
        QString syncError;
        if (!settings.sync(&syncError))
        {
            if (error != nullptr)
            {
                *error = QString("候选参数写入数据库失败：%1").arg(syncError);
            }
            return false;
        }
        if (!ConfigDatabase::WriteScopedSetting(
                QStringLiteral("robot"), m_robotName,
                QStringLiteral("KinematicsCandidates"), QStringLiteral("LatestId"),
                candidateId, QStringLiteral("string")))
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("候选参数已写入，但更新最新候选索引失败。");
            }
            return false;
        }
        m_candidateId = candidateId;
        return true;
    }

    void ReadControllerTool1()
    {
        if (m_driver == nullptr)
        {
            QMessageBox::warning(this, "读取Tool1", "当前没有机器人驱动。候选参数未改变。");
            return;
        }
        if (!m_driver->Supports(RobotDriverCapability::ToolDataRead))
        {
            QMessageBox::warning(this, "读取Tool1",
                "当前机器人品牌底层缺少“工具数据读取”适配能力，功能已限制。候选参数未改变。");
            return;
        }

        QString leaseError;
        const auto operationLease = RobotOperationLease::TryAcquire(
            m_driver, QStringLiteral("只读获取 Tool1 到候选运动学模型"), &leaseError);
        if (!operationLease)
        {
            QMessageBox::warning(this, "读取Tool1", leaseError);
            return;
        }

        T_ROBOT_COORS tool1;
        if (!m_driver->GetToolData(1, tool1))
        {
            const QString detail = DecodeRobotMessageText(m_driver->GetLastRobotError()).trimmed();
            QMessageBox::warning(
                this,
                "读取Tool1",
                detail.isEmpty()
                    ? "读取控制器Tool1失败，请确认机器人已连接且SDK支持工具读取。"
                    : QString("读取控制器Tool1失败：\n%1").arg(detail));
            return;
        }

        const double values[6] = { tool1.dX, tool1.dY, tool1.dZ, tool1.dRX, tool1.dRY, tool1.dRZ };
        for (int index = 0; index < 6; ++index)
        {
            const double limit = index < 3 ? 5000.0 : 3600.0;
            if (!std::isfinite(values[index]) || std::abs(values[index]) > limit)
            {
                QMessageBox::warning(
                    this,
                    "读取Tool1",
                    QString("控制器Tool1返回的第%1项不是有限合理值，已拒绝填入候选页面。").arg(index + 1));
                return;
            }
        }

        SetToolEditors(tool1);
        m_toolFixedCheck->setCheckState(Qt::PartiallyChecked);
        m_toolPoseKnownCheck->setChecked(false);
        m_toolReadFromController = true;
        m_toolPoseSource = "ControllerRead";
        m_toolPoseConvention = "ControllerNative_Unverified";
        m_toolSourceLabel->setText(
            QString("来源：控制器Tool编号1，只读时间 %1；实际变量名及Fixed字段当前通用驱动未返回。")
                .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")));
        SetStatus("已只读获取控制器Tool1并填入页面；实际变量名和姿态约定仍待验证，请核对后手工勾选TCP确认。尚未保存、未写机器人。", false);
    }

    bool SetToolEditors(const T_ROBOT_COORS& tool)
    {
        const double values[6] = { tool.dX, tool.dY, tool.dZ, tool.dRX, tool.dRY, tool.dRZ };
        for (int index = 0; index < 6; ++index)
        {
            const double limit = index < 3 ? 5000.0 : 3600.0;
            if (!std::isfinite(values[index]) || std::abs(values[index]) > limit)
            {
                for (QDoubleSpinBox* editor : m_toolEditors)
                {
                    editor->setValue(0.0);
                }
                return false;
            }
        }
        for (int index = 0; index < 6; ++index)
        {
            m_toolEditors[index]->setValue(values[index]);
        }
        return true;
    }

    void UpdateConventionDescription()
    {
        if (m_conventionDescription == nullptr || m_conventionCombo == nullptr)
        {
            return;
        }
        const bool modified = m_conventionCombo->currentData().toString() == "CraigMDH";
        m_conventionDescription->setText(modified
            ? "Craig MDH行定义：a(i-1)、alpha(i-1)、d(i)、theta0(i)。当前运行链仍是Standard DH；MDH在本页面仅作为候选记录，验证与启用时必须使用MDH建链。"
            : "Standard DH行定义：a(i)、alpha(i)、d(i)、theta0(i)。与当前KDL::Frame::DH运行链一致，但仍须核对厂家行号定义和零位。 ");
    }

    void SetStatus(const QString& text, bool error)
    {
        m_statusLabel->setText(text);
        m_statusLabel->setStyleSheet(error ? "color:#b00020;" : "color:#335c33;");
    }

    static void SaveOptionalPose(
        KinematicsCandidateSettings& settings,
        const QString& group,
        QCheckBox* knownCheck,
        const std::array<QDoubleSpinBox*, 6>& editors)
    {
        const bool known = knownCheck->isChecked();
        settings.setValue(group + "/Known", known);
        settings.setValue(group + "/RotationConvention", "Project_RPY_Deg_Unverified");
        if (known)
        {
            SaveSixValues(settings, group, editors);
        }
    }

    QString FindLatestCandidateId() const
    {
        QString candidateId;
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("robot"), m_robotName,
            QStringLiteral("KinematicsCandidates"), QStringLiteral("LatestId"),
            &candidateId);
        return candidateId.trimmed();
    }

private:
    RobotDriverAdaptor* m_driver = nullptr;
    QString m_robotName;
    QString m_candidateId;
    QString m_dhSource;
    QString m_limitSource;
    QString m_speedSource;
    QString m_toolPoseSource;
    QString m_toolPoseConvention;
    bool m_saved = false;
    bool m_toolReadFromController = false;
    int m_lastConventionIndex = 0;

    QLineEdit* m_modelEdit = nullptr;
    QLineEdit* m_toolNameEdit = nullptr;
    QComboBox* m_conventionCombo = nullptr;
    QComboBox* m_conditionCombo = nullptr;
    QLabel* m_conventionDescription = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_toolSourceLabel = nullptr;
    QTableWidget* m_jointTable = nullptr;
    std::array<std::array<QDoubleSpinBox*, 7>, kDhJointCount> m_jointEditors{};
    std::array<QComboBox*, kDhJointCount> m_directionCombos{};
    QCheckBox* m_baseKnownCheck = nullptr;
    QCheckBox* m_flangeKnownCheck = nullptr;
    QCheckBox* m_cadPoseKnownCheck = nullptr;
    QCheckBox* m_toolFixedCheck = nullptr;
    QCheckBox* m_toolPoseKnownCheck = nullptr;
    QCheckBox* m_sa10ConfirmedCheck = nullptr;
    std::array<QDoubleSpinBox*, 6> m_baseEditors{};
    std::array<QDoubleSpinBox*, 6> m_flangeEditors{};
    std::array<QDoubleSpinBox*, 6> m_cadPoseEditors{};
    std::array<QDoubleSpinBox*, 6> m_toolEditors{};
};
}

FunctionTestDialog::FunctionTestDialog(
    ContralUnit* pContralUnit,
    int unitIndex,
    CameraFrameCache* cameraCache,
    QWidget* parent,
    std::function<bool(const QString&, int)> workflowLauncher,
    std::function<CameraFrameCache*(int)> cameraCacheResolver)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_unitIndex(unitIndex)
    , m_pCameraCache(cameraCache)
    , m_workflowLauncher(std::move(workflowLauncher))
    , m_cameraCacheResolver(std::move(cameraCacheResolver))
{
    setWindowTitle("功能测试");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(760, 560), 0.76, 0.74);

    setStyleSheet(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #9ED8DB; }"
        "QPushButton { background: #1F3542; color: #F4FAFA; border: 1px solid #3C6475; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2C5364; border-color: #63C7D1; }"
        "QPushButton:pressed { background: #16303A; }"
        "QPushButton:disabled { background: #171f27; color: #68757e; border-color: #2b3943; }"
        "QPlainTextEdit { background: #0B1117; color: #BFE7EA; border: 1px solid #2E4656; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #B8C7CC; }");

    QVBoxLayout* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(12, 12, 12, 12);
    outerLayout->setSpacing(10);

    m_pPageTitleLabel = new QLabel("机器人功能测试区");
    m_pPageTitleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #F4FAFA;");
    outerLayout->addWidget(m_pPageTitleLabel);

    m_pPageHintLabel = new QLabel("这里集中放置设置速度、读取位置、原生程序、诊断、往返运动、零位运动等测试功能；入口按当前机器人底层声明的适配能力开放。");
    m_pPageHintLabel->setWordWrap(true);
    outerLayout->addWidget(m_pPageHintLabel);

    m_pTestTabs = new QTabWidget(this);
    QWidget* singleTestPage = new QWidget(m_pTestTabs);
    QVBoxLayout* singleTestLayout = new QVBoxLayout(singleTestPage);
    singleTestLayout->setContentsMargins(0, 8, 0, 0);
    singleTestLayout->setSpacing(10);

    QScrollArea* commandScrollArea = new QScrollArea(this);
    commandScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(commandScrollArea);

    m_pCommandContent = new QWidget(commandScrollArea);
    m_pCommandContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    QVBoxLayout* commandLayout = new QVBoxLayout(m_pCommandContent);
    commandLayout->setContentsMargins(0, 0, 8, 0);
    commandLayout->setSpacing(10);
    commandLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    QGridLayout* groupLayout = new QGridLayout();
    commandLayout->addLayout(groupLayout);

    QGroupBox* basicGroup = new QGroupBox("基础通讯/状态");
    QGridLayout* basicLayout = new QGridLayout(basicGroup);
    QPushButton* setSpeedBtn = CreateTestButton("设置速度");
    QPushButton* getPosBtn = CreateTestButton("读取当前位置");
    QPushButton* getPulseBtn = CreateTestButton("读取关节脉冲");
    QPushButton* checkDoneBtn = CreateTestButton("检查运行完成");
    QPushButton* setGetIntBtn = CreateTestButton("写读INT寄存器");
    QPushButton* callJobBtn = CreateTestButton("调用任务");
    QPushButton* uploadLsBtn = CreateTestButton("发送原生程序");
    QPushButton* curposDiagBtn = CreateTestButton("机器人诊断");
    QPushButton* timestampDiagBtn = CreateTestButton("状态时间轴+相机时间轴");
    basicLayout->addWidget(setSpeedBtn, 0, 0);
    basicLayout->addWidget(getPosBtn, 0, 1);
    basicLayout->addWidget(getPulseBtn, 1, 0);
    basicLayout->addWidget(checkDoneBtn, 1, 1);
    basicLayout->addWidget(setGetIntBtn, 2, 0);
    basicLayout->addWidget(callJobBtn, 2, 1);
    basicLayout->addWidget(uploadLsBtn, 3, 0);
    basicLayout->addWidget(curposDiagBtn, 3, 1);
    basicLayout->addWidget(timestampDiagBtn, 4, 0, 1, 2);
    groupLayout->addWidget(basicGroup, 0, 0);

    RobotDriverAdaptor* initialDriver = nullptr;
    if (m_pContralUnit != nullptr
        && m_unitIndex >= 0
        && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        initialDriver = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    }
    const bool canUploadNativeProgram = initialDriver != nullptr
        && initialDriver->Supports(RobotDriverCapability::NativeProgramUpload);
    const bool canRunDiagnostic = initialDriver != nullptr
        && initialDriver->Supports(RobotDriverCapability::DiagnosticCommand);
	auto configureCapabilityButton = [initialDriver](
		QPushButton* button,
		std::initializer_list<RobotDriverCapability> capabilities,
		const QString& featureName)
		{
			bool enabled = initialDriver != nullptr;
			qulonglong requiredMask = 0;
			for (const RobotDriverCapability capability : capabilities)
			{
				requiredMask |= static_cast<qulonglong>(RobotDriverCapabilityBit(capability));
				enabled = enabled && initialDriver->Supports(capability);
			}
			button->setProperty("requiredRobotCapabilities", requiredMask);
			button->setProperty("robotCapabilityFeatureName", featureName);
			button->setEnabled(enabled);
			button->setToolTip(enabled
				? QString()
				: QString("当前机器人底层未声明“%1”所需的全部适配能力，功能已禁用。")
					.arg(featureName));
		};
	configureCapabilityButton(setSpeedBtn,
		{ RobotDriverCapability::TeachPendantSpeedControl }, QStringLiteral("设置速度"));
	configureCapabilityButton(getPosBtn,
		{ RobotDriverCapability::PassiveState }, QStringLiteral("读取当前位置"));
	configureCapabilityButton(getPulseBtn,
		{ RobotDriverCapability::PassiveState }, QStringLiteral("读取关节脉冲"));
	configureCapabilityButton(checkDoneBtn,
		{ RobotDriverCapability::PassiveState }, QStringLiteral("检查运行状态"));
	configureCapabilityButton(setGetIntBtn,
		{ RobotDriverCapability::IntegerRegister }, QStringLiteral("写读整数寄存器"));
	configureCapabilityButton(callJobBtn,
		{ RobotDriverCapability::NativeProgramExecution,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort }, QStringLiteral("调用任务"));
	configureCapabilityButton(timestampDiagBtn,
		{ RobotDriverCapability::PassiveState }, QStringLiteral("状态时间轴诊断"));
    uploadLsBtn->setEnabled(canUploadNativeProgram);
    curposDiagBtn->setEnabled(canRunDiagnostic);
    uploadLsBtn->setToolTip(canUploadNativeProgram
        ? QString()
        : QStringLiteral("当前机器人驱动未实现原生程序上传适配接口。"));
    curposDiagBtn->setToolTip(canRunDiagnostic
        ? QString()
        : QStringLiteral("当前机器人驱动未实现诊断命令适配接口。"));

    QGroupBox* motionGroup = new QGroupBox("运动测试");
    QGridLayout* motionLayout = new QGridLayout(motionGroup);
    m_pMovlTestBtn = CreateTestButton("MOVL往返测试");
    m_pMovjTestBtn = CreateTestButton("MOVJ J2/J3 +5deg");
    m_pMoveZeroBtn = CreateTestButton("运动到零位");
    motionLayout->addWidget(m_pMovlTestBtn, 0, 0);
    motionLayout->addWidget(m_pMovjTestBtn, 1, 0);
    motionLayout->addWidget(m_pMoveZeroBtn, 2, 0);
    m_motionButtons = { m_pMovlTestBtn, m_pMovjTestBtn, m_pMoveZeroBtn, callJobBtn };
	configureCapabilityButton(m_pMovlTestBtn,
		{ RobotDriverCapability::LinearMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort }, QStringLiteral("MOVL往返测试"));
	configureCapabilityButton(m_pMovjTestBtn,
		{ RobotDriverCapability::JointMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort }, QStringLiteral("MOVJ测试"));
	configureCapabilityButton(m_pMoveZeroBtn,
		{ RobotDriverCapability::JointMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort }, QStringLiteral("运动到零位"));
    groupLayout->addWidget(motionGroup, 0, 1);

    QGroupBox* offlineGroup = new QGroupBox("离线数据处理");
    QGridLayout* offlineLayout = new QGridLayout(offlineGroup);
    QPushButton* filterLaserBtn = CreateTestButton("精测点云处理");
    QPushButton* currentFrameFilterBtn = CreateTestButton("当前帧点云滤波");
    offlineLayout->addWidget(filterLaserBtn, 0, 0);
    offlineLayout->addWidget(currentFrameFilterBtn, 1, 0);
    groupLayout->addWidget(offlineGroup, 1, 1);

    QGroupBox* kinematicsGroup = new QGroupBox("运动学/DH");
    QGridLayout* kinematicsLayout = new QGridLayout(kinematicsGroup);
    QPushButton* saveKinematicsSampleBtn = CreateTestButton("保存关节+直角");
    QPushButton* fitDhBtn = CreateTestButton("拟合DH参数");
    QPushButton* editKinematicsBtn = CreateTestButton("填写DH/MDH参数");
	configureCapabilityButton(saveKinematicsSampleBtn,
		{ RobotDriverCapability::PassiveState }, QStringLiteral("保存运动学样本"));
    kinematicsLayout->addWidget(saveKinematicsSampleBtn, 0, 0);
    kinematicsLayout->addWidget(fitDhBtn, 1, 0);
    kinematicsLayout->addWidget(editKinematicsBtn, 2, 0);
    groupLayout->addWidget(kinematicsGroup, 1, 0);
    commandLayout->addStretch(1);
    commandScrollArea->setWidget(m_pCommandContent);
    singleTestLayout->addWidget(commandScrollArea, 1);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1200);
    m_pLogText->setPlainText("功能测试日志：等待操作...");
    m_pLogText->setMinimumHeight(130);
    m_pLogText->setMaximumHeight(220);
    singleTestLayout->addWidget(m_pLogText);
    m_pTestTabs->addTab(singleTestPage, QStringLiteral("单项测试"));
    m_pAdaptorAcceptancePage = CreateAdaptorAcceptancePage();
    m_pTestTabs->addTab(m_pAdaptorAcceptancePage, QStringLiteral("机器人适配测试"));
    outerLayout->addWidget(m_pTestTabs, 1);

    connect(setSpeedBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucSetTpSpeedTest);
    connect(getPosBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucGetCurrentPosTest);
    connect(getPulseBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucGetCurrentPulseTest);
    connect(checkDoneBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCheckDoneTest);
    connect(setGetIntBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucSetGetIntTest);
    connect(callJobBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCallJobTest);
    connect(uploadLsBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucUploadLsTest);
    connect(curposDiagBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCurposDiagnosticTest);
    connect(timestampDiagBtn, &QPushButton::clicked, this, &FunctionTestDialog::RobotCameraTimestampDiagnosticTest);
    connect(m_pMovlTestBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucMovlTest);
    connect(m_pMovjTestBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucMovjTest);
    connect(m_pMoveZeroBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucMoveZeroTest);
    connect(editKinematicsBtn, &QPushButton::clicked, this, &FunctionTestDialog::EditKinematicsParameters);
    connect(saveKinematicsSampleBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCaptureKinematicsSample);
    connect(fitDhBtn, &QPushButton::clicked, this, &FunctionTestDialog::FitDhParametersFromSamples);
    connect(filterLaserBtn, &QPushButton::clicked, this, &FunctionTestDialog::OpenLaserWeldFilterTest);
    connect(currentFrameFilterBtn, &QPushButton::clicked, this, &FunctionTestDialog::ExportCurrentCameraFramePointFilterTest);

    m_pMotionStateTimer = new QTimer(this);
    m_pMotionStateTimer->setInterval(200);
    connect(m_pMotionStateTimer, &QTimer::timeout, this, &FunctionTestDialog::RefreshMotionButtonState);
    m_pMotionStateTimer->start();
    RefreshMotionButtonState();
}

QWidget* FunctionTestDialog::CreateAdaptorAcceptancePage()
{
    QWidget* page = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setContentsMargins(4, 8, 4, 4);
    layout->setSpacing(8);

    m_pAdaptorAcceptanceTitleLabel = new QLabel(QStringLiteral("机器人适配测试 — %1")
        .arg(AdaptorAcceptanceStorageRobotName()), page);
    QFont titleFont = m_pAdaptorAcceptanceTitleLabel->font();
    titleFont.setPointSize(titleFont.pointSize() + 4);
    titleFont.setBold(true);
    m_pAdaptorAcceptanceTitleLabel->setFont(titleFont);
    layout->addWidget(m_pAdaptorAcceptanceTitleLabel);

    QLabel* description = new QLabel(
        "所有机器人品牌使用同一套分阶段验收流程。自动证据和人工结论会按轮次写入数据库，"
        "并可随时导出 Markdown/JSON 报告，用于定位适配层或品牌底层缺失功能。",
        page);
    description->setWordWrap(true);
    layout->addWidget(description);

    QGroupBox* robotTargetGroup = new QGroupBox("测试机器人", page);
    QGridLayout* robotTargetLayout = new QGridLayout(robotTargetGroup);
    m_pAdaptorAcceptanceRobotCombo = new QComboBox(robotTargetGroup);
    m_pAdaptorAcceptanceRobotSummary = new QLabel(robotTargetGroup);
    m_pAdaptorAcceptanceRobotSummary->setWordWrap(true);
    int selectedRobotComboIndex = -1;
    if (m_pContralUnit != nullptr)
    {
        for (int index = 0;
            index < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()); ++index)
        {
            const auto& unit = m_pContralUnit->m_vtContralUnitInfo[index];
            const auto* driver = static_cast<RobotDriverAdaptor*>(unit.pUnitDriver);
            if (unit.sContralUnitType != "R" || driver == nullptr)
            {
                continue;
            }
            const QString unitName = QString::fromLocal8Bit(unit.sUnitName.c_str());
            const QString driverName = QString::fromStdString(
                driver->DriverDescriptor().displayName);
            m_pAdaptorAcceptanceRobotCombo->addItem(
                QStringLiteral("%1 — %2").arg(unitName, driverName), index);
            if (index == m_unitIndex)
            {
                selectedRobotComboIndex = m_pAdaptorAcceptanceRobotCombo->count() - 1;
            }
        }
    }
    if (selectedRobotComboIndex >= 0)
    {
        m_pAdaptorAcceptanceRobotCombo->setCurrentIndex(selectedRobotComboIndex);
    }
    else if (m_pAdaptorAcceptanceRobotCombo->count() > 0)
    {
        m_pAdaptorAcceptanceRobotCombo->setCurrentIndex(0);
        m_unitIndex = m_pAdaptorAcceptanceRobotCombo->currentData().toInt();
        const auto& selectedUnit = m_pContralUnit->m_vtContralUnitInfo[m_unitIndex];
        m_pCameraCache = m_cameraCacheResolver
            ? m_cameraCacheResolver(selectedUnit.nUnitNo) : nullptr;
    }
    else
    {
        m_pAdaptorAcceptanceRobotCombo->addItem("无可用机器人", -1);
        m_pAdaptorAcceptanceRobotCombo->setEnabled(false);
    }
    robotTargetLayout->addWidget(new QLabel("选择机器人：", robotTargetGroup), 0, 0);
    robotTargetLayout->addWidget(m_pAdaptorAcceptanceRobotCombo, 0, 1);
    robotTargetLayout->addWidget(m_pAdaptorAcceptanceRobotSummary, 0, 2, 1, 3);
    robotTargetLayout->setColumnStretch(2, 1);
    m_pAdaptorAcceptanceRunCombo = new QComboBox(robotTargetGroup);
    m_pAdaptorAcceptanceRunCombo->setMinimumContentsLength(22);
    m_pAdaptorAcceptanceSaveStatus = new QLabel(robotTargetGroup);
    m_pAdaptorAcceptanceSaveStatus->setWordWrap(true);
    robotTargetLayout->addWidget(new QLabel("验收记录：", robotTargetGroup), 1, 0);
    robotTargetLayout->addWidget(m_pAdaptorAcceptanceRunCombo, 1, 1);
    robotTargetLayout->addWidget(m_pAdaptorAcceptanceSaveStatus, 1, 2, 1, 3);
    layout->addWidget(robotTargetGroup);

    QLabel* safety = new QLabel(
        "统一验收按阶段执行，绝不会自动连续运行。程序下发步骤只允许加载检查；低速运动、扫描和实际焊接"
        "分别要求人工确认。所有寄存器写测试必须先备份原值、写后回读并恢复原值。能力缺失时只能记录为受限，不能人工改成通过。");
    safety->setWordWrap(true);
    safety->setStyleSheet(
        "QLabel { background:#3a2b13; color:#ffe5a3; border:1px solid #8a6728; "
        "border-radius:8px; padding:9px; }");
    layout->addWidget(safety);

    QSplitter* stageSplitter = new QSplitter(Qt::Horizontal, page);
    QWidget* navigationPanel = new QWidget(stageSplitter);
    QVBoxLayout* navigationLayout = new QVBoxLayout(navigationPanel);
    navigationLayout->setContentsMargins(0, 0, 6, 0);
    QLabel* navigationTitle = new QLabel("验收流程", navigationPanel);
    navigationTitle->setStyleSheet("font-size:16px; font-weight:bold; color:#9ED8DB;");
    navigationLayout->addWidget(navigationTitle);
    m_pAdaptorAcceptanceStageList = new QListWidget(navigationPanel);
    m_pAdaptorAcceptanceStageList->setSelectionMode(QAbstractItemView::SingleSelection);
    m_pAdaptorAcceptanceStageList->setSpacing(3);
    for (int stage = 0; stage < kAdaptorAcceptanceStageCount; ++stage)
    {
        m_pAdaptorAcceptanceStageList->addItem(
            QString::fromUtf8(kAdaptorAcceptanceStageNames[stage]));
    }
    navigationLayout->addWidget(m_pAdaptorAcceptanceStageList, 1);

    QWidget* detailPanel = new QWidget(stageSplitter);
    QVBoxLayout* detailLayout = new QVBoxLayout(detailPanel);
    detailLayout->setContentsMargins(8, 0, 0, 0);
    detailLayout->setSpacing(8);
    m_pAdaptorAcceptanceStageTitle = new QLabel(detailPanel);
    m_pAdaptorAcceptanceStageTitle->setStyleSheet(
        "font-size:18px; font-weight:bold; color:#F4FAFA;");
    detailLayout->addWidget(m_pAdaptorAcceptanceStageTitle);
    m_pAdaptorAcceptanceStageDescription = new QLabel(detailPanel);
    m_pAdaptorAcceptanceStageDescription->setWordWrap(true);
    detailLayout->addWidget(m_pAdaptorAcceptanceStageDescription);
    m_pAdaptorAcceptanceStageGate = new QLabel(detailPanel);
    m_pAdaptorAcceptanceStageGate->setWordWrap(true);
    m_pAdaptorAcceptanceStageGate->setStyleSheet(
        "QLabel { background:#13232d; border:1px solid #2E4656; border-radius:7px; padding:7px; }");
    detailLayout->addWidget(m_pAdaptorAcceptanceStageGate);
    m_pAdaptorAcceptanceStageStatus = new QLabel(detailPanel);
    m_pAdaptorAcceptanceStageStatus->setStyleSheet("font-weight:bold;");
    detailLayout->addWidget(m_pAdaptorAcceptanceStageStatus);

    m_pAdaptorAcceptanceStageStack = new QStackedWidget(detailPanel);
    auto addStagePage = [this](const QString& note) -> QFormLayout*
        {
            QWidget* stagePage = new QWidget(m_pAdaptorAcceptanceStageStack);
            QVBoxLayout* stagePageLayout = new QVBoxLayout(stagePage);
            stagePageLayout->setContentsMargins(0, 4, 0, 4);
            QLabel* noteLabel = new QLabel(note, stagePage);
            noteLabel->setWordWrap(true);
            noteLabel->setStyleSheet(
                "QLabel { color:#B8C7CC; background:#0B1117; border:1px solid #243b49; border-radius:7px; padding:7px; }");
            stagePageLayout->addWidget(noteLabel);
            QFormLayout* form = new QFormLayout();
            form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
            stagePageLayout->addLayout(form);
            stagePageLayout->addStretch(1);
            m_pAdaptorAcceptanceStageStack->addWidget(stagePage);
            return form;
        };

    addStagePage("无可编辑参数。端点来自所选机器人数据库配置；开始测试后只连接并回读，不主动断开。 ");
    addStagePage("先在示教器当前工程创建一个专用测试JOB。执行时选择该非main程序，先下载，再按原路径同名回传并回读校验；不会新增或删除工程文件。 ");
    QFormLayout* programForm = addStagePage(
        "生成到当前位置的单点原生程序，只下发供示教器加载/语法检查，不启动机器人。 ");
    m_pAdaptorAcceptanceProgramSpeedSpin = new QDoubleSpinBox();
    m_pAdaptorAcceptanceProgramSpeedSpin->setRange(60.0, 600.0);
    m_pAdaptorAcceptanceProgramSpeedSpin->setDecimals(0);
    m_pAdaptorAcceptanceProgramSpeedSpin->setSuffix(" mm/min");
    m_pAdaptorAcceptanceProgramSpeedSpin->setValue(60.0);
    programForm->addRow("程序测试速度：", m_pAdaptorAcceptanceProgramSpeedSpin);
    addStagePage("无可编辑参数。读取两次当前位置、当前关节脉冲、通用完成状态和已声明的结构化状态。 ");
    QFormLayout* motionForm = addStagePage(
        "必须确认机器人周围安全、实体急停已解除并握住示教器。确定后经适配层检查急停和报警，按需复位并回读；通过后自动切换模式、伺服上电；"
        "第一次执行向基坐标+Y外移，第二次执行返回记录的原始位姿。支持结构化状态的品牌会恢复执行前的模式和伺服状态。 ");
    m_pAdaptorAcceptanceDistanceSpin = new QDoubleSpinBox();
    m_pAdaptorAcceptanceDistanceSpin->setRange(1.0, 50.0);
    m_pAdaptorAcceptanceDistanceSpin->setDecimals(1);
    m_pAdaptorAcceptanceDistanceSpin->setSuffix(" mm（基坐标+Y）");
    m_pAdaptorAcceptanceDistanceSpin->setValue(10.0);
    m_pAdaptorAcceptanceSpeedSpin = new QDoubleSpinBox();
    m_pAdaptorAcceptanceSpeedSpin->setRange(60.0, 600.0);
    m_pAdaptorAcceptanceSpeedSpin->setDecimals(0);
    m_pAdaptorAcceptanceSpeedSpin->setSuffix(" mm/min");
    m_pAdaptorAcceptanceSpeedSpin->setValue(60.0);
    motionForm->addRow("低速位移：", m_pAdaptorAcceptanceDistanceSpin);
    motionForm->addRow("线速度：", m_pAdaptorAcceptanceSpeedSpin);
    m_pAdaptorJointMotionBtn = CreateTestButton("关节专项：J1 +0.5°（1%）");
    m_pAdaptorJointMotionStatus = new QLabel();
    m_pAdaptorJointMotionStatus->setWordWrap(true);
    motionForm->addRow("独立关节往返验收：", m_pAdaptorJointMotionBtn);
    motionForm->addRow("关节结论：", m_pAdaptorJointMotionStatus);
    connect(m_pAdaptorJointMotionBtn, &QPushButton::clicked, this,
        [this]() { RunAdaptorSafeLinearMotion(true); });
    m_pAdaptorJointCancelBtn = CreateTestButton("结束关节专项（不自动返回）");
    motionForm->addRow("放弃本次往返：", m_pAdaptorJointCancelBtn);
    connect(m_pAdaptorJointCancelBtn, &QPushButton::clicked, this, [this]()
        {
            if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning || !m_adaptorJointMovedOut) { return; }
            if (QMessageBox::warning(this, "结束关节专项",
                "机器人将保持当前位置，不发送返回运动。原点将作废，本次关节专项记为失败，请现场核对后重新测试。",
                QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Ok) { return; }
            m_adaptorJointMotionState = "fail";
            m_adaptorJointMotionEvidence += "\n人工结束关节专项，未自动返回；原点作废，需现场处理。";
            m_adaptorJointOriginalPulseValid = false;
            m_adaptorJointMovedOut = false;
            m_adaptorJointLease.reset();
            SaveAdaptorAcceptanceRun(); RefreshMotionButtonState(); RefreshAdaptorAcceptanceUi();
        });
    QWidget* motionPage = motionForm->parentWidget();
    auto* motionPageLayout = qobject_cast<QVBoxLayout*>(motionPage->layout());
    const auto modeControls = RobotAdaptorAcceptanceLayout::CreateModeControls(motionPage);
    // Keep this full-width panel outside QFormLayout: global compact-form
    // defaults deliberately limit ordinary fields to their size hints.
    motionPageLayout->insertWidget(motionPageLayout->count() - 1, modeControls.panel);
    m_pAdaptorModeCombinationCombo = modeControls.combo;
    m_pAdaptorModeSingleBtn = modeControls.buttons[0];
    m_pAdaptorModeBatchBtn = modeControls.buttons[1];
    m_pAdaptorModeApplyBtn = modeControls.buttons[2];
    m_pAdaptorModeStopBtn = modeControls.buttons[3];
    connect(m_pAdaptorModeSingleBtn, &QPushButton::clicked, this, [this]() { RunAdaptorModeCombinationTests(false); });
    connect(m_pAdaptorModeBatchBtn, &QPushButton::clicked, this, [this]() { RunAdaptorModeCombinationTests(true); });
    connect(m_pAdaptorModeStopBtn, &QPushButton::clicked, this, [this]()
    {
        if (!m_adaptorModeBatchRunning) { return; }
        auto* driver = GetFirstRobotDriverAdaptor();
        if (driver != nullptr) { RobotOperationLease::RequestCancellation(driver); }
    });
    connect(m_pAdaptorModeApplyBtn, &QPushButton::clicked, this, [this]()
    {
        if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning || RobotOperationLease::AnyActive()
            || m_adaptorAcceptanceMovedOut || m_adaptorJointMovedOut) { return; }
        auto* driver = GetFirstRobotDriverAdaptor();
        if (driver == nullptr) { return; }
        const std::string id = m_pAdaptorModeCombinationCombo->currentData().toString().toStdString();
        if (!driver->UseVerifiedModePreparation(id))
        { QMessageBox::warning(this, "选用模式组合", "模式组合未能固化。请选择已通过准备和恢复验证的组合，并查看以下原因。\n" + DecodeRobotMessageText(driver->GetLastRobotError())); return; }
        m_adaptorModeCombinationEvidence += QStringLiteral("\n已固化当前机器人数据流组合：%1，%2\n")
            .arg(QString::fromStdString(id), m_pAdaptorModeCombinationCombo->currentText());
        SaveAdaptorAcceptanceRun();
        RefreshAdaptorAcceptanceUi();
        QMessageBox::information(this, "选用模式组合", "已固化到当前机器人数据库，重启/重连自动恢复，运行前仍检查安全状态。\n后续直线、圆弧、关节、扫描和连续点动由品牌底层使用该数据流启动顺序；JOB仍走独立流程。机器人、端点或版本绑定不匹配时需重新测试选用。尚未执行位移。");
    });
    QFormLayout* interfaceForm = addStagePage(
        "仅使用现场明确预留、未被JOB/PLC使用的变量。逐个写入后暂停：在示教器核对窗口显示的实际变量名和测试值，"
        "点击“现场数值一致，回读校验”后才自动回读比较，再恢复并验证原值。取消/5分钟超时会尝试恢复；禁止关闭电源或强制退出。 ");
    m_pAdaptorAcceptanceIntIndexSpin = new QSpinBox();
    m_pAdaptorAcceptanceIntIndexSpin->setRange(0, 255);
    m_pAdaptorAcceptanceIntIndexSpin->setValue(254);
    m_pAdaptorAcceptanceRealIndexSpin = new QSpinBox();
    m_pAdaptorAcceptanceRealIndexSpin->setRange(0, 255);
    m_pAdaptorAcceptanceRealIndexSpin->setValue(254);
    interfaceForm->addRow("INT测试索引：", m_pAdaptorAcceptanceIntIndexSpin);
    interfaceForm->addRow("REAL测试索引：", m_pAdaptorAcceptanceRealIndexSpin);
    QFormLayout* assetForm = addStagePage(
        "选择本流程实际使用的工具号，只读检查运动学、限位、关节/直角闭环和控制器程序资产。手眼标定在下一阶段验证。 ");
    m_pAdaptorAcceptanceToolIndexSpin = new QSpinBox();
    m_pAdaptorAcceptanceToolIndexSpin->setRange(0, 15);
    m_pAdaptorAcceptanceToolIndexSpin->setValue(1);
    assetForm->addRow("检查工具号：", m_pAdaptorAcceptanceToolIndexSpin);
    QPushButton* calibrationAssetsButton = CreateTestButton("标定资产与模型优化");
    assetForm->addRow("品牌固定读取链路：", calibrationAssetsButton);
    connect(calibrationAssetsButton, &QPushButton::clicked, this, [this]()
        {
            OpenRobotCalibrationDialog(m_pContralUnit, m_unitIndex, this);
        });
    QFormLayout* transformForm = addStagePage(
        "转换后必须用已知标定块或TCP点实测误差。请把测量值和结论补充到下方证据框。 ");
    m_pAdaptorAcceptanceTwoToThreeToleranceSpin = new QDoubleSpinBox();
    m_pAdaptorAcceptanceTwoToThreeToleranceSpin->setRange(0.01, 100.0);
    m_pAdaptorAcceptanceTwoToThreeToleranceSpin->setDecimals(2);
    m_pAdaptorAcceptanceTwoToThreeToleranceSpin->setSuffix(" mm");
    m_pAdaptorAcceptanceTwoToThreeToleranceSpin->setValue(2.0);
    transformForm->addRow("允许实测误差：", m_pAdaptorAcceptanceTwoToThreeToleranceSpin);
    addStagePage("开始后打开现有先测后焊界面。选择仅扫描/空跑，完成后返回本页填写结果目录、点云和轨迹结论。 ");
    addStagePage("实际焊接是最后的真机阶段。先复核焊机、气体、送丝、工艺、起收弧和安全回撤，再打开现有流程。 ");
    addStagePage("点击右下角“导出当前测试报告”。即使尚未全部通过，也会输出失败、受限、跳过和未测试项。 ");
    detailLayout->addWidget(m_pAdaptorAcceptanceStageStack, 1);

    QGroupBox* evidenceGroup = new QGroupBox("当前阶段证据", detailPanel);
    QVBoxLayout* evidenceLayout = new QVBoxLayout(evidenceGroup);
    m_pAdaptorAcceptanceEvidence = new QPlainTextEdit(evidenceGroup);
    m_pAdaptorAcceptanceEvidence->setPlaceholderText(
        "自动检查会写入接口返回、程序身份和回读结果；人工检查请补充示教器加载、现场路径、实测误差、点云和焊缝观察结论。");
    m_pAdaptorAcceptanceEvidence->setMinimumHeight(115);
    evidenceLayout->addWidget(m_pAdaptorAcceptanceEvidence);
    detailLayout->addWidget(evidenceGroup);

    QHBoxLayout* buttons = new QHBoxLayout();
    m_pAdaptorAcceptanceExecuteBtn = CreateTestButton("执行/打开当前阶段");
    m_pAdaptorAcceptancePassBtn = CreateTestButton("人工确认通过");
    m_pAdaptorAcceptanceFailBtn = CreateTestButton("标记失败");
    m_pAdaptorAcceptanceSkipBtn = CreateTestButton("标记跳过");
    m_pAdaptorAcceptanceNewRunBtn = CreateTestButton("新建一轮验收");
    m_pAdaptorAcceptanceReportBtn = CreateTestButton("导出当前测试报告");
    buttons->addWidget(m_pAdaptorAcceptanceExecuteBtn);
    buttons->addWidget(m_pAdaptorAcceptancePassBtn);
    buttons->addWidget(m_pAdaptorAcceptanceFailBtn);
    buttons->addWidget(m_pAdaptorAcceptanceSkipBtn);
    detailLayout->addLayout(buttons);
    QHBoxLayout* reportButtons = new QHBoxLayout();
    reportButtons->addStretch(1);
    reportButtons->addWidget(m_pAdaptorAcceptanceNewRunBtn);
    reportButtons->addWidget(m_pAdaptorAcceptanceReportBtn);
    detailLayout->addLayout(reportButtons);

    auto* detailScroll = RobotAdaptorAcceptanceLayout::WrapDetailPanel(stageSplitter, detailPanel);
    stageSplitter->addWidget(navigationPanel);
    stageSplitter->addWidget(detailScroll);
    stageSplitter->setChildrenCollapsible(false);
    stageSplitter->setStretchFactor(0, 3);
    stageSplitter->setStretchFactor(1, 7);
    stageSplitter->setSizes({ 360, 900 });
    layout->addWidget(stageSplitter, 1);

    connect(m_pAdaptorAcceptanceRobotCombo,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this, &FunctionTestDialog::ChangeAdaptorAcceptanceRobot);
    m_pAdaptorAcceptanceSaveTimer = new QTimer(this);
    m_pAdaptorAcceptanceSaveTimer->setSingleShot(true);
    m_pAdaptorAcceptanceSaveTimer->setInterval(400);
    connect(m_pAdaptorAcceptanceSaveTimer, &QTimer::timeout,
        this, [this]() { SaveAdaptorAcceptanceRun(); });
    const auto scheduleAcceptanceSave = [this]()
        {
            if (!m_adaptorAcceptanceLoading && m_adaptorAcceptanceRecordLoaded)
            {
                if (m_adaptorAcceptanceSelectedStage >= 0
                    && m_adaptorAcceptanceSelectedStage < m_adaptorAcceptanceEvidence.size())
                {
                    m_adaptorAcceptanceEvidence[m_adaptorAcceptanceSelectedStage] =
                        m_pAdaptorAcceptanceEvidence->toPlainText();
                }
                m_pAdaptorAcceptanceSaveTimer->start();
                m_pAdaptorAcceptanceSaveStatus->setText("修改待保存…");
            }
        };
    connect(m_pAdaptorAcceptanceEvidence, &QPlainTextEdit::textChanged,
        this, scheduleAcceptanceSave);
    for (auto* spin : { m_pAdaptorAcceptanceProgramSpeedSpin,
        m_pAdaptorAcceptanceDistanceSpin, m_pAdaptorAcceptanceSpeedSpin,
        m_pAdaptorAcceptanceTwoToThreeToleranceSpin })
    {
        connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, scheduleAcceptanceSave);
    }
    for (auto* spin : { m_pAdaptorAcceptanceIntIndexSpin,
        m_pAdaptorAcceptanceRealIndexSpin, m_pAdaptorAcceptanceToolIndexSpin })
    {
        connect(spin, qOverload<int>(&QSpinBox::valueChanged),
            this, scheduleAcceptanceSave);
    }
    connect(qApp, &QCoreApplication::aboutToQuit,
        this, [this]() { SaveAdaptorAcceptanceRun(); });
    connect(m_pAdaptorAcceptanceRunCombo, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int index)
        {
            if (m_adaptorAcceptanceLoading || index < 0) { return; }
            const QString requested = m_pAdaptorAcceptanceRunCombo->itemData(index).toString();
            if (requested == m_adaptorAcceptanceRunId) { return; }
            if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning || RobotOperationLease::AnyActive()
                || (m_adaptorAcceptanceRecordLoaded && !SaveAdaptorAcceptanceRun()))
            {
                const QSignalBlocker blocker(m_pAdaptorAcceptanceRunCombo);
                m_pAdaptorAcceptanceRunCombo->setCurrentIndex(
                    m_pAdaptorAcceptanceRunCombo->findData(m_adaptorAcceptanceRunId));
                QMessageBox::warning(this, "切换验收记录", "测试运行中或当前记录未保存成功，不能切换验收轮次。");
                return;
            }
            LoadAdaptorAcceptanceRun(requested);
        });
    connect(m_pAdaptorAcceptanceStageList, &QListWidget::currentRowChanged,
        this, [this](int row)
        {
            if (m_adaptorAcceptanceLoading) { return; }
            if (m_adaptorAcceptanceSelectedStage >= 0
                && m_adaptorAcceptanceSelectedStage < m_adaptorAcceptanceEvidence.size()
                && m_pAdaptorAcceptanceEvidence != nullptr)
            {
                m_adaptorAcceptanceEvidence[m_adaptorAcceptanceSelectedStage] =
                    m_pAdaptorAcceptanceEvidence->toPlainText().trimmed();
                SaveAdaptorAcceptanceRun();
            }
            m_adaptorAcceptanceSelectedStage = row;
            if (m_pAdaptorAcceptanceStageStack != nullptr && row >= 0)
            {
                m_pAdaptorAcceptanceStageStack->setCurrentIndex(row);
            }
            if (row >= 0 && row < m_adaptorAcceptanceEvidence.size()
                && m_pAdaptorAcceptanceEvidence != nullptr)
            {
                m_pAdaptorAcceptanceEvidence->setPlainText(m_adaptorAcceptanceEvidence.at(row));
            }
            RefreshAdaptorAcceptanceUi();
            SaveAdaptorAcceptanceRun();
        });
    connect(m_pAdaptorAcceptanceExecuteBtn, &QPushButton::clicked,
        this, &FunctionTestDialog::ExecuteAdaptorAcceptanceStage);
    connect(m_pAdaptorAcceptancePassBtn, &QPushButton::clicked, this, [this]()
        { MarkAdaptorAcceptanceStage("pass", m_pAdaptorAcceptanceEvidence->toPlainText()); });
    connect(m_pAdaptorAcceptanceFailBtn, &QPushButton::clicked, this, [this]()
        { MarkAdaptorAcceptanceStage("fail", m_pAdaptorAcceptanceEvidence->toPlainText()); });
    connect(m_pAdaptorAcceptanceSkipBtn, &QPushButton::clicked, this, [this]()
        { MarkAdaptorAcceptanceStage("skipped", m_pAdaptorAcceptanceEvidence->toPlainText()); });
    connect(m_pAdaptorAcceptanceNewRunBtn, &QPushButton::clicked, this, [this]()
        {
            if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning
                || RobotOperationLease::AnyActive())
            {
                QMessageBox::warning(this, "新建适配验收",
                    "机器人硬件操作运行期间不能重置验收记录。");
                return;
            }
            if (QMessageBox::question(this, "新建适配验收",
                "将保留数据库中的上一轮记录并新建一轮未测试状态。是否继续？")
                == QMessageBox::Yes)
            {
                StartNewAdaptorAcceptanceRun();
            }
        });
    connect(m_pAdaptorAcceptanceReportBtn, &QPushButton::clicked,
        this, &FunctionTestDialog::FinalizeAdaptorAcceptance);

    LoadAdaptorAcceptanceRun();
    RefreshAdaptorAcceptanceUi();
    return page;
}

void FunctionTestDialog::ShowAdaptorAcceptancePage()
{
    if (m_pTestTabs == nullptr || m_pAdaptorAcceptancePage == nullptr) { return; }
    const int index = m_pTestTabs->indexOf(m_pAdaptorAcceptancePage);
    if (index >= 0)
    {
        m_pTestTabs->setCurrentIndex(index);
    }
    m_adaptorAcceptanceStandaloneMode = true;
    if (m_pTestTabs->tabBar() != nullptr)
    {
        m_pTestTabs->tabBar()->hide();
    }
    if (m_pPageTitleLabel != nullptr)
    {
        m_pPageTitleLabel->setText(QStringLiteral("机器人适配测试 — %1")
            .arg(AdaptorAcceptanceStorageRobotName()));
    }
    if (m_pPageHintLabel != nullptr)
    {
        m_pPageHintLabel->setText(
            "按统一阶段完成机器人控制连接、FTP、程序下发、状态读取、低速运动、接口、资产、二转三、扫描和实际焊接验收；"
            "任意阶段均可导出报告用于修改品牌底层。");
    }
    if (m_pAdaptorAcceptanceTitleLabel != nullptr)
    {
        m_pAdaptorAcceptanceTitleLabel->hide();
    }
    setWindowTitle(QStringLiteral("机器人适配测试"));
}

void FunctionTestDialog::ShowSingleTestPage()
{
    m_adaptorAcceptanceStandaloneMode = false;
    if (m_pTestTabs != nullptr)
    {
        m_pTestTabs->setCurrentIndex(0);
        if (m_pTestTabs->tabBar() != nullptr)
        {
            m_pTestTabs->tabBar()->show();
        }
    }
    if (m_pPageTitleLabel != nullptr)
    {
        m_pPageTitleLabel->setText(QStringLiteral("机器人功能测试区"));
    }
    if (m_pPageHintLabel != nullptr)
    {
        m_pPageHintLabel->setText(
            "这里集中放置设置速度、读取位置、原生程序、诊断、往返运动、零位运动等测试功能；"
            "入口按当前机器人底层声明的适配能力开放。");
    }
    if (m_pAdaptorAcceptanceTitleLabel != nullptr)
    {
        m_pAdaptorAcceptanceTitleLabel->show();
    }
    setWindowTitle(QStringLiteral("功能测试"));
}

void FunctionTestDialog::ChangeAdaptorAcceptanceRobot(int comboIndex)
{
    if (m_pAdaptorAcceptanceRobotCombo == nullptr || comboIndex < 0) { return; }
    const int nextUnitIndex = m_pAdaptorAcceptanceRobotCombo->itemData(comboIndex).toInt();
    if (nextUnitIndex == m_unitIndex)
    {
        RefreshAdaptorAcceptanceUi();
        return;
    }
    if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning || RobotOperationLease::AnyActive()
        || m_adaptorJointMovedOut)
    {
        const QSignalBlocker blocker(m_pAdaptorAcceptanceRobotCombo);
        const int previousIndex = m_pAdaptorAcceptanceRobotCombo->findData(m_unitIndex);
        if (previousIndex >= 0)
        {
            m_pAdaptorAcceptanceRobotCombo->setCurrentIndex(previousIndex);
        }
        QMessageBox::warning(this, "切换测试机器人",
            QStringLiteral("机器人硬件操作正在运行（%1），必须等待完成或安全停止后才能切换测试对象。")
                .arg(RobotOperationLease::ActiveSummary()));
        return;
    }
    if (m_pContralUnit == nullptr || nextUnitIndex < 0
        || nextUnitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, "切换测试机器人", "选择的机器人控制单元无效。");
        return;
    }
    if (m_adaptorAcceptanceSelectedStage >= 0
        && m_adaptorAcceptanceSelectedStage < m_adaptorAcceptanceEvidence.size()
        && m_pAdaptorAcceptanceEvidence != nullptr)
    {
        m_adaptorAcceptanceEvidence[m_adaptorAcceptanceSelectedStage] =
            m_pAdaptorAcceptanceEvidence->toPlainText().trimmed();
    }
    if (m_adaptorAcceptanceRecordLoaded && !SaveAdaptorAcceptanceRun())
    {
        const QSignalBlocker blocker(m_pAdaptorAcceptanceRobotCombo);
        m_pAdaptorAcceptanceRobotCombo->setCurrentIndex(
            m_pAdaptorAcceptanceRobotCombo->findData(m_unitIndex));
        QMessageBox::warning(this, "切换测试机器人", "当前验收记录未保存成功，不能切换机器人。");
        return;
    }
    m_unitIndex = nextUnitIndex;
    const int selectedUnitNo =
        m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].nUnitNo;
    m_pCameraCache = m_cameraCacheResolver
        ? m_cameraCacheResolver(selectedUnitNo) : nullptr;
    m_kinematicsSampleFilePath.clear();
    m_kinematicsSampleCount = 0;
    m_adaptorAcceptanceSelectedStage = -1;
    LoadAdaptorAcceptanceRun();
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
}

QString FunctionTestDialog::AdaptorAcceptanceStorageRobotName() const
{
    if (m_pContralUnit != nullptr && m_unitIndex >= 0
        && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        const auto* driver = static_cast<RobotDriverAdaptor*>(
            m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
        if (driver != nullptr && !driver->RobotName().empty())
        {
            return QString::fromLocal8Bit(driver->RobotName().c_str());
        }
        return QString::fromLocal8Bit(
            m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].sUnitName.c_str());
    }
    return QStringLiteral("UnknownRobot");
}

void FunctionTestDialog::StartNewAdaptorAcceptanceRun()
{
    if (m_adaptorJointMovedOut)
    { QMessageBox::warning(this, "关节验收", "请先完成关节返回，不能丢弃本次原点后新建验收。"); return; }
    if (!m_adaptorRegisterRecovery.isEmpty())
    { QMessageBox::warning(this, "寄存器恢复待确认", "请先执行第5项，核对未恢复变量的原值后再新建验收。"); return; }
    if (!m_adaptorAcceptanceRunId.isEmpty() && !SaveAdaptorAcceptanceRun()) { return; }
    m_adaptorAcceptanceLoading = true;
    m_adaptorAcceptanceRunId = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    m_adaptorEvidencePlanRevision = QString::fromUtf8(RobotAdaptorAcceptancePlan::Revision);
    m_adaptorAcceptanceStates.clear();
    m_adaptorAcceptanceEvidence.clear();
    m_adaptorModeCombinationEvidence.clear();
    m_adaptorJointMotionState = "pending";
    m_adaptorJointMotionEvidence.clear();
    m_adaptorJointOriginalPulseValid = false;
    m_adaptorJointMovedOut = false;
    for (int stage = 0; stage < kAdaptorAcceptanceStageCount; ++stage)
    {
        m_adaptorAcceptanceStates.push_back("pending");
        m_adaptorAcceptanceEvidence.push_back(QString());
    }
    m_adaptorAcceptanceOriginalPoseValid = false;
    m_adaptorAcceptanceMovedOut = false;
    m_adaptorAcceptanceMotionRoundTripCompleted = false;
    m_adaptorAcceptanceSelectedStage = 0;
    m_pAdaptorAcceptanceEvidence->clear();
    m_pAdaptorAcceptanceStageList->setCurrentRow(0);
    m_adaptorAcceptanceRecordLoaded = true;
    m_adaptorAcceptanceLoading = false;
    SaveAdaptorAcceptanceRun();
    RefreshAdaptorAcceptanceUi();
}

void FunctionTestDialog::LoadAdaptorAcceptanceRun(const QString& requestedRunId)
{
    if (m_adaptorJointMovedOut)
    { QMessageBox::warning(this, "关节验收", "请先完成关节返回，再切换验收记录。"); return; }
    m_adaptorAcceptanceLoading = true;
    m_adaptorAcceptanceRecordLoaded = false;
    m_adaptorRegisterRecovery.clear(); // Previous robot/run may have a different recovery record.
    if (m_pAdaptorAcceptanceSaveTimer != nullptr) { m_pAdaptorAcceptanceSaveTimer->stop(); }
    const QString robotName = AdaptorAcceptanceStorageRobotName();
    RobotAdaptorAcceptanceStore::History history;
    QString storageError;
    const auto loadFailed = [this](const QString& message)
    {
        m_adaptorAcceptanceRunId.clear();
        m_adaptorAcceptanceStates.clear();
        m_adaptorAcceptanceEvidence.clear();
        m_pAdaptorAcceptanceEvidence->clear();
        m_adaptorAcceptanceLoading = false;
        m_pAdaptorAcceptanceSaveStatus->setText("读取失败，未覆盖原记录：" + message);
        m_pAdaptorAcceptanceSaveStatus->setStyleSheet("color:#ff8e8e;");
        RefreshAdaptorAcceptanceUi();
    };
    if (!RobotAdaptorAcceptanceStore::ReadHistory(robotName, history, &storageError))
    {
        loadFailed(storageError);
        return;
    }
    const QString runId = requestedRunId.isEmpty()
        ? history.value("Latest").value("RunId") : requestedRunId;
    m_pAdaptorAcceptanceRunCombo->clear();
    const QStringList runIds = history.keys();
    for (auto it = runIds.crbegin(); it != runIds.crend(); ++it)
    {
        if (*it != "Latest") { m_pAdaptorAcceptanceRunCombo->addItem(*it, *it); }
    }
    if (history.isEmpty())
    {
        m_adaptorAcceptanceRunId.clear();
        m_adaptorAcceptanceLoading = false;
        StartNewAdaptorAcceptanceRun();
        return;
    }
    auto record = history.value(runId);
    if (runId.isEmpty() || !RobotAdaptorAcceptanceStore::NormalizeRecord(robotName, record, &storageError))
    {
        loadFailed(storageError.isEmpty() ? QStringLiteral("缺少有效的验收轮次索引。") : storageError);
        return;
    }
    auto* currentDriver = GetFirstRobotDriverAdaptor();
    if (currentDriver != nullptr && record.value("DriverType")
        != QString::fromStdString(currentDriver->DriverDescriptor().typeName))
    {
        loadFailed("记录所属机器人品牌与当前配置不一致；请新建验收轮次，不能沿用其他品牌的通过记录。");
        return;
    }
    m_adaptorAcceptanceRunId = runId;
    m_adaptorEvidencePlanRevision = record.value("EvidencePlanRevision", record.value("PlanRevision", "legacy-unrecorded"));
    m_adaptorRegisterRecovery = record.value("RegisterRecovery");
    m_adaptorModeCombinationEvidence = record.value("ModeCombinationEvidence");
    m_adaptorJointMotionState = record.value("JointMotionState", "pending");
    m_adaptorJointMotionEvidence = record.value("JointMotionEvidence");
    if (m_adaptorJointMotionState == "running" || m_adaptorJointMotionState == "awaiting_return")
    {
        m_adaptorJointMotionState = "fail";
        m_adaptorJointMotionEvidence += "\n上次关节往返未完成；原点不跨会话恢复，请现场确认后重新测试。";
    }
    m_adaptorJointOriginalPulseValid = false;
    m_adaptorJointMovedOut = false;
    m_pAdaptorAcceptanceRunCombo->setCurrentIndex(m_pAdaptorAcceptanceRunCombo->findData(runId));
    m_adaptorAcceptanceStates.clear();
    m_adaptorAcceptanceEvidence.clear();
    for (int stage = 0; stage < kAdaptorAcceptanceStageCount; ++stage)
    {
        const QString state = record.value(QStringLiteral("Stage%1State").arg(stage));
        const QString evidence = record.value(QStringLiteral("Stage%1Evidence").arg(stage));
        m_adaptorAcceptanceStates.push_back(state.isEmpty()
            ? QStringLiteral("pending") : state);
        m_adaptorAcceptanceEvidence.push_back(evidence);
    }
    const auto loadDouble = [&record](const QString& key, QDoubleSpinBox* spin)
    { bool ok = false; const double value = record.value(key).toDouble(&ok); if (ok) { spin->setValue(value); } };
    const auto loadInt = [&record](const QString& key, QSpinBox* spin)
    { bool ok = false; const int value = record.value(key).toInt(&ok); if (ok) { spin->setValue(value); } };
    loadDouble("ProgramSpeedMmPerMin", m_pAdaptorAcceptanceProgramSpeedSpin);
    loadDouble("LinearDistanceMm", m_pAdaptorAcceptanceDistanceSpin);
    loadDouble("LinearSpeedMmPerMin", m_pAdaptorAcceptanceSpeedSpin);
    loadDouble("TwoToThreeToleranceMm", m_pAdaptorAcceptanceTwoToThreeToleranceSpin);
    loadInt("IntegerRegisterIndex", m_pAdaptorAcceptanceIntIndexSpin);
    loadInt("RealRegisterIndex", m_pAdaptorAcceptanceRealIndexSpin);
    loadInt("ToolIndex", m_pAdaptorAcceptanceToolIndexSpin);
    // 真机位姿不能跨进程或跨页面恢复为可执行断点；重新进入后必须重新捕获。
    m_adaptorAcceptanceOriginalPoseValid = false;
    m_adaptorAcceptanceMovedOut = false;
    m_adaptorAcceptanceMotionRoundTripCompleted = false;
    m_adaptorAcceptanceSelectedStage = std::clamp(record.value("SelectedStage").toInt(), 0, kAdaptorAcceptanceStageCount - 1);
    m_pAdaptorAcceptanceStageList->setCurrentRow(m_adaptorAcceptanceSelectedStage);
    m_pAdaptorAcceptanceEvidence->setPlainText(m_adaptorAcceptanceEvidence.value(m_adaptorAcceptanceSelectedStage));
    m_adaptorAcceptanceRecordLoaded = true;
    m_adaptorAcceptanceLoading = false;
    SaveAdaptorAcceptanceRun();
    RefreshAdaptorAcceptanceUi();
}

bool FunctionTestDialog::SaveAdaptorAcceptanceRun()
{
    if (m_adaptorAcceptanceLoading || !m_adaptorAcceptanceRecordLoaded || m_adaptorAcceptanceRunId.isEmpty()) { return false; }
    if (m_pAdaptorAcceptanceSaveTimer != nullptr) { m_pAdaptorAcceptanceSaveTimer->stop(); }
    const QString robotName = AdaptorAcceptanceStorageRobotName();
    RobotAdaptorAcceptanceStore::Record record;
    record["UpdatedAt"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    record["EvidencePlanRevision"] = m_adaptorEvidencePlanRevision;
    record["RegisterRecovery"] = m_adaptorRegisterRecovery;
    record["ModeCombinationEvidence"] = m_adaptorModeCombinationEvidence;
    record["JointMotionState"] = m_adaptorJointMotionState;
    record["JointMotionEvidence"] = m_adaptorJointMotionEvidence;
    record["SelectedStage"] = QString::number(m_adaptorAcceptanceSelectedStage);
    RobotDriverAdaptor* driver = nullptr;
    if (m_pContralUnit != nullptr && m_unitIndex >= 0
        && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        driver = static_cast<RobotDriverAdaptor*>(
            m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    }
    if (driver != nullptr)
    {
        record["DriverType"] = QString::fromStdString(driver->DriverDescriptor().typeName);
        record["CapabilityMask"] = QString::number(driver->DriverCapabilities());
    }
    else { record["DriverType"] = "Unknown"; }
    if (m_pAdaptorAcceptanceProgramSpeedSpin != nullptr)
    {
        record["ProgramSpeedMmPerMin"] = QString::number(m_pAdaptorAcceptanceProgramSpeedSpin->value(), 'f', 3);
    }
    if (m_pAdaptorAcceptanceDistanceSpin != nullptr)
    {
        record["LinearDistanceMm"] = QString::number(m_pAdaptorAcceptanceDistanceSpin->value(), 'f', 3);
    }
    if (m_pAdaptorAcceptanceSpeedSpin != nullptr)
    {
        record["LinearSpeedMmPerMin"] = QString::number(m_pAdaptorAcceptanceSpeedSpin->value(), 'f', 3);
    }
    if (m_pAdaptorAcceptanceIntIndexSpin != nullptr)
    {
        record["IntegerRegisterIndex"] = QString::number(m_pAdaptorAcceptanceIntIndexSpin->value());
    }
    if (m_pAdaptorAcceptanceRealIndexSpin != nullptr)
    {
        record["RealRegisterIndex"] = QString::number(m_pAdaptorAcceptanceRealIndexSpin->value());
    }
    if (m_pAdaptorAcceptanceToolIndexSpin != nullptr)
    {
        record["ToolIndex"] = QString::number(m_pAdaptorAcceptanceToolIndexSpin->value());
    }
    if (m_pAdaptorAcceptanceTwoToThreeToleranceSpin != nullptr)
    {
        record["TwoToThreeToleranceMm"] = QString::number(m_pAdaptorAcceptanceTwoToThreeToleranceSpin->value(), 'f', 3);
    }
    for (int stage = 0; stage < kAdaptorAcceptanceStageCount
        && stage < m_adaptorAcceptanceStates.size(); ++stage)
    {
        record[QStringLiteral("Stage%1State").arg(stage)] = m_adaptorAcceptanceStates.at(stage);
        record[QStringLiteral("Stage%1Evidence").arg(stage)] = m_adaptorAcceptanceEvidence.value(stage);
    }
    QString error;
    const bool saved = RobotAdaptorAcceptanceStore::Save(robotName, m_adaptorAcceptanceRunId, record, &error);
    m_pAdaptorAcceptanceSaveStatus->setStyleSheet(saved ? "color:#78d6a6;" : "color:#ff8e8e;");
    m_pAdaptorAcceptanceSaveStatus->setText(saved
        ? QStringLiteral("已保存 %1｜数据库：%2").arg(record.value("UpdatedAt"), ConfigDatabase::DatabasePath())
        : QStringLiteral("保存失败，记录尚未落盘：%1").arg(error));
    if (saved)
    {
        const QSignalBlocker blocker(m_pAdaptorAcceptanceRunCombo);
        int index = m_pAdaptorAcceptanceRunCombo->findData(m_adaptorAcceptanceRunId);
        if (index < 0)
        {
            m_pAdaptorAcceptanceRunCombo->insertItem(0, m_adaptorAcceptanceRunId, m_adaptorAcceptanceRunId);
            index = 0;
        }
        m_pAdaptorAcceptanceRunCombo->setCurrentIndex(index);
    }
    return saved;
}

std::uint64_t FunctionTestDialog::AdaptorAcceptanceRequiredMask(int stage) const
{
    const auto bit = [](RobotDriverCapability capability)
        { return RobotDriverCapabilityBit(capability); };
    switch (stage)
    {
    case 0:
        return bit(RobotDriverCapability::ConnectionControl);
    case 1:
        return bit(RobotDriverCapability::FtpFileTransfer);
    case 2:
        return bit(RobotDriverCapability::ContinuousTrajectory)
            | bit(RobotDriverCapability::OfflineTrajectoryExport)
            | bit(RobotDriverCapability::NativeProgramUpload)
            | bit(RobotDriverCapability::FtpFileTransfer);
    case 3:
        return bit(RobotDriverCapability::PassiveState);
    case 4:
        return bit(RobotDriverCapability::LinearMotion)
            | bit(RobotDriverCapability::PassiveState)
            | bit(RobotDriverCapability::OperationModeControl)
            | bit(RobotDriverCapability::ServoPowerControl)
            | bit(RobotDriverCapability::VerifiedProgramCompletion)
            | bit(RobotDriverCapability::VerifiedSafeAbort);
    case 5:
    case 6:
    case 7:
        return bit(RobotDriverCapability::PassiveState);
    case 8:
        return MeasureThenWeldCapabilityPolicy::EntryMask<RobotDriverCapability>();
    case 9:
        return MeasureThenWeldCapabilityPolicy::WeldMask<RobotDriverCapability>(true);
    default:
        return 0;
    }
}

bool FunctionTestDialog::AdaptorAcceptancePrerequisitesReady(int stage, QString* reason) const
{
    QVector<int> prerequisites;
    switch (stage)
    {
    case 1: prerequisites = { 0 }; break;
    case 2: prerequisites = { 0, 1 }; break;
    case 3: prerequisites = { 0 }; break;
    case 4: prerequisites = { 0, 3 }; break;
    case 5: prerequisites = { 0, 3 }; break;
    case 6: prerequisites = { 0, 1, 3 }; break;
    case 7: prerequisites = { 3, 6 }; break;
    case 8: prerequisites = { 0, 2, 3, 4, 6, 7 }; break;
    case 9: prerequisites = { 0, 1, 2, 3, 4, 6, 7, 8 }; break;
    case 10:
        for (int index = 0; index < 10; ++index) { prerequisites.push_back(index); }
        break;
    default: break;
    }
    for (const int prerequisite : prerequisites)
    {
        if (m_adaptorAcceptanceStates.value(prerequisite) != "pass")
        {
            if (reason != nullptr)
            {
                *reason = QStringLiteral("必须先通过阶段%1：%2")
                    .arg(prerequisite)
                    .arg(QString::fromUtf8(kAdaptorAcceptanceStageNames[prerequisite]));
            }
            return false;
        }
    }
    if (reason != nullptr) { reason->clear(); }
    return true;
}

void FunctionTestDialog::RefreshAdaptorAcceptanceUi()
{
    if (m_pAdaptorAcceptanceStageList == nullptr) { return; }
    RobotDriverAdaptor* driver = nullptr;
    if (m_pContralUnit != nullptr && m_unitIndex >= 0
        && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        driver = static_cast<RobotDriverAdaptor*>(
            m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    }
    if (m_pAdaptorAcceptanceRobotSummary != nullptr)
    {
        if (driver == nullptr)
        {
            m_pAdaptorAcceptanceRobotSummary->setText("当前控制单元没有可用的机器人驱动。");
        }
        else
        {
            const RobotDriverDescriptor descriptor = driver->DriverDescriptor();
            const RobotConnectionEndpoint endpoint = driver->ControlEndpoint();
            m_pAdaptorAcceptanceRobotSummary->setText(QStringLiteral(
                "机器人=%1｜驱动=%2｜控制端点=%3:%4｜连接=%5｜能力掩码=%6")
                .arg(AdaptorAcceptanceStorageRobotName())
                .arg(QString::fromStdString(descriptor.displayName))
                .arg(QString::fromStdString(endpoint.host))
                .arg(endpoint.port)
                .arg(driver->IsConnected() ? "已连接" : "未连接")
                .arg(QString::number(driver->DriverCapabilities())));
        }
    }
    if (m_pPageTitleLabel != nullptr && m_adaptorAcceptanceStandaloneMode)
    {
        m_pPageTitleLabel->setText(QStringLiteral("机器人适配测试 — %1")
            .arg(AdaptorAcceptanceStorageRobotName()));
    }
    if (m_pAdaptorAcceptanceTitleLabel != nullptr)
    {
        m_pAdaptorAcceptanceTitleLabel->setText(QStringLiteral("机器人适配测试 — %1")
            .arg(AdaptorAcceptanceStorageRobotName()));
    }
    for (int stage = 0; stage < kAdaptorAcceptanceStageCount; ++stage)
    {
        const std::uint64_t required = AdaptorAcceptanceRequiredMask(stage);
        QStringList requiredNames;
        for (unsigned int bitIndex = 0; bitIndex <= RobotDriverCapabilityMaxBitIndex; ++bitIndex)
        {
            const auto capability = static_cast<RobotDriverCapability>(1ULL << bitIndex);
            if ((required & RobotDriverCapabilityBit(capability)) != 0)
            {
                requiredNames.push_back(QString::fromUtf8(
                    RobotDriverAdaptor::CapabilityDisplayName(capability)));
            }
        }
        QString state = m_adaptorAcceptanceStates.value(stage, "pending");
        if (state == "pending" && required != 0
            && (driver == nullptr || !driver->SupportsMask(required)))
        {
            state = "restricted";
        }
        QListWidgetItem* item = m_pAdaptorAcceptanceStageList->item(stage);
        if (item != nullptr)
        {
            item->setText(QStringLiteral("%1\n    %2")
                .arg(QString::fromUtf8(kAdaptorAcceptanceStageNames[stage]),
                    AdaptorAcceptanceStateText(state)));
            item->setToolTip(QStringLiteral("能力门禁：%1\n最近证据：%2")
                .arg(requiredNames.isEmpty() ? QStringLiteral("人工/本地数据门禁")
                        : requiredNames.join("、"),
                    m_adaptorAcceptanceEvidence.value(stage).simplified()));
            item->setForeground(state == "pass" ? QColor("#78d6a6")
                : (state == "fail" ? QColor("#ff8e8e")
                    : (state == "restricted" ? QColor("#ffd37a") : QColor("#B8C7CC"))));
        }
    }

    const int stage = m_pAdaptorAcceptanceStageList->currentRow();
    const std::uint64_t required = stage >= 0 ? AdaptorAcceptanceRequiredMask(stage) : 0;
    const bool capabilityReady = driver != nullptr
        && (required == 0 || driver->SupportsMask(required));
    QString prerequisiteError;
    const bool prerequisitesReady = stage == 10 || (stage >= 0
        && AdaptorAcceptancePrerequisitesReady(stage, &prerequisiteError));
    if (m_pAdaptorAcceptanceStageStack != nullptr && stage >= 0)
    {
        m_pAdaptorAcceptanceStageStack->setCurrentIndex(stage);
    }
    if (m_pAdaptorAcceptanceStageTitle != nullptr)
    {
        m_pAdaptorAcceptanceStageTitle->setText(stage >= 0
            ? QString::fromUtf8(kAdaptorAcceptanceStageNames[stage]) : QString());
    }
    if (m_pAdaptorAcceptanceStageDescription != nullptr)
    {
        m_pAdaptorAcceptanceStageDescription->setText(
            AdaptorAcceptanceStageDescription(stage));
    }
    if (m_pAdaptorAcceptanceStageStatus != nullptr)
    {
        QString state = m_adaptorAcceptanceStates.value(stage, "pending");
        if (stage >= 0 && required != 0 && !capabilityReady)
        {
            state = "restricted";
        }
        m_pAdaptorAcceptanceStageStatus->setText(QStringLiteral("当前状态：%1")
            .arg(AdaptorAcceptanceStateText(state)));
    }
    if (m_pAdaptorAcceptanceStageGate != nullptr)
    {
        QStringList requiredNames;
        for (unsigned int bitIndex = 0; bitIndex <= RobotDriverCapabilityMaxBitIndex; ++bitIndex)
        {
            const auto capability = static_cast<RobotDriverCapability>(1ULL << bitIndex);
            if ((required & RobotDriverCapabilityBit(capability)) != 0)
            {
                requiredNames.push_back(QString::fromUtf8(
                    RobotDriverAdaptor::CapabilityDisplayName(capability)));
            }
        }
        QString gateText = QStringLiteral("适配能力门禁：%1")
            .arg(requiredNames.isEmpty() ? QStringLiteral("人工/本地数据门禁")
                : requiredNames.join("、"));
        if (!capabilityReady && driver != nullptr && required != 0)
        {
            gateText += QStringLiteral("\n当前品牌缺少：%1")
                .arg(QString::fromUtf8(driver->MissingCapabilitiesText(required).c_str()));
        }
        if (!prerequisitesReady)
        {
            gateText += "\n前置条件：" + prerequisiteError;
        }
        m_pAdaptorAcceptanceStageGate->setText(gateText);
    }
    if (m_pAdaptorAcceptanceExecuteBtn != nullptr)
    {
        m_pAdaptorAcceptanceExecuteBtn->setEnabled(
            m_adaptorAcceptanceRecordLoaded && !m_adaptorAcceptanceBusy && (stage == 10 || capabilityReady)
            && prerequisitesReady && !m_adaptorJointMovedOut);
        m_pAdaptorAcceptanceExecuteBtn->setToolTip(!capabilityReady && driver != nullptr
            ? QStringLiteral("缺少适配能力：%1")
                .arg(QString::fromUtf8(driver->MissingCapabilitiesText(required).c_str()))
            : (!prerequisitesReady ? prerequisiteError : QString()));
        static const char* const actionTexts[kAdaptorAcceptanceStageCount] = {
            "开始连接测试", "开始FTP闭环测试", "生成并下发测试程序", "开始位置/状态读取",
            "开始低速外移", "开始接口写读恢复", "开始资产检查", "开始二转三验证",
            "打开扫描流程", "打开实际焊接流程", "导出当前测试报告"
        };
        if (stage == 4 && m_adaptorAcceptanceMovedOut)
        {
            m_pAdaptorAcceptanceExecuteBtn->setText("返回记录的原始位姿");
        }
        else if (stage >= 0 && stage < kAdaptorAcceptanceStageCount)
        {
            m_pAdaptorAcceptanceExecuteBtn->setText(
                QString::fromUtf8(actionTexts[stage]));
        }
    }
    const bool manualPassAllowed = m_adaptorAcceptanceRecordLoaded && !m_adaptorAcceptanceBusy
        && capabilityReady && prerequisitesReady && stage >= 0 && stage < 10 && stage != 5;
    if (m_pAdaptorAcceptancePassBtn != nullptr)
    {
        m_pAdaptorAcceptancePassBtn->setEnabled(manualPassAllowed);
        m_pAdaptorAcceptancePassBtn->setText(stage == 5 ? "回读后自动判定" : "人工确认通过");
        m_pAdaptorAcceptancePassBtn->setToolTip(stage == 5
            ? "逐个现场确认测试值后自动回读、比较和恢复；不能人工覆盖失败结果。" : "");
    }
    if (m_pAdaptorAcceptanceFailBtn != nullptr)
    {
        m_pAdaptorAcceptanceFailBtn->setEnabled(m_adaptorAcceptanceRecordLoaded && !m_adaptorAcceptanceBusy && stage >= 0);
    }
    if (m_pAdaptorAcceptanceSkipBtn != nullptr)
    {
        m_pAdaptorAcceptanceSkipBtn->setEnabled(m_adaptorAcceptanceRecordLoaded && !m_adaptorAcceptanceBusy && stage >= 0 && stage < 9);
    }
    if (m_pAdaptorAcceptanceRobotCombo != nullptr)
    {
        m_pAdaptorAcceptanceRobotCombo->setEnabled(
            !m_adaptorAcceptanceBusy && !m_bRobotCommandRunning && !m_adaptorJointMovedOut);
    }
    if (m_pAdaptorAcceptanceNewRunBtn != nullptr)
    {
        m_pAdaptorAcceptanceNewRunBtn->setEnabled(
            !m_adaptorAcceptanceBusy && !m_bRobotCommandRunning && !m_adaptorJointMovedOut);
    }
    if (m_pAdaptorAcceptanceRunCombo != nullptr)
    {
        m_pAdaptorAcceptanceRunCombo->setEnabled(!m_adaptorAcceptanceBusy && !m_bRobotCommandRunning && !m_adaptorJointMovedOut);
    }
    if (m_pAdaptorAcceptanceReportBtn != nullptr)
    {
        m_pAdaptorAcceptanceReportBtn->setEnabled(m_adaptorAcceptanceRecordLoaded && !m_adaptorAcceptanceBusy);
    }
    if (m_pAdaptorAcceptanceIntIndexSpin != nullptr)
    {
        m_pAdaptorAcceptanceIntIndexSpin->setEnabled(!m_adaptorAcceptanceBusy);
        m_pAdaptorAcceptanceRealIndexSpin->setEnabled(!m_adaptorAcceptanceBusy);
    }
    if (m_pAdaptorModeCombinationCombo != nullptr)
    {
        const auto cases = driver != nullptr ? driver->ModePreparationTestCases()
            : std::vector<RobotModePreparationTestCase>{};
        const QString robot = AdaptorAcceptanceStorageRobotName();
        RobotAdaptorAcceptanceLayout::PopulateModeOptions(m_pAdaptorModeCombinationCombo, robot, cases);
        const bool enabled = !cases.empty() && m_adaptorAcceptanceRecordLoaded
            && !m_adaptorAcceptanceBusy && !m_bRobotCommandRunning && !m_adaptorAcceptanceMovedOut && !m_adaptorJointMovedOut;
        m_pAdaptorModeCombinationCombo->setEnabled(enabled);
        m_pAdaptorModeSingleBtn->setEnabled(enabled);
        m_pAdaptorModeBatchBtn->setEnabled(enabled);
        m_pAdaptorModeApplyBtn->setEnabled(enabled);
        m_pAdaptorModeStopBtn->setEnabled(m_adaptorModeBatchRunning);
        const QString active = driver != nullptr ? QString::fromStdString(driver->ActiveModePreparationId()) : QString();
        m_pAdaptorModeCombinationCombo->setToolTip(cases.empty()
            ? QStringLiteral("当前品牌没有提供模式组合测试，不发送原始协议。")
            : QStringLiteral("当前已固化数据流组合：%1。组合测试不发位移，重启/重连按机器人、端点和版本绑定恢复，运行前仍检查安全状态。").arg(active.isEmpty() ? "尚未固化或绑定未通过" : active));
    }
    if (m_pAdaptorJointMotionBtn != nullptr)
    {
        const bool jointReady = driver != nullptr && driver->ExternalAxleType() == 0 && driver->SupportsAll({
            RobotDriverCapability::JointMotion, RobotDriverCapability::PassiveState,
            RobotDriverCapability::OperationModeControl, RobotDriverCapability::ServoPowerControl,
            RobotDriverCapability::VerifiedProgramCompletion, RobotDriverCapability::VerifiedSafeAbort });
        m_pAdaptorJointMotionBtn->setEnabled(m_adaptorAcceptanceRecordLoaded && !m_adaptorAcceptanceBusy
            && !m_bRobotCommandRunning && !m_adaptorAcceptanceMovedOut
            && jointReady && AdaptorAcceptancePrerequisitesReady(4));
        m_pAdaptorJointMotionBtn->setText(m_adaptorJointMovedOut
            ? "关节专项：返回原始关节（1%）" : "关节专项：J1 +0.5°（1%）");
        m_pAdaptorJointMotionStatus->setText(QString("%1。独立于直线验收；两段单独确认，回读及恢复成功后自动判定。%2")
            .arg(m_adaptorJointMotionState == "awaiting_return" ? QStringLiteral("待返回")
                : AdaptorAcceptanceStateText(m_adaptorJointMotionState))
            .arg(jointReady ? "" : " 当前品牌关节运动或安全能力未就绪。"));
        m_pAdaptorJointMotionStatus->setToolTip(m_adaptorJointMotionEvidence);
        m_pAdaptorJointCancelBtn->setEnabled(!m_adaptorAcceptanceBusy && !m_bRobotCommandRunning && m_adaptorJointMovedOut);
    }
}

void FunctionTestDialog::MarkAdaptorAcceptanceStage(
    const QString& state, const QString& evidence)
{
    if (m_pAdaptorAcceptanceStageList == nullptr || !m_adaptorAcceptanceRecordLoaded) { return; }
    const int stage = m_pAdaptorAcceptanceStageList->currentRow();
    if (stage < 0 || stage >= kAdaptorAcceptanceStageCount) { return; }
    if (m_adaptorAcceptanceBusy) { return; }
    if (state == "pass" && stage == 5)
    { QMessageBox::warning(this, "适配验收", "第5项必须逐个现场确认、回读校验并恢复成功，不能人工覆盖测试结果。"); return; }
    const std::uint64_t required = AdaptorAcceptanceRequiredMask(stage);
    RobotDriverAdaptor* driver = GetFirstRobotDriverAdaptor();
    if (driver == nullptr) { return; }
    if (state == "pass" && required != 0 && !driver->SupportsMask(required))
    {
        QMessageBox::warning(this, "适配验收",
            QStringLiteral("当前品牌缺少能力：%1。该阶段只能保持能力受限，不能人工标记通过。")
                .arg(QString::fromUtf8(driver->MissingCapabilitiesText(required).c_str())));
        return;
    }
    QString prerequisiteError;
    if (state == "pass" && !AdaptorAcceptancePrerequisitesReady(stage, &prerequisiteError))
    {
        QMessageBox::warning(this, "适配验收", prerequisiteError);
        return;
    }
    if (state == "pass" && evidence.trimmed().isEmpty())
    {
        QMessageBox::warning(this, "适配验收", "标记通过前必须填写或生成本阶段证据。");
        return;
    }
    if (state == "pass" && stage == 4
        && !m_adaptorAcceptanceMotionRoundTripCompleted)
    {
        QMessageBox::warning(this, "适配验收",
            "必须在本次界面会话中完成低速外移和返回原始位姿，才能把真机运动阶段标记为通过。");
        return;
    }
    m_adaptorAcceptanceStates[stage] = state;
    m_adaptorAcceptanceEvidence[stage] = evidence.trimmed();
    SaveAdaptorAcceptanceRun();
    RefreshAdaptorAcceptanceUi();
}

void FunctionTestDialog::FinishAdaptorAcceptanceStage(
    int stage, bool success, const QString& evidence, bool manualConfirmation)
{
    m_adaptorAcceptanceBusy = false;
    m_bRobotCommandRunning = false;
    if (stage >= 0 && stage < kAdaptorAcceptanceStageCount)
    {
        m_adaptorAcceptanceStates[stage] = success && !manualConfirmation
            ? QStringLiteral("pass")
            : (success ? QStringLiteral("pending") : QStringLiteral("fail"));
        m_adaptorAcceptanceEvidence[stage] = evidence;
        if (m_pAdaptorAcceptanceStageList != nullptr
            && m_pAdaptorAcceptanceStageList->currentRow() == stage
            && m_pAdaptorAcceptanceEvidence != nullptr)
        {
            m_pAdaptorAcceptanceEvidence->setPlainText(evidence);
        }
    }
    if (!SaveAdaptorAcceptanceRun() && stage == 5)
    {
        success = false;
        m_adaptorAcceptanceStates[5] = "fail";
        m_adaptorAcceptanceEvidence[5] += "\n最终结果保存失败，不能判定验收通过。";
        if (m_adaptorAcceptanceSelectedStage == 5)
        { m_pAdaptorAcceptanceEvidence->setPlainText(m_adaptorAcceptanceEvidence[5]); }
    }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    AppendLog(QStringLiteral("适配验收阶段%1：%2").arg(stage).arg(evidence));
    if (!success)
    {
        QMessageBox::warning(this, "适配验收", m_adaptorAcceptanceEvidence.value(stage, evidence));
    }
    else if (manualConfirmation)
    {
        QMessageBox::information(this, "适配验收",
            evidence + "\n\n请完成现场/示教器检查，在证据框补充结论后点击“人工确认通过”。");
    }
}

void FunctionTestDialog::ExecuteAdaptorAcceptanceStage()
{
    if (m_adaptorAcceptanceBusy || m_pAdaptorAcceptanceStageList == nullptr || m_adaptorJointMovedOut) { return; }
    if (!SaveAdaptorAcceptanceRun())
    {
        QMessageBox::warning(this, "适配验收", "验收记录未能保存到数据库，本次测试未启动。请检查记录区的错误提示。");
        return;
    }
    const int stage = m_pAdaptorAcceptanceStageList->currentRow();
    QString prerequisiteError;
    if (stage != 10 && !AdaptorAcceptancePrerequisitesReady(stage, &prerequisiteError))
    {
        QMessageBox::warning(this, "适配验收", prerequisiteError);
        return;
    }
    RobotDriverAdaptor* driver = GetFirstRobotDriverAdaptor();
    if (driver == nullptr) { return; }
    const std::uint64_t required = AdaptorAcceptanceRequiredMask(stage);
    if (required != 0 && !driver->SupportsMask(required))
    {
        const QString evidence = QStringLiteral("能力受限：%1")
            .arg(QString::fromUtf8(driver->MissingCapabilitiesText(required).c_str()));
        m_adaptorAcceptanceStates[stage] = "restricted";
        m_adaptorAcceptanceEvidence[stage] = evidence;
        m_pAdaptorAcceptanceEvidence->setPlainText(evidence);
        SaveAdaptorAcceptanceRun();
        RefreshAdaptorAcceptanceUi();
        QMessageBox::warning(this, "适配验收", evidence);
        return;
    }
    switch (stage)
    {
    case 0: RunAdaptorConnectionTest(); break;
    case 1: RunAdaptorFtpRoundTrip(); break;
    case 2: RunAdaptorProgramDownlink(); break;
    case 3: RunAdaptorPositionStatusCheck(); break;
    case 4: RunAdaptorSafeLinearMotion(); break;
    case 5: RunAdaptorInterfaceMatrix(); break;
    case 6: RunAdaptorAssetAudit(); break;
    case 7: RunAdaptorTwoToThreeCheck(); break;
    case 8: OpenAdaptorWorkflow("measureThenWeldScan"); break;
    case 9: OpenAdaptorWorkflow("measureThenWeldActual"); break;
    case 10: FinalizeAdaptorAcceptance(); break;
    default: break;
    }
}

bool FunctionTestDialog::BeginAdaptorAcceptanceStage(int stage)
{
    if (!m_adaptorAcceptanceRecordLoaded || stage < 0 || stage >= m_adaptorAcceptanceStates.size()) { return false; }
    const QString previous = m_adaptorAcceptanceStates.at(stage);
    m_adaptorAcceptanceStates[stage] = "running";
    if (!SaveAdaptorAcceptanceRun())
    {
        m_adaptorAcceptanceStates[stage] = previous;
        QMessageBox::warning(this, "适配验收", "执行前记录保存失败，未发送本阶段测试命令。请检查数据库错误提示。");
        return false;
    }
    m_adaptorAcceptanceBusy = true;
    m_bRobotCommandRunning = true;
    return true;
}

void FunctionTestDialog::RunAdaptorConnectionTest()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::ConnectionControl, QStringLiteral("机器人控制连接测试"));
    if (driver == nullptr) { return; }
    if (QMessageBox::question(this, "机器人控制连接测试",
        "请核对当前机器人名称、控制IP/端口和现场实体一致。\n"
        "请确认作业区域安全、实体急停可用。本步骤将建立或复用控制连接，"
        "并通过品牌适配层执行连接后初始化：自动清除可复位报警、切换自动模式、伺服上电等前置工作。\n"
        "不会启动运动，也不会绕过实体急停或控制权条件；初始化失败将记录为本阶段失败。是否继续？",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
        != QMessageBox::Yes)
    {
        return;
    }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("机器人适配验收连接测试"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "机器人控制连接测试", leaseError);
        return;
    }
    if (!BeginAdaptorAcceptanceStage(0)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, driver, lease]()
        {
            QStringList evidence;
            const RobotDriverDescriptor descriptor = driver->DriverDescriptor();
            const RobotConnectionEndpoint endpoint = driver->ControlEndpoint();
            evidence << QStringLiteral("驱动=%1，类型码=%2，机器人=%3，控制端点=%4:%5，能力掩码=%6")
                .arg(QString::fromStdString(descriptor.displayName))
                .arg(descriptor.typeCode)
                .arg(QString::fromLocal8Bit(driver->RobotName().c_str()))
                .arg(QString::fromStdString(endpoint.host))
                .arg(endpoint.port)
                .arg(QString::number(driver->DriverCapabilities()));
            const bool endpointOk = endpoint.IsValid();
            const bool reused = driver->IsConnected();
            const bool connectCommandOk = endpointOk && (reused || driver->Connect());
            const bool connectedReadback = connectCommandOk && driver->IsConnected();
            std::string initializationSummary;
            const bool initializationAttempted = endpointOk && connectCommandOk && connectedReadback;
            const bool initializationOk = initializationAttempted
                && driver->InitializeAfterConnect(&initializationSummary);
            const bool initializedConnectionReadback = connectedReadback && driver->IsConnected();
            const bool ok = endpointOk && connectCommandOk && connectedReadback
                && initializationOk && initializedConnectionReadback;
            evidence << QStringLiteral("端点有效=%1，连接来源=%2，Connect=%3，IsConnected回读=%4")
                .arg(endpointOk ? "OK" : "FAIL")
                .arg(reused ? "复用已有连接" : "本阶段新建连接")
                .arg(connectCommandOk ? "OK" : "FAIL")
                .arg(connectedReadback ? "已连接" : "未连接");
            evidence << QStringLiteral("InitializeAfterConnect=%1，初始化后连接回读=%2")
                .arg(initializationAttempted ? (initializationOk ? "OK" : "FAIL") : "未执行")
                .arg(initializedConnectionReadback ? "已连接" : "未连接");
            evidence << "连接后初始化摘要=" + (initializationSummary.empty()
                ? QStringLiteral("无") : DecodeRobotMessageText(initializationSummary));
            if (!ok)
            {
                if (connectedReadback && !initializationOk)
                {
                    evidence << QStringLiteral("通信连接已建立，但前置初始化失败，未就绪。");
                }
                evidence << "连接或初始化错误=" + DecodeRobotMessageText(driver->GetLastRobotError());
            }
            const QString result = evidence.join('\n');
            QMetaObject::invokeMethod(qApp, [self, ok, result]()
                {
                    if (self != nullptr)
                    {
                        self->FinishAdaptorAcceptanceStage(0, ok, result);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::RunAdaptorFtpRoundTrip()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::FtpFileTransfer, QStringLiteral("FTP闭环测试"));
    if (driver == nullptr) { return; }
    const RobotFileTransferProfile profile = driver->FileTransferProfile();
    if (QMessageBox::question(this, "FTP闭环测试",
        "请先在示教器当前工程内创建并保存一个专用测试JOB。底层将先下载该JOB，再按原路径同名回传，"
        "随后再次下载并核对SHA-256。不会新增、改名或删除工程文件；禁止选择main.pro或生产程序。是否继续？")
        != QMessageBox::Yes)
    {
        return;
    }
    std::string sessionError;
    const auto session = driver->CreateFileTransferSession(&sessionError);
    if (!session)
    {
        FinishAdaptorAcceptanceStage(1, false,
            QStringLiteral("创建FTP会话失败：%1").arg(QString::fromStdString(sessionError)));
        return;
    }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("机器人适配验收FTP闭环"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "FTP闭环测试", leaseError);
        return;
    }
    const QString remoteDirectory = QString::fromStdString(profile.defaultRemoteDirectory);
    const QString resultDirectory = AppPaths::WritablePath(
        QStringLiteral("Result/%1/AdaptorAcceptance/%2")
            .arg(AdaptorAcceptanceStorageRobotName(), m_adaptorAcceptanceRunId));
    QDir().mkpath(resultDirectory);
    const std::string remoteDirectoryBytes = remoteDirectory.toStdString();
    if (!BeginAdaptorAcceptanceStage(1)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, session, lease, profile, resultDirectory,
        remoteDirectoryBytes]()
        {
            QStringList evidence;
            std::vector<RobotControllerFileInfo> files;
            std::vector<std::pair<std::string, int>> pendingDirectories = {
                { remoteDirectoryBytes, 0 }
            };
            std::size_t directoryCursor = 0;
            std::size_t scannedDirectoryCount = 0;
            bool listOk = true;
            constexpr int kMaximumBrowseDepth = 4;
            constexpr std::size_t kMaximumDirectories = 64;
            while (directoryCursor < pendingDirectories.size()
                && directoryCursor < kMaximumDirectories)
            {
                const auto current = pendingDirectories[directoryCursor++];
                std::vector<RobotControllerFileInfo> entries;
                if (!session->ListProgramFiles(current.first, entries, 10000))
                {
                    listOk = false;
                    evidence << QStringLiteral("目录读取失败：%1；%2")
                        .arg(QString::fromStdString(current.first),
                            QString::fromStdString(session->LastError()));
                    break;
                }
                ++scannedDirectoryCount;
                for (const RobotControllerFileInfo& entry : entries)
                {
                    if (entry.isDirectory)
                    {
                        if (current.second < kMaximumBrowseDepth
                            && pendingDirectories.size() < kMaximumDirectories)
                        {
                            pendingDirectories.push_back({ entry.path, current.second + 1 });
                        }
                    }
                    else
                    {
                        files.push_back(entry);
                    }
                }
            }
            if (directoryCursor >= kMaximumDirectories
                && directoryCursor < pendingDirectories.size())
            {
                evidence << QStringLiteral("目录数量超过安全上限%1，停止继续递归。")
                    .arg(kMaximumDirectories);
            }
            files.erase(std::remove_if(files.begin(), files.end(),
                [&profile](const RobotControllerFileInfo& file)
                {
                    const QString name = QString::fromStdString(file.name).toLower();
                    if (name == QStringLiteral("main.pro")) { return true; }
                    return !std::any_of(
                        profile.acceptanceProgramExtensions.cbegin(),
                        profile.acceptanceProgramExtensions.cend(),
                        [&name](const std::string& extension)
                        {
                            return name.endsWith(QString::fromStdString(extension).toLower());
                        });
                }), files.end());
            evidence << QStringLiteral("FTP目录扫描：根目录=%1，已扫描目录=%2，可选JOB文件=%3")
                .arg(QString::fromStdString(remoteDirectoryBytes))
                .arg(scannedDirectoryCount).arg(files.size());
            if (!listOk || files.empty())
            {
                if (files.empty())
                {
                    evidence << "控制器目录中没有可用于回传的非main测试JOB。请先在示教器当前工程内创建并保存一个专用测试JOB，再重新测试。";
                }
                const QString result = evidence.join('\n');
                QMetaObject::invokeMethod(qApp, [self, result]()
                    {
                        if (self != nullptr)
                        {
                            self->FinishAdaptorAcceptanceStage(1, false, result);
                        }
                    }, Qt::QueuedConnection);
                return;
            }

            QString preferredExtension;
            if (!profile.localFileFilters.empty())
            {
                preferredExtension = QString::fromStdString(profile.localFileFilters.front());
                preferredExtension.remove('*');
                preferredExtension = preferredExtension.toLower();
            }
            std::stable_sort(files.begin(), files.end(), [&preferredExtension](
                const RobotControllerFileInfo& left,
                const RobotControllerFileInfo& right)
                {
                    const bool leftPreferred = QString::fromStdString(left.name)
                        .toLower().endsWith(preferredExtension);
                    const bool rightPreferred = QString::fromStdString(right.name)
                        .toLower().endsWith(preferredExtension);
                    if (leftPreferred != rightPreferred) { return leftPreferred; }
                    return QString::fromStdString(left.path).compare(
                        QString::fromStdString(right.path), Qt::CaseInsensitive) < 0;
                });
            QMetaObject::invokeMethod(qApp,
                [self, session, lease, files = std::move(files), evidence,
                    resultDirectory]() mutable
                {
                    if (self == nullptr) { return; }
                    QStringList choices;
                    choices.reserve(static_cast<int>(files.size()));
                    for (const RobotControllerFileInfo& file : files)
                    {
                        choices.push_back(QStringLiteral("%1  （%2 字节）")
                            .arg(QString::fromStdString(file.path)).arg(file.size));
                    }
                    bool accepted = false;
                    const QString choice = QInputDialog::getItem(
                        self,
                        "选择控制器专用测试JOB",
                        "将先下载所选文件，再按原路径同名回传并回读；禁止选择生产程序：",
                        choices, 0, false, &accepted);
                    if (!accepted || choice.isEmpty())
                    {
                        self->m_adaptorAcceptanceBusy = false;
                        self->m_bRobotCommandRunning = false;
                        self->m_adaptorAcceptanceStates[1] = "pending";
                        self->m_adaptorAcceptanceEvidence[1] =
                            "已读取控制器JOB列表，用户取消了文件选择；未执行下载、上传或删除。";
                        self->SaveAdaptorAcceptanceRun();
                        self->RefreshMotionButtonState();
                        self->RefreshAdaptorAcceptanceUi();
                        return;
                    }
                    const int selectedIndex = choices.indexOf(choice);
                    if (selectedIndex < 0 || selectedIndex >= static_cast<int>(files.size()))
                    {
                        self->FinishAdaptorAcceptanceStage(1, false,
                            "控制器JOB选择结果无效，未执行文件操作。");
                        return;
                    }
                    const RobotControllerFileInfo source = files[static_cast<std::size_t>(selectedIndex)];
                    const QString sourceRemotePath = QString::fromStdString(source.path);
                    const QString extension = QFileInfo(QString::fromStdString(source.name)).suffix();
                    QString remoteParent = sourceRemotePath;
                    remoteParent.replace('\\', '/');
                    const int slash = remoteParent.lastIndexOf('/');
                    remoteParent = slash >= 0 ? remoteParent.left(slash) : QString();
                    if (QMessageBox::question(self, "确认回传测试JOB",
                        QStringLiteral("即将测试：%1\n\n必须确认这是专用测试JOB、当前未运行，且允许按原路径同名回传。继续？")
                            .arg(sourceRemotePath)) != QMessageBox::Yes)
                    {
                        self->m_adaptorAcceptanceBusy = false;
                        self->m_bRobotCommandRunning = false;
                        self->m_adaptorAcceptanceStates[1] = "pending";
                        self->m_adaptorAcceptanceEvidence[1] =
                            "用户未确认所选文件为可回传的专用测试JOB；未执行下载或上传。";
                        self->SaveAdaptorAcceptanceRun();
                        self->RefreshMotionButtonState();
                        self->RefreshAdaptorAcceptanceUi();
                        return;
                    }
                    const QString localToken = QString::number(
                        QDateTime::currentMSecsSinceEpoch());
                    const QString sourceLocalPath = QDir(resultDirectory)
                        .filePath(QStringLiteral("ftp_source_%1%2")
                            .arg(localToken, extension.isEmpty() ? QString() : "." + extension));
                    const QString roundTripLocalPath = QDir(resultDirectory)
                        .filePath(QStringLiteral("ftp_roundtrip_%1%2")
                            .arg(localToken, extension.isEmpty() ? QString() : "." + extension));
                    const std::string sourceRemoteBytes = sourceRemotePath.toStdString();
                    const std::string sourceLocalBytes = QDir::toNativeSeparators(sourceLocalPath)
                        .toLocal8Bit().toStdString();
                    const std::string roundTripLocalBytes = QDir::toNativeSeparators(roundTripLocalPath)
                        .toLocal8Bit().toStdString();
                    std::thread([self, session, lease, evidence = std::move(evidence), source,
                        sourceRemotePath, remoteParent,
                        sourceLocalPath, roundTripLocalPath,
                        sourceRemoteBytes, sourceLocalBytes,
                        roundTripLocalBytes]() mutable
                        {
                            evidence.prepend(QStringLiteral(
                                "控制器工程/任务目录=%1\n"
                                "下载与同名回传JOB=%2\n"
                                "远端不新增文件、不删除文件\n")
                                .arg(remoteParent, sourceRemotePath));
                            evidence << QStringLiteral("下载源JOB声明大小=%1字节")
                                .arg(source.size);
                            bool ok = session->DownloadProgramFile(
                                sourceRemoteBytes, sourceLocalBytes);
                            evidence << QStringLiteral("第一步 下载原JOB=%1，本地=%2")
                                .arg(ok ? "OK" : "FAIL", sourceLocalPath);
                            QByteArray sourceHash;
                            qint64 sourceSize = 0;
                            QString hashError;
                            if (ok)
                            {
                                ok = FileSha256(sourceLocalPath, sourceHash, sourceSize, &hashError)
                                    && (source.size == 0
                                        || sourceSize == static_cast<qint64>(source.size));
                                evidence << QStringLiteral("下载文件校验=%1，大小=%2字节，SHA-256=%3")
                                    .arg(ok ? "OK" : "FAIL").arg(sourceSize)
                                    .arg(QString::fromLatin1(sourceHash.toHex()));
                                if (!hashError.isEmpty()) { evidence << hashError; }
                            }
                            bool uploaded = false;
                            if (ok)
                            {
                                uploaded = session->UploadProgramFile(
                                    sourceLocalBytes, sourceRemoteBytes, false);
                                ok = uploaded;
                                evidence << QStringLiteral("第二步 按原路径同名回传=%1，远端=%2；未执行上传前删除")
                                    .arg(ok ? "OK" : "FAIL", sourceRemotePath);
                            }
                            if (ok)
                            {
                                ok = session->DownloadProgramFile(
                                    sourceRemoteBytes, roundTripLocalBytes);
                                evidence << QStringLiteral("第三步 下载同名文件回读=%1，本地=%2")
                                    .arg(ok ? "OK" : "FAIL", roundTripLocalPath);
                            }
                            if (ok)
                            {
                                QByteArray roundTripHash;
                                qint64 roundTripSize = 0;
                                ok = FileSha256(roundTripLocalPath, roundTripHash,
                                    roundTripSize, &hashError)
                                    && sourceSize == roundTripSize
                                    && sourceHash == roundTripHash;
                                evidence << QStringLiteral("回读SHA-256/大小=%1，源=%2字节，回读=%3字节，SHA=%4")
                                    .arg(ok ? "一致" : "不一致")
                                    .arg(sourceSize).arg(roundTripSize)
                                    .arg(QString::fromLatin1(sourceHash.toHex()));
                                if (!hashError.isEmpty()) { evidence << hashError; }
                            }
                            evidence << "第四步 清理=无需删除；控制器文件名和工程文件数保持不变";
                            if (!ok && !session->LastError().empty())
                            {
                                evidence << QStringLiteral("FTP错误=%1")
                                    .arg(QString::fromStdString(session->LastError()));
                            }
                            const QString result = evidence.join('\n');
                            QMetaObject::invokeMethod(qApp, [self, ok, result]()
                                {
                                    if (self != nullptr)
                                    {
                                        self->FinishAdaptorAcceptanceStage(1, ok, result);
                                    }
                                }, Qt::QueuedConnection);
                        }).detach();
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::RunAdaptorProgramDownlink()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapabilities(
        { RobotDriverCapability::ContinuousTrajectory,
          RobotDriverCapability::OfflineTrajectoryExport,
          RobotDriverCapability::NativeProgramUpload,
          RobotDriverCapability::FtpFileTransfer },
        QStringLiteral("测试程序生成和下发"));
    if (driver == nullptr) { return; }
    const double programSpeed = m_pAdaptorAcceptanceProgramSpeedSpin->value();
    std::string speedError;
    if (!driver->ValidateLinearSpeedMmPerMin(programSpeed, &speedError))
    {
        QMessageBox::warning(this, "测试程序生成和下发",
            QString::fromStdString(speedError));
        return;
    }
    if (QMessageBox::question(this, "测试程序生成和下发",
        "将读取当前位置，生成只包含“以低速到当前位置”的单点空跑程序并下发到控制器。"
        "本步骤不会调用StartTrajectory、不会启动程序。下发后必须由人工在示教器上只做加载/语法检查。是否继续？")
        != QMessageBox::Yes)
    {
        return;
    }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("适配验收测试程序下发"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "测试程序生成和下发", leaseError);
        return;
    }
    if (!BeginAdaptorAcceptanceStage(2)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, driver, lease, programSpeed]()
        {
            T_ROBOT_COORS current;
            bool ok = driver->TryGetCurrentPos(current);
            RobotTrajectoryHandle handle;
            std::vector<T_ROBOT_MOVE_INFO> moves;
            if (ok)
            {
                T_ROBOT_MOVE_INFO move = {};
                move.nMoveType = MOVL;
                move.nPosType = POSVAR;
                move.tCoord = current;
                move.tSpeed = T_ROBOT_MOVE_SPEED(programSpeed, 0.0, 0.0);
                move.nMoveDevice = 0;
                move.nTrackNo = 0;
                moves.push_back(move);
                ok = driver->ReserveTrajectory(RobotTrajectoryPurpose::WeldDryRun, handle)
                    && driver->DownlinkTrajectory(
                        moves, RobotTrajectoryPurpose::WeldDryRun, handle);
            }
            QString evidence = ok
                ? QStringLiteral("单点空跑程序已生成并下发，但未启动。测试速度=%1 mm/min；程序=%2；本地程序=%3；本地数据=%4；远端程序=%5；远端数据=%6。")
                    .arg(programSpeed, 0, 'f', 0)
                    .arg(QString::fromStdString(handle.programName),
                        QString::fromLocal8Bit(handle.localProgramPath.c_str()),
                        QString::fromLocal8Bit(handle.localDataPath.c_str()),
                        QString::fromStdString(handle.remoteProgramPath),
                        QString::fromStdString(handle.remoteDataPath))
                : QStringLiteral("测试程序生成/下发失败：%1")
                    .arg(DecodeRobotMessageText(driver->GetLastRobotError()));
            if (ok)
            {
                evidence += "\n请在示教器确认：文件可见、工程/任务可加载、语法无报错；禁止在本阶段运行。";
            }
            QMetaObject::invokeMethod(qApp, [self, ok, evidence]()
                {
                    if (self != nullptr)
                    {
                        self->FinishAdaptorAcceptanceStage(2, ok, evidence, ok);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::RunAdaptorPositionStatusCheck()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::PassiveState, QStringLiteral("位置和状态读取"));
    if (driver == nullptr) { return; }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("适配验收位置状态读取"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "位置和状态读取", leaseError);
        return;
    }
    if (!BeginAdaptorAcceptanceStage(3)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, driver, lease]()
        {
            T_ROBOT_COORS firstPose;
            T_ROBOT_COORS secondPose;
            T_ANGLE_PULSE pulse;
            bool ok = driver->TryGetCurrentPos(firstPose)
                && driver->TryGetCurrentPulse(pulse);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ok = driver->TryGetCurrentPos(secondPose) && ok;
            const RobotMotionStatus motion = driver->ReadMotionStatus();
            ok = ok && motion.state != RobotMotionState::Unknown;
            QString evidence = QStringLiteral(
                "位姿1：%1\n位姿2：%2\n关节脉冲：J1=%3 J2=%4 J3=%5 J4=%6 J5=%7 J6=%8\n"
                "完成状态：state=%9 raw=%10 terminalVerified=%11 detail=%12")
                .arg(FormatAcceptancePose(firstPose), FormatAcceptancePose(secondPose))
                .arg(pulse.nSPulse).arg(pulse.nLPulse).arg(pulse.nUPulse)
                .arg(pulse.nRPulse).arg(pulse.nBPulse).arg(pulse.nTPulse)
                .arg(static_cast<int>(motion.state)).arg(motion.rawCode)
                .arg(motion.terminalVerified)
                .arg(QString::fromStdString(motion.detail));
            if (driver->Supports(RobotDriverCapability::StructuredControllerStatus))
            {
                const RobotControllerStatus status = driver->ReadControllerStatus();
                evidence += QStringLiteral("\n结构化状态：valid=%1 eStop=%2 servo=%3 fault=%4 warning=%5 mode=%6 permit=%7")
                    .arg(status.valid).arg(status.emergencyStop).arg(status.servoPowered)
                    .arg(status.systemFault).arg(status.systemWarning)
                    .arg(status.rawOperationMode).arg(status.hasControlPermit);
                ok = ok && status.valid;
            }
            if (!ok)
            {
                evidence += "\n错误：" + DecodeRobotMessageText(driver->GetLastRobotError());
            }
            QMetaObject::invokeMethod(qApp, [self, ok, evidence]()
                {
                    if (self != nullptr)
                    {
                        self->FinishAdaptorAcceptanceStage(3, ok, evidence);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::RunAdaptorModeCombinationTests(bool allCases)
{
    if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning || m_adaptorAcceptanceMovedOut || m_adaptorJointMovedOut) { return; }
    auto* driver = GetFirstRobotDriverAdaptor();
    if (driver == nullptr) { return; }
    auto cases = driver->ModePreparationTestCases();
    if (!allCases)
    {
        const std::string selected = m_pAdaptorModeCombinationCombo->currentData().toString().toStdString();
        cases.erase(std::remove_if(cases.begin(), cases.end(), [&selected](const auto& item) { return item.id != selected; }), cases.end());
    }
    if (cases.empty()) { QMessageBox::warning(this, "模式组合测试", "当前品牌没有可测试的组合。"); return; }
    QString error;
    if (!AdaptorAcceptancePrerequisitesReady(4, &error)) { QMessageBox::warning(this, "模式组合测试", error); return; }
    if (QMessageBox::warning(this, "模式组合测试确认", QStringLiteral(
        "将测试 %1 个模式/上电/数据流组合，不发送任何位移、不启动JOB、不释放实体急停。\n"
        "请确认周围安全、握住示教器，并确认机器人已停止、主任务停止/就绪、伺服已下电、数据流已关闭。"
        "按需清除已释放急停后的报警；每组后恢复原模式和下电状态，恢复失败立即终止后续组合。是否开始？").arg(cases.size()),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Ok) { return; }
    const auto lease = RobotOperationLease::TryAcquire(driver, "适配验收无位移模式组合测试", &error);
    if (!lease) { QMessageBox::warning(this, "模式组合测试", error); return; }
    if (!BeginAdaptorAcceptanceStage(4)) { return; }
    m_adaptorModeBatchRunning = true;
    m_adaptorAcceptanceMotionRoundTripCompleted = false;
    const QString priorEvidence = m_adaptorAcceptanceEvidence.value(4).trimmed();
    if (!priorEvidence.isEmpty() && !m_adaptorModeCombinationEvidence.contains(priorEvidence))
    { m_adaptorModeCombinationEvidence += "\n此前阶段4证据：\n" + priorEvidence + "\n"; }
    m_adaptorModeCombinationEvidence += QStringLiteral("\n=== 模式组合测试 %1，共 %2 组 ===\n")
        .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs)).arg(cases.size());
    if (!SaveAdaptorAcceptanceRun())
    {
        m_adaptorModeBatchRunning = false;
        m_adaptorAcceptanceBusy = false;
        m_bRobotCommandRunning = false;
        m_adaptorAcceptanceStates[4] = "fail";
        RefreshAdaptorAcceptanceUi();
        return;
    }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, driver, lease, cases]()
    {
        int passed = 0, attempted = 0;
        bool canContinue = true;
        QString summary;
        const auto before = driver->ReadControllerStatus();
        RobotAcceptanceAlarmPreparation::Result<RobotControllerStatus> alarm;
        if (RobotOperationLease::IsCancellationRequested(driver))
        { alarm.blockReason = "用户已停止，未发送报警复位和组合命令。"; }
        else if (!before.valid || !before.servoPowerKnown || before.servoPowered)
        { alarm.blockReason = "组合测试前必须确认状态有效且伺服已下电；未发送报警复位或测试命令。"; }
        else
        { alarm = RobotAcceptanceAlarmPreparation::Prepare(*driver, before,
            driver->Supports(RobotDriverCapability::StructuredControllerStatus), driver->Supports(RobotDriverCapability::AlarmReset)); }
        if (!alarm.ready)
        {
            canContinue = false;
            summary = QString::fromUtf8(alarm.blockReason.c_str()) + "\n" + DecodeRobotMessageText(alarm.commandError);
        }
        if (alarm.attempted)
        { summary += QStringLiteral("\n准备前报警复位：接口=%1，回读=%2，故障码=%3 -> %4\n")
            .arg(alarm.commandOk ? "OK" : "FAIL").arg(alarm.readbackOk ? "OK" : "FAIL")
            .arg(before.systemErrorCode).arg(alarm.after.systemErrorCode); }
        for (const auto& item : cases)
        {
            if (!canContinue || RobotOperationLease::IsCancellationRequested(driver)) { break; }
            bool startSaved = false;
            const QString started = QStringLiteral("\n开始组合 %1：%2（%3）\n")
                .arg(QString::fromStdString(item.id), QString::fromUtf8(item.name.c_str()),
                    QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
            QMetaObject::invokeMethod(qApp, [self, started, &startSaved]()
            {
                if (self == nullptr) { return; }
                self->m_adaptorModeCombinationEvidence += started;
                self->m_adaptorAcceptanceEvidence[4] = self->m_adaptorModeCombinationEvidence;
                if (self->m_adaptorAcceptanceSelectedStage == 4)
                { self->m_pAdaptorAcceptanceEvidence->setPlainText(self->m_adaptorAcceptanceEvidence[4]); }
                startSaved = self->SaveAdaptorAcceptanceRun();
            }, Qt::BlockingQueuedConnection);
            if (!startSaved) { canContinue = false; summary += "\n组合开始记录未保存，未发送该组合命令。"; break; }
            RobotModePreparationTestResult result;
            const bool ok = driver->RunModePreparationTestCase(item.id, result);
            ++attempted;
            if (ok && result.passed && result.restoreVerified) { ++passed; }
            canContinue = result.restoreVerified;
            const QString evidence = QString::fromUtf8(result.evidence.c_str());
            bool saved = false;
            QMetaObject::invokeMethod(qApp, [self, evidence, &saved]()
            {
                if (self == nullptr) { return; }
                self->m_adaptorModeCombinationEvidence += evidence + "\n";
                self->m_adaptorAcceptanceEvidence[4] = self->m_adaptorModeCombinationEvidence;
                if (self->m_adaptorAcceptanceSelectedStage == 4)
                { self->m_pAdaptorAcceptanceEvidence->setPlainText(self->m_adaptorAcceptanceEvidence[4]); }
                saved = self->SaveAdaptorAcceptanceRun();
                self->RefreshAdaptorAcceptanceUi();
            }, Qt::BlockingQueuedConnection);
            if (!saved) { canContinue = false; summary += "\n记录保存失败，已终止后续组合。"; }
        }
        for (int index = attempted; index < static_cast<int>(cases.size()); ++index)
        { summary += "\n未执行：" + QString::fromStdString(cases[index].id) + " " + QString::fromUtf8(cases[index].name.c_str()); }
        summary += QStringLiteral("\n组合测试：计划%1组，执行%2组，通过%3组；%4。\n"
            "通过仅证明准备和恢复命令闭环，不代表位移测试通过；请选用通过组合后单独执行10mm外移和返回。")
            .arg(cases.size()).arg(attempted).arg(passed)
            .arg(!canContinue ? "恢复/前置条件/保存未通过，后续组合未执行"
                : (RobotOperationLease::IsCancellationRequested(driver) ? "用户已停止，剩余组合未执行" : "已完成"));
        QMetaObject::invokeMethod(qApp, [self, summary, canContinue, passed]()
        {
            if (self == nullptr) { return; }
            self->m_adaptorModeBatchRunning = false;
            self->m_adaptorAcceptanceBusy = false;
            self->m_bRobotCommandRunning = false;
            self->m_adaptorModeCombinationEvidence += summary + "\n";
            self->m_adaptorAcceptanceEvidence[4] = self->m_adaptorModeCombinationEvidence;
            self->m_adaptorAcceptanceStates[4] = canContinue && passed > 0 ? "pending" : "fail";
            if (self->m_adaptorAcceptanceSelectedStage == 4)
            { self->m_pAdaptorAcceptanceEvidence->setPlainText(self->m_adaptorAcceptanceEvidence[4]); }
            self->SaveAdaptorAcceptanceRun();
            self->RefreshMotionButtonState();
            self->RefreshAdaptorAcceptanceUi();
            QMessageBox::information(self, "模式组合测试结果", summary);
        }, Qt::QueuedConnection);
    }).detach();
}

void FunctionTestDialog::RunAdaptorSafeLinearMotion(bool jointMotion)
{
    if (m_adaptorAcceptanceBusy || m_bRobotCommandRunning) { return; }
    if ((jointMotion && m_adaptorAcceptanceMovedOut) || (!jointMotion && m_adaptorJointMovedOut))
    { QMessageBox::warning(this, "低速验收", "请先完成当前类型运动的返回，再切换测试类型。"); return; }
    QString prerequisiteError;
    if (!AdaptorAcceptancePrerequisitesReady(4, &prerequisiteError))
    { QMessageBox::warning(this, "低速验收", prerequisiteError); return; }
    RobotDriverAdaptor* driver = GetFirstDriverWithCapabilities(
        { jointMotion ? RobotDriverCapability::JointMotion : RobotDriverCapability::LinearMotion,
          RobotDriverCapability::PassiveState,
          RobotDriverCapability::OperationModeControl,
          RobotDriverCapability::ServoPowerControl,
          RobotDriverCapability::VerifiedProgramCompletion,
          RobotDriverCapability::VerifiedSafeAbort },
        jointMotion ? QStringLiteral("关节往返专项") : QStringLiteral("低速直线验收运动"));
    if (driver == nullptr) { return; }
    if (jointMotion && driver->ExternalAxleType() != 0)
    { QMessageBox::warning(this, "关节专项", "此专项暂仅验收六轴机器人本体；带外轴配置需要专门验证外轴保持语义，不发送运动。"); return; }
    if (!driver->ModePreparationTestCases().empty() && driver->ActiveModePreparationId().empty())
    { QMessageBox::warning(this, "低速真机运动确认", "当前机器人没有可用的已固化模式组合。请先完成模式组合测试，并点击“选用已通过组合”保存到数据库。旧验收记录未固化的，需要重新测试并选用一次；绑定不匹配时也需重新验证。未验证的顺序不会用于真实运动。"); return; }
    const QMessageBox::StandardButton safetyConfirmation = QMessageBox::warning(
        this,
        "低速真机运动确认",
        (jointMotion ? QStringLiteral("关节专项：J1 +0.5° / 1%速度，再单独确认返回。其他轴保持原值。关节轨迹不是直线，请检查整个机械臂扫掠空间。\n")
            : QStringLiteral("测试方向固定为当前机器人基坐标 +Y。\n")) + QStringLiteral(
            "请确认实体急停已解除、机器人周围安全。支持状态回读的品牌将按需自动复位报警；复位失败或报警未清除时不会上电或运动。\n"
            "请确认已握住示教器并准备急停；按“确定”继续，按“取消”不发送任何运动命令。"),
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Cancel);
    if (safetyConfirmation != QMessageBox::Ok)
    {
        return;
    }
    const double distance = jointMotion ? RobotAcceptanceJointMotion::DeltaDegrees : m_pAdaptorAcceptanceDistanceSpin->value();
    const double speed = jointMotion ? RobotAcceptanceJointMotion::SpeedPercent : m_pAdaptorAcceptanceSpeedSpin->value();
    std::string speedError;
    if (!jointMotion && !driver->ValidateLinearSpeedMmPerMin(speed, &speedError))
    {
        QMessageBox::warning(this, "低速真机运动确认",
            QString::fromStdString(speedError));
        return;
    }
    QString leaseError;
    const auto lease = jointMotion && m_adaptorJointMovedOut ? m_adaptorJointLease
        : RobotOperationLease::TryAcquire(driver,
            jointMotion ? QStringLiteral("适配验收关节MOVJ") : QStringLiteral("适配验收低速MOVL"), &leaseError);
    if (!lease)
    { QMessageBox::warning(this, "低速真机运动确认", leaseError); return; }
    if (jointMotion && (!lease->Matches(driver) || lease->CancellationRequested()))
    {
        m_adaptorJointOriginalPulseValid = false;
        m_adaptorJointMovedOut = false;
        m_adaptorJointLease.reset();
        m_adaptorJointMotionState = "fail";
        m_adaptorJointMotionEvidence += "\n运动租约身份变化或已停止取消，禁止自动返回，请现场核对。";
        SaveAdaptorAcceptanceRun(); RefreshAdaptorAcceptanceUi();
        QMessageBox::warning(this, "关节验收", m_adaptorJointMotionEvidence); return;
    }
    T_ROBOT_COORS target;
    T_ANGLE_PULSE jointTarget, jointStart;
    const T_AXISUNIT jointUnits = driver->AxisUnit();
    const bool returning = jointMotion ? m_adaptorJointMovedOut : m_adaptorAcceptanceMovedOut;
    if (jointMotion)
    {
        if (!RobotAcceptanceJointMotion::ValidUnits(jointUnits) || !driver->TryGetCurrentPulse(jointStart))
        {
            m_adaptorJointOriginalPulseValid = false;
            m_adaptorJointMovedOut = false;
            m_adaptorJointLease.reset();
            m_adaptorJointMotionState = "fail";
            m_adaptorJointMotionEvidence += "\n获取当前关节/有效轴换算单位失败，原点作废；请重新获取标定资产并现场核对。";
            SaveAdaptorAcceptanceRun(); RefreshAdaptorAcceptanceUi();
            QMessageBox::warning(this, "关节验收", m_adaptorJointMotionEvidence); return;
        }
        if (returning)
        {
            if (!m_adaptorJointOriginalPulseValid)
            { QMessageBox::warning(this, "关节验收", "本次原点已失效，禁止猜测返回。"); return; }
            T_ANGLE_PULSE expectedOutward;
            double startError = 0;
            if (!RobotAcceptanceJointMotion::OutwardTarget(m_adaptorJointOriginalPulse, jointUnits, expectedOutward)
                || !RobotAcceptanceJointMotion::Matches(jointStart, expectedOutward, jointUnits, startError))
            {
                m_adaptorJointOriginalPulseValid = false;
                m_adaptorJointMovedOut = false;
                m_adaptorJointLease.reset();
                m_adaptorJointMotionState = "fail";
                m_adaptorJointMotionEvidence += "\n返回前位置已变化，原点已作废；请现场处理后重新测试。";
                SaveAdaptorAcceptanceRun(); RefreshAdaptorAcceptanceUi();
                QMessageBox::warning(this, "关节验收", m_adaptorJointMotionEvidence); return;
            }
            jointTarget = m_adaptorJointOriginalPulse;
        }
        else
        {
            if (!RobotAcceptanceJointMotion::OutwardTarget(jointStart, jointUnits, jointTarget))
            { QMessageBox::warning(this, "关节验收", "J1脉冲换算或范围检查失败，未发送运动。"); return; }
            m_adaptorJointOriginalPulse = jointStart;
            m_adaptorJointOriginalPulseValid = true;
        }
        const auto degrees = PulseToJointDegrees(jointTarget, jointUnits);
        for (int i = 0; i < 6; ++i)
        {
            const double minimum = driver->AxisLimitAngles().GetMinAngleByIndex(i);
            const double maximum = driver->AxisLimitAngles().GetMaxAngleByIndex(i);
            if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum
                || !std::isfinite(degrees[i]) || degrees[i] < minimum || degrees[i] > maximum)
            { QMessageBox::warning(this, "关节验收", QString("J%1限位缺失或目标超限，未发送运动。").arg(i + 1)); return; }
        }
    }
    else if (returning)
    {
        if (!m_adaptorAcceptanceOriginalPoseValid)
        {
            QMessageBox::warning(this, "低速真机运动确认",
                "原始位姿只保存在本次界面会话中，当前已丢失；禁止猜测返回点，请人工处理后重新开始本阶段。");
            return;
        }
        target = m_adaptorAcceptanceOriginalPose;
    }
    else
    {
        m_adaptorAcceptanceMotionRoundTripCompleted = false;
        if (!driver->TryGetCurrentPos(m_adaptorAcceptanceOriginalPose))
        {
            QMessageBox::warning(this, "低速真机运动确认",
                "捕获原始位姿失败：" + DecodeRobotMessageText(driver->GetLastRobotError()));
            return;
        }
        m_adaptorAcceptanceOriginalPoseValid = true;
        target = m_adaptorAcceptanceOriginalPose;
        target.dY += distance;
    }
    const QString action = returning ? QStringLiteral("返回原始位姿")
        : (jointMotion ? QStringLiteral("J1 +0.5°关节移动") : QStringLiteral("基坐标+Y移动%1 mm").arg(distance, 0, 'f', 1));
    const QString targetText = jointMotion
        ? QString("脉冲 J1=%1 J2=%2 J3=%3 J4=%4 J5=%5 J6=%6，外轴保持捕获值")
            .arg(jointTarget.nSPulse).arg(jointTarget.nLPulse).arg(jointTarget.nUPulse)
            .arg(jointTarget.nRPulse).arg(jointTarget.nBPulse).arg(jointTarget.nTPulse)
        : FormatAcceptancePose(target);
    if (QMessageBox::question(this, "最后运动确认",
        QStringLiteral("动作：%1\n速度：%2 %3\n目标：%4\n\n请继续握住示教器。是否发送这一条运动？")
            .arg(action).arg(speed, 0, 'f', 0).arg(jointMotion ? "%" : "mm/min").arg(targetText),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
        != QMessageBox::Yes)
    {
        return;
    }
    if (jointMotion)
    {
        m_adaptorJointMotionState = "running";
        m_adaptorJointMotionEvidence += "\n" + action + "：开始，等待安全准备和终态回读。";
        if (!SaveAdaptorAcceptanceRun())
        { m_adaptorJointMotionState = "fail"; return; }
        m_adaptorAcceptanceBusy = true;
        m_bRobotCommandRunning = true;
    }
    else if (!BeginAdaptorAcceptanceStage(4)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    const int externalAxisType = driver->ExternalAxleType();
    std::thread([self, driver, lease, target, speed, action, returning, distance, externalAxisType,
        jointMotion, jointTarget, jointStart, jointUnits, targetText]()
        {
            const bool hasStructuredStatus =
                driver->Supports(RobotDriverCapability::StructuredControllerStatus);
            const RobotControllerStatus before = hasStructuredStatus
                ? driver->ReadControllerStatus() : RobotControllerStatus{};
            const auto alarmPreparation = RobotAcceptanceAlarmPreparation::Prepare(
                *driver, before, hasStructuredStatus,
                driver->Supports(RobotDriverCapability::AlarmReset));

            // The brand owns a verified stream sequence. Do not force automatic
            // mode or Motor ON before its tested ordering; Move prepares it.
            const bool brandPreparesStream = !driver->ModePreparationTestCases().empty();
            driver->ClearLastRobotError();
            const bool modeAttempted = alarmPreparation.ready && !brandPreparesStream;
            const bool modeOk = alarmPreparation.ready
                && (brandPreparesStream || driver->SetOperationMode(RobotOperationMode::Automatic));
            const QString modeError = !modeAttempted || modeOk ? QString()
                : DecodeRobotMessageText(driver->GetLastRobotError());
            driver->ClearLastRobotError();
            const bool servoAttempted = modeOk && !brandPreparesStream;
            const bool servoOk = modeOk && (brandPreparesStream || driver->ServoOn());
            const QString servoError = !servoAttempted || servoOk ? QString()
                : DecodeRobotMessageText(driver->GetLastRobotError());

            const bool preparedReadAttempted = alarmPreparation.ready && hasStructuredStatus;
            const RobotControllerStatus prepared = preparedReadAttempted
                ? driver->ReadControllerStatus() : RobotControllerStatus{};
            const QString preparedError = preparedReadAttempted && !prepared.valid
                ? DecodeRobotMessageText(driver->GetLastRobotError()) : QString();
            const bool preparedStatusOk = !hasStructuredStatus || (prepared.valid
                && (brandPreparesStream || prepared.operationMode == RobotOperationMode::Automatic)
                && prepared.servoPowerKnown && (brandPreparesStream || prepared.servoPowered)
                && prepared.emergencyStopKnown && !prepared.emergencyStop
                && prepared.systemFaultKnown && !prepared.systemFault && !prepared.systemWarning
                && prepared.motion.terminalVerified
                && (!prepared.controlOwnerKnown || prepared.controlOwnedByApi)
                && (!prepared.controlPermitKnown || prepared.hasControlPermit));
            // The operator may have jogged at the pendant while confirmation was open.
            // Re-read all axes immediately before sending a small joint step.
            T_ANGLE_PULSE jointBeforeSend;
            double startErrorDegrees = 0;
            const bool jointStartOk = !jointMotion ||
                (RobotAcceptanceJointMotion::Units(driver->AxisUnit()) == RobotAcceptanceJointMotion::Units(jointUnits)
                    && driver->TryGetCurrentPulse(jointBeforeSend)
                    && RobotAcceptanceJointMotion::Matches(jointBeforeSend, jointStart, jointUnits, startErrorDegrees));
            const bool setupOk = alarmPreparation.ready && modeOk && servoOk && preparedStatusOk && jointStartOk;

            driver->ClearLastRobotError();
            QString trackingError;
            const bool tracked = setupOk
                && RobotOperationLease::MarkMotionStarted(driver, false, &trackingError);
            const bool moveOk = tracked && (jointMotion
                ? driver->MoveJointPercent(jointTarget, speed, externalAxisType)
                : driver->MoveLinearMmPerMin(target, speed, externalAxisType));
            const int done = moveOk ? driver->CheckRobotDone(100, 1800000) : -1;
            bool completed = false;
            if (moveOk && done == 1)
            {
                completed = RobotOperationLease::MarkMotionCompleted(driver);
            }
            T_ANGLE_PULSE jointActual;
            double jointErrorDegrees = 0;
            const bool jointReadbackOk = !jointMotion || (moveOk && done == 1 && completed
                && driver->TryGetCurrentPulse(jointActual)
                && RobotAcceptanceJointMotion::Matches(jointActual, jointTarget, jointUnits, jointErrorDegrees));
            const bool motionOk = moveOk && done == 1 && completed && jointReadbackOk;
            QString motionError = DecodeRobotMessageText(driver->GetLastRobotError());
            bool aborted = true;
            if (!motionOk && tracked)
            {
                aborted = RobotOperationLease::StopAndConfirmUnverifiedMotion(driver);
            }

            bool servoRestoreOk = true;
            bool modeRestoreOk = true;
            bool servoRestoreAttempted = false;
            bool modeRestoreAttempted = false;
            QString restoreError;
            if (hasStructuredStatus && before.valid)
            {
                if (before.servoPowerKnown && !before.servoPowered
                    && (servoOk || (prepared.valid && prepared.servoPowerKnown && prepared.servoPowered)))
                {
                    servoRestoreAttempted = true;
                    driver->ClearLastRobotError();
                    servoRestoreOk = driver->ServoOff();
                    if (!servoRestoreOk)
                    {
                        restoreError += QStringLiteral("伺服状态恢复失败：%1")
                            .arg(DecodeRobotMessageText(driver->GetLastRobotError()));
                    }
                }
                if (before.operationMode != RobotOperationMode::Unknown && modeOk)
                {
                    modeRestoreAttempted = true;
                    driver->ClearLastRobotError();
                    modeRestoreOk = driver->SetOperationMode(before.operationMode);
                    if (!modeRestoreOk)
                    {
                        if (!restoreError.isEmpty()) { restoreError += "；"; }
                        restoreError += QStringLiteral("运行模式恢复失败：%1")
                            .arg(DecodeRobotMessageText(driver->GetLastRobotError()));
                    }
                }
            }
            bool restoredReadbackOk = true;
            if (jointMotion && hasStructuredStatus && before.valid)
            {
                const auto restored = driver->ReadControllerStatus();
                restoredReadbackOk = restored.valid && restored.motion.terminalVerified
                    && (!before.servoPowerKnown || (restored.servoPowerKnown && restored.servoPowered == before.servoPowered))
                    && (before.operationMode == RobotOperationMode::Unknown || restored.operationMode == before.operationMode);
            }
            const bool restoreOk = servoRestoreOk && modeRestoreOk && restoredReadbackOk;
            const bool ok = setupOk && motionOk && aborted && restoreOk;
            QStringList detail;
            if (jointMotion)
            {
                detail << QStringLiteral("关节专项：起点重检=%1（最大差%2°）；终态6轴/外轴回读=%3（最大角差%4°）；恢复状态回读=%5。")
                    .arg(jointStartOk ? "OK" : "FAIL").arg(startErrorDegrees, 0, 'f', 6)
                    .arg(jointReadbackOk ? "OK" : "FAIL").arg(jointErrorDegrees, 0, 'f', 6)
                    .arg(restoredReadbackOk ? "OK" : "FAIL");
            }
            if (!driver->ActiveModePreparationId().empty())
            { detail << "本次数据流组合：" + QString::fromStdString(driver->ActiveModePreparationId())
                + "；由品牌底层在运动发送前按该组合切模式和上电，业务层不预先强制自动模式。"; }
            const auto outcome = [](bool attempted, bool success) -> QString
            { return attempted ? (success ? QStringLiteral("OK") : QStringLiteral("FAIL")) : QStringLiteral("未执行"); };
            const auto formatStatus = [](const RobotControllerStatus& status) -> QString
            {
                const auto knownFlag = [](bool known, bool value) -> QString
                { return known ? QString::number(value) : QStringLiteral("未知"); };
                const QString summary = QStringLiteral("valid=%1 mode=%2 eStop=%3 servo=%4 fault=%5 warning=%6 code=%7 owner=%8 permit=%9")
                    .arg(status.valid).arg(status.rawOperationMode)
                    .arg(knownFlag(status.emergencyStopKnown, status.emergencyStop))
                    .arg(knownFlag(status.servoPowerKnown, status.servoPowered))
                    .arg(knownFlag(status.systemFaultKnown, status.systemFault))
                    .arg(knownFlag(status.systemFaultKnown, status.systemWarning))
                    .arg(status.systemErrorCode < 0 ? QStringLiteral("未知")
                        : "0x" + QString::number(status.systemErrorCode, 16).toUpper().rightJustified(4, '0'))
                    .arg(status.rawControlOwner).arg(status.rawPermitState);
                return status.valid ? summary
                    : summary + "；读取错误：" + DecodeRobotMessageText(status.detail);
            };
            if (hasStructuredStatus)
            {
                detail << "准备前状态：" + formatStatus(before);
                if (alarmPreparation.readbackAttempted)
                {
                    detail << "报警复位后状态：" + formatStatus(alarmPreparation.after);
                }
            }
            detail << QStringLiteral("报警复位：接口=%1，复位后状态验证=%2")
                .arg(outcome(alarmPreparation.attempted, alarmPreparation.commandOk))
                .arg(outcome(alarmPreparation.readbackAttempted, alarmPreparation.readbackOk));
            if (!hasStructuredStatus)
            {
                detail << "当前品牌不支持完整急停/报警状态回读，未自动复位报警，请在示教器确认；保留原有品牌上电流程。";
            }
            else if (!alarmPreparation.attempted && alarmPreparation.ready)
            {
                detail << "急停已解除且无报警，无需发送报警复位。";
            }
            if (!alarmPreparation.commandError.empty())
            { detail << "报警复位接口错误：" + DecodeRobotMessageText(alarmPreparation.commandError); }
            if (!alarmPreparation.blockReason.empty())
            { detail << "准备阻止原因：" + QString::fromUtf8(alarmPreparation.blockReason.c_str()); }
            if (preparedReadAttempted) { detail << "自动准备后状态：" + formatStatus(prepared); }
            detail << QStringLiteral("自动准备：切自动模式=%1，伺服上电=%2，状态回读=%3")
                .arg(outcome(modeAttempted, modeOk))
                .arg(outcome(servoAttempted, servoOk))
                .arg(outcome(preparedReadAttempted, preparedStatusOk));
            if (!modeError.isEmpty()) { detail << "模式错误：" + modeError; }
            if (!servoError.isEmpty()) { detail << "伺服错误：" + servoError; }
            if (!preparedError.isEmpty()) { detail << "状态回读错误：" + preparedError; }
            if (!trackingError.isEmpty()) { detail << "运动登记失败：" + trackingError; }
            if (!motionError.isEmpty() && !motionOk) { detail << "运动错误：" + motionError; }
            if (!motionOk)
            {
                detail << (tracked
                    ? QStringLiteral("安全中止并确认=%1").arg(aborted ? "OK" : "FAIL")
                    : QStringLiteral("安全中止并确认=未执行（尚未发送运动命令）"));
            }
            detail << QStringLiteral("恢复执行前状态：伺服=%1，模式=%2")
                .arg(outcome(servoRestoreAttempted, servoRestoreOk))
                .arg(outcome(modeRestoreAttempted, modeRestoreOk));
            if (!restoreError.isEmpty()) { detail << restoreError; }
            const QString evidence = QStringLiteral(
                "%1：运动登记=%2，Move=%3，CheckRobotDone=%4，终态登记=%5，速度=%6，位移=%7，目标=%8\n%9")
                .arg(action).arg(outcome(setupOk, tracked)).arg(outcome(tracked, moveOk))
                .arg(moveOk ? QString::number(done) : QStringLiteral("未执行"))
                .arg(outcome(moveOk && done == 1, completed))
                .arg(QString::number(speed, 'f', 0) + (jointMotion ? " %" : " mm/min"))
                .arg(QString::number(distance, 'f', 1) + (jointMotion ? " deg" : " mm"))
                .arg(targetText)
                .arg(detail.join('\n'));
            QMetaObject::invokeMethod(qApp, [self, ok, motionOk, evidence, returning, jointMotion, lease]()
                {
                    if (self == nullptr) { return; }
                    if (jointMotion)
                    {
                        self->m_adaptorAcceptanceBusy = false;
                        self->m_bRobotCommandRunning = false;
                        self->m_adaptorJointMovedOut = ok && !returning;
                        self->m_adaptorJointLease = ok && !returning ? lease : nullptr;
                        if (!ok || returning) { self->m_adaptorJointOriginalPulseValid = false; }
                        self->m_adaptorJointMotionState = !ok ? "fail" : (returning ? "pass" : "awaiting_return");
                        self->m_adaptorJointMotionEvidence += "\n" + evidence;
                        if (ok && !returning)
                        { self->m_adaptorJointMotionEvidence += "\n已到关节外移点；必须再次点击关节专项按钮，单独确认返回原位。"; }
                        const bool resultSaved = self->SaveAdaptorAcceptanceRun();
                        if (!resultSaved)
                        {
                            self->m_adaptorJointMotionState = "fail";
                            self->m_adaptorJointMotionEvidence += "\n结果保存失败，不判定通过。";
                            self->m_adaptorJointOriginalPulseValid = false;
                            self->m_adaptorJointMovedOut = false;
                            self->m_adaptorJointLease.reset();
                        }
                        self->RefreshMotionButtonState();
                        self->RefreshAdaptorAcceptanceUi();
                        self->AppendLog("独立关节专项：" + evidence);
                        if (!ok || !resultSaved) { QMessageBox::warning(self, "关节专项", self->m_adaptorJointMotionEvidence); }
                        else { QMessageBox::information(self, "关节专项", evidence + (returning
                            ? "\n关节往返、终态及恢复已验证；独立保存，不覆盖直线验收。"
                            : "\n请再次点击关节专项按钮确认返回。")); }
                        return;
                    }
                    if (motionOk)
                    {
                        self->m_adaptorAcceptanceMovedOut = !returning;
                        if (returning)
                        {
                            self->m_adaptorAcceptanceOriginalPoseValid = false;
                            self->m_adaptorAcceptanceMotionRoundTripCompleted = ok;
                        }
                    }
                    else
                    {
                        self->m_adaptorAcceptanceMovedOut = false;
                        self->m_adaptorAcceptanceOriginalPoseValid = false;
                        self->m_adaptorAcceptanceMotionRoundTripCompleted = false;
                    }
                    self->FinishAdaptorAcceptanceStage(4, ok,
                        ok && !returning
                            ? evidence + "\n已到外移点；必须再次执行本阶段返回原位，当前不能标记通过。"
                            : evidence,
                        ok);
                }, Qt::QueuedConnection);
        }).detach();
}

bool FunctionTestDialog::SaveAdaptorRegisterProgress(const QString& evidence, const QString& recovery)
{
    const QString previousRecovery = m_adaptorRegisterRecovery;
    m_adaptorRegisterRecovery = recovery;
    m_adaptorAcceptanceEvidence[5] = evidence;
    if (m_adaptorAcceptanceSelectedStage == 5)
    {
        const QSignalBlocker blocker(m_pAdaptorAcceptanceEvidence);
        m_pAdaptorAcceptanceEvidence->setPlainText(evidence);
    }
    if (SaveAdaptorAcceptanceRun()) { return true; }
    m_adaptorRegisterRecovery = previousRecovery; // A failed clear must retain the recovery gate.
    return false;
}

bool FunctionTestDialog::ConfirmAdaptorRegisterValue(
    RobotDriverAdaptor* driver, const QString& description)
{
    const QString pending = m_adaptorAcceptanceEvidence.value(5) + "\n" + description
        + "\n等待现场核对；尚未执行确认后的验收回读。";
    if (!SaveAdaptorRegisterProgress(pending, m_adaptorRegisterRecovery)) { return false; }
    QMessageBox box(QMessageBox::Question, "核对测试变量", description
        + "\n\n请在示教器变量监控核对上述测试值，并确认现场无异常。"
          "\n确认后立即自动回读比较，然后恢复原值并验证。"
          "\n不一致、取消、关闭此窗口或5分钟超时：停止测试并尝试恢复原值。"
          "\n请勿强制退出程序、断网或断电。", QMessageBox::NoButton, this);
    auto* confirm = box.addButton("现场数值一致，回读校验", QMessageBox::AcceptRole);
    auto* cancel = box.addButton("不一致/取消并恢复", QMessageBox::RejectRole);
    box.setDefaultButton(cancel);
    box.setEscapeButton(cancel);
    box.setTextFormat(Qt::PlainText);
    bool timedOut = false;
    QTimer timeout;
    timeout.setSingleShot(true);
    connect(&timeout, &QTimer::timeout, &box, [&] { timedOut = true; box.reject(); });
    QTimer cancellation;
    connect(&cancellation, &QTimer::timeout, &box, [&] {
        if (RobotOperationLease::IsCancellationRequested(driver)) { box.reject(); }
    });
    timeout.start(5 * 60 * 1000);
    cancellation.start(100);
    box.exec();
    const bool accepted = box.clickedButton() == confirm && !timedOut
        && !RobotOperationLease::IsCancellationRequested(driver);
    const QString outcome = accepted ? "现场确认数值一致，开始自动回读"
        : timedOut ? "现场核对超时，开始恢复" : "现场不一致/取消，开始恢复";
    const bool saved = SaveAdaptorRegisterProgress(pending + "\n"
        + QDateTime::currentDateTime().toString(Qt::ISODateWithMs) + " " + outcome,
        m_adaptorRegisterRecovery);
    return accepted && saved;
}

bool FunctionTestDialog::CheckAdaptorRegisterRecovery()
{
    // Search all runs, so switching/new runs cannot hide an interrupted write.
    RobotAdaptorAcceptanceStore::History history;
    QString error;
    if (!RobotAdaptorAcceptanceStore::ReadHistory(AdaptorAcceptanceStorageRobotName(), history, &error))
    { QMessageBox::warning(this, "寄存器恢复检查", "读取历史失败，禁止写入：" + error); return false; }
    for (auto it = history.cbegin(); it != history.cend(); ++it)
    {
        const QString recovery = it.value().value("RegisterRecovery");
        if (recovery.isEmpty()) { continue; }
        if (m_adaptorAcceptanceRunId != it.key()) { LoadAdaptorAcceptanceRun(it.key()); }
        if (!m_adaptorAcceptanceRecordLoaded || m_adaptorAcceptanceRunId != it.key()) { return false; }
        m_pAdaptorAcceptanceStageList->setCurrentRow(5);
        const QJsonObject marker = QJsonDocument::fromJson(recovery.toUtf8()).object();
        auto* driver = GetFirstRobotDriverAdaptor();
        const QString kind = marker.value("kind").toString();
        const int index = marker.value("index").toInt(-1);
        bool originalValid = false;
        const QString originalText = marker.value("original").toString();
        const double original = originalText.toDouble(&originalValid);
        const bool real = kind == "REAL";
        if (driver == nullptr || (kind != "INT" && !real) || index < 0 || index > 255
            || !originalValid || !std::isfinite(original)
            || (!real && (original < std::numeric_limits<int>::min()
                || original > std::numeric_limits<int>::max() || std::trunc(original) != original))
            || marker.value("endpoint").toString() != RobotOperationLease::PersistentEndpointIdentity(driver)
            || marker.value("driver").toString() != QString::fromStdString(driver->DriverDescriptor().typeName))
        { QMessageBox::warning(this, "寄存器恢复检查", "原值记录无效或机器人端点/品牌已改变。禁止重写，保留记录供人工排查。\n" + recovery); return false; }
        const QString name = QString::fromStdString(driver->AcceptanceRegisterName(real, index));
        if (QMessageBox::question(this, "先处理未恢复变量",
            QStringLiteral("轮次%1存在未确认恢复的变量%2，原值=%3。\n"
                "请先在示教器确认该变量恢复为原值。点击Yes仅回读核对，不发送写入、不继续新测试。")
                .arg(it.key(), name, originalText), QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel)
            != QMessageBox::Yes) { return false; }
        const auto lease = RobotOperationLease::TryAcquire(driver, "寄存器恢复只读检查", &error);
        if (!lease) { QMessageBox::warning(this, "寄存器恢复检查", error); return false; }
        if (!BeginAdaptorAcceptanceStage(5)) { return false; }
        RefreshAdaptorAcceptanceUi();
        QPointer<FunctionTestDialog> self(this);
        std::thread([self, driver, lease, real, index, original, name, originalText]() {
            driver->ClearLastRobotError();
            double actual = 0;
            int integer = 0;
            const bool read = real ? driver->TryGetRealVar(index, actual, "REAL", 1)
                : driver->TryGetIntVar(index, integer, "INT");
            if (!real) { actual = integer; }
            const bool matches = read && (real ? std::abs(actual - original) <= 1e-6 : actual == original);
            const QString detail = QStringLiteral("\n中断恢复只读核对：%1 原值=%2 回读=%3 一致=%4\n%5")
                .arg(name, originalText, read ? QString::number(actual, 'g', 17) : "读取失败",
                    matches ? "OK" : "FAIL", DecodeRobotMessageText(driver->GetLastRobotError()));
            QMetaObject::invokeMethod(qApp, [self, matches, detail]() {
                if (!self) { return; }
                QString evidence = self->m_adaptorAcceptanceEvidence.value(5) + detail;
                if (matches && self->SaveAdaptorRegisterProgress(evidence, QString()))
                { evidence += "\n已核对原值并解除本条恢复提示，请重新开始第5项；本次没有执行新测试。"; }
                else { evidence += "\n恢复未确认或保存失败，保留原值记录，禁止开始新的写测试。"; }
                self->FinishAdaptorAcceptanceStage(5, false, evidence);
            }, Qt::QueuedConnection);
        }).detach();
        return false;
    }
    return true;
}

void FunctionTestDialog::RunAdaptorInterfaceMatrix()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::PassiveState, QStringLiteral("常用接口矩阵"));
    if (driver == nullptr) { return; }
    if (!CheckAdaptorRegisterRecovery()) { return; }
    const int intIndex = m_pAdaptorAcceptanceIntIndexSpin->value();
    const int realIndex = m_pAdaptorAcceptanceRealIndexSpin->value();
    const QString intName = QString::fromStdString(driver->AcceptanceRegisterName(false, intIndex));
    const QString realName = QString::fromStdString(driver->AcceptanceRegisterName(true, realIndex));
    const QString robotIdentity = AdaptorAcceptanceStorageRobotName() + " / "
        + QString::fromStdString(driver->DriverDescriptor().displayName) + " / "
        + RobotOperationLease::PersistentEndpointIdentity(driver);
    if (QMessageBox::question(this, "接口写读恢复",
        QStringLiteral("机器人：%1\nINT变量：%2；REAL变量：%3（仅测试品牌已声明的类型）。\n"
            "请确认变量专用于测试，未被JOB/PLC/握手逻辑使用，相关任务已停止。\n"
            "逐个备份原值并落盘，写入后等待你现场确认，再自动回读比较、恢复原值并验证。"
            "取消/超时将尝试恢复，任何失败都停止后续变量。是否开始？")
            .arg(robotIdentity, intName, realName), QMessageBox::Yes | QMessageBox::Cancel,
            QMessageBox::Cancel) != QMessageBox::Yes)
    {
        return;
    }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("适配验收常用接口矩阵"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "常用接口矩阵", leaseError);
        return;
    }
    if (!BeginAdaptorAcceptanceStage(5)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, driver, lease, intIndex, realIndex, intName, realName, robotIdentity]()
        {
            QStringList evidence;
            QString recovery;
            std::string interactionError;
            const auto saveProgress = [&](const QString& text, const QString& marker) {
                bool saved = false;
                QMetaObject::invokeMethod(qApp, [&] {
                    if (self) { saved = self->SaveAdaptorRegisterProgress(text, marker); }
                }, Qt::BlockingQueuedConnection);
                return saved;
            };
            const auto backup = [&](const QString& kind, int index, const QString& name,
                const QString& original, const QString& temporary) {
                interactionError.clear();
                if (RobotOperationLease::IsCancellationRequested(driver))
                { interactionError = "已取消，未写入测试变量。"; return false; }
                QJsonObject marker{{"kind", kind}, {"index", index}, {"original", original},
                    {"temporary", temporary}, {"name", name}, {"robot", robotIdentity},
                    {"endpoint", RobotOperationLease::PersistentEndpointIdentity(driver)},
                    {"driver", QString::fromStdString(driver->DriverDescriptor().typeName)}};
                recovery = QString::fromUtf8(QJsonDocument(marker).toJson(QJsonDocument::Compact));
                evidence << QStringLiteral("%1 原值=%2 测试值=%3；写入前备份，恢复尚未确认。")
                    .arg(name, original, temporary);
                if (saveProgress(evidence.join('\n'), recovery)) { return true; }
                interactionError = "原值数据库保存失败，未写入测试值。";
                return false;
            };
            const auto confirm = [&](const QString& name, const QString& original, const QString& temporary) {
                bool accepted = false;
                QMetaObject::invokeMethod(qApp, [&] {
                    if (self && !RobotOperationLease::IsCancellationRequested(driver))
                    { accepted = self->ConfirmAdaptorRegisterValue(driver,
                        QStringLiteral("机器人：%1\n变量：%2\n原值：%3\n示教器现在应显示的测试值：%4")
                            .arg(robotIdentity, name, original, temporary)); }
                }, Qt::BlockingQueuedConnection);
                evidence << QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
                    + " " + name + (accepted ? " 现场已确认数值一致，随后自动回读。" : " 现场不一致/取消/超时或记录失败，停止测试并恢复。");
                if (!accepted) { interactionError = "现场未确认或记录失败，不能通过验收。"; }
                return accepted;
            };
            const auto errorReader = [&] { return interactionError.empty() ? driver->GetLastRobotError() : interactionError; };
            evidence << "机器人：" + robotIdentity;
            QStringList capabilities;
            for (unsigned int bitIndex = 0; bitIndex <= RobotDriverCapabilityMaxBitIndex; ++bitIndex)
            {
                const auto capability = static_cast<RobotDriverCapability>(1ULL << bitIndex);
                if (driver->Supports(capability))
                {
                    capabilities.push_back(QString::fromUtf8(
                        RobotDriverAdaptor::CapabilityDisplayName(capability)));
                }
            }
            evidence << "已声明能力：" + capabilities.join("、");
            const RobotMotionStatus motion = driver->ReadMotionStatus();
            bool ok = (motion.state == RobotMotionState::Idle || motion.state == RobotMotionState::Completed)
                && motion.terminalVerified;
            if (!ok) { evidence << "未确认空闲/完成终态，禁止写入测试变量。"; }
            if (!ok) { evidence << "完成状态读取失败：" + DecodeRobotMessageText(driver->GetLastRobotError()); }
            evidence << QStringLiteral("完成状态接口：state=%1 raw=%2 terminalVerified=%3")
                .arg(static_cast<int>(motion.state)).arg(motion.rawCode)
                .arg(motion.terminalVerified);
            if (driver->Supports(RobotDriverCapability::StructuredControllerStatus))
            {
                const RobotControllerStatus status = driver->ReadControllerStatus();
                evidence << QStringLiteral("结构化状态接口：valid=%1，detail=%2")
                    .arg(status.valid).arg(QString::fromStdString(status.detail));
                if (!status.valid)
                { evidence << "控制器状态读取失败：" + DecodeRobotMessageText(driver->GetLastRobotError()); }
                ok = ok && status.valid;
            }
            if (driver->Supports(RobotDriverCapability::IntegerRegister))
            {
                if (!ok) { evidence << "INT寄存器：前序状态检查失败，未执行。"; }
                else
                {
                    const auto result = RobotRegisterRoundTrip::Run<int>(
                        [&, driver, intIndex](int& value) {
                            interactionError.clear();
                            driver->ClearLastRobotError();
                            return driver->TryGetIntVar(intIndex, value, "INT"); },
                        [&, driver, intIndex](int value) {
                            interactionError.clear();
                            driver->ClearLastRobotError();
                            return driver->SetIntVar(intIndex, value, PROGRAMVAR, "INT"); },
                        errorReader,
                        [](int original) { return original == std::numeric_limits<int>::max()
                            ? original - 1 : original + 1; },
                        [](int actual, int expected) { return actual == expected; },
                        [&](int original, int temporary) { return backup("INT", intIndex, intName,
                            QString::number(original), QString::number(temporary)); },
                        [&](int original, int temporary) { return confirm(intName,
                            QString::number(original), QString::number(temporary)); });
                    evidence << FormatRegisterRoundTripEvidence(intName,
                        result, [](int value) { return QString::number(value); });
                    ok = result.Passed();
                    if (result.restore.ok && result.restoreRead.ok && result.originalMatches) { recovery.clear(); }
                    if (!saveProgress(evidence.join('\n'), recovery)) { ok = false; evidence << "INT结果保存失败，停止后续测试。"; }
                }
            }
            else
            {
                evidence << "INT寄存器：品牌未声明，不执行写操作。";
            }
            if (driver->Supports(RobotDriverCapability::RealRegister))
            {
                if (!ok) { evidence << "REAL寄存器：前序检查或INT测试失败，未执行。"; }
                else
                {
                    const auto result = RobotRegisterRoundTrip::Run<double>(
                        [&, driver, realIndex](double& value) {
                            interactionError.clear();
                            driver->ClearLastRobotError();
                            return driver->TryGetRealVar(realIndex, value, "REAL", 1); },
                        [&, driver, realIndex](double value) {
                            interactionError.clear();
                            driver->ClearLastRobotError();
                            return driver->SetRealVar(realIndex, value, "REAL", 1); },
                        errorReader,
                        [](double original) { return original > 9999999.0
                            ? original - 0.125 : original + 0.125; },
                        [](double actual, double expected) { return std::abs(actual - expected) <= 1e-6; },
                        [&](double original, double temporary) { return backup("REAL", realIndex, realName,
                            QString::number(original, 'g', 17), QString::number(temporary, 'g', 17)); },
                        [&](double original, double temporary) { return confirm(realName,
                            QString::number(original, 'g', 17), QString::number(temporary, 'g', 17)); });
                    evidence << FormatRegisterRoundTripEvidence(realName,
                        result, [](double value) { return QString::number(value, 'g', 17); });
                    ok = result.Passed();
                    if (result.restore.ok && result.restoreRead.ok && result.originalMatches) { recovery.clear(); }
                    if (!saveProgress(evidence.join('\n'), recovery)) { ok = false; evidence << "REAL结果保存失败。"; }
                }
            }
            else
            {
                evidence << "REAL寄存器：品牌未声明，不执行写操作。";
            }
            evidence << "模式切换、报警复位、伺服上下电、示教速度属于会改变控制器状态的独立能力；"
                "本阶段只列出能力，不自动连续触发，请使用单项测试逐项验证并把结果补充到证据。";
            if (!ok)
            {
                evidence << "本阶段失败，后续寄存器测试已停止；各步错误已分别保留，请核对恢复结果后重试。";
            }
            const QString result = evidence.join('\n');
            QMetaObject::invokeMethod(qApp, [self, ok, result]()
                {
                    if (self != nullptr)
                    {
                        self->FinishAdaptorAcceptanceStage(5, ok, result, false);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::RunAdaptorAssetAudit()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::PassiveState, QStringLiteral("控制器与标定资产检查"));
    if (driver == nullptr) { return; }
    const int toolNo = m_pAdaptorAcceptanceToolIndexSpin->value();
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("适配验收资产检查"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "控制器与标定资产检查", leaseError);
        return;
    }
    if (!BeginAdaptorAcceptanceStage(6)) { return; }
    RefreshMotionButtonState();
    RefreshAdaptorAcceptanceUi();
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, driver, lease, toolNo]()
        {
            QStringList evidence;
            bool ok = true;
            if (driver->Supports(RobotDriverCapability::ControllerKinematicsRead))
            {
                RobotKinematicsValidationResult validation;
                const bool validationOk = driver->RefreshKinematicsFromController(validation);
                evidence << QStringLiteral(
                    "控制器运动学资产=%1，型号=%2\n"
                    "获取方式=%3\n"
                    "当前关节角=[%4,%5,%6,%7,%8,%9] deg\n"
                    "控制器基座法兰=X%10 Y%11 Z%12 RX%13 RY%14 RZ%15\n"
                    "计算基座法兰=X%16 Y%17 Z%18 RX%19 RY%20 RZ%21\n"
                    "关节/直角闭环误差：位置=%22 mm，姿态=%23 deg%24")
                    .arg(validationOk ? "OK" : "FAIL")
                    .arg(QString::fromStdString(validation.modelName))
                    .arg(QString::fromStdString(validation.acquisitionSummary))
                    .arg(validation.currentJointDegrees[0], 0, 'f', 6)
                    .arg(validation.currentJointDegrees[1], 0, 'f', 6)
                    .arg(validation.currentJointDegrees[2], 0, 'f', 6)
                    .arg(validation.currentJointDegrees[3], 0, 'f', 6)
                    .arg(validation.currentJointDegrees[4], 0, 'f', 6)
                    .arg(validation.currentJointDegrees[5], 0, 'f', 6)
                    .arg(validation.controllerFlangeInBase.dX, 0, 'f', 4)
                    .arg(validation.controllerFlangeInBase.dY, 0, 'f', 4)
                    .arg(validation.controllerFlangeInBase.dZ, 0, 'f', 4)
                    .arg(validation.controllerFlangeInBase.dRX, 0, 'f', 4)
                    .arg(validation.controllerFlangeInBase.dRY, 0, 'f', 4)
                    .arg(validation.controllerFlangeInBase.dRZ, 0, 'f', 4)
                    .arg(validation.calculatedFlangeInBase.dX, 0, 'f', 4)
                    .arg(validation.calculatedFlangeInBase.dY, 0, 'f', 4)
                    .arg(validation.calculatedFlangeInBase.dZ, 0, 'f', 4)
                    .arg(validation.calculatedFlangeInBase.dRX, 0, 'f', 4)
                    .arg(validation.calculatedFlangeInBase.dRY, 0, 'f', 4)
                    .arg(validation.calculatedFlangeInBase.dRZ, 0, 'f', 4)
                    .arg(validation.positionErrorMm, 0, 'f', 4)
                    .arg(validation.orientationErrorDeg, 0, 'f', 4)
                    .arg(validationOk ? QString() : QStringLiteral("，错误=")
                        + DecodeRobotMessageText(driver->GetLastRobotError()));
                ok = ok && validationOk;
            }
            else
            {
                evidence << "控制器运动学资产：品牌未声明ControllerKinematicsRead，继续检查本地已验证模型。";
            }
            const T_KINEMATICS& kinematics = driver->KinematicsParameters();
            const std::array<double, kDhParamCount> dh = KinematicsToParamArray(kinematics);
            bool dhFinite = true;
            bool dhNonZero = false;
            for (const double value : dh)
            {
                dhFinite = dhFinite && std::isfinite(value);
                dhNonZero = dhNonZero || std::abs(value) > 1e-12;
            }
            const T_AXISUNIT& axisUnit = driver->AxisUnit();
            const double mainUnits[6] = {
                axisUnit.dSPulseUnit, axisUnit.dLPulseUnit, axisUnit.dUPulseUnit,
                axisUnit.dRPulseUnit, axisUnit.dBPulseUnit, axisUnit.dTPulseUnit
            };
            const bool axisReady = std::all_of(std::begin(mainUnits), std::end(mainUnits),
                [](double value) { return std::isfinite(value) && std::abs(value) > 1e-15; });
            const T_AXISLIMITANGLE& limits = driver->AxisLimitAngles();
            bool limitsReady = true;
            for (int axis = 0; axis < 6; ++axis)
            {
                limitsReady = limitsReady
                    && std::isfinite(limits.GetMinAngleByIndex(axis))
                    && std::isfinite(limits.GetMaxAngleByIndex(axis))
                    && limits.GetMinAngleByIndex(axis) < limits.GetMaxAngleByIndex(axis);
            }
            evidence << QStringLiteral("本地数据库/运行时运动学：DH有限=%1，DH非全零=%2，主轴AxisUnit=%3，主轴限位=%4")
                .arg(dhFinite).arg(dhNonZero).arg(axisReady).arg(limitsReady);
            ok = ok && dhFinite && dhNonZero && axisReady && limitsReady;

            if (driver->Supports(RobotDriverCapability::ToolDataRead))
            {
                T_ROBOT_COORS tool;
                const bool toolOk = driver->GetToolData(toolNo, tool);
                evidence << (toolOk
                    ? QStringLiteral("控制器Tool%1读取=OK；%2")
                        .arg(toolNo).arg(FormatAcceptancePose(tool))
                    : QStringLiteral("控制器Tool%1读取=FAIL；%2")
                        .arg(toolNo).arg(DecodeRobotMessageText(driver->GetLastRobotError())));
                ok = ok && toolOk;
            }
            else
            {
                evidence << "控制器工具读取：品牌未声明ToolDataRead；需在示教器导出工具数据或补充品牌底层读取协议。";
            }

            if (driver->Supports(RobotDriverCapability::HandEyeMatrixRead))
            {
                double rotation[9] = {};
                double translation[3] = {};
                std::string controllerError;
                const bool controllerMatrixOk = driver->GetHandEyeMatrixVariable(
                    "eye", rotation, translation, &controllerError);
                evidence << QStringLiteral("控制器手眼变量eye读取=%1，T=(%2,%3,%4)%5")
                    .arg(controllerMatrixOk ? "OK" : "FAIL")
                    .arg(translation[0], 0, 'f', 6).arg(translation[1], 0, 'f', 6)
                    .arg(translation[2], 0, 'f', 6)
                    .arg(controllerError.empty() ? QString()
                        : "，错误=" + QString::fromStdString(controllerError));
                ok = ok && controllerMatrixOk;
            }
            else
            {
                evidence << "控制器手眼矩阵接口：品牌未声明HandEyeMatrixRead，本接口阶段不据此判失败；"
                    "本地手眼矩阵及二转三精度统一在阶段7验证。";
            }
            if (driver->Supports(RobotDriverCapability::CalibrationAssetDiscovery))
            {
                RobotCalibrationDiscovery discovery;
                std::atomic_bool cancel{ false };
                std::string discoveryError;
                const bool discoveryOk = driver->DiscoverCalibrationAssets(
                    discovery, cancel, discoveryError);
                evidence << QStringLiteral("品牌固定资产发现=%1；规则=%2；设备=%3；资产数=%4")
                    .arg(discoveryOk ? "OK" : "FAIL")
                    .arg(QString::fromStdString(discovery.recipeRevision))
                    .arg(QString::fromStdString(discovery.identity))
                    .arg(discovery.assets.size());
                for (const auto& asset : discovery.assets)
                {
                    if (asset.kind == "handeye" || asset.kind == "kinematics"
                        || asset.kind == "tool-calibration" || asset.kind == "workobject-calibration")
                    {
                        evidence << QStringLiteral("  %1 [%2] %3；%4")
                            .arg(QString::fromStdString(asset.name),
                                QString::fromStdString(asset.status),
                                QString::fromStdString(asset.source),
                                QString::fromStdString(asset.detail));
                    }
                }
                if (!discoveryError.empty())
                {
                    evidence << QStringLiteral("资产发现错误=%1")
                        .arg(QString::fromStdString(discoveryError));
                }
                ok = ok && discoveryOk;
            }
            else
            {
                evidence << "品牌固定资产发现：未声明；仅保留通用适配接口检查，不猜测厂商路径。";
            }
            if (driver->Supports(RobotDriverCapability::ControllerKinematicsRead))
            {
                evidence << "控制器机械数据已由品牌底层按固定来源获取并交叉校验；业务层只使用适配层返回的"
                    "运动学模型、轴单位、限位及关节/直角闭环结论，不读取品牌命令或FTP字段。";
            }
            else
            {
                evidence << "控制器运动学变量：当前品牌未提供自动资产探针。请在示教器确认机器人型号、"
                    "轴单位和零位/机械参数；获得变量名、文件路径和单位后再补充品牌底层。";
            }

            std::string ftpError;
            const auto session = driver->CreateFileTransferSession(&ftpError);
            if (session)
            {
                RobotProgramInventoryResult inventory;
                const bool inventoryOk = session->QueryProgramInventory(inventory, 10000);
                evidence << QStringLiteral("控制器程序资产清单（有限递归）=%1，根目录=%2，条目=%3，程序=%4")
                    .arg(inventoryOk ? "OK" : "FAIL")
                    .arg(QString::fromStdString(inventory.remoteDirectory))
                    .arg(inventory.entryCount).arg(inventory.programCount);
                ok = ok && inventoryOk;
            }
            else
            {
                evidence << QStringLiteral("无法读取控制器程序资产清单：%1")
                    .arg(QString::fromStdString(ftpError));
                ok = false;
            }
            if (!ok)
            {
                evidence << "缺失处理：优先从示教器导出Tool/运动学/变量和工程清单；"
                    "禁止用其它品牌或其它机器人参数代替。获得现场变量名、文件路径和单位后再补充品牌底层。";
            }
            const QString result = evidence.join('\n');
            QMetaObject::invokeMethod(qApp, [self, ok, result]()
                {
                    if (self != nullptr)
                    {
                        self->FinishAdaptorAcceptanceStage(6, ok, result, ok);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::RunAdaptorTwoToThreeCheck()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::PassiveState, QStringLiteral("二转三/手眼坐标转换"));
    if (driver == nullptr) { return; }
    const double toleranceMm = m_pAdaptorAcceptanceTwoToThreeToleranceSpin->value();
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("适配验收二转三检查"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, "二转三/手眼坐标转换", leaseError);
        return;
    }
    if (m_pCameraCache == nullptr)
    {
        FinishAdaptorAcceptanceStage(7, false,
            "当前机器人没有相机帧缓存，无法执行二转三检查。请先连接测量相机。" );
        return;
    }
    udpDataShow frame;
    if (!m_pCameraCache->Latest(frame))
    {
        FinishAdaptorAcceptanceStage(7, false,
            "尚未取得相机帧。请先打开相机并让激光落在可识别目标上。" );
        return;
    }
    cv::Point3d cameraPoint = frame.targetPoint;
    if ((!std::isfinite(cameraPoint.x) || !std::isfinite(cameraPoint.y)
        || !std::isfinite(cameraPoint.z)) && !frame.allResultPoint.empty())
    {
        cameraPoint = frame.allResultPoint.at(frame.allResultPoint.size() / 2);
    }
    if (!std::isfinite(cameraPoint.x) || !std::isfinite(cameraPoint.y)
        || !std::isfinite(cameraPoint.z))
    {
        FinishAdaptorAcceptanceStage(7, false, "相机帧没有有限的三维目标点。" );
        return;
    }
    T_ROBOT_COORS robotPose;
    if (!driver->TryGetCurrentPos(robotPose))
    {
        FinishAdaptorAcceptanceStage(7, false,
            "读取机器人位姿失败：" + DecodeRobotMessageText(driver->GetLastRobotError()));
        return;
    }
    const QString robotName = AdaptorAcceptanceStorageRobotName();
    const QString cameraSection = RobotDataHelper::MeasureCameraSection(robotName);
    HandEyeMatrixConfig matrix;
    QString error;
    QString label;
    if (!LoadExistingValidatedHandEyeMatrixConfig(
        robotName, cameraSection, matrix, &error, &label))
    {
        FinishAdaptorAcceptanceStage(7, false,
            QStringLiteral("加载已验证手眼矩阵失败：%1；%2").arg(error, label));
        return;
    }
    if (!ValidateControllerBoundHandEyeMatrix(
        robotName, cameraSection, matrix, driver, &error))
    {
        FinishAdaptorAcceptanceStage(7, false,
            QStringLiteral("控制器导入手眼矩阵绑定复核失败：%1").arg(error));
        return;
    }
    const Eigen::Vector3d camera(cameraPoint.x, cameraPoint.y, cameraPoint.z);
    const Eigen::Vector3d robotPoint = RobotCalculation::CalcLaserPointInRobot(
        robotPose, camera, matrix);
    const bool ok = robotPoint.allFinite();
    const QString evidence = QStringLiteral(
        "相机点=(%1,%2,%3)，机器人位姿=%4，手眼矩阵=%5，转换后的机器人基坐标点=(%6,%7,%8)。\n"
        "请用已知标定块/已知TCP点核对实际误差并在证据中填写毫米误差；本轮允许误差=%9 mm，"
        "仅数值有限不能代表精度通过。")
        .arg(camera.x(), 0, 'f', 6).arg(camera.y(), 0, 'f', 6).arg(camera.z(), 0, 'f', 6)
        .arg(FormatAcceptancePose(robotPose), label)
        .arg(robotPoint.x(), 0, 'f', 6).arg(robotPoint.y(), 0, 'f', 6)
        .arg(robotPoint.z(), 0, 'f', 6)
        .arg(toleranceMm, 0, 'f', 2);
    FinishAdaptorAcceptanceStage(7, ok, evidence, ok);
}

void FunctionTestDialog::OpenAdaptorWorkflow(const QString& workflowId)
{
    const int stage = workflowId == "measureThenWeldActual" ? 9 : 8;
    const QString modeText = stage == 9 ? QStringLiteral("实际焊接") : QStringLiteral("仅扫描/空跑");
    QString warning = stage == 9
        ? QStringLiteral("将打开现有先测后焊界面。必须确认实际焊接能力、焊机参数、气体/送丝、起收弧、"
            "掉弧处理和干跑轨迹均已通过；本验收页不会自动点击开始。")
        : QStringLiteral("将打开现有先测后焊界面。请取消“实际焊接”，先执行扫描/空跑，检查时间对齐、"
            "点云完整性、二转三、特征处理和轨迹生成；本验收页不会自动开始运动。");
    if (QMessageBox::question(this, modeText, warning + "\n\n是否打开现有流程？")
        != QMessageBox::Yes)
    {
        return;
    }
    if (!m_workflowLauncher)
    {
        FinishAdaptorAcceptanceStage(stage, false,
            "功能测试页没有配置现有流程启动回调。" );
        return;
    }
    const int selectedUnitNo = (m_pContralUnit != nullptr && m_unitIndex >= 0
        && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
        ? m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].nUnitNo : -1;
    if (!m_workflowLauncher(workflowId, selectedUnitNo))
    {
        FinishAdaptorAcceptanceStage(stage, false,
            QStringLiteral("无法为当前所选机器人打开现有先测后焊流程：%1。")
                .arg(AdaptorAcceptanceStorageRobotName()));
        return;
    }
    const QString evidence = QStringLiteral("已打开现有先测后焊流程，要求模式=%1。"
        "完成后返回本页，填写结果目录、点云/轨迹/焊缝观察和异常日志，再人工确认通过。")
        .arg(modeText);
    m_adaptorAcceptanceStates[stage] = "pending";
    m_adaptorAcceptanceEvidence[stage] = evidence;
    m_pAdaptorAcceptanceEvidence->setPlainText(evidence);
    SaveAdaptorAcceptanceRun();
    RefreshAdaptorAcceptanceUi();
}

void FunctionTestDialog::FinalizeAdaptorAcceptance()
{
    if (!m_adaptorAcceptanceRecordLoaded || !SaveAdaptorAcceptanceRun())
    {
        QMessageBox::warning(this, "机器人适配测试报告", "验收记录未能保存，无法导出报告，请检查数据库错误提示。");
        return;
    }
    if (m_adaptorAcceptanceBusy)
    {
        QMessageBox::warning(this, "机器人适配测试报告",
            "当前测试阶段仍在运行，等待完成或安全中止后再导出报告。");
        return;
    }
    if (m_pAdaptorAcceptanceStageList != nullptr && m_pAdaptorAcceptanceEvidence != nullptr)
    {
        const int selectedStage = m_pAdaptorAcceptanceStageList->currentRow();
        if (selectedStage >= 0 && selectedStage < m_adaptorAcceptanceEvidence.size())
        {
            m_adaptorAcceptanceEvidence[selectedStage] =
                m_pAdaptorAcceptanceEvidence->toPlainText().trimmed();
        }
    }

    RobotDriverAdaptor* driver = GetFirstRobotDriverAdaptor();
    if (driver == nullptr) { return; }
    const QString robotName = AdaptorAcceptanceStorageRobotName();
    const RobotDriverDescriptor descriptor = driver->DriverDescriptor();
    const RobotConnectionEndpoint endpoint = driver->ControlEndpoint();
    const QString generatedAt = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    QStringList effectiveStates;
    QStringList recommendations;
    bool allPassed = true;
    bool hasFailure = false;
    bool hasRestriction = false;
    for (int stage = 0; stage < 10; ++stage)
    {
        QString state = m_adaptorAcceptanceStates.value(stage, "pending");
        const std::uint64_t required = AdaptorAcceptanceRequiredMask(stage);
        if (required != 0 && !driver->SupportsMask(required))
        {
            state = "restricted";
        }
        effectiveStates.push_back(state);
        allPassed = allPassed && state == "pass";
        hasFailure = hasFailure || state == "fail";
        hasRestriction = hasRestriction || state == "restricted";
        if (state != "pass")
        {
            QString recommendation = QStringLiteral("阶段%1（%2）：%3")
                .arg(stage)
                .arg(QString::fromUtf8(kAdaptorAcceptanceStageNames[stage]))
                .arg(AdaptorAcceptanceStateText(state));
            if (state == "restricted")
            {
                recommendation += QStringLiteral("；缺少能力：%1")
                    .arg(QString::fromUtf8(driver->MissingCapabilitiesText(required).c_str()));
            }
            const QString evidence = m_adaptorAcceptanceEvidence.value(stage).simplified();
            if (!evidence.isEmpty())
            {
                recommendation += QStringLiteral("；证据：%1").arg(evidence.left(300));
            }
            recommendations.push_back(recommendation);
        }
    }

    const QString overallStatus = allPassed ? QStringLiteral("pass")
        : (hasFailure ? QStringLiteral("fail")
            : (hasRestriction ? QStringLiteral("restricted") : QStringLiteral("incomplete")));
    const QString overallText = allPassed ? QStringLiteral("通过")
        : (hasFailure ? QStringLiteral("失败")
            : (hasRestriction ? QStringLiteral("能力受限") : QStringLiteral("未完成")));

    const QString reportDirectory = AppPaths::WritablePath(
        QStringLiteral("Result/RobotAdaptorAcceptance/%1").arg(m_adaptorAcceptanceRunId));
    if (!QDir().mkpath(reportDirectory))
    {
        QMessageBox::warning(this, "机器人适配测试报告",
            "无法创建报告目录：\n" + reportDirectory);
        return;
    }
    const QString reportBaseName = QStringLiteral("robot_adaptor_acceptance_%1")
        .arg(m_adaptorAcceptanceRunId);
    const QString markdownPath = QDir(reportDirectory).filePath(reportBaseName + ".md");
    const QString jsonPath = QDir(reportDirectory).filePath(reportBaseName + ".json");

    QStringList capabilityNames;
    QJsonArray capabilityJson;
    for (unsigned int bitIndex = 0; bitIndex <= RobotDriverCapabilityMaxBitIndex; ++bitIndex)
    {
        const auto capability = static_cast<RobotDriverCapability>(1ULL << bitIndex);
        if (driver->Supports(capability))
        {
            const QString name = QString::fromUtf8(
                RobotDriverAdaptor::CapabilityDisplayName(capability));
            capabilityNames.push_back(name);
            capabilityJson.append(name);
        }
    }

    QString markdown;
    QTextStream markdownStream(&markdown);
    markdownStream << "# 机器人适配测试报告\n\n"
        << "- 报告格式：RobotAdaptorAcceptanceReportV1\n"
        << "- 验收流程版本：" << QString::fromUtf8(RobotAdaptorAcceptancePlan::Revision) << "\n"
        << "- 原始轮次证据版本：" << m_adaptorEvidencePlanRevision << "（新增专项单独记录）\n"
        << "- 运行编号：" << m_adaptorAcceptanceRunId << "\n"
        << "- 生成时间：" << generatedAt << "\n"
        << "- 机器人：" << robotName << "\n"
        << "- 驱动：" << QString::fromStdString(descriptor.displayName)
        << "（" << QString::fromStdString(descriptor.typeName) << " / " << descriptor.typeCode << "）\n"
        << "- 控制端点：" << QString::fromStdString(endpoint.host) << ":" << endpoint.port << "\n"
        << "- 能力掩码：" << QString::number(driver->DriverCapabilities()) << "\n"
        << "- 总体结论：" << overallText << "（" << overallStatus << "）\n\n"
        << "## 测试参数\n\n"
        << "- 原生程序测试速度：" << m_pAdaptorAcceptanceProgramSpeedSpin->value() << " mm/min\n"
        << "- 低速位移：" << m_pAdaptorAcceptanceDistanceSpin->value() << " mm（基坐标+Y）\n"
        << "- 线速度：" << m_pAdaptorAcceptanceSpeedSpin->value() << " mm/min\n"
        << "- INT测试索引：" << m_pAdaptorAcceptanceIntIndexSpin->value() << "\n"
        << "- REAL测试索引：" << m_pAdaptorAcceptanceRealIndexSpin->value() << "\n"
        << "- 工具号：" << m_pAdaptorAcceptanceToolIndexSpin->value() << "\n"
        << "- 二转三允许实测误差：" << m_pAdaptorAcceptanceTwoToThreeToleranceSpin->value() << " mm\n\n"
        << "## 驱动已声明能力\n\n"
        << (capabilityNames.isEmpty() ? QStringLiteral("无") : capabilityNames.join("、")) << "\n\n"
        << "## 分阶段结果\n";

    QJsonArray stagesJson;
    for (int stage = 0; stage < 10; ++stage)
    {
        const std::uint64_t required = AdaptorAcceptanceRequiredMask(stage);
        QStringList requiredNames;
        QJsonArray requiredJson;
        for (unsigned int bitIndex = 0; bitIndex <= RobotDriverCapabilityMaxBitIndex; ++bitIndex)
        {
            const auto capability = static_cast<RobotDriverCapability>(1ULL << bitIndex);
            if ((required & RobotDriverCapabilityBit(capability)) != 0)
            {
                const QString name = QString::fromUtf8(
                    RobotDriverAdaptor::CapabilityDisplayName(capability));
                requiredNames.push_back(name);
                requiredJson.append(name);
            }
        }
        const QString evidence = m_adaptorAcceptanceEvidence.value(stage).trimmed();
        markdownStream << "\n### " << QString::fromUtf8(kAdaptorAcceptanceStageNames[stage]) << "\n\n"
            << "- 状态：" << AdaptorAcceptanceStateText(effectiveStates[stage])
            << "（" << effectiveStates[stage] << "）\n"
            << "- 能力门禁：" << (requiredNames.isEmpty()
                ? QStringLiteral("人工/本地数据门禁") : requiredNames.join("、")) << "\n"
            << "- 证据：\n";
        if (evidence.isEmpty())
        {
            markdownStream << "> 未提供\n";
        }
        else
        {
            for (const QString& line : evidence.split('\n'))
            {
                markdownStream << "> " << line << "\n";
            }
        }

        QJsonObject stageJson;
        stageJson["index"] = stage;
        stageJson["name"] = QString::fromUtf8(kAdaptorAcceptanceStageNames[stage]);
        stageJson["state"] = effectiveStates[stage];
        stageJson["stateText"] = AdaptorAcceptanceStateText(effectiveStates[stage]);
        stageJson["requiredCapabilityMask"] = QString::number(required);
        stageJson["requiredCapabilities"] = requiredJson;
        stageJson["evidence"] = evidence;
        stagesJson.append(stageJson);
    }
    markdownStream << "\n## 独立关节运动专项\n\n"
        << "- 状态：" << m_adaptorJointMotionState << "\n"
        << "- 固定参数：J1 +0.5°，1%速度，两段分别人工确认。\n"
        << "- 本项不继承直线阶段通过，也不无条件限制不需要关节运动的流程。\n\n"
        << m_adaptorJointMotionEvidence << "\n";
    markdownStream << "\n## 无位移模式组合测试\n\n" << m_adaptorModeCombinationEvidence << "\n";
    markdownStream << "\n## 后续处理建议\n\n";
    if (recommendations.isEmpty())
    {
        markdownStream << "主流程阶段均已通过；关节运动等独立专项以各自结论为准，不代表所有机器人能力均通过。\n";
    }
    else
    {
        for (const QString& recommendation : recommendations)
        {
            markdownStream << "- " << recommendation << "\n";
        }
    }
    markdownStream.flush();

    QJsonObject parametersJson;
    parametersJson["programSpeedMmPerMin"] = m_pAdaptorAcceptanceProgramSpeedSpin->value();
    parametersJson["linearDistanceMm"] = m_pAdaptorAcceptanceDistanceSpin->value();
    parametersJson["linearSpeedMmPerMin"] = m_pAdaptorAcceptanceSpeedSpin->value();
    parametersJson["integerRegisterIndex"] = m_pAdaptorAcceptanceIntIndexSpin->value();
    parametersJson["realRegisterIndex"] = m_pAdaptorAcceptanceRealIndexSpin->value();
    parametersJson["toolIndex"] = m_pAdaptorAcceptanceToolIndexSpin->value();
    parametersJson["twoToThreeToleranceMm"] =
        m_pAdaptorAcceptanceTwoToThreeToleranceSpin->value();
    QJsonObject endpointJson;
    endpointJson["host"] = QString::fromStdString(endpoint.host);
    endpointJson["port"] = endpoint.port;
    QJsonObject driverJson;
    driverJson["displayName"] = QString::fromStdString(descriptor.displayName);
    driverJson["typeName"] = QString::fromStdString(descriptor.typeName);
    driverJson["typeCode"] = descriptor.typeCode;
    driverJson["capabilityMask"] = QString::number(driver->DriverCapabilities());
    driverJson["capabilities"] = capabilityJson;
    QJsonArray recommendationsJson;
    for (const QString& recommendation : recommendations)
    {
        recommendationsJson.append(recommendation);
    }
    QJsonObject reportJson;
    reportJson["schema"] = "RobotAdaptorAcceptanceReportV1";
    reportJson["planRevision"] = QString::fromUtf8(RobotAdaptorAcceptancePlan::Revision);
    reportJson["evidencePlanRevision"] = m_adaptorEvidencePlanRevision;
    reportJson["runId"] = m_adaptorAcceptanceRunId;
    reportJson["generatedAt"] = generatedAt;
    reportJson["robotName"] = robotName;
    reportJson["overallStatus"] = overallStatus;
    reportJson["overallText"] = overallText;
    reportJson["driver"] = driverJson;
    reportJson["controlEndpoint"] = endpointJson;
    reportJson["parameters"] = parametersJson;
    reportJson["modeCombinationEvidence"] = m_adaptorModeCombinationEvidence;
    reportJson["jointMotionState"] = m_adaptorJointMotionState;
    reportJson["jointMotionEvidence"] = m_adaptorJointMotionEvidence;
    reportJson["jointMotionDeltaDegrees"] = RobotAcceptanceJointMotion::DeltaDegrees;
    reportJson["jointMotionSpeedPercent"] = RobotAcceptanceJointMotion::SpeedPercent;
    reportJson["stages"] = stagesJson;
    reportJson["recommendations"] = recommendationsJson;

    QSaveFile markdownFile(markdownPath);
    bool markdownOk = markdownFile.open(QIODevice::WriteOnly);
    if (markdownOk)
    {
        markdownOk = markdownFile.write(markdown.toUtf8()) == markdown.toUtf8().size()
            && markdownFile.commit();
    }
    const QString markdownError = markdownOk ? QString() : markdownFile.errorString();
    const QByteArray jsonBytes = QJsonDocument(reportJson).toJson(QJsonDocument::Indented);
    QSaveFile jsonFile(jsonPath);
    bool jsonOk = jsonFile.open(QIODevice::WriteOnly);
    if (jsonOk)
    {
        jsonOk = jsonFile.write(jsonBytes) == jsonBytes.size() && jsonFile.commit();
    }
    const QString jsonError = jsonOk ? QString() : jsonFile.errorString();
    const bool reportOk = markdownOk && jsonOk;

    m_adaptorAcceptanceStates[10] = reportOk
        ? (allPassed ? QStringLiteral("pass")
            : (hasFailure ? QStringLiteral("fail")
                : (hasRestriction ? QStringLiteral("restricted") : QStringLiteral("pending"))))
        : QStringLiteral("fail");
    m_adaptorAcceptanceEvidence[10] = QStringLiteral(
        "总体结论=%1（%2）\nMarkdown=%3\nJSON=%4%5")
        .arg(overallText, overallStatus, markdownPath, jsonPath,
            reportOk ? QString()
                : QStringLiteral("\n报告写入失败：Markdown=%1；JSON=%2")
                    .arg(markdownError, jsonError));

    ConfigSection run(ConfigLocation::Robot(robotName, "RobotAdaptorAcceptance"));
    run.SetSectionName(m_adaptorAcceptanceRunId.toStdString());
    run.WriteString("OverallStatus", overallStatus.toStdString());
    run.WriteString("ReportGeneratedAt", generatedAt.toStdString());
    run.WriteString("ReportMarkdownPath", markdownPath.toStdString());
    run.WriteString("ReportJsonPath", jsonPath.toStdString());
    SaveAdaptorAcceptanceRun();
    if (m_pAdaptorAcceptanceEvidence != nullptr
        && m_pAdaptorAcceptanceStageList != nullptr)
    {
        const int selectedStage = m_pAdaptorAcceptanceStageList->currentRow();
        if (selectedStage >= 0 && selectedStage < m_adaptorAcceptanceEvidence.size())
        {
            m_pAdaptorAcceptanceEvidence->setPlainText(
                m_adaptorAcceptanceEvidence[selectedStage]);
        }
    }
    RefreshAdaptorAcceptanceUi();
    if (!reportOk)
    {
        QMessageBox::warning(this, "机器人适配测试报告", m_adaptorAcceptanceEvidence[10]);
        return;
    }
    QMessageBox::information(this, "机器人适配测试报告",
        m_adaptorAcceptanceEvidence[10]
        + (allPassed ? QStringLiteral("\n\n全部阶段已通过。")
            : QStringLiteral("\n\n报告已生成，可根据失败、受限和未完成阶段继续修改底层或补充现场信息。")));
}

bool FunctionTestDialog::RunDashboardTool(const QString& actionId)
{
    if (IsMotionBusy() || m_bRobotCommandRunning || RobotOperationLease::AnyActive())
    {
        AppendLog(QString("机器人命令运行中，已忽略新的功能测试入口：%1；请先返回主页使用安全停止或等待完成。")
            .arg(actionId));
        return true;
    }
    if (actionId == "setSpeed")
    {
        FanucSetTpSpeedTest();
    }
    else if (actionId == "getPulse")
    {
        FanucGetCurrentPulseTest();
    }
    else if (actionId == "checkDone")
    {
        FanucCheckDoneTest();
    }
    else if (actionId == "setGetInt")
    {
        FanucSetGetIntTest();
    }
    else if (actionId == "callJob")
    {
        FanucCallJobTest();
    }
    else if (actionId == "uploadLs")
    {
        FanucUploadLsTest();
    }
    else if (actionId == "curposDiagnostic")
    {
        FanucCurposDiagnosticTest();
    }
    else if (actionId == "timestampDiagnostic")
    {
        RobotCameraTimestampDiagnosticTest();
    }
    else if (actionId == "movlTest")
    {
        FanucMovlTest();
    }
    else if (actionId == "movjTest")
    {
        FanucMovjTest();
    }
    else if (actionId == "moveZero")
    {
        FanucMoveZeroTest();
    }
    else if (actionId == "captureKinematics")
    {
        FanucCaptureKinematicsSample();
    }
    else if (actionId == "fitDh")
    {
        FitDhParametersFromSamples();
    }
    else if (actionId == "currentFrameFilter")
    {
        ExportCurrentCameraFramePointFilterTest();
    }
    else
    {
        return false;
    }
    return true;
}

void FunctionTestDialog::closeEvent(QCloseEvent* event)
{
    // 嵌入页 close 只会隐藏并返回主页，不会析构；后台线程均以 QPointer 回传，
    // driver 删除/程序退出另有租约门禁。运行中必须允许返回主页触发固定安全停止。
    SaveAdaptorAcceptanceRun();
    QDialog::closeEvent(event);
}

RobotDriverAdaptor* FunctionTestDialog::GetFirstDriverWithCapability(
    RobotDriverCapability capability,
    const QString& actionName)
{
	return GetFirstDriverWithCapabilities({ capability }, actionName);
}

RobotDriverAdaptor* FunctionTestDialog::GetFirstDriverWithCapabilities(
	std::initializer_list<RobotDriverCapability> capabilities,
	const QString& actionName)
{
    if (m_pContralUnit == nullptr || m_unitIndex < 0 || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, actionName, "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, actionName, "当前控制单元未创建驱动。");
        return nullptr;
    }

	if (!pRobotDriverAdaptor->SupportsAll(capabilities))
	{
		QMessageBox::warning(this, actionName,
			QStringLiteral("当前机器人品牌底层缺少以下适配能力，功能已限制：%1。\n"
				"请在对应品牌驱动完成实现和现场验证后再声明能力位。")
			.arg(QString::fromUtf8(
				pRobotDriverAdaptor->MissingCapabilitiesText(capabilities).c_str())));
		return nullptr;
	}
    return pRobotDriverAdaptor;
}

RobotDriverAdaptor* FunctionTestDialog::GetFirstRobotDriverAdaptor()
{
    if (m_pContralUnit == nullptr || m_unitIndex < 0 || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, "功能测试", "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor =
        static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, "功能测试", "当前控制单元未创建驱动。");
        return nullptr;
    }
    return pRobotDriverAdaptor;
}

bool FunctionTestDialog::IsMotionBusy() const
{
    // closeEvent 始终允许隐藏本页返回主页；busy 仅用于冻结本页普通操作入口。
    return m_bFanucMovlRunning || m_bFanucMovjRunning || m_bFanucMoveZeroRunning
        || m_adaptorAcceptanceBusy;
}

void FunctionTestDialog::RefreshMotionButtonState()
{
    bool busy = IsMotionBusy() || m_bRobotCommandRunning || RobotOperationLease::AnyActive() || m_adaptorJointMovedOut;
	RobotDriverAdaptor* driver = nullptr;
    if (!busy && m_pContralUnit != nullptr && m_unitIndex >= 0 && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
		driver = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
        RobotDriverAdaptor::StateSnapshot snapshot;
		busy = driver != nullptr
			&& driver->LatestStateSnapshot(snapshot)
            && snapshot.done == 0;
    }
	else if (m_pContralUnit != nullptr && m_unitIndex >= 0
		&& m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
	{
		driver = static_cast<RobotDriverAdaptor*>(
			m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
	}

    for (QPushButton* button : m_motionButtons)
    {
        if (button != nullptr)
        {
			const qulonglong requiredMask = button->property("requiredRobotCapabilities").toULongLong();
			const bool capabilityReady = driver != nullptr
				&& requiredMask != 0
				&& (driver->DriverCapabilities() & requiredMask) == requiredMask;
			button->setEnabled(!busy && capabilityReady);
			if (!busy && !capabilityReady)
			{
				button->setToolTip(QStringLiteral(
					"当前机器人品牌底层缺少“%1”所需适配能力，功能已限制。")
					.arg(button->property("robotCapabilityFeatureName").toString()));
			}
        }
    }
    // 机器人命令期间整块冻结，避免任意普通按钮再弹 QInputDialog/QMessageBox；
    // 返回主页属于嵌入页外层导航，不在 commandContent 内，始终保持可用。
    if (m_pCommandContent != nullptr)
    {
        m_pCommandContent->setEnabled(!busy);
    }
}

void FunctionTestDialog::AppendLog(const QString& text)
{
    if (m_pLogText == nullptr)
    {
        return;
    }
    m_pLogText->appendPlainText(text);
}

QString FunctionTestDialog::EnsureKinematicsSampleFilePath()
{
    if (!m_kinematicsSampleFilePath.isEmpty())
    {
        return m_kinematicsSampleFilePath;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstRobotDriverAdaptor();
    if (pRobotDriverAdaptor == nullptr)
    {
        return QString();
    }

    const QString robotName = DefaultRobotName(pRobotDriverAdaptor);
    const QString dirPath = RobotDataHelper::BuildProjectPath(QString("Result/%1/KinematicsSamples").arg(robotName));
    QDir dir;
    if (!dir.mkpath(dirPath))
    {
        QMessageBox::warning(this, "运动学样本", "创建运动学样本目录失败:\n" + NativeAbsolutePath(dirPath));
        return QString();
    }

    m_kinematicsSampleFilePath = QDir(dirPath).filePath(
        QString("DhSamples_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));

    QFile file(m_kinematicsSampleFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this, "运动学样本", "创建运动学样本文件失败:\n" + NativeAbsolutePath(m_kinematicsSampleFilePath));
        m_kinematicsSampleFilePath.clear();
        return QString();
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream << KinematicsCsvHeader() << "\n";
    m_kinematicsSampleCount = 0;
    return m_kinematicsSampleFilePath;
}

void FunctionTestDialog::FanucGetCurrentPosTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapability(
		RobotDriverCapability::PassiveState, QStringLiteral("读取当前位置"));
    if (pRobotDriver == nullptr)
    {
        return;
    }

    long long robotMs = 0;
    long long pcRecvMs = 0;
    T_ROBOT_COORS pos;
    RobotDriverAdaptor::StateSnapshot snapshot;
    if (pRobotDriver->LatestStateSnapshot(snapshot))
    {
        robotMs = snapshot.robotMs;
        pcRecvMs = snapshot.pcRecvMs;
        pos = snapshot.pose;
    }
    const QString message = QString("当前位置: robot_ms=%1, pc_recv_ms=%2, X=%3, Y=%4, Z=%5, RX=%6, RY=%7, RZ=%8")
        .arg(robotMs)
        .arg(pcRecvMs)
        .arg(pos.dX, 0, 'f', 3)
        .arg(pos.dY, 0, 'f', 3)
        .arg(pos.dZ, 0, 'f', 3)
        .arg(pos.dRX, 0, 'f', 3)
        .arg(pos.dRY, 0, 'f', 3)
        .arg(pos.dRZ, 0, 'f', 3);
    AppendLog(message);
    QMessageBox::information(this, "读取当前位置", message);
}

void FunctionTestDialog::FanucGetCurrentPulseTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapability(
		RobotDriverCapability::PassiveState, QStringLiteral("读取关节脉冲"));
    if (pRobotDriver == nullptr)
    {
        return;
    }

    long long robotMs = 0;
    long long pcRecvMs = 0;
    T_ANGLE_PULSE pulse;
    RobotDriverAdaptor::StateSnapshot snapshot;
    if (pRobotDriver->LatestStateSnapshot(snapshot))
    {
        robotMs = snapshot.robotMs;
        pcRecvMs = snapshot.pcRecvMs;
        pulse = snapshot.pulse;
    }
    const QString message = QString("关节脉冲: robot_ms=%1, pc_recv_ms=%2, S=%3, L=%4, U=%5, R=%6, B=%7, T=%8, EX1=%9, EX2=%10, EX3=%11")
        .arg(robotMs)
        .arg(pcRecvMs)
        .arg(pulse.nSPulse)
        .arg(pulse.nLPulse)
        .arg(pulse.nUPulse)
        .arg(pulse.nRPulse)
        .arg(pulse.nBPulse)
        .arg(pulse.nTPulse)
        .arg(pulse.lBXPulse)
        .arg(pulse.lBYPulse)
        .arg(pulse.lBZPulse);
    AppendLog(message);
    QMessageBox::information(this, "读取关节脉冲", message);
}

void FunctionTestDialog::FanucCurposDiagnosticTest()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::PassiveState, QStringLiteral("机器人诊断"));
    if (driver == nullptr)
    {
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("功能测试机器人诊断"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "机器人诊断", leaseError);
        return;
    }

    QStringList lines;
    const RobotDriverDescriptor descriptor = driver->DriverDescriptor();
    const RobotConnectionEndpoint endpoint = driver->ControlEndpoint();
    lines << QStringLiteral("品牌=%1，端点=%2:%3，连接=%4")
        .arg(QString::fromStdString(descriptor.displayName))
        .arg(QString::fromStdString(endpoint.host))
        .arg(endpoint.port)
        .arg(driver->IsConnected() ? QStringLiteral("YES") : QStringLiteral("NO"));
    lines << QStringLiteral("状态=%1")
        .arg(DecodeRobotMessageText(driver->GetRobotStatusText()));

    T_ROBOT_COORS pose;
    if (driver->TryGetCurrentPos(pose))
    {
        lines << QStringLiteral("位置 X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6")
            .arg(pose.dX, 0, 'f', 3).arg(pose.dY, 0, 'f', 3).arg(pose.dZ, 0, 'f', 3)
            .arg(pose.dRX, 0, 'f', 3).arg(pose.dRY, 0, 'f', 3).arg(pose.dRZ, 0, 'f', 3);
    }
    else
    {
        lines << QStringLiteral("位置读取失败=%1")
            .arg(DecodeRobotMessageText(driver->GetLastRobotError()));
    }
    T_ANGLE_PULSE pulse;
    if (driver->TryGetCurrentPulse(pulse))
    {
        lines << QStringLiteral("脉冲 S=%1 L=%2 U=%3 R=%4 B=%5 T=%6")
            .arg(pulse.nSPulse).arg(pulse.nLPulse).arg(pulse.nUPulse)
            .arg(pulse.nRPulse).arg(pulse.nBPulse).arg(pulse.nTPulse);
    }
    else
    {
        lines << QStringLiteral("脉冲读取失败=%1")
            .arg(DecodeRobotMessageText(driver->GetLastRobotError()));
    }
    const RobotMotionStatus motion = driver->ReadMotionStatus();
    lines << QStringLiteral("运动状态=%1 raw=%2 terminal=%3 detail=%4")
        .arg(static_cast<int>(motion.state)).arg(motion.rawCode)
        .arg(motion.terminalVerified ? QStringLiteral("YES") : QStringLiteral("NO"))
        .arg(QString::fromStdString(motion.detail));

    const QString message = lines.join("\n");
    AppendLog("机器人诊断:\n" + message);
    QMessageBox::information(this, "机器人诊断", message);
}

void FunctionTestDialog::RobotCameraTimestampDiagnosticTest()
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstDriverWithCapability(
		RobotDriverCapability::PassiveState, QStringLiteral("状态时间轴+相机时间轴"));
    if (pRobotDriverAdaptor == nullptr)
    {
        return;
    }
    pRobotDriverAdaptor->StartStateMonitor(50);

    if (m_pCameraCache == nullptr)
    {
        const QString message = "当前机器人没有可用的专属相机缓存，请确认机器人相机线程已初始化。";
        AppendLog(message);
        QMessageBox::warning(this, "状态时间轴+相机时间轴", message);
        return;
    }

    const QString robotName = DefaultRobotName(pRobotDriverAdaptor);
    const bool hasNativeRobotTimestamp = pRobotDriverAdaptor->Supports(
        RobotDriverCapability::RobotTimestamp);
    const QString robotTimelineSource = hasNativeRobotTimestamp
        ? QStringLiteral("控制器原生时间戳")
        : QStringLiteral("PC接收steady时间（该品牌未提供控制器时间戳）");
    CameraFrameCache* cameraCache = m_pCameraCache;
    const std::uint64_t beginCameraSequence = cameraCache->Mark();

    QVector<RobotTimestampSample> robotSamples;
    robotSamples.reserve(512);
    long long lastRobotMs = std::numeric_limits<long long>::min();
    long long lastPcRecvMs = std::numeric_limits<long long>::min();
    std::uint64_t lastRobotSequence = 0;
    int duplicateRobotReadCount = 0;
    int missingRobotTimestampCount = 0;

    AppendLog(QString("状态时间轴+相机时间轴检测开始：采集 %1 ms；机器人状态时间轴来源=%2。")
		.arg(kRobotCameraTimestampCheckDurationMs)
        .arg(robotTimelineSource));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kRobotCameraTimestampCheckDurationMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        RobotDriverAdaptor::StateSnapshot snapshot;
        if (!pRobotDriverAdaptor->LatestStateSnapshot(snapshot) || !snapshot.valid)
        {
            ++missingRobotTimestampCount;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        const long long sampleTimelineMs = hasNativeRobotTimestamp ? snapshot.robotMs : snapshot.pcRecvMs;
        if (sampleTimelineMs > 0 && snapshot.pcRecvMs > 0)
        {
            if (snapshot.sequence != lastRobotSequence
                && (sampleTimelineMs != lastRobotMs || snapshot.pcRecvMs != lastPcRecvMs))
            {
                RobotTimestampSample sample;
                sample.index = robotSamples.size() + 1;
                sample.robotTimestampUs = static_cast<qint64>(sampleTimelineMs) * 1000;
                sample.pcReceiveTimestampUs = static_cast<qint64>(snapshot.pcRecvMs) * 1000;
                sample.pose = snapshot.pose;
                sample.done = snapshot.done;
                robotSamples.push_back(sample);
                lastRobotMs = sampleTimelineMs;
                lastPcRecvMs = snapshot.pcRecvMs;
                lastRobotSequence = snapshot.sequence;
            }
            else
            {
                ++duplicateRobotReadCount;
            }
        }
        else
        {
            ++missingRobotTimestampCount;
        }

        QApplication::processEvents(QEventLoop::AllEvents, 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const std::uint64_t endCameraSequence = cameraCache->Mark();
    const std::vector<CameraFrameCache::TimedFrame> cameraFrames =
        cameraCache->TimedFramesBetween(beginCameraSequence, endCameraSequence);

    QVector<double> cameraTimestampDeltaMs;
    QVector<double> cameraSystemDeltaMs;
    QVector<double> cameraRatioValues;
    int invalidCameraTimestampCount = 0;
    int cameraBackwardsCount = 0;
    int cameraDuplicateCount = 0;
    int cameraSystemNonIncreasingCount = 0;

    QVector<double> robotTimestampDeltaMs;
    QVector<double> robotSystemDeltaMs;
    QVector<double> robotRatioValues;
    int robotBackwardsCount = 0;
    int robotDuplicateTimestampCount = 0;
    int robotSystemNonIncreasingCount = 0;

    QStringList csvLines;
    csvLines.push_back(
        "event_index,source,system_time_us,source_index,sequence,camera_timestamp_us,camera_delta_us,camera_system_delta_us,camera_to_system_ratio,"
        "robot_timestamp_us,robot_delta_us,robot_system_delta_us,robot_to_system_ratio,"
        "robot_x,robot_y,robot_z,robot_rx,robot_ry,robot_rz,done,target_x,target_y,target_z,error");

    struct EventRow
    {
        qint64 systemTimeUs = 0;
        QStringList fields;
    };
    QVector<EventRow> events;
    events.reserve(static_cast<int>(cameraFrames.size()) + robotSamples.size());

    for (int index = 0; index < static_cast<int>(cameraFrames.size()); ++index)
    {
        const CameraFrameCache::TimedFrame& frame = cameraFrames[static_cast<std::size_t>(index)];
        QString cameraDeltaText;
        QString cameraSystemDeltaText;
        QString cameraRatioText;

        if (frame.cameraTimestampUs <= 0)
        {
            ++invalidCameraTimestampCount;
        }

        if (index > 0)
        {
            const CameraFrameCache::TimedFrame& prev = cameraFrames[static_cast<std::size_t>(index - 1)];
            const qint64 cameraDeltaUs = frame.cameraTimestampUs - prev.cameraTimestampUs;
            const qint64 systemDeltaUs = frame.receiveTimestampUs - prev.receiveTimestampUs;
            cameraDeltaText = QString::number(cameraDeltaUs);
            cameraSystemDeltaText = QString::number(systemDeltaUs);

            if (cameraDeltaUs < 0)
            {
                ++cameraBackwardsCount;
            }
            else if (cameraDeltaUs == 0)
            {
                ++cameraDuplicateCount;
            }
            if (systemDeltaUs <= 0)
            {
                ++cameraSystemNonIncreasingCount;
            }

            if (cameraDeltaUs > 0 && systemDeltaUs > 0)
            {
                const double ratio = static_cast<double>(cameraDeltaUs) / static_cast<double>(systemDeltaUs);
                cameraRatioText = QString::number(ratio, 'f', 6);
                cameraTimestampDeltaMs.push_back(static_cast<double>(cameraDeltaUs) / 1000.0);
                cameraSystemDeltaMs.push_back(static_cast<double>(systemDeltaUs) / 1000.0);
                cameraRatioValues.push_back(ratio);
            }
        }

        QStringList fields;
        fields
            << "0"
            << "camera"
            << QString::number(frame.receiveTimestampUs)
            << QString::number(index + 1)
            << QString::number(frame.sequence)
            << QString::number(frame.cameraTimestampUs)
            << cameraDeltaText
            << cameraSystemDeltaText
            << cameraRatioText
            << "" << "" << "" << ""
            << "" << "" << "" << "" << "" << "" << ""
            << QString::number(frame.targetPoint.x, 'f', 6)
            << QString::number(frame.targetPoint.y, 'f', 6)
            << QString::number(frame.targetPoint.z, 'f', 6)
            << CsvEscapeForFunctionTest(frame.errorMessage);

        EventRow event;
        event.systemTimeUs = frame.receiveTimestampUs;
        event.fields = fields;
        events.push_back(event);
    }

    for (int index = 0; index < robotSamples.size(); ++index)
    {
        const RobotTimestampSample& sample = robotSamples[index];
        QString robotDeltaText;
        QString robotSystemDeltaText;
        QString robotRatioText;

        if (index > 0)
        {
            const RobotTimestampSample& prev = robotSamples[index - 1];
            const qint64 robotDeltaUs = sample.robotTimestampUs - prev.robotTimestampUs;
            const qint64 systemDeltaUs = sample.pcReceiveTimestampUs - prev.pcReceiveTimestampUs;
            robotDeltaText = QString::number(robotDeltaUs);
            robotSystemDeltaText = QString::number(systemDeltaUs);

            if (robotDeltaUs < 0)
            {
                ++robotBackwardsCount;
            }
            else if (robotDeltaUs == 0)
            {
                ++robotDuplicateTimestampCount;
            }
            if (systemDeltaUs <= 0)
            {
                ++robotSystemNonIncreasingCount;
            }

            if (robotDeltaUs > 0 && systemDeltaUs > 0)
            {
                const double ratio = static_cast<double>(robotDeltaUs) / static_cast<double>(systemDeltaUs);
                robotRatioText = QString::number(ratio, 'f', 6);
                robotTimestampDeltaMs.push_back(static_cast<double>(robotDeltaUs) / 1000.0);
                robotSystemDeltaMs.push_back(static_cast<double>(systemDeltaUs) / 1000.0);
                robotRatioValues.push_back(ratio);
            }
        }

        QStringList fields;
        fields
            << "0"
            << "robot"
            << QString::number(sample.pcReceiveTimestampUs)
            << QString::number(sample.index)
            << ""
            << "" << "" << "" << ""
            << QString::number(sample.robotTimestampUs)
            << robotDeltaText
            << robotSystemDeltaText
            << robotRatioText
            << QString::number(sample.pose.dX, 'f', 6)
            << QString::number(sample.pose.dY, 'f', 6)
            << QString::number(sample.pose.dZ, 'f', 6)
            << QString::number(sample.pose.dRX, 'f', 6)
            << QString::number(sample.pose.dRY, 'f', 6)
            << QString::number(sample.pose.dRZ, 'f', 6)
            << QString::number(sample.done)
            << "" << "" << "" << "";

        EventRow event;
        event.systemTimeUs = sample.pcReceiveTimestampUs;
        event.fields = fields;
        events.push_back(event);
    }

    std::sort(events.begin(), events.end(), [](const EventRow& a, const EventRow& b)
        {
            return a.systemTimeUs < b.systemTimeUs;
        });
    for (int index = 0; index < events.size(); ++index)
    {
        events[index].fields[0] = QString::number(index + 1);
        csvLines.push_back(JoinCsvRow(events[index].fields));
    }

    const IntervalStats cameraTimestampStats = CalcIntervalStats(cameraTimestampDeltaMs);
    const IntervalStats cameraSystemStats = CalcIntervalStats(cameraSystemDeltaMs);
    const IntervalStats cameraRatioStats = CalcIntervalStats(cameraRatioValues);
    const IntervalStats robotTimestampStats = CalcIntervalStats(robotTimestampDeltaMs);
    const IntervalStats robotSystemStats = CalcIntervalStats(robotSystemDeltaMs);
    const IntervalStats robotRatioStats = CalcIntervalStats(robotRatioValues);

    qint64 cameraTotalTimestampUs = 0;
    qint64 cameraTotalSystemUs = 0;
    if (cameraFrames.size() >= 2)
    {
        cameraTotalTimestampUs = cameraFrames.back().cameraTimestampUs - cameraFrames.front().cameraTimestampUs;
        cameraTotalSystemUs = cameraFrames.back().receiveTimestampUs - cameraFrames.front().receiveTimestampUs;
    }
    qint64 robotTotalTimestampUs = 0;
    qint64 robotTotalSystemUs = 0;
    if (robotSamples.size() >= 2)
    {
        robotTotalTimestampUs = robotSamples.back().robotTimestampUs - robotSamples.front().robotTimestampUs;
        robotTotalSystemUs = robotSamples.back().pcReceiveTimestampUs - robotSamples.front().pcReceiveTimestampUs;
    }

    const double cameraTotalRatio = cameraTotalSystemUs > 0
        ? static_cast<double>(cameraTotalTimestampUs) / static_cast<double>(cameraTotalSystemUs)
        : 0.0;
    const double robotTotalRatio = robotTotalSystemUs > 0
        ? static_cast<double>(robotTotalTimestampUs) / static_cast<double>(robotTotalSystemUs)
        : 0.0;
    const double cameraVsRobotScale = std::abs(robotTotalRatio) > 1e-12
        ? cameraTotalRatio / robotTotalRatio
        : 0.0;
    const double cameraRatioCvPercent = std::abs(cameraRatioStats.mean) > 1e-12
        ? cameraRatioStats.stddev / std::abs(cameraRatioStats.mean) * 100.0
        : 0.0;
    const double robotRatioCvPercent = std::abs(robotRatioStats.mean) > 1e-12
        ? robotRatioStats.stddev / std::abs(robotRatioStats.mean) * 100.0
        : 0.0;

    QString conclusion;
    if (cameraFrames.size() < 3 || robotSamples.size() < 3)
    {
        conclusion = "结论：相机或机器人有效样本不足，无法判断两边时间轴。";
    }
    else if (std::abs(cameraTotalRatio - 1.0) <= 0.05
        && std::abs(robotTotalRatio - 1.0) <= 0.05
        && std::abs(cameraVsRobotScale - 1.0) <= 0.05)
    {
        conclusion = "结论：相机timestamp、机器人状态时间轴与本机接收时间的总时长比例都接近 1，时间单位假设基本成立。";
    }
    else
    {
        conclusion = QString("结论：时间比例存在偏差。camera/system=%1，robot/system=%2，camera/robot=%3。")
            .arg(cameraTotalRatio, 0, 'f', 6)
            .arg(robotTotalRatio, 0, 'f', 6)
            .arg(cameraVsRobotScale, 0, 'f', 6);
    }

    const QString csvPath = BuildRobotCameraTimestampCheckPath(robotName);
    QString saveError;
    const bool saveOk = RobotDataHelper::SaveTextFileLines(csvPath, csvLines, &saveError);

    QString resultText = QString(
        "采集时长：%1 ms\n"
        "相机帧数：%2，机器人样本数：%3\n"
        "相机总时长：timestamp=%4 ms，system=%5 ms，比例=%6\n"
        "机器人总时长：robot_time=%7 ms，system=%8 ms，比例=%9\n"
        "相机/机器人比例：%10\n\n"
        "%11\n%12\n%13\n"
        "相机比例CV：%14%\n\n"
        "%15\n%16\n%17\n"
        "机器人比例CV：%18%\n\n"
        "异常统计：相机无效=%19，倒退=%20，重复=%21，system非递增=%22；机器人缺时间=%23，重复读取=%24，倒退=%25，重复timestamp=%26，system非递增=%27\n\n"
        "%28\n\n"
        "CSV：%29")
        .arg(kRobotCameraTimestampCheckDurationMs)
        .arg(cameraFrames.size())
        .arg(robotSamples.size())
        .arg(static_cast<double>(cameraTotalTimestampUs) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(cameraTotalSystemUs) / 1000.0, 0, 'f', 3)
        .arg(cameraTotalRatio, 0, 'f', 6)
        .arg(static_cast<double>(robotTotalTimestampUs) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(robotTotalSystemUs) / 1000.0, 0, 'f', 3)
        .arg(robotTotalRatio, 0, 'f', 6)
        .arg(cameraVsRobotScale, 0, 'f', 6)
        .arg(FormatStatsLine("相机timestamp间隔", cameraTimestampStats, " ms"))
        .arg(FormatStatsLine("相机system间隔", cameraSystemStats, " ms"))
        .arg(FormatStatsLine("相机 timestamp/system 比例", cameraRatioStats, "", 6))
        .arg(cameraRatioCvPercent, 0, 'f', 2)
        .arg(FormatStatsLine("机器人时间轴间隔", robotTimestampStats, " ms"))
        .arg(FormatStatsLine("机器人system间隔", robotSystemStats, " ms"))
        .arg(FormatStatsLine("机器人 time/system 比例", robotRatioStats, "", 6))
        .arg(robotRatioCvPercent, 0, 'f', 2)
        .arg(invalidCameraTimestampCount)
        .arg(cameraBackwardsCount)
        .arg(cameraDuplicateCount)
        .arg(cameraSystemNonIncreasingCount)
        .arg(missingRobotTimestampCount)
        .arg(duplicateRobotReadCount)
        .arg(robotBackwardsCount)
        .arg(robotDuplicateTimestampCount)
        .arg(robotSystemNonIncreasingCount)
        .arg(conclusion)
        .arg(saveOk ? csvPath : QString("保存失败：%1").arg(saveError));
    resultText.prepend(QString("机器人状态时间轴来源：%1\n").arg(robotTimelineSource));

    AppendLog(QString("状态时间轴+相机时间轴检测完成：相机帧=%1，机器人样本=%2，camera/system=%3，robot/system=%4，camera/robot=%5，CSV=%6")
        .arg(cameraFrames.size())
        .arg(robotSamples.size())
        .arg(cameraTotalRatio, 0, 'f', 6)
        .arg(robotTotalRatio, 0, 'f', 6)
        .arg(cameraVsRobotScale, 0, 'f', 6)
        .arg(saveOk ? csvPath : QString("保存失败")));
    QMessageBox::information(this, "状态时间轴+相机时间轴", resultText);
}

void FunctionTestDialog::FanucCheckDoneTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapability(
		RobotDriverCapability::PassiveState, QStringLiteral("检查运行状态"));
    if (pRobotDriver == nullptr)
    {
        return;
    }

    long long robotMs = 0;
    long long pcRecvMs = 0;
    int done = -1;
    RobotDriverAdaptor::StateSnapshot snapshot;
    if (pRobotDriver->LatestStateSnapshot(snapshot))
    {
        robotMs = snapshot.robotMs;
        pcRecvMs = snapshot.pcRecvMs;
        done = snapshot.done;
    }
    const QString message = QString("CheckDone 返回值：%1，robot_ms=%2，pc_recv_ms=%3")
        .arg(done)
        .arg(robotMs)
        .arg(pcRecvMs);
    AppendLog(message);
    QMessageBox::information(this, "检查运行完成", message);
}

void FunctionTestDialog::FanucSetGetIntTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapability(
		RobotDriverCapability::IntegerRegister, QStringLiteral("写读INT寄存器"));
    if (pRobotDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const int index = QInputDialog::getInt(this, "写读INT寄存器", "寄存器编号：", 10, 1, 9999, 1, &ok);
    if (!ok)
    {
        return;
    }

    const int value = QInputDialog::getInt(this, "写读INT寄存器", "写入值：", 123, -999999, 999999, 1, &ok);
    if (!ok)
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("功能测试写读寄存器"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "写读INT寄存器", leaseError);
        return;
    }

    if (!pRobotDriver->SetIntVar(index, value))
    {
        QMessageBox::warning(this, "写读INT寄存器", DecodeRobotMessageText(GetStr("写入 INT%d 失败。", index)));
        return;
    }

    int readValue = 0;
    if (!pRobotDriver->TryGetIntVar(index, readValue))
    {
        QMessageBox::warning(this, "写读INT寄存器",
            "写入成功，但严格回读失败："
            + DecodeRobotMessageText(pRobotDriver->GetLastRobotError()));
        return;
    }
    const QString message = QString("写入 INT%1=%2, 读取值=%3").arg(index).arg(value).arg(readValue);
    AppendLog(message);
    QMessageBox::information(this, "写读INT寄存器", message);
}

void FunctionTestDialog::FanucSetTpSpeedTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapability(
		RobotDriverCapability::TeachPendantSpeedControl, QStringLiteral("设置速度"));
    if (pRobotDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const int speed = QInputDialog::getInt(this, "设置速度", "速度百分比：", 50, 1, 100, 1, &ok);
    if (!ok)
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("功能测试设置 TP 速度"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "设置速度", leaseError);
        return;
    }

    const bool setOk = pRobotDriver->SetTpSpeed(speed);
    const QString message = setOk ? QString("设置速度成功：%1").arg(speed) : QString("设置速度失败：%1").arg(speed);
    AppendLog(message);
    QMessageBox::information(this, "设置速度", message);
}

void FunctionTestDialog::FanucCallJobTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapabilities(
		{ RobotDriverCapability::NativeProgramExecution,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("调用任务"));
    if (pRobotDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const QString jobName = QInputDialog::getText(
        this, "调用任务", "任务/程序名（STEP可输入Project/Program；汇川输入当前工程的公共模块名，模块需提供Func Run()）：",
        QLineEdit::Normal, QString(), &ok);
    if (!ok || jobName.trimmed().isEmpty())
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("功能测试调用机器人任务"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "调用任务", leaseError);
        return;
    }

    const QByteArray jobNameBytes = jobName.trimmed().toLocal8Bit();
    m_bRobotCommandRunning = true;
    RefreshMotionButtonState();
    AppendLog(QString("正在尝试调用任务 %1；仅带可验证完成契约的入口允许启动。")
        .arg(jobName.trimmed()));
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pRobotDriver, jobNameBytes, operationLease]()
        {
            RobotMotionStatus terminalStatus;
            const bool flowOk = pRobotDriver->RunProgramAndWait(
                jobNameBytes.constData(), 5000, 1800000, 200, &terminalStatus);
            const QString detail = DecodeRobotMessageText(pRobotDriver->GetLastRobotError());
            const QString message = QString("调用任务%1：%2，state=%3，raw=%4，terminalVerified=%5，详情=%6")
                .arg(flowOk ? QStringLiteral("成功") : QStringLiteral("失败"))
                .arg(QString::fromLocal8Bit(jobNameBytes))
                .arg(static_cast<int>(terminalStatus.state))
                .arg(terminalStatus.rawCode)
                .arg(terminalStatus.terminalVerified ? QStringLiteral("YES") : QStringLiteral("NO"))
                .arg(detail);
            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bRobotCommandRunning = false;
                    self->RefreshMotionButtonState();
                    self->AppendLog(message);
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::FanucUploadLsTest()
{
    RobotDriverAdaptor* driver = GetFirstDriverWithCapability(
        RobotDriverCapability::NativeProgramUpload, QStringLiteral("发送原生程序"));
    if (driver == nullptr)
    {
        return;
    }

    const RobotFileTransferProfile profile = driver->FileTransferProfile();
    QStringList patterns;
    for (const std::string& filter : profile.localFileFilters)
    {
        patterns.push_back(QString::fromStdString(filter));
    }
    const QString profileLocalDirectory = QString::fromStdString(profile.defaultLocalDirectory);
    QString defaultDirectory = AppPaths::FindResourcePath(profileLocalDirectory);
    const QString writableDirectory = AppPaths::WritablePath(profileLocalDirectory);
    if (!QFileInfo(defaultDirectory).isDir() && QFileInfo(writableDirectory).isDir())
    {
        defaultDirectory = writableDirectory;
    }
    const QString filterText = patterns.isEmpty()
        ? QStringLiteral("所有文件 (*.*)")
        : QStringLiteral("当前机器人原生程序 (%1);;所有文件 (*.*)")
            .arg(patterns.join(' '));
    const QString nativeProgramPath = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("选择要上传的机器人原生程序"),
        defaultDirectory,
        filterText);
    if (nativeProgramPath.isEmpty())
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("功能测试上传原生程序"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "发送原生程序", leaseError);
        return;
    }

    const QByteArray nativeProgramPathBytes = nativeProgramPath.toLocal8Bit();
    const int ret = driver->UploadNativeProgramSource(nativeProgramPathBytes.constData());
    const QString message = ret == 0
        ? QString("原生程序发送成功：%1").arg(nativeProgramPath)
        : QString("原生程序发送失败，返回码=%1，文件=%2，详情=%3")
            .arg(ret)
            .arg(nativeProgramPath)
            .arg(DecodeRobotMessageText(driver->GetLastRobotError()));
    AppendLog(message);
    if (ret == 0)
    {
        QMessageBox::information(this, "发送原生程序", message);
    }
    else
    {
        QMessageBox::warning(this, "发送原生程序", message);
    }
}

void FunctionTestDialog::FanucMovlTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapabilities(
		{ RobotDriverCapability::LinearMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("MOVL往返测试"));
    if (pRobotDriver == nullptr)
    {
        return;
    }
    if (m_bFanucMovlRunning)
    {
        QMessageBox::information(this, "MOVL往返测试", "MOVL测试正在执行，请等本次运动结束。");
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("功能测试 MOVL"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "MOVL往返测试", leaseError);
        return;
    }

    const bool moveForward = m_bFanucMovlForward;
    m_bFanucMovlForward = !m_bFanucMovlForward;
    m_bFanucMovlRunning = true;
    RefreshMotionButtonState();
    AppendLog(QString("开始 MOVL %1 100mm 测试...").arg(moveForward ? "Y+" : "Y-"));

    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pRobotDriver, moveForward, operationLease]()
        {
            T_ROBOT_COORS target;
            const bool currentOk = pRobotDriver->TryGetCurrentPos(target);
            if (currentOk)
            {
                target.dY += moveForward ? 100.0 : -100.0;
            }

            const bool moveOk = currentOk
                && pRobotDriver->MoveLinearMmPerMin(target, 300.0, pRobotDriver->ExternalAxleType());
            const int done = moveOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
            const QString message = QString("MOVL %1 100mm, Move=%2, CheckRobotDone=%3")
                .arg(moveForward ? "Y+" : "Y-")
                .arg(moveOk ? "OK" : "FAIL")
                .arg(done);

            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bFanucMovlRunning = false;
                    self->RefreshMotionButtonState();
                    self->AppendLog(message);
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::FanucMovjTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapabilities(
		{ RobotDriverCapability::JointMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("MOVJ测试"));
    if (pRobotDriver == nullptr)
    {
        return;
    }
    if (m_bFanucMovjRunning)
    {
        QMessageBox::information(this, "MOVJ测试", "MOVJ测试正在执行，请等本次运动结束。");
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("功能测试 MOVJ"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "MOVJ测试", leaseError);
        return;
    }

    m_bFanucMovjRunning = true;
    RefreshMotionButtonState();
    AppendLog("开始 MOVJ J2/J3 +5deg 测试...");

    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pRobotDriver, operationLease]()
        {
            T_ANGLE_PULSE target;
            const bool currentOk = pRobotDriver->TryGetCurrentPulse(target);
            const double j2PulseUnit = pRobotDriver->AxisUnit().dLPulseUnit;
            const double j3PulseUnit = pRobotDriver->AxisUnit().dUPulseUnit;
            const long j2DeltaPulse = j2PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j2PulseUnit));
            const long j3DeltaPulse = j3PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j3PulseUnit));
            if (currentOk)
            {
                target.nLPulse += j2DeltaPulse;
                target.nUPulse += j3DeltaPulse;
            }

            const bool moveOk = currentOk
                && pRobotDriver->MoveJointPercent(target, 1.0, pRobotDriver->ExternalAxleType());
            const int done = moveOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
            const QString message = QString("MOVJ J2/J3 +5deg, J2DeltaPulse=%1, J3DeltaPulse=%2, Move=%3, CheckRobotDone=%4")
                .arg(j2DeltaPulse)
                .arg(j3DeltaPulse)
                .arg(moveOk ? "OK" : "FAIL")
                .arg(done);

            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bFanucMovjRunning = false;
                    self->RefreshMotionButtonState();
                    self->AppendLog(message);
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::FanucMoveZeroTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstDriverWithCapabilities(
		{ RobotDriverCapability::JointMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("运动到零位"));
    if (pRobotDriver == nullptr)
    {
        return;
    }
    if (m_bFanucMoveZeroRunning)
    {
        QMessageBox::information(this, "运动到零位", "零位运动正在执行，请等本次运动结束。");
        return;
    }

    const QMessageBox::StandardButton confirm = QMessageBox::question(
        this,
        "运动到零位",
        "将通过 MOVJ 低速运动到 J1-J6=0 的零位。\n请确认机器人周围安全，是否继续？",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (confirm != QMessageBox::Yes)
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("功能测试运动到零位"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "运动到零位", leaseError);
        return;
    }

    m_bFanucMoveZeroRunning = true;
    RefreshMotionButtonState();
    AppendLog("开始 MOVJ 到零位...");

    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pRobotDriver, operationLease]()
        {
            const T_ANGLE_PULSE zeroPulse = T_ANGLE_PULSE();
			const bool moveOk = pRobotDriver->MoveJointPercent(
				zeroPulse, 1.0, pRobotDriver->ExternalAxleType());
            const int done = moveOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
            T_ROBOT_COORS pos;
            T_ANGLE_PULSE pulse;
            const bool feedbackOk = pRobotDriver->TryGetCurrentPos(pos)
                && pRobotDriver->TryGetCurrentPulse(pulse);

            const QString message = QString(
                "MOVJ 到零位, Move=%1, CheckRobotDone=%2\n"
                "当前位置: X=%3, Y=%4, Z=%5, RX=%6, RY=%7, RZ=%8\n"
                "当前脉冲: S=%9, L=%10, U=%11, R=%12, B=%13, T=%14, EX1=%15, EX2=%16, EX3=%17\n"
                "反馈读取=%18")
                .arg(moveOk ? "OK" : "FAIL")
                .arg(done)
                .arg(pos.dX, 0, 'f', 3)
                .arg(pos.dY, 0, 'f', 3)
                .arg(pos.dZ, 0, 'f', 3)
                .arg(pos.dRX, 0, 'f', 3)
                .arg(pos.dRY, 0, 'f', 3)
                .arg(pos.dRZ, 0, 'f', 3)
                .arg(pulse.nSPulse)
                .arg(pulse.nLPulse)
                .arg(pulse.nUPulse)
                .arg(pulse.nRPulse)
                .arg(pulse.nBPulse)
                .arg(pulse.nTPulse)
                .arg(pulse.lBXPulse)
                .arg(pulse.lBYPulse)
                .arg(pulse.lBZPulse)
                .arg(feedbackOk ? QStringLiteral("OK") : QStringLiteral("FAIL"));

            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bFanucMoveZeroRunning = false;
                    self->RefreshMotionButtonState();
                    self->AppendLog(message);
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::FanucCaptureKinematicsSample()
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstDriverWithCapability(
		RobotDriverCapability::PassiveState, QStringLiteral("保存运动学样本"));
    if (pRobotDriverAdaptor == nullptr)
    {
        return;
    }

    const QString sampleFilePath = EnsureKinematicsSampleFilePath();
    if (sampleFilePath.isEmpty())
    {
        return;
    }

    T_ANGLE_PULSE pulse;
    T_ROBOT_COORS robotPose;
    if (!pRobotDriverAdaptor->TryGetCurrentPulse(pulse)
        || !pRobotDriverAdaptor->TryGetCurrentPos(robotPose))
    {
        QMessageBox::warning(this, "保存关节+直角",
            "读取机器人关节或直角坐标失败，样本未保存。\n"
            + DecodeRobotMessageText(pRobotDriverAdaptor->GetLastRobotError()));
        return;
    }
    QString toolName;
    const T_ROBOT_COORS toolCoors = EffectiveKinematicsTool(pRobotDriverAdaptor, &toolName);
    T_ROBOT_COORS modelPose;
    const bool fkOk = ForwardPoseFromDhParams(
        KinematicsToParamArray(pRobotDriverAdaptor->KinematicsParameters()),
        pRobotDriverAdaptor->AxisUnit(),
        pulse,
        toolCoors,
        modelPose);
    if (!fkOk)
    {
        QMessageBox::warning(this, "保存关节+直角", "当前 DH 参数回代失败，样本未保存。");
        return;
    }

    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
    const int sampleIndex = m_kinematicsSampleCount + 1;

    QFile file(sampleFilePath);
    if (!file.open(QIODevice::Append | QIODevice::Text))
    {
        QMessageBox::warning(this, "保存关节+直角", "打开样本文件失败:\n" + NativeAbsolutePath(sampleFilePath));
        return;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream << BuildKinematicsCsvRow(
        sampleIndex,
        timestamp,
        pulse,
        pRobotDriverAdaptor->AxisUnit(),
        robotPose,
        modelPose) << "\n";
    ++m_kinematicsSampleCount;

    const QString message = QString(
        "已保存第 %1 个运动学样本。\n"
        "文件：%2\n"
        "当前机器人位姿：X=%3 Y=%4 Z=%5 RX=%6 RY=%7 RZ=%8\n"
        "当前DH回代误差：dX=%9 dY=%10 dZ=%11 dRX=%12 dRY=%13 dRZ=%14\n"
        "说明：单个样本不能唯一反推整套DH，建议采至少 %15 个分散姿态点。")
        .arg(sampleIndex)
        .arg(NativeAbsolutePath(sampleFilePath))
        .arg(robotPose.dX, 0, 'f', 3)
        .arg(robotPose.dY, 0, 'f', 3)
        .arg(robotPose.dZ, 0, 'f', 3)
        .arg(robotPose.dRX, 0, 'f', 3)
        .arg(robotPose.dRY, 0, 'f', 3)
        .arg(robotPose.dRZ, 0, 'f', 3)
        .arg(modelPose.dX - robotPose.dX, 0, 'f', 3)
        .arg(modelPose.dY - robotPose.dY, 0, 'f', 3)
        .arg(modelPose.dZ - robotPose.dZ, 0, 'f', 3)
        .arg(WrapAngleDeg(modelPose.dRX - robotPose.dRX), 0, 'f', 3)
        .arg(WrapAngleDeg(modelPose.dRY - robotPose.dRY), 0, 'f', 3)
        .arg(WrapAngleDeg(modelPose.dRZ - robotPose.dRZ), 0, 'f', 3)
        .arg(kDhFitRecommendedSampleCount);

    const QString toolLine = QString("\n当前回代使用工具：%1  [X=%2 Y=%3 Z=%4 RX=%5 RY=%6 RZ=%7]")
        .arg(toolName)
        .arg(toolCoors.dX, 0, 'f', 3)
        .arg(toolCoors.dY, 0, 'f', 3)
        .arg(toolCoors.dZ, 0, 'f', 3)
        .arg(toolCoors.dRX, 0, 'f', 3)
        .arg(toolCoors.dRY, 0, 'f', 3)
        .arg(toolCoors.dRZ, 0, 'f', 3);

    AppendLog(message + toolLine);
    QMessageBox::information(this, "保存关节+直角", message + toolLine);
}

void FunctionTestDialog::EditKinematicsParameters()
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstRobotDriverAdaptor();
    if (pRobotDriverAdaptor == nullptr)
    {
        const QString message = "当前没有可用的机器人驱动，无法建立运动学候选配置。";
        AppendLog(message);
        QMessageBox::warning(this, "填写DH/MDH参数", message);
        return;
    }

    KinematicsDraftDialog dialog(pRobotDriverAdaptor, this);
    dialog.exec();
    if (!dialog.WasSaved())
    {
        AppendLog("运动学候选参数编辑已关闭，当前运行参数未改变。");
        return;
    }

    const QString message = QString(
        "运动学候选参数已保存：%1\n"
        "状态：CandidateUnvalidated；当前运行参数和机器人控制器均未改变。")
        .arg(dialog.DraftStorageLabel());
    AppendLog(message);
    QMessageBox::information(this, "填写DH/MDH参数", message);
}

void FunctionTestDialog::FitDhParametersFromSamples()
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstRobotDriverAdaptor();
    if (pRobotDriverAdaptor == nullptr)
    {
        return;
    }

    const QString sampleFilePath = EnsureKinematicsSampleFilePath();
    if (sampleFilePath.isEmpty())
    {
        return;
    }

    QVector<KinematicsFitSample> samples;
    QString error;
    if (!LoadKinematicsSamplesFromCsv(sampleFilePath, samples, &error))
    {
        QMessageBox::warning(this, "拟合DH参数", error);
        return;
    }

    if (samples.size() < kDhFitMinSampleCount)
    {
        QMessageBox::warning(
            this,
            "拟合DH参数",
            QString("当前只有 %1 个样本，至少需要 %2 个样本才能开始拟合。\n建议采集 %3 个以上、姿态尽量分散的点。")
                .arg(samples.size())
                .arg(kDhFitMinSampleCount)
                .arg(kDhFitRecommendedSampleCount));
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);
    QString toolName;
    const T_ROBOT_COORS toolCoors = EffectiveKinematicsTool(pRobotDriverAdaptor, &toolName);
    std::array<double, kDhParamCount> fittedParams{};
    double beforePositionRmse = 0.0;
    double beforeRotationRmse = 0.0;
    double afterPositionRmse = 0.0;
    double afterRotationRmse = 0.0;
    const bool fitOk = FitDhParamsByLeastSquares(
        samples,
        pRobotDriverAdaptor->KinematicsParameters(),
        pRobotDriverAdaptor->AxisUnit(),
        toolCoors,
        fittedParams,
        beforePositionRmse,
        beforeRotationRmse,
        afterPositionRmse,
        afterRotationRmse);
    QApplication::restoreOverrideCursor();

    if (!fitOk)
    {
        QMessageBox::warning(this, "拟合DH参数", "DH拟合失败，可能是样本分布不够开，或者当前位姿数据不一致。");
        return;
    }

    const std::array<double, kDhParamCount> initialParams = KinematicsToParamArray(pRobotDriverAdaptor->KinematicsParameters());
    const QString report = BuildDhParameterReport(
        samples,
        initialParams,
        fittedParams,
        beforePositionRmse,
        beforeRotationRmse,
        afterPositionRmse,
        afterRotationRmse);

    const QFileInfo sampleFileInfo(sampleFilePath);
    const QString reportPath = sampleFileInfo.dir().filePath(
        QString("DhFitReport_%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
    QString saveError;
    if (!RobotDataHelper::SaveTextFileLines(reportPath, report.split('\n'), &saveError))
    {
        QMessageBox::warning(this, "拟合DH参数", saveError);
        return;
    }

    const QString message = QString(
        "DH拟合完成。\n"
        "样本数：%1\n"
        "拟合前 RMSE：位置=%2 mm，姿态=%3 deg\n"
        "拟合后 RMSE：位置=%4 mm，姿态=%5 deg\n"
        "拟合使用工具：%6\n"
        "报告：%7\n"
        "说明：结果未自动写回数据库，请先核对报告再决定是否替换当前参数。")
        .arg(samples.size())
        .arg(beforePositionRmse, 0, 'f', 4)
        .arg(beforeRotationRmse, 0, 'f', 4)
        .arg(afterPositionRmse, 0, 'f', 4)
        .arg(afterRotationRmse, 0, 'f', 4)
        .arg(toolName)
        .arg(NativeAbsolutePath(reportPath));

    AppendLog(message);
    QMessageBox::information(this, "拟合DH参数", message);
}

void FunctionTestDialog::ExportCurrentCameraFramePointFilterTest()
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstRobotDriverAdaptor();
    if (pRobotDriverAdaptor == nullptr)
    {
        return;
    }

    if (m_pCameraCache == nullptr)
    {
        const QString message = "当前机器人没有可用的专属相机缓存，请确认机器人相机线程已初始化。";
        AppendLog(message);
        QMessageBox::warning(this, "当前帧点云滤波", message);
        return;
    }

    udpDataShow frame;
    if (!m_pCameraCache->Latest(frame))
    {
        const QString message = "当前相机缓存中没有可用帧。";
        AppendLog(message);
        QMessageBox::warning(this, "当前帧点云滤波", message);
        return;
    }

    if (frame.allResultPoint.empty())
    {
        const QString message = QString("当前帧没有 allResultPoint 点云数据，timestamp=%1。")
            .arg(frame.timestamp);
        AppendLog(message);
        QMessageBox::warning(this, "当前帧点云滤波", message);
        return;
    }

    const QString robotName = DefaultRobotName(pRobotDriverAdaptor);
    const QString outputDir = BuildCameraFramePointFilterTestDir(robotName);
    if (!QFileInfo::exists(outputDir))
    {
        const QString message = "创建当前帧点云滤波输出目录失败:\n" + outputDir;
        AppendLog(message);
        QMessageBox::warning(this, "当前帧点云滤波", message);
        return;
    }

    const LaserFramePoint3DFilterOptions filterOptions = BuildThreeSegmentCameraFrameFilterOptions();
    const std::vector<cv::Point3d> filteredFramePoints = FilterSingleFrameLaserPoint3D(frame.allResultPoint, filterOptions);
    const std::vector<LaserFramePoint3D> rawPoints = BuildLaserFramePoint3DList(frame.allResultPoint);
    const std::vector<LaserFramePoint3D> filteredPoints = BuildLaserFramePoint3DList(filteredFramePoints);

    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    const QString rawPath = QDir(outputDir).filePath(QString("CameraFrame3D_Raw_%1.txt").arg(timestamp));
    const QString filteredPath = QDir(outputDir).filePath(QString("CameraFrame3D_Filtered_%1.txt").arg(timestamp));

    QString error;
    if (!WriteLaserFramePoint3DFile(rawPath, rawPoints, &error))
    {
        AppendLog(error);
        QMessageBox::warning(this, "当前帧点云滤波", error);
        return;
    }
    if (!WriteLaserFramePoint3DFile(filteredPath, filteredPoints, &error))
    {
        AppendLog(error);
        QMessageBox::warning(this, "当前帧点云滤波", error);
        return;
    }

    const QString message = QString("当前帧三维点云滤波完成：原始点=%1，滤波后=%2。\n原始文件：%3\n滤波文件：%4")
        .arg(rawPoints.size())
        .arg(filteredPoints.size())
        .arg(NativeAbsolutePath(rawPath))
        .arg(NativeAbsolutePath(filteredPath));
    AppendLog(message);
    QMessageBox::information(this, "当前帧点云滤波", message);
}

void FunctionTestDialog::OpenLaserWeldFilterTest()
{
    const QString message = "精测点云处理已移到管理页面的“工艺”菜单，请从管理页面打开。";
    AppendLog(message);
    QMessageBox::information(this, "精测点云处理", message);
}
