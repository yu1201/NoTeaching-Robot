#include "FunctionTestDialog.h"

#include "CameraFrameCache.h"
#include "FANUCRobotDriver.h"
#include "RobotDataHelper.h"
#include "RobotDriverAdaptor.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"
#include "WindowStyleHelper.h"
#include "../portable/LaserFramePoint3DFilter/LaserFramePoint3DFilter.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QInputDialog>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStringConverter>
#include <QStringList>
#include <QTextDocument>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <thread>
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
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        const QString candidate = dir.filePath(relativePath);
        if (QFileInfo::exists(candidate))
        {
            return QDir::toNativeSeparators(QFileInfo(candidate).absoluteFilePath());
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return QString();
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
    if (pRobotDriverAdaptor == nullptr || pRobotDriverAdaptor->m_sRobotName.empty())
    {
        return "RobotA";
    }
    return QString::fromStdString(pRobotDriverAdaptor->m_sRobotName);
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
    if (pRobotDriverAdaptor != nullptr && HasMeaningfulToolOffset(pRobotDriverAdaptor->m_tTools.tGunTool))
    {
        if (nameOut != nullptr)
        {
            *nameOut = "GunTool_d";
        }
        return pRobotDriverAdaptor->m_tTools.tGunTool;
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
}

FunctionTestDialog::FunctionTestDialog(ContralUnit* pContralUnit, int unitIndex, CameraFrameCache* cameraCache, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_unitIndex(unitIndex)
    , m_pCameraCache(cameraCache)
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

    QLabel* titleLabel = new QLabel("机器人功能测试区");
    titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #F4FAFA;");
    outerLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("这里集中放置设置速度、读取位置、检查运行、往返运动、零位运动等测试功能；FANUC 专用按钮会在非 FANUC 驱动下禁用。");
    hintLabel->setWordWrap(true);
    outerLayout->addWidget(hintLabel);

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
    QPushButton* uploadLsBtn = CreateTestButton("发送FANUC LS");
    QPushButton* curposDiagBtn = CreateTestButton("FANUC CURPOS诊断");
    QPushButton* timestampDiagBtn = CreateTestButton("机器人+相机时间戳");
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
    const bool initialFanucDriver = dynamic_cast<FANUCRobotCtrl*>(initialDriver) != nullptr;
    uploadLsBtn->setEnabled(initialFanucDriver);
    curposDiagBtn->setEnabled(initialFanucDriver);
    const QString fanucOnlyTip = "该测试依赖 FANUC 常驻服务/LS 程序，当前机器人不是 FANUC 时不可用。";
    uploadLsBtn->setToolTip(initialFanucDriver ? QString() : fanucOnlyTip);
    curposDiagBtn->setToolTip(initialFanucDriver ? QString() : fanucOnlyTip);

    QGroupBox* motionGroup = new QGroupBox("运动测试");
    QGridLayout* motionLayout = new QGridLayout(motionGroup);
    m_pMovlTestBtn = CreateTestButton("MOVL往返测试");
    m_pMovjTestBtn = CreateTestButton("MOVJ J2/J3 +5deg");
    m_pMoveZeroBtn = CreateTestButton("运动到零位");
    motionLayout->addWidget(m_pMovlTestBtn, 0, 0);
    motionLayout->addWidget(m_pMovjTestBtn, 1, 0);
    motionLayout->addWidget(m_pMoveZeroBtn, 2, 0);
    m_motionButtons = { m_pMovlTestBtn, m_pMovjTestBtn, m_pMoveZeroBtn, callJobBtn };
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
    kinematicsLayout->addWidget(saveKinematicsSampleBtn, 0, 0);
    kinematicsLayout->addWidget(fitDhBtn, 1, 0);
    groupLayout->addWidget(kinematicsGroup, 1, 0);
    commandLayout->addStretch(1);
    commandScrollArea->setWidget(m_pCommandContent);
    outerLayout->addWidget(commandScrollArea, 1);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1200);
    m_pLogText->setPlainText("功能测试日志：等待操作...");
    m_pLogText->setMinimumHeight(130);
    m_pLogText->setMaximumHeight(220);
    outerLayout->addWidget(m_pLogText);

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
    QDialog::closeEvent(event);
}

FANUCRobotCtrl* FunctionTestDialog::GetFirstFanucDriver()
{
    if (m_pContralUnit == nullptr || m_unitIndex < 0 || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, "FANUC测试", "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, "FANUC测试", "当前控制单元未创建驱动。");
        return nullptr;
    }

    FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
    if (pFanucDriver == nullptr)
    {
        QMessageBox::warning(this, "FANUC测试", "当前控制单元不是 FANUC 驱动。");
        return nullptr;
    }
    return pFanucDriver;
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
    return m_bFanucMovlRunning || m_bFanucMovjRunning || m_bFanucMoveZeroRunning;
}

void FunctionTestDialog::RefreshMotionButtonState()
{
    bool busy = IsMotionBusy() || m_bRobotCommandRunning || RobotOperationLease::AnyActive();
    if (!busy && m_pContralUnit != nullptr && m_unitIndex >= 0 && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
        RobotDriverAdaptor::StateSnapshot snapshot;
        busy = pRobotDriverAdaptor != nullptr
            && pRobotDriverAdaptor->LatestStateSnapshot(snapshot)
            && snapshot.done == 0;
    }

    for (QPushButton* button : m_motionButtons)
    {
        if (button != nullptr)
        {
            button->setEnabled(!busy);
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
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pFanucDriver, QStringLiteral("功能测试 CURPOS 诊断"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "CURPOS诊断", leaseError);
        return;
    }

    const QStringList commands =
    {
        "GET_USER_PROGRAM",
        "GET_CUR_POS",
        "GET_CUR_POS_00",
        "GET_CUR_POS_001",
        "GET_CUR_POS_01",
        "GET_CUR_POS_10",
        "GET_CUR_POS_11",
        "GET_CUR_POS_011",
        "GET_CUR_POS_111",
        "GET_POS_VAR:20,0"
    };

    QStringList lines;
    for (const QString& command : commands)
    {
        const std::string response = pFanucDriver->SendRawCommandForTest(command.toStdString());
        lines << QString("%1 -> %2").arg(command, DecodeRobotMessageText(response));
    }

    const QString message = lines.join("\n");
    AppendLog("CURPOS诊断:\n" + message);
    QMessageBox::information(this, "CURPOS诊断", message);
}

void FunctionTestDialog::RobotCameraTimestampDiagnosticTest()
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetFirstRobotDriverAdaptor();
    if (pRobotDriverAdaptor == nullptr)
    {
        return;
    }
    pRobotDriverAdaptor->StartStateMonitor(50);

    if (m_pCameraCache == nullptr)
    {
        const QString message = "当前机器人没有可用的专属相机缓存，请确认机器人相机线程已初始化。";
        AppendLog(message);
        QMessageBox::warning(this, "机器人+相机时间戳", message);
        return;
    }

    const QString robotName = DefaultRobotName(pRobotDriverAdaptor);
    CameraFrameCache* cameraCache = m_pCameraCache;
    const std::uint64_t beginCameraSequence = cameraCache->Mark();

    QVector<RobotTimestampSample> robotSamples;
    robotSamples.reserve(512);
    long long lastRobotMs = std::numeric_limits<long long>::min();
    long long lastPcRecvMs = std::numeric_limits<long long>::min();
    std::uint64_t lastRobotSequence = 0;
    int duplicateRobotReadCount = 0;
    int missingRobotTimestampCount = 0;

    AppendLog(QString("机器人+相机时间戳检测开始：采集 %1 ms；FANUC/STEP 使用 robot_ms，其他使用 PC steady ms。")
        .arg(kRobotCameraTimestampCheckDurationMs));
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

        if (snapshot.robotMs > 0 && snapshot.pcRecvMs > 0)
        {
            if (snapshot.sequence != lastRobotSequence
                && (snapshot.robotMs != lastRobotMs || snapshot.pcRecvMs != lastPcRecvMs))
            {
                RobotTimestampSample sample;
                sample.index = robotSamples.size() + 1;
                sample.robotTimestampUs = static_cast<qint64>(snapshot.robotMs) * 1000;
                sample.pcReceiveTimestampUs = static_cast<qint64>(snapshot.pcRecvMs) * 1000;
                sample.pose = snapshot.pose;
                sample.done = snapshot.done;
                robotSamples.push_back(sample);
                lastRobotMs = snapshot.robotMs;
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
        conclusion = "结论：相机timestamp、机器人robot_ms与本机接收时间的总时长比例都接近 1，时间单位假设基本成立。";
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

    const QString resultText = QString(
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

    AppendLog(QString("机器人+相机时间戳检测完成：相机帧=%1，机器人样本=%2，camera/system=%3，robot/system=%4，camera/robot=%5，CSV=%6")
        .arg(cameraFrames.size())
        .arg(robotSamples.size())
        .arg(cameraTotalRatio, 0, 'f', 6)
        .arg(robotTotalRatio, 0, 'f', 6)
        .arg(cameraVsRobotScale, 0, 'f', 6)
        .arg(saveOk ? csvPath : QString("保存失败")));
    QMessageBox::information(this, "机器人+相机时间戳", resultText);
}

void FunctionTestDialog::FanucCheckDoneTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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

    const int readValue = pRobotDriver->GetIntVar(index);
    const QString message = QString("写入 INT%1=%2, 读取值=%3").arg(index).arg(value).arg(readValue);
    AppendLog(message);
    QMessageBox::information(this, "写读INT寄存器", message);
}

void FunctionTestDialog::FanucSetTpSpeedTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const QString jobName = QInputDialog::getText(this, "调用任务", "任务/程序名：", QLineEdit::Normal, "FANUC_PORT_OPEN_TEST", &ok);
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
    AppendLog(QString("已调用任务 %1，正在后台等待真实完成；可返回主页使用“安全停止本软件活动机器人任务”中止。")
        .arg(jobName.trimmed()));
    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pRobotDriver, jobNameBytes, operationLease]()
        {
            const bool callOk = pRobotDriver->CallJob(jobNameBytes.constData());
            const int done = callOk ? pRobotDriver->CheckRobotDone(200) : -1;
            const QString message = QString("调用任务%1：%2，CheckRobotDone=%3")
                .arg(callOk ? QStringLiteral("成功") : QStringLiteral("失败"))
                .arg(QString::fromLocal8Bit(jobNameBytes))
                .arg(done);
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
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    const QString lsPath = FindProjectFilePathForFunctionTest("SDK/FANUC/STARTALL.ls");
    if (lsPath.isEmpty())
    {
        QMessageBox::warning(this, "发送LS程序", "未找到测试程序文件：SDK/FANUC/STARTALL.ls");
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pFanucDriver, QStringLiteral("功能测试上传 LS 程序"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "发送LS程序", leaseError);
        return;
    }

    const QByteArray lsPathBytes = lsPath.toLocal8Bit();
    const int ret = pFanucDriver->UploadLsFile(lsPathBytes.constData());
    const QString message = ret == 0
        ? QString("LS程序发送成功：%1").arg(lsPath)
        : QString("LS程序发送失败，返回码=%1，文件=%2").arg(ret).arg(lsPath);
    AppendLog(message);
    if (ret == 0)
    {
        QMessageBox::information(this, "发送LS程序", message);
    }
    else
    {
        QMessageBox::warning(this, "发送LS程序", message);
    }
}

void FunctionTestDialog::FanucMovlTest()
{
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
            T_ROBOT_COORS target = pRobotDriver->GetCurrentPos();
            target.dY += moveForward ? 100.0 : -100.0;

            const bool moveOk = pRobotDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(5.0, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVL");
            const int done = moveOk ? pRobotDriver->CheckRobotDone(200) : -1;
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
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
            T_ANGLE_PULSE target = pRobotDriver->GetCurrentPulse();
            const double j2PulseUnit = pRobotDriver->m_tAxisUnit.dLPulseUnit;
            const double j3PulseUnit = pRobotDriver->m_tAxisUnit.dUPulseUnit;
            const long j2DeltaPulse = j2PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j2PulseUnit));
            const long j3DeltaPulse = j3PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j3PulseUnit));
            target.nLPulse += j2DeltaPulse;
            target.nUPulse += j3DeltaPulse;

            const bool moveOk = pRobotDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(1.0, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVJ");
            const int done = moveOk ? pRobotDriver->CheckRobotDone(200) : -1;
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
    RobotDriverAdaptor* pRobotDriver = GetFirstRobotDriverAdaptor();
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
            const T_ROBOT_MOVE_SPEED speed(1.0, 0.0, 0.0);
            const bool moveOk = pRobotDriver->MoveByJob(zeroPulse, speed, pRobotDriver->m_nExternalAxleType, "MOVJ");
            const int done = moveOk ? pRobotDriver->CheckRobotDone(200) : -1;
            const T_ROBOT_COORS pos = pRobotDriver->GetCurrentPos();
            const T_ANGLE_PULSE pulse = pRobotDriver->GetCurrentPulse();

            const QString message = QString(
                "MOVJ 到零位, Move=%1, CheckRobotDone=%2\n"
                "当前位置: X=%3, Y=%4, Z=%5, RX=%6, RY=%7, RZ=%8\n"
                "当前脉冲: S=%9, L=%10, U=%11, R=%12, B=%13, T=%14, EX1=%15, EX2=%16, EX3=%17")
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
                .arg(pulse.lBZPulse);

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

    const T_ANGLE_PULSE pulse = pRobotDriverAdaptor->GetCurrentPulse();
    const T_ROBOT_COORS robotPose = pRobotDriverAdaptor->GetCurrentPos();
    QString toolName;
    const T_ROBOT_COORS toolCoors = EffectiveKinematicsTool(pRobotDriverAdaptor, &toolName);
    T_ROBOT_COORS modelPose;
    const bool fkOk = ForwardPoseFromDhParams(
        KinematicsToParamArray(pRobotDriverAdaptor->m_tKinematics),
        pRobotDriverAdaptor->m_tAxisUnit,
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
        pRobotDriverAdaptor->m_tAxisUnit,
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
        pRobotDriverAdaptor->m_tKinematics,
        pRobotDriverAdaptor->m_tAxisUnit,
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

    const std::array<double, kDhParamCount> initialParams = KinematicsToParamArray(pRobotDriverAdaptor->m_tKinematics);
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
        "说明：结果未自动写回 ini，请先核对报告再决定是否替换当前参数。")
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
