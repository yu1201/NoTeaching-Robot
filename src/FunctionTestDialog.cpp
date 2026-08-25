#include "FunctionTestDialog.h"

#include "AppPaths.h"

#include "CameraFrameCache.h"
#include "RobotDataHelper.h"
#include "RobotDriverAdaptor.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"
#include "WindowStyleHelper.h"
#include "../portable/LaserFramePoint3DFilter/LaserFramePoint3DFilter.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
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
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStringConverter>
#include <QStringList>
#include <QTableWidget>
#include <QTabWidget>
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

class KinematicsDraftDialog final : public QDialog
{
public:
    explicit KinematicsDraftDialog(RobotDriverAdaptor* driver, QWidget* parent = nullptr)
        : QDialog(parent)
        , m_driver(driver)
        , m_robotName(DefaultRobotName(driver))
        , m_candidateDirectory(RobotDataHelper::BuildProjectPath(
              QString("Result/KinematicsCandidates/%1").arg(m_robotName)))
    {
        m_filePath = FindLatestCandidatePath();
        setWindowTitle("机器人运动学参数（候选配置）");
        setModal(true);
        resize(1120, 760);

        QVBoxLayout* rootLayout = new QVBoxLayout(this);

        QLabel* safetyLabel = new QLabel(
            "本页面只保存候选参数，不修改当前 RobotPara.ini、不重建运行中的KDL链，也不向机器人写入DH。"
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
                            "不会修改限位、速度、Tool1、Base/Flange或CAD姿态，也不会立即写文件。是否继续？")
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

    QString DraftPath() const
    {
        return m_filePath;
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

        m_toolSourceLabel = new QLabel("来源：尚未读取控制器；当前显示本地GunTool或候选文件值。");
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
        if (!QFileInfo::exists(m_filePath))
        {
            SetStatus(QString(
                "尚无候选文件；已载入SA10说明书中的焊接限位和最大速度。"
                "DH区全零表示待填写，不是有效模型。候选目录：%1")
                .arg(NativeAbsolutePath(m_candidateDirectory)), false);
            return;
        }

        QSettings settings(m_filePath, QSettings::IniFormat);
        const QString storedModel = settings.value("Model/RobotModel").toString().trimmed();
        const QString storedRobotName = settings.value("Model/RobotName").toString().trimmed();
        if (storedModel.compare("SA10/2000H", Qt::CaseInsensitive) != 0
            || storedRobotName.compare(m_robotName, Qt::CaseInsensitive) != 0)
        {
            SetStatus(QString("候选文件与当前目标不匹配，已拒绝加载：%1")
                .arg(NativeAbsolutePath(m_filePath)), true);
            return;
        }
        QString draftError;
        if (!ValidateDraftSettings(settings, &draftError))
        {
            SetStatus(QString("候选文件校验失败：%1；已保留说明书默认页面。文件：%2")
                .arg(draftError, NativeAbsolutePath(m_filePath)), true);
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
        m_dhSource = settings.value("Source/Dh", "CandidateFile_Unspecified").toString();
        m_limitSource = settings.value("Source/JointLimits", "CandidateFile_Unspecified").toString();
        m_speedSource = settings.value("Source/JointSpeed", "CandidateFile_Unspecified").toString();

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
            m_toolPoseSource = settings.value("Source/ToolPose", "CandidateFile_Unspecified").toString();
            m_toolPoseConvention = settings.value("Tool1/PoseConvention", "CandidateFile_Unspecified").toString();
            const QString source = settings.value("Tool1/Source", "候选文件").toString();
            m_toolSourceLabel->setText(QString("来源：%1").arg(source));
        }
        m_toolPoseKnownCheck->setChecked(toolPoseKnown);
        if (!toolPoseKnown)
        {
            m_toolSourceLabel->setText("来源：候选文件没有已人工确认的TCP；当前显示本地GunTool，未作为候选值载入。");
        }

        SetStatus(QString("已加载候选文件：%1；当前状态仍为未验证，并需重新确认当前机器人本体。")
            .arg(NativeAbsolutePath(m_filePath)), false);
        UpdateConventionDescription();
    }

    bool ValidateDraftSettings(QSettings& settings, QString* error) const
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
        QSettings& settings,
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
        QSettings& settings,
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
        QSettings& settings,
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

        if (!QDir().mkpath(m_candidateDirectory))
        {
            if (error != nullptr) *error = "无法创建候选参数目录：" + NativeAbsolutePath(m_candidateDirectory);
            return false;
        }

        const QString savePath = QDir(m_candidateDirectory).filePath(
            QString("KinematicsModelCandidate_%1.ini")
                .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")));
        QSettings settings(savePath, QSettings::IniFormat);
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
        settings.sync();

        if (settings.status() != QSettings::NoError)
        {
            if (error != nullptr) *error = "候选参数写入失败：" + NativeAbsolutePath(savePath);
            return false;
        }
        m_filePath = savePath;
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
        QSettings& settings,
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

    QString FindLatestCandidatePath() const
    {
        const QFileInfoList candidates = QDir(m_candidateDirectory).entryInfoList(
            { "KinematicsModelCandidate_*.ini" },
            QDir::Files | QDir::Readable,
            QDir::Time);
        return candidates.isEmpty() ? QString() : candidates.constFirst().absoluteFilePath();
    }

private:
    RobotDriverAdaptor* m_driver = nullptr;
    QString m_robotName;
    QString m_candidateDirectory;
    QString m_filePath;
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

    QLabel* hintLabel = new QLabel("这里集中放置设置速度、读取位置、原生程序、诊断、往返运动、零位运动等测试功能；入口按当前机器人底层声明的适配能力开放。");
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
    QPushButton* uploadLsBtn = CreateTestButton("发送原生程序");
    QPushButton* curposDiagBtn = CreateTestButton("机器人诊断");
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
		{ RobotDriverCapability::PassiveState }, QStringLiteral("机器人时间戳诊断"));
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
    return m_bFanucMovlRunning || m_bFanucMovjRunning || m_bFanucMoveZeroRunning;
}

void FunctionTestDialog::RefreshMotionButtonState()
{
    bool busy = IsMotionBusy() || m_bRobotCommandRunning || RobotOperationLease::AnyActive();
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
		RobotDriverCapability::PassiveState, QStringLiteral("机器人+相机时间戳"));
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
        this, "调用任务", "任务/程序名（STEP可输入 Project/Program）：",
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
        .arg(NativeAbsolutePath(dialog.DraftPath()));
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
