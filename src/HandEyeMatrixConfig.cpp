#include "HandEyeMatrixConfig.h"

#include "OPini.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kHandEyePi = 3.14159265358979323846;

QString FindProjectRootPathForHandEye()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        if (QFileInfo::exists(dir.filePath("QtWidgetsApplication4.sln")))
        {
            return QDir::toNativeSeparators(dir.absolutePath());
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return QDir::toNativeSeparators(QDir::currentPath());
}

QString BuildHandEyeIniTemplate(const QString& robotName, const QString& cameraSection, const HandEyeMatrixConfig& config)
{
    QString text;
    QTextStream stream(&text);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(12);
    stream << "[Base]\n";
    stream << "Version=1\n";
    stream << "RobotName=" << robotName << "\n";
    stream << "CameraSection=" << cameraSection << "\n\n";
    stream << "[HandEyeMatrix]\n";
    stream << "# 手眼矩阵参数：机器人局部坐标 = R_opt * 相机坐标 + t_opt\n";
    stream << "# 旋转矩阵\n";
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            stream << QString("R%1%2=").arg(row).arg(col) << config.rotation(row, col) << "\n";
        }
    }
    stream << "\n# 平移向量（mm）\n";
    stream << "T0=" << config.translation(0) << "\n";
    stream << "T1=" << config.translation(1) << "\n";
    stream << "T2=" << config.translation(2) << "\n";
    return text;
}

QString BuildHandEyeCalibrationIniTemplate(const QString& robotName, const QString& cameraSection, const HandEyeCalibrationConfig& config)
{
    QString text;
    QTextStream stream(&text);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(6);
    stream << "[Base]\n";
    stream << "Version=1\n";
    stream << "RobotName=" << robotName << "\n";
    stream << "CameraSection=" << cameraSection << "\n\n";

    stream << "[TcpPoint]\n";
    stream << "# 固定标定目标点，当前版本计算主要使用 XYZ\n";
    stream << "X=" << config.tcpPoint.dX << "\n";
    stream << "Y=" << config.tcpPoint.dY << "\n";
    stream << "Z=" << config.tcpPoint.dZ << "\n";
    stream << "RX=" << config.tcpPoint.dRX << "\n";
    stream << "RY=" << config.tcpPoint.dRY << "\n";
    stream << "RZ=" << config.tcpPoint.dRZ << "\n";
    stream << "BX=" << config.tcpPoint.dBX << "\n";
    stream << "BY=" << config.tcpPoint.dBY << "\n";
    stream << "BZ=" << config.tcpPoint.dBZ << "\n\n";

    for (int index = 0; index < config.samples.size(); ++index)
    {
        const HandEyeCalibrationSample& sample = config.samples[index];
        stream << QString("[Sample%1]\n").arg(index + 1);
        stream << "Valid=" << (sample.valid ? 1 : 0) << "\n";
        stream << "RobotX=" << sample.robotPose.dX << "\n";
        stream << "RobotY=" << sample.robotPose.dY << "\n";
        stream << "RobotZ=" << sample.robotPose.dZ << "\n";
        stream << "RobotRX=" << sample.robotPose.dRX << "\n";
        stream << "RobotRY=" << sample.robotPose.dRY << "\n";
        stream << "RobotRZ=" << sample.robotPose.dRZ << "\n";
        stream << "RobotBX=" << sample.robotPose.dBX << "\n";
        stream << "RobotBY=" << sample.robotPose.dBY << "\n";
        stream << "RobotBZ=" << sample.robotPose.dBZ << "\n";
        stream << "CameraX=" << sample.cameraPoint.x() << "\n";
        stream << "CameraY=" << sample.cameraPoint.y() << "\n";
        stream << "CameraZ=" << sample.cameraPoint.z() << "\n\n";
    }

    return text;
}

Eigen::Matrix3d RotX(double w)
{
    const double c = std::cos(w);
    const double s = std::sin(w);
    return (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                  0.0, c, -s,
                                  0.0, s, c).finished();
}

Eigen::Matrix3d RotY(double p)
{
    const double c = std::cos(p);
    const double s = std::sin(p);
    return (Eigen::Matrix3d() << c, 0.0, s,
                                  0.0, 1.0, 0.0,
                                  -s, 0.0, c).finished();
}

Eigen::Matrix3d RotZ(double r)
{
    const double c = std::cos(r);
    const double s = std::sin(r);
    return (Eigen::Matrix3d() << c, -s, 0.0,
                                  s, c, 0.0,
                                  0.0, 0.0, 1.0).finished();
}

Eigen::Matrix3d FanucRotation(const T_ROBOT_COORS& pose)
{
    const double w = pose.dRX * kHandEyePi / 180.0;
    const double p = pose.dRY * kHandEyePi / 180.0;
    const double r = pose.dRZ * kHandEyePi / 180.0;
    return RotZ(r) * RotY(p) * RotX(w);
}

Eigen::Vector3d ToPositionVector(const T_ROBOT_COORS& pose)
{
    return Eigen::Vector3d(pose.dX, pose.dY, pose.dZ);
}

bool IsFiniteVector(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

bool ReadDoubleValue(COPini& ini, const char* key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

bool ReadIntValue(COPini& ini, const char* key, int& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

bool WriteDoubleValue(COPini& ini, const char* key, double value)
{
    return ini.WriteString(key, value, 6);
}

bool WriteIntValue(COPini& ini, const char* key, int value)
{
    return ini.WriteString(key, value);
}
}

// ===== 默认值 / 文件路径 =====

QString GetMeasureCameraSectionName(const QString& robotName)
{
    COPini cameraIni;
    const std::string cameraParamPath = DATA_PATH + robotName.toStdString() + "\\CameraParam.ini";
    if (!cameraIni.SetFileName(cameraParamPath))
    {
        return "CAMERA0";
    }
    int measureCameraNo = 0;
    cameraIni.SetSectionName("Base");
    cameraIni.ReadString(false, "MeasureCameraNo", &measureCameraNo);
    return QString("CAMERA%1").arg(measureCameraNo);
}

QString GetHandEyeMatrixIniPath(const QString& robotName, const QString& cameraSection)
{
    const QString rootPath = FindProjectRootPathForHandEye();
    return QDir::toNativeSeparators(QDir(rootPath).filePath(QString("Data/%1/HandEyeMatrix_%2.ini").arg(robotName, cameraSection)));
}

QString GetHandEyeCalibrationIniPath(const QString& robotName, const QString& cameraSection)
{
    const QString rootPath = FindProjectRootPathForHandEye();
    return QDir::toNativeSeparators(QDir(rootPath).filePath(QString("Data/%1/HandEyeCalibration_%2.ini").arg(robotName, cameraSection)));
}

HandEyeMatrixConfig GetDefaultHandEyeMatrixConfig()
{
    HandEyeMatrixConfig config;
    config.rotation <<
        0.10801231, 0.98625159, 0.12506452,
        0.96930084, -0.13242660, 0.20716921,
        0.22088283, 0.09884831, -0.97027820;
    config.translation << -43.30622801, -147.37662796, 170.98681778;
    return config;
}

HandEyeCalibrationConfig GetDefaultHandEyeCalibrationConfig()
{
    HandEyeCalibrationConfig config;
    config.samples.resize(kHandEyeCalibrationSampleCount);
    return config;
}

// ===== 文件读写 =====

bool EnsureHandEyeMatrixIni(const QString& robotName, const QString& cameraSection, QString* error, QString* filePathOut)
{
    const QString filePath = GetHandEyeMatrixIniPath(robotName, cameraSection);
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }

    const QFileInfo fileInfo(filePath);
    const QString parentDir = fileInfo.dir().absolutePath();
    if (!QDir().mkpath(parentDir))
    {
        if (error != nullptr)
        {
            *error = QString("创建手眼矩阵目录失败：%1").arg(parentDir);
        }
        return false;
    }

    if (fileInfo.exists())
    {
        return true;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("创建手眼矩阵参数文件失败：%1").arg(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
    stream << BuildHandEyeIniTemplate(robotName, cameraSection, GetDefaultHandEyeMatrixConfig());
    return true;
}

bool LoadHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, HandEyeMatrixConfig& config, QString* error, QString* filePathOut)
{
    QString filePath;
    if (!EnsureHandEyeMatrixIni(robotName, cameraSection, error, &filePath))
    {
        return false;
    }
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }

    COPini ini;
    if (!ini.SetFileName(filePath.toStdString()))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼矩阵参数文件失败：%1").arg(filePath);
        }
        return false;
    }
    ini.SetSectionName("HandEyeMatrix");

    auto readValue = [&ini, error, filePath](const QString& key, double& value) -> bool
    {
        if (ini.ReadString(false, key.toStdString(), &value) <= 0)
        {
            if (error != nullptr)
            {
                *error = QString("读取手眼矩阵参数失败：%1，文件=%2").arg(key, filePath);
            }
            return false;
        }
        return true;
    };

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            if (!readValue(QString("R%1%2").arg(row).arg(col), config.rotation(row, col)))
            {
                return false;
            }
        }
    }
    for (int index = 0; index < 3; ++index)
    {
        if (!readValue(QString("T%1").arg(index), config.translation(index)))
        {
            return false;
        }
    }
    return true;
}

bool SaveHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, const HandEyeMatrixConfig& config, QString* error, QString* filePathOut)
{
    QString filePath;
    if (!EnsureHandEyeMatrixIni(robotName, cameraSection, error, &filePath))
    {
        return false;
    }
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }

    COPini ini;
    if (!ini.SetFileName(filePath.toStdString()))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼矩阵参数文件失败：%1").arg(filePath);
        }
        return false;
    }

    ini.SetSectionName("Base");
    ini.WriteString("Version", 1);
    ini.WriteString("RobotName", robotName.toStdString());
    ini.WriteString("CameraSection", cameraSection.toStdString());
    ini.SetSectionName("HandEyeMatrix");

    auto writeValue = [&ini, error, filePath](const QString& key, double value) -> bool
    {
        if (!ini.WriteString(key.toStdString(), value, 12))
        {
            if (error != nullptr)
            {
                *error = QString("写入手眼矩阵参数失败：%1，文件=%2").arg(key, filePath);
            }
            return false;
        }
        return true;
    };

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            if (!writeValue(QString("R%1%2").arg(row).arg(col), config.rotation(row, col)))
            {
                return false;
            }
        }
    }
    for (int index = 0; index < 3; ++index)
    {
        if (!writeValue(QString("T%1").arg(index), config.translation(index)))
        {
            return false;
        }
    }
    return true;
}

bool EnsureHandEyeCalibrationIni(const QString& robotName, const QString& cameraSection, QString* error, QString* filePathOut)
{
    const QString filePath = GetHandEyeCalibrationIniPath(robotName, cameraSection);
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }

    const QFileInfo fileInfo(filePath);
    const QString parentDir = fileInfo.dir().absolutePath();
    if (!QDir().mkpath(parentDir))
    {
        if (error != nullptr)
        {
            *error = QString("创建手眼标定目录失败：%1").arg(parentDir);
        }
        return false;
    }

    if (fileInfo.exists())
    {
        return true;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("创建手眼标定文件失败：%1").arg(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
    stream << BuildHandEyeCalibrationIniTemplate(robotName, cameraSection, GetDefaultHandEyeCalibrationConfig());
    return true;
}

bool LoadHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, HandEyeCalibrationConfig& config, QString* error, QString* filePathOut)
{
    QString filePath;
    if (!EnsureHandEyeCalibrationIni(robotName, cameraSection, error, &filePath))
    {
        return false;
    }
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }

    config = GetDefaultHandEyeCalibrationConfig();

    COPini ini;
    if (!ini.SetFileName(filePath.toStdString()))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼标定文件失败：%1").arg(filePath);
        }
        return false;
    }

    ini.SetSectionName("TcpPoint");
    ReadDoubleValue(ini, "X", config.tcpPoint.dX);
    ReadDoubleValue(ini, "Y", config.tcpPoint.dY);
    ReadDoubleValue(ini, "Z", config.tcpPoint.dZ);
    ReadDoubleValue(ini, "RX", config.tcpPoint.dRX);
    ReadDoubleValue(ini, "RY", config.tcpPoint.dRY);
    ReadDoubleValue(ini, "RZ", config.tcpPoint.dRZ);
    ReadDoubleValue(ini, "BX", config.tcpPoint.dBX);
    ReadDoubleValue(ini, "BY", config.tcpPoint.dBY);
    ReadDoubleValue(ini, "BZ", config.tcpPoint.dBZ);

    for (int index = 0; index < config.samples.size(); ++index)
    {
        HandEyeCalibrationSample& sample = config.samples[index];
        ini.SetSectionName(QString("Sample%1").arg(index + 1).toStdString());

        int valid = 0;
        ReadIntValue(ini, "Valid", valid);
        sample.valid = (valid != 0);

        ReadDoubleValue(ini, "RobotX", sample.robotPose.dX);
        ReadDoubleValue(ini, "RobotY", sample.robotPose.dY);
        ReadDoubleValue(ini, "RobotZ", sample.robotPose.dZ);
        ReadDoubleValue(ini, "RobotRX", sample.robotPose.dRX);
        ReadDoubleValue(ini, "RobotRY", sample.robotPose.dRY);
        ReadDoubleValue(ini, "RobotRZ", sample.robotPose.dRZ);
        ReadDoubleValue(ini, "RobotBX", sample.robotPose.dBX);
        ReadDoubleValue(ini, "RobotBY", sample.robotPose.dBY);
        ReadDoubleValue(ini, "RobotBZ", sample.robotPose.dBZ);

        double cameraX = 0.0;
        double cameraY = 0.0;
        double cameraZ = 0.0;
        ReadDoubleValue(ini, "CameraX", cameraX);
        ReadDoubleValue(ini, "CameraY", cameraY);
        ReadDoubleValue(ini, "CameraZ", cameraZ);
        sample.cameraPoint = Eigen::Vector3d(cameraX, cameraY, cameraZ);
    }

    return true;
}

bool SaveHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, const HandEyeCalibrationConfig& config, QString* error, QString* filePathOut)
{
    QString filePath;
    if (!EnsureHandEyeCalibrationIni(robotName, cameraSection, error, &filePath))
    {
        return false;
    }
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }

    COPini ini;
    if (!ini.SetFileName(filePath.toStdString()))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼标定文件失败：%1").arg(filePath);
        }
        return false;
    }

    ini.SetSectionName("Base");
    ini.WriteString("Version", 1);
    ini.WriteString("RobotName", robotName.toStdString());
    ini.WriteString("CameraSection", cameraSection.toStdString());

    ini.SetSectionName("TcpPoint");
    if (!(WriteDoubleValue(ini, "X", config.tcpPoint.dX)
        && WriteDoubleValue(ini, "Y", config.tcpPoint.dY)
        && WriteDoubleValue(ini, "Z", config.tcpPoint.dZ)
        && WriteDoubleValue(ini, "RX", config.tcpPoint.dRX)
        && WriteDoubleValue(ini, "RY", config.tcpPoint.dRY)
        && WriteDoubleValue(ini, "RZ", config.tcpPoint.dRZ)
        && WriteDoubleValue(ini, "BX", config.tcpPoint.dBX)
        && WriteDoubleValue(ini, "BY", config.tcpPoint.dBY)
        && WriteDoubleValue(ini, "BZ", config.tcpPoint.dBZ)))
    {
        if (error != nullptr)
        {
            *error = QString("写入固定标定目标点失败：%1").arg(filePath);
        }
        return false;
    }

    const int sampleCount = std::min<int>(static_cast<int>(config.samples.size()), kHandEyeCalibrationSampleCount);
    for (int index = 0; index < sampleCount; ++index)
    {
        const HandEyeCalibrationSample& sample = config.samples[index];
        ini.SetSectionName(QString("Sample%1").arg(index + 1).toStdString());
        if (!(WriteIntValue(ini, "Valid", sample.valid ? 1 : 0)
            && WriteDoubleValue(ini, "RobotX", sample.robotPose.dX)
            && WriteDoubleValue(ini, "RobotY", sample.robotPose.dY)
            && WriteDoubleValue(ini, "RobotZ", sample.robotPose.dZ)
            && WriteDoubleValue(ini, "RobotRX", sample.robotPose.dRX)
            && WriteDoubleValue(ini, "RobotRY", sample.robotPose.dRY)
            && WriteDoubleValue(ini, "RobotRZ", sample.robotPose.dRZ)
            && WriteDoubleValue(ini, "RobotBX", sample.robotPose.dBX)
            && WriteDoubleValue(ini, "RobotBY", sample.robotPose.dBY)
            && WriteDoubleValue(ini, "RobotBZ", sample.robotPose.dBZ)
            && WriteDoubleValue(ini, "CameraX", sample.cameraPoint.x())
            && WriteDoubleValue(ini, "CameraY", sample.cameraPoint.y())
            && WriteDoubleValue(ini, "CameraZ", sample.cameraPoint.z())))
        {
            if (error != nullptr)
            {
                *error = QString("写入第 %1 组手眼标定点失败：%2").arg(index + 1).arg(filePath);
            }
            return false;
        }
    }

    return true;
}

bool ComputeHandEyeMatrixFromCalibration(const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error)
{
    const Eigen::Vector3d tcpWorld = ToPositionVector(calibration.tcpPoint);
    QVector<Eigen::Vector3d> cameraPoints;
    QVector<Eigen::Vector3d> robotLocalPoints;

    for (int index = 0; index < calibration.samples.size(); ++index)
    {
        const HandEyeCalibrationSample& sample = calibration.samples[index];
        if (!sample.valid)
        {
            continue;
        }
        if (!IsFiniteVector(sample.cameraPoint))
        {
            continue;
        }

        const Eigen::Matrix3d robotRotation = FanucRotation(sample.robotPose);
        const Eigen::Vector3d robotWorld = ToPositionVector(sample.robotPose);
        const Eigen::Vector3d robotLocalPoint = robotRotation.transpose() * (tcpWorld - robotWorld);

        if (!IsFiniteVector(robotLocalPoint))
        {
            continue;
        }

        cameraPoints.push_back(sample.cameraPoint);
        robotLocalPoints.push_back(robotLocalPoint);
    }

    if (cameraPoints.size() < 3)
    {
        if (error != nullptr)
        {
            *error = QString("有效标定点不足，当前只有 %1 组，至少需要 3 组。").arg(cameraPoints.size());
        }
        return false;
    }

    Eigen::Vector3d cameraCenter = Eigen::Vector3d::Zero();
    Eigen::Vector3d robotCenter = Eigen::Vector3d::Zero();
    for (int index = 0; index < cameraPoints.size(); ++index)
    {
        cameraCenter += cameraPoints[index];
        robotCenter += robotLocalPoints[index];
    }
    cameraCenter /= static_cast<double>(cameraPoints.size());
    robotCenter /= static_cast<double>(robotLocalPoints.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index = 0; index < cameraPoints.size(); ++index)
    {
        const Eigen::Vector3d cameraDelta = cameraPoints[index] - cameraCenter;
        const Eigen::Vector3d robotDelta = robotLocalPoints[index] - robotCenter;
        covariance += cameraDelta * robotDelta.transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.matrixU().cols() != 3 || svd.matrixV().cols() != 3)
    {
        if (error != nullptr)
        {
            *error = "SVD 求解失败，无法计算手眼矩阵。";
        }
        return false;
    }

    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0.0)
    {
        Eigen::Matrix3d adjustedV = svd.matrixV();
        adjustedV.col(2) *= -1.0;
        rotation = adjustedV * svd.matrixU().transpose();
    }
    const Eigen::Vector3d translation = robotCenter - rotation * cameraCenter;

    if (!IsFiniteVector(translation) || !std::isfinite(rotation(0, 0)))
    {
        if (error != nullptr)
        {
            *error = "手眼矩阵求解结果无效，请检查采集点是否异常。";
        }
        return false;
    }

    config.rotation = rotation;
    config.translation = translation;
    return true;
}
