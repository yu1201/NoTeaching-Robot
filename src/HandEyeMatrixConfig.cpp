#include "HandEyeMatrixConfig.h"

#include "ConfigDatabase.h"
#include "OPini.h"
#include "RobotPoseTransform.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QTextStream>

#include <algorithm>
#include <cmath>

namespace
{
std::string ToUtf8StdString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

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

int ReadRobotTypeForHandEye(const QString& robotName)
{
    COPini ini;
    const std::string robotParaPath = DATA_PATH + ToUtf8StdString(robotName) + "\\RobotPara.ini";
    if (!ini.SetFileName(robotParaPath))
    {
        return ROBOT_TYPE_FANUC;
    }

    int robotType = ROBOT_TYPE_FANUC;
    ini.SetSectionName("BaseParam");
    ini.ReadString(false, "RobotType", &robotType);
    return RobotPoseTransform::NormalizeRobotType(robotType);
}
}

// ===== 默认值 / 文件路径 =====

QString GetMeasureCameraSectionName(const QString& robotName)
{
    COPini cameraIni;
    const std::string cameraParamPath = DATA_PATH + ToUtf8StdString(robotName) + "\\CameraParam.ini";
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
    config.robotType = ROBOT_TYPE_FANUC;
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

    if (!ConfigDatabase::IsAvailable())
    {
        if (error != nullptr)
        {
            *error = QString("配置库不存在或结构无效，请先运行迁移工具：%1").arg(ConfigDatabase::DatabasePath());
        }
        return false;
    }

    const std::string path = ToUtf8StdString(filePath);
    if (ConfigDatabase::HasIniFile(path))
    {
        return true;
    }

    COPini ini;
    if (!ini.SetFileName(false, path))
    {
        if (error != nullptr)
        {
            *error = QString("初始化手眼矩阵参数失败：%1").arg(filePath);
        }
        return false;
    }

    HandEyeMatrixConfig defaultConfig = GetDefaultHandEyeMatrixConfig();
    defaultConfig.robotType = ReadRobotTypeForHandEye(robotName);
    ini.SetSectionName("Base");
    ini.WriteString("Version", 1);
    ini.WriteString("RobotName", ToUtf8StdString(robotName));
    ini.WriteString("CameraSection", ToUtf8StdString(cameraSection));
    ini.WriteString("RobotType", RobotPoseTransform::NormalizeRobotType(defaultConfig.robotType));
    ini.WriteString("Calibrated", 0);  // 自动创建的默认矩阵不能用于真实扫描，必须经界面显式保存。

    ini.SetSectionName("HandEyeMatrix");
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            ini.WriteString(ToUtf8StdString(QString("R%1%2").arg(row).arg(col)), defaultConfig.rotation(row, col), 12);
        }
    }
    for (int index = 0; index < 3; ++index)
    {
        ini.WriteString(ToUtf8StdString(QString("T%1").arg(index)), defaultConfig.translation(index), 12);
    }
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
    if (!ini.SetFileName(ToUtf8StdString(filePath)))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼矩阵参数数据失败：%1").arg(filePath);
        }
        return false;
    }
    config.robotType = ReadRobotTypeForHandEye(robotName);
    ini.SetSectionName("HandEyeMatrix");

    auto readValue = [&ini, error, filePath](const QString& key, double& value) -> bool
    {
        if (ini.ReadString(false, ToUtf8StdString(key), &value) <= 0)
        {
            if (error != nullptr)
            {
                *error = QString("读取手眼矩阵参数失败：%1，参数=%2").arg(key, filePath);
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

bool LoadExistingValidatedHandEyeMatrixConfig(
    const QString& robotName,
    const QString& cameraSection,
    HandEyeMatrixConfig& config,
    QString* error,
    QString* filePathOut)
{
    const QString filePath = GetHandEyeMatrixIniPath(robotName, cameraSection);
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }
    const std::string path = ToUtf8StdString(filePath);
    if (!ConfigDatabase::IsAvailable() || !ConfigDatabase::HasIniFile(path))
    {
        if (error != nullptr)
        {
            *error = QString("手眼矩阵记录不存在，禁止使用自动默认值：%1").arg(filePath);
        }
        return false;
    }

    COPini metadataIni;
    if (!metadataIni.SetFileName(path))
    {
        if (error != nullptr)
        {
            *error = QString("无法打开手眼矩阵记录：%1").arg(filePath);
        }
        return false;
    }
    metadataIni.SetSectionName("Base");
    std::string storedRobotName;
    std::string storedCameraSection;
    if (metadataIni.ReadString(false, "RobotName", storedRobotName) <= 0
        || QString::fromUtf8(storedRobotName.c_str()) != robotName)
    {
        if (error != nullptr)
        {
            *error = QString("手眼矩阵机器人标识不匹配：期望=%1，文件=%2")
                .arg(robotName, filePath);
        }
        return false;
    }
    if (metadataIni.ReadString(false, "CameraSection", storedCameraSection) <= 0
        || QString::fromUtf8(storedCameraSection.c_str()) != cameraSection)
    {
        if (error != nullptr)
        {
            *error = QString("手眼矩阵相机分组不匹配：期望=%1，文件=%2")
                .arg(cameraSection, filePath);
        }
        return false;
    }

    int calibrated = 0;
    const bool hasCalibratedFlag = metadataIni.ReadString(false, "Calibrated", &calibrated) > 0;
    if (!LoadHandEyeMatrixConfig(robotName, cameraSection, config, error, filePathOut))
    {
        return false;
    }

    if (!config.rotation.allFinite() || !config.translation.allFinite())
    {
        if (error != nullptr)
        {
            *error = QString("手眼矩阵包含 NaN 或无穷值：%1").arg(filePath);
        }
        return false;
    }
    const Eigen::Matrix3d orthogonality = config.rotation.transpose() * config.rotation;
    const double orthogonalityError =
        (orthogonality - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    const double determinant = config.rotation.determinant();
    if (orthogonalityError > 0.02 || std::abs(determinant - 1.0) > 0.02)
    {
        if (error != nullptr)
        {
            *error = QString("手眼旋转矩阵无效：正交误差=%1，det=%2，文件=%3")
                .arg(orthogonalityError, 0, 'g', 8)
                .arg(determinant, 0, 'g', 8)
                .arg(filePath);
        }
        return false;
    }

    if (!hasCalibratedFlag || calibrated == 0)
    {
        // 兼容已有现场数据：旧版本没有 Calibrated 字段，只接受显式存在的 HandEyeReady=1，
        // 且矩阵不能仍是 Ensure 自动写入的硬编码默认值。重新在界面保存后会写入 Calibrated=1。
        COPini robotIni;
        const std::string robotParaPath = DATA_PATH + ToUtf8StdString(robotName) + "\\RobotPara.ini";
        int handEyeReady = 0;
        if (!robotIni.SetFileName(robotParaPath)
            || !robotIni.SetSectionName("SetupStatus")
            || robotIni.ReadString(false, "HandEyeReady", &handEyeReady) <= 0
            || handEyeReady == 0)
        {
            if (error != nullptr)
            {
                *error = QString("手眼矩阵尚未明确标记为已标定：%1").arg(filePath);
            }
            return false;
        }
        const HandEyeMatrixConfig defaultConfig = GetDefaultHandEyeMatrixConfig();
        const double defaultRotationDelta =
            (config.rotation - defaultConfig.rotation).cwiseAbs().maxCoeff();
        const double defaultTranslationDelta =
            (config.translation - defaultConfig.translation).cwiseAbs().maxCoeff();
        if (defaultRotationDelta < 1e-9 && defaultTranslationDelta < 1e-9)
        {
            if (error != nullptr)
            {
                *error = QString("手眼矩阵仍是自动生成的默认值，请在标定/矩阵界面确认并保存：%1")
                    .arg(filePath);
            }
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
    if (!ini.SetFileName(ToUtf8StdString(filePath)))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼矩阵参数数据失败：%1").arg(filePath);
        }
        return false;
    }

    ini.SetSectionName("Base");
    ini.WriteString("Version", 1);
    ini.WriteString("RobotName", ToUtf8StdString(robotName));
    ini.WriteString("CameraSection", ToUtf8StdString(cameraSection));
    ini.WriteString("RobotType", RobotPoseTransform::NormalizeRobotType(config.robotType));
    ini.WriteString("Calibrated", 1);
    ini.SetSectionName("HandEyeMatrix");

    auto writeValue = [&ini, error, filePath](const QString& key, double value) -> bool
    {
        if (!ini.WriteString(ToUtf8StdString(key), value, 12))
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

    if (!ConfigDatabase::IsAvailable())
    {
        if (error != nullptr)
        {
            *error = QString("配置库不存在或结构无效，请先运行迁移工具：%1").arg(ConfigDatabase::DatabasePath());
        }
        return false;
    }

    const std::string path = ToUtf8StdString(filePath);
    if (ConfigDatabase::HasIniFile(path))
    {
        return true;
    }

    COPini ini;
    if (!ini.SetFileName(false, path))
    {
        if (error != nullptr)
        {
            *error = QString("初始化手眼标定参数失败：%1").arg(filePath);
        }
        return false;
    }

    const HandEyeCalibrationConfig defaultConfig = GetDefaultHandEyeCalibrationConfig();
    ini.SetSectionName("Base");
    ini.WriteString("Version", 1);
    ini.WriteString("RobotName", ToUtf8StdString(robotName));
    ini.WriteString("CameraSection", ToUtf8StdString(cameraSection));

    ini.SetSectionName("TcpPoint");
    WriteDoubleValue(ini, "X", defaultConfig.tcpPoint.dX);
    WriteDoubleValue(ini, "Y", defaultConfig.tcpPoint.dY);
    WriteDoubleValue(ini, "Z", defaultConfig.tcpPoint.dZ);
    WriteDoubleValue(ini, "RX", defaultConfig.tcpPoint.dRX);
    WriteDoubleValue(ini, "RY", defaultConfig.tcpPoint.dRY);
    WriteDoubleValue(ini, "RZ", defaultConfig.tcpPoint.dRZ);
    WriteDoubleValue(ini, "BX", defaultConfig.tcpPoint.dBX);
    WriteDoubleValue(ini, "BY", defaultConfig.tcpPoint.dBY);
    WriteDoubleValue(ini, "BZ", defaultConfig.tcpPoint.dBZ);

    for (int index = 0; index < defaultConfig.samples.size(); ++index)
    {
        const HandEyeCalibrationSample& sample = defaultConfig.samples[index];
        ini.SetSectionName(ToUtf8StdString(QString("Sample%1").arg(index + 1)));
        WriteIntValue(ini, "Valid", sample.valid ? 1 : 0);
        WriteDoubleValue(ini, "RobotX", sample.robotPose.dX);
        WriteDoubleValue(ini, "RobotY", sample.robotPose.dY);
        WriteDoubleValue(ini, "RobotZ", sample.robotPose.dZ);
        WriteDoubleValue(ini, "RobotRX", sample.robotPose.dRX);
        WriteDoubleValue(ini, "RobotRY", sample.robotPose.dRY);
        WriteDoubleValue(ini, "RobotRZ", sample.robotPose.dRZ);
        WriteDoubleValue(ini, "RobotBX", sample.robotPose.dBX);
        WriteDoubleValue(ini, "RobotBY", sample.robotPose.dBY);
        WriteDoubleValue(ini, "RobotBZ", sample.robotPose.dBZ);
        WriteDoubleValue(ini, "CameraX", sample.cameraPoint.x());
        WriteDoubleValue(ini, "CameraY", sample.cameraPoint.y());
        WriteDoubleValue(ini, "CameraZ", sample.cameraPoint.z());
    }
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
    if (!ini.SetFileName(ToUtf8StdString(filePath)))
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
        ini.SetSectionName(ToUtf8StdString(QString("Sample%1").arg(index + 1)));

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
    if (!ini.SetFileName(ToUtf8StdString(filePath)))
    {
        if (error != nullptr)
        {
            *error = QString("打开手眼标定文件失败：%1").arg(filePath);
        }
        return false;
    }

    ini.SetSectionName("Base");
    ini.WriteString("Version", 1);
    ini.WriteString("RobotName", ToUtf8StdString(robotName));
    ini.WriteString("CameraSection", ToUtf8StdString(cameraSection));

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
        ini.SetSectionName(ToUtf8StdString(QString("Sample%1").arg(index + 1)));
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
    return ComputeHandEyeMatrixFromCalibration(ROBOT_TYPE_FANUC, calibration, config, error);
}

bool ComputeHandEyeMatrixFromCalibration(const QString& robotName, const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error)
{
    return ComputeHandEyeMatrixFromCalibration(ReadRobotTypeForHandEye(robotName), calibration, config, error);
}

bool ComputeHandEyeMatrixFromCalibration(int robotType, const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error)
{
    const int normalizedRobotType = RobotPoseTransform::NormalizeRobotType(robotType);
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

        const Eigen::Matrix3d robotRotation =
            RobotPoseTransform::RotationFromPose(sample.robotPose, normalizedRobotType);
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
    config.robotType = normalizedRobotType;
    return true;
}
