#include "HandEyeMatrixConfig.h"

#include "OPini.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

namespace
{
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
    stream << "RobotName=" << robotName << "\n\n";
    stream << "CameraSection=" << cameraSection << "\n\n";
    stream << "[HandEyeMatrix]\n";
    stream << "# 手眼矩阵参数：机器人坐标 = R_opt * 相机坐标 + t_opt\n";
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
