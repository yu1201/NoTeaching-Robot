#include "RobotDataHelper.h"

#include "ContralUnit.h"
#include "OPini.h"
#include "RobotDriverAdaptor.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace
{
QString ToNativeAbsolutePath(const QString& path)
{
    return QDir::toNativeSeparators(QFileInfo(path).absoluteFilePath());
}
}

// ===== 工程路径 =====

QString RobotDataHelper::FindProjectRootPath()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        if (QFileInfo::exists(dir.filePath("QtWidgetsApplication4.sln")))
        {
            return ToNativeAbsolutePath(dir.absolutePath());
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return ToNativeAbsolutePath(QDir::currentPath());
}

QString RobotDataHelper::FindProjectFilePath(const QString& relativePath)
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        const QString candidate = dir.filePath(relativePath);
        if (QFileInfo::exists(candidate))
        {
            return ToNativeAbsolutePath(candidate);
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return QString();
}

QString RobotDataHelper::BuildProjectPath(const QString& relativePath)
{
    const QString root = FindProjectRootPath();
    return ToNativeAbsolutePath(QDir(root).filePath(relativePath));
}

// ===== 机器人列表 / 驱动 =====

QVector<RobotDataHelper::RobotInfo> RobotDataHelper::LoadRobotList(ContralUnit* pContralUnit)
{
    QVector<RobotInfo> robots;
    if (pContralUnit != nullptr)
    {
        for (int index = 0; index < static_cast<int>(pContralUnit->m_vtContralUnitInfo.size()); ++index)
        {
            const T_CONTRAL_UNIT& unitInfo = pContralUnit->m_vtContralUnitInfo[index];
            RobotDriverAdaptor* pDriver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
            const QString robotName = QString::fromStdString(
                pDriver != nullptr && !pDriver->m_sRobotName.empty() ? pDriver->m_sRobotName : unitInfo.sUnitName);
            if (robotName.isEmpty())
            {
                continue;
            }

            const QString customName = QString::fromStdString(
                pDriver != nullptr && !pDriver->m_sCustomName.empty() ? pDriver->m_sCustomName : unitInfo.sChineseName);

            RobotInfo info;
            info.unitIndex = index;
            info.robotName = robotName;
            info.displayName = customName.isEmpty() || customName == robotName
                ? robotName
                : QString("%1 (%2)").arg(customName, robotName);

            bool duplicated = false;
            for (const RobotInfo& existing : robots)
            {
                if (existing.robotName == info.robotName)
                {
                    duplicated = true;
                    break;
                }
            }
            if (!duplicated)
            {
                robots.push_back(info);
            }
        }
    }

    if (robots.isEmpty())
    {
        RobotInfo info;
        info.unitIndex = -1;
        info.robotName = "RobotA";
        info.displayName = "RobotA";
        robots.push_back(info);
    }
    return robots;
}

RobotDriverAdaptor* RobotDataHelper::GetRobotDriver(ContralUnit* pContralUnit, int unitIndex)
{
    if (pContralUnit == nullptr
        || unitIndex < 0
        || unitIndex >= static_cast<int>(pContralUnit->m_vtContralUnitInfo.size()))
    {
        return nullptr;
    }
    return static_cast<RobotDriverAdaptor*>(pContralUnit->m_vtContralUnitInfo[unitIndex].pUnitDriver);
}

// ===== 相机参数 =====

QString RobotDataHelper::CameraParamPath(const QString& robotName)
{
    return BuildProjectPath(QString("Data/%1/CameraParam.ini").arg(robotName));
}

QVector<RobotDataHelper::CameraInfo> RobotDataHelper::LoadCameraList(const QString& robotName, int* pSelectedIndex)
{
    QVector<CameraInfo> cameras;
    int selectedIndex = 0;

    COPini ini;
    if (ini.SetFileName(CameraParamPath(robotName).toStdString()))
    {
        int cameraNum = 0;
        int measureCameraNo = 0;
        ini.SetSectionName("Base");
        ini.ReadString(false, "CameraNum", &cameraNum);
        ini.ReadString(false, "MeasureCameraNo", &measureCameraNo);
        if (cameraNum <= 0)
        {
            cameraNum = 1;
        }

        for (int index = 0; index < cameraNum; ++index)
        {
            const QString sectionName = QString("CAMERA%1").arg(index);
            ini.SetSectionName(sectionName.toStdString());

            std::string cameraName;
            const int readResult = ini.ReadString(false, "CameraName", cameraName);

            CameraInfo info;
            info.sectionName = sectionName;
            info.displayName = readResult > 0 && !cameraName.empty()
                ? QString("%1 (%2)").arg(QString::fromStdString(cameraName), sectionName)
                : sectionName;
            cameras.push_back(info);
        }

        const QString selectedSection = QString("CAMERA%1").arg(measureCameraNo);
        for (int index = 0; index < cameras.size(); ++index)
        {
            if (cameras[index].sectionName == selectedSection)
            {
                selectedIndex = index;
                break;
            }
        }
    }

    if (cameras.isEmpty())
    {
        CameraInfo info;
        info.sectionName = "CAMERA0";
        info.displayName = "CAMERA0";
        cameras.push_back(info);
    }

    if (pSelectedIndex != nullptr)
    {
        *pSelectedIndex = selectedIndex;
    }
    return cameras;
}

QString RobotDataHelper::MeasureCameraSection(const QString& robotName)
{
    COPini ini;
    if (!ini.SetFileName(CameraParamPath(robotName).toStdString()))
    {
        return "CAMERA0";
    }

    int measureCameraNo = 0;
    ini.SetSectionName("Base");
    ini.ReadString(false, "MeasureCameraNo", &measureCameraNo);
    return QString("CAMERA%1").arg(measureCameraNo);
}

bool RobotDataHelper::LoadCameraParam(const QString& robotName, const QString& cameraSection, CameraParamData& param, QString* error)
{
    const QString iniPath = CameraParamPath(robotName);
    COPini ini;
    if (!ini.SetFileName(iniPath.toStdString()))
    {
        if (error != nullptr)
        {
            *error = "读取相机参数失败：打开文件失败 " + iniPath;
        }
        return false;
    }

    param = CameraParamData();
    param.sectionName = cameraSection.isEmpty() ? "CAMERA0" : cameraSection;
    ini.SetSectionName(param.sectionName.toStdString());

    std::string textValue;
    double numberValue = 0.0;
    int intValue = 0;

    if (ini.ReadString(false, "DeviceAddress", textValue) > 0)
    {
        param.deviceAddress = QString::fromStdString(textValue);
    }
    if (ini.ReadString(false, "DevicePort", &intValue) > 0)
    {
        param.devicePort = QString::number(intValue);
    }
    if (ini.ReadString(false, "ExposureTime", &numberValue) > 0)
    {
        param.exposureTime = QString::number(numberValue, 'f', 6);
    }
    if (ini.ReadString(false, "GainLevel", &numberValue) > 0)
    {
        param.gainLevel = QString::number(numberValue, 'f', 6);
    }
    if (ini.ReadString(false, "CameraType", &intValue) > 0)
    {
        param.cameraType = QString::number(intValue);
    }
    return true;
}

bool RobotDataHelper::SaveCameraParam(const QString& robotName, const CameraParamData& param, QString* error)
{
    const QString iniPath = CameraParamPath(robotName);
    COPini ini;
    if (!ini.SetFileName(iniPath.toStdString()))
    {
        if (error != nullptr)
        {
            *error = "保存相机参数失败：打开文件失败 " + iniPath;
        }
        return false;
    }

    bool okPort = false;
    bool okExposure = false;
    bool okGain = false;
    bool okType = false;
    const int devicePort = param.devicePort.trimmed().toInt(&okPort);
    const double exposureTime = param.exposureTime.trimmed().toDouble(&okExposure);
    const double gainLevel = param.gainLevel.trimmed().toDouble(&okGain);
    const int cameraType = param.cameraType.trimmed().toInt(&okType);
    const QString deviceAddress = param.deviceAddress.trimmed();
    const QString sectionName = param.sectionName.isEmpty() ? "CAMERA0" : param.sectionName;

    if (deviceAddress.isEmpty() || !okPort || !okExposure || !okGain || !okType)
    {
        if (error != nullptr)
        {
            *error = "保存相机参数失败：请输入有效的设备IP、端口、曝光、增益和相机类型。";
        }
        return false;
    }

    ini.SetSectionName(sectionName.toStdString());
    const bool saveOk =
        ini.WriteString("DeviceAddress", deviceAddress.toStdString()) &&
        ini.WriteString("DevicePort", devicePort) &&
        ini.WriteString("ExposureTime", exposureTime, 6) &&
        ini.WriteString("GainLevel", gainLevel, 6) &&
        ini.WriteString("CameraType", cameraType);

    if (!saveOk && error != nullptr)
    {
        *error = QString("保存相机参数失败：%1 [%2]").arg(iniPath, sectionName);
    }
    return saveOk;
}

// ===== 精测量参数 =====

QString RobotDataHelper::PreciseMeasureParamPath(const QString& robotName)
{
    return BuildProjectPath(QString("Data/%1/PreciseMeasureParam.ini").arg(robotName));
}

QString RobotDataHelper::PreciseMeasureSectionName(const QString& robotName, QString* error)
{
    const QString path = PreciseMeasureParamPath(robotName);
    COPini ini;
    if (!ini.SetFileName(path.toLocal8Bit().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数文件失败：" + path;
        }
        return QString();
    }

    int useNo = 0;
    ini.SetSectionName("ALLPostion");
    ini.ReadString(false, "UsePostionNo", &useNo);
    return QString("Postion%1").arg(useNo);
}

bool RobotDataHelper::ReadPrecisePulse(const QString& robotName, const QString& prefix, T_ANGLE_PULSE& pulse, QString* error)
{
    const QString filePath = PreciseMeasureParamPath(robotName);
    const QString sectionName = PreciseMeasureSectionName(robotName, error);
    if (filePath.isEmpty() || sectionName.isEmpty())
    {
        return false;
    }
    return ReadPulse(filePath, sectionName, prefix, pulse, error);
}

bool RobotDataHelper::WritePrecisePulse(const QString& robotName, const QString& prefix, const T_ANGLE_PULSE& pulse, QString* error)
{
    const QString filePath = PreciseMeasureParamPath(robotName);
    const QString sectionName = PreciseMeasureSectionName(robotName, error);
    if (filePath.isEmpty() || sectionName.isEmpty())
    {
        return false;
    }
    return WritePulse(filePath, sectionName, prefix, pulse, error);
}

bool RobotDataHelper::ReadPreciseCoors(const QString& robotName, const QString& prefix, T_ROBOT_COORS& coors, QString* error)
{
    const QString filePath = PreciseMeasureParamPath(robotName);
    const QString sectionName = PreciseMeasureSectionName(robotName, error);
    if (filePath.isEmpty() || sectionName.isEmpty())
    {
        return false;
    }
    return ReadCoors(filePath, sectionName, prefix, coors, error);
}

bool RobotDataHelper::WritePreciseCoors(const QString& robotName, const QString& prefix, const T_ROBOT_COORS& coors, QString* error)
{
    const QString filePath = PreciseMeasureParamPath(robotName);
    const QString sectionName = PreciseMeasureSectionName(robotName, error);
    if (filePath.isEmpty() || sectionName.isEmpty())
    {
        return false;
    }
    return WriteCoors(filePath, sectionName, prefix, coors, error);
}

bool RobotDataHelper::WritePreciseParamValue(const QString& robotName, const QString& key, const QString& value, QString* error)
{
    const QString filePath = PreciseMeasureParamPath(robotName);
    const QString sectionName = PreciseMeasureSectionName(robotName, error);
    if (filePath.isEmpty() || sectionName.isEmpty())
    {
        return false;
    }
    return WriteParamValue(filePath, sectionName, key, value, error);
}

// ===== 底层 ini 读写 =====

bool RobotDataHelper::ReadPulse(const QString& filePath, const QString& sectionName, const QString& prefix, T_ANGLE_PULSE& pulse, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toLocal8Bit().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数文件失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(sectionName.toStdString());

    auto readValue = [&ini, &prefix, error](const QString& suffix, long& value) -> bool
        {
            const QString fullKey = prefix + "." + suffix;
            const int ok = ini.ReadString(fullKey.toStdString(), &value);
            if (ok <= 0)
            {
                if (error != nullptr)
                {
                    *error = "读取失败：" + fullKey;
                }
                return false;
            }
            return true;
        };

    const bool ok = readValue("nS", pulse.nSPulse)
        && readValue("nL", pulse.nLPulse)
        && readValue("nU", pulse.nUPulse)
        && readValue("nR", pulse.nRPulse)
        && readValue("nB", pulse.nBPulse)
        && readValue("nT", pulse.nTPulse)
        && readValue("lBX", pulse.lBXPulse)
        && readValue("lBY", pulse.lBYPulse)
        && readValue("lBZ", pulse.lBZPulse);

    if (!ok && error != nullptr)
    {
        *error += QString("，文件=%1，分组=%2").arg(filePath, sectionName);
    }
    return ok;
}

bool RobotDataHelper::WritePulse(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ANGLE_PULSE& pulse, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toLocal8Bit().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数文件失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(sectionName.toStdString());

    auto writeValue = [&ini, &prefix, error](const QString& suffix, long value) -> bool
        {
            const QString fullKey = prefix + "." + suffix;
            if (!ini.WriteString(fullKey.toStdString(), value))
            {
                if (error != nullptr)
                {
                    *error = "写入失败：" + fullKey;
                }
                return false;
            }
            return true;
        };

    const bool ok = writeValue("nS", pulse.nSPulse)
        && writeValue("nL", pulse.nLPulse)
        && writeValue("nU", pulse.nUPulse)
        && writeValue("nR", pulse.nRPulse)
        && writeValue("nB", pulse.nBPulse)
        && writeValue("nT", pulse.nTPulse)
        && writeValue("lBX", pulse.lBXPulse)
        && writeValue("lBY", pulse.lBYPulse)
        && writeValue("lBZ", pulse.lBZPulse);

    if (!ok && error != nullptr)
    {
        *error += QString("，文件=%1，分组=%2").arg(filePath, sectionName);
    }
    return ok;
}

bool RobotDataHelper::ReadCoors(const QString& filePath, const QString& sectionName, const QString& prefix, T_ROBOT_COORS& coors, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toLocal8Bit().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数文件失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(sectionName.toStdString());

    const int ok = ini.ReadString((prefix + ".").toStdString(), "", coors);
    if (ok <= 0)
    {
        if (error != nullptr)
        {
            *error = QString("读取失败：%1，文件=%2，分组=%3").arg(prefix, filePath, sectionName);
        }
        return false;
    }
    return true;
}

bool RobotDataHelper::WriteCoors(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ROBOT_COORS& coors, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toLocal8Bit().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数文件失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(sectionName.toStdString());

    if (!ini.WriteString((prefix + ".").toStdString(), "", coors))
    {
        if (error != nullptr)
        {
            *error = QString("写入失败：%1，文件=%2，分组=%3").arg(prefix, filePath, sectionName);
        }
        return false;
    }
    return true;
}

bool RobotDataHelper::WriteParamValue(const QString& filePath, const QString& sectionName, const QString& key, const QString& value, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toLocal8Bit().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数文件失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(sectionName.toStdString());
    if (!ini.WriteString(key.toStdString(), value.toStdString()))
    {
        if (error != nullptr)
        {
            *error = QString("写入失败：%1，文件=%2，分组=%3").arg(key, filePath, sectionName);
        }
        return false;
    }
    return true;
}
