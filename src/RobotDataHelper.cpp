#include "RobotDataHelper.h"

#include "AppPaths.h"
#include "ConfigDatabase.h"
#include "ContralUnit.h"
#include "OPini.h"
#include "RobotDriverAdaptor.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSaveFile>
#include <QStringList>
#include <QStringConverter>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
QString ToNativeAbsolutePath(const QString& path)
{
    return QDir::toNativeSeparators(QFileInfo(path).absoluteFilePath());
}

QString DecodeConfigTextForRobotData(const std::string& text)
{
    if (text.empty())
    {
        return QString();
    }

    const QByteArray bytes(text.data(), static_cast<int>(text.size()));
#ifdef Q_OS_WIN
    const auto decodeWindowsCodePage = [&bytes](UINT codePage, DWORD flags) -> QString
        {
            const int wideLength = MultiByteToWideChar(
                codePage,
                flags,
                bytes.constData(),
                bytes.size(),
                nullptr,
                0);
            if (wideLength <= 0)
            {
                return QString();
            }
            std::wstring wideText(static_cast<size_t>(wideLength), L'\0');
            MultiByteToWideChar(
                codePage,
                flags,
                bytes.constData(),
                bytes.size(),
                wideText.data(),
                wideLength);
            return QString::fromWCharArray(wideText.data(), wideLength);
        };

    const QString decodedUtf8Text = decodeWindowsCodePage(CP_UTF8, MB_ERR_INVALID_CHARS);
    if (!decodedUtf8Text.isNull() && !decodedUtf8Text.contains(QChar(0xfffd)))
    {
        return decodedUtf8Text;
    }
    const QString gbkText = decodeWindowsCodePage(936, 0);
    if (!gbkText.isNull())
    {
        return gbkText;
    }
#endif
    const QString fallbackUtf8Text = QString::fromUtf8(bytes.constData(), bytes.size());
    if (!fallbackUtf8Text.contains(QChar(0xfffd)))
    {
        return fallbackUtf8Text;
    }
    return QString::fromLocal8Bit(bytes.constData(), bytes.size());
}

std::string EncodeIniTextForRobotData(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

QStringList DefaultScanParamLines()
{
    return QStringList()
        << "#0号测量位置"
        << "#位置软极限"
        << "YMaxCar=0"
        << "YMinCar=0"
        << "YMaxRobot=0"
        << "YMinRobot=0"
        << "XMax=0"
        << "XMin=0"
        << "ZMax=0"
        << "ZMin=0"
        << ""
        << "#速度参数"
        << "ScanSpeed=0"
        << "RunSpeed=0"
        << "CameraTimeOffsetMs=0"
        << "dAcc=0"
        << "dDec=0"
        << "UseComputedScanSafe=0"
        << "ScanSafeOffsetDistanceMm=0"
        << "ScanSafeGunAngleDeg=0"
        << "ScanSafeXDirection=0"
        << "ScanSafeLiftHeightMm=0"
        << "ScanSafeFlipWarnThresholdDeg=0"
        << ""
        << "#下枪安全位置"
        << "StartSafePulseNum=0"
        << "StartSafePulse0.nS=0"
        << "StartSafePulse0.nL=0"
        << "StartSafePulse0.nU=0"
        << "StartSafePulse0.nR=0"
        << "StartSafePulse0.nB=0"
        << "StartSafePulse0.nT=0"
        << "StartSafePulse0.lBX=0"
        << "StartSafePulse0.lBY=0"
        << "StartSafePulse0.lBZ=0"
        << ""
        << "#机器人扫描起点位置姿态"
        << "StartPulse.nS=0"
        << "StartPulse.nL=0"
        << "StartPulse.nU=0"
        << "StartPulse.nR=0"
        << "StartPulse.nB=0"
        << "StartPulse.nT=0"
        << "StartPulse.lBX=0"
        << "StartPulse.lBY=0"
        << "StartPulse.lBZ=0"
        << "StartPos.X=0"
        << "StartPos.Y=0"
        << "StartPos.Z=0"
        << "StartPos.RX=0"
        << "StartPos.RY=0"
        << "StartPos.RZ=0"
        << "StartPos.BX=0"
        << "StartPos.BY=0"
        << "StartPos.BZ=0"
        << ""
        << "#机器人扫描终点位置姿态"
        << "EndPos.X=0"
        << "EndPos.Y=0"
        << "EndPos.Z=0"
        << "EndPos.RX=0"
        << "EndPos.RY=0"
        << "EndPos.RZ=0"
        << "EndPos.BX=0"
        << "EndPos.BY=0"
        << "EndPos.BZ=0"
        << ""
        << "#收枪安全位置"
        << "EndSafePulseNum=0"
        << "EndSafePulse0.nS=0"
        << "EndSafePulse0.nL=0"
        << "EndSafePulse0.nU=0"
        << "EndSafePulse0.nR=0"
        << "EndSafePulse0.nB=0"
        << "EndSafePulse0.nT=0"
        << "EndSafePulse0.lBX=0"
        << "EndSafePulse0.lBY=0"
        << "EndSafePulse0.lBZ=0";
}

QStringList DefaultWeldParamLines()
{
    return QStringList()
        << "#焊接执行"
        << "WeldEnable=1 ;1焊接，0空跑"
        << "WeldSpeedMmPerMin=400 ;焊接速度(mm/min)"
        << "DryRunSpeedMmPerMin=1000 ;空跑速度(mm/min)"
        << "WeldSafeMoveSpeedMmPerMin=1000 ;下枪/收枪安全位移动速度(mm/min)"
        << "StepOverlapRel=20 ;STEP连续过渡比例(OVERLAPREL)"
        << "FinalWeldTrajectoryStepMm=4 ;最终焊接轨迹抽样间距(mm)，只影响生成/下发轨迹点"
        << "ResumeBacktrackDistanceMm=5 ;断点续焊搭接回退距离(mm，按实际执行轨迹弧长)"
        << "WeldDirection=1 ;焊接方向：1起点到终点，-1终点到起点"
        << ""
        << "#坐标和枪角"
        << "WorldCoorDir=0 ;世界Z方向"
        << "RobotInstallDir=0 ;机器人安装方向"
        << "GunAngle=0 ;焊枪角度"
        << "GunLaserAngle=0 ;激光与焊枪夹角"
        << "GunCameraAngle=0 ;相机与焊枪夹角"
        << ""
        << "#焊接姿态"
        << "NormalWeldRx=0 ;平焊RX"
        << "NormalWeldRy=0 ;平焊RY"
        << "UseTaughtWeldPose=0 ;是否启用示教焊接姿态"
        << "TaughtWeldPoseRX=0 ;示教焊接平台标准姿态RX(deg)"
        << "TaughtWeldPoseRY=0 ;示教焊接平台标准姿态RY(deg)"
        << "TaughtWeldPoseRZ=0 ;示教焊接平台标准姿态RZ(deg)"
        << "CornerTransitionLeadDis=0 ;拐点过渡距离(mm)"
        << "WeldStartSkipDis=0 ;起点跳过距离(mm)"
        << "WeldEndSkipDis=0 ;终点跳过距离(mm)"
        << "WeldRzGainDeg=0 ;焊接姿态RZ增益(deg)"
        << "SlopeRzMinDeg=-20 ;爬坡/下坡段RZ负向夹紧(deg)"
        << "SlopeRzMaxDeg=20 ;爬坡/下坡段RZ正向夹紧(deg)";
}

std::string ToUtf8StdString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

QString DefaultIniLineKey(const QString& line)
{
    const int equalPos = line.indexOf('=');
    if (equalPos <= 0)
    {
        return QString();
    }
    return line.left(equalPos).trimmed();
}

QString DefaultIniLineValue(const QString& line)
{
    const int equalPos = line.indexOf('=');
    if (equalPos < 0)
    {
        return QString();
    }

    QString value = line.mid(equalPos + 1).trimmed();
    const int commentPos = value.indexOf(';');
    if (commentPos >= 0)
    {
        value = value.left(commentPos).trimmed();
    }
    return value;
}

bool WriteDefaultLineIfMissing(COPini& ini, const QString& line)
{
    const QString trimmed = line.trimmed();
    if (trimmed.isEmpty() || trimmed.startsWith('#') || trimmed.startsWith(';'))
    {
        return true;
    }

    const QString key = DefaultIniLineKey(line);
    if (key.isEmpty())
    {
        return true;
    }

    const std::string keyText = ToUtf8StdString(key);
    if (ini.CheckExists(keyText))
    {
        return true;
    }

    return ini.WriteString(keyText, ToUtf8StdString(DefaultIniLineValue(line)));
}

bool EnsureDefaultLinesInSection(COPini& ini, const QString& sectionName, const QStringList& lines)
{
    ini.SetSectionName(ToUtf8StdString(sectionName));
    for (const QString& line : lines)
    {
        if (!WriteDefaultLineIfMissing(ini, line))
        {
            return false;
        }
    }
    return true;
}

}

// ===== 工程路径 =====

QString RobotDataHelper::FindProjectRootPath()
{
    return QDir::toNativeSeparators(AppPaths::DataRootPath());
}

QString RobotDataHelper::FindProjectFilePath(const QString& relativePath)
{
    const QString existingPath = AppPaths::FindResourcePath(relativePath);
    return existingPath.isEmpty()
        ? QString()
        : QDir::toNativeSeparators(existingPath);
}

QString RobotDataHelper::BuildProjectPath(const QString& relativePath)
{
    return QDir::toNativeSeparators(AppPaths::WritablePath(relativePath));
}

// ===== 激光点文件 =====

bool RobotDataHelper::LoadIndexedPoint3DFile(
    const QString& filePath,
    QVector<RobotCalculation::IndexedPoint3D>& points,
    QString* error,
    const std::function<bool()>& stopRequested)
{
    points.clear();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (error != nullptr)
        {
            *error = "打开点文件失败: " + ToNativeAbsolutePath(filePath);
        }
        return false;
    }

    // 手写解析 "index x y z"（空格或逗号分隔、可能带引号），用 readLine→QByteArray + strtoll/strtod，
    // 取代 QTextStream 逐行 UTF-8 解码 + 每行 QRegularExpression 编译 + QString 分配——百万行点云读取数量级提速。
    const qint64 totalBytes = std::max<qint64>(1, file.size());
    points.reserve(static_cast<qsizetype>(std::min<qint64>(totalBytes / 40, 8000000)));

    // 分隔符判定（含行尾 '\0'）：空白/逗号/引号/CR/LF。
    auto isSep = [](char c) {
        return c == ' ' || c == '\t' || c == ',' || c == '"' || c == '\r' || c == '\n' || c == '\0';
    };
    // 从 cursor 跳过分隔符后解析一个 double；解析不到返回 false。
    // 注：strtod/strtoll 依赖 C 运行时 locale 的小数点为 '.'（本工程未调 setlocale，恒为 "C"；与 WorkpieceMeshBuilder 快路径前提一致）。
    auto nextNumber = [](const char*& p, double* out) -> bool {
        while (*p == ' ' || *p == '\t' || *p == ',' || *p == '"' || *p == '\r' || *p == '\n') ++p;
        char* end = nullptr;
        const double v = std::strtod(p, &end);
        if (end == p) return false;
        p = end;
        *out = v;
        return true;
    };

    quint64 lineCount = 0;
    while (!file.atEnd())
    {
        if ((lineCount++ & 0x3ffU) == 0U && stopRequested && stopRequested())
        {
            points.clear();
            if (error != nullptr)
            {
                *error = "已取消读取点文件: " + ToNativeAbsolutePath(filePath);
            }
            return false;
        }
        const QByteArray line = file.readLine();

        const char* cursor = line.constData();
        // 跳过外部导出文件首行可能带的 UTF-8 BOM（EF BB BF）；自产文件无 BOM。短路求值，越界安全（constData 以 '\0' 结尾）。
        if (static_cast<unsigned char>(cursor[0]) == 0xEF
            && static_cast<unsigned char>(cursor[1]) == 0xBB
            && static_cast<unsigned char>(cursor[2]) == 0xBF)
        {
            cursor += 3;
        }
        while (*cursor == ' ' || *cursor == '\t' || *cursor == ',' || *cursor == '"' || *cursor == '\r' || *cursor == '\n') ++cursor;
        if (*cursor == '\0')
        {
            continue;  // 空行/纯空白行
        }

        char* afterIndex = nullptr;
        const long long rawIndex = std::strtoll(cursor, &afterIndex, 10);
        // index 必须是整数且紧跟分隔符/行尾（"1.5" 这类非整数 index 视为坏行，避免字段错位）。
        const bool indexOk = (afterIndex != cursor) && isSep(*afterIndex);
        cursor = afterIndex;

        double x = 0.0, y = 0.0, z = 0.0;
        const bool ok = indexOk
            && nextNumber(cursor, &x) && nextNumber(cursor, &y) && nextNumber(cursor, &z)
            && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);

        if (!ok)
        {
            continue;  // 表头/残缺/非数字/非有限值行一律跳过（与 WorkpieceMeshBuilder 快路径一致，最终靠 points.isEmpty 兜底）
        }

        RobotCalculation::IndexedPoint3D point;
        point.index = (rawIndex >= std::numeric_limits<int>::min()
            && rawIndex <= std::numeric_limits<int>::max())
            ? static_cast<int>(rawIndex)
            : points.size() + 1;
        point.point = Eigen::Vector3d(x, y, z);
        points.push_back(point);
    }

    if (points.isEmpty())
    {
        if (error != nullptr)
        {
            *error = "点文件中没有读取到有效点: " + ToNativeAbsolutePath(filePath);
        }
        return false;
    }
    return true;
}

bool RobotDataHelper::SaveTextFileLines(const QString& filePath, const QStringList& lines, QString* error)
{
    QFileInfo info(filePath);
    QDir().mkpath(info.absolutePath());

    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = "保存文本文件失败: " + ToNativeAbsolutePath(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    for (const QString& line : lines)
    {
        stream << line << "\n";
    }
    stream.flush();
    if (stream.status() != QTextStream::Ok || !file.commit())
    {
        if (error != nullptr)
        {
            *error = "原子提交文本文件失败: " + ToNativeAbsolutePath(filePath);
        }
        return false;
    }
    return true;
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
            const QString robotName = DecodeConfigTextForRobotData(
                pDriver != nullptr && !pDriver->m_sRobotName.empty() ? pDriver->m_sRobotName : unitInfo.sUnitName);
            if (robotName.isEmpty())
            {
                continue;
            }

            const QString customName = DecodeConfigTextForRobotData(
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
    if (ini.SetFileName(ToUtf8StdString(CameraParamPath(robotName))))
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
            ini.SetSectionName(ToUtf8StdString(sectionName));

            std::string cameraName;
            const int readResult = ini.ReadString(false, "CameraName", cameraName);

            CameraInfo info;
            info.sectionName = sectionName;
            info.displayName = readResult > 0 && !cameraName.empty()
                ? QString("%1 (%2)").arg(DecodeConfigTextForRobotData(cameraName), sectionName)
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
    if (!ini.SetFileName(ToUtf8StdString(CameraParamPath(robotName))))
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
    if (!ini.SetFileName(ToUtf8StdString(iniPath)))
    {
        if (error != nullptr)
        {
            *error = "读取相机参数失败：打开文件失败 " + iniPath;
        }
        return false;
    }

    param = CameraParamData();
    param.sectionName = cameraSection.isEmpty() ? "CAMERA0" : cameraSection;
    ini.SetSectionName(ToUtf8StdString(param.sectionName));

    std::string textValue;
    double numberValue = 0.0;
    int intValue = 0;

    if (ini.ReadString(false, "DeviceAddress", textValue) > 0)
    {
        param.deviceAddress = DecodeConfigTextForRobotData(textValue);
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
    param.readFps = "100";  // 默认 100fps（≈10ms 轮询）；旧 CameraParam.ini 无此键时回退到该值
    if (ini.ReadString(false, "CameraReadFps", &intValue) > 0 && intValue > 0)
    {
        param.readFps = QString::number(intValue);
    }
    param.imageCaptureEnable = "1";
    if (ini.ReadString(false, "ImageCaptureEnable", &intValue) > 0)
    {
        param.imageCaptureEnable = intValue != 0 ? "1" : "0";
    }
    param.imageCaptureStride = "5";
    if (ini.ReadString(false, "ImageCaptureFrameStride", &intValue) > 0 && intValue > 0)
    {
        param.imageCaptureStride = QString::number(intValue);
    }
    return true;
}

bool RobotDataHelper::SaveCameraParam(const QString& robotName, const CameraParamData& param, QString* error)
{
    const QString iniPath = CameraParamPath(robotName);
    COPini ini;
    if (!ini.SetFileName(ToUtf8StdString(iniPath)))
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
    bool okFps = false;
    int readFps = param.readFps.trimmed().toInt(&okFps);
    if (!okFps || readFps <= 0)
    {
        readFps = 100;  // 读取帧率容错默认，不阻断保存
    }
    const int imageCaptureEnable = param.imageCaptureEnable.trimmed() == "0" ? 0 : 1;
    bool okStride = false;
    int imageCaptureStride = param.imageCaptureStride.trimmed().toInt(&okStride);
    if (!okStride || imageCaptureStride <= 0)
    {
        imageCaptureStride = 5;  // 抽帧间隔容错默认
    }
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

    ini.SetSectionName(ToUtf8StdString(sectionName));
    const bool saveOk =
        ini.WriteString("DeviceAddress", ToUtf8StdString(deviceAddress)) &&
        ini.WriteString("DevicePort", devicePort) &&
        ini.WriteString("ExposureTime", exposureTime, 6) &&
        ini.WriteString("GainLevel", gainLevel, 6) &&
        ini.WriteString("CameraType", cameraType) &&
        ini.WriteString("CameraReadFps", readFps) &&
        ini.WriteString("ImageCaptureEnable", imageCaptureEnable) &&
        ini.WriteString("ImageCaptureFrameStride", imageCaptureStride);

    if (!saveOk && error != nullptr)
    {
        *error = QString("保存相机参数失败：%1 [%2]").arg(iniPath, sectionName);
    }
    return saveOk;
}

// ===== 精测量参数 =====

QString RobotDataHelper::MeasureWeldParamPath(const QString& robotName)
{
    return BuildProjectPath(QString("Data/%1/MeasureWeldParam.ini").arg(robotName));
}

QString RobotDataHelper::MeasureWeldScanSectionName(int groupIndex)
{
    return QString("MeasureGroup%1.Scan").arg(std::max(0, groupIndex));
}

QString RobotDataHelper::MeasureWeldWeldSectionName(int groupIndex)
{
    return QString("MeasureGroup%1.Weld").arg(std::max(0, groupIndex));
}

bool RobotDataHelper::EnsureMeasureWeldParamFile(const QString& robotName, QString* error)
{
    const QString newPath = MeasureWeldParamPath(robotName);
    if (!ConfigDatabase::IsAvailable())
    {
        if (error != nullptr)
        {
            *error = QString("配置库不存在或结构无效，请先运行迁移工具：%1").arg(ConfigDatabase::DatabasePath());
        }
        return false;
    }

    COPini ini;
    if (!ini.SetFileName(false, ToUtf8StdString(newPath)))
    {
        if (error != nullptr)
        {
            *error = QString("打开配置库测量焊接参数失败：%1").arg(newPath);
        }
        return false;
    }

    ini.SetSectionName("MeasureWeldGroups");
    if (!ini.CheckExists("GroupCount"))
    {
        ini.WriteString("GroupCount", 1);
    }
    if (!ini.CheckExists("UseGroupNo"))
    {
        ini.WriteString("UseGroupNo", 0);
    }

    int groupCount = 1;
    ini.ReadString(false, "GroupCount", &groupCount);
    groupCount = std::max(1, groupCount);

    for (int index = 0; index < groupCount; ++index)
    {
        const QString groupNameKey = QString("Group%1Name").arg(index);
        if (!ini.CheckExists(ToUtf8StdString(groupNameKey)))
        {
            ini.WriteString(ToUtf8StdString(groupNameKey), ToUtf8StdString(QString("参数组%1").arg(index + 1)));
        }

        if (!EnsureDefaultLinesInSection(ini, MeasureWeldScanSectionName(index), DefaultScanParamLines())
            || !EnsureDefaultLinesInSection(ini, MeasureWeldWeldSectionName(index), DefaultWeldParamLines()))
        {
            if (error != nullptr)
            {
                *error = QString("写入配置库测量焊接默认参数失败：%1").arg(newPath);
            }
            return false;
        }
    }

    return true;
}

int RobotDataHelper::MeasureWeldCurrentGroupIndex(const QString& robotName, QString* error)
{
    if (!EnsureMeasureWeldParamFile(robotName, error))
    {
        return 0;
    }
    COPini ini;
    const QString path = MeasureWeldParamPath(robotName);
    if (!ini.SetFileName(path.toUtf8().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开测量焊接参数数据失败：" + path;
        }
        return 0;
    }
    int useNo = 0;
    ini.SetSectionName("MeasureWeldGroups");
    ini.ReadString(false, "UseGroupNo", &useNo);
    return std::max(0, useNo);
}

QString RobotDataHelper::MeasureWeldCurrentScanSectionName(const QString& robotName, QString* error)
{
    return MeasureWeldScanSectionName(MeasureWeldCurrentGroupIndex(robotName, error));
}

QString RobotDataHelper::MeasureWeldCurrentWeldSectionName(const QString& robotName, QString* error)
{
    return MeasureWeldWeldSectionName(MeasureWeldCurrentGroupIndex(robotName, error));
}

double RobotDataHelper::ReadActiveFinalWeldStepFallbackMm(const QString& robotName)
{
    constexpr double kDefaultStepMm = 4.0;
    if (robotName.trimmed().isEmpty())
    {
        return kDefaultStepMm;
    }
    const QString section = MeasureWeldCurrentWeldSectionName(robotName);
    if (section.isEmpty())
    {
        return kDefaultStepMm;
    }
    const QMap<QString, QString> values =
        ConfigDatabase::ReadIniSection(MeasureWeldParamPath(robotName), section);
    bool parsed = false;
    const double stepMm = values.value(QStringLiteral("FinalWeldTrajectoryStepMm")).toDouble(&parsed);
    return (parsed && std::isfinite(stepMm) && stepMm > 0.0)
        ? std::clamp(stepMm, 2.0, 100.0)
        : kDefaultStepMm;
}

int RobotDataHelper::ReadActiveWeldDirectionFallback(const QString& robotName)
{
    // 测量焊接参数里的旧 WeldDirection 值：工艺未设置焊接方向时的回退（默认起点到终点）。
    // 读法与先测后焊 LoadPresetParam 一致，保证回退值与管线生效值相同。
    int direction = 1;
    if (robotName.trimmed().isEmpty())
    {
        return 1;
    }
    COPini ini;
    if (ini.SetFileName(MeasureWeldParamPath(robotName).toUtf8().constData()))
    {
        ini.ReadString(false, "WeldDirection", &direction);
    }
    return direction < 0 ? -1 : 1;
}

// ===== 底层 ini 读写 =====

bool RobotDataHelper::ReadPulse(const QString& filePath, const QString& sectionName, const QString& prefix, T_ANGLE_PULSE& pulse, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toUtf8().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数数据失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(ToUtf8StdString(sectionName));

    auto readValue = [&ini, &prefix, error](const QString& suffix, long& value) -> bool
        {
            const QString fullKey = prefix + "." + suffix;
            const int ok = ini.ReadString(ToUtf8StdString(fullKey), &value);
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
        *error += QString("，参数=%1，分组=%2").arg(filePath, sectionName);
    }
    return ok;
}

bool RobotDataHelper::WritePulse(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ANGLE_PULSE& pulse, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toUtf8().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数数据失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(ToUtf8StdString(sectionName));

    auto writeValue = [&ini, &prefix, error](const QString& suffix, long value) -> bool
        {
            const QString fullKey = prefix + "." + suffix;
            if (!ini.WriteString(ToUtf8StdString(fullKey), value))
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
        *error += QString("，参数=%1，分组=%2").arg(filePath, sectionName);
    }
    return ok;
}

bool RobotDataHelper::ReadCoors(const QString& filePath, const QString& sectionName, const QString& prefix, T_ROBOT_COORS& coors, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toUtf8().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数数据失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(ToUtf8StdString(sectionName));

    coors = T_ROBOT_COORS();
    QStringList missingKeys;

    auto readRequired = [&ini, &prefix, &missingKeys](const QString& suffix, double& value)
        {
            const QString key = prefix + "." + suffix;
            if (ini.ReadString(false, ToUtf8StdString(key), &value) <= 0)
            {
                missingKeys << key;
            }
        };
    auto readOptional = [&ini, &prefix](const QString& suffix, double& value)
        {
            const QString key = prefix + "." + suffix;
            ini.ReadString(false, ToUtf8StdString(key), &value);
        };

    readRequired("X", coors.dX);
    readRequired("Y", coors.dY);
    readRequired("Z", coors.dZ);
    readRequired("RX", coors.dRX);
    readRequired("RY", coors.dRY);
    readRequired("RZ", coors.dRZ);
    readOptional("BX", coors.dBX);
    readOptional("BY", coors.dBY);
    readOptional("BZ", coors.dBZ);

    if (!missingKeys.isEmpty())
    {
        if (error != nullptr)
        {
            *error = QString("读取失败：%1，缺少 %2，参数=%3，分组=%4")
                .arg(prefix, missingKeys.join(", "), filePath, sectionName);
        }
        return false;
    }
    return true;
}

bool RobotDataHelper::WriteCoors(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ROBOT_COORS& coors, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toUtf8().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数数据失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(ToUtf8StdString(sectionName));

    if (!ini.WriteString(ToUtf8StdString(prefix + "."), "", coors))
    {
        if (error != nullptr)
        {
            *error = QString("写入失败：%1，参数=%2，分组=%3").arg(prefix, filePath, sectionName);
        }
        return false;
    }
    return true;
}

bool RobotDataHelper::WriteParamValue(const QString& filePath, const QString& sectionName, const QString& key, const QString& value, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(filePath.toUtf8().constData()))
    {
        if (error != nullptr)
        {
            *error = "打开参数数据失败：" + filePath;
        }
        return false;
    }
    ini.SetSectionName(EncodeIniTextForRobotData(sectionName));
    if (!ini.WriteString(EncodeIniTextForRobotData(key), EncodeIniTextForRobotData(value)))
    {
        if (error != nullptr)
        {
            *error = QString("写入失败：%1，参数=%2，分组=%3").arg(key, filePath, sectionName);
        }
        return false;
    }
    return true;
}
