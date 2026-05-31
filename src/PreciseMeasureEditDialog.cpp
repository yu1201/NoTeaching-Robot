#include "PreciseMeasureEditDialog.h"

#include "ConfigDatabase.h"
#include "FANUCRobotDriver.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QCheckBox>
#include <QComboBox>
#include <QByteArray>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QDoubleValidator>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSet>
#include <QSizePolicy>
#include <QSplitter>
#include <QStringList>
#include <QSignalBlocker>
#include <QTextDocument>
#include <QTextStream>
#include <QTimer>
#include <QVariant>
#include <QIntValidator>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
QString ProjectRootPathForPreciseEdit()
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

QString AxisKey(const QString& group, const QString& axis)
{
    return group + "." + axis;
}

constexpr int kShortValueEditMaxWidth = 118;
constexpr int kOtherParamLabelWidth = 126;
constexpr int kOtherParamEditorMinWidth = 150;
constexpr int kOtherParamEditorMaxWidth = 260;
constexpr auto USE_TAUGHT_WELD_POSE_KEY = "UseTaughtWeldPose";
constexpr auto TAUGHT_WELD_POSE_RX_KEY = "TaughtWeldPoseRX";
constexpr auto TAUGHT_WELD_POSE_RY_KEY = "TaughtWeldPoseRY";
constexpr auto TAUGHT_WELD_POSE_RZ_KEY = "TaughtWeldPoseRZ";
constexpr auto NORMAL_WELD_RX_KEY = "NormalWeldRx";
constexpr auto NORMAL_WELD_RY_KEY = "NormalWeldRy";
constexpr auto WELD_RZ_GAIN_KEY = "WeldRzGainDeg";

QLineEdit* CreateValueEdit(int minWidth = 76, int maxWidth = kShortValueEditMaxWidth)
{
    QLineEdit* edit = new QLineEdit();
    edit->setMinimumWidth(minWidth);
    edit->setMaximumWidth(maxWidth);
    edit->setAlignment(Qt::AlignRight);
    return edit;
}

QWidget* CreateUnitEditorForPrecise(QWidget* editor, const QString& unitText)
{
    QWidget* container = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);
    layout->addWidget(editor, 1);

    QLabel* unitLabel = new QLabel(unitText);
    unitLabel->setObjectName("UnitLabel");
    unitLabel->setMinimumWidth(unitText.compare("deg", Qt::CaseInsensitive) == 0 ? 34 : 28);
    unitLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    layout->addWidget(unitLabel);
    return container;
}

void MarkNumericEdit(QLineEdit* edit)
{
    if (edit == nullptr)
    {
        return;
    }
    edit->setProperty("touchKeyboardLayout", QStringLiteral("numeric"));
    edit->setInputMethodHints(Qt::ImhFormattedNumbersOnly);
}

QDoubleValidator* CreateDoubleValidator(QObject* parent)
{
    QDoubleValidator* validator = new QDoubleValidator(-1.0e12, 1.0e12, 6, parent);
    validator->setNotation(QDoubleValidator::StandardNotation);
    return validator;
}

QIntValidator* CreateIntValidator(QObject* parent)
{
    return new QIntValidator(std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), parent);
}

bool LooksNumericValue(const QString& value)
{
    const QString trimmed = value.trimmed();
    if (trimmed.isEmpty())
    {
        return true;
    }

    bool ok = false;
    trimmed.toDouble(&ok);
    return ok;
}

bool IsDedicatedPulseKey(const QString& key)
{
    return key.startsWith("StartPulse.", Qt::CaseInsensitive)
        || key.startsWith("StartPos.", Qt::CaseInsensitive)
        || key.startsWith("EndPulse.", Qt::CaseInsensitive)
        || key.startsWith("EndPos.", Qt::CaseInsensitive)
        || key.startsWith("StartSafePulse0.", Qt::CaseInsensitive)
        || key.startsWith("EndSafePulse0.", Qt::CaseInsensitive)
        || key.compare("StartSafePulseNum", Qt::CaseInsensitive) == 0
        || key.compare("EndSafePulseNum", Qt::CaseInsensitive) == 0
        || key.compare("UseComputedScanSafe", Qt::CaseInsensitive) == 0
        || key.compare("ScanSafeOffsetDistanceMm", Qt::CaseInsensitive) == 0
        || key.compare("ScanSafeGunAngleDeg", Qt::CaseInsensitive) == 0
        || key.compare("ScanSafeXDirection", Qt::CaseInsensitive) == 0
        || key.compare("ScanSafeLiftHeightMm", Qt::CaseInsensitive) == 0
        || key.compare("ScanSafeFlipWarnThresholdDeg", Qt::CaseInsensitive) == 0;
}

bool HasMeaningfulPulse(const T_ANGLE_PULSE& pulse)
{
    return pulse.nSPulse != 0
        || pulse.nLPulse != 0
        || pulse.nUPulse != 0
        || pulse.nRPulse != 0
        || pulse.nBPulse != 0
        || pulse.nTPulse != 0
        || pulse.lBXPulse != 0
        || pulse.lBYPulse != 0
        || pulse.lBZPulse != 0;
}

QString PulseListCountValue(const T_ANGLE_PULSE& pulse)
{
    return HasMeaningfulPulse(pulse) ? QStringLiteral("1") : QStringLiteral("0");
}

QStringList MinimalScanSectionLinesForPrecise()
{
    QStringList lines;
    lines
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
        << "EndPos.X=0"
        << "EndPos.Y=0"
        << "EndPos.Z=0"
        << "EndPos.RX=0"
        << "EndPos.RY=0"
        << "EndPos.RZ=0"
        << "EndPos.BX=0"
        << "EndPos.BY=0"
        << "EndPos.BZ=0"
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
    return lines;
}

QStringList MinimalWeldSectionLinesForPrecise()
{
    QStringList lines;
    lines
        << "WeldSafeMoveSpeedMmPerMin=1000"
        << "StepOverlapRel=20"
        << "WeldDirection=1"
        << "NormalWeldRx=0"
        << "NormalWeldRy=0"
        << "UseTaughtWeldPose=0"
        << "TaughtWeldPoseRX=0"
        << "TaughtWeldPoseRY=0"
        << "TaughtWeldPoseRZ=0"
        << "CornerTransitionLeadDis=0"
        << "WeldStartSkipDis=0"
        << "WeldEndSkipDis=0"
        << "WeldRzGainDeg=0"
        << "SlopeRzMinDeg=-20"
        << "SlopeRzMaxDeg=20";
    return lines;
}

constexpr auto CAMERA_READ_FPS_KEY = "CameraReadFps";
constexpr auto CAMERA_TIME_OFFSET_MS_KEY = "CameraTimeOffsetMs";
constexpr double DEFAULT_CAMERA_READ_FPS = 100.0;
constexpr double DEFAULT_CAMERA_TIME_OFFSET_MS = -300.0;

QString GroupMetaSectionName()
{
    return QStringLiteral("MeasureWeldGroups");
}

QString EditorId(const QString& sectionName, const QString& key)
{
    return sectionName + "\t" + key;
}

QString DisplayValueFromRawIniValue(const QString& rawValue, QString* inlineComment)
{
    const int commentPos = rawValue.indexOf(';');
    if (commentPos < 0)
    {
        if (inlineComment != nullptr)
        {
            inlineComment->clear();
        }
        return rawValue.trimmed();
    }

    if (inlineComment != nullptr)
    {
        *inlineComment = rawValue.mid(commentPos).trimmed();
    }
    return rawValue.left(commentPos).trimmed();
}

QString ValueForWriteWithInlineComment(QLineEdit* edit)
{
    if (edit == nullptr)
    {
        return QString();
    }

    const QString value = edit->text().trimmed();
    const QString inlineComment = edit->property("inlineComment").toString().trimmed();
    if (inlineComment.isEmpty() || value.contains(';'))
    {
        return value;
    }
    return value + " " + inlineComment;
}

QString DecodeIniTextForPrecise(const std::string& text);

std::string ToUtf8StdStringForPrecise(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

QString DecodeIniTextForPrecise(const std::string& text)
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

    const QString decodedUtf8 = decodeWindowsCodePage(CP_UTF8, MB_ERR_INVALID_CHARS);
    if (!decodedUtf8.isNull() && !decodedUtf8.contains(QChar(0xfffd)))
    {
        return decodedUtf8;
    }
    const QString decodedGbk = decodeWindowsCodePage(936, 0);
    if (!decodedGbk.isNull())
    {
        return decodedGbk;
    }
#endif
    QString decoded = QString::fromUtf8(bytes);
    if (!decoded.contains(QChar(0xfffd)))
    {
        return decoded;
    }
    return QString::fromLocal8Bit(bytes.constData(), bytes.size());
}

QByteArray EncodeUtf16LeForPrecise(const QString& text, bool includeBom)
{
    QByteArray bytes;
    bytes.reserve((includeBom ? 2 : 0) + text.size() * 2);
    if (includeBom)
    {
        bytes.append(char(0xFF));
        bytes.append(char(0xFE));
    }
    for (const QChar ch : text)
    {
        const ushort codeUnit = ch.unicode();
        bytes.append(static_cast<char>(codeUnit & 0xFF));
        bytes.append(static_cast<char>((codeUnit >> 8) & 0xFF));
    }
    return bytes;
}

QByteArray EncodeIniTextForPrecise(const QString& text)
{
    return EncodeUtf16LeForPrecise(text, true);
}

bool IsObsoletePreciseParamKey(const QString& key)
{
    return key.trimmed().compare("CornerArcRadiusMm", Qt::CaseInsensitive) == 0;
}

bool IsDedicatedWeldPoseTeachKey(const QString& key)
{
    const QString normalized = key.trimmed();
    return normalized.compare(USE_TAUGHT_WELD_POSE_KEY, Qt::CaseInsensitive) == 0
        || normalized.compare(TAUGHT_WELD_POSE_RX_KEY, Qt::CaseInsensitive) == 0
        || normalized.compare(TAUGHT_WELD_POSE_RY_KEY, Qt::CaseInsensitive) == 0
        || normalized.compare(TAUGHT_WELD_POSE_RZ_KEY, Qt::CaseInsensitive) == 0
        || normalized.compare(NORMAL_WELD_RX_KEY, Qt::CaseInsensitive) == 0
        || normalized.compare(NORMAL_WELD_RY_KEY, Qt::CaseInsensitive) == 0
        || normalized.compare(WELD_RZ_GAIN_KEY, Qt::CaseInsensitive) == 0;
}

bool IsWeldDirectionParamKey(const QString& key)
{
    return key.trimmed().compare("WeldDirection", Qt::CaseInsensitive) == 0;
}

QStringList ExtractSectionLinesForPrecise(const QString& content, const QString& sectionName)
{
    QStringList result;
    bool inSection = false;
    const QStringList lines = content.split('\n');
    for (const QString& line : lines)
    {
        const QString trimmed = line.trimmed();
        if (trimmed.startsWith('[') && trimmed.endsWith(']'))
        {
            const QString currentSection = trimmed.mid(1, trimmed.size() - 2).trimmed();
            if (inSection && currentSection.compare(sectionName, Qt::CaseInsensitive) != 0)
            {
                break;
            }
            inSection = currentSection.compare(sectionName, Qt::CaseInsensitive) == 0;
            continue;
        }
        if (inSection)
        {
            const int equalPos = line.indexOf('=');
            if (equalPos > 0 && IsObsoletePreciseParamKey(line.left(equalPos)))
            {
                continue;
            }
            result << line;
        }
    }
    while (!result.isEmpty() && result.last().trimmed().isEmpty())
    {
        result.removeLast();
    }
    return result;
}

QStringList ZeroSectionValuesForPrecise(const QStringList& lines)
{
    QStringList result;
    for (const QString& line : lines)
    {
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty() || trimmed.startsWith('#') || trimmed.startsWith(';'))
        {
            result << line;
            continue;
        }
        const int equalPos = line.indexOf('=');
        if (equalPos <= 0)
        {
            result << line;
            continue;
        }
        if (IsObsoletePreciseParamKey(line.left(equalPos)))
        {
            continue;
        }
        result << QString("%1=0").arg(line.left(equalPos).trimmed());
    }
    return result;
}

QMap<QString, QString> SectionMapFromLinesForPrecise(const QStringList& lines)
{
    QMap<QString, QString> values;
    for (const QString& line : lines)
    {
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty() || trimmed.startsWith('#') || trimmed.startsWith(';'))
        {
            continue;
        }
        const int equalPos = line.indexOf('=');
        if (equalPos <= 0)
        {
            continue;
        }
        const QString key = line.left(equalPos).trimmed();
        if (key.isEmpty() || IsObsoletePreciseParamKey(key))
        {
            continue;
        }
        values.insert(key, line.mid(equalPos + 1).trimmed());
    }
    return values;
}

QMap<QString, QString> ZeroSectionValuesForPrecise(const QMap<QString, QString>& values)
{
    QMap<QString, QString> result;
    for (auto it = values.constBegin(); it != values.constEnd(); ++it)
    {
        if (!IsObsoletePreciseParamKey(it.key()))
        {
            result.insert(it.key(), "0");
        }
    }
    return result;
}

QMap<QString, QString> ReadSectionMapForPrecise(
    const QString& path,
    const QString& sectionName,
    const QStringList& fallbackLines)
{
    QMap<QString, QString> values = ConfigDatabase::ReadIniSection(path, sectionName);
    if (values.isEmpty())
    {
        values = SectionMapFromLinesForPrecise(fallbackLines);
    }
    for (auto it = values.begin(); it != values.end(); )
    {
        if (IsObsoletePreciseParamKey(it.key()))
        {
            it = values.erase(it);
        }
        else
        {
            ++it;
        }
    }
    return values;
}

bool WriteSectionMapForPrecise(
    const QString& path,
    const QString& sectionName,
    const QMap<QString, QString>& values,
    QString& error)
{
    if (!ConfigDatabase::RemoveIniSection(path, sectionName))
    {
        error = QString("清理参数组失败：%1 [%2]").arg(path, sectionName);
        return false;
    }
    for (auto it = values.constBegin(); it != values.constEnd(); ++it)
    {
        if (IsObsoletePreciseParamKey(it.key()))
        {
            continue;
        }
        if (!RobotDataHelper::WriteParamValue(path, sectionName, it.key(), it.value(), &error))
        {
            return false;
        }
    }
    return true;
}

QString PreciseParamDisplayName(const QString& key)
{
    static const QMap<QString, QString> names = {
        { "ALLPostionNum", "位置总数" },
        { "UsePostionNo", "当前使用位置号" },
        { "StartSafePulseNum", "下枪安全位置数量" },
        { "EndSafePulseNum", "收枪安全位置数量" },
        { "ScanStartCarLoction", "大车扫描起始位置" },
        { "ScanEndtCarLoction", "大车扫描结束位置" },
        { "ScanLength", "扫描长度" },
        { "ScanSpeed", "扫描速度" },
        { "RunSpeed", "运行速度" },
        { "Acc", "加速度" },
        { "Dec", "减速度" },
        { "dAcc", "加速度系数" },
        { "dDec", "减速度系数" },
        { "TableY", "料台Y坐标" },
        { "TableZ", "料台Z坐标" },
        { "Range_XMax", "X最大范围" },
        { "Range_XMin", "X最小范围" },
        { "Range_YMax", "Y最大范围" },
        { "Range_YMin", "Y最小范围" },
        { "Range_ZMax", "Z最大范围" },
        { "Range_ZMin", "Z最小范围" },
        { "Range_XMAX", "X最大范围" },
        { "Range_XMIN", "X最小范围" },
        { "Range_YMAX", "Y最大范围" },
        { "Range_YMIN", "Y最小范围" },
        { "Range_ZMAX", "Z最大范围" },
        { "Range_ZMIN", "Z最小范围" },
        { "XMax", "X最大值" },
        { "XMin", "X最小值" },
        { "YMax", "Y最大值" },
        { "YMin", "Y最小值" },
        { "ZMax", "Z最大值" },
        { "ZMin", "Z最小值" },
        { "YMaxCar", "大车Y最大值" },
        { "YMinCar", "大车Y最小值" },
        { "YMaxRobot", "机器人Y最大值" },
        { "YMinRobot", "机器人Y最小值" },
        { "ImgStart_x", "图像起始X" },
        { "ImgEnd_x", "图像结束X" },
        { "Scanlength", "扫描长度" },
        { "ScanDir", "扫描方向" },
        { CAMERA_READ_FPS_KEY, "相机读取帧率" },
        { CAMERA_TIME_OFFSET_MS_KEY, "相机时间补偿(ms)" },
        { "ImgStartX", "图像起始X" },
        { "ImgEndX", "图像结束X" },
        { "TableScanDir", "料台扫描方向" },
        { "ExAxisEnable", "外部轴使能" },
        { "WeldEnable", "焊接开关" },
        { "WeldSpeedMmPerMin", "焊接速度" },
        { "DryRunSpeedMmPerMin", "空跑速度" },
        { "WeldSafeMoveSpeedMmPerMin", "安全位速度" },
        { "StepOverlapRel", "STEP过渡比例" },
        { "WorldCoorDir", "世界Z方向" },
        { "RobotInstallDir", "机器人安装方向" },
        { "GunAngle", "焊枪角度" },
        { "GunLaserAngle", "激光枪夹角" },
        { "GunCameraAngle", "相机枪夹角" },
        { "RotateToCamRxDir", "转相机RX方向" },
        { "HandEyeDis", "手眼距离" },
        { "MeasureDisThreshold", "端点测量阈值" },
        { "FlatMeasureRx", "平焊测量RX" },
        { "FlatMeasureRy", "平焊测量RY" },
        { "FlatWeldRx", "平焊焊接RX" },
        { "FlatWeldRy", "平焊焊接RY" },
        { "NormalWeldRx", "平焊RX" },
        { "NormalWeldRy", "平焊RY" },
        { USE_TAUGHT_WELD_POSE_KEY, "启用示教焊接姿态" },
        { TAUGHT_WELD_POSE_RX_KEY, "示教焊接RX" },
        { TAUGHT_WELD_POSE_RY_KEY, "示教焊接RY" },
        { TAUGHT_WELD_POSE_RZ_KEY, "示教焊接RZ" },
        { "CornerTransitionLeadDis", "拐点过渡距离" },
        { "WeldStartSkipDis", "起点跳过距离" },
        { "WeldEndSkipDis", "终点跳过距离" },
        { "WeldRzGainDeg", "焊接RZ增益" },
        { "SlopeRzMinDeg", "爬坡RZ负向夹紧" },
        { "SlopeRzMaxDeg", "爬坡RZ正向夹紧" },
        { "StandWeldRx", "立焊RX" },
        { "StandWeldRy", "立焊RY" },
        { "TransitionsRx", "过渡RX" },
        { "TransitionsRy", "过渡RY" },
        { "StandWeldScanFreeRx", "立焊扫描RX" },
        { "StandWeldScanFreeRy", "立焊扫描RY" },
        { "StandWeldScanDis", "立焊扫描距离" },
        { "StandWeldScanOffsetRz", "立焊扫描RZ偏移" },
        { "WeldNorAngleInHome", "安全位法向" },
        { "EndpointSearchDis", "端点搜索长度" },
        { "StartSearchOffeset_RZ", "起点RZ偏移" },
        { "EndSearchOffeset_RZ", "终点RZ偏移" },
        { "GunDownBackSafeDis", "收下枪安全距" },
        { "ShortSeamThreshold", "短焊缝阈值" },
        { "LengthSeamThreshold", "长焊缝阈值" },
        { "PointSpacing", "测量点间距" },
        { "CleanGunDis", "清枪距离" },
        { "FlatWeldContinue", "平焊连续" },
        { "CheckCollideEnable", "干涉检测" },
        { "WeldDirection", "焊接方向" },
        { "SameSideRelativeDisToBaseCenter", "同侧安全距" },
        { "ContralateralRelativeDisToBaseCenter", "对侧安全距" },
        { "ContralateralExAxisOffset", "对侧外轴偏移" },
        { "SameSideRelativeExPosDir", "同侧外轴方向" },
        { "StandInitWeldRy", "立焊初始RY" },
        { "JudgeOpenTrackLength", "跟踪开启长度" },
        { "RemoveCloud", "去点云料台" },
    };

    const auto it = names.find(key);
    if (it != names.end())
    {
        return it.value();
    }
    return key;
}

QString ScanParamGroupTitleForKey(const QString& key)
{
    const QString normalized = key.trimmed();
    static const QSet<QString> softLimitKeys = {
        "YMaxCar", "YMinCar", "YMaxRobot", "YMinRobot", "XMax", "XMin", "ZMax", "ZMin"
    };
    static const QSet<QString> speedKeys = {
        "ScanSpeed", "RunSpeed", "CameraReadFps", "CameraTimeOffsetMs", "dAcc", "dDec",
        "UseComputedScanSafe", "ScanSafeOffsetDistanceMm", "ScanSafeGunAngleDeg",
        "ScanSafeXDirection", "ScanSafeLiftHeightMm", "ScanSafeFlipWarnThresholdDeg"
    };
    static const QSet<QString> tableKeys = { "TableY", "TableZ" };
    static const QSet<QString> carInitKeys = { "ScanStartCarLoction", "ScanEndtCarLoction" };
    static const QSet<QString> range3DKeys = {
        "Range_XMax", "Range_XMin", "Range_YMax", "Range_YMin", "Range_ZMax", "Range_ZMin"
    };
    static const QSet<QString> range2DKeys = { "ImgStart_x", "ImgEnd_x", "ImgStart_y", "ImgEnd_y", "ImgStartX", "ImgEndX" };

    if (softLimitKeys.contains(normalized))
    {
        return QStringLiteral("位置软极限");
    }
    if (speedKeys.contains(normalized))
    {
        return QStringLiteral("速度参数");
    }
    if (tableKeys.contains(normalized))
    {
        return QStringLiteral("料台大小");
    }
    if (carInitKeys.contains(normalized))
    {
        return QStringLiteral("大车初始位置");
    }
    if (normalized.compare("Scanlength", Qt::CaseInsensitive) == 0
        || normalized.compare("ScanLength", Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("扫描长度");
    }
    if (normalized.compare("ScanDir", Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("扫描方向 123表示xyz(世界坐标系) 正负表示方向");
    }
    if (normalized.compare("ExAxisEnable", Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("外部轴使能 1运动外部轴 0运动机器人");
    }
    if (range3DKeys.contains(normalized))
    {
        return QStringLiteral("线扫点云处理范围 三维");
    }
    if (range2DKeys.contains(normalized))
    {
        return QStringLiteral("线扫点云处理范围 二维");
    }
    return QStringLiteral("通用参数");
}

QString WeldParamGroupTitleForKey(const QString& key)
{
    const QString normalized = key.trimmed();
    static const QSet<QString> executeKeys = {
        "WeldEnable", "WeldSpeedMmPerMin", "DryRunSpeedMmPerMin", "WeldSafeMoveSpeedMmPerMin",
        "StepOverlapRel", "WeldDirection"
    };
    static const QSet<QString> coordinateKeys = {
        "WorldCoorDir", "RobotInstallDir", "GunAngle", "GunLaserAngle", "GunCameraAngle",
        "RotateToCamRxDir", "HandEyeDis", "MeasureDisThreshold"
    };
    static const QSet<QString> poseKeys = {
        "FlatMeasureRx", "FlatMeasureRy", "FlatWeldRx", "FlatWeldRy", "NormalWeldRx",
        "NormalWeldRy", "CornerTransitionLeadDis", "WeldStartSkipDis",
        "WeldEndSkipDis", "WeldRzGainDeg", "SlopeRzMinDeg", "SlopeRzMaxDeg",
        "StandWeldRx", "StandWeldRy", "TransitionsRx",
        "TransitionsRy", "StandWeldScanFreeRx", "StandWeldScanFreeRy", "StandWeldScanDis",
        "StandWeldScanOffsetRz", "WeldNorAngleInHome", "EndpointSearchDis",
        "StartSearchOffeset_RZ", "EndSearchOffeset_RZ", "GunDownBackSafeDis",
        "ShortSeamThreshold", "LengthSeamThreshold", "PointSpacing", "StandInitWeldRy"
    };

    if (executeKeys.contains(normalized))
    {
        return QStringLiteral("焊接执行");
    }
    if (coordinateKeys.contains(normalized))
    {
        return QStringLiteral("坐标和枪角");
    }
    if (poseKeys.contains(normalized))
    {
        return QStringLiteral("焊接姿态");
    }
    return QStringLiteral("焊接流程参数");
}

QString OtherParamGroupTitleForKey(const QString& key, bool weldSection)
{
    return weldSection ? WeldParamGroupTitleForKey(key) : ScanParamGroupTitleForKey(key);
}

QString PreciseCommentText(const QString& line)
{
    QString text = line.trimmed();
    while (text.startsWith('#'))
    {
        text.remove(0, 1);
        text = text.trimmed();
    }
    return text;
}

QString ValueForWriteWithInlineComment(const QComboBox* combo)
{
    if (combo == nullptr)
    {
        return QString();
    }

    QString value = combo->currentData().toString();
    const QString comment = combo->property("inlineComment").toString().trimmed();
    if (!comment.isEmpty())
    {
        value += " " + comment;
    }
    return value;
}

void AddOtherParamEditor(
    QGridLayout* layout,
    QMap<QString, QLineEdit*>& editors,
    QMap<QString, QComboBox*>& comboEditors,
    int& row,
    int& colInGroup,
    const QString& sectionName,
    const QString& key,
    const QString& value)
{
    if (IsObsoletePreciseParamKey(key))
    {
        return;
    }

    const QString editorId = EditorId(sectionName, key);
    if (layout == nullptr || editors.contains(editorId) || comboEditors.contains(editorId))
    {
        return;
    }

    QString inlineComment;
    const QString displayValue = DisplayValueFromRawIniValue(value, &inlineComment);
    QLabel* label = new QLabel(PreciseParamDisplayName(key));
    label->setToolTip(key);
    label->setMinimumWidth(kOtherParamLabelWidth);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    const int uiCol = colInGroup * 2;

    if (IsWeldDirectionParamKey(key))
    {
        QComboBox* combo = new QComboBox();
        combo->addItem(QStringLiteral("起点到终点"), QStringLiteral("1"));
        combo->addItem(QStringLiteral("终点到起点"), QStringLiteral("-1"));
        bool ok = false;
        const int direction = displayValue.trimmed().toInt(&ok);
        combo->setCurrentIndex(ok && direction < 0 ? 1 : 0);
        combo->setToolTip(inlineComment.isEmpty() ? key : QString("%1\n%2").arg(key, inlineComment));
        combo->setProperty("paramSection", sectionName);
        combo->setProperty("paramKey", key);
        combo->setProperty("inlineComment", inlineComment);
        combo->setMinimumWidth(kOtherParamEditorMinWidth);
        combo->setMaximumWidth(kOtherParamEditorMaxWidth);
        combo->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        comboEditors.insert(editorId, combo);
        layout->addWidget(label, row, uiCol);
        layout->addWidget(combo, row, uiCol + 1, Qt::AlignLeft);
        ++colInGroup;
        if (colInGroup >= 2)
        {
            ++row;
            colInGroup = 0;
        }
        return;
    }

    QLineEdit* edit = new QLineEdit(displayValue);
    edit->setToolTip(inlineComment.isEmpty() ? key : QString("%1\n%2").arg(key, inlineComment));
    edit->setProperty("paramSection", sectionName);
    edit->setProperty("paramKey", key);
    edit->setProperty("inlineComment", inlineComment);
    edit->setMinimumWidth(kOtherParamEditorMinWidth);
    edit->setMaximumWidth(kOtherParamEditorMaxWidth);
    edit->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    edit->setAlignment(Qt::AlignRight);
    if (LooksNumericValue(displayValue))
    {
        edit->setValidator(CreateDoubleValidator(edit));
        MarkNumericEdit(edit);
    }
    editors.insert(editorId, edit);

    layout->addWidget(label, row, uiCol);
    layout->addWidget(edit, row, uiCol + 1, Qt::AlignLeft);
    ++colInGroup;
    if (colInGroup >= 2)
    {
        ++row;
        colInGroup = 0;
    }
}

}

PreciseMeasureEditDialog::PreciseMeasureEditDialog(
    ContralUnit* pContralUnit,
    QWidget* parent,
    bool positionTeachOnly,
    std::function<void()> openCameraPreviewCallback)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_bPositionTeachOnly(positionTeachOnly)
    , m_openCameraPreviewCallback(std::move(openCameraPreviewCallback))
{
    BuildUi();
    LoadRobotList();
    LoadCurrentParam();
}

void PreciseMeasureEditDialog::BuildUi()
{
    const QString dialogTitle = m_bPositionTeachOnly ? QStringLiteral("扫描位置示教") : QStringLiteral("测量焊接参数");
    setWindowTitle(dialogTitle);
    setObjectName("PreciseMeasureEditDialog");
    setWindowFlags(windowFlags() | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
    setMinimumSize(m_bPositionTeachOnly ? 1180 : 720, m_bPositionTeachOnly ? 620 : 520);
    setStyleSheet(QString(
        "QDialog#PreciseMeasureEditDialog { background: #111820; color: #ECF3F4; }"
        "QWidget#PreciseMeasurePage { background: #111820; color: #ECF3F4; }"
        "QWidget#PrecisePulsePanel, QWidget#PreciseOtherPanel { background: #111820; color: #ECF3F4; }"
        "QWidget#OtherParamWidget { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #9ED8DB; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:disabled { background: #16232D; color: #6D7F88; border-color: #263845; }"
        "QPushButton#OtherParamSectionHeader { background: #0B1117; color: #9EEBF0; border: 1px solid #2D5262; border-radius: 0px; padding: 8px 12px; font-weight: bold; text-align: left; }"
        "QPushButton#OtherParamSectionHeader:hover { background: #1A3543; border-color: #72D4DD; }"
        "QPushButton#ParamPageTab { background: #0B1117; color: #BFE8EC; border: 1px solid #2D5262; border-radius: 0px; padding: 8px 26px; font-weight: bold; }"
        "QPushButton#ParamPageTab:hover { background: #17313D; border-color: #72D4DD; }"
        "QPushButton#ParamPageTab:checked { background: #1A4A59; color: #9EEBF0; border-color: #72D4DD; }"
        "QWidget#ParamTabbedFrame { background: #111820; }"
        "QWidget#ParamTabContentFrame { background: #111820; border: 1px solid #2E4656; border-radius: 0px; }"
        "QWidget#OtherParamSection { background: #111820; }"
        "QWidget#OtherParamSectionBody { background: #0D151D; border: 1px solid #223D4C; border-top: 0px; border-radius: 0px; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 7px; padding: 4px 6px; }"
        "QLineEdit:disabled { background: #0A1016; color: #6D7F88; border-color: #263845; }"
        "QCheckBox { color: #BFE8EC; spacing: 8px; }"
        "QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #5C7A8B; background: #0B1117; border-radius: 2px; }"
        "QCheckBox::indicator:checked { background: #1E8AA0; border-color: #72D4DD; }"
        "QDoubleSpinBox { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 7px; padding: 4px 6px; min-height: 24px; }"
        "QGroupBox:disabled { border-color: #263845; color: #6D7F88; }"
        "QGroupBox:disabled::title { color: #6D7F88; }"
        "QScrollArea#PrecisePageScroll { background: #111820; border: none; }"
        "QScrollArea#PrecisePageScroll > QWidget > QWidget { background: #111820; }"
        "QScrollBar:vertical { background: #111820; width: 12px; margin: 0; }"
        "QScrollBar:vertical:disabled { background: transparent; }"
        "QScrollBar::handle:vertical { background: #385366; border-radius: 6px; min-height: 24px; }"
        "QScrollBar::handle:vertical:disabled { background: transparent; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"
        "QScrollBar::add-page:vertical:disabled, QScrollBar::sub-page:vertical:disabled { background: transparent; }"
        "QScrollBar:horizontal { background: #111820; height: 12px; margin: 0; }"
        "QScrollBar::handle:horizontal { background: #385366; border-radius: 6px; min-width: 24px; }"
        "QScrollBar::handle:horizontal:hover { background: #4D7488; }"
        "QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }"
        "QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }"
        "QLabel#UnitLabel { color: #8FA7B0; padding-left: 2px; }")
        + UnifiedComboBoxStyleSheet());

    QVBoxLayout* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    QScrollArea* pageScrollArea = new QScrollArea();
    pageScrollArea->setObjectName("PrecisePageScroll");
    ConfigureResponsiveScrollArea(pageScrollArea);
    outerLayout->addWidget(pageScrollArea);

    QWidget* pageWidget = new QWidget();
    pageWidget->setObjectName("PreciseMeasurePage");
    pageWidget->setMinimumWidth(m_bPositionTeachOnly ? 1360 : 1280);
    QVBoxLayout* rootLayout = new QVBoxLayout(pageWidget);
    rootLayout->setContentsMargins(22, 18, 22, 18);
    pageScrollArea->setWidget(pageWidget);

    QPushButton* reloadBtn = new QPushButton("重新读取");
    QPushButton* saveBtn = new QPushButton("保存参数");
    QPushButton* cameraPreviewBtn = nullptr;
    if (m_bPositionTeachOnly)
    {
        cameraPreviewBtn = new QPushButton("坡口相机预览");
        cameraPreviewBtn->setMinimumWidth(150);
        cameraPreviewBtn->setEnabled(static_cast<bool>(m_openCameraPreviewCallback));
        if (!m_openCameraPreviewCallback)
        {
            cameraPreviewBtn->setToolTip("当前入口没有连接坡口相机预览。");
        }
    }
    reloadBtn->setMinimumWidth(120);
    saveBtn->setMinimumWidth(120);

    QHBoxLayout* headerLayout = new QHBoxLayout();
    headerLayout->setSpacing(12);
    QVBoxLayout* headerLeftLayout = new QVBoxLayout();
    headerLeftLayout->setSpacing(10);
    QLabel* titleLabel = new QLabel(dialogTitle);
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    headerLeftLayout->addWidget(titleLabel);
    QHBoxLayout* robotLayout = new QHBoxLayout();
    robotLayout->addWidget(new QLabel("读取机器人："));
    m_pRobotCombo = new QComboBox();
    m_pRobotCombo->setMinimumWidth(220);
    m_pRobotCombo->setFixedHeight(28);
    robotLayout->addWidget(m_pRobotCombo);
    headerLeftLayout->addLayout(robotLayout);
    QHBoxLayout* groupLayout = new QHBoxLayout();
    groupLayout->addWidget(new QLabel("位置类型："));
    m_pGroupCombo = new QComboBox();
    m_pGroupCombo->setMinimumWidth(180);
    m_pGroupCombo->setFixedHeight(28);
    groupLayout->addWidget(m_pGroupCombo);
    groupLayout->addWidget(new QLabel("中文名："));
    m_pGroupNameEdit = new QLineEdit();
    m_pGroupNameEdit->setPlaceholderText("参数组名称");
    m_pGroupNameEdit->setMinimumWidth(150);
    groupLayout->addWidget(m_pGroupNameEdit);
    QPushButton* addGroupBtn = nullptr;
    QPushButton* copyGroupBtn = nullptr;
    QPushButton* deleteGroupBtn = nullptr;
    QPushButton* teachPositionBtn = nullptr;
    if (!m_bPositionTeachOnly)
    {
        addGroupBtn = new QPushButton("新建");
        copyGroupBtn = new QPushButton("复制");
        deleteGroupBtn = new QPushButton("删除");
        teachPositionBtn = new QPushButton("扫描位置示教");
        addGroupBtn->setMinimumWidth(88);
        copyGroupBtn->setMinimumWidth(88);
        deleteGroupBtn->setMinimumWidth(88);
        teachPositionBtn->setMinimumWidth(126);
        groupLayout->addWidget(addGroupBtn);
        groupLayout->addWidget(copyGroupBtn);
        groupLayout->addWidget(deleteGroupBtn);
        groupLayout->addWidget(teachPositionBtn);
    }
    groupLayout->addStretch(1);
    headerLeftLayout->addLayout(groupLayout);
    headerLayout->addLayout(headerLeftLayout);
    headerLayout->addStretch(1);
    if (cameraPreviewBtn != nullptr)
    {
        headerLayout->addWidget(cameraPreviewBtn, 0, Qt::AlignTop);
    }
    headerLayout->addWidget(reloadBtn, 0, Qt::AlignTop);
    headerLayout->addWidget(saveBtn, 0, Qt::AlignTop);
    headerLayout->addSpacing(142);
    rootLayout->addLayout(headerLayout);

    m_pContentSplitter = new QSplitter(Qt::Horizontal);
    m_pContentSplitter->setChildrenCollapsible(false);
    rootLayout->addWidget(m_pContentSplitter, 1);

    m_pPulsePanel = new QWidget(pageWidget);
    m_pPulsePanel->setObjectName("PrecisePulsePanel");
    m_pPulseGroupsLayout = new QGridLayout(m_pPulsePanel);
    m_pPulseGroupsLayout->setContentsMargins(0, 0, 0, 0);
    m_pPulseGroupsLayout->setAlignment(Qt::AlignTop);
    m_pulseGroupWidgets
        << CreatePulseGroup("下枪安全位置（脉冲）", "StartSafePulse0", "示教下枪安全位置", &PreciseMeasureEditDialog::TeachStartSafePulse, &PreciseMeasureEditDialog::SaveManualStartSafePulse)
        << CreateCoorsGroup("扫描起点（直角）", "StartPos", "示教起点位置", &PreciseMeasureEditDialog::TeachStartPos, &PreciseMeasureEditDialog::SaveManualStartPos)
        << CreateCoorsGroup("扫描终点（直角）", "EndPos", "示教终点位置", &PreciseMeasureEditDialog::TeachEndPos, &PreciseMeasureEditDialog::SaveManualEndPos)
        << CreatePulseGroup("收枪安全位置（脉冲）", "EndSafePulse0", "示教收枪安全位置", &PreciseMeasureEditDialog::TeachEndSafePulse, &PreciseMeasureEditDialog::SaveManualEndSafePulse);
    if (m_bPositionTeachOnly)
    {
        m_pWeldPoseTeachPanel = CreateWeldPoseTeachPanel();
        m_pulseGroupWidgets << m_pWeldPoseTeachPanel;
        m_pScanSafeParamPanel = CreateScanSafeParamPanel();
        m_pulseGroupWidgets << m_pScanSafeParamPanel;
    }

    m_pOtherPanel = new QWidget(pageWidget);
    m_pOtherPanel->setObjectName("PreciseOtherPanel");
    QVBoxLayout* otherPanelLayout = new QVBoxLayout(m_pOtherPanel);
    otherPanelLayout->setContentsMargins(0, 0, 0, 0);
    if (!m_bPositionTeachOnly)
    {
        m_pWeldPoseTeachPanel = CreateWeldPoseTeachPanel();
        otherPanelLayout->addWidget(m_pWeldPoseTeachPanel, 0, Qt::AlignTop);
    }
    QWidget* otherGroup = new QWidget();
    otherGroup->setObjectName("ParamTabbedFrame");
    otherGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    QVBoxLayout* otherLayout = new QVBoxLayout(otherGroup);
    otherLayout->setContentsMargins(0, 0, 0, 0);
    otherLayout->setSpacing(0);
    QHBoxLayout* paramTabLayout = new QHBoxLayout();
    paramTabLayout->setContentsMargins(36, 0, 0, 0);
    paramTabLayout->setSpacing(0);
    m_pScanParamTabBtn = new QPushButton("扫描参数");
    m_pWeldParamTabBtn = new QPushButton("焊接参数");
    m_pScanParamTabBtn->setObjectName("ParamPageTab");
    m_pWeldParamTabBtn->setObjectName("ParamPageTab");
    m_pScanParamTabBtn->setCheckable(true);
    m_pWeldParamTabBtn->setCheckable(true);
    m_pScanParamTabBtn->setChecked(true);
    m_pWeldParamTabBtn->setChecked(false);
    paramTabLayout->addWidget(m_pScanParamTabBtn);
    paramTabLayout->addWidget(m_pWeldParamTabBtn);
    paramTabLayout->addStretch(1);
    otherLayout->addLayout(paramTabLayout);

    QWidget* otherContentFrame = new QWidget();
    otherContentFrame->setObjectName("ParamTabContentFrame");
    QVBoxLayout* otherContentLayout = new QVBoxLayout(otherContentFrame);
    otherContentLayout->setContentsMargins(26, 18, 26, 22);
    otherContentLayout->setSpacing(12);

    QLabel* otherHint = new QLabel("扫描参数和焊接参数按当前选中的位置类型保存；修改后点击“保存参数”统一写回参数文件。");
    otherContentLayout->addWidget(otherHint);

    QWidget* otherWidget = new QWidget();
    otherWidget->setObjectName("OtherParamWidget");
    otherWidget->setAutoFillBackground(true);
    m_pOtherParamLayout = new QGridLayout(otherWidget);
    m_pOtherParamLayout->setContentsMargins(8, 8, 8, 8);
    m_pOtherParamLayout->setColumnStretch(1, 1);
    otherContentLayout->addWidget(otherWidget);
    otherLayout->addWidget(otherContentFrame);

    otherPanelLayout->addWidget(otherGroup, 0, Qt::AlignTop);
    otherPanelLayout->addStretch(1);

    if (m_bPositionTeachOnly)
    {
        m_pContentSplitter->addWidget(m_pPulsePanel);
        m_pOtherPanel->hide();
        m_pContentSplitter->setStretchFactor(0, 1);
    }
    else
    {
        m_pContentSplitter->addWidget(m_pOtherPanel);
        m_pPulsePanel->hide();
        m_pContentSplitter->setStretchFactor(0, 1);
    }

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1000);
    m_pLogText->setMaximumHeight(100);
    m_pLogText->setPlainText("日志：等待读取参数...");
    rootLayout->addWidget(m_pLogText);

    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &PreciseMeasureEditDialog::OnRobotChanged);
    connect(m_pGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &PreciseMeasureEditDialog::OnParamGroupChanged);
    if (addGroupBtn != nullptr)
    {
        connect(addGroupBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::AddZeroParamGroup);
    }
    if (copyGroupBtn != nullptr)
    {
        connect(copyGroupBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::CopyCurrentParamGroup);
    }
    if (deleteGroupBtn != nullptr)
    {
        connect(deleteGroupBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::DeleteCurrentParamGroup);
    }
    if (teachPositionBtn != nullptr)
    {
        connect(teachPositionBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::OpenPositionTeachDialog);
    }
    if (cameraPreviewBtn != nullptr)
    {
        connect(cameraPreviewBtn, &QPushButton::clicked, this, [this]()
            {
                if (m_openCameraPreviewCallback)
                {
                    m_openCameraPreviewCallback();
                }
            });
    }
    connect(m_pScanParamTabBtn, &QPushButton::clicked, this, [this]() { SwitchOtherParamPage(true); });
    connect(m_pWeldParamTabBtn, &QPushButton::clicked, this, [this]() { SwitchOtherParamPage(false); });
    connect(reloadBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::ReloadCurrentParam);
    connect(saveBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::SaveAllParamEdits);
    UpdateAdaptiveLayout();
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, m_bPositionTeachOnly ? QSize(1420, 760) : QSize(980, 680), 0.90, 0.82);
}

void PreciseMeasureEditDialog::resizeEvent(QResizeEvent* event)
{
    QDialog::resizeEvent(event);
    UpdateAdaptiveLayout();
}

void PreciseMeasureEditDialog::LoadRobotList()
{
    m_bLoading = true;
    m_pRobotCombo->clear();

    if (m_pContralUnit == nullptr)
    {
        AppendLog("控制单元为空，无法加载机器人列表。");
        m_bLoading = false;
        return;
    }

    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        QString error;
        const QString paramPath = RobotDataHelper::MeasureWeldParamPath(info.robotName);
        if (RobotDataHelper::EnsureMeasureWeldParamFile(info.robotName, &error))
        {
            m_pRobotCombo->addItem(info.displayName, info.unitIndex);
        }
        else
        {
            AppendLog(QString("跳过 %1：%2").arg(info.displayName, error.isEmpty() ? paramPath : error));
        }
    }

    if (m_pRobotCombo->count() <= 0)
    {
        AppendLog("控制单元中没有可用于精测量示教的机器人驱动。");
    }
    m_bLoading = false;
    LoadParamGroups();
}

void PreciseMeasureEditDialog::OnRobotChanged(int index)
{
    (void)index;
    if (!m_bLoading)
    {
        LoadParamGroups();
    }
}

void PreciseMeasureEditDialog::LoadParamGroups()
{
    if (m_pGroupCombo == nullptr)
    {
        return;
    }
    m_bLoading = true;
    m_pGroupCombo->clear();
    if (m_pGroupNameEdit != nullptr)
    {
        m_pGroupNameEdit->clear();
    }

    const QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        m_bLoading = false;
        return;
    }
    QString error;
    if (!RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &error))
    {
        AppendLog("读取参数组失败：" + error);
        m_bLoading = false;
        return;
    }

    COPini ini;
    const QString path = CurrentParamFilePath();
    if (!ini.SetFileName(path.toUtf8().constData()))
    {
        AppendLog("读取参数组失败：打开参数文件失败：" + path);
        m_bLoading = false;
        return;
    }

    int groupCount = 1;
    int useNo = 0;
    ini.SetSectionName(ToUtf8StdStringForPrecise(GroupMetaSectionName()));
    ini.ReadString(false, "GroupCount", &groupCount);
    ini.ReadString(false, "UseGroupNo", &useNo);
    groupCount = std::max(1, groupCount);
    useNo = std::clamp(useNo, 0, groupCount - 1);
    for (int index = 0; index < groupCount; ++index)
    {
        std::string groupName;
        ini.ReadString(false, ToUtf8StdStringForPrecise(QString("Group%1Name").arg(index)), groupName);
        const QString displayName = groupName.empty()
            ? QString("参数组%1").arg(index + 1)
            : DecodeIniTextForPrecise(groupName);
        m_pGroupCombo->addItem(QString("%1 / Group%2").arg(displayName).arg(index), index);
    }
    m_pGroupCombo->setCurrentIndex(useNo);
    if (m_pGroupNameEdit != nullptr)
    {
        m_pGroupNameEdit->setText(CurrentGroupName());
    }
    m_bLoading = false;
    LoadCurrentParam();
}

void PreciseMeasureEditDialog::OnParamGroupChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoading)
    {
        return;
    }
    QString error;
    RobotDataHelper::WriteParamValue(
        CurrentParamFilePath(),
        GroupMetaSectionName(),
        "UseGroupNo",
        QString::number(CurrentGroupIndex()),
        &error);
    if (m_pGroupNameEdit != nullptr)
    {
        m_pGroupNameEdit->setText(CurrentGroupName());
    }
    LoadCurrentParam();
}

void PreciseMeasureEditDialog::AddZeroParamGroup()
{
    QString error;
    if (!CreateParamGroup(false, error))
    {
        QMessageBox::warning(this, "新建参数组", error);
        AppendLog("新建参数组失败：" + error);
        return;
    }
    AppendLog("已新建全 0 参数组。");
}

void PreciseMeasureEditDialog::CopyCurrentParamGroup()
{
    QString error;
    if (!CreateParamGroup(true, error))
    {
        QMessageBox::warning(this, "复制参数组", error);
        AppendLog("复制参数组失败：" + error);
        return;
    }
    AppendLog("已复制当前参数组。");
}

void PreciseMeasureEditDialog::DeleteCurrentParamGroup()
{
    const QString robotName = CurrentRobotName();
    const int deleteIndex = CurrentGroupIndex();
    if (robotName.isEmpty() || deleteIndex < 0)
    {
        QMessageBox::warning(this, "删除参数组", "未选择机器人或参数组。");
        return;
    }

    QString error;
    if (!RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &error))
    {
        QMessageBox::warning(this, "删除参数组", error);
        AppendLog("删除参数组失败：" + error);
        return;
    }

    const QString path = CurrentParamFilePath();
    COPini ini;
    if (!ini.SetFileName(path.toUtf8().constData()))
    {
        error = "打开参数文件失败：" + path;
        QMessageBox::warning(this, "删除参数组", error);
        AppendLog("删除参数组失败：" + error);
        return;
    }

    int groupCount = 1;
    ini.SetSectionName(ToUtf8StdStringForPrecise(GroupMetaSectionName()));
    ini.ReadString(false, "GroupCount", &groupCount);
    groupCount = std::max(1, groupCount);
    if (groupCount <= 1)
    {
        QMessageBox::information(this, "删除参数组", "至少保留一个位置类型参数组，当前不能删除。");
        return;
    }

    const QString groupName = CurrentGroupName();
    const QString extraWarning = HasUnsavedChanges() ? "\n\n当前界面存在未保存修改，删除后这些修改会丢弃。" : QString();
    const auto ret = QMessageBox::question(
        this,
        "删除参数组",
        QString("确定删除当前位置类型：%1？%2").arg(groupName, extraWarning),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (ret != QMessageBox::Yes)
    {
        return;
    }

    QStringList groupNames;
    QList<bool> groupNameWasDefault;
    for (int index = 0; index < groupCount; ++index)
    {
        std::string rawName;
        ini.ReadString(false, ToUtf8StdStringForPrecise(QString("Group%1Name").arg(index)), rawName);
        const QString defaultName = QString("参数组%1").arg(index + 1);
        const QString name = rawName.empty() ? defaultName : QString::fromUtf8(rawName.data(), static_cast<int>(rawName.size()));
        groupNames << name;
        groupNameWasDefault << (rawName.empty() || name == defaultName);
    }

    const int newGroupCount = groupCount - 1;
    const int newUseIndex = std::clamp(deleteIndex > 0 ? deleteIndex - 1 : 0, 0, newGroupCount - 1);

    QList<QMap<QString, QString>> keptScanSections;
    QList<QMap<QString, QString>> keptWeldSections;
    QStringList keptGroupNames;
    int newIndex = 0;
    for (int oldIndex = 0; oldIndex < groupCount; ++oldIndex)
    {
        if (oldIndex == deleteIndex)
        {
            continue;
        }

        const QString newName = groupNameWasDefault.value(oldIndex, true)
            ? QString("参数组%1").arg(newIndex + 1)
            : groupNames.value(oldIndex, QString("参数组%1").arg(newIndex + 1));
        keptGroupNames << newName;
        keptScanSections << ReadSectionMapForPrecise(path, RobotDataHelper::MeasureWeldScanSectionName(oldIndex), MinimalScanSectionLinesForPrecise());
        keptWeldSections << ReadSectionMapForPrecise(path, RobotDataHelper::MeasureWeldWeldSectionName(oldIndex), MinimalWeldSectionLinesForPrecise());
        ++newIndex;
    }

    if (!ConfigDatabase::RemoveIniSection(path, GroupMetaSectionName()))
    {
        error = "清理参数组索引失败：" + path;
        QMessageBox::warning(this, "删除参数组", error);
        AppendLog("删除参数组失败：" + error);
        return;
    }
    for (int oldIndex = 0; oldIndex < groupCount; ++oldIndex)
    {
        ConfigDatabase::RemoveIniSection(path, RobotDataHelper::MeasureWeldScanSectionName(oldIndex));
        ConfigDatabase::RemoveIniSection(path, RobotDataHelper::MeasureWeldWeldSectionName(oldIndex));
    }

    if (!RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), "GroupCount", QString::number(newGroupCount), &error)
        || !RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), "UseGroupNo", QString::number(newUseIndex), &error))
    {
        QMessageBox::warning(this, "删除参数组", error);
        AppendLog("删除参数组失败：" + error);
        return;
    }
    for (int index = 0; index < newGroupCount; ++index)
    {
        if (!RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), QString("Group%1Name").arg(index), keptGroupNames.value(index), &error)
            || !WriteSectionMapForPrecise(path, RobotDataHelper::MeasureWeldScanSectionName(index), keptScanSections.value(index), error)
            || !WriteSectionMapForPrecise(path, RobotDataHelper::MeasureWeldWeldSectionName(index), keptWeldSections.value(index), error))
        {
            QMessageBox::warning(this, "删除参数组", error);
            AppendLog("删除参数组失败：" + error);
            return;
        }
    }

    LoadParamGroups();
    AppendLog(QString("已删除参数组：%1。").arg(groupName));
}

void PreciseMeasureEditDialog::TeachStartPos()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ROBOT_COORS coors = driver->GetCurrentPos();
    m_taughtStartPulse = driver->GetCurrentPulse();
    m_hasTaughtStartPulse = true;
    SetCoorsEditors("StartPos", coors);
    AppendLog("已读取当前机器人直角坐标和关节脉冲到扫描起点，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::TeachStartSafePulse()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ANGLE_PULSE pulse = driver->GetCurrentPulse();
    SetPulseEditors("StartSafePulse0", pulse);
    AppendLog("已读取当前机器人脉冲到下枪安全位置，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::TeachEndPos()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ROBOT_COORS coors = driver->GetCurrentPos();
    SetCoorsEditors("EndPos", coors);
    AppendLog("已读取当前机器人直角坐标到扫描终点，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::TeachEndSafePulse()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ANGLE_PULSE pulse = driver->GetCurrentPulse();
    SetPulseEditors("EndSafePulse0", pulse);
    AppendLog("已读取当前机器人脉冲到收枪安全位置，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::OpenPositionTeachDialog()
{
    DelayedLoadingGuard loading(this, "正在打开扫描位置示教", 1000);
    PreciseMeasureEditDialog* dialog = new PreciseMeasureEditDialog(
        m_pContralUnit,
        this,
        true,
        m_openCameraPreviewCallback);
    loading.Pulse();
    dialog->setAttribute(Qt::WA_DeleteOnClose);

    if (m_pRobotCombo != nullptr && dialog->m_pRobotCombo != nullptr)
    {
        dialog->m_pRobotCombo->setCurrentIndex(m_pRobotCombo->currentIndex());
    }
    if (m_pGroupCombo != nullptr && dialog->m_pGroupCombo != nullptr)
    {
        dialog->m_pGroupCombo->setCurrentIndex(m_pGroupCombo->currentIndex());
    }

    dialog->show();
    dialog->raise();
    dialog->activateWindow();
    loading.Finish();
}

void PreciseMeasureEditDialog::ReloadCurrentParam()
{
    LoadCurrentParam();
}

void PreciseMeasureEditDialog::closeEvent(QCloseEvent* event)
{
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, windowTitle(), [this]() { return SaveAllParamEdits(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

bool PreciseMeasureEditDialog::SaveAllParamEdits()
{
    if (m_bLoading)
    {
        return false;
    }

    QString error;
    if (m_bPositionTeachOnly)
    {
        T_ANGLE_PULSE startSafePulse;
        T_ROBOT_COORS startPos;
        T_ROBOT_COORS endPos;
        T_ANGLE_PULSE endSafePulse;
        if (!GetPulseFromEditors("StartSafePulse0", startSafePulse, error)
            || !GetCoorsFromEditors("StartPos", startPos, error)
            || !GetCoorsFromEditors("EndPos", endPos, error)
            || !GetPulseFromEditors("EndSafePulse0", endSafePulse, error))
        {
            QMessageBox::warning(this, "保存参数", error);
            AppendLog("保存失败：" + error);
            return false;
        }

        if (!WritePulse("StartSafePulse0", startSafePulse, error)
            || !WriteParamValue(CurrentSectionName(&error), "StartSafePulseNum", PulseListCountValue(startSafePulse), error)
            || !WriteCoors("StartPos", startPos, error)
            || !WriteCoors("EndPos", endPos, error)
            || !WriteParamValue(CurrentSectionName(&error), "EndSafePulseNum", PulseListCountValue(endSafePulse), error)
            || !WritePulse("EndSafePulse0", endSafePulse, error))
        {
            QMessageBox::warning(this, "保存参数", error);
            AppendLog("保存失败：" + error);
            return false;
        }

        if (m_hasTaughtStartPulse && !WritePulse("StartPulse", m_taughtStartPulse, error))
        {
            QMessageBox::warning(this, "保存参数", error);
            AppendLog("扫描起点关节脉冲保存失败：" + error);
            return false;
        }

        if (!SaveScanSafeParams(error))
        {
            QMessageBox::warning(this, "保存参数", error);
            AppendLog("扫描安全推算参数保存失败：" + error);
            return false;
        }
    }

    if (!SaveWeldPoseTeachParams(error))
    {
        QMessageBox::warning(this, "保存参数", error);
        AppendLog("焊接姿态示教参数保存失败：" + error);
        return false;
    }

    if (!SaveGroupMetadata(error))
    {
        QMessageBox::warning(this, "保存参数", error);
        AppendLog("参数组信息保存失败：" + error);
        return false;
    }

    if (!m_bPositionTeachOnly)
    {
        for (auto it = m_otherParamEditors.cbegin(); it != m_otherParamEditors.cend(); ++it)
        {
            QLineEdit* edit = it.value();
            if (edit == nullptr)
            {
                continue;
            }

            const QString sectionName = edit->property("paramSection").toString();
            const QString paramKey = edit->property("paramKey").toString();
            if (!WriteParamValue(sectionName, paramKey, ValueForWriteWithInlineComment(edit), error))
            {
                QMessageBox::warning(this, "保存参数", error);
                AppendLog("其它参数保存失败：" + error);
                return false;
            }
        }
        for (auto it = m_otherParamComboEditors.cbegin(); it != m_otherParamComboEditors.cend(); ++it)
        {
            QComboBox* combo = it.value();
            if (combo == nullptr)
            {
                continue;
            }

            const QString sectionName = combo->property("paramSection").toString();
            const QString paramKey = combo->property("paramKey").toString();
            if (!WriteParamValue(sectionName, paramKey, ValueForWriteWithInlineComment(combo), error))
            {
                QMessageBox::warning(this, "保存参数", error);
                AppendLog("其它参数保存失败：" + error);
                return false;
            }
        }
    }

    MarkCleanSnapshot();
    AppendLog(m_bPositionTeachOnly ? "扫描位置示教参数已保存。" : "测量焊接参数已统一保存。");
    QMessageBox::information(this, "保存参数", m_bPositionTeachOnly ? "扫描位置示教参数保存完成。" : "测量焊接参数保存完成。");
    return true;
}

void PreciseMeasureEditDialog::SaveManualStartSafePulse()
{
    if (m_bLoading)
    {
        return;
    }
    if (m_pUseComputedScanSafeCheck != nullptr && m_pUseComputedScanSafeCheck->isChecked())
    {
        AppendLog("当前使用推算扫描安全位，下枪安全位置示教已禁用。");
        return;
    }

    T_ANGLE_PULSE pulse;
    QString error;
    if (!GetPulseFromEditors("StartSafePulse0", pulse, error)
        || !WritePulse("StartSafePulse0", pulse, error)
        || !WriteParamValue(CurrentSectionName(&error), "StartSafePulseNum", PulseListCountValue(pulse), error))
    {
        AppendLog("下枪安全位置手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog("下枪安全位置手动修改已保存。");
}

void PreciseMeasureEditDialog::SaveManualStartPos()
{
    if (m_bLoading)
    {
        return;
    }

    T_ROBOT_COORS coors;
    QString error;
    if (!GetCoorsFromEditors("StartPos", coors, error) || !WriteCoors("StartPos", coors, error))
    {
        AppendLog("起点手动保存失败：" + error);
        return;
    }
    if (m_hasTaughtStartPulse && !WritePulse("StartPulse", m_taughtStartPulse, error))
    {
        AppendLog("起点关节脉冲保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    UpdateScanSafePreview();
    AppendLog("起点手动修改已保存。");
}

void PreciseMeasureEditDialog::SaveManualEndPos()
{
    if (m_bLoading)
    {
        return;
    }

    T_ROBOT_COORS coors;
    QString error;
    if (!GetCoorsFromEditors("EndPos", coors, error) || !WriteCoors("EndPos", coors, error))
    {
        AppendLog("终点手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    UpdateScanSafePreview();
    AppendLog("终点手动修改已保存。");
}

void PreciseMeasureEditDialog::SaveManualEndSafePulse()
{
    if (m_bLoading)
    {
        return;
    }
    if (m_pUseComputedScanSafeCheck != nullptr && m_pUseComputedScanSafeCheck->isChecked())
    {
        AppendLog("当前使用推算扫描安全位，收枪安全位置示教已禁用。");
        return;
    }

    T_ANGLE_PULSE pulse;
    QString error;
    if (!GetPulseFromEditors("EndSafePulse0", pulse, error)
        || !WritePulse("EndSafePulse0", pulse, error)
        || !WriteParamValue(CurrentSectionName(&error), "EndSafePulseNum", PulseListCountValue(pulse), error))
    {
        AppendLog("收枪安全位置手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog("收枪安全位置手动修改已保存。");
}

RobotDriverAdaptor* PreciseMeasureEditDialog::GetSelectedRobotDriver()
{
    if (m_pContralUnit == nullptr || m_pRobotCombo == nullptr || m_pRobotCombo->currentIndex() < 0)
    {
        QMessageBox::warning(this, "测量焊接参数", "未选择可用机器人。");
        return nullptr;
    }

    const int unitIndex = m_pRobotCombo->currentData().toInt();
    RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, unitIndex);
    if (driver == nullptr)
    {
        QMessageBox::warning(this, "测量焊接参数", "当前选择的机器人没有可用驱动。");
        return nullptr;
    }
    return driver;
}

QString PreciseMeasureEditDialog::CurrentRobotName() const
{
    if (m_pContralUnit == nullptr || m_pRobotCombo == nullptr || m_pRobotCombo->currentIndex() < 0)
    {
        return QString();
    }

    const int unitIndex = m_pRobotCombo->currentData().toInt();
    if (unitIndex < 0 || unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return QString();
    }

    const T_CONTRAL_UNIT& unitInfo = m_pContralUnit->m_vtContralUnitInfo[unitIndex];
    RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
    if (driver == nullptr)
    {
        return QString();
    }

    return QString::fromStdString(driver->m_sRobotName.empty() ? unitInfo.sUnitName : driver->m_sRobotName);
}

QString PreciseMeasureEditDialog::CurrentParamFilePath() const
{
    if (CurrentRobotName().isEmpty())
    {
        return QString();
    }
    return RobotDataHelper::MeasureWeldParamPath(CurrentRobotName());
}

QString PreciseMeasureEditDialog::CurrentSectionName(QString* error) const
{
    if (CurrentRobotName().isEmpty())
    {
        if (error != nullptr)
        {
            *error = "未选择机器人。";
        }
        return QString();
    }
    return RobotDataHelper::MeasureWeldScanSectionName(CurrentGroupIndex());
}

QString PreciseMeasureEditDialog::CurrentWeldSectionName(QString* error) const
{
    if (CurrentRobotName().isEmpty())
    {
        if (error != nullptr)
        {
            *error = "未选择机器人。";
        }
        return QString();
    }
    return RobotDataHelper::MeasureWeldWeldSectionName(CurrentGroupIndex());
}

int PreciseMeasureEditDialog::CurrentGroupIndex() const
{
    if (m_pGroupCombo != nullptr && m_pGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pGroupCombo->currentData().toInt());
    }
    return 0;
}

QString PreciseMeasureEditDialog::CurrentGroupName() const
{
    const int groupIndex = CurrentGroupIndex();
    if (!CurrentRobotName().isEmpty())
    {
        COPini ini;
        const QString path = CurrentParamFilePath();
        if (ini.SetFileName(path.toUtf8().constData()))
        {
            ini.SetSectionName(ToUtf8StdStringForPrecise(GroupMetaSectionName()));
            std::string groupName;
            ini.ReadString(false, ToUtf8StdStringForPrecise(QString("Group%1Name").arg(groupIndex)), groupName);
            if (!groupName.empty())
            {
                return DecodeIniTextForPrecise(groupName);
            }
        }
    }
    return QString("参数组%1").arg(groupIndex + 1);
}

bool PreciseMeasureEditDialog::LoadCurrentParam()
{
    QString error;
    T_ANGLE_PULSE startSafePulse;
    T_ANGLE_PULSE startPulse;
    T_ANGLE_PULSE endSafePulse;
    T_ROBOT_COORS startPos;
    T_ROBOT_COORS endPos;
    if (!ReadPulse("StartSafePulse0", startSafePulse, error)
        || !ReadPulse("EndSafePulse0", endSafePulse, error))
    {
        AppendLog("读取失败：" + error);
        return false;
    }

    QString startPosError;
    QString endPosError;
    const bool hasStartPos = ReadCoors("StartPos", startPos, startPosError);
    const bool hasEndPos = ReadCoors("EndPos", endPos, endPosError);
    if (!hasStartPos)
    {
        startPos = T_ROBOT_COORS();
    }
    if (!hasEndPos)
    {
        endPos = T_ROBOT_COORS();
    }

    SetEditorsBlocked(true);
    SetPulseEditors("StartSafePulse0", startSafePulse);
    SetCoorsEditors("StartPos", startPos);
    SetCoorsEditors("EndPos", endPos);
    SetPulseEditors("EndSafePulse0", endSafePulse);
    SetEditorsBlocked(false);
    if (ReadPulse("StartPulse", startPulse, error))
    {
        m_taughtStartPulse = startPulse;
        m_hasTaughtStartPulse = true;
    }
    else
    {
        m_hasTaughtStartPulse = false;
        AppendLog("未读取到扫描起点关节脉冲，重新示教起点后会自动补写。");
    }
    if (!m_bPositionTeachOnly)
    {
        LoadOtherParams();
    }
    else
    {
        LoadScanSafeParams();
    }
    LoadWeldPoseTeachParams();
    if (!hasStartPos)
    {
        AppendLog("未读取到扫描起点直角参数，请重新示教并保存。" + startPosError);
    }
    if (!hasEndPos)
    {
        AppendLog("未读取到扫描终点直角参数，请重新示教并保存。" + endPosError);
    }
    AppendLog(QString("已读取 %1 的测量焊接参数：%2。").arg(CurrentRobotName(), CurrentGroupName()));
    MarkCleanSnapshot();
    return true;
}

bool PreciseMeasureEditDialog::ReadPulse(const QString& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::ReadPulse(CurrentParamFilePath(), CurrentSectionName(&error), prefix, pulse, &error);
}

bool PreciseMeasureEditDialog::WritePulse(const QString& prefix, const T_ANGLE_PULSE& pulse, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WritePulse(CurrentParamFilePath(), CurrentSectionName(&error), prefix, pulse, &error);
}

bool PreciseMeasureEditDialog::ReadCoors(const QString& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::ReadCoors(CurrentParamFilePath(), CurrentSectionName(&error), prefix, coors, &error);
}

bool PreciseMeasureEditDialog::WriteCoors(const QString& prefix, const T_ROBOT_COORS& coors, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WriteCoors(CurrentParamFilePath(), CurrentSectionName(&error), prefix, coors, &error);
}

bool PreciseMeasureEditDialog::LoadOtherParams()
{
    if (m_pOtherParamLayout == nullptr)
    {
        return false;
    }

    QString error;
    const QString path = CurrentParamFilePath();
    const QString section = CurrentSectionName(&error);
    const QString weldSection = CurrentWeldSectionName(&error);
    if (path.isEmpty() || section.isEmpty() || weldSection.isEmpty())
    {
        AppendLog("读取其它参数失败：" + error);
        return false;
    }

    m_bLoading = true;
    ClearOtherParamEditors();

    bool hasOtherParam = false;
    int outerRow = 0;
    int row = 0;
    int colInGroup = 0;
    QGridLayout* currentGroupLayout = nullptr;
    QString currentGroupTitle;
    auto createCollapsibleGroup = [this, &outerRow, &row, &colInGroup, &currentGroupLayout, &currentGroupTitle](const QString& rawTitle, const QString& categoryTitle = QString()) -> QGridLayout*
        {
            QString title = rawTitle.trimmed().isEmpty() ? QStringLiteral("通用参数") : rawTitle.trimmed();
            currentGroupTitle = title;
            row = 0;
            colInGroup = 0;

            QWidget* sectionWidget = new QWidget();
            sectionWidget->setObjectName("OtherParamSection");
            sectionWidget->setProperty("paramPage", categoryTitle);
            QVBoxLayout* sectionLayout = new QVBoxLayout(sectionWidget);
            sectionLayout->setContentsMargins(0, 0, 0, 4);
            sectionLayout->setSpacing(0);

            QPushButton* headerButton = new QPushButton(sectionWidget);
            headerButton->setObjectName("OtherParamSectionHeader");
            headerButton->setCheckable(true);
            headerButton->setChecked(false);
            headerButton->setCursor(Qt::PointingHandCursor);
            headerButton->setProperty("_keep_wide_control", true);
            headerButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            headerButton->setMinimumHeight(34);
            headerButton->setMaximumHeight(34);
            headerButton->setMinimumWidth(0);
            headerButton->setMaximumWidth(QWIDGETSIZE_MAX);
            sectionLayout->addWidget(headerButton);

            QWidget* bodyWidget = new QWidget(sectionWidget);
            bodyWidget->setObjectName("OtherParamSectionBody");
            bodyWidget->setVisible(false);
            QGridLayout* bodyLayout = new QGridLayout(bodyWidget);
            bodyLayout->setContentsMargins(10, 10, 10, 10);
            bodyLayout->setHorizontalSpacing(12);
            bodyLayout->setVerticalSpacing(8);
            bodyLayout->setColumnMinimumWidth(0, kOtherParamLabelWidth);
            bodyLayout->setColumnMinimumWidth(1, kOtherParamEditorMaxWidth);
            bodyLayout->setColumnMinimumWidth(2, kOtherParamLabelWidth);
            bodyLayout->setColumnMinimumWidth(3, kOtherParamEditorMaxWidth);
            bodyLayout->setColumnStretch(0, 0);
            bodyLayout->setColumnStretch(1, 0);
            bodyLayout->setColumnStretch(2, 0);
            bodyLayout->setColumnStretch(3, 1);
            sectionLayout->addWidget(bodyWidget);

            auto refreshHeader = [headerButton, bodyWidget, title](bool checked)
                {
                    headerButton->setText(QString("%1 %2").arg(checked ? QStringLiteral("▼") : QStringLiteral("▶"), title));
                    bodyWidget->setVisible(checked);
            };
            connect(headerButton, &QPushButton::toggled, this, refreshHeader);
            refreshHeader(false);

            m_pOtherParamLayout->addWidget(sectionWidget, outerRow, 0, 1, 4);
            sectionWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            m_otherParamSectionWidgets << sectionWidget;
            ++outerRow;
            currentGroupLayout = bodyLayout;
            return bodyLayout;
        };
    auto ensureGroup = [&createCollapsibleGroup, &currentGroupLayout](const QString& title, const QString& categoryTitle) -> QGridLayout*
        {
            if (currentGroupLayout == nullptr)
            {
                return createCollapsibleGroup(title, categoryTitle);
            }
            return currentGroupLayout;
        };
    auto loadSectionParams = [&](const QString& targetSection, const QString& categoryTitle, bool skipDedicatedKeys)
        {
            const bool weldSection = categoryTitle == QStringLiteral("焊接参数");
            currentGroupLayout = nullptr;
            currentGroupTitle.clear();
            row = 0;
            colInGroup = 0;
            const QMap<QString, QString> values = ConfigDatabase::ReadIniSection(path, targetSection);
            QStringList groupOrder;
            QMap<QString, QList<QPair<QString, QString>>> groupedValues;
            for (auto it = values.constBegin(); it != values.constEnd(); ++it)
            {
                const QString key = it.key().trimmed();
                if (key.isEmpty() || IsObsoletePreciseParamKey(key))
                {
                    continue;
                }
                if (skipDedicatedKeys && IsDedicatedPulseKey(key))
                {
                    continue;
                }
                if (weldSection && IsDedicatedWeldPoseTeachKey(key))
                {
                    continue;
                }

                const QString groupTitle = OtherParamGroupTitleForKey(key, weldSection);
                if (!groupOrder.contains(groupTitle))
                {
                    groupOrder << groupTitle;
                }
                groupedValues[groupTitle].append(qMakePair(key, it.value()));
            }

            for (const QString& groupTitle : groupOrder)
            {
                currentGroupLayout = createCollapsibleGroup(groupTitle, categoryTitle);
                for (const auto& item : groupedValues.value(groupTitle))
                {
                    AddOtherParamEditor(currentGroupLayout, m_otherParamEditors, m_otherParamComboEditors, row, colInGroup, targetSection, item.first, item.second);
                    hasOtherParam = true;
                }
            }
        };

    loadSectionParams(section, QStringLiteral("扫描参数"), true);
    loadSectionParams(weldSection, QStringLiteral("焊接参数"), false);

    if (!m_otherParamEditors.contains(EditorId(section, CAMERA_READ_FPS_KEY))
        || !m_otherParamEditors.contains(EditorId(section, CAMERA_TIME_OFFSET_MS_KEY)))
    {
        currentGroupLayout = createCollapsibleGroup(QStringLiteral("相机时间参数"), QStringLiteral("扫描参数"));
    }

    if (!m_otherParamEditors.contains(EditorId(section, CAMERA_READ_FPS_KEY)))
    {
        AddOtherParamEditor(
            ensureGroup(QStringLiteral("相机时间参数"), QStringLiteral("扫描参数")),
            m_otherParamEditors,
            m_otherParamComboEditors,
            row,
            colInGroup,
            section,
            CAMERA_READ_FPS_KEY,
            QString::number(DEFAULT_CAMERA_READ_FPS, 'f', 0));
        hasOtherParam = true;
    }

    if (!m_otherParamEditors.contains(EditorId(section, CAMERA_TIME_OFFSET_MS_KEY)))
    {
        AddOtherParamEditor(
            ensureGroup(QStringLiteral("相机时间参数"), QStringLiteral("扫描参数")),
            m_otherParamEditors,
            m_otherParamComboEditors,
            row,
            colInGroup,
            section,
            CAMERA_TIME_OFFSET_MS_KEY,
            QString::number(DEFAULT_CAMERA_TIME_OFFSET_MS, 'f', 0));
        hasOtherParam = true;
    }

    if (!hasOtherParam)
    {
        AppendLog(QString("当前分组没有其它参数：%1").arg(section));
    }

    for (QComboBox* combo : m_otherParamComboEditors)
    {
        if (combo != nullptr)
        {
            connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, combo](int)
                {
                    SaveOtherParamComboEdit(combo);
                });
        }
    }

    UpdateOtherParamPageVisibility();
    m_bLoading = false;
    return true;
}

QWidget* PreciseMeasureEditDialog::CreateScanSafeParamPanel()
{
    QGroupBox* groupBox = new QGroupBox("扫描安全位推算参数");
    groupBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* layout = new QGridLayout(groupBox);
    layout->setContentsMargins(36, 28, 36, 18);
    layout->setHorizontalSpacing(14);
    layout->setVerticalSpacing(10);

    QLabel* hintLabel = new QLabel("只影响扫描段：勾选后由扫描起点/终点直角位姿按枪方向偏移生成安全位；未勾选时使用下枪/收枪安全位置的示教脉冲。");
    hintLabel->setWordWrap(true);
    layout->addWidget(hintLabel, 0, 0, 1, 6);

    m_pUseComputedScanSafeCheck = new QCheckBox("使用起点/终点直角位姿推算扫描安全位");
    layout->addWidget(m_pUseComputedScanSafeCheck, 1, 0, 1, 6);

    auto makeSpin = [](double minValue, double maxValue) -> QDoubleSpinBox*
        {
            QDoubleSpinBox* spin = new QDoubleSpinBox();
            spin->setRange(minValue, maxValue);
            spin->setDecimals(3);
            spin->setSingleStep(1.0);
            spin->setAlignment(Qt::AlignRight);
            spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
            spin->setMinimumWidth(132);
            spin->setProperty("touchKeyboardLayout", QStringLiteral("numeric"));
            spin->setInputMethodHints(Qt::ImhFormattedNumbersOnly);
            return spin;
        };

    m_pScanSafeOffsetSpin = makeSpin(1.0, 1000.0);
    m_pScanSafeGunAngleSpin = makeSpin(-89.0, 89.0);
    m_pScanSafeXDirectionCombo = new QComboBox();
    m_pScanSafeXDirectionCombo->addItem("X- 方向", -1);
    m_pScanSafeXDirectionCombo->addItem("X+ 方向", 1);
    m_pScanSafeXDirectionCombo->setMinimumWidth(132);
    m_pScanSafeLiftSpin = makeSpin(0.0, 1000.0);
    m_pScanSafeFlipWarnSpin = makeSpin(1.0, 360.0);

    auto addRowItem = [layout](int row, int col, const QString& labelText, QWidget* editor, const QString& unitText = QString())
        {
            QLabel* label = new QLabel(labelText);
            layout->addWidget(label, row, col);
            layout->addWidget(unitText.isEmpty() ? editor : CreateUnitEditorForPrecise(editor, unitText), row, col + 1);
        };

    addRowItem(2, 0, "安全偏移距离：", m_pScanSafeOffsetSpin, "mm");
    addRowItem(2, 2, "枪倾角：", m_pScanSafeGunAngleSpin, "deg");
    addRowItem(2, 4, "X 偏移方向：", m_pScanSafeXDirectionCombo);
    addRowItem(3, 0, "姿态切换抬高：", m_pScanSafeLiftSpin, "mm");
    addRowItem(3, 2, "姿态翻转提醒阈值：", m_pScanSafeFlipWarnSpin, "deg");

    m_pScanSafePreviewLabel = new QLabel();
    m_pScanSafePreviewLabel->setWordWrap(true);
    m_pScanSafePreviewLabel->setStyleSheet("color: #9EEBF0;");
    layout->addWidget(m_pScanSafePreviewLabel, 4, 0, 1, 6);

    m_pScanSafePathLabel = new QLabel();
    m_pScanSafePathLabel->setWordWrap(true);
    layout->addWidget(m_pScanSafePathLabel, 5, 0, 1, 6);

    connect(m_pUseComputedScanSafeCheck, &QCheckBox::toggled, this, [this]()
        {
            UpdateComputedScanSafeUiState();
            UpdateScanSafePreview();
        });
    connect(m_pScanSafeOffsetSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { UpdateScanSafePreview(); });
    connect(m_pScanSafeGunAngleSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { UpdateScanSafePreview(); });
    connect(m_pScanSafeXDirectionCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { UpdateScanSafePreview(); });
    connect(m_pScanSafeLiftSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { UpdateScanSafePreview(); });
    connect(m_pScanSafeFlipWarnSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { UpdateScanSafePreview(); });

    for (int i = 0; i < 6; ++i)
    {
        layout->setColumnStretch(i, (i % 2) == 0 ? 0 : 1);
    }
    return groupBox;
}

bool PreciseMeasureEditDialog::LoadScanSafeParams()
{
    if (!m_bPositionTeachOnly || m_pUseComputedScanSafeCheck == nullptr)
    {
        return true;
    }

    QString error;
    const QString path = CurrentParamFilePath();
    const QString section = CurrentSectionName(&error);
    if (path.isEmpty() || section.isEmpty())
    {
        AppendLog("读取扫描安全推算参数失败：" + error);
        return false;
    }

    int useComputed = 1;
    double offset = 150.0;
    double gunAngle = 30.0;
    int xDirection = -1;
    double liftHeight = 150.0;
    double flipWarn = 90.0;

    COPini ini;
    if (ini.SetFileName(path.toUtf8().constData()))
    {
        ini.SetSectionName(ToUtf8StdStringForPrecise(section));
        ini.ReadString(false, "UseComputedScanSafe", &useComputed);
        ini.ReadString(false, "ScanSafeOffsetDistanceMm", &offset);
        ini.ReadString(false, "ScanSafeGunAngleDeg", &gunAngle);
        ini.ReadString(false, "ScanSafeXDirection", &xDirection);
        ini.ReadString(false, "ScanSafeLiftHeightMm", &liftHeight);
        ini.ReadString(false, "ScanSafeFlipWarnThresholdDeg", &flipWarn);
    }
    else
    {
        AppendLog("读取扫描安全推算参数失败：打开参数文件失败：" + path);
        return false;
    }

    QSignalBlocker blockUse(m_pUseComputedScanSafeCheck);
    QSignalBlocker blockOffset(m_pScanSafeOffsetSpin);
    QSignalBlocker blockAngle(m_pScanSafeGunAngleSpin);
    QSignalBlocker blockDirection(m_pScanSafeXDirectionCombo);
    QSignalBlocker blockLift(m_pScanSafeLiftSpin);
    QSignalBlocker blockWarn(m_pScanSafeFlipWarnSpin);
    m_pUseComputedScanSafeCheck->setChecked(useComputed != 0);
    m_pScanSafeOffsetSpin->setValue(qBound(1.0, offset, 1000.0));
    m_pScanSafeGunAngleSpin->setValue(qBound(-89.0, gunAngle, 89.0));
    m_pScanSafeXDirectionCombo->setCurrentIndex(xDirection >= 0 ? 1 : 0);
    m_pScanSafeLiftSpin->setValue(qBound(0.0, liftHeight, 1000.0));
    m_pScanSafeFlipWarnSpin->setValue(qBound(1.0, flipWarn, 360.0));

    if (m_pScanSafePathLabel != nullptr)
    {
        m_pScanSafePathLabel->setText(QString("配置文件：%1\n当前位置类型：%2 [%3]")
            .arg(QDir::toNativeSeparators(path), CurrentGroupName(), section));
    }

    UpdateComputedScanSafeUiState();
    UpdateScanSafePreview();
    return true;
}

bool PreciseMeasureEditDialog::SaveScanSafeParams(QString& error) const
{
    if (!m_bPositionTeachOnly || m_pUseComputedScanSafeCheck == nullptr)
    {
        return true;
    }

    const QString section = CurrentSectionName(&error);
    return WriteParamValue(section, "UseComputedScanSafe", m_pUseComputedScanSafeCheck->isChecked() ? "1" : "0", error)
        && WriteParamValue(section, "ScanSafeOffsetDistanceMm", QString::number(m_pScanSafeOffsetSpin->value(), 'f', 6), error)
        && WriteParamValue(section, "ScanSafeGunAngleDeg", QString::number(m_pScanSafeGunAngleSpin->value(), 'f', 6), error)
        && WriteParamValue(section, "ScanSafeXDirection", QString::number(m_pScanSafeXDirectionCombo->currentData().toInt()), error)
        && WriteParamValue(section, "ScanSafeLiftHeightMm", QString::number(m_pScanSafeLiftSpin->value(), 'f', 6), error)
        && WriteParamValue(section, "ScanSafeFlipWarnThresholdDeg", QString::number(m_pScanSafeFlipWarnSpin->value(), 'f', 6), error);
}

QWidget* PreciseMeasureEditDialog::CreateWeldPoseTeachPanel()
{
    QGroupBox* groupBox = new QGroupBox("焊接姿态示教");
    groupBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* layout = new QGridLayout(groupBox);
    layout->setContentsMargins(36, 28, 36, 18);
    layout->setHorizontalSpacing(14);
    layout->setVerticalSpacing(10);

    m_pWeldPoseHintLabel = new QLabel();
    m_pWeldPoseHintLabel->setWordWrap(true);
    layout->addWidget(m_pWeldPoseHintLabel, 0, 0, 1, 8);

    m_pUseTaughtWeldPoseCheck = new QCheckBox("启用示教焊接姿态");
    layout->addWidget(m_pUseTaughtWeldPoseCheck, 1, 0, 1, 8);

    auto makeEdit = []() -> QLineEdit*
        {
            QLineEdit* edit = CreateValueEdit(112, 150);
            edit->setValidator(CreateDoubleValidator(edit));
            MarkNumericEdit(edit);
            return edit;
        };

    m_pWeldPoseRxEdit = makeEdit();
    m_pWeldPoseRyEdit = makeEdit();
    m_pWeldPoseRzEdit = makeEdit();

    auto addPoseItem = [layout](int col, const QString& labelText, QWidget* editor)
        {
            QLabel* label = new QLabel(labelText);
            layout->addWidget(label, 2, col);
            layout->addWidget(CreateUnitEditorForPrecise(editor, "deg"), 2, col + 1);
        };

    addPoseItem(0, "RX：", m_pWeldPoseRxEdit);
    addPoseItem(2, "RY：", m_pWeldPoseRyEdit);
    addPoseItem(4, "RZ：", m_pWeldPoseRzEdit);

    QPushButton* teachBtn = new QPushButton("示教当前位置为焊接平台标准姿态");
    teachBtn->setMinimumHeight(36);
    teachBtn->setMinimumWidth(280);
    layout->addWidget(teachBtn, 2, 6, 1, 2, Qt::AlignLeft);

    connect(m_pUseTaughtWeldPoseCheck, &QCheckBox::toggled, this, [this](bool checked)
        {
            if (!m_bLoading)
            {
                CacheCurrentWeldPoseEditorValues(!checked);
                ShowWeldPoseEditorValuesForMode(checked);
            }
            UpdateWeldPoseTeachUiState();
        });
    connect(teachBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::TeachCurrentWeldPlatformPose);

    for (int i = 0; i < 8; ++i)
    {
        layout->setColumnStretch(i, (i % 2) == 0 ? 0 : 1);
    }
    UpdateWeldPoseTeachUiState();
    return groupBox;
}

void PreciseMeasureEditDialog::CacheCurrentWeldPoseEditorValues(bool currentModeIsTaught)
{
    auto readValue = [](QLineEdit* edit, double fallback) -> double
        {
            if (edit == nullptr)
            {
                return fallback;
            }
            bool ok = false;
            const double value = edit->text().trimmed().toDouble(&ok);
            return ok && std::isfinite(value) ? value : fallback;
        };

    if (currentModeIsTaught)
    {
        m_taughtWeldPoseRxDeg = readValue(m_pWeldPoseRxEdit, m_taughtWeldPoseRxDeg);
        m_taughtWeldPoseRyDeg = readValue(m_pWeldPoseRyEdit, m_taughtWeldPoseRyDeg);
        m_taughtWeldPoseRzDeg = readValue(m_pWeldPoseRzEdit, m_taughtWeldPoseRzDeg);
    }
    else
    {
        m_normalWeldRxDeg = readValue(m_pWeldPoseRxEdit, m_normalWeldRxDeg);
        m_normalWeldRyDeg = readValue(m_pWeldPoseRyEdit, m_normalWeldRyDeg);
        m_weldRzGainDeg = readValue(m_pWeldPoseRzEdit, m_weldRzGainDeg);
    }
}

void PreciseMeasureEditDialog::ShowWeldPoseEditorValuesForMode(bool useTaught)
{
    const double rx = useTaught ? m_taughtWeldPoseRxDeg : m_normalWeldRxDeg;
    const double ry = useTaught ? m_taughtWeldPoseRyDeg : m_normalWeldRyDeg;
    const double rz = useTaught ? m_taughtWeldPoseRzDeg : m_weldRzGainDeg;
    if (m_pWeldPoseRxEdit != nullptr)
    {
        m_pWeldPoseRxEdit->setText(QString::number(rx, 'f', 6));
    }
    if (m_pWeldPoseRyEdit != nullptr)
    {
        m_pWeldPoseRyEdit->setText(QString::number(ry, 'f', 6));
    }
    if (m_pWeldPoseRzEdit != nullptr)
    {
        m_pWeldPoseRzEdit->setText(QString::number(rz, 'f', 6));
    }
}

void PreciseMeasureEditDialog::UpdateWeldPoseTeachUiState()
{
    if (m_pWeldPoseHintLabel == nullptr || m_pUseTaughtWeldPoseCheck == nullptr)
    {
        return;
    }

    if (m_pUseTaughtWeldPoseCheck->isChecked())
    {
        m_pWeldPoseHintLabel->setText(
            "启用后：先测后焊生成焊道姿态时 RX/RY 直接使用示教值；平台 RZ 使用示教 RZ，坡道 RZ 按“计算RZ - (计算平台RZ - 示教RZ)”修正。");
    }
    else
    {
        m_pWeldPoseHintLabel->setText(
            "未启用时：下面 RX/RY/RZ 分别编辑旧焊接姿态参数 NormalWeldRx、NormalWeldRy、WeldRzGainDeg，流程继续按原逻辑生成姿态。");
    }
}

bool PreciseMeasureEditDialog::LoadWeldPoseTeachParams()
{
    if (m_pUseTaughtWeldPoseCheck == nullptr)
    {
        return true;
    }

    QString error;
    const QString path = CurrentParamFilePath();
    const QString section = CurrentWeldSectionName(&error);
    if (path.isEmpty() || section.isEmpty())
    {
        AppendLog("读取焊接姿态示教参数失败：" + error);
        return false;
    }

    int useTaught = 0;
    m_normalWeldRxDeg = 0.0;
    m_normalWeldRyDeg = 0.0;
    m_weldRzGainDeg = 0.0;
    m_taughtWeldPoseRxDeg = m_normalWeldRxDeg;
    m_taughtWeldPoseRyDeg = m_normalWeldRyDeg;
    m_taughtWeldPoseRzDeg = 0.0;

    COPini ini;
    if (ini.SetFileName(path.toUtf8().constData()))
    {
        ini.SetSectionName(ToUtf8StdStringForPrecise(section));
        ini.ReadString(false, USE_TAUGHT_WELD_POSE_KEY, &useTaught);
        ini.ReadString(false, NORMAL_WELD_RX_KEY, &m_normalWeldRxDeg);
        ini.ReadString(false, NORMAL_WELD_RY_KEY, &m_normalWeldRyDeg);
        ini.ReadString(false, WELD_RZ_GAIN_KEY, &m_weldRzGainDeg);
        m_taughtWeldPoseRxDeg = m_normalWeldRxDeg;
        m_taughtWeldPoseRyDeg = m_normalWeldRyDeg;
        ini.ReadString(false, TAUGHT_WELD_POSE_RX_KEY, &m_taughtWeldPoseRxDeg);
        ini.ReadString(false, TAUGHT_WELD_POSE_RY_KEY, &m_taughtWeldPoseRyDeg);
        ini.ReadString(false, TAUGHT_WELD_POSE_RZ_KEY, &m_taughtWeldPoseRzDeg);
    }
    else
    {
        AppendLog("读取焊接姿态示教参数失败：打开参数文件失败：" + path);
        return false;
    }

    auto sanitize = [](double value) -> double
        {
            return std::isfinite(value) ? value : 0.0;
        };
    m_normalWeldRxDeg = sanitize(m_normalWeldRxDeg);
    m_normalWeldRyDeg = sanitize(m_normalWeldRyDeg);
    m_weldRzGainDeg = sanitize(m_weldRzGainDeg);
    m_taughtWeldPoseRxDeg = sanitize(m_taughtWeldPoseRxDeg);
    m_taughtWeldPoseRyDeg = sanitize(m_taughtWeldPoseRyDeg);
    m_taughtWeldPoseRzDeg = sanitize(m_taughtWeldPoseRzDeg);

    QSignalBlocker blockUse(m_pUseTaughtWeldPoseCheck);
    m_pUseTaughtWeldPoseCheck->setChecked(useTaught != 0);
    ShowWeldPoseEditorValuesForMode(useTaught != 0);
    UpdateWeldPoseTeachUiState();
    return true;
}

bool PreciseMeasureEditDialog::SaveWeldPoseTeachParams(QString& error)
{
    if (m_pUseTaughtWeldPoseCheck == nullptr)
    {
        return true;
    }

    const bool useTaught = m_pUseTaughtWeldPoseCheck->isChecked();
    CacheCurrentWeldPoseEditorValues(useTaught);

    const QString section = CurrentWeldSectionName(&error);
    if (section.isEmpty())
    {
        return false;
    }

    return WriteParamValue(section, USE_TAUGHT_WELD_POSE_KEY, useTaught ? "1" : "0", error)
        && WriteParamValue(section, NORMAL_WELD_RX_KEY, QString::number(m_normalWeldRxDeg, 'f', 6), error)
        && WriteParamValue(section, NORMAL_WELD_RY_KEY, QString::number(m_normalWeldRyDeg, 'f', 6), error)
        && WriteParamValue(section, WELD_RZ_GAIN_KEY, QString::number(m_weldRzGainDeg, 'f', 6), error)
        && WriteParamValue(section, TAUGHT_WELD_POSE_RX_KEY, QString::number(m_taughtWeldPoseRxDeg, 'f', 6), error)
        && WriteParamValue(section, TAUGHT_WELD_POSE_RY_KEY, QString::number(m_taughtWeldPoseRyDeg, 'f', 6), error)
        && WriteParamValue(section, TAUGHT_WELD_POSE_RZ_KEY, QString::number(m_taughtWeldPoseRzDeg, 'f', 6), error);
}

void PreciseMeasureEditDialog::TeachCurrentWeldPlatformPose()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const bool wasUsingTaught = m_pUseTaughtWeldPoseCheck != nullptr && m_pUseTaughtWeldPoseCheck->isChecked();
    CacheCurrentWeldPoseEditorValues(wasUsingTaught);
    const T_ROBOT_COORS coors = driver->GetCurrentPos();
    m_taughtWeldPoseRxDeg = coors.dRX;
    m_taughtWeldPoseRyDeg = coors.dRY;
    m_taughtWeldPoseRzDeg = coors.dRZ;

    if (m_pUseTaughtWeldPoseCheck != nullptr)
    {
        QSignalBlocker blockUse(m_pUseTaughtWeldPoseCheck);
        m_pUseTaughtWeldPoseCheck->setChecked(true);
    }
    ShowWeldPoseEditorValuesForMode(true);
    UpdateWeldPoseTeachUiState();
    AppendLog(QString("已读取当前机器人姿态为焊接平台标准姿态：RX=%1, RY=%2, RZ=%3，点击“保存参数”后写入文件。")
        .arg(m_taughtWeldPoseRxDeg, 0, 'f', 6)
        .arg(m_taughtWeldPoseRyDeg, 0, 'f', 6)
        .arg(m_taughtWeldPoseRzDeg, 0, 'f', 6));
}

void PreciseMeasureEditDialog::UpdateScanSafePreview()
{
    if (m_pScanSafePreviewLabel == nullptr
        || m_pScanSafeOffsetSpin == nullptr
        || m_pScanSafeGunAngleSpin == nullptr
        || m_pScanSafeXDirectionCombo == nullptr)
    {
        return;
    }

    T_ROBOT_COORS startPos;
    T_ROBOT_COORS endPos;
    QString error;
    const bool hasStart = GetCoorsFromEditors("StartPos", startPos, error);
    const bool hasEnd = GetCoorsFromEditors("EndPos", endPos, error);
    const double distance = m_pScanSafeOffsetSpin->value();
    constexpr double kPi = 3.14159265358979323846;
    const double angleRad = m_pScanSafeGunAngleSpin->value() * kPi / 180.0;
    const double xSign = m_pScanSafeXDirectionCombo->currentData().toInt() >= 0 ? 1.0 : -1.0;
    const double dx = xSign * distance * std::sin(angleRad);
    const double dz = distance * std::cos(angleRad);

    if (m_pUseComputedScanSafeCheck != nullptr && !m_pUseComputedScanSafeCheck->isChecked())
    {
        m_pScanSafePreviewLabel->setText("当前使用示教的下枪/收枪安全位置；扫描安全位推算参数会保存，但流程暂不使用。");
        return;
    }
    if (!hasStart || !hasEnd)
    {
        m_pScanSafePreviewLabel->setText("当前使用推算扫描安全位；请先读取或示教扫描起点、扫描终点后查看预估位置。");
        return;
    }

    m_pScanSafePreviewLabel->setText(QString("当前计算：X偏移=%1 mm，Z抬高=%2 mm。起点安全位≈ X=%3, Y=%4, Z=%5；终点安全位≈ X=%6, Y=%7, Z=%8")
        .arg(dx, 0, 'f', 3)
        .arg(dz, 0, 'f', 3)
        .arg(startPos.dX + dx, 0, 'f', 3)
        .arg(startPos.dY, 0, 'f', 3)
        .arg(startPos.dZ + dz, 0, 'f', 3)
        .arg(endPos.dX + dx, 0, 'f', 3)
        .arg(endPos.dY, 0, 'f', 3)
        .arg(endPos.dZ + dz, 0, 'f', 3));
}

void PreciseMeasureEditDialog::UpdateComputedScanSafeUiState()
{
    const bool useComputed = m_pUseComputedScanSafeCheck != nullptr && m_pUseComputedScanSafeCheck->isChecked();
    if (m_pStartSafePulseGroup != nullptr)
    {
        m_pStartSafePulseGroup->setEnabled(!useComputed);
        m_pStartSafePulseGroup->setToolTip(useComputed ? "当前使用起点直角位姿推算扫描下枪安全位，不使用此示教脉冲。" : QString());
    }
    if (m_pEndSafePulseGroup != nullptr)
    {
        m_pEndSafePulseGroup->setEnabled(!useComputed);
        m_pEndSafePulseGroup->setToolTip(useComputed ? "当前使用终点直角位姿推算扫描收枪安全位，不使用此示教脉冲。" : QString());
    }
}

bool PreciseMeasureEditDialog::WriteParamValue(const QString& sectionName, const QString& key, const QString& value, QString& error) const
{
    if (IsObsoletePreciseParamKey(key))
    {
        return true;
    }
    if (CurrentRobotName().isEmpty() || sectionName.isEmpty() || key.isEmpty())
    {
        error = "未选择机器人或参数项为空。";
        return false;
    }
    return RobotDataHelper::WriteParamValue(CurrentParamFilePath(), sectionName, key, value, &error);
}

bool PreciseMeasureEditDialog::SaveGroupMetadata(QString& error) const
{
    const QString path = CurrentParamFilePath();
    if (path.isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    const int groupIndex = CurrentGroupIndex();
    const QString groupName = m_pGroupNameEdit != nullptr && !m_pGroupNameEdit->text().trimmed().isEmpty()
        ? m_pGroupNameEdit->text().trimmed()
        : QString("参数组%1").arg(groupIndex + 1);
    if (!RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), "UseGroupNo", QString::number(groupIndex), &error))
    {
        return false;
    }
    if (!RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), QString("Group%1Name").arg(groupIndex), groupName, &error))
    {
        return false;
    }
    if (m_pGroupCombo != nullptr && m_pGroupCombo->currentIndex() >= 0)
    {
        m_pGroupCombo->setItemText(m_pGroupCombo->currentIndex(), QString("%1 / Group%2").arg(groupName).arg(groupIndex));
    }
    return true;
}

bool PreciseMeasureEditDialog::CreateParamGroup(bool copyCurrent, QString& error)
{
    const QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    if (!RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &error))
    {
        return false;
    }

    const QString path = CurrentParamFilePath();
    COPini ini;
    if (!ini.SetFileName(path.toUtf8().constData()))
    {
        error = "打开参数文件失败：" + path;
        return false;
    }
    int groupCount = 1;
    ini.SetSectionName(ToUtf8StdStringForPrecise(GroupMetaSectionName()));
    ini.ReadString(false, "GroupCount", &groupCount);
    groupCount = std::max(1, groupCount);
    const int newIndex = groupCount;
    const int sourceIndex = CurrentGroupIndex();
    QMap<QString, QString> scanValues = ReadSectionMapForPrecise(path, RobotDataHelper::MeasureWeldScanSectionName(sourceIndex), MinimalScanSectionLinesForPrecise());
    QMap<QString, QString> weldValues = ReadSectionMapForPrecise(path, RobotDataHelper::MeasureWeldWeldSectionName(sourceIndex), MinimalWeldSectionLinesForPrecise());
    if (!copyCurrent)
    {
        scanValues = ZeroSectionValuesForPrecise(scanValues);
        weldValues = ZeroSectionValuesForPrecise(weldValues);
    }

    if (!RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), "GroupCount", QString::number(newIndex + 1), &error)
        || !RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), "UseGroupNo", QString::number(newIndex), &error)
        || !RobotDataHelper::WriteParamValue(path, GroupMetaSectionName(), QString("Group%1Name").arg(newIndex), QString("参数组%1").arg(newIndex + 1), &error))
    {
        return false;
    }

    if (!WriteSectionMapForPrecise(path, RobotDataHelper::MeasureWeldScanSectionName(newIndex), scanValues, error)
        || !WriteSectionMapForPrecise(path, RobotDataHelper::MeasureWeldWeldSectionName(newIndex), weldValues, error))
    {
        return false;
    }
    LoadParamGroups();
    return true;
}

QGroupBox* PreciseMeasureEditDialog::CreatePulseGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)())
{
    Q_UNUSED(saveSlot);
    const QStringList axes = { "nS", "nL", "nU", "nR", "nB", "nT", "lBX", "lBY", "lBZ" };
    QGroupBox* groupBox = new QGroupBox(title);
    if (groupName == "StartSafePulse0")
    {
        m_pStartSafePulseGroup = groupBox;
    }
    else if (groupName == "EndSafePulse0")
    {
        m_pEndSafePulseGroup = groupBox;
    }
    groupBox->setFixedHeight(248);
    groupBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    QGridLayout* layout = new QGridLayout(groupBox);
    layout->setContentsMargins(36, 28, 36, 16);
    layout->setVerticalSpacing(10);
    layout->setHorizontalSpacing(8);
    for (int i = 0; i < axes.size(); ++i)
    {
        QLabel* label = new QLabel(axes[i]);
        QLineEdit* edit = CreateValueEdit();
        edit->setValidator(CreateIntValidator(edit));
        MarkNumericEdit(edit);
        m_editors.insert(AxisKey(groupName, axes[i]), edit);
        layout->addWidget(label, i / 3, (i % 3) * 2);
        layout->addWidget(edit, i / 3, (i % 3) * 2 + 1);
    }
    QPushButton* teachBtn = new QPushButton(teachText);
    teachBtn->setFixedWidth(220);
    teachBtn->setMinimumHeight(34);
    teachBtn->setMaximumHeight(40);
    layout->addWidget(teachBtn, 3, 0, 1, 6, Qt::AlignLeft);
    connect(teachBtn, &QPushButton::clicked, this, teachSlot);
    return groupBox;
}

QGroupBox* PreciseMeasureEditDialog::CreateCoorsGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)())
{
    Q_UNUSED(saveSlot);
    const QStringList axes = { "X", "Y", "Z", "RX", "RY", "RZ", "BX", "BY", "BZ" };
    QGroupBox* groupBox = new QGroupBox(title);
    groupBox->setFixedHeight(248);
    groupBox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    QGridLayout* layout = new QGridLayout(groupBox);
    layout->setContentsMargins(36, 28, 36, 16);
    layout->setVerticalSpacing(10);
    layout->setHorizontalSpacing(8);
    for (int i = 0; i < axes.size(); ++i)
    {
        QLabel* label = new QLabel(axes[i]);
        QLineEdit* edit = CreateValueEdit(88, 118);
        edit->setValidator(CreateDoubleValidator(edit));
        MarkNumericEdit(edit);
        m_editors.insert(AxisKey(groupName, axes[i]), edit);
        if (m_bPositionTeachOnly && (groupName == "StartPos" || groupName == "EndPos"))
        {
            connect(edit, &QLineEdit::textChanged, this, &PreciseMeasureEditDialog::UpdateScanSafePreview);
        }
        layout->addWidget(label, i / 3, (i % 3) * 2);
        layout->addWidget(edit, i / 3, (i % 3) * 2 + 1);
    }
    QPushButton* teachBtn = new QPushButton(teachText);
    teachBtn->setFixedWidth(220);
    teachBtn->setMinimumHeight(34);
    teachBtn->setMaximumHeight(40);
    layout->addWidget(teachBtn, 3, 0, 1, 6, Qt::AlignLeft);
    connect(teachBtn, &QPushButton::clicked, this, teachSlot);
    return groupBox;
}

void PreciseMeasureEditDialog::SetPulseEditors(const QString& groupName, const T_ANGLE_PULSE& pulse)
{
    const QMap<QString, long> values = {
        { "nS", pulse.nSPulse },
        { "nL", pulse.nLPulse },
        { "nU", pulse.nUPulse },
        { "nR", pulse.nRPulse },
        { "nB", pulse.nBPulse },
        { "nT", pulse.nTPulse },
        { "lBX", pulse.lBXPulse },
        { "lBY", pulse.lBYPulse },
        { "lBZ", pulse.lBZPulse },
    };

    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        QLineEdit* edit = m_editors.value(AxisKey(groupName, it.key()), nullptr);
        if (edit != nullptr)
        {
            edit->setText(QString::number(it.value()));
        }
    }
}

void PreciseMeasureEditDialog::SetCoorsEditors(const QString& groupName, const T_ROBOT_COORS& coors)
{
    const QMap<QString, double> values = {
        { "X", coors.dX },
        { "Y", coors.dY },
        { "Z", coors.dZ },
        { "RX", coors.dRX },
        { "RY", coors.dRY },
        { "RZ", coors.dRZ },
        { "BX", coors.dBX },
        { "BY", coors.dBY },
        { "BZ", coors.dBZ },
    };

    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        QLineEdit* edit = m_editors.value(AxisKey(groupName, it.key()), nullptr);
        if (edit != nullptr)
        {
            edit->setText(QString::number(it.value(), 'f', 6));
        }
    }
}

bool PreciseMeasureEditDialog::GetPulseFromEditors(const QString& groupName, T_ANGLE_PULSE& pulse, QString& error) const
{
    auto readValue = [this, groupName, &error](const QString& axis, long& value) -> bool
        {
            QLineEdit* edit = m_editors.value(AxisKey(groupName, axis), nullptr);
            if (edit == nullptr)
            {
                error = "找不到输入框：" + groupName + "." + axis;
                return false;
            }
            bool ok = false;
            const long parsed = edit->text().trimmed().toLong(&ok);
            if (!ok)
            {
                error = "输入不是整数：" + groupName + "." + axis;
                return false;
            }
            value = parsed;
            return true;
        };

    return readValue("nS", pulse.nSPulse)
        && readValue("nL", pulse.nLPulse)
        && readValue("nU", pulse.nUPulse)
        && readValue("nR", pulse.nRPulse)
        && readValue("nB", pulse.nBPulse)
        && readValue("nT", pulse.nTPulse)
        && readValue("lBX", pulse.lBXPulse)
        && readValue("lBY", pulse.lBYPulse)
        && readValue("lBZ", pulse.lBZPulse);
}

bool PreciseMeasureEditDialog::GetCoorsFromEditors(const QString& groupName, T_ROBOT_COORS& coors, QString& error) const
{
    auto readValue = [this, groupName, &error](const QString& axis, double& value) -> bool
        {
            QLineEdit* edit = m_editors.value(AxisKey(groupName, axis), nullptr);
            if (edit == nullptr)
            {
                error = "找不到输入框：" + groupName + "." + axis;
                return false;
            }
            bool ok = false;
            const double parsed = edit->text().trimmed().toDouble(&ok);
            if (!ok)
            {
                error = "输入不是数字：" + groupName + "." + axis;
                return false;
            }
            value = parsed;
            return true;
        };

    return readValue("X", coors.dX)
        && readValue("Y", coors.dY)
        && readValue("Z", coors.dZ)
        && readValue("RX", coors.dRX)
        && readValue("RY", coors.dRY)
        && readValue("RZ", coors.dRZ)
        && readValue("BX", coors.dBX)
        && readValue("BY", coors.dBY)
        && readValue("BZ", coors.dBZ);
}

void PreciseMeasureEditDialog::SetEditorsBlocked(bool blocked)
{
    m_bLoading = blocked;
    for (QLineEdit* edit : m_editors)
    {
        if (edit != nullptr)
        {
            edit->blockSignals(blocked);
        }
    }
    for (QComboBox* combo : m_otherParamComboEditors)
    {
        if (combo != nullptr)
        {
            combo->blockSignals(blocked);
        }
    }
    if (m_pUseComputedScanSafeCheck != nullptr)
    {
        m_pUseComputedScanSafeCheck->blockSignals(blocked);
    }
    if (m_pScanSafeOffsetSpin != nullptr)
    {
        m_pScanSafeOffsetSpin->blockSignals(blocked);
    }
    if (m_pScanSafeGunAngleSpin != nullptr)
    {
        m_pScanSafeGunAngleSpin->blockSignals(blocked);
    }
    if (m_pScanSafeXDirectionCombo != nullptr)
    {
        m_pScanSafeXDirectionCombo->blockSignals(blocked);
    }
    if (m_pScanSafeLiftSpin != nullptr)
    {
        m_pScanSafeLiftSpin->blockSignals(blocked);
    }
    if (m_pScanSafeFlipWarnSpin != nullptr)
    {
        m_pScanSafeFlipWarnSpin->blockSignals(blocked);
    }
}

void PreciseMeasureEditDialog::ClearOtherParamEditors()
{
    m_otherParamEditors.clear();
    m_otherParamComboEditors.clear();
    m_otherParamSectionWidgets.clear();
    if (m_pOtherParamLayout == nullptr)
    {
        return;
    }

    while (QLayoutItem* item = m_pOtherParamLayout->takeAt(0))
    {
        if (QWidget* widget = item->widget())
        {
            widget->deleteLater();
        }
        delete item;
    }
}

void PreciseMeasureEditDialog::SwitchOtherParamPage(bool showScanPage)
{
    m_showScanParamPage = showScanPage;
    if (m_pScanParamTabBtn != nullptr)
    {
        m_pScanParamTabBtn->setChecked(showScanPage);
    }
    if (m_pWeldParamTabBtn != nullptr)
    {
        m_pWeldParamTabBtn->setChecked(!showScanPage);
    }
    UpdateOtherParamPageVisibility();
}

void PreciseMeasureEditDialog::UpdateOtherParamPageVisibility()
{
    const QString activePage = m_showScanParamPage ? QStringLiteral("扫描参数") : QStringLiteral("焊接参数");
    for (QWidget* widget : m_otherParamSectionWidgets)
    {
        if (widget == nullptr)
        {
            continue;
        }
        widget->setVisible(widget->property("paramPage").toString() == activePage);
    }
}

void PreciseMeasureEditDialog::UpdateAdaptiveLayout()
{
    if (m_pContentSplitter == nullptr)
    {
        return;
    }

    const bool wide = width() >= 1050;
    RebuildPulseGroupLayout(wide);

    const Qt::Orientation targetOrientation = wide ? Qt::Horizontal : Qt::Vertical;
    if (m_pContentSplitter->orientation() != targetOrientation)
    {
        m_pContentSplitter->setOrientation(targetOrientation);
    }

    if (wide)
    {
        const int totalWidth = qMax(1, m_pContentSplitter->width());
        const int rightWidth = qBound(620, totalWidth / 3, 700);
        if (m_pOtherPanel != nullptr)
        {
            m_pOtherPanel->setMinimumWidth(rightWidth);
            m_pOtherPanel->setMaximumWidth(QWIDGETSIZE_MAX);
        }
        if (m_pPulsePanel != nullptr)
        {
            m_pPulsePanel->setMinimumWidth(0);
            m_pPulsePanel->setMaximumWidth(QWIDGETSIZE_MAX);
        }
        m_pContentSplitter->setSizes(QList<int>() << qMax(1, totalWidth - rightWidth) << rightWidth);
    }
    else
    {
        if (m_pOtherPanel != nullptr)
        {
            m_pOtherPanel->setMinimumWidth(0);
            m_pOtherPanel->setMaximumWidth(QWIDGETSIZE_MAX);
        }
        if (m_pPulsePanel != nullptr)
        {
            m_pPulsePanel->setMinimumWidth(0);
            m_pPulsePanel->setMaximumWidth(QWIDGETSIZE_MAX);
        }
        m_pContentSplitter->setSizes(QList<int>() << height() * 2 / 3 << height() / 3);
    }
}

void PreciseMeasureEditDialog::RebuildPulseGroupLayout(bool wide)
{
    if (m_pPulseGroupsLayout == nullptr)
    {
        return;
    }
    if (m_pPulseGroupsLayout->count() > 0 && m_bWideAdaptiveLayout == wide)
    {
        return;
    }

    while (QLayoutItem* item = m_pPulseGroupsLayout->takeAt(0))
    {
        delete item;
    }

    for (int i = 0; i < m_pulseGroupWidgets.size(); ++i)
    {
        QWidget* widget = m_pulseGroupWidgets[i];
        if (wide && widget == m_pScanSafeParamPanel)
        {
            m_pPulseGroupsLayout->addWidget(widget, (i + 1) / 2, 0, 1, 2, Qt::AlignTop);
            continue;
        }
        if (wide)
        {
            m_pPulseGroupsLayout->addWidget(widget, i / 2, i % 2, Qt::AlignTop);
        }
        else
        {
            m_pPulseGroupsLayout->addWidget(widget, i, 0, Qt::AlignTop);
        }
    }

    m_pPulseGroupsLayout->setColumnStretch(0, 1);
    m_pPulseGroupsLayout->setColumnStretch(1, wide ? 1 : 0);
    m_pPulseGroupsLayout->setRowStretch(wide ? 3 : m_pulseGroupWidgets.size(), 1);
    m_bWideAdaptiveLayout = wide;
}

void PreciseMeasureEditDialog::SaveOtherParamEdit()
{
    if (m_bLoading)
    {
        return;
    }

    QLineEdit* edit = qobject_cast<QLineEdit*>(sender());
    if (edit == nullptr)
    {
        return;
    }

    QString error;
    const QString sectionName = edit->property("paramSection").toString();
    const QString key = edit->property("paramKey").toString();
    const QString value = ValueForWriteWithInlineComment(edit);
    if (!WriteParamValue(sectionName, key, value, error))
    {
        AppendLog("其它参数保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog(QString("其它参数已保存：%1=%2").arg(PreciseParamDisplayName(key), value));
}

void PreciseMeasureEditDialog::SaveOtherParamComboEdit(QComboBox* combo)
{
    if (m_bLoading || combo == nullptr)
    {
        return;
    }

    QString error;
    const QString sectionName = combo->property("paramSection").toString();
    const QString key = combo->property("paramKey").toString();
    const QString value = ValueForWriteWithInlineComment(combo);
    if (!WriteParamValue(sectionName, key, value, error))
    {
        AppendLog("其它参数保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog(QString("其它参数已保存：%1=%2").arg(PreciseParamDisplayName(key), combo->currentText()));
}

bool PreciseMeasureEditDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString PreciseMeasureEditDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << CurrentRobotName();
    fields << QString::number(CurrentGroupIndex());
    fields << (m_pGroupNameEdit != nullptr ? m_pGroupNameEdit->text().trimmed() : QString());
    for (auto it = m_editors.cbegin(); it != m_editors.cend(); ++it)
    {
        fields << it.key() << (it.value() != nullptr ? it.value()->text().trimmed() : QString());
    }
    for (auto it = m_otherParamEditors.cbegin(); it != m_otherParamEditors.cend(); ++it)
    {
        fields << it.key() << (it.value() != nullptr ? it.value()->text().trimmed() : QString());
    }
    for (auto it = m_otherParamComboEditors.cbegin(); it != m_otherParamComboEditors.cend(); ++it)
    {
        fields << it.key() << (it.value() != nullptr ? it.value()->currentData().toString() : QString());
    }
    if (m_pUseComputedScanSafeCheck != nullptr)
    {
        fields << "ScanSafe.UseComputed" << (m_pUseComputedScanSafeCheck->isChecked() ? "1" : "0");
        fields << "ScanSafe.Offset" << QString::number(m_pScanSafeOffsetSpin != nullptr ? m_pScanSafeOffsetSpin->value() : 0.0, 'f', 6);
        fields << "ScanSafe.GunAngle" << QString::number(m_pScanSafeGunAngleSpin != nullptr ? m_pScanSafeGunAngleSpin->value() : 0.0, 'f', 6);
        fields << "ScanSafe.XDirection" << QString::number(m_pScanSafeXDirectionCombo != nullptr ? m_pScanSafeXDirectionCombo->currentData().toInt() : -1);
        fields << "ScanSafe.LiftHeight" << QString::number(m_pScanSafeLiftSpin != nullptr ? m_pScanSafeLiftSpin->value() : 0.0, 'f', 6);
        fields << "ScanSafe.FlipWarn" << QString::number(m_pScanSafeFlipWarnSpin != nullptr ? m_pScanSafeFlipWarnSpin->value() : 0.0, 'f', 6);
    }
    if (m_pUseTaughtWeldPoseCheck != nullptr)
    {
        fields << "WeldPose.UseTaught" << (m_pUseTaughtWeldPoseCheck->isChecked() ? "1" : "0");
        fields << "WeldPose.RX" << (m_pWeldPoseRxEdit != nullptr ? m_pWeldPoseRxEdit->text().trimmed() : QString());
        fields << "WeldPose.RY" << (m_pWeldPoseRyEdit != nullptr ? m_pWeldPoseRyEdit->text().trimmed() : QString());
        fields << "WeldPose.RZ" << (m_pWeldPoseRzEdit != nullptr ? m_pWeldPoseRzEdit->text().trimmed() : QString());
    }
    return fields.join('\n');
}

void PreciseMeasureEditDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void PreciseMeasureEditDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }
}
