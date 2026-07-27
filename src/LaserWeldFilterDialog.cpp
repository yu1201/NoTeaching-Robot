#include "LaserWeldFilterDialog.h"

#include "PointCloudProcessingConfig.h"
#include "WindowStyleHelper.h"

#include <QCheckBox>
#include <QComboBox>
#include <QByteArray>
#include <QByteArrayList>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextDocument>
#include <QVBoxLayout>

#include <cmath>
#include <utility>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
enum class PlainIniEncoding
{
    Utf8,
    Gbk,
    Local
};

struct PlainIniValueUpdate
{
    QString key;
    QString value;
};



#ifdef Q_OS_WIN
QString DecodeWindowsCodePage(const QByteArray& bytes, UINT codePage, DWORD flags, bool* ok = nullptr)
{
    if (ok != nullptr)
    {
        *ok = false;
    }
    if (bytes.isEmpty())
    {
        if (ok != nullptr)
        {
            *ok = true;
        }
        return QString();
    }

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
    const int convertedLength = MultiByteToWideChar(
        codePage,
        flags,
        bytes.constData(),
        bytes.size(),
        wideText.data(),
        wideLength);
    if (convertedLength <= 0)
    {
        return QString();
    }

    if (ok != nullptr)
    {
        *ok = true;
    }
    QString text = QString::fromWCharArray(wideText.data(), convertedLength);
    if (!text.isEmpty() && text.front() == QChar(0xfeff))
    {
        text.remove(0, 1);
    }
    return text;
}

QByteArray EncodeWindowsCodePage(const QString& text, UINT codePage)
{
    if (text.isEmpty())
    {
        return QByteArray();
    }

    const int byteLength = WideCharToMultiByte(
        codePage,
        0,
        reinterpret_cast<LPCWCH>(text.utf16()),
        text.size(),
        nullptr,
        0,
        nullptr,
        nullptr);
    if (byteLength <= 0)
    {
        return QByteArray();
    }

    QByteArray bytes(byteLength, Qt::Uninitialized);
    const int convertedLength = WideCharToMultiByte(
        codePage,
        0,
        reinterpret_cast<LPCWCH>(text.utf16()),
        text.size(),
        bytes.data(),
        bytes.size(),
        nullptr,
        nullptr);
    if (convertedLength <= 0)
    {
        return QByteArray();
    }
    bytes.truncate(convertedLength);
    return bytes;
}
#endif

bool IsValidUtf8Bytes(const QByteArray& bytes)
{
#ifdef Q_OS_WIN
    bool ok = false;
    DecodeWindowsCodePage(bytes, CP_UTF8, MB_ERR_INVALID_CHARS, &ok);
    return ok;
#else
    const QString text = QString::fromUtf8(bytes);
    return !text.contains(QChar(0xfffd));
#endif
}

PlainIniEncoding DetectPlainIniEncoding(const QByteArray& content)
{
    if (content.isEmpty() || IsValidUtf8Bytes(content))
    {
        return PlainIniEncoding::Utf8;
    }
#ifdef Q_OS_WIN
    bool gbkOk = false;
    DecodeWindowsCodePage(content, 936, 0, &gbkOk);
    if (gbkOk)
    {
        return PlainIniEncoding::Gbk;
    }
#endif
    return PlainIniEncoding::Local;
}

QString DecodePlainIniBytes(const QByteArray& bytes)
{
    if (bytes.isEmpty())
    {
        return QString();
    }
#ifdef Q_OS_WIN
    bool utf8Ok = false;
    const QString utf8Text = DecodeWindowsCodePage(bytes, CP_UTF8, MB_ERR_INVALID_CHARS, &utf8Ok);
    if (utf8Ok && !utf8Text.contains(QChar(0xfffd)))
    {
        return utf8Text;
    }

    bool gbkOk = false;
    const QString gbkText = DecodeWindowsCodePage(bytes, 936, 0, &gbkOk);
    if (gbkOk && !gbkText.contains(QChar(0xfffd)))
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

QByteArray EncodePlainIniText(const QString& text, PlainIniEncoding encoding)
{
    switch (encoding)
    {
    case PlainIniEncoding::Utf8:
        return text.toUtf8();
    case PlainIniEncoding::Gbk:
#ifdef Q_OS_WIN
        return EncodeWindowsCodePage(text, 936);
#else
        return text.toLocal8Bit();
#endif
    case PlainIniEncoding::Local:
    default:
        return text.toLocal8Bit();
    }
}

QString ReadPlainIniValue(const QString& filePath, const QString& key, const QString& defaultValue)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        return defaultValue;
    }

    const QList<QByteArray> lines = file.readAll().split('\n');
    const QByteArray keyBytes = key.toLatin1();
    for (QByteArray line : lines)
    {
        if (line.endsWith('\r'))
        {
            line.chop(1);
        }
        const QByteArray trimmed = line.trimmed();
        if (trimmed.isEmpty() || trimmed.startsWith("//") || trimmed.startsWith('#'))
        {
            continue;
        }
        const int equalsIndex = trimmed.indexOf('=');
        if (equalsIndex < 0)
        {
            continue;
        }
        const QByteArray currentKey = trimmed.left(equalsIndex).trimmed();
        if (currentKey.compare(keyBytes, Qt::CaseInsensitive) == 0)
        {
            return DecodePlainIniBytes(trimmed.mid(equalsIndex + 1).trimmed());
        }
    }
    return defaultValue;
}

bool WritePlainIniValues(const QString& filePath, const QVector<PlainIniValueUpdate>& updates, QString* error)
{
    QFile file(filePath);
    QByteArray content;
    if (file.exists())
    {
        if (!file.open(QIODevice::ReadOnly))
        {
            if (error != nullptr)
            {
                *error = QString("读取点云算法配置失败：%1").arg(file.errorString());
            }
            return false;
        }
        content = file.readAll();
        file.close();
    }

    const PlainIniEncoding encoding = DetectPlainIniEncoding(content);
    QByteArrayList lines = content.split('\n');
    QVector<bool> written(updates.size(), false);
    for (QByteArray& line : lines)
    {
        const bool hadCarriageReturn = line.endsWith('\r');
        QByteArray parseLine = line;
        if (hadCarriageReturn)
        {
            parseLine.chop(1);
        }
        const QByteArray trimmed = parseLine.trimmed();
        if (trimmed.isEmpty() || trimmed.startsWith("//") || trimmed.startsWith('#'))
        {
            continue;
        }
        const int equalsIndex = trimmed.indexOf('=');
        if (equalsIndex < 0)
        {
            continue;
        }
        const QByteArray currentKey = trimmed.left(equalsIndex).trimmed();
        for (int index = 0; index < updates.size(); ++index)
        {
            if (written[index])
            {
                continue;
            }
            if (currentKey.compare(updates[index].key.toLatin1(), Qt::CaseInsensitive) == 0)
            {
                const int rawEqualsIndex = parseLine.indexOf('=');
                QByteArray prefix = rawEqualsIndex >= 0
                    ? parseLine.left(rawEqualsIndex + 1)
                    : updates[index].key.toLatin1() + " =";
                if (rawEqualsIndex >= 0)
                {
                    const QByteArray afterEquals = parseLine.mid(rawEqualsIndex + 1);
                    int paddingLength = 0;
                    while (paddingLength < afterEquals.size()
                        && (afterEquals[paddingLength] == ' ' || afterEquals[paddingLength] == '\t'))
                    {
                        ++paddingLength;
                    }
                    prefix += afterEquals.left(paddingLength);
                }
                else
                {
                    prefix += ' ';
                }
                line = prefix + EncodePlainIniText(updates[index].value, encoding);
                if (hadCarriageReturn)
                {
                    line.append('\r');
                }
                written[index] = true;
                break;
            }
        }
    }

    for (int index = 0; index < updates.size(); ++index)
    {
        if (!written[index])
        {
            lines.append(updates[index].key.toLatin1() + " = " + EncodePlainIniText(updates[index].value, encoding));
        }
    }

    QFileInfo info(filePath);
    QDir().mkpath(info.absolutePath());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (error != nullptr)
        {
            *error = QString("写入点云算法配置失败：%1").arg(file.errorString());
        }
        return false;
    }
    file.write(lines.join('\n'));
    return true;
}

bool PlainIniBoolValue(const QString& text, bool defaultValue)
{
    const QString normalized = text.trimmed().toLower();
    if (normalized.isEmpty())
    {
        return defaultValue;
    }
    return normalized == "1" || normalized == "true" || normalized == "yes";
}

QString BoolIniText(bool value)
{
    return value ? "true" : "false";
}

QString TrimmedDecimalText(double value, int maxDecimals)
{
    QString text = QString::number(value, 'f', maxDecimals);
    if (text.contains('.'))
    {
        while (text.endsWith('0'))
        {
            text.chop(1);
        }
        if (text.endsWith('.'))
        {
            text.chop(1);
        }
    }
    return text == "-0" ? QString("0") : text;
}

QString FormatPlainIniNumberLikeCurrent(
    const QString& filePath,
    const QString& key,
    double value,
    int maxDecimals = 6)
{
    const QString currentText = ReadPlainIniValue(filePath, key, QString()).trimmed();
    bool numeric = false;
    currentText.toDouble(&numeric);
    if (numeric
        && !currentText.contains('.')
        && !currentText.contains('e', Qt::CaseInsensitive))
    {
        return QString::number(static_cast<long long>(std::llround(value)));
    }
    return TrimmedDecimalText(value, maxDecimals);
}

int ReadPlainIniIntValue(const QString& path, const QString& key, int defaultValue)
{
    bool ok = false;
    const double value = ReadPlainIniValue(path, key, QString::number(defaultValue)).toDouble(&ok);
    return ok ? static_cast<int>(std::round(value)) : defaultValue;
}

QWidget* CreateUnitEditor(QWidget* editor, const QString& unitText)
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

void PrepareWidePathEdit(QLineEdit* edit)
{
    if (edit == nullptr)
    {
        return;
    }
    edit->setMinimumWidth(680);
    edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void SetPathEditText(QLineEdit* edit, const QString& text)
{
    if (edit == nullptr)
    {
        return;
    }
    edit->setText(text);
    edit->setToolTip(text);
    edit->setCursorPosition(0);
}
}

LaserWeldFilterDialog::LaserWeldFilterDialog(
    std::function<bool()> liveSessionGuard,
    QWidget* parent)
    : QDialog(parent)
    , m_liveSessionGuard(std::move(liveSessionGuard))
{
    setWindowTitle("精测点云处理");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(980, 760), 0.84, 0.82);

    BuildUi();
    ApplyStyle();
    LoadSettings();
    connect(this, &QDialog::finished, this, [this](int) { SaveSettings(); });
}

void LaserWeldFilterDialog::BuildUi()
{
    QVBoxLayout* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(18, 18, 18, 18);
    outerLayout->setSpacing(12);

    QLabel* titleLabel = new QLabel("精测点云处理");
    titleLabel->setObjectName("TitleLabel");
    QLabel* hintLabel = new QLabel("管理预设参数扫描后的局部精测处理方式：四种方法分别使用 SDK 拐点、SDK 基础焊道、完整点云或相机特征点轨迹。");
    hintLabel->setObjectName("HintLabel");
    outerLayout->addWidget(titleLabel);
    outerLayout->addWidget(hintLabel);

    QScrollArea* settingsScrollArea = new QScrollArea(this);
    settingsScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(settingsScrollArea);

    QWidget* settingsContent = new QWidget(settingsScrollArea);
    settingsContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    QVBoxLayout* rootLayout = new QVBoxLayout(settingsContent);
    rootLayout->setContentsMargins(0, 0, 8, 0);
    rootLayout->setSpacing(12);
    rootLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    m_pAlgorithmTabWidget = new QTabWidget(settingsContent);
    QWidget* pointCloudTab = new QWidget(m_pAlgorithmTabWidget);
    QVBoxLayout* pointCloudLayout = new QVBoxLayout(pointCloudTab);
    pointCloudLayout->setContentsMargins(10, 10, 10, 10);
    pointCloudLayout->setSpacing(12);
    QWidget* featurePointTab = new QWidget(m_pAlgorithmTabWidget);
    QVBoxLayout* featurePointLayout = new QVBoxLayout(featurePointTab);
    featurePointLayout->setContentsMargins(10, 10, 10, 10);
    featurePointLayout->setSpacing(12);
    QWidget* validationTab = new QWidget(m_pAlgorithmTabWidget);
    QVBoxLayout* validationLayout = new QVBoxLayout(validationTab);
    validationLayout->setContentsMargins(10, 10, 10, 10);
    validationLayout->setSpacing(12);

    m_pSdkParamGroup = new QGroupBox("SDK 点云参数");
    QGridLayout* processingLayout = new QGridLayout(m_pSdkParamGroup);
    m_pExternalLibraryDirEdit = new QLineEdit();
    m_pExternalConfigPathEdit = new QLineEdit();
    PrepareWidePathEdit(m_pExternalLibraryDirEdit);
    PrepareWidePathEdit(m_pExternalConfigPathEdit);
    m_pExternalZTruncationSpin = new QDoubleSpinBox();
    m_pExternalZTruncationSpin->setRange(-9999.0, 9999.0);
    m_pExternalZTruncationSpin->setDecimals(3);
    m_pExternalZTruncationSpin->setSingleStep(0.5);
    m_pExternalResampleStepSpin = new QDoubleSpinBox();
    m_pExternalResampleStepSpin->setRange(0.1, 100.0);
    m_pExternalResampleStepSpin->setDecimals(3);
    m_pExternalResampleStepSpin->setSingleStep(0.5);
    m_pSdkWeldedStartCheck = new QCheckBox("使用已焊起点截断焊道");
    m_pSdkWeldedStartCheck->setToolTip("开启后，SDK 检测到工件上已焊段时返回新焊接的起点，\n焊道从该点截断只焊剩余部分；未检测到已焊段时不截断。\n仅 SDK 两种方法生效。");
    QPushButton* browseLibraryButton = new QPushButton("选择库目录");
    QPushButton* browseExternalConfigButton = new QPushButton("选择配置");
    processingLayout->setHorizontalSpacing(12);
    processingLayout->setVerticalSpacing(10);
    processingLayout->addWidget(new QLabel("Z截断值"), 0, 0);
    processingLayout->addWidget(CreateUnitEditor(m_pExternalZTruncationSpin, "mm"), 0, 1);
    processingLayout->addWidget(new QLabel("输出步长"), 0, 2);
    processingLayout->addWidget(CreateUnitEditor(m_pExternalResampleStepSpin, "mm"), 0, 3);
    processingLayout->addWidget(m_pSdkWeldedStartCheck, 1, 0, 1, 4);
    processingLayout->addWidget(new QLabel("库目录"), 2, 0);
    processingLayout->addWidget(m_pExternalLibraryDirEdit, 2, 1, 1, 3);
    processingLayout->addWidget(browseLibraryButton, 2, 4);
    processingLayout->addWidget(new QLabel("配置文件"), 3, 0);
    processingLayout->addWidget(m_pExternalConfigPathEdit, 3, 1, 1, 3);
    processingLayout->addWidget(browseExternalConfigButton, 3, 4);
    processingLayout->setColumnStretch(1, 1);
    processingLayout->setColumnStretch(3, 1);
    processingLayout->setColumnMinimumWidth(1, 320);
    processingLayout->setColumnMinimumWidth(3, 320);
    pointCloudLayout->addWidget(m_pSdkParamGroup);

    m_pSdkInnerGroup = new QGroupBox("SDK 算法内部参数");
    QGroupBox* cloudInnerGroup = m_pSdkInnerGroup;
    QGridLayout* cloudInnerLayout = new QGridLayout(cloudInnerGroup);
    cloudInnerLayout->setHorizontalSpacing(12);
    cloudInnerLayout->setVerticalSpacing(10);
    m_pCloudUprightCheck = new QCheckBox("正座机械臂采集");
    m_pCloudPlateThicknessSpin = new QDoubleSpinBox();
    m_pCloudPlateThicknessSpin->setRange(0.0, 9999.0);
    m_pCloudPlateThicknessSpin->setDecimals(3);
    m_pCloudRemoveFloorZSpin = new QDoubleSpinBox();
    m_pCloudRemoveFloorZSpin->setRange(-9999.0, 9999.0);
    m_pCloudRemoveFloorZSpin->setDecimals(3);
    m_pCloudThreadNumberSpin = new QSpinBox();
    m_pCloudThreadNumberSpin->setRange(1, 128);
    m_pCloudMoveDistanceSpin = new QDoubleSpinBox();
    m_pCloudMoveDistanceSpin->setRange(0.0, 9999.0);
    m_pCloudMoveDistanceSpin->setDecimals(3);
    m_pCloudDebugLogCheck = new QCheckBox("输出中间文件");
    m_pCloudLogPathEdit = new QLineEdit();
    PrepareWidePathEdit(m_pCloudLogPathEdit);
    m_pCloudPlaneThresholdSpin = new QDoubleSpinBox();
    m_pCloudPlaneThresholdSpin->setRange(0.0, 9999.0);
    m_pCloudPlaneThresholdSpin->setDecimals(3);
    m_pCloudMergeLinesAngleSpin = new QDoubleSpinBox();
    m_pCloudMergeLinesAngleSpin->setRange(0.0, 180.0);
    m_pCloudMergeLinesAngleSpin->setDecimals(3);
    m_pCloudMergeLinesDistanceSpin = new QDoubleSpinBox();
    m_pCloudMergeLinesDistanceSpin->setRange(0.0, 9999.0);
    m_pCloudMergeLinesDistanceSpin->setDecimals(3);
    m_pCloudClusterToleranceSpin = new QDoubleSpinBox();
    m_pCloudClusterToleranceSpin->setRange(0.0, 9999.0);
    m_pCloudClusterToleranceSpin->setDecimals(3);
    m_pCloudClusterCheck = new QCheckBox("启用聚类");
    m_pCloudRemoveNoiseCheck = new QCheckBox("库内去噪");
    m_pCloudRemoveNoiseCheck->setToolTip("SDK 内置去噪（is_remove_noise）：提取轨迹前先对点云做一遍去噪。20260617 版新增，默认开启。");
    m_pCloudSampleCheck = new QCheckBox("库内降采样找平面");
    m_pCloudSampleCheck->setToolTip("SDK 内置降采样（is_sample）：找平面前先对完整点云做体素降采样。\n"
        "波纹板\"先测后焊\"约 244 万点时，DLL 用 pcl_kdtree 找平面是主要耗时(120~200s)与崩溃点(Z截断退化)；\n"
        "开启后大幅降低 kdtree 压力——提速并显著减少崩溃。20260624 版新增，默认关闭。\n"
        "注意：只作用于\"找平面\"阶段，不改变最终焊道点的输出分辨率；现场启用后请确认焊缝精度无回退。\n"
        "仅 SDK 两种点云算法模式生效。");
    m_pCloudSampleSizeSpin = new QDoubleSpinBox();
    m_pCloudSampleSizeSpin->setRange(0.1, 100.0);
    m_pCloudSampleSizeSpin->setDecimals(2);
    m_pCloudSampleSizeSpin->setSingleStep(0.5);
    m_pCloudSampleSizeSpin->setToolTip("降采样体素大小（sample_size）：越大点越稀、找平面越快越粗。厂商默认 5。仅\"库内降采样\"开启时生效。");
    m_pCloudAboveZSpin = new QDoubleSpinBox();
    m_pCloudAboveZSpin->setRange(-9999.0, 9999.0);
    m_pCloudAboveZSpin->setDecimals(3);
    m_pCloudAboveZSpin->setSingleStep(0.1);
    m_pCloudAboveZSpin->setToolTip("抬高值（above_z）：SDK 20260624 版新增的平面抬升量，厂商默认 0.5。");
    m_pCloudDiscreteValueSpin = new QSpinBox();
    m_pCloudDiscreteValueSpin->setRange(0, 9999);
    m_pCloudDilateValueSpin = new QSpinBox();
    m_pCloudDilateValueSpin->setRange(0, 9999);
    m_pCloudErodeValueSpin = new QSpinBox();
    m_pCloudErodeValueSpin->setRange(0, 9999);
    m_pCloudLinesDisThresholdSpin = new QDoubleSpinBox();
    m_pCloudLinesDisThresholdSpin->setRange(0.0, 99999.0);
    m_pCloudLinesDisThresholdSpin->setDecimals(3);
    m_pCloudLineLengthSpin = new QDoubleSpinBox();
    m_pCloudLineLengthSpin->setRange(0.0, 9999.0);
    m_pCloudLineLengthSpin->setDecimals(3);

    cloudInnerLayout->addWidget(m_pCloudUprightCheck, 0, 0, 1, 2);
    cloudInnerLayout->addWidget(new QLabel("半板厚"), 0, 2);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudPlateThicknessSpin, "mm"), 0, 3);
    cloudInnerLayout->addWidget(new QLabel("去底板高度"), 1, 0);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudRemoveFloorZSpin, "mm"), 1, 1);
    cloudInnerLayout->addWidget(new QLabel("线程数"), 1, 2);
    cloudInnerLayout->addWidget(m_pCloudThreadNumberSpin, 1, 3);
    cloudInnerLayout->addWidget(new QLabel("首尾移动距离"), 2, 0);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudMoveDistanceSpin, "mm"), 2, 1);
    cloudInnerLayout->addWidget(m_pCloudDebugLogCheck, 2, 2, 1, 2);
    cloudInnerLayout->addWidget(new QLabel("日志目录"), 3, 0);
    cloudInnerLayout->addWidget(m_pCloudLogPathEdit, 3, 1, 1, 3);
    cloudInnerLayout->addWidget(new QLabel("平面内点距离"), 4, 0);
    cloudInnerLayout->addWidget(m_pCloudPlaneThresholdSpin, 4, 1);
    cloudInnerLayout->addWidget(new QLabel("直线合并角度"), 4, 2);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudMergeLinesAngleSpin, "deg"), 4, 3);
    cloudInnerLayout->addWidget(new QLabel("直线合并距离"), 5, 0);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudMergeLinesDistanceSpin, "mm"), 5, 1);
    cloudInnerLayout->addWidget(new QLabel("聚类阈值"), 5, 2);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudClusterToleranceSpin, "mm"), 5, 3);
    cloudInnerLayout->addWidget(m_pCloudClusterCheck, 6, 0, 1, 2);
    cloudInnerLayout->addWidget(new QLabel("缩放比例"), 6, 2);
    cloudInnerLayout->addWidget(m_pCloudDiscreteValueSpin, 6, 3);
    cloudInnerLayout->addWidget(new QLabel("膨胀核"), 7, 0);
    cloudInnerLayout->addWidget(m_pCloudDilateValueSpin, 7, 1);
    cloudInnerLayout->addWidget(new QLabel("腐蚀核"), 7, 2);
    cloudInnerLayout->addWidget(m_pCloudErodeValueSpin, 7, 3);
    cloudInnerLayout->addWidget(new QLabel("直线断头距离"), 8, 0);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudLinesDisThresholdSpin, "mm"), 8, 1);
    cloudInnerLayout->addWidget(new QLabel("直线最小长度"), 8, 2);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudLineLengthSpin, "mm"), 8, 3);
    cloudInnerLayout->addWidget(m_pCloudRemoveNoiseCheck, 9, 0, 1, 2);
    cloudInnerLayout->addWidget(m_pCloudSampleCheck, 9, 2, 1, 2);
    cloudInnerLayout->addWidget(new QLabel("降采样大小"), 10, 0);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudSampleSizeSpin, "mm"), 10, 1);
    cloudInnerLayout->addWidget(new QLabel("平面抬高值"), 10, 2);
    cloudInnerLayout->addWidget(CreateUnitEditor(m_pCloudAboveZSpin, "mm"), 10, 3);
    cloudInnerLayout->setColumnStretch(1, 1);
    cloudInnerLayout->setColumnStretch(3, 1);
    cloudInnerLayout->setColumnMinimumWidth(1, 300);
    cloudInnerLayout->setColumnMinimumWidth(3, 300);
    pointCloudLayout->addWidget(cloudInnerGroup);

    // 点云投影提取参数：方法③（点云算法+拟合）的"完整点云→下层焊道轨迹"提取阶段专用。
    // 数值 0（显示"自动"）表示按滤波拟合参数派生，与原内置行为一致。
    m_pProjectionGroup = new QGroupBox("点云投影提取参数（点云算法+拟合）");
    QGridLayout* projectionLayout = new QGridLayout(m_pProjectionGroup);
    projectionLayout->setHorizontalSpacing(12);
    projectionLayout->setVerticalSpacing(10);
    m_pProjStationWindowSpin = new QDoubleSpinBox();
    m_pProjStationWindowSpin->setRange(0.0, 9999.0);
    m_pProjStationWindowSpin->setDecimals(3);
    m_pProjStationWindowSpin->setSingleStep(0.5);
    m_pProjStationWindowSpin->setSpecialValueText("自动");
    m_pProjStationWindowSpin->setToolTip("沿焊缝方向取候选点的窗口半宽；自动=max(2.5, 输出步长×1.5)。");
    m_pProjTransverseWindowSpin = new QDoubleSpinBox();
    m_pProjTransverseWindowSpin->setRange(0.0, 9999.0);
    m_pProjTransverseWindowSpin->setDecimals(3);
    m_pProjTransverseWindowSpin->setSingleStep(1.0);
    m_pProjTransverseWindowSpin->setSpecialValueText("自动");
    m_pProjTransverseWindowSpin->setToolTip("垂直焊缝方向取候选点的窗口半宽；自动=max(10, 搜索窗口×1.5)。");
    m_pProjZBandBelowSpin = new QDoubleSpinBox();
    m_pProjZBandBelowSpin->setRange(0.0, 9999.0);
    m_pProjZBandBelowSpin->setDecimals(3);
    m_pProjZBandBelowSpin->setSingleStep(1.0);
    m_pProjZBandBelowSpin->setSpecialValueText("自动");
    m_pProjZBandBelowSpin->setToolTip("种子轨迹点下方纳入候选的高度范围；自动=max(14, Z连续阈值×4)。");
    m_pProjZBandAboveSpin = new QDoubleSpinBox();
    m_pProjZBandAboveSpin->setRange(0.0, 9999.0);
    m_pProjZBandAboveSpin->setDecimals(3);
    m_pProjZBandAboveSpin->setSingleStep(1.0);
    m_pProjZBandAboveSpin->setSpecialValueText("自动");
    m_pProjZBandAboveSpin->setToolTip("种子轨迹点上方纳入候选的高度范围；自动=max(12, Z突变阈值×2.5)。");
    m_pProjLayerLowSpin = new QDoubleSpinBox();
    m_pProjLayerLowSpin->setRange(0.0, 100.0);
    m_pProjLayerLowSpin->setDecimals(1);
    m_pProjLayerLowSpin->setSingleStep(5.0);
    m_pProjLayerLowSpin->setSuffix("%");
    m_pProjLayerLowSpin->setToolTip("候选点按 Z 排序后取作底板层的分位下界（默认 35%）。");
    m_pProjLayerHighSpin = new QDoubleSpinBox();
    m_pProjLayerHighSpin->setRange(0.0, 100.0);
    m_pProjLayerHighSpin->setDecimals(1);
    m_pProjLayerHighSpin->setSingleStep(5.0);
    m_pProjLayerHighSpin->setSuffix("%");
    m_pProjLayerHighSpin->setToolTip("候选点按 Z 排序后取作底板层的分位上界（默认 50%）。");
    m_pProjMaxCandidateSpin = new QSpinBox();
    m_pProjMaxCandidateSpin->setRange(1, 100000);
    m_pProjMaxCandidateSpin->setToolTip("每个种子点最多保留的底板候选点数（默认 160）。");
    m_pProjSmoothRadiusSpin = new QSpinBox();
    m_pProjSmoothRadiusSpin->setRange(0, 999);
    m_pProjSmoothRadiusSpin->setSpecialValueText("自动");
    m_pProjSmoothRadiusSpin->setToolTip("投影轮廓 Z 的中值平滑半径；自动=滤波平滑半径夹到 1~4。");
    projectionLayout->addWidget(new QLabel("站位窗口"), 0, 0);
    projectionLayout->addWidget(CreateUnitEditor(m_pProjStationWindowSpin, "mm"), 0, 1);
    projectionLayout->addWidget(new QLabel("横向窗口"), 0, 2);
    projectionLayout->addWidget(CreateUnitEditor(m_pProjTransverseWindowSpin, "mm"), 0, 3);
    projectionLayout->addWidget(new QLabel("种子下方Z带"), 1, 0);
    projectionLayout->addWidget(CreateUnitEditor(m_pProjZBandBelowSpin, "mm"), 1, 1);
    projectionLayout->addWidget(new QLabel("种子上方Z带"), 1, 2);
    projectionLayout->addWidget(CreateUnitEditor(m_pProjZBandAboveSpin, "mm"), 1, 3);
    projectionLayout->addWidget(new QLabel("底板层位下界"), 2, 0);
    projectionLayout->addWidget(m_pProjLayerLowSpin, 2, 1);
    projectionLayout->addWidget(new QLabel("底板层位上界"), 2, 2);
    projectionLayout->addWidget(m_pProjLayerHighSpin, 2, 3);
    projectionLayout->addWidget(new QLabel("每种子候选上限"), 3, 0);
    projectionLayout->addWidget(m_pProjMaxCandidateSpin, 3, 1);
    projectionLayout->addWidget(new QLabel("投影平滑半径"), 3, 2);
    projectionLayout->addWidget(m_pProjSmoothRadiusSpin, 3, 3);
    projectionLayout->setColumnStretch(1, 1);
    projectionLayout->setColumnStretch(3, 1);
    pointCloudLayout->addWidget(m_pProjectionGroup);
    pointCloudLayout->addStretch(1);

    QGroupBox* methodGroup = new QGroupBox("处理方法选择");
    QGridLayout* methodLayout = new QGridLayout(methodGroup);
    methodLayout->setHorizontalSpacing(12);
    methodLayout->setVerticalSpacing(8);
    m_pProcessingModeCombo = new QComboBox();
    const PointCloudProcessingConfig::Mode kMethodOrder[4] = {
        PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet,
        PointCloudProcessingConfig::Mode::SdkBaseWeldFit,
        PointCloudProcessingConfig::Mode::CloudFit,
        PointCloudProcessingConfig::Mode::LegacyLaserPath
    };
    for (PointCloudProcessingConfig::Mode method : kMethodOrder)
    {
        m_pProcessingModeCombo->addItem(
            PointCloudProcessingConfig::ModeDisplayName(method), static_cast<int>(method));
    }
    // 采样主轴对所有方法生效（①用于段类判定方向，②③④用于采样与段类），放方法组恒可用。
    m_pAxisCombo = new QComboBox();
    m_pAxisCombo->addItem("自动推断（按扫描方向）", static_cast<int>(PointCloudProcessingConfig::SampleAxisMode::Auto));
    m_pAxisCombo->addItem("按 X 方向采样/判段", static_cast<int>(PointCloudProcessingConfig::SampleAxisMode::AxisX));
    m_pAxisCombo->addItem("按 Y 方向采样/判段", static_cast<int>(PointCloudProcessingConfig::SampleAxisMode::AxisY));
    m_pAxisCombo->setToolTip("决定轨迹采样与平台/上坡/下坡段类判定的方向轴，对四种方法都生效；默认按扫描起止点方向自动推断。");
    QLabel* methodHintLabel = new QLabel(
        "SDK全处理：SDK拐点直接使用；SDK+拟合：SDK基础焊道再拟合；点云+拟合：完整点云投影提取下层轨迹再拟合；特征点+拟合：相机目标点轨迹拟合。处理失败直接报错，不回退其他方法。");
    methodHintLabel->setWordWrap(true);
    methodLayout->addWidget(new QLabel("当前方法"), 0, 0);
    methodLayout->addWidget(m_pProcessingModeCombo, 0, 1);
    methodLayout->addWidget(new QLabel("采样主轴"), 1, 0);
    methodLayout->addWidget(m_pAxisCombo, 1, 1);
    methodLayout->addWidget(methodHintLabel, 2, 0, 1, 3);
    methodLayout->setColumnStretch(2, 1);
    rootLayout->addWidget(methodGroup);

    QGroupBox* paramGroup = new QGroupBox("滤波拟合参数");
    QGridLayout* paramLayout = new QGridLayout(paramGroup);
    // 去噪/分段参数按实际方法动态启用：②跳过程序去噪并固定用方位角取角，③④使用程序几何链。
    // 这两个旧参数仍保留读写兼容，但当前几何拟合链没有消费，界面不再显示。
    m_pZThresholdSpin = new QDoubleSpinBox(paramGroup);
    m_pZThresholdSpin->setRange(-9999.0, 9999.0);
    m_pZThresholdSpin->setDecimals(3);
    m_pZThresholdSpin->setSingleStep(1.0);
    m_pZThresholdSpin->setValue(-230.0);
    m_pZThresholdSpin->hide();

    m_pZJumpThresholdSpin = new QDoubleSpinBox();
    m_pZJumpThresholdSpin->setRange(0.0, 9999.0);
    m_pZJumpThresholdSpin->setDecimals(3);
    m_pZJumpThresholdSpin->setSingleStep(0.5);
    m_pZJumpThresholdSpin->setValue(3.0);
    m_pZJumpThresholdSpin->setSpecialValueText("关闭");
    m_pZJumpThresholdSpin->setToolTip("仅方法③且“种子上方Z带”为自动时，用于推导上方候选范围；"
        "显式设置Z带后此项自动灰掉。");

    m_pZContinuityThresholdSpin = new QDoubleSpinBox();
    m_pZContinuityThresholdSpin->setRange(0.0, 9999.0);
    m_pZContinuityThresholdSpin->setDecimals(3);
    m_pZContinuityThresholdSpin->setSingleStep(0.5);
    m_pZContinuityThresholdSpin->setValue(2.0);
    m_pZContinuityThresholdSpin->setSpecialValueText("关闭");
    m_pZContinuityThresholdSpin->setToolTip("方法③④的局部离群点清理阈值；方法②使用SDK已去噪轨迹，跳过此项。");

    m_pSegmentBreakDistanceSpin = new QDoubleSpinBox();
    m_pSegmentBreakDistanceSpin->setRange(0.0, 9999.0);
    m_pSegmentBreakDistanceSpin->setDecimals(3);
    m_pSegmentBreakDistanceSpin->setSingleStep(1.0);
    m_pSegmentBreakDistanceSpin->setValue(6.0);
    m_pSegmentBreakDistanceSpin->setSpecialValueText("关闭");
    m_pSegmentBreakDistanceSpin->setToolTip("方法③④粗拟合阶段清理连续同类拐点游程时的距离基准，"
        "不是点云断段阈值；方法②固定走方位角分支，不使用。");

    m_pKeepLongestSegmentCheck = new QCheckBox("只保留最长连续段", paramGroup);
    m_pKeepLongestSegmentCheck->setChecked(true);
    m_pKeepLongestSegmentCheck->hide();

    m_pFeaturePointStrategyCombo = new QComboBox();
    m_pFeaturePointStrategyCombo->addItem(
        PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::LegacyGeometry),
        static_cast<int>(PointCloudProcessingConfig::FeaturePointStrategy::LegacyGeometry));
    m_pFeaturePointStrategyCombo->addItem(
        PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered),
        static_cast<int>(PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered));
    m_pFeaturePointStrategyCombo->addItem(
        PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys),
        static_cast<int>(PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys));
    // 方案三（立板投影到底板）已并入方法③"点云算法+拟合"做前置提取，不再单独作为拟合方案。

    m_pStepSpin = new QDoubleSpinBox();
    m_pStepSpin->setRange(0.1, 9999.0);
    m_pStepSpin->setDecimals(3);
    m_pStepSpin->setSingleStep(0.5);
    m_pStepSpin->setValue(2.0);

    m_pWindowSpin = new QDoubleSpinBox();
    m_pWindowSpin->setRange(0.0, 9999.0);
    m_pWindowSpin->setDecimals(3);
    m_pWindowSpin->setSingleStep(0.5);
    m_pWindowSpin->setValue(8.0);
    m_pWindowSpin->setToolTip("仅方法③且“投影横向窗口”为自动时，用于推导横向候选窗口；"
        "显式设置投影横向窗口后此项自动灰掉。");

    m_pPiecewiseToleranceSpin = new QDoubleSpinBox();
    m_pPiecewiseToleranceSpin->setRange(0.1, 9999.0);
    m_pPiecewiseToleranceSpin->setDecimals(3);
    m_pPiecewiseToleranceSpin->setSingleStep(0.5);
    m_pPiecewiseToleranceSpin->setValue(4.0);

    m_pPiecewiseMinSegmentSpin = new QSpinBox();
    m_pPiecewiseMinSegmentSpin->setRange(2, 9999);
    m_pPiecewiseMinSegmentSpin->setValue(10);

    m_pMinPointSpin = new QSpinBox();
    m_pMinPointSpin->setRange(2, 9999);
    m_pMinPointSpin->setValue(4);

    m_pSmoothRadiusSpin = new QSpinBox();
    m_pSmoothRadiusSpin->setRange(0, 999);
    m_pSmoothRadiusSpin->setValue(3);

    // 方位角拐点检测参数（SDK基础点云+滤波拟合流程固定使用：判方向转折角度定拐点，替代 DP）。
    m_pAzimuthTurnThresholdSpin = new QDoubleSpinBox();
    m_pAzimuthTurnThresholdSpin->setRange(1.0, 90.0);
    m_pAzimuthTurnThresholdSpin->setDecimals(1);
    m_pAzimuthTurnThresholdSpin->setSingleStep(1.0);
    m_pAzimuthTurnThresholdSpin->setValue(22.0);
    m_pAzimuthTurnThresholdSpin->setToolTip("拐点转角阈值：前后两段焊缝走向夹角超过此角度才算拐点。"
        "调大→滤掉波纹表面起伏(但缓拐角可能漏)；调小→更灵敏(但起伏可能多检)。仅 SDK基础点云+拟合流程用。");
    m_pAzimuthHeadingWindowSpin = new QSpinBox();
    m_pAzimuthHeadingWindowSpin->setRange(2, 100);
    m_pAzimuthHeadingWindowSpin->setValue(10);
    m_pAzimuthHeadingWindowSpin->setToolTip("拐点拟合窗口：用每点前后各 N 点局部拟合方向求转角。越大越平滑(抗起伏)，越小越灵敏。");
    m_pAzimuthNmsSpanSpin = new QDoubleSpinBox();
    m_pAzimuthNmsSpanSpin->setRange(1.0, 200.0);
    m_pAzimuthNmsSpanSpin->setDecimals(1);
    m_pAzimuthNmsSpanSpin->setValue(12.0);
    m_pAzimuthNmsSpanSpin->setToolTip("拐点非极大值抑制弧长：此距离内多个候选只保留转角最大的一个，避免一个折角被重复标。");
    m_pAzimuthStraightenResidualSpin = new QDoubleSpinBox();
    m_pAzimuthStraightenResidualSpin->setRange(0.5, 100.0);
    m_pAzimuthStraightenResidualSpin->setDecimals(1);
    m_pAzimuthStraightenResidualSpin->setValue(6.0);
    m_pAzimuthStraightenResidualSpin->setToolTip("直线化兜底残差：相邻拐点间实际焊道偏离直线超此值则补拐点(防漏检小台阶抄近路)。须大于波纹起伏幅度。");
    m_pAzimuthRefineFloorSpin = new QDoubleSpinBox();
    m_pAzimuthRefineFloorSpin->setRange(0.0, 50.0);
    m_pAzimuthRefineFloorSpin->setDecimals(1);
    m_pAzimuthRefineFloorSpin->setSingleStep(0.1);
    m_pAzimuthRefineFloorSpin->setValue(0.5);
    m_pAzimuthRefineFloorSpin->setToolTip("端区细化地板：粗拟合后再对\"一致弓向一侧\"的段补回漏检拐点(单侧弓出占比高才补，挡随机噪声)；"
        "首尾端区用此地板、中段=此×中段倍数。专治起终点附近缓弓拐点被抄近路。两条几何路径(SDK拟合/点云拟合)通用。");
    m_pCornerRefineEnableCheck = new QCheckBox("启用端区补拐点");
    m_pCornerRefineEnableCheck->setChecked(true);
    m_pCornerRefineEnableCheck->setToolTip("总开关：对粗拟合结果再做\"端区相干弓\"补拐点细化。关闭则完全保持原拐点。"
        "对②SDK点云算法+拟合、③点云算法+拟合、④特征点+拟合都生效。");
    m_pCornerRefineOneSidedSpin = new QDoubleSpinBox();
    m_pCornerRefineOneSidedSpin->setRange(50.0, 100.0);
    m_pCornerRefineOneSidedSpin->setDecimals(0);
    m_pCornerRefineOneSidedSpin->setSingleStep(5.0);
    m_pCornerRefineOneSidedSpin->setValue(80.0);
    m_pCornerRefineOneSidedSpin->setToolTip("单侧弓出门(%)：段内点偏向弦同一侧的占比≥此值才认定为真弓弯并补拐点。"
        "越高越严(只补明显一致的弓、更挡随机噪声)；随机噪声段约 50%。");
    m_pCornerRefineMidMultipleSpin = new QDoubleSpinBox();
    m_pCornerRefineMidMultipleSpin->setRange(1.0, 20.0);
    m_pCornerRefineMidMultipleSpin->setDecimals(1);
    m_pCornerRefineMidMultipleSpin->setSingleStep(0.5);
    m_pCornerRefineMidMultipleSpin->setValue(3.0);
    m_pCornerRefineMidMultipleSpin->setToolTip("中段地板倍数：中段补拐点的地板=端区细化地板×此倍数。"
        "体现\"漏检拐点多在引入/收尾段\"的先验——倍数越大中段越不易补(只补端区)，=1 则中段与端区同等灵敏。");
    m_pCornerRefineEndFracSpin = new QDoubleSpinBox();
    m_pCornerRefineEndFracSpin->setRange(0.0, 50.0);
    m_pCornerRefineEndFracSpin->setDecimals(0);
    m_pCornerRefineEndFracSpin->setSingleStep(5.0);
    m_pCornerRefineEndFracSpin->setValue(20.0);
    m_pCornerRefineEndFracSpin->setToolTip("端区占比(%)：焊缝首尾各此比例的弧长视为\"端区\"(用低地板、更易补拐点)，中间为中段(用高地板)。");
    m_pCornerPatternRefitCheck = new QCheckBox("启用平台重算(拐点交错约束)");
    m_pCornerPatternRefitCheck->setChecked(true);
    m_pCornerPatternRefitCheck->setToolTip("按波纹角成对规律(谷=两内角/峰=两外角)把拐点归成平台，每个平台从稠密点云三段拟合"
        "重算恰好 2 个边界角——自动修正源头\"多检\"(一个平台冒出多个同类角)与\"漏检\"(只检到一个)。搭接台阶点豁免。"
        "默认开，取代上面的\"端区补拐点\"(后者按几何加点、端区易过补)。对②③④三种拟合模式都生效。");
    m_pCornerPlatformMinSegSpin = new QSpinBox();
    m_pCornerPlatformMinSegSpin->setRange(3, 50);
    m_pCornerPlatformMinSegSpin->setValue(8);
    m_pCornerPlatformMinSegSpin->setToolTip("平台重算三段拟合每段最少点数：越大越稳健(抗噪)，但很短的平台可能拟合不出而回退保留原角。一般 6~12。");

    // 板材搭接 X 错位台阶检测(默认关，仅几何拟合流程)：双侧平台最小二乘判据。
    m_pLapSplitCheck = new QCheckBox("启用搭接错位检测(两侧分拟合·保台阶)");
    m_pLapSplitCheck->setToolTip("检测焊缝中心线上两块板材拼接处的侧向(X)错位台阶 → 错位两侧点云分别拟合、错位处保留 X 台阶不做过渡。判据：两侧近零斜率平台 + 中间窄跨突跳。仅几何拟合流程，默认关。");
    m_pLapStepHeightSpin = new QDoubleSpinBox();
    m_pLapStepHeightSpin->setRange(0.3, 50.0); m_pLapStepHeightSpin->setDecimals(2); m_pLapStepHeightSpin->setValue(1.0);
    m_pLapStepHeightSpin->setToolTip("台阶高门：候选中心处两侧平台拟合线的高度差大于此值才算错位。调大只认明显错位，调小更灵敏(抗噪余量缩小)。");
    m_pLapStepStationSpin = new QDoubleSpinBox();
    m_pLapStepStationSpin->setRange(2.0, 50.0); m_pLapStepStationSpin->setDecimals(1); m_pLapStepStationSpin->setValue(10.0);
    m_pLapStepStationSpin->setToolTip("单侧拟合窗口长度(主轴mm)：候选两侧各取此长度做平台直线拟合，越长斜率/残差估计越稳。");
    m_pLapStepFlatnessSpin = new QDoubleSpinBox();
    m_pLapStepFlatnessSpin->setRange(0.02, 10.0); m_pLapStepFlatnessSpin->setDecimals(2); m_pLapStepFlatnessSpin->setValue(0.12);
    m_pLapStepFlatnessSpin->setToolTip("平台残差上限：两侧拟合残差rms超此值=波纹/噪声/斜坡，不当平台(防误检)。");
    m_pLapStepSlopeSpin = new QDoubleSpinBox();
    m_pLapStepSlopeSpin->setRange(0.02, 2.0); m_pLapStepSlopeSpin->setDecimals(2); m_pLapStepSlopeSpin->setValue(0.10);
    m_pLapStepSlopeSpin->setToolTip("平台斜率上限(侧向mm/主轴mm)：两侧拟合斜率超此值=拐角斜边而非平台，排除拐角误判。拐角斜率约0.35，平台约0.05。");

    // SDK 基础焊道拟合前预平滑(结构自适应双边去锯齿)：仅 SDK点云算法+拟合(SdkBaseWeldFit)模式生效。
    m_pSdkBasePresmoothCheck = new QCheckBox("SdkBase 拟合前预平滑(去锯齿)");
    m_pSdkBasePresmoothCheck->setToolTip("仅「SDK点云算法+拟合」模式：拟合第一步前，对 SDK 重建的基础焊道做\"结构自适应 1D 双边\"去锯齿。\n"
        "按弧长参数化，值域核以\"邻点到局部切线的垂直偏移\"加权——直线段内的小锯齿被磨平，而搭接 X 台阶和真实拐角\n"
        "(垂直偏移大)处权重趋零、不跨过去平均，从机制上【不会把搭接段/拐角圆滑掉】。默认关闭，现场按需开启并标定。");
    m_pSdkBasePresmoothWindowSpin = new QDoubleSpinBox();
    m_pSdkBasePresmoothWindowSpin->setRange(0.5, 30.0); m_pSdkBasePresmoothWindowSpin->setDecimals(1); m_pSdkBasePresmoothWindowSpin->setSingleStep(0.5); m_pSdkBasePresmoothWindowSpin->setValue(3.0);
    m_pSdkBasePresmoothWindowSpin->setToolTip("预平滑空间窗 σs(沿弧长mm)：越大越平滑、去锯齿越狠，但须明显小于最短直线段长度，否则会跨过相邻拐角。一般 2~5mm。");
    m_pSdkBasePresmoothEdgeSpin = new QDoubleSpinBox();
    m_pSdkBasePresmoothEdgeSpin->setRange(0.05, 10.0); m_pSdkBasePresmoothEdgeSpin->setDecimals(2); m_pSdkBasePresmoothEdgeSpin->setSingleStep(0.1); m_pSdkBasePresmoothEdgeSpin->setValue(0.5);
    m_pSdkBasePresmoothEdgeSpin->setToolTip("预平滑保边阈值 σr(mm)：点到局部切线的垂直偏移大于约此值的跳变(搭接台阶/折角)会被保留不平滑。\n须【大于锯齿峰峰幅度、且明显小于搭接台阶高度】。例：锯齿≈0.2mm、台阶≈1.8mm 时取 0.3~0.5mm。调大→去锯齿更狠但可能开始啃台阶；调小→更保守但去锯齿弱。");

    // 基础焊道首尾段截断（②SDK+拟合 / ③点云+拟合 / ④特征点+拟合 三种拟合方案通用，非特化）。
    m_pEdgeTruncateCheck = new QCheckBox("基础焊道首尾截断");
    m_pEdgeTruncateCheck->setToolTip("对②SDK+拟合 / ③点云+拟合 / ④特征点+拟合 三种拟合方案都生效：拟合提取关键点【之前】，\n"
        "按点列【扫描顺序】(开头=首点侧 / 结尾=末点侧，与焊接方向无关)沿弧长截掉开头/结尾指定 mm 的点，剔除扫描进/出端坏点。\n"
        "与\"起点跳过/终点跳过\"(焊接段裁剪、晚于补偿)是不同层、互不影响。两端任一为 0 即该端不截；默认关。");
    m_pTruncateHeadSpin = new QDoubleSpinBox();
    m_pTruncateHeadSpin->setRange(0.0, 500.0); m_pTruncateHeadSpin->setDecimals(1); m_pTruncateHeadSpin->setSingleStep(1.0); m_pTruncateHeadSpin->setValue(0.0);
    m_pTruncateHeadSpin->setToolTip("开头截断弧长(mm)：从点列【首点侧】沿弧长截掉这么长。0=不截。");
    m_pTruncateTailSpin = new QDoubleSpinBox();
    m_pTruncateTailSpin->setRange(0.0, 500.0); m_pTruncateTailSpin->setDecimals(1); m_pTruncateTailSpin->setSingleStep(1.0); m_pTruncateTailSpin->setValue(0.0);
    m_pTruncateTailSpin->setToolTip("结尾截断弧长(mm)：从点列【末点侧】沿弧长截掉这么长。0=不截。");

    // 端区周期一致性补拐点（②③④拟合方案通用）：用波纹周期+典型拐角角度补回起/终点段漏掉的拐点。
    m_pEndPeriodRecoverCheck = new QCheckBox("端区周期补拐点");
    m_pEndPeriodRecoverCheck->setToolTip("对②③④三种拟合方案都生效，只负责补回起点/终点段漏检拐点：\n"
        "波纹板拐点近似【周期性】。先从中段可靠拐点算\"中位段长L=周期\"，再看端区(起点段/终点段)长度——\n"
        "若 ≥ 阈值×L 说明里面漏了拐点，按周期反推漏点位置，并在该处实测【弯折角】确认(直段≈0°不会误补)。\n"
        "长度定位 + 角度确认双判据。搭接台阶两拐点会先合成一段再算周期。与下面的同类短段合并独立，默认关。");
    m_pEndPeriodRatioSpin = new QDoubleSpinBox();
    m_pEndPeriodRatioSpin->setRange(1.05, 3.0); m_pEndPeriodRatioSpin->setDecimals(2); m_pEndPeriodRatioSpin->setSingleStep(0.05); m_pEndPeriodRatioSpin->setValue(1.2);
    m_pEndPeriodRatioSpin->setToolTip("判漏阈值：端段长 ÷ 周期L ≥ 此值才认为里面漏了拐点。1.3 表示端段比一个周期长 30% 以上就补。\n太小→收尾的部分周期段会被误判补点；太大→漏点补不回。一般 1.25~1.4。");
    m_pEndPeriodMinBendSpin = new QDoubleSpinBox();
    m_pEndPeriodMinBendSpin->setRange(1.0, 45.0); m_pEndPeriodMinBendSpin->setDecimals(1); m_pEndPeriodMinBendSpin->setSingleStep(1.0); m_pEndPeriodMinBendSpin->setValue(5.0);
    m_pEndPeriodMinBendSpin->setToolTip("补点最小弯折角(度)：预测窗口内候选点的弯折角 ≥ max(此值, 0.5×典型拐角角) 才补。\n防止在直段误补。波纹真拐角弯折常 7~13°、噪声基线 2~3°，取 5° 较稳。");
    m_pSameTypeShortMergeCheck = new QCheckBox("合并同类短段多余拐点");
    m_pSameTypeShortMergeCheck->setToolTip("对②③④三种拟合方案都生效，只负责删除多余拐点：\n"
        "排除首末与搭接段，把中间完整段按 IO/OI/II/OO 四类分别统计中位长度。相邻同类角之间若\n"
        "短于阈值、实际为坡且删点后恢复到同方向坡的典型长度，才合并；平的真实短平台保留。");
    m_pEndPeriodMergeSpin = new QDoubleSpinBox();
    m_pEndPeriodMergeSpin->setRange(0.05, 0.9); m_pEndPeriodMergeSpin->setDecimals(2); m_pEndPeriodMergeSpin->setSingleStep(0.05); m_pEndPeriodMergeSpin->setValue(0.4);
    m_pEndPeriodMergeSpin->setToolTip("短段阈值(候选段长/同类完整段中位)：越大越容易判为异常短段；越小越保守。"
        "还必须同时通过坡形和删后恢复长度验证，并非只看长度就删除。");
    m_pSameTypeMinReferenceSpin = new QSpinBox();
    m_pSameTypeMinReferenceSpin->setRange(2, 20);
    m_pSameTypeMinReferenceSpin->setValue(2);
    m_pSameTypeMinReferenceSpin->setToolTip("每一种同类平台和同方向坡至少需要多少个完整参考段才能自动合并。"
        "2 适合短工件；调大更保守。为避免单样本误删，最低为 2。");
    m_pSameTypeFlatSlopeSpin = new QDoubleSpinBox();
    m_pSameTypeFlatSlopeSpin->setRange(0.03, 0.30);
    m_pSameTypeFlatSlopeSpin->setDecimals(2);
    m_pSameTypeFlatSlopeSpin->setSingleStep(0.01);
    m_pSameTypeFlatSlopeSpin->setValue(0.15);
    m_pSameTypeFlatSlopeSpin->setToolTip("短段复核的坡/平台斜率分界：|侧向变化/主轴变化| 小于此值判平台，"
        "大于等于此值判坡。真实短平台不会因长度短被删除。");

    // 按平台边界重定拐点（②③④拟合方案通用）：波纹拐点全在"平台↔坡"交界。检测平的段(平台)，把拐点归位到平台两端。
    m_pPlatformSnapCheck = new QCheckBox("按平台边界重定拐点");
    m_pPlatformSnapCheck->setToolTip("对②③④三种拟合方案都生效，治【拐点被放到平台正中间→平台塌成长坡消失】、端区漏平台边界角：\n"
        "波纹板的拐点【全部位于\"平台↔坡\"交界】。本步沿焊道检测\"平的段(平台)\"，对每个平台保证两端各有一个边界拐点、\n"
        "删掉卡在平台内部(放错位)的拐点。对【已经正确】的平台幂等不动，只纠正错的。搭接台阶与起终点不参与。\n"
        "比\"端区周期补拐点\"更根本(直接按平台结构定角，统一治 缺/重/放错位 三种情况)。默认关。");
    m_pPlatformSnapFlatSlopeSpin = new QDoubleSpinBox();
    m_pPlatformSnapFlatSlopeSpin->setRange(0.03, 0.3); m_pPlatformSnapFlatSlopeSpin->setDecimals(2); m_pPlatformSnapFlatSlopeSpin->setSingleStep(0.01); m_pPlatformSnapFlatSlopeSpin->setValue(0.15);
    m_pPlatformSnapFlatSlopeSpin->setToolTip("平台判定斜率上限：点的侧向局部斜率 |Δ横向/Δ主轴| < 此值判为【平台(平)】，≥此值判为【坡】。\n波纹平台斜率≈0.01~0.05、上/下坡≈0.35~0.4，取 0.15 居中可靠区分。");
    m_pPlatformSnapMinFracSpin = new QDoubleSpinBox();
    m_pPlatformSnapMinFracSpin->setRange(0.1, 0.8); m_pPlatformSnapMinFracSpin->setDecimals(2); m_pPlatformSnapMinFracSpin->setSingleStep(0.05); m_pPlatformSnapMinFracSpin->setValue(0.25);
    m_pPlatformSnapMinFracSpin->setToolTip("平台最小长度(/周期)：平的段长 ≥ 此×周期L 才算真平台。更短的平的段视作噪声不参与。\n0.25 表示短于 25% 周期的平段忽略。太小→噪声误判平台；太大→真短平台被漏。");

    m_pSlopeConsistentCornerFitCheck = new QCheckBox("直线拟合排除圆弧段");
    m_pSlopeConsistentCornerFitCheck->setToolTip("启用后，平台线和坡度线只使用局部斜率一致的直线段拟合，再求交生成拐点。");

    m_pExportFitDebugCloudCheck = new QCheckBox("导出拟合调试点云(CloudCompare)");
    m_pExportFitDebugCloudCheck->setToolTip("启用后，每次拟合都把各段用到的点集、拟合直线和关键点导出到输出文件同目录的 FitDebug 子目录，可直接拖入 CloudCompare 核对每段拟合是否正确。");
    m_pExportFitDebugCloudCheck->setChecked(true);

    m_pExportWorkpieceFrameDebugCheck = new QCheckBox("导出完整点云逐帧文件(排查相机散点,大文件)");
    m_pExportWorkpieceFrameDebugCheck->setToolTip("仅排查相机点云散点时勾选：每次扫描把完整点云逐帧导出(每点带帧号/相机原始坐标/机器人位姿/时间戳)，文件可达数百MB、会明显拖慢流程。默认关闭，排查完请取消勾选。");
    m_pExportWorkpieceFrameDebugCheck->setChecked(false);

    // 滤波拟合方案置顶，方便切换；去噪/分段参数随后，再到拟合参数；采样主轴在方法组。
    paramLayout->addWidget(new QLabel("滤波拟合方案"), 0, 0);
    paramLayout->addWidget(m_pFeaturePointStrategyCombo, 0, 1, 1, 3);
    paramLayout->addWidget(new QLabel("Z突变阈值"), 1, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pZJumpThresholdSpin, "mm"), 1, 1);
    paramLayout->addWidget(new QLabel("Z连续阈值"), 1, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pZContinuityThresholdSpin, "mm"), 1, 3);
    paramLayout->addWidget(new QLabel("短同类段粗清理距离"), 2, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pSegmentBreakDistanceSpin, "mm"), 2, 1);
    paramLayout->addWidget(new QLabel("输出步长"), 2, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pStepSpin, "mm"), 2, 3);
    paramLayout->addWidget(new QLabel("搜索窗口"), 3, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pWindowSpin, "mm"), 3, 1);
    paramLayout->addWidget(new QLabel("分段拟合容差"), 3, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pPiecewiseToleranceSpin, "mm"), 3, 3);
    paramLayout->addWidget(new QLabel("每段最少点数"), 4, 0);
    paramLayout->addWidget(m_pPiecewiseMinSegmentSpin, 4, 1);
    paramLayout->addWidget(new QLabel("最小点数"), 4, 2);
    paramLayout->addWidget(m_pMinPointSpin, 4, 3);
    paramLayout->addWidget(new QLabel("平滑半径"), 5, 0);
    paramLayout->addWidget(m_pSmoothRadiusSpin, 5, 1);
    paramLayout->addWidget(m_pSlopeConsistentCornerFitCheck, 5, 2, 1, 2);
    paramLayout->addWidget(m_pExportFitDebugCloudCheck, 6, 0, 1, 4);
    paramLayout->addWidget(m_pExportWorkpieceFrameDebugCheck, 7, 0, 1, 4);
    paramLayout->addWidget(new QLabel("拐点转角阈值"), 8, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pAzimuthTurnThresholdSpin, "deg"), 8, 1);
    paramLayout->addWidget(new QLabel("拐点NMS弧长"), 8, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pAzimuthNmsSpanSpin, "mm"), 8, 3);
    paramLayout->addWidget(new QLabel("拐点拟合窗口"), 9, 0);
    paramLayout->addWidget(m_pAzimuthHeadingWindowSpin, 9, 1);
    paramLayout->addWidget(new QLabel("直线化残差"), 9, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pAzimuthStraightenResidualSpin, "mm"), 9, 3);
    paramLayout->addWidget(m_pCornerRefineEnableCheck, 10, 0, 1, 4);
    paramLayout->addWidget(new QLabel("端区细化地板"), 11, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pAzimuthRefineFloorSpin, "mm"), 11, 1);
    paramLayout->addWidget(new QLabel("单侧弓出门"), 11, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pCornerRefineOneSidedSpin, "%"), 11, 3);
    paramLayout->addWidget(new QLabel("中段地板倍数"), 12, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pCornerRefineMidMultipleSpin, "x"), 12, 1);
    paramLayout->addWidget(new QLabel("端区占比"), 12, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pCornerRefineEndFracSpin, "%"), 12, 3);
    paramLayout->addWidget(m_pCornerPatternRefitCheck, 13, 0, 1, 4);
    paramLayout->addWidget(new QLabel("平台最小段点数"), 14, 0);
    paramLayout->addWidget(m_pCornerPlatformMinSegSpin, 14, 1);
    paramLayout->addWidget(m_pLapSplitCheck, 15, 0, 1, 4);
    paramLayout->addWidget(new QLabel("错位台阶高门"), 16, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pLapStepHeightSpin, "mm"), 16, 1);
    paramLayout->addWidget(new QLabel("拟合窗口长度"), 16, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pLapStepStationSpin, "mm"), 16, 3);
    paramLayout->addWidget(new QLabel("平台残差上限"), 17, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pLapStepFlatnessSpin, "mm"), 17, 1);
    paramLayout->addWidget(new QLabel("平台斜率上限"), 17, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pLapStepSlopeSpin, "mm/mm"), 17, 3);
    paramLayout->addWidget(m_pSdkBasePresmoothCheck, 18, 0, 1, 4);
    paramLayout->addWidget(new QLabel("预平滑窗口"), 19, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pSdkBasePresmoothWindowSpin, "mm"), 19, 1);
    paramLayout->addWidget(new QLabel("预平滑保边阈值"), 19, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pSdkBasePresmoothEdgeSpin, "mm"), 19, 3);
    paramLayout->addWidget(m_pEdgeTruncateCheck, 20, 0, 1, 4);
    paramLayout->addWidget(new QLabel("开头截断"), 21, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pTruncateHeadSpin, "mm"), 21, 1);
    paramLayout->addWidget(new QLabel("结尾截断"), 21, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pTruncateTailSpin, "mm"), 21, 3);
    paramLayout->addWidget(m_pEndPeriodRecoverCheck, 22, 0, 1, 4);
    paramLayout->addWidget(new QLabel("判漏阈值(段长/周期)"), 23, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pEndPeriodRatioSpin, "x"), 23, 1);
    paramLayout->addWidget(new QLabel("补点最小弯折角"), 23, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pEndPeriodMinBendSpin, "deg"), 23, 3);
    paramLayout->addWidget(m_pSameTypeShortMergeCheck, 24, 0, 1, 4);
    paramLayout->addWidget(new QLabel("短段阈值(/同类中位)"), 25, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pEndPeriodMergeSpin, "x"), 25, 1);
    paramLayout->addWidget(new QLabel("每类最少参考段"), 25, 2);
    paramLayout->addWidget(m_pSameTypeMinReferenceSpin, 25, 3);
    paramLayout->addWidget(new QLabel("坡/平台斜率分界"), 26, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pSameTypeFlatSlopeSpin, ""), 26, 1);
    paramLayout->addWidget(m_pPlatformSnapCheck, 27, 0, 1, 4);
    paramLayout->addWidget(new QLabel("平台判定斜率上限"), 28, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pPlatformSnapFlatSlopeSpin, ""), 28, 1);
    paramLayout->addWidget(new QLabel("平台最小长度(/周期)"), 28, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pPlatformSnapMinFracSpin, "x"), 28, 3);
    paramLayout->setColumnStretch(1, 1);
    paramLayout->setColumnStretch(3, 1);
    featurePointLayout->addWidget(paramGroup);
    featurePointLayout->addStretch(1);

    QGroupBox* validationGroup = new QGroupBox("点云有效性检测");
    QGridLayout* validationLayoutGrid = new QGridLayout(validationGroup);
    validationLayoutGrid->setHorizontalSpacing(12);
    validationLayoutGrid->setVerticalSpacing(10);

    m_pValidationAuditOnlyCheck = new QCheckBox("审计模式（计算并记录全部指标，但不生成可执行质量证明）");
    m_pValidationAuditOnlyCheck->setToolTip(
        "仅用于历史数据标定。审计模式可以生成分析产物，但实际焊接、续焊和历史焊道执行会因没有 Enforce 质量证明而被阻止。");

    m_pValidationCoverageCheck = new QCheckBox("采集覆盖检测（强制）");
    m_pValidationMinFinitePointSpin = new QSpinBox();
    m_pValidationMinFinitePointSpin->setRange(0, 10000000);
    m_pValidationMinProjectedSpanSpin = new QDoubleSpinBox();
    m_pValidationMinProjectedSpanSpin->setRange(0.0, 999999.0);
    m_pValidationMinProjectedSpanSpin->setDecimals(3);
    m_pValidationMinProjectedSpanSpin->setSingleStep(10.0);

    m_pValidationContinuityCheck = new QCheckBox("连续性检测（强制）");
    m_pValidationMinStationCoverageSpin = new QDoubleSpinBox();
    m_pValidationMinStationCoverageSpin->setRange(0.0, 100.0);
    m_pValidationMinStationCoverageSpin->setDecimals(1);
    m_pValidationMinStationCoverageSpin->setSingleStep(5.0);
    m_pValidationMinStationCoverageSpin->setSuffix("%");
    m_pValidationMinLongestContinuousSpin = new QDoubleSpinBox();
    m_pValidationMinLongestContinuousSpin->setRange(0.0, 100.0);
    m_pValidationMinLongestContinuousSpin->setDecimals(1);
    m_pValidationMinLongestContinuousSpin->setSingleStep(5.0);
    m_pValidationMinLongestContinuousSpin->setSuffix("%");

    m_pValidationDenoiseRatioCheck = new QCheckBox("剔除比例检测（强制）");
    m_pValidationMaxRejectedRatioSpin = new QDoubleSpinBox();
    m_pValidationMaxRejectedRatioSpin->setRange(0.0, 100.0);
    m_pValidationMaxRejectedRatioSpin->setDecimals(1);
    m_pValidationMaxRejectedRatioSpin->setSingleStep(5.0);
    m_pValidationMaxRejectedRatioSpin->setSuffix("%");

    m_pValidationResidualCheck = new QCheckBox("拟合残差检测（强制）");
    m_pValidationMaxMedianResidualSpin = new QDoubleSpinBox();
    m_pValidationMaxMedianResidualSpin->setRange(0.0, 9999.0);
    m_pValidationMaxMedianResidualSpin->setDecimals(3);
    m_pValidationMaxP95ResidualSpin = new QDoubleSpinBox();
    m_pValidationMaxP95ResidualSpin->setRange(0.0, 9999.0);
    m_pValidationMaxP95ResidualSpin->setDecimals(3);
    m_pValidationResidualInlierThresholdSpin = new QDoubleSpinBox();
    m_pValidationResidualInlierThresholdSpin->setRange(0.0, 9999.0);
    m_pValidationResidualInlierThresholdSpin->setDecimals(3);
    m_pValidationMinResidualInlierRatioSpin = new QDoubleSpinBox();
    m_pValidationMinResidualInlierRatioSpin->setRange(0.0, 100.0);
    m_pValidationMinResidualInlierRatioSpin->setDecimals(1);
    m_pValidationMinResidualInlierRatioSpin->setSingleStep(5.0);
    m_pValidationMinResidualInlierRatioSpin->setSuffix("%");

    m_pValidationKeyPointCheck = new QCheckBox("起终点/拐点检测（强制）");
    m_pValidationMinKeyPointSpin = new QSpinBox();
    m_pValidationMinKeyPointSpin->setRange(0, 9999);
    m_pValidationMinCornerSpin = new QSpinBox();
    m_pValidationMinCornerSpin->setRange(0, 9999);
    m_pValidationMinSegmentLengthSpin = new QDoubleSpinBox();
    m_pValidationMinSegmentLengthSpin->setRange(0.0, 9999.0);
    m_pValidationMinSegmentLengthSpin->setDecimals(3);

    m_pValidationOutputCheck = new QCheckBox("输出结果检测（强制）");
    m_pValidationMinOutputPointSpin = new QSpinBox();
    m_pValidationMinOutputPointSpin->setRange(0, 10000000);
    m_pValidationMinOutputLengthRatioSpin = new QDoubleSpinBox();
    m_pValidationMinOutputLengthRatioSpin->setRange(0.0, 1000.0);
    m_pValidationMinOutputLengthRatioSpin->setDecimals(1);
    m_pValidationMinOutputLengthRatioSpin->setSingleStep(5.0);
    m_pValidationMinOutputLengthRatioSpin->setSuffix("%");

    validationLayoutGrid->addWidget(m_pValidationCoverageCheck, 0, 0, 1, 2);
    validationLayoutGrid->addWidget(new QLabel("最少有效点"), 0, 2);
    validationLayoutGrid->addWidget(m_pValidationMinFinitePointSpin, 0, 3);
    validationLayoutGrid->addWidget(new QLabel("最小主轴跨度"), 1, 0);
    validationLayoutGrid->addWidget(CreateUnitEditor(m_pValidationMinProjectedSpanSpin, "mm"), 1, 1);

    validationLayoutGrid->addWidget(m_pValidationContinuityCheck, 2, 0, 1, 2);
    validationLayoutGrid->addWidget(new QLabel("最小采样覆盖率"), 2, 2);
    validationLayoutGrid->addWidget(m_pValidationMinStationCoverageSpin, 2, 3);
    validationLayoutGrid->addWidget(new QLabel("最长连续段比例"), 3, 0);
    validationLayoutGrid->addWidget(m_pValidationMinLongestContinuousSpin, 3, 1);

    validationLayoutGrid->addWidget(m_pValidationDenoiseRatioCheck, 4, 0, 1, 2);
    validationLayoutGrid->addWidget(new QLabel("最大剔除比例"), 4, 2);
    validationLayoutGrid->addWidget(m_pValidationMaxRejectedRatioSpin, 4, 3);

    validationLayoutGrid->addWidget(m_pValidationResidualCheck, 5, 0, 1, 2);
    validationLayoutGrid->addWidget(new QLabel("中位残差上限"), 5, 2);
    validationLayoutGrid->addWidget(CreateUnitEditor(m_pValidationMaxMedianResidualSpin, "mm"), 5, 3);
    validationLayoutGrid->addWidget(new QLabel("P95残差上限"), 6, 0);
    validationLayoutGrid->addWidget(CreateUnitEditor(m_pValidationMaxP95ResidualSpin, "mm"), 6, 1);
    validationLayoutGrid->addWidget(new QLabel("内点残差阈值"), 6, 2);
    validationLayoutGrid->addWidget(CreateUnitEditor(m_pValidationResidualInlierThresholdSpin, "mm"), 6, 3);
    validationLayoutGrid->addWidget(new QLabel("最小内点比例"), 7, 0);
    validationLayoutGrid->addWidget(m_pValidationMinResidualInlierRatioSpin, 7, 1);

    validationLayoutGrid->addWidget(m_pValidationKeyPointCheck, 8, 0, 1, 2);
    validationLayoutGrid->addWidget(new QLabel("最少关键点"), 8, 2);
    validationLayoutGrid->addWidget(m_pValidationMinKeyPointSpin, 8, 3);
    validationLayoutGrid->addWidget(new QLabel("最少拐点"), 9, 0);
    validationLayoutGrid->addWidget(m_pValidationMinCornerSpin, 9, 1);
    validationLayoutGrid->addWidget(new QLabel("最短关键段"), 9, 2);
    validationLayoutGrid->addWidget(CreateUnitEditor(m_pValidationMinSegmentLengthSpin, "mm"), 9, 3);

    validationLayoutGrid->addWidget(m_pValidationOutputCheck, 10, 0, 1, 2);
    validationLayoutGrid->addWidget(new QLabel("最少输出点"), 10, 2);
    validationLayoutGrid->addWidget(m_pValidationMinOutputPointSpin, 10, 3);
    validationLayoutGrid->addWidget(new QLabel("输出长度/输入跨度"), 11, 0);
    validationLayoutGrid->addWidget(m_pValidationMinOutputLengthRatioSpin, 11, 1);
    const QList<QCheckBox*> mandatoryValidationChecks = {
        m_pValidationCoverageCheck,
        m_pValidationContinuityCheck,
        m_pValidationDenoiseRatioCheck,
        m_pValidationResidualCheck,
        m_pValidationKeyPointCheck,
        m_pValidationOutputCheck
    };
    for (QCheckBox* check : mandatoryValidationChecks)
    {
        check->setChecked(true);
        check->setEnabled(false);
        check->setToolTip("质量策略 v1 要求始终计算该项；如需标定阈值，请切换审计模式，而不是关闭检查。");
    }
    validationLayoutGrid->setColumnStretch(1, 1);
    validationLayoutGrid->setColumnStretch(3, 1);
    validationLayout->addWidget(m_pValidationAuditOnlyCheck);
    validationLayout->addWidget(validationGroup);
    validationLayout->addStretch(1);

    m_pAlgorithmTabWidget->addTab(pointCloudTab, "点云参数");
    m_pAlgorithmTabWidget->addTab(featurePointTab, "滤波拟合参数");
    m_pAlgorithmTabWidget->addTab(validationTab, "有效性检测");
    rootLayout->addWidget(m_pAlgorithmTabWidget);
    rootLayout->addStretch(1);

    settingsScrollArea->setWidget(settingsContent);
    outerLayout->addWidget(settingsScrollArea, 1);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch(1);
    QPushButton* saveSettingsButton = new QPushButton("保存设置");
    saveSettingsButton->setMinimumSize(150, 42);
    actionLayout->addWidget(saveSettingsButton);
    outerLayout->addLayout(actionLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1200);
    m_pLogText->setMinimumHeight(110);
    m_pLogText->setMaximumHeight(170);
    m_pLogText->setPlainText("日志：等待保存设置...");
    outerLayout->addWidget(m_pLogText);

    connect(browseLibraryButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::BrowseExternalLibraryDir);
    connect(browseExternalConfigButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::BrowseExternalConfigFile);
    connect(saveSettingsButton, &QPushButton::clicked, this, [this]()
        {
            QString error;
            if (!SaveSettings(&error))
            {
                const QString message = error.isEmpty() ? QStringLiteral("保存设置失败。") : QStringLiteral("保存设置失败：%1").arg(error);
                AppendLog(message);
                QMessageBox::warning(this, "精测点云处理", message);
                return;
            }
            // Enforce 会在数据库事务内应用不可放宽的安全边界；立即回读，让界面显示真正生效的值。
            LoadSettings();
            const QString message = "精测点云处理设置已保存，先测后焊流程将使用当前配置。";
            AppendLog(message);
            QMessageBox::information(this, "精测点云处理", message);
        });
    connect(m_pFeaturePointStrategyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
        {
            ApplyMethodEnableState();
            SaveSettings();
        });
    connect(m_pSlopeConsistentCornerFitCheck, &QCheckBox::toggled, this, [this](bool) { SaveSettings(); });
    connect(m_pExportFitDebugCloudCheck, &QCheckBox::toggled, this, [this](bool) { SaveSettings(); });
    connect(m_pExportWorkpieceFrameDebugCheck, &QCheckBox::toggled, this, [this](bool) { SaveSettings(); });
    connect(m_pExternalConfigPathEdit, &QLineEdit::editingFinished, this, &LaserWeldFilterDialog::LoadExternalAlgorithmConfig);
    connect(m_pProcessingModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
        {
            // 切换处理方法：按四方法启用矩阵联动禁用不相关参数，并保存。
            ApplyMethodEnableState();
            SaveSettings();
        });
    connect(m_pAxisCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { SaveSettings(); });
    const auto refreshParameterEnableState = [this](bool)
    {
        ApplyMethodEnableState();
    };
    for (QCheckBox* check : {
        m_pCornerRefineEnableCheck,
        m_pCornerPatternRefitCheck,
        m_pLapSplitCheck,
        m_pSdkBasePresmoothCheck,
        m_pEdgeTruncateCheck,
        m_pEndPeriodRecoverCheck,
        m_pSameTypeShortMergeCheck,
        m_pPlatformSnapCheck })
    {
        connect(check, &QCheckBox::toggled, this, refreshParameterEnableState);
    }
    connect(m_pProjTransverseWindowSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, [this](double) { ApplyMethodEnableState(); });
    connect(m_pProjZBandAboveSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, [this](double) { ApplyMethodEnableState(); });

    ApplyResponsivePageDefaults(this);
}

void LaserWeldFilterDialog::ApplyMethodEnableState()
{
    const PointCloudProcessingConfig::Mode mode = CurrentProcessingMode();
    const bool usesSdk = mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        || mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit;
    // 程序滤波拟合链（去噪/分段/拟合）：②③④使用；①直接用 SDK 拐点，失败即报错不回退。
    const bool usesFitChain = mode != PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet;
    // 点云投影提取（完整点云→下层轨迹）仅方法③使用。
    const bool usesProjection = mode == PointCloudProcessingConfig::Mode::CloudFit;
    const bool usesSdkBaseFit = mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit;
    const bool usesProgramDenoise = mode == PointCloudProcessingConfig::Mode::CloudFit
        || mode == PointCloudProcessingConfig::Mode::LegacyLaserPath;
    const PointCloudProcessingConfig::FeaturePointStrategy strategy = CurrentFeaturePointStrategy();
    const bool usesSlopeWaveFilter =
        strategy == PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered;
    const bool usesRobustSegmentedKeys =
        strategy == PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys;
    const auto setEnabled = [](QWidget* widget, bool enabled)
    {
        if (widget != nullptr)
        {
            widget->setEnabled(enabled);
        }
    };

    if (m_pSdkParamGroup != nullptr)
    {
        m_pSdkParamGroup->setEnabled(usesSdk);
    }
    if (m_pSdkInnerGroup != nullptr)
    {
        m_pSdkInnerGroup->setEnabled(usesSdk);
    }
    if (m_pProjectionGroup != nullptr)
    {
        m_pProjectionGroup->setEnabled(usesProjection);
    }

    // 只改变可编辑状态，不改值、不改勾选；SaveSettings 仍保存所有控件的原状态。
    setEnabled(m_pFeaturePointStrategyCombo, usesFitChain);
    setEnabled(m_pZJumpThresholdSpin,
        usesProjection && m_pProjZBandAboveSpin != nullptr && m_pProjZBandAboveSpin->value() <= 0.0);
    setEnabled(m_pZContinuityThresholdSpin, usesProgramDenoise);
    setEnabled(m_pSegmentBreakDistanceSpin, usesProgramDenoise);
    setEnabled(m_pStepSpin, usesFitChain);
    setEnabled(m_pWindowSpin,
        usesProjection && m_pProjTransverseWindowSpin != nullptr && m_pProjTransverseWindowSpin->value() <= 0.0);
    const bool usesPiecewiseControls = usesProgramDenoise || (usesSdkBaseFit && usesSlopeWaveFilter);
    setEnabled(m_pPiecewiseToleranceSpin, usesPiecewiseControls);
    setEnabled(m_pPiecewiseMinSegmentSpin,
        (usesProgramDenoise && (usesSlopeWaveFilter || usesRobustSegmentedKeys))
        || (usesSdkBaseFit && usesSlopeWaveFilter));
    setEnabled(m_pMinPointSpin, usesFitChain);
    setEnabled(m_pSmoothRadiusSpin, usesPiecewiseControls);

    for (QWidget* widget : {
        static_cast<QWidget*>(m_pAzimuthTurnThresholdSpin),
        static_cast<QWidget*>(m_pAzimuthHeadingWindowSpin),
        static_cast<QWidget*>(m_pAzimuthNmsSpanSpin),
        static_cast<QWidget*>(m_pAzimuthStraightenResidualSpin) })
    {
        setEnabled(widget, usesSdkBaseFit);
    }

    setEnabled(m_pCornerRefineEnableCheck, usesFitChain);
    const bool cornerRefineEnabled =
        usesFitChain && m_pCornerRefineEnableCheck != nullptr && m_pCornerRefineEnableCheck->isChecked();
    for (QWidget* widget : {
        static_cast<QWidget*>(m_pAzimuthRefineFloorSpin),
        static_cast<QWidget*>(m_pCornerRefineOneSidedSpin),
        static_cast<QWidget*>(m_pCornerRefineMidMultipleSpin),
        static_cast<QWidget*>(m_pCornerRefineEndFracSpin) })
    {
        setEnabled(widget, cornerRefineEnabled);
    }

    setEnabled(m_pCornerPatternRefitCheck, usesFitChain);
    setEnabled(m_pCornerPlatformMinSegSpin,
        usesFitChain && m_pCornerPatternRefitCheck != nullptr && m_pCornerPatternRefitCheck->isChecked());

    setEnabled(m_pLapSplitCheck, usesFitChain);
    const bool lapSplitEnabled = usesFitChain && m_pLapSplitCheck != nullptr && m_pLapSplitCheck->isChecked();
    for (QWidget* widget : {
        static_cast<QWidget*>(m_pLapStepHeightSpin),
        static_cast<QWidget*>(m_pLapStepStationSpin),
        static_cast<QWidget*>(m_pLapStepFlatnessSpin),
        static_cast<QWidget*>(m_pLapStepSlopeSpin) })
    {
        setEnabled(widget, lapSplitEnabled);
    }

    setEnabled(m_pSdkBasePresmoothCheck, usesSdkBaseFit);
    const bool sdkPresmoothEnabled =
        usesSdkBaseFit && m_pSdkBasePresmoothCheck != nullptr && m_pSdkBasePresmoothCheck->isChecked();
    setEnabled(m_pSdkBasePresmoothWindowSpin, sdkPresmoothEnabled);
    setEnabled(m_pSdkBasePresmoothEdgeSpin, sdkPresmoothEnabled);

    setEnabled(m_pEdgeTruncateCheck, usesFitChain);
    const bool edgeTruncateEnabled =
        usesFitChain && m_pEdgeTruncateCheck != nullptr && m_pEdgeTruncateCheck->isChecked();
    setEnabled(m_pTruncateHeadSpin, edgeTruncateEnabled);
    setEnabled(m_pTruncateTailSpin, edgeTruncateEnabled);

    setEnabled(m_pEndPeriodRecoverCheck, usesFitChain);
    const bool endRecoverEnabled =
        usesFitChain && m_pEndPeriodRecoverCheck != nullptr && m_pEndPeriodRecoverCheck->isChecked();
    setEnabled(m_pEndPeriodRatioSpin, endRecoverEnabled);
    setEnabled(m_pEndPeriodMinBendSpin, endRecoverEnabled);

    setEnabled(m_pSameTypeShortMergeCheck, usesFitChain);
    const bool sameTypeMergeEnabled =
        usesFitChain && m_pSameTypeShortMergeCheck != nullptr && m_pSameTypeShortMergeCheck->isChecked();
    setEnabled(m_pEndPeriodMergeSpin, sameTypeMergeEnabled);
    setEnabled(m_pSameTypeMinReferenceSpin, sameTypeMergeEnabled);
    setEnabled(m_pSameTypeFlatSlopeSpin, sameTypeMergeEnabled);

    setEnabled(m_pPlatformSnapCheck, usesFitChain);
    const bool platformSnapEnabled =
        usesFitChain && m_pPlatformSnapCheck != nullptr && m_pPlatformSnapCheck->isChecked();
    setEnabled(m_pPlatformSnapFlatSlopeSpin, platformSnapEnabled);
    setEnabled(m_pPlatformSnapMinFracSpin, platformSnapEnabled);
    setEnabled(m_pSlopeConsistentCornerFitCheck, usesFitChain);
    setEnabled(m_pExportFitDebugCloudCheck, usesFitChain);
    if (m_pAlgorithmTabWidget != nullptr)
    {
        // 点云参数页：SDK 组（①②）+ 投影提取组（③），按组禁用；④整页禁用。
        // 滤波拟合参数页仅作用于程序拟合链（②③④）；有效性门禁对四种方法都强制生效。
        m_pAlgorithmTabWidget->setTabEnabled(0, usesSdk || usesProjection);
        m_pAlgorithmTabWidget->setTabEnabled(1, usesFitChain);
        m_pAlgorithmTabWidget->setTabEnabled(2, true);
        if (!m_pAlgorithmTabWidget->isTabEnabled(m_pAlgorithmTabWidget->currentIndex()))
        {
            m_pAlgorithmTabWidget->setCurrentIndex(usesSdk ? 0 : 1);
        }
    }
}

void LaserWeldFilterDialog::ApplyStyle()
{
    setStyleSheet(QString(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QLabel { color: #B8C7CC; }"
        "QLabel#TitleLabel { font-size: 24px; font-weight: 700; color: #F4FAFA; }"
        "QLabel#HintLabel { color: #8FA7B0; }"
        "QLabel#UnitLabel { color: #8FA7B0; font-weight: 400; padding-left: 2px; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #9ED8DB; }"
        "QLineEdit, QDoubleSpinBox, QSpinBox { background: #0B1117; color: #F4FAFA; border: 1px solid #2E4656; border-radius: 10px; padding: 6px 10px; min-height: 34px; }"
        "QPushButton { background: #1F3542; color: #F4FAFA; border: 1px solid #3C6475; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2C5364; border-color: #63C7D1; }"
        "QPushButton:pressed { background: #16303A; }"
        "QTabWidget::pane { border: 1px solid #2E4656; border-radius: 10px; top: -1px; background: #101820; }"
        "QTabBar::tab { background: #0B1117; color: #B8C7CC; border: 1px solid #2E4656; padding: 8px 18px; min-width: 110px; }"
        "QTabBar::tab:selected { background: #214D5B; color: #FFFFFF; border-color: #65C7D0; }"
        "QTabBar::tab:hover { background: #18303B; color: #FFFFFF; }"
        "QPlainTextEdit { background: #0B1117; color: #BFE7EA; border: 1px solid #2E4656; border-radius: 10px; padding: 8px; }"
        // 禁用态统一压暗：自定义 QSS 指定固定色后 Qt 默认的禁用变灰会被覆盖，必须显式给 :disabled 规则。
        "QLineEdit:disabled, QDoubleSpinBox:disabled, QSpinBox:disabled { background: #0D141A; color: #55656D; border-color: #1E2F3A; }"
        "QComboBox:disabled { background: #0D141A; color: #55656D; border-color: #1E2F3A; }"
        "QComboBox::drop-down:disabled { background: #0D141A; border-left-color: #1E2F3A; }"
        "QPushButton:disabled { background: #15212B; color: #55656D; border-color: #1E2F3A; }"
        "QCheckBox:disabled { color: #55656D; }"
        "QLabel:disabled { color: #4A5860; }"
        "QLabel#UnitLabel:disabled { color: #3E4B53; }"
        "QGroupBox:disabled { border-color: #1E2F3A; }"
        "QGroupBox::title:disabled { color: #4A5860; }"
        "QTabBar::tab:disabled { background: #0A0F14; color: #44525A; }")
        + UnifiedComboBoxStyleSheet());
}

void LaserWeldFilterDialog::LoadSettings()
{
    const PointCloudProcessingConfig::Settings processingSettings = PointCloudProcessingConfig::Load();
    if (m_pProcessingModeCombo != nullptr)
    {
        const QSignalBlocker blocker(m_pProcessingModeCombo);
        const int processingModeIndex = m_pProcessingModeCombo->findData(static_cast<int>(processingSettings.mode));
        m_pProcessingModeCombo->setCurrentIndex(processingModeIndex >= 0 ? processingModeIndex : 0);
    }
    if (m_pFeaturePointStrategyCombo != nullptr)
    {
        const QSignalBlocker blocker(m_pFeaturePointStrategyCombo);
        const int strategyIndex = m_pFeaturePointStrategyCombo->findData(
            static_cast<int>(processingSettings.featurePointStrategy));
        m_pFeaturePointStrategyCombo->setCurrentIndex(strategyIndex >= 0 ? strategyIndex : 0);
    }
    SetPathEditText(m_pExternalLibraryDirEdit, QDir::toNativeSeparators(processingSettings.libraryDir));
    SetPathEditText(m_pExternalConfigPathEdit, QDir::toNativeSeparators(processingSettings.configPath));
    m_pExternalZTruncationSpin->setValue(processingSettings.zTruncationValue);
    m_pExternalResampleStepSpin->setValue(processingSettings.resampleStepMm);
    if (m_pSdkWeldedStartCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pSdkWeldedStartCheck);
        m_pSdkWeldedStartCheck->setChecked(processingSettings.sdkUseWeldedStartTruncation);
    }
    if (m_pSlopeConsistentCornerFitCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pSlopeConsistentCornerFitCheck);
        m_pSlopeConsistentCornerFitCheck->setChecked(processingSettings.slopeConsistentCornerFit);
    }
    if (m_pExportFitDebugCloudCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pExportFitDebugCloudCheck);
        m_pExportFitDebugCloudCheck->setChecked(processingSettings.exportFitDebugCloud);
    }
    if (m_pExportWorkpieceFrameDebugCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pExportWorkpieceFrameDebugCheck);
        m_pExportWorkpieceFrameDebugCheck->setChecked(processingSettings.exportWorkpieceFrameDebug);
    }
    m_pValidationAuditOnlyCheck->setChecked(
        processingSettings.validationPolicy == PointCloudProcessingConfig::ValidationPolicy::Audit);
    m_pValidationCoverageCheck->setChecked(true);
    m_pValidationMinFinitePointSpin->setValue(processingSettings.validationMinFinitePointCount);
    m_pValidationMinProjectedSpanSpin->setValue(processingSettings.validationMinProjectedSpanMm);
    m_pValidationContinuityCheck->setChecked(true);
    m_pValidationMinStationCoverageSpin->setValue(processingSettings.validationMinStationCoverageRatio * 100.0);
    m_pValidationMinLongestContinuousSpin->setValue(processingSettings.validationMinLongestContinuousRatio * 100.0);
    m_pValidationDenoiseRatioCheck->setChecked(true);
    m_pValidationMaxRejectedRatioSpin->setValue(processingSettings.validationMaxRejectedRatio * 100.0);
    m_pValidationResidualCheck->setChecked(true);
    m_pValidationMaxMedianResidualSpin->setValue(processingSettings.validationMaxMedianResidualMm);
    m_pValidationMaxP95ResidualSpin->setValue(processingSettings.validationMaxP95ResidualMm);
    m_pValidationResidualInlierThresholdSpin->setValue(processingSettings.validationResidualInlierThresholdMm);
    m_pValidationMinResidualInlierRatioSpin->setValue(processingSettings.validationMinResidualInlierRatio * 100.0);
    m_pValidationKeyPointCheck->setChecked(true);
    m_pValidationMinKeyPointSpin->setValue(processingSettings.validationMinKeyPointCount);
    m_pValidationMinCornerSpin->setValue(processingSettings.validationMinCornerCount);
    m_pValidationMinSegmentLengthSpin->setValue(processingSettings.validationMinSegmentLengthMm);
    m_pValidationOutputCheck->setChecked(true);
    m_pValidationMinOutputPointSpin->setValue(processingSettings.validationMinOutputPointCount);
    m_pValidationMinOutputLengthRatioSpin->setValue(processingSettings.validationMinOutputLengthRatio * 100.0);
    {
        const QSignalBlocker blocker(m_pAxisCombo);
        const int axisIndex = m_pAxisCombo->findData(static_cast<int>(processingSettings.sampleAxisMode));
        m_pAxisCombo->setCurrentIndex(axisIndex >= 0 ? axisIndex : 0);
    }
    m_pZThresholdSpin->setValue(processingSettings.cloudZThresholdMm);
    m_pZJumpThresholdSpin->setValue(processingSettings.cloudZJumpThresholdMm);
    m_pZContinuityThresholdSpin->setValue(processingSettings.cloudZContinuityThresholdMm);
    m_pSegmentBreakDistanceSpin->setValue(processingSettings.cloudSegmentBreakDistanceMm);
    m_pKeepLongestSegmentCheck->setChecked(processingSettings.cloudKeepLongestSegmentOnly);
    m_pStepSpin->setValue(processingSettings.fitSampleStepMm);
    m_pWindowSpin->setValue(processingSettings.fitSearchWindowMm);
    m_pPiecewiseToleranceSpin->setValue(processingSettings.fitPiecewiseToleranceMm);
    m_pPiecewiseMinSegmentSpin->setValue(processingSettings.fitPiecewiseMinSegmentPoints);
    m_pMinPointSpin->setValue(processingSettings.fitMinPointCount);
    m_pSmoothRadiusSpin->setValue(processingSettings.fitSmoothRadius);
    m_pAzimuthTurnThresholdSpin->setValue(processingSettings.fitAzimuthTurnThresholdDeg);
    m_pAzimuthHeadingWindowSpin->setValue(processingSettings.fitAzimuthHeadingWindow);
    m_pAzimuthNmsSpanSpin->setValue(processingSettings.fitAzimuthNmsSpanMm);
    m_pAzimuthStraightenResidualSpin->setValue(processingSettings.fitAzimuthStraightenResidualMm);
    m_pCornerRefineEnableCheck->setChecked(processingSettings.fitCornerRefineEnable);
    m_pAzimuthRefineFloorSpin->setValue(processingSettings.fitAzimuthRefineFloorMm);
    m_pCornerRefineOneSidedSpin->setValue(processingSettings.fitCornerRefineOneSidedPct);
    m_pCornerRefineMidMultipleSpin->setValue(processingSettings.fitCornerRefineMidMultiple);
    m_pCornerRefineEndFracSpin->setValue(processingSettings.fitCornerRefineEndFracPct);
    m_pCornerPatternRefitCheck->setChecked(processingSettings.fitCornerPatternRefitEnable);
    m_pCornerPlatformMinSegSpin->setValue(processingSettings.fitCornerPlatformMinSegPoints);
    m_pLapSplitCheck->setChecked(processingSettings.enableLapMisalignmentSplit);
    m_pLapStepHeightSpin->setValue(processingSettings.lapStepHeightThresholdMm);
    m_pLapStepStationSpin->setValue(processingSettings.lapStepStationWindowMm);
    m_pLapStepFlatnessSpin->setValue(processingSettings.lapStepSideFlatnessMm);
    m_pLapStepSlopeSpin->setValue(processingSettings.lapStepPlatformSlopeMax);
    m_pSdkBasePresmoothCheck->setChecked(processingSettings.sdkBasePresmoothEnable);
    m_pSdkBasePresmoothWindowSpin->setValue(processingSettings.sdkBasePresmoothWindowMm);
    m_pSdkBasePresmoothEdgeSpin->setValue(processingSettings.sdkBasePresmoothEdgeMm);
    m_pEdgeTruncateCheck->setChecked(processingSettings.fitEdgeTruncateEnable);
    m_pTruncateHeadSpin->setValue(processingSettings.fitTruncateHeadMm);
    m_pTruncateTailSpin->setValue(processingSettings.fitTruncateTailMm);
    m_pEndPeriodRecoverCheck->setChecked(processingSettings.fitEndPeriodRecoverEnable);
    m_pEndPeriodRatioSpin->setValue(processingSettings.fitEndPeriodRatioThreshold);
    m_pEndPeriodMinBendSpin->setValue(processingSettings.fitEndPeriodMinBendDeg);
    m_pSameTypeShortMergeCheck->setChecked(processingSettings.fitSameTypeShortMergeEnable);
    m_pEndPeriodMergeSpin->setValue(processingSettings.fitEndPeriodMergeFrac);
    m_pSameTypeMinReferenceSpin->setValue(processingSettings.fitSameTypeMinReferenceSegments);
    m_pSameTypeFlatSlopeSpin->setValue(processingSettings.fitSameTypeFlatSlope);
    m_pPlatformSnapCheck->setChecked(processingSettings.fitPlatformSnapEnable);
    m_pPlatformSnapFlatSlopeSpin->setValue(processingSettings.fitPlatformSnapFlatSlope);
    m_pPlatformSnapMinFracSpin->setValue(processingSettings.fitPlatformSnapMinFrac);
    m_pProjStationWindowSpin->setValue(processingSettings.projectionStationWindowMm);
    m_pProjTransverseWindowSpin->setValue(processingSettings.projectionTransverseWindowMm);
    m_pProjZBandBelowSpin->setValue(processingSettings.projectionZBandBelowMm);
    m_pProjZBandAboveSpin->setValue(processingSettings.projectionZBandAboveMm);
    m_pProjLayerLowSpin->setValue(processingSettings.projectionLayerLowPercent);
    m_pProjLayerHighSpin->setValue(processingSettings.projectionLayerHighPercent);
    m_pProjMaxCandidateSpin->setValue(processingSettings.projectionMaxCandidatePerSeed);
    m_pProjSmoothRadiusSpin->setValue(processingSettings.projectionSmoothRadius);
    ApplyMethodEnableState();
    LoadExternalAlgorithmConfig();
}

bool LaserWeldFilterDialog::SaveSettings(QString* error) const
{
    if (error != nullptr)
    {
        error->clear();
    }
    // finished/自动保存也会进入这里；撤权关闭页面时必须 fail closed，不能把
    // 未提交的外部 DLL/INI 路径作为副作用写回配置库。
    if (!m_liveSessionGuard || !m_liveSessionGuard())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("账号会话或工程师权限已失效，设置未保存。");
        }
        return false;
    }
    PointCloudProcessingConfig::Settings processingSettings = PointCloudProcessingConfig::Load();
    processingSettings.mode = CurrentProcessingMode();
    processingSettings.featurePointStrategy = CurrentFeaturePointStrategy();
    processingSettings.libraryDir = m_pExternalLibraryDirEdit->text().trimmed();
    processingSettings.configPath = m_pExternalConfigPathEdit->text().trimmed();
    processingSettings.zTruncationValue = m_pExternalZTruncationSpin->value();
    processingSettings.resampleStepMm = m_pExternalResampleStepSpin->value();
    processingSettings.sdkUseWeldedStartTruncation =
        m_pSdkWeldedStartCheck != nullptr && m_pSdkWeldedStartCheck->isChecked();
    {
        const QVariant axisData = m_pAxisCombo->currentData();
        processingSettings.sampleAxisMode = axisData.isValid()
            ? static_cast<PointCloudProcessingConfig::SampleAxisMode>(axisData.toInt())
            : PointCloudProcessingConfig::SampleAxisMode::Auto;
    }
    processingSettings.cloudZThresholdMm = m_pZThresholdSpin->value();
    processingSettings.cloudZJumpThresholdMm = m_pZJumpThresholdSpin->value();
    processingSettings.cloudZContinuityThresholdMm = m_pZContinuityThresholdSpin->value();
    processingSettings.cloudSegmentBreakDistanceMm = m_pSegmentBreakDistanceSpin->value();
    processingSettings.cloudKeepLongestSegmentOnly = m_pKeepLongestSegmentCheck->isChecked();
    processingSettings.fitSampleStepMm = m_pStepSpin->value();
    processingSettings.fitSearchWindowMm = m_pWindowSpin->value();
    processingSettings.fitPiecewiseToleranceMm = m_pPiecewiseToleranceSpin->value();
    processingSettings.fitPiecewiseMinSegmentPoints = m_pPiecewiseMinSegmentSpin->value();
    processingSettings.fitMinPointCount = m_pMinPointSpin->value();
    processingSettings.fitSmoothRadius = m_pSmoothRadiusSpin->value();
    processingSettings.fitAzimuthTurnThresholdDeg = m_pAzimuthTurnThresholdSpin->value();
    processingSettings.fitAzimuthHeadingWindow = m_pAzimuthHeadingWindowSpin->value();
    processingSettings.fitAzimuthNmsSpanMm = m_pAzimuthNmsSpanSpin->value();
    processingSettings.fitAzimuthStraightenResidualMm = m_pAzimuthStraightenResidualSpin->value();
    processingSettings.fitCornerRefineEnable = m_pCornerRefineEnableCheck->isChecked();
    processingSettings.fitAzimuthRefineFloorMm = m_pAzimuthRefineFloorSpin->value();
    processingSettings.fitCornerRefineOneSidedPct = m_pCornerRefineOneSidedSpin->value();
    processingSettings.fitCornerRefineMidMultiple = m_pCornerRefineMidMultipleSpin->value();
    processingSettings.fitCornerRefineEndFracPct = m_pCornerRefineEndFracSpin->value();
    processingSettings.fitCornerPatternRefitEnable = m_pCornerPatternRefitCheck->isChecked();
    processingSettings.fitCornerPlatformMinSegPoints = m_pCornerPlatformMinSegSpin->value();
    processingSettings.enableLapMisalignmentSplit = m_pLapSplitCheck->isChecked();
    processingSettings.lapStepHeightThresholdMm = m_pLapStepHeightSpin->value();
    processingSettings.lapStepStationWindowMm = m_pLapStepStationSpin->value();
    processingSettings.lapStepSideFlatnessMm = m_pLapStepFlatnessSpin->value();
    processingSettings.lapStepPlatformSlopeMax = m_pLapStepSlopeSpin->value();
    processingSettings.sdkBasePresmoothEnable = m_pSdkBasePresmoothCheck->isChecked();
    processingSettings.sdkBasePresmoothWindowMm = m_pSdkBasePresmoothWindowSpin->value();
    processingSettings.sdkBasePresmoothEdgeMm = m_pSdkBasePresmoothEdgeSpin->value();
    processingSettings.fitEdgeTruncateEnable = m_pEdgeTruncateCheck->isChecked();
    processingSettings.fitTruncateHeadMm = m_pTruncateHeadSpin->value();
    processingSettings.fitTruncateTailMm = m_pTruncateTailSpin->value();
    processingSettings.fitEndPeriodRecoverEnable = m_pEndPeriodRecoverCheck->isChecked();
    processingSettings.fitEndPeriodRatioThreshold = m_pEndPeriodRatioSpin->value();
    processingSettings.fitEndPeriodMinBendDeg = m_pEndPeriodMinBendSpin->value();
    processingSettings.fitSameTypeShortMergeEnable = m_pSameTypeShortMergeCheck->isChecked();
    processingSettings.fitEndPeriodMergeFrac = m_pEndPeriodMergeSpin->value();
    processingSettings.fitSameTypeMinReferenceSegments = m_pSameTypeMinReferenceSpin->value();
    processingSettings.fitSameTypeFlatSlope = m_pSameTypeFlatSlopeSpin->value();
    processingSettings.fitPlatformSnapEnable = m_pPlatformSnapCheck->isChecked();
    processingSettings.fitPlatformSnapFlatSlope = m_pPlatformSnapFlatSlopeSpin->value();
    processingSettings.fitPlatformSnapMinFrac = m_pPlatformSnapMinFracSpin->value();
    processingSettings.projectionStationWindowMm = m_pProjStationWindowSpin->value();
    processingSettings.projectionTransverseWindowMm = m_pProjTransverseWindowSpin->value();
    processingSettings.projectionZBandBelowMm = m_pProjZBandBelowSpin->value();
    processingSettings.projectionZBandAboveMm = m_pProjZBandAboveSpin->value();
    processingSettings.projectionLayerLowPercent = m_pProjLayerLowSpin->value();
    processingSettings.projectionLayerHighPercent = m_pProjLayerHighSpin->value();
    processingSettings.projectionMaxCandidatePerSeed = m_pProjMaxCandidateSpin->value();
    processingSettings.projectionSmoothRadius = m_pProjSmoothRadiusSpin->value();
    processingSettings.slopeConsistentCornerFit =
        m_pSlopeConsistentCornerFitCheck != nullptr && m_pSlopeConsistentCornerFitCheck->isChecked();
    processingSettings.exportFitDebugCloud =
        m_pExportFitDebugCloudCheck == nullptr || m_pExportFitDebugCloudCheck->isChecked();
    processingSettings.exportWorkpieceFrameDebug =
        m_pExportWorkpieceFrameDebugCheck != nullptr && m_pExportWorkpieceFrameDebugCheck->isChecked();
    processingSettings.validationPolicy = m_pValidationAuditOnlyCheck->isChecked()
        ? PointCloudProcessingConfig::ValidationPolicy::Audit
        : PointCloudProcessingConfig::ValidationPolicy::Enforce;
    processingSettings.validationProfileVersion = PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION;
    processingSettings.validationMinFinitePointCount = m_pValidationMinFinitePointSpin->value();
    processingSettings.validationMinProjectedSpanMm = m_pValidationMinProjectedSpanSpin->value();
    processingSettings.validationMinStationCoverageRatio = m_pValidationMinStationCoverageSpin->value() / 100.0;
    processingSettings.validationMinLongestContinuousRatio = m_pValidationMinLongestContinuousSpin->value() / 100.0;
    processingSettings.validationMaxRejectedRatio = m_pValidationMaxRejectedRatioSpin->value() / 100.0;
    processingSettings.validationMaxMedianResidualMm = m_pValidationMaxMedianResidualSpin->value();
    processingSettings.validationMaxP95ResidualMm = m_pValidationMaxP95ResidualSpin->value();
    processingSettings.validationResidualInlierThresholdMm = m_pValidationResidualInlierThresholdSpin->value();
    processingSettings.validationMinResidualInlierRatio = m_pValidationMinResidualInlierRatioSpin->value() / 100.0;
    processingSettings.validationMinKeyPointCount = m_pValidationMinKeyPointSpin->value();
    processingSettings.validationMinCornerCount = m_pValidationMinCornerSpin->value();
    processingSettings.validationMinSegmentLengthMm = m_pValidationMinSegmentLengthSpin->value();
    processingSettings.validationMinOutputPointCount = m_pValidationMinOutputPointSpin->value();
    processingSettings.validationMinOutputLengthRatio = m_pValidationMinOutputLengthRatioSpin->value() / 100.0;
    QString localError;
    if (!PointCloudProcessingConfig::Save(processingSettings, &localError))
    {
        if (error != nullptr)
        {
            *error = localError;
        }
        return false;
    }
    if (processingSettings.mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        || processingSettings.mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit)
    {
        SaveExternalAlgorithmConfig(&localError);
        if (!localError.isEmpty())
        {
            if (error != nullptr)
            {
                *error = localError;
            }
            return false;
        }
    }

    return true;
}

void LaserWeldFilterDialog::LoadExternalAlgorithmConfig()
{
    const QString configPath = m_pExternalConfigPathEdit != nullptr
        ? m_pExternalConfigPathEdit->text().trimmed()
        : QString();
    if (configPath.isEmpty() || !QFileInfo::exists(configPath))
    {
        return;
    }

    m_pCloudUprightCheck->setChecked(PlainIniBoolValue(ReadPlainIniValue(configPath, "ifupright", "true"), true));
    m_pCloudPlateThicknessSpin->setValue(ReadPlainIniValue(configPath, "Plate_thickness", "5").toDouble());
    m_pCloudRemoveFloorZSpin->setValue(ReadPlainIniValue(configPath, "Remove_Floor_ZValue", "6").toDouble());
    m_pCloudThreadNumberSpin->setValue(ReadPlainIniValue(configPath, "Thread_Number", "24").toInt());
    m_pCloudMoveDistanceSpin->setValue(ReadPlainIniValue(configPath, "Move_distance", "20").toDouble());
    m_pCloudDebugLogCheck->setChecked(PlainIniBoolValue(ReadPlainIniValue(configPath, "DEBUGLOG", "false"), false));
    SetPathEditText(m_pCloudLogPathEdit, QDir::toNativeSeparators(ReadPlainIniValue(configPath, "LOGPATH", QString())));
    m_pCloudPlaneThresholdSpin->setValue(ReadPlainIniValue(configPath, "Plane_Threshold", "3").toDouble());
    m_pCloudMergeLinesAngleSpin->setValue(ReadPlainIniValue(configPath, "Merge_Lines_Angle_Threshold", "20").toDouble());
    m_pCloudMergeLinesDistanceSpin->setValue(ReadPlainIniValue(configPath, "Merge_Lines_Dis_Threshold", "35").toDouble());
    m_pCloudClusterToleranceSpin->setValue(ReadPlainIniValue(configPath, "ClusterTolerance", "3.5").toDouble());
    m_pCloudClusterCheck->setChecked(PlainIniBoolValue(ReadPlainIniValue(configPath, "if_Cluster", "false"), false));
    m_pCloudRemoveNoiseCheck->setChecked(PlainIniBoolValue(ReadPlainIniValue(configPath, "is_remove_noise", "true"), true));
    m_pCloudSampleCheck->setChecked(PlainIniBoolValue(ReadPlainIniValue(configPath, "is_sample", "false"), false));
    m_pCloudSampleSizeSpin->setValue(ReadPlainIniValue(configPath, "sample_size", "5").toDouble());
    m_pCloudAboveZSpin->setValue(ReadPlainIniValue(configPath, "above_z", "0.5").toDouble());
    m_pCloudDiscreteValueSpin->setValue(ReadPlainIniIntValue(configPath, "Discrete_Value", 4));
    m_pCloudDilateValueSpin->setValue(ReadPlainIniIntValue(configPath, "Dilate_Value", 13));
    m_pCloudErodeValueSpin->setValue(ReadPlainIniIntValue(configPath, "Erode_Value", 9));
    m_pCloudLinesDisThresholdSpin->setValue(ReadPlainIniValue(configPath, "Lines_Dis_Threshold", "500").toDouble());
    m_pCloudLineLengthSpin->setValue(ReadPlainIniValue(configPath, "Line_Length", "70").toDouble());
}

void LaserWeldFilterDialog::SaveExternalAlgorithmConfig(QString* error) const
{
    const QString configPath = m_pExternalConfigPathEdit != nullptr
        ? m_pExternalConfigPathEdit->text().trimmed()
        : QString();
    if (configPath.isEmpty())
    {
        return;
    }

    const QVector<PlainIniValueUpdate> updates = {
        { "ifupright", BoolIniText(m_pCloudUprightCheck->isChecked()) },
        { "Plate_thickness", FormatPlainIniNumberLikeCurrent(configPath, "Plate_thickness", m_pCloudPlateThicknessSpin->value()) },
        { "Remove_Floor_ZValue", FormatPlainIniNumberLikeCurrent(configPath, "Remove_Floor_ZValue", m_pCloudRemoveFloorZSpin->value()) },
        { "Thread_Number", QString::number(m_pCloudThreadNumberSpin->value()) },
        { "Move_distance", FormatPlainIniNumberLikeCurrent(configPath, "Move_distance", m_pCloudMoveDistanceSpin->value()) },
        { "DEBUGLOG", BoolIniText(m_pCloudDebugLogCheck->isChecked()) },
        { "LOGPATH", QDir::toNativeSeparators(m_pCloudLogPathEdit->text().trimmed()) },
        { "Plane_Threshold", FormatPlainIniNumberLikeCurrent(configPath, "Plane_Threshold", m_pCloudPlaneThresholdSpin->value()) },
        { "Merge_Lines_Angle_Threshold", FormatPlainIniNumberLikeCurrent(configPath, "Merge_Lines_Angle_Threshold", m_pCloudMergeLinesAngleSpin->value()) },
        { "Merge_Lines_Dis_Threshold", FormatPlainIniNumberLikeCurrent(configPath, "Merge_Lines_Dis_Threshold", m_pCloudMergeLinesDistanceSpin->value()) },
        { "ClusterTolerance", FormatPlainIniNumberLikeCurrent(configPath, "ClusterTolerance", m_pCloudClusterToleranceSpin->value()) },
        { "if_Cluster", BoolIniText(m_pCloudClusterCheck->isChecked()) },
        { "is_remove_noise", BoolIniText(m_pCloudRemoveNoiseCheck->isChecked()) },
        { "is_sample", BoolIniText(m_pCloudSampleCheck->isChecked()) },
        { "sample_size", FormatPlainIniNumberLikeCurrent(configPath, "sample_size", m_pCloudSampleSizeSpin->value()) },
        { "above_z", FormatPlainIniNumberLikeCurrent(configPath, "above_z", m_pCloudAboveZSpin->value()) },
        { "Discrete_Value", QString::number(m_pCloudDiscreteValueSpin->value()) },
        { "Dilate_Value", QString::number(m_pCloudDilateValueSpin->value()) },
        { "Erode_Value", QString::number(m_pCloudErodeValueSpin->value()) },
        { "Lines_Dis_Threshold", FormatPlainIniNumberLikeCurrent(configPath, "Lines_Dis_Threshold", m_pCloudLinesDisThresholdSpin->value()) },
        { "Line_Length", FormatPlainIniNumberLikeCurrent(configPath, "Line_Length", m_pCloudLineLengthSpin->value()) }
    };
    WritePlainIniValues(configPath, updates, error);
}

void LaserWeldFilterDialog::BrowseExternalLibraryDir()
{
    const QString path = QFileDialog::getExistingDirectory(
        this,
        "选择新版点云库目录",
        m_pExternalLibraryDirEdit->text().trimmed(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (path.isEmpty())
    {
        return;
    }
    // Windows 原生目录框可能阻塞 Qt 会话定时器；返回后、修改任何可持久化
    // 外部 DLL 路径之前同步重验工程师会话。
    if (!m_liveSessionGuard || !m_liveSessionGuard())
    {
        AppendLog(QStringLiteral("账号会话或工程师权限已失效，已丢弃所选外部点云库目录。"));
        return;
    }

    SetPathEditText(m_pExternalLibraryDirEdit, QDir::toNativeSeparators(path));
    const QString defaultConfig = QDir(path).filePath("config/CorrugatedSheetPointCloudEctration.ini");
    if (m_pExternalConfigPathEdit->text().trimmed().isEmpty() || QFileInfo::exists(defaultConfig))
    {
        SetPathEditText(m_pExternalConfigPathEdit, QDir::toNativeSeparators(defaultConfig));
        LoadExternalAlgorithmConfig();
    }
    SaveSettings();
}

void LaserWeldFilterDialog::BrowseExternalConfigFile()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        "选择新版点云库配置",
        m_pExternalConfigPathEdit->text().trimmed(),
        "配置文件 (*.ini);;所有文件 (*.*)");
    if (path.isEmpty())
    {
        return;
    }
    if (!m_liveSessionGuard || !m_liveSessionGuard())
    {
        AppendLog(QStringLiteral("账号会话或工程师权限已失效，已丢弃所选外部点云库配置。"));
        return;
    }

    SetPathEditText(m_pExternalConfigPathEdit, QDir::toNativeSeparators(path));
    LoadExternalAlgorithmConfig();
    SaveSettings();
}

void LaserWeldFilterDialog::AppendLog(const QString& text)
{
    if (m_pLogText == nullptr)
    {
        return;
    }
    m_pLogText->appendPlainText(text);
}

PointCloudProcessingConfig::Mode LaserWeldFilterDialog::CurrentProcessingMode() const
{
    if (m_pProcessingModeCombo != nullptr && m_pProcessingModeCombo->currentIndex() >= 0)
    {
        return static_cast<PointCloudProcessingConfig::Mode>(m_pProcessingModeCombo->currentData().toInt());
    }
    return PointCloudProcessingConfig::Mode::LegacyLaserPath;
}

PointCloudProcessingConfig::FeaturePointStrategy LaserWeldFilterDialog::CurrentFeaturePointStrategy() const
{
    if (m_pFeaturePointStrategyCombo != nullptr && m_pFeaturePointStrategyCombo->currentIndex() >= 0)
    {
        return static_cast<PointCloudProcessingConfig::FeaturePointStrategy>(
            m_pFeaturePointStrategyCombo->currentData().toInt());
    }
    return PointCloudProcessingConfig::FeaturePointStrategy::LegacyGeometry;
}
