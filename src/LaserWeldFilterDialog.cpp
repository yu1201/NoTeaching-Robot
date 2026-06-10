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

LaserWeldFilterDialog::LaserWeldFilterDialog(QWidget* parent)
    : QDialog(parent)
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
    QPushButton* browseLibraryButton = new QPushButton("选择库目录");
    QPushButton* browseExternalConfigButton = new QPushButton("选择配置");
    processingLayout->setHorizontalSpacing(12);
    processingLayout->setVerticalSpacing(10);
    processingLayout->addWidget(new QLabel("Z截断值"), 0, 0);
    processingLayout->addWidget(CreateUnitEditor(m_pExternalZTruncationSpin, "mm"), 0, 1);
    processingLayout->addWidget(new QLabel("输出步长"), 0, 2);
    processingLayout->addWidget(CreateUnitEditor(m_pExternalResampleStepSpin, "mm"), 0, 3);
    processingLayout->addWidget(new QLabel("库目录"), 1, 0);
    processingLayout->addWidget(m_pExternalLibraryDirEdit, 1, 1, 1, 3);
    processingLayout->addWidget(browseLibraryButton, 1, 4);
    processingLayout->addWidget(new QLabel("配置文件"), 2, 0);
    processingLayout->addWidget(m_pExternalConfigPathEdit, 2, 1, 1, 3);
    processingLayout->addWidget(browseExternalConfigButton, 2, 4);
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
    // 去噪/分段参数属于程序滤波链，对走滤波拟合的方法（②③④）统一生效。
    m_pZThresholdSpin = new QDoubleSpinBox();
    m_pZThresholdSpin->setRange(-9999.0, 9999.0);
    m_pZThresholdSpin->setDecimals(3);
    m_pZThresholdSpin->setSingleStep(1.0);
    m_pZThresholdSpin->setValue(-230.0);

    m_pZJumpThresholdSpin = new QDoubleSpinBox();
    m_pZJumpThresholdSpin->setRange(0.0, 9999.0);
    m_pZJumpThresholdSpin->setDecimals(3);
    m_pZJumpThresholdSpin->setSingleStep(0.5);
    m_pZJumpThresholdSpin->setValue(3.0);
    m_pZJumpThresholdSpin->setSpecialValueText("关闭");

    m_pZContinuityThresholdSpin = new QDoubleSpinBox();
    m_pZContinuityThresholdSpin->setRange(0.0, 9999.0);
    m_pZContinuityThresholdSpin->setDecimals(3);
    m_pZContinuityThresholdSpin->setSingleStep(0.5);
    m_pZContinuityThresholdSpin->setValue(2.0);
    m_pZContinuityThresholdSpin->setSpecialValueText("关闭");

    m_pSegmentBreakDistanceSpin = new QDoubleSpinBox();
    m_pSegmentBreakDistanceSpin->setRange(0.0, 9999.0);
    m_pSegmentBreakDistanceSpin->setDecimals(3);
    m_pSegmentBreakDistanceSpin->setSingleStep(1.0);
    m_pSegmentBreakDistanceSpin->setValue(6.0);
    m_pSegmentBreakDistanceSpin->setSpecialValueText("关闭");

    m_pKeepLongestSegmentCheck = new QCheckBox("只保留最长连续段");
    m_pKeepLongestSegmentCheck->setChecked(true);

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

    m_pLineFitTrimSpin = new QSpinBox();
    m_pLineFitTrimSpin->setRange(0, 9999);
    m_pLineFitTrimSpin->setValue(0);

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

    m_pSlopeConsistentCornerFitCheck = new QCheckBox("直线拟合排除圆弧段");
    m_pSlopeConsistentCornerFitCheck->setToolTip("启用后，平台线和坡度线只使用局部斜率一致的直线段拟合，再求交生成拐点。");

    m_pExportFitDebugCloudCheck = new QCheckBox("导出拟合调试点云(CloudCompare)");
    m_pExportFitDebugCloudCheck->setToolTip("启用后，每次拟合都把各段用到的点集、拟合直线和关键点导出到输出文件同目录的 FitDebug 子目录，可直接拖入 CloudCompare 核对每段拟合是否正确。");
    m_pExportFitDebugCloudCheck->setChecked(true);

    // 滤波拟合方案置顶，方便切换；去噪/分段参数随后，再到拟合参数；采样主轴在方法组。
    paramLayout->addWidget(new QLabel("滤波拟合方案"), 0, 0);
    paramLayout->addWidget(m_pFeaturePointStrategyCombo, 0, 1, 1, 3);
    paramLayout->addWidget(new QLabel("下层 Z 阈值"), 1, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pZThresholdSpin, "mm"), 1, 1);
    paramLayout->addWidget(new QLabel("Z突变阈值"), 1, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pZJumpThresholdSpin, "mm"), 1, 3);
    paramLayout->addWidget(new QLabel("Z连续阈值"), 2, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pZContinuityThresholdSpin, "mm"), 2, 1);
    paramLayout->addWidget(new QLabel("段间跳变阈值"), 2, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pSegmentBreakDistanceSpin, "mm"), 2, 3);
    paramLayout->addWidget(new QLabel("输出步长"), 3, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pStepSpin, "mm"), 3, 1);
    paramLayout->addWidget(new QLabel("搜索窗口"), 3, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pWindowSpin, "mm"), 3, 3);
    paramLayout->addWidget(new QLabel("拟合裁首尾点数"), 4, 0);
    paramLayout->addWidget(m_pLineFitTrimSpin, 4, 1);
    paramLayout->addWidget(new QLabel("分段拟合容差"), 4, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pPiecewiseToleranceSpin, "mm"), 4, 3);
    paramLayout->addWidget(new QLabel("每段最少点数"), 5, 0);
    paramLayout->addWidget(m_pPiecewiseMinSegmentSpin, 5, 1);
    paramLayout->addWidget(new QLabel("最小点数"), 5, 2);
    paramLayout->addWidget(m_pMinPointSpin, 5, 3);
    paramLayout->addWidget(new QLabel("平滑半径"), 6, 0);
    paramLayout->addWidget(m_pSmoothRadiusSpin, 6, 1);
    paramLayout->addWidget(m_pKeepLongestSegmentCheck, 6, 2, 1, 2);
    paramLayout->addWidget(m_pSlopeConsistentCornerFitCheck, 7, 1, 1, 3);
    paramLayout->addWidget(m_pExportFitDebugCloudCheck, 8, 0, 1, 4);
    paramLayout->setColumnStretch(1, 1);
    paramLayout->setColumnStretch(3, 1);
    featurePointLayout->addWidget(paramGroup);
    featurePointLayout->addStretch(1);

    QGroupBox* validationGroup = new QGroupBox("点云有效性检测");
    QGridLayout* validationLayoutGrid = new QGridLayout(validationGroup);
    validationLayoutGrid->setHorizontalSpacing(12);
    validationLayoutGrid->setVerticalSpacing(10);

    m_pValidationCoverageCheck = new QCheckBox("启用采集覆盖检测");
    m_pValidationMinFinitePointSpin = new QSpinBox();
    m_pValidationMinFinitePointSpin->setRange(0, 10000000);
    m_pValidationMinProjectedSpanSpin = new QDoubleSpinBox();
    m_pValidationMinProjectedSpanSpin->setRange(0.0, 999999.0);
    m_pValidationMinProjectedSpanSpin->setDecimals(3);
    m_pValidationMinProjectedSpanSpin->setSingleStep(10.0);

    m_pValidationContinuityCheck = new QCheckBox("启用连续性检测");
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

    m_pValidationDenoiseRatioCheck = new QCheckBox("启用剔除比例检测");
    m_pValidationMaxRejectedRatioSpin = new QDoubleSpinBox();
    m_pValidationMaxRejectedRatioSpin->setRange(0.0, 100.0);
    m_pValidationMaxRejectedRatioSpin->setDecimals(1);
    m_pValidationMaxRejectedRatioSpin->setSingleStep(5.0);
    m_pValidationMaxRejectedRatioSpin->setSuffix("%");

    m_pValidationResidualCheck = new QCheckBox("启用拟合残差检测");
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

    m_pValidationKeyPointCheck = new QCheckBox("启用起终点/拐点检测");
    m_pValidationMinKeyPointSpin = new QSpinBox();
    m_pValidationMinKeyPointSpin->setRange(0, 9999);
    m_pValidationMinCornerSpin = new QSpinBox();
    m_pValidationMinCornerSpin->setRange(0, 9999);
    m_pValidationMinSegmentLengthSpin = new QDoubleSpinBox();
    m_pValidationMinSegmentLengthSpin->setRange(0.0, 9999.0);
    m_pValidationMinSegmentLengthSpin->setDecimals(3);

    m_pValidationOutputCheck = new QCheckBox("启用输出结果检测");
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
    validationLayoutGrid->setColumnStretch(1, 1);
    validationLayoutGrid->setColumnStretch(3, 1);
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
            const QString message = "精测点云处理设置已保存，先测后焊流程将使用当前配置。";
            AppendLog(message);
            QMessageBox::information(this, "精测点云处理", message);
        });
    connect(m_pFeaturePointStrategyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { SaveSettings(); });
    connect(m_pSlopeConsistentCornerFitCheck, &QCheckBox::toggled, this, [this](bool) { SaveSettings(); });
    connect(m_pExportFitDebugCloudCheck, &QCheckBox::toggled, this, [this](bool) { SaveSettings(); });
    connect(m_pExternalConfigPathEdit, &QLineEdit::editingFinished, this, &LaserWeldFilterDialog::LoadExternalAlgorithmConfig);
    connect(m_pProcessingModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
        {
            // 切换处理方法：按四方法启用矩阵联动禁用不相关参数，并保存。
            ApplyMethodEnableState();
            SaveSettings();
        });
    connect(m_pAxisCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { SaveSettings(); });

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
    if (m_pAlgorithmTabWidget != nullptr)
    {
        // 点云参数页：SDK 组（①②）+ 投影提取组（③），按组禁用；④整页禁用。
        // 滤波拟合参数页与有效性检测仅作用于程序滤波拟合链（②③④），①下禁用。
        m_pAlgorithmTabWidget->setTabEnabled(0, usesSdk || usesProjection);
        m_pAlgorithmTabWidget->setTabEnabled(1, usesFitChain);
        m_pAlgorithmTabWidget->setTabEnabled(2, usesFitChain);
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
    m_pValidationCoverageCheck->setChecked(processingSettings.validationCoverageEnabled);
    m_pValidationMinFinitePointSpin->setValue(processingSettings.validationMinFinitePointCount);
    m_pValidationMinProjectedSpanSpin->setValue(processingSettings.validationMinProjectedSpanMm);
    m_pValidationContinuityCheck->setChecked(processingSettings.validationContinuityEnabled);
    m_pValidationMinStationCoverageSpin->setValue(processingSettings.validationMinStationCoverageRatio * 100.0);
    m_pValidationMinLongestContinuousSpin->setValue(processingSettings.validationMinLongestContinuousRatio * 100.0);
    m_pValidationDenoiseRatioCheck->setChecked(processingSettings.validationDenoiseRatioEnabled);
    m_pValidationMaxRejectedRatioSpin->setValue(processingSettings.validationMaxRejectedRatio * 100.0);
    m_pValidationResidualCheck->setChecked(processingSettings.validationResidualEnabled);
    m_pValidationMaxMedianResidualSpin->setValue(processingSettings.validationMaxMedianResidualMm);
    m_pValidationMaxP95ResidualSpin->setValue(processingSettings.validationMaxP95ResidualMm);
    m_pValidationResidualInlierThresholdSpin->setValue(processingSettings.validationResidualInlierThresholdMm);
    m_pValidationMinResidualInlierRatioSpin->setValue(processingSettings.validationMinResidualInlierRatio * 100.0);
    m_pValidationKeyPointCheck->setChecked(processingSettings.validationKeyPointEnabled);
    m_pValidationMinKeyPointSpin->setValue(processingSettings.validationMinKeyPointCount);
    m_pValidationMinCornerSpin->setValue(processingSettings.validationMinCornerCount);
    m_pValidationMinSegmentLengthSpin->setValue(processingSettings.validationMinSegmentLengthMm);
    m_pValidationOutputCheck->setChecked(processingSettings.validationOutputEnabled);
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
    m_pLineFitTrimSpin->setValue(processingSettings.fitLineFitTrimCount);
    m_pPiecewiseToleranceSpin->setValue(processingSettings.fitPiecewiseToleranceMm);
    m_pPiecewiseMinSegmentSpin->setValue(processingSettings.fitPiecewiseMinSegmentPoints);
    m_pMinPointSpin->setValue(processingSettings.fitMinPointCount);
    m_pSmoothRadiusSpin->setValue(processingSettings.fitSmoothRadius);
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
    PointCloudProcessingConfig::Settings processingSettings = PointCloudProcessingConfig::Load();
    processingSettings.mode = CurrentProcessingMode();
    processingSettings.featurePointStrategy = CurrentFeaturePointStrategy();
    processingSettings.libraryDir = m_pExternalLibraryDirEdit->text().trimmed();
    processingSettings.configPath = m_pExternalConfigPathEdit->text().trimmed();
    processingSettings.zTruncationValue = m_pExternalZTruncationSpin->value();
    processingSettings.resampleStepMm = m_pExternalResampleStepSpin->value();
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
    processingSettings.fitLineFitTrimCount = m_pLineFitTrimSpin->value();
    processingSettings.fitPiecewiseToleranceMm = m_pPiecewiseToleranceSpin->value();
    processingSettings.fitPiecewiseMinSegmentPoints = m_pPiecewiseMinSegmentSpin->value();
    processingSettings.fitMinPointCount = m_pMinPointSpin->value();
    processingSettings.fitSmoothRadius = m_pSmoothRadiusSpin->value();
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
    processingSettings.validationCoverageEnabled = m_pValidationCoverageCheck->isChecked();
    processingSettings.validationMinFinitePointCount = m_pValidationMinFinitePointSpin->value();
    processingSettings.validationMinProjectedSpanMm = m_pValidationMinProjectedSpanSpin->value();
    processingSettings.validationContinuityEnabled = m_pValidationContinuityCheck->isChecked();
    processingSettings.validationMinStationCoverageRatio = m_pValidationMinStationCoverageSpin->value() / 100.0;
    processingSettings.validationMinLongestContinuousRatio = m_pValidationMinLongestContinuousSpin->value() / 100.0;
    processingSettings.validationDenoiseRatioEnabled = m_pValidationDenoiseRatioCheck->isChecked();
    processingSettings.validationMaxRejectedRatio = m_pValidationMaxRejectedRatioSpin->value() / 100.0;
    processingSettings.validationResidualEnabled = m_pValidationResidualCheck->isChecked();
    processingSettings.validationMaxMedianResidualMm = m_pValidationMaxMedianResidualSpin->value();
    processingSettings.validationMaxP95ResidualMm = m_pValidationMaxP95ResidualSpin->value();
    processingSettings.validationResidualInlierThresholdMm = m_pValidationResidualInlierThresholdSpin->value();
    processingSettings.validationMinResidualInlierRatio = m_pValidationMinResidualInlierRatioSpin->value() / 100.0;
    processingSettings.validationKeyPointEnabled = m_pValidationKeyPointCheck->isChecked();
    processingSettings.validationMinKeyPointCount = m_pValidationMinKeyPointSpin->value();
    processingSettings.validationMinCornerCount = m_pValidationMinCornerSpin->value();
    processingSettings.validationMinSegmentLengthMm = m_pValidationMinSegmentLengthSpin->value();
    processingSettings.validationOutputEnabled = m_pValidationOutputCheck->isChecked();
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

