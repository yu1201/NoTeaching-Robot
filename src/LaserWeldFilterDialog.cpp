#include "LaserWeldFilterDialog.h"

#include "ConfigDatabase.h"
#include "PointCloudExtractionProcessor.h"
#include "PointCloudProcessingConfig.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
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

QString LaserFilterSettingsPath()
{
    return QCoreApplication::applicationDirPath() + "/LaserWeldFilterDialog.ini";
}

QString BuildSuggestedOutputPath(const QString& inputPath, double step)
{
    if (inputPath.trimmed().isEmpty())
    {
        return QString();
    }

    const QFileInfo info(inputPath);
    const QString baseName = info.completeBaseName();
    const QString suffix = info.suffix().isEmpty() ? "txt" : info.suffix();
    const QString stepText = QString::number(step, 'f', step == std::floor(step) ? 0 : 1);
    return info.dir().filePath(QString("%1_Filtered_%2mm.%3").arg(baseName, stepText, suffix));
}

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
    UpdateSuggestedOutputPath();
    connect(this, &QDialog::finished, this, [this](int) { SaveSettings(); });
}

void LaserWeldFilterDialog::BuildUi()
{
    QVBoxLayout* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(18, 18, 18, 18);
    outerLayout->setSpacing(12);

    QLabel* titleLabel = new QLabel("精测点云处理");
    titleLabel->setObjectName("TitleLabel");
    QLabel* hintLabel = new QLabel("管理预设参数扫描后的局部精测点云处理方式；旧版使用目标轨迹点，新版使用局部完整点云并调用外部精测库。");
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

    QGroupBox* processingGroup = new QGroupBox("点云算法调用参数");
    QGridLayout* processingLayout = new QGridLayout(processingGroup);
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
    m_pExternalFallbackCheck = new QCheckBox("新版失败时回退旧版");
    m_pCloudAxisCombo = new QComboBox();
    m_pCloudAxisCombo->addItem("按 Y 方向判断平台/上坡/下坡", static_cast<int>(RobotCalculation::SampleAxis::AxisY));
    m_pCloudAxisCombo->addItem("按 X 方向判断平台/上坡/下坡", static_cast<int>(RobotCalculation::SampleAxis::AxisX));
    QPushButton* browseLibraryButton = new QPushButton("选择库目录");
    QPushButton* browseExternalConfigButton = new QPushButton("选择配置");
    processingLayout->setHorizontalSpacing(12);
    processingLayout->setVerticalSpacing(10);
    processingLayout->addWidget(new QLabel("属性主轴"), 0, 0);
    processingLayout->addWidget(m_pCloudAxisCombo, 0, 1);
    processingLayout->addWidget(m_pExternalFallbackCheck, 0, 2, 1, 3);
    processingLayout->addWidget(new QLabel("Z截断值"), 1, 0);
    processingLayout->addWidget(CreateUnitEditor(m_pExternalZTruncationSpin, "mm"), 1, 1);
    processingLayout->addWidget(new QLabel("输出步长"), 1, 2);
    processingLayout->addWidget(CreateUnitEditor(m_pExternalResampleStepSpin, "mm"), 1, 3);
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
    pointCloudLayout->addWidget(processingGroup);

    QGroupBox* cloudInnerGroup = new QGroupBox("点云算法内部参数");
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
    cloudInnerLayout->setColumnStretch(1, 1);
    cloudInnerLayout->setColumnStretch(3, 1);
    cloudInnerLayout->setColumnMinimumWidth(1, 300);
    cloudInnerLayout->setColumnMinimumWidth(3, 300);
    pointCloudLayout->addWidget(cloudInnerGroup);
    pointCloudLayout->addStretch(1);

    QGroupBox* methodGroup = new QGroupBox("处理方法选择");
    QGridLayout* methodLayout = new QGridLayout(methodGroup);
    methodLayout->setHorizontalSpacing(12);
    methodLayout->setVerticalSpacing(8);
    m_pProcessingModeCombo = new QComboBox();
    m_pProcessingModeCombo->addItem("点云算法", static_cast<int>(PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet));
    m_pProcessingModeCombo->addItem("特征点算法", static_cast<int>(PointCloudProcessingConfig::Mode::LegacyLaserPath));
    QLabel* methodHintLabel = new QLabel("点云算法调用完整局部点云和外部 DLL；特征点算法使用旧版目标点/特征点处理。");
    methodHintLabel->setWordWrap(true);
    methodLayout->addWidget(new QLabel("当前方法"), 0, 0);
    methodLayout->addWidget(m_pProcessingModeCombo, 0, 1);
    methodLayout->addWidget(methodHintLabel, 0, 2);
    methodLayout->setColumnStretch(2, 1);
    rootLayout->addWidget(methodGroup);

    QGroupBox* fileGroup = new QGroupBox("单文件测试");
    QGridLayout* fileLayout = new QGridLayout(fileGroup);
    m_pInputPathEdit = new QLineEdit();
    m_pOutputPathEdit = new QLineEdit();
    QPushButton* browseInputButton = new QPushButton("选择输入文件");
    QPushButton* browseOutputButton = new QPushButton("选择输出文件");
    fileLayout->addWidget(new QLabel("输入局部点云"), 0, 0);
    fileLayout->addWidget(m_pInputPathEdit, 0, 1);
    fileLayout->addWidget(browseInputButton, 0, 2);
    fileLayout->addWidget(new QLabel("输出轨迹文件"), 1, 0);
    fileLayout->addWidget(m_pOutputPathEdit, 1, 1);
    fileLayout->addWidget(browseOutputButton, 1, 2);
    fileLayout->setColumnStretch(1, 1);
    rootLayout->addWidget(fileGroup);

    QGroupBox* paramGroup = new QGroupBox("特征点算法参数");
    QGridLayout* paramLayout = new QGridLayout(paramGroup);
    m_pAxisCombo = new QComboBox();
    m_pAxisCombo->addItem("按 Y 方向每隔固定距离采样", static_cast<int>(RobotCalculation::SampleAxis::AxisY));
    m_pAxisCombo->addItem("按 X 方向每隔固定距离采样", static_cast<int>(RobotCalculation::SampleAxis::AxisX));

    m_pFitModeCombo = new QComboBox();
    m_pFitModeCombo->addItem("保持原始轨迹", static_cast<int>(RobotCalculation::LowerWeldFitMode::PreservePath));
    m_pFitModeCombo->addItem("直线拟合输出", static_cast<int>(RobotCalculation::LowerWeldFitMode::LineFit));
    m_pFitModeCombo->addItem("梯形分段拟合输出", static_cast<int>(RobotCalculation::LowerWeldFitMode::TrapezoidFit));
    m_pFitModeCombo->addItem("多段分段直线拟合", static_cast<int>(RobotCalculation::LowerWeldFitMode::PiecewiseLineFit));

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
    m_pFeaturePointStrategyCombo->addItem(
        PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::WorkpieceProjection),
        static_cast<int>(PointCloudProcessingConfig::FeaturePointStrategy::WorkpieceProjection));

    m_pZThresholdSpin = new QDoubleSpinBox();
    m_pZThresholdSpin->setRange(-9999.0, 9999.0);
    m_pZThresholdSpin->setDecimals(3);
    m_pZThresholdSpin->setSingleStep(1.0);
    m_pZThresholdSpin->setValue(-230.0);

    m_pZJumpThresholdSpin = new QDoubleSpinBox();
    m_pZJumpThresholdSpin->setRange(0.0, 9999.0);
    m_pZJumpThresholdSpin->setDecimals(3);
    m_pZJumpThresholdSpin->setSingleStep(0.5);
    m_pZJumpThresholdSpin->setValue(5.0);
    m_pZJumpThresholdSpin->setSpecialValueText("关闭");

    m_pZContinuityThresholdSpin = new QDoubleSpinBox();
    m_pZContinuityThresholdSpin->setRange(0.0, 9999.0);
    m_pZContinuityThresholdSpin->setDecimals(3);
    m_pZContinuityThresholdSpin->setSingleStep(0.5);
    m_pZContinuityThresholdSpin->setValue(3.0);
    m_pZContinuityThresholdSpin->setSpecialValueText("关闭");

    m_pSegmentBreakDistanceSpin = new QDoubleSpinBox();
    m_pSegmentBreakDistanceSpin->setRange(0.0, 9999.0);
    m_pSegmentBreakDistanceSpin->setDecimals(3);
    m_pSegmentBreakDistanceSpin->setSingleStep(1.0);
    m_pSegmentBreakDistanceSpin->setValue(12.0);
    m_pSegmentBreakDistanceSpin->setSpecialValueText("关闭");

    m_pKeepLongestSegmentCheck = new QCheckBox("只保留最长连续段");
    m_pKeepLongestSegmentCheck->setChecked(true);

    m_pStepSpin = new QDoubleSpinBox();
    m_pStepSpin->setRange(0.1, 100.0);
    m_pStepSpin->setDecimals(3);
    m_pStepSpin->setSingleStep(0.5);
    m_pStepSpin->setValue(2.0);

    m_pWindowSpin = new QDoubleSpinBox();
    m_pWindowSpin->setRange(0.0, 100.0);
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
    m_pPiecewiseToleranceSpin->setValue(2.0);

    m_pPiecewiseMinSegmentSpin = new QSpinBox();
    m_pPiecewiseMinSegmentSpin->setRange(2, 9999);
    m_pPiecewiseMinSegmentSpin->setValue(4);

    m_pMinPointSpin = new QSpinBox();
    m_pMinPointSpin->setRange(1, 999);
    m_pMinPointSpin->setValue(3);

    m_pSmoothRadiusSpin = new QSpinBox();
    m_pSmoothRadiusSpin->setRange(0, 20);
    m_pSmoothRadiusSpin->setValue(2);

    paramLayout->addWidget(new QLabel("采样主轴"), 0, 0);
    paramLayout->addWidget(m_pAxisCombo, 0, 1);
    paramLayout->addWidget(new QLabel("输出模式"), 0, 2);
    paramLayout->addWidget(m_pFitModeCombo, 0, 3);
    paramLayout->addWidget(new QLabel("下层 Z 阈值"), 1, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pZThresholdSpin, "mm"), 1, 1);
    paramLayout->addWidget(new QLabel("Z突变阈值"), 1, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pZJumpThresholdSpin, "mm"), 1, 3);
    paramLayout->addWidget(new QLabel("Z连续阈值"), 2, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pZContinuityThresholdSpin, "mm"), 2, 1);
    paramLayout->addWidget(new QLabel("输出步长"), 2, 2);
    paramLayout->addWidget(CreateUnitEditor(m_pStepSpin, "mm"), 2, 3);
    paramLayout->addWidget(new QLabel("搜索窗口"), 3, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pWindowSpin, "mm"), 3, 1);
    paramLayout->addWidget(new QLabel("拟合裁首尾点数"), 3, 2);
    paramLayout->addWidget(m_pLineFitTrimSpin, 3, 3);
    paramLayout->addWidget(new QLabel("分段拟合容差"), 4, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pPiecewiseToleranceSpin, "mm"), 4, 1);
    paramLayout->addWidget(new QLabel("每段最少点数"), 4, 2);
    paramLayout->addWidget(m_pPiecewiseMinSegmentSpin, 4, 3);
    paramLayout->addWidget(new QLabel("最小点数"), 5, 0);
    paramLayout->addWidget(m_pMinPointSpin, 5, 1);
    paramLayout->addWidget(new QLabel("平滑半径"), 5, 2);
    paramLayout->addWidget(m_pSmoothRadiusSpin, 5, 3);
    paramLayout->addWidget(new QLabel("段间跳变阈值"), 6, 0);
    paramLayout->addWidget(CreateUnitEditor(m_pSegmentBreakDistanceSpin, "mm"), 6, 1);
    paramLayout->addWidget(m_pKeepLongestSegmentCheck, 6, 2, 1, 2);
    paramLayout->addWidget(new QLabel("特征点拟合方案"), 7, 0);
    paramLayout->addWidget(m_pFeaturePointStrategyCombo, 7, 1, 1, 3);
    paramLayout->setColumnStretch(1, 1);
    paramLayout->setColumnStretch(3, 1);
    featurePointLayout->addWidget(paramGroup);
    featurePointLayout->addStretch(1);
    m_pAlgorithmTabWidget->addTab(pointCloudTab, "点云算法");
    m_pAlgorithmTabWidget->addTab(featurePointTab, "特征点算法");
    rootLayout->addWidget(m_pAlgorithmTabWidget);
    rootLayout->addStretch(1);

    settingsScrollArea->setWidget(settingsContent);
    outerLayout->addWidget(settingsScrollArea, 1);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch(1);
    QPushButton* saveSettingsButton = new QPushButton("保存设置");
    saveSettingsButton->setMinimumSize(150, 42);
    m_pRunButton = new QPushButton("开始处理");
    m_pRunButton->setMinimumSize(180, 42);
    actionLayout->addWidget(saveSettingsButton);
    actionLayout->addWidget(m_pRunButton);
    outerLayout->addLayout(actionLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1200);
    m_pLogText->setMinimumHeight(110);
    m_pLogText->setMaximumHeight(170);
    m_pLogText->setPlainText("处理日志：等待选择文件...");
    outerLayout->addWidget(m_pLogText);

    connect(browseInputButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::BrowseInputFile);
    connect(browseOutputButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::BrowseOutputFile);
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
    connect(m_pRunButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::RunFilter);
    connect(m_pStepSpin, &QDoubleSpinBox::valueChanged, this, [this](double) { UpdateSuggestedOutputPath(); });
    connect(m_pExternalResampleStepSpin, &QDoubleSpinBox::valueChanged, this, [this](double) { UpdateSuggestedOutputPath(); });
    connect(m_pFeaturePointStrategyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { SaveSettings(); });
    connect(m_pExternalConfigPathEdit, &QLineEdit::editingFinished, this, &LaserWeldFilterDialog::LoadExternalAlgorithmConfig);
    connect(m_pProcessingModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_pProcessingModeCombo == nullptr || m_pAlgorithmTabWidget == nullptr)
            {
                return;
            }
            const PointCloudProcessingConfig::Mode mode =
                static_cast<PointCloudProcessingConfig::Mode>(m_pProcessingModeCombo->currentData().toInt());
            const int tabIndex = mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet ? 0 : 1;
            if (m_pAlgorithmTabWidget->currentIndex() != tabIndex)
            {
                const QSignalBlocker blocker(m_pAlgorithmTabWidget);
                m_pAlgorithmTabWidget->setCurrentIndex(tabIndex);
            }
            UpdateSuggestedOutputPath();
            SaveSettings();
        });
    connect(m_pAlgorithmTabWidget, &QTabWidget::currentChanged, this, [this](int)
        {
            if (m_pProcessingModeCombo != nullptr)
            {
                const PointCloudProcessingConfig::Mode mode =
                    m_pAlgorithmTabWidget != nullptr && m_pAlgorithmTabWidget->currentIndex() == 0
                    ? PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
                    : PointCloudProcessingConfig::Mode::LegacyLaserPath;
                const int modeIndex = m_pProcessingModeCombo->findData(static_cast<int>(mode));
                if (modeIndex >= 0 && m_pProcessingModeCombo->currentIndex() != modeIndex)
                {
                    const QSignalBlocker blocker(m_pProcessingModeCombo);
                    m_pProcessingModeCombo->setCurrentIndex(modeIndex);
                }
            }
            UpdateSuggestedOutputPath();
            SaveSettings();
        });

    ApplyResponsivePageDefaults(this);
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
        "QPlainTextEdit { background: #0B1117; color: #BFE7EA; border: 1px solid #2E4656; border-radius: 10px; padding: 8px; }")
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
    if (m_pAlgorithmTabWidget != nullptr)
    {
        const QSignalBlocker blocker(m_pAlgorithmTabWidget);
        m_pAlgorithmTabWidget->setCurrentIndex(
            processingSettings.mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet ? 0 : 1);
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
    m_pExternalFallbackCheck->setChecked(processingSettings.fallbackToLegacy);

    const QString configPath = LaserFilterSettingsPath();
    const auto read = [&configPath](const QString& key, const QString& defaultValue = QString())
        {
            QString value;
            return ConfigDatabase::ReadSetting(configPath, key, &value) ? value : defaultValue;
        };
    const auto readBool = [&read](const QString& key, bool defaultValue)
        {
            const QString value = read(key);
            if (value.isEmpty())
            {
                return defaultValue;
            }
            const QString normalized = value.trimmed().toLower();
            return normalized == "1" || normalized == "true" || normalized == "yes";
        };

    m_pInputPathEdit->setText(read("Path/Input"));
    m_pOutputPathEdit->setText(read("Path/Output"));
    m_pCloudAxisCombo->setCurrentIndex(read("Cloud/Axis", "0").toInt());
    m_pAxisCombo->setCurrentIndex(read("Param/Axis", "0").toInt());
    m_pFitModeCombo->setCurrentIndex(read("Param/FitMode", "0").toInt());
    m_pZThresholdSpin->setValue(read("Param/ZThreshold", "-230.0").toDouble());
    m_pZJumpThresholdSpin->setValue(read("Param/ZJumpThreshold", "5.0").toDouble());
    m_pZContinuityThresholdSpin->setValue(read("Param/ZContinuityThreshold", "3.0").toDouble());
    m_pSegmentBreakDistanceSpin->setValue(read("Param/SegmentBreakDistance", "12.0").toDouble());
    m_pKeepLongestSegmentCheck->setChecked(readBool("Param/KeepLongestSegmentOnly", true));
    m_pStepSpin->setValue(read("Param/Step", "2.0").toDouble());
    m_pWindowSpin->setValue(read("Param/SearchWindow", "8.0").toDouble());
    m_pLineFitTrimSpin->setValue(read("Param/LineFitTrimCount", "0").toInt());
    m_pPiecewiseToleranceSpin->setValue(read("Param/PiecewiseFitTolerance", "2.0").toDouble());
    m_pPiecewiseMinSegmentSpin->setValue(read("Param/PiecewiseMinSegmentPoints", "4").toInt());
    m_pMinPointSpin->setValue(read("Param/MinPointCount", "3").toInt());
    m_pSmoothRadiusSpin->setValue(read("Param/SmoothRadius", "2").toInt());
    LoadExternalAlgorithmConfig();
}

bool LaserWeldFilterDialog::SaveSettings(QString* error) const
{
    PointCloudProcessingConfig::Settings processingSettings;
    processingSettings.mode = CurrentProcessingMode();
    processingSettings.featurePointStrategy = CurrentFeaturePointStrategy();
    processingSettings.libraryDir = m_pExternalLibraryDirEdit->text().trimmed();
    processingSettings.configPath = m_pExternalConfigPathEdit->text().trimmed();
    processingSettings.zTruncationValue = m_pExternalZTruncationSpin->value();
    processingSettings.resampleStepMm = m_pExternalResampleStepSpin->value();
    processingSettings.fallbackToLegacy = m_pExternalFallbackCheck->isChecked();
    QString localError;
    if (!PointCloudProcessingConfig::Save(processingSettings, &localError))
    {
        if (error != nullptr)
        {
            *error = localError;
        }
        return false;
    }
    if (processingSettings.mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet)
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

    const QString configPath = LaserFilterSettingsPath();
    bool ok = true;
    const auto write = [&configPath](const QString& key, const QString& value)
        {
            return ConfigDatabase::WriteSetting(configPath, key, value);
        };

    ok = write("Path/Input", m_pInputPathEdit->text().trimmed()) && ok;
    ok = write("Path/Output", m_pOutputPathEdit->text().trimmed()) && ok;
    ok = write("Cloud/Axis", QString::number(m_pCloudAxisCombo->currentIndex())) && ok;
    ok = write("Param/Axis", QString::number(m_pAxisCombo->currentIndex())) && ok;
    ok = write("Param/FitMode", QString::number(m_pFitModeCombo->currentIndex())) && ok;
    ok = write("Param/ZThreshold", QString::number(m_pZThresholdSpin->value(), 'f', 6)) && ok;
    ok = write("Param/ZJumpThreshold", QString::number(m_pZJumpThresholdSpin->value(), 'f', 6)) && ok;
    ok = write("Param/ZContinuityThreshold", QString::number(m_pZContinuityThresholdSpin->value(), 'f', 6)) && ok;
    ok = write("Param/SegmentBreakDistance", QString::number(m_pSegmentBreakDistanceSpin->value(), 'f', 6)) && ok;
    ok = write("Param/KeepLongestSegmentOnly", m_pKeepLongestSegmentCheck->isChecked() ? "1" : "0") && ok;
    ok = write("Param/Step", QString::number(m_pStepSpin->value(), 'f', 6)) && ok;
    ok = write("Param/SearchWindow", QString::number(m_pWindowSpin->value(), 'f', 6)) && ok;
    ok = write("Param/LineFitTrimCount", QString::number(m_pLineFitTrimSpin->value())) && ok;
    ok = write("Param/PiecewiseFitTolerance", QString::number(m_pPiecewiseToleranceSpin->value(), 'f', 6)) && ok;
    ok = write("Param/PiecewiseMinSegmentPoints", QString::number(m_pPiecewiseMinSegmentSpin->value())) && ok;
    ok = write("Param/MinPointCount", QString::number(m_pMinPointSpin->value())) && ok;
    ok = write("Param/SmoothRadius", QString::number(m_pSmoothRadiusSpin->value())) && ok;
    if (!ok)
    {
        if (error != nullptr)
        {
            *error = "写入精测点云处理测试参数失败。";
        }
        return false;
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
        { "Erode_Value", QString::number(m_pCloudErodeValueSpin->value()) }
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

void LaserWeldFilterDialog::BrowseInputFile()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        "选择激光点文件",
        m_pInputPathEdit->text().trimmed(),
        "点文件 (*.txt *.csv);;所有文件 (*.*)");
    if (path.isEmpty())
    {
        return;
    }

    m_pInputPathEdit->setText(QDir::toNativeSeparators(path));
    m_pOutputPathEdit->setText(QDir::toNativeSeparators(BuildSuggestedOutputPath(path, CurrentOutputStep())));
    SaveSettings();
}

void LaserWeldFilterDialog::BrowseOutputFile()
{
    const QString suggestedPath = m_pOutputPathEdit->text().trimmed().isEmpty()
        ? BuildSuggestedOutputPath(m_pInputPathEdit->text().trimmed(), CurrentOutputStep())
        : m_pOutputPathEdit->text().trimmed();
    const QString path = QFileDialog::getSaveFileName(
        this,
        "选择输出结果文件",
        suggestedPath,
        "文本文件 (*.txt);;CSV文件 (*.csv);;所有文件 (*.*)");
    if (path.isEmpty())
    {
        return;
    }

    m_pOutputPathEdit->setText(QDir::toNativeSeparators(path));
    SaveSettings();
}

void LaserWeldFilterDialog::UpdateSuggestedOutputPath()
{
    if (m_pInputPathEdit == nullptr || m_pOutputPathEdit == nullptr)
    {
        return;
    }

    if (!m_pOutputPathEdit->text().trimmed().isEmpty())
    {
        return;
    }

    const QString suggested = BuildSuggestedOutputPath(m_pInputPathEdit->text().trimmed(), CurrentOutputStep());
    if (!suggested.isEmpty())
    {
        m_pOutputPathEdit->setText(QDir::toNativeSeparators(suggested));
    }
}

void LaserWeldFilterDialog::RunFilter()
{
    const QString inputPath = m_pInputPathEdit->text().trimmed();
    if (inputPath.isEmpty())
    {
        QMessageBox::warning(this, "精测点云处理", "请先选择输入点文件。");
        return;
    }

    QString outputPath = m_pOutputPathEdit->text().trimmed();
    if (outputPath.isEmpty())
    {
        outputPath = BuildSuggestedOutputPath(inputPath, CurrentOutputStep());
        m_pOutputPathEdit->setText(QDir::toNativeSeparators(outputPath));
    }

    QVector<RobotCalculation::IndexedPoint3D> inputPoints;
    QString error;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(inputPath, inputPoints, &error))
    {
        AppendLog(error);
        QMessageBox::warning(this, "精测点云处理", error);
        return;
    }

    SaveSettings();

    const PointCloudProcessingConfig::Mode processingMode = CurrentProcessingMode();
    RobotCalculation::LowerWeldFilterResult result;
    QString processingSummary;
    if (processingMode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet)
    {
        PointCloudProcessingConfig::Settings processingSettings = PointCloudProcessingConfig::Load();
        processingSettings.mode = processingMode;
        processingSettings.libraryDir = m_pExternalLibraryDirEdit->text().trimmed();
        processingSettings.configPath = m_pExternalConfigPathEdit->text().trimmed();
        processingSettings.zTruncationValue = m_pExternalZTruncationSpin->value();
        processingSettings.resampleStepMm = m_pExternalResampleStepSpin->value();
        processingSettings.fallbackToLegacy = m_pExternalFallbackCheck->isChecked();

        const PointCloudExtractionProcessor::ExtractionResult extraction =
            PointCloudExtractionProcessor::ExtractCorrugatedSheet(
                inputPoints,
                processingSettings,
                Eigen::Vector3d::UnitX());
        if (!extraction.ok)
        {
            AppendLog(extraction.error);
            QMessageBox::warning(this, "精测点云处理", extraction.error);
            return;
        }

        const RobotCalculation::MeasureThenWeldAnalysisResult analysis =
            PointCloudExtractionProcessor::BuildAnalysisResult(extraction, CurrentParams());
        if (!analysis.ok)
        {
            AppendLog(analysis.error);
            QMessageBox::warning(this, "精测点云处理", analysis.error);
            return;
        }

        result = analysis.filterResult;
        processingSummary = QString("点云算法：DLL=%1\n配置=%2\nZ截断=%3 mm\n重采样步长=%4 mm")
            .arg(extraction.dllPath)
            .arg(extraction.configPath)
            .arg(processingSettings.zTruncationValue, 0, 'f', 3)
            .arg(processingSettings.resampleStepMm, 0, 'f', 3);
    }
    else
    {
        if (CurrentFeaturePointStrategy() == PointCloudProcessingConfig::FeaturePointStrategy::WorkpieceProjection)
        {
            RobotCalculation::LowerWeldFilterParams seedParams = CurrentParams();
            seedParams.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::SlopeWaveFiltered;
            const RobotCalculation::MeasureThenWeldAnalysisResult seedAnalysis =
                RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(inputPoints, seedParams);
            if (!seedAnalysis.ok)
            {
                AppendLog(seedAnalysis.error);
                QMessageBox::warning(this, "精测点云处理", seedAnalysis.error);
                return;
            }

            QVector<RobotCalculation::IndexedPoint3D> seedPoints;
            seedPoints.reserve(seedAnalysis.filterResult.points.size());
            for (const RobotCalculation::LowerWeldFilterPoint& point : seedAnalysis.filterResult.points)
            {
                RobotCalculation::IndexedPoint3D seedPoint;
                seedPoint.index = point.index;
                seedPoint.point = point.point;
                seedPoints.push_back(seedPoint);
            }

            result = RobotCalculation::ProjectWorkpieceCloudToLowerWeldPath(inputPoints, seedPoints, CurrentParams());
            if (!result.ok)
            {
                AppendLog(result.error);
                QMessageBox::warning(this, "精测点云处理", result.error);
                return;
            }
        }
        else
        {
            const RobotCalculation::MeasureThenWeldAnalysisResult analysis =
                RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(inputPoints, CurrentParams());
            if (!analysis.ok)
            {
                AppendLog(analysis.error);
                QMessageBox::warning(this, "精测点云处理", analysis.error);
                return;
            }
            result = analysis.filterResult;
        }
        processingSummary = QString("特征点算法：%1")
            .arg(PointCloudProcessingConfig::FeaturePointStrategyDisplayName(CurrentFeaturePointStrategy()));
    }

    if (!result.ok)
    {
        AppendLog(result.error);
        QMessageBox::warning(this, "精测点云处理", result.error);
        return;
    }

    QStringList lines;
    lines << "index x y z source";
    for (const RobotCalculation::LowerWeldFilterPoint& point : result.points)
    {
        lines << RobotCalculation::Vector3IndexedSpaceText(point.index, point.point, point.source);
    }

    if (!RobotDataHelper::SaveTextFileLines(outputPath, lines, &error))
    {
        AppendLog(error);
        QMessageBox::warning(this, "精测点云处理", error);
        return;
    }

    const QString summary = QString(
        "处理完成：\n处理方式=%1\n%2\n输出模式=%3\n拟合裁首尾点数=%4\n拟合段数=%5\n输入点=%6\n下层候选点=%7\n剔除Z突变=%8\n剔除Z连续异常=%9\n按连续段剔除=%10\n输出点=%11\n直接测量=%12\n插值=%13\n外推=%14\n输出文件=%15")
        .arg(CurrentProcessingModeText())
        .arg(processingSummary)
        .arg(processingMode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
            ? QString("新库输出轨迹")
            : PointCloudProcessingConfig::FeaturePointStrategyDisplayName(CurrentFeaturePointStrategy()))
        .arg(processingMode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
            ? 0
            : m_pLineFitTrimSpin->value())
        .arg(result.fitSegmentCount)
        .arg(result.inputPointCount)
        .arg(result.lowerPointCount)
        .arg(result.zJumpRejectedCount)
        .arg(result.zContinuityRejectedCount)
        .arg(result.segmentRejectedCount)
        .arg(result.points.size())
        .arg(result.measuredCount)
        .arg(result.interpolatedCount)
        .arg(result.extendedCount)
        .arg(QDir::toNativeSeparators(outputPath));
    AppendLog(summary);
    QMessageBox::information(this, "精测点云处理", summary);
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

QString LaserWeldFilterDialog::CurrentProcessingModeText() const
{
    return CurrentProcessingMode() == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        ? "点云算法"
        : "特征点算法";
}

double LaserWeldFilterDialog::CurrentOutputStep() const
{
    return CurrentProcessingMode() == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        ? (m_pExternalResampleStepSpin != nullptr ? m_pExternalResampleStepSpin->value() : 2.0)
        : (m_pStepSpin != nullptr ? m_pStepSpin->value() : 2.0);
}

RobotCalculation::LowerWeldFilterParams LaserWeldFilterDialog::CurrentParams() const
{
    RobotCalculation::LowerWeldFilterParams params;
    params.sampleAxis = CurrentProcessingMode() == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        ? static_cast<RobotCalculation::SampleAxis>(m_pCloudAxisCombo->currentData().toInt())
        : static_cast<RobotCalculation::SampleAxis>(m_pAxisCombo->currentData().toInt());
    params.fitMode = static_cast<RobotCalculation::LowerWeldFitMode>(m_pFitModeCombo->currentData().toInt());
    params.geometryStrategy =
        CurrentFeaturePointStrategy() == PointCloudProcessingConfig::FeaturePointStrategy::WorkpieceProjection
            ? RobotCalculation::LowerWeldGeometryStrategy::WorkpieceProjection
            : (CurrentFeaturePointStrategy() == PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys
                ? RobotCalculation::LowerWeldGeometryStrategy::RobustSegmentedKeys
                : (CurrentFeaturePointStrategy() == PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered
                    ? RobotCalculation::LowerWeldGeometryStrategy::SlopeWaveFiltered
                    : RobotCalculation::LowerWeldGeometryStrategy::LegacyGeometry));
    params.zThreshold = m_pZThresholdSpin->value();
    params.zJumpThreshold = m_pZJumpThresholdSpin->value();
    params.zContinuityThreshold = m_pZContinuityThresholdSpin->value();
    params.segmentBreakDistance = m_pSegmentBreakDistanceSpin->value();
    params.keepLongestSegmentOnly = m_pKeepLongestSegmentCheck->isChecked();
    params.sampleStep = m_pStepSpin->value();
    params.searchWindow = m_pWindowSpin->value();
    params.lineFitTrimCount = m_pLineFitTrimSpin->value();
    params.piecewiseFitTolerance = m_pPiecewiseToleranceSpin->value();
    params.piecewiseMinSegmentPoints = m_pPiecewiseMinSegmentSpin->value();
    params.minPointCount = m_pMinPointSpin->value();
    params.smoothRadius = m_pSmoothRadiusSpin->value();
    return params;
}
