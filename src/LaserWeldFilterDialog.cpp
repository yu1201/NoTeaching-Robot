#include "LaserWeldFilterDialog.h"

#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>

#include <cmath>

namespace
{
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
}

LaserWeldFilterDialog::LaserWeldFilterDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("焊道滤波测试");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(920, 680), 0.82, 0.78);

    BuildUi();
    ApplyStyle();
    LoadSettings();
    UpdateSuggestedOutputPath();
}

void LaserWeldFilterDialog::BuildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(18, 18, 18, 18);
    rootLayout->setSpacing(12);

    QLabel* titleLabel = new QLabel("焊道滤波测试");
    titleLabel->setObjectName("TitleLabel");
    QLabel* hintLabel = new QLabel("选择激光点文件后，按参数提取下面焊道并按固定间距重采样输出。默认输出为空格分隔文本。");
    hintLabel->setObjectName("HintLabel");
    rootLayout->addWidget(titleLabel);
    rootLayout->addWidget(hintLabel);

    QGroupBox* fileGroup = new QGroupBox("文件");
    QGridLayout* fileLayout = new QGridLayout(fileGroup);
    m_pInputPathEdit = new QLineEdit();
    m_pOutputPathEdit = new QLineEdit();
    QPushButton* browseInputButton = new QPushButton("选择输入文件");
    QPushButton* browseOutputButton = new QPushButton("选择输出文件");
    fileLayout->addWidget(new QLabel("输入点文件"), 0, 0);
    fileLayout->addWidget(m_pInputPathEdit, 0, 1);
    fileLayout->addWidget(browseInputButton, 0, 2);
    fileLayout->addWidget(new QLabel("输出结果文件"), 1, 0);
    fileLayout->addWidget(m_pOutputPathEdit, 1, 1);
    fileLayout->addWidget(browseOutputButton, 1, 2);
    fileLayout->setColumnStretch(1, 1);
    rootLayout->addWidget(fileGroup);

    QGroupBox* paramGroup = new QGroupBox("参数");
    QGridLayout* paramLayout = new QGridLayout(paramGroup);
    m_pAxisCombo = new QComboBox();
    m_pAxisCombo->addItem("按 Y 方向每隔固定距离采样", static_cast<int>(RobotCalculation::SampleAxis::AxisY));
    m_pAxisCombo->addItem("按 X 方向每隔固定距离采样", static_cast<int>(RobotCalculation::SampleAxis::AxisX));

    m_pFitModeCombo = new QComboBox();
    m_pFitModeCombo->addItem("保持原始轨迹", static_cast<int>(RobotCalculation::LowerWeldFitMode::PreservePath));
    m_pFitModeCombo->addItem("直线拟合输出", static_cast<int>(RobotCalculation::LowerWeldFitMode::LineFit));
    m_pFitModeCombo->addItem("梯形分段拟合输出", static_cast<int>(RobotCalculation::LowerWeldFitMode::TrapezoidFit));
    m_pFitModeCombo->addItem("多段分段直线拟合", static_cast<int>(RobotCalculation::LowerWeldFitMode::PiecewiseLineFit));

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
    m_pZJumpThresholdSpin->setSuffix(" mm");
    m_pZJumpThresholdSpin->setSpecialValueText("关闭");

    m_pZContinuityThresholdSpin = new QDoubleSpinBox();
    m_pZContinuityThresholdSpin->setRange(0.0, 9999.0);
    m_pZContinuityThresholdSpin->setDecimals(3);
    m_pZContinuityThresholdSpin->setSingleStep(0.5);
    m_pZContinuityThresholdSpin->setValue(3.0);
    m_pZContinuityThresholdSpin->setSuffix(" mm");
    m_pZContinuityThresholdSpin->setSpecialValueText("关闭");

    m_pSegmentBreakDistanceSpin = new QDoubleSpinBox();
    m_pSegmentBreakDistanceSpin->setRange(0.0, 9999.0);
    m_pSegmentBreakDistanceSpin->setDecimals(3);
    m_pSegmentBreakDistanceSpin->setSingleStep(1.0);
    m_pSegmentBreakDistanceSpin->setValue(12.0);
    m_pSegmentBreakDistanceSpin->setSuffix(" mm");
    m_pSegmentBreakDistanceSpin->setSpecialValueText("关闭");

    m_pKeepLongestSegmentCheck = new QCheckBox("只保留最长连续段");
    m_pKeepLongestSegmentCheck->setChecked(true);

    m_pStepSpin = new QDoubleSpinBox();
    m_pStepSpin->setRange(0.1, 100.0);
    m_pStepSpin->setDecimals(3);
    m_pStepSpin->setSingleStep(0.5);
    m_pStepSpin->setValue(2.0);
    m_pStepSpin->setSuffix(" mm");

    m_pWindowSpin = new QDoubleSpinBox();
    m_pWindowSpin->setRange(0.0, 100.0);
    m_pWindowSpin->setDecimals(3);
    m_pWindowSpin->setSingleStep(0.5);
    m_pWindowSpin->setValue(8.0);
    m_pWindowSpin->setSuffix(" mm");

    m_pLineFitTrimSpin = new QSpinBox();
    m_pLineFitTrimSpin->setRange(0, 9999);
    m_pLineFitTrimSpin->setValue(0);

    m_pPiecewiseToleranceSpin = new QDoubleSpinBox();
    m_pPiecewiseToleranceSpin->setRange(0.1, 9999.0);
    m_pPiecewiseToleranceSpin->setDecimals(3);
    m_pPiecewiseToleranceSpin->setSingleStep(0.5);
    m_pPiecewiseToleranceSpin->setValue(2.0);
    m_pPiecewiseToleranceSpin->setSuffix(" mm");

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
    paramLayout->addWidget(m_pZThresholdSpin, 1, 1);
    paramLayout->addWidget(new QLabel("Z突变阈值"), 1, 2);
    paramLayout->addWidget(m_pZJumpThresholdSpin, 1, 3);
    paramLayout->addWidget(new QLabel("Z连续阈值"), 2, 0);
    paramLayout->addWidget(m_pZContinuityThresholdSpin, 2, 1);
    paramLayout->addWidget(new QLabel("输出步长"), 2, 2);
    paramLayout->addWidget(m_pStepSpin, 2, 3);
    paramLayout->addWidget(new QLabel("搜索窗口"), 3, 0);
    paramLayout->addWidget(m_pWindowSpin, 3, 1);
    paramLayout->addWidget(new QLabel("拟合裁首尾点数"), 3, 2);
    paramLayout->addWidget(m_pLineFitTrimSpin, 3, 3);
    paramLayout->addWidget(new QLabel("分段拟合容差"), 4, 0);
    paramLayout->addWidget(m_pPiecewiseToleranceSpin, 4, 1);
    paramLayout->addWidget(new QLabel("每段最少点数"), 4, 2);
    paramLayout->addWidget(m_pPiecewiseMinSegmentSpin, 4, 3);
    paramLayout->addWidget(new QLabel("最小点数"), 5, 0);
    paramLayout->addWidget(m_pMinPointSpin, 5, 1);
    paramLayout->addWidget(new QLabel("平滑半径"), 5, 2);
    paramLayout->addWidget(m_pSmoothRadiusSpin, 5, 3);
    paramLayout->addWidget(new QLabel("段间跳变阈值"), 6, 0);
    paramLayout->addWidget(m_pSegmentBreakDistanceSpin, 6, 1);
    paramLayout->addWidget(m_pKeepLongestSegmentCheck, 6, 2, 1, 2);
    paramLayout->setColumnStretch(1, 1);
    paramLayout->setColumnStretch(3, 1);
    rootLayout->addWidget(paramGroup);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch(1);
    m_pRunButton = new QPushButton("开始滤波");
    m_pRunButton->setMinimumSize(180, 42);
    actionLayout->addWidget(m_pRunButton);
    rootLayout->addLayout(actionLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setPlainText("滤波日志：等待选择文件...");
    rootLayout->addWidget(m_pLogText, 1);

    connect(browseInputButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::BrowseInputFile);
    connect(browseOutputButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::BrowseOutputFile);
    connect(m_pRunButton, &QPushButton::clicked, this, &LaserWeldFilterDialog::RunFilter);
    connect(m_pStepSpin, &QDoubleSpinBox::valueChanged, this, [this](double) { UpdateSuggestedOutputPath(); });
}

void LaserWeldFilterDialog::ApplyStyle()
{
    setStyleSheet(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QLabel { color: #B8C7CC; }"
        "QLabel#TitleLabel { font-size: 24px; font-weight: 700; color: #F4FAFA; }"
        "QLabel#HintLabel { color: #8FA7B0; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #9ED8DB; }"
        "QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox { background: #0B1117; color: #F4FAFA; border: 1px solid #2E4656; border-radius: 10px; padding: 6px 10px; min-height: 34px; }"
        "QComboBox QAbstractItemView { background: #12202A; color: #F4FAFA; selection-background-color: #1F4F5C; }"
        "QPushButton { background: #1F3542; color: #F4FAFA; border: 1px solid #3C6475; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2C5364; border-color: #63C7D1; }"
        "QPushButton:pressed { background: #16303A; }"
        "QPlainTextEdit { background: #0B1117; color: #BFE7EA; border: 1px solid #2E4656; border-radius: 10px; padding: 8px; }");
}

void LaserWeldFilterDialog::LoadSettings()
{
    QSettings settings(LaserFilterSettingsPath(), QSettings::IniFormat);
    m_pInputPathEdit->setText(settings.value("Path/Input").toString());
    m_pOutputPathEdit->setText(settings.value("Path/Output").toString());
    m_pAxisCombo->setCurrentIndex(settings.value("Param/Axis", 0).toInt());
    m_pFitModeCombo->setCurrentIndex(settings.value("Param/FitMode", 0).toInt());
    m_pZThresholdSpin->setValue(settings.value("Param/ZThreshold", -230.0).toDouble());
    m_pZJumpThresholdSpin->setValue(settings.value("Param/ZJumpThreshold", 5.0).toDouble());
    m_pZContinuityThresholdSpin->setValue(settings.value("Param/ZContinuityThreshold", 3.0).toDouble());
    m_pSegmentBreakDistanceSpin->setValue(settings.value("Param/SegmentBreakDistance", 12.0).toDouble());
    m_pKeepLongestSegmentCheck->setChecked(settings.value("Param/KeepLongestSegmentOnly", true).toBool());
    m_pStepSpin->setValue(settings.value("Param/Step", 2.0).toDouble());
    m_pWindowSpin->setValue(settings.value("Param/SearchWindow", 8.0).toDouble());
    m_pLineFitTrimSpin->setValue(settings.value("Param/LineFitTrimCount", 0).toInt());
    m_pPiecewiseToleranceSpin->setValue(settings.value("Param/PiecewiseFitTolerance", 2.0).toDouble());
    m_pPiecewiseMinSegmentSpin->setValue(settings.value("Param/PiecewiseMinSegmentPoints", 4).toInt());
    m_pMinPointSpin->setValue(settings.value("Param/MinPointCount", 3).toInt());
    m_pSmoothRadiusSpin->setValue(settings.value("Param/SmoothRadius", 2).toInt());
}

void LaserWeldFilterDialog::SaveSettings() const
{
    QSettings settings(LaserFilterSettingsPath(), QSettings::IniFormat);
    settings.setValue("Path/Input", m_pInputPathEdit->text().trimmed());
    settings.setValue("Path/Output", m_pOutputPathEdit->text().trimmed());
    settings.setValue("Param/Axis", m_pAxisCombo->currentIndex());
    settings.setValue("Param/FitMode", m_pFitModeCombo->currentIndex());
    settings.setValue("Param/ZThreshold", m_pZThresholdSpin->value());
    settings.setValue("Param/ZJumpThreshold", m_pZJumpThresholdSpin->value());
    settings.setValue("Param/ZContinuityThreshold", m_pZContinuityThresholdSpin->value());
    settings.setValue("Param/SegmentBreakDistance", m_pSegmentBreakDistanceSpin->value());
    settings.setValue("Param/KeepLongestSegmentOnly", m_pKeepLongestSegmentCheck->isChecked());
    settings.setValue("Param/Step", m_pStepSpin->value());
    settings.setValue("Param/SearchWindow", m_pWindowSpin->value());
    settings.setValue("Param/LineFitTrimCount", m_pLineFitTrimSpin->value());
    settings.setValue("Param/PiecewiseFitTolerance", m_pPiecewiseToleranceSpin->value());
    settings.setValue("Param/PiecewiseMinSegmentPoints", m_pPiecewiseMinSegmentSpin->value());
    settings.setValue("Param/MinPointCount", m_pMinPointSpin->value());
    settings.setValue("Param/SmoothRadius", m_pSmoothRadiusSpin->value());
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
    m_pOutputPathEdit->setText(QDir::toNativeSeparators(BuildSuggestedOutputPath(path, m_pStepSpin->value())));
    SaveSettings();
}

void LaserWeldFilterDialog::BrowseOutputFile()
{
    const QString suggestedPath = m_pOutputPathEdit->text().trimmed().isEmpty()
        ? BuildSuggestedOutputPath(m_pInputPathEdit->text().trimmed(), m_pStepSpin->value())
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

    const QString suggested = BuildSuggestedOutputPath(m_pInputPathEdit->text().trimmed(), m_pStepSpin->value());
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
        QMessageBox::warning(this, "焊道滤波测试", "请先选择输入点文件。");
        return;
    }

    QString outputPath = m_pOutputPathEdit->text().trimmed();
    if (outputPath.isEmpty())
    {
        outputPath = BuildSuggestedOutputPath(inputPath, m_pStepSpin->value());
        m_pOutputPathEdit->setText(QDir::toNativeSeparators(outputPath));
    }

    QVector<RobotCalculation::IndexedPoint3D> inputPoints;
    QString error;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(inputPath, inputPoints, &error))
    {
        AppendLog(error);
        QMessageBox::warning(this, "焊道滤波测试", error);
        return;
    }

    const RobotCalculation::LowerWeldFilterResult result =
        RobotCalculation::FilterLowerWeldPath(inputPoints, CurrentParams());
    if (!result.ok)
    {
        AppendLog(result.error);
        QMessageBox::warning(this, "焊道滤波测试", result.error);
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
        QMessageBox::warning(this, "焊道滤波测试", error);
        return;
    }

    SaveSettings();

    const QString summary = QString(
        "滤波完成：\n输出模式=%1\n拟合裁首尾点数=%2\n拟合段数=%3\n输入点=%4\n下层候选点=%5\n剔除Z突变=%6\n剔除Z连续异常=%7\n按连续段剔除=%8\n输出点=%9\n直接测量=%10\n插值=%11\n外推=%12\n输出文件=%13")
        .arg(m_pFitModeCombo->currentText())
        .arg(m_pLineFitTrimSpin->value())
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
    QMessageBox::information(this, "焊道滤波测试", summary);
}

void LaserWeldFilterDialog::AppendLog(const QString& text)
{
    if (m_pLogText == nullptr)
    {
        return;
    }
    m_pLogText->appendPlainText(text);
}

RobotCalculation::LowerWeldFilterParams LaserWeldFilterDialog::CurrentParams() const
{
    RobotCalculation::LowerWeldFilterParams params;
    params.sampleAxis = static_cast<RobotCalculation::SampleAxis>(m_pAxisCombo->currentData().toInt());
    params.fitMode = static_cast<RobotCalculation::LowerWeldFitMode>(m_pFitModeCombo->currentData().toInt());
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
