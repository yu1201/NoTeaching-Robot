#include "MeasureThenWeldDialog.h"

#include "ConfigDatabase.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "MeasureThenWeldService.h"
#include "OPini.h"
#include "RobotDriverAdaptor.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "WeldProcessFile.h"
#include "WindowStyleHelper.h"
#include "groove/framebuffer.h"

#include <QCloseEvent>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSignalBlocker>
#include <QStringList>
#include <QTextDocument>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <thread>

namespace
{
constexpr auto RAW_LASER_FILE_NAME = "PreciseLaserPoint.txt";
constexpr auto WORKPIECE_CLOUD_FILE_NAME = "PreciseLaserPoint_WorkpieceCloud.txt";
constexpr auto PRESERVE_PATH_FILE_NAME = "PreciseLaserPoint_PreservePath_2mm.txt";
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";
constexpr int COMP_SEGMENT_COUNT = 4;
constexpr char POSE_GROUP_COUNT_KEY[] = "PoseCompGroupCount";
constexpr char POSE_ACTIVE_GROUP_INDEX_KEY[] = "ActivePoseCompGroupIndex";
constexpr char SEAM_GROUP_COUNT_KEY[] = "SeamCompGroupCount";
constexpr char SEAM_ACTIVE_GROUP_INDEX_KEY[] = "ActiveSeamCompGroupIndex";
constexpr char FINAL_WELD_TRAJECTORY_STEP_KEY[] = "FinalWeldTrajectoryStepMm";

std::string ToUtf8StdString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

double SafeSpeed(double value, double fallback)
{
    return value > 0.0 ? value : fallback;
}

bool HasSkipScanRebuildInput(const QDir& dir)
{
    return QFileInfo::exists(dir.filePath(RAW_LASER_FILE_NAME))
        || QFileInfo::exists(dir.filePath(PRESERVE_PATH_FILE_NAME))
        || QFileInfo::exists(dir.filePath(WORKPIECE_CLOUD_FILE_NAME));
}

bool HasSkipScanExecutableHistory(const QDir& dir)
{
    return QFileInfo::exists(dir.filePath(WELD_POSE_SEAM_COMP_FILE_NAME))
        || QFileInfo::exists(dir.filePath(WELD_POSE_FILE_NAME));
}

QString ResolveLaserPointDirFromSelection(const QString& selectedDir)
{
    const QFileInfo selectedInfo(selectedDir);
    if (!selectedInfo.exists() || !selectedInfo.isDir())
    {
        return QString();
    }

    const QDir dir(selectedInfo.absoluteFilePath());
    if (HasSkipScanRebuildInput(dir) || HasSkipScanExecutableHistory(dir))
    {
        return dir.absolutePath();
    }

    const QString laserDir = dir.filePath("LaserPoint");
    const QDir nestedLaserDir(laserDir);
    if (HasSkipScanRebuildInput(nestedLaserDir) || HasSkipScanExecutableHistory(nestedLaserDir))
    {
        return nestedLaserDir.absolutePath();
    }

    return QString();
}

QString BuildWeldProcessGroupKey(const T_WELD_PARA& weld)
{
    return QString("%1|%2")
        .arg(DecodeRobotMessageText(weld.strWorkPeace))
        .arg(weld.dWeldAngleSize, 0, 'f', 3);
}

int CountWeldProcessLayers(const std::vector<T_WELD_PARA>& weldList, const QString& groupKey)
{
    int layerCount = 0;
    for (const T_WELD_PARA& weld : weldList)
    {
        if (BuildWeldProcessGroupKey(weld) == groupKey)
        {
            ++layerCount;
        }
    }
    return std::max(1, layerCount);
}

QString BuildPoseCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldPoseCompParam.ini").arg(robotName));
}

QString BuildSeamCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldSeamCompParam.ini").arg(robotName));
}

void LoadCompGroupCombo(
    QComboBox* combo,
    const QString& path,
    const QString& allSection,
    const QString& rowCountKey,
    const QString& groupCountKey,
    const QString& activeGroupIndexKey,
    const QString& groupSectionPrefix,
    const QString& defaultNamePrefix)
{
    if (combo == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(combo);
    combo->clear();

    int activeIndex = 0;
    int groupCount = 1;
    const bool hasConfig = ConfigDatabase::HasIniFile(path);
    if (hasConfig)
    {
        COPini ini;
        if (ini.SetFileName(ToUtf8StdString(path)))
        {
            ini.SetSectionName(ToUtf8StdString(allSection));
            ini.ReadString(false, ToUtf8StdString(activeGroupIndexKey), &activeIndex);
            int configuredGroupCount = 0;
            if (ini.ReadString(false, ToUtf8StdString(groupCountKey), &configuredGroupCount) > 0)
            {
                groupCount = std::max(1, configuredGroupCount);
            }
            else
            {
                int rowCount = COMP_SEGMENT_COUNT;
                ini.ReadString(false, ToUtf8StdString(rowCountKey), &rowCount);
                groupCount = std::max(1, (std::max(0, rowCount) + COMP_SEGMENT_COUNT - 1) / COMP_SEGMENT_COUNT);
            }

            activeIndex = std::clamp(activeIndex, 0, groupCount - 1);
            for (int index = 0; index < groupCount; ++index)
            {
                QString groupName = QString("%1%2").arg(defaultNamePrefix).arg(index + 1);
                std::string encodedName;
                ini.SetSectionName(ToUtf8StdString(QString("%1%2").arg(groupSectionPrefix).arg(index)));
                if (ini.ReadString(false, "Name", encodedName) > 0)
                {
                    const QString decodedName = DecodeRobotMessageText(encodedName).trimmed();
                    if (!decodedName.isEmpty())
                    {
                        groupName = decodedName;
                    }
                }
                combo->addItem(QString("%1 / Group%2").arg(groupName).arg(index), index);
            }
        }
    }

    if (combo->count() <= 0)
    {
        combo->addItem(QString("%1%2 / Group0").arg(defaultNamePrefix).arg(1), 0);
        activeIndex = 0;
    }
    combo->setCurrentIndex(std::clamp(activeIndex, 0, combo->count() - 1));
    combo->setEnabled(combo->count() > 0);
}

bool SaveActiveCompGroupIndex(
    const QString& path,
    const QString& allSection,
    const QString& activeGroupIndexKey,
    int activeIndex,
    QString& error)
{
    if (path.isEmpty())
    {
        error = "补偿参数配置库不可用。";
        return false;
    }
    if (!ConfigDatabase::HasIniFile(path) && activeIndex <= 0)
    {
        return true;
    }

    COPini ini;
    if (!ini.SetFileName(false, ToUtf8StdString(path)))
    {
        error = "打开补偿参数失败。";
        return false;
    }
    ini.SetSectionName(ToUtf8StdString(allSection));
    if (!ini.WriteString(ToUtf8StdString(activeGroupIndexKey), std::max(0, activeIndex)))
    {
        error = QStringLiteral("写入补偿组选择失败。");
        return false;
    }
    return true;
}
}

MeasureThenWeldDialog::MeasureThenWeldDialog(ContralUnit* pContralUnit, int unitIndex, StartCameraFunc startCamera, StopCameraFunc stopCamera, CameraCacheFunc cameraCacheForUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_unitIndex(unitIndex)
    , m_pService(new MeasureThenWeldService())
    , m_startCamera(startCamera)
    , m_stopCamera(stopCamera)
    , m_cameraCacheForUnit(cameraCacheForUnit)
    , m_pCameraCache(nullptr)
{
    setWindowTitle("先测后焊");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(760, 500), 0.72, 0.66);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 12px; padding: 12px 18px; font-size: 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:pressed { background: #18303B; }"
        "QPushButton:disabled { background: #27323A; color: #7D8B91; border-color: #364650; }"
        "QCheckBox { color: #BFE8EC; spacing: 8px; font-size: 15px; }"
        "QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #5C7A8B; background: #0B1117; }"
        "QCheckBox::indicator:checked { background: #1E8AA0; border-color: #72D4DD; }"
        "QDoubleSpinBox { background: #05090D; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QComboBox { background: #05090D; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QComboBox::drop-down { width: 34px; border-left: 1px solid #36586A; background: #05090D; }"
        "QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
        "QComboBox QAbstractItemView { background: #081018; color: #F5FAFA; selection-background-color: #2D7D8C; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QProgressBar { background: #081018; color: #F5FAFA; border: 1px solid #2C4653; border-radius: 8px; text-align: center; min-height: 18px; }"
        "QProgressBar::chunk { background: #2D8DA0; border-radius: 7px; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("先测后焊功能");
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("预设参数：读取配置库中的当前测量焊接参数组，并执行安全姿态、扫描起点、扫描终点、收枪姿态；扫描段采集相机三维点，并在扫描后自动执行 PreservePath 拟合、焊道分类、焊接姿态生成和焊道补偿。也可以跳过扫描，直接选历史结果文件夹焊接。");
    hintLabel->setWordWrap(true);
    rootLayout->addWidget(hintLabel);

    QScrollArea* flowScrollArea = new QScrollArea(this);
    flowScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(flowScrollArea);

    QWidget* flowContent = new QWidget(flowScrollArea);
    flowContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    QVBoxLayout* flowLayout = new QVBoxLayout(flowContent);
    flowLayout->setContentsMargins(0, 0, 8, 0);
    flowLayout->setSpacing(12);
    flowLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    QGridLayout* selectorLayout = new QGridLayout();
    selectorLayout->setHorizontalSpacing(14);
    selectorLayout->setVerticalSpacing(8);
    selectorLayout->addWidget(new QLabel("机器人："), 0, 0);
    m_pRobotCombo = new QComboBox();
    m_pRobotCombo->setMinimumWidth(220);
    selectorLayout->addWidget(m_pRobotCombo, 0, 1);
    selectorLayout->addWidget(new QLabel("位置类型："), 0, 2);
    m_pParamGroupCombo = new QComboBox();
    m_pParamGroupCombo->setMinimumWidth(210);
    selectorLayout->addWidget(m_pParamGroupCombo, 0, 3);
    selectorLayout->addWidget(new QLabel("焊接工艺："), 1, 0);
    m_pWeldProcessCombo = new QComboBox();
    m_pWeldProcessCombo->setMinimumWidth(360);
    selectorLayout->addWidget(m_pWeldProcessCombo, 1, 1, 1, 3);
    selectorLayout->addWidget(new QLabel("姿态补偿组："), 2, 0);
    m_pPoseCompGroupCombo = new QComboBox();
    m_pPoseCompGroupCombo->setMinimumWidth(210);
    selectorLayout->addWidget(m_pPoseCompGroupCombo, 2, 1);
    selectorLayout->addWidget(new QLabel("焊道补偿组："), 2, 2);
    m_pSeamCompGroupCombo = new QComboBox();
    m_pSeamCompGroupCombo->setMinimumWidth(210);
    selectorLayout->addWidget(m_pSeamCompGroupCombo, 2, 3);
    selectorLayout->setColumnStretch(4, 1);
    flowLayout->addLayout(selectorLayout);

    QHBoxLayout* modeLayout = new QHBoxLayout();
    QLabel* modeLabel = new QLabel("运行模式：");
    modeLabel->setStyleSheet("font-weight: bold; color: #9ED8DB;");
    m_pActualWeldCheck = new QCheckBox("实际焊接");
    m_pActualWeldCheck->setChecked(true);
    QLabel* modeHintLabel = new QLabel("取消勾选后只空跑轨迹，使用焊接参数里的空跑速度。");
    QLabel* finalStepLabel = new QLabel("最终轨迹点间距：");
    finalStepLabel->setStyleSheet("font-weight: bold; color: #9ED8DB;");
    m_pFinalTrajectoryStepSpin = new QDoubleSpinBox();
    m_pFinalTrajectoryStepSpin->setRange(0.5, 100.0);
    m_pFinalTrajectoryStepSpin->setDecimals(3);
    m_pFinalTrajectoryStepSpin->setSingleStep(1.0);
    m_pFinalTrajectoryStepSpin->setValue(4.0);
    m_pFinalTrajectoryStepSpin->setKeyboardTracking(false);
    m_pFinalTrajectoryStepSpin->setMinimumWidth(110);
    QLabel* finalStepUnitLabel = new QLabel("mm");
    modeLayout->addWidget(modeLabel);
    modeLayout->addWidget(m_pActualWeldCheck);
    modeLayout->addWidget(modeHintLabel);
    modeLayout->addSpacing(24);
    modeLayout->addWidget(finalStepLabel);
    modeLayout->addWidget(m_pFinalTrajectoryStepSpin);
    modeLayout->addWidget(finalStepUnitLabel);
    modeLayout->addStretch();
    flowLayout->addLayout(modeLayout);

    QGridLayout* buttonLayout = new QGridLayout();
    m_pPresetParamBtn = new QPushButton("预设参数");
    m_pSkipScanWeldBtn = new QPushButton("跳过扫描焊接");
    m_pLineScanProcessBtn = new QPushButton("线扫处理");
    m_pPresetParamBtn->setMinimumHeight(64);
    m_pSkipScanWeldBtn->setMinimumHeight(64);
    m_pLineScanProcessBtn->setMinimumHeight(64);
    buttonLayout->addWidget(m_pPresetParamBtn, 0, 0);
    buttonLayout->addWidget(m_pLineScanProcessBtn, 0, 1);
    buttonLayout->addWidget(m_pSkipScanWeldBtn, 1, 0, 1, 2);
    flowLayout->addLayout(buttonLayout);

    QVBoxLayout* progressLayout = new QVBoxLayout();
    progressLayout->setSpacing(6);
    m_pProgressLabel = new QLabel("等待操作");
    m_pProgressLabel->setStyleSheet("color: #9ED8DB; font-weight: 600;");
    m_pProgressBar = new QProgressBar();
    m_pProgressBar->setRange(0, 100);
    m_pProgressBar->setValue(0);
    progressLayout->addWidget(m_pProgressLabel);
    progressLayout->addWidget(m_pProgressBar);
    flowLayout->addLayout(progressLayout);
    m_pProgressLabel->hide();
    m_pProgressBar->hide();
    flowLayout->addStretch(1);
    flowScrollArea->setWidget(flowContent);
    rootLayout->addWidget(flowScrollArea, 1);

    m_pProgressAnimationTimer = new QTimer(this);
    m_pProgressAnimationTimer->setInterval(300);
    connect(m_pProgressAnimationTimer, &QTimer::timeout, this, &MeasureThenWeldDialog::UpdateProgressAnimation);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1600);
    m_pLogText->setPlainText("流程日志：等待操作...");
    m_pLogText->setMinimumHeight(150);
    m_pLogText->setMaximumHeight(240);
    rootLayout->addWidget(m_pLogText);

    connect(m_pPresetParamBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunPresetParamFlow);
    connect(m_pSkipScanWeldBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunSkipScanWeldFlow);
    connect(m_pLineScanProcessBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunLineScanProcess);
    connect(m_pActualWeldCheck, &QCheckBox::toggled, this, &MeasureThenWeldDialog::SaveWeldModeToParam);
    connect(m_pFinalTrajectoryStepSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MeasureThenWeldDialog::SaveFinalTrajectoryStepToParam);
    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MeasureThenWeldDialog::OnRobotChanged);
    connect(m_pParamGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MeasureThenWeldDialog::OnParamGroupChanged);
    connect(m_pWeldProcessCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MeasureThenWeldDialog::OnWeldProcessChanged);
    connect(m_pPoseCompGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_bLoadingSelectors || m_bRunning)
            {
                return;
            }
            QString error;
            if (!SaveCurrentCompGroupSelections(error))
            {
                AppendLog("切换姿态补偿组失败：" + error);
                return;
            }
            AppendLog(QString("当前姿态补偿组已切换为：%1").arg(m_pPoseCompGroupCombo != nullptr ? m_pPoseCompGroupCombo->currentText() : QString()));
        });
    connect(m_pSeamCompGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_bLoadingSelectors || m_bRunning)
            {
                return;
            }
            QString error;
            if (!SaveCurrentCompGroupSelections(error))
            {
                AppendLog("切换焊道补偿组失败：" + error);
                return;
            }
            AppendLog(QString("当前焊道补偿组已切换为：%1").arg(m_pSeamCompGroupCombo != nullptr ? m_pSeamCompGroupCombo->currentText() : QString()));
        });
    LoadRobotList();
}

bool MeasureThenWeldDialog::IsRunning() const
{
    return m_bRunning;
}

void MeasureThenWeldDialog::ReloadSelectors()
{
    if (m_bRunning)
    {
        return;
    }
    LoadRobotList();
}

void MeasureThenWeldDialog::closeEvent(QCloseEvent* event)
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行，请等待当前流程结束后再关闭。");
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

void MeasureThenWeldDialog::LoadRobotList()
{
    if (m_pRobotCombo == nullptr)
    {
        return;
    }

    m_bLoadingSelectors = true;
    m_pRobotCombo->clear();

    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    int selectedIndex = -1;
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        const int row = m_pRobotCombo->count();
        m_pRobotCombo->addItem(info.displayName, info.unitIndex);
        m_pRobotCombo->setItemData(row, info.robotName, Qt::UserRole + 1);
        if (info.unitIndex == m_unitIndex)
        {
            selectedIndex = row;
        }
    }

    if (selectedIndex < 0 && m_pRobotCombo->count() > 0)
    {
        selectedIndex = 0;
    }
    if (selectedIndex >= 0)
    {
        m_pRobotCombo->setCurrentIndex(selectedIndex);
        m_unitIndex = m_pRobotCombo->currentData().toInt();
    }

    m_bLoadingSelectors = false;
    LoadParamGroups();
    LoadWeldProcessList();
    LoadCompGroupLists();
    RefreshWeldModeFromParam();
}

void MeasureThenWeldDialog::LoadParamGroups()
{
    if (m_pParamGroupCombo == nullptr)
    {
        return;
    }

    m_bLoadingSelectors = true;
    m_pParamGroupCombo->clear();

    const QString robotName = CurrentRobotName();
    QString error;
    if (robotName.isEmpty() || !RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &error))
    {
        if (!error.isEmpty())
        {
            AppendLog("读取位置类型失败：" + error);
        }
        m_bLoadingSelectors = false;
        return;
    }

    COPini ini;
    const QString path = RobotDataHelper::MeasureWeldParamPath(robotName);
    if (!ini.SetFileName(path.toLocal8Bit().constData()))
    {
        AppendLog("读取位置类型失败：打开参数数据失败：" + path);
        m_bLoadingSelectors = false;
        return;
    }

    int groupCount = 1;
    int useNo = 0;
    ini.SetSectionName("MeasureWeldGroups");
    ini.ReadString(false, "GroupCount", &groupCount);
    ini.ReadString(false, "UseGroupNo", &useNo);
    groupCount = std::max(1, groupCount);
    useNo = std::clamp(useNo, 0, groupCount - 1);
    for (int index = 0; index < groupCount; ++index)
    {
        std::string groupName;
        ini.ReadString(false, QString("Group%1Name").arg(index).toStdString(), groupName);
        const QString displayName = groupName.empty()
            ? QString("参数组%1").arg(index + 1)
            : DecodeRobotMessageText(groupName);
        m_pParamGroupCombo->addItem(QString("%1 / Group%2").arg(displayName).arg(index), index);
    }
    m_pParamGroupCombo->setCurrentIndex(useNo);
    m_bLoadingSelectors = false;
}

void MeasureThenWeldDialog::LoadWeldProcessList()
{
    if (m_pWeldProcessCombo == nullptr)
    {
        return;
    }

    m_bLoadingSelectors = true;
    m_pWeldProcessCombo->clear();
    m_pWeldProcessCombo->setEnabled(false);

    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        m_bLoadingSelectors = false;
        return;
    }

    WeldProcessFile processFile(*m_pContralUnit, m_unitIndex);
    if (!processFile.Init())
    {
        m_pWeldProcessCombo->addItem("请先在工艺界面重新创建工艺", -1);
        m_bLoadingSelectors = false;
        return;
    }
    const std::vector<T_WELD_PARA>& weldList = processFile.GetWeldParaList();
    QString selectedKey;
    if (!weldList.empty())
    {
        const int useIndex = std::clamp(processFile.GetUseWeldParaNo(), 0, static_cast<int>(weldList.size()) - 1);
        selectedKey = BuildWeldProcessGroupKey(weldList[useIndex]);
    }
    int selectedComboIndex = 0;
    QStringList seenKeys;
    for (int index = 0; index < static_cast<int>(weldList.size()); ++index)
    {
        const T_WELD_PARA& weld = weldList[index];
        const QString groupKey = BuildWeldProcessGroupKey(weld);
        if (seenKeys.contains(groupKey))
        {
            continue;
        }
        seenKeys.append(groupKey);
        const QString workpiece = DecodeRobotMessageText(weld.strWorkPeace);
        const int layerCount = CountWeldProcessLayers(weldList, groupKey);
        const QString layerText = layerCount > 1 ? QString(" | 共%1层").arg(layerCount) : QString();
        const QString displayName = QString("%1. %2 | 焊脚%3%4")
            .arg(m_pWeldProcessCombo->count() + 1)
            .arg(workpiece.isEmpty() ? QStringLiteral("未命名工艺") : workpiece)
            .arg(weld.dWeldAngleSize, 0, 'f', 1)
            .arg(layerText);
        m_pWeldProcessCombo->addItem(displayName, index);
        if (groupKey == selectedKey)
        {
            selectedComboIndex = m_pWeldProcessCombo->count() - 1;
        }
    }

    if (m_pWeldProcessCombo->count() > 0)
    {
        m_pWeldProcessCombo->setEnabled(true);
        m_pWeldProcessCombo->setCurrentIndex(std::clamp(selectedComboIndex, 0, m_pWeldProcessCombo->count() - 1));
    }
    else
    {
        const QString errorText = DecodeRobotMessageText(processFile.GetLastError());
        m_pWeldProcessCombo->addItem(errorText.isEmpty() ? QStringLiteral("未读取到焊接工艺") : errorText, -1);
    }

    m_bLoadingSelectors = false;
}

void MeasureThenWeldDialog::LoadCompGroupLists()
{
    const QString robotName = CurrentRobotName();
    m_bLoadingSelectors = true;
    LoadCompGroupCombo(
        m_pPoseCompGroupCombo,
        BuildPoseCompParamPath(robotName),
        "ALLWeldPoseComp",
        "PoseCompCount",
        POSE_GROUP_COUNT_KEY,
        POSE_ACTIVE_GROUP_INDEX_KEY,
        "WeldPoseCompGroup",
        "姿态补偿组");
    LoadCompGroupCombo(
        m_pSeamCompGroupCombo,
        BuildSeamCompParamPath(robotName),
        "ALLWeldSeamComp",
        "SeamCompCount",
        SEAM_GROUP_COUNT_KEY,
        SEAM_ACTIVE_GROUP_INDEX_KEY,
        "WeldSeamCompGroup",
        "焊道补偿组");
    m_bLoadingSelectors = false;
}

void MeasureThenWeldDialog::OnRobotChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoadingSelectors || m_pRobotCombo == nullptr)
    {
        return;
    }

    m_unitIndex = m_pRobotCombo->currentData().toInt();
    m_pCameraCache = ResolveCameraCacheForUnit(m_unitIndex);
    LoadParamGroups();
    LoadWeldProcessList();
    LoadCompGroupLists();
    RefreshWeldModeFromParam();
    AppendLog(QString("当前机器人已切换为：%1").arg(m_pRobotCombo->currentText()));
}

void MeasureThenWeldDialog::OnParamGroupChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoadingSelectors || m_bRunning)
    {
        return;
    }

    QString error;
    if (!SaveCurrentParamGroupSelection(error))
    {
        AppendLog("切换位置类型失败：" + error);
        return;
    }
    RefreshWeldModeFromParam();
    AppendLog(QString("当前位置类型已切换为：%1").arg(m_pParamGroupCombo != nullptr ? m_pParamGroupCombo->currentText() : QString()));
}

void MeasureThenWeldDialog::OnWeldProcessChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoadingSelectors || m_bRunning)
    {
        return;
    }

    QString error;
    if (!SaveCurrentWeldProcessSelection(error))
    {
        AppendLog("切换焊接工艺失败：" + error);
        return;
    }
    AppendLog(QString("当前焊接工艺已切换为：%1").arg(m_pWeldProcessCombo != nullptr ? m_pWeldProcessCombo->currentText() : QString()));
}

QString MeasureThenWeldDialog::CurrentRobotName() const
{
    if (m_pRobotCombo != nullptr && m_pRobotCombo->currentIndex() >= 0)
    {
        const QString robotName = m_pRobotCombo->currentData(Qt::UserRole + 1).toString();
        if (!robotName.trimmed().isEmpty())
        {
            return robotName.trimmed();
        }
    }

    RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, m_unitIndex);
    if (driver != nullptr && !driver->m_sRobotName.empty())
    {
        return QString::fromStdString(driver->m_sRobotName);
    }
    return QString();
}

int MeasureThenWeldDialog::CurrentParamGroupIndex() const
{
    if (m_pParamGroupCombo != nullptr && m_pParamGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pParamGroupCombo->currentData().toInt());
    }
    return 0;
}

int MeasureThenWeldDialog::CurrentPoseCompGroupIndex() const
{
    if (m_pPoseCompGroupCombo != nullptr && m_pPoseCompGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pPoseCompGroupCombo->currentData().toInt());
    }
    return 0;
}

int MeasureThenWeldDialog::CurrentSeamCompGroupIndex() const
{
    if (m_pSeamCompGroupCombo != nullptr && m_pSeamCompGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pSeamCompGroupCombo->currentData().toInt());
    }
    return 0;
}

bool MeasureThenWeldDialog::SaveCurrentParamGroupSelection(QString& error) const
{
    const QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WriteParamValue(
        RobotDataHelper::MeasureWeldParamPath(robotName),
        "MeasureWeldGroups",
        "UseGroupNo",
        QString::number(CurrentParamGroupIndex()),
        &error);
}

bool MeasureThenWeldDialog::SaveCurrentWeldProcessSelection(QString& error) const
{
    if (m_pWeldProcessCombo == nullptr || m_pWeldProcessCombo->currentIndex() < 0)
    {
        error = "未选择焊接工艺。";
        return false;
    }
    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        error = "未选择有效机器人。";
        return false;
    }

    const int weldProcessIndex = m_pWeldProcessCombo->currentData().toInt();
    if (weldProcessIndex < 0)
    {
        error = "当前焊接工艺无效。";
        return false;
    }

    WeldProcessFile processFile(*m_pContralUnit, m_unitIndex);
    if (!processFile.Init())
    {
        error = DecodeRobotMessageText(processFile.GetLastError());
        if (error.isEmpty())
        {
            error = "读取焊接工艺失败，请先在工艺界面重新创建工艺。";
        }
        return false;
    }
    if (!processFile.UpdateUseWeldParaNo(weldProcessIndex))
    {
        error = DecodeRobotMessageText(processFile.GetLastError());
        if (error.isEmpty())
        {
            error = "写入焊接工艺选择失败。";
        }
        return false;
    }
    return true;
}

bool MeasureThenWeldDialog::SaveCurrentCompGroupSelections(QString& error) const
{
    const QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }

    if (!SaveActiveCompGroupIndex(
        BuildPoseCompParamPath(robotName),
        "ALLWeldPoseComp",
        POSE_ACTIVE_GROUP_INDEX_KEY,
        CurrentPoseCompGroupIndex(),
        error))
    {
        return false;
    }

    if (!SaveActiveCompGroupIndex(
        BuildSeamCompParamPath(robotName),
        "ALLWeldSeamComp",
        SEAM_ACTIVE_GROUP_INDEX_KEY,
        CurrentSeamCompGroupIndex(),
        error))
    {
        return false;
    }
    return true;
}

CameraFrameCache* MeasureThenWeldDialog::ResolveCameraCacheForUnit(int unitIndex)
{
    if (m_cameraCacheForUnit)
    {
        return m_cameraCacheForUnit(unitIndex);
    }
    return m_pCameraCache;
}

RobotDriverAdaptor* MeasureThenWeldDialog::GetRobotDriver()
{
    if (m_pRobotCombo != nullptr && m_pRobotCombo->currentIndex() >= 0)
    {
        m_unitIndex = m_pRobotCombo->currentData().toInt();
    }

    if (m_pContralUnit == nullptr || m_unitIndex < 0 || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, "先测后焊", "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, "先测后焊", "当前控制单元未创建驱动。");
        return nullptr;
    }

    return pRobotDriverAdaptor;
}

bool MeasureThenWeldDialog::LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error)
{
    if (!SaveCurrentParamGroupSelection(error))
    {
        return false;
    }
    if (m_pWeldProcessCombo != nullptr && m_pWeldProcessCombo->isEnabled() && !SaveCurrentWeldProcessSelection(error))
    {
        return false;
    }
    if (!SaveCurrentCompGroupSelections(error))
    {
        return false;
    }
    return m_pService != nullptr && m_pService->LoadPresetParam(pRobotDriver, param, error);
}

bool MeasureThenWeldDialog::ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadPulse(ini, prefix, pulse, error);
}

bool MeasureThenWeldDialog::ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadCoors(ini, prefix, coors, error);
}

bool MeasureThenWeldDialog::ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadPulseList(ini, countKey, prefix, pulses, error);
}

bool MeasureThenWeldDialog::MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MovePulseAndWait(
        pRobotDriver,
        pulse,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MovePulseListAndWait(
        pRobotDriver,
        pulses,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MoveCoorsAndWait(
        pRobotDriver,
        coors,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MoveScanStartSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed)
{
    return m_pService != nullptr && m_pService->MoveScanStartSafeAndWait(
        pRobotDriver,
        param,
        speed,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); },
        [this](const QString& title, const QString& detail) { return ShowCheckpointDialog(title, detail); });
}

bool MeasureThenWeldDialog::MoveScanEndSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed)
{
    return m_pService != nullptr && m_pService->MoveScanEndSafeAndWait(
        pRobotDriver,
        param,
        speed,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::ScanMoveAndCollect(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath)
{
    return m_pService != nullptr && m_pService->ScanMoveAndCollect(
        pRobotDriver,
        param,
        savedPath,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text)
        {
            SetFlowStep(text);
            if (text.contains("扫描运动中"))
            {
                SetProgressBusy(40, text);
            }
            else if (text.contains("扫描完成"))
            {
                SetProgressBusy(62, text);
            }
            else if (text.contains("重新计算") || text.contains("特征分析") || text.contains("焊接姿态"))
            {
                SetProgressBusy(68, text);
            }
        },
        m_pCameraCache);
}

QString MeasureThenWeldDialog::BuildResultDir(const std::string& robotName) const
{
    return m_pService != nullptr ? m_pService->BuildResultDir(robotName) : QString();
}

bool MeasureThenWeldDialog::SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const
{
    return m_pService != nullptr && m_pService->SaveTextLines(filePath, lines, error);
}

bool MeasureThenWeldDialog::ConfirmContinue(const QString& actionName)
{
    SetFlowStep(QString("等待确认：%1").arg(actionName));

    if (QThread::currentThread() != thread())
    {
        // 流程在线程里跑，确认框必须切回 UI 线程并阻塞等待用户选择。
        bool confirmed = false;
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, actionName, &confirmed]()
            {
                if (self == nullptr)
                {
                    confirmed = false;
                    return;
                }
                confirmed = self->ConfirmContinue(actionName);
            }, Qt::BlockingQueuedConnection);
        return confirmed;
    }

    const QMessageBox::StandardButton ret = QMessageBox::question(
        this,
        "先测后焊确认",
        QString("即将执行：%1\n\n请确认机器人周围安全，是否继续移动？\n选择“取消”将退出当前流程。").arg(actionName),
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Cancel);
    const bool confirmed = (ret == QMessageBox::Ok);
    AppendLog(QString("%1：%2").arg(actionName).arg(confirmed ? "已确认" : "已取消"));
    return confirmed;
}

bool MeasureThenWeldDialog::ShowCheckpointDialog(const QString& title, const QString& detail)
{
    SetFlowStep(QString("关键节点确认：%1").arg(title));

    if (QThread::currentThread() != thread())
    {
        bool confirmed = false;
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, title, detail, &confirmed]()
            {
                if (self == nullptr)
                {
                    confirmed = false;
                    return;
                }
                confirmed = self->ShowCheckpointDialog(title, detail);
            }, Qt::BlockingQueuedConnection);
        return confirmed;
    }

    const QMessageBox::StandardButton ret = QMessageBox::question(
        this,
        title,
        detail + "\n\n选择“确定”继续，选择“取消”终止当前流程。",
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Ok);
    const bool confirmed = (ret == QMessageBox::Ok);
    AppendLog(QString("关键节点[%1]：%2").arg(title).arg(confirmed ? "已确认继续" : "已取消流程"));
    return confirmed;
}

void MeasureThenWeldDialog::RunPresetParamFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行。");
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pRobotDriver, param, error))
    {
        QMessageBox::warning(this, "预设参数", error);
        return;
    }
    param.bDoActualWeld = IsActualWeldModeChecked();
    const int unitIndexForRun = m_unitIndex;
    m_pCameraCache = ResolveCameraCacheForUnit(unitIndexForRun);
    SetRunning(true);
    ResetProgress("读取预设参数完成，准备启动相机");
    SetProgress(5, "读取预设参数完成");
    SetFlowStep("读取预设参数完成，准备启动相机");
    AppendLog(QString("已读取参数：%1，位置类型=%2 [%3]")
        .arg(QString::fromStdString(param.sIniFilePath))
        .arg(param.sParamGroupName)
        .arg(QString::fromStdString(param.sSectionName)));
    AppendLog(QString("焊接执行模式：%1，焊接速度=%2 mm/min，空跑速度=%3 mm/min，安全位速度=%4 mm/min")
        .arg(param.bDoActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑"))
        .arg(param.dWeldSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dDryRunSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dWeldSafeMoveSpeedMmPerMin, 0, 'f', 3));

    // 机器人运动和扫描采集放到后台线程，避免 UI 被 CheckRobotDone 和文件保存卡住。
    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self, pRobotDriver, param, unitIndexForRun]()
        {
            bool ok = true;
            QString message;
            QString cameraIP;
            QString savedPath;
            QString executeSummary;

            if (self != nullptr)
            {
                self->SetProgressBusy(8, "正在启动扫描相机");
            }
            QMetaObject::invokeMethod(qApp, [self, &cameraIP, &ok, unitIndexForRun]()
                {
                    // 相机 UDP 线程由主界面统一管理，这里通过回调启动。
                    if (self == nullptr)
                    {
                        ok = false;
                        return;
                    }
                    ok = self->m_startCamera ? self->m_startCamera(unitIndexForRun, cameraIP) : false;
                }, Qt::BlockingQueuedConnection);

            if (!ok)
            {
                message = "相机启动失败，流程中止。";
            }
            else
            {
                QMetaObject::invokeMethod(qApp, [self, cameraIP]()
                    {
                        if (self != nullptr)
                        {
                            self->SetFlowStep(QString("相机接收已启动：%1，准备扫描下枪安全位置").arg(cameraIP));
                            self->SetProgress(12, "相机接收已启动");
                            self->AppendLog(QString("相机接收已启动：%1").arg(cameraIP));
                        }
                    }, Qt::QueuedConnection);

                ok = self != nullptr && self->ConfirmContinue("扫描下枪安全位置");
                if (ok)
                {
                    self->SetFlowStep("准备移动到扫描下枪安全位置");
                    self->SetProgressBusy(18, "移动到扫描下枪安全位置");
                    // 1. 扫描前按起点位姿和配置推算安全位置，避免直接切入扫描起点。
                    ok = self != nullptr && self->MoveScanStartSafeAndWait(pRobotDriver, param, SafeSpeed(param.dRunSpeed, 1.0));
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("移动到扫描起点");
                }
                if (ok)
                {
                    self->SetFlowStep("准备移动到扫描起点");
                    self->SetProgressBusy(28, "移动到扫描起点");
                    // 2. 到扫描起点使用直线运动，保持扫描段的空间姿态连续。
                    ok = self != nullptr && self->MoveCoorsAndWait(pRobotDriver, param.tStartPos, SafeSpeed(param.dRunSpeed, 1.0), "扫描起点");
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("扫描终点并采集相机点");
                }
                if (ok)
                {
                    self->SetFlowStep("准备扫描终点并采集相机点");
                    self->SetProgressBusy(40, "扫描运动中，正在采集点云");
                    // 3. 从扫描起点运动到扫描终点，同时按 10ms 周期读取相机点。
                    ok = self != nullptr && self->ScanMoveAndCollect(pRobotDriver, param, savedPath);
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("扫描收枪安全位置");
                }
                if (ok)
                {
                    self->SetFlowStep("准备移动到扫描收枪安全位置");
                    self->SetProgressBusy(72, "移动到扫描收枪安全位置");
                    // 4. 扫描结束后按终点位姿和同一配置推算安全位置。
                    ok = self != nullptr && self->MoveScanEndSafeAndWait(pRobotDriver, param, SafeSpeed(param.dRunSpeed, 1.0));
                }
                if (ok)
                {
                    const QFileInfo weldPoseFileInfo(savedPath);
                    if (!weldPoseFileInfo.isFile())
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("未生成可下发的焊接姿态文件，当前结果=%1").arg(savedPath));
                        }
                        ok = false;
                        message = "预设参数流程已完成测量，但未生成可下发的焊接姿态文件。";
                    }
                }
                if (ok)
                {
                    ok = self != nullptr && self->ShowCheckpointDialog(
                        "扫描完成",
                        QString("扫描、拟合、焊道分类和焊接姿态生成已完成。\n焊接姿态文件：%1").arg(savedPath));
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("移动到焊接下枪安全位置并执行焊接轨迹");
                }
                if (ok)
                {
                    QString executeError;
                    T_ROBOT_COORS startSafeCoors;
                    T_ROBOT_COORS endSafeCoors;
                    if (self != nullptr)
                    {
                        self->SetFlowStep("准备执行焊接轨迹");
                        self->SetProgressBusy(84, "执行焊接轨迹");
                        self->AppendLog(QString("开始执行焊接轨迹：%1").arg(savedPath));
                    }

                    ok = self != nullptr
                        && self->m_pService != nullptr
                        && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                            pRobotDriver,
                            savedPath,
                            param,
                            executeSummary,
                            executeError,
                            &startSafeCoors,
                            &endSafeCoors,
                            [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                            [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                            [self](const QString& title, const QString& detail) -> bool
                            {
                                return self != nullptr && self->ShowCheckpointDialog(title, detail);
                            });
                    if (ok)
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("焊接轨迹执行完成：%1").arg(executeSummary));
                        }
                        message = QString("预设参数流程完成。\n结果位置：%1\n执行结果：%2\n下枪安全位置：%3\n收枪安全位置：%4")
                            .arg(savedPath)
                            .arg(executeSummary)
                            .arg(QString("%1, %2, %3")
                                .arg(startSafeCoors.dX, 0, 'f', 3)
                                .arg(startSafeCoors.dY, 0, 'f', 3)
                                .arg(startSafeCoors.dZ, 0, 'f', 3))
                            .arg(QString("%1, %2, %3")
                                .arg(endSafeCoors.dX, 0, 'f', 3)
                                .arg(endSafeCoors.dY, 0, 'f', 3)
                                .arg(endSafeCoors.dZ, 0, 'f', 3));
                    }
                    else
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("焊接轨迹执行失败：%1").arg(executeError));
                        }
                        message = QString("预设参数流程已完成测量，但焊接轨迹执行失败。\n%1").arg(executeError);
                    }
                }
                else if (message.isEmpty())
                {
                    message = "预设参数流程失败，请查看流程日志。";
                }
            }

            QMetaObject::invokeMethod(qApp, [self, message, ok]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    if (self->m_stopCamera)
                    {
                        // 流程正常完成、失败或用户取消，都会释放流程侧相机占用。
                        self->m_stopCamera();
                    }
                    self->SetFlowStep(ok ? "流程完成" : "流程失败，请查看流程日志");
                    self->FinishProgress(ok, ok ? QStringLiteral("流程完成") : QStringLiteral("流程失败，请查看流程日志"));
                    self->AppendLog(ok ? "流程完成。" : "流程失败。");
                    self->SetRunning(false);
                    if (ok)
                    {
                        QMessageBox::information(self, "预设参数", message);
                    }
                    else
                    {
                        QMessageBox::warning(self, "预设参数", message);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunSkipScanWeldFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行。");
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pRobotDriver, param, error))
    {
        QMessageBox::warning(this, "跳过扫描焊接", error);
        return;
    }
    param.bDoActualWeld = IsActualWeldModeChecked();

    const QString defaultDir = RobotDataHelper::BuildProjectPath(
        QString("Result/%1").arg(QString::fromStdString(param.sRobotName)));
    const QString selectedDir = QFileDialog::getExistingDirectory(
        this,
        "选择先测后焊结果文件夹",
        defaultDir,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selectedDir.isEmpty())
    {
        return;
    }

    const QString laserDir = ResolveLaserPointDirFromSelection(selectedDir);
    if (laserDir.isEmpty())
    {
        QMessageBox::warning(
            this,
            "跳过扫描焊接",
            QString("在所选目录中未找到原始激光点、PreservePath、局部完整点云、历史姿态文件或历史补偿文件。\n请选择结果目录本身，或其下的 LaserPoint 目录。"));
        return;
    }

    const QString preservePath = QDir(laserDir).filePath(PRESERVE_PATH_FILE_NAME);
    const QString poseFilePath = QDir(laserDir).filePath(WELD_POSE_FILE_NAME);
    const QString seamCompPath = QDir(laserDir).filePath(WELD_POSE_SEAM_COMP_FILE_NAME);
    const bool canRebuildFromLaser = HasSkipScanRebuildInput(QDir(laserDir));
    const bool hasExistingPoseFile = QFileInfo::exists(poseFilePath);
    const bool hasExistingSeamCompFile = QFileInfo::exists(seamCompPath);

    if (!canRebuildFromLaser && !hasExistingPoseFile && !hasExistingSeamCompFile)
    {
        QMessageBox::warning(
            this,
            "跳过扫描焊接",
            QString("所选目录缺少可重建或可执行的历史文件。\n需要至少包含 %1、%2、%3、%4 或 %5。")
                .arg(RAW_LASER_FILE_NAME)
                .arg(PRESERVE_PATH_FILE_NAME)
                .arg(WORKPIECE_CLOUD_FILE_NAME)
                .arg(WELD_POSE_FILE_NAME)
                .arg(WELD_POSE_SEAM_COMP_FILE_NAME));
        return;
    }

    SetRunning(true);
    ResetProgress("已选择历史结果，准备重新计算");
    SetProgress(8, "已选择历史结果");
    SetFlowStep("已选择历史结果，准备重新计算三份焊接文件");
    AppendLog(QString("跳过扫描模式：结果目录=%1").arg(selectedDir));
    AppendLog(QString("LaserPoint目录=%1").arg(laserDir));
    AppendLog(QString("PreservePath文件将输出到=%1").arg(preservePath));
    AppendLog(QString("姿态文件=%1").arg(poseFilePath));
    AppendLog(QString("补偿后文件将输出到=%1").arg(seamCompPath));
    AppendLog(QString("焊接执行模式：%1，焊接速度=%2 mm/min，空跑速度=%3 mm/min，安全位速度=%4 mm/min，示教姿态=%5，RZ增益=%6 deg，爬坡RZ夹紧=[%7, %8] deg")
        .arg(param.bDoActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑"))
        .arg(param.dWeldSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dDryRunSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dWeldSafeMoveSpeedMmPerMin, 0, 'f', 3)
        .arg(param.bUseTaughtWeldPose ? QStringLiteral("启用") : QStringLiteral("未启用"))
        .arg(param.dWeldRzGainDeg, 0, 'f', 3)
        .arg(param.dSlopeRzMinDeg, 0, 'f', 3)
        .arg(param.dSlopeRzMaxDeg, 0, 'f', 3));

    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self,
                 pRobotDriver,
                 param,
                 selectedDir,
                 laserDir,
                 preservePath = QString(preservePath),
                 poseFilePath = QString(poseFilePath),
                 seamCompPath = QString(seamCompPath),
                 canRebuildFromLaser,
                 hasExistingPoseFile,
                 hasExistingSeamCompFile]() mutable
        {
            bool ok = true;
            QString message;
            QString processError;
            QString rebuildSummary;
            QString executeSummary;
            QString rebuildPreservePath = preservePath;
            QString rebuildPoseFilePath = poseFilePath;
            QString rebuildSeamCompPath = seamCompPath;
            T_ROBOT_COORS startSafeCoors;
            T_ROBOT_COORS endSafeCoors;

            if (self != nullptr)
            {
                ok = self->ShowCheckpointDialog(
                    "跳过扫描确认",
                    QString("已选择结果目录：%1\n将按当前参数重新计算：\n%2\n%3\n%4")
                        .arg(selectedDir)
                        .arg(preservePath)
                        .arg(poseFilePath)
                        .arg(seamCompPath));
            }

            if (ok && canRebuildFromLaser)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("正在从历史LaserPoint重新计算三份焊接文件");
                    self->SetProgressBusy(28, "正在从历史LaserPoint重新计算焊接文件");
                }
                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->RebuildWeldFilesFromLaserDir(
                        param,
                        laserDir,
                        rebuildPreservePath,
                        rebuildPoseFilePath,
                        rebuildSeamCompPath,
                        rebuildSummary,
                        processError,
                        [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                        [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); });
                if (self != nullptr)
                {
                    if (ok)
                    {
                        self->AppendLog(QString("跳过扫描重建完成：%1").arg(rebuildSummary));
                        preservePath = rebuildPreservePath;
                        poseFilePath = rebuildPoseFilePath;
                        seamCompPath = rebuildSeamCompPath;
                    }
                    else
                    {
                        self->AppendLog(QString("跳过扫描重建文件失败：%1").arg(processError));
                    }
                }
            }
            else if (ok && hasExistingSeamCompFile)
            {
                rebuildSummary = QString("未找到原始激光点/PreservePath/局部完整点云，跳过重建，直接使用已有补偿文件：%1")
                    .arg(seamCompPath);
                if (self != nullptr)
                {
                    self->SetFlowStep("使用已有历史补偿文件，准备执行焊接轨迹");
                    self->SetProgress(48, "使用已有补偿文件");
                    self->AppendLog(rebuildSummary);
                }
            }
            else if (ok && hasExistingPoseFile)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("正在从历史姿态文件重新生成焊道补偿文件");
                    self->SetProgressBusy(42, "正在生成焊道补偿文件");
                    self->AppendLog(QString("未找到原始激光点/PreservePath/局部完整点云，改用历史姿态文件重新生成补偿：%1").arg(poseFilePath));
                }

                QString seamCompSummary;
                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->ApplyWeldSeamCompToPoseFile(
                        QString::fromStdString(param.sRobotName),
                        poseFilePath,
                        seamCompPath,
                        seamCompSummary,
                        processError);
                if (ok)
                {
                    rebuildSummary = QString("历史姿态文件补偿完成：Pose=%1；SeamComp=%2；%3")
                        .arg(poseFilePath, seamCompPath, seamCompSummary);
                    if (self != nullptr)
                    {
                        self->AppendLog(rebuildSummary);
                    }
                }
                else if (self != nullptr)
                {
                    self->AppendLog(QString("历史姿态文件补偿失败：%1").arg(processError));
                }
            }

            if (ok)
            {
                ok = self != nullptr && self->ShowCheckpointDialog(
                    "补偿完成",
                    QString("PreservePath：%1\n姿态文件：%2\n补偿文件：%3\n%4")
                        .arg(preservePath)
                        .arg(poseFilePath)
                        .arg(seamCompPath)
                        .arg(rebuildSummary));
            }

            if (ok)
            {
                ok = self != nullptr && self->ConfirmContinue("移动到焊接下枪安全位置并执行焊接轨迹");
            }

            if (ok)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("准备执行跳过扫描后的焊接轨迹");
                    self->SetProgressBusy(74, "执行焊接轨迹");
                    self->AppendLog(QString("开始执行焊接轨迹：%1").arg(seamCompPath));
                }

                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                        pRobotDriver,
                        seamCompPath,
                        param,
                        executeSummary,
                        processError,
                        &startSafeCoors,
                        &endSafeCoors,
                        [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                        [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                        [self](const QString& title, const QString& detail) -> bool
                        {
                            return self != nullptr && self->ShowCheckpointDialog(title, detail);
                        });
                if (ok)
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(QString("焊接轨迹执行完成：%1").arg(executeSummary));
                    }
                    message = QString("跳过扫描焊接完成。\n结果目录：%1\n姿态文件：%2\n补偿文件：%3\n执行结果：%4\n下枪安全位置：%5\n收枪安全位置：%6")
                        .arg(selectedDir)
                        .arg(poseFilePath)
                        .arg(seamCompPath)
                        .arg(executeSummary)
                        .arg(QString("%1, %2, %3")
                            .arg(startSafeCoors.dX, 0, 'f', 3)
                            .arg(startSafeCoors.dY, 0, 'f', 3)
                            .arg(startSafeCoors.dZ, 0, 'f', 3))
                        .arg(QString("%1, %2, %3")
                            .arg(endSafeCoors.dX, 0, 'f', 3)
                            .arg(endSafeCoors.dY, 0, 'f', 3)
                            .arg(endSafeCoors.dZ, 0, 'f', 3));
                }
                else
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(QString("焊接轨迹执行失败：%1").arg(processError));
                    }
                    message = QString("跳过扫描后焊接失败。\n%1").arg(processError);
                }
            }
            else if (message.isEmpty())
            {
                message = processError.isEmpty()
                    ? QString("跳过扫描焊接流程失败，请查看流程日志。")
                    : QString("跳过扫描焊接流程失败。\n%1").arg(processError);
            }

            QMetaObject::invokeMethod(qApp, [self, message, ok]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->SetFlowStep(ok ? "流程完成" : "流程失败，请查看流程日志");
                    self->FinishProgress(ok, ok ? QStringLiteral("跳过扫描焊接完成") : QStringLiteral("跳过扫描焊接失败"));
                    self->AppendLog(ok ? "跳过扫描焊接流程完成。" : "跳过扫描焊接流程失败。");
                    self->SetRunning(false);
                    if (ok)
                    {
                        QMessageBox::information(self, "跳过扫描焊接", message);
                    }
                    else
                    {
                        QMessageBox::warning(self, "跳过扫描焊接", message);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunLineScanProcess()
{
    SetFlowStep("大线扫粗定位功能暂未接入");
    AppendLog("线扫处理是整体大范围扫描获取多个焊道粗定位，当前精测点云库不接到这里。");
    QMessageBox::information(
        this,
        "线扫处理",
        "线扫处理用于整体大范围扫描和多个焊道粗定位。\n当前新增的点云库已接入预设参数后的局部精测量流程。");
}

void MeasureThenWeldDialog::RefreshWeldModeFromParam()
{
    if ((m_pActualWeldCheck == nullptr && m_pFinalTrajectoryStepSpin == nullptr) || m_pService == nullptr)
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!m_pService->LoadPresetParam(pRobotDriver, param, error))
    {
        return;
    }

    if (m_pActualWeldCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pActualWeldCheck);
        m_pActualWeldCheck->setChecked(param.bDoActualWeld);
        m_pActualWeldCheck->setToolTip(QString("勾选后按焊接速度运行：%1 mm/min\n取消勾选后按空跑速度运行：%2 mm/min\n下枪/收枪安全位置速度：%3 mm/min")
            .arg(param.dWeldSpeedMmPerMin, 0, 'f', 3)
            .arg(param.dDryRunSpeedMmPerMin, 0, 'f', 3)
            .arg(param.dWeldSafeMoveSpeedMmPerMin, 0, 'f', 3));
    }
    if (m_pFinalTrajectoryStepSpin != nullptr)
    {
        const QSignalBlocker blocker(m_pFinalTrajectoryStepSpin);
        m_pFinalTrajectoryStepSpin->setValue(param.dFinalWeldTrajectoryStepMm);
        m_pFinalTrajectoryStepSpin->setToolTip("只在最终生成/下发焊接轨迹时抽样；不影响前面的精测点、拐点、姿态补偿和焊道补偿。");
    }
}

void MeasureThenWeldDialog::SaveWeldModeToParam(bool doActualWeld)
{
    if (m_bRunning || m_pService == nullptr)
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!m_pService->LoadPresetParam(pRobotDriver, param, error))
    {
        AppendLog("保存焊接/空跑模式失败：" + error);
        return;
    }

    const QString paramPath = QString::fromStdString(param.sWeldParamFilePath.empty()
        ? param.sIniFilePath
        : param.sWeldParamFilePath);
    if (!RobotDataHelper::WriteParamValue(
        paramPath,
        QString::fromStdString(param.sWeldSectionName),
        "WeldEnable",
        doActualWeld ? "1" : "0",
        &error))
    {
        AppendLog("保存焊接/空跑模式失败：" + error);
        return;
    }

    AppendLog(QString("运行模式已切换为：%1").arg(doActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑")));
}

void MeasureThenWeldDialog::SaveFinalTrajectoryStepToParam(double stepMm)
{
    if (m_bLoadingSelectors || m_bRunning || m_pService == nullptr)
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!m_pService->LoadPresetParam(pRobotDriver, param, error))
    {
        AppendLog("保存最终轨迹点间距失败：" + error);
        return;
    }

    const double normalizedStepMm = (std::isfinite(stepMm) && stepMm > 0.0)
        ? std::clamp(stepMm, 0.5, 100.0)
        : 4.0;
    const QString paramPath = QString::fromStdString(param.sWeldParamFilePath.empty()
        ? param.sIniFilePath
        : param.sWeldParamFilePath);
    if (!RobotDataHelper::WriteParamValue(
        paramPath,
        QString::fromStdString(param.sWeldSectionName),
        FINAL_WELD_TRAJECTORY_STEP_KEY,
        QString::number(normalizedStepMm, 'f', 3),
        &error))
    {
        AppendLog("保存最终轨迹点间距失败：" + error);
        return;
    }

    AppendLog(QString("最终轨迹点间距已保存：%1 mm").arg(normalizedStepMm, 0, 'f', 3));
}

bool MeasureThenWeldDialog::IsActualWeldModeChecked() const
{
    return m_pActualWeldCheck == nullptr || m_pActualWeldCheck->isChecked();
}

void MeasureThenWeldDialog::AppendLog(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->AppendLog(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }
}

void MeasureThenWeldDialog::SetFlowStep(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->SetFlowStep(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    emit FlowStepChanged(QString("先测后焊流程进行中，目前：%1").arg(text));
}

void MeasureThenWeldDialog::ResetProgress(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->ResetProgress(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = false;
    m_nProgressValue = 0;
    m_sProgressText = text.trimmed().isEmpty() ? QStringLiteral("准备开始") : text.trimmed();
    if (m_pProgressAnimationTimer != nullptr)
    {
        m_pProgressAnimationTimer->stop();
    }
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(0);
        m_pProgressBar->show();
    }
}

void MeasureThenWeldDialog::SetProgress(int value, const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, value, text]()
            {
                if (self != nullptr)
                {
                    self->SetProgress(value, text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = false;
    m_nProgressValue = std::clamp(value, 0, 100);
    m_sProgressText = text.trimmed();
    if (m_pProgressAnimationTimer != nullptr)
    {
        m_pProgressAnimationTimer->stop();
    }
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText.isEmpty() ? QStringLiteral("处理中") : m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(m_nProgressValue);
        m_pProgressBar->show();
    }
}

void MeasureThenWeldDialog::SetProgressBusy(int baseValue, const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, baseValue, text]()
            {
                if (self != nullptr)
                {
                    self->SetProgressBusy(baseValue, text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = true;
    m_nProgressValue = (std::max)(m_nProgressValue, std::clamp(baseValue, 0, 96));
    m_sProgressText = text.trimmed().isEmpty() ? QStringLiteral("处理中") : text.trimmed();
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(m_nProgressValue);
        m_pProgressBar->show();
    }
    if (m_pProgressAnimationTimer != nullptr && !m_pProgressAnimationTimer->isActive())
    {
        m_pProgressAnimationTimer->start();
    }
}

void MeasureThenWeldDialog::FinishProgress(bool ok, const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, ok, text]()
            {
                if (self != nullptr)
                {
                    self->FinishProgress(ok, text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = false;
    m_nProgressValue = ok ? 100 : (std::max)(m_nProgressValue, 1);
    m_sProgressText = text.trimmed().isEmpty()
        ? (ok ? QStringLiteral("流程完成") : QStringLiteral("流程失败"))
        : text.trimmed();
    if (m_pProgressAnimationTimer != nullptr)
    {
        m_pProgressAnimationTimer->stop();
    }
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(m_nProgressValue);
        m_pProgressBar->show();
    }
}

void MeasureThenWeldDialog::UpdateProgressAnimation()
{
    if (!m_bProgressBusy)
    {
        if (m_pProgressAnimationTimer != nullptr)
        {
            m_pProgressAnimationTimer->stop();
        }
        return;
    }

    m_nProgressValue = (std::min)(95, m_nProgressValue + 1);
    static int dotTick = 0;
    dotTick = (dotTick + 1) % 4;
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(QString("%1%2").arg(m_sProgressText, QString(dotTick, QLatin1Char('.'))));
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setValue(m_nProgressValue);
    }
}

void MeasureThenWeldDialog::SetRunning(bool running)
{
    m_bRunning = running;
    m_pPresetParamBtn->setEnabled(!running);
    m_pSkipScanWeldBtn->setEnabled(!running);
    m_pLineScanProcessBtn->setEnabled(!running);
    if (m_pActualWeldCheck != nullptr)
    {
        m_pActualWeldCheck->setEnabled(!running);
    }
    if (m_pRobotCombo != nullptr)
    {
        m_pRobotCombo->setEnabled(!running);
    }
    if (m_pParamGroupCombo != nullptr)
    {
        m_pParamGroupCombo->setEnabled(!running);
    }
    if (m_pWeldProcessCombo != nullptr)
    {
        m_pWeldProcessCombo->setEnabled(!running && m_pWeldProcessCombo->count() > 0 && m_pWeldProcessCombo->currentData().toInt() >= 0);
    }
    if (m_pPoseCompGroupCombo != nullptr)
    {
        m_pPoseCompGroupCombo->setEnabled(!running && m_pPoseCompGroupCombo->count() > 0);
    }
    if (m_pSeamCompGroupCombo != nullptr)
    {
        m_pSeamCompGroupCombo->setEnabled(!running && m_pSeamCompGroupCombo->count() > 0);
    }
}
