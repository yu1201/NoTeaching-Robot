#include "WeldSeamCompDialog.h"

#include "ConfigDatabase.h"
#include "OPini.h"
#include "PointCloud3DView.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "WindowStyleHelper.h"

#include <QButtonGroup>
#include <QColor>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QSplitter>
#include <QStringConverter>
#include <QTextStream>
#include <QTimer>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QDoubleValidator>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStringList>
#include <QTextDocument>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
constexpr int EDITOR_FIELD_MAX_WIDTH = 240;
constexpr int COMP_SEGMENT_COUNT = 4;
constexpr int POSE_COMP_MATCH_BY_POSE = 0;
constexpr int POSE_COMP_MATCH_BY_SEGMENT_CODE = 1;
constexpr char POSE_COMP_MATCH_MODE_KEY[] = "PoseCompMatchMode";
constexpr char POSE_GROUP_COUNT_KEY[] = "PoseCompGroupCount";
constexpr char POSE_ACTIVE_GROUP_INDEX_KEY[] = "ActivePoseCompGroupIndex";
constexpr char SEAM_GROUP_COUNT_KEY[] = "SeamCompGroupCount";
constexpr char SEAM_ACTIVE_GROUP_INDEX_KEY[] = "ActiveSeamCompGroupIndex";
constexpr char CORNER_COMP_ENABLED_KEY[] = "Enabled";
constexpr char INNER_TO_OUTER_CORNER_COMP_KEY[] = "InnerToOuter";
constexpr char INNER_TO_INNER_CORNER_COMP_KEY[] = "InnerToInner";
constexpr char OUTER_TO_OUTER_CORNER_COMP_KEY[] = "OuterToOuter";
constexpr char OUTER_TO_INNER_CORNER_COMP_KEY[] = "OuterToInner";

int NormalizePoseCompMatchMode(int mode)
{
    return mode == POSE_COMP_MATCH_BY_SEGMENT_CODE
        ? POSE_COMP_MATCH_BY_SEGMENT_CODE
        : POSE_COMP_MATCH_BY_POSE;
}

QVector<WeldSeamCompDialog::CompType> BuildDefaultPoseTypes()
{
    return {
        { "姿态0 / 低平台", "low_platform" },
        { "姿态1 / 上升边", "rising_edge" },
        { "姿态2 / 高平台", "high_platform" },
        { "姿态3 / 下降边", "falling_edge" }
    };
}

QVector<WeldSeamCompDialog::SeamCompRow> BuildDefaultSeamRows()
{
    return {
        { "WeldSeamComp0", "低平台", "low_platform", 0.0, 0.0, 0.0 },
        { "WeldSeamComp1", "上升边", "rising_edge", 0.0, 0.0, 0.0 },
        { "WeldSeamComp2", "高平台", "high_platform", 0.0, 0.0, 0.0 },
        { "WeldSeamComp3", "下降边", "falling_edge", 0.0, 0.0, 0.0 }
    };
}

QString WeldPoseCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldPoseCompParam.ini").arg(robotName));
}

QString WeldSeamCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldSeamCompParam.ini").arg(robotName));
}

QString DatabaseText(const QString& title)
{
    return QString("%1：%2").arg(title, QDir::toNativeSeparators(ConfigDatabase::DatabasePath()));
}

QString PoseCornerGroupModule(int groupIndex)
{
    return QStringLiteral("WeldPoseCompParam/CornerCompensation/Group%1").arg(groupIndex);
}

QString PoseCornerSlotModule(int flatIndex)
{
    return QStringLiteral("WeldPoseCompParam/CornerCompensation/Slot%1").arg(flatIndex);
}

bool ReadRobotSetting(const QString& robotName, const QString& module, const QString& key, QString* value)
{
    if (robotName.trimmed().isEmpty())
    {
        return false;
    }
    return ConfigDatabase::ReadScopedSetting(QStringLiteral("robot"), robotName.trimmed(), module, key, value);
}

bool WriteRobotSetting(
    const QString& robotName,
    const QString& module,
    const QString& key,
    const QString& value,
    const QString& valueType,
    QString* error)
{
    if (robotName.trimmed().isEmpty())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("机器人名称为空，无法写入配置数据库。");
        }
        return false;
    }
    if (ConfigDatabase::WriteScopedSetting(QStringLiteral("robot"), robotName.trimmed(), module, key, value, valueType))
    {
        return true;
    }
    if (error != nullptr)
    {
        *error = QStringLiteral("写入配置数据库失败：%1 / %2").arg(module, key);
    }
    return false;
}

bool ReadRobotBool(const QString& robotName, const QString& module, const QString& key, bool defaultValue)
{
    QString value;
    if (!ReadRobotSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    const QString normalized = value.trimmed().toLower();
    return normalized == QStringLiteral("1")
        || normalized == QStringLiteral("true")
        || normalized == QStringLiteral("yes");
}

double ReadRobotDouble(const QString& robotName, const QString& module, const QString& key, double defaultValue)
{
    QString value;
    if (!ReadRobotSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    bool ok = false;
    const double parsed = value.trimmed().toDouble(&ok);
    return ok ? parsed : defaultValue;
}

QString FormatNumber(double value)
{
    return QString::number(value, 'f', 3);
}

bool ReadIniDouble(COPini& ini, const std::string& key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

std::string LocalString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData());
}

QDoubleSpinBox* CreateCornerCompensationSpin(QWidget* parent)
{
    QDoubleSpinBox* spin = new QDoubleSpinBox(parent);
    spin->setRange(-200.0, 200.0);
    spin->setDecimals(3);
    spin->setSingleStep(0.5);
    spin->setSuffix(" mm");
    spin->setMinimumWidth(150);
    spin->setValue(0.0);
    return spin;
}

bool IsCornerCompensationSegmentKind(const QString& segmentKind)
{
    return segmentKind.compare("rising_edge", Qt::CaseInsensitive) == 0
        || segmentKind.compare("falling_edge", Qt::CaseInsensitive) == 0;
}
}

WeldSeamCompDialog::WeldSeamCompDialog(ContralUnit* pContralUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_poseTypes(BuildDefaultPoseTypes())
{
    BuildUi();
    LoadRobotList();
    LoadCurrentParam();
}

void WeldSeamCompDialog::closeEvent(QCloseEvent* event)
{
    QString error;
    StoreEditorValues(false, error);
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "补偿参数", [this]() { return SaveCurrentParam(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

void WeldSeamCompDialog::BuildUi()
{
    setWindowTitle("补偿参数");
    setObjectName("WeldSeamCompDialog");
    setWindowFlags(windowFlags() | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
    setMinimumSize(560, 460);
    setStyleSheet(QString(
        "QDialog#WeldSeamCompDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:checked { background: #2F6F7A; border-color: #8EE7EC; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 7px; padding: 4px 6px; }"
        "QLineEdit[readOnly=\"true\"] { color: #A7C9CF; background: #101923; }"
        "QAbstractSpinBox { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 7px; padding: 4px 6px; }"
        "QAbstractSpinBox:disabled { color: #6E8590; background: #101923; }"
        "QCheckBox { color: #D8E8EA; spacing: 8px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }")
        + UnifiedComboBoxStyleSheet());

    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(18, 14, 18, 14);
    rootLayout->setSpacing(8);

    QLabel* titleLabel = new QLabel("补偿参数");
    titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QHBoxLayout* topLayout = new QHBoxLayout();
    topLayout->addWidget(new QLabel("机器人："));
    m_pRobotCombo = new QComboBox();
    m_pRobotCombo->setMinimumWidth(180);
    topLayout->addWidget(m_pRobotCombo);
    topLayout->addSpacing(18);

    m_pPoseModeBtn = new QPushButton("姿态补偿");
    m_pSeamModeBtn = new QPushButton("焊道补偿");
    m_pPoseModeBtn->setCheckable(true);
    m_pSeamModeBtn->setCheckable(true);
    m_pModeGroup = new QButtonGroup(this);
    m_pModeGroup->setExclusive(true);
    m_pModeGroup->addButton(m_pPoseModeBtn, 0);
    m_pModeGroup->addButton(m_pSeamModeBtn, 1);
    m_pPoseModeBtn->setChecked(true);
    topLayout->addWidget(m_pPoseModeBtn);
    topLayout->addWidget(m_pSeamModeBtn);
    topLayout->addStretch(1);
    rootLayout->addLayout(topLayout);

    QScrollArea* editorScrollArea = new QScrollArea(this);
    editorScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(editorScrollArea);

    QWidget* editorContent = new QWidget(editorScrollArea);
    editorContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    QVBoxLayout* editorContentLayout = new QVBoxLayout(editorContent);
    editorContentLayout->setContentsMargins(0, 0, 8, 0);
    editorContentLayout->setSpacing(8);
    editorContentLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    QVBoxLayout* contentLayout = new QVBoxLayout();
    contentLayout->setSpacing(10);

    QWidget* groupPanel = new QWidget();
    groupPanel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    groupPanel->setMaximumHeight(210);
    QVBoxLayout* groupPanelLayout = new QVBoxLayout(groupPanel);
    groupPanelLayout->setContentsMargins(0, 0, 0, 0);
    groupPanelLayout->setSpacing(6);
    QLabel* groupPanelTitle = new QLabel("补偿组");
    groupPanelTitle->setStyleSheet("font-weight: bold; color: #9ED8DB;");
    groupPanelLayout->addWidget(groupPanelTitle);
    m_pGroupList = new QListWidget();
    m_pGroupList->setMinimumWidth(170);
    m_pGroupList->setMinimumHeight(120);
    m_pGroupList->setStyleSheet(
        "QListWidget { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 4px; }"
        "QListWidget::item { padding: 6px 6px; border-radius: 5px; }"
        "QListWidget::item:selected { background: #2F6F7A; color: #F7FCFC; }"
        "QListWidget::item:hover { background: #233645; }");
    groupPanelLayout->addWidget(m_pGroupList, 1);

    QHBoxLayout* groupActionLayout = new QHBoxLayout();
    groupActionLayout->setContentsMargins(0, 0, 0, 0);
    groupActionLayout->setSpacing(6);
    m_pNewGroupBtn = new QPushButton("新建");
    m_pRenameGroupBtn = new QPushButton("重命名");
    m_pDeleteGroupBtn = new QPushButton("删除");
    m_pNewGroupBtn->setMinimumWidth(84);
    m_pRenameGroupBtn->setMinimumWidth(96);
    m_pDeleteGroupBtn->setMinimumWidth(84);
    groupActionLayout->addWidget(m_pNewGroupBtn);
    groupActionLayout->addWidget(m_pRenameGroupBtn);
    groupActionLayout->addWidget(m_pDeleteGroupBtn);
    groupActionLayout->addStretch(1);
    groupPanelLayout->addLayout(groupActionLayout);
    contentLayout->addWidget(groupPanel, 0);

    QGroupBox* editorGroup = new QGroupBox("补偿编辑");
    editorGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    editorGroup->setMaximumHeight(460);
    QGridLayout* editorLayout = new QGridLayout(editorGroup);
    editorLayout->setHorizontalSpacing(8);
    editorLayout->setVerticalSpacing(6);
    editorLayout->setAlignment(Qt::AlignTop);
    editorLayout->setColumnStretch(0, 0);
    editorLayout->setColumnStretch(1, 0);
    editorLayout->setColumnStretch(2, 0);
    editorLayout->setColumnStretch(5, 1);
    editorLayout->setRowStretch(9, 1);

    m_pPoseMatchModeLabel = new QLabel("姿态匹配：");
    editorLayout->addWidget(m_pPoseMatchModeLabel, 0, 0);
    m_pPoseMatchModeCombo = new QComboBox();
    m_pPoseMatchModeCombo->addItem("按姿态匹配", POSE_COMP_MATCH_BY_POSE);
    m_pPoseMatchModeCombo->addItem("按四类段属性", POSE_COMP_MATCH_BY_SEGMENT_CODE);
    m_pPoseMatchModeCombo->setMinimumWidth(150);
    editorLayout->addWidget(m_pPoseMatchModeCombo, 0, 1, 1, 2, Qt::AlignLeft);

    editorLayout->addWidget(new QLabel("段类型："), 1, 0);
    m_pTypeCombo = new QComboBox();
    m_pTypeCombo->setMinimumWidth(240);
    editorLayout->addWidget(m_pTypeCombo, 1, 1, 1, 2, Qt::AlignLeft);

    m_pHintLabel = new QLabel();
    m_pHintLabel->setWordWrap(true);
    editorLayout->addWidget(m_pHintLabel, 2, 0, 1, 6);

    QDoubleValidator* validator = new QDoubleValidator(-100000.0, 100000.0, 6, this);
    validator->setNotation(QDoubleValidator::StandardNotation);
    m_pPoseDisplayWidget = new QWidget();
    QGridLayout* poseLayout = new QGridLayout(m_pPoseDisplayWidget);
    poseLayout->setContentsMargins(0, 0, 0, 0);
    poseLayout->setHorizontalSpacing(8);
    const char* poseLabels[3] = { "姿态RX：", "姿态RY：", "姿态RZ：" };
    for (int index = 0; index < 3; ++index)
    {
        poseLayout->addWidget(new QLabel(QString::fromUtf8(poseLabels[index])), 0, index * 2);
        m_pPoseValues[index] = new QLineEdit();
        m_pPoseValues[index]->setValidator(validator);
        m_pPoseValues[index]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_pPoseValues[index]->setMinimumWidth(90);
        poseLayout->addWidget(m_pPoseValues[index], 0, index * 2 + 1);
    }
    editorLayout->addWidget(m_pPoseDisplayWidget, 3, 0, 1, 6, Qt::AlignLeft);

    for (int index = 0; index < 3; ++index)
    {
        m_pEditLabels[index] = new QLabel();
        m_pEditValues[index] = new QLineEdit();
        m_pEditValues[index]->setValidator(validator);
        m_pEditValues[index]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_pEditValues[index]->setMinimumWidth(130);
        editorLayout->addWidget(m_pEditLabels[index], 4 + index, 0);
        editorLayout->addWidget(m_pEditValues[index], 4 + index, 1, 1, 5, Qt::AlignLeft);
    }

    m_pCornerCompensationCheck = new QCheckBox("启用本组拐点补偿并生成补偿后 2mm 点");
    m_pCornerCompensationCheck->setToolTip("启用后，上升边和下降边的拐点补偿值会随当前姿态补偿组保存。");
    editorLayout->addWidget(m_pCornerCompensationCheck, 7, 0, 1, 6);

    m_pCornerCompensationWidget = new QWidget(editorGroup);
    QGridLayout* cornerCompLayout = new QGridLayout(m_pCornerCompensationWidget);
    cornerCompLayout->setContentsMargins(0, 0, 0, 0);
    cornerCompLayout->setHorizontalSpacing(8);
    cornerCompLayout->setVerticalSpacing(6);
    const char* cornerLabels[4] = {
        "inner intoout(mm)：",
        "inner intoin(mm)：",
        "outer outtoout(mm)：",
        "outer outtoin(mm)："
    };
    for (int index = 0; index < 4; ++index)
    {
        cornerCompLayout->addWidget(new QLabel(QString::fromUtf8(cornerLabels[index])), index / 2, (index % 2) * 2);
        m_pCornerCompensationValues[index] = CreateCornerCompensationSpin(m_pCornerCompensationWidget);
        cornerCompLayout->addWidget(m_pCornerCompensationValues[index], index / 2, (index % 2) * 2 + 1);
    }
    editorLayout->addWidget(m_pCornerCompensationWidget, 8, 0, 1, 6);
    contentLayout->addWidget(editorGroup, 1);
    editorContentLayout->addLayout(contentLayout);

    m_pPathLabel = new QLabel();
    m_pPathLabel->setWordWrap(true);
    editorContentLayout->addWidget(m_pPathLabel);
    editorContentLayout->addStretch(1);
    editorScrollArea->setWidget(editorContent);

    m_pCompPreviewTimer = new QTimer(this);
    m_pCompPreviewTimer->setSingleShot(true);
    m_pCompPreviewTimer->setInterval(120);
    connect(m_pCompPreviewTimer, &QTimer::timeout, this, [this]() { RecomputeCompPreview(); });

    editorScrollArea->setMinimumWidth(560);
    QSplitter* mainSplitter = new QSplitter(Qt::Horizontal, this);
    mainSplitter->setObjectName("CompMainSplitter");
    mainSplitter->setChildrenCollapsible(false);
    mainSplitter->addWidget(editorScrollArea);
    mainSplitter->addWidget(CreateCompPreviewPanel());
    mainSplitter->setStretchFactor(0, 0);
    mainSplitter->setStretchFactor(1, 1);
    mainSplitter->setSizes({ 780, 1040 });
    rootLayout->addWidget(mainSplitter, 1);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch(1);
    m_pReloadBtn = new QPushButton("重新读取");
    m_pSaveBtn = new QPushButton("保存");
    m_pReloadBtn->setMinimumWidth(120);
    m_pSaveBtn->setMinimumWidth(120);
    actionLayout->addWidget(m_pReloadBtn);
    actionLayout->addWidget(m_pSaveBtn);
    rootLayout->addLayout(actionLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1000);
    m_pLogText->setMaximumHeight(110);
    m_pLogText->setPlainText("日志：等待读取补偿参数...");
    rootLayout->addWidget(m_pLogText, 1);

    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (!m_bLoading)
            {
                LoadCurrentParam();
            }
        });
    connect(m_pModeGroup, qOverload<int>(&QButtonGroup::idClicked), this, [this](int id)
        {
            SetMode(id == 0 ? CompMode::Pose : CompMode::Seam);
        });
    connect(m_pGroupList, &QListWidget::currentRowChanged, this, [this](int index)
        {
            if (m_bLoading)
            {
                return;
            }
            QString error;
            StoreEditorValues(false, error);
            m_currentGroupIndex = std::max(0, index);
            if (m_mode == CompMode::Pose)
            {
                m_currentPoseGroupIndex = m_currentGroupIndex;
                m_poseCompMatchMode = CurrentPoseGroupMatchMode();
            }
            else
            {
                m_currentSeamGroupIndex = m_currentGroupIndex;
            }
            m_currentTypeIndex = 0;
            RefreshTypeCombo();
            RefreshEditor();
        });
    connect(m_pTypeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index)
        {
            if (m_bLoading)
            {
                return;
            }
            QString error;
            StoreEditorValues(false, error);
            m_currentTypeIndex = std::max(0, index);
            RefreshEditor();
        });
    connect(m_pPoseMatchModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index)
        {
            if (m_bLoading || m_pPoseMatchModeCombo == nullptr)
            {
                return;
            }
            QString error;
            StoreEditorValues(false, error);
            SetCurrentPoseGroupMatchMode(m_pPoseMatchModeCombo->itemData(index).toInt());
            RefreshEditor();
        });
    connect(m_pNewGroupBtn, &QPushButton::clicked, this, &WeldSeamCompDialog::AddCurrentGroup);
    connect(m_pRenameGroupBtn, &QPushButton::clicked, this, &WeldSeamCompDialog::RenameCurrentGroup);
    connect(m_pDeleteGroupBtn, &QPushButton::clicked, this, &WeldSeamCompDialog::DeleteCurrentGroup);
    connect(m_pReloadBtn, &QPushButton::clicked, this, [this]() { LoadCurrentParam(); });
    connect(m_pSaveBtn, &QPushButton::clicked, this, [this]() { SaveCurrentParam(); });
    connect(m_pCornerCompensationCheck, &QCheckBox::toggled, this, [this](bool)
        {
            RefreshCornerCompensationEditor();
            ScheduleCompPreview();
        });

    // 编辑补偿值时实时重算补偿后焊道（textEdited 只在用户输入时触发，不受 setText 影响）。
    for (int valueIndex = 0; valueIndex < 3; ++valueIndex)
    {
        if (m_pEditValues[valueIndex] != nullptr)
        {
            connect(m_pEditValues[valueIndex], &QLineEdit::textEdited, this, [this](const QString&) { ScheduleCompPreview(); });
        }
        if (m_pPoseValues[valueIndex] != nullptr)
        {
            connect(m_pPoseValues[valueIndex], &QLineEdit::textEdited, this, [this](const QString&) { ScheduleCompPreview(); });
        }
    }
    for (int cornerIndex = 0; cornerIndex < 4; ++cornerIndex)
    {
        if (m_pCornerCompensationValues[cornerIndex] != nullptr)
        {
            connect(m_pCornerCompensationValues[cornerIndex], qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { ScheduleCompPreview(); });
        }
    }

    RefreshGroupCombo();
    RefreshTypeCombo();
    RefreshEditor();

    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(1340, 780), 0.95, 0.9);
}

void WeldSeamCompDialog::LoadRobotList()
{
    if (m_pRobotCombo == nullptr)
    {
        return;
    }

    m_bLoading = true;
    m_pRobotCombo->clear();
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        m_pRobotCombo->addItem(info.displayName, info.robotName);
    }
    m_bLoading = false;
}

QString WeldSeamCompDialog::CurrentRobotName() const
{
    if (m_pRobotCombo == nullptr || m_pRobotCombo->currentIndex() < 0)
    {
        return QString();
    }
    return m_pRobotCombo->currentData().toString();
}

QString WeldSeamCompDialog::CurrentPoseParamPath() const
{
    const QString robotName = CurrentRobotName();
    return robotName.isEmpty() ? QString() : WeldPoseCompParamPath(robotName);
}

QString WeldSeamCompDialog::CurrentSeamParamPath() const
{
    const QString robotName = CurrentRobotName();
    return robotName.isEmpty() ? QString() : WeldSeamCompParamPath(robotName);
}

int WeldSeamCompDialog::CurrentTypeIndex() const
{
    if (CurrentGroupCount() <= 0)
    {
        return -1;
    }
    return std::clamp(m_currentTypeIndex, 0, COMP_SEGMENT_COUNT - 1);
}

int WeldSeamCompDialog::CurrentRowCount() const
{
    return CurrentGroupCount() > 0 ? COMP_SEGMENT_COUNT : 0;
}

int WeldSeamCompDialog::CurrentGroupCount() const
{
    return m_mode == CompMode::Pose ? m_poseGroupNames.size() : m_seamGroupNames.size();
}

int WeldSeamCompDialog::CurrentFlatRowIndex() const
{
    const int groupCount = CurrentGroupCount();
    if (groupCount <= 0)
    {
        return -1;
    }
    const int groupIndex = std::clamp(m_currentGroupIndex, 0, groupCount - 1);
    const int segmentIndex = CurrentTypeIndex();
    if (segmentIndex < 0)
    {
        return -1;
    }
    return groupIndex * COMP_SEGMENT_COUNT + segmentIndex;
}

QString WeldSeamCompDialog::ModeGroupName() const
{
    return m_mode == CompMode::Pose ? "姿态补偿组" : "焊道补偿组";
}

QString WeldSeamCompDialog::DefaultGroupName(int index) const
{
    return QString("%1%2").arg(m_mode == CompMode::Pose ? "姿态补偿组" : "焊道补偿组").arg(std::max(0, index) + 1);
}

QString WeldSeamCompDialog::DefaultPoseRowName(int index) const
{
    if (index >= 0 && index < m_poseTypes.size())
    {
        return m_poseTypes[index].displayName;
    }
    return QString("姿态补偿%1").arg(std::max(0, index) + 1);
}

QString WeldSeamCompDialog::DefaultPoseSegmentKind(int index) const
{
    if (index >= 0 && index < m_poseTypes.size())
    {
        return m_poseTypes[index].segmentKind;
    }
    return QString("PoseGroup%1").arg(std::max(0, index));
}

WeldSeamCompDialog::PoseCompRow WeldSeamCompDialog::MakeDefaultPoseRow(int groupIndex, int segmentIndex) const
{
    const int flatIndex = std::max(0, groupIndex) * COMP_SEGMENT_COUNT + std::clamp(segmentIndex, 0, COMP_SEGMENT_COUNT - 1);
    PoseCompRow row;
    row.sectionName = QString("WeldPoseComp%1").arg(flatIndex);
    row.name = DefaultPoseRowName(segmentIndex);
    row.segmentKind = DefaultPoseSegmentKind(segmentIndex);
    return row;
}

WeldSeamCompDialog::SeamCompRow WeldSeamCompDialog::MakeDefaultSeamRow(int groupIndex, int segmentIndex) const
{
    const QVector<SeamCompRow> defaultRows = BuildDefaultSeamRows();
    const int safeSegmentIndex = std::clamp(segmentIndex, 0, COMP_SEGMENT_COUNT - 1);
    SeamCompRow row = safeSegmentIndex < defaultRows.size()
        ? defaultRows[safeSegmentIndex]
        : SeamCompRow();
    row.sectionName = QString("WeldSeamComp%1").arg(std::max(0, groupIndex) * COMP_SEGMENT_COUNT + safeSegmentIndex);
    if (row.name.isEmpty())
    {
        row.name = QString("焊道补偿%1").arg(safeSegmentIndex + 1);
    }
    if (row.segmentKind.isEmpty())
    {
        row.segmentKind = DefaultPoseSegmentKind(safeSegmentIndex);
    }
    return row;
}

int WeldSeamCompDialog::CurrentPoseGroupMatchMode() const
{
    const int groupCount = static_cast<int>(m_poseGroupMatchModes.size());
    if (groupCount <= 0)
    {
        return NormalizePoseCompMatchMode(m_poseCompMatchMode);
    }
    const int groupIndex = std::clamp(m_currentPoseGroupIndex, 0, groupCount - 1);
    return NormalizePoseCompMatchMode(m_poseGroupMatchModes[groupIndex]);
}

void WeldSeamCompDialog::SetCurrentPoseGroupMatchMode(int mode)
{
    const int normalizedMode = NormalizePoseCompMatchMode(mode);
    m_poseCompMatchMode = normalizedMode;
    const int groupCount = static_cast<int>(m_poseGroupNames.size());
    if (groupCount <= 0)
    {
        return;
    }
    if (m_poseGroupMatchModes.size() < groupCount)
    {
        m_poseGroupMatchModes.resize(groupCount);
        for (int index = 0; index < m_poseGroupMatchModes.size(); ++index)
        {
            m_poseGroupMatchModes[index] = NormalizePoseCompMatchMode(m_poseGroupMatchModes[index]);
        }
    }
    const int groupIndex = std::clamp(m_currentPoseGroupIndex, 0, groupCount - 1);
    m_poseGroupMatchModes[groupIndex] = normalizedMode;
}

void WeldSeamCompDialog::LoadCurrentParam()
{
    const QString posePath = CurrentPoseParamPath();
    const QString seamPath = CurrentSeamParamPath();
    if (posePath.isEmpty() || seamPath.isEmpty())
    {
        AppendLog("未选择机器人，无法读取补偿参数。");
        return;
    }

    m_bLoading = true;
    LoadPoseParam(posePath);
    LoadSeamParam(seamPath);
    RefreshGroupCombo();
    RefreshTypeCombo();
    m_bLoading = false;
    RefreshEditor();

    AppendLog(QString("已读取补偿参数：姿态组=%1（%2条），焊道组=%3（%4条）。")
        .arg(m_poseGroupNames.size())
        .arg(m_poseRows.size())
        .arg(m_seamGroupNames.size())
        .arg(m_seamRows.size()));
    MarkCleanSnapshot();
}

void WeldSeamCompDialog::RefreshCornerCompensationEditor()
{
    const int flatIndex = CurrentFlatRowIndex();
    const bool poseMode = m_mode == CompMode::Pose;
    const bool hasGroup = m_currentGroupIndex >= 0 && m_currentGroupIndex < m_poseGroupCornerCompEnabled.size();
    const bool groupEnabled = hasGroup && m_poseGroupCornerCompEnabled[m_currentGroupIndex];
    const bool hasCornerRow = poseMode
        && flatIndex >= 0
        && flatIndex < m_poseRows.size()
        && IsCornerCompensationSegmentKind(m_poseRows[flatIndex].segmentKind);

    if (m_pCornerCompensationCheck != nullptr)
    {
        m_pCornerCompensationCheck->setVisible(poseMode);
        m_pCornerCompensationCheck->setEnabled(poseMode && hasGroup);
    }
    if (m_pCornerCompensationWidget != nullptr)
    {
        m_pCornerCompensationWidget->setVisible(hasCornerRow);
    }
    for (QDoubleSpinBox* spin : m_pCornerCompensationValues)
    {
        if (spin != nullptr)
        {
            spin->setEnabled(hasCornerRow && groupEnabled);
        }
    }
}

void WeldSeamCompDialog::LoadPoseParam(const QString& path)
{
    m_poseCompMatchMode = POSE_COMP_MATCH_BY_POSE;
    m_poseMatchMaxErrorDeg = 5.0;
    m_poseGroupNames.clear();
    m_poseGroupMatchModes.clear();
    m_poseGroupCornerCompEnabled.clear();
    m_poseRows.clear();

    if (path.isEmpty() || !ConfigDatabase::HasIniFile(path))
    {
        m_poseGroupNames.push_back("姿态补偿组1");
        m_poseGroupMatchModes.push_back(POSE_COMP_MATCH_BY_POSE);
        m_poseGroupCornerCompEnabled.push_back(false);
        m_poseRows.reserve(COMP_SEGMENT_COUNT);
        for (int index = 0; index < COMP_SEGMENT_COUNT; ++index)
        {
            m_poseRows.push_back(MakeDefaultPoseRow(0, index));
        }
        m_currentPoseGroupIndex = 0;
        LoadPoseCornerCompensationFromDatabase(CurrentRobotName());
        AppendLog("姿态补偿参数不存在，使用默认槽位。");
        return;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldPoseComp");
    ReadIniDouble(ini, "PoseMatchMaxErrorDeg", m_poseMatchMaxErrorDeg);
    int matchMode = m_poseCompMatchMode;
    if (ini.ReadString(false, POSE_COMP_MATCH_MODE_KEY, &matchMode) > 0)
    {
        m_poseCompMatchMode = NormalizePoseCompMatchMode(matchMode);
    }

    int activeGroupIndex = 0;
    ini.ReadString(false, POSE_ACTIVE_GROUP_INDEX_KEY, &activeGroupIndex);
    int count = COMP_SEGMENT_COUNT;
    ini.ReadString(false, "PoseCompCount", &count);
    count = std::max(0, count);
    int groupCount = 0;
    const bool hasGroupCount = ini.ReadString(false, POSE_GROUP_COUNT_KEY, &groupCount) > 0;
    groupCount = hasGroupCount
        ? std::max(0, groupCount)
        : (count <= 0 ? 0 : (count + COMP_SEGMENT_COUNT - 1) / COMP_SEGMENT_COUNT);
    const int expectedCount = groupCount * COMP_SEGMENT_COUNT;
    m_poseGroupNames.reserve(groupCount);
    m_poseRows.reserve(expectedCount);

    for (int groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        QString groupName = QString("姿态补偿组%1").arg(groupIndex + 1);
        std::string groupNameText;
        ini.SetSectionName(LocalString(QString("WeldPoseCompGroup%1").arg(groupIndex)));
        if (ini.ReadString(false, "Name", groupNameText) > 0)
        {
            groupName = DecodeRobotMessageText(groupNameText);
        }
        int groupMatchMode = m_poseCompMatchMode;
        if (ini.ReadString(false, POSE_COMP_MATCH_MODE_KEY, &groupMatchMode) <= 0)
        {
            groupMatchMode = m_poseCompMatchMode;
        }
        m_poseGroupNames.push_back(groupName);
        m_poseGroupMatchModes.push_back(NormalizePoseCompMatchMode(groupMatchMode));
        m_poseGroupCornerCompEnabled.push_back(false);

        for (int segmentIndex = 0; segmentIndex < COMP_SEGMENT_COUNT; ++segmentIndex)
        {
            const int flatIndex = groupIndex * COMP_SEGMENT_COUNT + segmentIndex;
            PoseCompRow row = MakeDefaultPoseRow(groupIndex, segmentIndex);

            std::string text;
            ini.SetSectionName(LocalString(QString("WeldPoseComp%1").arg(flatIndex)));
            if (ini.ReadString(false, "Name", text) > 0)
            {
                row.name = DecodeRobotMessageText(text);
            }
            if (ini.ReadString(false, "SegmentKind", text) > 0)
            {
                row.segmentKind = DecodeRobotMessageText(text);
            }
            if (row.segmentKind.isEmpty())
            {
                row.segmentKind = DefaultPoseSegmentKind(segmentIndex);
            }

            ReadIniDouble(ini, "Rx", row.poseRx);
            ReadIniDouble(ini, "Ry", row.poseRy);
            ReadIniDouble(ini, "Rz", row.poseRz);
            ReadIniDouble(ini, "CompX", row.compX);
            ReadIniDouble(ini, "CompY", row.compY);
            ReadIniDouble(ini, "CompZ", row.compZ);

            m_poseRows.push_back(row);
        }
    }
    m_currentPoseGroupIndex = groupCount > 0 ? std::clamp(activeGroupIndex, 0, groupCount - 1) : 0;
    if (m_poseGroupCornerCompEnabled.size() < groupCount)
    {
        m_poseGroupCornerCompEnabled.resize(groupCount);
    }
    if (m_poseGroupCornerCompEnabled.size() > groupCount)
    {
        m_poseGroupCornerCompEnabled.resize(groupCount);
    }
    LoadPoseCornerCompensationFromDatabase(CurrentRobotName());
    if (m_mode == CompMode::Pose)
    {
        m_currentGroupIndex = m_currentPoseGroupIndex;
    }
    m_poseCompMatchMode = CurrentPoseGroupMatchMode();
}

void WeldSeamCompDialog::LoadPoseCornerCompensationFromDatabase(const QString& robotName)
{
    if (robotName.trimmed().isEmpty())
    {
        return;
    }

    for (int groupIndex = 0; groupIndex < m_poseGroupCornerCompEnabled.size(); ++groupIndex)
    {
        m_poseGroupCornerCompEnabled[groupIndex] = ReadRobotBool(
            robotName,
            PoseCornerGroupModule(groupIndex),
            CORNER_COMP_ENABLED_KEY,
            false);
    }

    for (int flatIndex = 0; flatIndex < m_poseRows.size(); ++flatIndex)
    {
        PoseCompRow& row = m_poseRows[flatIndex];
        if (!IsCornerCompensationSegmentKind(row.segmentKind))
        {
            row.innerToOuterCornerComp = 0.0;
            row.innerToInnerCornerComp = 0.0;
            row.outerToOuterCornerComp = 0.0;
            row.outerToInnerCornerComp = 0.0;
            continue;
        }

        const QString module = PoseCornerSlotModule(flatIndex);
        row.innerToOuterCornerComp = ReadRobotDouble(
            robotName,
            module,
            INNER_TO_OUTER_CORNER_COMP_KEY,
            row.innerToOuterCornerComp);
        row.innerToInnerCornerComp = ReadRobotDouble(
            robotName,
            module,
            INNER_TO_INNER_CORNER_COMP_KEY,
            row.innerToInnerCornerComp);
        row.outerToOuterCornerComp = ReadRobotDouble(
            robotName,
            module,
            OUTER_TO_OUTER_CORNER_COMP_KEY,
            row.outerToOuterCornerComp);
        row.outerToInnerCornerComp = ReadRobotDouble(
            robotName,
            module,
            OUTER_TO_INNER_CORNER_COMP_KEY,
            row.outerToInnerCornerComp);
    }
}

void WeldSeamCompDialog::LoadSeamParam(const QString& path)
{
    m_seamGroupNames.clear();
    m_seamRows.clear();

    if (path.isEmpty() || !ConfigDatabase::HasIniFile(path))
    {
        m_seamGroupNames.push_back("焊道补偿组1");
        m_seamRows.reserve(COMP_SEGMENT_COUNT);
        for (int index = 0; index < COMP_SEGMENT_COUNT; ++index)
        {
            m_seamRows.push_back(MakeDefaultSeamRow(0, index));
        }
        m_currentSeamGroupIndex = 0;
        AppendLog("焊道补偿参数不存在，使用默认槽位。");
        return;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldSeamComp");
    int activeGroupIndex = 0;
    ini.ReadString(false, SEAM_ACTIVE_GROUP_INDEX_KEY, &activeGroupIndex);
    int count = COMP_SEGMENT_COUNT;
    ini.ReadString(false, "SeamCompCount", &count);
    count = std::max(0, count);
    int groupCount = 0;
    const bool hasGroupCount = ini.ReadString(false, SEAM_GROUP_COUNT_KEY, &groupCount) > 0;
    groupCount = hasGroupCount
        ? std::max(0, groupCount)
        : (count <= 0 ? 0 : (count + COMP_SEGMENT_COUNT - 1) / COMP_SEGMENT_COUNT);
    m_seamGroupNames.reserve(groupCount);
    m_seamRows.reserve(groupCount * COMP_SEGMENT_COUNT);

    for (int groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        QString groupName = QString("焊道补偿组%1").arg(groupIndex + 1);
        std::string groupNameText;
        ini.SetSectionName(LocalString(QString("WeldSeamCompGroup%1").arg(groupIndex)));
        if (ini.ReadString(false, "Name", groupNameText) > 0)
        {
            groupName = DecodeRobotMessageText(groupNameText);
        }
        m_seamGroupNames.push_back(groupName);

        for (int segmentIndex = 0; segmentIndex < COMP_SEGMENT_COUNT; ++segmentIndex)
        {
            const int flatIndex = groupIndex * COMP_SEGMENT_COUNT + segmentIndex;
            SeamCompRow row = MakeDefaultSeamRow(groupIndex, segmentIndex);

            std::string text;
            ini.SetSectionName(LocalString(QString("WeldSeamComp%1").arg(flatIndex)));
            if (ini.ReadString(false, "Name", text) > 0)
            {
                row.name = DecodeRobotMessageText(text);
            }
            if (ini.ReadString(false, "SegmentKind", text) > 0)
            {
                row.segmentKind = DecodeRobotMessageText(text);
            }
            if (row.segmentKind.isEmpty())
            {
                row.segmentKind = DefaultPoseSegmentKind(segmentIndex);
            }

            ReadIniDouble(ini, "WeldZComp", row.weldZComp);
            ReadIniDouble(ini, "WeldGunDirComp", row.weldGunDirComp);
            ReadIniDouble(ini, "WeldSeamDirComp", row.weldSeamDirComp);
            m_seamRows.push_back(row);
        }
    }
    m_currentSeamGroupIndex = groupCount > 0 ? std::clamp(activeGroupIndex, 0, groupCount - 1) : 0;
    if (m_mode == CompMode::Seam)
    {
        m_currentGroupIndex = m_currentSeamGroupIndex;
    }
}

void WeldSeamCompDialog::SetMode(CompMode mode)
{
    if (m_mode == mode)
    {
        RefreshEditor();
        return;
    }

    QString error;
    StoreEditorValues(false, error);
    if (m_mode == CompMode::Pose)
    {
        m_currentPoseGroupIndex = m_currentGroupIndex;
    }
    else
    {
        m_currentSeamGroupIndex = m_currentGroupIndex;
    }
    m_mode = mode;
    m_currentGroupIndex = m_mode == CompMode::Pose ? m_currentPoseGroupIndex : m_currentSeamGroupIndex;
    m_currentTypeIndex = 0;
    if (m_pPoseModeBtn != nullptr)
    {
        m_pPoseModeBtn->setChecked(m_mode == CompMode::Pose);
    }
    if (m_pSeamModeBtn != nullptr)
    {
        m_pSeamModeBtn->setChecked(m_mode == CompMode::Seam);
    }
    RefreshGroupCombo();
    RefreshTypeCombo();
    RefreshEditor();
}

void WeldSeamCompDialog::RefreshGroupCombo()
{
    if (m_pGroupList == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(m_pGroupList);
    const QVector<QString>& groupNames = m_mode == CompMode::Pose ? m_poseGroupNames : m_seamGroupNames;
    const int groupCount = static_cast<int>(groupNames.size());
    const int previousIndex = groupCount <= 0 ? -1 : std::clamp(m_currentGroupIndex, 0, groupCount - 1);
    m_pGroupList->clear();
    for (int index = 0; index < groupNames.size(); ++index)
    {
        m_pGroupList->addItem(groupNames[index]);
    }

    if (m_pGroupList->count() > 0)
    {
        m_currentGroupIndex = std::clamp(previousIndex, 0, m_pGroupList->count() - 1);
        if (m_mode == CompMode::Pose)
        {
            m_currentPoseGroupIndex = m_currentGroupIndex;
            m_poseCompMatchMode = CurrentPoseGroupMatchMode();
        }
        else
        {
            m_currentSeamGroupIndex = m_currentGroupIndex;
        }
        m_pGroupList->setCurrentRow(m_currentGroupIndex);
    }
    else
    {
        m_currentGroupIndex = -1;
    }
}

void WeldSeamCompDialog::RefreshTypeCombo()
{
    if (m_pTypeCombo == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(m_pTypeCombo);
    const int rowCount = CurrentRowCount();
    const int previousIndex = rowCount > 0 ? std::clamp(m_currentTypeIndex, 0, rowCount - 1) : -1;
    m_pTypeCombo->clear();
    const int groupIndex = CurrentGroupCount() > 0 ? std::clamp(m_currentGroupIndex, 0, CurrentGroupCount() - 1) : -1;
    for (int segmentIndex = 0; segmentIndex < rowCount; ++segmentIndex)
    {
        const int flatIndex = groupIndex * COMP_SEGMENT_COUNT + segmentIndex;
        QString name;
        QString segmentKind;
        if (m_mode == CompMode::Pose && flatIndex >= 0 && flatIndex < m_poseRows.size())
        {
            name = m_poseRows[flatIndex].name;
            segmentKind = m_poseRows[flatIndex].segmentKind;
        }
        else if (m_mode == CompMode::Seam && flatIndex >= 0 && flatIndex < m_seamRows.size())
        {
            name = m_seamRows[flatIndex].name;
            segmentKind = m_seamRows[flatIndex].segmentKind;
        }
        if (name.isEmpty())
        {
            name = m_mode == CompMode::Pose ? DefaultPoseRowName(segmentIndex) : MakeDefaultSeamRow(groupIndex, segmentIndex).name;
        }
        if (segmentKind.isEmpty())
        {
            segmentKind = DefaultPoseSegmentKind(segmentIndex);
        }
        m_pTypeCombo->addItem(QString("%1 (%2)").arg(name, segmentKind), segmentKind);
    }

    if (m_pTypeCombo->count() > 0)
    {
        m_currentTypeIndex = std::clamp(previousIndex, 0, m_pTypeCombo->count() - 1);
        m_pTypeCombo->setCurrentIndex(m_currentTypeIndex);
    }
    else
    {
        m_currentTypeIndex = -1;
    }
}

void WeldSeamCompDialog::RefreshEditor()
{
    const int index = CurrentTypeIndex();
    m_currentTypeIndex = index;

    if (m_pPoseDisplayWidget != nullptr)
    {
        m_pPoseDisplayWidget->setVisible(m_mode == CompMode::Pose);
    }
    if (m_pPoseMatchModeLabel != nullptr)
    {
        m_pPoseMatchModeLabel->setVisible(m_mode == CompMode::Pose);
    }
    if (m_pPoseMatchModeCombo != nullptr)
    {
        const QSignalBlocker blocker(m_pPoseMatchModeCombo);
        m_pPoseMatchModeCombo->setVisible(m_mode == CompMode::Pose);
        const int mode = CurrentPoseGroupMatchMode();
        const int modeIndex = std::max(0, m_pPoseMatchModeCombo->findData(mode));
        m_pPoseMatchModeCombo->setCurrentIndex(modeIndex);
    }
    if (m_pCornerCompensationCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pCornerCompensationCheck);
        const bool hasPoseGroup = m_mode == CompMode::Pose
            && m_currentGroupIndex >= 0
            && m_currentGroupIndex < m_poseGroupCornerCompEnabled.size();
        m_pCornerCompensationCheck->setChecked(hasPoseGroup && m_poseGroupCornerCompEnabled[m_currentGroupIndex]);
    }

    if (m_pPathLabel != nullptr)
    {
        m_pPathLabel->setText(m_mode == CompMode::Pose
            ? DatabaseText("姿态补偿配置库")
            : DatabaseText("焊道补偿配置库"));
    }

    if (m_pHintLabel != nullptr)
    {
        if (m_mode == CompMode::Pose)
        {
            m_pHintLabel->setText(CurrentPoseGroupMatchMode() == POSE_COMP_MATCH_BY_SEGMENT_CODE
                ? QString("姿态补偿：按四类段属性直接匹配低平台、上升边、高平台、下降边，用于特殊件；补偿仍按当前点姿态旋转到世界坐标后叠加。")
                : QString("姿态补偿：按当前点姿态匹配下方姿态值，匹配成功后将 X/Y/Z 补偿按当前姿态旋转到世界坐标后叠加。"));
        }
        else
        {
            m_pHintLabel->setText("焊道补偿：Z 为世界 Z 向；枪反向为垂直 Z 轴和焊道方向的侧向；焊道方向为沿点序切线方向。");
        }
    }

    const bool hasRow = index >= 0;
    const bool hasRobot = !CurrentRobotName().isEmpty();
    if (m_pGroupList != nullptr)
    {
        m_pGroupList->setEnabled(hasRobot);
    }
    if (m_pTypeCombo != nullptr)
    {
        m_pTypeCombo->setEnabled(hasRow);
    }
    if (m_pNewGroupBtn != nullptr)
    {
        m_pNewGroupBtn->setEnabled(hasRobot);
    }
    if (m_pRenameGroupBtn != nullptr)
    {
        m_pRenameGroupBtn->setEnabled(hasRobot && hasRow);
    }
    if (m_pDeleteGroupBtn != nullptr)
    {
        m_pDeleteGroupBtn->setEnabled(hasRobot && hasRow);
    }
    for (int valueIndex = 0; valueIndex < 3; ++valueIndex)
    {
        if (m_pEditValues[valueIndex] != nullptr)
        {
            m_pEditValues[valueIndex]->setEnabled(hasRow);
        }
        if (m_pPoseValues[valueIndex] != nullptr)
        {
            m_pPoseValues[valueIndex]->setEnabled(hasRow && m_mode == CompMode::Pose);
        }
    }

    if (!hasRow)
    {
        if (m_pEditLabels[0] != nullptr) m_pEditLabels[0]->setText(m_mode == CompMode::Pose ? "X补偿(mm)：" : "Z向补偿(mm)：");
        if (m_pEditLabels[1] != nullptr) m_pEditLabels[1]->setText(m_mode == CompMode::Pose ? "Y补偿(mm)：" : "枪反向补偿(mm)：");
        if (m_pEditLabels[2] != nullptr) m_pEditLabels[2]->setText(m_mode == CompMode::Pose ? "Z补偿(mm)：" : "焊道方向补偿(mm)：");
        for (int valueIndex = 0; valueIndex < 3; ++valueIndex)
        {
            if (m_pEditValues[valueIndex] != nullptr) m_pEditValues[valueIndex]->clear();
            if (m_pPoseValues[valueIndex] != nullptr) m_pPoseValues[valueIndex]->clear();
        }
        for (QDoubleSpinBox* spin : m_pCornerCompensationValues)
        {
            if (spin != nullptr)
            {
                const QSignalBlocker blocker(spin);
                spin->setValue(0.0);
            }
        }
        RefreshCornerCompensationEditor();
        ScheduleCompPreview();
        return;
    }

    if (m_mode == CompMode::Pose)
    {
        const int flatIndex = CurrentFlatRowIndex();
        if (flatIndex >= 0 && flatIndex < m_poseRows.size())
        {
            const PoseCompRow& row = m_poseRows[flatIndex];
            if (m_pPoseValues[0] != nullptr) m_pPoseValues[0]->setText(FormatNumber(row.poseRx));
            if (m_pPoseValues[1] != nullptr) m_pPoseValues[1]->setText(FormatNumber(row.poseRy));
            if (m_pPoseValues[2] != nullptr) m_pPoseValues[2]->setText(FormatNumber(row.poseRz));
            if (m_pEditLabels[0] != nullptr) m_pEditLabels[0]->setText("X补偿(mm)：");
            if (m_pEditLabels[1] != nullptr) m_pEditLabels[1]->setText("Y补偿(mm)：");
            if (m_pEditLabels[2] != nullptr) m_pEditLabels[2]->setText("Z补偿(mm)：");
            if (m_pEditValues[0] != nullptr) m_pEditValues[0]->setText(FormatNumber(row.compX));
            if (m_pEditValues[1] != nullptr) m_pEditValues[1]->setText(FormatNumber(row.compY));
            if (m_pEditValues[2] != nullptr) m_pEditValues[2]->setText(FormatNumber(row.compZ));
            if (m_pCornerCompensationValues[0] != nullptr)
            {
                const QSignalBlocker blocker(m_pCornerCompensationValues[0]);
                m_pCornerCompensationValues[0]->setValue(row.innerToOuterCornerComp);
            }
            if (m_pCornerCompensationValues[1] != nullptr)
            {
                const QSignalBlocker blocker(m_pCornerCompensationValues[1]);
                m_pCornerCompensationValues[1]->setValue(row.innerToInnerCornerComp);
            }
            if (m_pCornerCompensationValues[2] != nullptr)
            {
                const QSignalBlocker blocker(m_pCornerCompensationValues[2]);
                m_pCornerCompensationValues[2]->setValue(row.outerToOuterCornerComp);
            }
            if (m_pCornerCompensationValues[3] != nullptr)
            {
                const QSignalBlocker blocker(m_pCornerCompensationValues[3]);
                m_pCornerCompensationValues[3]->setValue(row.outerToInnerCornerComp);
            }
        }
    }
    else
    {
        const int flatIndex = CurrentFlatRowIndex();
        if (flatIndex >= 0 && flatIndex < m_seamRows.size())
        {
            const SeamCompRow& row = m_seamRows[flatIndex];
            if (m_pEditLabels[0] != nullptr) m_pEditLabels[0]->setText("Z向补偿(mm)：");
            if (m_pEditLabels[1] != nullptr) m_pEditLabels[1]->setText("枪反向补偿(mm)：");
            if (m_pEditLabels[2] != nullptr) m_pEditLabels[2]->setText("焊道方向补偿(mm)：");
            if (m_pEditValues[0] != nullptr) m_pEditValues[0]->setText(FormatNumber(row.weldZComp));
            if (m_pEditValues[1] != nullptr) m_pEditValues[1]->setText(FormatNumber(row.weldGunDirComp));
            if (m_pEditValues[2] != nullptr) m_pEditValues[2]->setText(FormatNumber(row.weldSeamDirComp));
        }
    }
    RefreshCornerCompensationEditor();
    ScheduleCompPreview();
}

bool WeldSeamCompDialog::StoreEditorValues(bool validate, QString& error)
{
    const int index = CurrentTypeIndex();
    const int flatIndex = CurrentFlatRowIndex();
    if (index < 0 || flatIndex < 0)
    {
        return true;
    }

    double values[3] = { 0.0, 0.0, 0.0 };
    for (int valueIndex = 0; valueIndex < 3; ++valueIndex)
    {
        if (m_pEditValues[valueIndex] == nullptr)
        {
            continue;
        }
        bool ok = false;
        values[valueIndex] = m_pEditValues[valueIndex]->text().trimmed().toDouble(&ok);
        if (!ok || !std::isfinite(values[valueIndex]))
        {
            if (validate)
            {
                error = QString("第 %1 个补偿值不是有效数字。").arg(valueIndex + 1);
                return false;
            }
            return true;
        }
    }

    if (m_mode == CompMode::Pose)
    {
        if (m_currentGroupIndex >= 0 && m_currentGroupIndex < m_poseGroupCornerCompEnabled.size())
        {
            m_poseGroupCornerCompEnabled[m_currentGroupIndex] =
                m_pCornerCompensationCheck != nullptr && m_pCornerCompensationCheck->isChecked();
        }
        if (flatIndex >= m_poseRows.size())
        {
            return true;
        }
        double poseValues[3] = {
            m_poseRows[flatIndex].poseRx,
            m_poseRows[flatIndex].poseRy,
            m_poseRows[flatIndex].poseRz
        };
        const char* poseNames[3] = { "姿态RX", "姿态RY", "姿态RZ" };
        for (int valueIndex = 0; valueIndex < 3; ++valueIndex)
        {
            if (m_pPoseValues[valueIndex] == nullptr)
            {
                continue;
            }
            bool ok = false;
            poseValues[valueIndex] = m_pPoseValues[valueIndex]->text().trimmed().toDouble(&ok);
            if (!ok || !std::isfinite(poseValues[valueIndex]))
            {
                if (validate)
                {
                    error = QString("%1 不是有效数字。").arg(QString::fromUtf8(poseNames[valueIndex]));
                    return false;
                }
                return true;
            }
        }
        m_poseRows[flatIndex].poseRx = poseValues[0];
        m_poseRows[flatIndex].poseRy = poseValues[1];
        m_poseRows[flatIndex].poseRz = poseValues[2];
        m_poseRows[flatIndex].compX = values[0];
        m_poseRows[flatIndex].compY = values[1];
        m_poseRows[flatIndex].compZ = values[2];
        if (IsCornerCompensationSegmentKind(m_poseRows[flatIndex].segmentKind))
        {
            if (m_pCornerCompensationValues[0] != nullptr)
            {
                m_poseRows[flatIndex].innerToOuterCornerComp = m_pCornerCompensationValues[0]->value();
            }
            if (m_pCornerCompensationValues[1] != nullptr)
            {
                m_poseRows[flatIndex].innerToInnerCornerComp = m_pCornerCompensationValues[1]->value();
            }
            if (m_pCornerCompensationValues[2] != nullptr)
            {
                m_poseRows[flatIndex].outerToOuterCornerComp = m_pCornerCompensationValues[2]->value();
            }
            if (m_pCornerCompensationValues[3] != nullptr)
            {
                m_poseRows[flatIndex].outerToInnerCornerComp = m_pCornerCompensationValues[3]->value();
            }
        }
    }
    else
    {
        if (flatIndex >= m_seamRows.size())
        {
            return true;
        }
        m_seamRows[flatIndex].weldZComp = values[0];
        m_seamRows[flatIndex].weldGunDirComp = values[1];
        m_seamRows[flatIndex].weldSeamDirComp = values[2];
    }

    return true;
}

bool WeldSeamCompDialog::EditGroupName(QString& name, const QString& title)
{
    QDialog dialog(this);
    dialog.setWindowTitle(title);
    dialog.setStyleSheet(styleSheet());

    QFormLayout* formLayout = new QFormLayout(&dialog);
    formLayout->setContentsMargins(18, 16, 18, 16);
    formLayout->setSpacing(10);

    QLineEdit* nameEdit = new QLineEdit(name, &dialog);
    nameEdit->setMinimumWidth(260);
    formLayout->addRow("组名称：", nameEdit);

    QLabel* hintLabel = new QLabel("每个补偿组固定包含低平台、上升边、高平台、下降边四条数据。", &dialog);
    hintLabel->setWordWrap(true);
    formLayout->addRow(hintLabel);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    if (QPushButton* okButton = buttonBox->button(QDialogButtonBox::Ok))
    {
        okButton->setText("确定");
    }
    if (QPushButton* cancelButton = buttonBox->button(QDialogButtonBox::Cancel))
    {
        cancelButton->setText("取消");
    }
    formLayout->addRow(buttonBox);

    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    while (dialog.exec() == QDialog::Accepted)
    {
        const QString nextName = nameEdit->text().trimmed();
        if (nextName.isEmpty())
        {
            QMessageBox::warning(&dialog, title, "组名称不能为空。");
            continue;
        }

        name = nextName;
        return true;
    }

    return false;
}

void WeldSeamCompDialog::AddCurrentGroup()
{
    QString error;
    if (!StoreEditorValues(true, error))
    {
        QMessageBox::warning(this, "新建补偿组", error);
        return;
    }

    const int newGroupIndex = CurrentGroupCount();
    QString name = DefaultGroupName(newGroupIndex);

    if (!EditGroupName(name, QString("新建%1").arg(ModeGroupName())))
    {
        return;
    }

    if (m_mode == CompMode::Pose)
    {
        const int sourceMatchMode = CurrentPoseGroupMatchMode();
        m_poseGroupNames.push_back(name);
        m_poseGroupMatchModes.push_back(sourceMatchMode);
        const int sourceGroupIndex = CurrentGroupCount() > 1 ? std::clamp(m_currentGroupIndex, 0, CurrentGroupCount() - 2) : -1;
        const bool sourceCornerCompEnabled =
            sourceGroupIndex >= 0 && sourceGroupIndex < m_poseGroupCornerCompEnabled.size()
                ? m_poseGroupCornerCompEnabled[sourceGroupIndex]
                : false;
        m_poseGroupCornerCompEnabled.push_back(sourceCornerCompEnabled);
        for (int segmentIndex = 0; segmentIndex < COMP_SEGMENT_COUNT; ++segmentIndex)
        {
            PoseCompRow row = MakeDefaultPoseRow(newGroupIndex, segmentIndex);
            const int sourceFlatIndex = sourceGroupIndex >= 0 ? sourceGroupIndex * COMP_SEGMENT_COUNT + segmentIndex : -1;
            if (sourceFlatIndex >= 0 && sourceFlatIndex < m_poseRows.size())
            {
                row.poseRx = m_poseRows[sourceFlatIndex].poseRx;
                row.poseRy = m_poseRows[sourceFlatIndex].poseRy;
                row.poseRz = m_poseRows[sourceFlatIndex].poseRz;
                row.innerToOuterCornerComp = m_poseRows[sourceFlatIndex].innerToOuterCornerComp;
                row.innerToInnerCornerComp = m_poseRows[sourceFlatIndex].innerToInnerCornerComp;
                row.outerToOuterCornerComp = m_poseRows[sourceFlatIndex].outerToOuterCornerComp;
                row.outerToInnerCornerComp = m_poseRows[sourceFlatIndex].outerToInnerCornerComp;
            }
            m_poseRows.push_back(row);
        }
        m_currentPoseGroupIndex = newGroupIndex;
    }
    else
    {
        m_seamGroupNames.push_back(name);
        for (int segmentIndex = 0; segmentIndex < COMP_SEGMENT_COUNT; ++segmentIndex)
        {
            m_seamRows.push_back(MakeDefaultSeamRow(newGroupIndex, segmentIndex));
        }
        m_currentSeamGroupIndex = newGroupIndex;
    }

    m_currentGroupIndex = newGroupIndex;
    m_currentTypeIndex = 0;
    RefreshGroupCombo();
    RefreshTypeCombo();
    RefreshEditor();
    AppendLog(QString("已新建%1：%2，包含四条段类型数据。").arg(ModeGroupName(), name));
}

void WeldSeamCompDialog::RenameCurrentGroup()
{
    QString error;
    if (!StoreEditorValues(true, error))
    {
        QMessageBox::warning(this, "重命名补偿组", error);
        return;
    }

    const int index = CurrentGroupCount() > 0 ? std::clamp(m_currentGroupIndex, 0, CurrentGroupCount() - 1) : -1;
    if (index < 0)
    {
        return;
    }

    QString name;
    if (m_mode == CompMode::Pose)
    {
        if (index >= m_poseGroupNames.size())
        {
            return;
        }
        name = m_poseGroupNames[index];
    }
    else
    {
        if (index >= m_seamGroupNames.size())
        {
            return;
        }
        name = m_seamGroupNames[index];
    }

    if (!EditGroupName(name, QString("重命名%1").arg(ModeGroupName())))
    {
        return;
    }

    if (m_mode == CompMode::Pose)
    {
        m_poseGroupNames[index] = name;
    }
    else
    {
        m_seamGroupNames[index] = name;
    }

    m_currentGroupIndex = index;
    if (m_mode == CompMode::Pose)
    {
        m_currentPoseGroupIndex = index;
    }
    else
    {
        m_currentSeamGroupIndex = index;
    }
    RefreshGroupCombo();
    RefreshTypeCombo();
    RefreshEditor();
    AppendLog(QString("已重命名%1：%2。").arg(ModeGroupName(), name));
}

void WeldSeamCompDialog::DeleteCurrentGroup()
{
    QString error;
    if (!StoreEditorValues(true, error))
    {
        QMessageBox::warning(this, "删除补偿组", error);
        return;
    }

    const int index = CurrentGroupCount() > 0 ? std::clamp(m_currentGroupIndex, 0, CurrentGroupCount() - 1) : -1;
    if (index < 0)
    {
        return;
    }

    QString name;
    if (m_mode == CompMode::Pose)
    {
        if (index >= m_poseGroupNames.size())
        {
            return;
        }
        name = m_poseGroupNames[index];
    }
    else
    {
        if (index >= m_seamGroupNames.size())
        {
            return;
        }
        name = m_seamGroupNames[index];
    }

    const QString message = QString("确定删除%1“%2”吗？这会同时删除低平台、上升边、高平台、下降边四条数据。")
        .arg(ModeGroupName(), name);
    if (QMessageBox::question(this, "删除补偿组", message, QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    if (m_mode == CompMode::Pose)
    {
        m_poseGroupNames.removeAt(index);
        if (index < m_poseGroupMatchModes.size())
        {
            m_poseGroupMatchModes.removeAt(index);
        }
        if (index < m_poseGroupCornerCompEnabled.size())
        {
            m_poseGroupCornerCompEnabled.removeAt(index);
        }
        const int start = index * COMP_SEGMENT_COUNT;
        for (int segmentIndex = 0; segmentIndex < COMP_SEGMENT_COUNT && start < m_poseRows.size(); ++segmentIndex)
        {
            m_poseRows.removeAt(start);
        }
        for (int rowIndex = 0; rowIndex < m_poseRows.size(); ++rowIndex)
        {
            m_poseRows[rowIndex].sectionName = QString("WeldPoseComp%1").arg(rowIndex);
        }
        const int nextCount = m_poseGroupNames.size();
        m_currentPoseGroupIndex = nextCount > 0 ? std::min(index, nextCount - 1) : 0;
        m_currentGroupIndex = m_currentPoseGroupIndex;
    }
    else
    {
        m_seamGroupNames.removeAt(index);
        const int start = index * COMP_SEGMENT_COUNT;
        for (int segmentIndex = 0; segmentIndex < COMP_SEGMENT_COUNT && start < m_seamRows.size(); ++segmentIndex)
        {
            m_seamRows.removeAt(start);
        }
        for (int rowIndex = 0; rowIndex < m_seamRows.size(); ++rowIndex)
        {
            m_seamRows[rowIndex].sectionName = QString("WeldSeamComp%1").arg(rowIndex);
        }
        const int nextCount = m_seamGroupNames.size();
        m_currentSeamGroupIndex = nextCount > 0 ? std::min(index, nextCount - 1) : 0;
        m_currentGroupIndex = m_currentSeamGroupIndex;
    }

    m_currentTypeIndex = CurrentGroupCount() > 0 ? 0 : -1;
    RefreshGroupCombo();
    RefreshTypeCombo();
    RefreshEditor();
    AppendLog(QString("已删除%1：%2。").arg(ModeGroupName(), name));
}

bool WeldSeamCompDialog::SaveCurrentParam()
{
    QString error;
    if (!StoreEditorValues(true, error))
    {
        QMessageBox::warning(this, "保存补偿参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    // 保存前把旧补偿参数（本次编辑前已持久化的值）+ 旧的补偿结果轨迹备份进所选预览目录，便于回溯对比。
    if (!m_compPreviewDir.isEmpty())
    {
        const QString backupRoot = QDir(m_compPreviewDir).filePath(
            "CompBackup/" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
        QString backupSummary;
        BackupOldCompFiles(backupRoot, backupSummary);
        AppendLog(QString("已备份旧补偿数据 → %1（%2）").arg(QDir::toNativeSeparators(backupRoot), backupSummary));
    }

    if (!SavePoseParam(CurrentPoseParamPath(), error))
    {
        QMessageBox::warning(this, "保存补偿参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }
    if (!SaveSeamParam(CurrentSeamParamPath(), error))
    {
        QMessageBox::warning(this, "保存补偿参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }
    if (!SavePoseCornerCompensationToDatabase(CurrentRobotName(), error))
    {
        QMessageBox::warning(this, "保存补偿参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }
    MarkCleanSnapshot();
    AppendLog("姿态补偿参数和焊道补偿参数已保存。");
    QMessageBox::information(this, "保存补偿参数", "补偿参数保存完成。");
    return true;
}

bool WeldSeamCompDialog::SavePoseParam(const QString& path, QString& error) const
{
    if (path.isEmpty())
    {
        error = "姿态补偿配置库不可用。";
        return false;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldPoseComp");
    const int poseGroupCount = static_cast<int>(m_poseGroupNames.size());
    const int activeGroupIndex = poseGroupCount <= 0
        ? 0
        : std::clamp(m_currentPoseGroupIndex, 0, poseGroupCount - 1);
    const int activeMatchMode = poseGroupCount <= 0 || activeGroupIndex >= m_poseGroupMatchModes.size()
        ? NormalizePoseCompMatchMode(m_poseCompMatchMode)
        : NormalizePoseCompMatchMode(m_poseGroupMatchModes[activeGroupIndex]);
    if (!ini.WriteString("PoseCompCount", static_cast<int>(m_poseRows.size()))
        || !ini.WriteString(POSE_GROUP_COUNT_KEY, static_cast<int>(m_poseGroupNames.size()))
        || !ini.WriteString(POSE_ACTIVE_GROUP_INDEX_KEY, activeGroupIndex)
        || !ini.WriteString(POSE_COMP_MATCH_MODE_KEY, activeMatchMode)
        || !ini.WriteString("PoseMatchMaxErrorDeg", m_poseMatchMaxErrorDeg, 6))
    {
        error = "写入姿态补偿总配置失败。";
        return false;
    }

    for (int groupIndex = 0; groupIndex < m_poseGroupNames.size(); ++groupIndex)
    {
        ini.SetSectionName(LocalString(QString("WeldPoseCompGroup%1").arg(groupIndex)));
        const int groupMatchMode = groupIndex < m_poseGroupMatchModes.size()
            ? NormalizePoseCompMatchMode(m_poseGroupMatchModes[groupIndex])
            : activeMatchMode;
        if (!ini.WriteString("Name", LocalString(m_poseGroupNames[groupIndex]))
            || !ini.WriteString(POSE_COMP_MATCH_MODE_KEY, groupMatchMode))
        {
            error = QString("写入姿态补偿组失败：WeldPoseCompGroup%1").arg(groupIndex);
            return false;
        }
    }

    for (int index = 0; index < m_poseRows.size(); ++index)
    {
        const PoseCompRow& row = m_poseRows[index];
        ini.SetSectionName(LocalString(QString("WeldPoseComp%1").arg(index)));
        if (!ini.WriteString("GroupIndex", index / COMP_SEGMENT_COUNT)
            || !ini.WriteString("GroupName", LocalString(index / COMP_SEGMENT_COUNT < m_poseGroupNames.size() ? m_poseGroupNames[index / COMP_SEGMENT_COUNT] : QString()))
            || !ini.WriteString("Name", LocalString(row.name))
            || !ini.WriteString("SegmentKind", LocalString(row.segmentKind))
            || !ini.WriteString("Rx", row.poseRx, 6)
            || !ini.WriteString("Ry", row.poseRy, 6)
            || !ini.WriteString("Rz", row.poseRz, 6)
            || !ini.WriteString("CompX", row.compX, 6)
            || !ini.WriteString("CompY", row.compY, 6)
            || !ini.WriteString("CompZ", row.compZ, 6))
        {
            error = "写入姿态补偿槽失败：" + row.sectionName;
            return false;
        }
    }
    return true;
}

bool WeldSeamCompDialog::SavePoseCornerCompensationToDatabase(const QString& robotName, QString& error) const
{
    for (int groupIndex = 0; groupIndex < m_poseGroupNames.size(); ++groupIndex)
    {
        const bool enabled =
            groupIndex >= 0 && groupIndex < m_poseGroupCornerCompEnabled.size()
                ? m_poseGroupCornerCompEnabled[groupIndex]
                : false;
        if (!WriteRobotSetting(
            robotName,
            PoseCornerGroupModule(groupIndex),
            CORNER_COMP_ENABLED_KEY,
            enabled ? QStringLiteral("1") : QStringLiteral("0"),
            QStringLiteral("bool"),
            &error))
        {
            return false;
        }
    }

    for (int flatIndex = 0; flatIndex < m_poseRows.size(); ++flatIndex)
    {
        const PoseCompRow& row = m_poseRows[flatIndex];
        if (!IsCornerCompensationSegmentKind(row.segmentKind))
        {
            continue;
        }

        const QString module = PoseCornerSlotModule(flatIndex);
        const auto writeDouble = [&](const QString& key, double value)
        {
            return WriteRobotSetting(
                robotName,
                module,
                key,
                QString::number(value, 'f', 6),
                QStringLiteral("double"),
                &error);
        };
        if (!writeDouble(INNER_TO_OUTER_CORNER_COMP_KEY, row.innerToOuterCornerComp)
            || !writeDouble(INNER_TO_INNER_CORNER_COMP_KEY, row.innerToInnerCornerComp)
            || !writeDouble(OUTER_TO_OUTER_CORNER_COMP_KEY, row.outerToOuterCornerComp)
            || !writeDouble(OUTER_TO_INNER_CORNER_COMP_KEY, row.outerToInnerCornerComp))
        {
            return false;
        }
    }
    return true;
}

bool WeldSeamCompDialog::SaveSeamParam(const QString& path, QString& error) const
{
    if (path.isEmpty())
    {
        error = "焊道补偿配置库不可用。";
        return false;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldSeamComp");
    const int seamGroupCount = static_cast<int>(m_seamGroupNames.size());
    const int activeGroupIndex = seamGroupCount <= 0
        ? 0
        : std::clamp(m_currentSeamGroupIndex, 0, seamGroupCount - 1);
    if (!ini.WriteString("SeamCompCount", static_cast<int>(m_seamRows.size()))
        || !ini.WriteString(SEAM_GROUP_COUNT_KEY, static_cast<int>(m_seamGroupNames.size()))
        || !ini.WriteString(SEAM_ACTIVE_GROUP_INDEX_KEY, activeGroupIndex))
    {
        error = "写入焊道补偿总配置失败。";
        return false;
    }

    for (int groupIndex = 0; groupIndex < m_seamGroupNames.size(); ++groupIndex)
    {
        ini.SetSectionName(LocalString(QString("WeldSeamCompGroup%1").arg(groupIndex)));
        if (!ini.WriteString("Name", LocalString(m_seamGroupNames[groupIndex])))
        {
            error = QString("写入焊道补偿组失败：WeldSeamCompGroup%1").arg(groupIndex);
            return false;
        }
    }

    for (int index = 0; index < m_seamRows.size(); ++index)
    {
        const SeamCompRow& row = m_seamRows[index];
        ini.SetSectionName(LocalString(QString("WeldSeamComp%1").arg(index)));
        if (!ini.WriteString("GroupIndex", index / COMP_SEGMENT_COUNT)
            || !ini.WriteString("GroupName", LocalString(index / COMP_SEGMENT_COUNT < m_seamGroupNames.size() ? m_seamGroupNames[index / COMP_SEGMENT_COUNT] : QString()))
            || !ini.WriteString("Name", LocalString(row.name))
            || !ini.WriteString("SegmentKind", LocalString(row.segmentKind))
            || !ini.WriteString("WeldZComp", row.weldZComp, 6)
            || !ini.WriteString("WeldGunDirComp", row.weldGunDirComp, 6)
            || !ini.WriteString("WeldSeamDirComp", row.weldSeamDirComp, 6))
        {
            error = "写入焊道补偿槽失败：" + row.sectionName;
            return false;
        }
    }
    return true;
}

bool WeldSeamCompDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString WeldSeamCompDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << CurrentRobotName()
           << QString::number(NormalizePoseCompMatchMode(m_poseCompMatchMode))
           << QString::number(m_poseMatchMaxErrorDeg, 'f', 6)
           << QString::number(m_currentPoseGroupIndex)
           << QString::number(m_currentSeamGroupIndex);

    for (int index = 0; index < m_poseGroupNames.size(); ++index)
    {
        fields << "poseGroup"
               << m_poseGroupNames[index]
               << QString::number(index < m_poseGroupMatchModes.size()
                   ? NormalizePoseCompMatchMode(m_poseGroupMatchModes[index])
                   : NormalizePoseCompMatchMode(m_poseCompMatchMode))
               << QString::number(index < m_poseGroupCornerCompEnabled.size()
                   && m_poseGroupCornerCompEnabled[index] ? 1 : 0);
    }
    for (const PoseCompRow& row : m_poseRows)
    {
        fields << row.sectionName << row.name << row.segmentKind
               << QString::number(row.poseRx, 'f', 6)
               << QString::number(row.poseRy, 'f', 6)
               << QString::number(row.poseRz, 'f', 6)
               << QString::number(row.compX, 'f', 6)
               << QString::number(row.compY, 'f', 6)
               << QString::number(row.compZ, 'f', 6)
               << QString::number(row.innerToOuterCornerComp, 'f', 6)
               << QString::number(row.innerToInnerCornerComp, 'f', 6)
               << QString::number(row.outerToOuterCornerComp, 'f', 6)
               << QString::number(row.outerToInnerCornerComp, 'f', 6);
    }
    for (const QString& groupName : m_seamGroupNames)
    {
        fields << "seamGroup" << groupName;
    }
    for (const SeamCompRow& row : m_seamRows)
    {
        fields << row.sectionName << row.name << row.segmentKind
               << QString::number(row.weldZComp, 'f', 6)
               << QString::number(row.weldGunDirComp, 'f', 6)
               << QString::number(row.weldSeamDirComp, 'f', 6);
    }
    return fields.join('\n');
}

void WeldSeamCompDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void WeldSeamCompDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }
}

// ===================== 补偿前 / 补偿后 焊道可视化对比 =====================

QWidget* WeldSeamCompDialog::CreateCompPreviewPanel()
{
    QGroupBox* panel = new QGroupBox("补偿前 / 补偿后 焊道对比");
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 16, 10, 10);
    layout->setSpacing(8);

    QHBoxLayout* toolbar = new QHBoxLayout();
    toolbar->setSpacing(6);
    QPushButton* chooseBtn = new QPushButton("选择目录");
    m_pCompPreviewDirEdit = new QLineEdit();
    m_pCompPreviewDirEdit->setReadOnly(true);
    m_pCompPreviewDirEdit->setPlaceholderText("选择扫描结果 LaserPoint 目录…");
    QPushButton* topBtn = new QPushButton("俯视");
    QPushButton* frontBtn = new QPushButton("正视");
    QPushButton* camBtn = new QPushButton("相机");
    QPushButton* resetBtn = new QPushButton("重置");
    for (QPushButton* button : { topBtn, frontBtn, camBtn, resetBtn })
    {
        button->setMinimumWidth(54);
    }
    toolbar->addWidget(chooseBtn);
    toolbar->addWidget(m_pCompPreviewDirEdit, 1);
    toolbar->addWidget(topBtn);
    toolbar->addWidget(frontBtn);
    toolbar->addWidget(camBtn);
    toolbar->addWidget(resetBtn);
    layout->addLayout(toolbar);

    QHBoxLayout* layerToggleLayout = new QHBoxLayout();
    layerToggleLayout->setSpacing(14);
    m_pShowOriginalCheck = new QCheckBox("原始焊道");
    m_pShowBeforeCheck = new QCheckBox("补偿前");
    m_pShowAfterCheck = new QCheckBox("补偿后");
    m_pShowOriginalCheck->setChecked(false);
    m_pShowBeforeCheck->setChecked(true);
    m_pShowAfterCheck->setChecked(true);
    // 颜色图例：复选框文字用各图层实际颜色显示（灰=原始焊道 / 蓝=补偿前 / 橙=补偿后），与 3D 视图配色一致。
    m_pShowOriginalCheck->setStyleSheet("QCheckBox { color: #96A0AA; font-weight: bold; }");
    m_pShowBeforeCheck->setStyleSheet("QCheckBox { color: #78AFD7; font-weight: bold; }");
    m_pShowAfterCheck->setStyleSheet("QCheckBox { color: #FF9628; font-weight: bold; }");
    layerToggleLayout->addWidget(new QLabel("显示："));
    layerToggleLayout->addWidget(m_pShowOriginalCheck);
    layerToggleLayout->addWidget(m_pShowBeforeCheck);
    layerToggleLayout->addWidget(m_pShowAfterCheck);
    layerToggleLayout->addStretch(1);
    layout->addLayout(layerToggleLayout);

    m_pCompPreviewView = new pcview::PointCloud3DView();
    m_pCompPreviewView->setMinimumSize(360, 360);
    layout->addWidget(m_pCompPreviewView, 1);

    m_pCompPreviewInfoLabel = new QLabel("灰=原始焊道  蓝=补偿前  橙=补偿后；可勾选显示哪些图层，调整补偿值即可实时对比。");
    m_pCompPreviewInfoLabel->setWordWrap(true);
    m_pCompPreviewInfoLabel->setStyleSheet("color:#BFE8EC;");
    layout->addWidget(m_pCompPreviewInfoLabel);

    connect(chooseBtn, &QPushButton::clicked, this, [this]() { ChooseCompPreviewDirectory(); });
    connect(topBtn, &QPushButton::clicked, this, [this]() { if (m_pCompPreviewView != nullptr) m_pCompPreviewView->SetTopView(); });
    connect(frontBtn, &QPushButton::clicked, this, [this]() { if (m_pCompPreviewView != nullptr) m_pCompPreviewView->SetFrontView(); });
    connect(camBtn, &QPushButton::clicked, this, [this]() { if (m_pCompPreviewView != nullptr) m_pCompPreviewView->SetCameraLikeView(); });
    connect(resetBtn, &QPushButton::clicked, this, [this]() { if (m_pCompPreviewView != nullptr) m_pCompPreviewView->ResetView(); });
    for (QCheckBox* check : { m_pShowOriginalCheck, m_pShowBeforeCheck, m_pShowAfterCheck })
    {
        connect(check, &QCheckBox::toggled, this, [this](bool) { ApplyCompPreviewLayerVisibility(); });
    }

    return panel;
}

void WeldSeamCompDialog::ChooseCompPreviewDirectory()
{
    QString start = m_compPreviewDir;
    if (start.isEmpty())
    {
        start = RobotDataHelper::BuildProjectPath(QString("Result/%1").arg(CurrentRobotName()));
    }
    const QString dir = QFileDialog::getExistingDirectory(this, "选择扫描结果点云目录", start);
    if (dir.isEmpty())
    {
        return;
    }
    m_compPreviewDir = QDir::toNativeSeparators(dir);
    if (m_pCompPreviewDirEdit != nullptr)
    {
        m_pCompPreviewDirEdit->setText(m_compPreviewDir);
    }
    m_compPreviewBaseline.clear();
    m_compPreviewOriginal.clear();
    QString originalError;
    if (!m_compPreviewService.LoadCompPreviewOriginalTrack(m_compPreviewDir, m_compPreviewOriginal, originalError))
    {
        m_compPreviewOriginal.clear();
        AppendLog("原始焊道：" + originalError);
    }
    AppendLog(QString("补偿预览目录：%1").arg(m_compPreviewDir));
    RecomputeCompPreview();
}

void WeldSeamCompDialog::ScheduleCompPreview()
{
    if (m_bLoading)
    {
        return;
    }
    if (m_pCompPreviewTimer != nullptr)
    {
        m_pCompPreviewTimer->start();
    }
    else
    {
        RecomputeCompPreview();
    }
}

void WeldSeamCompDialog::ApplyCompPreviewLayerVisibility()
{
    if (m_pCompPreviewView == nullptr)
    {
        return;
    }
    // 图层顺序：0=原始焊道 1=补偿前 2=补偿后。
    m_pCompPreviewView->SetLayerVisible(0, m_pShowOriginalCheck != nullptr && m_pShowOriginalCheck->isChecked());
    m_pCompPreviewView->SetLayerVisible(1, m_pShowBeforeCheck == nullptr || m_pShowBeforeCheck->isChecked());
    m_pCompPreviewView->SetLayerVisible(2, m_pShowAfterCheck == nullptr || m_pShowAfterCheck->isChecked());
}

int WeldSeamCompDialog::CurrentRobotType() const
{
    // 品牌旋转合成（FANUC=Rz·Ry·Rx / STEP 反序）影响姿态补偿方向，按真实机型取。
    const QString robot = CurrentRobotName();
    if (m_pContralUnit != nullptr)
    {
        for (const T_CONTRAL_UNIT& unit : m_pContralUnit->m_vtContralUnitInfo)
        {
            if (QString::fromStdString(unit.sUnitName) != robot || unit.pUnitDriver == nullptr)
            {
                continue;
            }
            RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unit.pUnitDriver);
            if (dynamic_cast<STEPRobotCtrl*>(driver) != nullptr)
            {
                return ROBOT_TYPE_STEP;
            }
            if (dynamic_cast<FANUCRobotCtrl*>(driver) != nullptr)
            {
                return ROBOT_TYPE_FANUC;
            }
        }
    }
    return ROBOT_TYPE_FANUC;
}

MeasureThenWeldService::CompPreviewKind WeldSeamCompDialog::CurrentCompPreviewKind() const
{
    if (m_mode == CompMode::Seam)
    {
        return MeasureThenWeldService::CompPreviewKind::Seam;
    }
    if (m_pCornerCompensationCheck != nullptr && m_pCornerCompensationCheck->isChecked())
    {
        return MeasureThenWeldService::CompPreviewKind::Corner;
    }
    return MeasureThenWeldService::CompPreviewKind::Pose;
}

MeasureThenWeldService::CompPreviewEditValues WeldSeamCompDialog::CollectCompPreviewEditValues() const
{
    MeasureThenWeldService::CompPreviewEditValues edits;
    if (m_mode == CompMode::Seam)
    {
        const int base = m_currentSeamGroupIndex * COMP_SEGMENT_COUNT;
        for (int seg = 0; seg < COMP_SEGMENT_COUNT; ++seg)
        {
            const int flat = base + seg;
            if (flat < 0 || flat >= m_seamRows.size())
            {
                continue;
            }
            edits.weldZComp[seg] = m_seamRows[flat].weldZComp;
            edits.weldGunDirComp[seg] = m_seamRows[flat].weldGunDirComp;
            edits.weldSeamDirComp[seg] = m_seamRows[flat].weldSeamDirComp;
        }
    }
    else
    {
        const int base = m_currentPoseGroupIndex * COMP_SEGMENT_COUNT;
        for (int seg = 0; seg < COMP_SEGMENT_COUNT; ++seg)
        {
            const int flat = base + seg;
            if (flat < 0 || flat >= m_poseRows.size())
            {
                continue;
            }
            edits.poseRx[seg] = m_poseRows[flat].poseRx;
            edits.poseRy[seg] = m_poseRows[flat].poseRy;
            edits.poseRz[seg] = m_poseRows[flat].poseRz;
            edits.compX[seg] = m_poseRows[flat].compX;
            edits.compY[seg] = m_poseRows[flat].compY;
            edits.compZ[seg] = m_poseRows[flat].compZ;
        }
        edits.poseMatchMode = CurrentPoseGroupMatchMode();
        edits.poseMatchMaxErrorDeg = m_poseMatchMaxErrorDeg;
        edits.robotType = CurrentRobotType();
        edits.cornerEnabled = (m_pCornerCompensationCheck != nullptr && m_pCornerCompensationCheck->isChecked());
        const int risingFlat = base + 1;   // 上升边
        const int fallingFlat = base + 3;  // 下降边
        if (risingFlat >= 0 && risingFlat < m_poseRows.size())
        {
            edits.risingInnerToOuter = m_poseRows[risingFlat].innerToOuterCornerComp;
            edits.risingInnerToInner = m_poseRows[risingFlat].innerToInnerCornerComp;
            edits.risingOuterToOuter = m_poseRows[risingFlat].outerToOuterCornerComp;
            edits.risingOuterToInner = m_poseRows[risingFlat].outerToInnerCornerComp;
        }
        if (fallingFlat >= 0 && fallingFlat < m_poseRows.size())
        {
            edits.fallingInnerToOuter = m_poseRows[fallingFlat].innerToOuterCornerComp;
            edits.fallingInnerToInner = m_poseRows[fallingFlat].innerToInnerCornerComp;
            edits.fallingOuterToOuter = m_poseRows[fallingFlat].outerToOuterCornerComp;
            edits.fallingOuterToInner = m_poseRows[fallingFlat].outerToInnerCornerComp;
        }
    }
    return edits;
}

void WeldSeamCompDialog::RecomputeCompPreview()
{
    if (m_pCompPreviewView == nullptr)
    {
        return;
    }

    if (m_compPreviewDir.isEmpty())
    {
        m_pCompPreviewView->SetLayers({});
        m_pCompPreviewView->SetDirectionArrows({});
        if (m_pCompPreviewInfoLabel != nullptr)
        {
            m_pCompPreviewInfoLabel->setText("未选择目录。点\"选择目录\"载入扫描结果焊道后即可对比补偿。");
        }
        return;
    }

    QString storeError;
    StoreEditorValues(false, storeError);

    const MeasureThenWeldService::CompPreviewKind kind = CurrentCompPreviewKind();
    const bool usesKeyPoints = (kind == MeasureThenWeldService::CompPreviewKind::Corner);

    if (m_compPreviewBaseline.isEmpty()
        || m_compPreviewBaselineDir != m_compPreviewDir
        || m_compPreviewBaselineUsesKeyPoints != usesKeyPoints)
    {
        QString loadError;
        QVector<MeasureThenWeldService::CompPreviewPoint> baseline;
        if (!m_compPreviewService.LoadCompPreviewBaseline(kind, m_compPreviewDir, baseline, loadError))
        {
            m_compPreviewBaseline.clear();
            m_pCompPreviewView->SetLayers({});
            m_pCompPreviewView->SetDirectionArrows({});
            if (m_pCompPreviewInfoLabel != nullptr)
            {
                m_pCompPreviewInfoLabel->setText("读取失败：" + loadError);
            }
            return;
        }
        m_compPreviewBaseline = baseline;
        m_compPreviewBaselineDir = m_compPreviewDir;
        m_compPreviewBaselineUsesKeyPoints = usesKeyPoints;
    }

    const MeasureThenWeldService::CompPreviewEditValues edits = CollectCompPreviewEditValues();
    const MeasureThenWeldService::CompPreviewResult result =
        m_compPreviewService.RecomputeCompPreview(kind, CurrentRobotName(), m_compPreviewBaseline, edits);

    auto toLayerPoints = [](const QVector<MeasureThenWeldService::CompPreviewPoint>& points)
    {
        QVector<pcview::PointCloudVec3> out;
        out.reserve(points.size());
        for (const MeasureThenWeldService::CompPreviewPoint& point : points)
        {
            out.push_back({ point.x, point.y, point.z });
        }
        return out;
    };

    const bool showOriginal = m_pShowOriginalCheck != nullptr && m_pShowOriginalCheck->isChecked();
    const bool showBefore = m_pShowBeforeCheck == nullptr || m_pShowBeforeCheck->isChecked();
    const bool showAfter = m_pShowAfterCheck == nullptr || m_pShowAfterCheck->isChecked();

    // 固定图层顺序：0=原始焊道 1=补偿前 2=补偿后（与 ApplyCompPreviewLayerVisibility 对应）。
    QVector<pcview::PointCloud3DView::Layer> layers;
    pcview::PointCloud3DView::Layer originalLayer;
    originalLayer.name = "原始焊道";
    originalLayer.color = QColor(150, 160, 170);
    originalLayer.connectLines = true;
    originalLayer.visible = showOriginal;
    originalLayer.points = toLayerPoints(m_compPreviewOriginal);
    layers.push_back(originalLayer);

    pcview::PointCloud3DView::Layer beforeLayer;
    beforeLayer.name = "补偿前";
    beforeLayer.color = QColor(120, 175, 215);
    beforeLayer.connectLines = true;
    beforeLayer.visible = showBefore;
    beforeLayer.points = toLayerPoints(result.before);
    layers.push_back(beforeLayer);

    pcview::PointCloud3DView::Layer afterLayer;
    afterLayer.name = "补偿后";
    afterLayer.color = QColor(255, 150, 40);
    afterLayer.connectLines = true;
    afterLayer.visible = showAfter;
    afterLayer.points = toLayerPoints(result.after);
    layers.push_back(afterLayer);

    m_pCompPreviewView->SetLayers(layers);

    // 方向箭头：由 service 按补偿类型（焊缝/姿态/拐点）统一产出，这里只按 colorId 渲染。
    QVector<pcview::PointCloud3DView::DirectionArrow> arrows;
    arrows.reserve(result.arrows.size());
    for (const MeasureThenWeldService::CompPreviewArrow& source : result.arrows)
    {
        pcview::PointCloud3DView::DirectionArrow arrow;
        arrow.origin = { source.origin[0], source.origin[1], source.origin[2] };
        arrow.vector = { source.vector[0], source.vector[1], source.vector[2] };
        arrow.label = source.label;
        arrow.doubleHeaded = source.doubleHeaded;
        switch (source.colorId)
        {
        case 1: arrow.color = QColor(255, 215, 64); break;   // 枪反向
        case 2: arrow.color = QColor(120, 255, 150); break;  // 焊道方向
        case 3: arrow.color = QColor(255, 90, 90); break;    // 工具X
        case 4: arrow.color = QColor(90, 255, 90); break;    // 工具Y
        case 5: arrow.color = QColor(90, 200, 255); break;   // 工具Z
        case 6: arrow.color = QColor(255, 150, 40); break;   // 拐点位移
        default: arrow.color = QColor(90, 200, 255); break;  // Z向
        }
        arrows.push_back(arrow);
    }
    m_pCompPreviewView->SetDirectionArrows(arrows);

    if (m_pCompPreviewInfoLabel != nullptr)
    {
        QString info = QString("补偿前 %1 点 / 补偿后 %2 点").arg(result.before.size()).arg(result.after.size());
        if (!result.ok && !result.error.isEmpty())
        {
            info += "（" + result.error + "）";
        }
        m_pCompPreviewInfoLabel->setText(info);
    }
}

void WeldSeamCompDialog::BackupOldCompFiles(const QString& backupRoot, QString& summary) const
{
    summary.clear();
    QDir().mkpath(backupRoot);
    QStringList done;

    // 1. 旧补偿参数快照（m_cleanSnapshot = 本次编辑前已持久化的姿态/焊道/拐点补偿参数）。
    const QString snapPath = QDir(backupRoot).filePath("CompParamSnapshot_old.txt");
    QFile snapFile(snapPath);
    if (snapFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&snapFile);
        out.setEncoding(QStringConverter::Utf8);
        out << "# 机器人：" << CurrentRobotName() << "\n";
        out << "# 备份时间：" << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") << "\n";
        out << "# 内容：本次保存前已持久化的补偿参数（姿态/焊道/拐点）\n";
        out << m_cleanSnapshot;
        snapFile.close();
        done << QStringLiteral("旧补偿参数快照");
    }

    // 2. 复制旧的补偿前/后结果轨迹文件。
    if (!m_compPreviewDir.isEmpty())
    {
        const QStringList names = {
            QStringLiteral("PreciseLaserPoint_WeldPose_2mm.txt"),
            QStringLiteral("PreciseLaserPoint_WeldPose_2mm_SeamComp.txt"),
            QStringLiteral("PreciseLaserPoint_Classified.txt"),
            QStringLiteral("PreciseLaserPoint_CornerComp_Classified.txt"),
            QStringLiteral("PreciseLaserPoint_KeyPoints.txt")
        };
        int copied = 0;
        for (const QString& name : names)
        {
            const QString src = QDir(m_compPreviewDir).filePath(name);
            if (QFileInfo::exists(src))
            {
                const QString dst = QDir(backupRoot).filePath(name);
                QFile::remove(dst);
                if (QFile::copy(src, dst))
                {
                    ++copied;
                }
            }
        }
        if (copied > 0)
        {
            done << QString("旧轨迹文件 %1 个").arg(copied);
        }
    }

    summary = done.isEmpty() ? QStringLiteral("无可备份内容") : done.join(QStringLiteral("，"));
}
