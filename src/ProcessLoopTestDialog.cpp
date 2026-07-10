#include "ProcessLoopTestDialog.h"

#include "ConfigDatabase.h"
#include "ContralUnit.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"
#include "WeldProcessFile.h"
#include "WindowStyleHelper.h"

#include <algorithm>

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDateTime>
#include <QScrollArea>
#include <QVBoxLayout>

namespace
{
	// 测试设置持久化：ConfigDatabase 全局域 ProcessLoopTest 模块（UI 线程读写）。
	QString ReadTestSetting(const QString& key, const QString& defaultValue)
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(),
			QStringLiteral("ProcessLoopTest"), key, &value) || value.trimmed().isEmpty())
		{
			return defaultValue;
		}
		return value.trimmed();
	}

	void WriteTestSetting(const QString& key, const QString& value)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(),
			QStringLiteral("ProcessLoopTest"), key, value);
	}
}

ProcessLoopTestDialog::ProcessLoopTestDialog(ContralUnit* pContralUnit,
    int defaultUnitIndex,
    LoadDefaultsFunc loadDefaults,
    RunnerFunc runner,
    QWidget* parent)
    : QDialog(parent)
    , m_loadDefaults(std::move(loadDefaults))
    , m_runner(std::move(runner))
    , m_pContralUnit(pContralUnit)
{
    setWindowTitle(QStringLiteral("流程测试"));
    BuildUi();

    // 预选默认单元（LoadSettings 里保存过的单元会覆盖它）。
    if (m_unitCombo != nullptr)
    {
        const int idx = m_unitCombo->findData(defaultUnitIndex);
        if (idx >= 0)
        {
            m_unitCombo->setCurrentIndex(idx);
        }
    }
    LoadSettings();
    ReloadSelectorsForCurrentUnit();
    ReloadDefaultsForCurrentUnit();
}

ProcessLoopTestDialog::~ProcessLoopTestDialog()
{
    m_stopRequested.store(true);
    JoinWorker();
}

void ProcessLoopTestDialog::BuildUi()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    auto* title = new QLabel(QStringLiteral("流程测试（自动循环先测后焊）"), this);
    title->setStyleSheet("font-size: 18px; font-weight: bold; color: #9ED8DB;");
    root->addWidget(title);

    auto* hint = new QLabel(QStringLiteral(
        "循环执行先测后焊流程（下枪安全位 → 扫描起点 → 扫描采集 → 收枪安全位 →〔可选〕焊接），用于重复性/稳定性验证。"
        "参数默认取「测量焊接参数」里已设好的预设值，勾选下方「覆盖」的项才用界面值替换。"
        "默认仅扫描；勾选「包含焊接段」才执行焊接。"), this);
    hint->setWordWrap(true);
    hint->setStyleSheet("color: #7E9AA6; font-size: 12px;");
    root->addWidget(hint);

    // 小屏/平板分辨率下内容放不下时滚动而不是挤压重叠：设置与日志全部放进自适应滚动区
    //（PrepareEmbeddedPage 会对页内 QScrollArea 追加响应式配置，与其它管理页一致）。
    auto* scrollArea = new QScrollArea(this);
    scrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(scrollArea);
    auto* content = new QWidget(scrollArea);
    content->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(0, 0, 8, 0);
    contentLayout->setSpacing(12);
    contentLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    // —— 采集设置 ——
    auto* collectBox = new QGroupBox(QStringLiteral("测试设置"), this);
    auto* collectForm = new QFormLayout(collectBox);
    collectForm->setLabelAlignment(Qt::AlignRight);

    m_unitCombo = new QComboBox(collectBox);
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        const int row = m_unitCombo->count();
        m_unitCombo->addItem(info.displayName, info.unitIndex);
        m_unitCombo->setItemData(row, info.robotName, Qt::UserRole + 1);
    }
    connect(m_unitCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int)
        {
            ReloadSelectorsForCurrentUnit();
            ReloadDefaultsForCurrentUnit();
        });
    collectForm->addRow(QStringLiteral("目标机器人："), m_unitCombo);

    auto* countRow = new QHBoxLayout();
    m_repeatSpin = new QSpinBox(collectBox);
    m_repeatSpin->setRange(1, 100000);
    m_repeatSpin->setValue(10);
    m_repeatSpin->setSuffix(QStringLiteral(" 次"));
    countRow->addWidget(m_repeatSpin);
    m_infiniteCheck = new QCheckBox(QStringLiteral("持续循环（手动停止）"), collectBox);
    connect(m_infiniteCheck, &QCheckBox::toggled, this, [this](bool on) { m_repeatSpin->setDisabled(on); });
    countRow->addWidget(m_infiniteCheck);
    countRow->addStretch();
    collectForm->addRow(QStringLiteral("循环次数："), countRow);

    m_stopOnFailureCheck = new QCheckBox(QStringLiteral("某次失败即停止（推荐：机器人异常时不硬跑）"), collectBox);
    m_stopOnFailureCheck->setChecked(true);
    collectForm->addRow(QString(), m_stopOnFailureCheck);

    m_doWeldCheck = new QCheckBox(QStringLiteral("包含焊接段（不勾=仅扫描；勾选后每次扫描完成即执行焊接，实际焊接/空跑按预设）"), collectBox);
    m_doWeldCheck->setChecked(false);
    collectForm->addRow(QString(), m_doWeldCheck);

    // 定时扫描：两次扫描之间的等待间隔。0=连续；配合「持续循环」= 每隔 X 分钟自动扫一次。
    auto* intervalRow = new QHBoxLayout();
    m_intervalSpin = new QSpinBox(collectBox);
    m_intervalSpin->setRange(0, 100000);
    m_intervalSpin->setValue(0);
    intervalRow->addWidget(m_intervalSpin);
    m_intervalUnitCombo = new QComboBox(collectBox);
    m_intervalUnitCombo->addItem(QStringLiteral("秒"), 1);
    m_intervalUnitCombo->addItem(QStringLiteral("分钟"), 60);
    intervalRow->addWidget(m_intervalUnitCombo);
    auto* intervalHint = new QLabel(QStringLiteral("（0=连续不等待；每次扫描完成后等待此时间再开始下一次）"), collectBox);
    intervalHint->setStyleSheet("color:#7E9AA6; font-size:11px;");
    intervalHint->setWordWrap(true);
    intervalRow->addWidget(intervalHint, 1);
    collectForm->addRow(QStringLiteral("扫描间隔："), intervalRow);
    contentLayout->addWidget(collectBox);

    // —— 参数覆盖 ——
    auto* paramBox = new QGroupBox(QStringLiteral("参数（与先测后焊共用「当前选用」，改选即生效；勾选「覆盖」才替换数值）"), this);
    auto* paramLayout = new QVBoxLayout(paramBox);

    // 与先测后焊一致的四个选择器：位置类型 / 焊接工艺 / 姿态补偿组 / 焊道补偿组。
    // 改选即写回「当前选用」（UseGroupNo / UseWeldParaNo / Active*CompGroupIndex），
    // 循环里 LoadPresetParam 读的就是这些——两边界面看到的选择始终一致。
    // 2×2 网格：标签右对齐、四个下拉两列对齐等宽，避免错落。
    auto* selectorGrid = new QGridLayout();
    selectorGrid->setHorizontalSpacing(12);
    selectorGrid->setVerticalSpacing(8);
    m_paramGroupCombo = new QComboBox(paramBox);
    m_processCombo = new QComboBox(paramBox);
    m_poseCompCombo = new QComboBox(paramBox);
    m_seamCompCombo = new QComboBox(paramBox);
    for (QComboBox* c : { m_paramGroupCombo, m_processCombo, m_poseCompCombo, m_seamCompCombo })
    {
        c->setFixedWidth(300);   // 固定宽度：宽屏下不被拉长
    }
    auto addSelector = [selectorGrid, paramBox](int row, int col, const QString& text, QComboBox* combo)
        {
            QLabel* lab = new QLabel(text, paramBox);
            lab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            selectorGrid->addWidget(lab, row, col * 2);
            selectorGrid->addWidget(combo, row, col * 2 + 1);
        };
    addSelector(0, 0, QStringLiteral("位置类型："), m_paramGroupCombo);
    addSelector(0, 1, QStringLiteral("焊接工艺："), m_processCombo);
    addSelector(1, 0, QStringLiteral("姿态补偿组："), m_poseCompCombo);
    addSelector(1, 1, QStringLiteral("焊道补偿组："), m_seamCompCombo);
    selectorGrid->setColumnStretch(4, 1);   // 余量全给右侧空列，四个下拉整体靠左
    paramLayout->addLayout(selectorGrid);

    connect(m_paramGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_loadingSelectors || m_running.load())
            {
                return;
            }
            QString error;
            const QString robot = CurrentRobotName();
            if (robot.isEmpty() || !RobotDataHelper::WriteParamValue(
                RobotDataHelper::MeasureWeldParamPath(robot),
                QStringLiteral("MeasureWeldGroups"), QStringLiteral("UseGroupNo"),
                QString::number(std::max(0, m_paramGroupCombo->currentData().toInt())), &error))
            {
                AppendLog(QStringLiteral("切换位置类型失败：%1").arg(error));
                return;
            }
            AppendLog(QStringLiteral("位置类型已切换为：%1").arg(m_paramGroupCombo->currentText()));
            ReloadDefaultsForCurrentUnit();
        });
    connect(m_processCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_loadingSelectors || m_running.load() || m_pContralUnit == nullptr)
            {
                return;
            }
            const int unitIndex = m_unitCombo != nullptr ? m_unitCombo->currentData().toInt() : -1;
            const int processIndex = m_processCombo->currentData().toInt();
            if (unitIndex < 0 || processIndex < 0)
            {
                return;
            }
            WeldProcessFile processFile(*m_pContralUnit, unitIndex);
            if (!processFile.Init() || !processFile.UpdateUseWeldParaNo(processIndex))
            {
                AppendLog(QStringLiteral("切换焊接工艺失败：%1")
                    .arg(DecodeRobotMessageText(processFile.GetLastError())));
                return;
            }
            AppendLog(QStringLiteral("焊接工艺已切换为：%1").arg(m_processCombo->currentText()));
        });
    auto saveCompSelection = [this](QComboBox* combo, const QString& fileName,
        const QString& allSection, const QString& activeKey, const QString& what)
        {
            if (m_loadingSelectors || m_running.load() || combo == nullptr)
            {
                return;
            }
            const QString robot = CurrentRobotName();
            if (robot.isEmpty())
            {
                return;
            }
            QString error;
            if (!RobotDataHelper::WriteParamValue(
                RobotDataHelper::BuildProjectPath(QStringLiteral("Data/%1/%2").arg(robot, fileName)),
                allSection, activeKey,
                QString::number(std::max(0, combo->currentData().toInt())), &error))
            {
                AppendLog(QStringLiteral("切换%1失败：%2").arg(what, error));
                return;
            }
            AppendLog(QStringLiteral("%1已切换为：%2").arg(what, combo->currentText()));
        };
    connect(m_poseCompCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this, saveCompSelection](int)
        {
            saveCompSelection(m_poseCompCombo, QStringLiteral("WeldPoseCompParam.ini"),
                QStringLiteral("ALLWeldPoseComp"), QStringLiteral("ActivePoseCompGroupIndex"), QStringLiteral("姿态补偿组"));
        });
    connect(m_seamCompCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
        [this, saveCompSelection](int)
        {
            saveCompSelection(m_seamCompCombo, QStringLiteral("WeldSeamCompParam.ini"),
                QStringLiteral("ALLWeldSeamComp"), QStringLiteral("ActiveSeamCompGroupIndex"), QStringLiteral("焊道补偿组"));
        });

    m_defaultsLabel = new QLabel(QStringLiteral("预设：加载中…"), paramBox);
    m_defaultsLabel->setStyleSheet("color: #8FB0BC; font-size: 12px;");
    paramLayout->addWidget(m_defaultsLabel);

    auto addOverrideRow = [paramBox, paramLayout](const QString& label, QCheckBox*& check,
        QDoubleSpinBox*& spin, double minV, double maxV, int decimals, const QString& suffix)
        {
            auto* row = new QHBoxLayout();
            check = new QCheckBox(QStringLiteral("覆盖"), paramBox);
            row->addWidget(check);
            auto* lab = new QLabel(label, paramBox);
            lab->setMinimumWidth(120);
            row->addWidget(lab);
            spin = new QDoubleSpinBox(paramBox);
            spin->setRange(minV, maxV);
            spin->setDecimals(decimals);
            spin->setSuffix(suffix);
            spin->setEnabled(false);
            row->addWidget(spin);
            row->addStretch();
            QObject::connect(check, &QCheckBox::toggled, spin, &QWidget::setEnabled);
            paramLayout->addLayout(row);
        };
    addOverrideRow(QStringLiteral("扫描速度"), m_ovScanSpeedCheck, m_scanSpeedSpin, 1.0, 100000.0, 1, QStringLiteral(" mm/min"));
    addOverrideRow(QStringLiteral("运行速度"), m_ovRunSpeedCheck, m_runSpeedSpin, 1.0, 100000.0, 1, QStringLiteral(" mm/min"));
    addOverrideRow(QStringLiteral("相机时间偏移"), m_ovCameraOffsetCheck, m_cameraOffsetSpin, -10000.0, 10000.0, 1, QStringLiteral(" ms"));
    contentLayout->addWidget(paramBox);

    // —— 控制/进度 ——
    auto* ctrlRow = new QHBoxLayout();
    m_startBtn = new QPushButton(QStringLiteral("开始测试"), this);
    m_startBtn->setMinimumHeight(42);
    m_startBtn->setStyleSheet("QPushButton { background:#1F5A46; color:#EAFBF3; border:1px solid #3E8E70;"
        "border-radius:8px; font-size:15px; font-weight:600; padding:6px 18px; }"
        "QPushButton:hover { background:#2A7358; } QPushButton:disabled { background:#2A3A36; color:#6E8880; }");
    connect(m_startBtn, &QPushButton::clicked, this, &ProcessLoopTestDialog::OnStart);
    ctrlRow->addWidget(m_startBtn);

    m_stopBtn = new QPushButton(QStringLiteral("停止"), this);
    m_stopBtn->setMinimumHeight(42);
    m_stopBtn->setEnabled(false);
    m_stopBtn->setStyleSheet("QPushButton { background:#5A2323; color:#FBEAEA; border:1px solid #8E3E3E;"
        "border-radius:8px; font-size:15px; font-weight:600; padding:6px 18px; }"
        "QPushButton:hover { background:#733030; } QPushButton:disabled { background:#3A2A2A; color:#886E6E; }");
    connect(m_stopBtn, &QPushButton::clicked, this, &ProcessLoopTestDialog::OnStop);
    ctrlRow->addWidget(m_stopBtn);
    ctrlRow->addStretch();
    contentLayout->addLayout(ctrlRow);

    m_progressLabel = new QLabel(QStringLiteral("就绪。"), this);
    m_progressLabel->setStyleSheet("font-size: 14px; color: #D6E7EA; font-weight: 600;");
    m_progressLabel->setWordWrap(true);
    contentLayout->addWidget(m_progressLabel);

    m_statLabel = new QLabel(QStringLiteral("成功 0 · 失败 0"), this);
    m_statLabel->setStyleSheet("color: #8FB0BC; font-size: 12px;");
    contentLayout->addWidget(m_statLabel);

    m_logText = new QPlainTextEdit(this);
    m_logText->setReadOnly(true);
    m_logText->setMinimumHeight(160);
    m_logText->setStyleSheet("QPlainTextEdit { background:#0E1A1F; color:#CFE3E6; border:1px solid #294049;"
        "border-radius:6px; font-family: Consolas, monospace; font-size: 12px; }");
    contentLayout->addWidget(m_logText, 1);

    scrollArea->setWidget(content);
    root->addWidget(scrollArea, 1);

    resize(720, 640);
}

void ProcessLoopTestDialog::LoadSettings()
{
    if (m_unitCombo != nullptr)
    {
        bool ok = false;
        const int savedUnit = ReadTestSetting(QStringLiteral("UnitIndex"), QStringLiteral("-1")).toInt(&ok);
        if (ok && savedUnit >= 0)
        {
            const int idx = m_unitCombo->findData(savedUnit);
            if (idx >= 0) { m_unitCombo->setCurrentIndex(idx); }
        }
    }
    if (m_repeatSpin != nullptr)
    {
        m_repeatSpin->setValue(ReadTestSetting(QStringLiteral("RepeatCount"), QStringLiteral("10")).toInt());
    }
    if (m_infiniteCheck != nullptr)
    {
        m_infiniteCheck->setChecked(ReadTestSetting(QStringLiteral("Infinite"), QStringLiteral("0")) == QStringLiteral("1"));
    }
    if (m_stopOnFailureCheck != nullptr)
    {
        m_stopOnFailureCheck->setChecked(ReadTestSetting(QStringLiteral("StopOnFailure"), QStringLiteral("1")) == QStringLiteral("1"));
    }
    if (m_doWeldCheck != nullptr)
    {
        m_doWeldCheck->setChecked(ReadTestSetting(QStringLiteral("DoWeld"), QStringLiteral("0")) == QStringLiteral("1"));
    }
    if (m_intervalSpin != nullptr)
    {
        m_intervalSpin->setValue(ReadTestSetting(QStringLiteral("IntervalValue"), QStringLiteral("0")).toInt());
    }
    if (m_intervalUnitCombo != nullptr)
    {
        const int mul = ReadTestSetting(QStringLiteral("IntervalUnit"), QStringLiteral("1")).toInt();
        const int idx = m_intervalUnitCombo->findData(mul);
        if (idx >= 0) { m_intervalUnitCombo->setCurrentIndex(idx); }
    }
    // 覆盖项：勾选状态 + 值都恢复；未勾选的值随后由 ReloadDefaultsForCurrentUnit 用预设回填。
    if (m_ovScanSpeedCheck != nullptr)
    {
        m_ovScanSpeedCheck->setChecked(ReadTestSetting(QStringLiteral("OvScanSpeed"), QStringLiteral("0")) == QStringLiteral("1"));
        if (m_scanSpeedSpin != nullptr && m_ovScanSpeedCheck->isChecked())
        {
            m_scanSpeedSpin->setValue(ReadTestSetting(QStringLiteral("ScanSpeed"), QStringLiteral("0")).toDouble());
        }
    }
    if (m_ovRunSpeedCheck != nullptr)
    {
        m_ovRunSpeedCheck->setChecked(ReadTestSetting(QStringLiteral("OvRunSpeed"), QStringLiteral("0")) == QStringLiteral("1"));
        if (m_runSpeedSpin != nullptr && m_ovRunSpeedCheck->isChecked())
        {
            m_runSpeedSpin->setValue(ReadTestSetting(QStringLiteral("RunSpeed"), QStringLiteral("0")).toDouble());
        }
    }
    if (m_ovCameraOffsetCheck != nullptr)
    {
        m_ovCameraOffsetCheck->setChecked(ReadTestSetting(QStringLiteral("OvCameraOffset"), QStringLiteral("0")) == QStringLiteral("1"));
        if (m_cameraOffsetSpin != nullptr && m_ovCameraOffsetCheck->isChecked())
        {
            m_cameraOffsetSpin->setValue(ReadTestSetting(QStringLiteral("CameraOffset"), QStringLiteral("0")).toDouble());
        }
    }
}

void ProcessLoopTestDialog::SaveSettings() const
{
    if (m_unitCombo != nullptr)
    {
        WriteTestSetting(QStringLiteral("UnitIndex"), QString::number(m_unitCombo->currentData().toInt()));
    }
    if (m_repeatSpin != nullptr)
    {
        WriteTestSetting(QStringLiteral("RepeatCount"), QString::number(m_repeatSpin->value()));
    }
    if (m_infiniteCheck != nullptr)
    {
        WriteTestSetting(QStringLiteral("Infinite"), m_infiniteCheck->isChecked() ? QStringLiteral("1") : QStringLiteral("0"));
    }
    if (m_stopOnFailureCheck != nullptr)
    {
        WriteTestSetting(QStringLiteral("StopOnFailure"), m_stopOnFailureCheck->isChecked() ? QStringLiteral("1") : QStringLiteral("0"));
    }
    if (m_doWeldCheck != nullptr)
    {
        WriteTestSetting(QStringLiteral("DoWeld"), m_doWeldCheck->isChecked() ? QStringLiteral("1") : QStringLiteral("0"));
    }
    if (m_intervalSpin != nullptr)
    {
        WriteTestSetting(QStringLiteral("IntervalValue"), QString::number(m_intervalSpin->value()));
    }
    if (m_intervalUnitCombo != nullptr)
    {
        WriteTestSetting(QStringLiteral("IntervalUnit"), QString::number(m_intervalUnitCombo->currentData().toInt()));
    }
    if (m_ovScanSpeedCheck != nullptr && m_scanSpeedSpin != nullptr)
    {
        WriteTestSetting(QStringLiteral("OvScanSpeed"), m_ovScanSpeedCheck->isChecked() ? QStringLiteral("1") : QStringLiteral("0"));
        WriteTestSetting(QStringLiteral("ScanSpeed"), QString::number(m_scanSpeedSpin->value(), 'f', 1));
    }
    if (m_ovRunSpeedCheck != nullptr && m_runSpeedSpin != nullptr)
    {
        WriteTestSetting(QStringLiteral("OvRunSpeed"), m_ovRunSpeedCheck->isChecked() ? QStringLiteral("1") : QStringLiteral("0"));
        WriteTestSetting(QStringLiteral("RunSpeed"), QString::number(m_runSpeedSpin->value(), 'f', 1));
    }
    if (m_ovCameraOffsetCheck != nullptr && m_cameraOffsetSpin != nullptr)
    {
        WriteTestSetting(QStringLiteral("OvCameraOffset"), m_ovCameraOffsetCheck->isChecked() ? QStringLiteral("1") : QStringLiteral("0"));
        WriteTestSetting(QStringLiteral("CameraOffset"), QString::number(m_cameraOffsetSpin->value(), 'f', 1));
    }
}

QString ProcessLoopTestDialog::CurrentRobotName() const
{
    if (m_unitCombo != nullptr && m_unitCombo->currentIndex() >= 0)
    {
        const QString robotName = m_unitCombo->currentData(Qt::UserRole + 1).toString().trimmed();
        if (!robotName.isEmpty())
        {
            return robotName;
        }
    }
    return QString();
}

void ProcessLoopTestDialog::ReloadSelectorsForCurrentUnit()
{
    m_loadingSelectors = true;
    LoadParamGroupCombo();
    LoadProcessCombo();
    const QString robot = CurrentRobotName();
    LoadCompCombo(m_poseCompCombo,
        RobotDataHelper::BuildProjectPath(QStringLiteral("Data/%1/WeldPoseCompParam.ini").arg(robot)),
        QStringLiteral("ALLWeldPoseComp"), QStringLiteral("PoseCompCount"),
        QStringLiteral("PoseCompGroupCount"), QStringLiteral("ActivePoseCompGroupIndex"),
        QStringLiteral("WeldPoseCompGroup"), QStringLiteral("姿态补偿组"));
    LoadCompCombo(m_seamCompCombo,
        RobotDataHelper::BuildProjectPath(QStringLiteral("Data/%1/WeldSeamCompParam.ini").arg(robot)),
        QStringLiteral("ALLWeldSeamComp"), QStringLiteral("SeamCompCount"),
        QStringLiteral("SeamCompGroupCount"), QStringLiteral("ActiveSeamCompGroupIndex"),
        QStringLiteral("WeldSeamCompGroup"), QStringLiteral("焊道补偿组"));
    m_loadingSelectors = false;
}

void ProcessLoopTestDialog::LoadParamGroupCombo()
{
    if (m_paramGroupCombo == nullptr)
    {
        return;
    }
    m_paramGroupCombo->clear();
    const QString robot = CurrentRobotName();
    QString error;
    if (robot.isEmpty() || !RobotDataHelper::EnsureMeasureWeldParamFile(robot, &error))
    {
        return;
    }
    COPini ini;
    const QString path = RobotDataHelper::MeasureWeldParamPath(robot);
    if (!ini.SetFileName(path.toUtf8().constData()))
    {
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
        ini.ReadString(false, QStringLiteral("Group%1Name").arg(index).toStdString(), groupName);
        const QString displayName = groupName.empty()
            ? QStringLiteral("参数组%1").arg(index + 1)
            : DecodeRobotMessageText(groupName);
        m_paramGroupCombo->addItem(QStringLiteral("%1 / Group%2").arg(displayName).arg(index), index);
    }
    m_paramGroupCombo->setCurrentIndex(useNo);
}

void ProcessLoopTestDialog::LoadProcessCombo()
{
    if (m_processCombo == nullptr)
    {
        return;
    }
    m_processCombo->clear();
    m_processCombo->setEnabled(false);
    const int unitIndex = m_unitCombo != nullptr ? m_unitCombo->currentData().toInt() : -1;
    if (m_pContralUnit == nullptr || unitIndex < 0
        || unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return;
    }
    WeldProcessFile processFile(*m_pContralUnit, unitIndex);
    if (!processFile.Init())
    {
        m_processCombo->addItem(QStringLiteral("请先在工艺界面重新创建工艺"), -1);
        return;
    }
    const std::vector<T_WELD_PARA>& weldList = processFile.GetWeldParaList();
    const int useIndex = weldList.empty()
        ? -1
        : std::clamp(processFile.GetUseWeldParaNo(), 0, static_cast<int>(weldList.size()) - 1);
    int selectedCombo = 0;
    for (int index = 0; index < static_cast<int>(weldList.size()); ++index)
    {
        const T_WELD_PARA& weld = weldList[index];
        const QString workpiece = DecodeRobotMessageText(weld.strWorkPeace);
        m_processCombo->addItem(QStringLiteral("%1. %2 | 焊脚%3")
            .arg(index + 1)
            .arg(workpiece.isEmpty() ? QStringLiteral("未命名工艺") : workpiece)
            .arg(weld.dWeldAngleSize, 0, 'f', 1), index);
        if (index == useIndex)
        {
            selectedCombo = m_processCombo->count() - 1;
        }
    }
    if (m_processCombo->count() > 0)
    {
        m_processCombo->setEnabled(true);
        m_processCombo->setCurrentIndex(std::clamp(selectedCombo, 0, m_processCombo->count() - 1));
    }
    else
    {
        m_processCombo->addItem(QStringLiteral("未读取到焊接工艺"), -1);
    }
}

void ProcessLoopTestDialog::LoadCompCombo(QComboBox* combo, const QString& path, const QString& allSection,
    const QString& rowCountKey, const QString& groupCountKey, const QString& activeKey,
    const QString& groupSectionPrefix, const QString& defaultNamePrefix)
{
    if (combo == nullptr)
    {
        return;
    }
    constexpr int kCompSegmentCount = 4;   // 每组恰好 4 段（低平台/上升边/高平台/下降边）
    combo->clear();
    int activeIndex = 0;
    int groupCount = 1;
    if (ConfigDatabase::HasIniFile(path))
    {
        COPini ini;
        if (ini.SetFileName(path.toUtf8().constData()))
        {
            ini.SetSectionName(allSection.toUtf8().constData());
            ini.ReadString(false, activeKey.toUtf8().constData(), &activeIndex);
            int configuredGroupCount = 0;
            if (ini.ReadString(false, groupCountKey.toUtf8().constData(), &configuredGroupCount) > 0)
            {
                groupCount = std::max(1, configuredGroupCount);
            }
            else
            {
                int rowCount = kCompSegmentCount;
                ini.ReadString(false, rowCountKey.toUtf8().constData(), &rowCount);
                groupCount = std::max(1, (std::max(0, rowCount) + kCompSegmentCount - 1) / kCompSegmentCount);
            }
            activeIndex = std::clamp(activeIndex, 0, groupCount - 1);
            for (int index = 0; index < groupCount; ++index)
            {
                QString groupName = QStringLiteral("%1%2").arg(defaultNamePrefix).arg(index + 1);
                std::string encodedName;
                ini.SetSectionName(QStringLiteral("%1%2").arg(groupSectionPrefix).arg(index).toUtf8().constData());
                if (ini.ReadString(false, "Name", encodedName) > 0)
                {
                    const QString decoded = DecodeRobotMessageText(encodedName).trimmed();
                    if (!decoded.isEmpty())
                    {
                        groupName = decoded;
                    }
                }
                combo->addItem(QStringLiteral("%1 / Group%2").arg(groupName).arg(index), index);
            }
        }
    }
    if (combo->count() <= 0)
    {
        combo->addItem(QStringLiteral("%1%2 / Group0").arg(defaultNamePrefix).arg(1), 0);
        activeIndex = 0;
    }
    combo->setCurrentIndex(std::clamp(activeIndex, 0, combo->count() - 1));
    combo->setEnabled(combo->count() > 0);
}

void ProcessLoopTestDialog::ReloadDefaultsForCurrentUnit()
{
    if (m_unitCombo == nullptr || !m_loadDefaults)
    {
        return;
    }
    const int unitIndex = m_unitCombo->currentData().toInt();
    const ProcessLoopTestDefaults d = m_loadDefaults(unitIndex);
    if (!d.ok)
    {
        m_defaultsLabel->setText(QStringLiteral("预设：读取失败（%1）。开始前请确认该单元已示教并设置好测量焊接参数。")
            .arg(d.issue.isEmpty() ? QStringLiteral("未知原因") : d.issue));
        return;
    }
    m_defaultsLabel->setText(QStringLiteral("预设「%1」：扫描速度 %2 mm/min · 运行速度 %3 mm/min · 相机偏移 %4 ms")
        .arg(d.paramGroupName.isEmpty() ? QStringLiteral("参数组") : d.paramGroupName)
        .arg(d.scanSpeedMmPerMin, 0, 'f', 1)
        .arg(d.runSpeedMmPerMin, 0, 'f', 1)
        .arg(d.cameraTimeOffsetMs, 0, 'f', 1));
    // 覆盖框未勾选时用预设值预填，方便用户在此基础上微调。
    if (m_scanSpeedSpin != nullptr && !m_ovScanSpeedCheck->isChecked()) { m_scanSpeedSpin->setValue(d.scanSpeedMmPerMin); }
    if (m_runSpeedSpin != nullptr && !m_ovRunSpeedCheck->isChecked()) { m_runSpeedSpin->setValue(d.runSpeedMmPerMin); }
    if (m_cameraOffsetSpin != nullptr && !m_ovCameraOffsetCheck->isChecked()) { m_cameraOffsetSpin->setValue(d.cameraTimeOffsetMs); }
}

ProcessLoopTestSettings ProcessLoopTestDialog::CollectSettings() const
{
    ProcessLoopTestSettings s;
    s.unitIndex = m_unitCombo != nullptr ? m_unitCombo->currentData().toInt() : -1;
    s.infinite = m_infiniteCheck != nullptr && m_infiniteCheck->isChecked();
    s.repeatCount = m_repeatSpin != nullptr ? m_repeatSpin->value() : 1;
    s.stopOnFailure = m_stopOnFailureCheck == nullptr || m_stopOnFailureCheck->isChecked();
    s.doWeld = m_doWeldCheck != nullptr && m_doWeldCheck->isChecked();
    const int intervalUnitMul = m_intervalUnitCombo != nullptr ? m_intervalUnitCombo->currentData().toInt() : 1;
    s.scanIntervalSeconds = (m_intervalSpin != nullptr ? m_intervalSpin->value() : 0) * intervalUnitMul;
    s.overrideScanSpeed = m_ovScanSpeedCheck != nullptr && m_ovScanSpeedCheck->isChecked();
    s.scanSpeedMmPerMin = m_scanSpeedSpin != nullptr ? m_scanSpeedSpin->value() : 0.0;
    s.overrideRunSpeed = m_ovRunSpeedCheck != nullptr && m_ovRunSpeedCheck->isChecked();
    s.runSpeedMmPerMin = m_runSpeedSpin != nullptr ? m_runSpeedSpin->value() : 0.0;
    s.overrideCameraOffset = m_ovCameraOffsetCheck != nullptr && m_ovCameraOffsetCheck->isChecked();
    s.cameraTimeOffsetMs = m_cameraOffsetSpin != nullptr ? m_cameraOffsetSpin->value() : 0.0;
    return s;
}

void ProcessLoopTestDialog::OnStart()
{
    if (m_running.load() || !m_runner)
    {
        return;
    }
    // 互锁：先测后焊流程正在驱动机器人时，不允许再起流程测试（同机同驱动会冲突）。
    if (m_preStartGuard)
    {
        QString reason;
        if (!m_preStartGuard(reason))
        {
            QMessageBox::warning(this, QStringLiteral("流程测试"),
                reason.isEmpty() ? QStringLiteral("当前有流程正在运行，无法开始测试。") : reason);
            return;
        }
    }
    // 焊接段是物理动作、循环里会反复执行：开始前明确二次确认现场安全。
    if (m_doWeldCheck != nullptr && m_doWeldCheck->isChecked())
    {
        const auto ret = QMessageBox::warning(this, QStringLiteral("流程测试 · 包含焊接段"),
            QStringLiteral("已勾选「包含焊接段」：每次循环将在扫描后执行焊接动作"
                "（实际焊接或空跑按预设设置）。\n\n请确认现场安全、机器人工作范围内无人。\n\n确认开始？"),
            QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
        if (ret != QMessageBox::Ok)
        {
            return;
        }
    }
    JoinWorker();  // 回收上一轮已结束的线程

    SaveSettings();  // 记住本次测试设置，下次打开自动恢复
    const ProcessLoopTestSettings settings = CollectSettings();
    RobotDriverAdaptor* driver = nullptr;
    if (m_pContralUnit != nullptr
        && settings.unitIndex >= 0
        && settings.unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        driver = static_cast<RobotDriverAdaptor*>(
            m_pContralUnit->m_vtContralUnitInfo[settings.unitIndex].pUnitDriver);
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("流程循环测试"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, QStringLiteral("流程测试"), leaseError);
        return;
    }
    m_logText->clear();
    UpdateProgressUi(0, settings.infinite ? 0 : settings.repeatCount, 0, 0, QStringLiteral("准备中…"), false);
    SetRunningUi(true);
    m_stopRequested.store(false);
    m_running.store(true);

    // 进度/日志回调：后台线程调用 → 编列到 UI 线程执行。
    auto progressCb = [this](int it, int total, int ok, int fail, const QString& step, bool finished)
        {
            QMetaObject::invokeMethod(this, [this, it, total, ok, fail, step, finished]()
                { UpdateProgressUi(it, total, ok, fail, step, finished); }, Qt::QueuedConnection);
        };
    auto logCb = [this](const QString& text)
        {
            QMetaObject::invokeMethod(this, [this, text]() { AppendLog(text); }, Qt::QueuedConnection);
        };

    m_worker = std::thread([this, settings, progressCb, logCb, operationLease]()
        {
            m_runner(settings, &m_stopRequested, progressCb, logCb);
            m_running.store(false);
        });
}

void ProcessLoopTestDialog::OnStop()
{
    if (!m_running.load())
    {
        return;
    }
    m_stopRequested.store(true);
    if (m_stopBtn != nullptr) { m_stopBtn->setEnabled(false); }
    AppendLog(QStringLiteral("已请求停止，将在当前步骤结束后停下…"));
}

void ProcessLoopTestDialog::JoinWorker()
{
    if (m_worker.joinable())
    {
        m_worker.join();
    }
}

void ProcessLoopTestDialog::SetRunningUi(bool running)
{
    if (m_startBtn != nullptr) { m_startBtn->setEnabled(!running); }
    if (m_stopBtn != nullptr) { m_stopBtn->setEnabled(running); }
    if (m_unitCombo != nullptr) { m_unitCombo->setEnabled(!running); }
    if (m_repeatSpin != nullptr) { m_repeatSpin->setEnabled(!running && !(m_infiniteCheck && m_infiniteCheck->isChecked())); }
    if (m_infiniteCheck != nullptr) { m_infiniteCheck->setEnabled(!running); }
    if (m_stopOnFailureCheck != nullptr) { m_stopOnFailureCheck->setEnabled(!running); }
    if (m_doWeldCheck != nullptr) { m_doWeldCheck->setEnabled(!running); }
    if (m_intervalSpin != nullptr) { m_intervalSpin->setEnabled(!running); }
    if (m_intervalUnitCombo != nullptr) { m_intervalUnitCombo->setEnabled(!running); }
    for (QCheckBox* c : { m_ovScanSpeedCheck, m_ovRunSpeedCheck, m_ovCameraOffsetCheck })
    {
        if (c != nullptr) { c->setEnabled(!running); }
    }
    for (QComboBox* c : { m_paramGroupCombo, m_processCombo, m_poseCompCombo, m_seamCompCombo })
    {
        if (c != nullptr) { c->setEnabled(!running && c->count() > 0); }
    }
}

void ProcessLoopTestDialog::UpdateProgressUi(int iteration, int total, int okCount, int failCount,
    const QString& step, bool finished)
{
    if (m_progressLabel != nullptr)
    {
        const QString countText = total > 0
            ? QStringLiteral("第 %1 / %2 次").arg(iteration).arg(total)
            : QStringLiteral("第 %1 次（持续）").arg(iteration);
        m_progressLabel->setText(finished
            ? QStringLiteral("测试结束。%1").arg(step)
            : QStringLiteral("%1 · %2").arg(countText, step));
    }
    if (m_statLabel != nullptr)
    {
        m_statLabel->setText(QStringLiteral("成功 %1 · 失败 %2").arg(okCount).arg(failCount));
    }
    if (finished)
    {
        m_running.store(false);
        SetRunningUi(false);
    }
}

void ProcessLoopTestDialog::AppendLog(const QString& text)
{
    if (m_logText != nullptr)
    {
        m_logText->appendPlainText(QStringLiteral("[%1] %2")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss"), text));
    }
}

void ProcessLoopTestDialog::closeEvent(QCloseEvent* event)
{
    SaveSettings();  // 关页也保存（用户只改设置没点开始的情况）
    if (m_running.load())
    {
        // 不阻塞 UI：请求停止即返回，当前步骤在后台自然收尾后循环退出（与「停止」按钮一致）。
        // 本页对象由主窗口缓存复用、不随关闭销毁，后台线程持有的 this 始终有效；
        // 线程在下次「开始」(JoinWorker 回收) 或程序退出 (析构 Join) 时回收。
        // 程序整体退出另有主窗口守卫拦截（流程未结束不放行关闭），不会留悬垂运动。
        m_stopRequested.store(true);
        AppendLog(QStringLiteral("界面关闭：已请求停止，当前步骤结束后自动停下（无需等待）。"));
    }
    else
    {
        JoinWorker();  // 已结束：顺手回收上一轮线程
    }
    QDialog::closeEvent(event);
}
