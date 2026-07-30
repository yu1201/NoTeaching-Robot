#include "ScanSafetyGateDialog.h"

#include "PointCloudProcessingConfig.h"

#include <QCheckBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QShowEvent>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <algorithm>
#include <iterator>
#include <utility>

namespace
{
QLabel* MakeSummaryValue(const QString& objectName)
{
    auto* label = new QLabel();
    label->setObjectName(objectName);
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumHeight(34);
    label->setWordWrap(true);
    label->setProperty("summaryValue", true);
    return label;
}

QLabel* MakeFieldLabel(const QString& text)
{
    auto* label = new QLabel(text);
    label->setProperty("fieldLabel", true);
    return label;
}

void SetTableItem(QTableWidget* table, int row, int column, const QString& text)
{
    auto* item = new QTableWidgetItem(text);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    item->setTextAlignment(column == 0
        ? Qt::AlignLeft | Qt::AlignVCenter
        : Qt::AlignLeft | Qt::AlignTop);
    table->setItem(row, column, item);
}

PointCloudProcessingConfig::Settings SafetyGateDefaults()
{
    PointCloudProcessingConfig::Settings defaults;
    defaults.safetyGateProofIntegrityEnabled = true;
    defaults.safetyGateProductionPurposeEnabled = true;
    defaults.safetyGateRobotNameBindingEnabled = true;
    defaults.safetyGateCaseBindingEnabled = true;
    defaults.safetyGateEndpointBindingEnabled = true;
    defaults.safetyGateCameraHandEyeBindingEnabled = true;
    defaults.safetyGateFreshnessEnabled = true;
    defaults.safetyGatePolicySnapshotEnabled = true;
    defaults.safetyGateInputEvidenceEnabled = true;
    defaults.safetyGateAuthorizedPoseIdentityEnabled = true;
    defaults.safetyGateTrajectoryStructureEnabled = true;
    defaults.safetyGateMotionPrecheckEnabled = true;
    return defaults;
}
}

ScanSafetyGateDialog::ScanSafetyGateDialog(
    std::function<bool()> modifyGuard,
    QWidget* parent)
    : QDialog(parent)
    , m_modifyGuard(std::move(modifyGuard))
{
    setObjectName(QStringLiteral("scanSafetyGateDialog"));
    setWindowFlags(Qt::Widget);
    setMinimumSize(920, 680);
    BuildUi();
    Reload();
}

void ScanSafetyGateDialog::BuildUi()
{
    setStyleSheet(QStringLiteral(R"(
        QDialog#scanSafetyGateDialog {
            background: #0d1720;
            color: #d7e5ee;
        }
        QDialog#scanSafetyGateDialog QScrollArea,
        QDialog#scanSafetyGateDialog QWidget#scanSafetyGateContent {
            background: #0d1720;
            border: 0;
        }
        QDialog#scanSafetyGateDialog QFrame[card="true"],
        QDialog#scanSafetyGateDialog QGroupBox {
            background: #111e28;
            border: 1px solid #294454;
            border-radius: 10px;
        }
        QDialog#scanSafetyGateDialog QGroupBox {
            margin-top: 12px;
            padding-top: 6px;
            font-weight: 600;
            color: #8fd8ef;
        }
        QDialog#scanSafetyGateDialog QGroupBox::title {
            subcontrol-origin: margin;
            left: 14px;
            padding: 0 6px;
        }
        QDialog#scanSafetyGateDialog QLabel {
            color: #cbd9e2;
        }
        QDialog#scanSafetyGateDialog QLabel[summaryValue="true"] {
            color: #8fe5b2;
            background: #0b151d;
            border: 1px solid #315163;
            border-radius: 6px;
            font-weight: 600;
            padding: 5px 10px;
        }
        QDialog#scanSafetyGateDialog QLabel[fieldLabel="true"] {
            color: #98adba;
        }
        QDialog#scanSafetyGateDialog QComboBox,
        QDialog#scanSafetyGateDialog QSpinBox,
        QDialog#scanSafetyGateDialog QDoubleSpinBox {
            min-height: 30px;
            color: #edf6fb;
            background: #071118;
            border: 1px solid #345367;
            border-radius: 5px;
            padding: 0 8px;
        }
        QDialog#scanSafetyGateDialog QComboBox:focus,
        QDialog#scanSafetyGateDialog QSpinBox:focus,
        QDialog#scanSafetyGateDialog QDoubleSpinBox:focus {
            border-color: #52b8d8;
        }
        QDialog#scanSafetyGateDialog QCheckBox:disabled {
            color: #8fe5b2;
        }
        QDialog#scanSafetyGateDialog QTableWidget {
            color: #d7e5ee;
            background: #0b151d;
            alternate-background-color: #101d27;
            border: 1px solid #294454;
            border-radius: 6px;
            gridline-color: #29404f;
            selection-background-color: #244b5d;
        }
        QDialog#scanSafetyGateDialog QHeaderView::section {
            color: #9edcf0;
            background: #172733;
            border: 0;
            border-right: 1px solid #294454;
            border-bottom: 1px solid #294454;
            padding: 8px;
            font-weight: 600;
        }
        QDialog#scanSafetyGateDialog QPushButton {
            min-height: 34px;
            min-width: 112px;
            color: #e8f5fb;
            background: #213746;
            border: 1px solid #3d6579;
            border-radius: 7px;
            padding: 3px 16px;
        }
        QDialog#scanSafetyGateDialog QPushButton:hover {
            background: #294a5c;
        }
        QDialog#scanSafetyGateDialog QPushButton#saveSafetyGateButton {
            background: #176c72;
            border-color: #37a7ac;
        }
    )"));

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);

    m_scrollArea = new QScrollArea(this);
    m_scrollArea->setObjectName(QStringLiteral("scanSafetyGateScrollArea"));
    m_scrollArea->setWidgetResizable(true);
    m_scrollArea->setFrameShape(QFrame::NoFrame);
    rootLayout->addWidget(m_scrollArea);

    auto* content = new QWidget();
    content->setObjectName(QStringLiteral("scanSafetyGateContent"));
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(18, 18, 18, 18);
    contentLayout->setSpacing(14);

    auto* heading = new QLabel(QStringLiteral("流程与机器人运动安全门禁"));
    heading->setObjectName(QStringLiteral("scanSafetyGateHeading"));
    heading->setStyleSheet(QStringLiteral("font-size: 22px; font-weight: 700; color: #eef8fc;"));
    contentLayout->addWidget(heading);

    auto* intro = new QLabel(QStringLiteral(
        "本页只管理流程身份、证明链和机器人运动前复核门禁。"
        "扫描结果及焊道有效性统一在“测量参数 → 有效性检测”中查看和配置；"
        "只有通过管理员身份复核后才能保存或恢复本页开关。"));
    intro->setObjectName(QStringLiteral("scanSafetyGateIntroLabel"));
    intro->setWordWrap(true);
    contentLayout->addWidget(intro);

    auto* summaryCard = new QFrame();
    summaryCard->setObjectName(QStringLiteral("scanSafetyGateSummaryCard"));
    summaryCard->setProperty("card", true);
    auto* summaryLayout = new QGridLayout(summaryCard);
    summaryLayout->setContentsMargins(16, 14, 16, 14);
    summaryLayout->setHorizontalSpacing(12);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("流程门禁记录")), 0, 0);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("本页职责")), 0, 1);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("运动前复核状态")), 0, 2);
    m_profileSummaryLabel = MakeSummaryValue(QStringLiteral("validationProfileSummaryLabel"));
    m_policySummaryLabel = MakeSummaryValue(QStringLiteral("validationPolicySummaryLabel"));
    m_proofSummaryLabel = MakeSummaryValue(QStringLiteral("validationProofSummaryLabel"));
    summaryLayout->addWidget(m_profileSummaryLabel, 1, 0);
    summaryLayout->addWidget(m_policySummaryLabel, 1, 1);
    summaryLayout->addWidget(m_proofSummaryLabel, 1, 2);
    summaryLayout->setColumnStretch(0, 1);
    summaryLayout->setColumnStretch(1, 1);
    summaryLayout->setColumnStretch(2, 1);
    contentLayout->addWidget(summaryCard);

    m_changeWarningLabel = new QLabel();
    m_changeWarningLabel->setObjectName(QStringLiteral("validationChangeWarningLabel"));
    m_changeWarningLabel->setWordWrap(true);
    m_changeWarningLabel->setMinimumHeight(48);
    m_changeWarningLabel->setStyleSheet(QStringLiteral(
        "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
        "color:#ffd878; padding:10px 12px; font-weight:600;"));
    contentLayout->addWidget(m_changeWarningLabel);

    auto* hardGateGroup =
        new QGroupBox(QStringLiteral("流程与机器人运动安全门禁（管理员可配置）"));
    hardGateGroup->setObjectName(QStringLiteral("systemHardGateGroup"));
    auto* hardGateLayout = new QVBoxLayout(hardGateGroup);
    hardGateLayout->setContentsMargins(12, 18, 12, 12);
    m_hardGateTable = new QTableWidget();
    m_hardGateTable->setObjectName(QStringLiteral("systemHardGateTable"));
    hardGateLayout->addWidget(m_hardGateTable);
    BuildHardGateTable();
    contentLayout->addWidget(hardGateGroup);

    auto* actionCard = new QFrame();
    actionCard->setObjectName(QStringLiteral("scanSafetyGateActionCard"));
    actionCard->setProperty("card", true);
    auto* actionLayout = new QHBoxLayout(actionCard);
    actionLayout->setContentsMargins(14, 12, 14, 12);
    auto* actionHint = new QLabel(QStringLiteral(
        "保存和载入流程/运动安全默认记录均会重新验证管理员身份。"));
    actionHint->setWordWrap(true);
    actionLayout->addWidget(actionHint, 1);
    m_reloadButton = new QPushButton(QStringLiteral("重新加载"));
    m_reloadButton->setObjectName(QStringLiteral("reloadSafetyGateButton"));
    m_restoreDefaultsButton = new QPushButton(QStringLiteral("载入安全默认值"));
    m_restoreDefaultsButton->setObjectName(QStringLiteral("restoreSafetyGateDefaultsButton"));
    m_saveButton = new QPushButton(QStringLiteral("保存"));
    m_saveButton->setObjectName(QStringLiteral("saveSafetyGateButton"));
    actionLayout->addWidget(m_reloadButton);
    actionLayout->addWidget(m_restoreDefaultsButton);
    actionLayout->addWidget(m_saveButton);
    contentLayout->addWidget(actionCard);

    contentLayout->addStretch(1);
    m_scrollArea->setWidget(content);

    connect(m_reloadButton, &QPushButton::clicked, this, [this]() { Reload(); });
    connect(m_restoreDefaultsButton, &QPushButton::clicked, this, [this]()
    {
        RestoreSafetyDefaults();
    });
    connect(m_saveButton, &QPushButton::clicked, this, [this]() { Save(); });
    ConnectChangeTracking();
}

void ScanSafetyGateDialog::BuildHardGateTable()
{
    struct HardGateRow
    {
        const char* objectName;
        const char* name;
        const char* validation;
        const char* disabledEffect;
        QCheckBox** check;
    };

    const HardGateRow rows[] = {
        {
            "safetyGateProofIntegrityEnabledCheckBox",
            "流程证明结构与防篡改",
            "执行证明和 HMAC 收据必须完整、可验签，并与当前案例绑定；本项不判定点云或焊道质量。",
            "关闭后跳过拒绝闭锁、HMAC、持久收据及运动期证明租约复核。",
            &m_proofIntegrityGateCheck
        },
        {
            "safetyGateProductionPurposeEnabledCheckBox",
            "生产用途",
            "证明用途必须为 production，诊断、预览或测试用途的证明不能进入生产流程。",
            "关闭后不再要求 production/Enforce/PASS/authorized 用途与状态。",
            &m_productionPurposeGateCheck
        },
        {
            "safetyGateRobotNameBindingEnabledCheckBox",
            "机器人逻辑名称绑定",
            "证明记录的机器人逻辑名称必须与当前所选机器人名称一致。",
            "关闭后允许证明机器人逻辑名称与当前机器人不同。",
            &m_robotNameBindingGateCheck
        },
        {
            "safetyGateCaseBindingEnabledCheckBox",
            "案例目录绑定",
            "证明中的规范化案例目录必须与当前运行案例目录一致。",
            "关闭后允许证明案例目录与当前轨迹目录不同。",
            &m_caseBindingGateCheck
        },
        {
            "safetyGateEndpointBindingEnabledCheckBox",
            "TCP 持久端点/控制单元绑定",
            "持久化 TCP 端点和控制单元身份必须与生成证明时一致。",
            "关闭后跳过机器人持久端点/控制单元一致性复核。",
            &m_endpointBindingGateCheck
        },
        {
            "safetyGateCameraHandEyeBindingEnabledCheckBox",
            "相机与手眼绑定",
            "定位相机身份、相机配置和手眼标定身份必须与生成证明时一致。",
            "关闭后跳过相机身份、相机配置和手眼标定一致性复核。",
            &m_cameraHandEyeBindingGateCheck
        },
        {
            "safetyGateFreshnessEnabledCheckBox",
            "证明新鲜度",
            "流程证明及其绑定输入不超过 24 小时；时间戳最多允许未来偏差 5 分钟。",
            "关闭后跳过24小时有效期、未来时间和扫描/证明时序复核。",
            &m_freshnessGateCheck
        },
        {
            "safetyGatePolicySnapshotEnabledCheckBox",
            "执行配置快照身份",
            "证明记录的处理配置快照身份必须与当前执行配置一致；具体有效性参数由有效性检测页负责。",
            "关闭后不比较 schema/profile/算法、处理模式、阈值和开关快照。",
            &m_policySnapshotGateCheck
        },
        {
            "safetyGateInputEvidenceEnabledCheckBox",
            "流程输入证据身份",
            "流程输入证据的路径、大小和摘要必须与生成执行证明时一致，不在本页评价输入质量。",
            "关闭后不回读并核对原始输入证据的大小和摘要。",
            &m_inputEvidenceGateCheck
        },
        {
            "safetyGateAuthorizedPoseIdentityEnabledCheckBox",
            "授权运动轨迹身份",
            "机器人将执行的轨迹和授权位姿身份必须与流程证明记录一致。",
            "关闭后不核对授权轨迹文件名、大小和 SHA256。",
            &m_authorizedPoseIdentityGateCheck
        },
        {
            "safetyGateTrajectoryStructureEnabledCheckBox",
            "机器人执行轨迹结构",
            "运动下发前复核轨迹记录数量、字段完整性和数值可解析性；焊道有效性由有效性检测页负责。",
            "关闭后跳过执行轨迹索引、标签结构和续焊文件身份复核；基础解析仍保留。",
            &m_trajectoryStructureGateCheck
        },
        {
            "safetyGateMotionPrecheckEnabledCheckBox",
            "运动前复核与限值",
            "每次运动前重新读取配置、绑定和证据，并检查机器人位姿、关节/笛卡尔值及工艺限值。",
            "关闭后跳过外部运动前身份/限值回调；机器人指令基础可执行条件仍保留。",
            &m_motionPrecheckGateCheck
        }
    };

    m_hardGateTable->setColumnCount(4);
    m_hardGateTable->setHorizontalHeaderLabels({
        QStringLiteral("开关"),
        QStringLiteral("系统门禁"),
        QStringLiteral("复核内容"),
        QStringLiteral("关闭后的实际影响")
    });
    m_hardGateTable->setRowCount(static_cast<int>(std::size(rows)));
    m_hardGateTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_hardGateTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_hardGateTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_hardGateTable->setAlternatingRowColors(true);
    m_hardGateTable->setWordWrap(true);
    m_hardGateTable->verticalHeader()->setVisible(false);
    m_hardGateTable->horizontalHeader()->setStretchLastSection(true);
    m_hardGateTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_hardGateTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_hardGateTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    m_hardGateTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

    for (int row = 0; row < static_cast<int>(std::size(rows)); ++row)
    {
        auto* check = new QCheckBox(QStringLiteral("开启"));
        check->setObjectName(QString::fromLatin1(rows[row].objectName));
        check->setChecked(true);
        check->setToolTip(QString::fromUtf8(rows[row].disabledEffect));
        *rows[row].check = check;
        m_hardGateTable->setCellWidget(row, 0, check);
        SetTableItem(m_hardGateTable, row, 1, QString::fromUtf8(rows[row].name));
        SetTableItem(m_hardGateTable, row, 2, QString::fromUtf8(rows[row].validation));
        SetTableItem(m_hardGateTable, row, 3, QString::fromUtf8(rows[row].disabledEffect));
    }
    m_hardGateTable->resizeRowsToContents();
    m_hardGateTable->setMinimumHeight(680);
}

void ScanSafetyGateDialog::ConnectChangeTracking()
{
    const auto changed = [this]()
    {
        if (!m_loading)
        {
            SetDirty(true);
        }
    };

    const QList<QCheckBox*> gateChecks = {
        m_proofIntegrityGateCheck,
        m_productionPurposeGateCheck,
        m_robotNameBindingGateCheck,
        m_caseBindingGateCheck,
        m_endpointBindingGateCheck,
        m_cameraHandEyeBindingGateCheck,
        m_freshnessGateCheck,
        m_policySnapshotGateCheck,
        m_inputEvidenceGateCheck,
        m_authorizedPoseIdentityGateCheck,
        m_trajectoryStructureGateCheck,
        m_motionPrecheckGateCheck
    };
    for (QCheckBox* check : gateChecks)
    {
        connect(check, &QCheckBox::toggled, this, [this, changed](bool)
        {
            UpdateSummary();
            changed();
        });
    }
}

void ScanSafetyGateDialog::Reload()
{
    const PointCloudProcessingConfig::Settings settings =
        PointCloudProcessingConfig::Load();

    m_loading = true;
    m_proofIntegrityGateCheck->setChecked(settings.safetyGateProofIntegrityEnabled);
    m_productionPurposeGateCheck->setChecked(settings.safetyGateProductionPurposeEnabled);
    m_robotNameBindingGateCheck->setChecked(settings.safetyGateRobotNameBindingEnabled);
    m_caseBindingGateCheck->setChecked(settings.safetyGateCaseBindingEnabled);
    m_endpointBindingGateCheck->setChecked(settings.safetyGateEndpointBindingEnabled);
    m_cameraHandEyeBindingGateCheck->setChecked(settings.safetyGateCameraHandEyeBindingEnabled);
    m_freshnessGateCheck->setChecked(settings.safetyGateFreshnessEnabled);
    m_policySnapshotGateCheck->setChecked(settings.safetyGatePolicySnapshotEnabled);
    m_inputEvidenceGateCheck->setChecked(settings.safetyGateInputEvidenceEnabled);
    m_authorizedPoseIdentityGateCheck->setChecked(settings.safetyGateAuthorizedPoseIdentityEnabled);
    m_trajectoryStructureGateCheck->setChecked(settings.safetyGateTrajectoryStructureEnabled);
    m_motionPrecheckGateCheck->setChecked(settings.safetyGateMotionPrecheckEnabled);

    m_loading = false;
    UpdateSummary();
    SetDirty(false);
}

bool ScanSafetyGateDialog::HasUnsavedChanges() const noexcept
{
    return m_dirty;
}

bool ScanSafetyGateDialog::AuthorizeModification(const QString& actionName)
{
    if (m_modifyGuard && m_modifyGuard())
    {
        return true;
    }

    QMessageBox::warning(
        this,
        QStringLiteral("需要管理员权限"),
        QStringLiteral(
            "%1未通过管理员身份复核。工程师账号可以查看，但不能修改流程与机器人运动安全门禁。")
            .arg(actionName));
    return false;
}

void ScanSafetyGateDialog::RestoreSafetyDefaults()
{
    if (!AuthorizeModification(QStringLiteral("载入安全默认值")))
    {
        return;
    }

    const PointCloudProcessingConfig::Settings defaults = SafetyGateDefaults();
    m_loading = true;
    m_proofIntegrityGateCheck->setChecked(defaults.safetyGateProofIntegrityEnabled);
    m_productionPurposeGateCheck->setChecked(defaults.safetyGateProductionPurposeEnabled);
    m_robotNameBindingGateCheck->setChecked(defaults.safetyGateRobotNameBindingEnabled);
    m_caseBindingGateCheck->setChecked(defaults.safetyGateCaseBindingEnabled);
    m_endpointBindingGateCheck->setChecked(defaults.safetyGateEndpointBindingEnabled);
    m_cameraHandEyeBindingGateCheck->setChecked(defaults.safetyGateCameraHandEyeBindingEnabled);
    m_freshnessGateCheck->setChecked(defaults.safetyGateFreshnessEnabled);
    m_policySnapshotGateCheck->setChecked(defaults.safetyGatePolicySnapshotEnabled);
    m_inputEvidenceGateCheck->setChecked(defaults.safetyGateInputEvidenceEnabled);
    m_authorizedPoseIdentityGateCheck->setChecked(defaults.safetyGateAuthorizedPoseIdentityEnabled);
    m_trajectoryStructureGateCheck->setChecked(defaults.safetyGateTrajectoryStructureEnabled);
    m_motionPrecheckGateCheck->setChecked(defaults.safetyGateMotionPrecheckEnabled);
    m_loading = false;
    UpdateSummary();
    SetDirty(true);
}

void ScanSafetyGateDialog::Save()
{
    if (!m_dirty)
    {
        return;
    }
    if (!AuthorizeModification(QStringLiteral("保存流程与机器人运动安全门禁")))
    {
        return;
    }

    const QString disabledGates = DisabledGateDescription();
    if (!disabledGates.isEmpty())
    {
        QString risk = QStringLiteral(
            "以下门禁将被关闭：\n%1\n\n"
            "关闭后，对应流程身份、证明链或机器人运动前复核会在下一次流程中真实跳过。"
            "扫描结果和焊道有效性仍由“有效性检测”页独立控制。"
            "\n\n确认以管理员身份保存这些记录吗？").arg(disabledGates);
        if (QMessageBox::warning(
                this,
                QStringLiteral("确认关闭安全门禁"),
                risk,
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No) != QMessageBox::Yes)
        {
            return;
        }
    }

    // 并发安全：保存前重新加载最新处理配置，只覆盖本页负责的流程/运动安全记录，
    // 不读写有效性页面负责的 Validation 策略、开关或数值。
    PointCloudProcessingConfig::Settings settings =
        PointCloudProcessingConfig::Load();
    settings.safetyGateProofIntegrityEnabled = m_proofIntegrityGateCheck->isChecked();
    settings.safetyGateProductionPurposeEnabled = m_productionPurposeGateCheck->isChecked();
    settings.safetyGateRobotNameBindingEnabled = m_robotNameBindingGateCheck->isChecked();
    settings.safetyGateCaseBindingEnabled = m_caseBindingGateCheck->isChecked();
    settings.safetyGateEndpointBindingEnabled = m_endpointBindingGateCheck->isChecked();
    settings.safetyGateCameraHandEyeBindingEnabled = m_cameraHandEyeBindingGateCheck->isChecked();
    settings.safetyGateFreshnessEnabled = m_freshnessGateCheck->isChecked();
    settings.safetyGatePolicySnapshotEnabled = m_policySnapshotGateCheck->isChecked();
    settings.safetyGateInputEvidenceEnabled = m_inputEvidenceGateCheck->isChecked();
    settings.safetyGateAuthorizedPoseIdentityEnabled = m_authorizedPoseIdentityGateCheck->isChecked();
    settings.safetyGateTrajectoryStructureEnabled = m_trajectoryStructureGateCheck->isChecked();
    settings.safetyGateMotionPrecheckEnabled = m_motionPrecheckGateCheck->isChecked();

    QString error;
    if (!PointCloudProcessingConfig::Save(settings, &error))
    {
        QMessageBox::critical(
            this,
            QStringLiteral("保存失败"),
            error.isEmpty()
                ? QStringLiteral("流程与机器人运动安全门禁保存失败。")
                : error);
        return;
    }

    Reload();
    QMessageBox::information(
        this,
        QStringLiteral("保存成功"),
        QStringLiteral(
            "流程与机器人运动安全门禁已保存。"
            "开关将在后续流程中实际生效；有效性检测页配置未被修改。"));
}

void ScanSafetyGateDialog::UpdateSummary()
{
    const bool hasDisabledSafetyGateRecord = HasDisabledCoreSafetyGateUi();
    const QList<QCheckBox*> gateChecks = {
        m_proofIntegrityGateCheck,
        m_productionPurposeGateCheck,
        m_robotNameBindingGateCheck,
        m_caseBindingGateCheck,
        m_endpointBindingGateCheck,
        m_cameraHandEyeBindingGateCheck,
        m_freshnessGateCheck,
        m_policySnapshotGateCheck,
        m_inputEvidenceGateCheck,
        m_authorizedPoseIdentityGateCheck,
        m_trajectoryStructureGateCheck,
        m_motionPrecheckGateCheck
    };
    const int enabledCount = static_cast<int>(std::count_if(
        gateChecks.cbegin(),
        gateChecks.cend(),
        [](const QCheckBox* check) { return check->isChecked(); }));

    m_profileSummaryLabel->setText(QStringLiteral("%1/%2 门禁开启")
        .arg(enabledCount)
        .arg(gateChecks.size()));
    m_policySummaryLabel->setText(QStringLiteral("流程身份 · 证明链 · 运动前复核"));
    m_proofSummaryLabel->setText(hasDisabledSafetyGateRecord
        ? QStringLiteral("存在已关闭门禁 · 后续流程跳过对应复核")
        : QStringLiteral("流程与运动门禁全部开启"));

    const QString normalSummaryStyle = QStringLiteral(
        "color:#8fe5b2; background:#0b151d; border:1px solid #315163;"
        "border-radius:6px; font-weight:600; padding:5px 10px;");
    const QString warningSummaryStyle = QStringLiteral(
        "color:#ffd878; background:#392b0c; border:1px solid #8a6820;"
        "border-radius:6px; font-weight:700; padding:5px 10px;");
    m_policySummaryLabel->setStyleSheet(normalSummaryStyle);
    m_proofSummaryLabel->setStyleSheet(
        hasDisabledSafetyGateRecord ? warningSummaryStyle : normalSummaryStyle);
    UpdateChangeWarning();
}

bool ScanSafetyGateDialog::HasDisabledCoreSafetyGateUi() const
{
    // 本页开关实际控制流程/运动复核，但不参与焊道或扫描结果有效性判定。
    return !m_proofIntegrityGateCheck->isChecked()
        || !m_productionPurposeGateCheck->isChecked()
        || !m_robotNameBindingGateCheck->isChecked()
        || !m_caseBindingGateCheck->isChecked()
        || !m_endpointBindingGateCheck->isChecked()
        || !m_cameraHandEyeBindingGateCheck->isChecked()
        || !m_freshnessGateCheck->isChecked()
        || !m_policySnapshotGateCheck->isChecked()
        || !m_inputEvidenceGateCheck->isChecked()
        || !m_authorizedPoseIdentityGateCheck->isChecked()
        || !m_trajectoryStructureGateCheck->isChecked()
        || !m_motionPrecheckGateCheck->isChecked();
}

QString ScanSafetyGateDialog::DisabledGateDescription() const
{
    QStringList disabled;
    const auto addIfDisabled = [&disabled](const QCheckBox* check, const QString& name)
    {
        if (!check->isChecked())
        {
            disabled.push_back(QStringLiteral("• %1").arg(name));
        }
    };
    addIfDisabled(m_proofIntegrityGateCheck, QStringLiteral("流程证明结构与防篡改"));
    addIfDisabled(m_productionPurposeGateCheck, QStringLiteral("生产用途"));
    addIfDisabled(m_robotNameBindingGateCheck, QStringLiteral("机器人逻辑名称绑定"));
    addIfDisabled(m_caseBindingGateCheck, QStringLiteral("案例目录绑定"));
    addIfDisabled(m_endpointBindingGateCheck, QStringLiteral("TCP 持久端点/控制单元绑定"));
    addIfDisabled(m_cameraHandEyeBindingGateCheck, QStringLiteral("相机与手眼绑定"));
    addIfDisabled(m_freshnessGateCheck, QStringLiteral("证明新鲜度"));
    addIfDisabled(m_policySnapshotGateCheck, QStringLiteral("执行配置快照身份"));
    addIfDisabled(m_inputEvidenceGateCheck, QStringLiteral("流程输入证据身份"));
    addIfDisabled(m_authorizedPoseIdentityGateCheck, QStringLiteral("授权运动轨迹身份"));
    addIfDisabled(m_trajectoryStructureGateCheck, QStringLiteral("机器人执行轨迹结构"));
    addIfDisabled(m_motionPrecheckGateCheck, QStringLiteral("运动前复核与限值"));
    return disabled.join(QLatin1Char('\n'));
}

void ScanSafetyGateDialog::UpdateChangeWarning()
{
    if (HasDisabledCoreSafetyGateUi())
    {
        m_changeWarningLabel->setStyleSheet(QStringLiteral(
            "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
            "color:#ffd878; padding:10px 12px; font-weight:600;"));
        m_changeWarningLabel->setText(QStringLiteral(
            "部分流程/运动安全门禁已关闭：后续流程会真实跳过对应复核。"
            "扫描结果和焊道有效性配置不受本页影响。"));
        return;
    }

    m_changeWarningLabel->setStyleSheet(QStringLiteral(
        "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
        "color:#ffd878; padding:10px 12px; font-weight:600;"));
    if (m_dirty)
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "存在未保存变更。本页只保存流程身份、证明链和机器人运动前复核门禁，"
            "保存后会在后续流程中实际生效；不会改动有效性检测页配置。"));
    }
    else
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "本页所有流程与机器人运动安全门禁均已开启；"
            "扫描结果和焊道有效性由有效性检测页独立管理。"));
    }
}

void ScanSafetyGateDialog::SetDirty(bool dirty)
{
    m_dirty = dirty;
    m_saveButton->setEnabled(dirty);
    UpdateChangeWarning();
}

void ScanSafetyGateDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    // 管理栈切换页面只会隐藏本页；保留尚未保存的管理员编辑，避免返回本页时静默丢失。
    // 没有待保存修改时则重新取一次配置，以反映其他管理页刚完成的保存。
    if (!m_dirty)
    {
        Reload();
    }
}
