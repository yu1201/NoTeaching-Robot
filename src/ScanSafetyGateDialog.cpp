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
    defaults.validationCoverageEnabled = true;
    defaults.validationSdkBaseIntegrityEnabled = true;
    defaults.validationContinuityEnabled = true;
    defaults.validationDenoiseRatioEnabled = true;
    defaults.validationResidualEnabled = true;
    defaults.validationKeyPointEnabled = true;
    defaults.validationOutputEnabled = true;
    defaults.validationSegmentHardLimitsEnabled = true;
    defaults.validationFinalTrajectoryStepEnabled = true;
    defaults.validationFinalLengthBindingEnabled = true;
    defaults.validationFinalTopologyBindingEnabled = true;
    defaults.validationFinalSourceBindingEnabled = true;
    defaults.validationFinalSemanticIntegrityEnabled = true;
    defaults.systemInterlocks = SystemInterlockPolicy{};
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
        "本页统一列出 25 项可配置门禁和 10 项可独立开启/关闭的系统互锁。"
        "点云、焊道和最终轨迹的数值门限仍在“测量参数 → 有效性检测”中编辑；"
        "本页可统一启停它们的拦截开关。只有通过管理员身份复核后才能保存可配置项。"
        "全局互锁关闭时，普通新机器人操作不再受这 10 类条件的统一准入拦截；"
        "紧急 STOP 和专用恢复流程自身的必要校验仍保留。"));
    intro->setObjectName(QStringLiteral("scanSafetyGateIntroLabel"));
    intro->setWordWrap(true);
    contentLayout->addWidget(intro);

    auto* summaryCard = new QFrame();
    summaryCard->setObjectName(QStringLiteral("scanSafetyGateSummaryCard"));
    summaryCard->setProperty("card", true);
    auto* summaryLayout = new QGridLayout(summaryCard);
    summaryLayout->setContentsMargins(16, 14, 16, 14);
    summaryLayout->setHorizontalSpacing(12);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("可配置门禁")), 0, 0);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("本页职责")), 0, 1);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("系统互锁")), 0, 2);
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

    auto* qualityGateGroup =
        new QGroupBox(QStringLiteral("点云、焊道与最终轨迹有效性门禁（管理员可配置）"));
    qualityGateGroup->setObjectName(QStringLiteral("qualityGateGroup"));
    auto* qualityGateLayout = new QVBoxLayout(qualityGateGroup);
    qualityGateLayout->setContentsMargins(12, 18, 12, 12);
    m_qualityGateTable = new QTableWidget();
    m_qualityGateTable->setObjectName(QStringLiteral("qualityGateTable"));
    qualityGateLayout->addWidget(m_qualityGateTable);
    BuildQualityGateTable();
    contentLayout->addWidget(qualityGateGroup);

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

    auto* mandatoryGateGroup =
        new QGroupBox(QStringLiteral("全局系统互锁（每项独立开启/关闭）"));
    mandatoryGateGroup->setObjectName(QStringLiteral("mandatoryGateGroup"));
    auto* mandatoryGateLayout = new QVBoxLayout(mandatoryGateGroup);
    mandatoryGateLayout->setContentsMargins(12, 18, 12, 12);
    auto* mandatoryModeHint = new QLabel(QStringLiteral(
        "每项开关只控制本行准入条件；保存成功后立即应用。机器人控制进程单实例项在下次启动时生效。"
        "紧急 STOP 和当前流程取消仍执行；缺失驱动、无效恢复轨迹等基础执行条件仍检查。"));
    mandatoryModeHint->setObjectName(QStringLiteral("mandatorySystemInterlockModeHint"));
    mandatoryModeHint->setWordWrap(true);
    mandatoryGateLayout->addWidget(mandatoryModeHint);
    m_mandatoryGateTable = new QTableWidget();
    m_mandatoryGateTable->setObjectName(QStringLiteral("mandatoryGateTable"));
    mandatoryGateLayout->addWidget(m_mandatoryGateTable);
    BuildMandatoryGateTable();
    contentLayout->addWidget(mandatoryGateGroup);

    auto* actionCard = new QFrame();
    actionCard->setObjectName(QStringLiteral("scanSafetyGateActionCard"));
    actionCard->setProperty("card", true);
    auto* actionLayout = new QHBoxLayout(actionCard);
    actionLayout->setContentsMargins(14, 12, 14, 12);
    auto* actionHint = new QLabel(QStringLiteral(
        "保存和载入安全默认值均会重新验证管理员身份；"
        "系统互锁保存成功后立即应用，机器人控制进程单实例项需重启生效。"));
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

void ScanSafetyGateDialog::BuildQualityGateTable()
{
    struct QualityGateRow
    {
        const char* objectName;
        const char* name;
        const char* validation;
        const char* disabledEffect;
        QCheckBox** check;
    };

    const QualityGateRow rows[] = {
        {
            "validationCoverageEnabledCheckBox",
            "点云覆盖与有限点",
            "检查有限点数和扫描向投影跨度。数值门限在有效性检测页编辑。",
            "关闭后不因有限点过少或投影跨度不足拒绝点云。",
            &m_coverageGateCheck
        },
        {
            "validationSdkBaseIntegrityEnabledCheckBox",
            "SDKBase 完整点云覆盖",
            "方法②在平滑、截断和拟合前比较 SDKBase 焊道与完整点云的扫描向覆盖和端点偏差。",
            "关闭后 SDKBase 局部焊道不再因覆盖不足或端点偏差过大被拦截。",
            &m_sdkBaseIntegrityGateCheck
        },
        {
            "validationContinuityEnabledCheckBox",
            "扫描向连续性",
            "检查扫描站位覆盖率和最长连续段比例。",
            "关闭后不因扫描断层、空洞或最长连续段不足拒绝。",
            &m_continuityGateCheck
        },
        {
            "validationDenoiseRatioEnabledCheckBox",
            "去噪拒绝比例",
            "检查滤除点占输入点的比例是否超过设定上限。",
            "关闭后大比例点被去除也不再单独导致质量拒绝。",
            &m_denoiseRatioGateCheck
        },
        {
            "validationResidualEnabledCheckBox",
            "拟合残差",
            "检查中位残差、95 分位残差和残差内点率。",
            "关闭后不因拟合残差过大或内点率不足拒绝。",
            &m_residualGateCheck
        },
        {
            "validationKeyPointEnabledCheckBox",
            "拐点与分段充分性",
            "检查特征点、拐点数量和最小分段长度。",
            "关闭后不因特征点/拐点过少或分段过短拒绝。",
            &m_keyPointGateCheck
        },
        {
            "validationOutputEnabledCheckBox",
            "焊道输出数量与长度",
            "检查输出点数以及输出长度相对输入的保留比例。",
            "关闭后输出点过少或焊道长度损失不再单独导致拒绝。",
            &m_outputGateCheck
        },
        {
            "validationSegmentHardLimitsEnabledCheckBox",
            "焊道最短分段",
            "检查普通段、搭接段和端点相邻段的硬性最小长度。",
            "关闭后过短焊道分段不再被最短段门限拦截。",
            &m_segmentHardLimitsGateCheck
        },
        {
            "validationFinalTrajectoryStepEnabledCheckBox",
            "最终轨迹相邻步长",
            "检查最终执行点的位置步长、控制器欧拉差和物理姿态差。",
            "关闭后相邻位置或姿态跳变不再由该门禁拦截。",
            &m_finalTrajectoryStepGateCheck
        },
        {
            "validationFinalLengthBindingEnabledCheckBox",
            "最终轨迹长度绑定",
            "检查补偿后最终轨迹长度相对补偿前轨迹的比例。",
            "关闭后不因最终轨迹长度异常缩短或拉长拒绝。",
            &m_finalLengthBindingGateCheck
        },
        {
            "validationFinalTopologyBindingEnabledCheckBox",
            "最终轨迹拓扑绑定",
            "检查最终轨迹匹配弧长、来源唯一覆盖和来源弧长跨度。",
            "关闭后不再拦截拓扑映射覆盖不足或来源重复。",
            &m_finalTopologyBindingGateCheck
        },
        {
            "validationFinalSourceBindingEnabledCheckBox",
            "最终轨迹来源位姿绑定",
            "检查最终点相对来源点的位移和物理姿态变化。",
            "关闭后不再因最终点偏离来源位姿过大拒绝。",
            &m_finalSourceBindingGateCheck
        },
        {
            "validationFinalSemanticIntegrityEnabledCheckBox",
            "最终文件结构与语义完整性",
            "检查索引连续、标签/角度语义、起终点/拐点/搭接语义和生成文件字节摘要。",
            "关闭后不再执行最终文件的结构与语义一致性拦截。",
            &m_finalSemanticIntegrityGateCheck
        }
    };

    m_qualityGateTable->setColumnCount(4);
    m_qualityGateTable->setHorizontalHeaderLabels({
        QStringLiteral("开关"),
        QStringLiteral("有效性门禁"),
        QStringLiteral("检查内容"),
        QStringLiteral("关闭后的实际影响")
    });
    m_qualityGateTable->setRowCount(static_cast<int>(std::size(rows)));
    m_qualityGateTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_qualityGateTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_qualityGateTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_qualityGateTable->setAlternatingRowColors(true);
    m_qualityGateTable->setWordWrap(true);
    m_qualityGateTable->verticalHeader()->setVisible(false);
    m_qualityGateTable->horizontalHeader()->setStretchLastSection(true);
    m_qualityGateTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_qualityGateTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_qualityGateTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    m_qualityGateTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

    for (int row = 0; row < static_cast<int>(std::size(rows)); ++row)
    {
        auto* check = new QCheckBox(QStringLiteral("开启"));
        check->setObjectName(QString::fromLatin1(rows[row].objectName));
        check->setChecked(true);
        check->setToolTip(QString::fromUtf8(rows[row].disabledEffect));
        *rows[row].check = check;
        m_qualityGateTable->setCellWidget(row, 0, check);
        SetTableItem(m_qualityGateTable, row, 1, QString::fromUtf8(rows[row].name));
        SetTableItem(m_qualityGateTable, row, 2, QString::fromUtf8(rows[row].validation));
        SetTableItem(m_qualityGateTable, row, 3, QString::fromUtf8(rows[row].disabledEffect));
    }
    m_qualityGateTable->resizeRowsToContents();
    m_qualityGateTable->setMinimumHeight(760);
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

void ScanSafetyGateDialog::BuildMandatoryGateTable()
{
    struct MandatoryGateRow
    {
        const char* objectName;
        const char* name;
        const char* blockedCondition;
        const char* recovery;
    };

    // 这些条目覆盖 ApplicationInstanceGuard 和 RobotOperationLease 的全局准入链，
    // 以及 WeldSafetyRecoveryStore 的持久恢复链；行序与 SystemInterlock 一一对应。
    const MandatoryGateRow rows[] = {
        {
            "mandatorySingleProcessInterlock",
            "机器人控制进程单实例",
            "同一数据根目录已有另一个可构造机器人驱动的进程。",
            "关闭重复进程，保留唯一控制实例。"
        },
        {
            "mandatoryDriverEndpointIdentityInterlock",
            "机器人持久端点准入",
            "TCP 主机/端口无法形成可持久的物理端点身份。关闭后允许进程内驱动身份；驱动仍必须存在。",
            "修正当前控制单元驱动、IP 和端口配置后重试。"
        },
        {
            "mandatoryAccountSessionInterlock",
            "账号会话准入",
            "交互式账号未登录、会话失效或权限身份未通过。",
            "恢复有效登录会话并通过相应权限复核。"
        },
        {
            "mandatoryStateTransitionInterlock",
            "系统切换期间新操作闭锁",
            "系统正在更新、登出、重载配置或其他需要冻结新机器人操作的状态过渡。",
            "等待所有切换 owner 完成并释放闭锁 token。"
        },
        {
            "mandatorySafeRetreatPendingInterlock",
            "焊后安全回撤持久闭锁",
            "SafeRetreatPending=1，或 RecordV2/marker/端点索引缺失、损坏、不唯一；旧焊接程序终态或收枪到位尚未验证。",
            "仅使用“焊后安全回撤恢复”：先验证终止旧程序，再到绑定安全位并回读确认。"
        },
        {
            "mandatoryVerifiedStopInterlock",
            "STOP 后新流程准入",
            "上次停机尚未回读确认。关闭后允许取得新流程租约；当前流程 STOP/取消仍然执行。",
            "执行安全 STOP/终止并取得控制器稳定停止回读后解锁。"
        },
        {
            "mandatoryExclusiveOperationLeaseInterlock",
            "同机器人/同物理端点单操作租约",
            "同一驱动或同一规范化 TCP 端点已有高层操作持有租约。",
            "等待当前 owner 完成收尾并释放租约；不得通过改名或改配置重复取得。"
        },
        {
            "mandatoryMotionLeaseOwnershipInterlock",
            "运动命令租约准入",
            "运动命令未持有对应硬件操作租约。此项不关闭当前流程取消/STOP，也不关闭独立的运动终态检查。",
            "由合法高层流程重新取得租约；STOP 未确认前不发新运动。"
        },
        {
            "mandatoryMotionTerminalInterlock",
            "上一条运动稳定终态",
            "上一条已下发运动尚未得到完成或真实终止的稳定回读。",
            "等待完成回读；异常时走可验证中止，未确认前不开始下一条运动。"
        },
        {
            "mandatoryRecoveryIdentityInterlock",
            "恢复记录身份与独占绑定",
            "断点续焊/安全回撤的 checkpoint、端点、程序、轨迹、SHA256 或完整记录在确认后发生变化，或同端点存在第二恢复者。",
            "开启时重读完整记录并独占绑定；关闭时使用已确认快照。恢复所需的基础轨迹、程序终止和原子状态迁移仍检查。"
        }
    };

    m_mandatoryGateTable->setColumnCount(4);
    m_mandatoryGateTable->setHorizontalHeaderLabels({
        QStringLiteral("状态"),
        QStringLiteral("强制系统互锁"),
        QStringLiteral("实际拦截条件"),
        QStringLiteral("解除/恢复方式")
    });
    m_mandatoryGateTable->setRowCount(static_cast<int>(std::size(rows)));
    m_mandatoryGateTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_mandatoryGateTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_mandatoryGateTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_mandatoryGateTable->setAlternatingRowColors(true);
    m_mandatoryGateTable->setWordWrap(true);
    m_mandatoryGateTable->verticalHeader()->setVisible(false);
    m_mandatoryGateTable->horizontalHeader()->setStretchLastSection(true);
    m_mandatoryGateTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_mandatoryGateTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_mandatoryGateTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    m_mandatoryGateTable->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

    for (int row = 0; row < static_cast<int>(std::size(rows)); ++row)
    {
        auto* check = new QCheckBox(QStringLiteral("开启"));
        check->setObjectName(QString::fromLatin1(rows[row].objectName));
        check->setChecked(true);
        m_mandatoryGateChecks.at(static_cast<std::size_t>(row)) = check;
        check->setToolTip(QStringLiteral("独立控制本行准入检查，保存并重启后生效。"));
        m_mandatoryGateTable->setCellWidget(row, 0, check);
        SetTableItem(m_mandatoryGateTable, row, 1, QString::fromUtf8(rows[row].name));
        SetTableItem(m_mandatoryGateTable, row, 2, QString::fromUtf8(rows[row].blockedCondition));
        SetTableItem(m_mandatoryGateTable, row, 3, QString::fromUtf8(rows[row].recovery));
    }
    m_mandatoryGateTable->resizeRowsToContents();
    m_mandatoryGateTable->setMinimumHeight(660);
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
        m_coverageGateCheck,
        m_sdkBaseIntegrityGateCheck,
        m_continuityGateCheck,
        m_denoiseRatioGateCheck,
        m_residualGateCheck,
        m_keyPointGateCheck,
        m_outputGateCheck,
        m_segmentHardLimitsGateCheck,
        m_finalTrajectoryStepGateCheck,
        m_finalLengthBindingGateCheck,
        m_finalTopologyBindingGateCheck,
        m_finalSourceBindingGateCheck,
        m_finalSemanticIntegrityGateCheck,
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
    for (auto* check : m_mandatoryGateChecks)
    {
        connect(check, &QCheckBox::toggled, this, [this, changed](bool)
        {
            UpdateMandatoryGateStatus();
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
    m_coverageGateCheck->setChecked(settings.validationCoverageEnabled);
    m_sdkBaseIntegrityGateCheck->setChecked(settings.validationSdkBaseIntegrityEnabled);
    m_continuityGateCheck->setChecked(settings.validationContinuityEnabled);
    m_denoiseRatioGateCheck->setChecked(settings.validationDenoiseRatioEnabled);
    m_residualGateCheck->setChecked(settings.validationResidualEnabled);
    m_keyPointGateCheck->setChecked(settings.validationKeyPointEnabled);
    m_outputGateCheck->setChecked(settings.validationOutputEnabled);
    m_segmentHardLimitsGateCheck->setChecked(settings.validationSegmentHardLimitsEnabled);
    m_finalTrajectoryStepGateCheck->setChecked(settings.validationFinalTrajectoryStepEnabled);
    m_finalLengthBindingGateCheck->setChecked(settings.validationFinalLengthBindingEnabled);
    m_finalTopologyBindingGateCheck->setChecked(settings.validationFinalTopologyBindingEnabled);
    m_finalSourceBindingGateCheck->setChecked(settings.validationFinalSourceBindingEnabled);
    m_finalSemanticIntegrityGateCheck->setChecked(settings.validationFinalSemanticIntegrityEnabled);
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
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
        m_mandatoryGateChecks[i]->setChecked(settings.systemInterlocks.enabled[i]);
    UpdateMandatoryGateStatus();

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
            "%1未通过管理员身份复核。工程师账号可以查看，但不能修改有效性、流程或运动复核门禁。")
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
    m_coverageGateCheck->setChecked(defaults.validationCoverageEnabled);
    m_sdkBaseIntegrityGateCheck->setChecked(defaults.validationSdkBaseIntegrityEnabled);
    m_continuityGateCheck->setChecked(defaults.validationContinuityEnabled);
    m_denoiseRatioGateCheck->setChecked(defaults.validationDenoiseRatioEnabled);
    m_residualGateCheck->setChecked(defaults.validationResidualEnabled);
    m_keyPointGateCheck->setChecked(defaults.validationKeyPointEnabled);
    m_outputGateCheck->setChecked(defaults.validationOutputEnabled);
    m_segmentHardLimitsGateCheck->setChecked(defaults.validationSegmentHardLimitsEnabled);
    m_finalTrajectoryStepGateCheck->setChecked(defaults.validationFinalTrajectoryStepEnabled);
    m_finalLengthBindingGateCheck->setChecked(defaults.validationFinalLengthBindingEnabled);
    m_finalTopologyBindingGateCheck->setChecked(defaults.validationFinalTopologyBindingEnabled);
    m_finalSourceBindingGateCheck->setChecked(defaults.validationFinalSourceBindingEnabled);
    m_finalSemanticIntegrityGateCheck->setChecked(defaults.validationFinalSemanticIntegrityEnabled);
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
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
        m_mandatoryGateChecks[i]->setChecked(defaults.systemInterlocks.enabled[i]);
    UpdateMandatoryGateStatus();
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
    if (!AuthorizeModification(QStringLiteral("保存全部可配置门禁")))
    {
        return;
    }

    const QString disabledGates = DisabledGateDescription();
    if (!disabledGates.isEmpty())
    {
        const QString risk = QStringLiteral(
            "以下门禁/互锁将被独立关闭：\n%1\n\n"
            "各项只跳过本行对应检查；保存成功后立即应用，进程单实例项需重启。"
            "紧急 STOP 和当前流程取消仍执行。\n\n确认保存？").arg(disabledGates);
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

    // 并发安全：保存前重新加载最新处理配置，只覆盖本页列出的 25 个门禁开关
    // 和 10 个独立系统互锁；有效性数值门限和处理算法不修改。
    PointCloudProcessingConfig::Settings settings =
        PointCloudProcessingConfig::Load();
    settings.validationCoverageEnabled = m_coverageGateCheck->isChecked();
    settings.validationSdkBaseIntegrityEnabled = m_sdkBaseIntegrityGateCheck->isChecked();
    settings.validationContinuityEnabled = m_continuityGateCheck->isChecked();
    settings.validationDenoiseRatioEnabled = m_denoiseRatioGateCheck->isChecked();
    settings.validationResidualEnabled = m_residualGateCheck->isChecked();
    settings.validationKeyPointEnabled = m_keyPointGateCheck->isChecked();
    settings.validationOutputEnabled = m_outputGateCheck->isChecked();
    settings.validationSegmentHardLimitsEnabled = m_segmentHardLimitsGateCheck->isChecked();
    settings.validationFinalTrajectoryStepEnabled = m_finalTrajectoryStepGateCheck->isChecked();
    settings.validationFinalLengthBindingEnabled = m_finalLengthBindingGateCheck->isChecked();
    settings.validationFinalTopologyBindingEnabled = m_finalTopologyBindingGateCheck->isChecked();
    settings.validationFinalSourceBindingEnabled = m_finalSourceBindingGateCheck->isChecked();
    settings.validationFinalSemanticIntegrityEnabled = m_finalSemanticIntegrityGateCheck->isChecked();
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
    const auto runtimeBeforeSave = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    bool immediateInterlockChanged = false;
    bool singleProcessChanged = false;
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
    {
        const bool enabled = m_mandatoryGateChecks[i]->isChecked();
        if (runtimeBeforeSave.enabled[i] != enabled)
        {
            if (i == static_cast<std::size_t>(SystemInterlock::SingleProcess))
                singleProcessChanged = true;
            else
                immediateInterlockChanged = true;
        }
        settings.systemInterlocks.enabled[i] = enabled;
    }

    QString error;
    if (!PointCloudProcessingConfig::Save(settings, &error))
    {
        QMessageBox::critical(
            this,
            QStringLiteral("保存失败"),
            error.isEmpty()
                ? QStringLiteral("全部可配置门禁保存失败。")
                : error);
        return;
    }

    Reload();
    QMessageBox::information(
        this,
        QStringLiteral("保存成功"),
        QStringLiteral(
            "25 项可配置门禁已保存，将在后续流程中实际生效。"
            "有效性数值门限未被修改。%1%2")
            .arg(immediateInterlockChanged
                ? QStringLiteral("9 项运行互锁的改动已立即应用。")
                : QStringLiteral("9 项运行互锁状态未改变。"))
            .arg(singleProcessChanged
                ? QStringLiteral("机器人控制进程单实例项已保存，将在下次启动时生效。")
                : QString()));
}

void ScanSafetyGateDialog::UpdateSummary()
{
    const QList<QCheckBox*> gateChecks = {
        m_coverageGateCheck,
        m_sdkBaseIntegrityGateCheck,
        m_continuityGateCheck,
        m_denoiseRatioGateCheck,
        m_residualGateCheck,
        m_keyPointGateCheck,
        m_outputGateCheck,
        m_segmentHardLimitsGateCheck,
        m_finalTrajectoryStepGateCheck,
        m_finalLengthBindingGateCheck,
        m_finalTopologyBindingGateCheck,
        m_finalSourceBindingGateCheck,
        m_finalSemanticIntegrityGateCheck,
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

    m_profileSummaryLabel->setText(QStringLiteral("%1/%2 可配置门禁开启")
        .arg(enabledCount)
        .arg(gateChecks.size()));
    m_policySummaryLabel->setText(QStringLiteral("有效性 · 流程身份 · 证明链 · 运动前复核"));
    int enabledInterlocks = 0;
    int changedInterlocks = 0;
    bool singleProcessChanged = false;
    const auto runtime = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
    {
        const bool selected = m_mandatoryGateChecks[i]->isChecked();
        enabledInterlocks += selected ? 1 : 0;
        if (selected != runtime.enabled[i])
        {
            if (i == static_cast<std::size_t>(SystemInterlock::SingleProcess)) singleProcessChanged = true;
            else ++changedInterlocks;
        }
    }
    m_proofSummaryLabel->setText(QStringLiteral("%1/10 系统互锁开启%2")
        .arg(enabledInterlocks).arg(changedInterlocks > 0 || singleProcessChanged
            ? QStringLiteral(" · %1 项待保存%2").arg(changedInterlocks)
                .arg(singleProcessChanged ? QStringLiteral("，单实例项需重启") : QString())
            : QString()));

    const QString normalSummaryStyle = QStringLiteral(
        "color:#8fe5b2; background:#0b151d; border:1px solid #315163;"
        "border-radius:6px; font-weight:600; padding:5px 10px;");
    m_policySummaryLabel->setStyleSheet(normalSummaryStyle);
    const QString disabledSummaryStyle = QStringLiteral(
        "color:#ff9b91; background:#351516; border:1px solid #8d3b3f;"
        "border-radius:6px; font-weight:700; padding:5px 10px;");
    m_proofSummaryLabel->setStyleSheet(
        enabledInterlocks == 10 && changedInterlocks == 0 && !singleProcessChanged
            ? normalSummaryStyle : disabledSummaryStyle);
    UpdateChangeWarning();
}

void ScanSafetyGateDialog::UpdateMandatoryGateStatus()
{
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
    {
        auto* check = m_mandatoryGateChecks[i];
        check->setText(check->isChecked() ? QStringLiteral("开启") : QStringLiteral("关闭"));
    }
}

bool ScanSafetyGateDialog::HasDisabledConfigurableGateUi() const
{
    return !m_coverageGateCheck->isChecked()
        || !m_sdkBaseIntegrityGateCheck->isChecked()
        || !m_continuityGateCheck->isChecked()
        || !m_denoiseRatioGateCheck->isChecked()
        || !m_residualGateCheck->isChecked()
        || !m_keyPointGateCheck->isChecked()
        || !m_outputGateCheck->isChecked()
        || !m_segmentHardLimitsGateCheck->isChecked()
        || !m_finalTrajectoryStepGateCheck->isChecked()
        || !m_finalLengthBindingGateCheck->isChecked()
        || !m_finalTopologyBindingGateCheck->isChecked()
        || !m_finalSourceBindingGateCheck->isChecked()
        || !m_finalSemanticIntegrityGateCheck->isChecked()
        || !m_proofIntegrityGateCheck->isChecked()
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
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
        addIfDisabled(m_mandatoryGateChecks[i], m_mandatoryGateTable->item(static_cast<int>(i), 1)->text());
    addIfDisabled(m_coverageGateCheck, QStringLiteral("点云覆盖与有限点"));
    addIfDisabled(m_sdkBaseIntegrityGateCheck, QStringLiteral("SDKBase 完整点云覆盖"));
    addIfDisabled(m_continuityGateCheck, QStringLiteral("扫描向连续性"));
    addIfDisabled(m_denoiseRatioGateCheck, QStringLiteral("去噪拒绝比例"));
    addIfDisabled(m_residualGateCheck, QStringLiteral("拟合残差"));
    addIfDisabled(m_keyPointGateCheck, QStringLiteral("拐点与分段充分性"));
    addIfDisabled(m_outputGateCheck, QStringLiteral("焊道输出数量与长度"));
    addIfDisabled(m_segmentHardLimitsGateCheck, QStringLiteral("焊道最短分段"));
    addIfDisabled(m_finalTrajectoryStepGateCheck, QStringLiteral("最终轨迹相邻步长"));
    addIfDisabled(m_finalLengthBindingGateCheck, QStringLiteral("最终轨迹长度绑定"));
    addIfDisabled(m_finalTopologyBindingGateCheck, QStringLiteral("最终轨迹拓扑绑定"));
    addIfDisabled(m_finalSourceBindingGateCheck, QStringLiteral("最终轨迹来源位姿绑定"));
    addIfDisabled(m_finalSemanticIntegrityGateCheck, QStringLiteral("最终文件结构与语义完整性"));
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
    QStringList changed;
    QStringList disabled;
    const auto runtime = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    bool singleProcessChanged = false;
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
    {
        const QString name = m_mandatoryGateTable->item(static_cast<int>(i), 1)->text();
        const bool enabled = m_mandatoryGateChecks[i]->isChecked();
        if (enabled != runtime.enabled[i])
        {
            if (i == static_cast<std::size_t>(SystemInterlock::SingleProcess)) singleProcessChanged = true;
            else changed.append(name);
        }
        if (!enabled) disabled.append(name);
    }
    if (!changed.isEmpty() || !disabled.isEmpty())
    {
        m_changeWarningLabel->setStyleSheet(QStringLiteral(
            "background:#351516; border:1px solid #8d3b3f; border-radius:7px;"
            "color:#ffaaa2; padding:10px 12px; font-weight:700;"));
        m_changeWarningLabel->setText(
            (disabled.isEmpty() ? QString() : QStringLiteral("已选择关闭：%1。").arg(disabled.join(QStringLiteral("、"))))
            + (changed.isEmpty() ? QStringLiteral("其他运行互锁独立生效。")
                : QStringLiteral("以下项与当前运行状态不同，保存后立即应用：%1。").arg(changed.join(QStringLiteral("、"))))
            + (singleProcessChanged ? QStringLiteral("机器人控制进程单实例项需重启生效。") : QString()));
        return;
    }
    if (HasDisabledConfigurableGateUi())
    {
        m_changeWarningLabel->setStyleSheet(QStringLiteral(
            "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
            "color:#ffd878; padding:10px 12px; font-weight:600;"));
        m_changeWarningLabel->setText(QStringLiteral(
            "部分可配置门禁已关闭：后续流程会真实跳过对应有效性检查或流程/运动复核。"
            "10 项系统互锁均为开启。"));
        return;
    }

    m_changeWarningLabel->setStyleSheet(QStringLiteral(
        "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
        "color:#ffd878; padding:10px 12px; font-weight:600;"));
    if (m_dirty)
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "存在未保存变更。本页保存 25 项有效性/流程/运动复核开关；"
            "系统互锁逐项保存并立即应用；机器人控制进程单实例项需重启生效。"));
    }
    else
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "25 项可配置门禁均已开启；10 项独立系统互锁当前已开启。"
            "有效性数值门限仍在“测量参数 → 有效性检测”编辑。"));
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
