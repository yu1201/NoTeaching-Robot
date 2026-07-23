#include "ScanSafetyGateDialog.h"

#include "PointCloudProcessingConfig.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
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
#include <QSpinBox>
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

QDoubleSpinBox* MakeDoubleSpin(
    const QString& objectName,
    double minimum,
    double maximum,
    int decimals,
    double step,
    const QString& suffix = QString())
{
    auto* spin = new QDoubleSpinBox();
    spin->setObjectName(objectName);
    spin->setRange(minimum, maximum);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    spin->setSuffix(suffix);
    spin->setKeyboardTracking(false);
    return spin;
}

QSpinBox* MakeIntSpin(
    const QString& objectName,
    int minimum,
    int maximum,
    int step = 1)
{
    auto* spin = new QSpinBox();
    spin->setObjectName(objectName);
    spin->setRange(minimum, maximum);
    spin->setSingleStep(step);
    spin->setKeyboardTracking(false);
    return spin;
}

QCheckBox* MakeGateCheck(const QString& objectName, const QString& text)
{
    auto* check = new QCheckBox(text);
    check->setObjectName(objectName);
    check->setChecked(true);
    check->setToolTip(QStringLiteral(
        "该门禁可由管理员配置。关闭前会显示风险确认；"
        "已启用质量门禁的 Enforce 数值安全边界不会被放宽。"));
    return check;
}

QLabel* MakeFieldLabel(const QString& text)
{
    auto* label = new QLabel(text);
    label->setProperty("fieldLabel", true);
    return label;
}

QGroupBox* MakeGateGroup(
    const QString& objectName,
    const QString& title,
    QCheckBox* enabledCheck,
    const QList<QPair<QString, QWidget*>>& fields)
{
    auto* group = new QGroupBox(title);
    group->setObjectName(objectName);
    auto* layout = new QGridLayout(group);
    layout->setContentsMargins(16, 18, 16, 14);
    layout->setHorizontalSpacing(12);
    layout->setVerticalSpacing(10);
    layout->addWidget(enabledCheck, 0, 0, 1, 4);
    for (int i = 0; i < fields.size(); ++i)
    {
        const int row = 1 + i / 2;
        const int column = (i % 2) * 2;
        layout->addWidget(MakeFieldLabel(fields.at(i).first), row, column);
        layout->addWidget(fields.at(i).second, row, column + 1);
    }
    layout->setColumnStretch(1, 1);
    layout->setColumnStretch(3, 1);
    return group;
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

PointCloudProcessingConfig::Settings ValidationDefaults()
{
    PointCloudProcessingConfig::Settings defaults;
    defaults.validationPolicy = PointCloudProcessingConfig::ValidationPolicy::Enforce;
    defaults.validationProfileVersion =
        PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION;
    defaults.validationCoverageEnabled = true;
    defaults.validationContinuityEnabled = true;
    defaults.validationDenoiseRatioEnabled = true;
    defaults.validationResidualEnabled = true;
    defaults.validationKeyPointEnabled = true;
    defaults.validationOutputEnabled = true;
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

    auto* heading = new QLabel(QStringLiteral("扫描安全门禁"));
    heading->setObjectName(QStringLiteral("scanSafetyGateHeading"));
    heading->setStyleSheet(QStringLiteral("font-size: 22px; font-weight: 700; color: #eef8fc;"));
    contentLayout->addWidget(heading);

    auto* intro = new QLabel(QStringLiteral(
        "集中配置扫描质量门禁、证明绑定和运动前安全门禁。工程师可查看；"
        "只有通过管理员身份复核后才能保存或恢复默认值。"));
    intro->setObjectName(QStringLiteral("scanSafetyGateIntroLabel"));
    intro->setWordWrap(true);
    contentLayout->addWidget(intro);

    auto* summaryCard = new QFrame();
    summaryCard->setObjectName(QStringLiteral("scanSafetyGateSummaryCard"));
    summaryCard->setProperty("card", true);
    auto* summaryLayout = new QGridLayout(summaryCard);
    summaryLayout->setContentsMargins(16, 14, 16, 14);
    summaryLayout->setHorizontalSpacing(12);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("门禁启用数量")), 0, 0);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("当前有效策略")), 0, 1);
    summaryLayout->addWidget(MakeFieldLabel(QStringLiteral("生产安全状态")), 0, 2);
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

    auto* policyGroup = new QGroupBox(QStringLiteral("执行策略"));
    policyGroup->setObjectName(QStringLiteral("validationPolicyGroup"));
    auto* policyLayout = new QGridLayout(policyGroup);
    policyLayout->setContentsMargins(16, 18, 16, 14);
    m_validationPolicyCombo = new QComboBox();
    m_validationPolicyCombo->setObjectName(QStringLiteral("validationPolicyComboBox"));
    m_validationPolicyCombo->addItem(
        QStringLiteral("Enforce（不通过即拒绝生成可执行证明）"),
        static_cast<int>(PointCloudProcessingConfig::ValidationPolicy::Enforce));
    m_validationPolicyCombo->addItem(
        QStringLiteral("Audit（只计算和记录，不生成可执行证明）"),
        static_cast<int>(PointCloudProcessingConfig::ValidationPolicy::Audit));
    policyLayout->addWidget(MakeFieldLabel(QStringLiteral("策略")), 0, 0);
    policyLayout->addWidget(m_validationPolicyCombo, 0, 1);
    m_policyBoundaryLabel = new QLabel();
    m_policyBoundaryLabel->setObjectName(QStringLiteral("validationPolicyBoundaryLabel"));
    m_policyBoundaryLabel->setWordWrap(true);
    policyLayout->addWidget(m_policyBoundaryLabel, 1, 0, 1, 2);
    policyLayout->setColumnStretch(1, 1);
    contentLayout->addWidget(policyGroup);

    m_coverageEnabledCheck = MakeGateCheck(
        QStringLiteral("validationCoverageEnabledCheckBox"),
        QStringLiteral("启用覆盖范围门禁"));
    m_minFinitePointCountSpin = MakeIntSpin(
        QStringLiteral("validationMinFinitePointCountSpinBox"), 0, 10000000, 50);
    m_minProjectedSpanSpin = MakeDoubleSpin(
        QStringLiteral("validationMinProjectedSpanMmSpinBox"), 0.0, 999999.0, 3, 10.0, QStringLiteral(" mm"));
    contentLayout->addWidget(MakeGateGroup(
        QStringLiteral("validationCoverageGroup"),
        QStringLiteral("1. 采集覆盖"),
        m_coverageEnabledCheck,
        {
            { QStringLiteral("最少有限点数"), m_minFinitePointCountSpin },
            { QStringLiteral("最小投影跨度"), m_minProjectedSpanSpin }
        }));

    m_continuityEnabledCheck = MakeGateCheck(
        QStringLiteral("validationContinuityEnabledCheckBox"),
        QStringLiteral("启用连续性门禁"));
    m_minStationCoverageSpin = MakeDoubleSpin(
        QStringLiteral("validationMinStationCoverageRatioSpinBox"), 0.0, 100.0, 1, 5.0, QStringLiteral("%"));
    m_minLongestContinuousSpin = MakeDoubleSpin(
        QStringLiteral("validationMinLongestContinuousRatioSpinBox"), 0.0, 100.0, 1, 5.0, QStringLiteral("%"));
    contentLayout->addWidget(MakeGateGroup(
        QStringLiteral("validationContinuityGroup"),
        QStringLiteral("2. 扫描连续性"),
        m_continuityEnabledCheck,
        {
            { QStringLiteral("最小站位覆盖率"), m_minStationCoverageSpin },
            { QStringLiteral("最长连续段占比"), m_minLongestContinuousSpin }
        }));

    m_denoiseRatioEnabledCheck = MakeGateCheck(
        QStringLiteral("validationDenoiseRatioEnabledCheckBox"),
        QStringLiteral("启用剔除比例门禁"));
    m_maxRejectedRatioSpin = MakeDoubleSpin(
        QStringLiteral("validationMaxRejectedRatioSpinBox"), 0.0, 100.0, 1, 5.0, QStringLiteral("%"));
    contentLayout->addWidget(MakeGateGroup(
        QStringLiteral("validationDenoiseRatioGroup"),
        QStringLiteral("3. 滤波剔除比例"),
        m_denoiseRatioEnabledCheck,
        {
            { QStringLiteral("最大剔除比例"), m_maxRejectedRatioSpin }
        }));

    m_residualEnabledCheck = MakeGateCheck(
        QStringLiteral("validationResidualEnabledCheckBox"),
        QStringLiteral("启用拟合残差门禁"));
    m_maxMedianResidualSpin = MakeDoubleSpin(
        QStringLiteral("validationMaxMedianResidualMmSpinBox"), 0.0, 9999.0, 3, 0.1, QStringLiteral(" mm"));
    m_maxP95ResidualSpin = MakeDoubleSpin(
        QStringLiteral("validationMaxP95ResidualMmSpinBox"), 0.0, 9999.0, 3, 0.1, QStringLiteral(" mm"));
    m_residualInlierThresholdSpin = MakeDoubleSpin(
        QStringLiteral("validationResidualInlierThresholdMmSpinBox"), 0.0, 9999.0, 3, 0.1, QStringLiteral(" mm"));
    m_minResidualInlierRatioSpin = MakeDoubleSpin(
        QStringLiteral("validationMinResidualInlierRatioSpinBox"), 0.0, 100.0, 1, 5.0, QStringLiteral("%"));
    contentLayout->addWidget(MakeGateGroup(
        QStringLiteral("validationResidualGroup"),
        QStringLiteral("4. 拟合残差"),
        m_residualEnabledCheck,
        {
            { QStringLiteral("最大中位残差"), m_maxMedianResidualSpin },
            { QStringLiteral("最大 P95 残差"), m_maxP95ResidualSpin },
            { QStringLiteral("残差内点阈值"), m_residualInlierThresholdSpin },
            { QStringLiteral("最小残差内点率"), m_minResidualInlierRatioSpin }
        }));

    m_keyPointEnabledCheck = MakeGateCheck(
        QStringLiteral("validationKeyPointEnabledCheckBox"),
        QStringLiteral("启用起终点/拐点门禁"));
    m_minKeyPointCountSpin = MakeIntSpin(
        QStringLiteral("validationMinKeyPointCountSpinBox"), 0, 9999);
    m_minCornerCountSpin = MakeIntSpin(
        QStringLiteral("validationMinCornerCountSpinBox"), 0, 9999);
    m_minSegmentLengthSpin = MakeDoubleSpin(
        QStringLiteral("validationMinSegmentLengthMmSpinBox"), 0.0, 9999.0, 3, 1.0, QStringLiteral(" mm"));
    contentLayout->addWidget(MakeGateGroup(
        QStringLiteral("validationKeyPointGroup"),
        QStringLiteral("5. 起终点与拐点"),
        m_keyPointEnabledCheck,
        {
            { QStringLiteral("最少关键点数"), m_minKeyPointCountSpin },
            { QStringLiteral("最少拐点数"), m_minCornerCountSpin },
            { QStringLiteral("最短分段长度"), m_minSegmentLengthSpin }
        }));

    m_outputEnabledCheck = MakeGateCheck(
        QStringLiteral("validationOutputEnabledCheckBox"),
        QStringLiteral("启用输出结果门禁"));
    m_minOutputPointCountSpin = MakeIntSpin(
        QStringLiteral("validationMinOutputPointCountSpinBox"), 0, 10000000, 10);
    m_minOutputLengthRatioSpin = MakeDoubleSpin(
        QStringLiteral("validationMinOutputLengthRatioSpinBox"), 0.0, 1000.0, 1, 5.0, QStringLiteral("%"));
    contentLayout->addWidget(MakeGateGroup(
        QStringLiteral("validationOutputGroup"),
        QStringLiteral("6. 输出结果"),
        m_outputEnabledCheck,
        {
            { QStringLiteral("最少输出点数"), m_minOutputPointCountSpin },
            { QStringLiteral("最小输出长度比"), m_minOutputLengthRatioSpin }
        }));

    m_changeWarningLabel = new QLabel();
    m_changeWarningLabel->setObjectName(QStringLiteral("validationChangeWarningLabel"));
    m_changeWarningLabel->setWordWrap(true);
    m_changeWarningLabel->setMinimumHeight(48);
    m_changeWarningLabel->setStyleSheet(QStringLiteral(
        "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
        "color:#ffd878; padding:10px 12px; font-weight:600;"));
    contentLayout->addWidget(m_changeWarningLabel);

    auto* hardGateGroup = new QGroupBox(QStringLiteral("系统安全门禁（管理员可配置）"));
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
    auto* actionHint = new QLabel(QStringLiteral("保存和载入安全默认值均会重新验证管理员身份。"));
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
            "证明结构与防篡改",
            "固定 schema/profile；质量证明和 HMAC 收据必须完整、可验签，并与当前案例绑定。",
            "关闭后仅允许维护审计，不能生成可执行证明或进行生产运动。",
            &m_proofIntegrityGateCheck
        },
        {
            "safetyGateProductionPurposeEnabledCheckBox",
            "生产用途",
            "证明用途必须为 production，诊断、预览或测试用途的证明不能进入生产流程。",
            "关闭后仅允许维护审计，不能把非生产用途证明用于真实焊接。",
            &m_productionPurposeGateCheck
        },
        {
            "safetyGateRobotNameBindingEnabledCheckBox",
            "机器人逻辑名称绑定",
            "证明记录的机器人逻辑名称必须与当前所选机器人名称一致。",
            "仅放宽逻辑名称；TCP 持久端点/控制单元、相机及手眼身份仍继续强制匹配。",
            &m_robotNameBindingGateCheck
        },
        {
            "safetyGateCaseBindingEnabledCheckBox",
            "案例目录绑定",
            "证明中的规范化案例目录必须与当前运行案例目录一致。",
            "关闭后仅允许维护审计，跨案例证据不能用于生产。",
            &m_caseBindingGateCheck
        },
        {
            "safetyGateEndpointBindingEnabledCheckBox",
            "TCP 持久端点/控制单元绑定",
            "持久化 TCP 端点和控制单元身份必须与生成证明时一致。",
            "关闭后仅允许维护审计，不能向真实机器人下发生产运动。",
            &m_endpointBindingGateCheck
        },
        {
            "safetyGateCameraHandEyeBindingEnabledCheckBox",
            "相机与手眼绑定",
            "定位相机身份、相机配置和手眼标定身份必须与生成证明时一致。",
            "关闭后仅允许维护审计；更换相机或重新标定后的旧证明不能用于生产。",
            &m_cameraHandEyeBindingGateCheck
        },
        {
            "safetyGateFreshnessEnabledCheckBox",
            "证明新鲜度",
            "质量证明及原始扫描不超过 24 小时；时间戳最多允许未来偏差 5 分钟。",
            "关闭后仅允许维护审计，过期证明不能生成生产运动。",
            &m_freshnessGateCheck
        },
        {
            "safetyGatePolicySnapshotEnabledCheckBox",
            "处理策略与阈值快照",
            "处理模式、特征策略、质量策略和所有 validation 阈值快照必须与当前配置一致。",
            "关闭后仅允许维护审计，策略变化后的旧证明不能用于生产。",
            &m_policySnapshotGateCheck
        },
        {
            "safetyGateInputEvidenceEnabledCheckBox",
            "输入证据身份",
            "原始点云、扫描元数据及关键输入证据的路径、大小和摘要必须匹配。",
            "关闭后仅允许维护审计，被替换的输入不能用于生产。",
            &m_inputEvidenceGateCheck
        },
        {
            "safetyGateAuthorizedPoseIdentityEnabledCheckBox",
            "授权焊道与位姿身份",
            "生成轨迹所依赖的焊道、特征输出和授权位姿身份必须与证明记录一致。",
            "关闭后仅允许维护审计，未经证明授权的位姿不能用于真实焊接。",
            &m_authorizedPoseIdentityGateCheck
        },
        {
            "safetyGateTrajectoryStructureEnabledCheckBox",
            "轨迹结构",
            "轨迹点数量、段结构、字段完整性及数值有限性均需通过结构校验。",
            "关闭后仅允许维护审计，畸形或非有限轨迹不能下发。",
            &m_trajectoryStructureGateCheck
        },
        {
            "safetyGateMotionPrecheckEnabledCheckBox",
            "运动前复核与限值",
            "每次运动前重新读取配置、绑定和证据，并检查机器人位姿、关节/笛卡尔值及工艺限值。",
            "关闭后仅允许维护审计；禁止真实焊接和机器人生产运动。",
            &m_motionPrecheckGateCheck
        }
    };

    m_hardGateTable->setColumnCount(4);
    m_hardGateTable->setHorizontalHeaderLabels({
        QStringLiteral("开关"),
        QStringLiteral("系统门禁"),
        QStringLiteral("复核内容"),
        QStringLiteral("关闭影响")
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

    connect(m_validationPolicyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this, changed](int)
    {
        UpdatePolicyUi();
        UpdateSummary();
        changed();
    });

    const QList<QCheckBox*> gateChecks = {
        m_coverageEnabledCheck,
        m_continuityEnabledCheck,
        m_denoiseRatioEnabledCheck,
        m_residualEnabledCheck,
        m_keyPointEnabledCheck,
        m_outputEnabledCheck,
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
            UpdatePolicyUi();
            UpdateSummary();
            changed();
        });
    }

    const QList<QSpinBox*> intSpins = {
        m_minFinitePointCountSpin,
        m_minKeyPointCountSpin,
        m_minCornerCountSpin,
        m_minOutputPointCountSpin
    };
    for (QSpinBox* spin : intSpins)
    {
        connect(spin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [changed](int) { changed(); });
    }

    const QList<QDoubleSpinBox*> doubleSpins = {
        m_minProjectedSpanSpin,
        m_minStationCoverageSpin,
        m_minLongestContinuousSpin,
        m_maxRejectedRatioSpin,
        m_maxMedianResidualSpin,
        m_maxP95ResidualSpin,
        m_residualInlierThresholdSpin,
        m_minResidualInlierRatioSpin,
        m_minSegmentLengthSpin,
        m_minOutputLengthRatioSpin
    };
    for (QDoubleSpinBox* spin : doubleSpins)
    {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [changed](double) { changed(); });
    }
}

void ScanSafetyGateDialog::Reload()
{
    const PointCloudProcessingConfig::Settings settings =
        PointCloudProcessingConfig::Load();

    m_loading = true;
    const int policyIndex = m_validationPolicyCombo->findData(
        static_cast<int>(settings.validationPolicy));
    m_validationPolicyCombo->setCurrentIndex(policyIndex >= 0 ? policyIndex : 0);
    UpdatePolicyUi();

    m_coverageEnabledCheck->setChecked(settings.validationCoverageEnabled);
    m_minFinitePointCountSpin->setValue(settings.validationMinFinitePointCount);
    m_minProjectedSpanSpin->setValue(settings.validationMinProjectedSpanMm);

    m_continuityEnabledCheck->setChecked(settings.validationContinuityEnabled);
    m_minStationCoverageSpin->setValue(settings.validationMinStationCoverageRatio * 100.0);
    m_minLongestContinuousSpin->setValue(settings.validationMinLongestContinuousRatio * 100.0);

    m_denoiseRatioEnabledCheck->setChecked(settings.validationDenoiseRatioEnabled);
    m_maxRejectedRatioSpin->setValue(settings.validationMaxRejectedRatio * 100.0);

    m_residualEnabledCheck->setChecked(settings.validationResidualEnabled);
    m_maxMedianResidualSpin->setValue(settings.validationMaxMedianResidualMm);
    m_maxP95ResidualSpin->setValue(settings.validationMaxP95ResidualMm);
    m_residualInlierThresholdSpin->setValue(settings.validationResidualInlierThresholdMm);
    m_minResidualInlierRatioSpin->setValue(settings.validationMinResidualInlierRatio * 100.0);

    m_keyPointEnabledCheck->setChecked(settings.validationKeyPointEnabled);
    m_minKeyPointCountSpin->setValue(settings.validationMinKeyPointCount);
    m_minCornerCountSpin->setValue(settings.validationMinCornerCount);
    m_minSegmentLengthSpin->setValue(settings.validationMinSegmentLengthMm);

    m_outputEnabledCheck->setChecked(settings.validationOutputEnabled);
    m_minOutputPointCountSpin->setValue(settings.validationMinOutputPointCount);
    m_minOutputLengthRatioSpin->setValue(settings.validationMinOutputLengthRatio * 100.0);

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

    UpdatePolicyUi();
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
        QStringLiteral("%1未通过管理员身份复核。工程师账号可以查看，但不能修改扫描安全门禁。")
            .arg(actionName));
    return false;
}

void ScanSafetyGateDialog::RestoreSafetyDefaults()
{
    if (!AuthorizeModification(QStringLiteral("载入安全默认值")))
    {
        return;
    }

    const PointCloudProcessingConfig::Settings defaults = ValidationDefaults();
    m_loading = true;
    m_validationPolicyCombo->setCurrentIndex(
        m_validationPolicyCombo->findData(
            static_cast<int>(PointCloudProcessingConfig::ValidationPolicy::Enforce)));
    m_coverageEnabledCheck->setChecked(defaults.validationCoverageEnabled);
    m_continuityEnabledCheck->setChecked(defaults.validationContinuityEnabled);
    m_denoiseRatioEnabledCheck->setChecked(defaults.validationDenoiseRatioEnabled);
    m_residualEnabledCheck->setChecked(defaults.validationResidualEnabled);
    m_keyPointEnabledCheck->setChecked(defaults.validationKeyPointEnabled);
    m_outputEnabledCheck->setChecked(defaults.validationOutputEnabled);
    m_minFinitePointCountSpin->setValue(defaults.validationMinFinitePointCount);
    m_minProjectedSpanSpin->setValue(defaults.validationMinProjectedSpanMm);
    m_minStationCoverageSpin->setValue(defaults.validationMinStationCoverageRatio * 100.0);
    m_minLongestContinuousSpin->setValue(defaults.validationMinLongestContinuousRatio * 100.0);
    m_maxRejectedRatioSpin->setValue(defaults.validationMaxRejectedRatio * 100.0);
    m_maxMedianResidualSpin->setValue(defaults.validationMaxMedianResidualMm);
    m_maxP95ResidualSpin->setValue(defaults.validationMaxP95ResidualMm);
    m_residualInlierThresholdSpin->setValue(defaults.validationResidualInlierThresholdMm);
    m_minResidualInlierRatioSpin->setValue(defaults.validationMinResidualInlierRatio * 100.0);
    m_minKeyPointCountSpin->setValue(defaults.validationMinKeyPointCount);
    m_minCornerCountSpin->setValue(defaults.validationMinCornerCount);
    m_minSegmentLengthSpin->setValue(defaults.validationMinSegmentLengthMm);
    m_minOutputPointCountSpin->setValue(defaults.validationMinOutputPointCount);
    m_minOutputLengthRatioSpin->setValue(defaults.validationMinOutputLengthRatio * 100.0);
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
    UpdatePolicyUi();
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
    if (!AuthorizeModification(QStringLiteral("保存扫描安全门禁")))
    {
        return;
    }

    const QString disabledGates = DisabledGateDescription();
    if (!disabledGates.isEmpty())
    {
        QString risk = QStringLiteral(
            "以下门禁将被关闭：\n%1\n\n"
            "关闭门禁会放宽扫描证明复核。").arg(disabledGates);
        if (HasDisabledCoreSafetyGateUi())
        {
            risk += QStringLiteral(
                "\n\n维护审计状态：禁止生成可执行证明、真实焊接和机器人生产运动。");
        }
        if (!m_robotNameBindingGateCheck->isChecked())
        {
            risk += QStringLiteral(
                "\n\n机器人名称关闭只放宽逻辑名称；TCP 持久端点/控制单元、"
                "相机及手眼身份仍强制匹配。");
        }
        risk += QStringLiteral("\n\n确认以管理员身份保存这些关闭项吗？");
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

    // 并发安全：保存前重新加载全部最新处理配置，只覆盖本页负责的 validation 字段，
    // 避免覆盖另一个管理页刚保存的点云处理参数。
    PointCloudProcessingConfig::Settings settings =
        PointCloudProcessingConfig::Load();
    settings.validationProfileVersion =
        PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION;
    settings.validationPolicy = static_cast<PointCloudProcessingConfig::ValidationPolicy>(
        m_validationPolicyCombo->currentData().toInt());
    settings.validationCoverageEnabled = m_coverageEnabledCheck->isChecked();
    settings.validationMinFinitePointCount = m_minFinitePointCountSpin->value();
    settings.validationMinProjectedSpanMm = m_minProjectedSpanSpin->value();
    settings.validationContinuityEnabled = m_continuityEnabledCheck->isChecked();
    settings.validationMinStationCoverageRatio = m_minStationCoverageSpin->value() / 100.0;
    settings.validationMinLongestContinuousRatio = m_minLongestContinuousSpin->value() / 100.0;
    settings.validationDenoiseRatioEnabled = m_denoiseRatioEnabledCheck->isChecked();
    settings.validationMaxRejectedRatio = m_maxRejectedRatioSpin->value() / 100.0;
    settings.validationResidualEnabled = m_residualEnabledCheck->isChecked();
    settings.validationMaxMedianResidualMm = m_maxMedianResidualSpin->value();
    settings.validationMaxP95ResidualMm = m_maxP95ResidualSpin->value();
    settings.validationResidualInlierThresholdMm = m_residualInlierThresholdSpin->value();
    settings.validationMinResidualInlierRatio = m_minResidualInlierRatioSpin->value() / 100.0;
    settings.validationKeyPointEnabled = m_keyPointEnabledCheck->isChecked();
    settings.validationMinKeyPointCount = m_minKeyPointCountSpin->value();
    settings.validationMinCornerCount = m_minCornerCountSpin->value();
    settings.validationMinSegmentLengthMm = m_minSegmentLengthSpin->value();
    settings.validationOutputEnabled = m_outputEnabledCheck->isChecked();
    settings.validationMinOutputPointCount = m_minOutputPointCountSpin->value();
    settings.validationMinOutputLengthRatio = m_minOutputLengthRatioSpin->value() / 100.0;
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

    const bool savedInMaintenanceAudit =
        PointCloudProcessingConfig::HasDisabledCoreSafetyGate(settings);

    QString error;
    if (!PointCloudProcessingConfig::Save(settings, &error))
    {
        QMessageBox::critical(
            this,
            QStringLiteral("保存失败"),
            error.isEmpty() ? QStringLiteral("扫描安全门禁保存失败。") : error);
        return;
    }

    Reload();
    QMessageBox::information(
        this,
        QStringLiteral("保存成功"),
        savedInMaintenanceAudit
            ? QStringLiteral(
                "扫描安全门禁已保存并进入维护审计状态。禁止生成可执行证明、"
                "真实焊接和机器人生产运动；重新开启全部核心系统门禁、"
                "选择 Enforce 并保存后才能恢复。")
            : QStringLiteral(
                "扫描安全门禁已保存。旧质量证明因门禁或阈值快照发生变化而失效；"
                "继续生产前必须从原始点云重新构建质量证明。"));
}

void ScanSafetyGateDialog::UpdatePolicyUi()
{
    const bool configuredEnforce =
        m_validationPolicyCombo->currentData().toInt()
        == static_cast<int>(PointCloudProcessingConfig::ValidationPolicy::Enforce);
    const bool maintenanceAudit = HasDisabledCoreSafetyGateUi();

    // Audit 可以用于诊断性观察；Enforce 的安全地板/上限不能从界面放宽。
    m_minFinitePointCountSpin->setRange(configuredEnforce ? 300 : 0, 10000000);
    m_minProjectedSpanSpin->setRange(configuredEnforce ? 180.0 : 0.0, 999999.0);
    m_minStationCoverageSpin->setRange(configuredEnforce ? 55.0 : 0.0, 100.0);
    m_minLongestContinuousSpin->setRange(configuredEnforce ? 60.0 : 0.0, 100.0);
    m_maxRejectedRatioSpin->setRange(0.0, configuredEnforce ? 40.0 : 100.0);
    m_maxMedianResidualSpin->setRange(0.0, configuredEnforce ? 3.0 : 9999.0);
    m_maxP95ResidualSpin->setRange(0.0, configuredEnforce ? 8.0 : 9999.0);
    m_residualInlierThresholdSpin->setRange(0.0, configuredEnforce ? 6.0 : 9999.0);
    m_minResidualInlierRatioSpin->setRange(configuredEnforce ? 75.0 : 0.0, 100.0);
    m_minKeyPointCountSpin->setRange(configuredEnforce ? 6 : 0, 9999);
    m_minCornerCountSpin->setRange(configuredEnforce ? 4 : 0, 9999);
    m_minOutputPointCountSpin->setRange(configuredEnforce ? 80 : 0, 10000000);
    m_minOutputLengthRatioSpin->setRange(configuredEnforce ? 70.0 : 0.0, 1000.0);

    m_minFinitePointCountSpin->setEnabled(m_coverageEnabledCheck->isChecked());
    m_minProjectedSpanSpin->setEnabled(m_coverageEnabledCheck->isChecked());
    m_minStationCoverageSpin->setEnabled(m_continuityEnabledCheck->isChecked());
    m_minLongestContinuousSpin->setEnabled(m_continuityEnabledCheck->isChecked());
    m_maxRejectedRatioSpin->setEnabled(m_denoiseRatioEnabledCheck->isChecked());
    m_maxMedianResidualSpin->setEnabled(m_residualEnabledCheck->isChecked());
    m_maxP95ResidualSpin->setEnabled(m_residualEnabledCheck->isChecked());
    m_residualInlierThresholdSpin->setEnabled(m_residualEnabledCheck->isChecked());
    m_minResidualInlierRatioSpin->setEnabled(m_residualEnabledCheck->isChecked());
    m_minKeyPointCountSpin->setEnabled(m_keyPointEnabledCheck->isChecked());
    m_minCornerCountSpin->setEnabled(m_keyPointEnabledCheck->isChecked());
    m_minSegmentLengthSpin->setEnabled(m_keyPointEnabledCheck->isChecked());
    m_minOutputPointCountSpin->setEnabled(m_outputEnabledCheck->isChecked());
    m_minOutputLengthRatioSpin->setEnabled(m_outputEnabledCheck->isChecked());

    if (maintenanceAudit)
    {
        m_policyBoundaryLabel->setText(configuredEnforce
            ? QStringLiteral(
                "有效策略为 Audit：已有核心系统门禁关闭。配置的 Enforce 数值安全边界仍保留，"
                "但维护审计状态禁止生成可执行证明、真实焊接和机器人生产运动。")
            : QStringLiteral(
                "有效策略为 Audit：已有核心系统门禁关闭。维护审计状态禁止生成可执行证明、"
                "真实焊接和机器人生产运动。"));
        m_policyBoundaryLabel->setStyleSheet(QStringLiteral("color:#ff9a9f; font-weight:700;"));
    }
    else if (configuredEnforce)
    {
        m_policyBoundaryLabel->setText(QStringLiteral(
            "已启用质量门禁的 Enforce 安全边界已锁定：有限点≥300、跨度≥180 mm、站位覆盖≥55%、"
            "最长连续段≥60%、剔除≤40%、中位/P95残差≤3/8 mm、残差内点阈值≤6 mm、"
            "内点率≥75%、关键点/拐点≥6/4、输出点≥80、输出长度比≥70%。"));
        if (!m_robotNameBindingGateCheck->isChecked())
        {
            m_policyBoundaryLabel->setText(m_policyBoundaryLabel->text()
                + QStringLiteral(
                    "\n机器人逻辑名称已放宽；TCP 持久端点/控制单元、相机及手眼身份仍强制匹配。"));
        }
        m_policyBoundaryLabel->setStyleSheet(QStringLiteral("color:#8fe5b2;"));
    }
    else
    {
        m_policyBoundaryLabel->setText(QStringLiteral(
            "Audit 只计算和记录全部门禁指标，不生成可用于机器人生产运动的质量证明。"));
        if (!m_robotNameBindingGateCheck->isChecked())
        {
            m_policyBoundaryLabel->setText(m_policyBoundaryLabel->text()
                + QStringLiteral(
                    "\n机器人逻辑名称已放宽；TCP 持久端点/控制单元、相机及手眼身份仍强制匹配。"));
        }
        m_policyBoundaryLabel->setStyleSheet(QStringLiteral("color:#ffd878;"));
    }
}

void ScanSafetyGateDialog::UpdateSummary()
{
    const bool configuredEnforce =
        m_validationPolicyCombo->currentData().toInt()
        == static_cast<int>(PointCloudProcessingConfig::ValidationPolicy::Enforce);
    const bool maintenanceAudit = HasDisabledCoreSafetyGateUi();
    const bool qualityRelaxed =
        !m_coverageEnabledCheck->isChecked()
        || !m_continuityEnabledCheck->isChecked()
        || !m_denoiseRatioEnabledCheck->isChecked()
        || !m_residualEnabledCheck->isChecked()
        || !m_keyPointEnabledCheck->isChecked()
        || !m_outputEnabledCheck->isChecked();
    const QList<QCheckBox*> gateChecks = {
        m_coverageEnabledCheck,
        m_continuityEnabledCheck,
        m_denoiseRatioEnabledCheck,
        m_residualEnabledCheck,
        m_keyPointEnabledCheck,
        m_outputEnabledCheck,
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

    m_profileSummaryLabel->setText(QStringLiteral("%1/%2 开启 · Profile v%3")
        .arg(enabledCount)
        .arg(gateChecks.size())
        .arg(PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION));
    m_policySummaryLabel->setText(maintenanceAudit
        ? QStringLiteral("有效 Audit · 核心门禁关闭")
        : (configuredEnforce
            ? QStringLiteral("Enforce · 失败即拒绝")
            : QStringLiteral("Audit · 仅审计、不可执行")));
    m_proofSummaryLabel->setText(maintenanceAudit
        ? QStringLiteral("维护审计状态 · 禁止生产")
        : (!configuredEnforce
            ? QStringLiteral("仅审计 · 禁止生产")
            : (!m_robotNameBindingGateCheck->isChecked() && qualityRelaxed
                ? QStringLiteral("名称/质量已放宽 · 端点强制")
                : (!m_robotNameBindingGateCheck->isChecked()
                    ? QStringLiteral("逻辑名称放宽 · 端点强制")
                    : (qualityRelaxed
                        ? QStringLiteral("质量门禁已放宽 · Enforce")
                        : QStringLiteral("生产安全状态 · 正常"))))));

    const QString normalSummaryStyle = QStringLiteral(
        "color:#8fe5b2; background:#0b151d; border:1px solid #315163;"
        "border-radius:6px; font-weight:600; padding:5px 10px;");
    const QString dangerSummaryStyle = QStringLiteral(
        "color:#ff9a9f; background:#301216; border:1px solid #a5444d;"
        "border-radius:6px; font-weight:700; padding:5px 10px;");
    const QString warningSummaryStyle = QStringLiteral(
        "color:#ffd878; background:#392b0c; border:1px solid #8a6820;"
        "border-radius:6px; font-weight:700; padding:5px 10px;");
    m_policySummaryLabel->setStyleSheet(maintenanceAudit
        ? dangerSummaryStyle
        : normalSummaryStyle);
    m_proofSummaryLabel->setStyleSheet(
        maintenanceAudit || !configuredEnforce
            ? dangerSummaryStyle
            : (qualityRelaxed || !m_robotNameBindingGateCheck->isChecked()
                ? warningSummaryStyle
                : normalSummaryStyle));
    UpdateChangeWarning();
}

bool ScanSafetyGateDialog::HasDisabledCoreSafetyGateUi() const
{
    // 机器人逻辑名称是唯一允许单独放宽而不进入维护审计状态的系统门禁。
    return !m_proofIntegrityGateCheck->isChecked()
        || !m_productionPurposeGateCheck->isChecked()
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
    addIfDisabled(m_coverageEnabledCheck, QStringLiteral("采集覆盖"));
    addIfDisabled(m_continuityEnabledCheck, QStringLiteral("扫描连续性"));
    addIfDisabled(m_denoiseRatioEnabledCheck, QStringLiteral("滤波剔除比例"));
    addIfDisabled(m_residualEnabledCheck, QStringLiteral("拟合残差"));
    addIfDisabled(m_keyPointEnabledCheck, QStringLiteral("起终点与拐点"));
    addIfDisabled(m_outputEnabledCheck, QStringLiteral("输出结果"));
    addIfDisabled(m_proofIntegrityGateCheck, QStringLiteral("证明结构与防篡改"));
    addIfDisabled(m_productionPurposeGateCheck, QStringLiteral("生产用途"));
    addIfDisabled(m_robotNameBindingGateCheck, QStringLiteral("机器人逻辑名称绑定"));
    addIfDisabled(m_caseBindingGateCheck, QStringLiteral("案例目录绑定"));
    addIfDisabled(m_endpointBindingGateCheck, QStringLiteral("TCP 持久端点/控制单元绑定"));
    addIfDisabled(m_cameraHandEyeBindingGateCheck, QStringLiteral("相机与手眼绑定"));
    addIfDisabled(m_freshnessGateCheck, QStringLiteral("证明新鲜度"));
    addIfDisabled(m_policySnapshotGateCheck, QStringLiteral("处理策略与阈值快照"));
    addIfDisabled(m_inputEvidenceGateCheck, QStringLiteral("输入证据身份"));
    addIfDisabled(m_authorizedPoseIdentityGateCheck, QStringLiteral("授权焊道与位姿身份"));
    addIfDisabled(m_trajectoryStructureGateCheck, QStringLiteral("轨迹结构"));
    addIfDisabled(m_motionPrecheckGateCheck, QStringLiteral("运动前复核与限值"));
    return disabled.join(QLatin1Char('\n'));
}

void ScanSafetyGateDialog::UpdateChangeWarning()
{
    if (HasDisabledCoreSafetyGateUi())
    {
        m_changeWarningLabel->setStyleSheet(QStringLiteral(
            "background:#301216; border:1px solid #a5444d; border-radius:7px;"
            "color:#ff9a9f; padding:10px 12px; font-weight:700;"));
        m_changeWarningLabel->setText(QStringLiteral(
            "维护审计状态：禁止生成可执行证明、真实焊接和机器人生产运动。"
            "重新开启全部核心系统门禁、选择 Enforce 并保存后才能恢复生产。"));
        return;
    }

    m_changeWarningLabel->setStyleSheet(QStringLiteral(
        "background:#392b0c; border:1px solid #8a6820; border-radius:7px;"
        "color:#ffd878; padding:10px 12px; font-weight:600;"));
    if (!m_robotNameBindingGateCheck->isChecked())
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "机器人逻辑名称门禁已关闭：只放宽逻辑名称；TCP 持久端点/控制单元、"
            "相机及手眼身份仍强制匹配，可继续使用 Enforce。"));
    }
    else if (m_dirty)
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "存在未保存变更。保存后，使用旧门禁、旧处理策略或旧阈值快照的质量证明都会失效；"
            "必须从原始点云重新处理并生成新证明。"));
    }
    else
    {
        m_changeWarningLabel->setText(QStringLiteral(
            "安全提示：任何门禁、执行策略或 validation 阈值变更都会使旧质量证明失效；"
            "继续生产前必须从原始点云重新构建。"));
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
