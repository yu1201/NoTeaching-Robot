#pragma once

#include <QDialog>

#include <functional>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QScrollArea;
class QShowEvent;
class QSpinBox;
class QTableWidget;
class QString;

// 管理页中的扫描安全门禁配置页。
//
// 页面允许工程师查看全部扫描质量和系统安全门禁；保存或载入安全默认值前必须通过
// modifyGuard 的管理员身份复核。关闭核心系统门禁只进入维护审计状态，不能生成可执行证明。
class ScanSafetyGateDialog : public QDialog
{
public:
    explicit ScanSafetyGateDialog(
        std::function<bool()> modifyGuard,
        QWidget* parent = nullptr);

    // 从配置存储重新读取。只读操作，不需要管理员权限。
    void Reload();
    bool HasUnsavedChanges() const noexcept;

protected:
    void showEvent(QShowEvent* event) override;

private:
    void BuildUi();
    void BuildHardGateTable();
    void ConnectChangeTracking();
    bool AuthorizeModification(const QString& actionName);
    void RestoreSafetyDefaults();
    void Save();
    void UpdatePolicyUi();
    void UpdateSummary();
    void UpdateChangeWarning();
    void SetDirty(bool dirty);
    bool HasDisabledCoreSafetyGateUi() const;
    QString DisabledGateDescription() const;

    std::function<bool()> m_modifyGuard;
    bool m_loading = false;
    bool m_dirty = false;

    QScrollArea* m_scrollArea = nullptr;
    QLabel* m_profileSummaryLabel = nullptr;
    QLabel* m_policySummaryLabel = nullptr;
    QLabel* m_proofSummaryLabel = nullptr;
    QLabel* m_policyBoundaryLabel = nullptr;
    QLabel* m_changeWarningLabel = nullptr;

    QComboBox* m_validationPolicyCombo = nullptr;

    QCheckBox* m_coverageEnabledCheck = nullptr;
    QSpinBox* m_minFinitePointCountSpin = nullptr;
    QDoubleSpinBox* m_minProjectedSpanSpin = nullptr;

    QCheckBox* m_continuityEnabledCheck = nullptr;
    QDoubleSpinBox* m_minStationCoverageSpin = nullptr;
    QDoubleSpinBox* m_minLongestContinuousSpin = nullptr;

    QCheckBox* m_denoiseRatioEnabledCheck = nullptr;
    QDoubleSpinBox* m_maxRejectedRatioSpin = nullptr;

    QCheckBox* m_residualEnabledCheck = nullptr;
    QDoubleSpinBox* m_maxMedianResidualSpin = nullptr;
    QDoubleSpinBox* m_maxP95ResidualSpin = nullptr;
    QDoubleSpinBox* m_residualInlierThresholdSpin = nullptr;
    QDoubleSpinBox* m_minResidualInlierRatioSpin = nullptr;

    QCheckBox* m_keyPointEnabledCheck = nullptr;
    QSpinBox* m_minKeyPointCountSpin = nullptr;
    QSpinBox* m_minCornerCountSpin = nullptr;
    QDoubleSpinBox* m_minSegmentLengthSpin = nullptr;

    QCheckBox* m_outputEnabledCheck = nullptr;
    QSpinBox* m_minOutputPointCountSpin = nullptr;
    QDoubleSpinBox* m_minOutputLengthRatioSpin = nullptr;

    QCheckBox* m_proofIntegrityGateCheck = nullptr;
    QCheckBox* m_productionPurposeGateCheck = nullptr;
    QCheckBox* m_robotNameBindingGateCheck = nullptr;
    QCheckBox* m_caseBindingGateCheck = nullptr;
    QCheckBox* m_endpointBindingGateCheck = nullptr;
    QCheckBox* m_cameraHandEyeBindingGateCheck = nullptr;
    QCheckBox* m_freshnessGateCheck = nullptr;
    QCheckBox* m_policySnapshotGateCheck = nullptr;
    QCheckBox* m_inputEvidenceGateCheck = nullptr;
    QCheckBox* m_authorizedPoseIdentityGateCheck = nullptr;
    QCheckBox* m_trajectoryStructureGateCheck = nullptr;
    QCheckBox* m_motionPrecheckGateCheck = nullptr;

    QTableWidget* m_hardGateTable = nullptr;
    QPushButton* m_reloadButton = nullptr;
    QPushButton* m_restoreDefaultsButton = nullptr;
    QPushButton* m_saveButton = nullptr;
};
