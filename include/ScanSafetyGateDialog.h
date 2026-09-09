#pragma once

#include <QDialog>
#include "SystemInterlockPolicy.h"

#include <functional>

class QCheckBox;
class QLabel;
class QPushButton;
class QScrollArea;
class QShowEvent;
class QTableWidget;
class QString;

// 管理页中的流程与机器人运动安全门禁配置页。
//
// 本页统一列出所有可配置的点云/焊道/最终轨迹有效性门禁以及
// 流程身份、证明链和运动前复核门禁。数值门限仍在“测量参数/有效性检测”编辑。
// 进程、会话、停机、租约和焊后恢复互锁均在本页独立控制。
// 保存或载入安全默认值前必须通过 modifyGuard 的管理员身份复核。
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
    void BuildQualityGateTable();
    void BuildHardGateTable();
    void BuildMandatoryGateTable();
    void ConnectChangeTracking();
    bool AuthorizeModification(const QString& actionName);
    void RestoreSafetyDefaults();
    void Save();
    void UpdateSummary();
    void UpdateMandatoryGateStatus();
    void UpdateChangeWarning();
    void SetDirty(bool dirty);
    bool HasDisabledConfigurableGateUi() const;
    QString DisabledGateDescription() const;

    std::function<bool()> m_modifyGuard;
    bool m_loading = false;
    bool m_dirty = false;

    QScrollArea* m_scrollArea = nullptr;
    QLabel* m_profileSummaryLabel = nullptr;
    QLabel* m_policySummaryLabel = nullptr;
    QLabel* m_proofSummaryLabel = nullptr;
    QLabel* m_changeWarningLabel = nullptr;

    QCheckBox* m_coverageGateCheck = nullptr;
    QCheckBox* m_sdkBaseIntegrityGateCheck = nullptr;
    QCheckBox* m_continuityGateCheck = nullptr;
    QCheckBox* m_denoiseRatioGateCheck = nullptr;
    QCheckBox* m_residualGateCheck = nullptr;
    QCheckBox* m_keyPointGateCheck = nullptr;
    QCheckBox* m_outputGateCheck = nullptr;
    QCheckBox* m_segmentHardLimitsGateCheck = nullptr;
    QCheckBox* m_finalTrajectoryStepGateCheck = nullptr;
    QCheckBox* m_finalLengthBindingGateCheck = nullptr;
    QCheckBox* m_finalTopologyBindingGateCheck = nullptr;
    QCheckBox* m_finalSourceBindingGateCheck = nullptr;
    QCheckBox* m_finalSemanticIntegrityGateCheck = nullptr;

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
    std::array<QCheckBox*, SystemInterlockCount> m_mandatoryGateChecks{};

    QTableWidget* m_qualityGateTable = nullptr;
    QTableWidget* m_hardGateTable = nullptr;
    QTableWidget* m_mandatoryGateTable = nullptr;
    QPushButton* m_reloadButton = nullptr;
    QPushButton* m_restoreDefaultsButton = nullptr;
    QPushButton* m_saveButton = nullptr;
};
