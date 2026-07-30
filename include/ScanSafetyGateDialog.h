#pragma once

#include <QDialog>

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
// 点云、焊道和最终轨迹的有效性检查统一由“测量参数/有效性检测”页面负责。
// 本页只控制流程身份、证明链和机器人运动前复核；各开关会在生产校验点实际生效。
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
    void BuildHardGateTable();
    void ConnectChangeTracking();
    bool AuthorizeModification(const QString& actionName);
    void RestoreSafetyDefaults();
    void Save();
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
    QLabel* m_changeWarningLabel = nullptr;

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
