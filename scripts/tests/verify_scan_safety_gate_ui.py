"""Static contract checks for the scan-safety-gate management page."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def section(text: str, start: str, end: str | None = None) -> str:
    begin = text.index(start)
    finish = len(text) if end is None else text.index(end, begin + len(start))
    return text[begin:finish]


main = read("src/QtWidgetsApplication4.cpp")
main_header = read("include/QtWidgetsApplication4.h")
dialog = read("src/ScanSafetyGateDialog.cpp")
dialog_header = read("include/ScanSafetyGateDialog.h")
config = read("src/PointCloudProcessingConfig.cpp")
config_header = read("include/PointCloudProcessingConfig.h")
service = read("src/MeasureThenWeldService.cpp")
legacy_processing_dialog = read("src/LaserWeldFilterDialog.cpp")
project = read("QtWidgetsApplication4.vcxproj")
filters = read("QtWidgetsApplication4.vcxproj.filters")

quality_names = (
    "Coverage",
    "Continuity",
    "DenoiseRatio",
    "Residual",
    "KeyPoint",
    "Output",
)
safety_names = (
    "ProofIntegrity",
    "ProductionPurpose",
    "RobotNameBinding",
    "CaseBinding",
    "EndpointBinding",
    "CameraHandEyeBinding",
    "Freshness",
    "PolicySnapshot",
    "InputEvidence",
    "AuthorizedPoseIdentity",
    "TrajectoryStructure",
    "MotionPrecheck",
)

assert '#include "ScanSafetyGateDialog.h"' in main
assert 'createManagementAction("流程与运动安全门禁"' in main
assert "void QtWidgetsApplication4::OpenScanSafetyGatePage()" in main
assert "ShowManagementEmbeddedPage(m_pScanSafetyGatePage)" in main
assert "new ScanSafetyGateDialog(modifyGuard, m_pManagementStack)" in main
assert "RoleLevel(kRoleEngineer)" in section(
    main,
    "void QtWidgetsApplication4::OpenScanSafetyGatePage()",
    "void QtWidgetsApplication4::OpenModelAlignmentPage()",
)
assert "RoleLevel(kRoleAdmin)" in section(
    main,
    "void QtWidgetsApplication4::OpenScanSafetyGatePage()",
    "void QtWidgetsApplication4::OpenModelAlignmentPage()",
)
assert "ScanSafetyGateDialog* m_pScanSafetyGatePage = nullptr;" in main_header
assert 'ClCompile Include="src\\ScanSafetyGateDialog.cpp"' in project
assert 'ClInclude Include="include\\ScanSafetyGateDialog.h"' in project
assert 'ClCompile Include="src\\ScanSafetyGateDialog.cpp"' in filters
assert 'ClInclude Include="include\\ScanSafetyGateDialog.h"' in filters

assert "std::function<bool()> modifyGuard" in dialog_header
assert "void Reload();" in dialog_header
assert "void Save();" in dialog_header
assert "QScrollArea" in dialog_header
assert "setWindowFlags(Qt::Widget)" in dialog
show_body = section(dialog, "void ScanSafetyGateDialog::showEvent(QShowEvent* event)")
assert "if (!m_dirty)" in show_body and "Reload();" in show_body
assert "QMessageBox::warning(" in section(
    dialog, "void ScanSafetyGateDialog::Save()", "void ScanSafetyGateDialog::UpdateSummary()"
)
assert "确认关闭安全门禁" in dialog
assert "PointCloudProcessingConfig::Load();" in section(
    dialog, "void ScanSafetyGateDialog::Save()", "void ScanSafetyGateDialog::UpdateSummary()"
)
assert "PointCloudProcessingConfig::Save(settings, &error)" in dialog

reload_body = section(
    dialog,
    "void ScanSafetyGateDialog::Reload()",
    "bool ScanSafetyGateDialog::HasUnsavedChanges()",
)
save_body = section(
    dialog,
    "void ScanSafetyGateDialog::Save()",
    "void ScanSafetyGateDialog::UpdateSummary()",
)
defaults_body = section(
    dialog,
    "PointCloudProcessingConfig::Settings SafetyGateDefaults()",
    "ScanSafetyGateDialog::ScanSafetyGateDialog(",
)

legacy_load = section(
    legacy_processing_dialog,
    "void LaserWeldFilterDialog::LoadSettings()",
    "bool LaserWeldFilterDialog::SaveSettings(QString* error) const",
)
legacy_save = section(
    legacy_processing_dialog,
    "bool LaserWeldFilterDialog::SaveSettings(QString* error) const",
    "void LaserWeldFilterDialog::LoadExternalAlgorithmConfig()",
)
quality_widgets = {
    "Coverage": "m_pValidationCoverageCheck",
    "Continuity": "m_pValidationContinuityCheck",
    "DenoiseRatio": "m_pValidationDenoiseRatioCheck",
    "Residual": "m_pValidationResidualCheck",
    "KeyPoint": "m_pValidationKeyPointCheck",
    "Output": "m_pValidationOutputCheck",
}
fixed_validity_widgets = {
    "SegmentHardLimits": "m_pValidationSegmentHardLimitsCheck",
    "FinalTrajectoryStep": "m_pValidationFinalTrajectoryStepCheck",
    "FinalLengthBinding": "m_pValidationFinalLengthBindingCheck",
    "FinalTopologyBinding": "m_pValidationFinalTopologyBindingCheck",
    "FinalSourceBinding": "m_pValidationFinalSourceBindingCheck",
    "FinalSemanticIntegrity": "m_pValidationFinalSemanticIntegrityCheck",
}
for name in quality_names:
    field = f"validation{name}Enabled"
    assert f"bool {field} = true;" in config_header
    assert field not in defaults_body
    assert field not in reload_body
    assert field not in save_body
    assert f'ReadBoolSetting("Validation/{name}Enabled"' in config
    assert f'write("Validation/{name}Enabled"' in config
    widget = quality_widgets[name]
    assert f"{widget}->setChecked(processingSettings.{field})" in legacy_load
    assert f"processingSettings.{field} = {widget}->isChecked();" in legacy_save
    assert widget not in dialog

for name, widget in fixed_validity_widgets.items():
    field = f"validation{name}Enabled"
    assert f"bool {field} = true;" in config_header
    assert field not in defaults_body
    assert field not in reload_body
    assert field not in save_body
    assert f'ReadBoolSetting("Validation/{name}Enabled"' in config
    assert f'write("Validation/{name}Enabled"' in config
    assert (f"{widget}->setChecked(\n"
            f"        processingSettings.{field});") in legacy_load
    assert f"processingSettings.{field} =" in legacy_save
    assert widget in legacy_processing_dialog
    assert widget not in dialog

for name in safety_names:
    field = f"safetyGate{name}Enabled"
    assert f"bool {field} = true;" in config_header
    assert f"defaults.{field} = true;" in defaults_body
    assert field in reload_body
    assert field in save_body
    assert f'ReadBoolSetting("SafetyGates/{name}Enabled"' in config
    assert f'write("SafetyGates/{name}Enabled"' in config
    assert f"safetyGate{name}EnabledCheckBox" in dialog

core_helper = section(
    config,
    "bool PointCloudProcessingConfig::CoreSafetyGatesEnabled(",
    "bool PointCloudProcessingConfig::HasDisabledCoreSafetyGate(",
)
for name in safety_names:
    assert f"safetyGate{name}Enabled" in core_helper
assert "CURRENT_SAFETY_GATE_BEHAVIOR_VERSION = 2" in config_header
assert 'ReadIntSetting("SafetyGates/BehaviorVersion", 0)' in config
assert "storedSafetyGateBehaviorVersion < 1" in config
assert "settings.validationPolicy = ValidationPolicy::Enforce;" in config
assert "settings.validationPolicy = ValidationPolicy::Audit;" not in config
assert 'write("SafetyGates/BehaviorVersion"' in config

ui_core_helper = section(
    dialog,
    "bool ScanSafetyGateDialog::HasDisabledCoreSafetyGateUi() const",
    "QString ScanSafetyGateDialog::DisabledGateDescription() const",
)
assert "m_robotNameBindingGateCheck" in ui_core_helper
assert "维护审计状态" not in dialog
assert "流程与机器人运动安全门禁" in dialog
assert "测量参数 → 有效性检测" in dialog
assert "不会改动有效性检测页配置" in dialog
assert "流程与运动门禁全部开启" in dialog
assert "关闭后的实际影响" in dialog
assert "%1/%2 门禁开启" in dialog
assert "finalWeldPoseHardGateGroup" not in dialog
assert "validationPolicyComboBox" not in dialog
assert "validationMinFinitePointCountSpinBox" not in dialog

assert "QJsonObject BuildSafetyGateRecords" in service
for name in safety_names:
    assert f"settings.safetyGate{name}Enabled" in service
assert 'root.insert("safetyGateRecords", BuildSafetyGateRecords(settings));' in service
assert 'thresholds.insert("robotNameBindingEnabled", false);' not in service
assert "settings.safetyGateRobotNameBindingEnabled\n        && proofRobotName.compare" in service
assert "currentSettings.safetyGateRobotNameBindingEnabled" in service
assert "TCP 持久端点" in dialog
assert "实际生效" in dialog
assert "原点云质量证明绑定机器人" in service
assert "当前控制单元为" in service

assert "焊道有效性门限" in legacy_processing_dialog
assert "所有门限都可编辑" in legacy_processing_dialog
assert "finalWeldPoseSourceBindingLimitsLabel" in legacy_processing_dialog
assert "控制器欧拉差仅作跨奇异位形诊断" in legacy_processing_dialog

print("SCAN_SAFETY_GATE_UI_OK")
