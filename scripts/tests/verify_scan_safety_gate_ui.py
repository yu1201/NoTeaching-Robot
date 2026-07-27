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
assert 'createManagementAction("扫描安全门禁"' in main
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
    dialog, "void ScanSafetyGateDialog::Save()", "void ScanSafetyGateDialog::UpdatePolicyUi()"
)
assert "确认关闭安全门禁" in dialog
assert "PointCloudProcessingConfig::Load();" in section(
    dialog, "void ScanSafetyGateDialog::Save()", "void ScanSafetyGateDialog::UpdatePolicyUi()"
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
    "void ScanSafetyGateDialog::UpdatePolicyUi()",
)
defaults_body = section(
    dialog,
    "PointCloudProcessingConfig::Settings ValidationDefaults()",
    "ScanSafetyGateDialog::ScanSafetyGateDialog(",
)

for name in quality_names:
    field = f"validation{name}Enabled"
    assert f"bool {field} = true;" in config_header
    assert f"defaults.{field} = true;" in defaults_body
    assert field in reload_body
    assert field in save_body
    assert f'ReadBoolSetting("Validation/{name}Enabled"' in config
    assert f'write("Validation/{name}Enabled"' in config
    assert f"validation{name}EnabledCheckBox" in dialog

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
assert "safetyGateRobotNameBindingEnabled" not in core_helper
for name in safety_names:
    if name != "RobotNameBinding":
        assert f"safetyGate{name}Enabled" in core_helper
assert "CURRENT_SAFETY_GATE_BEHAVIOR_VERSION = 1" in config_header
assert 'ReadIntSetting("SafetyGates/BehaviorVersion", 0)' in config
assert "storedSafetyGateBehaviorVersion < CURRENT_SAFETY_GATE_BEHAVIOR_VERSION" in config
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
assert "系统安全门禁开关只作为审计记录" in dialog
assert "不会改变 Validation 策略" in dialog
assert "系统门禁仅记录 · 流程正常" in dialog
assert "记录说明" in dialog
assert "18" not in dialog or "%1/%2 开启" in dialog

assert "QJsonObject BuildSafetyGateRecords" in service
for name in safety_names:
    assert f"settings.safetyGate{name}Enabled" in service
assert 'root.insert("safetyGateRecords", BuildSafetyGateRecords(settings));' in service
assert 'thresholds.insert("robotNameBindingEnabled", false);' not in service
assert "safetyGateRobotNameBindingEnabled\n        && proofRobotName.compare" not in service
assert "TCP 持久端点" in dialog
assert "固定规则校验" in dialog
assert "原点云质量证明绑定机器人" in service
assert "当前控制单元为" in service

legacy_save = section(
    legacy_processing_dialog,
    "bool LaserWeldFilterDialog::SaveSettings(QString* error) const",
    "void LaserWeldFilterDialog::LoadExternalAlgorithmConfig()",
)
for name in quality_names:
    assert f"processingSettings.validation{name}Enabled = true;" not in legacy_save

print("SCAN_SAFETY_GATE_UI_OK")
