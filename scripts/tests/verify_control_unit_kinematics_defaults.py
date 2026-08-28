import re
from pathlib import Path


repo = Path(__file__).resolve().parents[2]
main = (repo / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")
registry = (repo / "src" / "RobotDriverRegistry.cpp").read_text(encoding="utf-8")
registry_header = (repo / "include" / "RobotDriverRegistry.h").read_text(encoding="utf-8")
config_section = (repo / "src" / "ConfigSection.cpp").read_text(encoding="utf-8")


def body(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish]


management = body(
    main,
    "class ControlUnitManagementDialog final",
    "class FtpJobManagementDialog final",
)
defaults = body(
    management,
    "static QMap<QString, QString> DefaultKinematicsForRobotType",
    "static bool TryGetMapValueCaseInsensitive",
)
step_defaults = body(defaults, "if (robotType == ROBOT_TYPE_STEP)", "if (robotType == ROBOT_TYPE_FANUC)")
fanuc_defaults = body(defaults, "if (robotType == ROBOT_TYPE_FANUC)", "return {};")

expected_keys = {
    *(f"d{field}{axis}" for axis in range(1, 7) for field in ("A", "AL", "D", "TH")),
    *(f"d{axis}{suffix}" for axis in "SLURBT" for suffix in ("Angle", "Pulse")),
    *(f"d{side}{axis}Angle" for axis in "SLURBT" for side in ("Max", "Min")),
}


def literal_values(block: str) -> dict[str, str]:
    return dict(re.findall(r'\{\s*"([^"]+)",\s*"([^"]*)"\s*\}', block))


step_values = literal_values(step_defaults)
fanuc_values = literal_values(fanuc_defaults)
assert set(step_values) == expected_keys, "STEP factory kinematics template is incomplete"
assert set(fanuc_values) == expected_keys, "FANUC factory kinematics template is incomplete"
for axis in "SLURBT":
    assert step_values[f"d{axis}Angle"] == "90.0"
    assert step_values[f"d{axis}Pulse"] == "180000"
    assert fanuc_values[f"d{axis}Angle"] == "90.0"
    assert fanuc_values[f"d{axis}Pulse"] == "90000"

merge = body(
    management,
    "static void MergeMissingConfigValues",
    "bool EnsureRobotTypeTemplate",
)
assert "!TryGetMapValueCaseInsensitive(target, sourceIt.key(), ignored)" in merge
assert "target.insert(sourceIt.key(), sourceIt.value())" in merge

validation = body(
    management,
    "static bool ValidateKinematicsValues",
    "void PopulateRobotModelCombo",
)
for token in (
    "std::isfinite(number)",
    "std::isfinite(angle)",
    "std::isfinite(pulse)",
    "std::abs(angle) < 1e-15",
    "std::abs(pulse) < 1e-15",
):
    assert token in validation, f"missing kinematics validation: {token}"

ensure = body(
    management,
    "bool EnsureRobotKinematics",
    "bool EnsureWorkpieceTemplateModules",
)
for token in (
    "RobotTypeTemplateConfig(unit.robotType)",
    "templateKinematics",
    "MergeMissingConfigValues(kinematics, factoryDefaults)",
    "ReplaceScopedModuleSectionsAtomically(",
    "ReadScopedModuleSnapshot(",
    "ValidateKinematicsValues(verified, requiredShape, error)",
):
    assert token in ensure, f"missing kinematics repair contract: {token}"
assert "TemplateRobotConfig" not in management
assert "ConfigLocation::WorkpieceTemplate(" not in ensure
assert "templateUnitName" not in registry_header
assert 'QStringLiteral("robot_type_template")' in config_section
for template_id in ('"fanuc", 9000', '"step", 30312', '"inovance", 2222'):
    assert template_id in registry, f"missing independent type template: {template_id}"

template_snapshot = body(
    management,
    "static QMap<QString, QMap<QString, QString>> DefaultRobotTypeTemplateSnapshot",
    "static bool TryGetMapValueCaseInsensitive",
)
for token in (
    'snapshot["TemplateMeta"]',
    '{ "TemplateId",',
    '{ "RobotType",',
    '{ "TemplateRevision", QStringLiteral("1") }',
):
    assert token in template_snapshot, f"missing type-template identity data: {token}"

template_gate = body(
    management,
    "bool EnsureRobotTypeTemplate",
    "static bool ValidateKinematicsValues",
)
for token in (
    'TryGetMapValueCaseInsensitive(meta, "TemplateId"',
    'TryGetMapValueCaseInsensitive(meta, "RobotType"',
    "ReplaceScopedModuleSectionsAtomically(",
    "禁止把实际控制单元或其他品牌当作模板",
):
    assert token in template_gate, f"missing type-template identity gate: {token}"

parameter_gate = body(
    management,
    "bool EnsureRobotParameters",
    "bool EnsureRobotKinematics",
)
for token in (
    "RobotTypeTemplateConfig(unit.robotType)",
    "ReadScopedSettingStatus(",
    "storedType != unit.robotType",
    "CopyScopedModule(",
    "true",
):
    assert token in parameter_gate, f"missing brand-switch template isolation: {token}"

write_robot = body(management, "bool WriteRobotPara", "void ReloadControlUnits")
assert write_robot.index("EnsureRobotKinematics(unit, error)") < write_robot.index('ini.SetSectionName("BaseParam")')
for token in (
    "!robotTypeChanged && unit.enabled",
    "!robotTypeChanged && unit.cameraParamReady",
    "!robotTypeChanged && unit.handEyeReady",
):
    assert token in write_robot, f"brand switch does not fail closed: {token}"
assert "!WriteRobotPara(unit, true, error) || !WriteControlInfo(nextUnits, error)" in management
assert "!WriteRobotPara(edited, isNew, error) || !WriteControlInfo(nextUnits, error)" in management

print("CONTROL_UNIT_KINEMATICS_DEFAULTS_OK")
