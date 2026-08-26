import re
from pathlib import Path


repo = Path(__file__).resolve().parents[2]
main = (repo / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")


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
    "static bool ValidateKinematicsValues",
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
    "TemplateRobotConfig(unit.robotType, unit.unitName)",
    "ConfigLocation::WorkpieceTemplate(",
    "MergeMissingConfigValues(kinematics, factoryDefaults)",
    "ReplaceScopedModuleSectionsAtomically(",
    "ReadScopedModuleSnapshot(",
    "ValidateKinematicsValues(verified, factoryDefaults, error)",
):
    assert token in ensure, f"missing kinematics repair contract: {token}"

write_robot = body(management, "bool WriteRobotPara", "void ReloadControlUnits")
assert write_robot.index("EnsureRobotKinematics(unit, error)") < write_robot.index('ini.SetSectionName("BaseParam")')
assert "!WriteRobotPara(unit, true, error) || !WriteControlInfo(nextUnits, error)" in management
assert "!WriteRobotPara(edited, isNew, error) || !WriteControlInfo(nextUnits, error)" in management

print("CONTROL_UNIT_KINEMATICS_DEFAULTS_OK")
