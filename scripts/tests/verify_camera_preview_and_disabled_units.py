from pathlib import Path


repo = Path(__file__).resolve().parents[2]
source = (repo / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")


def body(text: str, start: str, end: str) -> str:
    begin = text.index(start)
    finish = text.index(end, begin)
    return text[begin:finish]


management = body(
    source,
    "class ControlUnitManagementDialog final",
    "class FtpJobManagementDialog final",
)
read_units = body(management, "QList<UnitConfig> ReadUnits", "void LoadRobotPara")
for token in (
    "ConfigDatabase::TryListScopedSettingIds(",
    'QStringLiteral("robot")',
    'QStringLiteral("RobotPara/BaseParam")',
    "LoadRobotPara(unit);",
    "unit.enabled = false;",
    "unitRowByName.contains(lookupName)",
):
    assert token in read_units, f"disabled control-unit recovery missing: {token}"
assert read_units.index("LoadRobotPara(unit);") < read_units.index("unit.enabled = false;")

load_robot = body(management, "void LoadRobotPara", "void LoadUnits")
assert 'ReadConfigString(robotIni, "ChineseName")' in load_robot
assert "unit.customName.trimmed().isEmpty()" in load_robot

write_robot = body(management, "bool WriteRobotPara", "void ReloadControlUnits")
assert 'WriteConfigString(ini, "ChineseName", unit.chineseName)' in write_robot

preview = body(
    source,
    "void QtWidgetsApplication4::GrooveCameraTest(bool checked)",
    "void QtWidgetsApplication4::UpdateGrooveCameraData()",
)
assert preview.index("OpenGroovePointCloudDialog();") < preview.index(
    "IsCurrentRobotSetupReady(true, false, &setupIssue)"
)
for token in (
    'QStringLiteral("相机未连接")',
    "LoadGrooveCameraEndpointForUnit(unitIndex, cameraIP, configuredCameraPort)",
    "EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true, false)",
    "QTimer::singleShot(0, previewDialog",
):
    assert token in preview, f"non-blocking camera preview diagnostic missing: {token}"
assert 'QMessageBox::warning(this, "坡口相机测试"' not in preview
assert 'ui.GrooveCameraTestBtn->setChecked(false)' not in preview

print("CAMERA_PREVIEW_AND_DISABLED_UNITS_OK")
