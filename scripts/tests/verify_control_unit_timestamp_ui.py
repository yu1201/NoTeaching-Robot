"""Offline source regression for per-control-unit time-axis configuration UI."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ui = (ROOT / "src/QtWidgetsApplication4.cpp").read_text(encoding="utf-8")
header = (ROOT / "include/QtWidgetsApplication4.h").read_text(encoding="utf-8")
registry = (ROOT / "src/RobotDriverRegistry.cpp").read_text(encoding="utf-8")
registry_header = (ROOT / "include/RobotDriverRegistry.h").read_text(encoding="utf-8")

def body(signature: str) -> str:
    start = ui.index("{", ui.index(signature))
    depth, end = 1, start + 1
    while depth:
        depth += (ui[end] == "{") - (ui[end] == "}")
        end += 1
    return ui[start:end]

for removed in ("m_pScanTimestampSourceCombo", "m_pStepSdkInterfaceModeCombo",
                "RefreshScanTimestampSourceUi", "RefreshStepSdkInterfaceModeUi"):
    assert removed not in ui and removed not in header, f"old global home-page control remains: {removed}"
for forbidden in ("LoadScanTimestampSource()", "LoadStepSdkInterfaceMode()",
                  "SaveScanTimestampSource(", "SaveStepSdkInterfaceMode("):
    assert forbidden not in ui, f"home UI must not use mutable global timing configuration: {forbidden}"

for name in ("ControlUnitScanTimestampSource", "ControlUnitStepSdkInterfaceMode"):
    assert f'setObjectName("{name}")' in ui, name
assert 'bool supportsRobotTimestamp = false;' in registry_header
assert 'bool usesStepTimestampInterface = false;' in registry_header
assert '"Job/FANUC", true, false' in registry
assert '"Job/STEP", true, true' in registry

load = body("void LoadRobotPara(UnitConfig& unit) const")
assert "LoadScanTimestampSource(unit.unitName)" in load
assert "LoadStepSdkInterfaceMode(unit.unitName)" in load
fill = body("void FillEditor(const UnitConfig& unit, bool existing)")
assert "findData(unit.stepSdkInterfaceMode)" in fill
assert "findData(unit.scanTimestampSource)" in fill
assert fill.index("findData(unit.scanTimestampSource)") < fill.index("ApplyEditorRobotTypeUi();")

apply = body("void ApplyEditorRobotTypeUi()")
assert "setup->supportsRobotTimestamp" in apply and "setup->usesStepTimestampInterface" in apply
assert 'm_stepSdkInterfaceModeCombo->currentData().toString() != "legacy"' in apply
assert "m_scanTimestampSourceCombo->setEnabled(nativeTimestampAvailable)" in apply
assert 'm_scanTimestampSourceCombo->findData("pc")' in apply
assert "SetFormRowVisible(m_editorForm, m_stepSdkInterfaceModeCombo, usesStepInterface)" in apply

copy = body("void PrepareNewUnit(bool copySelected)")
assert "unit = m_units.at(m_editingRow)" in copy, "copy must preserve the selected unit's timing values"
collect = body("bool CollectEditor(UnitConfig& unit, QString& error) const")
assert "unit.scanTimestampSource = m_scanTimestampSourceCombo->currentData().toString()" in collect
assert "unit.stepSdkInterfaceMode = m_stepSdkInterfaceModeCombo->currentData().toString()" in collect
write = body("bool WriteRobotPara(const UnitConfig& unit, bool isNew, QString& error) const")
assert "RobotConfig(unit.unitName)" in write
assert 'location.module + QStringLiteral("/BaseParam")' in write
assert '"ScanTimestampSource", nativeTimestampAllowed ? unit.scanTimestampSource : QStringLiteral("pc")' in write
assert '"StepSdkInterfaceMode", unit.stepSdkInterfaceMode' in write
assert "ConfigDatabase::WriteScopedSettings" in write

# Snapshot checks alone are insufficient: block new leases before checking
# active operations, and keep the token for the entire database mutation.
block = body("struct ConfigEditBlock")
assert "RobotOperationLease::AddNewOperationsBlock" in block
assert "RobotOperationLease::RemoveNewOperationsBlock(token)" in block
assert "ConfigEditBlock(const ConfigEditBlock&) = delete" in block
save = body("bool SaveCurrent(bool reloadAfterSave)")
for mutation, write_call in [(save, "WriteRobotPara(edited"), (copy, "WriteRobotPara(unit"),
                             (write, "EnsureRobotParameters(")]:
    assert mutation.index("const ConfigEditBlock configEditBlock;") < mutation.index("RobotOperationLease::AnyActive()") < mutation.index(write_call)
assert copy.index("RunNewUnitWizard(unit, copySelected)") < copy.index("const ConfigEditBlock configEditBlock;"), "do not block all robots while the user is still in the wizard"
reload = body("void ReloadControlUnits()")
assert reload.index("const ConfigEditBlock configEditBlock;") < reload.index("RobotOperationLease::AnyActive()") < reload.index("m_reloadCallback();")

# Dashboard is display-only, and its label must describe the actual selected
# driver's available clock (Inovance never advertises native timestamps).
assert "EffectiveScanTimestampSource(" in ui
assert "LoadScanTimestampSource(QString::fromStdString(pRobotDriver->RobotName()))" in ui
assert "pRobotDriver->Supports(RobotDriverCapability::RobotTimestamp)" in ui
print("PASS: per-control-unit timestamp editor, copy/load/save, unsupported-PC normalization and display-only dashboard")
