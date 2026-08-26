from pathlib import Path


repo = Path(__file__).resolve().parents[2]
main = (repo / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")
helper = (repo / "src" / "RobotDataHelper.cpp").read_text(encoding="utf-8")
helper_header = (repo / "include" / "RobotDataHelper.h").read_text(encoding="utf-8")
registry_header = (repo / "include" / "RobotDriverRegistry.h").read_text(encoding="utf-8")
fanuc_driver = (repo / "src" / "FANUCRobotDriver.cpp").read_text(encoding="utf-8")
step_driver = (repo / "src" / "StepRobotDriver.cpp").read_text(encoding="utf-8")
inovance_driver = (repo / "src" / "InovanceRobotDriver.cpp").read_text(encoding="utf-8")


def body(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    finish = source.index(end, begin)
    return source[begin:finish]


management = body(
    main,
    "class ControlUnitManagementDialog final",
    "class FtpJobManagementDialog final",
)

for token in (
    'form->addRow("机器人型号", robotModelRow)',
    'basicForm->addRow("机器人型号", robotModelRow)',
    'ReadConfigString(robotIni, "RobotModelId")',
    'WriteConfigString(ini, "RobotModelId", unit.robotModelId)',
    "PopulateRobotModelCombo",
    "RobotModelManagerDialog dialog",
    '<< "类型" << "机器人型号"',
):
    assert token in management, f"missing control-unit robot-model contract: {token}"

assert "QString robotModelId;" in management
assert 'unit.robotModelId = "step.sa10-2000h"' not in management
assert "DefaultFtpCredentialForRobotType" in management
for token in ("defaultFtpHost", "defaultFtpPort", "defaultFtpUser", "defaultFtpPassword"):
    assert token in registry_header, f"missing model-bound FTP default field: {token}"
    assert f"setup->{token}" in management, f"control-unit UI does not consume model FTP default: {token}"
for name, source in (
    ("FANUC", fanuc_driver),
    ("STEP", step_driver),
    ("Inovance", inovance_driver),
):
    for token in ("setup->defaultFtpHost", "setup->defaultFtpPort",
                  "setup->defaultFtpUser", "setup->defaultFtpPassword"):
        assert token in source, f"{name} driver does not consume model FTP default: {token}"

for token in ("int robotType = -1;", "QString robotModelId;"):
    assert token in helper_header, f"missing RobotInfo identity field: {token}"

robot_list = body(
    helper,
    "QVector<RobotDataHelper::RobotInfo> RobotDataHelper::LoadRobotList",
    "RobotDriverAdaptor* RobotDataHelper::GetRobotDriver",
)
for token in (
    "ReadRobotIdentityForModelWelding",
    "unitInfo.sUnitName",
    "info.robotType",
    "info.robotModelId",
):
    assert token in robot_list, f"missing runtime robot-model identity read: {token}"

identity_reader = body(
    helper,
    "void ReadRobotIdentityForModelWelding",
    "QStringList DefaultScanParamLines",
)
assert '"RobotModelId"' in identity_reader
assert '"RobotType"' in identity_reader
for token in (
    "robotType = -1;",
    "AppPaths::IsSafePathComponent(unitName)",
    "ConfigDatabase::ReadScopedModuleSnapshot(",
    'QStringLiteral("robot"), unitName, QStringLiteral("RobotPara")',
    'QStringLiteral("BaseParam")',
    'valueForKey(QStringLiteral("RobotType"))',
    'valueForKey(QStringLiteral("RobotModelId"))',
    ".toInt(&typeOk)",
):
    assert token in identity_reader, f"missing fail-closed configured RobotType read: {token}"

assert "QFileInfo robotParaInfo(path)" not in identity_reader

assert "info.robotType = pDriver" not in robot_list
assert "pDriver->m_nRobotType" not in robot_list
assert "ReadRobotIdentityForModelWelding(\n                DecodeConfigTextForRobotData(unitInfo.sUnitName)" in robot_list

print("CONTROL_UNIT_ROBOT_MODEL_OK")
