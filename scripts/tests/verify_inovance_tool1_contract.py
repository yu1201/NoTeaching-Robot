from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


adaptor = (ROOT / "include" / "RobotDriverAdaptor.h").read_text(encoding="utf-8")
header = (ROOT / "include" / "InovanceRobotDriver.h").read_text(encoding="utf-8")
driver = (ROOT / "src" / "InovanceRobotDriver.cpp").read_text(encoding="utf-8")
ui = (ROOT / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")
step = (ROOT / "src" / "StepRobotDriver.cpp").read_text(encoding="utf-8")

require(
    "inline constexpr int kApplicationGunToolNumber = 1;" in adaptor,
    "brand-neutral Tool1 gun contract is missing",
)
require(
    "int m_toolNo = kApplicationGunToolNumber;" in header,
    "Inovance driver member must default to Tool1",
)
require(
    "int m_wobjNo = 1;" in header,
    "Inovance driver member must default to Wobj1",
)
require(
    'baseParam.insert("ToolNo", QString::number(kApplicationGunToolNumber));' in ui,
    "new Inovance brand templates must default to Tool1",
)
require(
    'baseParam.insert("WobjNo", QStringLiteral("1"));' in ui,
    "new Inovance brand templates must default to Wobj1",
)
require(
    "robotType == ROBOT_TYPE_INOVANCE" in ui
    and 'valueIt.value().trimmed() == QStringLiteral("0")' in ui
    and "legacyTool || legacyWobj" in ui
    and 'valueIt.value() = QStringLiteral("1");' in ui,
    "legacy Inovance brand-template Tool0/Wobj0 migration is missing",
)
require(
    "if (m_toolNo == 0)" in driver
    and 'ini.WriteString("ToolNo", m_toolNo)' in driver,
    "legacy live-unit Tool0 migration/readback persistence is missing",
)
require(
    "if (m_wobjNo == 0)" in driver
    and 'ini.WriteString("WobjNo", m_wobjNo)' in driver,
    "legacy live-unit Wobj0 migration/readback persistence is missing",
)
require(
    "m_toolNo != kApplicationGunToolNumber" in driver
    and "m_wobjNo != 1" in driver
    and "已标定Tool1和Wobj1" in driver,
    "Inovance connection/JOB path must fail closed outside Tool1/Wobj1",
)
require(
    "tool1,WORLD" in step,
    "STEP native program generation no longer proves the shared Tool1 contract",
)

print("PASS: Inovance Tool1/Wobj1 contract, legacy zero migration, and fail-closed enforcement")
