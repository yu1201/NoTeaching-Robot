#!/usr/bin/env python3
"""Fail when business code bypasses RobotDriverAdaptor's robot contract."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
ALLOWED = {
    Path("src/RobotDriverAdaptor.cpp"),
    Path("src/FANUCRobotDriver.cpp"),
    Path("src/StepRobotDriver.cpp"),
    Path("src/InovanceRobotDriver.cpp"),
    Path("src/RobotDriverRegistry.cpp"),
    Path("src/RobotFtpFileTransfer.cpp"),
    Path("src/FTPClient.cpp"),
    # These FTP users target the online archive/model service, not a robot controller.
    Path("src/OnlineServicesDialog.cpp"),
    Path("src/OnlineServicesLoginDialog.cpp"),
    Path("src/ScanDataUploader.cpp"),
    Path("src/RobotModelManagerDialog.cpp"),
    Path("include/FANUCRobotDriver.h"),
    Path("include/STEPRobotDriver.h"),
    Path("include/InovanceRobotDriver.h"),
    Path("include/FTPClient.h"),
    Path("include/RobotFtpFileTransfer.h"),
    Path("include/OnlineServicesConfig.h"),
    Path("include/ScanDataUploader.h"),
}
FORBIDDEN = {
    "concrete FANUC driver type": re.compile(r"\bFANUCRobotCtrl\b"),
    "concrete STEP driver type": re.compile(r"\bSTEPRobotCtrl\b"),
    "concrete Inovance driver type": re.compile(r"\bInovanceRobotCtrl\b"),
    "FANUC driver header": re.compile(r"FANUCRobotDriver\.h"),
    "STEP driver header": re.compile(r"STEPRobotDriver\.h"),
    "Inovance driver header": re.compile(r"InovanceRobotDriver\.h"),
    "raw STEP SDK status": re.compile(r"STEPROBOTSDK::"),
    "raw FANUC motion register": re.compile(r"FANUC_MOTION_STATE_REG"),
    "direct FTP transport": re.compile(r"\b(?:FtpClient|FtpRemoteFileInfo)\b"),
    "direct configured socket call": re.compile(r"\b(?:InitSocket|CloseSocket)\s*\("),
    "direct robot transport field": re.compile(
        r"->m_(?:sSocketIP|nSocketPort|sFTPIP|nFTPPort|sFTPUser|sFTPPassWord|pFTP)\b"
    ),
    "direct robot adaptor state": re.compile(
        r"->m_(?:tKinematics|tAxisUnit|tAxisLimitAngle|tTools|tFirstTool|tHomePulse|"
        r"nRobotNo|sRobotName|sCustomName|nRobotType|nExternalAxleType|nRobotAxisCount|"
        r"eRobotBrand|pRobotLog)\b"
    ),
    "legacy business MoveByJob": re.compile(r"\bMoveByJob\s*\("),
    "legacy business ContiMoveAny": re.compile(r"\bContiMoveAny\s*\("),
    "legacy business CallJob": re.compile(r"\bCallJob\s*\("),
    "ambiguous business GetCurrentPos value fallback": re.compile(r"\bGetCurrentPos\s*\(\s*\)"),
    "ambiguous business GetCurrentPulse value fallback": re.compile(r"\bGetCurrentPulse\s*\(\s*\)"),
}


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


violations: list[str] = []
for folder in (ROOT / "src", ROOT / "include"):
    for path in sorted(folder.glob("**/*")):
        if path.suffix.lower() not in {".cpp", ".h", ".hpp"}:
            continue
        relative = path.relative_to(ROOT)
        if relative in ALLOWED or relative == Path("include/RobotDriverAdaptor.h"):
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for label, pattern in FORBIDDEN.items():
            for match in pattern.finditer(text):
                line = text.count("\n", 0, match.start()) + 1
                violations.append(f"{relative}:{line}: {label}")

if violations:
    fail("business layer bypasses RobotDriverAdaptor:\n  " + "\n  ".join(violations))

adaptor = (ROOT / "include/RobotDriverAdaptor.h").read_text(encoding="utf-8")
required_contract = (
    "DriverDescriptor() const = 0",
    "DriverCapabilities() const = 0",
    "ControlEndpoint() const = 0",
    "Connect() = 0",
    "Disconnect() = 0",
    "CreateFileTransferSession(",
    "RobotName() const noexcept",
    "ExternalAxleType() const noexcept",
    "KinematicsParameters() const noexcept",
    "AxisUnit() const noexcept",
    "Tools() const noexcept",
    "SetConfiguredGunTool(",
    "WriteLog(LogColor color, const char* format, ...) const",
    "MoveLinearMmPerMin(",
    "MoveJointPercent(",
    "ReadMotionStatus() = 0",
    "ReserveTrajectory(",
    "DownlinkTrajectory(",
    "StartTrajectory(",
    "WaitTrajectory(",
    "AbortCurrentProgramSafely() = 0",
    "TryGetCurrentPos(T_ROBOT_COORS& pos) = 0",
    "TryGetCurrentPulse(T_ANGLE_PULSE& pulse) = 0",
    "CheckRobotDone(int nDelayTime = 200, int runTimeoutMs = 1800000) = 0",
    "GetToolData(int nToolNo, T_ROBOT_COORS& robotToolData) = 0",
    "TryGetIntVar(int nIndex, int& value, const char* cStrPreFix = \"INT\") = 0",
    "GetHandEyeMatrixVariable(const char* variableName, double rotation[9], double translation[3], std::string* error = nullptr) = 0",
)
for token in required_contract:
    if token not in adaptor:
        fail(f"mandatory adaptor contract token is missing: {token}")

for implementation in (
    "FANUCRobotDriver.cpp", "StepRobotDriver.cpp", "InovanceRobotDriver.cpp"
):
    text = (ROOT / "src" / implementation).read_text(encoding="utf-8")
    for token in (
        "DriverCapabilities() const",
        "ControlEndpoint() const",
        "CreateFileTransferSession(",
        "MoveLinearMmPerMin(",
        "ReadMotionStatus()",
        "StartTrajectory(",
        "WaitTrajectory(",
        "AbortCurrentProgramSafely()",
    ):
        if token not in text:
            fail(f"{implementation} does not implement required contract token: {token}")

if '#include "FTPClient.h"' in adaptor or "ftpPassword" in adaptor:
    fail("RobotDriverAdaptor must declare FTP functionality without owning the FTP transport or credentials")

for raw_hook in (
    "InitSocket", "CloseSocket", "ContiMoveAny", "CallJob",
    "InitFtp", "UploadFile", "DownloadFile", "MoveByJob",
):
    if raw_hook in adaptor:
        fail(f"RobotDriverAdaptor still declares brand-bottom hook: {raw_hook}")

state_marker = "//----------------------------------------变量类--------------------------------------------//"
if state_marker not in adaptor or "protected:" not in adaptor.split(state_marker, 1)[1].split("private:", 1)[0]:
    fail("RobotDriverAdaptor runtime state must remain protected behind functional accessors")

registry = (ROOT / "src/RobotDriverRegistry.cpp").read_text(encoding="utf-8")
for token in (
    "ROBOT_TYPE_FANUC",
    "ROBOT_TYPE_STEP",
    "CreateFanucDriver",
    "CreateStepDriver",
    "ROBOT_TYPE_INOVANCE",
    "CreateInovanceDriver",
    "RobotDriverRegistry::SetupProfile",
):
    if token not in registry:
        fail(f"robot driver registry is missing: {token}")

for critical_file in (
    "MeasureThenWeldService.cpp",
    "RobotJogDialog.cpp",
    "HandEyeCalibrationDialog.cpp",
    "FunctionTestDialog.cpp",
    "QtWidgetsApplication4.cpp",
):
    text = (ROOT / "src" / critical_file).read_text(encoding="utf-8")
    if "SupportsAll(" not in text:
        fail(f"critical robot entry file has no multi-capability gate: {critical_file}")

print("PASS: business robot operations are routed through RobotDriverAdaptor")
