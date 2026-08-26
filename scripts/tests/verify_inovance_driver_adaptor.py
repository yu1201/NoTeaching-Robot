#!/usr/bin/env python3
"""Static safety/contract gate for the Inovance RobotDriverAdaptor driver."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def main() -> None:
    header = read("include/InovanceRobotDriver.h")
    driver = read("src/InovanceRobotDriver.cpp")
    registry = read("src/RobotDriverRegistry.cpp")
    contract = read("docs/robot-driver-adaptor-contract.md")
    ui = read("src/QtWidgetsApplication4.cpp")

    require(
        "class InovanceRobotCtrl final : public RobotDriverAdaptor" in header,
        "Inovance must be a complete RobotDriverAdaptor implementation",
    )
    require("ROBOT_TYPE_INOVANCE" in registry, "Inovance type is not registered")
    require("CreateInovanceDriver" in registry, "Inovance factory is missing")
    require(
        '{ "RobotC", 2222, 0, false, 7777, "192.168.23.25", "robot",' in registry,
        "Inovance model defaults must bind the 2222 control and configured FTP endpoints",
    )

    protocol = function_body(driver, "bool InovanceRobotCtrl::SendCommand(")
    for token in (
        '"@@" + command + "$$"',
        'framed.find("$$")',
        'framed.find("##")',
        "kMaxProtocolResponse",
        "m_socketMutex",
        "CloseSocketLocked()",
        "ProtocolErrorText(response)",
    ):
        require(token in protocol, f"Inovance framed protocol evidence missing: {token}")

    capabilities = function_body(
        driver, "std::uint64_t InovanceRobotCtrl::DriverCapabilities() const"
    )
    for capability in (
        "PassiveState",
        "LinearMotion",
        "JointMotion",
        "ContinuousTrajectory",
        "ContinuousJog",
        "PauseResume",
        "OperationModeControl",
        "NativeProgramUpload",
        "DiagnosticCommand",
        "CartesianRegister",
        "VerifiedProgramCompletion",
        "VerifiedSafeAbort",
        "ExternalAxis",
        "ConnectionControl",
        "AlarmReset",
        "ServoPowerControl",
        "ToolDataRead",
        "TeachPendantSpeedControl",
        "FtpFileTransfer",
    ):
        require(
            f"RobotDriverCapability::{capability}" in capabilities,
            f"Inovance implemented capability missing: {capability}",
        )
    for unsupported in (
        "RobotTimestamp",
        "PersistentProgramRecovery",
        "NativeProgramExecution",
        "OfflineTrajectoryExport",
        "ActualArcWeld",
        "IntegerRegister",
        "HandEyeMatrixRead",
        "HandEyeSupportProgramInstall",
    ):
        require(
            f"RobotDriverCapability::{unsupported}" not in capabilities,
            f"Inovance must not claim unverified capability: {unsupported}",
        )
    require("jointUnitsReady" in capabilities, "JointMotion must be gated by configured AxisUnit")
    require(
        "m_nExternalAxleType != 0" in capabilities,
        "ExternalAxis must only be advertised for a configured external axis",
    )

    initialize = function_body(
        driver, "bool InovanceRobotCtrl::InitializeAfterConnect("
    )
    permit = function_body(driver, "bool InovanceRobotCtrl::EnsureControlPermit()")
    ready = function_body(driver, "bool InovanceRobotCtrl::EnsureMotionReady()")
    for token in ("CurCtrlDev", "CurPermit", "AcqPermit", "ForceControlPermit"):
        require(token in permit or token in driver, f"permit safety missing: {token}")
    for token in ("Get_EStopSts", "Get_SysErrSts", "Get_MotorSts"):
        require(token in ready, f"pre-motion gate missing: {token}")
    for token in ("Get_ToolCNum", "Get_WobjNum", "UserLogin", "CurUserType"):
        require(token in initialize, f"initialization identity check missing: {token}")

    cart = function_body(driver, "bool InovanceRobotCtrl::SendCartesianMove(")
    for token in (
        "target.dRZ",
        "target.dRY",
        "target.dRX",
        '"MovLRobP "',
        '"Get_CurCmdNum"',
        "m_externalValues",
        "m_armConfig",
    ):
        require(token in cart, f"Inovance Cartesian mapping missing: {token}")
    require("size() > 128" in cart, "MovLRobP must enforce the manual parameter limit")
    joint = function_body(driver, "bool InovanceRobotCtrl::SendJointMove(")
    for token in ("MovJAbsRobJP", "dSPulseUnit", "Get_CurCmdNum"):
        require(token in joint, f"Inovance joint conversion missing: {token}")
    require("size() > 128" in joint, "MovJAbsRobJP must enforce the manual parameter limit")

    start = function_body(driver, "bool InovanceRobotCtrl::StartTrajectory(")
    wait = function_body(driver, "bool InovanceRobotCtrl::WaitTrajectory(")
    abort = function_body(driver, "bool InovanceRobotCtrl::AbortCurrentProgramSafely()")
    pause = function_body(driver, "bool InovanceRobotCtrl::PauseTrackedMotion(")
    resume = function_body(driver, "bool InovanceRobotCtrl::ResumeTrackedMotion(")
    for token in (
        "FingerprintMoveInfos",
        "Get_CurCmdCacheNum",
        "m_maxBufferedCommands",
        "m_finalCommandId",
    ):
        require(token in start, f"trajectory identity/backpressure missing: {token}")
    for token in ("WaitForCommandDone", 'SetDataStreamMode("OFF", 0)'):
        require(token in wait, f"trajectory completion missing: {token}")
    for token in ("Get_CmdSts", "Get_MotionSts", "stableDone >= 2"):
        require(token in driver, f"exact completion witness missing: {token}")
    for token in ("Dsmode", "Get_DsMode", "stableStopped >= 3"):
        require(token in abort, f"verified safe abort missing: {token}")
    for token in ("PAUSE", "Get_CurCmdNum", "positionDeviation", "angleDeviation"):
        require(token in pause, f"verified pause missing: {token}")
    for token in ("CONTINUE", "expectedProgramName", "checkpointPose"):
        require(token in resume, f"verified resume missing: {token}")

    shutdown = function_body(
        driver, "bool InovanceRobotCtrl::ShutdownBeforeDisconnect()"
    )
    for token in ("AbortCurrentProgramSafely", "Motor OFF", "RemovePermit"):
        require(token in shutdown, f"safe disconnect sequence missing: {token}")

    transfer = function_body(
        driver, "std::shared_ptr<RobotFileTransferSession> InovanceRobotCtrl::CreateFileTransferSession("
    )
    for token in ("RobotFtpFileTransfer", "m_ftpIp", "m_ftpPort", "FileTransferProfile()"):
        require(token in transfer, f"Inovance FTP session wiring missing: {token}")
    profile = function_body(driver, "RobotFileTransferProfile InovanceRobotCtrl::FileTransferProfile() const")
    for token in ('"/TeachProgram"', '"*.pro"', '"*.prj"', '"*.pts"', '"*.jsn"'):
        require(token in profile, f"Inovance FTP profile metadata missing: {token}")

    prepare_upload = function_body(driver, "bool InovanceRobotCtrl::PrepareNativeProgramUpload()")
    require("CreateFileTransferSession" in prepare_upload and "return true;" in prepare_upload,
            "Inovance native upload preparation must validate the FTP bottom")
    upload = function_body(driver, "int InovanceRobotCtrl::UploadNativeProgramSource(")
    for token in (
        "is_regular_file",
        'Get_TaskRunSts 0',
        'Get_TaskPrgPath 0',
        "InovanceActiveProjectDirectory",
        "UploadProgramFile",
        "return 0;",
    ):
        require(token in upload, f"Inovance native program upload gate missing: {token}")
    require("taskStatus == 1" in upload, "Inovance upload must refuse replacing a running task")

    for signature, explanation in (
        ("bool InovanceRobotCtrl::RunProgramAndWait(", "按名称原生程序执行保持关闭"),
        ("bool InovanceRobotCtrl::InstallHandEyeSupportPrograms(", "无法安装机器人侧手眼辅助程序"),
        ("bool InovanceRobotCtrl::GetHandEyeMatrixVariable(", "未定义可读取的3x3旋转"),
    ):
        body = function_body(driver, signature)
        require("return false;" in body, f"unsupported feature must fail closed: {signature}")
        require(explanation in body, f"unsupported feature lacks actionable reason: {signature}")

    require("defaultFtpPort > 0" in ui, "configuration UI does not hide unsupported FTP")
    require(
        "汇川已登记2222远程以太网与FTP/原生程序上传底层" in ui,
        "configuration UI lacks Inovance capability warning",
    )
    require(
        "PC接收steady时间（该品牌未提供控制器时间戳）" in read("src/FunctionTestDialog.cpp"),
        "timestamp diagnostic does not disclose the Inovance PC-time fallback",
    )
    for token in (
        "`ROBOT_TYPE_INOVANCE`",
        "`Get_CmdSts(id)=1`",
        "`PersistentProgramRecovery`",
        "`ActualArcWeld`",
        "PC steady",
        "首次现场使用前",
    ):
        require(token in contract, f"Inovance capability boundary is undocumented: {token}")

    print(
        "PASS: Inovance adaptor uses verified 2222 framing, model-bound FTP upload, PC-time fallback, and explicit capability limits"
    )


if __name__ == "__main__":
    main()
