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
    state = "code"
    escaped = False
    for index in range(brace, len(source)):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""
        if state == "line_comment":
            if char == "\n":
                state = "code"
            continue
        if state == "block_comment":
            if char == "*" and next_char == "/":
                state = "block_comment_end"
            continue
        if state == "block_comment_end":
            state = "code"
            continue
        if state in ("string", "char"):
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif (state == "string" and char == '"') or (state == "char" and char == "'"):
                state = "code"
            continue
        if char == "/" and next_char == "/":
            state = "line_comment"
            continue
        if char == "/" and next_char == "*":
            state = "block_comment"
            continue
        if char == '"':
            state = "string"
            continue
        if char == "'":
            state = "char"
            continue
        if char == "{":
            depth += 1
        elif char == "}":
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
        '{ "inovance", 2222, 0, false, 7777, "192.168.23.25", "robot",' in registry,
        "Inovance model defaults must bind the 2222 control and configured FTP endpoints",
    )
    require('"RobotC", 2222' not in registry,
            "Inovance type template must not be bound to a real RobotC control unit")
    require('QStringLiteral("robot_type_template")' in read("src/ConfigSection.cpp"),
            "independent robot-type database scope is missing")
    for token in ("RobotTypeTemplateConfig(robotType)", 'snapshot["TemplateMeta"]'):
        require(token in ui, f"Inovance independent type-template wiring missing: {token}")

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
        "OperationModeControl",
        "NativeProgramUpload",
        "NativeProgramExecution",
        "DiagnosticCommand",
        "CartesianRegister",
        "IntegerRegister",
        "VerifiedProgramCompletion",
        "VerifiedSafeAbort",
        "ExternalAxis",
        "ConnectionControl",
        "AlarmReset",
        "ServoPowerControl",
        "ToolDataRead",
        "TeachPendantSpeedControl",
        "FtpFileTransfer",
        "OfflineTrajectoryExport",
    ):
        require(
            f"RobotDriverCapability::{capability}" in capabilities,
            f"Inovance implemented capability missing: {capability}",
        )
    for unsupported in (
        "RobotTimestamp",
        "PersistentProgramRecovery",
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
    require(
        "HasVerifiedWeldJobContract(nullptr)" in capabilities
        and "RobotDriverCapability::ActualArcWeld" in capabilities,
        "ActualArcWeld must be gated by the complete controller-side weld JOB contract",
    )
    require(
        "RobotDriverCapability::PauseResume" not in capabilities,
        "Inovance must not advertise PauseResume for native controller JOBs",
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
    for token in (
        "SetDataStreamMode",
        "Get_DsMode",
        "Prg Stop",
        "BackStartLine",
        "Get_TaskRunSts 0",
        "stableStopped >= 3",
    ):
        require(token in abort, f"verified safe abort missing: {token}")
    for token in (
        "IsInovanceNativeTrajectoryPurpose",
        "没有原生程序暂停/续行命令",
        "PAUSE",
        "Get_CurCmdNum",
        "positionDeviation",
        "angleDeviation",
    ):
        require(token in pause, f"verified pause missing: {token}")
    for token in (
        "IsInovanceNativeTrajectoryPurpose",
        "不支持通过2222协议从中断行续行",
        "CONTINUE",
        "expectedProgramName",
        "checkpointPose",
    ):
        require(token in resume, f"verified resume missing: {token}")

    validate_job = function_body(driver, "bool InovanceRobotCtrl::ValidateMoveInfos(")
    for token in (
        "HasVerifiedWeldJobContract",
        "bArcStartBeforeMove",
        "bArcEndAfterMove",
        "bUseTransitionWeldParams",
        "bHasTrackParam",
        "bHasWeaveParam",
        "bAppPointwiseWeave",
        "mappedDaInRange",
    ):
        require(token in validate_job, f"Inovance weld JOB validation missing: {token}")

    generate_job = function_body(driver, "bool InovanceRobotCtrl::WriteTrajectoryJobFile(")
    for token in (
        "QTWIDGETSAPP4_INOVANCE_TRAJECTORY_JOB_V1",
        '"Func Run()\\r\\n"',
        '"LP["',
        '"JP["',
        '"Movl LP["',
        '"MovAbsJ JP["',
        '"Set Out["',
        '"Set DA["',
        '"Wait In["',
        '"IConnect "',
        '"ISigIn(ONCE,"',
        '"IActive "',
        '"IEnable;\\r\\n"',
        '"Trap ArcLostTrap()\\r\\n"',
        "Alarm[",
        '"EndFunc;\\r\\n"',
        "kInovanceProgramInstructionLimit",
        "programContentSha256",
        "programContentSize",
    ):
        require(token in generate_job, f"Inovance controller JOB generator missing: {token}")
    require(
        "Until In[" not in generate_job,
        "arc-loss monitoring must not force every weld move to Z[0]",
    )
    fingerprint = function_body(driver, "std::uint64_t InovanceRobotCtrl::FingerprintMoveInfos(")
    for token in ("RobotTrajectoryPurpose::ActualWeld", "m_weldArcInterruptId", "weldValues"):
        require(token in fingerprint, f"frozen weld JOB identity missing: {token}")

    upload_job = function_body(driver, "bool InovanceRobotCtrl::UploadTrajectoryJob(")
    for token in (
        "Get_TaskRunSts 0",
        "InovanceActiveMainProgram",
        "kInovanceProgramFileLimit",
        "UploadProgramFile",
        "DownloadProgramFile",
        "InovanceContentSha256",
    ):
        require(token in upload_job, f"Inovance generated JOB upload gate missing: {token}")
    verify_job = function_body(
        driver, "bool InovanceRobotCtrl::VerifyTrajectoryJobRemoteIdentity("
    )
    for token in ("DownloadProgramFile", "programContentSize", "programContentSha256"):
        require(token in verify_job, f"Inovance pre-start JOB identity check missing: {token}")

    weld_contract = function_body(
        driver, "bool InovanceRobotCtrl::HasVerifiedWeldJobContract("
    )
    for token in (
        "m_weldJobEnabled",
        "m_weldArcEnableDo",
        "m_weldReadyDi",
        "m_weldArcEstablishedDi",
        "m_weldCurrentDa",
        "m_weldVoltageDa",
        "m_weldReadyTimeoutMs",
        "m_weldArcStartTimeoutMs",
        "m_weldArcEndTimeoutMs",
        "m_weldAlarmIndex",
        "m_weldArcInterruptId",
    ):
        require(token in weld_contract, f"Inovance WeldJob DB contract missing: {token}")
    prepare_welder = function_body(driver, "bool InovanceRobotCtrl::PrepareWeldJobHardware(")
    for token in ("Get_DOCfg", "Get_DACfg", "ConfirmWeldArcOutputOff"):
        require(token in prepare_welder, f"Inovance weld hardware preflight missing: {token}")
    confirm_arc_off = function_body(driver, "bool InovanceRobotCtrl::ConfirmWeldArcOutputOff(")
    for token in ("Set_DO", "Get_DO", "stableOff >= 3"):
        require(token in confirm_arc_off, f"Inovance verified arc-off missing: {token}")
    reload_config = function_body(driver, "void InovanceRobotCtrl::ReloadRuntimeConfiguration()")
    for token in ("m_trajectoryRunning", "m_nativeProgramRunning", "禁止重载机器人配置"):
        require(token in reload_config, f"Inovance live JOB config freeze missing: {token}")

    downlink = function_body(driver, "bool InovanceRobotCtrl::DownlinkTrajectory(")
    for token in (
        "IsInovanceNativeTrajectoryPurpose",
        "WriteTrajectoryJobFile",
        "UploadTrajectoryJob",
        "FingerprintMoveInfos",
    ):
        require(token in downlink, f"Inovance native JOB downlink missing: {token}")
    export_job = function_body(driver, "bool InovanceRobotCtrl::ExportTrajectoryProgramFiles(")
    require("WriteTrajectoryJobFile" in export_job, "offline export must use the JOB generator")
    for token in (
        "VerifyTrajectoryJobRemoteIdentity",
        "PrepareWeldJobHardware",
        "std::async",
        "RunProgramAndWait",
        "m_nativeProgramRunning",
    ):
        require(token in start, f"Inovance asynchronous native JOB start missing: {token}")
    for token in (
        "m_nativeTrajectoryFuture",
        "RequestCancellation",
        "AbortCurrentProgramSafely",
        "ConfirmWeldArcOutputOff",
    ):
        require(token in wait, f"Inovance native JOB wait/abort missing: {token}")

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

    execute = function_body(driver, "bool InovanceRobotCtrl::RunProgramAndWait(")
    for token in (
        "ParseInovanceProgramRequest",
        "Get_TaskRunSts 0",
        "Get_TaskPrgPath 0",
        "InovanceActiveMainProgram",
        "kInovanceProgramFileLimit",
        "DownloadProgramFile",
        "ValidateInovanceCallableModule",
        "WriteInovanceDispatcher",
        "UploadProgramFile",
        "main_uploaded_verify.pro",
        "main_restored_verify.pro",
        "backupMainContent",
        "restoreOriginalMain",
        "mainRestoreVerified",
        "BackStartLine",
        "Prg Start",
        "Get_TaskProgramLine 0",
        "Get_SysErrSts",
        "kInovanceNativeProgramStateByte",
        "MarkMotionStarted",
        "MarkMotionCompleted",
        "StopAndConfirmUnverifiedMotion",
        "stableCompleted >= 3",
    ):
        require(token in execute, f"Inovance native execution gate missing: {token}")
    for token in (
        "QTWIDGETSAPP4_INOVANCE_DISPATCHER_V1",
        '<< "Include \\\""',
        '<< ".Run();',
        "B[",
        "= 10",
        "Func",
        "Run",
    ):
        require(token in driver, f"Inovance dispatcher/module contract missing: {token}")

    get_int = function_body(driver, "bool InovanceRobotCtrl::TryGetIntVar(")
    set_int = function_body(driver, "bool InovanceRobotCtrl::SetIntVar(\n    int index")
    for token in ("Get_R ", "Get_B ", "Get_PlcVar DInt"):
        require(token in get_int, f"Inovance integer read mapping missing: {token}")
    for token in ("Set_R ", "Set_B ", "TryGetIntVar", "verified != value"):
        require(token in set_int, f"Inovance integer write/readback missing: {token}")

    for signature, explanation in (
        ("bool InovanceRobotCtrl::InstallHandEyeSupportPrograms(", "当前没有经验证的汇川手眼辅助PRO模块"),
        ("bool InovanceRobotCtrl::GetHandEyeMatrixVariable(", "未定义可读取的3x3旋转"),
    ):
        body = function_body(driver, signature)
        require("return false;" in body, f"unsupported feature must fail closed: {signature}")
        require(explanation in body, f"unsupported feature lacks actionable reason: {signature}")

    require("defaultFtpPort > 0" in ui, "configuration UI does not hide unsupported FTP")
    for token in (
        "汇川已登记2222远程以太网、FTP、R/B寄存器和同工程原生JOB执行",
        "HK_WELD_JOB.pro",
        "WeldJob现场IO/DA映射完整",
        "原生JOB不支持暂停续行",
    ):
        require(token in ui, f"configuration UI lacks Inovance JOB boundary: {token}")
    for token in (
        'snapshot["WeldJob"]',
        '"ArcEnableDO"',
        '"ReadyDI"',
        '"ArcEstablishedDI"',
        '"CurrentDA"',
        '"VoltageDA"',
        '"ArcInterruptId"',
    ):
        require(token in ui, f"Inovance type template lacks safe WeldJob default: {token}")
    require(
        "PC接收steady时间（该品牌未提供控制器时间戳）" in read("src/FunctionTestDialog.cpp"),
        "timestamp diagnostic does not disclose the Inovance PC-time fallback",
    )
    for token in (
        "`ROBOT_TYPE_INOVANCE`",
        "`Get_CmdSts(id)=1`",
        "`PersistentProgramRecovery`",
        "`ActualArcWeld`",
        "`OfflineTrajectoryExport`",
        "`PauseResume`",
        "`NativeProgramExecution`",
        "`B255=10`",
        "`HK_WELD_JOB.pro`",
        "`Get_DOCfg`/`Get_DACfg`",
        "`ArcInterruptId`",
        "最多16个PRO文件",
        "PC steady",
        "首次现场使用前",
    ):
        require(token in contract, f"Inovance capability boundary is undocumented: {token}")

    print(
        "PASS: Inovance adaptor uses verified 2222 framing, model-bound FTP, controller-side generated JOB motion/weld logic, same-project dispatch, R/B completion, PC-time fallback, and explicit capability limits"
    )


if __name__ == "__main__":
    main()
