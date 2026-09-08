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
    calibration = read("src/InovanceCalibration.cpp")
    weld_calibration = read("include/InovanceWeldCalibration.h")
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

    protocol = function_body(driver, "bool InovanceRobotCtrl::SendCommandLocked(")
    for token in (
        '"@@" + command + "$$"',
        'framed.find("$$")',
        'framed.find("##")',
        "kMaxProtocolResponse",
        "CloseSocketLocked()",
        "ProtocolErrorText(response)",
    ):
        require(token in protocol, f"Inovance framed protocol evidence missing: {token}")
    send = function_body(driver, "bool InovanceRobotCtrl::SendCommand(")
    query = function_body(driver, "bool InovanceRobotCtrl::QueryInt(")
    connect = function_body(driver, "bool InovanceRobotCtrl::ConnectWithPolicy(bool explicitRetry)")
    close = function_body(driver, "bool InovanceRobotCtrl::CloseSocketLocked()")
    for body in (send, query, connect):
        require("m_socketMutex" in body, "connection/login and commands must share the socket lock")
    for body in (send, query):
        require("m_connectionReady.load()" in body, "unauthenticated commands must be blocked")
    require(connect.index("LoginUserLocked()") < connect.index("m_connectionReady.store(true)"),
            "Connect must not succeed before login and permission readback")
    require("QueryIntLocked(\"Get_ConnectState\"" in connect,
            "connection handshake must use the already-locked query path")
    require("CloseSocketLocked();\n        return false;" in connect,
            "failed connection/authentication must close the transport")
    for field in ("m_userLoggedIn", "m_connectionReady"):
        require(f"{field}.store(false)" in close, "disconnect must invalidate login state")
    require("m_connectionReady.load()" in function_body(driver, "bool InovanceRobotCtrl::IsConnected()"),
            "half-authenticated TCP must not appear connected")
    login = function_body(driver, "bool InovanceRobotCtrl::LoginUserLocked()")
    require("InovanceUserLogin::Login" in login and "QueryIntLocked" in login,
            "driver must execute the tested login implementation")
    for forbidden in ("Motor ON", "Set_Mode", "Set_ToolCNum", "Set_WobjNum", "Prg Start"):
        require(forbidden not in connect + login, "auto-login must not operate robot/coordinates")
    require('command.rfind("UserLogin ", 0)' in protocol and "InovanceUserLogin::RejectionDetail(response)" in protocol,
            "login errors must not echo credentials from the controller")
    require("ConnectWithPolicy(true)" in function_body(driver, "bool InovanceRobotCtrl::Connect()"),
            "explicit connection action must allow an operator-directed retry")
    monitor = function_body(driver, "void InovanceRobotCtrl::EnsureConnectionForMonitor()")
    require("ConnectWithPolicy(false)" in monitor and "Connect();" not in monitor,
            "monitor must not repeatedly retry rejected/uncertain login")
    require("m_loginRetry.Begin(explicitRetry, blockedError)" in connect
            and "m_loginRetry.Block(error)" in login and "m_loginRetry" not in close,
            "failed authentication must remain latched across transport closure")
    for body in (send, query):
        require("m_loginRetry.Error()" in body, "disconnected commands must preserve login diagnosis")
    for command in ("Get_FwVersion", "CurCtrlDev", "CurUserType", "CurPermit"):
        require(command in login, f"login failure diagnosis missing: {command}")

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
        "CircularMotion",
        "RealRegister",
        "StructuredControllerStatus",
        "HandEyeMatrixRead",
        "ControllerKinematicsCalculate",
        "CalibrationAssetDiscovery",
    ):
        require(
            f"RobotDriverCapability::{capability}" in capabilities,
            f"Inovance implemented capability missing: {capability}",
        )
    for unsupported in (
        "RobotTimestamp",
        "PersistentProgramRecovery",
        "HandEyeSupportProgramInstall",
    ):
        require(
            f"RobotDriverCapability::{unsupported}" not in capabilities,
            f"Inovance must not claim unverified capability: {unsupported}",
        )
    require("m_connectionReady.load() && m_kinematicsSession.Ready()" in capabilities,
            "JointMotion must require this connection's live validated AxisUnit, not cached configuration")
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
    for token in ("Get_ToolCNum", "Get_WobjNum", "InovanceConnectionPreparation::Prepare",
                  "RobotOperationLease::CurrentOwner(this)", "m_modeConnectionEpoch.load()"):
        require(token in initialize, f"initialization identity check missing: {token}")
    require("Connect()" not in initialize,
            "explicit connection preparation must not reconnect or allow monitor-triggered energizing")
    require("m_explicitCoordinateSetup" not in initialize
            and '"Set_ToolCNum"' in initialize and '"Set_WobjNum"' in initialize,
            "explicit connection preparation must always set and verify Tool1/Wobj1")

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
    circular = function_body(driver, "bool InovanceRobotCtrl::SendCircularMove(")
    for token in (
        "via.dRZ",
        "target.dRZ",
        '"MovCRobP "',
        '"Get_CurCmdNum"',
        "m_externalValues",
        "m_armConfig",
    ):
        require(token in circular, f"Inovance circular mapping missing: {token}")
    require("viaParameter.size() > 128" in circular,
            "MovCRobP must enforce the manual parameter limit")
    direct_moves = (
        function_body(driver, "bool InovanceRobotCtrl::MoveLinearMmPerMin("),
        function_body(driver, "bool InovanceRobotCtrl::MoveCircularMmPerMin("),
        function_body(driver, "bool InovanceRobotCtrl::MoveJointPercent("),
    )
    for direct_move in direct_moves:
        for token in ("EnsureMotionReady", 'SetDataStreamMode("ON", 1)',
                      "BeginTrackedDirectMotion"):
            require(token in direct_move, f"Inovance direct-move acceptance lifecycle missing: {token}")
        require("WaitForCommandDone" not in direct_move,
                "Move* must return after command acceptance so scan sampling can run concurrently")
    move_circular = direct_moves[1]
    require("SendCircularMove" in move_circular, "Inovance circular command mapping is missing")
    joint = function_body(driver, "bool InovanceRobotCtrl::SendJointMove(")
    for token in ("MovJAbsRobJP", "dSPulseUnit", "Get_CurCmdNum"):
        require(token in joint, f"Inovance joint conversion missing: {token}")
    require("size() > 128" in joint, "MovJAbsRobJP must enforce the manual parameter limit")

    start = function_body(driver, "bool InovanceRobotCtrl::StartTrajectory(")
    wait = function_body(driver, "bool InovanceRobotCtrl::WaitTrajectory(")
    track_direct = function_body(driver, "bool InovanceRobotCtrl::BeginTrackedDirectMotion(")
    finalize_direct = function_body(driver, "bool InovanceRobotCtrl::FinalizeCompletedDataStreamMotion(")
    check_done = function_body(driver, "int InovanceRobotCtrl::CheckRobotDone(")
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
    for token in ("m_activeHandle", "m_finalCommandId", "m_trajectoryRunning"):
        require(token in track_direct, f"direct motion identity is not frozen: {token}")
    for token in ('SetDataStreamMode("OFF", 0)', "m_activeHandle.started = false",
                  "m_finalCommandId = -1"):
        require(token in finalize_direct, f"direct motion terminal cleanup is incomplete: {token}")
    require("FinalizeCompletedDataStreamMotion" in check_done,
            "CheckRobotDone must close the tracked data stream after terminal witness")
    require("WaitForCommandDone(trackedCommandId" in check_done,
            "CheckRobotDone must retain the stable command-id completion witness")
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
        "if (motion == 2)",
        "returnedToStart",
        "BackStartLine后未连续确认Get_MotionSts=0",
        "taskStatus != 1 && motion == 0",
    ):
        require(token in abort, f"interrupted-state cleanup missing: {token}")
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
        "bHasTrackParam",
        "bHasWeaveParam",
        "bAppPointwiseWeave",
        "InovanceNativeWeaveShape",
        "arcSegments",
        "ArcTrackData",
    ):
        require(token in validate_job, f"Inovance weld JOB validation missing: {token}")
    require(
        "过渡电流/电压尚无连续切换证明" not in validate_job,
        "Inovance must not reject transition weld parameters now handled by WeldSet",
    )

    generate_job = function_body(driver, "bool InovanceRobotCtrl::WriteTrajectoryJobFile(")
    for token in (
        "InovanceProgramInfo",
        "ReadControllerProgramRobotName",
        "controllerRobotName",
        "kInovanceCallableFunction",
        '"LP["',
        '"Movl LP["',
        '"Movj LP["',
        '"WeldOn ArcData["',
        '"WeldSet ArcData["',
        '"WeldOff ArcData["',
        '"],RPM["',
        '"],ArcOffT["',
        "AppendInovanceWeaveCommand",
        '"WeaveOff;\\r\\n"',
        '"EndFunc;\\r\\n"',
        "kInovanceProgramInstructionLimit",
        "programContentSha256",
        "programContentSize",
    ):
        require(token in generate_job, f"Inovance controller JOB generator missing: {token}")
    for token in ('"] =  "', '<< "; "', '<< ";\\r\\n"'):
        require(token in generate_job,
                f"Inovance controller-export LP field syntax missing: {token}")
    require(
        all(token not in generate_job for token in ('"] = {("', '<< "), ("', '<< ")};\\r\\n"')),
        "Inovance PRO LP assignment must not use aggregate braces/parentheses",
    )
    require(
        all(token not in generate_job for token in ('"Set DA["', '"Set Out["', '"MovAbsJ JP["')),
        "Inovance native JOB must not fall back to legacy DA/DO welding or global JP points",
    )
    for token in ("speedMmPerSecond", 'const std::string zone = "Fine"', '<< speedMmPerSecond',
                  '<< "],Wobj[" << m_wobjNo << "];\\r\\n"'):
        require(token in generate_job,
                f"Inovance pendant-authored MOVL form missing: {token}")
    require('std::string("Z[0]")' not in generate_job,
            "Inovance generated trajectory must use Fine for every point")
    require(generate_job.count('<< "],Wobj[" << m_wobjNo') == 2,
            "Inovance MovL and MovJ must both bind every point to Wobj1")
    for token in ('"].Bstatic:1,"',):
        require(token not in generate_job,
                f"Inovance generated motion must not reintroduce field-rejected syntax: {token}")
    require(
        all(token not in generate_job for token in (
            "QTWIDGETSAPP4_INOVANCE_TRAJECTORY_JOB",
            "PC_TIMESTAMP=",
            "Native LP syntax confirmed",
        )),
        "Inovance generated PRO must not place module-scope comments before LP declarations",
    )
    validate_module = function_body(driver, "bool ValidateInovanceCallableModule(")
    for token in (
        "localPointDeclaration",
        "localPointIndexes",
        "moduleScopeComment",
        "ProgramInfo后、局部点声明前不能放置模块级注释",
        "公共模块重复声明LP[",
    ):
        require(token in validate_module,
                f"Inovance callable module duplicate-LP gate missing: {token}")
    fingerprint = function_body(driver, "std::uint64_t InovanceRobotCtrl::FingerprintMoveInfos(")
    for token in (
        "bUseTransitionWeldParams",
        "tWeaveParam",
        "tTrackParam",
        "m_toolNo",
        "m_wobjNo",
    ):
        require(token in fingerprint, f"frozen weld JOB identity missing: {token}")

    upload_job = function_body(driver, "bool InovanceRobotCtrl::UploadTrajectoryJob(")
    for token in (
        "Get_TaskRunSts 0",
        "InovanceActiveMainProgram",
        "RegisterInovanceProgramInProject",
        "remoteProjectPath",
        "projectBackupPath",
        "projectChanged",
        "previousProgramPath",
        "previousDynamicProgramPath",
        "rollbackTargetProgram",
        "rollbackDynamicProgram",
        "dynamicRemotePath",
        "_dynamic_uploaded_verify.pro",
        "kInovanceProgramFileLimit",
        "UploadProgramFile",
        "DownloadProgramFile",
        "InovanceContentSha256",
        "dataContentSha256",
        "dataContentSize",
    ):
        require(token in upload_job, f"Inovance generated JOB upload gate missing: {token}")
    require(
        "UploadProgramFile(handle.localProgramPath, remotePath, false)" in upload_job,
        "Inovance JOB replacement must not delete the existing remote PRO before upload",
    )
    require(
        "handle.localProgramPath, dynamicRemotePath, false" in upload_job,
        "Inovance Call target must be synchronized to Task0/DynamicCall",
    )
    require(
        "handle.remoteProgramPath = dynamicRemotePath" in upload_job,
        "Inovance handle must bind the executable DynamicCall copy",
    )
    register_project = function_body(driver, "bool RegisterInovanceProgramInProject(")
    for token in (
        'QStringLiteral("EnterProgramFile")',
        'root.value(QStringLiteral("ProgramFiles")).toArray()',
        'name.compare(requested, Qt::CaseInsensitive) == 0',
        'projectPrograms.size() > kInovanceProgramFileLimit',
        'root.insert(QStringLiteral("ProgramFilesCount"), projectProgramCount)',
    ):
        require(token in register_project,
                f"Inovance Call-target PRJ registration gate missing: {token}")
    verify_job = function_body(
        driver, "bool InovanceRobotCtrl::VerifyTrajectoryJobRemoteIdentity("
    )
    for token in (
        "DownloadProgramFile",
        "programContentSize",
        "programContentSha256",
        "remoteDataPath",
        "dataContentSize",
        "dataContentSha256",
    ):
        require(token in verify_job, f"Inovance pre-start JOB identity check missing: {token}")

    weld_contract = function_body(
        driver, "bool InovanceRobotCtrl::HasVerifiedWeldJobContract("
    )
    for token in ("m_toolNo", "m_wobjNo", "kApplicationGunToolNumber", "已标定Tool1和Wobj1"):
        require(token in weld_contract, f"Inovance native weld contract missing: {token}")
    require(
        "m_weldJobEnabled" not in weld_contract,
        "legacy WeldJob IO/DA mapping must not gate native ArcData welding capability",
    )
    prepare_welder = function_body(driver, "bool InovanceRobotCtrl::PrepareWeldJobHardware(")
    for token in (
        "EnsureMotionReady",
        "!m_weldJobEnabled",
        "Get_DOCfg",
        "Get_DACfg",
        "ConfirmWeldArcOutputOff",
    ):
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
    for token in ("ServoOff", "RemovePermit"):
        require(token in shutdown, f"safe disconnect sequence missing: {token}")
    servo_off = function_body(driver, "bool InovanceRobotCtrl::ServoOff()")
    for token in ("AbortCurrentProgramSafely", "InovanceModeSequence::RestoreOff", "originalMode"):
        require(token in servo_off, f"verified ServoOff sequence missing: {token}")
    sequence = read("include/InovanceModeSequence.h")
    for token in ('"Motor OFF"', 's.motor == 0', '"Dsmode OFF"', 's.stream == 0',
                  'reply.find("e4:")', 'state.mode != 2', 'SetMode(ops, 1, trace, true)',
                  'sample < 3', 'state.mode != originalMode'):
        require(token in sequence, f"verified brand mode-sequence recovery missing: {token}")
    for forbidden in ('"Prg Start"', '"EStop OFF"', '"ResetErr"', '"MovL', '"MovJ', '"MovC'):
        require(forbidden not in sequence, f"mode diagnostics must not issue motion/reset: {forbidden}")
    for token in ('ModePreparationTestCases() const', 'RunModePreparationTestCase(',
                  'm_verifiedModePreparations[id] = epoch', 'm_modeConnectionEpoch.fetch_add(1)',
                  'InovanceModeSequence::Find(ActiveModePreparationId(), plan)'):
        require(token in driver, f"mode recipe proof/connection binding missing: {token}")
    require(driver.count('!SetDataStreamMode("ON", 1) || !EnsureMotionReady()') == 5,
            "all five stream motion entry points must prepare the selected recipe before motor-ready validation")

    transfer = function_body(
        driver, "std::shared_ptr<RobotFileTransferSession> InovanceRobotCtrl::CreateFileTransferSession("
    )
    for token in ("RobotFtpFileTransfer", "m_ftpIp", "m_ftpPort", "FileTransferProfile()"):
        require(token in transfer, f"Inovance FTP session wiring missing: {token}")
    profile = function_body(driver, "RobotFileTransferProfile InovanceRobotCtrl::FileTransferProfile() const")
    for token in ('"/TeachProgram"', '"*.pro"', '"*.prj"', '"*.pts"', '"*.jsn"', '"*.dat"', 'acceptanceProgramExtensions = { ".pro" }'):
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
        "ParseInovanceProgramRobotName",
        "ReadControllerProgramRobotName",
        "moduleRobotName != controllerRobotName",
        "mainRobotName != controllerRobotName",
        "RegisterInovanceProgramInProject",
        "projectWouldChange",
        "WriteInovanceDispatcher",
        "UploadProgramFile",
        "main_uploaded_verify.pro",
        "main_restored_verify.pro",
        'SendCommand("Get_SysErr", faultCode)',
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
    require('"/DynamicCall/" + actualModuleFile' in execute,
            "Inovance pre-start verification must read the DynamicCall target")
    active_main = function_body(driver, "bool InovanceActiveMainProgram(")
    for token in ("/TeachProgram/", "projectBegin", "projectEnd", "projectName"):
        require(token in active_main, f"Inovance V2/V3 project identity parsing missing: {token}")
    for token in (
        "InovanceProgramInfo",
        '<< "Call \\\""',
        '<< kInovanceCallableFunction',
        "B[",
        "= 10",
        "Func",
        "func1",
    ):
        require(token in driver, f"Inovance dispatcher/module contract missing: {token}")
    dispatcher = function_body(driver, "bool WriteInovanceDispatcher(")
    require("QTWIDGETSAPP4_INOVANCE_DISPATCHER" not in dispatcher,
            "Inovance dispatcher must not emit module-scope comments after ProgramInfo")
    require('Include ' not in dispatcher and '.Run();' not in dispatcher,
            "Inovance permanent dispatcher must use Call, not Include/module.Run")

    get_int = function_body(driver, "bool InovanceRobotCtrl::TryGetIntVar(")
    set_int = function_body(driver, "bool InovanceRobotCtrl::SetIntVar(\n    int index")
    for token in ("Get_R ", "Get_B ", "Get_PlcVar DInt"):
        require(token in get_int, f"Inovance integer read mapping missing: {token}")
    for token in ("Set_R ", "Set_B ", "TryGetIntVar", "verified != value"):
        require(token in set_int, f"Inovance integer write/readback missing: {token}")

    get_real = function_body(driver, "bool InovanceRobotCtrl::TryGetRealVar(")
    set_real = function_body(driver, "bool InovanceRobotCtrl::SetRealVar(")
    for token in ("Get_D ", "values.size() != 1", "0..255"):
        require(token in get_real, f"Inovance D real read mapping missing: {token}")
    for token in ("Set_D ", "TryGetRealVar", "9999999.999"):
        require(token in set_real, f"Inovance D real write/readback missing: {token}")

    set_position = function_body(driver, "bool InovanceRobotCtrl::WriteCartesianRegister(")
    for body, name in ((set_int, "R/B"), (set_real, "D"), (set_position, "P")):
        require("if (!EnsureControlPermit()) { return false; }" in body,
                f"{name} writes must fail closed if control permit preparation fails")
        require(body.index("EnsureControlPermit()") < body.index("SendCommand("),
                f"{name} write was sent before verifying controller permission")
        for forbidden in ("EnsureMotionReady", "Motor ON", "SetOperationMode", "Connect()"):
            require(forbidden not in body, f"register write must not implicitly prepare motion: {forbidden}")
    require("owner != 0 && owner != 2" in permit and "owner == 2 && !m_forceControlPermit" in permit,
            "invalid or externally owned permission must fail closed")

    controller_status = function_body(driver, "RobotControllerStatus InovanceRobotCtrl::ReadControllerStatus()")
    for token in (
        "Get_Mode",
        "Get_EStopSts",
        "Get_MotorSts",
        "Get_MotionSts",
        "Get_SysErrSts",
        "QuerySystemErrorCode",
        "CurCtrlDev",
        "CurPermit",
    ):
        require(token in controller_status, f"Inovance structured status missing: {token}")
    require("QueryLeadingInt" in permit and "QueryLeadingInt" in controller_status,
            "CurPermit must accept the documented trailing owner endpoint fields")

    kinematics = function_body(driver, "bool InovanceRobotCtrl::RefreshKinematicsFromController(")
    for token in (
        "Get_StrPara", "Get_StrParaComp", "Get_RdctRatio", "Get_ZeroPos",
        "Get_AxisNLim", "Get_AxisPLim", "Get_RobJPHere", "Get_PosHerePulse",
        "Get_ToolCNum", "Get_WobjNum", "Get_WobjData",
        "i32EncBit", "dCoupParam",
        "InstallValidatedKinematicsModel", "positionErrorMm > 2.0",
        "orientationErrorDeg > 0.1",
    ):
        require(token in kinematics, f"Inovance read-only kinematics acquisition missing: {token}")
    require("ControllerKinematicsRead" in driver,
            "Inovance validated kinematics capability is not declared")
    require('/RobotParams/MachineParams.json' in driver,
            "Inovance MachineParams FTP source is not fixed")

    body = function_body(driver, "bool InovanceRobotCtrl::InstallHandEyeSupportPrograms(")
    require("return false;" in body and "当前没有经验证的汇川手眼辅助PRO模块" in body,
            "unsupported hand-eye support program must fail closed")
    hand_eye = function_body(driver, "bool InovanceRobotCtrl::GetHandEyeMatrixVariable(")
    require("ReadControllerHandEye(0" in hand_eye and "camera-to-tool-tcp" in hand_eye,
            "legacy hand-eye read must delegate to the typed controller calibration contract")
    for token in (
        "/RCFamily/Controller/Teachology/InoRobPluginWeld/config.xml",
        "WELD_LaserConfig", "Get_ToolData", "Get_ToolCNum",
        "ReadKinematicsReference", "CalculateControllerForward",
        "CalculateControllerInverse", "DiscoverCalibrationAssets",
        "ReadControllerHandEye", "ValidateControllerHandEyeContext",
    ):
        require(token in calibration or token in weld_calibration,
                f"Inovance fixed calibration/kinematics source missing: {token}")

    require("defaultFtpPort > 0" in ui, "configuration UI does not hide unsupported FTP")
    for token in (
        "汇川已登记2222远程以太网、FTP、R/B/D寄存器、MOVL/MOVJ/MOVC、伺服上下电、结构化控制器状态和同工程原生JOB执行",
        "HK_WELD_JOB.pro",
        "WeldOn/WeldSet/WeldOff",
        "WeldJob IO/DA映射仅作为可选额外关弧见证",
        "原生JOB暂停续行",
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
        "`WeldOn ArcData[0],AC[],AV[],WS[],RPM[0]`",
        "`ArcTrackData`",
        "`Get_D`/`Set_D`",
        "`MovCRobP`",
        "`Get_SysErr`",
        "最多16个PRO文件",
        "PC steady",
        "首次现场使用前",
    ):
        require(token in contract, f"Inovance capability boundary is undocumented: {token}")

    print(
        "PASS: Inovance adaptor uses verified 2222 framing, model-bound FTP, MOVC, R/B/D readback, structured status, field-native ArcData/WeaveData JOB generation, V2/V3 same-project dispatch, PC-time fallback, and explicit capability limits"
    )


if __name__ == "__main__":
    main()
