from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    step = read("src/StepRobotDriver.cpp")
    step_h = read("include/STEPRobotDriver.h")
    fanuc = read("src/FANUCRobotDriver.cpp")
    adaptor_h = read("include/RobotDriverAdaptor.h")
    service = read("src/MeasureThenWeldService.cpp")
    hand_eye = read("src/HandEyeCalibrationDialog.cpp")
    precise_editor = read("src/PreciseMeasureEditDialog.cpp")
    virtual_weld = read("src/VirtualWeldTestDialog.cpp")
    adaptor = read("src/RobotDriverAdaptor.cpp")
    timeout_policy = read("include/RobotMotionTimeoutPolicy.h")
    jogl = read("SDK/FANUC/FANUC_JOGL.ls")
    jogj = read("SDK/FANUC/FANUC_JOGJ.ls")
    production = "\n".join(
        read(path)
        for path in (
            "src/FANUCRobotDriver.cpp",
            "src/FunctionTestDialog.cpp",
            "src/MeasureThenWeldService.cpp",
            "src/QtWidgetsApplication4.cpp",
            "src/RobotJogDialog.cpp",
        )
    )

    for token in (
        'kStepCompletionWitnessName = "ntdone"',
        'kStepCompletionWitnessHoldName = "ntdonewait"',
        "kStepCompletionWitnessHoldMs = 1500",
        '"INT " << kStepCompletionWitnessName << " := 0"',
        'kStepCompletionWitnessName) + ":=0;"',
        'kStepCompletionWitnessName) + ":=1;"',
        "ArmGeneratedProgramCompletionWitness",
        "VerifyGeneratedProgramCompletionWitnessLocked",
        "VerifyGeneratedProgramReadyForStartLocked",
        "STEP START前程序身份已变化或不在可验证初始态",
        "WaitIsFinished();",
        "kStepPauseWaitTimeoutMs",
        "currentState != STEPROBOTSDK::eStop",
        "witnessRet != 0 || witnessValue != 0",
        "程序已卸载，但未稳定回读到空程序+eStop终态",
        "VariableIntModifyCmd",
        "VariableIntReadCmd",
        "STEP程序进入eStop但在%dms稳定窗口内没有自然完成见证",
        "kStepCompletionWitnessSettleTimeoutMs = 1000",
        "kStepCompletionWitnessPollIntervalMs = 50",
        "kStepCompletionWitnessStableReads = 3",
        "STEP自然完成见证稳定确认",
        "STEP自然完成运行态令牌已锁存",
        "StepBuildCompletionMessageToken",
        "m_completionWitnessRuntimeLatched = true",
        "std::thread completionObserver",
        "stopCompletionObserver",
        "completedBarrierState",
        "已自动中止并确认暂停程序不可恢复",
        "returnedToRunningState",
        "bCompletedBeforeRunConfirmation",
        "completionWitnessed = VerifyGeneratedProgramCompletionWitnessLocked",
        "已启动且在启动确认窗口内自然完成",
        "activeRunElapsed.count() >= runTimeoutMs",
        "StopAndConfirmUnverifiedMotion(this)",
    ):
        require(token in step, f"STEP terminal gate missing: {token}")

    arc_off = step.find('StepAppendCommand(oss, std::string("ARCOFF(")')
    barrier = step.find('StepAppendCommand(oss, "WaitIsFinished();")')
    witness_set = step.find('StepAppendCommand(oss, std::string(kStepCompletionWitnessName) + ":=1;")')
    message_witness = step.find('"Message(\\\"" + StepBuildCompletionMessageToken(programName)', witness_set)
    witness_hold = step.find('std::string("WaitTime(") + kStepCompletionWitnessHoldName', message_witness)
    require(0 <= arc_off < barrier < witness_set < message_witness < witness_hold,
            "STEP runtime completion token is not held after ARCOFF and the physical-motion barrier")
    require("SaveData(" not in step,
            "STEP per-motion completion witness must not wear controller storage")

    post_start = step[
        step.index("bool STEPRobotCtrl::VerifyGeneratedProgramAfterStartWithSdkUnlock"):
        step.index("bool STEPRobotCtrl::ArmGeneratedProgramContentWitness")
    ]
    observer_unlock = post_start.index("sdkLock.unlock()")
    observer_start = post_start.index("completionObserver = std::thread", observer_unlock)
    remote_verify = post_start.index("VerifyGeneratedProgramRemoteContentSnapshot", observer_start)
    observer_join = post_start.index("completionObserver.join()", remote_verify)
    observer_relock = post_start.index("sdkLock.lock()", observer_join)
    require(observer_unlock < observer_start < remote_verify < observer_join < observer_relock,
            "STEP completion token is not observed throughout the unlocked post-START FTP window")
    require(".detach()" not in post_start,
            "STEP post-START completion observer can outlive its driver state")
    arm_call = step.find("if (!ArmGeneratedProgramCompletionWitness(")
    start_call = step.find("if (!Prog_startRun_Py())", arm_call)
    require(0 <= arm_call < start_call,
            "STEP completion witness is not reset and read back before START")
    start_confirmation = step[
        start_call:
        step.index("bool STEPRobotCtrl::ServoOff()", start_call)
    ]
    require(
        start_confirmation.index("nProgramState == STEPROBOTSDK::eStop")
        < start_confirmation.index(
            "completionWitnessed = VerifyGeneratedProgramCompletionWitnessLocked")
        < start_confirmation.index("bCompletedBeforeRunConfirmation = true"),
        "STEP short program completion is not accepted through the same runtime witness")

    program_mode_command = step.find(
        "ProgramRunModeCmd(static_cast<int>(STEPROBOTSDK::eContinue))", arm_call)
    if program_mode_command < 0:
        program_mode_command = step.find(
            "ProgramRunModeCmd(continueProgramMode)", arm_call)
    require(0 <= program_mode_command < start_call,
            "STEP generated motion cannot switch to continuous mode before START")
    guarded_mode_command = step.rfind(
        "if (programModeBefore != continueProgramMode)",
        arm_call,
        program_mode_command)
    require(0 <= guarded_mode_command < program_mode_command,
            "STEP redundantly writes continuous mode even when it is already active")
    for token in (
        "const int continueProgramMode = static_cast<int>(STEPROBOTSDK::eContinue)",
        "const int programModeBefore = static_cast<int>(getProgramMode())",
        "bool programModeCommandIssued = false",
        "programStateBeforeModeCommand == STEPROBOTSDK::eStop",
        "切换连续运行模式前程序未稳定停止",
        "int stableContinueReads = 0",
        "stableContinueReads >= 3",
        "CommandIssued=%d",
        "STEP ContiMoveAny 连续运行模式确认",
    ):
        require(token in step,
                f"STEP continuous-mode confirmation missing: {token}")
    for token in (
        "m_completionWitnessProjectName",
        "m_completionWitnessMessageToken",
        "m_completionWitnessRuntimeLatched",
    ):
        require(token in step_h,
                f"STEP witness identity/runtime latch is not persisted: {token}")

    for token in (
        "FanucParseIntStrict(payload, done)",
        "done == 0 || done == 1",
        "DONE仅允许0/1",
        "CallJobWithCompletionState",
        "setAndVerifyStreamReg(FANUC_STREAM_TERMINAL_REG, 0)",
        "CallJobInternal(programName, FANUC_STREAM_TERMINAL_REG, 1, true)",
        "TryGetIntVarStrict(nStateReg, \"INT\", resetValue)",
        "m_llCompletionWitnessCallJobPcMs",
        "FANUC任务已终止，但没有同一次CALL_JOB的程序完成见证",
        "FanucElapsedMs(waitStartTime) >= runTimeoutMs",
        "FANUC任务在完成寄存器置位前已终止",
        "FANUC拒绝启动无完成寄存器契约的程序",
        "机器人服务未保留活动任务身份",
        "threadToJoin = std::move(m_continuousMoveThread)",
        "commandDefinitelyNotSent = !requestOk && !commandMayHaveBeenSent",
        "TryGetCurrentPos(T_ROBOT_COORS& pos)",
        "TryGetCurrentPulse(T_ANGLE_PULSE& pulse)",
        "FanucLinearSpeedRegister",
        "static_cast<int>(std::floor(percent))",
    ):
        require(token in fanuc, f"FANUC terminal gate missing: {token}")
    require("PrepareNextProgramCompletionState" not in fanuc,
            "FANUC witness is still configured through a racy two-step global slot")
    fanuc_check_done = fanuc[
        fanuc.index("int FANUCRobotCtrl::CheckDone()"):
        fanuc.index("int FANUCRobotCtrl::CheckDonePassive")
    ]
    require("atoi(" not in fanuc_check_done,
            "FANUC DONE payload still uses fail-open atoi parsing")

    managed_start = fanuc[
        fanuc.index("const auto startManagedStreamJob"):
        fanuc.index("const bool initRunOk")
    ]
    start_lock = managed_start.index("m_callJobStartMutex")
    terminal_reset = managed_start.index("setAndVerifyStreamReg(FANUC_STREAM_TERMINAL_REG, 0)")
    managed_call = managed_start.index(
        "CallJobInternal(programName, FANUC_STREAM_TERMINAL_REG, 1, true)")
    require(0 <= start_lock < terminal_reset < managed_call,
            "FANUC stream terminal reset/readback is not atomic with the witnessed CALL")

    for name, program in (("FANUC_JOGL", jogl), ("FANUC_JOGJ", jogj)):
        require(re.search(r"/MN\s*\n\s*1:\s+R\[85\]=0\s*;", program) is not None,
                f"{name} does not clear R[85] as its first executable instruction")
        require(re.search(r"R\[85\]=1\s*;\s*\n\s*\d+:\s+END\s*;", program) is not None,
                f"{name} does not set R[85] immediately before natural END")

    for token in (
        "bool STEPRobotCtrl::TryGetCurrentPos",
        "bool STEPRobotCtrl::TryGetCurrentPulse",
        "换算结果超出脉冲范围",
    ):
        require(token in step, f"STEP strict feedback gate missing: {token}")

    require("kMotionTimeoutMs = 1800000" in timeout_policy,
            "Cartesian admission no longer has the bounded 30-minute cap")
    require("AdmitCartesianMove" in service,
            "measure/weld flow does not apply Cartesian timeout admission")
    require(hand_eye.count("AdmitCartesianMove") >= 2,
            "hand-eye automatic moves are not admitted before both MOVL paths")
    require("FANUC扫描流程已在首条运动前拒绝" in service,
            "FANUC scan speed is not validated before the first motion")
    require("snapshot.valid = freshPose && finitePose && nonZeroPose" in adaptor,
            "state snapshots can still mark failed/stale zero poses as valid")
    require("const long long validationNowMs = RobotDriverSteadyMs()" in adaptor
            and "posePcRecvMs <= validationNowMs" in adaptor,
            "pose freshness is still compared with the timestamp taken before passive I/O")
    require("snapshot.robotMs = poseRobotMs" in adaptor
            and "snapshot.pcRecvMs = posePcRecvMs" in adaptor,
            "state snapshots still forge pose timestamps from done/now fallbacks")
    require("pRobotDriver->TryGetCurrentPos(strictPose)" in service,
            "scan pose sampling still has no strict active fallback")
    require("if (useRobotTimestampForScan)" in service
            and "steady-clock纪元混入robot_ms序列" in service,
            "robot-time scans can still mix a PC-steady active fallback into robot_ms")
    require("sample.pose = pRobotDriver->GetCurrentPos()" not in service,
            "scan pose sampling still queues the fail-open zero fallback")
    require("扫描运动期间严格机器人位姿读取失败" in service,
            "scan motion does not abort after strict pose sampling fails")
    duplicate_frame_guard = re.search(
        r"selectedTimestampMs == lastRobotMonitorMs\)\s*\{.*?return true;",
        service,
        re.S)
    require(duplicate_frame_guard is not None,
            "a repeated valid passive frame is still treated as a hard sampling failure")
    require("GetCurrentPos()" not in hand_eye,
            "hand-eye capture/wait still accepts fail-open zero poses")
    require("运动已下发，但严格读取起始位置失败" in hand_eye,
            "hand-eye wait can still disable its no-motion watchdog after a failed start read")
    require("lastProgressTime" in hand_eye and "movementSeen" not in hand_eye,
            "hand-eye no-motion protection is still cumulative instead of a rolling progress watchdog")
    require("m_hasLastTestMoveTarget = false" in hand_eye
            and "m_hasLastTestReturnPose = false" in hand_eye,
            "a failed new hand-eye diagnostic can still expose the previous move target")
    require("ConfirmRobotStoppedBeforeHandEyeMove" in hand_eye
            and "done != STEPROBOTSDK::eStop" in hand_eye
            and "done != 1" in hand_eye,
            "hand-eye MOVL entry can still stack onto running/paused/unknown robot state")
    require(hand_eye.count("ConfirmRobotStoppedBeforeHandEyeMove(driver") >= 2,
            "not every hand-eye MOVL path performs an active stopped-state check")
    require(precise_editor.count("TryGetCurrentPos") >= 3
            and precise_editor.count("TryGetCurrentPulse") >= 3,
            "scan/weld teaching entries do not preserve old values on strict-read failure")
    require("driver->TryGetCurrentPos(coors)" in virtual_weld,
            "virtual weld start capture still accepts a fail-open zero pose")
    require("m_lastWeldPosePath.clear()" in virtual_weld
            and "m_lastWeldPosePath.isEmpty() || !m_hasStartCoors" in virtual_weld,
            "a failed/repeated virtual-weld start read can still run an old trajectory")
    require("InvalidateGeneratedTrajectory" in virtual_weld
            and "每次生成尝试先作废旧产物" in virtual_weld,
            "virtual-weld input changes or failed regeneration can still run a stale trajectory")

    require("runTimeoutMs = 1800000" in adaptor_h,
            "driver completion API has no bounded default")
    require("CheckRobotDone(pollDelayMs, finishTimeoutMs)" in service,
            "service motion wait does not forward its existing timeout budget")

    one_argument_call = re.compile(r"CheckRobotDone\(\s*[^,()]+\s*\)")
    require(not one_argument_call.search(production),
            "a production CheckRobotDone caller still omits an explicit timeout")

    print("PASS: STEP/FANUC terminal success requires a program witness and all waits are bounded")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
