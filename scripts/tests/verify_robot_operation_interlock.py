from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    require(begin >= 0, f"missing section start: {start}")
    finish = text.find(end, begin + len(start))
    require(finish >= 0, f"missing section end after {start}: {end}")
    return text[begin:finish]


def require_leased(text: str, start: str, end: str, *, captured: bool = False) -> None:
    body = section(text, start, end)
    require("RobotOperationLease::TryAcquire" in body, f"hardware entry is not leased: {start}")
    if captured:
        require("operationLease" in body and "std::thread" in body,
                f"background hardware entry does not retain its lease: {start}")


def main() -> int:
    lease_h = read("include/RobotOperationLease.h")
    lease_cpp = read("src/RobotOperationLease.cpp")
    app = read("src/QtWidgetsApplication4.cpp")
    measure = read("src/MeasureThenWeldDialog.cpp")
    process_loop = read("src/ProcessLoopTestDialog.cpp")
    virtual_weld = read("src/VirtualWeldTestDialog.cpp")
    function_test = read("src/FunctionTestDialog.cpp")
    jog = read("src/RobotJogDialog.cpp")
    hand_eye = read("src/HandEyeCalibrationDialog.cpp")
    matrix = read("src/HandEyeMatrixDialog.cpp")
    fanuc_driver = read("src/FANUCRobotDriver.cpp")
    step_driver = read("src/StepRobotDriver.cpp")

    for token in ("TryAcquire", "SetNewOperationsAllowed", "NewOperationsAllowed",
                  "AddNewOperationsBlock", "RemoveNewOperationsBlock",
                  "CurrentOwner", "AnyActive", "ActiveSummary", "Matches",
                  "MarkMotionStarted", "MarkMotionCompleted", "MotionCompletionPending",
                  "StopAndConfirmUnverifiedMotion"):
        require(token in lease_h, f"lease API missing: {token}")
    for token in ("std::mutex", "g_activeOperations", "g_newOperationsAllowed",
                  "g_newOperationBlocks", "g_nextOperationBlockToken",
                  "g_newOperationsBlockedReason", "g_nextOperationToken", "second.token == m_token",
                  "NormalizeSocketHost", "m_sSocketIP", "m_nSocketPort", "m_identityKey"):
        require(token in lease_cpp, f"lease registry safety mechanism missing: {token}")
    for token in ("motionCompletionPending", "UnresolvedStop{ m_driver, true }",
                  "AbortCurrentProgramSafely", "ConfirmCancellationHandled"):
        require(token in lease_cpp, f"unverified motion fail-closed mechanism missing: {token}")
    require("QString m_identityKey" in lease_h and "bool Matches" in lease_h,
            "lease does not retain and compare its frozen physical endpoint identity")
    require("MarkMotionStarted(this, false" in fanuc_driver
            and "MarkMotionCompleted(this)" in fanuc_driver
            and "StopAndConfirmUnverifiedMotion(this)" in fanuc_driver,
            "FANUC accepted-task lifecycle is not tied to a verified terminal state")
    require("MarkMotionStarted(this, resumeExisting" in step_driver
            and "MarkMotionCompleted(this)" in step_driver
            and "StopAndConfirmUnverifiedMotion(this)" in step_driver,
            "STEP START lifecycle is not tied to abort/eStop verification")
    for token in ("m_motionTrackedProjectName", "m_motionTrackedProgramName",
                  "currentProject != m_motionTrackedProjectName",
                  "currentProgram != m_motionTrackedProgramName"):
        require(token in step_driver or token in read("include/STEPRobotDriver.h"),
                f"STEP pause/resume program identity guard missing: {token}")

    for start, end in (
        ("void MeasureThenWeldDialog::RunPresetParamFlow()", "void MeasureThenWeldDialog::RunSkipScanWeldFlow()"),
        ("void MeasureThenWeldDialog::RunSkipScanWeldFlow()", "void MeasureThenWeldDialog::RunLineScanProcess()"),
        ("void MeasureThenWeldDialog::RunResumeWeldFlow()", "void MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow()"),
        ("void MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow()", "void MeasureThenWeldDialog::RefreshWeldModeFromParam()"),
    ):
        require_leased(measure, start, end, captured=True)
    require("ShowNonModalFlowResult" in measure
            and "setWindowModality(Qt::NonModal)" in measure
            and "QMessageBox::warning(self" not in measure
            and "QMessageBox::information(self" not in measure,
            "measure-then-weld background result can block safety-stop retry")

    require_leased(process_loop, "void ProcessLoopTestDialog::OnStart()", "void ProcessLoopTestDialog::OnStop()", captured=True)
    process_loop_runner = section(
        app,
        "void QtWidgetsApplication4::RunProcessLoopTest(",
        "void QtWidgetsApplication4::OpenProcessLoopTestPage()")
    require("!RobotOperationLease::NewOperationsAllowed()" in process_loop_runner
            and "stopped" in process_loop_runner,
            "long-lived process-loop lease can start another cycle after account-session revocation")
    require(process_loop_runner.count("stopped()") >= 8,
            "process loop does not recheck session/stop state at cycle, weld, and interval boundaries")
    virtual_start = virtual_weld.find("void VirtualWeldTestDialog::OnRunOnRobot()")
    require(virtual_start >= 0, "missing virtual-weld hardware entry")
    virtual_body = virtual_weld[virtual_start:]
    require("RobotOperationLease::TryAcquire" in virtual_body
            and "std::thread" in virtual_body
            and "operationLease" in virtual_body,
            "virtual-weld background execution does not retain a robot lease")

    for start, end in (
        ("void FunctionTestDialog::FanucCurposDiagnosticTest()", "void FunctionTestDialog::RobotCameraTimestampDiagnosticTest()"),
        ("void FunctionTestDialog::FanucSetGetIntTest()", "void FunctionTestDialog::FanucSetTpSpeedTest()"),
        ("void FunctionTestDialog::FanucSetTpSpeedTest()", "void FunctionTestDialog::FanucCallJobTest()"),
        ("void FunctionTestDialog::FanucCallJobTest()", "void FunctionTestDialog::FanucUploadLsTest()"),
        ("void FunctionTestDialog::FanucUploadLsTest()", "void FunctionTestDialog::FanucMovlTest()"),
        ("void FunctionTestDialog::FanucMovlTest()", "void FunctionTestDialog::FanucMovjTest()"),
        ("void FunctionTestDialog::FanucMovjTest()", "void FunctionTestDialog::FanucMoveZeroTest()"),
        ("void FunctionTestDialog::FanucMoveZeroTest()", "void FunctionTestDialog::FanucCaptureKinematicsSample()"),
    ):
        require_leased(function_test, start, end, captured="std::thread" in section(function_test, start, end))

    for start, end in (
        ("void RobotJogDialog::MoveToCartesianTarget()", "void RobotJogDialog::MoveToJointTarget()"),
        ("void RobotJogDialog::MoveToJointTarget()", "void RobotJogDialog::StartJog("),
        ("void RobotJogDialog::BeginJog()", "void RobotJogDialog::StepJog("),
        ("void RobotJogDialog::StepJog(", "void RobotJogDialog::StopJog()"),
    ):
        require_leased(jog, start, end)
    require("m_jogOperationLease" in jog and "EndContinuousMoveQueue" in jog,
            "continuous jog lease must survive until the driver queue is joined")
    require("setWindowModality(Qt::NonModal)" in jog
            and "QMessageBox::warning" not in jog
            and "msgBox.exec()" not in jog,
            "robot-jog failure feedback can block the homepage safety stop")

    for start, end in (
        ("bool HandEyeCalibrationDialog::CaptureTcpPoint()", "bool HandEyeCalibrationDialog::CaptureSample("),
        ("bool HandEyeCalibrationDialog::CaptureSample(", "bool HandEyeCalibrationDialog::EnsureCameraReady("),
        ("bool HandEyeCalibrationDialog::TestHandEyeMatrix()", "bool HandEyeCalibrationDialog::CheckCameraTimestampIntervals()"),
        ("bool HandEyeCalibrationDialog::UploadRobotHandEyeCheckProgram(", "void HandEyeCalibrationDialog::SetAutoCalibrationUiRunning("),
        ("bool HandEyeCalibrationDialog::UploadAutoCalibrationProgram()", "bool HandEyeCalibrationDialog::StartAutoCalibration()"),
        ("bool HandEyeCalibrationDialog::StartAutoCalibration()", "void HandEyeCalibrationDialog::OpenMatrixDialog()"),
    ):
        require_leased(hand_eye, start, end, captured=start.endswith("StartAutoCalibration()"))
    require("m_bAutoCalibrationRunning.load()" in section(
        hand_eye, "void HandEyeCalibrationDialog::closeEvent(", "bool HandEyeCalibrationDialog::LoadConfig()"),
        "hand-eye window can close while automatic robot motion is active")
    require_leased(matrix, "bool HandEyeMatrixDialog::ReadRobotEyeVariable()", "bool HandEyeMatrixDialog::SaveConfig()")

    for start, end in (
        ("void QtWidgetsApplication4::RobotRunTest()", "void QtWidgetsApplication4::OpenWeldProcessDialog()"),
        ("void QtWidgetsApplication4::FanucConnectTest()", "void QtWidgetsApplication4::FanucDisconnectTest()"),
        ("void QtWidgetsApplication4::FanucDisconnectTest()", "void QtWidgetsApplication4::RobotClearAlarmTest()"),
        ("void QtWidgetsApplication4::RobotClearAlarmTest()", "void QtWidgetsApplication4::RobotSwitchStepMode()"),
        ("void QtWidgetsApplication4::ReadTool1ToGunTool()", "void QtWidgetsApplication4::FanucGetCurrentPosTest()"),
        ("void QtWidgetsApplication4::FanucSetGetIntTest()", "void QtWidgetsApplication4::FanucSetTpSpeedTest()"),
        ("void QtWidgetsApplication4::FanucSetTpSpeedTest()", "void QtWidgetsApplication4::FanucCallJobTest()"),
        ("void QtWidgetsApplication4::FanucCallJobTest()", "void QtWidgetsApplication4::FanucUploadLsTest()"),
        ("void QtWidgetsApplication4::FanucUploadLsTest()", "void QtWidgetsApplication4::FanucMovlTest()"),
        ("void QtWidgetsApplication4::FanucMovlTest()", "void QtWidgetsApplication4::FanucMovjTest()"),
        ("void QtWidgetsApplication4::FanucMovjTest()", "void QtWidgetsApplication4::FanucMoveZeroTest()"),
        ("void QtWidgetsApplication4::FanucMoveZeroTest()", "void QtWidgetsApplication4::OpenRobotJogDialog()"),
    ):
        require_leased(app, start, end)

    require_leased(app, "void QtWidgetsApplication4::RunRobotMotionForCli(", "bool QtWidgetsApplication4::UploadFanucServiceBundleForCli(")
    require_leased(app, "bool QtWidgetsApplication4::RunMeasureThenWeldScanOnlyRepeatForCli(", "ProcessLoopTestDefaults QtWidgetsApplication4::LoadProcessLoopTestDefaults(")
    require("--robot-no-wait 已因全局硬件互锁禁用" in app,
            "CLI no-wait can release the lease while physical motion continues")
    require("fanucCliLease" in app and "CheckRobotDone" in section(
        app, "void QtWidgetsApplication4::RunCommandLineActions(", "FANUCRobotCtrl* QtWidgetsApplication4::GetFirstFanucDriverForCli()"),
        "CLI FANUC operations are not held through completion")
    ftp_task = section(app, "void RunFtpTask(", "void RefreshRemoteFiles()")
    require("FTP Job：%1" in app and "operationLease" in ftp_task,
        "FTP job operations are not covered by the per-robot lease")
    require("m_liveSessionGuard" in ftp_task
            and ftp_task.find("m_liveSessionGuard") < ftp_task.find("RobotOperationLease::TryAcquire"),
            "FTP native-dialog return can reach a robot lease without synchronous live-session validation")
    require("提交 FTP Job 文件操作" in app
            and "RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer)" in app,
            "FTP task submission is not tied to a current engineer session")
    for token in ("m_bEnforceInteractiveSessionLeaseGate", "账号会话已失效；禁止启动新的机器人硬件操作。"):
        require(token in app, f"interactive account-to-lease gate missing: {token}")
    require(app.count("RobotOperationLease::SetNewOperationsAllowed(true)") == 3,
            "only successful login/guest and teardown may reopen the lease gate")
    require(app.count("RobotOperationLease::SetNewOperationsAllowed(") >= 8,
            "login/logout/session-revocation paths do not consistently drive the lease gate")
    auth_page = section(app, "void QtWidgetsApplication4::ShowAuthPage(", "bool QtWidgetsApplication4::VerifyAccount(")
    require("RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow()" in auth_page
            and "setWindowModality(Qt::NonModal)" in auth_page,
            "account switching can hide the only safety-stop entry during an active robot flow")
    require(auth_page.find("RobotOperationLease::AnyActive()")
            < auth_page.find("RobotOperationLease::SetNewOperationsAllowed("),
            "voluntary account switching closes the lease gate before preserving active-flow safety controls")

    close_guard = section(app, "void QtWidgetsApplication4::closeEvent(", "bool QtWidgetsApplication4::eventFilter(")
    reload_guard = section(app, "auto reloadControlUnits = [this]()", "auto unitIndexForRobotName = [this]")
    camera_mode_guard = section(app, "void QtWidgetsApplication4::SetSharedScanCameraReceiverMode(", "void QtWidgetsApplication4::SetAuthRegisterMode(")
    for name, body in (("application close", close_guard), ("control-unit reload", reload_guard), ("camera topology switch", camera_mode_guard)):
        require("RobotOperationLease::AnyActive()" in body, f"{name} ignores active robot leases")
    require("RobotOperationLease::AnyActive()" in section(
        app, "bool QtWidgetsApplication4::EnsureScanCameraRunningForUnit(", "CameraFrameCache* QtWidgetsApplication4::ScanCameraCacheForUnit("),
        "shared camera topology can be destroyed during another robot flow")
    require(app.count("RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow()") >= 3,
            "OTA installation guard does not cover every robot hardware operation")

    print("PASS: per-robot operation leases cover hardware entries and destructive lifecycle gates")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
