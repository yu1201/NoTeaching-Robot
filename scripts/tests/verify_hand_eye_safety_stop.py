from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    require(begin >= 0, f"missing section: {start}")
    finish = text.find(end, begin + len(start))
    require(finish >= 0, f"missing section end: {end}")
    return text[begin:finish]


def main() -> int:
    header = (ROOT / "include/HandEyeCalibrationDialog.h").read_text(encoding="utf-8")
    source = (ROOT / "src/HandEyeCalibrationDialog.cpp").read_text(encoding="utf-8")

    constructor = section(
        source,
        "HandEyeCalibrationDialog::HandEyeCalibrationDialog(",
        "void HandEyeCalibrationDialog::closeEvent(",
    )
    for token in (
        'm_pSafetyStopBtn = new QPushButton("安全停止本窗口机器人任务"',
        'm_pMoveToLastTestPointBtn = new QPushButton("运动到最近检测点"',
        'm_pReturnToLastTestPoseBtn = new QPushButton("退回检测起点"',
        "windowLayout->addWidget(safetyBar",
        "m_pScrollContent = new QWidget(scrollArea)",
        "RequestSafetyStop()",
    ):
        require(token in constructor, f"fixed local safety-stop UI missing: {token}")

    stop = section(
        source,
        "void HandEyeCalibrationDialog::RequestSafetyStop()",
        "bool HandEyeCalibrationDialog::LoadConfig()",
    )
    for token in (
        "RobotOperationLease::RequestCancellation",
        "RobotOperationLease::MotionCompletionPending",
        "fanucDriver->Prog_stop_Py()",
        "stepDriver->AbortCurrentProgram()",
        "RobotOperationLease::MarkMotionCompleted(driver)",
        "RobotOperationLease::ConfirmCancellationHandled(driver)",
        "QPointer<HandEyeCalibrationDialog>",
        "std::thread",
        "m_pCaptureConfirmation->reject()",
    ):
        require(token in stop, f"safety-stop contract missing: {token}")
    require(stop.find("if (stopOk)") < stop.find("RobotOperationLease::ConfirmCancellationHandled(driver)"),
            "cancellation latch can be cleared without a successful robot-side stop")

    wait = section(
        source,
        "bool WaitGenericRobotDone(",
        "QString SampleStateText(",
    )
    for token in (
        "STEPROBOTSDK::eRun || lastState == STEPROBOTSDK::ePause",
        "lastState == STEPROBOTSDK::eStop",
        "kHandEyeFanucDoneStartupGuardMs",
        "kHandEyeFanucDoneStableSamples",
        "elapsedMs >= kHandEyeFanucDoneStartupGuardMs",
        "RobotOperationLease::StopAndConfirmUnverifiedMotion(driver)",
        "RobotOperationLease::RequestCancellation(driver)",
        "driver->CheckRobotDone(pollIntervalMs, remainingTimeoutMs)",
        "RobotOperationLease::MotionCompletionPending(driver)",
    ):
        require(token in wait, f"trusted motion completion contract missing: {token}")
    final_pose_read = wait.find("driver->TryGetCurrentPos(finalPose)")
    target_tolerance = wait.find("positionError > kHandEyeAutoArrivePositionToleranceMm")
    completion_witness = wait.find("driver->CheckRobotDone(pollIntervalMs, remainingTimeoutMs)")
    pending_cleared = wait.find("RobotOperationLease::MotionCompletionPending(driver)", completion_witness)
    terminal_success = wait.find("*terminalVerifiedOut = true;")
    require(0 <= final_pose_read < target_tolerance < completion_witness < pending_cleared < terminal_success,
            "final pose/tolerance and the driver witness must pass before pending/terminal success")
    require(wait.find("RobotOperationLease::StopAndConfirmUnverifiedMotion(driver)")
            < wait.find("RobotOperationLease::RequestCancellation(driver)"),
            "an automatic emergency stop failure path is not re-latched for explicit STOP acknowledgement")
    require("RobotOperationLease::MarkMotionCompleted(driver)" not in wait,
            "HandEye wait clears motion pending directly instead of delegating to the driver completion witness")
    require("*terminalVerifiedOut = stopConfirmed" not in wait,
            "an emergency stop is still being reported as natural motion completion")
    require("*terminalVerifiedOut = false;" in wait,
            "unverified completion no longer keeps the HandEye safety-stop target")

    test = section(
        source,
        "bool HandEyeCalibrationDialog::TestHandEyeMatrix()",
        "bool HandEyeCalibrationDialog::StartRobotPoseMove(",
    )
    for token in (
        "m_bRobotTestRunning.exchange(true)",
        "RobotOperationLease::TryAcquire",
        "QPointer<HandEyeCalibrationDialog>",
        "std::thread",
        "CallJobAndWaitStateDone",
        "operationLease->CancellationRequested()",
        "m_pSafetyStopDriver = driver",
        'm_activeRobotTaskOwner = QStringLiteral("手眼矩阵检测")',
        "RobotOperationLease::MotionCompletionPending(driver)",
        "RobotOperationLease::StopAndConfirmUnverifiedMotion(driver)",
        "ApplyRobotTaskTerminalState",
        "operationLease.reset()",
        "outcome.offerMove = false",
    ):
        require(token in test, f"asynchronous hand-eye test contract missing: {token}")
    require("QMessageBox::" not in test,
            "asynchronous hand-eye test can create a modal result dialog")

    move = section(
        source,
        "bool HandEyeCalibrationDialog::StartRobotPoseMove(",
        "bool HandEyeCalibrationDialog::CheckCameraTimestampIntervals()",
    )
    for token in (
        "RobotOperationLease::TryAcquire",
        "std::thread",
        "WaitGenericRobotDone",
        "operationLease->CancellationRequested()",
        "m_pSafetyStopDriver = driver",
        "RobotOperationLease::StopAndConfirmUnverifiedMotion(driver)",
        "sawPreExistingRunning",
        "moveSubmissionAttempted",
        "terminalVerified = !RobotOperationLease::MotionCompletionPending(driver)",
        "ApplyRobotTaskTerminalState",
        "QPointer<HandEyeCalibrationDialog>",
    ):
        require(token in move, f"asynchronous pose move contract missing: {token}")
    require("QMessageBox::" not in move,
            "asynchronous pose move can create a modal result dialog")

    auto = section(
        source,
        "bool HandEyeCalibrationDialog::StartAutoCalibration()",
        "void HandEyeCalibrationDialog::OpenMatrixDialog()",
    )
    confirm = section(auto, "auto confirmCapture", "auto finish")
    for token in (
        "Qt::NonModal",
        "confirmBox->setModal(false)",
        "confirmBox->show()",
        "std::future",
        "operationLease->CancellationRequested()",
        "m_pCaptureConfirmation",
    ):
        require(token in confirm, f"non-modal capture confirmation contract missing: {token}")
    require("confirmBox->open()" not in confirm,
            "QDialog::open can restore modal behavior and hide the local STOP path")
    finish = section(auto, "auto finish", "const auto terminalIsVerified")
    require("QMessageBox::" not in finish,
            "automatic calibration completion can create a modal dialog")
    for token in (
        "m_pSafetyStopDriver = driver",
        'm_activeRobotTaskOwner = QStringLiteral("手眼自动标定")',
        "return !RobotOperationLease::MotionCompletionPending(driver)",
        "ComputeAndSaveMatrix(false)",
    ):
        require(token in auto, f"automatic calibration terminal contract missing: {token}")

    for token in (
        "m_bRobotTestRunning",
        "m_bSafetyStopRunning",
        "m_bSafetyStopRequested",
        "m_pSafetyStopDriver",
        "m_activeRobotTaskOwner",
        "m_pCaptureConfirmation",
        "m_pScrollContent",
        "m_pMoveToLastTestPointBtn",
        "m_pReturnToLastTestPoseBtn",
        "ApplyRobotTaskTerminalState",
    ):
        require(token in header, f"hand-eye lifecycle state missing: {token}")

    close = section(
        source,
        "void HandEyeCalibrationDialog::closeEvent(",
        "void HandEyeCalibrationDialog::RequestSafetyStop()",
    )
    require("event->ignore()" in close and "AppendLog(message)" in close,
            "busy close is not converted into a non-modal status/log rejection")
    require("QMessageBox::" not in close,
            "busy hand-eye close can create a modal dialog that blocks the local STOP")

    busy_ui = section(
        source,
        "void HandEyeCalibrationDialog::RefreshBusyInteractionState()",
        "void HandEyeCalibrationDialog::SetAutoCalibrationStateText(",
    )
    for token in (
        "m_bAutoCalibrationRunning.load()",
        "m_bRobotTestRunning.load()",
        "m_bSafetyStopRunning.load()",
        "m_pSafetyStopDriver != nullptr",
        "m_pScrollContent->setEnabled(!busy)",
    ):
        require(token in busy_ui, f"hand-eye busy interaction freeze missing: {token}")

    print("PASS: hand-eye motion requires final-pose and controller-witness completion before clearing safety state")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
