"""Static safety guard for STEP scan pause/resume and numeric scan progress.

This does not talk to a robot.  It locks down the fail-closed wiring that makes
the scan Pause button safe: the exact tracked STEP program is frozen, paused
frames are excluded, motion timeout counts active running time only, and resume
drops the camera backlog.  It also verifies that observable robot-position
progress reaches the GUI through the shared RunScanCycle path.
"""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def section(text: str, start: str, end: str) -> str:
    start_index = text.find(start)
    if start_index < 0:
        raise AssertionError(f"missing section start: {start}")
    end_index = text.find(end, start_index + len(start))
    if end_index < 0:
        raise AssertionError(f"missing section end: {end}")
    return text[start_index:end_index]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    dialog = read("src/MeasureThenWeldDialog.cpp")
    dialog_header = read("include/MeasureThenWeldDialog.h")
    service = read("src/MeasureThenWeldService.cpp")
    service_header = read("include/MeasureThenWeldService.h")
    step = read("src/StepRobotDriver.cpp")
    step_header = read("include/STEPRobotDriver.h")

    tracked_identity = section(
        step,
        "bool STEPRobotCtrl::GetTrackedMotionIdentity(",
        "bool STEPRobotCtrl::PauseTrackedProgramAndWait(",
    )
    require(
        "GetTrackedMotionIdentity" in step_header,
        "STEP driver must expose a read-only tracked-motion identity API",
    )
    for token in (
        "m_sdkCommandMutex",
        "RobotOperationLease::IsCancellationRequested(this)",
        "RobotOperationLease::MotionCompletionPending(this)",
        "currentProject != m_motionTrackedProjectName",
        "currentProgram != m_motionTrackedProgramName",
        "currentState != STEPROBOTSDK::eRun",
        "currentState != STEPROBOTSDK::ePause",
        "currentState != STEPROBOTSDK::eStop",
        "*alreadyStopped = currentState == STEPROBOTSDK::eStop",
        "projectName = m_motionTrackedProjectName",
        "programName = m_motionTrackedProgramName",
    ):
        require(token in tracked_identity, f"tracked STEP identity gate missing: {token}")

    require(
        "enum class PauseControlMode" in dialog_header
        and "Scan," in dialog_header
        and "Weld" in dialog_header
        and "m_scanPauseProgramName" in dialog_header
        and "m_scanMotionPaused" in dialog_header,
        "dialog must keep scan pause state separate from weld breakpoint state",
    )
    pause_handler = section(
        dialog,
        "void MeasureThenWeldDialog::OnPauseResumeClicked()",
        "void MeasureThenWeldDialog::OnResumeWeldClicked()",
    )
    scan_pause = section(
        pause_handler,
        "if (m_pauseControlMode == PauseControlMode::Scan)",
        "WeldResumePlanner::CheckpointRecord activeRecord;",
    )
    for token in (
        "const QString expectedProgram = m_scanPauseProgramName",
        "PauseTrackedProgramAndWait(",
        "ToUtf8StdString(expectedProgram)",
        "QString::fromStdString(pausedProgram) != expectedProgram",
        "ResumeTrackedProgramFromPause(",
        "m_scanMotionPaused = true",
        "m_scanMotionPaused = false",
        'monitor->PauseButton()->setText(QStringLiteral("继续"))',
        'monitor->PauseButton()->setText(QStringLiteral("暂停"))',
        "monitor->ResumeWeldButton()->setEnabled(false)",
    ):
        require(token in scan_pause, f"scan-specific pause/resume gate missing: {token}")
    require(
        "SavePausedBreakpointRecord" not in scan_pause,
        "scan pause must not create a weld-resume checkpoint",
    )

    scan_move = section(
        service,
        "bool MeasureThenWeldService::ScanMoveAndCollect(",
        "bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(",
    )
    require(
        "using ScanProgressCallback = std::function<void(double)>;" in service_header
        and "using ScanPauseAvailabilityCallback = std::function<void(bool, const QString&)>;"
        in service_header,
        "shared scan service must expose progress and pause-availability callbacks",
    )
    for token in (
        "pStepDriver->GetTrackedMotionIdentity(",
        "trackedProject, trackedProgram, &trackedMotionAlreadyStopped",
        "if (!trackedMotionAlreadyStopped)",
        "publishScanPauseAvailability(true, trackedScanProgram)",
        "ScanPauseScopeGuard",
        "publishScanPauseAvailability(false)",
    ):
        require(token in scan_move, f"scan pause identity/lifecycle wiring missing: {token}")

    polling = section(
        scan_move,
        "qint64 activeRunElapsedMs = 0;",
        "const qint64 scanMotionElapsedMs = activeRunElapsedMs;",
    )
    require(
        "const bool isPausedState" in polling
        and "motionState == STEPROBOTSDK::ePause" in polling,
        "STEP ePause must be treated as a distinct resumable scan state",
    )
    require(
        "if (motionStarted && !scanPaused)" in polling
        and "activeRunElapsedMs += budgetDeltaMs" in polling
        and "if (motionStarted && activeRunElapsedMs > scanFinishTimeoutMs)" in polling,
        "scan completion timeout must count active running time instead of wall time",
    )
    require(
        "SteadyNowMs() - motionStartMs > scanFinishTimeoutMs" not in polling,
        "paused wall-clock time must not consume the scan motion timeout",
    )
    paused_branch = section(
        polling,
        "if (isPausedState)",
        "else if (scanPaused)",
    )
    require(
        "frameCache->SetImageCaptureDir(QString())" in paused_branch
        and "lastPulledCameraSequence = frameCache->Mark()" in paused_branch
        and "appendRobotPose()" not in paused_branch
        and "pullScanCameraFrames()" not in paused_branch,
        "pause state must disable image capture and avoid pose/camera ingestion",
    )
    require(
        "RobotMotionTimeoutPolicy::kMotionTimeoutMs" in paused_branch
        and "暂停超过 30 分钟安全上限" in paused_branch,
        "scan pause must retain a finite fail-closed safety timeout",
    )
    resumed_branch = section(
        polling,
        "else if (scanPaused)",
        "if (isRunningState)",
    )
    require(
        "lastPulledCameraSequence = frameCache->Mark()" not in resumed_branch
        and "frameCache->SetImageCaptureDir(imageCaptureTmpDir, imageCaptureStride)"
        in resumed_branch,
        "paused polling must own the discard watermark so resume keeps START-after frames",
    )
    running_collection = section(
        polling,
        "if (motionStarted && !isPausedState)",
        "if (motionStarted && isDoneState)",
    )
    for token in ("appendRobotPose()", "reportScanProgress()", "pullScanCameraFrames()"):
        require(token in running_collection, f"active scan collection missing: {token}")

    progress_reporter = section(
        scan_move,
        "auto reportScanProgress =",
        "auto pullScanCameraFramesTo =",
    )
    for token in (
        "param.tEndPos.dX - param.tStartPos.dX",
        "currentPose.dX - param.tStartPos.dX",
        "std::clamp(",
        "lastReportedScanPercent",
        "percent > lastReportedScanPercent",
        "scanProgressCallback(",
    ):
        require(token in progress_reporter, f"observable scan-position progress missing: {token}")

    run_cycle = section(
        service,
        "bool MeasureThenWeldService::RunScanCycle(",
        "bool MeasureThenWeldService::ScanMoveAndCollect(",
    )
    require(
        "scanProgressCallback" in run_cycle
        and "scanPauseAvailability" in run_cycle
        and "ScanMoveAndCollect(" in run_cycle,
        "RunScanCycle must forward progress and pause callbacks to ScanMoveAndCollect",
    )
    preset_flow = section(
        dialog,
        "void MeasureThenWeldDialog::RunPresetParamFlow()",
        "void MeasureThenWeldDialog::RunSkipScanWeldFlow()",
    )
    for token in (
        "RunScanCycle(",
        "[self](double ratio)",
        "std::clamp(ratio, 0.0, 1.0) * 15.0",
        "self->SetProgressBusy(",
        "[self](bool available, const QString& programName)",
        "self->SetScanPauseAvailable(available, programName)",
    ):
        require(token in preset_flow, f"preset RunScanCycle UI wiring missing: {token}")

    print(
        "PASS: STEP scan pause binds tracked identity, excludes paused samples, "
        "uses active runtime, drops resume backlog, and reports position progress to the UI"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
