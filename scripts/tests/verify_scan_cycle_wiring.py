"""Static safety guard for the three measure-then-weld scan entry points.

This does not simulate a robot.  It prevents the GUI, CLI and ProcessLoop
orchestrations from drifting back to separate safety-position sequences while
hardware-independent driver tests are being introduced.
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
    app = read("src/QtWidgetsApplication4.cpp")
    service = read("src/MeasureThenWeldService.cpp")
    service_header = read("include/MeasureThenWeldService.h")
    camera_cache = read("src/CameraFrameCache.cpp")
    hand_eye = read("src/HandEyeMatrixConfig.cpp")

    gui_flow = section(
        dialog,
        "void MeasureThenWeldDialog::RunPresetParamFlow()",
        "void MeasureThenWeldDialog::RunSkipScanWeldFlow()",
    )
    cli_flow = section(
        app,
        "bool QtWidgetsApplication4::RunMeasureThenWeldScanOnlyRepeatForCli(",
        "ProcessLoopTestDefaults QtWidgetsApplication4::LoadProcessLoopTestDefaults(",
    )
    loop_flow = section(
        app,
        "void QtWidgetsApplication4::RunProcessLoopTest(",
        "void QtWidgetsApplication4::OpenProcessLoopTestPage()",
    )
    calibration_flow = section(
        dialog,
        "void MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow()",
        "void MeasureThenWeldDialog::RefreshWeldModeFromParam()",
    )

    for name, flow in (("GUI", gui_flow), ("CLI", cli_flow), ("ProcessLoop", loop_flow)):
        require(flow.count("RunScanCycle(") == 1, f"{name} must call RunScanCycle exactly once")

    for name, flow in (("CLI", cli_flow), ("ProcessLoop", loop_flow)):
        require("vtStartSafePulse" not in flow, f"{name} must not execute start-safe pulses directly")
        require("vtEndSafePulse" not in flow, f"{name} must not execute end-safe pulses directly")

    runner = section(
        service,
        "bool MeasureThenWeldService::RunScanCycle(",
        "bool MeasureThenWeldService::ScanMoveAndCollect(",
    )
    preset_loader = section(
        service,
        "bool MeasureThenWeldService::LoadPresetParam(",
        "bool MeasureThenWeldService::ReadPulse(",
    )
    manual_safe_loader = section(
        preset_loader,
        "if (!param.bUseComputedScanSafe)",
        "return true;",
    )
    start_safe_move = section(
        service,
        "bool MeasureThenWeldService::MoveScanStartSafeAndWait(",
        "bool MeasureThenWeldService::MoveScanEndSafeAndWait(",
    )
    end_safe_move = section(
        service,
        "bool MeasureThenWeldService::MoveScanEndSafeAndWait(",
        "bool MeasureThenWeldService::RunScanCycle(",
    )
    for required_call in (
        "MoveScanStartSafeAndWait(",
        "MoveCoorsAndWait(",
        "ScanMoveAndCollect(",
        "MoveScanEndSafeAndWait(",
    ):
        require(required_call in runner, f"RunScanCycle missing {required_call}")

    require(
        "param.vtStartSafePulse.empty()" in manual_safe_loader
        and "param.vtEndSafePulse.empty()" in manual_safe_loader,
        "manual scan-safe mode must explicitly detect an empty start/end pulse list",
    )
    require(
        "param.bUseComputedScanSafe = true" not in manual_safe_loader,
        "empty taught safe positions must not silently switch to computed-safe mode",
    )
    require(
        "return false;" in manual_safe_loader
        and "流程已中止" in manual_safe_loader,
        "empty taught safe positions must fail closed with an actionable error",
    )
    for name, safe_move in (("start-safe", start_safe_move), ("end-safe", end_safe_move)):
        require(
            "if (!param.bUseComputedScanSafe)" in safe_move
            and "MovePulseListAndWait(" in safe_move,
            f"{name} helper must route taught-safe mode through the fail-closed pulse-list mover",
        )
        require(
            "自动改用扫描安全位置推算" not in safe_move,
            f"{name} helper must not silently fall back to computed-safe motion",
        )
    require("if (!scanProgress.motionCompleted)" in runner, "runner must gate on confirmed scan completion")
    require("result.safelyRetracted = true" in runner, "runner must record safe retraction")
    require(
        'beforeAction("扫描收枪安全位置")' not in runner,
        "confirmed scan completion must force safe retraction without a cancellable GUI checkpoint",
    )
    require(
        "!param.bUseComputedScanSafe" in runner
        and "param.vtStartSafePulse.empty()" in runner
        and "param.vtEndSafePulse.empty()" in runner
        and "禁止静默改用自动计算安全位" in runner,
        "RunScanCycle must defensively reject empty taught safe positions before motion",
    )
    require("progress->motionStarted = true" in service, "scan motion start must be recorded")
    require("progress->motionCompleted = true" in service, "scan motion completion must be recorded")
    require("QString caseDir;" in service_header, "scan result must expose its case directory")
    require("QString weldPosePath;" in service_header, "scan result must expose its weld pose separately")
    require("scanCycle.caseDir" in cli_flow, "CLI upload must consume the explicit case directory")
    require("scanCycle.caseDir" in loop_flow, "ProcessLoop upload must consume the explicit case directory")
    require("WaitForReadyFrameAfter" in app, "camera startup must wait for a fresh frame")
    require("m_readyCondition.wait_for" in camera_cache, "camera readiness must use a blocking condition")
    require(
        "LoadExistingValidatedHandEyeMatrixConfig" in runner,
        "RunScanCycle must validate the hand-eye matrix before movement",
    )
    require("Calibrated\", 0" in hand_eye, "auto-created hand-eye matrices must be marked uncalibrated")
    require("Calibrated\", 1" in hand_eye, "explicitly saved hand-eye matrices must be marked calibrated")
    require(
        "RunScanCycle(" in calibration_flow
        and "runCalibrationScan(param," in calibration_flow
        and "runCalibrationScan(paramReverse," in calibration_flow,
        "forward and reverse time-offset calibration scans must use the shared safe scan runner",
    )
    require(
        "self->ScanMoveAndCollect(" not in calibration_flow,
        "time-offset calibration must not bypass forced safe retraction",
    )
    require(
        "cameraCacheForRun = self->ResolveCameraCacheForUnit" in calibration_flow,
        "time-offset calibration must refresh its cache after camera startup",
    )
    stale_cache_clear = calibration_flow.find("m_pCameraCache = nullptr;")
    calibration_running = calibration_flow.find("SetRunning(true);")
    require(
        0 <= stale_cache_clear < calibration_running,
        "time-offset calibration must clear a stale cache pointer before SetRunning touches it",
    )
    require(
        "!scanCycle.weldPosePath.isEmpty()" in calibration_flow
        and ": scanCycle.caseDir" in calibration_flow,
        "time-offset calibration must preserve the case-directory fallback for key-point-only results",
    )

    print("PASS: shared scan runner, fail-closed safe positions, forced retraction (including calibration), fresh-frame gate and strict hand-eye wiring are present")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
