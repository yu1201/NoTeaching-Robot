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
    for required_call in (
        "MoveScanStartSafeAndWait(",
        "MoveCoorsAndWait(",
        "ScanMoveAndCollect(",
        "MoveScanEndSafeAndWait(",
    ):
        require(required_call in runner, f"RunScanCycle missing {required_call}")

    require("if (pulses.empty())" in service, "empty pulse lists must fail closed")
    require("if (!scanProgress.motionCompleted)" in runner, "runner must gate on confirmed scan completion")
    require("result.safelyRetracted = true" in runner, "runner must record safe retraction")
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

    print("PASS: shared scan runner, fresh-frame gate and strict hand-eye wiring are present")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
