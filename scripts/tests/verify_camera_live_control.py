#!/usr/bin/env python3
"""Regression gates for shared SKJ control handle and live image delivery."""

from pathlib import Path


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
    require(finish >= 0, f"missing section end: {end}")
    return text[begin:finish]


def main() -> int:
    worker_h = read("include/groove/scancameraskjworker.h")
    worker = read("src/groove/scancameraskjworker.cpp")
    app = read("src/QtWidgetsApplication4.cpp")
    flow = read("src/MeasureThenWeldDialog.cpp")

    for token in (
        "bool readControlParameters(SKJCameraParameterValues* values",
        "bool setControlParameter(SKJCameraControlClient::Parameter parameter",
        "bool setLaserEnabled(bool enabled",
    ):
        require(token in worker_h, f"worker shared-handle control API missing: {token}")

    for symbol in (
        'resolve("SKJCamera_GetExposure")',
        'resolve("SKJCamera_SetExposure")',
        'resolve("SKJCamera_GetGain")',
        'resolve("SKJCamera_SetGain")',
        'resolve("SKJCamera_GetBinarize")',
        'resolve("SKJCamera_SetBinarize")',
        'resolve("SKJCamera_LaserOn")',
        'resolve("SKJCamera_LaserOff")',
    ):
        require(symbol in worker, f"worker does not resolve control symbol: {symbol}")

    require("read control parameters on shared handle" in worker,
            "parameter reads are not routed through the live SDK handle")
    require('QString("%1 on shared handle ret=%2")' in worker,
            "laser commands are not routed through the live SDK handle")

    preview = section(
        app,
        "void QtWidgetsApplication4::OpenGroovePointCloudDialog()",
        "void QtWidgetsApplication4::CloseGrooveCameraPreviewWindow()",
    )
    require(preview.count("invokeLiveCameraCommand(") == 3,
            "refresh/set/laser must all attempt the existing worker handle")
    for call in (
        "worker->readControlParameters(&values, workerError)",
        "worker->setControlParameter(parameter, value, workerError)",
        "worker->setLaserEnabled(enabled, workerError)",
    ):
        require(call in preview, f"preview control route missing: {call}")

    update = section(
        app,
        "void QtWidgetsApplication4::UpdateGrooveCameraData()",
        "void QtWidgetsApplication4::RobotRunTest()",
    )
    image_update = update.find("SetCameraImage(cache->LatestImage())")
    no_point_cloud = update.find("if (!hasFrame)")
    require(0 <= image_update < no_point_cloud,
            "camera image is still gated by availability of a point-cloud frame")

    preset = section(
        flow,
        "void MeasureThenWeldDialog::RunPresetParamFlow()",
        "void MeasureThenWeldDialog::RunSkipScanWeldFlow()",
    )
    calibration = section(
        flow,
        "void MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow()",
        "void MeasureThenWeldDialog::SetRunning(bool running)",
    )
    for name, block in (("preset", preset), ("calibration", calibration)):
        assignment = block.find("self->m_pCameraCache = cameraCacheForRun;")
        enable = block.find("cameraCacheForRun->SetLiveImageEnabled(true);", assignment)
        success_gate = block.find("ok = cameraCacheForRun != nullptr;", assignment)
        require(0 <= assignment < enable < success_gate,
                f"{name} flow does not enable live images on the post-start cache")

    print("camera live control regression gates passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
