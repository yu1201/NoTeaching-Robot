#!/usr/bin/env python3
"""Static gate for Reserve -> Downlink -> Start -> Wait trajectory semantics."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def function_body(text: str, signature: str) -> str:
    start = text.find(signature)
    if start < 0:
        raise AssertionError(f"missing function: {signature}")
    opening = text.find("{", start)
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def main() -> None:
    fanuc = read("src/FANUCRobotDriver.cpp")
    step = read("src/StepRobotDriver.cpp")
    service = read("src/MeasureThenWeldService.cpp")

    fanuc_start = function_body(fanuc, "bool FANUCRobotCtrl::StartTrajectory")
    step_start = function_body(step, "bool STEPRobotCtrl::StartTrajectory")
    for brand, body in (("FANUC", fanuc_start), ("STEP", step_start)):
        if "handle.prepared" not in body:
            raise AssertionError(f"{brand} StartTrajectory must require a prepared handle")
        if "DownlinkTrajectory(" in body:
            raise AssertionError(f"{brand} StartTrajectory must not generate or upload implicitly")

    if "ContiMoveAnyWithProgramName(moveInfos, handle.programName, &handle)" not in step_start:
        raise AssertionError("STEP StartTrajectory must bind start to the downlinked handle")

    step_execute = function_body(step, "int STEPRobotCtrl::ContiMoveAnyWithProgramName")
    for needle in (
        "usePreparedUpload",
        "preparedHandle->programContentSha256",
        "内容已偏离Downlink冻结身份",
        "未重复上传",
    ):
        if needle not in step_execute:
            raise AssertionError(f"STEP prepared-start identity gate is missing: {needle}")

    scan = service[service.find("RobotTrajectoryHandle scanTrajectoryHandle") :]
    reserve = scan.find("ReserveTrajectory(")
    downlink = scan.find("DownlinkTrajectory(")
    start = scan.find("StartTrajectory(")
    if not (0 <= reserve < downlink < start):
        raise AssertionError("scan trajectory must follow Reserve -> Downlink -> Start")

    print("ROBOT_TRAJECTORY_LIFECYCLE_OK")


if __name__ == "__main__":
    main()
