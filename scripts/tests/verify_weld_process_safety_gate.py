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
    require(finish >= 0, f"missing section end after {start}: {end}")
    return text[begin:finish]


def main() -> int:
    storage = read("src/WeldProcessFile.cpp")
    service = read("src/MeasureThenWeldService.cpp")
    step = read("src/STEPRobotDriver.cpp")
    contract = read("include/WeldProcessValidation.h")

    for token in (
        "ValidateStoredWeldProcess",
        "ValidateActualWeldProcess",
        "ValidateActualMoveInfos",
        "std::isfinite(value)",
        '"WeldVelocity"',
        '"TrackCurrent"',
        '"TrackVoltage"',
    ):
        require(token in contract, f"shared weld safety contract missing: {token}")

    for token in (
        "ValidateWeave(tWeaveDate",
        "ValidateStoredWeldProcess(tWeldPara",
        "ValidateStoredWeldProcess(item",
        "Present-but-invalid primary data must never be hidden",
        "拒绝写入非法焊接工艺",
    ):
        require(token in storage, f"WeldProcessFile load/write gate missing: {token}")

    execute = section(
        service,
        "bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(",
        "bool MeasureThenWeldService::",
    )
    actual_gate = execute.find("weldProcessSafetyError")
    move_build = execute.find("BuildWeldPoseMoveInfos(")
    first_motion = execute.find("if (safetySettings.safetyGateMotionPrecheckEnabled")
    require("&& executionPreMotion" in execute,
            "motion precheck callback is not controlled by its safety-gate switch")
    require(0 <= actual_gate < move_build < first_motion,
            "actual-weld process gate is not before move construction and first motion")

    serializer = section(
        step,
        "std::string StepBuildSrdContent(",
        "std::string StepBuildSrpContent(",
    )
    require("if (actualWeld)" in serializer
            and "ValidateActualMoveInfos" in serializer
            and "return std::string();" in serializer,
            "STEP SRD serializer does not fail closed for invalid actual-weld metadata")
    require("StepFiniteOrDefault(value, 0.0)" in step,
            "test expects legacy dry-run formatter behavior to remain explicit")

    print("PASS: stored, execution-entry, and STEP serialization weld safety gates are wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
