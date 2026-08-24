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
    header = read("include/FANUCRobotDriver.h")
    driver = read("src/FANUCRobotDriver.cpp")
    service = read("src/MeasureThenWeldService.cpp")
    contract_doc = read("docs/fanuc-actual-weld-contract.md")

    for token in (
        "enum class TrajectoryProgramMode",
        "DryRun",
        "ActualWeld",
        "HasVerifiedArcWeldContract",
    ):
        require(token in header, f"FANUC explicit execution mode contract missing: {token}")

    metadata_detector = section(
        driver,
        "bool FanucContainsWeldMetadata(",
        "long long FanucElapsedMs",
    )
    for token in (
        "info.bWeldProcessEnabled",
        "info.dArcStartCurrent",
        "info.dWeldCurrent",
        "info.dWeldSpeedMmPerMin",
        "info.dArcEndCurrent",
        "!std::isfinite(value)",
    ):
        require(token in metadata_detector,
                f"FANUC weld metadata detector can silently lose a process field: {token}")

    legacy_continuous = section(
        driver,
        "int FANUCRobotCtrl::ContiMoveAny(",
        "bool FANUCRobotCtrl::HasVerifiedArcWeldContract",
    )
    legacy_gate = legacy_continuous.find("FanucContainsWeldMetadata")
    legacy_generation = legacy_continuous.find("FanucMakeTimestamp")
    require(0 <= legacy_gate < legacy_generation,
            "legacy FANUC ContiMoveAny can bypass the weld-contract gate")
    require("实焊不得旁路ArcTool契约" in legacy_continuous,
            "legacy FANUC continuous-move rejection is not explicit")

    upload = section(
        driver,
        "int FANUCRobotCtrl::UploadMultiPointTpProgram(",
        "bool FANUCRobotCtrl::SendWeldTriangleWeaveProgram",
    )
    actual_gate = upload.find("mode == TrajectoryProgramMode::ActualWeld")
    metadata_gate = upload.find("FanucContainsWeldMetadata")
    program_generation = upload.find("FanucMakeTpProgramName")
    upload_call = upload.find("UploadLsFile")
    require(0 <= actual_gate < program_generation < upload_call,
            "FANUC actual-weld rejection is not before LS generation/upload")
    require(0 <= metadata_gate < program_generation,
            "FANUC dry-run weld-metadata rejection is not before LS generation")
    for token in (
        "FANUC_ACTUAL_WELD_CONTRACT_MISSING",
        "FANUC_DRY_RUN_CONTAINS_WELD_METADATA",
        "禁止静默丢弃起弧/收弧/摆动/跟踪字段",
    ):
        require(token in upload, f"FANUC upload fail-closed evidence missing: {token}")

    builder = section(
        driver,
        "std::string FanucBuildTpMoveLsContent(",
        "void FanucLogMovePoint",
    )
    require("info.dOverlapRel <= 0.0" in builder,
            "FANUC lap-step/zero-overlap points are not forced to FINE")
    for forbidden in ("Arc Start[", "Arc End[", "WELD_SPEED", "/APPL"):
        require(forbidden not in builder,
                f"dry-run FANUC builder unexpectedly emits an unverified arc token: {forbidden}")

    execution = section(
        service,
        "bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(",
        "bool MeasureThenWeldService::",
    )
    contract_gate = execution.find("RobotDriverCapability::ActualArcWeld")
    first_motion_gate = execution.find("MoveCoorsAndWait(")
    require(0 <= contract_gate < first_motion_gate,
            "FANUC actual-weld contract is not rejected before the first production motion gate")
    require("RobotTrajectoryPurpose::ActualWeld" in execution
            and "RobotTrajectoryPurpose::WeldDryRun" in execution,
            "production upload does not pass an explicit adaptor trajectory purpose")

    downlink = section(
        service,
        "bool MeasureThenWeldService::DownlinkWeldPoseFile(",
        "bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(",
    )
    require("RobotTrajectoryPurpose::WeldDryRun" in downlink,
            "read-only downlink path is not explicitly dry-run through the adaptor")

    for method, next_method in (
        ("SendWeldTriangleWeaveProgram", "SendWeldLWeaveProgram"),
        ("SendWeldLWeaveProgram", "SendWeldProgram"),
        ("SendWeldProgram", "UploadKlFile"),
    ):
        body = section(
            driver,
            f"bool FANUCRobotCtrl::{method}",
            f"FANUCRobotCtrl::{next_method}",
        )
        require("return false;" in body and "SetLastRobotError" in body,
                f"placeholder FANUC weld API is not fail-closed: {method}")
        require("UploadKlFile(" not in body,
                f"placeholder FANUC weld API can still upload a fake weld program: {method}")

    for token in (
        "当前只支持空跑",
        "旧 `ContiMoveAny` KL/VAR 路径",
        "同一控制器、同一 ArcTool 版本和同一焊机配置",
        "灭弧状态或焊接输出回读",
        "不得把 FANUC 标记为实际焊接能力",
    ):
        require(token in contract_doc, f"FANUC field-unlock gate is undocumented: {token}")

    print("PASS: FANUC actual weld is fail-closed until a field-verified ArcTool contract exists")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
