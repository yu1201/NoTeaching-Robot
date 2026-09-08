#!/usr/bin/env python3
"""Static contract gate for the FANUC RobotDriverAdaptor implementation."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def main() -> None:
    driver = read("src/FANUCRobotDriver.cpp")
    service = read("SDK/FANUC/FanucServiceLib.kl")
    contract = read("docs/robot-driver-adaptor-contract.md")
    measure_then_weld = read("src/MeasureThenWeldService.cpp")

    capabilities = function_body(
        driver, "std::uint64_t FANUCRobotCtrl::DriverCapabilities() const")
    for capability in (
        "PassiveState",
        "LinearMotion",
        "JointMotion",
        "ContinuousTrajectory",
        "ContinuousJog",
        "PauseResume",
        "NativeProgramUpload",
        "DiagnosticCommand",
        "VerifiedProgramCompletion",
        "VerifiedSafeAbort",
        "HandEyeProgramSupport",
        "OfflineTrajectoryExport",
        "ConnectionControl",
        "ToolDataRead",
        "IntegerRegister",
        "NativeProgramExecution",
        "FtpFileTransfer",
        "HandEyeMatrixRead",
        "HandEyeSupportProgramInstall",
    ):
        require(
            f"RobotDriverCapability::{capability}" in capabilities,
            f"FANUC verified capability missing: {capability}",
        )

    for unsupported in (
        "ActualArcWeld",
        "ExternalAxis",
        "OperationModeControl",
        "AlarmReset",
        "ServoPowerControl",
    ):
        require(
            f"RobotDriverCapability::{unsupported}" not in capabilities,
            f"FANUC must not claim unverified capability: {unsupported}",
        )

    exporter = function_body(
        driver, "bool FANUCRobotCtrl::ExportTrajectoryProgramFiles(")
    for token in (
        "HasVerifiedArcWeldContract",
        "FanucContainsWeldMetadata",
        "FanucContainsExternalAxisTarget",
        "FanucCanonicalTrajectoryToNative",
        "FanucIsValidControllerProgramName",
        "FanucBuildTpMoveLsContent",
        "FanucWriteTextFile",
        "FanucCompileLsToTp",
        "handle.localProgramPath",
        "handle.localDataPath",
        "handle.prepared = true",
    ):
        require(token in exporter, f"FANUC offline export evidence missing: {token}")
    require(
        exporter.index("HasVerifiedArcWeldContract")
        < exporter.index("FanucWriteTextFile"),
        "FANUC actual-weld gate must run before offline files are written",
    )
    for forbidden in ("UploadFile(", "UploadLsFile(", "m_pFTP", "remoteTpPath"):
        require(
            forbidden not in exporter,
            f"FANUC offline export must not contact the controller: {forbidden}",
        )

    downlink = function_body(
        driver, "int FANUCRobotCtrl::UploadMultiPointTpProgram(")
    require(
        "FanucContainsExternalAxisTarget" in downlink,
        "FANUC generated TP must reject external-axis targets instead of dropping them",
    )
    require(
        "FanucIsValidControllerProgramName" in downlink,
        "FANUC generated TP must validate its controller program identity",
    )

    for signature, command in (
        ("bool FANUCRobotCtrl::ServoOn()", "SERVO_ON"),
        ("bool FANUCRobotCtrl::ServoOff()", "SERVO_OFF"),
        ("bool FANUCRobotCtrl::cleanAlarm()", "CLEAR_ALARM"),
        ("bool FANUCRobotCtrl::SetSysMode(int mode)", "SET_SYS_MODE"),
    ):
        body = function_body(driver, signature)
        require("return false;" in body, f"FANUC placeholder must fail closed: {signature}")
        require(
            "FanucRequest(" not in body,
            f"FANUC placeholder still accepts a fake service OK: {signature}",
        )
        require(
            f"WriteErr(ch, '{command}_UNSUPPORTED')" in service,
            f"FANUC service must report unsupported instead of OK: {command}",
        )

    pause = function_body(driver, "bool FANUCRobotCtrl::PauseTrackedMotion(")
    resume = function_body(driver, "bool FANUCRobotCtrl::ResumeTrackedMotion(")
    for token in (
        "PAUSE_TASK:",
        "GET_ACTIVE_TASK",
        "FanucPositionDeviationMm",
        "FanucAngleDeviationDeg",
        "afterPause.status != 1",
        "afterPause.line != reportedLine",
    ):
        require(token in pause, f"FANUC verified pause evidence missing: {token}")
    for token in (
        "RESUME_TASK:",
        "GET_ACTIVE_TASK",
        "secondSnapshot.status != 1",
        "secondSnapshot.line != firstSnapshot.line",
        "positionDeviation > maxPositionDeviationMm",
        "angleDeviation > maxAngleDeviationDeg",
    ):
        require(token in resume, f"FANUC verified resume evidence missing: {token}")

    tool = function_body(driver, "bool FANUCRobotCtrl::GetToolData(")
    require("TOOL_DATA_V1" in tool and "GET_TOOL_DATA:" in tool,
            "FANUC tool data is not bound to the verified service feature")
    require("parts.size()" not in tool or "size() != 6" in tool,
            "FANUC tool response must contain exactly six values")

    hand_eye = function_body(driver, "bool FANUCRobotCtrl::GetHandEyeMatrixVariable(")
    for token in (
        "HAND_EYE_R100_V1",
        "GET_HE_MATRIX:eye",
        "size() != 12",
        "FanucRotationMatrixIsValid",
    ):
        require(token in hand_eye, f"FANUC hand-eye matrix evidence missing: {token}")

    for token in (
        "PAUSE_TASK(task_name, TRUE, TRUE, pause_status)",
        "CONT_TASK(task_name, continue_status)",
        "TSK_LINENUM",
        "$MNUTOOL[1,",
        "$MNUTOOLNUM[1]",
        "GET_HE_MATRIX:",
        "reg_no = 99 + idx",
        "LIB=20260825_ADAPTOR_V3",
    ):
        require(token in service, f"FANUC resident service evidence missing: {token}")

    for fake_command in (
        "SET_SPEED:",
        "LOAD_USER_PROGRAM:",
        "UNLOAD_USER_PROGRAM",
        "PROGRAM_START",
        "AXIS_PULSE_MOVE:",
        "POS_MOVE:",
        "MOVE_BY_JOB_AXIS:",
        "MOVE_BY_JOB_POS:",
        "MOVE_BY_JOB_VECTOR:",
    ):
        command_index = service.index(fake_command)
        command_window = service[command_index : command_index + 180]
        require("WriteOk(ch, '')" not in command_window,
                f"FANUC service still returns fake OK: {fake_command}")

    for signature in (
        "bool FANUCRobotCtrl::MoveLinearMmPerMin(",
        "bool FANUCRobotCtrl::MoveJointPercent(",
    ):
        body = function_body(driver, signature)
        require(
            "externalAxleType != 0" in body and "return false;" in body,
            f"FANUC direct motion must reject configured external axes: {signature}",
        )

    for token in (
        "`OfflineTrajectoryExport`",
        "`ExternalAxis`",
        "`OperationModeControl`",
        "`AlarmReset`",
        "`ServoPowerControl`",
        "`ToolDataRead`",
        "R[100]~R[111]",
        "不能用通信成功代替功能完成",
    ):
        require(token in contract, f"FANUC capability boundary is undocumented: {token}")

    offline_business = function_body(
        measure_then_weld,
        "bool MeasureThenWeldService::GenerateRobotWeldProgramFiles(",
    )
    require(
        "actualWeld && !pRobotDriver->Supports(RobotDriverCapability::ActualArcWeld)"
        in offline_business,
        "business offline export must reject actual weld before calling a dry-run-only brand",
    )
    require(
        "pRobotDriver->ExternalAxleType() != 0" in offline_business
        and "RobotDriverCapability::ExternalAxis" in offline_business,
        "business offline export must gate a configured external axis",
    )
    require(
        "transitionCommandSpeed,\n        actualWeld," in offline_business,
        "dry-run offline export must not inject weld-process metadata",
    )

    print(
        "PASS: FANUC adaptor verifies pause/tool/hand-eye services and keeps unsafe features fail-closed"
    )


if __name__ == "__main__":
    main()
