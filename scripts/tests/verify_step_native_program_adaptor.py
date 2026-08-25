#!/usr/bin/env python3
"""Static regression gate for the STEP native-program adaptor integration."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(text: str, needle: str, message: str) -> None:
    if needle not in text:
        raise AssertionError(f"{message}: missing {needle!r}")


def reject(text: str, needle: str, message: str) -> None:
    if needle in text:
        raise AssertionError(f"{message}: found {needle!r}")


def function_body(text: str, signature: str) -> str:
    start = text.find(signature)
    if start < 0:
        raise AssertionError(f"missing function signature: {signature}")
    opening = text.find("{", start)
    if opening < 0:
        raise AssertionError(f"missing function opening brace: {signature}")
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
    adaptor_h = read("include/RobotDriverAdaptor.h")
    adaptor_cpp = read("src/RobotDriverAdaptor.cpp")
    step = read("src/StepRobotDriver.cpp")
    fanuc = read("src/FANUCRobotDriver.cpp")
    function_test = read("src/FunctionTestDialog.cpp")
    hand_eye = read("src/HandEyeCalibrationDialog.cpp")
    app = read("src/QtWidgetsApplication4.cpp")

    require(adaptor_h, "HandEyeSupportProgramInstall = 1ULL << 27",
            "support-program installation must have an independent capability")
    require(adaptor_cpp, 'return "手眼辅助程序安装"',
            "the new capability needs a user-facing restriction message")

    capabilities = function_body(step, "STEPRobotCtrl::DriverCapabilities")
    for capability in (
        "NativeProgramUpload",
        "DiagnosticCommand",
        "NativeProgramExecution",
        "HandEyeSupportProgramInstall",
    ):
        require(capabilities, f"RobotDriverCapability::{capability}",
                f"STEP must declare {capability}")
    reject(capabilities, "RobotDriverCapability::HandEyeProgramSupport",
           "STEP must not claim robot-side hand-eye result validation")

    prepare = function_body(step, "STEPRobotCtrl::PrepareNativeProgramUpload")
    require(prepare, "CreateFileTransferSession", "STEP upload preparation must validate the FTP bottom")

    upload = function_body(step, "STEPRobotCtrl::UploadNativeProgramSource")
    for needle in (
        'extension != ".srp"',
        'extension != ".srd"',
        "StepResolveNativeRemoteDirectory",
        "CreateFileTransferSession",
        "UploadProgramFile",
    ):
        require(upload, needle, "STEP native upload must route through the FTP session")
    reject(upload, "m_pFTP", "the adaptor implementation must not merge with the raw FTP client")

    diagnostic = function_body(step, "STEPRobotCtrl::SendDiagnosticCommand")
    for needle in (
        'normalizedCommand == "GET_USER_PROGRAM"',
        'normalizedCommand == "GET_CUR_POS"',
        'normalizedCommand == "GET_CUR_PULSE"',
        "m_pSTEPRobotClient->test(trimmedCommand.c_str())",
    ):
        require(diagnostic, needle, "STEP diagnostic mapping or native passthrough is incomplete")

    execute = function_body(step, "STEPRobotCtrl::RunProgramAndWait")
    for needle in (
        "StepResolveNativeProgramIdentity",
        "LoadUserProgram(requestedProject, requestedProgram, true)",
        "RobotOperationLease::MarkMotionStarted",
        "SetModeCmd(MODEKEY::START, true)",
        "snapshot.project != requestedProject",
        "snapshot.program != requestedProgram",
        "STEPROBOTSDK::eError",
        "stableStoppedReads >= 3",
        "RobotOperationLease::MarkMotionCompleted",
        "RobotOperationLease::StopAndConfirmUnverifiedMotion",
    ):
        require(execute, needle, "STEP native execution safety/completion loop is incomplete")
    reject(execute, "Prog_startRun_Py", "native execution must not borrow the generated-program ntdone start path")
    reject(execute, "CheckRobotDone", "native execution must not require the generated-program ntdone witness")

    install = function_body(step, "STEPRobotCtrl::InstallHandEyeSupportPrograms")
    for needle in (
        '"Job/STEP/handeyetest.srp"',
        '"Job/STEP/handeyetest.srd"',
        "UploadNativeProgramSource(programBytes.constData())",
        "UploadNativeProgramSource(dataBytes.constData())",
    ):
        require(install, needle, "STEP hand-eye support program installation is incomplete")

    require(fanuc, "RobotDriverCapability::HandEyeSupportProgramInstall",
            "FANUC must preserve its support-program installation entry")
    if hand_eye.count("RobotDriverCapability::HandEyeSupportProgramInstall") < 2:
        raise AssertionError("both hand-eye installation entries must use the installation capability")
    require(hand_eye, "driver->Supports(RobotDriverCapability::HandEyeProgramSupport)",
            "robot-side hand-eye result comparison must keep its independent gate")

    upload_test = function_body(function_test, "void FunctionTestDialog::FanucUploadLsTest()")
    require(upload_test, "driver->FileTransferProfile()",
            "the generic upload test must use the current driver file profile")
    require(upload_test, "QFileDialog::getOpenFileName",
            "the generic upload test must let the operator select a native file")
    reject(upload_test, "SDK/FANUC/STARTALL.ls",
           "the generic upload test must not inject a FANUC file into STEP")
    diagnostic_test = function_body(
        function_test, "void FunctionTestDialog::FanucCurposDiagnosticTest()"
    )
    for needle in (
        "driver->DriverDescriptor()",
        "driver->TryGetCurrentPos(pose)",
        "driver->TryGetCurrentPulse(pulse)",
        "driver->ReadMotionStatus()",
    ):
        require(diagnostic_test, needle,
                "the generic diagnostic set must use adaptor functions across brands")
    reject(diagnostic_test, '"GET_CUR_PULSE"',
           "generic diagnostics must not send a FANUC/STEP raw command to another brand")
    reject(function_test, 'CreateTestButton("发送FANUC LS")',
           "the test UI must not present the adaptor entry as FANUC-only")

    require(app, "DriverDescriptor().family != RobotDriverFamily::Fanuc",
            "legacy --fanuc-* CLI commands must reject non-FANUC drivers")
    reject(app, '{ "uploadLs", "发送FANUC LS" }',
           "the dashboard native-program entry must be brand-neutral")

    print("STEP_NATIVE_PROGRAM_ADAPTOR_OK")


if __name__ == "__main__":
    main()
