#!/usr/bin/env python3
"""Static regression gate for STEP/FANUC generated-program isolation."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(text: str, needle: str, message: str) -> None:
    if needle not in text:
        raise AssertionError(f"{message}: missing {needle!r}")


def forbid(text: str, needle: str, message: str) -> None:
    if needle in text:
        raise AssertionError(f"{message}: forbidden {needle!r}")


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


def verify_step(step: str) -> None:
    require(step, "kStepMaxProgramNameLength = 31", "STEP name must fit SDK char[32]")
    require(step, "g_stepGeneratedNameSequence.fetch_add", "STEP names need an atomic sequence")
    require(step, "StepProcessNonce()", "STEP names need a process nonce")
    require(step, "StepControllerIdentity", "STEP names need controller identity")
    for identity_part in (
        "ctrl->m_sSocketIP",
        "ctrl->m_nSocketPort",
        "ctrl->m_sFTPIP",
        "ctrl->m_nFTPPort",
        "GetCurrentProcessId()",
        "reinterpret_cast<std::uintptr_t>(ctrl)",
    ):
        require(step, identity_part, "STEP identity is incomplete")

    sanitize = function_body(step, "std::string StepSanitizeProgramName")
    require(sanitize, "std::isalnum", "STEP controller names must be alphanumeric")
    require(sanitize, "ch != '_'", "STEP controller names may retain underscore")
    require(sanitize, "std::isalpha", "STEP controller names must start with a letter")
    require(sanitize, "resize(kStepMaxProgramNameLength)", "STEP names must be length bounded")

    timestamp_name = function_body(
        step, "std::string STEPRobotCtrl::MakeTimestampWeldProgramName"
    )
    require(timestamp_name, "StepMakeUniqueToken", "weld names must be strongly unique")
    require(timestamp_name, '"Weld_"', "weld names must preserve the established prefix")
    require(timestamp_name, '"%y%m%d%H"', "weld names should retain readable time")

    conti = function_body(step, "int STEPRobotCtrl::ContiMoveAny(")
    require(conti, "StepMakeProgramName(this)", "automatic STEP name must include instance identity")
    init_socket = function_body(step, "bool STEPRobotCtrl::InitSocket")
    require(init_socket, "m_sSocketIP = connectedIp", "STEP runtime endpoint must update isolation identity")
    require(init_socket, "m_nSocketPort", "STEP runtime port must update isolation identity")
    conti_named = function_body(step, "int STEPRobotCtrl::ContiMoveAnyWithProgramName")
    require(
        conti_named,
        "StepInstanceOutputDirectory(this)",
        "STEP local files must use endpoint/instance directory",
    )
    forbid(
        conti_named,
        "StepResolveOutputDirectory(std::string())",
        "STEP runtime files must not share the legacy flat directory",
    )

    atomic_write = function_body(step, "bool StepAtomicReplaceTextFile")
    for primitive in (
        "CREATE_NEW",
        "FILE_ATTRIBUTE_TEMPORARY",
        "FlushFileBuffers",
        "MoveFileExW",
        "MOVEFILE_REPLACE_EXISTING",
        "MOVEFILE_WRITE_THROUGH",
        "DeleteFileW",
    ):
        require(atomic_write, primitive, "STEP file replacement is not atomic/durable")
    if step.count("g_stepGeneratedFilePairMutex") < 3:
        raise AssertionError("STEP SRP/SRD generation is not mutually excluded at both entry points")


def verify_fanuc(fanuc: str, header: str) -> None:
    require(fanuc, "FANUC_MAX_GENERATED_PROGRAM_NAME = 31", "FANUC names need a hard bound")
    require(fanuc, "g_fanucGeneratedNameSequence.fetch_add", "FANUC names need atomic sequence")
    require(fanuc, "FanucControllerIdentity", "FANUC names need controller identity")
    require(fanuc, 'FanucMakeControllerProgramName(ctrl, "FK")', "KL name prefix must be safe")
    require(fanuc, 'FanucMakeControllerProgramName(ctrl, "FT")', "TP name prefix must be safe")
    forbid(fanuc, 'GetStr("FM%02d%02d%02d%02d"', "minute-granularity FANUC name can collide")
    forbid(fanuc, 'GetStr("FM%02d%02d%02d"', "second-granularity FANUC name can collide")

    for caller in (
        "const std::string programName = FanucMakeProgramName(this);",
        "const std::string programName = FanucMakeTpProgramName(this);",
    ):
        require(fanuc, caller, "FANUC generated name must include driver identity")
    if fanuc.count("FanucGeneratedProgramDirectory(this)") < 3:
        raise AssertionError("FANUC KL/LS/JOGBUF files are not all instance isolated")

    atomic_write = function_body(fanuc, "bool FanucWriteTextFile")
    for primitive in (
        "CREATE_NEW",
        "FlushFileBuffers",
        "MoveFileExW",
        "MOVEFILE_REPLACE_EXISTING",
        "MOVEFILE_WRITE_THROUGH",
        "DeleteFileW",
    ):
        require(atomic_write, primitive, "FANUC file replacement is not atomic/durable")
    require(fanuc, "g_fanucGeneratedFilePairMutex", "FANUC KL/VAR pair needs exclusion")
    require(fanuc, "g_fanucCompilerMutex", "WinOLPC compiler needs process exclusion")

    for member in (
        "m_fixedMoveExecutionMutex",
        "m_fixedMoveUploadMutex",
        "m_fixedMoveUploadEndpointKey",
        "m_fixedMovlUploaded",
        "m_fixedMovjUploaded",
        "m_fixedMovlLocalIdentity",
        "m_fixedMovjLocalIdentity",
    ):
        require(header, member, "FANUC fixed TP cache must be per instance")
    forbid(fanuc, "static bool movjUploaded", "MOVJ cache must not be process-global")
    forbid(fanuc, "static bool movlUploaded", "MOVL cache must not be process-global")

    ensure = function_body(fanuc, "bool FANUCRobotCtrl::EnsureFixedMoveTpUploaded")
    require(ensure, "FanucControllerIdentity(this, false)", "cache must be endpoint scoped")
    require(ensure, "FtpClient fixedMoveFtp", "fixed TP cache needs an endpoint-local FTP session")
    require(ensure, 'listFiles("/md/"', "cached TP must be checked on the controller")
    require(ensure, "FanucReadFileIdentity", "fixed TP cache must notice local binary changes")
    require(ensure, "file.size == localIdentity.size", "remote TP size must match the local binary")
    require(ensure, "fixedMoveFtp.downloadFile", "remote TP bytes must be downloaded before reuse")
    require(ensure, "Matches(localIdentity, remoteIdentity)", "remote TP needs SHA-256 content identity")
    require(ensure, "fixedMoveFtp.uploadFile", "missing TP must be recoverably re-uploaded")
    if ensure.count("remoteFileMatchesContent()") < 2:
        raise AssertionError("fixed TP must be verified before reuse and after upload")

    create_run = function_body(fanuc, "bool FANUCRobotCtrl::CreateUploadRunTpMove")
    require(create_run, "m_fixedMoveExecutionMutex", "fixed TP motion must not interleave")
    require(create_run, "FanucFixedEndpointTransferMutex", "same-endpoint verify and CALL must serialize")
    require(create_run, "endpointVerifiedCallLock", "remote content proof is not held through CALL_JOB")
    require(create_run, "EnsureFixedMoveTpUploaded", "fixed TP motion must use scoped cache")
    if not (create_run.index("endpointVerifiedCallLock")
            < create_run.index("EnsureFixedMoveTpUploaded")
            < create_run.index("CallJobWithCompletionState")):
        raise AssertionError("same-endpoint mutex does not cover verify -> CALL_JOB TOCTOU window")
    require(create_run, "InvalidateFixedMoveUploadCache", "failed start must invalidate cache")
    require(
        create_run,
        "Do not auto-retry CALL_JOB",
        "ambiguous motion start must not be retried automatically",
    )

    init_socket = function_body(fanuc, "bool FANUCRobotCtrl::InitSocket")
    close_socket = function_body(fanuc, "bool FANUCRobotCtrl::CloseSocket")
    require(init_socket, "InvalidateFixedMoveUploadCache", "reconnect must invalidate cache")
    require(close_socket, "InvalidateFixedMoveUploadCache", "disconnect must invalidate cache")


def main() -> int:
    step = read("src/StepRobotDriver.cpp")
    fanuc = read("src/FANUCRobotDriver.cpp")
    header = read("include/FANUCRobotDriver.h")
    verify_step(step)
    verify_fanuc(fanuc, header)
    print("PASS: STEP/FANUC generated-program names, local files, and fixed TP cache are isolated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
