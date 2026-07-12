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
    dialog = read("src/MeasureThenWeldDialog.cpp")
    service = read("src/MeasureThenWeldService.cpp")
    service_h = read("include/MeasureThenWeldService.h")
    step = read("src/StepRobotDriver.cpp")
    planner = read("src/WeldResumePlanner.cpp")
    project = read("QtWidgetsApplication4.vcxproj")
    process_loop = read("src/QtWidgetsApplication4.cpp")

    resume = section(
        dialog,
        "void MeasureThenWeldDialog::RunResumeWeldFlow()",
        "void MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow()",
    )
    for forbidden in (
        "lastModified()",
        "kBacktrackPoints",
        "latestPosePath",
        "parts[0].toDouble()",
        'ReadString(false, "Valid"',
    ):
        require(forbidden not in resume, f"legacy fail-open resume logic remains: {forbidden}")
    for token in (
        "ReadBreakpointRecord",
        'checkpointRecord.state != QStringLiteral("paused")',
        "ValidateCheckpointTime",
        "QDateTime::currentDateTimeUtc()",
        "PersistentEndpointIdentity",
        "ResolveWeldExecutionParameters",
        "resumeCheckpointSupported",
        "parameterFingerprint",
        "ResolveBoundTrajectory",
        "sourceTrajectorySha256",
        "PlanFromPausedPoseBound",
        "resumePlan.resumeArcMm",
        "取得硬件租约后机器人端点或绑定轨迹发生变化",
        "续焊运动/START前机器人名称、类型或端点身份发生变化",
        '&pausedRecoveryBinding,\n            QStringLiteral("paused"),\n            QStringLiteral("resuming")',
        "inputAlreadyInExecutionOrder",
    ):
        if token == "inputAlreadyInExecutionOrder":
            require("resumePlan.resumeArcMm,\n                    true," in resume,
                    "V2 trajectory is not explicitly marked as execution-order input")
        else:
            require(token in resume, f"bound resume gate missing: {token}")
    require("WeldResumePlanner::PlanFromPausedPose(" not in resume,
            "production resume still plans from an unbound trajectory snapshot")

    pause = section(
        dialog,
        "void MeasureThenWeldDialog::OnPauseResumeClicked()",
        "void MeasureThenWeldDialog::OnResumeWeldClicked()",
    )
    for token in (
        "ActiveWeldCheckpointRecord",
        "PauseTrackedProgramAndWait",
        "activeRecord.programName",
        "SavePausedBreakpointRecord",
        "m_activeWeldCheckpointRecord = pausedEncoded",
        'QStringLiteral("paused"),\n            QStringLiteral("continuing")',
        "m_activeWeldCheckpointRecord = continuingEncoded",
        "ResumeTrackedProgramFromPause",
    ):
        require(token in pause, f"stable pause checkpoint gate missing: {token}")
    require("Prog_stop_Py()" not in pause,
            "dialog still snapshots immediately after a bare STOP command")
    require("LatestStateSnapshot" not in pause,
            "continue gate still fails open on a missing or stale cached pose")

    direction = service.find("ApplyWeldDirectionToExecutionRecords(weldPosePreset, records);")
    clip = service.find("ClipWeldPoseRecordsAtArcLength(records, resumeStartArcMm")
    require(0 <= direction < clip, "resume clipping occurs before execution direction is applied")
    for token in (
        "preserveInputRecords",
        "inputAlreadyInExecutionOrder))",
        "poseFilePath, inputAlreadyInExecutionOrder",
        "ResolveEffectiveFinalStepMm",
        "BuildEffectiveWeldExecutionFingerprint",
        "executionIdentity.parameterFingerprint",
        "executionIdentity.resumeCheckpointSupported",
        "weldPosePreset.weaveAppPointwise",
        "weldPosePreset.weaveEnabled || weldPosePreset.trackEnabled",
        "ExecutionContextGuard",
        "InvalidateStoredWeldResumeCheckpoint",
        "if (!inputAlreadyInExecutionOrder)",
        "resumeMode != inputAlreadyInExecutionOrder",
        "resumeMode && !executionPrepared",
        "expectedSourceSha256",
        "ComputeFileSha256ForResumeGate",
        "V2续焊实际解析轨迹与绑定的预期 SHA256 不一致",
        "executionPreMotion",
        "机器人运动前复核焊接执行身份失败",
        "WeldSafetyRecoveryStore::InvalidateIfNoPending",
        'stopBeforeNextWeldAction("启动STEP焊接轨迹程序")',
        "PersistProgramCompletedUnretracted",
        "FinishSafelyRetracted",
    ):
        require(token in service, f"service execution identity/lifecycle gate missing: {token}")
    require("WeldExecutionFinishedCallback" in service_h,
            "service cannot close the pause context immediately after weld completion")
    require("qualityProofPosePath" in service_h
            and "identity.qualityProofPosePath" in dialog,
            "repeat resume loses the original authorized SeamComp proof source")
    require('superseded.state = QStringLiteral("superseded")' in dialog,
            "starting a new full weld does not tombstone an older paused checkpoint")
    preset_flow = section(dialog, "void MeasureThenWeldDialog::RunPresetParamFlow()",
                          "void MeasureThenWeldDialog::RunSkipScanWeldFlow()")
    skip_flow = section(dialog, "void MeasureThenWeldDialog::RunSkipScanWeldFlow()",
                        "void MeasureThenWeldDialog::RunLineScanProcess()")
    require("InvalidateStoredWeldResumeCheckpoint" in preset_flow,
            "preset flow can move the robot before invalidating an old checkpoint")
    require("InvalidateStoredWeldResumeCheckpoint" in skip_flow,
            "skip-scan flow can start a new weld identity before invalidating an old checkpoint")
    require("if (!resumeCheckpointSupported && isResumeFlow)" in dialog,
            "pointwise centerline can still enter automatic resume")
    require('resumeStartArcMm < 0.0 && resumeStartArcMm != -1.0' in service,
            "non-finite/invalid resume arc can fall through to full-trajectory execution")

    for token in (
        "const std::string& expectedProgramName",
        "m_motionTrackedProgramName != expectedProgramName",
        "currentProgram != expectedProgramName",
        "RobotOperationLease::IsCancellationRequested(this)",
        "firstProgramLine != secondProgramLine",
        "poseAngleDriftDeg > 0.2",
        "bool STEPRobotCtrl::ResumeTrackedProgramFromPause",
        "STEP暂停失败：机器人连接不可用",
        "connectionUsable()",
        "ConnectStatus() < 0",
        "positionDeviation > maxPositionDeviationMm",
        "return ProgStartRunWithSdkLock(sdkLock, true)",
    ):
        require(token in step, f"STEP stable-pause safety check missing: {token}")

    for token in (
        "trajectoryInExecutionOrder",
        "sourceIdentityConsistent",
        "sha256Pattern",
        "SAME_LOCAL_ARC_POSITION_EPSILON_MM",
        "parts[2].toDouble",
        "parts[3].toDouble",
        "parts[4].toDouble",
        "LoadExecutionTrajectorySnapshot",
        "QCryptographicHash::hash(payload",
        "size = payload.size()",
        "PlanFromPausedPoseBound",
        "MaxResumeCheckpointAgeSeconds",
        "MaxFutureClockSkewSeconds",
        "created.secsTo(now)",
        "断点已超过 %1 小时有效期",
        "expectedIdentity.trajectorySha256",
        "expectedIdentity.trajectorySize",
        "expectedIdentity.trajectoryPointCount",
    ):
        require(token in planner, f"planner validation/column parsing missing: {token}")
    require("executionIdentity.sampledPoseSha256" in service
            and "record.trajectorySha256.compare(sampledPoseSha256" in dialog,
            "saved FinalSampled snapshot is not identical to the frozen checkpoint identity")

    require('src\\WeldResumePlanner.cpp' in project and 'include\\WeldResumePlanner.h' in project,
            "planner sources are not part of the Visual Studio project")
    require("/*resumeStartArcMm=*/-1.0, /*inputAlreadyInExecutionOrder=*/false, stopped" in process_loop,
            "process-loop call still maps stop callback to the old positional parameter")
    require("if (settings.doWeld)" in process_loop
            and "InvalidateStoredWeldResumeCheckpoint" in process_loop,
            "process-loop weld mode can scan before invalidating an old checkpoint")

    print("PASS: V2 weld resume binds immutable execution identity and uses stable pause plus millimeter arc backtrack")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
