from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LEASE_H = (ROOT / "include" / "RobotOperationLease.h").read_text(encoding="utf-8")
LEASE = (ROOT / "src" / "RobotOperationLease.cpp").read_text(encoding="utf-8")
STORE = (ROOT / "src" / "WeldSafetyRecoveryStore.cpp").read_text(encoding="utf-8")
DIALOG = (ROOT / "src" / "MeasureThenWeldDialog.cpp").read_text(encoding="utf-8")
STEP = (ROOT / "src" / "STEPRobotDriver.cpp").read_text(encoding="utf-8")
POLICY = (ROOT / "include" / "RobotRecoverySafetyPolicy.h").read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"FAIL: {message}")


require("TryAcquirePausedResume" in LEASE_H, "dedicated paused resume lease API is missing")
paused_start = LEASE.index("RobotOperationLease::Ptr RobotOperationLease::TryAcquirePausedResume")
paused_end = LEASE.index("RobotOperationLease::Ptr RobotOperationLease::TryAcquireImpl", paused_start)
paused_lease = LEASE[paused_start:paused_end]
require("AcquireExclusiveRecoveryBinding" in paused_lease,
        "paused lease does not acquire the Store endpoint binding after registration")
require(LEASE.count("AcquireExclusiveRecoveryBinding") >= 2
        and "ReleaseExclusiveRecoveryBinding" in LEASE,
        "safe-retreat and paused leases do not both own/release Store bindings")
destructor = LEASE[LEASE.index("RobotOperationLease::~RobotOperationLease()") :]
require(destructor.index("ReleaseExclusiveRecoveryBinding")
        < destructor.index("m_driver == nullptr")
        < destructor.index("std::lock_guard<std::mutex> lock"),
        "lease destructor can return before releasing Store/hardware registrations")
for token in (
    "g_activeEndpointRecoveryBindings",
    "CollectEndpointRecoveryCandidatesLocked",
    "TryListScopedSettingIdsBounded",
    "ReadEndpointIndexLocked",
    "candidates.size() != 1",
    "candidate.record.robotName != storedRobot",
):
    require(token in STORE, f"exclusive endpoint recovery binding missing {token}")

resume = DIALOG[DIALOG.index("void MeasureThenWeldDialog::RunResumeWeldFlow"):
                DIALOG.index("void MeasureThenWeldDialog::RunSafeRetreatRecoveryFlow")]
require("TryAcquirePausedResume" in resume and "TryAcquire(" not in resume,
        "paused resume still uses ordinary pending-blocked acquisition")
require(resume.index("TerminatePersistedProgramBeforeRecovery")
        < resume.index("TransitionBoundRecordState"),
        "paused record transitions to resuming before old program termination")
require("RevalidateExclusiveRecoveryBinding" in resume
        and "encodedSha256" in STORE
        and "selected.encoded != binding->encodedRecord" in STORE,
        "paused STOP window lacks full encoded record/hash CAS")

retreat = DIALOG[DIALOG.index("void MeasureThenWeldDialog::RunSafeRetreatRecoveryFlow"):]
require(retreat.index("TerminatePersistedProgramBeforeRecovery")
        < retreat.index("MoveCoorsAndWait"),
        "interrupted recovery can Move before persisted program termination")
terminate_pos = retreat.index("TerminatePersistedProgramBeforeRecovery")
require(retreat.index("RevalidateExclusiveRecoveryBinding", terminate_pos)
        < retreat.index("MoveCoorsAndWait"),
        "safe-retreat does not revalidate the exclusive binding after STOP/Kill")
helper = DIALOG[DIALOG.index("bool TerminatePersistedProgramBeforeRecovery"):
                DIALOG.index("class LaserLineLiveView")]
require(helper.index("MarkMotionStarted") < helper.index("AbortPersistedMotion")
        < helper.index("MarkMotionCompleted"),
        "adaptor restart recovery does not make an unknown prior task fail closed")

require("AbortPersistedProgramForRecovery" in STEP,
        "STEP restart recovery termination API is missing")
step_abort = STEP[STEP.index("bool STEPRobotCtrl::AbortPersistedProgramForRecovery"):
                  STEP.index("int STEPRobotCtrl::GetCurrentProgramLine")]
for token in ("stableEmptyStopped", "ResolveStepPersistedProgramAction", "ProgramKillCmd"):
    require(token in step_abort, f"STEP persisted termination missing {token}")

for token in (
    "StepReadProgramContentIdentity",
    "ArmGeneratedProgramContentWitness",
    "VerifyGeneratedProgramRemoteContentLocked",
    "downloadFileBounded",
    "SameProgramContent",
    "StepTemporaryFileCleanup",
):
    require(token in STEP, f"STEP remote content gate missing {token}")
load = STEP[STEP.index("bool STEPRobotCtrl::LoadUserProgram"):STEP.index("bool STEPRobotCtrl::UnLoadUserProgramer")]
require(load.count("VerifyGeneratedProgramRemoteContentLocked") >= 2,
        "SDK-locked program load window is not checked before and after load")
load_critical = load[load.index("nRet = WithSdkCommand"):
                     load.index("});", load.index("nRet = WithSdkCommand"))]
require(load_critical.count("ClearGeneratedProgramContentWitnessLocked") >= 2,
        "LoadUserProgram clears a mismatched witness outside the SDK mutex")
start = STEP[STEP.index("bool STEPRobotCtrl::VerifyGeneratedProgramReadyForStartLocked"):
             STEP.index("bool STEPRobotCtrl::WriteContiMoveAnyFiles")]
require("VerifyGeneratedProgramRemoteContentLocked" in start
        and "StopAndUnloadGeneratedProgramLocked" in start,
        "remote swap before START is not stopped and rejected")
require(STEP.count("START后远端SRP/SRD内容复核失败") >= 2
        and "暂停恢复START前远端SRP/SRD内容已变化" in STEP,
        "normal/resumed START windows are not remotely rechecked and stopped")
post_start = STEP[STEP.index("bool STEPRobotCtrl::VerifyGeneratedProgramAfterStartWithSdkUnlock"):
                  STEP.index("bool STEPRobotCtrl::ArmGeneratedProgramContentWitness")]
post_start_unlock = post_start.index("sdkLock.unlock()")
post_start_remote_verify = post_start.index(
    "VerifyGeneratedProgramRemoteContentSnapshot", post_start_unlock)
post_start_relock = post_start.index("sdkLock.lock()", post_start_remote_verify)
require(post_start_unlock < post_start_remote_verify < post_start_relock,
        "post-START WinINet verification still holds the STOP-blocking SDK mutex")
for token in (
    "RemoteContentVerificationGate",
    "CancelActiveRemoteContentVerification",
    "SameProgramContentWitness",
    "ClearGeneratedProgramContentWitnessIfSnapshotLocked",
    "m_motionTrackedProjectName == expectedProject",
    "m_completionWitnessProjectName == expectedProject",
):
    require(token in STEP or token in POLICY,
            f"post-START unlocked verification missing {token}")
require(STEP.count("VerifyGeneratedProgramAfterStartWithSdkUnlock(") >= 3,
        "both STEP START implementations do not use unlocked post-START verification")
resume_start = STEP[STEP.index("bool STEPRobotCtrl::ResumeTrackedProgramFromPause"):
                    STEP.index("bool STEPRobotCtrl::AbortCurrentProgram()")]
require("ProgStartRunWithSdkLock(sdkLock, true)" in resume_start
        and "Prog_startRun_Py(true)" not in resume_start,
        "paused resume retains an outer recursive SDK lock during post-START FTP")
abort_current = STEP[STEP.index("bool STEPRobotCtrl::AbortCurrentProgram()"):
                     STEP.index("bool STEPRobotCtrl::AbortPersistedProgramForRecovery")]
require(abort_current.index("CancelActiveRemoteContentVerification()")
        < abort_current.index("m_sdkCommandMutex"),
        "red STOP waits for the SDK mutex before cancelling blocked FTP verification")
policy_test = (ROOT / "scripts" / "tests" / "robot_recovery_safety_policy_tests.cpp").read_text(
    encoding="utf-8")
for token in ("blockedVerifier", "ftpCondition", "stopCompleted",
              "SameProgramContentWitness", "verificationToken"):
    require(token in policy_test,
            f"post-START FTP/STOP concurrency test missing {token}")
require("QCryptographicHash::Sha256" in POLICY and "expected.size == observed.size" in POLICY,
        "STEP content identity is not bound to both SHA256 and size")
require("kStepMaximumGeneratedProgramBytes" in STEP
        and "声明大小为空或超过128MiB上限" in STEP,
        "oversized STEP remote declarations are not rejected before body download")

print("PASS: interrupted, paused and STEP remote-content static gates")
