from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
service = (ROOT / "src" / "MeasureThenWeldService.cpp").read_text(encoding="utf-8")
dialog = (ROOT / "src" / "MeasureThenWeldDialog.cpp").read_text(encoding="utf-8")
planner = (ROOT / "src" / "WeldResumePlanner.cpp").read_text(encoding="utf-8")
store = (ROOT / "src" / "WeldSafetyRecoveryStore.cpp").read_text(encoding="utf-8")
virtual = (ROOT / "src" / "VirtualWeldTestDialog.cpp").read_text(encoding="utf-8")
application = (ROOT / "src" / "QtWidgetsApplication4.cpp").read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


execute_start = service.index("bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos")
execute_end = service.index("bool MeasureThenWeldService::LoadCompPreviewBaseline", execute_start)
execute = service[execute_start:execute_end]

callback_gate = execute.index("if (!executionPrepared || !executionFinished)")
first_motion = execute.index("MoveCoorsAndWait(")
require(callback_gate < first_motion,
        "callback-less CLI/virtual caller can reach a real robot motion")
callsite_counts = {
    "MeasureThenWeldDialog.cpp": dialog.count("ExecuteWeldPoseFileWithSafePos("),
    "VirtualWeldTestDialog.cpp": virtual.count("ExecuteWeldPoseFileWithSafePos("),
    "QtWidgetsApplication4.cpp": application.count("ExecuteWeldPoseFileWithSafePos("),
}
require(callsite_counts == {
    "MeasureThenWeldDialog.cpp": 3,
    "VirtualWeldTestDialog.cpp": 1,
    "QtWidgetsApplication4.cpp": 1,
}, f"unreviewed ExecuteWeldPoseFileWithSafePos call-site set: {callsite_counts}")
require("WeldExecutionTerminalResult& terminal" in dialog,
        "interactive production call sites do not supply persistent terminal callbacks")
require("SyntheticVirtualTest" in virtual
        and "WeldSafetyRecoverySession" in virtual
        and "safetySession->Prepare" in virtual
        and "safetySession->Finish" in virtual,
        "virtual real-motion caller lacks the shared persistent recovery session")
require("WeldSafetyRecoverySession" in application
        and "weldSafetySession->Prepare" in application
        and "weldSafetySession->Finish" in application,
        "automatic process-loop weld lacks the shared persistent recovery session")

persist = execute.index("PersistProgramCompletedUnretracted")
post_confirm = execute.index('"焊后确认"', persist)
retreat = execute.index("MoveCoorsAndWait(", post_confirm)
verify = execute.index("VerifyRobotAtSafePose", retreat)
safe_finish = execute.index("FinishSafelyRetracted", verify)
require(persist < post_confirm < retreat < verify < safe_finish,
        "safe terminal ordering is not persist -> confirm -> retreat -> verify -> finish")
require("executionContextGuard.Finish(true)" not in execute,
        "program completion still releases context before safe retreat")
for token in (
    "if (!postWeldConfirmed)",
    "用户在焊后确认选择取消；未发起任何新运动",
    "ShouldAttemptMandatoryRetreat(postWeldConfirmed, stopLatched)",
    "安全 STOP 已锁存；未发起收枪运动",
    "RobotOperationLease::RequestCancellation(pRobotDriver)",
):
    require(token in execute, f"Cancel/STOP/unretracted gate missing: {token}")

for token in (
    "SafeRetreatPending",
    'record.state = QStringLiteral("unretracted")',
    'record.state = QStringLiteral("interrupted")',
    "BuildSafeRetreatWitness",
    "m_activeWeldCheckpointRecord = value",
    "RunSafeRetreatRecoveryFlow",
    "绝不会再次执行焊缝",
):
    require(token in dialog, f"persistent recovery gate missing: {token}")
require("std::recursive_mutex g_storeMutex" in store
        and "g_weldBreakpointRecordMutex" not in dialog,
        "WeldBreakpoint database module still has multiple unrelated locks/writers")
begin = store[store.index("bool WeldSafetyRecoveryStore::BeginOrUpdatePending"):
              store.index("bool WeldSafetyRecoveryStore::WriteCompletedAndClearPending")]
complete = store[store.index("bool WeldSafetyRecoveryStore::WriteCompletedAndClearPending"):
                 store.index("bool WeldSafetyRecoveryStore::InvalidateIfNoPending")]
require(begin.index("WritePendingLocked") < begin.index("WriteRecordLocked"),
        "pending begin writes RecordV2 before fail-closed marker")
require(complete.index("WriteRecordLocked") < complete.index("WritePendingLocked"),
        "safe completion clears marker before verified RecordV2 write")
require("写后回读不一致" in store,
        "shared recovery store does not verify database writes by readback")
require("GetPrivateProfileStringW" not in store
        and "ReadScopedSettingStatus" in store
        and "kMaxRecordUtf8Bytes = 64 * 1024" in store,
        "RecordV2 is not using the bounded tri-state ConfigStore reader")
lease = (ROOT / "src" / "RobotOperationLease.cpp").read_text(encoding="utf-8")
require("PersistentAdmissionBlocked" in lease
        and "TryAcquireSafetyRecovery" in lease,
        "persistent unretracted state does not globally block new endpoint leases")
require('record.state == QStringLiteral("unretracted")' in planner
        and 'record.state == QStringLiteral("interrupted")' in planner,
        "planner does not recognize fail-closed recovery states")
require("ValidateSafeRetreatRecoveryRecord" in planner,
        "recovery record witness is not validated")

print("PASS: weld completion cannot release before confirmed, verified safe retreat")
