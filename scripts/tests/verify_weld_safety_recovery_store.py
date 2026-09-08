from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
HEADER = (ROOT / "include" / "ConfigDatabase.h").read_text(encoding="utf-8")
DATABASE = (ROOT / "src" / "ConfigDatabase.cpp").read_text(encoding="utf-8")
STORE = (ROOT / "src" / "WeldSafetyRecoveryStore.cpp").read_text(encoding="utf-8")
TEST = (ROOT / "scripts" / "tests" / "weld_safety_recovery_store_tests.cpp").read_text(
    encoding="utf-8"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"FAIL: {message}")


require("enum class ReadStatus" in HEADER, "ConfigStore reads do not distinguish missing from error")
require("ReadScopedSettingStatus" in HEADER and "ReadIniValueStatus" not in HEADER,
        "database-native tri-state read API is missing or path compatibility remains")
require("query.lastError().isValid()" in DATABASE
        and "QTWIDGETSAPP4_TEST_CONFIG_CURSOR_ERROR" in DATABASE,
        "single-value cursor failures can still collapse to NotFound")
require("TryListScopedSettingIdsBounded" in HEADER and "LIMIT ?" in DATABASE,
        "legacy endpoint scan has no query-level count bound")
require("query.lastError().isValid()" in DATABASE,
        "bounded enumeration does not detect cursor errors")

require("GetPrivateProfileStringW" not in STORE and "COPini" not in STORE,
        "recovery reads still bypass ConfigStore")
require("kMaxRecordUtf8Bytes = 64 * 1024" in STORE and "value.toUtf8() != utf8" in STORE,
        "RecordV2 lacks a strict length/UTF-8 read gate")
require('value == "0"' in STORE and 'value == "1"' in STORE,
        "pending marker is not strictly parsed")
require("RecordV2 但 SafeRetreatPending 缺失" in STORE,
        "marker-missing records are not failed closed")
invalidate = STORE[STORE.index("bool WeldSafetyRecoveryStore::InvalidateIfNoPending"):
                   STORE.index("bool WeldSafetyRecoveryStore::PersistentAdmissionBlocked")]
require("WriteRecordLocked" in invalidate and "WritePendingLocked(robotName, false" in invalidate,
        "safe invalidation does not leave an explicit marker for the global alias scan")

require('kEndpointScopeType[] = "weld_safety_endpoint"' in STORE,
        "canonical endpoint index scope is missing")
require("NormalizePersistentEndpointIdentity" in STORE and "storedEndpoint != endpoint" in STORE,
        "endpoint index identity is not recomputed and read back")
require("WriteScopedSettings" in STORE and "ReadEndpointIndexLocked(endpoint" in STORE,
        "endpoint index is not transactionally written and read back")
require("TryListScopedSettingIdsBounded" in STORE and "kMaxRobotNameLength" in STORE,
        "legacy alias scan lacks hard count/name bounds")
require("NormalizePersistentEndpointIdentity(record.robotEndpoint)" in STORE,
        "legacy records are compared without canonicalizing their endpoint")
for token in (
    "AcquireExclusiveRecoveryBinding",
    "g_activeEndpointRecoveryBindings",
    "CollectEndpointRecoveryCandidatesLocked",
    "candidates.size() != 1",
    "RevalidateExclusiveRecoveryBinding",
    "TransitionBoundRecordState",
    "selected.encodedSha256 != binding->encodedSha256",
):
    require(token in STORE, f"exclusive recovery binding/CAS missing: {token}")
scan = STORE[STORE.index("for (const QString& storedRobot : storedRobots)"):
             STORE.index("return false;", STORE.index("for (const QString& storedRobot : storedRobots)"))]
require("storedRobot == normalizedRobot" in scan and "Qt::CaseInsensitive" not in scan,
        "legacy scan skips a distinct case-sensitive ConfigStore scope")

begin = STORE.index("bool WeldSafetyRecoveryStore::BeginOrUpdatePending")
end = STORE.index("bool WeldSafetyRecoveryStore::WriteCompletedAndClearPending", begin)
body = STORE[begin:end]
require(body.index("EnsureEndpointIndexForRecordLocked") < body.index("WritePendingLocked")
        < body.index("WriteRecordLocked"),
        "pending write order does not preserve endpoint-index fail-closed semantics")

for token, scenario in (
    ("long RecordV2", "long ConfigStore RecordV2 roundtrip"),
    ("RecordV2 without marker", "missing marker fail-closed"),
    ("same physical endpoint under another robot name", "endpoint alias bypass"),
    ("bounded legacy scan missed a canonical endpoint alias", "pre-index canonical alias scan"),
    ("case-only robot scope alias bypassed", "case-only pre-index scope alias"),
    ("legacy pre-index alias pending did not block", "exclusive legacy alias recovery"),
    ("two safe-retreat records", "dual safe-retreat endpoint ambiguity"),
    ("STOP-period safe-retreat replacement", "post-STOP safe-retreat CAS"),
    ("STOP-period paused record replacement", "post-STOP paused CAS"),
    ("ConfigStore read failure must fail closed", "read failure fail-closed"),
):
    require(token in TEST, f"dynamic test missing: {scenario}")

print("PASS: WeldSafetyRecoveryStore static storage gates")
