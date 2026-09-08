#!/usr/bin/env python3
"""Read-only source integration gates; no database, controller or build access.

The store's offline C++ tests cover JSON/CAS behaviour. These checks cover the
driver call boundaries that a store-only test cannot observe. They do not
replace a dynamic concurrency test or authorize changing the power sequence.
"""

import re
import sys

sys.dont_write_bytecode = True
from verify_inovance_driver_adaptor import function_body, read, require


def scopes_at(source: str, position: int) -> tuple[int, ...]:
    """Lexical brace scopes, ignoring comments and ordinary C++ literals."""
    tokens = r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\''
    code = re.sub(tokens, lambda match: " " * len(match.group()), source, flags=re.S)
    scopes = []
    for index, char in enumerate(code[:position]):
        if char == "{":
            scopes.append(index)
        elif char == "}":
            scopes.pop()
    return tuple(scopes)


def main() -> None:
    driver = read("src/InovanceRobotDriver.cpp")
    store = read("include/RobotModePreparationStore.h")
    sequence = read("include/InovanceModeSequence.h")
    body = lambda name: function_body(driver, "InovanceRobotCtrl::" + name + "(")
    restore = body("RestoreModePreparation")
    retest = body("RunModePreparationTestCase")
    select = body("UseVerifiedModePreparation")
    active = body("ActiveModePreparationId")
    ops = body("ModeSequenceOps")
    failures = []

    def case(name, check):
        try:
            check()
            print("PASS:", name)
        except (AssertionError, ValueError) as error:
            failures.append(name)
            print("FAIL:", name, "-", error)

    def restore_is_read_only():
        commands = re.findall(r'SendCommand(?:Locked)?\("([^"]+)"', restore)
        require(commands == ["Get_RobotType", "Get_FwVersion"],
                "restore may issue only the two live identity queries")
        for forbidden in ("EnsureControlPermit(", "ServoOn(", "SetOperationMode(",
                          "SetDataStreamMode(", "::Test(", "::Prepare(",
                          "::RestoreOff(", "SendCommand(", "RunModePreparationTestCase("):
            require(forbidden not in restore, "restore calls " + forbidden)
        connect = body("ConnectWithPolicy")
        require(connect.index("lock.unlock();") < connect.index("RestoreModePreparation();"),
                "Connect must release its socket lock before restore")
        require(connect.index("RestoreModePreparation();") < connect.index("RefreshKinematicsFromController("),
                "recipe loading must not depend on FTP/FK/Joints validation")

    def restore_binding_and_epoch():
        for token in ("binding.robotName", "binding.driver", "binding.host", "binding.port",
                      "binding.controllerModel", "binding.firmware", "kStrategyRevision",
                      "SendCommandLocked", "epoch != m_modeConnectionEpoch.load()"):
            require(token in restore, "missing identity/session gate " + token)
        require(restore.index("socketLock(m_socketMutex)") < restore.index('SendCommandLocked("Get_RobotType"'),
                "identity queries must hold socket mutex")
        require(restore.index("RobotModePreparationStore::Load(") < restore.index("modeLock(m_modePreparationMutex)"),
                "restore DB read must be outside mode publication lock")
        socket_scope = scopes_at(restore, restore.index("socketLock(m_socketMutex)"))
        load_scope = scopes_at(restore, restore.index("RobotModePreparationStore::Load("))
        require(load_scope[:len(socket_scope)] != socket_scope,
                "restore must release the identity socket lock before SQLite access")
        require(restore.index("epoch != m_modeConnectionEpoch.load()") < restore.index("m_activeModePreparation = plan.id"),
                "changed epoch must not publish restored strategy")

    def unknown_or_bad_record_stays_inactive():
        require("status == RobotModePreparationStore::LoadStatus::Found" in restore,
                "only a successful store load may activate")
        require("InovanceModeSequence::Find(record.planId.toStdString(), plan)" in restore,
                "unknown saved recipe must not activate")
        require(restore.index("m_activeModePreparation.clear()") < restore.index("if (restored)"),
                "failed load must clear live selection")
        require("m_modePreparationBindingEpoch = identityError.empty() ? epoch : 0" in restore,
                "failed identity read must not grant test/persistence binding")
        for forbidden in ("ModeCombinationEvidence", "ReadHistory", "Stage4State"):
            require(forbidden not in restore and forbidden not in store,
                    "legacy evidence must not migrate into authority: " + forbidden)

    def retest_revokes_before_controller_sequence():
        require("if (m_persistedModePreparation == id)" in retest,
                "retesting a persisted plan must withdraw its old PASS")
        require(retest.index("m_activeModePreparation.clear()") < retest.index("RobotModePreparationStore::Revoke("),
                "revoke failure must not leave the live plan active")
        require(retest.index("RobotModePreparationStore::Revoke(") < retest.index("InovanceModeSequence::Test("),
                "durable revocation must precede any energizing test")
        failed_revoke = function_body(retest, "if (!RobotModePreparationStore::Revoke(")
        require("return false;" in failed_revoke, "DB revoke failure must prevent retest")
        require("result.passed = result.passed && result.restoreVerified" in retest,
                "incomplete recovery cannot produce a savable record")

    def persistence_before_live_selection():
        require(select.index("found->second != epoch") < select.index("RobotModePreparationStore::SaveVerified("),
                "only a current-epoch successful test may be persisted")
        failed_save = function_body(select, "if (!RobotModePreparationStore::SaveVerified(")
        require("return false;" in failed_save, "DB failure must not announce success")
        saved_at = select.index("RobotModePreparationStore::SaveVerified(")
        require(saved_at < select.index("epoch != m_modeConnectionEpoch.load()", saved_at)
                < select.index("m_activeModePreparation = id"),
                "save must finish and session must remain bound before live publication")
        require("SendCommand(" not in select and "SendCommandLocked(" not in select,
                "selection/store I/O must not issue hardware commands under mode mutex")

    def active_is_session_bound():
        for token in ("m_connectionReady.load()", "m_modePreparationBindingEpoch == m_modeConnectionEpoch.load()",
                      "!m_persistedModePreparation.empty()", "m_activeModePreparation == m_persistedModePreparation"):
            require(token in active, "Active is missing " + token)
        require("m_modeConnectionEpoch.fetch_add(1)" in body("CloseSocketLocked"),
                "disconnect must invalidate active session binding")
        labels = body("ModePreparationTestCases")
        require("本连接通过·已固化" in labels and "已恢复固化策略" in labels,
                "restored preference must not be displayed as this-session test PASS")

    def operations_do_not_cross_connections():
        for signature, command in (("ops.read =", "QueryIntLocked("),
                                   ("ops.send =", "SendCommandLocked(")):
            operation = function_body(ops, signature)
            require("m_socketMutex" in operation, signature + " must check epoch inside socket lock")
            require(operation.index("m_socketMutex") < operation.index("epoch != m_modeConnectionEpoch.load()")
                    < operation.index(command), signature + " checks epoch too early or after I/O")
            require("SendCommand(" not in operation and "QueryInt(" not in operation,
                    signature + " must not reacquire the socket lock")
        require("const auto epoch = m_modeConnectionEpoch.load();" not in ops,
                "Ops must use its caller's expected epoch, not silently capture a newer connection")
        test_call = re.search(r"InovanceModeSequence::Test\(ModeSequenceOps\(([^)]*)\)", retest)
        require(test_call is not None and test_call.group(1).strip() == "epoch",
                "retest must pass the original frozen epoch through to all operations")
        stream = body("SetDataStreamMode")
        require(re.search(r"ModeSequenceOps\(\s*[A-Za-z_]\w*\s*\)", stream) is not None,
                "data stream preparation must carry its entry epoch, not adopt a reconnect")

    def stream_ownership_publication_is_locked():
        stream = body("SetDataStreamMode")
        require(stream.index("m_modeConnectionEpoch.load()") < stream.index("queryStream(currentMode)"),
                "stream entry epoch must be frozen before reading controller state")
        query = function_body(stream, "const auto queryStream =")
        require(query.index("m_socketMutex") < query.index("epoch != m_modeConnectionEpoch.load()")
                < query.index("QueryIntLocked("), "stream read must be bound inside the socket lock")
        for operation, publications in (
            (stream, ("m_dataStreamEntryMode.store(plan.mode)",
                      "m_dataStreamEntryMotor.store(1)",
                      "m_dataStreamEnabled.store(true)", "m_dataStreamEntryMode.store(-1)",
                      "m_dataStreamEntryMotor.store(-1)")),
            (body("ServoOff"), ("m_dataStreamEntryMode.store(-1)", "m_dataStreamEntryMotor.store(-1)")),
        ):
            for token in publications:
                publication = operation.index(token)
                lock = operation.rfind("socketLock(m_socketMutex)", 0, publication)
                require(lock >= 0 and scopes_at(operation, lock) == scopes_at(operation, publication),
                        token + " must publish while holding socket mutex")
                require("epoch != m_modeConnectionEpoch.load()" in operation[lock:publication],
                        token + " must recheck the entry epoch before publication")

    def runtime_interlocks_and_power_order_remain():
        stream = body("SetDataStreamMode")
        for token in ("EnsureControlPermit()", "InovanceModeSequence::Find(ActiveModePreparationId(), plan)",
                      "currentMode != 0", "!before.Safe()", "plan.streamFirst",
                      "InovanceModeSequence::OpenRuntimeStream(",
                      "InovanceModeSequence::CloseRuntimeStream("):
            require(token in stream, "runtime preparation gate removed: " + token)
        open_runtime = function_body(sequence, "inline bool OpenRuntimeStream(")
        close_runtime = function_body(sequence, "inline bool CloseRuntimeStream(")
        require('Step(ops, "Dsmode ON"' in open_runtime
                and "EnsureRuntimeReady(ops, mode" in open_runtime and '"Motor OFF"' not in open_runtime,
                "production entry must stabilize the selected mode/power before opening the stream")
        require('Step(ops, "Dsmode OFF"' in close_runtime
                and '"Set_Mode' not in close_runtime and '"Motor ' not in close_runtime,
                "production exit must only close the data stream and preserve mode/servo")
        ensure_runtime = function_body(sequence, "inline bool EnsureRuntimeReady(")
        settle_runtime = function_body(sequence, "inline bool SettleRuntimeMode(")
        require("SetMode(ops, mode" in ensure_runtime and 'Step(ops, "Motor ON"' in ensure_runtime
                and "sample < 10" in settle_runtime,
                "production mode switch must settle before the final resident servo enable")
        prepare = function_body(sequence, "inline bool Prepare(")
        require('Step(ops, "Motor ON"' in prepare and 'Step(ops, "Dsmode ON"' in prepare,
                "acceptance combinations must retain their validated power/data-stream sequencing")
        require("plan.streamFirst ? streamOn() && motorOn() : motorOn() && streamOn()" in prepare,
                "acceptance-test startup ordering changed")

    def store_uses_robot_scope_and_atomic_readback():
        write = function_body(store, "inline bool WriteAtomically(")
        require("ConfigDatabase::CompareAndSwapScopedSetting(" in write,
                "store mutations must use transaction + compare + readback")
        require('"robot", binding.robotName, Module(), Key()' in write,
                "strategy scope must be the selected robot")
        load = function_body(store, "inline LoadStatus Load(")
        for token in ("BindingJson(stored) != BindingJson(expected)", 'object.value("revoked")',
                      'object.value("passed")', 'object.value("restoreVerified")', "ValidateRecord(loaded, error)"):
            require(token in load, "store fail-closed gate missing: " + token)

    case("restore issues identity queries only", restore_is_read_only)
    case("restore binds identity and session", restore_binding_and_epoch)
    case("unknown/DB-error/legacy records stay inactive", unknown_or_bad_record_stays_inactive)
    case("retest durably revokes old PASS first", retest_revokes_before_controller_sequence)
    case("save/readback and epoch precede live selection", persistence_before_live_selection)
    case("active strategy remains session bound", active_is_session_bound)
    case("mode operations cannot cross connection epochs", operations_do_not_cross_connections)
    case("stream ownership publication is socket/epoch guarded", stream_ownership_publication_is_locked)
    case("runtime uses verified mode with resident power while acceptance order remains diagnostic", runtime_interlocks_and_power_order_remain)
    case("robot-scoped atomic store contract", store_uses_robot_scope_and_atomic_readback)
    if failures:
        raise SystemExit(f"FAIL: {len(failures)} of 10 persistence integration gates")
    print("PASS: all 10 Inovance mode persistence integration gates (static/offline only)")


if __name__ == "__main__":
    main()
