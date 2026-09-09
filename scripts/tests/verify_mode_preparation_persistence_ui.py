"""Offline UI/service contract for persisted robot-owned preparation recipes."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ui = (ROOT / "src/FunctionTestDialog.cpp").read_text(encoding="utf-8")
service = (ROOT / "src/MeasureThenWeldService.cpp").read_text(encoding="utf-8")
driver = (ROOT / "src/InovanceRobotDriver.cpp").read_text(encoding="utf-8")

begin = ui.index("connect(m_pAdaptorModeApplyBtn,")
end = ui.index("QFormLayout* interfaceForm", begin)
apply = ui[begin:end]
assert "driver->UseVerifiedModePreparation(id)" in apply
assert apply.index("driver->UseVerifiedModePreparation(id)") < apply.index("已固化当前机器人数据流组合")
assert "DecodeRobotMessageText(driver->GetLastRobotError())" in apply
assert "已固化到当前机器人数据库，重启/重连自动恢复，运行前仍检查安全状态" in apply
assert "机器人、端点或版本绑定不匹配时需重新测试选用" in apply
assert "重新连接后需重新验证" not in apply
assert "已选用本连接数据流组合" not in ui
assert "重连后需要重新验证" not in ui

# Evidence/report content is still only evidence. Readiness is obtained from
# the adaptor, and an old pass record cannot be treated as authorization.
assert "driver->ActiveModePreparationId().empty()" in ui
assert "旧验收记录未固化的，需要重新测试并选用一次" in ui
for forbidden in ("m_verifiedModePreparations", "InovanceModePreparationStore",
                  "dynamic_cast<InovanceRobotCtrl", "static_cast<InovanceRobotCtrl"):
    assert forbidden not in ui, f"business UI must not bypass brand adaptor authorization: {forbidden}"

failure = service.index('const QString robotError = DecodeRobotMessageText(pRobotDriver->GetLastRobotError()).trimmed();')
end = service.index("result.lastPhase = ScanCyclePhase::AtStartSafe", failure)
block = service[failure:end]
assert "扫描循环失败：未能到达扫描下枪安全位置" in block
assert "机器人最近错误：%1" in block
assert "return fail(failure, true);" in block, "preserve existing stop/recovery semantics"
assert "GetRobotStatusText(" not in block, "do not overwrite the captured failure with another live query"
assert "safetyCheckpointRejected" in service[service.rfind("if (!MoveScanStartSafeAndWait(", 0, failure):failure]

# A read-only status diagnostic must not erase the exact completion failure that
# the outer scan-cycle dialog/report needs.  The controller can also expose one
# transient motion=2 sample while direct-stream segments are switched; require a
# stable interruption witness instead of releasing the active command identity.
wait_begin = service.index("bool WaitRobotMotionDone(")
wait_end = service.index("bool MeasureThenWeldService::LoadPresetParam", wait_begin)
wait = service[wait_begin:wait_end]
assert "const std::string completionError" in wait
assert "RobotMotionStatusText(pRobotDriver)" in wait
assert "pRobotDriver->SetLastRobotError(completionError)" in wait

completion_begin = driver.index("bool InovanceRobotCtrl::WaitForCommandDone(")
completion_end = driver.index("bool InovanceRobotCtrl::BeginTrackedDirectMotion", completion_begin)
completion = driver[completion_begin:completion_end]
assert "int stableInterrupted = 0;" in completion
assert "++stableInterrupted;" in completion
assert "stableInterrupted >= 3" in completion
assert "stableInterrupted = 0;" in completion
assert "Get_CmdSts=" in completion and "Get_MotionSts=2" in completion

print("PASS: persisted preparation UI keeps adaptor authority, preserves scan errors and debounces transient Inovance interruption state")
