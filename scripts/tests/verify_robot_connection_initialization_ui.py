#!/usr/bin/env python3
"""Offline contract for explicit, leased brand-neutral connection preparation."""
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def method(text: str, signature: str, next_signature: str) -> str:
    return text.split(signature, 1)[1].split(next_signature, 1)[0]


main = source("src/QtWidgetsApplication4.cpp")
acceptance = source("src/FunctionTestDialog.cpp")
homepage = method(main, "void QtWidgetsApplication4::FanucConnectTest()",
                  "void QtWidgetsApplication4::FanucDisconnectTest()")
stage = method(acceptance, "void FunctionTestDialog::RunAdaptorConnectionTest()",
               "void FunctionTestDialog::RunAdaptorFtpRoundTrip()")

for name, body in (("homepage", homepage), ("acceptance stage 0", stage)):
    assert body.index("QMessageBox::question(") < body.index("RobotOperationLease::TryAcquire("), name
    assert body.index("RobotOperationLease::TryAcquire(") < body.index("->Connect()"), name
    assert body.index("->Connect()") < body.index("->InitializeAfterConnect(&initializationSummary)"), name
    assert "QMessageBox::Yes | QMessageBox::No, QMessageBox::No" in body, name
    assert "作业区域安全、实体急停可用" in body, name
    assert "自动清除可复位报警、切换自动模式、伺服上电" in body, name
    assert "初始化" in body and "启动运动" in body, name
    assert "通信连接已建立，但前置初始化失败，未就绪" in body, name
    assert "GetLastRobotError()" in body, name
    for forbidden in ("QInputDialog", "->ServoOn(", "->cleanAlarm(", "->SetOperationMode(",
                      "->StartTrajectory(", "dynamic_cast<InovanceRobotCtrl", "->Disconnect()"):
        assert forbidden not in body, f"{name} must stay on the initialization adaptor hook: {forbidden}"

assert "const bool ok = reused || pRobotDriver->Connect();" in homepage
assert "当前机器人已经连接" not in homepage, "reuse must not return before preparation"
assert "if (initializationOk && connectedReadback)" in homepage
success = homepage.index("机器人连接及前置初始化成功")
assert homepage.index("if (initializationOk && connectedReadback)") < success
assert homepage.index("QMessageBox::warning", success) < homepage.index("通信连接已建立", success)
assert homepage.count("StartStateMonitor(50)") == 1, "retain the existing homepage monitor setup"

assert "const bool connectCommandOk = endpointOk && (reused || driver->Connect());" in stage
assert "const bool initializationAttempted = endpointOk && connectCommandOk && connectedReadback;" in stage
assert "const bool initializationOk = initializationAttempted\n                && driver->InitializeAfterConnect" in stage
assert "&& initializationOk && initializedConnectionReadback;" in stage, "communication alone cannot pass stage 0"
assert "InitializeAfterConnect=%1，初始化后连接回读=%2" in stage
assert "连接后初始化摘要=" in stage and "DecodeRobotMessageText(initializationSummary)" in stage
assert "FinishAdaptorAcceptanceStage(0, ok, result)" in stage
assert "std::thread([self, driver, lease]()" in stage, "the operation lease must cover initialization in the worker"
assert "Qt::QueuedConnection" in stage
assert "StartStateMonitor(" not in stage, "do not introduce monitor thread affinity changes in stage 0"
assert "不上使能、不切换模式" not in stage
assert "初始化失败不能通过" in source("include/RobotAdaptorAcceptancePlan.h")

# State readers and background reconnect paths must not become actuator-writing
# preparation entry points. Direct user actions are the only UI callers.
assert main.count("->InitializeAfterConnect(") == 1
assert acceptance.count("->InitializeAfterConnect(") == 1
for relative in ("src/InovanceRobotDriver.cpp", "src/StepRobotDriver.cpp",
                 "src/FANUCRobotDriver.cpp", "src/RobotDriverAdaptor.cpp"):
    text = source(relative)
    definitions = list(re.finditer(
        r"^(?:bool|void)\s+\w+::(?P<name>Connect(?:WithPolicy)?|EnsureConnectionForMonitor|"
        r"StartStateMonitor|StateMonitorWorker|PrepareStateMonitor|TryGetCurrentPos|TryGetCurrentPulse)\([^;]*?\)"
        r"(?:\s+const)?\s*\{", text, re.MULTILINE))
    for definition in definitions:
        remaining = text[definition.end():]
        following = re.search(r"^\}", remaining, re.MULTILINE)
        assert following, f"cannot identify method end: {relative}:{definition.group('name')}"
        body = remaining[:following.end()]
        assert "InitializeAfterConnect(" not in body, (
            f"background/read-only path cannot perform initialization: {relative}:{definition.group('name')}"
        )

# The monitor preserves structured motion semantics. In particular, Inovance
# raw motion=2 must never be flattened into the same visible label as motion=1.
adaptor = source("include/RobotDriverAdaptor.h")
adaptor_cpp = source("src/RobotDriverAdaptor.cpp")
preparation = source("include/InovanceConnectionPreparation.h")
assert "RobotMotionStatus motion;" in adaptor
assert "snapshot.motion = ReadMotionStatusPassive" in adaptor_cpp
for token in (
    'case RobotMotionState::Running: stateText = "运动中"',
    'case RobotMotionState::Interrupted: stateText = "运动已中断"',
    'case RobotMotionState::Faulted: stateText = "故障"',
):
    assert token in main, token

# Only a powered-off, stream-off, task-stopped interruption may be reset, and
# project reset commands do not prove success without a motion=0 readback.
for token in (
    "RecoverableInterrupted()",
    "InterruptedRecoveryInvariant()",
    'Send(ops, "Prg Stop"',
    'Send(ops, "BackStartLine"',
    "运动中断态已连续确认3次",
    "s.motion == 0 && s.permit == 1",
):
    assert token in preparation, token
for forbidden in ('Send(ops, "Motor OFF"', 'Send(ops, "Dsmode ON"', 'Send(ops, "EStop OFF"'):
    assert forbidden not in preparation, forbidden
assert "prepareKinematics" in preparation
assert "RefreshKinematicsFromController(validation)" in source("src/InovanceRobotDriver.cpp")
assert source("src/InovanceRobotDriver.cpp").index("ops.prepareCoordinates") < source("src/InovanceRobotDriver.cpp").index("ops.prepareKinematics")

print("PASS: explicit leased initialization, structured motion display and fail-closed Inovance interruption recovery")
