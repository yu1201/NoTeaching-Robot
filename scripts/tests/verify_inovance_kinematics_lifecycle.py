"""Read-only source contract regression; never connects to a robot."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
driver = (ROOT / "src/InovanceRobotDriver.cpp").read_text(encoding="utf-8")

def body(signature: str) -> str:
    start = driver.index("{", driver.index(signature))
    depth = 1
    end = start + 1
    while depth:
        depth += (driver[end] == "{") - (driver[end] == "}")
        end += 1
    return driver[start:end]

connect = body("bool InovanceRobotCtrl::ConnectWithPolicy(")
assert connect.index("lock.unlock()") < connect.index("RefreshKinematicsFromController(kinematics)")
assert "return IsConnected();" in connect, "failed assets must not discard the authenticated read-only connection"
close = body("bool InovanceRobotCtrl::CloseSocketLocked(")
assert "m_kinematicsSession.Invalidate()" in close and "m_passivePulseValid = false" in close
refresh = body("bool InovanceRobotCtrl::RefreshKinematicsFromController(")
for token in ["m_kinematicsSession.Invalidate()", '"Get_MotionSts"', "StationarySample(joints, finalJoints)",
              '"cRobotName"', '"i32AxisNum"', '"sourceSha256"', '"controlHost"', '"ftpHost"',
              '"InovanceKinematics"', '"ValidatedSnapshot"', "ConfigDatabase::WriteScopedSetting",
              "ConfigDatabase::ReadScopedSetting", "readback != serialized", "generation != m_kinematicsSession.Generation()"]:
    assert token in refresh, token
assert refresh.index("readback != serialized") < refresh.index("InstallValidatedKinematicsModel") < refresh.index("m_kinematicsSession.Publish")
assert refresh.index("ConfigDatabase::WriteScopedSetting") < refresh.index("socketLock(m_socketMutex)"), "SQLite must not block the STOP socket lock"
for forbidden in ["ServoOn(", "EnsureControlPermit(", "SetOperationMode(", "SendJointMove(", "uploadFile("]:
    assert forbidden not in refresh, forbidden
pulse = body("bool InovanceRobotCtrl::TryGetCurrentPulse(")
assert "std::try_to_lock" in pulse and "m_kinematicsSession.Ready()" in pulse
assert '"Get_PosHerePulse"' not in pulse, "raw encoder fallback must not return success through generic pulse API"
reload = body("void InovanceRobotCtrl::ReloadRuntimeConfiguration(")
assert reload.index("CloseSocketLocked()") < reload.index("InitRobotDriver(") < reload.index("ConnectWithPolicy(false)")
for token in ["m_continuousJogRunning.load()", "MotionCompletionPending(this)", 'QueryIntLocked("Get_MotionSts", motion)']:
    assert token in reload, token
wire = body("bool InovanceRobotCtrl::SendCommandLocked(")
assert "m_kinematicsReadInProgress.load()" in wire
exclusion = wire[wire.index("m_kinematicsReadInProgress.load()"):wire.index("汇川正在只读校验运动学资产")]
for command in ['"Prg Start"', '"Motor ON"', '"Dsmode ON"']:
    assert command in exclusion, command
for command in ['"Prg Stop"', '"Motor OFF"', '"Dsmode OFF"']:
    assert command not in exclusion, f"safety command must not be blocked: {command}"
print("PASS: live connection kinematics recipe, independent database evidence, stale-session revocation and no raw-pulse fallback")
