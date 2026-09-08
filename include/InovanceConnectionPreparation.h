#pragma once

#include <functional>
#include <sstream>
#include <string>

// Deterministic, no-motion connection preparation for Inovance controllers.
// The caller owns connection/session locking. This helper never reconnects,
// starts a program, moves the robot, releases an emergency stop or powers off.
// A stranded, powered-off motion-interrupted state may be cleared with the
// controller's documented project Stop/BackStartLine commands before normal
// preparation continues. The terminal motion=0 readback remains mandatory.
namespace InovanceConnectionPreparation
{
struct State
{
    int mode = -1;
    int motor = -1;
    int stream = -1;
    int motion = -1;
    int task = -1;
    int estop = -1;
    int fault = -1;
    int controlDevice = -1;
    int permit = -1;

    bool Known() const
    {
        return (mode == 1 || mode == 2) && (motor == 0 || motor == 1)
            && stream >= 0 && stream <= 2 && motion >= 0
            && (task == 0 || task == 1 || task == 10)
            && (estop == 0 || estop == 1) && fault >= 0
            && controlDevice >= 0 && controlDevice <= 4
            && permit >= 0 && permit <= 2;
    }
    bool TaskStopped() const { return task == 0 || task == 10; }
    bool Idle() const { return motion == 0 && TaskStopped(); }
    bool InterruptedRecoveryInvariant() const
    {
        return Known() && (motion == 0 || motion == 2) && TaskStopped()
            && motor == 0 && stream == 0 && estop == 0 && fault == 0
            && controlDevice == 2 && permit != 2;
    }
    bool RecoverableInterrupted() const
    {
        return motion == 2 && InterruptedRecoveryInvariant();
    }
    std::string Text() const
    {
        std::ostringstream out;
        out << "mode=" << mode << " motor=" << motor << " ds=" << stream
            << " motion=" << motion << " task=" << task << " eStop=" << estop
            << " fault=" << fault << " ctrlDev=" << controlDevice << " permit=" << permit;
        return out.str();
    }
};

struct Ops
{
    std::function<bool(State&, std::string&)> read;
    std::function<bool(const std::string&, std::string&)> send;
    std::function<bool(std::string&)> prepareCoordinates;
    std::function<bool(std::string&)> prepareKinematics;
    std::function<bool()> cancelled;
    std::function<void()> delay;
};

inline bool Read(const Ops& ops, State& state, std::string& evidence, const char* label)
{
    if (ops.cancelled())
    {
        evidence += std::string(label) + "：已取消或连接发生变化。\n";
        return false;
    }
    std::string error;
    const bool ok = ops.read(state, error);
    evidence += std::string(label) + "：" + (ok ? state.Text() : "读取失败：" + error) + "\n";
    return ok;
}

inline bool BaselineAllowed(const State& state, bool allowFault, std::string& evidence)
{
    if (!state.Known()) { evidence += "控制器状态不完整或超出已知范围，停止初始化。\n"; return false; }
    if (!state.Idle()) { evidence += "机器人或主任务未停止，停止初始化。\n"; return false; }
    if (state.stream != 0) { evidence += "存在已打开的数据流，禁止接管并停止初始化。\n"; return false; }
    if (state.estop != 0) { evidence += "实体急停尚未解除；接口不会解除急停，停止初始化。\n"; return false; }
    if (state.controlDevice != 2)
    {
        evidence += "控制设备不是远程以太网客户端；请在示教器/InoRobotLab中切换，接口只支持查询，停止初始化。\n";
        return false;
    }
    if (state.permit == 2) { evidence += "控制许可被其它远程客户端占用，自动初始化不强抢。\n"; return false; }
    if (!allowFault && state.fault != 0) { evidence += "报警尚未清除，停止后续模式切换和上电。\n"; return false; }
    return true;
}

inline bool Send(const Ops& ops, const std::string& command, std::string& evidence)
{
    if (ops.cancelled()) { evidence += "已取消，未发送：" + command + "\n"; return false; }
    std::string reply;
    const bool ok = ops.send(command, reply);
    evidence += command + " -> " + (ok ? "OK " : "FAIL ") + reply + "\n";
    return ok;
}

inline bool WaitFor(const Ops& ops, const std::function<bool(const State&)>& expected,
    bool allowFault, std::string& evidence, const char* label, State& state)
{
    for (int attempt = 0; attempt < 40; ++attempt)
    {
        if (!Read(ops, state, evidence, label) || !BaselineAllowed(state, allowFault, evidence)) return false;
        if (expected(state)) return true;
        if (attempt + 1 < 40) ops.delay();
    }
    evidence += std::string(label) + "未在限定时间内达到预期，停止初始化。\n";
    return false;
}

inline bool ReadInterruptedRecovery(const Ops& ops, State& state,
    std::string& evidence, const char* label)
{
    if (!Read(ops, state, evidence, label)) return false;
    if (!state.InterruptedRecoveryInvariant())
    {
        evidence += "中断态恢复期间状态越出安全边界；要求motion仅为0/2、伺服下电、数据流关闭、任务停止、无急停/故障及远程控制设备。\n";
        return false;
    }
    return true;
}

inline bool WaitForInterruptedRecovery(const Ops& ops,
    const std::function<bool(const State&)>& expected,
    std::string& evidence, const char* label, State& state, int attempts = 40)
{
    for (int attempt = 0; attempt < attempts; ++attempt)
    {
        if (!ReadInterruptedRecovery(ops, state, evidence, label)) return false;
        if (expected(state)) return true;
        if (attempt + 1 < attempts) ops.delay();
    }
    return false;
}

inline bool RecoverInterruptedState(const Ops& ops, State& state, std::string& evidence)
{
    if (!state.RecoverableInterrupted() || state.permit != 1)
    {
        evidence += "运动中断态不满足自动恢复条件或当前客户端尚未取得控制许可。\n";
        return false;
    }

    // Do not react to one transient motion=2 sample. Require three consecutive
    // powered-off, stream-off, task-stopped samples before issuing no-motion
    // controller reset commands.
    for (int sample = 0; sample < 3; ++sample)
    {
        if (!ReadInterruptedRecovery(ops, state, evidence, "中断态稳定确认")) return false;
        if (state.motion == 0)
        {
            evidence += "运动中断态已自行恢复为停止，SKIP工程复位。\n";
            return true;
        }
        if (state.motion != 2 || state.permit != 1)
        {
            evidence += "中断态稳定确认失败，未发送工程复位命令。\n";
            return false;
        }
        ops.delay();
    }

    evidence += "运动中断态已连续确认3次；执行不启动运动的工程停止/返回启动行恢复。\n";
    if (!Send(ops, "Prg Stop", evidence)) return false;

    // Prg Stop can itself clear the controller's interrupted state. Observe a
    // short bounded window before deciding whether BackStartLine is necessary.
    if (WaitForInterruptedRecovery(ops,
        [](const State& s) { return s.motion == 0 && s.permit == 1; },
        evidence, "工程停止回读", state, 10))
    {
        evidence += "工程停止已清除运动中断态，SKIP返回启动行。\n";
        return true;
    }

    if (!state.RecoverableInterrupted() || state.permit != 1)
    {
        evidence += "工程停止后状态不再满足受限恢复条件，未发送返回启动行。\n";
        return false;
    }
    if (!Send(ops, "BackStartLine", evidence)
        || !WaitForInterruptedRecovery(ops,
            [](const State& s) { return s.motion == 0 && s.permit == 1; },
            evidence, "返回启动行回读", state))
    {
        evidence += "工程停止/返回启动行后仍未连续确认motion=0，禁止切模式和上电。\n";
        return false;
    }
    evidence += "运动中断态恢复完成：motion=0，机器人未启动运动。\n";
    return true;
}

inline bool Prepare(const Ops& ops, std::string& evidence)
{
    evidence.clear();
    if (!ops.read || !ops.send || !ops.cancelled || !ops.delay)
    { evidence = "汇川连接后初始化回调不完整，未执行。\n"; return false; }

    State state;
    if (!Read(ops, state, evidence, "初始化基线")) return false;
    const bool recoverInterrupted = state.RecoverableInterrupted();
    if (!recoverInterrupted && !BaselineAllowed(state, true, evidence)) return false;
    if (recoverInterrupted)
    {
        evidence += "检测到可恢复运动中断态：伺服已下电、数据流关闭且主任务停止；先取得本客户端许可，再受限复位。\n";
    }

    if (state.permit == 0)
    {
        if (!Send(ops, "AcqPermit", evidence)) return false;
        if (recoverInterrupted)
        {
            if (!WaitForInterruptedRecovery(ops,
                [](const State& s) { return s.permit == 1; },
                evidence, "中断态许可回读", state)) return false;
        }
        else if (!WaitFor(ops, [](const State& s) { return s.permit == 1; }, true,
            evidence, "许可回读", state)) return false;
    }
    else { evidence += "控制许可：当前客户端已持有，SKIP。\n"; }

    if (recoverInterrupted && !RecoverInterruptedState(ops, state, evidence)) return false;

    if (state.fault != 0)
    {
        if (!Send(ops, "ResetErr", evidence)
            || !WaitFor(ops, [](const State& s) { return s.fault == 0; }, true,
                evidence, "报警复位回读", state)) return false;
    }
    else { evidence += "清除报警：当前无报警，SKIP。\n"; }

    if (!Read(ops, state, evidence, "坐标准备前") || !BaselineAllowed(state, false, evidence)
        || state.permit != 1) return false;
    if (ops.prepareCoordinates)
    {
        std::string coordinateEvidence;
        if (!ops.prepareCoordinates(coordinateEvidence))
        {
            evidence += "工具/工件坐标准备：FAIL " + coordinateEvidence + "\n";
            return false;
        }
        evidence += "工具/工件坐标准备：OK " + coordinateEvidence + "\n";
    }
    else { evidence += "工具/工件坐标准备：无配置动作，SKIP。\n"; }

    if (ops.prepareKinematics)
    {
        std::string kinematicsEvidence;
        if (!ops.prepareKinematics(kinematicsEvidence))
        {
            evidence += "运动学资产准备：FAIL " + kinematicsEvidence + "\n";
            return false;
        }
        evidence += "运动学资产准备：OK " + kinematicsEvidence + "\n";
    }
    else { evidence += "运动学资产准备：无配置动作，SKIP。\n"; }

    if (!Read(ops, state, evidence, "自动模式前") || !BaselineAllowed(state, false, evidence)
        || state.permit != 1) return false;
    const bool switchedToAutomatic = state.mode != 2;
    if (switchedToAutomatic)
    {
        if (!Send(ops, "Set_Mode 2", evidence)
            || !WaitFor(ops, [](const State& s) { return s.mode == 2; }, false,
                evidence, "自动模式回读", state)) return false;
    }
    else { evidence += "自动模式：已是模式2，SKIP。\n"; }

    if (switchedToAutomatic)
    {
        // Get_Mode can report the new mode before the controller finishes the
        // transition and drops servo power. Let that transition settle before
        // deciding whether Motor ON is required.
        for (int sample = 0; sample < 10; ++sample)
        {
            if (!Read(ops, state, evidence, "自动模式稳定回读")
                || !BaselineAllowed(state, false, evidence) || state.mode != 2
                || state.permit != 1) return false;
            ops.delay();
        }
    }
    if (!Read(ops, state, evidence, "伺服上电前") || !BaselineAllowed(state, false, evidence)
        || state.permit != 1 || state.mode != 2) return false;
    if (state.motor != 1)
    {
        if (!Send(ops, "Motor ON", evidence)
            || !WaitFor(ops, [](const State& s) { return s.motor == 1; }, false,
                evidence, "伺服上电回读", state)) return false;
    }
    else { evidence += "伺服上电：已上电，SKIP。\n"; }

    for (int sample = 0; sample < 5; ++sample)
    {
        if (!Read(ops, state, evidence, "初始化终态") || !BaselineAllowed(state, false, evidence)
            || state.controlDevice != 2 || state.permit != 1 || state.mode != 2
            || state.motor != 1 || state.stream != 0)
        { evidence += "初始化终态未满足远程许可、自动模式、伺服上电及数据流关闭条件。\n"; return false; }
        ops.delay();
    }
    evidence += "连接后前置初始化完成：远程控制已确认、当前客户端许可、无报警、自动模式、伺服上电；未启动运动。\n";
    return true;
}
}
