#pragma once

#include <functional>
#include <sstream>
#include <string>
#include <vector>

// Controller-side mode sequencing. No motion, program-start, alarm-reset or
// emergency-stop-release command is issued by this diagnostic sequence.
namespace InovanceModeSequence
{
// Bump when the preparation/restoration contract changes. A persisted plan
// from another contract must be revalidated, not silently reused.
inline constexpr int kStrategyRevision = 1;
struct State
{
    int mode = -1, motor = -1, stream = -1, motion = -1, task = -1, estop = -1, fault = -1;
    bool Idle() const { return motion == 0 && (task == 0 || task == 10); }
    bool Known() const { return (mode == 1 || mode == 2) && (motor == 0 || motor == 1)
        && stream >= 0 && stream <= 2 && (estop == 0 || estop == 1) && fault >= 0; }
    bool Safe() const { return Known() && Idle() && estop == 0 && fault == 0; }
    std::string Text() const
    {
        std::ostringstream out;
        out << "mode=" << mode << " motor=" << motor << " ds=" << stream
            << " motion=" << motion << " task=" << task << " eStop=" << estop << " fault=" << fault;
        return out.str();
    }
};
struct Plan
{
    std::string id, name;
    int mode = 1;
    bool streamFirst = false, powerOffFirst = false;
};
inline std::vector<Plan> Plans()
{
    std::vector<Plan> plans;
    for (int mode : { 1, 2 })
        for (bool streamFirst : { false, true })
            for (bool powerOffFirst : { false, true })
            {
                Plan plan;
                plan.mode = mode; plan.streamFirst = streamFirst; plan.powerOffFirst = powerOffFirst;
                plan.id = std::to_string(mode) + (streamFirst ? "_ds_motor" : "_motor_ds")
                    + (powerOffFirst ? "_off_motor_ds" : "_off_ds_motor");
                plan.name = std::string(mode == 1 ? "手动" : "自动")
                    + (streamFirst ? "｜开数据流→上电" : "｜上电→开数据流")
                    + (powerOffFirst ? "｜下电→关数据流" : "｜关数据流→下电");
                plans.push_back(plan);
            }
    return plans;
}
inline bool Find(const std::string& id, Plan& plan)
{
    for (const auto& candidate : Plans())
        if (candidate.id == id) { plan = candidate; return true; }
    return false;
}
struct Ops
{
    std::function<bool(State&, std::string&)> read;
    std::function<bool(const std::string&, std::string&)> send;
    std::function<bool()> cancelled;
    std::function<void()> delay;
};
inline bool Read(const Ops& ops, State& state, std::string& trace, const char* label)
{
    std::string error;
    const bool ok = ops.read(state, error);
    trace += std::string(label) + ": " + (ok ? state.Text() : "读取失败：" + error) + "\n";
    return ok;
}
inline bool Step(const Ops& ops, const std::string& command,
    const std::function<bool(const State&)>& expected, std::string& trace, bool cleanup = false)
{
    if (!cleanup && ops.cancelled()) { trace += "已取消，未发送：" + command + "\n"; return false; }
    State before;
    if (!Read(ops, before, trace, "命令前") || !before.Idle()
        || (!cleanup && !before.Safe()))
    { trace += "前置状态不安全，未发送：" + command + "\n"; return false; }
    std::string reply;
    const bool acknowledged = ops.send(command, reply);
    trace += command + " -> " + (acknowledged ? "OK " : "FAIL ") + reply + "\n";
    State after;
    for (int attempt = 0; attempt < 40; ++attempt)
    {
        if (!Read(ops, after, trace, "回读") || !after.Known() || !after.Idle()
            || (!cleanup && (!after.Safe() || ops.cancelled()))) { return false; }
        if (expected(after)) { return acknowledged; }
        if (!acknowledged) { return false; }
        ops.delay();
    }
    trace += "状态回读未达到预期，停止后续步骤。\n";
    return false;
}
inline bool SetMode(const Ops& ops, int mode, std::string& trace, bool cleanup = false)
{
    return Step(ops, "Set_Mode " + std::to_string(mode),
        [mode](const State& s) { return s.mode == mode; }, trace, cleanup);
}
inline bool RuntimeReady(const State& state, int mode)
{
    return state.Safe() && state.mode == mode && state.motor == 1;
}
inline bool VerifyStableRuntimeState(const Ops& ops, int mode, int stream, std::string& trace,
    const char* label, bool cleanup = false)
{
    State state;
    for (int sample = 0; sample < 5; ++sample)
    {
        if ((!cleanup && ops.cancelled()) || !Read(ops, state, trace, label)
            || !RuntimeReady(state, mode) || state.stream != stream)
        { return false; }
        ops.delay();
    }
    return true;
}
inline bool SettleRuntimeMode(const Ops& ops, int mode, State& settled,
    std::string& trace, const char* label, bool cleanup = false)
{
    // Set_Mode is asynchronous on the field controller: its first matching
    // read can still be followed by a delayed servo drop. Observe the mode for
    // 250 ms before the final Motor ON instead of treating that first read as
    // the end of the transition.
    for (int sample = 0; sample < 10; ++sample)
    {
        if ((!cleanup && ops.cancelled()) || !Read(ops, settled, trace, label)
            || !settled.Safe() || settled.mode != mode || settled.stream != 0)
        { return false; }
        ops.delay();
    }
    return true;
}
inline bool EnsureRuntimeReady(const Ops& ops, int mode, std::string& trace,
    bool cleanup = false)
{
    if (mode != 1 && mode != 2) { trace += "生产数据流运行模式无效。\n"; return false; }
    State state;
    if (!Read(ops, state, trace, "生产运行态准备前") || !state.Safe() || state.stream != 0)
    { return false; }
    const bool switchedMode = state.mode != mode;
    if (switchedMode && !SetMode(ops, mode, trace, cleanup)) { return false; }
    if (switchedMode
        && !SettleRuntimeMode(ops, mode, state, trace, "模式切换稳定回读", cleanup))
    { return false; }
    if (switchedMode || state.motor != 1)
    {
        if (!Step(ops, "Motor ON", [mode](const State& s) {
            return s.mode == mode && s.motor == 1 && s.stream == 0;
        }, trace, cleanup)) { return false; }
    }
    return VerifyStableRuntimeState(ops, mode, 0, trace, "伺服常驻稳定回读", cleanup);
}
// The connection baseline remains automatic + powered. A controller whose
// verified data-stream recipe uses manual mode is switched once on first
// direct motion. The mode is allowed to settle before Motor ON and is then kept
// powered between direct-motion segments. Native JOB startup independently
// restores automatic mode.
inline bool RestoreRuntimeReady(const Ops& ops, int mode, std::string& trace)
{
    State state;
    if (!Read(ops, state, trace, "运行态恢复前") || !state.Known() || !state.Idle()
        || state.estop != 0 || state.fault != 0)
    { return false; }
    if (state.stream != 0
        && !Step(ops, "Dsmode OFF", [](const State& s) { return s.stream == 0; }, trace, true))
    { return false; }
    if (ops.cancelled())
    {
        trace += "运行已取消：仅关闭数据流，不自动切模式或重新上电。\n";
        return false;
    }
    return EnsureRuntimeReady(ops, mode, trace);
}
inline bool OpenRuntimeStream(const Ops& ops, int mode, std::string& trace)
{
    if (!EnsureRuntimeReady(ops, mode, trace))
    {
        trace += "生产运行未能进入已验收模式、伺服上电、数据流关闭的稳定状态；未发送开流命令。\n";
        return false;
    }
    if (!Step(ops, "Dsmode ON", [mode](const State& s) { return RuntimeReady(s, mode)
        && s.stream == 1; }, trace)
        || !VerifyStableRuntimeState(ops, mode, 1, trace, "生产开流稳定回读"))
    {
        const bool restored = RestoreRuntimeReady(ops, mode, trace);
        trace += std::string("生产运行态恢复=") + (restored ? "PASS\n" : "FAIL\n");
        return false;
    }
    return true;
}
inline bool CloseRuntimeStream(const Ops& ops, int mode, std::string& trace)
{
    State before;
    if (!Read(ops, before, trace, "生产关流基线") || !before.Known() || !before.Idle()
        || before.estop != 0 || before.fault != 0 || before.mode != mode || before.motor != 1)
    {
        trace += "生产关流要求机器人停止、保持已验收运行模式且伺服保持上电。\n";
        return false;
    }
    if (before.stream != 0
        && !Step(ops, "Dsmode OFF", [mode](const State& s) { return RuntimeReady(s, mode)
            && s.stream == 0; }, trace, true))
    { return false; }
    return VerifyStableRuntimeState(ops, mode, 0, trace, "生产关流稳定回读", true);
}
// Restore to stopped, data-stream OFF, motor OFF, original mode. Fallback to
// manual mode is bounded and only follows an e4 response in a verified idle state.
inline bool RestoreOff(const Ops& ops, int originalMode, std::string& trace)
{
    State state;
    if ((originalMode != 1 && originalMode != 2)
        || !Read(ops, state, trace, "恢复前") || !state.Known() || !state.Idle()) { return false; }
    if (state.stream != 0)
    {
        if (!Step(ops, "Dsmode OFF", [](const State& s) { return s.stream == 0; }, trace, true)) { return false; }
    }
    if (!Read(ops, state, trace, "下电前") || !state.Idle() || state.stream != 0) { return false; }
    if (state.motor != 0)
    {
        // Keep the failing command's own response, not a subsequent read error.
        bool modeRejected = false;
        Ops offOps = ops;
        offOps.send = [&](const std::string& command, std::string& reply)
        {
            const bool ok = ops.send(command, reply);
            modeRejected = !ok && reply.find("e4:") != std::string::npos;
            return ok;
        };
        if (!Step(offOps, "Motor OFF", [](const State& s) { return s.motor == 0; }, trace, true))
        {
            if (!modeRejected || !Read(ops, state, trace, "e4后状态")
                || !state.Idle() || state.stream != 0 || state.mode != 2
                || !SetMode(ops, 1, trace, true)
                || !Step(ops, "Motor OFF", [](const State& s) { return s.motor == 0; }, trace, true)) { return false; }
        }
    }
    if (!Read(ops, state, trace, "恢复模式前") || !state.Idle() || state.motor != 0 || state.stream != 0) { return false; }
    if (state.mode != originalMode && !SetMode(ops, originalMode, trace, true)) { return false; }
    for (int sample = 0; sample < 3; ++sample)
    {
        if (!Read(ops, state, trace, "恢复终态") || !state.Idle()
            || state.mode != originalMode || state.motor != 0 || state.stream != 0) { return false; }
        ops.delay();
    }
    return true;
}
inline bool Prepare(const Ops& ops, const Plan& plan, std::string& trace)
{
    State before;
    if (!Read(ops, before, trace, "准备基线") || !before.Safe() || before.stream != 0
        || (before.motor != 0 && before.motor != 1)) { return false; }
    if (plan.streamFirst && before.motor != 0 && !RestoreOff(ops, before.mode, trace)) { return false; }
    if (!SetMode(ops, plan.mode, trace)) { return false; }
    const auto motorOn = [&]() { return Step(ops, "Motor ON", [](const State& s) { return s.motor == 1; }, trace); };
    const auto streamOn = [&]() { return Step(ops, "Dsmode ON", [](const State& s) { return s.stream == 1; }, trace); };
    if (!(plan.streamFirst ? streamOn() && motorOn() : motorOn() && streamOn())) { return false; }
    State after;
    return Read(ops, after, trace, "准备终态") && after.Safe() && after.mode == plan.mode
        && after.motor == 1 && after.stream == 1 && !ops.cancelled();
}
inline bool Test(const Ops& ops, const Plan& plan, bool& restoreVerified, std::string& trace)
{
    restoreVerified = false;
    State baseline;
    if (!Read(ops, baseline, trace, "测试基线") || !baseline.Safe()
        || baseline.motor != 0 || baseline.stream != 0)
    {
        trace += "要求机器人已停止、任务停止/就绪、无急停/报警、伺服下电且数据流关闭；未执行测试。\n";
        return false;
    }
    bool passed = Prepare(ops, plan, trace);
    if (passed)
    {
        const auto streamOff = [&]() { return Step(ops, "Dsmode OFF", [](const State& s) { return s.stream == 0; }, trace); };
        const auto motorOff = [&]() { return Step(ops, "Motor OFF", [](const State& s) { return s.motor == 0; }, trace); };
        passed = plan.powerOffFirst ? motorOff() && streamOff() : streamOff() && motorOff();
    }
    // Recovery is allowed after cancellation; no more test startup is allowed.
    restoreVerified = RestoreOff(ops, baseline.mode, trace);
    State restored;
    restoreVerified = restoreVerified && Read(ops, restored, trace, "恢复后安全状态") && restored.Safe();
    passed = passed && restoreVerified && !ops.cancelled();
    trace += std::string("组合结果=") + (passed ? "PASS" : "FAIL")
        + "，恢复验证=" + (restoreVerified ? "PASS\n" : "FAIL（禁止继续）\n");
    return passed;
}
}
