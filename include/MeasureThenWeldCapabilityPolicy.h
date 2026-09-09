#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

// Shared by the configuration entry, adaptor acceptance and execution preflight.
// Capability is a template parameter so the policy can be tested without a robot,
// Qt, KDL or a driver instance. Production callers use RobotDriverCapability.
namespace MeasureThenWeldCapabilityPolicy
{
template <typename Capability>
constexpr std::uint64_t BasicMotionMask()
{
    return static_cast<std::uint64_t>(Capability::LinearMotion)
        | static_cast<std::uint64_t>(Capability::PassiveState)
        | static_cast<std::uint64_t>(Capability::VerifiedProgramCompletion)
        | static_cast<std::uint64_t>(Capability::VerifiedSafeAbort);
}

template <typename Capability>
constexpr std::uint64_t EntryMask()
{
    // Opening the page does not select a safety route or send a motion command.
    return BasicMotionMask<Capability>()
        | static_cast<std::uint64_t>(Capability::ContinuousTrajectory);
}

template <typename Capability>
constexpr std::uint64_t WeldMask(bool actualWeld)
{
    return EntryMask<Capability>()
        | (actualWeld ? static_cast<std::uint64_t>(Capability::ActualArcWeld) : 0);
}

template <typename Capability>
constexpr std::uint64_t ScanMask(bool useComputedScanSafe, bool continuousTrajectory)
{
    return BasicMotionMask<Capability>()
        | (!useComputedScanSafe ? static_cast<std::uint64_t>(Capability::JointMotion) : 0)
        | (continuousTrajectory ? static_cast<std::uint64_t>(Capability::ContinuousTrajectory) : 0);
}

inline bool ValidateWristAxisUnits(
    const std::array<double, 3>& degreesPerPulse,
    std::string* error = nullptr)
{
    constexpr const char* axisNames[] = { "R", "B", "T" };
    for (std::size_t index = 0; index < degreesPerPulse.size(); ++index)
    {
        if (!std::isfinite(degreesPerPulse[index]) || std::abs(degreesPerPulse[index]) <= 1e-12)
        {
            if (error != nullptr)
            {
                *error = std::string(axisNames[index])
                    + "轴脉冲角度单位无效，无法把关节读数转换为角度；禁止跳过姿态翻转检查。";
            }
            return false;
        }
    }
    if (error != nullptr) { error->clear(); }
    return true;
}

inline bool TryMaxWristDeltaDeg(
    const std::array<long, 3>& currentPulse,
    const std::array<long, 3>& targetPulse,
    const std::array<double, 3>& degreesPerPulse,
    double& maxDeltaDeg,
    std::string* error = nullptr)
{
    maxDeltaDeg = 0.0;
    if (!ValidateWristAxisUnits(degreesPerPulse, error)) { return false; }
    for (std::size_t index = 0; index < currentPulse.size(); ++index)
    {
        // Convert before subtraction: Windows long subtraction can overflow even
        // though both captured pulse values and the angle difference are valid.
        const double delta = std::abs((static_cast<double>(currentPulse[index])
            - static_cast<double>(targetPulse[index])) * degreesPerPulse[index]);
        if (!std::isfinite(delta))
        {
            if (error != nullptr)
            {
                *error = "关节角度差不是有限数，禁止继续扫描安全位运动。";
            }
            maxDeltaDeg = 0.0;
            return false;
        }
        maxDeltaDeg = std::max(maxDeltaDeg, delta);
    }
    return true;
}
}
