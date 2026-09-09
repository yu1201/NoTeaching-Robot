#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

// Pure arithmetic for the common acceptance test, never controller commands.
namespace RobotAcceptanceJointMotion
{
inline constexpr double DeltaDegrees = 0.5;
inline constexpr double SpeedPercent = 1.0;
template<class Axis> std::array<double, 6> Units(const Axis& axis)
{
    return { axis.dSPulseUnit, axis.dLPulseUnit, axis.dUPulseUnit,
        axis.dRPulseUnit, axis.dBPulseUnit, axis.dTPulseUnit };
}
template<class Pulse> std::array<long long, 9> Values(const Pulse& pulse)
{
    return { pulse.nSPulse, pulse.nLPulse, pulse.nUPulse, pulse.nRPulse,
        pulse.nBPulse, pulse.nTPulse, pulse.lBXPulse, pulse.lBYPulse, pulse.lBZPulse };
}
template<class Axis> bool ValidUnits(const Axis& axis)
{
    for (double unit : Units(axis))
        if (!std::isfinite(unit) || std::abs(unit) < 1e-12 || std::abs(unit) > 0.02) return false;
    return true;
}
template<class Pulse, class Axis> bool OutwardTarget(const Pulse& start, const Axis& axis, Pulse& target)
{
    if (!ValidUnits(axis)) return false;
    const double delta = std::round(DeltaDegrees / axis.dSPulseUnit);
    const double value = static_cast<double>(start.nSPulse) + delta;
    if (!std::isfinite(value) || delta == 0 || value < (std::numeric_limits<long>::min)()
        || value > (std::numeric_limits<long>::max)()) return false;
    target = start; // All other robot/external axes retain their captured values.
    target.nSPulse = static_cast<long>(value);
    return true;
}
template<class Pulse, class Axis> bool Matches(const Pulse& actual, const Pulse& target,
    const Axis& axis, double& maxErrorDegrees)
{
    maxErrorDegrees = 0;
    if (!ValidUnits(axis)) return false;
    const auto a = Values(actual), t = Values(target);
    const auto units = Units(axis);
    bool ok = true;
    for (int i = 0; i < 6; ++i)
    {
        const double error = static_cast<double>(std::abs(static_cast<long double>(a[i]) - static_cast<long double>(t[i])) * std::abs(units[i]));
        maxErrorDegrees = (std::max)(maxErrorDegrees, error);
        ok = ok && error <= (std::max)(0.02, 2 * std::abs(units[i]));
    }
    for (int i = 6; i < 9; ++i)
        ok = ok && std::abs(static_cast<long double>(a[i]) - static_cast<long double>(t[i])) <= 2;
    return ok;
}
}
