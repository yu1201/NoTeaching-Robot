#pragma once

#include <cmath>

namespace RobotAxisUnitValidation
{
// A missing/partial calibration is unavailable, never a reused previous-axis value.
inline double FromCalibration(bool angleRead, double angle, bool pulseRead, double pulse)
{
    if (!angleRead || !pulseRead || !std::isfinite(angle) || !std::isfinite(pulse)
        || pulse == 0.0)
    {
        return 0.0;
    }
    const double unit = angle / pulse;
    return std::isfinite(unit) && std::abs(unit) >= 1e-15 ? unit : 0.0;
}
}
