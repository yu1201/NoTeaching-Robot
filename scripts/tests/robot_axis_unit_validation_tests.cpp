#include "RobotAxisUnitValidation.h"
#include <iostream>
#include <limits>

int main()
{
    using RobotAxisUnitValidation::FromCalibration;
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const bool valid = FromCalibration(true, 360., true, 360000.) == .001
        && FromCalibration(true, -360., true, 360000.) == -.001
        && FromCalibration(false, 360., true, 360000.) == 0.
        && FromCalibration(true, 360., false, 360000.) == 0.
        && FromCalibration(false, 0., false, 0.) == 0.
        && FromCalibration(true, 360., true, 0.) == 0.
        && FromCalibration(true, 0., true, 360000.) == 0.
        && FromCalibration(true, nan, true, 360000.) == 0.
        && FromCalibration(true, 360., true, inf) == 0.
        && FromCalibration(true, 1e308, true, 1e-308) == 0.
        && FromCalibration(true, 1e-308, true, 1e308) == 0.;
    std::cout << (valid ? "PASS" : "FAIL")
        << ": missing, partial, nonfinite and zero axis units fail closed; signed calibration preserved\n";
    return valid ? 0 : 1;
}
