#include "MeasureThenWeldCapabilityPolicy.h"

#include <cstdlib>
#include <iostream>
#include <limits>

// Deliberately non-production bit positions: policy must use the enum contract,
// not duplicate numeric capability values from RobotDriverAdaptor.
enum class Capability : std::uint64_t
{
    PassiveState = 1ULL << 17,
    LinearMotion = 1ULL << 3,
    JointMotion = 1ULL << 28,
    ContinuousTrajectory = 1ULL << 8,
    VerifiedProgramCompletion = 1ULL << 25,
    VerifiedSafeAbort = 1ULL << 2,
    ActualArcWeld = 1ULL << 31,
};

static int checks = 0;
static void Check(bool condition, const char* label)
{
    ++checks;
    if (!condition) { std::cerr << "FAIL: " << label << '\n'; std::exit(1); }
}

static bool Supports(std::uint64_t available, std::uint64_t required)
{
    return (available & required) == required;
}

int main()
{
    namespace Policy = MeasureThenWeldCapabilityPolicy;
    const auto bit = [](Capability capability) { return static_cast<std::uint64_t>(capability); };
    const std::uint64_t linearDriver = bit(Capability::PassiveState) | bit(Capability::LinearMotion)
        | bit(Capability::ContinuousTrajectory) | bit(Capability::VerifiedProgramCompletion)
        | bit(Capability::VerifiedSafeAbort);
    Check(Supports(linearDriver, Policy::EntryMask<Capability>()),
        "linear-only brand may open configuration");
    Check(Supports(linearDriver, Policy::ScanMask<Capability>(true, false)),
        "computed safety route requires no joint motion");
    Check(Supports(linearDriver, Policy::ScanMask<Capability>(true, true)),
        "computed continuous scan may run without joint motion");
    Check(!Supports(linearDriver, Policy::ScanMask<Capability>(false, false)),
        "taught safety route blocks a linear-only brand");
    Check(!Supports(linearDriver, Policy::ScanMask<Capability>(false, true)),
        "continuous scan cannot bypass taught-route joint requirement");
    Check(Supports(linearDriver | bit(Capability::JointMotion), Policy::ScanMask<Capability>(false, true)),
        "verified joint capability admits configured taught route");
    Check(Supports(linearDriver, Policy::WeldMask<Capability>(false)), "dry run does not require arc welding");
    Check(!Supports(linearDriver, Policy::WeldMask<Capability>(true)), "actual welding requires arc capability");
    Check(Supports(linearDriver | bit(Capability::ActualArcWeld), Policy::WeldMask<Capability>(true)),
        "actual welding capability does not imply joint motion");
    const auto withoutContinuous = linearDriver & ~bit(Capability::ContinuousTrajectory);
    Check(Supports(withoutContinuous, Policy::ScanMask<Capability>(true, false)),
        "ordinary endpoint scan preserves existing linear requirement");
    Check(!Supports(withoutContinuous, Policy::ScanMask<Capability>(true, true)),
        "custom trajectory cannot fall back to a linear endpoint move");
    for (const auto required : { Capability::PassiveState, Capability::LinearMotion,
        Capability::VerifiedProgramCompletion, Capability::VerifiedSafeAbort })
    {
        const auto incomplete = linearDriver & ~bit(required);
        Check(!Supports(incomplete, Policy::EntryMask<Capability>()), "entry preserves basic safety capabilities");
        Check(!Supports(incomplete, Policy::ScanMask<Capability>(true, false)), "scan preserves basic safety capabilities");
    }

    double delta = -1.0;
    std::string error;
    Check(Policy::TryMaxWristDeltaDeg({ 1000, -2000, 3000 }, { 0, 0, 0 },
        { 0.1, 0.01, 0.001 }, delta, &error) && delta == 100.0 && error.empty(),
        "pulse values use each axis's degrees-per-pulse scale");
    Check(Policy::TryMaxWristDeltaDeg({ 90, -90, 45 }, { 0, 0, 0 },
        { 1.0, 1.0, 1.0 }, delta, &error) && delta == 90.0,
        "degree-valued driver remains supported with unit scale");
    Check(Policy::TryMaxWristDeltaDeg({ 1000, 0, 0 }, { 0, 0, 0 },
        { -0.1, 0.01, 0.001 }, delta, &error) && delta == 100.0,
        "signed axis scale preserves direction-independent risk magnitude");
    const double invalidUnits[] = { 0.0, -0.0, 1e-13, -1e-13,
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity() };
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        for (double invalid : invalidUnits)
        {
            std::array<double, 3> units = { 0.1, 0.1, 0.1 };
            units[axis] = invalid;
            delta = 123.0;
            Check(!Policy::TryMaxWristDeltaDeg({ 0, 0, 0 }, { 0, 0, 0 }, units, delta, &error)
                && !error.empty() && delta == 0.0,
                "invalid wrist unit blocks even identical pulses; never implies zero risk");
        }
    }
    const long highest = std::numeric_limits<long>::max();
    const long lowest = std::numeric_limits<long>::lowest();
    const double expected = (static_cast<double>(highest) - static_cast<double>(lowest)) * 0.001;
    Check(Policy::TryMaxWristDeltaDeg({ highest, 0, 0 }, { lowest, 0, 0 },
        { 0.001, 1.0, 1.0 }, delta, &error) && delta == expected,
        "pulse difference converts before signed long subtraction");
    Check(!Policy::TryMaxWristDeltaDeg({ highest, 0, 0 }, { lowest, 0, 0 },
        { std::numeric_limits<double>::max(), 1.0, 1.0 }, delta, &error) && !error.empty(),
        "angle overflow blocks instead of bypassing flip check");
    Check(Policy::TryMaxWristDeltaDeg({ 0, 0, 0 }, { 0, 0, 0 }, { 1.0, 1.0, 1.0 }, delta, &error)
        && delta == 0.0 && error.empty(), "valid zero delta clears prior errors");
    std::cout << "PASS: " << checks << " measure-then-weld capability and wrist-unit checks (offline)\n";
}
