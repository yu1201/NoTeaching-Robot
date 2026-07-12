#include "RobotCalculation.h"

#include <QCoreApplication>

#include <chrono>
#include <iostream>

namespace
{
QVector<RobotCalculation::IndexedPoint3D> BuildTrack(int count)
{
    QVector<RobotCalculation::IndexedPoint3D> points;
    points.reserve(count);
    for (int index = 0; index < count; ++index)
    {
        RobotCalculation::IndexedPoint3D point;
        point.index = index;
        const double station = static_cast<double>(index) * 0.02;
        point.point = Eigen::Vector3d(
            10.0 * std::sin(station * 0.03), station, -200.0 + 0.2 * std::sin(station));
        points.push_back(point);
    }
    return points;
}

bool RunCancelableAnalysis(bool alreadyDenoised, int cancelAtCheck, const char* label)
{
    const QVector<RobotCalculation::IndexedPoint3D> points = BuildTrack(100000);
    RobotCalculation::LowerWeldFilterParams params;
    params.inputAlreadyDenoised = alreadyDenoised;
    params.sampleStep = 2.0;
    params.minPointCount = 3;
    int checks = 0;
    params.stopRequested = [&checks, cancelAtCheck]()
    {
        return ++checks >= cancelAtCheck;
    };

    const auto started = std::chrono::steady_clock::now();
    const auto result = RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(points, params);
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (result.ok || !result.error.contains(QStringLiteral("已取消"))
        || checks < cancelAtCheck || elapsed > 2.0)
    {
        std::cerr << label << " cancellation failed: ok=" << result.ok
                  << " checks=" << checks << " elapsed=" << elapsed << "\n";
        return false;
    }
    return true;
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    // The first threshold is beyond the finite-input loop and lands in the
    // local-outlier hot stage.  The second continues through clean-input
    // projection into the azimuth/refine path.
    if (!RunCancelableAnalysis(false, 115, "local-outlier"))
    {
        return 1;
    }
    if (!RunCancelableAnalysis(true, 520, "azimuth"))
    {
        return 2;
    }
    std::cout << "PASS: geometry hot loops honor the rebuild stop token promptly\n";
    return 0;
}
