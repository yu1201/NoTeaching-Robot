#include "RobotCalculation.h"

#include <QCoreApplication>

#include <chrono>
#include <cmath>
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

bool VerifyStraightFeatureCurve()
{
    QVector<RobotCalculation::IndexedPoint3D> points;
    for (int index = 0; index <= 100; ++index)
    {
        RobotCalculation::IndexedPoint3D point;
        point.index = index;
        const double x = static_cast<double>(index);
        point.point = Eigen::Vector3d(x, 2.0 * x, -0.5 * x);
        points.push_back(point);
    }

    RobotCalculation::LowerWeldFilterParams params;
    params.sampleStep = 2.5;
    params.smoothRadius = 3;
    const auto result = RobotCalculation::BuildSmoothFeatureCurve(points, params);
    if (!result.ok || result.points.size() < 80)
    {
        std::cerr << "straight feature curve generation failed: "
                  << result.error.toStdString() << "\n";
        return false;
    }
    for (const auto& point : result.points)
    {
        if (std::abs(point.point.y() - 2.0 * point.point.x()) > 1e-8
            || std::abs(point.point.z() + 0.5 * point.point.x()) > 1e-8)
        {
            std::cerr << "straight feature curve left the source line\n";
            return false;
        }
    }
    return true;
}

bool VerifyNoisyFeatureCurveIsSmoothed()
{
    QVector<RobotCalculation::IndexedPoint3D> points;
    double inputSquaredError = 0.0;
    for (int index = 0; index <= 200; ++index)
    {
        const double x = static_cast<double>(index) * 0.5;
        const double baseY = 0.3 * std::sin(x / 15.0);
        const double baseZ = 0.1 * std::cos(x / 20.0);
        const double noiseY = (index % 2 == 0) ? 0.4 : -0.4;
        const double noiseZ = (index % 3 == 0) ? 0.2 : -0.1;
        RobotCalculation::IndexedPoint3D point;
        point.index = index;
        point.point = Eigen::Vector3d(x, baseY + noiseY, baseZ + noiseZ);
        points.push_back(point);
        inputSquaredError += noiseY * noiseY + noiseZ * noiseZ;
    }

    RobotCalculation::LowerWeldFilterParams params;
    params.sampleStep = 1.0;
    params.smoothRadius = 4;
    params.zContinuityThreshold = 3.0;
    const auto result = RobotCalculation::BuildSmoothFeatureCurve(points, params);
    if (!result.ok || result.points.size() < 90)
    {
        std::cerr << "noisy feature curve generation failed: "
                  << result.error.toStdString() << "\n";
        return false;
    }

    double outputSquaredError = 0.0;
    for (const auto& point : result.points)
    {
        const double x = point.point.x();
        const double baseY = 0.3 * std::sin(x / 15.0);
        const double baseZ = 0.1 * std::cos(x / 20.0);
        outputSquaredError += std::pow(point.point.y() - baseY, 2.0)
            + std::pow(point.point.z() - baseZ, 2.0);
    }
    const double inputRms = std::sqrt(inputSquaredError / points.size());
    const double outputRms = std::sqrt(outputSquaredError / result.points.size());
    if (!(outputRms < inputRms * 0.55))
    {
        std::cerr << "feature curve smoothing insufficient: input_rms=" << inputRms
                  << " output_rms=" << outputRms << "\n";
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
    if (!VerifyStraightFeatureCurve())
    {
        return 3;
    }
    if (!VerifyNoisyFeatureCurveIsSmoothed())
    {
        return 4;
    }
    std::cout << "PASS: geometry cancellation and feature-point smooth-curve contracts hold\n";
    return 0;
}
