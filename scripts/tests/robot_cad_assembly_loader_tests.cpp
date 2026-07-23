#include "RobotCadAssemblyLoader.h"

#include <TopoDS_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTextStream>

#include <array>
#include <cmath>

namespace
{
bool Near(double actual, double expected, double toleranceMm)
{
    return std::isfinite(actual) && std::abs(actual - expected) <= toleranceMm;
}

int Fail(QTextStream& error, const QString& message)
{
    error << "FAIL: " << message << Qt::endl;
    return 1;
}
}

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    QTextStream output(stdout);
    QTextStream errorOutput(stderr);
    const QStringList arguments = application.arguments();
    if (arguments.size() < 2 || arguments.size() > 3
        || (arguments.size() == 3
            && arguments.at(2) != QStringLiteral("--bounds-only")))
    {
        return Fail(errorOutput, QStringLiteral(
            "usage: RobotCadAssemblyLoaderTests <SA10 STEP> [--bounds-only]"));
    }
    const bool boundsOnly = arguments.size() == 3;

    RobotCadAssemblyLoader::Result result;
    RobotCadAssemblyLoader::Options options;
    options.buildDetailedPresentation = !boundsOnly;
    // bounds-only 必须忽略这个详细显示选项，确保不会意外触发网格化。
    options.prepareDisplayTriangulation = true;
    QString error;
    QElapsedTimer timer;
    timer.start();
    if (!RobotCadAssemblyLoader::LoadFile(arguments.at(1), result, error, &options))
        return Fail(errorOutput, QStringLiteral("real vendor STEP load failed: %1").arg(error));

    if (boundsOnly)
    {
        if (result.assemblyShape || result.j0Shape || !result.displayBlocks.empty())
        {
            return Fail(errorOutput, QStringLiteral(
                "bounds-only retained a heavy assembly/J0/display B-Rep"));
        }
        if (result.statistics.detailedPresentationBuilt
            || result.statistics.displayTriangulationPrepared
            || result.statistics.displayBlockCount != 0)
        {
            return Fail(errorOutput, QStringLiteral(
                "bounds-only reported detailed presentation or triangulation"));
        }
    }
    else
    {
        if (!result.assemblyShape || result.assemblyShape->IsNull()
            || !result.j0Shape || result.j0Shape->IsNull())
            return Fail(errorOutput, QStringLiteral("loader returned a null robot/J0 shape"));
        if (!result.statistics.detailedPresentationBuilt)
            return Fail(errorOutput, QStringLiteral("detailed presentation was not reported"));
    }
    if (!result.base.valid || !result.base.j0BoundsMm.valid
        || !result.statistics.assemblyBoundsMm.valid)
        return Fail(errorOutput, QStringLiteral("loader returned invalid assembly/base bounds"));
    if (result.statistics.jointComponentCount != 7
        || result.statistics.includedComponentCount != 7
        || result.statistics.includedPipelineCount != 0)
    {
        return Fail(errorOutput, QStringLiteral("default selection is not exactly J0-J6"));
    }
    if (result.statistics.sourceLengthUnits.isEmpty())
        return Fail(errorOutput, QStringLiteral("STEP source length unit was not reported"));
    if (result.statistics.sourceSha256
            != QStringLiteral("f8299f6deabc7b6f6ae799592f1f93e2366311d552bf91ac431eccec14bfcaa8")
        || (!boundsOnly && !result.statistics.displayTriangulationPrepared))
    {
        return Fail(errorOutput, QStringLiteral(
            "vendor STEP identity or background display cache is unexpected"));
    }
    if (!boundsOnly && (result.displayBlocks.size() < 2
        || result.statistics.displayBlockCount
            != static_cast<qsizetype>(result.displayBlocks.size())))
    {
        return Fail(errorOutput, QStringLiteral(
            "vendor B-Rep was not organized into progressive display blocks"));
    }
    qsizetype displayFaceTotal = 0;
    for (size_t blockIndex = 0; !boundsOnly && blockIndex < result.displayBlocks.size(); ++blockIndex)
    {
        if (!result.displayBlocks[blockIndex]
            || result.displayBlocks[blockIndex]->IsNull())
        {
            return Fail(errorOutput, QStringLiteral("progressive display block is null"));
        }
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(
            *result.displayBlocks[blockIndex], TopAbs_FACE, faces);
        if (faces.Extent() <= 0 || faces.Extent() > 300)
        {
            return Fail(errorOutput, QStringLiteral(
                "progressive display block is empty or exceeds 300 B-Rep faces"));
        }
        displayFaceTotal += static_cast<qsizetype>(faces.Extent());
    }
    if (!boundsOnly && displayFaceTotal != result.statistics.faceCount)
        return Fail(errorOutput, QStringLiteral("display blocks do not cover every B-Rep face"));
    if ((result.base.sourceUp - Eigen::Vector3d::UnitY()).norm() > 1.0e-12)
        return Fail(errorOutput, QStringLiteral("source up direction is not +Y"));

    std::array<bool, 7> foundJoint{};
    int discoveredPipelines = 0;
    int skippedComponents = 0;
    for (const auto& component : result.statistics.components)
    {
        if (component.jointIndex >= 0 && component.jointIndex <= 6)
        {
            foundJoint[static_cast<size_t>(component.jointIndex)] = true;
            if (!component.included || !component.boundsMm.valid
                || component.faceCount <= 0)
            {
                return Fail(errorOutput, QStringLiteral(
                    "included J%1 statistics/bounds are incomplete")
                    .arg(component.jointIndex));
            }
        }
        else if (!component.included)
        {
            ++skippedComponents;
            if (boundsOnly
                && (component.solidCount != 0 || component.faceCount != 0
                    || component.boundsMm.valid))
            {
                return Fail(errorOutput, QStringLiteral(
                    "non-included component still performed topology/bounds work"));
            }
        }
        if (component.isPipeline) ++discoveredPipelines;
    }
    if (discoveredPipelines != 1)
        return Fail(errorOutput, QStringLiteral("vendor pipeline component was not identified"));
    for (int jointIndex = 0; jointIndex <= 6; ++jointIndex)
    {
        if (!foundJoint[static_cast<size_t>(jointIndex)])
            return Fail(errorOutput, QStringLiteral("statistics omitted J%1").arg(jointIndex));
    }
    if (skippedComponents <= 0)
        return Fail(errorOutput, QStringLiteral("test STEP exposed no skipped component"));
    if (result.statistics.solidCount != 44 || result.statistics.faceCount != 26632)
        return Fail(errorOutput, QStringLiteral(
            "included J0-J6 topology count is not 44 solids / 26632 faces"));

    // 厂商 SA10-2000H 原始 STEP 的已核验毫米制边界。这里既验证装配位置被保留，
    // 也验证巨大的“3D工作空间图”没有进入默认输出。
    const auto& bounds = result.statistics.assemblyBoundsMm;
    constexpr double toleranceMm = 0.10;
    if (!Near(bounds.minimumMm.x(), -305.111766, toleranceMm)
        || !Near(bounds.minimumMm.y(), -178.670531, toleranceMm)
        || !Near(bounds.minimumMm.z(), -1520.713452, toleranceMm)
        || !Near(bounds.maximumMm.x(), 321.519974, toleranceMm)
        || !Near(bounds.maximumMm.y(), 1462.366241, toleranceMm)
        || !Near(bounds.maximumMm.z(), 369.080484, toleranceMm))
    {
        return Fail(errorOutput, QStringLiteral("J0-J6 bounds changed or workspace geometry leaked in"));
    }
    if (!Near(result.base.minimumYmm, -178.669083, toleranceMm))
        return Fail(errorOutput, QStringLiteral("J0 conservative base Y is unexpected"));

    output << "PASS: real SA10 STEP "
           << (boundsOnly ? "bounds-only" : "detailed") << " loaded in "
           << timer.elapsed() << " ms; components="
           << result.statistics.discoveredComponentCount << ", joints="
           << result.statistics.jointComponentCount << ", solids="
           << result.statistics.solidCount << ", faces=" << result.statistics.faceCount
           << ", display_blocks=" << result.statistics.displayBlockCount
           << ", units=" << result.statistics.sourceLengthUnits.join(',') << ", names="
           << result.statistics.productNameEncoding << Qt::endl;
    output << "AABB mm: min(" << bounds.minimumMm.x() << ',' << bounds.minimumMm.y() << ','
           << bounds.minimumMm.z() << ") max(" << bounds.maximumMm.x() << ','
           << bounds.maximumMm.y() << ',' << bounds.maximumMm.z() << ')' << Qt::endl;
    return 0;
}
