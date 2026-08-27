#include "PlatformSemanticValidator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct PoseRow
{
    int rawIndex = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double path = 0.0;
    std::string kind;
    bool lapStep = false;
};

std::string NormalizeKind(std::string kind)
{
    const std::vector<std::string> suffixes{"_transition", "_arc"};
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (const std::string& suffix : suffixes)
        {
            if (kind.size() > suffix.size()
                && kind.compare(kind.size() - suffix.size(), suffix.size(), suffix) == 0)
            {
                kind.erase(kind.size() - suffix.size());
                changed = true;
            }
        }
    }
    return kind;
}

double Distance(const PoseRow& a, const PoseRow& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        std::cerr << "usage: replay <weld-pose.txt> <depth-axis-x> <depth-axis-y> <depth-axis-z>\n";
        return 2;
    }
    double axisX = std::stod(argv[2]);
    double axisY = std::stod(argv[3]);
    double axisZ = std::stod(argv[4]);
    const double axisNorm = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (!std::isfinite(axisNorm) || axisNorm <= 1e-9)
    {
        std::cerr << "invalid depth axis\n";
        return 2;
    }
    axisX /= axisNorm;
    axisY /= axisNorm;
    axisZ /= axisNorm;

    std::ifstream input(argv[1]);
    if (!input)
    {
        std::cerr << "cannot open weld pose file\n";
        return 2;
    }
    std::string line;
    std::getline(input, line);
    std::vector<PoseRow> rows;
    while (std::getline(input, line))
    {
        std::istringstream stream(line);
        int weldIndex = 0;
        PoseRow row;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        std::string pointType;
        int lap = 0;
        if (!(stream >> weldIndex >> row.rawIndex
              >> row.x >> row.y >> row.z
              >> rx >> ry >> rz >> bx >> by >> bz
              >> pointType >> row.kind >> lap))
        {
            continue;
        }
        row.kind = NormalizeKind(row.kind);
        row.lapStep = lap != 0;
        if (!rows.empty())
        {
            row.path = rows.back().path + Distance(rows.back(), row);
        }
        rows.push_back(row);
    }
    if (rows.size() < 2)
    {
        std::cerr << "weld pose file has insufficient rows\n";
        return 2;
    }

    std::vector<PlatformSemanticValidator::AssignedSegment> segments;
    std::size_t begin = 0;
    while (begin < rows.size())
    {
        std::size_t end = begin;
        bool lapStep = rows[begin].lapStep;
        while (end + 1 < rows.size() && rows[end + 1].kind == rows[begin].kind)
        {
            ++end;
            lapStep = lapStep || rows[end].lapStep;
        }
        const PoseRow& a = rows[begin];
        const PoseRow& b = rows[end];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        const double depthBegin = a.x * axisX + a.y * axisY + a.z * axisZ;
        const double depthEnd = b.x * axisX + b.y * axisY + b.z * axisZ;
        const double depthDelta = depthEnd - depthBegin;
        const double lengthSquared = dx * dx + dy * dy + dz * dz;
        segments.push_back({
            a.rawIndex,
            b.rawIndex,
            a.path,
            b.path,
            depthBegin,
            depthEnd,
            std::sqrt(std::max(0.0, lengthSquared - depthDelta * depthDelta)),
            a.kind,
            lapStep,
            begin == 0 || end + 1 == rows.size()
        });
        begin = end + 1;
    }

    const auto result = PlatformSemanticValidator::EvaluateAssignedSegments(segments, 0.15);
    for (const auto& segment : segments)
    {
        const double depthDelta = segment.depthEnd - segment.depthBegin;
        const double slope = segment.lateralSpan > 1e-9
            ? std::abs(depthDelta) / segment.lateralSpan : 0.0;
        std::cout << "FIELD_SEGMENT raw=" << segment.beginRawIndex
                  << "->" << segment.endRawIndex
                  << " kind=" << segment.kind
                  << " depth_delta=" << depthDelta
                  << " slope=" << slope
                  << " endpoint=" << (segment.endpointSegment ? 1 : 0)
                  << " lap=" << (segment.lapStep ? 1 : 0) << '\n';
    }
    std::cout << "FIELD_REPLAY valid=" << (result.valid ? 1 : 0)
              << " evidence=" << (result.sufficientEvidence ? 1 : 0)
              << " segments=" << result.checkedSegmentCount
              << " platforms=" << result.platformSegmentCount
              << " slopes=" << result.slopeSegmentCount
              << " low_high_mm=" << result.lowHighSeparation
              << " rms_mm=" << result.residualRms
              << " max_mm=" << result.maxResidual << '\n';
    std::cout << "FIELD_SEQUENCE " << result.fingerprint << '\n';
    for (const std::string& failure : result.failures)
    {
        std::cout << "FIELD_FAILURE " << failure << '\n';
    }
    return result.valid && result.sufficientEvidence ? 0 : 1;
}
