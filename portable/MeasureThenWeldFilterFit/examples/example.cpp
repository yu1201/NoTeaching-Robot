#include "MeasureThenWeldFilterFit.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace
{
std::vector<mtw_filter_fit::IndexedPoint3D> LoadIndexedPointFile(const std::string& path)
{
    std::ifstream input(path);
    std::vector<mtw_filter_fit::IndexedPoint3D> points;
    std::string line;
    while (std::getline(input, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream parser(line);
        mtw_filter_fit::IndexedPoint3D point;
        if (parser >> point.index >> point.point.x >> point.point.y >> point.point.z)
        {
            points.push_back(point);
        }
    }
    return points;
}

bool SaveLines(const std::string& path, const std::vector<std::string>& lines)
{
    std::ofstream output(path);
    if (!output)
    {
        return false;
    }
    for (const std::string& line : lines)
    {
        output << line << '\n';
    }
    return true;
}
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: MeasureThenWeldFilterFitExample <input.txt> <output-prefix>\n";
        return 2;
    }

    const std::vector<mtw_filter_fit::IndexedPoint3D> input = LoadIndexedPointFile(argv[1]);
    mtw_filter_fit::FilterFitParams params =
        mtw_filter_fit::MeasureThenWeldDefaultParams(
            mtw_filter_fit::SampleAxis::AxisY,
            mtw_filter_fit::GeometryStrategy::LegacyGeometry);
    // 主程序里的“直线拟合排除圆弧段”开关。需要避免圆弧/过渡点拉偏直线交点时打开：
    // params.useSlopeConsistentCornerFit = true;

    const std::string prefix = argv[2];
    // 导出每段拟合“用到的点集”和“拟合出的直线”到 <prefix>_FitDebug 目录，便于拖入 CloudCompare 核对拟合。
    params.exportFitDebugCloud = true;
    params.fitDebugDir = prefix + "_FitDebug";

    // 这里对应先测后焊的 PreservePath + 拐点生成：
    // 先做局部去噪和几何关键点拟合，再把起点/拐点/终点按 2 mm 展开成焊接路径。
    const mtw_filter_fit::AnalysisResult analysis =
        mtw_filter_fit::AnalyzeMeasureThenWeldPath(input, params);
    if (!analysis.ok)
    {
        std::cerr << "Analyze failed: " << analysis.error << '\n';
        return 1;
    }

    SaveLines(prefix + "_PreservePath_2mm.txt",
        mtw_filter_fit::BuildFilterOutputLines(analysis.filterResult));
    SaveLines(prefix + "_Classified.txt",
        mtw_filter_fit::BuildClassifiedOutputLines(analysis.classificationResult));
    SaveLines(prefix + "_KeyPoints.txt",
        mtw_filter_fit::BuildKeyPointOutputLines(analysis.keyPoints));
    SaveLines(prefix + "_Noise.txt",
        mtw_filter_fit::BuildNoiseOutputLines(input, analysis.filterResult));

    std::cout << "ok: fitted=" << analysis.filterResult.points.size()
              << " classified=" << analysis.classificationResult.points.size()
              << " keyPoints=" << analysis.keyPoints.size() << '\n';
    return 0;
}
