# MeasureThenWeldFilterFit

这是“先测后焊”流程里点云滤波、轨迹拟合和拐点生成的可移植版本。代码从当前工程的 `RobotCalculation`、`PointCloudExtractionProcessor` 和 `MeasureThenWeldService` 调用链整理而来，去掉了 Qt、Eigen 和主工程类型依赖，只保留 C++ 标准库。

## 文件

- `include/MeasureThenWeldFilterFit.h`：对外接口、参数、结果结构体。
- `src/MeasureThenWeldFilterFit.cpp`：滤波、拟合、拐点生成实现，关键步骤已加注释。
- `examples/example.cpp`：读取 `index x y z` 文本，输出 PreservePath、分类点、拐点和噪点文件。
- `CMakeLists.txt`：最小构建脚本。

## 对应当前主流程

主工程里的最新流程大致是：

1. 扫描生成 `PreciseLaserPoint.txt` 和 `PreciseLaserPoint_WorkpieceCloud.txt`。
2. `BuildOriginalTrackFitParams()` 生成默认参数：`sampleStep=2mm`、`piecewiseFitTolerance=4mm`、`piecewiseMinSegmentPoints=10`、`smoothRadius=3`。
3. 如果选择“立板投影到底板”，先调用 `ProjectWorkpieceCloudToLowerWeldPath()`，再进入特征分析。
4. 如果启用外部新版点云库，可把 DLL 返回的起点/终点/拐点轨迹交给 `ResampleTrackPoints()` 和 `BuildAnalysisFromExternalTrack()`。
5. 常规旧流程使用 `AnalyzeMeasureThenWeldPath()`，它会生成：
   - `filterResult.points`：PreservePath 轨迹点；
   - `classificationResult.points`：起点、终点、内拐点、外拐点和 2mm 普通焊道点；
   - `keyPoints`：只包含起点、终点和拐点。

## 特征点拟合方案

`FilterFitParams::geometryStrategy` 对应主程序“精测点云处理 / 特征点算法”里的方案切换：

- `LegacyGeometry`：旧版几何拟合，保留原 Douglas-Peucker 关键点和相邻段交点逻辑。
- `SlopeWaveFiltered`：方案一，先排除斜面中部波动点，再用旧版关键点逻辑。
- `RobustSegmentedKeys`：方案二，用中值 profile 做鲁棒分段关键点，降低局部波动误判拐点的概率。
- `WorkpieceProjection`：方案三/四类“立板投影到底板”预处理，先用 `ProjectWorkpieceCloudToLowerWeldPath()` 把完整工件点云投到底板焊道，再进入特征点分析。

`FilterFitParams::useSlopeConsistentCornerFit` 对应主程序里的“直线拟合排除圆弧段”开关。开启后，拐点交点不再直接用整段点拟合直线，而是：

1. 在相邻段内滑动窗口拟合局部方向；
2. 选出斜率方向一致、权重最大的直线核心窗口；
3. 排除靠近拐角的圆弧/过渡点后重拟合直线；
4. 两侧核心直线求交，并用局部点云距离和主带高度检查保护交点。

这个开关适合“直线段很明显，但圆弧/过渡点把交点拉偏”的现场数据；默认关闭，避免改变旧现场的结果。

## 拐点生成

`AnalyzeMeasureThenWeldPath()` 内部包含拐点生成逻辑：

1. 对有效点做局部中值去噪。
2. 沿扫描方向建立二维截面坐标 `(s, h)`，并平滑侧向高度 `h`。
3. 用 Douglas-Peucker 分段拟合生成关键点候选。
4. 对相邻分段做直线拟合，内点用两侧直线交点微调。
5. 通过当前关键点相对前后关键点的侧向凸起量判断拐点类型：
   - `3 inner_corner`
   - `4 outer_corner`
6. 保留起点/拐点/终点，并按 `sampleStep` 插入 `normal` 点，生成 2mm 焊接路径。

这部分对应主工程里 `AnalyzeMeasureThenWeldLowerWeldPathGeometry()`、`BuildGeometryKeyIndexes()`、`BuildFittedGeometryKeyPoints()` 和 `GeometryCornerType()` 的职责。

## 使用示例

```cpp
#include "MeasureThenWeldFilterFit.h"

std::vector<mtw_filter_fit::IndexedPoint3D> laserPoints = LoadYourPoints();

auto params = mtw_filter_fit::MeasureThenWeldDefaultParams(
    mtw_filter_fit::SampleAxis::AxisY,
    mtw_filter_fit::GeometryStrategy::LegacyGeometry);
// params.useSlopeConsistentCornerFit = true; // 需要排除圆弧/过渡段时再打开

mtw_filter_fit::AnalysisResult result =
    mtw_filter_fit::AnalyzeMeasureThenWeldPath(laserPoints, params);

if (result.ok) {
    auto preservePath = mtw_filter_fit::BuildFilterOutputLines(result.filterResult);
    auto classified = mtw_filter_fit::BuildClassifiedOutputLines(result.classificationResult);
    auto keyPoints = mtw_filter_fit::BuildKeyPointOutputLines(result.keyPoints);
}
```

如果当前配置选的是“立板投影到底板”：

```cpp
auto params = mtw_filter_fit::MeasureThenWeldDefaultParams(
    mtw_filter_fit::SampleAxis::AxisY,
    mtw_filter_fit::GeometryStrategy::WorkpieceProjection);

auto projected = mtw_filter_fit::ProjectWorkpieceCloudToLowerWeldPath(
    workpieceCloudPoints,
    seedLaserPathPoints,
    params);

if (projected.ok) {
    std::vector<mtw_filter_fit::IndexedPoint3D> projectedInput;
    for (const auto& p : projected.points) {
        projectedInput.push_back({p.index, p.point});
    }
    auto result = mtw_filter_fit::AnalyzeMeasureThenWeldPath(projectedInput, params);
}
```

如果外部 DLL 已经输出轨迹点：

```cpp
std::vector<mtw_filter_fit::ExternalTrackPoint> rawTrack = LoadDllTrackPoints();
auto track2mm = mtw_filter_fit::ResampleTrackPoints(rawTrack, 2.0);
auto result = mtw_filter_fit::BuildAnalysisFromExternalTrack(track2mm, originalPointCount, params);
```

## 输出格式

`BuildFilterOutputLines()` 输出与主工程 `PreciseLaserPoint_PreservePath_2mm.txt` 相同的基本格式：

```text
index x y z source
```

`BuildClassifiedOutputLines()` 输出：

```text
# index x y z type_code type_name source
# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise
```

`BuildKeyPointOutputLines()` 只输出起点、终点和拐点：

```text
# source_index x y z type_code type_name source
# 1=start 2=end 3=inner_corner 4=outer_corner
```

## 拟合调试导出 (CloudCompare)

把 `FilterFitParams::exportFitDebugCloud` 置 true 且设置 `fitDebugDir`（`example.cpp` 默认用 `<输出前缀>_FitDebug`），`AnalyzeMeasureThenWeldPath()` 会把每段直线拟合“用到的点集”和“拟合出的直线”导出成 CloudCompare 友好的 ASCII 点云，用来核对分段拟合是否正确。文件写到 `fitDebugDir/`：

- `fit_all_points.txt`：所有段的输入点，按段号上色 `X Y Z R G B segment dist_to_fit smoothN`，其中 `dist_to_fit` 是该点到本段拟合直线的垂距（在 CloudCompare 里按这个标量上色即可看出哪些点贴合、哪些被当作离群点甩开）。
- `fit_all_lines.txt`：每段拟合直线沿 s 密集采样并还原回 3D 的点，与点集同段同色 `X Y Z R G B segment`。
- `fit_keypoints.txt`：起点/终点/拐点（红色）`X Y Z R G B key_type`。
- `fit_axes.txt`：本次拟合的局部坐标系（质心 center 与 main/side/normal 三轴）。
- `segments/seg_XX_points.txt`、`segments/seg_XX_line.txt`：每段单独的输入点集与拟合直线。

CloudCompare 导入时把 `//` 开头的表头行当注释跳过，额外的 `segment`、`dist_to_fit` 等列会被识别为标量字段（scalar field）。主工程 `RobotCalculation` 侧有等价实现 `ExportGeometryFitDebugClouds()`，由“精测点云处理”界面的“导出拟合调试点云(CloudCompare)”开关控制（默认开启），导出到输出文件同目录的 `FitDebug/` 子目录。

## 构建

```powershell
cmake -S portable/MeasureThenWeldFilterFit -B portable/MeasureThenWeldFilterFit/build
cmake --build portable/MeasureThenWeldFilterFit/build --config Release
```

也可以直接把 `include/MeasureThenWeldFilterFit.h` 和 `src/MeasureThenWeldFilterFit.cpp` 拷到其它 C++17 工程里编译。

## 说明

- 这个包不包含机器人姿态插值、手眼变换、焊接姿态补偿和 STEP/焊接执行，只包含点云轨迹处理。
- `AnalyzeMeasureThenWeldPath()` 是当前先测后焊里 PreservePath 与拐点生成的主入口。
- `FilterLowerWeldPath()` 保留了旧的中值重采样滤波和直线/分段拟合模式，适合做调试或独立滤波测试。
- 外部新版点云库的 Windows DLL 加载代码没有放进来；移植时建议让业务层加载 DLL，再把返回轨迹点传给 `BuildAnalysisFromExternalTrack()`。
