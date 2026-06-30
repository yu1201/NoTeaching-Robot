# MeasureThenWeldFilterFit

这是“先测后焊”流程里点云滤波、轨迹拟合和拐点生成的可移植版本。代码从当前工程的 `RobotCalculation`、`PointCloudExtractionProcessor` 和 `MeasureThenWeldService` 调用链整理而来，去掉了 Qt 和主工程类型依赖。

> **2026-06 已同步主程序当前在用的波纹板拐点管线**（方位角拐点检测、相干弓补点、搭接错位检测、平台重算、端区周期补漏/删错、按平台边界重定拐点、基础焊道首尾截断）。详见 [CHANGELOG.md](CHANGELOG.md) 与下方「当前几何拐点管线」。
>
> **依赖**：本模块用 **Eigen3（header-only）** 做 (s, smoothH) 平面上的 2D 线性代数（直线 PCA / 方向夹角），这是唯一的第三方依赖，无需 Qt；其余只用 C++17 标准库。**Eigen 头文件已随附在 `third_party/eigen/` 下**——整个 `MeasureThenWeldFilterFit/` 目录拷到别处即可直接编译，无需另装 Eigen、无需联网、无需传 `-DEIGEN3_INCLUDE_DIR`。CMake 默认就用这份随附副本（也可显式传 `-DEIGEN3_INCLUDE_DIR` 或装系统 Eigen3 改用系统版）。

## 文件

- `include/MeasureThenWeldFilterFit.h`：对外接口、参数、结果结构体。
- `src/MeasureThenWeldFilterFit.cpp`：滤波、拟合、拐点生成实现，每个管线函数都有详细注释。
- `examples/example.cpp`：读取 `index x y z` 文本，输出 PreservePath、分类点、拐点和噪点文件。
- `examples/regression_test.cpp`：自包含端到端回归测试（合成波纹 + 双边预平滑），注册为 ctest。
- `TOPOLOGY.md`：函数调用拓扑（Mermaid 流程图 + ASCII 调用树 + 共享原语表）。
- `docs/function_topology.svg`：函数拓扑渲染图（独立 SVG，自带样式白底，浏览器/查看器直接打开）。
- `CHANGELOG.md`：算法同步与版本变更记录。
- `third_party/eigen/`：随附的 Eigen3 头文件（唯一第三方依赖，header-only）。
- `CMakeLists.txt`：最小构建脚本（默认用随附 Eigen，可一键 cmake/ctest）。

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

## 当前几何拐点管线（与主程序同步）

`AnalyzeMeasureThenWeldPath()` 现在按主程序 `AnalyzeMeasureThenWeldLowerWeldPathGeometry()` 的顺序串起整条波纹板拐点管线：

1. **有效点筛选**（去 NaN）。
2. **基础焊道首尾截断**（`TrimLowerWeldPathByArcLength`，可选）：按点列数组序沿累积弧长截掉开头/结尾指定 mm 的点，剔除扫描进/出端坏点。**在去噪/拟合之前**执行。
3. **去噪与投影**：
   - `inputAlreadyDenoised=false`：MAD 局部去噪 + 分支离群剔除 + 投影平滑（旧行为）。
   - `inputAlreadyDenoised=true`（SDK `is_remove_noise` 已开，基础焊道已干净）：**全部跳过**（重复处理会削圆尖角、移位拐点），投影不平滑。
4. **关键点检测**（两条路径）：
   - `inputAlreadyDenoised=true` → **方位角法** `BuildAzimuthCornerKeyIndexes`（局部最小二乘拟合左右方向、判方向转折角、NMS）+ `SplitNonStraightAzimuthSegments`（直线化兜底补漏检拐点）。比 Douglas-Peucker「离弦最远点」更能区分真折角与波纹起伏。
   - 否则 → 旧 **Douglas-Peucker** `BuildGeometryKeyIndexes` + `PruneShortSameTypeRuns`。
5. **相干弓补点** `RefineCornersByCoherentBow`（两路径通用；默认关，已被平台重算取代）。
6. **搭接错位检测** `DetectLapMisalignmentBoundaries`（可选）：双侧平台最小二乘判据找 X 错位台阶，台阶端点作硬段边界注入 `keyIndexes` 并以 `isLapStepKey` 标记；台阶端点在拟合时直接取原始投影、保留干净 X 台阶。
7. **平台重算** `RefitCornersByPlatformPattern`（默认开）：按 II/OO 类型游程把拐点归平台，每平台从稠密路径三段（坡-平台-坡）拟合重算恰好 2 个边界角——对源头多检（5→2/3→2）/漏检（1→2）一套覆盖。
8. **端区周期补漏 + 删错**（可选，`enableEndPeriodCornerRecover`）：
   - `RecoverEndRegionCornersByPeriod`：中段中位段长 L=周期；端段长 ≥ `ratio*L` 则按周期反推漏点位置、用弯折角确认补回（治起/终段盲区漏拐点）。
   - `MergeTooCloseSameTypeCorners`：相邻**同类**拐点间距 ≪ 周期且**之间是坡**（假双拐点）才合并；**之间是平台（真平台）绝不删**，且删两侧都陡的“坡中”那个、保贴平台的边界角。
9. **按平台边界重定拐点** `SnapCornersToPlatforms`（可选，`enablePlatformCornerSnap`）：检测平的段（平台），保证每平台两端各有边界角、删掉卡在平台正中间（放错位）的角。治“拐点放平台中间→平台塌成长坡消失”。对已正确平台幂等不动。
10. **关键点求交拟合** `BuildFittedKeyPoints` → 分类 + 2mm 展开。

### 新增参数（字段名与主程序 `LowerWeldFilterParams` 一一对应）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `inputAlreadyDenoised` | false | 置 true → 跳过自身去噪/平滑、关键点改用方位角法（SdkBaseWeldFit 路径） |
| `azimuthHeadingWindow` / `azimuthTurnThresholdDeg` / `azimuthNmsSpanMm` / `azimuthStraightenResidualMm` | 10 / 22° / 12mm / 6mm | 方位角拐点检测 |
| `cornerRefineEnable` + `azimuthRefineFloorMm` / `cornerRefineOneSidedFrac` / `cornerRefineMidMultiple` / `cornerRefineEndFrac` | false | 相干弓补点（默认关） |
| `cornerPatternRefitEnable` + `cornerPlatformMinSegPoints` | true / 8 | 平台重算（默认开） |
| `enableLapMisalignmentSplit` + `lapStepHeightThresholdMm` / `lapStepStationWindowMm` / `lapStepSideFlatnessMm` / `lapStepPlatformSlopeMax` | false | 搭接错位检测 |
| `enableEdgeTruncate` + `truncateHeadMm` / `truncateTailMm` | false / 0 / 0 | 首尾截断 |
| `enableEndPeriodCornerRecover` + `endPeriodRatioThreshold` / `endPeriodMinBendDeg` / `endPeriodMergeFrac` | false / 1.2 / 5° / 0.4 | 端区周期补漏 + 删错 |
| `enablePlatformCornerSnap` + `platformSnapFlatSlope` / `platformSnapMinFrac` | false / 0.15 / 0.25 | 按平台边界重定拐点 |

### 复现主程序 SdkBaseWeldFit（SDK 点云+拟合）

```cpp
auto params = mtw_filter_fit::MeasureThenWeldDefaultParams(mtw_filter_fit::SampleAxis::AxisY);
params.inputAlreadyDenoised      = true;   // SDK 已去噪 → 方位角法 + 跳过自身去噪
params.cornerPatternRefitEnable  = true;   // 平台重算（默认已开）
params.enableEndPeriodCornerRecover = true;
params.enablePlatformCornerSnap  = true;   // 治“平台消失”
// 如需截掉扫描进/出端坏点：
// params.enableEdgeTruncate = true; params.truncateHeadMm = params.truncateTailMm = 10.0;
auto result = mtw_filter_fit::AnalyzeMeasureThenWeldPath(sdkBasePoints, params);
```

> 在现场 `20260628_016` 基础焊道上，该配置与主程序输出一致：27 个关键点、末端两个平台正确恢复、内/外角严格交替（见 CHANGELOG）。

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
# Eigen 已随附在 third_party/eigen/，无需任何额外参数即可配置+构建：
cmake -S portable/MeasureThenWeldFilterFit -B portable/MeasureThenWeldFilterFit/build
cmake --build portable/MeasureThenWeldFilterFit/build --config Release
ctest --test-dir portable/MeasureThenWeldFilterFit/build -C Release   # 跑自包含回归测试
# 想改用系统/别处的 Eigen，再显式传： -DEIGEN3_INCLUDE_DIR=<含 Eigen 子目录的那一层>
```

也可以直接把 `include/`、`src/` 和 `third_party/eigen/` 三者拷到其它 C++17 工程里编译，只需让编译器能 `#include <Eigen/Dense>`（把 `third_party/eigen` 加进 include 路径即可，Eigen 是 header-only、无需链接）。

## 说明

- 这个包不包含机器人姿态插值、手眼变换、焊接姿态补偿和 STEP/焊接执行，只包含点云轨迹处理。
- `AnalyzeMeasureThenWeldPath()` 是当前先测后焊里 PreservePath 与拐点生成的主入口，已串起完整波纹板拐点管线（见上）。`examples/example.cpp` 默认走 DP 路径；要走 SdkBase 方位角路径，按上面示例置 `inputAlreadyDenoised=true` 等开关。
- `FilterLowerWeldPath()` 保留了旧的中值重采样滤波和直线/分段拟合模式，适合做调试或独立滤波测试。
- 外部新版点云库的 Windows DLL 加载代码没有放进来；移植时建议让业务层加载 DLL，再把返回轨迹点传给 `BuildAnalysisFromExternalTrack()`。
- **SdkBase 基础焊道双边预平滑去锯齿**已移植为独立函数 `BilateralPresmoothSdkBaseWeld()`——它是几何拟合之前对原始 SdkBase 点云的**可选预处理**（不在 `AnalyzeMeasureThenWeldPath` 管线内），由调用方在拟合前手动调用：`BilateralPresmoothSdkBaseWeld(pts, 3.0, 0.5); auto r = AnalyzeMeasureThenWeldPath(pts, params);`。结构自适应、保住搭接台阶/折角。
- 搭接台阶端点在本包用原始投影近似（主程序用本侧段直线在台阶 s 处取值），默认关时不影响。
- **回归测试**：`examples/regression_test.cpp`（CMake 目标 `MeasureThenWeldFilterFitRegression`，已注册为 ctest）自包含——合成波纹板跑全管线断言「II/OO 平台成对、主轴单调、无平台塌陷」，外加双边预平滑「去锯齿 + 保台阶」。运行：`ctest -C Release` 或直接跑该 exe。
