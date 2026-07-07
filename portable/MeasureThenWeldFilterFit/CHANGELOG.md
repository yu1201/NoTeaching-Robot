# 更新日志 / CHANGELOG

本模块是主工程 `RobotCalculation` / `MeasureThenWeldService` 里「先测后焊」点云滤波·轨迹拟合·拐点生成的可移植抽取版。本文件记录与主程序算法的同步。

## 2026-06-30 — 同步当前波纹板拐点管线

把主程序 `AnalyzeMeasureThenWeldLowerWeldPathGeometry()` 当前在用、但本模块此前缺失的整条波纹板拐点管线移植进来，并把 `AnalyzeMeasureThenWeldPath()` 改成同样的调用顺序。

### 新增依赖

- **Eigen3（header-only）**：用于 (s, smoothH) 平面上的 2D 线性代数（段直线 PCA 拟合、方向夹角）。此前为「纯标准库」，现新增此唯一第三方依赖（仍无 Qt）。
- **Eigen 头文件随附在 `third_party/eigen/`**（连同 MPL2 等许可证文件），整个模块目录拷走即可直接编译，无需另装 Eigen / 无需联网 / 无需传 `-DEIGEN3_INCLUDE_DIR`。CMake 默认用该随附副本，也可显式传 `-DEIGEN3_INCLUDE_DIR` 或装系统 Eigen3 改用系统版。

### 新增算法（与主程序逐函数对齐，字段/逻辑一致）

- `BuildAzimuthCornerKeyIndexes` + `SplitNonStraightAzimuthSegments`：**方位角拐点检测**（已去噪输入路径，替代 Douglas-Peucker）——局部最小二乘拟合左右方向、判方向转折角、按 |转角| 降序 NMS；直线化兜底补回漏检拐点。
- `RefineCornersByCoherentBow`：起终点先验自适应细化（相干弓补点；默认关，已被平台重算取代）。
- `DetectLapMisalignmentBoundaries`：板材搭接 X 错位台阶检测（双侧平台最小二乘判据 + 沿主轴 NMS）。
- `FitPlatformTwoCorners` + `RefitCornersByPlatformPattern`：**平台重算**（II/OO 类型游程归平台，每平台三段拟合重算 2 个边界角；对源头多检/漏检免疫，默认开）。
- `RecoverEndRegionCornersByPeriod`：**端区周期一致性补拐点**（中段中位段长=周期，反推端区盲区漏点 + 弯折角确认）。
- `MergeTooCloseSameTypeCorners`：**周期/角度删错拐点**，且**分平台**——两同类拐点之间是平台（真平台）绝不删，仅之间是坡（假双拐点）才删、并保留贴平台的边界角。
- `SnapCornersToPlatforms`：**按平台边界重定拐点**——检测平台、把拐点归位到平台两端、删掉卡在平台正中间（放错位）的角；治“拐点放平台中间→平台塌成长坡消失”。对已正确平台幂等不动。
- `TrimLowerWeldPathByArcLength`：基础焊道首尾段截断（拟合提取关键点之前执行）。
- `BilateralPresmoothSdkBaseWeld`：SDK 基础焊道「结构自适应 1D 双边」预平滑去锯齿（几何拟合之前的**可选预处理**，独立函数、不在 `AnalyzeMeasureThenWeldPath` 管线内，由调用方在拟合前手动调用）。垂直偏移值域核保住搭接台阶/折角，仅用 `Point3D` 运算无需 Eigen。
- 共享原语：`FitGeometrySegmentLine`（段直线 PCA）、`GeometryBendAngleDegAt`（航向弯折角）、`LocalLateralSlopeAt`（侧向局部斜率）。

### `FilterFitParams` 新增字段（与主程序 `LowerWeldFilterParams` 同名）

`inputAlreadyDenoised`；`azimuth*`（4 项）；`cornerRefineEnable` + 相干弓 4 项；`cornerPatternRefitEnable` + `cornerPlatformMinSegPoints`；`enableLapMisalignmentSplit` + `lapStep*`（4 项）；`enableEdgeTruncate` + `truncate*`；`enableEndPeriodCornerRecover` + `endPeriod*`（3 项）；`enablePlatformCornerSnap` + `platformSnap*`（2 项）。默认值与主程序一致（多数功能默认关，平台重算默认开）。

### orchestrator 变化

`AnalyzeMeasureThenWeldPath()` 现按主程序顺序：有效点 → 首尾截断 → 去噪/投影（`inputAlreadyDenoised` 时跳过、不平滑）→ 关键点检测（方位角 或 DP）→ 相干弓 → 搭接注入 → 平台重算 → 端区周期补漏/删错 → 平台重定 → 求交拟合 → 分类 + 2mm 展开。`BuildFittedKeyPoints` 增加可选 `isLapStepKey` 参数（搭接台阶端点取原始投影，保留干净 X 台阶）。

### 验证

- `Debug`/`Release` 经 CMake（MSVC 2022 + Eigen 3.4）编译通过，无错误无警告。
- 端到端：现场基础焊道 `RobotC/20260628_016` 的 `PreciseLaserPoint_SdkBase.txt`（3328 点），以 SdkBaseWeldFit 等效参数（`inputAlreadyDenoised` + 平台重算 + 端区周期 + 平台重定）运行，输出 **27 个关键点**、末端两个平台正确恢复（拐点 idx 2756/2883/3036/3159），与主程序一致。
- 自包含回归测试 `examples/regression_test.cpp`（CMake 目标 `MeasureThenWeldFilterFitRegression`，注册为 ctest）：合成 6 周期波纹板跑全管线，断言「II/OO 平台成对（同类游程≤2）、主轴单调、无平台塌陷」（26 关键点、12 内角 + 12 外角）；并测双边预平滑「锯齿总变差降 65%」「3mm 真实台阶完整保住」。`ctest -C Release` 全通过。

### 未包含

- 外部点云 DLL 加载、机器人姿态/手眼/焊接补偿与 STEP 执行（本模块始终只做点云轨迹处理）。
