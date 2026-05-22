# LaserFramePoint3DFilter

这个文件夹是当前帧激光三维点云滤波的可移植版本。

## 需要拷贝的文件

- `LaserFramePoint3DFilter.h`
- `LaserFramePoint3DFilter.cpp`

## 依赖

不依赖 Qt 或本项目其它文件。函数入参直接使用 `std::vector<cv::Point3d>`，因此需要 OpenCV core 头文件和 C++ 标准库：

- `<opencv2/core.hpp>`
- `<vector>`
- `<algorithm>`
- `<cmath>`
- `<cstddef>`

建议使用 C++11 或更高版本编译。

## 输入输出

输入是按激光线顺序排列的一帧三维点，可直接传相机帧里的 `allResultPoint`：

```cpp
std::vector<cv::Point3d> allResultPoint;
```

输出是滤波后的 `std::vector<cv::Point3d>`。滤波逻辑包含非有限值剔除、局部中值残差剔除、局部连续支撑剔除、前后邻域桥接毛刺剔除、初筛误删短线恢复、小簇过滤和主连续轮廓筛选，可过滤单点、少量跳出主激光线的孤立噪点，以及反光形成的独立连续假线。

短线恢复只会恢复夹在两段已保留主轮廓之间、点序连续、端点物理距离合理且自身近似直线的小段，用来防止拐角附近的短平台被误删。小簇过滤会始终保留主轮廓；默认不保留独立副轮廓，避免反光噪点形成连续假线后残留。如果真实场景存在多条互相断开的有效激光线，可把 `profileComponentKeepStandalone` 和 `profileRunKeepStandalone` 改为 `true`。

已知是三段式轮廓时，可打开 `enableDominantLineSegmentFilter`。这个模式不依赖输入点序连续，而是在截面二维平面里按空间距离搜索最长的 2-3 条直线段，适合原始帧中间区域被反光点穿插、点序来回跳的情况。主线段确定后，默认会再按 `enableDominantLineTrendRecovery` 把三条趋势线延长到相邻交点，形成折线，然后只从剩余点里捞回贴近折线且投影落在折线段内的点，避免拐角附近整团点被直接删掉，也避免超出折线端点的点被误回收。

三段模式的快速搜索由 `dominantLineFastSampleCount` 和 `dominantLineFastCandidateCount` 控制。前者限制抽样点数量，后者限制进入精排的候选线数量；数值越小越快，数值越大越稳。当前默认值按 60FPS 预览优先设置，疑难点云如果漏线，可以先提高 `dominantLineFastSampleCount`。

## 使用示例

不需要额外包装结构体，直接传 `std::vector<cv::Point3d>`：

```cpp
LaserFramePoint3DFilterOptions options;
options.localSpikeThresholdMinMm = 0.35;       // 单点毛刺最小残差阈值，越小过滤越强
options.bridgeDistanceThresholdMinMm = 0.45;   // 前后邻域桥接距离阈值，越小过滤越强
options.supportRadiusStepScale = 5.0;          // 邻域支撑半径倍率，越大越容易保留连续点
options.deletedBridgeEndpointGapMinMm = 6.0;   // 恢复误删短线时，前后主轮廓端点允许的最小连接距离
options.clusterBridgeMaxIndexGap = 80;         // 小簇桥接保留时，前后主轮廓允许的最大点序间隔
options.profileComponentKeepStandalone = false;// 默认删除独立反光分支
options.profileRunKeepStandalone = false;      // 默认只保留最大连续激光线
options.enableDominantLineSegmentFilter = true;// 已知三段式轮廓时，只保留最长的2-3段主直线
options.dominantLineFastSampleCount = 80;      // 快速候选线抽样点上限，越小越快
options.dominantLineFastCandidateCount = 64;   // 进入精排的候选线数量，越大越稳
options.enableDominantLineTrendRecovery = true;// 按三段交点折线，回收贴近折线内部的点

std::vector<cv::Point3d> filtered = FilterSingleFrameLaserPoint3D(allResultPoint, options);
```
