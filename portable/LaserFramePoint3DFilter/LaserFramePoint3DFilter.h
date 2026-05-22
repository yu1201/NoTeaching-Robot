#pragma once

#include <opencv2/core.hpp>

#include <vector>

struct LaserFramePoint3DFilterOptions
{
    // 有效点少于该数量时不做滤波，避免小样本被误删。
    int minPointCount = 7;

    // 局部中值窗口半径，越大越平滑，但拐角附近更容易被拉平。
    int medianWindowRadius = 4;

    // 邻域支撑半径 = max(最小半径, 中位点距 * 倍率)，用于判断孤立点。
    double supportRadiusMinMm = 1.0;
    double supportRadiusStepScale = 5.0;

    // 只检查当前点前后多少个序号内的邻点，防止跨段误支撑。
    int supportIndexRadius = 6;

    // 前后端点保留范围和所需邻域支撑数；端点天然邻点少，所以阈值更低。
    int endpointPreserveCount = 2;
    int endpointMinSupportCount = 1;
    int middleMinSupportCount = 2;

    // 局部中值残差阈值 = max(最小阈值, 残差中位数 + MAD估计sigma * 倍率)。
    double residualThresholdMinMm = 2.0;
    double residualSigmaScale = 4.0;

    // 局部毛刺判断：邻域支撑很少且残差超过该阈值时删除。
    int localSpikeSupportMaxCount = 2;
    double localSpikeThresholdMinMm = 0.35;
    double localSpikeThresholdScale = 0.25;

    // 桥接毛刺过滤：用前后邻域中值连成短线，删除明显偏离这条线的单点/少量跳点。
    bool enableBridgeSpikeFilter = true;
    int bridgeInnerGap = 2;
    int bridgeOuterRadius = 6;
    int bridgeMinNeighborCount = 2;
    double bridgeDistanceThresholdMinMm = 0.45;
    double bridgeDistanceStepScale = 2.2;
    double bridgeMaxLengthMinMm = 3.0;
    double bridgeMaxLengthStepScale = 14.0;

    // 小簇过滤：删除滤波后仍残留的短小孤立点簇。
    // 主簇始终保留；其它副簇需要同时满足点数、主簇比例和物理跨度，避免几十个点的飞点簇残留。
    bool enableSmallClusterFilter = true;
    double clusterLinkRadiusMinMm = 1.8;
    double clusterLinkRadiusStepScale = 8.0;
    int clusterMinPointCount = 40;
    double clusterMinSizeRatioToLargest = 0.10;
    double clusterMinSpanMm = 25.0;

    // 小簇桥接保留：夹在两段有效线之间、点序连续且自身近似直线的小段保留。
    // 用于防止拐角附近短平台被小簇过滤误删，同时仍然删除前后没有有效线支撑的孤立噪点。
    bool enableSequenceBridgeClusterPreserve = true;
    int clusterBridgeMinPointCount = 7;
    double clusterBridgeMinSpanMm = 1.5;
    int clusterBridgeMaxIndexGap = 80;
    double clusterContinuityEndpointGapMinMm = 8.0;
    double clusterContinuityEndpointGapStepScale = 50.0;
    double clusterBridgeMaxAverageLineResidualMm = 1.5;
    double clusterBridgeMaxLineResidualMm = 3.0;

    // 初筛误删短线恢复：局部中值/邻域支撑可能在拐角短平台处误删一小段。
    // 只有当前后都有已保留轮廓、端点物理距离足够近、该段本身近似直线时才恢复。
    bool enableDeletedSequenceBridgeRestore = true;
    int deletedBridgeMinPointCount = 5;
    double deletedBridgeMinSpanMm = 1.0;
    int deletedBridgeMaxIndexGap = 80;
    double deletedBridgeEndpointGapMinMm = 6.0;
    double deletedBridgeEndpointGapStepScale = 25.0;
    double deletedBridgeMaxAverageLineResidualMm = 1.5;
    double deletedBridgeMaxLineResidualMm = 3.0;

    // 激光截面过滤使用的二维坐标轴。0=X, 1=Y, 2=Z。
    // 当前坡口相机单帧轮廓默认在 Y/Z 平面；如果相机坐标定义变化，可切到 X/Z 等组合。
    int profileAxis0 = 1;
    int profileAxis1 = 2;

    // 最终按激光截面连通域清理，专门删除离主轮廓很远的小噪点团。
    // 反光噪点有时会形成连续假线，默认只保留主轮廓；如果真实场景存在多条有效独立线，可打开 standalone 保留。
    bool enableProfileConnectedComponentFilter = true;
    double profileComponentLinkRadiusMinMm = 2.0;
    double profileComponentLinkRadiusStepScale = 10.0;
    bool profileComponentKeepStandalone = false;
    int profileComponentStandaloneMinPointCount = 80;
    double profileComponentStandaloneMinSizeRatioToLargest = 0.35;
    double profileComponentStandaloneMinSpanMm = 40.0;

    // 主轮廓链过滤：按点序把轮廓切成连续段，默认只保留最大连续链。
    // 用于删除局部点数不少、但和主激光轮廓不是同一条连续线的反光分支。
    bool enableProfileDominantChainFilter = true;
    double profileRunSplitGapMinMm = 0.8;
    double profileRunSplitGapStepScale = 8.0;
    double profileRunChainEndpointGapMinMm = 1.2;
    double profileRunChainEndpointGapStepScale = 12.0;
    bool profileRunKeepStandalone = false;
    int profileRunStandaloneMinPointCount = 120;
    double profileRunStandaloneMinSpanMm = 50.0;

    // 已知三段式轮廓专用：在截面平面里搜索近似直线段，只保留最长的2-3段。
    // 用于当前帧里存在反光短线段时，让短线不参与主轮廓判断。
    bool enableDominantLineSegmentFilter = false;
    int dominantLineMinSegmentCount = 2;
    int dominantLineMaxSegmentCount = 3;
    int dominantLineMinPointCount = 18;
    double dominantLineMinSpanMm = 5.0;
    double dominantLineDistanceThresholdMinMm = 0.45;
    double dominantLineDistanceThresholdStepScale = 2.8;
    double dominantLineSplitGapMinMm = 1.2;
    double dominantLineSplitGapStepScale = 8.0;
    // 快速候选线搜索的抽样点上限。数值越小越快，但极端点云下可能漏掉真实长线。
    int dominantLineFastSampleCount = 80;
    // 对抽样生成的粗候选线做精排的数量。数值越大越稳，但耗时会上升。
    int dominantLineFastCandidateCount = 64;

    // 三段折线回收：主线段确定后，先把三条趋势线延长到相邻交点形成折线。
    // 再从剩余点里捞回贴近折线、且投影落在折线段内的点。
    bool enableDominantLineTrendRecovery = true;
    double dominantLineTrendRecoverDistanceMinMm = 1.2;             //1.2
    double dominantLineTrendRecoverDistanceStepScale = 3.0;         //3.0
    double dominantLineTrendRecoverEndpointToleranceMm = 1;
};

// inputPoints: 一帧激光线三维点，要求按激光线顺序排列。
// options: 所有会影响滤波强度的参数，默认值适合当前坡口相机单帧毛刺过滤。
// return: 滤波后的三维点，可直接继续用于显示、拟合或保存。
std::vector<cv::Point3d> FilterSingleFrameLaserPoint3D(
    const std::vector<cv::Point3d>& inputPoints,
    const LaserFramePoint3DFilterOptions& options = LaserFramePoint3DFilterOptions());
