#pragma once

namespace WeldPoseValidationLimits
{
// 点云结果、焊道结构和最终补偿轨迹的固定生产硬门限。
// 这些值会进入质量证明阈值快照；修改后必须重新编译、发版并重建证明。
inline constexpr double kMinimumNonLapSegmentMm = 3.0;
inline constexpr double kMinimumLapOrEndpointAdjacentSegmentMm = 0.25;
inline constexpr double kMaxAdjacentPositionStepMm = 50.0;
inline constexpr double kMaxAdjacentControllerEulerStepDeg = 90.0;
inline constexpr double kMaxAdjacentPhysicalOrientationStepDeg = 90.0;
inline constexpr double kMinFinalToPreCompLengthRatio = 0.93;
inline constexpr double kMaxFinalToPreCompLengthRatio = 1.25;
inline constexpr double kMinFinalMatchedArcRatio = 0.90;
inline constexpr double kMinSourceUniqueCoverageRatio = 0.55;
inline constexpr double kMinSourceArcSpanRatio = 0.90;
inline constexpr double kMaxSourceDisplacementMm = 25.0;
inline constexpr double kMaxSourcePhysicalOrientationDeltaDeg = 60.0;
}
