#pragma once

#include "Const.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// Weld-process values can come from the editor, the scoped settings database,
// or the legacy tab-separated mirror.  Keep one fail-closed contract for all
// three paths and for the final STEP move metadata.
namespace WeldProcessValidation
{
constexpr double kUiDoubleMin = -999999.0;
constexpr double kUiDoubleMax = 999999.0;
constexpr double kOverlapMin = 0.0;
constexpr double kOverlapMax = 200.0;
constexpr double kFinalStepMinMm = 2.0;
constexpr double kFinalStepMaxMm = 100.0;
constexpr double kTransitionRadiusMinMm = 2.0;
constexpr int kMaximumStoredProcessEntries = 4096;

namespace Detail
{
inline std::string FormatValue(double value)
{
    if (std::isnan(value))
    {
        return "nan";
    }
    if (std::isinf(value))
    {
        return value < 0.0 ? "-inf" : "inf";
    }
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

inline void AddError(
    std::vector<std::string>& errors,
    const std::string& field,
    double value,
    const std::string& allowed)
{
    errors.push_back("字段 " + field + "=" + FormatValue(value)
        + " 无效，允许范围 " + allowed + "。");
}

inline void CheckClosedRange(
    std::vector<std::string>& errors,
    const std::string& field,
    double value,
    double minimum,
    double maximum)
{
    if (!std::isfinite(value) || value < minimum || value > maximum)
    {
        AddError(errors, field, value,
            "[" + FormatValue(minimum) + "," + FormatValue(maximum) + "]");
    }
}

inline void CheckPositiveRange(
    std::vector<std::string>& errors,
    const std::string& field,
    double value,
    double maximum)
{
    if (!std::isfinite(value) || value <= 0.0 || value > maximum)
    {
        AddError(errors, field, value,
            "(0," + FormatValue(maximum) + "]");
    }
}

inline void CheckIntegerRange(
    std::vector<std::string>& errors,
    const std::string& field,
    int value,
    int minimum,
    int maximum)
{
    if (value < minimum || value > maximum)
    {
        errors.push_back("字段 " + field + "=" + std::to_string(value)
            + " 无效，允许范围 [" + std::to_string(minimum) + ","
            + std::to_string(maximum) + "]。");
    }
}

inline bool IsSupportedWeaveShape(int value)
{
    return value == 0 || (value >= 5 && value <= 18);
}

inline std::string JoinErrors(const std::vector<std::string>& errors)
{
    std::ostringstream out;
    for (size_t index = 0; index < errors.size(); ++index)
    {
        if (index > 0)
        {
            out << ' ';
        }
        out << errors[index];
    }
    return out.str();
}

inline void CollectWeaveErrors(
    const T_WeaveDate& weave,
    const std::string& prefix,
    std::vector<std::string>& errors)
{
    CheckIntegerRange(errors, prefix + "WeaveType", weave.nWeaveType, 0, 2);
    if (!IsSupportedWeaveShape(weave.nWeaveShape))
    {
        errors.push_back("字段 " + prefix + "WeaveShape="
            + std::to_string(weave.nWeaveShape)
            + " 无效，只允许控制器定义的 {0,5..18}。");
    }
    CheckIntegerRange(errors, prefix + "PauseContinue", weave.nPauseContinue, 0, 1);
    CheckClosedRange(errors, prefix + "WeaveFrequencyHz",
        weave.dWeaveFrequencyHz, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "WeaveAmplitudeMm",
        weave.dWeaveAmplitudeMm, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "SwingDirectionDeg",
        weave.dSwingDirectionDeg, kUiDoubleMin, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "WeavePlaneAngleDeg",
        weave.dWeavePlaneAngleDeg, kUiDoubleMin, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "SpaceAngleDeg",
        weave.dSpaceAngleDeg, kUiDoubleMin, kUiDoubleMax);
    CheckIntegerRange(errors, prefix + "PauseTime1Ms", weave.nPauseTime1Ms, 0, 999999);
    CheckIntegerRange(errors, prefix + "PauseTime2Ms", weave.nPauseTime2Ms, 0, 999999);
    CheckIntegerRange(errors, prefix + "PauseTime3Ms", weave.nPauseTime3Ms, 0, 999999);
    CheckIntegerRange(errors, prefix + "PauseTime4Ms", weave.nPauseTime4Ms, 0, 999999);
    CheckClosedRange(errors, prefix + "EndLengthMm",
        weave.dEndLengthMm, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "EndWidthMm",
        weave.dEndWidthMm, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "CenterHeightMm",
        weave.dCenterHeightMm, kUiDoubleMin, kUiDoubleMax);
}

inline void CollectTrackErrors(
    const T_TrackData& track,
    const std::string& prefix,
    std::vector<std::string>& errors,
    bool actualWeld)
{
    const auto checkDouble = [&](const char* field, double value)
        {
            CheckClosedRange(errors, prefix + field, value, kUiDoubleMin, kUiDoubleMax);
        };
    checkDouble("LateralGain", track.dLateralGain);
    checkDouble("LeftAreaCoefficient", track.dLeftAreaCoefficient);
    checkDouble("RightAreaCoefficient", track.dRightAreaCoefficient);
    CheckClosedRange(errors, prefix + "VerticalReferenceCurrent",
        track.dVerticalReferenceCurrent, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "VerticalCycleLength",
        track.dVerticalCycleLength, 0.0, kUiDoubleMax);
    checkDouble("VerticalGain", track.dVerticalGain);
    CheckClosedRange(errors, prefix + "LateralMinCompPerCycle",
        track.dLateralMinCompPerCycle, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "LateralMaxCompPerCycle",
        track.dLateralMaxCompPerCycle, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "LateralMaxCompTotal",
        track.dLateralMaxCompTotal, 0.0, kUiDoubleMax);
    checkDouble("LateralAsymmetryCoefficient", track.dLateralAsymmetryCoefficient);
    checkDouble("LateralReserved6", track.dLateralReserved6);
    checkDouble("LateralReserved5", track.dLateralReserved5);
    checkDouble("LateralReserved4", track.dLateralReserved4);
    checkDouble("LateralReserved3", track.dLateralReserved3);
    checkDouble("LateralReserved2", track.dLateralReserved2);
    checkDouble("LateralReserved1", track.dLateralReserved1);
    CheckClosedRange(errors, prefix + "VerticalMinCompPerCycle",
        track.dVerticalMinCompPerCycle, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "VerticalMaxCompPerCycle",
        track.dVerticalMaxCompPerCycle, 0.0, kUiDoubleMax);
    CheckClosedRange(errors, prefix + "VerticalMaxCompTotal",
        track.dVerticalMaxCompTotal, 0.0, kUiDoubleMax);
    checkDouble("VerticalAsymmetryCoefficient", track.dVerticalAsymmetryCoefficient);
    checkDouble("VerticalReserved6", track.dVerticalReserved6);
    checkDouble("VerticalReserved5", track.dVerticalReserved5);
    checkDouble("VerticalReserved4", track.dVerticalReserved4);
    checkDouble("VerticalReserved3", track.dVerticalReserved3);
    checkDouble("VerticalReserved2", track.dVerticalReserved2);
    checkDouble("VerticalReserved1", track.dVerticalReserved1);

    if (actualWeld)
    {
        CheckIntegerRange(errors, prefix + "VerticalModeFlag", track.nVerticalModeFlag, 0, 1);
        CheckIntegerRange(errors, prefix + "TimeOrDistanceMode", track.nTimeOrDistanceMode, 0, 1);
        CheckIntegerRange(errors, prefix + "LateralBeginCycle", track.nLateralBeginCycle, 0, 999999);
        CheckIntegerRange(errors, prefix + "VerticalBeginCycle", track.nVerticalBeginCycle, 0, 999999);
        CheckIntegerRange(errors, prefix + "VerticalSustainCycle", track.nVerticalSustainCycle, 0, 999999);
        CheckIntegerRange(errors, prefix + "TimeIntervalMs", track.nTimeIntervalMs, 0, 999999);
        CheckIntegerRange(errors, prefix + "DistanceIntervalMm", track.nDistanceIntervalMm, 0, 999999);
    }
}

inline void CollectStoredWeldErrors(
    const T_WELD_PARA& weld,
    std::vector<std::string>& errors)
{
    const auto checkNonNegative = [&](const char* field, double value)
        {
            CheckClosedRange(errors, field, value, 0.0, kUiDoubleMax);
        };
    const auto checkSigned = [&](const char* field, double value)
        {
            CheckClosedRange(errors, field, value, kUiDoubleMin, kUiDoubleMax);
        };

    checkNonNegative("WeldAngleSize", weld.dWeldAngleSize);
    CheckIntegerRange(errors, "LayerNo", weld.nLayerNo, 1, 999999);
    checkNonNegative("StartArcCurrent", weld.dStartArcCurrent);
    checkNonNegative("StartArcVoltage", weld.dStartArcVoltage);
    checkNonNegative("StartWaitTime", weld.dStartWaitTime);
    checkNonNegative("TrackCurrent", weld.dTrackCurrent);
    checkNonNegative("TrackVoltage", weld.dTrackVoltage);
    checkNonNegative("WeldVelocity", weld.WeldVelocity);
    checkNonNegative("StopArcCurrent", weld.dStopArcCurrent);
    checkNonNegative("StopArcVoltage", weld.dStopArcVoltage);
    checkNonNegative("StopWaitTime", weld.dStopWaitTime);
    checkNonNegative("WrapCurrent1", weld.dWrapCurrentt1);
    checkNonNegative("WrapVoltage1", weld.dWrapVoltage1);
    checkNonNegative("WrapWaitTime1", weld.dWrapWaitTime1);
    checkNonNegative("WrapCurrent2", weld.dWrapCurrentt2);
    checkNonNegative("WrapVoltage2", weld.dWrapVoltage2);
    checkNonNegative("WrapWaitTime2", weld.dWrapWaitTime2);
    checkNonNegative("WrapCurrent3", weld.dWrapCurrentt3);
    checkNonNegative("WrapVoltage3", weld.dWrapVoltage3);
    checkNonNegative("WrapWaitTime3", weld.dWrapWaitTime3);
    checkSigned("CrosswiseOffset", weld.CrosswiseOffset);
    checkSigned("VerticalOffset", weld.verticalOffset);
    checkSigned("WeldAngle", weld.dWeldAngle);
    checkSigned("WeldDipAngle", weld.dWeldDipAngle);
    checkNonNegative("CornerArcTransitionRadius", weld.dCornerArcTransitionRadius);
    checkNonNegative("CornerArcTransitionSpeed", weld.dCornerArcTransitionSpeed);
    checkNonNegative("CornerArcTransitionCurrent", weld.dCornerArcTransitionCurrent);
    checkNonNegative("CornerArcTransitionVoltage", weld.dCornerArcTransitionVoltage);
    CheckClosedRange(errors, "WeldOverlapRel", weld.dWeldOverlapRel, kOverlapMin, kOverlapMax);
    CheckIntegerRange(errors, "ArcMode", weld.nArcMode, 0, 7);
    CheckIntegerRange(errors, "WeldPostureType", weld.nWeldPostureType, 0, 3);
    CheckIntegerRange(errors, "CornerArcTransitionApplyScope",
        weld.nCornerArcTransitionApplyScope, 0, 2);
    CheckIntegerRange(errors, "WeldDirection", weld.nWeldDirection, -1, 1);
    CheckIntegerRange(errors, "WeaveEnable", weld.nWeaveEnable, 0, 1);
    CheckIntegerRange(errors, "TrackEnable", weld.nTrackEnable, 0, 1);
    CheckIntegerRange(errors, "CornerArcTransitionRadiusEnable",
        weld.nCornerArcTransitionRadiusEnable, 0, 1);
    CheckIntegerRange(errors, "CornerArcTransitionSpeedEnable",
        weld.nCornerArcTransitionSpeedEnable, 0, 1);
    CheckIntegerRange(errors, "CornerArcTransitionCurrentEnable",
        weld.nCornerArcTransitionCurrentEnable, 0, 1);
    CheckIntegerRange(errors, "CornerArcTransitionVoltageEnable",
        weld.nCornerArcTransitionVoltageEnable, 0, 1);

    if (weld.nCornerArcTransitionRadiusEnable != 0
        && weld.dCornerArcTransitionRadius < kTransitionRadiusMinMm)
    {
        AddError(errors, "CornerArcTransitionRadius", weld.dCornerArcTransitionRadius,
            "[" + FormatValue(kTransitionRadiusMinMm) + "," + FormatValue(kUiDoubleMax) + "] when enabled");
    }
    if (weld.dFinalWeldTrajectoryStepMm != 0.0)
    {
        CheckClosedRange(errors, "FinalWeldTrajectoryStepMm",
            weld.dFinalWeldTrajectoryStepMm, kFinalStepMinMm, kFinalStepMaxMm);
    }

    CollectTrackErrors(weld.tTrackParam, "Track.", errors, true);
}
}

inline bool ValidateWeave(const T_WeaveDate& weave, std::string& error)
{
    std::vector<std::string> errors;
    Detail::CollectWeaveErrors(weave, std::string(), errors);
    error = Detail::JoinErrors(errors);
    return errors.empty();
}

inline bool ValidateStoredEntryCount(int count, std::string& error)
{
    if (count <= 0 || count > kMaximumStoredProcessEntries)
    {
        error = "配置条目数=" + std::to_string(count)
            + " 无效，允许范围 [1,"
            + std::to_string(kMaximumStoredProcessEntries) + "]。";
        return false;
    }
    error.clear();
    return true;
}

inline bool ValidateStoredWeldProcess(const T_WELD_PARA& weld, std::string& error)
{
    std::vector<std::string> errors;
    Detail::CollectStoredWeldErrors(weld, errors);
    error = Detail::JoinErrors(errors);
    return errors.empty();
}

inline bool ValidateActualWeldProcess(const T_WELD_PARA& weld, std::string& error)
{
    std::vector<std::string> errors;
    Detail::CollectStoredWeldErrors(weld, errors);
    Detail::CheckPositiveRange(errors, "WeldVelocity", weld.WeldVelocity, kUiDoubleMax);
    Detail::CheckIntegerRange(errors, "ArcMode", weld.nArcMode, 0, 7);
    Detail::CheckIntegerRange(errors, "WeldPostureType", weld.nWeldPostureType, 0, 3);
    if (weld.nCornerArcTransitionSpeedEnable != 0)
    {
        Detail::CheckPositiveRange(errors, "CornerArcTransitionSpeed",
            weld.dCornerArcTransitionSpeed, kUiDoubleMax);
    }
    if ((weld.nCornerArcTransitionCurrentEnable != 0)
        != (weld.nCornerArcTransitionVoltageEnable != 0))
    {
        errors.push_back("字段 CornerArcTransitionCurrentEnable/CornerArcTransitionVoltageEnable 必须同时启用或同时关闭。");
    }
    if (weld.nWeaveEnable != 0)
    {
        Detail::CollectWeaveErrors(weld.tWeaveParam, "Weave.", errors);
        Detail::CheckPositiveRange(errors, "Weave.WeaveFrequencyHz",
            weld.tWeaveParam.dWeaveFrequencyHz, kUiDoubleMax);
    }
    if (weld.nTrackEnable != 0)
    {
        Detail::CollectTrackErrors(weld.tTrackParam, "Track.", errors, true);
    }
    error = Detail::JoinErrors(errors);
    return errors.empty();
}

inline bool ValidateActualMoveInfos(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    std::string& error)
{
    std::vector<std::string> errors;
    for (size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& info = moveInfos[index];
        if (!info.bWeldProcessEnabled)
        {
            continue;
        }
        const std::string prefix = "Move[" + std::to_string(index) + "].";
        Detail::CheckClosedRange(errors, prefix + "ArcStartCurrent",
            info.dArcStartCurrent, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "ArcStartVoltage",
            info.dArcStartVoltage, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "ArcStartWaitTime",
            info.dArcStartWaitTime, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "WeldCurrent",
            info.dWeldCurrent, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "WeldVoltage",
            info.dWeldVoltage, 0.0, kUiDoubleMax);
        Detail::CheckPositiveRange(errors, prefix + "WeldSpeedMmPerMin",
            info.dWeldSpeedMmPerMin, kUiDoubleMax);
        Detail::CheckPositiveRange(errors, prefix + "MotionSpeed",
            info.tSpeed.dSpeed, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "ArcEndCurrent",
            info.dArcEndCurrent, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "ArcEndVoltage",
            info.dArcEndVoltage, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "ArcEndWaitTime",
            info.dArcEndWaitTime, 0.0, kUiDoubleMax);
        Detail::CheckClosedRange(errors, prefix + "OverlapRel",
            info.dOverlapRel, kOverlapMin, kOverlapMax);
        Detail::CheckIntegerRange(errors, prefix + "ArcMode", info.nArcMode, 0, 7);
        Detail::CheckIntegerRange(errors, prefix + "PostureType", info.nPostureType, 0, 3);
        if (info.bHasWeaveParam)
        {
            Detail::CollectWeaveErrors(info.tWeaveParam, prefix + "Weave.", errors);
            Detail::CheckPositiveRange(errors, prefix + "Weave.WeaveFrequencyHz",
                info.tWeaveParam.dWeaveFrequencyHz, kUiDoubleMax);
        }
        if (info.bHasTrackParam)
        {
            Detail::CollectTrackErrors(info.tTrackParam, prefix + "Track.", errors, true);
        }
    }
    error = Detail::JoinErrors(errors);
    return errors.empty();
}
}
