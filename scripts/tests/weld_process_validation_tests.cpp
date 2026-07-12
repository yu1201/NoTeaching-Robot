#include "WeldProcessValidation.h"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace
{
void Require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

T_WELD_PARA ValidProcess()
{
    T_WELD_PARA weld {};
    weld.strWorkPeace = "test";
    weld.strWeldType = "test";
    weld.nLayerNo = 1;
    weld.WeldVelocity = 400.0;
    weld.dWeldOverlapRel = 20.0;
    weld.nArcMode = 4;
    weld.nWeldPostureType = 1;
    return weld;
}

void RequireInvalidStored(
    const T_WELD_PARA& weld,
    const std::string& field,
    const std::string& renderedValue)
{
    std::string error;
    Require(!WeldProcessValidation::ValidateStoredWeldProcess(weld, error),
        "invalid stored weld process was accepted");
    Require(error.find(field) != std::string::npos,
        "stored validation error does not identify the field");
    Require(error.find(renderedValue) != std::string::npos,
        "stored validation error does not include the value");
}
}

int main()
{
    std::string error;

    T_WELD_PARA weld = ValidProcess();
    Require(WeldProcessValidation::ValidateStoredWeldProcess(weld, error),
        "normal stored weld process must pass");
    Require(WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "normal actual weld process must pass");

    weld = ValidProcess();
    weld.dTrackCurrent = std::numeric_limits<double>::quiet_NaN();
    RequireInvalidStored(weld, "TrackCurrent", "nan");

    weld = ValidProcess();
    weld.dTrackVoltage = std::numeric_limits<double>::infinity();
    RequireInvalidStored(weld, "TrackVoltage", "inf");

    weld = ValidProcess();
    weld.dStartArcCurrent = -0.001;
    RequireInvalidStored(weld, "StartArcCurrent", "-0.001");

    weld = ValidProcess();
    weld.dStopWaitTime = -1.0;
    RequireInvalidStored(weld, "StopWaitTime", "-1");

    weld = ValidProcess();
    weld.tTrackParam.dVerticalReferenceCurrent = -1.0;
    RequireInvalidStored(weld, "Track.VerticalReferenceCurrent", "-1");

    weld = ValidProcess();
    weld.dTrackCurrent = WeldProcessValidation::kUiDoubleMax + 1.0;
    RequireInvalidStored(weld, "TrackCurrent", "1000000");

    weld = ValidProcess();
    weld.dWeldOverlapRel = WeldProcessValidation::kOverlapMax + 0.001;
    RequireInvalidStored(weld, "WeldOverlapRel", "200.001");

    weld = ValidProcess();
    weld.nArcMode = 8;
    RequireInvalidStored(weld, "ArcMode", "8");

    weld = ValidProcess();
    weld.dFinalWeldTrajectoryStepMm = WeldProcessValidation::kFinalStepMinMm - 0.001;
    RequireInvalidStored(weld, "FinalWeldTrajectoryStepMm", "1.999");

    weld = ValidProcess();
    weld.WeldVelocity = 0.0;
    Require(WeldProcessValidation::ValidateStoredWeldProcess(weld, error),
        "zero legacy speed must remain loadable/editable");
    Require(!WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "zero speed must not pass the actual-weld gate");
    Require(error.find("WeldVelocity=0") != std::string::npos,
        "actual-weld speed error must identify field and value");

    weld = ValidProcess();
    weld.tWeaveParam.dWeaveFrequencyHz = 0.0;
    Require(WeldProcessValidation::ValidateStoredWeldProcess(weld, error),
        "zero legacy weave frequency must remain loadable/editable");
    Require(!WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "zero weave frequency must not pass when weave is enabled");
    Require(error.find("Weave.WeaveFrequencyHz=0") != std::string::npos,
        "enabled-weave frequency error must identify field and value");
    weld.nWeaveEnable = 0;
    Require(WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "zero weave frequency must remain valid when weave is disabled");
    weld.nWeaveEnable = 1;
    weld.tWeaveParam.dWeaveFrequencyHz = std::numeric_limits<double>::min();
    Require(WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "smallest positive finite weave frequency must pass the actual-weld gate");

    weld = ValidProcess();
    weld.dStartArcCurrent = WeldProcessValidation::kUiDoubleMax;
    weld.dStartArcVoltage = 0.0;
    weld.dStartWaitTime = WeldProcessValidation::kUiDoubleMax;
    weld.dTrackCurrent = 0.0;
    weld.dTrackVoltage = WeldProcessValidation::kUiDoubleMax;
    weld.WeldVelocity = WeldProcessValidation::kUiDoubleMax;
    weld.dStopArcCurrent = 0.0;
    weld.dStopArcVoltage = WeldProcessValidation::kUiDoubleMax;
    weld.dStopWaitTime = 0.0;
    weld.dWeldOverlapRel = WeldProcessValidation::kOverlapMax;
    weld.dFinalWeldTrajectoryStepMm = WeldProcessValidation::kFinalStepMaxMm;
    weld.nCornerArcTransitionRadiusEnable = 1;
    weld.dCornerArcTransitionRadius = WeldProcessValidation::kTransitionRadiusMinMm;
    weld.nCornerArcTransitionSpeedEnable = 1;
    weld.dCornerArcTransitionSpeed = 0.001;
    Require(WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "legal inclusive boundaries must pass");

    weld.dFinalWeldTrajectoryStepMm = WeldProcessValidation::kFinalStepMinMm;
    Require(WeldProcessValidation::ValidateActualWeldProcess(weld, error),
        "minimum legal final step must pass");

    T_WeaveDate weave {};
    weave.dWeaveAmplitudeMm = std::numeric_limits<double>::quiet_NaN();
    Require(!WeldProcessValidation::ValidateWeave(weave, error),
        "non-finite weave value must fail");
    Require(error.find("WeaveAmplitudeMm=nan") != std::string::npos,
        "weave error must identify field and value");

    weave = T_WeaveDate{};
    weave.nWeaveShape = 4;
    Require(!WeldProcessValidation::ValidateWeave(weave, error),
        "undefined weave shape below the controller enum range must fail");
    Require(error.find("WeaveShape=4") != std::string::npos,
        "weave shape error must identify the invalid value");
    weave.nWeaveShape = 19;
    Require(!WeldProcessValidation::ValidateWeave(weave, error),
        "undefined weave shape above the controller enum range must fail");
    for (const int validShape : { 0, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 })
    {
        weave.nWeaveShape = validShape;
        Require(WeldProcessValidation::ValidateWeave(weave, error),
            "documented controller weave shape must pass");
    }

    Require(WeldProcessValidation::ValidateStoredEntryCount(1, error),
        "minimum stored entry count must pass");
    Require(WeldProcessValidation::ValidateStoredEntryCount(
        WeldProcessValidation::kMaximumStoredProcessEntries, error),
        "maximum stored entry count must pass");
    Require(!WeldProcessValidation::ValidateStoredEntryCount(0, error),
        "zero stored entry count must fail");
    Require(!WeldProcessValidation::ValidateStoredEntryCount(
        std::numeric_limits<int>::max(), error),
        "hostile stored entry count must fail before reserve");

    T_ROBOT_MOVE_INFO move {};
    move.bWeldProcessEnabled = true;
    move.tSpeed.dSpeed = 400.0;
    move.dWeldSpeedMmPerMin = 400.0;
    move.dOverlapRel = 20.0;
    move.nArcMode = 4;
    move.nPostureType = 1;
    std::vector<T_ROBOT_MOVE_INFO> moves { move };
    Require(WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "normal STEP actual-weld metadata must pass");

    moves[0] = move;
    moves[0].bHasWeaveParam = true;
    moves[0].tWeaveParam.dWeaveFrequencyHz = 0.0;
    Require(!WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "STEP metadata must reject a zero frequency when weave data is sent");
    Require(error.find("Move[0].Weave.WeaveFrequencyHz=0") != std::string::npos,
        "STEP weave-frequency error must identify point, field, and value");
    moves[0].tWeaveParam.dWeaveFrequencyHz = std::numeric_limits<double>::min();
    Require(WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "STEP metadata must accept the smallest positive finite weave frequency");

    moves[0].dWeldVoltage = std::numeric_limits<double>::infinity();
    Require(!WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "STEP non-finite weld metadata must fail");
    Require(error.find("Move[0].WeldVoltage=inf") != std::string::npos,
        "STEP error must identify point, field, and value");

    moves[0] = move;
    moves[0].dWeldSpeedMmPerMin = -1.0;
    Require(!WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "STEP negative weld speed must fail");

    moves[0] = move;
    moves[0].tSpeed.dSpeed = std::numeric_limits<double>::quiet_NaN();
    Require(!WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "STEP non-finite motion speed must fail");

    moves[0] = move;
    moves[0].dArcStartCurrent = WeldProcessValidation::kUiDoubleMax + 1.0;
    Require(!WeldProcessValidation::ValidateActualMoveInfos(moves, error),
        "STEP out-of-range current must fail");

    std::cout << "PASS: weld-process finite/range and STEP actual-weld gates\n";
    return 0;
}
