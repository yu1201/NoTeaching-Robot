#include "RobotAcceptanceJointMotion.h"
#include <cstdlib>
#include <iostream>
#include <climits>
struct Axis { double dSPulseUnit=.001, dLPulseUnit=.001, dUPulseUnit=.001,
    dRPulseUnit=.001, dBPulseUnit=.001, dTPulseUnit=.001; };
struct Pulse { long nSPulse=1000, nLPulse=2000, nUPulse=3000, nRPulse=4000,
    nBPulse=5000, nTPulse=6000, lBXPulse=7, lBYPulse=8, lBZPulse=9; };
static void Check(bool ok, const char* message)
{ if (!ok) { std::cerr << "FAIL " << message << '\n'; std::exit(1); } }
int main()
{
    using namespace RobotAcceptanceJointMotion;
    Axis axis; Pulse start, target; double error=0;
    Check(OutwardTarget(start,axis,target) && target.nSPulse==1500, "J1 degree to pulse");
    Check(target.nLPulse==start.nLPulse && target.lBZPulse==start.lBZPulse, "preserve non-J1 axes");
    Check(Matches(target,target,axis,error) && error==0, "exact feedback");
    Check(!Matches(start,target,axis,error), "done without movement is failure");
    Pulse changed=target; changed.nUPulse+=100;
    Check(!Matches(changed,target,axis,error), "other robot axis mismatch");
    changed=target; changed.lBYPulse+=3;
    Check(!Matches(changed,target,axis,error), "external axis mismatch");
    axis.dSPulseUnit=-.001;
    Check(OutwardTarget(start,axis,target) && target.nSPulse==500, "negative unit preserves positive angle");
    axis.dSPulseUnit=0; Check(!OutwardTarget(start,axis,target), "zero unit blocked");
    axis.dSPulseUnit=.1; Check(!OutwardTarget(start,axis,target), "coarse unit blocked");
    axis.dSPulseUnit=std::numeric_limits<double>::quiet_NaN();
    Check(!OutwardTarget(start,axis,target), "nonfinite unit blocked");
    axis=Axis{}; start.nSPulse=LONG_MAX;
    Check(!OutwardTarget(start,axis,target), "pulse overflow blocked");
    start=Pulse{}; target=Pulse{}; start.lBYPulse=LONG_MIN; target.lBYPulse=LONG_MAX;
    Check(!Matches(start,target,axis,error), "extreme difference no signed overflow");
    start=Pulse{}; target=Pulse{}; start.nSPulse=LONG_MIN; target.nSPulse=LONG_MAX;
    Check(!Matches(start,target,axis,error), "extreme main axis difference");
    axis.dTPulseUnit=0; Check(!Matches(start,target,axis,error), "missing nonmoving axis unit blocked");
    std::cout << "PASS joint acceptance arithmetic: 14 checks\n";
}
