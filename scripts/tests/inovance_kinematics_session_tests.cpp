#include "InovanceKinematicsSession.h"
#include <iostream>
#include <limits>
#include <stdexcept>

static void Require(bool condition, const char* error)
{
    if (!condition) { throw std::runtime_error(error); }
}

int main()
{
    try
    {
        InovanceKinematicsSession session;
        Require(!session.Ready(), "fresh driver cannot trust database/cache AxisUnit");
        const auto first = session.Invalidate();
        Require(session.Publish(first) && session.Ready(), "live validation may enable current session");
        const auto disconnected = session.Invalidate();
        Require(!session.Ready(), "disconnect must revoke the old model immediately");
        Require(!session.Publish(first) && !session.Ready(), "late download from old session must not publish");
        const auto next = session.Invalidate();
        Require(!session.Publish(disconnected), "a second reconnect must invalidate intermediate results");
        Require(session.Publish(next) && session.Ready(), "fresh revalidation can recover readiness");
        session.Invalidate();
        Require(!session.Ready(), "refresh failure must not retain the previous success");
        InovanceKinematicsSession recreated;
        Require(!recreated.Ready(), "recreated driver must not inherit persisted permission");
        std::atomic_bool reading{false};
        {
            InovanceKinematicsReadScope scope(reading);
            Require(reading.load(), "read-only command exclusion was not armed");
        }
        Require(!reading.load(), "read-only command exclusion was not released");

        const std::vector<double> joints{1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0};
        Require(InovanceKinematicsSession::StationarySample(joints, joints), "stable sample rejected");
        auto changed = joints;
        changed[0] += 0.01;
        Require(!InovanceKinematicsSession::StationarySample(joints, changed), "moving joint accepted");
        changed = joints;
        changed[8] += 1;
        Require(!InovanceKinematicsSession::StationarySample(joints, changed), "moving external axis accepted");
        changed = joints;
        changed[2] = std::numeric_limits<double>::quiet_NaN();
        Require(!InovanceKinematicsSession::StationarySample(joints, changed), "NaN sample accepted");
        Require(!InovanceKinematicsSession::StationarySample(joints, {1, 2}), "truncated sample accepted");
        std::cout << "PASS: Inovance kinematics session invalidation, late-result rejection and stationary sample gates\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
