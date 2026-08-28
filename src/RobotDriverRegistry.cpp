#include "RobotDriverRegistry.h"

#include "Const.h"
#include "FANUCRobotDriver.h"
#include "InovanceRobotDriver.h"
#include "RobotDriverAdaptor.h"
#include "STEPRobotDriver.h"

#include <algorithm>

namespace
{
RobotDriverAdaptor* CreateFanucDriver(const std::string& unitName, RobotLog* log)
{
    return new FANUCRobotCtrl(unitName, log);
}

RobotDriverAdaptor* CreateStepDriver(const std::string& unitName, RobotLog* log)
{
    return new STEPRobotCtrl(unitName, log);
}

RobotDriverAdaptor* CreateInovanceDriver(const std::string& unitName, RobotLog* log)
{
    return new InovanceRobotCtrl(unitName, log);
}
}

const std::vector<RobotDriverRegistration>& RobotDriverRegistry::RegisteredTypes()
{
    static const std::vector<RobotDriverRegistration> registrations =
    {
        {
            ROBOT_TYPE_FANUC,
            "FANUC",
            &CreateFanucDriver,
            { "fanuc", 9000, 9001, true, 21, "", "anonymous", "", false, "", "/md", false, "Job/FANUC" }
        },
        {
            ROBOT_TYPE_STEP,
            "STEP",
            &CreateStepDriver,
            { "step", 30312, 0, false, 21, "", "root", "", true, "PCRobot", "/UserPrograms", true, "Job/STEP" }
        },
        {
            ROBOT_TYPE_INOVANCE,
            "汇川 Inovance",
            &CreateInovanceDriver,
            { "inovance", 2222, 0, false, 7777, "192.168.23.25", "robot", "123456", false, "", "/TeachProgram", false, "Job/Inovance" }
        },
    };
    return registrations;
}

const RobotDriverRegistration* RobotDriverRegistry::Find(int typeCode)
{
    const auto& registrations = RegisteredTypes();
    const auto found = std::find_if(
        registrations.cbegin(), registrations.cend(),
        [typeCode](const RobotDriverRegistration& registration)
        {
            return registration.typeCode == typeCode;
        });
    return found == registrations.cend() ? nullptr : &(*found);
}

const RobotDriverSetupProfile* RobotDriverRegistry::SetupProfile(int typeCode)
{
    const RobotDriverRegistration* registration = Find(typeCode);
    return registration == nullptr ? nullptr : &registration->setup;
}

bool RobotDriverRegistry::IsRegistered(int typeCode)
{
    const RobotDriverRegistration* registration = Find(typeCode);
    return registration != nullptr && registration->create != nullptr;
}

std::string RobotDriverRegistry::DisplayName(int typeCode)
{
    const RobotDriverRegistration* registration = Find(typeCode);
    return registration == nullptr
        ? "UNREGISTERED(" + std::to_string(typeCode) + ")"
        : registration->displayName;
}

RobotDriverAdaptor* RobotDriverRegistry::Create(
    int typeCode,
    const std::string& unitName,
    RobotLog* log,
    std::string* error)
{
    const RobotDriverRegistration* registration = Find(typeCode);
    if (registration == nullptr || registration->create == nullptr)
    {
        if (error != nullptr)
        {
            *error = "RobotType=" + std::to_string(typeCode)
                + " 未登记品牌底层，驱动创建已拒绝。";
        }
        return nullptr;
    }

    RobotDriverAdaptor* driver = registration->create(unitName, log);
    if (driver == nullptr && error != nullptr)
    {
        *error = DisplayName(typeCode) + " 品牌底层创建失败。";
    }
    else if (error != nullptr)
    {
        error->clear();
    }
    return driver;
}
