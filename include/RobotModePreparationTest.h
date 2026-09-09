#pragma once
#include <string>

struct RobotModePreparationTestCase
{
    std::string id;
    std::string name;
};

struct RobotModePreparationTestResult
{
    bool passed = false;
    bool restoreVerified = false;
    std::string evidence;
};
