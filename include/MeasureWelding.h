#pragma once

#include "Const.h"

#include <string>

class COPini;
class RobotDriverAdaptor;

class MeasureWelding
{
public:


public:
    MeasureWelding();

    // 工作流程入口，当前先完成“读取线扫起点和终点脉冲姿态”
    bool RunWorkFlow();

    // 默认读取 Table1 分组
    bool ReadRobotScanRange(const RobotDriverAdaptor* pRobotDriver);
    // 指定读取某个分组，例如 Table1 / Table2
    bool ReadRobotScanRange(const RobotDriverAdaptor* pRobotDriver, const std::string& sectionName);
    // 直接指定 ini 路径和分组，方便独立测试
    bool ReadRobotScanRange(const std::string& iniFilePath, const std::string& sectionName = "Table1");

    const T_SCAN_WELDING_PARAM& GetScanWeldingParam() const;
    std::string GetLastError() const;

private:
    std::string ResolveRobotIniFilePath(const std::string& robotName) const;
    bool ReadStartPulse(COPini& iniReader, const std::string& sectionName, T_ANGLE_PULSE& tStartPulse) const;
    bool ReadEndPulse(COPini& iniReader, const std::string& sectionName, T_ANGLE_PULSE& tEndPulse) const;

    void LogInfo(const char* format, ...) const;
    void LogError(const char* format, ...) const;

private:
    T_SCAN_WELDING_PARAM m_tScanWeldingParam;
    std::string m_sLastError;
};
