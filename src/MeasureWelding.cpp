#include "MeasureWelding.h"

#include "OPini.h"
#include "RobotDriverAdaptor.h"
#include "RobotLog.h"
#include "RobotMessage.h"

#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <vector>

namespace
{
RobotLog& MeasureWeldingLogger()
{
    static RobotLog logger(".//Log//MeasureWelding.txt");
    return logger;
}

namespace fs = std::filesystem;
}

MeasureWelding::MeasureWelding()
{
}

bool MeasureWelding::RunWorkFlow()
{
    m_sLastError.clear();

    // 当前流程第一步：读取线扫起点和终点脉冲姿态
    if (!m_tScanWeldingParam.bValid)
    {
        m_sLastError = "工作流程执行失败：起点和终点脉冲姿态未准备好。";
        LogError("%s", m_sLastError.c_str());
        showErrorMessage("焊缝测量", "%s", m_sLastError.c_str());
        return false;
    }

    showInfoMessage("焊缝测量", "线扫起点和终点脉冲姿态读取成功。");
    return true;
}

bool MeasureWelding::ReadRobotScanRange(const RobotDriverAdaptor* pRobotDriver)
{
    return ReadRobotScanRange(pRobotDriver, "Table1");
}

bool MeasureWelding::ReadRobotScanRange(const RobotDriverAdaptor* pRobotDriver, const std::string& sectionName)
{
    if (pRobotDriver == nullptr)
    {
        m_sLastError = "机器人驱动对象为空。";
        LogError("%s", m_sLastError.c_str());
        showErrorMessage("焊缝测量", "%s", m_sLastError.c_str());
        return false;
    }

    // 机器人名称从 RobotDriverAdaptor 中获取
    const std::string robotName = pRobotDriver->m_sRobotName;
    const std::string iniFilePath = ResolveRobotIniFilePath(robotName);
    const bool bRet = ReadRobotScanRange(iniFilePath, sectionName);
    if (bRet)
    {
        m_tScanWeldingParam.sRobotName = robotName;
    }
    return bRet;
}

bool MeasureWelding::ReadRobotScanRange(const std::string& iniFilePath, const std::string& sectionName)
{
    m_sLastError.clear();
    m_tScanWeldingParam = T_SCAN_WELDING_PARAM();
    m_tScanWeldingParam.sIniFilePath = iniFilePath;
    m_tScanWeldingParam.sSectionName = sectionName;

    if (iniFilePath.empty())
    {
        m_sLastError = "未找到 LineScanParam.ini 文件。";
        LogError("%s", m_sLastError.c_str());
        showErrorMessage("焊缝测量", "%s", m_sLastError.c_str());
        return false;
    }

    if (!fs::exists(fs::path(iniFilePath)))
    {
        m_sLastError = "LineScanParam.ini 文件不存在: " + iniFilePath;
        LogError("%s", m_sLastError.c_str());
        showErrorMessage("焊缝测量", "%s", m_sLastError.c_str());
        return false;
    }

    COPini iniReader;
    iniReader.SetFileName(iniFilePath);

    LogInfo("开始读取线扫参数文件: %s, Section: %s", iniFilePath.c_str(), sectionName.c_str());

    // 读取 [Table1] 这类分组中的起点/终点脉冲姿态
    const bool bStartOk = ReadStartPulse(iniReader, sectionName, m_tScanWeldingParam.tStartPulse);
    const bool bEndOk = ReadEndPulse(iniReader, sectionName, m_tScanWeldingParam.tEndPulse);

    if (!bStartOk || !bEndOk)
    {
        m_sLastError = "读取线扫起点或终点脉冲姿态失败，Section: " + sectionName;
        LogError("%s", m_sLastError.c_str());
        showErrorMessage("焊缝测量", "%s", m_sLastError.c_str());
        return false;
    }

    m_tScanWeldingParam.bValid = true;

    LogInfo("线扫脉冲姿态读取成功, Start(S=%ld,L=%ld,U=%ld,R=%ld,B=%ld,T=%ld), End(S=%ld,L=%ld,U=%ld,R=%ld,B=%ld,T=%ld)",
        m_tScanWeldingParam.tStartPulse.nSPulse,
        m_tScanWeldingParam.tStartPulse.nLPulse,
        m_tScanWeldingParam.tStartPulse.nUPulse,
        m_tScanWeldingParam.tStartPulse.nRPulse,
        m_tScanWeldingParam.tStartPulse.nBPulse,
        m_tScanWeldingParam.tStartPulse.nTPulse,
        m_tScanWeldingParam.tEndPulse.nSPulse,
        m_tScanWeldingParam.tEndPulse.nLPulse,
        m_tScanWeldingParam.tEndPulse.nUPulse,
        m_tScanWeldingParam.tEndPulse.nRPulse,
        m_tScanWeldingParam.tEndPulse.nBPulse,
        m_tScanWeldingParam.tEndPulse.nTPulse);

    return true;
}

const T_SCAN_WELDING_PARAM& MeasureWelding::GetScanWeldingParam() const
{
    return m_tScanWeldingParam;
}

std::string MeasureWelding::GetLastError() const
{
    return m_sLastError;
}

std::string MeasureWelding::ResolveRobotIniFilePath(const std::string& robotName) const
{
    if (robotName.empty())
    {
        return "";
    }

    // 按约定路径 DATA/<机器人名>/LineScanParam.ini 定位线扫参数文件
    const fs::path iniFilePath = fs::current_path() / "DATA" / robotName / "LineScanParam.ini";
    if (fs::exists(iniFilePath))
    {
        return iniFilePath.string();
    }

    return "";
}

bool MeasureWelding::ReadStartPulse(COPini& iniReader, const std::string& sectionName, T_ANGLE_PULSE& tStartPulse) const
{
    iniReader.SetSectionName(sectionName);

    // 读取 ini 中的起点脉冲姿态:
    // startpulse.nS / startpulse.nL / startpulse.nU / startpulse.nR / startpulse.nB / startpulse.nT
    // startpulse.lBX / startpulse.lBY / startpulse.lBZ
    int bRtn = 1;
    bRtn = (bRtn && iniReader.ReadString("startpulse.nS", &tStartPulse.nSPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.nL", &tStartPulse.nLPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.nU", &tStartPulse.nUPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.nR", &tStartPulse.nRPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.nB", &tStartPulse.nBPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.nT", &tStartPulse.nTPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.lBX", &tStartPulse.lBXPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.lBY", &tStartPulse.lBYPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("startpulse.lBZ", &tStartPulse.lBZPulse) > 0) ? 1 : 0;

    return (bRtn > 0);
}

bool MeasureWelding::ReadEndPulse(COPini& iniReader, const std::string& sectionName, T_ANGLE_PULSE& tEndPulse) const
{
    iniReader.SetSectionName(sectionName);

    // 读取 ini 中的终点脉冲姿态:
    // endpulse.nS / endpulse.nL / endpulse.nU / endpulse.nR / endpulse.nB / endpulse.nT
    // endpulse.lBX / endpulse.lBY / endpulse.lBZ
    int bRtn = 1;
    bRtn = (bRtn && iniReader.ReadString("endpulse.nS", &tEndPulse.nSPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.nL", &tEndPulse.nLPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.nU", &tEndPulse.nUPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.nR", &tEndPulse.nRPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.nB", &tEndPulse.nBPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.nT", &tEndPulse.nTPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.lBX", &tEndPulse.lBXPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.lBY", &tEndPulse.lBYPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("endpulse.lBZ", &tEndPulse.lBZPulse) > 0) ? 1 : 0;

    return (bRtn > 0);
}

void MeasureWelding::LogInfo(const char* format, ...) const
{
    va_list args;
    va_start(args, format);
    const int size = vsnprintf(nullptr, 0, format, args) + 1;
    va_end(args);

    if (size <= 0)
    {
        return;
    }

    std::vector<char> buffer(size);
    va_start(args, format);
    vsnprintf(buffer.data(), static_cast<size_t>(size), format, args);
    va_end(args);

    MeasureWeldingLogger().write(LogColor::SUCCESS, "%s", buffer.data());
}

void MeasureWelding::LogError(const char* format, ...) const
{
    va_list args;
    va_start(args, format);
    const int size = vsnprintf(nullptr, 0, format, args) + 1;
    va_end(args);

    if (size <= 0)
    {
        return;
    }

    std::vector<char> buffer(size);
    va_start(args, format);
    vsnprintf(buffer.data(), static_cast<size_t>(size), format, args);
    va_end(args);

    MeasureWeldingLogger().write(LogColor::ERR, "%s", buffer.data());
}
