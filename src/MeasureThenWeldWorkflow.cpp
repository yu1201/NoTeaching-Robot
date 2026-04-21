#include "MeasureThenWeldWorkflow.h"

#include "RobotDriverAdaptor.h"
#include "RobotLog.h"

#include <cstdarg>
#include <cstdio>

namespace
{
RobotLog& MeasureThenWeldLogger()
{
    static RobotLog logger(".//Log//MeasureThenWeldWorkflow.txt");
    return logger;
}
}

MeasureThenWeldWorkflow::MeasureThenWeldWorkflow(RobotDriverAdaptor* pRobotDriver, RobotLog* pRobotLog)
    : m_pRobotDriver(pRobotDriver)
    , m_pRobotLog(pRobotLog)
{
}

bool MeasureThenWeldWorkflow::Run(const T_MEASURE_THEN_WELD_CONTEXT& context)
{
    m_context = context;
    m_sLastError.clear();
    SetStage(E_MEASURE_THEN_WELD_STAGE::Idle);

    LogInfo("先测后焊流程开始: Section=%s, WeldTrackNo=%d, DryRun=%d",
        m_context.sScanSectionName.c_str(), m_context.nWeldTrackNo, m_context.bDryRun ? 1 : 0);

    if (!PrepareMeasure(m_context.sScanSectionName))
    {
        return false;
    }

    if (!RunMeasure())
    {
        return false;
    }

    if (!GenerateWeldPath())
    {
        return false;
    }

    if (!RunWeld())
    {
        return false;
    }

    SetStage(E_MEASURE_THEN_WELD_STAGE::Done);
    LogInfo("先测后焊流程完成");
    return true;
}

bool MeasureThenWeldWorkflow::PrepareMeasure(const std::string& sectionName)
{
    SetStage(E_MEASURE_THEN_WELD_STAGE::LoadMeasureConfig);

    if (m_pRobotDriver == nullptr)
    {
        return Fail("先测后焊流程失败：机器人驱动对象为空。");
    }

    if (!m_measureReader.ReadRobotScanRange(m_pRobotDriver, sectionName))
    {
        return Fail("读取线扫起点/终点失败：" + m_measureReader.GetLastError());
    }

    LogInfo("测量参数准备完成: Section=%s", sectionName.c_str());
    return true;
}

bool MeasureThenWeldWorkflow::RunMeasure()
{
    SetStage(E_MEASURE_THEN_WELD_STAGE::RunMeasure);

    if (m_measureExecutor)
    {
        if (!m_measureExecutor(*this))
        {
            return Fail("测量执行器返回失败。");
        }
    }
    else
    {
        // 默认只完成流程占位，后续接入线扫或视觉算法时通过 SetMeasureExecutor 注入。
        LogInfo("未设置测量执行器，默认跳过实际测量动作。");
    }

    return true;
}

bool MeasureThenWeldWorkflow::GenerateWeldPath()
{
    SetStage(E_MEASURE_THEN_WELD_STAGE::GenerateWeldPath);

    if (m_weldPathBuilder)
    {
        m_vtWeldPath.clear();
        if (!m_weldPathBuilder(m_measureReader.GetScanWeldingParam(), m_vtWeldPath))
        {
            return Fail("焊接轨迹生成器返回失败。");
        }
    }

    LogInfo("焊接轨迹准备完成: 点数=%zu", m_vtWeldPath.size());
    return true;
}

bool MeasureThenWeldWorkflow::RunWeld()
{
    SetStage(E_MEASURE_THEN_WELD_STAGE::RunWeld);

    if (m_context.bDryRun)
    {
        LogInfo("DryRun 模式，跳过实际焊接下发。");
        return true;
    }

    if (m_weldExecutor)
    {
        if (!m_weldExecutor(*this))
        {
            return Fail("焊接执行器返回失败。");
        }
        return true;
    }

    if (m_pRobotDriver == nullptr)
    {
        return Fail("焊接执行失败：机器人驱动对象为空。");
    }

    if (m_vtWeldPath.empty())
    {
        return Fail("焊接执行失败：焊接轨迹为空。");
    }

    const int nRet = m_pRobotDriver->ContiMoveAny(m_vtWeldPath);
    if (nRet != 0)
    {
        return Fail("焊接轨迹下发失败，ContiMoveAny 返回值=" + std::to_string(nRet));
    }

    LogInfo("焊接轨迹已下发: 点数=%zu", m_vtWeldPath.size());
    return true;
}

void MeasureThenWeldWorkflow::SetMeasureExecutor(WorkflowCallback executor)
{
    m_measureExecutor = executor;
}

void MeasureThenWeldWorkflow::SetWeldExecutor(WorkflowCallback executor)
{
    m_weldExecutor = executor;
}

void MeasureThenWeldWorkflow::SetWeldPathBuilder(WeldPathBuilder builder)
{
    m_weldPathBuilder = builder;
}

void MeasureThenWeldWorkflow::SetWeldPath(const std::vector<T_ROBOT_MOVE_INFO>& weldPath)
{
    m_vtWeldPath = weldPath;
}

void MeasureThenWeldWorkflow::ClearWeldPath()
{
    m_vtWeldPath.clear();
}

E_MEASURE_THEN_WELD_STAGE MeasureThenWeldWorkflow::GetStage() const
{
    return m_stage;
}

const T_MEASURE_THEN_WELD_CONTEXT& MeasureThenWeldWorkflow::GetContext() const
{
    return m_context;
}

const T_SCAN_WELDING_PARAM& MeasureThenWeldWorkflow::GetScanParam() const
{
    return m_measureReader.GetScanWeldingParam();
}

const std::vector<T_ROBOT_MOVE_INFO>& MeasureThenWeldWorkflow::GetWeldPath() const
{
    return m_vtWeldPath;
}

std::string MeasureThenWeldWorkflow::GetLastError() const
{
    return m_sLastError;
}

void MeasureThenWeldWorkflow::SetStage(E_MEASURE_THEN_WELD_STAGE stage)
{
    m_stage = stage;
}

bool MeasureThenWeldWorkflow::Fail(const std::string& message)
{
    m_sLastError = message;
    SetStage(E_MEASURE_THEN_WELD_STAGE::Failed);
    LogError("%s", m_sLastError.c_str());
    return false;
}

void MeasureThenWeldWorkflow::LogInfo(const char* format, ...) const
{
    char buffer[1024] = {};
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::SUCCESS, "%s", buffer);
    }
    else
    {
        MeasureThenWeldLogger().write(LogColor::SUCCESS, "%s", buffer);
    }
}

void MeasureThenWeldWorkflow::LogError(const char* format, ...) const
{
    char buffer[1024] = {};
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::ERR, "%s", buffer);
    }
    else
    {
        MeasureThenWeldLogger().write(LogColor::ERR, "%s", buffer);
    }
}
