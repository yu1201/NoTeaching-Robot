#pragma once

#include "Const.h"
#include "MeasureWelding.h"

#include <functional>
#include <string>
#include <vector>

class RobotDriverAdaptor;
class RobotLog;

// 先测后焊流程阶段。外部界面可以用这个状态做进度显示或故障定位。
enum class E_MEASURE_THEN_WELD_STAGE
{
    Idle = 0,
    LoadMeasureConfig,
    RunMeasure,
    GenerateWeldPath,
    RunWeld,
    Done,
    Failed
};

// 单次先测后焊任务的运行参数。
struct T_MEASURE_THEN_WELD_CONTEXT
{
    std::string sScanSectionName = "Table1"; // LineScanParam.ini 中的料台分组
    int nWeldTrackNo = 1;                    // 焊缝/轨迹编号，后续可用于算法或工艺筛选
    bool bDryRun = false;                    // true 时只跑流程和日志，不下发实际焊接运动
};

// 先测后焊流程编排类：
// 1. 从机器人目录读取线扫起点/终点脉冲；
// 2. 调用外部测量执行器；
// 3. 调用外部轨迹生成器；
// 4. 使用 RobotDriverAdaptor::ContiMoveAny 下发焊接轨迹，或调用自定义焊接执行器。
class MeasureThenWeldWorkflow
{
public:
    using WorkflowCallback = std::function<bool(MeasureThenWeldWorkflow&)>;
    using WeldPathBuilder = std::function<bool(const T_SCAN_WELDING_PARAM&, std::vector<T_ROBOT_MOVE_INFO>&)>;

    explicit MeasureThenWeldWorkflow(RobotDriverAdaptor* pRobotDriver, RobotLog* pRobotLog = nullptr);

    bool Run(const T_MEASURE_THEN_WELD_CONTEXT& context = T_MEASURE_THEN_WELD_CONTEXT());

    bool PrepareMeasure(const std::string& sectionName = "Table1");
    bool RunMeasure();
    bool GenerateWeldPath();
    bool RunWeld();

    void SetMeasureExecutor(WorkflowCallback executor);
    void SetWeldExecutor(WorkflowCallback executor);
    void SetWeldPathBuilder(WeldPathBuilder builder);
    void SetWeldPath(const std::vector<T_ROBOT_MOVE_INFO>& weldPath);
    void ClearWeldPath();

    E_MEASURE_THEN_WELD_STAGE GetStage() const;
    const T_MEASURE_THEN_WELD_CONTEXT& GetContext() const;
    const T_SCAN_WELDING_PARAM& GetScanParam() const;
    const std::vector<T_ROBOT_MOVE_INFO>& GetWeldPath() const;
    std::string GetLastError() const;

private:
    void SetStage(E_MEASURE_THEN_WELD_STAGE stage);
    bool Fail(const std::string& message);
    void LogInfo(const char* format, ...) const;
    void LogError(const char* format, ...) const;

private:
    RobotDriverAdaptor* m_pRobotDriver = nullptr;
    RobotLog* m_pRobotLog = nullptr;

    MeasureWelding m_measureReader;
    T_MEASURE_THEN_WELD_CONTEXT m_context;
    std::vector<T_ROBOT_MOVE_INFO> m_vtWeldPath;

    WorkflowCallback m_measureExecutor;
    WorkflowCallback m_weldExecutor;
    WeldPathBuilder m_weldPathBuilder;

    E_MEASURE_THEN_WELD_STAGE m_stage = E_MEASURE_THEN_WELD_STAGE::Idle;
    std::string m_sLastError;
};
