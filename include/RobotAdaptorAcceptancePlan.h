#pragma once

#include <array>

// 所有机器人品牌共用的适配验收规范。新增品牌只能在 RobotDriverAdaptor
// 下实现能力，不得复制、删减或改写一套品牌专用验收流程。
namespace RobotAdaptorAcceptancePlan
{
inline constexpr int StageCount = 11;
inline constexpr const char* Revision = "RobotAdaptorAcceptancePlan-20260907-v2";

inline constexpr std::array<const char*, StageCount> StageNames = {
    "0 机器人控制连接测试",
    "1 FTP下载/同名上传/回读闭环",
    "2 生成并下发只加载不运行的测试程序",
    "3 当前位置、关节与完成状态读取",
    "4 低速直线单步移动并返回原位",
    "5 常用接口与寄存器写读恢复",
    "6 工具、运动学与程序资产接口检查",
    "7 二转三/手眼坐标转换验证",
    "8 先测后焊扫描流程",
    "9 实际焊接流程",
    "10 恢复检查与验收汇总"
};

inline constexpr std::array<const char*, StageCount> StageDescriptions = {
    "核对所选机器人和控制端点，经人工安全确认后建立或复用连接，通过品牌适配层完成可复位报警清除、自动模式及伺服上电等前置初始化；不启动运动，初始化失败不能通过。",
    "从品牌底层默认目录递归查找控制器已有测试JOB，选择后先下载，再按原路径同名回传并回读校验，不新增工程文件。",
    "读取当前位置，生成单点低速原生程序并下发；只允许在示教器加载和检查语法，本阶段禁止启动。",
    "连续读取笛卡尔位置、关节脉冲、通用完成状态以及品牌声明的结构化控制器状态。",
    "经适配层检查急停及报警，沿基坐标+Y低速外移后返回原位。另有独立关节J1 +0.5度/1%低速往返专项，两段分别人工确认；直线通过不代表关节通过。",
    "检查状态；逐个备份预留INT/REAL原值、写入测试值并等待现场确认，确认后自动回读比较，再恢复原值并验证。",
    "只读检查Tool、控制器/运行时DH、轴单位、限位、关节直角闭环和控制器程序资产清单；不要求机械运动。",
    "加载本地已验证手眼矩阵，使用当前相机点和当前机器人位姿执行现有二转三，并记录已知点实测误差。",
    "打开现有先测后焊流程做仅扫描/空跑，检查时间对齐、点云、后处理和轨迹，不自动开始运动。",
    "在前序门禁通过且品牌声明实际焊接能力后打开现有流程，由人工执行实际焊接验收。",
    "汇总当前轮次。任何时候都能导出Markdown和JSON；未测试、失败、跳过和能力受限会如实写入报告。"
};

inline const char* Name(int stage)
{
    return stage >= 0 && stage < StageCount ? StageNames[stage] : "";
}

inline const char* Description(int stage)
{
    return stage >= 0 && stage < StageCount ? StageDescriptions[stage] : "";
}
}
