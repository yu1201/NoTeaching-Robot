# 关节运动就绪与先测后焊入口修复

## 问题与证据

- 现场主页面显示缺少“关节运动”，但历史验收 0–7 已通过。
- RobotC 的历史阶段 6 报告记载轴单位、限位和机械模型读取成功；该轮脉冲/关节闭环误差
  为 0.0008 度。当前数据库未配置 `RobotPara/Kinematics`，旧读取只安装到当次驱动内存。
- 主页面把 `JointMotion` 作为无条件入口门禁，验收阶段 8/9 却不检查它；RobotC 当前保存的
  扫描组均为 `UseComputedScanSafe=1`，该路线使用直线运动，但仍需要有效关节读数作翻转检查。

## 固定行为

1. 汇川新控制连接验证登录后，按底层固定配方只读获取 TCP/FTP 机械参数，交叉核对身份、
   结构/零点/减速比/编码器/补偿/限位、静止状态和关节闭环。成功后保存并回读
   `robot/<机器人名>/InovanceKinematics/ValidatedSnapshot`，再发布本会话就绪状态。
2. 数据库快照含设备双端点、型号/固件、源文件及哈希、DH/轴单位/限位和校验结果，只作证据，
   不代替下次连接的实时验证。断线/重载/重新校验撤销旧许可；失败不妨碍仅控制连接诊断。
3. 未就绪时通用关节读取明确失败，不返回厂商绝对编码器脉冲作为替代。通用配置读取也拒绝
   缺项、部分轴参数、零除、非有限比例，避免继承上一轴的数值。
4. 主页、菜单、验收 8/9 共用入口策略；实际扫描按安全位模式检查能力。示教安全位缺关节能力
   在任何运动前拒绝，计算安全位必须成功读取当前关节并检查轴单位，不自动改变路线。
5. 阶段 4 新增独立 J1 +0.5 度 / 1% 速度往返测试；每段分别确认，整段持有操作租约，检查
   起点/单位/限位、发送前位置、终点回读和状态恢复。当前专项保守限制无外部轴。
   可显式结束专项但不自动返回。历史直线通过不等于关节通过，返回点不跨进程恢复。

## 离线验证

- `run_robot_axis_unit_validation_tests.ps1`：有效/反向/缺失/部分/非有限/溢出比例。
- `run_inovance_kinematics_session_tests.ps1`：会话失效、过期结果、静止样本、只读期间排斥。
- `verify_inovance_kinematics_lifecycle.py`：连接/锁次序、快照回读、STOP不阻挡、无绝对脉冲后备。
- `verify_inovance_kinematics_acquisition.py`、`verify_inovance_driver_adaptor.py`：既有配方/协议回归。
- `run_robot_acceptance_joint_motion_tests.ps1`：14 项关节目标与回读算术边界。
- `run_measure_then_weld_capability_policy_tests.ps1`：46 项门禁/腕部单位检查。
- `verify_measure_then_weld_capability_policy.py`、`verify_robot_adaptor_acceptance_flow.py`：共享入口、
  运动前门禁、独立关节记录及跨重启中断处理。
- `run_robot_adaptor_acceptance_store_tests.ps1`：独立进程 seed/restart/verify 和失败注入。
- `run_robot_adaptor_acceptance_layout_tests.ps1`：新增执行/结束按钮，3 种分辨率 × 4 种字体，
  可完整滚动到达，无水平溢出。
- 适配层业务边界、扫描链路、变姿态扫描、暂停/恢复、安全撤离静态回归。

本轮未连接、上电或移动现场机器人。离线通过不等于现场动作已通过；需用新版本重连确认就绪，
再按现场安全条件执行专项或扫描/空跑验收。优化模型仍是候选，本次没有自动启用补偿候选。
