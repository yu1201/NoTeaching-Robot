# 控制单元时间轴配置与现场扫描诊断（2026-09-07）

## 配置归属

- 入口：机器人 → 控制单元管理 → 编辑控制单元。
- `robot/<内部名>/RobotPara/BaseParam/ScanTimestampSource`：`robot` / `pc`。
- `robot/<内部名>/RobotPara/BaseParam/StepSdkInterfaceMode`：`timestamp` / `legacy`，仅 STEP 显示。
- 主页移除这两个可编辑入口，只显示当前机器人有效时间轴。
- 旧全局值仅在该机器人字段不存在时作为迁移读取来源；保存只写机器人作用域，禁止再写全局。
- 品牌登记不支持原生时间戳或 STEP 选择旧接口时，固定 PC 接收 steady 时间。运行时还必须检查适配层实际 `RobotTimestamp` 能力，不把 PC 时间称作机器人原生时间戳。
- 时间轴每轮扫描读取一次并冻结；STEP SDK 模式在连接时锁定，保存后需重载或重连，不在运动中热切换。
- 控制单元保存遇到活动机器人任务时拒绝；配置写入失败不得宣称成功。

## 最新现场记录：RobotC / 20260907_001

证据：`Log/2026-09-07/MeasureThenWeldLog.txt` 与该轮 `Result/RobotC/20260907_001`。

- 15:54:12.624 开始扫描，15:54:41.966 确认运动完成。
- 15:55:01.525 安全收枪完成。
- SDK 收到 1762 个新帧，无取帧错误；最长无新帧间隔 51 ms。
- 业务最终仅消费 1 个相机帧，6 个机器人位姿，有效激光点 0，完整点云 1016 点。
- 统计时间对齐因位姿样本 `6 < 20` 未启用，不是相机帧样本不足。
- 15:55:31.264 流程失败；直接原因是点云算法未返回有效焊道点（0 点）。

当前汇川 `GetCurrentPosPassive` 的 robotMs 与 pcRecvMs 使用同一 PC steady 时间，日志标成 robot_ms 有歧义，但不是此次漏帧主因。

主因是同步调用顺序：扫描调用 `MoveLinearMmPerMin`，汇川实现内部等待 `WaitForCommandDone`；返回后业务才记录机器人位姿、标记相机起始队列位置并开始采集循环。运动期间已到达的相机帧被后置队列水位排除。本次时间轴配置迁移不宣称修复该同步采集缺陷；需单独完善异步运动/采集契约并验证中止、终态及时间窗口。

## 保持使能的边界（只读审查）

原生 JOB 正常结束和恢复 main 调度程序没有主动 ServoOff。数据流运动保存进入前 mode/motor；原来 OFF 时退出恢复 OFF。验收则按原始状态恢复。部分 stream-first 组合在进入前会先下电。

可以另行实现经人工确认的生产会话内保持使能，但需先验证 ON 基线下连续数据流、数据流→JOB、JOB→数据流、连续 JOB 的切换。保持 Servo ON 不等于保持数据流 ON；不允许故障、急停或失联后自动反复上使能。本次未改变现场使能策略、未连接或操作机器人。

## 本地双 EXE 交付核对

2026-09-07 用户确认退出后，用当前 `底层优化适配` 工作区同一份源码顺序构建两份本地 Release。版本均为 `2026.08.24.1333`，未修改版本号，未发布安装包、未提交或推送源码。

输出目录：`E:\WorkFile\bowen\QtWidgetsApplication4\x64\Release`。

| 文件 | 修改时间 | 字节数 | PE 产品名 | SHA-256 |
|---|---|---:|---|---|
| QtWidgetsApplication4.exe | 2026-09-07 16:11:41 | 9750528 | NoTeaching-Robot | AE0717E09C2961E0454C33B1CBBC32AE6201B09514D961328364635760415541 |
| HK-Pathlynx-CORPLA.exe | 2026-09-07 16:13:21 | 9750528 | HK-Pathlynx-CORPLA | BBF62C89A33F4418D60ED9306E80D236D066673255F80F292A28BD344A94244E |

已回读核对两份 PE 均为 x64，FileVersion/ProductVersion 一致、OriginalFilename 与对应文件名一致。编译均为 0 错误；警告为现有 STEP SDK 宏/导出与缺少 SDK PDB 信息。

日志：`output/control-unit-timestamp-20260907/build-final-neutral.log`、`build-final-brand.log`。

离线验证通过：`verify_control_unit_timestamp_ui.py`、`verify_robot_runtime_timestamp_scope.py`、`verify_robot_driver_adaptor_boundary.py`、`run_robot_runtime_timestamp_config_tests.ps1`（隔离数据库 seed/restart 与保存失败注入）、`git diff --check`。包含最后补充的重载 token 保护；未进行实机或完整应用 UI 验收。
