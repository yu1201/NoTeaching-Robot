# 机器人适配层契约

`RobotDriverAdaptor` 是程序内唯一允许业务代码依赖的机器人接口。机器人品牌、SDK、
控制器状态码、程序格式、寄存器、原生速度单位和通信细节必须留在派生驱动内部。

## 分层规则

- `src/` 中除机器人驱动实现和控制单元工厂外，不得包含具体机器人驱动头文件、具体驱动类、厂商 SDK 状态码或 `dynamic_cast` 到具体驱动。
- 业务运动统一调用 `MoveLinearMmPerMin`、`MoveJointPercent`、轨迹生命周期接口和标准状态接口。
- 直线速度统一为 `mm/min`，关节速度统一为百分比；厂商单位换算由派生驱动完成。
- 业务判断统一使用 `RobotMotionState`，不得解释厂商原始状态码。`rawCode` 只用于日志和诊断。
- 新功能先扩展适配层契约，再由各品牌驱动实现；不得在业务层增加品牌分支。
- 品牌驱动只能在 `RobotDriverRegistry` 登记；`ContralUnit`、配置页和业务页不得包含具体驱动头文件。

## 强制实现与能力声明

新增机器人驱动必须继承 `RobotDriverAdaptor` 并实现全部纯虚函数。即使某项硬件不支持，
也必须显式返回失败、填写 `GetLastRobotError()`，并且不得声明对应能力。

`DriverCapabilities()` 只声明已真实实现且可以验证终态的能力。业务层按能力开放入口；
没有能力时必须在运动或状态变更前关闭入口或 fail-closed。

完整先测后焊至少需要以下能力：

- `PassiveState`、`LinearMotion`、`JointMotion`
- `ContinuousTrajectory`
- `VerifiedProgramCompletion`、`VerifiedSafeAbort`
- 需要扫描/焊接暂停和断点恢复时：`PauseResume`、`PersistentProgramRecovery`
- 需要真实起弧时：`ActualArcWeld`
- 需要离线生成控制器程序时：`OfflineTrajectoryExport`
- 使用外部轴时：`ExternalAxis`

手眼辅助程序、连续长按点动、原生程序上传和诊断命令属于独立能力；缺少它们只关闭对应功能，
不得影响通用运动、扫描和焊接业务调用路径。

连接/断开、报警复位、伺服上电、工具读取、寄存器读写、示教速度、受验证原生程序执行、FTP、
机器人手眼矩阵读取也分别使用独立能力位。管理测试、CLI、示教和诊断入口与生产流程遵守同一规则：
能力缺失时在调用品牌底层前禁用或拒绝，并显示具体缺失能力。

## 轨迹生命周期

业务层只使用以下生命周期：

1. `ReserveTrajectory` 冻结程序身份。
2. `DownlinkTrajectory` 生成并下发程序。
3. `StartTrajectory` 启动同一程序身份。
4. `WaitTrajectory` 返回标准终态和可验证完成见证。
5. 任何未知、超时或取消路径调用 `AbortCurrentProgramSafely`，只有稳定终态见证成功才可解除运动闭锁。

暂停/恢复必须绑定 `GetTrackedMotionIdentity` 返回的同一程序身份，恢复前还要校验暂停位姿偏差。
不能证明程序身份或自然完成时，不得把停止态当成成功。

## 新机器人接入清单

1. 在驱动层实现全部纯虚函数，并完成标准单位、状态和错误信息转换。
2. 仅为已验证功能设置能力位；真实焊接能力必须包含起弧、灭弧及焊接完成证明。
3. 只在 `RobotDriverRegistry` 登记新驱动，并填写 `RobotDriverSetupProfile` 的端口、监控通道、FTP、
   控制器工程与本地任务目录元数据；注册表会向控制单元工厂和配置界面公开已安装品牌，业务页面无需增加品牌分支。
4. 运行 `python scripts/tests/verify_robot_driver_adaptor_boundary.py`，确认业务层没有越过适配层。
5. 运行机器人专项安全测试和 Release 编译。
6. 上真实机器人前分别验证连接、读状态、单点运动、轨迹完成、安全中止、暂停恢复和断电/重启恢复。

满足全部纯虚契约并声明业务所需的全部能力后，新机器人可复用现有程序功能；只满足部分能力时，
程序只开放已声明能力覆盖的功能，不允许用默认成功或品牌判断绕过缺失能力。

## 当前品牌能力边界

- FANUC 已登记；未声明 `PauseResume`、`ActualArcWeld`、`OfflineTrajectoryExport`、
  `HandEyeMatrixRead`，涉及这些能力的入口必须保持关闭。
- STEP 已登记；未声明 `ContinuousJog`、`NativeProgramUpload`、`DiagnosticCommand`、
  `HandEyeProgramSupport`、`NativeProgramExecution`，涉及这些能力的入口必须保持关闭。
- 汇川未登记，也没有 `InovanceRobotDriver` 底层实现，配置界面不得出现为可选品牌。

汇川《远程以太网控制功能用户手册》4.25.1 已确认端口 2222 的 API/字符串协议可提供：
连接与控制权、用户登录、故障复位、伺服使能、模式和速度、当前笛卡尔/关节/脉冲位置、
MovJ/MovL、数据流、暂停/继续、运动命令编号和完成状态、独立轴、工具参数、系统时间及工程启停。
这些信息足以设计候选驱动，但不能直接声明完整兼容。

汇川接入前仍须补齐并验证以下闭环：

1. SDK 头文件、导入库、DLL 的版本和位数。当前 `ReleaseToUser_2.zip` 内层包有密码，尚不能审查或构建。
2. 数据流轨迹的缓存背压、每条命令身份、自然完成见证、超时后的稳定安全中止。
3. 程序上传后的内容身份回读、精确程序身份以及控制器/软件重启后的持久恢复策略。
4. 真实起弧/灭弧和焊接完成证明；手册只证明运动可带 IO，不能据此声明 `ActualArcWeld`。
5. 控制器时间与 PC 接收时间的精确对应、手眼辅助程序/矩阵变量、整型寄存器命名映射和 FTP 文件语义。
6. 汇川位姿/臂型/外部轴约定与本程序坐标链的实机标定。

因此汇川在完成纯接口、只声明已验证能力、加入注册表并通过 Release/静态/实机测试前，保持
“未安装品牌底层”的 fail-closed 状态。
