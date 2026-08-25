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
- 适配层只声明功能，不保存或实现品牌通信协议。SDK、Socket、FTP 客户端、账号、控制器目录和程序扩展名
  必须由品牌驱动持有，并由品牌驱动组合调用独立底层组件。
- 连接统一调用 `Connect`/`Disconnect`；业务层不得读取 Socket 配置或调用 `InitSocket`。
- 机器人文件功能统一通过 `CreateFileTransferSession` 返回的适配层会话调用。业务层不得包含或创建
  `FtpClient`，会话 DTO 不得携带 FTP 账号或密码。
- `InitSocket`、`InitFtp`、`UploadFile`、`CallJob`、`MoveByJob`、`ContiMoveAny` 等协议/程序级钩子
  不属于适配层，必须只存在于品牌驱动及其底层组件中。
- 机器人名称、类型、轴单位、运动学参数、工具、外部轴和日志等运行时状态同样通过查询、设置或写入
  接口访问；业务层不得直接读写适配器或品牌驱动成员。

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

手眼辅助程序安装、机器人侧手眼验证、连续长按点动、原生程序上传和诊断命令属于独立能力；缺少它们只关闭对应功能，
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
`StartTrajectory` 不得隐式重新生成或上传；STEP 的 SRP/SRD 内容哈希和大小在 Downlink 时冻结，
Start 前必须再次核对本地内容并回读远端内容，确认一致后才能加载和启动。

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

- FANUC 已登记；运动、轨迹完成、安全中止、FTP、原生程序上传/执行、连续点动、离线 LS/TP
  轨迹导出和手眼辅助程序均通过适配层接入。`PauseResume` 已通过活动任务精确身份、`PAUSE_TASK`/
  `CONT_TASK`、`TSK_STATUS`、`TSK_LINENUM` 和两次稳定位姿闭环接入；`ToolDataRead` 读取
  `$MNUTOOL[1,n]` 的 XYZWPR；`HandEyeMatrixRead` 约定变量 `eye` 为 `R[100]~R[111]`：
  前 9 个寄存器按行优先存放 3x3 旋转矩阵，后 3 个存放毫米平移，读取后必须通过有限值、正交性
  和正行列式校验。现场必须先上传本版本 `FanucServiceLib.pc` 并重启 `STARTALL`，旧服务会因缺少
  `LIB=20260825_ADAPTOR_V3` 能力标识而失败关闭。
- FANUC 当前仍未声明 `ActualArcWeld`、`ExternalAxis`、`OperationModeControl`、`AlarmReset`、
  `ServoPowerControl`。这些能力分别需要现场 ArcTool schedule/灭弧回读、多运动组或扩展轴配置、
  External Mode Select/DCS 安全链以及 UOP/安全 PLC 的动作与状态回读；在获得现场配置和终态证据前，
  相关入口必须保持关闭，不能用通信成功代替功能完成。服务器中旧的 `SET_SPEED`、加载/启动和旧运动
  命令占位 `OK` 已全部改为明确 `ERR`。现有 FANUC LS/TP 生成器只写 GP1/J1-J6，外部轴目标会
  显式拒绝，不能静默丢弃变位机坐标。
- STEP 已登记；原生 SRP/SRD/SR 上传通过适配层组合 FTP 会话实现，原生程序执行通过
  ProgramLoad/START、精确工程与程序身份、控制器错误消息、运行进度和稳定 `eStop` 闭环实现；
  自动生成的扫描/焊接轨迹仍额外保留 SRP/SRD 内容身份和 `ntdone` 自然完成见证。
  STEP 手眼辅助 SRP/SRD 已可安装，但尚未声明机器人侧手眼计算结果验证；连续长按点动也未声明，
  这两个入口保持关闭。
- 汇川已登记为 `ROBOT_TYPE_INOVANCE`，底层直接实现手册4.25.1的2222端口字符串协议：请求
  `@@command$$`、应答 `##result$$`，所有命令由单一互斥请求/应答通道串行执行，不依赖加密SDK包。
  控制入口逐项验证 `CurCtrlDev=2`、`CurPermit=1`、急停释放、故障状态、伺服状态；默认禁止强抢
  其它客户端的许可。初始化会校验当前Tool/Wobj与配置一致，管理级登录密码只留在品牌驱动配置内，
  不进入适配层DTO或日志。
- 汇川当前声明连接、状态、MOVL/MOVJ、数据流轨迹、连续点动、暂停/继续、模式切换、报警复位、
  伺服上电、全局速度、工具读取、P位置寄存器和前三个外部轴。`X,Y,Z,A,B,C` 按手册语义映射为
  通用 `X,Y,Z,RZ,RY,RX`；运动前回读并保留ArmType和未由通用结构暴露的E4~E6。首次现场使用前
  必须以已知姿态核验该映射；关节脉冲运动还必须配置真实AxisUnit，驱动在比例为空时不声明
  `JointMotion` 并失败关闭。`ExternalAxis` 也只在当前控制单元实际配置外部轴时声明。
- 汇川轨迹在 `DownlinkTrajectory` 冻结会话内指纹，`StartTrajectory` 按
  `Get_CurCmdCacheNum` 做配置上限背压，并为每条运动取得新的 `Get_CurCmdNum`；自然完成要求末条
  `Get_CmdSts(id)=1` 与 `Get_MotionSts=0` 连续回读。暂停使用 `Dsmode PAUSE`、同一数据流身份、
  稳定停止和两次稳定位姿；恢复前校验检查点偏差。安全中止要求数据流暂停/关闭后连续回读
  `Get_DsMode=0` 且 `Get_MotionSts=0`，不会把非本驱动数据流的运动伪装成已停止。
- 汇川暂不声明 `RobotTimestamp`、`PersistentProgramRecovery`、`NativeProgramUpload`、
  `NativeProgramExecution`、`FtpFileTransfer`、`OfflineTrajectoryExport`、`ActualArcWeld`、
  `IntegerRegister`、`HandEyeMatrixRead`、`HandEyeSupportProgramInstall`。原因分别是：手册没有提供
  毫秒级同步时间轴；数据流身份不跨上位机重启；没有已验证的FTP/原生工程部署和按名称选程闭环；
  运动IO尚未绑定现场焊机起弧/灭弧反馈；PLC DInt只证明读取未证明对应写入；没有本程序需要的
  手眼矩阵变量与辅助工程契约。相关业务入口必须显示缺失能力，不得用 `Prg Start` 启动“当前工程”
  冒充按名称原生程序执行。
- 汇川已通过C++ Release编译和静态适配契约；真实控制器上的连接、模式、伺服、已知点低速运动、
  缓存背压、暂停恢复、外部轴和断开安全顺序仍是上线门禁。控制器切回示教器前必须先通过适配层
  完成运动中止、`Motor OFF`、`RemovePermit`，再断开TCP连接。
