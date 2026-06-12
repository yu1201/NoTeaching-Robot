# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 仓库定位（先读这条）

- **真正的工程根 = `QtWidgetsApplication4/` 子目录**，git 仓库、`.sln`、`src/`、`include/` 都在那里。当前打开的 `E:\WorkFile\bowen` 是其父目录，本身**不是** git 仓库，只散落着备份/中间产物（`_backups/`、`*.obj`、`ConfigStore.db` 副本、`Data*/` 等），不要在父目录里改代码。
- 程序运行时会从 exe 位置**向上最多 6 层**寻找 `QtWidgetsApplication4.sln` 并把工作目录设到该层（`SetWorkingDirectoryToProjectRoot`，`src/main.cpp`）。所有相对路径（`Data/`、`Log/`、`Result/`、`Job/STEP/`、`SDK/FANUC/`）都相对这个根解析；`FindProjectFilePath()` 也按同样的逐级上溯逻辑。

## 这是什么

NoTeaching-Robot（免示教机器人焊接上位机）：**Qt 6.7.3 + C++17 + MSVC2022**，单 exe 桌面应用，面向波纹板（CorrugatedPlate）激光「先测后焊」场景。同时支持 **STEP（新时达）** 与 **FANUC** 两套机器人驱动，UI 全中文、深色主题、适配平板触控。无网络后端，所有状态落在本地 `Data/ConfigStore.db`。

## 构建 / 打包 / 运行

无自动化单元测试框架。「测试」在本项目指：`Debug|x64` + `Release|x64` 编译通过 + Inno Setup 打包通过 + 现场扫描 CLI 验证（见下）。每次发版的构建验证记录写进 `README.md` 和 `docs/worklog.md`。

```powershell
# 构建（在 QtWidgetsApplication4/ 下）——只有 Debug|x64 / Release|x64 两种配置，无 Win32
MSBuild.exe QtWidgetsApplication4.sln /m /p:Configuration=Release /p:Platform=x64 /v:m
#   或用 VS2022 打开 .sln 直接编译

# 两段式发版打包
powershell -ExecutionPolicy Bypass -File scripts\build_release_package.ps1   # → dist\QtWidgetsApplication4\
powershell -ExecutionPolicy Bypass -File scripts\build_installer.ps1          # → dist\installer\NoTeaching-Robot-Setup-v<ver>.exe（Inno Setup）

# 切换 STEP SDK（之后必须重新编译 Debug+Release 两个配置）
powershell -ExecutionPolicy Bypass -File scripts\switch_step_sdk.ps1 -Mode timestamp|legacy

# 旧 INI/TXT 配置迁移进 ConfigStore.db（现场用 ConfigMigrate.exe，无需 Python）
tools\ConfigMigrate_Run.cmd
python tools\migrate_config_to_sqlite.py --source Data --encrypt

# 独立验证可移植滤波模块（唯一的 CMake 子项目，带 example）
cmake -S portable/MeasureThenWeldFilterFit -B portable/MeasureThenWeldFilterFit/build
cmake --build portable/MeasureThenWeldFilterFit/build --config Release
```

**构建依赖路径硬编码在 `.vcxproj` 里**，换机器必须改：Qt `E:\workspace\soft\QT\6.7.3\msvc2022_64`、OpenCV `E:\OpenCV4.6.0`、Eigen `E:\Eigen3.4\eigen-3.4.0`、Orocos KDL `C:\Program Files\orocos_kdl`；STEP `Robot-SDK.lib`/`Robot-SDKd.lib` 需自行放进 `SDK\STEP`。必需编译选项 `/utf-8 /bigobj /Zm2000` 与 `DisableSpecificWarnings 4828` 不能去掉（翻译单元巨大、中文字面量多）。`AfterBuildDebug/Release` MSBuild target 会自动跑 `windeployqt` 并拷贝 `opencv_world460[d].dll`、`SKJCamera.dll`、`qt_zh_CN.qm`，因此裸构建出的 `x64\<Config>\` 已可直接运行。

## CLI 模式（与 GUI 共用同一个窗口对象）

`main()` 永远只构造**一个** `QtWidgetsApplication4` 窗口；带 `--no-show` 时不 `show()`，但仍调用 `ApplyStartupArguments()` → 在事件循环里 `RunCommandLineActions()`（手写参数扫描，非 `QCommandLineParser`，选项值取**下一个** argv）。常用：

```powershell
QtWidgetsApplication4.exe --help-cli
QtWidgetsApplication4.exe --no-show --robot RobotB --robot-movel-relative "0,0,10" --robot-speed 300
QtWidgetsApplication4.exe --no-show --robot RobotC --measure-then-weld-scan-only-repeat 3     # 现场只扫描验证
QtWidgetsApplication4.exe --no-show --laser-classify "Result\RobotB\<case>\LaserPoint\PreciseLaserPoint.txt"
QtWidgetsApplication4.exe --no-show --fanuc-upload-services --skip-upload-wait                # 上传 KAREL/TP 服务
```

完整列表见 `docs/cli-commands.md`。新增 CLI flag 时要同时：在 `RunCommandLineActions` 加分支、在 `--help-cli` 文本登记、并加入自动退出白名单（`hasAutoExitCliAction`）否则进程不退出。`--fanuc-*` 走第一个 FANUC 驱动，不走 `--robot` 选择逻辑。

## 架构大局

整套代码的主线是「**扫描重建焊缝 → 生成补偿后轨迹 → 下发机器人**」，分以下层次（多数逻辑需跨文件阅读）：

**1. 应用外壳（`QtWidgetsApplication4.cpp`，约 14.8k 行）** — 单窗口三页：`m_pAuthPage`（登录）/`m_pDashboardPage`（大按钮主页，面向操作员）在 `m_pMainStack` 内，而**管理页 `m_pManagementPage` 是独立顶层窗口**，自带 `m_pManagementStack` 和菜单栏。各功能弹窗不是自由对话框，而是通过 `PrepareEmbeddedPage` **嵌入某个 stack**；落到主栈还是管理栈由 `m_bOpenEmbeddedInManagement` 标志决定。账号/角色（operator/engineer/admin，`kRole*`，等级 1/2/3）经 `RequirePermission` 把守：管理页需 engineer+，账号管理/调试日志需 admin。账号、登录态、密码、接收模式都存在 `ConfigDatabase`（非 QSettings），密码 `SHA256(user\npassword)`，首启 `EnsureDefaultAdminAccount` 种 admin/admin。

**2. 机器人驱动抽象** — `RobotDriverAdaptor`（基类，声明全套 `virtual` 连接/运动/状态/变量/FTP 接口，并用 **Orocos KDL** 实现品牌无关的 DH FK/IK）。`ContralUnit::InitContralUnit` 读每个单元 `Data\<UnitName>\RobotPara.ini` 的 `[BaseParam]RobotType`：`1`→`STEPRobotCtrl`、`2`→`FANUCRobotCtrl`（`Const.h`：`ROBOT_TYPE_STEP/FANUC`），存为 `T_CONTRAL_UNIT.pUnitDriver`。**全程用基类指针；FANUC 专有功能（常驻服务上传、CURPOS/raw 诊断、固定 TP 点动）必须 `dynamic_cast<FANUCRobotCtrl*>` 并判空。** FANUC 用两条 TCP：S4 控制口（newline 文本协议，`FanucRequest` 经 `m_hMutex` 串行化，出错即 `CloseSocket` 触发重连）+ S5 监控口（后台线程解析 `MON:` 帧填互斥保护的状态缓存，passive 读不占 S4）。STEP 无裸 socket，包 `STEPROBOTSDK::RobotComClient`；连续运动生成 `.srp/.srd` 文本经 FTP（`FtpClient`，WinINet）上传到 `PCRobot` 工程再加载运行。

**3. 配置与持久化** — 历史上每字段一个 INI/TXT，现已**统一进单个 SQLite `Data/ConfigStore.db`**（Qt `QSQLITE`，schema 版本 **4**）。`COPini`（`OPini.cpp`）保留旧 `ReadString/WriteString` 签名但转发给 `ConfigDatabase`；像 `Data/RobotA/RobotPara.ini` 这样的路径现在只是**逻辑键**，磁盘上没有这些文件——**不要再写代码去 fopen/QFile 这些 .ini/.txt**。`ConfigDatabase` 把 `(file, section, key)` 三元组映射成 scope（`global`/`robot`/`workpiece_template`/`result`/`account`）+ module + key 行；`Data/<RobotName>/…`→scope=robot。敏感键（含 password/pass/token/secret）与开启 `encrypt_new_values` 时的新值以 `enc:v1:` XOR **混淆**（非真加密）存储，C++ 与 Python 迁移器实现必须**字节兼容**。

**4. 先测后焊核心管线（`MeasureThenWeldService.cpp`，约 9.2k 行，逻辑几乎都在匿名命名空间自由函数里，公有方法在约 6075 行后）** — `MeasureThenWeldDialog::RunPresetParamFlow` 在 detached `std::thread` 上按序驱动、每步弹确认框：开相机 → `MoveScanStartSafeAndWait`（脉冲安全位）→ `MoveCoorsAndWait(起点)` → **`ScanMoveAndCollect`**（核心，约 6681–7931 行）→ `MoveScanEndSafeAndWait` → `ExecuteWeldPoseFileWithSafePos`。`ScanMoveAndCollect` 并发采三路：机器人位姿（~50ms）、相机帧（`CameraFrameCache`）、帧内激光点；**时间插值匹配是关键不变量**——相机时间戳通过一次性偏移对齐到机器人时间轴，`InterpolateRobotPose` 插出该时刻机器人位姿，`CalcLaserPointInRobot`（手眼矩阵）把相机点变换到机器人/基坐标，早于首个/晚于末个机器人采样的帧丢弃。随后 `AnalyzeMeasureThenWeldPointCloud` 走外部 SDK 或旧算法 → PreservePath + 分类关键点 → `BuildSegmentPoseOutputLines` 分四段算 RZ、套姿态补偿、2mm 加密 → `PreciseLaserPoint_WeldPose_2mm.txt` → `ApplyWeldSeamCompToPoseFile` 焊缝补偿 → `…_SeamComp.txt`（最终执行文件）。`RebuildWeldFilesFromLaserDir` 支持从已存 `LaserPoint` 目录离线重建（跳过扫描）。

**5. 激光/点云处理** — 由 `PointCloudProcessingConfig::Mode` 切换两条路径：`ExternalCorrugatedSheet`（把点云喂给外部 **PointCloudExtration.dll**，`LoadLibraryExW` + 按 **MSVC mangled 名字字符串**解析导出函数，返回的 `ExternalTrackPoint*` 必须用 SDK 的 release 释放；**绝不改 `SDK/PointCloudExtration/` 源码**，只写一份 `*.runtime.ini` 传给 DLL）与 `LegacyLaserPath`（`RobotCalculation` 旧特征点几何）。坡口相机接收两模式：`m_bUseSharedScanCameraReceiver=false`→**TCP 独立，固定端口 50006**（每单元一 socket）；`true`→**UDP 共享端口 50004**，按相机 IP 分发到各单元缓存。可移植 `LaserFramePoint3DFilter`（仅依赖 OpenCV，无 Qt/Eigen）做单帧三维滤波保留最长 2-3 条主直线，是运行时真相来源（`FunctionTestDialog` 里那份 `Deprecated...` 拷贝勿用）。

**6. 手眼标定与几何** — 手眼关系 `robot_local = R_opt*camera + t_opt`，每 (robot,camera) 存 `Data/<robot>/HandEyeMatrix_<CAMERAn>.ini`。`CalcLaserPointInRobot` 是正向 camera→base，`ComputeHandEyeMatrixFromCalibration`（Kabsch/SVD，≥3/6 有效样本，含反射修正）是其逆。**旋转合成随 robotType 不同**：FANUC = `Rz*Ry*Rx`，STEP = `Rx*Ry*Rz`，一律走 `RobotPoseTransform::RotationFromAnglesDeg`。手眼/焊接数学用 **Eigen**，只有驱动里的运动学链用 **KDL**（mm↔m、deg↔rad 转换在 `CoorsToKDLFrame`）。`WeldPoseAverageUpdater` 离线按四段做 MAD 离群过滤的姿态平均，回写 `WeldPoseCompParam.ini`。

**7. 焊接工艺与参数** — 新工艺文件格式（`WeldProcessFile`）：`WeldPara.txt` 每行**严格 84 字段**、`WeaveDate.txt` 每行**15 字段**，TAB 分隔、首行 `USE\t<idx>`、存在 ConfigDatabase（逻辑键），列数不符报「格式已升级，请重新创建工艺内容」。`WeldProcessFile.cpp` 的 `ParseWeldLine/BuildWeldFields` 与 `MeasureThenWeldService.cpp` 里按下标 0..83 的解析器必须**同步改**。真正的 `ARCON/ARCSET/ARCOFF/ARCMODE` + `WEAVEDATA wd0/TRACKDATA td0` + `.srp/.srd` 生成在 **`StepRobotDriver.cpp`**（空跑 `actualWeld=false` 不发 ARC 指令）。`MeasureWeldParam.ini` 按「位置类型」分组（`[MeasureWeldGroups]` + `MeasureGroup<i>.Scan/.Weld`）。补偿分姿态补偿/焊缝补偿，每组恰好 4 段，扁平下标 `g*4+seg`。

## 关键约定与不变量（跨子系统，容易踩坑）

- **分类型码是全局不变量**：`1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise`（`RobotCalculation::LowerWeldPointType` 与所有 `*_Classified.txt`），不要重编号。
- **四类焊段 ↔ 补偿槽硬映射**：`low_platform=0 / rising_edge=1 / high_platform=2 / falling_edge=3`（中文 低平台/上升边/高平台/下降边）。段类由 `AssignSegmentKindsByMeasurementGunDepth` 按枪深投影判定（远=低平台，近=高平台）。
- **采样间距**：焊缝轨迹 `_2mm` 文件按 2.0mm 加密；最终**下发轨迹另按 `dFinalWeldTrajectoryStepMm`（默认 4mm）抽样**，另存 `*_FinalSampled`，2mm 文件本身保持稠密。
- **速度单位随品牌不同**：FANUC 直线命令 mm/s（配置 mm/min ÷60），MOVJ 按百分比（驱动同时接受 `20` 与 `2000` 表示 20%）；STEP 保持 mm/min。一律走 `LinearCommandSpeedForRobot`。
- **时间戳**：FANUC 用机器人侧 `robot_ms`（S5 帧）；STEP 在 `STEP_SDK_HAS_TIMESTAMP=1` 时用 SDK `getTimestamp()`，否则与脉冲/CheckDone 一样暂用 PC `steady_clock`。内部一律微秒 `qint64`。
- **相机 Z 符号陷阱**：`allResultPoint` 的 Z 与 `targetPoint` 相反，线点云入 `CalcLaserPointInRobot` 前要取 `-z`（约 7022 行）；绘图约定 `XData=point.y, YData=point.z`。
- **安全位约定**：下枪/收枪安全位是**脉冲点经 MOVJ**；扫描起点/终点是**直角坐标经 MOVL**，且必须在「测量焊接参数」页示教并保存，否则流程不跑。下枪/收枪安全位强制从世界 X- 方向退刀；STEP 枪轴 -Y，FANUC +Y。
- **RZ 角度必须环绕规范化**：用 `NormalizeAngleNear` 取最近等价、`NormalizeRobotRzOutputRange` 夹到 (-180,180] 且 +180 映射为 -180；RZ 平均要带参考值，勿用朴素均值。
- **`STEP_SDK_HAS_TIMESTAMP`**（`include/StepSdkBuildConfig.h`，由 `switch_step_sdk.ps1` 生成，勿手改）用 `#if` 切换 `StepRobotDriver.cpp` 的真实行为；宏与所链 `.lib` 不匹配会静默走错 API。
- 中文文本统一以 UTF-8 `std::string` 存取；机器人/报警文本经 `DecodeRobotMessageText`（UTF-8→GBK→local 兜底）渲染。UI 跨线程更新一律 `QMetaObject::invokeMethod`。
- **`Data/` 由现场拥有**：安装/升级**不覆盖、不删除** `Data/`（`installer/QtWidgetsApplication4.iss` 排除 `Data\*`，打包脚本剥离 `ConfigStore.db*`），首启自动建空 schema + admin/admin。
- **版本号需多处同步**：`v2026.MM.DD[.HHMM]` 要在 `src/main.cpp`（`setApplicationVersion`）、`src/QtWidgetsApplication4.cpp`、`installer/*.iss`、`README.md` 同步更新，并追加 README/worklog 更新条目。
- git：`.gitignore` 忽略所有 `*.exe/dll/lib` 但**强制纳入** `SDK/STEP/Robot-SDK[d].lib` 与整个 `SDK/PointCloudExtration/`；**安装包不进 git**（2026-06-12 起，经 GitHub Release 分发，`dist/` 整目录忽略）；`*.srp/*.srd` 经 `.gitattributes` 强制 LF。提交一律署名 `yu1201`，不加任何 Co-Authored-By 行。
- **Sk\* 是有意保留的厂商库，勿删**：`SkFunction.cpp/SkDataClass.cpp/SkGrooveRecog_global.h`（SkGrooveRecog 坡口识别）是**相机厂商的源码库**，当前零调用但按用户决定保留备用（实时焊缝跟踪领域模型：坡口间隙/错边输出、跟踪状态机、RANSAC 拟合）。死代码审计时跳过它。

## 数据与产物路径

- 配置：单文件 `Data/ConfigStore.db`（+ 现场账号 `Data/Accounts.ini`、`Data/LoginState.ini` 被 gitignore）。
- 扫描结果：`Result/<RobotName>/yyyyMMdd_NNN/{CameraPoint,RobotPoint,LaserPoint}/…`（`NNN` 三位自增）。`CameraPoint/RobotPoint` 为逗号 CSV，`LaserPoint` 点云为空格分隔；落盘用序号排序（内部仍按时间戳插值）。
- 关键 LaserPoint 文件：`PreciseLaserPoint.txt`（原始）/`_WorkpieceCloud.txt`/`_PreservePath_2mm.txt`/`_Classified.txt` + `_KeyPoints.txt`/`_WeldPose_2mm.txt`/`_WeldPose_2mm_SeamComp.txt`（执行文件）。

## 已知未决问题（`docs/issues.md`）

- 手眼矩阵计算结果存在偏差（#16，本地解算与预期不符，待查解算逻辑/输入点含义/标定基线）。
- FANUC `PR[80]` 读回异常（#17，机器人侧手眼校验仍需联调）。
- 先测后焊 UI 的「线扫处理」入口仍是空 stub（#10）；手眼 ini 读失败会静默回退默认矩阵（#12，需更明显告警）。

## 文档

`docs/` 是 Notion（主协作文档）的手工同步快照，改动代码逻辑/参数/流程时**两边都要更新**并刷新文件顶部「人工整理日期」。索引 `docs/README.md`；流程 `docs/workflow.md`；界面 `docs/ui.md`；CLI `docs/cli-commands.md`；波纹板方案 `docs/wavy-board-plan.md`；问题/工作记录 `docs/issues.md`、`docs/worklog.md`。
