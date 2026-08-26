# 工作记录摘要

- 人工整理日期：`2026-08-25`
- Notion 页面：<https://www.notion.so/1eb0a83f808e4cdd84d554753436275f>

这份文档按日期整理当前阶段已经完成或已立项的关键工作项，详细表结构仍以 Notion 为准。

## 2026-08-25

- 开发中：修复现场案例 `20260825_030` 搭接位置在焊接姿态/补偿阶段被拟合成直线的问题。根因是平台/坡面姿态补偿交点重建的连续裁剪跨过了 `geometry_lap_step` 两个硬锚点；现在裁剪或交点改写一旦触及搭接点就原子放弃本次重建，生成输出前再核对原始搭接 `raw_index` 是否全部保留，任一丢失则失败关闭并阻止生成跨接直线轨迹。回归门禁纳入 `_030` 的 `raw_index=147/149` 裁剪窗口失败模式。未提交、未发布。署名 yu1201。

## 2026-08-24

- 本地测试候选 v2026.08.24.1333：汇总机器人适配层与关节/脉冲缺省值、模型焊接候选与逆向投影、扫描位姿变化测试、跨 Windows 用户配置修复、FTP 程序数量提醒及源码离线环境配套。机器人/焊接静态安全门禁、84 项配置库回归和三组几何/流程 C++ 测试已通过；双通道 Release 与安装包验证进行中，未提交、未推送、未上传 OTA/GitHub/GitLab。线上 OTA 服务器载荷正常，但 v3 清单 7 天有效期已过，需在用户确认本地候选后随正式发布刷新。署名 yu1201。
- 开发中：增加机器人控制器 FTP 程序数量后台提醒。GUI 启动 8 秒后检查全部已配置控制单元，预设先测后焊和跳过扫描焊接成功完成 2 秒后检查对应机器人；盘点使用独立只读 FTP 会话，不复用轨迹上传会话、不持有驱动生命周期指针，连接或列目录失败只写 `Log/RobotProgramInventory.log`。STEP 只统计 `PCRobot.sr` 中 `.srp` 程序，FANUC 统计 `/md` 中 `.tp/.pc` 并按程序名去重；默认阈值 100，可在 FTP Job 文件管理页写入 `ConfigStore.db`。超过阈值显示非模态人工清理提醒，代码不调用删除接口。未提交、未发布。署名 yu1201。
- 开发中：修复早期 schema v5/auth3 数据库缺少 `legacy_credential_scrub_*` 元数据时，应用内自动修复即使现场没有旧 INI 凭据也被证明门禁拒绝的问题。仅对已通过全部 schema、账号/管理员和 DPAPI 白名单验证的当前库开放空证明补建：实时清理清单必须严格为空，且明文数据库残留检查必须通过；存在旧凭据、部分证明或其他异常仍在备份前失败关闭并保持全部输入不变。真实附件库从 530 条设置安全移除 9 条跨用户不可读 DPAPI 状态，剩余 521 条配置逐字段及时间戳零漂移，1 个账号和管理员保留；Python 与新 `ConfigMigrate.exe` 的修复结果 SHA-256 一致，84 项数据库回归通过。未提交、未发布。署名 yu1201。

## 2026-08-23

- 开发中：扩展 `ConfigMigrate` 的应用内“自动修复”能力，处理完整 `ConfigStore.db` 被复制到另一 Windows 用户后 CurrentUser-DPAPI 绑定不可读的已知场景。恢复器先验证 schema 5/auth3、SQLite 完整性、账号/管理员不变量、旧凭据清理证明和所有非 DPAPI 配置，再要求全部不可读 DPAPI 行同时满足严格白名单；随后创建并回读整库 DPAPI 备份，原子删除不可移植的登录偏好、记忆密码、在线服务/机器人 FTP 凭据和点云证明材料。登录偏好缺省关闭且不在开发机重建 DPAPI，使修复库可搬回目标 Windows 用户后再按当地身份写入；账号密码摘要及普通业务参数逐行保持不变。未知、混合、畸形或不完整状态继续失败关闭。真实旧库副本演练中 9415 行降为 9305 行，删除 110 行不可移植状态，9305 行非目标配置零漂移，82 项数据库回归通过。未提交、未发布。署名 yu1201。

## 2026-07-30

- 版本 v2026.07.30.1643：完成现场坡面焊接姿态、最终姿态回读、点云有效性门禁和参数单位显示修复。焊接姿态由仅调整 RZ 改为按完整工具坐标系随焊道坡度生成，保持焊枪与焊道垂直、与世界 Z 的工艺夹角以及靠机器人侧焊接约束，并兼容 RX 接近 ±180°、RY 接近 0°的等价欧拉姿态；来源绑定优先按物理位姿匹配，控制器欧拉差只记录诊断，错误信息输出实际位移/姿态差和配置门限。精测点云页将焊道与扫描结果有效性检查从机器人流程安全门禁中分离，全部有效性项提供独立开关和可调阈值；更新 20260729 PointCloudExtration DLL/lib，接入 `pixel_num` 短毛刺清理参数，修复现场点云生成异常及非圆弧直线段处理。全软件参数编辑框移除内嵌 mm、deg、% 等单位，统一在编辑框外显示。Release x64 和专项回归已通过，相关现场版本已由用户完成测试并确认发版。署名 yu1201。

## 2026-07-29

- 版本 v2026.07.29.1105：完成机器人模型服务器同步与焊道补偿预览修正。机器人模型库新增服务器模型页，可上传、下载和校验 STEP 总装、J0–J6 碰撞简模及由原始 STEP/B-Rep 生成的缩略图，传输过程显示文件队列、确定型进度、速度和 ETA；服务器模型目录与扫描案例隔离，下载后仍须通过客户端可信型号、revision、适配器及碰撞完整性门禁。低平台、上升边、高平台、下降边的姿态补偿统一使用焊道切向、焊道法向和世界 Z 局部基准，坡面焊枪姿态不再旋转位置补偿方向；平台与坡面补偿后使用无限直线延长求交并由平台锚定交点高程，避免修改坡面时带动平台内部点位。姿态与焊道补偿预览分别显示选中段或整条轨迹高亮、世界合成向量和实际方向箭头，实时修改参数只刷新图层并保留当前视角。相关本地 Release 已完成用户测试。署名 yu1201。

## 2026-07-28

- 已纳入 v2026.07.29.1105：修正姿态补偿段交界的几何耦合及上下坡补偿方向。低平台、上升边、高平台、下降边统一使用焊道局部基准，界面字段改为“焊道切向补偿 / 焊道法向补偿 / 世界Z补偿”；坡面 RZ 夹紧和姿态过渡只影响焊枪姿态，不再把位置补偿旋到焊道反向。平台与坡面先按各自补偿做刚性平移，再以平移后的无限直线延长求交；交点高程锚定平台，裁掉越界点、定步长补齐延长缺口。坡面仍按相邻交点保持直线，平台不再从起点到新交点整段重插值，避免单独修改上升边或下降边时带动相邻平台内部点位。补偿预览与实际输出共用同一路径；姿态模式显示选中段高亮和实际补偿方向，焊道模式同样显示整条轨迹高亮、世界合成向量和实际补偿方向。实时修改参数只刷新图层并保留当前旋转、缩放和平移，首次载入或重新读取时才自动适配视图。署名 yu1201。

## 2026-07-27

- 已纳入 v2026.07.29.1105：机器人模型库增加服务器同步。服务器建立独立
  `/模型文件/机器人模型文件/{assets,collision,previews,catalog}` 目录；客户端可读取不可变版本清单，
  上传/下载 STEP、J0–J6 碰撞简模与 480×320 原始 STEP 缩略图，并显示文件队列、确定型进度、速度、ETA 和取消状态。
  选择服务器型号时显示由原始 STEP/B-Rep 曲面生成的缩略图和大图，不再显示碰撞简模方框；旧版简模
  预览停止展示并提示重新上传同一型号。列表还显示型号、驱动类型、STEP 大小和简模型号；清单、预览及资产均校验
  大小和 SHA-256，下载后仍通过代码内可信型号、适配器、STEP revision、碰撞 payload 与 J0–J6 门禁。
  已将现有 `step.sa10-2000h`（新时达 SA10-2000H）、简模型号
  `1a8f6c2389970fcfaf7bc4e8854a63a6bda2f5c85a16449ab12c550dda2905dd`
  和旧版简模预览图上传到生产服务器，FTP 清单回读、服务器端 STEP/简模/预览 SHA-256、实际中文预览视觉回读
  均通过。Release x64
  构建与四项机器人模型专项测试通过。后续已改为从原始 STEP 生成缩略图；本地 SA10
  103.9 MB 总装实测 61 秒生成 480×320 原始外形图并通过像素门禁，尚需重新上传同一型号替换
  服务器旧预览。署名 yu1201。

- 版本 v2026.07.27.0930：完成本轮现场焊道、连续扫描和在线服务修复。波纹焊道按上坡、下坡、上平台、下平台独立统计段长并合并同类型短段伪拐点，相关阈值进入滤波拟合参数页且无效项动态禁用、保存值不丢失；拐点补充与两类补偿中间文件恢复为折线，仅在全部补偿完成后生成圆弧过渡，并修复最终部分拐点缺弧。STEP 连续模式扫描终点完成判定修正，门禁开关只记录、不阻断流程。远程下载增加队列和确定型进度，恢复自动上传状态同步；仅上传身份可在默认只读状态查看服务器配置，输入本机密码后方可修改。安装器继续排除现场 Data 且不在安装阶段操作数据库。署名 yu1201。

## 2026-07-23

- 版本 v2026.07.23.1745：取消安装阶段的账号与配置数据库迁移。安装器覆盖升级时只更新程序文件并排除整个 `Data` 载荷，不再读取或写入 `ConfigStore.db`，也不再执行迁移、提交、回滚、同版本恢复或数据库状态门禁；无论数据库是否匹配，安装均可完成并启动程序。程序继续使用既有安全恢复页检测不兼容数据库，用户点击“自动修复”后才调用随包安装的受控迁移工具。发版门禁与安装器专项测试改为禁止任何安装期数据库执行链，并验证应用启动不依赖数据库迁移结果。署名 yu1201。

- 版本 v2026.07.23.1530：完成模型驱动焊接、在线服务权限、统一焊道补偿和波纹焊道拐点修复。① 新增 STEP 工件导入、CAD 焊接预览、连续 B-Rep 边吸附、刚性配准、机器人型号/理论模型/碰撞包络管理及执行确认门禁，工件和坡口可独立旋转且保持锚点显示位置。② 在线服务入口改为服务器实时认证，按全权限、FTP 权限、上传权限分层开放升级、数据、配置和管理功能；登录凭据由 DPAPI 保护，上传页提供可恢复的确定型进度、速度、ETA 和队列快照。③ 焊道补偿组迁移为整条焊道一套 Z、枪反向和焊道方向补偿，同时保留姿态四槽与轨迹段属性；安全位回撤可选择自动或固定世界坐标方向。④ 点云查看默认坐标方向修正并归一化相机基向量，不改变点数据和交互。⑤ 波纹焊道按上坡、下坡、上平台、下平台分别统计完整周期段长，首尾不完整段使用局部判据，合并远短于同类均值的成对伪拐点；补偿阶段基于修复后斜坡传播平台类别，现场案例最终焊道不再残留额外双拐点。相关专项回归、Release x64 构建及现场测试通过；未修改本机正式安装版。署名 yu1201。

## 2026-07-17

- 已纳入 v2026.07.23.1530：在线服务认证与上传界面重构。入口改为服务器 FTP 实时验证，固定 `devicedata`（全权限）、`ftpoperator`（FTP 权限）、`uploader`（上传权限）三级账号；在线升级对三级开放，FTP 权限可上传/下载及修改服务器 IP/设备名称但不能查看服务器统计，删除/建目录/账号管理仅全权限开放。配置页移除 FTP 凭据、端口、完整 OTA URL 与管理令牌，只保留单一服务器 IP 和设备名称；验证成功的最近 FTP 凭据继续由 DPAPI 保护存入数据库。上传页增加当前文件、确定型字节进度、速度/ETA 与待传队列，后台状态可在重新进页时恢复。服务器管理服务与离线部署门禁同步支持三个受保护固定账号。未修改本机正式安装版。

- 已纳入 v2026.07.23.1530：测量焊接参数的“焊接姿态”新增安全位世界坐标回撤方向（自动兼容旧算法、X−、X+、Y−、Y+）。缺省继续使用旧 X− 优先逻辑；固定方向解决焊道沿 X 轴时横向安全位方向不稳定的问题。收下枪安全距改为只允许正有限数，运行日志记录实际方向模式，执行指纹绑定该参数，默认配置与迁移模板同步补齐。未修改本机正式安装版。

## 2026-07-16

- 版本 v2026.07.16.1546：修复品牌增量升级脚本因 `QString::arg` 占位编号缺口导致参数整体错位，patch/full 模板统一改为命名哨兵显式渲染，并增加从生产模板真实生成 CMD 的回归门禁。修复 SKJ 相机控制与点云连接争抢：参数及激光命令复用现有 `50006` worker 句柄；图像 `50001` 刷新与点云出帧解耦；先测后焊和时间补偿标定在 runtime 重建后补开实时图像泵。STEP 扫描新增与受跟踪程序精确绑定的安全暂停/继续，暂停期间停止位姿、点云和图像采集并排除活动超时，恢复时丢弃暂停窗口积压帧；运行监控改为由真实位姿和阶段里程碑驱动的确定型数值进度与真实耗时。相关静态安全回归、x64 Debug/Release 编译及现场测试通过；未修改本机正式安装版。署名 yu1201。

- 版本 v2026.07.16.1236：修复 STEP 扫描安全位运动已经真实完成却因程序结束后局部变量恢复初值而误报失败。动态 SRP 在 `WaitIsFinished` 确认运动/收弧完成后发送按程序唯一的 Info 令牌并保留短运行窗口；上位机在正常轮询及 START 后不持 SDK 锁的远端内容复核期间并行锁存，稳定 eStop、同一工程/程序及无错误同时成立才确认成功。每次 START 强制下发连续运行模式并连续三次稳定回读；提前 STOP、断连、身份/内容变化、异常暂停恢复仍自动 STOP/Kill 并失败关闭。方案不调用高频落盘的 `SaveData`，避免控制器存储磨损。同步修复主页手动断连：先停止自动重连监控，串行关闭连接并清空旧快照，界面即时显示处理中/成功/失败及驱动原因，手动重连后重新启动监控。5 组机器人静态安全回归、Debug/Release x64 编译和现场正常扫描验证通过；未修改本机正式安装版。署名 yu1201。

- 版本 v2026.07.16.0010：修复本地 Debug/Release 与正式打包遗漏 Qt TLS 后端的问题，`windeployqt` 保留 `tls` 插件类型并强制校验 `qschannelbackend.dll`，在线服务 HTTPS 管理接口与状态读取不再因缺少后端失败。署名 yu1201。

- 版本 v2026.07.16.0010：主页急停按钮从独立整行移入“机器人监控 / 运行日志”右侧的固定安全栏，充分使用监控区域右侧余量并释放主页纵向空间；“现场小工具”布局保持不变，无活动机器人程序时按钮仍为灰色禁用。署名 yu1201。

- 版本 v2026.07.16.0010：账号认证库安全恢复页增加一键受控修复。按钮只从程序/安装根的可信 `tools` 目录调用 `ConfigMigrate.exe`；开发构建额外核对工具内嵌迁移源码 SHA-256。修复引导不把可写路径插入 shell 源码，而是通过受控环境传值；后台等待当前 PID 退出、取得与主程序相同的机器级互斥后，执行无 `--overwrite` 的加密备份与原子迁移，再以同一数据根重启。新进程只有在 C++ schema/auth/管理员完整性复核通过后才恢复登录，失败则继续锁定并保留日志。旧 schema v4 现场库副本迁移到 schema v5、DPAPI 备份回读、原库哈希不变、Windows PowerShell 5 成功/失败交接实测、80 项 Python 数据库回归、完整 C++ 认证安全矩阵、Debug/Release x64 构建均通过；未修改本机正式安装版。署名 yu1201。

## 2026-07-15

- 版本 v2026.07.15.1519：修复账号管理“修改密码”被固定当成临时密码重置、导致用户使用新密码登录后仍被强制二次改密的问题。① `TryUpdateAccountByAdministrator` 增加显式密码策略参数，管理界面默认把新密码作为最终密码，仅勾选“下次登录强制再次修改密码（临时密码）”时写入 `MustChangePassword=1`；空密码与强制策略的矛盾组合失败关闭。② 新建账号继续使用初始临时密码；账号列表与改密窗口文案区分“首次创建”和“下次登录”，切换目标账号时清空未保存的新密码与策略。③ 动态测试验证两种管理员改密模式、旧密码失效、新密码有效、记忆凭据撤销及异常组合不改变账户状态；完整 C++ 认证 runner、80 项 Python 数据库回归与独立审查通过。署名 yu1201。

- 版本 v2026.07.15.1440：修复 v2026.07.15.1118 登录成功后再次启动被账号库完整性门禁锁定的问题。根因是 `SaveLoginState` 将字符串字段与布尔偏好放进同一个默认 `string` 批写，导致迁移时已规范化的 `RememberPassword`/`AutoLogin` 在首次登录后又被写回 DPAPI `string`。① 新增 `ConfigDatabase::WriteLoginState` 专用原子入口，固定四字段的 string/string/bool/bool 类型及敏感保护，界面与动态测试共用同一生产入口。② 安装迁移器只对已完整证明的 auth3 双字段 DPAPI string 0/1 漂移开放恢复；先验证全部 schema/认证结构并创建 DPAPI 备份，再在私有事务中把两项安全关闭并 CAS 原子发布，账号/Profile 行不参与重写。③ 缺行、混合形态、不可解密、非 0/1 或账号异常均在备份/发布前失败且源字节不变；0 行或部分/完整 canonical bool 保持兼容 no-op。完整 C++ runner、80 项 Python、ConfigMigrate parity、`git diff --check` 与两轮独立 P1/P2 审查通过。署名 yu1201。

- 版本 v2026.07.15.1118：修复 v2026.07.15.0928 覆盖安装后认证库仍进入安全恢复页的问题。根因是旧数据库存在 `LoginState/Settings/AutoLogin` 明文行，而 Python 安装迁移器只迁移 `LoginState/General`，导致安装器校验与 C++ 启动校验语义不一致。① auth 语义升级到 v3，Python/C++ 同时迁移 General、Settings 与现有 canonical LoginState；`UserName`、`AccountHistory` 仅在解码值一致时合并，任何来源/目标冲突、大小写影子或错误 scope 都原子回滚。② `RememberPassword`、`AutoLogin` 不继承旧值，统一写为 DPAPI 保护的 bool 0；auth3 只接受规范 DPAPI 布尔形态，旧 plaintext 发布形态仅允许作为 auth1/auth2 迁移输入。③ C++ 启动路径与安装迁移器统一前向闭锁，schema5 只允许 auth1/auth2/auth3，缺失、损坏或未来版本在事务前拒绝且数据库完整字节不变。④ 使用 `D:\SoftWare\HK-Pathlynx-CORPLA\Data` 的只读副本完成 preinstall、commit、最终 verify 演练，原始数据库 SHA-256 不变；77 项 Python、完整 C++ runner、ConfigMigrate parity、PowerShell 安装迁移与同版本恢复、AppPaths/恢复界面检查全部通过。署名 yu1201。

- 版本 v2026.07.15.0928：完成安装迁移原子事务与同版本恢复的第三轮失败关闭加固。① Python 与 PowerShell/内嵌 C# 的正式库发布、OLD quarantine、失败 staging、DPAPI envelope、解密回读临时库均改为精确句柄绑定；首次绑定必须是单链接普通文件，绑定后才出现的硬链接由同一句柄统一清零并 `FlushFileBuffers`，之后才做 delete disposition，避免目录项删除或进程强杀后通过 alias 留下账号/配置原文。② NEW 已完整回读且受保护备份有效后才进入不可逆 OLD scrub；该提交点之后禁止恢复旧库，raw/empty quarantine、sidecar、事务记录及强杀拓扑都由下一进程按持久记录收敛。③ 安装文件已落盘但数据库 commit/finalize 失败时写入 HKLM64 受保护恢复标记，绑定版本、neutral/brand 通道、规范安装目录、主 EXE、事务路径与 SHA-256；同包重跑在 InitializeSetup 与 PrepareToInstall 双重回读，伪造 `NOOP_CURRENT`、HKCU、异通道、`/DIR` 换目录或事务漂移均失败关闭。④ 手工迁移入口补齐目标 Data 目录安全创建，parity 夹具同步遵守核心迁移器“目标父目录必须预先存在”的身份边界。Python 74 项、PowerShell 安装迁移完整矩阵（460.652 秒）、ConfigMigrate parity、13 个 C++ runner、Windows/Ubuntu/OTA 离线矩阵、中性/品牌 Inno 编译、Release x64 clean Rebuild、AppPaths 与独立 P1/P2 终审全部通过。署名 yu1201。

## 2026-07-13

- 版本 v2026.07.13.2115：完成账号/配置数据库覆盖安装迁移与认证恢复界面闭环。① 安装前只在 `Data` 同目录创建或升级随机暂存库，正式库在新版文件安装完成后通过写穿透原子替换发布；事务记录先于任何数据库变更持久化，并绑定输入目录递归清单、相对路径、长度、SHA-256、正式库/暂存库/DPAPI 备份身份。② `NOOP_ABSENT` 绑定规范空输入，CREATE 全生命周期绑定实际导入输入；恢复、提交、取消和回滚逐阶段回读，拒绝 sidecar、重解析点、伪造状态、晚到凭据、非凭据参数漂移和残留明文备份。③ 发布后凭据清理未完成时进入持久 `PUBLISHED_FINALIZE_PENDING`，事务记录不会提前删除；INI 正常替换与回滚在 Windows 均使用 `MoveFileExW(REPLACE_EXISTING|WRITE_THROUGH)`，清理完成并联合回读为 complete 后才解锁。④ C++ 在任何账号/数据库访问入口检查精确事务记录对象，已打开连接遇到记录也立即关闭；记录移除后必须重新打开并完整验证 schema/auth，pending 或损坏库不能利用连接缓存恢复。⑤ 认证安全恢复页隐藏所有登录/注册输入，仅保留完整说明与关闭按钮，避免小窗口挤压。安装事务 474 秒故障矩阵、schema 31/31、Python/EXE parity、14 个 C++ runner、Windows/Ubuntu/OTA 全量离线回归、真实 ISCC、独立 P1/P2 终审、Release x64 clean Rebuild 与 AppPaths 动态检查全部通过。署名 yu1201。

- 版本 v2026.07.13.0610：完成运动恢复、点云授权、异步生命周期与发版可信链的第二轮系统加固。① 中性 GUI、品牌 GUI 与控制型 CLI 共用机器级单实例互斥，第二进程不能绕过进程内机器人操作租约；持久恢复记录纳入端点别名/索引、工程/程序内容身份和完整快照 CAS，暂停或中断重启在首条运动前必须完成可验证 STOP/Kill/灭弧，STEP SRP/SRD 上传后下载回读 SHA-256/大小，并在 START 后复核窗口保持 STOP 可在 500ms 内完成。② 点云 schema3 质量证明增加 HMAC/DPAPI、授权上下文、执行期租约和可持久回读的 denied tombstone；即使授权文件因独占或 ACL 无法删除，旧授权也不能再次被 Verify 接受。焊接输入、轨迹和恢复数据增加有限值、大小、行数、身份与终态门禁，失败后保持安全闭锁和可恢复收枪。③ 模型对齐、点云读取、BCPD、远端 ZIP/worker、扫描归档和上传统一使用受控取消、进程退出等待、对象所有权与原子提升语义；PLY/点云/轨迹/管理响应加入硬上限。④ 发版链固定可信 Git/GitHub、MSBuild、Qt、Inno Setup、Python/PyInstaller 工具闭包及关键 PE 发布者/哈希，使用净化子进程环境、严格凭据 ACL、SSH 主机指纹和有界 SFTP；GitHub 分支经认证 API 校验后才允许 OTA/GitHub 外发，OTA 升级为签名 v3 清单。⑤ 共享 `uploader` 账号按新策略退出默认入口，设备上传改用独立身份。完整 C++/PowerShell/Python/Windows/Ubuntu 离线矩阵、独立 P1/P2 审查、无并发 Release x64 clean Rebuild 与 AppPaths 动态检查通过。署名 yu1201。

## 2026-07-12

- 发布 v2026.07.12.0606：完成本轮安全与发版治理计划。① 全局机器人操作租约覆盖手动点动、功能测试、先测后焊、断点续焊、虚拟焊接和手眼标定，运动成功必须同时满足任务终态、报警/反馈与目标位置校验；停止、超时、关闭和失败分支阻止后续追加运动并要求安全收枪。② FANUC 实焊增加运行能力与状态契约，未验证时失败关闭；点云 SDK 临时状态按进程隔离，点云质量、有效焊道和工艺上下文成为焊接硬门禁。③ 断点续焊绑定任务/工艺/轨迹身份，按毫米弧长恢复并拒绝错任务、过期或不可验证断点。④ 统一 AppPaths 与配置迁移语义，账号、FTP、管理令牌等凭据改用当前 Windows 用户保护存储，旧数据迁移可验证、失败可回滚。⑤ 发版工具固定中性/品牌历史 AppId `A5A7E2A0-8226-40BB-B126-94C5D298B3CF`，全量 OTA 强制 `/DIR=当前程序目录`；新增 FANUC 12 TP + 9 PC 清单、ConfigMigrate 可复现构建、内嵌版本/哈希/文件清单/双通道体积/远端回读和签名 v2 清单门禁，旧单通道上传入口永久失败。⑥ 在线服务账号事务、只上传 FTP、数据边界、部署回滚与凭据处理加固；Windows/Ubuntu 离线 CI 只验证且不接触生产系统。Release x64、专项静态门禁、凭据/OTA/管理接口/部署离线测试与独立安全终审均通过。署名 yu1201。

## 2026-07-10

- 发布 v2026.07.10.1750（紧急修复全量 OTA 换盘事故）：① 根因实锤——v1643 发版把品牌 iss 的 `AppId` 从 `A5A7E2A0…`（与中性共用的历史值，所有存量装机的注册表键）改成 `F3E0296A…`，Inno `UsePreviousAppDir` 按新 AppId 查不到既有安装 → 落到 `DefaultDirName={localappdata}` 在 C 盘装出第二份（本机复现：D:\SoftWare 旧装 1000 挂 A5A7E2A0 键，1643 全量后 C:\Users\...\AppData\Local 出现新副本挂 F3E0296A 键；升级脚本重启旧 exe → 装后自检报「升级未生效」为标志性症状）。② 修复：品牌 `AppId`/`MyAppGuid` 回退 `A5A7E2A0…` 与中性一致（**AppId 一经发布永不可改**；一台设备只装一个通道，共用无害）；客户端全量升级命令补 `/DIR="当前程序目录"`（`InstallDownloadedPackage`），对手工拷贝部署的设备强制原地安装。③ 在线服务：uploader 只上传账号收紧——账号管理禁用、远程数据锁定本机设备（禁下载/建目录），进入页与重开界面自动刷新；远程数据顶行布局修复。④ 流程测试与先测后焊/断点续焊/相机标定互锁，运行中禁退出/禁升级，关页异步停止不冻结界面。教训记忆 `installer-fullota-dir-trap.md`。署名 yu1201。

- 发布 v2026.07.10.1643：① GUI、CLI、流程测试三条扫描入口统一走 `MeasureThenWeldService::RunScanCycle`，固定执行手眼预检、起点安全位、扫描起点、扫描采集和终点安全位；空脉冲列表失败关闭，扫描已完成时即使数据处理失败也先安全收枪。② 相机启动必须等到本轮新鲜有效帧；生产扫描只接受已存在且通过有限值、旋转正交性、行列式和机器人/相机归属校验的手眼矩阵，不再以默认矩阵兜底运动。③ 停止请求在扫描中延迟到安全收枪后退出，并在焊接下枪安全位、焊接起点和启动轨迹之间逐步骤复检，阻止停止后的追加运动。④ 案例目录与焊接姿态路径分离，上传、循环计数、可选焊接段的成功/失败语义收敛。新增静态连线检查 `scripts/tests/verify_scan_cycle_wiring.py`。

- 发布 v2026.07.10.1007：① 中文设备名 FTP 上传乱码/失败修复——`FtpClient` 远程路径由 W 版 WinINet API 改为 A 版 + UTF-8 字节直通（W 版按系统代码页把中文转成「?」乱码目录、且与 A 版传文件路径对不上导致 550；删掉危险的 `s2w`）。② 「在线服务」重做为云控制台式仪表盘：左侧导航（总览/在线升级/数据上传/远程数据/账号管理/服务器配置）+ 总览页大数字统计卡（磁盘用量+进度条/云端数据/设备数/待传队列）+ 设备资源列表。③ FTP 账号管理（管理员）：经服务器管理接口 `scripts/server/ota_admin.py`（systemd `ota-admin`，nginx 8090 的 `/admin/` 反代 + `X-Admin-Token` 鉴权，令牌在服务器 `/opt/ota-admin/token`）做账号增删/改密码/改权限（仅上传/全权限）与磁盘·设备统计；uploader/devicedata 受保护禁删；客户端「服务器配置」加管理令牌（混淆存储）。④ 远程数据加删除服务器数据包/新建设备目录（支持中文）。⑤ 案例上传改多选对话框（全选/Ctrl/Shift，按名字顺序入队）。⑥ 流程测试页：设置持久化（ConfigDatabase）、小屏自适应滚动、循环扫描数据自动上传钩子（成功/失败都传，非波纹板分析失败也收原始激光点）、定时扫描间隔、包含焊接段开关、与先测后焊同源的位置类型/焊接工艺/姿态·焊道补偿组四选择器（改选即写回当前选用）。⑦ 全对话框统一深色控制台 QSS + 列表/表格选中高亮加强。构建 `Release x64` 0 错误；WinINet A 版建中文目录服务器侧实测正确；管理接口账号全生命周期实测通过。署名 yu1201。

- 发布 v2026.07.10.1006：OTA 增量升级增加「版本兼容判定 + 装后自检」，防止把只含 exe 的补丁装到 DLL 不一致的老设备、以及补丁装了版本没变的坑（1004 那类）。① `latest.json` 补丁块加 `baseMinVersion`（=非 exe 载荷/DLL 自哪个版本起没变的最低版本），客户端「本机版本 < baseMinVersion」时自动回退全量安装（`OnlineServicesDialog::OnManifestReply`）；② 升级前记 `PendingUpdateTargetVersion`，重启后 `QtWidgetsApplication4::CheckPendingUpdateResult` 比对实际版本，未达目标弹「升级未生效」告警（本地演示通过）；③ 发版脚本 `scripts/upload_ota.py`：算每通道「非 exe 载荷哈希」（排除 BUILD_VERSION.txt 等每次变的）、维护服务器端 `payload_history.json`、自动算 `baseMinVersion`（载荷没变一路回溯、DLL 一变顶到那版），`--seed-versions` 回填已知同载荷老版本；密码走环境变量不写死（仓库公开）。只传 103（156 已弃）。构建 `Release x64` 0 错误。署名 yu1201。

## 2026-07-09

- 发布 v2026.07.09.1005：新增管理页「流程测试」（调试菜单，嵌入式管理页）——自动循环跑先测后焊流程用于重复性/稳定性验证。可选目标机器人 / 循环次数（或持续循环）/ 失败即停；参数默认取「测量焊接参数」预设，扫描速度/运行速度/相机时间偏移三项可勾选覆盖；「定时扫描」（两次扫描之间按秒/分钟等待，配合持续循环=每隔 X 分钟自动扫一次）；「包含焊接段」开关（默认仅扫描，勾选后每次扫描完成执行焊接、实际焊接/空跑按预设 `bDoActualWeld`，开始前二次安全确认；确认点循环内自动放行、停止即中止）。后台线程执行、可随时停止（200ms 响应 + 倒计时），相机启动编列主线程。复用 `MeasureThenWeldService`（`LoadPresetParam`/`MovePulseListAndWait`/`MoveCoorsAndWait`/`ScanMoveAndCollect`/`ExecuteWeldPoseFileWithSafePos`）。新增 `ProcessLoopTestDialog.h/.cpp`。构建 `Release x64` 0 错误。署名 yu1201。
- 发布 v2026.07.09.1004：修复扫描数据 FTP 上传始终失败的 bug——`FtpClient` 构造函数不建立连接，而 `ScanDataUploader` 上传预检构造后未 `connect()` 就直接 `createRemoteDirRecursive` 建设备目录，`createRemoteDirRecursive` 首行 `if(!m_hFtpSession) return false` 立即失败，导致**每台设备**都传不上、服务器 `/data` 一直为空（与账号密码/网络无关；WinINet 级实测同机同凭据登录+建目录本可成）。修复：`createRemoteDirRecursive` 会话为空时先 `connectFtpServer()` + 新增公开 `connect()`；上传预检拆「登录/建目录」两步、失败原因分开报（现场可区分密码错 vs 无写权限）。另：管理页「在线服务」由独立弹窗改为嵌入式管理页（`OpenOnlineServicesDialog` 走 `PrepareEmbeddedPage`，`eventFilter` 纳入 Close 名单）。构建 `Release x64` 0 错误、WinINet 级 FTP 连接+建目录实测通过。署名 yu1201。
- 发布 v2026.07.09.1003：客户端默认地址由域名改回 IP `103.217.203.52`。原因：域名 `xiaomomoyun.cn` 未 ICP 备案，国内对「未备案域名指向境内服务器」的 HTTP 请求按 Host 头拦截返回 403（Server: ADM/2.1.1，非 nginx；实测域名 8/8 全 403、IP 8/8 全 200，103 上无 WAF/宝塔——拦截在网络路径按 Host 头判、与端口无关），换 8090 端口也躲不掉。IP 直连不受影响。域名备案通过后可再切回域名享受免重发版。数据盘：103 的扫描数据目录 `/srv/devicedata/data` 已 bind 挂载到 110G 数据盘 `/www/devicedata`（fstab 持久化，FTP 上传实测落盘正确）。
- 发布 v2026.07.09.1002：OTA/上传服务器迁移到 103.217.203.52（三台实测出口带宽 156=0.8Mbps / 106=2.9Mbps / 103=32Mbps，103 最快约 40×，现场下载更新快很多）。客户端默认地址改用**域名** `xiaomomoyun.cn`（`OnlineServicesConfig` UpdateBaseUrl/FtpHost，不再写死 IP，以后换服务器只改 DNS 免重发版），DNS 已指向 103。103 部署同 156（nginx :8090 双通道 + vsftpd 两级账号、共享组 setgid、pasv 公网回址；SSH 端口 48890）。
- 发布 v2026.07.09.1001：`Debug x64` + `Release x64` 编译通过(0 错误)；两段式打包通过；中性+品牌两个安装包+增量补丁均上架各自 OTA 通道；GitHub Release 同步。署名 yu1201。
- OTA 服务器实际部署上线(156.239.225.105，Ubuntu 22.04)：nginx :8090 双通道(neutral/brand)静态更新源 + vsftpd(chroot /srv/devicedata、pasv 公网回址修 NAT) + ufw + 30 天 cron 清理。(8080 被机上 qqbot 占用改用 8090。)
- 在线更新 502 修复(关键)：客户端检查更新默认吃系统代理，现场装了代理软件时代理连不到自建服务器回 502。改 `OnlineServicesDialog` 的 `QNetworkAccessManager` 显式 `setProxy(QNetworkProxy::NoProxy)` 直连；FTP 上传走 WinINet `INTERNET_OPEN_TYPE_DIRECT` 本就不走代理。诊断实测:直连 200 / 经代理 502。
- 通道判据由 exe 名改 `BrandingConfig::IsActive()`：品牌/中性主程序 exe **同名** `QtWidgetsApplication4.exe`(品牌靠 vcxproj `TargetName` 产出 `HK-Pathlynx-CORPLA.exe`、安装器输出名不同)，exe 名不可区分通道；改用 branding/ 随包分发的 `IsActive()` 判定。
- FTP 两级账号:`uploader`(上传专用,vsftpd `download_enable=NO` 禁下载,随包默认;共享组 ftpdata+setgid+umask002 保证跨账号 devicedata 能读回)、`devicedata`(全权限,管理员手填)。客户端默认 FTP 账号改为 uploader(`OnlineServicesConfig`)。
- 上传生命周期与退出交互:`FtpClient` 新增 `uploadFileWithProgress`(FtpOpenFile+InternetWriteFile 256KB分块、进度回调、`std::atomic<bool>*` 取消、取消删半截文件;不动原子 `FtpPutFileA`/STEP 上传);`ScanDataUploader` 加取消标志/进度快照/ETA/`RequestCancel`/`CancelAndWait`;`OnlineServicesDialog::closeEvent` 弹后台继续/停止;主窗口 `closeEvent` 上传中拦退出+进度框(第几案例/文件%/速度/ETA/剩余数)+强退删半截。扫描完 scan-only 也自动上传。
- 打包残留 exe 修复:同一 x64\Release 反复构建品牌+中性会互留残留 exe，`build_release_package.ps1` 拷贝排除列表原本漏加"另一套 exe"(品牌分支排除 qtwidgetsapplication4.exe、main 排除 hk-pathlynx-corpla.exe)。发布流程固化:中性+品牌一次发齐、各上架对应 OTA 通道。

## 2026-07-08

- 发布 v2026.07.08.1704：`Debug x64` + `Release x64` 编译通过(0 错误)；两段式打包通过。署名 yu1201。
- 启动流程健壮性修复(`StepRobotDriver.cpp`)：启动前查机器人状态——暂停态(ePause)程序不响应 STOP，改为跳过停止直接 `ProgramKillCmd` 卸载再启动；切自动失败时 `close()`+`InitSocket` 重连后重试。
- 「流程免确认」勾选框：`ConfirmContinue`/`ShowCheckpointDialog` 按文案白名单放行，跳过中间步骤与信息类确认，保留首次运动/进入焊接/翻转风险告警/跳过扫描目录核对。状态存 `MeasureThenWeld/Runtime.SkipFlowConfirms`(默认关)。
- 相机接收缓冲帧数界面可调：坡口相机预览滑条(2~16，默认 8)，去抖+同值去重提交；扫描运行中只写配置+cache 不下发 SDK(改缓冲会清空未取帧)。经对抗审查修复 3 问题。
- 在线服务(`OnlineServicesConfig`/`OnlineServicesDialog`/`ScanDataUploader`)：OTA 在线升级(清单按 exe 名判定 neutral/brand 通道、优先增量补丁、SHA256 校验、引导批处理静默安装并重启、流程中禁装)；扫描数据 FTP 上传(预设参数+scan-only 完成后按开关打包上传 `/data/<设备名>/`、失败留持久化队列重试、上传前 3 秒 TCP 联网预检不卡流程)；admin 远程数据浏览下载解压到 `Result/Remote/<设备>/`；主页版本号点击→「关于」在线更新。
- 自建 OTA 服务器(`scripts/server/deploy_online_services.sh`，156.239.225.105)：nginx :8090 双通道 + vsftpd chroot + 30 天清理；发布脚本 `scripts/upload_release.ps1`。

## 2026-07-07

- 发布 v2026.07.07.1720：`Debug x64` + `Release x64` 编译通过(0 错误)；两段式打包通过，生成 `dist/installer/NoTeaching-Robot-Setup-v2026.07.07.1720.exe`。署名 yu1201。
- 相机 SDK 更新(v1.2.0 修改时间戳获取方式/按序取帧版)：整包替换(旧接口零删改双验证)。SDK 内部点云改 FIFO 环形缓冲(2~16 帧可设，`SetFrameBufferCount`)，`GetLatestFrame` 语义从「取最新(单槽覆盖)」变「按序弹最旧」；接入：连接后设 FIFO=8、`pollFrame` 改排空式(每 tick 循环取到空，上限16，旧 DLL 行为兼容)、时间戳单位运行时自适应(首两帧差<2000→毫秒×1000归一，≥2000→微秒原样，判定写 CameraSkjClient 日志——防"修改时间戳获取方式"暗改单位打歪统计对齐)。
- 时间对齐算法开关：`UseStatTimeAlign`(MeasureGroup<i>.Scan，默认1)。测量焊接参数→相机时间参数组新增「时间对齐算法」真下拉(开启·统计对齐/关闭·首帧对齐旧算法，照 WeldDirection combo 机制)；`ScanMoveAndCollect` 统计对齐块按此开关，关闭时日志明示对照测试模式。用途：相机新时间戳可行性对照(旧算法直测新时间戳质量)。
- SdkPollLog 增强三列：`recv_frame_seq`(我方成功取帧累计号)、`frame_channel`(SDK帧号——厂商临时以 GetChannel 承载，实测逐帧+1单调递增不回绕，int32 约414天@60fps 才绕)、`channel_delta`(帧号差，>1 即丢帧，差-1=丢帧数——比时间戳差更硬的判据)。worker 在 Release 前读 GetChannel 入 PollStatus。
- 坡口相机预览「图像传输」开关：相机参数区激光按钮下新增 checkable 按钮，即时 Connect/DisconnectImage(50001)；状态经 CameraFrameCache 共享，扫描自动连接同样尊重(关闭后扫描不再连图像口)；worker 新增 slot setImageTransportEnabled + connect/disconnectImageTransport 拆分。
- 点云处理库更新 20260706：仅 DLL/lib/h/pdb(接口 diff 零改动；由新旧 DLL 二进制提取参数名对比证实 26 键全保留零新增——"klrb"系 PDB RSDS 签名字节非参数)，无需动界面/模板/运行时兜底。`tmp_sdk_probe.py` 回归通过(0617 崩溃场景 145 万点 Z=-410 正常返回焊道点；探针临时配置补齐 0624 三键)。
- 丢帧根因排查(证据链完整，结论=相机端点云提取超时跳帧)：①扫描中丢帧率 11.4%(990帧丢127)，49/50 丢帧点间隔期间客户端 FIFO 为空且持续轮询(-106)——非我方取慢；②TCP 可靠传输排除网络；③断开图像传输重扫 11.65% 无改善——排除图像流挤占；④独立 ctypes 探针(FIFO=16排空)单变量实验：空场景(0点/帧)丢帧 1.53%@58.9fps，同位置激光上工件(~417点/帧)丢帧 7.54%@55.5fps——跳号率与相机端提取负载强相关，实际出帧率掉到 55.5 说明相机处理管线掉帧。反馈厂商：优化提取耗时或提供降载选项。丢帧对成品影响可控(点云全帧叠加、统计对齐不依赖连续帧)。

## 2026-07-06

- 发布 v2026.07.06.1806：`Debug x64` + `Release x64` 编译通过(0 错误)；两段式打包通过，生成 `dist/installer/NoTeaching-Robot-Setup-v2026.07.06.1806.exe`。本批合并 07-01~07-06 全部工作(下述各条)。署名 yu1201。
- 统计时间对齐(方案A)：同向重复扫描偏移根因=首帧单点对齐抖动(位姿50ms量化+运行态检测+单帧传输抖动全幅38.5ms，合计±60ms×50mm/s=±3mm整体平移，实测拐点偏移与首帧映射偏移严格线性)。改为整场统计估计：相机→PC钟差取900+帧(帧戳,PC接收)P10低分位(实测相邻扫描互差<2ms)、PC→机器人轴取位姿样本(轴戳,pcRecv)中位差；扫描结束用统计偏移对全部帧重新插值匹配+手眼变换(百ms级)，正向修正时先补采静止位姿延长时间轴防尾帧丢弃；样本不足回退首帧法。日志打印新旧偏移与修正量。`ScanMoveAndCollect`。评审(3员对抗)修复：δ0行进方向符号(CRITICAL，起点轴坐标大于终点的示教会反号发散)、示教安全位序列swap后需reverse(MAJOR)、跳点计数清零、映射日志语义分叉、尾帧翻转计数等。
- 相机时间补偿自动标定：先测后焊页新按钮，一键正向扫→起终点互换反向扫(计算安全位自动按新起点推算；示教安全位列表swap+reverse)→内拐配内拐/外拐配外拐最近邻(±6mm)中位分裂→δ0=split/(2v·u_a)(u_a=行进方向带符号轴分量)→建议补偿=当前−δ0，确认后写入该机器人全部参数组CameraTimeOffsetMs。正向扫后预检拐点≥6不足即中止；门禁：配对≥6、MAD≤1mm、|δ0|≤6mm/(2v)。正反分裂=2vδ的物理：常数延迟正反反号，随机项已被方案A消除故可标。
- 运行监控最大化窗口(`RunMonitorDialog`，cpp内定义无Q_OBJECT)：流程启动自动最大化弹出——流程步骤+进度、实时激光线点云+相机图像、日志、暂停/继续与断点续焊按钮；主界面实时区迁入。点云视图固定标尺(X恒±120mm中心0、Y同比例尺中心首帧锁定、偏超半窗80%才重锁)+严格等比+黑底网格坐标标注+触控按钮调节点大小(±0.5)/视野(±40mm)。图像QLabel设SizePolicy::Ignored修复setPixmap→sizeHint→布局放大的反馈膨胀。
- 暂停/继续与断点续焊(STEP)：暂停=SetModeCmd(STOP)(程序态→暂停(1)可续)，继续=SetModeCmd(START)；暂停时记录getCurrentLine行号+快照位姿并落盘`Data/<robot>/WeldBreakpoint.ini`(重启不丢)；继续前回位检查(偏差>2mm弹警告确认，自动回位待真机验证暂停态命令兼容性)。断点续焊独立流程(主界面新按钮)：只认落盘断点(无记录报错，不用当前位姿兜底防挪枪错位)→自动取最新`SeamComp`→落盘位姿最近点(>20mm判不匹配拒绝)→回退3点(≈5mm搭接)→`ExecuteWeldPoseFileWithSafePos`新参数resumeSkipPoints裁剪records(安全位按续焊首点重推、ARCON自动生成在续焊首点前)→成功清除断点。driver新封装GetCurrentProgramLine/SetProgramLine(SetpcCmd)。`.srp`行↔点非线性(条件ARCSET/WaitTime行)故弃行号映射用位姿匹配。
- 相机SDK v1.2.0(图像传输)接入：旧接口零删除(头diff+DLL导出表双验证)，新增SKJCamera_ConnectImage(独立口50001)/GetLatestImage(SDK内FFmpeg解码BGR24)等13符号，依赖avcodec-61等4个FFmpeg DLL(vcxproj AfterBuild同拷、打包自动带)。worker可选解析(旧DLL自动禁用)；扫描期间按相机参数开关(ImageCaptureEnable默认开)/抽帧间隔(ImageCaptureFrameStride默认5)后台采集存WebP质量80(qwebp插件从windeployqt排除表恢复×4处)，先存Temp扫描后rename进`Result/<案例>/CameraImage/`，文件名img_<SDK图像戳>_<PC接收us>.webp与SdkPollLog时间轴可对齐；保存独立线程(队列上限8丢旧)不拖取帧。实时显示通道CameraFrameCache::SetLatestImage/LiveImageEnabled，坡口相机预览新增「相机图像」页签(主区大图与点云互斥切换，三段线自动切回滤波后)。
- 修复SDK取帧状态被清空bug：运动结束后二次`frameCache->Clear()`连带清poll日志发生在快照前，致sdk_status列恒空、无帧门禁从未生效；拆出`ClearPollStatus()`仅扫描开始调用。SdkPollLog.csv(poll_index,pc_recv_us,pc_delta_us,ret,ret_name,frame_ts_us,frame_ts_delta_us,point_count)每次GetLatestFrame一行，实测定位丢帧：我方轮询0卡顿，145丢帧=72上游(间隔内连续duplicate)+73轮询同频踩点(帧率60=相机60fps拍频，调250后消)。
- 扫描时间轴(robot_ms/pc_recv_ms)主页监控面板新增「扫描匹配时间轴」实时行(每刷新现读配置)；STEP暂停键值勘察：MODEKEY枚举(GBK转码)START=4/STOP=23/MSTOP=100，driver既有Prog_startRun_Py/Prog_stop_Py即START/STOP封装。

## 2026-06-30

- 发布 v2026.06.30.1321：`Release x64` 编译通过(0 错误)；两段式打包通过，生成 `dist/installer/NoTeaching-Robot-Setup-v2026.06.30.1321.exe`。本次提交今日批次：相机取帧状态 CSV + 长时间无帧异常终止、相机读取帧率从测量参数迁至相机参数(`CameraParam.ini` 的 `CameraReadFps`，真正驱动取帧轮询)、调试功能结果打包压缩(zip)。署名 yu1201；其它会话未提交改动(`RobotCalculation`/`PointCloudProcessingConfig`/`LaserWeldFilterDialog`/portable `MeasureThenWeldFilterFit`、点云 SDK DLL)与软著文档/`Job` 散落产物未纳入本次提交(但安装包由当前工作区构建，二进制含上述改动)。
- 相机取帧状态记录 + 无帧门禁：`ScanCameraSkjWorker::pollFrame` 每次轮询把 SDK 返回码记入 `CameraFrameCache` 的 poll 日志(与帧同一 steady 时钟)；`ScanMoveAndCollect` 在扫描运动+尾部匹配结束时快照，统计相邻取到帧之间的最长无帧间断(+末帧到结束的尾部间断)，>1s(或全程无帧)则写盘后报错 `return false` 终止流程。匹配明细 `PreciseLaserPoint_MatchDebug.csv` 末尾追加独立行记录所有非0取帧状态——新增 `sdk_status`/`sdk_recv_timestamp_us` 两列、不复用原 `status` 列。改动 `CameraFrameCache.h/.cpp`、`groove/scancameraskjworker.cpp`、`MeasureThenWeldService.cpp`。
- 相机读取帧率迁移到相机参数：原「测量焊接参数→扫描参数」里的 `CameraReadFps` 实际不驱动取帧(轮询写死 `kPollIntervalMs=10ms`)、名不副实(对抗核查 CONFIRMED：全仓无任何 sleep/节流/SDK 帧率设置消费它)。删除该测量参数(UI/结构体/ini 读写/校验/CLI 日志/Python 迁移器模板全清，旧 ini 残留入 `IsObsoletePreciseParamKey` 保存时自动清除)；在「相机参数→测量相机基础参数」(`CameraBasicParamDialog`/`CameraParam.ini` 的 `CameraReadFps`)新增「相机读取帧率(fps)」，经 `LoadGrooveCameraEndpointForUnit` 换算成轮询间隔(round(1000/帧率))驱动 `ScanCameraSkjWorker` 定时器；`CameraRuntime` 加 `cameraPollIntervalMs` 并纳入 `needRestart`，改值在下次预览/扫描重连生效。默认 100fps(=10ms)保持原行为。改动 `RobotDataHelper.h/.cpp`、`CameraBasicParamDialog.h/.cpp`、`groove/scancameraskjworker.h/.cpp`、`QtWidgetsApplication4.h/.cpp`、`MeasureThenWeldDialog.h`、`PreciseMeasureEditDialog.cpp`、`MeasureThenWeldService.cpp`、`tools/migrate_config_to_sqlite.py`。
- 调试功能「结果打包压缩」：新增 `ResultArchiveDialog`(Qt6 私有 `QZipWriter`，后台 `std::thread` + 进度条 + 取消)，按案例级勾选、按机器人/日期多选，把 `Result/` 打包成 zip，默认存 `Result/Archives/`、可自定义保存位置。改动新增 `ResultArchiveDialog.h/.cpp`，接入 `QtWidgetsApplication4.cpp/.h` 调试菜单与 `.vcxproj`(`QtMoc` 项)。

## 2026-06-26

- 发布 v2026.06.26.1817：`Debug x64` + `Release x64` 编译通过(0 错误)；两段式打包通过，生成 `dist/installer/NoTeaching-Robot-Setup-v2026.06.26.1817.exe`。本次发布合并今日(跨多对话分支)未提交批次：点云预览/焊道补偿界面异步化+进度条、点云解析器重写提速、SDK 库内降采样开关、删除 DelayedLoadingGuard 守卫机制。署名 yu1201；FANUC `SDK/FANUC/laser_macros/`(明图宏去品牌成 AIH_，另一条线)与软著文档/Job 散落产物未纳入本次提交。
- 点云预览(`PointCloudViewerDialog`/`PointCloud3DView`)与焊道补偿预览(`WeldSeamCompDialog`)界面异步化(治 UI 卡死)：原先打开/切目录时在 UI 线程同步读上百万点点云、重算基准焊道+补偿，界面长时间「未响应」。改为先显示界面、`detached` 后台线程读盘解析+计算、按字节/分阶段进度条且可取消；补偿预览把「重算基准焊道→读焊道→`ComputeCompPreviewStages`」整段移后台、抽 `ApplyComputedStages` 与实时编辑预览共用；两者加 `m_destroyed`/`m_workerCount` 生命周期守卫(析构置位并等 worker 归零)。改动 `QtWidgetsApplication4.cpp`、`WeldSeamCompDialog.h/.cpp`、`PointCloud3DView.h/.cpp`、`MeasureThenWeldDialog.cpp`。`Debug`+`Release` 0 错误。
- 点云文件解析重写提速 5~10x：整文件一次性读入内存 + 手写字节分词 + `std::from_chars`/`strtod`(locale 无关、零拷贝、严格)替代逐行 `QTextStream`+正则；带进度回调(每 16384 行回报+可取消)，可在后台线程调用。`PointCloud3DView.cpp`。
- 新增 SDK「库内降采样」界面开关(is_sample/sample_size/above_z)：20260624 版库自带的 ini 采样找平面参数(采样开关/体素大小/抬升值)此前没接进界面(生成 runtime.ini 只「缺则补 false」、「SDK 算法内部参数」组独漏这三个)。按既有模式在 `LaserWeldFilterDialog` 加 3 个控件、随 `m_pSdkInnerGroup` 按模式整体启停、读写 SDK config ini，不碰 SDK 源码/模板。开启「库内降采样」可大幅降低 pcl_kdtree 压力、提速并显著减少崩溃；默认关闭(与厂商模板一致)，需现场实测精度无回退。改动 `LaserWeldFilterDialog.h/.cpp`。`Release x64` 0 错误。
- 移除 `DelayedLoadingGuard` 延时加载守卫、统一「先开界面+后台异步」：清理 24 处调用点(`QtWidgetsApplication4.cpp`、`CameraParamDialog.cpp`、`HandEyeCalibrationDialog.cpp`、`PreciseMeasureEditDialog.cpp` 等)与 `WindowStyleHelper.h/.cpp` 中该类定义(约 -225 行)；配置库查看改后台只读连接查询+UI 分块填充(填充 O(N²)→O(N))；先测后焊 `LoadRobotList`、工艺组合框数据延后一拍加载，主入口秒开。`Debug`+`Release` 0 错误。
- 现场诊断(本批，多为分析未改代码)：①现场「漏拐点」根因=处理方式是 `CloudFit`(几何/DP 检测)，而方位角补拐点(`azimuthStraightenResidual`/`RefineCornersByCoherentBow`)只在 `sdkfit` 路径生效；实测缺失拐点在末段 KP12→端点 Y≈661/X≈1198 处弓出 3.71mm 被直线化抹平(`WeldPose_2mm` 该段残差被压到 0)。建议切 `sdkfit` 或降残差阈值。②现场「程序运行中无法强制关闭、疑似库崩溃」用现场日志定性:点云 SDK 每轮正常完成(~42s/输出 1342 点)、非库崩溃；真因是 STEP 连续焊接轨迹启动后中途停在「暂停(1)」(无报警)→`CheckRobotDone=-1001` 失败→后续「等待程序停止」分钟级超时期间 `m_bRunning=true` 拦住关窗(MOVL 直线运动同轮全部成功，只连续运动暂停)。③进一步确认现场「卡死」可由 SDK 库内去噪 `is_remove_noise=true` 在现场点云上死循环触发(本机样例云上同输入三次输出逐字节一致/确定，仅 ~40s 不卡)——现场临时缓解=关「库内去噪」；并行排查另坐实一处真实卡死隐患(补偿预览 worker 5 分钟 `waitForFinished`+无 JobObject+析构自旋+closeEvent 漏检，已被本批补偿预览异步化改造部分缓解)。`is_remove_noise` 默认值与卡死硬化(超时缩短/JobObject)按用户要求暂不改。

## 2026-06-25

- 发布 v2026.06.25.2139：`Debug x64` + `Release x64` 编译通过(0 错误)；两段式打包通过，生成 `dist/installer/NoTeaching-Robot-Setup-v2026.06.25.2139.exe`(约 61MB)。本次发布合并今日未提交批次：平台重算拐点结构约束、点云查看器增强、点云 SDK 修复版(20260624)、工艺摆动按工艺独立、机器人/相机异步连接(连同当日早些已提交的 pointwise 摆动一并构成 06-25 发布)。署名 yu1201；软著文档/Job 散落产物未纳入提交。
- 点云处理 SDK 更新到厂商修复版(20260624)：替换 `SDK/PointCloudExtration/PointCloudExtration.dll`(795648→794624字节)/.lib 与出厂 config。修复 0617 版在 Z 截断退化输入(实测同工件云+Z截断=-410)下 pcl_kdtree 段错误回归——新版同输入正常返回焊道点。header/导出接口不变、加载器/lib 调用无需改，只换 DLL。**新增 3 个 ini 参数** `is_sample`/`sample_size`/`above_z`(及 `Lines_Dis_Threshold` 500→30、`Step` 5→2)，新版库**硬要求** `is_sample`(缺则 `Para_name is_sample not found!` 返回 count=-1，同当年 is_remove_noise)。三处同步：①仓库出厂模板(新装机 seed) ②`PointCloudExtractionProcessor.cpp PrepareRuntimeExternalConfigPath` 加兜底注入(现场 `Data/` 副本不被安装包覆盖、缺新键时生成 runtime.ini 时自动补) ③本机 `Data/` 副本。注：app 实际读 `Data/CorrugatedSheetPointCloudEctration.ini`(存在则始终优先，仓库模板仅首启播种)。`Release`+`Debug` 验证通过。
- 工艺摆动数据按工艺独立(修「共享 WeaveData[0]」缺陷)：历史上所有工艺共享同一份摆动数据，改一个工艺的摆动会波及其它工艺。改为每个工艺自带独立摆动、`nWeaveTypeNo` 恒等于工艺下标、摆动列表与工艺列表一一平行(`WeldProcessFile::NormalizeWeaveTypeParallel`，幂等迁移历史数据)；新增/删除/分组重排工艺时摆动随之同步重排，保存时摆动与工艺两份都写。改动 `WeldProcessFile.h/.cpp`、`WeldProcessDialog.cpp`。
- 点云查看器(`PointCloudViewerDialog`，QtWidgetsApplication4.cpp)三项增强：①**按处理方法显示全链路数据**——读 `PointCloudProcessingConfig::Load().mode` 按方法列文件(直到焊接姿态)：SDK全处理=完整工件点云+分类+焊接姿态；SDK+拟合=工件+SDK基础焊道+分类+焊接姿态；点云+拟合=工件+原始精确+保留路径+分类+焊接姿态；特征点+拟合=原始精确+分类+焊接姿态(原为写死4文件列表，与方法无关)；缺失文件自动跳过。②**每种点云默认颜色**——`PreciseLaserPoint_WorkpieceCloud.txt` 完整工件大点云**默认白色**(小点1.2px、不连线)，原始浅蓝灰、SDK基础橙、保留路径青、分类彩虹、焊接姿态黄。③**左下「显示设置」控件**(对选中图层实时生效、不重置相机视角)：颜色类型(固定/按顺序渐变→`Layer.rainbow`)、点大小(0=自动→`Layer.pointSize`)、连线显示(`Layer.connectLines`)。`PointCloud3DView` 加 `SetLayersPreserveView`(改样式不 FitToLayers)。改动：`PointCloud3DView.h`、`QtWidgetsApplication4.cpp`(LoadCurrentDirectory 方法感知 specs+默认色+pointSize、左面板加 QComboBox/QDoubleSpinBox/QCheckBox、RefreshProperties 同步控件、新增 ApplyStyleToSelectedLayer)。`Release x64` 0 错误。
- 拐点 inner/outer 结构约束「平台重算」（取代不可靠的"删多点"，治源头多检/漏检）：现场 CloudFit 一次扫描出现拐点过检（同一份输入：补拐点扩 CloudFit 之前 9 个干净拐点，之后 16 个——即上一条的补拐点在该工件起点端区过补了）。先证伪了"事后按规律删多点"：在已过检的拐点集上，prominence/间距都被污染（实测谷1 假点 35.6/116.8 的 prominence 反比真角 45/105 高），任何"挑哪个删"都误删真角。改用**平台重算**：波纹角成对(谷=II/峰=OO)，按类型游程把拐点归平台→每平台在 [上一平台末角,下一平台首角] 区域对稠密路径做「坡-平台-坡」三段直线拟合、两折点=2个真边界角。位置全由几何**重算**而非从污染集**挑选**，故对源头多检(5→2/3→2)、漏检(1→2)一并免疫，搭接台阶点豁免。离线验证 RobotC/20260625_015 过检 16 个 → 稳定还原 `II OO II OO II`（谷1 5→45.3/111.0 对照好case 44.8/110.5；谷2 3→325.9/392.0；峰/末稳定；连旧版漏的末端角也补齐）。实现：`RobotCalculation.cpp` 新增 `FitPlatformTwoCorners`(前缀和 O(1) 段残差 + 网格搜 2 折点) + `RefitCornersByPlatformPattern`，置于 lap 注入后、并重建 `isLapStepKey` 对齐；新 `LowerWeldFilterParams.cornerPatternRefitEnable`(默认开)/`cornerPlatformMinSegPoints`(默认8)。**补拐点 `cornerRefineEnable` 默认改关**（平台重算取代、避免端区过补；需要纯几何补点再开）。界面「滤波拟合参数」新增分组：**「启用平台重算」开关 + 「平台最小段点数」参数**(`Fit/CornerPatternRefitEnable`/`Fit/CornerPlatformMinSegPoints`)。配置 Load/Save/clamp + 映射齐。`Release x64` 0 错误。**待现场重扫验证过检被收敛为 II OO 成对**。
- 补拐点(起终点先验自适应细化)扩到所有几何路径，修 CloudFit 末端漏拐点：现场反馈新版「点云算法+拟合」(CloudFit)末端引出段漏拐点(RobotC/20260625_015)。根因=补拐点 `RefineAzimuthCornersByCoherentBow` 此前**只挂在方位角分支**(SdkBaseWeldFit/skipDenoise)，而 CloudFit/特征点走 DP 几何分支(`BuildGeometryKeyIndexes`+prune，`AnalyzeMeasureThenWeldLowerWeldPathGeometry` 的 else 分支，`inputAlreadyDenoised=false`)、根本没调用补拐点。漏点定位：`FitDebug/fit_all_points.txt` 末段(seg11,Y≈597–678)拟合残差最大(均垂距0.73mm/峰值4.72mm，其它段仅0.1–0.4)，是单侧96%的相干~4mm弓——正属端区缓弓漏检。修复：把补拐点改名 `RefineCornersByCoherentBow` 并**移到 `if(skipDenoise)/else` 之后**，两条几何路径(方位角/DP)统一调用；CloudFit/特征点同样受益，按端区地板0.5mm+单侧门80%可补回该末端弓拐点。仅 `RobotCalculation.cpp` 改动(方位角分支行为不变、移出后位置相对 lap-step 注入不变)。`Release x64` 0 错误。**待现场在 CloudFit 模式重扫/离线重建验证末端拐点补出**。更正：前一条"补拐点仅影响 SdkBaseWeldFit"已不成立(现覆盖全部几何路径)。
- 端区补拐点提到「滤波拟合参数」界面、做成可调分组：原先只有"端区细化地板"一个 spinbox、其余门限写死常量(单侧80%/中段×3/端区20%)。现加：总开关 `Fit/CornerRefineEnable`(关则完全跳过补拐点) + 单侧弓出门(%) `Fit/CornerRefineOneSidedPct`[50,100] + 中段地板倍数 `Fit/CornerRefineMidMultiple`[≥1] + 端区占比(%) `Fit/CornerRefineEndFracPct`[0,50]，连同已有 `Fit/AzimuthRefineFloorMm` 共 1 开关+4 参数。`RefineCornersByCoherentBow` 改读 params(越界由配置层 clamp 兜底)、`LowerWeldFilterParams` 加 4 字段、`Settings` 加 4 键 + Load/Save/clamp、`MeasureThenWeldService` 映射(% 转分数)、`LaserWeldFilterDialog` 加 1 checkbox+3 spinbox(布局在原方位角参数下方、搭接块下移)。改动：`RobotCalculation.h/.cpp`、`PointCloudProcessingConfig.h/.cpp`、`MeasureThenWeldService.cpp`、`LaserWeldFilterDialog.h/.cpp`。`Release x64` 0 错误。
- 上位机自建「点位摆动」(pointwise)开放给先测后焊生产流程：原先 pointwise 摆动只在「虚拟焊接测试」放行、先测后焊主流程遇到会报错拦截（防未真机验证就上线）。现确认开放——`GenerateStepWeldProgramFiles`/`ExecuteWeldPoseFileWithSafePos` 的 `allowPointwiseWeave` 默认值 false→true（`MeasureThenWeldService.h`），先测后焊执行入口（`ExecuteWeldPoseFileWithSafePos` 下枪执行）与焊道补偿后 job 同步生成走默认即放行 pointwise 真机下发；拦截块保留作钩子（传 false 可临时禁用，提示文案改中性）。虚拟测试/CLI 仍显式传 true 不变。提醒：真机首次跑 pointwise 需现场验证速度补偿后实际 TCP 速度、停留 WaitTime 在连续运动里的行为、摆弧倾斜角倒向（均原标「需现场验证」项），建议先小段试焊
- pointwise 摆动姿态(RZ)跨 ±180 边界插值修复：现场反馈先测后焊生成的 Weld srd 里焊枪姿态在拐弯处出现"假翻转"波动(RZ 在一小段内从 -158° 连续扫到 +159° 穿过 0°)。根因=`ExpandMoveInfosByPointwiseWeave` 姿态插值原为朴素线性 `a+(b-a)*u`，当中心线相邻点 RZ 跨 ±180(如 -179° 与 +179° 实际只差 1~2° 但跨边界)，朴素插值走经 0 的 ~358° 长路径、焊枪假性转近一整圈。修复：加角度最近等价插值 helper `WeaveLerpAngleDeg`(差值先折到 [-180,180] 再线性插值、输出夹 [-180,180))，RX/RY/RZ 都改用它。虚拟焊接测试因造的是直线、RZ 恒定不跨界故没暴露。实测同 case：修复前 1538 点有 24 个穿 0，修复后 0 个穿 0
- pointwise 每周期采样点数可配：原 `kPointsPerCycle=16` 硬编码改为工艺可调。摆动参数页加「每周期点数」下拉(4/8/12/16/24/32，机器人原生模式置灰，只 pointwise 用)，复用工艺死字段 `nStandWeldDir` 存储(每工艺独立)；driver 端 `NormalizeWeavePointsPerCycle` 规范化(取 4 的倍数以保证停留落 1/4·2/4·3/4·4/4 相位、最小 4、0/旧工艺回退 16)。实测严格线性：8→733 点 / 16→1464 / 24→2187 / 32→2916
- 版本 v2026.06.25.1122；main 中性版 Release x64 编译通过；品牌版 hk-pathlynx-corpla 同步 merge + 编译 + 打包（本地安装包 `HK-Pathlynx-CORPLA-Setup-v2026.06.25.1122.exe`，按需未推送/未 GitHub Release）

## 2026-06-24

- 主窗口启动慢的**真正主因**修复 = STEP 机器人驱动构造内同步连机（先前"相机异步化"只是次要点，故现场感觉没效果；后用日志时间线定位到主因）：`ContralUnit.txt` 铁证——双机启动 "创建新时达驱动成功:RobotC→RobotA" 恒差整 **5s**、单机同秒完成。根因：`new STEPRobotCtrl`(`ContralUnit.cpp:122`，在窗口构造、`app.exec()` 之前)→`InitRobotDriver`→`InitSocket`(`StepRobotDriver.cpp:1308`)→闭源 SDK `m_pSTEPRobotClient->init()` 同步阻塞 `connect()`，机器人离线时每台挂满 ~5s OS 超时、逐单元串行累加，把主窗口可见时间(show 仅排队、需 `app.exec()` 才绘制)往后推。FANUC 惰性连接、`StartStateMonitor` 仅起线程，均不阻塞。修复：构造不再同步连机，连接移到本就在后台跑的状态监控线程首连(`StateMonitorWorker` 开头调虚钩子 `EnsureConnectionForMonitor`，STEP 重写为 `if(!IsConnected()) InitSocket`)，主窗口立即可见。基类静态 `s_connectDriversAtConstruct`(默认 true) 区分模式：GUI(`main` 检测非 `--no-show`)置 false→构造不连/后台连；CLI(`--no-show`)保持 true→构造同步连，确保 CLI 命令执行时已连上(零回归)。配套：STEP `m_bSocketConnected` 原子化(后台连与 UI/CLI 读写并发)、STEP 析构顺序反转为先 `StopStateMonitor`(join 后台首连)再 `CloseSocket`(防连接/关闭竞态)。FANUC 不受影响。改动：`main.cpp`、`RobotDriverAdaptor.h/.cpp`、`StepRobotDriver.cpp`、`STEPRobotDriver.h`。`Release x64` 0 错误。残留登记：相机 UDP 共享接收分支 `QtWidgetsApplication4.cpp:12207` 的 `Qt::BlockingQueuedConnection` 当前 TCP 模式不触发，切 UDP 模式前需一并改。
- 主窗口启动相机连接异步化（**次要**阻塞点，非主因，见上条）：相机连不上时每个机器人单元干等满 3000ms(SKJCamera `setConnectTimeout`)，多单元叠加可达数秒。根因：窗口构造函数(`show()` 之前)经 `InitializeScanCameraRuntimes`→`EnsureScanCameraRunningForUnit` 用 `Qt::BlockingQueuedConnection` 调相机 `startClient`(`scancameraskjworker.cpp` 同步 `m_connect`)，阻塞 UI 线程直到连上或超时(机器人自身连接走后台状态监控线程、本不阻塞)。修复：`EnsureScanCameraRunningForUnit` 加 `blockingConnect`(默认 true，现有调用不变)，启动路径 `InitializeScanCameraRuntimes` 传 false→`Qt::QueuedConnection` 异步发起连接，主窗口立即显示、相机后台连、状态经 `diagnosticChanged` 反映；扫描前(`clearCache=true`)各处仍同步(注：阻塞只保证"已尝试连接"、不保证连上，故改异步不丢连接保证)。改动：`QtWidgetsApplication4.h/.cpp`。`Release x64` 0 错误
- 方位角拐点「起终点先验自适应细化」（修右段/端区缓弓拐点被直线化抄近路）：现象=「SDK点云算法+拟合」(SdkBaseWeldFit) 下，引入/收尾段的缓弓拐点(离弦偏差仅 0.5~1mm 量级)低于直线化兜底残差(默认 6mm)被并入直线，下发轨迹在端区抄近路（实测 RobotC/20260623_008，Z截断=-400）。根因定位：该模式 `inputAlreadyDenoised=true` 走方位角分支(`BuildAzimuthCornerKeyIndexes`+`SplitNonStraightAzimuthSegments`)，拐点在投影 (s=主轴≈Y, smoothH=第二主轴≈X 横向) 平面按离弦垂距递归劈分——**与 DP/`fitPiecewiseToleranceMm` 无关**(那只在非去噪 else 分支生效)。直接降全局残差到 0.5mm 会把 6→16 个拐点(端区真点与中段噪声混杂，复刻实测)，无法只补端区。新增 `RefineAzimuthCornersByCoherentBow`：粗拟合后对仍偏直的段做按需细化，仅当段内点「一致弓向弦同一侧」(单侧占比≥80%，区分真弓弯与左右乱跳的随机噪声——复刻实测噪声点单侧≈59%被剔、真弓弯≥85%保留) 且离弦峰值超**位置相关地板**(首尾各20%弧长端区用 `azimuthRefineFloorMm` 默认 0.5mm、中段×3，体现漏检拐点多在端区的现场先验) 时补该段离弦最远点，最小段长 4mm 护栏 + 最多 8 轮递归。复刻验证：原 6 强拐点保留、端区净增 4 个真拐点({27,46,646,716})、中段与噪声不动。新增配置键 `Fit/AzimuthRefineFloorMm`(默认 0.5，≤0 关闭) + 滤波拟合参数界面「端区细化地板(mm)」spinbox。仅影响 SdkBaseWeldFit 的 skipDenoise 路径，旧版 DP 分支零改动。`Release x64` 编译通过(0 错误)。改动文件：`RobotCalculation.h/.cpp`、`PointCloudProcessingConfig.h/.cpp`、`MeasureThenWeldService.cpp`、`LaserWeldFilterDialog.h/.cpp`
- SDK 点云库崩溃进程隔离（防崩，根因实测确认）：现场用 Windows 事件日志定位崩溃模块=`pcl_kdtree_release.dll`、异常码 0xC0000005；根因是把 Z 截断值改成 -410(让 SDK 截断后点集为空)后，SDK(PointCloudExtration 依赖的 PCL KdTree)对空点云 `setInputCloud` 不检查直接建树而段错误——而 SDK 是 `Thread_Number=24` 多线程，崩在工作线程，同进程的 SEH/任何用户态 handler 都无法安全恢复(撤销了一版降线程的规避做法)。最终方案=**进程隔离**：在 `main()` 构造主窗口前拦截 `--pointcloud-extract-worker` 子进程模式(只调 SDK 提取、不连机器人、调完即退)，主进程 `ExtractCorrugatedSheetIsolated` 经临时文件传点云/收结果、用 QProcess 跑子进程，子进程崩溃(CrashExit/非0退出码)时报可读错误、主程序(GUI/机器人)不挂。本地用 worker 直接喂这批点云重现 0xC0000005(PCL `empty input cloud`)、确认隔离边界生效。每次 SDK 提取多一次点云文件中转(~2-3s)换稳定性。不改 SDK 源码
- 焊接姿态/圆滑参数下放到「基础工艺参数」：定位姿态参数=STEP srp `Lin/WLin` 第 4 参(原硬编码 WLin=eVAR/Lin=NULL)、圆滑=`OVERLAPREL`(原由测量参数 dStepOverlapRel 提供)。「工艺参数→基础工艺参数」界面新增「焊接姿态」下拉(NULL/可变/恒定/腕关节→srp 写 NULL/eVAR/eCONST/eWRIST，恒定/腕关节为推测待现场验证)与「圆滑(过渡比例)%」；T_WELD_PARA 加 nWeldPostureType+dWeldOverlapRel，工艺文件字段 84→86 **向后兼容**(ParseWeldLine 接受 84 或 86、旧文件新字段用默认 可变/20，BuildWeldFields 始终输出 86，下次保存自动补全)，照 cornerArc 模板贯穿 Const.h/WeldProcessFile/WeldProcessDialog/MeasureThenWeldService/StepRobotDriver；ApplyActiveWeldProcessToPreset 在测量参数段之后调用故工艺圆滑覆盖测量默认；T_ROBOT_MOVE_INFO.nPostureType 默认 0(NULL)保持非焊接点原行为
- 焊接动态特性(DYNAMIC)暂置 NULL：按现场要求实焊 `WLin` 第 2 参由 `dynName(ntdyn0)` 改为 `NULL`，不由程序指定动态、用机器人默认；空跑 Lin/PTP 仍保留 dynName(需要时一行恢复)
- 焊道补偿后同步生成 STEP job：先测后焊(扫描)与跳过扫描重建两个流程，在 `ApplyWeldSeamCompToPoseFile` 生成 `_SeamComp` 后立即调 `GenerateStepWeldProgramFiles`(复用 `--generate-step-weld-program` 同一套)把实焊 srp/srd 生成到焊道同目录(Result/<robot>/<case>/LaserPoint/Weld_时间戳.srp/.srd)，便于提取查看，不必等下枪执行；失败只记日志不阻断焊道
- 版本 v2026.06.24.1305；提交前撤销了 SDK 头(CRLF 行尾误翻)与 Job/STEP srd(运行产物)的非预期改动；main 中性版 Debug/Release 编译 + Inno 打包通过，品牌版 hk-pathlynx-corpla 同步 + 编译 + 打包 + GitHub Release
- 明图 FANUC 示教器宏（MT_*，按《明图激光传感器使用手册-FANUC 高级版》实现）：5 个数学宏 MT_JD(两线交点/公垂线中点)、MT_YX(圆心)、MT_CZ(垂足)、MT_YC(延长点，P2 外侧)、MT_YH(用户坐标系，三种求法) 落在 `SDK/FANUC/mingtu/`，每个 = TP 外壳(`.ls`，AR[1..n]→R[61..68]) + KAREL 内核(`.kl`，GET/SET_POS_REG·GET/SET_INT_REG)，沿用本项目 TP-CALL-KAREL 模式(同 FANUC_JOGJ→LOADJOGBUF)。`oracle.py` 为无真机数学标准答案(Python，自检 + MT_YH 跑 2 万组随机点对拍通过，KAREL 与其同公式)。WinOLPC(`ktrans`/`maketp`，cwd=bin、`maketp` 不带 /config)5 个 .kl→.pc + 5 个 .ls→.tp 全部编译成功，交付件归集 `SDK/FANUC/mingtu/compiled/`(5 .pc + 5 .tp + 5 .ls + `安装说明.md`)；MT_YX 内核原变量名 `by` 撞 KAREL 保留字(FOR..BY)已改名。可走上位机现成 FanucCompileKlToPc/LsToTp + FtpClient + 常驻服务 部署。待办：MT_SEARCH(寻位)/MT_SUB_WELD(多层多道)未做(涉 SENSOR SEARCH/Track/Weave/MP Offset 等 TP 指令)；残留 MT_YH 写 `$MNUFRAME` 写保护待控制器确认(暂写 PR[96] + `UFRAME[号]=PR[96]` 应用)。无真机，数学正确性靠 oracle 对拍
- 上位机自建「点位摆动」(pointwise 非标摆动)：STEP/新时达机器人原生摆动不支持拐弯花样，故由上位机在算出的中心线轨迹上叠加摆动、生成密集点列下发。几何放基类 `RobotDriverAdaptor` 的 static 方法 `ExpandMoveInfosByPointwiseWeave`(品牌无关，STEP/FANUC 共用)：按弧长参数化(相位 φ=2π·s/λ，节距 λ=v/f)、切向 T(相邻点差分)+横向 N(worldZ×T)标架、每周期 16 点，位置与姿态(RX/RY/RZ)都从中心线插值。4 波形：正弦(横向 A·sinφ)、三角(横向三角波)、纵向往复挑弧(eBackForward 纯切向前后折线)、L摆(eLTriangle 占位待安川/新时达手册)
  - 速度补偿：摆动 TCP 路径比中心线长，单独模块 `ApplyWeaveSpeedCompensation` 算伸长系数 k=Σ摆动点距/中心线长，两速度字段(STEP `dWeldSpeedMmPerMin` + FANUC `tSpeed.dSpeed`)都乘 k 保行进速度恒定；送丝跟电流电压故乘 k 不动热输入
  - 摆弧倾斜角(`dSwingDirectionDeg`)：横向方向绕焊道切向旋转 swingDeg，0=水平摆 / 45=斜面摆(直角内角焊缝把摆动面摆到 45° 平分、与工件两面成等腰直角三角形) / 90=竖直摆，符号定倒向
  - 摆动停留：相位每跨一个 k·π/2(1/4波峰/2/4中/3/4波谷/4/4中)在该点插 STEP `.srp` `WaitTime(INT变量,TRUE);` 完全停 nPauseTime1-4 毫秒(`.srd` 建 `INT` 变量，照真实样本 `test123.srp` 写法)，仅 >0 才生成指令
  - 圆滑：摆动点 OVERLAPREL 沿用工艺填的值、不强制 0(用户要可调，提醒圆滑大会把密集摆动点磨成圆弧削掉摆幅)
  - 来源开关独立：复用工艺死字段 `nWrapConditionNo`(0=机器人原生走 WEAVEDATA / 1=上位机自建)，不占 `nWeaveType`(原生运动学方式)；生产先测后焊流程遇 pointwise 报错暂不开放，仅虚拟焊接测试/CLI 放行
  - 界面：摆动参数页加「摆动实现」下拉，切上位机自建时摆弧形状只留正弦/三角/纵向往复可选(L摆 + 其它原生波形置灰)、用不到的参数置灰，并补 `QComboBox:disabled` 样式让深色主题下置灰可见；摆幅/频率/结束长宽编辑框加下限 0(摆弧倾斜角保留可负)
  - 测试支架：新增独立 `VirtualWeldTestDialog`(从先测后焊抽离)走全管线一键生成 `Weld_*.srp/.srd` 验证；CLI `--test-pointwise-weave [shape amp freq]` 离线造直线跑真实摆动算法输出对照(`WeaveTest_centerline/weave.txt`)；离线已验证正弦/三角/纵向往复三波形正确
- 工艺参数界面完善：① 加「机器人」选择下拉(工艺库按机器人 scope，原先界面无法切机器人致显示"机器人下面没有工艺")；②「焊接姿态」改名「姿态参数」+ 新增「动态特性」下拉(NULL / ntdyn0，复用死字段 nWeldMethod 存储，控 WLin 第 2 参 DYNAMIC)；③ 全量审计工艺参数消费(约 90 字段)：真用 ~21 / 仅原生下发 ~42 / 界面能填但完全没用的死字段 ~27(含包角整组 18 个、横向/竖向补偿、焊接角度/倾角、立焊方向等)，供后续隐藏或复用参考
- 焊道补偿「精简轨迹」开关：焊缝补偿界面加复选框，开启后最终下发抽样只保留特殊点(起终点/拐点/段边界/圆弧过渡边界/搭接台阶点)、丢弃中间普通加密点，相邻特殊点间走直线插补，随焊道补偿存盘(`SimplifyKeepAnchorsOnly`)；配套把先测后焊主界面里重复的「最终轨迹点间距」编辑框移除(统一到工艺/补偿界面调整)
- 版本 v2026.06.24.2359(承 1305，本批：明图 FANUC 宏 + pointwise 上位机摆动全套 + 工艺界面完善 + 焊道精简轨迹)；中性版打包时发现并修复跨品牌污染——`build_release_package.ps1` 会把 `x64\Release` 残留的另一品牌 exe(`HK-Pathlynx-CORPLA.exe`)一并打进中性包，已清残留重打、核对包内零品牌痕迹；开发机 STEP SDK 已切回 timestamp(新版 2.4.2)；main 中性版 Debug/Release 编译 + Inno 打包通过

## 2026-06-23

- 板材搭接 X 错位台阶检测与特殊拟合（默认关，仅滤波拟合几何流程）：现场两板拼接在侧向 X 出现未对齐错位（实测 RobotC/20260623_008 焊缝中心线 y≈-198 处 X 跳变约 1.79mm，整条缝仅此一处），原拟合把台阶当噪声/缓变抹平或抄近路。新增「检测 → 错位两侧点云分别拟合 → 错位处保留垂直 X 台阶（不做圆弧/斜坡过渡）」
  - 判据选型（多智能体并行 6 判据 + 合成数据对抗验证）：选「双侧平台最小二乘」——候选过渡中心两侧各留 gap 取窗口对侧向 h 做直线拟合，要求两侧斜率近 0（平台，区别于拐角斜边 |k|≈0.35，裕度高一个量级）+ 残差 rms 小（排波纹/噪声）+ 中心两线高度差>阈值（台阶高），沿主轴 NMS 吸收过冲毛刺。淘汰项：sum|Δx| 累积（噪声逐点累加成假台阶，σ=0.08mm 平台 96% 误检）、z-score 信噪比（低噪声段噪声地板失效产生虚警）、去趋势曲率（触发依赖过冲毛刺、对单调台阶系统性漏检）、RANSAC 断点（对过渡锐度敏感、展宽斜坡漏检）。一律用未平滑原始 h（smoothH 均值平滑会把约 2mm 窄跨台阶削平致漏检）。实测唯一命中 y≈-198、零误检（8 个真实拐角不误判）
  - 下游执行安全（对抗审查 RZ/段类型/补偿/抽样/srp 五环节，根因统一）：台阶端点被赋 InnerCorner/OuterCorner，原下游全靠 pointType 含 corner 间接识别 → 圆角化/抽样/补偿会把垂直台阶磨平/斜穿。根治：`WeldPoseFileRecord` 增 `isLapStep` 字段 + 落盘第 14 列 `is_lap_step`（向后兼容旧文件，13 列旧文件解析为 false），台阶身份贯穿全链路；据此豁免/强保留——圆角候选否决 + 平滑/交点重建跳过（垂直台阶不被圆弧化）、4mm 最终抽样去密/闸门/收尾对台阶两点强制成对保留（防 1.82mm<4mm 丢第二点致焊枪斜穿台阶）、焊缝补偿 + 姿态补偿强制台阶两点同槽（防跨 low/high 槽不等位移斜切台阶/叠加额外 Z 差）、srp 连续运动台阶点 `dOverlapRel=0` 精确转角（防 20% 转弯区把台阶两近 90°角混合圆化）、平台直线拟合窗口排除台阶点（防侧向跳变污染 junction 交点）。台阶段 RZ 沿用上一平台段（段方向≈侧向，按焊道法向算会 ~90° 突跳，改为焊枪保持平台姿态横移跨台阶）、kind 继承平台不当独立焊段
  - 参数开放：检测 4 参数（台阶高门 / 拟合窗口长度 / 平台残差上限 / 平台斜率上限）+ 总开关，加到「精测点云处理」→「滤波拟合参数」界面，存 `ConfigStore.db`（Fit/LapStep* 键），默认关
  - 实测验证：rebuild RobotC/20260623_008，开启后 KeyPoints 出台阶双点（source_index 739/740，X 1307.53→1309.32，source=geometry_lap_step）、`is_lap_step=1` 一路流到最终执行文件 SeamComp、台阶 X 垂直保留（未圆角化/未抽样抹平/未斜切）、RZ 无 90° 突跳（仅 3° 两平台自然差异）；默认关复验行为不变。版本 v2026.06.23.1854；main 中性版 Debug x64 / Release x64 编译 + Inno 打包通过（NoTeaching-Robot-Setup-v2026.06.23.1854.exe）；品牌版 hk-pathlynx-corpla 同步 merge + 编译 + 打包 + GitHub Release

## 2026-06-18

- 点云框选编辑增强（参考 CloudCompare/MeshLab/Blender）：双行上下文工具栏（进入编辑才显示编辑条，删除/恢复等危险操作红色右置）；新增套索选（自由多边形圈选 PNPoly + 透明 overlay 不触发 GL 重绘，Shift 减选）、反选/全不选/隔离显示（shader discard 仅渲染选中红点）；「清理▾」下拉的 SOR 统计离群 / 连通域去飞点配非模态参数面板（预览标红→确认删除，选择与删除解耦、删除可撤销）
- SOR 去噪提速约百倍 + 后台化：网格 cell 估算从 span.norm()/cbrt(n) 体密度改为最大两维面密度（激光扫描工件点云近似 2D 表面，旧式高估格距约 28× 致每格点数爆炸→邻域候选膨胀），目标每格点数随 k 放大使多数点 r=1 命中；16M 格上限改放大 cell 重算而非静默 return 0。SOR/连通域改后台线程计算 + WindowModal 进度条可取消，不卡界面（算法拆静态 ComputeXxxMask 吃坐标快照 + UI 线程 ApplySelectionMask 校验 size，m_bEditBusy 闸门 + 析构触发取消防退出冻结）
- 点云分离图层（固定双层）：「图层▾」分离选中 / 切换编辑图层 / 合并 / 显示另层，可把可疑区切到另一图层单独跑 SOR/连通域去噪不误伤主体、最后合并；GLWidget 裸指针恒指固定成员 m_editCloud、切层用 swap 换内容杜绝 QVector 重分配悬垂；第二组 VBO + shader uBackgroundMode 渲染暗灰只读背景层；重新生成合并可见层、隐藏层视为逻辑删除
- 点云处理 SDK 更新（20260617）：替换 PointCloudExtration.dll（795KB）/.lib，导出接口逐字节未变（C++ 调用与按 mangled 名动态加载均无需改）；config 模板新增库内去噪开关 is_remove_noise（默认开），「新版点云库」SDK 参数界面加可调复选框；生成 runtime.ini 时检测缺失即兜底注入 is_remove_noise=true（与 LOGPATH 同模式），根治现场旧配置缺该键报 `Para_name is_remove_noise not found` 致先测后焊/离线重建中断
- 先测后焊拐点错位/漏检/抄近路修复（SdkBaseWeldFit 方法，SDK基础点云+程序滤波拟合）：多智能体对抗诊断 + 实测确认根因在程序滤波拟合环节——对 SDK 已清洗的干净稠密焊道仍套用为带噪相机轨迹设计的「MAD去噪→7点平滑→Douglas-Peucker 取离弦最远点→prominence 判内外角」链；DP「离弦最远点」非真折角顶点，缓变/台阶段拐点系统性偏移（实测某外拐角标在 y=440、真折角在 460、偏 19.7mm），平滑削圆尖角致真折角漏检 → 焊道抄近路（侧偏 7.7mm）+ 多标伪拐角。先后排除「只跳去噪」「调 DP 容差」（实测两头不靠：调大漏检、调小多检波纹起伏伪拐角）。最终：SDK基础点云+滤波拟合流程固定走「跳去噪/平滑 + 方位角拐点检测」（s-h 投影面局部最小二乘拟合左右段方向→有符号转角→22°阈值候选→同区域保留转角最大 NMS + 段内偏离弦>6mm 直线化兜底），判方向转折角度而非离弦距离、天然区分真折角与波纹周期起伏；顶点仍由 BuildFittedGeometryKeyPoints 局部求交精修、内外角仍 GeometryCornerType 判定。方位角 4 参数（转角阈值/拟合窗口/NMS弧长/直线化残差）从硬编码改为 PointCloudProcessingConfig 配置项、映射进 fitParams，开放到「滤波拟合参数」界面可调。现场实测重建 RobotC/20260615_005：拐角由 16(含 -462/8.15 两伪拐角 + y=440 错位 19.7mm + 漏 460 真折角) 纠正为 14、逐个对齐 SDK 提取的真实拐角、y=460 真折角检出、伪拐角与抄近路消除（真机 C++ PCA 投影验证，非离线近似）
- 版本 v2026.06.18.1808；main 中性版 Debug x64 / Release x64 多轮编译通过；本批 8 个提交已推送 main

## 2026-06-16

- 日志系统统一与按天归档：10 类日志统一走 RobotLog，按天归档 Log/<yyyy-MM-dd>/；新增线程安全 writeLine（毫秒时间戳、不走 printf、跨天自动换目录、打开失败可恢复）。修复 CLI 日志混写进机器人A单元日志（独立 CliLog.txt）；查看器按天重定向 + fromUtf8 + 48KB 截断对齐行边界；RobotLog::write 的 printf % 加固（OPini/FTPClient 运行时串改 write("%s",...)）
- 点云读写性能优化：完整点云写盘分块批量；LoadIndexedPoint3DFile 改 readLine/readAll + strtod 手扫，消除每行 QRegularExpression 编译（百万行级提速）；离线重建加读写速率日志
- 先测后焊离线重建容错：激光特征点文件为空（如相机坡口识别全失败）时不再直接失败，点云链方法改用完整点云重建，与实时扫描 canUseExternalCloud 判定一致
- 本地品牌覆盖（中性支持，git 默认呈现不变）：新增 BrandingConfig，工程内 branding/（.gitignore 排除）存在则用品牌名+图标，否则默认 NoTeaching-Robot + 原图标；管理页桌面图标底色开关，切换刷新窗口/任务栏；桌面/开始菜单快捷方式按品牌名重命名 + 换图标（COM IShellLink）；.rc/app.ico/.iss/.qrc 未改，exe 内嵌图标不变
- 中性包不夹带 branding/：打包脚本仅在 branding/ 被 git 跟踪时才拷贝，故 main（中性）安装包不带品牌资源，品牌版改由独立分支提供
- 主页标题精简：先把主页大标题改走 BrandingConfig::DashboardTitle 可定制，后直接移除（与顶部窗口标题栏重复）；主页第一行改为「版本徽章 + 当前用户 + 机器人选择器」轻量状态条，副标题与流程大按钮上移（DashboardTitle API 与 branding.ini 该键随之失去消费点，保留无害）
- 行尾根治：新增 .gitattributes（* text=auto eol=lf + 二进制白名单 + SDK 整体排除），一次性 renormalize 全部自有源码为 LF，并设 core.autocrlf=false，根治 autocrlf 叠加历史混杂行尾导致的整文件 CRLF/LF 翻动；srp/srd 仍按机器人识别要求强制 LF，SDK 厂商库一字节未动
- 品牌版隔离：全品牌 HK-Pathlynx-CORPLA / 海瞰智焊 走独立分支 hk-pathlynx-corpla（含 branding/、品牌 iss/vcxproj，exe 文件名 HK-Pathlynx-CORPLA.exe），安装包经该分支 GitHub Release 分发；品牌版按「每天最后统一更新」节奏同步，不逐次跟随 main
- 版本 v2026.06.16.1411；main 中性版 Debug/Release 编译与 Inno 打包验证（去大标题、行尾根治后复验 Debug+Release 通过）；品牌版 Release 编译通过

## 2026-06-15

- 坡口相机数据协议升级适配（SKJF）
  - 现象：相机固件升级后，坡口相机预览/扫描收不到点云。抓端口实测帧头变为 `53 4B 4A 46`（"SKJF"），旧接收按旧 magic `0xABCDEF12`(PointCloundResultFrame) 解析、永远找不到帧头（接收计数在涨、解码 0 帧）
  - 厂商提供新版 SKJCamera SDK（句柄式 C ABI，`SKJCamera_GetLatestFrame` + `SKJFrame_*` 访问器，内部自管 TCP 连接/接收线程/SKJF 解析）；工程 `SDK/SKJCamera` 已是含数据接口的新版 DLL，且参数控制(SKJCameraControlClient)早已在用同一 DLL
  - 新增 `ScanCameraSkjWorker`（QLibrary 动态加载，QTimer 10ms 轮询 GetLatestFrame → 转 udpDataShow → 填 CameraFrameCache），与旧 worker 接口同构（同 diagnosticChanged 信号）；CameraRuntime 的 TCP 独立模式改用它，旧 ScanCameraTcpClientWorker 保留不删。预览/扫描下游零改动
  - 相机时间戳实测为微秒（两次会话首帧时间戳差 52,951,358 与墙钟 52.95s 一致；SDK 文档标称毫秒有误），直接填入与机器人微秒时间轴对齐——切勿 ×1000
  - 验证：预览出点云、FPS 60、完整点云连续、参数控制正常
- 机器人姿态插值 ±180 环绕修复（影响扫描精度的真实缺陷）
  - 用完整点云逐帧调试数据（每点带机器人位姿+时间戳）定位：完整点云起/终点翘起散点的帧，rx 是乱值(27/-121/-89/-5…)，而正常帧 rx 恒为 180、ry/rz 与相机原始坐标都正常
  - 根因：`RobotCalculation::InterpolateRobotPose` 对 rx/ry/rz 朴素线性插值，姿态角跨 ±180 抖动时把 179 与 -179 插成 0（错误中间角）→ 那几帧点云整体变换偏。起/终点姿态微调跨界触发，中间匀速段 rx 稳定不跨界故 0 散点
  - 修复：rx/ry/rz 改最近等价角插值（差值规范化到 (-180,180] 再插），不跨界时与原结果完全一致；修正所有扫描在姿态跨界帧的位姿插值（完整点云、焊缝识别点、轨迹生成同受益）
- 点云提取库路径回退兜底
  - `PointCloudExtration.dll`/config 在配置目录（数据库可能残留旧部署绝对路径如 `B:\NoTeaching-Robot\...`）找不到时，自动回退工程内默认目录，避免换机器/换盘符报"未找到精测点云库"导致先测后焊特征分析失败
- 完整点云逐帧调试导出（独立开关，默认关）
  - 精测点云界面新增"导出完整点云逐帧文件(排查相机散点,大文件)"复选框，存 `ConfigStore.db` 键 `FeaturePoint/ExportWorkpieceFrameDebug`，默认 false
  - 勾选后扫描导出 `PreciseLaserPoint_WorkpieceCloud_FrameDebug.txt`：每点 `X Y Z frame_index camera_x camera_y camera_z robot_x/y/z/rx/ry/rz camera_raw_ts_us mapped_ts_us`，CloudCompare 按 frame_index 标量着色定位散点帧、按各列逐帧判成因；与目标点/特征点(MatchDebug)流程独立
  - 拼接用 snprintf 一次格式化（避免 430 万点 ×15 次 QString::arg 链）；默认关闭避免每次扫描生成数百MB大文件拖慢流程
- 版本与发布
  - 应用版本更新为 `v2026.06.15.1835`
  - `Debug x64` 与 `Release x64` 编译通过；两段式打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.15.1835.exe`
  - 安装包大小 `59,468,829` bytes（56.7MB，FrameDebug 默认关、不影响包体），SHA256 `5A9EDA712A578A9C70DB6D161477DD512AB0B606F773691348460DD3EAC792B6`；包内 `BUILD_VERSION.txt` 一致（BuiltAt 2026-06-15 18:39:10）

## 2026-06-12

- 拟合调试点云导出（FitDebug，CloudCompare 核对用）
  - 精测点云处理"特征点算法参数"区新增"导出拟合调试点云(CloudCompare)"复选框，默认开启；配置存 `ConfigStore.db` 键 `FeaturePoint/ExportFitDebugCloud`（不写 ini）
  - 每次几何拟合导出到结果同目录 `FitDebug/`：`fit_all_points.txt`（每段输入点按段号上色，附到本段拟合直线垂距 `dist_to_fit` 与 `smoothN` 标量）、`fit_all_lines.txt`（每段拟合直线沿 s 采样还原 3D）、`fit_keypoints.txt`（起终点/拐点）、`fit_axes.txt`（质心+三轴局部坐标系）、`segments/seg_XX_*`（每段单独文件）
  - 三条路径全部生效：真机先测后焊（导出到本次结果 LaserPoint 目录）、精测界面单文件测试（输出文件同目录）、CLI `--laser-classify[-dir]` / `--rebuild-measure-weld-files`（输入文件同目录）
  - 便携包 `MeasureThenWeldFilterFit` 同步等价实现（标准库版），example 输出 `<前缀>_FitDebug`；已用 Result/Test 17 个用例批量验证
- 拐点拟合优化（随"直线拟合排除圆弧段"开关生效）
  - 背景：拐点打分以"到最近点云距离"为主导，真实折弯顶点恰悬在圆弧外侧而被罚分/否决，回退到落在圆弧上的平滑种子——大圆弧拐角被拉得近、小圆弧准，各拐角离焊道距离不一致
  - 顶点模式：开关开启时 `IsGeometryCornerProjectionUsable` 跳过"到点云最近距离"与"主带高度"两道会误杀真顶点的否决，仅保留 s 范围与放宽后的最大偏移（span*0.30，12~50mm）兜底防跑飞
  - 排圆弧加强：`cornerGuard` 上限 12→24mm（系数 0.10→0.18），启用门槛 4×→2× 让凹槽底部短平台等短段也裁两端圆弧，段直线方向不再被拐角圆弧带偏
  - 验证：Result/Test 17 用例拐点数全部不变、无跑飞；拐点到焊道距离差异为各拐角圆弧矢高的几何必然（用户确认接受"两直线交点"定义）
- 独立圆弧过渡预览（只输出、不参与主流程）
  - 在分类点（`PreciseLaserPoint_Classified.txt`）生成后，用与主流程相同的 `ApplyCornerArcTransitionToWeldPoseRecords`（工艺圆角半径 `cornerArcRadiusMm`）对拐角做圆弧过渡，输出 `PreciseLaserPoint_ArcTransitionPreview.txt`（X Y Z R G B，绿=圆弧过渡段、灰=直线段）
  - 叠加原始 `PreciseLaserPoint.txt` 即可核对工艺圆角是否贴合实际焊道；真正的圆弧过渡仍在焊缝补偿之后（`_SeamComp` 阶段）执行，预览不影响任何实际焊接文件；受同一调试导出开关控制
- STEP 机器人系统升级包归档（timestamp SDK 配套）
  - timestamp 2.4.2 构建要求控制器系统升级到 `SRS_V1.8.1.20260302_0603T`（升级包版本号 `20260302_0603` 与 SDK 归档目录一致，即同一配套发布）
  - 升级包收入 `SDK/STEP/versions/timestamp_2.4.2_20260302_0603/SRS_V1.8.1.20260302_0603T.ARM.zip`（57.6 MB），与该目录下的 2.4.2 SDK 三件套同处一处，现场升级机器人系统时直接取用
  - 未升级系统的 STEP 机器人不能使用 timestamp 构建（状态报文错位），需 `switch_step_sdk.ps1 -Mode legacy` 切回旧库重编

- 安装包体积优化（dist 330.5→147.8 MB，安装包 154.5→56.7 MB；四路代理审计 + dumpbin 依赖闭包实证，运行行为零变化）
  - 打包脚本（build_release_package.ps1）：顶层拷贝按文件名排除 `vc_redist*.exe`（一份 2026-03 遗留在 x64\Release 的 24.4 MB 旧版曾被全量拷贝混入包根，iss 实际只运行 Prerequisites\ 那份）；PointCloudExtration 改为按 dumpbin /DEPENDENTS 依赖闭包白名单拷贝（16 个 DLL + config\，剔除 6 个 debug 版 opencv 2413d 与闭包外 pcl_surface/visualization 等共约 42 MB，不动 SDK 源目录）；STEP versions 只随包携带 `SRS_*.zip`（Robot-SDK[d].lib 为链接期产物、exe 已静态链接，现场无编译器，4 个 lib 共 32 MB 运行期零作用）；windeployqt 之后清理本配置目录中按新参数不再产出的陈旧部署文件（windeployqt 只增不删）
  - Qt 部署裁剪（vcxproj 两处 AfterBuild + 打包脚本共三处 windeployqt 参数对齐）：`--no-system-d3d-compiler --no-system-dxc-compiler`（dxcompiler/dxil/D3Dcompiler 共 19 MB，纯 Widgets+QOpenGLWidget 不走 D3D RHI）；`--exclude-plugins qpdf,qtiff,qwebp,qtga,qicns,qwbmp,qgif,qsqlodbc,qsqlpsql,qsqlmimer`（无 PDF 功能、图标全 svg/ico、仅用 QSQLITE；保守保留 qjpeg）；`--skip-plugin-types tls,networkinformation,generic`（明文 socket、FTP 走 WinINet）
  - SRS 机器人系统升级包不随安装包分发（用户决定）：`SDK\STEP\versions` 整目录不进包（曾短暂做成 Inno 可选组件后撤销），升级包仅在源码仓库归档，按需另发现场；iss 主条目 Excludes 保留 versions 排除作双保险
  - 编译提速与诊断：`MultiProcessorCompilation` 两配置改 true（全量 Release 重建实测 1.3 分钟）；Release 改完整 LTCG（`UseLinkTimeCodeGeneration`）+ `/Gw`（exe 4.35→4.25 MB）；链接加 `/DEBUG` 生成 PDB 供现场崩溃 dump 符号化（PDB 被打包排除不进包）
  - 验证：Debug/Release 编译通过；裁剪项全部不在 dist、保留项齐全；用裁剪后 dist exe 跑 `--laser-classify` 全管线 EXIT 0、分类与 FitDebug 输出与裁剪前一致（14 关键点）；Inno 组件编译通过
  - 审计结论存档：运行速度无可感知优化空间（拟合热点为毫秒级、瓶颈在机器人物理运动与网络 I/O，PGO 评估不值得做）；明确不做的负优化：/O1、UPX、/arch:AVX2（现场平板无 AVX）、/fp:fast、关 /GL；下一档可选项：断开 opencv_world460.dll 链接可再省 61.4 MB（exe 仅导入 18 个符号、集中在两处 UI 消费点），本次未实施

- 代码大体检与死代码清扫（四路代理审计：死代码/重复/结构/仓库卫生，全部 grep/dumpbin 实证）
  - 顺带修复 2 个真实缺陷：① CLI `BuildCliOriginalTrackFitParams` 漏抄 featurePointStrategy→geometryStrategy 映射，`--laser-classify` 此前永远跑 LegacyGeometry、与真机流程不同源（已补齐映射）；② UDP 共享相机链路 `BuildUdpFrame` 拷贝时丢失 TCP 版的 (0,0,0)→NaN targetPoint 替换，0 值伪目标点会混入下游（已对齐 TCP 行为）
  - 死代码清扫合计 **-10,614 行**：整文件删除 13 个（vendored 坡口库 SkFunction/SkDataClass 约 6300 行零外部消费者仍在编译、死类 LineCoarseScan/MeasureThenWeldWorkflow/MeasureWelding 1227 行、未参编孤儿 groove/tcpsensorclientworker 253 行），vcxproj/.filters 同步；**勘误：Sk\* 五文件随后按用户决定恢复**——它是相机厂商的坡口识别源码库（SkGrooveRecog），保留备用，已在 CLAUDE.md 标注勿删；手术删除 41 项（FilterLowerWeldPath 整条旧滤波链 1370 行——唯一调用者零调用、活代码全走 Direct→Geometry；FunctionTestDialog 自注 Deprecated 的滤波拷贝 352 行；FANUC 旧版 MoveByJob 两重载与 STEP 幽灵声明；约 25 个零调用散兵函数）；连带清理死数据流 LowerWeldFitMode 枚举、params.fitMode/lineFitTrimCount 字段、精测界面"拟合裁首尾点数"无效 spinbox 与 `Fit/LineFitTrimCount` 配置键
  - 保留项（审计确认有真实消费，勿删）：piecewiseFitTolerance/piecewiseMinSegmentPoints/searchWindow、AnalyzeMeasureThenWeldLowerWeldPathDirect 转发壳、portable/ 两模块（README 注明的有意副本）
  - 验证：Debug/Release 编译 0 错误；清扫前后用 `--laser-classify-dir Result\Test` 对 17 用例 41 个输出文件逐字节对比**全部一致**（纯删除零行为变化；基线取自 bug 修复之后）

- 仓库瘦身（git 历史重写，克隆体积 1.05GB → 89MB）
  - 根因：`.gitignore` 曾强制纳入 `dist/installer/` 安装包，全历史累计 14 个安装包 blob 共 1044MB，占可达对象的 83%
  - 处理：删除强制纳入规则（`dist/` 整目录忽略，安装包经 GitHub Release 分发）；`git rm --cached` 7 个在跟踪安装包；`git filter-repo --invert-paths --path dist/installer` 重写全历史（3 分支 + 17 标签），镜像备份后强推覆盖远程
  - 结果：`.git` 1.4GB→90MB；新增忽略 `tmp/ analysis_output/ dev_backups/ portable/*/build/`；`CLAUDE.md` 纳入版本管理并同步 git 约定（安装包不进 git、提交只署名 yu1201）
  - 磁盘清理：旧安装包 3107MB→155MB（保留已发布的 v2026.06.12.1146），根目录历史构建日志/obj 约 110MB 清除；重写前镜像备份存于 `../_backups/repo_before_filterrepo.git`
  - 注意：历史 SHA 全变，其它机器的旧克隆需重新 clone；GitHub Release 附件不受影响

- 去重与结构整理（批3 第一阶段）
  - 几何拟合参数名册收敛为唯一来源：新增 `MeasureThenWeldService::BuildTrackFitParamsFromSettings(settings, fallbackSampleAxis)`，真机 `BuildOriginalTrackFitParams`（追加拐点补偿）与 CLI `BuildCliOriginalTrackFitParams`（保持不启用拐点补偿）均改为委托——此前两处 42 行手抄已实际漂移过一次（CLI 漏 geometryStrategy 映射），今后新增参数只改一处
  - `ContralUnit.h` include 下沉：22 行的头此前拖着两套驱动头（连带 KDL/socket/FTP）灌进 14 个 TU，现仅保留 `Const.h + <vector>`，驱动头下沉到 cpp；消费方破口以前向声明/直接 include 补齐（MeasureThenWeldDialog.h、HandEyeCalibrationDialog.h、WeldSeamCompDialog.cpp）
  - 顺带根治三个老隐患：工程级 `NOMINMAX + WIN32_LEAN_AND_MEAN`（此前靠驱动头先于 windows.h 的脆弱包含顺序压住 min/max 宏与 winsock1/2 冲突，已核实无裸 max/min 宏用法）；`u_short`→`unsigned short`（15 处，等价 typedef，消除 winsock 隐性依赖）
  - 验证：Debug/Release 零错误；Result/Test 17 用例 41 输出文件与基线逐字节一致
  - 待办（批3 后续，按计划分步）：SaveTextLines 四份收敛、GBK 解码四胞胎合一、分类输出行 8 处统一（需先定 raw/raw_noise 取舍）、groove 帧协议层合并（需现场验证）、CLI 块拆独立 cpp（约 2000 行）、管理页内联类外迁（约 6000 行）、WeldPoseGeneration 拆分（约 3950 行，配离线重建逐字节回归）

- 版本与发布（第二批）
  - 应用版本更新为 `v2026.06.12.1630`（并入：两处缺陷修复、安装包瘦身 154.5→56.7MB、死代码清扫、参数名册唯一来源、SRS 升级包改为不随包分发）
  - `Debug x64` 与 `Release x64` 编译通过；两段式打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.12.1630.exe`
  - 安装包大小 `59,479,416` bytes，SHA256 `C2BE556045953D6F2B5314C820635024EDA61C901ED8E172AB8A6614E91AA444`；包内 `BUILD_VERSION.txt` 一致（BuiltAt 2026-06-12 16:32:55）
  - 重要变化：本版起安装包**不含** SRS 机器人系统升级包（v1146 曾随包），现场需要时从仓库 `SDK/STEP/versions/` 归档单独分发

- 版本与发布
  - 应用版本更新为 `v2026.06.12.1146`
  - `Debug x64` 与 `Release x64` 编译通过；两段式打包通过（build_installer.ps1 -AppVersion），生成 `NoTeaching-Robot-Setup-v2026.06.12.1146.exe`
  - 安装包大小 `161,979,023` bytes，SHA256 `C952A99D87527F61C0D4EC3D1DB8B341D202C111E8AEEA3709E5CAFD8F600A26`；包内 `BUILD_VERSION.txt` 与应用版本一致（BuiltAt 2026-06-12 11:52:47）
  - 注意：安装包因携带 57.6 MB 机器人系统升级包而超出 GitHub 单文件 100 MiB 限制，本版起安装包不再提交进 `dist/installer/`，仅作为 GitHub Release 附件分发（README 下载入口不变）

## 2026-06-11

- STEP SDK 切换 timestamp 2.4.2 构建（机器人位置错位修复）
  - 现象：新固件机器人（焊接机器人3/RobotC）下主页"机器人监控"位置整体错位一格（X=0.000，真实 X 顺移到 Y，依次类推）
  - 根因：新版协议在 `PlatformToSDKStruct` 的 `m_CurrentLine` 与 `m_ActPos` 之间插入 `uint64_t m_TimeStamp_ms`（8 字节），控制器按新布局发报文、程序此前为 legacy 2.4.1 构建按旧布局解析，时间戳之后全部字段（位姿/Cartpos_World 等）错位 8 字节；扫描采集位姿走同一链路，错位状态下扫描数据同样不可用
  - 管理界面"STEP接口"下拉是运行时开关，仅在 timestamp 构建下生效——legacy 构建下新接口编译期被裁掉，UI 切换无效（监控源显示"旧版库构建"即编译期分支）
  - 处理：SDK 三件套切换到 `versions/timestamp_2.4.2_20260302_0603`、`STEP_SDK_HAS_TIMESTAMP=1` 全量重编；未升级固件的 STEP 机器人现场需 `switch_step_sdk.ps1 -Mode legacy` 切回重编（双版本不能同 exe 并存，实时切换方案经评估为双包装 DLL，按用户决策暂不实施）

- 工件模型网格化 v4→v7（形态修复迭代，缓存标记逐版失效重建）
  - v4：放弃扫描线条带化——单帧滤波每帧保留 2~3 条主直线段（多段折线），任何"每帧一条线"的条带假设都不成立（弦向投影段间重叠、排序交错、桥接检查丢弃大片导致碎裂）；改为 PCA 主平面高度场栅格化：抽样 PCA 求板面主平面（质心+两主轴+法向），全点投影 (u,v,h)，规则栅格 2×2 三角化——与帧结构/点序/摆放姿态完全解耦
  - v5：小孔邻域填补（扫描行距大于格宽形成的虚线孔，≥3 有效邻居取中值，最多两轮）+ 3×3 中值滤波
  - v6：格内聚合改"最大簇中位数"（按 3mm 高度间隙切簇取点数最多簇）——消除立面过渡格在两面间摆动拉出的竖直细丝毛刺与悬空噪声簇；中值滤波加到两遍（代价：折弯凸角约 3mm 倒圆，渲染级可接受）
  - v7（多代理审查确认后修复）：输入非有限值设防（离线重建读回的点云可能含 `nan` 行，进投格 int 转换与排序为 UB）；孔填补加双向包夹条件（防止向自由立边外长出约 3mm 悬空假唇边）；`SaveMeshPly` 改临时文件+原子替换（防进程中途终止留下被误判有效的截断缓存）
  - 多代理审查同批修复：QPainter 关闭 GL_MULTISAMPLE 不恢复导致首次画框后 MSAA 永久失效；查看器析构加销毁令牌+工作线程计数等待，杜绝应用退出时 detached 后台线程对悬垂 this 的调用；编辑模式中重开查看器的状态机错位；"恢复原始"先校验源点云再删缓存；忙时换目录给出明确提示

- 工件模型查看器迁入管理界面 + 框选编辑
  - 入口迁移：管理界面"调试 → 工件模型"独立页签（PrepareEmbeddedPage 嵌入管理栈，工程师权限），补偿界面"工件模型"按钮与弹窗删除；页内"打开结果…"自选 `Result\<机器人>\<结果>` 目录（自动进 LaserPoint 子目录，校验有点云或缓存）
  - 工具栏分组美化：视图（实体面/点云/高度图，互斥高亮）|相机（俯视/复位）|编辑（框选编辑/删除所选/撤销/重新生成/恢复原始）|导出 STL；主操作青色/危险操作红色点缀、组间分隔线、disabled 显式变暗
  - 框选编辑（选中-删除两段式）：进入编辑后台加载完整点云（进度条+可取消）；左键拖矩形选中（亮红、多框累加、穿透全部深度，与 CloudCompare segment 一致），Delete/"删除所选"删除（撤销栈 20 步），Esc 清空选择，Ctrl+左键旋转；"重新生成"用剩余点后台重建网格覆写缓存，并把裁剪点云原子写出 `PreciseLaserPoint_WorkpieceCloud_Edited.txt`——此后建模与编辑均优先用裁剪文件（原始 `_WorkpieceCloud.txt` 永不改动），"恢复原始"删裁剪文件+缓存从原始重建
  - 性能（按 CloudCompare 路线 + 业界调研验证）：橡皮筋 QRubberBand 覆盖层（拖动零点云重绘，并消除 QPainter 混合渲染的状态污染）；选中投影判定按硬件线程数分块并行+矩阵乘手动展开+float 坐标缓存（340 万点约数十 ms→约 10ms 级）；选中标记独立 1 float/点小缓冲、容量够时原地 write；regen 启动即退出编辑模式冻结交互防数据分叉，进度框移除不可用的取消按钮

- STEP SDK 升级全量审计与修复（12 代理逐项核查）
  - 确认安全：驱动调用的 35 个接口两版签名（含默认参数）逐字一致；OPERATIONMODE/PROGRAMSTATE/PROGRAMMODE/MESSAGETYPE/MODEKEY 等枚举无值重排（驱动全部字面值比较点无受害）；RobotCartPos/AXISPOS/MessageData 等位姿消息结构逐字段一致；运动下发链路（.srp/.srd + ProgramLoadCmd/ProgramRunModeCmd/SetModeCmd 序列）无隐性旧行为依赖；EasyTcpClient.hpp 经 lib 内嵌源文件校验和证实与 2.4.2 严格配套（新版焊接指令结构有重排与 int→double，本工程不调用，无影响）
  - 修复（审计确认）：状态时间轴会话锁定——连接后首个 getTimestamp 样本决定走机器人时间戳或 PC 接收时间，会话内不再切换，锁定机器人轴后偶发 0 值沿用上一有效时间戳（原实现逐次回退 PC 时钟，两种纪元可混进同一扫描序列破坏时间插值）；控制器未提供时间戳时机器人日志记告警（每连接一次）；StepUseTimestampSdkInterface 进程级缓存（原监控线程每 50ms 查一次 SQLite），管理界面切换接口模式时主动失效
  - 切换流程缺陷修复：switch_step_sdk.ps1 拷贝列表补 EasyTcpClient.hpp，两个 versions 归档目录补齐对应版本（legacy=2.4.1 原版包内文件、timestamp=lib 校验和匹配的激活文件）
  - 现场确认项：控制器固件需与 2.4.2 配套（位置显示正常即证明）；联机检查 robot_ms 与 pc_recv_ms 是否明显不同（恒相等=控制器未填时间戳走了 PC 回退，日志有提示）

- 版本与发布
  - 应用版本更新为 `v2026.06.11.1711`（替代 v2026.06.11.1634，并入 SDK 升级审计修复）
  - `Debug x64` 与 `Release x64` 编译通过；两段式打包通过（build_installer.ps1 显式 -AppVersion），生成 `NoTeaching-Robot-Setup-v2026.06.11.1711.exe`
  - 安装包大小 `101,608,458` bytes，SHA256 `140B29B05CB9D15EA6C03BB5305386496C9F756A83D4B926948675C0655ABAAC`；包内 `BUILD_VERSION.txt` 与应用版本一致（BuiltAt 2026-06-11 17:13:40）

## 2026-06-10

- 先测后焊扫描
  - 扫描运动完成超时由固定 120s 改为按扫描距离/配置速度动态估计（预计时间×2+30s，夹 [120s,1800s]，与焊接执行同算法），修复慢速测量长焊缝误报超时；扫描开始日志输出距离/预计时间/超时值

- 点云 SDK 更新（20260609 版）与已焊起点截断
  - 替换 PointCloudExtration.dll/.lib/.h 与出厂配置（厂商新默认参数：平面内点距离 1.5、直线合并角度 25、合并距离 100、干扰性长度 40）；Data 参数副本同步更新（调试输出默认关闭）
  - 新版 DLL 要求 LOGPATH 必须有值：运行时配置副本对空 LOGPATH 自动注入项目 `Log\PointCloudExtration` 默认目录（界面留空仍为未设置语义），修复 "Para_name LOGPATH not found" 弹窗
  - SDK 主接口新增已焊起点出参（terminal）：检测到工件已焊段时返回新焊接的起点，零点视为无已焊段；导出函数签名与解析名同步更新
  - 新增"使用已焊起点截断焊道"开关（SDK 参数组，默认关闭，仅 SDK 两种方法生效）：开启后焊道从已焊交界点截断只焊剩余部分；截断侧按测量焊接参数"焊接方向"选取（起点到终点截头部、终点到起点截尾部），截断后端点类型相应重置，两种方向下执行都从已焊交界处接续；未检测到已焊段或截断后点数不足时不截断，日志记录截断详情

- 精测点云处理扩为四种方法
  - 当前方法扩为：SDK点云算法全处理（SDK 拐点直接使用）、SDK点云算法+拟合（SDK 基础焊道再滤波拟合）、点云算法+拟合（完整点云先经立板投影提取下层轨迹、再滤波拟合，新增）、特征点+拟合（相机目标点轨迹，原旧流程）
  - 原"方案三：立板投影到底板"从拟合方案下拉移除，其投影提取能力并入"点云算法+拟合"方法做固定前置（投影轨迹仍落盘 `PreciseLaserPoint_WorkpieceExtracted.txt`）；存量配置选了方案三的按旧版几何拟合处理
  - 投影提取算法的内置参数开放为独立可调（点云参数页"点云投影提取参数"组，仅点云算法+拟合启用）：站位窗口/横向窗口/种子上下方Z带（0=自动按滤波参数派生，与原行为一致）、底板层位分位上下界（默认 35%~50%）、每种子候选上限（默认 160）、投影平滑半径（0=自动）；便携版 `MeasureThenWeldFilterFit` 暂未同步该参数化
  - 先测后焊管线按四方法分支；处理失败直接报错，不回退其他方法（原"失败回退特征点+拟合"功能整体删除，含界面勾选、配置键与 CLI 覆盖项）
  - CLI `--pointcloud-processing-mode` 扩展 `sdk|sdkfit|cloudfit|legacy` 四个取值

- 精测点云处理界面重构
  - 删除单文件测试功能（输入/输出/执行入口）
  - 点云参数页只放 SDK 参数（"SDK 点云参数/SDK 算法内部参数"），仅 SDK 两种方法启用
  - 特征点算法页改名"滤波拟合参数"，拟合方案选择置顶，去噪/分段参数（Z阈值/Z突变/Z连续/段间跳变/保留最长段）与拟合参数同页（对走滤波链的三种方法生效）
  - 按方法联动禁用不相关参数组与标签页；SDK全处理下滤波拟合页与有效性检测页禁用

- 工件点云逆向建模与三维查看
  - 利用完整点云的有序扫描线特性做轻量网格化（相邻扫描线条带三角化，零新第三方依赖；遮挡缺口不桥接），生成模型缓存 `PreciseLaserPoint_WorkpieceMesh.ply`（二进制，顶点+法线+三角索引，可直接拖入 CloudCompare）——扫描/重建时自动生成一次，之后秒开
  - 新增工件模型查看器（QOpenGLWidget 硬件渲染）：实体面（双面光照+高度伪彩）/点云/2.5D 高度图三种模式，补偿界面预览工具栏"工件模型"按钮打开；老目录首次打开后台线程自动生成缓存（进度条+可取消，不阻塞界面）；支持导出二进制 STL（CAD 软件可直接导入）
  - 网格化适配任意工件摆放姿态（竖立波纹板实测修正）：分帧按 3D 相邻点距突变、帧内弧长参数化、蛇形方向统一、高度/伪彩轴自动取表面偏移方向；缓存带算法版本标记，旧版缓存自动失效重建；点云文本解析改手写（较正则快一个量级）
  - 工程接入 Qt opengl/openglwidgets 模块（vcxproj include 与链接沿用手写路径模式）

- 补偿界面
  - 补偿预览工具栏新增"按当前方法重算"：对所选目录按当前处理方法离线重建基准焊道（覆盖分类点/焊接姿态/补偿文件，带确认框），完成后自动重载六阶段预览——切换处理方法后先重算基准再调补偿
  - 焊道显示打开时在焊道中段旁显示"焊接方向"箭头（XY 平面内）；箭头位置按扫描位置取边——放在扫描时机器人轨迹所在的一侧，与焊接方向无关
  - 四种方法处理成功时各自落盘独立基础焊道文件（`_SdkClass`/`_SdkBase`/`_PointBase`/`_PointLaser`，index x y z 格式），文件存在即表示该目录已按该方法完成焊道生成；原 `_WorkpieceExtracted` 由 `_PointBase` 取代
  - 补偿界面"原始数据"图层统一显示当前方法自己的基础焊道文件（未生成时回退相机目标点轨迹）；载入目录检测到当前方法基础焊道缺失且有原始输入时提示并自动重算一次，先测后焊"跳过扫描焊接"本就强制按当前方法重建
  - 焊接方向（起点到终点/终点到起点）迁入工艺参数按工艺组保存：工艺界面拐点页与补偿界面工艺区域均可设置；老工艺未存过时预填测量页旧值，保存固化进工艺；执行方向反转、已焊起点截断、预览箭头全链统一按工艺值生效；测量焊接参数页不再显示该项（旧值保留作回退）
  - 精测点云处理界面禁用的参数控件显式置灰（补全 :disabled 样式）

- 版本与发布
  - 应用版本更新为 `v2026.06.11.1405`（自 v2026.06.10.2247 起依次并入 SDK 20260609 更新、截断方向修复、预览方向箭头、焊接方向迁工艺、扫描超时动态化、箭头侧偏按扫描位置取边、LOGPATH 空值兜底、四方法独立基础焊道与缺失自动重算、工件模型缓存与三维查看器、网格化竖立姿态修复与后台进度条与 STL 导出、进度框取消连接断言修复，逐次重新发版替代）
  - `Debug x64` 与 `Release x64` 编译通过；Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.11.1405.exe`
  - 安装包大小 `101,577,219` bytes，SHA256 `F24963BAF01E8C2350117EA8C77935C8CD0A0B2E7841E7F5D40D341040B8B7C0`
  - 已验证包内 `BUILD_VERSION.txt` 为 `2026.06.11.1405`、Qt6OpenGL/Qt6OpenGLWidgets 随包部署、SDK dll 与厂商更新包字节一致；安装包文件名/注册表版本/应用版本三者一致（打包脚本默认取打包时刻为版本，发版需显式传 -AppVersion 与源码同步）

- 滤波拟合数值参数接入管线（淘汰死参数）
  - 原界面数值参数（Z阈值/Z突变/Z连续/段间跳变/步长/窗口/容差/平滑等）此前只被测试功能消费、管线硬编码；现以新配置键真正接入主管线与 CLI `--laser-classify`，默认值=原硬编码值，现场不动界面则行为不变
  - 采样主轴新增"自动推断（按扫描方向）/X/Y"三态，对四种方法都生效（含 SDK 全处理的段类判定方向），移入处理方法组恒可用；GUI 与 CLI 在固定主轴时行为一致
  - 拟合模式固定保持原始轨迹（PreservePath），不再开放下拉
  - 多代理代码审查确认的 6 项问题（CLI 主轴分叉、启用矩阵与回退数据流不一致、控件范围窄于配置层等）已全部修复

## 2026-06-06

- 配置数据库与安装升级
  - `ConfigDatabase` 支持在 `Data/ConfigStore.db` 不存在时自动创建空 schema，新设备首次运行不再依赖安装包内置现场数据库
  - 安装包不再携带当前 `ConfigStore.db`、机器人单元、工艺、补偿、相机、手眼等旧 INI/TXT 参数文件
  - Inno Setup 只创建空 `Data` 目录，升级安装不删除、不覆盖现场已有 `Data` 数据
  - 空配置库下会自动创建默认 `admin/admin` 账号，控制单元管理允许重新新建机器人单元
  - 新建控制单元时，工件模板不存在不再阻塞保存；已有模板数据时仍可复制作为默认参数

- STEP SDK、状态时间戳和现场兼容
  - STEP SDK 更新到支持机器人时间戳的新版接口，并保留旧版 SDK 库用于未升级机器人现场
  - 安装包随带 `SDK/STEP/versions` 的新版时间戳库和旧版库，以及 `switch_step_sdk.ps1` 切换工具
  - 管理界面增加 PC 时间/机器人时间选择，先测后焊扫描可按配置选择时间戳来源
  - 状态显示补充当前数据来源说明，便于现场判断状态来自 SDK、监控接口或缓存

- 点云、拐点和补偿
  - 精测点云查看支持点击设置旋转中心、单指旋转、双指平移和捏合缩放
  - 便携版 `MeasureThenWeldFilterFit` 同步新增斜率一致直线拟合排除圆弧段方案，并更新 README、示例和压缩包
  - 精测量界面增加拐点生成方案切换；先测后焊按保存后的方案生成分类点和焊接轨迹
  - 补偿组新增拐点补偿参数，按上坡/下坡分别保存 inner_corner 与 outer_corner 的方向补偿，并生成补偿后的 2mm 点文件
  - 最终焊接轨迹支持仅在下发前按点间距抽样，保留前面滤波、拐点识别和补偿计算结果

- 版本与发布
  - 应用版本更新为 `v2026.06.06.0106`
  - `Debug x64` 编译通过；警告为第三方/SDK PDB 缺失和 STEP SDK 头文件宏/导出提示
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.06.0106.exe`
  - 安装包大小 `101,226,607` bytes，SHA256 `2CAF825461EE9DF2A57D80DBDA1304C55DFD56A902B50CBD4168E31B58E7CF04`
  - 已验证包内 `BUILD_VERSION.txt` 为 `2026.06.06.0106`，`dist\QtWidgetsApplication4\Data` 不含默认参数文件
  - 已复制发布目录到临时空目录启动验证：首次启动前无 `ConfigStore.db`，启动后自动生成空配置库和默认账号配置

## 2026-06-04

- 精测点云处理与先测后焊
  - 精测点处理新增旧版几何拟合、方案一斜面波动滤波、方案二鲁棒分段关键点和方案三立板投影到底板切换
  - 精测点处理界面新增保存入口，当前方案写入配置后可被先测后焊流程读取
  - `PreciseLaserPoint_WorkpieceCloud.txt` 支持提取生成下游 `PreciseLaserPoint.txt`，并继续接旧流程
  - 新版点云 SDK 接入后增加运行配置保护，`DEBUGLOG` 路径不可用时自动生成临时配置，避免 SDK 调用时直接 abort
  - 最终焊接轨迹新增抽样间距参数，只在生成下发轨迹前抽样，不影响前面的拐点识别和补偿计算；抽样结果保存为 `_FinalSampled.txt`

- 工艺参数、STEP 程序和 FTP
  - 工艺界面新增焊接模式下拉，支持 `0` 直流一元、`1` 脉冲一元、`2` JOB模式、`3` 近控模式、`4` 分别、`5` CC/CV、`6` TIG、`7` CMT
  - STEP SRP 生成在 `ARCON` 前写入 `ARCMODE(...)`，并随工艺参数携带当前焊接模式
  - “关联摆动号”改为“是否启用摆动”开关，新增“是否启用跟踪”开关；关闭时对应参数页禁用，生成焊接文件时摆动或跟踪变量写为 `NULL`
  - FTP Job 文件批量删除改为一次确认和串行删除，避免连续弹窗导致界面卡死退出

- 现场工具、相机和 SDK
  - 主界面现场工具区改为可拖动排序的工具面板，支持右键新增、删除和恢复默认顺序
  - 工具拖动期间暂停外部刷新，拖动后仍触发原按钮逻辑，解决拖动卡死和移动后点击无效的问题
  - 新增 `SKJCameraControlClient`，相机参数支持通过 `SKJCamera.dll` 读取/设置曝光、增益、二值化阈值和激光开关
  - Debug/Release 构建都会随目标目录复制 `SKJCamera.dll`

- 版本与发布
  - 应用版本更新为 `v2026.06.04.1832`
  - `Debug x64` 编译通过，0 警告 0 错误
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.04.1832.exe`
  - 安装包大小 `98,770,051` bytes，SHA256 `2AB0AC00FAE6979F09C0EBE1BBC2580C53544DEB960CC2A366662F8D603C5BC7`
  - 已验证包内 `BUILD_VERSION.txt` 为 `2026.06.04.1832`，`dist\QtWidgetsApplication4\QtWidgetsApplication4.exe --help-cli` 退出码为 `0`

## 2026-06-01

- STEP 摆动/跟踪工艺与先测后焊
  - 工艺参数新增 `WEAVEDATA` 摆动数据和 `TRACKDATA` 跟踪数据，工艺界面新增“跟踪参数”页
  - 摆动参数和跟踪参数的名称、单位按 STEP 示教器界面对齐；STEP SRD 输出 `WEAVEDATA wd0`、`TRACKDATA td0`
  - STEP SRP 的 `WLin` 指令改为携带 `wd0/td0`，已用 RobotC 的 `20260601_004` 数据生成并上传机器人 FTP 后回读校验
  - 工艺文件升级为新格式，不再兼容旧 `WeldPara.txt` / `WeaveDate.txt`；旧格式读取失败时提示重新创建工艺内容，并允许进入工艺界面新建

- 页面打开、数据库和下拉刷新
  - 延迟进度条改为统一守卫逻辑，页面打开过程中出现错误弹窗时会自动收起加载框，避免进度条卡住
  - 修复工艺读取失败后阻止进入工艺页的问题，失败时进入空界面并提示点击 `+` 重新创建
  - 修复新建控制单元、工艺、焊接参数组、补偿参数组后先测后焊下拉列表不刷新的问题
  - 修复带后缀控制单元删除时报错的问题，并清理错误副本控制单元相关数据库记录

- 版本与发布
  - 应用版本更新为 `v2026.06.01.1912`
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.01.1912.exe`
  - 安装包大小 `98,689,584` bytes，SHA256 `136601E5B9CBEEE659A9856A2D7175EA59C74146ACB76709D824846367E43824`

- 精测点云处理与 SDK 接入
  - 新增 `PointCloudExtractionProcessor` / `PointCloudProcessingConfig`，通过应用层封装调用外部 `PointCloudExtration` SDK，不修改算法源码
  - 管理界面新增精测点云处理页，支持点云算法和旧特征点算法切换；点云算法参数、库目录、配置文件和失败回退策略可在界面维护
  - 新算法流程拆分输出 SDK 点云焊道提取文件、SDK 拟合结果文件和最终下游读取的 `PreciseLaserPoint_Classified.txt`
  - 先测后焊可按当前精测处理方式接入点云算法结果，再继续属性补全、裁剪、补偿、过渡和焊接流程

- 姿态示教、补偿和焊接参数
  - 测量焊接参数新增“焊接姿态示教”分区，包含启用开关、RX/RY/RZ 编辑和“示教当前位置为焊接平台标准姿态”
  - 启用示教焊接姿态后，姿态生成 RX/RY 使用示教值，平台 RZ 使用示教 RZ，坡道 RZ 按计算平台 RZ 与示教 RZ 的差值补偿，并处理 `-180/180` 环绕
  - 未启用示教时，旧的 `NormalWeldRx`、`NormalWeldRy`、`WeldRzGainDeg` 仍可在同一分区读取和修改
  - 旧数据迁移工具补齐新增的扫描安全位、示教姿态、补偿组和精测点云参数，旧值只补缺不覆盖

- 界面、菜单和打包
  - 管理界面只保留顶部菜单栏，移除额外常用工具栏；菜单栏改为直角样式
  - 工艺参数、测量焊接参数、补偿参数、相机和精测点云页面统一接入滚动区，编辑框设定合理宽度，低分辨率下不再裁切控件
  - 相机预览窗口关闭和页面切换时增加资源释放保护，避免预览窗口卡住后无法关闭
  - `ConfigMigrate.exe` 重新生成，安装包随带更新后的迁移工具和点云 SDK 运行文件

- 版本与发布
  - 应用版本更新为 `v2026.06.01.0146`
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.06.01.0146.exe`
  - 安装包大小 `92,921,566` bytes，SHA256 `15BDC463FF25415FC9CF66823C6F8D4EB501ACE8D02789A27B1A46F914E7B3E6`

## 2026-05-30

- 配置数据库与乱码修复
  - 新增 `ConfigDatabase`，主程序通过 Qt SQL 读写 `Data/ConfigStore.db`
  - 账号、相机、手眼、测量焊接参数、焊接工艺、补偿参数等读写逐步切到配置库，路径和中文内容统一按 UTF-8 存储
  - 新增 `tools/migrate_config_to_sqlite.py` / `ConfigMigrate_Run.cmd`，用于把旧 `Data/*.ini`、`WeldPara.txt`、`WeaveDate.txt` 迁移进数据库
  - 安装包排除 `ConfigStore.db*`，升级时只补默认模板，不覆盖现场工艺名称和参数数据库

- FTP Job 文件管理
  - `FtpClient` 新增远端目录列表能力，删除接口支持页面统一确认
  - 管理界面新增 `FTP Job 文件`页面，复用当前机器人 FTP IP、端口、账号和密码
  - 页面支持本地 Job 与机器人 FTP Job 双列表，执行上传、下载、刷新和删除

- 先测后焊、工艺和补偿组
  - 先测后焊界面新增焊接工艺、姿态补偿组、焊道补偿组下拉选择，并把选择写回当前机器人配置
  - 实际焊接和运动轨迹速度优先使用焊接工艺 `WeldVelocity`
  - 姿态补偿和焊道补偿改为组管理，每组固定包含低平台、上升边、高平台、下降边四条数据
  - 姿态匹配类型作为姿态组属性保存，姿态组可编辑 RX/RY/RZ 和 XYZ 补偿
  - `焊道补偿`管理入口从`相机`菜单移动到`工艺`菜单

- STEP、点云和现场工具
  - STEP 焊接程序生成修正 `ARCON/ARCSET/ARCOFF` 变量写法，空跑模式不再生成焊接指令
  - 工艺参数新增拐点过渡作用范围：圆弧、过渡、圆弧+过渡
  - 新增自研离线点云查看窗口，支持图层列表、旋转、平移和缩放
  - 新增 `--rebuild-measure-weld-files`，可从已有 `LaserPoint` 目录重建 PreservePath、焊接姿态和补偿文件
  - 先测后焊界面增加进度条和忙碌提示，多个管理页面打开时增加延迟加载提示

- 版本与发布
  - 应用版本更新为 `v2026.05.30.0330`
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.05.30.0330.exe`

## 2026-05-27

- 相机接收模式
  - 坡口相机 TCP 接收迁移到 `ScanCameraTcpClientWorker` 封装，第三方 `tcpsensorclientworker` 源码恢复为独立保留
  - 恢复管理界面的“TCP 独立 / UDP 共享”模式切换，UDP 共享接收按相机 IP 分发到对应机器人缓存
  - UDP 接收补强完整帧解析，支持缓存里连续解析多帧，避免 50004 高帧率下只消费单帧造成扫描点云稀疏

- 诊断工具与扫描日志
  - FPS 检测工具新增 `-Protocol Udp` / `-Udp`，并增加 `camera_fps_probe_udp_gui.cmd`
  - UDP 50004 现场测试 10 秒约 `48.30 fps`，20 秒约 `47.90 fps`
  - 先测后焊扫描完成日志新增缓存新增帧数和估算缓存 FPS，便于对比实际相机帧率和采样点云密度

- 手眼矩阵与平板界面
  - 手眼矩阵参数界面新增“读取机器人 eye”，STEP 机器人支持从全局变量文件/SDK 读取手眼矩阵并填入参数
  - 手眼标定界面加入滚动区域，低分辨率平板上按钮和参数区不再被窗口边界裁掉

- 版本与发布
  - 应用版本更新为 `v2026.05.27.1711`
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.05.27.1711.exe`

## 2026-05-26

- STEP 焊接与轨迹生成
  - 焊接姿态文件支持离线生成 STEP `Weld_时间.srp/.srd`
  - 实际焊接模式写入 `ARCON/ARCSET/ARCOFF`，空跑模式不生成焊接指令
  - STEP 轨迹支持 `OVERLAPREL` 过渡比例、焊接方向、拐点过渡速度和过渡电流电压

- 焊接工艺与参数界面
  - 焊接工艺新增包角参数启用项，以及拐点过渡半径、速度、电流、电压启用项
  - 测量焊接参数新增扫描位置示教入口、计算安全位参数、焊接方向和 STEP 过渡比例
  - `CornerArcRadiusMm` 从测量焊接参数迁移到焊接工艺拐点过渡参数，减少重复配置

- CLI、相机与安装包
  - 先测后焊扫描 CLI 支持 `--robot` / `--robot-unit` 指定 STEP/FANUC 机器人
  - 坡口相机 FPS 离线检测工具加入安装包，测试完成后窗口停留显示平均帧数和一帧样例点云数据
  - Release 打包补齐 Qt 中文翻译文件，平板标准弹窗按钮保持中文

- 版本与发布
  - 应用版本更新为 `v2026.05.26.1959`
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.05.26.1959.exe`

## 2026-05-25

- 平板管理页滚动与点击
  - 窗口辅助只在窗口打开时做一次自适应处理，不再用外层滚动区包装管理内嵌页
  - 账号管理、控制单元管理、相机参数和测量焊接参数页改为页面内部横向/纵向滚动，低分辨率下不再裁掉按钮和右侧内容
  - 统一中文字体 fallback，减少下拉框、菜单和标题在平板系统上出现乱码的概率

- 参数乱码与现场 `Data` 保留
  - 测量焊接参数折叠标题改为按参数 key 固定分组，不再读取 `MeasureWeldParam.ini` 注释
  - 兼容安装升级保留现场 `Data` 的策略，即使旧参数文件注释编码异常，界面标题也能正常显示

- 坡口相机与点云
  - 坡口相机预览接收改为 `PointCloundTcpServer` TCP 独立连接，固定端口 `50006`
  - 旧 UDP 共享端口模式入口在界面上关闭，诊断信息改为 TCP 接收次数、解码帧和缓存帧
  - 先测后焊完整工件点云生成前统一 `allResultPoint` 的 Z 轴符号到目标点坐标约定

- 版本与发布
  - 应用版本更新为 `v2026.05.25.1826`
  - 安装脚本默认版本改为带小时分钟，包内写入 `BUILD_VERSION.txt`
  - `Release x64` 和 Inno Setup 打包通过，生成 `NoTeaching-Robot-Setup-v2026.05.25.1826.exe`

## 2026-05-22

- 三段式点云滤波
  - 新增可移植 `LaserFramePoint3DFilter` 模块，用于单帧三维激光点云滤波
  - 已知三段式轮廓时支持只保留最长 2-3 条主直线，降低反光短线干扰
  - 快速候选线搜索参数暴露到 `LaserFramePoint3DFilterOptions`，便于现场按速度/稳定性调节

- 坡口相机预览与测试
  - 坡口相机预览增加原始/滤波点云可视化弹窗，支持缩放、平移和目标点显示
  - 功能测试页增加当前相机帧点云滤波导出，便于保存原始帧、滤波帧和统计信息
  - `LaserFramePoint3DFilter.cpp` 在 Debug x64 下单独开启优化，调试界面也能接近 Release 滤波速度

- 登录体验与输出文件
  - 登录页支持历史账号下拉、按账号保存密码、游客登录和更紧凑的卡片式布局
  - 先测后焊完整工件点云文件 `PreciseLaserPoint_WorkpieceCloud.txt` 简化为 `index x y z`
  - 应用版本更新为 `v2026.05.22`
  - `Release x64` 打包通过，生成版本化安装包 `NoTeaching-Robot-Setup-v2026.05.22.exe`
  - 安装包默认 `Data` 参数改为从 Git index 导出，避免未提交现场配置混入安装包

## 2026-05-20

- 焊接轨迹与补偿
  - 焊接轨迹生成新增拐点圆弧过渡，默认半径 `2mm`
  - 圆弧处理放在焊道补偿之后，避免先圆弧再补偿导致轨迹形变
  - 补偿结果日志增加圆弧处数、半径和新增点数统计

- 点云特征与调试输出
  - 激光点云处理加入几何特征分析路径，继续用于优化起点、终点、内外拐点提取
  - 分类流程新增关键点输出文件，方便单独查看拐点和端点
  - 点云索引读取兼容超大索引，避免原始索引溢出导致读取失败

- CLI 与批处理
  - CLI 激光分类新增目录批处理入口，可一次处理结果目录下的 `PreciseLaserPoint.txt`
  - 非窗口类 CLI 命令执行完成后自动退出，减少脚本调用卡住

- 版本与发布
  - 应用版本更新为 `v2026.05.20`
  - `Release x64` 编译通过，剩余警告为 STEP SDK 宏重定义
  - 已生成新版安装包并随 Git 提交推送；Release 附件待 GitHub CLI 重新登录后补传

## 2026-05-19

- 平板触控与窗口策略
  - 新增虚拟键盘管理，支持文本键盘、数字九宫格、键盘模式切换和长按空白区域拖动
  - 数字类编辑框接入数字输入限制，速度、端口、IP 等参数减少误输入
  - 登录页和主界面恢复按最小尺寸打开，管理页面默认最大化

- 控制单元与引导流程
  - 管理页控制单元维护继续完善，新增四段式 IP 输入框
  - STEP/FANUC 新建向导按机器人类型切换默认 FTP 参数；FANUC 隐藏 STEP 工程名，STEP 隐藏不需要的监控端口
  - 相机基础参数拆成弹窗，向导第四步可打开相机基础参数，第五步可打开手眼标定

- 界面与稳定性
  - 当前用户入口改为下拉菜单，合并退出登录和管理页面入口
  - 修复相机 runtime 重载、窗口尺寸、滚动条预留、按钮重叠、下拉选中显示和标题栏颜色等问题
  - 安装包升级时保留现场 `Data` 机器人、账号和参数文件，默认模板允许覆盖更新

- 版本与发布
  - 应用版本更新为 `v2026.05.19.2`
  - 已准备 GitHub Release 安装包和发布说明

## 2026-05-18

- 控制单元管理与新建向导
  - 管理页新增控制单元管理，支持读取 `Data` 下机器人文件夹并维护启用状态、中文名、IP、端口、FTP、STEP 工程和工件类型
  - 新建控制单元加入向导流程，当前工件类型支持波纹板
  - 新建时从 `Data/WorkpieceTemplates/CorrugatedPlate` 复制测量焊接、相机、手眼、工艺和补偿默认参数

- 首次配置状态与功能禁用
  - 相机参数和手眼标定加入首次设置状态
  - 向导可跳过相机参数或手眼标定，但跳过后会禁用先测后焊、坡口相机预览等依赖功能
  - 相机参数保存、手眼矩阵保存后自动写入完成状态并刷新功能入口

- 参数兼容清理
  - 删除旧 `PreciseMeasureParam.ini`、`WeldLineParam.ini`、`WeldPara.ini`、`WeaveDate.ini`、`WeldCompParam.ini` 的读取和迁移逻辑
  - 测量焊接参数统一使用 `MeasureWeldParam.ini`
  - 工艺参数继续使用 `WeldPara.txt` / `WeaveDate.txt`

- 稳定性修复
  - 修复控制单元保存重载时，相机接收线程旧诊断信号晚到并访问已释放 runtime 导致崩溃的问题
  - 重建相机 runtime 时增加对象存活集合保护，旧信号到达后会直接忽略

- 版本与构建
  - 应用版本更新为 `v2026.05.18`
  - `Debug x64` 编译通过，0 错误，0 警告
  - 本次提交不包含安装包二进制

## 2026-05-12

- 多机器人相机接收链路重构
  - 取消旧的全局相机队列依赖，按机器人创建独立相机缓存
  - 共享接收模式下按 UDP 发送端 IP 分发到对应机器人缓存
  - 独立接收模式保留为备用，适合后续每台相机配置不同本地端口

- 相机配置与预览联动
  - 相机端口改为读取机器人对应的相机参数配置
  - 坡口相机预览显示当前相机 IP，方便现场确认 RobotA / RobotB 数据来源
  - 先测后焊、手眼标定和相机预览统一使用当前机器人对应的相机缓存

- 管理页与现场入口收口
  - 相机接收模式切换放入管理页面，主界面不再允许调整
  - 功能测试界面删除额外的相机接收调试块，只保留机器人/离线测试功能
  - 主界面继续保持大按钮用于流程和子界面，小工具仅保留现场必要操作

- 多机器人并发准备
  - 点动、先测后焊页面按机器人索引独立缓存窗口和运行对象
  - 相机缓存按机器人隔离，减少两个机器人同时操作时的数据串扰风险
  - 后续需要 RobotA / RobotB 同时运行扫描或点动做现场回归

- 版本与构建
  - 应用版本更新为 `v2026.05.12`
  - `Debug x64` 编译通过
  - 当前剩余警告为第三方库 PDB 缺失，不影响运行

## 2026-04-14

- [完成机器人驱动适配层与类型切换基础能力](https://www.notion.so/349c868d819b81d1abd5d3cf549a832e)
  - 完成 `RobotType` 分流
  - 建立 STEP / FANUC 两套驱动入口

## 2026-04-17

- [完成工艺参数界面、点动控制框架与主界面样式统一](https://www.notion.so/349c868d819b8155ba4bd2a678fe8200)
  - 完成主要子界面骨架
  - 主界面统一为深色风格

## 2026-04-20

- [完成先测后焊主流程、精测量参数编辑、相机参数与手眼矩阵配置](https://www.notion.so/349c868d819b8186995ad9b9bd1cddaa)
  - 预设参数流程已串通
  - 配套参数编辑界面已上线

- [完成 FANUC 速度口径修正、异常点过滤与结果文件格式整理](https://www.notion.so/349c868d819b81649327cb66c7c0a3c6)
  - 修正 `MOVJ / MOVL` 速度口径
  - 清理异常点参与激光计算的问题
  - 输出文件改为序号格式

- [完成主界面监控/日志拆分与功能入口整理](https://www.notion.so/349c868d819b81f082ccdc3fc643432b)
  - 机器人监控与日志区域拆分
  - 常用功能入口重新整理

## 2026-04-21

- [评估机器人指令参与手眼标定计算方案](https://www.notion.so/349c868d819b8153919ddf092a39201a)
  - 当前作为需求/方案项立项
  - 后续重点验证精度、便利性和耗时

## 2026-04-22

- 手眼标定界面继续优化
  - 调整为更紧凑的深色布局
  - 固定标定目标点与六组采样并排显示
  - 六组采样改为标签页切换，自动标定时自动切换到当前组
  - 自动标定开始前补充相机自动打开检查

- 手眼参数检测链路补充
  - 新增本地矩阵计算结果与机器人程序结果对比入口
  - 增加 `FANUC_HECHECK` 机器人侧检测程序模板
  - `PR[80]` 读取解析改为跳过元信息字段，按 `XYZWPR` 读取

- 当前待排查问题
  - 手眼矩阵计算结果存在偏差，待检查是否为矩阵本身问题
  - 机器人 `PR[80]` 获取结果仍有异常，待继续联调检测

## 2026-04-24

- 先测后焊焊接链路补齐
  - 接入焊道补偿后处理、下枪/收枪安全位、多点 TP 下发和 `R[93]` 完成判断
  - 增加关键节点确认弹窗，焊接轨迹改为中间点 `CNT` 过渡

- 历史结果复焊能力补充
  - 新增“跳过扫描焊接”，可直接选择历史结果目录
  - 读取 `PreciseLaserPoint_WeldPose_2mm.txt` 后，按当前补偿参数重生成 `PreciseLaserPoint_WeldPose_2mm_SeamComp.txt`

- 构建与发布整理
  - Release 构建稳定配置调整为 `/Zm2000` 且默认关闭 `/MP`
  - 重新生成并发布 `v2026.04.24` 安装包

## 2026-04-27

- 相机时间戳与扫描重复性排查
  - 精测量参数新增“相机读取帧率（fps）”
  - 扫描插值改为相机点等待机器人时间戳追上后再匹配
  - 增加全局相机帧缓存，避免多个功能直接竞争底层相机队列

## 2026-04-28

- 补偿参数界面完成
  - 补偿界面改为“姿态补偿 / 焊道补偿”切换模式
  - 支持机器人下拉、补偿类型下拉、参数重新读取和保存
  - 姿态补偿显示当前姿态值，焊道补偿隐藏姿态显示并切换字段名称

- 焊道补偿能力补充
  - 焊道补偿新增“焊道方向补偿”
  - 先测后焊补偿后处理同时支持 Z 向、枪反向和焊道方向补偿

- 界面与保存逻辑统一
  - 下拉框样式抽成通用模板，统一箭头和边框显示
  - 手动保存类界面增加关闭前未保存修改提示
  - 标题栏深色、补偿数值编辑框右对齐

- 版本发布
  - 应用版本更新为 `v2026.04.28`
  - 生成 Release 安装包并作为 GitHub Release 附件发布

## 2026-04-30

- 先测后焊特征提取链路更新
  - 原始点云改为先走新的拐点提取函数
  - 拐点之间按 2mm 间距扩充，再生成焊接姿态与补偿文件
  - 直通流程不再依赖旧分类器

- 当前位置获取联调
  - 主动和被动读取都改为先读 `PR[3] / PR[4]`
  - 再用 `JOINT2POS` 和当前关节位姿反算直角位置
  - 用于排查工具坐标和示教器显示不一致的问题
  - 原因是 `CURPOS` 只能直接拿到法兰位置，单独使用时容易与示教器显示的 TCP 位置不一致

- 机器人点动界面修复
  - 修复后台线程直接弹窗导致的 GUI thread 断言
  - T 轴点动结束提示改为回到界面线程处理

- 发布准备
  - 应用版本同步更新为 `v2026.04.30`
  - 本次变更准备重新打包安装包并同步 GitHub / Notion 记录

## 2026-05-07

- 主界面与功能入口重构
  - 主界面恢复为大按钮快速操作页，面向新手保留连接服务、先测后焊、坡口相机预览、读取当前位置和标定入口
  - 新增管理页面，集中放置工艺参数、精测量参数、相机/标定、焊道补偿、点动控制和功能测试等高级功能
  - 主功能页改为在主界面内刷新显示，避免多个独立子窗口在任务栏难以区分
  - 首页标题仅在主页显示，功能页打开时直接铺满主界面内容区

- 人员账号与权限控制
  - 新增本地账号登录、退出、注册和角色权限
  - 权限分为操作员、工程师、管理员
  - 首次启动自动创建默认管理员账号，账号数据保存在本地 `Data/Accounts.ini`
  - 高级入口增加工程师/管理员权限校验，注册账号需要管理员权限
  - `Data/Accounts.ini` 已加入 `.gitignore`，避免提交现场账号数据

- 先测后焊与焊道补偿修复
  - 焊接下枪安全位置方向修正，避免补偿后 X 方向反向
  - RZ 计算增加斜面旋转限制，两个斜面旋转角限制在 `+-20` 度内
  - 波纹板焊道补偿改用稳定参考法向，减少锁定姿态后因局部法向变化造成的轨迹不连续、重叠和间距异常
  - 补偿后轨迹验证恢复为约 `2mm` 相邻点间距

- 时间戳与相机帧采集排查
  - 增加机器人与相机时间戳验证功能，用于 1 分钟采样对比时间间隔
  - FANUC 被动监控时间戳接入扫描匹配链路，避免使用 PC 接收时间代替机器人自身时间
  - 增加 CLI 上传 FANUC 服务程序能力，便于现场直接更新服务端程序

- 稳定性与内存增长控制
  - 相机全局帧缓存上限从 `50000` 降到 `8000`
  - 相机缓存清理改为释放底层 deque 占用，停止坡口相机接收时同步清缓存
  - 先测后焊扫描改为拉取相机帧后立即提取时间戳/目标点，不再额外保存一份完整点云帧，降低扫描峰值内存
  - UDP 拼包增加最大包和最大缓冲保护，异常包不会持续撑大接收缓冲
  - 主要 UI 日志框增加最大行数，避免长时间运行日志无限增长
  - 去掉相机每帧 `qDebug()` 输出，减少调试运行时的输出压力

- 构建验证
  - `Release x64` 编译通过
  - `Debug x64` 编译通过
  - 当前剩余警告为既有 STEP 宏重定义、第三方 PDB 缺失或点云帧头大小转换警告，不影响本次功能运行

## 2026-05-08

- 版本发布与说明同步
  - 应用版本更新为 `v2026.05.08`
  - README、安装脚本默认版本和启动器版本号已同步
  - 本地登录状态 `Data/LoginState.ini` 加入 `.gitignore`，避免提交记住密码/自动登录状态

- STEP / FANUC 通用化继续推进
  - `RobotDriverAdaptor` 补充通用被动读取接口，FANUC 使用机器人端 `robot_ms`，STEP/其他驱动暂用 PC steady 时间轴
  - 连接、FTP、读取位置/关节、变量读写、速度设置、任务调用、MOVL/MOVJ、点动目标运动优先走通用驱动接口
  - 功能测试界面改为“机器人功能测试区”，通用测试支持 STEP/FANUC；FANUC LS 上传与 CURPOS/raw 诊断明确作为 FANUC 专用功能
  - STEP 默认配置补入 `RobotB`，主页切换机器人时按已创建驱动判断可用性

- 先测后焊与手眼标定流程同步
  - 先测后焊扫描位姿采集使用通用被动时间轴，STEP 当前使用 PC steady ms 参与相机时间映射
  - STEP 焊道执行继续走 `ContiMoveAny` 生成、上传和启动轨迹，FANUC 继续走多点 TP/服务流程
  - 手眼自动标定主流程改为读取位置变量并调用通用运动接口；机器人侧 `FANUC_HECHECK` 对比仍保留为 FANUC 专用

- 主界面与登录体验收口
  - 主页机器人下拉框显示“中文名 / 驱动类型 (RobotName)”，并修复 GBK 配置中文名乱码
  - 不可用或未知驱动会禁用现场操作按钮，并提示“机器人类型错误/不可用”，管理页面仍可使用
  - 大按钮主页默认最大化，普通子界面统一增加右上角“返回主页”按钮，返回时仍尊重未保存/运动中拦截
  - 登录页继续收口为深色单卡片模式，支持登录/注册切换、保存密码、自动登录、账号记忆、游客登录和主页退出登录

- 当前仍保留的专用/待补项
  - FANUC 服务包上传、LS 上传、CURPOS/raw 诊断仍为 FANUC 专用
  - STEP 长按连续点动暂未实现，当前提示使用单击步进或目标点运动
  - STEP 机器人侧手眼验证尚未接入，需要后续根据 STEP 可用指令另做方案

- 构建验证
  - `Debug x64` 编译通过

## 2026-05-11

- 测量焊接参数重构
  - 将原精测量参数和焊接参数合并到 `MeasureWeldParam.ini`
  - 按位置类型分组保存，支持分组下拉、新建、复制、删除和中文名编辑
  - 新建参数组默认为全 0，复制参数组继承当前组的扫描和焊接参数

- 先测后焊参数读取
  - 先测后焊流程改为读取当前选中位置类型下的扫描参数和焊接参数
  - 运行日志中输出当前使用的参数组，降低错用旧配置的风险

- 参数界面布局优化
  - 右侧参数区拆分为“扫描参数 / 焊接参数”两个标签页
  - 折叠参数项改为直角标签式显示，输入框使用固定宽度
  - 修复全屏后输入框被横向拉长、折叠项展开时左右区域联动变形的问题

- 构建验证
  - `Debug x64` 编译通过
  - 0 错误，0 警告

## 2026-05-09

- RobotB / STEP 现场适配
  - 补齐 `Data/RobotB` 参数文件，RobotB 默认按 STEP 驱动配置
  - STEP 连接后自动执行清报警、切自动模式和上使能
  - 主页连接按钮改为连接/断开状态切换，并新增清除报警、STEP 模式切换入口

- STEP Job 与运动链路修复
  - STEP 动态运动程序统一上传到 `PCRobot` 工程
  - `SRP/SRD` 生成改为现场可加载的 LF 换行格式
  - `ContiMoveAny` 支持直角/关节点变量类型，工程变量读取按 `_project` 作用域处理
  - 运动失败时返回最近机器人状态/报警，便于现场判断未使能、程序运行中或参数加载失败

- 点动和自动标定保护
  - STEP 自动运行状态下禁止点动，避免运行程序未退出时误发移动
  - 点动界面修复后台线程创建弹窗导致的 Qt GUI thread 断言
  - 手眼自动标定启动前只要求相机打开，不再因三维点为空提前阻断
  - `CheckDone` 后增加目标位置复核，防止机器人没有实际移动却被误判为到位

- 相机与主界面联动
  - 相机参数/预览页取消独立机器人选择，直接使用主界面当前机器人
  - 主界面切换机器人后重新打开相机页，会按当前机器人重新加载对应相机和手眼配置

- 构建验证
  - `Debug x64` 编译通过
  - 当前剩余警告为 `orocos-kdld.pdb`、`Robot-SDKd.pdb` 第三方调试符号缺失
