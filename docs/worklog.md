# 工作记录摘要

- 人工整理日期：`2026-06-16`
- Notion 页面：<https://www.notion.so/1eb0a83f808e4cdd84d554753436275f>

这份文档按日期整理当前阶段已经完成或已立项的关键工作项，详细表结构仍以 Notion 为准。

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
