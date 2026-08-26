# 工作流程

- 人工整理日期：`2026-05-18`
- Notion 页面：<https://www.notion.so/336c868d819b8193871ee2c86237c6f0>

## 总体说明

当前工程已经具备 STEP / FANUC 两套驱动入口。FANUC 继续走常驻服务/TP 文件方案，STEP 现场流程逐步接入通用驱动接口与 `PCRobot` 工程动态 Job。控制单元管理负责维护机器人启用状态、通讯参数、工件类型和首次配置状态。

- `RobotType=1`：新时达 STEP 驱动
- `RobotType=2`：FANUC 驱动

驱动创建逻辑已经在控制单元中按类型切换，便于后续继续补第三方机器人。

## 控制单元与首次配置

- 管理页面从 `Data/ConfigStore.db` 读取已保存的控制单元与机器人作用域。
- 每个控制单元可维护中文名、IP、端口、FTP、STEP 工程路径、工件类型和启用状态。
- 新建控制单元通过向导完成，当前工件类型支持波纹板，并从 `workpiece/CorrugatedPlate/*` 数据库模板模块复制默认参数。
- 相机参数和手眼标定可跳过；跳过后依赖功能入口会禁用，完成保存后自动解除限制。

## FANUC 当前落地流程

### 1. 服务准备

- 发送 FANUC 常驻服务程序
- 机器人侧手动运行 `STARTALL`
- PC 通过 TCP 与常驻服务保持连接

说明：

- 当前如果服务程序有更新，推荐先停止旧服务，再发送文件，然后人工重启 `STARTALL`
- 这是为了避免机器人仍运行旧版本程序

### 2. 参数准备

先测后焊流程读取：

- 当前机器人 `MeasureWeldParam` 数据库模块
- 当前机器人 `CameraParam` 数据库模块
- 当前机器人和相机对应的 `HandEyeMatrix/<CameraSection>` 数据库模块

其中：

- 下枪安全位、收枪安全位使用脉冲点
- 扫描起点、扫描终点使用直角坐标

### 3. 先测后焊流程顺序

当前主流程顺序为：

1. 读取参数并检查机器人/相机配置
2. 打开相机接收
3. 确认后执行 `MOVJ` 到下枪安全位
4. 确认后执行 `MOVL` 到扫描起点
5. 扫描段执行 `MOVL` 到扫描终点，同时采集相机点和机器人位姿
6. 确认后执行 `MOVJ` 到收枪安全位
7. 保存目标点云、完整工件点云和后续焊接结果文件

每一段动作前都有确认弹窗，取消后会退出当前流程。

焊接执行阶段的下枪/收枪安全位由最终焊接轨迹首尾点计算：在世界 Z+ 抬升的同时，沿“安全位回撤方向”做等权水平回撤。`自动`保留旧算法（按焊道法向和枪轴择向、世界 X− 优先）；固定 X−、X+、Y−、Y+ 时不再依赖焊道朝向。`收下枪安全距`表示三维合成位移长度，必须大于 0 mm。

## 速度约定

### MOVL

- 扫描速度参数 `ScanSpeed` 以 `mm/min` 保存
- 下发 FANUC 时转换成 `mm/sec`

例如：

- `ScanSpeed=2000`
- 实际下发约 `33.333 mm/sec`

### MOVJ

- FANUC 固定 TP 程序中的 `MOVJ` 速度使用百分比
- 当前驱动兼容两种输入习惯：
  - `20` 表示 `20%`
  - `2000` 也表示 `20%`

## 采样与插值

- 相机帧按机器人维度缓存，缓存默认约 `2000` 帧
- 机器人监控线程按机器人维度常驻采样，供当前位置和扫描插值使用
- 激光点计算时，以相机时间为主时间轴
- 使用相邻两个机器人时间戳做线性插值

FANUC 使用机器人侧时间轴，STEP 当前仍临时使用 PC steady 时间轴参与匹配。

## 输出文件

结果默认保存到：

- `Result/<RobotName>/yyyyMMdd_NNN/CameraPoint/PreciseCameraPoint.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/RobotPoint/PreciseRobotPoint.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/LaserPoint/PreciseLaserPoint.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/LaserPoint/PreciseLaserPoint_WorkpieceCloud.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/LaserPoint/PreciseLaserPoint_PreservePath_2mm.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/LaserPoint/PreciseLaserPoint_WeldPose_2mm.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/LaserPoint/PreciseLaserPoint_WeldPose_2mm_SeamComp.txt`

当前文件内容为：

- 相机点：`index,x,y,z,error`
- 机器人位姿：`index,x,y,z,rx,ry,rz,bx,by,bz`
- 激光点：`index,x,y,z`

内部插值仍使用时间戳，但落盘时改成序号，便于后处理。

## 当前未完成项

- `线扫处理` 入口已预留，尚未接入正式算法
- STEP 已接入连接、FTP、读取位置/关节、变量读写、MOVL/MOVJ、点动目标运动和先测后焊执行入口，仍需继续现场验证动态 Job 加载与运动状态反馈
- 相机真实时间戳已接入，STEP 被动时间戳当前暂用 PC steady 时间轴
- 手眼标定优化方案仍在评估中
