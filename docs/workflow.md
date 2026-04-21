# 工作流程

- 人工整理日期：`2026-04-21`
- Notion 页面：<https://www.notion.so/336c868d819b8193871ee2c86237c6f0>

## 总体说明

当前工程已经具备 STEP / FANUC 两套驱动入口，但先测后焊主流程目前主要按 FANUC 方案打通。

- `RobotType=1`：新时达 STEP 驱动
- `RobotType=2`：FANUC 驱动

驱动创建逻辑已经在控制单元中按类型切换，便于后续继续补第三方机器人。

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

- `PreciseMeasureParam.ini`
- 相机参数 ini
- 手眼矩阵 ini

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
7. 保存扫描结果文件

每一段动作前都有确认弹窗，取消后会退出当前流程。

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

- 相机采样当前按约 `10ms` 频率处理
- 机器人位姿当前按约 `50ms` 周期读取
- 激光点计算时，以相机时间为主时间轴
- 使用相邻两个机器人时间戳做线性插值

当前相机真实时间戳尚未正式接入，暂时使用 PC 接收时刻模拟。

## 输出文件

结果默认保存到：

- `Result/<RobotName>/yyyyMMdd_NNN/CameraPoint/PreciseCameraPoint.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/RobotPoint/PreciseRobotPoint.txt`
- `Result/<RobotName>/yyyyMMdd_NNN/LaserPoint/PreciseLaserPoint.txt`

当前文件内容为：

- 相机点：`index,x,y,z,error`
- 机器人位姿：`index,x,y,z,rx,ry,rz,bx,by,bz`
- 激光点：`index,x,y,z`

内部插值仍使用时间戳，但落盘时改成序号，便于后处理。

## 当前未完成项

- `线扫处理` 入口已预留，尚未接入正式算法
- STEP 驱动还没有正式并入先测后焊主流程
- 相机真实时间戳仍需联调
- 手眼标定优化方案仍在评估中
