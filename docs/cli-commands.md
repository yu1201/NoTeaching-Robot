# CLI 命令说明书

更新时间：2026-06-10

适用程序：`QtWidgetsApplication4.exe` / `NoTeaching-Robot`

本文档按当前程序源码中的命令行入口整理，主要对应 `src/main.cpp` 和 `src/QtWidgetsApplication4.cpp` 的 `ApplyStartupArguments`、`RunCommandLineActions` 以及各 `Run*ForCli` 实现。

## 1. 基本用法

```powershell
QtWidgetsApplication4.exe [命令] [参数...]
```

常用启动方式：

```powershell
QtWidgetsApplication4.exe --help-cli
QtWidgetsApplication4.exe --no-show --laser-classify "Result\RobotB\20260520_1\LaserPoint\PreciseLaserPoint.txt"
QtWidgetsApplication4.exe --no-show --robot RobotB --robot-movel-relative "0,0,10" --robot-speed 300
```

执行规则：

- 程序启动后会解析 `app.arguments()`，有参数时会打开命令行控制台并执行 CLI 动作。
- `--no-show` 会隐藏主窗口，适合自动化、批处理、现场脚本。
- CLI 日志会输出到控制台，并按天归档追加到 `Log/<yyyy-MM-dd>/CliLog.txt`。
- 多个命令可以组合使用；通用机器人运动和 FANUC `--fanuc-raw` / `--fanuc-call` 会按参数顺序执行。
- 当前实现主要通过日志报告失败，进程退出码不作为可靠的成功/失败判断依据。

## 2. 全局命令

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--help-cli` | 无 | 打印程序内置 CLI 帮助并退出。 |
| `--no-show` | 无 | 不显示主窗口。若没有打开窗口类命令，CLI 动作结束后会自动退出。 |
| `--quit-after` | `<ms>` | 指定若干毫秒后退出程序，可和打开窗口类命令一起使用。 |

说明：

- `--quit-after` 参数必须是非负整数。
- 只有 `--open-*` 窗口命令会阻止默认自动退出；如需窗口打开后自动关闭，请配合 `--quit-after`。

## 3. 打开界面窗口

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--open-function-test` | 无 | 打开机器人功能测试窗口。 |
| `--open-jog` | 无 | 打开机器人点动控制窗口。 |
| `--open-precise-measure` | 无 | 打开测量焊接参数窗口。 |
| `--open-camera-param` | 无 | 打开相机参数窗口。 |

示例：

```powershell
QtWidgetsApplication4.exe --open-jog
QtWidgetsApplication4.exe --open-precise-measure --quit-after 60000
```

## 4. 通用机器人 CLI

### 4.1 机器人选择

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--robot` | `<UnitNo|RobotA|RobotB|中文名>` | 选择通用机器人 CLI 目标。 |
| `--robot-unit` | `<UnitNo|RobotA|RobotB|中文名>` | 隐藏兼容别名，逻辑同 `--robot`。 |

选择规则：

- 未指定时，默认使用当前机器人单元；若当前单元不可用，则尝试第一个可用机器人。
- 选择值可以是机器人单元编号、列表序号、`sUnitName`、中文名称、驱动机器人名、自定义名或显示标签。

### 4.2 运动与连接

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--robot-connect` | 无 | 连接选中的机器人驱动，只连不动。 |
| `--robot-movel` | `<X,Y,Z,RX,RY,RZ[,BX,BY,BZ]>` | 发送绝对直角 `MOVL`，默认速度 `500 mm/min`。 |
| `--robot-movel-relative` | `<DX,DY,DZ[,DRX,DRY,DRZ,BX,BY,BZ]>` | 读取当前位置后叠加增量，再发送直角 `MOVL`。 |
| `--robot-movj` | `<S,L,U,R,B,T[,EX1,EX2,EX3]>` | 发送关节脉冲 `MOVJ`，默认速度 `1`。 |
| `--robot-speed` | `<VALUE>` | 覆盖本次运动速度；`MOVL` 按 `mm/min`，`MOVJ` 按驱动百分比/约定。 |
| `--robot-done-delay` | `<ms>` | 运动完成轮询间隔，默认 `200 ms`。 |
| `--robot-no-wait` | 无 | 运动下发后不等待完成。 |

参数格式：

- 坐标和脉冲列表支持逗号、中文逗号、顿号、分号、竖线或空白分隔。
- `--robot-movel` 接受 6 到 9 个数；后 3 个为外部轴 `BX/BY/BZ`，不填按 0。
- `--robot-movel-relative` 接受 3 到 9 个数；未给姿态/外部轴增量时按 0。
- `--robot-movj` 接受 6 到 9 个数，数值会四舍五入为脉冲整数。

示例：

```powershell
QtWidgetsApplication4.exe --no-show --robot RobotB --robot-connect
QtWidgetsApplication4.exe --no-show --robot RobotB --robot-movel "100,200,300,180,0,90" --robot-speed 500
QtWidgetsApplication4.exe --no-show --robot RobotB --robot-movel-relative "0 0 10" --robot-speed 300 --robot-no-wait
QtWidgetsApplication4.exe --no-show --robot RobotB --robot-movj "0,1000,2000,0,0,0" --robot-speed 1
```

## 5. FANUC 服务 CLI

FANUC CLI 当前使用程序内第一个 FANUC 驱动目标，不走 `--robot` 选择逻辑。

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--fanuc-connect` | 无 | 连接 FANUC 常驻服务端口。 |
| `--fanuc-upload-services` | 无 | 上传/编译 FANUC 服务库、常驻服务、监控服务、作业运行器、点动程序和固定 TP。 |
| `--skip-upload-wait` | 无 | 上传服务后不等待回车，适合自动化脚本。 |
| `--fanuc-curpos-diag` | 无 | 运行当前位置和 `PR[20]` 相关诊断命令。 |
| `--fanuc-pr20-diag` | 无 | 仅读取 FANUC `PR[20]` 诊断点。 |
| `--fanuc-raw` | `<CMD>` | 发送一条原始 FANUC 服务命令。可重复传入多条。 |
| `--fanuc-call` | `<PROGRAM>` | 调用机器人程序。可重复传入多个程序名。 |

`--fanuc-upload-services` 当前上传内容：

- `SDK/FANUC/FanucServiceLib.kl`
- `SDK/FANUC/FanucResidentService.kl`
- `SDK/FANUC/FanucMonitorService.kl`
- `SDK/FANUC/FanucJobRunner.kl`
- `SDK/FANUC/LOADJOGBUF.kl`
- `SDK/FANUC/STARTALL.tp` 到 `/md/STARTALL.tp`
- `SDK/FANUC/FANUC_JOGL.ls`
- `SDK/FANUC/FANUC_JOGJ.ls`

示例：

```powershell
QtWidgetsApplication4.exe --no-show --fanuc-connect
QtWidgetsApplication4.exe --no-show --fanuc-upload-services --skip-upload-wait
QtWidgetsApplication4.exe --no-show --fanuc-raw "GET_CUR_POS"
QtWidgetsApplication4.exe --no-show --fanuc-call "STARTALL"
QtWidgetsApplication4.exe --no-show --fanuc-curpos-diag
```

## 6. 先测后焊扫描 CLI

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--measure-then-weld-scan-only-repeat` | `<N>` | 自动执行先测后焊扫描流程 `N` 次，只到收枪安全位置，不执行焊接。 |
| `--measure-then-weld-scan-speed` | `<mm/min>` | 覆盖本次 CLI 扫描速度，不修改 ini。 |
| `--measure-then-weld-camera-offset-ms` | `<ms>` | 覆盖本次 CLI 相机时间补偿，不修改 ini。 |

执行内容：

- 加载先测后焊预设参数。
- 按 `--robot` / `--robot-unit` 指定的机器人运行；不指定时使用当前或第一个可用机器人。
- 确保所选机器人单元的扫描相机运行。
- 每轮执行起点安全位、扫描起点、扫描采集、终点安全位。
- STEP / FANUC 都走通用机器人驱动；只有 `--fanuc-*` 命令仍依赖 FANUC 服务连接。

示例：

```powershell
QtWidgetsApplication4.exe --no-show --robot RobotC --measure-then-weld-scan-only-repeat 3
QtWidgetsApplication4.exe --no-show --robot RobotC --measure-then-weld-scan-only-repeat 5 --measure-then-weld-scan-speed 600 --measure-then-weld-camera-offset-ms -35
```

## 7. 离线激光点云与焊道处理 CLI

### 7.1 激光点云分类

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--laser-classify` | `<FILE>` | 对单个激光点云文件做去噪、拟合、起终点和拐点分类。 |
| `--laser-classify-output` | `<FILE>` | 指定分类结果输出文件。 |
| `--laser-classify-dir` | `<DIR>` | 递归批量处理目录下所有 `PreciseLaserPoint.txt`。 |

单文件默认输出：

- `<base>_Classified.txt`
- `<base>_KeyPoints.txt`
- `<base>_Classified_Noise.txt`

批量目录默认输出：

- `<base>_Classified_Geometry.txt`
- `<base>_KeyPoints_Geometry.txt`
- `<base>_Classified_Geometry_Noise.txt`

分类结果字段：

```text
# index x y z type_code type_name source
```

当前类型码：

| type_code | type_name |
| --- | --- |
| `1` | `start` |
| `2` | `end` |
| `3` | `inner_corner` |
| `4` | `outer_corner` |
| `5` | `normal` |
| `6` | `noise` |

示例：

```powershell
QtWidgetsApplication4.exe --no-show --laser-classify "Result\RobotB\20260520_1\LaserPoint\PreciseLaserPoint.txt"
QtWidgetsApplication4.exe --no-show --laser-classify "Result\RobotB\20260520_1\LaserPoint\PreciseLaserPoint.txt" --laser-classify-output "D:\out\point_classified.txt"
QtWidgetsApplication4.exe --no-show --laser-classify-dir "Result\RobotB"
```

### 7.2 焊缝补偿

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--apply-weld-seam-comp` | `<FILE>` | 对焊道姿态文件应用 `WeldSeamCompParam.ini` 补偿。 |
| `--apply-weld-seam-comp-output` | `<FILE>` | 指定补偿结果输出文件；默认另存 `_SeamComp`。 |

说明：

- 机器人名会优先从路径中的 `Result/<RobotName>/...` 推断；失败时默认 `RobotA`。
- 输出路径不能和输入路径相同。

示例：

```powershell
QtWidgetsApplication4.exe --no-show --apply-weld-seam-comp "Result\RobotB\20260520_1\LaserPoint\PreciseLaserPoint_WeldPose_2mm.txt"
QtWidgetsApplication4.exe --no-show --apply-weld-seam-comp "D:\pose.txt" --apply-weld-seam-comp-output "D:\pose_SeamComp.txt"
```

### 7.3 重建先测后焊文件

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--rebuild-measure-weld-files` | `<DIR>` | 从已有 `LaserPoint` 目录重建 `PreciseLaserPoint_PreservePath_2mm.txt`、`PreciseLaserPoint_WeldPose_2mm.txt` 和补偿文件。 |
| `--pointcloud-processing-mode` | `sdk` / `sdkfit` / `cloudfit` / `legacy` | 本次运行临时覆盖精测点云处理方法（依次对应 SDK点云算法全处理 / SDK点云算法+拟合 / 点云算法+拟合 / 特征点+拟合），不写入配置。 |

说明：

- 目录通常是一次结果下的 `LaserPoint` 文件夹。
- 机器人和参数读取仍按 `--robot` / 当前机器人选择。
- 适合跳过重新扫描，只对已有点云重新生成姿态和补偿结果。
- 滤波拟合数值参数（Z阈值/段间跳变/步长等）与采样主轴读取精测点云处理界面保存的配置，与现场实际运行一致。

示例：

```powershell
QtWidgetsApplication4.exe --no-show --robot RobotC --rebuild-measure-weld-files "Result\RobotC\20260528_020\LaserPoint"
```

### 7.4 生成 STEP 焊接程序

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--generate-step-weld-program` | `<FILE>` | 根据焊接姿态文件生成 STEP `Weld_时间.srp/.srd`。默认生成实际焊接程序，包含 `ARCON/ARCOFF`。 |
| `--generate-step-weld-program-output-dir` | `<DIR>` | 指定 STEP 焊接程序输出目录，默认 `Job\STEP`。 |
| `--generate-step-weld-program-dry-run` | 无 | 按空跑轨迹生成 STEP 文件，不生成 `ARCON/ARCSET/ARCOFF` 焊接指令。 |
| `--generate-step-weld-speed` | `<mm/min>` | 覆盖本次 STEP 文件轨迹速度，不修改 ini。 |

说明：

- 机器人名会优先从路径中的 `Result/<RobotName>/...` 推断；失败时默认 `RobotA`。
- 输出日志会包含程序名、生成摘要、`.srp` 和 `.srd` 路径。

示例：

```powershell
QtWidgetsApplication4.exe --no-show --generate-step-weld-program "Result\RobotB\20260520_1\LaserPoint\PreciseLaserPoint_WeldPose_2mm_SeamComp.txt"
QtWidgetsApplication4.exe --no-show --generate-step-weld-program "D:\pose_SeamComp.txt" --generate-step-weld-program-output-dir "Job\STEP" --generate-step-weld-speed 350
QtWidgetsApplication4.exe --no-show --generate-step-weld-program "D:\pose_SeamComp.txt" --generate-step-weld-program-dry-run
```

### 7.5 更新焊道平均姿态库

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `--update-weld-pose-average` | `<FILE_OR_DIR>` | 离线统计四类焊道平均姿态，并更新机器人补偿姿态库。 |

说明：

- 输入可以是文件或目录。
- 输入为目录时，会查找 `PreciseLaserPoint_WeldPose_2mm.txt`，包括当前目录、`LaserPoint/` 子目录和递归子目录。
- 默认更新 `Data/<RobotName>/WeldPoseCompParam.ini`。
- 默认报告输出为 `PreciseLaserPoint_WeldPose_2mm_PoseAverageReport.txt`。
- 当前统计四组典型段：低平台、上升边、高平台、下降边。

示例：

```powershell
QtWidgetsApplication4.exe --no-show --update-weld-pose-average "Result\RobotB\20260520_1"
QtWidgetsApplication4.exe --no-show --update-weld-pose-average "Result\RobotB\20260520_1\LaserPoint\PreciseLaserPoint_WeldPose_2mm.txt"
```

## 8. 推荐批处理模板

### 8.1 点云分类到焊接程序

```powershell
QtWidgetsApplication4.exe --no-show `
  --laser-classify "Result\RobotB\case1\LaserPoint\PreciseLaserPoint.txt" `
  --apply-weld-seam-comp "Result\RobotB\case1\LaserPoint\PreciseLaserPoint_WeldPose_2mm.txt" `
  --generate-step-weld-program "Result\RobotB\case1\LaserPoint\PreciseLaserPoint_WeldPose_2mm_SeamComp.txt" `
  --generate-step-weld-program-output-dir "Job\STEP" `
  --quit-after 1000
```

### 8.2 现场只做扫描验证

```powershell
QtWidgetsApplication4.exe --no-show `
  --robot RobotC `
  --measure-then-weld-scan-only-repeat 3 `
  --measure-then-weld-scan-speed 600 `
  --measure-then-weld-camera-offset-ms -35
```

### 8.3 FANUC 服务更新

```powershell
QtWidgetsApplication4.exe --no-show --fanuc-upload-services --skip-upload-wait
```

## 9. 注意事项

- FANUC 服务上传后，若未加 `--skip-upload-wait`，程序会等待回车，方便现场人员在示教器上重启 `STARTALL`。
- `--fanuc-curpos-diag` 会连续发送多条当前位置/PR 读取命令，适合排查 FANUC 常驻服务解析问题。
- 离线命令读取和写入文件时，应避免把输出路径设成输入路径。
- 通用机器人运动命令会自动尝试连接机器人；连接失败时只记录日志，不继续执行运动。
- 现场脚本建议统一加 `--no-show`，并把输入输出路径写成明确路径，减少工作目录差异导致的找不到文件问题。
