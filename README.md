# NoTeaching-Robot

免示教机器人上位机项目，基于 Qt + Visual Studio 开发，当前主要集成了：

- STEP 机器人驱动
- FANUC 机器人驱动
- FANUC KAREL/TP 文件生成、发送与调用
- 机器人点动、状态监控、功能测试
- 先测后焊流程
- 相机参数、手眼矩阵、精测量参数编辑

## 下载

- 仓库首页：<https://github.com/yu1201/NoTeaching-Robot>
- 最新安装包页面：<https://github.com/yu1201/NoTeaching-Robot/releases/latest>
- 当前版本安装包：<https://github.com/yu1201/NoTeaching-Robot/releases/tag/v2026.04.21>

建议普通使用者直接从 `Releases` 页面下载安装包 `NoTeaching-Robot-Setup.exe`，不用自己编译源码。

## 开发环境

当前工程按本机路径配置，直接编译前需要先准备以下依赖：

- Visual Studio 2022
- Qt 6.7.3 MSVC 2022 x64
- OpenCV 4.6.0
- Eigen 3.4.0
- Orocos KDL

工程里仍保留了 STEP/FANUC 相关接口、KAREL 源文件和示例 LS 文件，但运行期 DLL、`x64` 输出目录、日志和结果文件都已排除，不再上传到仓库。

## 目录说明

- `include`：头文件
- `src`：源码实现
- `Data`：机器人参数、工艺参数、相机参数
- `SDK/FANUC`：FANUC KAREL、LS、测试程序
- `SDK/STEP`：STEP SDK 头文件和链接库
- `Job`：示例任务/缓存任务文件
- `icons`：界面图标资源

## 文档

项目代码和 Notion 文档是绑定维护的。GitHub 里保留了一份便于直接预览的 Markdown 版本，Notion 继续作为协作主文档。

- Notion 总入口：<https://www.notion.so/336c868d819b80a181c7ef8a393b53ad>
- GitHub 文档索引：[`docs/README.md`](docs/README.md)
- 工作流程：[`docs/workflow.md`](docs/workflow.md)
- 界面说明：[`docs/ui.md`](docs/ui.md)
- 波纹板项目方案：[`docs/wavy-board-plan.md`](docs/wavy-board-plan.md)
- 问题表摘要：[`docs/issues.md`](docs/issues.md)
- 工作记录摘要：[`docs/worklog.md`](docs/worklog.md)

## 当前功能

### STEP / FANUC 驱动

- 根据 `RobotType` 自动切换机器人驱动
- STEP 常规控制接口
- FANUC TCP 常驻服务通信
- FANUC LS/KL/PC 文件发送与调用
- MOVJ / MOVL / 连续点动 / 固定 TP 调用

### UI 界面

- 主界面统一深色风格
- 功能测试子界面
- 机器人点动控制子界面
- 工艺参数编辑界面
- 精测量数据修改界面
- 手眼矩阵参数界面
- 先测后焊流程界面

### 流程与数据

- 扫描安全位采用脉冲点
- 扫描起点/终点采用直角坐标
- 扫描过程中采集相机点、机器人位姿、激光点
- 激光点计算支持时间插值
- 支持按机器人和相机维度独立保存配置

## 构建说明

1. 用 Visual Studio 2022 打开 `QtWidgetsApplication4.sln`
2. 确认本机 Qt / OpenCV / Eigen / Orocos KDL 路径与工程配置一致
3. 编译 `Debug|x64` 或 `Release|x64`

说明：

- `QtWidgetsApplication4.vcxproj` 当前仍带有本机绝对路径配置，如果换电脑，需要先改 Qt / OpenCV / Eigen / KDL 路径
- STEP 的 `Robot-SDKd.lib` / `Robot-SDK.lib` 需要自行放到 `SDK/STEP`

## 仓库精简说明

这次已经把下面这些内容从仓库管理中排除了：

- Visual Studio 中间文件和编译输出
- 运行日志、结果文件、临时目录
- 本地备份参数文件
- STEP / KAREL 手册和压缩包
- 重复的 SDK 副本目录

如果需要保留本地文档或备份文件，可以继续放在工程目录里，但不会再进入 Git。
