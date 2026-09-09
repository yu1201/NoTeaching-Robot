# 只读运动学辨识实验流程

人工整理日期：2026-09-06

## 范围与边界

用于比较“本地模型计算”与“控制器计算”，不是实体机器人精度标定。当前为独立工程实验工具，
尚未接入功能测试 UI、生产正逆解或扫描流程，不替代 `RobotDriverAdaptor` 的业务入口。
后续生产接入只能通过该适配层，品牌协议仍由品牌底层负责。

- 仅使用计算/读取接口；不登录、不申请许可、不运动、不上电、不切换模式、不复位、不改工具/工件。
- FTP 仅读取 `/RobotParams/MachineParams.json`，不上传、不删除、不修改控制器文件。
- `scripts/inovance_kinematics_reference.py` 是实验的汇川只读数据源；通用采样、拟合和验收不发送品牌命令。
- 每轮创建独立 `experiment.sqlite`，保存原始请求/应答、采样计划、工具负载条件、源指纹、候选模型和报告。
- 不打开生产 `Data/ConfigStore.db`，不自动装载候选模型。JSON/Markdown 仅为诊断导出，不是业务配置文件。
- 绑定地址、型号、固件和机械参数文件哈希；未获取硬件唯一序列号，不能凭 RobotC 标签宣称具有跨设备可移植性。
- 外部轴、机器人持工件、移动工件当前不支持。工具/工件/负载号是显式的计算参数，不更改活动状态。
- 采样前后校验来源、工具/工件内容及活动编号；中途定期检查停止状态。变化或错误时保留未完成数据库并拒绝拟合，
  不发送停止/恢复命令，不自动重连重试，不让部分数据冒充完成。

## 运行

依赖 Python 和 NumPy；不需要 SciPy。先运行纯离线测试：

```powershell
python scripts/tests/test_kinematics_identification.py
```

在本进程设置 `INOVANCE_FTP_PASSWORD`（不写入源码/数据库/日志），然后：

```powershell
python scripts/run_kinematics_identification.py collect --host <控制器IP> --robot-label <机器人标识> --profiles 0,0,0 1,1,0 --training-count 240 --rate 5 --output output/kinematics-identification/<新轮次>
python scripts/run_kinematics_identification.py fit --database output/kinematics-identification/<新轮次>/experiment.sqlite
python scripts/run_kinematics_identification.py validate-controller --host <同一控制器IP> --database output/kinematics-identification/<新轮次>/experiment.sqlite
python scripts/run_kinematics_identification.py validate-offsets --host <同一控制器IP> --database output/kinematics-identification/<新轮次>/experiment.sqlite
```

每组 profile 表示工具号、工件号、负载号。失败的轮次保留，重新采样必须使用新目录，不覆盖旧证据。
`fit` 不访问网络。`validate-controller` 将本地逆解结果传入控制器**正解计算接口**核对，不执行该关节位置。
`validate-offsets` 使用未参与拟合的偏移 TCP 目标（位置+0.35/-0.25/+0.20 mm，A/B/C+0.015/-0.012/+0.018 deg），
分别比较本地逆解和控制器逆解的独立闭环。原始FK样本逆解后若舍入回同一组关节角，零误差不意味着超高实体精度。

仅刚性链无法拟合时，可在**新轮次**使用 `fit --method gravity --database ...`。
这会增加由关节轴、下游连杆和重力方向构成的平滑柔顺项；不是查表插值或任意 XYZ 偏移。
当前仅支持安装重力角为零的实验上下文；其他安装方向需另行分析，不能套用本次模型。
模型形式改变后需使用新随机种子重新采样，再作独立验证。旧失败报告不覆盖、不删除。

## 数据与优化

1. 预先固定随机种子与数据分区：每条件 240 训练、80 随机独立测试、80 区域留出、41 连续轨迹、5 当前点重复样本。
   六轴覆盖关节限位内部；训练不覆盖的 J2/J5 高角度带仅用于区域验证。测试样本不会参与拟合。
2. 全程使用实际发送的三位小数关节角。FK 返回六位小数位姿；IK 输入按现场官方 SDK 的三位小数格式序列化，
   记录舍入后的目标。不把通信小数位误差算成唯一物理误差。
3. 以现有 DH 模型为基线，增加基座及六个固定关节段的等效刚性变换，采用带阻尼/正则化的最小二乘。
   位置残差单位 mm，姿态残差采用旋转矩阵的 SO(3) 对数，500 mm 权重换算，不直接相减欧拉角。
4. 42 个参数存在规范自由度，只作为可计算的等效几何；用最小范数、正则化和修正范围限制稳定优化，
   报告雅可比数值秩，不声称每个参数就是可独立辨识的厂家出厂参数。不能用更多样本补救错误的模型结构。
5. 每种工具/负载条件独立拟合，不将负载影响混为所有条件共享的本体参数。
6. 逆解使用与候选正解完全相同的链，只验证近邻初值分支，不宣称全局多解、奇异点、翻腕、碰撞或安全轨迹已通过。
7. 柔顺模型的基函数是关节轴上重力力矩对下游连杆质量/一阶矩的线性项，训练集 SVD 剔除相关方向；
   再联合辨识等效几何和关节小角度变形。系数不是可直接写回机器人的质量、重心或刚度。
   本地逆解会重新计算同一柔顺正解及其数值雅可比，不允许正解补偿、逆解仍用原模型。

## 验收与后续

- 在所有独立分区分别报告法兰/TCP 的平均、RMS、P95、最大位置误差及姿态误差。
- 暂定**实验数值目标**为最大 0.05 mm / 0.01 deg，采样前固化，不根据结果放宽；不是生产精度标准。
- 本地 FK→IK→FK 自洽不能代替独立验证，因此还将本地求得的关节角交给控制器 FK，比较同一工具条件下的目标 TCP。
- 即使达标也不自动部署。需要新的独立数据、上下文变化失效检查、限位/分支/连续性验证和适配层集成审查。
- 若刚性等效链不能达标，明确保留失败结论；再分析柔顺/重力等非几何误差，不给整片工作空间加单点固定偏移。

实验结果：见每轮目录中的 `report.md` / `report.json`，以 `experiment.sqlite` 中原始证据为准。
