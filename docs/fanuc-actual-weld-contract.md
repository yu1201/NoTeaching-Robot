# FANUC 实际焊接契约与解锁门禁

## 当前状态

FANUC 多点 TP 当前只支持空跑。选择“实际焊接”时，程序必须在第一条机器人运动前失败关闭，不能把普通 `L/J` 轨迹误报成已起弧焊接。

原因是仓库目前没有绑定以下现场真源：

- 与目标控制器及焊机一致、由示教器导出的 ArcTool LS 模板；
- 起弧、正常焊接、拐点过渡、收弧 schedule 编号及其电流/电压对应关系；
- 摆动和跟踪 schedule 的启用与结束语义；
- 程序异常停止后“电弧已经关闭”的可回读硬件见证。

仓库中的 `SDK/FANUC/WeldProgram.kl`、`WeldTriangleWeave.kl` 和 `WeldLWeave.kl` 是明确的占位文件，不构成焊接能力。对应的三个 `SendWeld*` 接口也必须拒绝下发。

## 当前安全契约

1. `FANUCRobotCtrl::UploadMultiPointTpProgram` 要求调用方显式选择 `DryRun` 或 `ActualWeld`，没有隐式默认值。
2. `ActualWeld` 在缺少已验证 ArcTool 契约时返回失败，不创建、不编译、不上传 TP。
3. `DryRun` 如果携带起弧、收弧、摆动、跟踪或其他焊接元数据，也必须失败；禁止静默丢字段后继续运动。
4. 旧 `ContiMoveAny` KL/VAR 路径同样只允许纯运动；发现数值工艺参数或焊接标志时必须在创建文件前拒绝，不能成为旁路入口。
5. 生产执行入口在第一条安全位运动前重复检查该契约，驱动入口再做一次纵深防御。
6. 空跑 LS 不包含 `/APPL ARC`、`Arc Start`、`Arc End`、`WELD_SPEED`、摆动或跟踪指令。
7. 搭接台阶等 `dOverlapRel <= 0` 的点在 FANUC 空跑 TP 中使用 `FINE`，普通连续点继续使用既有 `CNT50`；不能把 STEP 的 `OVERLAPREL` 数值直接当作 FANUC CNT 等级。

## 解锁实际焊接前必须补齐

现场必须提供同一控制器、同一 ArcTool 版本和同一焊机配置导出的最小可用 LS，至少覆盖：

- `/APPL` 与焊接设备号；
- 起弧运动及 start schedule；
- 正常焊接速度/工艺 schedule；
- 可选过渡、摆动和跟踪 schedule；
- 收弧运动及 crater/end schedule；
- 异常停止后的灭弧状态或焊接输出回读。

实现时需要新增 FANUC 专用工艺身份并纳入执行指纹，不能复用 STEP 的 `nArcMode`，也不能把软件中的电流/电压数值猜测映射到任意 schedule。

## 验收顺序

1. 纯生成器测试：空跑零焊接指令；实焊恰好一次起弧和一次收弧；完成寄存器只能在收弧之后写入。
2. 使用目标 ArcTool/WeldPRO 控制器配置运行 `maketp`，并对 TP 反汇编回读，不能只依赖 HandlingPRO 编译返回码。
3. 在 ROBOGUIDE 中验证 schedule、摆动、跟踪、异常停止和完成见证。
4. 真机先做断焊机空跑，再做低能量短焊；分别注入软件停止、网络断开、程序报警和控制器暂停，验证灭弧时间及闭锁。
5. 未通过上述真机故障测试前，不得把 FANUC 标记为实际焊接能力，也不得发布会自动触发 FANUC 实焊的流程。
