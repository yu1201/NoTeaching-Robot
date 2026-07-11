# 点云质量门禁 Profile v1

标定日期：2026-07-11。

## 数据与结论

- 对 `Result/**/SdkPointCloud/SchemeCompare` 中 101 组 `BaseWeldPointCloudFit` 历史产物进行了离线回算。
- 除旧的“所有关键段至少 15 mm”规则外，现有阈值通过 97/101；其余 4 组均是明显弱样本（点数、跨度、连续段或输出长度严重不足）。
- 连续性门限通过 100/101；残差门限通过 101/101，历史最大中位残差 0.615 mm、最大 P95 4.315 mm、最小 6 mm 内点率 99.0%，现门限 3/8 mm、75% 保留了较大裕量。
- 旧 15 mm 规则只通过 55/101，原因是合法的相邻 `geometry_lap_step` 关键点通常仅相距约 0.56–4.39 mm，不能作为统一硬门限。
- 另对 101 组 SDK `FeaturePoint` 直出轨迹单独回放，95/101 通过。6 组拒绝项分别是短轨迹、关键点/拐点不足或终点倒退；历史唯一 2.500 mm 短段紧邻起点，按端段规则通过并告警。

Profile v1 因此采用：

- 内部非搭接关键段 `<3 mm` 硬失败，`3–15 mm` 只告警；
- 成对搭接台阶关键段 `<0.25 mm` 硬失败；
- 紧邻唯一起点或终点的端段 `<0.25 mm` 硬失败，避免 SDK 不提供搭接标签时误杀合法裁剪端段；
- 起点、终点必须各恰好一个；
- 覆盖、连续性、剔除比例、残差、关键点、输出六类指标始终计算；
- 最终 SeamComp 还必须原子写入、回读，验证有限值、点数、连续索引、总长和最大点距。
- 最终 SeamComp 总长同时绑定本次实际点云跨度与补偿前姿态长度；先扣除工艺明确声明的 StartSkip/EndSkip，再要求至少保留预期轨迹的 93%，且不得膨胀到 125% 以上。该上限会拒绝历史中长度异常增至 1.68–2.93 倍的往返/错序产物，同时不会误拒绝短焊道上的合法端点裁剪。
- 最终点按 `raw_index` 在补偿前记录顺序上做单调对齐，允许重复 raw 和少量没有同 raw 的插值点，但要求匹配弧长至少 90%、声明裁剪后源记录唯一覆盖至少 55%、声明裁剪后源弧长跨度至少 90%；因此完整反序或只在少数 raw 点间往返不能再用重复行数稀释门禁。
- 最终点相对补偿前同源点的位移不超过 25 mm，控制器欧拉最短路差与按机器人旋转顺序计算的 SO(3) 物理姿态差均不超过 60°；相邻最终点的两种姿态差均不超过 90°。
- `point_type`、`segment_kind` 与搭接台阶标志只接受已知 token，并且拐点、段类型、台阶标志必须能在补偿前轨迹的 25 mm 邻域找到对应语义；补偿函数还会对内存生成的完整 UTF-8 字节计算 SHA/大小，最终结构验证必须命中同一快照，因此生成后只删除 `_arc`、`_transition`、corner 或 lap 标签也会失败。

SDK 直出模式不再用 `max(output,input)` 或固定 `rejected=0` 填充指标：完整点云的总数、有限点数和无效点数由隔离 worker 返回；连续性、残差、关键点与输出门禁评估实际将被执行的 SDK 轨迹，连续性分箱使用 `External/ResampleStepMm`。

## 迁移策略

旧现场 `ConfigStore.db` 可能持久化六个 `Validation/*Enabled=0`，安装和 OTA 又会保留 `Data`。因此 Profile v1 使用 `Validation/ProfileVersion` 与 `Validation/Policy`：

- 缺少 ProfileVersion 或版本旧于 1：运行时迁移到 `Enforce`，不再信任历史全关值；
- `Enforce`：质量失败直接终止分析，不生成可执行证明；
- `Audit`：完整计算和落盘，但质量证明不授权下发、STEP job、焊接或续焊；
- 没有 `Off` 发布状态，六类计算不可逐项关闭。
- `Enforce` 对 101 组语料标定出的安全阈值设置不可放宽的边界；需要更宽阈值时只能切换 `Audit`，而 `Audit` 永远不能进入执行。
- Profile、Policy 与全部阈值在 SQLite 单一事务中提交，并通过单条查询快照读取，避免中断或并发读写留下/观察到“新版本号 + 旧阈值”的混合配置。

每个生产案例在 `LaserPoint/PreciseLaserPoint_QualityGate.json` 保存算法修订 `pcq-v1-20260711-d`、策略修订哈希、阈值、指标、输入文件哈希、补偿前姿态哈希和最终 SeamComp 哈希。补偿前与最终轨迹分别使用一次 `readAll` 字节快照同时完成解析、大小和 SHA256 计算；proof 提交前还会回读并逐项比较这两个快照，任一文件在结构验证后被替换都会失败。执行时会重新比较 proof 中完整 `thresholds` 对象、处理模式和策略哈希，不能只改展示阈值而继续授权。跳过扫描、续焊、STEP 生成、下发及执行均重新验证已授权快照；实际运动前再次复核当前策略和证明。虚拟焊道没有通用跳过开关，只接受本进程刚生成、登记过路径/机器人/大小/SHA256 且写后验证为固定姿态 ±Y 直线的文件。

角度归一化不再使用逐圈 `while ±360`；所有姿态文件角度必须是有限且位于 ±3600° 的合理表示，并通过有界余数运算归一化，避免 `1e308` 一类有限大数使验证线程永久循环。

该 JSON 用于防止误改、陈旧文件和流程串线，不以抵御能同时改写程序数据与证明文件的本机恶意管理员为威胁模型；若后续需要对抗主动篡改，应引入外部可信签名根。

## 复跑

```powershell
python scripts/audit_pointcloud_quality.py `
  --result-root Result `
  --output-dir Temp/PointCloudQualityAudit
```

输出为固定字段的 `quality_audit.jsonl` 与 `quality_audit.csv`，不会修改历史案例。

SDK 直出轨迹回放：

```powershell
python scripts/audit_sdk_direct_quality.py `
  --result-root Result `
  --output Temp/SdkDirectQualityAudit.jsonl
```

该回放验证历史 FeaturePoint 的轨迹侧门禁；完整点云有限/无效计数由 SDK worker 在真实运行时验证。
