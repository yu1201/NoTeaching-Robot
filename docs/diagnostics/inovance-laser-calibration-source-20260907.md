# 汇川激光标定运行参数来源（2026-09-07）

现场示教器路径：弧焊工艺 → 激光器配置 → 激光传感器0 → 标定结果。

控制器固定只读来源：

`/RCFamily/Controller/Teachology/InoRobPluginWeld/config.xml`

结构：`WELD_LaserConfig/E00`，类型 `S_LASERCONFIG`。字段布局由同插件的 `typedef.h`
核对，`f64LaserCoord[6]` 位于展开值 115..120，单位为 mm/deg，姿态顺序为 A/B/C（Rz/Ry/Rx）。

现场读取值：

- 标定结果 XYZABC：`97.3294, 67.8006, -134.2267, 76.7707, -1.8085, 146.2318`
- 控制器报告误差 XYZABC：`0.0733, 0.1065, 0.1342, 0, 0, 0`
- 绑定工具：`Tool1`
- 相机地址：`192.168.39.5`
- 参考系：`camera -> bound Tool TCP`

五组控制器保存的机器人位姿与激光点按 `T_robot_tcp * T_camera_to_tool * p_camera`
闭环到同一基座点，最大残差约 `0.444 mm`。这能验证字段方向和参考系，不代表实体标定精度优于该数值。

禁止使用 `/Plugins/InoRobPluginWeld/config.xml`：该文件是出厂默认配置，不是当前运行标定结果。
解析器同时拒绝空/退化五点、字段数变化、非有限值和未确认合成顺序的非零补偿。
