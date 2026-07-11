# 运行目录与旧数据迁移

更新时间：2026-07-11

程序启动时固定三个互不混用的路径真源：

- `InstallRoot`：只读程序资源根，包含 `SDK/`、`Tools/`、`branding/`、`translations/`。可写 data root 和启动目录不能覆盖这些资源。
- `DataRoot`：现场可写根，统一包含 `Data/`、`Result/`、`Log/`、`Temp/`、`Job/`。
- `OriginalWorkingDirectory`：仅用于解析 CLI 用户显式传入的相对输入/输出；内部配置和产物不再跟随启动目录漂移。

## 选择 data root

优先级固定为：

1. `--data-root <DIR>` 或 `--data-root=<DIR>`；
2. 环境变量 `QTWIDGETSAPP4_DATA_ROOT`；
3. 安装根（保持既有现场默认数据位置兼容）。

示例：

```powershell
QtWidgetsApplication4.exe --data-root "D:\NoTeachingRobotData"
$env:QTWIDGETSAPP4_DATA_ROOT = "D:\NoTeachingRobotData"
QtWidgetsApplication4.exe
```

只读安装目录不会静默改用按品牌分叉的 LocalAppData，避免升级后读到另一份空数据库。此部署方式必须显式指定一个可写 data root。初始化会逐一写探针验证五个运行目录；失败时 CLI 写 stderr 并返回 `2`，GUI 同时显示错误框。

`--print-app-paths-json` 会在构造主窗口、打开数据库和连接硬件前输出诊断结果并退出：

```powershell
QtWidgetsApplication4.exe --print-app-paths-json --data-root "D:\NoTeachingRobotData"
```

它用于确认安装根、数据根、启动 cwd、数据库路径、资源根和内部路径越界门禁。点云子进程及升级重启会继承同一个 `QTWIDGETSAPP4_DATA_ROOT`。

## 迁移旧 Data

迁移默认不覆盖已有 `ConfigStore.db`；升级旧 schema 时会先备份并在事务中迁移。把旧安装目录的数据迁到新的 data root：

```powershell
tools\ConfigMigrate_Run.cmd `
  --data-root "D:\NoTeachingRobotData" `
  --source "C:\旧版本\NoTeaching-Robot\Data"
```

自动化调用追加 `--no-pause`。脚本会打印最终 source、data root 和数据库绝对路径，目标固定为 `<DataRoot>\Data\ConfigStore.db`。

如果新版程序已经先创建了 v4 数据库，迁移器会明确失败且保持数据库原样，禁止把“只补默认值”假报成旧配置已导入。确认要用旧 `Data` 替换目标库时显式追加 `--overwrite`；工具会先生成带时间戳的 `.bak_overwrite_*` 备份，再重建目标库。

发版打包前会由当前 `migrate_config_to_sqlite.py` 重建 `ConfigMigrate.exe`，并比较可执行文件内嵌源码 SHA256；工具缺失或源码不一致时打包和安装器编译都会硬失败。

## 不变量

- 中性版和品牌版只要指定同一 data root，就读取同一份现场数据；品牌显示名不能改变存储身份。
- 包内资源只从可信安装根读取；data root、启动 cwd 和仓库祖先目录不能注入 SDK、FANUC 程序或 branding。
- 内部相对路径拒绝 `..` 越界、绝对路径、UNC、`C:foo` 盘符相对路径和 NTFS ADS。
- OTA 清单、FTP 列表和远程 zip 中的不可信名称必须是安全的单一路径组件；路径分隔符、保留设备名、符号链接和归档越界项均失败关闭。
- `Data/...` 在配置库中仍是逻辑键，不因物理 data root 改变而改名。
