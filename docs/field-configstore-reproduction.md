# 现场 ConfigStore 调试复现

该接口用于把现场 `ConfigStore.db` 安全转移到开发机。现场数据库里的
DPAPI 字段只能由原 Windows 用户解密，所以不能直接复制数据库。

接口采用“开发机公钥加密、开发机私钥解密”的方式：

- 私钥只生成并保存在开发机，文件本身再由开发机当前用户 DPAPI 保护。
- 现场只拿到公钥。现场导出时，敏感字段只在内存中解密，磁盘上只生成
  公钥加密的 `.debugdb` 包，不生成明文数据库。
- 开发机导入后，所有敏感字段重新使用开发机当前用户 DPAPI 保护。
- 源数据库不会被修改；导入目标必须不存在，接口不会覆盖数据库。

## 1. 开发机生成一次接收密钥

先建立一个仅用于现场复现的目录，然后执行：

```powershell
.\tools\ConfigMigrate.exe `
  --create-debug-transfer-key `
  --debug-public-key "E:\FieldRepro\config-debug.public.pem" `
  --debug-private-key "E:\FieldRepro\config-debug.private.dpapi"
```

只把 `config-debug.public.pem` 和同版本的 `ConfigMigrate.exe` 发到现场。
`config-debug.private.dpapi` 不能离开开发机，也不能换 Windows 用户使用。

## 2. 现场导出

关闭现场程序，确认 `ConfigStore.db-wal`、`ConfigStore.db-shm` 和
`ConfigStore.db-journal` 均不存在。必须使用原来运行软件、能够读取该
数据库密码的 Windows 用户执行：

```powershell
.\ConfigMigrate.exe `
  --export-debug-database "D:\FieldExport\RobotC.debugdb" `
  --db "D:\SoftWare\HK-Pathlynx-CORPLA\Data\ConfigStore.db" `
  --debug-public-key "D:\FieldExport\config-debug.public.pem"
```

把生成的 `RobotC.debugdb` 发回开发机。它包含完整现场配置，虽然已经
端到端加密，仍应按敏感文件管理。

如果导出提示某个现场 DPAPI 配置无法解密，说明执行命令的 Windows
用户不是数据库敏感字段的原绑定用户；工具会停止且不会生成可用包。

## 3. 开发机导入到隔离环境

创建全新的复现根目录及其 `Data` 子目录，目标 `ConfigStore.db` 必须
不存在：

```powershell
New-Item -ItemType Directory -Path "E:\FieldRepro\RobotC\Data"

.\tools\ConfigMigrate.exe `
  --import-debug-database "E:\FieldRepro\RobotC.debugdb" `
  --db "E:\FieldRepro\RobotC\Data\ConfigStore.db" `
  --debug-private-key "E:\FieldRepro\config-debug.private.dpapi"
```

随后让程序只使用这个隔离目录：

```powershell
.\x64\HK-Pathlynx-CORPLA.exe --data-root "E:\FieldRepro\RobotC"
```

若现场库包含已知的 auth3 登录偏好类型异常，导入会原样保留该异常，
以便复现“自动修复”路径；修复完成后再在同一隔离目录中跑现场数据。
