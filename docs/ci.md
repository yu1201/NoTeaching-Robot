# CI 离线门禁

- 人工整理日期：`2026-07-11`

仓库的 GitHub Actions 只做验证，不执行部署、`git push`、OTA 上传或 GitHub Release。工作流文件为
`.github/workflows/offline-gates.yml`，在 `main`、`hk-pathlynx-corpla` 的 push、所有 pull request
以及手工触发时运行。

## 固定安全边界

- 权限只有 `contents: read`；checkout 不持久化 GitHub 凭据。
- 不使用 `pull_request_target`，不读取任何 GitHub secret。
- `OTA_SIGNING_KEY_FILE` 与 `OTA_SSH_PASSWORD` 在 job 中显式置空；工作流不调用
  `upload_ota.py publish-dual`。
- 不连接 OTA/FTP/机器人/相机等生产端点。`ota_admin.py` 测试只使用临时目录、假的系统账号数据库/
  命令执行器和内存 HTTP handler（不监听端口）。部署测试先验证默认 dry-run，再以
  `DEPLOY_OFFLINE_TEST=1` 和临时 `DEPLOY_ROOT` 演练隔离事务；不会修改 runner 的账号、服务或防火墙。
- Windows job 有 20 分钟、Ubuntu 在线服务 job 有 10 分钟硬超时；两者均不构建完整 Qt 工程，也不依赖
  本机 Visual Studio、Qt SDK 或机器人 SDK。
- checkout/setup-python 固定到完整 action commit；Python 与测试依赖固定版本。

## CI 覆盖范围

1. `test_upload_ota_offline.py`：双通道候选、签名、清单、原子上传顺序、回滚和远端假 SFTP。
2. `test_ota_admin_offline.py`：临时文件上的账号事务/回滚/并发、令牌文件权限、HTTP 鉴权与输入边界、
   通用错误脱敏和统计目录符号链接隔离；不会调用真实用户管理命令。
3. 关键静态门禁：OTA 客户端、AppPaths、机器人互锁、FANUC 实焊禁用、运动终态、点云质量、
   扫描周期、凭据治理和断点续焊绑定。
4. PowerShell clean-checkout smoke：所有发布脚本语法、版本/AppId、旧上传入口 fail-fast、清单同尺寸
   篡改、现场目录泄漏、旧 PASS 失效，以及缺少 FANUC 运行文件时必须失败。
5. `test_deploy_online_services_offline.sh`：Ubuntu 上先对部署器和测试自身执行 `bash -n`，再用临时
   根目录验证 dry-run 零写入、既有白名单保留、upload-only、危险配置冲突拒绝、SSH/UFW 计划、
   setgid/共享组约束和明文参数拒绝；隔离 apply 还验证安装结果及第二次执行幂等，生产路径保持未触碰。

`scripts/tests/test_release_packaging_gates.ps1` 的完整版本仍保留为本地发版门禁。它会两次构建
`ConfigMigrate.exe`，并需要被 `.gitignore` 排除、不会出现在 GitHub clean checkout 中的
`SDK/FANUC/*.tp` 与 `*.pc` 权威运行文件，因此不适合 GitHub-hosted runner。CI 运行上述
clean-checkout smoke；真正打包前仍必须在准备好的本地双 worktree 上运行完整 PowerShell 测试和
pair gate，不能用 CI smoke 代替。

## 本地复现

```powershell
python -B scripts/tests/test_upload_ota_offline.py
python -B scripts/tests/test_ota_admin_offline.py
python -B scripts/tests/verify_ota_release_gate.py
python -B scripts/tests/verify_app_paths.py --static-only
python -B scripts/tests/verify_robot_operation_interlock.py

# 需要本地 FANUC 运行文件和 PyInstaller；发版前执行。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/tests/test_release_packaging_gates.ps1

# 在 Bash/Ubuntu 环境复现部署脚本的 CI 门禁；apply 演练只写测试创建的临时根目录。
bash -n scripts/server/deploy_online_services.sh
bash -n scripts/tests/test_deploy_online_services_offline.sh
bash scripts/tests/test_deploy_online_services_offline.sh
```

仓库分支保护应把 `Offline gates (Windows)` 与 `Online service offline gates (Ubuntu)` 都设为必需检查；
这只提升合并门禁，不改变“本地构建、用户测试确认后才能对外发布”的发版授权边界。
