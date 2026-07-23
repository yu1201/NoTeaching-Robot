# 在线服务部署

- 人工整理日期：`2026-07-11`

`scripts/server/deploy_online_services.sh` 用于在 Ubuntu 22.04/24.04 重建 OTA 静态源、
FTP 数据中转、`ota-admin` 管理接口和清理任务。脚本默认只预演；不会因为未传参数而修改服务器。

## 部署前准备

部署器绝不运行 `apt`，所有系统依赖必须在独立维护窗口预装。安装/升级 vsftpd 前先从外部防火墙
隔离 21 与被动端口，或执行 `systemctl mask --runtime --now vsftpd.service`；预装完成后保持服务 inactive，
用 `systemctl unmask --runtime vsftpd.service` 只解除本轮 runtime mask，再运行部署器。脚本会在任何配置
写入前核对依赖，缺失时直接失败。

真实 apply 前把部署器和管理服务源码复制到 root-only 目录；不要让 root 直接执行普通用户可写 checkout
中的脚本：

```bash
install -d -m 0700 -o root -g root /root/no-teaching-deploy
install -m 0600 -o root -g root scripts/server/deploy_online_services.sh /root/no-teaching-deploy/
install -m 0600 -o root -g root scripts/server/ota_admin.py /root/no-teaching-deploy/
install -m 0600 -o root -g root scripts/server/provision_default_ftp_accounts.py /root/no-teaching-deploy/
```

为三级默认账号 `devicedata`、`ftpoperator`、`uploader` 分别准备一行密码文件。文件应由 root 持有，权限为 0600；
不要把密码放在命令行、仓库、日志或 CI secret 输出中。

```bash
install -d -m 0700 /root/no-teaching-secrets
install -m 0600 /dev/null /root/no-teaching-secrets/devicedata.password
install -m 0600 /dev/null /root/no-teaching-secrets/ftpoperator.password
install -m 0600 /dev/null /root/no-teaching-secrets/uploader.password
# 在服务器本机以不回显方式写入三份文件，每份仅一行。
```

先执行 dry-run，核对现有 nginx 站点数、FTP 白名单、SSH 端口和被动模式受信网回址：

```bash
/bin/bash scripts/server/deploy_online_services.sh \
  --ssh-port 48890 \
  --pasv-address "<VPN/受信网内可达 IP 或主机名>"
```

## 执行部署

真实 apply 使用绝对 Bash 路径和 privileged mode，避免继承启动脚本或 shell function：

```bash
sudo -H /bin/bash -p /root/no-teaching-deploy/deploy_online_services.sh --apply \
  --skip-packages \
  --ota-admin-source /root/no-teaching-deploy/ota_admin.py \
  --ssh-port 48890 \
  --pasv-address "<VPN/受信网内可达 IP 或主机名>" \
  --devicedata-password-file /root/no-teaching-secrets/devicedata.password
```

已有 `devicedata` 必须省略密码文件并保留原密码；为已有账号传密码文件会直接失败。
管理令牌在首次部署时由服务器本机生成到 `/opt/ota-admin/token`，以后部署保持原值。

部署与健康检查完成后，在服务器本机通过回环管理接口创建或轮换三级默认账号。辅助脚本只接受
root-only 秘密文件路径，密码内容只在内存中进入 `127.0.0.1` 请求，不写临时文件、不打印：

```bash
sudo -H python3 -I /root/no-teaching-deploy/provision_default_ftp_accounts.py \
  --devicedata-password-file /root/no-teaching-secrets/devicedata.password \
  --ftpoperator-password-file /root/no-teaching-secrets/ftpoperator.password \
  --uploader-password-file /root/no-teaching-secrets/uploader.password
```

UFW 默认不修改。只有明确需要脚本追加规则时才加 `--configure-ufw`，并必须通过
`--ftp-allow-cidr` 指定 VPN/受信私网；需要同时启用 UFW 时再加 `--enable-ufw`。脚本拒绝全网和普通
公网 CIDR，并会在发现既有 FTP `Anywhere`/更宽放行规则时停止，要求人工审计删除，不会自行 reset
或删规则。UFW 位于核心部署事务提交后，不属于文件回滚范围。使用 `--enable-ufw` 时，脚本还会确认
`--ssh-port` 存在监听且与当前 SSH 会话端口一致；无法证明不会锁死管理连接时拒绝自动启用。

例如只为 VPN 地址段追加 FTP 规则：

```bash
sudo -H /bin/bash -p /root/no-teaching-deploy/deploy_online_services.sh --apply \
  --skip-packages \
  --ota-admin-source /root/no-teaching-deploy/ota_admin.py \
  --configure-ufw \
  --ftp-allow-cidr 10.20.0.0/16 \
  --ssh-port 48890
```

## 固定权限模型

- `uploader`：写一次上传；禁止下载、chmod、删除、改名、追加和 REST 续写，保留建目录与首次 STOR。
  客户端每次尝试使用唯一远端文件名，新文件以 0440 落地，之后同一共享凭据不能再次打开改写。
- `ftpoperator`：FTP 上传、浏览和下载权限；客户端允许修改服务器 IP 与设备名称，但不展示服务器
  磁盘、设备统计和账号管理，也不开放删除数据或新建设备目录。
- `devicedata`：全权限；全局新文件模式为 0644，避免 `ftpdata` 组成员改写彼此文件。
- 三个固定账号属于 `ftpdata`；`ftpoperator` 与 `uploader` 额外加入 `devicedata` 数据访问组。
  共享根 `/srv/devicedata/data` 为 2771/setgid，`local_umask=002`。
- `/etc/vsftpd.userlist` 的既有受管账号与注释会保留；非法、重复、缺失系统身份、UID/GID 别名、
  隐藏组成员或弱权限配置都会使部署停止，不会静默继承。
- nginx 对外只提供双通道 `latest.json`、`.exe`、`.zip`；`/admin/` 反代仅允许 loopback，且不公开
  `payload_history.json`。

真实部署会审计存量普通文件；发现 uploader 自有文件仍可写，或任意文件有 group/other 可写位时直接
停止，要求先备份并执行受审权限迁移，不会在部署过程中静默递归修改用户数据。部署和管理接口还共用
root-only 跨进程锁，部署先等待在途账号事务完成，再停服务、重读白名单并持锁至健康检查结束。

## 验证与回滚

脚本在提交前验证 nginx、systemd、`ota-admin` 回环与 nginx 反代健康检查、vsftpd 220 banner；
失败时先在停服状态恢复托管文件、账号/组和目录权限，验证后最后恢复服务状态。成功后会显示
root-only 备份目录，但不会显示秘密内容。

仓库离线门禁：

```bash
bash -n scripts/server/deploy_online_services.sh
bash -n scripts/tests/test_deploy_online_services_offline.sh
bash scripts/tests/test_deploy_online_services_offline.sh
python -B scripts/tests/test_ota_admin_offline.py
```

离线门禁会在临时根目录连续执行两次隔离部署并比较托管指纹；不会调用宿主机账号、systemd、
UFW、`chpasswd` 或生产网络。正式上线仍需在目标 Ubuntu 上分别验证三种账号登录：
`uploader` 只上传、`ftpoperator` 上传下载、`devicedata` 全权限，以及 NAT 被动连接和重启后服务自启。

## 传输安全边界

当前 8090 是 HTTP，设备数据通道仍是传统 FTP。数字签名保护 OTA 清单与载荷完整性，但不会加密
设备数据。部署生成的 nginx 将 `/admin/` 限制为 loopback；客户端也会在读取令牌、设置请求头之前
拒绝非 HTTPS 的远端管理地址。远程管理必须经 HTTPS 反代或 SSH 可信隧道，不能用公网 HTTP。

发版脚本使用的 SFTP 发布账号、SSH 主机密钥信任和密钥/密码由服务器基础设施单独预置；本部署器
不会创建发布账号，也不会把发布秘密写入仓库或文档。当前 WinINet 数据上传仍是传统 FTP，账号密码
和扫描数据都没有传输加密；在迁移到 FTPS/SFTP 前，只允许放在 VPN 或受信专网内，禁止把 21/被动
端口直接暴露到不可信公网。共享 `uploader` 还需要独立数据分区或项目配额、剩余空间告警和保留空间
作为上线验收项；30 天清理只能做保留策略，不能替代容量上限。
