#!/usr/bin/env bash
# 在线服务一键部署（Ubuntu 22.04/24.04，root 执行）：
#   bash deploy_online_services.sh <FTP密码>
# 部署内容：
#   1. nginx  :8090  → /var/www/ota/{neutral,brand}/   OTA 静态更新源（latest.json + 安装包）
#   2. vsftpd :21 + 被动口 40000-40100 → 用户 devicedata，chroot 在 /srv/devicedata
#      客户端上传路径 /data/<设备名>/… 落在 /srv/devicedata/data/<设备名>/…
#   3. ufw 防火墙只放行 22/8090/21/40000-40100
#   4. cron 每日清理 30 天前的上传数据（服务器只做中转，公司侧及时回拉）
# 域名 xiaomomoyun.cn 未备案期间走 IP/域名+8090 端口，备案后再迁 80/443。
set -euo pipefail

FTP_PASSWORD="${1:-}"
if [ -z "$FTP_PASSWORD" ]; then
    echo "用法: bash deploy_online_services.sh <FTP密码>"
    exit 1
fi

echo "=== 1/5 安装 nginx + vsftpd + ufw ==="
apt-get update -y
DEBIAN_FRONTEND=noninteractive apt-get install -y nginx vsftpd ufw

echo "=== 2/5 配置 OTA 静态站点 (:8090) ==="
mkdir -p /var/www/ota/neutral /var/www/ota/brand
cat > /etc/nginx/sites-available/ota <<'NGINX'
server {
    listen 8090;
    server_name _;
    # 客户端 UpdateBaseUrl 默认 http://<服务器>:8090/ota → URL 带 /ota 前缀，
    # root 取 /var/www 使 /ota/neutral/xx 落到 /var/www/ota/neutral/xx。
    # （8080 在 156.239.225.105 上被既有 qqbot 服务占用，统一用 8090。）
    root /var/www;
    autoindex off;
    # 只暴露两个通道下的清单、安装包与增量补丁，其余一律 404
    location ~ ^/ota/(neutral|brand)/latest\.json$ {
        add_header Cache-Control "no-cache";
    }
    location ~ ^/ota/(neutral|brand)/.*\.(exe|zip)$ {
    }
    location / {
        return 404;
    }
}
NGINX
ln -sf /etc/nginx/sites-available/ota /etc/nginx/sites-enabled/ota
rm -f /etc/nginx/sites-enabled/default
nginx -t
systemctl enable --now nginx
systemctl reload nginx

echo "=== 3/5 配置 vsftpd（用户 devicedata，chroot /srv/devicedata） ==="
id devicedata &>/dev/null || useradd -d /srv/devicedata -s /usr/sbin/nologin devicedata
echo "devicedata:${FTP_PASSWORD}" | chpasswd
# chroot 根目录必须 root 属主且不可写，数据写到子目录 data/
mkdir -p /srv/devicedata/data
chown root:root /srv/devicedata
chmod 755 /srv/devicedata
chown -R devicedata:devicedata /srv/devicedata/data
# nologin 需在 shells 白名单里 vsftpd 才放行登录
grep -qx '/usr/sbin/nologin' /etc/shells || echo '/usr/sbin/nologin' >> /etc/shells
cat > /etc/vsftpd.conf <<'VSFTPD'
listen=YES
listen_ipv6=NO
anonymous_enable=NO
local_enable=YES
write_enable=YES
local_umask=022
chroot_local_user=YES
allow_writeable_chroot=NO
pasv_enable=YES
pasv_min_port=40000
pasv_max_port=40100
utf8_filesystem=YES
xferlog_enable=YES
xferlog_file=/var/log/vsftpd.log
idle_session_timeout=300
data_connection_timeout=120
userlist_enable=YES
userlist_file=/etc/vsftpd.userlist
userlist_deny=NO
VSFTPD
echo "devicedata" > /etc/vsftpd.userlist
systemctl enable --now vsftpd
systemctl restart vsftpd

echo "=== 4/5 防火墙 ==="
ufw allow 22/tcp
ufw allow 8090/tcp
ufw allow 21/tcp
ufw allow 40000:40100/tcp
ufw --force enable

echo "=== 5/5 上传数据 30 天自动清理 ==="
cat > /etc/cron.daily/clean-devicedata <<'CRON'
#!/bin/sh
find /srv/devicedata/data -type f -mtime +30 -delete
find /srv/devicedata/data -mindepth 1 -type d -empty -delete
CRON
chmod +x /etc/cron.daily/clean-devicedata

echo ""
echo "部署完成 ✓"
echo "OTA 源:   http://$(hostname -I | awk '{print $1}'):8090/ota 路径为 /neutral 与 /brand（客户端配置 UpdateBaseUrl=http://xiaomomoyun.cn:8090）"
echo "          发布用 scripts/upload_release.ps1 推送 latest.json + 安装包到 /var/www/ota/<channel>/"
echo "FTP:      主机 xiaomomoyun.cn 端口 21 用户 devicedata（客户端「在线服务」页填写）"
echo "数据落盘: /srv/devicedata/data/<设备名>/*.zip（30 天自动清理）"
