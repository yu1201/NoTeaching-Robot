#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OTA 服务器管理接口（root 运行，systemd 常驻）：服务器统计 + FTP 账号增删/改密码/改权限。
只监听 127.0.0.1:8091，经 nginx 8090 的 /admin/ 反代对外（不开新端口）；
所有请求须带头 X-Admin-Token（令牌在 /opt/ota-admin/token，600 权限）。

账号操作完全复刻线上 vsftpd 约定：
  useradd -M -d /srv/devicedata -s /usr/sbin/nologin -G ftpdata <name>   （chroot 共享目录，绝不 userdel -r）
  /etc/vsftpd.userlist 白名单增删行
  /etc/vsftpd_user_conf/<name> 写 download_enable=NO ⇒ 仅上传；删除该文件 ⇒ 全权限
受保护账号（禁止删除）：uploader（随安装包分发，删了全现场断传）、devicedata（管理员账号）。
"""
import hmac
import json
import os
import pwd
import re
import shutil
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LISTEN = ("127.0.0.1", 8091)
TOKEN_FILE = "/opt/ota-admin/token"
DATA_DIR = "/srv/devicedata/data"
USERLIST = "/etc/vsftpd.userlist"
USER_CONF_DIR = "/etc/vsftpd_user_conf"
FTP_GROUP = "ftpdata"
FTP_HOME = "/srv/devicedata"
PROTECTED = {"uploader", "devicedata"}          # 禁止删除
NAME_RE = re.compile(r"^[a-z][a-z0-9_-]{2,31}$")  # 账号名：小写字母开头，3-32 位

with open(TOKEN_FILE, "r", encoding="utf-8") as f:
    ADMIN_TOKEN = f.read().strip()

_stats_cache = {"at": 0.0, "data": None}


def run(cmd, input_text=None):
    r = subprocess.run(cmd, input=input_text, capture_output=True, text=True, timeout=30)
    if r.returncode != 0:
        raise RuntimeError((r.stderr or r.stdout or "命令失败").strip()[:300])
    return r.stdout


def list_accounts():
    names = []
    if os.path.exists(USERLIST):
        with open(USERLIST, "r", encoding="utf-8") as f:
            names = [ln.strip() for ln in f if ln.strip()]
    out = []
    for n in names:
        conf = os.path.join(USER_CONF_DIR, n)
        upload_only = False
        if os.path.exists(conf):
            with open(conf, "r", encoding="utf-8") as f:
                upload_only = "download_enable=NO" in f.read()
        out.append({"name": n, "permission": "upload" if upload_only else "full",
                    "protected": n in PROTECTED})
    return out


def write_userlist(names):
    with open(USERLIST, "w", encoding="utf-8") as f:
        f.write("\n".join(names) + "\n")


def set_permission(name, permission):
    conf = os.path.join(USER_CONF_DIR, name)
    if permission == "upload":
        os.makedirs(USER_CONF_DIR, exist_ok=True)
        with open(conf, "w", encoding="utf-8") as f:
            f.write("download_enable=NO\n")
    else:
        if os.path.exists(conf):
            os.remove(conf)


def account_create(name, password, permission):
    if not NAME_RE.match(name):
        raise ValueError("账号名需小写字母开头、3-32位（小写字母/数字/_-）")
    if len(password) < 8:
        raise ValueError("密码至少 8 位")
    if permission not in ("upload", "full"):
        raise ValueError("permission 须为 upload 或 full")
    names = [a["name"] for a in list_accounts()]
    if name in names:
        raise ValueError("账号已存在")
    try:
        pwd.getpwnam(name)
        exists_sys = True
    except KeyError:
        exists_sys = False
    if not exists_sys:
        # -M 不建家目录（chroot 共享 /srv/devicedata 已存在，root 属主不可写）
        run(["useradd", "-M", "-d", FTP_HOME, "-s", "/usr/sbin/nologin", "-G", FTP_GROUP, name])
    run(["chpasswd"], input_text=f"{name}:{password}\n")
    set_permission(name, permission)
    write_userlist(names + [name])


def account_update(name, password=None, permission=None):
    names = [a["name"] for a in list_accounts()]
    if name not in names:
        raise ValueError("账号不存在")
    if password:
        if len(password) < 8:
            raise ValueError("密码至少 8 位")
        run(["chpasswd"], input_text=f"{name}:{password}\n")
    if permission:
        if permission not in ("upload", "full"):
            raise ValueError("permission 须为 upload 或 full")
        set_permission(name, permission)


def account_delete(name):
    if name in PROTECTED:
        raise ValueError("受保护账号，禁止删除")
    names = [a["name"] for a in list_accounts()]
    if name not in names:
        raise ValueError("账号不存在")
    # 绝不 userdel -r：家目录是共享 chroot /srv/devicedata；数据文件保留（属主变孤儿 uid 无碍）
    run(["userdel", name])
    write_userlist([n for n in names if n != name])
    conf = os.path.join(USER_CONF_DIR, name)
    if os.path.exists(conf):
        os.remove(conf)


def build_stats():
    now = time.time()
    if _stats_cache["data"] is not None and now - _stats_cache["at"] < 30:
        return _stats_cache["data"]
    disk = shutil.disk_usage(DATA_DIR)
    devices = []
    total = 0
    if os.path.isdir(DATA_DIR):
        for entry in sorted(os.listdir(DATA_DIR)):
            full = os.path.join(DATA_DIR, entry)
            if not os.path.isdir(full):
                continue
            size = 0
            files = 0
            newest = 0.0
            for root, _dirs, fs in os.walk(full):
                for fn in fs:
                    try:
                        st = os.stat(os.path.join(root, fn))
                        size += st.st_size
                        files += 1
                        newest = max(newest, st.st_mtime)
                    except OSError:
                        pass
            total += size
            devices.append({"name": entry, "bytes": size, "files": files,
                            "lastUploadEpoch": int(newest)})
    data = {"disk": {"totalBytes": disk.total, "usedBytes": disk.used, "freeBytes": disk.free},
            "dataBytes": total, "devices": devices, "serverTimeEpoch": int(now)}
    _stats_cache["at"] = now
    _stats_cache["data"] = data
    return data


class Handler(BaseHTTPRequestHandler):
    server_version = "OtaAdmin/1"

    def _reply(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _auth(self):
        token = self.headers.get("X-Admin-Token", "")
        if not hmac.compare_digest(token, ADMIN_TOKEN):
            self._reply(401, {"ok": False, "error": "令牌无效"})
            return False
        return True

    def _body_json(self):
        length = min(int(self.headers.get("Content-Length", "0") or 0), 65536)
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self):
        if not self._auth():
            return
        if self.path == "/admin/api/ping":
            self._reply(200, {"ok": True})
        elif self.path == "/admin/api/stats":
            self._reply(200, {"ok": True, **build_stats()})
        elif self.path == "/admin/api/accounts":
            self._reply(200, {"ok": True, "accounts": list_accounts()})
        else:
            self._reply(404, {"ok": False, "error": "未知接口"})

    def do_POST(self):
        if not self._auth():
            return
        if self.path == "/admin/api/accounts":
            try:
                b = self._body_json()
                account_create(str(b.get("name", "")).strip(), str(b.get("password", "")),
                               str(b.get("permission", "upload")))
                self._reply(200, {"ok": True})
            except Exception as e:
                self._reply(400, {"ok": False, "error": str(e)})
        else:
            self._reply(404, {"ok": False, "error": "未知接口"})

    def do_PATCH(self):
        if not self._auth():
            return
        m = re.match(r"^/admin/api/accounts/([^/]+)$", self.path)
        if not m:
            self._reply(404, {"ok": False, "error": "未知接口"})
            return
        try:
            b = self._body_json()
            account_update(m.group(1), password=b.get("password") or None,
                           permission=b.get("permission") or None)
            self._reply(200, {"ok": True})
        except Exception as e:
            self._reply(400, {"ok": False, "error": str(e)})

    def do_DELETE(self):
        if not self._auth():
            return
        m = re.match(r"^/admin/api/accounts/([^/]+)$", self.path)
        if not m:
            self._reply(404, {"ok": False, "error": "未知接口"})
            return
        try:
            account_delete(m.group(1))
            self._reply(200, {"ok": True})
        except Exception as e:
            self._reply(400, {"ok": False, "error": str(e)})

    def log_message(self, fmt, *args):
        print("[%s] %s" % (self.address_string(), fmt % args), flush=True)


if __name__ == "__main__":
    ThreadingHTTPServer(LISTEN, Handler).serve_forever()
