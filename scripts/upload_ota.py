#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把安装包 / 增量补丁 / latest.json 上传到自建 OTA 服务器（默认 103.217.203.52:48890）。
自动维护每通道「非 exe 载荷哈希」历史（服务器上 payload_history.json），据此算出补丁的
baseMinVersion —— 增量补丁只含主 exe，只有设备当前版本 >= baseMinVersion（其非 exe 载荷/DLL
与目标一致）时增量才安全；更老的设备由客户端自动回退全量安装。

密码从环境变量 OTA_SSH_PASSWORD 读，**不写死在脚本里**（本仓库公开）。

两种用法：
  # 1) 只算某通道 dist 的载荷哈希（构建后立刻算，因为 dist 会被下一通道构建覆盖）
  python scripts/upload_ota.py --compute-only --dist-dir dist/QtWidgetsApplication4 --exe HK-Pathlynx-CORPLA.exe

  # 2) 上传某通道（用上一步算好的载荷哈希）
  set OTA_SSH_PASSWORD=****   （PowerShell: $env:OTA_SSH_PASSWORD='****'）
  python scripts/upload_ota.py --version 2026.07.09.1006 --channel brand ^
      --installer dist/installer/HK-Pathlynx-CORPLA-Setup-v2026.07.09.1006.exe ^
      --patch %TEMP%/HK-Pathlynx-CORPLA-Patch-v2026.07.09.1006.zip ^
      --payload-hash <上一步输出> --notes "xxx"
  # 中性通道去掉 --patch（惯例只发全量）。
"""
import argparse, hashlib, json, os, posixpath, sys, time

# 载荷哈希排除项：主 exe（被补丁替换）、构建元数据（每次都变）、运行目录（现场拥有、非分发内容）。
PAYLOAD_EXCLUDE_FILES = {"build_version.txt", "deploy_notes.txt"}
PAYLOAD_EXCLUDE_TOPDIRS = {"data", "log", "result", "temp"}


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().lower()


def payload_hash(dist_dir, main_exe_name):
    """dist 里除主 exe / 构建元数据 / 运行目录外，所有文件的合并哈希（相对路径+文件哈希，排序合并）。"""
    items = []
    for root, _dirs, files in os.walk(dist_dir):
        for fn in files:
            full = os.path.join(root, fn)
            rel = os.path.relpath(full, dist_dir).replace("\\", "/")
            top = rel.split("/", 1)[0].lower()
            if rel.lower() == main_exe_name.lower():
                continue
            if rel.lower() in PAYLOAD_EXCLUDE_FILES:
                continue
            if top in PAYLOAD_EXCLUDE_TOPDIRS:
                continue
            items.append((rel, sha256_file(full)))
    items.sort()
    h = hashlib.sha256()
    for rel, fh in items:
        h.update((rel + ":" + fh + "\n").encode("utf-8"))
    return h.hexdigest().lower(), len(items)


def ver_key(v):
    return [int(x) for x in v.split(".") if x.isdigit()]


def compute_base_min_version(history, cur_ver, cur_hash):
    """history: {版本: 载荷哈希}。返回「载荷哈希与当前一致、且以当前版结尾的最长连续区间」的最早版本。
    连续性按版本升序判定：从当前版往前，哈希一致就并入，一遇到不同就停。"""
    hist = dict(history)
    hist[cur_ver] = cur_hash
    vers = sorted(hist.keys(), key=ver_key)
    idx = vers.index(cur_ver)
    base = cur_ver
    for i in range(idx, -1, -1):
        if hist[vers[i]] == cur_hash:
            base = vers[i]
        else:
            break
    return base, hist


def make_ssh(host, port, user, pw):
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(host, port=port, username=user, password=pw, timeout=30)
    return c


def sftp_get_json(sftp, remote_path):
    try:
        with sftp.open(remote_path, "r") as f:
            return json.loads(f.read().decode("utf-8"))
    except IOError:
        return {}


def put_retry(host, port, user, pw, local, remote, tries=4):
    for i in range(tries):
        c = None
        try:
            c = make_ssh(host, port, user, pw)
            s = c.open_sftp()
            s.put(local, remote)
            s.close()
            c.close()
            return True
        except Exception as e:
            print(f"  put {posixpath.basename(remote)} 失败{i+1}: {str(e)[:60]}", flush=True)
            try:
                if c:
                    c.close()
            except Exception:
                pass
            time.sleep(4)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--compute-only", action="store_true", help="只算 dist 载荷哈希并打印，不上传")
    ap.add_argument("--dist-dir", default="dist/QtWidgetsApplication4")
    ap.add_argument("--exe", default="", help="主程序 exe 名（compute-only 必填）")
    ap.add_argument("--version")
    ap.add_argument("--channel", choices=["brand", "neutral"])
    ap.add_argument("--installer")
    ap.add_argument("--patch", default="")
    ap.add_argument("--payload-hash", default="")
    ap.add_argument("--seed-versions", default="",
                    help="逗号分隔的历史版本；这些版本已知与本版非 exe 载荷一致时回填进历史，"
                         "使 baseMinVersion 能回溯到它们（仅在你确认这些版本 DLL 与本版相同时用）。")
    ap.add_argument("--notes", default="")
    ap.add_argument("--server", default="103.217.203.52")
    ap.add_argument("--port", type=int, default=48890)
    ap.add_argument("--user", default="root")
    a = ap.parse_args()

    if a.compute_only:
        if not a.exe:
            sys.exit("compute-only 需要 --exe")
        h, n = payload_hash(a.dist_dir, a.exe)
        print(h)
        print(f"(载荷文件数={n})", file=sys.stderr)
        return

    for req in ("version", "channel", "installer", "payload_hash"):
        if not getattr(a, req.replace("-", "_")):
            sys.exit(f"上传缺参数 --{req.replace('_','-')}")
    if not os.path.exists(a.installer):
        sys.exit("找不到安装包: " + a.installer)
    if a.patch and not os.path.exists(a.patch):
        sys.exit("找不到补丁: " + a.patch)
    pw = os.environ.get("OTA_SSH_PASSWORD")
    if not pw:
        sys.exit("请先设置环境变量 OTA_SSH_PASSWORD（服务器 root 密码），脚本不写死密码。")

    remote_dir = "/var/www/ota/" + a.channel
    hist_remote = remote_dir + "/payload_history.json"

    # 取服务器已有历史，算 baseMinVersion
    c = make_ssh(a.server, a.port, a.user, pw)
    sftp = c.open_sftp()
    history = sftp_get_json(sftp, hist_remote)
    sftp.close(); c.close()
    # 回填已知同载荷的历史版本（DLL 与本版一致时才用），让 baseMinVersion 能回溯到它们。
    for sv in [s.strip() for s in a.seed_versions.split(",") if s.strip()]:
        history.setdefault(sv, a.payload_hash)
    base_min, new_hist = compute_base_min_version(history, a.version, a.payload_hash)
    print(f"通道={a.channel} 版本={a.version} 载荷哈希={a.payload_hash[:16]}… baseMinVersion={base_min}", flush=True)

    # 组 latest.json
    manifest = {
        "version": a.version,
        "file": os.path.basename(a.installer),
        "sha256": sha256_file(a.installer),
        "size": os.path.getsize(a.installer),
        "notes": a.notes,
    }
    if a.patch:
        manifest["patch"] = {
            "file": os.path.basename(a.patch),
            "sha256": sha256_file(a.patch),
            "size": os.path.getsize(a.patch),
            "baseMinVersion": base_min,
        }

    tmp = os.environ.get("TEMP", ".")
    manifest_path = os.path.join(tmp, f"latest_{a.channel}.json")
    with open(manifest_path, "w", encoding="utf-8") as f:
        f.write(json.dumps(manifest, ensure_ascii=False))
    hist_path = os.path.join(tmp, f"payload_history_{a.channel}.json")
    with open(hist_path, "w", encoding="utf-8") as f:
        f.write(json.dumps(new_hist, ensure_ascii=False, indent=0))

    # 上传顺序：先安装包/补丁/历史，最后 latest.json（客户端看到新版时文件已就位）
    ok = put_retry(a.server, a.port, a.user, pw, a.installer, remote_dir + "/" + os.path.basename(a.installer))
    if ok and a.patch:
        ok = put_retry(a.server, a.port, a.user, pw, a.patch, remote_dir + "/" + os.path.basename(a.patch))
    if ok:
        ok = put_retry(a.server, a.port, a.user, pw, hist_path, hist_remote)
    if ok:
        ok = put_retry(a.server, a.port, a.user, pw, manifest_path, remote_dir + "/latest.json")
    print(f"{a.channel}: {'OK' if ok else '失败'}", flush=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
