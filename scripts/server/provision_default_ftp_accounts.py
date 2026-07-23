#!/usr/bin/python3 -I
"""Provision or rotate the three protected FTP accounts through loopback ota-admin.

Only secret *file paths* are accepted on the command line. Secret contents remain
in memory and are never printed or written to a temporary file.
"""

import argparse
import json
import os
import stat
import urllib.error
import urllib.request


API_ROOT = "http://127.0.0.1:8091/admin/api"
EXPECTED = {
    "devicedata": "full",
    "ftpoperator": "full",
    "uploader": "upload",
}


def read_secret(path, label, minimum, maximum):
    info = os.lstat(path)
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise RuntimeError(f"{label} must be a regular non-symlink file")
    if hasattr(os, "geteuid") and info.st_uid != 0:
        raise RuntimeError(f"{label} must be owned by root")
    if stat.S_IMODE(info.st_mode) & 0o077:
        raise RuntimeError(f"{label} must not be readable by group or other users")
    with open(path, "r", encoding="utf-8") as stream:
        lines = stream.read().splitlines()
    if len(lines) != 1 or not minimum <= len(lines[0]) <= maximum:
        raise RuntimeError(f"{label} must contain exactly one valid-length line")
    value = lines[0]
    if ":" in value or any(ord(ch) < 0x20 or ord(ch) == 0x7F for ch in value):
        raise RuntimeError(f"{label} contains a forbidden character")
    return value


def request(token, method, path, body=None):
    payload = None if body is None else json.dumps(body, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        API_ROOT + path,
        data=payload,
        method=method,
        headers={
            "X-Admin-Token": token,
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=20) as response:
            raw = response.read(256 * 1024 + 1)
    except urllib.error.HTTPError as exc:
        raw = exc.read(256 * 1024 + 1)
        raise RuntimeError(f"ota-admin rejected {method} {path} with HTTP {exc.code}") from None
    if len(raw) > 256 * 1024:
        raise RuntimeError("ota-admin response exceeds the safety limit")
    document = json.loads(raw.decode("utf-8"))
    if not isinstance(document, dict) or document.get("ok") is not True:
        raise RuntimeError(f"ota-admin rejected {method} {path}")
    return document


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--token-file", default="/opt/ota-admin/token")
    parser.add_argument("--devicedata-password-file", required=True)
    parser.add_argument("--ftpoperator-password-file", required=True)
    parser.add_argument("--uploader-password-file", required=True)
    args = parser.parse_args()
    if hasattr(os, "geteuid") and os.geteuid() != 0:
        raise SystemExit("must run as root on the server")

    token = read_secret(args.token_file, "admin token", 32, 512)
    password_paths = {
        "devicedata": args.devicedata_password_file,
        "ftpoperator": args.ftpoperator_password_file,
        "uploader": args.uploader_password_file,
    }
    accounts = {
        item.get("name"): item
        for item in request(token, "GET", "/accounts").get("accounts", [])
        if isinstance(item, dict)
    }
    for name, permission in EXPECTED.items():
        password = read_secret(password_paths[name], f"{name} password", 8, 256)
        if name in accounts:
            request(token, "PATCH", f"/accounts/{name}", {"password": password})
        else:
            request(token, "POST", "/accounts", {
                "name": name,
                "password": password,
                "permission": permission,
            })
        password = None
        print(f"{name}: provisioned ({permission})")

    verified = {item.get("name"): item for item in request(token, "GET", "/accounts").get("accounts", [])}
    for name, permission in EXPECTED.items():
        item = verified.get(name, {})
        if item.get("permission") != permission or item.get("protected") is not True:
            raise RuntimeError(f"post-provision verification failed for {name}")
    print("three protected default FTP accounts verified")


if __name__ == "__main__":
    main()
