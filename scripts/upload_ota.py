#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""NoTeaching-Robot OTA 双通道发布工具。

发布被拆成两个显式阶段：

1. ``prepare-dual`` 只做本地校验，生成包含完整文件清单和哈希的候选报告；
2. ``publish-dual`` 重新校验报告引用的本地文件，然后才连接服务器，同时发布
   neutral/brand 两个通道。

单通道发布、裸 ``payload_hash`` 和无证据的 ``seed_versions`` 已禁用。远端发布先把
两个通道的所有载荷上传到临时名并回读验证，再原子切换历史和两份 latest.json；任一
latest 切换失败时会回滚已经切换的 latest/history。
"""

from __future__ import annotations

import argparse
import base64
import datetime as _datetime
import errno
import hashlib
import hmac
import importlib.util
import io
import json
import os
import posixpath
import re
import secrets
import socket
import stat as stat_module
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any, BinaryIO, Iterable


PRODUCTION_HOST = "103.217.203.52"
PRODUCTION_PORT = 48890
PRODUCTION_USER = "root"
REMOTE_ROOT = "/var/www/ota"
PUBLISHED_APP_ID = "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
REPORT_SCHEMA = "noteaching-ota-dual-candidate-v1"
MANIFEST_SCHEMA_VERSION = 2
MANIFEST_SIGNATURE_ALGORITHM = "RSA-PKCS1-SHA256"
CLIENT_OTA_PUBLIC_KEY_FINGERPRINT = "5686a45ad2f6c2d84ba7d911c31cfb4ab972d6afcd9e677c0d9b535629749eda"
MAX_INSTALLER_SIZE_DELTA = 2 * 1024 * 1024
MAX_MANIFEST_JSON_BYTES = 256 * 1024
MAX_REMOTE_JSON_BYTES = 4 * 1024 * 1024
MAX_UPDATE_PAYLOAD_BYTES = 512 * 1024 * 1024
EXPECTED_FANUC_TP_COUNT = 12
EXPECTED_FANUC_PC_COUNT = 9
VERSION_RE = re.compile(r"^[0-9]{4}\.[0-9]{2}\.[0-9]{2}\.[0-9]{4}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SAFE_REMOTE_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,199}$")
PAYLOAD_EXCLUDE_FILES = {"build_version.txt", "deploy_notes.txt"}
PAYLOAD_EXCLUDE_TOPDIRS = {"data", "log", "result", "temp"}
CHANNELS = ("neutral", "brand")
CHANNEL_EXE = {
    "neutral": "QtWidgetsApplication4.exe",
    "brand": "HK-Pathlynx-CORPLA.exe",
}


class ReleaseGateError(RuntimeError):
    """发版硬门禁失败。"""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ReleaseGateError(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().lower()


def sha256_file(path: os.PathLike[str] | str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest().lower()


def _sha256_stream(stream: BinaryIO) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    for chunk in iter(lambda: stream.read(1 << 20), b""):
        if isinstance(chunk, str):
            chunk = chunk.encode("utf-8")
        digest.update(chunk)
        size += len(chunk)
    return digest.hexdigest().lower(), size


def parse_version(value: str) -> tuple[int, int, int, int]:
    _require(isinstance(value, str) and VERSION_RE.fullmatch(value) is not None,
             f"版本号格式非法：{value!r}；必须是 yyyy.MM.dd.HHmm。")
    year_text, month_text, day_text, hm_text = value.split(".")
    year, month, day = int(year_text), int(month_text), int(day_text)
    hour, minute = int(hm_text[:2]), int(hm_text[2:])
    _require(2000 <= year <= 9999, f"版本年份超出范围：{value}")
    try:
        _datetime.datetime(year, month, day, hour, minute)
    except ValueError as exc:
        raise ReleaseGateError(f"版本号不是有效日期时间：{value} ({exc})") from exc
    return year, month, day, hour * 100 + minute


def compare_versions(lhs: str, rhs: str) -> int:
    left = parse_version(lhs)
    right = parse_version(rhs)
    return (left > right) - (left < right)


def _expected_installer_name(channel: str, version: str) -> str:
    parse_version(version)
    if channel == "neutral":
        return f"NoTeaching-Robot-Setup-v{version}.exe"
    if channel == "brand":
        return f"HK-Pathlynx-CORPLA-Setup-v{version}.exe"
    raise ReleaseGateError(f"未知 OTA 通道：{channel!r}")


def _expected_patch_name(channel: str, version: str) -> str:
    _require(channel == "brand", "中性通道禁止发布增量补丁。")
    parse_version(version)
    return f"HK-Pathlynx-CORPLA-Patch-v{version}.zip"


def _validate_remote_name(name: str) -> None:
    _require(isinstance(name, str) and SAFE_REMOTE_NAME_RE.fullmatch(name) is not None,
             f"远端文件名不安全：{name!r}")
    _require(name not in {".", ".."} and "/" not in name and "\\" not in name,
             f"远端文件名必须是单一文件名：{name!r}")


def _contains_target_version(path: Path, version: str) -> bool:
    target_ascii = version.encode("ascii")
    target_utf16 = version.encode("utf-16le")
    overlap = max(len(target_ascii), len(target_utf16)) - 1
    tail = b""
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            data = tail + chunk
            if target_ascii in data or target_utf16 in data:
                return True
            tail = data[-overlap:] if overlap > 0 else b""
    return False


def _inventory_digest(entries: Iterable[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    for entry in entries:
        digest.update(
            f"{entry['path']}:{entry['size']}:{entry['sha256']}\n".encode("utf-8")
        )
    return digest.hexdigest().lower()


def inspect_dist(dist_dir: os.PathLike[str] | str, channel: str, version: str) -> dict[str, Any]:
    """校验 dist 并返回完整 inventory 与非 exe 载荷哈希。"""
    parse_version(version)
    _require(channel in CHANNELS, f"未知 OTA 通道：{channel!r}")
    dist = Path(dist_dir).expanduser()
    _require(dist.exists(), f"dist 目录不存在：{dist}")
    _require(dist.is_dir(), f"dist 路径不是目录：{dist}")
    dist = dist.resolve()

    entries: list[dict[str, Any]] = []
    casefold_paths: set[str] = set()
    for path in sorted(dist.rglob("*"), key=lambda item: item.as_posix().casefold()):
        _require(not path.is_symlink(), f"dist 中禁止符号链接：{path}")
        if not path.is_file():
            continue
        relative = path.relative_to(dist).as_posix()
        folded = relative.casefold()
        _require(folded not in casefold_paths, f"dist 中存在大小写冲突路径：{relative}")
        casefold_paths.add(folded)
        size = path.stat().st_size
        entries.append({"path": relative, "size": size, "sha256": sha256_file(path)})

    _require(entries, f"dist 目录为空：{dist}")
    expected_exe = CHANNEL_EXE[channel]
    root_executables = [
        entry["path"] for entry in entries
        if "/" not in entry["path"] and Path(entry["path"]).suffix.casefold() == ".exe"
    ]
    _require(root_executables == [expected_exe],
             f"{channel} dist 根目录必须且只能有 {expected_exe}；实际={root_executables}")
    exe_path = dist / expected_exe
    _require(_contains_target_version(exe_path, version),
             f"主程序未内嵌目标版本 {version}：{exe_path}")
    fanuc_tp = [entry for entry in entries if entry["path"].casefold().startswith("sdk/fanuc/")
                and entry["path"].casefold().endswith(".tp")]
    fanuc_pc = [entry for entry in entries if entry["path"].casefold().startswith("sdk/fanuc/")
                and entry["path"].casefold().endswith(".pc")]
    _require(len(fanuc_tp) == EXPECTED_FANUC_TP_COUNT and len(fanuc_pc) == EXPECTED_FANUC_PC_COUNT,
             f"{channel} dist 的 SDK/FANUC 运行产物必须是 "
             f"tp={EXPECTED_FANUC_TP_COUNT}/pc={EXPECTED_FANUC_PC_COUNT}；"
             f"实际 tp={len(fanuc_tp)} pc={len(fanuc_pc)}。")
    branding_entries = [entry for entry in entries if entry["path"].casefold().startswith("branding/")]
    if channel == "brand":
        _require(any(entry["path"].casefold() == "branding/branding.ini" for entry in branding_entries),
                 "brand dist 缺 branding/branding.ini，运行时会误入 neutral OTA 通道。")
    else:
        _require(not branding_entries, "neutral dist 混入 branding/，会误入 brand OTA 通道。")

    payload_entries: list[dict[str, Any]] = []
    for entry in entries:
        relative = entry["path"]
        folded = relative.casefold()
        top = folded.split("/", 1)[0]
        if folded == expected_exe.casefold():
            continue
        if folded in PAYLOAD_EXCLUDE_FILES or top in PAYLOAD_EXCLUDE_TOPDIRS:
            continue
        payload_entries.append(entry)
    _require(payload_entries,
             f"{channel} dist 除主程序/运行目录外没有可校验载荷，拒绝空载荷哈希。")

    payload_digest = hashlib.sha256()
    for entry in payload_entries:
        payload_digest.update(
            f"{entry['path']}:{entry['sha256']}\n".encode("utf-8")
        )
    exe_entry = next(entry for entry in entries if entry["path"] == expected_exe)
    return {
        "distDir": str(dist),
        "mainExe": expected_exe,
        "mainExeSize": exe_entry["size"],
        "mainExeSha256": exe_entry["sha256"],
        "payloadHash": payload_digest.hexdigest().lower(),
        "payloadFileCount": len(payload_entries),
        "inventoryHash": _inventory_digest(entries),
        "inventory": entries,
    }


def _cross_channel_evidence(channels: dict[str, Any]) -> dict[str, Any]:
    neutral = channels["neutral"]
    brand = channels["brand"]
    neutral_size = neutral["installer"]["size"]
    brand_size = brand["installer"]["size"]
    size_delta = abs(neutral_size - brand_size)
    _require(size_delta <= MAX_INSTALLER_SIZE_DELTA,
             f"中性/品牌安装包体积相差 {size_delta} 字节（超过 2 MiB），疑似漏打运行文件。")

    def comparable_inventory(channel_data: dict[str, Any]) -> list[dict[str, Any]]:
        main_exe = channel_data["dist"]["mainExe"].casefold()
        result = []
        for entry in channel_data["dist"]["inventory"]:
            folded = entry["path"].casefold()
            if folded == main_exe or folded in PAYLOAD_EXCLUDE_FILES:
                continue
            if folded.startswith("branding/") or folded == "icons/app.ico":
                continue
            result.append(entry)
        return result

    neutral_runtime = comparable_inventory(neutral)
    brand_runtime = comparable_inventory(brand)
    _require(neutral_runtime == brand_runtime,
             "中性/品牌 dist 的非品牌运行文件清单或哈希不一致，拒绝发布。")
    return {
        "installerSizeDelta": size_delta,
        "maxInstallerSizeDelta": MAX_INSTALLER_SIZE_DELTA,
        "sharedRuntimeFileCount": len(neutral_runtime),
        "sharedRuntimeInventoryHash": _inventory_digest(neutral_runtime),
    }


def _resolve_gate_file(path_value: Any, label: str) -> Path:
    _require(isinstance(path_value, str) and path_value.strip(), f"{label} 路径缺失。")
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError(f"{label} 不存在或不可访问。") from exc
    _require(path.is_file(), f"{label} 不是文件。")
    return path


def _same_local_path(left: Any, right: Any) -> bool:
    try:
        return os.path.normcase(str(Path(str(left)).expanduser().resolve(strict=True))) == \
            os.path.normcase(str(Path(str(right)).expanduser().resolve(strict=True)))
    except OSError:
        return False


def _normalize_gate_inventory(value: Any, label: str) -> list[dict[str, Any]]:
    _require(isinstance(value, list), f"{label} 必须是文件清单数组。")
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in value:
        _require(isinstance(item, dict), f"{label} 含非对象条目。")
        path = item.get("path")
        size = item.get("size")
        digest = item.get("sha256")
        _require(isinstance(path, str) and path and "\\" not in path and ":" not in path
                 and not path.startswith("/") and all(part not in {"", ".", ".."} for part in path.split("/")),
                 f"{label} 含非法相对路径：{path!r}")
        key = path.casefold()
        _require(key not in seen, f"{label} 含大小写冲突路径：{path}")
        seen.add(key)
        _require(isinstance(size, int) and not isinstance(size, bool) and size >= 0,
                 f"{label} 中 {path} 的 size 非法。")
        _require(isinstance(digest, str) and SHA256_RE.fullmatch(digest) is not None,
                 f"{label} 中 {path} 的 sha256 非法。")
        result.append({"path": path, "size": size, "sha256": digest})
    return sorted(result, key=lambda item: item["path"].casefold())


def _inspect_pair_gate(
    pair_gate_path_value: os.PathLike[str] | str,
    version: str,
    channels: dict[str, Any],
) -> dict[str, Any]:
    """把 PowerShell package/installer/pair PASS 链绑定到 OTA 候选。"""
    pair_path = _resolve_gate_file(str(pair_gate_path_value), "双通道 pair gate report")
    pair_bytes = pair_path.read_bytes()
    pair = _load_json_bytes(pair_bytes, f"pair gate {pair_path}")
    _require(isinstance(pair, dict), "pair gate 根节点必须是对象。")
    _require(pair.get("schemaVersion") == 1 and pair.get("kind") == "release-pair"
             and pair.get("status") == "pass" and isinstance(pair.get("runId"), str)
             and bool(pair.get("runId").strip()), "pair gate 不是有效 PASS 报告。")
    _require(pair.get("appId") == PUBLISHED_APP_ID, "pair gate AppId 不是已发布固定值。")
    _require(pair.get("version") == version, "pair gate 版本与 OTA 候选不一致。")
    current_gate_script = Path(__file__).resolve().with_name("release_gate_common.ps1")
    _require(current_gate_script.is_file(), "缺少当前 release_gate_common.ps1。")
    current_gate_sha = sha256_file(current_gate_script)
    _require(pair.get("gateScriptSha256") == current_gate_sha,
             "pair gate 由不同版本的 release_gate_common.ps1 生成。")
    pair_producer = Path(__file__).resolve().with_name("verify_release_pair.ps1")
    _require(pair_producer.is_file() and pair.get("producerScriptSha256") == sha256_file(pair_producer),
             "pair gate 由不同版本的 verify_release_pair.ps1 生成。")
    _require(pair.get("maxInstallerSizeDifferenceBytes") == MAX_INSTALLER_SIZE_DELTA,
             "pair gate 的双通道安装包体积阈值不一致。")
    actual_delta = abs(channels["neutral"]["installer"]["size"]
                       - channels["brand"]["installer"]["size"])
    _require(pair.get("installerSizeDifferenceBytes") == actual_delta,
             "pair gate 的安装包体积差证据已失效。")

    pair_fanuc_sha = pair.get("fanucManifestSha256")
    pair_migrator_sha = pair.get("configMigrateSha256")
    _require(isinstance(pair_fanuc_sha, str) and SHA256_RE.fullmatch(pair_fanuc_sha) is not None,
             "pair gate FANUC manifest 哈希非法。")
    _require(isinstance(pair_migrator_sha, str) and SHA256_RE.fullmatch(pair_migrator_sha) is not None,
             "pair gate ConfigMigrate 哈希非法。")

    channel_evidence: dict[str, Any] = {}
    package_reports: dict[str, dict[str, Any]] = {}
    for channel in CHANNELS:
        node = pair.get(channel)
        _require(isinstance(node, dict), f"pair gate 缺 {channel} 节点。")
        head = node.get("head")
        _require(isinstance(head, str) and re.fullmatch(r"[0-9a-f]{40}", head) is not None,
                 f"pair gate {channel}.head 非法。")
        installer_gate_path = _resolve_gate_file(
            node.get("installerGateReport"), f"{channel} installer gate report"
        )
        installer_gate_sha = sha256_file(installer_gate_path)
        _require(node.get("installerGateSha256") == installer_gate_sha,
                 f"{channel} installer gate report 在 pair gate 后发生变化。")
        installer_gate = _load_json_bytes(
            installer_gate_path.read_bytes(), f"{channel} installer gate report"
        )
        _require(isinstance(installer_gate, dict)
                 and installer_gate.get("schemaVersion") == 1
                 and installer_gate.get("kind") == "installer"
                 and installer_gate.get("status") == "pass"
                 and isinstance(installer_gate.get("runId"), str)
                 and bool(installer_gate.get("runId").strip()),
                 f"{channel} installer gate 不是有效 PASS 报告。")
        _require(installer_gate.get("gateScriptSha256") == current_gate_sha,
                 f"{channel} installer gate 脚本哈希已过期。")
        installer_producer = _resolve_gate_file(
            str(Path(str(installer_gate.get("repoRoot"))) / "scripts" / "build_installer.ps1"),
            f"{channel} build_installer.ps1",
        )
        _require(installer_gate.get("producerScriptSha256") == sha256_file(installer_producer),
                 f"{channel} installer gate producer 脚本已变化。")
        _require(installer_gate.get("appId") == PUBLISHED_APP_ID
                 and installer_gate.get("version") == version
                 and installer_gate.get("channel") == channel
                 and installer_gate.get("head") == head,
                 f"{channel} installer gate 身份/version/HEAD 不匹配。")

        installer_node = installer_gate.get("installer")
        candidate_installer = channels[channel]["installer"]
        _require(isinstance(installer_node, dict)
                 and _same_local_path(installer_node.get("path"), candidate_installer["path"])
                 and installer_node.get("name") == candidate_installer["file"]
                 and installer_node.get("size") == candidate_installer["size"]
                 and installer_node.get("sha256") == candidate_installer["sha256"]
                 and installer_node.get("productVersion") == version,
                 f"{channel} installer bytes/version 不再匹配 installer gate。")
        _require(node.get("installerSha256") == candidate_installer["sha256"]
                 and node.get("installerSize") == candidate_installer["size"],
                 f"{channel} installer 不再匹配 pair gate。")

        package_gate_path = _resolve_gate_file(
            installer_gate.get("packageGateReport"), f"{channel} package gate report"
        )
        package_gate_sha = sha256_file(package_gate_path)
        _require(installer_gate.get("packageGateReportSha256") == package_gate_sha,
                 f"{channel} package gate 在 installer gate 后发生变化。")
        package_gate = _load_json_bytes(package_gate_path.read_bytes(), f"{channel} package gate report")
        _require(isinstance(package_gate, dict)
                 and package_gate.get("schemaVersion") == 1
                 and package_gate.get("kind") == "package"
                 and package_gate.get("status") == "pass"
                 and isinstance(package_gate.get("runId"), str)
                 and bool(package_gate.get("runId").strip()),
                 f"{channel} package gate 不是有效 PASS 报告。")
        _require(package_gate.get("gateScriptSha256") == current_gate_sha,
                 f"{channel} package gate 脚本哈希已过期。")
        package_producer = _resolve_gate_file(
            str(Path(str(package_gate.get("repoRoot"))) / "scripts" / "build_release_package.ps1"),
            f"{channel} build_release_package.ps1",
        )
        _require(package_gate.get("producerScriptSha256") == sha256_file(package_producer),
                 f"{channel} package gate producer 脚本已变化。")
        _require(package_gate.get("appId") == PUBLISHED_APP_ID
                 and package_gate.get("version") == version
                 and package_gate.get("channel") == channel
                 and package_gate.get("head") == head,
                 f"{channel} package gate 身份/version/HEAD 不匹配。")
        _require(_same_local_path(package_gate.get("packageDir"), channels[channel]["dist"]["distDir"]),
                 f"{channel} package gate 未绑定当前候选 dist。")
        _require(package_gate.get("expectedExe") == channels[channel]["dist"]["mainExe"]
                 and package_gate.get("executableSha256") == channels[channel]["dist"]["mainExeSha256"],
                 f"{channel} package gate 主程序证据不匹配。")
        gate_inventory = _normalize_gate_inventory(
            package_gate.get("packageInventory"), f"{channel} package inventory"
        )
        candidate_inventory = _normalize_gate_inventory(
            channels[channel]["dist"]["inventory"], f"{channel} candidate inventory"
        )
        _require(gate_inventory == candidate_inventory,
                 f"{channel} dist inventory 在 package gate 后发生变化。")

        fanuc = package_gate.get("fanucManifest")
        migrator = package_gate.get("configMigrate")
        migrate_run = package_gate.get("configMigrateRun")
        _require(isinstance(fanuc, dict) and fanuc.get("sha256") == pair_fanuc_sha
                 and fanuc.get("fileCount") == EXPECTED_FANUC_TP_COUNT + EXPECTED_FANUC_PC_COUNT,
                 f"{channel} FANUC gate 证据不匹配。")
        _require(isinstance(migrator, dict) and migrator.get("sha256") == pair_migrator_sha
                 and isinstance(migrator.get("sourceSha256"), str)
                 and SHA256_RE.fullmatch(migrator["sourceSha256"]) is not None,
                 f"{channel} ConfigMigrate gate 证据不匹配。")
        _require(isinstance(migrate_run, dict)
                 and isinstance(migrate_run.get("sha256"), str)
                 and SHA256_RE.fullmatch(migrate_run["sha256"]) is not None,
                 f"{channel} ConfigMigrate_Run gate 证据不匹配。")
        package_reports[channel] = package_gate
        channel_evidence[channel] = {
            "head": head,
            "installerGateReport": str(installer_gate_path),
            "installerGateSha256": installer_gate_sha,
            "packageGateReport": str(package_gate_path),
            "packageGateSha256": package_gate_sha,
        }

    _require(package_reports["neutral"]["configMigrate"]["sourceSha256"]
             == package_reports["brand"]["configMigrate"]["sourceSha256"],
             "双通道 ConfigMigrate source provenance 不一致。")
    _require(package_reports["neutral"]["configMigrateRun"]["sha256"]
             == package_reports["brand"]["configMigrateRun"]["sha256"],
             "双通道 ConfigMigrate_Run.cmd 不一致。")
    return {
        "path": str(pair_path),
        "sha256": sha256_bytes(pair_bytes),
        "runId": pair["runId"],
        "gateScriptSha256": current_gate_sha,
        "producerScriptSha256": sha256_file(pair_producer),
        "appId": PUBLISHED_APP_ID,
        "version": version,
        "neutral": channel_evidence["neutral"],
        "brand": channel_evidence["brand"],
    }


def _inspect_installer(path_value: os.PathLike[str] | str, channel: str, version: str) -> dict[str, Any]:
    path = Path(path_value).expanduser()
    expected_name = _expected_installer_name(channel, version)
    _require(path.exists() and path.is_file(), f"找不到安装包：{path}")
    _require(not path.is_symlink(), f"安装包禁止符号链接：{path}")
    _require(path.name == expected_name,
             f"{channel} 安装包命名错误：期望 {expected_name}，实际 {path.name}")
    _validate_remote_name(path.name)
    path = path.resolve()
    size = path.stat().st_size
    _require(0 < size <= MAX_UPDATE_PAYLOAD_BYTES,
             f"安装包大小必须在 1..512 MiB：{path} ({size} bytes)")
    return {"path": str(path), "file": path.name, "size": size, "sha256": sha256_file(path)}


def _zip_member_is_symlink(info: zipfile.ZipInfo) -> bool:
    mode = (info.external_attr >> 16) & 0xFFFF
    return stat_module.S_ISLNK(mode)


def _inspect_patch(
    path_value: os.PathLike[str] | str,
    channel: str,
    version: str,
    dist_info: dict[str, Any],
) -> dict[str, Any]:
    path = Path(path_value).expanduser()
    expected_name = _expected_patch_name(channel, version)
    _require(path.exists() and path.is_file(), f"找不到增量补丁：{path}")
    _require(not path.is_symlink(), f"增量补丁禁止符号链接：{path}")
    _require(path.name == expected_name,
             f"增量补丁命名错误：期望 {expected_name}，实际 {path.name}")
    _validate_remote_name(path.name)
    path = path.resolve()
    _require(0 < path.stat().st_size <= MAX_UPDATE_PAYLOAD_BYTES,
             f"增量补丁大小必须在 1..512 MiB：{path} ({path.stat().st_size} bytes)")

    try:
        with zipfile.ZipFile(path, "r") as archive:
            members = archive.infolist()
            _require(len(members) == 1, f"增量补丁必须只含一个主程序；实际条目数={len(members)}")
            member = members[0]
            expected_exe = CHANNEL_EXE[channel]
            _require(not member.is_dir() and member.filename == expected_exe,
                     f"增量补丁只能包含根目录 {expected_exe}；实际={member.filename!r}")
            _require(not _zip_member_is_symlink(member), "增量补丁主程序不能是符号链接。")
            _require((member.flag_bits & 0x1) == 0, "增量补丁不能使用加密 ZIP 条目。")
            _require(member.file_size == dist_info["mainExeSize"],
                     "增量补丁内主程序大小与候选 dist 不一致。")
            with archive.open(member, "r") as stream:
                member_sha, member_size = _sha256_stream(stream)
    except (OSError, zipfile.BadZipFile, RuntimeError) as exc:
        if isinstance(exc, ReleaseGateError):
            raise
        raise ReleaseGateError(f"无法校验增量补丁 {path}: {exc}") from exc

    _require(member_size == dist_info["mainExeSize"], "增量补丁解压大小与候选主程序不一致。")
    _require(member_sha == dist_info["mainExeSha256"],
             "增量补丁内主程序哈希与候选 dist 主程序不一致。")
    return {
        "path": str(path),
        "file": path.name,
        "size": path.stat().st_size,
        "sha256": sha256_file(path),
        "embeddedExe": CHANNEL_EXE[channel],
        "embeddedExeSize": member_size,
        "embeddedExeSha256": member_sha,
    }


def _validate_notes(notes: str) -> str:
    _require(isinstance(notes, str), "更新说明必须是字符串。")
    _require(len(notes) <= 4000, "更新说明不能超过 4000 个字符。")
    _require(all(ch == "\n" or ch == "\r" or ord(ch) >= 0x20 for ch in notes),
             "更新说明包含非法控制字符。")
    return notes


def _candidate_digest(report_without_digest: dict[str, Any]) -> str:
    encoded = json.dumps(
        report_without_digest, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(encoded)


def prepare_dual_candidate(
    version: str,
    neutral_dist_dir: os.PathLike[str] | str,
    neutral_installer: os.PathLike[str] | str,
    brand_dist_dir: os.PathLike[str] | str,
    brand_installer: os.PathLike[str] | str,
    brand_patch: os.PathLike[str] | str,
    pair_gate_report: os.PathLike[str] | str,
    notes: str = "",
) -> dict[str, Any]:
    parse_version(version)
    notes = _validate_notes(notes)
    channels: dict[str, Any] = {}
    for channel, dist_dir, installer_path in (
        ("neutral", neutral_dist_dir, neutral_installer),
        ("brand", brand_dist_dir, brand_installer),
    ):
        dist_info = inspect_dist(dist_dir, channel, version)
        channel_info: dict[str, Any] = {
            "channel": channel,
            "dist": dist_info,
            "installer": _inspect_installer(installer_path, channel, version),
            "patch": None,
        }
        if channel == "brand":
            channel_info["patch"] = _inspect_patch(brand_patch, channel, version, dist_info)
        channels[channel] = channel_info

    report: dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "version": version,
        "notes": notes,
        "generatedAtUtc": _datetime.datetime.now(_datetime.timezone.utc).isoformat(),
        "channels": channels,
        "crossChannel": _cross_channel_evidence(channels),
        "pairGate": _inspect_pair_gate(pair_gate_report, version, channels),
    }
    digest_source = {key: value for key, value in report.items() if key != "generatedAtUtc"}
    report["candidateSha256"] = _candidate_digest(digest_source)
    return report


def _write_json_atomic(path_value: os.PathLike[str] | str, value: dict[str, Any], overwrite: bool) -> Path:
    path = Path(path_value).expanduser().resolve()
    _require(overwrite or not path.exists(), f"候选报告已存在，拒绝覆盖：{path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except OSError:
            pass
        raise
    return path


def _reject_duplicate_json_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ReleaseGateError(f"JSON 含重复字段：{key}")
        result[key] = value
    return result


def _load_json_bytes(data: bytes, label: str) -> Any:
    try:
        return json.loads(data.decode("utf-8"), object_pairs_hook=_reject_duplicate_json_keys)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReleaseGateError(f"{label} 不是有效 UTF-8 JSON：{exc}") from exc


def load_and_revalidate_report(path_value: os.PathLike[str] | str) -> dict[str, Any]:
    path = Path(path_value).expanduser()
    _require(path.exists() and path.is_file(), f"候选报告不存在：{path}")
    report = _load_json_bytes(path.read_bytes(), f"候选报告 {path}")
    _require(isinstance(report, dict), "候选报告根节点必须是对象。")
    _require(report.get("schema") == REPORT_SCHEMA, "候选报告 schema 不受支持。")
    version = report.get("version")
    parse_version(version)
    notes = _validate_notes(report.get("notes"))
    channels = report.get("channels")
    _require(isinstance(channels, dict) and set(channels) == set(CHANNELS),
             "候选报告必须同时且仅包含 neutral/brand。")
    supplied_digest = report.get("candidateSha256")
    _require(isinstance(supplied_digest, str) and SHA256_RE.fullmatch(supplied_digest),
             "候选报告 candidateSha256 非法。")
    digest_source = {
        key: value for key, value in report.items()
        if key not in {"generatedAtUtc", "candidateSha256"}
    }
    _require(_candidate_digest(digest_source) == supplied_digest, "候选报告内容摘要不匹配。")

    recomputed: dict[str, Any] = {}
    for channel in CHANNELS:
        channel_data = channels[channel]
        _require(isinstance(channel_data, dict), f"{channel} 候选节点非法。")
        dist_data = channel_data.get("dist")
        installer_data = channel_data.get("installer")
        _require(isinstance(dist_data, dict) and isinstance(installer_data, dict),
                 f"{channel} 候选缺 dist/installer。")
        current_dist = inspect_dist(dist_data.get("distDir"), channel, version)
        current_installer = _inspect_installer(installer_data.get("path"), channel, version)
        current_patch = None
        if channel == "brand":
            patch_data = channel_data.get("patch")
            _require(isinstance(patch_data, dict), "brand 候选必须包含单 exe 增量补丁。")
            current_patch = _inspect_patch(patch_data.get("path"), channel, version, current_dist)
        else:
            _require(channel_data.get("patch") is None, "neutral 候选禁止补丁。")
        recomputed[channel] = {
            "channel": channel,
            "dist": current_dist,
            "installer": current_installer,
            "patch": current_patch,
        }
        _require(recomputed[channel] == channel_data,
                 f"{channel} 候选文件在 prepare 后发生变化，拒绝发布。")

    cross_channel = _cross_channel_evidence(recomputed)
    _require(report.get("crossChannel") == cross_channel,
             "候选报告的双通道体积/运行文件对照证据不一致。")
    pair_gate_node = report.get("pairGate")
    _require(isinstance(pair_gate_node, dict), "候选报告缺 pairGate 证据。")
    pair_gate = _inspect_pair_gate(pair_gate_node.get("path"), version, recomputed)
    _require(pair_gate_node == pair_gate, "pair gate 或其引用报告在 prepare 后发生变化。")

    return {
        "schema": REPORT_SCHEMA,
        "version": version,
        "notes": notes,
        "channels": recomputed,
        "crossChannel": cross_channel,
        "pairGate": pair_gate,
        "candidateSha256": supplied_digest,
    }


def compute_base_min_version(history: dict[str, str], current_version: str, current_hash: str) -> tuple[str, dict[str, str]]:
    parse_version(current_version)
    _require(SHA256_RE.fullmatch(current_hash) is not None, "当前 payloadHash 非法。")
    normalized: dict[str, str] = {}
    for version, payload_digest in history.items():
        parse_version(version)
        _require(isinstance(payload_digest, str) and SHA256_RE.fullmatch(payload_digest) is not None,
                 f"payload_history 中 {version} 的哈希非法。")
        normalized[version] = payload_digest

    if current_version in normalized:
        _require(normalized[current_version] == current_hash,
                 f"远端历史中版本 {current_version} 已绑定不同 payloadHash，禁止改写同版本载荷。")
    if normalized:
        highest = max(normalized, key=parse_version)
        _require(compare_versions(current_version, highest) >= 0,
                 f"目标版本 {current_version} 低于远端历史最高版本 {highest}。")
    normalized[current_version] = current_hash
    ordered_versions = sorted(normalized, key=parse_version)
    index = ordered_versions.index(current_version)
    base = current_version
    for pos in range(index, -1, -1):
        version = ordered_versions[pos]
        if normalized[version] != current_hash:
            break
        base = version
    return base, normalized


def _json_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _load_signing_module() -> Any:
    module_path = Path(__file__).resolve().with_name("ota_manifest_signing.py")
    _require(module_path.is_file(), "缺少 OTA manifest 签名模块。")
    spec = importlib.util.spec_from_file_location("noteaching_ota_manifest_signing", module_path)
    _require(spec is not None and spec.loader is not None, "无法加载 OTA manifest 签名模块。")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _require(module.SCHEMA_VERSION == MANIFEST_SCHEMA_VERSION
             and module.SIGNATURE_ALGORITHM == MANIFEST_SIGNATURE_ALGORITHM,
             "OTA manifest 签名模块的 schema/算法与发布工具不一致。")
    return module


def _resolve_signing_key(path_value: os.PathLike[str] | str) -> Path:
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("OTA 签名密钥不存在或不可访问。") from exc
    _require(path.is_file(), "OTA 签名密钥路径不是文件。")
    repo_root = Path(__file__).resolve().parents[1]
    try:
        path.relative_to(repo_root)
    except ValueError:
        return path
    raise ReleaseGateError("OTA 签名私钥必须位于项目目录之外。")


def make_manifest_signer(key_path_value: os.PathLike[str] | str):
    """返回只输出签名文本的闭包；私钥内容不会进入 report/日志。"""
    key_path = _resolve_signing_key(key_path_value)
    signing = _load_signing_module()
    try:
        public_key = signing.load_private_key(key_path).public_key()
    except Exception as exc:
        raise ReleaseGateError(f"无法加载 OTA manifest 签名密钥：{exc}") from exc
    actual_public_fingerprint = sha256_bytes(signing.cng_public_blob(public_key))
    _require(hmac.compare_digest(actual_public_fingerprint, CLIENT_OTA_PUBLIC_KEY_FINGERPRINT),
             "OTA 签名私钥与客户端内置公钥不匹配；已在联网前拒绝。")

    def signer(manifest: dict[str, Any]) -> str:
        try:
            signature = signing.sign_manifest(manifest, key_path)
            _require(signing.verify_manifest(manifest, signature, public_key),
                     "OTA manifest 本地签名自检失败。")
            return signature
        except ReleaseGateError:
            raise
        except Exception as exc:
            raise ReleaseGateError(f"OTA manifest 签名失败：{exc}") from exc

    def verify(manifest: dict[str, Any]) -> bool:
        return signing.verify_manifest(manifest, manifest.get("signature", ""), public_key)

    signer.verify = verify  # type: ignore[attr-defined]
    return signer


def _is_missing_error(exc: BaseException) -> bool:
    return isinstance(exc, (FileNotFoundError,)) or getattr(exc, "errno", None) == errno.ENOENT


def _read_remote_bytes(
    sftp: Any,
    remote_path: str,
    *,
    missing_ok: bool = False,
    max_bytes: int = MAX_REMOTE_JSON_BYTES,
) -> bytes | None:
    remote_stat = _remote_stat(sftp, remote_path)
    if remote_stat is None:
        if missing_ok:
            return None
        raise ReleaseGateError(f"远端文件不存在：{remote_path}")
    _require(0 < int(remote_stat.st_size) <= max_bytes,
             f"远端 JSON 大小非法：{remote_path} ({remote_stat.st_size} bytes)")
    try:
        with sftp.open(remote_path, "rb") as stream:
            data = stream.read()
            result = data.encode("utf-8") if isinstance(data, str) else data
            _require(len(result) == int(remote_stat.st_size), f"远端 JSON 回读大小变化：{remote_path}")
            return result
    except OSError as exc:
        raise ReleaseGateError(f"读取远端文件失败：{remote_path}: {exc}") from exc


def _remote_stat(sftp: Any, remote_path: str) -> Any | None:
    try:
        return sftp.stat(remote_path)
    except OSError as exc:
        if _is_missing_error(exc):
            return None
        raise ReleaseGateError(f"读取远端状态失败：{remote_path}: {exc}") from exc


def _remote_sha_size(sftp: Any, remote_path: str) -> tuple[str, int]:
    try:
        with sftp.open(remote_path, "rb") as stream:
            digest, size = _sha256_stream(stream)
    except OSError as exc:
        raise ReleaseGateError(f"回读远端文件失败：{remote_path}: {exc}") from exc
    remote_stat = _remote_stat(sftp, remote_path)
    _require(remote_stat is not None and int(remote_stat.st_size) == size,
             f"远端 stat/回读大小不一致：{remote_path}")
    return digest, size


def _verify_remote(sftp: Any, remote_path: str, expected_sha: str, expected_size: int) -> None:
    actual_sha, actual_size = _remote_sha_size(sftp, remote_path)
    _require(actual_size == expected_size,
             f"远端文件大小不匹配：{remote_path}，期望 {expected_size}，实际 {actual_size}")
    _require(actual_sha == expected_sha,
             f"远端文件 SHA256 不匹配：{remote_path}，期望 {expected_sha}，实际 {actual_sha}")


def _validate_history_bytes(data: bytes | None, label: str) -> dict[str, str]:
    if data is None:
        return {}
    value = _load_json_bytes(data, label)
    _require(isinstance(value, dict), f"{label} 根节点必须是对象。")
    result: dict[str, str] = {}
    for version, digest in value.items():
        parse_version(version)
        _require(isinstance(digest, str) and SHA256_RE.fullmatch(digest) is not None,
                 f"{label} 中 {version} 的哈希非法。")
        result[version] = digest
    return result


def _validate_manifest(value: Any, channel: str, *, allow_legacy: bool = False) -> dict[str, Any]:
    _require(isinstance(value, dict), f"{channel} latest.json 根节点必须是对象。")
    schema_version = value.get("schemaVersion")
    legacy = schema_version is None
    if legacy:
        _require(allow_legacy, f"{channel} latest.json 缺 schemaVersion=2。")
        allowed_top = {"version", "file", "sha256", "size", "notes"}
        if channel == "brand" and value.get("patch") is not None:
            allowed_top.add("patch")
        _require(set(value) == allowed_top,
                 f"{channel} 旧 latest.json 含缺失/未知字段：{sorted(set(value) ^ allowed_top)}")
    else:
        _require(schema_version == MANIFEST_SCHEMA_VERSION,
                 f"{channel} latest.json schemaVersion 非法：{schema_version!r}")
        _require(value.get("channel") == channel, f"{channel} latest.json channel 不匹配。")
        _require(value.get("signatureAlgorithm") == MANIFEST_SIGNATURE_ALGORITHM,
                 f"{channel} latest.json signatureAlgorithm 不匹配。")
        signature = value.get("signature")
        _require(isinstance(signature, str), f"{channel} latest.json signature 缺失。")
        try:
            signature_bytes = base64.b64decode(signature, validate=True)
        except Exception as exc:
            raise ReleaseGateError(f"{channel} latest.json signature 不是合法 Base64。") from exc
        _require(len(signature_bytes) == 384,
                 f"{channel} latest.json signature 不是 RSA-3072 签名。")
        allowed_top = {
            "schemaVersion", "channel", "version", "file", "sha256", "size", "notes",
            "signatureAlgorithm", "signature",
        }
        if channel == "brand" and value.get("patch") is not None:
            allowed_top.add("patch")
        _require(set(value) == allowed_top,
                 f"{channel} v2 latest.json 含缺失/未知字段：{sorted(set(value) ^ allowed_top)}")
    required = {"version", "file", "sha256", "size", "notes"}
    _require(required.issubset(value), f"{channel} latest.json 缺少字段：{sorted(required - set(value))}")
    version = value["version"]
    parse_version(version)
    expected_file = _expected_installer_name(channel, version)
    _require(value["file"] == expected_file, f"{channel} latest 安装包名与版本不一致。")
    _require(isinstance(value["sha256"], str) and SHA256_RE.fullmatch(value["sha256"]) is not None,
             f"{channel} latest sha256 非法。")
    _require(isinstance(value["size"], int) and not isinstance(value["size"], bool)
             and 0 < value["size"] <= MAX_UPDATE_PAYLOAD_BYTES,
             f"{channel} latest size 非法。")
    _validate_notes(value["notes"])
    patch = value.get("patch")
    if patch is not None:
        _require(channel == "brand", "远端 neutral latest 不应含 patch。")
        _require(isinstance(patch, dict), "远端 patch 必须是对象。")
        patch_required = {"file", "sha256", "size", "baseMinVersion"}
        _require(set(patch) == patch_required,
                 f"远端 patch 含缺失/未知字段：{sorted(set(patch) ^ patch_required)}")
        _require(patch["file"] == _expected_patch_name(channel, version), "远端 patch 文件名与版本不一致。")
        _require(isinstance(patch["sha256"], str) and SHA256_RE.fullmatch(patch["sha256"]) is not None,
                 "远端 patch sha256 非法。")
        _require(isinstance(patch["size"], int) and not isinstance(patch["size"], bool)
                 and 0 < patch["size"] <= MAX_UPDATE_PAYLOAD_BYTES,
                 "远端 patch size 非法。")
        parse_version(patch["baseMinVersion"])
        _require(compare_versions(patch["baseMinVersion"], version) <= 0,
                 "远端 patch.baseMinVersion 高于目标版本。")
    return value


def _payload_identity(manifest: dict[str, Any]) -> dict[str, Any]:
    identity: dict[str, Any] = {
        "version": manifest["version"],
        "file": manifest["file"],
        "sha256": manifest["sha256"],
        "size": manifest["size"],
    }
    if manifest.get("patch") is not None:
        identity["patch"] = manifest["patch"]
    return identity


def _stage_local_file(
    sftp: Any,
    local_path: str,
    final_path: str,
    txn: str,
    expected_sha: str,
    expected_size: int,
) -> dict[str, Any]:
    actual_sha = sha256_file(local_path)
    actual_size = os.path.getsize(local_path)
    _require(actual_sha == expected_sha and actual_size == expected_size,
             f"候选文件在联网前后发生变化，拒绝上传：{local_path}")
    existing = _remote_stat(sftp, final_path)
    if existing is not None:
        _verify_remote(sftp, final_path, expected_sha, expected_size)
        return {"final": final_path, "temp": None, "sha256": expected_sha, "size": expected_size}
    temp_path = f"{final_path}.tmp.{txn}"
    try:
        sftp.remove(temp_path)
    except OSError as exc:
        if not _is_missing_error(exc):
            raise ReleaseGateError(f"清理远端临时文件失败：{temp_path}: {exc}") from exc
    sftp.put(local_path, temp_path)
    _verify_remote(sftp, temp_path, expected_sha, expected_size)
    return {"final": final_path, "temp": temp_path, "sha256": expected_sha, "size": expected_size}


def _stage_bytes(sftp: Any, data: bytes, final_path: str, txn: str, tag: str) -> dict[str, Any]:
    expected_sha = sha256_bytes(data)
    expected_size = len(data)
    temp_path = f"{final_path}.tmp.{txn}.{tag}"
    try:
        sftp.remove(temp_path)
    except OSError as exc:
        if not _is_missing_error(exc):
            raise ReleaseGateError(f"清理远端临时文件失败：{temp_path}: {exc}") from exc
    if hasattr(sftp, "putfo"):
        sftp.putfo(io.BytesIO(data), temp_path, file_size=len(data))
    else:
        with sftp.open(temp_path, "wb") as stream:
            stream.write(data)
    _verify_remote(sftp, temp_path, expected_sha, expected_size)
    return {"final": final_path, "temp": temp_path, "sha256": expected_sha, "size": expected_size}


def _atomic_commit(sftp: Any, staged: dict[str, Any]) -> None:
    temp_path = staged.get("temp")
    if temp_path is None:
        _verify_remote(sftp, staged["final"], staged["sha256"], staged["size"])
        return
    try:
        sftp.posix_rename(temp_path, staged["final"])
    except Exception as exc:
        raise ReleaseGateError(
            f"服务器不支持/未完成原子重命名 {temp_path} -> {staged['final']}：{exc}"
        ) from exc
    _verify_remote(sftp, staged["final"], staged["sha256"], staged["size"])
    staged["temp"] = None


def _restore_remote_bytes(sftp: Any, final_path: str, old_data: bytes | None, txn: str, tag: str) -> None:
    if old_data is None:
        try:
            sftp.remove(final_path)
        except OSError as exc:
            if not _is_missing_error(exc):
                raise
        return
    staged = _stage_bytes(sftp, old_data, final_path, txn, f"rollback-{tag}")
    _atomic_commit(sftp, staged)


def _cleanup_temps(sftp: Any, staged_items: Iterable[dict[str, Any]]) -> None:
    for staged in staged_items:
        temp_path = staged.get("temp")
        if not temp_path:
            continue
        try:
            sftp.remove(temp_path)
        except OSError:
            pass


def _build_remote_plan(sftp: Any, report: dict[str, Any], manifest_signer: Any) -> dict[str, dict[str, Any]]:
    version = report["version"]
    plans: dict[str, dict[str, Any]] = {}
    for channel in CHANNELS:
        candidate = report["channels"][channel]
        remote_dir = posixpath.join(REMOTE_ROOT, channel)
        latest_path = posixpath.join(remote_dir, "latest.json")
        history_path = posixpath.join(remote_dir, "payload_history.json")
        old_latest_bytes = _read_remote_bytes(
            sftp, latest_path, missing_ok=True, max_bytes=MAX_MANIFEST_JSON_BYTES
        )
        old_history_bytes = _read_remote_bytes(sftp, history_path, missing_ok=True)
        history = _validate_history_bytes(old_history_bytes, f"{channel} payload_history.json")
        base_min, new_history = compute_base_min_version(
            history, version, candidate["dist"]["payloadHash"]
        )

        installer = candidate["installer"]
        manifest: dict[str, Any] = {
            "schemaVersion": MANIFEST_SCHEMA_VERSION,
            "channel": channel,
            "version": version,
            "file": installer["file"],
            "sha256": installer["sha256"],
            "size": installer["size"],
            "notes": report["notes"],
            "signatureAlgorithm": MANIFEST_SIGNATURE_ALGORITHM,
        }
        patch = candidate.get("patch")
        if patch is not None:
            manifest["patch"] = {
                "file": patch["file"],
                "sha256": patch["sha256"],
                "size": patch["size"],
                "baseMinVersion": base_min,
            }
        try:
            manifest["signature"] = manifest_signer(manifest)
        except Exception as exc:
            if isinstance(exc, ReleaseGateError):
                raise
            raise ReleaseGateError(f"{channel} manifest 签名失败：{exc}") from exc
        _validate_manifest(manifest, channel)
        verifier = getattr(manifest_signer, "verify", None)
        _require(callable(verifier) and verifier(manifest),
                 f"{channel} manifest 签名生成后验签失败。")
        manifest_bytes = _json_bytes(manifest)
        _require(len(manifest_bytes) <= MAX_MANIFEST_JSON_BYTES,
                 f"{channel} latest.json 超过客户端 256 KiB 接收上限。")
        history_bytes = _json_bytes(new_history)

        old_manifest = None
        already_visible = False
        if old_latest_bytes is not None:
            old_manifest = _validate_manifest(
                _load_json_bytes(old_latest_bytes, f"{channel} latest.json"), channel, allow_legacy=True
            )
            if old_manifest.get("schemaVersion") == MANIFEST_SCHEMA_VERSION:
                _require(callable(verifier) and verifier(old_manifest),
                         f"{channel} 远端 latest.json 签名校验失败。")
            comparison = compare_versions(version, old_manifest["version"])
            _require(comparison >= 0,
                     f"{channel} 目标版本 {version} 低于远端 latest {old_manifest['version']}。")
            if comparison == 0:
                _require(old_manifest.get("schemaVersion") == MANIFEST_SCHEMA_VERSION,
                         f"{channel} 远端同版本仍是未签名旧清单；必须升新版本，禁止原版本补签。")
                _require(_payload_identity(old_manifest) == _payload_identity(manifest),
                         f"{channel} 远端同版本已绑定不同安装包/补丁，禁止覆盖。")
                already_visible = True
                # 同版本同载荷是幂等恢复，不允许借机改 notes/baseMinVersion。后续最终回读以
                # 已公开的原始字节为准，避免无 latest 切换却误判内容不一致。
                manifest = old_manifest
                manifest_bytes = old_latest_bytes
                _verify_remote(
                    sftp,
                    posixpath.join(remote_dir, old_manifest["file"]),
                    old_manifest["sha256"],
                    old_manifest["size"],
                )
                if old_manifest.get("patch") is not None:
                    old_patch = old_manifest["patch"]
                    _verify_remote(
                        sftp,
                        posixpath.join(remote_dir, old_patch["file"]),
                        old_patch["sha256"],
                        old_patch["size"],
                    )

        plans[channel] = {
            "channel": channel,
            "remoteDir": remote_dir,
            "latestPath": latest_path,
            "historyPath": history_path,
            "oldLatest": old_latest_bytes,
            "oldHistory": old_history_bytes,
            "manifest": manifest,
            "manifestBytes": manifest_bytes,
            "historyBytes": history_bytes,
            "candidate": candidate,
            "alreadyVisible": already_visible,
        }
    return plans


def _publish_dual_locked(sftp: Any, report: dict[str, Any], manifest_signer: Any) -> dict[str, Any]:
    """调用方持有远端发布锁时执行双通道发布。"""
    _require(report.get("schema") == REPORT_SCHEMA, "发布报告 schema 非法。")
    parse_version(report.get("version"))
    _require(set(report.get("channels", {})) == set(CHANNELS), "发布必须同时包含双通道。")
    _require(callable(manifest_signer), "发布必须提供 OTA manifest 签名器。")
    plans = _build_remote_plan(sftp, report, manifest_signer)
    txn = f"{report['version'].replace('.', '')}-{secrets.token_hex(6)}"
    all_staged: list[dict[str, Any]] = []
    history_committed: list[str] = []
    latest_committed: list[str] = []

    try:
        # 1) 两通道所有载荷先以临时名上传并回读 size+sha；此时 latest/history 均未变化。
        for channel in CHANNELS:
            plan = plans[channel]
            candidate = plan["candidate"]
            payload_stages = [
                _stage_local_file(
                    sftp,
                    candidate["installer"]["path"],
                    posixpath.join(plan["remoteDir"], candidate["installer"]["file"]),
                    txn,
                    candidate["installer"]["sha256"],
                    candidate["installer"]["size"],
                )
            ]
            if candidate.get("patch") is not None:
                payload_stages.append(
                    _stage_local_file(
                        sftp,
                        candidate["patch"]["path"],
                        posixpath.join(plan["remoteDir"], candidate["patch"]["file"]),
                        txn,
                        candidate["patch"]["sha256"],
                        candidate["patch"]["size"],
                    )
                )
            plan["payloadStages"] = payload_stages
            all_staged.extend(payload_stages)
            plan["historyStage"] = _stage_bytes(
                sftp, plan["historyBytes"], plan["historyPath"], txn, f"{channel}-history"
            )
            all_staged.append(plan["historyStage"])
            if not plan["alreadyVisible"]:
                plan["latestStage"] = _stage_bytes(
                    sftp, plan["manifestBytes"], plan["latestPath"], txn, f"{channel}-latest"
                )
                all_staged.append(plan["latestStage"])
            else:
                plan["latestStage"] = None

        # 2) 两通道载荷都验证完后才原子落最终文件名，再逐份回读。
        for channel in CHANNELS:
            for staged in plans[channel]["payloadStages"]:
                _atomic_commit(sftp, staged)

        # 3) payload_history 先原子切换；任一失败先回滚已切历史，latest 仍未变化。
        try:
            for channel in CHANNELS:
                history_committed.append(channel)
                _atomic_commit(sftp, plans[channel]["historyStage"])
        except Exception:
            for committed_channel in reversed(history_committed):
                committed_plan = plans[committed_channel]
                _restore_remote_bytes(
                    sftp,
                    committed_plan["historyPath"],
                    committed_plan["oldHistory"],
                    txn,
                    f"{committed_channel}-history",
                )
            raise

        # 4) latest 永远最后切换。第二份失败时回滚第一份及双通道历史。
        try:
            for channel in CHANNELS:
                stage = plans[channel]["latestStage"]
                if stage is None:
                    continue
                # 先登记再切换：即使 rename 已成功但随后的回读失败，也必须回滚该通道。
                latest_committed.append(channel)
                _atomic_commit(sftp, stage)

            # 最终远端回读：两份 latest 字节、引用载荷及 history 必须全部一致。
            for channel in CHANNELS:
                plan = plans[channel]
                latest_bytes = _read_remote_bytes(sftp, plan["latestPath"])
                _require(latest_bytes == plan["manifestBytes"],
                         f"{channel} latest.json 最终回读内容不一致。")
                history_bytes = _read_remote_bytes(sftp, plan["historyPath"])
                _require(history_bytes == plan["historyBytes"],
                         f"{channel} payload_history.json 最终回读内容不一致。")
                for staged in plan["payloadStages"]:
                    _verify_remote(sftp, staged["final"], staged["sha256"], staged["size"])
        except Exception as publish_exc:
            rollback_errors: list[str] = []
            for committed_channel in reversed(latest_committed):
                committed_plan = plans[committed_channel]
                try:
                    _restore_remote_bytes(
                        sftp,
                        committed_plan["latestPath"],
                        committed_plan["oldLatest"],
                        txn,
                        f"{committed_channel}-latest",
                    )
                except Exception as exc:
                    rollback_errors.append(f"{committed_channel} latest: {exc}")
            for committed_channel in reversed(history_committed):
                committed_plan = plans[committed_channel]
                try:
                    _restore_remote_bytes(
                        sftp,
                        committed_plan["historyPath"],
                        committed_plan["oldHistory"],
                        txn,
                        f"{committed_channel}-history",
                    )
                except Exception as exc:
                    rollback_errors.append(f"{committed_channel} history: {exc}")
            if rollback_errors:
                raise ReleaseGateError(
                    f"发布失败且回滚不完整：{publish_exc}；{'；'.join(rollback_errors)}"
                ) from publish_exc
            raise

        return {
            "version": report["version"],
            "channels": {
                channel: {
                    "latest": plans[channel]["manifest"],
                    "alreadyVisible": plans[channel]["alreadyVisible"],
                }
                for channel in CHANNELS
            },
        }
    finally:
        _cleanup_temps(sftp, all_staged)


def publish_dual_with_sftp(sftp: Any, report: dict[str, Any], manifest_signer: Any) -> dict[str, Any]:
    """通过已连接的 SFTP 发布；供 CLI 与离线假 SFTP 测试共同调用。"""
    lock_path = posixpath.join(REMOTE_ROOT, ".dual-publish.lock")
    try:
        sftp.mkdir(lock_path)
    except Exception as exc:
        raise ReleaseGateError(
            f"无法取得 OTA 双通道发布锁 {lock_path}；可能有另一发布正在进行或上次异常中止：{exc}"
        ) from exc
    try:
        return _publish_dual_locked(sftp, report, manifest_signer)
    finally:
        try:
            sftp.rmdir(lock_path)
        except Exception as exc:
            raise ReleaseGateError(f"发布锁清理失败，需人工检查：{lock_path}: {exc}") from exc


def _known_host_lookup_name(host: str, port: int) -> str:
    return host if port == 22 else f"[{host}]:{port}"


def _validate_known_hosts_path(path_value: str) -> str:
    try:
        path = Path(path_value).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ReleaseGateError("known_hosts 不存在或不可访问。") from exc
    _require(path.is_file(), f"known_hosts 不是文件：{path}")
    repo_root = Path(__file__).resolve().parents[1]
    try:
        path.relative_to(repo_root)
    except ValueError:
        return str(path)
    raise ReleaseGateError("known_hosts 必须放在项目目录之外，禁止随仓库分发信任根。")


def _normalize_host_fingerprint(value: str) -> str:
    _require(isinstance(value, str) and value.startswith("SHA256:"),
             "主机指纹必须使用 OpenSSH SHA256:<base64> 格式。")
    body = value[len("SHA256:"):].rstrip("=")
    _require(re.fullmatch(r"[A-Za-z0-9+/]{43}", body) is not None,
             "主机 SHA256 指纹格式非法。")
    return f"SHA256:{body}"


def make_ssh(
    host: str,
    port: int,
    user: str,
    password: str,
    *,
    known_hosts: str | None = None,
    host_key_sha256: str | None = None,
    timeout: int = 30,
) -> Any:
    """只信任项目外 known_hosts 或显式 SHA256 主机指纹；绝不自动接受新主机。"""
    _require(bool(known_hosts) ^ bool(host_key_sha256),
             "必须且只能提供 --known-hosts 或 --host-key-sha256 之一。")
    import paramiko

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    if known_hosts:
        client.load_host_keys(_validate_known_hosts_path(known_hosts))
    else:
        expected = _normalize_host_fingerprint(host_key_sha256 or "")
        transport = None
        raw_socket = None
        try:
            raw_socket = socket.create_connection((host, port), timeout=timeout)
            transport = paramiko.Transport(raw_socket)
            transport.start_client(timeout=timeout)
            server_key = transport.get_remote_server_key()
            actual_body = base64.b64encode(hashlib.sha256(server_key.asbytes()).digest()).decode("ascii").rstrip("=")
            actual = f"SHA256:{actual_body}"
            _require(hmac.compare_digest(actual, expected),
                     f"SSH 主机指纹不匹配：期望 {expected}，实际 {actual}")
            client.get_host_keys().add(_known_host_lookup_name(host, port), server_key.get_name(), server_key)
        finally:
            if transport is not None:
                transport.close()
            elif raw_socket is not None:
                raw_socket.close()

    client.connect(
        host,
        port=port,
        username=user,
        password=password,
        timeout=timeout,
        banner_timeout=timeout,
        auth_timeout=timeout,
        allow_agent=False,
        look_for_keys=False,
    )
    return client


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="NoTeaching-Robot OTA 双通道硬门禁发布工具")
    subparsers = parser.add_subparsers(dest="command", required=True)

    compute = subparsers.add_parser("compute", help="只校验一个 dist 并输出 inventory/payloadHash，不联网")
    compute.add_argument("--version", required=True)
    compute.add_argument("--channel", choices=CHANNELS, required=True)
    compute.add_argument("--dist-dir", required=True)

    prepare = subparsers.add_parser("prepare-dual", help="离线校验双通道并生成候选报告")
    prepare.add_argument("--version", required=True)
    prepare.add_argument("--neutral-dist-dir", required=True)
    prepare.add_argument("--neutral-installer", required=True)
    prepare.add_argument("--brand-dist-dir", required=True)
    prepare.add_argument("--brand-installer", required=True)
    prepare.add_argument("--brand-patch", required=True)
    prepare.add_argument("--pair-gate", required=True,
                         help="verify_release_pair.ps1 生成的 release-pair PASS 报告")
    prepare.add_argument("--notes", default="")
    prepare.add_argument("--report", required=True)
    prepare.add_argument("--force-report", action="store_true")

    publish = subparsers.add_parser("publish-dual", help="按候选报告发布双通道（会联网）")
    publish.add_argument("--report", required=True)
    publish.add_argument("--server", default=PRODUCTION_HOST)
    publish.add_argument("--port", type=int, default=PRODUCTION_PORT)
    publish.add_argument("--user", default=PRODUCTION_USER)
    publish.add_argument(
        "--signing-key",
        default=os.environ.get(
            "OTA_SIGNING_KEY_FILE",
            str(Path.home() / ".codex" / "secrets" / "ota-release-signing-key.dpapi"),
        ),
        help="项目外 CurrentUser-DPAPI 私钥；也可用 OTA_SIGNING_KEY_FILE",
    )
    trust = publish.add_mutually_exclusive_group(required=True)
    trust.add_argument("--known-hosts", help="项目目录外的专用 known_hosts 文件")
    trust.add_argument("--host-key-sha256", help="OpenSSH SHA256:<base64> 主机指纹")
    return parser


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    legacy_flags = {"--compute-only", "--payload-hash", "--seed-versions", "--channel", "--installer", "--patch"}
    if argv and argv[0].startswith("-") and argv[0] not in {"-h", "--help"}:
        used = sorted(set(argv) & legacy_flags)
        raise ReleaseGateError(
            "旧的单通道/裸哈希调用已禁用。请先运行 `upload_ota.py prepare-dual --help` "
            f"生成双通道候选报告，再运行 publish-dual。旧参数={used}"
        )

    args = _build_parser().parse_args(argv)
    if args.command == "compute":
        result = inspect_dist(args.dist_dir, args.channel, args.version)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
        return 0
    if args.command == "prepare-dual":
        report = prepare_dual_candidate(
            args.version,
            args.neutral_dist_dir,
            args.neutral_installer,
            args.brand_dist_dir,
            args.brand_installer,
            args.brand_patch,
            args.pair_gate,
            args.notes,
        )
        report_path = _write_json_atomic(args.report, report, args.force_report)
        print(f"双通道候选报告已生成：{report_path}")
        print(f"version={report['version']} candidateSha256={report['candidateSha256']}")
        for channel in CHANNELS:
            channel_data = report["channels"][channel]
            print(
                f"{channel}: files={len(channel_data['dist']['inventory'])} "
                f"inventory={channel_data['dist']['inventoryHash']} "
                f"payload={channel_data['dist']['payloadHash']}"
            )
        return 0
    if args.command == "publish-dual":
        _require(args.server == PRODUCTION_HOST and args.port == PRODUCTION_PORT,
                 f"当前发版规则只允许生产端 {PRODUCTION_HOST}:{PRODUCTION_PORT}。")
        report = load_and_revalidate_report(args.report)
        manifest_signer = make_manifest_signer(args.signing_key)
        password = os.environ.get("OTA_SSH_PASSWORD", "")
        _require(bool(password), "请通过环境变量 OTA_SSH_PASSWORD 提供密码；脚本不保存密码。")
        ssh = make_ssh(
            args.server,
            args.port,
            args.user,
            password,
            known_hosts=args.known_hosts,
            host_key_sha256=args.host_key_sha256,
        )
        try:
            sftp = ssh.open_sftp()
            try:
                result = publish_dual_with_sftp(sftp, report, manifest_signer)
            finally:
                sftp.close()
        finally:
            ssh.close()
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
        return 0
    raise ReleaseGateError(f"未知命令：{args.command}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ReleaseGateError as exc:
        print(f"发布门禁失败：{exc}", file=sys.stderr)
        raise SystemExit(2)
