#!/usr/bin/env python3
"""Migrate legacy Data/*.ini and weld process txt files into ConfigStore.db.

The main program intentionally does not read legacy files anymore. Run this
tool once on an existing Data directory before starting the new build.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import ctypes
from ctypes import wintypes
import datetime as _dt
import hashlib
import hmac
import json
import os
import re
import shutil
import sqlite3
import sys
import tempfile
from pathlib import Path


SECRET = b"NoTeachingRobotConfigStoreV1"
SCHEMA_VERSION = "5"
AUTH_SEMANTIC_VERSION = "2"
DPAPI_PREFIX = "dpapi:user:v1:"
DPAPI_BACKUP_MAGIC = b"NoTeaching-Robot ConfigStore DPAPI backup v1\n"
DPAPI_BACKUP_PURPOSE = "configstore-database-backup-v1"
SCRUB_STATE_KEY = "legacy_credential_scrub_state"
SCRUB_MANIFEST_KEY = "legacy_credential_scrub_manifest"
DPAPI_ENTROPY_PREFIX = (
    "NoTeaching-Robot|A5A7E2A0-8226-40BB-B126-94C5D298B3CF|credentials-v1|"
)
TEXT_FILE_NAMES = {"WeaveDate.txt", "WeldPara.txt"}
SETTINGS_REQUIRED_COLUMNS = {
    "scope_type",
    "scope_id",
    "module",
    "key_name",
    "value_text",
    "value_type",
    "sensitive",
    "encrypted",
    "updated_at",
}
MOJIBAKE_MARKERS = (
    "\ufffd",
    "锟斤拷",
    "ï¿½",
    "Ã",
    "Â",
    "鍙",
    "缁",
    "鐒",
    "哄",
    "櫒",
    "浜",
)

DEFAULT_ROBOT_NAMES = {
    "RobotA": "机器人1",
    "RobotB": "机器人2",
    "RobotC": "焊接机器人3",
    "RobotD": "焊接机器人4",
}

DEFAULT_CAMERA_NAMES = {
    "CAMERA0": "跟踪相机",
    "CAMERA1": "测量相机",
    "CAMERA2": "线扫相机1",
    "CAMERA3": "线扫相机2",
}

COMP_SEGMENT_COUNT = 4
POSE_COMP_MATCH_BY_POSE = "0"
POSE_SEGMENT_TYPES = (
    ("姿态0 / 低平台", "low_platform"),
    ("姿态1 / 上升边", "rising_edge"),
    ("姿态2 / 高平台", "high_platform"),
    ("姿态3 / 下降边", "falling_edge"),
)
SEAM_SEGMENT_TYPES = (
    ("低平台", "low_platform"),
    ("上升边", "rising_edge"),
    ("高平台", "high_platform"),
    ("下降边", "falling_edge"),
)

MEASURE_WELD_SCAN_DEFAULTS = (
    ("YMaxCar", "0"),
    ("YMinCar", "0"),
    ("YMaxRobot", "0"),
    ("YMinRobot", "0"),
    ("XMax", "0"),
    ("XMin", "0"),
    ("ZMax", "0"),
    ("ZMin", "0"),
    ("ScanSpeed", "0"),
    ("RunSpeed", "0"),
    ("CameraTimeOffsetMs", "0"),
    ("dAcc", "0"),
    ("dDec", "0"),
    ("UseComputedScanSafe", "0"),
    ("ScanSafeOffsetDistanceMm", "0"),
    ("ScanSafeGunAngleDeg", "0"),
    ("ScanSafeXDirection", "0"),
    ("ScanSafeLiftHeightMm", "0"),
    ("ScanSafeFlipWarnThresholdDeg", "0"),
    ("StartSafePulseNum", "0"),
    ("StartSafePulse0.nS", "0"),
    ("StartSafePulse0.nL", "0"),
    ("StartSafePulse0.nU", "0"),
    ("StartSafePulse0.nR", "0"),
    ("StartSafePulse0.nB", "0"),
    ("StartSafePulse0.nT", "0"),
    ("StartSafePulse0.lBX", "0"),
    ("StartSafePulse0.lBY", "0"),
    ("StartSafePulse0.lBZ", "0"),
    ("StartPulse.nS", "0"),
    ("StartPulse.nL", "0"),
    ("StartPulse.nU", "0"),
    ("StartPulse.nR", "0"),
    ("StartPulse.nB", "0"),
    ("StartPulse.nT", "0"),
    ("StartPulse.lBX", "0"),
    ("StartPulse.lBY", "0"),
    ("StartPulse.lBZ", "0"),
    ("StartPos.X", "0"),
    ("StartPos.Y", "0"),
    ("StartPos.Z", "0"),
    ("StartPos.RX", "0"),
    ("StartPos.RY", "0"),
    ("StartPos.RZ", "0"),
    ("StartPos.BX", "0"),
    ("StartPos.BY", "0"),
    ("StartPos.BZ", "0"),
    ("EndPos.X", "0"),
    ("EndPos.Y", "0"),
    ("EndPos.Z", "0"),
    ("EndPos.RX", "0"),
    ("EndPos.RY", "0"),
    ("EndPos.RZ", "0"),
    ("EndPos.BX", "0"),
    ("EndPos.BY", "0"),
    ("EndPos.BZ", "0"),
    ("EndSafePulseNum", "0"),
    ("EndSafePulse0.nS", "0"),
    ("EndSafePulse0.nL", "0"),
    ("EndSafePulse0.nU", "0"),
    ("EndSafePulse0.nR", "0"),
    ("EndSafePulse0.nB", "0"),
    ("EndSafePulse0.nT", "0"),
    ("EndSafePulse0.lBX", "0"),
    ("EndSafePulse0.lBY", "0"),
    ("EndSafePulse0.lBZ", "0"),
)

MEASURE_WELD_WELD_DEFAULTS = (
    ("WeldEnable", "1"),
    ("WeldSpeedMmPerMin", "400"),
    ("DryRunSpeedMmPerMin", "1000"),
    ("WeldSafeMoveSpeedMmPerMin", "1000"),
    ("StepOverlapRel", "20"),
    ("FinalWeldTrajectoryStepMm", "4"),
    ("WeldDirection", "1"),
    ("WorldCoorDir", "0"),
    ("RobotInstallDir", "0"),
    ("GunAngle", "0"),
    ("GunLaserAngle", "0"),
    ("GunCameraAngle", "0"),
    ("NormalWeldRx", "0"),
    ("NormalWeldRy", "0"),
    ("UseTaughtWeldPose", "0"),
    ("TaughtWeldPoseRX", "0"),
    ("TaughtWeldPoseRY", "0"),
    ("TaughtWeldPoseRZ", "0"),
    ("CornerTransitionLeadDis", "0"),
    ("WeldStartSkipDis", "0"),
    ("WeldEndSkipDis", "0"),
    ("WeldRzGainDeg", "0"),
    ("SlopeRzMinDeg", "-20"),
    ("SlopeRzMaxDeg", "20"),
)


def decode_file(path: Path, forced_encoding: str | None = None) -> tuple[str, str]:
    data = path.read_bytes()
    if forced_encoding:
        return data.decode(forced_encoding, errors="replace"), forced_encoding
    if data.startswith(b"\xff\xfe"):
        return data[2:].decode("utf-16le", errors="replace"), "utf-16le"
    if data.startswith(b"\xfe\xff"):
        return data[2:].decode("utf-16be", errors="replace"), "utf-16be"
    if data.startswith(b"\xef\xbb\xbf"):
        return data.decode("utf-8-sig", errors="replace"), "utf-8-sig"

    for encoding in ("utf-8", "gbk", "mbcs"):
        try:
            return data.decode(encoding), encoding
        except UnicodeDecodeError:
            pass
        except LookupError:
            pass

    candidates: list[tuple[int, str, str]] = []
    for encoding in ("utf-8", "gbk", "mbcs"):
        try:
            text = data.decode(encoding, errors="replace")
        except LookupError:
            continue
        candidates.append((text.count("\ufffd"), encoding, text))
    if candidates:
        replacement_count, encoding, text = min(candidates, key=lambda item: item[0])
        return text, f"{encoding}-replace({replacement_count})"
    return data.decode("utf-8", errors="replace"), "utf-8-replace"


def detect_mojibake(text: str) -> str | None:
    for marker in MOJIBAKE_MARKERS:
        if marker in text:
            return marker
    return None


def decode_checked_file(path: Path, forced_encoding: str | None, allow_mojibake: bool) -> str:
    text, encoding = decode_file(path, forced_encoding)
    marker = detect_mojibake(text)
    if marker and not allow_mojibake:
        raise SystemExit(
            "Possible mojibake detected while migrating legacy config.\n"
            f"File: {path}\n"
            f"Encoding used: {encoding}\n"
            f"Marker: {marker!r}\n"
            "Fix the source file or retry with --encoding gbk/utf-8/utf-16le. "
            "Use --allow-mojibake only if you confirm the text is acceptable."
        )
    return text


def sanitize_legacy_text_file(path: Path, forced_encoding: str | None, allow_mojibake: bool) -> str:
    text, encoding = decode_file(path, forced_encoding)
    marker = detect_mojibake(text)
    if not marker or allow_mojibake:
        return text

    sanitized = text
    for mojibake_marker in MOJIBAKE_MARKERS:
        sanitized = sanitized.replace(mojibake_marker, "")
    print(
        "Warning: mojibake markers were removed from legacy text file.\n"
        f"  File: {path}\n"
        f"  Encoding used: {encoding}\n"
        "  The structured numeric content is kept; damaged text/comment characters are dropped."
    )
    return sanitized


def check_ini_rows(path: Path, rows: list[tuple[str, str, str]], encoding: str, allow_mojibake: bool) -> None:
    if allow_mojibake:
        return
    for section, key, value in rows:
        marker = detect_mojibake("\n".join((section, key, value)))
        if marker:
            raise SystemExit(
                "Possible mojibake detected while migrating legacy INI values.\n"
                f"File: {path}\n"
                f"Encoding used: {encoding}\n"
                f"Section: {section}\n"
                f"Key: {key}\n"
                f"Marker: {marker!r}\n"
                "Fix the source value or retry with --encoding gbk/utf-8/utf-16le. "
                "Use --allow-mojibake only if you confirm the value is acceptable."
            )


def build_section_map(rows: list[tuple[str, str, str]]) -> dict[str, dict[str, str]]:
    sections: dict[str, dict[str, str]] = {}
    for section, key, value in rows:
        sections.setdefault(section, {})[key] = value
    return sections


def read_robot_custom_name(data_dir: Path, robot_name: str) -> str | None:
    if not robot_name:
        return None
    robot_para = data_dir / robot_name / "RobotPara.ini"
    if not robot_para.exists():
        return None
    text, _encoding = decode_file(robot_para)
    for section, key, value in parse_ini(text):
        if section == "BaseParam" and key == "CustomName" and not detect_mojibake(value):
            return value.strip() or None
    return None


def default_for_mojibake_value(
    data_dir: Path,
    section: str,
    key: str,
    sections: dict[str, dict[str, str]],
) -> str | None:
    if section == "ChineseName":
        match = re.fullmatch(r"Unit(\d+)", key)
        if match:
            robot_name = sections.get("UnitName", {}).get(key, "").strip()
            custom_name = read_robot_custom_name(data_dir, robot_name)
            if custom_name:
                return custom_name
            if robot_name in DEFAULT_ROBOT_NAMES:
                return DEFAULT_ROBOT_NAMES[robot_name]
            unit_index = int(match.group(1)) + 1
            unit_type = sections.get("UnitType", {}).get(key, "").strip()
            return f"焊接机器人{unit_index}" if unit_type == "6" else f"机器人{unit_index}"

    if section == "MeasureWeldGroups":
        match = re.fullmatch(r"Group(\d+)Name", key)
        if match:
            return f"参数组{int(match.group(1)) + 1}"

    if key == "CameraName" and section in DEFAULT_CAMERA_NAMES:
        return DEFAULT_CAMERA_NAMES[section]

    if key == "CustomName":
        robot_name = sections.get("BaseParam", {}).get("RobotName", "").strip()
        if robot_name in DEFAULT_ROBOT_NAMES:
            return DEFAULT_ROBOT_NAMES[robot_name]

    if key.endswith("Name"):
        return "默认名称"
    return None


def replace_mojibake_ini_rows(
    data_dir: Path,
    path: Path,
    rows: list[tuple[str, str, str]],
    encoding: str,
    allow_mojibake: bool,
) -> tuple[list[tuple[str, str, str]], list[tuple[str, str, str, str]]]:
    sections = build_section_map(rows)
    fixed_rows: list[tuple[str, str, str]] = []
    replacements: list[tuple[str, str, str, str]] = []

    for section, key, value in rows:
        marker = detect_mojibake("\n".join((section, key, value)))
        if marker:
            default_value = default_for_mojibake_value(data_dir, section, key, sections)
            if default_value is None:
                if allow_mojibake:
                    fixed_rows.append((section, key, value))
                    continue
                raise SystemExit(
                    "Possible mojibake detected while migrating legacy INI values.\n"
                    f"File: {path}\n"
                    f"Encoding used: {encoding}\n"
                    f"Section: {section}\n"
                    f"Key: {key}\n"
                    f"Marker: {marker!r}\n"
                    "No default value is known for this field. Fix the source value, "
                    "add a default to the migration tool, or use --allow-mojibake."
                )
            fixed_rows.append((section, key, default_value))
            replacements.append((section, key, value, default_value))
            continue
        fixed_rows.append((section, key, value))

    return fixed_rows, replacements


def normalize_text(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def strip_inline_comment(value: str) -> str:
    # Keep semicolons that are part of values; strip normal "value ; comment".
    return re.split(r"\s+;", value, maxsplit=1)[0].strip()


def parse_ini(text: str) -> list[tuple[str, str, str]]:
    rows: list[tuple[str, str, str]] = []
    section = ""
    for raw_line in normalize_text(text).split("\n"):
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith(";"):
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip()
            continue
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if not key:
            continue
        rows.append((section, key, strip_inline_comment(value)))
    return rows


def _encode_legacy_ini(text: str, encoding: str, original: bytes) -> bytes:
    if "-replace(" in encoding:
        raise ValueError(
            "Refusing to rewrite a legacy INI that required replacement decoding"
        )
    if encoding == "utf-8-sig":
        return text.encode("utf-8-sig")
    if encoding == "utf-16le":
        encoded = text.encode("utf-16le")
        return (b"\xff\xfe" + encoded) if original.startswith(b"\xff\xfe") else encoded
    if encoding == "utf-16be":
        encoded = text.encode("utf-16be")
        return (b"\xfe\xff" + encoded) if original.startswith(b"\xfe\xff") else encoded
    return text.encode(encoding)


def _is_explicit_legacy_credential_key(key: str) -> bool:
    canonical = re.sub(r"[^a-z0-9]", "", key.casefold())
    if canonical in {"pass", "ftppass"}:
        return True
    return canonical.endswith((
        "password",
        "passwd",
        "pwd",
        "passphrase",
        "token",
        "secret",
        "credential",
        "credentials",
        "apikey",
    ))


def _legacy_ini_credential_action(path: Path, section: str, key: str) -> str:
    section_name = normalize_section(section).casefold()
    key_name = normalize_source_key(key).casefold()
    if section_name.startswith("users/"):
        if key_name == "passwordhash":
            return "remove"
        if key_name in {"mustchangepassword", "passwordchangedat"}:
            return "keep"
    if section_name in {"savedpasswords", "rememberedcredentials"}:
        return "remove"
    if section_name == "general" and key_name in {"rememberpassword", "autologin"}:
        return "disable"
    if key_name == "passwordbase64":
        return "remove"
    if _is_explicit_legacy_credential_key(key_name):
        return "remove"
    return "keep"


def _atomic_replace_bytes(path: Path, content: bytes) -> None:
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.credential-scrub-",
        suffix=".tmp",
        dir=path.parent,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(file_descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            shutil.copystat(path, temporary_path)
        os.replace(temporary_path, path)
    except Exception:
        try:
            os.close(file_descriptor)
        except OSError:
            pass
        temporary_path.unlink(missing_ok=True)
        raise


def _sanitized_legacy_ini_bytes(
    path: Path,
    forced_encoding: str | None,
) -> tuple[bytes, int]:
    original = path.read_bytes()
    text, encoding = decode_file(path, forced_encoding)
    section = ""
    drop_section = False
    output: list[str] = []
    removed_values = 0
    changed = False
    for line in text.splitlines(keepends=True):
        body = line.rstrip("\r\n")
        newline = line[len(body):]
        stripped = body.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            section = stripped[1:-1].strip()
            drop_section = (
                normalize_section(section).casefold()
                in {"savedpasswords", "rememberedcredentials"}
            )
            if drop_section:
                changed = True
                continue
            output.append(line)
            continue
        if drop_section:
            if stripped and not stripped.startswith((";", "#")) and "=" in body:
                removed_values += 1
            changed = True
            continue
        if not stripped or stripped.startswith((";", "#")) or "=" not in body:
            output.append(line)
            continue
        separator = body.find("=")
        key = body[:separator].strip()
        action = _legacy_ini_credential_action(path, section, key)
        if action == "remove":
            changed = True
            removed_values += 1
            continue
        if action == "disable":
            output.append(body[:separator + 1] + "0" + newline)
            changed = changed or body[separator + 1:].strip() != "0"
            continue
        output.append(line)
    if not changed:
        return original, 0
    return _encode_legacy_ini("".join(output), encoding, original), removed_values


def prepare_legacy_ini_credential_scrub(
    data_dir: Path,
    forced_encoding: str | None,
) -> list[dict[str, object]]:
    root = data_dir.resolve()
    candidates = sorted(
        path for path in data_dir.rglob("*")
        if path.is_file() and path.suffix.casefold() == ".ini"
    )
    manifest: list[dict[str, object]] = []
    for path in candidates:
        original = path.read_bytes()
        sanitized, removed_values = _sanitized_legacy_ini_bytes(path, forced_encoding)
        if sanitized == original:
            continue
        relative = path.resolve().relative_to(root).as_posix()
        manifest.append({
            "path": relative,
            "before_sha256": hashlib.sha256(original).hexdigest(),
            "after_sha256": hashlib.sha256(sanitized).hexdigest(),
            "removed_values": removed_values,
        })
    return manifest


def serialize_legacy_credential_scrub_manifest(
    manifest: list[dict[str, object]],
) -> str:
    return json.dumps(manifest, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def parse_legacy_credential_scrub_manifest(value: str) -> list[dict[str, object]]:
    if len(value.encode("utf-8")) > 1024 * 1024:
        raise ValueError("Legacy credential scrub manifest is too large")
    parsed = json.loads(value)
    if not isinstance(parsed, list):
        raise ValueError("Legacy credential scrub manifest is not a list")
    for item in parsed:
        if not isinstance(item, dict) or set(item) != {
            "path", "before_sha256", "after_sha256", "removed_values"
        }:
            raise ValueError("Legacy credential scrub manifest entry is invalid")
        if (
            not isinstance(item["path"], str)
            or not isinstance(item["removed_values"], int)
            or item["removed_values"] < 0
            or re.fullmatch(r"[0-9a-f]{64}", str(item["before_sha256"])) is None
            or re.fullmatch(r"[0-9a-f]{64}", str(item["after_sha256"])) is None
        ):
            raise ValueError("Legacy credential scrub manifest values are invalid")
    return parsed


def apply_legacy_ini_credential_scrub(
    data_dir: Path,
    forced_encoding: str | None,
    manifest: list[dict[str, object]],
) -> tuple[int, int]:
    root = data_dir.resolve()
    modified_files = 0
    removed_values = 0
    for item in manifest:
        relative = Path(str(item["path"]))
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Legacy credential scrub path escapes the Data directory")
        path = (root / relative).resolve()
        path.relative_to(root)
        if not path.is_file():
            raise ValueError(f"Legacy credential scrub source is missing: {path}")
        current = path.read_bytes()
        current_hash = hashlib.sha256(current).hexdigest()
        before_hash = str(item["before_sha256"])
        after_hash = str(item["after_sha256"])
        if hmac.compare_digest(current_hash, after_hash):
            continue
        if not hmac.compare_digest(current_hash, before_hash):
            raise ValueError(f"Legacy credential source changed after migration: {path}")
        sanitized, removed = _sanitized_legacy_ini_bytes(path, forced_encoding)
        if not hmac.compare_digest(hashlib.sha256(sanitized).hexdigest(), after_hash):
            raise ValueError(f"Legacy credential scrub output changed unexpectedly: {path}")
        _atomic_replace_bytes(path, sanitized)
        modified_files += 1
        removed_values += removed
    return modified_files, removed_values


def scrub_legacy_ini_credentials(
    data_dir: Path,
    forced_encoding: str | None,
) -> tuple[int, int]:
    manifest = prepare_legacy_ini_credential_scrub(data_dir, forced_encoding)
    return apply_legacy_ini_credential_scrub(data_dir, forced_encoding, manifest)


def normalize_data_path(path: Path, data_dir: Path) -> str:
    relative = path.resolve().relative_to(data_dir.resolve())
    return "Data/" + relative.as_posix()


def normalize_section(section: str) -> str:
    cleaned = section.strip().replace("\\", "/")
    return cleaned.strip("/")


def normalize_source_key(key: str) -> str:
    cleaned = key.strip().replace("\\", "/")
    return cleaned.strip("/")


def clean_db_name(text: str) -> str:
    cleaned = text.strip().replace("\\", "/").replace("_", " ")
    return re.sub(r"\s+", " ", cleaned).strip()


def base_file_name_for_db(source_path: str) -> str:
    file_name = source_path.replace("\\", "/").rsplit("/", 1)[-1]
    if "." in file_name:
        file_name = file_name.rsplit(".", 1)[0]
    return clean_db_name(file_name) or "未命名"


def normalize_source_path_for_db(source_path: str) -> str:
    normalized = source_path.strip().replace("\\", "/")
    while "//" in normalized:
        normalized = normalized.replace("//", "/")

    lower = normalized.lower()
    for marker in ("/data/", "/result/"):
        pos = lower.find(marker)
        if pos >= 0:
            return normalized[pos + 1:]
    if lower.startswith("data/") or lower.startswith("result/"):
        return normalized
    return normalized


def build_file_identity(source_path: str) -> dict[str, str]:
    normalized = normalize_source_path_for_db(source_path)
    parts = [part for part in normalized.split("/") if part]
    category = "配置数据"
    robot_name = ""

    if parts and parts[0].lower() == "data":
        category = "数据配置"
        if len(parts) >= 3 and parts[1].lower().startswith("robot"):
            robot_name = clean_db_name(parts[1])
    elif parts and parts[0].lower() == "result":
        category = "结果数据"
        if len(parts) >= 3 and parts[1].lower().startswith("robot"):
            robot_name = clean_db_name(parts[1])

    return {
        "category": category,
        "robot_name": robot_name or "全局",
        "file_name": base_file_name_for_db(normalized),
        "source_path": normalized,
    }


def split_key_name(source_key: str) -> tuple[str, str]:
    key = source_key
    item = ""
    parts = [part for part in source_key.replace("\\", "/").split("/") if part]
    if len(parts) > 1:
        key = parts[-1]
        item = " / ".join(parts[:-1])
    else:
        underscore = source_key.rfind("_")
        if 0 < underscore < len(source_key) - 1:
            item = source_key[:underscore]
            key = source_key[underscore + 1:]
    return clean_db_name(item), clean_db_name(key)


def build_ini_identity(source_path: str, section: str, key: str) -> dict[str, str]:
    file_identity = build_file_identity(source_path)
    source_section = normalize_section(section)
    source_key = normalize_source_key(key)
    section_parts = [part for part in source_section.split("/") if part]
    group_name = clean_db_name(section_parts[0]) if section_parts else "默认"
    section_item = " / ".join(section_parts[1:]) if len(section_parts) > 1 else ""
    key_item, key_name = split_key_name(source_key)
    item_name = clean_db_name(" / ".join(part for part in (section_item, key_item) if part.strip()))
    file_identity.update(
        {
            "group_name": group_name,
            "item_name": item_name,
            "key_name": key_name or "值",
            "source_section": source_section,
            "source_key": source_key,
        }
    )
    return file_identity


def is_sensitive_setting_key(key: str) -> bool:
    lowered = key.lower()
    return any(marker in lowered for marker in (
        "password", "passwd", "pass", "token", "secret", "credential", "api_key", "apikey"
    ))


def is_portable_authentication_value(scope_type: str, module: str, key: str) -> bool:
    return (
        normalize_section(scope_type).lower() == "account"
        and normalize_section(module).lower() == "profile"
        and normalize_source_key(key).lower()
        in {"passwordhash", "role", "mustchangepassword", "passwordchangedat"}
    )


def requires_dpapi_protection(
    scope_type: str,
    module: str,
    key: str,
    sensitive: bool,
) -> bool:
    if not sensitive:
        return False
    if is_portable_authentication_value(scope_type, module, key):
        return False
    return True


def module_base_from_stored_file_name(stored_file_name: str) -> str:
    file_name = stored_file_name.replace("\\", "/").rsplit("/", 1)[-1]
    base_name = file_name.rsplit(".", 1)[0].strip() if "." in file_name else file_name.strip()
    lowered = base_name.lower()
    if lowered == "contralunitinfo":
        return "ControlUnits"
    if lowered == "robotpara":
        return "RobotPara"
    if lowered == "cameraparam":
        return "CameraParam"
    if lowered.startswith("handeyematrix_"):
        return f"HandEyeMatrix/{base_name[len('HandEyeMatrix_'):]}"
    if lowered == "handeyematrix":
        return "HandEyeMatrix"
    if lowered.startswith("handeyecalibration_"):
        return f"HandEyeCalibration/{base_name[len('HandEyeCalibration_'):]}"
    if lowered == "linescanparam":
        return "LineScanParam"
    if lowered == "linecoarsescanparam":
        return "LineCoarseScanParam"
    if lowered == "measureweldparam":
        return "MeasureWeldParam"
    if lowered == "weldposecompparam":
        return "WeldPoseCompParam"
    if lowered == "weldseamcompparam":
        return "WeldSeamCompParam"
    if lowered == "weavedate":
        return "WeldProcess/WeaveData"
    if lowered == "weldpara":
        return "WeldProcess/WeldParameters"
    return normalize_section(base_name or file_name)


def build_scoped_file_identity(source_path: str, key_name: str = "") -> dict[str, object]:
    normalized = normalize_source_path_for_db(source_path)
    parts = [part for part in normalized.split("/") if part]
    scope_type = "global"
    scope_id = ""
    stored_file_name = normalized.rsplit("/", 1)[-1] if normalized else ""

    if parts and parts[0].lower() == "data":
        if len(parts) >= 4 and parts[1].lower() == "workpiecetemplates":
            scope_type = "workpiece_template"
            scope_id = parts[2].strip()
            stored_file_name = "/".join(parts[3:])
        elif len(parts) >= 3:
            scope_type = "robot"
            scope_id = parts[1].strip()
            stored_file_name = "/".join(parts[2:])
        elif len(parts) >= 2:
            stored_file_name = "/".join(parts[1:])
    elif parts and parts[0].lower() == "result":
        scope_type = "result"
        if len(parts) >= 3:
            scope_id = parts[1].strip()
            stored_file_name = "/".join(parts[2:])

    key = normalize_source_key(key_name or "Content")
    module = normalize_section(module_base_from_stored_file_name(stored_file_name))
    return {
        "valid": bool(scope_type.strip() and module.strip() and key.strip()),
        "scope_type": normalize_section(scope_type).lower(),
        "scope_id": scope_id.strip(),
        "module": module,
        "key_name": key,
        "value_type": "text",
        "sensitive": is_sensitive_setting_key(key),
    }


def build_scoped_ini_identity(source_path: str, section: str, key: str) -> dict[str, object]:
    identity = build_scoped_file_identity(source_path, key)
    normalized_section = normalize_section(section)
    if normalized_section:
        identity["module"] = normalize_section(f"{identity['module']}/{normalized_section}")
    identity["value_type"] = "string"
    identity["sensitive"] = bool(identity["sensitive"]) or is_sensitive_setting_key(normalized_section) or is_sensitive_setting_key(key)
    identity["valid"] = bool(identity["valid"] and str(identity["module"]).strip())
    return identity


def key_stream(nonce: bytes, size: int) -> bytes:
    output = bytearray()
    counter = 0
    while len(output) < size:
        block = SECRET + nonce + b":" + str(counter).encode("ascii")
        output.extend(hashlib.sha256(block).digest())
        counter += 1
    return bytes(output[:size])


def protect_legacy_text(text: str) -> str:
    nonce = os.urandom(16)
    data = bytearray(text.encode("utf-8"))
    stream = key_stream(nonce, len(data))
    for index, byte in enumerate(stream):
        data[index] ^= byte
    nonce_b64 = base64.b64encode(nonce).decode("ascii").rstrip("=")
    data_b64 = base64.b64encode(bytes(data)).decode("ascii").rstrip("=")
    return f"enc:v1:{nonce_b64}:{data_b64}"


def padded_base64(text: str) -> bytes:
    encoded = text.encode("ascii")
    pad = len(encoded) % 4
    if pad:
        encoded += b"=" * (4 - pad)
    return encoded


def unprotect_legacy_text(stored: str) -> str | None:
    parts = stored.split(":")
    if len(parts) != 4 or parts[0] != "enc" or parts[1] != "v1":
        return None
    try:
        nonce = base64.b64decode(padded_base64(parts[2]))
        data = bytearray(base64.b64decode(padded_base64(parts[3])))
    except Exception:
        return None
    if len(nonce) != 16:
        return None
    stream = key_stream(nonce, len(data))
    for index, byte in enumerate(stream):
        data[index] ^= byte
    try:
        return bytes(data).decode("utf-8")
    except UnicodeDecodeError:
        return bytes(data).decode("utf-8", errors="replace")


class _DataBlob(ctypes.Structure):
    _fields_ = [("cbData", wintypes.DWORD), ("pbData", ctypes.POINTER(ctypes.c_ubyte))]


def protection_purpose(scope_type: str, scope_id: str, module: str, key: str) -> str:
    return "\n".join((
        normalize_section(scope_type).lower(),
        scope_id.strip(),
        normalize_section(module),
        normalize_source_key(key),
    ))


def _blob_from_bytes(data: bytes) -> tuple[_DataBlob, ctypes.Array]:
    buffer = ctypes.create_string_buffer(data, max(1, len(data)))
    pointer = ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte)) if data else None
    return _DataBlob(len(data), pointer), buffer


def _dpapi_crypt(data: bytes, purpose: str, protect: bool) -> bytes:
    if os.name != "nt" or not purpose.strip():
        raise RuntimeError("DPAPI CurrentUser protection requires Windows and a non-empty purpose")
    crypt32 = ctypes.WinDLL("Crypt32.dll", use_last_error=True)
    kernel32 = ctypes.WinDLL("Kernel32.dll", use_last_error=True)
    crypt32.CryptProtectData.argtypes = [
        ctypes.POINTER(_DataBlob), wintypes.LPCWSTR, ctypes.POINTER(_DataBlob),
        ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(_DataBlob),
    ]
    crypt32.CryptUnprotectData.argtypes = [
        ctypes.POINTER(_DataBlob), ctypes.c_void_p, ctypes.POINTER(_DataBlob),
        ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(_DataBlob),
    ]
    crypt32.CryptProtectData.restype = wintypes.BOOL
    crypt32.CryptUnprotectData.restype = wintypes.BOOL
    kernel32.LocalFree.argtypes = [ctypes.c_void_p]
    kernel32.LocalFree.restype = ctypes.c_void_p
    function = crypt32.CryptProtectData if protect else crypt32.CryptUnprotectData
    function.restype = wintypes.BOOL
    input_blob, input_buffer = _blob_from_bytes(data)
    entropy_blob, entropy_buffer = _blob_from_bytes((DPAPI_ENTROPY_PREFIX + purpose).encode("utf-8"))
    output_blob = _DataBlob()
    flags = 0x1  # CRYPTPROTECT_UI_FORBIDDEN
    if protect:
        ok = function(
            ctypes.byref(input_blob),
            "NoTeaching-Robot local credential",
            ctypes.byref(entropy_blob),
            None,
            None,
            flags,
            ctypes.byref(output_blob),
        )
    else:
        ok = function(
            ctypes.byref(input_blob),
            None,
            ctypes.byref(entropy_blob),
            None,
            None,
            flags,
            ctypes.byref(output_blob),
        )
    del input_buffer, entropy_buffer
    if not ok:
        raise OSError(ctypes.get_last_error(), "Windows DPAPI operation failed")
    try:
        return ctypes.string_at(output_blob.pbData, output_blob.cbData)
    finally:
        if output_blob.pbData:
            ctypes.memset(output_blob.pbData, 0, output_blob.cbData)
            kernel32.LocalFree(output_blob.pbData)


def protect_sensitive_text(text: str, purpose: str) -> str:
    protected = _dpapi_crypt(text.encode("utf-8"), purpose, True)
    return DPAPI_PREFIX + base64.urlsafe_b64encode(protected).decode("ascii").rstrip("=")


def unprotect_sensitive_text(stored: str, purpose: str) -> str | None:
    if not stored.startswith(DPAPI_PREFIX) or not purpose.strip():
        return None
    try:
        encoded = stored[len(DPAPI_PREFIX):]
        protected = base64.b64decode(padded_base64(encoded), altchars=b"-_", validate=True)
        return _dpapi_crypt(protected, purpose, False).decode("utf-8")
    except (OSError, ValueError, UnicodeDecodeError):
        return None


def _read_dpapi_database_backup(path: Path) -> bytes:
    content = path.read_bytes()
    if not content.startswith(DPAPI_BACKUP_MAGIC):
        raise ValueError("Not a supported DPAPI ConfigStore backup")
    try:
        digest_line, encoded = content[len(DPAPI_BACKUP_MAGIC):].split(b"\n", 1)
        if not re.fullmatch(rb"[0-9a-f]{64}", digest_line):
            raise ValueError("Invalid backup digest")
        protected = base64.b64decode(encoded.strip(), altchars=b"-_", validate=True)
        database_bytes = _dpapi_crypt(protected, DPAPI_BACKUP_PURPOSE, False)
    except (OSError, ValueError) as exc:
        raise ValueError("Cannot decrypt or parse the DPAPI ConfigStore backup") from exc
    if not hmac.compare_digest(hashlib.sha256(database_bytes).hexdigest().encode("ascii"), digest_line):
        raise ValueError("DPAPI ConfigStore backup hash mismatch")
    if not database_bytes.startswith(b"SQLite format 3\x00"):
        raise ValueError("Decrypted backup is not a SQLite database")
    return database_bytes


def create_dpapi_database_backup(db_path: Path, backup_path: Path) -> Path:
    backup_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        source = sqlite3.connect(db_path)
        destination = sqlite3.connect(":memory:")
        try:
            source.backup(destination)
            if not hasattr(destination, "serialize"):
                raise RuntimeError("This Python SQLite build cannot serialize an in-memory backup")
            database_bytes = destination.serialize()
        finally:
            destination.close()
            source.close()
        digest = hashlib.sha256(database_bytes).hexdigest().encode("ascii")
        protected = _dpapi_crypt(database_bytes, DPAPI_BACKUP_PURPOSE, True)
        encoded = base64.urlsafe_b64encode(protected)
        _atomic_replace_bytes(backup_path, DPAPI_BACKUP_MAGIC + digest + b"\n" + encoded + b"\n")
        verified = _read_dpapi_database_backup(backup_path)
        if not hmac.compare_digest(hashlib.sha256(verified).digest(), hashlib.sha256(database_bytes).digest()):
            raise ValueError("DPAPI backup verification failed")
        return backup_path
    except Exception:
        backup_path.unlink(missing_ok=True)
        raise


def restore_dpapi_database_backup(backup_path: Path, db_path: Path, overwrite: bool) -> None:
    database_bytes = _read_dpapi_database_backup(backup_path)
    db_path.parent.mkdir(parents=True, exist_ok=True)
    if db_path.exists() and not overwrite:
        raise SystemExit(f"Target database already exists; use --overwrite to restore: {db_path}")
    if db_path.exists():
        timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        rollback = db_path.with_name(f"{db_path.name}.pre_restore_{timestamp}.dpapi.bak")
        create_dpapi_database_backup(db_path, rollback)
        print(f"Protected current database before restore: {rollback}")
    staging_fd, staging_name = tempfile.mkstemp(
        prefix=f".{db_path.name}.restore-",
        suffix=".tmp",
        dir=db_path.parent,
    )
    staging_path = Path(staging_name)
    try:
        with os.fdopen(staging_fd, "wb") as stream:
            stream.write(database_bytes)
            stream.flush()
            os.fsync(stream.fileno())
        connection = sqlite3.connect(staging_path)
        try:
            result = connection.execute("PRAGMA integrity_check").fetchone()
            if result != ("ok",):
                raise ValueError(f"Restored database failed integrity_check: {result}")
        finally:
            connection.close()
        os.replace(staging_path, db_path)
    except Exception:
        try:
            os.close(staging_fd)
        except OSError:
            pass
        staging_path.unlink(missing_ok=True)
        raise
    print(f"Restored DPAPI database backup: {db_path}")


def decode_stored_text(
    stored: str,
    encrypted: int | str | None,
    purpose: str = "",
) -> str | None:
    if stored.startswith(DPAPI_PREFIX):
        return unprotect_sensitive_text(stored, purpose)
    if stored.startswith("enc:v1:"):
        return unprotect_legacy_text(stored)
    if encrypted not in (None, 0, "0", False):
        return None
    return stored


def stored_text(text: str, encrypt: bool, sensitive: bool = False, purpose: str = "") -> tuple[str, int]:
    if sensitive:
        return protect_sensitive_text(text, purpose), 1
    if not encrypt:
        return text, 0
    return protect_legacy_text(text), 1


def create_current_tables(conn: sqlite3.Connection) -> None:
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS meta (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        )
        """
    )
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS settings (
            scope_type TEXT NOT NULL,
            scope_id TEXT NOT NULL DEFAULT '',
            module TEXT NOT NULL,
            key_name TEXT NOT NULL,
            value_text TEXT NOT NULL,
            value_type TEXT NOT NULL DEFAULT 'string',
            sensitive INTEGER NOT NULL DEFAULT 0,
            encrypted INTEGER NOT NULL DEFAULT 0,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            PRIMARY KEY(scope_type, scope_id, module, key_name)
        )
        """
    )
    conn.execute(
        """
        CREATE INDEX IF NOT EXISTS idx_settings_scope
            ON settings(scope_type, scope_id, module)
        """
    )


def set_schema_meta(
    conn: sqlite3.Connection,
    encrypt: bool,
    authentication_initialized: bool = True,
) -> None:
    now = _dt.datetime.now().isoformat(timespec="seconds")
    conn.execute("INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)", ("schema_version", SCHEMA_VERSION))
    conn.execute("INSERT OR IGNORE INTO meta(key, value) VALUES(?, ?)", ("created_at", now))
    conn.execute("INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)", ("encrypt_new_values", "1" if encrypt else "0"))
    conn.execute("INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)", ("sensitive_protection", "dpapi-current-user-v1"))
    conn.execute(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        ("auth_semantic_version", AUTH_SEMANTIC_VERSION),
    )
    conn.execute(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        ("auth_initialized", "1" if authentication_initialized else "0"),
    )


def init_schema(conn: sqlite3.Connection, encrypt: bool) -> None:
    conn.execute("PRAGMA secure_delete=ON")
    create_current_tables(conn)
    set_schema_meta(conn, encrypt)


def table_exists(conn: sqlite3.Connection, table_name: str) -> bool:
    row = conn.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name=? LIMIT 1",
        (table_name,),
    ).fetchone()
    return row is not None


def table_columns(conn: sqlite3.Connection, table_name: str) -> set[str]:
    return {row[1] for row in conn.execute(f"PRAGMA table_info({table_name})").fetchall()}


def has_current_schema(conn: sqlite3.Connection) -> bool:
    return table_exists(conn, "settings") and SETTINGS_REQUIRED_COLUMNS.issubset(table_columns(conn, "settings"))


def current_schema_version(conn: sqlite3.Connection) -> str:
    if not table_exists(conn, "meta"):
        return ""
    row = conn.execute("SELECT value FROM meta WHERE key='schema_version' LIMIT 1").fetchone()
    return str(row[0]).strip() if row and row[0] is not None else ""


def has_legacy_schema(conn: sqlite3.Connection) -> bool:
    required = {"file_path", "section_name", "key_name", "value_text"}
    return table_exists(conn, "ini_values") and required.issubset(table_columns(conn, "ini_values"))


def has_legacy_display_schema(conn: sqlite3.Connection) -> bool:
    required = {"source_path", "source_section", "source_key", "value_text"}
    return table_exists(conn, "ini_values") and required.issubset(table_columns(conn, "ini_values"))


def has_legacy_text_schema(conn: sqlite3.Connection) -> bool:
    if not table_exists(conn, "text_files"):
        return False
    columns = table_columns(conn, "text_files")
    return "content_text" in columns and ("source_path" in columns or "file_path" in columns)


def has_app_settings_schema(conn: sqlite3.Connection) -> bool:
    required = {"group_name", "key_name", "value_text"}
    return table_exists(conn, "app_settings") and required.issubset(table_columns(conn, "app_settings"))


def has_any_legacy_config_table(conn: sqlite3.Connection) -> bool:
    return has_legacy_schema(conn) or has_legacy_display_schema(conn) or has_legacy_text_schema(conn) or has_app_settings_schema(conn)


def drop_legacy_config_tables(conn: sqlite3.Connection) -> None:
    for table_name in ("ini_values", "text_files", "app_settings"):
        conn.execute(f"DROP TABLE IF EXISTS {table_name}")


def migrate_legacy_tables_to_settings(conn: sqlite3.Connection, encrypt: bool, overwrite: bool = False) -> tuple[int, int, int]:
    ini_count = 0
    text_count = 0
    app_setting_count = 0

    if table_exists(conn, "ini_values"):
        columns = table_columns(conn, "ini_values")
        encrypted_expr = "encrypted" if "encrypted" in columns else "0"
        if {"source_path", "source_section", "source_key", "value_text"}.issubset(columns):
            query = f"SELECT source_path, source_section, source_key, value_text, {encrypted_expr} FROM ini_values"
        elif {"file_path", "section_name", "key_name", "value_text"}.issubset(columns):
            query = f"SELECT file_path, section_name, key_name, value_text, {encrypted_expr} FROM ini_values"
        else:
            query = ""
        if query:
            for source_path, section, key, value, encrypted in conn.execute(query).fetchall():
                plain = decode_stored_text(str(value), encrypted)
                if plain is None:
                    raise ValueError("Cannot decode encrypted legacy INI value")
                ini_count += int(insert_ini_value(
                    conn,
                    str(source_path or ""),
                    str(section or ""),
                    str(key or ""),
                    plain,
                    encrypt,
                    overwrite=overwrite,
                ))

    if has_legacy_text_schema(conn):
        columns = table_columns(conn, "text_files")
        path_column = "source_path" if "source_path" in columns else "file_path"
        encrypted_expr = "encrypted" if "encrypted" in columns else "0"
        for source_path, content, encrypted in conn.execute(
            f"SELECT {path_column}, content_text, {encrypted_expr} FROM text_files"
        ).fetchall():
            plain = decode_stored_text(str(content), encrypted)
            if plain is None:
                raise ValueError("Cannot decode encrypted legacy text value")
            text_count += int(insert_text_file(
                conn,
                str(source_path or ""),
                plain,
                encrypt,
                overwrite=overwrite,
            ))

    if has_app_settings_schema(conn):
        columns = table_columns(conn, "app_settings")
        encrypted_expr = "encrypted" if "encrypted" in columns else "0"
        for module, key, value, encrypted in conn.execute(
            f"SELECT group_name, key_name, value_text, {encrypted_expr} FROM app_settings"
        ).fetchall():
            plain = decode_stored_text(str(value), encrypted)
            if plain is None:
                raise ValueError("Cannot decode encrypted legacy app setting")
            app_setting_count += int(insert_scoped_setting(
                conn,
                "global",
                "",
                str(module or ""),
                str(key or ""),
                plain,
                encrypt,
                overwrite=overwrite,
            ))

    return ini_count, text_count, app_setting_count


def insert_ini_value(
    conn: sqlite3.Connection,
    source_path: str,
    section: str,
    key: str,
    value: str,
    encrypt: bool,
    overwrite: bool = False,
) -> bool:
    identity = build_scoped_ini_identity(source_path, section, key)
    if not identity["valid"]:
        return False
    sensitive = bool(identity["sensitive"])
    purpose = protection_purpose(
        str(identity["scope_type"]), str(identity["scope_id"]),
        str(identity["module"]), str(identity["key_name"])
    )
    recoverable_secret = requires_dpapi_protection(
        str(identity["scope_type"]), str(identity["module"]), str(identity["key_name"]), sensitive
    )
    text, encrypted = stored_text(str(value), encrypt, recoverable_secret, purpose)
    command = "INSERT OR REPLACE" if overwrite else "INSERT OR IGNORE"
    cursor = conn.execute(
        f"""
        {command} INTO settings
            (scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))
        """,
        (
            identity["scope_type"],
            identity["scope_id"],
            identity["module"],
            identity["key_name"],
            text,
            identity["value_type"],
            1 if identity["sensitive"] else 0,
            encrypted,
        ),
    )
    return cursor.rowcount > 0


def insert_text_file(
    conn: sqlite3.Connection,
    source_path: str,
    content: str,
    encrypt: bool,
    overwrite: bool = False,
) -> bool:
    identity = build_scoped_file_identity(source_path)
    if not identity["valid"]:
        return False
    sensitive = bool(identity["sensitive"])
    purpose = protection_purpose(
        str(identity["scope_type"]), str(identity["scope_id"]),
        str(identity["module"]), str(identity["key_name"])
    )
    recoverable_secret = requires_dpapi_protection(
        str(identity["scope_type"]), str(identity["module"]), str(identity["key_name"]), sensitive
    )
    text, encrypted = stored_text(str(content), encrypt, recoverable_secret, purpose)
    command = "INSERT OR REPLACE" if overwrite else "INSERT OR IGNORE"
    cursor = conn.execute(
        f"""
        {command} INTO settings
            (scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))
        """,
        (
            identity["scope_type"],
            identity["scope_id"],
            identity["module"],
            identity["key_name"],
            text,
            identity["value_type"],
            1 if identity["sensitive"] else 0,
            encrypted,
        ),
    )
    return cursor.rowcount > 0


def insert_scoped_setting(
    conn: sqlite3.Connection,
    scope_type: str,
    scope_id: str,
    module: str,
    key: str,
    value: str,
    encrypt: bool,
    value_type: str = "string",
    overwrite: bool = False,
) -> bool:
    normalized_key = normalize_source_key(key)
    normalized_module = normalize_section(module)
    if not scope_type.strip() or not normalized_module or not normalized_key:
        return False
    sensitive = is_sensitive_setting_key(normalized_module) or is_sensitive_setting_key(normalized_key)
    purpose = protection_purpose(
        normalize_section(scope_type).lower(), scope_id.strip(),
        normalized_module, normalized_key
    )
    recoverable_secret = requires_dpapi_protection(
        normalize_section(scope_type).lower(), normalized_module, normalized_key, sensitive
    )
    text, encrypted = stored_text(
        str(value),
        encrypt and not is_portable_authentication_value(
            scope_type, normalized_module, normalized_key
        ),
        recoverable_secret,
        purpose,
    )
    command = "INSERT OR REPLACE" if overwrite else "INSERT OR IGNORE"
    cursor = conn.execute(
        f"""
        {command} INTO settings
            (scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))
        """,
        (
            normalize_section(scope_type).lower(),
            scope_id.strip(),
            normalized_module,
            normalized_key,
            text,
            value_type.strip().lower() or "string",
            1 if sensitive else 0,
            encrypted,
        ),
    )
    return cursor.rowcount > 0


def _is_safe_account_name(user_name: str) -> bool:
    return 3 <= len(user_name) <= 32 and re.fullmatch(r"[\w.\-]+", user_name, re.UNICODE) is not None


def _parse_supported_pbkdf2_password_record(
    record: str,
) -> tuple[int, bytes, bytes] | None:
    parts = record.split(":")
    if len(parts) != 5 or parts[:2] != ["pbkdf2-sha256", "v1"]:
        return None
    try:
        if re.fullmatch(r"[0-9]+", parts[2]) is None:
            return None
        iterations = int(parts[2])
        if (
            iterations < 100_000
            or iterations > 2_000_000
            or not parts[3]
            or not parts[4]
            or len(parts[3]) > 32
            or len(parts[4]) > 64
            or re.fullmatch(r"[A-Za-z0-9_-]+={0,2}", parts[3]) is None
            or re.fullmatch(r"[A-Za-z0-9_-]+={0,2}", parts[4]) is None
        ):
            return None
        salt = base64.b64decode(
            padded_base64(parts[3]), altchars=b"-_", validate=True
        )
        expected = base64.b64decode(
            padded_base64(parts[4]), altchars=b"-_", validate=True
        )
    except (ValueError, TypeError, UnicodeEncodeError, binascii.Error):
        return None
    if len(salt) != 16 or len(expected) != 32:
        return None
    return iterations, salt, expected


def _is_supported_password_record(record: str) -> bool:
    if record.startswith("pbkdf2-sha256:v1:"):
        return _parse_supported_pbkdf2_password_record(record) is not None
    return re.fullmatch(r"[0-9a-fA-F]{64}", record) is not None


def _is_known_bootstrap_password_record(user_name: str, record: str) -> bool:
    if user_name.casefold() != "admin":
        return False
    legacy = hashlib.sha256(b"admin\nadmin").hexdigest()
    if re.fullmatch(r"[0-9a-fA-F]{64}", record):
        return hmac.compare_digest(record.casefold(), legacy)
    parsed = _parse_supported_pbkdf2_password_record(record)
    if parsed is None:
        return False
    iterations, salt, expected = parsed
    derived = hashlib.pbkdf2_hmac("sha256", b"admin", salt, iterations, dklen=32)
    return hmac.compare_digest(derived, expected)


def _decoded_scoped_value(
    conn: sqlite3.Connection,
    scope_type: str,
    scope_id: str,
    module: str,
    key: str,
) -> str | None:
    row = conn.execute(
        """
        SELECT value_text, encrypted FROM settings
        WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?
        """,
        (normalize_section(scope_type).lower(), scope_id.strip(), normalize_section(module), normalize_source_key(key)),
    ).fetchone()
    if row is None:
        return None
    return decode_stored_text(
        str(row[0]), row[1], protection_purpose(scope_type, scope_id, module, key)
    )


def _parse_authentication_bool(value: str) -> bool | None:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    return None


def _normalize_existing_account_profiles(
    conn: sqlite3.Connection,
    encrypt: bool,
) -> tuple[int, int]:
    account_rows = conn.execute(
        """
        SELECT DISTINCT scope_id FROM settings
        WHERE scope_type='account' AND module='Profile'
        ORDER BY scope_id COLLATE NOCASE
        """
    ).fetchall()
    account_ids: list[str] = []
    folded_account_ids: dict[str, str] = {}
    for (raw_account_id,) in account_rows:
        account_id = str(raw_account_id)
        if account_id != account_id.strip() or not _is_safe_account_name(account_id):
            raise ValueError("Unsafe existing account/Profile id")
        folded = account_id.casefold()
        if folded in folded_account_ids and folded_account_ids[folded] != account_id:
            raise ValueError("Case-colliding account/Profile ids cannot be merged safely")
        folded_account_ids[folded] = account_id
        account_ids.append(account_id)

    administrator_count = 0
    for account_id in account_ids:
        stored_values = {
            str(key): (str(stored), encrypted)
            for key, stored, encrypted in conn.execute(
                """
                SELECT key_name, value_text, encrypted FROM settings
                WHERE scope_type='account' AND scope_id=? AND module='Profile'
                  AND key_name IN ('PasswordHash', 'Role', 'MustChangePassword')
                """,
                (account_id,),
            ).fetchall()
        }
        if "PasswordHash" not in stored_values or "Role" not in stored_values:
            raise ValueError("Existing account/Profile has no complete password/role record")

        decoded_values: dict[str, str] = {}
        for key in ("PasswordHash", "Role"):
            stored, encrypted = stored_values[key]
            decoded = decode_stored_text(
                stored,
                encrypted,
                protection_purpose("account", account_id, "Profile", key),
            )
            if decoded is None:
                raise ValueError(f"Cannot decode existing account/Profile {key}")
            decoded_values[key] = decoded

        password_record = decoded_values["PasswordHash"]
        role = decoded_values["Role"]
        if not _is_supported_password_record(password_record):
            raise ValueError("Existing account/Profile has an unsupported password record")
        if role not in {"operator", "engineer", "admin"}:
            raise ValueError("Existing account/Profile has an invalid role")
        if role == "admin":
            administrator_count += 1

        must_change_password = True
        if "MustChangePassword" in stored_values:
            stored, encrypted = stored_values["MustChangePassword"]
            decoded = decode_stored_text(
                stored,
                encrypted,
                protection_purpose(
                    "account", account_id, "Profile", "MustChangePassword"
                ),
            )
            if decoded is None:
                raise ValueError(
                    "Cannot decode existing account/Profile MustChangePassword"
                )
            parsed = _parse_authentication_bool(decoded)
            if parsed is None:
                raise ValueError(
                    "Existing account/Profile has an invalid MustChangePassword"
                )
            must_change_password = parsed
        if _is_known_bootstrap_password_record(account_id, password_record):
            must_change_password = True

        for key, value, value_type in (
            ("PasswordHash", password_record, "string"),
            ("Role", role, "string"),
            (
                "MustChangePassword",
                "1" if must_change_password else "0",
                "bool",
            ),
        ):
            insert_scoped_setting(
                conn,
                "account",
                account_id,
                "Profile",
                key,
                value,
                encrypt,
                value_type,
                overwrite=True,
            )
    return len(account_ids), administrator_count


def migrate_authentication_semantics(
    conn: sqlite3.Connection,
    encrypt: bool,
) -> tuple[int, int]:
    legacy_accounts: dict[str, dict[str, object]] = {}
    rows = conn.execute(
        """
        SELECT module, key_name, value_text, encrypted FROM settings
        WHERE scope_type='global' AND scope_id='' AND module LIKE 'Accounts/Users/%'
        """
    ).fetchall()
    for module, key, stored, encrypted in rows:
        module_text = str(module)
        user_name = module_text[len("Accounts/Users/"):].strip()
        if not _is_safe_account_name(user_name):
            raise ValueError("Unsafe legacy account name")
        decoded = decode_stored_text(
            str(stored), encrypted, protection_purpose("global", "", module_text, str(key))
        )
        if decoded is None:
            raise ValueError("Cannot decode a legacy account record")
        folded_user_name = user_name.casefold()
        if folded_user_name in legacy_accounts and legacy_accounts[folded_user_name]["user"] != user_name:
            raise ValueError("Case-colliding legacy account names cannot be merged safely")
        bucket = legacy_accounts.setdefault(folded_user_name, {"user": user_name, "values": {}})
        values = bucket["values"]
        assert isinstance(values, dict)
        values[str(key)] = decoded

    for bucket in legacy_accounts.values():
        user_name = str(bucket["user"])
        values = bucket["values"]
        assert isinstance(values, dict)
        source_hash = str(values.get("PasswordHash", ""))
        source_role = str(values.get("Role", ""))
        if not source_hash or source_role not in {"operator", "engineer", "admin"}:
            raise ValueError("Legacy account has no complete password/role record")
        destination_hash = _decoded_scoped_value(conn, "account", user_name, "Profile", "PasswordHash")
        replace_destination = destination_hash is None
        if (
            destination_hash is not None
            and _is_known_bootstrap_password_record(user_name, destination_hash)
        ):
            replace_destination = True
        if replace_destination:
            conn.execute(
                "DELETE FROM settings WHERE scope_type='account' AND scope_id=? AND module='Profile'",
                (user_name,),
            )
            for key in ("PasswordHash", "Role", "CreatedAt", "UpdatedAt"):
                if key not in values:
                    continue
                insert_scoped_setting(
                    conn, "account", user_name, "Profile", key, str(values[key]),
                    encrypt, "datetime" if key.endswith("At") else "string", overwrite=True,
                )
            destination_hash = source_hash
            is_bootstrap = (
                destination_hash is not None
                and _is_known_bootstrap_password_record(user_name, destination_hash)
            )
            insert_scoped_setting(
                conn, "account", user_name, "Profile", "MustChangePassword",
                "1" if is_bootstrap else "0", encrypt, "bool", overwrite=True,
            )
        elif _decoded_scoped_value(
            conn, "account", user_name, "Profile", "MustChangePassword"
        ) is None:
            insert_scoped_setting(
                conn, "account", user_name, "Profile", "MustChangePassword",
                "1", encrypt, "bool", overwrite=True,
            )

    account_counts = _normalize_existing_account_profiles(conn, encrypt)

    for key, stored, encrypted in conn.execute(
        """
        SELECT key_name, value_text, encrypted FROM settings
        WHERE scope_type='global' AND scope_id='' AND module='LoginState/General'
        """
    ).fetchall():
        key_text = str(key)
        if key_text not in {"UserName", "AccountHistory"}:
            continue
        decoded = decode_stored_text(
            str(stored), encrypted,
            protection_purpose("global", "", "LoginState/General", key_text),
        )
        if decoded is None:
            raise ValueError("Cannot decode legacy login state")
        insert_scoped_setting(
            conn, "global", "", "LoginState", key_text, decoded,
            encrypt, "list" if key_text == "AccountHistory" else "string", overwrite=True,
        )

    conn.execute(
        """
        DELETE FROM settings WHERE scope_type='global' AND scope_id='' AND (
            module LIKE 'Accounts/Users/%' OR module='LoginState/General' OR
            module LIKE 'LoginState/SavedPasswords%' OR
            module LIKE 'LoginState/RememberedCredentials%' OR
            lower(key_name)='passwordbase64'
        )
        """
    )
    insert_scoped_setting(conn, "global", "", "LoginState", "RememberPassword", "0", encrypt, "bool", True)
    insert_scoped_setting(conn, "global", "", "LoginState", "AutoLogin", "0", encrypt, "bool", True)

    upgrades: list[tuple[str, str, str, str, str, str]] = []
    for scope_type, scope_id, module, key, stored, value_type, sensitive, encrypted in conn.execute(
        """
        SELECT scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted
        FROM settings
        """
    ).fetchall():
        semantic_sensitive = bool(sensitive) or is_sensitive_setting_key(str(module)) or is_sensitive_setting_key(str(key))
        recoverable_secret = requires_dpapi_protection(
            str(scope_type), str(module), str(key), semantic_sensitive
        )
        if not recoverable_secret:
            continue
        purpose = protection_purpose(str(scope_type), str(scope_id), str(module), str(key))
        decoded = decode_stored_text(str(stored), encrypted, purpose)
        if decoded is None:
            raise ValueError("Cannot decode a sensitive setting during v5 migration")
        if str(stored).startswith(DPAPI_PREFIX):
            continue
        upgrades.append((str(scope_type), str(scope_id), str(module), str(key), decoded, str(value_type)))
    for scope_type, scope_id, module, key, value, value_type in upgrades:
        insert_scoped_setting(
            conn, scope_type, scope_id, module, key, value, encrypt, value_type, overwrite=True
        )
    return account_counts


def read_ini_value(conn: sqlite3.Connection, source_path: str, section: str, key: str) -> str | None:
    identity = build_scoped_ini_identity(source_path, section, key)
    if not identity["valid"]:
        return None
    row = conn.execute(
        """
        SELECT value_text, encrypted FROM settings
        WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?
        """,
        (
            identity["scope_type"],
            identity["scope_id"],
            identity["module"],
            identity["key_name"],
        ),
    ).fetchone()
    if row is None:
        return None
    return decode_stored_text(
        row[0], row[1],
        protection_purpose(
            str(identity["scope_type"]), str(identity["scope_id"]),
            str(identity["module"]), str(identity["key_name"])
        ),
    )


def parse_int(value: str | None, fallback: int) -> int:
    if value is None:
        return fallback
    try:
        return int(float(value.strip()))
    except (TypeError, ValueError):
        return fallback


def discover_robot_names(conn: sqlite3.Connection, data_dir: Path) -> list[str]:
    names: set[str] = set()
    if data_dir.exists():
        for child in data_dir.iterdir():
            if child.is_dir() and child.name.lower().startswith("robot"):
                names.add(child.name)

    if has_current_schema(conn):
        for (scope_id,) in conn.execute("SELECT DISTINCT scope_id FROM settings WHERE scope_type='robot' AND scope_id<>''"):
            if scope_id:
                names.add(str(scope_id))

    unit_count = parse_int(read_ini_value(conn, "Data/ContralUnitInfo.ini", "UnitNum", "UnitNum"), 0)
    for index in range(max(0, unit_count)):
        value = read_ini_value(conn, "Data/ContralUnitInfo.ini", "UnitName", f"Unit{index}")
        if value and value.strip():
            names.add(value.strip())

    if has_current_schema(conn):
        for _key, stored, encrypted in conn.execute(
            """
            SELECT key_name, value_text, encrypted
            FROM settings
            WHERE scope_type='global' AND scope_id='' AND module='ControlUnits/UnitName'
            """
        ):
            decoded = decode_stored_text(
                stored,
                encrypted,
                protection_purpose("global", "", "ControlUnits/UnitName", _key),
            )
            if decoded and decoded.strip():
                names.add(decoded.strip())

    return sorted(names, key=str.lower)


def ensure_measure_weld_defaults(
    conn: sqlite3.Connection,
    robot_names: list[str],
    encrypt: bool,
) -> int:
    inserted = 0
    for robot_name in robot_names:
        source_path = f"Data/{robot_name}/MeasureWeldParam.ini"
        inserted += int(insert_ini_value(conn, source_path, "MeasureWeldGroups", "GroupCount", "1", encrypt))
        inserted += int(insert_ini_value(conn, source_path, "MeasureWeldGroups", "UseGroupNo", "0", encrypt))
        group_count = parse_int(read_ini_value(conn, source_path, "MeasureWeldGroups", "GroupCount"), 1)
        group_count = max(1, group_count)
        for group_index in range(group_count):
            inserted += int(insert_ini_value(
                conn,
                source_path,
                "MeasureWeldGroups",
                f"Group{group_index}Name",
                f"参数组{group_index + 1}",
                encrypt,
            ))
            scan_section = f"MeasureGroup{group_index}.Scan"
            weld_section = f"MeasureGroup{group_index}.Weld"
            for key, value in MEASURE_WELD_SCAN_DEFAULTS:
                inserted += int(insert_ini_value(conn, source_path, scan_section, key, value, encrypt))
            for key, value in MEASURE_WELD_WELD_DEFAULTS:
                inserted += int(insert_ini_value(conn, source_path, weld_section, key, value, encrypt))
    return inserted


def ensure_point_cloud_defaults(conn: sqlite3.Connection, data_dir: Path, encrypt: bool) -> int:
    project_root = data_dir.resolve().parent
    library_dir = project_root / "SDK" / "PointCloudExtration"
    config_path = library_dir / "config" / "CorrugatedSheetPointCloudEctration.ini"
    defaults = (
        ("General", "ProcessingMode", "LegacyLaserPath"),
        ("External", "LibraryDir", str(library_dir)),
        ("External", "ConfigPath", str(config_path)),
        ("External", "ZTruncationValue", "6.0"),
        ("External", "ResampleStepMm", "2.0"),
        ("External", "FallbackToLegacy", "1"),
    )
    inserted = 0
    for section, key, value in defaults:
        inserted += int(insert_ini_value(conn, "Data/PointCloudProcessing.ini", section, key, value, encrypt))
    return inserted


def ensure_pose_comp_defaults(
    conn: sqlite3.Connection,
    robot_names: list[str],
    encrypt: bool,
) -> int:
    inserted = 0
    for robot_name in robot_names:
        source_path = f"Data/{robot_name}/WeldPoseCompParam.ini"
        count = parse_int(read_ini_value(conn, source_path, "ALLWeldPoseComp", "PoseCompCount"), COMP_SEGMENT_COUNT)
        group_count = parse_int(read_ini_value(conn, source_path, "ALLWeldPoseComp", "PoseCompGroupCount"), 0)
        if group_count <= 0:
            group_count = max(1, (max(0, count) + COMP_SEGMENT_COUNT - 1) // COMP_SEGMENT_COUNT)
        expected_count = max(COMP_SEGMENT_COUNT, group_count * COMP_SEGMENT_COUNT)
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldPoseComp", "PoseCompCount", str(expected_count), encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldPoseComp", "PoseCompGroupCount", str(group_count), encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldPoseComp", "ActivePoseCompGroupIndex", "0", encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldPoseComp", "PoseCompMatchMode", POSE_COMP_MATCH_BY_POSE, encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldPoseComp", "PoseMatchMaxErrorDeg", "5.0", encrypt))

        for group_index in range(group_count):
            group_section = f"WeldPoseCompGroup{group_index}"
            group_name = f"姿态补偿组{group_index + 1}"
            inserted += int(insert_ini_value(conn, source_path, group_section, "Name", group_name, encrypt))
            inserted += int(insert_ini_value(conn, source_path, group_section, "PoseCompMatchMode", POSE_COMP_MATCH_BY_POSE, encrypt))
            for segment_index, (name, segment_kind) in enumerate(POSE_SEGMENT_TYPES):
                flat_index = group_index * COMP_SEGMENT_COUNT + segment_index
                section = f"WeldPoseComp{flat_index}"
                defaults = (
                    ("GroupIndex", str(group_index)),
                    ("GroupName", group_name),
                    ("Name", name),
                    ("SegmentKind", segment_kind),
                    ("Rx", "0.000000"),
                    ("Ry", "0.000000"),
                    ("Rz", "0.000000"),
                    ("CompX", "0.000000"),
                    ("CompY", "0.000000"),
                    ("CompZ", "0.000000"),
                )
                for key, value in defaults:
                    inserted += int(insert_ini_value(conn, source_path, section, key, value, encrypt))
    return inserted


def ensure_seam_comp_defaults(
    conn: sqlite3.Connection,
    robot_names: list[str],
    encrypt: bool,
) -> int:
    inserted = 0
    for robot_name in robot_names:
        source_path = f"Data/{robot_name}/WeldSeamCompParam.ini"
        count = parse_int(read_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompCount"), COMP_SEGMENT_COUNT)
        group_count = parse_int(read_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompGroupCount"), 0)
        if group_count <= 0:
            group_count = max(1, (max(0, count) + COMP_SEGMENT_COUNT - 1) // COMP_SEGMENT_COUNT)
        expected_count = max(COMP_SEGMENT_COUNT, group_count * COMP_SEGMENT_COUNT)
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompCount", str(expected_count), encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompGroupCount", str(group_count), encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "ActiveSeamCompGroupIndex", "0", encrypt))

        for group_index in range(group_count):
            group_section = f"WeldSeamCompGroup{group_index}"
            group_name = f"焊道补偿组{group_index + 1}"
            inserted += int(insert_ini_value(conn, source_path, group_section, "Name", group_name, encrypt))
            for segment_index, (name, segment_kind) in enumerate(SEAM_SEGMENT_TYPES):
                flat_index = group_index * COMP_SEGMENT_COUNT + segment_index
                section = f"WeldSeamComp{flat_index}"
                defaults = (
                    ("GroupIndex", str(group_index)),
                    ("GroupName", group_name),
                    ("Name", name),
                    ("SegmentKind", segment_kind),
                    ("WeldZComp", "0.000000"),
                    ("WeldGunDirComp", "0.000000"),
                    ("WeldSeamDirComp", "0.000000"),
                )
                for key, value in defaults:
                    inserted += int(insert_ini_value(conn, source_path, section, key, value, encrypt))
    return inserted


def ensure_runtime_defaults(conn: sqlite3.Connection, data_dir: Path, encrypt: bool) -> tuple[int, dict[str, int]]:
    robot_names = discover_robot_names(conn, data_dir)
    details = {
        "measure_weld": ensure_measure_weld_defaults(conn, robot_names, encrypt),
        "point_cloud": ensure_point_cloud_defaults(conn, data_dir, encrypt),
        "pose_comp": ensure_pose_comp_defaults(conn, robot_names, encrypt),
        "seam_comp": ensure_seam_comp_defaults(conn, robot_names, encrypt),
    }
    return sum(details.values()), details


def set_legacy_credential_scrub_pending(
    connection: sqlite3.Connection,
    manifest: list[dict[str, object]] | None,
) -> None:
    if manifest is None:
        return
    connection.execute(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        (SCRUB_STATE_KEY, "pending"),
    )
    connection.execute(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        (SCRUB_MANIFEST_KEY, serialize_legacy_credential_scrub_manifest(manifest)),
    )


def read_legacy_credential_scrub_provenance(
    db_path: Path,
) -> tuple[str, list[dict[str, object]] | None]:
    connection = sqlite3.connect(db_path)
    try:
        if current_schema_version(connection) != SCHEMA_VERSION or not has_current_schema(connection):
            return "", None
        rows = dict(connection.execute(
            "SELECT key, value FROM meta WHERE key IN (?, ?)",
            (SCRUB_STATE_KEY, SCRUB_MANIFEST_KEY),
        ).fetchall())
    finally:
        connection.close()
    state = str(rows.get(SCRUB_STATE_KEY, ""))
    manifest_text = rows.get(SCRUB_MANIFEST_KEY)
    if state not in {"pending", "complete"} or not isinstance(manifest_text, str):
        return "", None
    return state, parse_legacy_credential_scrub_manifest(manifest_text)


def mark_legacy_credential_scrub_complete(db_path: Path) -> None:
    connection = sqlite3.connect(db_path)
    try:
        connection.execute("BEGIN IMMEDIATE")
        cursor = connection.execute(
            "UPDATE meta SET value='complete' WHERE key=? AND value='pending'",
            (SCRUB_STATE_KEY,),
        )
        if cursor.rowcount != 1:
            raise ValueError("Legacy credential scrub provenance is no longer pending")
        connection.commit()
    except Exception:
        connection.rollback()
        raise
    finally:
        connection.close()


def enforce_no_plaintext_configstore_residue(
    data_dir: Path,
    forced_encoding: str | None,
) -> None:
    plaintext_candidates = sorted({
        path.resolve()
        for pattern in (
            "ConfigStore.db.bak*",
            "ConfigStore.db-wal",
            "ConfigStore.db-shm",
            "ConfigStore.db-journal",
            ".ConfigStore.db.backup-snapshot-*.tmp",
            ".ConfigStore.db.restore-*.tmp",
            ".ConfigStore.db.staging-*.tmp",
        )
        for path in data_dir.glob(pattern)
        if path.is_file() and not path.name.casefold().endswith(".dpapi.bak")
    })
    plaintext_candidates.extend(
        path.resolve()
        for path in data_dir.rglob("*")
        if path.is_file()
        and ".ini" in path.name.casefold()
        and path.suffix.casefold() != ".ini"
        and _sanitized_legacy_ini_bytes(path, forced_encoding)[0] != path.read_bytes()
    )
    plaintext_candidates = sorted(set(plaintext_candidates))
    if not plaintext_candidates:
        return
    print("SECURITY GATE: pre-existing plaintext database backup/journal candidates require review:")
    for candidate in plaintext_candidates:
        print(f"  {candidate}")
    print("Rotate affected credentials before release; this tool does not claim SSD/VSS secure erasure.")
    raise ValueError("Plaintext ConfigStore backup/journal candidates block release")


def _finish_legacy_credential_scrub(
    data_dir: Path,
    db_path: Path,
    forced_encoding: str | None,
    scrub_requested: bool,
) -> None:
    in_place = (
        db_path.name.casefold() == "configstore.db"
        and data_dir.resolve() == db_path.parent.resolve()
    )
    if not scrub_requested:
        if in_place and any(".ini" in path.name.casefold() for path in data_dir.rglob("*")):
            print(
                "SECURITY NOTICE: in-place legacy INI credentials were retained because "
                "--scrub-legacy-credentials was not requested."
            )
        return
    if not in_place:
        print(
            "SECURITY NOTICE: legacy source credentials were not modified because source and target "
            "Data directories differ. Restrict the source and rotate affected credentials."
        )
        return
    state, manifest = read_legacy_credential_scrub_provenance(db_path)
    if state == "complete":
        remaining = prepare_legacy_ini_credential_scrub(data_dir, forced_encoding)
        if remaining:
            raise ValueError(
                "Legacy credentials appeared after the proven migration scrub; refusing to delete unproven source data"
            )
        enforce_no_plaintext_configstore_residue(data_dir, forced_encoding)
        print("Legacy credential scrub was already completed and remains clean.")
        return
    if state != "pending" or manifest is None:
        raise ValueError(
            "Current v5 database has no pending legacy-migration scrub provenance; refusing to delete source credentials"
        )
    modified_files, removed_values = apply_legacy_ini_credential_scrub(
        data_dir, forced_encoding, manifest
    )
    remaining = prepare_legacy_ini_credential_scrub(data_dir, forced_encoding)
    if remaining:
        raise ValueError(
            "Unproven or changed legacy credential files remain after the migration scrub"
        )
    enforce_no_plaintext_configstore_residue(data_dir, forced_encoding)
    mark_legacy_credential_scrub_complete(db_path)
    print(
        f"Legacy credential scrub completed: files={modified_files}, "
        f"removed credential values={removed_values}"
    )


def migrate_existing_database_to_current(
    db_path: Path,
    data_dir: Path,
    encrypt_new_values: bool,
    scrub_manifest: list[dict[str, object]] | None = None,
    forced_encoding: str | None = None,
) -> bool:
    if not db_path.exists():
        return False

    with sqlite3.connect(db_path) as conn:
        has_legacy_tables = has_any_legacy_config_table(conn)
        has_settings_schema = has_current_schema(conn)
        has_meta_table = table_exists(conn, "meta")
        version = current_schema_version(conn)
        if has_meta_table and not version:
            raise SystemExit("ConfigStore meta table has no schema_version; refusing to guess")
        if version not in {"", "4", SCHEMA_VERSION}:
            raise SystemExit(f"Unsupported future or unknown ConfigStore schema: {version}")
        if version == SCHEMA_VERSION and not has_settings_schema:
            raise SystemExit("Current ConfigStore schema metadata has incompatible settings columns")
        if version == "4" and not (has_settings_schema or has_legacy_tables):
            raise SystemExit("ConfigStore schema v4 has neither current nor legacy configuration tables")
        authentication_initialized = True
        authentication_semantic_version = ""
        if has_meta_table:
            semantic_row = conn.execute(
                "SELECT value FROM meta WHERE key='auth_semantic_version' LIMIT 1"
            ).fetchone()
            if semantic_row and semantic_row[0] is not None:
                authentication_semantic_version = str(semantic_row[0])
        if version == SCHEMA_VERSION:
            if authentication_semantic_version not in {"1", AUTH_SEMANTIC_VERSION}:
                raise SystemExit(
                    "Unsupported or missing current-v5 authentication semantic version"
                )
            initialized_row = conn.execute(
                "SELECT value FROM meta WHERE key='auth_initialized' LIMIT 1"
            ).fetchone()
            initialized_value = (
                str(initialized_row[0])
                if initialized_row and initialized_row[0] is not None
                else ""
            )
            if initialized_value not in {"0", "1"}:
                raise SystemExit(
                    "Current-v5 authentication initialization marker is invalid"
                )
            authentication_initialized = initialized_value == "1"
        has_outdated_current_schema = version == "4"
        needs_authentication_semantic_upgrade = (
            version == SCHEMA_VERSION
            and authentication_semantic_version != AUTH_SEMANTIC_VERSION
        )
        needs_migration = (
            has_legacy_tables
            or has_outdated_current_schema
            or needs_authentication_semantic_upgrade
        )
        if not needs_migration:
            return False
        if version == SCHEMA_VERSION and scrub_manifest is not None:
            provenance_rows = dict(conn.execute(
                "SELECT key, value FROM meta WHERE key IN (?, ?)",
                (SCRUB_STATE_KEY, SCRUB_MANIFEST_KEY),
            ).fetchall())
            existing_scrub_state = str(
                provenance_rows.get(SCRUB_STATE_KEY, "")
            )
            existing_manifest_text = provenance_rows.get(SCRUB_MANIFEST_KEY)
            if (
                existing_scrub_state not in {"pending", "complete"}
                or not isinstance(existing_manifest_text, str)
            ):
                raise SystemExit(
                    "Current-v5 authentication upgrade has no valid legacy scrub provenance"
                )
            parse_legacy_credential_scrub_manifest(existing_manifest_text)
            if existing_scrub_state == "complete":
                if scrub_manifest:
                    raise ValueError(
                        "Legacy credentials appeared after the proven migration scrub; "
                        "refusing to delete unproven source data"
                    )
                enforce_no_plaintext_configstore_residue(
                    data_dir, forced_encoding
                )
        legacy_text_names = {name.casefold() for name in TEXT_FILE_NAMES}
        has_disk_inputs = any(
            path.is_file()
            and (
                ".ini" in path.name.casefold()
                or path.name.casefold() in legacy_text_names
            )
            for path in data_dir.rglob("*")
        )
        if version != SCHEMA_VERSION and has_disk_inputs:
            raise SystemExit(
                "Existing database upgrade does not import legacy INI/TXT files. "
                "Refusing to ignore or scrub them; use an explicit --overwrite migration after reviewing the protected backup."
            )

    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = db_path.with_name(
        f"{db_path.name}.bak_v{SCHEMA_VERSION}_{timestamp}.dpapi.bak"
    )
    create_dpapi_database_backup(db_path, backup)

    with sqlite3.connect(db_path) as conn:
        try:
            conn.execute("PRAGMA secure_delete=ON")
            conn.execute("BEGIN")
            create_current_tables(conn)
            legacy_counts = migrate_legacy_tables_to_settings(conn, encrypt_new_values)
            migrate_authentication_semantics(conn, encrypt_new_values)
            default_count, default_details = ensure_runtime_defaults(conn, data_dir, encrypt_new_values)
            drop_legacy_config_tables(conn)
            set_schema_meta(
                conn,
                encrypt_new_values,
                authentication_initialized=authentication_initialized,
            )
            if version != SCHEMA_VERSION:
                set_legacy_credential_scrub_pending(conn, scrub_manifest)
            conn.commit()
        except Exception:
            conn.rollback()
            raise

    print(f"Upgraded existing database to schema v{SCHEMA_VERSION}: {db_path}")
    print(f"Created and verified DPAPI-protected database backup: {backup}")
    print(f"Legacy rows migrated: INI={legacy_counts[0]}, text={legacy_counts[1]}, app_settings={legacy_counts[2]}")
    print(f"Runtime defaults added: {default_count} ({default_details})")
    return True


def migrate(
    data_dir: Path,
    db_path: Path,
    overwrite: bool,
    encrypt: bool,
    forced_encoding: str | None,
    allow_mojibake: bool,
    scrub_legacy_credentials: bool = False,
) -> None:
    data_dir = data_dir.resolve()
    db_path = db_path.resolve()
    if not data_dir.exists():
        raise SystemExit(f"Data directory not found: {data_dir}")
    in_place_scrub = (
        scrub_legacy_credentials
        and db_path.name.casefold() == "configstore.db"
        and data_dir == db_path.parent
    )
    scrub_manifest = prepare_legacy_ini_credential_scrub(data_dir, forced_encoding) \
        if in_place_scrub else None

    if db_path.exists():
        if not overwrite:
            if migrate_existing_database_to_current(
                db_path,
                data_dir,
                encrypt,
                scrub_manifest,
                forced_encoding,
            ):
                _finish_legacy_credential_scrub(
                    data_dir, db_path, forced_encoding, scrub_legacy_credentials
                )
                return
            if in_place_scrub:
                _finish_legacy_credential_scrub(
                    data_dir, db_path, forced_encoding, True
                )
                return
            raise SystemExit(
                "Target database already exists and was not changed. "
                f"To replace it from legacy Data, rerun with --overwrite (a backup is created first): {db_path}"
            )
        timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = db_path.with_name(
            f"{db_path.name}.bak_overwrite_{timestamp}.dpapi.bak"
        )
        create_dpapi_database_backup(db_path, backup)
        print(f"Created and verified DPAPI-protected database backup: {backup}")

    db_path.parent.mkdir(parents=True, exist_ok=True)
    staging_fd, staging_name = tempfile.mkstemp(
        prefix=f".{db_path.name}.staging-",
        suffix=".tmp",
        dir=db_path.parent,
    )
    os.close(staging_fd)
    staging_path = Path(staging_name)
    ini_count = 0
    text_count = 0
    value_count = 0
    default_replacements: list[tuple[str, str, str, str, str]] = []

    try:
        conn = sqlite3.connect(staging_path)
        try:
            init_schema(conn, encrypt)

            for path in sorted(data_dir.rglob("*.ini")):
                file_path = normalize_data_path(path, data_dir)
                decoded_text, encoding = decode_file(path, forced_encoding)
                rows = parse_ini(decoded_text)
                rows, replacements = replace_mojibake_ini_rows(data_dir, path, rows, encoding, allow_mojibake)
                default_replacements.extend((file_path, section, key, old_value, new_value)
                                            for section, key, old_value, new_value in replacements)
                check_ini_rows(path, rows, encoding, allow_mojibake)
                for section, key, value in rows:
                    value_count += int(insert_ini_value(conn, file_path, section, key, value, encrypt, overwrite=True))
                ini_count += 1

            for path in sorted(data_dir.rglob("*.txt")):
                if path.name not in TEXT_FILE_NAMES:
                    continue
                file_path = normalize_data_path(path, data_dir)
                content = normalize_text(sanitize_legacy_text_file(path, forced_encoding, allow_mojibake))
                text_count += int(insert_text_file(conn, file_path, content, encrypt, overwrite=True))

            account_count, administrator_count = migrate_authentication_semantics(
                conn, encrypt
            )
            if account_count > 0 and administrator_count == 0:
                raise ValueError(
                    "Migrated account/Profile data has no valid administrator"
                )
            authentication_initialized = account_count > 0
            default_count, default_details = ensure_runtime_defaults(conn, data_dir, encrypt)
            drop_legacy_config_tables(conn)
            set_schema_meta(
                conn,
                encrypt,
                authentication_initialized=authentication_initialized,
            )
            set_legacy_credential_scrub_pending(conn, scrub_manifest)
            conn.commit()
        finally:
            conn.close()
        os.replace(staging_path, db_path)
    except Exception:
        staging_path.unlink(missing_ok=True)
        raise

    print(f"Created: {db_path}")
    print(f"INI files: {ini_count}, values: {value_count}")
    print(f"Text files: {text_count}")
    print(f"Runtime defaults added: {default_count} ({default_details})")
    print(f"Encrypted: {'yes' if encrypt else 'no'}")
    _finish_legacy_credential_scrub(
        data_dir, db_path, forced_encoding, scrub_legacy_credentials
    )
    if default_replacements:
        print("Mojibake values replaced with defaults:")
        for file_path, section, key, old_value, new_value in default_replacements:
            print(f"  {file_path} [{section}] {key}: {old_value!r} -> {new_value!r}")


def parent_process_name() -> str:
    if os.name != "nt":
        return ""
    try:
        import ctypes
        from ctypes import wintypes

        class ProcessEntry32W(ctypes.Structure):
            _fields_ = [
                ("dwSize", wintypes.DWORD),
                ("cntUsage", wintypes.DWORD),
                ("th32ProcessID", wintypes.DWORD),
                ("th32DefaultHeapID", ctypes.c_void_p),
                ("th32ModuleID", wintypes.DWORD),
                ("cntThreads", wintypes.DWORD),
                ("th32ParentProcessID", wintypes.DWORD),
                ("pcPriClassBase", wintypes.LONG),
                ("dwFlags", wintypes.DWORD),
                ("szExeFile", wintypes.WCHAR * 260),
            ]

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        snapshot = kernel32.CreateToolhelp32Snapshot(0x00000002, 0)
        if snapshot == ctypes.c_void_p(-1).value:
            return ""

        processes: dict[int, tuple[int, str]] = {}
        entry = ProcessEntry32W()
        entry.dwSize = ctypes.sizeof(ProcessEntry32W)
        try:
            if kernel32.Process32FirstW(snapshot, ctypes.byref(entry)):
                while True:
                    processes[int(entry.th32ProcessID)] = (
                        int(entry.th32ParentProcessID),
                        str(entry.szExeFile),
                    )
                    if not kernel32.Process32NextW(snapshot, ctypes.byref(entry)):
                        break
        finally:
            kernel32.CloseHandle(snapshot)

        parent_id = processes.get(os.getpid(), (0, ""))[0]
        return processes.get(parent_id, (0, ""))[1]
    except Exception:
        return ""


def launched_from_explorer() -> bool:
    return parent_process_name().lower() == "explorer.exe"


def pause_console() -> None:
    try:
        input("\n按回车键关闭窗口...")
    except EOFError:
        pass


def current_source_sha256() -> str:
    if getattr(sys, "frozen", False):
        source_path = Path(getattr(sys, "_MEIPASS")) / "_migrator_source" / "migrate_config_to_sqlite.py"
    else:
        source_path = Path(__file__)
    if not source_path.is_file():
        raise RuntimeError(f"Embedded migration source is missing: {source_path}")
    return hashlib.sha256(source_path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description="Migrate Data config files to ConfigStore.db")
    parser.add_argument("--source", default="Data", help="Data directory to migrate")
    parser.add_argument("--db", default=None, help="Output SQLite database path")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing database")
    parser.add_argument("--encrypt", action="store_true", help="Encrypt stored values")
    parser.add_argument(
        "--scrub-legacy-credentials",
        action="store_true",
        help="After a successful in-place migration, atomically remove recoverable credentials from legacy INI files",
    )
    parser.add_argument(
        "--restore-dpapi-backup",
        type=Path,
        default=None,
        help="Restore a ConfigStore DPAPI backup to --db and exit",
    )
    parser.add_argument(
        "--encoding",
        default=None,
        help="Force legacy file encoding, for example gbk, utf-8, or utf-16le",
    )
    parser.add_argument(
        "--allow-mojibake",
        action="store_true",
        help="Continue even if suspicious garbled text markers are found",
    )
    parser.add_argument("--pause", action="store_true", help="Keep the console window open after finishing")
    parser.add_argument(
        "--print-source-sha256",
        action="store_true",
        help="Print the SHA256 of the source embedded in this executable and exit",
    )
    args = parser.parse_args()

    if args.print_source_sha256:
        print(current_source_sha256())
        return

    should_pause = args.pause or launched_from_explorer()
    exit_code = 0
    try:
        source = Path(args.source)
        db = Path(args.db) if args.db else source / "ConfigStore.db"
        if args.restore_dpapi_backup is not None:
            print(f"Resolved DPAPI backup: {args.restore_dpapi_backup.resolve()}")
            print(f"Resolved restore target: {db.resolve()}")
            restore_dpapi_database_backup(
                args.restore_dpapi_backup.resolve(), db.resolve(), args.overwrite
            )
            return
        print(f"Resolved source Data: {source.resolve()}")
        print(f"Resolved target database: {db.resolve()}")
        migrate(
            source,
            db,
            args.overwrite,
            args.encrypt,
            args.encoding,
            args.allow_mojibake,
            args.scrub_legacy_credentials,
        )
    except SystemExit as exc:
        if exc.code not in (None, 0):
            print(exc.code)
            exit_code = exc.code if isinstance(exc.code, int) else 1
    except Exception as exc:
        print(f"Migration failed: {exc}")
        exit_code = 1
    finally:
        if should_pause:
            pause_console()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
