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
from contextlib import closing
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
INSTALLER_STAGING_NAME_RE = re.compile(
    r"\.ConfigStore\.db\.install-(create|upgrade)-([0-9a-f]{32})\.tmp"
)
RUNTIME_ONLY_INI_NAMES = {
    # This is an SDK runtime input, not a source of ConfigStore settings.  It
    # legitimately lives beside Data in deployed installations and must not
    # force an otherwise self-contained schema upgrade into --overwrite.
    "corrugatedsheetpointcloudectration.ini",
}
RUNTIME_POINT_CLOUD_KEYS = {
    b"ifupright",
    b"Plate_thickness",
    b"Remove_Floor_ZValue",
    b"Thread_Number",
    b"Move_distance",
    b"DEBUGLOG",
    b"LOGPATH",
    b"Plane_Threshold",
    b"Merge_Lines_Angle_Threshold",
    b"Lines_Dis_Threshold",
    b"Merge_Lines_Dis_Threshold",
    b"ClusterTolerance",
    b"if_Cluster",
    b"Discrete_Value",
    b"Dilate_Value",
    b"Erode_Value",
    b"Line_Length",
    b"X_AXIS",
    b"Y_AXIS",
    b"Z_AXIS",
    b"Save_File_Name",
    b"Step",
    b"is_remove_noise",
    b"is_sample",
    b"sample_size",
    b"above_z",
}

# MoveFileExW flags used for durable same-volume replacement on Windows.  The
# write-through flag is part of the credential-scrub durability contract: a
# successful return must not merely mean that the rename is still cached.
MOVEFILE_REPLACE_EXISTING = 0x00000001
MOVEFILE_WRITE_THROUGH = 0x00000008
LEGACY_ACCOUNT_MODULE = "Accounts/Users"
LEGACY_ACCOUNT_MODULE_PREFIX = LEGACY_ACCOUNT_MODULE + "/"
LEGACY_ACCOUNT_PROFILE_FIELDS = {
    "PasswordHash",
    "Role",
    "CreatedAt",
    "UpdatedAt",
    "MustChangePassword",
}
LEGACY_ACCOUNT_TIME_FIELDS = {"CreatedAt", "UpdatedAt"}
ACCOUNT_PROFILE_TIME_FIELDS = LEGACY_ACCOUNT_TIME_FIELDS | {"PasswordChangedAt"}
ACCOUNT_PROFILE_PORTABLE_FIELDS = {
    "PasswordHash",
    "Role",
    "MustChangePassword",
} | ACCOUNT_PROFILE_TIME_FIELDS
# Retain the existing name for the canonical-field/case checks.  Every
# authentication Profile field is also a portable field; none may be tied to
# the Windows account that happened to run a migration or verification.
ACCOUNT_PROFILE_SECURITY_FIELDS = ACCOUNT_PROFILE_PORTABLE_FIELDS
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


def _replace_file_write_through(source: Path, destination: Path) -> None:
    if os.name != "nt":
        os.replace(source, destination)
        return

    move_file_ex = ctypes.WinDLL(
        "kernel32", use_last_error=True
    ).MoveFileExW
    move_file_ex.argtypes = (
        wintypes.LPCWSTR,
        wintypes.LPCWSTR,
        wintypes.DWORD,
    )
    move_file_ex.restype = wintypes.BOOL
    flags = MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH
    source_text = os.path.abspath(os.fspath(source))
    destination_text = os.path.abspath(os.fspath(destination))
    if not move_file_ex(source_text, destination_text, flags):
        error = ctypes.get_last_error()
        raise OSError(
            error,
            f"MoveFileExW write-through replacement failed: "
            f"{ctypes.FormatError(error)}",
            destination_text,
        )


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
        _replace_file_write_through(temporary_path, path)
    except Exception:
        try:
            os.close(file_descriptor)
        except OSError:
            pass
        temporary_path.unlink(missing_ok=True)
        raise


def _atomic_create_bytes(path: Path, content: bytes) -> None:
    """Create a file exclusively and never replace a pre-existing path."""
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_BINARY"):
        flags |= os.O_BINARY
    file_descriptor = os.open(path, flags, 0o600)
    created = True
    try:
        with os.fdopen(file_descriptor, "wb") as stream:
            file_descriptor = -1
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        if file_descriptor >= 0:
            os.close(file_descriptor)
        if created:
            path.unlink(missing_ok=True)
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
    plans: list[tuple[Path, bytes, bytes, int]] = []
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
        plans.append((path, current, sanitized, removed))

    # Validate every source and every expected output before changing the first
    # file.  Cross-file replacement is not an OS transaction, so retain each
    # original and roll back already-written files if an unexpected I/O error
    # occurs during the commit pass.
    committed: list[tuple[Path, bytes]] = []
    try:
        for path, original, sanitized, removed in plans:
            _atomic_replace_bytes(path, sanitized)
            committed.append((path, original))
            modified_files += 1
            removed_values += removed
        for path, _original, sanitized, _removed in plans:
            if not hmac.compare_digest(
                hashlib.sha256(path.read_bytes()).digest(),
                hashlib.sha256(sanitized).digest(),
            ):
                raise ValueError(
                    f"Legacy credential scrub did not persist expected bytes: {path}"
                )
    except Exception as exc:
        rollback_failures: list[str] = []
        for path, original in reversed(committed):
            try:
                _atomic_replace_bytes(path, original)
            except Exception as rollback_exc:
                rollback_failures.append(f"{path}: {rollback_exc}")
        if rollback_failures:
            raise RuntimeError(
                "Legacy credential scrub failed and rollback was incomplete: "
                + "; ".join(rollback_failures)
            ) from exc
        raise
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
        and normalize_source_key(key).casefold()
        in {field.casefold() for field in ACCOUNT_PROFILE_PORTABLE_FIELDS}
    )


def is_login_preference(scope_type: str, module: str, key: str) -> bool:
    return (
        normalize_section(scope_type).casefold() == "global"
        and normalize_section(module).casefold() == "loginstate"
        and normalize_source_key(key).casefold()
        in {"rememberpassword", "autologin"}
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


def is_compatible_disabled_login_preference(
    scope_type: str,
    scope_id: str,
    module: str,
    key: str,
    stored: str,
    value_type: str,
    sensitive: object,
    encrypted: object,
) -> bool:
    # Released schema-v5/auth-v2 builds wrote a few exact boolean preference
    # shapes as plaintext; these values are not credentials.  Keep the
    # exception read-only and shape-exact.  Current writers still use the
    # recoverable-secret DPAPI policy, and all near-miss shapes fail closed.
    if (
        scope_type != "global"
        or scope_id != ""
        or module != "LoginState"
        or key not in {"RememberPassword", "AutoLogin"}
        or encrypted not in (0, "0", False)
    ):
        return False
    marker_is_sensitive = sensitive in (1, "1", True)
    marker_is_nonsensitive = sensitive in (0, "0", False)
    python_cleared_shape = (
        stored == "0"
        and value_type == "bool"
        and marker_is_sensitive
    )
    if python_cleared_shape:
        return True
    if key == "RememberPassword":
        return (
            stored == "0"
            and value_type == "string"
            and marker_is_sensitive
        )
    return (
        stored in {"0", "1"}
        and value_type == "string"
        and marker_is_nonsensitive
    )


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
    # CryptProtectData accepts cbData == 0, but Windows still requires pbData
    # to address valid storage.  A null pointer makes protection of an empty
    # sensitive value fail with ERROR_INVALID_PARAMETER on affected systems.
    # Keep the one-byte buffer alive and pass it as a non-null sentinel while
    # retaining the truthful zero byte count.
    pointer = ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte))
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
    backup_created = False
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
        _atomic_create_bytes(
            backup_path,
            DPAPI_BACKUP_MAGIC + digest + b"\n" + encoded + b"\n",
        )
        backup_created = True
        verified = _read_dpapi_database_backup(backup_path)
        if not hmac.compare_digest(hashlib.sha256(verified).digest(), hashlib.sha256(database_bytes).digest()):
            raise ValueError("DPAPI backup verification failed")
        return backup_path
    except Exception:
        if backup_created:
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
        """
        SELECT 1 FROM sqlite_master
        WHERE type='table' AND lower(name)=lower(?) LIMIT 1
        """,
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
    sensitive = (
        is_sensitive_setting_key(normalized_module)
        or is_sensitive_setting_key(normalized_key)
        or is_login_preference(scope_type, normalized_module, normalized_key)
    )
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


def _require_portable_account_profile_storage(
    stored: str,
    encrypted: object,
    field: str,
    context: str,
) -> None:
    if field not in ACCOUNT_PROFILE_PORTABLE_FIELDS:
        raise ValueError("Unsupported portable account/Profile field")
    if (
        encrypted not in (0, "0", False)
        or stored.startswith(DPAPI_PREFIX)
        or stored.startswith("enc:v1:")
    ):
        raise ValueError(
            f"{context} account/Profile {field} must use portable plaintext storage"
        )


def _account_iso_instant(value: str, field: str) -> _dt.datetime:
    if field not in ACCOUNT_PROFILE_TIME_FIELDS:
        raise ValueError("Unsupported account timestamp field")
    if (
        value != value.strip()
        or len(value) > 64
        or re.fullmatch(
            r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}"
            r"(?:\.\d{1,6})?(?:Z|[+-]\d{2}:\d{2})?",
            value,
        ) is None
    ):
        raise ValueError(f"Legacy account {field} is not a strict ISO timestamp")
    try:
        parsed = _dt.datetime.fromisoformat(
            value[:-1] + "+00:00" if value.endswith("Z") else value
        )
    except ValueError as exc:
        raise ValueError(
            f"Legacy account {field} is not a valid ISO timestamp"
        ) from exc
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=_dt.timezone.utc)
    offset = parsed.utcoffset()
    if offset is None:
        raise ValueError(f"Legacy account {field} has an invalid timezone")
    return parsed.astimezone(_dt.timezone.utc)


def _merge_account_timestamps(field: str, values: list[str]) -> str | None:
    if not values:
        return None
    parsed = [(_account_iso_instant(value, field), value) for value in values]
    if field == "CreatedAt":
        return min(parsed, key=lambda item: (item[0], item[1]))[1]
    if field == "UpdatedAt":
        return max(parsed, key=lambda item: (item[0], item[1]))[1]
    raise ValueError("Unsupported account timestamp merge field")


def _account_profile_ids(conn: sqlite3.Connection) -> list[str]:
    rows = conn.execute(
        """
        SELECT DISTINCT scope_type, scope_id, module FROM settings
        WHERE lower(trim(scope_type))='account'
          AND lower(trim(module))='profile'
        ORDER BY scope_id COLLATE NOCASE
        """
    ).fetchall()
    account_ids: list[str] = []
    folded_account_ids: dict[str, str] = {}
    seen_account_ids: set[str] = set()
    for raw_scope_type, raw_account_id, raw_module in rows:
        scope_type = str(raw_scope_type)
        account_id = str(raw_account_id)
        module = str(raw_module)
        if scope_type != "account" or module != "Profile":
            raise ValueError(
                "Account profile scope_type/module has non-canonical spelling"
            )
        if account_id != account_id.strip() or not _is_safe_account_name(account_id):
            raise ValueError("Unsafe existing account/Profile id")
        folded = account_id.casefold()
        if folded in folded_account_ids and folded_account_ids[folded] != account_id:
            raise ValueError("Case-colliding account/Profile ids cannot be merged safely")
        folded_account_ids[folded] = account_id
        if account_id not in seen_account_ids:
            account_ids.append(account_id)
            seen_account_ids.add(account_id)
    return account_ids


def _normalize_existing_account_profiles(
    conn: sqlite3.Connection,
    encrypt: bool,
) -> tuple[int, int]:
    account_ids = _account_profile_ids(conn)

    administrator_count = 0
    for account_id in account_ids:
        stored_values: dict[str, tuple[str, object]] = {}
        for key, stored, encrypted in conn.execute(
            """
            SELECT key_name, value_text, encrypted FROM settings
            WHERE scope_type='account' AND scope_id=? AND module='Profile'
            """,
            (account_id,),
        ).fetchall():
            key_text = str(key)
            canonical_key = next(
                (
                    candidate
                    for candidate in ACCOUNT_PROFILE_SECURITY_FIELDS
                    if candidate.casefold() == key_text.casefold()
                ),
                None,
            )
            if canonical_key is None:
                continue
            if key_text != canonical_key:
                raise ValueError(
                    "Existing account/Profile has a case-variant authentication field"
                )
            _require_portable_account_profile_storage(
                str(stored), encrypted, key_text, "Existing"
            )
            if key_text in {
                "PasswordHash", "Role", "MustChangePassword",
                "CreatedAt", "UpdatedAt", "PasswordChangedAt",
            }:
                stored_values[key_text] = (str(stored), encrypted)
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

        for time_field in ("CreatedAt", "UpdatedAt", "PasswordChangedAt"):
            if time_field not in stored_values:
                continue
            stored, encrypted = stored_values[time_field]
            decoded = decode_stored_text(
                stored,
                encrypted,
                protection_purpose(
                    "account", account_id, "Profile", time_field
                ),
            )
            if decoded is None:
                raise ValueError(
                    f"Cannot decode existing account/Profile {time_field}"
                )
            _account_iso_instant(decoded, time_field)

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


def _validate_legacy_authentication_module_spelling(
    conn: sqlite3.Connection,
) -> None:
    account_base = LEGACY_ACCOUNT_MODULE
    login_general = "LoginState/General"
    saved_passwords = "LoginState/SavedPasswords"
    remembered_credentials = "LoginState/RememberedCredentials"
    rows = conn.execute(
        """
        SELECT DISTINCT module FROM settings
        WHERE scope_type='global' AND scope_id='' AND (
            lower(module)=lower(?) OR lower(module) LIKE lower(?) OR
            lower(module)=lower(?) OR lower(module) LIKE lower(?) OR
            lower(module) LIKE lower(?)
        )
        """,
        (
            account_base,
            LEGACY_ACCOUNT_MODULE_PREFIX + "%",
            login_general,
            saved_passwords + "%",
            remembered_credentials + "%",
        ),
    ).fetchall()
    for (raw_module,) in rows:
        module = str(raw_module)
        folded = module.casefold()
        canonical = False
        if folded == account_base.casefold():
            canonical = module == account_base
        elif folded.startswith(LEGACY_ACCOUNT_MODULE_PREFIX.casefold()):
            canonical = module.startswith(LEGACY_ACCOUNT_MODULE_PREFIX)
        elif folded == login_general.casefold():
            canonical = module == login_general
        elif folded.startswith(saved_passwords.casefold()):
            canonical = module.startswith(saved_passwords)
        elif folded.startswith(remembered_credentials.casefold()):
            canonical = module.startswith(remembered_credentials)
        if not canonical:
            raise ValueError(
                "Legacy authentication module has non-canonical casing"
            )


def migrate_authentication_semantics(
    conn: sqlite3.Connection,
    encrypt: bool,
) -> tuple[int, int]:
    _validate_legacy_authentication_module_spelling(conn)
    legacy_accounts: dict[str, dict[str, object]] = {}
    rows = conn.execute(
        """
        SELECT module, key_name, value_text, encrypted FROM settings
        WHERE scope_type='global' AND scope_id='' AND (
            lower(module)=lower(?) OR lower(module) LIKE lower(?)
        )
        """
        , (LEGACY_ACCOUNT_MODULE, LEGACY_ACCOUNT_MODULE_PREFIX + "%")
    ).fetchall()
    for module, key, stored, encrypted in rows:
        module_text = str(module)
        key_text = str(key)
        if module_text == LEGACY_ACCOUNT_MODULE:
            key_parts = key_text.split("/")
            if len(key_parts) != 2:
                raise ValueError("Malformed flat legacy account key")
            user_name, profile_key = key_parts
        elif module_text.startswith(LEGACY_ACCOUNT_MODULE_PREFIX):
            user_name = module_text[len(LEGACY_ACCOUNT_MODULE_PREFIX):]
            profile_key = key_text
        else:
            raise ValueError("Unexpected legacy account module")
        if (
            user_name != user_name.strip()
            or profile_key != profile_key.strip()
            or not _is_safe_account_name(user_name)
        ):
            raise ValueError("Unsafe legacy account name")
        if profile_key not in LEGACY_ACCOUNT_PROFILE_FIELDS:
            raise ValueError("Unsupported legacy account profile field")
        decoded = decode_stored_text(
            str(stored), encrypted,
            protection_purpose("global", "", module_text, key_text),
        )
        if decoded is None:
            raise ValueError("Cannot decode a legacy account record")
        folded_user_name = user_name.casefold()
        if folded_user_name in legacy_accounts and legacy_accounts[folded_user_name]["user"] != user_name:
            raise ValueError("Case-colliding legacy account names cannot be merged safely")
        bucket = legacy_accounts.setdefault(folded_user_name, {"user": user_name, "values": {}})
        values = bucket["values"]
        assert isinstance(values, dict)
        if profile_key in LEGACY_ACCOUNT_TIME_FIELDS:
            candidates = [decoded]
            if profile_key in values:
                candidates.append(str(values[profile_key]))
            merged_timestamp = _merge_account_timestamps(profile_key, candidates)
            assert merged_timestamp is not None
            values[profile_key] = merged_timestamp
        else:
            if profile_key in values and values[profile_key] != decoded:
                raise ValueError("Conflicting duplicate legacy account profile field")
            values[profile_key] = decoded

    existing_account_ids = {
        account_id.casefold(): account_id
        for account_id in _account_profile_ids(conn)
    }

    for bucket in legacy_accounts.values():
        user_name = str(bucket["user"])
        values = bucket["values"]
        assert isinstance(values, dict)
        source_hash = str(values.get("PasswordHash", ""))
        source_role = str(values.get("Role", ""))
        if (
            not _is_supported_password_record(source_hash)
            or source_role not in {"operator", "engineer", "admin"}
        ):
            raise ValueError("Legacy account has no complete password/role record")
        source_must_change_explicit = "MustChangePassword" in values
        source_must_change = _is_known_bootstrap_password_record(user_name, source_hash)
        if source_must_change_explicit:
            parsed = _parse_authentication_bool(str(values["MustChangePassword"]))
            if parsed is None:
                raise ValueError("Legacy account has an invalid MustChangePassword")
            source_must_change = parsed or source_must_change

        folded_user_name = user_name.casefold()
        existing_account_id = existing_account_ids.get(folded_user_name)
        if existing_account_id is not None and existing_account_id != user_name:
            raise ValueError("Legacy account conflicts with a differently-cased target account")

        destination_values: dict[str, str] = {}
        destination_rows = conn.execute(
            """
            SELECT key_name, value_text, encrypted FROM settings
            WHERE scope_type='account' AND scope_id=? AND module='Profile'
            """,
            (user_name,),
        ).fetchall()
        for destination_key, destination_stored, destination_encrypted in destination_rows:
            destination_key_text = str(destination_key)
            canonical_destination_key = next(
                (
                    candidate
                    for candidate in ACCOUNT_PROFILE_SECURITY_FIELDS
                    if candidate.casefold() == destination_key_text.casefold()
                ),
                None,
            )
            if canonical_destination_key is None:
                continue
            if destination_key_text != canonical_destination_key:
                raise ValueError(
                    "Target account/Profile has a case-variant authentication field"
                )
            _require_portable_account_profile_storage(
                str(destination_stored), destination_encrypted,
                destination_key_text, "Target",
            )
            decoded_destination = decode_stored_text(
                str(destination_stored),
                destination_encrypted,
                protection_purpose(
                    "account", user_name, "Profile", destination_key_text
                ),
            )
            if decoded_destination is None:
                raise ValueError("Cannot decode a target account/Profile field")
            destination_values[destination_key_text] = decoded_destination

        destination_exists = bool(destination_rows)
        destination_hash = destination_values.get("PasswordHash")
        destination_role = destination_values.get("Role")
        merged_time_values: dict[str, str] = {}
        for time_field in ("CreatedAt", "UpdatedAt"):
            time_candidates: list[str] = []
            if time_field in values:
                time_candidates.append(str(values[time_field]))
            if time_field in destination_values:
                time_candidates.append(destination_values[time_field])
            merged_timestamp = _merge_account_timestamps(
                time_field, time_candidates
            )
            if merged_timestamp is not None:
                merged_time_values[time_field] = merged_timestamp
        replace_destination = not destination_exists
        if destination_exists:
            if destination_hash is None or destination_role is None:
                raise ValueError("Legacy account conflicts with an incomplete target profile")
            if (
                not _is_supported_password_record(destination_hash)
                or destination_role not in {"operator", "engineer", "admin"}
            ):
                raise ValueError("Legacy account conflicts with an invalid target profile")
            if (
                destination_role == "admin"
                and _is_known_bootstrap_password_record(user_name, destination_hash)
            ):
                replace_destination = True
            elif destination_hash != source_hash or destination_role != source_role:
                raise ValueError("Legacy account conflicts with an existing target profile")
            else:
                if "MustChangePassword" in destination_values:
                    parsed_destination = _parse_authentication_bool(
                        destination_values["MustChangePassword"]
                    )
                    if parsed_destination is None:
                        raise ValueError(
                            "Legacy account conflicts with an invalid target MustChangePassword"
                        )
                    if (
                        source_must_change_explicit
                        and parsed_destination != source_must_change
                    ):
                        raise ValueError(
                            "Legacy account conflicts with target MustChangePassword"
                        )
        if replace_destination:
            conn.execute(
                "DELETE FROM settings WHERE scope_type='account' AND scope_id=? AND module='Profile'",
                (user_name,),
            )
            for key in ("PasswordHash", "Role"):
                insert_scoped_setting(
                    conn, "account", user_name, "Profile", key, str(values[key]),
                    encrypt, "string", overwrite=True,
                )
            for key, timestamp in merged_time_values.items():
                insert_scoped_setting(
                    conn, "account", user_name, "Profile", key, timestamp,
                    encrypt, "datetime", overwrite=True,
                )
            insert_scoped_setting(
                conn, "account", user_name, "Profile", "MustChangePassword",
                "1" if source_must_change else "0", encrypt, "bool", overwrite=True,
            )
        else:
            for key, timestamp in merged_time_values.items():
                if destination_values.get(key) != timestamp:
                    insert_scoped_setting(
                        conn, "account", user_name, "Profile", key,
                        timestamp, encrypt, "datetime", overwrite=True,
                    )
            if "MustChangePassword" not in destination_values:
                missing_destination_must_change = (
                    source_must_change
                    if source_must_change_explicit
                    else True
                )
                insert_scoped_setting(
                    conn, "account", user_name, "Profile", "MustChangePassword",
                    "1" if missing_destination_must_change else "0",
                    encrypt, "bool", overwrite=True,
                )

        existing_account_ids[folded_user_name] = user_name

    account_counts = _normalize_existing_account_profiles(conn, encrypt)

    for login_module, key, stored, encrypted in conn.execute(
        """
        SELECT module, key_name, value_text, encrypted FROM settings
        WHERE scope_type='global' AND scope_id=''
          AND lower(module)=lower('LoginState/General')
        """
    ).fetchall():
        if str(login_module) != "LoginState/General":
            raise ValueError("Legacy login-state module has non-canonical casing")
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
            lower(module)=lower(?) OR lower(module) LIKE lower(?) OR
            lower(module)=lower('LoginState/General') OR
            lower(module) LIKE lower('LoginState/SavedPasswords%') OR
            lower(module) LIKE lower('LoginState/RememberedCredentials%') OR
            lower(key_name)='passwordbase64'
        )
        """,
        (LEGACY_ACCOUNT_MODULE, LEGACY_ACCOUNT_MODULE_PREFIX + "%"),
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
        semantic_sensitive = (
            bool(sensitive)
            or is_sensitive_setting_key(str(module))
            or is_sensitive_setting_key(str(key))
            or is_login_preference(str(scope_type), str(module), str(key))
        )
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
    enforce_no_plaintext_configstore_residue(data_dir, forced_encoding)
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


def _is_runtime_only_ini(path: Path, data_dir: Path) -> bool:
    if path.name.casefold() not in RUNTIME_ONLY_INI_NAMES:
        return False
    try:
        link_status = path.lstat()
    except OSError:
        return False
    if path.is_symlink() or (
        getattr(link_status, "st_file_attributes", 0) & 0x400
    ):
        return False
    if path.resolve().parent.as_posix().casefold() != data_dir.resolve().as_posix().casefold():
        return False
    try:
        size = path.stat().st_size
        if size <= 0 or size > 128 * 1024:
            return False
        content = path.read_bytes()
    except OSError:
        return False
    if b"\0" in content:
        return False

    observed_keys: set[bytes] = set()
    has_runtime_setting = False
    for line in content.split(b"\n"):
        line = line.strip()
        if line.startswith(b"\xef\xbb\xbf"):
            line = line[3:].strip()
        if not line or line.startswith((b"//", b"#", b";")):
            continue
        if line.startswith(b"["):
            return False
        equals = line.find(b"=")
        if equals <= 0:
            return False
        key = line[:equals].strip()
        folded_key = key.lower()
        if key not in RUNTIME_POINT_CLOUD_KEYS or folded_key in observed_keys:
            return False
        observed_keys.add(folded_key)
        has_runtime_setting = True
    return has_runtime_setting


def _validated_upgrade_backup_path(
    db_path: Path,
    backup_path: Path,
    require_absent: bool = True,
) -> Path:
    if not backup_path.is_absolute():
        raise ValueError("--upgrade-backup must be an absolute path")
    resolved_db = db_path.resolve()
    resolved_backup = backup_path.resolve()
    if resolved_backup.parent != resolved_db.parent:
        raise ValueError("--upgrade-backup must be beside the target database")
    backup_name = resolved_backup.name.casefold()
    if (
        not backup_name.startswith(resolved_db.name.casefold() + ".")
        or not backup_name.endswith(".dpapi.bak")
    ):
        raise ValueError(
            "--upgrade-backup must be named after the target database and end in .dpapi.bak"
        )
    if resolved_backup == resolved_db:
        raise ValueError("--upgrade-backup cannot be the target database")
    if require_absent and resolved_backup.exists():
        raise FileExistsError(f"Upgrade backup already exists: {resolved_backup}")
    return resolved_backup


def _path_is_reparse_point(path: Path) -> bool:
    try:
        status = path.lstat()
    except OSError:
        return False
    return path.is_symlink() or bool(
        getattr(status, "st_file_attributes", 0) & 0x400
    )


def _absolute_lexical_path(path: Path) -> Path:
    # Path.resolve() follows reparse points.  Installer-path policy must inspect
    # the caller-provided directory/file identity before resolving it.
    return Path(os.path.abspath(os.fspath(path)))


def _same_lexical_path(left: Path, right: Path) -> bool:
    return os.path.normcase(os.fspath(left)) == os.path.normcase(os.fspath(right))


def _file_sha256_digest(path: Path) -> bytes:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.digest()


def _installer_staging_sidecars(db_path: Path) -> tuple[Path, Path, Path]:
    return tuple(Path(os.fspath(db_path) + suffix) for suffix in ("-journal", "-wal", "-shm"))


def _reject_installer_staging_sidecars(db_path: Path) -> None:
    for sidecar in _installer_staging_sidecars(db_path):
        if os.path.lexists(sidecar):
            raise ValueError(
                f"Installer staging database has an unsafe SQLite sidecar: {sidecar.name}"
            )


def _validate_installer_staging_target(
    data_dir: Path,
    db_path: Path,
) -> tuple[Path, Path, str]:
    lexical_data = _absolute_lexical_path(data_dir)
    lexical_db = _absolute_lexical_path(db_path)
    if not lexical_data.is_dir() or _path_is_reparse_point(lexical_data):
        raise ValueError("--installer-staging requires a non-reparse Data directory")
    if not _same_lexical_path(lexical_db.parent, lexical_data):
        raise ValueError("--installer-staging target must be directly inside Data")

    match = INSTALLER_STAGING_NAME_RE.fullmatch(lexical_db.name)
    if match is None:
        raise ValueError(
            "--installer-staging target name must be "
            ".ConfigStore.db.install-(create|upgrade)-<32 lowercase hex>.tmp"
        )
    mode = match.group(1)
    target_exists = os.path.lexists(lexical_db)
    if target_exists and (
        _path_is_reparse_point(lexical_db) or not lexical_db.is_file()
    ):
        raise ValueError("--installer-staging target must be a non-reparse regular file")
    if mode == "create" and target_exists:
        raise FileExistsError(f"Installer create staging target already exists: {lexical_db}")
    if mode == "upgrade" and not target_exists:
        raise FileNotFoundError(f"Installer upgrade staging target is missing: {lexical_db}")

    resolved_data = lexical_data.resolve(strict=True)
    if _path_is_reparse_point(resolved_data):
        raise ValueError("--installer-staging resolved Data directory is a reparse point")
    resolved_db = resolved_data / lexical_db.name
    if mode == "upgrade" and lexical_db.resolve(strict=True) != resolved_db:
        raise ValueError("--installer-staging target escapes the real Data directory")

    final_db = resolved_data / "ConfigStore.db"
    final_exists = os.path.lexists(final_db)
    if mode == "create":
        if final_exists:
            raise FileExistsError(
                "Installer create staging requires the final ConfigStore.db to be absent"
            )
    else:
        if (
            not final_exists
            or _path_is_reparse_point(final_db)
            or not final_db.is_file()
        ):
            raise FileNotFoundError(
                "Installer upgrade staging requires a regular final ConfigStore.db"
            )
        if (
            final_db.stat().st_size != resolved_db.stat().st_size
            or not hmac.compare_digest(
                _file_sha256_digest(final_db), _file_sha256_digest(resolved_db)
            )
        ):
            raise ValueError(
                "Installer upgrade staging is not a byte-identical copy of ConfigStore.db"
            )
    _reject_installer_staging_sidecars(resolved_db)
    return resolved_data, resolved_db, mode


def _validate_installer_verification_paths(
    data_dir: Path,
    db_path: Path,
) -> tuple[Path, Path]:
    lexical_data = _absolute_lexical_path(data_dir)
    lexical_db = _absolute_lexical_path(db_path)
    if not lexical_data.is_dir() or _path_is_reparse_point(lexical_data):
        raise ValueError(
            "--verify-installer-state requires a non-reparse Data directory"
        )
    if not _same_lexical_path(lexical_db.parent, lexical_data):
        raise ValueError(
            "--verify-installer-state database must be directly inside Data"
        )
    if (
        lexical_db.name != "ConfigStore.db"
        and INSTALLER_STAGING_NAME_RE.fullmatch(lexical_db.name) is None
    ):
        raise ValueError(
            "--verify-installer-state database name is not canonical"
        )
    if (
        not lexical_db.is_file()
        or _path_is_reparse_point(lexical_db)
    ):
        raise ValueError(
            "--verify-installer-state database must be a non-reparse regular file"
        )

    resolved_data = lexical_data.resolve(strict=True)
    resolved_db = lexical_db.resolve(strict=True)
    if resolved_db != resolved_data / lexical_db.name:
        raise ValueError(
            "--verify-installer-state database escapes the Data directory"
        )
    _reject_installer_staging_sidecars(resolved_db)
    return resolved_data, resolved_db


def _read_only_data_inventory(
    data_dir: Path,
) -> tuple[tuple[str, int, str], ...]:
    root = data_dir.resolve(strict=True)
    paths = sorted(
        root.rglob("*"),
        key=lambda path: path.relative_to(root).as_posix().casefold(),
    )
    inventory: list[tuple[str, int, str]] = []
    for path in paths:
        relative = path.relative_to(root).as_posix()
        if _path_is_reparse_point(path):
            raise ValueError(
                f"Installer Data inventory contains a reparse point: {relative}"
            )
        if path.is_dir():
            continue
        if not path.is_file():
            raise ValueError(
                f"Installer Data inventory contains a non-regular entry: {relative}"
            )
        resolved = path.resolve(strict=True)
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ValueError(
                f"Installer Data inventory entry escapes Data: {relative}"
            ) from exc
        inventory.append(
            (relative, resolved.stat().st_size, _file_sha256_digest(resolved).hex())
        )
    return tuple(inventory)


def _reserve_installer_create_target(db_path: Path) -> None:
    flags = os.O_RDWR | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_BINARY"):
        flags |= os.O_BINARY
    descriptor = os.open(db_path, flags, 0o600)
    os.close(descriptor)


def _canonical_scrub_manifest_map(
    manifest: list[dict[str, object]],
) -> dict[str, tuple[str, dict[str, object]]]:
    entries: dict[str, tuple[str, dict[str, object]]] = {}
    for item in manifest:
        path_text = str(item["path"])
        relative = Path(path_text)
        if (
            not path_text
            or "\\" in path_text
            or relative.is_absolute()
            or any(part in {"", ".", ".."} for part in relative.parts)
            or relative.as_posix() != path_text
        ):
            raise ValueError("Legacy credential scrub manifest path is not canonical")
        identity = path_text.casefold()
        if identity in entries:
            raise ValueError("Legacy credential scrub manifest has duplicate paths")
        entries[identity] = (path_text, item)
    return entries


def _validate_installer_scrub_provenance(
    data_dir: Path,
    db_path: Path,
    forced_encoding: str | None,
    fresh_manifest: list[dict[str, object]],
) -> str:
    state, stored_manifest = read_legacy_credential_scrub_provenance(db_path)
    if state not in {"pending", "complete"} or stored_manifest is None:
        raise ValueError(
            "Installer staging database has no valid credential scrub provenance"
        )

    root = data_dir.resolve()
    stored_entries = _canonical_scrub_manifest_map(stored_manifest)
    fresh_entries = _canonical_scrub_manifest_map(fresh_manifest)
    for identity, (_path_text, fresh_item) in fresh_entries.items():
        stored = stored_entries.get(identity)
        if stored is None or fresh_item != stored[1]:
            raise ValueError(
                "Legacy credentials appeared outside the stored scrub provenance"
            )

    expected_fresh: set[str] = set()
    for identity, (path_text, item) in stored_entries.items():
        relative = Path(path_text)
        candidate = root.joinpath(*relative.parts)
        if (
            not candidate.is_file()
            or _path_is_reparse_point(candidate)
        ):
            raise ValueError(
                f"Stored legacy credential source is missing or unsafe: {path_text}"
            )
        resolved = candidate.resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ValueError(
                "Stored legacy credential source escapes the Data directory"
            ) from exc

        current_hash = _file_sha256_digest(resolved).hex()
        before_hash = str(item["before_sha256"])
        after_hash = str(item["after_sha256"])
        if hmac.compare_digest(before_hash, after_hash):
            raise ValueError("Stored legacy credential scrub transition is empty")
        is_before = hmac.compare_digest(current_hash, before_hash)
        is_after = hmac.compare_digest(current_hash, after_hash)
        if state == "complete":
            if not is_after:
                raise ValueError(
                    f"Completed legacy credential source changed: {path_text}"
                )
            continue
        if not (is_before or is_after):
            raise ValueError(
                f"Pending legacy credential source changed: {path_text}"
            )
        if is_before:
            sanitized, removed_values = _sanitized_legacy_ini_bytes(
                resolved, forced_encoding
            )
            if (
                not hmac.compare_digest(
                    hashlib.sha256(sanitized).hexdigest(), after_hash
                )
                or removed_values != int(item["removed_values"])
            ):
                raise ValueError(
                    f"Pending legacy credential scrub output changed: {path_text}"
                )
            expected_fresh.add(identity)

    if state == "complete":
        if fresh_entries:
            raise ValueError(
                "Legacy credentials appeared after the completed migration scrub"
            )
    elif set(fresh_entries) != expected_fresh:
        raise ValueError(
            "Pending legacy credential scrub provenance does not match current sources"
        )
    # The installer must reject every already-known finalization blocker before
    # it publishes the staged database and new application files.  Pending
    # provenance permits only the recorded before/after INI transition; it does
    # not make plaintext database or credential-bearing backup residue safe.
    enforce_no_plaintext_configstore_residue(data_dir, forced_encoding)
    return state


def _verify_and_report_deferred_scrub(
    data_dir: Path,
    db_path: Path,
    forced_encoding: str | None,
    scrub_manifest: list[dict[str, object]] | None,
    installer_staging: bool,
) -> None:
    if scrub_manifest is None:
        raise ValueError("Deferred credential scrub manifest was not prepared")
    if installer_staging:
        state = _validate_installer_scrub_provenance(
            data_dir, db_path, forced_encoding, scrub_manifest
        )
        _reject_installer_staging_sidecars(db_path)
        if state == "pending":
            print(
                "Legacy credential scrub deferred with verified pending provenance; "
                "finalize after installer staging commit before starting the application."
            )
        else:
            print(
                "Legacy credential scrub provenance is complete and verified; "
                "no installer finalization is required."
            )
    else:
        print(
            "Legacy credential scrub deferred with verified pending provenance; "
            "finalize with --scrub-legacy-credentials before starting the application."
        )


def verify_installer_state(data_dir: Path, db_path: Path) -> tuple[str, Path]:
    resolved_data, resolved_db = _validate_installer_verification_paths(
        data_dir, db_path
    )
    before_inventory = _read_only_data_inventory(resolved_data)
    before_db_hash = _file_sha256_digest(resolved_db)
    current_state = verify_current_database(resolved_db)
    fresh_manifest = prepare_legacy_ini_credential_scrub(resolved_data, None)
    provenance_state = _validate_installer_scrub_provenance(
        resolved_data, resolved_db, None, fresh_manifest
    )
    if provenance_state != current_state:
        raise ValueError(
            "Installer database verification and scrub provenance states differ"
        )
    _reject_installer_staging_sidecars(resolved_db)
    after_inventory = _read_only_data_inventory(resolved_data)
    after_db_hash = _file_sha256_digest(resolved_db)
    if (
        before_inventory != after_inventory
        or not hmac.compare_digest(before_db_hash, after_db_hash)
    ):
        raise ValueError(
            "Read-only installer verification observed a Data directory change"
        )
    return current_state, resolved_db


def _default_upgrade_backup_path(db_path: Path) -> Path:
    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = f"{db_path.name}.bak_v{SCHEMA_VERSION}_{timestamp}"
    for suffix in range(1000):
        unique = "" if suffix == 0 else f"_{suffix}"
        candidate = db_path.with_name(f"{base_name}{unique}.dpapi.bak")
        if not candidate.exists():
            return candidate
    raise RuntimeError("Cannot allocate a unique protected upgrade backup path")


def verify_current_database(db_path: Path) -> str:
    resolved = db_path.resolve()
    if not resolved.is_file():
        raise ValueError(f"ConfigStore database not found: {resolved}")
    connection = sqlite3.connect(resolved.as_uri() + "?mode=ro", uri=True)
    try:
        connection.execute("PRAGMA query_only=ON")
        integrity = connection.execute("PRAGMA integrity_check").fetchone()
        if integrity != ("ok",):
            raise ValueError(f"ConfigStore integrity_check failed: {integrity}")
        if current_schema_version(connection) != SCHEMA_VERSION:
            raise ValueError("ConfigStore is not schema v5")
        if not has_current_schema(connection):
            raise ValueError("ConfigStore settings schema is incompatible")

        for legacy_table in ("ini_values", "text_files", "app_settings"):
            if table_exists(connection, legacy_table):
                raise ValueError(
                    f"Current ConfigStore retains legacy table: {legacy_table}"
                )
        orphan_count = connection.execute(
            """
            SELECT COUNT(*) FROM settings
            WHERE (
                lower(module)=lower(?) OR lower(module) LIKE lower(?) OR
                lower(module)=lower('LoginState/General') OR
                (lower(module) LIKE lower('LoginState/SavedPasswords%') AND NOT (
                    module='LoginState/SavedPasswords' AND
                    scope_type='global' AND scope_id=''
                )) OR
                (lower(module) LIKE lower('LoginState/RememberedCredentials%') AND NOT (
                    module='LoginState/RememberedCredentials' AND
                    scope_type='global' AND scope_id=''
                )) OR
                lower(key_name)='passwordbase64'
            )
            """,
            (LEGACY_ACCOUNT_MODULE, LEGACY_ACCOUNT_MODULE_PREFIX + "%"),
        ).fetchone()
        if orphan_count is None or int(orphan_count[0]) != 0:
            raise ValueError("Current ConfigStore retains legacy authentication rows")

        for (
            scope_type, scope_id, module, key, stored, value_type,
            sensitive, encrypted,
        ) in connection.execute(
            """
            SELECT scope_type, scope_id, module, key_name, value_text,
                   value_type, sensitive, encrypted
            FROM settings
            WHERE module IN (
                'LoginState/SavedPasswords',
                'LoginState/RememberedCredentials'
            )
            """
        ).fetchall():
            scope_text = str(scope_type)
            scope_id_text = str(scope_id)
            module_text = str(module)
            key_text = str(key)
            stored_text_value = str(stored)
            if (
                scope_text != "global"
                or scope_id_text != ""
                or not _is_safe_account_name(key_text)
                or str(value_type) != "string"
                or sensitive not in (1, "1", True)
                or encrypted not in (1, "1", True)
                or not stored_text_value.startswith(DPAPI_PREFIX)
            ):
                raise ValueError(
                    "Current remembered credential has a non-canonical shape"
                )
            decoded_credential = decode_stored_text(
                stored_text_value,
                encrypted,
                protection_purpose(
                    scope_text, scope_id_text, module_text, key_text
                ),
            )
            if decoded_credential is None or decoded_credential == "":
                raise ValueError(
                    "Current remembered credential cannot be read back"
                )

        meta_rows = connection.execute("SELECT key, value FROM meta").fetchall()
        if any(key is None or value is None for key, value in meta_rows):
            raise ValueError("ConfigStore metadata contains null keys or values")
        duplicate_meta = connection.execute(
            "SELECT key FROM meta GROUP BY key HAVING COUNT(*)<>1 LIMIT 1"
        ).fetchone()
        if duplicate_meta is not None:
            raise ValueError("ConfigStore metadata contains duplicate keys")
        meta = dict(meta_rows)
        required_meta = {
            "schema_version",
            "encrypt_new_values",
            "sensitive_protection",
            "auth_semantic_version",
            "auth_initialized",
        }
        if not required_meta.issubset(meta):
            raise ValueError("ConfigStore metadata is incomplete")
        if str(meta.get("encrypt_new_values", "")) not in {"0", "1"}:
            raise ValueError("ConfigStore encrypt_new_values marker is invalid")
        if str(meta.get("sensitive_protection", "")) != "dpapi-current-user-v1":
            raise ValueError("ConfigStore sensitive protection marker is invalid")
        if "created_at" in meta and not str(meta["created_at"]).strip():
            raise ValueError("ConfigStore created_at metadata is invalid")
        if str(meta.get("auth_semantic_version", "")) != AUTH_SEMANTIC_VERSION:
            raise ValueError("ConfigStore authentication semantic version is not current")
        authentication_initialized = str(meta.get("auth_initialized", ""))
        if authentication_initialized not in {"0", "1"}:
            raise ValueError("ConfigStore auth_initialized marker is invalid")

        scrub_state = str(meta.get(SCRUB_STATE_KEY, ""))
        scrub_manifest_text = meta.get(SCRUB_MANIFEST_KEY)
        if scrub_state:
            if scrub_state not in {"pending", "complete"}:
                raise ValueError("ConfigStore legacy credential scrub state is invalid")
            if not isinstance(scrub_manifest_text, str):
                raise ValueError("ConfigStore legacy credential scrub manifest is missing")
            parse_legacy_credential_scrub_manifest(scrub_manifest_text)
        elif scrub_manifest_text is not None:
            raise ValueError("ConfigStore has a scrub manifest without a scrub state")

        account_ids = _account_profile_ids(connection)

        administrator_count = 0
        for account_id in account_ids:
            for raw_key, stored, encrypted in connection.execute(
                """
                SELECT key_name, value_text, encrypted FROM settings
                WHERE scope_type='account' AND scope_id=? AND module='Profile'
                """,
                (account_id,),
            ).fetchall():
                key_text = str(raw_key)
                canonical_key = next(
                    (
                        candidate
                        for candidate in ACCOUNT_PROFILE_SECURITY_FIELDS
                        if candidate.casefold() == key_text.casefold()
                    ),
                    None,
                )
                if canonical_key is not None and key_text != canonical_key:
                    raise ValueError(
                        "Current account/Profile has a case-variant authentication field"
                    )
                if canonical_key is not None:
                    _require_portable_account_profile_storage(
                        str(stored), encrypted, key_text, "Current"
                    )
            values: dict[str, str] = {}
            for key, stored, encrypted in connection.execute(
                """
                SELECT key_name, value_text, encrypted FROM settings
                WHERE scope_type='account' AND scope_id=? AND module='Profile'
                  AND key_name IN ('PasswordHash', 'Role', 'MustChangePassword')
                """,
                (account_id,),
            ).fetchall():
                key_text = str(key)
                decoded = decode_stored_text(
                    str(stored), encrypted,
                    protection_purpose(
                        "account", account_id, "Profile", key_text
                    ),
                )
                if decoded is None:
                    raise ValueError("Cannot decode a current account/Profile field")
                values[key_text] = decoded
            if set(values) != {"PasswordHash", "Role", "MustChangePassword"}:
                raise ValueError("Current account/Profile is incomplete")
            if not _is_supported_password_record(values["PasswordHash"]):
                raise ValueError("Current account/Profile password record is invalid")
            if values["Role"] not in {"operator", "engineer", "admin"}:
                raise ValueError("Current account/Profile role is invalid")
            if _parse_authentication_bool(values["MustChangePassword"]) is None:
                raise ValueError("Current account/Profile MustChangePassword is invalid")
            for key, stored, encrypted in connection.execute(
                """
                SELECT key_name, value_text, encrypted FROM settings
                WHERE scope_type='account' AND scope_id=? AND module='Profile'
                  AND key_name IN ('CreatedAt', 'UpdatedAt', 'PasswordChangedAt')
                """,
                (account_id,),
            ).fetchall():
                key_text = str(key)
                decoded = decode_stored_text(
                    str(stored), encrypted,
                    protection_purpose(
                        "account", account_id, "Profile", key_text
                    ),
                )
                if decoded is None:
                    raise ValueError(
                        "Cannot decode a current account/Profile timestamp"
                    )
                _account_iso_instant(decoded, key_text)
            if values["Role"] == "admin":
                administrator_count += 1

        if authentication_initialized == "0" and account_ids:
            raise ValueError("Uninitialized authentication contains account profiles")
        if authentication_initialized == "1" and (
            not account_ids or administrator_count == 0
        ):
            raise ValueError("Initialized authentication has no valid administrator")

        for (
            scope_type, scope_id, module, key, stored, value_type,
            sensitive, encrypted,
        ) in connection.execute(
            """
            SELECT scope_type, scope_id, module, key_name, value_text,
                   value_type, sensitive, encrypted
            FROM settings
            """
        ).fetchall():
            scope_text = str(scope_type)
            scope_id_text = str(scope_id)
            module_text = str(module)
            key_text = str(key)
            stored_value = str(stored)
            value_type_text = str(value_type)
            purpose = protection_purpose(
                scope_text, scope_id_text, module_text, key_text
            )
            login_preference = is_login_preference(
                scope_text, module_text, key_text
            )
            compatible_disabled_preference = (
                is_compatible_disabled_login_preference(
                    scope_text, scope_id_text, module_text, key_text,
                    stored_value, value_type_text, sensitive, encrypted,
                )
            )
            semantic_sensitive = (
                bool(sensitive)
                or is_sensitive_setting_key(module_text)
                or is_sensitive_setting_key(key_text)
                or login_preference
            )
            requires_dpapi = requires_dpapi_protection(
                scope_text, module_text, key_text, semantic_sensitive
            )
            if (
                requires_dpapi
                and not stored_value.startswith(DPAPI_PREFIX)
                and not compatible_disabled_preference
            ):
                raise ValueError("A recoverable sensitive setting is not protected by DPAPI")
            if (
                stored_value.startswith(DPAPI_PREFIX)
                or encrypted not in (None, 0, "0", False)
            ) and decode_stored_text(stored_value, encrypted, purpose) is None:
                raise ValueError("A protected ConfigStore setting cannot be read back")
    finally:
        connection.close()
    return scrub_state or "none"


def migrate_existing_database_to_current(
    db_path: Path,
    data_dir: Path,
    encrypt_new_values: bool,
    scrub_manifest: list[dict[str, object]] | None = None,
    forced_encoding: str | None = None,
    upgrade_backup_path: Path | None = None,
) -> bool:
    if not db_path.exists():
        return False

    with closing(sqlite3.connect(db_path)) as conn:
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
                (
                    ".ini" in path.name.casefold()
                    and not _is_runtime_only_ini(path, data_dir)
                )
                or path.name.casefold() in legacy_text_names
            )
            for path in data_dir.rglob("*")
        )
        if version != SCHEMA_VERSION and has_disk_inputs:
            raise SystemExit(
                "Existing database upgrade does not import legacy INI/TXT files. "
                "Refusing to ignore or scrub them; use an explicit --overwrite migration after reviewing the protected backup."
            )

    backup = (
        _validated_upgrade_backup_path(db_path, upgrade_backup_path)
        if upgrade_backup_path is not None
        else _default_upgrade_backup_path(db_path)
    )
    create_dpapi_database_backup(db_path, backup)

    with closing(sqlite3.connect(db_path)) as conn:
        try:
            conn.execute("PRAGMA secure_delete=ON")
            conn.execute("BEGIN")
            create_current_tables(conn)
            legacy_counts = migrate_legacy_tables_to_settings(conn, encrypt_new_values)
            account_count, administrator_count = migrate_authentication_semantics(
                conn, encrypt_new_values
            )
            if (
                (authentication_initialized and account_count == 0)
                or (not authentication_initialized and account_count != 0)
                or (account_count > 0 and administrator_count == 0)
            ):
                raise ValueError(
                    "Migrated authentication initialization/account/admin state is inconsistent"
                )
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
    upgrade_backup_path: Path | None = None,
    defer_credential_scrub: bool = False,
    installer_staging: bool = False,
) -> None:
    installer_staging_mode = ""
    if installer_staging:
        if not defer_credential_scrub:
            raise ValueError(
                "--installer-staging requires --defer-credential-scrub"
            )
        if overwrite:
            raise ValueError("--installer-staging cannot be combined with --overwrite")
        data_dir, db_path, installer_staging_mode = (
            _validate_installer_staging_target(data_dir, db_path)
        )
    else:
        data_dir = data_dir.resolve()
        db_path = db_path.resolve()
    if not data_dir.exists():
        raise SystemExit(f"Data directory not found: {data_dir}")
    if defer_credential_scrub and scrub_legacy_credentials:
        raise ValueError(
            "--defer-credential-scrub and --scrub-legacy-credentials are mutually exclusive"
        )
    if defer_credential_scrub and not installer_staging:
        if overwrite or db_path.exists():
            raise ValueError(
                "--defer-credential-scrub is only valid for a new non-overwrite database"
            )
        if (
            db_path.name.casefold() != "configstore.db"
            or data_dir != db_path.parent
        ):
            raise ValueError(
                "--defer-credential-scrub requires an in-place Data/ConfigStore.db target"
            )
    if upgrade_backup_path is not None:
        if overwrite:
            raise ValueError("--upgrade-backup is only valid for a non-overwrite database upgrade")
        if not db_path.exists():
            raise ValueError("--upgrade-backup requires an existing target database")
        # Validate path policy before inspecting or changing the database.  The
        # file itself is created only after a real upgrade has been confirmed.
        upgrade_backup_path = _validated_upgrade_backup_path(
            db_path,
            upgrade_backup_path,
            require_absent=installer_staging,
        )
    if installer_staging_mode == "upgrade":
        if upgrade_backup_path is None:
            raise ValueError(
                "Installer upgrade staging requires an explicit --upgrade-backup"
            )
        expected_backup = db_path.with_name(
            db_path.name + ".install-backup.dpapi.bak"
        )
        if upgrade_backup_path != expected_backup:
            raise ValueError(
                "Installer upgrade staging backup must be named "
                "<staging basename>.install-backup.dpapi.bak"
            )
    elif installer_staging_mode == "create" and upgrade_backup_path is not None:
        raise ValueError("Installer create staging cannot use --upgrade-backup")
    in_place_scrub = (
        (scrub_legacy_credentials or defer_credential_scrub)
        and db_path.name.casefold() == "configstore.db"
        and data_dir == db_path.parent
    )
    deferred_staging_scrub = installer_staging and defer_credential_scrub
    scrub_manifest = prepare_legacy_ini_credential_scrub(data_dir, forced_encoding) \
        if (in_place_scrub or deferred_staging_scrub) else None

    if deferred_staging_scrub and db_path.exists() and scrub_manifest is not None:
        existing_scrub_state, _existing_scrub_manifest = (
            read_legacy_credential_scrub_provenance(db_path)
        )
        if existing_scrub_state:
            _validate_installer_scrub_provenance(
                data_dir, db_path, forced_encoding, scrub_manifest
            )

    if db_path.exists():
        if not overwrite:
            if migrate_existing_database_to_current(
                db_path,
                data_dir,
                encrypt,
                scrub_manifest,
                forced_encoding,
                upgrade_backup_path,
            ):
                if defer_credential_scrub:
                    _verify_and_report_deferred_scrub(
                        data_dir,
                        db_path,
                        forced_encoding,
                        scrub_manifest,
                        installer_staging,
                    )
                    return
                _finish_legacy_credential_scrub(
                    data_dir, db_path, forced_encoding, scrub_legacy_credentials
                )
                return
            if deferred_staging_scrub:
                _verify_and_report_deferred_scrub(
                    data_dir,
                    db_path,
                    forced_encoding,
                    scrub_manifest,
                    installer_staging,
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
    publish_staging_path = True
    if installer_staging_mode == "create":
        _reserve_installer_create_target(db_path)
        staging_path = db_path
        publish_staging_path = False
    else:
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
        if publish_staging_path:
            os.replace(staging_path, db_path)
    except Exception:
        staging_path.unlink(missing_ok=True)
        raise

    print(f"Created: {db_path}")
    print(f"INI files: {ini_count}, values: {value_count}")
    print(f"Text files: {text_count}")
    print(f"Runtime defaults added: {default_count} ({default_details})")
    print(f"Encrypted: {'yes' if encrypt else 'no'}")
    if defer_credential_scrub:
        _verify_and_report_deferred_scrub(
            data_dir,
            db_path,
            forced_encoding,
            scrub_manifest,
            installer_staging,
        )
    else:
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
    parser.add_argument("--source", default=None, help="Data directory to migrate (default: Data)")
    parser.add_argument("--db", default=None, help="Output SQLite database path")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing database")
    parser.add_argument("--encrypt", action="store_true", help="Encrypt stored values")
    parser.add_argument(
        "--scrub-legacy-credentials",
        action="store_true",
        help="After a successful in-place migration, atomically remove recoverable credentials from legacy INI files",
    )
    parser.add_argument(
        "--defer-credential-scrub",
        action="store_true",
        help=(
            "For a new in-place non-overwrite database, persist pending scrub "
            "provenance without changing legacy INI files"
        ),
    )
    parser.add_argument(
        "--installer-staging",
        action="store_true",
        help=(
            "Installer-only transaction mode: migrate an exact create/upgrade "
            "staging database beside the real Data source and retain pending "
            "credential-scrub provenance"
        ),
    )
    parser.add_argument(
        "--restore-dpapi-backup",
        type=Path,
        default=None,
        help="Restore a ConfigStore DPAPI backup to --db and exit",
    )
    parser.add_argument(
        "--upgrade-backup",
        type=Path,
        default=None,
        help=(
            "Absolute, non-existing .dpapi.bak path beside --db for the "
            "verified pre-upgrade backup (existing non-overwrite upgrades only)"
        ),
    )
    parser.add_argument(
        "--verify-current",
        action="store_true",
        help="Read-only verification of a current ConfigStore database",
    )
    parser.add_argument(
        "--verify-installer-state",
        action="store_true",
        help=(
            "Strict read-only verification of current ConfigStore schema and "
            "its real Data credential-scrub provenance"
        ),
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

    if args.verify_installer_state:
        incompatible = (
            args.overwrite
            or args.encrypt
            or args.scrub_legacy_credentials
            or args.defer_credential_scrub
            or args.installer_staging
            or args.restore_dpapi_backup is not None
            or args.upgrade_backup is not None
            or args.verify_current
            or args.encoding is not None
            or args.allow_mojibake
            or args.print_source_sha256
        )
        if args.source is None or args.db is None or incompatible:
            parser.error(
                "--verify-installer-state requires explicit --source and --db "
                "and cannot be combined with other modes"
            )

    if args.installer_staging:
        if not args.defer_credential_scrub:
            parser.error("--installer-staging requires --defer-credential-scrub")
        if (
            args.scrub_legacy_credentials
            or args.overwrite
            or args.restore_dpapi_backup is not None
            or args.verify_current
            or args.verify_installer_state
        ):
            parser.error(
                "--installer-staging cannot be combined with scrub, overwrite, "
                "restore, or verify modes"
            )

    if args.print_source_sha256:
        print(current_source_sha256())
        return

    should_pause = args.pause or launched_from_explorer()
    exit_code = 0
    try:
        source = Path(args.source) if args.source is not None else Path("Data")
        db = Path(args.db) if args.db else source / "ConfigStore.db"
        if args.verify_installer_state:
            scrub_state, resolved_db = verify_installer_state(source, db)
            print(
                "Verified installer ConfigStore state "
                f"(scrub={scrub_state}): {resolved_db}"
            )
            return
        if args.verify_current:
            incompatible = (
                args.overwrite
                or args.encrypt
                or args.scrub_legacy_credentials
                or args.defer_credential_scrub
                or args.installer_staging
                or args.verify_installer_state
                or args.restore_dpapi_backup is not None
                or args.upgrade_backup is not None
                or args.encoding is not None
                or args.allow_mojibake
            )
            if args.db is None or incompatible:
                raise ValueError(
                    "--verify-current requires explicit --db and cannot be combined with migration options"
                )
            before = hashlib.sha256(db.resolve().read_bytes()).hexdigest()
            scrub_state = verify_current_database(db)
            after = hashlib.sha256(db.resolve().read_bytes()).hexdigest()
            if not hmac.compare_digest(before, after):
                raise ValueError("Read-only verification observed a database change")
            print(
                f"Verified current ConfigStore schema v{SCHEMA_VERSION} "
                f"(scrub={scrub_state}): {db.resolve()}"
            )
            return
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
            args.upgrade_backup,
            args.defer_credential_scrub,
            args.installer_staging,
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
