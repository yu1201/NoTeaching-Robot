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
from contextlib import closing, ExitStack
import datetime as _dt
import hashlib
import hmac
import json
import os
import re
import sqlite3
import sys
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Callable, NamedTuple


SECRET = b"NoTeachingRobotConfigStoreV1"
SCHEMA_VERSION = "5"
AUTH_SEMANTIC_VERSION = "3"
DPAPI_PREFIX = "dpapi:user:v1:"
DPAPI_BACKUP_MAGIC = b"NoTeaching-Robot ConfigStore DPAPI backup v1\n"
DPAPI_BACKUP_PURPOSE = "configstore-database-backup-v1"
DEBUG_TRANSFER_PRIVATE_KEY_MAGIC = (
    b"NoTeaching-Robot ConfigStore debug transfer private key v1\n"
)
DEBUG_TRANSFER_PRIVATE_KEY_PURPOSE = "configstore-debug-transfer-private-key-v1"
DEBUG_TRANSFER_PACKAGE_MAGIC = (
    b"NoTeaching-Robot ConfigStore debug transfer package v1\n"
)
DEBUG_TRANSFER_PACKAGE_FORMAT = "configstore-debug-transfer-v1"
DEBUG_TRANSFER_PROTECTION = "debug-transfer-envelope-v1"
DEBUG_TRANSFER_VALUE_PREFIX = "debug-transfer:plain:v1:"
DEBUG_TRANSFER_META_KEYS = {
    "debug_transfer_format",
    "debug_transfer_recipient_sha256",
    "debug_transfer_setting_count",
    "debug_transfer_source_sha256",
}
ATOMIC_REPLACE_TRANSACTION_FORMAT = "NoTeaching-Robot-Atomic-Replace-v1"
ATOMIC_REPLACE_TRANSACTION_SUFFIX = ".atomic-transaction-v1"
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
    ("GunDownBackSafeDis", "70"),
    ("WeldSafeRetreatDirection", "0"),
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


def decode_bytes(
    data: bytes,
    forced_encoding: str | None = None,
) -> tuple[str, str]:
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


def decode_file(path: Path, forced_encoding: str | None = None) -> tuple[str, str]:
    absolute_path = Path(os.path.abspath(path))
    with _LockedWin32ReadFile(
        absolute_path, "Config text decode source", require_single_link=True
    ) as locked_source:
        content = locked_source.read_bytes()
        result = decode_bytes(content, forced_encoding)
        locked_source.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(content).digest(),
            hashlib.sha256(locked_source.read_bytes()).digest(),
        ):
            raise ValueError("Config text decode source changed while read")
        return result


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
    absolute_path = Path(os.path.abspath(path))
    with _LockedWin32ReadFile(
        absolute_path,
        "Legacy text sanitization source",
        require_single_link=True,
    ) as locked_source:
        content = locked_source.read_bytes()
        result = sanitize_legacy_text_bytes(
            absolute_path, content, forced_encoding, allow_mojibake
        )
        locked_source.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(content).digest(),
            hashlib.sha256(locked_source.read_bytes()).digest(),
        ):
            raise ValueError("Legacy text sanitization source changed while read")
        return result


def sanitize_legacy_text_bytes(
    path: Path,
    content: bytes,
    forced_encoding: str | None,
    allow_mojibake: bool,
) -> str:
    text, encoding = decode_bytes(content, forced_encoding)
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


def _canonical_data_relative_path(path_text: str, label: str) -> Path:
    """Return one Windows-safe, canonical POSIX path below an anchored root."""
    if not isinstance(path_text, str) or not path_text:
        raise ValueError(f"{label} path is empty")
    posix_path = PurePosixPath(path_text)
    windows_path = PureWindowsPath(path_text)
    parts = posix_path.parts
    if (
        "\\" in path_text
        or not parts
        or posix_path.is_absolute()
        or windows_path.is_absolute()
        or bool(windows_path.drive)
        or bool(windows_path.root)
        or posix_path.as_posix() != path_text
        or any(part in {"", ".", ".."} for part in parts)
        or any(
            any(ord(character) < 32 or character in '<>:"|?*' for character in part)
            or part.endswith((" ", "."))
            for part in parts
        )
    ):
        raise ValueError(f"{label} path is not a canonical relative path")
    return Path(*parts)


def read_robot_custom_name(
    data_dir: Path,
    robot_name: str,
    source_contents: dict[str, bytes] | None = None,
) -> str | None:
    if not robot_name:
        return None
    try:
        relative_robot = _canonical_data_relative_path(
            robot_name, "Robot custom-name source"
        )
    except ValueError:
        return None
    if len(relative_robot.parts) != 1:
        return None
    relative_robot_para = (
        PurePosixPath(relative_robot.as_posix()) / "RobotPara.ini"
    ).as_posix()
    if source_contents is not None:
        source_content = source_contents.get(relative_robot_para.casefold())
        if source_content is None:
            return None
        text, _encoding = decode_bytes(source_content)
        for section, key, value in parse_ini(text):
            if (
                section == "BaseParam"
                and key == "CustomName"
                and not detect_mojibake(value)
            ):
                return value.strip() or None
        return None
    root = Path(os.path.abspath(data_dir))
    robot_para = root.joinpath(*relative_robot.parts) / "RobotPara.ini"
    if not robot_para.exists():
        return None
    with _LockedWin32ReadFile(
        robot_para,
        "Robot custom-name source",
        require_single_link=True,
    ) as locked_source:
        source_content = locked_source.read_bytes()
        text, _encoding = decode_bytes(source_content)
        locked_source.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(source_content).digest(),
            hashlib.sha256(locked_source.read_bytes()).digest(),
        ):
            raise ValueError("Robot custom-name source changed while read")
    for section, key, value in parse_ini(text):
        if section == "BaseParam" and key == "CustomName" and not detect_mojibake(value):
            return value.strip() or None
    return None


def default_for_mojibake_value(
    data_dir: Path,
    section: str,
    key: str,
    sections: dict[str, dict[str, str]],
    source_contents: dict[str, bytes] | None = None,
) -> str | None:
    if section == "ChineseName":
        match = re.fullmatch(r"Unit(\d+)", key)
        if match:
            robot_name = sections.get("UnitName", {}).get(key, "").strip()
            custom_name = read_robot_custom_name(
                data_dir, robot_name, source_contents
            )
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
    source_contents: dict[str, bytes] | None = None,
) -> tuple[list[tuple[str, str, str]], list[tuple[str, str, str, str]]]:
    sections = build_section_map(rows)
    fixed_rows: list[tuple[str, str, str]] = []
    replacements: list[tuple[str, str, str, str]] = []

    for section, key, value in rows:
        marker = detect_mojibake("\n".join((section, key, value)))
        if marker:
            default_value = default_for_mojibake_value(
                data_dir, section, key, sections, source_contents
            )
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


def _atomic_replace_bytes(
    path: Path,
    content: bytes,
    *,
    expected_existing: bytes | None = None,
    require_absent: bool = False,
    sqlite_sidecar_base: Path | None = None,
    expected_existing_identity: tuple[int, int] | None = None,
    publication_guard_factory: Callable[[], _AtomicPublicationGuard] | None = None,
) -> None:
    if require_absent == (expected_existing is not None):
        raise ValueError(
            "Atomic replacement requires exactly one destination policy"
        )
    if os.name == "nt":
        if (
            sqlite_sidecar_base is None
            and expected_existing_identity is None
            and publication_guard_factory is None
        ):
            # Preserve the narrow callback contract used by the INI scrub
            # transaction tests; database publication supplies the additional
            # identity and sidecar gates below.
            _win32_atomic_replace_bytes(
                path,
                content,
                expected_existing=expected_existing,
                require_absent=require_absent,
            )
        else:
            extended_arguments: dict[str, object] = {
                "sqlite_sidecar_base": sqlite_sidecar_base,
                "expected_existing_identity": expected_existing_identity,
            }
            if publication_guard_factory is not None:
                extended_arguments["publication_guard_factory"] = (
                    publication_guard_factory
                )
            _win32_atomic_replace_bytes(
                path,
                content,
                expected_existing=expected_existing,
                require_absent=require_absent,
                **extended_arguments,
            )
        return

    raise OSError(
        "Identity-bound atomic replacement is supported only on Windows"
    )


def _sanitized_legacy_ini_bytes(
    path: Path,
    forced_encoding: str | None,
) -> tuple[bytes, int]:
    path = Path(os.path.abspath(path))
    with _LockedWin32ReadFile(
        path, "Legacy credential INI", require_single_link=True
    ) as locked_source:
        original = locked_source.read_bytes()
        result = _sanitized_legacy_ini_content(
            path, original, forced_encoding
        )
        locked_source.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(original).digest(),
            hashlib.sha256(locked_source.read_bytes()).digest(),
        ):
            raise ValueError("Legacy credential INI changed during read-back")
        return result


def _sanitized_legacy_ini_content(
    path: Path,
    original: bytes,
    forced_encoding: str | None,
) -> tuple[bytes, int]:
    text, encoding = decode_bytes(original, forced_encoding)
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


def _legacy_ini_contains_scrubbable_credentials(
    path: Path,
    forced_encoding: str | None,
) -> bool:
    absolute_path = Path(os.path.abspath(path))
    with _LockedWin32ReadFile(
        absolute_path,
        "Plaintext credential residue source",
        require_single_link=True,
    ) as locked_source:
        original = locked_source.read_bytes()
        sanitized, _removed = _sanitized_legacy_ini_content(
            absolute_path, original, forced_encoding
        )
        locked_source.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(original).digest(),
            hashlib.sha256(locked_source.read_bytes()).digest(),
        ):
            raise ValueError("Plaintext credential residue source changed")
        return sanitized != original


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
        absolute_path = Path(os.path.abspath(path))
        with _LockedWin32ReadFile(
            absolute_path,
            "Legacy credential scrub manifest source",
            require_single_link=True,
        ) as locked_source:
            original = locked_source.read_bytes()
            sanitized, removed_values = _sanitized_legacy_ini_content(
                absolute_path, original, forced_encoding
            )
            relative = absolute_path.resolve().relative_to(root).as_posix()
            locked_source.ensure_single_link()
            if not hmac.compare_digest(
                hashlib.sha256(original).digest(),
                hashlib.sha256(locked_source.read_bytes()).digest(),
            ):
                raise ValueError(
                    "Legacy credential scrub manifest source changed"
                )
        if sanitized == original:
            continue
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
            or type(item["removed_values"]) is not int
            or item["removed_values"] < 0
            or not isinstance(item["before_sha256"], str)
            or not isinstance(item["after_sha256"], str)
            or re.fullmatch(r"[0-9a-f]{64}", item["before_sha256"]) is None
            or re.fullmatch(r"[0-9a-f]{64}", item["after_sha256"]) is None
        ):
            raise ValueError("Legacy credential scrub manifest values are invalid")
    _canonical_scrub_manifest_map(parsed)
    return parsed


def _apply_legacy_ini_credential_scrub_anchored(
    data_dir: Path,
    forced_encoding: str | None,
    manifest: list[dict[str, object]],
) -> tuple[int, int]:
    root = Path(os.path.abspath(data_dir))
    _canonical_scrub_manifest_map(manifest)
    recover_pending_atomic_transactions(root)
    plans: list[
        tuple[Path, bytes, bytes, int, tuple[int, int]]
    ] = []
    modified_files = 0
    removed_values = 0
    for item in manifest:
        relative = _canonical_data_relative_path(
            str(item["path"]), "Legacy credential scrub manifest"
        )
        path = root.joinpath(*relative.parts)
        if not path.is_file():
            raise ValueError(f"Legacy credential scrub source is missing: {path}")
        with _LockedWin32ReadFile(
            path,
            "Legacy credential scrub source",
            require_single_link=True,
        ) as locked_source:
            current = locked_source.read_bytes()
            current_hash = hashlib.sha256(current).hexdigest()
            before_hash = str(item["before_sha256"])
            after_hash = str(item["after_sha256"])
            if hmac.compare_digest(current_hash, after_hash):
                continue
            if not hmac.compare_digest(current_hash, before_hash):
                raise ValueError(
                    f"Legacy credential source changed after migration: {path}"
                )
            sanitized, removed = _sanitized_legacy_ini_content(
                path, current, forced_encoding
            )
            if not hmac.compare_digest(
                hashlib.sha256(sanitized).hexdigest(), after_hash
            ):
                raise ValueError(
                    f"Legacy credential scrub output changed unexpectedly: {path}"
                )
            locked_source.ensure_single_link()
            if not hmac.compare_digest(
                hashlib.sha256(current).digest(),
                hashlib.sha256(locked_source.read_bytes()).digest(),
            ) or locked_source.identity is None:
                raise ValueError(
                    f"Legacy credential scrub source changed during planning: {path}"
                )
            source_identity = locked_source.identity
        plans.append((path, current, sanitized, removed, source_identity))

    # Validate every source and every expected output before changing the first
    # file.  Cross-file replacement is not an OS transaction, so retain each
    # original and roll back already-written files if an unexpected I/O error
    # occurs during the commit pass.
    committed: list[tuple[Path, bytes, bytes, tuple[int, int]]] = []
    try:
        for path, original, sanitized, removed, source_identity in plans:
            _atomic_replace_bytes(
                path,
                sanitized,
                expected_existing=original,
                expected_existing_identity=source_identity,
            )
            with _LockedWin32ReadFile(
                path,
                "Published legacy credential scrub source",
                require_single_link=True,
            ) as published_source:
                published_content = published_source.read_bytes()
                if not hmac.compare_digest(
                    hashlib.sha256(published_content).digest(),
                    hashlib.sha256(sanitized).digest(),
                ) or published_source.identity is None:
                    raise ValueError(
                        f"Legacy credential scrub did not persist expected bytes: {path}"
                    )
                published_identity = published_source.identity
            committed.append(
                (path, original, sanitized, published_identity)
            )
            modified_files += 1
            removed_values += removed
    except Exception as exc:
        rollback_failures: list[str] = []
        for path, original, sanitized, published_identity in reversed(committed):
            try:
                _atomic_replace_bytes(
                    path,
                    original,
                    expected_existing=sanitized,
                    expected_existing_identity=published_identity,
                )
            except Exception as rollback_exc:
                rollback_failures.append(f"{path}: {rollback_exc}")
        if rollback_failures:
            raise RuntimeError(
                "Legacy credential scrub failed and rollback was incomplete: "
                + "; ".join(rollback_failures)
            ) from exc
        raise
    return modified_files, removed_values


def apply_legacy_ini_credential_scrub(
    data_dir: Path,
    forced_encoding: str | None,
    manifest: list[dict[str, object]],
) -> tuple[int, int]:
    root = Path(os.path.abspath(data_dir))
    with _LockedWin32DirectoryChain(
        root, "Legacy credential scrub Data authority"
    ) as root_guard:
        result = _apply_legacy_ini_credential_scrub_anchored(
            root, forced_encoding, manifest
        )
        root_guard.ensure_unchanged()
        return result


def scrub_legacy_ini_credentials(
    data_dir: Path,
    forced_encoding: str | None,
) -> tuple[int, int]:
    recover_pending_atomic_transactions(data_dir.resolve())
    manifest = prepare_legacy_ini_credential_scrub(data_dir, forced_encoding)
    return apply_legacy_ini_credential_scrub(data_dir, forced_encoding, manifest)


def normalize_data_path(path: Path, data_dir: Path) -> str:
    relative = path.resolve().relative_to(data_dir.resolve())
    return "Data/" + relative.as_posix()


class _LegacySourceSnapshot(NamedTuple):
    kind: str
    absolute_path: Path
    relative_path: str
    database_path: str
    identity: tuple[int, int]
    sha256: bytes
    content: bytes


def _capture_legacy_source_snapshot(
    data_dir: Path,
    held_locks: ExitStack | None = None,
) -> tuple[_LegacySourceSnapshot, ...]:
    root = Path(os.path.abspath(data_dir))
    canonical_text_names = {
        name.casefold(): name for name in TEXT_FILE_NAMES
    }
    candidates = sorted(
        root.rglob("*"), key=lambda path: os.path.normcase(str(path))
    )
    entries: list[_LegacySourceSnapshot] = []
    logical_paths: dict[str, str] = {}
    for candidate in candidates:
        suffix = candidate.suffix.casefold()
        if suffix == ".ini":
            kind = "ini"
        elif (
            suffix == ".txt"
            and candidate.name.casefold() in canonical_text_names
        ):
            kind = "text"
        else:
            continue
        if not candidate.is_file():
            continue
        absolute_path = Path(os.path.abspath(candidate))
        try:
            relative_text = absolute_path.relative_to(root).as_posix()
        except ValueError as exc:
            raise ValueError("Legacy migration source escapes the Data directory") from exc
        relative = _canonical_data_relative_path(
            relative_text, "Legacy migration source"
        )
        canonical_relative = PurePosixPath(relative.as_posix())
        if kind == "text":
            canonical_relative = canonical_relative.with_name(
                canonical_text_names[candidate.name.casefold()]
            )
        database_path = "Data/" + canonical_relative.as_posix()
        logical_identity = database_path.casefold()
        previous = logical_paths.get(logical_identity)
        if previous is not None:
            raise ValueError(
                "Legacy migration sources have a case-insensitive path collision: "
                f"{previous} and {relative_text}"
            )
        logical_paths[logical_identity] = relative_text
        source_lock = _LockedWin32ReadFile(
            absolute_path,
            "Legacy migration source snapshot",
            require_single_link=True,
        )

        def capture_locked_source(
            locked_source: _LockedWin32ReadFile,
        ) -> tuple[bytes, tuple[int, int]]:
            content = locked_source.read_bytes()
            normalized_path = normalize_data_path(absolute_path, root)
            if normalized_path != "Data/" + relative_text:
                raise ValueError(
                    "Legacy migration source normalization changed identity"
                )
            locked_source.ensure_single_link()
            if locked_source.identity is None or not hmac.compare_digest(
                hashlib.sha256(content).digest(),
                hashlib.sha256(locked_source.read_bytes()).digest(),
            ):
                raise ValueError("Legacy migration source changed while snapshotted")
            return content, locked_source.identity

        if held_locks is None:
            with source_lock as locked_source:
                content, identity = capture_locked_source(locked_source)
        else:
            locked_source = held_locks.enter_context(source_lock)
            content, identity = capture_locked_source(locked_source)
        entries.append(_LegacySourceSnapshot(
            kind,
            absolute_path,
            relative_text,
            database_path,
            identity,
            hashlib.sha256(content).digest(),
            content,
        ))
    return tuple(sorted(entries, key=lambda item: item.database_path.casefold()))


def _assert_legacy_source_snapshot_unchanged(
    data_dir: Path,
    expected: tuple[_LegacySourceSnapshot, ...],
) -> None:
    current = _capture_legacy_source_snapshot(data_dir)
    if current != expected:
        raise ValueError(
            "Legacy INI/text source inventory changed during migration"
        )


class _AtomicPublicationGuard(NamedTuple):
    locks: ExitStack
    validate_after_publish: Callable[[], None]


def _acquire_legacy_source_publication_guard(
    data_dir: Path,
    expected: tuple[_LegacySourceSnapshot, ...],
) -> _AtomicPublicationGuard:
    locks = ExitStack()
    try:
        current = _capture_legacy_source_snapshot(data_dir, locks)
        if current != expected:
            raise ValueError(
                "Legacy INI/text source inventory changed at publication"
            )
        return _AtomicPublicationGuard(
            locks,
            lambda: _assert_legacy_source_snapshot_unchanged(
                data_dir, expected
            ),
        )
    except BaseException:
        locks.close()
        raise


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
    return lowered == "autologin" or any(marker in lowered for marker in (
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


def _read_dpapi_database_backup_content(content: bytes) -> bytes:
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


def _read_dpapi_database_backup(path: Path) -> bytes:
    path = Path(os.path.abspath(path))
    _reject_pending_atomic_replacement(path)
    with _LockedWin32ReadFile(
        path, "DPAPI restore backup", require_single_link=True
    ) as locked:
        content = locked.read_bytes()
        database_bytes = _read_dpapi_database_backup_content(content)
        locked.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(content).digest(),
            hashlib.sha256(locked.read_bytes()).digest(),
        ):
            raise ValueError("DPAPI restore backup changed during read-back")
        return database_bytes


def _absolute_regular_nonreparse_file(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} path must be absolute")
    absolute = Path(os.path.abspath(path))
    if not absolute.is_file():
        raise ValueError(f"{label} is missing or is not a regular file")
    current = absolute
    while True:
        metadata = os.lstat(current)
        attributes = int(getattr(metadata, "st_file_attributes", 0))
        if os.path.islink(current) or attributes & 0x00000400:
            raise ValueError(f"{label} path contains a reparse point")
        if current.parent == current:
            break
        current = current.parent
    return absolute.resolve(strict=True)


class _ByHandleFileInformation(ctypes.Structure):
    _fields_ = [
        ("dwFileAttributes", wintypes.DWORD),
        ("ftCreationTime", wintypes.FILETIME),
        ("ftLastAccessTime", wintypes.FILETIME),
        ("ftLastWriteTime", wintypes.FILETIME),
        ("dwVolumeSerialNumber", wintypes.DWORD),
        ("nFileSizeHigh", wintypes.DWORD),
        ("nFileSizeLow", wintypes.DWORD),
        ("nNumberOfLinks", wintypes.DWORD),
        ("nFileIndexHigh", wintypes.DWORD),
        ("nFileIndexLow", wintypes.DWORD),
    ]


class _FileRenameInformation(ctypes.Structure):
    _fields_ = [
        ("ReplaceIfExists", wintypes.BOOLEAN),
        ("RootDirectory", wintypes.HANDLE),
        ("FileNameLength", wintypes.DWORD),
        ("FileName", wintypes.WCHAR * 1),
    ]


class _FileDispositionInformation(ctypes.Structure):
    _fields_ = [("DeleteFile", wintypes.BOOL)]


class _LockedWin32ReadFile:
    """Bind one exact regular file while a stable read or SQLite write is active."""

    _GENERIC_READ = 0x80000000
    _FILE_READ_ATTRIBUTES = 0x00000080
    _FILE_SHARE_READ = 0x00000001
    _FILE_SHARE_WRITE = 0x00000002
    _OPEN_EXISTING = 3
    _FILE_ATTRIBUTE_DIRECTORY = 0x00000010
    _FILE_ATTRIBUTE_REPARSE_POINT = 0x00000400
    _FILE_FLAG_BACKUP_SEMANTICS = 0x02000000
    _FILE_FLAG_OPEN_REPARSE_POINT = 0x00200000
    _FILE_FLAG_SEQUENTIAL_SCAN = 0x08000000
    _FILE_BEGIN = 0
    _INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

    def __init__(
        self,
        path: Path,
        label: str,
        allow_writes: bool = False,
        expected_file_stat: os.stat_result | None = None,
        require_single_link: bool | None = None,
    ) -> None:
        if os.name != "nt":
            raise OSError(f"{label} locking requires Windows")
        if not path.is_absolute():
            raise ValueError(f"{label} path must be absolute")
        # Preserve the caller's lexical path.  Treating a pre-resolved junction
        # target as the expected path would let an ancestor swap become the new
        # authority between lstat/resolve and CreateFileW.
        self.path = Path(os.path.abspath(path))
        _absolute_regular_nonreparse_file(self.path, label)
        self.label = label
        self.allow_writes = allow_writes
        self.expected_file_stat = expected_file_stat
        self.require_single_link = (
            allow_writes if require_single_link is None else require_single_link
        )
        self.file_share_mode = self._FILE_SHARE_READ | (
            self._FILE_SHARE_WRITE if allow_writes else 0
        )
        self.handle: int | None = None
        self.directory_handles: list[int] = []
        self.final_path: Path | None = None
        self.identity: tuple[int, int] | None = None
        self.number_of_links = 0
        self.size = 0

    @staticmethod
    def _kernel32() -> ctypes.WinDLL:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR,
            wintypes.DWORD,
            wintypes.DWORD,
            ctypes.c_void_p,
            wintypes.DWORD,
            wintypes.DWORD,
            wintypes.HANDLE,
        ]
        kernel32.CreateFileW.restype = wintypes.HANDLE
        kernel32.GetFileInformationByHandle.argtypes = [
            wintypes.HANDLE,
            ctypes.POINTER(_ByHandleFileInformation),
        ]
        kernel32.GetFileInformationByHandle.restype = wintypes.BOOL
        kernel32.GetFinalPathNameByHandleW.argtypes = [
            wintypes.HANDLE,
            wintypes.LPWSTR,
            wintypes.DWORD,
            wintypes.DWORD,
        ]
        kernel32.GetFinalPathNameByHandleW.restype = wintypes.DWORD
        kernel32.SetFilePointerEx.argtypes = [
            wintypes.HANDLE,
            ctypes.c_longlong,
            ctypes.POINTER(ctypes.c_longlong),
            wintypes.DWORD,
        ]
        kernel32.SetFilePointerEx.restype = wintypes.BOOL
        kernel32.ReadFile.argtypes = [
            wintypes.HANDLE,
            ctypes.c_void_p,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            ctypes.c_void_p,
        ]
        kernel32.ReadFile.restype = wintypes.BOOL
        kernel32.WriteFile.argtypes = [
            wintypes.HANDLE,
            ctypes.c_void_p,
            wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD),
            ctypes.c_void_p,
        ]
        kernel32.WriteFile.restype = wintypes.BOOL
        kernel32.FlushFileBuffers.argtypes = [wintypes.HANDLE]
        kernel32.FlushFileBuffers.restype = wintypes.BOOL
        kernel32.SetEndOfFile.argtypes = [wintypes.HANDLE]
        kernel32.SetEndOfFile.restype = wintypes.BOOL
        kernel32.SetFileInformationByHandle.argtypes = [
            wintypes.HANDLE,
            ctypes.c_int,
            ctypes.c_void_p,
            wintypes.DWORD,
        ]
        kernel32.SetFileInformationByHandle.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL
        return kernel32

    @staticmethod
    def _normalized_handle_path(text: str) -> Path:
        if text.startswith("\\\\?\\UNC\\"):
            text = "\\\\" + text[8:]
        elif text.startswith("\\\\?\\"):
            text = text[4:]
        return Path(os.path.abspath(text))

    def __enter__(self) -> "_LockedWin32ReadFile":
        kernel32 = self._kernel32()
        directories: list[Path] = []
        current = self.path.parent
        while True:
            directories.append(current)
            if current.parent == current:
                break
            current = current.parent
        directories.reverse()
        try:
            for directory in directories:
                directory_handle = kernel32.CreateFileW(
                    str(directory),
                    self._FILE_READ_ATTRIBUTES,
                    self._FILE_SHARE_READ | self._FILE_SHARE_WRITE,
                    None,
                    self._OPEN_EXISTING,
                    self._FILE_FLAG_BACKUP_SEMANTICS
                    | self._FILE_FLAG_OPEN_REPARSE_POINT,
                    None,
                )
                if directory_handle == self._INVALID_HANDLE_VALUE:
                    error = ctypes.get_last_error()
                    raise OSError(
                        error,
                        f"Cannot lock a parent directory of {self.label}",
                    )
                self.directory_handles.append(directory_handle)
                directory_information = _ByHandleFileInformation()
                if not kernel32.GetFileInformationByHandle(
                    directory_handle, ctypes.byref(directory_information)
                ):
                    error = ctypes.get_last_error()
                    raise OSError(
                        error,
                        f"Cannot inspect a parent directory of {self.label}",
                    )
                if (
                    not directory_information.dwFileAttributes
                    & self._FILE_ATTRIBUTE_DIRECTORY
                    or directory_information.dwFileAttributes
                    & self._FILE_ATTRIBUTE_REPARSE_POINT
                ):
                    raise ValueError(
                        f"{self.label} path contains a reparse directory"
                    )
                directory_buffer = ctypes.create_unicode_buffer(32768)
                directory_length = kernel32.GetFinalPathNameByHandleW(
                    directory_handle,
                    directory_buffer,
                    len(directory_buffer),
                    0,
                )
                if (
                    directory_length == 0
                    or directory_length >= len(directory_buffer)
                ):
                    error = ctypes.get_last_error()
                    raise OSError(
                        error,
                        f"Cannot resolve a parent directory of {self.label}",
                    )
                final_directory = self._normalized_handle_path(
                    directory_buffer.value
                )
                if os.path.normcase(str(final_directory)) != os.path.normcase(
                    str(directory)
                ):
                    raise ValueError(
                        f"{self.label} path resolves through a replaced directory"
                    )

            # Re-check every lexical component only after its no-delete handle
            # has been acquired from the volume root down to the file parent.
            _absolute_regular_nonreparse_file(self.path, self.label)
        except Exception:
            self.close()
            raise

        handle = kernel32.CreateFileW(
            str(self.path),
            self._GENERIC_READ,
            self.file_share_mode,
            None,
            self._OPEN_EXISTING,
            self._FILE_FLAG_OPEN_REPARSE_POINT | self._FILE_FLAG_SEQUENTIAL_SCAN,
            None,
        )
        if handle == self._INVALID_HANDLE_VALUE:
            error = ctypes.get_last_error()
            self.close()
            raise OSError(error, f"Cannot lock {self.label} for stable read-back")
        self.handle = handle
        try:
            information = _ByHandleFileInformation()
            if not kernel32.GetFileInformationByHandle(
                handle, ctypes.byref(information)
            ):
                error = ctypes.get_last_error()
                raise OSError(error, f"Cannot inspect locked {self.label}")
            if information.dwFileAttributes & (
                self._FILE_ATTRIBUTE_DIRECTORY | self._FILE_ATTRIBUTE_REPARSE_POINT
            ):
                raise ValueError(f"{self.label} is not a non-reparse regular file")
            if self.expected_file_stat is not None:
                current_stat = os.stat(self.path, follow_symlinks=False)
                if not os.path.samestat(self.expected_file_stat, current_stat):
                    raise ValueError(
                        f"{self.label} no longer names the exclusively created file"
                    )
            self.number_of_links = int(information.nNumberOfLinks)
            if self.require_single_link and self.number_of_links != 1:
                raise ValueError(
                    f"{self.label} must have exactly one hard link before modification"
                )

            path_buffer = ctypes.create_unicode_buffer(32768)
            path_length = kernel32.GetFinalPathNameByHandleW(
                handle, path_buffer, len(path_buffer), 0
            )
            if path_length == 0 or path_length >= len(path_buffer):
                error = ctypes.get_last_error()
                raise OSError(error, f"Cannot resolve locked {self.label}")
            final_path = self._normalized_handle_path(path_buffer.value)
            if os.path.normcase(str(final_path)) != os.path.normcase(str(self.path)):
                raise ValueError(
                    f"{self.label} handle resolved through a replaced or reparse path"
                )
            self.final_path = final_path
            self.identity = (
                int(information.dwVolumeSerialNumber),
                (int(information.nFileIndexHigh) << 32)
                | int(information.nFileIndexLow),
            )
            self.size = (
                (int(information.nFileSizeHigh) << 32)
                | int(information.nFileSizeLow)
            )
            return self
        except Exception:
            self.close()
            raise

    def ensure_single_link(self) -> None:
        """Re-check a writable handle before a transaction is committed."""
        if self.handle is None:
            raise RuntimeError(f"{self.label} is not locked")
        information = _ByHandleFileInformation()
        kernel32 = self._kernel32()
        if not kernel32.GetFileInformationByHandle(
            self.handle, ctypes.byref(information)
        ):
            error = ctypes.get_last_error()
            raise OSError(error, f"Cannot re-inspect locked {self.label}")
        if information.dwFileAttributes & (
            self._FILE_ATTRIBUTE_DIRECTORY | self._FILE_ATTRIBUTE_REPARSE_POINT
        ):
            raise ValueError(f"{self.label} is not a non-reparse regular file")
        self.number_of_links = int(information.nNumberOfLinks)
        self.size = (
            (int(information.nFileSizeHigh) << 32)
            | int(information.nFileSizeLow)
        )
        current_identity = (
            int(information.dwVolumeSerialNumber),
            (int(information.nFileIndexHigh) << 32)
            | int(information.nFileIndexLow),
        )
        current_path = _win32_handle_path(
            kernel32, self.handle, f"locked {self.label}"
        )
        if (
            self.identity != current_identity
            or os.path.normcase(str(current_path))
            != os.path.normcase(str(self.path))
        ):
            raise ValueError(
                f"{self.label} identity or path changed while locked"
            )
        if self.number_of_links != 1:
            raise ValueError(
                f"{self.label} must have exactly one hard link while being modified"
            )

    def read_bytes(self) -> bytes:
        if self.handle is None:
            raise RuntimeError(f"{self.label} is not locked")
        kernel32 = self._kernel32()
        new_position = ctypes.c_longlong()
        if not kernel32.SetFilePointerEx(
            self.handle,
            ctypes.c_longlong(0),
            ctypes.byref(new_position),
            self._FILE_BEGIN,
        ):
            error = ctypes.get_last_error()
            raise OSError(error, f"Cannot rewind locked {self.label}")
        remaining = self.size
        chunks: list[bytes] = []
        while remaining:
            block_size = min(remaining, 1024 * 1024)
            buffer = ctypes.create_string_buffer(block_size)
            read_count = wintypes.DWORD()
            if not kernel32.ReadFile(
                self.handle,
                buffer,
                block_size,
                ctypes.byref(read_count),
                None,
            ):
                error = ctypes.get_last_error()
                raise OSError(error, f"Cannot read locked {self.label}")
            if read_count.value == 0:
                raise OSError(f"Unexpected end of locked {self.label}")
            chunks.append(buffer.raw[:read_count.value])
            remaining -= read_count.value
        return b"".join(chunks)

    def close(self) -> None:
        kernel32 = self._kernel32()
        if self.handle is not None:
            kernel32.CloseHandle(self.handle)
            self.handle = None
        for directory_handle in reversed(self.directory_handles):
            kernel32.CloseHandle(directory_handle)
        self.directory_handles.clear()

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()


class _LockedWin32DirectoryChain:
    """Anchor every lexical directory from the volume root through one parent."""

    def __init__(self, directory: Path, label: str) -> None:
        if os.name != "nt" or not directory.is_absolute():
            raise ValueError(f"{label} directory must be an absolute Windows path")
        self.directory = Path(os.path.abspath(directory))
        self.label = label
        self.handles: list[int] = []
        self.expected_paths: list[Path] = []
        self.expected_identities: list[tuple[int, int]] = []

    def __enter__(self) -> "_LockedWin32DirectoryChain":
        kernel32 = _LockedWin32ReadFile._kernel32()
        directories: list[Path] = []
        current = self.directory
        while True:
            directories.append(current)
            if current.parent == current:
                break
            current = current.parent
        directories.reverse()
        try:
            for directory in directories:
                handle = kernel32.CreateFileW(
                    str(directory),
                    _LockedWin32ReadFile._FILE_READ_ATTRIBUTES,
                    _LockedWin32ReadFile._FILE_SHARE_READ
                    | _LockedWin32ReadFile._FILE_SHARE_WRITE,
                    None,
                    _LockedWin32ReadFile._OPEN_EXISTING,
                    _LockedWin32ReadFile._FILE_FLAG_BACKUP_SEMANTICS
                    | _LockedWin32ReadFile._FILE_FLAG_OPEN_REPARSE_POINT,
                    None,
                )
                if handle == _LockedWin32ReadFile._INVALID_HANDLE_VALUE:
                    error = ctypes.get_last_error()
                    raise OSError(
                        error, f"Cannot anchor a directory for {self.label}"
                    )
                self.handles.append(handle)
                information = _ByHandleFileInformation()
                if not kernel32.GetFileInformationByHandle(
                    handle, ctypes.byref(information)
                ):
                    error = ctypes.get_last_error()
                    raise OSError(
                        error, f"Cannot inspect a directory for {self.label}"
                    )
                if (
                    not information.dwFileAttributes
                    & _LockedWin32ReadFile._FILE_ATTRIBUTE_DIRECTORY
                    or information.dwFileAttributes
                    & _LockedWin32ReadFile._FILE_ATTRIBUTE_REPARSE_POINT
                ):
                    raise ValueError(
                        f"{self.label} path contains a reparse directory"
                    )
                buffer = ctypes.create_unicode_buffer(32768)
                length = kernel32.GetFinalPathNameByHandleW(
                    handle, buffer, len(buffer), 0
                )
                if length == 0 or length >= len(buffer):
                    error = ctypes.get_last_error()
                    raise OSError(
                        error, f"Cannot resolve a directory for {self.label}"
                    )
                final_path = _LockedWin32ReadFile._normalized_handle_path(
                    buffer.value
                )
                if os.path.normcase(str(final_path)) != os.path.normcase(
                    str(directory)
                ):
                    raise ValueError(
                        f"{self.label} path resolves through a replaced directory"
                    )
                self.expected_paths.append(directory)
                self.expected_identities.append((
                    int(information.dwVolumeSerialNumber),
                    (int(information.nFileIndexHigh) << 32)
                    | int(information.nFileIndexLow),
                ))
            return self
        except BaseException:
            self.close()
            raise

    def ensure_unchanged(self) -> None:
        if not (
            len(self.handles)
            == len(self.expected_paths)
            == len(self.expected_identities)
        ):
            raise RuntimeError(f"{self.label} directory authority is incomplete")
        kernel32 = _LockedWin32ReadFile._kernel32()
        for handle, expected_path, expected_identity in zip(
            self.handles,
            self.expected_paths,
            self.expected_identities,
            strict=True,
        ):
            information = _ByHandleFileInformation()
            if not kernel32.GetFileInformationByHandle(
                handle, ctypes.byref(information)
            ):
                error = ctypes.get_last_error()
                raise OSError(
                    error, f"Cannot re-inspect {self.label} directory authority"
                )
            if (
                not information.dwFileAttributes
                & _LockedWin32ReadFile._FILE_ATTRIBUTE_DIRECTORY
                or information.dwFileAttributes
                & _LockedWin32ReadFile._FILE_ATTRIBUTE_REPARSE_POINT
            ):
                raise ValueError(
                    f"{self.label} directory authority became unsafe"
                )
            identity = (
                int(information.dwVolumeSerialNumber),
                (int(information.nFileIndexHigh) << 32)
                | int(information.nFileIndexLow),
            )
            final_path = _win32_handle_path(
                kernel32, handle, f"{self.label} directory authority"
            )
            if identity != expected_identity or os.path.normcase(
                str(final_path)
            ) != os.path.normcase(str(expected_path)):
                raise ValueError(
                    f"{self.label} directory identity changed while in use"
                )

    def close(self) -> None:
        kernel32 = _LockedWin32ReadFile._kernel32()
        for handle in reversed(self.handles):
            kernel32.CloseHandle(handle)
        self.handles.clear()
        self.expected_paths.clear()
        self.expected_identities.clear()

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()


def _win32_handle_information(
    kernel32: ctypes.WinDLL,
    handle: int,
    label: str,
) -> _ByHandleFileInformation:
    information = _ByHandleFileInformation()
    if not kernel32.GetFileInformationByHandle(
        handle, ctypes.byref(information)
    ):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot inspect {label}")
    if information.dwFileAttributes & (
        _LockedWin32ReadFile._FILE_ATTRIBUTE_DIRECTORY
        | _LockedWin32ReadFile._FILE_ATTRIBUTE_REPARSE_POINT
    ):
        raise ValueError(f"{label} is not a non-reparse regular file")
    return information


def _win32_handle_path(
    kernel32: ctypes.WinDLL,
    handle: int,
    label: str,
) -> Path:
    buffer = ctypes.create_unicode_buffer(32768)
    length = kernel32.GetFinalPathNameByHandleW(handle, buffer, len(buffer), 0)
    if length == 0 or length >= len(buffer):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot resolve {label}")
    return _LockedWin32ReadFile._normalized_handle_path(buffer.value)


def _win32_read_exact_handle(
    kernel32: ctypes.WinDLL,
    handle: int,
    size: int,
    label: str,
) -> bytes:
    new_position = ctypes.c_longlong()
    if not kernel32.SetFilePointerEx(
        handle,
        ctypes.c_longlong(0),
        ctypes.byref(new_position),
        _LockedWin32ReadFile._FILE_BEGIN,
    ):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot rewind {label}")
    remaining = size
    chunks: list[bytes] = []
    while remaining:
        block_size = min(remaining, 1024 * 1024)
        buffer = ctypes.create_string_buffer(block_size)
        read_count = wintypes.DWORD()
        if not kernel32.ReadFile(
            handle,
            buffer,
            block_size,
            ctypes.byref(read_count),
            None,
        ):
            error = ctypes.get_last_error()
            raise OSError(error, f"Cannot read {label}")
        if read_count.value == 0:
            raise OSError(f"Unexpected end of {label}")
        chunks.append(buffer.raw[:read_count.value])
        remaining -= read_count.value
    return b"".join(chunks)


def _win32_write_exact_handle(
    kernel32: ctypes.WinDLL,
    handle: int,
    content: bytes,
    label: str,
) -> None:
    offset = 0
    while offset < len(content):
        chunk = content[offset:offset + 1024 * 1024]
        buffer = ctypes.create_string_buffer(chunk)
        write_count = wintypes.DWORD()
        if not kernel32.WriteFile(
            handle,
            buffer,
            len(chunk),
            ctypes.byref(write_count),
            None,
        ):
            error = ctypes.get_last_error()
            raise OSError(error, f"Cannot write {label}")
        if write_count.value != len(chunk):
            raise OSError(f"Short write while writing {label}")
        offset += write_count.value
    if not kernel32.FlushFileBuffers(handle):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot flush {label}")


def _win32_rename_handle(
    kernel32: ctypes.WinDLL,
    handle: int,
    destination: Path,
    replace_existing: bool,
    label: str,
) -> None:
    destination_text = str(Path(os.path.abspath(destination)))
    encoded = destination_text.encode("utf-16-le")
    # WCHAR[1] is already part of sizeof(FILE_RENAME_INFO); allocate the full
    # variable tail plus one terminating WCHAR so the kernel never observes a
    # truncated or unterminated buffer on current Windows builds.
    size = _FileRenameInformation.FileName.offset + len(encoded) + 2
    buffer = ctypes.create_string_buffer(size)
    information = ctypes.cast(
        buffer, ctypes.POINTER(_FileRenameInformation)
    ).contents
    information.ReplaceIfExists = bool(replace_existing)
    information.RootDirectory = None
    information.FileNameLength = len(encoded)
    ctypes.memmove(
        ctypes.addressof(buffer) + _FileRenameInformation.FileName.offset,
        encoded,
        len(encoded),
    )
    if not kernel32.SetFileInformationByHandle(
        handle, 3, buffer, size
    ):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot rename exact {label} handle")


def _win32_set_delete_handle(
    kernel32: ctypes.WinDLL,
    handle: int,
    label: str,
    delete: bool,
) -> None:
    information = _FileDispositionInformation(bool(delete))
    if not kernel32.SetFileInformationByHandle(
        handle, 4, ctypes.byref(information), ctypes.sizeof(information)
    ):
        error = ctypes.get_last_error()
        action = "delete" if delete else "clear deletion of"
        raise OSError(error, f"Cannot {action} exact {label} handle")


def _win32_delete_handle(
    kernel32: ctypes.WinDLL,
    handle: int,
    label: str,
) -> None:
    _win32_set_delete_handle(kernel32, handle, label, True)


def _win32_scrub_exact_handle(
    kernel32: ctypes.WinDLL,
    handle: int,
    label: str,
) -> None:
    """Truncate one file object so every unexpected hard-link alias is empty."""
    new_position = ctypes.c_longlong()
    if not kernel32.SetFilePointerEx(
        handle,
        ctypes.c_longlong(0),
        ctypes.byref(new_position),
        _LockedWin32ReadFile._FILE_BEGIN,
    ):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot rewind unsafe {label}")
    if not kernel32.SetEndOfFile(handle):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot scrub unsafe {label}")
    if not kernel32.FlushFileBuffers(handle):
        error = ctypes.get_last_error()
        raise OSError(error, f"Cannot flush scrubbed {label}")


def _atomic_transaction_path(path: Path) -> Path:
    return path.with_name(f".{path.name}{ATOMIC_REPLACE_TRANSACTION_SUFFIX}")


def _atomic_transaction_object_paths(
    path: Path,
    transaction_id: str,
) -> tuple[Path, Path]:
    if re.fullmatch(r"[0-9a-f]{32}", transaction_id) is None:
        raise ValueError("Atomic replacement transaction id is invalid")
    return (
        path.with_name(f".{path.name}.atomic-{transaction_id}.tmp"),
        path.with_name(
            f".{path.name}.atomic-rollback-{transaction_id}.tmp"
        ),
    )


def _serialize_atomic_transaction(
    mode: str,
    transaction_id: str,
    old_sha256: str | None,
    new_sha256: str,
    reserve_sqlite_sidecars: bool = False,
) -> bytes:
    if mode not in {"create", "replace"}:
        raise ValueError("Atomic replacement transaction mode is invalid")
    if re.fullmatch(r"[0-9a-f]{32}", transaction_id) is None:
        raise ValueError("Atomic replacement transaction id is invalid")
    if (
        (mode == "create" and old_sha256 is not None)
        or (mode == "replace" and (
            old_sha256 is None
            or re.fullmatch(r"[0-9a-f]{64}", old_sha256) is None
        ))
        or re.fullmatch(r"[0-9a-f]{64}", new_sha256) is None
    ):
        raise ValueError("Atomic replacement transaction digest is invalid")
    if not isinstance(reserve_sqlite_sidecars, bool):
        raise ValueError("Atomic replacement transaction object kind is invalid")
    document = {
        "format": ATOMIC_REPLACE_TRANSACTION_FORMAT,
        "mode": mode,
        "new_sha256": new_sha256,
        "old_sha256": old_sha256,
        "reserve_sqlite_sidecars": reserve_sqlite_sidecars,
        "transaction_id": transaction_id,
    }
    payload = json.dumps(
        document,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    digest = hashlib.sha256(payload).hexdigest().encode("ascii")
    return payload + b"\nsha256=" + digest + b"\n"


def _parse_atomic_transaction(content: bytes) -> dict[str, object]:
    if b"\r" in content or not content.endswith(b"\n"):
        raise ValueError("Atomic replacement transaction framing is invalid")
    lines = content.split(b"\n")
    if len(lines) != 3 or lines[-1] != b"" or not lines[1].startswith(
        b"sha256="
    ):
        raise ValueError("Atomic replacement transaction framing is invalid")
    payload = lines[0]
    expected = hashlib.sha256(payload).hexdigest().encode("ascii")
    if not hmac.compare_digest(lines[1][len(b"sha256="):], expected):
        raise ValueError("Atomic replacement transaction digest is invalid")
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("Atomic replacement transaction payload is invalid") from exc
    if not isinstance(document, dict) or set(document) != {
        "format", "mode", "new_sha256", "old_sha256",
        "reserve_sqlite_sidecars", "transaction_id"
    }:
        raise ValueError("Atomic replacement transaction fields are invalid")
    canonical = json.dumps(
        document,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    if not hmac.compare_digest(payload, canonical):
        raise ValueError("Atomic replacement transaction is not canonical")
    if document.get("format") != ATOMIC_REPLACE_TRANSACTION_FORMAT:
        raise ValueError("Atomic replacement transaction format is invalid")
    mode = document.get("mode")
    transaction_id = document.get("transaction_id")
    old_sha256 = document.get("old_sha256")
    new_sha256 = document.get("new_sha256")
    reserve_sqlite_sidecars = document.get("reserve_sqlite_sidecars")
    if not isinstance(mode, str) or not isinstance(transaction_id, str) \
            or not isinstance(new_sha256, str) \
            or not isinstance(reserve_sqlite_sidecars, bool):
        raise ValueError("Atomic replacement transaction values are invalid")
    # Reuse the writer's strict policy instead of maintaining a looser parser.
    expected_content = _serialize_atomic_transaction(
        mode,
        transaction_id,
        old_sha256 if isinstance(old_sha256, str) else None,
        new_sha256,
        reserve_sqlite_sidecars,
    )
    if not hmac.compare_digest(content, expected_content):
        raise ValueError("Atomic replacement transaction values are invalid")
    return {
        "mode": mode,
        "transaction_id": transaction_id,
        "old_sha256": old_sha256 if isinstance(old_sha256, str) else None,
        "new_sha256": new_sha256,
        "reserve_sqlite_sidecars": reserve_sqlite_sidecars,
    }


def _win32_open_atomic_object(
    kernel32: ctypes.WinDLL,
    path: Path,
    label: str,
    *,
    writable: bool,
) -> tuple[int, bytes, _ByHandleFileInformation] | None:
    generic_write = 0x40000000
    delete_access = 0x00010000
    desired_access = _LockedWin32ReadFile._GENERIC_READ | delete_access
    if writable:
        desired_access |= generic_write
    handle = kernel32.CreateFileW(
        str(path),
        desired_access,
        _LockedWin32ReadFile._FILE_SHARE_READ,
        None,
        _LockedWin32ReadFile._OPEN_EXISTING,
        _LockedWin32ReadFile._FILE_FLAG_OPEN_REPARSE_POINT
        | _LockedWin32ReadFile._FILE_FLAG_SEQUENTIAL_SCAN,
        None,
    )
    if handle == _LockedWin32ReadFile._INVALID_HANDLE_VALUE:
        error = ctypes.get_last_error()
        if error in {2, 3}:
            return None
        raise OSError(error, f"Cannot bind {label}: {path}")
    try:
        information = _win32_handle_information(kernel32, handle, label)
        if os.path.normcase(str(_win32_handle_path(
            kernel32, handle, label
        ))) != os.path.normcase(str(path)):
            raise ValueError(f"The {label} path was replaced")
        size = (
            (int(information.nFileSizeHigh) << 32)
            | int(information.nFileSizeLow)
        )
        content = _win32_read_exact_handle(kernel32, handle, size, label)
        return handle, content, information
    except BaseException:
        kernel32.CloseHandle(handle)
        raise


def _win32_recover_atomic_transaction(
    kernel32: ctypes.WinDLL,
    path: Path,
    expected_reserve_sqlite_sidecars: bool | None = None,
) -> None:
    """Reconcile a process-killed exact-handle publication before a new one."""
    record_path = _atomic_transaction_path(path)
    record_object = _win32_open_atomic_object(
        kernel32,
        record_path,
        "atomic replacement transaction record",
        writable=False,
    )
    if record_object is None:
        return

    record_handle, record_content, record_information = record_object
    opened: list[int] = [record_handle]
    sidecar_handles: list[tuple[int, Path]] = []
    record_delete_pending = False
    security_error: str | None = None
    try:
        if int(record_information.nNumberOfLinks) != 1:
            raise ValueError(
                "Atomic replacement transaction record has a hard-link alias"
            )
        record = _parse_atomic_transaction(record_content)
        reserve_sqlite_sidecars = bool(record["reserve_sqlite_sidecars"])
        if (
            expected_reserve_sqlite_sidecars is not None
            and reserve_sqlite_sidecars
            != expected_reserve_sqlite_sidecars
        ):
            raise ValueError(
                "Atomic replacement transaction object kind is inconsistent"
            )
        if reserve_sqlite_sidecars:
            sidecar_handles = _win32_reserve_sqlite_sidecars(kernel32, path)
        transaction_id = str(record["transaction_id"])
        staging_path, quarantine_path = _atomic_transaction_object_paths(
            path, transaction_id
        )
        canonical_object = _win32_open_atomic_object(
            kernel32, path, "atomic recovery canonical", writable=False
        )
        staging_object = _win32_open_atomic_object(
            kernel32, staging_path, "atomic recovery staging", writable=True
        )
        quarantine_object = _win32_open_atomic_object(
            kernel32, quarantine_path, "atomic recovery quarantine", writable=True
        )
        for atomic_object in (
            canonical_object, staging_object, quarantine_object
        ):
            if atomic_object is not None:
                opened.append(atomic_object[0])

        def object_hash(
            atomic_object: tuple[int, bytes, _ByHandleFileInformation] | None,
        ) -> str | None:
            if atomic_object is None:
                return None
            return hashlib.sha256(atomic_object[1]).hexdigest()

        def require_single_link(
            atomic_object: tuple[int, bytes, _ByHandleFileInformation],
            label: str,
        ) -> None:
            information = _win32_handle_information(
                kernel32, atomic_object[0], label
            )
            if int(information.nNumberOfLinks) != 1:
                raise ValueError(f"{label} has a hard-link alias")

        def remove_staging() -> None:
            nonlocal security_error
            if staging_object is None:
                return
            information = _win32_handle_information(
                kernel32, staging_object[0], "atomic recovery staging"
            )
            if int(information.nNumberOfLinks) != 1:
                security_error = (
                    "Recovered atomic staging had a hard-link alias; all bound "
                    "bytes were scrubbed"
                )
            # The process may have been killed in the middle of writing this
            # object.  It is not authoritative until its exact handle is
            # published, so clear every bound alias before removing its name.
            # Scrubbing is deliberately unconditional: a hard link can be
            # created after the inspection above but before delete disposition.
            _win32_scrub_exact_handle(
                kernel32, staging_object[0], "atomic recovery staging"
            )
            _win32_delete_handle(
                kernel32, staging_object[0], "atomic recovery staging"
            )

        mode = str(record["mode"])
        old_sha256 = record["old_sha256"]
        new_sha256 = str(record["new_sha256"])
        canonical_hash = object_hash(canonical_object)
        quarantine_hash = object_hash(quarantine_object)

        if mode == "create":
            if quarantine_object is not None:
                raise ValueError(
                    "Atomic create recovery found an impossible quarantine"
                )
            if canonical_object is None:
                remove_staging()
            elif (
                canonical_hash == new_sha256
                and staging_object is None
            ):
                require_single_link(
                    canonical_object, "atomic recovery published create"
                )
            else:
                raise ValueError(
                    "Atomic create recovery found an ambiguous canonical state"
                )
        elif mode == "replace":
            if not isinstance(old_sha256, str):
                raise ValueError(
                    "Atomic replace recovery has no original digest"
                )
            if (
                canonical_hash == old_sha256
                and quarantine_object is None
            ):
                if canonical_object is None:
                    raise RuntimeError("Atomic recovery lost its canonical handle")
                require_single_link(
                    canonical_object, "atomic recovery original canonical"
                )
                remove_staging()
            elif (
                canonical_object is None
                and quarantine_hash == old_sha256
            ):
                if quarantine_object is None:
                    raise RuntimeError("Atomic recovery lost its quarantine handle")
                quarantine_information = _win32_handle_information(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery original quarantine",
                )
                if int(quarantine_information.nNumberOfLinks) != 1:
                    # The old object is still the only durable authority in
                    # this topology.  Renaming it back and retiring the record
                    # would make an attacker-created alias containing the old
                    # bytes permanent and untraceable.  Leave the transaction
                    # completely intact so every later entry remains fail
                    # closed until the alias is removed or an explicit secure
                    # recovery can scrub it.
                    raise ValueError(
                        "Recovered original quarantine has a hard-link alias; "
                        "the atomic transaction remains pending"
                    )
                _win32_rename_handle(
                    kernel32,
                    quarantine_object[0],
                    path,
                    False,
                    "atomic recovery original quarantine",
                )
                remove_staging()
            elif (
                canonical_hash == new_sha256
                and staging_object is None
                and quarantine_object is None
            ):
                if canonical_object is None:
                    raise RuntimeError("Atomic recovery lost its published handle")
                require_single_link(
                    canonical_object, "atomic recovery published replacement"
                )
            elif (
                canonical_hash == new_sha256
                and staging_object is None
                and quarantine_hash == old_sha256
            ):
                if canonical_object is None or quarantine_object is None:
                    raise RuntimeError("Atomic recovery lost a commit handle")
                require_single_link(
                    canonical_object, "atomic recovery published replacement"
                )
                quarantine_information = _win32_handle_information(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery committed quarantine",
                )
                # NEW is already the verified canonical authority.  OLD will
                # never be restored from this topology, so irreversibly clear
                # the exact quarantine object (and every alias) before asking
                # Windows to remove its name.  The scrub must not depend on a
                # pre-disposition link count: an alias can appear immediately
                # after that observation.
                _win32_scrub_exact_handle(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery committed quarantine",
                )
                if int(quarantine_information.nNumberOfLinks) != 1:
                    security_error = (
                        "Recovered committed quarantine had a hard-link alias; "
                        "all bound old bytes were scrubbed"
                    )
                _win32_delete_handle(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery committed quarantine",
                )
            elif (
                canonical_hash == new_sha256
                and staging_object is None
                and quarantine_object is not None
                and quarantine_object[1] == b""
            ):
                if canonical_object is None:
                    raise RuntimeError(
                        "Atomic recovery lost its published scrubbed handle"
                    )
                require_single_link(
                    canonical_object, "atomic recovery published replacement"
                )
                quarantine_information = _win32_handle_information(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery scrubbed quarantine",
                )
                # Re-scrub an empty crash-retained quarantine before
                # disposition.  This keeps the delete contract identical to
                # the non-empty branch and closes the last-window alias race.
                _win32_scrub_exact_handle(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery scrubbed quarantine",
                )
                if int(quarantine_information.nNumberOfLinks) != 1:
                    security_error = (
                        "Recovered scrubbed quarantine retained empty aliases"
                    )
                _win32_delete_handle(
                    kernel32,
                    quarantine_object[0],
                    "atomic recovery scrubbed quarantine",
                )
            else:
                raise ValueError(
                    "Atomic replacement recovery found an ambiguous file state"
                )
        else:
            raise ValueError("Atomic replacement recovery mode is invalid")

        _win32_delete_handle(
            kernel32, record_handle, "atomic replacement transaction record"
        )
        record_delete_pending = True
    finally:
        for handle in reversed(opened):
            kernel32.CloseHandle(handle)
        for handle, _sidecar_path in reversed(sidecar_handles):
            kernel32.CloseHandle(handle)
    if not record_delete_pending:
        raise RuntimeError("Atomic recovery did not retire its transaction record")
    if security_error is not None:
        raise ValueError(security_error)


def _win32_reserve_sqlite_sidecars(
    kernel32: ctypes.WinDLL,
    sqlite_sidecar_base: Path,
) -> list[tuple[int, Path]]:
    generic_write = 0x40000000
    delete_access = 0x00010000
    create_new = 1
    file_attribute_normal = 0x00000080
    file_attribute_temporary = 0x00000100
    file_flag_delete_on_close = 0x04000000
    handles: list[tuple[int, Path]] = []
    try:
        for suffix in ("-journal", "-wal", "-shm"):
            sidecar_path = Path(str(sqlite_sidecar_base) + suffix)
            sidecar_handle = kernel32.CreateFileW(
                str(sidecar_path),
                _LockedWin32ReadFile._GENERIC_READ
                | generic_write
                | delete_access,
                0,
                None,
                create_new,
                file_attribute_normal
                | file_attribute_temporary
                | file_flag_delete_on_close
                | _LockedWin32ReadFile._FILE_FLAG_OPEN_REPARSE_POINT,
                None,
            )
            if sidecar_handle == _LockedWin32ReadFile._INVALID_HANDLE_VALUE:
                error = ctypes.get_last_error()
                if error in {80, 183}:
                    raise ValueError(
                        "Atomic SQLite publication found an unsafe sidecar: "
                        f"{sidecar_path.name}"
                    )
                raise OSError(
                    error,
                    f"Cannot reserve SQLite sidecar name: {sidecar_path}",
                )
            handles.append((sidecar_handle, sidecar_path))
            sidecar_information = _win32_handle_information(
                kernel32,
                sidecar_handle,
                f"SQLite sidecar sentinel {sidecar_path.name}",
            )
            sidecar_size = (
                (int(sidecar_information.nFileSizeHigh) << 32)
                | int(sidecar_information.nFileSizeLow)
            )
            if (
                sidecar_information.dwFileAttributes
                & (
                    _LockedWin32ReadFile._FILE_ATTRIBUTE_DIRECTORY
                    | _LockedWin32ReadFile._FILE_ATTRIBUTE_REPARSE_POINT
                )
                or int(sidecar_information.nNumberOfLinks) != 1
                or sidecar_size != 0
                or os.path.normcase(str(_win32_handle_path(
                    kernel32,
                    sidecar_handle,
                    f"SQLite sidecar sentinel {sidecar_path.name}",
                ))) != os.path.normcase(str(sidecar_path))
            ):
                raise ValueError("SQLite sidecar sentinel identity is unsafe")
        return handles
    except BaseException:
        for handle, _path in reversed(handles):
            kernel32.CloseHandle(handle)
        raise


def _recover_pending_atomic_replacement(
    path: Path,
    *,
    reserve_sqlite_sidecars: bool | None = None,
) -> None:
    path = Path(os.path.abspath(path))
    record_path = _atomic_transaction_path(path)
    if not os.path.lexists(record_path):
        return
    if os.name != "nt":
        raise OSError(
            "Pending identity-bound atomic recovery requires Windows"
        )
    if not path.parent.is_dir():
        raise ValueError("Pending atomic recovery parent is missing")
    kernel32 = _LockedWin32ReadFile._kernel32()
    with _LockedWin32DirectoryChain(path.parent, "Atomic recovery"):
        _win32_recover_atomic_transaction(
            kernel32, path, reserve_sqlite_sidecars
        )


def recover_pending_atomic_transactions(root: Path) -> None:
    """Recover every interrupted mutation before any source inventory read."""
    root = Path(os.path.abspath(root))
    if not root.is_dir():
        return
    suffix = ATOMIC_REPLACE_TRANSACTION_SUFFIX
    records = sorted(
        record
        for record in root.rglob(f".*{suffix}")
    )
    for record in records:
        name = record.name
        if not name.startswith(".") or not name.endswith(suffix):
            raise ValueError("Pending atomic transaction name is invalid")
        target_name = name[1:-len(suffix)]
        if not target_name or target_name in {".", ".."}:
            raise ValueError("Pending atomic transaction target is invalid")
        target = record.with_name(target_name)
        _recover_pending_atomic_replacement(target)


def _reject_pending_atomic_replacement(path: Path) -> None:
    path = Path(os.path.abspath(path))
    candidates = [_atomic_transaction_path(path)]
    prefix = f".{path.name}.atomic-"
    if path.parent.is_dir():
        candidates.extend(
            candidate
            for candidate in path.parent.iterdir()
            if candidate.name.startswith(prefix)
            and candidate.name.endswith(".tmp")
        )
    unsafe = sorted({candidate for candidate in candidates if os.path.lexists(candidate)})
    if unsafe:
        raise ValueError(
            "Pending atomic replacement must be recovered before read-only "
            "verification: " + ", ".join(str(candidate) for candidate in unsafe)
        )


def reject_pending_atomic_transactions(root: Path) -> None:
    root = Path(os.path.abspath(root))
    if not root.is_dir():
        return
    records = sorted(root.rglob(
        f".*{ATOMIC_REPLACE_TRANSACTION_SUFFIX}"
    ))
    if records:
        raise ValueError(
            "Pending atomic transactions must be recovered before read-only "
            "verification: " + ", ".join(str(record) for record in records)
        )


def _win32_atomic_replace_bytes(
    path: Path,
    content: bytes,
    *,
    expected_existing: bytes | None,
    require_absent: bool,
    sqlite_sidecar_base: Path | None = None,
    expected_existing_identity: tuple[int, int] | None = None,
    publication_guard_factory: Callable[[], _AtomicPublicationGuard] | None = None,
) -> None:
    path = Path(os.path.abspath(path))
    if sqlite_sidecar_base is not None:
        sqlite_sidecar_base = Path(os.path.abspath(sqlite_sidecar_base))
        if sqlite_sidecar_base.parent != path.parent:
            raise ValueError(
                "Atomic SQLite sidecar base must share the publication parent"
            )
    if not path.is_absolute() or not path.parent.is_dir():
        raise ValueError(f"Atomic replacement parent is invalid: {path.parent}")
    kernel32 = _LockedWin32ReadFile._kernel32()
    generic_write = 0x40000000
    delete_access = 0x00010000
    create_new = 1
    open_existing = _LockedWin32ReadFile._OPEN_EXISTING
    file_attribute_normal = 0x00000080
    file_flag_write_through = 0x80000000
    invalid_handle = _LockedWin32ReadFile._INVALID_HANDLE_VALUE
    old_handle: int | None = None
    new_handle: int | None = None
    record_handle: int | None = None
    new_path: Path | None = None
    rollback_path: Path | None = None
    record_path = _atomic_transaction_path(path)
    old_quarantined = False
    old_delete_pending = False
    new_published = False
    record_delete_pending = False
    success = False
    committed_security_error: str | None = None
    must_preserve_verified_new = False
    old_cleanup_deferred = False
    rollback_failures: list[str] = []
    sidecar_handles: list[tuple[int, Path]] = []
    sidecars_delete_pending: set[int] = set()
    publication_guard: _AtomicPublicationGuard | None = None

    with _LockedWin32DirectoryChain(path.parent, "Atomic replacement"):
        try:
            if sqlite_sidecar_base is not None:
                # CREATE_NEW is the absence check.  The non-shared exact
                # handles reserve all SQLite sidecar names through publication,
                # eliminating the post-check-to-commit creation window.
                sidecar_handles = _win32_reserve_sqlite_sidecars(
                    kernel32, sqlite_sidecar_base
                )
            _win32_recover_atomic_transaction(kernel32, path)
            if require_absent:
                if os.path.lexists(path):
                    raise FileExistsError(
                        f"Atomic-create destination already exists: {path}"
                    )
            else:
                if expected_existing is None:
                    raise ValueError("Atomic replacement has no expected destination")
                old_handle = kernel32.CreateFileW(
                    str(path),
                    _LockedWin32ReadFile._GENERIC_READ
                    | generic_write
                    | delete_access,
                    _LockedWin32ReadFile._FILE_SHARE_READ,
                    None,
                    open_existing,
                    _LockedWin32ReadFile._FILE_FLAG_OPEN_REPARSE_POINT
                    | _LockedWin32ReadFile._FILE_FLAG_SEQUENTIAL_SCAN,
                    None,
                )
                if old_handle == invalid_handle:
                    error = ctypes.get_last_error()
                    raise OSError(error, f"Cannot bind atomic destination: {path}")
                old_information = _win32_handle_information(
                    kernel32, old_handle, "atomic destination"
                )
                if int(old_information.nNumberOfLinks) != 1:
                    raise ValueError(
                        "Atomic destination must have exactly one hard link"
                    )
                old_identity = (
                    int(old_information.dwVolumeSerialNumber),
                    (int(old_information.nFileIndexHigh) << 32)
                    | int(old_information.nFileIndexLow),
                )
                if (
                    expected_existing_identity is not None
                    and old_identity != expected_existing_identity
                ):
                    raise ValueError(
                        "Atomic destination identity changed before publication"
                    )
                if os.path.normcase(str(_win32_handle_path(
                    kernel32, old_handle, "atomic destination"
                ))) != os.path.normcase(str(path)):
                    raise ValueError("Atomic destination identity was replaced")
                old_size = (
                    (int(old_information.nFileSizeHigh) << 32)
                    | int(old_information.nFileSizeLow)
                )
                observed = _win32_read_exact_handle(
                    kernel32, old_handle, old_size, "atomic destination"
                )
                if not hmac.compare_digest(observed, expected_existing):
                    raise ValueError(f"Atomic-replace destination changed: {path}")

            transaction_id = os.urandom(16).hex()
            new_path, rollback_path = _atomic_transaction_object_paths(
                path, transaction_id
            )
            transaction_content = _serialize_atomic_transaction(
                "create" if require_absent else "replace",
                transaction_id,
                None if require_absent else hashlib.sha256(
                    expected_existing or b""
                ).hexdigest(),
                hashlib.sha256(content).hexdigest(),
                sqlite_sidecar_base is not None,
            )
            record_handle = kernel32.CreateFileW(
                str(record_path),
                _LockedWin32ReadFile._GENERIC_READ
                | generic_write
                | delete_access,
                _LockedWin32ReadFile._FILE_SHARE_READ,
                None,
                create_new,
                file_attribute_normal
                | file_flag_write_through
                | _LockedWin32ReadFile._FILE_FLAG_OPEN_REPARSE_POINT,
                None,
            )
            if record_handle == invalid_handle:
                error = ctypes.get_last_error()
                if error in {80, 183}:
                    raise FileExistsError(
                        "An atomic replacement transaction is already pending"
                    )
                raise OSError(
                    error, "Cannot create atomic replacement transaction record"
                )
            _win32_write_exact_handle(
                kernel32,
                record_handle,
                transaction_content,
                "atomic replacement transaction record",
            )
            record_information = _win32_handle_information(
                kernel32,
                record_handle,
                "atomic replacement transaction record",
            )
            record_size = (
                (int(record_information.nFileSizeHigh) << 32)
                | int(record_information.nFileSizeLow)
            )
            if (
                int(record_information.nNumberOfLinks) != 1
                or os.path.normcase(str(_win32_handle_path(
                    kernel32,
                    record_handle,
                    "atomic replacement transaction record",
                ))) != os.path.normcase(str(record_path))
                or not hmac.compare_digest(
                    _win32_read_exact_handle(
                        kernel32,
                        record_handle,
                        record_size,
                        "atomic replacement transaction record",
                    ),
                    transaction_content,
                )
            ):
                raise ValueError(
                    "Atomic replacement transaction record failed read-back"
                )

            new_handle = kernel32.CreateFileW(
                str(new_path),
                _LockedWin32ReadFile._GENERIC_READ
                | generic_write
                | delete_access,
                0,
                None,
                create_new,
                file_attribute_normal
                | file_flag_write_through
                | _LockedWin32ReadFile._FILE_FLAG_OPEN_REPARSE_POINT,
                None,
            )
            if new_handle == invalid_handle:
                error = ctypes.get_last_error()
                raise OSError(error, f"Cannot create atomic staging file: {path}")

            _win32_write_exact_handle(
                kernel32, new_handle, content, "atomic staging file"
            )
            new_information = _win32_handle_information(
                kernel32, new_handle, "atomic staging file"
            )
            if int(new_information.nNumberOfLinks) != 1:
                raise ValueError("Atomic staging file gained a hard-link alias")
            new_size = (
                (int(new_information.nFileSizeHigh) << 32)
                | int(new_information.nFileSizeLow)
            )
            if new_size != len(content) or not hmac.compare_digest(
                _win32_read_exact_handle(
                    kernel32, new_handle, new_size, "atomic staging file"
                ),
                content,
            ):
                raise ValueError("Atomic staging file read-back did not match")

            if old_handle is not None:
                old_information = _win32_handle_information(
                    kernel32, old_handle, "atomic destination"
                )
                old_size = (
                    (int(old_information.nFileSizeHigh) << 32)
                    | int(old_information.nFileSizeLow)
                )
                if (
                    int(old_information.nNumberOfLinks) != 1
                    or not hmac.compare_digest(
                        _win32_read_exact_handle(
                            kernel32,
                            old_handle,
                            old_size,
                            "atomic destination",
                        ),
                        expected_existing or b"",
                    )
                ):
                    raise ValueError(
                        "Atomic destination changed before publication"
                    )
                if rollback_path is None:
                    raise RuntimeError(
                        "Atomic replacement has no bound rollback quarantine"
                    )
                _win32_rename_handle(
                    kernel32,
                    old_handle,
                    rollback_path,
                    False,
                    "old destination",
                )
                old_quarantined = True
                quarantined_information = _win32_handle_information(
                    kernel32, old_handle, "quarantined old destination"
                )
                quarantined_size = (
                    (int(quarantined_information.nFileSizeHigh) << 32)
                    | int(quarantined_information.nFileSizeLow)
                )
                if (
                    int(quarantined_information.nNumberOfLinks) != 1
                    or os.path.normcase(str(_win32_handle_path(
                        kernel32,
                        old_handle,
                        "quarantined old destination",
                    ))) != os.path.normcase(str(rollback_path))
                    or not hmac.compare_digest(
                        _win32_read_exact_handle(
                            kernel32,
                            old_handle,
                            quarantined_size,
                            "quarantined old destination",
                        ),
                        expected_existing or b"",
                    )
                ):
                    raise ValueError(
                        "Old atomic destination changed during quarantine"
                    )
            elif os.path.lexists(path):
                raise FileExistsError(
                    f"Atomic-create destination appeared before publish: {path}"
                )

            if publication_guard_factory is not None:
                publication_guard = publication_guard_factory()

            prepublish_information = _win32_handle_information(
                kernel32, new_handle, "atomic staging before publication"
            )
            prepublish_size = (
                (int(prepublish_information.nFileSizeHigh) << 32)
                | int(prepublish_information.nFileSizeLow)
            )
            if (
                int(prepublish_information.nNumberOfLinks) != 1
                or new_path is None
                or os.path.normcase(str(_win32_handle_path(
                    kernel32,
                    new_handle,
                    "atomic staging before publication",
                ))) != os.path.normcase(str(new_path))
                or prepublish_size != len(content)
                or not hmac.compare_digest(
                    _win32_read_exact_handle(
                        kernel32,
                        new_handle,
                        prepublish_size,
                        "atomic staging before publication",
                    ),
                    content,
                )
            ):
                raise ValueError(
                    "Atomic staging changed immediately before publication"
                )
            _win32_rename_handle(
                kernel32, new_handle, path, False, "new destination"
            )
            new_published = True
            if os.path.normcase(str(_win32_handle_path(
                kernel32, new_handle, "published destination"
            ))) != os.path.normcase(str(path)):
                raise ValueError("Published atomic destination path is not canonical")
            published_information = _win32_handle_information(
                kernel32, new_handle, "published destination"
            )
            published_size = (
                (int(published_information.nFileSizeHigh) << 32)
                | int(published_information.nFileSizeLow)
            )
            if (
                int(published_information.nNumberOfLinks) != 1
                or published_size != len(content)
                or not hmac.compare_digest(
                    _win32_read_exact_handle(
                        kernel32,
                        new_handle,
                        published_size,
                        "published destination",
                    ),
                    content,
                )
            ):
                raise ValueError("Published atomic destination failed read-back")
            if not kernel32.FlushFileBuffers(new_handle):
                error = ctypes.get_last_error()
                raise OSError(error, "Cannot flush published atomic destination")
            for sidecar_handle, sidecar_path in sidecar_handles:
                sidecar_information = _win32_handle_information(
                    kernel32,
                    sidecar_handle,
                    f"published SQLite sidecar sentinel {sidecar_path.name}",
                )
                sidecar_size = (
                    (int(sidecar_information.nFileSizeHigh) << 32)
                    | int(sidecar_information.nFileSizeLow)
                )
                if (
                    int(sidecar_information.nNumberOfLinks) != 1
                    or sidecar_size != 0
                    or os.path.normcase(str(_win32_handle_path(
                        kernel32,
                        sidecar_handle,
                        f"published SQLite sidecar sentinel {sidecar_path.name}",
                    ))) != os.path.normcase(str(sidecar_path))
                ):
                    raise ValueError(
                        "SQLite sidecar sentinel changed during publication"
                    )
            final_new_information = _win32_handle_information(
                kernel32, new_handle, "final published destination"
            )
            if (
                int(final_new_information.nNumberOfLinks) != 1
                or os.path.normcase(str(_win32_handle_path(
                    kernel32, new_handle, "final published destination"
                ))) != os.path.normcase(str(path))
            ):
                raise ValueError(
                    "Published atomic destination gained a hard-link or path alias"
                )
            if old_handle is not None:
                final_old_information = _win32_handle_information(
                    kernel32, old_handle, "final old destination quarantine"
                )
                final_old_size = (
                    (int(final_old_information.nFileSizeHigh) << 32)
                    | int(final_old_information.nFileSizeLow)
                )
                if (
                    int(final_old_information.nNumberOfLinks) != 1
                    or rollback_path is None
                    or os.path.normcase(str(_win32_handle_path(
                        kernel32,
                        old_handle,
                        "final old destination quarantine",
                    ))) != os.path.normcase(str(rollback_path))
                    or not hmac.compare_digest(
                        _win32_read_exact_handle(
                            kernel32,
                            old_handle,
                            final_old_size,
                            "final old destination quarantine",
                        ),
                        expected_existing or b"",
                    )
                ):
                    raise ValueError(
                        "Old atomic destination gained a hard-link or changed"
                    )
            record_information = _win32_handle_information(
                kernel32,
                record_handle,
                "committed atomic replacement transaction record",
            )
            if (
                int(record_information.nNumberOfLinks) != 1
                or os.path.normcase(str(_win32_handle_path(
                    kernel32,
                    record_handle,
                    "committed atomic replacement transaction record",
                ))) != os.path.normcase(str(record_path))
            ):
                raise ValueError(
                    "Atomic replacement transaction record identity changed"
                )
            if publication_guard is not None:
                publication_guard.validate_after_publish()
            # Mark sentinels delete-pending while their exclusive handles still
            # reserve the names.  They disappear only after all database
            # commit/rollback decisions have completed and the handles close.
            for sidecar_handle, sidecar_path in sidecar_handles:
                _win32_delete_handle(
                    kernel32,
                    sidecar_handle,
                    f"SQLite sidecar sentinel {sidecar_path.name}",
                )
                sidecars_delete_pending.add(sidecar_handle)
            if publication_guard is not None:
                publication_guard.validate_after_publish()
            if old_handle is not None:
                # NEW has passed every read-back and publication guard.  OLD is
                # now discarded, so clear the exact quarantine object and all
                # of its hard-link aliases *before* delete disposition.  This
                # is unconditional: checking the link count first leaves a
                # last-window race in which one alias can retain raw database
                # or credential bytes.  Set the no-rollback latch before the
                # scrub call because truncation may succeed even if a callback
                # or later flush/disposition reports an exception.
                must_preserve_verified_new = True
                _win32_scrub_exact_handle(
                    kernel32,
                    old_handle,
                    "retired old atomic destination",
                )
                scrubbed_old_information = _win32_handle_information(
                    kernel32,
                    old_handle,
                    "scrubbed old atomic destination",
                )
                scrubbed_old_size = (
                    (int(scrubbed_old_information.nFileSizeHigh) << 32)
                    | int(scrubbed_old_information.nFileSizeLow)
                )
                if scrubbed_old_size != 0:
                    raise ValueError(
                        "Retired old atomic destination did not scrub to zero bytes"
                    )
                try:
                    _win32_delete_handle(
                        kernel32,
                        old_handle,
                        "old atomic destination",
                    )
                    old_delete_pending = True
                    delete_pending_information = _win32_handle_information(
                        kernel32,
                        old_handle,
                        "delete-pending scrubbed old atomic destination",
                    )
                    # A positive count now represents only empty aliases.  The
                    # sensitive bytes were already durably flushed away, so it
                    # is safe to retire the transaction while still surfacing
                    # the unexpected alias to the caller.
                    if int(delete_pending_information.nNumberOfLinks) != 0:
                        committed_security_error = (
                            "Old atomic destination retained a late hard-link alias; "
                            "the verified new destination was retained and every "
                            "bound old alias was scrubbed to empty content"
                        )
                except BaseException as old_cleanup_exc:
                    old_cleanup_deferred = True
                    committed_security_error = (
                        "The verified new destination was retained after OLD was "
                        "scrubbed; quarantine cleanup was deferred: "
                        + str(old_cleanup_exc)
                    )
            if publication_guard is not None:
                publication_guard.validate_after_publish()
            if old_cleanup_deferred:
                committed_security_error += (
                    "; transaction record retained for next-entry recovery"
                )
            else:
                try:
                    _win32_delete_handle(
                        kernel32,
                        record_handle,
                        "atomic replacement transaction record",
                    )
                    record_delete_pending = True
                except BaseException as record_exc:
                    if committed_security_error is None:
                        raise
                    committed_security_error += (
                        "; transaction record cleanup was deferred: "
                        + str(record_exc)
                    )
            if publication_guard is not None:
                publication_guard.validate_after_publish()
            success = True
        except BaseException as exc:
            # Python can observe an exception immediately after a successful
            # Win32 rename/disposition (for example a test hook or asynchronous
            # interruption).  Derive commit topology from the still-open exact
            # handles before making any rollback decision; the booleans below
            # are only confirmations, never the source of truth.
            topology_failures: list[str] = []
            if old_handle is not None:
                try:
                    old_state = _win32_handle_information(
                        kernel32, old_handle, "uncertain old atomic destination"
                    )
                    if int(old_state.nNumberOfLinks) == 0:
                        old_delete_pending = True
                        old_quarantined = True
                    else:
                        old_location = _win32_handle_path(
                            kernel32,
                            old_handle,
                            "uncertain old atomic destination",
                        )
                        if rollback_path is not None and os.path.normcase(
                            str(old_location)
                        ) == os.path.normcase(str(rollback_path)):
                            old_quarantined = True
                        elif os.path.normcase(str(old_location)) == os.path.normcase(
                            str(path)
                        ):
                            old_quarantined = False
                        else:
                            topology_failures.append(
                                f"old handle has unexpected path {old_location}"
                            )
                except BaseException as topology_exc:
                    topology_failures.append(
                        f"cannot classify old handle: {topology_exc}"
                    )
            if new_handle is not None:
                try:
                    new_location = _win32_handle_path(
                        kernel32,
                        new_handle,
                        "uncertain new atomic destination",
                    )
                    if os.path.normcase(str(new_location)) == os.path.normcase(
                        str(path)
                    ):
                        new_published = True
                    elif new_path is not None and os.path.normcase(
                        str(new_location)
                    ) == os.path.normcase(str(new_path)):
                        new_published = False
                    else:
                        topology_failures.append(
                            f"new handle has unexpected path {new_location}"
                        )
                except BaseException as topology_exc:
                    topology_failures.append(
                        f"cannot classify new handle: {topology_exc}"
                    )
            if record_handle is not None:
                try:
                    record_state = _win32_handle_information(
                        kernel32,
                        record_handle,
                        "uncertain atomic replacement transaction record",
                    )
                    if int(record_state.nNumberOfLinks) == 0:
                        record_delete_pending = True
                except BaseException as topology_exc:
                    topology_failures.append(
                        f"cannot classify transaction record: {topology_exc}"
                    )
            if topology_failures:
                raise RuntimeError(
                    "Atomic replacement outcome is uncertain; transaction "
                    "record retained: " + "; ".join(topology_failures)
                ) from exc

            preserve_verified_new = must_preserve_verified_new
            if (
                old_delete_pending
                and old_handle is not None
                and not preserve_verified_new
            ):
                try:
                    _win32_set_delete_handle(
                        kernel32,
                        old_handle,
                        "old atomic destination",
                        False,
                    )
                    old_delete_pending = False
                except BaseException as rollback_exc:
                    rollback_failures.append(
                        f"clear old deletion disposition: {rollback_exc}"
                    )
                    # OLD may disappear when its handle closes.  NEW has
                    # already passed full read-back before OLD disposition is
                    # issued, so retain NEW rather than risk losing both.
                    preserve_verified_new = new_published
            if (
                new_published
                and new_handle is not None
                and not preserve_verified_new
            ):
                try:
                    if new_path is None:
                        raise RuntimeError(
                            "Atomic rollback lost its deterministic staging path"
                        )
                    _win32_rename_handle(
                        kernel32,
                        new_handle,
                        new_path,
                        False,
                        "failed new destination",
                    )
                    new_published = False
                except BaseException as rollback_exc:
                    try:
                        if new_path is None or os.path.normcase(str(
                            _win32_handle_path(
                                kernel32,
                                new_handle,
                                "post-error failed new destination",
                            )
                        )) != os.path.normcase(str(new_path)):
                            raise rollback_exc
                        new_published = False
                    except BaseException:
                        rollback_failures.append(
                            f"remove failed publication: {rollback_exc}"
                        )
            if (
                old_quarantined
                and old_handle is not None
                and not new_published
                and not old_delete_pending
            ):
                try:
                    _win32_rename_handle(
                        kernel32,
                        old_handle,
                        path,
                        False,
                        "rollback destination",
                    )
                    old_quarantined = False
                except BaseException as rollback_exc:
                    try:
                        if os.path.normcase(str(_win32_handle_path(
                            kernel32,
                            old_handle,
                            "post-error rollback destination",
                        ))) != os.path.normcase(str(path)):
                            raise rollback_exc
                        old_quarantined = False
                    except BaseException:
                        rollback_failures.append(
                            f"restore old destination: {rollback_exc}"
                        )
            if new_handle is not None and not new_published:
                try:
                    # A failed staging object is never authoritative.  Always
                    # zero and flush its exact handle before disposition; a
                    # pre-delete link-count check cannot cover an alias created
                    # in the final window.
                    _win32_scrub_exact_handle(
                        kernel32, new_handle, "failed atomic staging"
                    )
                    _win32_delete_handle(
                        kernel32, new_handle, "failed atomic staging"
                    )
                except BaseException as rollback_exc:
                    rollback_failures.append(
                        f"delete failed staging: {rollback_exc}"
                    )
            for sidecar_handle, sidecar_path in sidecar_handles:
                if sidecar_handle not in sidecars_delete_pending:
                    try:
                        _win32_delete_handle(
                            kernel32,
                            sidecar_handle,
                            f"failed SQLite sidecar sentinel {sidecar_path.name}",
                        )
                    except BaseException as rollback_exc:
                        rollback_failures.append(
                            f"delete SQLite sidecar sentinel {sidecar_path}: "
                            f"{rollback_exc}"
                        )
            if (
                not rollback_failures
                and not new_published
                and not old_quarantined
                and record_handle is not None
                and not record_delete_pending
            ):
                try:
                    _win32_delete_handle(
                        kernel32,
                        record_handle,
                        "rolled-back atomic replacement transaction record",
                    )
                    record_delete_pending = True
                except BaseException as rollback_exc:
                    rollback_failures.append(
                        f"retire rolled-back transaction record: {rollback_exc}"
                    )
            if rollback_failures:
                raise RuntimeError(
                    "Atomic replacement failed and rollback was incomplete: "
                    + "; ".join(rollback_failures)
                    + (f"; rollback={rollback_path}" if rollback_path else "")
                ) from exc
            raise
        finally:
            if publication_guard is not None:
                publication_guard.locks.close()
            if old_handle is not None:
                kernel32.CloseHandle(old_handle)
            if new_handle is not None:
                if not success and not new_published:
                    # Delete disposition was requested in the error path; the
                    # close removes the exact object regardless of path swaps.
                    pass
                kernel32.CloseHandle(new_handle)
            if record_handle is not None:
                kernel32.CloseHandle(record_handle)
            for sidecar_handle, _sidecar_path in sidecar_handles:
                kernel32.CloseHandle(sidecar_handle)
    if committed_security_error is not None:
        raise ValueError(committed_security_error)


def _canonical_sqlite_snapshot_bytes(database_bytes: bytes) -> bytes:
    if not database_bytes.startswith(b"SQLite format 3\x00"):
        raise ValueError("SQLite snapshot source has no valid database header")
    source = sqlite3.connect(":memory:")
    destination = sqlite3.connect(":memory:")
    try:
        if not hasattr(source, "deserialize"):
            raise RuntimeError(
                "This Python SQLite build cannot deserialize an in-memory backup"
            )
        source.deserialize(database_bytes)
        source_check = source.execute("PRAGMA integrity_check").fetchall()
        if source_check != [("ok",)]:
            raise ValueError(f"SQLite snapshot source failed integrity_check: {source_check}")
        source.backup(destination)
        destination_check = destination.execute("PRAGMA integrity_check").fetchall()
        if destination_check != [("ok",)]:
            raise ValueError(
                f"SQLite snapshot failed integrity_check: {destination_check}"
            )
        if not hasattr(destination, "serialize"):
            raise RuntimeError(
                "This Python SQLite build cannot serialize an in-memory backup"
            )
        database_bytes = destination.serialize()
        return database_bytes
    finally:
        destination.close()
        source.close()


def _verify_serialized_sqlite_integrity(database_bytes: bytes) -> None:
    _canonical_sqlite_snapshot_bytes(database_bytes)


def _reject_sqlite_sidecars(db_path: Path, context: str) -> None:
    for suffix in ("-journal", "-wal", "-shm"):
        if os.path.lexists(Path(str(db_path) + suffix)):
            raise ValueError(
                f"{context} has an unsafe SQLite sidecar: {db_path.name}{suffix}"
            )


def verify_dpapi_database_backup_against(
    backup_path: Path,
    db_path: Path,
) -> None:
    if not backup_path.is_absolute() or not db_path.is_absolute():
        raise ValueError("DPAPI backup and source database paths must be absolute")
    # Keep the caller's lexical authority.  _LockedWin32ReadFile anchors each
    # component with no-delete handles and rejects any final-path redirection.
    backup_path = Path(os.path.abspath(backup_path))
    db_path = Path(os.path.abspath(db_path))
    if backup_path.parent != db_path.parent:
        raise ValueError("DPAPI backup and source database must share one directory")
    _reject_pending_atomic_replacement(backup_path)
    _reject_pending_atomic_replacement(db_path)
    _reject_sqlite_sidecars(db_path, "DPAPI backup source")
    with _LockedWin32ReadFile(
        backup_path, "DPAPI backup", require_single_link=True
    ) as locked_backup:
        with _LockedWin32ReadFile(
            db_path, "Source database", require_single_link=True
        ) as locked_database:
            if locked_backup.final_path == locked_database.final_path:
                raise ValueError(
                    "DPAPI backup and source database must be different files"
                )
            if locked_backup.identity == locked_database.identity:
                raise ValueError(
                    "DPAPI backup and source database must not be hard links"
                )
            if locked_backup.final_path is None or locked_database.final_path is None:
                raise RuntimeError("Locked file paths were not resolved")
            if locked_backup.final_path.parent != locked_database.final_path.parent:
                raise ValueError(
                    "DPAPI backup and source database must share one directory"
                )
            backup_content = locked_backup.read_bytes()
            database_content = locked_database.read_bytes()
            protected_snapshot = _read_dpapi_database_backup_content(backup_content)
            protected_canonical = _canonical_sqlite_snapshot_bytes(
                protected_snapshot
            )
            current_canonical = _canonical_sqlite_snapshot_bytes(database_content)
            if not hmac.compare_digest(
                hashlib.sha256(protected_canonical).digest(),
                hashlib.sha256(current_canonical).digest(),
            ):
                raise ValueError(
                    "DPAPI backup does not match the source database logical snapshot"
                )
            if not hmac.compare_digest(
                hashlib.sha256(backup_content).digest(),
                hashlib.sha256(locked_backup.read_bytes()).digest(),
            ) or not hmac.compare_digest(
                hashlib.sha256(database_content).digest(),
                hashlib.sha256(locked_database.read_bytes()).digest(),
            ):
                raise ValueError(
                    "DPAPI backup verification observed an input file change"
                )
    _reject_sqlite_sidecars(db_path, "Verified DPAPI backup source")


def _create_dpapi_database_backup_from_bytes(
    database_bytes: bytes,
    backup_path: Path,
) -> Path:
    backup_path = Path(os.path.abspath(backup_path))
    if not backup_path.parent.is_dir():
        raise ValueError("DPAPI backup parent must already exist")
    parent_guard = _LockedWin32DirectoryChain(
        backup_path.parent, "DPAPI backup creation"
    )
    parent_guard.__enter__()
    try:
        _verify_serialized_sqlite_integrity(database_bytes)
        digest = hashlib.sha256(database_bytes).hexdigest().encode("ascii")
        protected = _dpapi_crypt(database_bytes, DPAPI_BACKUP_PURPOSE, True)
        encoded = base64.urlsafe_b64encode(protected)
        backup_content = (
            DPAPI_BACKUP_MAGIC + digest + b"\n" + encoded + b"\n"
        )
        _recover_pending_atomic_replacement(
            backup_path, reserve_sqlite_sidecars=False
        )
        if os.path.lexists(backup_path):
            with _LockedWin32ReadFile(
                backup_path,
                "Existing recovered DPAPI backup",
                require_single_link=True,
            ) as existing:
                existing_content = existing.read_bytes()
                try:
                    existing_database = _read_dpapi_database_backup_content(
                        existing_content
                    )
                except ValueError as exc:
                    raise FileExistsError(
                        f"DPAPI backup path already exists: {backup_path}"
                    ) from exc
                if not hmac.compare_digest(
                    hashlib.sha256(existing_database).digest(),
                    hashlib.sha256(database_bytes).digest(),
                ):
                    raise FileExistsError(
                        f"DPAPI backup path already exists: {backup_path}"
                    )
                existing.ensure_single_link()
                return backup_path
        _atomic_replace_bytes(
            backup_path, backup_content, require_absent=True
        )
        with _LockedWin32ReadFile(
            backup_path, "Created DPAPI backup", require_single_link=True
        ) as locked:
            verified = _read_dpapi_database_backup_content(locked.read_bytes())
            if not hmac.compare_digest(
                hashlib.sha256(verified).digest(),
                hashlib.sha256(database_bytes).digest(),
            ):
                raise ValueError("DPAPI backup verification failed")
        return backup_path
    finally:
        parent_guard.close()


def _securely_discard_created_dpapi_backup(
    backup_path: Path,
    expected_database_bytes: bytes,
) -> None:
    """Scrub one newly-created, identity-bound backup before removing its name."""
    backup_path = Path(os.path.abspath(backup_path))
    if os.name != "nt":
        raise OSError("Secure DPAPI backup cleanup requires Windows")
    if not backup_path.parent.is_dir():
        raise ValueError("Secure DPAPI backup cleanup parent is missing")
    kernel32 = _LockedWin32ReadFile._kernel32()
    handle: int | None = None
    delete_pending = False
    late_empty_alias = False
    with _LockedWin32DirectoryChain(
        backup_path.parent, "Changed-source DPAPI backup cleanup"
    ) as parent_guard:
        atomic_object = _win32_open_atomic_object(
            kernel32,
            backup_path,
            "changed-source DPAPI backup cleanup",
            writable=True,
        )
        if atomic_object is None:
            raise ValueError(
                "Created DPAPI backup disappeared before secure cleanup"
            )
        handle, backup_content, information = atomic_object
        try:
            parent_guard.ensure_unchanged()
            if information.dwFileAttributes & (
                _LockedWin32ReadFile._FILE_ATTRIBUTE_DIRECTORY
                | _LockedWin32ReadFile._FILE_ATTRIBUTE_REPARSE_POINT
            ):
                raise ValueError(
                    "Created DPAPI backup cleanup target is not a non-reparse file"
                )
            # Refuse to truncate an object that was already multiply linked when
            # the exact cleanup handle was bound.  It could be an unrelated
            # same-content victim linked into the backup name.  Once a verified
            # single-link handle is bound, however, scrubbing is unconditional:
            # aliases created in the final window must be emptied as well.
            if int(information.nNumberOfLinks) != 1:
                raise ValueError(
                    "Created DPAPI backup had a hard-link alias before secure "
                    "cleanup; the canonical backup was preserved"
                )
            protected_database = _read_dpapi_database_backup_content(
                backup_content
            )
            if not hmac.compare_digest(
                hashlib.sha256(protected_database).digest(),
                hashlib.sha256(expected_database_bytes).digest(),
            ):
                raise ValueError(
                    "Created DPAPI backup cleanup target hash did not match"
                )
            current_information = _win32_handle_information(
                kernel32,
                handle,
                "changed-source DPAPI backup cleanup",
            )
            current_size = (
                (int(current_information.nFileSizeHigh) << 32)
                | int(current_information.nFileSizeLow)
            )
            current_content = _win32_read_exact_handle(
                kernel32,
                handle,
                current_size,
                "changed-source DPAPI backup cleanup",
            )
            if current_size != len(backup_content) or not hmac.compare_digest(
                hashlib.sha256(current_content).digest(),
                hashlib.sha256(backup_content).digest(),
            ):
                raise ValueError(
                    "Created DPAPI backup changed before secure cleanup"
                )
            _win32_scrub_exact_handle(
                kernel32, handle, "changed-source DPAPI backup cleanup"
            )
            scrubbed_information = _win32_handle_information(
                kernel32,
                handle,
                "scrubbed changed-source DPAPI backup cleanup",
            )
            scrubbed_size = (
                (int(scrubbed_information.nFileSizeHigh) << 32)
                | int(scrubbed_information.nFileSizeLow)
            )
            if scrubbed_size != 0:
                raise ValueError(
                    "Created DPAPI backup did not scrub to zero bytes"
                )
            _win32_delete_handle(
                kernel32, handle, "changed-source DPAPI backup cleanup"
            )
            delete_pending = True
            disposition_information = _win32_handle_information(
                kernel32,
                handle,
                "delete-pending changed-source DPAPI backup cleanup",
            )
            late_empty_alias = int(disposition_information.nNumberOfLinks) != 0
        finally:
            kernel32.CloseHandle(handle)
    if not delete_pending:
        raise RuntimeError("Created DPAPI backup cleanup did not set disposition")
    if late_empty_alias:
        raise ValueError(
            "Created DPAPI backup gained a late hard-link alias; every bound "
            "alias was scrubbed to empty content"
        )


def create_dpapi_database_backup(db_path: Path, backup_path: Path) -> Path:
    _reject_sqlite_sidecars(db_path, "SQLite backup source")
    with _LockedWin32ReadFile(
        db_path, "SQLite backup source", require_single_link=True
    ) as locked:
        _reject_sqlite_sidecars(db_path, "Locked SQLite backup source")
        source_content = locked.read_bytes()
        canonical_snapshot = _canonical_sqlite_snapshot_bytes(source_content)
        created = _create_dpapi_database_backup_from_bytes(
            canonical_snapshot, backup_path
        )
        _reject_sqlite_sidecars(db_path, "Protected SQLite backup source")
        if not hmac.compare_digest(
            hashlib.sha256(source_content).digest(),
            hashlib.sha256(locked.read_bytes()).digest(),
        ):
            try:
                _securely_discard_created_dpapi_backup(
                    created, canonical_snapshot
                )
            except Exception as cleanup_exc:
                raise RuntimeError(
                    "SQLite backup source changed during protected backup; "
                    "secure exact-handle cleanup reported an unsafe condition: "
                    + str(cleanup_exc)
                ) from cleanup_exc
            raise ValueError("SQLite backup source changed during protected backup")
    _reject_sqlite_sidecars(db_path, "Completed SQLite backup source")
    return created


def restore_dpapi_database_backup(backup_path: Path, db_path: Path, overwrite: bool) -> None:
    backup_path = Path(os.path.abspath(backup_path))
    db_path = Path(os.path.abspath(db_path))
    if os.name != "nt":
        raise OSError("DPAPI restore requires Windows identity guards")
    if not db_path.parent.is_dir():
        raise ValueError(
            "DPAPI restore target parent must already exist as a directory"
        )
    destination_guard = _LockedWin32DirectoryChain(
        db_path.parent, "DPAPI restore destination"
    )
    destination_guard.__enter__()
    try:
        _restore_dpapi_database_backup_anchored(
            backup_path, db_path, overwrite, destination_guard
        )
    finally:
        destination_guard.close()


def _restore_dpapi_database_backup_anchored(
    backup_path: Path,
    db_path: Path,
    overwrite: bool,
    destination_guard: _LockedWin32DirectoryChain,
) -> None:
    database_bytes = _read_dpapi_database_backup(backup_path)
    _verify_serialized_sqlite_integrity(database_bytes)
    destination_guard.ensure_unchanged()
    _recover_pending_atomic_replacement(
        db_path, reserve_sqlite_sidecars=True
    )
    _reject_sqlite_sidecars(db_path, "DPAPI restore destination")
    if db_path.exists() and not overwrite:
        raise SystemExit(f"Target database already exists; use --overwrite to restore: {db_path}")
    if db_path.exists():
        with _LockedWin32ReadFile(
            db_path,
            "DPAPI restore destination",
            require_single_link=True,
        ) as locked_database:
            _reject_sqlite_sidecars(
                db_path, "Locked DPAPI restore destination"
            )
            source_content = locked_database.read_bytes()
            source_snapshot = _canonical_sqlite_snapshot_bytes(
                source_content
            )
            timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            rollback = db_path.with_name(
                f"{db_path.name}.pre_restore_{timestamp}.dpapi.bak"
            )
            destination_guard.ensure_unchanged()
            _create_dpapi_database_backup_from_bytes(source_snapshot, rollback)
            locked_database.ensure_single_link()
            if not hmac.compare_digest(
                source_content, locked_database.read_bytes()
            ):
                raise ValueError("DPAPI restore destination changed after backup")
            source_identity = locked_database.identity
            if source_identity is None:
                raise RuntimeError("DPAPI restore destination identity is missing")
            locked_database.ensure_single_link()
        destination_guard.ensure_unchanged()
        _atomic_replace_bytes(
            db_path,
            database_bytes,
            expected_existing=source_content,
            sqlite_sidecar_base=db_path,
            expected_existing_identity=source_identity,
        )
        print(f"Protected current database before restore: {rollback}")
    else:
        destination_guard.ensure_unchanged()
        _atomic_replace_bytes(
            db_path,
            database_bytes,
            require_absent=True,
            sqlite_sidecar_base=db_path,
        )
    _reject_sqlite_sidecars(db_path, "Restored DPAPI database")
    print(f"Restored DPAPI database backup: {db_path}")


def _debug_transfer_crypto():
    try:
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import (
            padding as asymmetric_padding,
            rsa,
        )
        from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    except ImportError as exc:
        raise RuntimeError(
            "Debug database transfer requires the bundled cryptography runtime"
        ) from exc
    return hashes, serialization, asymmetric_padding, rsa, AESGCM


def _strict_base64_decode(text: object, label: str) -> bytes:
    if not isinstance(text, str) or not text:
        raise ValueError(f"{label} is missing")
    try:
        return base64.b64decode(text.encode("ascii"), validate=True)
    except (UnicodeEncodeError, binascii.Error, ValueError) as exc:
        raise ValueError(f"{label} is not canonical base64") from exc


def _debug_transfer_public_fingerprint(public_key: object) -> str:
    _hashes, serialization, _padding, _rsa, _aesgcm = _debug_transfer_crypto()
    public_der = public_key.public_bytes(
        serialization.Encoding.DER,
        serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    return hashlib.sha256(public_der).hexdigest()


def _read_locked_file(path: Path, label: str) -> bytes:
    lexical = _absolute_lexical_path(path)
    _absolute_regular_nonreparse_file(lexical, label)
    _reject_pending_atomic_replacement(lexical)
    with _LockedWin32ReadFile(
        lexical, label, require_single_link=True
    ) as locked:
        content = locked.read_bytes()
        locked.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(content).digest(),
            hashlib.sha256(locked.read_bytes()).digest(),
        ):
            raise ValueError(f"{label} changed during read-back")
        return content


def _new_debug_transfer_path(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} path must be absolute")
    lexical = _absolute_lexical_path(path)
    if not lexical.parent.is_dir():
        raise ValueError(f"{label} parent directory does not exist")
    if os.path.lexists(lexical):
        raise FileExistsError(f"{label} already exists: {lexical}")
    _reject_pending_atomic_replacement(lexical)
    return lexical


def create_debug_transfer_key(
    public_key_path: Path,
    private_key_path: Path,
) -> None:
    public_path = _new_debug_transfer_path(
        public_key_path, "Debug transfer public key"
    )
    private_path = _new_debug_transfer_path(
        private_key_path, "Debug transfer private key"
    )
    if _same_lexical_path(public_path, private_path):
        raise ValueError("Debug transfer public and private key paths must differ")

    _hashes, serialization, _padding, rsa, _aesgcm = _debug_transfer_crypto()
    private_key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
    public_key = private_key.public_key()
    public_bytes = public_key.public_bytes(
        serialization.Encoding.PEM,
        serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    private_der = private_key.private_bytes(
        serialization.Encoding.DER,
        serialization.PrivateFormat.PKCS8,
        serialization.NoEncryption(),
    )
    fingerprint = _debug_transfer_public_fingerprint(public_key)
    protected_private = _dpapi_crypt(
        private_der, DEBUG_TRANSFER_PRIVATE_KEY_PURPOSE, True
    )
    private_content = (
        DEBUG_TRANSFER_PRIVATE_KEY_MAGIC
        + fingerprint.encode("ascii")
        + b"\n"
        + base64.b64encode(protected_private)
        + b"\n"
    )

    _atomic_replace_bytes(public_path, public_bytes, require_absent=True)
    _atomic_replace_bytes(private_path, private_content, require_absent=True)
    if not hmac.compare_digest(
        hashlib.sha256(_read_locked_file(
            public_path, "Created debug transfer public key"
        )).digest(),
        hashlib.sha256(public_bytes).digest(),
    ) or not hmac.compare_digest(
        hashlib.sha256(_read_locked_file(
            private_path, "Created debug transfer private key"
        )).digest(),
        hashlib.sha256(private_content).digest(),
    ):
        raise ValueError("Debug transfer key read-back verification failed")

    print(f"Created debug transfer public key: {public_path}")
    print(f"Created CurrentUser-DPAPI private key: {private_path}")
    print(f"Debug transfer recipient SHA256: {fingerprint}")


def _load_debug_transfer_public_key(path: Path) -> tuple[object, str]:
    content = _read_locked_file(path, "Debug transfer public key")
    _hashes, serialization, _padding, rsa, _aesgcm = _debug_transfer_crypto()
    try:
        public_key = serialization.load_pem_public_key(content)
    except (TypeError, ValueError) as exc:
        raise ValueError("Debug transfer public key cannot be parsed") from exc
    if (
        not isinstance(public_key, rsa.RSAPublicKey)
        or public_key.key_size < 3072
        or public_key.public_numbers().e != 65537
    ):
        raise ValueError("Debug transfer public key must be RSA-3072 or stronger")
    return public_key, _debug_transfer_public_fingerprint(public_key)


def _load_debug_transfer_private_key(path: Path) -> tuple[object, str]:
    content = _read_locked_file(path, "Debug transfer private key")
    if not content.startswith(DEBUG_TRANSFER_PRIVATE_KEY_MAGIC):
        raise ValueError("Not a supported debug transfer private key")
    lines = content[len(DEBUG_TRANSFER_PRIVATE_KEY_MAGIC):].splitlines()
    if (
        len(lines) != 2
        or re.fullmatch(rb"[0-9a-f]{64}", lines[0]) is None
    ):
        raise ValueError("Debug transfer private key envelope is invalid")
    protected = _strict_base64_decode(
        lines[1].decode("ascii", errors="strict"),
        "Debug transfer private key payload",
    )
    try:
        private_der = _dpapi_crypt(
            protected, DEBUG_TRANSFER_PRIVATE_KEY_PURPOSE, False
        )
    except OSError as exc:
        raise ValueError(
            "Debug transfer private key is not bound to this Windows user"
        ) from exc

    _hashes, serialization, _padding, rsa, _aesgcm = _debug_transfer_crypto()
    try:
        private_key = serialization.load_der_private_key(
            private_der, password=None
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("Debug transfer private key cannot be parsed") from exc
    if (
        not isinstance(private_key, rsa.RSAPrivateKey)
        or private_key.key_size < 3072
        or private_key.public_key().public_numbers().e != 65537
    ):
        raise ValueError("Debug transfer private key must be RSA-3072 or stronger")
    fingerprint = _debug_transfer_public_fingerprint(private_key.public_key())
    if not hmac.compare_digest(fingerprint, lines[0].decode("ascii")):
        raise ValueError("Debug transfer private key fingerprint mismatch")
    return private_key, fingerprint


def _debug_transfer_known_auth_drift(database_bytes: bytes) -> bool:
    snapshot = _canonical_sqlite_snapshot_bytes(database_bytes)
    with closing(sqlite3.connect(":memory:")) as connection:
        if not hasattr(connection, "deserialize"):
            raise RuntimeError(
                "This Python SQLite build cannot inspect a private debug snapshot"
            )
        connection.deserialize(snapshot)
        return _has_known_auth3_login_preference_type_drift(connection)


def _prepare_debug_transfer_snapshot(
    source_content: bytes,
    source_sha256: str,
    recipient_sha256: str,
) -> tuple[bytes, int]:
    snapshot = _canonical_sqlite_snapshot_bytes(source_content)
    with closing(sqlite3.connect(":memory:")) as connection:
        if not hasattr(connection, "deserialize") or not hasattr(
            connection, "serialize"
        ):
            raise RuntimeError(
                "This Python SQLite build cannot prepare a private debug snapshot"
            )
        connection.deserialize(snapshot)
        try:
            connection.execute("BEGIN IMMEDIATE")
            metadata = dict(connection.execute(
                "SELECT key, value FROM meta"
            ).fetchall())
            if (
                current_schema_version(connection) != SCHEMA_VERSION
                or not has_current_schema(connection)
                or str(metadata.get("sensitive_protection", ""))
                != "dpapi-current-user-v1"
                or any(key in metadata for key in DEBUG_TRANSFER_META_KEYS)
            ):
                raise ValueError(
                    "Source database is not a clean current ConfigStore"
                )

            transferred = 0
            rows = connection.execute(
                """
                SELECT scope_type, scope_id, module, key_name, value_text
                FROM settings
                ORDER BY scope_type, scope_id, module, key_name
                """
            ).fetchall()
            for scope_type, scope_id, module, key, stored in rows:
                stored_text_value = str(stored)
                if not stored_text_value.startswith(DPAPI_PREFIX):
                    continue
                purpose = protection_purpose(
                    str(scope_type), str(scope_id), str(module), str(key)
                )
                plain = unprotect_sensitive_text(stored_text_value, purpose)
                if plain is None:
                    raise ValueError(
                        "A field DPAPI setting cannot be decrypted by the "
                        "current Windows user"
                    )
                transfer_value = (
                    DEBUG_TRANSFER_VALUE_PREFIX
                    + base64.b64encode(plain.encode("utf-8")).decode("ascii")
                )
                cursor = connection.execute(
                    """
                    UPDATE settings SET value_text=?
                    WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?
                      AND value_text=?
                    """,
                    (
                        transfer_value,
                        str(scope_type),
                        str(scope_id),
                        str(module),
                        str(key),
                        stored_text_value,
                    ),
                )
                if cursor.rowcount != 1:
                    raise ValueError(
                        "A field DPAPI setting changed during transfer preparation"
                    )
                transferred += 1

            connection.execute(
                "UPDATE meta SET value=? WHERE key='sensitive_protection'",
                (DEBUG_TRANSFER_PROTECTION,),
            )
            transfer_meta = {
                "debug_transfer_format": DEBUG_TRANSFER_PACKAGE_FORMAT,
                "debug_transfer_recipient_sha256": recipient_sha256,
                "debug_transfer_setting_count": str(transferred),
                "debug_transfer_source_sha256": source_sha256,
            }
            for key, value in transfer_meta.items():
                connection.execute(
                    "INSERT INTO meta(key, value) VALUES(?, ?)",
                    (key, value),
                )
            remaining_dpapi = connection.execute(
                "SELECT COUNT(*) FROM settings WHERE value_text LIKE ?",
                (DPAPI_PREFIX + "%",),
            ).fetchone()
            if remaining_dpapi is None or int(remaining_dpapi[0]) != 0:
                raise ValueError(
                    "Debug transfer snapshot retains field DPAPI values"
                )
            integrity = connection.execute("PRAGMA integrity_check").fetchall()
            if integrity != [("ok",)]:
                raise ValueError(
                    f"Debug transfer snapshot failed integrity_check: {integrity}"
                )
            connection.commit()
            return (
                _canonical_sqlite_snapshot_bytes(connection.serialize()),
                transferred,
            )
        except BaseException:
            connection.rollback()
            raise


def export_debug_database(
    db_path: Path,
    package_path: Path,
    public_key_path: Path,
) -> None:
    database_path = _absolute_lexical_path(db_path)
    package = _new_debug_transfer_path(
        package_path, "Debug database transfer package"
    )
    public_key, recipient_sha256 = _load_debug_transfer_public_key(
        public_key_path
    )
    _absolute_regular_nonreparse_file(
        database_path, "Field ConfigStore database"
    )
    _reject_pending_atomic_replacement(database_path)
    _reject_sqlite_sidecars(database_path, "Field ConfigStore database")

    with _LockedWin32ReadFile(
        database_path,
        "Field ConfigStore database",
        require_single_link=True,
    ) as locked_database:
        source_content = locked_database.read_bytes()
        source_sha256 = hashlib.sha256(source_content).hexdigest()
        known_drift = _debug_transfer_known_auth_drift(source_content)
        verify_current_database(
            database_path,
            allow_known_auth3_login_preference_drift=known_drift,
        )
        locked_database.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(source_content).digest(),
            hashlib.sha256(locked_database.read_bytes()).digest(),
        ):
            raise ValueError(
                "Field ConfigStore changed during debug export"
            )
        portable_database, transferred = _prepare_debug_transfer_snapshot(
            source_content, source_sha256, recipient_sha256
        )
        locked_database.ensure_single_link()

    hashes, _serialization, asymmetric_padding, _rsa, AESGCM = (
        _debug_transfer_crypto()
    )
    header = {
        "createdUtc": _dt.datetime.now(_dt.timezone.utc)
        .isoformat(timespec="seconds")
        .replace("+00:00", "Z"),
        "format": DEBUG_TRANSFER_PACKAGE_FORMAT,
        "knownAuth3LoginPreferenceDrift": known_drift,
        "portableDatabaseSha256": hashlib.sha256(
            portable_database
        ).hexdigest(),
        "recipientKeySha256": recipient_sha256,
        "sourceDatabaseSha256": source_sha256,
        "transferredSettingCount": transferred,
    }
    associated_data = json.dumps(
        header, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("ascii")
    content_key = os.urandom(32)
    nonce = os.urandom(12)
    ciphertext = AESGCM(content_key).encrypt(
        nonce, portable_database, associated_data
    )
    wrapped_key = public_key.encrypt(
        content_key,
        asymmetric_padding.OAEP(
            mgf=asymmetric_padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=DEBUG_TRANSFER_PACKAGE_FORMAT.encode("ascii"),
        ),
    )
    payload = dict(header)
    payload.update({
        "ciphertext": base64.b64encode(ciphertext).decode("ascii"),
        "nonce": base64.b64encode(nonce).decode("ascii"),
        "wrappedKey": base64.b64encode(wrapped_key).decode("ascii"),
    })
    package_content = (
        DEBUG_TRANSFER_PACKAGE_MAGIC
        + json.dumps(
            payload,
            ensure_ascii=True,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("ascii")
        + b"\n"
    )
    _atomic_replace_bytes(package, package_content, require_absent=True)
    if not hmac.compare_digest(
        hashlib.sha256(_read_locked_file(
            package, "Created debug database transfer package"
        )).digest(),
        hashlib.sha256(package_content).digest(),
    ):
        raise ValueError("Debug database transfer package read-back failed")

    print(f"Created encrypted debug database package: {package}")
    print(f"Source database SHA256: {source_sha256}")
    print(f"Transferred DPAPI settings: {transferred}")
    print(f"Debug transfer recipient SHA256: {recipient_sha256}")


def _decrypt_debug_database_package(
    package_path: Path,
    private_key_path: Path,
) -> tuple[bytes, dict[str, object]]:
    content = _read_locked_file(
        package_path, "Debug database transfer package"
    )
    if not content.startswith(DEBUG_TRANSFER_PACKAGE_MAGIC):
        raise ValueError("Not a supported debug database transfer package")
    try:
        payload = json.loads(
            content[len(DEBUG_TRANSFER_PACKAGE_MAGIC):].decode("ascii")
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("Debug database transfer package is invalid") from exc
    expected_keys = {
        "ciphertext",
        "createdUtc",
        "format",
        "knownAuth3LoginPreferenceDrift",
        "nonce",
        "portableDatabaseSha256",
        "recipientKeySha256",
        "sourceDatabaseSha256",
        "transferredSettingCount",
        "wrappedKey",
    }
    if not isinstance(payload, dict) or set(payload) != expected_keys:
        raise ValueError("Debug database transfer package fields are invalid")
    if (
        payload.get("format") != DEBUG_TRANSFER_PACKAGE_FORMAT
        or not isinstance(payload.get("createdUtc"), str)
        or not isinstance(
            payload.get("knownAuth3LoginPreferenceDrift"), bool
        )
        or not isinstance(payload.get("transferredSettingCount"), int)
        or int(payload["transferredSettingCount"]) < 0
    ):
        raise ValueError("Debug database transfer package header is invalid")
    for key in (
        "portableDatabaseSha256",
        "recipientKeySha256",
        "sourceDatabaseSha256",
    ):
        if (
            not isinstance(payload.get(key), str)
            or re.fullmatch(r"[0-9a-f]{64}", str(payload[key])) is None
        ):
            raise ValueError(
                f"Debug database transfer package {key} is invalid"
            )

    private_key, private_fingerprint = _load_debug_transfer_private_key(
        private_key_path
    )
    if not hmac.compare_digest(
        private_fingerprint, str(payload["recipientKeySha256"])
    ):
        raise ValueError(
            "Debug database package was encrypted for a different recipient key"
        )
    header = {
        key: payload[key]
        for key in (
            "createdUtc",
            "format",
            "knownAuth3LoginPreferenceDrift",
            "portableDatabaseSha256",
            "recipientKeySha256",
            "sourceDatabaseSha256",
            "transferredSettingCount",
        )
    }
    associated_data = json.dumps(
        header, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("ascii")
    wrapped_key = _strict_base64_decode(
        payload["wrappedKey"], "Debug package wrapped key"
    )
    nonce = _strict_base64_decode(
        payload["nonce"], "Debug package nonce"
    )
    ciphertext = _strict_base64_decode(
        payload["ciphertext"], "Debug package ciphertext"
    )
    if len(nonce) != 12:
        raise ValueError("Debug package AES-GCM nonce length is invalid")

    hashes, _serialization, asymmetric_padding, _rsa, AESGCM = (
        _debug_transfer_crypto()
    )
    try:
        content_key = private_key.decrypt(
            wrapped_key,
            asymmetric_padding.OAEP(
                mgf=asymmetric_padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=DEBUG_TRANSFER_PACKAGE_FORMAT.encode("ascii"),
            ),
        )
        portable_database = AESGCM(content_key).decrypt(
            nonce, ciphertext, associated_data
        )
    except Exception as exc:
        raise ValueError(
            "Debug database package authentication or decryption failed"
        ) from exc
    if not hmac.compare_digest(
        hashlib.sha256(portable_database).hexdigest(),
        str(payload["portableDatabaseSha256"]),
    ):
        raise ValueError("Debug database package plaintext hash mismatch")
    _verify_serialized_sqlite_integrity(portable_database)
    return portable_database, payload


def _rebind_debug_transfer_snapshot(
    portable_database: bytes,
    package_header: dict[str, object],
) -> tuple[bytes, int, bool]:
    snapshot = _canonical_sqlite_snapshot_bytes(portable_database)
    with closing(sqlite3.connect(":memory:")) as connection:
        if not hasattr(connection, "deserialize") or not hasattr(
            connection, "serialize"
        ):
            raise RuntimeError(
                "This Python SQLite build cannot rebind a private debug snapshot"
            )
        connection.deserialize(snapshot)
        try:
            connection.execute("BEGIN IMMEDIATE")
            metadata = dict(connection.execute(
                "SELECT key, value FROM meta"
            ).fetchall())
            expected_count = int(package_header["transferredSettingCount"])
            if (
                current_schema_version(connection) != SCHEMA_VERSION
                or not has_current_schema(connection)
                or str(metadata.get("sensitive_protection", ""))
                != DEBUG_TRANSFER_PROTECTION
                or str(metadata.get("debug_transfer_format", ""))
                != DEBUG_TRANSFER_PACKAGE_FORMAT
                or str(metadata.get("debug_transfer_recipient_sha256", ""))
                != str(package_header["recipientKeySha256"])
                or str(metadata.get("debug_transfer_source_sha256", ""))
                != str(package_header["sourceDatabaseSha256"])
                or str(metadata.get("debug_transfer_setting_count", ""))
                != str(expected_count)
            ):
                raise ValueError(
                    "Debug database snapshot provenance is incomplete"
                )
            if connection.execute(
                "SELECT COUNT(*) FROM settings WHERE value_text LIKE ?",
                (DPAPI_PREFIX + "%",),
            ).fetchone() != (0,):
                raise ValueError(
                    "Debug database snapshot contains foreign DPAPI values"
                )

            rows = connection.execute(
                """
                SELECT scope_type, scope_id, module, key_name, value_text
                FROM settings
                WHERE value_text LIKE ?
                ORDER BY scope_type, scope_id, module, key_name
                """,
                (DEBUG_TRANSFER_VALUE_PREFIX + "%",),
            ).fetchall()
            if len(rows) != expected_count:
                raise ValueError(
                    "Debug database transferred-setting count mismatch"
                )
            for scope_type, scope_id, module, key, stored in rows:
                stored_text_value = str(stored)
                encoded = stored_text_value[len(DEBUG_TRANSFER_VALUE_PREFIX):]
                plain_bytes = _strict_base64_decode(
                    encoded, "Debug transferred setting"
                )
                try:
                    plain = plain_bytes.decode("utf-8")
                except UnicodeDecodeError as exc:
                    raise ValueError(
                        "Debug transferred setting is not UTF-8"
                    ) from exc
                purpose = protection_purpose(
                    str(scope_type), str(scope_id), str(module), str(key)
                )
                local_dpapi = protect_sensitive_text(plain, purpose)
                cursor = connection.execute(
                    """
                    UPDATE settings SET value_text=?, encrypted=1
                    WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?
                      AND value_text=?
                    """,
                    (
                        local_dpapi,
                        str(scope_type),
                        str(scope_id),
                        str(module),
                        str(key),
                        stored_text_value,
                    ),
                )
                if cursor.rowcount != 1:
                    raise ValueError(
                        "A debug transferred setting changed during local rebind"
                    )

            connection.execute(
                "UPDATE meta SET value='dpapi-current-user-v1' "
                "WHERE key='sensitive_protection'"
            )
            connection.executemany(
                "DELETE FROM meta WHERE key=?",
                [(key,) for key in sorted(DEBUG_TRANSFER_META_KEYS)],
            )
            remaining_transfer = connection.execute(
                "SELECT COUNT(*) FROM settings WHERE value_text LIKE ?",
                (DEBUG_TRANSFER_VALUE_PREFIX + "%",),
            ).fetchone()
            if remaining_transfer != (0,):
                raise ValueError(
                    "Local debug database retains transfer plaintext markers"
                )
            integrity = connection.execute("PRAGMA integrity_check").fetchall()
            if integrity != [("ok",)]:
                raise ValueError(
                    f"Local debug database failed integrity_check: {integrity}"
                )
            known_drift = _has_known_auth3_login_preference_type_drift(
                connection
            )
            if known_drift != bool(
                package_header["knownAuth3LoginPreferenceDrift"]
            ):
                raise ValueError(
                    "Debug database authentication-drift provenance changed"
                )
            connection.commit()
            return (
                _canonical_sqlite_snapshot_bytes(connection.serialize()),
                len(rows),
                known_drift,
            )
        except BaseException:
            connection.rollback()
            raise


def import_debug_database(
    package_path: Path,
    db_path: Path,
    private_key_path: Path,
) -> None:
    database_path = _new_debug_transfer_path(
        db_path, "Local debug ConfigStore database"
    )
    _reject_sqlite_sidecars(
        database_path, "Local debug ConfigStore database"
    )
    portable_database, header = _decrypt_debug_database_package(
        package_path, private_key_path
    )
    local_database, rebound_count, known_drift = (
        _rebind_debug_transfer_snapshot(portable_database, header)
    )
    staging_path = database_path.with_name(
        f".{database_path.name}.debug-import-{os.urandom(16).hex()}.tmp"
    )
    _new_debug_transfer_path(
        staging_path, "Local debug ConfigStore verification staging"
    )
    try:
        _atomic_replace_bytes(
            staging_path,
            local_database,
            require_absent=True,
            sqlite_sidecar_base=staging_path,
        )
        verify_current_database(
            staging_path,
            allow_known_auth3_login_preference_drift=known_drift,
        )
        staged_content = _read_locked_file(
            staging_path, "Verified local debug ConfigStore staging"
        )
        if not hmac.compare_digest(
            hashlib.sha256(staged_content).digest(),
            hashlib.sha256(local_database).digest(),
        ):
            raise ValueError(
                "Local debug ConfigStore staging bytes changed"
            )
        _atomic_replace_bytes(
            database_path,
            staged_content,
            require_absent=True,
            sqlite_sidecar_base=database_path,
        )
        verify_current_database(
            database_path,
            allow_known_auth3_login_preference_drift=known_drift,
        )
    finally:
        if os.path.lexists(staging_path):
            os.unlink(staging_path)

    print(f"Imported locally rebound debug database: {database_path}")
    print(f"Source database SHA256: {header['sourceDatabaseSha256']}")
    print(f"Rebound DPAPI settings: {rebound_count}")
    if known_drift:
        print(
            "Imported database retains the known auth3 login-preference "
            "type drift so the local automatic-repair path can be reproduced."
        )


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


def _table_schema_signature(
    conn: sqlite3.Connection,
    table_name: str,
) -> tuple[tuple[str, str, int, str | None, int], ...]:
    return tuple(
        (
            str(row[1]),
            str(row[2]).upper(),
            int(row[3]),
            None if row[4] is None else str(row[4]),
            int(row[5]),
        )
        for row in conn.execute(f"PRAGMA table_info({table_name})").fetchall()
    )


def _index_key_signature(
    conn: sqlite3.Connection,
    index_name: str,
) -> tuple[tuple[str, str, int], ...]:
    quoted_name = index_name.replace('"', '""')
    rows = conn.execute(
        f'PRAGMA index_xinfo("{quoted_name}")'
    ).fetchall()
    signature: list[tuple[str, str, int]] = []
    for row in rows:
        if int(row[5]) != 1:
            continue
        if row[2] is None or int(row[1]) < 0:
            return (("<invalid-index-expression>", "", 0),)
        signature.append((
            str(row[2]),
            str(row[4]).upper(),
            int(row[3]),
        ))
    return tuple(signature)


def _has_safe_current_schema_objects(conn: sqlite3.Connection) -> bool:
    expected_table_sql = {
        "meta": "create table meta(key text primary key,value text not null)",
        "settings": (
            "create table settings("
            "scope_type text not null,"
            "scope_id text not null default '',"
            "module text not null,"
            "key_name text not null,"
            "value_text text not null,"
            "value_type text not null default 'string',"
            "sensitive integer not null default 0,"
            "encrypted integer not null default 0,"
            "updated_at text not null default current_timestamp,"
            "primary key(scope_type,scope_id,module,key_name))"
        ),
    }
    for table_name, expected_sql in expected_table_sql.items():
        row = conn.execute(
            "SELECT sql FROM sqlite_master WHERE type='table' AND name=?",
            (table_name,),
        ).fetchone()
        if (
            row is None
            or not isinstance(row[0], str)
            or re.sub(r"\s+", "", row[0]).casefold()
            != re.sub(r"\s+", "", expected_sql).casefold()
        ):
            return False
    trigger = conn.execute(
        """
        SELECT 1 FROM sqlite_master
        WHERE type='trigger'
        LIMIT 1
        """
    ).fetchone()
    if trigger is not None:
        return False

    expected_primary_keys = {
        "meta": (("key", "BINARY", 0),),
        "settings": (
            ("scope_type", "BINARY", 0),
            ("scope_id", "BINARY", 0),
            ("module", "BINARY", 0),
            ("key_name", "BINARY", 0),
        ),
    }
    for table_name, expected_primary_key in expected_primary_keys.items():
        indexes = conn.execute(
            f"PRAGMA index_list({table_name})"
        ).fetchall()
        primary_indexes = [
            row for row in indexes
            if int(row[2]) == 1 and str(row[3]).casefold() == "pk"
            and int(row[4]) == 0
        ]
        if (
            len(primary_indexes) != 1
            or _index_key_signature(
                conn, str(primary_indexes[0][1])
            ) != expected_primary_key
        ):
            return False
        for row in indexes:
            index_name = str(row[1])
            unique = int(row[2])
            origin = str(row[3]).casefold()
            partial = int(row[4])
            if origin == "pk":
                continue
            if table_name == "settings" and index_name == "idx_settings_scope":
                if (
                    unique != 0
                    or origin != "c"
                    or partial != 0
                    or _index_key_signature(conn, index_name) != (
                        ("scope_type", "BINARY", 0),
                        ("scope_id", "BINARY", 0),
                        ("module", "BINARY", 0),
                    )
                ):
                    return False
                continue
            return False
    return True


def has_current_schema(conn: sqlite3.Connection) -> bool:
    expected_meta = (
        ("key", "TEXT", 0, None, 1),
        ("value", "TEXT", 1, None, 0),
    )
    expected_settings = (
        ("scope_type", "TEXT", 1, None, 1),
        ("scope_id", "TEXT", 1, "''", 2),
        ("module", "TEXT", 1, None, 3),
        ("key_name", "TEXT", 1, None, 4),
        ("value_text", "TEXT", 1, None, 0),
        ("value_type", "TEXT", 1, "'string'", 0),
        ("sensitive", "INTEGER", 1, "0", 0),
        ("encrypted", "INTEGER", 1, "0", 0),
        ("updated_at", "TEXT", 1, "CURRENT_TIMESTAMP", 0),
    )
    return (
        table_exists(conn, "meta")
        and table_exists(conn, "settings")
        and _table_schema_signature(conn, "meta") == expected_meta
        and _table_schema_signature(conn, "settings") == expected_settings
        and _has_safe_current_schema_objects(conn)
    )


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
    allow_legacy_v4_wrapped: bool = False,
) -> None:
    if field not in ACCOUNT_PROFILE_PORTABLE_FIELDS:
        raise ValueError("Unsupported portable account/Profile field")
    if (
        encrypted in (0, "0", False)
        and not stored.startswith(DPAPI_PREFIX)
        and not stored.startswith("enc:v1:")
    ):
        return
    if (
        allow_legacy_v4_wrapped
        and encrypted in (1, "1", True)
        and stored.startswith("enc:v1:")
        and not stored.startswith(DPAPI_PREFIX)
    ):
        return
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
    allow_legacy_v4_wrapped: bool = False,
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
                str(stored), encrypted, key_text, "Existing",
                allow_legacy_v4_wrapped=allow_legacy_v4_wrapped,
            )
            if key_text in {
                "PasswordHash", "Role", "MustChangePassword",
                "CreatedAt", "UpdatedAt", "PasswordChangedAt",
            }:
                stored_values[key_text] = (str(stored), encrypted)
        if "PasswordHash" not in stored_values or "Role" not in stored_values:
            raise ValueError("Existing account/Profile has no complete password/role record")

        decoded_values: dict[str, str] = {}
        for key, (stored, encrypted) in stored_values.items():
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
        if "MustChangePassword" in decoded_values:
            parsed = _parse_authentication_bool(decoded_values["MustChangePassword"])
            if parsed is None:
                raise ValueError(
                    "Existing account/Profile has an invalid MustChangePassword"
                )
            must_change_password = parsed
        if _is_known_bootstrap_password_record(account_id, password_record):
            must_change_password = True

        for time_field in ("CreatedAt", "UpdatedAt", "PasswordChangedAt"):
            if time_field not in decoded_values:
                continue
            _account_iso_instant(decoded_values[time_field], time_field)

        normalized_values: list[tuple[str, str, str]] = [
            ("PasswordHash", password_record, "string"),
            ("Role", role, "string"),
            (
                "MustChangePassword",
                "1" if must_change_password else "0",
                "bool",
            ),
        ]
        normalized_values.extend(
            (time_field, decoded_values[time_field], "datetime")
            for time_field in ("CreatedAt", "UpdatedAt", "PasswordChangedAt")
            if time_field in decoded_values
        )
        for key, value, value_type in normalized_values:
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
    login_settings = "LoginState/Settings"
    saved_passwords = "LoginState/SavedPasswords"
    remembered_credentials = "LoginState/RememberedCredentials"
    rows = conn.execute(
        """
        SELECT DISTINCT module FROM settings
        WHERE scope_type='global' AND scope_id='' AND (
            lower(module)=lower(?) OR lower(module) LIKE lower(?) OR
            lower(module)=lower(?) OR lower(module)=lower(?) OR
            lower(module) LIKE lower(?) OR
            lower(module) LIKE lower(?)
        )
        """,
        (
            account_base,
            LEGACY_ACCOUNT_MODULE_PREFIX + "%",
            login_general,
            login_settings,
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
        elif folded == login_settings.casefold():
            canonical = module == login_settings
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
    allow_legacy_v4_wrapped_profiles: bool = False,
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
                allow_legacy_v4_wrapped=allow_legacy_v4_wrapped_profiles,
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

    account_counts = _normalize_existing_account_profiles(
        conn,
        encrypt,
        allow_legacy_v4_wrapped=allow_legacy_v4_wrapped_profiles,
    )

    canonical_login_fields = {
        field.casefold(): field
        for field in (
            "UserName",
            "AccountHistory",
            "RememberPassword",
            "AutoLogin",
            "PasswordBase64",
        )
    }
    portable_login_fields = {
        "UserName": "string",
        "AccountHistory": "list",
    }
    migrated_login_values: dict[str, str] = {}
    for (
        login_scope_type,
        login_scope_id,
        login_module,
        key,
        stored,
        encrypted,
    ) in conn.execute(
        """
        SELECT scope_type, scope_id, module, key_name, value_text, encrypted
        FROM settings
        WHERE lower(module) IN (
            lower('LoginState/General'), lower('LoginState/Settings')
        )
        ORDER BY module, key_name
        """
    ).fetchall():
        module_text = str(login_module)
        if (
            str(login_scope_type) != "global"
            or str(login_scope_id) != ""
            or module_text not in {"LoginState/General", "LoginState/Settings"}
        ):
            raise ValueError("Legacy login-state module has non-canonical casing")
        key_text = str(key)
        canonical_key = canonical_login_fields.get(key_text.casefold())
        if canonical_key is None:
            raise ValueError("Legacy login-state module has an unsupported field")
        if key_text != canonical_key:
            raise ValueError("Legacy login-state field has non-canonical casing")
        if canonical_key not in portable_login_fields:
            continue
        decoded = decode_stored_text(
            str(stored), encrypted,
            protection_purpose("global", "", module_text, canonical_key),
        )
        if decoded is None:
            raise ValueError("Cannot decode legacy login state")
        previous = migrated_login_values.get(canonical_key)
        if previous is not None and previous != decoded:
            raise ValueError("Conflicting legacy login-state field")
        migrated_login_values[canonical_key] = decoded

    if conn.execute(
        """
        SELECT 1 FROM settings
        WHERE lower(module)=lower('LoginState') AND module<>'LoginState'
        LIMIT 1
        """
    ).fetchone() is not None:
        raise ValueError("Target login-state module has non-canonical casing")

    existing_login_keys: set[str] = set()
    for (
        login_scope_type,
        login_scope_id,
        login_module,
        key,
        stored,
        encrypted,
    ) in conn.execute(
        """
        SELECT scope_type, scope_id, module, key_name, value_text, encrypted
        FROM settings
        WHERE lower(module)=lower('LoginState')
          AND lower(key_name) IN (
              lower('UserName'), lower('AccountHistory'),
              lower('RememberPassword'), lower('AutoLogin'),
              lower('PasswordBase64')
          )
        ORDER BY module, key_name
        """
    ).fetchall():
        module_text = str(login_module)
        key_text = str(key)
        canonical_key = canonical_login_fields.get(key_text.casefold())
        if (
            str(login_scope_type) != "global"
            or str(login_scope_id) != ""
            or module_text != "LoginState"
            or canonical_key is None
            or key_text != canonical_key
        ):
            raise ValueError("Target login-state field has non-canonical casing")
        if canonical_key not in portable_login_fields:
            continue
        decoded = decode_stored_text(
            str(stored), encrypted,
            protection_purpose("global", "", "LoginState", canonical_key),
        )
        if decoded is None:
            raise ValueError("Cannot decode target login state")
        previous = migrated_login_values.get(canonical_key)
        if previous is not None and previous != decoded:
            raise ValueError("Conflicting target login-state field")
        existing_login_keys.add(canonical_key)

    for key_text, decoded in migrated_login_values.items():
        if key_text in existing_login_keys:
            continue
        insert_scoped_setting(
            conn, "global", "", "LoginState", key_text, decoded,
            encrypt, portable_login_fields[key_text], overwrite=True,
        )

    conn.execute(
        """
        DELETE FROM settings WHERE scope_type='global' AND scope_id='' AND (
            lower(module)=lower(?) OR lower(module) LIKE lower(?) OR
            lower(module)=lower('LoginState/General') OR
            lower(module)=lower('LoginState/Settings') OR
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


def discover_robot_names(
    conn: sqlite3.Connection,
    data_dir: Path,
    include_disk_directories: bool = True,
) -> list[str]:
    names: set[str] = set()
    if include_disk_directories and data_dir.exists():
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
        schema_text = read_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompSchemaVersion")
        count_text = read_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompCount")
        group_count_text = read_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompGroupCount")
        schema_version = parse_int(schema_text, 0)
        legacy_identity = build_scoped_ini_identity(
            source_path, "WeldSeamComp0", "dummy"
        )
        legacy_module_pattern = str(legacy_identity["module"])[:-1] + "[0-9]*"
        has_legacy_slots = (
            conn.execute(
                """
                SELECT 1 FROM settings
                WHERE scope_type=? AND scope_id=? AND module GLOB ?
                LIMIT 1
                """,
                (
                    legacy_identity["scope_type"],
                    legacy_identity["scope_id"],
                    legacy_module_pattern,
                ),
            ).fetchone()
            is not None
        )

        # Existing v1 four-slot data remains authoritative until the application
        # explicitly loads, verifies and saves it as v2.  Adding only the v2 marker
        # here would make empty group-level values shadow valid legacy slots.
        if schema_version < 2 and (
            count_text is not None
            or group_count_text is not None
            or has_legacy_slots
        ):
            continue

        group_count = max(1, parse_int(group_count_text, parse_int(count_text, 1)))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompCount", str(group_count), encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompGroupCount", str(group_count), encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "ActiveSeamCompGroupIndex", "0", encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "SimplifyKeepAnchorsOnly", "0", encrypt))

        for group_index in range(group_count):
            group_section = f"WeldSeamCompGroup{group_index}"
            group_name = f"焊道补偿组{group_index + 1}"
            defaults = (
                ("Name", group_name),
                ("WeldZComp", "0.000000"),
                ("WeldGunDirComp", "0.000000"),
                ("WeldSeamDirComp", "0.000000"),
            )
            for key, value in defaults:
                inserted += int(insert_ini_value(conn, source_path, group_section, key, value, encrypt))
        inserted += int(insert_ini_value(conn, source_path, "ALLWeldSeamComp", "SeamCompSchemaVersion", "2", encrypt))
    return inserted


def ensure_runtime_defaults(
    conn: sqlite3.Connection,
    data_dir: Path,
    encrypt: bool,
    include_disk_directories: bool = True,
) -> tuple[int, dict[str, int]]:
    robot_names = discover_robot_names(
        conn,
        data_dir,
        include_disk_directories=include_disk_directories,
    )
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
    db_path = Path(os.path.abspath(db_path))
    _reject_pending_atomic_replacement(db_path)
    _reject_sqlite_sidecars(db_path, "Credential scrub provenance database")
    with _LockedWin32ReadFile(
        db_path,
        "Credential scrub provenance database",
        require_single_link=True,
    ) as locked_database:
        _reject_sqlite_sidecars(
            db_path, "Locked credential scrub provenance database"
        )
        database_content = locked_database.read_bytes()
        with closing(sqlite3.connect(":memory:")) as connection:
            if not hasattr(connection, "deserialize"):
                raise RuntimeError(
                    "This Python SQLite build cannot deserialize scrub provenance"
                )
            connection.deserialize(database_content)
            is_current = (
                current_schema_version(connection) == SCHEMA_VERSION
                and has_current_schema(connection)
            )
            rows = dict(connection.execute(
                "SELECT key, value FROM meta WHERE key IN (?, ?)",
                (SCRUB_STATE_KEY, SCRUB_MANIFEST_KEY),
            ).fetchall()) if is_current else {}
        locked_database.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(database_content).digest(),
            hashlib.sha256(locked_database.read_bytes()).digest(),
        ):
            raise ValueError(
                "Credential scrub provenance database changed during read-back"
            )
        _reject_sqlite_sidecars(
            db_path, "Completed credential scrub provenance database"
        )
    if not is_current:
        return "", None
    state = str(rows.get(SCRUB_STATE_KEY, ""))
    manifest_text = rows.get(SCRUB_MANIFEST_KEY)
    if state not in {"pending", "complete"} or not isinstance(manifest_text, str):
        return "", None
    return state, parse_legacy_credential_scrub_manifest(manifest_text)


def mark_legacy_credential_scrub_complete(db_path: Path) -> None:
    db_path = Path(os.path.abspath(db_path))
    if not db_path.parent.is_dir():
        raise ValueError("Credential scrub database parent is missing")
    parent_guard = _LockedWin32DirectoryChain(
        db_path.parent, "Credential scrub completion"
    )
    parent_guard.__enter__()
    try:
        _mark_legacy_credential_scrub_complete_anchored(
            db_path, parent_guard
        )
    finally:
        parent_guard.close()


def _prepare_legacy_credential_scrub_completion(
    source_content: bytes,
    expected_manifest: list[dict[str, object]] | None = None,
) -> bytes:
    source_snapshot = _canonical_sqlite_snapshot_bytes(source_content)
    with closing(sqlite3.connect(":memory:")) as connection:
        if not hasattr(connection, "deserialize") or not hasattr(
            connection, "serialize"
        ):
            raise RuntimeError(
                "This Python SQLite build cannot update a private locked snapshot"
            )
        connection.deserialize(source_snapshot)
        try:
            connection.execute("BEGIN IMMEDIATE")
            provenance = dict(connection.execute(
                "SELECT key, value FROM meta WHERE key IN (?, ?)",
                (SCRUB_STATE_KEY, SCRUB_MANIFEST_KEY),
            ).fetchall())
            manifest_text = provenance.get(SCRUB_MANIFEST_KEY)
            if not isinstance(manifest_text, str):
                raise ValueError(
                    "Legacy credential scrub provenance manifest is missing"
                )
            stored_manifest = parse_legacy_credential_scrub_manifest(
                manifest_text
            )
            if (
                expected_manifest is not None
                and stored_manifest != expected_manifest
            ):
                raise ValueError(
                    "Legacy credential scrub provenance manifest changed"
                )
            cursor = connection.execute(
                "UPDATE meta SET value='complete' WHERE key=? AND value='pending'",
                (SCRUB_STATE_KEY,),
            )
            if cursor.rowcount != 1:
                raise ValueError(
                    "Legacy credential scrub provenance is no longer pending"
                )
            integrity = connection.execute("PRAGMA integrity_check").fetchall()
            if integrity != [("ok",)]:
                raise ValueError(
                    f"Credential scrub completion failed integrity_check: {integrity}"
                )
            connection.commit()
            return _canonical_sqlite_snapshot_bytes(connection.serialize())
        except BaseException:
            connection.rollback()
            raise


def _mark_legacy_credential_scrub_complete_anchored(
    db_path: Path,
    parent_guard: _LockedWin32DirectoryChain,
) -> None:
    parent_guard.ensure_unchanged()
    _recover_pending_atomic_replacement(
        db_path, reserve_sqlite_sidecars=True
    )
    _reject_sqlite_sidecars(db_path, "Credential scrub completion database")
    publication_bytes: bytes | None = None
    with _LockedWin32ReadFile(
            db_path,
            "Credential scrub completion database",
            require_single_link=True,
    ) as locked_database:
        _reject_sqlite_sidecars(
            db_path, "Locked credential scrub completion database"
        )
        source_content = locked_database.read_bytes()
        publication_bytes = _prepare_legacy_credential_scrub_completion(
            source_content
        )
        locked_database.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(source_content).digest(),
            hashlib.sha256(locked_database.read_bytes()).digest(),
        ):
            raise ValueError(
                "Credential scrub database changed while completion was prepared"
            )
        if publication_bytes is None:
            raise RuntimeError(
                "Credential scrub completion produced no publication image"
            )
        source_identity = locked_database.identity
        if source_identity is None:
            raise RuntimeError("Credential scrub source identity is missing")
        locked_database.ensure_single_link()
    parent_guard.ensure_unchanged()
    _atomic_replace_bytes(
        db_path,
        publication_bytes,
        expected_existing=source_content,
        sqlite_sidecar_base=db_path,
        expected_existing_identity=source_identity,
    )
    _reject_sqlite_sidecars(db_path, "Completed credential scrub database")
    with _LockedWin32ReadFile(
        db_path,
        "Published credential scrub completion database",
        require_single_link=True,
    ) as published_database:
        if not hmac.compare_digest(
            hashlib.sha256(publication_bytes).digest(),
            hashlib.sha256(published_database.read_bytes()).digest(),
        ):
            raise ValueError(
                "Published credential scrub completion bytes do not match"
            )


def enforce_no_plaintext_configstore_residue(
    data_dir: Path,
    forced_encoding: str | None,
) -> None:
    pending_records = sorted(data_dir.rglob(
        f".*{ATOMIC_REPLACE_TRANSACTION_SUFFIX}"
    ))
    if pending_records:
        raise ValueError(
            "Pending atomic replacement records must be recovered before the "
            "plaintext-residue gate: "
            + ", ".join(str(record) for record in pending_records)
        )
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
            ".ConfigStore.db.atomic-*.tmp",
            ".ConfigStore.db.atomic-rollback-*.tmp",
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
        and _legacy_ini_contains_scrubbable_credentials(
            path, forced_encoding
        )
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
    db_path = Path(os.path.abspath(db_path))
    recover_pending_atomic_transactions(data_dir)
    _recover_pending_atomic_replacement(
        db_path, reserve_sqlite_sidecars=True
    )
    completion_source_content: bytes | None = None
    completion_source_identity: tuple[int, int] | None = None
    completion_publication_bytes: bytes | None = None
    completion_legacy_sources: tuple[_LegacySourceSnapshot, ...] | None = None
    with _LockedWin32ReadFile(
        db_path,
        "Legacy credential scrub database",
        require_single_link=True,
    ) as locked_database:
        # Hold the exact database identity before touching the legacy INI
        # sources.  A pre-existing external hard-link alias must therefore
        # fail before either side of the two-resource transition is changed.
        initial_database_content = locked_database.read_bytes()
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
        locked_database.ensure_single_link()
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
        completion_legacy_sources = _capture_legacy_source_snapshot(data_dir)
        locked_database.ensure_single_link()
        completion_source_content = locked_database.read_bytes()
        if not hmac.compare_digest(
            initial_database_content, completion_source_content
        ):
            raise ValueError(
                "Credential scrub database changed before completion publication"
            )
        completion_source_identity = locked_database.identity
        if completion_source_identity is None:
            raise RuntimeError(
                "Credential scrub completion source identity is missing"
            )
        completion_publication_bytes = (
            _prepare_legacy_credential_scrub_completion(
                completion_source_content, manifest
            )
        )
        locked_database.ensure_single_link()
    if (
        completion_source_content is None
        or completion_source_identity is None
        or completion_publication_bytes is None
        or completion_legacy_sources is None
    ):
        raise RuntimeError("Credential scrub completion publication is incomplete")
    _atomic_replace_bytes(
        db_path,
        completion_publication_bytes,
        expected_existing=completion_source_content,
        sqlite_sidecar_base=db_path,
        expected_existing_identity=completion_source_identity,
        publication_guard_factory=lambda: (
            _acquire_legacy_source_publication_guard(
                data_dir, completion_legacy_sources
            )
        ),
    )
    _reject_sqlite_sidecars(db_path, "Completed credential scrub database")
    with _LockedWin32ReadFile(
        db_path,
        "Published credential scrub completion database",
        require_single_link=True,
    ) as published_database:
        if not hmac.compare_digest(
            hashlib.sha256(completion_publication_bytes).digest(),
            hashlib.sha256(published_database.read_bytes()).digest(),
        ):
            raise ValueError(
                "Published credential scrub completion bytes do not match"
            )
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
    absolute_path = Path(os.path.abspath(path))
    try:
        with _LockedWin32ReadFile(
            absolute_path,
            "Runtime-only INI gate source",
            require_single_link=True,
        ) as locked_source:
            if absolute_path.resolve().parent.as_posix().casefold() \
                    != data_dir.resolve().as_posix().casefold():
                return False
            content = locked_source.read_bytes()
            if len(content) <= 0 or len(content) > 128 * 1024:
                return False
            locked_source.ensure_single_link()
            if not hmac.compare_digest(
                hashlib.sha256(content).digest(),
                hashlib.sha256(locked_source.read_bytes()).digest(),
            ):
                raise ValueError("Runtime-only INI changed during validation")
    except FileNotFoundError:
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
    absolute_path = Path(os.path.abspath(path))
    with _LockedWin32ReadFile(
        absolute_path,
        "Config migration digest source",
        require_single_link=True,
    ) as locked_source:
        content = locked_source.read_bytes()
        locked_source.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(content).digest(),
            hashlib.sha256(locked_source.read_bytes()).digest(),
        ):
            raise ValueError("Config migration digest source changed while read")
        return hashlib.sha256(content).digest()


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
) -> tuple[Path, Path, str, dict[str, object] | None]:
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
    if not _same_lexical_path(resolved_data, lexical_data):
        raise ValueError("--installer-staging Data directory must not resolve elsewhere")
    resolved_db = resolved_data / lexical_db.name
    if mode == "upgrade" and lexical_db.resolve(strict=True) != resolved_db:
        raise ValueError("--installer-staging target escapes the real Data directory")

    final_db = lexical_data / "ConfigStore.db"
    final_exists = os.path.lexists(final_db)
    upgrade_authority: dict[str, object] | None = None
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
        _reject_sqlite_sidecars(
            final_db, "Installer upgrade final ConfigStore"
        )
        with ExitStack() as stack:
            locked_staging = stack.enter_context(
                _LockedWin32ReadFile(
                    lexical_db,
                    "Installer upgrade staging authority",
                    allow_writes=True,
                    require_single_link=True,
                )
            )
            locked_final = stack.enter_context(
                _LockedWin32ReadFile(
                    final_db,
                    "Installer upgrade final authority",
                    require_single_link=True,
                )
            )
            staging_content = locked_staging.read_bytes()
            final_content = locked_final.read_bytes()
            if not hmac.compare_digest(staging_content, final_content):
                raise ValueError(
                    "Installer upgrade staging is not a byte-identical copy "
                    "of ConfigStore.db"
                )
            locked_staging.ensure_single_link()
            locked_final.ensure_single_link()
            if (
                not hmac.compare_digest(
                    hashlib.sha256(staging_content).digest(),
                    hashlib.sha256(locked_staging.read_bytes()).digest(),
                )
                or not hmac.compare_digest(
                    hashlib.sha256(final_content).digest(),
                    hashlib.sha256(locked_final.read_bytes()).digest(),
                )
            ):
                raise ValueError(
                    "Installer upgrade authority changed during validation"
                )
            upgrade_authority = {
                "staging_identity": locked_staging.identity,
                "staging_sha256": hashlib.sha256(staging_content).digest(),
                "final_path": final_db,
                "final_identity": locked_final.identity,
                "final_sha256": hashlib.sha256(final_content).digest(),
            }
    _reject_installer_staging_sidecars(lexical_db)
    # Return the lexical authority.  The migration's Win32 directory/file
    # guards will anchor this exact path before SQLite opens it.
    return lexical_data, lexical_db, mode, upgrade_authority


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

    # Preserve lexical authority for the exact-handle verifier.  Resolving a
    # junction here would turn its target into the new trusted path and make
    # the later no-follow checks ineffective.
    with _LockedWin32DirectoryChain(
        lexical_data, "Installer state verification"
    ):
        _absolute_regular_nonreparse_file(
            lexical_db, "Installer state verification database"
        )
    _reject_installer_staging_sidecars(lexical_db)
    return lexical_data, lexical_db


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


def _canonical_scrub_manifest_map(
    manifest: list[dict[str, object]],
) -> dict[str, tuple[str, dict[str, object]]]:
    entries: dict[str, tuple[str, dict[str, object]]] = {}
    for item in manifest:
        path_text = str(item["path"])
        _canonical_data_relative_path(
            path_text, "Legacy credential scrub manifest"
        )
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
    lexical_data = _absolute_lexical_path(data_dir)
    if os.name != "nt" or not lexical_data.is_dir():
        raise ValueError(
            "Installer state verification requires an existing Windows Data directory"
        )
    data_guard = _LockedWin32DirectoryChain(
        lexical_data, "Installer state verification authority"
    )
    data_guard.__enter__()
    try:
        result = _verify_installer_state_anchored(data_dir, db_path)
        data_guard.ensure_unchanged()
        return result
    finally:
        data_guard.close()


def _verify_installer_state_anchored(
    data_dir: Path,
    db_path: Path,
) -> tuple[str, Path]:
    resolved_data, resolved_db = _validate_installer_verification_paths(
        data_dir, db_path
    )
    reject_pending_atomic_transactions(resolved_data)
    _reject_pending_atomic_replacement(resolved_db)
    with _LockedWin32ReadFile(
        resolved_db,
        "Installer state selected database",
        require_single_link=True,
    ) as selected_database:
        selected_content = selected_database.read_bytes()
        before_inventory = _read_only_data_inventory(resolved_data)
        current_state = verify_current_database(resolved_db)
        fresh_manifest = prepare_legacy_ini_credential_scrub(
            resolved_data, None
        )
        provenance_state = _validate_installer_scrub_provenance(
            resolved_data, resolved_db, None, fresh_manifest
        )
        if provenance_state != current_state:
            raise ValueError(
                "Installer database verification and scrub provenance states differ"
            )
        _reject_installer_staging_sidecars(resolved_db)
        after_inventory = _read_only_data_inventory(resolved_data)
        selected_database.ensure_single_link()
        if (
            before_inventory != after_inventory
            or not hmac.compare_digest(
                hashlib.sha256(selected_content).digest(),
                hashlib.sha256(selected_database.read_bytes()).digest(),
            )
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


def _has_known_auth3_login_preference_type_drift(
    connection: sqlite3.Connection,
) -> bool:
    """Recognize only the auth3 writer bug that stored both bools as strings."""
    rows = connection.execute(
        """
        SELECT key_name, value_text, value_type, sensitive, encrypted
        FROM settings
        WHERE scope_type='global' AND scope_id='' AND module='LoginState'
          AND key_name IN ('RememberPassword', 'AutoLogin')
        ORDER BY key_name
        """
    ).fetchall()
    if not rows:
        return False

    shapes: set[str] = set()
    for key, stored, value_type, sensitive, encrypted in rows:
        stored_text_value = str(stored)
        common_shape = (
            sensitive in (1, "1", True)
            and encrypted in (1, "1", True)
            and stored_text_value.startswith(DPAPI_PREFIX)
        )
        if not common_shape:
            raise ValueError(
                "Current auth3 login preference drift is not safely recoverable"
            )
        shapes.add(str(value_type))

    if shapes == {"bool"}:
        return False
    if (
        shapes == {"string"}
        and len(rows) == 2
        and {str(row[0]) for row in rows} == {
            "RememberPassword", "AutoLogin"
        }
    ):
        for key, stored, _value_type, _sensitive, encrypted in rows:
            decoded = decode_stored_text(
                str(stored),
                encrypted,
                protection_purpose(
                    "global", "", "LoginState", str(key)
                ),
            )
            if decoded not in {"0", "1"}:
                raise ValueError(
                    "Current auth3 login preference drift cannot be read safely"
                )
        return True
    if shapes == {"string"}:
        raise ValueError(
            "Current auth3 string login preference drift is incomplete"
        )
    raise ValueError(
        "Current auth3 login preferences have a mixed canonical/drift shape"
    )


def _valid_dpapi_envelope_shape(stored: str) -> bool:
    if not stored.startswith(DPAPI_PREFIX):
        return False
    try:
        payload = base64.b64decode(
            padded_base64(stored[len(DPAPI_PREFIX):]),
            altchars=b"-_",
            validate=True,
        )
    except (ValueError, UnicodeEncodeError, binascii.Error):
        return False
    return 64 <= len(payload) <= 16 * 1024 * 1024


def _foreign_user_dpapi_reset_category(
    scope_type: str,
    scope_id: str,
    module: str,
    key: str,
    value_type: str,
    sensitive: object,
    encrypted: object,
) -> str | None:
    """Classify only DPAPI state that is safe to invalidate after a user move."""
    if sensitive not in (1, "1", True) or encrypted not in (1, "1", True):
        return None
    if (
        scope_type == "global"
        and scope_id == ""
        and module == "LoginState"
        and key in {"RememberPassword", "AutoLogin"}
        and value_type == "bool"
    ):
        return "login_preference"
    if (
        scope_type == "global"
        and scope_id == ""
        and module in {
            "LoginState/SavedPasswords",
            "LoginState/RememberedCredentials",
        }
        and value_type == "string"
        and _is_safe_account_name(key)
    ):
        return "remembered_credential"
    if (
        scope_type == "global"
        and scope_id == ""
        and module == "OnlineServices"
        and key in {"AdminApiToken", "FtpPassword"}
        and value_type == "string"
    ):
        return "online_service_credential"
    if (
        scope_type in {"robot", "workpiece_template"}
        and 0 < len(scope_id) <= 256
        and scope_id == scope_id.strip()
        and not any(character in scope_id for character in "\x00\r\n")
        and module == "RobotPara/BaseParam"
        and key == "FTPPassWord"
        and value_type == "string"
    ):
        return "robot_ftp_credential"
    if (
        scope_type == "global"
        and scope_id == ""
        and module == "PointCloudProofSecurity"
        and key == "HmacKeyV1"
        and value_type == "secret"
    ):
        return "pointcloud_proof_key"
    if (
        scope_type == "pointcloud-proof-receipt"
        and re.fullmatch(
            r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
            r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}",
            scope_id,
        ) is not None
        and module == "QualityGateReceiptV1"
        and key == "Receipt"
        and value_type == "json"
    ):
        return "pointcloud_proof_receipt"
    return None


def _foreign_user_dpapi_reset_plan(
    connection: sqlite3.Connection,
) -> list[tuple[str, str, str, str, str]] | None:
    """Recognize a complete CurrentUser-DPAPI binding move, never partial damage."""
    protected_rows = connection.execute(
        """
        SELECT scope_type, scope_id, module, key_name, value_text,
               value_type, sensitive, encrypted
        FROM settings
        WHERE value_text LIKE ?
        ORDER BY scope_type, scope_id, module, key_name
        """,
        (DPAPI_PREFIX + "%",),
    ).fetchall()
    if not protected_rows:
        return None

    readable_count = 0
    unreadable_rows: list[tuple[object, ...]] = []
    for row in protected_rows:
        scope_type, scope_id, module, key, stored, _value_type, _sensitive, encrypted = row
        decoded = decode_stored_text(
            str(stored),
            encrypted,
            protection_purpose(
                str(scope_type), str(scope_id), str(module), str(key)
            ),
        )
        if decoded is None:
            unreadable_rows.append(row)
        else:
            readable_count += 1

    if not unreadable_rows:
        return None
    if readable_count != 0 or len(unreadable_rows) != len(protected_rows):
        raise ValueError(
            "Current ConfigStore has mixed readable and unreadable DPAPI state"
        )

    plan: list[tuple[str, str, str, str, str]] = []
    for (
        scope_type, scope_id, module, key, stored,
        value_type, sensitive, encrypted,
    ) in unreadable_rows:
        scope_text = str(scope_type)
        scope_id_text = str(scope_id)
        module_text = str(module)
        key_text = str(key)
        if not _valid_dpapi_envelope_shape(str(stored)):
            raise ValueError(
                "Current ConfigStore contains a malformed unreadable DPAPI envelope"
            )
        category = _foreign_user_dpapi_reset_category(
            scope_text,
            scope_id_text,
            module_text,
            key_text,
            str(value_type),
            sensitive,
            encrypted,
        )
        if category is None:
            raise ValueError(
                "Current ConfigStore has unsupported unreadable DPAPI state"
            )
        plan.append(
            (scope_text, scope_id_text, module_text, key_text, category)
        )

    preference_keys = {
        key
        for scope_type, scope_id, module, key, category in plan
        if category == "login_preference"
        and scope_type == "global"
        and scope_id == ""
        and module == "LoginState"
    }
    if preference_keys != {"RememberPassword", "AutoLogin"}:
        raise ValueError(
            "Foreign-user DPAPI recovery requires both canonical login preferences"
        )
    return plan


def verify_current_database(
    db_path: Path,
    *,
    allow_known_auth3_login_preference_drift: bool = False,
    allow_foreign_user_dpapi_reset: bool = False,
) -> str:
    if (
        allow_known_auth3_login_preference_drift
        and allow_foreign_user_dpapi_reset
    ):
        raise ValueError("Current ConfigStore verification recovery modes conflict")
    db_path = Path(os.path.abspath(db_path))
    _reject_pending_atomic_replacement(db_path)
    _reject_sqlite_sidecars(db_path, "Current ConfigStore verification")
    stack = ExitStack()
    try:
        locked_database = stack.enter_context(
            _LockedWin32ReadFile(
                db_path,
                "Current ConfigStore verification",
                require_single_link=True,
            )
        )
        _reject_sqlite_sidecars(
            db_path, "Locked current ConfigStore verification"
        )
        database_content = locked_database.read_bytes()
        connection = stack.enter_context(closing(sqlite3.connect(":memory:")))
        if not hasattr(connection, "deserialize"):
            raise RuntimeError(
                "This Python SQLite build cannot deserialize an in-memory "
                "verification database"
            )
        connection.deserialize(database_content)
        connection.execute("PRAGMA query_only=ON")
        integrity = connection.execute("PRAGMA integrity_check").fetchone()
        if integrity != ("ok",):
            raise ValueError(f"ConfigStore integrity_check failed: {integrity}")
        if current_schema_version(connection) != SCHEMA_VERSION:
            raise ValueError("ConfigStore is not schema v5")
        if not has_current_schema(connection):
            raise ValueError("ConfigStore meta/settings schema is incompatible")
        if (
            allow_known_auth3_login_preference_drift
            and not _has_known_auth3_login_preference_type_drift(connection)
        ):
            raise ValueError(
                "Current ConfigStore does not contain the exact recoverable "
                "auth3 login preference drift"
            )
        foreign_dpapi_plan = (
            _foreign_user_dpapi_reset_plan(connection)
            if allow_foreign_user_dpapi_reset
            else None
        )
        if allow_foreign_user_dpapi_reset and not foreign_dpapi_plan:
            raise ValueError(
                "Current ConfigStore does not contain an exact recoverable "
                "foreign-user DPAPI binding"
            )
        foreign_dpapi_identities = {
            entry[:4] for entry in (foreign_dpapi_plan or [])
        }

        duplicate_setting = connection.execute(
            """
            SELECT scope_type, scope_id, module, key_name
            FROM settings
            GROUP BY scope_type, scope_id, module, key_name
            HAVING COUNT(*)<>1
            LIMIT 1
            """
        ).fetchone()
        if duplicate_setting is not None:
            raise ValueError("ConfigStore settings contains duplicate logical keys")

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
                lower(module)=lower('LoginState/Settings') OR
                (lower(module)=lower('LoginState') AND module<>'LoginState') OR
                (module='LoginState' AND lower(key_name) IN (
                    lower('UserName'), lower('AccountHistory'),
                    lower('RememberPassword'), lower('AutoLogin'),
                    lower('PasswordBase64')
                ) AND (
                    key_name NOT IN (
                        'UserName', 'AccountHistory', 'RememberPassword',
                        'AutoLogin', 'PasswordBase64'
                    ) OR scope_type<>'global' OR scope_id<>''
                )) OR
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

        for key, stored, value_type, sensitive, encrypted in connection.execute(
            """
            SELECT key_name, value_text, value_type, sensitive, encrypted
            FROM settings
            WHERE scope_type='global' AND scope_id='' AND module='LoginState'
              AND key_name IN ('RememberPassword', 'AutoLogin')
            """
        ).fetchall():
            key_text = str(key)
            stored_text_value = str(stored)
            value_type_text = str(value_type)
            canonical_shape = value_type_text == "bool"
            known_drift_shape = (
                allow_known_auth3_login_preference_drift
                and value_type_text == "string"
            )
            if (
                not (canonical_shape or known_drift_shape)
                or sensitive not in (1, "1", True)
                or encrypted not in (1, "1", True)
                or not stored_text_value.startswith(DPAPI_PREFIX)
            ):
                raise ValueError(
                    "Current login preference has a non-canonical protected shape"
                )
            decoded_preference = decode_stored_text(
                stored_text_value,
                encrypted,
                protection_purpose(
                    "global", "", "LoginState", key_text
                ),
            )
            identity = ("global", "", "LoginState", key_text)
            if (
                decoded_preference not in {"0", "1"}
                and identity not in foreign_dpapi_identities
            ):
                raise ValueError("Current login preference is not a canonical boolean")

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
            identity = (scope_text, scope_id_text, module_text, key_text)
            if (
                (decoded_credential is None or decoded_credential == "")
                and identity not in foreign_dpapi_identities
            ):
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
            ):
                raise ValueError("A recoverable sensitive setting is not protected by DPAPI")
            decoded_value = None
            if stored_value.startswith(DPAPI_PREFIX) or encrypted not in (
                None, 0, "0", False
            ):
                decoded_value = decode_stored_text(
                    stored_value, encrypted, purpose
                )
            if (
                stored_value.startswith(DPAPI_PREFIX)
                or encrypted not in (None, 0, "0", False)
            ) and decoded_value is None and (
                scope_text, scope_id_text, module_text, key_text
            ) not in foreign_dpapi_identities:
                raise ValueError("A protected ConfigStore setting cannot be read back")
        locked_database.ensure_single_link()
        if not hmac.compare_digest(
            hashlib.sha256(database_content).digest(),
            hashlib.sha256(locked_database.read_bytes()).digest(),
        ):
            raise ValueError(
                "Current ConfigStore verification observed an input file change"
            )
        _reject_sqlite_sidecars(
            db_path, "Completed current ConfigStore verification"
        )
    finally:
        stack.close()
    return scrub_state or "none"


def migrate_existing_database_to_current(
    db_path: Path,
    data_dir: Path,
    encrypt_new_values: bool,
    scrub_manifest: list[dict[str, object]] | None = None,
    forced_encoding: str | None = None,
    upgrade_backup_path: Path | None = None,
    installer_upgrade_authority: dict[str, object] | None = None,
    directory_authority: _LockedWin32DirectoryChain | None = None,
) -> bool:
    if not db_path.exists():
        return False
    disk_source_snapshot = _capture_legacy_source_snapshot(data_dir)
    _reject_sqlite_sidecars(db_path, "Existing database upgrade source")
    backup: Path | None = None
    publication_bytes: bytes | None = None
    with ExitStack() as stack:
        locked_database = stack.enter_context(
            _LockedWin32ReadFile(
                db_path,
                "Existing database upgrade source",
                require_single_link=True,
            )
        )
        _reject_sqlite_sidecars(db_path, "Locked database upgrade source")
        source_content = locked_database.read_bytes()
        if installer_upgrade_authority is not None:
            expected_staging_identity = installer_upgrade_authority.get(
                "staging_identity"
            )
            expected_staging_sha256 = installer_upgrade_authority.get(
                "staging_sha256"
            )
            reference_path = installer_upgrade_authority.get("final_path")
            expected_final_identity = installer_upgrade_authority.get(
                "final_identity"
            )
            expected_final_sha256 = installer_upgrade_authority.get(
                "final_sha256"
            )
            if (
                locked_database.identity != expected_staging_identity
                or not isinstance(expected_staging_sha256, bytes)
                or not hmac.compare_digest(
                    hashlib.sha256(source_content).digest(),
                    expected_staging_sha256,
                )
                or not isinstance(reference_path, Path)
                or not isinstance(expected_final_sha256, bytes)
            ):
                raise ValueError(
                    "Installer upgrade staging authority changed before lock"
                )
            locked_reference = stack.enter_context(
                _LockedWin32ReadFile(
                    reference_path,
                    "Installer upgrade final authority recheck",
                    require_single_link=True,
                )
            )
            reference_content = locked_reference.read_bytes()
            if (
                locked_reference.identity != expected_final_identity
                or not hmac.compare_digest(
                    hashlib.sha256(reference_content).digest(),
                    expected_final_sha256,
                )
                or not hmac.compare_digest(source_content, reference_content)
            ):
                raise ValueError(
                    "Installer upgrade final authority changed before lock"
                )
            locked_database.ensure_single_link()
            locked_reference.ensure_single_link()
        source_snapshot = _canonical_sqlite_snapshot_bytes(
            source_content
        )
        conn = stack.enter_context(closing(sqlite3.connect(":memory:")))
        if not hasattr(conn, "deserialize") or not hasattr(conn, "serialize"):
            raise RuntimeError(
                "This Python SQLite build cannot migrate a private locked snapshot"
            )
        conn.deserialize(source_snapshot)
        try:
            # Version selection, backup proof and migration run on one private
            # snapshot connection.  The source file stays byte-identical under
            # a no-write/no-delete identity lock until a complete replacement
            # image has passed every gate.
            conn.execute("BEGIN IMMEDIATE")

            has_legacy_tables = has_any_legacy_config_table(conn)
            has_settings_schema = has_current_schema(conn)
            has_meta_table = table_exists(conn, "meta")
            version = current_schema_version(conn)
            if has_meta_table and not version:
                raise SystemExit(
                    "ConfigStore meta table has no schema_version; refusing to guess"
                )
            if version not in {"", "4", SCHEMA_VERSION}:
                raise SystemExit(
                    f"Unsupported future or unknown ConfigStore schema: {version}"
                )
            if version == SCHEMA_VERSION and not has_settings_schema:
                raise SystemExit(
                    "Current ConfigStore schema metadata has incompatible settings columns"
                )
            if version == "4" and not (has_settings_schema or has_legacy_tables):
                raise SystemExit(
                    "ConfigStore schema v4 has neither current nor legacy configuration tables"
                )
            authentication_initialized = True
            authentication_semantic_version = ""
            if has_meta_table:
                semantic_row = conn.execute(
                    "SELECT value FROM meta WHERE key='auth_semantic_version' LIMIT 1"
                ).fetchone()
                if semantic_row and semantic_row[0] is not None:
                    authentication_semantic_version = str(semantic_row[0])
            if version == SCHEMA_VERSION:
                if authentication_semantic_version not in {
                    "1", "2", AUTH_SEMANTIC_VERSION
                }:
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
            known_auth3_login_preference_drift = (
                version == SCHEMA_VERSION
                and authentication_semantic_version == AUTH_SEMANTIC_VERSION
                and _has_known_auth3_login_preference_type_drift(conn)
            )
            foreign_user_dpapi_reset_plan = (
                _foreign_user_dpapi_reset_plan(conn)
                if version == SCHEMA_VERSION
                and authentication_semantic_version == AUTH_SEMANTIC_VERSION
                and not known_auth3_login_preference_drift
                else None
            )
            if known_auth3_login_preference_drift:
                # Prove that this exact locked source is otherwise a fully valid
                # current database before allowing the narrow writer-bug repair.
                verify_current_database(
                    db_path,
                    allow_known_auth3_login_preference_drift=True,
                )
                locked_database.ensure_single_link()
                if not hmac.compare_digest(
                    hashlib.sha256(source_content).digest(),
                    hashlib.sha256(locked_database.read_bytes()).digest(),
                ):
                    raise ValueError(
                        "Known auth3 login preference source changed during validation"
                    )
            if foreign_user_dpapi_reset_plan:
                # A raw database copied to another Windows user keeps portable
                # account hashes and business configuration, but its CurrentUser
                # DPAPI values cannot be recovered.  Prove that every unreadable
                # value belongs to the narrow disposable allowlist before reset.
                verify_current_database(
                    db_path,
                    allow_foreign_user_dpapi_reset=True,
                )
                locked_database.ensure_single_link()
                if not hmac.compare_digest(
                    hashlib.sha256(source_content).digest(),
                    hashlib.sha256(locked_database.read_bytes()).digest(),
                ):
                    raise ValueError(
                        "Foreign-user DPAPI source changed during validation"
                    )
            needs_migration = (
                has_legacy_tables
                or has_outdated_current_schema
                or needs_authentication_semantic_upgrade
                or known_auth3_login_preference_drift
                or bool(foreign_user_dpapi_reset_plan)
            )
            if not needs_migration:
                conn.rollback()
                return False
            initialize_empty_scrub_provenance = False
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
                    provenance_is_absent = (
                        SCRUB_STATE_KEY not in provenance_rows
                        and SCRUB_MANIFEST_KEY not in provenance_rows
                    )
                    exact_current_repair = (
                        known_auth3_login_preference_drift
                        or bool(foreign_user_dpapi_reset_plan)
                    )
                    if not (
                        provenance_is_absent
                        and scrub_manifest == []
                        and exact_current_repair
                    ):
                        raise SystemExit(
                            "Current-v5 repair has no valid legacy scrub provenance; "
                            "automatic recovery requires an empty legacy credential scrub plan"
                        )
                    # Early schema-v5 databases predate scrub provenance.  An
                    # exact current-schema repair may adopt an empty manifest,
                    # but never auto-delete newly discovered legacy credentials.
                    enforce_no_plaintext_configstore_residue(
                        data_dir, forced_encoding
                    )
                    initialize_empty_scrub_provenance = True
                else:
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
            # ConfigStore is the sole configuration authority once an existing
            # database is present.  Legacy INI/TXT files may remain beside it
            # for compatibility with old releases, but their values must never
            # overwrite or veto a schema upgrade.  The immutable snapshot is
            # still retained as a transaction guard, and credential-bearing INI
            # fields are scrubbed only through the separately verified manifest.

            backup = (
                _validated_upgrade_backup_path(db_path, upgrade_backup_path)
                if upgrade_backup_path is not None
                else _default_upgrade_backup_path(db_path)
            )
            if directory_authority is not None:
                directory_authority.ensure_unchanged()
            locked_database.ensure_single_link()
            _create_dpapi_database_backup_from_bytes(source_snapshot, backup)
            _reject_sqlite_sidecars(
                db_path, "Protected database upgrade source"
            )
            locked_database.ensure_single_link()

            conn.execute("PRAGMA secure_delete=ON")
            create_current_tables(conn)
            if known_auth3_login_preference_drift:
                for preference_key in ("RememberPassword", "AutoLogin"):
                    insert_scoped_setting(
                        conn,
                        "global",
                        "",
                        "LoginState",
                        preference_key,
                        "0",
                        encrypt_new_values,
                        "bool",
                        overwrite=True,
                    )
                legacy_counts = (0, 0, 0)
                default_count = 0
                default_details = {
                    "auth3_login_preference_type_repair": 2,
                }
            elif foreign_user_dpapi_reset_plan:
                category_counts: dict[str, int] = {}
                for scope_type, scope_id, module, key, category in (
                    foreign_user_dpapi_reset_plan
                ):
                    deleted = conn.execute(
                        """
                        DELETE FROM settings
                        WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?
                        """,
                        (scope_type, scope_id, module, key),
                    )
                    if deleted.rowcount != 1:
                        raise ValueError(
                            "Foreign-user DPAPI recovery source changed in staging"
                        )
                    category_counts[category] = category_counts.get(category, 0) + 1
                legacy_counts = (0, 0, 0)
                default_count = 0
                default_details = {
                    "foreign_user_dpapi_rows_removed": len(
                        foreign_user_dpapi_reset_plan
                    ),
                    "login_preferences_removed": category_counts.get(
                        "login_preference", 0
                    ),
                    "remembered_credentials_removed": category_counts.get(
                        "remembered_credential", 0
                    ),
                    "service_credentials_removed": category_counts.get(
                        "online_service_credential", 0
                    ) + category_counts.get("robot_ftp_credential", 0),
                    "pointcloud_proof_rows_removed": category_counts.get(
                        "pointcloud_proof_key", 0
                    ) + category_counts.get("pointcloud_proof_receipt", 0),
                }
            else:
                legacy_counts = migrate_legacy_tables_to_settings(
                    conn, encrypt_new_values
                )
                account_count, administrator_count = migrate_authentication_semantics(
                    conn,
                    encrypt_new_values,
                    allow_legacy_v4_wrapped_profiles=(version == "4"),
                )
                if (
                    (authentication_initialized and account_count == 0)
                    or (not authentication_initialized and account_count != 0)
                    or (account_count > 0 and administrator_count == 0)
                ):
                    raise ValueError(
                        "Migrated authentication initialization/account/admin state is inconsistent"
                    )
                default_count, default_details = ensure_runtime_defaults(
                    conn,
                    data_dir,
                    encrypt_new_values,
                    include_disk_directories=False,
                )
                drop_legacy_config_tables(conn)
                set_schema_meta(
                    conn,
                    encrypt_new_values,
                    authentication_initialized=authentication_initialized,
                )
                if version != SCHEMA_VERSION:
                    set_legacy_credential_scrub_pending(conn, scrub_manifest)
            if initialize_empty_scrub_provenance:
                set_legacy_credential_scrub_pending(conn, scrub_manifest)
            integrity = conn.execute("PRAGMA integrity_check").fetchall()
            if integrity != [("ok",)]:
                raise ValueError(
                    f"Migrated database failed integrity_check: {integrity}"
                )
            locked_database.ensure_single_link()
            conn.commit()
            publication_bytes = _canonical_sqlite_snapshot_bytes(conn.serialize())
            locked_database.ensure_single_link()
            if not hmac.compare_digest(
                hashlib.sha256(source_content).digest(),
                hashlib.sha256(locked_database.read_bytes()).digest(),
            ):
                raise ValueError(
                    "Existing database changed while its private upgrade was prepared"
                )
            if backup is None:
                raise RuntimeError(
                    "Database upgrade committed without a protected backup"
                )
            if publication_bytes is None:
                raise RuntimeError(
                    "Database upgrade produced no verified publication image"
                )
            source_identity = locked_database.identity
            if source_identity is None:
                raise RuntimeError("Database upgrade source identity is missing")
            locked_database.ensure_single_link()
        except BaseException:
            conn.rollback()
            raise

    with ExitStack() as publication_locks:
        if installer_upgrade_authority is not None:
            reference_path = installer_upgrade_authority.get("final_path")
            expected_final_identity = installer_upgrade_authority.get(
                "final_identity"
            )
            expected_final_sha256 = installer_upgrade_authority.get(
                "final_sha256"
            )
            if (
                not isinstance(reference_path, Path)
                or not isinstance(expected_final_sha256, bytes)
            ):
                raise ValueError("Installer upgrade final authority is invalid")
            final_publication_lock = publication_locks.enter_context(
                _LockedWin32ReadFile(
                    reference_path,
                    "Installer upgrade final publication authority",
                    require_single_link=True,
                )
            )
            final_publication_content = final_publication_lock.read_bytes()
            if (
                final_publication_lock.identity != expected_final_identity
                or not hmac.compare_digest(
                    hashlib.sha256(final_publication_content).digest(),
                    expected_final_sha256,
                )
            ):
                raise ValueError(
                    "Installer upgrade final authority changed before publication"
                )
        if directory_authority is not None:
            directory_authority.ensure_unchanged()
        _atomic_replace_bytes(
            db_path,
            publication_bytes,
            expected_existing=source_content,
            sqlite_sidecar_base=db_path,
            expected_existing_identity=source_identity,
            publication_guard_factory=lambda: (
                _acquire_legacy_source_publication_guard(
                    data_dir, disk_source_snapshot
                )
            ),
        )

    _reject_sqlite_sidecars(db_path, "Committed database upgrade")
    with _LockedWin32ReadFile(
        db_path, "Published database upgrade", require_single_link=True
    ) as published_database:
        if not hmac.compare_digest(
            hashlib.sha256(publication_bytes).digest(),
            hashlib.sha256(published_database.read_bytes()).digest(),
        ):
            raise ValueError("Published database upgrade bytes do not match")

    print(f"Upgraded existing database to schema v{SCHEMA_VERSION}: {db_path}")
    print(f"Created and verified DPAPI-protected database backup: {backup}")
    print(f"Legacy rows migrated: INI={legacy_counts[0]}, text={legacy_counts[1]}, app_settings={legacy_counts[2]}")
    if disk_source_snapshot:
        print(
            "Ignored legacy disk configuration values (ConfigStore is authoritative): "
            f"files={len(disk_source_snapshot)}"
        )
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
    lexical_data = _absolute_lexical_path(data_dir)
    lexical_db = _absolute_lexical_path(db_path)
    if os.name != "nt":
        raise OSError("ConfigStore migration requires Windows identity guards")
    if not lexical_data.is_dir():
        raise SystemExit(f"Data directory not found: {lexical_data}")
    if not lexical_db.parent.is_dir():
        raise ValueError(
            "ConfigStore target parent must already exist as a directory"
        )
    guards: list[_LockedWin32DirectoryChain] = []
    try:
        for directory, label in (
            (lexical_data, "ConfigStore source Data authority"),
            (lexical_db.parent, "ConfigStore destination authority"),
        ):
            guard = _LockedWin32DirectoryChain(directory, label)
            guard.__enter__()
            guards.append(guard)
        _migrate_anchored(
            lexical_data,
            lexical_db,
            overwrite,
            encrypt,
            forced_encoding,
            allow_mojibake,
            scrub_legacy_credentials,
            upgrade_backup_path,
            defer_credential_scrub,
            installer_staging,
            tuple(guards),
        )
    finally:
        for guard in reversed(guards):
            guard.close()


def _migrate_anchored(
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
    authority_guards: tuple[_LockedWin32DirectoryChain, ...] = (),
) -> None:
    def ensure_authority() -> None:
        for authority_guard in authority_guards:
            authority_guard.ensure_unchanged()

    installer_staging_mode = ""
    installer_upgrade_authority: dict[str, object] | None = None
    if installer_staging:
        if not defer_credential_scrub:
            raise ValueError(
                "--installer-staging requires --defer-credential-scrub"
            )
        if overwrite:
            raise ValueError("--installer-staging cannot be combined with --overwrite")
        (
            data_dir,
            db_path,
            installer_staging_mode,
            installer_upgrade_authority,
        ) = (
            _validate_installer_staging_target(data_dir, db_path)
        )
    else:
        data_dir = _absolute_lexical_path(data_dir)
        db_path = _absolute_lexical_path(db_path)
    if os.name != "nt":
        raise OSError("ConfigStore migration requires Windows identity guards")
    if not data_dir.is_dir():
        raise SystemExit(f"Data directory not found: {data_dir}")
    if not db_path.parent.is_dir():
        raise ValueError(
            "ConfigStore target parent must already exist as a directory"
        )
    # Reject redirected lexical roots before recovery, temporary-file
    # creation, or any other mutation.  Creation is intentionally limited to
    # an already-existing, identity-anchored directory.
    with _LockedWin32DirectoryChain(data_dir, "ConfigStore source Data"):
        pass
    with _LockedWin32DirectoryChain(
        db_path.parent, "ConfigStore migration destination"
    ):
        pass
    ensure_authority()
    recover_pending_atomic_transactions(data_dir)
    _recover_pending_atomic_replacement(
        db_path,
        reserve_sqlite_sidecars=True,
    )
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
    builds_from_legacy_sources = overwrite or not db_path.exists()
    legacy_source_snapshot = (
        _capture_legacy_source_snapshot(data_dir)
        if builds_from_legacy_sources
        else None
    )
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

    _reject_sqlite_sidecars(db_path, "ConfigStore migration destination")
    overwrite_expected_content: bytes | None = None
    overwrite_stack: ExitStack | None = None
    overwrite_locked_database: _LockedWin32ReadFile | None = None
    overwrite_source_identity: tuple[int, int] | None = None
    if db_path.exists():
        if not overwrite:
            if migrate_existing_database_to_current(
                db_path,
                data_dir,
                encrypt,
                scrub_manifest,
                forced_encoding,
                upgrade_backup_path,
                installer_upgrade_authority,
                authority_guards[-1] if authority_guards else None,
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
        overwrite_stack = ExitStack()
        try:
            overwrite_locked_database = overwrite_stack.enter_context(
                _LockedWin32ReadFile(
                    db_path,
                    "ConfigStore overwrite source",
                    require_single_link=True,
                )
            )
            _reject_sqlite_sidecars(db_path, "Locked ConfigStore overwrite source")
            overwrite_expected_content = overwrite_locked_database.read_bytes()
            overwrite_source_identity = overwrite_locked_database.identity
            if overwrite_source_identity is None:
                raise RuntimeError("ConfigStore overwrite source identity is missing")
            overwrite_snapshot = _canonical_sqlite_snapshot_bytes(
                overwrite_expected_content
            )
            ensure_authority()
            overwrite_locked_database.ensure_single_link()
            _create_dpapi_database_backup_from_bytes(overwrite_snapshot, backup)
            _reject_sqlite_sidecars(
                db_path, "Protected ConfigStore overwrite source"
            )
            overwrite_locked_database.ensure_single_link()
            if not hmac.compare_digest(
                overwrite_expected_content,
                overwrite_locked_database.read_bytes(),
            ):
                raise ValueError(
                    "Overwrite destination changed while its backup was created"
                )
        except BaseException:
            overwrite_stack.close()
            overwrite_stack = None
            overwrite_locked_database = None
            overwrite_source_identity = None
            raise
        print(f"Created and verified DPAPI-protected database backup: {backup}")

    ini_count = 0
    text_count = 0
    value_count = 0
    default_replacements: list[tuple[str, str, str, str, str]] = []
    publication_bytes: bytes | None = None
    target_directory_guard = _LockedWin32DirectoryChain(
        db_path.parent, "ConfigStore staging creation"
    )
    target_directory_guard.__enter__()

    try:
        if legacy_source_snapshot is None:
            raise RuntimeError("Legacy source snapshot was not captured")
        ini_source_contents = {
            item.relative_path.casefold(): item.content
            for item in legacy_source_snapshot
            if item.kind == "ini"
        }
        with closing(sqlite3.connect(":memory:")) as conn:
            if not hasattr(conn, "serialize"):
                raise RuntimeError(
                    "This Python SQLite build cannot serialize a private migration"
                )
            try:
                conn.execute("BEGIN IMMEDIATE")
                init_schema(conn, encrypt)

                for source in legacy_source_snapshot:
                    if source.kind != "ini":
                        continue
                    source_path = source.absolute_path
                    source_content = source.content
                    file_path = source.database_path
                    decoded_text, encoding = decode_bytes(
                        source_content, forced_encoding
                    )
                    rows = parse_ini(decoded_text)
                    rows, replacements = replace_mojibake_ini_rows(
                        data_dir,
                        source_path,
                        rows,
                        encoding,
                        allow_mojibake,
                        ini_source_contents,
                    )
                    default_replacements.extend(
                        (file_path, section, key, old_value, new_value)
                        for section, key, old_value, new_value in replacements
                    )
                    check_ini_rows(
                        source_path, rows, encoding, allow_mojibake
                    )
                    for section, key, value in rows:
                        value_count += int(insert_ini_value(
                            conn,
                            file_path,
                            section,
                            key,
                            value,
                            encrypt,
                            overwrite=True,
                        ))
                    ini_count += 1

                for source in legacy_source_snapshot:
                    if source.kind != "text":
                        continue
                    source_path = source.absolute_path
                    source_content = source.content
                    file_path = source.database_path
                    content = normalize_text(sanitize_legacy_text_bytes(
                        source_path,
                        source_content,
                        forced_encoding,
                        allow_mojibake,
                    ))
                    text_count += int(insert_text_file(
                        conn, file_path, content, encrypt, overwrite=True
                    ))

                account_count, administrator_count = (
                    migrate_authentication_semantics(conn, encrypt)
                )
                if account_count > 0 and administrator_count == 0:
                    raise ValueError(
                        "Migrated account/Profile data has no valid administrator"
                    )
                authentication_initialized = account_count > 0
                default_count, default_details = ensure_runtime_defaults(
                    conn, data_dir, encrypt
                )
                drop_legacy_config_tables(conn)
                set_schema_meta(
                    conn,
                    encrypt,
                    authentication_initialized=authentication_initialized,
                )
                set_legacy_credential_scrub_pending(conn, scrub_manifest)
                conn.commit()
                publication_bytes = _canonical_sqlite_snapshot_bytes(
                    conn.serialize()
                )
            except BaseException:
                conn.rollback()
                raise

        if publication_bytes is None:
            raise RuntimeError("ConfigStore migration produced no publication image")
        if overwrite_expected_content is None:
            _assert_legacy_source_snapshot_unchanged(
                data_dir, legacy_source_snapshot
            )
            ensure_authority()
            _atomic_replace_bytes(
                db_path,
                publication_bytes,
                require_absent=True,
                sqlite_sidecar_base=db_path,
                publication_guard_factory=lambda: (
                    _acquire_legacy_source_publication_guard(
                        data_dir, legacy_source_snapshot
                    )
                ),
            )
        else:
            if (
                overwrite_locked_database is None
                or overwrite_source_identity is None
                or overwrite_stack is None
            ):
                raise RuntimeError(
                    "ConfigStore overwrite source lock was lost before publication"
                )
            overwrite_locked_database.ensure_single_link()
            if not hmac.compare_digest(
                overwrite_expected_content,
                overwrite_locked_database.read_bytes(),
            ):
                raise ValueError(
                    "Overwrite destination changed before atomic publication"
                )
            _reject_sqlite_sidecars(
                db_path, "ConfigStore overwrite source before publication"
            )
            overwrite_stack.close()
            overwrite_stack = None
            overwrite_locked_database = None
            _assert_legacy_source_snapshot_unchanged(
                data_dir, legacy_source_snapshot
            )
            ensure_authority()
            _atomic_replace_bytes(
                db_path,
                publication_bytes,
                expected_existing=overwrite_expected_content,
                sqlite_sidecar_base=db_path,
                expected_existing_identity=overwrite_source_identity,
                publication_guard_factory=lambda: (
                    _acquire_legacy_source_publication_guard(
                        data_dir, legacy_source_snapshot
                    )
                ),
            )
        _reject_sqlite_sidecars(db_path, "Published ConfigStore database")
    finally:
        if overwrite_stack is not None:
            overwrite_stack.close()
        target_directory_guard.close()

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
        "--verify-dpapi-backup-against",
        type=Path,
        default=None,
        help=(
            "Read-only verification that a DPAPI backup is a complete logical "
            "SQLite snapshot of the explicit --db source"
        ),
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
        "--create-debug-transfer-key",
        action="store_true",
        help=(
            "Create an RSA recipient public key and a CurrentUser-DPAPI "
            "private key for encrypted field-database reproduction"
        ),
    )
    parser.add_argument(
        "--export-debug-database",
        type=Path,
        default=None,
        metavar="PACKAGE",
        help=(
            "On the field PC, decrypt --db with its owning Windows user and "
            "write an end-to-end encrypted reproduction package"
        ),
    )
    parser.add_argument(
        "--import-debug-database",
        type=Path,
        default=None,
        metavar="PACKAGE",
        help=(
            "On the developer PC, decrypt a reproduction package and rebind "
            "its protected settings into the non-existing --db"
        ),
    )
    parser.add_argument(
        "--debug-public-key",
        type=Path,
        default=None,
        help="Absolute RSA public-key path for debug key creation or field export",
    )
    parser.add_argument(
        "--debug-private-key",
        type=Path,
        default=None,
        help=(
            "Absolute CurrentUser-DPAPI private-key path for debug key "
            "creation or local import"
        ),
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

    debug_mode_count = sum((
        bool(args.create_debug_transfer_key),
        args.export_debug_database is not None,
        args.import_debug_database is not None,
    ))
    if debug_mode_count > 1:
        parser.error("Debug database transfer modes are mutually exclusive")
    if debug_mode_count:
        common_incompatible = (
            args.source is not None
            or args.overwrite
            or args.encrypt
            or args.scrub_legacy_credentials
            or args.defer_credential_scrub
            or args.installer_staging
            or args.restore_dpapi_backup is not None
            or args.verify_dpapi_backup_against is not None
            or args.upgrade_backup is not None
            or args.verify_current
            or args.verify_installer_state
            or args.encoding is not None
            or args.allow_mojibake
            or args.print_source_sha256
        )
        if common_incompatible:
            parser.error(
                "Debug database transfer cannot be combined with migration, "
                "overwrite, restore, or verify modes"
            )
        if args.create_debug_transfer_key:
            if (
                args.db is not None
                or args.debug_public_key is None
                or args.debug_private_key is None
            ):
                parser.error(
                    "--create-debug-transfer-key requires "
                    "--debug-public-key and --debug-private-key, without --db"
                )
        elif args.export_debug_database is not None:
            if (
                args.db is None
                or args.debug_public_key is None
                or args.debug_private_key is not None
            ):
                parser.error(
                    "--export-debug-database requires --db and "
                    "--debug-public-key only"
                )
        elif (
            args.db is None
            or args.debug_private_key is None
            or args.debug_public_key is not None
        ):
            parser.error(
                "--import-debug-database requires --db and "
                "--debug-private-key only"
            )
    elif (
        args.debug_public_key is not None
        or args.debug_private_key is not None
    ):
        parser.error(
            "Debug transfer key paths require a debug database transfer mode"
        )

    if args.verify_dpapi_backup_against is not None:
        incompatible = (
            args.source is not None
            or args.overwrite
            or args.encrypt
            or args.scrub_legacy_credentials
            or args.defer_credential_scrub
            or args.installer_staging
            or args.restore_dpapi_backup is not None
            or args.upgrade_backup is not None
            or args.verify_current
            or args.verify_installer_state
            or args.encoding is not None
            or args.allow_mojibake
            or args.print_source_sha256
        )
        if args.db is None or incompatible:
            parser.error(
                "--verify-dpapi-backup-against requires explicit --db and "
                "cannot be combined with other modes"
            )

    if args.verify_installer_state:
        incompatible = (
            args.overwrite
            or args.encrypt
            or args.scrub_legacy_credentials
            or args.defer_credential_scrub
            or args.installer_staging
            or args.restore_dpapi_backup is not None
            or args.verify_dpapi_backup_against is not None
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
            or args.verify_dpapi_backup_against is not None
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
        if args.create_debug_transfer_key:
            create_debug_transfer_key(
                args.debug_public_key, args.debug_private_key
            )
            return
        if args.export_debug_database is not None:
            export_debug_database(
                Path(args.db),
                args.export_debug_database,
                args.debug_public_key,
            )
            return
        if args.import_debug_database is not None:
            import_debug_database(
                args.import_debug_database,
                Path(args.db),
                args.debug_private_key,
            )
            return
        source = Path(args.source) if args.source is not None else Path("Data")
        db = Path(args.db) if args.db else source / "ConfigStore.db"
        if args.verify_dpapi_backup_against is not None:
            verify_dpapi_database_backup_against(
                args.verify_dpapi_backup_against, db
            )
            print(
                "Verified DPAPI backup logical snapshot against source database: "
                f"{_absolute_lexical_path(db)}"
            )
            return
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
                or args.verify_dpapi_backup_against is not None
                or args.upgrade_backup is not None
                or args.encoding is not None
                or args.allow_mojibake
            )
            if args.db is None or incompatible:
                raise ValueError(
                    "--verify-current requires explicit --db and cannot be combined with migration options"
                )
            scrub_state = verify_current_database(db)
            print(
                f"Verified current ConfigStore schema v{SCHEMA_VERSION} "
                f"(scrub={scrub_state}): {_absolute_lexical_path(db)}"
            )
            return
        if args.restore_dpapi_backup is not None:
            lexical_backup = _absolute_lexical_path(args.restore_dpapi_backup)
            lexical_db = _absolute_lexical_path(db)
            print(f"Authorized DPAPI backup path: {lexical_backup}")
            print(f"Authorized restore target path: {lexical_db}")
            restore_dpapi_database_backup(
                lexical_backup, lexical_db, args.overwrite
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
