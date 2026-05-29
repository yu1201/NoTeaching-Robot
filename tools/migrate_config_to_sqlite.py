#!/usr/bin/env python3
"""Migrate legacy Data/*.ini and weld process txt files into ConfigStore.db.

The main program intentionally does not read legacy files anymore. Run this
tool once on an existing Data directory before starting the new build.
"""

from __future__ import annotations

import argparse
import base64
import datetime as _dt
import hashlib
import os
import re
import shutil
import sqlite3
import sys
from pathlib import Path


SECRET = b"NoTeachingRobotConfigStoreV1"
SCHEMA_VERSION = "2"
TEXT_FILE_NAMES = {"WeaveDate.txt", "WeldPara.txt"}
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


def key_stream(nonce: bytes, size: int) -> bytes:
    output = bytearray()
    counter = 0
    while len(output) < size:
        block = SECRET + nonce + b":" + str(counter).encode("ascii")
        output.extend(hashlib.sha256(block).digest())
        counter += 1
    return bytes(output[:size])


def protect_text(text: str) -> str:
    nonce = os.urandom(16)
    data = bytearray(text.encode("utf-8"))
    stream = key_stream(nonce, len(data))
    for index, byte in enumerate(stream):
        data[index] ^= byte
    nonce_b64 = base64.b64encode(nonce).decode("ascii").rstrip("=")
    data_b64 = base64.b64encode(bytes(data)).decode("ascii").rstrip("=")
    return f"enc:v1:{nonce_b64}:{data_b64}"


def stored_text(text: str, encrypt: bool) -> tuple[str, int]:
    if not encrypt:
        return text, 0
    return protect_text(text), 1


def create_current_tables(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        CREATE TABLE IF NOT EXISTS meta (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS ini_values (
            category TEXT NOT NULL DEFAULT '',
            robot_name TEXT NOT NULL DEFAULT '',
            file_name TEXT NOT NULL DEFAULT '',
            group_name TEXT NOT NULL DEFAULT '',
            item_name TEXT NOT NULL DEFAULT '',
            key_name TEXT NOT NULL DEFAULT '',
            source_path TEXT NOT NULL,
            source_section TEXT NOT NULL,
            source_key TEXT NOT NULL,
            value_text TEXT NOT NULL,
            encrypted INTEGER NOT NULL DEFAULT 0,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            PRIMARY KEY(source_path, source_section, source_key)
        );
        CREATE INDEX IF NOT EXISTS idx_ini_values_display
            ON ini_values(category, robot_name, file_name, group_name, item_name, key_name);
        CREATE TABLE IF NOT EXISTS text_files (
            category TEXT NOT NULL DEFAULT '',
            robot_name TEXT NOT NULL DEFAULT '',
            file_name TEXT NOT NULL DEFAULT '',
            source_path TEXT PRIMARY KEY,
            content_text TEXT NOT NULL,
            encrypted INTEGER NOT NULL DEFAULT 0,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        );
        CREATE INDEX IF NOT EXISTS idx_text_files_display
            ON text_files(category, robot_name, file_name);
        """
    )


def set_schema_meta(conn: sqlite3.Connection, encrypt: bool) -> None:
    now = _dt.datetime.now().isoformat(timespec="seconds")
    conn.execute("INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)", ("schema_version", SCHEMA_VERSION))
    conn.execute("INSERT OR IGNORE INTO meta(key, value) VALUES(?, ?)", ("created_at", now))
    conn.execute("INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)", ("encrypt_new_values", "1" if encrypt else "0"))


def init_schema(conn: sqlite3.Connection, encrypt: bool) -> None:
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
    required = {
        "category",
        "robot_name",
        "file_name",
        "group_name",
        "item_name",
        "key_name",
        "source_path",
        "source_section",
        "source_key",
        "value_text",
        "encrypted",
        "updated_at",
    }
    return table_exists(conn, "ini_values") and required.issubset(table_columns(conn, "ini_values"))


def has_legacy_schema(conn: sqlite3.Connection) -> bool:
    required = {"file_path", "section_name", "key_name", "value_text"}
    return (
        table_exists(conn, "ini_values")
        and table_exists(conn, "text_files")
        and required.issubset(table_columns(conn, "ini_values"))
    )


def unique_table_name(conn: sqlite3.Connection, base_name: str) -> str:
    name = f"{base_name}_legacy"
    suffix = 1
    while table_exists(conn, name):
        name = f"{base_name}_legacy_{suffix}"
        suffix += 1
    return name


def refresh_current_schema_identities(conn: sqlite3.Connection) -> tuple[int, int]:
    ini_rows = conn.execute(
        "SELECT rowid, source_path, source_section, source_key FROM ini_values"
    ).fetchall()
    ini_updates = 0
    for rowid, source_path, section, key in ini_rows:
        identity = build_ini_identity(source_path or "", section or "", key or "")
        conn.execute(
            """
            UPDATE ini_values
            SET category=?, robot_name=?, file_name=?, group_name=?, item_name=?,
                key_name=?, source_path=?, source_section=?, source_key=?
            WHERE rowid=?
            """,
            (
                identity["category"],
                identity["robot_name"],
                identity["file_name"],
                identity["group_name"],
                identity["item_name"],
                identity["key_name"],
                identity["source_path"],
                identity["source_section"],
                identity["source_key"],
                rowid,
            ),
        )
        ini_updates += 1

    text_rows = conn.execute(
        "SELECT rowid, source_path FROM text_files"
    ).fetchall()
    text_updates = 0
    for rowid, source_path in text_rows:
        identity = build_file_identity(source_path or "")
        conn.execute(
            """
            UPDATE text_files
            SET category=?, robot_name=?, file_name=?, source_path=?
            WHERE rowid=?
            """,
            (
                identity["category"],
                identity["robot_name"],
                identity["file_name"],
                identity["source_path"],
                rowid,
            ),
        )
        text_updates += 1

    return ini_updates, text_updates


def migrate_existing_database_to_v2(db_path: Path, encrypt_new_values: bool) -> bool:
    if not db_path.exists():
        return False

    with sqlite3.connect(db_path) as conn:
        if has_current_schema(conn):
            ini_updates, text_updates = refresh_current_schema_identities(conn)
            set_schema_meta(conn, encrypt_new_values)
            conn.commit()
            print(f"Database already uses schema v{SCHEMA_VERSION}: {db_path}")
            print(f"Refreshed display fields: INI values={ini_updates}, text files={text_updates}")
            return True
        if not has_legacy_schema(conn):
            return False

    timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = db_path.with_suffix(db_path.suffix + f".bak_v1_{timestamp}")
    shutil.copy2(db_path, backup)

    with sqlite3.connect(db_path) as conn:
        legacy_ini = unique_table_name(conn, "ini_values")
        legacy_text = unique_table_name(conn, "text_files")
        try:
            conn.execute("BEGIN")
            conn.execute(f"ALTER TABLE ini_values RENAME TO {legacy_ini}")
            conn.execute(f"ALTER TABLE text_files RENAME TO {legacy_text}")
            create_current_tables(conn)

            for file_path, section, key, value, encrypted, updated_at in conn.execute(
                f"SELECT file_path, section_name, key_name, value_text, encrypted, updated_at FROM {legacy_ini}"
            ):
                identity = build_ini_identity(file_path, section, key)
                conn.execute(
                    """
                    INSERT OR REPLACE INTO ini_values
                        (category, robot_name, file_name, group_name, item_name, key_name,
                         source_path, source_section, source_key, value_text, encrypted, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        identity["category"],
                        identity["robot_name"],
                        identity["file_name"],
                        identity["group_name"],
                        identity["item_name"],
                        identity["key_name"],
                        identity["source_path"],
                        identity["source_section"],
                        identity["source_key"],
                        value,
                        encrypted,
                        updated_at,
                    ),
                )

            for file_path, content, encrypted, updated_at in conn.execute(
                f"SELECT file_path, content_text, encrypted, updated_at FROM {legacy_text}"
            ):
                identity = build_file_identity(file_path)
                conn.execute(
                    """
                    INSERT OR REPLACE INTO text_files(
                        category, robot_name, file_name, source_path, content_text, encrypted, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        identity["category"],
                        identity["robot_name"],
                        identity["file_name"],
                        identity["source_path"],
                        content,
                        encrypted,
                        updated_at,
                    ),
                )

            conn.execute(f"DROP TABLE {legacy_ini}")
            conn.execute(f"DROP TABLE {legacy_text}")
            set_schema_meta(conn, encrypt_new_values)
            conn.commit()
        except Exception:
            conn.rollback()
            raise

    print(f"Upgraded existing database to schema v{SCHEMA_VERSION}: {db_path}")
    print(f"Backed up old database: {backup}")
    return True


def migrate(
    data_dir: Path,
    db_path: Path,
    overwrite: bool,
    encrypt: bool,
    forced_encoding: str | None,
    allow_mojibake: bool,
) -> None:
    data_dir = data_dir.resolve()
    db_path = db_path.resolve()
    if not data_dir.exists():
        raise SystemExit(f"Data directory not found: {data_dir}")

    if db_path.exists():
        if not overwrite:
            if migrate_existing_database_to_v2(db_path, encrypt):
                return
            raise SystemExit(f"Database already exists, pass --overwrite: {db_path}")
        backup = db_path.with_suffix(db_path.suffix + ".bak")
        shutil.copy2(db_path, backup)
        db_path.unlink()
        print(f"Backed up old database: {backup}")

    db_path.parent.mkdir(parents=True, exist_ok=True)
    ini_count = 0
    text_count = 0
    value_count = 0
    default_replacements: list[tuple[str, str, str, str, str]] = []

    with sqlite3.connect(db_path) as conn:
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
                identity = build_ini_identity(file_path, section, key)
                text, encrypted = stored_text(value, encrypt)
                conn.execute(
                    """
                    INSERT OR REPLACE INTO ini_values
                        (category, robot_name, file_name, group_name, item_name, key_name,
                         source_path, source_section, source_key, value_text, encrypted, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))
                    """,
                    (
                        identity["category"],
                        identity["robot_name"],
                        identity["file_name"],
                        identity["group_name"],
                        identity["item_name"],
                        identity["key_name"],
                        identity["source_path"],
                        identity["source_section"],
                        identity["source_key"],
                        text,
                        encrypted,
                    ),
                )
                value_count += 1
            ini_count += 1

        for path in sorted(data_dir.rglob("*.txt")):
            if path.name not in TEXT_FILE_NAMES:
                continue
            file_path = normalize_data_path(path, data_dir)
            identity = build_file_identity(file_path)
            content = normalize_text(sanitize_legacy_text_file(path, forced_encoding, allow_mojibake))
            text, encrypted = stored_text(content, encrypt)
            conn.execute(
                """
                INSERT OR REPLACE INTO text_files(
                    category, robot_name, file_name, source_path, content_text, encrypted, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, datetime('now'))
                """,
                (
                    identity["category"],
                    identity["robot_name"],
                    identity["file_name"],
                    identity["source_path"],
                    text,
                    encrypted,
                ),
            )
            text_count += 1

        conn.commit()

    print(f"Created: {db_path}")
    print(f"INI files: {ini_count}, values: {value_count}")
    print(f"Text files: {text_count}")
    print(f"Encrypted: {'yes' if encrypt else 'no'}")
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Migrate Data config files to ConfigStore.db")
    parser.add_argument("--source", default="Data", help="Data directory to migrate")
    parser.add_argument("--db", default=None, help="Output SQLite database path")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing database")
    parser.add_argument("--encrypt", action="store_true", help="Encrypt stored values")
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
    args = parser.parse_args()

    should_pause = args.pause or launched_from_explorer()
    exit_code = 0
    try:
        source = Path(args.source)
        db = Path(args.db) if args.db else source / "ConfigStore.db"
        migrate(source, db, args.overwrite, args.encrypt, args.encoding, args.allow_mojibake)
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
