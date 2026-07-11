#!/usr/bin/env python3
"""Verify that the field ConfigMigrate.exe matches the current Python source."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import runpy


REPO_ROOT = Path(__file__).resolve().parents[2]


def run_command(
    command: list[str],
    cwd: Path,
    environment: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env = {**os.environ, "PYTHONUTF8": "1"}
    if environment:
        env.update(environment)
    return subprocess.run(
        command,
        cwd=cwd,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
        env=env,
    )


def run_checked(
    command: list[str],
    cwd: Path,
    environment: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    result = run_command(command, cwd, environment)
    if result.returncode != 0:
        raise AssertionError(
            f"Command failed ({result.returncode}): {command}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result


def write_fixture(root: Path) -> None:
    robot = root / "RobotA"
    robot.mkdir(parents=True)
    (robot / "RobotPara.ini").write_text(
        "[BaseParam]\nRobotType=2\nRobotName=机器人1\nPassword=secret-value\n",
        encoding="utf-8",
    )
    (robot / "CameraParam.ini").write_text(
        "[CAMERA0/Base]\nCameraReadFps=73\nCameraIP=192.0.2.10\n",
        encoding="utf-8",
    )
    (robot / "MeasureWeldParam.ini").write_text(
        "[Scan]\nScanSpeed=321\nRunSpeed=654\nCameraTimeOffsetMs=12\n",
        encoding="utf-8",
    )


def database_snapshot(
    path: Path,
    decrypt_value,
) -> tuple[tuple[tuple[str, str], ...], tuple[tuple[object, ...], ...]]:
    connection = sqlite3.connect(path)
    try:
        meta = tuple(
            sorted(
                (str(key), str(value))
                for key, value in connection.execute("SELECT key, value FROM meta")
                if key != "created_at"
            )
        )
        raw_settings = connection.execute(
                """
                SELECT scope_type, scope_id, module, key_name, value_text,
                       value_type, sensitive, encrypted
                FROM settings
                ORDER BY scope_type, scope_id, module, key_name
                """
            ).fetchall()
        settings = tuple(
            (*row[:4], decrypt_value(str(row[4]), row[7]) if row[7] else str(row[4]), *row[5:])
            for row in raw_settings
        )
    finally:
        connection.close()
    return meta, settings


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--exe",
        type=Path,
        default=REPO_ROOT / "tools" / "ConfigMigrate.exe",
    )
    args = parser.parse_args()

    source_script = REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"
    exe = args.exe.resolve()
    if not exe.is_file():
        raise AssertionError(f"ConfigMigrate.exe not found: {exe}")

    expected_hash = hashlib.sha256(source_script.read_bytes()).hexdigest()
    reported = run_checked([str(exe), "--print-source-sha256"], REPO_ROOT).stdout.strip().splitlines()[-1]
    if reported.lower() != expected_hash:
        raise AssertionError(f"Stale ConfigMigrate.exe: expected {expected_hash}, got {reported}")

    migration_module = runpy.run_path(str(source_script), run_name="config_migrate_parity_module")
    decrypt_value = migration_module["decode_stored_text"]

    (REPO_ROOT / "tmp").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="config-migrate-parity-",
        dir=REPO_ROOT / "tmp",
        ignore_cleanup_errors=True,
    ) as temp_text:
        temp = Path(temp_text)
        template = temp / "template" / "Data"
        write_fixture(template)
        python_db = temp / "python" / "target" / "ConfigStore.db"
        exe_db = temp / "exe" / "target" / "ConfigStore.db"

        run_checked(
            [sys.executable, str(source_script), "--source", str(template), "--db", str(python_db), "--encrypt"],
            temp,
        )
        run_checked(
            [str(exe), "--source", str(template), "--db", str(exe_db), "--encrypt"],
            temp,
        )
        python_snapshot = database_snapshot(python_db, decrypt_value)
        exe_snapshot = database_snapshot(exe_db, decrypt_value)
        if python_snapshot != exe_snapshot:
            raise AssertionError("Python and ConfigMigrate.exe produced different logical database contents")

        if any(
            module == "MeasureWeldParam" and key == "CameraReadFps"
            for _, _, module, key, *_ in exe_snapshot[1]
        ):
            raise AssertionError("Obsolete MeasureWeldParam/CameraReadFps default was reintroduced")

        caller = temp / "batch-caller"
        caller.mkdir()
        batch_target = caller / "relative-data-root"
        batch = REPO_ROOT / "tools" / "ConfigMigrate_Run.cmd"
        batch_result = run_checked(
            ["cmd", "/d", "/c", str(batch), "--data-root", "relative-data-root", "--source", str(template), "--no-pause"],
            caller,
        )
        batch_db = batch_target / "Data" / "ConfigStore.db"
        if not batch_db.is_file():
            raise AssertionError(f"Batch wrapper did not use the requested data root: {batch_db}")
        if "Resolved target database:" not in batch_result.stdout:
            raise AssertionError("Batch wrapper did not print the resolved target database")
        if database_snapshot(batch_db, decrypt_value) != python_snapshot:
            raise AssertionError("Batch data-root migration differs from direct migration")

        env_target = temp / "env-data-root"
        run_checked(
            ["cmd", "/d", "/c", str(batch), "--source", str(template), "--no-pause"],
            caller,
            {"QTWIDGETSAPP4_DATA_ROOT": str(env_target)},
        )
        env_db = env_target / "Data" / "ConfigStore.db"
        if not env_db.is_file() or database_snapshot(env_db, decrypt_value) != python_snapshot:
            raise AssertionError("Batch wrapper did not inherit QTWIDGETSAPP4_DATA_ROOT")

        existing_root = temp / "existing-v4-root"
        existing_db = existing_root / "Data" / "ConfigStore.db"
        empty_source = temp / "empty-source" / "Data"
        empty_source.mkdir(parents=True)
        run_checked(
            [str(exe), "--source", str(empty_source), "--db", str(existing_db), "--encrypt"],
            temp,
        )
        existing_before = database_snapshot(existing_db, decrypt_value)

        refused = run_command(
            [str(exe), "--source", str(template), "--db", str(existing_db), "--encrypt"],
            temp,
        )
        if refused.returncode == 0 or "already exists" not in (refused.stdout + refused.stderr):
            raise AssertionError("Existing current-schema database was not rejected without --overwrite")
        if database_snapshot(existing_db, decrypt_value) != existing_before:
            raise AssertionError("Refused current-schema migration modified the target database")

        batch_refused = run_command(
            [
                "cmd", "/d", "/c", str(batch),
                "--data-root", str(existing_root),
                "--source", str(template),
                "--no-pause",
            ],
            caller,
        )
        if batch_refused.returncode == 0:
            raise AssertionError("Batch wrapper silently accepted an existing v4 database")
        if database_snapshot(existing_db, decrypt_value) != existing_before:
            raise AssertionError("Batch refusal modified the existing v4 database")

        run_checked(
            [
                "cmd", "/d", "/c", str(batch),
                "--data-root", str(existing_root),
                "--source", str(template),
                "--overwrite",
                "--no-pause",
            ],
            caller,
        )
        overwritten = database_snapshot(existing_db, decrypt_value)
        if overwritten != python_snapshot:
            raise AssertionError("Explicit --overwrite did not rebuild the target from legacy Data")
        if not any(
            scope_id == "RobotA" and module.startswith("RobotPara")
            and key == "RobotType" and value == "2"
            for _, scope_id, module, key, value, *_ in overwritten[1]
        ):
            raise AssertionError("Explicit --overwrite did not import the legacy RobotPara values")
        if not list(existing_db.parent.glob("ConfigStore.db.bak_overwrite_*")):
            raise AssertionError("Explicit --overwrite did not create a timestamped database backup")

    print("PASS: ConfigMigrate parity, data-root wrapper, and existing-v4 fail-closed overwrite flow")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
