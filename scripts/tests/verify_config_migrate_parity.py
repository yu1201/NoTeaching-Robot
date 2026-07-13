#!/usr/bin/env python3
"""Verify that the field ConfigMigrate.exe matches the current Python source."""

from __future__ import annotations

import argparse
import base64
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


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def data_inventory(path: Path) -> dict[str, tuple[int, str]]:
    return {
        entry.relative_to(path).as_posix(): (
            entry.stat().st_size,
            file_sha256(entry),
        )
        for entry in path.rglob("*")
        if entry.is_file()
    }


def create_settings_schema(connection: sqlite3.Connection) -> None:
    connection.execute("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
    connection.execute(
        """
        CREATE TABLE settings(
            scope_type TEXT NOT NULL, scope_id TEXT NOT NULL DEFAULT '', module TEXT NOT NULL,
            key_name TEXT NOT NULL, value_text TEXT NOT NULL,
            value_type TEXT NOT NULL DEFAULT 'string', sensitive INTEGER NOT NULL DEFAULT 0,
            encrypted INTEGER NOT NULL DEFAULT 0,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            PRIMARY KEY(scope_type, scope_id, module, key_name)
        )
        """
    )


def write_auth1_database(
    path: Path,
    scrub_state: str,
    scrub_manifest_text: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    profile = {
        "PasswordHash": hashlib.sha256(
            b"admin\ninstaller-auth1-parity"
        ).hexdigest(),
        "Role": "admin",
        "MustChangePassword": "0",
        "CreatedAt": "2026-01-02T03:04:05",
        "UpdatedAt": "2026-02-03T04:05:06Z",
        "PasswordChangedAt": "2026-02-02T03:04:05+08:00",
    }
    with sqlite3.connect(path) as connection:
        create_settings_schema(connection)
        connection.executemany(
            "INSERT INTO meta(key, value) VALUES(?, ?)",
            (
                ("schema_version", "5"),
                ("auth_semantic_version", "1"),
                ("auth_initialized", "1"),
                ("encrypt_new_values", "1"),
                ("sensitive_protection", "dpapi-current-user-v1"),
                ("legacy_credential_scrub_state", scrub_state),
                ("legacy_credential_scrub_manifest", scrub_manifest_text),
            ),
        )
        for key, value in profile.items():
            connection.execute(
                """
                INSERT INTO settings(
                    scope_type, scope_id, module, key_name, value_text,
                    value_type, sensitive, encrypted
                ) VALUES('account', 'admin', 'Profile', ?, ?, ?, ?, 0)
                """,
                (
                    key,
                    value,
                    (
                        "bool" if key == "MustChangePassword"
                        else "datetime" if key.endswith("At")
                        else "string"
                    ),
                    1 if "Password" in key else 0,
                ),
            )


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
    user_name = "legacy_operator"
    temporary_password = "Only-For-Config-Migration-Test"
    legacy_hash = hashlib.sha256(f"{user_name}\n{temporary_password}".encode("utf-8")).hexdigest()
    admin_name = "legacy_admin"
    admin_password = "Only-For-Admin-Migration-Test"
    admin_hash = hashlib.sha256(
        f"{admin_name}\n{admin_password}".encode("utf-8")
    ).hexdigest()
    (root / "Accounts.ini").write_text(
        f"[Users/{user_name}]\nPasswordHash={legacy_hash}\nRole=operator\nCreatedAt=2026-01-02T03:04:05\n"
        f"[Users/{admin_name}]\nPasswordHash={admin_hash}\nRole=admin\nCreatedAt=2026-01-02T03:04:05\n",
        encoding="utf-8",
    )
    remembered = base64.b64encode(temporary_password.encode("utf-8")).decode("ascii")
    (root / "LoginState.ini").write_text(
        f"[General]\nUserName={user_name}\nRememberPassword=1\nAutoLogin=1\nPasswordBase64={remembered}\n"
        f"[SavedPasswords]\n{user_name}={remembered}\n",
        encoding="utf-8",
    )
    (root / "Process.ini").write_text(
        "[Runtime]\nPassCount=5\nBypassQualityGate=0\nCompassHeading=12\n"
        "FTPPassWord=Synthetic-Explicit-Credential\n",
        encoding="utf-8",
    )


def database_snapshot(
    path: Path,
    decrypt_value,
    purpose_value,
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
            (
                *row[:4],
                decrypt_value(
                    str(row[4]), row[7],
                    purpose_value(str(row[0]), str(row[1]), str(row[2]), str(row[3])),
                ) if row[7] else str(row[4]),
                *row[5:],
            )
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
    purpose_value = migration_module["protection_purpose"]
    protect_legacy_value = migration_module["protect_legacy_text"]
    protect_sensitive_value = migration_module["protect_sensitive_text"]
    read_dpapi_backup = migration_module["_read_dpapi_database_backup"]
    bootstrap_record_check = migration_module["_is_known_bootstrap_password_record"]
    bootstrap_salt = bytes(range(16))
    bootstrap_key = hashlib.pbkdf2_hmac("sha256", b"admin", bootstrap_salt, 600000, dklen=32)
    bootstrap_record = "pbkdf2-sha256:v1:600000:{}:{}".format(
        base64.urlsafe_b64encode(bootstrap_salt).decode("ascii").rstrip("="),
        base64.urlsafe_b64encode(bootstrap_key).decode("ascii").rstrip("="),
    )
    if not bootstrap_record_check("admin", bootstrap_record):
        raise AssertionError("Python migration does not recognize a PBKDF2 known bootstrap record")
    bootstrap_prefix, bootstrap_encoded_key = bootstrap_record.rsplit(":", 1)
    tampered_record = bootstrap_prefix + ":" + (
        ("A" if bootstrap_encoded_key[0] != "A" else "B") + bootstrap_encoded_key[1:]
    )
    if bootstrap_record_check("admin", tampered_record):
        raise AssertionError("Python migration accepted a tampered PBKDF2 bootstrap record")

    (REPO_ROOT / "tmp").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="config-migrate-parity-",
        dir=REPO_ROOT / "tmp",
        ignore_cleanup_errors=True,
    ) as temp_text:
        temp = Path(temp_text)
        template = temp / "template" / "Data"
        write_fixture(template)
        template_hashes = {
            path.relative_to(template): file_sha256(path)
            for path in template.rglob("*.ini")
        }
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
        python_snapshot = database_snapshot(python_db, decrypt_value, purpose_value)
        exe_snapshot = database_snapshot(exe_db, decrypt_value, purpose_value)
        if python_snapshot != exe_snapshot:
            raise AssertionError("Python and ConfigMigrate.exe produced different logical database contents")

        if any(
            module == "MeasureWeldParam" and key == "CameraReadFps"
            for _, _, module, key, *_ in exe_snapshot[1]
        ):
            raise AssertionError("Obsolete MeasureWeldParam/CameraReadFps default was reintroduced")

        if not any(
            scope == "account" and scope_id == "legacy_operator" and module == "Profile"
            and key == "Role" and value == "operator"
            for scope, scope_id, module, key, value, *_ in exe_snapshot[1]
        ):
            raise AssertionError("Legacy Accounts.ini account was not mapped to account/Profile")
        if not any(
            scope == "account" and scope_id == "legacy_admin" and module == "Profile"
            and key == "Role" and value == "admin"
            for scope, scope_id, module, key, value, *_ in exe_snapshot[1]
        ):
            raise AssertionError("Migrated account/Profile data has no administrator")
        if any(
            module.startswith("Accounts/Users/")
            or module.startswith("LoginState/SavedPasswords")
            or key.lower() == "passwordbase64"
            for _, _, module, key, *_ in exe_snapshot[1]
        ):
            raise AssertionError("Legacy reversible authentication state survived migration")
        with sqlite3.connect(exe_db) as raw_connection:
            schema = raw_connection.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
            if schema != ("5",):
                raise AssertionError(f"Unexpected schema version: {schema}")
            auth_initialized = raw_connection.execute(
                "SELECT value FROM meta WHERE key='auth_initialized'"
            ).fetchone()
            if auth_initialized != ("1",):
                raise AssertionError(f"Migrated authentication state is not initialized: {auth_initialized}")
            weak_sensitive = raw_connection.execute(
                """
                SELECT COUNT(*) FROM settings
                WHERE sensitive<>0 AND value_text NOT LIKE 'dpapi:user:v1:%'
                  AND NOT (
                    scope_type='account' AND module='Profile'
                    AND lower(key_name) IN ('passwordhash', 'mustchangepassword', 'passwordchangedat')
                  )
                """
            ).fetchone()[0]
            if weak_sensitive:
                raise AssertionError("Sensitive rows were not protected with DPAPI")

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
        if database_snapshot(batch_db, decrypt_value, purpose_value) != python_snapshot:
            raise AssertionError("Batch data-root migration differs from direct migration")

        env_target = temp / "env-data-root"
        run_checked(
            ["cmd", "/d", "/c", str(batch), "--source", str(template), "--no-pause"],
            caller,
            {"QTWIDGETSAPP4_DATA_ROOT": str(env_target)},
        )
        env_db = env_target / "Data" / "ConfigStore.db"
        if not env_db.is_file() or database_snapshot(env_db, decrypt_value, purpose_value) != python_snapshot:
            raise AssertionError("Batch wrapper did not inherit QTWIDGETSAPP4_DATA_ROOT")
        if {
            path.relative_to(template): file_sha256(path)
            for path in template.rglob("*.ini")
        } != template_hashes:
            raise AssertionError("Cross-directory migration modified the legacy source files")

        installer_runners = (
            ("python", [sys.executable, str(source_script)]),
            ("exe", [str(exe)]),
        )
        installer_create_data = temp / "installer-create" / "Data"
        write_fixture(installer_create_data)
        installer_create_source_hashes = {
            path.relative_to(installer_create_data): file_sha256(path)
            for path in installer_create_data.rglob("*.ini")
        }
        installer_create_snapshots = {}
        for label, runner in installer_runners:
            staging = installer_create_data / (
                ".ConfigStore.db.install-create-"
                + ("1" if label == "python" else "2") * 32
                + ".tmp"
            )
            run_checked(
                runner + [
                    "--source", str(installer_create_data),
                    "--db", str(staging),
                    "--encrypt",
                    "--defer-credential-scrub",
                    "--installer-staging",
                ],
                temp,
            )
            if (installer_create_data / "ConfigStore.db").exists():
                raise AssertionError(
                    f"{label} installer create staging wrote the final database"
                )
            if installer_create_source_hashes != {
                path.relative_to(installer_create_data): file_sha256(path)
                for path in installer_create_data.rglob("*.ini")
            }:
                raise AssertionError(
                    f"{label} installer create staging modified legacy source files"
                )
            before_verify = data_inventory(installer_create_data)
            verified = run_checked(
                runner + [
                    "--verify-installer-state",
                    "--source", str(installer_create_data),
                    "--db", str(staging),
                ],
                temp,
            )
            if "(scrub=pending)" not in verified.stdout:
                raise AssertionError(
                    f"{label} installer create staging did not verify as pending"
                )
            if data_inventory(installer_create_data) != before_verify:
                raise AssertionError(
                    f"{label} installer create verification modified Data"
                )
            installer_create_snapshots[label] = database_snapshot(
                staging, decrypt_value, purpose_value
            )
        if installer_create_snapshots["python"] != installer_create_snapshots["exe"]:
            python_meta, python_settings = installer_create_snapshots["python"]
            exe_meta, exe_settings = installer_create_snapshots["exe"]
            raise AssertionError(
                "Python and ConfigMigrate.exe installer create staging differ logically: "
                f"meta_only_python={set(python_meta) - set(exe_meta)!r}, "
                f"meta_only_exe={set(exe_meta) - set(python_meta)!r}, "
                f"settings_only_python={set(python_settings) - set(exe_settings)!r}, "
                f"settings_only_exe={set(exe_settings) - set(python_settings)!r}"
            )

        installer_upgrade_data = temp / "installer-upgrade" / "Data"
        installer_upgrade_data.mkdir(parents=True)
        installer_upgrade_final = installer_upgrade_data / "ConfigStore.db"
        admin_record = hashlib.sha256(
            b"admin\ninstaller-upgrade-parity"
        ).hexdigest()
        with sqlite3.connect(installer_upgrade_final) as connection:
            create_settings_schema(connection)
            connection.execute(
                "INSERT INTO meta(key, value) VALUES('schema_version', '4')"
            )
            connection.executemany(
                """
                INSERT INTO settings(
                    scope_type, scope_id, module, key_name, value_text,
                    value_type, sensitive, encrypted
                ) VALUES('global', '', 'Accounts/Users/admin', ?, ?, ?, ?, 0)
                """,
                (
                    ("PasswordHash", admin_record, "string", 1),
                    ("Role", "admin", "string", 0),
                    ("CreatedAt", "2026-01-02T03:04:05", "datetime", 0),
                ),
            )
        installer_upgrade_ini = (
            installer_upgrade_data / "CorrugatedSheetPointCloudEctration.ini"
        )
        installer_upgrade_ini.write_text(
            "ifupright=true\nSave_File_Name=installer-upgrade-parity\n",
            encoding="utf-8",
        )
        installer_upgrade_final_before = file_sha256(installer_upgrade_final)
        installer_upgrade_ini_before = file_sha256(installer_upgrade_ini)
        installer_upgrade_snapshots = {}
        for label, runner in installer_runners:
            staging = installer_upgrade_data / (
                ".ConfigStore.db.install-upgrade-"
                + ("3" if label == "python" else "4") * 32
                + ".tmp"
            )
            staging.write_bytes(installer_upgrade_final.read_bytes())
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )
            run_checked(
                runner + [
                    "--source", str(installer_upgrade_data),
                    "--db", str(staging),
                    "--encrypt",
                    "--defer-credential-scrub",
                    "--installer-staging",
                    "--upgrade-backup", str(backup),
                ],
                temp,
            )
            if file_sha256(installer_upgrade_final) != installer_upgrade_final_before:
                raise AssertionError(
                    f"{label} installer upgrade staging modified final ConfigStore.db"
                )
            if file_sha256(installer_upgrade_ini) != installer_upgrade_ini_before:
                raise AssertionError(
                    f"{label} installer upgrade staging modified legacy source files"
                )
            if not backup.is_file():
                raise AssertionError(
                    f"{label} installer upgrade staging did not create its bound backup"
                )
            before_verify = data_inventory(installer_upgrade_data)
            verified = run_checked(
                runner + [
                    "--verify-installer-state",
                    "--source", str(installer_upgrade_data),
                    "--db", str(staging),
                ],
                temp,
            )
            if "(scrub=pending)" not in verified.stdout:
                raise AssertionError(
                    f"{label} installer upgrade staging did not verify as pending"
                )
            if data_inventory(installer_upgrade_data) != before_verify:
                raise AssertionError(
                    f"{label} installer upgrade verification modified Data"
                )
            installer_upgrade_snapshots[label] = database_snapshot(
                staging, decrypt_value, purpose_value
            )
        if installer_upgrade_snapshots["python"] != installer_upgrade_snapshots["exe"]:
            raise AssertionError(
                "Python and ConfigMigrate.exe installer upgrade staging differ logically"
            )

        auth1_complete_data = temp / "installer-auth1-complete" / "Data"
        auth1_complete_data.mkdir(parents=True)
        auth1_complete_runtime = (
            auth1_complete_data / "CorrugatedSheetPointCloudEctration.ini"
        )
        auth1_complete_runtime.write_text(
            "ifupright=true\nSave_File_Name=auth1-complete-parity\n",
            encoding="utf-8",
        )
        auth1_complete_final = auth1_complete_data / "ConfigStore.db"
        write_auth1_database(auth1_complete_final, "complete", "[]")
        auth1_complete_final_hash = file_sha256(auth1_complete_final)
        auth1_complete_source_hash = file_sha256(auth1_complete_runtime)
        auth1_complete_snapshots = {}
        for label, runner in installer_runners:
            staging = auth1_complete_data / (
                ".ConfigStore.db.install-upgrade-"
                + ("5" if label == "python" else "6") * 32
                + ".tmp"
            )
            staging.write_bytes(auth1_complete_final.read_bytes())
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )
            result = run_checked(
                runner + [
                    "--source", str(auth1_complete_data),
                    "--db", str(staging),
                    "--encrypt",
                    "--defer-credential-scrub",
                    "--installer-staging",
                    "--upgrade-backup", str(backup),
                ],
                temp,
            )
            if "no installer finalization is required" not in result.stdout:
                raise AssertionError(
                    f"{label} auth1 complete staging incorrectly requested finalization"
                )
            before_verify = data_inventory(auth1_complete_data)
            verified = run_checked(
                runner + [
                    "--verify-installer-state",
                    "--source", str(auth1_complete_data),
                    "--db", str(staging),
                ],
                temp,
            )
            if "(scrub=complete)" not in verified.stdout:
                raise AssertionError(
                    f"{label} auth1 complete staging did not preserve complete provenance"
                )
            if data_inventory(auth1_complete_data) != before_verify:
                raise AssertionError(
                    f"{label} auth1 complete verification modified Data"
                )
            if (
                file_sha256(auth1_complete_final) != auth1_complete_final_hash
                or file_sha256(auth1_complete_runtime) != auth1_complete_source_hash
            ):
                raise AssertionError(
                    f"{label} auth1 complete staging modified final/source files"
                )
            auth1_complete_snapshots[label] = database_snapshot(
                staging, decrypt_value, purpose_value
            )
            staging.unlink()
            backup.unlink()
        if auth1_complete_snapshots["python"] != auth1_complete_snapshots["exe"]:
            raise AssertionError(
                "Python and ConfigMigrate.exe auth1 complete staging differ logically"
            )

        auth1_pending_data = temp / "installer-auth1-pending" / "Data"
        auth1_pending_data.mkdir(parents=True)
        auth1_pending_first = auth1_pending_data / "First.ini"
        auth1_pending_second = auth1_pending_data / "Second.ini"
        auth1_pending_first.write_text(
            "[Remote]\nApiToken=first-parity-secret\nKeepMe=1\n",
            encoding="utf-8",
        )
        auth1_pending_second.write_text(
            "[Remote]\nPassword=second-parity-secret\nKeepMe=2\n",
            encoding="utf-8",
        )
        auth1_pending_manifest = migration_module[
            "prepare_legacy_ini_credential_scrub"
        ](auth1_pending_data, None)
        auth1_pending_final = auth1_pending_data / "ConfigStore.db"
        write_auth1_database(
            auth1_pending_final,
            "pending",
            migration_module["serialize_legacy_credential_scrub_manifest"](
                auth1_pending_manifest
            ),
        )
        first_after, _removed = migration_module[
            "_sanitized_legacy_ini_bytes"
        ](auth1_pending_first, None)
        auth1_pending_first.write_bytes(first_after)
        auth1_pending_final_hash = file_sha256(auth1_pending_final)
        auth1_pending_source_hashes = {
            path.name: file_sha256(path)
            for path in (auth1_pending_first, auth1_pending_second)
        }
        auth1_pending_snapshots = {}
        for label, runner in installer_runners:
            staging = auth1_pending_data / (
                ".ConfigStore.db.install-upgrade-"
                + ("7" if label == "python" else "8") * 32
                + ".tmp"
            )
            staging.write_bytes(auth1_pending_final.read_bytes())
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )
            run_checked(
                runner + [
                    "--source", str(auth1_pending_data),
                    "--db", str(staging),
                    "--encrypt",
                    "--defer-credential-scrub",
                    "--installer-staging",
                    "--upgrade-backup", str(backup),
                ],
                temp,
            )
            before_verify = data_inventory(auth1_pending_data)
            verified = run_checked(
                runner + [
                    "--verify-installer-state",
                    "--source", str(auth1_pending_data),
                    "--db", str(staging),
                ],
                temp,
            )
            if "(scrub=pending)" not in verified.stdout:
                raise AssertionError(
                    f"{label} auth1 partial-after staging did not preserve pending provenance"
                )
            if data_inventory(auth1_pending_data) != before_verify:
                raise AssertionError(
                    f"{label} auth1 pending verification modified Data"
                )
            if file_sha256(auth1_pending_final) != auth1_pending_final_hash:
                raise AssertionError(
                    f"{label} auth1 partial-after staging modified final ConfigStore.db"
                )
            if {
                path.name: file_sha256(path)
                for path in (auth1_pending_first, auth1_pending_second)
            } != auth1_pending_source_hashes:
                raise AssertionError(
                    f"{label} auth1 partial-after staging modified source files"
                )
            auth1_pending_snapshots[label] = database_snapshot(
                staging, decrypt_value, purpose_value
            )
            staging.unlink()
            backup.unlink()
        if auth1_pending_snapshots["python"] != auth1_pending_snapshots["exe"]:
            raise AssertionError(
                "Python and ConfigMigrate.exe auth1 partial-after staging differ logically"
            )

        verify_complete_data = temp / "verify-complete-new-credentials" / "Data"
        verify_complete_data.mkdir(parents=True)
        verify_complete_db = verify_complete_data / "ConfigStore.db"
        write_auth1_database(verify_complete_db, "complete", "[]")
        with sqlite3.connect(verify_complete_db) as connection:
            connection.execute(
                "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
            )
            connection.commit()
        (verify_complete_data / "Unexpected.ini").write_text(
            "[Remote]\nPassword=new-password\nApiToken=new-token\n",
            encoding="utf-8",
        )
        verify_complete_before = data_inventory(verify_complete_data)
        for label, runner in installer_runners:
            result = run_command(
                runner + [
                    "--verify-installer-state",
                    "--source", str(verify_complete_data),
                    "--db", str(verify_complete_db),
                ],
                temp,
            )
            if result.returncode == 0:
                raise AssertionError(
                    f"{label} installer verification accepted credentials added after complete scrub"
                )
            if data_inventory(verify_complete_data) != verify_complete_before:
                raise AssertionError(
                    f"{label} rejected complete-state verification modified Data"
                )

        for case in ("unknown", "changed"):
            verify_pending_data = (
                temp / f"verify-pending-{case}" / "Data"
            )
            verify_pending_data.mkdir(parents=True)
            verify_pending_known = verify_pending_data / "Known.ini"
            verify_pending_known.write_text(
                "[Remote]\nApiToken=known-secret\nKeepMe=1\n",
                encoding="utf-8",
            )
            verify_pending_manifest = migration_module[
                "prepare_legacy_ini_credential_scrub"
            ](verify_pending_data, None)
            verify_pending_db = verify_pending_data / "ConfigStore.db"
            write_auth1_database(
                verify_pending_db,
                "pending",
                migration_module[
                    "serialize_legacy_credential_scrub_manifest"
                ](verify_pending_manifest),
            )
            with sqlite3.connect(verify_pending_db) as connection:
                connection.execute(
                    "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
                )
                connection.commit()
            if case == "unknown":
                (verify_pending_data / "Unknown.ini").write_text(
                    "[Remote]\nPassword=unknown-secret\n",
                    encoding="utf-8",
                )
            else:
                verify_pending_known.write_bytes(
                    verify_pending_known.read_bytes() + b"Changed=1\n"
                )
            verify_pending_before = data_inventory(verify_pending_data)
            for label, runner in installer_runners:
                result = run_command(
                    runner + [
                        "--verify-installer-state",
                        "--source", str(verify_pending_data),
                        "--db", str(verify_pending_db),
                    ],
                    temp,
                )
                if result.returncode == 0:
                    raise AssertionError(
                        f"{label} installer verification accepted {case} pending credentials"
                    )
                if data_inventory(verify_pending_data) != verify_pending_before:
                    raise AssertionError(
                        f"{label} rejected {case} pending verification modified Data"
                    )

        for case in ("database-backup", "credential-ini-backup"):
            residue_data = temp / f"verify-pending-residue-{case}" / "Data"
            residue_data.mkdir(parents=True)
            (residue_data / "Known.ini").write_text(
                "[Remote]\nApiToken=known-secret\nKeepMe=1\n",
                encoding="utf-8",
            )
            if case == "database-backup":
                (residue_data / "ConfigStore.db.bak-preexisting").write_bytes(
                    b"plaintext database backup sentinel"
                )
            else:
                (residue_data / "Credentials.ini.bak").write_text(
                    "[Remote]\nPassword=backup-secret\nKeepMe=2\n",
                    encoding="utf-8",
                )
            residue_manifest = migration_module[
                "prepare_legacy_ini_credential_scrub"
            ](residue_data, None)
            residue_db = residue_data / "ConfigStore.db"
            write_auth1_database(
                residue_db,
                "pending",
                migration_module[
                    "serialize_legacy_credential_scrub_manifest"
                ](residue_manifest),
            )
            with sqlite3.connect(residue_db) as connection:
                connection.execute(
                    "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
                )
                connection.commit()
            residue_before = data_inventory(residue_data)
            for label, runner in installer_runners:
                result = run_command(
                    runner + [
                        "--verify-installer-state",
                        "--source", str(residue_data),
                        "--db", str(residue_db),
                    ],
                    temp,
                )
                combined_output = result.stdout + result.stderr
                if (
                    result.returncode == 0
                    or "Plaintext ConfigStore backup/journal candidates block release"
                    not in combined_output
                ):
                    raise AssertionError(
                        f"{label} installer verification accepted pending {case} residue: "
                        f"{combined_output}"
                    )
                if data_inventory(residue_data) != residue_before:
                    raise AssertionError(
                        f"{label} rejected pending {case} residue modified Data"
                    )

        in_place_data = temp / "in-place" / "Data"
        write_fixture(in_place_data)
        with (in_place_data / "LoginState.ini").open("a", encoding="utf-8") as stream:
            stream.write("[WindowState]\nKeepMe=preserved\n; preserved comment\n")
        (in_place_data / "LoginState_backup.ini").write_text(
            "[General]\nUserName=backup_user\nRememberPassword=1\nAutoLogin=1\n"
            "PasswordBase64=U3ludGhldGlj\n[SavedPasswords]\nbackup_user=U3ludGhldGlj\n",
            encoding="utf-8",
        )
        (in_place_data / "Accounts_backup.ini").write_text(
            "[Users/backup_user]\nPasswordHash="
            + hashlib.sha256(b"backup_user\nSynthetic").hexdigest()
            + "\nRole=operator\n",
            encoding="utf-8",
        )
        login_backup = in_place_data / "LoginState.ini.bak-before-sec1"
        login_backup.write_bytes((in_place_data / "LoginState.ini").read_bytes())
        login_backup_hash = file_sha256(login_backup)
        in_place_db = in_place_data / "ConfigStore.db"
        gated_migration = run_command(
            [
                str(exe), "--source", str(in_place_data), "--db", str(in_place_db),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        if gated_migration.returncode == 0 or "SECURITY GATE" not in (
            gated_migration.stdout + gated_migration.stderr
        ):
            raise AssertionError("Plaintext legacy INI backup did not block migration completion")
        if (
            not in_place_db.is_file()
            or file_sha256(login_backup) != login_backup_hash
            or "PasswordBase64=" not in login_backup.read_text(encoding="utf-8")
        ):
            raise AssertionError("Credential backup gate changed the unproven INI backup or lost the migrated DB")
        with sqlite3.connect(in_place_db) as connection:
            scrub_state = connection.execute(
                "SELECT value FROM meta WHERE key='legacy_credential_scrub_state'"
            ).fetchone()
        if scrub_state != ("pending",):
            raise AssertionError(f"Credential scrub gate did not preserve pending provenance: {scrub_state}")
        login_backup.unlink()
        run_checked(
            [
                str(exe), "--source", str(in_place_data), "--db", str(in_place_db),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        with sqlite3.connect(in_place_db) as connection:
            scrub_state = connection.execute(
                "SELECT value FROM meta WHERE key='legacy_credential_scrub_state'"
            ).fetchone()
        if scrub_state != ("complete",):
            raise AssertionError(f"Credential scrub retry did not complete provenance: {scrub_state}")
        accounts_after = (in_place_data / "Accounts.ini").read_text(encoding="utf-8")
        login_after = (in_place_data / "LoginState.ini").read_text(encoding="utf-8")
        robot_after = (in_place_data / "RobotA" / "RobotPara.ini").read_text(encoding="utf-8")
        process_after = (in_place_data / "Process.ini").read_text(encoding="utf-8")
        login_named_backup_after = (in_place_data / "LoginState_backup.ini").read_text(encoding="utf-8")
        accounts_named_backup_after = (in_place_data / "Accounts_backup.ini").read_text(encoding="utf-8")
        if "PasswordHash=" in accounts_after or "Role=operator" not in accounts_after or "CreatedAt=" not in accounts_after:
            raise AssertionError("In-place account scrub did not remove only the legacy password record")
        if (
            "PasswordBase64=" in login_after
            or "[SavedPasswords]" in login_after
            or "RememberPassword=0" not in login_after
            or "AutoLogin=0" not in login_after
            or "UserName=legacy_operator" not in login_after
            or "KeepMe=preserved" not in login_after
            or "; preserved comment" not in login_after
        ):
            raise AssertionError("In-place login-state scrub lost safe values or retained reversible credentials")
        if "Password=" in robot_after or "RobotType=2" not in robot_after:
            raise AssertionError("In-place remote credential scrub lost non-secret robot configuration")
        if (
            "FTPPassWord=" in process_after
            or "PassCount=5" not in process_after
            or "BypassQualityGate=0" not in process_after
            or "CompassHeading=12" not in process_after
        ):
            raise AssertionError("Credential scrub removed pass-substring non-secret fields")
        if "PasswordBase64=" in login_named_backup_after or "[SavedPasswords]" in login_named_backup_after:
            raise AssertionError("LoginState_backup.ini retained reversible login credentials")
        if "PasswordHash=" in accounts_named_backup_after or "Role=operator" not in accounts_named_backup_after:
            raise AssertionError("Accounts_backup.ini scrub did not preserve safe account metadata")
        in_place_snapshot = database_snapshot(in_place_db, decrypt_value, purpose_value)
        clean_login_state = (in_place_data / "LoginState.ini").read_bytes()
        with (in_place_data / "LoginState.ini").open("a", encoding="utf-8") as stream:
            stream.write("[SavedPasswords]\nlegacy_operator=U3ludGhldGljLUNyYXNoLVJlc2lkdWFs\n")
        reprovenance_result = run_command(
            [
                str(exe), "--source", str(in_place_data), "--db", str(in_place_db),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        if reprovenance_result.returncode == 0 or "appeared after the proven migration scrub" not in (
            reprovenance_result.stdout + reprovenance_result.stderr
        ):
            raise AssertionError("Credential data added after completed scrub was accepted without provenance")
        if database_snapshot(in_place_db, decrypt_value, purpose_value) != in_place_snapshot:
            raise AssertionError("Rejected post-scrub credential data modified the current database")
        if "[SavedPasswords]" not in (in_place_data / "LoginState.ini").read_text(encoding="utf-8"):
            raise AssertionError("Rejected post-scrub credential data was deleted without provenance")
        (in_place_data / "LoginState.ini").write_bytes(clean_login_state)
        recovered_hashes = {
            path.relative_to(in_place_data): file_sha256(path)
            for path in in_place_data.rglob("*")
            if path.is_file() and ".ini" in path.name.lower()
        }
        run_checked(
            [
                str(exe), "--source", str(in_place_data), "--db", str(in_place_db),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        if {
            path.relative_to(in_place_data): file_sha256(path)
            for path in in_place_data.rglob("*")
            if path.is_file() and ".ini" in path.name.lower()
        } != recovered_hashes:
            raise AssertionError("Second credential scrub was not byte-idempotent")

        in_place_collision = temp / "in-place-collision" / "Data"
        in_place_collision.mkdir(parents=True)
        collision_hash_a = hashlib.sha256(b"Foo\nSynthetic-A").hexdigest()
        collision_hash_b = hashlib.sha256(b"foo\nSynthetic-B").hexdigest()
        collision_accounts = in_place_collision / "Accounts.ini"
        collision_login = in_place_collision / "LoginState.ini"
        collision_accounts.write_text(
            "[Users/Foo]\n"
            f"PasswordHash={collision_hash_a}\nRole=admin\n"
            "[Users/foo]\n"
            f"PasswordHash={collision_hash_b}\nRole=operator\n",
            encoding="utf-8",
        )
        collision_login.write_text(
            "[General]\nRememberPassword=1\nPasswordBase64=U3ludGhldGlj\n",
            encoding="utf-8",
        )
        collision_before = (file_sha256(collision_accounts), file_sha256(collision_login))
        collision_failed = run_command(
            [
                str(exe), "--source", str(in_place_collision),
                "--db", str(in_place_collision / "ConfigStore.db"),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        if collision_failed.returncode == 0:
            raise AssertionError("In-place case-collision fixture unexpectedly succeeded")
        if collision_before != (file_sha256(collision_accounts), file_sha256(collision_login)):
            raise AssertionError("Failed in-place migration modified legacy credential files")

        unproven_data = temp / "unproven-v5" / "Data"
        unproven_data.mkdir(parents=True)
        unproven_db = unproven_data / "ConfigStore.db"
        unproven_empty_source = temp / "unproven-empty-source"
        unproven_empty_source.mkdir()
        run_checked(
            [
                str(exe), "--source", str(unproven_empty_source),
                "--db", str(unproven_db), "--encrypt",
            ],
            temp,
        )
        unproven_db_before = database_snapshot(unproven_db, decrypt_value, purpose_value)
        write_fixture(unproven_data)
        unproven_source_hashes = {
            path.relative_to(unproven_data): file_sha256(path)
            for path in unproven_data.rglob("*.ini")
        }
        unproven_result = run_command(
            [
                str(exe), "--source", str(unproven_data), "--db", str(unproven_db),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        if unproven_result.returncode == 0 or "no pending legacy-migration scrub provenance" not in (
            unproven_result.stdout + unproven_result.stderr
        ):
            raise AssertionError("Unproven current-v5 source was accepted for destructive credential scrub")
        if database_snapshot(unproven_db, decrypt_value, purpose_value) != unproven_db_before:
            raise AssertionError("Rejected unproven credential scrub modified the current database")
        if {
            path.relative_to(unproven_data): file_sha256(path)
            for path in unproven_data.rglob("*.ini")
        } != unproven_source_hashes:
            raise AssertionError("Rejected unproven credential scrub modified legacy source files")

        unproven_v4_data = temp / "unproven-v4" / "Data"
        unproven_v4_data.mkdir(parents=True)
        unproven_v4_db = unproven_v4_data / "ConfigStore.db"
        with sqlite3.connect(unproven_v4_db) as connection:
            connection.execute("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
            connection.execute("INSERT INTO meta(key, value) VALUES('schema_version', '4')")
            connection.execute(
                """
                CREATE TABLE settings(
                    scope_type TEXT NOT NULL, scope_id TEXT NOT NULL DEFAULT '', module TEXT NOT NULL,
                    key_name TEXT NOT NULL, value_text TEXT NOT NULL,
                    value_type TEXT NOT NULL DEFAULT 'string', sensitive INTEGER NOT NULL DEFAULT 0,
                    encrypted INTEGER NOT NULL DEFAULT 0,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    PRIMARY KEY(scope_type, scope_id, module, key_name)
                )
                """
            )
        connection.close()
        write_fixture(unproven_v4_data)
        unproven_v4_db_hash = file_sha256(unproven_v4_db)
        unproven_v4_source_hashes = {
            path.relative_to(unproven_v4_data): file_sha256(path)
            for path in unproven_v4_data.rglob("*.ini")
        }
        unproven_v4_result = run_command(
            [
                str(exe), "--source", str(unproven_v4_data), "--db", str(unproven_v4_db),
                "--encrypt", "--scrub-legacy-credentials",
            ],
            temp,
        )
        if unproven_v4_result.returncode == 0 or "does not import legacy INI/TXT" not in (
            unproven_v4_result.stdout + unproven_v4_result.stderr
        ):
            raise AssertionError("v4 upgrade scrubbed credential-bearing disk INI without importing it")
        if file_sha256(unproven_v4_db) != unproven_v4_db_hash:
            raise AssertionError("Rejected v4 disk-credential migration modified the database")
        if {
            path.relative_to(unproven_v4_data): file_sha256(path)
            for path in unproven_v4_data.rglob("*.ini")
        } != unproven_v4_source_hashes:
            raise AssertionError("Rejected v4 disk-credential migration modified source INI files")

        semantic_source = temp / "semantic-upgrade-empty-source"
        semantic_source.mkdir()
        for initialized_value in ("1",):
            semantic_account = f"semantic_user_{initialized_value}"
            semantic_password_record = hashlib.sha256(
                f"{semantic_account}\nSynthetic-Semantic-Password".encode("utf-8")
            ).hexdigest()
            semantic_db = (
                temp / f"semantic-v1-initialized-{initialized_value}"
                / "ConfigStore.db"
            )
            semantic_db.parent.mkdir(parents=True)
            connection = sqlite3.connect(semantic_db)
            try:
                create_settings_schema(connection)
                connection.executemany(
                    "INSERT INTO meta(key, value) VALUES(?, ?)",
                    (
                        ("schema_version", "5"),
                        ("auth_semantic_version", "1"),
                        ("auth_initialized", initialized_value),
                        ("encrypt_new_values", "1"),
                    ),
                )
                password_purpose = purpose_value(
                    "account", semantic_account, "Profile", "PasswordHash"
                )
                password_stored = (
                    protect_sensitive_value(
                        semantic_password_record, password_purpose
                    )
                    if initialized_value == "1"
                    else protect_legacy_value(semantic_password_record)
                )
                connection.execute(
                    """
                    INSERT INTO settings(
                        scope_type, scope_id, module, key_name, value_text,
                        value_type, sensitive, encrypted
                    ) VALUES('account', ?, 'Profile', 'PasswordHash', ?, 'string', 1, 1)
                    """,
                    (semantic_account, password_stored),
                )
                connection.execute(
                    """
                    INSERT INTO settings(
                        scope_type, scope_id, module, key_name, value_text,
                        value_type, sensitive, encrypted
                    ) VALUES('account', ?, 'Profile', 'Role', ?, 'string', 0, 1)
                    """,
                    (semantic_account, protect_legacy_value("admin")),
                )
                if initialized_value == "0":
                    must_change_purpose = purpose_value(
                        "account", semantic_account, "Profile", "MustChangePassword"
                    )
                    connection.execute(
                        """
                        INSERT INTO settings(
                            scope_type, scope_id, module, key_name, value_text,
                            value_type, sensitive, encrypted
                        ) VALUES(
                            'account', ?, 'Profile', 'MustChangePassword',
                            ?, 'bool', 1, 1
                        )
                        """,
                        (
                            semantic_account,
                            protect_sensitive_value("false", must_change_purpose),
                        ),
                    )
                connection.commit()
            finally:
                connection.close()

            semantic_before = file_sha256(semantic_db)
            semantic_result = run_command(
                [
                    str(exe), "--source", str(semantic_source),
                    "--db", str(semantic_db), "--encrypt",
                ],
                temp,
            )
            if (
                semantic_result.returncode == 0
                or "portable plaintext storage" not in (
                    semantic_result.stdout + semantic_result.stderr
                )
                or file_sha256(semantic_db) != semantic_before
            ):
                raise AssertionError(
                    "Machine-bound existing account/Profile values were not rejected atomically"
                )

        portable_values = {
            "PasswordHash": hashlib.sha256(
                b"admin\nPortable-Profile-Parity"
            ).hexdigest(),
            "Role": "admin",
            "MustChangePassword": "0",
            "PasswordChangedAt": "2026-05-28T12:34:56+08:00",
            "CreatedAt": "2026-05-07T00:00:00",
            "UpdatedAt": "2026-06-01T00:00:00Z",
        }
        for wrapper in ("dpapi", "enc"):
            for field, plain in portable_values.items():
                portable_db = (
                    temp / f"portable-{wrapper}-{field}" / "ConfigStore.db"
                )
                portable_db.parent.mkdir(parents=True)
                with sqlite3.connect(portable_db) as connection:
                    migration_module["create_current_tables"](connection)
                    migration_module["set_schema_meta"](
                        connection, True, authentication_initialized=True
                    )
                    for candidate, value in portable_values.items():
                        stored = value
                        if candidate == field:
                            stored = (
                                protect_sensitive_value(
                                    value,
                                    purpose_value(
                                        "account", "admin", "Profile", candidate
                                    ),
                                )
                                if wrapper == "dpapi"
                                else protect_legacy_value(value)
                            )
                        connection.execute(
                            """
                            INSERT INTO settings(
                                scope_type, scope_id, module, key_name,
                                value_text, value_type, sensitive, encrypted
                            ) VALUES('account', 'admin', 'Profile', ?, ?, ?, ?, 0)
                            """,
                            (
                                candidate,
                                stored,
                                "bool" if candidate == "MustChangePassword"
                                else "datetime" if candidate.endswith("At")
                                else "string",
                                1 if "Password" in candidate else 0,
                            ),
                        )
                portable_before = file_sha256(portable_db)
                portable_result = run_command(
                    [str(exe), "--verify-current", "--db", str(portable_db)],
                    temp,
                )
                if (
                    portable_result.returncode == 0
                    or "portable plaintext storage" not in (
                        portable_result.stdout + portable_result.stderr
                    )
                    or file_sha256(portable_db) != portable_before
                ):
                    raise AssertionError(
                        f"Wrapped portable field was not rejected read-only: {wrapper}/{field}"
                    )

        credential_modules = (
            "LoginState/RememberedCredentials",
            "LoginState/SavedPasswords",
        )
        for module in credential_modules:
            credential_db = temp / f"credential-{module.rsplit('/', 1)[-1]}" / "ConfigStore.db"
            credential_db.parent.mkdir(parents=True)
            protected = protect_sensitive_value(
                "remembered-secret",
                purpose_value("global", "", module, "admin"),
            )
            with sqlite3.connect(credential_db) as connection:
                migration_module["create_current_tables"](connection)
                migration_module["set_schema_meta"](
                    connection, True, authentication_initialized=False
                )
                connection.execute(
                    """
                    INSERT INTO settings(
                        scope_type, scope_id, module, key_name, value_text,
                        value_type, sensitive, encrypted
                    ) VALUES('global', '', ?, 'admin', ?, 'string', 1, 1)
                    """,
                    (module, protected),
                )
            credential_before = file_sha256(credential_db)
            run_checked(
                [str(exe), "--verify-current", "--db", str(credential_db)],
                temp,
            )
            if file_sha256(credential_db) != credential_before:
                raise AssertionError(
                    f"Read-only credential verification modified {module}"
                )

        invalid_credential_shapes = (
            ("scope", "robot", "RobotA", credential_modules[0], "admin", "protected"),
            ("case", "global", "", "loginstate/rememberedcredentials", "admin", "protected"),
            ("nested", "global", "", credential_modules[0] + "/admin", "admin", "protected"),
            ("plaintext", "global", "", credential_modules[0], "admin", "plaintext"),
            ("damaged", "global", "", credential_modules[0], "admin", "damaged"),
        )
        for name, scope_type, scope_id, module, key, storage in invalid_credential_shapes:
            credential_db = temp / f"invalid-credential-{name}" / "ConfigStore.db"
            credential_db.parent.mkdir(parents=True)
            stored = (
                "remembered-secret" if storage == "plaintext"
                else migration_module["DPAPI_PREFIX"] + "not-valid-base64"
                if storage == "damaged"
                else protect_sensitive_value(
                    "remembered-secret",
                    purpose_value(scope_type, scope_id, module, key),
                )
            )
            with sqlite3.connect(credential_db) as connection:
                migration_module["create_current_tables"](connection)
                migration_module["set_schema_meta"](
                    connection, True, authentication_initialized=False
                )
                connection.execute(
                    """
                    INSERT INTO settings(
                        scope_type, scope_id, module, key_name, value_text,
                        value_type, sensitive, encrypted
                    ) VALUES(?, ?, ?, ?, ?, 'string', 1, ?)
                    """,
                    (
                        scope_type, scope_id, module, key, stored,
                        0 if storage == "plaintext" else 1,
                    ),
                )
            credential_before = file_sha256(credential_db)
            credential_result = run_command(
                [str(exe), "--verify-current", "--db", str(credential_db)],
                temp,
            )
            if (
                credential_result.returncode == 0
                or file_sha256(credential_db) != credential_before
            ):
                raise AssertionError(
                    f"Invalid remembered credential was not rejected read-only: {name}"
                )

        compatible_current_db = temp / "current-without-created-at" / "ConfigStore.db"
        compatible_current_db.parent.mkdir(parents=True)
        with sqlite3.connect(compatible_current_db) as connection:
            migration_module["create_current_tables"](connection)
            migration_module["set_schema_meta"](
                connection, True, authentication_initialized=False
            )
            connection.execute("DELETE FROM meta WHERE key='created_at'")
            connection.executemany(
                """
                INSERT INTO settings(
                    scope_type, scope_id, module, key_name, value_text,
                    value_type, sensitive, encrypted
                ) VALUES('global', '', 'LoginState', ?, '0', 'bool', ?, 0)
                """,
                (("RememberPassword", 1), ("AutoLogin", 1)),
            )
        compatible_before = file_sha256(compatible_current_db)
        run_checked(
            [str(exe), "--verify-current", "--db", str(compatible_current_db)],
            temp,
        )
        if file_sha256(compatible_current_db) != compatible_before:
            raise AssertionError(
                "Read-only verification modified a compatible DB without created_at"
            )
        with sqlite3.connect(compatible_current_db) as connection:
            connection.execute(
                """
                UPDATE settings
                SET value_text='0', value_type='string', sensitive=1
                WHERE scope_type='global' AND scope_id=''
                  AND module='LoginState' AND key_name='RememberPassword'
                """
            )
            connection.execute(
                """
                UPDATE settings
                SET value_text='1', value_type='string', sensitive=0
                WHERE scope_type='global' AND scope_id=''
                  AND module='LoginState' AND key_name='AutoLogin'
                """
            )
        published_preference_before = file_sha256(compatible_current_db)
        run_checked(
            [str(exe), "--verify-current", "--db", str(compatible_current_db)],
            temp,
        )
        if file_sha256(compatible_current_db) != published_preference_before:
            raise AssertionError(
                "Read-only verification modified published LoginState shapes"
            )
        with sqlite3.connect(compatible_current_db) as connection:
            connection.execute(
                """
                UPDATE settings SET value_text='1'
                WHERE scope_type='global' AND scope_id=''
                  AND module='LoginState' AND key_name='RememberPassword'
                """
            )
        incompatible_preference_before = file_sha256(compatible_current_db)
        incompatible_preference_result = run_command(
            [str(exe), "--verify-current", "--db", str(compatible_current_db)],
            temp,
        )
        if (
            incompatible_preference_result.returncode == 0
            or file_sha256(compatible_current_db)
            != incompatible_preference_before
        ):
            raise AssertionError(
                "Nonzero plaintext login preference was not rejected read-only"
            )

        uppercase_legacy_db = temp / "current-uppercase-legacy-table" / "ConfigStore.db"
        uppercase_legacy_db.parent.mkdir(parents=True)
        with sqlite3.connect(uppercase_legacy_db) as connection:
            migration_module["create_current_tables"](connection)
            migration_module["set_schema_meta"](
                connection, True, authentication_initialized=False
            )
            connection.execute(
                "CREATE TABLE INI_VALUES(file_path TEXT, section_name TEXT, key_name TEXT, value_text TEXT)"
            )
        uppercase_before = file_sha256(uppercase_legacy_db)
        uppercase_result = run_command(
            [str(exe), "--verify-current", "--db", str(uppercase_legacy_db)],
            temp,
        )
        if (
            uppercase_result.returncode == 0
            or file_sha256(uppercase_legacy_db) != uppercase_before
        ):
            raise AssertionError(
                "Case-variant legacy table was not rejected read-only"
            )

        semantic_empty_db = temp / "semantic-v1-uninitialized-empty" / "ConfigStore.db"
        semantic_empty_db.parent.mkdir(parents=True)
        connection = sqlite3.connect(semantic_empty_db)
        try:
            create_settings_schema(connection)
            connection.executemany(
                "INSERT INTO meta(key, value) VALUES(?, ?)",
                (
                    ("schema_version", "5"),
                    ("auth_semantic_version", "1"),
                    ("auth_initialized", "0"),
                    ("encrypt_new_values", "1"),
                ),
            )
            connection.commit()
        finally:
            connection.close()
        run_checked(
            [
                str(exe), "--source", str(semantic_source),
                "--db", str(semantic_empty_db), "--encrypt",
            ],
            temp,
        )
        connection = sqlite3.connect(semantic_empty_db)
        try:
            empty_semantic_meta = dict(connection.execute(
                "SELECT key, value FROM meta WHERE key IN "
                "('auth_semantic_version', 'auth_initialized')"
            ).fetchall())
            empty_semantic_accounts = connection.execute(
                "SELECT COUNT(*) FROM settings WHERE scope_type='account' AND module='Profile'"
            ).fetchone()
        finally:
            connection.close()
        if empty_semantic_meta != {
            "auth_semantic_version": "2",
            "auth_initialized": "0",
        } or empty_semantic_accounts != (0,):
            raise AssertionError(
                "Uninitialized semantic upgrade did not remain account-empty"
            )

        semantic_in_place_data = temp / "semantic-v1-in-place" / "Data"
        semantic_in_place_data.mkdir(parents=True)
        semantic_in_place_db = semantic_in_place_data / "ConfigStore.db"
        semantic_in_place_ini = semantic_in_place_data / "Process.ini"
        semantic_in_place_ini.write_text(
            "[Runtime]\nPassCount=5\n", encoding="utf-8"
        )
        semantic_in_place_account = "semantic_in_place"
        semantic_in_place_hash = hashlib.sha256(
            f"{semantic_in_place_account}\nSynthetic".encode("utf-8")
        ).hexdigest()
        connection = sqlite3.connect(semantic_in_place_db)
        try:
            create_settings_schema(connection)
            connection.executemany(
                "INSERT INTO meta(key, value) VALUES(?, ?)",
                (
                    ("schema_version", "5"),
                    ("auth_semantic_version", "1"),
                    ("auth_initialized", "1"),
                    ("legacy_credential_scrub_state", "complete"),
                    ("legacy_credential_scrub_manifest", "[]"),
                ),
            )
            connection.executemany(
                """
                INSERT INTO settings(
                    scope_type, scope_id, module, key_name, value_text,
                    value_type, sensitive, encrypted
                ) VALUES('account', ?, 'Profile', ?, ?, 'string', ?, 0)
                """,
                (
                    (
                        semantic_in_place_account,
                        "PasswordHash",
                        semantic_in_place_hash,
                        1,
                    ),
                    (semantic_in_place_account, "Role", "admin", 0),
                ),
            )
            connection.commit()
        finally:
            connection.close()
        semantic_in_place_ini_hash = file_sha256(semantic_in_place_ini)
        run_checked(
            [
                str(exe), "--source", str(semantic_in_place_data),
                "--db", str(semantic_in_place_db), "--encrypt",
                "--scrub-legacy-credentials",
            ],
            temp,
        )
        connection = sqlite3.connect(semantic_in_place_db)
        try:
            preserved_provenance = dict(connection.execute(
                """
                SELECT key, value FROM meta WHERE key IN (
                    'auth_semantic_version', 'auth_initialized',
                    'legacy_credential_scrub_state',
                    'legacy_credential_scrub_manifest'
                )
                """
            ).fetchall())
        finally:
            connection.close()
        if preserved_provenance != {
            "auth_semantic_version": "2",
            "auth_initialized": "1",
            "legacy_credential_scrub_state": "complete",
            "legacy_credential_scrub_manifest": "[]",
        }:
            raise AssertionError(
                f"Semantic upgrade rewrote completed scrub provenance: {preserved_provenance}"
            )
        if file_sha256(semantic_in_place_ini) != semantic_in_place_ini_hash:
            raise AssertionError("Semantic upgrade changed a proven-clean legacy INI")

        connection = sqlite3.connect(semantic_in_place_db)
        try:
            connection.execute(
                "UPDATE meta SET value='1' WHERE key='auth_semantic_version'"
            )
            connection.commit()
        finally:
            connection.close()
        with semantic_in_place_ini.open("a", encoding="utf-8") as stream:
            stream.write(
                "[SavedPasswords]\nsemantic_in_place=U3ludGhldGljLVVucHJvdmVu\n"
            )
        post_complete_db_hash = file_sha256(semantic_in_place_db)
        post_complete_ini_hash = file_sha256(semantic_in_place_ini)
        post_complete_result = run_command(
            [
                str(exe), "--source", str(semantic_in_place_data),
                "--db", str(semantic_in_place_db), "--encrypt",
                "--scrub-legacy-credentials",
            ],
            temp,
        )
        if post_complete_result.returncode == 0 or "appeared after the proven migration scrub" not in (
            post_complete_result.stdout + post_complete_result.stderr
        ):
            raise AssertionError(
                "Semantic upgrade accepted credentials added after completed scrub"
            )
        if (
            file_sha256(semantic_in_place_db) != post_complete_db_hash
            or file_sha256(semantic_in_place_ini) != post_complete_ini_hash
        ):
            raise AssertionError(
                "Rejected post-complete semantic upgrade modified the database or source"
            )

        invalid_profile_cases = (
            ("password", "not-a-password-record", "admin", "0"),
            (
                "role",
                hashlib.sha256(b"invalid_role\nSynthetic").hexdigest(),
                "root",
                "0",
            ),
            (
                "must-change",
                hashlib.sha256(b"invalid_bool\nSynthetic").hexdigest(),
                "admin",
                "sometimes",
            ),
        )
        for case_name, password_record, role, must_change in invalid_profile_cases:
            invalid_db = temp / f"semantic-v1-invalid-{case_name}" / "ConfigStore.db"
            invalid_db.parent.mkdir(parents=True)
            account_id = f"invalid_{case_name.replace('-', '_')}"
            connection = sqlite3.connect(invalid_db)
            try:
                create_settings_schema(connection)
                connection.executemany(
                    "INSERT INTO meta(key, value) VALUES(?, ?)",
                    (
                        ("schema_version", "5"),
                        ("auth_semantic_version", "1"),
                        ("auth_initialized", "0"),
                    ),
                )
                connection.executemany(
                    """
                    INSERT INTO settings(
                        scope_type, scope_id, module, key_name, value_text,
                        value_type, sensitive, encrypted
                    ) VALUES('account', ?, 'Profile', ?, ?, ?, ?, 0)
                    """,
                    (
                        (account_id, "PasswordHash", password_record, "string", 1),
                        (account_id, "Role", role, "string", 0),
                        (account_id, "MustChangePassword", must_change, "bool", 1),
                    ),
                )
                connection.commit()
            finally:
                connection.close()
            invalid_before = file_sha256(invalid_db)
            invalid_result = run_command(
                [
                    str(exe), "--source", str(semantic_source),
                    "--db", str(invalid_db), "--encrypt",
                ],
                temp,
            )
            if invalid_result.returncode == 0:
                raise AssertionError(
                    f"Invalid existing account/Profile {case_name} was accepted"
                )
            if file_sha256(invalid_db) != invalid_before:
                raise AssertionError(
                    f"Rejected account/Profile {case_name} migration changed the database"
                )

        nested_text_data = temp / "nested-text-v4" / "Data"
        (nested_text_data / "nested" / "deeper").mkdir(parents=True)
        nested_text_db = nested_text_data / "ConfigStore.db"
        connection = sqlite3.connect(nested_text_db)
        try:
            create_settings_schema(connection)
            connection.execute(
                "INSERT INTO meta(key, value) VALUES('schema_version', '4')"
            )
            connection.commit()
        finally:
            connection.close()
        nested_weave = nested_text_data / "nested" / "WeaveDate.txt"
        nested_weld = nested_text_data / "nested" / "deeper" / "WELDPARA.TXT"
        nested_weave.write_bytes(b"Synthetic nested weave input\n")
        nested_weld.write_bytes(b"Synthetic nested weld input\n")
        nested_text_before = (
            file_sha256(nested_text_db),
            file_sha256(nested_weave),
            file_sha256(nested_weld),
        )
        nested_text_result = run_command(
            [
                str(exe), "--source", str(nested_text_data),
                "--db", str(nested_text_db), "--encrypt",
            ],
            temp,
        )
        if nested_text_result.returncode == 0 or "does not import legacy INI/TXT" not in (
            nested_text_result.stdout + nested_text_result.stderr
        ):
            raise AssertionError("Nested legacy WeaveDate/WeldPara inputs bypassed the v4 gate")
        if nested_text_before != (
            file_sha256(nested_text_db),
            file_sha256(nested_weave),
            file_sha256(nested_weld),
        ):
            raise AssertionError("Rejected nested legacy text migration was not byte-identical")

        existing_root = temp / "existing-v5-root"
        existing_db = existing_root / "Data" / "ConfigStore.db"
        empty_source = temp / "empty-source" / "Data"
        empty_source.mkdir(parents=True)
        run_checked(
            [str(exe), "--source", str(empty_source), "--db", str(existing_db), "--encrypt"],
            temp,
        )
        connection = sqlite3.connect(existing_db)
        try:
            empty_auth_initialized = connection.execute(
                "SELECT value FROM meta WHERE key='auth_initialized'"
            ).fetchone()
        finally:
            connection.close()
        if empty_auth_initialized != ("0",):
            raise AssertionError(
                f"Empty Data did not remain authentication-uninitialized: {empty_auth_initialized}"
            )
        existing_before = database_snapshot(existing_db, decrypt_value, purpose_value)

        refused = run_command(
            [str(exe), "--source", str(template), "--db", str(existing_db), "--encrypt"],
            temp,
        )
        if refused.returncode == 0 or "already exists" not in (refused.stdout + refused.stderr):
            raise AssertionError("Existing current-schema database was not rejected without --overwrite")
        if database_snapshot(existing_db, decrypt_value, purpose_value) != existing_before:
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
            raise AssertionError("Batch wrapper silently accepted an existing current-schema database")
        if database_snapshot(existing_db, decrypt_value, purpose_value) != existing_before:
            raise AssertionError("Batch refusal modified the existing current-schema database")

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
        overwritten = database_snapshot(existing_db, decrypt_value, purpose_value)
        if overwritten != python_snapshot:
            raise AssertionError("Explicit --overwrite did not rebuild the target from legacy Data")
        if not any(
            scope_id == "RobotA" and module.startswith("RobotPara")
            and key == "RobotType" and value == "2"
            for _, scope_id, module, key, value, *_ in overwritten[1]
        ):
            raise AssertionError("Explicit --overwrite did not import the legacy RobotPara values")
        protected_backups = list(existing_db.parent.glob("ConfigStore.db.bak_overwrite_*.dpapi.bak"))
        if len(protected_backups) != 1:
            raise AssertionError("Explicit --overwrite did not create exactly one DPAPI database backup")
        protected_backup = protected_backups[0]
        protected_raw = protected_backup.read_bytes()
        if b"SQLite format 3\x00" in protected_raw or b"Synthetic" in protected_raw:
            raise AssertionError("DPAPI database backup exposes a SQLite header or fixture secret")
        restored_db = temp / "restored-backup" / "ConfigStore.db"
        run_checked(
            [
                str(exe), "--restore-dpapi-backup", str(protected_backup),
                "--db", str(restored_db),
            ],
            temp,
        )
        if restored_db.read_bytes() != read_dpapi_backup(protected_backup):
            raise AssertionError("DPAPI database backup restore was not byte-identical to its verified payload")
        if list(existing_db.parent.glob(f".{existing_db.name}.staging-*.tmp")):
            raise AssertionError("Successful overwrite left a staging database behind")

        operator_only_source = temp / "operator-only-source" / "Data"
        operator_only_source.mkdir(parents=True)
        operator_only_account = "operator_only"
        operator_only_hash = hashlib.sha256(
            f"{operator_only_account}\nSynthetic".encode("utf-8")
        ).hexdigest()
        operator_only_accounts = operator_only_source / "Accounts.ini"
        operator_only_accounts.write_text(
            f"[Users/{operator_only_account}]\n"
            f"PasswordHash={operator_only_hash}\nRole=operator\n",
            encoding="utf-8",
        )
        operator_only_source_hash = file_sha256(operator_only_accounts)
        operator_only_new_db = temp / "operator-only-new-target" / "ConfigStore.db"
        operator_only_new_result = run_command(
            [
                str(exe), "--source", str(operator_only_source),
                "--db", str(operator_only_new_db), "--encrypt",
            ],
            temp,
        )
        if operator_only_new_result.returncode == 0 or "no valid administrator" not in (
            operator_only_new_result.stdout + operator_only_new_result.stderr
        ):
            raise AssertionError("A new operator-only account store was accepted")
        if (
            operator_only_new_db.exists()
            or file_sha256(operator_only_accounts) != operator_only_source_hash
            or list(operator_only_new_db.parent.glob(f".{operator_only_new_db.name}.staging-*.tmp"))
        ):
            raise AssertionError("Rejected new operator-only migration changed its source or target")

        atomic_root = temp / "atomic-overwrite"
        atomic_db = atomic_root / "Data" / "ConfigStore.db"
        run_checked(
            [str(exe), "--source", str(empty_source), "--db", str(atomic_db), "--encrypt"],
            temp,
        )
        atomic_before = file_sha256(atomic_db)
        operator_only_overwrite_result = run_command(
            [
                str(exe), "--source", str(operator_only_source),
                "--db", str(atomic_db), "--encrypt", "--overwrite",
            ],
            temp,
        )
        if operator_only_overwrite_result.returncode == 0 or "no valid administrator" not in (
            operator_only_overwrite_result.stdout + operator_only_overwrite_result.stderr
        ):
            raise AssertionError("An operator-only overwrite was accepted")
        if (
            file_sha256(atomic_db) != atomic_before
            or file_sha256(operator_only_accounts) != operator_only_source_hash
            or list(atomic_db.parent.glob(f".{atomic_db.name}.staging-*.tmp"))
        ):
            raise AssertionError("Rejected operator-only overwrite changed its source or live database")

        collision_source = temp / "collision-source" / "Data"
        collision_source.mkdir(parents=True)
        collision_hash_a = hashlib.sha256(b"Foo\nSynthetic-A").hexdigest()
        collision_hash_b = hashlib.sha256(b"foo\nSynthetic-B").hexdigest()
        (collision_source / "Accounts.ini").write_text(
            "[Users/Foo]\n"
            f"PasswordHash={collision_hash_a}\nRole=admin\n"
            "[Users/foo]\n"
            f"PasswordHash={collision_hash_b}\nRole=operator\n",
            encoding="utf-8",
        )
        atomic_failed = run_command(
            [
                str(exe), "--source", str(collision_source), "--db", str(atomic_db),
                "--encrypt", "--overwrite",
            ],
            temp,
        )
        if atomic_failed.returncode == 0:
            raise AssertionError("Case-colliding auth fixture unexpectedly replaced the live database")
        if file_sha256(atomic_db) != atomic_before:
            raise AssertionError("Failed overwrite changed the original database")
        if list(atomic_db.parent.glob(f".{atomic_db.name}.staging-*.tmp")):
            raise AssertionError("Failed overwrite left a staging database behind")

        corrupt_legacy_db = temp / "corrupt-legacy" / "ConfigStore.db"
        corrupt_legacy_db.parent.mkdir(parents=True)
        with sqlite3.connect(corrupt_legacy_db) as connection:
            connection.execute("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
            connection.execute("INSERT INTO meta(key, value) VALUES('schema_version', '4')")
            connection.execute(
                """
                CREATE TABLE ini_values(
                    file_path TEXT, section_name TEXT, key_name TEXT,
                    value_text TEXT, encrypted INTEGER
                )
                """
            )
            connection.execute(
                "INSERT INTO ini_values VALUES(?, ?, ?, ?, 1)",
                ("Data/RobotA/RobotPara.ini", "BaseParam", "FtpPassword", "enc:v1:bad:bad"),
            )
        corrupt_before = file_sha256(corrupt_legacy_db)
        corrupt_result = run_command(
            [str(exe), "--source", str(empty_source), "--db", str(corrupt_legacy_db), "--encrypt"],
            temp,
        )
        if corrupt_result.returncode == 0:
            raise AssertionError("Corrupt encrypted legacy row was accepted")
        if file_sha256(corrupt_legacy_db) != corrupt_before:
            raise AssertionError("Corrupt legacy migration was not fully rolled back")
        with sqlite3.connect(corrupt_legacy_db) as connection:
            if connection.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone() != ("4",):
                raise AssertionError("Corrupt legacy migration changed schema_version")
            if connection.execute("SELECT COUNT(*) FROM ini_values").fetchone() != (1,):
                raise AssertionError("Corrupt legacy migration dropped its source row")

        future_db = temp / "future-schema" / "ConfigStore.db"
        future_db.parent.mkdir(parents=True)
        with sqlite3.connect(future_db) as connection:
            connection.execute("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
            connection.execute("INSERT INTO meta(key, value) VALUES('schema_version', '6')")
            connection.execute(
                "CREATE TABLE ini_values(file_path TEXT, section_name TEXT, key_name TEXT, value_text TEXT)"
            )
        future_before = file_sha256(future_db)
        future_result = run_command(
            [str(exe), "--source", str(empty_source), "--db", str(future_db), "--encrypt"],
            temp,
        )
        if future_result.returncode == 0 or file_sha256(future_db) != future_before:
            raise AssertionError("Future schema with legacy-looking tables was not rejected byte-identically")

    print(
        "PASS: ConfigMigrate v5 parity, installer staging, DPAPI/auth migration, "
        "atomic overwrite/rollback, and future-schema gate"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
