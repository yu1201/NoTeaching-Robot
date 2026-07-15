#!/usr/bin/env python3
"""Focused dynamic coverage for safe ConfigStore v4 -> v5 upgrades."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import runpy
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import closing
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
MIGRATOR = runpy.run_path(
    str(REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_schema4_upgrade_tests",
)


def create_settings_schema(connection: sqlite3.Connection, version: str = "4") -> None:
    connection.execute("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
    connection.execute(
        """
        CREATE TABLE settings(
            scope_type TEXT NOT NULL, scope_id TEXT NOT NULL DEFAULT '',
            module TEXT NOT NULL, key_name TEXT NOT NULL,
            value_text TEXT NOT NULL, value_type TEXT NOT NULL DEFAULT 'string',
            sensitive INTEGER NOT NULL DEFAULT 0,
            encrypted INTEGER NOT NULL DEFAULT 0,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            PRIMARY KEY(scope_type, scope_id, module, key_name)
        )
        """
    )
    connection.execute(
        "INSERT INTO meta(key, value) VALUES('schema_version', ?)",
        (version,),
    )


def insert_setting(
    connection: sqlite3.Connection,
    scope_type: str,
    scope_id: str,
    module: str,
    key: str,
    value: str,
    *,
    value_type: str = "string",
    sensitive: int = 0,
    encrypted: int = 0,
) -> None:
    connection.execute(
        """
        INSERT INTO settings(
            scope_type, scope_id, module, key_name, value_text,
            value_type, sensitive, encrypted
        ) VALUES(?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            scope_type, scope_id, module, key, value,
            value_type, sensitive, encrypted,
        ),
    )


def password_record(user_name: str, marker: str) -> str:
    return hashlib.sha256(f"{user_name}\n{marker}".encode("utf-8")).hexdigest()


def database_snapshot(path: Path) -> tuple[tuple[object, ...], tuple[object, ...]]:
    with closing(sqlite3.connect(path)) as connection, connection:
        meta = tuple(connection.execute("SELECT key, value FROM meta ORDER BY key"))
        settings = tuple(connection.execute(
            """
            SELECT scope_type, scope_id, module, key_name, value_text,
                   value_type, sensitive, encrypted, updated_at
            FROM settings
            ORDER BY scope_type, scope_id, module, key_name
            """
        ))
    return meta, settings


def add_legacy_account(
    connection: sqlite3.Connection,
    user_name: str,
    role: str,
    record: str,
    *,
    flat: bool,
    protect: bool,
    created_at: str | None = "2026-01-02T03:04:05",
    updated_at: str | None = None,
) -> None:
    module = "Accounts/Users" if flat else f"Accounts/Users/{user_name}"
    fields = {
        "PasswordHash": record,
        "Role": role,
    }
    if created_at is not None:
        fields["CreatedAt"] = created_at
    if updated_at is not None:
        fields["UpdatedAt"] = updated_at
    for field, plain in fields.items():
        key = f"{user_name}/{field}" if flat else field
        stored = plain
        encrypted = 0
        if protect and field == "PasswordHash":
            stored = MIGRATOR["protect_sensitive_text"](
                plain,
                MIGRATOR["protection_purpose"]("global", "", module, key),
            )
            encrypted = 1
        elif protect and field == "Role":
            stored = MIGRATOR["protect_legacy_text"](plain)
            encrypted = 1
        insert_setting(
            connection, "global", "", module, key, stored,
            sensitive=1 if field == "PasswordHash" else 0,
            encrypted=encrypted,
            value_type="datetime" if field.endswith("At") else "string",
        )


class StrictLegacyAccountTests(unittest.TestCase):
    def _connection(self) -> sqlite3.Connection:
        connection = sqlite3.connect(":memory:")
        create_settings_schema(connection)
        connection.commit()
        return connection

    def test_rejects_conflicting_duplicate_layouts(self) -> None:
        connection = self._connection()
        try:
            record = password_record("admin", "one")
            add_legacy_account(
                connection, "admin", "admin", record, flat=True, protect=False
            )
            add_legacy_account(
                connection, "admin", "operator", record, flat=False, protect=False
            )
            with self.assertRaisesRegex(ValueError, "Conflicting duplicate"):
                MIGRATOR["migrate_authentication_semantics"](connection, True)
        finally:
            connection.close()

    def test_rejects_case_collision_and_incomplete_fields(self) -> None:
        cases = ("case", "incomplete", "field-case")
        for case in cases:
            with self.subTest(case=case):
                connection = self._connection()
                try:
                    if case == "case":
                        add_legacy_account(
                            connection, "Admin", "admin",
                            password_record("Admin", "one"),
                            flat=True, protect=False,
                        )
                        add_legacy_account(
                            connection, "admin", "admin",
                            password_record("admin", "two"),
                            flat=False, protect=False,
                        )
                    elif case == "incomplete":
                        insert_setting(
                            connection, "global", "", "Accounts/Users",
                            "admin/PasswordHash", password_record("admin", "one"),
                            sensitive=1,
                        )
                    else:
                        insert_setting(
                            connection, "global", "", "Accounts/Users",
                            "admin/passwordhash", password_record("admin", "one"),
                            sensitive=1,
                        )
                        insert_setting(
                            connection, "global", "", "Accounts/Users",
                            "admin/Role", "admin",
                        )
                    with self.assertRaises(ValueError):
                        MIGRATOR["migrate_authentication_semantics"](
                            connection, True
                        )
                finally:
                    connection.close()

    def test_rejects_case_variant_legacy_modules(self) -> None:
        fixtures = (
            (
                (
                    ("accounts/users", "admin/PasswordHash", password_record("admin", "one"), 1),
                    ("accounts/users", "admin/Role", "admin", 0),
                ),
                "authentication module",
            ),
            (
                (("loginstATE/general", "UserName", "admin", 0),),
                "authentication module",
            ),
            (
                (("LoginState/savedpasswords", "admin", "legacy", 1),),
                "authentication module",
            ),
        )
        for rows, expected in fixtures:
            with self.subTest(module=rows[0][0]):
                connection = self._connection()
                try:
                    for module, key, value, sensitive in rows:
                        insert_setting(
                            connection, "global", "", module, key, value,
                            sensitive=sensitive,
                        )
                    with self.assertRaisesRegex(ValueError, expected):
                        MIGRATOR["migrate_authentication_semantics"](
                            connection, True
                        )
                finally:
                    connection.close()

    def test_timestamp_metadata_merges_across_field_shapes(self) -> None:
        fixtures = (
            {
                "name": "flat-vs-nested-created",
                "flat_created": "2026-05-07T00:00:00",
                "nested_created": "2026-05-28T00:00:00",
                "target_created": None,
                "flat_updated": None,
                "nested_updated": None,
                "target_updated": None,
                "expected_created": "2026-05-07T00:00:00",
                "expected_updated": None,
            },
            {
                "name": "legacy-vs-target-created",
                "flat_created": "2026-05-28T00:00:00",
                "nested_created": "2026-05-28T00:00:00",
                "target_created": "2026-05-07T00:00:00",
                "flat_updated": None,
                "nested_updated": None,
                "target_updated": None,
                "expected_created": "2026-05-07T00:00:00",
                "expected_updated": None,
            },
            {
                "name": "flat-nested-target-updated",
                "flat_created": "2026-05-07T00:00:00",
                "nested_created": "2026-05-28T00:00:00",
                "target_created": "2026-06-05T00:00:00",
                "flat_updated": "2026-05-15T00:00:00",
                "nested_updated": "2026-05-20T00:00:00",
                "target_updated": "2026-06-01T00:00:00",
                "expected_created": "2026-05-07T00:00:00",
                "expected_updated": "2026-06-01T00:00:00",
            },
        )
        for fixture in fixtures:
            with self.subTest(shape=fixture["name"]):
                connection = self._connection()
                try:
                    record = password_record("admin", "matching-security")
                    add_legacy_account(
                        connection, "admin", "admin", record,
                        flat=True, protect=False,
                        created_at=fixture["flat_created"],
                        updated_at=fixture["flat_updated"],
                    )
                    add_legacy_account(
                        connection, "admin", "admin", record,
                        flat=False, protect=False,
                        created_at=fixture["nested_created"],
                        updated_at=fixture["nested_updated"],
                    )
                    if fixture["target_created"] is not None:
                        for key, value, value_type in (
                            ("PasswordHash", record, "string"),
                            ("Role", "admin", "string"),
                            ("CreatedAt", fixture["target_created"], "datetime"),
                            ("MustChangePassword", "0", "bool"),
                        ):
                            insert_setting(
                                connection, "account", "admin", "Profile",
                                key, str(value), value_type=value_type,
                                sensitive=1 if key in {
                                    "PasswordHash", "MustChangePassword"
                                } else 0,
                            )
                        if fixture["target_updated"] is not None:
                            insert_setting(
                                connection, "account", "admin", "Profile",
                                "UpdatedAt", str(fixture["target_updated"]),
                                value_type="datetime",
                            )

                    MIGRATOR["migrate_authentication_semantics"](
                        connection, False
                    )
                    self.assertEqual(
                        MIGRATOR["_decoded_scoped_value"](
                            connection, "account", "admin", "Profile",
                            "CreatedAt",
                        ),
                        fixture["expected_created"],
                    )
                    self.assertEqual(
                        MIGRATOR["_decoded_scoped_value"](
                            connection, "account", "admin", "Profile",
                            "UpdatedAt",
                        ),
                        fixture["expected_updated"],
                    )
                finally:
                    connection.close()

    def test_migration_profile_reads_reject_wrapped_portable_fields(self) -> None:
        wrappers = ["enc:v1"]
        if os.name == "nt":
            wrappers.append("dpapi:user:v1")
        fields = (
            "PasswordHash", "Role", "MustChangePassword",
            "PasswordChangedAt", "CreatedAt", "UpdatedAt",
        )
        for context in ("existing", "target"):
            for wrapper in wrappers:
                for wrapped_field in fields:
                    with self.subTest(
                        context=context, wrapper=wrapper, field=wrapped_field
                    ):
                        connection = self._connection()
                        try:
                            values = {
                                "PasswordHash": password_record(
                                    "admin", "portable-profile"
                                ),
                                "Role": "admin",
                                "MustChangePassword": "0",
                                "PasswordChangedAt": "2026-05-28T12:34:56+08:00",
                                "CreatedAt": "2026-05-07T00:00:00",
                                "UpdatedAt": "2026-06-01T00:00:00Z",
                            }
                            if context == "target":
                                add_legacy_account(
                                    connection, "admin", "admin",
                                    values["PasswordHash"], flat=True,
                                    protect=False,
                                    created_at=values["CreatedAt"],
                                    updated_at=values["UpdatedAt"],
                                )
                            for key, plain in values.items():
                                stored = plain
                                if key == wrapped_field:
                                    if wrapper == "enc:v1":
                                        stored = MIGRATOR["protect_legacy_text"](plain)
                                    else:
                                        stored = MIGRATOR["protect_sensitive_text"](
                                            plain,
                                            MIGRATOR["protection_purpose"](
                                                "account", "admin", "Profile", key
                                            ),
                                        )
                                insert_setting(
                                    connection, "account", "admin", "Profile",
                                    key, stored,
                                    value_type=(
                                        "bool" if key == "MustChangePassword"
                                        else "datetime" if key.endswith("At")
                                        else "string"
                                    ),
                                    sensitive=1 if "Password" in key else 0,
                                    encrypted=1 if key == wrapped_field else 0,
                                )
                            with self.assertRaisesRegex(
                                ValueError, "portable plaintext storage"
                            ):
                                MIGRATOR["migrate_authentication_semantics"](
                                    connection, True
                                )
                        finally:
                            connection.close()


class RuntimeIniGateTests(unittest.TestCase):
    def _v4_database(self, root: Path) -> Path:
        data = root / "Data"
        data.mkdir(parents=True)
        db = data / "ConfigStore.db"
        with closing(sqlite3.connect(db)) as connection, connection:
            create_settings_schema(connection)
        return db

    def test_only_exact_root_runtime_shape_is_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            valid = data / "CorrugatedSheetPointCloudEctration.ini"
            valid.write_bytes(
                b"// runtime input\nifupright=true\nSave_File_Name=C:\\Result\\out.txt\n"
            )
            self.assertTrue(MIGRATOR["_is_runtime_only_ini"](valid, data))

            invalid_contents = {
                "unknown": b"ifupright=true\nUnknownKey=1\n",
                "section": b"[General]\nifupright=true\n",
                "duplicate": b"ifupright=true\nifupright=false\n",
                "case-duplicate": b"ifupright=true\nIFUPRIGHT=false\n",
                "credential": b"ifupright=true\nApiToken=must-not-bypass\n",
                "malformed": b"ifupright=true\nnot-an-assignment\n",
            }
            for case, content in invalid_contents.items():
                with self.subTest(case=case):
                    valid.write_bytes(content)
                    self.assertFalse(MIGRATOR["_is_runtime_only_ini"](valid, data))

            nested = data / "nested" / valid.name
            nested.parent.mkdir()
            nested.write_bytes(b"ifupright=true\n")
            self.assertFalse(MIGRATOR["_is_runtime_only_ini"](nested, data))
            valid.unlink(missing_ok=True)
            symlink_target = data / "runtime-target.ini"
            symlink_target.write_bytes(b"ifupright=true\n")
            try:
                valid.symlink_to(symlink_target)
            except OSError:
                pass
            else:
                self.assertFalse(MIGRATOR["_is_runtime_only_ini"](valid, data))
                valid.unlink()
            valid.write_bytes(b"ifupright=true\n" + b" " * (128 * 1024))
            self.assertFalse(MIGRATOR["_is_runtime_only_ini"](valid, data))

    def test_legacy_ini_and_invalid_runtime_ini_still_block(self) -> None:
        fixtures = (
            (Path("Process.ini"), "[Runtime]\nPassCount=5\n"),
            (
                Path("CorrugatedSheetPointCloudEctration.ini"),
                "[Network]\nApiToken=must-not-bypass\n",
            ),
            (
                Path("CorrugatedSheetPointCloudEctration.ini"),
                "ifupright=true\nUnknownKey=1\n",
            ),
            (
                Path("CorrugatedSheetPointCloudEctration.ini"),
                "[General]\nifupright=true\n",
            ),
            (
                Path("CorrugatedSheetPointCloudEctration.ini"),
                "ifupright=true\nifupright=false\n",
            ),
            (
                Path("nested") / "CorrugatedSheetPointCloudEctration.ini",
                "ifupright=true\n",
            ),
        )
        for relative, content in fixtures:
            with self.subTest(path=relative), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                db = self._v4_database(root)
                candidate = db.parent / relative
                candidate.parent.mkdir(parents=True, exist_ok=True)
                candidate.write_text(content, encoding="utf-8")
                before = database_snapshot(db)
                with self.assertRaisesRegex(SystemExit, "does not import legacy"):
                    MIGRATOR["migrate_existing_database_to_current"](
                        db, db.parent, True
                    )
                self.assertEqual(before, database_snapshot(db))

    def test_unneeded_upgrade_does_not_create_requested_backup(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            backup = data / "ConfigStore.db.install-upgrade-unused.dpapi.bak"
            with closing(sqlite3.connect(db)) as connection, connection:
                create_settings_schema(connection, "5")
                connection.executemany(
                    "INSERT INTO meta(key, value) VALUES(?, ?)",
                    (("auth_semantic_version", "2"), ("auth_initialized", "0")),
                )
            changed = MIGRATOR["migrate_existing_database_to_current"](
                db, data, True, upgrade_backup_path=backup
            )
            self.assertFalse(changed)
            self.assertFalse(backup.exists())


class ReadOnlyCurrentVerificationTests(unittest.TestCase):
    def _current_database(self, root: Path) -> Path:
        data = root / "Data"
        data.mkdir()
        db = data / "ConfigStore.db"
        with closing(sqlite3.connect(db)) as connection, connection:
            MIGRATOR["create_current_tables"](connection)
            MIGRATOR["set_schema_meta"](
                connection, True, authentication_initialized=False
            )
        return db

    def _current_authenticated_database(self, root: Path) -> tuple[Path, dict[str, str]]:
        db = self._current_database(root)
        values = {
            "PasswordHash": password_record("admin", "portable-profile"),
            "Role": "admin",
            "MustChangePassword": "0",
            "CreatedAt": "2026-05-07T00:00:00",
            "UpdatedAt": "2026-06-01T00:00:00Z",
            "PasswordChangedAt": "2026-05-28T12:34:56+08:00",
        }
        with closing(sqlite3.connect(db)) as connection, connection:
            connection.execute(
                "UPDATE meta SET value='1' WHERE key='auth_initialized'"
            )
            for key, value in values.items():
                insert_setting(
                    connection, "account", "admin", "Profile", key, value,
                    value_type=(
                        "bool" if key == "MustChangePassword"
                        else "datetime" if key.endswith("At")
                        else "string"
                    ),
                    sensitive=1 if "Password" in key else 0,
                )
        return db, values

    def test_cli_verify_is_read_only_and_rejects_migration_flags(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            db = self._current_database(Path(text))
            before = hashlib.sha256(db.read_bytes()).hexdigest()
            command = [
                sys.executable,
                str(REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"),
                "--verify-current",
                "--db",
                str(db),
            ]
            result = subprocess.run(
                command,
                cwd=REPO_ROOT,
                text=True,
                encoding="utf-8",
                errors="replace",
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("Verified current ConfigStore schema v5", result.stdout)
            self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

            rejected = subprocess.run(
                command + ["--encrypt"],
                cwd=REPO_ROOT,
                text=True,
                encoding="utf-8",
                errors="replace",
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(rejected.returncode, 0)
            self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute("DELETE FROM meta WHERE key='created_at'")
            without_created_at = hashlib.sha256(db.read_bytes()).hexdigest()
            compatible = subprocess.run(
                command,
                cwd=REPO_ROOT,
                text=True,
                encoding="utf-8",
                errors="replace",
                capture_output=True,
                check=False,
            )
            self.assertEqual(
                compatible.returncode, 0,
                compatible.stdout + compatible.stderr,
            )
            self.assertEqual(
                hashlib.sha256(db.read_bytes()).hexdigest(), without_created_at
            )

    @unittest.skipUnless(os.name == "nt", "Installer verification requires Windows")
    def test_current_and_installer_verifiers_require_exact_primary_keys(self) -> None:
        variants = (
            ("settings-no-pk", False),
            ("settings-no-pk-duplicates", True),
            ("meta-no-pk", False),
            ("meta-no-pk-duplicates", True),
        )
        for variant, add_duplicate in variants:
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                db = self._current_database(root)
                data = db.parent
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.executemany(
                        "INSERT INTO meta(key, value) VALUES(?, ?)",
                        (
                            (MIGRATOR["SCRUB_STATE_KEY"], "complete"),
                            (
                                MIGRATOR["SCRUB_MANIFEST_KEY"],
                                MIGRATOR[
                                    "serialize_legacy_credential_scrub_manifest"
                                ]([]),
                            ),
                        ),
                    )
                    if variant.startswith("settings"):
                        connection.execute(
                            "ALTER TABLE settings RENAME TO settings_original"
                        )
                        connection.execute(
                            """
                            CREATE TABLE settings(
                                scope_type TEXT NOT NULL,
                                scope_id TEXT NOT NULL DEFAULT '',
                                module TEXT NOT NULL,
                                key_name TEXT NOT NULL,
                                value_text TEXT NOT NULL,
                                value_type TEXT NOT NULL DEFAULT 'string',
                                sensitive INTEGER NOT NULL DEFAULT 0,
                                encrypted INTEGER NOT NULL DEFAULT 0,
                                updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                            )
                            """
                        )
                        connection.execute(
                            "INSERT INTO settings SELECT * FROM settings_original"
                        )
                        connection.execute("DROP TABLE settings_original")
                        connection.execute(
                            """
                            INSERT INTO settings(
                                scope_type, scope_id, module, key_name, value_text
                            ) VALUES('global', '', 'Motion', 'Speed', '1')
                            """
                        )
                        if add_duplicate:
                            connection.execute(
                                """
                                INSERT INTO settings(
                                    scope_type, scope_id, module, key_name, value_text
                                ) VALUES('global', '', 'Motion', 'Speed', '999')
                                """
                            )
                    else:
                        connection.execute(
                            "ALTER TABLE meta RENAME TO meta_original"
                        )
                        connection.execute(
                            "CREATE TABLE meta(key TEXT, value TEXT NOT NULL)"
                        )
                        connection.execute(
                            "INSERT INTO meta SELECT * FROM meta_original"
                        )
                        connection.execute("DROP TABLE meta_original")
                        if add_duplicate:
                            connection.execute(
                                "INSERT INTO meta(key, value) VALUES('created_at', 'duplicate')"
                            )

                before = hashlib.sha256(db.read_bytes()).hexdigest()
                with self.assertRaisesRegex(ValueError, "schema is incompatible"):
                    MIGRATOR["verify_current_database"](db)
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)
                with self.assertRaisesRegex(ValueError, "schema is incompatible"):
                    MIGRATOR["verify_installer_state"](data, db)
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

    @unittest.skipUnless(os.name == "nt", "Installer verification requires Windows")
    def test_verifiers_reject_schema_triggers_unique_indexes_and_nocase_pk(self) -> None:
        for variant in (
            "trigger",
            "auxiliary-trigger",
            "extra-unique",
            "expression-index",
            "check-constraint",
            "nocase-primary-key",
        ):
            with self.subTest(variant=variant), tempfile.TemporaryDirectory() as text:
                db = self._current_database(Path(text))
                data = db.parent
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.executemany(
                        "INSERT INTO meta(key, value) VALUES(?, ?)",
                        (
                            (MIGRATOR["SCRUB_STATE_KEY"], "complete"),
                            (
                                MIGRATOR["SCRUB_MANIFEST_KEY"],
                                MIGRATOR[
                                    "serialize_legacy_credential_scrub_manifest"
                                ]([]),
                            ),
                        ),
                    )
                    if variant == "trigger":
                        connection.execute(
                            """
                            CREATE TRIGGER destructive_settings_insert
                            AFTER INSERT ON settings
                            BEGIN
                                DELETE FROM settings;
                            END
                            """
                        )
                    elif variant == "auxiliary-trigger":
                        connection.execute(
                            "CREATE TABLE auxiliary(value TEXT)"
                        )
                        connection.execute(
                            """
                            CREATE TRIGGER auxiliary_changes_settings
                            AFTER INSERT ON auxiliary
                            BEGIN
                                DELETE FROM settings;
                                UPDATE meta SET value='0'
                                WHERE key='auth_initialized';
                            END
                            """
                        )
                    elif variant == "extra-unique":
                        connection.execute(
                            "CREATE UNIQUE INDEX unexpected_module_unique ON settings(module)"
                        )
                    elif variant == "expression-index":
                        connection.execute("DROP INDEX idx_settings_scope")
                        connection.execute(
                            "CREATE INDEX idx_settings_scope ON settings("
                            "scope_type, scope_id, module, lower(value_text))"
                        )
                    elif variant == "check-constraint":
                        connection.execute(
                            "ALTER TABLE settings RENAME TO settings_original"
                        )
                        connection.execute(
                            """
                            CREATE TABLE settings(
                                scope_type TEXT NOT NULL,
                                scope_id TEXT NOT NULL DEFAULT '',
                                module TEXT NOT NULL,
                                key_name TEXT NOT NULL,
                                value_text TEXT NOT NULL CHECK(value_text<>'forbidden'),
                                value_type TEXT NOT NULL DEFAULT 'string',
                                sensitive INTEGER NOT NULL DEFAULT 0,
                                encrypted INTEGER NOT NULL DEFAULT 0,
                                updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                                PRIMARY KEY(scope_type, scope_id, module, key_name)
                            )
                            """
                        )
                        connection.execute(
                            "INSERT INTO settings SELECT * FROM settings_original"
                        )
                        connection.execute("DROP TABLE settings_original")
                        connection.execute(
                            "CREATE INDEX idx_settings_scope "
                            "ON settings(scope_type, scope_id, module)"
                        )
                    else:
                        connection.execute(
                            "ALTER TABLE settings RENAME TO settings_original"
                        )
                        connection.execute(
                            """
                            CREATE TABLE settings(
                                scope_type TEXT COLLATE NOCASE NOT NULL,
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
                        connection.execute(
                            "INSERT INTO settings SELECT * FROM settings_original"
                        )
                        connection.execute("DROP TABLE settings_original")
                        connection.execute(
                            "CREATE INDEX idx_settings_scope "
                            "ON settings(scope_type, scope_id, module)"
                        )

                before = hashlib.sha256(db.read_bytes()).hexdigest()
                with self.assertRaisesRegex(ValueError, "schema is incompatible"):
                    MIGRATOR["verify_current_database"](db)
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)
                with self.assertRaisesRegex(ValueError, "schema is incompatible"):
                    MIGRATOR["verify_installer_state"](data, db)
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

    def test_verify_rejects_orphans_and_incomplete_metadata(self) -> None:
        cases = (
            "auth-row", "auth-row-case", "login-row-case", "legacy-table",
            "legacy-table-case", "missing-meta", "blank-created-at",
            "plaintext-secret", "profile-module-case",
            "profile-scope-case", "profile-scope-id",
            "nonglobal-accounts", "nonglobal-general",
            "nonglobal-saved-passwords", "nonglobal-remembered-credentials",
            "nonglobal-passwordbase64",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                db = self._current_database(Path(text))
                with closing(sqlite3.connect(db)) as connection, connection:
                    if case in {"auth-row", "auth-row-case"}:
                        insert_setting(
                            connection, "global", "",
                            "Accounts/Users" if case == "auth-row" else "accounts/users",
                            "admin/Role", "admin",
                        )
                    elif case == "login-row-case":
                        insert_setting(
                            connection, "global", "", "loginstate/general",
                            "UserName", "admin",
                        )
                    elif case in {"legacy-table", "legacy-table-case"}:
                        connection.execute(
                            "CREATE TABLE "
                            + ("INI_VALUES" if case == "legacy-table-case" else "ini_values")
                            + "(file_path TEXT, section_name TEXT, key_name TEXT, value_text TEXT)"
                        )
                    elif case == "missing-meta":
                        connection.execute(
                            "DELETE FROM meta WHERE key='sensitive_protection'"
                        )
                    elif case == "blank-created-at":
                        connection.execute(
                            "UPDATE meta SET value='' WHERE key='created_at'"
                        )
                    elif case in {
                        "profile-module-case", "profile-scope-case",
                        "profile-scope-id",
                    }:
                        insert_setting(
                            connection,
                            "Account" if case == "profile-scope-case" else "account",
                            " admin " if case == "profile-scope-id" else "shadow",
                            "profile" if case == "profile-module-case" else "Profile",
                            "Role", "admin",
                        )
                    elif case.startswith("nonglobal-"):
                        module, key, value = {
                            "nonglobal-accounts": (
                                "Accounts/Users/shadow", "Role", "operator"
                            ),
                            "nonglobal-general": (
                                "LoginState/General", "UserName", "shadow"
                            ),
                            "nonglobal-saved-passwords": (
                                "LoginState/SavedPasswords/shadow",
                                "shadow", "reversible",
                            ),
                            "nonglobal-remembered-credentials": (
                                "LoginState/RememberedCredentials/shadow",
                                "shadow", "reversible",
                            ),
                            "nonglobal-passwordbase64": (
                                "Unrelated", "PasswordBase64", "reversible"
                            ),
                        }[case]
                        insert_setting(
                            connection, "robot", "RobotA", module, key, value
                        )
                    else:
                        insert_setting(
                            connection, "global", "", "RuntimeSecrets",
                            "ApiToken", "plaintext", sensitive=1,
                        )
                before = hashlib.sha256(db.read_bytes()).hexdigest()
                with self.assertRaises(ValueError):
                    MIGRATOR["verify_current_database"](db)
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

    def test_wrapped_portable_profile_fields_are_rejected_read_only(self) -> None:
        wrappers = ["enc:v1"]
        if os.name == "nt":
            wrappers.append("dpapi:user:v1")
        fields = (
            "PasswordHash", "Role", "MustChangePassword",
            "PasswordChangedAt", "CreatedAt", "UpdatedAt",
        )
        for wrapper in wrappers:
            for field in fields:
                with (
                    self.subTest(wrapper=wrapper, field=field),
                    tempfile.TemporaryDirectory() as text,
                ):
                    db, values = self._current_authenticated_database(Path(text))
                    if wrapper == "enc:v1":
                        wrapped = MIGRATOR["protect_legacy_text"](values[field])
                    else:
                        wrapped = MIGRATOR["protect_sensitive_text"](
                            values[field],
                            MIGRATOR["protection_purpose"](
                                "account", "admin", "Profile", field
                            ),
                        )
                    with closing(sqlite3.connect(db)) as connection, connection:
                        connection.execute(
                            """
                            UPDATE settings SET value_text=?, encrypted=0
                            WHERE scope_type='account' AND scope_id='admin'
                              AND module='Profile' AND key_name=?
                            """,
                            (wrapped, field),
                        )
                    before = hashlib.sha256(db.read_bytes()).hexdigest()
                    with self.assertRaisesRegex(
                        ValueError, "portable plaintext storage"
                    ):
                        MIGRATOR["verify_current_database"](db)
                    self.assertEqual(
                        hashlib.sha256(db.read_bytes()).hexdigest(), before
                    )

    @unittest.skipUnless(os.name == "nt", "CurrentUser DPAPI tests require Windows")
    def test_canonical_remembered_credentials_are_validated_read_only(self) -> None:
        canonical_modules = (
            "LoginState/RememberedCredentials",
            "LoginState/SavedPasswords",
        )
        for module in canonical_modules:
            with self.subTest(valid_module=module), tempfile.TemporaryDirectory() as text:
                db = self._current_database(Path(text))
                protected = MIGRATOR["protect_sensitive_text"](
                    "remembered-secret",
                    MIGRATOR["protection_purpose"](
                        "global", "", module, "admin"
                    ),
                )
                with closing(sqlite3.connect(db)) as connection, connection:
                    insert_setting(
                        connection, "global", "", module, "admin", protected,
                        sensitive=1, encrypted=1,
                    )
                before = hashlib.sha256(db.read_bytes()).hexdigest()
                self.assertEqual(MIGRATOR["verify_current_database"](db), "none")
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

        invalid_shapes = (
            ("wrong-scope", "robot", "RobotA", canonical_modules[0], "admin", "string", 1, 1, "protected"),
            ("wrong-case", "global", "", "loginstate/rememberedcredentials", "admin", "string", 1, 1, "protected"),
            ("nested", "global", "", canonical_modules[0] + "/admin", "admin", "string", 1, 1, "protected"),
            ("plaintext", "global", "", canonical_modules[0], "admin", "string", 1, 0, "plaintext"),
            ("damaged", "global", "", canonical_modules[0], "admin", "string", 1, 1, "damaged"),
            ("unsafe-key", "global", "", canonical_modules[0], " admin ", "string", 1, 1, "protected"),
            ("sensitive-marker", "global", "", canonical_modules[0], "admin", "string", 0, 1, "protected"),
            ("encrypted-marker", "global", "", canonical_modules[0], "admin", "string", 1, 0, "protected"),
            ("value-type", "global", "", canonical_modules[0], "admin", "text", 1, 1, "protected"),
        )
        for (
            name, scope_type, scope_id, module, key, value_type,
            sensitive, encrypted, storage,
        ) in invalid_shapes:
            with self.subTest(invalid=name), tempfile.TemporaryDirectory() as text:
                db = self._current_database(Path(text))
                if storage == "plaintext":
                    stored = "remembered-secret"
                elif storage == "damaged":
                    stored = MIGRATOR["DPAPI_PREFIX"] + "not-valid-base64"
                else:
                    stored = MIGRATOR["protect_sensitive_text"](
                        "remembered-secret",
                        MIGRATOR["protection_purpose"](
                            scope_type, scope_id, module, key
                        ),
                    )
                with closing(sqlite3.connect(db)) as connection, connection:
                    insert_setting(
                        connection, scope_type, scope_id, module, key, stored,
                        value_type=value_type, sensitive=sensitive,
                        encrypted=encrypted,
                    )
                before = hashlib.sha256(db.read_bytes()).hexdigest()
                with self.assertRaises(ValueError):
                    MIGRATOR["verify_current_database"](db)
                self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

    def test_plaintext_disabled_login_preferences_are_compatible_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            db = self._current_database(Path(text))
            with closing(sqlite3.connect(db)) as connection, connection:
                insert_setting(
                    connection, "global", "", "LoginState",
                    "RememberPassword", "0", value_type="bool",
                    sensitive=1, encrypted=0,
                )
                insert_setting(
                    connection, "global", "", "LoginState",
                    "AutoLogin", "0", value_type="bool",
                    sensitive=1, encrypted=0,
                )
            before = hashlib.sha256(db.read_bytes()).hexdigest()
            self.assertEqual(MIGRATOR["verify_current_database"](db), "none")
            self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before)

        published_shapes = (
            ("AutoLogin", "0", "string", 0),
            ("AutoLogin", "1", "string", 0),
            ("RememberPassword", "0", "string", 1),
        )
        for key, value, value_type, sensitive in published_shapes:
            with (
                self.subTest(
                    published_key=key, value=value,
                    value_type=value_type, sensitive=sensitive,
                ),
                tempfile.TemporaryDirectory() as text,
            ):
                db = self._current_database(Path(text))
                with closing(sqlite3.connect(db)) as connection, connection:
                    insert_setting(
                        connection, "global", "", "LoginState", key, value,
                        value_type=value_type, sensitive=sensitive,
                        encrypted=0,
                    )
                before = hashlib.sha256(db.read_bytes()).hexdigest()
                self.assertEqual(
                    MIGRATOR["verify_current_database"](db), "none"
                )
                self.assertEqual(
                    hashlib.sha256(db.read_bytes()).hexdigest(), before
                )

        invalid_shapes = (
            ("RememberPassword", "1", "bool", 1, 0),
            ("RememberPassword", "1", "string", 1, 0),
            ("RememberPassword", "0", "string", 0, 0),
            ("AutoLogin", "false", "bool", 1, 0),
            ("AutoLogin", "2", "string", 0, 0),
            ("AutoLogin", "0", "string", 1, 0),
            ("AutoLogin", "0", "bool", 0, 0),
            ("AutoLogin", "0", "bool", 1, 1),
        )
        for key, value, value_type, sensitive, encrypted in invalid_shapes:
            with (
                self.subTest(
                    key=key, value=value, value_type=value_type,
                    sensitive=sensitive, encrypted=encrypted,
                ),
                tempfile.TemporaryDirectory() as text,
            ):
                db = self._current_database(Path(text))
                with closing(sqlite3.connect(db)) as connection, connection:
                    insert_setting(
                        connection, "global", "", "LoginState", key, value,
                        value_type=value_type, sensitive=sensitive,
                        encrypted=encrypted,
                    )
                before = hashlib.sha256(db.read_bytes()).hexdigest()
                with self.assertRaises(ValueError):
                    MIGRATOR["verify_current_database"](db)
                self.assertEqual(
                    hashlib.sha256(db.read_bytes()).hexdigest(), before
                )

    def test_pending_manifest_is_parsed_but_does_not_allow_db_orphans(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            db = self._current_database(Path(text))
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.executemany(
                    "INSERT INTO meta(key, value) VALUES(?, ?)",
                    (
                        (MIGRATOR["SCRUB_STATE_KEY"], "pending"),
                        (MIGRATOR["SCRUB_MANIFEST_KEY"], "[]"),
                    ),
                )
            self.assertEqual(MIGRATOR["verify_current_database"](db), "pending")
            with closing(sqlite3.connect(db)) as connection, connection:
                insert_setting(
                    connection, "global", "", "LoginState/General",
                    "PasswordBase64", "legacy",
                )
            with self.assertRaisesRegex(ValueError, "legacy authentication"):
                MIGRATOR["verify_current_database"](db)


@unittest.skipUnless(os.name == "nt", "CurrentUser DPAPI tests require Windows")
class WindowsSchema4UpgradeTests(unittest.TestCase):
    def test_atomic_publication_explicitly_rejects_non_windows_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            target = Path(text) / "must-not-be-created.bin"
            with (
                mock.patch.object(os, "name", "posix"),
                self.assertRaisesRegex(OSError, "only on Windows"),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target, b"no fallback replace", require_absent=True
                )
            self.assertFalse(target.exists())

    def test_windows_credential_scrub_publish_is_handle_bound_and_rolls_back(self) -> None:
        source_text = (
            REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"
        ).read_text(encoding="utf-8")
        atomic_start = source_text.index("def _atomic_replace_bytes(")
        atomic_end = source_text.index("def _sanitized_legacy_ini_bytes(")
        atomic_source = source_text[atomic_start:atomic_end]
        publisher_start = source_text.index("def _win32_rename_handle(")
        publisher_end = source_text.index("def _canonical_sqlite_snapshot_bytes(")
        publisher_source = source_text[publisher_start:publisher_end]
        self.assertIn("_win32_atomic_replace_bytes(", atomic_source)
        self.assertIn("SetFileInformationByHandle", publisher_source)
        self.assertIn(
            "_win32_rename_handle(",
            publisher_source,
        )
        self.assertNotIn("os.replace(", atomic_source)

        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            first = data / "First.ini"
            second = data / "Second.ini"
            first.write_text(
                "[Remote]\nApiToken=first-secret\nKeepMe=1\n",
                encoding="utf-8",
            )
            second.write_text(
                "[Remote]\nPassword=second-secret\nKeepMe=2\n",
                encoding="utf-8",
            )
            originals = {path: path.read_bytes() for path in (first, second)}
            manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                data, None
            )
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "pending", manifest)
            db_before = db.read_bytes()
            function_globals = MIGRATOR["_atomic_replace_bytes"].__globals__
            actual_replace = function_globals["_win32_atomic_replace_bytes"]
            calls: list[str] = []

            def faulting_replace(
                path: Path,
                content: bytes,
                *,
                expected_existing: bytes | None,
                require_absent: bool,
                sqlite_sidecar_base: Path | None = None,
                expected_existing_identity: tuple[int, int] | None = None,
            ) -> None:
                calls.append(path.name)
                if len(calls) == 2:
                    raise OSError("injected second-file replacement failure")
                actual_replace(
                    path,
                    content,
                    expected_existing=expected_existing,
                    require_absent=require_absent,
                    sqlite_sidecar_base=sqlite_sidecar_base,
                    expected_existing_identity=expected_existing_identity,
                )

            function_globals["_win32_atomic_replace_bytes"] = faulting_replace
            try:
                with (
                    mock.patch.object(
                        os,
                        "replace",
                        side_effect=AssertionError(
                            "Windows credential scrub called os.replace"
                        ),
                    ),
                    self.assertRaisesRegex(
                        OSError, "injected second-file replacement failure"
                    ),
                ):
                    MIGRATOR["_finish_legacy_credential_scrub"](
                        data, db, None, True
                    )
            finally:
                function_globals["_win32_atomic_replace_bytes"] = actual_replace

            self.assertEqual(
                {path: path.read_bytes() for path in (first, second)},
                originals,
            )
            self.assertEqual(db.read_bytes(), db_before)
            self.assertEqual(
                MIGRATOR["read_legacy_credential_scrub_provenance"](db)[0],
                "pending",
            )
            self.assertEqual(
                calls,
                ["First.ini", "Second.ini", "First.ini"],
            )
            self.assertFalse(
                any(data.glob(".*.atomic-*.tmp"))
            )

    def test_credential_scrub_rejects_rooted_manifest_without_external_write(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            outside = root / "outside.ini"
            outside.write_text(
                "[Remote]\nPassword=outside-secret\nKeepMe=1\n",
                encoding="utf-8",
            )
            original = outside.read_bytes()
            sanitized, removed = MIGRATOR["_sanitized_legacy_ini_bytes"](
                outside, None
            )
            absolute = Path(os.path.abspath(outside))
            leading_root = "\\" + str(absolute.relative_to(Path(absolute.anchor)))
            manifest = [{
                "path": leading_root,
                "before_sha256": hashlib.sha256(original).hexdigest(),
                "after_sha256": hashlib.sha256(sanitized).hexdigest(),
                "removed_values": removed,
            }]

            with self.assertRaisesRegex(ValueError, "canonical relative"):
                MIGRATOR["parse_legacy_credential_scrub_manifest"](
                    MIGRATOR["serialize_legacy_credential_scrub_manifest"](
                        manifest
                    )
                )
            with self.assertRaisesRegex(ValueError, "canonical relative"):
                MIGRATOR["apply_legacy_ini_credential_scrub"](
                    data, None, manifest
                )
            self.assertEqual(outside.read_bytes(), original)
            self.assertFalse(any(data.iterdir()))

    def test_robot_custom_name_rejects_rooted_external_source(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            outside_robot = root / "OutsideRobot"
            outside_robot.mkdir()
            robot_para = outside_robot / "RobotPara.ini"
            robot_para.write_text(
                "[BaseParam]\nCustomName=OUTSIDE_SECRET_NAME\n",
                encoding="utf-8",
            )
            absolute = Path(os.path.abspath(outside_robot))
            leading_root = "\\" + str(absolute.relative_to(Path(absolute.anchor)))

            self.assertIsNone(
                MIGRATOR["read_robot_custom_name"](data, leading_root)
            )
            replacement = MIGRATOR["default_for_mojibake_value"](
                data,
                "ChineseName",
                "Unit0",
                {
                    "UnitName": {"Unit0": leading_root},
                    "UnitType": {"Unit0": "6"},
                },
            )
            self.assertEqual(replacement, "焊接机器人1")
            self.assertNotIn("OUTSIDE_SECRET_NAME", replacement or "")

    def _run_cli(self, *arguments: object) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"),
                *(str(argument) for argument in arguments),
            ],
            cwd=REPO_ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
        )

    def _assert_cli_pending(self, db: Path) -> None:
        result = self._run_cli("--verify-current", "--db", db)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("(scrub=pending)", result.stdout)

    def _assert_cli_complete(self, db: Path) -> None:
        result = self._run_cli("--verify-current", "--db", db)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("(scrub=complete)", result.stdout)

    def _tree_snapshot(self, data: Path) -> dict[str, bytes]:
        return {
            path.relative_to(data).as_posix(): path.read_bytes()
            for path in data.rglob("*")
            if path.is_file()
        }

    def _verify_installer_cli(
        self,
        data: Path,
        db: Path,
    ) -> subprocess.CompletedProcess[str]:
        return self._run_cli(
            "--verify-installer-state",
            "--source", data,
            "--db", db,
        )

    def _create_auth1_database(
        self,
        path: Path,
        scrub_state: str,
        scrub_manifest: list[dict[str, object]],
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        values = {
            "PasswordHash": password_record("admin", "auth1-staging"),
            "Role": "admin",
            "MustChangePassword": "0",
            "CreatedAt": "2026-01-02T03:04:05",
            "UpdatedAt": "2026-02-03T04:05:06Z",
            "PasswordChangedAt": "2026-02-02T03:04:05+08:00",
        }
        with closing(sqlite3.connect(path)) as connection, connection:
            MIGRATOR["create_current_tables"](connection)
            MIGRATOR["set_schema_meta"](
                connection, True, authentication_initialized=True
            )
            connection.execute(
                "UPDATE meta SET value='1' WHERE key='auth_semantic_version'"
            )
            connection.executemany(
                "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
                (
                    (MIGRATOR["SCRUB_STATE_KEY"], scrub_state),
                    (
                        MIGRATOR["SCRUB_MANIFEST_KEY"],
                        MIGRATOR["serialize_legacy_credential_scrub_manifest"](
                            scrub_manifest
                        ),
                    ),
                ),
            )
            for key, value in values.items():
                insert_setting(
                    connection,
                    "account",
                    "admin",
                    "Profile",
                    key,
                    value,
                    value_type=(
                        "bool" if key == "MustChangePassword"
                        else "datetime" if key.endswith("At")
                        else "string"
                    ),
                    sensitive=1 if "Password" in key else 0,
                )

    def test_read_only_verification_refuses_pending_atomic_recovery(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            original_hash = hashlib.sha256(db.read_bytes()).hexdigest()
            record = MIGRATOR["_atomic_transaction_path"](db)
            record.write_bytes(MIGRATOR["_serialize_atomic_transaction"](
                "replace",
                "1" * 32,
                original_hash,
                "2" * 64,
            ))
            before = self._tree_snapshot(data)
            with self.assertRaisesRegex(ValueError, "Pending atomic"):
                MIGRATOR["verify_current_database"](db)
            self.assertEqual(self._tree_snapshot(data), before)

    def test_dpapi_verifier_refuses_pending_atomic_inputs_without_mutation(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            backup = data / "ConfigStore.db.verify.dpapi.bak"
            MIGRATOR["create_dpapi_database_backup"](db, backup)

            for pending_path in (db, backup):
                with self.subTest(pending=pending_path.name):
                    record = MIGRATOR["_atomic_transaction_path"](
                        pending_path
                    )
                    record.write_bytes(
                        MIGRATOR["_serialize_atomic_transaction"](
                            "replace",
                            "3" * 32,
                            hashlib.sha256(pending_path.read_bytes()).hexdigest(),
                            "4" * 64,
                        )
                    )
                    before = self._tree_snapshot(data)
                    with self.assertRaisesRegex(ValueError, "Pending atomic"):
                        MIGRATOR["verify_dpapi_database_backup_against"](
                            backup, db
                        )
                    self.assertEqual(self._tree_snapshot(data), before)
                    record.unlink()

    def test_current_verifier_binds_one_exact_database_identity(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute(
                    "UPDATE meta SET value='2' "
                    "WHERE key='auth_semantic_version'"
                )
            external_alias = root / "external-configstore-alias.db"
            os.link(db, external_alias)
            before = self._tree_snapshot(root)
            with self.assertRaisesRegex(ValueError, "exactly one hard link"):
                MIGRATOR["verify_current_database"](db)
            with self.assertRaisesRegex(ValueError, "exactly one hard link"):
                MIGRATOR["verify_installer_state"](data, db)
            self.assertEqual(self._tree_snapshot(root), before)

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute(
                    "UPDATE meta SET value='2' "
                    "WHERE key='auth_semantic_version'"
                )
            replacement = root / "replacement.db"
            replacement.write_bytes(db.read_bytes())
            renamed_data = root / "RenamedData"
            sqlite_module = MIGRATOR["verify_current_database"].__globals__["sqlite3"]
            actual_connect = sqlite_module.connect
            attacked = False

            def connect_with_path_swap_probe(
                *arguments: object, **keywords: object
            ) -> sqlite3.Connection:
                nonlocal attacked
                if not attacked and arguments and arguments[0] == ":memory:":
                    with self.assertRaises(OSError):
                        os.replace(replacement, db)
                    with self.assertRaises(OSError):
                        os.replace(data, renamed_data)
                    attacked = True
                return actual_connect(*arguments, **keywords)

            with mock.patch.object(
                sqlite_module, "connect", connect_with_path_swap_probe
            ):
                self.assertEqual(
                    MIGRATOR["verify_current_database"](db), "complete"
                )
            self.assertTrue(attacked)
            self.assertTrue(db.is_file())
            self.assertTrue(replacement.is_file())
            self.assertFalse(renamed_data.exists())

    def test_installer_verifier_holds_the_authorized_data_identity(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute(
                    "DELETE FROM meta WHERE key='schema_version'"
                )

            attacker_data = root / "AttackerData"
            attacker_data.mkdir()
            attacker_db = attacker_data / "ConfigStore.db"
            self._create_auth1_database(attacker_db, "complete", [])
            with closing(sqlite3.connect(attacker_db)) as connection, connection:
                connection.execute(
                    "UPDATE meta SET value='2' "
                    "WHERE key='auth_semantic_version'"
                )
            moved = root / "OriginalData"
            function_globals = MIGRATOR["verify_installer_state"].__globals__
            actual_reject = function_globals[
                "reject_pending_atomic_transactions"
            ]
            attacked = False

            def reject_with_parent_swap(path: Path) -> None:
                nonlocal attacked
                if not attacked:
                    os.replace(data, moved)
                    os.replace(attacker_data, data)
                    attacked = True
                actual_reject(path)

            with (
                mock.patch.dict(
                    function_globals,
                    {
                        "reject_pending_atomic_transactions":
                            reject_with_parent_swap,
                    },
                ),
                self.assertRaisesRegex(ValueError, "directory identity changed"),
            ):
                MIGRATOR["verify_installer_state"](data, db)
            self.assertTrue(attacked)
            self.assertTrue(data.is_dir())
            self.assertTrue(moved.is_dir())
            self.assertFalse(attacker_data.exists())

    def test_restore_rejects_lexical_reparse_paths_before_mutation(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            source = root / "Source.db"
            self._create_auth1_database(source, "complete", [])
            backup = root / "Source.db.restore.dpapi.bak"
            MIGRATOR["create_dpapi_database_backup"](source, backup)
            migration_source = root / "MigrationSource"
            migration_source.mkdir()
            runtime_ini = migration_source / "Runtime.ini"
            runtime_ini.write_text(
                "[Runtime]\nPassCount=5\n", encoding="utf-8"
            )
            outside = root / "Outside"
            outside.mkdir()
            lexical = root / "LexicalData"
            try:
                os.symlink(outside, lexical, target_is_directory=True)
            except OSError as exc:
                self.skipTest(
                    f"directory symlink creation is unavailable: {exc}"
                )

            source_before = source.read_bytes()
            backup_before = backup.read_bytes()
            runtime_before = runtime_ini.read_bytes()
            outside_before = self._tree_snapshot(outside)
            try:
                result = self._run_cli(
                    "--restore-dpapi-backup", backup,
                    "--db", lexical / "ConfigStore.db",
                )
                self.assertNotEqual(
                    result.returncode, 0, result.stdout + result.stderr
                )
                self.assertFalse((outside / "ConfigStore.db").exists())

                with self.assertRaisesRegex(
                    ValueError, "parent must already exist|reparse"
                ):
                    MIGRATOR["restore_dpapi_database_backup"](
                        backup,
                        lexical / "CreatedOutside" / "ConfigStore.db",
                        False,
                    )
                self.assertFalse((outside / "CreatedOutside").exists())

                migration = self._run_cli(
                    "--source", migration_source,
                    "--db", lexical / "CreatedOutside" / "ConfigStore.db",
                    "--encrypt",
                )
                self.assertNotEqual(
                    migration.returncode,
                    0,
                    migration.stdout + migration.stderr,
                )
                self.assertFalse((outside / "CreatedOutside").exists())
                self.assertEqual(self._tree_snapshot(outside), outside_before)
                self.assertEqual(source.read_bytes(), source_before)
                self.assertEqual(backup.read_bytes(), backup_before)
                self.assertEqual(runtime_ini.read_bytes(), runtime_before)
            finally:
                lexical.unlink(missing_ok=True)

    def test_atomic_publication_binds_source_and_destination_handles(self) -> None:
        function_globals = MIGRATOR["_atomic_replace_bytes"].__globals__
        actual_rename = function_globals["_win32_rename_handle"]
        handle_path = function_globals["_win32_handle_path"]

        with tempfile.TemporaryDirectory() as text:
            data = Path(text)
            target = data / "atomic-target.bin"
            source_attacker = data / "source-attacker.bin"
            source_attacker.write_bytes(b"source attacker must remain")
            source_attack_blocked = False

            def probe_source_handle(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                nonlocal source_attack_blocked
                if label == "new destination":
                    source_path = handle_path(
                        kernel32, handle, "test atomic source"
                    )
                    with self.assertRaises(OSError):
                        os.replace(source_attacker, source_path)
                    source_attack_blocked = True
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            with mock.patch.dict(
                function_globals,
                {"_win32_rename_handle": probe_source_handle},
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target, b"created bytes", require_absent=True
                )
            self.assertTrue(source_attack_blocked)
            self.assertEqual(target.read_bytes(), b"created bytes")
            self.assertEqual(
                source_attacker.read_bytes(), b"source attacker must remain"
            )

            destination_attacker = data / "destination-attacker.bin"
            destination_attacker.write_bytes(b"destination attacker must remain")
            destination_attack_blocked = False

            def probe_destination_handle(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                nonlocal destination_attack_blocked
                if label == "old destination":
                    with self.assertRaises(OSError):
                        os.replace(destination_attacker, target)
                    destination_attack_blocked = True
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            before = target.read_bytes()
            with mock.patch.dict(
                function_globals,
                {"_win32_rename_handle": probe_destination_handle},
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target, b"replacement bytes", expected_existing=before
                )
            self.assertTrue(destination_attack_blocked)
            self.assertEqual(target.read_bytes(), b"replacement bytes")
            self.assertEqual(
                destination_attacker.read_bytes(),
                b"destination attacker must remain",
            )

            target.unlink()
            racing_bytes = b"independent creator won the absent target"
            raced = False

            def create_destination_race(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                nonlocal raced
                if label == "new destination" and not raced:
                    destination.write_bytes(racing_bytes)
                    raced = True
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            with (
                mock.patch.dict(
                    function_globals,
                    {"_win32_rename_handle": create_destination_race},
                ),
                self.assertRaises(OSError),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target, b"must not replace racer", require_absent=True
                )
            self.assertTrue(raced)
            self.assertEqual(target.read_bytes(), racing_bytes)
            self.assertFalse(any(data.glob(".atomic-target.bin.atomic-*.tmp")))

    def test_sqlite_atomic_publication_reserves_all_sidecar_names(self) -> None:
        function_globals = MIGRATOR["_atomic_replace_bytes"].__globals__
        actual_rename = function_globals["_win32_rename_handle"]

        for destination_state in ("absent", "existing"):
            with self.subTest(destination_state=destination_state), \
                    tempfile.TemporaryDirectory() as text:
                data = Path(text)
                db = data / "ConfigStore.db"
                expected: bytes | None = None
                if destination_state == "existing":
                    db.write_bytes(b"old database bytes")
                    expected = db.read_bytes()
                blocked_suffixes: set[str] = set()

                def probe_reserved_sidecars(
                    kernel32: object,
                    handle: int,
                    destination: Path,
                    replace_existing: bool,
                    label: str,
                ) -> None:
                    if label == "new destination":
                        for suffix in ("-journal", "-wal", "-shm"):
                            sidecar = Path(str(db) + suffix)
                            with self.assertRaises(OSError):
                                sidecar.write_bytes(
                                    f"attacker {suffix}".encode("ascii")
                                )
                            blocked_suffixes.add(suffix)
                    actual_rename(
                        kernel32, handle, destination, replace_existing, label
                    )

                with mock.patch.dict(
                    function_globals,
                    {"_win32_rename_handle": probe_reserved_sidecars},
                ):
                    kwargs: dict[str, object] = {
                        "sqlite_sidecar_base": db,
                    }
                    if expected is None:
                        kwargs["require_absent"] = True
                    else:
                        kwargs["expected_existing"] = expected
                    MIGRATOR["_atomic_replace_bytes"](
                        db, b"new database bytes", **kwargs
                    )

                self.assertEqual(
                    blocked_suffixes, {"-journal", "-wal", "-shm"}
                )
                self.assertEqual(db.read_bytes(), b"new database bytes")
                self.assertFalse(any(
                    Path(str(db) + suffix).exists()
                    for suffix in ("-journal", "-wal", "-shm")
                ))
                self.assertFalse(any(data.glob(".ConfigStore.db.atomic-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            data = Path(text)
            db = data / "ConfigStore.db"
            db.write_bytes(b"stable original database")
            wal = Path(str(db) + "-wal")
            wal.write_bytes(b"pre-existing wal sentinel")
            before = db.read_bytes()
            with self.assertRaisesRegex(ValueError, "sidecar"):
                MIGRATOR["_atomic_replace_bytes"](
                    db,
                    b"must not publish",
                    expected_existing=before,
                    sqlite_sidecar_base=db,
                )
            self.assertEqual(db.read_bytes(), before)
            self.assertEqual(wal.read_bytes(), b"pre-existing wal sentinel")
            self.assertFalse(any(data.glob(".ConfigStore.db.atomic-*.tmp")))

    def test_atomic_publication_recovers_every_forced_commit_stage(self) -> None:
        child_script = r'''
import os
import runpy
import time
from pathlib import Path

root = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
target = Path(os.environ["CONFIG_MIGRATE_TARGET"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
stage = os.environ["CONFIG_MIGRATE_KILL_STAGE"]
alias_text = os.environ.get("CONFIG_MIGRATE_QUARANTINE_ALIAS", "")
sqlite_target = os.environ.get("CONFIG_MIGRATE_SQLITE_TARGET", "") == "1"
migrator = runpy.run_path(
    str(root / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_atomic_kill_test",
)
function_globals = migrator["_win32_atomic_replace_bytes"].__globals__
original_rename = function_globals["_win32_rename_handle"]
original_write = function_globals["_win32_write_exact_handle"]
original_delete = function_globals["_win32_delete_handle"]

def pause(stage_name):
    marker.write_text(stage_name, encoding="ascii")
    time.sleep(30)

def pause_after_commit_step(kernel32, handle, destination, replace_existing, label):
    original_rename(kernel32, handle, destination, replace_existing, label)
    if alias_text and stage == "old-quarantined" and label == "old destination":
        os.link(destination, Path(alias_text))
    if ((stage == "old-quarantined" and label == "old destination") or
            (stage == "new-published" and label == "new destination")):
        pause(stage)

def pause_after_durable_write(kernel32, handle, content, label):
    if stage == "staging-opened" and label == "atomic staging file":
        pause(stage)
    original_write(kernel32, handle, content, label)
    if ((stage == "record-written" and
            label == "atomic replacement transaction record") or
            (stage == "staging-written" and label == "atomic staging file")):
        pause(stage)

def pause_after_delete_pending(kernel32, handle, label):
    original_delete(kernel32, handle, label)
    if ((stage == "old-delete-pending" and
            label == "old atomic destination") or
            (stage == "record-retired" and
            label == "atomic replacement transaction record")):
        pause(stage)

function_globals["_win32_rename_handle"] = pause_after_commit_step
function_globals["_win32_write_exact_handle"] = pause_after_durable_write
function_globals["_win32_delete_handle"] = pause_after_delete_pending
arguments = {
    "expected_existing": b"old stable bytes",
}
if sqlite_target:
    arguments["sqlite_sidecar_base"] = target
migrator["_atomic_replace_bytes"](
    target, b"new committed bytes", **arguments
)
'''
        for stage in (
            "record-written",
            "staging-opened",
            "staging-written",
            "old-quarantined",
            "new-published",
            "old-delete-pending",
            "record-retired",
        ):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                target = root / "atomic-target.bin"
                marker = root / "commit-stage.marker"
                target.write_bytes(b"old stable bytes")
                environment = os.environ.copy()
                environment.update({
                    "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                    "CONFIG_MIGRATE_TARGET": str(target),
                    "CONFIG_MIGRATE_MARKER": str(marker),
                    "CONFIG_MIGRATE_KILL_STAGE": stage,
                })
                child = subprocess.Popen(
                    [sys.executable, "-B", "-c", child_script],
                    cwd=REPO_ROOT,
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                try:
                    deadline = time.monotonic() + 20
                    while not marker.exists() and time.monotonic() < deadline:
                        time.sleep(0.05)
                    if not marker.exists():
                        child.kill()
                        output, errors = child.communicate(timeout=10)
                        self.fail(
                            f"child did not reach {stage}; "
                            f"stdout={output!r}; stderr={errors!r}"
                        )
                    child.kill()
                    child.communicate(timeout=10)
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.communicate(timeout=10)

                self.assertEqual(
                    any(root.glob(".*.atomic-transaction-v1")),
                    stage != "record-retired",
                )
                expected_after_recovery = (
                    b"old stable bytes"
                    if stage in {
                        "record-written", "staging-opened", "staging-written",
                        "old-quarantined"
                    }
                    else b"new committed bytes"
                )
                MIGRATOR["recover_pending_atomic_transactions"](root)
                self.assertEqual(
                    target.read_bytes(), expected_after_recovery
                )
                self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
                self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
                self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))
                MIGRATOR["_atomic_replace_bytes"](
                    target,
                    b"next complete bytes",
                    expected_existing=expected_after_recovery,
                )
                self.assertEqual(target.read_bytes(), b"next complete bytes")
                self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
                self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
                self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            marker = root / "commit-stage.marker"
            alias = root / "forced-kill-old-alias.bin"
            target.write_bytes(b"old stable bytes")
            environment = os.environ.copy()
            environment.update({
                "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                "CONFIG_MIGRATE_TARGET": str(target),
                "CONFIG_MIGRATE_MARKER": str(marker),
                "CONFIG_MIGRATE_KILL_STAGE": "old-quarantined",
                "CONFIG_MIGRATE_QUARANTINE_ALIAS": str(alias),
            })
            child = subprocess.Popen(
                [sys.executable, "-B", "-c", child_script],
                cwd=REPO_ROOT,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not marker.exists():
                    child.kill()
                    output, errors = child.communicate(timeout=10)
                    self.fail(
                        "child did not reach aliased old-quarantined stage; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                child.kill()
                child.communicate(timeout=10)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.communicate(timeout=10)

            self.assertFalse(target.exists())
            self.assertEqual(alias.read_bytes(), b"old stable bytes")
            before = self._tree_snapshot(root)
            for _attempt in range(2):
                with self.assertRaisesRegex(
                    ValueError, "hard-link alias.*remains pending"
                ):
                    MIGRATOR["recover_pending_atomic_transactions"](root)
                self.assertEqual(self._tree_snapshot(root), before)
            self.assertTrue(any(root.glob(".*.atomic-transaction-v1")))
            self.assertTrue(any(root.glob(".*.atomic-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / (
                ".ConfigStore.db.install-upgrade-"
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.tmp"
            )
            marker = root / "sqlite-commit-stage.marker"
            target.write_bytes(b"old stable bytes")
            environment = os.environ.copy()
            environment.update({
                "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                "CONFIG_MIGRATE_TARGET": str(target),
                "CONFIG_MIGRATE_MARKER": str(marker),
                "CONFIG_MIGRATE_KILL_STAGE": "new-published",
                "CONFIG_MIGRATE_SQLITE_TARGET": "1",
            })
            child = subprocess.Popen(
                [sys.executable, "-B", "-c", child_script],
                cwd=REPO_ROOT,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not marker.exists():
                    child.kill()
                    output, errors = child.communicate(timeout=10)
                    self.fail(
                        "SQLite child did not reach new-published stage; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                child.kill()
                child.communicate(timeout=10)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.communicate(timeout=10)

            wal = Path(str(target) + "-wal")
            wal.write_bytes(b"unsafe interrupted WAL")
            before = self._tree_snapshot(root)
            with self.assertRaisesRegex(ValueError, "unsafe sidecar"):
                MIGRATOR["recover_pending_atomic_transactions"](root)
            self.assertEqual(self._tree_snapshot(root), before)
            self.assertTrue(any(root.glob(".*.atomic-transaction-v1")))
            wal.unlink()
            MIGRATOR["recover_pending_atomic_transactions"](root)
            self.assertEqual(target.read_bytes(), b"new committed bytes")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_dpapi_backup_create_recovers_forced_publication_stages(self) -> None:
        child_script = r'''
import os
import runpy
import time
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
source = Path(os.environ["CONFIG_MIGRATE_BACKUP_SOURCE"])
backup = Path(os.environ["CONFIG_MIGRATE_BACKUP_TARGET"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
stage = os.environ["CONFIG_MIGRATE_KILL_STAGE"]
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_backup_kill_test",
)
function_globals = migrator["_create_dpapi_database_backup_from_bytes"].__globals__
original_rename = function_globals["_win32_rename_handle"]
original_write = function_globals["_win32_write_exact_handle"]
original_delete = function_globals["_win32_delete_handle"]
locked_type = function_globals["_LockedWin32ReadFile"]
original_enter = locked_type.__enter__

def pause(name):
    marker.write_text(name, encoding="ascii")
    time.sleep(30)

def write_then_pause(kernel32, handle, content, label):
    original_write(kernel32, handle, content, label)
    if ((stage == "record-written" and
            label == "atomic replacement transaction record") or
            (stage == "staging-written" and label == "atomic staging file")):
        pause(stage)

def rename_then_pause(kernel32, handle, destination, replace_existing, label):
    original_rename(kernel32, handle, destination, replace_existing, label)
    if stage == "new-published" and label == "new destination":
        pause(stage)

def delete_then_pause(kernel32, handle, label):
    original_delete(kernel32, handle, label)
    if (stage == "record-retired" and
            label == "atomic replacement transaction record"):
        pause(stage)

def enter_then_pause(locked):
    result = original_enter(locked)
    if stage == "verification-open" and locked.label == "Created DPAPI backup":
        pause(stage)
    return result

function_globals["_win32_write_exact_handle"] = write_then_pause
function_globals["_win32_rename_handle"] = rename_then_pause
function_globals["_win32_delete_handle"] = delete_then_pause
locked_type.__enter__ = enter_then_pause
migrator["_create_dpapi_database_backup_from_bytes"](
    source.read_bytes(), backup
)
'''
        for stage in (
            "record-written",
            "staging-written",
            "new-published",
            "record-retired",
            "verification-open",
        ):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                source = root / "Source.db"
                with closing(sqlite3.connect(source)) as connection, connection:
                    connection.execute("CREATE TABLE sample(value TEXT)")
                    connection.execute("INSERT INTO sample VALUES('stable')")
                backup = root / "Source.db.forced.dpapi.bak"
                marker = root / "backup-stage.marker"
                environment = os.environ.copy()
                environment.update({
                    "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                    "CONFIG_MIGRATE_BACKUP_SOURCE": str(source),
                    "CONFIG_MIGRATE_BACKUP_TARGET": str(backup),
                    "CONFIG_MIGRATE_MARKER": str(marker),
                    "CONFIG_MIGRATE_KILL_STAGE": stage,
                })
                child = subprocess.Popen(
                    [sys.executable, "-B", "-c", child_script],
                    cwd=REPO_ROOT,
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                try:
                    deadline = time.monotonic() + 20
                    while not marker.exists() and time.monotonic() < deadline:
                        time.sleep(0.05)
                    if not marker.exists():
                        child.kill()
                        output, errors = child.communicate(timeout=10)
                        self.fail(
                            f"backup child did not reach {stage}; "
                            f"stdout={output!r}; stderr={errors!r}"
                        )
                    child.kill()
                    child.communicate(timeout=10)
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.communicate(timeout=10)

                MIGRATOR["_create_dpapi_database_backup_from_bytes"](
                    source.read_bytes(), backup
                )
                restored = MIGRATOR["_read_dpapi_database_backup"](backup)
                self.assertEqual(
                    MIGRATOR["_canonical_sqlite_snapshot_bytes"](restored),
                    MIGRATOR["_canonical_sqlite_snapshot_bytes"](
                        source.read_bytes()
                    ),
                )
                self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
                self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
                self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_changed_source_backup_cleanup_preserves_prebound_alias(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            source = root / "Source.db"
            with closing(sqlite3.connect(source)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
                connection.execute(
                    "INSERT INTO sample VALUES('prebound cleanup alias')"
                )
            backup = root / "Source.db.changed.dpapi.bak"
            alias = root / "prebound-backup-alias.dpapi.bak"
            function_globals = MIGRATOR[
                "create_dpapi_database_backup"
            ].__globals__
            original_create = function_globals[
                "_create_dpapi_database_backup_from_bytes"
            ]
            locked_type = function_globals["_LockedWin32ReadFile"]
            original_read = locked_type.read_bytes
            source_reads = 0

            def create_then_link(
                database_bytes: bytes,
                backup_path: Path,
            ) -> Path:
                result = original_create(database_bytes, backup_path)
                os.link(result, alias)
                return result

            def report_changed_source(locked: object) -> bytes:
                nonlocal source_reads
                content = original_read(locked)
                if locked.label == "SQLite backup source":
                    source_reads += 1
                    if source_reads == 2:
                        return content + b"synthetic source change"
                return content

            with (
                mock.patch.dict(
                    function_globals,
                    {
                        "_create_dpapi_database_backup_from_bytes":
                            create_then_link,
                    },
                ),
                mock.patch.object(
                    locked_type, "read_bytes", report_changed_source
                ),
                self.assertRaisesRegex(
                    RuntimeError,
                    "source changed during protected backup.*exact-handle cleanup",
                ),
            ):
                MIGRATOR["create_dpapi_database_backup"](source, backup)

            self.assertEqual(source_reads, 2)
            self.assertTrue(backup.is_file())
            self.assertEqual(backup.read_bytes(), alias.read_bytes())
            self.assertGreater(len(alias.read_bytes()), 0)
            self.assertEqual(os.stat(backup).st_nlink, 2)
            self.assertEqual(
                MIGRATOR["_read_dpapi_database_backup_content"](
                    alias.read_bytes()
                ),
                MIGRATOR["_canonical_sqlite_snapshot_bytes"](
                    source.read_bytes()
                ),
            )
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_changed_source_cleanup_never_claims_empty_multilink_object(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            source = root / "Source.db"
            with closing(sqlite3.connect(source)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
            canonical = MIGRATOR["_canonical_sqlite_snapshot_bytes"](
                source.read_bytes()
            )
            backup = root / "unowned-empty.dpapi.bak"
            alias = root / "unowned-empty-alias.dpapi.bak"
            backup.write_bytes(b"")
            os.link(backup, alias)

            with self.assertRaisesRegex(ValueError, "hard-link alias"):
                MIGRATOR["_securely_discard_created_dpapi_backup"](
                    backup, canonical
                )

            self.assertTrue(backup.is_file())
            self.assertTrue(alias.is_file())
            self.assertEqual(backup.read_bytes(), b"")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertEqual(os.stat(backup).st_nlink, 2)

    def test_changed_source_backup_cleanup_scrubs_postbind_alias_before_kill(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            source = root / "Source.db"
            with closing(sqlite3.connect(source)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
                connection.execute(
                    "INSERT INTO sample VALUES('reported post-bind alias')"
                )
            backup = root / "Source.db.changed.dpapi.bak"
            alias = root / "reported-postbind-backup-alias.dpapi.bak"
            function_globals = MIGRATOR[
                "create_dpapi_database_backup"
            ].__globals__
            locked_type = function_globals["_LockedWin32ReadFile"]
            original_read = locked_type.read_bytes
            original_scrub = function_globals["_win32_scrub_exact_handle"]
            handle_path = function_globals["_win32_handle_path"]
            source_reads = 0
            linked = False

            def report_changed_source(locked: object) -> bytes:
                nonlocal source_reads
                content = original_read(locked)
                if locked.label == "SQLite backup source":
                    source_reads += 1
                    if source_reads == 2:
                        return content + b"synthetic source change"
                return content

            def link_after_single_link_bind_then_scrub(
                kernel32: object, handle: int, label: str
            ) -> None:
                nonlocal linked
                if label == "changed-source DPAPI backup cleanup" and not linked:
                    bound_backup = handle_path(
                        kernel32, handle, "reported post-bind backup"
                    )
                    os.link(bound_backup, alias)
                    linked = True
                original_scrub(kernel32, handle, label)

            with (
                mock.patch.object(
                    locked_type, "read_bytes", report_changed_source
                ),
                mock.patch.dict(
                    function_globals,
                    {
                        "_win32_scrub_exact_handle":
                            link_after_single_link_bind_then_scrub,
                    },
                ),
                self.assertRaisesRegex(
                    RuntimeError,
                    "late hard-link alias.*scrubbed to empty content",
                ),
            ):
                MIGRATOR["create_dpapi_database_backup"](source, backup)

            self.assertEqual(source_reads, 2)
            self.assertTrue(linked)
            self.assertFalse(backup.exists())
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

        child_script = r'''
import os
import runpy
import time
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
source = Path(os.environ["CONFIG_MIGRATE_BACKUP_SOURCE"])
backup = Path(os.environ["CONFIG_MIGRATE_BACKUP_TARGET"])
alias = Path(os.environ["CONFIG_MIGRATE_BACKUP_ALIAS"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_changed_source_backup_cleanup_kill_test",
)
function_globals = migrator["create_dpapi_database_backup"].__globals__
locked_type = function_globals["_LockedWin32ReadFile"]
original_read = locked_type.read_bytes
original_scrub = function_globals["_win32_scrub_exact_handle"]
original_delete = function_globals["_win32_delete_handle"]
handle_path = function_globals["_win32_handle_path"]
source_reads = 0
linked = False
paused = False

def report_changed_source(locked):
    global source_reads
    content = original_read(locked)
    if locked.label == "SQLite backup source":
        source_reads += 1
        if source_reads == 2:
            return content + b"synthetic source change"
    return content

def link_after_single_link_bind_then_scrub(kernel32, handle, label):
    global linked
    if label == "changed-source DPAPI backup cleanup" and not linked:
        bound_backup = handle_path(
            kernel32, handle, "post-bind changed-source backup"
        )
        os.link(bound_backup, alias)
        linked = True
    original_scrub(kernel32, handle, label)

def delete_then_pause(kernel32, handle, label):
    global paused
    original_delete(kernel32, handle, label)
    if label == "changed-source DPAPI backup cleanup" and not paused:
        paused = True
        marker.write_text("backup-delete-pending", encoding="ascii")
        time.sleep(30)

locked_type.read_bytes = report_changed_source
function_globals["_win32_scrub_exact_handle"] = (
    link_after_single_link_bind_then_scrub
)
function_globals["_win32_delete_handle"] = delete_then_pause
migrator["create_dpapi_database_backup"](source, backup)
'''
        recovery_script = r'''
import os
import runpy
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
root = Path(os.environ["CONFIG_MIGRATE_DATA_ROOT"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_changed_source_backup_cleanup_restart_test",
)
migrator["recover_pending_atomic_transactions"](root)
'''

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            source = root / "Source.db"
            with closing(sqlite3.connect(source)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
                connection.execute(
                    "INSERT INTO sample VALUES('post-bind cleanup alias')"
                )
            source_before = source.read_bytes()
            backup = root / "Source.db.changed.dpapi.bak"
            alias = root / "postbind-backup-alias.dpapi.bak"
            marker = root / "backup-delete-pending.marker"
            environment = os.environ.copy()
            environment.update({
                "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                "CONFIG_MIGRATE_DATA_ROOT": str(root),
                "CONFIG_MIGRATE_BACKUP_SOURCE": str(source),
                "CONFIG_MIGRATE_BACKUP_TARGET": str(backup),
                "CONFIG_MIGRATE_BACKUP_ALIAS": str(alias),
                "CONFIG_MIGRATE_MARKER": str(marker),
            })
            child = subprocess.Popen(
                [sys.executable, "-B", "-c", child_script],
                cwd=REPO_ROOT,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not marker.exists():
                    child.kill()
                    output, errors = child.communicate(timeout=10)
                    self.fail(
                        "backup cleanup did not reach delete disposition; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                child.kill()
                child.communicate(timeout=10)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.communicate(timeout=10)

            self.assertEqual(source.read_bytes(), source_before)
            self.assertFalse(backup.exists())
            self.assertTrue(alias.is_file())
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

            recovered = subprocess.run(
                [sys.executable, "-B", "-c", recovery_script],
                cwd=REPO_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
            self.assertEqual(
                recovered.returncode,
                0,
                f"stdout={recovered.stdout!r}; stderr={recovered.stderr!r}",
            )
            self.assertEqual(source.read_bytes(), source_before)
            self.assertFalse(backup.exists())
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))

    def test_atomic_publication_rejects_last_moment_hardlinks(self) -> None:
        function_globals = MIGRATOR["_atomic_replace_bytes"].__globals__
        actual_rename = function_globals["_win32_rename_handle"]
        actual_delete = function_globals["_win32_delete_handle"]
        handle_path = function_globals["_win32_handle_path"]

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            alias = root / "late-old-alias.bin"
            target.write_bytes(b"old sensitive bytes")
            injected = False

            def link_old_at_commit(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                nonlocal injected
                if label == "old destination" and not injected:
                    os.link(target, alias)
                    injected = True
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            with (
                mock.patch.dict(
                    function_globals,
                    {"_win32_rename_handle": link_old_at_commit},
                ),
                self.assertRaisesRegex(ValueError, "hard-link|changed"),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target,
                    b"must not publish",
                    expected_existing=b"old sensitive bytes",
                )
            self.assertTrue(injected)
            self.assertEqual(target.read_bytes(), b"old sensitive bytes")
            self.assertEqual(alias.read_bytes(), b"old sensitive bytes")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            alias = root / "late-new-alias.bin"
            target.write_bytes(b"old stable bytes")
            injected = False

            def link_staging_at_publish(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                nonlocal injected
                if label == "new destination" and not injected:
                    staging_path = handle_path(
                        kernel32, handle, "late linked staging"
                    )
                    os.link(staging_path, alias)
                    injected = True
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            with (
                mock.patch.dict(
                    function_globals,
                    {"_win32_rename_handle": link_staging_at_publish},
                ),
                self.assertRaisesRegex(ValueError, "hard-link|read-back"),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target,
                    b"new sensitive bytes",
                    expected_existing=b"old stable bytes",
                )
            self.assertTrue(injected)
            self.assertEqual(target.read_bytes(), b"old stable bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            alias = root / "last-window-old-alias.bin"
            target.write_bytes(b"old secret bytes")
            injected = False

            def link_old_before_delete_disposition(
                kernel32: object,
                handle: int,
                label: str,
            ) -> None:
                nonlocal injected
                if label == "old atomic destination" and not injected:
                    quarantine = handle_path(
                        kernel32, handle, "last-window old quarantine"
                    )
                    os.link(quarantine, alias)
                    injected = True
                actual_delete(kernel32, handle, label)

            with (
                mock.patch.dict(
                    function_globals,
                    {
                        "_win32_delete_handle":
                            link_old_before_delete_disposition,
                    },
                ),
                self.assertRaisesRegex(
                    ValueError, "late hard-link alias|bound old alias"
                ),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target,
                    b"verified new bytes",
                    expected_existing=b"old secret bytes",
                )
            self.assertTrue(injected)
            self.assertEqual(target.read_bytes(), b"verified new bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_atomic_rollback_scrubs_staging_before_last_window_alias(self) -> None:
        function_globals = MIGRATOR["_atomic_replace_bytes"].__globals__
        actual_rename = function_globals["_win32_rename_handle"]
        actual_delete = function_globals["_win32_delete_handle"]
        handle_path = function_globals["_win32_handle_path"]

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            alias = root / "rollback-staging-last-window-alias.bin"
            target.write_bytes(b"old stable bytes")
            linked = False

            def fail_before_new_publish(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                if label == "new destination":
                    raise RuntimeError("forced rollback before new publish")
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            def link_after_failed_staging_scrub(
                kernel32: object, handle: int, label: str
            ) -> None:
                nonlocal linked
                if label == "failed atomic staging" and not linked:
                    staging = handle_path(
                        kernel32, handle, "last-window failed staging"
                    )
                    os.link(staging, alias)
                    linked = True
                actual_delete(kernel32, handle, label)

            with (
                mock.patch.dict(
                    function_globals,
                    {
                        "_win32_rename_handle": fail_before_new_publish,
                        "_win32_delete_handle": link_after_failed_staging_scrub,
                    },
                ),
                self.assertRaisesRegex(
                    RuntimeError, "forced rollback before new publish"
                ),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target,
                    b"discarded sensitive staging bytes",
                    expected_existing=b"old stable bytes",
                )

            self.assertTrue(linked)
            self.assertEqual(target.read_bytes(), b"old stable bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_recovery_scrubs_quarantine_before_forced_disposition_kill(
        self,
    ) -> None:
        publish_script = r'''
import os
import runpy
import time
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
target = Path(os.environ["CONFIG_MIGRATE_TARGET"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_recovery_quarantine_publish_test",
)
function_globals = migrator["_win32_atomic_replace_bytes"].__globals__
original_rename = function_globals["_win32_rename_handle"]

def pause_after_new_publish(
    kernel32, handle, destination, replace_existing, label
):
    original_rename(kernel32, handle, destination, replace_existing, label)
    if label == "new destination":
        marker.write_text("new-published", encoding="ascii")
        time.sleep(30)

function_globals["_win32_rename_handle"] = pause_after_new_publish
migrator["_atomic_replace_bytes"](
    target,
    b"new committed bytes",
    expected_existing=b"old sensitive bytes",
)
'''
        recovery_script = r'''
import os
import runpy
import time
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
root = Path(os.environ["CONFIG_MIGRATE_DATA_ROOT"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
alias = Path(os.environ["CONFIG_MIGRATE_OLD_ALIAS"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_recovery_quarantine_kill_test",
)
function_globals = migrator["recover_pending_atomic_transactions"].__globals__
original_delete = function_globals["_win32_delete_handle"]
handle_path = function_globals["_win32_handle_path"]
paused = False

def alias_delete_then_pause(kernel32, handle, label):
    global paused
    if label == "atomic recovery committed quarantine" and not paused:
        quarantine = handle_path(
            kernel32, handle, "recovery committed quarantine kill test"
        )
        os.link(quarantine, alias)
        original_delete(kernel32, handle, label)
        paused = True
        marker.write_text("quarantine-delete-pending", encoding="ascii")
        time.sleep(30)
        return
    original_delete(kernel32, handle, label)

function_globals["_win32_delete_handle"] = alias_delete_then_pause
migrator["recover_pending_atomic_transactions"](root)
'''
        finish_recovery_script = r'''
import os
import runpy
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
root = Path(os.environ["CONFIG_MIGRATE_DATA_ROOT"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_recovery_quarantine_finish_test",
)
migrator["recover_pending_atomic_transactions"](root)
'''

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            publish_marker = root / "new-published.marker"
            recovery_marker = root / "recovery-delete-pending.marker"
            alias = root / "recovery-old-alias.bin"
            target.write_bytes(b"old sensitive bytes")
            common_environment = os.environ.copy()
            common_environment.update({
                "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                "CONFIG_MIGRATE_DATA_ROOT": str(root),
                "CONFIG_MIGRATE_TARGET": str(target),
            })

            publish_environment = common_environment.copy()
            publish_environment["CONFIG_MIGRATE_MARKER"] = str(publish_marker)
            publisher = subprocess.Popen(
                [sys.executable, "-B", "-c", publish_script],
                cwd=REPO_ROOT,
                env=publish_environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not publish_marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not publish_marker.exists():
                    publisher.kill()
                    output, errors = publisher.communicate(timeout=10)
                    self.fail(
                        "publisher did not retain raw quarantine; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                publisher.kill()
                publisher.communicate(timeout=10)
            finally:
                if publisher.poll() is None:
                    publisher.kill()
                    publisher.communicate(timeout=10)

            quarantines = list(root.glob(".*.atomic-rollback-*.tmp"))
            self.assertEqual(len(quarantines), 1)
            self.assertEqual(quarantines[0].read_bytes(), b"old sensitive bytes")
            self.assertEqual(target.read_bytes(), b"new committed bytes")
            self.assertTrue(any(root.glob(".*.atomic-transaction-v1")))

            recovery_environment = common_environment.copy()
            recovery_environment.update({
                "CONFIG_MIGRATE_MARKER": str(recovery_marker),
                "CONFIG_MIGRATE_OLD_ALIAS": str(alias),
            })
            recovery = subprocess.Popen(
                [sys.executable, "-B", "-c", recovery_script],
                cwd=REPO_ROOT,
                env=recovery_environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not recovery_marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not recovery_marker.exists():
                    recovery.kill()
                    output, errors = recovery.communicate(timeout=10)
                    self.fail(
                        "recovery did not reach quarantine disposition; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                recovery.kill()
                recovery.communicate(timeout=10)
            finally:
                if recovery.poll() is None:
                    recovery.kill()
                    recovery.communicate(timeout=10)

            self.assertEqual(target.read_bytes(), b"new committed bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))
            self.assertTrue(any(root.glob(".*.atomic-transaction-v1")))

            finished = subprocess.run(
                [sys.executable, "-B", "-c", finish_recovery_script],
                cwd=REPO_ROOT,
                env=common_environment,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
            self.assertEqual(
                finished.returncode,
                0,
                f"stdout={finished.stdout!r}; stderr={finished.stderr!r}",
            )
            self.assertEqual(target.read_bytes(), b"new committed bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_rollback_scrubs_staging_before_forced_disposition_kill(
        self,
    ) -> None:
        rollback_script = r'''
import os
import runpy
import time
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
target = Path(os.environ["CONFIG_MIGRATE_TARGET"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
alias = Path(os.environ["CONFIG_MIGRATE_STAGING_ALIAS"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_rollback_staging_kill_test",
)
function_globals = migrator["_win32_atomic_replace_bytes"].__globals__
original_rename = function_globals["_win32_rename_handle"]
original_delete = function_globals["_win32_delete_handle"]
handle_path = function_globals["_win32_handle_path"]
paused = False

def fail_before_new_publish(
    kernel32, handle, destination, replace_existing, label
):
    if label == "new destination":
        raise RuntimeError("forced rollback before new publish")
    original_rename(kernel32, handle, destination, replace_existing, label)

def alias_delete_then_pause(kernel32, handle, label):
    global paused
    if label == "failed atomic staging" and not paused:
        staging = handle_path(
            kernel32, handle, "rollback staging disposition kill test"
        )
        os.link(staging, alias)
        original_delete(kernel32, handle, label)
        paused = True
        marker.write_text("staging-delete-pending", encoding="ascii")
        time.sleep(30)
        return
    original_delete(kernel32, handle, label)

function_globals["_win32_rename_handle"] = fail_before_new_publish
function_globals["_win32_delete_handle"] = alias_delete_then_pause
migrator["_atomic_replace_bytes"](
    target,
    b"discarded sensitive staging bytes",
    expected_existing=b"old stable bytes",
)
'''
        finish_recovery_script = r'''
import os
import runpy
from pathlib import Path

repo = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
root = Path(os.environ["CONFIG_MIGRATE_DATA_ROOT"])
migrator = runpy.run_path(
    str(repo / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_rollback_staging_finish_test",
)
migrator["recover_pending_atomic_transactions"](root)
'''

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            marker = root / "rollback-delete-pending.marker"
            alias = root / "rollback-staging-alias.bin"
            target.write_bytes(b"old stable bytes")
            environment = os.environ.copy()
            environment.update({
                "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                "CONFIG_MIGRATE_DATA_ROOT": str(root),
                "CONFIG_MIGRATE_TARGET": str(target),
                "CONFIG_MIGRATE_MARKER": str(marker),
                "CONFIG_MIGRATE_STAGING_ALIAS": str(alias),
            })
            rollback = subprocess.Popen(
                [sys.executable, "-B", "-c", rollback_script],
                cwd=REPO_ROOT,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not marker.exists():
                    rollback.kill()
                    output, errors = rollback.communicate(timeout=10)
                    self.fail(
                        "rollback did not reach staging disposition; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                rollback.kill()
                rollback.communicate(timeout=10)
            finally:
                if rollback.poll() is None:
                    rollback.kill()
                    rollback.communicate(timeout=10)

            self.assertEqual(target.read_bytes(), b"old stable bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))
            self.assertTrue(any(root.glob(".*.atomic-transaction-v1")))

            finished = subprocess.run(
                [sys.executable, "-B", "-c", finish_recovery_script],
                cwd=REPO_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
            self.assertEqual(
                finished.returncode,
                0,
                f"stdout={finished.stdout!r}; stderr={finished.stderr!r}",
            )
            self.assertEqual(target.read_bytes(), b"old stable bytes")
            self.assertEqual(alias.read_bytes(), b"")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_old_quarantine_alias_is_empty_across_forced_scrub_boundaries(
        self,
    ) -> None:
        child_script = r'''
import os
import runpy
import time
from pathlib import Path

root = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
target = Path(os.environ["CONFIG_MIGRATE_TARGET"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
alias = Path(os.environ["CONFIG_MIGRATE_OLD_ALIAS"])
stage = os.environ["CONFIG_MIGRATE_KILL_STAGE"]
migrator = runpy.run_path(
    str(root / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_atomic_scrub_kill_test",
)
function_globals = migrator["_win32_atomic_replace_bytes"].__globals__
original_scrub = function_globals["_win32_scrub_exact_handle"]
original_delete = function_globals["_win32_delete_handle"]
handle_path = function_globals["_win32_handle_path"]
linked = False

def pause(stage_name):
    marker.write_text(stage_name, encoding="ascii")
    time.sleep(30)

def alias_then_scrub(kernel32, handle, label):
    global linked
    if label == "retired old atomic destination" and not linked:
        quarantine = handle_path(
            kernel32, handle, "forced-kill old quarantine"
        )
        os.link(quarantine, alias)
        linked = True
    if stage == "before-old-scrub" and label == "retired old atomic destination":
        pause(stage)
    original_scrub(kernel32, handle, label)
    if stage == "old-scrubbed" and label == "retired old atomic destination":
        pause(stage)

def delete_then_pause(kernel32, handle, label):
    original_delete(kernel32, handle, label)
    if stage == "old-delete-pending" and label == "old atomic destination":
        pause(stage)

function_globals["_win32_scrub_exact_handle"] = alias_then_scrub
function_globals["_win32_delete_handle"] = delete_then_pause
migrator["_atomic_replace_bytes"](
    target,
    b"new committed bytes",
    expected_existing=b"old sensitive bytes",
)
'''
        for stage in (
            "before-old-scrub",
            "old-scrubbed",
            "old-delete-pending",
        ):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                target = root / "atomic-target.bin"
                marker = root / "scrub-stage.marker"
                alias = root / "forced-kill-old-alias.bin"
                target.write_bytes(b"old sensitive bytes")
                environment = os.environ.copy()
                environment.update({
                    "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                    "CONFIG_MIGRATE_TARGET": str(target),
                    "CONFIG_MIGRATE_MARKER": str(marker),
                    "CONFIG_MIGRATE_OLD_ALIAS": str(alias),
                    "CONFIG_MIGRATE_KILL_STAGE": stage,
                })
                child = subprocess.Popen(
                    [sys.executable, "-B", "-c", child_script],
                    cwd=REPO_ROOT,
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                try:
                    deadline = time.monotonic() + 20
                    while not marker.exists() and time.monotonic() < deadline:
                        time.sleep(0.05)
                    if not marker.exists():
                        child.kill()
                        output, errors = child.communicate(timeout=10)
                        self.fail(
                            f"child did not reach {stage}; "
                            f"stdout={output!r}; stderr={errors!r}"
                        )
                    child.kill()
                    child.communicate(timeout=10)
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.communicate(timeout=10)

                self.assertEqual(target.read_bytes(), b"new committed bytes")
                self.assertTrue(alias.is_file())
                self.assertEqual(
                    alias.read_bytes(),
                    (
                        b"old sensitive bytes"
                        if stage == "before-old-scrub"
                        else b""
                    ),
                )
                self.assertTrue(any(root.glob(".*.atomic-transaction-v1")))

                if stage == "before-old-scrub":
                    with self.assertRaisesRegex(
                        ValueError,
                        "committed quarantine had a hard-link alias",
                    ):
                        MIGRATOR["recover_pending_atomic_transactions"](root)
                elif stage == "old-scrubbed":
                    with self.assertRaisesRegex(
                        ValueError,
                        "scrubbed quarantine retained empty aliases",
                    ):
                        MIGRATOR["recover_pending_atomic_transactions"](root)
                else:
                    MIGRATOR["recover_pending_atomic_transactions"](root)

                self.assertEqual(target.read_bytes(), b"new committed bytes")
                self.assertEqual(alias.read_bytes(), b"")
                self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
                self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
                self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_atomic_publication_reconciles_post_success_exceptions(self) -> None:
        function_globals = MIGRATOR["_atomic_replace_bytes"].__globals__
        actual_rename = function_globals["_win32_rename_handle"]
        actual_delete = function_globals["_win32_delete_handle"]

        cases = (
            ("rename", {"old destination"}),
            ("rename", {"new destination"}),
            ("delete", {"old atomic destination"}),
            ("delete", {"atomic replacement transaction record"}),
            ("rename", {"old destination", "rollback destination"}),
            ("rename", {"new destination", "failed new destination"}),
        )
        for operation, labels in cases:
            with self.subTest(operation=operation, labels=sorted(labels)), \
                    tempfile.TemporaryDirectory() as text:
                root = Path(text)
                target = root / "atomic-target.bin"
                target.write_bytes(b"old stable bytes")
                raised: set[str] = set()

                def rename_then_raise(
                    kernel32: object,
                    handle: int,
                    destination: Path,
                    replace_existing: bool,
                    label: str,
                ) -> None:
                    actual_rename(
                        kernel32,
                        handle,
                        destination,
                        replace_existing,
                        label,
                    )
                    if label in labels and label not in raised:
                        raised.add(label)
                        raise RuntimeError(f"post-success rename: {label}")

                def delete_then_raise(
                    kernel32: object, handle: int, label: str
                ) -> None:
                    actual_delete(kernel32, handle, label)
                    if label in labels and label not in raised:
                        raised.add(label)
                        raise RuntimeError(f"post-success delete: {label}")

                patches = {
                    "_win32_rename_handle": rename_then_raise,
                    "_win32_delete_handle": delete_then_raise,
                }
                expected_exception = (
                    ValueError
                    if operation == "delete"
                    and labels == {"old atomic destination"}
                    else RuntimeError
                )
                with (
                    mock.patch.dict(function_globals, patches),
                    self.assertRaises(expected_exception),
                ):
                    MIGRATOR["_atomic_replace_bytes"](
                        target,
                        b"new committed bytes",
                        expected_existing=b"old stable bytes",
                    )
                self.assertTrue(raised)
                if any(root.glob(".*.atomic-transaction-v1")):
                    MIGRATOR["recover_pending_atomic_transactions"](root)
                expected = (
                    b"new committed bytes"
                    if operation == "delete"
                    else b"old stable bytes"
                )
                self.assertEqual(target.read_bytes(), expected)
                self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
                self.assertFalse(any(root.glob(".*.atomic-*.tmp")))
                self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "atomic-target.bin"
            alias = root / "scrubbed-old-alias.bin"
            target.write_bytes(b"old secret bytes")
            actual_scrub = function_globals["_win32_scrub_exact_handle"]
            linked = False

            def scrub_then_raise(
                kernel32: object, handle: int, label: str
            ) -> None:
                nonlocal linked
                if label == "retired old atomic destination" and not linked:
                    quarantine = function_globals["_win32_handle_path"](
                        kernel32, handle, "scrub interruption quarantine"
                    )
                    os.link(quarantine, alias)
                    linked = True
                actual_scrub(kernel32, handle, label)
                raise RuntimeError("post-success old scrub")

            with (
                mock.patch.dict(
                    function_globals,
                    {
                        "_win32_scrub_exact_handle": scrub_then_raise,
                    },
                ),
                self.assertRaisesRegex(RuntimeError, "post-success old scrub"),
            ):
                MIGRATOR["_atomic_replace_bytes"](
                    target,
                    b"verified new bytes",
                    expected_existing=b"old secret bytes",
                )
            self.assertTrue(linked)
            self.assertEqual(target.read_bytes(), b"verified new bytes")
            self.assertEqual(alias.read_bytes(), b"")
            with self.assertRaisesRegex(
                ValueError, "scrubbed quarantine retained empty aliases"
            ):
                MIGRATOR["recover_pending_atomic_transactions"](root)
            self.assertEqual(target.read_bytes(), b"verified new bytes")
            self.assertFalse(any(root.glob(".*.atomic-transaction-v1")))
            self.assertFalse(any(root.glob(".*.atomic-rollback-*.tmp")))

    def test_sidecar_sentinels_are_removed_after_forced_process_termination(self) -> None:
        if os.name != "nt":
            self.skipTest("Windows handle delete-on-close regression")
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            target = root / "ConfigStore.db"
            marker = root / "sentinel-created.marker"
            child_script = r'''
import os
import runpy
import time
from pathlib import Path

root = Path(os.environ["CONFIG_MIGRATE_REPO_ROOT"])
target = Path(os.environ["CONFIG_MIGRATE_TARGET"])
marker = Path(os.environ["CONFIG_MIGRATE_MARKER"])
migrator = runpy.run_path(
    str(root / "tools" / "migrate_config_to_sqlite.py"),
    run_name="config_migrate_forced_termination_test",
)
function_globals = migrator["_win32_atomic_replace_bytes"].__globals__
original = function_globals["_win32_handle_information"]
paused = False

def pause_after_sentinel(kernel32, handle, label):
    global paused
    if label.startswith("SQLite sidecar sentinel") and not paused:
        paused = True
        marker.write_text("created", encoding="ascii")
        time.sleep(30)
    return original(kernel32, handle, label)

function_globals["_win32_handle_information"] = pause_after_sentinel
migrator["_atomic_replace_bytes"](
    target,
    b"SQLite format 3\\x00forced termination test",
    require_absent=True,
    sqlite_sidecar_base=target,
)
'''
            environment = os.environ.copy()
            environment.update({
                "CONFIG_MIGRATE_REPO_ROOT": str(REPO_ROOT),
                "CONFIG_MIGRATE_TARGET": str(target),
                "CONFIG_MIGRATE_MARKER": str(marker),
            })
            child = subprocess.Popen(
                [sys.executable, "-B", "-c", child_script],
                cwd=REPO_ROOT,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 20
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not marker.exists():
                    child.kill()
                    output, errors = child.communicate(timeout=10)
                    self.fail(
                        "child did not create sentinels; "
                        f"stdout={output!r}; stderr={errors!r}"
                    )
                child.kill()
                child.communicate(timeout=10)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.communicate(timeout=10)
            self.assertFalse(target.exists())
            self.assertFalse(any(
                Path(str(target) + suffix).exists()
                for suffix in ("-journal", "-wal", "-shm")
            ))
            self.assertFalse(any(root.glob(".ConfigStore.db.atomic-*.tmp")))

    def test_new_migration_and_absent_restore_reject_sidecars_and_races(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            runtime = data / "Runtime.ini"
            runtime.write_text("[Runtime]\nPassCount=5\n", encoding="utf-8")
            db = data / "ConfigStore.db"
            wal = Path(str(db) + "-wal")
            wal.write_bytes(b"migration wal sentinel")
            source_before = runtime.read_bytes()
            with self.assertRaisesRegex(ValueError, "sidecar"):
                MIGRATOR["migrate"](
                    data, db, False, True, None, False
                )
            self.assertFalse(db.exists())
            self.assertEqual(wal.read_bytes(), b"migration wal sentinel")
            self.assertEqual(runtime.read_bytes(), source_before)

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            source = root / "Source.db"
            with closing(sqlite3.connect(source)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
                connection.execute("INSERT INTO sample VALUES('restore source')")
            backup = root / "Source.db.restore.dpapi.bak"
            MIGRATOR["create_dpapi_database_backup"](source, backup)
            backup_before = backup.read_bytes()
            target = root / "ConfigStore.db"
            wal = Path(str(target) + "-wal")
            wal.write_bytes(b"restore wal sentinel")
            with self.assertRaisesRegex(ValueError, "sidecar"):
                MIGRATOR["restore_dpapi_database_backup"](
                    backup, target, False
                )
            self.assertFalse(target.exists())
            self.assertEqual(wal.read_bytes(), b"restore wal sentinel")
            self.assertEqual(backup.read_bytes(), backup_before)

            wal.unlink()
            function_globals = MIGRATOR[
                "_atomic_replace_bytes"
            ].__globals__
            actual_rename = function_globals["_win32_rename_handle"]
            racing_bytes = b"restore destination racer"
            raced = False

            def race_restore_destination(
                kernel32: object,
                handle: int,
                destination: Path,
                replace_existing: bool,
                label: str,
            ) -> None:
                nonlocal raced
                if label == "new destination" and not raced:
                    destination.write_bytes(racing_bytes)
                    raced = True
                actual_rename(
                    kernel32, handle, destination, replace_existing, label
                )

            with (
                mock.patch.dict(
                    function_globals,
                    {"_win32_rename_handle": race_restore_destination},
                ),
                self.assertRaises(OSError),
            ):
                MIGRATOR["restore_dpapi_database_backup"](
                    backup, target, False
                )
            self.assertTrue(raced)
            self.assertEqual(target.read_bytes(), racing_bytes)
            self.assertEqual(backup.read_bytes(), backup_before)
            self.assertFalse(any(root.glob(".ConfigStore.db.atomic-*.tmp")))
            self.assertFalse(any(root.glob("ConfigStore.db.pre_restore_*.dpapi.bak")))

    def test_writer_commit_between_snapshot_and_cas_is_never_lost(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            runtime = data / "Runtime.ini"
            runtime.write_text("[Runtime]\nPassCount=7\n", encoding="utf-8")
            runtime_before = runtime.read_bytes()
            db = data / "ConfigStore.db"
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute(
                    "CREATE TABLE writer_marker(value TEXT NOT NULL)"
                )
            migration_globals = MIGRATOR["migrate"].__globals__
            actual_atomic = migration_globals["_atomic_replace_bytes"]
            writer_committed = False

            def commit_writer_before_cas(
                path: Path,
                content: bytes,
                *,
                expected_existing: bytes | None = None,
                require_absent: bool = False,
                sqlite_sidecar_base: Path | None = None,
                expected_existing_identity: tuple[int, int] | None = None,
                publication_guard_factory: object | None = None,
            ) -> None:
                nonlocal writer_committed
                if Path(path) == db and expected_existing is not None:
                    with closing(sqlite3.connect(db)) as writer, writer:
                        writer.execute(
                            "INSERT INTO writer_marker VALUES('late writer')"
                        )
                    writer_committed = True
                actual_atomic(
                    path,
                    content,
                    expected_existing=expected_existing,
                    require_absent=require_absent,
                    sqlite_sidecar_base=sqlite_sidecar_base,
                    expected_existing_identity=expected_existing_identity,
                    publication_guard_factory=publication_guard_factory,
                )

            with (
                mock.patch.dict(
                    migration_globals,
                    {"_atomic_replace_bytes": commit_writer_before_cas},
                ),
                self.assertRaisesRegex(ValueError, "changed"),
            ):
                MIGRATOR["migrate"](
                    data, db, True, True, None, False
                )
            self.assertTrue(writer_committed)
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM writer_marker"
                    ).fetchall(),
                    [("late writer",)],
                )
            self.assertEqual(runtime.read_bytes(), runtime_before)
            backups = list(data.glob(
                "ConfigStore.db.bak_overwrite_*.dpapi.bak"
            ))
            self.assertEqual(len(backups), 1)
            protected_snapshot = MIGRATOR[
                "_read_dpapi_database_backup"
            ](backups[0])
            snapshot = sqlite3.connect(":memory:")
            try:
                snapshot.deserialize(protected_snapshot)
                self.assertEqual(
                    snapshot.execute(
                        "SELECT value FROM writer_marker"
                    ).fetchall(),
                    [],
                )
            finally:
                snapshot.close()
            self.assertFalse(any(data.glob(".ConfigStore.db.staging-*.tmp")))
            self.assertFalse(any(data.glob(".ConfigStore.db.atomic-*.tmp")))

        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "pending", [])
            migration_globals = MIGRATOR[
                "mark_legacy_credential_scrub_complete"
            ].__globals__
            actual_atomic = migration_globals["_atomic_replace_bytes"]
            writer_committed = False

            def commit_meta_before_cas(
                path: Path,
                content: bytes,
                *,
                expected_existing: bytes | None = None,
                require_absent: bool = False,
                sqlite_sidecar_base: Path | None = None,
                expected_existing_identity: tuple[int, int] | None = None,
            ) -> None:
                nonlocal writer_committed
                with closing(sqlite3.connect(db)) as writer, writer:
                    writer.execute(
                        "INSERT INTO meta(key, value) VALUES('late_writer', 'kept')"
                    )
                writer_committed = True
                actual_atomic(
                    path,
                    content,
                    expected_existing=expected_existing,
                    require_absent=require_absent,
                    sqlite_sidecar_base=sqlite_sidecar_base,
                    expected_existing_identity=expected_existing_identity,
                )

            with (
                mock.patch.dict(
                    migration_globals,
                    {"_atomic_replace_bytes": commit_meta_before_cas},
                ),
                self.assertRaisesRegex(ValueError, "changed"),
            ):
                MIGRATOR["mark_legacy_credential_scrub_complete"](db)
            self.assertTrue(writer_committed)
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key='late_writer'"
                    ).fetchone(),
                    ("kept",),
                )
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key=?",
                        (MIGRATOR["SCRUB_STATE_KEY"],),
                    ).fetchone(),
                    ("pending",),
                )
            self.assertFalse(any(data.glob(".ConfigStore.db.atomic-*.tmp")))
            self.assertFalse(any(
                Path(str(db) + suffix).exists()
                for suffix in ("-journal", "-wal", "-shm")
            ))

    def test_finish_scrub_completion_cas_rejects_database_identity_swap(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            account_ini = data / "Accounts.ini"
            account_ini.write_text(
                "[Users/admin]\n"
                f"PasswordHash={password_record('admin', 'scrub-swap-a')}\n"
                "Role=admin\n",
                encoding="utf-8",
            )
            manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                data, None
            )
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "pending", manifest)
            replacement = data / "replacement.db"
            self._create_auth1_database(replacement, "pending", [])
            with closing(sqlite3.connect(replacement)) as connection, connection:
                connection.execute(
                    "UPDATE meta SET value=? WHERE key='auth_semantic_version'",
                    (MIGRATOR["AUTH_SEMANTIC_VERSION"],),
                )
            migration_globals = MIGRATOR[
                "_finish_legacy_credential_scrub"
            ].__globals__
            actual_atomic = migration_globals["_atomic_replace_bytes"]
            swapped = False

            def swap_before_completion_cas(
                path: Path,
                content: bytes,
                **keywords: object,
            ) -> None:
                nonlocal swapped
                if Path(path) == db and not swapped:
                    os.replace(replacement, db)
                    swapped = True
                actual_atomic(path, content, **keywords)

            with (
                mock.patch.dict(
                    migration_globals,
                    {"_atomic_replace_bytes": swap_before_completion_cas},
                ),
                self.assertRaisesRegex(ValueError, "identity changed"),
            ):
                MIGRATOR["_finish_legacy_credential_scrub"](
                    data, db, None, True
                )

            self.assertTrue(swapped)
            self.assertNotIn(
                "PasswordHash=", account_ini.read_text(encoding="utf-8")
            )
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key=?",
                        (MIGRATOR["SCRUB_STATE_KEY"],),
                    ).fetchone(),
                    ("pending",),
                )
            self.assertEqual(
                MIGRATOR["verify_current_database"](db), "pending"
            )

    def test_finish_scrub_rejects_late_credentials_during_completion_commit(self) -> None:
        for attack in ("atomic-entry", "retirement"):
            with self.subTest(attack=attack), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                account_ini = data / "Accounts.ini"
                account_ini.write_text(
                    "[Users/admin]\n"
                    f"PasswordHash={password_record('admin', attack)}\n"
                    "Role=admin\n",
                    encoding="utf-8",
                )
                manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                    data, None
                )
                db = data / "ConfigStore.db"
                self._create_auth1_database(db, "pending", manifest)
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.execute(
                        "UPDATE meta SET value=? "
                        "WHERE key='auth_semantic_version'",
                        (MIGRATOR["AUTH_SEMANTIC_VERSION"],),
                    )
                migration_globals = MIGRATOR[
                    "_finish_legacy_credential_scrub"
                ].__globals__
                injected = False

                def create_late_secret() -> None:
                    nonlocal injected
                    (data / "LateSecrets.ini").write_text(
                        "[Remote]\nApiToken=late-secret\n",
                        encoding="utf-8",
                    )
                    injected = True

                if attack == "atomic-entry":
                    actual_atomic = migration_globals["_atomic_replace_bytes"]

                    def create_before_completion_atomic(
                        path: Path,
                        content: bytes,
                        **keywords: object,
                    ) -> None:
                        if Path(path) == db and not injected:
                            create_late_secret()
                        actual_atomic(path, content, **keywords)

                    patches = {
                        "_atomic_replace_bytes": create_before_completion_atomic
                    }
                else:
                    actual_delete = migration_globals["_win32_delete_handle"]

                    def create_during_completion_retirement(
                        kernel32: object,
                        handle: int,
                        label: str,
                    ) -> None:
                        if (
                            label.startswith("SQLite sidecar sentinel")
                            and not injected
                        ):
                            create_late_secret()
                        actual_delete(kernel32, handle, label)

                    patches = {
                        "_win32_delete_handle": (
                            create_during_completion_retirement
                        )
                    }

                with (
                    mock.patch.dict(migration_globals, patches),
                    self.assertRaisesRegex(ValueError, "source inventory changed"),
                ):
                    MIGRATOR["_finish_legacy_credential_scrub"](
                        data, db, None, True
                    )
                self.assertTrue(injected)
                with closing(sqlite3.connect(db)) as connection:
                    self.assertEqual(
                        connection.execute(
                            "SELECT value FROM meta WHERE key=?",
                            (MIGRATOR["SCRUB_STATE_KEY"],),
                        ).fetchone(),
                        ("pending",),
                    )
                self.assertEqual(
                    MIGRATOR["verify_current_database"](db), "pending"
                )
                fresh = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                    data, None
                )
                self.assertEqual(
                    [item["path"] for item in fresh], ["LateSecrets.ini"]
                )
                self.assertFalse(any(
                    data.glob(f".*{MIGRATOR['ATOMIC_REPLACE_TRANSACTION_SUFFIX']}")
                ))

    def test_empty_sensitive_value_dpapi_round_trip(self) -> None:
        blob, sentinel = MIGRATOR["_blob_from_bytes"](b"")
        self.assertEqual(blob.cbData, 0)
        self.assertTrue(bool(blob.pbData))
        self.assertGreaterEqual(len(sentinel), 1)
        purpose = MIGRATOR["protection_purpose"](
            "global", "", "RuntimeSecrets", "ApiToken"
        )
        protected = MIGRATOR["protect_sensitive_text"]("", purpose)
        self.assertEqual(
            MIGRATOR["unprotect_sensitive_text"](protected, purpose), ""
        )

    def test_realistic_in_place_upgrade_preserves_8940_rows(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            runtime_ini = data / "CorrugatedSheetPointCloudEctration.ini"
            runtime_ini.write_text(
                "ifupright=true\nSave_File_Name=runtime-only-marker\n",
                encoding="utf-8",
            )
            runtime_before = runtime_ini.read_bytes()
            admin_hash = password_record("admin", "field-admin")
            cyh_hash = password_record("cyh", "field-cyh")
            with closing(sqlite3.connect(db)) as connection, connection:
                create_settings_schema(connection)
                unrelated = [
                    (
                        "global", "", f"SyntheticRuntime/Group{index // 100}",
                        f"Item{index:04d}", f"preserve-{index:04d}",
                        "string", 0, 0,
                    )
                    for index in range(8940)
                ]
                connection.executemany(
                    """
                    INSERT INTO settings(
                        scope_type, scope_id, module, key_name, value_text,
                        value_type, sensitive, encrypted
                    ) VALUES(?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    unrelated,
                )
                # The field database contains admin in both historical layouts;
                # identical decoded duplicates are intentional and safe.
                add_legacy_account(
                    connection, "admin", "admin", admin_hash,
                    flat=True, protect=True,
                    created_at="2026-05-07T00:00:00",
                    updated_at="2026-05-15T00:00:00",
                )
                add_legacy_account(
                    connection, "admin", "admin", admin_hash,
                    flat=False, protect=True,
                    created_at="2026-05-28T00:00:00",
                    updated_at="2026-05-20T00:00:00",
                )
                add_legacy_account(
                    connection, "cyh", "operator", cyh_hash,
                    flat=True, protect=True,
                )
                # A released v4 application could already have materialized the
                # same account under account/Profile while encrypt_new_values=1
                # still wrapped every field with the portable legacy enc:v1
                # codec.  v4 -> v5 must decode this exact mixed representation,
                # reconcile it with Accounts/Users, and normalize every Profile
                # field to portable plaintext storage.
                legacy_wrapped_profile = {
                    "PasswordHash": admin_hash,
                    "Role": "admin",
                    "MustChangePassword": "0",
                    "PasswordChangedAt": "2026-05-28T12:34:56+08:00",
                    "CreatedAt": "2026-06-05T00:00:00",
                    "UpdatedAt": "2026-06-01T00:00:00",
                }
                connection.execute(
                    "INSERT INTO meta(key, value) VALUES('encrypt_new_values', '1')"
                )
                for key, value in legacy_wrapped_profile.items():
                    insert_setting(
                        connection, "account", "admin", "Profile",
                        key, MIGRATOR["protect_legacy_text"](value),
                        value_type=(
                            "bool" if key == "MustChangePassword"
                            else "datetime" if key.endswith("At")
                            else "string"
                        ),
                        sensitive=1 if "Password" in key else 0,
                        encrypted=1,
                    )
                insert_setting(
                    connection, "global", "", "RuntimeSecrets", "ApiToken", "",
                    sensitive=1,
                )
            expected_unrelated = tuple(
                (f"SyntheticRuntime/Group{index // 100}", f"Item{index:04d}", f"preserve-{index:04d}")
                for index in range(8940)
            )
            backup = data / "ConfigStore.db.install-upgrade-test.dpapi.bak"

            MIGRATOR["migrate"](
                data, db, False, True, None, False,
                upgrade_backup_path=backup,
            )

            self.assertTrue(backup.is_file())
            self.assertEqual(runtime_ini.read_bytes(), runtime_before)
            with closing(sqlite3.connect(db)) as connection, connection:
                self.assertEqual(
                    connection.execute("PRAGMA integrity_check").fetchone(), ("ok",)
                )
                meta = dict(connection.execute(
                    "SELECT key, value FROM meta WHERE key IN "
                    "('schema_version', 'auth_semantic_version', 'auth_initialized')"
                ))
                self.assertEqual(meta, {
                    "schema_version": "5",
                    "auth_semantic_version": "2",
                    "auth_initialized": "1",
                })
                preserved = tuple(connection.execute(
                    """
                    SELECT module, key_name, value_text FROM settings
                    WHERE module LIKE 'SyntheticRuntime/%'
                    ORDER BY CAST(substr(key_name, 5) AS INTEGER)
                    """
                ))
                self.assertEqual(preserved, expected_unrelated)
                self.assertEqual(
                    connection.execute(
                        "SELECT COUNT(*) FROM settings WHERE value_text='runtime-only-marker'"
                    ).fetchone(),
                    (0,),
                )
                profiles = {
                    (scope_id, key): (value, encrypted)
                    for scope_id, key, value, encrypted in connection.execute(
                        """
                        SELECT scope_id, key_name, value_text, encrypted
                        FROM settings
                        WHERE scope_type='account' AND module='Profile'
                        """
                    )
                }
                expected_admin_profile = {
                    "PasswordHash": admin_hash,
                    "Role": "admin",
                    "MustChangePassword": "0",
                    "PasswordChangedAt": "2026-05-28T12:34:56+08:00",
                    # CreatedAt keeps the earliest valid legacy/target instant;
                    # UpdatedAt keeps the latest.
                    "CreatedAt": "2026-05-07T00:00:00",
                    "UpdatedAt": "2026-06-01T00:00:00",
                }
                for key, value in expected_admin_profile.items():
                    self.assertEqual(
                        profiles[("admin", key)],
                        (value, 0),
                        f"v4 wrapped Profile field was not normalized: {key}",
                    )
                    self.assertFalse(value.startswith("enc:v1:"))
                self.assertEqual(profiles[("cyh", "PasswordHash")], (cyh_hash, 0))
                self.assertEqual(profiles[("cyh", "Role")], ("operator", 0))
                self.assertEqual(
                    MIGRATOR["_decoded_scoped_value"](
                        connection, "account", "admin", "Profile", "CreatedAt"
                    ),
                    "2026-05-07T00:00:00",
                )
                self.assertEqual(
                    MIGRATOR["_decoded_scoped_value"](
                        connection, "account", "admin", "Profile", "UpdatedAt"
                    ),
                    "2026-06-01T00:00:00",
                )
                self.assertIsNone(
                    MIGRATOR["_decoded_scoped_value"](
                        connection, "account", "cyh", "Profile", "UpdatedAt"
                    )
                )
                self.assertEqual(
                    connection.execute(
                        """
                        SELECT COUNT(*) FROM settings
                        WHERE scope_type='global' AND scope_id='' AND (
                            module='Accounts/Users' OR module LIKE 'Accounts/Users/%'
                        )
                        """
                    ).fetchone(),
                    (0,),
                )
                stored_empty = connection.execute(
                    """
                    SELECT value_text, encrypted FROM settings
                    WHERE scope_type='global' AND scope_id=''
                      AND module='RuntimeSecrets' AND key_name='ApiToken'
                    """
                ).fetchone()
                self.assertIsNotNone(stored_empty)
                self.assertTrue(stored_empty[0].startswith("dpapi:user:v1:"))
                self.assertEqual(stored_empty[1], 1)
                self.assertEqual(
                    MIGRATOR["decode_stored_text"](
                        stored_empty[0], stored_empty[1],
                        MIGRATOR["protection_purpose"](
                            "global", "", "RuntimeSecrets", "ApiToken"
                        ),
                    ),
                    "",
                )

            before_verify = hashlib.sha256(db.read_bytes()).hexdigest()
            self.assertEqual(MIGRATOR["verify_current_database"](db), "none")
            self.assertEqual(hashlib.sha256(db.read_bytes()).hexdigest(), before_verify)

    def test_schema4_rejects_malformed_or_dpapi_profile_wrappers_atomically(self) -> None:
        for case in ("malformed-enc-v1", "dpapi-current-user"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                db = data / "ConfigStore.db"
                record = password_record("admin", "strict-wrapper-negative")
                with closing(sqlite3.connect(db)) as connection, connection:
                    create_settings_schema(connection)
                    add_legacy_account(
                        connection, "admin", "admin", record,
                        flat=True, protect=False,
                    )
                    if case == "malformed-enc-v1":
                        wrapped = "enc:v1:not-valid-base64:not-valid-base64"
                    else:
                        wrapped = MIGRATOR["protect_sensitive_text"](
                            record,
                            MIGRATOR["protection_purpose"](
                                "account", "admin", "Profile", "PasswordHash"
                            ),
                        )
                    insert_setting(
                        connection, "account", "admin", "Profile",
                        "PasswordHash", wrapped, sensitive=1, encrypted=1,
                    )
                    insert_setting(
                        connection, "account", "admin", "Profile",
                        "Role", "admin",
                    )
                before = db.read_bytes()
                backup = data / (
                    f"ConfigStore.db.install-upgrade-{case}.dpapi.bak"
                )
                with self.assertRaises(ValueError):
                    MIGRATOR["migrate_existing_database_to_current"](
                        db, data, True, upgrade_backup_path=backup
                    )
                self.assertEqual(db.read_bytes(), before)
                self.assertTrue(backup.is_file())

    def test_schema5_rejects_exact_enc_v1_encrypted_profile_atomically(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            with closing(sqlite3.connect(db)) as connection, connection:
                row = connection.execute(
                    """
                    SELECT value_text FROM settings
                    WHERE scope_type='account' AND scope_id='admin'
                      AND module='Profile' AND key_name='PasswordHash'
                    """
                ).fetchone()
                self.assertIsNotNone(row)
                connection.execute(
                    """
                    UPDATE settings SET value_text=?, encrypted=1
                    WHERE scope_type='account' AND scope_id='admin'
                      AND module='Profile' AND key_name='PasswordHash'
                    """,
                    (MIGRATOR["protect_legacy_text"](str(row[0])),),
                )
            before = db.read_bytes()
            backup = data / "ConfigStore.db.schema5-exact-wrapper.dpapi.bak"
            with self.assertRaisesRegex(ValueError, "portable plaintext storage"):
                MIGRATOR["migrate_existing_database_to_current"](
                    db, data, True, upgrade_backup_path=backup
                )
            self.assertEqual(db.read_bytes(), before)
            self.assertTrue(backup.is_file())

    def test_upgrade_backup_and_migration_keep_one_locked_identity(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            record = password_record("admin", "single-identity")
            with closing(sqlite3.connect(db)) as connection, connection:
                create_settings_schema(connection)
                add_legacy_account(
                    connection, "admin", "admin", record,
                    flat=True, protect=False,
                )
                connection.execute(
                    "CREATE TABLE layout_noise(id INTEGER PRIMARY KEY, value BLOB)"
                )
                connection.executemany(
                    "INSERT INTO layout_noise(value) VALUES(?)",
                    ((os.urandom(1024),) for _ in range(96)),
                )
                connection.execute("DROP TABLE layout_noise")
            original_bytes = db.read_bytes()
            replacement = data / "replacement.db"
            replacement.write_bytes(original_bytes)
            backup = data / "ConfigStore.db.single-identity.dpapi.bak"
            observed = {"replace_blocked": False}
            migration_globals = MIGRATOR[
                "migrate_existing_database_to_current"
            ].__globals__
            original_create = migration_globals[
                "_create_dpapi_database_backup_from_bytes"
            ]

            def create_and_probe(
                database_bytes: bytes,
                backup_path: Path,
            ) -> Path:
                result = original_create(database_bytes, backup_path)
                with self.assertRaises(OSError):
                    os.replace(replacement, db)
                observed["replace_blocked"] = True
                return result

            with mock.patch.dict(
                migration_globals,
                {"_create_dpapi_database_backup_from_bytes": create_and_probe},
            ):
                self.assertTrue(
                    MIGRATOR["migrate_existing_database_to_current"](
                        db, data, True, upgrade_backup_path=backup
                    )
                )
            self.assertEqual(
                observed,
                {"replace_blocked": True},
            )
            protected_snapshot = MIGRATOR[
                "_read_dpapi_database_backup"
            ](backup)
            self.assertNotEqual(
                hashlib.sha256(original_bytes).digest(),
                hashlib.sha256(protected_snapshot).digest(),
                "fixture does not exercise the old raw-file SHA mismatch",
            )
            original_snapshot = sqlite3.connect(":memory:")
            try:
                original_snapshot.deserialize(protected_snapshot)
                self.assertEqual(
                    original_snapshot.execute(
                        "SELECT value FROM meta WHERE key='schema_version'"
                    ).fetchone(),
                    ("4",),
                )
            finally:
                original_snapshot.close()
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key='schema_version'"
                    ).fetchone(),
                    (MIGRATOR["SCHEMA_VERSION"],),
                )

    def test_existing_upgrade_rejects_preexisting_and_inflight_hardlinks(self) -> None:
        for attack in ("preexisting", "during-backup"):
            with self.subTest(attack=attack), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                data = root / "Data"
                data.mkdir()
                db = data / "ConfigStore.db"
                with closing(sqlite3.connect(db)) as connection, connection:
                    create_settings_schema(connection)
                    add_legacy_account(
                        connection,
                        "admin",
                        "admin",
                        password_record("admin", f"hardlink-{attack}"),
                        flat=True,
                        protect=False,
                    )
                original = db.read_bytes()
                external_alias = root / "external-configstore-alias.db"
                backup = data / f"ConfigStore.db.{attack}.dpapi.bak"
                migration_globals = MIGRATOR[
                    "migrate_existing_database_to_current"
                ].__globals__
                original_backup_create = migration_globals[
                    "_create_dpapi_database_backup_from_bytes"
                ]

                if attack == "preexisting":
                    os.link(db, external_alias)
                    patch_values: dict[str, object] = {}
                else:
                    def create_backup_then_link(
                        database_bytes: bytes,
                        backup_path: Path,
                    ) -> Path:
                        result = original_backup_create(database_bytes, backup_path)
                        os.link(db, external_alias)
                        return result

                    patch_values = {
                        "_create_dpapi_database_backup_from_bytes": (
                            create_backup_then_link
                        )
                    }

                with (
                    mock.patch.dict(migration_globals, patch_values),
                    self.assertRaisesRegex(ValueError, "exactly one hard link"),
                ):
                    MIGRATOR["migrate_existing_database_to_current"](
                        db, data, True, upgrade_backup_path=backup
                    )

                self.assertEqual(db.read_bytes(), original)
                self.assertEqual(external_alias.read_bytes(), original)
                self.assertEqual(os.stat(db).st_nlink, 2)
                self.assertEqual(backup.exists(), attack == "during-backup")
                self.assertFalse(any(
                    Path(str(db) + suffix).exists()
                    for suffix in ("-journal", "-wal", "-shm")
                ))
                with closing(sqlite3.connect(db)) as connection:
                    self.assertEqual(
                        connection.execute(
                            "SELECT value FROM meta WHERE key='schema_version'"
                        ).fetchone(),
                        ("4",),
                    )

    def test_installer_create_builds_only_in_memory_before_atomic_publish(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            (data / "Accounts.ini").write_text(
                "[Users/admin]\n"
                f"PasswordHash={password_record('admin', 'reserve-swap')}\n"
                "Role=admin\nCreatedAt=2026-01-02T03:04:05\n",
                encoding="utf-8",
            )
            staging = data / (
                ".ConfigStore.db.install-create-"
                "abcdef0123456789abcdef0123456789.tmp"
            )
            migration_globals = MIGRATOR["migrate"].__globals__
            sqlite_module = migration_globals["sqlite3"]
            actual_connect = sqlite_module.connect
            opened: list[str] = []

            def memory_only_connect(
                database: object, *arguments: object, **keywords: object
            ) -> sqlite3.Connection:
                opened.append(str(database))
                if str(database) != ":memory:":
                    raise AssertionError(
                        f"migration opened an on-disk SQLite path: {database}"
                    )
                return actual_connect(database, *arguments, **keywords)

            with mock.patch.object(
                sqlite_module, "connect", memory_only_connect
            ):
                MIGRATOR["migrate"](
                    data,
                    staging,
                    False,
                    True,
                    None,
                    False,
                    defer_credential_scrub=True,
                    installer_staging=True,
                )

            self.assertTrue(opened)
            self.assertEqual(set(opened), {":memory:"})
            self.assertTrue(staging.is_file())
            self.assertEqual(
                MIGRATOR["verify_current_database"](staging), "pending"
            )
            self.assertFalse(any(
                Path(str(staging) + suffix).exists()
                for suffix in ("-journal", "-wal", "-shm")
            ))
            self.assertFalse(any(data.glob("*.dpapi.bak")))

    def test_normal_migration_has_no_plaintext_staging_file(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            (data / "Runtime.ini").write_text(
                "[Runtime]\nPassCount=5\n",
                encoding="utf-8",
            )
            db = data / "ConfigStore.db"
            with mock.patch.object(
                tempfile,
                "mkstemp",
                side_effect=AssertionError(
                    "migration must not create a plaintext staging file"
                ),
            ) as mkstemp_mock:
                MIGRATOR["migrate"](
                    data, db, False, True, None, False
                )

            mkstemp_mock.assert_not_called()
            self.assertTrue(db.is_file())
            self.assertFalse(any(data.glob(".ConfigStore.db.staging-*.tmp")))
            self.assertEqual(
                MIGRATOR["verify_current_database"](db), "none"
            )

    def test_migration_binds_each_legacy_source_before_normalization(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            runtime = data / "Runtime.ini"
            runtime.write_text(
                "[Runtime]\nPassCount=5\n", encoding="utf-8"
            )
            outside = root / "privileged.ini"
            outside.write_text(
                "[Runtime]\nPassCount=999\n", encoding="utf-8"
            )
            replacement = root / "replacement-link.ini"
            weave = data / "WeaveDate.txt"
            weave.write_text("original weld text", encoding="utf-8")
            outside_text = root / "privileged-weave.txt"
            outside_text.write_text("external weld text", encoding="utf-8")
            replacement_text = root / "replacement-link.txt"
            try:
                os.symlink(outside, replacement)
                os.symlink(outside_text, replacement_text)
            except OSError:
                replacement.unlink(missing_ok=True)
                replacement_text.unlink(missing_ok=True)
                os.link(outside, replacement)
                os.link(outside_text, replacement_text)
            db = data / "ConfigStore.db"
            function_globals = MIGRATOR["migrate"].__globals__
            actual_normalize = function_globals["normalize_data_path"]
            attacked: set[str] = set()
            replacements = {
                "Runtime.ini": (replacement, runtime),
                "WeaveDate.txt": (replacement_text, weave),
            }

            def normalize_then_swap(path: Path, root_path: Path) -> str:
                result = actual_normalize(path, root_path)
                if path.name in replacements and path.name not in attacked:
                    source_replacement, destination = replacements[path.name]
                    with self.assertRaises(OSError):
                        os.replace(source_replacement, destination)
                    attacked.add(path.name)
                return result

            with mock.patch.dict(
                function_globals,
                {"normalize_data_path": normalize_then_swap},
            ):
                MIGRATOR["migrate"](
                    data, db, False, False, None, False
                )
            self.assertEqual(attacked, {"Runtime.ini", "WeaveDate.txt"})
            self.assertEqual(
                runtime.read_text(encoding="utf-8"),
                "[Runtime]\nPassCount=5\n",
            )
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    MIGRATOR["read_ini_value"](
                        connection,
                        "Data/Runtime.ini",
                        "Runtime",
                        "PassCount",
                    ),
                    "5",
                )
                identity = MIGRATOR["build_scoped_file_identity"](
                    "Data/WeaveDate.txt"
                )
                self.assertEqual(
                    connection.execute(
                        "SELECT value_text FROM settings WHERE "
                        "scope_type=? AND scope_id=? AND module=? AND key_name=?",
                        (
                            identity["scope_type"],
                            identity["scope_id"],
                            identity["module"],
                            identity["key_name"],
                        ),
                    ).fetchone(),
                    ("original weld text",),
                )

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            outside = root / "external-runtime.ini"
            outside.write_text(
                "[Runtime]\nPassCount=999\n", encoding="utf-8"
            )
            os.link(outside, data / "Runtime.ini")
            db = data / "ConfigStore.db"
            with self.assertRaisesRegex(ValueError, "exactly one hard link"):
                MIGRATOR["migrate"](
                    data, db, False, False, None, False
                )
            self.assertFalse(db.exists())

        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            outside = root / "external-weave.txt"
            outside.write_text("external weld text", encoding="utf-8")
            os.link(outside, data / "WeaveDate.txt")
            db = data / "ConfigStore.db"
            with self.assertRaisesRegex(ValueError, "exactly one hard link"):
                MIGRATOR["migrate"](
                    data, db, False, False, None, False
                )
            self.assertFalse(db.exists())

    def test_migration_rejects_late_legacy_source_create_or_rename(self) -> None:
        for attack in ("create", "rename"):
            with self.subTest(attack=attack), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                original = data / "Runtime.ini"
                original.write_text(
                    "[Runtime]\nPassCount=5\n", encoding="utf-8"
                )
                db = data / "ConfigStore.db"
                migration_globals = MIGRATOR["migrate"].__globals__
                actual_assert = migration_globals[
                    "_assert_legacy_source_snapshot_unchanged"
                ]
                injected = False

                def mutate_then_assert(
                    root: Path,
                    expected: tuple[object, ...],
                ) -> None:
                    nonlocal injected
                    if not injected:
                        injected = True
                        if attack == "create":
                            (data / "Late.ini").write_text(
                                "[Runtime]\nPassCount=999\n", encoding="utf-8"
                            )
                        else:
                            original.rename(data / "Renamed.ini")
                    actual_assert(root, expected)

                with (
                    mock.patch.dict(
                        migration_globals,
                        {
                            "_assert_legacy_source_snapshot_unchanged": (
                                mutate_then_assert
                            )
                        },
                    ),
                    self.assertRaisesRegex(ValueError, "inventory changed"),
                ):
                    MIGRATOR["migrate"](
                        data, db, False, False, None, False
                    )
                self.assertTrue(injected)
                self.assertFalse(db.exists())

    def test_source_guard_is_bound_to_atomic_publication_and_rolls_back(self) -> None:
        for attack in (
            "atomic-entry-change",
            "commit-window-create",
            "retire-window-create",
        ):
            with self.subTest(attack=attack), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                runtime = data / "Runtime.ini"
                runtime.write_text(
                    "[Runtime]\nPassCount=5\n", encoding="utf-8"
                )
                db = data / "ConfigStore.db"
                migration_globals = MIGRATOR["migrate"].__globals__

                if attack == "atomic-entry-change":
                    actual_atomic = migration_globals["_atomic_replace_bytes"]
                    injected = False

                    def mutate_at_atomic_entry(
                        path: Path,
                        content: bytes,
                        **keywords: object,
                    ) -> None:
                        nonlocal injected
                        if not injected:
                            runtime.write_text(
                                "[Runtime]\nPassCount=999\n",
                                encoding="utf-8",
                            )
                            injected = True
                        actual_atomic(path, content, **keywords)

                    patches = {
                        "_atomic_replace_bytes": mutate_at_atomic_entry
                    }
                elif attack == "commit-window-create":
                    actual_rename = migration_globals["_win32_rename_handle"]
                    injected = False

                    def create_during_commit(
                        kernel32: object,
                        handle: int,
                        destination: Path,
                        replace_existing: bool,
                        label: str,
                    ) -> None:
                        nonlocal injected
                        if label == "new destination" and not injected:
                            (data / "Late.ini").write_text(
                                "[Runtime]\nPassCount=999\n",
                                encoding="utf-8",
                            )
                            injected = True
                        actual_rename(
                            kernel32,
                            handle,
                            destination,
                            replace_existing,
                            label,
                        )

                    patches = {"_win32_rename_handle": create_during_commit}
                else:
                    actual_delete = migration_globals["_win32_delete_handle"]
                    injected = False

                    def create_during_retirement(
                        kernel32: object,
                        handle: int,
                        label: str,
                    ) -> None:
                        nonlocal injected
                        if (
                            label.startswith("SQLite sidecar sentinel")
                            and not injected
                        ):
                            (data / "Late.ini").write_text(
                                "[Runtime]\nPassCount=999\n",
                                encoding="utf-8",
                            )
                            injected = True
                        actual_delete(kernel32, handle, label)

                    patches = {"_win32_delete_handle": create_during_retirement}

                with (
                    mock.patch.dict(migration_globals, patches),
                    self.assertRaisesRegex(ValueError, "source inventory changed"),
                ):
                    MIGRATOR["migrate"](
                        data, db, False, False, None, False
                    )
                self.assertTrue(injected)
                self.assertFalse(db.exists())
                self.assertFalse(any(data.glob(".*atomic-transaction-v1")))

    def test_existing_v4_and_v5_upgrades_bind_disk_source_inventory(self) -> None:
        for version in ("4", "5-auth1"):
            with self.subTest(version=version), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                db = data / "ConfigStore.db"
                if version == "4":
                    with closing(sqlite3.connect(db)) as connection, connection:
                        create_settings_schema(connection, "4")
                        add_legacy_account(
                            connection,
                            "admin",
                            "admin",
                            password_record("admin", "late-input-v4"),
                            flat=True,
                            protect=False,
                        )
                    expected_version = "4"
                else:
                    self._create_auth1_database(db, "complete", [])
                    expected_version = "5"
                backup = data / f"ConfigStore.db.{version}.dpapi.bak"
                migration_globals = MIGRATOR[
                    "migrate_existing_database_to_current"
                ].__globals__
                actual_atomic = migration_globals["_atomic_replace_bytes"]
                injected = False

                def add_late_input(
                    path: Path,
                    content: bytes,
                    **keywords: object,
                ) -> None:
                    nonlocal injected
                    if not injected:
                        (data / "Late.ini").write_text(
                            "[Runtime]\nPassCount=999\n", encoding="utf-8"
                        )
                        injected = True
                    actual_atomic(path, content, **keywords)

                with (
                    mock.patch.dict(
                        migration_globals,
                        {"_atomic_replace_bytes": add_late_input},
                    ),
                    self.assertRaisesRegex(ValueError, "source inventory changed"),
                ):
                    MIGRATOR["migrate_existing_database_to_current"](
                        db,
                        data,
                        True,
                        upgrade_backup_path=backup,
                    )
                self.assertTrue(injected)
                with closing(sqlite3.connect(db)) as connection:
                    self.assertEqual(
                        connection.execute(
                            "SELECT value FROM meta WHERE key='schema_version'"
                        ).fetchone(),
                        (expected_version,),
                    )

    def test_text_source_case_is_canonical_and_collisions_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            text_file = data / "WeldPara.TXT"
            text_file.write_text("mixed-case weld text", encoding="utf-8")
            db = data / "ConfigStore.db"
            MIGRATOR["migrate"](
                data, db, False, False, None, False
            )
            identity = MIGRATOR["build_scoped_file_identity"](
                "Data/WeldPara.txt"
            )
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value_text FROM settings WHERE "
                        "scope_type=? AND scope_id=? AND module=? AND key_name=?",
                        (
                            identity["scope_type"],
                            identity["scope_id"],
                            identity["module"],
                            identity["key_name"],
                        ),
                    ).fetchone(),
                    ("mixed-case weld text",),
                )

        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            canonical = data / "WeldPara.txt"
            canonical.write_text("collision", encoding="utf-8")
            case_alias = data / "WELDPARA.TXT"
            path_type = type(data)
            actual_rglob = path_type.rglob

            def colliding_rglob(path: Path, pattern: str):
                if path == data and pattern == "*":
                    return iter((canonical, case_alias))
                return actual_rglob(path, pattern)

            with (
                mock.patch.object(path_type, "rglob", colliding_rglob),
                self.assertRaisesRegex(ValueError, "path collision"),
            ):
                MIGRATOR["migrate"](
                    data,
                    data / "ConfigStore.db",
                    False,
                    False,
                    None,
                    False,
                )
            self.assertFalse((data / "ConfigStore.db").exists())

    def test_scrub_finalize_rejects_database_hardlink_before_ini_change(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            root = Path(text)
            data = root / "Data"
            data.mkdir()
            account_ini = data / "Accounts.ini"
            account_ini.write_text(
                "[Users/admin]\n"
                f"PasswordHash={password_record('admin', 'finalize-link')}\n"
                "Role=admin\n",
                encoding="utf-8",
            )
            manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                data, None
            )
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "pending", manifest)
            ini_before = account_ini.read_bytes()
            db_before = db.read_bytes()
            external_alias = root / "external-finalize-alias.db"
            os.link(db, external_alias)

            with self.assertRaisesRegex(ValueError, "exactly one hard link"):
                MIGRATOR["_finish_legacy_credential_scrub"](
                    data, db, None, True
                )

            self.assertEqual(account_ini.read_bytes(), ini_before)
            self.assertEqual(db.read_bytes(), db_before)
            self.assertEqual(external_alias.read_bytes(), db_before)
            with closing(sqlite3.connect(db)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key=?",
                        (MIGRATOR["SCRUB_STATE_KEY"],),
                    ).fetchone(),
                    ("pending",),
                )
            self.assertFalse(any(data.glob(".*.credential-scrub-*.tmp")))

    def test_backup_verify_blocks_path_swap_while_handles_are_locked(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
                connection.execute("INSERT INTO sample VALUES('stable')")
            backup = data / "ConfigStore.db.verify-lock.dpapi.bak"
            MIGRATOR["create_dpapi_database_backup"](db, backup)
            replacement_db = data / "replacement-source.db"
            replacement_backup = data / "replacement-backup.dpapi.bak"
            renamed_data = data.with_name("RenamedData")
            replacement_db.write_bytes(db.read_bytes())
            replacement_backup.write_bytes(backup.read_bytes())
            locked_type = MIGRATOR["_LockedWin32ReadFile"]
            original_read = locked_type.read_bytes
            attacked = False

            def read_with_path_swap_probe(locked: object) -> bytes:
                nonlocal attacked
                if not attacked:
                    with self.assertRaises(OSError):
                        os.replace(replacement_db, db)
                    with self.assertRaises(OSError):
                        os.replace(replacement_backup, backup)
                    with self.assertRaises(OSError):
                        os.replace(data, renamed_data)
                    attacked = True
                return original_read(locked)

            with mock.patch.object(
                locked_type, "read_bytes", read_with_path_swap_probe
            ):
                MIGRATOR["verify_dpapi_database_backup_against"](backup, db)
            self.assertTrue(attacked)

            hardlink = data / "ConfigStore.db.hardlink.dpapi.bak"
            os.link(db, hardlink)
            with self.assertRaisesRegex(ValueError, "hard link"):
                MIGRATOR["verify_dpapi_database_backup_against"](hardlink, db)
            hardlink.unlink()

            sidecar = Path(str(db) + "-wal")
            injected = False

            def read_with_sidecar_probe(locked: object) -> bytes:
                nonlocal injected
                if not injected:
                    sidecar.write_bytes(b"transient sidecar sentinel")
                    injected = True
                return original_read(locked)

            try:
                with mock.patch.object(
                    locked_type, "read_bytes", read_with_sidecar_probe
                ):
                    with self.assertRaisesRegex(ValueError, "sidecar"):
                        MIGRATOR["verify_dpapi_database_backup_against"](
                            backup, db
                        )
                self.assertTrue(injected)
            finally:
                sidecar.unlink(missing_ok=True)

    def test_failure_rolls_back_incomplete_and_target_conflict(self) -> None:
        cases = (
            "empty", "no-admin", "incomplete", "target-conflict",
            "target-field-case", "invalid-created", "invalid-target-time",
            "target-module-case",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                db = data / "ConfigStore.db"
                with closing(sqlite3.connect(db)) as connection, connection:
                    create_settings_schema(connection)
                    insert_setting(
                        connection, "global", "", "Synthetic", "KeepMe", "unchanged"
                    )
                    if case == "no-admin":
                        add_legacy_account(
                            connection, "cyh", "operator",
                            password_record("cyh", "operator-only"),
                            flat=True, protect=False,
                        )
                    elif case == "incomplete":
                        insert_setting(
                            connection, "global", "", "Accounts/Users",
                            "admin/PasswordHash", password_record("admin", "source"),
                            sensitive=1,
                        )
                    elif case == "invalid-created":
                        add_legacy_account(
                            connection, "admin", "admin",
                            password_record("admin", "source-admin"),
                            flat=True, protect=False,
                            created_at="2026-02-30T00:00:00",
                        )
                    elif case in {
                        "target-conflict", "target-field-case",
                        "invalid-target-time", "target-module-case",
                    }:
                        admin_record = password_record("admin", "source-admin")
                        cyh_record = password_record("cyh", "source-cyh")
                        add_legacy_account(
                            connection, "admin", "admin",
                            admin_record,
                            flat=True, protect=False,
                        )
                        add_legacy_account(
                            connection, "cyh", "operator",
                            cyh_record,
                            flat=True, protect=False,
                        )
                        if case in {"target-conflict", "target-field-case"}:
                            insert_setting(
                                connection, "account", "cyh", "Profile",
                                "PasswordHash", password_record(
                                    "cyh", "different-target"
                                ),
                                sensitive=1,
                            )
                            insert_setting(
                                connection, "account", "cyh", "Profile",
                                "role" if case == "target-field-case" else "Role",
                                "engineer",
                            )
                            insert_setting(
                                connection, "account", "cyh", "Profile",
                                "MustChangePassword", "0", value_type="bool",
                                sensitive=1,
                            )
                        elif case == "invalid-target-time":
                            for key, value, value_type in (
                                ("PasswordHash", admin_record, "string"),
                                ("Role", "admin", "string"),
                                ("CreatedAt", "not-an-iso-time", "datetime"),
                            ):
                                insert_setting(
                                    connection, "account", "admin", "Profile",
                                    key, value, value_type=value_type,
                                    sensitive=1 if key == "PasswordHash" else 0,
                                )
                        else:
                            insert_setting(
                                connection, "account", "cyh", "profile",
                                "Role", "operator",
                            )
                before = database_snapshot(db)
                before_bytes = db.read_bytes()
                backup = data / f"ConfigStore.db.install-upgrade-{case}.dpapi.bak"
                with self.assertRaises(ValueError):
                    MIGRATOR["migrate_existing_database_to_current"](
                        db, data, True, upgrade_backup_path=backup
                    )
                self.assertEqual(database_snapshot(db), before)
                self.assertEqual(db.read_bytes(), before_bytes)
                self.assertTrue(backup.is_file())
                with closing(sqlite3.connect(db)) as connection, connection:
                    self.assertEqual(
                        connection.execute("PRAGMA integrity_check").fetchone(), ("ok",)
                    )

    def test_existing_backup_is_never_overwritten_or_deleted(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            with closing(sqlite3.connect(db)) as connection, connection:
                create_settings_schema(connection)
            backup = data / "ConfigStore.db.install-upgrade-existing.dpapi.bak"
            backup.write_bytes(b"pre-existing sentinel")
            with self.assertRaises(FileExistsError):
                MIGRATOR["migrate_existing_database_to_current"](
                    db, data, True, upgrade_backup_path=backup
                )
            self.assertEqual(backup.read_bytes(), b"pre-existing sentinel")

    def test_deferred_new_database_scrub_and_preflight_failure(self) -> None:
        for mutate_after_migration in (False, True):
            with self.subTest(mutate=mutate_after_migration), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                account_ini = data / "Accounts.ini"
                process_ini = data / "Process.ini"
                admin_hash = password_record("admin", "deferred")
                account_ini.write_text(
                    "[Users/admin]\n"
                    f"PasswordHash={admin_hash}\nRole=admin\n"
                    "CreatedAt=2026-01-02T03:04:05\n",
                    encoding="utf-8",
                )
                process_ini.write_text(
                    "[Runtime]\nApiToken=deferred-secret\nPassCount=5\n",
                    encoding="utf-8",
                )
                account_before = account_ini.read_bytes()
                process_before = process_ini.read_bytes()
                db = data / "ConfigStore.db"
                MIGRATOR["migrate"](
                    data, db, False, True, None, False,
                    defer_credential_scrub=True,
                )
                self.assertEqual(account_ini.read_bytes(), account_before)
                self.assertEqual(process_ini.read_bytes(), process_before)
                with closing(sqlite3.connect(db)) as connection, connection:
                    pending = dict(connection.execute(
                        "SELECT key, value FROM meta WHERE key IN (?, ?)",
                        (
                            MIGRATOR["SCRUB_STATE_KEY"],
                            MIGRATOR["SCRUB_MANIFEST_KEY"],
                        ),
                    ))
                self.assertEqual(pending[MIGRATOR["SCRUB_STATE_KEY"]], "pending")
                self.assertEqual(MIGRATOR["verify_current_database"](db), "pending")

                if mutate_after_migration:
                    process_ini.write_bytes(process_before + b"ChangedAfterMigration=1\n")
                    account_pre_finalize = account_ini.read_bytes()
                    process_pre_finalize = process_ini.read_bytes()
                    with self.assertRaisesRegex(ValueError, "changed after migration"):
                        MIGRATOR["migrate"](
                            data, db, False, True, None, False,
                            scrub_legacy_credentials=True,
                        )
                    self.assertEqual(account_ini.read_bytes(), account_pre_finalize)
                    self.assertEqual(process_ini.read_bytes(), process_pre_finalize)
                    self.assertEqual(MIGRATOR["verify_current_database"](db), "pending")
                else:
                    MIGRATOR["migrate"](
                        data, db, False, True, None, False,
                        scrub_legacy_credentials=True,
                    )
                    self.assertNotIn(b"PasswordHash=", account_ini.read_bytes())
                    self.assertNotIn(b"ApiToken=", process_ini.read_bytes())
                    self.assertEqual(MIGRATOR["verify_current_database"](db), "complete")

    def test_installer_create_staging_preserves_real_data_and_is_pending(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            accounts = data / "Accounts.ini"
            process = data / "Process.ini"
            accounts.write_text(
                "[Users/admin]\n"
                f"PasswordHash={password_record('admin', 'installer-create')}\n"
                "Role=admin\nCreatedAt=2026-01-02T03:04:05\n",
                encoding="utf-8",
            )
            process.write_text(
                "[Runtime]\nApiToken=installer-create-secret\nPassCount=5\n",
                encoding="utf-8",
            )
            source_before = {
                path.name: path.read_bytes() for path in (accounts, process)
            }
            final_db = data / "ConfigStore.db"
            staging = data / (
                ".ConfigStore.db.install-create-"
                "0123456789abcdef0123456789abcdef.tmp"
            )

            result = self._run_cli(
                "--source", data,
                "--db", staging,
                "--encrypt",
                "--defer-credential-scrub",
                "--installer-staging",
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue(staging.is_file())
            self.assertFalse(final_db.exists())
            self.assertEqual(
                {path.name: path.read_bytes() for path in (accounts, process)},
                source_before,
            )
            self.assertEqual(MIGRATOR["verify_current_database"](staging), "pending")
            self._assert_cli_pending(staging)
            with closing(sqlite3.connect(staging)) as connection:
                provenance = dict(connection.execute(
                    "SELECT key, value FROM meta WHERE key IN (?, ?)",
                    (MIGRATOR["SCRUB_STATE_KEY"], MIGRATOR["SCRUB_MANIFEST_KEY"]),
                ))
            self.assertEqual(provenance[MIGRATOR["SCRUB_STATE_KEY"]], "pending")
            manifest = MIGRATOR["parse_legacy_credential_scrub_manifest"](
                provenance[MIGRATOR["SCRUB_MANIFEST_KEY"]]
            )
            self.assertEqual({item["path"] for item in manifest}, {
                "Accounts.ini", "Process.ini",
            })
            self.assertFalse(any(
                Path(str(staging) + suffix).exists()
                for suffix in ("-journal", "-wal", "-shm")
            ))

    def test_installer_upgrade_staging_preserves_final_db_and_sources(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            final_db = data / "ConfigStore.db"
            admin_record = password_record("admin", "installer-upgrade")
            with closing(sqlite3.connect(final_db)) as connection, connection:
                create_settings_schema(connection)
                add_legacy_account(
                    connection,
                    "admin",
                    "admin",
                    admin_record,
                    flat=True,
                    protect=True,
                )
            runtime_ini = data / "CorrugatedSheetPointCloudEctration.ini"
            runtime_ini.write_text(
                "ifupright=true\nSave_File_Name=installer-upgrade-runtime\n",
                encoding="utf-8",
            )
            final_before = final_db.read_bytes()
            source_before = runtime_ini.read_bytes()
            staging = data / (
                ".ConfigStore.db.install-upgrade-"
                "fedcba9876543210fedcba9876543210.tmp"
            )
            staging.write_bytes(final_before)
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )

            result = self._run_cli(
                "--source", data,
                "--db", staging,
                "--encrypt",
                "--defer-credential-scrub",
                "--installer-staging",
                "--upgrade-backup", backup,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(final_db.read_bytes(), final_before)
            self.assertEqual(runtime_ini.read_bytes(), source_before)
            self.assertNotEqual(staging.read_bytes(), final_before)
            self.assertTrue(backup.is_file())
            restored = data / "restored-upgrade-backup.tmp"
            restored.write_bytes(MIGRATOR["_read_dpapi_database_backup"](backup))
            with closing(sqlite3.connect(restored)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key='schema_version'"
                    ).fetchone(),
                    ("4",),
                )
                self.assertEqual(
                    connection.execute(
                        "SELECT COUNT(*) FROM settings WHERE module LIKE 'Accounts/Users%'"
                    ).fetchone(),
                    (3,),
                )
            restored.unlink()
            self.assertEqual(MIGRATOR["verify_current_database"](staging), "pending")
            self._assert_cli_pending(staging)
            with closing(sqlite3.connect(staging)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key='schema_version'"
                    ).fetchone(),
                    ("5",),
                )
                manifest_text = connection.execute(
                    "SELECT value FROM meta WHERE key=?",
                    (MIGRATOR["SCRUB_MANIFEST_KEY"],),
                ).fetchone()[0]
            manifest = MIGRATOR["parse_legacy_credential_scrub_manifest"](
                manifest_text
            )
            self.assertEqual(manifest, [])

    def test_installer_auth1_complete_staging_preserves_complete_state(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            runtime_ini = data / "CorrugatedSheetPointCloudEctration.ini"
            runtime_ini.write_text(
                "ifupright=true\nSave_File_Name=auth1-complete-runtime\n",
                encoding="utf-8",
            )
            final_db = data / "ConfigStore.db"
            self._create_auth1_database(final_db, "complete", [])
            final_before = final_db.read_bytes()
            source_before = runtime_ini.read_bytes()
            staging = data / (
                ".ConfigStore.db.install-upgrade-"
                "cccccccccccccccccccccccccccccccc.tmp"
            )
            staging.write_bytes(final_before)
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )

            result = self._run_cli(
                "--source", data,
                "--db", staging,
                "--encrypt",
                "--defer-credential-scrub",
                "--installer-staging",
                "--upgrade-backup", backup,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("no installer finalization is required", result.stdout)
            self.assertEqual(final_db.read_bytes(), final_before)
            self.assertEqual(runtime_ini.read_bytes(), source_before)
            self.assertTrue(backup.is_file())
            self.assertEqual(MIGRATOR["verify_current_database"](staging), "complete")
            self._assert_cli_complete(staging)
            with closing(sqlite3.connect(staging)) as connection:
                meta = dict(connection.execute(
                    "SELECT key, value FROM meta WHERE key IN "
                    "('auth_semantic_version', ?, ?)",
                    (MIGRATOR["SCRUB_STATE_KEY"], MIGRATOR["SCRUB_MANIFEST_KEY"]),
                ))
            self.assertEqual(meta["auth_semantic_version"], "2")
            self.assertEqual(meta[MIGRATOR["SCRUB_STATE_KEY"]], "complete")
            self.assertEqual(
                MIGRATOR["parse_legacy_credential_scrub_manifest"](
                    meta[MIGRATOR["SCRUB_MANIFEST_KEY"]]
                ),
                [],
            )
            before_verify = self._tree_snapshot(data)
            installer_verify = self._verify_installer_cli(data, staging)
            self.assertEqual(
                installer_verify.returncode,
                0,
                installer_verify.stdout + installer_verify.stderr,
            )
            self.assertIn("(scrub=complete)", installer_verify.stdout)
            self.assertEqual(self._tree_snapshot(data), before_verify)
            restored = data / "auth1-complete-backup.tmp"
            restored.write_bytes(MIGRATOR["_read_dpapi_database_backup"](backup))
            with closing(sqlite3.connect(restored)) as connection:
                self.assertEqual(
                    connection.execute(
                        "SELECT value FROM meta WHERE key='auth_semantic_version'"
                    ).fetchone(),
                    ("1",),
                )

    def test_installer_auth1_pending_accepts_partial_after_state(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            first = data / "First.ini"
            second = data / "Second.ini"
            first.write_text(
                "[Remote]\nApiToken=first-secret\nKeepMe=1\n",
                encoding="utf-8",
            )
            second.write_text(
                "[Remote]\nPassword=second-secret\nKeepMe=2\n",
                encoding="utf-8",
            )
            stored_manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                data, None
            )
            self.assertEqual(len(stored_manifest), 2)
            final_db = data / "ConfigStore.db"
            self._create_auth1_database(final_db, "pending", stored_manifest)
            first_sanitized, _removed = MIGRATOR["_sanitized_legacy_ini_bytes"](
                first, None
            )
            first.write_bytes(first_sanitized)
            source_before = {
                first.name: first.read_bytes(), second.name: second.read_bytes()
            }
            final_before = final_db.read_bytes()
            staging = data / (
                ".ConfigStore.db.install-upgrade-"
                "dddddddddddddddddddddddddddddddd.tmp"
            )
            staging.write_bytes(final_before)
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )

            result = self._run_cli(
                "--source", data,
                "--db", staging,
                "--encrypt",
                "--defer-credential-scrub",
                "--installer-staging",
                "--upgrade-backup", backup,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("verified pending provenance", result.stdout)
            self.assertEqual(final_db.read_bytes(), final_before)
            self.assertEqual(
                {first.name: first.read_bytes(), second.name: second.read_bytes()},
                source_before,
            )
            self.assertTrue(backup.is_file())
            self.assertEqual(MIGRATOR["verify_current_database"](staging), "pending")
            self._assert_cli_pending(staging)
            with closing(sqlite3.connect(staging)) as connection:
                meta = dict(connection.execute(
                    "SELECT key, value FROM meta WHERE key IN "
                    "('auth_semantic_version', ?, ?)",
                    (MIGRATOR["SCRUB_STATE_KEY"], MIGRATOR["SCRUB_MANIFEST_KEY"]),
                ))
            self.assertEqual(meta["auth_semantic_version"], "2")
            self.assertEqual(meta[MIGRATOR["SCRUB_STATE_KEY"]], "pending")
            self.assertEqual(
                MIGRATOR["parse_legacy_credential_scrub_manifest"](
                    meta[MIGRATOR["SCRUB_MANIFEST_KEY"]]
                ),
                stored_manifest,
            )
            before_verify = self._tree_snapshot(data)
            installer_verify = self._verify_installer_cli(data, staging)
            self.assertEqual(
                installer_verify.returncode,
                0,
                installer_verify.stdout + installer_verify.stderr,
            )
            self.assertIn("(scrub=pending)", installer_verify.stdout)
            self.assertEqual(self._tree_snapshot(data), before_verify)

    def test_installer_auth1_pending_rejects_unknown_or_changed_sources(self) -> None:
        for case in ("unknown", "changed"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                known = data / "Known.ini"
                known.write_text(
                    "[Remote]\nApiToken=known-secret\nKeepMe=1\n",
                    encoding="utf-8",
                )
                stored_manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                    data, None
                )
                final_db = data / "ConfigStore.db"
                self._create_auth1_database(final_db, "pending", stored_manifest)
                if case == "unknown":
                    (data / "Unknown.ini").write_text(
                        "[Remote]\nPassword=new-secret\n",
                        encoding="utf-8",
                    )
                else:
                    known.write_bytes(known.read_bytes() + b"Changed=1\n")
                source_before = {
                    path.name: path.read_bytes() for path in data.glob("*.ini")
                }
                final_before = final_db.read_bytes()
                staging = data / (
                    ".ConfigStore.db.install-upgrade-"
                    "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee.tmp"
                )
                staging.write_bytes(final_before)
                staging_before = staging.read_bytes()
                backup = staging.with_name(
                    staging.name + ".install-backup.dpapi.bak"
                )

                result = self._run_cli(
                    "--source", data,
                    "--db", staging,
                    "--encrypt",
                    "--defer-credential-scrub",
                    "--installer-staging",
                    "--upgrade-backup", backup,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(final_db.read_bytes(), final_before)
                self.assertEqual(staging.read_bytes(), staging_before)
                self.assertEqual(
                    {path.name: path.read_bytes() for path in data.glob("*.ini")},
                    source_before,
                )
                self.assertFalse(backup.exists())

    def test_verify_installer_state_rejects_complete_with_new_credentials(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            self._create_auth1_database(db, "complete", [])
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute(
                    "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
                )
            credentials = data / "Unexpected.ini"
            credentials.write_text(
                "[Remote]\nPassword=new-password\nApiToken=new-token\n",
                encoding="utf-8",
            )
            before = self._tree_snapshot(data)

            result = self._verify_installer_cli(data, db)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(self._tree_snapshot(data), before)

    def test_verify_installer_state_rejects_pending_unknown_or_changed(self) -> None:
        for case in ("unknown", "changed"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                known = data / "Known.ini"
                known.write_text(
                    "[Remote]\nApiToken=known-secret\nKeepMe=1\n",
                    encoding="utf-8",
                )
                manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                    data, None
                )
                db = data / "ConfigStore.db"
                self._create_auth1_database(db, "pending", manifest)
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.execute(
                        "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
                    )
                if case == "unknown":
                    (data / "Unknown.ini").write_text(
                        "[Remote]\nPassword=unknown-secret\n",
                        encoding="utf-8",
                    )
                else:
                    known.write_bytes(known.read_bytes() + b"Changed=1\n")
                before = self._tree_snapshot(data)

                result = self._verify_installer_cli(data, db)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self._tree_snapshot(data), before)

    def test_verify_installer_state_rejects_pending_plaintext_residue(self) -> None:
        for case in ("database-backup", "credential-ini-backup"):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                data = Path(text) / "Data"
                data.mkdir()
                known = data / "Known.ini"
                known.write_text(
                    "[Remote]\nApiToken=known-secret\nKeepMe=1\n",
                    encoding="utf-8",
                )
                if case == "database-backup":
                    (data / "ConfigStore.db.bak-preexisting").write_bytes(
                        b"plaintext database backup sentinel"
                    )
                else:
                    (data / "Credentials.ini.bak").write_text(
                        "[Remote]\nPassword=backup-secret\nKeepMe=2\n",
                        encoding="utf-8",
                    )
                manifest = MIGRATOR["prepare_legacy_ini_credential_scrub"](
                    data, None
                )
                db = data / "ConfigStore.db"
                self._create_auth1_database(db, "pending", manifest)
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.execute(
                        "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
                    )
                before = self._tree_snapshot(data)

                result = self._verify_installer_cli(data, db)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(
                    "Plaintext ConfigStore backup/journal candidates block release",
                    result.stdout + result.stderr,
                )
                self.assertEqual(self._tree_snapshot(data), before)

    def test_verify_installer_state_cli_contract_is_strict(self) -> None:
        cases = (
            "missing-source",
            "missing-db",
            "overwrite",
            "encrypt",
            "scrub",
            "defer",
            "staging",
            "restore",
            "upgrade-backup",
            "verify-current",
            "encoding",
            "mojibake",
            "print-source",
            "bad-name",
            "other-directory",
            "sidecar",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                data = root / "Data"
                data.mkdir()
                db = data / "ConfigStore.db"
                self._create_auth1_database(db, "complete", [])
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.execute(
                        "UPDATE meta SET value='2' WHERE key='auth_semantic_version'"
                    )
                arguments: list[object] = [
                    "--verify-installer-state",
                    "--source", data,
                    "--db", db,
                ]
                if case == "missing-source":
                    del arguments[1:3]
                elif case == "missing-db":
                    del arguments[3:5]
                elif case == "overwrite":
                    arguments.append("--overwrite")
                elif case == "encrypt":
                    arguments.append("--encrypt")
                elif case == "scrub":
                    arguments.append("--scrub-legacy-credentials")
                elif case == "defer":
                    arguments.append("--defer-credential-scrub")
                elif case == "staging":
                    arguments.extend((
                        "--installer-staging", "--defer-credential-scrub"
                    ))
                elif case == "restore":
                    arguments.extend(("--restore-dpapi-backup", data / "fake.bak"))
                elif case == "upgrade-backup":
                    arguments.extend(("--upgrade-backup", data / "fake.dpapi.bak"))
                elif case == "verify-current":
                    arguments.append("--verify-current")
                elif case == "encoding":
                    arguments.extend(("--encoding", "utf-8"))
                elif case == "mojibake":
                    arguments.append("--allow-mojibake")
                elif case == "print-source":
                    arguments.append("--print-source-sha256")
                elif case == "bad-name":
                    bad = data / "Other.db"
                    bad.write_bytes(db.read_bytes())
                    arguments[4] = bad
                elif case == "other-directory":
                    other = root / "Other"
                    other.mkdir()
                    other_db = other / "ConfigStore.db"
                    other_db.write_bytes(db.read_bytes())
                    arguments[4] = other_db
                elif case == "sidecar":
                    Path(str(db) + "-wal").write_bytes(b"sidecar sentinel")
                before = self._tree_snapshot(data)

                result = self._run_cli(*arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self._tree_snapshot(data), before)

    def test_verify_dpapi_backup_cli_contract_is_strict_and_read_only(self) -> None:
        cases = (
            "missing-db", "relative-backup", "same-path", "other-directory",
            "source", "overwrite", "encrypt", "scrub", "defer", "staging",
            "restore", "upgrade-backup", "verify-current", "verify-installer",
            "encoding", "mojibake", "print-source", "sidecar",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                data = root / "Data"
                data.mkdir()
                db = data / "ConfigStore.db"
                with closing(sqlite3.connect(db)) as connection, connection:
                    connection.execute("CREATE TABLE sample(value TEXT)")
                    connection.execute("INSERT INTO sample VALUES('contract')")
                backup = data / "ConfigStore.db.contract.dpapi.bak"
                MIGRATOR["create_dpapi_database_backup"](db, backup)
                arguments: list[object] = [
                    "--verify-dpapi-backup-against", backup,
                    "--db", db,
                ]
                if case == "missing-db":
                    del arguments[2:4]
                elif case == "relative-backup":
                    arguments[1] = Path(backup.name)
                elif case == "same-path":
                    arguments[1] = db
                elif case == "other-directory":
                    other = root / "Other"
                    other.mkdir()
                    other_backup = other / backup.name
                    other_backup.write_bytes(backup.read_bytes())
                    arguments[1] = other_backup
                elif case == "source":
                    arguments.extend(("--source", data))
                elif case == "overwrite":
                    arguments.append("--overwrite")
                elif case == "encrypt":
                    arguments.append("--encrypt")
                elif case == "scrub":
                    arguments.append("--scrub-legacy-credentials")
                elif case == "defer":
                    arguments.append("--defer-credential-scrub")
                elif case == "staging":
                    arguments.extend((
                        "--installer-staging", "--defer-credential-scrub"
                    ))
                elif case == "restore":
                    arguments.extend(("--restore-dpapi-backup", backup))
                elif case == "upgrade-backup":
                    arguments.extend(("--upgrade-backup", backup))
                elif case == "verify-current":
                    arguments.append("--verify-current")
                elif case == "verify-installer":
                    arguments.extend(("--verify-installer-state", "--source", data))
                elif case == "encoding":
                    arguments.extend(("--encoding", "utf-8"))
                elif case == "mojibake":
                    arguments.append("--allow-mojibake")
                elif case == "print-source":
                    arguments.append("--print-source-sha256")
                elif case == "sidecar":
                    Path(str(db) + "-wal").write_bytes(b"sidecar sentinel")
                before = self._tree_snapshot(root)
                result = self._run_cli(*arguments)
                self.assertNotEqual(
                    result.returncode, 0, result.stdout + result.stderr
                )
                self.assertEqual(self._tree_snapshot(root), before)

        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            db = data / "ConfigStore.db"
            with closing(sqlite3.connect(db)) as connection, connection:
                connection.execute("CREATE TABLE sample(value TEXT)")
                connection.execute("INSERT INTO sample VALUES('read-only')")
            backup = data / "ConfigStore.db.read-only.dpapi.bak"
            MIGRATOR["create_dpapi_database_backup"](db, backup)
            before = self._tree_snapshot(data)
            result = self._run_cli(
                "--verify-dpapi-backup-against", backup,
                "--db", db,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(self._tree_snapshot(data), before)

    def test_installer_upgrade_staging_keeps_legacy_input_gate(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            final_db = data / "ConfigStore.db"
            with closing(sqlite3.connect(final_db)) as connection, connection:
                create_settings_schema(connection)
            legacy_ini = data / "Remote.ini"
            legacy_ini.write_text(
                "[Remote]\nApiToken=must-not-be-ignored\nKeepMe=1\n",
                encoding="utf-8",
            )
            final_before = final_db.read_bytes()
            source_before = legacy_ini.read_bytes()
            staging = data / (
                ".ConfigStore.db.install-upgrade-"
                "abcdefabcdefabcdefabcdefabcdefab.tmp"
            )
            staging.write_bytes(final_before)
            staging_before = staging.read_bytes()
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )

            result = self._run_cli(
                "--source", data,
                "--db", staging,
                "--encrypt",
                "--defer-credential-scrub",
                "--installer-staging",
                "--upgrade-backup", backup,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                "does not import legacy INI/TXT", result.stdout + result.stderr
            )
            self.assertEqual(final_db.read_bytes(), final_before)
            self.assertEqual(staging.read_bytes(), staging_before)
            self.assertEqual(legacy_ini.read_bytes(), source_before)
            self.assertFalse(backup.exists())

    def test_installer_upgrade_rechecks_staging_and_final_authority(self) -> None:
        with tempfile.TemporaryDirectory() as text:
            data = Path(text) / "Data"
            data.mkdir()
            final_db = data / "ConfigStore.db"
            self._create_auth1_database(final_db, "complete", [])
            staging = data / (
                ".ConfigStore.db.install-upgrade-"
                "dddddddddddddddddddddddddddddddd.tmp"
            )
            staging.write_bytes(final_db.read_bytes())
            attacker = data / "attacker-staging.db"
            attacker.write_bytes(staging.read_bytes())
            with closing(sqlite3.connect(attacker)) as connection, connection:
                connection.execute(
                    "UPDATE settings SET value_text=? "
                    "WHERE scope_type='account' AND scope_id='admin' "
                    "AND module='Profile' AND key_name='PasswordHash'",
                    (password_record("admin", "attacker-swap"),),
                )
            attacker_bytes = attacker.read_bytes()
            final_before = final_db.read_bytes()
            backup = staging.with_name(
                staging.name + ".install-backup.dpapi.bak"
            )
            function_globals = MIGRATOR["migrate"].__globals__
            actual_recover = function_globals[
                "recover_pending_atomic_transactions"
            ]
            swapped = False

            def recover_after_staging_swap(root: Path) -> None:
                nonlocal swapped
                if not swapped:
                    os.replace(attacker, staging)
                    swapped = True
                actual_recover(root)

            with (
                mock.patch.dict(
                    function_globals,
                    {
                        "recover_pending_atomic_transactions":
                            recover_after_staging_swap,
                    },
                ),
                self.assertRaisesRegex(
                    ValueError, "staging authority changed"
                ),
            ):
                MIGRATOR["migrate"](
                    data,
                    staging,
                    False,
                    True,
                    None,
                    False,
                    defer_credential_scrub=True,
                    installer_staging=True,
                    upgrade_backup_path=backup,
                )
            self.assertTrue(swapped)
            self.assertEqual(final_db.read_bytes(), final_before)
            self.assertEqual(staging.read_bytes(), attacker_bytes)
            self.assertFalse(backup.exists())
            self.assertFalse(any(data.glob(".*.atomic-transaction-v1")))

    def test_installer_staging_rejects_invalid_contract_without_side_effects(self) -> None:
        cases = (
            "missing-defer",
            "print-source-missing-defer",
            "scrub",
            "overwrite",
            "restore",
            "verify",
            "wrong-name",
            "uppercase-token",
            "other-directory",
            "create-target-exists",
            "create-final-exists",
            "upgrade-target-missing",
            "upgrade-copy-mismatch",
            "upgrade-backup-missing",
            "create-with-upgrade-backup",
            "ordinary-defer-staging-name",
        )
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as text:
                root = Path(text)
                data = root / "Data"
                data.mkdir()
                source = data / "Secrets.ini"
                source.write_text(
                    "[Remote]\nApiToken=must-remain\nKeepMe=1\n",
                    encoding="utf-8",
                )
                source_before = source.read_bytes()
                final_db = data / "ConfigStore.db"
                create_name = (
                    ".ConfigStore.db.install-create-"
                    "11111111111111111111111111111111.tmp"
                )
                upgrade_name = (
                    ".ConfigStore.db.install-upgrade-"
                    "22222222222222222222222222222222.tmp"
                )
                staging = data / create_name
                arguments: list[object] = [
                    "--source", data,
                    "--db", staging,
                    "--encrypt",
                    "--defer-credential-scrub",
                    "--installer-staging",
                ]

                if case == "missing-defer":
                    arguments.remove("--defer-credential-scrub")
                elif case == "print-source-missing-defer":
                    arguments.remove("--defer-credential-scrub")
                    arguments.append("--print-source-sha256")
                elif case == "scrub":
                    arguments.append("--scrub-legacy-credentials")
                elif case == "overwrite":
                    arguments.append("--overwrite")
                elif case == "restore":
                    arguments.extend(("--restore-dpapi-backup", data / "fake.bak"))
                elif case == "verify":
                    arguments.append("--verify-current")
                elif case == "wrong-name":
                    arguments[3] = data / ".ConfigStore.db.install-create-short.tmp"
                elif case == "uppercase-token":
                    arguments[3] = data / (
                        ".ConfigStore.db.install-create-"
                        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA.tmp"
                    )
                elif case == "other-directory":
                    other = root / "Other"
                    other.mkdir()
                    arguments[3] = other / create_name
                elif case == "create-target-exists":
                    staging.write_bytes(b"create target sentinel")
                elif case == "create-final-exists":
                    final_db.write_bytes(b"final sentinel")
                elif case == "upgrade-target-missing":
                    arguments[3] = data / upgrade_name
                elif case == "upgrade-copy-mismatch":
                    final_db.write_bytes(b"final bytes")
                    upgrade = data / upgrade_name
                    upgrade.write_bytes(b"different bytes")
                    arguments[3] = upgrade
                    arguments.extend((
                        "--upgrade-backup",
                        upgrade.with_name(upgrade.name + ".install-backup.dpapi.bak"),
                    ))
                elif case == "upgrade-backup-missing":
                    final_db.write_bytes(b"same bytes")
                    upgrade = data / upgrade_name
                    upgrade.write_bytes(b"same bytes")
                    arguments[3] = upgrade
                elif case == "create-with-upgrade-backup":
                    arguments.extend((
                        "--upgrade-backup",
                        staging.with_name(staging.name + ".install-backup.dpapi.bak"),
                    ))
                elif case == "ordinary-defer-staging-name":
                    arguments.remove("--installer-staging")

                final_before = (
                    final_db.read_bytes() if final_db.exists() else None
                )
                result = self._run_cli(*arguments)
                self.assertNotEqual(
                    result.returncode, 0, result.stdout + result.stderr
                )
                self.assertEqual(source.read_bytes(), source_before)
                self.assertEqual(
                    final_db.read_bytes() if final_db.exists() else None,
                    final_before,
                )
                if case == "create-target-exists":
                    self.assertEqual(staging.read_bytes(), b"create target sentinel")
                self.assertFalse(any(data.glob("*.install-backup.dpapi.bak")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
