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
                                    encrypted=0,
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
    def test_windows_credential_scrub_replace_is_write_through_and_rolls_back(self) -> None:
        source_text = (
            REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"
        ).read_text(encoding="utf-8")
        helper_start = source_text.index("def _replace_file_write_through(")
        atomic_start = source_text.index("def _atomic_replace_bytes(")
        atomic_end = source_text.index("def _atomic_create_bytes(")
        helper_source = source_text[helper_start:atomic_start]
        atomic_source = source_text[atomic_start:atomic_end]
        self.assertIn("MoveFileExW", helper_source)
        self.assertIn(
            "MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH",
            helper_source,
        )
        self.assertNotIn("os.replace(", atomic_source)
        self.assertIn(
            "_replace_file_write_through(temporary_path, path)",
            atomic_source,
        )

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
            actual_replace = function_globals["_replace_file_write_through"]
            calls: list[tuple[str, str]] = []

            def faulting_replace(source: Path, destination: Path) -> None:
                calls.append((source.name, destination.name))
                if len(calls) == 2:
                    raise OSError("injected second-file replacement failure")
                actual_replace(source, destination)

            function_globals["_replace_file_write_through"] = faulting_replace
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
                function_globals["_replace_file_write_through"] = actual_replace

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
                [destination for _source, destination in calls],
                ["First.ini", "Second.ini", "First.ini"],
            )
            self.assertFalse(
                any(data.glob(".*.credential-scrub-*.tmp"))
            )

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
                for key, value, value_type, sensitive in (
                    ("PasswordHash", admin_hash, "string", 1),
                    ("Role", "admin", "string", 0),
                    ("CreatedAt", "2026-06-05T00:00:00", "datetime", 0),
                    ("UpdatedAt", "2026-06-01T00:00:00", "datetime", 0),
                ):
                    insert_setting(
                        connection, "account", "admin", "Profile",
                        key, value, value_type=value_type,
                        sensitive=sensitive,
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
                self.assertEqual(profiles[("admin", "PasswordHash")], (admin_hash, 0))
                self.assertEqual(profiles[("admin", "Role")], ("admin", 0))
                self.assertEqual(profiles[("cyh", "PasswordHash")], (cyh_hash, 0))
                self.assertEqual(profiles[("cyh", "Role")], ("operator", 0))
                self.assertEqual(
                    profiles[("admin", "CreatedAt")],
                    ("2026-05-07T00:00:00", 0),
                )
                self.assertEqual(
                    profiles[("admin", "UpdatedAt")],
                    ("2026-06-01T00:00:00", 0),
                )
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
