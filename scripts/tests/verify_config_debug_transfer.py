#!/usr/bin/env python3
"""Verify encrypted field ConfigStore export and local DPAPI rebinding."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile


REPO_ROOT = Path(__file__).resolve().parents[2]
MIGRATOR_PATH = REPO_ROOT / "tools" / "migrate_config_to_sqlite.py"


def load_migrator():
    spec = importlib.util.spec_from_file_location(
        "config_debug_transfer_migrator", MIGRATOR_PATH
    )
    if spec is None or spec.loader is None:
        raise AssertionError("Cannot load ConfigMigrate source")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_migrator(*arguments: object, expect_success: bool = True):
    environment = dict(os.environ)
    environment["PYTHONUTF8"] = "1"
    executable = environment.get("CONFIG_MIGRATE_UNDER_TEST", "").strip()
    command = (
        [executable]
        if executable
        else [sys.executable, "-I", str(MIGRATOR_PATH)]
    )
    completed = subprocess.run(
        [*command, *map(str, arguments)],
        cwd=REPO_ROOT,
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if expect_success and completed.returncode != 0:
        raise AssertionError(
            f"ConfigMigrate failed ({completed.returncode}):\n"
            f"{completed.stdout}"
        )
    if not expect_success and completed.returncode == 0:
        raise AssertionError(
            "ConfigMigrate unexpectedly accepted an invalid debug transfer"
        )
    return completed


def create_field_database(module, path: Path) -> dict[tuple[str, str, str, str], str]:
    connection = sqlite3.connect(path)
    try:
        module.init_schema(connection, encrypt=True)
        module.set_schema_meta(
            connection, encrypt=True, authentication_initialized=False
        )
        values = {
            ("global", "", "LoginState", "RememberPassword"): "1",
            ("global", "", "LoginState", "AutoLogin"): "1",
            ("global", "", "OnlineServices", "FtpPassword"):
                "field-ftp-secret-20260728",
            ("robot", "RobotC", "RobotPara", "RobotName"): "RobotC",
        }
        for identity, value in values.items():
            inserted = module.insert_scoped_setting(
                connection,
                *identity,
                value,
                True,
                "string",
                overwrite=False,
            )
            if not inserted:
                raise AssertionError(f"Cannot create fixture setting: {identity}")
        connection.commit()
    finally:
        connection.close()
    return values


def read_plain_values(module, path: Path):
    plain: dict[tuple[str, str, str, str], str] = {}
    stored: dict[tuple[str, str, str, str], str] = {}
    connection = sqlite3.connect(path)
    try:
        for scope_type, scope_id, section, key, value, encrypted in connection.execute(
            """
            SELECT scope_type, scope_id, module, key_name, value_text, encrypted
            FROM settings
            ORDER BY scope_type, scope_id, module, key_name
            """
        ):
            identity = (
                str(scope_type), str(scope_id), str(section), str(key)
            )
            stored[identity] = str(value)
            decoded = module.decode_stored_text(
                str(value),
                encrypted,
                module.protection_purpose(*identity),
            )
            if decoded is None:
                raise AssertionError(f"Cannot decode imported setting: {identity}")
            plain[identity] = decoded
        metadata = dict(connection.execute(
            "SELECT key, value FROM meta"
        ).fetchall())
        login_types = dict(connection.execute(
            """
            SELECT key_name, value_type FROM settings
            WHERE scope_type='global' AND scope_id='' AND module='LoginState'
              AND key_name IN ('RememberPassword', 'AutoLogin')
            """
        ).fetchall())
    finally:
        connection.close()
    return plain, stored, metadata, login_types


def main() -> None:
    if os.name != "nt":
        raise SystemExit("SKIP: debug transfer verification requires Windows DPAPI")
    module = load_migrator()
    with tempfile.TemporaryDirectory(
        prefix="ConfigDebugTransfer-", dir=REPO_ROOT / "tmp"
    ) as temporary:
        root = Path(temporary).resolve()
        source_db = root / "FieldConfigStore.db"
        imported_db = root / "LocalData" / "ConfigStore.db"
        imported_db.parent.mkdir()
        public_key = root / "reproduction.public.pem"
        private_key = root / "reproduction.private.dpapi"
        package = root / "field-config.debugdb"

        expected = create_field_database(module, source_db)
        source_sha256 = hashlib.sha256(source_db.read_bytes()).hexdigest()
        module.verify_current_database(
            source_db, allow_known_auth3_login_preference_drift=True
        )

        run_migrator(
            "--create-debug-transfer-key",
            "--debug-public-key", public_key,
            "--debug-private-key", private_key,
        )
        run_migrator(
            "--export-debug-database", package,
            "--db", source_db,
            "--debug-public-key", public_key,
        )
        if hashlib.sha256(source_db.read_bytes()).hexdigest() != source_sha256:
            raise AssertionError("Field database changed during debug export")
        package_bytes = package.read_bytes()
        if b"field-ftp-secret-20260728" in package_bytes:
            raise AssertionError("Encrypted debug package leaked plaintext")
        if not package_bytes.startswith(module.DEBUG_TRANSFER_PACKAGE_MAGIC):
            raise AssertionError("Debug package magic is missing")

        run_migrator(
            "--import-debug-database", package,
            "--db", imported_db,
            "--debug-private-key", private_key,
        )
        module.verify_current_database(
            imported_db, allow_known_auth3_login_preference_drift=True
        )
        source_plain, source_stored, _source_meta, _source_types = (
            read_plain_values(module, source_db)
        )
        local_plain, local_stored, local_meta, local_types = read_plain_values(
            module, imported_db
        )
        if source_plain != expected or local_plain != expected:
            raise AssertionError("Imported logical settings differ from the field database")
        protected_identities = [
            identity
            for identity, value in source_stored.items()
            if value.startswith(module.DPAPI_PREFIX)
        ]
        if not protected_identities or any(
            source_stored[identity] == local_stored[identity]
            for identity in protected_identities
        ):
            raise AssertionError("Field DPAPI values were not locally rebound")
        if (
            local_meta.get("sensitive_protection") != "dpapi-current-user-v1"
            or any(key in local_meta for key in module.DEBUG_TRANSFER_META_KEYS)
        ):
            raise AssertionError("Imported database retains debug transfer metadata")
        if set(local_types.values()) != {"string"}:
            raise AssertionError("Known auth3 field drift was not reproduced")

        other_public = root / "other.public.pem"
        other_private = root / "other.private.dpapi"
        wrong_target = root / "WrongRecipient" / "ConfigStore.db"
        wrong_target.parent.mkdir()
        run_migrator(
            "--create-debug-transfer-key",
            "--debug-public-key", other_public,
            "--debug-private-key", other_private,
        )
        mismatch = run_migrator(
            "--import-debug-database", package,
            "--db", wrong_target,
            "--debug-private-key", other_private,
            expect_success=False,
        )
        if wrong_target.exists() or "different recipient key" not in mismatch.stdout:
            raise AssertionError("Recipient-key mismatch did not fail closed")

        payload = json.loads(
            package_bytes[len(module.DEBUG_TRANSFER_PACKAGE_MAGIC):]
            .decode("ascii")
        )
        ciphertext = payload["ciphertext"]
        replacement = "A" if ciphertext[-1] != "A" else "B"
        payload["ciphertext"] = ciphertext[:-1] + replacement
        tampered_package = root / "tampered.debugdb"
        tampered_package.write_bytes(
            module.DEBUG_TRANSFER_PACKAGE_MAGIC
            + json.dumps(
                payload, sort_keys=True, separators=(",", ":")
            ).encode("ascii")
            + b"\n"
        )
        tampered_target = root / "Tampered" / "ConfigStore.db"
        tampered_target.parent.mkdir()
        run_migrator(
            "--import-debug-database", tampered_package,
            "--db", tampered_target,
            "--debug-private-key", private_key,
            expect_success=False,
        )
        if tampered_target.exists():
            raise AssertionError("Tampered debug package created a database")

    print(
        "PASS: field ConfigStore is exported end-to-end encrypted and "
        "locally rebound without changing the source"
    )


if __name__ == "__main__":
    main()
