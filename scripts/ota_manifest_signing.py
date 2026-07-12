#!/usr/bin/env python3
"""OTA manifest v3 signing helpers.

The release private key is never stored in the repository.  On Windows the
generated PKCS#8 key is protected with CurrentUser DPAPI; the client contains
only the corresponding CNG RSA public-key blob.
"""

from __future__ import annotations

import argparse
import base64
import ctypes
from ctypes import wintypes
import datetime as _datetime
import hashlib
import os
from pathlib import Path
import re
import struct
import sys

from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa


SCHEMA_VERSION = 3
LEGACY_SIGNED_SCHEMA_VERSION = 2
SIGNATURE_ALGORITHM = "RSA-PKCS1-SHA256"
MANIFEST_VALIDITY_SECONDS = 7 * 24 * 60 * 60
MANIFEST_CLOCK_SKEW_SECONDS = 10 * 60
UTC_TIMESTAMP_RE = re.compile(
    r"^[0-9]{4}-(?:0[1-9]|1[0-2])-(?:0[1-9]|[12][0-9]|3[01])T"
    r"(?:[01][0-9]|2[0-3]):[0-5][0-9]:[0-5][0-9]Z$"
)
DPAPI_MAGIC = b"NTR-OTA-SIGNING-KEY-DPAPI-V1\n"
DPAPI_ENTROPY = b"NoTeaching-Robot/OTA-Signing-Key/v1"
BCRYPT_RSAPUBLIC_MAGIC = 0x31415352  # 'RSA1'


class DATA_BLOB(ctypes.Structure):
    _fields_ = [("cbData", wintypes.DWORD), ("pbData", ctypes.POINTER(ctypes.c_ubyte))]


def _blob(data: bytes) -> tuple[DATA_BLOB, object]:
    buffer = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
    return DATA_BLOB(len(data), ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte))), buffer


def _dpapi(protect: bool, data: bytes) -> bytes:
    if os.name != "nt":
        raise RuntimeError("OTA signing-key DPAPI operations require Windows")
    crypt32 = ctypes.WinDLL("crypt32", use_last_error=True)
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    source, source_buffer = _blob(data)
    entropy, entropy_buffer = _blob(DPAPI_ENTROPY)
    output = DATA_BLOB()
    _ = source_buffer, entropy_buffer
    if protect:
        ok = crypt32.CryptProtectData(
            ctypes.byref(source),
            "NoTeaching-Robot OTA release signing key",
            ctypes.byref(entropy),
            None,
            None,
            0x1,  # CRYPTPROTECT_UI_FORBIDDEN
            ctypes.byref(output),
        )
    else:
        ok = crypt32.CryptUnprotectData(
            ctypes.byref(source),
            None,
            ctypes.byref(entropy),
            None,
            None,
            0x1,
            ctypes.byref(output),
        )
    if not ok:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        return ctypes.string_at(output.pbData, output.cbData)
    finally:
        kernel32.LocalFree(output.pbData)


def _strict_text(value: object, field: str) -> str:
    if not isinstance(value, str) or "\r" in value or "\n" in value:
        raise ValueError(f"manifest {field} must be a single-line string")
    return value


def strict_utc_timestamp(value: object, field: str) -> tuple[str, _datetime.datetime]:
    text = _strict_text(value, field)
    if UTC_TIMESTAMP_RE.fullmatch(text) is None:
        raise ValueError(f"manifest {field} must be canonical UTC YYYY-MM-DDTHH:MM:SSZ")
    try:
        parsed = _datetime.datetime.strptime(text, "%Y-%m-%dT%H:%M:%SZ").replace(
            tzinfo=_datetime.timezone.utc
        )
    except ValueError as exc:
        raise ValueError(f"manifest {field} is not a valid UTC calendar time") from exc
    return text, parsed


def manifest_freshness_contract(
    manifest: dict,
) -> tuple[str, str, _datetime.datetime, _datetime.datetime]:
    published_text, published = strict_utc_timestamp(
        manifest.get("publishedAtUtc"), "publishedAtUtc"
    )
    expires_text, expires = strict_utc_timestamp(
        manifest.get("expiresAtUtc"), "expiresAtUtc"
    )
    if int((expires - published).total_seconds()) != MANIFEST_VALIDITY_SECONDS:
        raise ValueError(
            "manifest expiresAtUtc must be exactly 7 days after publishedAtUtc"
        )
    return published_text, expires_text, published, expires


def _payload_values(
    manifest: dict,
) -> tuple[str, str, str, str, int, str, str, str, int, str]:
    channel = _strict_text(manifest.get("channel"), "channel")
    version = _strict_text(manifest.get("version"), "version")
    file_name = _strict_text(manifest.get("file"), "file")
    file_sha = _strict_text(manifest.get("sha256"), "sha256")
    file_size = manifest.get("size")
    if not isinstance(file_size, int) or isinstance(file_size, bool) or file_size <= 0:
        raise ValueError("manifest size must be a positive integer")
    notes = manifest.get("notes", "")
    if not isinstance(notes, str):
        raise ValueError("manifest notes must be a string")

    patch = manifest.get("patch")
    if patch is None:
        patch_file = patch_sha = patch_base = ""
        patch_size = 0
    else:
        if not isinstance(patch, dict):
            raise ValueError("manifest patch must be an object")
        patch_file = _strict_text(patch.get("file"), "patch.file")
        patch_sha = _strict_text(patch.get("sha256"), "patch.sha256")
        patch_base = _strict_text(patch.get("baseMinVersion"), "patch.baseMinVersion")
        patch_size = patch.get("size")
        if not isinstance(patch_size, int) or isinstance(patch_size, bool) or patch_size <= 0:
            raise ValueError("manifest patch.size must be a positive integer")

    return (
        channel,
        version,
        file_name,
        file_sha,
        file_size,
        notes,
        patch_file,
        patch_sha,
        patch_size,
        patch_base,
    )


def signature_payload(manifest: dict) -> bytes:
    """Build the byte-exact v3 payload shared with the C++ BCrypt verifier."""
    if manifest.get("schemaVersion") != SCHEMA_VERSION:
        raise ValueError("manifest schemaVersion must be 3")
    (
        channel,
        version,
        file_name,
        file_sha,
        file_size,
        notes,
        patch_file,
        patch_sha,
        patch_size,
        patch_base,
    ) = _payload_values(manifest)
    published_text, expires_text, _, _ = manifest_freshness_contract(manifest)

    notes_sha = hashlib.sha256(notes.encode("utf-8")).hexdigest()
    fields = (
        ("schemaVersion", str(SCHEMA_VERSION)),
        ("channel", channel),
        ("version", version),
        ("publishedAtUtc", published_text),
        ("expiresAtUtc", expires_text),
        ("file", file_name),
        ("sha256", file_sha),
        ("size", str(file_size)),
        ("notesSha256", notes_sha),
        ("patch.file", patch_file),
        ("patch.sha256", patch_sha),
        ("patch.size", str(patch_size)),
        ("patch.baseMinVersion", patch_base),
    )
    return ("NoTeaching-Robot OTA Manifest Signature v3\n" +
            "".join(f"{name}={value}\n" for name, value in fields)).encode("utf-8")


def legacy_signature_payload_v2(manifest: dict) -> bytes:
    """Reproduce the v2 bytes for the installed-client compatibility endpoint."""
    if manifest.get("schemaVersion") != LEGACY_SIGNED_SCHEMA_VERSION:
        raise ValueError("legacy manifest schemaVersion must be 2")
    (
        channel,
        version,
        file_name,
        file_sha,
        file_size,
        notes,
        patch_file,
        patch_sha,
        patch_size,
        patch_base,
    ) = _payload_values(manifest)
    notes_sha = hashlib.sha256(notes.encode("utf-8")).hexdigest()
    fields = (
        ("schemaVersion", str(LEGACY_SIGNED_SCHEMA_VERSION)),
        ("channel", channel),
        ("version", version),
        ("file", file_name),
        ("sha256", file_sha),
        ("size", str(file_size)),
        ("notesSha256", notes_sha),
        ("patch.file", patch_file),
        ("patch.sha256", patch_sha),
        ("patch.size", str(patch_size)),
        ("patch.baseMinVersion", patch_base),
    )
    return ("NoTeaching-Robot OTA Manifest Signature v2\n" +
            "".join(f"{name}={value}\n" for name, value in fields)).encode("utf-8")


def load_private_key(path: os.PathLike[str] | str):
    raw = Path(path).read_bytes()
    if not raw.startswith(DPAPI_MAGIC):
        raise ValueError("signing key is not a DPAPI v1 key file")
    protected = base64.b64decode(raw[len(DPAPI_MAGIC):], validate=True)
    der = _dpapi(False, protected)
    return serialization.load_der_private_key(der, password=None)


def sign_manifest(manifest: dict, key_path: os.PathLike[str] | str) -> str:
    key = load_private_key(key_path)
    signature = key.sign(signature_payload(manifest), padding.PKCS1v15(), hashes.SHA256())
    return base64.b64encode(signature).decode("ascii")


def sign_legacy_manifest_v2(manifest: dict, key_path: os.PathLike[str] | str) -> str:
    """Sign the compatibility manifest consumed by already-deployed v2 clients."""
    key = load_private_key(key_path)
    signature = key.sign(
        legacy_signature_payload_v2(manifest), padding.PKCS1v15(), hashes.SHA256()
    )
    return base64.b64encode(signature).decode("ascii")


def verify_manifest(manifest: dict, signature_b64: str, public_key) -> bool:
    try:
        schema_version = manifest.get("schemaVersion")
        if schema_version == SCHEMA_VERSION:
            payload = signature_payload(manifest)
        elif schema_version == LEGACY_SIGNED_SCHEMA_VERSION:
            payload = legacy_signature_payload_v2(manifest)
        else:
            return False
        public_key.verify(
            base64.b64decode(signature_b64, validate=True),
            payload,
            padding.PKCS1v15(),
            hashes.SHA256(),
        )
        return True
    except Exception:
        return False


def cng_public_blob(public_key) -> bytes:
    numbers = public_key.public_numbers()
    exponent = numbers.e.to_bytes((numbers.e.bit_length() + 7) // 8, "big")
    modulus = numbers.n.to_bytes((numbers.n.bit_length() + 7) // 8, "big")
    header = struct.pack(
        "<LLLLLL",
        BCRYPT_RSAPUBLIC_MAGIC,
        len(modulus) * 8,
        len(exponent),
        len(modulus),
        0,
        0,
    )
    return header + exponent + modulus


def generate_key(path: Path, *, force: bool = False) -> str:
    if path.exists() and not force:
        raise FileExistsError(f"refusing to overwrite existing signing key: {path}")
    key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
    der = key.private_bytes(
        serialization.Encoding.DER,
        serialization.PrivateFormat.PKCS8,
        serialization.NoEncryption(),
    )
    protected = _dpapi(True, der)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(DPAPI_MAGIC + base64.b64encode(protected))
    return base64.b64encode(cng_public_blob(key.public_key())).decode("ascii")


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("generate", help="generate a CurrentUser-DPAPI RSA-3072 key")
    generate.add_argument("--key-file", required=True)
    generate.add_argument("--force", action="store_true")
    show = sub.add_parser("show-public-blob", help="print the CNG public blob for an existing key")
    show.add_argument("--key-file", required=True)
    args = parser.parse_args()

    if args.command == "generate":
        public_blob = generate_key(Path(args.key_file), force=args.force)
    else:
        public_blob = base64.b64encode(cng_public_blob(
            load_private_key(args.key_file).public_key())).decode("ascii")
    print(public_blob)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
