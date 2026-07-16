#!/usr/bin/env python3
"""Static fail-closed OTA client and immutable installer identity gate."""

from __future__ import annotations

from pathlib import Path
import ast
import base64
import hashlib
import os
import re
import subprocess
import struct
import sys
import tempfile

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding, rsa


ROOT = Path(__file__).resolve().parents[2]
APP_ID = "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
OTA_PUBLIC_KEY_FINGERPRINT = "5686a45ad2f6c2d84ba7d911c31cfb4ab972d6afcd9e677c0d9b535629749eda"
sys.path.insert(0, str(ROOT / "scripts"))

from ota_manifest_signing import (  # noqa: E402
    MANIFEST_CLOCK_SKEW_SECONDS,
    MANIFEST_VALIDITY_SECONDS,
    SCHEMA_VERSION,
    SIGNATURE_ALGORITHM,
    cng_public_blob,
    signature_payload,
    verify_manifest,
)


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    require(begin >= 0, f"missing section start: {start}")
    finish = text.find(end, begin + len(start))
    require(finish >= 0, f"missing section end: {end}")
    return text[begin:finish]


def extract_cpp_string_template(source: str, begin_marker: str, end_marker: str) -> str:
    block = section(source, begin_marker, end_marker)
    literal_parts = re.findall(r'"(?:\\.|[^"\\])*"', block)
    require(literal_parts, f"no C++ string literals found between {begin_marker} and {end_marker}")
    try:
        return "".join(ast.literal_eval(part) for part in literal_parts)
    except (SyntaxError, ValueError) as exc:
        raise AssertionError(f"invalid C++ batch template literal: {exc}") from exc


def extract_named_render_bindings(
    source: str,
    begin_marker: str,
    end_marker: str,
) -> list[tuple[str, str]]:
    block = section(source, begin_marker, end_marker)
    bindings = re.findall(
        r'\{\s*QStringLiteral\("(@@OTA_[A-Z0-9_]+@@)"\),\s*([A-Za-z_][A-Za-z0-9_]*)\s*\}',
        block,
    )
    require(bindings, f"no named batch bindings found between {begin_marker} and {end_marker}")
    require(len({marker for marker, _ in bindings}) == len(bindings),
            f"duplicate named batch binding between {begin_marker} and {end_marker}")
    return bindings


def render_bootstrap_from_production_source(
    source: str,
    template_markers: tuple[str, str],
    render_markers: tuple[str, str],
    sample_values: dict[str, str],
) -> str:
    template = extract_cpp_string_template(source, *template_markers)
    bindings = extract_named_render_bindings(source, *render_markers)
    template_sentinels = set(re.findall(r"@@OTA_[A-Z0-9_]+@@", template))
    bound_sentinels = {marker for marker, _ in bindings}
    require(template_sentinels == bound_sentinels,
            f"template/binding sentinel mismatch: {sorted(template_sentinels ^ bound_sentinels)}")
    rendered = template
    for marker, variable in bindings:
        require(variable in sample_values, f"no regression sample for production binding {variable}")
        require("@@OTA_" not in sample_values[variable],
                f"regression sample for {variable} injects a named sentinel")
        rendered = rendered.replace(marker, sample_values[variable])
    require("@@OTA_" not in rendered, "rendered OTA bootstrap retains a named sentinel")
    return rendered


def verify_rendered_ota_bootstrap(source: str) -> None:
    app = r"D:\Program Files\HK-Pathlynx-CORPLA\HK-Pathlynx-CORPLA.exe"
    app_dir = r"D:\Program Files\HK-Pathlynx-CORPLA"
    payload = r"D:\Program Files\HK-Pathlynx-CORPLA\Temp\OnlineUpdate\payload.bin"
    failure_log = r"D:\Program Files\HK-Pathlynx-CORPLA\Temp\OnlineUpdate\install_failed.txt"
    failure_marker = r"D:\Program Files\HK-Pathlynx-CORPLA\Temp\OnlineUpdate\patch_failed_2099.12.31.2359.flag"
    health_marker = r"D:\Program Files\HK-Pathlynx-CORPLA\Temp\OnlineUpdate\patch_healthy_2099.12.31.2359.flag"
    wait_command = (
        'powershell.exe -NoLogo -NoProfile -NonInteractive -Command "wait-for-pid"\r\n'
    )
    verify_command = (
        'powershell.exe -NoLogo -NoProfile -NonInteractive -Command "verify-size-and-sha256"\r\n'
    )
    samples = {
        "waitForCurrentProcess": wait_command,
        "payload": payload,
        "appDir": app_dir,
        "failureLogPath": failure_log,
        "appPath": app,
        "verifyPayload": verify_command,
        "patchFailureMarkerPath": failure_marker,
        "patchHealthMarkerPath": health_marker,
    }
    patch = render_bootstrap_from_production_source(
        source,
        ("// OTA_PATCH_BOOTSTRAP_TEMPLATE_BEGIN", "// OTA_PATCH_BOOTSTRAP_TEMPLATE_END"),
        ("// OTA_PATCH_BOOTSTRAP_RENDER_BEGIN", "// OTA_PATCH_BOOTSTRAP_RENDER_END"),
        samples,
    )
    full = render_bootstrap_from_production_source(
        source,
        ("// OTA_FULL_BOOTSTRAP_TEMPLATE_BEGIN", "// OTA_FULL_BOOTSTRAP_TEMPLATE_END"),
        ("// OTA_FULL_BOOTSTRAP_RENDER_BEGIN", "// OTA_FULL_BOOTSTRAP_RENDER_END"),
        samples,
    )

    for name, rendered in (("patch", patch), ("full", full)):
        without_crlf = rendered.replace("\r\n", "")
        require("\r" not in without_crlf and "\n" not in without_crlf,
                f"{name} bootstrap contains a non-CRLF line ending")
        require(rendered.endswith("\r\n"), f"{name} bootstrap lacks a final CRLF")
        require(not re.search(r"%[1-9][0-9]?", rendered),
                f"{name} bootstrap retains a positional QString placeholder")
        require(f'set "OTA_APP={app}"\r\n' in rendered,
                f"{name} bootstrap OTA_APP is not the real executable")
        require(f'set "OTA_HEALTH={health_marker}"\r\n' in rendered,
                f"{name} bootstrap health marker is misbound")
        require(failure_log not in re.search(r'^set "OTA_APP=.*$', rendered, re.MULTILINE).group(0),
                f"{name} bootstrap points OTA_APP at install_failed.txt")
        require(wait_command + "if errorlevel 1 goto :wait_failed\r\n" in rendered,
                f"{name} bootstrap wait command is not line-delimited")

    for assignment in (
        f'set "OTA_NEW={app}.ota-new"\r\n',
        f'set "OTA_BACKUP={app}.ota-backup"\r\n',
        f'set "OTA_FAILED={app}.ota-failed"\r\n',
        f'set "OTA_PATCH_FAILURE_MARKER={failure_marker}"\r\n',
        f'set "OTA_HEALTH={health_marker}"\r\n',
    ):
        require(assignment in patch, f"patch bootstrap binding missing: {assignment.strip()}")
    require(patch.count(verify_command) == 2,
            "patch bootstrap must verify both staged and copied executables")
    require(patch.count(verify_command + "if errorlevel 1 goto") == 2,
            "patch verification command is not separated from its error gate")
    require(f'>"{failure_log}" echo Staged patch hash or size verification failed.\r\n' in patch,
            "patch failure detail is not written to install_failed.txt")
    require(f'start "" /wait "{payload}" /VERYSILENT /SUPPRESSMSGBOXES /NORESTART /DIR="{app_dir}"\r\n' in full,
            "full bootstrap installer payload or install directory is misbound")
    require(full.count(verify_command) == 1,
            "full bootstrap must verify the installer exactly once")
    require(f'>"{failure_marker}" echo full_failed\r\n' in full,
            "full bootstrap failure marker is misbound")


def verify_native_cmd_bootstrap_launch() -> None:
    if os.name != "nt":
        return
    with tempfile.TemporaryDirectory(prefix="ota-cmd-") as temporary:
        special_dir = Path(temporary) / "%OTA_SHOULD_NOT_EXPAND% ! path with spaces"
        special_dir.mkdir()
        bootstrap = special_dir / "apply update.cmd"
        sentinel = special_dir / "ran.txt"
        bootstrap.write_bytes(
            b'@echo off\r\n>"%~dp0ran.txt" echo ok\r\nexit /b 0\r\n'
        )
        environment = os.environ.copy()
        environment["NO_TEACHING_OTA_BOOTSTRAP"] = str(bootstrap)
        environment["OTA_SHOULD_NOT_EXPAND"] = "EXPANDED"
        environment["ERRORLEVEL"] = "0"
        completed = subprocess.run(
            'cmd.exe /D /V:OFF /S /C ""%NO_TEACHING_OTA_BOOTSTRAP%""',
            env=environment,
            capture_output=True,
            timeout=15,
            check=False,
        )
        require(completed.returncode == 0 and sentinel.read_text().strip() == "ok",
                "native cmd bootstrap launch failed for spaces/percent/bang path")

        exit_check = special_dir / "negative exit check.cmd"
        caught = special_dir / "caught-negative.txt"
        exit_check.write_bytes(
            b'@echo off\r\n'
            b'powershell.exe -NoLogo -NoProfile -NonInteractive -Command "exit -1"\r\n'
            b'if errorlevel 1 goto :caught\r\n'
            b'if not errorlevel 0 goto :caught\r\n'
            b'exit /b 9\r\n'
            b':caught\r\n'
            b'>"%~dp0caught-negative.txt" echo caught\r\n'
            b'exit /b 0\r\n'
        )
        environment["NO_TEACHING_OTA_BOOTSTRAP"] = str(exit_check)
        completed = subprocess.run(
            'cmd.exe /D /V:OFF /S /C ""%NO_TEACHING_OTA_BOOTSTRAP%""',
            env=environment,
            capture_output=True,
            timeout=15,
            check=False,
        )
        require(completed.returncode == 0 and caught.read_text().strip() == "caught",
                "negative cmd exit code bypassed the poisoned-ERRORLEVEL-safe gate")


def main() -> int:
    app = read("src/OnlineServicesDialog.cpp")
    main_app = read("src/QtWidgetsApplication4.cpp")
    header = read("include/OnlineServicesDialog.h")
    installer = read("installer/QtWidgetsApplication4.iss")
    verify_native_cmd_bootstrap_launch()
    verify_rendered_ota_bootstrap(app)

    for token in (
        "IsStrictOtaVersion",
        "IsStrictSha256",
        "TryReadManifestSize",
        "TryParseCanonicalOtaUtc",
        "ValidateOtaFreshnessWindow",
        "ValidateOtaManifestFreshness",
        "ExpectedInstallerName",
        "ExpectedPatchName",
        "kMaximumManifestBytes",
        "kMaximumUpdatePayloadBytes",
        "BuildOtaManifestSignaturePayload",
        "VerifyOtaManifestSignature",
        "OtaManifestKnownAnswerSelfTest",
        "IsStrictManifestShape",
        "HashFileSha256",
        "BCryptVerifySignature",
        "BCRYPT_PAD_PKCS1",
    ):
        require(token in app, f"strict OTA manifest primitive missing: {token}")
    require("qint64 m_remoteSize" in header, "full installer manifest size is not retained")
    require("m_remoteManifestPublishedAtUtc" in header
            and "m_remoteManifestExpiresAtUtc" in header,
            "signed manifest freshness window is not retained through download authorization")
    public_blob_match = re.search(
        r'kOtaReleasePublicKeyBlobBase64\[\]\s*=\s*\n?\s*"([A-Za-z0-9+/=]+)"',
        app,
    )
    require(public_blob_match is not None, "embedded OTA release public key is missing")
    embedded_blob = base64.b64decode(public_blob_match.group(1), validate=True)
    require(hashlib.sha256(embedded_blob).hexdigest() == OTA_PUBLIC_KEY_FINGERPRINT,
            "embedded OTA release public-key fingerprint drifted")

    manifest = section(
        app,
        "void OnlineServicesDialog::OnManifestReply(",
        "void OnlineServicesDialog::StartDownload()",
    )
    for token in (
        "QJsonParseError",
        "TryReadManifestSize(obj.value(QStringLiteral(\"size\"))",
        "IsStrictSha256(remoteSha256)",
        "patchBaseMinVersion",
        "品牌增量补丁字段不完整或不合法，已安全回退全量安装",
        "中性通道清单禁止包含 patch",
        "RSA 签名验证失败",
        "UTC 新鲜度合约失败",
        "manifestPublishedAtUtc",
        "manifestExpiresAtUtc",
        "HighestSeenUpdateVersion",
        "疑似旧清单回放",
        "PatchFailedForVersion",
        "曾在退出后失败，本次强制改用全量安装包",
    ):
        require(token in manifest, f"manifest fail-closed gate missing: {token}")
    require("缺该字段时按旧行为" not in manifest,
            "patch still accepts a missing baseMinVersion")
    signature_gate = manifest.find("VerifyOtaManifestSignature(obj)")
    freshness_gate = manifest.find("ValidateOtaManifestFreshness(")
    watermark_read = manifest.find("HighestSeenUpdateVersion")
    notes_gate = manifest.find("IsValidOtaNotes(notes)")
    patch_gate = manifest.find("const bool validPatch")
    watermark_write = manifest.find("SetHighestSeenUpdateVersion")
    offer_state = manifest.find("m_remoteVersion = remoteVersion")
    require(0 <= signature_gate < freshness_gate < watermark_read < notes_gate < patch_gate
            < watermark_write < offer_state,
            "watermark must be committed only after signature, freshness, notes, and patch semantics")
    require('/latest-v3.json' in app and 'latest.json remains the signed v2' in app,
            "current client does not use the separate v3 endpoint during compatibility migration")
    require('setReadBufferSize(kMaximumManifestBytes + 1)' in app
            and 'otaManifestSizeRejected' in app,
            "manifest response is not bounded while being received")

    sample = {
        "schemaVersion": SCHEMA_VERSION,
        "signatureAlgorithm": SIGNATURE_ALGORITHM,
        "channel": "brand",
        "version": "2026.07.12.0030",
        "publishedAtUtc": "2026-07-12T00:30:00Z",
        "expiresAtUtc": "2026-07-19T00:30:00Z",
        "file": "HK-Pathlynx-CORPLA-Setup-v2026.07.12.0030.exe",
        "sha256": "a" * 64,
        "size": 86657296,
        "notes": "签名回归",
        "patch": {
            "file": "HK-Pathlynx-CORPLA-Patch-v2026.07.12.0030.zip",
            "sha256": "b" * 64,
            "size": 3456789,
            "baseMinVersion": "2026.07.10.1750",
        },
    }
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    signature = key.sign(signature_payload(sample), padding.PKCS1v15(), hashes.SHA256())
    encoded = base64.b64encode(signature).decode("ascii")
    require(verify_manifest(sample, encoded, key.public_key()),
            "Python signer/verifier rejected a valid canonical manifest")
    tampered = dict(sample)
    tampered["size"] += 1
    require(not verify_manifest(tampered, encoded, key.public_key()),
            "manifest signature did not bind installer size")
    tampered_time = dict(sample)
    tampered_time["expiresAtUtc"] = "2026-07-19T00:30:01Z"
    require(not verify_manifest(tampered_time, encoded, key.public_key()),
            "manifest signature did not bind freshness timestamps")
    require(MANIFEST_VALIDITY_SECONDS == 7 * 24 * 60 * 60
            and MANIFEST_CLOCK_SKEW_SECONDS == 10 * 60,
            "Python freshness constants drifted from the reviewed 7-day/10-minute TOFU contract")
    require(cng_public_blob(key.public_key())[:4] == b"RSA1",
            "generated client public-key blob is not a CNG RSA1 blob")

    known_hash_match = re.search(
        r'kOtaKnownAnswerPayloadSha256\[\]\s*=\s*\n?\s*"([0-9a-f]{64})"', app
    )
    known_signature_match = re.search(
        r'kOtaKnownAnswerSignatureBase64\[\]\s*=\s*\n?\s*"([A-Za-z0-9+/=]+)"', app
    )
    require(known_hash_match is not None and known_signature_match is not None,
            "C++ OTA canonical known-answer vector is missing")
    require(hashlib.sha256(signature_payload(sample)).hexdigest() == known_hash_match.group(1),
            "Python canonical payload drifted from the C++ known-answer digest")
    magic, bit_length, exponent_size, modulus_size, prime1_size, prime2_size = \
        struct.unpack("<LLLLLL", embedded_blob[:24])
    require(magic == 0x31415352 and bit_length == modulus_size * 8
            and prime1_size == 0 and prime2_size == 0,
            "embedded CNG RSA public blob header is invalid")
    exponent_start = 24
    modulus_start = exponent_start + exponent_size
    production_public_key = rsa.RSAPublicNumbers(
        int.from_bytes(embedded_blob[exponent_start:modulus_start], "big"),
        int.from_bytes(embedded_blob[modulus_start:modulus_start + modulus_size], "big"),
    ).public_key()
    require(verify_manifest(sample, known_signature_match.group(1), production_public_key),
            "production public key rejected the fixed cross-language manifest vector")

    download = section(
        app,
        "void OnlineServicesDialog::OnDownloadFinished(",
        "void OnlineServicesDialog::InstallDownloadedPackage()",
    )
    for token in (
        "QNetworkRequest::ContentLengthHeader",
        "contentLength != expectedSize",
        "transferred != expectedSize",
        "QFileInfo(partPath).size() != expectedSize",
        "HashFileSha256(partPath)",
        "IsStrictSha256(expectedSha)",
        "actual != expectedSha",
    ):
        require(token in download, f"download size/hash gate missing: {token}")
    require("跳过校验" not in download, "missing SHA256 still has a pass-through path")
    require('setReadBufferSize(1024 * 1024)' in app and 'otaPayloadSink' in app,
            "payload is not streamed through a bounded reply buffer")
    start_download = section(
        app,
        "void OnlineServicesDialog::StartDownload()",
        "void OnlineServicesDialog::OnDownloadFinished(",
    )
    require(start_download.find("ValidateOtaFreshnessWindow(")
            < start_download.find("m_network->get(request)"),
            "download can start without re-checking expiry/system-clock drift")
    for token in (
        'otaWasPatch',
        'otaVersion',
        'otaChannel',
        'otaFileName',
        'otaExpectedSha256',
        'm_checkingForUpdate',
        '更新文件正在下载，完成或失败前禁止刷新清单',
    ):
        require(token in app or token in header,
                f"download/manifest asynchronous state is not pinned: {token}")
    require('FallbackToFullDownload(reason)' in download
            and '已自动切换为签发清单中的全量安装包' in download,
            "brand patch failures do not switch to the signed full installer")

    install = section(
        app,
        "void OnlineServicesDialog::InstallDownloadedPackage()",
        "void OnlineServicesDialog::AppendLog(",
    )
    renderer = section(
        app,
        "QString RenderNamedBatchTemplate(",
        "bool IsAllowedOtaUrl(",
    )
    for token in (
        "script.count(marker) <= 0",
        "value.contains(markerPrefix)",
        "script.replace(marker, value)",
        "script.contains(markerPrefix) ? QString() : script",
    ):
        require(token in renderer, f"named batch renderer is not fail-closed: {token}")
    bootstrap_generator = section(
        install,
        "const qint64 currentPid",
        "QFile bootstrap(",
    )
    require(".arg(" not in bootstrap_generator,
            "OTA bootstrap generation still depends on positional QString placeholders")
    for token in (
        "Get-Process -Id",
        ".WaitForExit()",
        "Get-FileHash",
        "ExtractExecutablePatchToStaging",
        "[System.IO.File]::Replace",
        "RenderNamedBatchTemplate",
        "if errorlevel 1 goto",
        "if not errorlevel 0 goto",
        "install_failed.txt",
        "SetPendingUpdateTargetVersion",
        'QStringLiteral("/D")',
        'QStringLiteral("/V:OFF")',
        "scriptBytes.size()",
        "bootstrap.flush()",
        "setNativeArguments",
        '/D /V:OFF /S /C \\\"\\\"%NO_TEACHING_OTA_BOOTSTRAP%\\\"\\\"',
        "RobotOperationLease::AddNewOperationsBlock",
        "RobotOperationLease::RemoveNewOperationsBlock",
        "RobotOperationLease::AnyActive",
        "OTA_PATCH_FAILURE_MARKER",
        "OTA_HEALTH",
        "echo patch_failed",
    ):
        require(token in install, f"full OTA wait/failure gate missing: {token}")
    require(install.find("Get-Process -Id") < install.find("/VERYSILENT"),
            "full installer can start before the running application exits")
    require("tar -xf" not in install,
            "incremental OTA still extracts directly over the live executable")
    require('"%ERRORLEVEL%"' not in install,
            "batch exit checks trust a poisonable ERRORLEVEL environment variable")
    for token in (
        "QTimer::singleShot(15000",
        "patch_healthy_%1.flag",
        "QSaveFile marker",
    ):
        require(token in main_app, f"post-update health handshake missing: {token}")
    require("connect(sink, &QObject::destroyed" in app and "QFile::remove(partPath)" in app,
            "destroying the OTA dialog can leak a partial download file")

    require(
        re.search(rf'#define MyAppGuid "{re.escape(APP_ID)}"', installer) is not None,
        "installer MyAppGuid drifted from the published identity",
    )
    require(f"AppId={{{{{APP_ID}}}" in installer,
            "installer AppId drifted from the published identity")

    print("PASS: OTA v3 freshness/signature, payload size/hash, /DIR, and AppId gates")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        raise SystemExit(1)
