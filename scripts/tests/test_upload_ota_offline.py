#!/usr/bin/env python3
"""upload_ota.py 离线门禁与双通道原子发布测试；不建立任何网络连接。"""

from __future__ import annotations

import errno
import base64
import contextlib
import datetime as dt
import importlib.util
import io
import json
import os
import re
import struct
import subprocess
import sys
import tempfile
import types
import unittest
import uuid
from unittest import mock
import zipfile
from pathlib import Path
from functools import lru_cache

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding, rsa


sys.dont_write_bytecode = True
REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / "scripts" / "upload_ota.py"
SPEC = importlib.util.spec_from_file_location("upload_ota_offline", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
ota = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ota)
signing = ota._load_signing_module()


@lru_cache(maxsize=None)
def _compiled_test_pe_bytes(filename: str, version: str, product_name: str) -> bytes:
    """Build one real x64 PE with VersionInfo, then cache its bytes for all offline cases."""
    with tempfile.TemporaryDirectory(prefix="ota-real-pe-fixture-") as temp:
        root = Path(temp)
        output = root / filename
        compiler = root / "compile_fixture.ps1"
        compiler.write_text(
            r'''param(
    [Parameter(Mandatory = $true)][string]$OutputPath,
    [Parameter(Mandatory = $true)][string]$Version,
    [Parameter(Mandatory = $true)][string]$ProductName
)
$ErrorActionPreference = "Stop"
$escapedProduct = $ProductName.Replace('\', '\\').Replace('"', '\"')
$escapedVersion = $Version.Replace('\', '\\').Replace('"', '\"')
$code = @"
using System;
using System.Reflection;
[assembly: AssemblyTitle("Trusted publish fixture")]
[assembly: AssemblyProduct("$escapedProduct")]
[assembly: AssemblyVersion("$escapedVersion")]
[assembly: AssemblyFileVersion("$escapedVersion")]
[assembly: AssemblyInformationalVersion("$escapedVersion")]
public static class Program { public static int Main() { return 0; } }
"@
$provider = New-Object Microsoft.CSharp.CSharpCodeProvider
try {
    $parameters = New-Object System.CodeDom.Compiler.CompilerParameters
    $parameters.CompilerOptions = "/platform:x64 /target:winexe"
    $parameters.OutputAssembly = $OutputPath
    $parameters.GenerateExecutable = $true
    $result = $provider.CompileAssemblyFromSource($parameters, $code)
    if ($result.Errors.HasErrors) {
        throw "C# PE fixture compilation failed: $($result.Errors | Out-String)"
    }
}
finally {
    $provider.Dispose()
}
''',
            encoding="utf-8",
        )
        command = [
            str(ota._trusted_windows_powershell()),
            "-NoLogo", "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
            "-File", str(compiler),
            "-OutputPath", str(output),
            "-Version", version,
            "-ProductName", product_name,
        ]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=120, check=False)
        if completed.returncode != 0 or not output.is_file():
            raise AssertionError(
                "real PE fixture compilation failed:\n" + completed.stdout + completed.stderr
            )
        return output.read_bytes()


def _assert_real_x64_pe(path: Path) -> None:
    data = path.read_bytes()
    if len(data) < 0x100 or data[:2] != b"MZ":
        raise AssertionError(f"not a real DOS/PE image: {path}")
    pe_offset = struct.unpack_from("<I", data, 0x3C)[0]
    if pe_offset + 6 > len(data) or data[pe_offset:pe_offset + 4] != b"PE\0\0":
        raise AssertionError(f"missing PE signature: {path}")
    machine = struct.unpack_from("<H", data, pe_offset + 4)[0]
    if machine != 0x8664:
        raise AssertionError(f"not x64 PE: {path}, machine=0x{machine:04x}")


def make_test_signer():
    private_key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
    public_key = private_key.public_key()

    def signer(manifest):
        signature = private_key.sign(
            signing.signature_payload(manifest), padding.PKCS1v15(), hashes.SHA256()
        )
        encoded = base64.b64encode(signature).decode("ascii")
        if not signing.verify_manifest(manifest, encoded, public_key):
            raise AssertionError("test signer self-check failed")
        return encoded

    def verify(manifest):
        return signing.verify_manifest(manifest, manifest.get("signature", ""), public_key)

    def sign_legacy_v2(manifest):
        signature = private_key.sign(
            signing.legacy_signature_payload_v2(manifest),
            padding.PKCS1v15(),
            hashes.SHA256(),
        )
        return base64.b64encode(signature).decode("ascii")

    signer.verify = verify
    signer.sign_legacy_v2 = sign_legacy_v2
    signer.sign_legacy = sign_legacy_v2
    signer.public_key = public_key
    return signer


class _FakeStat:
    def __init__(self, size: int):
        self.st_size = size


class FakeSftp:
    def __init__(self):
        self.files: dict[str, bytes] = {}
        self.directories: set[str] = set()
        self.operations: list[tuple[str, str, str | None]] = []
        self.fail_rename_once: set[str] = set()
        self.fail_rmdir_once: set[str] = set()

    def mkdir(self, path: str):
        self.operations.append(("mkdir", path, None))
        if path in self.directories:
            raise OSError(errno.EEXIST, "exists", path)
        self.directories.add(path)

    def rmdir(self, path: str):
        self.operations.append(("rmdir", path, None))
        if path in self.fail_rmdir_once:
            self.fail_rmdir_once.remove(path)
            raise OSError(errno.EACCES, "injected rmdir failure", path)
        if path not in self.directories:
            raise FileNotFoundError(errno.ENOENT, "missing", path)
        self.directories.remove(path)

    def stat(self, path: str):
        if path not in self.files:
            raise FileNotFoundError(errno.ENOENT, "missing", path)
        return _FakeStat(len(self.files[path]))

    def open(self, path: str, mode: str):
        if "r" in mode:
            if path not in self.files:
                raise FileNotFoundError(errno.ENOENT, "missing", path)
            return io.BytesIO(self.files[path])
        raise AssertionError(f"unexpected fake open mode: {mode}")

    def put(self, local: str, remote: str):
        self.operations.append(("put", remote, local))
        self.files[remote] = Path(local).read_bytes()

    def putfo(self, stream, remote: str, file_size: int = 0):
        self.operations.append(("putfo", remote, str(file_size)))
        self.files[remote] = stream.read()

    def remove(self, path: str):
        if path not in self.files:
            raise FileNotFoundError(errno.ENOENT, "missing", path)
        self.operations.append(("remove", path, None))
        del self.files[path]

    def posix_rename(self, source: str, destination: str):
        self.operations.append(("rename", source, destination))
        if destination in self.fail_rename_once:
            self.fail_rename_once.remove(destination)
            raise OSError(errno.EIO, "injected rename failure", destination)
        if source not in self.files:
            raise FileNotFoundError(errno.ENOENT, "missing", source)
        self.files[destination] = self.files.pop(source)


class CandidateFixture:
    VERSION = "2026.07.12.1200"

    def __init__(self, root: Path, *, real_pe: bool = False):
        self.root = root
        self.real_pe = real_pe
        self.neutral_dist = root / "neutral-dist"
        self.brand_dist = root / "brand-dist"
        self.neutral_dist.mkdir()
        self.brand_dist.mkdir()
        self.neutral_exe = self.neutral_dist / ota.CHANNEL_EXE["neutral"]
        self.brand_exe = self.brand_dist / ota.CHANNEL_EXE["brand"]
        if real_pe:
            self.neutral_exe.write_bytes(_compiled_test_pe_bytes(
                self.neutral_exe.name, self.VERSION, "NoTeaching-Robot"
            ))
            self.brand_exe.write_bytes(_compiled_test_pe_bytes(
                self.brand_exe.name, self.VERSION, "HK-Pathlynx-CORPLA"
            ))
        else:
            self.neutral_exe.write_bytes(b"MZ-neutral-" + self.VERSION.encode("ascii"))
            self.brand_exe.write_bytes(b"MZ-brand-" + self.VERSION.encode("ascii"))
        (self.neutral_dist / "Qt6Core.dll").write_bytes(b"shared-runtime")
        (self.brand_dist / "Qt6Core.dll").write_bytes(b"shared-runtime")
        for dist in (self.neutral_dist, self.brand_dist):
            (dist / "SDK" / "FANUC").mkdir(parents=True)
            for index in range(ota.EXPECTED_FANUC_TP_COUNT):
                (dist / "SDK" / "FANUC" / f"runtime-{index:02d}.tp").write_bytes(f"tp-{index}".encode())
            for index in range(ota.EXPECTED_FANUC_PC_COUNT):
                (dist / "SDK" / "FANUC" / f"runtime-{index:02d}.pc").write_bytes(f"pc-{index}".encode())
        (self.brand_dist / "branding").mkdir()
        (self.brand_dist / "branding" / "app_color.ico").write_bytes(b"color-icon")
        (self.brand_dist / "branding" / "app_nobg.ico").write_bytes(b"nobg-icon")

        self.neutral_installer = root / ota._expected_installer_name("neutral", self.VERSION)
        self.brand_installer = root / ota._expected_installer_name("brand", self.VERSION)
        self.brand_patch = root / ota._expected_patch_name("brand", self.VERSION)
        if real_pe:
            self.neutral_installer.write_bytes(_compiled_test_pe_bytes(
                self.neutral_installer.name, self.VERSION, "NoTeaching-Robot"
            ))
            self.brand_installer.write_bytes(_compiled_test_pe_bytes(
                self.brand_installer.name, self.VERSION, "HK-Pathlynx-CORPLA"
            ))
        else:
            self.neutral_installer.write_bytes(b"neutral-installer")
            self.brand_installer.write_bytes(b"brand-installer")
        with zipfile.ZipFile(self.brand_patch, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.write(self.brand_exe, arcname=self.brand_exe.name)

    @staticmethod
    def _write_json(path: Path, value):
        path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True), encoding="utf-8")

    def make_pair_gate(self) -> Path:
        scripts_dir = self.root / "scripts"
        scripts_dir.mkdir(exist_ok=True)
        for script_name in (
            "build_installer.ps1",
            "build_release_package.ps1",
            "build_config_migrate.ps1",
            "release_gate_common.ps1",
            "verify_release_pair.ps1",
            "upload_ota.py",
            "ota_manifest_signing.py",
        ):
            (scripts_dir / script_name).write_bytes((REPO_ROOT / "scripts" / script_name).read_bytes())
        gate_common_sha = ota.sha256_file(scripts_dir / "release_gate_common.ps1")
        installer_producer_sha = ota.sha256_file(scripts_dir / "build_installer.ps1")
        package_producer_sha = ota.sha256_file(scripts_dir / "build_release_package.ps1")
        fanuc_sha = "f" * 64
        migrator_sha = "c" * 64
        source_sha = "d" * 64
        migrate_run_sha = "e" * 64
        migrate_install_sha = "9" * 64
        heads = {"neutral": "a" * 40, "brand": "b" * 40}
        channel_nodes = {}
        for channel, dist_dir, installer_path in (
            ("neutral", self.neutral_dist, self.neutral_installer),
            ("brand", self.brand_dist, self.brand_installer),
        ):
            dist = ota.inspect_dist(dist_dir, channel, self.VERSION)
            installer = ota._inspect_installer(installer_path, channel, self.VERSION)
            package_path = self.root / f"package-{channel}.json"
            package = {
                "schemaVersion": 1,
                "kind": "package",
                "status": "pass",
                "runId": f"package-{channel}",
                "gateScriptSha256": gate_common_sha,
                "producerScriptSha256": package_producer_sha,
                "appId": ota.PUBLISHED_APP_ID,
                "version": self.VERSION,
                "channel": channel,
                "head": heads[channel],
                "repoRoot": str(self.root),
                "packageDir": str(dist_dir.resolve()),
                "expectedExe": dist["mainExe"],
                "executableSha256": dist["mainExeSha256"],
                "fanucManifest": {"sha256": fanuc_sha, "fileCount": 21},
                "configMigrate": {"sha256": migrator_sha, "sourceSha256": source_sha},
                "configMigrateRun": {"path": str(self.root / "ConfigMigrate_Run.cmd"),
                                     "sha256": migrate_run_sha},
                "configMigrateInstall": {
                    "path": str(self.root / "ConfigMigrate_Install.ps1"),
                    "sha256": migrate_install_sha,
                },
                "packageInventory": dist["inventory"],
            }
            self._write_json(package_path, package)
            installer_gate_path = self.root / f"installer-{channel}.json"
            installer_gate = {
                "schemaVersion": 1,
                "kind": "installer",
                "status": "pass",
                "runId": f"installer-{channel}",
                "gateScriptSha256": gate_common_sha,
                "producerScriptSha256": installer_producer_sha,
                "appId": ota.PUBLISHED_APP_ID,
                "version": self.VERSION,
                "channel": channel,
                "head": heads[channel],
                "repoRoot": str(self.root),
                "packageGateReport": str(package_path.resolve()),
                "packageGateReportSha256": ota.sha256_file(package_path),
                "installer": {
                    "path": installer["path"],
                    "name": installer["file"],
                    "size": installer["size"],
                    "sha256": installer["sha256"],
                    "productVersion": self.VERSION,
                },
            }
            self._write_json(installer_gate_path, installer_gate)
            channel_nodes[channel] = {
                "head": heads[channel],
                "repoRoot": str(self.root.resolve()),
                "installerGateReport": str(installer_gate_path.resolve()),
                "installerGateSha256": ota.sha256_file(installer_gate_path),
                "installerSha256": installer["sha256"],
                "installerSize": installer["size"],
            }

        pair_path = self.root / "release-pair-gate.json"
        pair = {
            "schemaVersion": 1,
            "kind": "release-pair",
            "status": "pass",
            "runId": "pair-run",
            "gateScriptSha256": gate_common_sha,
            "producerScriptSha256": ota.sha256_file(scripts_dir / "verify_release_pair.ps1"),
            "appId": ota.PUBLISHED_APP_ID,
            "version": self.VERSION,
            "maxInstallerSizeDifferenceBytes": ota.MAX_INSTALLER_SIZE_DELTA,
            "installerSizeDifferenceBytes": abs(
                self.neutral_installer.stat().st_size - self.brand_installer.stat().st_size
            ),
            "commonInventoryFileCount": 1,
            "fanucManifestSha256": fanuc_sha,
            "configMigrateSha256": migrator_sha,
            "configMigrateInstallSha256": migrate_install_sha,
            "neutral": channel_nodes["neutral"],
            "brand": channel_nodes["brand"],
        }
        self._write_json(pair_path, pair)
        self.pair_gate = pair_path
        return pair_path

    def prepare(self):
        pair_gate = self.make_pair_gate()
        return ota.prepare_dual_candidate(
            self.VERSION,
            self.neutral_dist,
            self.neutral_installer,
            self.brand_dist,
            self.brand_installer,
            self.brand_patch,
            pair_gate,
            "offline test",
        )

    def trust(self, report, mutate_attestation=None):
        runner = TrustedVerifierFixture(self, report, mutate_attestation=mutate_attestation)
        trusted = ota.trust_candidate_for_publish(report, _runner=runner)
        self.last_trusted_runner = runner
        return trusted


class TrustedVerifierFixture:
    """A subprocess-shaped test double for the one-shot trusted verifier protocol.

    It is intentionally usable only through ``trust_candidate_for_publish``: the production
    publish entry has no CLI flag or environment variable for replacing its fixed verifier.
    Positive remote tests use real x64 PE images, while the explicit negative test retains the
    historical fake-MZ/fake-installer/self-written-PASS fixture.
    """

    def __init__(self, fixture: CandidateFixture, report, *, mutate_attestation=None):
        self.fixture = fixture
        self.report = report
        self.mutate_attestation = mutate_attestation
        self.calls = 0

    @staticmethod
    def _argument(command, name: str) -> str:
        try:
            return command[command.index(name) + 1]
        except (ValueError, IndexError) as exc:
            raise AssertionError(f"trusted verifier command missing {name}") from exc

    def __call__(self, command, **_kwargs):
        self.calls += 1
        verifier_path = Path(self._argument(command, "-File")).resolve()
        if verifier_path != (self.fixture.root / "scripts" / "verify_release_pair.ps1").resolve():
            raise AssertionError(f"unexpected trusted verifier: {verifier_path}")
        if "-NoProfile" not in command or "-NonInteractive" not in command:
            raise AssertionError("trusted verifier was not launched profile-free/non-interactively")

        for path in (
            self.fixture.neutral_exe,
            self.fixture.brand_exe,
            self.fixture.neutral_installer,
            self.fixture.brand_installer,
        ):
            _assert_real_x64_pe(path)

        challenge = self._argument(command, "-PublishChallenge")
        candidate_sha = self._argument(command, "-ExpectedCandidateSha256")
        pair_sha = self._argument(command, "-ExpectedPairGateSha256")
        output = Path(self._argument(command, "-AttestationOutputPath"))
        pair = self.report["pairGate"]
        attestation = {
            "schemaVersion": 1,
            "kind": "publish-attestation",
            "status": "pass",
            "challenge": challenge,
            "candidateSha256": candidate_sha,
            "generatedAtUtc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "verifierProcessId": os.getpid(),
            "verifierScriptSha256": ota.sha256_file(
                self.fixture.root / "scripts" / "verify_release_pair.ps1"
            ),
            "gateScriptSha256": ota.sha256_file(
                self.fixture.root / "scripts" / "release_gate_common.ps1"
            ),
            "appId": ota.PUBLISHED_APP_ID,
            "version": self.report["version"],
            "pairGate": {
                "path": pair["path"],
                "sha256": pair_sha,
                "runId": pair["runId"],
            },
        }
        for channel in ota.CHANNELS:
            pair_channel = pair[channel]
            installer = self.report["channels"][channel]["installer"]
            attestation[channel] = {
                "head": pair_channel["head"],
                "installerGateReport": pair_channel["installerGateReport"],
                "installerGateSha256": pair_channel["installerGateSha256"],
                "packageGateReport": pair_channel["packageGateReport"],
                "packageGateSha256": pair_channel["packageGateSha256"],
                "installerSha256": installer["sha256"],
                "installerSize": installer["size"],
            }
        if self.mutate_attestation is not None:
            self.mutate_attestation(attestation)
        output.write_text(
            json.dumps(attestation, ensure_ascii=False, sort_keys=True), encoding="utf-8"
        )
        return subprocess.CompletedProcess(command, 0, stdout="PASS: trusted fixture\n", stderr="")


class LocalGateTests(unittest.TestCase):
    def test_strict_version_rejects_malformed_or_impossible_values(self):
        for value in ("v2026.07.12.1200", "2026.7.12.1200", "2026.02.30.1200", "2026.07.12.bad"):
            with self.subTest(value=value), self.assertRaises(ota.ReleaseGateError):
                ota.parse_version(value)

    def test_missing_empty_and_exe_only_dist_fail_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.assertRaises(ota.ReleaseGateError):
                ota.inspect_dist(root / "missing", "neutral", CandidateFixture.VERSION)
            empty = root / "empty"
            empty.mkdir()
            with self.assertRaises(ota.ReleaseGateError):
                ota.inspect_dist(empty, "neutral", CandidateFixture.VERSION)
            exe_only = root / "exe-only"
            exe_only.mkdir()
            (exe_only / ota.CHANNEL_EXE["neutral"]).write_bytes(
                b"MZ" + CandidateFixture.VERSION.encode("ascii")
            )
            with self.assertRaises(ota.ReleaseGateError):
                ota.inspect_dist(exe_only, "neutral", CandidateFixture.VERSION)

    def test_main_exe_must_be_unique_and_embed_target_version(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            (fixture.neutral_dist / "rogue.exe").write_bytes(b"rogue")
            with self.assertRaises(ota.ReleaseGateError):
                ota.inspect_dist(fixture.neutral_dist, "neutral", fixture.VERSION)
            (fixture.neutral_dist / "rogue.exe").unlink()
            fixture.neutral_exe.write_bytes(b"MZ-old-version")
            with self.assertRaises(ota.ReleaseGateError):
                ota.inspect_dist(fixture.neutral_dist, "neutral", fixture.VERSION)

    def test_installer_and_patch_names_are_exact(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            wrong_installer = fixture.root / "neutral.exe"
            wrong_installer.write_bytes(b"installer")
            with self.assertRaises(ota.ReleaseGateError):
                ota._inspect_installer(wrong_installer, "neutral", fixture.VERSION)
            wrong_patch = fixture.root / "brand.zip"
            wrong_patch.write_bytes(fixture.brand_patch.read_bytes())
            dist = ota.inspect_dist(fixture.brand_dist, "brand", fixture.VERSION)
            with self.assertRaises(ota.ReleaseGateError):
                ota._inspect_patch(wrong_patch, "brand", fixture.VERSION, dist)

    def test_patch_must_contain_only_the_exact_candidate_exe(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            with zipfile.ZipFile(fixture.brand_patch, "a", zipfile.ZIP_DEFLATED) as archive:
                archive.writestr("extra.dll", b"extra")
            dist = ota.inspect_dist(fixture.brand_dist, "brand", fixture.VERSION)
            with self.assertRaises(ota.ReleaseGateError):
                ota._inspect_patch(fixture.brand_patch, "brand", fixture.VERSION, dist)

    def test_report_revalidation_detects_post_prepare_mutation(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            report = fixture.prepare()
            report_path = fixture.root / "candidate.json"
            ota._write_json_atomic(report_path, report, False)
            fixture.brand_exe.write_bytes(fixture.brand_exe.read_bytes() + b"mutated")
            with self.assertRaises(ota.ReleaseGateError):
                ota.load_and_revalidate_report(report_path)

    def test_pair_gate_or_referenced_gate_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            report = fixture.prepare()
            report_path = fixture.root / "candidate.json"
            ota._write_json_atomic(report_path, report, False)
            pair = json.loads(fixture.pair_gate.read_text(encoding="utf-8"))
            neutral_gate = Path(pair["neutral"]["installerGateReport"])
            neutral_gate.write_text(neutral_gate.read_text(encoding="utf-8") + " ", encoding="utf-8")
            with self.assertRaises(ota.ReleaseGateError):
                ota.load_and_revalidate_report(report_path)

    def test_fake_mz_fake_installer_and_handwritten_pass_have_no_publish_authority(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            report = fixture.prepare()
            sftp = FakeSftp()
            with self.assertRaisesRegex(ota.ReleaseGateError, "普通/手写候选 JSON"):
                ota.publish_dual_with_sftp(sftp, report, lambda _manifest: "unused")
            self.assertEqual(sftp.operations, [], "untrusted JSON reached the remote publish lock")

            report_path = fixture.root / "handwritten-candidate.json"
            ota._write_json_atomic(report_path, report, False)
            with self.assertRaisesRegex(ota.ReleaseGateError, "受信任发布验证器拒绝"):
                ota.load_and_trust_publish_report(report_path)
            self.assertEqual(sftp.operations, [], "failed trusted verification performed remote I/O")

    def test_real_pe_fixture_gets_fresh_candidate_bound_capability_and_replay_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp), real_pe=True)
            report = fixture.prepare()
            trusted = fixture.trust(report)
            self.assertIsInstance(trusted, ota._TrustedPublishCandidate)
            self.assertEqual(fixture.last_trusted_runner.calls, 1)
            for path in (
                fixture.neutral_exe,
                fixture.brand_exe,
                fixture.neutral_installer,
                fixture.brand_installer,
            ):
                _assert_real_x64_pe(path)

            with self.assertRaisesRegex(ota.ReleaseGateError, "challenge"):
                fixture.trust(
                    report,
                    mutate_attestation=lambda evidence: evidence.__setitem__(
                        "challenge", "0" * 64
                    ),
                )

    def test_cross_channel_runtime_or_multi_megabyte_size_delta_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            (fixture.brand_dist / "Qt6Core.dll").write_bytes(b"different-runtime")
            with self.assertRaises(ota.ReleaseGateError):
                fixture.prepare()
        with tempfile.TemporaryDirectory() as temp:
            fixture = CandidateFixture(Path(temp))
            fixture.brand_installer.write_bytes(b"x" * (ota.MAX_INSTALLER_SIZE_DELTA + 100))
            with self.assertRaises(ota.ReleaseGateError):
                fixture.prepare()

    def test_legacy_single_channel_and_raw_hash_entry_is_disabled(self):
        with self.assertRaises(ota.ReleaseGateError):
            ota.main(["--version", CandidateFixture.VERSION, "--channel", "neutral"])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            ota._build_parser().parse_args(["publish-dual", "--seed-versions", "x"])

    def test_known_hosts_must_be_outside_repository_and_auto_add_is_absent(self):
        with self.assertRaises(ota.ReleaseGateError):
            ota._validate_known_hosts_path(str(MODULE_PATH))
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn("RejectPolicy", source)
        self.assertNotIn("AutoAddPolicy", source)

    def test_local_children_drop_release_secrets_and_use_devnull_stdin(self):
        build_injection_names = [
            "CL", "_CL_", "LINK", "_LINK_", "INCLUDE", "LIB", "LIBPATH",
            "QTDIR", "QMAKEFEATURES", "QT_PLUGIN_PATH", "MSBuildSDKsPath",
            "VSINSTALLDIR", "VCINSTALLDIR", "VCToolsInstallDir",
            "WindowsSdkDir", "UniversalCRTSdkDir", "ExtensionSdkDir",
            "FrameworkDir", "DevEnvDir", "VisualStudioVersion",
            "CustomBeforeMicrosoftCommonTargets", "CustomAfterMicrosoftCommonTargets",
            "ForceImportBeforeCppTargets", "ForceImportAfterCppTargets",
            "DirectoryBuildPropsPath", "DirectoryBuildTargetsPath",
            "TOTALLY_UNKNOWN_RELEASE_ATTACK",
        ]
        command = [
            sys.executable,
            "-c",
            "import json,os,sys; print(json.dumps({"
            "'ota':os.environ.get('OTA_SSH_PASSWORD'),"
            "'gh':os.environ.get('GH_TOKEN'),"
            "'gh_host':os.environ.get('GH_HOST'),"
            "'git_ssh':os.environ.get('GIT_SSH_COMMAND'),"
            "'git_global':os.environ.get('GIT_CONFIG_GLOBAL'),"
            "'system_root':os.environ.get('SystemRoot'),"
            "'comspec':os.environ.get('ComSpec'),"
            f"'build':sorted(k for k in os.environ if k in {build_injection_names!r}),"
            "'path':os.environ.get('PATH'),"
            "'stdin':sys.stdin.read()}))",
        ]
        with mock.patch.dict(os.environ, {
            "OTA_SSH_PASSWORD": "must-never-reach-child",
            "GH_TOKEN": "must-not-reach-local-gate",
            "GH_HOST": "attacker.invalid",
            "GIT_SSH_COMMAND": str(REPO_ROOT / "attacker" / "ssh.exe"),
            "GIT_CONFIG_GLOBAL": str(REPO_ROOT / "attacker" / "gitconfig"),
            "PYTHONPATH": str(REPO_ROOT / "attacker"),
            "PATH": str(REPO_ROOT / "attacker"),
            "SystemRoot": str(REPO_ROOT / "attacker"),
            "ComSpec": str(REPO_ROOT / "attacker" / "cmd.exe"),
            **{name: str(REPO_ROOT / "attacker") for name in build_injection_names},
        }, clear=False):
            completed = ota._run_local_checked(
                command, cwd=REPO_ROOT, label="sanitized child attack test"
            )
        observed = json.loads(completed.stdout)
        self.assertIsNone(observed["ota"])
        self.assertIsNone(observed["gh"])
        self.assertIsNone(observed["gh_host"])
        self.assertIsNone(observed["git_ssh"])
        self.assertIn(observed["git_global"], {"NUL", "/dev/null"})
        self.assertNotIn(str(REPO_ROOT / "attacker"), observed["system_root"])
        self.assertNotIn(str(REPO_ROOT / "attacker"), observed["comspec"])
        self.assertEqual(observed["build"], [])
        self.assertEqual(observed["stdin"], "")
        self.assertNotIn(str(REPO_ROOT / "attacker"), observed["path"])
        with mock.patch.dict(os.environ, {
            "GH_TOKEN": "phase-scoped-token",
            "GH_HOST": "attacker.invalid",
            "GIT_SSH_COMMAND": "attacker-ssh",
        }, clear=False):
            github_environment = ota._sanitized_local_environment(
                allow_github_credentials=True
            )
        self.assertEqual(github_environment["GH_TOKEN"], "phase-scoped-token")
        self.assertEqual(github_environment["GH_HOST"], "github.com")
        self.assertNotIn("GIT_SSH_COMMAND", github_environment)

    def test_password_is_never_accepted_from_startup_environment(self):
        args = types.SimpleNamespace(ssh_password_file=None, ssh_password_stdin=True)
        with mock.patch.dict(os.environ, {"OTA_SSH_PASSWORD": "legacy-secret"}, clear=False), \
                self.assertRaisesRegex(ota.ReleaseGateError, "永久禁用|已永久禁用"):
            ota._read_ssh_password_after_gates(args)
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(sys, "stdin", io.StringIO("stdin-secret\n")):
            self.assertEqual(ota._read_ssh_password_after_gates(args), "stdin-secret")

    def test_password_acl_rejects_other_sids_deny_and_inheritance(self):
        current = "S-1-5-21-100-200-300-1001"
        secure = {
            "ownerSid": current,
            "currentSid": current,
            "protected": True,
            "currentCanRead": True,
            "rules": [
                {"sid": current, "type": "Allow", "inherited": False},
                {"sid": "S-1-5-18", "type": "Allow", "inherited": False},
                {"sid": "S-1-5-32-544", "type": "Allow", "inherited": False},
            ],
            "sha256": "a" * 64,
        }
        ota._validate_windows_private_acl_metadata(secure)
        attacks = []
        other_sid = json.loads(json.dumps(secure))
        other_sid["rules"].append({
            "sid": "S-1-5-21-999-888-777-1002", "type": "Allow", "inherited": False,
        })
        attacks.append(other_sid)
        deny = json.loads(json.dumps(secure))
        deny["rules"][0]["type"] = "Deny"
        attacks.append(deny)
        inherited = json.loads(json.dumps(secure))
        inherited["rules"][0]["inherited"] = True
        attacks.append(inherited)
        wrong_owner = json.loads(json.dumps(secure))
        wrong_owner["ownerSid"] = "S-1-5-21-999-888-777-1002"
        attacks.append(wrong_owner)
        ambiguous_parent = json.loads(json.dumps(secure))
        ambiguous_parent["protected"] = False
        attacks.append(ambiguous_parent)
        for attack in attacks:
            with self.subTest(attack=attack), self.assertRaises(ota.ReleaseGateError):
                ota._validate_windows_private_acl_metadata(attack)

    def test_production_release_rejects_known_hosts_in_favor_of_fingerprint(self):
        args = types.SimpleNamespace(
            version=CandidateFixture.VERSION,
            notes="",
            server=ota.PRODUCTION_HOST,
            port=ota.PRODUCTION_PORT,
            user=ota.PRODUCTION_USER,
            known_hosts=str(REPO_ROOT / "known_hosts"),
            host_key_sha256=None,
        )
        builder = mock.Mock(side_effect=AssertionError("builder must not run"))
        with self.assertRaisesRegex(ota.ReleaseGateError, "host-key-sha256"):
            ota.trusted_release_dual(args, _candidate_builder=builder)
        builder.assert_not_called()

    def test_unsigned_path_injected_git_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fake_git = Path(temporary) / "git.exe"
            fake_git.write_bytes(b"MZ-not-signed")
            unsigned = subprocess.CompletedProcess(
                [], 0,
                stdout=json.dumps({"status": "NotSigned", "subject": "", "sha256": "a" * 64}),
                stderr="",
            )
            with mock.patch.object(ota.subprocess, "run", return_value=unsigned), \
                    self.assertRaisesRegex(ota.ReleaseGateError, "Authenticode"):
                ota._resolve_and_verify_trusted_tool(
                    fake_git,
                    name="Git for Windows",
                    expected_basename="git.exe",
                    signer_tokens=("Git for Windows",),
                )

    def test_toolchain_closure_rejects_dependency_drift_links_bounds_and_growth(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            qt_bin = root / "qt" / "bin"
            qt_plugins = root / "qt" / "plugins"
            qt_msbuild = root / "qtmsbuild"
            inno = root / "inno"
            qt_bin.mkdir(parents=True)
            qt_plugins.mkdir(parents=True)
            qt_msbuild.mkdir()
            inno.mkdir()
            qt_core = qt_bin / "Qt6Core.dll"
            qt_core.write_bytes(b"trusted-qt-core")
            (qt_plugins / "qwindows.dll").write_bytes(b"trusted-plugin")
            iscc_dependency = inno / "Setup.e32"
            iscc_dependency.write_bytes(b"trusted-inno-engine")
            qt_msbuild_props = qt_msbuild / "Qt.props"
            qt_msbuild_props.write_bytes(b"trusted-qt-msbuild-props")
            initial_qt = ota._snapshot_bounded_tree(
                "Qt build/deploy", [("bin", qt_bin), ("plugins", qt_plugins)],
                max_entries=32, max_bytes=4096,
            )
            initial_inno = ota._snapshot_bounded_tree(
                "Inno Setup compiler", [("install", inno)],
                max_entries=32, max_bytes=4096,
            )
            initial_qt_msbuild = ota._snapshot_bounded_tree(
                "Qt MSBuild integration", [("install", qt_msbuild)],
                max_entries=32, max_bytes=4096,
            )
            expectations = {
                snapshot.name: {
                    "sha256": snapshot.sha256,
                    "entries": snapshot.entries,
                    "files": snapshot.files,
                    "size": snapshot.size,
                }
                for snapshot in (initial_qt, initial_inno, initial_qt_msbuild)
            }
            qt_core.write_bytes(b"pre-start-attacker-qt-core")
            attacked_qt = ota._snapshot_bounded_tree(
                "Qt build/deploy", [("bin", qt_bin), ("plugins", qt_plugins)],
                max_entries=32, max_bytes=4096,
            )
            with self.assertRaisesRegex(ota.ReleaseGateError, "禁止学习"):
                ota._require_expected_toolchain_closures(
                    (attacked_qt, initial_inno, initial_qt_msbuild), expectations
                )
            qt_core.write_bytes(b"trusted-qt-core")
            iscc_dependency.write_bytes(b"pre-start-attacker-inno-engine")
            attacked_inno = ota._snapshot_bounded_tree(
                "Inno Setup compiler", [("install", inno)],
                max_entries=32, max_bytes=4096,
            )
            with self.assertRaisesRegex(ota.ReleaseGateError, "禁止学习"):
                ota._require_expected_toolchain_closures(
                    (initial_qt, attacked_inno, initial_qt_msbuild), expectations
                )
            iscc_dependency.write_bytes(b"trusted-inno-engine")
            qt_msbuild_props.write_bytes(b"pre-start-attacker-qt-msbuild")
            attacked_qt_msbuild = ota._snapshot_bounded_tree(
                "Qt MSBuild integration", [("install", qt_msbuild)],
                max_entries=32, max_bytes=4096,
            )
            with self.assertRaisesRegex(ota.ReleaseGateError, "禁止学习"):
                ota._require_expected_toolchain_closures(
                    (initial_qt, initial_inno, attacked_qt_msbuild), expectations
                )
            qt_msbuild_props.write_bytes(b"trusted-qt-msbuild-props")
            closure = ota._snapshot_bounded_tree(
                "fixture toolchain",
                [("bin", qt_bin), ("plugins", qt_plugins), ("inno", inno)],
                max_entries=32,
                max_bytes=4096,
            )
            ota._verify_bounded_tree_snapshot(closure)
            qt_core.write_bytes(b"attacker-qt-core")
            with self.assertRaisesRegex(ota.ReleaseGateError, "发生变化"):
                ota._verify_bounded_tree_snapshot(closure)
            qt_core.write_bytes(b"trusted-qt-core")
            iscc_dependency.write_bytes(b"attacker-inno-engine")
            with self.assertRaisesRegex(ota.ReleaseGateError, "发生变化"):
                ota._verify_bounded_tree_snapshot(closure)

            link_like = qt_bin / "link.dll"
            link_like.write_bytes(b"link-target")
            original_reparse_check = ota._path_is_reparse_point
            with mock.patch.object(
                ota,
                "_path_is_reparse_point",
                side_effect=lambda path: Path(path) == link_like
                or original_reparse_check(Path(path)),
            ), self.assertRaisesRegex(ota.ReleaseGateError, "link/reparse"):
                ota._snapshot_bounded_tree(
                    "link fixture", [("bin", qt_bin)],
                    max_entries=32, max_bytes=4096,
                )

            with self.assertRaisesRegex(ota.ReleaseGateError, "最大条目数"):
                ota._snapshot_bounded_tree(
                    "entry fixture", [("bin", qt_bin)],
                    max_entries=2, max_bytes=4096,
                )

            growing = root / "growing.dll"
            growing.write_bytes(b"x")
            real_open = open

            def growing_open(path, *args, **kwargs):
                if Path(path) == growing:
                    return io.BytesIO(b"xx")
                return real_open(path, *args, **kwargs)

            with mock.patch("builtins.open", side_effect=growing_open), \
                    self.assertRaisesRegex(ota.ReleaseGateError, "增长"):
                ota._snapshot_bounded_tree(
                    "growth fixture", [("file", growing)],
                    max_entries=2, max_bytes=4096,
                )

    def test_fresh_trusted_gate_precedes_signing_secrets_socket_and_remote_lock(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        trusted_flow = source[
            source.index("def trusted_release_dual("):
            source.index("def _build_parser()")
        ]
        trusted = trusted_flow.index("trusted_report = trust_factory(")
        origin = trusted_flow.index("origin_verifier(repo_root, trusted_report)")
        github_preflight = trusted_flow.index("github_preflight = github_preflight_factory(")
        ota_publish = trusted_flow.index("ota_result = ota_publisher(")
        github_publish = trusted_flow.index("github_result = github_publisher(")
        self.assertLess(trusted, origin)
        self.assertLess(origin, github_preflight)
        self.assertLess(github_preflight, ota_publish)
        self.assertLess(ota_publish, github_publish)

        ota_production = source[
            source.index("def _publish_ota_production("):
            source.index("def trusted_release_dual(")
        ]
        signing_key = ota_production.index("manifest_signer = signer_factory(")
        ssh_password = ota_production.index("password = _read_ssh_password_after_gates(args)")
        socket_open = ota_production.index("ssh = ssh_factory(")
        self.assertLess(signing_key, ssh_password)
        self.assertLess(ssh_password, socket_open)

        publish_api = source[
            source.index("def publish_dual_with_sftp("):
            source.index("def _known_host_lookup_name(")
        ]
        self.assertLess(
            publish_api.index("_require_trusted_publish_candidate(report)"),
            publish_api.index("sftp.mkdir(lock_path)"),
        )
        self.assertIn('"System32" / "WindowsPowerShell" / "v1.0" / "powershell.exe"', source)
        self.assertNotIn("shell=True", source)

    def test_only_in_process_clean_build_entry_can_reach_release_secrets(self):
        host_key = "SHA256:" + "A" * 43
        with self.assertRaisesRegex(ota.ReleaseGateError, "永久禁用"):
            ota.main([
                "publish-dual", "--report", "handwritten.json",
                "--host-key-sha256", host_key,
            ])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            ota._build_parser().parse_args([
                "trusted-release-dual", "--version", CandidateFixture.VERSION,
                "--runtime-source", ".", "--report", "forged.json",
                "--host-key-sha256", host_key,
            ])

        @contextlib.contextmanager
        def failed_build(_args):
            raise ota.ReleaseGateError("synthetic clean build failure")
            yield  # pragma: no cover

        args = types.SimpleNamespace(
            version=CandidateFixture.VERSION,
            notes="",
            runtime_source=".",
            server=ota.PRODUCTION_HOST,
            port=ota.PRODUCTION_PORT,
            user=ota.PRODUCTION_USER,
            signing_key="must-not-be-read",
            known_hosts=None,
            host_key_sha256=host_key,
        )
        signer = mock.Mock(side_effect=AssertionError("signing secret was touched"))
        ssh = mock.Mock(side_effect=AssertionError("network was touched"))
        with self.assertRaisesRegex(ota.ReleaseGateError, "clean build failure"):
            ota.trusted_release_dual(
                args,
                _candidate_builder=failed_build,
                _signer_factory=signer,
                _ssh_factory=ssh,
            )
        signer.assert_not_called()
        ssh.assert_not_called()

    def test_ota_failure_never_creates_github_and_github_failure_is_ambiguous(self):
        host_key = "SHA256:" + "A" * 43
        args = types.SimpleNamespace(
            version=CandidateFixture.VERSION,
            notes="release notes",
            runtime_source=".",
            server=ota.PRODUCTION_HOST,
            port=ota.PRODUCTION_PORT,
            user=ota.PRODUCTION_USER,
            signing_key="unused-by-injected-publisher",
            known_hosts=None,
            host_key_sha256=host_key,
        )
        local_report = {"candidate": "verifier-owned"}
        trusted_report = {"candidate": "fresh-trusted"}

        @contextlib.contextmanager
        def built(_args):
            yield local_report

        trust = mock.Mock(return_value=trusted_report)
        origin = mock.Mock()
        preflight = mock.Mock(return_value={"tag": f"v{args.version}"})
        github = mock.Mock()
        ota_failed = mock.Mock(side_effect=ota.ReleaseGateError("synthetic OTA failure"))
        with self.assertRaisesRegex(ota.ReleaseGateError, "synthetic OTA failure"):
            ota.trusted_release_dual(
                args,
                _candidate_builder=built,
                _trust_factory=trust,
                _origin_verifier=origin,
                _github_preflight_factory=preflight,
                _ota_publisher=ota_failed,
                _github_publisher=github,
            )
        github.assert_not_called()

        ota_succeeded = mock.Mock(return_value={"status": "OTA readback passed"})
        github_failed = mock.Mock(side_effect=RuntimeError("synthetic GitHub failure"))
        with self.assertRaisesRegex(
            ota.ReleaseGateError,
            "OTA 双通道已成功外发.*部分发布/外部状态有歧义",
        ):
            ota.trusted_release_dual(
                args,
                _candidate_builder=built,
                _trust_factory=trust,
                _origin_verifier=origin,
                _github_preflight_factory=preflight,
                _ota_publisher=ota_succeeded,
                _github_publisher=github_failed,
            )
        ota_succeeded.assert_called_once()
        github_failed.assert_called_once()

    def test_local_cleanup_failure_is_reported_without_masking_external_result(self):
        args = types.SimpleNamespace(
            version=CandidateFixture.VERSION,
            notes="release notes",
            runtime_source=".",
            server=ota.PRODUCTION_HOST,
            port=ota.PRODUCTION_PORT,
            user=ota.PRODUCTION_USER,
            known_hosts=None,
            host_key_sha256="SHA256:" + "A" * 43,
        )
        trusted_report = {"candidate": "fresh-trusted"}
        built_contexts = []

        def make_context():
            tool = ota._TrustedTool("test", Path(sys.executable), "0" * 64, "test")
            context = ota._BuiltReleaseContext(
                report={"candidate": "verifier-owned"},
                verifier_root=REPO_ROOT,
                brand_verifier_root=REPO_ROOT,
                main_head="a" * 40,
                brand_head="b" * 40,
                trusted_file_sha256={},
                git_tool=tool,
                gh_tool=tool,
                python_tool=tool,
                build_tools=(tool,),
                toolchain_closures=(),
                protected_tool_roots=(),
                python_runtime_root=REPO_ROOT,
                python_runtime_sha256="0" * 64,
            )
            built_contexts.append(context)
            return context

        @contextlib.contextmanager
        def built(_args):
            context = make_context()
            try:
                yield context
            finally:
                context.cleanup_errors.append("worktree neutral: synthetic cleanup failure")

        common = {
            "_candidate_builder": built,
            "_trust_factory": mock.Mock(return_value=trusted_report),
            "_origin_verifier": mock.Mock(),
            "_github_preflight_factory": mock.Mock(return_value={"tag": "test"}),
            "_ota_publisher": mock.Mock(return_value={"status": "OTA readback passed"}),
        }
        with mock.patch.object(ota, "_revalidate_built_release_context"):
            result = ota.trusted_release_dual(
                args,
                _github_publisher=mock.Mock(return_value={"status": "GitHub readback passed"}),
                **common,
            )
        self.assertEqual(
            result["localCleanup"]["status"], "failed-after-external-publication"
        )
        self.assertIn("synthetic cleanup failure", result["localCleanup"]["errors"][0])

        with mock.patch.object(ota, "_revalidate_built_release_context"), \
                self.assertRaisesRegex(
                    ota.ReleaseGateError,
                    "OTA 双通道已成功外发.*部分发布/外部状态有歧义",
                ) as raised:
            ota.trusted_release_dual(
                args,
                _github_publisher=mock.Mock(side_effect=RuntimeError("external failure")),
                **common,
            )
        self.assertIsInstance(raised.exception.__cause__, RuntimeError)
        self.assertIn("synthetic cleanup failure", built_contexts[-1].cleanup_errors[0])

    def test_origin_refs_and_github_release_are_bound_to_exact_candidate(self):
        neutral_head = "a" * 40
        brand_head = "b" * 40
        report = {
            "version": CandidateFixture.VERSION,
            "pairGate": {
                "neutral": {"head": neutral_head},
                "brand": {"head": brand_head},
            },
        }
        exact_refs = (
            f"{neutral_head}\trefs/heads/main\n"
            f"{brand_head}\trefs/heads/hk-pathlynx-corpla"
        )
        def exact_git(_root, *arguments):
            if arguments[:3] == ("remote", "get-url", "--all"):
                return "https://github.com/yu1201/NoTeaching-Robot.git"
            if arguments[:4] == ("remote", "get-url", "--push", "--all"):
                return "git@github.com:yu1201/NoTeaching-Robot.git"
            return exact_refs

        with mock.patch.object(ota, "_git_text", side_effect=exact_git):
            ota._verify_pushed_origin_heads(REPO_ROOT, report)
        self.assertEqual(
            ota._normalize_github_remote_url(
                "git@github.com-443:yu1201/NoTeaching-Robot.git"
            ),
            ota.EXPECTED_GITHUB_REPOSITORY,
        )
        def wrong_ref_git(_root, *arguments):
            if arguments and arguments[0] == "remote":
                return "https://github.com/yu1201/NoTeaching-Robot.git"
            return f"{'c' * 40}\trefs/heads/main\n{brand_head}\trefs/heads/hk-pathlynx-corpla"
        with mock.patch.object(ota, "_git_text", side_effect=wrong_ref_git), \
                self.assertRaisesRegex(ota.ReleaseGateError, "origin refs"):
            ota._verify_pushed_origin_heads(REPO_ROOT, report)

        def wrong_repo_git(_root, *arguments):
            if arguments and arguments[0] == "remote":
                return "https://github.com/attacker/wrong-repository.git"
            return exact_refs
        with mock.patch.object(ota, "_git_text", side_effect=wrong_repo_git), \
                self.assertRaisesRegex(ota.ReleaseGateError, "固定为"):
            ota._verify_pushed_origin_heads(REPO_ROOT, report)

        args = types.SimpleNamespace(
            version=CandidateFixture.VERSION,
            notes="release notes",
            gh_exe=sys.executable,
        )
        def exact_gh(_gh, arguments, *, cwd, label, timeout=10 * 60):
            del cwd, label, timeout
            if arguments[:2] == ["repo", "view"]:
                return types.SimpleNamespace(stdout=json.dumps({
                    "nameWithOwner": ota.EXPECTED_GITHUB_REPOSITORY,
                    "viewerPermission": "WRITE",
                }))
            if arguments[0] == "api" and "/git/ref/heads/" in arguments[1]:
                branch = arguments[1].rsplit("/", 1)[-1]
                sha = neutral_head if branch == "main" else brand_head
                return types.SimpleNamespace(stdout=json.dumps({
                    "ref": f"refs/heads/{branch}",
                    "object": {"type": "commit", "sha": sha},
                }))
            return types.SimpleNamespace(stdout="")
        not_found = types.SimpleNamespace(returncode=1, stdout="", stderr="HTTP 404 Not Found")
        existing_tag = types.SimpleNamespace(returncode=0, stdout="{}", stderr="")
        with mock.patch.object(ota, "_run_gh_checked", side_effect=exact_gh), \
                mock.patch.object(
                    ota.subprocess, "run", side_effect=[not_found, existing_tag]
                ), self.assertRaisesRegex(ota.ReleaseGateError, "GitHub tag ref.*已存在"):
            ota._github_release_preflight(REPO_ROOT, args, report)

        wrong_repo = types.SimpleNamespace(stdout=json.dumps({
            "nameWithOwner": "attacker/wrong-repository",
            "viewerPermission": "ADMIN",
        }))
        with mock.patch.object(ota, "_run_gh_checked", return_value=wrong_repo), \
                self.assertRaisesRegex(ota.ReleaseGateError, "精确为"):
            ota._github_release_preflight(REPO_ROOT, args, report)

    def test_detached_trust_file_drift_is_rejected_before_remote_lock(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = CandidateFixture(Path(temporary), real_pe=True)
            trusted = fixture.trust(fixture.prepare())
            signing_module = fixture.root / "scripts" / "ota_manifest_signing.py"
            signing_module.write_text(
                signing_module.read_text(encoding="utf-8") + "\n# concurrent drift\n",
                encoding="utf-8",
            )
            sftp = FakeSftp()
            with self.assertRaisesRegex(ota.ReleaseGateError, "受信任验证代码"):
                ota.publish_dual_with_sftp(sftp, trusted, make_test_signer())
            self.assertEqual(sftp.operations, [])

    def test_github_release_readback_requires_two_assets_and_exact_main_commit(self):
        neutral_head = "a" * 40
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            neutral = root / f"NoTeaching-Robot-Setup-{CandidateFixture.VERSION}.exe"
            brand = root / f"HK-Pathlynx-CORPLA-Setup-{CandidateFixture.VERSION}.exe"
            neutral.write_bytes(b"neutral-installer")
            brand.write_bytes(b"brand-installer")
            report = {
                "pairGate": {"neutral": {"head": neutral_head}},
                "channels": {
                    "neutral": {
                        "installer": {
                            "path": str(neutral),
                            "file": neutral.name,
                            "sha256": ota.sha256_file(neutral),
                            "size": neutral.stat().st_size,
                        },
                    },
                    "brand": {
                        "installer": {
                            "path": str(brand),
                            "file": brand.name,
                            "sha256": ota.sha256_file(brand),
                            "size": brand.stat().st_size,
                        },
                    },
                },
            }
            args = types.SimpleNamespace(version=CandidateFixture.VERSION, notes="notes")
            preflight = {
                "gh": "gh.exe",
                "repo": "owner/repository",
                "tag": f"v{CandidateFixture.VERSION}",
            }
            metadata = {
                "tag_name": preflight["tag"],
                "target_commitish": "main",
                "draft": True,
                "prerelease": False,
                "html_url": "https://example.invalid/release",
                "assets": [
                    {"id": 101, "name": neutral.name, "state": "uploaded", "size": neutral.stat().st_size},
                    {"id": 102, "name": brand.name, "state": "uploaded", "size": brand.stat().st_size},
                ],
            }
            state = {"published": False, "corruptDownload": False}
            call_order = []

            def fake_gh(_gh, arguments, *, cwd, label, timeout=10 * 60):
                del cwd, label, timeout
                if arguments[:2] == ["release", "create"]:
                    call_order.append("draft-create")
                    self.assertIn("--draft", arguments)
                if arguments[0] == "api":
                    if arguments[1].endswith("/git/ref/heads/main"):
                        call_order.append("main-ref-readback")
                        return types.SimpleNamespace(stdout=json.dumps({
                            "ref": "refs/heads/main",
                            "object": {"type": "commit", "sha": neutral_head},
                        }))
                    call_order.append("published-api" if state["published"] else "draft-api")
                    response = dict(metadata)
                    response["draft"] = not state["published"]
                    return types.SimpleNamespace(stdout=json.dumps(response))
                if arguments[:2] == ["release", "download"]:
                    call_order.append("draft-download")
                    destination = Path(arguments[arguments.index("--dir") + 1])
                    neutral_bytes = b"corrupt" if state["corruptDownload"] else neutral.read_bytes()
                    (destination / neutral.name).write_bytes(neutral_bytes)
                    (destination / brand.name).write_bytes(brand.read_bytes())
                if arguments[:2] == ["release", "edit"]:
                    call_order.append("release-edit")
                    state["published"] = True
                return types.SimpleNamespace(stdout="")

            original_fake_gh = fake_gh
            def fake_gh_with_tag(_gh, arguments, *, cwd, label, timeout=10 * 60):
                if arguments[0] == "api" and "/git/ref/tags/" in arguments[1]:
                    call_order.append("tag-readback")
                    return types.SimpleNamespace(stdout=json.dumps({
                        "ref": f"refs/tags/{preflight['tag']}",
                        "object": {"type": "commit", "sha": neutral_head},
                    }))
                return original_fake_gh(
                    _gh, arguments, cwd=cwd, label=label, timeout=timeout
                )

            with mock.patch.object(ota, "_run_gh_checked", side_effect=fake_gh_with_tag):
                result = ota._publish_github_release(
                    REPO_ROOT, args, report, preflight
                )
            self.assertEqual(result["target"], "main")
            self.assertEqual(result["commit"], neutral_head)
            self.assertLess(call_order.index("draft-download"), call_order.index("tag-readback"))
            self.assertLess(call_order.index("main-ref-readback"), call_order.index("release-edit"))
            self.assertLess(call_order.index("release-edit"), call_order.index("published-api"))
            self.assertLess(call_order.index("published-api"), call_order.index("tag-readback"))
            self.assertEqual(call_order.count("tag-readback"), 1)

            state["published"] = False
            call_order.clear()
            metadata["assets"].append(dict(metadata["assets"][1]))
            with mock.patch.object(ota, "_run_gh_checked", side_effect=fake_gh), \
                    self.assertRaisesRegex(ota.ReleaseGateError, "恰好两份"):
                ota._publish_github_release(REPO_ROOT, args, report, preflight)
            self.assertNotIn("release-edit", call_order)

            metadata["assets"].pop()
            state["published"] = False
            state["corruptDownload"] = True
            call_order.clear()
            with mock.patch.object(ota, "_run_gh_checked", side_effect=fake_gh), \
                    self.assertRaisesRegex(ota.ReleaseGateError, "资产回读 size/hash"):
                ota._publish_github_release(REPO_ROOT, args, report, preflight)
            self.assertNotIn("tag-readback", call_order)
            self.assertNotIn("release-edit", call_order)

    def test_brand_source_boundary_rejects_src_or_installer_run_drift(self):
        def git_blob(ref: str, relative: str) -> bytes:
            return subprocess.check_output(
                ["git", "show", f"{ref}:{relative}"], cwd=REPO_ROOT
            )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            neutral = root / "neutral"
            brand = root / "brand"
            neutral.mkdir()
            brand.mkdir()
            for relative in (".gitignore", "QtWidgetsApplication4.vcxproj",
                             "installer/QtWidgetsApplication4.iss"):
                for target, ref in ((neutral, "refs/heads/main"),
                                    (brand, "refs/heads/hk-pathlynx-corpla")):
                    path = target / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(git_blob(ref, relative))
            for relative in ota._ALLOWED_BRAND_TRACKED_DELTA:
                if relative in {".gitignore", "QtWidgetsApplication4.vcxproj",
                                "installer/QtWidgetsApplication4.iss"}:
                    continue
                path = brand / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(git_blob("refs/heads/hk-pathlynx-corpla", relative))

            main_head = subprocess.check_output(
                ["git", "rev-parse", "refs/heads/main"], cwd=REPO_ROOT, text=True
            ).strip()
            brand_head = subprocess.check_output(
                ["git", "rev-parse", "refs/heads/hk-pathlynx-corpla"],
                cwd=REPO_ROOT, text=True,
            ).strip()
            allowed_delta = "\n".join(sorted(ota._ALLOWED_BRAND_TRACKED_DELTA))
            original_git_text = ota._git_text

            def clean_diff(repo_root, *arguments):
                if arguments and arguments[0] == "diff":
                    return allowed_delta
                return original_git_text(repo_root, *arguments)

            with mock.patch.object(ota, "_git_text", side_effect=clean_diff):
                ota._assert_brand_source_boundary(
                    REPO_ROOT, neutral, brand, main_head, brand_head
                )

            def malicious_diff(repo_root, *arguments):
                if arguments and arguments[0] == "diff":
                    return allowed_delta + "\nsrc/Backdoor.cpp"
                return original_git_text(repo_root, *arguments)
            with mock.patch.object(ota, "_git_text", side_effect=malicious_diff), \
                    self.assertRaisesRegex(ota.ReleaseGateError, "allowlist"):
                ota._assert_brand_source_boundary(
                    REPO_ROOT, neutral, brand, main_head, brand_head
                )

            iss_path = brand / "installer" / "QtWidgetsApplication4.iss"
            iss_path.write_text(
                iss_path.read_text(encoding="utf-8-sig")
                + '\n[Run]\nFilename: "{app}\\evil.exe"\n',
                encoding="utf-8",
            )
            with mock.patch.object(ota, "_git_text", side_effect=clean_diff), \
                    self.assertRaisesRegex(ota.ReleaseGateError, "功能行"):
                ota._assert_brand_source_boundary(
                    REPO_ROOT, neutral, brand, main_head, brand_head
                )

    def test_wrong_manifest_signing_key_is_rejected_before_a_signer_is_returned(self):
        wrong_key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
        fake_signing = types.SimpleNamespace(
            load_private_key=lambda _path: wrong_key,
            cng_public_blob=signing.cng_public_blob,
        )
        with mock.patch.object(ota, "_resolve_signing_key", return_value=Path("outside-key.dpapi")), \
                mock.patch.object(ota, "_load_signing_module", return_value=fake_signing), \
                self.assertRaises(ota.ReleaseGateError):
            ota.make_manifest_signer("ignored")

    def test_publisher_public_key_fingerprint_matches_client_blob(self):
        client = (REPO_ROOT / "src" / "OnlineServicesDialog.cpp").read_text(encoding="utf-8")
        match = re.search(
            r'kOtaReleasePublicKeyBlobBase64\[\]\s*=\s*\n?\s*"([A-Za-z0-9+/=]+)"', client
        )
        self.assertIsNotNone(match)
        client_blob = base64.b64decode(match.group(1), validate=True)
        self.assertEqual(ota.sha256_bytes(client_blob), ota.CLIENT_OTA_PUBLIC_KEY_FINGERPRINT)

    def test_manifest_schema_rejects_unknown_fields_and_payloads_over_client_limit(self):
        signer = make_test_signer()
        now = dt.datetime(2026, 7, 12, 12, 0, 0, tzinfo=dt.timezone.utc)
        published, expires, _ = ota._new_manifest_freshness(now)
        manifest = {
            "schemaVersion": ota.MANIFEST_SCHEMA_VERSION,
            "channel": "brand",
            "version": CandidateFixture.VERSION,
            "publishedAtUtc": published,
            "expiresAtUtc": expires,
            "file": ota._expected_installer_name("brand", CandidateFixture.VERSION),
            "sha256": "a" * 64,
            "size": 10,
            "notes": "test",
            "signatureAlgorithm": ota.MANIFEST_SIGNATURE_ALGORITHM,
            "patch": {
                "file": ota._expected_patch_name("brand", CandidateFixture.VERSION),
                "sha256": "b" * 64,
                "size": 5,
                "baseMinVersion": CandidateFixture.VERSION,
            },
        }
        manifest["signature"] = signer(manifest)
        ota._validate_manifest(manifest, "brand", require_fresh=True, now=now)
        with self.assertRaises(ota.ReleaseGateError):
            ota._validate_manifest({**manifest, "unknown": "rejected"}, "brand")
        bad_patch = json.loads(json.dumps(manifest))
        bad_patch["patch"]["unknown"] = 1
        with self.assertRaises(ota.ReleaseGateError):
            ota._validate_manifest(bad_patch, "brand")
        oversized = dict(manifest)
        oversized["size"] = ota.MAX_UPDATE_PAYLOAD_BYTES + 1
        oversized["signature"] = signer(oversized)
        with self.assertRaises(ota.ReleaseGateError):
            ota._validate_manifest(oversized, "brand")

    def test_manifest_utc_freshness_boundaries_and_signature_binding(self):
        signer = make_test_signer()
        now = dt.datetime(2026, 7, 12, 12, 0, 0, tzinfo=dt.timezone.utc)

        def manifest_for(published: dt.datetime, expires: dt.datetime):
            value = {
                "schemaVersion": ota.MANIFEST_SCHEMA_VERSION,
                "channel": "neutral",
                "version": CandidateFixture.VERSION,
                "publishedAtUtc": ota._format_manifest_utc(published),
                "expiresAtUtc": ota._format_manifest_utc(expires),
                "file": ota._expected_installer_name("neutral", CandidateFixture.VERSION),
                "sha256": "a" * 64,
                "size": 10,
                "notes": "freshness",
                "signatureAlgorithm": ota.MANIFEST_SIGNATURE_ALGORITHM,
            }
            value["signature"] = signer(value)
            return value

        lifetime = dt.timedelta(seconds=ota.MANIFEST_VALIDITY_SECONDS)
        skew = dt.timedelta(seconds=ota.MANIFEST_CLOCK_SKEW_SECONDS)
        accepted = manifest_for(now + skew, now + skew + lifetime)
        ota._validate_manifest(accepted, "neutral", require_fresh=True, now=now)

        too_future = manifest_for(now + skew + dt.timedelta(seconds=1),
                                  now + skew + dt.timedelta(seconds=1) + lifetime)
        with self.assertRaisesRegex(ota.ReleaseGateError, "过度未来"):
            ota._validate_manifest(too_future, "neutral", require_fresh=True, now=now)

        just_within_expiry_skew = manifest_for(
            now - lifetime - skew,
            now - skew,
        )
        ota._validate_manifest(
            just_within_expiry_skew, "neutral", require_fresh=True, now=now
        )
        expired = manifest_for(
            now - lifetime - skew - dt.timedelta(seconds=1),
            now - skew - dt.timedelta(seconds=1),
        )
        with self.assertRaisesRegex(ota.ReleaseGateError, "已过期"):
            ota._validate_manifest(expired, "neutral", require_fresh=True, now=now)

        malformed = dict(accepted)
        malformed["publishedAtUtc"] = "2026-07-12T12:00:00+00:00"
        with self.assertRaisesRegex(ota.ReleaseGateError, "严格 UTC"):
            ota._validate_manifest(malformed, "neutral")
        wrong_lifetime = dict(accepted)
        wrong_lifetime["expiresAtUtc"] = ota._format_manifest_utc(now + lifetime + dt.timedelta(seconds=1))
        with self.assertRaisesRegex(ota.ReleaseGateError, "7 天"):
            ota._validate_manifest(wrong_lifetime, "neutral")

        tampered_time = dict(accepted)
        tampered_time["expiresAtUtc"] = ota._format_manifest_utc(
            now + skew + lifetime + dt.timedelta(seconds=1)
        )
        self.assertFalse(signer.verify(tampered_time))

    def test_notes_limit_counts_unicode_scalars_like_qstring_to_ucs4(self):
        notes = "😀" * 3000
        self.assertEqual(len(ota._validate_notes(notes)), 3000)
        self.assertEqual(len(notes.encode("utf-16-le")) // 2, 6000)
        client = (REPO_ROOT / "src" / "OnlineServicesDialog.cpp").read_text(encoding="utf-8")
        self.assertIn("value.toUcs4()", client)
        self.assertIn("IsValidOtaNotes(notes)", client)
        self.assertLess(client.index("IsValidOtaNotes(notes)"),
                        client.index("SetHighestSeenUpdateVersion(channel, remoteVersion)"))


class AtomicPublishTests(unittest.TestCase):
    def test_remote_reads_stop_when_file_grows_after_stat(self):
        class GrowingRemote(FakeSftp):
            def __init__(self):
                super().__init__()
                self.files["/growing.json"] = b"123456789"

            def stat(self, _path):
                return _FakeStat(8)

        remote = GrowingRemote()
        with self.assertRaisesRegex(ota.ReleaseGateError, "增长|大小变化"):
            ota._read_remote_bytes(remote, "/growing.json", max_bytes=8)
        with self.assertRaisesRegex(ota.ReleaseGateError, "增长|不一致"):
            ota._remote_sha_size(
                remote, "/growing.json", expected_size=8, max_bytes=8
            )

    def test_sftp_wrapper_enforces_io_timeout_and_total_deadline(self):
        now = [100.0]

        class Channel:
            def __init__(self):
                self.timeouts = []

            def settimeout(self, value):
                self.timeouts.append(value)

        class Inner:
            def __init__(self):
                self.channel = Channel()
                self.stat_calls = 0
                self.stream_closed = False

            def get_channel(self):
                return self.channel

            def stat(self, _path):
                self.stat_calls += 1
                return _FakeStat(1)

            def open(self, _path, _mode):
                owner = self

                class Stream:
                    def __enter__(self):
                        return self

                    def __exit__(self, *_args):
                        owner.stream_closed = True

                    def read(self, _size):
                        now[0] = 106.0
                        return b"x"

                return Stream()

            def close(self):
                return None

        inner = Inner()
        guarded = ota._DeadlineSftpClient(
            inner, total_timeout=5, io_timeout=2, _clock=lambda: now[0]
        )
        self.assertEqual(inner.channel.timeouts[-1], 2)
        guarded.stat("/ok")
        self.assertEqual(inner.stat_calls, 1)
        now[0] = 106.0
        with self.assertRaisesRegex(ota.ReleaseGateError, "deadline"):
            guarded.stat("/late")
        self.assertEqual(inner.stat_calls, 1)
        now[0] = 100.0
        guarded = ota._DeadlineSftpClient(
            inner, total_timeout=5, io_timeout=2, _clock=lambda: now[0]
        )
        with self.assertRaisesRegex(ota.ReleaseGateError, "deadline"):
            with guarded.open("/slow", "rb") as stream:
                stream.read(1)
        self.assertTrue(inner.stream_closed)

        now[0] = 0.0

        class ProgressiveUpload:
            def __init__(self):
                self.channel = Channel()
                self.files = {}
                self.closed = False

            def get_channel(self):
                return self.channel

            def stat(self, path):
                if path not in self.files:
                    raise FileNotFoundError(errno.ENOENT, "missing", path)
                return _FakeStat(len(self.files[path]))

            def remove(self, path):
                if path not in self.files:
                    raise FileNotFoundError(errno.ENOENT, "missing", path)
                del self.files[path]

            def put(self, local, remote, callback=None, confirm=True):
                del confirm
                payload = Path(local).read_bytes()
                self.files[remote] = b"partial"
                for transferred in (1, 2, len(payload)):
                    now[0] += 2.0
                    callback(transferred, len(payload))

            def close(self):
                self.closed = True

        upload = ProgressiveUpload()
        guarded_upload = ota._DeadlineSftpClient(
            upload, total_timeout=5, io_timeout=2, _clock=lambda: now[0]
        )
        with tempfile.TemporaryDirectory() as temporary:
            local = Path(temporary) / "payload.exe"
            local.write_bytes(b"payload")
            with self.assertRaisesRegex(ota.ReleaseGateError, "deadline"):
                ota._stage_local_file(
                    guarded_upload,
                    str(local),
                    "/payload.exe",
                    "txn",
                    ota.sha256_file(local),
                    local.stat().st_size,
                )
        self.assertEqual(upload.files, {}, "deadline left a remote temp payload")
        guarded_upload.close()
        self.assertTrue(upload.closed)

    def _candidate(self):
        temp = tempfile.TemporaryDirectory()
        fixture = CandidateFixture(Path(temp.name), real_pe=True)
        report = fixture.prepare()
        return temp, fixture, fixture.trust(report)

    @staticmethod
    def _seed_signed_v2_latest(sftp, fixture, report, signer):
        for channel in ota.CHANNELS:
            candidate = report["channels"][channel]
            installer = candidate["installer"]
            manifest = {
                "schemaVersion": ota.LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION,
                "channel": channel,
                "version": report["version"],
                "file": installer["file"],
                "sha256": installer["sha256"],
                "size": installer["size"],
                "notes": "preserve signed v2 notes during schema migration",
                "signatureAlgorithm": ota.MANIFEST_SIGNATURE_ALGORITHM,
            }
            patch = candidate.get("patch")
            if patch is not None:
                manifest["patch"] = {
                    "file": patch["file"],
                    "sha256": patch["sha256"],
                    "size": patch["size"],
                    "baseMinVersion": report["version"],
                }
            manifest["signature"] = signer.sign_legacy_v2(manifest)
            remote_dir = f"/var/www/ota/{channel}"
            sftp.files[f"{remote_dir}/latest.json"] = ota._json_bytes(manifest)
            sftp.files[f"{remote_dir}/{installer['file']}"] = Path(installer["path"]).read_bytes()
            if patch is not None:
                sftp.files[f"{remote_dir}/{patch['file']}"] = Path(patch["path"]).read_bytes()

    def test_both_channels_stage_and_verify_before_latest_is_switched(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        result = ota.publish_dual_with_sftp(sftp, report, signer)
        self.assertEqual(result["version"], fixture.VERSION)

        for operation, remote, _extra in sftp.operations:
            if operation in {"put", "putfo"}:
                self.assertIn(".tmp.", remote, f"direct final-name upload detected: {remote}")
        rename_destinations = [extra for op, _src, extra in sftp.operations if op == "rename"]
        self.assertEqual(
            rename_destinations[-4:],
            [
                "/var/www/ota/neutral/latest.json",
                "/var/www/ota/neutral/latest-v3.json",
                "/var/www/ota/brand/latest.json",
                "/var/www/ota/brand/latest-v3.json",
            ],
        )
        first_latest = min(
            i for i, (op, _src, destination) in enumerate(sftp.operations)
            if op == "rename" and destination
            and destination.endswith(("/latest.json", "/latest-v3.json"))
        )
        earlier_destinations = [
            destination for op, _src, destination in sftp.operations[:first_latest] if op == "rename"
        ]
        self.assertIn("/var/www/ota/neutral/payload_history.json", earlier_destinations)
        self.assertIn("/var/www/ota/brand/payload_history.json", earlier_destinations)
        self.assertIn(
            f"/var/www/ota/neutral/{fixture.neutral_installer.name}", earlier_destinations
        )
        self.assertIn(f"/var/www/ota/brand/{fixture.brand_patch.name}", earlier_destinations)

        for channel in ota.CHANNELS:
            legacy_path = f"/var/www/ota/{channel}/latest.json"
            legacy = json.loads(sftp.files[legacy_path].decode("utf-8"))
            self.assertEqual(legacy["schemaVersion"], ota.LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION)
            self.assertTrue(signing.verify_manifest(legacy, legacy["signature"], signer.public_key))
            latest_path = f"/var/www/ota/{channel}/latest-v3.json"
            latest = json.loads(sftp.files[latest_path].decode("utf-8"))
            self.assertEqual(latest["version"], fixture.VERSION)
            self.assertEqual(latest["schemaVersion"], ota.MANIFEST_SCHEMA_VERSION)
            self.assertEqual(latest["channel"], channel)
            self.assertEqual(latest["signatureAlgorithm"], "RSA-PKCS1-SHA256")
            published = ota._parse_manifest_utc(latest["publishedAtUtc"], "publishedAtUtc")
            expires = ota._parse_manifest_utc(latest["expiresAtUtc"], "expiresAtUtc")
            self.assertEqual(
                int((expires - published).total_seconds()), ota.MANIFEST_VALIDITY_SECONDS
            )
            self.assertTrue(signing.verify_manifest(latest, latest["signature"], signer.public_key))
            tampered = dict(latest)
            tampered["notes"] += " tampered"
            self.assertFalse(signing.verify_manifest(tampered, latest["signature"], signer.public_key))
            history = json.loads(
                sftp.files[f"/var/www/ota/{channel}/payload_history.json"].decode("utf-8")
            )
            self.assertIn(fixture.VERSION, history)
        self.assertNotIn("/var/www/ota/.dual-publish.lock", sftp.directories)
        operations_after_publish = list(sftp.operations)
        with self.assertRaisesRegex(ota.ReleaseGateError, "见证已使用"):
            ota.publish_dual_with_sftp(sftp, report, signer)
        self.assertEqual(sftp.operations, operations_after_publish)

    def test_lock_cleanup_failure_preserves_committed_result(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        lock_path = "/var/www/ota/.dual-publish.lock"
        sftp.fail_rmdir_once.add(lock_path)
        result = ota.publish_dual_with_sftp(sftp, report, make_test_signer())
        self.assertTrue(result["committed"])
        self.assertEqual(result["lockCleanup"]["status"], "failed-after-commit")
        for channel in ota.CHANNELS:
            self.assertIn(f"/var/www/ota/{channel}/latest-v3.json", sftp.files)

    def test_second_latest_failure_rolls_back_first_latest_and_both_histories(self):
        temp, _fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        sftp.fail_rename_once.add("/var/www/ota/brand/latest-v3.json")
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, report, make_test_signer())
        for channel in ota.CHANNELS:
            self.assertNotIn(f"/var/www/ota/{channel}/latest.json", sftp.files)
            self.assertNotIn(f"/var/www/ota/{channel}/latest-v3.json", sftp.files)
            self.assertNotIn(f"/var/www/ota/{channel}/payload_history.json", sftp.files)
        self.assertNotIn("/var/www/ota/.dual-publish.lock", sftp.directories)

    def test_signed_v2_remote_is_verified_and_upgraded_in_place_to_fresh_v3(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        self._seed_signed_v2_latest(sftp, fixture, report, signer)

        result = ota.publish_dual_with_sftp(sftp, report, signer)
        self.assertTrue(all(not item["alreadyVisible"] for item in result["channels"].values()))
        for channel in ota.CHANNELS:
            legacy = json.loads(
                sftp.files[f"/var/www/ota/{channel}/latest.json"].decode("utf-8")
            )
            self.assertEqual(
                legacy["schemaVersion"], ota.LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION
            )
            self.assertEqual(legacy["notes"], "preserve signed v2 notes during schema migration")
            latest = json.loads(
                sftp.files[f"/var/www/ota/{channel}/latest-v3.json"].decode("utf-8")
            )
            self.assertEqual(latest["schemaVersion"], ota.MANIFEST_SCHEMA_VERSION)
            self.assertEqual(
                latest["notes"], "preserve signed v2 notes during schema migration"
            )
            ota._validate_manifest(latest, channel, require_fresh=True)
            self.assertTrue(signer.verify(latest))

    def test_signed_same_version_payload_swap_is_still_rejected(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        candidate = report["channels"]["neutral"]["installer"]
        swapped = {
            "schemaVersion": ota.LEGACY_SIGNED_MANIFEST_SCHEMA_VERSION,
            "channel": "neutral",
            "version": report["version"],
            "file": candidate["file"],
            "sha256": "c" * 64,
            "size": candidate["size"],
            "notes": "signed but different payload",
            "signatureAlgorithm": ota.MANIFEST_SIGNATURE_ALGORITHM,
        }
        swapped["signature"] = signer.sign_legacy_v2(swapped)
        sftp.files["/var/www/ota/neutral/latest.json"] = ota._json_bytes(swapped)
        with self.assertRaisesRegex(ota.ReleaseGateError, "不同安装包/补丁"):
            ota.publish_dual_with_sftp(sftp, report, signer)
        self.assertFalse(any(op in {"put", "putfo"} for op, _a, _b in sftp.operations))

    def test_expired_v3_same_payload_is_renewed_without_changing_release_metadata(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        plain_report = dict(report)
        ota.publish_dual_with_sftp(sftp, report, signer)

        expired_published = dt.datetime.now(dt.timezone.utc).replace(microsecond=0) \
            - dt.timedelta(seconds=ota.MANIFEST_VALIDITY_SECONDS + ota.MANIFEST_CLOCK_SKEW_SECONDS + 1)
        expired_expires = expired_published + dt.timedelta(
            seconds=ota.MANIFEST_VALIDITY_SECONDS
        )
        old_times = {}
        for channel in ota.CHANNELS:
            legacy_path = f"/var/www/ota/{channel}/latest.json"
            legacy = json.loads(sftp.files[legacy_path].decode("utf-8"))
            legacy["notes"] = "signed metadata must survive renewal"
            legacy["signature"] = signer.sign_legacy_v2(legacy)
            sftp.files[legacy_path] = ota._json_bytes(legacy)
            latest_path = f"/var/www/ota/{channel}/latest-v3.json"
            latest = json.loads(sftp.files[latest_path].decode("utf-8"))
            latest["notes"] = "signed metadata must survive renewal"
            latest["publishedAtUtc"] = ota._format_manifest_utc(expired_published)
            latest["expiresAtUtc"] = ota._format_manifest_utc(expired_expires)
            latest["signature"] = signer(latest)
            old_times[channel] = latest["publishedAtUtc"]
            sftp.files[latest_path] = ota._json_bytes(latest)

        renewed = fixture.trust(plain_report)
        result = ota.publish_dual_with_sftp(sftp, renewed, signer)
        self.assertTrue(all(not item["alreadyVisible"] for item in result["channels"].values()))
        for channel in ota.CHANNELS:
            latest = json.loads(
                sftp.files[f"/var/www/ota/{channel}/latest-v3.json"].decode("utf-8")
            )
            self.assertNotEqual(latest["publishedAtUtc"], old_times[channel])
            self.assertEqual(latest["notes"], "signed metadata must survive renewal")
            ota._validate_manifest(latest, channel, require_fresh=True)
            self.assertTrue(signer.verify(latest))

    def test_signed_idempotent_retry_passes_but_tampered_remote_manifest_is_rejected(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        plain_report = dict(report)
        ota.publish_dual_with_sftp(sftp, report, signer)
        retry = ota.publish_dual_with_sftp(sftp, fixture.trust(plain_report), signer)
        self.assertTrue(all(item["alreadyVisible"] for item in retry["channels"].values()))

        neutral_latest_path = "/var/www/ota/neutral/latest-v3.json"
        tampered = json.loads(sftp.files[neutral_latest_path].decode("utf-8"))
        tampered["notes"] += " tampered"
        sftp.files[neutral_latest_path] = ota._json_bytes(tampered)
        puts_before = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, fixture.trust(plain_report), signer)
        puts_after = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        self.assertEqual(puts_before, puts_after)

    def test_same_version_recovery_rejects_client_incompatible_unknown_fields(self):
        for target in ("top", "patch"):
            with self.subTest(target=target):
                temp, fixture, report = self._candidate()
                self.addCleanup(temp.cleanup)
                sftp = FakeSftp()
                signer = make_test_signer()
                plain_report = dict(report)
                ota.publish_dual_with_sftp(sftp, report, signer)
                brand_path = "/var/www/ota/brand/latest-v3.json"
                manifest = json.loads(sftp.files[brand_path].decode("utf-8"))
                if target == "top":
                    manifest["futureField"] = "client rejects this"
                else:
                    manifest["patch"]["futureField"] = "client rejects this"
                # 未知字段不在签名 payload 中，即使原签名仍有效，publisher 也必须按客户端
                # 的精确 schema 拒绝，不能把这些旧字节当作幂等成功。
                self.assertTrue(signing.verify_manifest(
                    manifest, manifest["signature"], signer.public_key
                ))
                sftp.files[brand_path] = ota._json_bytes(manifest)
                puts_before = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
                with self.assertRaises(ota.ReleaseGateError):
                    ota.publish_dual_with_sftp(sftp, fixture.trust(plain_report), signer)
                puts_after = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
                self.assertEqual(puts_before, puts_after)

    def test_same_version_recovery_rejects_manifest_padding_over_client_limit(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        plain_report = dict(report)
        ota.publish_dual_with_sftp(sftp, report, signer)
        latest_path = "/var/www/ota/neutral/latest-v3.json"
        original = sftp.files[latest_path]
        # JSON 尾部空白既不改变字段也不改变 canonical 签名，但客户端会在解析前按原始
        # 响应 256 KiB 上限拒绝；publisher 必须使用同一上限，不能把它报成幂等成功。
        sftp.files[latest_path] = original + b" " * (
            ota.MAX_MANIFEST_JSON_BYTES + 1 - len(original)
        )
        puts_before = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, fixture.trust(plain_report), signer)
        puts_after = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        self.assertEqual(puts_before, puts_after)

    def test_local_candidate_swap_after_preflight_is_rejected_before_upload(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        fixture.neutral_installer.write_bytes(b"swapped-after-report")
        sftp = FakeSftp()
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, report, make_test_signer())
        self.assertFalse(any(op in {"put", "putfo"} for op, _a, _b in sftp.operations))

    def test_remote_downgrade_and_same_version_different_payload_are_rejected_before_put(self):
        temp, fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        plain_report = dict(report)
        for remote_version, same_version in (("2026.07.12.1300", False), (fixture.VERSION, True)):
            with self.subTest(remote_version=remote_version):
                sftp = FakeSftp()
                manifest = {
                    "version": remote_version,
                    "file": ota._expected_installer_name("neutral", remote_version),
                    "sha256": "a" * 64,
                    "size": 1,
                    "notes": "remote",
                }
                sftp.files["/var/www/ota/neutral/latest.json"] = ota._json_bytes(manifest)
                with self.assertRaises(ota.ReleaseGateError):
                    ota.publish_dual_with_sftp(
                        sftp, fixture.trust(plain_report), make_test_signer()
                    )
                self.assertFalse(any(op in {"put", "putfo"} for op, _a, _b in sftp.operations))
                self.assertNotIn("/var/www/ota/.dual-publish.lock", sftp.directories)
                self.assertEqual(same_version, remote_version == fixture.VERSION)


if __name__ == "__main__":
    unittest.main(verbosity=2)
