#!/usr/bin/env python3
"""upload_ota.py 离线门禁与双通道原子发布测试；不建立任何网络连接。"""

from __future__ import annotations

import errno
import base64
import contextlib
import importlib.util
import io
import json
import os
import re
import sys
import tempfile
import types
import unittest
from unittest import mock
import zipfile
from pathlib import Path

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

    signer.verify = verify
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

    def mkdir(self, path: str):
        self.operations.append(("mkdir", path, None))
        if path in self.directories:
            raise OSError(errno.EEXIST, "exists", path)
        self.directories.add(path)

    def rmdir(self, path: str):
        self.operations.append(("rmdir", path, None))
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

    def __init__(self, root: Path):
        self.root = root
        self.neutral_dist = root / "neutral-dist"
        self.brand_dist = root / "brand-dist"
        self.neutral_dist.mkdir()
        self.brand_dist.mkdir()
        self.neutral_exe = self.neutral_dist / ota.CHANNEL_EXE["neutral"]
        self.brand_exe = self.brand_dist / ota.CHANNEL_EXE["brand"]
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
        (self.brand_dist / "branding" / "branding.ini").write_bytes(b"brand=1")

        self.neutral_installer = root / ota._expected_installer_name("neutral", self.VERSION)
        self.brand_installer = root / ota._expected_installer_name("brand", self.VERSION)
        self.brand_patch = root / ota._expected_patch_name("brand", self.VERSION)
        self.neutral_installer.write_bytes(b"neutral-installer")
        self.brand_installer.write_bytes(b"brand-installer")
        with zipfile.ZipFile(self.brand_patch, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.write(self.brand_exe, arcname=self.brand_exe.name)

    @staticmethod
    def _write_json(path: Path, value):
        path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True), encoding="utf-8")

    def make_pair_gate(self) -> Path:
        gate_common_sha = ota.sha256_file(REPO_ROOT / "scripts" / "release_gate_common.ps1")
        scripts_dir = self.root / "scripts"
        scripts_dir.mkdir(exist_ok=True)
        for script_name in ("build_installer.ps1", "build_release_package.ps1"):
            (scripts_dir / script_name).write_bytes((REPO_ROOT / "scripts" / script_name).read_bytes())
        installer_producer_sha = ota.sha256_file(scripts_dir / "build_installer.ps1")
        package_producer_sha = ota.sha256_file(scripts_dir / "build_release_package.ps1")
        fanuc_sha = "f" * 64
        migrator_sha = "c" * 64
        source_sha = "d" * 64
        migrate_run_sha = "e" * 64
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
            "producerScriptSha256": ota.sha256_file(REPO_ROOT / "scripts" / "verify_release_pair.ps1"),
            "appId": ota.PUBLISHED_APP_ID,
            "version": self.VERSION,
            "maxInstallerSizeDifferenceBytes": ota.MAX_INSTALLER_SIZE_DELTA,
            "installerSizeDifferenceBytes": abs(
                self.neutral_installer.stat().st_size - self.brand_installer.stat().st_size
            ),
            "commonInventoryFileCount": 1,
            "fanucManifestSha256": fanuc_sha,
            "configMigrateSha256": migrator_sha,
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
        manifest = {
            "schemaVersion": 2,
            "channel": "brand",
            "version": CandidateFixture.VERSION,
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
        ota._validate_manifest(manifest, "brand")
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


class AtomicPublishTests(unittest.TestCase):
    def _candidate(self):
        temp = tempfile.TemporaryDirectory()
        fixture = CandidateFixture(Path(temp.name))
        return temp, fixture, fixture.prepare()

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
            rename_destinations[-2:],
            ["/var/www/ota/neutral/latest.json", "/var/www/ota/brand/latest.json"],
        )
        first_latest = min(
            i for i, (op, _src, destination) in enumerate(sftp.operations)
            if op == "rename" and destination and destination.endswith("/latest.json")
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
            latest_path = f"/var/www/ota/{channel}/latest.json"
            latest = json.loads(sftp.files[latest_path].decode("utf-8"))
            self.assertEqual(latest["version"], fixture.VERSION)
            self.assertEqual(latest["schemaVersion"], 2)
            self.assertEqual(latest["channel"], channel)
            self.assertEqual(latest["signatureAlgorithm"], "RSA-PKCS1-SHA256")
            self.assertTrue(signing.verify_manifest(latest, latest["signature"], signer.public_key))
            tampered = dict(latest)
            tampered["notes"] += " tampered"
            self.assertFalse(signing.verify_manifest(tampered, latest["signature"], signer.public_key))
            history = json.loads(
                sftp.files[f"/var/www/ota/{channel}/payload_history.json"].decode("utf-8")
            )
            self.assertIn(fixture.VERSION, history)
        self.assertNotIn("/var/www/ota/.dual-publish.lock", sftp.directories)

    def test_second_latest_failure_rolls_back_first_latest_and_both_histories(self):
        temp, _fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        sftp.fail_rename_once.add("/var/www/ota/brand/latest.json")
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, report, make_test_signer())
        for channel in ota.CHANNELS:
            self.assertNotIn(f"/var/www/ota/{channel}/latest.json", sftp.files)
            self.assertNotIn(f"/var/www/ota/{channel}/payload_history.json", sftp.files)
        self.assertNotIn("/var/www/ota/.dual-publish.lock", sftp.directories)

    def test_signed_idempotent_retry_passes_but_tampered_remote_manifest_is_rejected(self):
        temp, _fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        ota.publish_dual_with_sftp(sftp, report, signer)
        retry = ota.publish_dual_with_sftp(sftp, report, signer)
        self.assertTrue(all(item["alreadyVisible"] for item in retry["channels"].values()))

        neutral_latest_path = "/var/www/ota/neutral/latest.json"
        tampered = json.loads(sftp.files[neutral_latest_path].decode("utf-8"))
        tampered["notes"] += " tampered"
        sftp.files[neutral_latest_path] = ota._json_bytes(tampered)
        puts_before = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, report, signer)
        puts_after = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        self.assertEqual(puts_before, puts_after)

    def test_same_version_recovery_rejects_client_incompatible_unknown_fields(self):
        for target in ("top", "patch"):
            with self.subTest(target=target):
                temp, _fixture, report = self._candidate()
                self.addCleanup(temp.cleanup)
                sftp = FakeSftp()
                signer = make_test_signer()
                ota.publish_dual_with_sftp(sftp, report, signer)
                brand_path = "/var/www/ota/brand/latest.json"
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
                    ota.publish_dual_with_sftp(sftp, report, signer)
                puts_after = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
                self.assertEqual(puts_before, puts_after)

    def test_same_version_recovery_rejects_manifest_padding_over_client_limit(self):
        temp, _fixture, report = self._candidate()
        self.addCleanup(temp.cleanup)
        sftp = FakeSftp()
        signer = make_test_signer()
        ota.publish_dual_with_sftp(sftp, report, signer)
        latest_path = "/var/www/ota/neutral/latest.json"
        original = sftp.files[latest_path]
        # JSON 尾部空白既不改变字段也不改变 canonical 签名，但客户端会在解析前按原始
        # 响应 256 KiB 上限拒绝；publisher 必须使用同一上限，不能把它报成幂等成功。
        sftp.files[latest_path] = original + b" " * (
            ota.MAX_MANIFEST_JSON_BYTES + 1 - len(original)
        )
        puts_before = sum(op in {"put", "putfo"} for op, _a, _b in sftp.operations)
        with self.assertRaises(ota.ReleaseGateError):
            ota.publish_dual_with_sftp(sftp, report, signer)
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
                    ota.publish_dual_with_sftp(sftp, report, make_test_signer())
                self.assertFalse(any(op in {"put", "putfo"} for op, _a, _b in sftp.operations))
                self.assertNotIn("/var/www/ota/.dual-publish.lock", sftp.directories)
                self.assertEqual(same_version, remote_version == fixture.VERSION)


if __name__ == "__main__":
    unittest.main(verbosity=2)
