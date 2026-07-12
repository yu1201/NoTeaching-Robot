#!/usr/bin/env python3
"""Behavioral regression for schema-3 point-cloud proof integrity.

The C++ verifier is wired by verify_pointcloud_quality_gate.py.  This test
exercises the security protocol itself: HMAC over canonical JSON, a protected
per-scan receipt, context freshness, and execution-time input/pose rereads.
"""

from __future__ import annotations

import copy
import hashlib
import hmac
import json
import os
from datetime import datetime, timedelta, timezone
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import UUID, uuid4


MAX_AGE = timedelta(hours=24)
MAX_FUTURE_SKEW = timedelta(minutes=5)
MAC_HEADER = b"NoTeaching-Robot PointCloud Quality Proof v3\n"


def utc_text(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def parse_strict_utc(value: object) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise ValueError("timestamp is not explicit UTC")
    return datetime.fromisoformat(value[:-1] + "+00:00")


def canonical(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()


def sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def evidence(path: Path) -> dict[str, object]:
    payload = path.read_bytes()
    return {"relativePath": path.name, "size": len(payload), "sha256": sha256(payload)}


def mac_payload(proof: dict[str, object]) -> bytes:
    unsigned = copy.deepcopy(proof)
    integrity = dict(unsigned.get("integrity", {}))
    integrity.pop("mac", None)
    unsigned["integrity"] = integrity
    return MAC_HEADER + canonical(unsigned)


def sign(proof: dict[str, object], key: bytes) -> None:
    proof["integrity"] = {
        "algorithm": "HMAC-SHA256-DPAPI-CurrentUser-v1",
        "keyId": "pointcloud-proof-hmac-key-v1",
    }
    proof["integrity"]["mac"] = hmac.new(key, mac_payload(proof), hashlib.sha256).hexdigest()  # type: ignore[index]


def serialized_proof(proof: dict[str, object]) -> bytes:
    return json.dumps(proof, ensure_ascii=False, sort_keys=True, indent=2).encode()


def receipt_record(proof: dict[str, object], laser_dir: Path, payload: bytes) -> bytes:
    context = proof["productionContext"]
    integrity = proof["integrity"]
    assert isinstance(context, dict) and isinstance(integrity, dict)
    return canonical({
        "receiptVersion": 1,
        "scanRunId": context["scanRunId"],
        "createdUtc": proof["createdUtc"],
        "casePathSha256": sha256(str(laser_dir.resolve()).encode()),
        "contextSha256": sha256(canonical(context)),
        "inputEvidenceSha256": sha256(canonical(proof["inputs"])),
        "poseEvidenceSha256": sha256(canonical(proof["artifacts"])),
        "proofSha256": sha256(payload),
        "proofMac": integrity["mac"],
    })


def validate(
    proof: dict[str, object],
    proof_payload: bytes,
    receipts: dict[str, bytes],
    laser_dir: Path,
    *,
    key: bytes,
    now: datetime,
    endpoint: str,
    camera: str,
    hand_eye: str,
) -> None:
    if proof.get("schemaVersion") != 3:
        raise ValueError("legacy schema")
    integrity = proof.get("integrity")
    if not isinstance(integrity, dict) or set(integrity) != {"algorithm", "keyId", "mac"}:
        raise ValueError("missing integrity")
    expected_mac = hmac.new(key, mac_payload(proof), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(str(integrity.get("mac")), expected_mac):
        raise ValueError("forged proof")

    context = proof.get("productionContext")
    if not isinstance(context, dict) or context.get("contextRevision") != 1:
        raise ValueError("missing context")
    run_id = str(context.get("scanRunId"))
    UUID(run_id)
    expected_receipt = receipt_record(proof, laser_dir, proof_payload)
    if run_id not in receipts or not hmac.compare_digest(receipts[run_id], expected_receipt):
        raise ValueError("missing or mismatched protected receipt")
    if proof.get("generationMode") not in {"liveScan", "validatedRebuild"}:
        raise ValueError("untrusted generation mode")
    if context.get("origin") != "liveRobotCameraScan":
        raise ValueError("offline origin")
    if context.get("robotEndpoint") != endpoint:
        raise ValueError("endpoint mismatch")
    if context.get("cameraSection") != camera:
        raise ValueError("camera mismatch")
    if context.get("handEyeSha256") != hand_eye:
        raise ValueError("calibration mismatch")

    created = parse_strict_utc(proof.get("createdUtc"))
    scanned = parse_strict_utc(context.get("scanStartedUtc"))
    for value in (created, scanned):
        if now - value > MAX_AGE:
            raise ValueError("stale")
        if value - now > MAX_FUTURE_SKEW:
            raise ValueError("future")
    if scanned - created > MAX_FUTURE_SKEW:
        raise ValueError("invalid scan/proof order")

    inputs = proof.get("inputs")
    if not isinstance(inputs, list) or not inputs:
        raise ValueError("unbound input")
    for expected in inputs:
        if not isinstance(expected, dict):
            raise ValueError("bad input evidence")
        name = expected.get("relativePath")
        if not isinstance(name, str) or Path(name).name != name:
            raise ValueError("input escapes LaserPoint")
        if evidence(laser_dir / name) != expected:
            raise ValueError("input changed")

    artifacts = proof.get("artifacts")
    if not isinstance(artifacts, dict) or not isinstance(artifacts.get("authorizedPose"), dict):
        raise ValueError("missing authorized pose")
    pose = artifacts["authorizedPose"]
    assert isinstance(pose, dict)
    pose_name = pose.get("relativePath")
    if not isinstance(pose_name, str) or evidence(laser_dir / pose_name) != pose:
        raise ValueError("authorized pose changed")


def must_reject(*args: object, **kwargs: object) -> None:
    try:
        validate(*args, **kwargs)  # type: ignore[arg-type]
    except (AssertionError, KeyError, TypeError, ValueError, FileNotFoundError):
        return
    raise AssertionError("unsafe proof mutation was accepted")


def main() -> int:
    now = datetime.now(timezone.utc)
    key = os.urandom(32)
    endpoint = "tcp:[192.0.2.10]:5000"
    camera = "CAMERA0"
    hand_eye = sha256(b"validated hand-eye matrix")
    with TemporaryDirectory(prefix="pcq-context-") as temporary:
        laser_dir = Path(temporary) / "CaseA" / "LaserPoint"
        laser_dir.mkdir(parents=True)
        raw = laser_dir / "PreciseLaserPoint.txt"
        raw.write_bytes(b"1 10.0 20.0 30.0\n2 11.0 21.0 31.0\n")
        pose_path = laser_dir / "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt"
        pose_path.write_bytes(b"1 1 10 20 30 0 0 0\n2 2 11 21 31 0 0 0\n")
        run_id = str(uuid4())
        proof: dict[str, object] = {
            "schemaVersion": 3,
            "purpose": "production",
            "createdUtc": utc_text(now),
            "generationMode": "liveScan",
            "productionContext": {
                "contextRevision": 1,
                "origin": "liveRobotCameraScan",
                "scanRunId": run_id,
                "scanStartedUtc": utc_text(now - timedelta(minutes=2)),
                "robotEndpoint": endpoint,
                "cameraSection": camera,
                "handEyeSha256": hand_eye,
            },
            "inputs": [evidence(raw)],
            "artifacts": {"authorizedPose": evidence(pose_path)},
        }
        sign(proof, key)
        payload = serialized_proof(proof)
        receipts = {run_id: receipt_record(proof, laser_dir, payload)}
        validate(proof, payload, receipts, laser_dir, key=key, now=now,
                 endpoint=endpoint, camera=camera, hand_eye=hand_eye)

        # A hand-authored PASS/pose substitution cannot reuse the original MAC/receipt.
        forged = copy.deepcopy(proof)
        forged["artifacts"]["authorizedPose"]["sha256"] = "0" * 64  # type: ignore[index]
        forged_payload = serialized_proof(forged)
        must_reject(forged, forged_payload, receipts, laser_dir, key=key, now=now,
                    endpoint=endpoint, camera=camera, hand_eye=hand_eye)

        # Signing with an attacker key and inventing JSON fields still cannot create the protected receipt.
        wrong_key = os.urandom(32)
        resigned = copy.deepcopy(proof)
        sign(resigned, wrong_key)
        resigned_payload = serialized_proof(resigned)
        must_reject(resigned, resigned_payload, receipts, laser_dir, key=key, now=now,
                    endpoint=endpoint, camera=camera, hand_eye=hand_eye)

        must_reject(proof, payload, {}, laser_dir, key=key, now=now,
                    endpoint=endpoint, camera=camera, hand_eye=hand_eye)
        legacy = copy.deepcopy(proof)
        legacy["schemaVersion"] = 2
        must_reject(legacy, serialized_proof(legacy), receipts, laser_dir, key=key, now=now,
                    endpoint=endpoint, camera=camera, hand_eye=hand_eye)

        raw.write_bytes(raw.read_bytes() + b"3 12.0 22.0 32.0\n")
        must_reject(proof, payload, receipts, laser_dir, key=key, now=now,
                    endpoint=endpoint, camera=camera, hand_eye=hand_eye)

    print("point-cloud schema-3 HMAC/receipt/input behavioral tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
