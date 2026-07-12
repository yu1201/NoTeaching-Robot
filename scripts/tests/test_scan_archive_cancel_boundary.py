#!/usr/bin/env python3
"""Behavioral proof for the ScanDataUploader's controlled bsdtar boundary.

The source-wiring test verifies that ScanDataUploader uses this exact absolute
compressor/argument/kill sequence.  This test exercises the OS boundary itself:
one complete archive must have the expected central-directory inventory, and a
large in-flight archive must be killable promptly with its partial output
removable while the source remains intact.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import time
import zipfile


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def system_tar() -> Path:
    root = os.environ.get("SystemRoot", r"C:\Windows")
    candidate = Path(root) / "System32" / "tar.exe"
    require(candidate.is_file(), f"fixed system tar is unavailable: {candidate}")
    return candidate


def run_success_case(tar: Path, root: Path) -> None:
    source_root = root / "success-root"
    case = source_root / "Robot-1" / "Case A"
    case.mkdir(parents=True)
    expected = {
        "Robot-1/Case A/alpha.txt": b"alpha\n",
        "Robot-1/Case A/\u70b9\u4e91.bin": bytes(range(64)),
    }
    for relative, data in expected.items():
        path = source_root / Path(relative)
        path.write_bytes(data)

    file_list = root / "success.files"
    file_list.write_bytes(b"\0".join(name.encode("utf-8") for name in expected) + b"\0")
    archive = root / "success.zip"
    subprocess.run(
        [
            str(tar),
            "-c",
            "--format",
            "zip",
            "--options",
            "hdrcharset=UTF-8",
            "-f",
            str(archive),
            "-C",
            str(source_root),
            "--null",
            "--no-recursion",
            "-T",
            str(file_list),
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=20,
    )
    with zipfile.ZipFile(archive) as opened:
        infos = opened.infolist()
        require(len(infos) == len(expected), "central-directory count mismatch")
        actual_inventory = {info.filename: info.file_size for info in infos}
        expected_inventory = {name: len(data) for name, data in expected.items()}
        require(
            actual_inventory == expected_inventory,
            f"central-directory path/size mismatch: {actual_inventory!r} != {expected_inventory!r}",
        )


def run_cancel_case(tar: Path, root: Path) -> None:
    source_root = root / "cancel-root"
    case = source_root / "Robot-2" / "Case-B"
    case.mkdir(parents=True)
    source = case / "large-random.bin"
    # Incompressible enough that the archive is reliably still running when its
    # output first appears, without consuming the production 768 MiB allowance.
    remaining = 96 * 1024 * 1024
    with source.open("wb") as stream:
        while remaining:
            chunk = os.urandom(min(1024 * 1024, remaining))
            stream.write(chunk)
            remaining -= len(chunk)

    file_list = root / "cancel.files"
    file_list.write_bytes(b"Robot-2/Case-B/large-random.bin\0")
    archive = root / "cancel.zip"
    proc = subprocess.Popen(
        [
            str(tar),
            "-c",
            "--format",
            "zip",
            "--options",
            "hdrcharset=UTF-8",
            "-f",
            str(archive),
            "-C",
            str(source_root),
            "--null",
            "--no-recursion",
            "-T",
            str(file_list),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and proc.poll() is None:
        if archive.exists() and archive.stat().st_size > 0:
            break
        time.sleep(0.01)
    require(proc.poll() is None, "large archive completed before cancellation boundary was exercised")

    cancel_started = time.monotonic()
    proc.kill()  # Same hard process boundary as QProcess::kill on Windows.
    proc.wait(timeout=5)
    cancel_elapsed = time.monotonic() - cancel_started
    require(cancel_elapsed < 5.0, f"compressor did not terminate promptly: {cancel_elapsed:.3f}s")
    archive.unlink(missing_ok=True)
    file_list.unlink(missing_ok=True)
    require(not archive.exists(), "partial archive survived cancellation cleanup")
    require(source.is_file() and source.stat().st_size == 96 * 1024 * 1024, "source was changed by cancellation")


def main() -> int:
    tar = system_tar()
    with tempfile.TemporaryDirectory(prefix="scan-archive-cancel-") as temp:
        root = Path(temp)
        run_success_case(tar, root)
        run_cancel_case(tar, root)
    print("PASS: fixed bsdtar boundary preserves inventory and is promptly killable with no partial ZIP")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
