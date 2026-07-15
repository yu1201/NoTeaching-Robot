#!/usr/bin/env python3
"""Exercise the exact Windows PowerShell account-repair handoff command.

The test uses temporary batch stubs only.  It never opens the repository or
installed ConfigStore.db and never starts the robot application.
"""

from __future__ import annotations

import ast
import ctypes
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time
import uuid


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_PATH = REPO_ROOT / "src" / "QtWidgetsApplication4.cpp"


def extract_repair_command() -> str:
    source = SOURCE_PATH.read_text(encoding="utf-8-sig")
    marker = "const QString repairCommand = QStringLiteral("
    begin = source.index(marker) + len(marker)
    end = source.index(");\n\n\tQProcess launcher;", begin)
    literal_block = source[begin:end]
    literals = re.findall(r'"(?:\\.|[^"\\])*"', literal_block)
    if not literals:
        raise AssertionError("repair command has no C++ string literals")
    command = "".join(ast.literal_eval(item) for item in literals)
    if "AUTH_REPAIR_STATUS" not in command or "Start-Process" not in command:
        raise AssertionError("repair command extraction is incomplete")
    return command


def system_powershell() -> Path:
    buffer = ctypes.create_unicode_buffer(32768)
    length = ctypes.windll.kernel32.GetSystemDirectoryW(buffer, len(buffer))
    if length <= 0 or length >= len(buffer):
        raise AssertionError("GetSystemDirectoryW failed")
    path = Path(buffer.value) / "WindowsPowerShell" / "v1.0" / "powershell.exe"
    if not path.is_file():
        raise AssertionError(f"system PowerShell was not found: {path}")
    return path


def wait_for_file(path: Path, timeout_seconds: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if path.is_file():
            return
        time.sleep(0.05)
    raise AssertionError(f"restart marker was not created: {path}")


def run_case(command: str, powershell: Path, root: Path, exit_code: int) -> None:
    case_root = root / f"case {exit_code} & env"
    source_dir = case_root / "source data"
    database_path = case_root / "target data" / "ConfigStore.db"
    source_dir.mkdir(parents=True)
    database_path.parent.mkdir(parents=True)

    tool_path = case_root / "fake migrator.cmd"
    restart_path = case_root / "fake restart.cmd"
    status_path = case_root / "repair status.txt"
    log_path = case_root / "repair log.txt"
    restarted_path = case_root / "restart marker.txt"
    tool_path.write_text(
        "@echo off\r\n"
        'if not "%~1"=="--source" exit /b 91\r\n'
        'if not "%~2"=="%AUTH_REPAIR_SOURCE%" exit /b 92\r\n'
        'if not "%~3"=="--db" exit /b 93\r\n'
        'if not "%~4"=="%AUTH_REPAIR_DB%" exit /b 94\r\n'
        'if not "%~5"=="--encrypt" exit /b 95\r\n'
        'if not "%~6"=="--scrub-legacy-credentials" exit /b 96\r\n'
        'if not "%~7"=="" exit /b 97\r\n'
        "exit /b %AUTH_REPAIR_EXPECTED_EXIT%\r\n",
        encoding="ascii",
        newline="",
    )
    restart_path.write_text(
        "@echo off\r\n"
        '> "%AUTH_REPAIR_RESTARTED%" echo restarted\r\n'
        "exit /b 0\r\n",
        encoding="ascii",
        newline="",
    )

    environment = os.environ.copy()
    environment.pop("ERRORLEVEL", None)
    environment.update(
        {
            "AUTH_REPAIR_TOOL": str(tool_path),
            "AUTH_REPAIR_SOURCE": str(source_dir),
            "AUTH_REPAIR_DB": str(database_path),
            "AUTH_REPAIR_APP": str(restart_path),
            "AUTH_REPAIR_STATUS": str(status_path),
            "AUTH_REPAIR_LOG": str(log_path),
            "AUTH_REPAIR_MUTEX": (
                "Local\\NoTeachingRobot-AuthRepair-Handoff-Test-" + uuid.uuid4().hex
            ),
            "AUTH_REPAIR_PID": "2147483647",
            "AUTH_REPAIR_EXPECTED_EXIT": str(exit_code),
            "AUTH_REPAIR_RESTARTED": str(restarted_path),
        }
    )
    completed = subprocess.run(
        [
            str(powershell),
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            command,
        ],
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    if completed.returncode != exit_code:
        raise AssertionError(
            f"handoff returned {completed.returncode}, expected {exit_code}: "
            + completed.stderr.decode(errors="replace")
        )
    expected_status = b"success" if exit_code == 0 else f"failed:{exit_code}".encode()
    actual_status = status_path.read_bytes()
    if actual_status != expected_status:
        raise AssertionError(
            f"status bytes differ: expected {expected_status!r}, got {actual_status!r}"
        )
    wait_for_file(restarted_path)


def main() -> None:
    if os.name != "nt":
        print("SKIP: account repair handoff requires Windows PowerShell 5")
        return
    command = extract_repair_command()
    powershell = system_powershell()
    with tempfile.TemporaryDirectory(prefix="NoTeachingRobot-AuthRepair-") as temporary:
        root = Path(temporary)
        run_case(command, powershell, root, 0)
        run_case(command, powershell, root, 17)
    print("PASS: exact PowerShell repair handoff success/failure semantics")


if __name__ == "__main__":
    main()
