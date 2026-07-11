#!/usr/bin/env python3
"""Static and process-level checks for the unified runtime path contract."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


REPO_ROOT = Path(__file__).resolve().parents[2]
WRITABLE_DIRS = ("Data", "Result", "Log", "Temp", "Job")


def same_path(left: str | Path, right: str | Path) -> bool:
    return os.path.normcase(os.path.abspath(os.fspath(left))) == os.path.normcase(
        os.path.abspath(os.fspath(right))
    )


def assert_static_wiring() -> None:
    required_fragments = {
        "src/main.cpp": ("AppPaths::Initialize", "--print-app-paths-json", "PrintCliHelp"),
        "src/CliHelp.cpp": ("QtWidgetsApplication4 command line options", "--data-root <DIR>"),
        "src/ConfigDatabase.cpp": ('WritablePath(QStringLiteral("Data/ConfigStore.db"))',),
        "src/RobotLog.cpp": ("AppPaths::WritablePath", "requestedInfo.isAbsolute"),
        "src/FANUCRobotDriver.cpp": ('WritablePath(QStringLiteral("Job/FANUC', "FindResourcePath"),
        "src/StepRobotDriver.cpp": ('WritablePath(QStringLiteral("Job/STEP',),
        "src/OnlineServicesDialog.cpp": (
            'WritablePath(QStringLiteral("Temp/OnlineUpdate"))',
            "IsSafeRemotePathComponent",
            'WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), fileName)',
        ),
        "src/ScanDataUploader.cpp": (
            'WritablePath(QStringLiteral("Temp/OnlineUpload"))',
            "ResolveSafeResultCaseDir",
            'WritableChildPath(QStringLiteral("Temp/OnlineUpload"), zipName)',
            "IsSafePathComponent(config.deviceName)",
        ),
        "src/PointCloudExtractionProcessor.cpp": ('WritablePath(QStringLiteral("Temp/PointCloudWorkers"))',),
        "src/BcpdModelAligner.cpp": ('WritablePath(QStringLiteral("Temp/BCPD"))',),
        "src/MeasureThenWeldService.cpp": ("AppPaths::CommandLinePath",),
        "src/WeldPoseAverageUpdater.cpp": ("AppPaths::CommandLinePath",),
        "tools/ConfigMigrate_Run.cmd": ("QTWIDGETSAPP4_DATA_ROOT", "--data-root", "DB_PATH"),
        "scripts/build_release_package.ps1": ("--print-source-sha256", "ConfigMigrate.exe is stale"),
    }
    for relative, fragments in required_fragments.items():
        text = (REPO_ROOT / relative).read_text(encoding="utf-8", errors="replace")
        for fragment in fragments:
            if fragment not in text:
                raise AssertionError(f"Missing AppPaths wiring in {relative}: {fragment}")

    main_text = (REPO_ROOT / "src" / "main.cpp").read_text(encoding="utf-8", errors="replace")
    if main_text.find('arguments.contains(QStringLiteral("--help-cli"))') > main_text.find("AppPaths::Initialize"):
        raise AssertionError("--help-cli is not handled before AppPaths initialization")

    current_path_pattern = re.compile(r"QDir::current(?:Path\s*\(|\s*\(\))|std::filesystem::current_path\s*\(")
    allowed = {REPO_ROOT / "src" / "AppPaths.cpp", REPO_ROOT / "src" / "main.cpp"}
    violations: list[str] = []
    for path in sorted((REPO_ROOT / "src").rglob("*.cpp")):
        if path in allowed:
            continue
        for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            if current_path_pattern.search(line) and not line.lstrip().startswith("//"):
                violations.append(f"{path.relative_to(REPO_ROOT)}:{line_number}: {line.strip()}")
    if violations:
        raise AssertionError("Production cwd dependencies remain:\n" + "\n".join(violations))

    for relative in ("src/PointCloudExtractionProcessor.cpp", "src/BcpdModelAligner.cpp"):
        text = (REPO_ROOT / relative).read_text(encoding="utf-8", errors="replace")
        if "QDir::temp" in text or "std::filesystem::temp_directory_path" in text:
            raise AssertionError(f"System temp bypass remains in {relative}")

    online_text = (REPO_ROOT / "src" / "OnlineServicesDialog.cpp").read_text(
        encoding="utf-8", errors="replace"
    )
    for unsafe_fragment in (
        'DownloadTempDir() + "/" +',
        'const QString localZip = localDir + "/" + name',
    ):
        if unsafe_fragment in online_text:
            raise AssertionError(f"Untrusted online filename concatenation remains: {unsafe_fragment}")
    if online_text.count("IsSafeRemotePathComponent") < 10 or "IsSafeArchiveEntry" not in online_text:
        raise AssertionError("OTA/FTP untrusted names are not fail-closed at every use boundary")
    patch_check = online_text.find("if (m_usePatch && !IsSafeExecutableOnlyPatch(m_downloadedPath))")
    tar_extract = online_text.find('"tar -xf')
    if patch_check < 0 or tar_extract < 0 or patch_check > tar_extract:
        raise AssertionError("OTA patch archive is not validated before external tar extraction")

    uploader_text = (REPO_ROOT / "src" / "ScanDataUploader.cpp").read_text(
        encoding="utf-8", errors="replace"
    )
    if 'UploadTempDir() + "/" +' in uploader_text or "zipPath.toStdString()" in uploader_text:
        raise AssertionError("Scan uploader still concatenates or mis-encodes an untrusted local zip path")

    installer = (REPO_ROOT / "installer" / "QtWidgetsApplication4.iss").read_text(
        encoding="utf-8", errors="replace"
    )
    for line in installer.splitlines():
        if "ConfigMigrate" in line and "skipifsourcedoesntexist" in line.lower():
            raise AssertionError("Installer still treats ConfigMigrate as optional")


def runtime_environment(
    runtime_dir: Path,
    environment: dict[str, str] | None = None,
) -> dict[str, str]:
    env = os.environ.copy()
    env.pop("QTWIDGETSAPP4_DATA_ROOT", None)
    env["QT_QPA_PLATFORM"] = "windows"
    env["QT_PLUGIN_PATH"] = str(runtime_dir)
    env["PATH"] = str(runtime_dir) + os.pathsep + env.get("PATH", "")
    if environment:
        env.update(environment)
    return env


def run_diagnostic(
    exe: Path,
    runtime_dir: Path,
    cwd: Path,
    arguments: list[str],
    environment: dict[str, str] | None = None,
) -> tuple[subprocess.CompletedProcess[str], dict[str, object] | None]:
    env = runtime_environment(runtime_dir, environment)
    result = subprocess.run(
        [str(exe), "--print-app-paths-json", *arguments],
        cwd=cwd,
        env=env,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=20,
        check=False,
    )
    payload = None
    for line in reversed(result.stdout.splitlines()):
        try:
            candidate = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict) and candidate.get("schemaVersion") == 1:
            payload = candidate
            break
    return result, payload


def require_success(result: subprocess.CompletedProcess[str], payload: dict[str, object] | None) -> dict[str, object]:
    if result.returncode != 0 or payload is None:
        raise AssertionError(
            f"Path diagnostic failed ({result.returncode})\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return payload


def assert_payload(payload: dict[str, object], cwd: Path, data_root: Path) -> None:
    if not same_path(str(payload["dataRoot"]), data_root):
        raise AssertionError(f"Wrong dataRoot: {payload['dataRoot']} != {data_root}")
    if not same_path(str(payload["originalWorkingDirectory"]), cwd):
        raise AssertionError("OriginalWorkingDirectory did not preserve caller cwd")
    if not same_path(str(payload["currentWorkingDirectory"]), data_root):
        raise AssertionError("Process cwd was not pinned to dataRoot")
    if not same_path(str(payload["databasePath"]), data_root / "Data" / "ConfigStore.db"):
        raise AssertionError("Database path escaped dataRoot")
    if not same_path(str(payload["writableProbe"]), data_root / "Result" / "path-probe"):
        raise AssertionError("WritablePath did not use dataRoot")
    if not same_path(str(payload["safeChildProbe"]), data_root / "Temp" / "OnlineUpdate" / "payload.zip"):
        raise AssertionError("WritableChildPath did not use a contained dataRoot target")
    if not same_path(str(payload["cliRelativeProbe"]), cwd / "relative-path-probe"):
        raise AssertionError("CLI relative path did not use OriginalWorkingDirectory")
    for key in (
        "rejectTraversal",
        "rejectDriveRelative",
        "rejectAbsolute",
        "rejectUnc",
        "rejectAds",
        "rejectMixedTraversal",
        "rejectUnsafeComponent",
        "rejectReservedComponent",
        "rejectComponentSlash",
        "rejectComponentAds",
        "acceptUnicodeComponent",
    ):
        if payload.get(key) is not True:
            raise AssertionError(f"Internal path safety check failed: {key}")
    for name in WRITABLE_DIRS:
        if not (data_root / name).is_dir():
            raise AssertionError(f"Writable layout directory missing: {data_root / name}")


def assert_runtime(exe: Path) -> None:
    exe = exe.resolve()
    if not exe.is_file():
        raise AssertionError(f"Application executable not found: {exe}")
    runtime_dir = exe.parent
    (REPO_ROOT / "tmp").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="app-paths-runtime-", dir=REPO_ROOT / "tmp") as temp_text:
        temp = Path(temp_text)
        caller = temp / "caller"
        caller.mkdir()
        help_data_root = temp / "help-must-not-create-data"
        help_before = {path.relative_to(caller) for path in caller.rglob("*")}
        help_result = subprocess.run(
            [str(exe), "--help-cli", "--data-root", str(help_data_root)],
            cwd=caller,
            env=runtime_environment(runtime_dir),
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=20,
            check=False,
        )
        if help_result.returncode != 0 or "QtWidgetsApplication4 command line options" not in help_result.stdout:
            raise AssertionError(
                f"--help-cli did not exit cleanly\nstdout:\n{help_result.stdout}\nstderr:\n{help_result.stderr}"
            )
        if help_data_root.exists() or {path.relative_to(caller) for path in caller.rglob("*")} != help_before:
            raise AssertionError("--help-cli initialized runtime data or modified caller cwd")

        data_root = temp / "现场 数据根"
        fake_sdk = data_root / "SDK"
        fake_sdk.mkdir(parents=True)
        (fake_sdk / "shadow.txt").write_text("must not be trusted", encoding="utf-8")
        before = {path.relative_to(caller) for path in caller.rglob("*")}

        result, payload = run_diagnostic(exe, runtime_dir, caller, ["--data-root", str(data_root)])
        payload = require_success(result, payload)
        assert_payload(payload, caller, data_root)
        if not same_path(str(payload["installRoot"]), REPO_ROOT):
            raise AssertionError(f"Development executable did not resolve repo resource root: {payload['installRoot']}")
        if not same_path(str(payload["sdkPath"]), REPO_ROOT / "SDK"):
            raise AssertionError("Writable data-root SDK shadowed trusted development SDK")
        after = {path.relative_to(caller) for path in caller.rglob("*")}
        if after != before:
            raise AssertionError(f"Arbitrary caller cwd was modified: {sorted(after - before)}")

        relative_caller = temp / "relative-caller"
        relative_caller.mkdir()
        result, payload = run_diagnostic(
            exe,
            runtime_dir,
            relative_caller,
            ["--data-root", "relative root", "--data-root=relative root"],
        )
        payload = require_success(result, payload)
        assert_payload(payload, relative_caller, relative_caller / "relative root")

        env_root = temp / "env-root"
        result, payload = run_diagnostic(
            exe,
            runtime_dir,
            caller,
            [],
            {"QTWIDGETSAPP4_DATA_ROOT": str(env_root)},
        )
        payload = require_success(result, payload)
        assert_payload(payload, caller, env_root)
        if payload.get("hasExplicitDataRoot") is not True:
            raise AssertionError("Inherited data root was not marked explicit")

        cli_root = temp / "cli-wins"
        result, payload = run_diagnostic(
            exe,
            runtime_dir,
            caller,
            ["--data-root", str(cli_root)],
            {"QTWIDGETSAPP4_DATA_ROOT": str(env_root)},
        )
        assert_payload(require_success(result, payload), caller, cli_root)

        invalid_cases = (
            ["--data-root"],
            ["--data-root", str(temp / "one"), "--data-root", str(temp / "two")],
            ["--data-root", "C:drive-relative"],
            ["--data-root", "safe:ads"],
        )
        for invalid in invalid_cases:
            result, payload = run_diagnostic(exe, runtime_dir, caller, invalid)
            if result.returncode != 2 or payload is not None:
                raise AssertionError(f"Invalid data-root was not rejected: {invalid}\n{result.stdout}\n{result.stderr}")

        bad_package = temp / "repo-subdir-bad-package"
        bad_package.mkdir()
        copied_exe = bad_package / exe.name
        shutil.copy2(exe, copied_exe)
        copied_data = temp / "copied-data"
        (copied_data / "SDK").mkdir(parents=True)
        result, payload = run_diagnostic(
            copied_exe,
            runtime_dir,
            caller,
            ["--data-root", str(copied_data)],
        )
        payload = require_success(result, payload)
        if not same_path(str(payload["installRoot"]), bad_package):
            raise AssertionError("Repo-subdir package borrowed its ancestor source root")
        if payload.get("sdkPath") not in ("", None):
            raise AssertionError("Bad package borrowed SDK from dataRoot, cwd, or ancestor repo")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--exe",
        type=Path,
        default=REPO_ROOT / "x64" / "Release" / "QtWidgetsApplication4.exe",
    )
    parser.add_argument("--static-only", action="store_true")
    args = parser.parse_args()

    assert_static_wiring()
    if not args.static_only:
        assert_runtime(args.exe)
    print("PASS: AppPaths static wiring" + ("" if args.static_only else " and runtime matrix"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
