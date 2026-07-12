from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    header = (ROOT / "include" / "ApplicationInstanceGuard.h").read_text(encoding="utf-8")
    source = (ROOT / "src" / "ApplicationInstanceGuard.cpp").read_text(encoding="utf-8")
    main_cpp = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")

    for token in (
        "QLockFile",
        "setStaleLockTime(0)",
        "tryLock(0)",
        "getLockInfo",
        "QtWidgetsApplication4/robot-hardware-control/v1",
    ):
        require(token in source, f"cross-process guard mechanism missing: {token}")
    for token in (
        "Global\\\\NoTeaching-Robot-Hardware-Control-v1-",
        "CreateMutexW",
        "ERROR_ALREADY_EXISTS",
        "ERROR_ACCESS_DENIED",
        "CloseHandle",
    ):
        require(token in source, f"machine-wide Windows guard mechanism missing: {token}")
    require("static Ptr TryAcquire" in header,
            "cross-process guard API is missing")
    require("ApplicationInstanceGuard::TryAcquire" in main_cpp,
            "main does not acquire the cross-process robot-control guard")

    path_probe = main_cpp.find("arguments.contains(QStringLiteral(\"--print-app-paths-json\"))")
    worker = main_cpp.find("--pointcloud-extract-worker")
    guard = main_cpp.find("ApplicationInstanceGuard::TryAcquire")
    drivers = main_cpp.find("RobotDriverAdaptor::s_connectDriversAtConstruct")
    window = main_cpp.find("QtWidgetsApplication4 window")
    require(0 <= path_probe < guard,
            "read-only path probe is not exempt before the process guard")
    require(0 <= worker < guard,
            "isolated point-cloud worker is not exempt before the process guard")
    require(guard < drivers < window,
            "robot-control process guard is not acquired before driver/window construction")
    require("arguments.contains(QStringLiteral(\"--no-show\"))" in main_cpp[guard:drivers],
            "headless hardware CLI does not receive a deterministic lock failure path")

    print("PASS: GUI and hardware CLI share one cross-process robot-control guard")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
