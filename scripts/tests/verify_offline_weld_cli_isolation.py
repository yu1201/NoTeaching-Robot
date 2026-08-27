from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    main_cpp = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
    control_cpp = (ROOT / "src" / "ContralUnit.cpp").read_text(encoding="utf-8")
    adaptor_header = (ROOT / "include" / "RobotDriverAdaptor.h").read_text(encoding="utf-8")
    adaptor_cpp = (ROOT / "src" / "RobotDriverAdaptor.cpp").read_text(encoding="utf-8")

    offline_begin = main_cpp.find("bool IsOfflineWeldFileInvocation")
    window = main_cpp.find("QtWidgetsApplication4 window")
    require(0 <= offline_begin < window,
            "offline weld-file CLI classifier must run before window/driver construction")
    offline_body = main_cpp[offline_begin:window]
    for option in (
        "--rebuild-measure-weld-files",
        "--generate-step-weld-program",
    ):
        require(option in offline_body,
                f"offline weld-file CLI option is not isolated: {option}")
    require("RobotDriverAdaptor::s_connectDriversAtConstruct = false" in offline_body,
            "offline weld-file CLI still allows synchronous controller connection")
    require("RobotDriverAdaptor::s_startStateMonitorsAtConstruct = false" in offline_body,
            "offline weld-file CLI still allows background controller monitoring")

    require("static std::atomic<bool> s_startStateMonitorsAtConstruct" in adaptor_header,
            "state-monitor construction switch is missing from the driver contract")
    require("RobotDriverAdaptor::s_startStateMonitorsAtConstruct{ true }" in adaptor_cpp,
            "normal GUI/hardware behavior is not preserved by default")
    require("RobotDriverAdaptor::s_startStateMonitorsAtConstruct.load()" in control_cpp,
            "control-unit construction does not honor offline monitor isolation")

    print("PASS: offline weld-file CLI loads configuration without connecting robot hardware")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
