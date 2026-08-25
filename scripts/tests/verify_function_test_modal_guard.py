from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    require(begin >= 0, f"missing section: {start}")
    finish = text.find(end, begin + len(start))
    require(finish >= 0, f"missing section end: {end}")
    return text[begin:finish]


def main() -> int:
    header = (ROOT / "include/FunctionTestDialog.h").read_text(encoding="utf-8")
    source = (ROOT / "src/FunctionTestDialog.cpp").read_text(encoding="utf-8")
    app = (ROOT / "src/QtWidgetsApplication4.cpp").read_text(encoding="utf-8")

    constructor = section(
        source,
        "FunctionTestDialog::FunctionTestDialog(",
        "bool FunctionTestDialog::RunDashboardTool(",
    )
    require("m_pCommandContent = new QWidget(commandScrollArea)" in constructor,
            "ordinary function-test controls do not have one common busy container")
    require("commandScrollArea->setWidget(m_pCommandContent)" in constructor,
            "busy container is not the actual scroll content")
    require("QWidget* m_pCommandContent" in header,
            "function-test busy container is not retained")

    dashboard = section(
        source,
        "bool FunctionTestDialog::RunDashboardTool(",
        "void FunctionTestDialog::closeEvent(",
    )
    require("RobotOperationLease::AnyActive()" in dashboard and "return false" in dashboard,
            "dashboard tool dispatch can open a modal input while a robot task is active")

    close = section(
        source,
        "void FunctionTestDialog::closeEvent(",
        "RobotDriverAdaptor* FunctionTestDialog::GetFirstDriverWithCapability(",
    )
    require("QDialog::closeEvent(event)" in close,
            "embedded page cannot return to the homepage while a robot task runs")
    require("QMessageBox::" not in close,
            "return-to-home path can create a blocking modal dialog")

    refresh = section(
        source,
        "void FunctionTestDialog::RefreshMotionButtonState()",
        "void FunctionTestDialog::AppendLog(",
    )
    for token in (
        "m_bRobotCommandRunning",
        "RobotOperationLease::AnyActive()",
        "m_pCommandContent->setEnabled(!busy)",
    ):
        require(token in refresh, f"function-test modal guard missing: {token}")

    async_ranges = (
        ("void FunctionTestDialog::FanucCallJobTest()", "void FunctionTestDialog::FanucUploadLsTest()"),
        ("void FunctionTestDialog::FanucMovlTest()", "void FunctionTestDialog::FanucMovjTest()"),
        ("void FunctionTestDialog::FanucMovjTest()", "void FunctionTestDialog::FanucMoveZeroTest()"),
        ("void FunctionTestDialog::FanucMoveZeroTest()", "void FunctionTestDialog::FanucCaptureKinematicsSample()"),
    )
    for start, end in async_ranges:
        body = section(source, start, end)
        require("std::thread" in body and "RefreshMotionButtonState()" in body,
                f"robot command is not asynchronously guarded: {start}")
        require("QMessageBox::information(self" not in body,
                f"robot completion can raise a modal over the homepage: {start}")

    dashboard_refresh = section(
        app,
        "void QtWidgetsApplication4::RefreshDashboardConnectionState()",
        "bool QtWidgetsApplication4::EnsureRobotUiActionIdle(",
    )
    require("RobotOperationLease::AnyActive()" in dashboard_refresh
            and "m_robotOperationWidgets" in dashboard_refresh
            and "widget->setEnabled(baseEnabled && !robotOperationBusy)" in dashboard_refresh,
            "homepage ordinary robot controls remain enabled while a robot task is active")
    emergency_stop = section(
        app,
        "void QtWidgetsApplication4::RobotEmergencyStop()",
        "void QtWidgetsApplication4::RobotSwitchStepMode()",
    )
    require("warning->setWindowModality(Qt::NonModal)" in emergency_stop,
            "failed homepage safety stop can block its own retry button")
    require("QMessageBox::critical(self" not in emergency_stop,
            "failed homepage safety stop still uses a blocking critical dialog")

    print("PASS: function-test robot commands freeze all ordinary modal-producing controls while homepage return stays available")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
