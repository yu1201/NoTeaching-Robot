#!/usr/bin/env python3
"""Static regression gate for the robot FTP program inventory monitor."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(text: str, needle: str, message: str) -> None:
    if needle not in text:
        raise AssertionError(f"{message}: missing {needle!r}")


def function_body(text: str, signature: str) -> str:
    start = text.find(signature)
    if start < 0:
        raise AssertionError(f"missing function signature: {signature}")
    opening = text.find("{", start)
    if opening < 0:
        raise AssertionError(f"missing function opening brace: {signature}")
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def main() -> None:
    adaptor_h = read("include/RobotDriverAdaptor.h")
    ftp_bottom = read("src/RobotFtpFileTransfer.cpp")
    step = read("src/StepRobotDriver.cpp")
    fanuc = read("src/FANUCRobotDriver.cpp")
    dialog_h = read("include/MeasureThenWeldDialog.h")
    dialog_cpp = read("src/MeasureThenWeldDialog.cpp")
    app_h = read("include/QtWidgetsApplication4.h")
    app = read("src/QtWidgetsApplication4.cpp")

    for text in (adaptor_h, app_h):
        require(text, "RobotProgramInventory", "inventory contract must be declared")
    require(adaptor_h, "RobotFileTransferSession", "FTP functionality must be an adaptor contract")
    require(adaptor_h, "CreateFileTransferSession", "drivers must create brand-specific FTP sessions")
    if "ftpPassword" in adaptor_h or '#include "FTPClient.h"' in adaptor_h:
        raise AssertionError("adaptor contract must not expose FTP credentials or the FTP transport")

    count = function_body(ftp_bottom, "CountProgramUnits")
    require(count, "std::set<std::string> programNames", "same program stem must be deduplicated")
    require(count, "entry.isDirectory", "directories must not be counted as programs")
    require(count, "find_last_of('.')", "counting must use the executable extension")

    step_query = function_body(step, "STEPRobotCtrl::CreateFileTransferSession")
    step_profile = function_body(step, "STEPRobotCtrl::FileTransferProfile")
    require(step_profile, "m_sStepProjectName.empty()", "STEP inventory must use the configured project")
    require(step_profile, "kStepDynamicJobProjectName", "STEP inventory must fall back to PCRobot")
    require(step_query, 'std::vector<std::string>{ ".srp" }', "STEP SRD data files must not double-count")

    fanuc_query = function_body(fanuc, "FANUCRobotCtrl::CreateFileTransferSession")
    require(fanuc, 'profile.defaultRemoteDirectory = "/md"', "FANUC must inspect MD")
    require(fanuc_query, 'std::vector<std::string>{ ".tp", ".pc" }', "FANUC executable types must be counted")

    require(dialog_h, "void WeldFlowCompleted();", "successful workflow signal is missing")
    if dialog_cpp.count("emit self->WeldFlowCompleted();") != 2:
        raise AssertionError("preset and skip-scan success exits must both trigger the monitor")

    schedule = function_body(app, "QtWidgetsApplication4::ScheduleRobotProgramInventoryCheck")
    for needle in (
        "std::thread",
        "fileTransfer->QueryProgramInventory",
        "programCount <= static_cast<std::size_t>(threshold)",
        "Qt::NonModal",
        "本软件不会自动删除机器人文件",
    ):
        require(schedule, needle, "background read-only monitor contract is incomplete")
    if "deleteFile(" in schedule:
        raise AssertionError("inventory monitor must never delete robot files")
    if "FtpClient" in schedule or "ftpPassword" in schedule:
        raise AssertionError("inventory monitor must not access the FTP transport or credentials")

    require(app, "QTimer::singleShot(8000", "GUI startup check is missing")
    require(app, 'QStringLiteral("程序启动")', "startup trigger must be identified")
    require(app, "&MeasureThenWeldDialog::WeldFlowCompleted", "workflow completion hook is missing")
    require(app, "QTimer::singleShot(2000", "post-workflow controller settle delay is missing")
    require(app, 'QStringLiteral("RobotProgramInventoryMonitor")', "threshold must live in ConfigStore")
    require(app, 'QStringLiteral("100")', "default warning threshold must be 100")
    require(app, "保存提醒阈值", "FTP Job page must expose threshold editing")

    print("ROBOT_PROGRAM_INVENTORY_MONITOR_OK")


if __name__ == "__main__":
    main()
