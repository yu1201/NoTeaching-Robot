"""Static guard for honest, synchronized measure-then-weld progress UI.

The bar stays determinate (0..100) so it reads as numeric flow progress instead
of a looping loading animation.  Values may only move at observable scan
positions or real phase milestones; the timer is allowed to refresh elapsed
text, never to manufacture percentage points.
"""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def section(text: str, start: str, end: str) -> str:
    start_index = text.find(start)
    if start_index < 0:
        raise AssertionError(f"missing section start: {start}")
    end_index = text.find(end, start_index + len(start))
    if end_index < 0:
        raise AssertionError(f"missing section end: {end}")
    return text[start_index:end_index]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    dialog = read("src/MeasureThenWeldDialog.cpp")
    header = read("include/MeasureThenWeldDialog.h")

    monitor = section(
        dialog,
        "class RunMonitorDialog final : public QDialog",
        "// ===== 相机时间补偿自动标定 =====",
    )
    reset = section(
        dialog,
        "void MeasureThenWeldDialog::ResetProgress(",
        "void MeasureThenWeldDialog::SetProgress(",
    )
    determinate = section(
        dialog,
        "void MeasureThenWeldDialog::SetProgress(",
        "void MeasureThenWeldDialog::SetProgressBusy(",
    )
    busy = section(
        dialog,
        "void MeasureThenWeldDialog::SetProgressBusy(",
        "void MeasureThenWeldDialog::FinishProgress(",
    )
    finish = section(
        dialog,
        "void MeasureThenWeldDialog::FinishProgress(",
        "void MeasureThenWeldDialog::UpdateProgressAnimation(",
    )
    timer_update = section(
        dialog,
        "void MeasureThenWeldDialog::UpdateProgressAnimation(",
        "void MeasureThenWeldDialog::SetRunning(",
    )

    monitor_value = section(
        monitor,
        "void SetProgressValue(int value)",
        "void SetProgressBusy(int value, const QString& elapsedText)",
    )
    monitor_busy = section(
        monitor,
        "void SetProgressBusy(int value, const QString& elapsedText)",
        "void SetBusyElapsed(int value, const QString& elapsedText)",
    )
    monitor_elapsed = section(
        monitor,
        "void SetBusyElapsed(int value, const QString& elapsedText)",
        "void SetProgressResult(bool ok, int value)",
    )

    require(
        monitor_value.find("setRange(0, 100)") < monitor_value.find("setValue(safeValue)"),
        "run monitor determinate progress must restore 0..100 before setting a value",
    )
    require(
        monitor_busy.find("m_pProgress->setRange(0, 100)")
        < monitor_busy.find("m_pProgress->setValue(safeValue)")
        and "SetBusyElapsed(safeValue, elapsedText)" in monitor_busy,
        "run monitor busy state must remain a synchronized 0..100 numeric bar",
    )
    require(
        "m_pProgressBar->setRange(0, 100)" in busy
        and "m_pProgressBar->setValue(m_nProgressValue)" in busy
        and "monitor->SetProgressBusy(m_nProgressValue, elapsedText)" in busy,
        "busy state must synchronize both progress bars as numeric values",
    )
    require(
        "setRange(0, 0)" not in monitor_busy
        and "setRange(0, 0)" not in busy,
        "busy progress must not regress to Qt's looping indeterminate animation",
    )
    require(
        'QStringLiteral("约 %1% · 本阶段耗时 %2")' in monitor_elapsed
        and 'QStringLiteral("约 %1% · 本阶段耗时 %2")' in busy
        and 'QStringLiteral("约 %1% · 本阶段耗时 %2")' in timer_update,
        "main page and run monitor must show approximate percent plus true stage elapsed time",
    )

    for name, body, monitor_call in (
        ("reset", reset, "monitor->SetProgressValue(0)"),
        ("determinate", determinate, "monitor->SetProgressValue(m_nProgressValue)"),
        ("finish", finish, "monitor->SetProgressResult(ok, m_nProgressValue)"),
    ):
        require(
            "m_pProgressBar->setRange(0, 100)" in body and monitor_call in body,
            f"{name} state must restore and synchronize determinate progress",
        )

    require(
        "m_nProgressValue =" not in timer_update
        and "m_nProgressValue++" not in timer_update
        and "++m_nProgressValue" not in timer_update
        and "setValue(" not in timer_update,
        "elapsed-time timer must never manufacture progress values",
    )
    require(
        "(std::min)(95, m_nProgressValue + 1)" not in dialog,
        "legacy timer-generated 95 percent progress must stay removed",
    )
    require(
        "m_pProgressAnimationTimer->setInterval(1000)" in dialog
        and "m_progressStageElapsed" in header
        and "FormatFlowElapsed" in timer_update,
        "busy progress must refresh real stage elapsed time once per second",
    )
    require(
        dialog.count('setObjectName("FlowProgressCard")') >= 2
        and dialog.count('setObjectName("FlowProgressMeta")') >= 2
        and dialog.count('setObjectName("FlowProgressBar")') >= 2,
        "main page and run monitor must share the progress-card visual structure",
    )
    require(
        'flowState=\\"success\\"' in dialog
        and 'flowState=\\"failure\\"' in dialog,
        "progress UI must expose distinct success and failure styling",
    )

    print(
        "PASS: progress is synchronized and determinate, values come from real "
        "milestones, elapsed time is honest, and no loading loop remains"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
