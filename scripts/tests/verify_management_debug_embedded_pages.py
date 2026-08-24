"""Verify Debug menu tools that must stay inside the management content stack."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    require(begin >= 0, f"missing section start: {start}")
    finish = text.find(end, begin + len(start))
    require(finish >= 0, f"missing section end: {end}")
    return text[begin:finish]


def main() -> int:
    app = (ROOT / "src/QtWidgetsApplication4.cpp").read_text(encoding="utf-8")
    archive_header = (ROOT / "include/ResultArchiveDialog.h").read_text(encoding="utf-8")
    archive_source = (ROOT / "src/ResultArchiveDialog.cpp").read_text(encoding="utf-8")

    require('createManagementAction("结果打包压缩", [this]() { OpenResultArchiveDialog(); })' in app,
            "result archive is missing from the management Debug menu")
    archive_open = section(
        app,
        "void QtWidgetsApplication4::OpenResultArchiveDialog()",
        "ScanDataUploader* QtWidgetsApplication4::EnsureScanDataUploader()",
    )
    for token in (
        "m_pManagementStack",
        "m_pResultArchivePage = new ResultArchiveDialog(resultRoot, m_pManagementStack)",
        "PrepareEmbeddedPage(m_pResultArchivePage, m_pManagementStack)",
        "ShowManagementEmbeddedPage(m_pResultArchivePage)",
    ):
        require(token in archive_open, f"result archive embedded wiring missing: {token}")
    for forbidden in ("dlg->show()", "findChild<ResultArchiveDialog*>", "WA_DeleteOnClose"):
        require(forbidden not in archive_open,
                f"result archive still uses standalone-window wiring: {forbidden}")
    require("class ResultArchiveDialog : public QWidget" in archive_header,
            "result archive remains a QDialog instead of an embedded page")
    require("setWindowFlags(Qt::Widget)" in archive_source,
            "result archive does not force widget page semantics")
    require("返回管理首页" in archive_source,
            "embedded result archive lacks a management-page return action")
    require("watched == m_pResultArchivePage" in app,
            "closing the embedded archive page does not route back to management home")

    print("PASS: scan pose-variation and result archive are management Debug embedded pages")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
