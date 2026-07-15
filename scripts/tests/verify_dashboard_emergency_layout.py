#!/usr/bin/env python3
"""Static gate for the dashboard emergency-stop side-column layout."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE = (REPO_ROOT / "src" / "QtWidgetsApplication4.cpp").read_text(
    encoding="utf-8-sig"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    begin = SOURCE.index("QSplitter* robotInfoSplitter = new QSplitter(Qt::Vertical);")
    end = SOURCE.index(
        "connect(m_pCurrentUserButton, &QPushButton::clicked", begin
    )
    layout = SOURCE[begin:end]

    for token in (
        'QGroupBox* emergencyStopGroup = new QGroupBox("任务安全控制"',
        'emergencyStopGroup->setObjectName("DashboardEmergencyStopGroup")',
        "emergencyStopGroup->setFixedWidth(190)",
        "emergencyStopPanelLayout->addWidget(\n\t\tm_pDashboardEmergencyStopBtn",
        "monitorAndSafetyLayout->addWidget(robotInfoSplitter, 1)",
        "monitorAndSafetyLayout->addWidget(emergencyStopGroup, 0)",
        "dashboardLayout->addLayout(monitorAndSafetyLayout, 1)",
        "不替代控制柜、示教器急停",
    ):
        require(token in layout, f"emergency side-column layout missing: {token}")

    require(
        "emergencyStopLayout" not in layout,
        "emergency stop still occupies a separate dashboard row",
    )
    require(
        "toolLayout->addWidget(m_pDashboardEmergencyStopBtn" not in SOURCE
        and "dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(m_pDashboardEmergencyStopBtn" not in SOURCE,
        "emergency stop was incorrectly moved into the field-tool panel",
    )
    require(
        'm_pDashboardEmergencyStopBtn->setEnabled(false);' in SOURCE
        and "m_pDashboardEmergencyStopBtn->setEnabled(robotOperationBusy);" in SOURCE,
        "emergency stop no longer stays disabled when no robot program is active",
    )

    print("PASS: emergency stop uses the monitor/log side column")


if __name__ == "__main__":
    main()
