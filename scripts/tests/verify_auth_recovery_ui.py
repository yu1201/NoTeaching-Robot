#!/usr/bin/env python3
"""Static gate for the fail-closed account recovery page.

The recovery state must not leave normal login controls looking actionable, and
its diagnostic text must remain readable on the application's minimum window.
"""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE = (REPO_ROOT / "src" / "QtWidgetsApplication4.cpp").read_text(
    encoding="utf-8-sig"
)
HEADER = (REPO_ROOT / "include" / "QtWidgetsApplication4.h").read_text(
    encoding="utf-8-sig"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(start: str, end: str) -> str:
    begin = SOURCE.find(start)
    finish = SOURCE.find(end, begin + len(start))
    require(begin >= 0 and finish > begin, f"cannot locate source section: {start}")
    return SOURCE[begin:finish]


def main() -> None:
    construction = section(
        "QtWidgetsApplication4::QtWidgetsApplication4(QWidget* parent)",
        "QtWidgetsApplication4::~QtWidgetsApplication4()",
    )
    refresh = section(
        "void QtWidgetsApplication4::RefreshAuthModeUi()",
        "void QtWidgetsApplication4::LoadLoginState()",
    )

    require(
        "QPushButton:disabled" in construction,
        "auth mode buttons have no explicit readable disabled style",
    )
    require(
        'authCard->setMaximumWidth(520)' in construction
        and 'm_pAuthHintLabel->setMinimumWidth(420)' in construction,
        "auth recovery diagnostic is still constrained to the old narrow card",
    )
    require(
        'QStringLiteral("账号认证库升级或完整性校验未通过' in construction,
        "startup recovery message does not explain upgrade/integrity failure",
    )
    for token in (
        "const bool recoveryRequired = m_bAccountRecoveryRequired;",
        "m_pAuthLoginModeBtn->setVisible(showModeSwitch);",
        "m_pAuthRegisterModeBtn->setVisible(showModeSwitch);",
        "m_pAuthSubmitBtn->setVisible(!recoveryRequired);",
        "m_pLoginNameCombo->setVisible(!recoveryRequired);",
        "m_pLoginPasswordEdit->setVisible(!recoveryRequired);",
        'QStringLiteral("关闭程序")',
    ):
        require(token in refresh, f"account recovery UI gate missing: {token}")
    require(
        "QPushButton* m_pAuthCancelBtn;" in HEADER,
        "recovery close action is not retained as managed UI state",
    )
    require(
        "if (m_bAccountRecoveryRequired)" in construction
        and "close();" in construction,
        "recovery close action does not close the locked application",
    )
    show_auth = section(
        "void QtWidgetsApplication4::ShowAuthPage(const QString& promptMessage)",
        "bool QtWidgetsApplication4::VerifyAccount(",
    )
    require(
        "m_bAccountRecoveryRequired && m_pAuthCancelBtn != nullptr" in show_auth
        and "m_pAuthCancelBtn->setFocus(Qt::OtherFocusReason);" in show_auth,
        "keyboard focus is left on a hidden login field during recovery",
    )

    print("PASS: account recovery page is readable, explicit, and non-actionable")


if __name__ == "__main__":
    main()
