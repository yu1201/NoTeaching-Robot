#!/usr/bin/env python3
"""Static gate for the fail-closed account recovery and controlled repair page.

The recovery state must not leave normal login controls looking actionable, and
its repair action must hand the database to the provenance-checked migrator only
after this process exits.  Unknown damage must remain locked.
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
        "m_pAuthRepairBtn->setVisible(recoveryRequired);",
        "m_pAuthRepairBtn->setEnabled(recoveryRequired && !m_bAuthRepairRunning);",
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
        "QPushButton* m_pAuthRepairBtn;" in HEADER
        and "void StartAccountDatabaseRepair();" in HEADER,
        "controlled account repair action is not retained as managed UI state",
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
        "m_bAccountRecoveryRequired" in show_auth
        and "m_pAuthRepairBtn->setFocus(Qt::OtherFocusReason);" in show_auth
        and "m_pAuthCancelBtn->setFocus(Qt::OtherFocusReason);" in show_auth,
        "keyboard focus does not prefer repair with a close fallback during recovery",
    )

    repair = section(
        "void QtWidgetsApplication4::StartAccountDatabaseRepair()",
        "void QtWidgetsApplication4::CheckPendingAccountDatabaseRepairResult()",
    )
    for token in (
        'QStringLiteral("tools/ConfigMigrate.exe")',
        'QStringLiteral("tools/migrate_config_to_sqlite.py")',
        'QStringLiteral("--print-source-sha256")',
        "AuthRepairFileSha256(migrationSourcePath)",
        "m_pAuthRepairBtn->repaint();",
        "provenanceProbe.waitForStarted(5000)",
        "provenanceProbe.waitForFinished(15000)",
        "ApplicationInstanceGuard::MachineMutexName(",
        "Get-Process -Id ([int]$env:AUTH_REPAIR_PID)",
        "--source $env:AUTH_REPAIR_SOURCE --db $env:AUTH_REPAIR_DB --encrypt --scrub-legacy-credentials",
        'environment.insert(QStringLiteral("AUTH_REPAIR_TOOL")',
        'environment.insert(QStringLiteral("AUTH_REPAIR_PID")',
        "TrustedWindowsPowerShellPath()",
        "launcher.setProgram(powerShellPath)",
        "CREATE_NO_WINDOW",
        "launcher.startDetached()",
        "QApplication::quit();",
    ):
        require(token in repair, f"controlled repair handoff missing: {token}")
    require(
        "--overwrite" not in repair,
        "automatic repair must never request a destructive overwrite migration",
    )
    require(
        "$status+'`r`n'" not in repair
        and "WriteAllText($env:AUTH_REPAIR_STATUS,$status," in repair,
        "repair status must be written as the exact success/failure token",
    )
    require(
        repair.index("Get-Process -Id ([int]$env:AUTH_REPAIR_PID)")
        < repair.index("--source $env:AUTH_REPAIR_SOURCE"),
        "repair tool may run before the current process exits",
    )
    require(
        ".arg(" not in repair,
        "repair bootstrap must not interpolate writable paths into shell source",
    )
    require(
        'launcher.setProgram(QStringLiteral("powershell.exe"))' not in repair,
        "repair bootstrap must not resolve PowerShell through cwd or PATH",
    )
    for token in (
        "GetSystemDirectoryW(",
        "FILE_ATTRIBUTE_REPARSE_POINT",
        "powerShellInfo.canonicalFilePath()",
    ):
        require(token in SOURCE, f"trusted system PowerShell resolution missing: {token}")

    result = section(
        "void QtWidgetsApplication4::CheckPendingAccountDatabaseRepairResult()",
        "void QtWidgetsApplication4::RefreshAuthModeUi()",
    )
    require(
        "if (!m_bAccountRecoveryRequired)" in result
        and 'if (result == QStringLiteral("success"))' in result
        and "已通过程序复核，可以正常登录" in result,
        "repair success is trusted without the restarted runtime integrity gate",
    )

    print("PASS: account recovery stays fail-closed and offers controlled repair")


if __name__ == "__main__":
    main()
