#!/usr/bin/env python3
"""Static regression checks for the three server-authenticated online-service roles."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8", errors="replace")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    config = read("include/OnlineServicesConfig.h")
    login = read("src/OnlineServicesLoginDialog.cpp")
    app = read("src/QtWidgetsApplication4.cpp")
    dialog = read("src/OnlineServicesDialog.cpp")
    uploader = read("src/ScanDataUploader.cpp")

    for token in (
        'QStringLiteral("devicedata")',
        'QStringLiteral("ftpoperator")',
        'QStringLiteral("uploader")',
        "enum class AccessLevel",
        "AccessLevel::Upload",
        "AccessLevel::Ftp",
        "AccessLevel::Full",
        "IsDefaultFtpAccount",
    ):
        require(token in config, f"missing fixed role mapping: {token}")

    require("ftp.connect()" in login, "online-services login is not verified by the server")
    require("OnlineServicesLoginDialog login(this)" in app, "clicking online services does not require login")
    require("SetFtpUser(onlineAccount)" in app and "SetFtpPassword(login.Password())" in app,
            "verified default FTP credential is not persisted")
    require("SetAdminToken" not in dialog and "m_adminTokenEdit" not in dialog,
            "admin token is still exposed in the server configuration UI")
    require("m_serverHostEdit" in dialog and "m_deviceNameEdit" in dialog,
            "consolidated server-IP/device-name configuration is missing")
    require("m_serverConfigNavRow" in dialog and "setNavRowEnabled(m_serverConfigNavRow, ftpAllowed" in dialog,
            "upload-only role can still open server configuration")
    require("setNavRowEnabled(m_remoteNavRow, ftpAllowed" in dialog,
            "FTP remote browse/download permission is not wired")
    require("m_remoteDeleteBtn->setEnabled(!m_remoteBusy && fullAllowed)" in dialog,
            "destructive remote operation is not limited to full access")
    require("if (!HasFullAccess())" in dialog and 'm_cardDisk->setText(QStringLiteral("无权限"))' in dialog,
            "server information is visible to the FTP role")
    for token in (
        "CanUseSecureAdminTransport",
        "RefreshServerStatsViaFtp",
        "SetServerStatusBanner",
        "ServerStatusLevel::Error",
        "服务器读取失败：FTP 无法列出数据目录",
        'ftp.listFiles("/data"',
        'QStringLiteral("FTP 只读统计 · 磁盘容量需 HTTPS 管理通道")',
    ):
        require(token in dialog, f"secure server-stats fallback is missing: {token}")
    require('if (!CanUseSecureAdminTransport() || !adminUrl.isValid())' in dialog,
            "public plaintext HTTP can send the management token")
    require(uploader.count("IsDefaultFtpAccount") >= 2,
            "background upload does not enforce the three fixed accounts")
    require("updateGroup" in dialog and "m_checkUpdateBtn" in dialog,
            "online upgrade page was removed from restricted sessions")
    for token in (
        "m_uploadCurrentLabel",
        "m_uploadProgressBar",
        "m_uploadQueueLabel",
        "RefreshUploadUi",
        "CurrentProgress()",
        "uploadProgress",
    ):
        require(token in dialog, f"upload progress UI is missing: {token}")
    require('setRange(0, 100)' in dialog and 'setFormat(QStringLiteral("%1%")' in dialog,
            "upload progress is not a determinate percentage bar")

    print("PASS: online-service roles, server login, permissions, and upload progress UI are wired")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
