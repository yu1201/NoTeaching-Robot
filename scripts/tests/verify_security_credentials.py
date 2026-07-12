#!/usr/bin/env python3
"""Static SEC1 wiring and no-default-secret release gate."""

from __future__ import annotations

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    security = read("src/CredentialSecurity.cpp")
    config = read("src/ConfigDatabase.cpp")
    authentication = read("src/ConfigDatabaseAuthentication.cpp")
    app = read("src/QtWidgetsApplication4.cpp")
    laser_filter = read("src/LaserWeldFilterDialog.cpp")
    laser_filter_header = read("include/LaserWeldFilterDialog.h")
    online_dialog = read("src/OnlineServicesDialog.cpp")
    remote_archive = read("src/RemoteArchiveSecurity.cpp")
    scan_uploader = read("src/ScanDataUploader.cpp")
    ftp_client = read("src/FTPClient.cpp")
    migrator = read("tools/migrate_config_to_sqlite.py")
    migrate_batch = read("tools/ConfigMigrate_Run.cmd")
    online = read("include/OnlineServicesConfig.h")
    ftp = read("include/FTPClient.h")
    project = read("QtWidgetsApplication4.vcxproj")

    for token in (
        "QPasswordDigestor::deriveKeyPbkdf2",
        "kPbkdf2Iterations = 600000",
        "QRandomGenerator::system()",
        "ConstantTimeEquals",
        "CryptProtectData",
        "CryptUnprotectData",
        "CRYPTPROTECT_UI_FORBIDDEN",
    ):
        require(token in security, f"credential security primitive missing: {token}")
    require("CRYPTPROTECT_LOCAL_MACHINE" not in security, "DPAPI machine scope is forbidden")
    require("dpapi:user:v1:" in security, "DPAPI record is not explicitly current-user scoped")

    for token in (
        'constexpr char kSchemaVersion[] = "5"',
        'constexpr char kAuthenticationSemanticVersion[] = "2"',
        "MigrateLegacyAuthenticationSettings",
        "NormalizeExistingAccountProfiles",
        "IsPortableAuthenticationValue",
        "HasUnsafePlaintextConfigStoreResidue",
        "legacy_credential_scrub_state",
        "auth_semantic_version",
        "sensitive_protection",
        "ProtectionPurpose",
        "CredentialSecurity::ProtectForCurrentUser",
    ):
        require(token in config, f"v5 credential migration gate missing: {token}")
    for token in (
        "BEGIN IMMEDIATE",
        "auth_initialized",
        "TryInitializeAuthenticationAccount",
        "TryCreateAccount",
        "TryReadAccountSecurityState",
        "TryCompareAndSetAccountPassword",
        "TryUpdateAccountByAdministrator",
        "TryDeleteAccountByAdministrator",
        "ParseStrictBool",
        "AccountFingerprint",
        "MustChangePassword",
        "UPDATE meta SET value='1'",
    ):
        require(token in authentication, f"atomic authentication bootstrap gate missing: {token}")
    for token in (
        "DPAPI_BACKUP_MAGIC",
        "create_dpapi_database_backup",
        "restore_dpapi_database_backup",
        "SCRUB_STATE_KEY",
        'AUTH_SEMANTIC_VERSION = "2"',
        "_normalize_existing_account_profiles",
        "is_portable_authentication_value",
        "Migrated account/Profile data has no valid administrator",
        "set_legacy_credential_scrub_pending",
        '"before_sha256"',
        '"after_sha256"',
        "_is_explicit_legacy_credential_key",
        'destination = sqlite3.connect(":memory:")',
        "database_bytes = destination.serialize()",
        "SECURITY GATE",
        "appeared after the proven migration scrub",
        'auth_initialized", "1" if authentication_initialized else "0"',
    ):
        require(token in migrator, f"migration credential cleanup gate missing: {token}")
    require("--scrub-legacy-credentials" in migrate_batch, "default migration wrapper does not scrub in-place credentials")

    for token in (
        "MustChangePassword",
        "PromptForcedPasswordChange",
        "ValidNeedsUpgrade",
        "TryCompareAndSetAccountPassword",
        "TryUpdateAccountByAdministrator",
        "TryDeleteAccountByAdministrator",
        "ValidateCurrentAccountSession",
        "RevokePrivilegedUiAccess",
        "m_bInitialAdministratorSetupRequired",
        "m_bSessionRevocationPendingSafetyStop",
        "m_bEnforceInteractiveSessionLeaseGate",
        "_requires_live_account_session",
        "accountSessionTimer",
        "m_sCurrentUserSecurityFingerprint",
        "<敏感值已隐藏>",
        "RememberedCredentialsGroup",
        "RobotOperationLease::SetNewOperationsAllowed",
        "提交 FTP Job 文件操作",
        "主页安全停止入口保持可用",
        "!RobotOperationLease::NewOperationsAllowed()",
    ):
        require(token in app, f"account lifecycle gate missing: {token}")
    require("std::function<bool()> liveSessionGuard" in laser_filter_header,
            "point-cloud external library settings do not receive a live engineer-session guard")
    require(laser_filter.count("!m_liveSessionGuard || !m_liveSessionGuard()") >= 3,
            "point-cloud settings/native pickers do not fail closed after session revocation")
    require(
        re.search(
            r"bool LaserWeldFilterDialog::SaveSettings\(.*?m_liveSessionGuard.*?PointCloudProcessingConfig::Load",
            laser_filter,
            re.S,
        ) is not None,
        "point-cloud external DLL path can be persisted before synchronous session validation",
    )
    require("保存精测点云与外部库设置" in app,
            "point-cloud external library guard is not wired to current engineer-session validation")
    for function_name in ("LoginCurrentAccount", "LoginAsGuest", "RegisterAccount"):
        require(
            re.search(
                rf"void QtWidgetsApplication4::{function_name}\(\)\s*\{{\s*if \(m_bAccountRecoveryRequired\)",
                app,
            ) is not None,
            f"account recovery state does not block {function_name}",
        )
    require(
        re.search(
            r"void QtWidgetsApplication4::ShowDashboardPage\(\).*?ValidateCurrentAccountSession",
            app,
            re.S,
        ) is not None,
        "dashboard does not revalidate the current account session",
    )
    require("PasswordBase64\", passwordBase64" not in app, "new Base64 password write remains reachable")
    require("HashAccountPassword" not in app, "legacy single-SHA writer remains reachable")
    require(
        re.search(
            r'WriteNewAccountRecord\(\s*QStringLiteral\("admin"\),\s*QStringLiteral\("admin"\)',
            app,
        ) is None,
        "fresh installation still seeds a public admin/admin credential",
    )
    for token in (
        "AuthorizePrivilegedAction",
        "m_privilegedActionGuard",
        "configGroup->setVisible(!m_aboutMode && m_remoteBrowseAllowed)",
        "升级源、FTP 与管理令牌仅允许有效的本地管理员会话修改",
        'return user != QStringLiteral("devicedata")',
        "账号管理仅允许已保存的 devicedata 身份并要求管理令牌",
        "普通设备账号权限由服务端固定为 upload-only",
    ):
        require(token in online_dialog, f"online-service live admin gate missing: {token}")

    admin_request_match = re.search(
        r"void OnlineServicesDialog::AdminRequest\(.*?\n\}\n\nvoid OnlineServicesDialog::RefreshServerStats",
        online_dialog,
        re.S,
    )
    require(admin_request_match is not None, "AdminRequest implementation is missing")
    admin_request = admin_request_match.group(0)
    admin_transport_tokens = (
        'FtpUser().trimmed() != QStringLiteral("devicedata")',
        "address.isLoopback()",
        'scheme != QStringLiteral("https")',
        "禁止公网明文 HTTP",
        "OnlineServicesConfig::AdminToken()",
        "QNetworkRequest::SameOriginRedirectPolicy",
        'setRawHeader("X-Admin-Token"',
        "sendCustomRequest",
    )
    for token in admin_transport_tokens:
        require(token in admin_request, f"AdminRequest secure transport gate missing: {token}")
    require(
        admin_request.index('scheme != QStringLiteral("https")')
        < admin_request.index("OnlineServicesConfig::AdminToken()")
        < admin_request.index("QNetworkRequest::SameOriginRedirectPolicy")
        < admin_request.index('setRawHeader("X-Admin-Token"')
        < admin_request.index("sendCustomRequest"),
        "AdminRequest can read or forward the token before transport and redirect checks",
    )

    require(
        re.search(r'FtpPassword\(\).*?ReadValue\(QStringLiteral\("FtpPassword"\),\s*QString\(\)\)', online, re.S),
        "online FTP password still has a non-empty source default",
    )
    require(
        re.search(r'FtpUser\(\).*?ReadValue\(QStringLiteral\("FtpUser"\),\s*QString\(\)\)', online, re.S),
        "retired shared FTP user still has a non-empty source default",
    )
    require('const std::string& ftpPwd = ' not in ftp, "FtpClient still has a password default argument")
    require("credential.password.clear();" in app, "robot FTP template still injects a password")
    require("QUuid::createUuid()" in scan_uploader and "remoteZipName" in scan_uploader,
            "device uploader does not use a unique remote filename per attempt")
    require(
        "TruncateToUtf8Bytes" in scan_uploader
        and "kMaximumRemoteComponentUtf8Bytes = 240" in scan_uploader
        and "remoteZipName.toUtf8().size() > kMaximumRemoteComponentUtf8Bytes" in scan_uploader,
        "unique remote upload names are not bounded by filesystem UTF-8 byte limits",
    )
    require(
        "QString::number(archiveBytes)" in scan_uploader
        and "IsCompleteRemoteArchive(fileName" in online_dialog
        and "embeddedBytes != listedBytes" in remote_archive
        and "openedBytes != listedBytes" in remote_archive,
        "write-once archives do not hide incomplete FTP EOF remnants by declared size",
    )
    require(
        re.search(r"uploadFileWithProgress\(.*?\},\s*false\s*\);", scan_uploader, re.S) is not None,
        "write-once uploader still requests remote delete capability",
    )
    upload_body_match = re.search(
        r"bool FtpClient::uploadFileWithProgress\(.*?\n\}\n\nbool FtpClient::listFiles",
        ftp_client,
        re.S,
    )
    require(upload_body_match is not None, "FtpClient progress-upload implementation is missing")
    upload_body = upload_body_match.group(0)
    for token in (
        "return allowRemoteDelete && FtpDeleteFileA",
        "ferror(fp)",
        "sent != total",
        "InternetCloseHandle(hRemote) != FALSE",
    ):
        require(token in upload_body, f"progress upload completion gate missing: {token}")
    require(
        re.search(
            r"if \(uploaded\).*?\+\+doneItems.*?if \(m_cancel\.load\(\)\).*?OnItemFinished\(caseDir, uploaded",
            scan_uploader,
            re.S,
        ) is not None,
        "late upload cancellation can retain a fully committed archive for duplicate retry",
    )
    require(
        scan_uploader.count("QCoreApplication::sendPostedEvents(this, QEvent::MetaCall)") >= 2,
        "joined upload worker callbacks are not synchronously persisted before shutdown",
    )

    require('src\\CredentialSecurity.cpp' in project, "CredentialSecurity.cpp is not in the product build")
    require('src\\ConfigDatabaseAuthentication.cpp' in project, "ConfigDatabaseAuthentication.cpp is not in the product build")
    require('include\\CredentialSecurity.h' in project, "CredentialSecurity.h is not in the product build")

    # Generic semantic scan: production headers/sources may name credential fields,
    # but must not initialize password/token/secret fields with non-empty literals.
    candidates = []
    literal_pattern = re.compile(
        r'(?i)(password|passwd|pwd|token|secret)\w*\s*(?:=|,)\s*'
        r'(?:QStringLiteral\()?"([^"\r\n]+)"'
    )
    for folder in (ROOT / "include", ROOT / "src"):
        for path in folder.rglob("*"):
            if path.suffix.lower() not in {".h", ".hpp", ".cpp", ".cc"}:
                continue
            for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
                match = literal_pattern.search(line)
                if match and match.group(2).strip():
                    candidates.append(f"{path.relative_to(ROOT)}:{line_number}")
    require(not candidates, "non-empty credential-like source defaults: " + ", ".join(candidates))

    print("PASS: SEC1 credential wiring and no-default-secret gate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
