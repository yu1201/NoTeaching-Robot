#pragma once

#include <QString>

namespace CredentialSecurity
{
    enum class PasswordVerification
    {
        Invalid,
        Valid,
        ValidNeedsUpgrade
    };

    // PBKDF2-HMAC-SHA256 record with a per-password random salt.  The record is
    // self describing so iteration increases can be introduced without
    // invalidating existing accounts.
    QString CreatePasswordRecord(const QString& password, QString* error = nullptr);
    PasswordVerification VerifyPasswordRecord(
        const QString& userName,
        const QString& password,
        const QString& storedRecord);
    bool IsSupportedPasswordRecord(const QString& storedRecord);
    bool IsCurrentPasswordRecord(const QString& storedRecord);

    // Windows DPAPI, current-user scope.  The returned text is safe to place in
    // ConfigStore.db but is intentionally not portable to another Windows user.
    bool ProtectForCurrentUser(
        const QString& plainText,
        const QString& purpose,
        QString* protectedText,
        QString* error = nullptr);
    bool UnprotectForCurrentUser(
        const QString& protectedText,
        const QString& purpose,
        QString* plainText,
        QString* error = nullptr);
    bool IsCurrentUserProtected(const QString& protectedText);
}
