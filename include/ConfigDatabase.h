#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <string>

class ConfigDatabase
{
public:
    enum class ReadStatus
    {
        Found,
        NotFound,
        Error
    };

    static QString DatabasePath();
    static QString NormalizeFilePath(const QString& fileName);
    static std::string NormalizeFilePath(const std::string& fileName);

    static bool IsAvailable();

    static bool HasIniFile(const std::string& fileName);
    static bool HasIniFile(const QString& fileName);
    static bool ReadIniValue(
        const std::string& fileName,
        const std::string& sectionName,
        const std::string& keyName,
        std::string* value);
    static ReadStatus ReadIniValueStatus(
        const std::string& fileName,
        const std::string& sectionName,
        const std::string& keyName,
        std::string* value);
    static bool WriteIniValue(
        const std::string& fileName,
        const std::string& sectionName,
        const std::string& keyName,
        const std::string& value);

    static bool HasTextFile(const std::string& fileName);
    static bool HasTextFile(const QString& fileName);
    static bool ReadTextFile(const std::string& fileName, std::string* content);
    static bool WriteTextFile(const std::string& fileName, const std::string& content);
    static bool CopyTextFile(const QString& sourceFileName, const QString& targetFileName, bool overwriteExisting = false);
    static bool RemoveConfigPathPrefix(const QString& sourcePathPrefix);

    static bool CopyIniFile(const QString& sourceFileName, const QString& targetFileName, bool overwriteExisting = false);
    static bool RemoveIniGroup(const QString& fileName, const QString& groupName);
    static QMap<QString, QString> ReadIniSection(const QString& fileName, const QString& sectionName);
    static bool RemoveIniSection(const QString& fileName, const QString& sectionName);

    static bool ReadScopedSetting(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName,
        QString* value);
    static ReadStatus ReadScopedSettingStatus(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName,
        QString* value);
    static QMap<QString, QString> ReadScopedSettings(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName);
    static bool WriteScopedSetting(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName,
        const QString& value,
        const QString& valueType = QStringLiteral("string"),
        bool sensitive = false);
    static bool WriteScopedSettings(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QMap<QString, QString>& values,
        const QString& valueType = QStringLiteral("string"));
    static bool TryReadAuthenticationInitialized(bool* initialized);
    static bool TryInitializeAuthenticationAccount(
        const QString& accountId,
        const QMap<QString, QString>& profileValues);
    static bool TryCreateAccount(
        const QString& accountId,
        const QMap<QString, QString>& profileValues,
        const QString& administratorId = QString());
    static bool TryReadAccountSecurityState(
        const QString& accountId,
        QString* passwordRecord,
        QString* role,
        bool* mustChangePassword,
        QString* fingerprint);
    static bool TryCompareAndSetAccountPassword(
        const QString& accountId,
        const QString& expectedPasswordRecord,
        const QString& newPasswordRecord,
        bool requireMustChangePassword,
        bool newMustChangePassword,
        QString* currentRole,
        QString* newFingerprint);
    static bool TryUpdateAccountByAdministrator(
        const QString& administratorId,
        const QString& accountId,
        const QString& newRole,
        const QString& newPasswordRecord = QString());
    static bool TryDeleteAccountByAdministrator(
        const QString& administratorId,
        const QString& accountId);
    static bool RemoveScopedSetting(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName);
    static QStringList ListScopedSettingIds(const QString& scopeType, const QString& moduleName = QString());
    static bool TryListScopedSettingIds(
        const QString& scopeType,
        const QString& moduleName,
        QStringList* ids);
    static bool TryListScopedSettingIdsBounded(
        const QString& scopeType,
        const QString& moduleName,
        qsizetype maxCount,
        qsizetype maxIdLength,
        QStringList* ids);
    static bool RemoveScopedSettings(const QString& scopeType, const QString& scopeId, const QString& moduleName = QString());
};
