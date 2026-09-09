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

    struct ScopedSettingValue
    {
        QString value;
        QString valueType = QStringLiteral("string");
        bool sensitive = false;
    };

    static QString DatabasePath();

    static bool IsAvailable();

    static bool HasScopedModule(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName);
    static bool CopyScopedModule(
        const QString& sourceScopeType,
        const QString& sourceScopeId,
        const QString& sourceModuleName,
        const QString& targetScopeType,
        const QString& targetScopeId,
        const QString& targetModuleName,
        bool overwriteExisting = false);
    static bool ReadScopedModuleSnapshot(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        QMap<QString, QMap<QString, QString>>& sectionValues,
        QString* error = nullptr);
    static bool RemoveScopedModuleSection(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& sectionName);
    static bool ReplaceScopedModuleSectionsAtomically(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QMap<QString, QMap<QString, QString>>& sectionValues,
        const QStringList& removeSections,
        QString* error = nullptr);

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
    // 原子比较并写入单个 scoped setting。expectedValue==nullptr 表示期望记录不存在；
    // SQLite BEGIN IMMEDIATE 覆盖读取、比较和写入，避免跨进程的丢失更新。
    static bool CompareAndSwapScopedSetting(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName,
        const QString* expectedValue,
        const QString& newValue,
        const QString& valueType = QStringLiteral("string"),
        bool sensitive = false,
        bool* conflict = nullptr,
        QString* error = nullptr);
    // 带 witness 的原子比较写入：同一 BEGIN IMMEDIATE 事务中先验证 witness
    // 的完整明文值，再验证 target 的期望值、写入 target、回读并提交。
    // expectedTargetValue==nullptr 表示期望 target 不存在。
    static bool CompareAndSwapScopedSettingWithWitness(
        const QString& witnessScopeType,
        const QString& witnessScopeId,
        const QString& witnessModuleName,
        const QString& witnessKeyName,
        const QString& expectedWitnessValue,
        const QString& targetScopeType,
        const QString& targetScopeId,
        const QString& targetModuleName,
        const QString& targetKeyName,
        const QString* expectedTargetValue,
        const QString& newTargetValue,
        const QString& targetValueType = QStringLiteral("string"),
        bool targetSensitive = false,
        bool* conflict = nullptr,
        QString* error = nullptr);
    static bool WriteScopedSettings(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QMap<QString, QString>& values,
        const QString& valueType = QStringLiteral("string"),
        QString* error = nullptr);
    static bool WriteScopedSettings(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QMap<QString, ScopedSettingValue>& values,
        QString* error = nullptr);
    static bool WriteLoginState(
        const QStringList& accountHistory,
        const QString& userName,
        bool rememberPassword,
        bool autoLogin);
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
        const QString& newPasswordRecord,
        bool forcePasswordChange);
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
