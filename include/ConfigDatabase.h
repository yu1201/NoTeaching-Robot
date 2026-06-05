#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <string>

class ConfigDatabase
{
public:
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

    static QStringList ListIniGroups(const QString& fileName, const QString& parentGroup = QString());
    static QStringList ListIniSections(const QString& fileName);
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
    static bool WriteScopedSetting(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName,
        const QString& value,
        const QString& valueType = QStringLiteral("string"),
        bool sensitive = false);
    static bool RemoveScopedSetting(
        const QString& scopeType,
        const QString& scopeId,
        const QString& moduleName,
        const QString& keyName);
    static QStringList ListScopedSettingIds(const QString& scopeType, const QString& moduleName = QString());
    static bool RemoveScopedSettings(const QString& scopeType, const QString& scopeId, const QString& moduleName = QString());
};
