#include "OnlineServicesConfig.h"
#include "ScanDataUploader.h"

#include <QMap>
#include <QString>

#include <iostream>

namespace
{
QMap<QString, QString> g_values;

}

bool ConfigDatabase::ReadScopedSetting(
    const QString&,
    const QString&,
    const QString&,
    const QString& keyName,
    QString* value)
{
    const auto it = g_values.constFind(keyName);
    if (it == g_values.cend() || value == nullptr)
    {
        return false;
    }
    *value = *it;
    return true;
}

bool ConfigDatabase::WriteScopedSetting(
    const QString&,
    const QString&,
    const QString&,
    const QString& keyName,
    const QString& value,
    const QString&,
    bool)
{
    g_values.insert(keyName, value);
    return true;
}

int main()
{
    g_values.clear();
    if (OnlineServicesConfig::FtpUser() != OnlineServicesConfig::UploadOnlyAccount()
        || OnlineServicesConfig::ServerHost().isEmpty()
        || OnlineServicesConfig::DeviceName().isEmpty())
    {
        std::cerr << "default online-services settings are incomplete\n";
        return 1;
    }
    if (!OnlineServicesConfig::IsDefaultFtpAccount(OnlineServicesConfig::FullAccessAccount())
        || !OnlineServicesConfig::IsDefaultFtpAccount(OnlineServicesConfig::FtpAccessAccount())
        || !OnlineServicesConfig::IsDefaultFtpAccount(OnlineServicesConfig::UploadOnlyAccount())
        || OnlineServicesConfig::IsDefaultFtpAccount(QStringLiteral("robot-01")))
    {
        std::cerr << "fixed account allow-list is incorrect\n";
        return 2;
    }
    if (OnlineServicesConfig::AccessLevelForAccount(OnlineServicesConfig::FullAccessAccount())
            != OnlineServicesConfig::AccessLevel::Full
        || OnlineServicesConfig::AccessLevelForAccount(OnlineServicesConfig::FtpAccessAccount())
            != OnlineServicesConfig::AccessLevel::Ftp
        || OnlineServicesConfig::AccessLevelForAccount(OnlineServicesConfig::UploadOnlyAccount())
            != OnlineServicesConfig::AccessLevel::Upload)
    {
        std::cerr << "account access-level mapping is incorrect\n";
        return 3;
    }
    if (!OnlineServicesConfig::HasFtpAccess(OnlineServicesConfig::AccessLevel::Ftp)
        || !OnlineServicesConfig::HasFtpAccess(OnlineServicesConfig::AccessLevel::Full)
        || OnlineServicesConfig::HasFtpAccess(OnlineServicesConfig::AccessLevel::Upload)
        || !OnlineServicesConfig::HasFullAccess(OnlineServicesConfig::AccessLevel::Full))
    {
        std::cerr << "access-level ordering is incorrect\n";
        return 4;
    }
    for (const QString& invalid : {
        QStringLiteral("ab"), QStringLiteral("1robot"), QStringLiteral("Robot-01"),
        QStringLiteral("robot.dot"), QString(33, QLatin1Char('a')) })
    {
        if (OnlineServicesConfig::IsServerAccountName(invalid))
        {
            std::cerr << "server account regex accepted invalid name\n";
            return 5;
        }
    }

    const QString ownedName = QStringLiteral("upload_0123456789abcdef0123456789abcdef.zip");
    const QString ownedListName = ownedName + QStringLiteral(".files");
    const qint64 now = 2LL * ScanDataUploadPolicy::TempArchiveTtlMs;
    if (!ScanDataUploadPolicy::IsOwnedTempArchiveName(ownedName)
        || ScanDataUploadPolicy::IsOwnedTempArchiveName(QStringLiteral("other_0123456789abcdef0123456789abcdef.zip"))
        || ScanDataUploadPolicy::IsOwnedTempArchiveName(QStringLiteral("upload_0123456789ABCDEF0123456789ABCDEF.zip"))
        || ScanDataUploadPolicy::IsOwnedTempArchiveName(QStringLiteral("upload_0123456789abcdef.zip")))
    {
        std::cerr << "temporary archive ownership pattern is not exact\n";
        return 7;
    }
    if (!ScanDataUploadPolicy::IsOwnedTempArchiveListName(ownedListName)
        || ScanDataUploadPolicy::IsOwnedTempArchiveListName(ownedName)
        || ScanDataUploadPolicy::IsOwnedTempArchiveListName(
            QStringLiteral("upload_0123456789abcdef0123456789abcdef.files")))
    {
        std::cerr << "temporary archive-list ownership pattern is not exact\n";
        return 8;
    }
    if (ScanDataUploadPolicy::ShouldDeleteTempArchive(
            ownedName, true, false, now - ScanDataUploadPolicy::TempArchiveTtlMs + 1, now)
        || !ScanDataUploadPolicy::ShouldDeleteTempArchive(
            ownedName, true, false, now - ScanDataUploadPolicy::TempArchiveTtlMs, now)
        || ScanDataUploadPolicy::ShouldDeleteTempArchive(
            ownedName, false, false, now - ScanDataUploadPolicy::TempArchiveTtlMs, now)
        || ScanDataUploadPolicy::ShouldDeleteTempArchive(
            ownedName, true, true, now - ScanDataUploadPolicy::TempArchiveTtlMs, now)
        || ScanDataUploadPolicy::ShouldDeleteTempArchive(
            ownedName, true, false, now + 1, now))
    {
        std::cerr << "temporary archive TTL/link/regular-file policy is not fail closed\n";
        return 9;
    }
    if (!ScanDataUploadPolicy::ShouldDeleteTempArchiveList(
            ownedListName, true, false, now - ScanDataUploadPolicy::TempArchiveTtlMs, now)
        || ScanDataUploadPolicy::ShouldDeleteTempArchiveList(
            ownedListName, true, true, now - ScanDataUploadPolicy::TempArchiveTtlMs, now))
    {
        std::cerr << "temporary archive-list TTL/link policy is not fail closed\n";
        return 10;
    }

    std::cout << "PASS: fixed online-service roles and stale temporary archive policies are valid\n";
    return 0;
}
