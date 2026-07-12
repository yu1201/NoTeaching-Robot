#include "OnlineServicesConfig.h"
#include "ScanDataUploader.h"

#include <QMap>
#include <QString>

#include <iostream>

namespace
{
QMap<QString, QString> g_values;

void SetIdentity(const QString& user, const QString& device)
{
    g_values.insert(QStringLiteral("FtpUser"), user);
    g_values.insert(QStringLiteral("DeviceName"), device);
}

bool ExpectRejected(const QString& user, const QString& device, const QString& reasonPart)
{
    SetIdentity(user, device);
    QString error;
    return !OnlineServicesConfig::HasDeviceBoundUploadIdentity(&error)
        && error.contains(reasonPart);
}
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
    if (!OnlineServicesConfig::FtpUser().isEmpty()
        || !OnlineServicesConfig::IsServerAccountName(OnlineServicesConfig::DeviceName()))
    {
        std::cerr << "default FTP identity is not empty-user/valid-device fail closed\n";
        return 1;
    }
    if (!ExpectRejected(QStringLiteral("uploader"), QStringLiteral("robot-01"), QStringLiteral("永久退役")))
    {
        std::cerr << "shared uploader identity was not rejected\n";
        return 2;
    }
    if (!ExpectRejected(QStringLiteral("robot-02"), QStringLiteral("robot-01"), QStringLiteral("完全一致")))
    {
        std::cerr << "cross-device account/device mismatch was not rejected\n";
        return 3;
    }
    if (!ExpectRejected(QStringLiteral("robot-01"), QStringLiteral("Robot-01"), QStringLiteral("必须匹配")))
    {
        std::cerr << "case-only account/device mismatch was not rejected\n";
        return 4;
    }

    if (!ExpectRejected(QStringLiteral("devicedata"), QStringLiteral("devicedata"), QStringLiteral("全权限")))
    {
        std::cerr << "full admin identity was accepted for automatic upload\n";
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

    SetIdentity(QStringLiteral("robot-01"), QStringLiteral("robot-01"));
    QString error = QStringLiteral("stale");
    if (!OnlineServicesConfig::HasDeviceBoundUploadIdentity(&error) || !error.isEmpty())
    {
        std::cerr << "matching per-device identity was rejected\n";
        return 6;
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

    std::cout << "PASS: device identity and stale temporary archive policies fail closed\n";
    return 0;
}
