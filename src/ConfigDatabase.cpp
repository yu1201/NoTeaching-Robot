#include "ConfigDatabase.h"

#include "AppPaths.h"
#include "CredentialSecurity.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QRandomGenerator>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QThread>
#include <algorithm>
#include <cstring>
#include <vector>

namespace
{
constexpr char kSchemaVersion[] = "5";
constexpr char kAuthenticationSemanticVersion[] = "2";
constexpr char kSecret[] = "NoTeachingRobotConfigStoreV1";

struct ScopedSettingIdentity
{
    bool valid = false;
    QString scopeType;
    QString scopeId;
    QString module;
    QString keyName;
    QString valueType = QStringLiteral("string");
    bool sensitive = false;
};

QString NormalizeSection(const QString& sectionName);
QString NormalizeSourceKey(const QString& keyName);
ScopedSettingIdentity BuildScopedFileIdentity(const QString& fileName, const QString& keyName = QString());
ScopedSettingIdentity BuildScopedIniIdentity(const QString& fileName, const QString& sectionName, const QString& keyName);
bool WriteScopedSettingValue(QSqlDatabase& db, const ScopedSettingIdentity& identity, const QString& value);
bool EnsureCurrentSchema(QSqlDatabase& db);
bool HasLegacyDiskConfigurationInputs(const QString& databasePath);
bool HasUnsafePlaintextConfigStoreResidue(const QString& databasePath);

QString ConnectionName()
{
    const quintptr threadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
    return QString("ConfigStore_%1").arg(threadId);
}

QString FindProjectRoot()
{
    return AppPaths::DataRootPath();
}

QSqlDatabase OpenDatabase()
{
    const QString connectionName = ConnectionName();
    if (QSqlDatabase::contains(connectionName))
    {
        QSqlDatabase db = QSqlDatabase::database(connectionName, false);
        if (db.isOpen())
        {
            return db;
        }
        if (!QFileInfo::exists(db.databaseName())
            && (HasLegacyDiskConfigurationInputs(db.databaseName())
                || HasUnsafePlaintextConfigStoreResidue(db.databaseName())))
        {
            qCritical() << "Legacy INI/TXT configuration exists without ConfigStore; "
                           "run ConfigMigrate before starting the application.";
            return QSqlDatabase();
        }
        if (db.open())
        {
            QSqlQuery pragma(db);
            pragma.exec("PRAGMA busy_timeout=3000");
            pragma.exec("PRAGMA secure_delete=ON");
            if (EnsureCurrentSchema(db))
            {
                return db;
            }
            db.close();
        }
        return QSqlDatabase();
    }

    const QString dbPath = ConfigDatabase::DatabasePath();
    const QFileInfo dbInfo(dbPath);
    QDir dbDir = dbInfo.dir();
    if (!dbDir.exists() && !dbDir.mkpath(QStringLiteral(".")))
    {
        return QSqlDatabase();
    }
    if (!dbInfo.exists()
        && (HasLegacyDiskConfigurationInputs(dbPath)
            || HasUnsafePlaintextConfigStoreResidue(dbPath)))
    {
        qCritical() << "Legacy INI/TXT configuration exists without ConfigStore; "
                       "run ConfigMigrate before starting the application.";
        return QSqlDatabase();
    }
    if (dbInfo.exists())
    {
        QFile dbFile(dbPath);
        const QFileDevice::Permissions permissions = dbFile.permissions();
        if ((permissions & QFileDevice::WriteOwner) == 0 || (permissions & QFileDevice::WriteUser) == 0)
        {
            dbFile.setPermissions(permissions | QFileDevice::WriteOwner | QFileDevice::WriteUser);
        }
    }

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", connectionName);
    db.setDatabaseName(dbPath);
    if (!db.open())
    {
        return QSqlDatabase();
    }

    QSqlQuery pragma(db);
    pragma.exec("PRAGMA busy_timeout=3000");
    pragma.exec("PRAGMA secure_delete=ON");

    if (!EnsureCurrentSchema(db))
    {
        db.close();
        return QSqlDatabase();
    }

    return db;
}

QString DecodeMaybeLocal(const std::string& text)
{
    QString value = QString::fromUtf8(text.c_str(), static_cast<int>(text.size()));
    if (value.contains(QChar(0xfffd)))
    {
        value = QString::fromLocal8Bit(text.c_str(), static_cast<int>(text.size()));
    }
    return value;
}

QString FromUtf8StdString(const std::string& text)
{
    return QString::fromUtf8(text.data(), static_cast<int>(text.size()));
}

QByteArray MakeKeyStream(const QByteArray& nonce, int byteCount)
{
    QByteArray stream;
    int counter = 0;
    while (stream.size() < byteCount)
    {
        QByteArray block;
        block.append(kSecret, static_cast<int>(strlen(kSecret)));
        block.append(nonce);
        block.append(':');
        block.append(QByteArray::number(counter++));
        stream.append(QCryptographicHash::hash(block, QCryptographicHash::Sha256));
    }
    stream.truncate(byteCount);
    return stream;
}

QString ProtectLegacyText(const QString& plainText)
{
    QByteArray nonce;
    nonce.resize(16);
    for (int i = 0; i < nonce.size(); ++i)
    {
        nonce[i] = static_cast<char>(QRandomGenerator::global()->bounded(256));
    }

    QByteArray bytes = plainText.toUtf8();
    const QByteArray stream = MakeKeyStream(nonce, bytes.size());
    for (int i = 0; i < bytes.size(); ++i)
    {
        bytes[i] = static_cast<char>(static_cast<unsigned char>(bytes[i]) ^ static_cast<unsigned char>(stream[i]));
    }

    return QString("enc:v1:%1:%2")
        .arg(QString::fromLatin1(nonce.toBase64(QByteArray::OmitTrailingEquals)))
        .arg(QString::fromLatin1(bytes.toBase64(QByteArray::OmitTrailingEquals)));
}

bool UnprotectLegacyText(const QString& storedText, QString* plainText)
{
    if (plainText == nullptr)
    {
        return false;
    }
    const QStringList parts = storedText.split(':');
    if (parts.size() != 4 || parts.at(0) != "enc" || parts.at(1) != "v1")
    {
        return false;
    }

    auto paddedBase64 = [](QByteArray text)
        {
            const int pad = text.size() % 4;
            if (pad != 0)
            {
                text.append(QByteArray(4 - pad, '='));
            }
            return text;
        };

    constexpr auto base64Options = QByteArray::Base64Encoding | QByteArray::IgnoreBase64DecodingErrors;
    const QByteArray nonce = QByteArray::fromBase64(paddedBase64(parts.at(2).toLatin1()), base64Options);
    QByteArray bytes = QByteArray::fromBase64(paddedBase64(parts.at(3).toLatin1()), base64Options);
    if (nonce.size() != 16)
    {
        return false;
    }

    const QByteArray stream = MakeKeyStream(nonce, bytes.size());
    for (int i = 0; i < bytes.size(); ++i)
    {
        bytes[i] = static_cast<char>(static_cast<unsigned char>(bytes[i]) ^ static_cast<unsigned char>(stream[i]));
    }
    *plainText = QString::fromUtf8(bytes);
    return true;
}

bool ShouldEncryptNewValues(QSqlDatabase& db)
{
    QSqlQuery query(db);
    query.prepare("SELECT value FROM meta WHERE key='encrypt_new_values'");
    if (!query.exec() || !query.next())
    {
        return false;
    }
    const QString value = query.value(0).toString().trimmed().toLower();
    return value == "1" || value == "true" || value == "yes";
}

bool DecodeStoredText(
    const QString& storedText,
    int encrypted,
    const QString& protectionPurpose,
    QString* plainText)
{
    if (plainText == nullptr)
    {
        return false;
    }
    if (CredentialSecurity::IsCurrentUserProtected(storedText))
    {
        return CredentialSecurity::UnprotectForCurrentUser(storedText, protectionPurpose, plainText);
    }
    if (storedText.startsWith("enc:v1:"))
    {
        return UnprotectLegacyText(storedText, plainText);
    }
    if (encrypted != 0)
    {
        return false;
    }
    *plainText = storedText;
    return true;
}

QString StoredTextForWrite(QSqlDatabase& db, const QString& plainText, int* encrypted)
{
    if (encrypted != nullptr)
    {
        *encrypted = 0;
    }
    if (!ShouldEncryptNewValues(db))
    {
        return plainText;
    }
    if (encrypted != nullptr)
    {
        *encrypted = 1;
    }
    return ProtectLegacyText(plainText);
}

bool EncodeStoredText(
    QSqlDatabase& db,
    const QString& plainText,
    bool sensitive,
    const QString& protectionPurpose,
    QString* storedText,
    int* encrypted)
{
    if (storedText == nullptr || encrypted == nullptr)
    {
        return false;
    }
    if (sensitive)
    {
        if (!CredentialSecurity::ProtectForCurrentUser(plainText, protectionPurpose, storedText))
        {
            return false;
        }
        *encrypted = 1;
        return true;
    }
    *storedText = StoredTextForWrite(db, plainText, encrypted);
    return true;
}

QString NormalizeSection(const QString& sectionName)
{
    QString cleaned = sectionName.trimmed();
    cleaned.replace('\\', '/');
    while (cleaned.startsWith('/'))
    {
        cleaned.remove(0, 1);
    }
    while (cleaned.endsWith('/'))
    {
        cleaned.chop(1);
    }
    return cleaned;
}

QString NormalizeSourceKey(const QString& keyName)
{
    QString cleaned = keyName.trimmed();
    cleaned.replace('\\', '/');
    while (cleaned.startsWith('/'))
    {
        cleaned.remove(0, 1);
    }
    while (cleaned.endsWith('/'))
    {
        cleaned.chop(1);
    }
    return cleaned;
}

QString NormalizeScopeId(const QString& scopeId)
{
    const QString cleaned = scopeId.trimmed();
    return cleaned.isNull() ? QString::fromLatin1("") : cleaned;
}

QString ProtectionPurpose(
    const QString& scopeType,
    const QString& scopeId,
    const QString& module,
    const QString& keyName)
{
    return QStringLiteral("%1\n%2\n%3\n%4")
        .arg(
            NormalizeSection(scopeType).toLower(),
            NormalizeScopeId(scopeId),
            NormalizeSection(module),
            NormalizeSourceKey(keyName));
}

bool IsSensitiveSettingKey(const QString& keyName)
{
    const QString lower = keyName.toLower();
    return lower.contains(QStringLiteral("password"))
        || lower.contains(QStringLiteral("passwd"))
        || lower.contains(QStringLiteral("pass"))
        || lower.contains(QStringLiteral("token"))
        || lower.contains(QStringLiteral("secret"))
        || lower.contains(QStringLiteral("credential"))
        || lower.contains(QStringLiteral("api_key"))
        || lower.contains(QStringLiteral("apikey"));
}

bool IsPortableAuthenticationValue(
    const QString& scopeType,
    const QString& module,
    const QString& keyName)
{
    if (NormalizeSection(scopeType).compare(QStringLiteral("account"), Qt::CaseInsensitive) != 0
        || NormalizeSection(module).compare(QStringLiteral("Profile"), Qt::CaseInsensitive) != 0)
    {
        return false;
    }
    return keyName.compare(QStringLiteral("PasswordHash"), Qt::CaseInsensitive) == 0
        || keyName.compare(QStringLiteral("Role"), Qt::CaseInsensitive) == 0
        || keyName.compare(QStringLiteral("MustChangePassword"), Qt::CaseInsensitive) == 0
        || keyName.compare(QStringLiteral("PasswordChangedAt"), Qt::CaseInsensitive) == 0;
}

bool RequiresDpapiProtection(
    const QString& scopeType,
    const QString& module,
    const QString& keyName,
    bool sensitive)
{
    if (!sensitive)
    {
        return false;
    }
    if (IsPortableAuthenticationValue(scopeType, module, keyName))
    {
        // These are a one-way verifier and non-secret account metadata.  Keep
        // them portable across Windows users; only recoverable credentials use
        // DPAPI CurrentUser.
        return false;
    }
    return true;
}

QString ModuleBaseFromStoredFileName(const QString& storedFileName)
{
    const QString baseName = QFileInfo(storedFileName).completeBaseName().trimmed();
    if (baseName.compare(QStringLiteral("ContralUnitInfo"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("ControlUnits");
    }
    if (baseName.compare(QStringLiteral("RobotPara"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("RobotPara");
    }
    if (baseName.compare(QStringLiteral("CameraParam"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("CameraParam");
    }
    if (baseName.startsWith(QStringLiteral("HandEyeMatrix_"), Qt::CaseInsensitive))
    {
        return QStringLiteral("HandEyeMatrix/%1").arg(baseName.mid(QStringLiteral("HandEyeMatrix_").size()));
    }
    if (baseName.compare(QStringLiteral("HandEyeMatrix"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("HandEyeMatrix");
    }
    if (baseName.startsWith(QStringLiteral("HandEyeCalibration_"), Qt::CaseInsensitive))
    {
        return QStringLiteral("HandEyeCalibration/%1").arg(baseName.mid(QStringLiteral("HandEyeCalibration_").size()));
    }
    if (baseName.compare(QStringLiteral("LineScanParam"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("LineScanParam");
    }
    if (baseName.compare(QStringLiteral("LineCoarseScanParam"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("LineCoarseScanParam");
    }
    if (baseName.compare(QStringLiteral("MeasureWeldParam"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("MeasureWeldParam");
    }
    if (baseName.compare(QStringLiteral("WeldPoseCompParam"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("WeldPoseCompParam");
    }
    if (baseName.compare(QStringLiteral("WeldSeamCompParam"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("WeldSeamCompParam");
    }
    if (baseName.compare(QStringLiteral("WeaveDate"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("WeldProcess/WeaveData");
    }
    if (baseName.compare(QStringLiteral("WeldPara"), Qt::CaseInsensitive) == 0)
    {
        return QStringLiteral("WeldProcess/WeldParameters");
    }
    return NormalizeSection(baseName.isEmpty() ? QFileInfo(storedFileName).fileName() : baseName);
}

ScopedSettingIdentity BuildScopedFileIdentity(const QString& fileName, const QString& keyName)
{
    ScopedSettingIdentity identity;
    const QString normalizedPath = ConfigDatabase::NormalizeFilePath(fileName);
    const QStringList parts = normalizedPath.split('/', Qt::SkipEmptyParts);

    identity.scopeType = QStringLiteral("global");
    identity.scopeId.clear();

    QString storedFileName = QFileInfo(normalizedPath).fileName();
    if (!parts.isEmpty() && parts.first().compare(QStringLiteral("Data"), Qt::CaseInsensitive) == 0)
    {
        if (parts.size() >= 4 && parts.at(1).compare(QStringLiteral("WorkpieceTemplates"), Qt::CaseInsensitive) == 0)
        {
            identity.scopeType = QStringLiteral("workpiece_template");
            identity.scopeId = parts.at(2).trimmed();
            storedFileName = parts.mid(3).join('/');
        }
        else if (parts.size() >= 3)
        {
            identity.scopeType = QStringLiteral("robot");
            identity.scopeId = parts.at(1).trimmed();
            storedFileName = parts.mid(2).join('/');
        }
        else if (parts.size() >= 2)
        {
            storedFileName = parts.mid(1).join('/');
        }
    }
    else if (!parts.isEmpty() && parts.first().compare(QStringLiteral("Result"), Qt::CaseInsensitive) == 0)
    {
        identity.scopeType = QStringLiteral("result");
        if (parts.size() >= 3)
        {
            identity.scopeId = parts.at(1).trimmed();
            storedFileName = parts.mid(2).join('/');
        }
    }

    identity.module = NormalizeSection(ModuleBaseFromStoredFileName(storedFileName));
    identity.keyName = NormalizeSourceKey(keyName.isEmpty() ? QStringLiteral("Content") : keyName);
    identity.valueType = QStringLiteral("text");
    identity.sensitive = IsSensitiveSettingKey(identity.keyName);
    identity.valid = !identity.scopeType.trimmed().isEmpty()
        && !identity.module.trimmed().isEmpty()
        && !identity.keyName.trimmed().isEmpty();
    return identity;
}

ScopedSettingIdentity BuildScopedIniIdentity(const QString& fileName, const QString& sectionName, const QString& keyName)
{
    ScopedSettingIdentity identity = BuildScopedFileIdentity(fileName, keyName);
    const QString section = NormalizeSection(sectionName);
    if (!section.isEmpty())
    {
        identity.module = NormalizeSection(identity.module + QStringLiteral("/") + section);
    }
    identity.valueType = QStringLiteral("string");
    identity.sensitive = identity.sensitive || IsSensitiveSettingKey(section) || IsSensitiveSettingKey(keyName);
    identity.valid = identity.valid && !identity.module.isEmpty();
    return identity;
}

ConfigDatabase::ReadStatus ReadScopedSettingValueStatus(
    QSqlDatabase& db,
    const ScopedSettingIdentity& identity,
    QString* value)
{
    if (!identity.valid || value == nullptr)
    {
        return ConfigDatabase::ReadStatus::Error;
    }
    value->clear();

    QSqlQuery query(db);
    query.prepare("SELECT value_text, encrypted, sensitive FROM settings WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(NormalizeSection(identity.module));
    query.addBindValue(NormalizeSourceKey(identity.keyName));
    if (!query.exec())
    {
        return ConfigDatabase::ReadStatus::Error;
    }
    const bool hasRow = query.next();
#if defined(CONFIG_DATABASE_TEST_INJECT_CURSOR_ERROR)
    if (!hasRow
        && qEnvironmentVariableIntValue("QTWIDGETSAPP4_TEST_CONFIG_CURSOR_ERROR") == 1)
    {
        return ConfigDatabase::ReadStatus::Error;
    }
#endif
    if (!hasRow)
    {
        return query.lastError().isValid()
            ? ConfigDatabase::ReadStatus::Error
            : ConfigDatabase::ReadStatus::NotFound;
    }
    const QString storedText = query.value(0).toString();
    const int encrypted = query.value(1).toInt();
    const bool sensitive = identity.sensitive || query.value(2).toInt() != 0
        || IsSensitiveSettingKey(identity.module) || IsSensitiveSettingKey(identity.keyName);
    const QString purpose = ProtectionPurpose(
        identity.scopeType, identity.scopeId, identity.module, identity.keyName);
    if (!DecodeStoredText(storedText, encrypted, purpose, value))
    {
        return ConfigDatabase::ReadStatus::Error;
    }
    query.finish();
    const bool requiresDpapi = RequiresDpapiProtection(
        identity.scopeType, identity.module, identity.keyName, sensitive);
    if (requiresDpapi && !CredentialSecurity::IsCurrentUserProtected(storedText))
    {
        ScopedSettingIdentity upgraded = identity;
        upgraded.sensitive = true;
        return WriteScopedSettingValue(db, upgraded, *value)
            ? ConfigDatabase::ReadStatus::Found
            : ConfigDatabase::ReadStatus::Error;
    }
    return ConfigDatabase::ReadStatus::Found;
}

bool ReadScopedSettingValue(QSqlDatabase& db, const ScopedSettingIdentity& identity, QString* value)
{
    return ReadScopedSettingValueStatus(db, identity, value)
        == ConfigDatabase::ReadStatus::Found;
}

bool WriteScopedSettingValue(QSqlDatabase& db, const ScopedSettingIdentity& identity, const QString& value)
{
    if (!identity.valid)
    {
        return false;
    }

    const bool sensitive = identity.sensitive
        || IsSensitiveSettingKey(identity.module)
        || IsSensitiveSettingKey(identity.keyName);
    int encrypted = 0;
    QString storedText;
    const QString purpose = ProtectionPurpose(
        identity.scopeType, identity.scopeId, identity.module, identity.keyName);
    const bool requiresDpapi = RequiresDpapiProtection(
        identity.scopeType, identity.module, identity.keyName, sensitive);
    if (IsPortableAuthenticationValue(
            identity.scopeType, identity.module, identity.keyName))
    {
        storedText = value;
    }
    else if (!EncodeStoredText(db, value, requiresDpapi, purpose, &storedText, &encrypted))
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare(
        "INSERT OR REPLACE INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(NormalizeSection(identity.module));
    query.addBindValue(NormalizeSourceKey(identity.keyName));
    query.addBindValue(storedText);
    query.addBindValue(identity.valueType.trimmed().isEmpty() ? QStringLiteral("string") : identity.valueType.trimmed().toLower());
    query.addBindValue(sensitive ? 1 : 0);
    query.addBindValue(encrypted);
    return query.exec();
}

QStringList TableColumns(QSqlDatabase& db, const QString& tableName)
{
    QStringList columns;
    QSqlQuery query(db);
    query.prepare(QStringLiteral("PRAGMA table_info(%1)").arg(tableName));
    if (!query.exec())
    {
        return columns;
    }
    while (query.next())
    {
        columns << query.value(1).toString();
    }
    return columns;
}


bool ExecSql(QSqlDatabase& db, const QString& sql)
{
    QSqlQuery query(db);
    return query.exec(sql);
}

bool CreateCurrentTables(QSqlDatabase& db)
{
    return ExecSql(db, QStringLiteral(
        "CREATE TABLE IF NOT EXISTS meta ("
        "key TEXT PRIMARY KEY,"
        "value TEXT NOT NULL)"))
        && ExecSql(db, QStringLiteral(
            "CREATE TABLE IF NOT EXISTS settings ("
            "scope_type TEXT NOT NULL,"
            "scope_id TEXT NOT NULL DEFAULT '',"
            "module TEXT NOT NULL,"
            "key_name TEXT NOT NULL,"
            "value_text TEXT NOT NULL,"
            "value_type TEXT NOT NULL DEFAULT 'string',"
            "sensitive INTEGER NOT NULL DEFAULT 0,"
            "encrypted INTEGER NOT NULL DEFAULT 0,"
            "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,"
            "PRIMARY KEY(scope_type, scope_id, module, key_name))"))
        && ExecSql(db, QStringLiteral(
            "CREATE INDEX IF NOT EXISTS idx_settings_scope "
            "ON settings(scope_type, scope_id, module)"));
}

bool TableExists(QSqlDatabase& db, const QString& tableName)
{
    QSqlQuery query(db);
    query.prepare("SELECT 1 FROM sqlite_master WHERE type='table' AND name=? LIMIT 1");
    query.addBindValue(tableName);
    return query.exec() && query.next();
}

QString MetaValue(QSqlDatabase& db, const QString& key)
{
    if (!TableExists(db, QStringLiteral("meta")))
    {
        return QString();
    }
    QSqlQuery query(db);
    query.prepare("SELECT value FROM meta WHERE key=?");
    query.addBindValue(key);
    return query.exec() && query.next() ? query.value(0).toString().trimmed() : QString();
}

bool SetMetaValue(QSqlDatabase& db, const QString& key, const QString& value)
{
    QSqlQuery query(db);
    query.prepare("INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)");
    query.addBindValue(key);
    query.addBindValue(value);
    return query.exec();
}

bool HasCurrentSettingsColumns(QSqlDatabase& db)
{
    const QStringList columns = TableColumns(db, QStringLiteral("settings"));
    const QStringList required = {
        QStringLiteral("scope_type"),
        QStringLiteral("scope_id"),
        QStringLiteral("module"),
        QStringLiteral("key_name"),
        QStringLiteral("value_text"),
        QStringLiteral("value_type"),
        QStringLiteral("sensitive"),
        QStringLiteral("encrypted"),
        QStringLiteral("updated_at")
    };
    return std::all_of(required.cbegin(), required.cend(), [&columns](const QString& column)
        {
            return columns.contains(column);
        });
}

bool IsSafeMigratedAccountName(const QString& userName)
{
    if (userName.size() < 3 || userName.size() > 32)
    {
        return false;
    }
    return std::all_of(userName.cbegin(), userName.cend(), [](const QChar& ch)
        {
            return ch.isLetterOrNumber() || ch == '_' || ch == '-' || ch == '.';
        });
}

bool ReadRawScopedValue(
    QSqlDatabase& db,
    const QString& scopeType,
    const QString& scopeId,
    const QString& module,
    const QString& keyName,
    QString* value)
{
    QSqlQuery query(db);
    query.prepare(
        "SELECT value_text, encrypted FROM settings "
        "WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(NormalizeSection(module));
    query.addBindValue(NormalizeSourceKey(keyName));
    if (!query.exec() || !query.next())
    {
        return false;
    }
    return DecodeStoredText(
        query.value(0).toString(),
        query.value(1).toInt(),
        ProtectionPurpose(scopeType, scopeId, module, keyName),
        value);
}

struct LegacyAccount
{
    QString userName;
    QMap<QString, QString> values;
};

struct SensitiveUpgrade
{
    QString scopeType;
    QString scopeId;
    QString module;
    QString keyName;
    QString value;
    QString valueType;
};

bool MigrateLegacyAccounts(QSqlDatabase& db)
{
    QMap<QString, LegacyAccount> legacyAccounts;

    QSqlQuery accountQuery(db);
    accountQuery.prepare(
        "SELECT module, key_name, value_text, encrypted FROM settings "
        "WHERE scope_type='global' AND scope_id='' AND module LIKE 'Accounts/Users/%'");
    if (!accountQuery.exec())
    {
        return false;
    }
    while (accountQuery.next())
    {
        const QString module = accountQuery.value(0).toString();
        const QString userName = module.mid(QStringLiteral("Accounts/Users/").size()).trimmed();
        const QString keyName = accountQuery.value(1).toString();
        if (!IsSafeMigratedAccountName(userName))
        {
            return false;
        }
        QString decoded;
        if (!DecodeStoredText(
                accountQuery.value(2).toString(),
                accountQuery.value(3).toInt(),
                ProtectionPurpose(QStringLiteral("global"), QString(), module, keyName),
                &decoded))
        {
            return false;
        }
        const QString foldedUserName = userName.toCaseFolded();
        if (legacyAccounts.contains(foldedUserName)
            && legacyAccounts.value(foldedUserName).userName != userName)
        {
            // The historical UI treated account ids case-sensitively.  Merging
            // Foo and foo could combine one role with another password.
            return false;
        }
        LegacyAccount& account = legacyAccounts[foldedUserName];
        account.userName = userName;
        account.values.insert(keyName, decoded);
    }
    accountQuery.finish();

    for (auto it = legacyAccounts.cbegin(); it != legacyAccounts.cend(); ++it)
    {
        const LegacyAccount& source = it.value();
        const QString sourceHash = source.values.value(QStringLiteral("PasswordHash"));
        const QString sourceRole = source.values.value(QStringLiteral("Role"));
        if (sourceHash.isEmpty()
            || (sourceRole != QStringLiteral("operator")
                && sourceRole != QStringLiteral("engineer")
                && sourceRole != QStringLiteral("admin")))
        {
            return false;
        }

        QString destinationHash;
        const bool destinationExists = ReadRawScopedValue(
            db,
            QStringLiteral("account"),
            source.userName,
            QStringLiteral("Profile"),
            QStringLiteral("PasswordHash"),
            &destinationHash);
        bool replaceDestination = !destinationExists;
        if (destinationExists
            && source.userName.compare(QStringLiteral("admin"), Qt::CaseInsensitive) == 0
            && CredentialSecurity::VerifyPasswordRecord(
                source.userName, QStringLiteral("admin"), destinationHash)
                != CredentialSecurity::PasswordVerification::Invalid)
        {
            // A v4 start may already have inserted the known bootstrap account
            // after missing these legacy rows.  Restore the legacy account as
            // the authoritative value in that one narrow case.
            replaceDestination = true;
        }

        if (replaceDestination)
        {
            QSqlQuery clearProfile(db);
            clearProfile.prepare(
                "DELETE FROM settings WHERE scope_type='account' AND scope_id=? AND module='Profile'");
            clearProfile.addBindValue(source.userName);
            if (!clearProfile.exec())
            {
                return false;
            }
            for (const QString& keyName : {
                    QStringLiteral("PasswordHash"),
                    QStringLiteral("Role"),
                    QStringLiteral("CreatedAt"),
                    QStringLiteral("UpdatedAt") })
            {
                if (!source.values.contains(keyName))
                {
                    continue;
                }
                ScopedSettingIdentity identity;
                identity.valid = true;
                identity.scopeType = QStringLiteral("account");
                identity.scopeId = source.userName;
                identity.module = QStringLiteral("Profile");
                identity.keyName = keyName;
                identity.valueType = keyName.endsWith(QStringLiteral("At"))
                    ? QStringLiteral("datetime")
                    : QStringLiteral("string");
                identity.sensitive = keyName == QStringLiteral("PasswordHash");
                if (!WriteScopedSettingValue(db, identity, source.values.value(keyName)))
                {
                    return false;
                }
            }
            destinationHash = sourceHash;

            const bool isKnownBootstrap = source.userName.compare(
                    QStringLiteral("admin"), Qt::CaseInsensitive) == 0
                && CredentialSecurity::VerifyPasswordRecord(
                    source.userName, QStringLiteral("admin"), destinationHash)
                    != CredentialSecurity::PasswordVerification::Invalid;
            ScopedSettingIdentity mustChangeIdentity;
            mustChangeIdentity.valid = true;
            mustChangeIdentity.scopeType = QStringLiteral("account");
            mustChangeIdentity.scopeId = source.userName;
            mustChangeIdentity.module = QStringLiteral("Profile");
            mustChangeIdentity.keyName = QStringLiteral("MustChangePassword");
            mustChangeIdentity.valueType = QStringLiteral("bool");
            if (!WriteScopedSettingValue(
                    db,
                    mustChangeIdentity,
                    isKnownBootstrap ? QStringLiteral("1") : QStringLiteral("0")))
            {
                return false;
            }
        }
        else
        {
            QString existingMustChange;
            if (!ReadRawScopedValue(
                    db,
                    QStringLiteral("account"),
                    source.userName,
                    QStringLiteral("Profile"),
                    QStringLiteral("MustChangePassword"),
                    &existingMustChange))
            {
                ScopedSettingIdentity mustChangeIdentity;
                mustChangeIdentity.valid = true;
                mustChangeIdentity.scopeType = QStringLiteral("account");
                mustChangeIdentity.scopeId = source.userName;
                mustChangeIdentity.module = QStringLiteral("Profile");
                mustChangeIdentity.keyName = QStringLiteral("MustChangePassword");
                mustChangeIdentity.valueType = QStringLiteral("bool");
                if (!WriteScopedSettingValue(db, mustChangeIdentity, QStringLiteral("1")))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

bool ParseAuthenticationBool(const QString& value, bool* parsed)
{
    if (parsed == nullptr)
    {
        return false;
    }
    const QString normalized = value.trimmed().toLower();
    if (normalized == QLatin1String("1")
        || normalized == QLatin1String("true")
        || normalized == QLatin1String("yes"))
    {
        *parsed = true;
        return true;
    }
    if (normalized == QLatin1String("0")
        || normalized == QLatin1String("false")
        || normalized == QLatin1String("no"))
    {
        *parsed = false;
        return true;
    }
    return false;
}

bool IsAuthenticationRole(const QString& role)
{
    return role == QLatin1String("operator")
        || role == QLatin1String("engineer")
        || role == QLatin1String("admin");
}

bool NormalizeExistingAccountProfiles(QSqlDatabase& db)
{
    QSqlQuery accountQuery(db);
    if (!accountQuery.exec(QStringLiteral(
            "SELECT DISTINCT scope_id FROM settings "
            "WHERE scope_type='account' AND module='Profile' ORDER BY scope_id COLLATE NOCASE")))
    {
        return false;
    }
    QStringList accountIds;
    QMap<QString, QString> foldedAccountIds;
    while (accountQuery.next())
    {
        const QString accountId = accountQuery.value(0).toString().trimmed();
        const QString folded = accountId.toCaseFolded();
        if (!IsSafeMigratedAccountName(accountId)
            || (foldedAccountIds.contains(folded)
                && foldedAccountIds.value(folded) != accountId))
        {
            return false;
        }
        foldedAccountIds.insert(folded, accountId);
        accountIds.append(accountId);
    }
    accountQuery.finish();

    for (const QString& accountId : accountIds)
    {
        QSqlQuery profileQuery(db);
        profileQuery.prepare(QStringLiteral(
            "SELECT key_name, value_text, encrypted FROM settings "
            "WHERE scope_type='account' AND scope_id=? AND module='Profile' "
            "AND key_name IN ('PasswordHash', 'Role', 'MustChangePassword')"));
        profileQuery.addBindValue(accountId);
        if (!profileQuery.exec())
        {
            return false;
        }
        QMap<QString, QPair<QString, int>> storedValues;
        while (profileQuery.next())
        {
            storedValues.insert(
                profileQuery.value(0).toString(),
                qMakePair(profileQuery.value(1).toString(), profileQuery.value(2).toInt()));
        }
        profileQuery.finish();
        if (!storedValues.contains(QStringLiteral("PasswordHash"))
            || !storedValues.contains(QStringLiteral("Role")))
        {
            return false;
        }

        auto decode = [&storedValues, &accountId](const QString& key, QString* value)
            {
                const auto stored = storedValues.value(key);
                return DecodeStoredText(
                    stored.first,
                    stored.second,
                    ProtectionPurpose(
                        QStringLiteral("account"), accountId,
                        QStringLiteral("Profile"), key),
                    value);
            };
        QString passwordRecord;
        QString role;
        if (!decode(QStringLiteral("PasswordHash"), &passwordRecord)
            || !decode(QStringLiteral("Role"), &role)
            || !CredentialSecurity::IsSupportedPasswordRecord(passwordRecord)
            || !IsAuthenticationRole(role))
        {
            return false;
        }

        bool mustChangePassword = true;
        if (storedValues.contains(QStringLiteral("MustChangePassword")))
        {
            QString mustChangeText;
            if (!decode(QStringLiteral("MustChangePassword"), &mustChangeText)
                || !ParseAuthenticationBool(mustChangeText, &mustChangePassword))
            {
                return false;
            }
        }
        if (accountId.compare(QStringLiteral("admin"), Qt::CaseInsensitive) == 0
            && CredentialSecurity::VerifyPasswordRecord(
                accountId, QStringLiteral("admin"), passwordRecord)
                != CredentialSecurity::PasswordVerification::Invalid)
        {
            mustChangePassword = true;
        }

        for (const auto& value : {
                qMakePair(QStringLiteral("PasswordHash"), passwordRecord),
                qMakePair(QStringLiteral("Role"), role),
                qMakePair(
                    QStringLiteral("MustChangePassword"),
                    mustChangePassword ? QStringLiteral("1") : QStringLiteral("0")) })
        {
            ScopedSettingIdentity identity;
            identity.valid = true;
            identity.scopeType = QStringLiteral("account");
            identity.scopeId = accountId;
            identity.module = QStringLiteral("Profile");
            identity.keyName = value.first;
            identity.valueType = value.first == QLatin1String("MustChangePassword")
                ? QStringLiteral("bool")
                : QStringLiteral("string");
            identity.sensitive = value.first.contains(
                QStringLiteral("Password"), Qt::CaseInsensitive);
            if (!WriteScopedSettingValue(db, identity, value.second))
            {
                return false;
            }
        }
    }
    return true;
}

bool MigrateLegacyLoginState(QSqlDatabase& db)
{
    QSqlQuery loginQuery(db);
    loginQuery.prepare(
        "SELECT key_name, value_text, encrypted FROM settings "
        "WHERE scope_type='global' AND scope_id='' AND module='LoginState/General'");
    if (!loginQuery.exec())
    {
        return false;
    }
    QMap<QString, QString> loginValues;
    while (loginQuery.next())
    {
        const QString keyName = loginQuery.value(0).toString();
        if (keyName != QStringLiteral("UserName")
            && keyName != QStringLiteral("AccountHistory"))
        {
            continue;
        }
        QString decoded;
        if (!DecodeStoredText(
                loginQuery.value(1).toString(),
                loginQuery.value(2).toInt(),
                ProtectionPurpose(QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), keyName),
                &decoded))
        {
            return false;
        }
        loginValues.insert(keyName, decoded);
    }
    loginQuery.finish();
    for (auto it = loginValues.cbegin(); it != loginValues.cend(); ++it)
    {
        ScopedSettingIdentity identity;
        identity.valid = true;
        identity.scopeType = QStringLiteral("global");
        identity.module = QStringLiteral("LoginState");
        identity.keyName = it.key();
        identity.valueType = it.key() == QStringLiteral("AccountHistory")
            ? QStringLiteral("list")
            : QStringLiteral("string");
        if (!WriteScopedSettingValue(db, identity, it.value()))
        {
            return false;
        }
    }

    // Never migrate a reversible remembered password.  The next successful
    // login may opt into a fresh DPAPI-protected credential.
    if (!ExecSql(db, QStringLiteral(
            "DELETE FROM settings WHERE scope_type='global' AND scope_id='' AND ("
            "module LIKE 'Accounts/Users/%' OR module='LoginState/General' OR "
            "module LIKE 'LoginState/SavedPasswords%' OR "
            "module LIKE 'LoginState/RememberedCredentials%' OR "
            "lower(key_name)='passwordbase64')")))
    {
        return false;
    }

    for (const auto& pair : {
            qMakePair(QStringLiteral("RememberPassword"), QStringLiteral("0")),
            qMakePair(QStringLiteral("AutoLogin"), QStringLiteral("0")) })
    {
        ScopedSettingIdentity identity;
        identity.valid = true;
        identity.scopeType = QStringLiteral("global");
        identity.module = QStringLiteral("LoginState");
        identity.keyName = pair.first;
        identity.valueType = QStringLiteral("bool");
        if (!WriteScopedSettingValue(db, identity, pair.second))
        {
            return false;
        }
    }
    return true;
}

bool UpgradeRecoverableSensitiveSettings(QSqlDatabase& db)
{
    std::vector<SensitiveUpgrade> sensitiveUpgrades;
    QSqlQuery sensitiveQuery(db);
    if (!sensitiveQuery.exec(
            "SELECT scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted "
            "FROM settings"))
    {
        return false;
    }
    while (sensitiveQuery.next())
    {
        const QString scopeType = sensitiveQuery.value(0).toString();
        const QString scopeId = sensitiveQuery.value(1).toString();
        const QString module = sensitiveQuery.value(2).toString();
        const QString keyName = sensitiveQuery.value(3).toString();
        const QString storedText = sensitiveQuery.value(4).toString();
        const bool sensitive = sensitiveQuery.value(6).toInt() != 0
            || IsSensitiveSettingKey(module)
            || IsSensitiveSettingKey(keyName);
        const bool requiresDpapi = RequiresDpapiProtection(scopeType, module, keyName, sensitive);
        if (!requiresDpapi)
        {
            continue;
        }
        QString decoded;
        if (!DecodeStoredText(
                storedText,
                sensitiveQuery.value(7).toInt(),
                ProtectionPurpose(scopeType, scopeId, module, keyName),
                &decoded))
        {
            return false;
        }
        if (CredentialSecurity::IsCurrentUserProtected(storedText))
        {
            // A DPAPI-looking prefix is not evidence of a valid credential.
            // The canonical field purpose must decrypt successfully before the
            // schema/auth version can commit.
            continue;
        }
        sensitiveUpgrades.push_back({
            scopeType,
            scopeId,
            module,
            keyName,
            decoded,
            sensitiveQuery.value(5).toString() });
    }
    sensitiveQuery.finish();
    for (const SensitiveUpgrade& upgrade : sensitiveUpgrades)
    {
        ScopedSettingIdentity identity;
        identity.valid = true;
        identity.scopeType = upgrade.scopeType;
        identity.scopeId = upgrade.scopeId;
        identity.module = upgrade.module;
        identity.keyName = upgrade.keyName;
        identity.valueType = upgrade.valueType;
        identity.sensitive = true;
        if (!WriteScopedSettingValue(db, identity, upgrade.value))
        {
            return false;
        }
    }
    return true;
}

bool MigrateLegacyAuthenticationSettings(QSqlDatabase& db)
{
    return MigrateLegacyAccounts(db)
        && NormalizeExistingAccountProfiles(db)
        && MigrateLegacyLoginState(db)
        && UpgradeRecoverableSensitiveSettings(db)
        && SetMetaValue(
            db,
            QStringLiteral("auth_semantic_version"),
            QString::fromLatin1(kAuthenticationSemanticVersion));
}

bool SetCurrentSchemaMetadata(QSqlDatabase& db)
{
    return SetMetaValue(db, QStringLiteral("schema_version"), QString::fromLatin1(kSchemaVersion))
        && SetMetaValue(db, QStringLiteral("encrypt_new_values"), QStringLiteral("0"))
        && SetMetaValue(db, QStringLiteral("sensitive_protection"), QStringLiteral("dpapi-current-user-v1"));
}

bool HasLegacyDiskConfigurationInputs(const QString& databasePath)
{
    const QFileInfo databaseInfo(databasePath);
    QDirIterator iterator(
        databaseInfo.absolutePath(),
        QDir::Files | QDir::NoDotAndDotDot,
        QDirIterator::Subdirectories);
    while (iterator.hasNext())
    {
        iterator.next();
        const QString fileName = iterator.fileName().toLower();
        if (fileName.contains(QStringLiteral(".ini"))
            || fileName == QStringLiteral("weavedate.txt")
            || fileName == QStringLiteral("weldpara.txt"))
        {
            return true;
        }
    }
    return false;
}

bool HasUnsafePlaintextConfigStoreResidue(const QString& databasePath)
{
    const QDir directory(QFileInfo(databasePath).absolutePath());
    const QFileInfoList candidates = directory.entryInfoList(
        QStringList()
            << QStringLiteral("ConfigStore.db.bak*")
            << QStringLiteral(".ConfigStore.db.backup-snapshot-*.tmp")
            << QStringLiteral(".ConfigStore.db.restore-*.tmp")
            << QStringLiteral(".ConfigStore.db.staging-*.tmp"),
        QDir::Files | QDir::NoDotAndDotDot);
    return std::any_of(candidates.cbegin(), candidates.cend(), [](const QFileInfo& candidate)
        {
            return !candidate.fileName().endsWith(
                QStringLiteral(".dpapi.bak"), Qt::CaseInsensitive);
        });
}

bool EnsureCurrentSchema(QSqlDatabase& db)
{
    const bool hasMeta = TableExists(db, QLatin1String("meta"));
    const QString existingVersion = hasMeta ? MetaValue(db, QLatin1String("schema_version")) : QString();
    if (hasMeta && existingVersion.isEmpty())
    {
        return false;
    }
    if (!existingVersion.isEmpty()
        && existingVersion != QStringLiteral("4")
        && existingVersion != QString::fromLatin1(kSchemaVersion))
    {
        return false;
    }
    if (HasUnsafePlaintextConfigStoreResidue(db.databaseName()))
    {
        qCritical() << "Plaintext ConfigStore backup or temporary residue requires review; "
                       "remove it and rotate affected credentials before starting the application.";
        return false;
    }
    if ((existingVersion.isEmpty() || existingVersion == QStringLiteral("4"))
        && HasLegacyDiskConfigurationInputs(db.databaseName()))
    {
        qCritical() << "Legacy INI/TXT configuration exists beside a non-current ConfigStore; "
                       "run ConfigMigrate before starting the application.";
        return false;
    }

    if (!hasMeta)
    {
        QSqlQuery tablesQuery(db);
        if (!tablesQuery.exec("SELECT COUNT(*) FROM sqlite_master WHERE type='table'")
            || !tablesQuery.next()
            || tablesQuery.value(0).toInt() != 0)
        {
            return false;
        }
    }

    if (hasMeta && (!TableExists(db, QStringLiteral("settings")) || !HasCurrentSettingsColumns(db)))
    {
        return false;
    }

    const QString credentialScrubState = hasMeta
        ? MetaValue(db, QStringLiteral("legacy_credential_scrub_state"))
        : QString();
    if (!credentialScrubState.isEmpty()
        && credentialScrubState != QLatin1String("complete"))
    {
        qCritical() << "Legacy credential scrub is incomplete or invalid; "
                       "rerun ConfigMigrate before starting the application.";
        return false;
    }

    const QString authenticationSemanticVersion = MetaValue(
        db, QStringLiteral("auth_semantic_version"));
    const bool needsMigration = existingVersion != QString::fromLatin1(kSchemaVersion)
        || authenticationSemanticVersion != QString::fromLatin1(kAuthenticationSemanticVersion);
    if (!needsMigration)
    {
        bool authenticationStateValid = false;
        const int authenticationState = MetaValue(
            db, QLatin1String("auth_initialized")).toInt(&authenticationStateValid);
        if (!authenticationStateValid || authenticationState < 0 || authenticationState > 1)
        {
            return false;
        }
        return SetCurrentSchemaMetadata(db);
    }
    QString authenticationInitializedValue = hasMeta ? QStringLiteral("1") : QStringLiteral("0");
    if (existingVersion == QString::fromLatin1(kSchemaVersion))
    {
        const QString existingAuthenticationState = MetaValue(
            db, QStringLiteral("auth_initialized"));
        if (existingAuthenticationState != QLatin1String("0")
            && existingAuthenticationState != QLatin1String("1"))
        {
            return false;
        }
        authenticationInitializedValue = existingAuthenticationState;
    }
    if (!db.transaction())
    {
        return false;
    }
    if (!CreateCurrentTables(db)
        || !HasCurrentSettingsColumns(db)
        || !MigrateLegacyAuthenticationSettings(db)
        || !SetMetaValue(
            db,
            QLatin1String("auth_initialized"),
            authenticationInitializedValue)
        || !SetCurrentSchemaMetadata(db)
        || !db.commit())
    {
        db.rollback();
        return false;
    }
    return true;
}

QStringList UniqueSorted(QStringList values)
{
    values.removeDuplicates();
    values.sort(Qt::CaseInsensitive);
    return values;
}
}

QString ConfigDatabase::DatabasePath()
{
    return AppPaths::WritablePath(QStringLiteral("Data/ConfigStore.db"));
}

QString ConfigDatabase::NormalizeFilePath(const QString& fileName)
{
    QString path = fileName.trimmed();
    path.replace('\\', '/');
    path = QDir::cleanPath(path);
    while (path.startsWith("./"))
    {
        path.remove(0, 2);
    }

    QString lower = path.toLower();
    if (lower.startsWith("data/"))
    {
        return path;
    }
    if (lower.startsWith("result/"))
    {
        return path;
    }

    QFileInfo info(path);
    if (info.isAbsolute())
    {
        QDir root(AppPaths::DataRootPath());
        const QString rel = root.relativeFilePath(info.absoluteFilePath()).replace('\\', '/');
        const QString relLower = rel.toLower();
        const bool insideDataRoot = !QFileInfo(rel).isAbsolute()
            && rel != QStringLiteral("..")
            && !rel.startsWith(QStringLiteral("../"));
        if (insideDataRoot && relLower.startsWith("data/"))
        {
            return QDir::cleanPath(rel);
        }
        if (insideDataRoot && relLower.startsWith("result/"))
        {
            return QDir::cleanPath(rel);
        }
    }

    // 兼容从旧工程根传入的绝对/带前缀路径。用最后一个段标记，避免
    // D:/data/site/Data/... 误截成 data/site/Data/...。
    lower = path.toLower();
    const int dataPos = lower.lastIndexOf("/data/");
    if (dataPos >= 0)
    {
        return path.mid(dataPos + 1);
    }
    const int resultPos = lower.lastIndexOf("/result/");
    if (resultPos >= 0)
    {
        return path.mid(resultPos + 1);
    }

    return path;
}

std::string ConfigDatabase::NormalizeFilePath(const std::string& fileName)
{
    const QString normalized = NormalizeFilePath(DecodeMaybeLocal(fileName));
    const QByteArray bytes = normalized.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

bool ConfigDatabase::IsAvailable()
{
    QSqlDatabase db = OpenDatabase();
    return db.isValid() && db.isOpen();
}

bool ConfigDatabase::HasIniFile(const std::string& fileName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity identity = BuildScopedFileIdentity(FromUtf8StdString(NormalizeFilePath(fileName)));
    if (!identity.valid)
    {
        return false;
    }
    const QString modulePrefix = NormalizeSection(identity.module) + QStringLiteral("/");

    QSqlQuery query(db);
    query.prepare("SELECT 1 FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?) LIMIT 1");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(NormalizeSection(identity.module));
    query.addBindValue(modulePrefix.size());
    query.addBindValue(modulePrefix);
    return query.exec() && query.next();
}

bool ConfigDatabase::HasIniFile(const QString& fileName)
{
    const QByteArray bytes = fileName.toUtf8();
    return HasIniFile(std::string(bytes.constData(), static_cast<size_t>(bytes.size())));
}

bool ConfigDatabase::ReadIniValue(
    const std::string& fileName,
    const std::string& sectionName,
    const std::string& keyName,
    std::string* value)
{
    return ReadIniValueStatus(fileName, sectionName, keyName, value)
        == ReadStatus::Found;
}

ConfigDatabase::ReadStatus ConfigDatabase::ReadIniValueStatus(
    const std::string& fileName,
    const std::string& sectionName,
    const std::string& keyName,
    std::string* value)
{
    if (value == nullptr)
    {
        return ReadStatus::Error;
    }
    value->clear();

    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return ReadStatus::Error;
    }

    const ScopedSettingIdentity scopedIdentity = BuildScopedIniIdentity(
        FromUtf8StdString(NormalizeFilePath(fileName)),
        DecodeMaybeLocal(sectionName),
        DecodeMaybeLocal(keyName));
    if (!scopedIdentity.valid)
    {
        return ReadStatus::Error;
    }
    QString scopedValue;
    const ReadStatus status = ReadScopedSettingValueStatus(db, scopedIdentity, &scopedValue);
    if (status != ReadStatus::Found)
    {
        return status;
    }

    const QByteArray bytes = scopedValue.toUtf8();
    *value = std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
    return ReadStatus::Found;
}

bool ConfigDatabase::WriteIniValue(
    const std::string& fileName,
    const std::string& sectionName,
    const std::string& keyName,
    const std::string& value)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity scopedIdentity = BuildScopedIniIdentity(
        FromUtf8StdString(NormalizeFilePath(fileName)),
        DecodeMaybeLocal(sectionName),
        DecodeMaybeLocal(keyName));
    return WriteScopedSettingValue(db, scopedIdentity, DecodeMaybeLocal(value));
}

bool ConfigDatabase::HasTextFile(const std::string& fileName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity identity = BuildScopedFileIdentity(FromUtf8StdString(NormalizeFilePath(fileName)));
    QString value;
    return ReadScopedSettingValue(db, identity, &value);
}

bool ConfigDatabase::HasTextFile(const QString& fileName)
{
    const QByteArray bytes = fileName.toUtf8();
    return HasTextFile(std::string(bytes.constData(), static_cast<size_t>(bytes.size())));
}

bool ConfigDatabase::ReadTextFile(const std::string& fileName, std::string* content)
{
    if (content == nullptr)
    {
        return false;
    }

    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity identity = BuildScopedFileIdentity(FromUtf8StdString(NormalizeFilePath(fileName)));
    QString scopedValue;
    if (!ReadScopedSettingValue(db, identity, &scopedValue))
    {
        return false;
    }
    const QByteArray bytes = scopedValue.toUtf8();
    *content = std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
    return true;
}

bool ConfigDatabase::WriteTextFile(const std::string& fileName, const std::string& content)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    ScopedSettingIdentity identity = BuildScopedFileIdentity(FromUtf8StdString(NormalizeFilePath(fileName)));
    identity.valueType = QStringLiteral("text");
    return WriteScopedSettingValue(db, identity, DecodeMaybeLocal(content));
}

bool ConfigDatabase::CopyTextFile(const QString& sourceFileName, const QString& targetFileName, bool overwriteExisting)
{
    std::string content;
    if (!ReadTextFile(sourceFileName.toUtf8().constData(), &content))
    {
        return false;
    }
    if (!overwriteExisting && HasTextFile(targetFileName))
    {
        return true;
    }
    return WriteTextFile(targetFileName.toUtf8().constData(), content);
}

bool ConfigDatabase::RemoveConfigPathPrefix(const QString& sourcePathPrefix)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QString normalized = NormalizeFilePath(sourcePathPrefix);
    normalized.replace('\\', '/');
    while (normalized.endsWith('/'))
    {
        normalized.chop(1);
    }
    if (normalized.isEmpty())
    {
        return false;
    }

    if (!db.transaction())
    {
        return false;
    }

    const QStringList prefixParts = normalized.split('/', Qt::SkipEmptyParts);
    QString deleteScopeType;
    QString deleteScopeId;
    if (prefixParts.size() == 2
        && prefixParts.at(0).compare(QStringLiteral("Data"), Qt::CaseInsensitive) == 0)
    {
        deleteScopeType = QStringLiteral("robot");
        deleteScopeId = prefixParts.at(1);
    }
    else if (prefixParts.size() == 3
        && prefixParts.at(0).compare(QStringLiteral("Data"), Qt::CaseInsensitive) == 0
        && prefixParts.at(1).compare(QStringLiteral("WorkpieceTemplates"), Qt::CaseInsensitive) == 0)
    {
        deleteScopeType = QStringLiteral("workpiece_template");
        deleteScopeId = prefixParts.at(2);
    }

    if (!deleteScopeType.isEmpty())
    {
        QSqlQuery scopedDelete(db);
        scopedDelete.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=?");
        scopedDelete.addBindValue(NormalizeSection(deleteScopeType).toLower());
        scopedDelete.addBindValue(NormalizeScopeId(deleteScopeId));
        if (!scopedDelete.exec())
        {
            db.rollback();
            return false;
        }
    }
    else
    {
        const ScopedSettingIdentity prefixIdentity = BuildScopedFileIdentity(normalized);
        if (prefixIdentity.valid)
        {
            QSqlQuery scopedDelete(db);
            if (normalized.endsWith(QStringLiteral(".ini"), Qt::CaseInsensitive)
                || normalized.endsWith(QStringLiteral(".txt"), Qt::CaseInsensitive))
            {
                const QString modulePrefix = NormalizeSection(prefixIdentity.module) + QStringLiteral("/");
                scopedDelete.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?)");
                scopedDelete.addBindValue(NormalizeSection(prefixIdentity.scopeType).toLower());
                scopedDelete.addBindValue(NormalizeScopeId(prefixIdentity.scopeId));
                scopedDelete.addBindValue(NormalizeSection(prefixIdentity.module));
                scopedDelete.addBindValue(modulePrefix.size());
                scopedDelete.addBindValue(modulePrefix);
            }
            else
            {
                scopedDelete.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=?");
                scopedDelete.addBindValue(NormalizeSection(prefixIdentity.scopeType).toLower());
                scopedDelete.addBindValue(NormalizeScopeId(prefixIdentity.scopeId));
            }
            if (!scopedDelete.exec())
            {
                db.rollback();
                return false;
            }
        }
    }

    return db.commit();
}



bool ConfigDatabase::CopyIniFile(const QString& sourceFileName, const QString& targetFileName, bool overwriteExisting)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity source = BuildScopedFileIdentity(sourceFileName);
    const ScopedSettingIdentity target = BuildScopedFileIdentity(targetFileName);
    if (!source.valid || !target.valid || !HasIniFile(sourceFileName))
    {
        return false;
    }

    if (overwriteExisting)
    {
        QSqlQuery deleteQuery(db);
        const QString targetPrefix = NormalizeSection(target.module) + QStringLiteral("/");
        deleteQuery.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?)");
        deleteQuery.addBindValue(NormalizeSection(target.scopeType).toLower());
        deleteQuery.addBindValue(NormalizeScopeId(target.scopeId));
        deleteQuery.addBindValue(NormalizeSection(target.module));
        deleteQuery.addBindValue(targetPrefix.size());
        deleteQuery.addBindValue(targetPrefix);
        if (!deleteQuery.exec())
        {
            return false;
        }
    }

    QSqlQuery sourceQuery(db);
    const QString sourceModule = NormalizeSection(source.module);
    const QString sourcePrefix = sourceModule + QStringLiteral("/");
    sourceQuery.prepare(
        "SELECT module, key_name, value_text, value_type, sensitive, encrypted, updated_at "
        "FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?)");
    sourceQuery.addBindValue(NormalizeSection(source.scopeType).toLower());
    sourceQuery.addBindValue(NormalizeScopeId(source.scopeId));
    sourceQuery.addBindValue(sourceModule);
    sourceQuery.addBindValue(sourcePrefix.size());
    sourceQuery.addBindValue(sourcePrefix);
    if (!sourceQuery.exec())
    {
        return false;
    }

    QSqlQuery insertQuery(db);
    insertQuery.prepare(QStringLiteral("INSERT OR %1 INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?)")
        .arg(overwriteExisting ? QStringLiteral("REPLACE") : QStringLiteral("IGNORE")));
    while (sourceQuery.next())
    {
        const QString sourceModuleValue = NormalizeSection(sourceQuery.value(0).toString());
        const QString suffix = sourceModuleValue == sourceModule
            ? QString()
            : sourceModuleValue.mid(sourceModule.size());
        insertQuery.addBindValue(NormalizeSection(target.scopeType).toLower());
        insertQuery.addBindValue(NormalizeScopeId(target.scopeId));
        insertQuery.addBindValue(NormalizeSection(target.module + suffix));
        insertQuery.addBindValue(NormalizeSourceKey(sourceQuery.value(1).toString()));
        insertQuery.addBindValue(sourceQuery.value(2));
        insertQuery.addBindValue(sourceQuery.value(3));
        insertQuery.addBindValue(sourceQuery.value(4));
        insertQuery.addBindValue(sourceQuery.value(5));
        insertQuery.addBindValue(sourceQuery.value(6));
        if (!insertQuery.exec())
        {
            return false;
        }
    }
    return true;
}

bool ConfigDatabase::RemoveIniGroup(const QString& fileName, const QString& groupName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity identity = BuildScopedFileIdentity(fileName);
    if (!identity.valid)
    {
        return false;
    }
    const QString normalizedGroup = NormalizeSection(groupName);
    const QString module = NormalizeSection(identity.module + QStringLiteral("/") + normalizedGroup);
    const QString modulePrefix = module + QStringLiteral("/");
    QSqlQuery query(db);
    query.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?)");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(module);
    query.addBindValue(modulePrefix.size());
    query.addBindValue(modulePrefix);
    return query.exec();
}

QMap<QString, QString> ConfigDatabase::ReadIniSection(const QString& fileName, const QString& sectionName)
{
    QMap<QString, QString> values;
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return values;
    }

    const ScopedSettingIdentity identity = BuildScopedIniIdentity(fileName, sectionName, QStringLiteral("dummy"));
    if (!identity.valid)
    {
        return values;
    }

    QSqlQuery query(db);
    query.prepare("SELECT key_name, value_text, encrypted FROM settings WHERE scope_type=? AND scope_id=? AND module=? ORDER BY key_name COLLATE NOCASE");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(NormalizeSection(identity.module));
    if (!query.exec())
    {
        return values;
    }

    while (query.next())
    {
        QString plainText;
        const QString keyName = query.value(0).toString();
        if (!DecodeStoredText(
                query.value(1).toString(),
                query.value(2).toInt(),
                ProtectionPurpose(identity.scopeType, identity.scopeId, identity.module, keyName),
                &plainText))
        {
            continue;
        }
        values.insert(keyName, plainText);
    }
    return values;
}

bool ConfigDatabase::RemoveIniSection(const QString& fileName, const QString& sectionName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity identity = BuildScopedIniIdentity(fileName, sectionName, QStringLiteral("dummy"));
    if (!identity.valid)
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND module=?");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(NormalizeSection(identity.module));
    return query.exec();
}

bool ConfigDatabase::ReadScopedSetting(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName,
    QString* value)
{
    return ReadScopedSettingStatus(scopeType, scopeId, moduleName, keyName, value)
        == ReadStatus::Found;
}

ConfigDatabase::ReadStatus ConfigDatabase::ReadScopedSettingStatus(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName,
    QString* value)
{
    if (value == nullptr)
    {
        return ReadStatus::Error;
    }
    value->clear();
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return ReadStatus::Error;
    }
    ScopedSettingIdentity identity;
    identity.scopeType = scopeType;
    identity.scopeId = scopeId;
    identity.module = moduleName;
    identity.keyName = keyName;
    identity.sensitive = IsSensitiveSettingKey(moduleName) || IsSensitiveSettingKey(keyName);
    identity.valid = !NormalizeSection(scopeType).isEmpty()
        && !NormalizeSection(moduleName).isEmpty()
        && !NormalizeSourceKey(keyName).isEmpty();
    return ReadScopedSettingValueStatus(db, identity, value);
}

QMap<QString, QString> ConfigDatabase::ReadScopedSettings(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName)
{
    QMap<QString, QString> values;
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return values;
    }
    QSqlQuery query(db);
    query.prepare(
        "SELECT key_name, value_text, encrypted, sensitive FROM settings "
        "WHERE scope_type=? AND scope_id=? AND module=? ORDER BY key_name");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(NormalizeSection(moduleName));
    if (!query.exec())
    {
        return values;
    }
    QMap<QString, QString> upgrades;
    while (query.next())
    {
        QString decoded;
        const QString keyName = query.value(0).toString();
        if (DecodeStoredText(
                query.value(1).toString(),
                query.value(2).toInt(),
                ProtectionPurpose(scopeType, scopeId, moduleName, keyName),
                &decoded))
        {
            values.insert(keyName, decoded);
            const bool sensitive = query.value(3).toInt() != 0
                || IsSensitiveSettingKey(moduleName)
                || IsSensitiveSettingKey(keyName);
            const bool requiresDpapi = RequiresDpapiProtection(scopeType, moduleName, keyName, sensitive);
            if (requiresDpapi && !CredentialSecurity::IsCurrentUserProtected(query.value(1).toString()))
            {
                upgrades.insert(keyName, decoded);
            }
        }
    }
    query.finish();
    for (auto it = upgrades.cbegin(); it != upgrades.cend(); ++it)
    {
        ScopedSettingIdentity identity;
        identity.valid = true;
        identity.scopeType = scopeType;
        identity.scopeId = scopeId;
        identity.module = moduleName;
        identity.keyName = it.key();
        identity.sensitive = true;
        if (!WriteScopedSettingValue(db, identity, it.value()))
        {
            return QMap<QString, QString>();
        }
    }
    return values;
}

bool ConfigDatabase::WriteScopedSetting(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName,
    const QString& value,
    const QString& valueType,
    bool sensitive)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const bool effectiveSensitive = sensitive
        || IsSensitiveSettingKey(moduleName)
        || IsSensitiveSettingKey(keyName);
    int encrypted = 0;
    QString storedText;
    const QString purpose = ProtectionPurpose(scopeType, scopeId, moduleName, keyName);
    const bool requiresDpapi = RequiresDpapiProtection(scopeType, moduleName, keyName, effectiveSensitive);
    if (!EncodeStoredText(db, value, requiresDpapi, purpose, &storedText, &encrypted))
    {
        return false;
    }
    QSqlQuery query(db);
    query.prepare(
        "INSERT OR REPLACE INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(NormalizeSection(moduleName));
    query.addBindValue(NormalizeSourceKey(keyName));
    query.addBindValue(storedText);
    query.addBindValue(valueType.trimmed().isEmpty() ? QStringLiteral("string") : valueType.trimmed().toLower());
    query.addBindValue(effectiveSensitive ? 1 : 0);
    query.addBindValue(encrypted);
    return query.exec();
}

bool ConfigDatabase::WriteScopedSettings(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QMap<QString, QString>& values,
    const QString& valueType)
{
    if (values.isEmpty())
    {
        return true;
    }
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen() || !db.transaction())
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare(
        "INSERT OR REPLACE INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))");
    const QString normalizedScopeType = NormalizeSection(scopeType).toLower();
    const QString normalizedScopeId = NormalizeScopeId(scopeId);
    const QString normalizedModule = NormalizeSection(moduleName);
    const QString normalizedValueType = valueType.trimmed().isEmpty()
        ? QStringLiteral("string")
        : valueType.trimmed().toLower();
    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        const bool sensitive = IsSensitiveSettingKey(normalizedModule)
            || IsSensitiveSettingKey(it.key());
        int encrypted = 0;
        QString storedText;
        const QString purpose = ProtectionPurpose(
            normalizedScopeType, normalizedScopeId, normalizedModule, it.key());
        const bool requiresDpapi = RequiresDpapiProtection(
            normalizedScopeType, normalizedModule, it.key(), sensitive);
        if (!EncodeStoredText(db, it.value(), requiresDpapi, purpose, &storedText, &encrypted))
        {
            db.rollback();
            return false;
        }
        query.bindValue(0, normalizedScopeType);
        query.bindValue(1, normalizedScopeId);
        query.bindValue(2, normalizedModule);
        query.bindValue(3, NormalizeSourceKey(it.key()));
        query.bindValue(4, storedText);
        query.bindValue(5, normalizedValueType);
        query.bindValue(6, sensitive ? 1 : 0);
        query.bindValue(7, encrypted);
        if (!query.exec())
        {
            db.rollback();
            return false;
        }
        query.finish();
    }
    if (!db.commit())
    {
        db.rollback();
        return false;
    }
    return true;
}

bool ConfigDatabase::RemoveScopedSetting(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(NormalizeSection(moduleName));
    query.addBindValue(NormalizeSourceKey(keyName));
    return query.exec();
}

QStringList ConfigDatabase::ListScopedSettingIds(const QString& scopeType, const QString& moduleName)
{
    QStringList ids;
    return TryListScopedSettingIds(scopeType, moduleName, &ids) ? ids : QStringList();
}

bool ConfigDatabase::TryListScopedSettingIds(
    const QString& scopeType,
    const QString& moduleName,
    QStringList* ids)
{
    if (ids == nullptr)
    {
        return false;
    }
    ids->clear();
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QSqlQuery query(db);
    if (moduleName.trimmed().isEmpty())
    {
        query.prepare("SELECT DISTINCT scope_id FROM settings WHERE scope_type=? ORDER BY scope_id COLLATE NOCASE");
        query.addBindValue(NormalizeSection(scopeType).toLower());
    }
    else
    {
        query.prepare("SELECT DISTINCT scope_id FROM settings WHERE scope_type=? AND module=? ORDER BY scope_id COLLATE NOCASE");
        query.addBindValue(NormalizeSection(scopeType).toLower());
        query.addBindValue(NormalizeSection(moduleName));
    }
    if (!query.exec())
    {
        return false;
    }

    QStringList values;
    while (query.next())
    {
        values << query.value(0).toString();
    }
    *ids = UniqueSorted(values);
    return true;
}

bool ConfigDatabase::TryListScopedSettingIdsBounded(
    const QString& scopeType,
    const QString& moduleName,
    qsizetype maxCount,
    qsizetype maxIdLength,
    QStringList* ids)
{
    if (ids == nullptr || maxCount <= 0 || maxIdLength <= 0)
    {
        return false;
    }
    ids->clear();
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QSqlQuery query(db);
    if (moduleName.trimmed().isEmpty())
    {
        query.prepare(
            "SELECT DISTINCT scope_id FROM settings WHERE scope_type=? "
            "ORDER BY scope_id COLLATE NOCASE LIMIT ?");
        query.addBindValue(NormalizeSection(scopeType).toLower());
        query.addBindValue(static_cast<qlonglong>(maxCount + 1));
    }
    else
    {
        query.prepare(
            "SELECT DISTINCT scope_id FROM settings WHERE scope_type=? AND module=? "
            "ORDER BY scope_id COLLATE NOCASE LIMIT ?");
        query.addBindValue(NormalizeSection(scopeType).toLower());
        query.addBindValue(NormalizeSection(moduleName));
        query.addBindValue(static_cast<qlonglong>(maxCount + 1));
    }
    if (!query.exec())
    {
        return false;
    }

    QStringList values;
    while (query.next())
    {
        const QString id = query.value(0).toString();
        if (id.isEmpty() || id.size() > maxIdLength || values.size() >= maxCount)
        {
            return false;
        }
        values << id;
    }
    if (query.lastError().isValid())
    {
        return false;
    }
    *ids = UniqueSorted(values);
    return ids->size() == values.size();
}

bool ConfigDatabase::RemoveScopedSettings(const QString& scopeType, const QString& scopeId, const QString& moduleName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QSqlQuery query(db);
    if (moduleName.trimmed().isEmpty())
    {
        query.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=?");
        query.addBindValue(NormalizeSection(scopeType).toLower());
        query.addBindValue(NormalizeScopeId(scopeId));
    }
    else
    {
        query.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND module=?");
        query.addBindValue(NormalizeSection(scopeType).toLower());
        query.addBindValue(NormalizeScopeId(scopeId));
        query.addBindValue(NormalizeSection(moduleName));
    }
    return query.exec();
}
