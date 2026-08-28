#include "ConfigDatabase.h"

#include "AppPaths.h"
#include "CredentialSecurity.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDate>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSet>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QThread>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{
constexpr char kSchemaVersion[] = "5";
constexpr char kAuthenticationSemanticVersion[] = "3";
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
bool WriteScopedSettingValue(QSqlDatabase& db, const ScopedSettingIdentity& identity, const QString& value);
bool EnsureCurrentSchema(QSqlDatabase& db);
bool HasUnsafePlaintextConfigStoreResidue(const QString& databasePath);
bool HasInstallerTransactionRecord(const QString& databasePath);

class ThreadDatabaseConnection
{
public:
    ThreadDatabaseConnection()
    {
        static std::atomic<quint64> nextId{ 0 };
        const quintptr threadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
        m_name = QStringLiteral("ConfigStore_%1_%2")
            .arg(threadId)
            .arg(nextId.fetch_add(1, std::memory_order_relaxed));
    }

    ~ThreadDatabaseConnection()
    {
        if (!QSqlDatabase::contains(m_name))
        {
            return;
        }
        {
            QSqlDatabase database = QSqlDatabase::database(m_name, false);
            if (database.isValid())
            {
                database.close();
            }
        }
        QSqlDatabase::removeDatabase(m_name);
    }

    const QString& name() const
    {
        return m_name;
    }

private:
    QString m_name;
};

QString ConnectionName()
{
    thread_local ThreadDatabaseConnection connection;
    return connection.name();
}

QString FindProjectRoot()
{
    return AppPaths::DataRootPath();
}

bool HasInstallerTransactionRecord(const QString& databasePath)
{
    const QFileInfo databaseInfo(databasePath);
    const QString recordName = QStringLiteral("ConfigStore.db.install-transaction-v1");
    const QString recordPath = databaseInfo.dir().filePath(recordName);
    const QFileInfo recordInfo(recordPath);
    if (recordInfo.exists() || recordInfo.isSymLink())
    {
        return true;
    }

    // QFileInfo::exists() is false for a dangling symbolic link. Enumerating with
    // QDir::System keeps that link visible, so every filesystem object at the
    // transaction-record path remains a fail-closed startup barrier.
    const QDir dataDirectory = databaseInfo.dir();
    const QFileInfoList entries = dataDirectory.entryInfoList(
        QDir::AllEntries | QDir::Hidden | QDir::System | QDir::NoDotAndDotDot);
    return std::any_of(entries.cbegin(), entries.cend(), [&recordName](const QFileInfo& entry)
        {
#ifdef Q_OS_WIN
            return entry.fileName().compare(recordName, Qt::CaseInsensitive) == 0;
#else
            return entry.fileName() == recordName;
#endif
        });
}

QSqlDatabase OpenDatabase()
{
    const QString configuredDatabasePath = ConfigDatabase::DatabasePath();
    const QString connectionName = ConnectionName();
    if (HasInstallerTransactionRecord(configuredDatabasePath))
    {
        if (QSqlDatabase::contains(connectionName))
        {
            QSqlDatabase existingDatabase = QSqlDatabase::database(connectionName, false);
            if (existingDatabase.isValid())
            {
                existingDatabase.close();
            }
        }
        qCritical() << "ConfigStore installer transaction is unresolved; "
                       "account access remains in security recovery.";
        return QSqlDatabase();
    }

    if (QSqlDatabase::contains(connectionName))
    {
        QSqlDatabase db = QSqlDatabase::database(connectionName, false);
        const QString connectionDatabasePath = db.databaseName().isEmpty()
            ? configuredDatabasePath : db.databaseName();
        if (HasInstallerTransactionRecord(connectionDatabasePath))
        {
            if (db.isValid())
            {
                db.close();
            }
            qCritical() << "ConfigStore installer transaction appeared while a database "
                           "connection was open; account access remains in security recovery.";
            return QSqlDatabase();
        }
        if (db.isOpen())
        {
            return db;
        }
        if (!QFileInfo::exists(db.databaseName())
            && HasUnsafePlaintextConfigStoreResidue(db.databaseName()))
        {
            qCritical() << "Unsafe plaintext ConfigStore residue exists without ConfigStore.";
            return QSqlDatabase();
        }
        if (db.open())
        {
            if (HasInstallerTransactionRecord(connectionDatabasePath))
            {
                db.close();
                return QSqlDatabase();
            }
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

    const QString dbPath = configuredDatabasePath;
    const QFileInfo dbInfo(dbPath);
    QDir dbDir = dbInfo.dir();
    if (!dbDir.exists() && !dbDir.mkpath(QStringLiteral(".")))
    {
        return QSqlDatabase();
    }
    if (!dbInfo.exists() && HasUnsafePlaintextConfigStoreResidue(dbPath))
    {
        qCritical() << "Unsafe plaintext ConfigStore residue exists without ConfigStore.";
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
    if (HasInstallerTransactionRecord(dbPath))
    {
        db.close();
        return QSqlDatabase();
    }
    if (!db.open())
    {
        return QSqlDatabase();
    }

    if (HasInstallerTransactionRecord(dbPath))
    {
        db.close();
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

    if (HasInstallerTransactionRecord(dbPath))
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

bool HasLegacyConfigFileSuffix(const QString& value)
{
    const QString lower = value.trimmed().toLower();
    return lower.endsWith(QStringLiteral(".ini"))
        || lower.endsWith(QStringLiteral(".txt"))
        || lower.endsWith(QStringLiteral(".cfg"))
        || lower.endsWith(QStringLiteral(".conf"));
}

bool IsDatabaseNativeIdentityPart(
    const QString& rawValue,
    bool allowHierarchy,
    bool allowEmpty = false,
    bool allowColon = false)
{
    const QString value = rawValue.trimmed();
    if (value.isEmpty())
    {
        return allowEmpty;
    }
    if (value.contains(QLatin1Char('\\'))
        || (!allowColon && value.contains(QLatin1Char(':')))
        || (allowColon && value.size() >= 2
            && value.at(0).isLetter() && value.at(1) == QLatin1Char(':'))
        || (!allowHierarchy && value.contains(QLatin1Char('/')))
        || value.startsWith(QLatin1Char('/'))
        || value.endsWith(QLatin1Char('/'))
        || value.contains(QStringLiteral("//")))
    {
        return false;
    }
    const QStringList parts = value.split(QLatin1Char('/'));
    for (const QString& part : parts)
    {
        if (part.isEmpty()
            || part == QStringLiteral(".")
            || part == QStringLiteral("..")
            || HasLegacyConfigFileSuffix(part))
        {
            return false;
        }
    }
    return true;
}

bool IsDatabaseNativeScopeIdentity(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    bool allowEmptyModule = false)
{
    const QString normalizedScope = NormalizeSection(scopeType).toLower();
    if (!IsDatabaseNativeIdentityPart(normalizedScope, false)
        || !IsDatabaseNativeIdentityPart(moduleName, true, allowEmptyModule))
    {
        return false;
    }
    if (normalizedScope == QStringLiteral("global"))
    {
        return scopeId.trimmed().isEmpty();
    }
    return IsDatabaseNativeIdentityPart(scopeId, false, false, true);
}

bool IsDatabaseNativeSettingIdentity(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName)
{
    return IsDatabaseNativeScopeIdentity(scopeType, scopeId, moduleName)
        && IsDatabaseNativeIdentityPart(keyName, false);
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
        || lower.contains(QStringLiteral("apikey"))
        || lower == QLatin1String("autologin");
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
        || keyName.compare(QStringLiteral("PasswordChangedAt"), Qt::CaseInsensitive) == 0
        || keyName.compare(QStringLiteral("CreatedAt"), Qt::CaseInsensitive) == 0
        || keyName.compare(QStringLiteral("UpdatedAt"), Qt::CaseInsensitive) == 0;
}

bool HasPortableAuthenticationStorage(const QString& storedText, int encrypted)
{
    return encrypted == 0
        && !CredentialSecurity::IsCurrentUserProtected(storedText)
        && !storedText.startsWith(QLatin1String("enc:v1:"));
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
    query.prepare(
        "SELECT 1 FROM sqlite_master WHERE type='table' "
        "AND lower(name)=lower(?) LIMIT 1");
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

const QStringList& LegacyAccountFieldNames()
{
    static const QStringList fields = {
        QStringLiteral("PasswordHash"),
        QStringLiteral("Role"),
        QStringLiteral("CreatedAt"),
        QStringLiteral("UpdatedAt"),
        QStringLiteral("MustChangePassword")
    };
    return fields;
}

bool CanonicalLegacyAccountField(const QString& fieldName, QString* canonical)
{
    if (canonical == nullptr)
    {
        return false;
    }
    for (const QString& supported : LegacyAccountFieldNames())
    {
        if (fieldName.compare(supported, Qt::CaseInsensitive) == 0)
        {
            *canonical = supported;
            return true;
        }
    }
    return false;
}

bool ParseAuthenticationBool(const QString& value, bool* parsed);
bool CanonicalCurrentAccountSecurityField(
    const QString& fieldName,
    QString* canonical);

bool IsLegacyAccountTimestampField(const QString& fieldName)
{
    return fieldName == QLatin1String("CreatedAt")
        || fieldName == QLatin1String("UpdatedAt");
}

bool ParseAccountTimestampInstant(const QString& value, qint64* instantMicros)
{
    if (instantMicros == nullptr
        || value != value.trimmed()
        || value.size() > 64)
    {
        return false;
    }
    static const QRegularExpression expression(QStringLiteral(
        "\\A(\\d{4})-(\\d{2})-(\\d{2})T(\\d{2}):(\\d{2}):(\\d{2})"
        "(?:\\.(\\d{1,6}))?(?:(Z)|([+-])(\\d{2}):(\\d{2}))?\\z"));
    const QRegularExpressionMatch match = expression.match(value);
    if (!match.hasMatch())
    {
        return false;
    }
    const QDate date(
        match.captured(1).toInt(),
        match.captured(2).toInt(),
        match.captured(3).toInt());
    const int hour = match.captured(4).toInt();
    const int minute = match.captured(5).toInt();
    const int second = match.captured(6).toInt();
    if (!date.isValid()
        || hour < 0 || hour > 23
        || minute < 0 || minute > 59
        || second < 0 || second > 59)
    {
        return false;
    }
    QString fraction = match.captured(7);
    fraction = fraction.leftJustified(6, QLatin1Char('0'));
    const int microsecond = fraction.isEmpty() ? 0 : fraction.toInt();
    int offsetSeconds = 0;
    if (!match.captured(9).isEmpty())
    {
        const int offsetHour = match.captured(10).toInt();
        const int offsetMinute = match.captured(11).toInt();
        if (offsetHour > 23 || offsetMinute > 59)
        {
            return false;
        }
        offsetSeconds = (offsetHour * 60 + offsetMinute) * 60;
        if (match.captured(9) == QLatin1String("-"))
        {
            offsetSeconds = -offsetSeconds;
        }
    }
    const qint64 localSeconds = date.toJulianDay() * 24 * 60 * 60
        + hour * 60 * 60 + minute * 60 + second;
    *instantMicros = (localSeconds - offsetSeconds) * 1000000 + microsecond;
    return true;
}

bool MergeAccountTimestamps(
    const QString& fieldName,
    const QStringList& candidates,
    QString* merged)
{
    if (merged == nullptr || !IsLegacyAccountTimestampField(fieldName))
    {
        return false;
    }
    merged->clear();
    qint64 selectedInstant = 0;
    bool selected = false;
    for (const QString& candidate : candidates)
    {
        qint64 candidateInstant = 0;
        if (!ParseAccountTimestampInstant(candidate, &candidateInstant))
        {
            return false;
        }
        const int lexicalOrder = selected
            ? QString::compare(candidate, *merged, Qt::CaseSensitive) : 0;
        const bool choose = !selected
            || (fieldName == QLatin1String("CreatedAt")
                && (candidateInstant < selectedInstant
                    || (candidateInstant == selectedInstant && lexicalOrder < 0)))
            || (fieldName == QLatin1String("UpdatedAt")
                && (candidateInstant > selectedInstant
                    || (candidateInstant == selectedInstant && lexicalOrder > 0)));
        if (choose)
        {
            *merged = candidate;
            selectedInstant = candidateInstant;
            selected = true;
        }
    }
    return true;
}

bool ValidateCredentialScrubManifest(const QString& value)
{
    if (value.toUtf8().size() > 1024 * 1024)
    {
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(
        value.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isArray())
    {
        return false;
    }
    static const QSet<QString> requiredKeys = {
        QStringLiteral("path"),
        QStringLiteral("before_sha256"),
        QStringLiteral("after_sha256"),
        QStringLiteral("removed_values")
    };
    static const QRegularExpression sha256Pattern(
        QStringLiteral("\\A[0-9a-f]{64}\\z"));
    for (const QJsonValue& entry : document.array())
    {
        if (!entry.isObject())
        {
            return false;
        }
        const QJsonObject object = entry.toObject();
        const QStringList objectKeys = object.keys();
        const QSet<QString> keys(objectKeys.cbegin(), objectKeys.cend());
        const QJsonValue removedValues = object.value(QStringLiteral("removed_values"));
        if (keys != requiredKeys
            || !object.value(QStringLiteral("path")).isString()
            || !object.value(QStringLiteral("before_sha256")).isString()
            || !object.value(QStringLiteral("after_sha256")).isString()
            || !removedValues.isDouble()
            || !std::isfinite(removedValues.toDouble())
            || removedValues.toDouble() < 0
            || std::floor(removedValues.toDouble()) != removedValues.toDouble()
            || !sha256Pattern.match(
                object.value(QStringLiteral("before_sha256")).toString()).hasMatch()
            || !sha256Pattern.match(
                object.value(QStringLiteral("after_sha256")).toString()).hasMatch())
        {
            return false;
        }
    }
    return true;
}

bool ValidateLegacyAccountValues(LegacyAccount* account)
{
    if (account == nullptr
        || !account->values.contains(QStringLiteral("PasswordHash"))
        || !account->values.contains(QStringLiteral("Role")))
    {
        return false;
    }
    const QString passwordRecord = account->values.value(QStringLiteral("PasswordHash"));
    const QString role = account->values.value(QStringLiteral("Role"));
    if (passwordRecord.size() > 512
        || !CredentialSecurity::IsSupportedPasswordRecord(passwordRecord)
        || (role != QStringLiteral("operator")
            && role != QStringLiteral("engineer")
            && role != QStringLiteral("admin")))
    {
        return false;
    }
    for (const QString& timestampField : {
            QStringLiteral("CreatedAt"), QStringLiteral("UpdatedAt") })
    {
        qint64 ignoredInstant = 0;
        if (account->values.contains(timestampField)
            && !ParseAccountTimestampInstant(
                account->values.value(timestampField), &ignoredInstant))
        {
            return false;
        }
    }
    if (account->values.contains(QStringLiteral("MustChangePassword")))
    {
        bool mustChange = false;
        if (!ParseAuthenticationBool(
                account->values.value(QStringLiteral("MustChangePassword")), &mustChange))
        {
            return false;
        }
        account->values[QStringLiteral("MustChangePassword")] = mustChange
            ? QStringLiteral("1") : QStringLiteral("0");
    }
    return true;
}

bool ReadExistingAccountProfile(
    QSqlDatabase& db,
    const QString& userName,
    bool* exists,
    QMap<QString, QString>* values)
{
    if (exists == nullptr || values == nullptr)
    {
        return false;
    }
    *exists = false;
    values->clear();
    QSqlQuery query(db);
    query.prepare(
        "SELECT key_name, value_text, encrypted FROM settings "
        "WHERE scope_type='account' AND scope_id=? AND module='Profile' "
        "ORDER BY key_name");
    query.addBindValue(userName);
    if (!query.exec())
    {
        return false;
    }
    while (query.next())
    {
        *exists = true;
        const QString rawField = query.value(0).toString();
        QString canonicalField;
        if (!CanonicalCurrentAccountSecurityField(rawField, &canonicalField))
        {
            // Other profile metadata is allowed, but a case-variant of a
            // security field must never be silently ignored or merged.
            continue;
        }
        if (rawField != canonicalField || values->contains(canonicalField))
        {
            return false;
        }
        bool encryptedValid = false;
        const int encrypted = query.value(2).toInt(&encryptedValid);
        const QString storedText = query.value(1).toString();
        if (!encryptedValid
            || !HasPortableAuthenticationStorage(storedText, encrypted))
        {
            return false;
        }
        values->insert(canonicalField, storedText);
    }
    if (values->contains(QStringLiteral("MustChangePassword")))
    {
        bool mustChange = false;
        if (!ParseAuthenticationBool(
                values->value(QStringLiteral("MustChangePassword")), &mustChange))
        {
            return false;
        }
        (*values)[QStringLiteral("MustChangePassword")] = mustChange
            ? QStringLiteral("1") : QStringLiteral("0");
    }
    for (const QString& timestampField : {
            QStringLiteral("CreatedAt"),
            QStringLiteral("UpdatedAt"),
            QStringLiteral("PasswordChangedAt") })
    {
        qint64 ignoredInstant = 0;
        if (values->contains(timestampField)
            && !ParseAccountTimestampInstant(
                values->value(timestampField), &ignoredInstant))
        {
            return false;
        }
    }
    return true;
}

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
        "SELECT scope_type, scope_id, module, key_name, value_text, encrypted FROM settings "
        "WHERE lower(module)='accounts/users' "
        "OR lower(module) GLOB 'accounts/users/*' "
        "ORDER BY module, key_name");
    if (!accountQuery.exec())
    {
        return false;
    }
    while (accountQuery.next())
    {
        if (accountQuery.value(0).toString() != QStringLiteral("global")
            || !accountQuery.value(1).toString().isEmpty())
        {
            return false;
        }
        const QString module = accountQuery.value(2).toString();
        const QString rawKeyName = accountQuery.value(3).toString();
        QString userName;
        QString rawFieldName;
        if (module == QStringLiteral("Accounts/Users"))
        {
            const int separator = rawKeyName.indexOf(QLatin1Char('/'));
            if (separator <= 0 || separator != rawKeyName.lastIndexOf(QLatin1Char('/')))
            {
                return false;
            }
            userName = rawKeyName.left(separator);
            rawFieldName = rawKeyName.mid(separator + 1);
        }
        else if (module.startsWith(QStringLiteral("Accounts/Users/")))
        {
            userName = module.mid(QStringLiteral("Accounts/Users/").size());
            rawFieldName = rawKeyName;
        }
        else
        {
            // The SQL deliberately finds ASCII case variants so they fail
            // closed instead of surviving as an ambiguous orphan record.
            return false;
        }
        QString fieldName;
        if (userName != userName.trimmed()
            || rawFieldName != rawFieldName.trimmed()
            || !IsSafeMigratedAccountName(userName)
            || !CanonicalLegacyAccountField(rawFieldName, &fieldName)
            || rawFieldName != fieldName)
        {
            return false;
        }
        QString decoded;
        if (!DecodeStoredText(
                accountQuery.value(4).toString(),
                accountQuery.value(5).toInt(),
                ProtectionPurpose(QStringLiteral("global"), QString(), module, rawKeyName),
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
        if (account.values.contains(fieldName))
        {
            if (IsLegacyAccountTimestampField(fieldName))
            {
                QString mergedTimestamp;
                if (!MergeAccountTimestamps(
                        fieldName,
                        QStringList{ account.values.value(fieldName), decoded },
                        &mergedTimestamp))
                {
                    return false;
                }
                account.values[fieldName] = mergedTimestamp;
                continue;
            }
            // v4 databases can contain both historical layouts.  Identical
            // security-field duplicates are harmless; conflicting duplicates
            // are not safe to merge because that could pair one role with
            // another password or forced-change state.
            if (account.values.value(fieldName) != decoded)
            {
                return false;
            }
            continue;
        }
        account.values.insert(fieldName, decoded);
    }
    accountQuery.finish();

    for (auto it = legacyAccounts.begin(); it != legacyAccounts.end(); ++it)
    {
        LegacyAccount& source = it.value();
        if (!ValidateLegacyAccountValues(&source))
        {
            return false;
        }
        const QString sourceHash = source.values.value(QStringLiteral("PasswordHash"));
        const QString sourceRole = source.values.value(QStringLiteral("Role"));
        bool destinationExists = false;
        QMap<QString, QString> destinationValues;
        if (!ReadExistingAccountProfile(
                db, source.userName, &destinationExists, &destinationValues))
        {
            return false;
        }
        bool replaceDestination = !destinationExists;
        const QString destinationHash = destinationValues.value(QStringLiteral("PasswordHash"));
        QMap<QString, QString> mergedTimeValues;
        for (const QString& timestampField : {
                QStringLiteral("CreatedAt"),
                QStringLiteral("UpdatedAt") })
        {
            QStringList candidates;
            if (source.values.contains(timestampField))
            {
                candidates.append(source.values.value(timestampField));
            }
            if (destinationValues.contains(timestampField))
            {
                candidates.append(destinationValues.value(timestampField));
            }
            if (!candidates.isEmpty())
            {
                QString mergedTimestamp;
                if (!MergeAccountTimestamps(
                        timestampField, candidates, &mergedTimestamp))
                {
                    return false;
                }
                mergedTimeValues.insert(timestampField, mergedTimestamp);
            }
        }
        const bool replaceKnownBootstrap = destinationExists
            && destinationValues.contains(QStringLiteral("PasswordHash"))
            && destinationValues.contains(QStringLiteral("Role"))
            && destinationValues.value(QStringLiteral("Role")) == QStringLiteral("admin")
            && sourceRole == QStringLiteral("admin")
            && source.userName.compare(QStringLiteral("admin"), Qt::CaseInsensitive) == 0
            && CredentialSecurity::VerifyPasswordRecord(
                source.userName, QStringLiteral("admin"), destinationHash)
                != CredentialSecurity::PasswordVerification::Invalid;
        if (replaceKnownBootstrap)
        {
            // A v4 start may already have inserted the known bootstrap account
            // after missing these legacy rows.  Restore the legacy account as
            // the authoritative value in that one narrow case.
            replaceDestination = true;
        }
        else if (destinationExists)
        {
            if (!destinationValues.contains(QStringLiteral("PasswordHash"))
                || !destinationValues.contains(QStringLiteral("Role"))
                || !CredentialSecurity::IsSupportedPasswordRecord(destinationHash)
                || (destinationValues.value(QStringLiteral("Role")) != QStringLiteral("operator")
                    && destinationValues.value(QStringLiteral("Role")) != QStringLiteral("engineer")
                    && destinationValues.value(QStringLiteral("Role")) != QStringLiteral("admin")))
            {
                return false;
            }
            for (auto value = source.values.cbegin(); value != source.values.cend(); ++value)
            {
                if (IsLegacyAccountTimestampField(value.key()))
                {
                    continue;
                }
                if (!destinationValues.contains(value.key())
                    || destinationValues.value(value.key()) != value.value())
                {
                    // Never guess whether the v4 source or a partially-created
                    // account/Profile target is authoritative.
                    return false;
                }
            }
        }

        if (replaceDestination)
        {
            for (auto timestamp = mergedTimeValues.cbegin();
                 timestamp != mergedTimeValues.cend(); ++timestamp)
            {
                source.values[timestamp.key()] = timestamp.value();
            }
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
            const bool isKnownBootstrap = source.userName.compare(
                    QStringLiteral("admin"), Qt::CaseInsensitive) == 0
                && CredentialSecurity::VerifyPasswordRecord(
                    source.userName, QStringLiteral("admin"), sourceHash)
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
                    source.values.contains(QStringLiteral("MustChangePassword"))
                        ? source.values.value(QStringLiteral("MustChangePassword"))
                        : (isKnownBootstrap ? QStringLiteral("1") : QStringLiteral("0"))))
            {
                return false;
            }
        }
        else
        {
            for (auto timestamp = mergedTimeValues.cbegin();
                 timestamp != mergedTimeValues.cend(); ++timestamp)
            {
                if (destinationValues.value(timestamp.key()) == timestamp.value())
                {
                    continue;
                }
                ScopedSettingIdentity timestampIdentity;
                timestampIdentity.valid = true;
                timestampIdentity.scopeType = QStringLiteral("account");
                timestampIdentity.scopeId = source.userName;
                timestampIdentity.module = QStringLiteral("Profile");
                timestampIdentity.keyName = timestamp.key();
                timestampIdentity.valueType = QStringLiteral("datetime");
                if (!WriteScopedSettingValue(
                        db, timestampIdentity, timestamp.value()))
                {
                    return false;
                }
            }
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

bool NormalizeExistingAccountProfiles(
    QSqlDatabase& db,
    int* accountCount,
    int* administratorCount)
{
    if (accountCount == nullptr || administratorCount == nullptr)
    {
        return false;
    }
    *accountCount = 0;
    *administratorCount = 0;
    QSqlQuery accountQuery(db);
    if (!accountQuery.exec(QStringLiteral(
            "SELECT DISTINCT scope_type, scope_id, module FROM settings "
            "WHERE lower(trim(scope_type))='account' "
            "AND lower(trim(module))='profile' "
            "ORDER BY scope_id COLLATE NOCASE")))
    {
        return false;
    }
    QStringList accountIds;
    QMap<QString, QString> foldedAccountIds;
    while (accountQuery.next())
    {
        const QString rawAccountId = accountQuery.value(1).toString();
        const QString accountId = rawAccountId.trimmed();
        const QString folded = accountId.toCaseFolded();
        if (accountQuery.value(0).toString() != QStringLiteral("account")
            || accountQuery.value(2).toString() != QStringLiteral("Profile")
            || rawAccountId != accountId
            || !IsSafeMigratedAccountName(accountId)
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
            "ORDER BY key_name"));
        profileQuery.addBindValue(accountId);
        if (!profileQuery.exec())
        {
            return false;
        }
        QMap<QString, QPair<QString, int>> storedValues;
        while (profileQuery.next())
        {
            const QString rawField = profileQuery.value(0).toString();
            QString canonicalField;
            if (!CanonicalCurrentAccountSecurityField(rawField, &canonicalField))
            {
                continue;
            }
            if (rawField != canonicalField)
            {
                return false;
            }
            if (storedValues.contains(canonicalField))
            {
                return false;
            }
            bool encryptedValid = false;
            const int encrypted = profileQuery.value(2).toInt(&encryptedValid);
            const QString storedText = profileQuery.value(1).toString();
            if (!encryptedValid
                || !HasPortableAuthenticationStorage(storedText, encrypted))
            {
                return false;
            }
            storedValues.insert(
                canonicalField, qMakePair(storedText, encrypted));
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
        if (role == QStringLiteral("admin"))
        {
            ++(*administratorCount);
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

        QMap<QString, QString> normalizedTimestamps;
        for (const QString& timestampField : {
                QStringLiteral("CreatedAt"),
                QStringLiteral("UpdatedAt"),
                QStringLiteral("PasswordChangedAt") })
        {
            if (!storedValues.contains(timestampField))
            {
                continue;
            }
            QString timestamp;
            qint64 ignoredInstant = 0;
            if (!decode(timestampField, &timestamp)
                || !ParseAccountTimestampInstant(timestamp, &ignoredInstant))
            {
                return false;
            }
            normalizedTimestamps.insert(timestampField, timestamp);
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
        for (auto timestamp = normalizedTimestamps.cbegin();
             timestamp != normalizedTimestamps.cend(); ++timestamp)
        {
            ScopedSettingIdentity identity;
            identity.valid = true;
            identity.scopeType = QStringLiteral("account");
            identity.scopeId = accountId;
            identity.module = QStringLiteral("Profile");
            identity.keyName = timestamp.key();
            identity.valueType = QStringLiteral("datetime");
            identity.sensitive = timestamp.key() == QLatin1String("PasswordChangedAt");
            if (!WriteScopedSettingValue(db, identity, timestamp.value()))
            {
                return false;
            }
        }
    }
    *accountCount = accountIds.size();
    return true;
}

bool CanonicalLegacyLoginField(const QString& fieldName, QString* canonical)
{
    if (canonical == nullptr)
    {
        return false;
    }
    for (const QString& supported : {
            QStringLiteral("UserName"),
            QStringLiteral("AccountHistory"),
            QStringLiteral("RememberPassword"),
            QStringLiteral("AutoLogin"),
            QStringLiteral("PasswordBase64") })
    {
        if (fieldName.compare(supported, Qt::CaseInsensitive) == 0)
        {
            *canonical = supported;
            return true;
        }
    }
    return false;
}

bool MigrateLegacyLoginState(QSqlDatabase& db)
{
    QSqlQuery loginQuery(db);
    loginQuery.prepare(
        "SELECT scope_type, scope_id, module, key_name, value_text, encrypted FROM settings "
        "WHERE lower(module) IN ('loginstate/general', 'loginstate/settings') OR ("
        "lower(module)='loginstate' AND lower(key_name) IN ("
        "'username', 'accounthistory', 'rememberpassword', 'autologin', 'passwordbase64')) "
        "ORDER BY module, key_name");
    if (!loginQuery.exec())
    {
        return false;
    }
    QMap<QString, QString> loginValues;
    const auto mergeLoginValue = [&loginValues](
                                     const QString& keyName,
                                     const QString& value)
        {
            if (loginValues.contains(keyName))
            {
                return loginValues.value(keyName) == value;
            }
            loginValues.insert(keyName, value);
            return true;
        };
    while (loginQuery.next())
    {
        const QString scopeType = loginQuery.value(0).toString();
        const QString scopeId = loginQuery.value(1).toString();
        const QString module = loginQuery.value(2).toString();
        const QString lowerModule = module.toLower();
        const bool legacyGeneral = lowerModule == QLatin1String("loginstate/general");
        const bool legacySettings = lowerModule == QLatin1String("loginstate/settings");
        const QString canonicalModule = legacyGeneral
            ? QStringLiteral("LoginState/General")
            : legacySettings
                ? QStringLiteral("LoginState/Settings")
                : QStringLiteral("LoginState");
        if (scopeType != QStringLiteral("global")
            || !scopeId.isEmpty()
            || module != canonicalModule)
        {
            return false;
        }
        const QString keyName = loginQuery.value(3).toString();
        QString canonicalField;
        if (!CanonicalLegacyLoginField(keyName, &canonicalField)
            || keyName != canonicalField)
        {
            return false;
        }
        if (keyName != QStringLiteral("UserName")
            && keyName != QStringLiteral("AccountHistory"))
        {
            continue;
        }
        QString decoded;
        if (!DecodeStoredText(
                loginQuery.value(4).toString(),
                loginQuery.value(5).toInt(),
                ProtectionPurpose(scopeType, scopeId, module, keyName),
                &decoded))
        {
            return false;
        }
        if (!mergeLoginValue(canonicalField, decoded))
        {
            return false;
        }
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
            "module='Accounts/Users' OR module GLOB 'Accounts/Users/*' OR "
            "lower(module)='loginstate/general' OR "
            "lower(module)='loginstate/settings' OR "
            "lower(module) GLOB 'loginstate/savedpasswords*' OR "
            "lower(module) GLOB 'loginstate/rememberedcredentials*' OR "
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
        const bool requiresDpapi = RequiresDpapiProtection(
            scopeType, module, keyName, sensitive);
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

bool MigrateLegacyAuthenticationSettings(QSqlDatabase& db, bool authenticationInitialized)
{
    int accountCount = 0;
    int administratorCount = 0;
    if (!MigrateLegacyAccounts(db)
        || !NormalizeExistingAccountProfiles(db, &accountCount, &administratorCount)
        || (!authenticationInitialized && accountCount != 0)
        || (authenticationInitialized && administratorCount == 0)
        || !MigrateLegacyLoginState(db)
        || !UpgradeRecoverableSensitiveSettings(db))
    {
        return false;
    }
    QSqlQuery orphanQuery(db);
    if (!orphanQuery.exec(QStringLiteral(
            "SELECT COUNT(*) FROM settings WHERE "
            "lower(module)='accounts/users' OR "
            "lower(module) GLOB 'accounts/users/*' OR "
            "lower(module)='loginstate/general' OR "
            "lower(module)='loginstate/settings' OR "
            "lower(module) GLOB 'loginstate/savedpasswords*' OR "
            "lower(module) GLOB 'loginstate/rememberedcredentials*' OR "
            "lower(key_name)='passwordbase64'"))
        || !orphanQuery.next()
        || orphanQuery.value(0).toInt() != 0)
    {
        return false;
    }
    return SetMetaValue(
            db,
            QStringLiteral("auth_semantic_version"),
            QString::fromLatin1(kAuthenticationSemanticVersion));
}

bool SetCurrentSchemaMetadata(QSqlDatabase& db)
{
    return ExecSql(db, QStringLiteral(
            "INSERT OR IGNORE INTO meta(key, value) VALUES("
            "'created_at', strftime('%Y-%m-%dT%H:%M:%S', 'now'))"))
        && !MetaValue(db, QStringLiteral("created_at")).isEmpty()
        && SetMetaValue(db, QStringLiteral("schema_version"), QString::fromLatin1(kSchemaVersion))
        && SetMetaValue(db, QStringLiteral("encrypt_new_values"), QStringLiteral("0"))
        && SetMetaValue(db, QStringLiteral("sensitive_protection"), QStringLiteral("dpapi-current-user-v1"));
}

bool CanonicalCurrentAccountSecurityField(
    const QString& fieldName,
    QString* canonical)
{
    if (CanonicalLegacyAccountField(fieldName, canonical))
    {
        return true;
    }
    if (canonical != nullptr
        && fieldName.compare(
            QStringLiteral("PasswordChangedAt"), Qt::CaseInsensitive) == 0)
    {
        *canonical = QStringLiteral("PasswordChangedAt");
        return true;
    }
    return false;
}

bool ValidateCurrentAuthenticationIntegrity(
    QSqlDatabase& db,
    bool authenticationInitialized)
{
    const auto reject = [](const char* reason)
        {
            qCritical() << "Current authentication integrity check failed:" << reason;
            return false;
        };
    QSqlQuery integrityQuery(db);
    if (!integrityQuery.exec(QStringLiteral("PRAGMA integrity_check"))
        || !integrityQuery.next()
        || integrityQuery.value(0).toString() != QLatin1String("ok")
        || integrityQuery.next())
    {
        return reject("sqlite-integrity");
    }
    for (const QString& legacyTable : {
            QStringLiteral("ini_values"),
            QStringLiteral("text_files"),
            QStringLiteral("app_settings") })
    {
        if (TableExists(db, legacyTable))
        {
            return reject("legacy-table");
        }
    }

    QSqlQuery orphanQuery(db);
    if (!orphanQuery.exec(QStringLiteral(
            "SELECT COUNT(*) FROM settings WHERE "
            "lower(module)='accounts/users' OR "
            "lower(module) GLOB 'accounts/users/*' OR "
            "lower(module)='loginstate/general' OR "
            "lower(module)='loginstate/settings' OR "
            "(lower(module)='loginstate' AND module<>'LoginState') OR "
            "(module='LoginState' AND lower(key_name) IN ("
            "'username', 'accounthistory', 'rememberpassword', 'autologin', 'passwordbase64') "
            "AND (key_name NOT IN ("
            "'UserName', 'AccountHistory', 'RememberPassword', 'AutoLogin', 'PasswordBase64') "
            "OR scope_type<>'global' OR scope_id<>'')) OR "
            "(lower(module) GLOB 'loginstate/savedpasswords*' "
            "AND NOT (module='LoginState/SavedPasswords' "
            "AND scope_type='global' AND scope_id='')) OR "
            "(lower(module) GLOB 'loginstate/rememberedcredentials*' "
            "AND NOT (module='LoginState/RememberedCredentials' "
            "AND scope_type='global' AND scope_id='')) OR "
            "lower(key_name)='passwordbase64'"))
        || !orphanQuery.next()
        || orphanQuery.value(0).toInt() != 0)
    {
        return reject("legacy-authentication-orphan");
    }

    QSqlQuery accountQuery(db);
    if (!accountQuery.exec(QStringLiteral(
            "SELECT DISTINCT scope_type, scope_id, module FROM settings "
            "WHERE lower(trim(scope_type))='account' "
            "AND lower(trim(module))='profile' "
            "ORDER BY scope_id COLLATE NOCASE")))
    {
        return reject("account-profile-query");
    }
    QStringList accountIds;
    QMap<QString, QString> foldedAccountIds;
    while (accountQuery.next())
    {
        const QString scopeType = accountQuery.value(0).toString();
        const QString accountId = accountQuery.value(1).toString();
        const QString module = accountQuery.value(2).toString();
        const QString foldedAccountId = accountId.toCaseFolded();
        if (scopeType != QLatin1String("account")
            || module != QLatin1String("Profile")
            || accountId != accountId.trimmed()
            || !IsSafeMigratedAccountName(accountId)
            || (foldedAccountIds.contains(foldedAccountId)
                && foldedAccountIds.value(foldedAccountId) != accountId))
        {
            return reject("account-profile-shadow-or-id");
        }
        foldedAccountIds.insert(foldedAccountId, accountId);
        if (!accountIds.contains(accountId))
        {
            accountIds.append(accountId);
        }
    }
    accountQuery.finish();

    int administratorCount = 0;
    for (const QString& accountId : accountIds)
    {
        QSqlQuery profileQuery(db);
        profileQuery.prepare(QStringLiteral(
            "SELECT key_name, value_text, encrypted FROM settings "
            "WHERE scope_type='account' AND scope_id=? AND module='Profile' "
            "ORDER BY key_name"));
        profileQuery.addBindValue(accountId);
        if (!profileQuery.exec())
        {
            return reject("account-profile-read");
        }
        QMap<QString, QPair<QString, int>> storedValues;
        while (profileQuery.next())
        {
            const QString rawField = profileQuery.value(0).toString();
            QString canonicalField;
            if (!CanonicalCurrentAccountSecurityField(rawField, &canonicalField))
            {
                continue;
            }
            if (rawField != canonicalField)
            {
                return reject("account-field-case");
            }
            if (storedValues.contains(canonicalField))
            {
                return reject("account-field-duplicate");
            }
            bool encryptedValid = false;
            const int encrypted = profileQuery.value(2).toInt(&encryptedValid);
            if (!encryptedValid)
            {
                return reject("account-field-encrypted-marker");
            }
            storedValues.insert(
                canonicalField,
                qMakePair(profileQuery.value(1).toString(), encrypted));
        }
        profileQuery.finish();
        for (const QString& requiredField : {
                QStringLiteral("PasswordHash"),
                QStringLiteral("Role"),
                QStringLiteral("MustChangePassword") })
        {
            if (!storedValues.contains(requiredField))
            {
                return reject("account-required-fields");
            }
        }
        for (const QString& portableField : {
                QStringLiteral("PasswordHash"),
                QStringLiteral("Role"),
                QStringLiteral("MustChangePassword"),
                QStringLiteral("PasswordChangedAt"),
                QStringLiteral("CreatedAt"),
                QStringLiteral("UpdatedAt") })
        {
            if (!storedValues.contains(portableField))
            {
                continue;
            }
            const auto stored = storedValues.value(portableField);
            if (!HasPortableAuthenticationStorage(stored.first, stored.second))
            {
                return reject("portable-account-field-protection");
            }
        }
        auto decode = [&storedValues, &accountId](
                          const QString& fieldName, QString* value)
            {
                const auto stored = storedValues.value(fieldName);
                return DecodeStoredText(
                    stored.first,
                    stored.second,
                    ProtectionPurpose(
                        QStringLiteral("account"), accountId,
                        QStringLiteral("Profile"), fieldName),
                    value);
            };
        QString passwordRecord;
        QString role;
        QString mustChangeText;
        bool mustChange = false;
        if (!decode(QStringLiteral("PasswordHash"), &passwordRecord)
            || !decode(QStringLiteral("Role"), &role)
            || !decode(QStringLiteral("MustChangePassword"), &mustChangeText)
            || !CredentialSecurity::IsSupportedPasswordRecord(passwordRecord)
            || !IsAuthenticationRole(role)
            || !ParseAuthenticationBool(mustChangeText, &mustChange))
        {
            return reject("account-security-values");
        }
        if (role == QLatin1String("admin"))
        {
            ++administratorCount;
        }
        for (const QString& timestampField : {
                QStringLiteral("CreatedAt"),
                QStringLiteral("UpdatedAt"),
                QStringLiteral("PasswordChangedAt") })
        {
            if (!storedValues.contains(timestampField))
            {
                continue;
            }
            QString timestamp;
            qint64 ignoredInstant = 0;
            if (!decode(timestampField, &timestamp)
                || !ParseAccountTimestampInstant(timestamp, &ignoredInstant))
            {
                return reject("account-timestamp");
            }
        }
    }
    if ((!authenticationInitialized && !accountIds.isEmpty())
        || (authenticationInitialized
            && (accountIds.isEmpty() || administratorCount == 0)))
    {
        return reject("administrator-invariant");
    }

    QSqlQuery sensitiveQuery(db);
    if (!sensitiveQuery.exec(QStringLiteral(
            "SELECT scope_type, scope_id, module, key_name, value_text, sensitive, encrypted, value_type "
            "FROM settings")))
    {
        return reject("sensitive-setting-query");
    }
    while (sensitiveQuery.next())
    {
        bool sensitiveValid = false;
        bool encryptedValid = false;
        const int sensitiveMarker = sensitiveQuery.value(5).toInt(&sensitiveValid);
        const int encrypted = sensitiveQuery.value(6).toInt(&encryptedValid);
        if (!sensitiveValid || !encryptedValid)
        {
            return reject("sensitive-setting-marker");
        }
        const QString scopeType = sensitiveQuery.value(0).toString();
        const QString scopeId = sensitiveQuery.value(1).toString();
        const QString module = sensitiveQuery.value(2).toString();
        const QString keyName = sensitiveQuery.value(3).toString();
        const QString storedText = sensitiveQuery.value(4).toString();
        const QString valueType = sensitiveQuery.value(7).toString();
        const bool canonicalRememberedCredential =
            scopeType == QLatin1String("global")
            && scopeId.isEmpty()
            && (module == QLatin1String("LoginState/RememberedCredentials")
                || module == QLatin1String("LoginState/SavedPasswords"));
        if (canonicalRememberedCredential)
        {
            QString decodedCredential;
            if (!IsSafeMigratedAccountName(keyName)
                || valueType != QLatin1String("string")
                || sensitiveMarker != 1
                || encrypted != 1
                || !CredentialSecurity::IsCurrentUserProtected(storedText)
                || !DecodeStoredText(
                    storedText, encrypted,
                    ProtectionPurpose(scopeType, scopeId, module, keyName),
                    &decodedCredential)
                || decodedCredential.isEmpty())
            {
                return reject("remembered-credential-shape");
            }
        }
        const bool semanticSensitive = sensitiveMarker != 0
            || IsSensitiveSettingKey(module)
            || IsSensitiveSettingKey(keyName);
        const bool requiresDpapi = RequiresDpapiProtection(
            scopeType, module, keyName, semanticSensitive);
        const bool canonicalLoginPreference =
            scopeType == QLatin1String("global")
            && scopeId.isEmpty()
            && module == QLatin1String("LoginState")
            && (keyName == QLatin1String("RememberPassword")
                || keyName == QLatin1String("AutoLogin"));
        if (canonicalLoginPreference)
        {
            QString decodedPreference;
            if (valueType != QLatin1String("bool")
                || sensitiveMarker != 1
                || encrypted != 1
                || !CredentialSecurity::IsCurrentUserProtected(storedText)
                || !DecodeStoredText(
                    storedText,
                    encrypted,
                    ProtectionPurpose(scopeType, scopeId, module, keyName),
                    &decodedPreference)
                || (decodedPreference != QLatin1String("0")
                    && decodedPreference != QLatin1String("1")))
            {
                return reject("login-preference-shape");
            }
        }
        if (requiresDpapi
            && !CredentialSecurity::IsCurrentUserProtected(storedText))
        {
            return reject("sensitive-setting-not-dpapi");
        }
        if (CredentialSecurity::IsCurrentUserProtected(storedText)
            || encrypted != 0)
        {
            QString decoded;
            if (!DecodeStoredText(
                    storedText,
                    encrypted,
                    ProtectionPurpose(scopeType, scopeId, module, keyName),
                    &decoded))
            {
                return reject("protected-setting-unreadable");
            }
        }
    }
    return true;
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
    if (HasInstallerTransactionRecord(db.databaseName()))
    {
        db.close();
        qCritical() << "ConfigStore schema access is blocked by an unresolved installer transaction.";
        return false;
    }

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
    bool credentialScrubManifestExists = false;
    QString credentialScrubManifest;
    if (hasMeta)
    {
        QSqlQuery manifestQuery(db);
        manifestQuery.prepare(QStringLiteral(
            "SELECT value FROM meta WHERE key='legacy_credential_scrub_manifest'"));
        if (!manifestQuery.exec())
        {
            return false;
        }
        if (manifestQuery.next())
        {
            credentialScrubManifestExists = true;
            credentialScrubManifest = manifestQuery.value(0).toString();
            if (manifestQuery.next())
            {
                return false;
            }
        }
    }
    if (!credentialScrubState.isEmpty()
        && credentialScrubState != QLatin1String("complete"))
    {
        qCritical() << "Legacy credential scrub is incomplete or invalid; "
                       "rerun ConfigMigrate before starting the application.";
        return false;
    }
    if ((credentialScrubState == QLatin1String("complete")
            && (!credentialScrubManifestExists
                || !ValidateCredentialScrubManifest(credentialScrubManifest)))
        || (credentialScrubState.isEmpty() && credentialScrubManifestExists))
    {
        qCritical() << "Legacy credential scrub manifest provenance is invalid; "
                       "rerun ConfigMigrate before starting the application.";
        return false;
    }

    const QString authenticationSemanticVersion = MetaValue(
        db, QStringLiteral("auth_semantic_version"));
    if (existingVersion == QString::fromLatin1(kSchemaVersion)
        && authenticationSemanticVersion != QLatin1String("1")
        && authenticationSemanticVersion != QLatin1String("2")
        && authenticationSemanticVersion != QString::fromLatin1(kAuthenticationSemanticVersion))
    {
        qCritical() << "Current schema has an unsupported or missing authentication semantic version";
        return false;
    }
    const bool needsMigration = existingVersion != QString::fromLatin1(kSchemaVersion)
        || authenticationSemanticVersion != QString::fromLatin1(kAuthenticationSemanticVersion);
    if (!needsMigration)
    {
        const QString authenticationState = MetaValue(
            db, QLatin1String("auth_initialized"));
        const QString encryptNewValues = MetaValue(
            db, QLatin1String("encrypt_new_values"));
        if ((authenticationState != QLatin1String("0")
                && authenticationState != QLatin1String("1"))
            || (encryptNewValues != QLatin1String("0")
                && encryptNewValues != QLatin1String("1"))
            || MetaValue(db, QStringLiteral("sensitive_protection"))
                != QLatin1String("dpapi-current-user-v1"))
        {
            qCritical() << "Current authentication metadata check failed";
            return false;
        }
        QSqlQuery createdAtQuery(db);
        createdAtQuery.prepare(QStringLiteral(
            "SELECT value FROM meta WHERE key='created_at'"));
        if (!createdAtQuery.exec())
        {
            return false;
        }
        if (createdAtQuery.next()
            && (createdAtQuery.value(0).isNull()
                || createdAtQuery.value(0).toString().trimmed().isEmpty()))
        {
            qCritical() << "Current schema created_at metadata is empty";
            return false;
        }
        const bool authenticationIntegrityValid = ValidateCurrentAuthenticationIntegrity(
            db, authenticationState == QLatin1String("1"));
        if (HasInstallerTransactionRecord(db.databaseName()))
        {
            db.close();
            return false;
        }
        return authenticationIntegrityValid;
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
    if (HasInstallerTransactionRecord(db.databaseName()))
    {
        db.close();
        return false;
    }
    if (!db.transaction())
    {
        return false;
    }
    const bool authenticationInitialized = authenticationInitializedValue == QLatin1String("1");
    const bool migrationValid = CreateCurrentTables(db)
        && HasCurrentSettingsColumns(db)
        && MigrateLegacyAuthenticationSettings(db, authenticationInitialized)
        && SetMetaValue(
            db,
            QLatin1String("auth_initialized"),
            authenticationInitializedValue)
        && SetCurrentSchemaMetadata(db)
        && ValidateCurrentAuthenticationIntegrity(db, authenticationInitialized);
    if (!migrationValid)
    {
        db.rollback();
        return false;
    }
    if (HasInstallerTransactionRecord(db.databaseName()))
    {
        db.rollback();
        db.close();
        return false;
    }
    if (!db.commit())
    {
        db.rollback();
        return false;
    }
    if (HasInstallerTransactionRecord(db.databaseName()))
    {
        db.close();
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


bool ConfigDatabase::IsAvailable()
{
    QSqlDatabase db = OpenDatabase();
    return db.isValid() && db.isOpen();
}

bool ConfigDatabase::HasScopedModule(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName)
{
    QSqlDatabase db = OpenDatabase();
    const QString baseModule = NormalizeSection(moduleName);
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(scopeType, scopeId, baseModule))
    {
        return false;
    }

    const QString modulePrefix = baseModule + QStringLiteral("/");
    QSqlQuery query(db);
    query.prepare("SELECT 1 FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?) LIMIT 1");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(baseModule);
    query.addBindValue(modulePrefix.size());
    query.addBindValue(modulePrefix);
    return query.exec() && query.next();
}

bool ConfigDatabase::CopyScopedModule(
    const QString& sourceScopeType,
    const QString& sourceScopeId,
    const QString& sourceModuleName,
    const QString& targetScopeType,
    const QString& targetScopeId,
    const QString& targetModuleName,
    bool overwriteExisting)
{
    QSqlDatabase db = OpenDatabase();
    const QString sourceModule = NormalizeSection(sourceModuleName);
    const QString targetModule = NormalizeSection(targetModuleName);
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(sourceScopeType, sourceScopeId, sourceModule)
        || !IsDatabaseNativeScopeIdentity(targetScopeType, targetScopeId, targetModule)
        || !HasScopedModule(sourceScopeType, sourceScopeId, sourceModule))
    {
        return false;
    }

    if (!db.transaction())
    {
        return false;
    }
    const auto rollback = [&db]() { db.rollback(); return false; };
    if (overwriteExisting)
    {
        QSqlQuery deleteQuery(db);
        const QString targetPrefix = targetModule + QStringLiteral("/");
        deleteQuery.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?)");
        deleteQuery.addBindValue(NormalizeSection(targetScopeType).toLower());
        deleteQuery.addBindValue(NormalizeScopeId(targetScopeId));
        deleteQuery.addBindValue(targetModule);
        deleteQuery.addBindValue(targetPrefix.size());
        deleteQuery.addBindValue(targetPrefix);
        if (!deleteQuery.exec())
        {
            return rollback();
        }
    }

    QSqlQuery sourceQuery(db);
    const QString sourcePrefix = sourceModule + QStringLiteral("/");
    sourceQuery.prepare(
        "SELECT module, key_name, value_text, value_type, sensitive, encrypted "
        "FROM settings WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?)");
    sourceQuery.addBindValue(NormalizeSection(sourceScopeType).toLower());
    sourceQuery.addBindValue(NormalizeScopeId(sourceScopeId));
    sourceQuery.addBindValue(sourceModule);
    sourceQuery.addBindValue(sourcePrefix.size());
    sourceQuery.addBindValue(sourcePrefix);
    if (!sourceQuery.exec())
    {
        return rollback();
    }

    while (sourceQuery.next())
    {
        const QString sourceModuleValue = NormalizeSection(sourceQuery.value(0).toString());
        const QString suffix = sourceModuleValue == sourceModule
            ? QString()
            : sourceModuleValue.mid(sourceModule.size());
        const QString sourceKey = NormalizeSourceKey(sourceQuery.value(1).toString());
        const QString targetModuleValue = NormalizeSection(targetModule + suffix);

        if (!overwriteExisting)
        {
            QSqlQuery existingQuery(db);
            existingQuery.prepare(
                "SELECT 1 FROM settings WHERE scope_type=? AND scope_id=? AND module=? AND key_name=? LIMIT 1");
            existingQuery.addBindValue(NormalizeSection(targetScopeType).toLower());
            existingQuery.addBindValue(NormalizeScopeId(targetScopeId));
            existingQuery.addBindValue(targetModuleValue);
            existingQuery.addBindValue(sourceKey);
            if (!existingQuery.exec())
            {
                return rollback();
            }
            if (existingQuery.next())
            {
                continue;
            }
        }

        QString plainText;
        if (!DecodeStoredText(
                sourceQuery.value(2).toString(),
                sourceQuery.value(5).toInt(),
                ProtectionPurpose(
                    sourceScopeType, sourceScopeId, sourceModuleValue, sourceKey),
                &plainText))
        {
            return rollback();
        }

        ScopedSettingIdentity targetIdentity;
        targetIdentity.scopeType = NormalizeSection(targetScopeType).toLower();
        targetIdentity.scopeId = NormalizeScopeId(targetScopeId);
        targetIdentity.module = targetModuleValue;
        targetIdentity.keyName = sourceKey;
        targetIdentity.valueType = sourceQuery.value(3).toString();
        targetIdentity.sensitive = sourceQuery.value(4).toInt() != 0;
        targetIdentity.valid = IsDatabaseNativeSettingIdentity(
            targetIdentity.scopeType,
            targetIdentity.scopeId,
            targetIdentity.module,
            targetIdentity.keyName);
        // DPAPI protection is purpose-bound to scope/module/key.  A raw
        // ciphertext copy would become unreadable after changing from the
        // robot-type template scope to an actual control-unit scope.
        if (!targetIdentity.valid
            || !WriteScopedSettingValue(db, targetIdentity, plainText))
        {
            return rollback();
        }
    }
    return db.commit();
}

bool ConfigDatabase::ReadScopedModuleSnapshot(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    QMap<QString, QMap<QString, QString>>& sectionValues,
    QString* error)
{
    sectionValues.clear();
    if (error != nullptr)
    {
        error->clear();
    }
    QSqlDatabase db = OpenDatabase();
    const QString baseModule = NormalizeSection(moduleName);
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(scopeType, scopeId, baseModule))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("配置数据库位置无效或数据库不可用。");
        }
        return false;
    }

    const QString modulePrefix = baseModule + QStringLiteral("/");
    QSqlQuery query(db);
    query.prepare(
        "SELECT module, key_name, value_text, encrypted FROM settings "
        "WHERE scope_type=? AND scope_id=? AND (module=? OR substr(module, 1, ?)=?) "
        "ORDER BY module COLLATE NOCASE, key_name COLLATE NOCASE");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(baseModule);
    query.addBindValue(modulePrefix.size());
    query.addBindValue(modulePrefix);
    if (!query.exec())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("读取配置模块快照失败：%1").arg(query.lastError().text());
        }
        return false;
    }

    while (query.next())
    {
        const QString module = NormalizeSection(query.value(0).toString());
        if (module == baseModule)
        {
            continue;
        }
        const QString sectionName = module.mid(modulePrefix.size());
        const QString keyName = NormalizeSourceKey(query.value(1).toString());
        QString plainText;
        if (sectionName.isEmpty() || keyName.isEmpty()
            || !DecodeStoredText(
                query.value(2).toString(),
                query.value(3).toInt(),
                ProtectionPurpose(scopeType, scopeId, module, keyName),
                &plainText))
        {
            sectionValues.clear();
            if (error != nullptr)
            {
                *error = QStringLiteral("解码配置模块失败：[%1] %2").arg(sectionName, keyName);
            }
            return false;
        }
        sectionValues[sectionName].insert(keyName, plainText);
    }
    if (query.lastError().isValid())
    {
        sectionValues.clear();
        if (error != nullptr)
        {
            *error = QStringLiteral("遍历配置模块失败：%1").arg(query.lastError().text());
        }
        return false;
    }
    return true;
}

bool ConfigDatabase::RemoveScopedModuleSection(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& sectionName)
{
    QSqlDatabase db = OpenDatabase();
    const QString module = NormalizeSection(moduleName + QStringLiteral("/") + sectionName);
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(scopeType, scopeId, module))
    {
        return false;
    }
    QSqlQuery query(db);
    query.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND module=?");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(module);
    return query.exec();
}

bool ConfigDatabase::ReplaceScopedModuleSectionsAtomically(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QMap<QString, QMap<QString, QString>>& sectionValues,
    const QStringList& removeSections,
    QString* error)
{
    if (error != nullptr)
    {
        error->clear();
    }
    QSqlDatabase db = OpenDatabase();
    const QString normalizedScope = NormalizeSection(scopeType).toLower();
    const QString normalizedScopeId = NormalizeScopeId(scopeId);
    const QString baseModule = NormalizeSection(moduleName);
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(normalizedScope, normalizedScopeId, baseModule))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("配置数据库位置无效或数据库不可用。");
        }
        return false;
    }
    if (!db.transaction())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法开始配置模块原子更新：%1").arg(db.lastError().text());
        }
        return false;
    }
    const auto fail = [&db, error](const QString& message)
        {
            db.rollback();
            if (error != nullptr)
            {
                *error = message;
            }
            return false;
        };

    QSet<QString> removedModules;
    for (const QString& sectionName : removeSections)
    {
        if (!IsDatabaseNativeIdentityPart(sectionName, true))
        {
            return fail(QStringLiteral("配置分区身份不是数据库原生格式：%1").arg(sectionName));
        }
        const QString module = NormalizeSection(baseModule + QStringLiteral("/") + sectionName);
        if (module.isEmpty())
        {
            return fail(QStringLiteral("配置分区无效：%1").arg(sectionName));
        }
        if (removedModules.contains(module))
        {
            continue;
        }
        QSqlQuery deleteQuery(db);
        deleteQuery.prepare("DELETE FROM settings WHERE scope_type=? AND scope_id=? AND module=?");
        deleteQuery.addBindValue(normalizedScope);
        deleteQuery.addBindValue(normalizedScopeId);
        deleteQuery.addBindValue(module);
        if (!deleteQuery.exec())
        {
            return fail(QStringLiteral("删除旧配置分区失败：%1；%2")
                .arg(sectionName, deleteQuery.lastError().text()));
        }
        removedModules.insert(module);
    }

    for (auto sectionIt = sectionValues.cbegin(); sectionIt != sectionValues.cend(); ++sectionIt)
    {
        if (!IsDatabaseNativeIdentityPart(sectionIt.key(), true))
        {
            return fail(QStringLiteral("配置分区身份不是数据库原生格式：%1").arg(sectionIt.key()));
        }
        const QString module = NormalizeSection(baseModule + QStringLiteral("/") + sectionIt.key());
        for (auto valueIt = sectionIt.value().cbegin(); valueIt != sectionIt.value().cend(); ++valueIt)
        {
            ScopedSettingIdentity identity;
            identity.scopeType = normalizedScope;
            identity.scopeId = normalizedScopeId;
            identity.module = module;
            identity.keyName = valueIt.key();
            identity.valueType = QStringLiteral("string");
            identity.sensitive = IsSensitiveSettingKey(module) || IsSensitiveSettingKey(valueIt.key());
            identity.valid = IsDatabaseNativeSettingIdentity(
                normalizedScope, normalizedScopeId, module, valueIt.key());
            if (!identity.valid || !WriteScopedSettingValue(db, identity, valueIt.value()))
            {
                return fail(QStringLiteral("写入配置失败：[%1] %2")
                    .arg(sectionIt.key(), valueIt.key()));
            }
        }
    }
    if (!db.commit())
    {
        return fail(QStringLiteral("提交配置模块原子更新失败：%1").arg(db.lastError().text()));
    }
    return true;
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
    identity.valid = IsDatabaseNativeSettingIdentity(
        scopeType, scopeId, moduleName, keyName);
    return ReadScopedSettingValueStatus(db, identity, value);
}

QMap<QString, QString> ConfigDatabase::ReadScopedSettings(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName)
{
    QMap<QString, QString> values;
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(scopeType, scopeId, moduleName))
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
            const bool requiresDpapi = RequiresDpapiProtection(
                scopeType, moduleName, keyName, sensitive);
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
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeSettingIdentity(scopeType, scopeId, moduleName, keyName))
    {
        return false;
    }

    const bool effectiveSensitive = sensitive
        || IsSensitiveSettingKey(moduleName)
        || IsSensitiveSettingKey(keyName);
    int encrypted = 0;
    QString storedText;
    const QString purpose = ProtectionPurpose(scopeType, scopeId, moduleName, keyName);
    const bool requiresDpapi = RequiresDpapiProtection(
        scopeType, moduleName, keyName, effectiveSensitive);
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

bool ConfigDatabase::CompareAndSwapScopedSetting(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName,
    const QString* expectedValue,
    const QString& newValue,
    const QString& valueType,
    bool sensitive,
    bool* conflict,
    QString* error)
{
    if (conflict != nullptr)
    {
        *conflict = false;
    }
    if (error != nullptr)
    {
        error->clear();
    }
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("配置数据库不可用。");
        }
        return false;
    }

    ScopedSettingIdentity identity;
    identity.scopeType = scopeType;
    identity.scopeId = scopeId;
    identity.module = moduleName;
    identity.keyName = keyName;
    identity.valueType = valueType;
    identity.sensitive = sensitive
        || IsSensitiveSettingKey(moduleName)
        || IsSensitiveSettingKey(keyName);
    identity.valid = IsDatabaseNativeSettingIdentity(
        scopeType, scopeId, moduleName, keyName);
    if (!identity.valid)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("配置记录身份无效。");
        }
        return false;
    }

    QSqlQuery transaction(db);
    if (!transaction.exec(QStringLiteral("BEGIN IMMEDIATE TRANSACTION")))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法开始配置原子更新：%1")
                .arg(transaction.lastError().text());
        }
        return false;
    }
    auto rollback = [&db]()
        {
            QSqlQuery query(db);
            query.exec(QStringLiteral("ROLLBACK"));
        };

    QString currentValue;
    const ReadStatus status = ReadScopedSettingValueStatus(db, identity, &currentValue);
    if (status == ReadStatus::Error)
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("原子更新读取当前配置失败。");
        }
        return false;
    }
    const bool expectedMatches = expectedValue == nullptr
        ? status == ReadStatus::NotFound
        : status == ReadStatus::Found && currentValue == *expectedValue;
    if (!expectedMatches)
    {
        rollback();
        if (conflict != nullptr)
        {
            *conflict = true;
        }
        if (error != nullptr)
        {
            *error = QStringLiteral("配置记录已被其他进程或操作更新。");
        }
        return false;
    }
    if (!WriteScopedSettingValue(db, identity, newValue))
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("原子更新写入配置失败。");
        }
        return false;
    }
    QString writtenValue;
    const ReadStatus writtenStatus = ReadScopedSettingValueStatus(db, identity, &writtenValue);
    if (writtenStatus != ReadStatus::Found || writtenValue != newValue)
    {
        rollback();
        if (error != nullptr)
        {
            *error = writtenStatus == ReadStatus::Error
                ? QStringLiteral("原子更新写后回读失败。")
                : QStringLiteral("原子更新写后回读不一致。");
        }
        return false;
    }
    QSqlQuery commit(db);
    if (!commit.exec(QStringLiteral("COMMIT")))
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("原子更新提交失败：%1").arg(commit.lastError().text());
        }
        return false;
    }
    return true;
}

bool ConfigDatabase::CompareAndSwapScopedSettingWithWitness(
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
    const QString& targetValueType,
    bool targetSensitive,
    bool* conflict,
    QString* error)
{
    if (conflict != nullptr)
    {
        *conflict = false;
    }
    if (error != nullptr)
    {
        error->clear();
    }
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("配置数据库不可用。");
        }
        return false;
    }

    ScopedSettingIdentity witnessIdentity;
    witnessIdentity.scopeType = witnessScopeType;
    witnessIdentity.scopeId = witnessScopeId;
    witnessIdentity.module = witnessModuleName;
    witnessIdentity.keyName = witnessKeyName;
    witnessIdentity.valid = IsDatabaseNativeSettingIdentity(
        witnessScopeType, witnessScopeId, witnessModuleName, witnessKeyName);

    ScopedSettingIdentity targetIdentity;
    targetIdentity.scopeType = targetScopeType;
    targetIdentity.scopeId = targetScopeId;
    targetIdentity.module = targetModuleName;
    targetIdentity.keyName = targetKeyName;
    targetIdentity.valueType = targetValueType;
    targetIdentity.sensitive = targetSensitive
        || IsSensitiveSettingKey(targetModuleName)
        || IsSensitiveSettingKey(targetKeyName);
    targetIdentity.valid = IsDatabaseNativeSettingIdentity(
        targetScopeType, targetScopeId, targetModuleName, targetKeyName);
    if (!witnessIdentity.valid || !targetIdentity.valid)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("witness 或 target 配置记录身份无效。");
        }
        return false;
    }

    QSqlQuery transaction(db);
    if (!transaction.exec(QStringLiteral("BEGIN IMMEDIATE TRANSACTION")))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法开始带 witness 的配置原子更新：%1")
                .arg(transaction.lastError().text());
        }
        return false;
    }
    auto rollback = [&db]()
        {
            QSqlQuery query(db);
            query.exec(QStringLiteral("ROLLBACK"));
        };

    QString currentWitnessValue;
    const ReadStatus witnessStatus =
        ReadScopedSettingValueStatus(db, witnessIdentity, &currentWitnessValue);
    if (witnessStatus == ReadStatus::Error)
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("原子更新读取 witness 配置失败。");
        }
        return false;
    }
    if (witnessStatus != ReadStatus::Found || currentWitnessValue != expectedWitnessValue)
    {
        rollback();
        if (conflict != nullptr)
        {
            *conflict = true;
        }
        if (error != nullptr)
        {
            *error = QStringLiteral("witness 配置记录已被其他进程或操作更新。");
        }
        return false;
    }

    QString currentTargetValue;
    const ReadStatus targetStatus =
        ReadScopedSettingValueStatus(db, targetIdentity, &currentTargetValue);
    if (targetStatus == ReadStatus::Error)
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("原子更新读取 target 配置失败。");
        }
        return false;
    }
    const bool targetMatches = expectedTargetValue == nullptr
        ? targetStatus == ReadStatus::NotFound
        : targetStatus == ReadStatus::Found && currentTargetValue == *expectedTargetValue;
    if (!targetMatches)
    {
        rollback();
        if (conflict != nullptr)
        {
            *conflict = true;
        }
        if (error != nullptr)
        {
            *error = QStringLiteral("target 配置记录已被其他进程或操作更新。");
        }
        return false;
    }
    if (!WriteScopedSettingValue(db, targetIdentity, newTargetValue))
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("带 witness 的原子更新写入 target 失败。");
        }
        return false;
    }

    QString writtenTargetValue;
    const ReadStatus writtenStatus =
        ReadScopedSettingValueStatus(db, targetIdentity, &writtenTargetValue);
    if (writtenStatus != ReadStatus::Found || writtenTargetValue != newTargetValue)
    {
        rollback();
        if (error != nullptr)
        {
            *error = writtenStatus == ReadStatus::Error
                ? QStringLiteral("带 witness 的原子更新写后回读失败。")
                : QStringLiteral("带 witness 的原子更新写后回读不一致。");
        }
        return false;
    }

    QSqlQuery commit(db);
    if (!commit.exec(QStringLiteral("COMMIT")))
    {
        rollback();
        if (error != nullptr)
        {
            *error = QStringLiteral("带 witness 的原子更新提交失败：%1")
                .arg(commit.lastError().text());
        }
        return false;
    }
    return true;
}

bool ConfigDatabase::WriteScopedSettings(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QMap<QString, QString>& values,
    const QString& valueType)
{
    QMap<QString, ScopedSettingValue> typedValues;
    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        ScopedSettingValue typedValue;
        typedValue.value = it.value();
        typedValue.valueType = valueType;
        typedValues.insert(it.key(), typedValue);
    }
    return WriteScopedSettings(scopeType, scopeId, moduleName, typedValues);
}

bool ConfigDatabase::WriteScopedSettings(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QMap<QString, ScopedSettingValue>& values)
{
    if (!IsDatabaseNativeScopeIdentity(scopeType, scopeId, moduleName))
    {
        return false;
    }
    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        if (!IsDatabaseNativeIdentityPart(it.key(), false))
        {
            return false;
        }
    }
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
    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        const ScopedSettingValue& typedValue = it.value();
        const QString normalizedValueType = typedValue.valueType.trimmed().isEmpty()
            ? QStringLiteral("string")
            : typedValue.valueType.trimmed().toLower();
        const bool sensitive = typedValue.sensitive
            || IsSensitiveSettingKey(normalizedModule)
            || IsSensitiveSettingKey(it.key());
        int encrypted = 0;
        QString storedText;
        const QString purpose = ProtectionPurpose(
            normalizedScopeType, normalizedScopeId, normalizedModule, it.key());
        const bool requiresDpapi = RequiresDpapiProtection(
            normalizedScopeType, normalizedModule, it.key(), sensitive);
        if (!EncodeStoredText(
                db, typedValue.value, requiresDpapi, purpose, &storedText, &encrypted))
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

bool ConfigDatabase::WriteLoginState(
    const QStringList& accountHistory,
    const QString& userName,
    bool rememberPassword,
    bool autoLogin)
{
    QMap<QString, ScopedSettingValue> values;
    values.insert(
        QStringLiteral("AccountHistory"),
        { accountHistory.join(QLatin1Char('\n')), QStringLiteral("string"), false });
    values.insert(
        QStringLiteral("UserName"),
        { userName, QStringLiteral("string"), false });
    values.insert(
        QStringLiteral("RememberPassword"),
        { rememberPassword ? QStringLiteral("1") : QStringLiteral("0"),
          QStringLiteral("bool"), true });
    values.insert(
        QStringLiteral("AutoLogin"),
        { autoLogin ? QStringLiteral("1") : QStringLiteral("0"),
          QStringLiteral("bool"), true });
    return WriteScopedSettings(
        QStringLiteral("global"), QString(), QStringLiteral("LoginState"), values);
}

bool ConfigDatabase::RemoveScopedSetting(
    const QString& scopeType,
    const QString& scopeId,
    const QString& moduleName,
    const QString& keyName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeSettingIdentity(scopeType, scopeId, moduleName, keyName))
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
    if (ids == nullptr
        || !IsDatabaseNativeIdentityPart(scopeType, false)
        || (!moduleName.trimmed().isEmpty()
            && !IsDatabaseNativeIdentityPart(moduleName, true)))
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
        const QString id = query.value(0).toString();
        if (IsDatabaseNativeIdentityPart(
                id,
                false,
                NormalizeSection(scopeType).compare(QStringLiteral("global"), Qt::CaseInsensitive) == 0,
                true))
        {
            values << id;
        }
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
    if (ids == nullptr || maxCount <= 0 || maxIdLength <= 0
        || !IsDatabaseNativeIdentityPart(scopeType, false)
        || (!moduleName.trimmed().isEmpty()
            && !IsDatabaseNativeIdentityPart(moduleName, true)))
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
        if (!IsDatabaseNativeIdentityPart(
                id,
                false,
                NormalizeSection(scopeType).compare(QStringLiteral("global"), Qt::CaseInsensitive) == 0,
                true))
        {
            continue;
        }
        if (id.size() > maxIdLength || values.size() >= maxCount)
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
    if (!db.isValid() || !db.isOpen()
        || !IsDatabaseNativeScopeIdentity(
            scopeType, scopeId, moduleName, moduleName.trimmed().isEmpty()))
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
