#include "ConfigDatabase.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRandomGenerator>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QThread>
#include <algorithm>
#include <cstring>

namespace
{
constexpr char kSchemaVersion[] = "4";
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
bool EnsureCurrentSchema(QSqlDatabase& db);

QString ConnectionName()
{
    const quintptr threadId = reinterpret_cast<quintptr>(QThread::currentThreadId());
    return QString("ConfigStore_%1").arg(threadId);
}

QString FindProjectRoot()
{
    QStringList bases;
    if (QCoreApplication::instance() != nullptr)
    {
        bases << QCoreApplication::applicationDirPath();
    }
    bases << QDir::currentPath();

    QString firstDataRoot;
    for (const QString& base : bases)
    {
        QDir dir(base);
        for (int depth = 0; depth < 8; ++depth)
        {
            const QString dataPath = dir.filePath("Data");
            const QFileInfo dataInfo(dataPath);
            if (dataInfo.exists() && dataInfo.isDir())
            {
                if (firstDataRoot.isEmpty())
                {
                    firstDataRoot = dir.absolutePath();
                }
                if (QFileInfo::exists(QDir(dataPath).filePath("ConfigStore.db")))
                {
                    return dir.absolutePath();
                }
            }
            if (!dir.cdUp())
            {
                break;
            }
        }
    }

    if (!firstDataRoot.isEmpty())
    {
        return firstDataRoot;
    }
    return QDir::currentPath();
}

QSqlDatabase OpenDatabase()
{
    const QString connectionName = ConnectionName();
    if (QSqlDatabase::contains(connectionName))
    {
        QSqlDatabase db = QSqlDatabase::database(connectionName);
        if (db.isOpen())
        {
            return db;
        }
        if (db.open())
        {
            return db;
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

QString ProtectText(const QString& plainText)
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

bool UnprotectText(const QString& storedText, QString* plainText)
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

bool DecodeStoredText(const QString& storedText, int encrypted, QString* plainText)
{
    if (plainText == nullptr)
    {
        return false;
    }
    if (encrypted != 0 || storedText.startsWith("enc:v1:"))
    {
        return UnprotectText(storedText, plainText);
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
    return ProtectText(plainText);
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

bool IsSensitiveSettingKey(const QString& keyName)
{
    const QString lower = keyName.toLower();
    return lower.contains(QStringLiteral("password"))
        || lower.contains(QStringLiteral("passwd"))
        || lower.contains(QStringLiteral("pass"))
        || lower.contains(QStringLiteral("token"))
        || lower.contains(QStringLiteral("secret"));
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

bool ReadScopedSettingValue(QSqlDatabase& db, const ScopedSettingIdentity& identity, QString* value)
{
    if (!identity.valid || value == nullptr)
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare("SELECT value_text, encrypted FROM settings WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?");
    query.addBindValue(NormalizeSection(identity.scopeType).toLower());
    query.addBindValue(NormalizeScopeId(identity.scopeId));
    query.addBindValue(NormalizeSection(identity.module));
    query.addBindValue(NormalizeSourceKey(identity.keyName));
    if (!query.exec())
    {
        return false;
    }
    if (!query.next())
    {
        return false;
    }
    const QString storedText = query.value(0).toString();
    const int encrypted = query.value(1).toInt();
    return DecodeStoredText(storedText, encrypted, value);
}

bool WriteScopedSettingValue(QSqlDatabase& db, const ScopedSettingIdentity& identity, const QString& value)
{
    if (!identity.valid)
    {
        return false;
    }

    int encrypted = 0;
    const QString storedText = identity.sensitive
        ? ProtectText(value)
        : StoredTextForWrite(db, value, &encrypted);
    if (identity.sensitive)
    {
        encrypted = 1;
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
    query.addBindValue(identity.sensitive ? 1 : 0);
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

bool SetSchemaVersion(QSqlDatabase& db)
{
    QSqlQuery query(db);
    query.prepare("INSERT OR REPLACE INTO meta(key, value) VALUES('schema_version', ?)");
    query.addBindValue(QString::fromLatin1(kSchemaVersion));
    if (!query.exec())
    {
        return false;
    }

    QSqlQuery encryptQuery(db);
    encryptQuery.prepare("INSERT OR IGNORE INTO meta(key, value) VALUES('encrypt_new_values', '1')");
    return encryptQuery.exec();
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

bool EnsureCurrentSchema(QSqlDatabase& db)
{
    return CreateCurrentTables(db)
        && HasCurrentSettingsColumns(db)
        && SetSchemaVersion(db);
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
    return QDir(FindProjectRoot()).filePath("Data/ConfigStore.db");
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

    const QString lower = path.toLower();
    const int dataPos = lower.indexOf("/data/");
    if (dataPos >= 0)
    {
        return path.mid(dataPos + 1);
    }
    const int resultPos = lower.indexOf("/result/");
    if (resultPos >= 0)
    {
        return path.mid(resultPos + 1);
    }
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
        QDir root(FindProjectRoot());
        const QString rel = root.relativeFilePath(info.absoluteFilePath()).replace('\\', '/');
        if (rel.toLower().startsWith("data/"))
        {
            return QDir::cleanPath(rel);
        }
        if (rel.toLower().startsWith("result/"))
        {
            return QDir::cleanPath(rel);
        }
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
    if (value == nullptr)
    {
        return false;
    }

    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const ScopedSettingIdentity scopedIdentity = BuildScopedIniIdentity(
        FromUtf8StdString(NormalizeFilePath(fileName)),
        DecodeMaybeLocal(sectionName),
        DecodeMaybeLocal(keyName));
    if (!scopedIdentity.valid)
    {
        return false;
    }
    QString scopedValue;
    if (!ReadScopedSettingValue(db, scopedIdentity, &scopedValue))
    {
        return false;
    }

    const QByteArray bytes = scopedValue.toUtf8();
    *value = std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
    return true;
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
        if (!DecodeStoredText(query.value(1).toString(), query.value(2).toInt(), &plainText))
        {
            continue;
        }
        values.insert(query.value(0).toString(), plainText);
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
    if (value == nullptr)
    {
        return false;
    }

    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare("SELECT value_text, encrypted FROM settings WHERE scope_type=? AND scope_id=? AND module=? AND key_name=?");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(NormalizeSection(moduleName));
    query.addBindValue(NormalizeSourceKey(keyName));
    if (!query.exec())
    {
        return false;
    }
    if (!query.next())
    {
        return false;
    }

    const QString storedText = query.value(0).toString();
    const int encrypted = query.value(1).toInt();
    return DecodeStoredText(storedText, encrypted, value);
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
        "SELECT key_name, value_text, encrypted FROM settings "
        "WHERE scope_type=? AND scope_id=? AND module=? ORDER BY key_name");
    query.addBindValue(NormalizeSection(scopeType).toLower());
    query.addBindValue(NormalizeScopeId(scopeId));
    query.addBindValue(NormalizeSection(moduleName));
    if (!query.exec())
    {
        return values;
    }
    while (query.next())
    {
        QString decoded;
        if (DecodeStoredText(query.value(1).toString(), query.value(2).toInt(), &decoded))
        {
            values.insert(query.value(0).toString(), decoded);
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

    int encrypted = 0;
    const QString storedText = sensitive
        ? ProtectText(value)
        : StoredTextForWrite(db, value, &encrypted);
    if (sensitive)
    {
        encrypted = 1;
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
    query.addBindValue(sensitive ? 1 : 0);
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
        "VALUES(?, ?, ?, ?, ?, ?, 0, ?, datetime('now'))");
    const QString normalizedScopeType = NormalizeSection(scopeType).toLower();
    const QString normalizedScopeId = NormalizeScopeId(scopeId);
    const QString normalizedModule = NormalizeSection(moduleName);
    const QString normalizedValueType = valueType.trimmed().isEmpty()
        ? QStringLiteral("string")
        : valueType.trimmed().toLower();
    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        int encrypted = 0;
        const QString storedText = StoredTextForWrite(db, it.value(), &encrypted);
        query.bindValue(0, normalizedScopeType);
        query.bindValue(1, normalizedScopeId);
        query.bindValue(2, normalizedModule);
        query.bindValue(3, NormalizeSourceKey(it.key()));
        query.bindValue(4, storedText);
        query.bindValue(5, normalizedValueType);
        query.bindValue(6, encrypted);
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
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return QStringList();
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
        return QStringList();
    }

    QStringList ids;
    while (query.next())
    {
        ids << query.value(0).toString();
    }
    return UniqueSorted(ids);
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
