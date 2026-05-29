#include "ConfigDatabase.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QThread>
#include <QVariant>
#include <algorithm>
#include <cstring>

namespace
{
constexpr char kSchemaVersion[] = "2";
constexpr char kSecret[] = "NoTeachingRobotConfigStoreV1";

struct ConfigFileIdentity
{
    QString category;
    QString robotName;
    QString fileName;
    QString sourcePath;
};

struct ConfigIniIdentity
{
    ConfigFileIdentity file;
    QString groupName;
    QString itemName;
    QString keyName;
    QString sourceSection;
    QString sourceKey;
};

QString NormalizeSection(const QString& sectionName);
QString NormalizeSourceKey(const QString& keyName);
ConfigFileIdentity BuildFileIdentity(const QString& fileName);
ConfigIniIdentity BuildIniIdentity(const QString& fileName, const QString& sectionName, const QString& keyName);
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

    for (const QString& base : bases)
    {
        QDir dir(base);
        for (int depth = 0; depth < 8; ++depth)
        {
            if (QFileInfo::exists(dir.filePath("Data")))
            {
                return dir.absolutePath();
            }
            if (!dir.cdUp())
            {
                break;
            }
        }
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
    if (!QFileInfo::exists(dbPath))
    {
        return QSqlDatabase();
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

    const QByteArray nonce = QByteArray::fromBase64(paddedBase64(parts.at(2).toLatin1()));
    QByteArray bytes = QByteArray::fromBase64(paddedBase64(parts.at(3).toLatin1()));
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

QString CleanDbName(QString text)
{
    text = text.trimmed();
    text.replace('\\', '/');
    text.replace('_', ' ');
    text.replace(QRegularExpression(QStringLiteral("\\s+")), QStringLiteral(" "));
    return text.trimmed();
}

QString BaseFileNameForDb(const QString& sourcePath)
{
    QString name = QFileInfo(sourcePath).completeBaseName();
    if (name.isEmpty())
    {
        name = QFileInfo(sourcePath).fileName();
    }
    return CleanDbName(name);
}

ConfigFileIdentity BuildFileIdentity(const QString& fileName)
{
    ConfigFileIdentity identity;
    identity.sourcePath = ConfigDatabase::NormalizeFilePath(fileName);

    const QStringList parts = identity.sourcePath.split('/', Qt::SkipEmptyParts);
    if (!parts.isEmpty() && parts.first().compare(QStringLiteral("Data"), Qt::CaseInsensitive) == 0)
    {
        identity.category = QStringLiteral("数据配置");
        if (parts.size() >= 3 && parts.at(1).startsWith(QStringLiteral("Robot"), Qt::CaseInsensitive))
        {
            identity.robotName = CleanDbName(parts.at(1));
        }
    }
    else if (!parts.isEmpty() && parts.first().compare(QStringLiteral("Result"), Qt::CaseInsensitive) == 0)
    {
        identity.category = QStringLiteral("结果数据");
        if (parts.size() >= 3 && parts.at(1).startsWith(QStringLiteral("Robot"), Qt::CaseInsensitive))
        {
            identity.robotName = CleanDbName(parts.at(1));
        }
    }
    else
    {
        identity.category = QStringLiteral("配置数据");
    }

    if (identity.robotName.isEmpty())
    {
        identity.robotName = QStringLiteral("全局");
    }
    identity.fileName = BaseFileNameForDb(identity.sourcePath);
    if (identity.fileName.isEmpty())
    {
        identity.fileName = QStringLiteral("未命名");
    }
    return identity;
}

void SplitKeyName(const QString& sourceKey, QString* itemName, QString* keyName)
{
    QString localItem;
    QString localKey = sourceKey;
    const QStringList pathParts = sourceKey.split('/', Qt::SkipEmptyParts);
    if (pathParts.size() > 1)
    {
        QStringList itemParts = pathParts;
        localKey = itemParts.takeLast();
        localItem = itemParts.join(QStringLiteral(" / "));
    }
    else
    {
        const int underscore = sourceKey.lastIndexOf('_');
        if (underscore > 0 && underscore + 1 < sourceKey.size())
        {
            localItem = sourceKey.left(underscore);
            localKey = sourceKey.mid(underscore + 1);
        }
    }

    if (itemName != nullptr)
    {
        *itemName = CleanDbName(localItem);
    }
    if (keyName != nullptr)
    {
        *keyName = CleanDbName(localKey);
    }
}

ConfigIniIdentity BuildIniIdentity(const QString& fileName, const QString& sectionName, const QString& keyName)
{
    ConfigIniIdentity identity;
    identity.file = BuildFileIdentity(fileName);
    identity.sourceSection = NormalizeSection(sectionName);
    identity.sourceKey = NormalizeSourceKey(keyName);

    const QStringList sectionParts = identity.sourceSection.split('/', Qt::SkipEmptyParts);
    identity.groupName = sectionParts.isEmpty() ? QStringLiteral("默认") : CleanDbName(sectionParts.first());
    QString sectionItem;
    if (sectionParts.size() > 1)
    {
        sectionItem = sectionParts.mid(1).join(QStringLiteral(" / "));
    }

    QString keyItem;
    SplitKeyName(identity.sourceKey, &keyItem, &identity.keyName);
    QStringList itemParts;
    if (!sectionItem.trimmed().isEmpty())
    {
        itemParts << sectionItem;
    }
    if (!keyItem.trimmed().isEmpty())
    {
        itemParts << keyItem;
    }
    identity.itemName = CleanDbName(itemParts.join(QStringLiteral(" / ")));
    if (identity.keyName.isEmpty())
    {
        identity.keyName = QStringLiteral("值");
    }
    return identity;
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

bool TableExists(QSqlDatabase& db, const QString& tableName)
{
    QSqlQuery query(db);
    query.prepare("SELECT 1 FROM sqlite_master WHERE type='table' AND name=? LIMIT 1");
    query.addBindValue(tableName);
    return query.exec() && query.next();
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
            "CREATE TABLE IF NOT EXISTS ini_values ("
            "category TEXT NOT NULL DEFAULT '',"
            "robot_name TEXT NOT NULL DEFAULT '',"
            "file_name TEXT NOT NULL DEFAULT '',"
            "group_name TEXT NOT NULL DEFAULT '',"
            "item_name TEXT NOT NULL DEFAULT '',"
            "key_name TEXT NOT NULL DEFAULT '',"
            "source_path TEXT NOT NULL,"
            "source_section TEXT NOT NULL,"
            "source_key TEXT NOT NULL,"
            "value_text TEXT NOT NULL,"
            "encrypted INTEGER NOT NULL DEFAULT 0,"
            "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,"
            "PRIMARY KEY(source_path, source_section, source_key))"))
        && ExecSql(db, QStringLiteral(
            "CREATE INDEX IF NOT EXISTS idx_ini_values_display "
            "ON ini_values(category, robot_name, file_name, group_name, item_name, key_name)"))
        && ExecSql(db, QStringLiteral(
            "CREATE TABLE IF NOT EXISTS text_files ("
            "category TEXT NOT NULL DEFAULT '',"
            "robot_name TEXT NOT NULL DEFAULT '',"
            "file_name TEXT NOT NULL DEFAULT '',"
            "source_path TEXT PRIMARY KEY,"
            "content_text TEXT NOT NULL,"
            "encrypted INTEGER NOT NULL DEFAULT 0,"
            "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP)"))
        && ExecSql(db, QStringLiteral(
            "CREATE INDEX IF NOT EXISTS idx_text_files_display "
            "ON text_files(category, robot_name, file_name)"));
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

QString UniqueLegacyTableName(QSqlDatabase& db, const QString& baseName)
{
    QString name = baseName + QStringLiteral("_legacy");
    int suffix = 1;
    while (TableExists(db, name))
    {
        name = QStringLiteral("%1_legacy_%2").arg(baseName).arg(suffix++);
    }
    return name;
}

bool MigrateLegacySchema(QSqlDatabase& db)
{
    if (!TableExists(db, QStringLiteral("ini_values")) || !TableExists(db, QStringLiteral("text_files")))
    {
        return false;
    }

    const QString legacyIni = UniqueLegacyTableName(db, QStringLiteral("ini_values"));
    const QString legacyText = UniqueLegacyTableName(db, QStringLiteral("text_files"));
    if (!db.transaction())
    {
        return false;
    }

    bool ok = ExecSql(db, QStringLiteral("ALTER TABLE ini_values RENAME TO %1").arg(legacyIni))
        && ExecSql(db, QStringLiteral("ALTER TABLE text_files RENAME TO %1").arg(legacyText))
        && CreateCurrentTables(db);
    if (!ok)
    {
        db.rollback();
        return false;
    }

    QSqlQuery legacyIniQuery(db);
    if (!legacyIniQuery.exec(QStringLiteral(
        "SELECT file_path, section_name, key_name, value_text, encrypted, updated_at FROM %1")
        .arg(legacyIni)))
    {
        db.rollback();
        return false;
    }

    QSqlQuery insertIni(db);
    insertIni.prepare(
        "INSERT OR REPLACE INTO ini_values("
        "category, robot_name, file_name, group_name, item_name, key_name, "
        "source_path, source_section, source_key, value_text, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

    while (legacyIniQuery.next())
    {
        const ConfigIniIdentity identity = BuildIniIdentity(
            legacyIniQuery.value(0).toString(),
            legacyIniQuery.value(1).toString(),
            legacyIniQuery.value(2).toString());
        insertIni.addBindValue(identity.file.category);
        insertIni.addBindValue(identity.file.robotName);
        insertIni.addBindValue(identity.file.fileName);
        insertIni.addBindValue(identity.groupName);
        insertIni.addBindValue(identity.itemName);
        insertIni.addBindValue(identity.keyName);
        insertIni.addBindValue(identity.file.sourcePath);
        insertIni.addBindValue(identity.sourceSection);
        insertIni.addBindValue(identity.sourceKey);
        insertIni.addBindValue(legacyIniQuery.value(3));
        insertIni.addBindValue(legacyIniQuery.value(4));
        insertIni.addBindValue(legacyIniQuery.value(5));
        if (!insertIni.exec())
        {
            db.rollback();
            return false;
        }
    }

    QSqlQuery legacyTextQuery(db);
    if (!legacyTextQuery.exec(QStringLiteral(
        "SELECT file_path, content_text, encrypted, updated_at FROM %1")
        .arg(legacyText)))
    {
        db.rollback();
        return false;
    }

    QSqlQuery insertText(db);
    insertText.prepare(
        "INSERT OR REPLACE INTO text_files("
        "category, robot_name, file_name, source_path, content_text, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?)");

    while (legacyTextQuery.next())
    {
        const ConfigFileIdentity identity = BuildFileIdentity(legacyTextQuery.value(0).toString());
        insertText.addBindValue(identity.category);
        insertText.addBindValue(identity.robotName);
        insertText.addBindValue(identity.fileName);
        insertText.addBindValue(identity.sourcePath);
        insertText.addBindValue(legacyTextQuery.value(1));
        insertText.addBindValue(legacyTextQuery.value(2));
        insertText.addBindValue(legacyTextQuery.value(3));
        if (!insertText.exec())
        {
            db.rollback();
            return false;
        }
    }

    ok = ExecSql(db, QStringLiteral("DROP TABLE %1").arg(legacyIni))
        && ExecSql(db, QStringLiteral("DROP TABLE %1").arg(legacyText))
        && SetSchemaVersion(db);
    if (!ok)
    {
        db.rollback();
        return false;
    }
    return db.commit();
}

bool HasCurrentIniColumns(QSqlDatabase& db)
{
    const QStringList columns = TableColumns(db, QStringLiteral("ini_values"));
    const QStringList required = {
        QStringLiteral("category"),
        QStringLiteral("robot_name"),
        QStringLiteral("file_name"),
        QStringLiteral("group_name"),
        QStringLiteral("item_name"),
        QStringLiteral("key_name"),
        QStringLiteral("source_path"),
        QStringLiteral("source_section"),
        QStringLiteral("source_key"),
        QStringLiteral("value_text"),
        QStringLiteral("encrypted"),
        QStringLiteral("updated_at")
    };
    return std::all_of(required.cbegin(), required.cend(), [&columns](const QString& column)
        {
            return columns.contains(column);
        });
}

bool HasLegacyIniColumns(QSqlDatabase& db)
{
    const QStringList columns = TableColumns(db, QStringLiteral("ini_values"));
    return columns.contains(QStringLiteral("file_path"))
        && columns.contains(QStringLiteral("section_name"))
        && columns.contains(QStringLiteral("key_name"))
        && columns.contains(QStringLiteral("value_text"));
}

bool EnsureCurrentSchema(QSqlDatabase& db)
{
    if (!TableExists(db, QStringLiteral("meta")))
    {
        return false;
    }

    QSqlQuery metaQuery(db);
    metaQuery.prepare("SELECT value FROM meta WHERE key='schema_version'");
    const QString version = (metaQuery.exec() && metaQuery.next()) ? metaQuery.value(0).toString() : QString();
    if (version == QString::fromLatin1(kSchemaVersion) && HasCurrentIniColumns(db))
    {
        return CreateCurrentTables(db) && SetSchemaVersion(db);
    }

    if (HasLegacyIniColumns(db))
    {
        return MigrateLegacySchema(db);
    }

    return false;
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

    QSqlQuery query(db);
    query.prepare("SELECT 1 FROM ini_values WHERE source_path=? LIMIT 1");
    query.addBindValue(FromUtf8StdString(NormalizeFilePath(fileName)));
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

    QSqlQuery query(db);
    const ConfigIniIdentity identity = BuildIniIdentity(
        FromUtf8StdString(NormalizeFilePath(fileName)),
        DecodeMaybeLocal(sectionName),
        DecodeMaybeLocal(keyName));
    query.prepare("SELECT value_text, encrypted FROM ini_values WHERE source_path=? AND source_section=? AND source_key=?");
    query.addBindValue(identity.file.sourcePath);
    query.addBindValue(identity.sourceSection);
    query.addBindValue(identity.sourceKey);
    if (!query.exec() || !query.next())
    {
        return false;
    }

    QString plainText;
    if (!DecodeStoredText(query.value(0).toString(), query.value(1).toInt(), &plainText))
    {
        return false;
    }
    const QByteArray bytes = plainText.toUtf8();
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

    int encrypted = 0;
    const QString storedText = StoredTextForWrite(db, DecodeMaybeLocal(value), &encrypted);

    const ConfigIniIdentity identity = BuildIniIdentity(
        FromUtf8StdString(NormalizeFilePath(fileName)),
        DecodeMaybeLocal(sectionName),
        DecodeMaybeLocal(keyName));
    QSqlQuery query(db);
    query.prepare("INSERT OR REPLACE INTO ini_values("
        "category, robot_name, file_name, group_name, item_name, key_name, "
        "source_path, source_section, source_key, value_text, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))");
    query.addBindValue(identity.file.category);
    query.addBindValue(identity.file.robotName);
    query.addBindValue(identity.file.fileName);
    query.addBindValue(identity.groupName);
    query.addBindValue(identity.itemName);
    query.addBindValue(identity.keyName);
    query.addBindValue(identity.file.sourcePath);
    query.addBindValue(identity.sourceSection);
    query.addBindValue(identity.sourceKey);
    query.addBindValue(storedText);
    query.addBindValue(encrypted);
    return query.exec();
}

bool ConfigDatabase::HasTextFile(const std::string& fileName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    QSqlQuery query(db);
    query.prepare("SELECT 1 FROM text_files WHERE source_path=? LIMIT 1");
    query.addBindValue(FromUtf8StdString(NormalizeFilePath(fileName)));
    return query.exec() && query.next();
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

    QSqlQuery query(db);
    query.prepare("SELECT content_text, encrypted FROM text_files WHERE source_path=?");
    query.addBindValue(FromUtf8StdString(NormalizeFilePath(fileName)));
    if (!query.exec() || !query.next())
    {
        return false;
    }

    QString plainText;
    if (!DecodeStoredText(query.value(0).toString(), query.value(1).toInt(), &plainText))
    {
        return false;
    }
    const QByteArray bytes = plainText.toUtf8();
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

    int encrypted = 0;
    const QString storedText = StoredTextForWrite(db, DecodeMaybeLocal(content), &encrypted);

    const ConfigFileIdentity identity = BuildFileIdentity(FromUtf8StdString(NormalizeFilePath(fileName)));
    QSqlQuery query(db);
    query.prepare("INSERT OR REPLACE INTO text_files("
        "category, robot_name, file_name, source_path, content_text, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, datetime('now'))");
    query.addBindValue(identity.category);
    query.addBindValue(identity.robotName);
    query.addBindValue(identity.fileName);
    query.addBindValue(identity.sourcePath);
    query.addBindValue(storedText);
    query.addBindValue(encrypted);
    return query.exec();
}

bool ConfigDatabase::CopyTextFile(const QString& sourceFileName, const QString& targetFileName, bool overwriteExisting)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    if (!HasTextFile(sourceFileName))
    {
        return false;
    }

    QSqlQuery sourceQuery(db);
    sourceQuery.prepare("SELECT content_text, encrypted FROM text_files WHERE source_path=?");
    sourceQuery.addBindValue(NormalizeFilePath(sourceFileName));
    if (!sourceQuery.exec() || !sourceQuery.next())
    {
        return false;
    }

    const ConfigFileIdentity target = BuildFileIdentity(targetFileName);
    QSqlQuery query(db);
    query.prepare(QStringLiteral("INSERT OR %1 INTO text_files("
        "category, robot_name, file_name, source_path, content_text, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, datetime('now'))")
        .arg(overwriteExisting ? QStringLiteral("REPLACE") : QStringLiteral("IGNORE")));
    query.addBindValue(target.category);
    query.addBindValue(target.robotName);
    query.addBindValue(target.fileName);
    query.addBindValue(target.sourcePath);
    query.addBindValue(sourceQuery.value(0));
    query.addBindValue(sourceQuery.value(1));
    return query.exec();
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

    const QString childPrefix = normalized + QStringLiteral("/");
    if (!db.transaction())
    {
        return false;
    }

    QSqlQuery iniQuery(db);
    iniQuery.prepare("DELETE FROM ini_values WHERE source_path=? OR substr(source_path, 1, ?)=?");
    iniQuery.addBindValue(normalized);
    iniQuery.addBindValue(childPrefix.size());
    iniQuery.addBindValue(childPrefix);
    if (!iniQuery.exec())
    {
        db.rollback();
        return false;
    }

    QSqlQuery textQuery(db);
    textQuery.prepare("DELETE FROM text_files WHERE source_path=? OR substr(source_path, 1, ?)=?");
    textQuery.addBindValue(normalized);
    textQuery.addBindValue(childPrefix.size());
    textQuery.addBindValue(childPrefix);
    if (!textQuery.exec())
    {
        db.rollback();
        return false;
    }

    return db.commit();
}

QStringList ConfigDatabase::ListIniGroups(const QString& fileName, const QString& parentGroup)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return QStringList();
    }

    const QString normalizedParent = NormalizeSection(parentGroup);
    const QString prefix = normalizedParent.isEmpty() ? QString() : normalizedParent + "/";
    QStringList groups;

    QSqlQuery query(db);
    query.prepare("SELECT DISTINCT source_section FROM ini_values WHERE source_path=?");
    query.addBindValue(NormalizeFilePath(fileName));
    if (!query.exec())
    {
        return QStringList();
    }

    while (query.next())
    {
        const QString section = NormalizeSection(query.value(0).toString());
        if (!section.startsWith(prefix))
        {
            continue;
        }
        const QString remain = section.mid(prefix.size());
        if (remain.isEmpty() || remain.contains('/'))
        {
            continue;
        }
        groups << remain;
    }
    return UniqueSorted(groups);
}

QStringList ConfigDatabase::ListIniSections(const QString& fileName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return QStringList();
    }

    QStringList sections;
    QSqlQuery query(db);
    query.prepare("SELECT DISTINCT source_section FROM ini_values WHERE source_path=? ORDER BY source_section COLLATE NOCASE");
    query.addBindValue(NormalizeFilePath(fileName));
    if (!query.exec())
    {
        return QStringList();
    }
    while (query.next())
    {
        sections << query.value(0).toString();
    }
    return sections;
}

bool ConfigDatabase::CopyIniFile(const QString& sourceFileName, const QString& targetFileName, bool overwriteExisting)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    if (!HasIniFile(sourceFileName))
    {
        return false;
    }

    if (overwriteExisting)
    {
        QSqlQuery deleteQuery(db);
        deleteQuery.prepare("DELETE FROM ini_values WHERE source_path=?");
        deleteQuery.addBindValue(NormalizeFilePath(targetFileName));
        if (!deleteQuery.exec())
        {
            return false;
        }
    }

    QSqlQuery sourceQuery(db);
    sourceQuery.prepare("SELECT source_section, source_key, value_text, encrypted FROM ini_values WHERE source_path=?");
    sourceQuery.addBindValue(NormalizeFilePath(sourceFileName));
    if (!sourceQuery.exec())
    {
        return false;
    }

    QSqlQuery insertQuery(db);
    insertQuery.prepare(QStringLiteral("INSERT OR %1 INTO ini_values("
        "category, robot_name, file_name, group_name, item_name, key_name, "
        "source_path, source_section, source_key, value_text, encrypted, updated_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, datetime('now'))")
        .arg(overwriteExisting ? QStringLiteral("REPLACE") : QStringLiteral("IGNORE")));
    while (sourceQuery.next())
    {
        const ConfigIniIdentity identity = BuildIniIdentity(
            targetFileName,
            sourceQuery.value(0).toString(),
            sourceQuery.value(1).toString());
        insertQuery.addBindValue(identity.file.category);
        insertQuery.addBindValue(identity.file.robotName);
        insertQuery.addBindValue(identity.file.fileName);
        insertQuery.addBindValue(identity.groupName);
        insertQuery.addBindValue(identity.itemName);
        insertQuery.addBindValue(identity.keyName);
        insertQuery.addBindValue(identity.file.sourcePath);
        insertQuery.addBindValue(identity.sourceSection);
        insertQuery.addBindValue(identity.sourceKey);
        insertQuery.addBindValue(sourceQuery.value(2));
        insertQuery.addBindValue(sourceQuery.value(3));
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

    const QString normalizedGroup = NormalizeSection(groupName);
    QSqlQuery query(db);
    query.prepare("DELETE FROM ini_values WHERE source_path=? AND (source_section=? OR source_section LIKE ?)");
    query.addBindValue(NormalizeFilePath(fileName));
    query.addBindValue(normalizedGroup);
    query.addBindValue(normalizedGroup + "/%");
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

    QSqlQuery query(db);
    query.prepare("SELECT source_key, value_text, encrypted FROM ini_values WHERE source_path=? AND source_section=? ORDER BY source_key COLLATE NOCASE");
    query.addBindValue(NormalizeFilePath(fileName));
    query.addBindValue(NormalizeSection(sectionName));
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

    QSqlQuery query(db);
    query.prepare("DELETE FROM ini_values WHERE source_path=? AND source_section=?");
    query.addBindValue(NormalizeFilePath(fileName));
    query.addBindValue(NormalizeSection(sectionName));
    return query.exec();
}

bool ConfigDatabase::ReadSetting(const QString& fileName, const QString& keyName, QString* value)
{
    if (value == nullptr)
    {
        return false;
    }
    const int slash = keyName.lastIndexOf('/');
    const QString section = slash >= 0 ? keyName.left(slash) : QStringLiteral("Settings");
    const QString key = slash >= 0 ? keyName.mid(slash + 1) : keyName;
    std::string raw;
    if (!ReadIniValue(fileName.toUtf8().constData(), section.toUtf8().constData(), key.toUtf8().constData(), &raw))
    {
        return false;
    }
    *value = DecodeMaybeLocal(raw);
    return true;
}

bool ConfigDatabase::WriteSetting(const QString& fileName, const QString& keyName, const QString& value)
{
    const int slash = keyName.lastIndexOf('/');
    const QString section = slash >= 0 ? keyName.left(slash) : QStringLiteral("Settings");
    const QString key = slash >= 0 ? keyName.mid(slash + 1) : keyName;
    return WriteIniValue(
        fileName.toUtf8().constData(),
        section.toUtf8().constData(),
        key.toUtf8().constData(),
        value.toUtf8().constData());
}

bool ConfigDatabase::RemoveSetting(const QString& fileName, const QString& keyName)
{
    QSqlDatabase db = OpenDatabase();
    if (!db.isValid() || !db.isOpen())
    {
        return false;
    }

    const int slash = keyName.lastIndexOf('/');
    const QString section = slash >= 0 ? keyName.left(slash) : QStringLiteral("Settings");
    const QString key = slash >= 0 ? keyName.mid(slash + 1) : keyName;
    QSqlQuery query(db);
    query.prepare("DELETE FROM ini_values WHERE source_path=? AND source_section=? AND source_key=?");
    query.addBindValue(NormalizeFilePath(fileName));
    query.addBindValue(NormalizeSection(section));
    query.addBindValue(NormalizeSourceKey(key));
    return query.exec();
}
