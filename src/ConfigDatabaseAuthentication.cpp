#include "ConfigDatabase.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QSet>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QThread>
#include <QUuid>

namespace
{
class ScopedSqliteConnection
{
public:
    explicit ScopedSqliteConnection(const QString& purpose)
        : m_name(QStringLiteral("ConfigDatabaseAuth_%1_%2_%3")
            .arg(
                purpose,
                QString::number(reinterpret_cast<quintptr>(QThread::currentThreadId()), 16),
                QUuid::createUuid().toString(QUuid::WithoutBraces)))
    {
        m_database = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), m_name);
        m_database.setDatabaseName(ConfigDatabase::DatabasePath());
        if (m_database.open())
        {
            QSqlQuery pragma(m_database);
            pragma.exec(QStringLiteral("PRAGMA busy_timeout=3000"));
            pragma.exec(QStringLiteral("PRAGMA secure_delete=ON"));
        }
    }

    ~ScopedSqliteConnection()
    {
        if (m_database.isValid())
        {
            m_database.close();
        }
        m_database = QSqlDatabase();
        QSqlDatabase::removeDatabase(m_name);
    }

    QSqlDatabase& database()
    {
        return m_database;
    }

private:
    QString m_name;
    QSqlDatabase m_database;
};

bool Rollback(QSqlDatabase& database)
{
    QSqlQuery rollback(database);
    rollback.exec(QStringLiteral("ROLLBACK"));
    return false;
}

bool IsValidRole(const QString& role)
{
    return role == QLatin1String("operator")
        || role == QLatin1String("engineer")
        || role == QLatin1String("admin");
}

bool ParseStrictBool(const QString& value, bool* parsed)
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

QString AccountFingerprint(
    const QString& passwordRecord,
    const QString& role,
    bool mustChangePassword)
{
    const QByteArray material = passwordRecord.toUtf8()
        + '\n' + role.toUtf8()
        + '\n' + (mustChangePassword ? QByteArrayLiteral("1") : QByteArrayLiteral("0"));
    return QString::fromLatin1(
        QCryptographicHash::hash(material, QCryptographicHash::Sha256).toHex());
}

bool ReadAccountState(
    QSqlDatabase& database,
    const QString& accountId,
    QString* passwordRecord,
    QString* role,
    bool* mustChangePassword,
    QString* fingerprint)
{
    if (passwordRecord == nullptr || role == nullptr
        || mustChangePassword == nullptr || fingerprint == nullptr)
    {
        return false;
    }
    QSqlQuery query(database);
    query.prepare(QStringLiteral(
        "SELECT key_name, value_text FROM settings "
        "WHERE scope_type='account' AND scope_id=? AND module='Profile' "
        "AND key_name IN ('PasswordHash', 'Role', 'MustChangePassword')"));
    query.addBindValue(accountId.trimmed());
    if (!query.exec())
    {
        return false;
    }
    QMap<QString, QString> values;
    while (query.next())
    {
        values.insert(query.value(0).toString(), query.value(1).toString());
    }
    const QString currentPassword = values.value(QStringLiteral("PasswordHash"));
    const QString currentRole = values.value(QStringLiteral("Role"));
    bool currentMustChange = false;
    if (values.size() != 3
        || currentPassword.isEmpty()
        || !IsValidRole(currentRole)
        || !ParseStrictBool(values.value(QStringLiteral("MustChangePassword")), &currentMustChange))
    {
        return false;
    }
    *passwordRecord = currentPassword;
    *role = currentRole;
    *mustChangePassword = currentMustChange;
    *fingerprint = AccountFingerprint(currentPassword, currentRole, currentMustChange);
    return true;
}

bool BeginImmediate(QSqlDatabase& database)
{
    QSqlQuery begin(database);
    return begin.exec(QStringLiteral("BEGIN IMMEDIATE"));
}

bool UpsertProfileValue(
    QSqlDatabase& database,
    const QString& accountId,
    const QString& key,
    const QString& value,
    const QString& valueType,
    bool sensitive)
{
    QSqlQuery query(database);
    query.prepare(QStringLiteral(
        "INSERT INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES('account', ?, 'Profile', ?, ?, ?, ?, 0, datetime('now')) "
        "ON CONFLICT(scope_type, scope_id, module, key_name) DO UPDATE SET "
        "value_text=excluded.value_text, value_type=excluded.value_type, "
        "sensitive=excluded.sensitive, encrypted=0, updated_at=datetime('now')"));
    query.addBindValue(accountId.trimmed());
    query.addBindValue(key);
    query.addBindValue(value);
    query.addBindValue(valueType);
    query.addBindValue(sensitive ? 1 : 0);
    return query.exec();
}

bool ClearRememberedCredentials(QSqlDatabase& database, const QString& accountId)
{
    QSqlQuery remove(database);
    remove.prepare(QStringLiteral(
        "DELETE FROM settings WHERE scope_type='global' AND scope_id='' AND ("
        "(module IN ('LoginState/RememberedCredentials', 'LoginState/SavedPasswords') AND key_name=?) "
        "OR (module='LoginState' AND lower(key_name)='passwordbase64'))"));
    remove.addBindValue(accountId.trimmed());
    if (!remove.exec())
    {
        return false;
    }
    QSqlQuery disable(database);
    return disable.exec(QStringLiteral(
        "UPDATE settings SET value_text='0', encrypted=0, updated_at=datetime('now') "
        "WHERE scope_type='global' AND scope_id='' AND module='LoginState' "
        "AND key_name IN ('RememberPassword', 'AutoLogin')"));
}

bool ValidateAdministrator(QSqlDatabase& database, const QString& accountId)
{
    QString password;
    QString role;
    QString fingerprint;
    bool mustChange = true;
    return ReadAccountState(
            database, accountId, &password, &role, &mustChange, &fingerprint)
        && role == QLatin1String("admin")
        && !mustChange;
}

int AdministratorCount(QSqlDatabase& database)
{
    QSqlQuery query(database);
    if (!query.exec(QStringLiteral(
            "SELECT COUNT(*) FROM settings WHERE scope_type='account' "
            "AND module='Profile' AND key_name='Role' AND value_text='admin'"))
        || !query.next())
    {
        return -1;
    }
    return query.value(0).toInt();
}
}

bool ConfigDatabase::TryReadAuthenticationInitialized(bool* initialized)
{
    if (initialized == nullptr || !IsAvailable())
    {
        return false;
    }
    *initialized = false;
    ScopedSqliteConnection connection(QStringLiteral("read"));
    QSqlDatabase& database = connection.database();
    if (!database.isValid() || !database.isOpen())
    {
        return false;
    }
    QSqlQuery query(database);
    if (!query.exec(QStringLiteral(
            "SELECT value FROM meta WHERE key='auth_initialized' LIMIT 1"))
        || !query.next())
    {
        return false;
    }
    const QString value = query.value(0).toString();
    if (value != QLatin1String("0") && value != QLatin1String("1"))
    {
        return false;
    }
    *initialized = value == QLatin1String("1");
    return true;
}

bool ConfigDatabase::TryInitializeAuthenticationAccount(
    const QString& accountId,
    const QMap<QString, QString>& profileValues)
{
    static const QSet<QString> allowedKeys = {
        QStringLiteral("PasswordHash"),
        QStringLiteral("Role"),
        QStringLiteral("MustChangePassword"),
        QStringLiteral("CreatedAt"),
        QStringLiteral("PasswordChangedAt")
    };
    const QString normalizedAccountId = accountId.trimmed();
    bool mustChangePassword = true;
    if (normalizedAccountId.isEmpty()
        || profileValues.isEmpty()
        || profileValues.value(QStringLiteral("Role")) != QLatin1String("admin")
        || profileValues.value(QStringLiteral("PasswordHash")).isEmpty()
        || !ParseStrictBool(
            profileValues.value(QStringLiteral("MustChangePassword")),
            &mustChangePassword))
    {
        return false;
    }
    for (auto it = profileValues.cbegin(); it != profileValues.cend(); ++it)
    {
        if (!allowedKeys.contains(it.key()))
        {
            return false;
        }
    }
    if (!IsAvailable())
    {
        return false;
    }

    ScopedSqliteConnection connection(QStringLiteral("initialize"));
    QSqlDatabase& database = connection.database();
    if (!database.isValid() || !database.isOpen())
    {
        return false;
    }
    QSqlQuery begin(database);
    if (!begin.exec(QStringLiteral("BEGIN IMMEDIATE")))
    {
        return false;
    }
    QSqlQuery stateQuery(database);
    if (!stateQuery.exec(QStringLiteral(
            "SELECT value FROM meta WHERE key='auth_initialized' LIMIT 1"))
        || !stateQuery.next()
        || stateQuery.value(0).toString() != QLatin1String("0"))
    {
        return Rollback(database);
    }
    QSqlQuery countQuery(database);
    if (!countQuery.exec(QStringLiteral(
            "SELECT COUNT(*) FROM settings WHERE scope_type='account'"))
        || !countQuery.next()
        || countQuery.value(0).toLongLong() != 0)
    {
        return Rollback(database);
    }

    QSqlQuery insert(database);
    insert.prepare(QStringLiteral(
        "INSERT INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES('account', ?, 'Profile', ?, ?, ?, ?, 0, datetime('now'))"));
    for (auto it = profileValues.cbegin(); it != profileValues.cend(); ++it)
    {
        const bool sensitive = it.key().contains(
            QStringLiteral("Password"), Qt::CaseInsensitive);
        const QString valueType = it.key() == QLatin1String("MustChangePassword")
            ? QStringLiteral("bool")
            : (it.key().endsWith(QLatin1String("At"))
                ? QStringLiteral("datetime")
                : QStringLiteral("string"));
        insert.bindValue(0, normalizedAccountId);
        insert.bindValue(1, it.key());
        insert.bindValue(2, it.value());
        insert.bindValue(3, valueType);
        insert.bindValue(4, sensitive ? 1 : 0);
        if (!insert.exec())
        {
            return Rollback(database);
        }
        insert.finish();
    }

    QSqlQuery initialize(database);
    if (!initialize.exec(QStringLiteral(
            "UPDATE meta SET value='1' "
            "WHERE key='auth_initialized' AND value='0'"))
        || initialize.numRowsAffected() != 1)
    {
        return Rollback(database);
    }
    QSqlQuery commit(database);
    if (!commit.exec(QStringLiteral("COMMIT")))
    {
        return Rollback(database);
    }
    return true;
}

bool ConfigDatabase::TryCreateAccount(
    const QString& accountId,
    const QMap<QString, QString>& profileValues,
    const QString& administratorId)
{
    static const QSet<QString> allowedKeys = {
        QStringLiteral("PasswordHash"),
        QStringLiteral("Role"),
        QStringLiteral("MustChangePassword"),
        QStringLiteral("CreatedAt"),
        QStringLiteral("PasswordChangedAt")
    };
    const QString normalizedAccountId = accountId.trimmed();
    bool mustChange = false;
    if (normalizedAccountId.isEmpty()
        || profileValues.value(QStringLiteral("PasswordHash")).isEmpty()
        || !IsValidRole(profileValues.value(QStringLiteral("Role")))
        || !ParseStrictBool(
            profileValues.value(QStringLiteral("MustChangePassword")), &mustChange))
    {
        return false;
    }
    for (auto it = profileValues.cbegin(); it != profileValues.cend(); ++it)
    {
        if (!allowedKeys.contains(it.key()))
        {
            return false;
        }
    }
    if (!IsAvailable())
    {
        return false;
    }
    ScopedSqliteConnection connection(QStringLiteral("create_account"));
    QSqlDatabase& database = connection.database();
    if (!database.isOpen() || !BeginImmediate(database))
    {
        return false;
    }
    QSqlQuery initialized(database);
    if (!initialized.exec(QStringLiteral(
            "SELECT value FROM meta WHERE key='auth_initialized' LIMIT 1"))
        || !initialized.next()
        || initialized.value(0).toString() != QLatin1String("1"))
    {
        return Rollback(database);
    }
    if (!administratorId.trimmed().isEmpty()
        && !ValidateAdministrator(database, administratorId.trimmed()))
    {
        return Rollback(database);
    }
    QSqlQuery exists(database);
    exists.prepare(QStringLiteral(
        "SELECT COUNT(*) FROM settings WHERE scope_type='account' "
        "AND scope_id=? COLLATE NOCASE"));
    exists.addBindValue(normalizedAccountId);
    if (!exists.exec() || !exists.next() || exists.value(0).toInt() != 0)
    {
        return Rollback(database);
    }
    QSqlQuery insert(database);
    insert.prepare(QStringLiteral(
        "INSERT INTO settings("
        "scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at) "
        "VALUES('account', ?, 'Profile', ?, ?, ?, ?, 0, datetime('now'))"));
    for (auto it = profileValues.cbegin(); it != profileValues.cend(); ++it)
    {
        const bool sensitive = it.key().contains(
            QStringLiteral("Password"), Qt::CaseInsensitive);
        const QString valueType = it.key() == QLatin1String("MustChangePassword")
            ? QStringLiteral("bool")
            : (it.key().endsWith(QLatin1String("At"))
                ? QStringLiteral("datetime")
                : QStringLiteral("string"));
        insert.bindValue(0, normalizedAccountId);
        insert.bindValue(1, it.key());
        insert.bindValue(2, it.value());
        insert.bindValue(3, valueType);
        insert.bindValue(4, sensitive ? 1 : 0);
        if (!insert.exec())
        {
            return Rollback(database);
        }
        insert.finish();
    }
    QSqlQuery commit(database);
    return commit.exec(QStringLiteral("COMMIT")) || Rollback(database);
}

bool ConfigDatabase::TryReadAccountSecurityState(
    const QString& accountId,
    QString* passwordRecord,
    QString* role,
    bool* mustChangePassword,
    QString* fingerprint)
{
    if (!IsAvailable())
    {
        return false;
    }
    ScopedSqliteConnection connection(QStringLiteral("read_account"));
    QSqlDatabase& database = connection.database();
    return database.isOpen() && ReadAccountState(
        database,
        accountId.trimmed(),
        passwordRecord,
        role,
        mustChangePassword,
        fingerprint);
}

bool ConfigDatabase::TryCompareAndSetAccountPassword(
    const QString& accountId,
    const QString& expectedPasswordRecord,
    const QString& newPasswordRecord,
    bool requireMustChangePassword,
    bool newMustChangePassword,
    QString* currentRole,
    QString* newFingerprint)
{
    if (accountId.trimmed().isEmpty()
        || expectedPasswordRecord.isEmpty()
        || newPasswordRecord.isEmpty()
        || currentRole == nullptr
        || newFingerprint == nullptr
        || !IsAvailable())
    {
        return false;
    }
    ScopedSqliteConnection connection(QStringLiteral("password_cas"));
    QSqlDatabase& database = connection.database();
    if (!database.isOpen() || !BeginImmediate(database))
    {
        return false;
    }
    QString storedPassword;
    QString role;
    QString oldFingerprint;
    bool storedMustChange = false;
    if (!ReadAccountState(
            database, accountId, &storedPassword, &role,
            &storedMustChange, &oldFingerprint)
        || storedPassword != expectedPasswordRecord
        || (requireMustChangePassword && !storedMustChange))
    {
        return Rollback(database);
    }
    QSqlQuery updatePassword(database);
    updatePassword.prepare(QStringLiteral(
        "UPDATE settings SET value_text=?, sensitive=1, encrypted=0, updated_at=datetime('now') "
        "WHERE scope_type='account' AND scope_id=? AND module='Profile' "
        "AND key_name='PasswordHash' AND value_text=?"));
    updatePassword.addBindValue(newPasswordRecord);
    updatePassword.addBindValue(accountId.trimmed());
    updatePassword.addBindValue(expectedPasswordRecord);
    if (!updatePassword.exec() || updatePassword.numRowsAffected() != 1
        || !UpsertProfileValue(
            database, accountId, QStringLiteral("MustChangePassword"),
            newMustChangePassword ? QStringLiteral("1") : QStringLiteral("0"),
            QStringLiteral("bool"), true)
        || !UpsertProfileValue(
            database, accountId, QStringLiteral("PasswordChangedAt"),
            QDateTime::currentDateTime().toString(Qt::ISODate),
            QStringLiteral("datetime"), true)
        || !ClearRememberedCredentials(database, accountId))
    {
        return Rollback(database);
    }
    QSqlQuery commit(database);
    if (!commit.exec(QStringLiteral("COMMIT")))
    {
        return Rollback(database);
    }
    *currentRole = role;
    *newFingerprint = AccountFingerprint(newPasswordRecord, role, newMustChangePassword);
    return true;
}

bool ConfigDatabase::TryUpdateAccountByAdministrator(
    const QString& administratorId,
    const QString& accountId,
    const QString& newRole,
    const QString& newPasswordRecord)
{
    if (!IsValidRole(newRole) || !IsAvailable())
    {
        return false;
    }
    const QString actor = administratorId.trimmed();
    const QString target = accountId.trimmed();
    ScopedSqliteConnection connection(QStringLiteral("admin_update"));
    QSqlDatabase& database = connection.database();
    if (!database.isOpen() || !BeginImmediate(database)
        || !ValidateAdministrator(database, actor))
    {
        return Rollback(database);
    }
    QString targetPassword;
    QString currentRole;
    QString targetFingerprint;
    bool targetMustChange = false;
    if (!ReadAccountState(
            database, target, &targetPassword, &currentRole,
            &targetMustChange, &targetFingerprint))
    {
        return Rollback(database);
    }
    const bool changesRole = currentRole != newRole;
    const bool resetsPassword = !newPasswordRecord.isEmpty();
    if (actor.compare(target, Qt::CaseInsensitive) == 0
        && (changesRole || resetsPassword))
    {
        return Rollback(database);
    }
    if (currentRole == QLatin1String("admin")
        && newRole != QLatin1String("admin")
        && AdministratorCount(database) <= 1)
    {
        return Rollback(database);
    }
    QSqlQuery updateRole(database);
    updateRole.prepare(QStringLiteral(
        "UPDATE settings SET value_text=?, updated_at=datetime('now') "
        "WHERE scope_type='account' AND scope_id=? AND module='Profile' AND key_name='Role'"));
    updateRole.addBindValue(newRole);
    updateRole.addBindValue(target);
    if (!updateRole.exec() || updateRole.numRowsAffected() != 1)
    {
        return Rollback(database);
    }
    if (resetsPassword)
    {
        QSqlQuery updatePassword(database);
        updatePassword.prepare(QStringLiteral(
            "UPDATE settings SET value_text=?, sensitive=1, encrypted=0, updated_at=datetime('now') "
            "WHERE scope_type='account' AND scope_id=? AND module='Profile' AND key_name='PasswordHash'"));
        updatePassword.addBindValue(newPasswordRecord);
        updatePassword.addBindValue(target);
        if (!updatePassword.exec() || updatePassword.numRowsAffected() != 1
            || !UpsertProfileValue(
                database, target, QStringLiteral("MustChangePassword"), QStringLiteral("1"),
                QStringLiteral("bool"), true)
            || !UpsertProfileValue(
                database, target, QStringLiteral("PasswordChangedAt"),
                QDateTime::currentDateTime().toString(Qt::ISODate),
                QStringLiteral("datetime"), true)
            || !ClearRememberedCredentials(database, target))
        {
            return Rollback(database);
        }
    }
    QSqlQuery commit(database);
    return commit.exec(QStringLiteral("COMMIT")) || Rollback(database);
}

bool ConfigDatabase::TryDeleteAccountByAdministrator(
    const QString& administratorId,
    const QString& accountId)
{
    if (!IsAvailable())
    {
        return false;
    }
    const QString actor = administratorId.trimmed();
    const QString target = accountId.trimmed();
    if (actor.compare(target, Qt::CaseInsensitive) == 0)
    {
        return false;
    }
    ScopedSqliteConnection connection(QStringLiteral("admin_delete"));
    QSqlDatabase& database = connection.database();
    if (!database.isOpen() || !BeginImmediate(database)
        || !ValidateAdministrator(database, actor))
    {
        return Rollback(database);
    }
    QString targetPassword;
    QString targetRole;
    QString targetFingerprint;
    bool targetMustChange = false;
    if (!ReadAccountState(
            database, target, &targetPassword, &targetRole,
            &targetMustChange, &targetFingerprint)
        || (targetRole == QLatin1String("admin") && AdministratorCount(database) <= 1))
    {
        return Rollback(database);
    }
    QSqlQuery remove(database);
    remove.prepare(QStringLiteral(
        "DELETE FROM settings WHERE scope_type='account' AND scope_id=?"));
    remove.addBindValue(target);
    if (!remove.exec() || remove.numRowsAffected() < 1
        || !ClearRememberedCredentials(database, target))
    {
        return Rollback(database);
    }
    QSqlQuery commit(database);
    return commit.exec(QStringLiteral("COMMIT")) || Rollback(database);
}
