#include "AppPaths.h"
#include "ConfigDatabase.h"
#include "CredentialSecurity.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QPasswordDigestor>
#include <QSet>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QTemporaryDir>
#include <QTextStream>

#include <atomic>
#include <thread>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#ifndef SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE
#define SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE 0x2
#endif

namespace
{
void Check(bool condition, const QString& message)
{
    if (!condition)
    {
        QTextStream(stderr) << "FAIL: " << message << Qt::endl;
        std::exit(1);
    }
}

void CreateSettingsTable(QSqlDatabase& db, bool upperCaseNames = false)
{
    QSqlQuery query(db);
    const QString metaName = upperCaseNames
        ? QStringLiteral("META") : QStringLiteral("meta");
    const QString settingsName = upperCaseNames
        ? QStringLiteral("SETTINGS") : QStringLiteral("settings");
    Check(query.exec(QStringLiteral(
        "CREATE TABLE %1(key TEXT PRIMARY KEY, value TEXT NOT NULL)").arg(metaName)),
        QStringLiteral("create meta"));
    Check(query.exec(QStringLiteral(
        "CREATE TABLE %1("
        "scope_type TEXT NOT NULL, scope_id TEXT NOT NULL DEFAULT '', module TEXT NOT NULL, "
        "key_name TEXT NOT NULL, value_text TEXT NOT NULL, value_type TEXT NOT NULL DEFAULT 'string', "
        "sensitive INTEGER NOT NULL DEFAULT 0, encrypted INTEGER NOT NULL DEFAULT 0, "
        "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, "
        "PRIMARY KEY(scope_type, scope_id, module, key_name))").arg(settingsName)),
        QStringLiteral("create settings"));
}

void InsertRaw(
    QSqlDatabase& db,
    const QString& scope,
    const QString& scopeId,
    const QString& module,
    const QString& key,
    const QString& value,
    bool sensitive = false,
    bool encrypted = false,
    const QString& valueType = QStringLiteral("string"))
{
    QSqlQuery query(db);
    query.prepare(
        "INSERT INTO settings(scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?)");
    query.addBindValue(scope);
    query.addBindValue(scopeId.isNull() ? QStringLiteral("") : scopeId);
    query.addBindValue(module);
    query.addBindValue(key);
    query.addBindValue(value);
    query.addBindValue(valueType);
    query.addBindValue(sensitive ? 1 : 0);
    query.addBindValue(encrypted ? 1 : 0);
    const bool inserted = query.exec();
    Check(inserted, QStringLiteral("insert raw fixture %1/%2: %3")
        .arg(module, key, query.lastError().text()));
}

QString FileSha256(const QString& path)
{
    QFile file(path);
    Check(file.open(QIODevice::ReadOnly), QStringLiteral("open database for hash"));
    return QString::fromLatin1(QCryptographicHash::hash(file.readAll(), QCryptographicHash::Sha256).toHex());
}

QByteArray ReadFileBytes(const QString& path)
{
    QFile file(path);
    Check(file.open(QIODevice::ReadOnly), QStringLiteral("open fixture file for byte comparison"));
    return file.readAll();
}

QByteArray InstallerTransactionRecord(const QStringList& payloadLines)
{
    const QByteArray payload = payloadLines.join(QLatin1Char('\n')).toUtf8();
    return payload
        + QByteArrayLiteral("\nPAYLOAD_SHA256=")
        + QCryptographicHash::hash(payload, QCryptographicHash::Sha256).toHex()
        + QByteArrayLiteral("\n");
}

QString ProtectLegacyFixture(const QString& plainText)
{
    const QByteArray nonce = QByteArray::fromHex("00112233445566778899aabbccddeeff");
    QByteArray stream;
    int counter = 0;
    while (stream.size() < plainText.toUtf8().size())
    {
        QByteArray block("NoTeachingRobotConfigStoreV1");
        block.append(nonce);
        block.append(':');
        block.append(QByteArray::number(counter++));
        stream.append(QCryptographicHash::hash(block, QCryptographicHash::Sha256));
    }
    QByteArray bytes = plainText.toUtf8();
    stream.truncate(bytes.size());
    for (int index = 0; index < bytes.size(); ++index)
    {
        bytes[index] = static_cast<char>(
            static_cast<unsigned char>(bytes.at(index))
            ^ static_cast<unsigned char>(stream.at(index)));
    }
    return QStringLiteral("enc:v1:%1:%2")
        .arg(QString::fromLatin1(nonce.toBase64(QByteArray::OmitTrailingEquals)))
        .arg(QString::fromLatin1(bytes.toBase64(QByteArray::OmitTrailingEquals)));
}

int RunFutureSchemaTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("future schema temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("future data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("future_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open future fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '6')"), QStringLiteral("write schema 6"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("future_fixture"));
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize future data root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("future schema must be rejected"));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("future schema must stay rejected after connection reopen"));
    Check(FileSha256(dbPath) == before, QStringLiteral("future schema database must stay byte-identical"));
    QTextStream(stdout) << "PASS: future ConfigStore schema rejected without modification" << Qt::endl;
    return 0;
}

int RunPythonInteropTest(const QString& dataRoot)
{
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << dataRoot,
            &pathError),
        QStringLiteral("initialize Python migration root: %1").arg(pathError));
    Check(ConfigDatabase::IsAvailable(), QStringLiteral("open Python-generated ConfigStore"));
    bool authenticationInitialized = false;
    Check(
        ConfigDatabase::TryReadAuthenticationInitialized(&authenticationInitialized)
            && authenticationInitialized,
        QStringLiteral("Python migration marks existing authentication data initialized"));
    QString value;
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"), QString(), QStringLiteral("OnlineServices"),
            QStringLiteral("FtpPassword"), &value)
            && value == QStringLiteral("Synthetic-Python-DPAPI-Interop-Only"),
        QStringLiteral("C++ reads Python DPAPI credential with the same purpose"));
    Check(
        ConfigDatabase::ListScopedSettingIds(QStringLiteral("account"), QStringLiteral("Profile"))
            .contains(QStringLiteral("python_legacy_user")),
        QStringLiteral("C++ reads Python account semantic migration"));
    QString passwordRecord;
    QString role;
    QString fingerprint;
    bool mustChangePassword = true;
    Check(
        ConfigDatabase::TryReadAccountSecurityState(
            QStringLiteral("python_legacy_user"),
            &passwordRecord,
            &role,
            &mustChangePassword,
            &fingerprint)
            && CredentialSecurity::IsSupportedPasswordRecord(passwordRecord)
            && role == QStringLiteral("admin")
            && !mustChangePassword
            && !fingerprint.isEmpty(),
        QStringLiteral("Python account security fields are canonical for the raw C++ login API"));
    QTextStream(stdout) << "PASS: Python ConfigMigrate DPAPI is readable by C++ runtime" << Qt::endl;
    return 0;
}

int RunCorruptDpapiMigrationTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("corrupt DPAPI temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("corrupt DPAPI data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("corrupt_dpapi_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open corrupt DPAPI fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"), QStringLiteral("write corrupt fixture schema"));
        InsertRaw(
            db,
            QStringLiteral("global"),
            QString(),
            QStringLiteral("OnlineServices"),
            QStringLiteral("FtpPassword"),
            QStringLiteral("dpapi:user:v1:AAAA"),
            true,
            true);
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("corrupt_dpapi_fixture"));
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize corrupt DPAPI root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("corrupt DPAPI must abort v4 migration"));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("corrupt DPAPI must remain rejected after reopen"));
    Check(FileSha256(dbPath) == before, QStringLiteral("failed DPAPI migration must fully roll back"));
    QTextStream(stdout) << "PASS: corrupt DPAPI aborts migration with byte-identical rollback" << Qt::endl;
    return 0;
}

int RunAuthenticationInitializationTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("authentication initialization temp root"));
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize fresh authentication root: %1").arg(pathError));
    Check(ConfigDatabase::IsAvailable(), QStringLiteral("create fresh ConfigStore"));
    bool initialized = true;
    Check(
        ConfigDatabase::TryReadAuthenticationInitialized(&initialized) && !initialized,
        QStringLiteral("fresh database has an explicit uninitialized authentication marker"));

    const QString initialAdministratorRecord = CredentialSecurity::CreatePasswordRecord(
        QStringLiteral("Synthetic-Initial-Administrator-Test"));
    const QString currentAdministratorRecord = CredentialSecurity::CreatePasswordRecord(
        QStringLiteral("Synthetic-Current-Administrator-Test"));
    const QString secondAdministratorRecord = CredentialSecurity::CreatePasswordRecord(
        QStringLiteral("Synthetic-Second-Administrator-Test"));
    Check(!initialAdministratorRecord.isEmpty()
        && !currentAdministratorRecord.isEmpty()
        && !secondAdministratorRecord.isEmpty(),
        QStringLiteral("create supported authentication test records"));
    QMap<QString, QString> bootstrap;
    bootstrap.insert(QStringLiteral("PasswordHash"), initialAdministratorRecord);
    bootstrap.insert(QStringLiteral("Role"), QStringLiteral("admin"));
    bootstrap.insert(QStringLiteral("MustChangePassword"), QStringLiteral("1"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("auth_init_trigger"));
        db.setDatabaseName(temp.filePath(QStringLiteral("Data/ConfigStore.db")));
        Check(db.open(), QStringLiteral("open authentication rollback fixture"));
        QSqlQuery query(db);
        Check(query.exec(
            "CREATE TRIGGER reject_auth_initialization BEFORE UPDATE ON meta "
            "WHEN OLD.key='auth_initialized' BEGIN SELECT RAISE(ABORT, 'injected'); END"),
            QStringLiteral("create authentication marker failure trigger"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("auth_init_trigger"));
    Check(
        !ConfigDatabase::TryInitializeAuthenticationAccount(
            QStringLiteral("admin"), bootstrap),
        QStringLiteral("injected marker failure rejects bootstrap"));
    Check(
        ConfigDatabase::TryReadAuthenticationInitialized(&initialized) && !initialized
            && ConfigDatabase::ListScopedSettingIds(
                QStringLiteral("account"), QStringLiteral("Profile")).isEmpty(),
        QStringLiteral("marker failure rolls back the partial bootstrap account"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("auth_init_trigger_remove"));
        db.setDatabaseName(temp.filePath(QStringLiteral("Data/ConfigStore.db")));
        Check(db.open(), QStringLiteral("open authentication rollback fixture for cleanup"));
        QSqlQuery query(db);
        Check(query.exec("DROP TRIGGER reject_auth_initialization"), QStringLiteral("remove failure trigger"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("auth_init_trigger_remove"));
    QMap<QString, QString> userChosenAdministrator = bootstrap;
    userChosenAdministrator.insert(QStringLiteral("MustChangePassword"), QStringLiteral("0"));
    Check(
        ConfigDatabase::TryInitializeAuthenticationAccount(
            QStringLiteral("admin"), userChosenAdministrator),
        QStringLiteral("user-chosen initial administrator commits without a public temporary password"));
    Check(
        ConfigDatabase::TryReadAuthenticationInitialized(&initialized) && initialized,
        QStringLiteral("bootstrap account and initialization marker commit together"));

    QMap<QString, QString> loginPreferences;
    loginPreferences.insert(QStringLiteral("RememberPassword"), QStringLiteral("1"));
    loginPreferences.insert(QStringLiteral("AutoLogin"), QStringLiteral("1"));
    Check(
        ConfigDatabase::WriteScopedSetting(
            QStringLiteral("global"), QString(),
            QStringLiteral("LoginState/RememberedCredentials"),
            QStringLiteral("admin"), QStringLiteral("Synthetic-Remembered-Credential"),
            QStringLiteral("string"), true)
            && ConfigDatabase::WriteScopedSettings(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                loginPreferences, QStringLiteral("bool")),
        QStringLiteral("create real protected remembered-login fixture"));
    std::atomic_bool rememberedStateReopens{ false };
    std::thread rememberedStateProbe([&rememberedStateReopens]()
        {
            rememberedStateReopens = ConfigDatabase::IsAvailable();
        });
    rememberedStateProbe.join();
    Check(rememberedStateReopens.load(),
        QStringLiteral("protected remembered login state survives a fresh thread reopen"));
    Check(
        ConfigDatabase::WriteScopedSetting(
            QStringLiteral("global"), QString(),
            QStringLiteral("LoginState/SavedPasswords"),
            QStringLiteral("admin"), QStringLiteral("Synthetic-Saved-Password"),
            QStringLiteral("string"), true)
            && ConfigDatabase::WriteScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("PasswordBase64"), QStringLiteral("Synthetic-Base64"),
                QStringLiteral("string"), true),
        QStringLiteral("create deprecated remembered rows for credential-clear test"));

    QString currentRole;
    QString currentFingerprint;
    Check(
        ConfigDatabase::TryCompareAndSetAccountPassword(
            QStringLiteral("admin"),
            initialAdministratorRecord,
            currentAdministratorRecord,
            false,
            false,
            &currentRole,
            &currentFingerprint)
            && currentRole == QStringLiteral("admin")
            && !currentFingerprint.isEmpty(),
        QStringLiteral("forced password change uses compare-and-set"));
    QString removedCredential;
    Check(
        !ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"), QString(),
            QStringLiteral("LoginState/RememberedCredentials"),
            QStringLiteral("admin"), &removedCredential)
            && !ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/SavedPasswords"),
                QStringLiteral("admin"), &removedCredential)
            && !ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("PasswordBase64"), &removedCredential)
            && !ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("RememberPassword"), &removedCredential)
            && !ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("AutoLogin"), &removedCredential),
        QStringLiteral("password change removes all remembered credential state"));
    QString storedPassword;
    bool storedMustChange = true;
    QString storedFingerprint;
    Check(
        ConfigDatabase::TryReadAccountSecurityState(
            QStringLiteral("admin"), &storedPassword, &currentRole,
            &storedMustChange, &storedFingerprint)
            && storedPassword == currentAdministratorRecord
            && !storedMustChange
            && storedFingerprint == currentFingerprint,
        QStringLiteral("strict account security snapshot after password change"));
    Check(
        !ConfigDatabase::TryCompareAndSetAccountPassword(
            QStringLiteral("admin"),
            QStringLiteral("synthetic-test-record"),
            QStringLiteral("stale-write-must-fail"),
            false,
            false,
            &currentRole,
            &currentFingerprint),
        QStringLiteral("stale password snapshot cannot overwrite a newer password"));

    QMap<QString, QString> secondAdmin;
    secondAdmin.insert(QStringLiteral("PasswordHash"), secondAdministratorRecord);
    secondAdmin.insert(QStringLiteral("Role"), QStringLiteral("admin"));
    secondAdmin.insert(QStringLiteral("MustChangePassword"), QStringLiteral("0"));
    Check(
        ConfigDatabase::TryCreateAccount(
            QStringLiteral("second_admin"), secondAdmin, QStringLiteral("admin")),
        QStringLiteral("validated administrator creates a second account atomically"));
    Check(
        !ConfigDatabase::TryCreateAccount(
            QStringLiteral("SECOND_ADMIN"), secondAdmin, QStringLiteral("admin")),
        QStringLiteral("case-colliding account creation is rejected"));
    Check(
        !ConfigDatabase::TryUpdateAccountByAdministrator(
            QStringLiteral("admin"), QStringLiteral("admin"), QStringLiteral("operator")),
        QStringLiteral("administrator cannot demote the active actor"));
    Check(
        !ConfigDatabase::TryUpdateAccountByAdministrator(
            QStringLiteral("admin"), QStringLiteral("admin"), QStringLiteral("admin"),
            QStringLiteral("self-reset-must-fail")),
        QStringLiteral("administrator cannot reset the active actor password"));

    std::atomic_int deleteSuccesses{ 0 };
    std::thread firstDelete([&deleteSuccesses]()
        {
            if (ConfigDatabase::TryDeleteAccountByAdministrator(
                    QStringLiteral("admin"), QStringLiteral("second_admin")))
            {
                ++deleteSuccesses;
            }
        });
    std::thread secondDelete([&deleteSuccesses]()
        {
            if (ConfigDatabase::TryDeleteAccountByAdministrator(
                    QStringLiteral("second_admin"), QStringLiteral("admin")))
            {
                ++deleteSuccesses;
            }
        });
    firstDelete.join();
    secondDelete.join();
    Check(deleteSuccesses.load() == 1, QStringLiteral("concurrent administrators cannot delete each other"));
    int remainingAdministrators = 0;
    for (const QString& account : ConfigDatabase::ListScopedSettingIds(
            QStringLiteral("account"), QStringLiteral("Profile")))
    {
        QString passwordRecord;
        QString role;
        QString fingerprint;
        bool mustChange = true;
        if (ConfigDatabase::TryReadAccountSecurityState(
                account, &passwordRecord, &role, &mustChange, &fingerprint)
            && role == QStringLiteral("admin"))
        {
            ++remainingAdministrators;
        }
    }
    Check(remainingAdministrators == 1, QStringLiteral("exactly one administrator survives concurrent deletion"));
    const int connectionCountBeforeStress = QSqlDatabase::connectionNames().size();
    std::atomic_int threadConnectionSuccesses{ 0 };
    for (int iteration = 0; iteration < 64; ++iteration)
    {
        std::thread probe([&threadConnectionSuccesses]()
            {
                if (ConfigDatabase::IsAvailable())
                {
                    ++threadConnectionSuccesses;
                }
            });
        probe.join();
    }
    Check(
        threadConnectionSuccesses.load() == 64
            && QSqlDatabase::connectionNames().size() == connectionCountBeforeStress,
        QStringLiteral("thread-id reuse cannot retain or cross-use ConfigStore connections"));
    const QString survivingAccount = ConfigDatabase::ListScopedSettingIds(
        QStringLiteral("account"), QStringLiteral("Profile")).value(0);
    Check(
        !survivingAccount.isEmpty()
            && ConfigDatabase::WriteScopedSetting(
                QStringLiteral("account"), survivingAccount, QStringLiteral("Profile"),
                QStringLiteral("MustChangePassword"), QStringLiteral("invalid"),
                QStringLiteral("bool")),
        QStringLiteral("inject malformed MustChangePassword fixture"));
    Check(
        !ConfigDatabase::TryReadAccountSecurityState(
            survivingAccount, &storedPassword, &currentRole,
            &storedMustChange, &storedFingerprint),
        QStringLiteral("malformed MustChangePassword fails closed"));

    QMap<QString, QString> secondBootstrap = bootstrap;
    const QString rejectedBootstrapProbe = QStringLiteral("rejected_bootstrap_probe");
    Check(
        !ConfigDatabase::TryInitializeAuthenticationAccount(
            rejectedBootstrapProbe, secondBootstrap),
        QStringLiteral("second bootstrap transaction is rejected"));
    Check(
        !ConfigDatabase::ListScopedSettingIds(QStringLiteral("account"), QStringLiteral("Profile"))
            .contains(rejectedBootstrapProbe),
        QStringLiteral("rejected bootstrap leaves no partial account"));

    for (const QString& account : ConfigDatabase::ListScopedSettingIds(
            QStringLiteral("account"), QStringLiteral("Profile")))
    {
        Check(
            ConfigDatabase::RemoveScopedSettings(
                QStringLiteral("account"), account, QStringLiteral("Profile")),
            QStringLiteral("simulate an initialized account store being cleared"));
    }
    Check(
        !ConfigDatabase::TryInitializeAuthenticationAccount(
            QStringLiteral("admin"), bootstrap),
        QStringLiteral("cleared initialized account store cannot recreate the known bootstrap"));
    QTextStream(stdout) << "PASS: authentication bootstrap is one-time and atomic" << Qt::endl;
    return 0;
}

int RunEmptyV4RecoveryTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("empty v4 recovery temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("empty v4 data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("empty_v4_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open empty v4 fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"), QStringLiteral("write empty v4 schema"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("empty_v4_fixture"));
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize empty v4 root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("empty initialized v4 store fails closed"));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("empty v4 store remains unavailable"));
    Check(FileSha256(dbPath) == before,
        QStringLiteral("empty v4 administrator-integrity failure rolls back byte-identically"));
    QTextStream(stdout) << "PASS: empty existing ConfigStore fails admin integrity without modification" << Qt::endl;
    return 0;
}

int RunInvalidAuthenticationMetadataTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("invalid authentication metadata temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("invalid metadata data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("invalid_auth_meta_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open invalid authentication metadata fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '5')"), QStringLiteral("write current schema"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', '1')"), QStringLiteral("write auth semantic version"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_initialized', 'invalid')"), QStringLiteral("write invalid auth marker"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("invalid_auth_meta_fixture"));
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize invalid metadata root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("invalid auth marker must be rejected"));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("invalid auth marker remains rejected after reopen"));
    Check(FileSha256(dbPath) == before, QStringLiteral("invalid auth metadata database stays byte-identical"));
    QTextStream(stdout) << "PASS: invalid authentication metadata is rejected without modification" << Qt::endl;
    return 0;
}

int RunAuthenticationSemanticVersionGateTest(const QString& scenario)
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("authentication semantic gate temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))),
        QStringLiteral("authentication semantic gate data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString connectionName = QStringLiteral("auth_semantic_gate_%1").arg(scenario);
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open authentication semantic gate fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '5')"),
            QStringLiteral("write authentication semantic gate schema"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_initialized', '0')"),
            QStringLiteral("write authentication semantic gate initialization marker"));
        if (scenario == QStringLiteral("invalid"))
        {
            Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', 'invalid')"),
                QStringLiteral("write invalid authentication semantic version"));
        }
        else if (scenario == QStringLiteral("future"))
        {
            Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', '999')"),
                QStringLiteral("write future authentication semantic version"));
        }
        else
        {
            Check(scenario == QStringLiteral("missing"),
                QStringLiteral("known authentication semantic gate scenario"));
        }
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);

    const QByteArray before = ReadFileBytes(dbPath);
    QString pathError;
    Check(AppPaths::Initialize(
        QStringList() << QStringLiteral("CredentialSecurityTests")
                      << QStringLiteral("--data-root") << temp.path(),
        &pathError),
        QStringLiteral("initialize authentication semantic gate root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("authentication semantic %1 must be rejected").arg(scenario));
    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("authentication semantic %1 remains rejected after reopen").arg(scenario));
    Check(ReadFileBytes(dbPath) == before,
        QStringLiteral("authentication semantic %1 remains byte-identical").arg(scenario));
    QTextStream(stdout) << "PASS: authentication semantic " << scenario
        << " is rejected without modification" << Qt::endl;
    return 0;
}

int RunLegacyDiskInputGateTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("legacy disk input gate temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("legacy disk data dir"));
    const QString accountsPath = temp.filePath(QStringLiteral("Data/Accounts.ini"));
    const QByteArray source = QByteArrayLiteral(
        "[Users/legacy_operator]\nPasswordHash=synthetic-legacy-record\nRole=operator\n");
    {
        QFile file(accountsPath);
        Check(file.open(QIODevice::WriteOnly), QStringLiteral("write legacy disk input fixture"));
        Check(file.write(source) == source.size(), QStringLiteral("write complete legacy disk fixture"));
    }
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize legacy disk gate root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("runtime refuses to bypass legacy disk migration"));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("legacy disk gate remains closed after reopen"));
    Check(
        !QFileInfo::exists(temp.filePath(QStringLiteral("Data/ConfigStore.db"))),
        QStringLiteral("legacy disk gate does not create an empty ConfigStore"));
    QFile sourceAfter(accountsPath);
    Check(sourceAfter.open(QIODevice::ReadOnly), QStringLiteral("reopen legacy disk fixture"));
    Check(sourceAfter.readAll() == source, QStringLiteral("runtime gate leaves legacy credentials byte-identical"));
    QTextStream(stdout) << "PASS: runtime requires ConfigMigrate before opening legacy disk configuration" << Qt::endl;
    return 0;
}

int RunCredentialScrubPendingGateTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("pending scrub gate temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("pending scrub data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("pending_scrub_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open pending scrub fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '5')"), QStringLiteral("write pending schema"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', '3')"), QStringLiteral("write pending auth semantic"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_initialized', '1')"), QStringLiteral("write pending auth marker"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('legacy_credential_scrub_state', 'pending')"), QStringLiteral("write pending scrub state"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("pending_scrub_fixture"));
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize pending scrub root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("runtime rejects pending credential scrub provenance"));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("pending credential scrub remains rejected"));
    Check(FileSha256(dbPath) == before, QStringLiteral("pending scrub rejection leaves database byte-identical"));
    QTextStream(stdout) << "PASS: runtime rejects incomplete legacy credential scrub provenance" << Qt::endl;
    return 0;
}

int RunPlaintextBackupGateTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("plaintext backup gate temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("plaintext backup data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("plaintext_backup_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open plaintext backup fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '5')"), QStringLiteral("write backup schema"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', '3')"), QStringLiteral("write backup auth semantic"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_initialized', '0')"), QStringLiteral("write backup auth marker"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("plaintext_backup_fixture"));
    const QString backupPath = temp.filePath(QStringLiteral("Data/ConfigStore.db.bak_legacy"));
    const QByteArray backupContent = QByteArrayLiteral("SQLite format 3\0Synthetic-Plaintext-Credential-Backup");
    {
        QFile backup(backupPath);
        Check(backup.open(QIODevice::WriteOnly), QStringLiteral("create plaintext backup residue"));
        Check(backup.write(backupContent) == backupContent.size(), QStringLiteral("write plaintext backup residue"));
    }
    const QString databaseBefore = FileSha256(dbPath);
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize plaintext backup root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(), QStringLiteral("runtime rejects plaintext ConfigStore backup residue"));
    Check(FileSha256(dbPath) == databaseBefore, QStringLiteral("backup gate leaves current database byte-identical"));
    QFile backupAfter(backupPath);
    Check(backupAfter.open(QIODevice::ReadOnly), QStringLiteral("reopen plaintext backup residue"));
    Check(backupAfter.readAll() == backupContent, QStringLiteral("runtime never edits the unproven plaintext backup"));
    QTextStream(stdout) << "PASS: runtime rejects plaintext ConfigStore backup residue" << Qt::endl;
    return 0;
}

QString LegacyPasswordHash(const QString& userName, const QString& password)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        QStringLiteral("%1\n%2").arg(userName, password).toUtf8(),
        QCryptographicHash::Sha256).toHex());
}

void InsertFlatLegacyAccount(
    QSqlDatabase& db,
    const QString& userName,
    const QString& passwordHash,
    const QString& role)
{
    InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"),
        userName + QStringLiteral("/PasswordHash"), passwordHash, true);
    InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"),
        userName + QStringLiteral("/Role"), role);
}

void InsertNestedLegacyAccount(
    QSqlDatabase& db,
    const QString& userName,
    const QString& passwordHash,
    const QString& role)
{
    const QString module = QStringLiteral("Accounts/Users/") + userName;
    InsertRaw(db, QStringLiteral("global"), QString(), module,
        QStringLiteral("PasswordHash"), passwordHash, true);
    InsertRaw(db, QStringLiteral("global"), QString(), module,
        QStringLiteral("Role"), role);
}

void InsertCurrentAuthenticationMetadata(
    QSqlDatabase& db,
    bool authenticationInitialized,
    const QString& authenticationSemanticVersion = QStringLiteral("3"))
{
    QSqlQuery query(db);
    for (const auto& item : {
            qMakePair(QStringLiteral("schema_version"), QStringLiteral("5")),
            qMakePair(
                QStringLiteral("auth_semantic_version"),
                authenticationSemanticVersion),
            qMakePair(
                QStringLiteral("auth_initialized"),
                authenticationInitialized ? QStringLiteral("1") : QStringLiteral("0")),
            qMakePair(QStringLiteral("encrypt_new_values"), QStringLiteral("0")),
            qMakePair(
                QStringLiteral("sensitive_protection"),
                QStringLiteral("dpapi-current-user-v1")) })
    {
        query.prepare(QStringLiteral("INSERT INTO meta(key, value) VALUES(?, ?)"));
        query.addBindValue(item.first);
        query.addBindValue(item.second);
        Check(query.exec(), QStringLiteral("insert current metadata %1").arg(item.first));
    }
}

void InsertCurrentAccountProfile(
    QSqlDatabase& db,
    const QString& userName,
    const QString& role)
{
    InsertRaw(
        db, QStringLiteral("account"), userName, QStringLiteral("Profile"),
        QStringLiteral("PasswordHash"),
        LegacyPasswordHash(userName, QStringLiteral("Current-Integrity-Test")),
        true);
    InsertRaw(
        db, QStringLiteral("account"), userName, QStringLiteral("Profile"),
        QStringLiteral("Role"), role);
    InsertRaw(
        db, QStringLiteral("account"), userName, QStringLiteral("Profile"),
        QStringLiteral("MustChangePassword"), QStringLiteral("0"), true);
}

void CreateCurrentCompleteAuthenticationFixture(const QString& databasePath)
{
    static std::atomic_uint fixtureId{ 0 };
    const QString connectionName = QStringLiteral("installer_transaction_fixture_%1")
        .arg(fixtureId.fetch_add(1));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
        db.setDatabaseName(databasePath);
        Check(db.open(), QStringLiteral("open installer transaction fixture"));
        CreateSettingsTable(db);
        InsertCurrentAuthenticationMetadata(db, true);
        InsertCurrentAccountProfile(db, QStringLiteral("admin"), QStringLiteral("admin"));
        QSqlQuery query(db);
        Check(query.exec(QStringLiteral(
            "INSERT INTO meta(key, value) VALUES('legacy_credential_scrub_state', 'complete')")),
            QStringLiteral("write complete credential scrub state"));
        Check(query.exec(QStringLiteral(
            "INSERT INTO meta(key, value) VALUES('legacy_credential_scrub_manifest', '[]')")),
            QStringLiteral("write complete credential scrub manifest"));
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);
}

int RunLegacyLoginSemanticUpgradeTest(const QString& scenario)
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("legacy login semantic temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))),
        QStringLiteral("legacy login semantic data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString connectionName = QStringLiteral("legacy_login_semantic_") + scenario;
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open legacy login semantic fixture"));
        CreateSettingsTable(db);
        InsertCurrentAuthenticationMetadata(
            db, true,
            scenario == QStringLiteral("current-settings-orphan")
                ? QStringLiteral("3") : QStringLiteral("2"));
        InsertCurrentAccountProfile(db, QStringLiteral("admin"), QStringLiteral("admin"));

        if (scenario == QStringLiteral("success"))
        {
            const QString accountHistory = QStringLiteral("admin,operator");
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("UserName"),
                QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/General"), QStringLiteral("UserName"),
                QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("UserName"),
                QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/General"), QStringLiteral("AccountHistory"),
                accountHistory);
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("AccountHistory"),
                accountHistory);
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/General"), QStringLiteral("RememberPassword"),
                QStringLiteral("1"), true, false, QStringLiteral("bool"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("AutoLogin"),
                QStringLiteral("1"), false, false, QStringLiteral("string"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("PasswordBase64"),
                QStringLiteral("c3ludGhldGljLXRlc3Qtb25seQ=="));
        }
        else if (scenario == QStringLiteral("module-case"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("loginstate/settings"), QStringLiteral("UserName"),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("field-case"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("username"),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("source-conflict"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/General"), QStringLiteral("UserName"),
                QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("UserName"),
                QStringLiteral("operator"));
        }
        else if (scenario == QStringLiteral("target-conflict"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("UserName"),
                QStringLiteral("current-admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("UserName"),
                QStringLiteral("stale-admin"));
        }
        else if (scenario == QStringLiteral("current-settings-orphan"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/Settings"), QStringLiteral("AutoLogin"),
                QStringLiteral("1"), false, false, QStringLiteral("string"));
        }
        else
        {
            Check(false, QStringLiteral("unknown legacy login semantic scenario"));
        }
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);

    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(AppPaths::Initialize(
        QStringList() << QStringLiteral("CredentialSecurityTests")
                      << QStringLiteral("--data-root") << temp.path(),
        &pathError), QStringLiteral("initialize legacy login semantic root: %1").arg(pathError));
    const bool shouldOpen = scenario == QStringLiteral("success");
    Check(ConfigDatabase::IsAvailable() == shouldOpen,
        QStringLiteral("legacy login semantic result for %1").arg(scenario));
    const QString afterFirstOpen = FileSha256(dbPath);
    Check(ConfigDatabase::IsAvailable() == shouldOpen,
        QStringLiteral("legacy login semantic result remains stable for %1").arg(scenario));
    if (!shouldOpen)
    {
        Check(afterFirstOpen == before && FileSha256(dbPath) == before,
            QStringLiteral("legacy login semantic %1 rolls back byte-identically").arg(scenario));
        QTextStream(stdout) << "PASS: legacy login semantic " << scenario
            << " fails closed with byte-identical rollback" << Qt::endl;
        return 0;
    }

    Check(afterFirstOpen != before && FileSha256(dbPath) == afterFirstOpen,
        QStringLiteral("auth2 login semantic upgrade commits once"));
    QString userName;
    QString accountHistory;
    QString rememberPassword;
    QString autoLogin;
    Check(ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
            QStringLiteral("UserName"), &userName)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("AccountHistory"), &accountHistory)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("RememberPassword"), &rememberPassword)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
                QStringLiteral("AutoLogin"), &autoLogin)
            && userName == QLatin1String("admin")
            && accountHistory == QLatin1String("admin,operator")
            && rememberPassword == QLatin1String("0")
            && autoLogin == QLatin1String("0"),
        QStringLiteral("auth2 login state preserves identity and disables remembered login"));

    QString passwordRecord;
    QString role;
    QString fingerprint;
    bool mustChangePassword = true;
    Check(ConfigDatabase::TryReadAccountSecurityState(
            QStringLiteral("admin"), &passwordRecord, &role,
            &mustChangePassword, &fingerprint)
            && role == QLatin1String("admin")
            && !passwordRecord.isEmpty()
            && !fingerprint.isEmpty(),
        QStringLiteral("auth2 login semantic upgrade preserves administrator"));

    const QString verifyConnection = QStringLiteral("legacy_login_semantic_verify");
    {
        QSqlDatabase verifyDb = QSqlDatabase::addDatabase(
            QStringLiteral("QSQLITE"), verifyConnection);
        verifyDb.setDatabaseName(dbPath);
        Check(verifyDb.open(), QStringLiteral("open upgraded login semantic verification"));
        QSqlQuery verifyQuery(verifyDb);
        Check(verifyQuery.exec(QStringLiteral(
                "SELECT value FROM meta WHERE key='auth_semantic_version'"))
                && verifyQuery.next()
                && verifyQuery.value(0).toString() == QLatin1String("3")
                && !verifyQuery.next(),
            QStringLiteral("auth semantic version upgraded to 3"));
        Check(verifyQuery.exec(QStringLiteral(
                "SELECT COUNT(*) FROM settings WHERE "
                "lower(module) IN ('loginstate/general', 'loginstate/settings')"))
                && verifyQuery.next() && verifyQuery.value(0).toInt() == 0,
            QStringLiteral("legacy General and Settings modules removed"));
        Check(verifyQuery.exec(QStringLiteral(
                "SELECT value_text, value_type, sensitive, encrypted FROM settings "
                "WHERE scope_type='global' AND scope_id='' AND module='LoginState' "
                "AND key_name IN ('RememberPassword','AutoLogin') ORDER BY key_name")),
            QStringLiteral("read canonical disabled login preference storage"));
        int verifiedPreferences = 0;
        while (verifyQuery.next())
        {
            Check(CredentialSecurity::IsCurrentUserProtected(
                    verifyQuery.value(0).toString())
                    && verifyQuery.value(1).toString() == QLatin1String("bool")
                    && verifyQuery.value(2).toInt() == 1
                    && verifyQuery.value(3).toInt() == 1,
                QStringLiteral("disabled login preference is canonical DPAPI bool"));
            ++verifiedPreferences;
        }
        Check(verifiedPreferences == 2,
            QStringLiteral("both canonical login preferences were reset"));
        verifyDb.close();
    }
    QSqlDatabase::removeDatabase(verifyConnection);
    QTextStream(stdout)
        << "PASS: auth2 LoginState General/Settings upgrades to auth3 safely"
        << Qt::endl;
    return 0;
}

bool CreateTestFileSymbolicLink(const QString& linkPath, const QString& targetPath)
{
    if (CreateSymbolicLinkW(
            reinterpret_cast<LPCWSTR>(linkPath.utf16()),
            reinterpret_cast<LPCWSTR>(targetPath.utf16()),
            SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE) != FALSE)
    {
        return true;
    }
    if (GetLastError() == ERROR_INVALID_PARAMETER)
    {
        return CreateSymbolicLinkW(
            reinterpret_cast<LPCWSTR>(linkPath.utf16()),
            reinterpret_cast<LPCWSTR>(targetPath.utf16()),
            0) != FALSE;
    }
    return false;
}

bool RemoveTestFilesystemObject(const QString& path, bool directory)
{
    if (directory)
    {
        return QDir(path).removeRecursively();
    }
    if (QFile::remove(path))
    {
        return true;
    }
    return DeleteFileW(reinterpret_cast<LPCWSTR>(path.utf16())) != FALSE;
}

int RunInstallerTransactionGateTest(const QString& scenario)
{
    const QSet<QString> supportedScenarios = {
        QStringLiteral("valid"),
        QStringLiteral("arbitrary"),
        QStringLiteral("directory"),
        QStringLiteral("symlink"),
        QStringLiteral("dangling-symlink"),
        QStringLiteral("existing-connection"),
        QStringLiteral("existing-connection-pending"),
        QStringLiteral("existing-connection-corrupt"),
        QStringLiteral("noop-absent")
    };
    Check(supportedScenarios.contains(scenario),
        QStringLiteral("known installer transaction gate scenario"));

    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("installer transaction gate temp root"));
    const QString dataPath = temp.filePath(QStringLiteral("Data"));
    Check(QDir().mkpath(dataPath), QStringLiteral("installer transaction gate data dir"));
    const QString databasePath = QDir(dataPath).filePath(QStringLiteral("ConfigStore.db"));
    const QString transactionPath = QDir(dataPath).filePath(
        QStringLiteral("ConfigStore.db.install-transaction-v1"));
    const bool databaseInitiallyAbsent = scenario == QStringLiteral("noop-absent");
    if (!databaseInitiallyAbsent)
    {
        CreateCurrentCompleteAuthenticationFixture(databasePath);
    }

    QString pathError;
    const bool connectionPreopened = scenario.startsWith(
        QStringLiteral("existing-connection"));
    if (connectionPreopened)
    {
        Check(AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError), QStringLiteral("initialize pre-open transaction root: %1").arg(pathError));
        Check(ConfigDatabase::IsAvailable(),
            QStringLiteral("open current database before installer record appears"));
    }

    QByteArray expectedRecordBytes;
    QString linkedTargetPath;
    const bool recordIsDirectory = scenario == QStringLiteral("directory");
    const bool recordIsSymlink = scenario == QStringLiteral("symlink")
        || scenario == QStringLiteral("dangling-symlink");
    if (recordIsDirectory)
    {
        Check(QDir().mkpath(transactionPath), QStringLiteral("create transaction directory barrier"));
        QFile sentinel(QDir(transactionPath).filePath(QStringLiteral("sentinel.bin")));
        expectedRecordBytes = QByteArrayLiteral("transaction-directory-must-stay-byte-identical\0payload");
        Check(sentinel.open(QIODevice::WriteOnly), QStringLiteral("create transaction directory sentinel"));
        Check(sentinel.write(expectedRecordBytes) == expectedRecordBytes.size(),
            QStringLiteral("write transaction directory sentinel"));
        sentinel.close();
    }
    else if (recordIsSymlink)
    {
        linkedTargetPath = temp.filePath(QStringLiteral("transaction-record-target"));
        if (scenario == QStringLiteral("symlink"))
        {
            QFile target(linkedTargetPath);
            expectedRecordBytes = QByteArrayLiteral("linked-installer-transaction-record");
            Check(target.open(QIODevice::WriteOnly), QStringLiteral("create transaction symlink target"));
            Check(target.write(expectedRecordBytes) == expectedRecordBytes.size(),
                QStringLiteral("write transaction symlink target"));
            target.close();
        }
        if (!CreateTestFileSymbolicLink(transactionPath, linkedTargetPath))
        {
            QTextStream(stdout)
                << "SKIP: Windows cannot create installer transaction symlink; "
                   "source-level dangling-link enumeration remains required"
                << Qt::endl;
            return 0;
        }
        Check(QFileInfo(transactionPath).isSymLink(),
            QStringLiteral("installer transaction fixture is a symbolic link"));
    }
    else
    {
        if (scenario == QStringLiteral("arbitrary"))
        {
            expectedRecordBytes = QByteArray::fromHex("00ff10434f52525550540d0a")
                + QByteArrayLiteral("not-a-valid-transaction");
        }
        else if (databaseInitiallyAbsent)
        {
            expectedRecordBytes = InstallerTransactionRecord({
                QStringLiteral("FORMAT=NoTeaching-Robot-Install-Transaction-v1"),
                QStringLiteral("MODE=NOOP_ABSENT"),
                QStringLiteral("SOURCE_INVENTORY_SHA256=") + QString(64, QLatin1Char('0')),
                QStringLiteral("ORIGINAL_STATE=ABSENT")
            });
        }
        else
        {
            expectedRecordBytes = InstallerTransactionRecord({
                QStringLiteral("FORMAT=NoTeaching-Robot-Install-Transaction-v1"),
                QStringLiteral("MODE=NOOP_CURRENT"),
                QStringLiteral("CURRENT_SHA256=") + FileSha256(databasePath)
            });
        }
        QFile record(transactionPath);
        Check(record.open(QIODevice::WriteOnly | QIODevice::NewOnly),
            QStringLiteral("create installer transaction record"));
        Check(record.write(expectedRecordBytes) == expectedRecordBytes.size(),
            QStringLiteral("write installer transaction record"));
        record.close();
    }

    if (!connectionPreopened)
    {
        Check(AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError), QStringLiteral("initialize transaction gate root: %1").arg(pathError));
    }

    const QString databaseHashBefore = databaseInitiallyAbsent
        ? QString() : FileSha256(databasePath);
    QMap<QString, QString> blockedBootstrap;
    blockedBootstrap.insert(
        QStringLiteral("PasswordHash"),
        CredentialSecurity::CreatePasswordRecord(QStringLiteral("Blocked-Installer-Account-Test")));
    blockedBootstrap.insert(QStringLiteral("Role"), QStringLiteral("admin"));
    blockedBootstrap.insert(QStringLiteral("MustChangePassword"), QStringLiteral("1"));
    Check(!blockedBootstrap.value(QStringLiteral("PasswordHash")).isEmpty(),
        QStringLiteral("create supported blocked bootstrap record"));

    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("installer transaction blocks database availability"));
    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("installer transaction repeatedly blocks database availability"));
    bool initialized = true;
    Check(!ConfigDatabase::TryReadAuthenticationInitialized(&initialized),
        QStringLiteral("installer transaction blocks authentication metadata reads"));
    Check(!ConfigDatabase::TryInitializeAuthenticationAccount(
            QStringLiteral("blocked_bootstrap"), blockedBootstrap),
        QStringLiteral("installer transaction blocks default account bootstrap"));
    Check(!ConfigDatabase::TryCreateAccount(
            QStringLiteral("blocked_created"), blockedBootstrap),
        QStringLiteral("installer transaction blocks account creation"));
    QString blockedValue;
    Check(ConfigDatabase::ReadScopedSettingStatus(
            QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
            QStringLiteral("AutoLogin"), &blockedValue) == ConfigDatabase::ReadStatus::Error,
        QStringLiteral("installer transaction blocks login-state reads"));
    Check(!ConfigDatabase::WriteScopedSetting(
            QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
            QStringLiteral("AutoLogin"), QStringLiteral("1"), QStringLiteral("bool")),
        QStringLiteral("installer transaction blocks login-state writes"));
    std::atomic_bool threadAvailable{ true };
    std::thread threadProbe([&threadAvailable]()
        {
            threadAvailable = ConfigDatabase::IsAvailable();
        });
    threadProbe.join();
    Check(!threadAvailable.load(),
        QStringLiteral("installer transaction blocks fresh-thread database access"));

    if (databaseInitiallyAbsent)
    {
        Check(!QFileInfo::exists(databasePath),
            QStringLiteral("NOOP_ABSENT transaction never creates ConfigStore"));
    }
    else
    {
        Check(FileSha256(databasePath) == databaseHashBefore,
            QStringLiteral("installer transaction gate leaves ConfigStore byte-identical"));
    }
    if (recordIsDirectory)
    {
        Check(ReadFileBytes(QDir(transactionPath).filePath(QStringLiteral("sentinel.bin")))
                == expectedRecordBytes,
            QStringLiteral("transaction directory remains byte-identical"));
        Check(QDir(transactionPath).entryList(
                QDir::AllEntries | QDir::NoDotAndDotDot).size() == 1,
            QStringLiteral("transaction directory inventory remains unchanged"));
    }
    else if (scenario == QStringLiteral("dangling-symlink"))
    {
        Check(QFileInfo(transactionPath).isSymLink() && !QFileInfo::exists(linkedTargetPath),
            QStringLiteral("dangling transaction symlink remains unchanged"));
    }
    else
    {
        Check(ReadFileBytes(transactionPath) == expectedRecordBytes,
            QStringLiteral("installer transaction record remains byte-identical"));
    }

    const bool invalidateWhileBlocked = scenario == QStringLiteral("existing-connection-pending")
        || scenario == QStringLiteral("existing-connection-corrupt");
    QString invalidDatabaseHash;
    if (invalidateWhileBlocked)
    {
        const QString tamperConnectionName = QStringLiteral("blocked_database_tamper_") + scenario;
        {
            QSqlDatabase tamperDatabase = QSqlDatabase::addDatabase(
                QStringLiteral("QSQLITE"), tamperConnectionName);
            tamperDatabase.setDatabaseName(databasePath);
            Check(tamperDatabase.open(),
                QStringLiteral("open externally modified database while runtime is blocked"));
            QSqlQuery query(tamperDatabase);
            if (scenario == QStringLiteral("existing-connection-pending"))
            {
                Check(query.exec(QStringLiteral(
                    "UPDATE meta SET value='pending' "
                    "WHERE key='legacy_credential_scrub_state'"))
                        && query.numRowsAffected() == 1,
                    QStringLiteral("externally mark credential scrub pending while blocked"));
            }
            else
            {
                Check(query.exec(QStringLiteral(
                    "UPDATE meta SET value='invalid' WHERE key='auth_initialized'"))
                        && query.numRowsAffected() == 1,
                    QStringLiteral("externally corrupt authentication metadata while blocked"));
            }
            tamperDatabase.close();
        }
        QSqlDatabase::removeDatabase(tamperConnectionName);
        invalidDatabaseHash = FileSha256(databasePath);
        Check(invalidDatabaseHash != databaseHashBefore,
            QStringLiteral("external blocked-period database change is observable"));
        Check(ReadFileBytes(transactionPath) == expectedRecordBytes,
            QStringLiteral("external database change does not alter transaction record"));
    }

    Check(RemoveTestFilesystemObject(transactionPath, recordIsDirectory),
        QStringLiteral("simulate helper commit removing transaction record"));
    Check(!QFileInfo::exists(transactionPath) && !QFileInfo(transactionPath).isSymLink(),
        QStringLiteral("installer transaction record is gone after simulated helper commit"));
    if (invalidateWhileBlocked)
    {
        Check(!ConfigDatabase::IsAvailable(),
            QStringLiteral("record removal cannot bypass a full reopen and schema validation"));
        Check(!ConfigDatabase::IsAvailable(),
            QStringLiteral("invalid database remains rejected after the cached connection reopens"));
        bool invalidInitialized = true;
        Check(!ConfigDatabase::TryReadAuthenticationInitialized(&invalidInitialized),
            QStringLiteral("invalid reopened database exposes no authentication metadata"));
        Check(!ConfigDatabase::TryInitializeAuthenticationAccount(
                QStringLiteral("blocked_bootstrap"), blockedBootstrap),
            QStringLiteral("invalid reopened database cannot bootstrap a default account"));
        Check(FileSha256(databasePath) == invalidDatabaseHash,
            QStringLiteral("full reopen rejection leaves blocked-period database bytes unchanged"));
        QTextStream(stdout) << "PASS: installer transaction gate " << scenario
            << " closes cached connections and revalidates after record removal" << Qt::endl;
        return 0;
    }

    Check(ConfigDatabase::IsAvailable(),
        QStringLiteral("valid database access recovers only after transaction record removal"));
    if (databaseInitiallyAbsent)
    {
        Check(QFileInfo::exists(databasePath),
            QStringLiteral("fresh ConfigStore is created only after NOOP_ABSENT record removal"));
        bool freshInitialized = true;
        Check(ConfigDatabase::TryReadAuthenticationInitialized(&freshInitialized)
                && !freshInitialized,
            QStringLiteral("fresh post-transaction store remains explicitly uninitialized"));
    }
    else
    {
        const QStringList accounts = ConfigDatabase::ListScopedSettingIds(
            QStringLiteral("account"), QStringLiteral("Profile"));
        Check(accounts == QStringList{ QStringLiteral("admin") },
            QStringLiteral("recovered database contains only the authoritative account"));
        Check(FileSha256(databasePath) == databaseHashBefore,
            QStringLiteral("record removal and read-only recovery leave ConfigStore byte-identical"));
    }

    QTextStream(stdout) << "PASS: installer transaction gate " << scenario
        << " fails closed without mutation and recovers after record removal" << Qt::endl;
    return 0;
}

int RunLegacyTimestampMergeTest()
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("legacy timestamp merge temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))),
        QStringLiteral("legacy timestamp merge data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString adminHash = LegacyPasswordHash(
        QStringLiteral("admin"), QStringLiteral("Field-Timestamp-Merge-Test"));
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(
            QStringLiteral("QSQLITE"), QStringLiteral("legacy_timestamp_merge_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open legacy timestamp merge fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"),
            QStringLiteral("write timestamp merge schema 4"));

        InsertFlatLegacyAccount(
            db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
        InsertRaw(db, QStringLiteral("global"), QString(),
            QStringLiteral("Accounts/Users"), QStringLiteral("admin/CreatedAt"),
            QStringLiteral("2026-05-07T00:00:00"));
        InsertRaw(db, QStringLiteral("global"), QString(),
            QStringLiteral("Accounts/Users"), QStringLiteral("admin/UpdatedAt"),
            QStringLiteral("2026-05-15T00:00:00"));

        InsertNestedLegacyAccount(
            db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
        InsertRaw(db, QStringLiteral("global"), QString(),
            QStringLiteral("Accounts/Users/admin"), QStringLiteral("CreatedAt"),
            ProtectLegacyFixture(QStringLiteral("2026-05-28T00:00:00")),
            false, true, QStringLiteral("datetime"));
        InsertRaw(db, QStringLiteral("global"), QString(),
            QStringLiteral("Accounts/Users/admin"), QStringLiteral("UpdatedAt"),
            ProtectLegacyFixture(QStringLiteral("2026-05-20T00:00:00")),
            false, true, QStringLiteral("datetime"));

        InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("PasswordHash"), adminHash, true);
        InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("Role"), QStringLiteral("admin"));
        InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("MustChangePassword"), QStringLiteral("0"), true);
        InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("CreatedAt"),
            QStringLiteral("2026-06-05T00:00:00"));
        InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("UpdatedAt"),
            QStringLiteral("2026-06-01T00:00:00"),
            false, false, QStringLiteral("datetime"));
        InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("PasswordChangedAt"),
            QStringLiteral("2026-05-31T12:34:56Z"),
            true, false, QStringLiteral("datetime"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("legacy_timestamp_merge_fixture"));

    QString pathError;
    Check(AppPaths::Initialize(
        QStringList() << QStringLiteral("CredentialSecurityTests")
                      << QStringLiteral("--data-root") << temp.path(),
        &pathError), QStringLiteral("initialize timestamp merge root: %1").arg(pathError));
    Check(ConfigDatabase::IsAvailable(),
        QStringLiteral("three-way account timestamp conflict is safely mergeable"));
    QString createdAt;
    QString updatedAt;
    QString passwordChangedAt;
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("CreatedAt"), &createdAt)
            && ConfigDatabase::ReadScopedSetting(
            QStringLiteral("account"), QStringLiteral("admin"),
            QStringLiteral("Profile"), QStringLiteral("UpdatedAt"), &updatedAt)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("PasswordChangedAt"),
                &passwordChangedAt)
            && createdAt == QStringLiteral("2026-05-07T00:00:00")
            && updatedAt == QStringLiteral("2026-06-01T00:00:00")
            && passwordChangedAt == QStringLiteral("2026-05-31T12:34:56Z"),
        QStringLiteral("timestamps merge deterministically and encrypted source timestamps normalize"));
    QTextStream(stdout)
        << "PASS: three-way legacy/target account timestamps merge deterministically"
        << Qt::endl;
    return 0;
}

int RunCurrentAuthenticationIntegrityTest(const QString& scenario)
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("current authentication integrity temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))),
        QStringLiteral("current authentication integrity data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString connectionName = QStringLiteral("current_auth_integrity_") + scenario;
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open current authentication integrity fixture"));
        CreateSettingsTable(db, scenario == QStringLiteral("table-name-case"));
        InsertCurrentAuthenticationMetadata(db, true);
        InsertCurrentAccountProfile(
            db, QStringLiteral("admin"),
            scenario == QStringLiteral("no-admin")
                ? QStringLiteral("operator") : QStringLiteral("admin"));
        QSqlQuery query(db);
        const auto insertMeta = [&query](const QString& key, const QString& value)
            {
                query.prepare(QStringLiteral("INSERT INTO meta(key, value) VALUES(?, ?)"));
                query.addBindValue(key);
                query.addBindValue(value);
                Check(query.exec(), QStringLiteral("insert current integrity metadata %1").arg(key));
            };
        if (scenario == QStringLiteral("module-case-shadow"))
        {
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("profile"), QStringLiteral("Shadow"),
                QStringLiteral("must-not-survive"));
        }
        else if (scenario == QStringLiteral("legacy-orphan"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("Accounts/Users"), QStringLiteral("admin/Role"),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("bad-dpapi"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("OnlineServices"), QStringLiteral("FtpPassword"),
                QStringLiteral("dpapi:user:v1:not-valid-base64"), true, true);
        }
        else if (scenario == QStringLiteral("portable-dpapi-prefix"))
        {
            QString protectedRole;
            Check(CredentialSecurity::ProtectForCurrentUser(
                QStringLiteral("admin"),
                QStringLiteral("account\nadmin\nProfile\nRole"),
                &protectedRole), QStringLiteral("protect portable Role fixture"));
            query.prepare(QStringLiteral(
                "UPDATE settings SET value_text=?, encrypted=0 "
                "WHERE scope_type='account' AND scope_id='admin' "
                "AND module='Profile' AND key_name='Role'"));
            query.addBindValue(protectedRole);
            Check(query.exec() && query.numRowsAffected() == 1,
                QStringLiteral("write DPAPI-prefixed portable Role fixture"));
        }
        else if (scenario == QStringLiteral("portable-legacy-prefix"))
        {
            query.prepare(QStringLiteral(
                "UPDATE settings SET value_text=?, encrypted=0 "
                "WHERE scope_type='account' AND scope_id='admin' "
                "AND module='Profile' AND key_name='Role'"));
            query.addBindValue(ProtectLegacyFixture(QStringLiteral("admin")));
            Check(query.exec() && query.numRowsAffected() == 1,
                QStringLiteral("write enc:v1-prefixed portable Role fixture"));
        }
        else if (scenario == QStringLiteral("portable-created-at-encrypted"))
        {
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("CreatedAt"),
                ProtectLegacyFixture(QStringLiteral("2026-07-13T00:00:00Z")),
                false, true, QStringLiteral("datetime"));
        }
        else if (scenario == QStringLiteral("bad-password-changed-at"))
        {
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("PasswordChangedAt"),
                QStringLiteral("not-an-iso-timestamp"), true, false,
                QStringLiteral("datetime"));
        }
        else if (scenario == QStringLiteral("manifest-without-state"))
        {
            insertMeta(QStringLiteral("legacy_credential_scrub_manifest"), QStringLiteral("[]"));
        }
        else if (scenario == QStringLiteral("bad-complete-manifest"))
        {
            insertMeta(QStringLiteral("legacy_credential_scrub_state"), QStringLiteral("complete"));
            insertMeta(QStringLiteral("legacy_credential_scrub_manifest"), QStringLiteral("{}"));
        }
        else if (scenario == QStringLiteral("complete-scrub"))
        {
            insertMeta(QStringLiteral("legacy_credential_scrub_state"), QStringLiteral("complete"));
            insertMeta(QStringLiteral("legacy_credential_scrub_manifest"), QStringLiteral("[]"));
        }
        else if (scenario == QStringLiteral("legacy-disabled-login-preferences"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("RememberPassword"),
                QStringLiteral("0"), true, false, QStringLiteral("bool"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("AutoLogin"),
                QStringLiteral("0"), true, false, QStringLiteral("bool"));
        }
        else if (scenario == QStringLiteral("plaintext-enabled-login-preference"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("RememberPassword"),
                QStringLiteral("1"), true, false, QStringLiteral("bool"));
        }
        else if (scenario == QStringLiteral("plaintext-autologin-sensitive-zero"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("AutoLogin"),
                QStringLiteral("1"), false, false, QStringLiteral("bool"));
        }
        else if (scenario == QStringLiteral("released-login-preferences"))
        {
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("RememberPassword"),
                QStringLiteral("0"), true, false, QStringLiteral("string"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState"), QStringLiteral("AutoLogin"),
                QStringLiteral("1"), false, false, QStringLiteral("string"));
        }
        else if (scenario == QStringLiteral("protected-remembered-credential"))
        {
            QString protectedCredential;
            QString protectedSavedPassword;
            Check(CredentialSecurity::ProtectForCurrentUser(
                QStringLiteral("remembered-secret"),
                QStringLiteral("global\n\nLoginState/RememberedCredentials\nadmin"),
                &protectedCredential),
                QStringLiteral("protect remembered credential fixture"));
            Check(CredentialSecurity::ProtectForCurrentUser(
                QStringLiteral("saved-secret"),
                QStringLiteral("global\n\nLoginState/SavedPasswords\nadmin"),
                &protectedSavedPassword),
                QStringLiteral("protect saved password fixture"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/RememberedCredentials"),
                QStringLiteral("admin"), protectedCredential,
                true, true, QStringLiteral("string"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/SavedPasswords"),
                QStringLiteral("admin"), protectedSavedPassword,
                true, true, QStringLiteral("string"));
        }
        else if (scenario.startsWith(QStringLiteral("remembered-credential-invalid-")))
        {
            QString keyName = QStringLiteral("admin");
            QString plainText = QStringLiteral("remembered-secret");
            QString valueType = QStringLiteral("string");
            bool sensitive = true;
            bool encrypted = true;
            if (scenario.endsWith(QStringLiteral("unsafe-key")))
            {
                keyName = QStringLiteral("..");
            }
            else if (scenario.endsWith(QStringLiteral("empty")))
            {
                plainText.clear();
            }
            else if (scenario.endsWith(QStringLiteral("type")))
            {
                valueType = QStringLiteral("bool");
            }
            else if (scenario.endsWith(QStringLiteral("sensitive")))
            {
                sensitive = false;
            }
            else if (scenario.endsWith(QStringLiteral("encrypted")))
            {
                encrypted = false;
            }
            else
            {
                Check(false, QStringLiteral("unknown invalid remembered credential scenario"));
            }
            QString protectedCredential;
            Check(CredentialSecurity::ProtectForCurrentUser(
                plainText,
                QStringLiteral("global\n\nLoginState/RememberedCredentials\n") + keyName,
                &protectedCredential),
                QStringLiteral("protect invalid remembered credential fixture"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/RememberedCredentials"),
                keyName, protectedCredential, sensitive, encrypted, valueType);
        }
        else if (scenario == QStringLiteral("empty-created-at"))
        {
            insertMeta(QStringLiteral("created_at"), QStringLiteral("   "));
        }
        else if (scenario == QStringLiteral("legacy-table-case"))
        {
            Check(query.exec(QStringLiteral("CREATE TABLE INI_VALUES(dummy INTEGER)")),
                QStringLiteral("create case-variant legacy table"));
        }
        else if (scenario != QStringLiteral("no-admin")
            && scenario != QStringLiteral("valid")
            && scenario != QStringLiteral("table-name-case"))
        {
            Check(false, QStringLiteral("unknown current integrity scenario"));
        }
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(AppPaths::Initialize(
        QStringList() << QStringLiteral("CredentialSecurityTests")
                      << QStringLiteral("--data-root") << temp.path(),
        &pathError), QStringLiteral("initialize current integrity root: %1").arg(pathError));
    const bool shouldOpen = scenario == QStringLiteral("valid")
        || scenario == QStringLiteral("complete-scrub")
        || scenario == QStringLiteral("protected-remembered-credential")
        || scenario == QStringLiteral("table-name-case");
    Check(ConfigDatabase::IsAvailable() == shouldOpen,
        QStringLiteral("current integrity result for %1").arg(scenario));
    Check(ConfigDatabase::IsAvailable() == shouldOpen,
        QStringLiteral("current integrity result remains stable for %1").arg(scenario));
    std::atomic_bool threadResult{ !shouldOpen };
    std::thread threadProbe([&threadResult]()
        {
            threadResult = ConfigDatabase::IsAvailable();
        });
    threadProbe.join();
    Check(threadResult.load() == shouldOpen,
        QStringLiteral("current integrity result on fresh thread for %1").arg(scenario));
    Check(FileSha256(dbPath) == before,
        QStringLiteral("current integrity %1 leaves database byte-identical").arg(scenario));
    QTextStream(stdout) << "PASS: current schema " << scenario
        << (shouldOpen ? " validates" : " fails closed")
        << " without database writes"
        << Qt::endl;
    return 0;
}

int RunLegacyMigrationRollbackTest(const QString& scenario)
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("legacy rollback temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))),
        QStringLiteral("legacy rollback data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString connectionName = QStringLiteral("legacy_rollback_fixture_") + scenario;
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open legacy rollback fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"),
            QStringLiteral("write rollback schema 4"));
        const QString adminHash = LegacyPasswordHash(
            QStringLiteral("admin"), QStringLiteral("Original-Admin-Test"));
        if (scenario == QStringLiteral("source-conflict"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertNestedLegacyAccount(
                db, QStringLiteral("admin"),
                LegacyPasswordHash(QStringLiteral("admin"), QStringLiteral("Different-Admin-Test")),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("target-conflict"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("PasswordHash"),
                LegacyPasswordHash(QStringLiteral("admin"), QStringLiteral("Different-Target-Test")),
                true);
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("Role"), QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("target-protected-portable"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("PasswordHash"), adminHash, true);
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("Role"),
                ProtectLegacyFixture(QStringLiteral("admin")), false, true);
        }
        else if (scenario == QStringLiteral("target-malformed-password-changed-at"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("PasswordHash"),
                LegacyPasswordHash(QStringLiteral("admin"), QStringLiteral("admin")), true);
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("Role"), QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("account"), QStringLiteral("admin"),
                QStringLiteral("Profile"), QStringLiteral("PasswordChangedAt"),
                QStringLiteral("not-iso"), true, false, QStringLiteral("datetime"));
        }
        else if (scenario == QStringLiteral("no-admin"))
        {
            InsertFlatLegacyAccount(
                db, QStringLiteral("cyh"),
                LegacyPasswordHash(QStringLiteral("cyh"), QStringLiteral("Operator-Test")),
                QStringLiteral("operator"));
        }
        else if (scenario == QStringLiteral("user-case-conflict"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertNestedLegacyAccount(
                db, QStringLiteral("Admin"),
                LegacyPasswordHash(QStringLiteral("Admin"), QStringLiteral("Other-Admin-Test")),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("field-case-conflict"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("Accounts/Users/admin"), QStringLiteral("role"),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("login-module-case-conflict"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("loginstate/general"), QStringLiteral("UserName"),
                QStringLiteral("admin"));
        }
        else if (scenario == QStringLiteral("login-field-case-conflict"))
        {
            InsertFlatLegacyAccount(db, QStringLiteral("admin"), adminHash, QStringLiteral("admin"));
            InsertRaw(db, QStringLiteral("global"), QString(),
                QStringLiteral("LoginState/General"), QStringLiteral("username"),
                QStringLiteral("admin"));
        }
        else
        {
            Check(false, QStringLiteral("unknown legacy rollback scenario"));
        }
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);
    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(AppPaths::Initialize(
        QStringList() << QStringLiteral("CredentialSecurityTests")
                      << QStringLiteral("--data-root") << temp.path(),
        &pathError), QStringLiteral("initialize rollback root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("legacy %1 must fail closed").arg(scenario));
    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("legacy %1 remains unavailable").arg(scenario));
    Check(FileSha256(dbPath) == before,
        QStringLiteral("legacy %1 rolls back byte-identically").arg(scenario));
    QTextStream(stdout) << "PASS: legacy " << scenario
        << " fails closed with byte-identical rollback" << Qt::endl;
    return 0;
}

int RunRejectedRuntimeIniTest(const QString& scenario)
{
    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("runtime INI rejection temp root"));
    const QString dataDir = temp.filePath(QStringLiteral("Data"));
    Check(QDir().mkpath(dataDir), QStringLiteral("runtime INI rejection data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString connectionName = QStringLiteral("runtime_ini_fixture_") + scenario;
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open runtime INI rejection fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"),
            QStringLiteral("write runtime INI schema 4"));
        InsertFlatLegacyAccount(
            db, QStringLiteral("admin"),
            LegacyPasswordHash(QStringLiteral("admin"), QStringLiteral("Admin-Ini-Test")),
            QStringLiteral("admin"));
        db.close();
    }
    QSqlDatabase::removeDatabase(connectionName);

    QString iniDirectory = dataDir;
    QByteArray content;
    if (scenario == QStringLiteral("unknown-key"))
    {
        content = QByteArrayLiteral("ifupright=true\nUnexpectedRuntimeKey=1\n");
    }
    else if (scenario == QStringLiteral("section"))
    {
        content = QByteArrayLiteral("[PointCloud]\nifupright=true\n");
    }
    else if (scenario == QStringLiteral("nested"))
    {
        iniDirectory = QDir(dataDir).filePath(QStringLiteral("runtime"));
        Check(QDir().mkpath(iniDirectory), QStringLiteral("create nested runtime INI dir"));
        content = QByteArrayLiteral("ifupright=true\nPlate_thickness=5\n");
    }
    else if (scenario == QStringLiteral("credential-key"))
    {
        content = QByteArrayLiteral("ifupright=true\nPassword=plaintext-secret\n");
    }
    else if (scenario == QStringLiteral("symlink"))
    {
        const QString targetPath = temp.filePath(QStringLiteral("RuntimePointCloudTarget.ini"));
        QFile target(targetPath);
        content = QByteArrayLiteral("ifupright=true\nPlate_thickness=5\n");
        Check(target.open(QIODevice::WriteOnly), QStringLiteral("write runtime INI symlink target"));
        Check(target.write(content) == content.size(),
            QStringLiteral("write complete runtime INI symlink target"));
        target.close();
        const QString linkPath = QDir(iniDirectory).filePath(
            QStringLiteral("CorrugatedSheetPointCloudEctration.ini"));
        bool linked = CreateSymbolicLinkW(
            reinterpret_cast<LPCWSTR>(linkPath.utf16()),
            reinterpret_cast<LPCWSTR>(targetPath.utf16()),
            SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE) != FALSE;
        DWORD linkError = linked ? ERROR_SUCCESS : GetLastError();
        if (!linked && linkError == ERROR_INVALID_PARAMETER)
        {
            linked = CreateSymbolicLinkW(
                reinterpret_cast<LPCWSTR>(linkPath.utf16()),
                reinterpret_cast<LPCWSTR>(targetPath.utf16()),
                0) != FALSE;
            linkError = linked ? ERROR_SUCCESS : GetLastError();
        }
        if (!linked)
        {
            QTextStream(stdout) << "SKIP: Windows cannot create runtime INI symlink (error "
                << linkError << "); source-level isSymLink gate remains required" << Qt::endl;
            return 0;
        }
        Check(QFileInfo(linkPath).isSymLink(), QStringLiteral("runtime INI fixture is a symlink"));
        content.clear();
    }
    else
    {
        Check(false, QStringLiteral("unknown runtime INI scenario"));
    }
    if (scenario != QStringLiteral("symlink"))
    {
        QFile ini(QDir(iniDirectory).filePath(
            QStringLiteral("CorrugatedSheetPointCloudEctration.ini")));
        Check(ini.open(QIODevice::WriteOnly), QStringLiteral("write rejected runtime INI"));
        Check(ini.write(content) == content.size(), QStringLiteral("write complete rejected runtime INI"));
        ini.close();
    }

    const QString before = FileSha256(dbPath);
    QString pathError;
    Check(AppPaths::Initialize(
        QStringList() << QStringLiteral("CredentialSecurityTests")
                      << QStringLiteral("--data-root") << temp.path(),
        &pathError), QStringLiteral("initialize rejected runtime INI root: %1").arg(pathError));
    Check(!ConfigDatabase::IsAvailable(),
        QStringLiteral("runtime INI %1 must remain blocked").arg(scenario));
    Check(FileSha256(dbPath) == before,
        QStringLiteral("runtime INI %1 leaves schema4 byte-identical").arg(scenario));
    QTextStream(stdout) << "PASS: runtime INI " << scenario
        << " remains a blocking legacy input" << Qt::endl;
    return 0;
}
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (app.arguments().contains(QStringLiteral("--future-schema")))
    {
        return RunFutureSchemaTest();
    }
    const int pythonInteropIndex = app.arguments().indexOf(QStringLiteral("--python-interop-root"));
    if (pythonInteropIndex >= 0 && pythonInteropIndex + 1 < app.arguments().size())
    {
        return RunPythonInteropTest(app.arguments().at(pythonInteropIndex + 1));
    }
    if (app.arguments().contains(QStringLiteral("--legacy-timestamp-merge")))
    {
        return RunLegacyTimestampMergeTest();
    }
    const int currentIntegrityIndex = app.arguments().indexOf(
        QStringLiteral("--current-auth-integrity"));
    if (currentIntegrityIndex >= 0
        && currentIntegrityIndex + 1 < app.arguments().size())
    {
        return RunCurrentAuthenticationIntegrityTest(
            app.arguments().at(currentIntegrityIndex + 1));
    }
    const int legacyLoginSemanticIndex = app.arguments().indexOf(
        QStringLiteral("--legacy-login-semantic"));
    if (legacyLoginSemanticIndex >= 0
        && legacyLoginSemanticIndex + 1 < app.arguments().size())
    {
        return RunLegacyLoginSemanticUpgradeTest(
            app.arguments().at(legacyLoginSemanticIndex + 1));
    }
    const int installerTransactionIndex = app.arguments().indexOf(
        QStringLiteral("--installer-transaction-gate"));
    if (installerTransactionIndex >= 0
        && installerTransactionIndex + 1 < app.arguments().size())
    {
        return RunInstallerTransactionGateTest(
            app.arguments().at(installerTransactionIndex + 1));
    }
    const int legacyRollbackIndex = app.arguments().indexOf(QStringLiteral("--legacy-rollback"));
    if (legacyRollbackIndex >= 0 && legacyRollbackIndex + 1 < app.arguments().size())
    {
        return RunLegacyMigrationRollbackTest(app.arguments().at(legacyRollbackIndex + 1));
    }
    const int rejectedRuntimeIniIndex = app.arguments().indexOf(
        QStringLiteral("--rejected-runtime-ini"));
    if (rejectedRuntimeIniIndex >= 0
        && rejectedRuntimeIniIndex + 1 < app.arguments().size())
    {
        return RunRejectedRuntimeIniTest(app.arguments().at(rejectedRuntimeIniIndex + 1));
    }
    if (app.arguments().contains(QStringLiteral("--corrupt-dpapi")))
    {
        return RunCorruptDpapiMigrationTest();
    }
    if (app.arguments().contains(QStringLiteral("--auth-initialization")))
    {
        return RunAuthenticationInitializationTest();
    }
    if (app.arguments().contains(QStringLiteral("--empty-v4-recovery")))
    {
        return RunEmptyV4RecoveryTest();
    }
    if (app.arguments().contains(QStringLiteral("--invalid-auth-metadata")))
    {
        return RunInvalidAuthenticationMetadataTest();
    }
    const int authenticationSemanticGateIndex = app.arguments().indexOf(
        QStringLiteral("--auth-semantic-gate"));
    if (authenticationSemanticGateIndex >= 0
        && authenticationSemanticGateIndex + 1 < app.arguments().size())
    {
        return RunAuthenticationSemanticVersionGateTest(
            app.arguments().at(authenticationSemanticGateIndex + 1));
    }
    if (app.arguments().contains(QStringLiteral("--legacy-disk-gate")))
    {
        return RunLegacyDiskInputGateTest();
    }
    if (app.arguments().contains(QStringLiteral("--pending-scrub-gate")))
    {
        return RunCredentialScrubPendingGateTest();
    }
    if (app.arguments().contains(QStringLiteral("--plaintext-backup-gate")))
    {
        return RunPlaintextBackupGateTest();
    }
    const QString password = QStringLiteral("S3cure-测试-Password-🙂");
    QString error;
    const QString first = CredentialSecurity::CreatePasswordRecord(password, &error);
    const QString second = CredentialSecurity::CreatePasswordRecord(password, &error);
    Check(!first.isEmpty() && !second.isEmpty(), QStringLiteral("PBKDF2 record creation"));
    Check(CredentialSecurity::IsSupportedPasswordRecord(first), QStringLiteral("current KDF record format validation"));
    Check(first != second, QStringLiteral("per-record random salts"));
    Check(CredentialSecurity::IsCurrentPasswordRecord(first), QStringLiteral("current KDF format"));
    Check(
        CredentialSecurity::VerifyPasswordRecord(QStringLiteral("tester"), password, first)
            == CredentialSecurity::PasswordVerification::Valid,
        QStringLiteral("correct Unicode password"));
    Check(
        CredentialSecurity::VerifyPasswordRecord(QStringLiteral("tester"), password + QLatin1Char('x'), first)
            == CredentialSecurity::PasswordVerification::Invalid,
        QStringLiteral("wrong password rejection"));

    QStringList strongerParts = first.split(QLatin1Char(':'));
    Check(strongerParts.size() == 5, QStringLiteral("parse current PBKDF2 fixture"));
    const QByteArray strongerSalt = QByteArray::fromBase64(
        strongerParts.at(3).toLatin1(), QByteArray::Base64UrlEncoding);
    const QByteArray strongerKey = QPasswordDigestor::deriveKeyPbkdf2(
        QCryptographicHash::Sha256,
        password.toUtf8(),
        strongerSalt,
        700000,
        32);
    strongerParts[2] = QStringLiteral("700000");
    strongerParts[4] = QString::fromLatin1(
        strongerKey.toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals));
    Check(
        CredentialSecurity::VerifyPasswordRecord(
            QStringLiteral("tester"), password, strongerParts.join(QLatin1Char(':')))
            == CredentialSecurity::PasswordVerification::Valid,
        QStringLiteral("stronger future PBKDF2 record is not downgraded"));

    QString malformed = first;
    malformed.replace(QStringLiteral(":600000:"), QStringLiteral(":999999999:"));
    Check(
        CredentialSecurity::VerifyPasswordRecord(QStringLiteral("tester"), password, malformed)
            == CredentialSecurity::PasswordVerification::Invalid,
        QStringLiteral("malicious iteration count rejection"));
    malformed = first;
    malformed.truncate(malformed.size() - 3);
    Check(
        CredentialSecurity::VerifyPasswordRecord(QStringLiteral("tester"), password, malformed)
            == CredentialSecurity::PasswordVerification::Invalid,
        QStringLiteral("truncated record rejection"));

    const QString legacyFormatPassword = QStringLiteral("Legacy-Only-Test");
    const QString legacy = QString::fromLatin1(QCryptographicHash::hash(
        QStringLiteral("legacy_user\n%1").arg(legacyFormatPassword).toUtf8(),
        QCryptographicHash::Sha256).toHex());
    Check(
        CredentialSecurity::VerifyPasswordRecord(QStringLiteral("legacy_user"), legacyFormatPassword, legacy)
            == CredentialSecurity::PasswordVerification::ValidNeedsUpgrade,
        QStringLiteral("legacy record accepted only for upgrade"));
    Check(CredentialSecurity::IsSupportedPasswordRecord(legacy), QStringLiteral("legacy SHA record format validation"));
    Check(
        CredentialSecurity::VerifyPasswordRecord(QStringLiteral("legacy_user"), QStringLiteral("wrong"), legacy)
            == CredentialSecurity::PasswordVerification::Invalid,
        QStringLiteral("legacy wrong password rejection"));

    const QString purpose = QStringLiteral("global\n\nOnlineServices\nFtpPassword");
    QString protectedFirst;
    QString protectedSecond;
    Check(
        CredentialSecurity::ProtectForCurrentUser(password, purpose, &protectedFirst, &error),
        QStringLiteral("DPAPI protect: %1").arg(error));
    Check(
        CredentialSecurity::ProtectForCurrentUser(password, purpose, &protectedSecond, &error),
        QStringLiteral("second DPAPI protect: %1").arg(error));
    Check(protectedFirst != protectedSecond, QStringLiteral("DPAPI randomized ciphertext"));
    QString recovered;
    Check(
        CredentialSecurity::UnprotectForCurrentUser(protectedFirst, purpose, &recovered, &error)
            && recovered == password,
        QStringLiteral("DPAPI Unicode round trip: %1").arg(error));
    QString protectedEmpty;
    recovered = QStringLiteral("sentinel");
    Check(
        CredentialSecurity::ProtectForCurrentUser(
            QString(), purpose, &protectedEmpty, &error)
            && !protectedEmpty.isEmpty()
            && CredentialSecurity::UnprotectForCurrentUser(
                protectedEmpty, purpose, &recovered, &error)
            && recovered.isEmpty(),
        QStringLiteral("DPAPI empty payload round trip: %1").arg(error));
    recovered.clear();
    Check(
        !CredentialSecurity::UnprotectForCurrentUser(
            protectedFirst, purpose + QStringLiteral("-wrong"), &recovered, &error)
            && recovered.isEmpty(),
        QStringLiteral("DPAPI purpose binding"));
    QString tampered = protectedFirst;
    tampered[tampered.size() - 1] = tampered.endsWith(QLatin1Char('A')) ? QLatin1Char('B') : QLatin1Char('A');
    recovered.clear();
    Check(
        !CredentialSecurity::UnprotectForCurrentUser(tampered, purpose, &recovered, &error)
            && recovered.isEmpty(),
        QStringLiteral("DPAPI tamper rejection"));

    QTemporaryDir temp;
    Check(temp.isValid(), QStringLiteral("v4 migration temp root"));
    Check(QDir().mkpath(temp.filePath(QStringLiteral("Data"))), QStringLiteral("v4 data dir"));
    const QString dbPath = temp.filePath(QStringLiteral("Data/ConfigStore.db"));
    const QString legacyUser = QStringLiteral("cyh");
    const QString legacyPassword = QStringLiteral("Legacy-Account-Test-Only");
    const QString legacyHash = QString::fromLatin1(QCryptographicHash::hash(
        QStringLiteral("%1\n%2").arg(legacyUser, legacyPassword).toUtf8(),
        QCryptographicHash::Sha256).toHex());
    const QString adminPassword = QStringLiteral("Original-Admin-Test-Only");
    const QString adminHash = QString::fromLatin1(QCryptographicHash::hash(
        QStringLiteral("admin\n%1").arg(adminPassword).toUtf8(),
        QCryptographicHash::Sha256).toHex());
    const QString remoteSecret = QStringLiteral("");
    const QString authoritativeUser = QStringLiteral("current_admin");
    const QString authoritativePasswordRecord = CredentialSecurity::CreatePasswordRecord(
        QStringLiteral("Current-Authoritative-Password"), &error);
    Check(!authoritativePasswordRecord.isEmpty(), QStringLiteral("create authoritative account fixture"));
    {
        QFile runtimeIni(temp.filePath(
            QStringLiteral("Data/CorrugatedSheetPointCloudEctration.ini")));
        Check(runtimeIni.open(QIODevice::WriteOnly), QStringLiteral("create runtime point-cloud INI"));
        const QByteArray runtimeSettings = QByteArrayLiteral(
            "// runtime point-cloud parameters\n"
            "ifupright = true\n"
            "Plate_thickness = 5\n"
            "Save_File_Name = .\\AllType.txt\n"
            "above_z = 0.5\n");
        Check(runtimeIni.write(runtimeSettings) == runtimeSettings.size(),
            QStringLiteral("write runtime point-cloud INI"));
    }
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("v4_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open v4 fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"), QStringLiteral("write schema 4"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('encrypt_new_values', '0')"), QStringLiteral("write old encryption flag"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"), QStringLiteral("cyh/PasswordHash"), legacyHash, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"), QStringLiteral("cyh/Role"), QStringLiteral("operator"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"), QStringLiteral("cyh/CreatedAt"), QStringLiteral("2026-01-02T03:04:05"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"), QStringLiteral("admin/PasswordHash"), adminHash, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"), QStringLiteral("admin/Role"), QStringLiteral("admin"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users"), QStringLiteral("admin/CreatedAt"), QStringLiteral("2026-01-02T03:04:05"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/admin"),
            QStringLiteral("PasswordHash"), ProtectLegacyFixture(adminHash), true, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/admin"),
            QStringLiteral("Role"), ProtectLegacyFixture(QStringLiteral("admin")), false, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/admin"),
            QStringLiteral("CreatedAt"),
            ProtectLegacyFixture(QStringLiteral("2026-01-02T03:04:05")), false, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("UserName"), legacyUser);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("RememberPassword"), QStringLiteral("1"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("AutoLogin"), QStringLiteral("1"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("PasswordBase64"), QString::fromLatin1(legacyPassword.toUtf8().toBase64()));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("OnlineServices"), QStringLiteral("FtpPassword"), remoteSecret);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/current_admin"), QStringLiteral("PasswordHash"), authoritativePasswordRecord, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/current_admin"), QStringLiteral("Role"), QStringLiteral("admin"));
        InsertRaw(
            db, QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
            QStringLiteral("PasswordHash"), authoritativePasswordRecord, true, false);
        InsertRaw(
            db, QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
            QStringLiteral("Role"), QStringLiteral("admin"), false, false);
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("v4_fixture"));

    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize v4 data root: %1").arg(pathError));
    Check(ConfigDatabase::IsAvailable(), QStringLiteral("runtime v4 to v5 migration"));
    bool authenticationInitialized = false;
    Check(
        ConfigDatabase::TryReadAuthenticationInitialized(&authenticationInitialized)
            && authenticationInitialized,
        QStringLiteral("migrated account store requires recovery instead of default bootstrap"));
    Check(
        ConfigDatabase::ListScopedSettingIds(QStringLiteral("account"), QStringLiteral("Profile"))
            .contains(legacyUser)
            && ConfigDatabase::ListScopedSettingIds(
                QStringLiteral("account"), QStringLiteral("Profile"))
                .contains(QStringLiteral("admin")),
        QStringLiteral("flat cyh plus duplicate-layout admin semantic mapping"));
    QString migratedHash;
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("account"), legacyUser, QStringLiteral("Profile"),
            QStringLiteral("PasswordHash"), &migratedHash),
        QStringLiteral("read migrated account hash"));
    Check(
        CredentialSecurity::VerifyPasswordRecord(legacyUser, legacyPassword, migratedHash)
            == CredentialSecurity::PasswordVerification::ValidNeedsUpgrade,
        QStringLiteral("legacy account remains verifiable for login-time KDF upgrade"));
    QString migratedAdminHash;
    QString migratedAdminRole;
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("account"), QStringLiteral("admin"), QStringLiteral("Profile"),
            QStringLiteral("PasswordHash"), &migratedAdminHash)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("account"), QStringLiteral("admin"), QStringLiteral("Profile"),
                QStringLiteral("Role"), &migratedAdminRole)
            && migratedAdminRole == QStringLiteral("admin")
            && CredentialSecurity::VerifyPasswordRecord(
                QStringLiteral("admin"), adminPassword, migratedAdminHash)
                == CredentialSecurity::PasswordVerification::ValidNeedsUpgrade,
        QStringLiteral("duplicate-layout admin remains complete and verifiable"));
    QString value;
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
            QStringLiteral("RememberPassword"), &value)
            && value == QStringLiteral("0"),
        QStringLiteral("legacy remembered password disabled"));
    Check(
        !ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"), QString(), QStringLiteral("LoginState"),
            QStringLiteral("PasswordBase64"), &value),
        QStringLiteral("legacy Base64 password removed"));
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"), QString(), QStringLiteral("OnlineServices"),
            QStringLiteral("FtpPassword"), &value)
            && value.isEmpty(),
        QStringLiteral("empty remote credential survives DPAPI migration"));
    QString authoritativeHashAfter;
    QString authoritativeRoleAfter;
    QString authoritativeMustChangeAfter;
    Check(
        ConfigDatabase::ReadScopedSetting(
            QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
            QStringLiteral("PasswordHash"), &authoritativeHashAfter)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
                QStringLiteral("Role"), &authoritativeRoleAfter)
            && ConfigDatabase::ReadScopedSetting(
                QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
                QStringLiteral("MustChangePassword"), &authoritativeMustChangeAfter)
            && authoritativeHashAfter == authoritativePasswordRecord
            && authoritativeRoleAfter == QStringLiteral("admin")
            && authoritativeMustChangeAfter == QStringLiteral("1"),
        QStringLiteral("authoritative account keeps role, password, and forced-change state"));
    QString strictPasswordRecord;
    QString strictRole;
    QString strictFingerprint;
    bool strictMustChange = false;
    Check(
        ConfigDatabase::TryReadAccountSecurityState(
            authoritativeUser,
            &strictPasswordRecord,
            &strictRole,
            &strictMustChange,
            &strictFingerprint)
            && strictPasswordRecord == authoritativePasswordRecord
            && strictRole == QStringLiteral("admin")
            && strictMustChange
            && !strictFingerprint.isEmpty(),
        QStringLiteral("real v4 enc:v1 account profile is normalized for strict v5 login"));

    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("v5_verify"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open migrated v5 database"));
        QSqlQuery query(db);
        Check(query.exec("SELECT value FROM meta WHERE key='schema_version'") && query.next()
            && query.value(0).toString() == QStringLiteral("5"), QStringLiteral("schema version 5"));
        Check(query.exec(
            "SELECT key, value FROM meta WHERE key IN "
            "('schema_version', 'auth_semantic_version', 'auth_initialized') ORDER BY key")
            && query.next() && query.value(0).toString() == QStringLiteral("auth_initialized")
            && query.value(1).toString() == QStringLiteral("1")
            && query.next() && query.value(0).toString() == QStringLiteral("auth_semantic_version")
            && query.value(1).toString() == QStringLiteral("3")
            && query.next() && query.value(0).toString() == QStringLiteral("schema_version")
            && query.value(1).toString() == QStringLiteral("5")
            && !query.next(), QStringLiteral("authentication migration metadata"));
        Check(query.exec(
            "SELECT COUNT(*) FROM settings WHERE lower(module)='accounts/users' "
            "OR lower(module) GLOB 'accounts/users/*' "
            "OR lower(module)='loginstate/general' "
            "OR lower(module)='loginstate/settings' "
            "OR lower(module) GLOB 'loginstate/savedpasswords*' "
            "OR lower(module) GLOB 'loginstate/rememberedcredentials*' "
            "OR lower(key_name)='passwordbase64'")
            && query.next() && query.value(0).toInt() == 0,
            QStringLiteral("legacy authentication rows removed"));
        Check(query.exec(
            "SELECT value_text FROM settings WHERE scope_type='global' AND module='OnlineServices' "
            "AND key_name='FtpPassword'")
            && query.next()
            && query.value(0).toString().startsWith(QStringLiteral("dpapi:user:v1:")),
            QStringLiteral("remote credential stored as DPAPI"));
        Check(query.exec(
            "SELECT COUNT(*) FROM settings WHERE scope_type='account' AND scope_id='current_admin' "
            "AND module='Profile' AND key_name IN ('PasswordHash', 'Role', 'MustChangePassword') "
            "AND encrypted=0")
            && query.next() && query.value(0).toInt() == 3,
            QStringLiteral("v4 account security fields are canonical plaintext verifier metadata"));
        db.close();
    }
    QSqlDatabase::removeDatabase(QStringLiteral("v5_verify"));

    QTextStream(stdout) << "PASS: PBKDF2, DPAPI, and runtime ConfigStore v4-to-v5 migration" << Qt::endl;
    return 0;
}
