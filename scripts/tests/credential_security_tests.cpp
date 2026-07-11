#include "AppPaths.h"
#include "ConfigDatabase.h"
#include "CredentialSecurity.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QPasswordDigestor>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QTemporaryDir>
#include <QTextStream>

#include <atomic>
#include <thread>

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

void CreateSettingsTable(QSqlDatabase& db)
{
    QSqlQuery query(db);
    Check(query.exec("CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)"), QStringLiteral("create meta"));
    Check(query.exec(
        "CREATE TABLE settings("
        "scope_type TEXT NOT NULL, scope_id TEXT NOT NULL DEFAULT '', module TEXT NOT NULL, "
        "key_name TEXT NOT NULL, value_text TEXT NOT NULL, value_type TEXT NOT NULL DEFAULT 'string', "
        "sensitive INTEGER NOT NULL DEFAULT 0, encrypted INTEGER NOT NULL DEFAULT 0, "
        "updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, "
        "PRIMARY KEY(scope_type, scope_id, module, key_name))"),
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
    bool encrypted = false)
{
    QSqlQuery query(db);
    query.prepare(
        "INSERT INTO settings(scope_type, scope_id, module, key_name, value_text, sensitive, encrypted) "
        "VALUES(?, ?, ?, ?, ?, ?, ?)");
    query.addBindValue(scope);
    query.addBindValue(scopeId.isNull() ? QStringLiteral("") : scopeId);
    query.addBindValue(module);
    query.addBindValue(key);
    query.addBindValue(value);
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

    QMap<QString, QString> bootstrap;
    bootstrap.insert(QStringLiteral("PasswordHash"), QStringLiteral("synthetic-test-record"));
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

    QString currentRole;
    QString currentFingerprint;
    Check(
        ConfigDatabase::TryCompareAndSetAccountPassword(
            QStringLiteral("admin"),
            QStringLiteral("synthetic-test-record"),
            QStringLiteral("synthetic-current-admin-record"),
            false,
            false,
            &currentRole,
            &currentFingerprint)
            && currentRole == QStringLiteral("admin")
            && !currentFingerprint.isEmpty(),
        QStringLiteral("forced password change uses compare-and-set"));
    QString storedPassword;
    bool storedMustChange = true;
    QString storedFingerprint;
    Check(
        ConfigDatabase::TryReadAccountSecurityState(
            QStringLiteral("admin"), &storedPassword, &currentRole,
            &storedMustChange, &storedFingerprint)
            && storedPassword == QStringLiteral("synthetic-current-admin-record")
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
    secondAdmin.insert(QStringLiteral("PasswordHash"), QStringLiteral("synthetic-second-admin-record"));
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
    QString pathError;
    Check(
        AppPaths::Initialize(
            QStringList() << QStringLiteral("CredentialSecurityTests")
                          << QStringLiteral("--data-root") << temp.path(),
            &pathError),
        QStringLiteral("initialize empty v4 root: %1").arg(pathError));
    Check(ConfigDatabase::IsAvailable(), QStringLiteral("migrate empty v4 store"));
    bool initialized = false;
    Check(
        ConfigDatabase::TryReadAuthenticationInitialized(&initialized) && initialized,
        QStringLiteral("an existing empty database is marked initialized for recovery"));
    QMap<QString, QString> bootstrap;
    bootstrap.insert(QStringLiteral("PasswordHash"), QStringLiteral("synthetic-test-record"));
    bootstrap.insert(QStringLiteral("Role"), QStringLiteral("admin"));
    Check(
        !ConfigDatabase::TryInitializeAuthenticationAccount(
            QStringLiteral("admin"), bootstrap),
        QStringLiteral("empty migrated v4 store refuses known bootstrap recreation"));
    QTextStream(stdout) << "PASS: empty existing ConfigStore enters recovery semantics" << Qt::endl;
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
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', '2')"), QStringLiteral("write pending auth semantic"));
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
        Check(query.exec("INSERT INTO meta(key, value) VALUES('auth_semantic_version', '2')"), QStringLiteral("write backup auth semantic"));
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
    const QString legacyUser = QStringLiteral("legacy_operator");
    const QString legacyPassword = QStringLiteral("Legacy-Account-Test-Only");
    const QString legacyHash = QString::fromLatin1(QCryptographicHash::hash(
        QStringLiteral("%1\n%2").arg(legacyUser, legacyPassword).toUtf8(),
        QCryptographicHash::Sha256).toHex());
    const QString remoteSecret = QStringLiteral("Synthetic-Remote-Credential-Test-Only");
    const QString authoritativeUser = QStringLiteral("current_admin");
    const QString authoritativePasswordRecord = CredentialSecurity::CreatePasswordRecord(
        QStringLiteral("Current-Authoritative-Password"), &error);
    Check(!authoritativePasswordRecord.isEmpty(), QStringLiteral("create authoritative account fixture"));
    const QString staleLegacyHash = QString::fromLatin1(QCryptographicHash::hash(
        QStringLiteral("%1\nStale-Legacy-Password").arg(authoritativeUser).toUtf8(),
        QCryptographicHash::Sha256).toHex());
    {
        QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), QStringLiteral("v4_fixture"));
        db.setDatabaseName(dbPath);
        Check(db.open(), QStringLiteral("open v4 fixture"));
        CreateSettingsTable(db);
        QSqlQuery query(db);
        Check(query.exec("INSERT INTO meta(key, value) VALUES('schema_version', '4')"), QStringLiteral("write schema 4"));
        Check(query.exec("INSERT INTO meta(key, value) VALUES('encrypt_new_values', '0')"), QStringLiteral("write old encryption flag"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/legacy_operator"), QStringLiteral("PasswordHash"), legacyHash, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/legacy_operator"), QStringLiteral("Role"), QStringLiteral("operator"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/legacy_operator"), QStringLiteral("CreatedAt"), QStringLiteral("2026-01-02T03:04:05"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("UserName"), legacyUser);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("RememberPassword"), QStringLiteral("1"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("AutoLogin"), QStringLiteral("1"));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("LoginState/General"), QStringLiteral("PasswordBase64"), QString::fromLatin1(legacyPassword.toUtf8().toBase64()));
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("OnlineServices"), QStringLiteral("FtpPassword"), remoteSecret);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/current_admin"), QStringLiteral("PasswordHash"), staleLegacyHash, true);
        InsertRaw(db, QStringLiteral("global"), QString(), QStringLiteral("Accounts/Users/current_admin"), QStringLiteral("Role"), QStringLiteral("operator"));
        InsertRaw(
            db, QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
            QStringLiteral("PasswordHash"), ProtectLegacyFixture(authoritativePasswordRecord), true, true);
        InsertRaw(
            db, QStringLiteral("account"), authoritativeUser, QStringLiteral("Profile"),
            QStringLiteral("Role"), ProtectLegacyFixture(QStringLiteral("admin")), false, true);
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
            .contains(legacyUser),
        QStringLiteral("legacy account semantic mapping"));
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
            && value == remoteSecret,
        QStringLiteral("remote credential survives DPAPI migration"));
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
            "SELECT COUNT(*) FROM settings WHERE module LIKE 'Accounts/Users/%' "
            "OR module LIKE 'LoginState/SavedPasswords%' OR lower(key_name)='passwordbase64'")
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
