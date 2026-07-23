#include "AppPaths.h"
#include "TheoreticalRobotModelStore.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryDir>
#include <QTextStream>

#include <filesystem>
#include <system_error>

namespace
{
int Fail(const QString& message)
{
    QTextStream(stderr) << "FAIL: " << message << Qt::endl;
    return 1;
}

bool Require(bool condition, const QString& message)
{
    if (!condition) QTextStream(stderr) << "FAIL: " << message << Qt::endl;
    return condition;
}

QByteArray StepFixture(const QByteArray& identity)
{
    return QByteArray("ISO-10303-21;\n"
                      "HEADER;\n"
                      "FILE_DESCRIPTION(('theoretical robot store test'),'2;1');\n"
                      "FILE_NAME('")
        + identity
        + QByteArray("','','',(''),(''),'','');\n"
                     "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n"
                     "ENDSEC;\n"
                     "DATA;\n"
                     "/* ") + identity + QByteArray(" */\n"
                     "ENDSEC;\n"
                     "END-ISO-10303-21;\n");
}

bool WriteBytes(const QString& path, const QByteArray& bytes)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    const bool ok = file.write(bytes) == bytes.size();
    file.close();
    return ok;
}

QByteArray ReadBytes(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return QByteArray();
    return file.readAll();
}

QString Sha256(const QByteArray& bytes)
{
    return QString::fromLatin1(
        QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}

bool IsAsciiHashFileName(const QString& value)
{
    if (value.size() != 69 || !value.endsWith(QStringLiteral(".step"))) return false;
    for (int index = 0; index < 64; ++index)
    {
        const QChar ch = value.at(index);
        if (!((ch >= QLatin1Char('0') && ch <= QLatin1Char('9'))
              || (ch >= QLatin1Char('a') && ch <= QLatin1Char('f'))))
            return false;
    }
    return true;
}

bool CreateFileSymlink(
    const QString& target,
    const QString& link,
    QString& diagnostic)
{
    std::error_code error;
#ifdef _WIN32
    std::filesystem::create_symlink(
        std::filesystem::path(target.toStdWString()),
        std::filesystem::path(link.toStdWString()), error);
#else
    std::filesystem::create_symlink(
        std::filesystem::path(target.toUtf8().constData()),
        std::filesystem::path(link.toUtf8().constData()), error);
#endif
    diagnostic = error
        ? QString::fromLocal8Bit(error.message().c_str())
        : QString();
    return !error;
}

bool RemoveFileSystemEntry(const QString& path)
{
    std::error_code error;
#ifdef _WIN32
    const bool removed = std::filesystem::remove(
        std::filesystem::path(path.toStdWString()), error);
#else
    const bool removed = std::filesystem::remove(
        std::filesystem::path(path.toUtf8().constData()), error);
#endif
    return removed && !error;
}

int RunPreInitializationProbe()
{
    QList<TheoreticalRobotModelStore::Asset> assets{
        TheoreticalRobotModelStore::Asset{ QStringLiteral("stale") }
    };
    TheoreticalRobotModelStore::Asset active{
        QStringLiteral("stale")
    };
    bool hasActive = true;
    QString error;
    if (!Require(!AppPaths::IsInitialized(),
                 QStringLiteral("AppPaths unexpectedly initialized before probe"))
        || !Require(TheoreticalRobotModelStore::StoreDirectory().isEmpty()
                        && TheoreticalRobotModelStore::ManifestFilePath().isEmpty(),
                    QStringLiteral("store exposed an install-root fallback before AppPaths init"))
        || !Require(!TheoreticalRobotModelStore::ListAssets(assets, error)
                        && assets.isEmpty() && !error.isEmpty(),
                    QStringLiteral("pre-init ListAssets did not fail closed"))
        || !Require(!TheoreticalRobotModelStore::ReadActiveRecord(
                        active, hasActive, error)
                        && active.sha256.isEmpty() && !hasActive && !error.isEmpty(),
                    QStringLiteral("pre-init ReadActiveRecord did not fail closed"))
        || !Require(!TheoreticalRobotModelStore::CompareExchangeActive(
                        QString(), QString(), error)
                        && !error.isEmpty(),
                    QStringLiteral("pre-init CompareExchangeActive did not fail closed")))
        return 1;
    QTextStream(stdout) << "PASS: pre-initialization store access fails closed" << Qt::endl;
    return 0;
}

int RunDataJunctionProbe(QCoreApplication& application)
{
    QTemporaryDir dataRoot;
    QTemporaryDir outsideRoot;
    if (!dataRoot.isValid() || !outsideRoot.isValid())
        return Fail(QStringLiteral("cannot create junction probe roots"));

    QString error;
    if (!AppPaths::Initialize(
            QStringList{ application.applicationFilePath(),
                         QStringLiteral("--data-root"), dataRoot.path() },
            &error))
        return Fail(QStringLiteral("cannot initialize junction probe AppPaths: %1").arg(error));

    const QString dataPath = QDir(dataRoot.path()).filePath(QStringLiteral("Data"));
    if (!QDir(dataPath).removeRecursively())
        return Fail(QStringLiteral("cannot remove empty Data directory for junction probe"));

#ifdef _WIN32
    QProcess mklink;
    const QString arguments = QStringLiteral("/d /c mklink /J \"%1\" \"%2\"")
        .arg(QDir::toNativeSeparators(dataPath),
             QDir::toNativeSeparators(outsideRoot.path()));
    mklink.setProgram(QStringLiteral("cmd.exe"));
    mklink.setNativeArguments(arguments);
    mklink.start();
    if (!mklink.waitForFinished(10000) || mklink.exitStatus() != QProcess::NormalExit
        || mklink.exitCode() != 0 || !QFileInfo(dataPath).isJunction())
    {
        return Fail(QStringLiteral("cannot create NTFS junction probe: %1")
                        .arg(QString::fromLocal8Bit(mklink.readAllStandardError())));
    }
#else
    std::error_code linkError;
    std::filesystem::create_directory_symlink(
        std::filesystem::path(outsideRoot.path().toUtf8().constData()),
        std::filesystem::path(dataPath.toUtf8().constData()), linkError);
    if (linkError || !QFileInfo(dataPath).isSymLink())
        return Fail(QStringLiteral("cannot create directory symlink probe"));
#endif

    QList<TheoreticalRobotModelStore::Asset> assets;
    const bool rejected = !TheoreticalRobotModelStore::ListAssets(assets, error)
        && assets.isEmpty()
        && !QFileInfo(QDir(outsideRoot.path()).filePath(
                          QStringLiteral("RobotModels"))).exists();

    // Remove only the temporary link entry; never recurse through it.
    const bool removed = QDir(dataRoot.path()).rmdir(QStringLiteral("Data"));
    if (!Require(rejected, QStringLiteral("Data junction escaped the data-root boundary"))
        || !Require(removed, QStringLiteral("cannot remove temporary Data junction")))
        return 1;
    QTextStream(stdout) << "PASS: data-directory junction is rejected" << Qt::endl;
    return 0;
}

int RunRealStepProbe(QCoreApplication& application, const QString& sourcePath)
{
    QTemporaryDir dataRoot;
    if (!dataRoot.isValid())
        return Fail(QStringLiteral("cannot create real-STEP probe data root"));
    const QFileInfo source(sourcePath);
    if (!source.exists() || !source.isFile())
        return Fail(QStringLiteral("real-STEP probe source does not exist: %1").arg(sourcePath));

    QString error;
    if (!AppPaths::Initialize(
            QStringList{ application.applicationFilePath(),
                         QStringLiteral("--data-root"), dataRoot.path() },
            &error))
        return Fail(QStringLiteral("cannot initialize real-STEP probe AppPaths: %1").arg(error));

    QElapsedTimer timer;
    timer.start();
    TheoreticalRobotModelStore::Asset imported;
    if (!TheoreticalRobotModelStore::ImportStepFile(sourcePath, imported, error))
        return Fail(QStringLiteral("real STEP import failed: %1").arg(error));
    const qint64 importMs = timer.elapsed();
    QString resolvedPath;
    TheoreticalRobotModelStore::Asset resolved;
    if (!TheoreticalRobotModelStore::ResolveActive(resolvedPath, resolved, error)
        || resolved.sha256 != imported.sha256
        || resolved.sizeBytes != source.size()
        || QFileInfo(resolvedPath).size() != source.size())
    {
        return Fail(QStringLiteral("real STEP resolve/identity failed: %1").arg(error));
    }
    QTextStream(stdout) << "PASS: real STEP " << source.size() << " bytes imported in "
                        << importMs << " ms and resolved with full integrity check" << Qt::endl;
    return 0;
}
}

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    if (application.arguments().contains(QStringLiteral("--preinit-probe")))
        return RunPreInitializationProbe();
    if (application.arguments().contains(QStringLiteral("--junction-probe")))
        return RunDataJunctionProbe(application);
    const int realStepIndex = application.arguments().indexOf(QStringLiteral("--real-step-probe"));
    if (realStepIndex >= 0)
    {
        if (realStepIndex + 1 >= application.arguments().size())
            return Fail(QStringLiteral("--real-step-probe requires a STEP path"));
        return RunRealStepProbe(application, application.arguments().at(realStepIndex + 1));
    }
    QTemporaryDir sourceRoot;
    QTemporaryDir dataRoot;
    if (!sourceRoot.isValid() || !dataRoot.isValid())
        return Fail(QStringLiteral("cannot create temporary test roots"));

    QString error;
    if (!AppPaths::Initialize(
            QStringList{ application.applicationFilePath(),
                         QStringLiteral("--data-root"), dataRoot.path() },
            &error))
        return Fail(QStringLiteral("cannot initialize AppPaths: %1").arg(error));

    QList<TheoreticalRobotModelStore::Asset> assets;
    if (!Require(TheoreticalRobotModelStore::ListAssets(assets, error) && assets.isEmpty(),
                 QStringLiteral("new store did not start empty: %1").arg(error)))
        return 1;
    TheoreticalRobotModelStore::Asset manifestActive{
        QStringLiteral("must-be-cleared")
    };
    bool hasManifestActive = true;
    if (!Require(TheoreticalRobotModelStore::ReadActiveRecord(
                     manifestActive, hasManifestActive, error)
                     && !hasManifestActive && manifestActive.sha256.isEmpty(),
                 QStringLiteral("empty manifest did not report an explicit no-active state: %1")
                     .arg(error)))
        return 1;

    const QByteArray firstBytes = StepFixture("robot-A");
    const QString firstSource = sourceRoot.filePath(QStringLiteral("理论机器人 A.step"));
    if (!WriteBytes(firstSource, firstBytes)) return Fail(QStringLiteral("cannot write first fixture"));

    TheoreticalRobotModelStore::Asset first;
    if (!Require(TheoreticalRobotModelStore::ImportStepFile(firstSource, first, error),
                 QStringLiteral("first import failed: %1").arg(error)))
        return 1;
    if (!Require(first.sha256 == Sha256(firstBytes)
                     && first.storedFileName == first.sha256 + QStringLiteral(".step")
                     && IsAsciiHashFileName(first.storedFileName)
                     && first.originalDisplayName == QFileInfo(firstSource).fileName()
                     && first.sizeBytes == firstBytes.size(),
                 QStringLiteral("first asset metadata is inconsistent")))
        return 1;

    QString resolvedPath;
    TheoreticalRobotModelStore::Asset resolved;
    if (!Require(TheoreticalRobotModelStore::ResolveActive(resolvedPath, resolved, error),
                 QStringLiteral("cannot resolve first active asset: %1").arg(error))
        || !Require(resolved.sha256 == first.sha256
                        && QFileInfo(resolvedPath).fileName() == first.storedFileName
                        && ReadBytes(resolvedPath) == firstBytes,
                    QStringLiteral("resolved first asset content/path is wrong")))
        return 1;

    // 相同内容、不同扩展名和显示名应去重，并保留第一次导入的原始显示名。
    const QString duplicateSource = sourceRoot.filePath(QStringLiteral("duplicate.stp"));
    if (!WriteBytes(duplicateSource, firstBytes)) return Fail(QStringLiteral("cannot write duplicate"));
    TheoreticalRobotModelStore::Asset duplicate;
    if (!Require(TheoreticalRobotModelStore::ImportStepFile(
                     duplicateSource, duplicate, error),
                 QStringLiteral("same-content import failed: %1").arg(error))
        || !Require(duplicate.sha256 == first.sha256
                        && duplicate.originalDisplayName == first.originalDisplayName,
                    QStringLiteral("same-content import did not preserve original metadata"))
        || !Require(TheoreticalRobotModelStore::ListAssets(assets, error)
                        && assets.size() == 1,
                    QStringLiteral("same-content import created a duplicate list entry")))
        return 1;

    // 不同内容得到不同文件；第一个文件必须保持原样。
    const QByteArray secondBytes = StepFixture("robot-B");
    const QString secondSource = sourceRoot.filePath(QStringLiteral("robot-B.STEP"));
    if (!WriteBytes(secondSource, secondBytes)) return Fail(QStringLiteral("cannot write second"));
    TheoreticalRobotModelStore::Asset second;
    if (!Require(TheoreticalRobotModelStore::ImportStepFile(secondSource, second, error),
                 QStringLiteral("second import failed: %1").arg(error))
        || !Require(second.sha256 != first.sha256,
                    QStringLiteral("different content reused the first identity"))
        || !Require(TheoreticalRobotModelStore::ListAssets(assets, error)
                        && assets.size() == 2,
                    QStringLiteral("asset list did not retain both contents")))
        return 1;
    const QString firstStored = QDir(TheoreticalRobotModelStore::StoreDirectory())
        .filePath(first.storedFileName);
    const QString secondStored = QDir(TheoreticalRobotModelStore::StoreDirectory())
        .filePath(second.storedFileName);
    if (!Require(ReadBytes(firstStored) == firstBytes && ReadBytes(secondStored) == secondBytes,
                 QStringLiteral("import overwrote an existing different asset")))
        return 1;

    // 事务式候选导入不得提前替换当前资产；只有显式 SetActive 后才切换。
    if (!Require(TheoreticalRobotModelStore::SetActive(first.sha256, error),
                 QStringLiteral("cannot restore first asset before staged import: %1").arg(error)))
        return 1;
    const QByteArray stagedBytes = StepFixture("robot-staged");
    const QString stagedSource = sourceRoot.filePath(QStringLiteral("robot-staged.step"));
    TheoreticalRobotModelStore::Asset staged;
    if (!WriteBytes(stagedSource, stagedBytes)
        || !Require(TheoreticalRobotModelStore::ImportStepFile(
                        stagedSource, staged, error, false),
                    QStringLiteral("staged import failed: %1").arg(error))
        || !Require(TheoreticalRobotModelStore::ResolveActive(
                        resolvedPath, resolved, error)
                        && resolved.sha256 == first.sha256,
                    QStringLiteral("staged import replaced the previous active asset"))
        || !Require(TheoreticalRobotModelStore::SetActive(staged.sha256, error),
                    QStringLiteral("cannot activate validated staged asset: %1").arg(error))
        || !Require(TheoreticalRobotModelStore::ResolveActive(
                        resolvedPath, resolved, error)
                        && resolved.sha256 == staged.sha256,
                    QStringLiteral("explicit staged activation did not take effect"))
        || !Require(TheoreticalRobotModelStore::SetActive(second.sha256, error),
                    QStringLiteral("cannot restore second asset after staged test: %1").arg(error)))
        return 1;

    // active 的更新必须是跨进程锁内的 CAS：空值是合法状态，错误预期不能
    // 覆盖其它实例已经选择的资产，且首次导入失败时可条件恢复为空。
    if (!Require(TheoreticalRobotModelStore::CompareExchangeActive(
                     second.sha256, QString(), error),
                 QStringLiteral("cannot conditionally clear second active asset: %1")
                     .arg(error))
        || !Require(TheoreticalRobotModelStore::ReadActiveRecord(
                        manifestActive, hasManifestActive, error)
                        && !hasManifestActive && manifestActive.sha256.isEmpty(),
                    QStringLiteral("cleared active record did not report empty"))
        || !Require(TheoreticalRobotModelStore::CompareExchangeActive(
                        QString(), first.sha256, error),
                    QStringLiteral("none-to-A compare/exchange failed: %1").arg(error)))
        return 1;

    error.clear();
    if (!Require(!TheoreticalRobotModelStore::CompareExchangeActive(
                     second.sha256, staged.sha256, error)
                     && !error.isEmpty(),
                 QStringLiteral("wrong expected SHA was accepted by compare/exchange"))
        || !Require(TheoreticalRobotModelStore::ReadActiveRecord(
                        manifestActive, hasManifestActive, error)
                        && hasManifestActive && manifestActive.sha256 == first.sha256,
                    QStringLiteral("failed CAS changed the active asset"))
        || !Require(TheoreticalRobotModelStore::CompareExchangeActive(
                        first.sha256, QString(), error),
                    QStringLiteral("A-to-empty conditional rollback failed: %1").arg(error))
        || !Require(TheoreticalRobotModelStore::ReadActiveRecord(
                        manifestActive, hasManifestActive, error)
                        && !hasManifestActive && manifestActive.sha256.isEmpty(),
                    QStringLiteral("A-to-empty rollback did not leave no active record"))
        || !Require(TheoreticalRobotModelStore::CompareExchangeActive(
                        QString(), second.sha256, error),
                    QStringLiteral("cannot restore second after CAS tests: %1").arg(error)))
        return 1;

    // 已被不同内容占用的哈希文件名绝不能覆盖。
    const QByteArray collisionBytes = StepFixture("robot-collision");
    const QString collisionSource = sourceRoot.filePath(QStringLiteral("collision.step"));
    const QString collisionTarget = QDir(TheoreticalRobotModelStore::StoreDirectory())
        .filePath(Sha256(collisionBytes) + QStringLiteral(".step"));
    const QByteArray occupied("occupied by different content");
    if (!WriteBytes(collisionSource, collisionBytes) || !WriteBytes(collisionTarget, occupied))
        return Fail(QStringLiteral("cannot prepare occupied hash path"));
    TheoreticalRobotModelStore::Asset collision;
    if (!Require(!TheoreticalRobotModelStore::ImportStepFile(
                     collisionSource, collision, error)
                     && collision.sha256.isEmpty()
                     && ReadBytes(collisionTarget) == occupied,
                 QStringLiteral("different content at hash path was overwritten or accepted")))
        return 1;

    // 非 STEP 边界、错误扩展名、目录和超大文件均应被拒绝。
    const QString invalidEnvelope = sourceRoot.filePath(QStringLiteral("invalid.step"));
    const QString wrongExtension = sourceRoot.filePath(QStringLiteral("robot.txt"));
    const QString directoryPath = sourceRoot.filePath(QStringLiteral("directory.step"));
    if (!WriteBytes(invalidEnvelope, QByteArray("not a STEP\n"))
        || !WriteBytes(wrongExtension, firstBytes)
        || !QDir().mkpath(directoryPath))
        return Fail(QStringLiteral("cannot prepare invalid fixtures"));
    TheoreticalRobotModelStore::Asset rejected;
    if (!Require(!TheoreticalRobotModelStore::ImportStepFile(invalidEnvelope, rejected, error)
                     && !TheoreticalRobotModelStore::ImportStepFile(wrongExtension, rejected, error)
                     && !TheoreticalRobotModelStore::ImportStepFile(directoryPath, rejected, error),
                 QStringLiteral("invalid envelope/extension/directory was accepted")))
        return 1;

    const QString oversizedPath = sourceRoot.filePath(QStringLiteral("oversized.step"));
    QFile oversized(oversizedPath);
    if (!oversized.open(QIODevice::WriteOnly)
        || !oversized.resize(TheoreticalRobotModelStore::MaximumAssetBytes + 1))
        return Fail(QStringLiteral("cannot create sparse oversized fixture"));
    oversized.close();
    if (!Require(!TheoreticalRobotModelStore::ImportStepFile(oversizedPath, rejected, error),
                 QStringLiteral("oversized STEP was accepted")))
        return 1;

    // 在系统允许创建符号链接时，验证源链接不会被跟随。
    const QString linkPath = sourceRoot.filePath(QStringLiteral("linked.step"));
    QString symlinkDiagnostic;
    const bool symlinksAvailable = CreateFileSymlink(
        firstSource, linkPath, symlinkDiagnostic);
    if (symlinksAvailable)
    {
        if (!Require(QFileInfo(linkPath).isSymLink()
                         && !TheoreticalRobotModelStore::ImportStepFile(linkPath, rejected, error),
                     QStringLiteral("symbolic-link STEP source was accepted")))
            return 1;
    }
    else
    {
        QTextStream(stdout) << "SKIP: file-symlink probes unavailable: "
                            << symlinkDiagnostic << Qt::endl;
    }

    // 发布目标上的 dangling symlink 不能被 QFile copy fallback 跟随，更不能在库外落盘。
    if (symlinksAvailable)
    {
        const QByteArray danglingBytes = StepFixture("robot-dangling-target");
        const QString danglingSource = sourceRoot.filePath(QStringLiteral("dangling-target.step"));
        const QString outsideTarget = sourceRoot.filePath(QStringLiteral("outside-target.step"));
        const QString danglingStored = QDir(TheoreticalRobotModelStore::StoreDirectory())
            .filePath(Sha256(danglingBytes) + QStringLiteral(".step"));
        QString diagnostic;
        if (!WriteBytes(danglingSource, danglingBytes)
            || !CreateFileSymlink(outsideTarget, danglingStored, diagnostic))
            return Fail(QStringLiteral("cannot prepare dangling asset symlink: %1").arg(diagnostic));
        TheoreticalRobotModelStore::Asset danglingRejected;
        if (!Require(QFileInfo(danglingStored).isSymLink()
                         && !TheoreticalRobotModelStore::ImportStepFile(
                             danglingSource, danglingRejected, error)
                         && danglingRejected.sha256.isEmpty()
                         && !QFileInfo::exists(outsideTarget)
                         && QFileInfo(danglingStored).isSymLink(),
                     QStringLiteral("dangling asset symlink was followed or accepted")))
            return 1;
        if (!RemoveFileSystemEntry(danglingStored))
            return Fail(QStringLiteral("cannot remove dangling asset symlink"));
    }

    // SetActive 和 ResolveActive 都必须重新读取并验证资产。
    if (!Require(TheoreticalRobotModelStore::SetActive(first.sha256.toUpper(), error),
                 QStringLiteral("cannot select first asset: %1").arg(error)))
        return 1;
    QFile tamper(firstStored);
    if (!tamper.open(QIODevice::Append) || tamper.write("tamper") != 6)
        return Fail(QStringLiteral("cannot tamper active asset"));
    tamper.close();
    manifestActive = second;
    hasManifestActive = false;
    if (!Require(TheoreticalRobotModelStore::ReadActiveRecord(
                     manifestActive, hasManifestActive, error)
                     && hasManifestActive && manifestActive.sha256 == first.sha256,
                 QStringLiteral(
                     "manifest-only active read touched or rejected the tampered STEP: %1")
                     .arg(error)))
        return 1;
    resolvedPath = QStringLiteral("must-be-cleared");
    resolved = second;
    if (!Require(!TheoreticalRobotModelStore::ResolveActive(resolvedPath, resolved, error)
                     && resolvedPath.isEmpty() && resolved.sha256.isEmpty(),
                 QStringLiteral("tampered active asset did not fail closed")))
        return 1;
    if (!Require(TheoreticalRobotModelStore::SetActive(second.sha256, error),
                 QStringLiteral("cannot select intact second asset: %1").arg(error)))
        return 1;

    const QString manifestPath = TheoreticalRobotModelStore::ManifestFilePath();
    const QByteArray goodManifest = ReadBytes(manifestPath);
    if (goodManifest.isEmpty())
        return Fail(QStringLiteral("cannot snapshot valid manifest"));

    if (symlinksAvailable)
    {
        // dangling lock 和 manifest 链接必须在任何外部目标被创建前关闭失败。
        const QString lockPath = QDir(TheoreticalRobotModelStore::StoreDirectory())
            .filePath(QStringLiteral("store.lock"));
        const QString outsideLock = sourceRoot.filePath(QStringLiteral("outside-store.lock"));
        QString diagnostic;
        if (!CreateFileSymlink(outsideLock, lockPath, diagnostic))
            return Fail(QStringLiteral("cannot prepare dangling lock symlink: %1").arg(diagnostic));
        if (!Require(!TheoreticalRobotModelStore::SetActive(second.sha256, error)
                         && !TheoreticalRobotModelStore::CompareExchangeActive(
                             second.sha256, QString(), error)
                         && !QFileInfo::exists(outsideLock)
                         && QFileInfo(lockPath).isSymLink(),
                     QStringLiteral(
                         "dangling mutation-lock symlink was followed or accepted by Set/CAS")))
            return 1;
        if (!RemoveFileSystemEntry(lockPath))
            return Fail(QStringLiteral("cannot remove dangling lock symlink"));

        const QString outsideManifest = sourceRoot.filePath(
            QStringLiteral("outside-assets.json"));
        if (!QFile::remove(manifestPath)
            || !CreateFileSymlink(outsideManifest, manifestPath, diagnostic))
            return Fail(QStringLiteral("cannot prepare dangling manifest symlink: %1")
                            .arg(diagnostic));
        assets = { first };
        if (!Require(!TheoreticalRobotModelStore::ListAssets(assets, error)
                         && assets.isEmpty()
                         && !QFileInfo::exists(outsideManifest)
                         && QFileInfo(manifestPath).isSymLink(),
                     QStringLiteral("dangling manifest symlink was followed or accepted")))
            return 1;
        if (!RemoveFileSystemEntry(manifestPath) || !WriteBytes(manifestPath, goodManifest))
            return Fail(QStringLiteral("cannot restore manifest after symlink probe"));
    }

    // 达到 1024 条时，新内容必须在复制前被拒绝，且不得写出第 1025 条清单或孤儿资产。
    QJsonArray fullAssets;
    for (int index = 0; index < 1024; ++index)
    {
        const QString hash = QStringLiteral("%1").arg(index, 64, 16, QLatin1Char('0'));
        fullAssets.append(QJsonObject{
            { QStringLiteral("sha256"), hash },
            { QStringLiteral("storedFileName"), hash + QStringLiteral(".step") },
            { QStringLiteral("originalDisplayName"),
              QStringLiteral("capacity-%1.step").arg(index) },
            { QStringLiteral("sizeBytes"), 1 },
            { QStringLiteral("importedUtc"), QStringLiteral("2026-07-22T00:00:00.000Z") }
        });
    }
    const QByteArray fullManifest = QJsonDocument(QJsonObject{
        { QStringLiteral("schemaVersion"), 1 },
        { QStringLiteral("activeSha256"), QString() },
        { QStringLiteral("assets"), fullAssets }
    }).toJson(QJsonDocument::Compact);
    const QByteArray capacityBytes = StepFixture("robot-capacity-overflow");
    const QString capacitySource = sourceRoot.filePath(QStringLiteral("capacity.step"));
    const QString capacityTarget = QDir(TheoreticalRobotModelStore::StoreDirectory())
        .filePath(Sha256(capacityBytes) + QStringLiteral(".step"));
    if (!WriteBytes(manifestPath, fullManifest) || !WriteBytes(capacitySource, capacityBytes))
        return Fail(QStringLiteral("cannot prepare manifest-capacity probe"));
    TheoreticalRobotModelStore::Asset capacityRejected;
    if (!Require(!TheoreticalRobotModelStore::ImportStepFile(
                     capacitySource, capacityRejected, error)
                     && capacityRejected.sha256.isEmpty()
                     && !QFileInfo::exists(capacityTarget)
                     && ReadBytes(manifestPath) == fullManifest,
                 QStringLiteral("1024-entry manifest accepted a new asset or was modified")))
        return 1;
    if (!WriteBytes(manifestPath, goodManifest))
        return Fail(QStringLiteral("cannot restore manifest after capacity probe"));

    // 清单中的路径只能是严格的 SHA.step；任何越界解析都必须关闭失败。
    const QJsonObject maliciousAsset{
        { QStringLiteral("sha256"), second.sha256 },
        { QStringLiteral("storedFileName"), QStringLiteral("../escape.step") },
        { QStringLiteral("originalDisplayName"), second.originalDisplayName },
        { QStringLiteral("sizeBytes"), second.sizeBytes },
        { QStringLiteral("importedUtc"), second.importedUtc }
    };
    const QJsonDocument maliciousManifest(QJsonObject{
        { QStringLiteral("schemaVersion"), 1 },
        { QStringLiteral("activeSha256"), second.sha256 },
        { QStringLiteral("assets"), QJsonArray{ maliciousAsset } }
    });
    if (!WriteBytes(manifestPath,
                    maliciousManifest.toJson(QJsonDocument::Compact)))
        return Fail(QStringLiteral("cannot write traversal manifest fixture"));
    resolvedPath = QStringLiteral("must-be-cleared");
    resolved = second;
    if (!Require(!TheoreticalRobotModelStore::ResolveActive(resolvedPath, resolved, error)
                     && resolvedPath.isEmpty() && resolved.sha256.isEmpty(),
                 QStringLiteral("out-of-bounds manifest path did not fail closed")))
        return 1;
    assets = { first };
    if (!Require(!TheoreticalRobotModelStore::ListAssets(assets, error) && assets.isEmpty(),
                 QStringLiteral("invalid manifest leaked stale list entries")))
        return 1;

    QTextStream(stdout)
        << "PASS: theoretical robot STEP store import/dedup/atomic manifest/"
           "manifest-only/CAS/size-symlink-dangling-link/capacity/tamper fail-closed tests"
        << Qt::endl;
    return 0;
}
