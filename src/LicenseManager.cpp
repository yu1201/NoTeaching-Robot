#include "LicenseManager.h"
#include "LicenseBuildConfig.h"
#include "LicenseDialog.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <bcrypt.h>
#include <wincrypt.h>
#include <sddl.h>
#include <Aclapi.h>
#include <ShlObj.h>

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLockFile>
#include <QMutex>
#include <QMutexLocker>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QSaveFile>
#include <QSysInfo>
#include <QThread>
#include <QTimer>
#include <QUuid>
#include <algorithm>
#include <cmath>
#include <limits>

#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "crypt32.lib")
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "ole32.lib")

namespace {
constexpr qint64 MaximumLeaseSeconds = 7 * 24 * 60 * 60;
constexpr qint64 ClockToleranceSeconds = 300;
constexpr qint64 MaximumFileBytes = 1024 * 1024;
const QByteArray StateEntropy("HKPathlynx-License-v1");

QString NewNonce()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces)
        + QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QString RegistrationSecret()
{
    QByteArray bytes(32, '\0');
    if (BCryptGenRandom(nullptr, reinterpret_cast<PUCHAR>(bytes.data()),
        static_cast<ULONG>(bytes.size()), BCRYPT_USE_SYSTEM_PREFERRED_RNG) < 0) return {};
    return QString::fromLatin1(bytes.toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals));
}

bool Fail(QString* error, const QString& message)
{
    if (error) *error = message;
    return false;
}

bool IsDigest(const QString& text)
{
    if (text.size() != 64) return false;
    for (const QChar ch : text)
        if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'))) return false;
    return true;
}

bool IsUsefulIdentifier(QString value)
{
    value = value.trimmed().toLower();
    if (value.size() < 3 || value.size() > 512) return false;
    const QStringList placeholders = {
        "none", "unknown", "default string", "to be filled by o.e.m.",
        "to be filled by oem", "system serial number", "base board serial number",
        "not specified", "not applicable", "invalid", "o.e.m.", "na", "n/a"
    };
    if (placeholders.contains(value)) return false;
    QString significant;
    for (const QChar ch : value) if (ch.isLetterOrNumber()) significant += ch;
    if (significant.isEmpty()) return false;
    bool zeros = true, fs = true;
    for (const QChar ch : significant) {
        zeros = zeros && ch == '0';
        fs = fs && ch == 'f';
    }
    return !zeros && !fs;
}

QString IdentifierDigest(const char* component, const QString& value)
{
    if (!IsUsefulIdentifier(value)) return {};
    const QByteArray material = QByteArray("HKLicense/v1/") + component + '/'
        + value.trimmed().toUpper().toUtf8();
    return QString::fromLatin1(QCryptographicHash::hash(material,
        QCryptographicHash::Sha256).toHex());
}

QString SystemIdentifier()
{
    HKEY key = nullptr;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"SOFTWARE\\Microsoft\\Cryptography",
        0, KEY_QUERY_VALUE | KEY_WOW64_64KEY, &key) != ERROR_SUCCESS) return {};
    wchar_t value[256] = {};
    DWORD type = 0, size = sizeof(value);
    const LONG result = RegQueryValueExW(key, L"MachineGuid", nullptr,
        &type, reinterpret_cast<BYTE*>(value), &size);
    RegCloseKey(key);
    if (result != ERROR_SUCCESS || type != REG_SZ || size < sizeof(wchar_t)
        || size >= sizeof(value)) return {};
    return QString::fromWCharArray(value);
}

QString BaseboardIdentifier()
{
    // RawSMBIOSData has an 8-byte header followed by bounded DMI structures.
    constexpr DWORD provider = ('R' << 24) | ('S' << 16) | ('M' << 8) | 'B';
    const UINT bytes = GetSystemFirmwareTable(provider, 0, nullptr, 0);
    if (bytes < 8 || bytes > 1024 * 1024) return {};
    QByteArray raw(static_cast<qsizetype>(bytes), '\0');
    if (GetSystemFirmwareTable(provider, 0, raw.data(), bytes) != bytes) return {};
    DWORD declared = 0;
    memcpy(&declared, raw.constData() + 4, sizeof(declared));
    const qsizetype end = std::min<qsizetype>(raw.size(), 8 + declared);
    qsizetype cursor = 8;
    while (cursor + 4 <= end) {
        const auto* bytesAt = reinterpret_cast<const unsigned char*>(raw.constData() + cursor);
        const unsigned type = bytesAt[0], length = bytesAt[1];
        if (length < 4 || cursor + length > end) break;
        const qsizetype stringsStart = cursor + length;
        qsizetype next = stringsStart;
        while (next + 1 < end && !(raw[next] == '\0' && raw[next + 1] == '\0')) ++next;
        if (next + 1 >= end) break;
        if (type == 2 && length >= 8) {
            const unsigned serialIndex = bytesAt[7];
            qsizetype str = stringsStart;
            unsigned index = 1;
            while (serialIndex > 0 && str < next) {
                qsizetype terminator = str;
                while (terminator <= next && raw[terminator] != '\0') ++terminator;
                if (index == serialIndex) {
                    const QString serial = QString::fromLatin1(raw.constData() + str,
                        terminator - str).trimmed();
                    if (IsUsefulIdentifier(serial)) return serial;
                    break;
                }
                ++index;
                str = terminator + 1;
            }
        }
        if (type == 127) break;
        cursor = next + 2;
    }
    return {};
}

QString DiskIdentifier()
{
    wchar_t windowsPath[MAX_PATH] = {};
    if (!GetWindowsDirectoryW(windowsPath, MAX_PATH)) return {};
    wchar_t volumePath[MAX_PATH] = {};
    if (!GetVolumePathNameW(windowsPath, volumePath, MAX_PATH)) return {};
    DWORD serial = 0;
    if (!GetVolumeInformationW(volumePath, nullptr, 0, &serial, nullptr,
        nullptr, nullptr, 0) || !serial) return {};
    return QString::number(serial, 16).rightJustified(8, '0');
}

QJsonObject MachineFingerprint()
{
    return {
        {"system", IdentifierDigest("system", SystemIdentifier())},
        {"baseboard", IdentifierDigest("baseboard", BaseboardIdentifier())},
        {"disk", IdentifierDigest("disk", DiskIdentifier())}
    };
}

bool FingerprintMatches(const QJsonObject& expected, const QJsonObject& current)
{
    int matches = 0;
    for (const char* name : {"system", "baseboard", "disk"}) {
        const QString saved = expected.value(name).toString();
        const QString local = current.value(name).toString();
        if (!IsDigest(saved)) return false;
        if (IsDigest(local) && saved == local) ++matches;
    }
    return matches >= 2;
}

bool CompleteFingerprint(const QJsonObject& fingerprint)
{
    return IsDigest(fingerprint.value("system").toString())
        && IsDigest(fingerprint.value("baseboard").toString())
        && IsDigest(fingerprint.value("disk").toString());
}

QString DefaultStatePath()
{
    PWSTR programData = nullptr;
    if (FAILED(SHGetKnownFolderPath(FOLDERID_ProgramData, 0, nullptr, &programData)))
        return {};
    const QString root = QString::fromWCharArray(programData);
    CoTaskMemFree(programData);
    return QDir(root).filePath(QStringLiteral("HKPathlynx/Licensing/v1/license-state.bin"));
}

bool RestrictDirectory(const QString& path, QString* error)
{
    HANDLE token = nullptr;
    if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token))
        return Fail(error, QStringLiteral("无法读取授权存储目录的访问身份。"));
    DWORD size = 0;
    GetTokenInformation(token, TokenUser, nullptr, 0, &size);
    QByteArray buffer(size, '\0');
    const bool queried = size > 0 && GetTokenInformation(token, TokenUser,
        buffer.data(), size, &size);
    CloseHandle(token);
    if (!queried) return Fail(error, QStringLiteral("无法读取授权存储目录的访问身份。"));
    LPWSTR sidText = nullptr;
    if (!ConvertSidToStringSidW(reinterpret_cast<TOKEN_USER*>(buffer.data())->User.Sid, &sidText))
        return Fail(error, QStringLiteral("无法配置授权存储权限。"));
    const QString sddl = QStringLiteral("D:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FA;;;%1)")
        .arg(QString::fromWCharArray(sidText));
    LocalFree(sidText);
    PSECURITY_DESCRIPTOR descriptor = nullptr;
    if (!ConvertStringSecurityDescriptorToSecurityDescriptorW(
        reinterpret_cast<LPCWSTR>(sddl.utf16()), SDDL_REVISION_1, &descriptor, nullptr))
        return Fail(error, QStringLiteral("无法配置授权存储权限。"));
    PACL acl = nullptr;
    BOOL present = FALSE, defaulted = FALSE;
    const bool found = GetSecurityDescriptorDacl(descriptor, &present, &acl, &defaulted);
    const QString native = QDir::toNativeSeparators(path);
    const DWORD result = found && present ? SetNamedSecurityInfoW(
        const_cast<LPWSTR>(reinterpret_cast<LPCWSTR>(native.utf16())), SE_FILE_OBJECT,
        DACL_SECURITY_INFORMATION | PROTECTED_DACL_SECURITY_INFORMATION,
        nullptr, nullptr, acl, nullptr) : ERROR_INVALID_SECURITY_DESCR;
    LocalFree(descriptor);
    if (result != ERROR_SUCCESS)
        return Fail(error, QStringLiteral("授权目录权限不足，请使用安装授权的 Windows 账户运行。"));
    return true;
}

bool Dpapi(const QByteArray& input, QByteArray* output, bool protect, QString* error)
{
    DATA_BLOB in { static_cast<DWORD>(input.size()),
        reinterpret_cast<BYTE*>(const_cast<char*>(input.constData())) };
    DATA_BLOB entropy { static_cast<DWORD>(StateEntropy.size()),
        reinterpret_cast<BYTE*>(const_cast<char*>(StateEntropy.constData())) };
    DATA_BLOB out {};
    const BOOL ok = protect
        ? CryptProtectData(&in, L"HKPathlynx machine license", &entropy, nullptr,
            nullptr, CRYPTPROTECT_LOCAL_MACHINE | CRYPTPROTECT_UI_FORBIDDEN, &out)
        : CryptUnprotectData(&in, nullptr, &entropy, nullptr, nullptr,
            CRYPTPROTECT_UI_FORBIDDEN, &out);
    if (!ok) return Fail(error, protect ? QStringLiteral("无法加密本机授权缓存。")
        : QStringLiteral("本机授权缓存无法解密，可能来自其他电脑或已经损坏。"));
    *output = QByteArray(reinterpret_cast<const char*>(out.pbData), out.cbData);
    SecureZeroMemory(out.pbData, out.cbData);
    LocalFree(out.pbData);
    return true;
}

QByteArray DecodeUrl64(const QJsonValue& value)
{
    if (!value.isString()) return {};
    const QByteArray encoded = value.toString().toLatin1();
    if (encoded.isEmpty() || encoded.size() > MaximumFileBytes) return {};
    const QByteArray decoded = QByteArray::fromBase64(encoded,
        QByteArray::Base64UrlEncoding | QByteArray::AbortOnBase64DecodingErrors);
    if (decoded.toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals) != encoded)
        return {};
    return decoded;
}

bool Integer(const QJsonValue& value, qint64* result)
{
    if (!value.isDouble()) return false;
    const double number = value.toDouble();
    if (!std::isfinite(number) || std::floor(number) != number
        || number < 0 || number > 9007199254740991.0) return false;
    *result = static_cast<qint64>(number);
    return true;
}

bool VerifyRsa(const QByteArray& blob, const QByteArray& payload, const QByteArray& signature)
{
    if (blob.size() < static_cast<qsizetype>(sizeof(BCRYPT_RSAKEY_BLOB)) || signature.size() != 384)
        return false;
    BCRYPT_RSAKEY_BLOB header {};
    memcpy(&header, blob.constData(), sizeof(header));
    if (header.Magic != BCRYPT_RSAPUBLIC_MAGIC || header.BitLength != 3072
        || header.cbModulus != 384 || header.cbPublicExp < 1 || header.cbPublicExp > 8
        || header.cbPrime1 != 0 || header.cbPrime2 != 0
        || blob.size() != static_cast<qsizetype>(sizeof(header) + header.cbPublicExp + header.cbModulus))
        return false;
    BCRYPT_ALG_HANDLE algorithm = nullptr;
    BCRYPT_KEY_HANDLE key = nullptr;
    if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_RSA_ALGORITHM, nullptr, 0) < 0) return false;
    const NTSTATUS imported = BCryptImportKeyPair(algorithm, nullptr, BCRYPT_RSAPUBLIC_BLOB,
        &key, reinterpret_cast<PUCHAR>(const_cast<char*>(blob.constData())),
        static_cast<ULONG>(blob.size()), 0);
    bool valid = false;
    if (imported >= 0) {
        const QByteArray digest = QCryptographicHash::hash(payload, QCryptographicHash::Sha256);
        BCRYPT_PKCS1_PADDING_INFO padding { BCRYPT_SHA256_ALGORITHM };
        valid = BCryptVerifySignature(key, &padding,
            reinterpret_cast<PUCHAR>(const_cast<char*>(digest.constData())), digest.size(),
            reinterpret_cast<PUCHAR>(const_cast<char*>(signature.constData())), signature.size(),
            BCRYPT_PAD_PKCS1) >= 0;
        BCryptDestroyKey(key);
    }
    BCryptCloseAlgorithmProvider(algorithm, 0);
    return valid;
}
} // namespace

struct LicenseManager::Data
{
    mutable QMutex mutex;
    bool initialized = false;
    bool started = false;
    bool requestInFlight = false;
    bool readOnly = false;
    QString statePath;
    QString installationId;
    QString deviceToken;
    QString registrationSecret;
    QJsonObject fingerprint;
    QJsonObject envelope;
    QJsonObject policy;
    QString acceptedNonce;
    QString pendingOfflineNonce;
    qint64 highestRevision = 0;
    qint64 appliedRevision = 0;
    qint64 savedWallTime = 0;
    qint64 serverTime = 0;
    qint64 serverWallTime = 0;
    QElapsedTimer elapsed;
    qint64 sessionTrustedStart = 0;
    qint64 sessionWallStart = 0;
    QString runtimeState = QStringLiteral("idle");
    QString effectStatus = QStringLiteral("locked");
    QString storageError;
    QString syncMessage;
    QNetworkAccessManager* network = nullptr;
    QTimer* timer = nullptr;
#ifdef HK_LICENSE_TEST_BUILD
    qint64 testNow = 0;
    QByteArray testPublicBlob;
#endif

    qint64 WallNow() const
    {
#ifdef HK_LICENSE_TEST_BUILD
        if (testNow > 0) return testNow;
#endif
        return QDateTime::currentSecsSinceEpoch();
    }

    QByteArray PublicBlob() const
    {
#ifdef HK_LICENSE_TEST_BUILD
        if (!testPublicBlob.isEmpty()) return testPublicBlob;
#endif
        return QByteArray::fromBase64(QByteArray(HK_LICENSE_PUBLIC_KEY_B64),
            QByteArray::AbortOnBase64DecodingErrors);
    }

    qint64 TrustedNow() const
    {
        if (serverTime <= 0 || serverWallTime <= 0) return WallNow();
        const qint64 wallAdvanced = serverTime + std::max<qint64>(0, WallNow() - serverWallTime);
        const qint64 monotonic = sessionTrustedStart + (elapsed.isValid() ? elapsed.elapsed() / 1000 : 0);
        return std::max(wallAdvanced, monotonic);
    }

    QString Denial() const
    {
        if (!initialized) return QStringLiteral("授权尚未初始化。");
        if (!storageError.isEmpty()) return storageError;
        if (PublicBlob().isEmpty()) return QStringLiteral("此构建未配置授权验证公钥，请联系管理员。");
        if (policy.isEmpty()) return QStringLiteral("尚未激活，请输入激活码或申请试用。");
        const QString desired = policy.value("desiredState").toString();
        if (desired == "revoked") return QStringLiteral("此设备授权已撤销，请联系管理员。");
        if (desired == "suspended") return QStringLiteral("此设备已被管理员锁定。%1")
            .arg(policy.value("reason").toString());
        if (WallNow() + ClockToleranceSeconds < savedWallTime
            || (elapsed.isValid() && WallNow() + ClockToleranceSeconds
                < sessionWallStart + elapsed.elapsed() / 1000))
            return QStringLiteral("检测到系统时钟回退，请联网同步授权。");
        const qint64 now = TrustedNow();
        const QJsonValue expiry = policy.value("expiresAt");
        if (!expiry.isNull() && now >= expiry.toInteger()) return QStringLiteral("授权或试用期已到期。");
        if (now >= policy.value("leaseUntil").toInteger())
            return QStringLiteral("离线授权宽限期已结束，请联网同步授权。");
        if (!policy.value("entitlements").toArray().contains(QStringLiteral("all")))
            return QStringLiteral("当前授权未包含生产操作权限。");
        return {};
    }

    bool Save(QString* error, bool freshOnlineClock = false)
    {
        if (readOnly) return Fail(error, QStringLiteral("只读授权实例不能修改缓存。"));
        if (statePath.isEmpty()) return Fail(error, QStringLiteral("无法定位机器授权目录。"));
        const QString directory = QFileInfo(statePath).absolutePath();
        if (!QDir().mkpath(directory)) return Fail(error, QStringLiteral("无法创建机器授权目录。"));
        if (!RestrictDirectory(directory, error)) return false;
        QLockFile writeLock(statePath + QStringLiteral(".lock"));
        if (!writeLock.tryLock(1000))
            return Fail(error, QStringLiteral("授权缓存正在由另一进程更新，请稍后同步。"));
        QFile previous(statePath);
        if (previous.exists()) {
            if (!previous.open(QIODevice::ReadOnly) || previous.size() > MaximumFileBytes)
                return Fail(error, QStringLiteral("无法检查已有授权缓存版本。"));
            QByteArray priorPlain;
            if (!Dpapi(previous.readAll(), &priorPlain, false, error)) return false;
            previous.close(); // Windows must release the read handle before atomic replacement.
            const QJsonObject prior = QJsonDocument::fromJson(priorPlain).object();
            SecureZeroMemory(priorPlain.data(), priorPlain.size());
            if (prior.value("installationId").toString() != installationId
                || prior.value("highestVerifiedRevision").toInteger() > highestRevision
                || prior.value("serverTime").toInteger() > serverTime)
                return Fail(error, QStringLiteral("另一进程已保存较新的授权状态，请重新启动软件。"));
            appliedRevision = std::max(appliedRevision, prior.value("lastAppliedRevision").toInteger());
            if (!freshOnlineClock)
                savedWallTime = std::max(savedWallTime, prior.value("savedWallTime").toInteger());
        }
        savedWallTime = std::max(savedWallTime, WallNow());
        const QJsonObject state {
            {"schemaVersion", 1}, {"installationId", installationId},
            {"deviceToken", deviceToken}, {"policy", envelope},
            {"registrationSecret", registrationSecret},
            {"acceptedNonce", acceptedNonce}, {"pendingOfflineNonce", pendingOfflineNonce},
            {"highestVerifiedRevision", highestRevision}, {"lastAppliedRevision", appliedRevision},
            {"savedWallTime", savedWallTime}, {"serverTime", serverTime},
            {"serverWallTime", serverWallTime}
        };
        QByteArray plain = QJsonDocument(state).toJson(QJsonDocument::Compact), encrypted;
        const bool protectedOk = Dpapi(plain, &encrypted, true, error);
        SecureZeroMemory(plain.data(), plain.size());
        if (!protectedOk) return false;
        QSaveFile file(statePath);
        file.setDirectWriteFallback(false);
        if (!file.open(QIODevice::WriteOnly) || file.write(encrypted) != encrypted.size()
            || !file.commit()) return Fail(error, QStringLiteral("无法原子保存本机授权缓存。"));
        return true;
    }

    bool Verify(const QJsonObject& candidate, const QString& nonce,
        QJsonObject* decoded, QString* error) const
    {
        const QByteArray blob = PublicBlob();
        if (blob.isEmpty()) return Fail(error, QStringLiteral("此构建未配置授权验证公钥。"));
        const QString keyId = candidate.value("keyId").toString();
        const QString configuredKeyId = QString::fromUtf8(HK_LICENSE_KEY_ID);
        if (keyId.isEmpty() || keyId.size() > 128
            || (!configuredKeyId.isEmpty() && keyId != configuredKeyId))
            return Fail(error, QStringLiteral("授权签名密钥标识不匹配。"));
        const QByteArray payload = DecodeUrl64(candidate.value("payload"));
        const QByteArray signature = DecodeUrl64(candidate.value("signature"));
        if (payload.isEmpty() || !VerifyRsa(blob, payload, signature))
            return Fail(error, QStringLiteral("授权签名验证失败。"));
        QJsonParseError parse;
        const QJsonDocument document = QJsonDocument::fromJson(payload, &parse);
        if (parse.error != QJsonParseError::NoError || !document.isObject())
            return Fail(error, QStringLiteral("授权内容格式无效。"));
        const QJsonObject value = document.object();
        qint64 schema = 0, revision = 0, issued = 0, lease = 0, expiry = 0;
        if (!Integer(value.value("schemaVersion"), &schema) || schema != 1
            || !Integer(value.value("policyRevision"), &revision) || revision < 1
            || !Integer(value.value("issuedAt"), &issued) || issued < 1577836800
            || issued > 4102444800LL || !Integer(value.value("leaseUntil"), &lease)
            || lease == 0 || lease > issued + MaximumLeaseSeconds)
            return Fail(error, QStringLiteral("授权版本或有效期字段无效。"));
        if (value.value("installationId").toString() != installationId
            || value.value("deviceId").toString().isEmpty())
            return Fail(error, QStringLiteral("授权不属于当前安装实例。"));
        if (!FingerprintMatches(value.value("fingerprint").toObject(), fingerprint))
            return Fail(error, QStringLiteral("机器码匹配不足，请联系管理员换机或重新绑定。"));
        if (nonce.isEmpty() || value.value("requestNonce").toString() != nonce)
            return Fail(error, QStringLiteral("授权响应与本次请求不匹配。"));
        if (revision < highestRevision)
            return Fail(error, QStringLiteral("拒绝旧版本授权状态，请同步管理端最新设置。"));
        const QString desired = value.value("desiredState").toString();
        const QString kind = value.value("licenseKind").toString();
        if ((desired != "active" && desired != "suspended" && desired != "revoked")
            || (kind != "trial" && kind != "permanent" && kind != "term")
            || !value.value("entitlements").isArray()
            || !value.value("reason").isString())
            return Fail(error, QStringLiteral("授权状态字段无效。"));
        const QJsonValue expires = value.value("expiresAt");
        if ((kind == "permanent" && !expires.isNull())
            || (kind != "permanent" && (!Integer(expires, &expiry) || expiry == 0 || lease > expiry)))
            return Fail(error, QStringLiteral("授权到期时间无效。"));
        if (revision == highestRevision && !policy.isEmpty()) {
            for (const char* field : {"installationId", "deviceId", "fingerprint", "desiredState",
                "licenseKind", "expiresAt", "reason", "entitlements"})
                if (value.value(field) != policy.value(field))
                    return Fail(error, QStringLiteral("同一策略版本的授权内容发生冲突。"));
        }
        if (serverTime > 0 && issued + ClockToleranceSeconds < serverTime)
            return Fail(error, QStringLiteral("拒绝过期的授权响应时间。"));
        *decoded = value;
        return true;
    }

    bool Accept(const QJsonObject& candidate, const QString& nonce,
        const QString& token, QString* error, bool online = true)
    {
        QJsonObject decoded;
        if (!Verify(candidate, nonce, &decoded, error)) return false;
        if (!online && WallNow() + ClockToleranceSeconds < savedWallTime)
            return Fail(error, QStringLiteral("系统时钟已回退，请联网同步后再导入离线授权。"));
        const qint64 offlineNow = std::max(WallNow(), TrustedNow());
        if (!token.isEmpty()) {
            if (token.size() < 32 || token.size() > 4096 || token.contains('\r') || token.contains('\n'))
                return Fail(error, QStringLiteral("设备认证凭据格式无效。"));
            deviceToken = token;
            registrationSecret.clear();
        }
        envelope = candidate;
        policy = decoded;
        acceptedNonce = nonce;
        highestRevision = decoded.value("policyRevision").toInteger();
        serverTime = decoded.value("issuedAt").toInteger();
        // Only a fresh online nonce proves that issuedAt describes *now*.
        // Importing an old offline response must not restart its seven-day lease.
        const qint64 trustedNow = online ? serverTime : std::max(serverTime, offlineNow);
        serverWallTime = WallNow() - (trustedNow - serverTime);
        if (online) savedWallTime = WallNow();
        if (!online) pendingOfflineNonce.clear();
        sessionTrustedStart = trustedNow;
        sessionWallStart = WallNow();
        elapsed.restart();
        storageError.clear();
        if (!Save(error, online)) {
            storageError = error ? *error : QStringLiteral("授权缓存保存失败。");
            return false;
        }
        syncMessage = QStringLiteral("已验证管理端策略；等待实际应用状态回执。");
        return true;
    }

    bool Load(QString* error)
    {
        QFile file(statePath);
        if (!file.exists()) return true;
        if (!file.open(QIODevice::ReadOnly) || file.size() <= 0 || file.size() > MaximumFileBytes)
            return Fail(error, QStringLiteral("本机授权缓存无法读取或格式无效。"));
        QByteArray plain;
        if (!Dpapi(file.readAll(), &plain, false, error)) return false;
        QJsonParseError parse;
        const QJsonDocument doc = QJsonDocument::fromJson(plain, &parse);
        SecureZeroMemory(plain.data(), plain.size());
        if (parse.error != QJsonParseError::NoError || !doc.isObject())
            return Fail(error, QStringLiteral("本机授权缓存内容无效。"));
        const QJsonObject state = doc.object();
        qint64 schema = 0;
        if (!Integer(state.value("schemaVersion"), &schema) || schema != 1
            || QUuid(state.value("installationId").toString()).isNull()
            || !Integer(state.value("highestVerifiedRevision"), &highestRevision)
            || !Integer(state.value("lastAppliedRevision"), &appliedRevision)
            || appliedRevision > highestRevision
            || !Integer(state.value("savedWallTime"), &savedWallTime)
            || !Integer(state.value("serverTime"), &serverTime)
            || !Integer(state.value("serverWallTime"), &serverWallTime))
            return Fail(error, QStringLiteral("本机授权缓存版本或状态无效。"));
        installationId = state.value("installationId").toString();
        deviceToken = state.value("deviceToken").toString();
        registrationSecret = state.value("registrationSecret").toString();
        if (deviceToken.size() > 4096 || deviceToken.contains('\r') || deviceToken.contains('\n'))
            return Fail(error, QStringLiteral("本机设备凭据无效。"));
        if (!registrationSecret.isEmpty() && DecodeUrl64(registrationSecret).size() != 32)
            return Fail(error, QStringLiteral("本机注册恢复凭据无效。"));
        acceptedNonce = state.value("acceptedNonce").toString();
        pendingOfflineNonce = state.value("pendingOfflineNonce").toString();
        envelope = state.value("policy").toObject();
        if (!envelope.isEmpty()) {
            QJsonObject decoded;
            if (!Verify(envelope, acceptedNonce, &decoded, error)) return false;
            if (decoded.value("policyRevision").toInteger() != highestRevision)
                return Fail(error, QStringLiteral("本机授权策略与已验证版本不一致。"));
            policy = decoded;
            if (serverTime != decoded.value("issuedAt").toInteger() || serverWallTime <= 0)
                return Fail(error, QStringLiteral("本机授权时间锚点无效。"));
            sessionTrustedStart = serverTime + std::max<qint64>(0, WallNow() - serverWallTime);
            sessionWallStart = WallNow();
            elapsed.restart();
        } else if (highestRevision != 0 || appliedRevision != 0) {
            return Fail(error, QStringLiteral("本机授权策略缺失。"));
        }
        return true;
    }
};

LicenseManager& LicenseManager::Instance()
{
    // Application lifetime: avoids QObject/network destruction after QApplication.
    static LicenseManager* manager = new LicenseManager;
    return *manager;
}

LicenseManager::LicenseManager() : d(std::make_unique<Data>()) {}
LicenseManager::~LicenseManager() = default;

LicenseManager::LicenseMode LicenseManager::Mode() const
{
    return static_cast<LicenseMode>(HK_LICENSE_MODE);
}

bool LicenseManager::IsEnforced() const { return Mode() == LicenseMode::Enforce; }

void LicenseManager::Initialize(bool readOnly)
{
    Q_ASSERT(QThread::currentThread() == thread());
    QMutexLocker lock(&d->mutex);
    if (d->initialized) return;
    d->initialized = true;
    d->readOnly = readOnly;
    if (Mode() == LicenseMode::Off) return;
    d->statePath = DefaultStatePath();
    d->fingerprint = MachineFingerprint();
    d->installationId = QUuid::createUuid().toString(QUuid::WithoutBraces);
    QString error;
    if (d->statePath.isEmpty() || !d->Load(&error)) {
        d->storageError = error.isEmpty() ? QStringLiteral("无法读取机器授权目录。") : error;
        return;
    }
    if (!readOnly && !QFileInfo::exists(d->statePath) && !d->Save(&error))
        d->storageError = error;
}

void LicenseManager::Start()
{
    Q_ASSERT(QThread::currentThread() == thread());
    Initialize();
    {
        QMutexLocker lock(&d->mutex);
        if (d->started || d->readOnly || Mode() == LicenseMode::Off) return;
        d->started = true;
        d->network = new QNetworkAccessManager(this);
        d->timer = new QTimer(this);
        d->timer->setInterval(60 * 1000);
        connect(d->timer, &QTimer::timeout, this, [this] {
            SyncNow();
            NotifyChanged(); // Expiry/clock rollback must update gates even while offline.
        });
        d->timer->start();
    }
    SyncNow();
}

bool LicenseManager::CanStartProtectedOperation(QString* reason) const
{
    QMutexLocker lock(&d->mutex);
    if (Mode() == LicenseMode::Off) {
        if (reason) reason->clear();
        return true;
    }
    const QString denial = d->Denial();
    if (reason) *reason = denial;
    return Mode() != LicenseMode::Enforce || denial.isEmpty();
}

bool LicenseManager::IsLockRequested() const
{
    QMutexLocker lock(&d->mutex);
    return IsEnforced() && !d->Denial().isEmpty();
}

bool LicenseManager::IsNetworkBusy() const
{
    QMutexLocker lock(&d->mutex);
    return d->requestInFlight;
}

QString LicenseManager::StatusText() const
{
    QMutexLocker lock(&d->mutex);
    if (Mode() == LicenseMode::Off)
        return QStringLiteral("此版本默认未启用授权管理。\n不注册设备、不上报在线状态，也不执行远程锁定。\n启用方式由发布构建确定。");
    QString text = Mode() == LicenseMode::Audit
        ? QStringLiteral("授权审计模式（记录状态，允许继续操作）\n")
        : QStringLiteral("授权强制模式\n");
    const QString denial = d->Denial();
    text += denial.isEmpty() ? QStringLiteral("授权有效\n") : denial + '\n';
    if (!d->policy.isEmpty()) {
        const QString kind = d->policy.value("licenseKind").toString();
        text += QStringLiteral("授权类型：%1\n").arg(kind == "trial" ? QStringLiteral("试用")
            : kind == "permanent" ? QStringLiteral("永久") : QStringLiteral("按期限授权"));
        const QJsonValue expiry = d->policy.value("expiresAt");
        if (!expiry.isNull()) text += QStringLiteral("到期时间：%1\n")
            .arg(QDateTime::fromSecsSinceEpoch(expiry.toInteger()).toString("yyyy-MM-dd HH:mm:ss"));
        text += QStringLiteral("离线有效至：%1\n策略版本：已接收 %2 / 已应用 %3\n")
            .arg(QDateTime::fromSecsSinceEpoch(d->policy.value("leaseUntil").toInteger())
                .toString("yyyy-MM-dd HH:mm:ss"))
            .arg(d->highestRevision).arg(d->appliedRevision);
    }
    if (!d->installationId.isEmpty()) text += QStringLiteral("安装标识：%1\n").arg(d->installationId);
    if (d->requestInFlight) text += QStringLiteral("正在联网同步…\n");
    if (!d->syncMessage.isEmpty()) text += d->syncMessage;
    return text.trimmed();
}

void LicenseManager::NotifyChanged()
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this] { NotifyChanged(); }, Qt::QueuedConnection);
        return;
    }
    if (onPolicyChanged) onPolicyChanged();
}

void LicenseManager::SetRuntimeState(const QString& state)
{
    if (state != "idle" && state != "robotBusy" && state != "safeRecovery") return;
    QMutexLocker lock(&d->mutex);
    if (d->runtimeState != state) d->runtimeState = state;
}

void LicenseManager::SetEffectStatus(const QString& status)
{
    if (status != "active" && status != "pendingSafeStop" && status != "locked") return;
    QMutexLocker lock(&d->mutex);
    if (d->effectStatus != status) d->effectStatus = status;
}

void LicenseManager::AcknowledgeAppliedPolicy()
{
    bool changed = false;
    {
        QMutexLocker lock(&d->mutex);
        if (Mode() == LicenseMode::Off || d->readOnly || d->policy.isEmpty()
            || d->highestRevision <= d->appliedRevision) return;
        const bool requiresLock = !d->Denial().isEmpty();
        // Receiving a suspension is not a successful stop acknowledgement.
        if (requiresLock && (d->effectStatus != "locked" || d->runtimeState != "idle")) return;
        if (!requiresLock && d->effectStatus != "active") return;
        const qint64 previous = d->appliedRevision;
        d->appliedRevision = d->highestRevision;
        QString error;
        if (!d->Save(&error)) {
            d->appliedRevision = previous;
            d->storageError = error;
        } else {
            d->syncMessage = QStringLiteral("管理设置已在本机应用；将在下一次心跳上报。 ");
        }
        changed = true;
    }
    if (changed) NotifyChanged();
}

void LicenseManager::Register(const QString& activationCode)
{
    Q_ASSERT(QThread::currentThread() == thread());
    Start();
    QJsonObject request;
    {
        QMutexLocker lock(&d->mutex);
        if (Mode() == LicenseMode::Off || d->readOnly) return;
        if (!CompleteFingerprint(d->fingerprint) && d->deviceToken.isEmpty()) {
            d->syncMessage = QStringLiteral("无法获取完整机器标识，请联系管理员检查系统、主板和系统盘信息。");
            lock.unlock();
            NotifyChanged();
            return;
        }
        request = {
            {"installationId", d->installationId}, {"fingerprint", d->fingerprint},
            {"deviceName", QSysInfo::machineHostName()}, {"channel", QString::fromUtf8(HK_LICENSE_CHANNEL)},
            {"appVersion", QCoreApplication::applicationVersion()},
            {"activationCode", activationCode.trimmed()}, {"nonce", NewNonce()}
        };
        if (d->deviceToken.isEmpty()) {
            if (d->registrationSecret.isEmpty()) d->registrationSecret = RegistrationSecret();
            if (d->registrationSecret.isEmpty()) {
                d->syncMessage = QStringLiteral("无法生成安全的设备注册凭据。");
                lock.unlock();
                NotifyChanged();
                return;
            }
            request.insert(QStringLiteral("registrationSecret"), d->registrationSecret);
        }
    }
    Post(QStringLiteral("/register"), request, true);
}

void LicenseManager::SyncNow()
{
    Q_ASSERT(QThread::currentThread() == thread());
    QJsonObject request;
    {
        QMutexLocker lock(&d->mutex);
        if (Mode() == LicenseMode::Off || d->readOnly || !d->started || d->deviceToken.isEmpty()) return;
        request = {
            {"installationId", d->installationId}, {"nonce", NewNonce()},
            {"lastAppliedRevision", d->appliedRevision}, {"highestVerifiedRevision", d->highestRevision},
            {"runtimeState", d->runtimeState}, {"effectStatus", d->effectStatus},
            {"appVersion", QCoreApplication::applicationVersion()}
        };
    }
    Post(QStringLiteral("/sync"), request, false);
}

void LicenseManager::Post(const QString& endpoint, QJsonObject body, bool registration)
{
    QString bearer;
    QNetworkAccessManager* network = nullptr;
    {
        QMutexLocker lock(&d->mutex);
        if (d->requestInFlight || !d->network || d->readOnly || Mode() == LicenseMode::Off) return;
        if (d->PublicBlob().isEmpty()) {
            d->syncMessage = QStringLiteral("缺少发布授权公钥，无法联网激活。");
            lock.unlock();
            NotifyChanged();
            return;
        }
        QString saveError;
        if (!d->Save(&saveError)) {
            d->storageError = saveError;
            lock.unlock();
            NotifyChanged();
            return;
        }
        bearer = d->deviceToken;
        network = d->network;
        d->requestInFlight = true;
        d->syncMessage.clear();
    }
    const QUrl url(QString::fromUtf8(HK_LICENSE_SERVER_URL) + endpoint);
    if (!url.isValid() || url.scheme() != "https" || url.host().isEmpty()
        || !url.userInfo().isEmpty() || url.hasQuery() || url.hasFragment()) {
        {
            QMutexLocker lock(&d->mutex);
            d->requestInFlight = false;
            d->syncMessage = QStringLiteral("授权服务器地址必须是有效的 HTTPS 地址。");
        }
        NotifyChanged();
        return;
    }
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    request.setRawHeader("Accept", "application/json");
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::ManualRedirectPolicy);
    request.setTransferTimeout(15000);
    if (!bearer.isEmpty()) request.setRawHeader("Authorization", "Bearer " + bearer.toUtf8());
    QNetworkReply* reply = network->post(request, QJsonDocument(body).toJson(QJsonDocument::Compact));
    QTimer::singleShot(15000, reply, [reply] {
        if (!reply->isFinished()) reply->abort();
    });
    // Qt's default certificate/hostname validation stays enabled. In particular,
    // never ignore sslErrors, downgrade to HTTP, or follow a credential redirect.
    auto bytes = std::make_shared<QByteArray>();
    auto oversized = std::make_shared<bool>(false);
    connect(reply, &QIODevice::readyRead, this, [reply, bytes, oversized] {
        const QByteArray chunk = reply->readAll();
        if (bytes->size() + chunk.size() > MaximumFileBytes) {
            *oversized = true;
            reply->abort();
        } else *bytes += chunk;
    });
    const QString nonce = body.value("nonce").toString();
    connect(reply, &QNetworkReply::finished, this, [this, reply, bytes, oversized, nonce, registration] {
        const QByteArray tail = reply->readAll();
        if (bytes->size() + tail.size() > MaximumFileBytes) *oversized = true;
        else *bytes += tail;
        const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        QString error;
        bool accepted = false;
        {
            QMutexLocker lock(&d->mutex);
            d->requestInFlight = false;
            if (*oversized) {
                error = QStringLiteral("授权服务响应超过大小限制。");
            } else if (reply->error() != QNetworkReply::NoError || status < 200 || status >= 300) {
                // A transport/HTTP error never creates an administrative lock.
                // An existing signed lease remains the source of offline access.
                error = status > 0
                    ? QStringLiteral("联网同步失败（HTTP %1），按已签名离线授权判断有效期。").arg(status)
                    : QStringLiteral("无法安全连接授权服务器，按已签名离线授权判断有效期。");
                const QJsonObject response = QJsonDocument::fromJson(*bytes).object();
                const QString code = response.value("error").toObject().value("code").toString();
                if (!code.isEmpty() && code.size() <= 80) error += QStringLiteral("\n服务代码：%1").arg(code);
            } else {
                QJsonParseError parse;
                const QJsonDocument doc = QJsonDocument::fromJson(*bytes, &parse);
                if (parse.error != QJsonParseError::NoError || !doc.isObject()) {
                    error = QStringLiteral("授权服务响应格式无效。");
                } else {
                    const QJsonObject response = doc.object();
                    const QString token = registration ? response.value("deviceToken").toString() : QString();
                    if (registration && d->deviceToken.isEmpty() && token.isEmpty())
                        error = QStringLiteral("激活响应缺少设备凭据。");
                    else accepted = d->Accept(response.value("policy").toObject(), nonce, token, &error);
                }
            }
            if (!accepted) d->syncMessage = error;
        }
        reply->deleteLater();
        NotifyChanged();
    });
    NotifyChanged();
}

bool LicenseManager::ExportOfflineRequest(const QString& path, QString* error)
{
    Initialize();
    QJsonObject request;
    {
        QMutexLocker lock(&d->mutex);
        if (Mode() == LicenseMode::Off || d->readOnly)
            return Fail(error, QStringLiteral("当前构建未启用授权，不能导出申请。"));
        if (!CompleteFingerprint(d->fingerprint))
            return Fail(error, QStringLiteral("缺少有效机器标识，无法导出离线申请。"));
        d->pendingOfflineNonce = NewNonce();
        if (!d->Save(error)) return false; // Persist nonce before exposing the request.
        request = {
            {"schemaVersion", 1}, {"installationId", d->installationId},
            {"fingerprint", d->fingerprint}, {"deviceName", QSysInfo::machineHostName()},
            {"channel", QString::fromUtf8(HK_LICENSE_CHANNEL)},
            {"appVersion", QCoreApplication::applicationVersion()}, {"nonce", d->pendingOfflineNonce}
        };
    }
    QSaveFile file(path);
    const QByteArray data = QJsonDocument(request).toJson(QJsonDocument::Indented);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size() || !file.commit())
        return Fail(error, QStringLiteral("无法写入离线申请文件。"));
    return true;
}

bool LicenseManager::ImportOfflineResponse(const QString& path, QString* error)
{
    Initialize();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() <= 0 || file.size() > MaximumFileBytes)
        return Fail(error, QStringLiteral("无法读取离线授权文件或文件过大。"));
    QJsonParseError parse;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parse);
    if (parse.error != QJsonParseError::NoError || !doc.isObject())
        return Fail(error, QStringLiteral("离线授权文件格式无效。"));
    bool accepted = false;
    {
        QMutexLocker lock(&d->mutex);
        if (Mode() == LicenseMode::Off || d->readOnly)
            return Fail(error, QStringLiteral("当前构建未启用授权，不能导入授权。"));
        if (d->pendingOfflineNonce.isEmpty())
            return Fail(error, QStringLiteral("请先在此电脑导出离线申请，再导入对应授权文件。"));
        const QJsonObject response = doc.object();
        const QString token = response.value("deviceToken").toString();
        if (d->deviceToken.isEmpty() && token.isEmpty())
            return Fail(error, QStringLiteral("首次离线授权缺少设备凭据。"));
        accepted = d->Accept(response.value("policy").toObject(), d->pendingOfflineNonce, token, error, false);
    }
    NotifyChanged();
    return accepted;
}

void LicenseManager::ShowDialog(QWidget* parent, std::function<void()> safetyRecovery,
    std::function<void()> safetyStop, std::function<bool()> safetyBusy, bool acknowledgeBlockedGate)
{
    Initialize();
    Start();
    LicenseDialog dialog(parent, std::move(safetyRecovery), std::move(safetyStop),
        std::move(safetyBusy), acknowledgeBlockedGate);
    dialog.exec();
}

#ifdef HK_LICENSE_TEST_BUILD
void LicenseManager::ResetForTest(const QString& statePath, const QJsonObject& fingerprint,
    const QByteArray& publicBlob, const QString& installationId, qint64 now)
{
    Q_ASSERT(!d->started);
    d = std::make_unique<Data>();
    d->initialized = true;
    d->statePath = statePath;
    d->fingerprint = fingerprint;
    d->testPublicBlob = publicBlob;
    d->installationId = installationId;
    d->testNow = now;
}
void LicenseManager::SetNowForTest(qint64 now)
{
    QMutexLocker lock(&d->mutex);
    d->testNow = now;
}
bool LicenseManager::ApplyPolicyForTest(const QJsonObject& policy, const QString& nonce, QString* error, bool online)
{
    QMutexLocker lock(&d->mutex);
    return d->Accept(policy, nonce, QString(), error, online);
}
bool LicenseManager::ReloadForTest(QString* error)
{
    QMutexLocker lock(&d->mutex);
    d->policy = QJsonObject();
    d->envelope = QJsonObject();
    return d->Load(error);
}
qint64 LicenseManager::HighestRevisionForTest() const
{
    QMutexLocker lock(&d->mutex);
    return d->highestRevision;
}
qint64 LicenseManager::AppliedRevisionForTest() const
{
    QMutexLocker lock(&d->mutex);
    return d->appliedRevision;
}
#endif
