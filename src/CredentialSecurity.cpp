#include "CredentialSecurity.h"

#include <QByteArray>
#include <QCryptographicHash>
#include <QPasswordDigestor>
#include <QRandomGenerator>
#include <QStringList>

#include <algorithm>
#include <cstring>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <wincrypt.h>

#pragma comment(lib, "Crypt32.lib")

namespace
{
constexpr int kSaltBytes = 16;
constexpr int kDerivedKeyBytes = 32;
constexpr int kPbkdf2Iterations = 600000;
constexpr int kMinimumAcceptedIterations = 100000;
constexpr int kMaximumAcceptedIterations = 2000000;
constexpr char kPasswordPrefix[] = "pbkdf2-sha256:v1:";
constexpr char kDpapiPrefix[] = "dpapi:user:v1:";
constexpr char kDpapiEntropyPrefix[] =
    "NoTeaching-Robot|A5A7E2A0-8226-40BB-B126-94C5D298B3CF|credentials-v1|";

QByteArray ToBase64Url(const QByteArray& bytes)
{
    return bytes.toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals);
}

bool FromBase64Url(const QByteArray& encoded, QByteArray* bytes)
{
    if (bytes == nullptr || encoded.isEmpty())
    {
        return false;
    }
    const auto decoded = QByteArray::fromBase64Encoding(
        encoded,
        QByteArray::Base64UrlEncoding | QByteArray::AbortOnBase64DecodingErrors);
    if (!decoded)
    {
        return false;
    }
    *bytes = decoded.decoded;
    return true;
}

bool ConstantTimeEquals(const QByteArray& left, const QByteArray& right)
{
    if (left.size() != right.size())
    {
        return false;
    }
    unsigned char difference = 0;
    for (qsizetype index = 0; index < left.size(); ++index)
    {
        difference |= static_cast<unsigned char>(left.at(index))
            ^ static_cast<unsigned char>(right.at(index));
    }
    return difference == 0;
}

QByteArray RandomBytes(int size)
{
    QByteArray bytes(size, Qt::Uninitialized);
    int offset = 0;
    while (offset < size)
    {
        const quint32 value = QRandomGenerator::system()->generate();
        const int count = (std::min)(size - offset, static_cast<int>(sizeof(value)));
        memcpy(bytes.data() + offset, &value, static_cast<size_t>(count));
        offset += count;
    }
    return bytes;
}

QByteArray DerivePasswordKey(const QString& password, const QByteArray& salt, int iterations)
{
    return QPasswordDigestor::deriveKeyPbkdf2(
        QCryptographicHash::Sha256,
        password.toUtf8(),
        salt,
        iterations,
        kDerivedKeyBytes);
}

bool ParsePbkdf2Record(
    const QString& storedRecord,
    int* iterations,
    QByteArray* salt,
    QByteArray* expected)
{
    if (iterations == nullptr || salt == nullptr || expected == nullptr)
    {
        return false;
    }
    const QStringList parts = storedRecord.split(':');
    bool iterationsOk = false;
    if (parts.size() != 5
        || parts.at(0) != QStringLiteral("pbkdf2-sha256")
        || parts.at(1) != QStringLiteral("v1"))
    {
        return false;
    }
    const int parsedIterations = parts.at(2).toInt(&iterationsOk);
    QByteArray parsedSalt;
    QByteArray parsedExpected;
    if (!iterationsOk
        || parsedIterations < kMinimumAcceptedIterations
        || parsedIterations > kMaximumAcceptedIterations
        || parts.at(3).size() > 32
        || parts.at(4).size() > 64
        || !FromBase64Url(parts.at(3).toLatin1(), &parsedSalt)
        || !FromBase64Url(parts.at(4).toLatin1(), &parsedExpected)
        || parsedSalt.size() != kSaltBytes
        || parsedExpected.size() != kDerivedKeyBytes)
    {
        return false;
    }
    *iterations = parsedIterations;
    *salt = parsedSalt;
    *expected = parsedExpected;
    return true;
}

bool IsLegacySha256Record(const QString& storedRecord)
{
    if (storedRecord.size() != 64)
    {
        return false;
    }
    return std::all_of(storedRecord.cbegin(), storedRecord.cend(), [](const QChar& character)
        {
            const ushort value = character.unicode();
            return (value >= '0' && value <= '9')
                || (value >= 'a' && value <= 'f')
                || (value >= 'A' && value <= 'F');
        });
}

QString WindowsErrorText(DWORD code)
{
    wchar_t* buffer = nullptr;
    const DWORD count = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        code,
        0,
        reinterpret_cast<wchar_t*>(&buffer),
        0,
        nullptr);
    const QString text = count > 0 && buffer != nullptr
        ? QString::fromWCharArray(buffer, static_cast<int>(count)).trimmed()
        : QStringLiteral("Windows error %1").arg(code);
    if (buffer != nullptr)
    {
        LocalFree(buffer);
    }
    return text;
}

DATA_BLOB BlobFor(QByteArray& bytes)
{
    DATA_BLOB blob{};
    blob.cbData = static_cast<DWORD>(bytes.size());
    blob.pbData = bytes.isEmpty()
        ? nullptr
        : reinterpret_cast<BYTE*>(bytes.data());
    return blob;
}
}

QString CredentialSecurity::CreatePasswordRecord(const QString& password, QString* error)
{
    const QByteArray salt = RandomBytes(kSaltBytes);
    const QByteArray derived = DerivePasswordKey(password, salt, kPbkdf2Iterations);
    if (salt.size() != kSaltBytes || derived.size() != kDerivedKeyBytes)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法生成账号密码摘要。");
        }
        return QString();
    }
    if (error != nullptr)
    {
        error->clear();
    }
    return QStringLiteral("%1%2:%3:%4")
        .arg(QString::fromLatin1(kPasswordPrefix))
        .arg(kPbkdf2Iterations)
        .arg(QString::fromLatin1(ToBase64Url(salt)))
        .arg(QString::fromLatin1(ToBase64Url(derived)));
}

CredentialSecurity::PasswordVerification CredentialSecurity::VerifyPasswordRecord(
    const QString& userName,
    const QString& password,
    const QString& storedRecord)
{
    if (password.isEmpty() || storedRecord.isEmpty())
    {
        return PasswordVerification::Invalid;
    }

    if (storedRecord.startsWith(QString::fromLatin1(kPasswordPrefix)))
    {
        int iterations = 0;
        QByteArray salt;
        QByteArray expected;
        if (!ParsePbkdf2Record(storedRecord, &iterations, &salt, &expected))
        {
            return PasswordVerification::Invalid;
        }
        const QByteArray actual = DerivePasswordKey(password, salt, iterations);
        if (!ConstantTimeEquals(expected, actual))
        {
            return PasswordVerification::Invalid;
        }
        return iterations >= kPbkdf2Iterations
            ? PasswordVerification::Valid
            : PasswordVerification::ValidNeedsUpgrade;
    }

    // Legacy account record: SHA256(trimmed-user + "\n" + password).
    // It is accepted only to perform an immediate, mandatory upgrade.
    if (!IsLegacySha256Record(storedRecord))
    {
        return PasswordVerification::Invalid;
    }
    const QByteArray expected = QByteArray::fromHex(storedRecord.toLatin1());
    if (expected.size() != 32)
    {
        return PasswordVerification::Invalid;
    }
    const QByteArray actual = QCryptographicHash::hash(
        QStringLiteral("%1\n%2").arg(userName.trimmed(), password).toUtf8(),
        QCryptographicHash::Sha256);
    return ConstantTimeEquals(expected, actual)
        ? PasswordVerification::ValidNeedsUpgrade
        : PasswordVerification::Invalid;
}

bool CredentialSecurity::IsSupportedPasswordRecord(const QString& storedRecord)
{
    if (storedRecord.startsWith(QString::fromLatin1(kPasswordPrefix)))
    {
        int iterations = 0;
        QByteArray salt;
        QByteArray expected;
        return ParsePbkdf2Record(storedRecord, &iterations, &salt, &expected);
    }
    return IsLegacySha256Record(storedRecord);
}

bool CredentialSecurity::IsCurrentPasswordRecord(const QString& storedRecord)
{
    return storedRecord.startsWith(
        QStringLiteral("%1%2:").arg(QString::fromLatin1(kPasswordPrefix)).arg(kPbkdf2Iterations));
}

bool CredentialSecurity::ProtectForCurrentUser(
    const QString& plainText,
    const QString& purpose,
    QString* protectedText,
    QString* error)
{
    if (protectedText == nullptr || purpose.trimmed().isEmpty())
    {
        return false;
    }
    QByteArray inputBytes = plainText.toUtf8();
    QByteArray entropyBytes(kDpapiEntropyPrefix, static_cast<int>(strlen(kDpapiEntropyPrefix)));
    entropyBytes.append(purpose.toUtf8());
    DATA_BLOB input = BlobFor(inputBytes);
    DATA_BLOB entropy = BlobFor(entropyBytes);
    DATA_BLOB output{};
    if (!CryptProtectData(
            &input,
            L"NoTeaching-Robot local credential",
            &entropy,
            nullptr,
            nullptr,
            CRYPTPROTECT_UI_FORBIDDEN,
            &output))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("DPAPI 加密失败：%1").arg(WindowsErrorText(GetLastError()));
        }
        SecureZeroMemory(inputBytes.data(), static_cast<SIZE_T>(inputBytes.size()));
        return false;
    }

    const QByteArray protectedBytes(
        reinterpret_cast<const char*>(output.pbData),
        static_cast<int>(output.cbData));
    *protectedText = QString::fromLatin1(kDpapiPrefix) + QString::fromLatin1(ToBase64Url(protectedBytes));
    if (output.pbData != nullptr)
    {
        SecureZeroMemory(output.pbData, output.cbData);
        LocalFree(output.pbData);
    }
    SecureZeroMemory(inputBytes.data(), static_cast<SIZE_T>(inputBytes.size()));
    if (error != nullptr)
    {
        error->clear();
    }
    return true;
}

bool CredentialSecurity::UnprotectForCurrentUser(
    const QString& protectedText,
    const QString& purpose,
    QString* plainText,
    QString* error)
{
    if (plainText == nullptr || purpose.trimmed().isEmpty() || !IsCurrentUserProtected(protectedText))
    {
        return false;
    }
    QByteArray encryptedBytes;
    if (!FromBase64Url(protectedText.mid(static_cast<int>(strlen(kDpapiPrefix))).toLatin1(), &encryptedBytes))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("DPAPI 密文格式无效。");
        }
        return false;
    }
    QByteArray entropyBytes(kDpapiEntropyPrefix, static_cast<int>(strlen(kDpapiEntropyPrefix)));
    entropyBytes.append(purpose.toUtf8());
    DATA_BLOB input = BlobFor(encryptedBytes);
    DATA_BLOB entropy = BlobFor(entropyBytes);
    DATA_BLOB output{};
    if (!CryptUnprotectData(
            &input,
            nullptr,
            &entropy,
            nullptr,
            nullptr,
            CRYPTPROTECT_UI_FORBIDDEN,
            &output))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("DPAPI 解密失败：%1").arg(WindowsErrorText(GetLastError()));
        }
        return false;
    }

    const QByteArray plainBytes(
        reinterpret_cast<const char*>(output.pbData),
        static_cast<int>(output.cbData));
    const QString decoded = QString::fromUtf8(plainBytes.constData(), plainBytes.size());
    if (decoded.toUtf8() != plainBytes)
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("DPAPI 明文不是有效 UTF-8。");
        }
        if (output.pbData != nullptr)
        {
            SecureZeroMemory(output.pbData, output.cbData);
            LocalFree(output.pbData);
        }
        return false;
    }
    *plainText = decoded;
    if (output.pbData != nullptr)
    {
        SecureZeroMemory(output.pbData, output.cbData);
        LocalFree(output.pbData);
    }
    if (error != nullptr)
    {
        error->clear();
    }
    return true;
}

bool CredentialSecurity::IsCurrentUserProtected(const QString& protectedText)
{
    return protectedText.startsWith(QString::fromLatin1(kDpapiPrefix));
}
