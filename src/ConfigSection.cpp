#include "ConfigSection.h"

#include "ConfigDatabase.h"
#include "RobotLog.h"

#include <QByteArray>
#include <QStringList>

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace
{
QString NormalizePart(const QString& value)
{
    QString normalized = value.trimmed();
    normalized.replace('\\', '/');
    while (normalized.contains(QStringLiteral("//")))
    {
        normalized.replace(QStringLiteral("//"), QStringLiteral("/"));
    }
    while (normalized.startsWith('/'))
    {
        normalized.remove(0, 1);
    }
    while (normalized.endsWith('/'))
    {
        normalized.chop(1);
    }
    return normalized;
}

bool HasLegacyConfigFileSuffix(const QString& value)
{
    const QString lower = value.trimmed().toLower();
    return lower.endsWith(QStringLiteral(".ini"))
        || lower.endsWith(QStringLiteral(".txt"))
        || lower.endsWith(QStringLiteral(".cfg"))
        || lower.endsWith(QStringLiteral(".conf"));
}

bool IsDatabaseNativePart(
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

std::string ToStdString(const QString& value)
{
    const QByteArray bytes = value.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

QString ToQString(const std::string& value)
{
    return QString::fromUtf8(value.data(), static_cast<int>(value.size()));
}
}

ConfigLocation ConfigLocation::Global(const QString& moduleName)
{
    return { QStringLiteral("global"), QString(), NormalizePart(moduleName) };
}

ConfigLocation ConfigLocation::Robot(const QString& robotName, const QString& moduleName)
{
    return { QStringLiteral("robot"), robotName.trimmed(), NormalizePart(moduleName) };
}

ConfigLocation ConfigLocation::WorkpieceTemplate(const QString& templateName, const QString& moduleName)
{
    return { QStringLiteral("workpiece_template"), templateName.trimmed(), NormalizePart(moduleName) };
}

ConfigLocation ConfigLocation::Result(const QString& resultId, const QString& moduleName)
{
    return { QStringLiteral("result"), resultId.trimmed(), NormalizePart(moduleName) };
}

bool ConfigLocation::IsValid() const
{
    const QString normalizedScope = NormalizePart(scopeType).toLower();
    if (!IsDatabaseNativePart(normalizedScope, false)
        || !IsDatabaseNativePart(module, true))
    {
        return false;
    }
    if (normalizedScope == QStringLiteral("global"))
    {
        return scopeId.trimmed().isEmpty();
    }
    return IsDatabaseNativePart(scopeId, false, false, true);
}

ConfigSection::ConfigSection() = default;

ConfigSection::ConfigSection(const ConfigLocation& location)
{
    SetLocation(location);
}

bool ConfigSection::SetLocation(const ConfigLocation& location)
{
    if (!location.IsValid())
    {
        return false;
    }
    m_location = location;
    m_location.scopeType = NormalizePart(m_location.scopeType).toLower();
    m_location.scopeId = m_location.scopeId.trimmed();
    m_location.module = NormalizePart(m_location.module);
    return true;
}

bool ConfigSection::SetSectionName(const std::string& sectionName)
{
    const QString requested = ToQString(sectionName);
    if (!IsDatabaseNativePart(requested, true))
    {
        m_sectionName.clear();
        return false;
    }
    m_sectionName = NormalizePart(requested);
    return true;
}

QString ConfigSection::ModuleName() const
{
    return m_sectionName.isEmpty()
        ? m_location.module
        : m_location.module + QStringLiteral("/") + m_sectionName;
}

bool ConfigSection::HasData() const
{
    return m_location.IsValid()
        && !ConfigDatabase::ReadScopedSettings(
            m_location.scopeType, m_location.scopeId, ModuleName()).isEmpty();
}

bool ConfigSection::CheckExists(const std::string& key) const
{
    std::string ignored;
    return ReadRaw(key, &ignored, false) > 0;
}

int ConfigSection::ReadRaw(const std::string& key, std::string* value, bool check) const
{
    if (value == nullptr || !m_location.IsValid() || ModuleName().isEmpty())
    {
        return 0;
    }
    QString stored;
    const ConfigDatabase::ReadStatus status = ConfigDatabase::ReadScopedSettingStatus(
        m_location.scopeType,
        m_location.scopeId,
        ModuleName(),
        ToQString(key),
        &stored);
    if (status != ConfigDatabase::ReadStatus::Found)
    {
        CheckRead(key, 0, check);
        return 0;
    }
    *value = ToStdString(stored);
    const size_t length = value->size();
    if (length > static_cast<size_t>(INT_MAX))
    {
        CheckRead(key, -1, check);
        return -1;
    }
    return static_cast<int>(length == 0 ? 1 : length);
}

bool ConfigSection::WriteRaw(const std::string& key, const std::string& value)
{
    return m_location.IsValid()
        && !ModuleName().isEmpty()
        && ConfigDatabase::WriteScopedSetting(
            m_location.scopeType,
            m_location.scopeId,
            ModuleName(),
            ToQString(key),
            ToQString(value));
}

void ConfigSection::CheckRead(const std::string& key, int result, bool check) const
{
    if (!check || result > 0)
    {
        return;
    }
    RobotLog log("Log/RobotRunLog.txt", true);
    log.write(
        LogColor::WARNING,
        "Database configuration read failed: scope=%s id=%s module=%s key=%s",
        ToStdString(m_location.scopeType).c_str(),
        ToStdString(m_location.scopeId).c_str(),
        ToStdString(ModuleName()).c_str(),
        key.c_str());
}

int ConfigSection::ReadString(const std::string& key, char value[]) const
{
    if (value == nullptr)
    {
        return 0;
    }
    std::string stored;
    const int result = ReadRaw(key, &stored, true);
    if (result > 0)
    {
        strncpy_s(value, 255, stored.c_str(), _TRUNCATE);
    }
    return result;
}

int ConfigSection::ReadString(const std::string& key, std::string& value) const
{
    return ReadRaw(key, &value, true);
}

int ConfigSection::ReadString(bool check, const std::string& key, std::string& value) const
{
    return ReadRaw(key, &value, check);
}

int ConfigSection::ReadString(bool check, const std::string& key, std::string* value) const
{
    return value == nullptr ? 0 : ReadRaw(key, value, check);
}

int ConfigSection::ReadString(const std::string& key, bool* value, bool check) const
{
    std::string stored;
    const int result = ReadRaw(key, &stored, check);
    if (result > 0 && value != nullptr)
    {
        *value = std::atoi(stored.c_str()) == 1;
    }
    return result;
}

int ConfigSection::ReadString(bool check, const std::string& key, bool* value) const
{
    return ReadString(key, value, check);
}

#define DEFINE_NUMERIC_READER(TYPE, CONVERT) \
int ConfigSection::ReadString(const std::string& key, TYPE* value) const \
{ \
    std::string stored; \
    const int result = ReadRaw(key, &stored, true); \
    if (result > 0 && value != nullptr) { *value = static_cast<TYPE>(CONVERT); } \
    return result; \
}

DEFINE_NUMERIC_READER(double, std::atof(stored.c_str()))
DEFINE_NUMERIC_READER(float, std::atof(stored.c_str()))
DEFINE_NUMERIC_READER(int, std::atoi(stored.c_str()))
DEFINE_NUMERIC_READER(long, std::atol(stored.c_str()))
DEFINE_NUMERIC_READER(long long, std::strtoll(stored.c_str(), nullptr, 10))
DEFINE_NUMERIC_READER(unsigned long, std::strtoul(stored.c_str(), nullptr, 10))

#undef DEFINE_NUMERIC_READER

#define DEFINE_OPTIONAL_NUMERIC_READER(TYPE, CONVERT) \
int ConfigSection::ReadString(bool check, const std::string& key, TYPE* value) const \
{ \
    std::string stored; \
    const int result = ReadRaw(key, &stored, check); \
    if (result > 0 && value != nullptr) { *value = static_cast<TYPE>(CONVERT); } \
    return result; \
}

DEFINE_OPTIONAL_NUMERIC_READER(double, std::atof(stored.c_str()))
DEFINE_OPTIONAL_NUMERIC_READER(int, std::atoi(stored.c_str()))
DEFINE_OPTIONAL_NUMERIC_READER(long, std::atol(stored.c_str()))

#undef DEFINE_OPTIONAL_NUMERIC_READER

bool ConfigSection::WriteString(const std::string& key, const std::string& value)
{
    return WriteRaw(key, value);
}

bool ConfigSection::WriteString(const std::string& key, bool value)
{
    return WriteRaw(key, value ? "1" : "0");
}

bool ConfigSection::WriteString(const std::string& key, int value)
{
    return WriteRaw(key, std::to_string(value));
}

bool ConfigSection::WriteString(const std::string& key, long value)
{
    return WriteRaw(key, std::to_string(value));
}

bool ConfigSection::WriteString(const std::string& key, double value, unsigned int decimalDigits)
{
    char format[16] = {};
    char buffer[256] = {};
    sprintf_s(format, sizeof(format), "%%.%uf", decimalDigits);
    sprintf_s(buffer, sizeof(buffer), format, value);
    return WriteRaw(key, buffer);
}

int ConfigSection::ReadString(
    const std::string& key1, const std::string& key2, T_ANGLE_PULSE& pulse, T_ANGLE_PULSE fields) const
{
    int ok = 1;
#define READ_PULSE_FIELD(MEMBER, NAME) if (fields.MEMBER > 0) { ok = ok && ReadString(key1 + NAME + key2, &pulse.MEMBER) > 0; }
    READ_PULSE_FIELD(nSPulse, "S")
    READ_PULSE_FIELD(nLPulse, "L")
    READ_PULSE_FIELD(nUPulse, "U")
    READ_PULSE_FIELD(nRPulse, "R")
    READ_PULSE_FIELD(nBPulse, "B")
    READ_PULSE_FIELD(nTPulse, "T")
    READ_PULSE_FIELD(lBXPulse, "BX")
    READ_PULSE_FIELD(lBYPulse, "BY")
    READ_PULSE_FIELD(lBZPulse, "BZ")
#undef READ_PULSE_FIELD
    return ok;
}

int ConfigSection::ReadString(
    const std::string& key1, const std::string& key2, T_ROBOT_COORS& coors, T_ROBOT_COORS fields) const
{
    int ok = 1;
#define READ_COOR_FIELD(MEMBER, NAME) if (fields.MEMBER > 0) { ok = ok && ReadString(key1 + NAME + key2, &coors.MEMBER) > 0; }
    READ_COOR_FIELD(dX, "X")
    READ_COOR_FIELD(dY, "Y")
    READ_COOR_FIELD(dZ, "Z")
    READ_COOR_FIELD(dRX, "RX")
    READ_COOR_FIELD(dRY, "RY")
    READ_COOR_FIELD(dRZ, "RZ")
    READ_COOR_FIELD(dBX, "BX")
    READ_COOR_FIELD(dBY, "BY")
    READ_COOR_FIELD(dBZ, "BZ")
#undef READ_COOR_FIELD
    return ok;
}

bool ConfigSection::WriteString(
    const std::string& key1, const std::string& key2, T_ANGLE_PULSE pulse, T_ANGLE_PULSE fields)
{
    bool ok = true;
#define WRITE_PULSE_FIELD(MEMBER, NAME) if (fields.MEMBER > 0) { ok = ok && WriteString(key1 + NAME + key2, pulse.MEMBER); }
    WRITE_PULSE_FIELD(nSPulse, "S")
    WRITE_PULSE_FIELD(nLPulse, "L")
    WRITE_PULSE_FIELD(nUPulse, "U")
    WRITE_PULSE_FIELD(nRPulse, "R")
    WRITE_PULSE_FIELD(nBPulse, "B")
    WRITE_PULSE_FIELD(nTPulse, "T")
    WRITE_PULSE_FIELD(lBXPulse, "BX")
    WRITE_PULSE_FIELD(lBYPulse, "BY")
    WRITE_PULSE_FIELD(lBZPulse, "BZ")
#undef WRITE_PULSE_FIELD
    return ok;
}

bool ConfigSection::WriteString(
    const std::string& key1, const std::string& key2, T_ROBOT_COORS coors, T_ROBOT_COORS fields)
{
    bool ok = true;
#define WRITE_COOR_FIELD(MEMBER, NAME) if (fields.MEMBER > 0) { ok = ok && WriteString(key1 + NAME + key2, coors.MEMBER); }
    WRITE_COOR_FIELD(dX, "X")
    WRITE_COOR_FIELD(dY, "Y")
    WRITE_COOR_FIELD(dZ, "Z")
    WRITE_COOR_FIELD(dRX, "RX")
    WRITE_COOR_FIELD(dRY, "RY")
    WRITE_COOR_FIELD(dRZ, "RZ")
    WRITE_COOR_FIELD(dBX, "BX")
    WRITE_COOR_FIELD(dBY, "BY")
    WRITE_COOR_FIELD(dBZ, "BZ")
#undef WRITE_COOR_FIELD
    return ok;
}

int ConfigSection::ReadAddString(const std::string& key, bool* value, bool initialValue)
{
    const int result = ReadString(false, key, value);
    if (result <= 0 && value != nullptr) { *value = initialValue; WriteString(key, initialValue); }
    return result;
}

int ConfigSection::ReadAddString(const std::string& key, int* value, int initialValue)
{
    const int result = ReadString(false, key, value);
    if (result <= 0 && value != nullptr) { *value = initialValue; WriteString(key, initialValue); }
    return result;
}

int ConfigSection::ReadAddString(const std::string& key, int* value, long initialValue)
{
    return ReadAddString(key, value, static_cast<int>(initialValue));
}

int ConfigSection::ReadAddString(const std::string& key, double* value, double initialValue)
{
    const int result = ReadString(false, key, value);
    if (result <= 0 && value != nullptr) { *value = initialValue; WriteString(key, initialValue); }
    return result;
}

int ConfigSection::ReadAddString(const std::string& key, std::string& value, const std::string& initialValue)
{
    const int result = ReadString(false, key, value);
    if (result <= 0) { value = initialValue; WriteString(key, initialValue); }
    return result;
}
