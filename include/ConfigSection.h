#pragma once

#include "Const.h"

#include <QString>

#include <string>

struct ConfigLocation
{
    QString scopeType;
    QString scopeId;
    QString module;

    static ConfigLocation Global(const QString& moduleName);
    static ConfigLocation Robot(const QString& robotName, const QString& moduleName);
    static ConfigLocation RobotTypeTemplate(const QString& templateId, const QString& moduleName);
    static ConfigLocation WorkpieceTemplate(const QString& templateName, const QString& moduleName);
    static ConfigLocation Result(const QString& resultId, const QString& moduleName);

    bool IsValid() const;
};

// Database-native section reader/writer. A location is always expressed as
// scope + object id + module; filesystem paths are deliberately not accepted.
class ConfigSection
{
public:
    ConfigSection();
    explicit ConfigSection(const ConfigLocation& location);

    bool SetLocation(const ConfigLocation& location);
    bool SetSectionName(const std::string& sectionName);
    bool HasData() const;
    bool CheckExists(const std::string& key) const;

    bool WriteString(const std::string& key, const std::string& value);
    bool WriteString(const std::string& key, double value, unsigned int decimalDigits = 6);
    bool WriteString(const std::string& key, int value);
    bool WriteString(const std::string& key, long value);
    bool WriteString(const std::string& key, bool value);
    bool WriteString(const std::string& key1, const std::string& key2, T_ANGLE_PULSE pulse,
        T_ANGLE_PULSE fields = T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, 1, 1, 1));
    bool WriteString(const std::string& key1, const std::string& key2, T_ROBOT_COORS coors,
        T_ROBOT_COORS fields = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1));

    int ReadString(const std::string& key, char value[]) const;
    int ReadString(const std::string& key, double* value) const;
    int ReadString(const std::string& key, float* value) const;
    int ReadString(const std::string& key, int* value) const;
    int ReadString(const std::string& key, long* value) const;
    int ReadString(const std::string& key, long long* value) const;
    int ReadString(const std::string& key, unsigned long* value) const;
    int ReadString(const std::string& key, std::string& value) const;
    int ReadString(const std::string& key, bool* value, bool check = true) const;
    int ReadString(const std::string& key1, const std::string& key2, T_ANGLE_PULSE& pulse,
        T_ANGLE_PULSE fields = T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, 1, 1, 1)) const;
    int ReadString(const std::string& key1, const std::string& key2, T_ROBOT_COORS& coors,
        T_ROBOT_COORS fields = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1)) const;

    int ReadString(bool check, const std::string& key, bool* value) const;
    int ReadString(bool check, const std::string& key, int* value) const;
    int ReadString(bool check, const std::string& key, long* value) const;
    int ReadString(bool check, const std::string& key, double* value) const;
    int ReadString(bool check, const std::string& key, std::string& value) const;
    int ReadString(bool check, const std::string& key, std::string* value) const;

    int ReadAddString(const std::string& key, bool* value, bool initialValue);
    int ReadAddString(const std::string& key, int* value, int initialValue);
    int ReadAddString(const std::string& key, int* value, long initialValue);
    int ReadAddString(const std::string& key, double* value, double initialValue);
    int ReadAddString(const std::string& key, std::string& value, const std::string& initialValue);

    const ConfigLocation& Location() const { return m_location; }
    QString ModuleName() const;

private:
    int ReadRaw(const std::string& key, std::string* value, bool check) const;
    bool WriteRaw(const std::string& key, const std::string& value);
    void CheckRead(const std::string& key, int result, bool check) const;

    ConfigLocation m_location;
    QString m_sectionName;
};
