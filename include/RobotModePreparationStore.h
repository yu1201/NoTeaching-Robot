#pragma once

#include "ConfigDatabase.h"
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QRegularExpression>
#include <QString>

// Brand-neutral persistence only. No hardware calls and no parsing of manual
// acceptance evidence. SaveVerified is called only after the driver proves the
// selected sequence AND restoration on the currently bound controller.
namespace RobotModePreparationStore
{
struct Binding
{
    QString robotName, driver, host, firmware, controllerModel;
    int port = 0;
    int revision = 1; // Driver-owned sequence semantics revision, not app version.
};
struct Record
{
    QString planId, verifiedAtUtc, evidence;
};
enum class LoadStatus { Found, NotFound, Error, Mismatch };

inline QString Module() { return QStringLiteral("RobotPara/ModePreparation"); }
inline QString Key() { return QStringLiteral("SelectedStrategy"); }

inline bool ValidText(const QString& text, int maximumLength)
{
    if (text.isEmpty() || text != text.trimmed() || text.size() > maximumLength) return false;
    for (const QChar c : text) if (c.isNull() || c.category() == QChar::Other_Control) return false;
    return true;
}
inline bool ValidateBinding(const Binding& binding, QString& error)
{
    error.clear();
    const bool valid = ValidText(binding.robotName, 128)
        && !binding.robotName.contains('/') && !binding.robotName.contains('\\')
        && binding.robotName != "." && binding.robotName != ".."
        && ValidText(binding.driver, 128) && ValidText(binding.host, 255)
        && !binding.host.contains(QRegularExpression(QStringLiteral("[\\s/@#]")))
        && ValidText(binding.firmware, 512) && ValidText(binding.controllerModel, 512)
        && binding.port > 0 && binding.port <= 65535 && binding.revision > 0;
    if (!valid)
    {
        error = QStringLiteral("模式组合绑定不完整或无效：必须提供机器人、驱动、控制端点、固件、控制器型号和有效时序版本。");
    }
    return valid;
}
inline bool ValidateRecord(const Record& record, QString& error)
{
    static const QRegularExpression planPattern(QStringLiteral("^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$"));
    const QDateTime timestamp = QDateTime::fromString(record.verifiedAtUtc, Qt::ISODateWithMs);
    const bool valid = ValidText(record.planId, 128) && planPattern.match(record.planId).hasMatch()
        && timestamp.isValid() && record.verifiedAtUtc.endsWith('Z')
        && !record.evidence.trimmed().isEmpty() && record.evidence.size() <= 256 * 1024;
    if (!valid) error = QStringLiteral("模式组合验证记录无效：策略ID、UTC验证时间或验证证据缺失/格式错误。");
    return valid;
}
inline QJsonObject BindingJson(const Binding& binding)
{
    return { {"robotName", binding.robotName}, {"driver", binding.driver},
        {"host", binding.host}, {"port", binding.port}, {"firmware", binding.firmware},
        {"controllerModel", binding.controllerModel}, {"revision", binding.revision} };
}
inline QString Serialize(const QJsonObject& object)
{
    return QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Compact));
}

inline LoadStatus Load(const Binding& expected, Record& record, QString& error)
{
    record = {};
    if (!ValidateBinding(expected, error)) return LoadStatus::Error;
    QString raw;
    const auto status = ConfigDatabase::ReadScopedSettingStatus(
        "robot", expected.robotName, Module(), Key(), &raw);
    if (status == ConfigDatabase::ReadStatus::NotFound) return LoadStatus::NotFound;
    if (status == ConfigDatabase::ReadStatus::Error)
    { error = QStringLiteral("读取已验证模式组合失败：配置数据库或记录解码错误。"); return LoadStatus::Error; }
    QJsonParseError parseError;
    const auto document = QJsonDocument::fromJson(raw.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    { error = QStringLiteral("已验证模式组合JSON损坏，禁止恢复。"); return LoadStatus::Error; }
    const auto object = document.object();
    if (!object.value("schema").isDouble() || object.value("schema").toDouble() != 1.0
        || !object.value("binding").isObject() || !object.value("revoked").isBool()
        || !object.value("passed").isBool() || !object.value("restoreVerified").isBool())
    { error = QStringLiteral("已验证模式组合结构或schema不支持，禁止恢复。"); return LoadStatus::Error; }
    const auto identity = object.value("binding").toObject();
    Binding stored;
    stored.robotName = identity.value("robotName").toString();
    stored.driver = identity.value("driver").toString();
    stored.host = identity.value("host").toString();
    stored.firmware = identity.value("firmware").toString();
    stored.controllerModel = identity.value("controllerModel").toString();
    stored.port = identity.value("port").toInt(-1);
    stored.revision = identity.value("revision").toInt(-1);
    if (!ValidateBinding(stored, error) || !identity.value("port").isDouble()
        || identity.value("port").toDouble() != stored.port || !identity.value("revision").isDouble()
        || identity.value("revision").toDouble() != stored.revision)
    { error = QStringLiteral("已验证模式组合的控制器绑定数据损坏，禁止恢复。"); return LoadStatus::Error; }
    if (BindingJson(stored) != BindingJson(expected))
    { error = QStringLiteral("已验证模式组合绑定不匹配：机器人/驱动/端点/固件/型号/时序版本已变化，请重新验证。"); return LoadStatus::Mismatch; }
    if (object.value("revoked").toBool())
    {
        if (object.value("passed").toBool() || object.value("restoreVerified").toBool())
        { error = QStringLiteral("模式组合撤销记录含冲突的通过标志，禁止恢复。"); return LoadStatus::Error; }
        error = QStringLiteral("该机器人保存的模式组合已撤销，必须重新测试并验证恢复后才能选用。");
        return LoadStatus::NotFound;
    }
    if (!object.value("passed").toBool() || !object.value("restoreVerified").toBool())
    { error = QStringLiteral("模式组合未同时通过测试和恢复验证，禁止恢复。"); return LoadStatus::Error; }
    Record loaded { object.value("planId").toString(), object.value("verifiedAtUtc").toString(), object.value("evidence").toString() };
    if (!ValidateRecord(loaded, error)) return LoadStatus::Error;
    record = loaded;
    error.clear();
    return LoadStatus::Found;
}

inline bool WriteAtomically(const Binding& binding, const QJsonObject& object, QString& error)
{
    QString previous;
    const auto status = ConfigDatabase::ReadScopedSettingStatus(
        "robot", binding.robotName, Module(), Key(), &previous);
    if (status == ConfigDatabase::ReadStatus::Error)
    { error = QStringLiteral("模式组合更新前读取失败，原记录未覆盖。"); return false; }
    bool conflict = false;
    // ConfigDatabase implements BEGIN IMMEDIATE, compare, write, exact plaintext
    // readback and COMMIT. Write/readback errors roll back the entire mutation.
    return ConfigDatabase::CompareAndSwapScopedSetting(
        "robot", binding.robotName, Module(), Key(),
        status == ConfigDatabase::ReadStatus::Found ? &previous : nullptr,
        Serialize(object), "json", false, &conflict, &error);
}
inline bool SaveVerified(const Binding& binding, const Record& record, QString& error)
{
    if (!ValidateBinding(binding, error) || !ValidateRecord(record, error)) return false;
    return WriteAtomically(binding, {
        {"schema", 1}, {"binding", BindingJson(binding)}, {"revoked", false},
        {"passed", true}, {"restoreVerified", true}, {"planId", record.planId},
        {"verifiedAtUtc", record.verifiedAtUtc}, {"evidence", record.evidence} }, error);
}
inline bool Revoke(const Binding& binding, QString& error)
{
    if (!ValidateBinding(binding, error)) return false;
    return WriteAtomically(binding, {
        {"schema", 1}, {"binding", BindingJson(binding)}, {"revoked", true},
        {"passed", false}, {"restoreVerified", false},
        {"revokedAtUtc", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)} }, error);
}
}
