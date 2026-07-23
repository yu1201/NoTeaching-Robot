#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

namespace WeldSeamCompConfig
{
constexpr int SCHEMA_VERSION = 2;

struct Values
{
    double weldZComp = 0.0;
    double weldGunDirComp = 0.0;
    double weldSeamDirComp = 0.0;
};

struct Group
{
    QString name;
    Values values;
};

struct Document
{
    QVector<Group> groups;
    int activeGroupIndex = 0;
    bool simplifyKeepAnchorsOnly = false;
    bool sourceExists = false;
    bool loadedFromLegacy = false;
    bool legacyValuesConflict = false;
    int legacySectionCount = 0;
    int storedGroupCount = 0;
    QStringList warnings;
};

Document MakeDefaultDocument();
bool Load(const QString& path, Document& document, QString& error);
bool SaveV2(const QString& path, const Document& document, QString& error);
}
