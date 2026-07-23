#include "WeldSeamCompConfig.h"

#include "ConfigDatabase.h"
#include "RobotMessage.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr int LEGACY_ROWS_PER_GROUP = 4;
constexpr int LEGACY_POSE_AVERAGE_GROUP_COUNT = 4;
constexpr int MAX_REASONABLE_GROUP_COUNT = 10000;
constexpr char ALL_SECTION[] = "ALLWeldSeamComp";
constexpr char SCHEMA_VERSION_KEY[] = "SeamCompSchemaVersion";
constexpr char COUNT_KEY[] = "SeamCompCount";
constexpr char GROUP_COUNT_KEY[] = "SeamCompGroupCount";
constexpr char ACTIVE_GROUP_KEY[] = "ActiveSeamCompGroupIndex";
constexpr char SIMPLIFY_KEY[] = "SimplifyKeepAnchorsOnly";

using IniSection = QMap<QString, QString>;
using IniFileSnapshot = QMap<QString, IniSection>;

const IniSection& SnapshotSection(const IniFileSnapshot& snapshot, const QString& sectionName)
{
    const auto it = snapshot.constFind(sectionName);
    if (it != snapshot.cend())
    {
        return it.value();
    }
    static const IniSection empty;
    return empty;
}

bool ReadInt(const IniSection& section, const char* key, int& value)
{
    const auto it = section.constFind(QString::fromLatin1(key));
    if (it == section.cend())
    {
        return false;
    }
    bool ok = false;
    const int loaded = it.value().trimmed().toInt(&ok);
    if (!ok)
    {
        return false;
    }
    value = loaded;
    return true;
}

bool ReadDouble(const IniSection& section, const char* key, double& value)
{
    const auto it = section.constFind(QString::fromLatin1(key));
    if (it == section.cend())
    {
        return false;
    }
    bool ok = false;
    const double loaded = it.value().trimmed().toDouble(&ok);
    if (!ok || !std::isfinite(loaded))
    {
        return false;
    }
    value = loaded;
    return true;
}

bool IsNonZero(const WeldSeamCompConfig::Values& values)
{
    constexpr double epsilon = 1e-9;
    return std::abs(values.weldZComp) > epsilon
        || std::abs(values.weldGunDirComp) > epsilon
        || std::abs(values.weldSeamDirComp) > epsilon;
}

bool NearlyEqual(const WeldSeamCompConfig::Values& left, const WeldSeamCompConfig::Values& right)
{
    constexpr double epsilon = 1e-9;
    return std::abs(left.weldZComp - right.weldZComp) <= epsilon
        && std::abs(left.weldGunDirComp - right.weldGunDirComp) <= epsilon
        && std::abs(left.weldSeamDirComp - right.weldSeamDirComp) <= epsilon;
}

bool ReadValues(const IniSection& section, WeldSeamCompConfig::Values& values)
{
    bool readAny = false;
    readAny = ReadDouble(section, "WeldZComp", values.weldZComp) || readAny;
    readAny = ReadDouble(section, "WeldGunDirComp", values.weldGunDirComp) || readAny;
    readAny = ReadDouble(section, "WeldSeamDirComp", values.weldSeamDirComp) || readAny;
    return readAny;
}

bool FiniteValues(const WeldSeamCompConfig::Values& values)
{
    return std::isfinite(values.weldZComp)
        && std::isfinite(values.weldGunDirComp)
        && std::isfinite(values.weldSeamDirComp);
}

struct LegacyCandidate
{
    QString segmentKind;
    WeldSeamCompConfig::Values values;
};

int SelectLegacyCandidate(const QVector<LegacyCandidate>& candidates, bool& conflict)
{
    conflict = false;
    QVector<int> nonZeroIndices;
    int corrugatedIndex = -1;
    for (int index = 0; index < candidates.size(); ++index)
    {
        if (candidates[index].segmentKind.compare(QStringLiteral("CorrugatedPlate"), Qt::CaseInsensitive) == 0)
        {
            corrugatedIndex = index;
        }
        if (IsNonZero(candidates[index].values))
        {
            nonZeroIndices.push_back(index);
        }
    }

    if (nonZeroIndices.isEmpty())
    {
        return corrugatedIndex >= 0 ? corrugatedIndex : 0;
    }
    if (nonZeroIndices.size() == 1)
    {
        return nonZeroIndices.front();
    }

    const WeldSeamCompConfig::Values& first = candidates[nonZeroIndices.front()].values;
    bool allEqual = true;
    for (int index : nonZeroIndices)
    {
        if (!NearlyEqual(first, candidates[index].values))
        {
            allEqual = false;
            break;
        }
    }
    if (allEqual)
    {
        return nonZeroIndices.front();
    }

    conflict = true;
    if (corrugatedIndex >= 0 && IsNonZero(candidates[corrugatedIndex].values))
    {
        return corrugatedIndex;
    }
    return nonZeroIndices.front();
}

QString DefaultGroupName(int index)
{
    return QStringLiteral("焊道补偿组%1").arg(index + 1);
}
}

namespace WeldSeamCompConfig
{
Document MakeDefaultDocument()
{
    Document document;
    Group group;
    group.name = DefaultGroupName(0);
    document.groups.push_back(group);
    document.storedGroupCount = 1;
    return document;
}

bool Load(const QString& path, Document& document, QString& error)
{
    error.clear();
    document = MakeDefaultDocument();
    if (path.trimmed().isEmpty())
    {
        error = QStringLiteral("焊道补偿配置路径为空。");
        return false;
    }
    IniFileSnapshot snapshot;
    if (!ConfigDatabase::ReadIniFileSnapshot(path, snapshot, &error))
    {
        return false;
    }
    if (snapshot.isEmpty())
    {
        return true;
    }
    document.sourceExists = true;

    int schemaVersion = 0;
    int count = 0;
    int groupCount = 0;
    int activeGroupIndex = 0;
    int simplify = 0;
    const IniSection& allSection = SnapshotSection(snapshot, QString::fromLatin1(ALL_SECTION));
    ReadInt(allSection, SCHEMA_VERSION_KEY, schemaVersion);
    ReadInt(allSection, COUNT_KEY, count);
    const bool hasGroupCount = ReadInt(allSection, GROUP_COUNT_KEY, groupCount);
    ReadInt(allSection, ACTIVE_GROUP_KEY, activeGroupIndex);
    ReadInt(allSection, SIMPLIFY_KEY, simplify);
    document.simplifyKeepAnchorsOnly = simplify != 0;

    const bool v2 = schemaVersion >= SCHEMA_VERSION;
    if (v2)
    {
        groupCount = hasGroupCount ? groupCount : count;
    }
    else
    {
        document.loadedFromLegacy = true;
        count = std::max(0, count);
        groupCount = hasGroupCount
            ? groupCount
            : (count <= 0 ? 1 : (count + LEGACY_ROWS_PER_GROUP - 1) / LEGACY_ROWS_PER_GROUP);
        document.legacySectionCount = std::max(count, std::max(0, groupCount) * LEGACY_ROWS_PER_GROUP);
    }

    groupCount = std::clamp(groupCount, 0, MAX_REASONABLE_GROUP_COUNT);
    document.storedGroupCount = groupCount;
    document.groups.clear();
    document.groups.reserve(groupCount);

    for (int groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        Group group;
        group.name = DefaultGroupName(groupIndex);
        const IniSection& groupSection = SnapshotSection(
            snapshot, QStringLiteral("WeldSeamCompGroup%1").arg(groupIndex));
        const QString encodedName = groupSection.value(QStringLiteral("Name"));
        if (!encodedName.isEmpty())
        {
            const QByteArray encodedNameBytes = encodedName.toUtf8();
            const QString decoded = DecodeRobotMessageText(encodedNameBytes.constData()).trimmed();
            if (!decoded.isEmpty())
            {
                group.name = decoded;
            }
        }

        if (v2)
        {
            ReadValues(groupSection, group.values);
        }
        else
        {
            QVector<LegacyCandidate> candidates;
            candidates.reserve(LEGACY_ROWS_PER_GROUP);
            const int baseIndex = groupIndex * LEGACY_ROWS_PER_GROUP;
            for (int slotIndex = 0; slotIndex < LEGACY_ROWS_PER_GROUP; ++slotIndex)
            {
                LegacyCandidate candidate;
                const IniSection& legacySection = SnapshotSection(
                    snapshot, QStringLiteral("WeldSeamComp%1").arg(baseIndex + slotIndex));
                const QString encodedKind = legacySection.value(QStringLiteral("SegmentKind"));
                if (!encodedKind.isEmpty())
                {
                    const QByteArray encodedKindBytes = encodedKind.toUtf8();
                    candidate.segmentKind = DecodeRobotMessageText(encodedKindBytes.constData()).trimmed();
                }
                ReadValues(legacySection, candidate.values);
                candidates.push_back(candidate);
            }

            bool groupConflict = false;
            const int selectedIndex = SelectLegacyCandidate(candidates, groupConflict);
            if (selectedIndex >= 0 && selectedIndex < candidates.size())
            {
                group.values = candidates[selectedIndex].values;
            }
            const QString selectedKind = selectedIndex >= 0 && selectedIndex < candidates.size()
                ? candidates[selectedIndex].segmentKind
                : QString();
            if (groupConflict)
            {
                document.legacyValuesConflict = true;
                document.warnings.push_back(QStringLiteral(
                    "旧焊道补偿组“%1”存在多个不同的非零段类型值；当前采用 %2，旧槽位将保留且不会自动清理。")
                    .arg(group.name, selectedKind.isEmpty() ? QStringLiteral("首个非零槽") : selectedKind));
            }
            else
            {
                document.warnings.push_back(QStringLiteral(
                    "旧焊道补偿组“%1”已折叠为整条焊道统一补偿（来源：%2）。")
                    .arg(group.name, selectedKind.isEmpty() ? QStringLiteral("旧槽位") : selectedKind));
            }
        }
        document.groups.push_back(group);
    }

    if (document.groups.isEmpty())
    {
        document.activeGroupIndex = 0;
    }
    else
    {
        document.activeGroupIndex = std::clamp(activeGroupIndex, 0, static_cast<int>(document.groups.size()) - 1);
    }
    return true;
}

bool SaveV2(const QString& path, const Document& document, QString& error)
{
    error.clear();
    if (path.trimmed().isEmpty())
    {
        error = QStringLiteral("焊道补偿配置路径为空。");
        return false;
    }
    if (document.groups.size() > MAX_REASONABLE_GROUP_COUNT)
    {
        error = QStringLiteral("焊道补偿组数量异常。");
        return false;
    }
    for (const Group& group : document.groups)
    {
        if (!FiniteValues(group.values))
        {
            error = QStringLiteral("焊道补偿包含无效数值。");
            return false;
        }
    }

    const int groupCount = static_cast<int>(document.groups.size());
    const int activeGroupIndex = groupCount <= 0
        ? 0
        : std::clamp(document.activeGroupIndex, 0, groupCount - 1);

    QMap<QString, QMap<QString, QString>> sections;
    for (int groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        const Group& group = document.groups[groupIndex];
        QMap<QString, QString> values;
        values.insert(QStringLiteral("Name"),
            group.name.trimmed().isEmpty() ? DefaultGroupName(groupIndex) : group.name);
        values.insert(QStringLiteral("WeldZComp"), QString::number(group.values.weldZComp, 'f', 6));
        values.insert(QStringLiteral("WeldGunDirComp"), QString::number(group.values.weldGunDirComp, 'f', 6));
        values.insert(QStringLiteral("WeldSeamDirComp"), QString::number(group.values.weldSeamDirComp, 'f', 6));
        sections.insert(QStringLiteral("WeldSeamCompGroup%1").arg(groupIndex), values);
    }

    QMap<QString, QString> allValues;
    allValues.insert(QString::fromLatin1(COUNT_KEY), QString::number(groupCount));
    allValues.insert(QString::fromLatin1(GROUP_COUNT_KEY), QString::number(groupCount));
    allValues.insert(QString::fromLatin1(ACTIVE_GROUP_KEY), QString::number(activeGroupIndex));
    allValues.insert(QString::fromLatin1(SIMPLIFY_KEY), document.simplifyKeepAnchorsOnly ? QStringLiteral("1") : QStringLiteral("0"));
    allValues.insert(QString::fromLatin1(SCHEMA_VERSION_KEY), QString::number(SCHEMA_VERSION));
    sections.insert(QString::fromLatin1(ALL_SECTION), allValues);

    const int oldGroupCount = std::clamp(document.storedGroupCount, 0, MAX_REASONABLE_GROUP_COUNT);
    QStringList removeSections;
    removeSections.push_back(QString::fromLatin1(ALL_SECTION));
    removeSections.push_back(QStringLiteral("WeldSeamPoseAverage"));
    for (int groupIndex = 0; groupIndex < LEGACY_POSE_AVERAGE_GROUP_COUNT; ++groupIndex)
    {
        removeSections.push_back(QStringLiteral("WeldSeamPoseAverage%1").arg(groupIndex));
    }
    const int replaceGroupCount = std::max(groupCount, oldGroupCount);
    for (int groupIndex = 0; groupIndex < replaceGroupCount; ++groupIndex)
    {
        removeSections.push_back(QStringLiteral("WeldSeamCompGroup%1").arg(groupIndex));
    }

    if (document.loadedFromLegacy && !document.legacyValuesConflict)
    {
        const int cleanupCount = std::clamp(document.legacySectionCount, 0, MAX_REASONABLE_GROUP_COUNT * LEGACY_ROWS_PER_GROUP);
        for (int sectionIndex = 0; sectionIndex < cleanupCount; ++sectionIndex)
        {
            removeSections.push_back(QStringLiteral("WeldSeamComp%1").arg(sectionIndex));
        }
    }

    // 分组值、版本标记和旧槽清理在同一事务内提交。已有 v2 配置重复保存时，
    // 其他读取者也只能看到提交前或提交后的完整快照。
    if (!ConfigDatabase::ReplaceIniSectionsAtomically(path, sections, removeSections, &error))
    {
        return false;
    }

    Document verified;
    QString verifyError;
    if (!Load(path, verified, verifyError)
        || verified.loadedFromLegacy
        || verified.groups.size() != document.groups.size()
        || verified.activeGroupIndex != activeGroupIndex
        || verified.simplifyKeepAnchorsOnly != document.simplifyKeepAnchorsOnly)
    {
        error = QStringLiteral("回读校验焊道补偿配置失败：%1").arg(verifyError);
        return false;
    }
    for (int groupIndex = 0; groupIndex < groupCount; ++groupIndex)
    {
        if (!NearlyEqual(verified.groups[groupIndex].values, document.groups[groupIndex].values))
        {
            error = QStringLiteral("回读校验焊道补偿组失败：WeldSeamCompGroup%1").arg(groupIndex);
            return false;
        }
    }
    return true;
}
}
