#include "WeldProcessFile.h"

#include "ConfigDatabase.h"
#include "RobotLog.h"
#include "RobotMessage.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <sstream>
#include <vector>

namespace
{
RobotLog& WeldProcessLogger()
{
    static RobotLog logger(".//Log//WeldProcessFile.txt");
    return logger;
}

constexpr char kDelimiter = '\t';
constexpr int kWeaveFieldCount = 15;
constexpr int kWeldCoreFieldCount = 48;
constexpr int kWeldTrackFieldCount = 33;
constexpr int kWeldSwitchFieldCount = 2;
constexpr int kWeldArcModeFieldCount = 1;
constexpr int kWeldFieldCount = kWeldCoreFieldCount + kWeldTrackFieldCount + kWeldSwitchFieldCount + kWeldArcModeFieldCount;

int NormalizeArcMode(int mode)
{
    return mode >= 0 && mode <= 7 ? mode : 4;
}
}

WeldProcessFile::WeldProcessFile(const T_CONTRAL_UNIT& tContralUnitInfo)
    : m_tContralUnitInfo(tContralUnitInfo)
{
}

WeldProcessFile::WeldProcessFile(const ContralUnit& contralUnit, int nUnitIndex)
{
    if (nUnitIndex < 0 || nUnitIndex >= static_cast<int>(contralUnit.m_vtContralUnitInfo.size()))
    {
        m_sLastError = "控制单元索引越界，无法读取工艺文件。";
        return;
    }
    m_tContralUnitInfo = contralUnit.m_vtContralUnitInfo[nUnitIndex];
}

WeldProcessFile::~WeldProcessFile()
{
}

bool WeldProcessFile::Init()
{
    return LoadFromControlUnit(m_tContralUnitInfo);
}

bool WeldProcessFile::PrepareForRecreate()
{
    m_sLastError.clear();
    m_nAllWeaveTypeNum = 0;
    m_nUseWeaveTypeNo = 0;
    m_nAllWeldParaNum = 0;
    m_nUseWeldParaNo = 0;
    m_vtWeaveTypeList.clear();
    m_vtWeldParaList.clear();

    if (m_tContralUnitInfo.sUnitName.empty())
    {
        m_sLastError = "控制单元名称为空，无法重新创建工艺内容。";
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    m_sWeaveIniFilePath = BuildWeaveIniPath(m_tContralUnitInfo.sUnitName);
    m_sWeldIniFilePath = BuildWeldIniPath(m_tContralUnitInfo.sUnitName);
    EnsureGlobalStorage(m_tContralUnitInfo.nUnitNo);
    return true;
}

bool WeldProcessFile::LoadFromControlUnit(const ContralUnit& contralUnit, int nUnitIndex)
{
    if (nUnitIndex < 0 || nUnitIndex >= static_cast<int>(contralUnit.m_vtContralUnitInfo.size()))
    {
        m_sLastError = "控制单元索引越界，无法读取工艺文件。";
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    return LoadFromControlUnit(contralUnit.m_vtContralUnitInfo[nUnitIndex]);
}

bool WeldProcessFile::LoadFromControlUnit(const T_CONTRAL_UNIT& tContralUnitInfo)
{
    m_sLastError.clear();
    m_tContralUnitInfo = tContralUnitInfo;
    m_nAllWeaveTypeNum = 0;
    m_nUseWeaveTypeNo = 0;
    m_nAllWeldParaNum = 0;
    m_nUseWeldParaNo = 0;
    m_vtWeaveTypeList.clear();
    m_vtWeldParaList.clear();

    if (tContralUnitInfo.sUnitName.empty())
    {
        m_sLastError = "控制单元名称为空，无法定位工艺文件。";
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    m_sWeaveIniFilePath = BuildWeaveIniPath(tContralUnitInfo.sUnitName);
    m_sWeldIniFilePath = BuildWeldIniPath(tContralUnitInfo.sUnitName);
    EnsureGlobalStorage(tContralUnitInfo.nUnitNo);

    if (!ConfigDatabase::HasTextFile(m_sWeaveIniFilePath) || !ConfigDatabase::HasTextFile(m_sWeldIniFilePath))
    {
        m_sLastError = GetStr("配置库中未找到工艺数据：%s / %s",
            m_sWeaveIniFilePath.c_str(),
            m_sWeldIniFilePath.c_str());
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    if (!LoadWeaveTxt())
    {
        LogError("%s", m_sLastError.c_str());
        return false;
    }
    if (!LoadWeldTxt())
    {
        LogError("%s", m_sLastError.c_str());
        return false;
    }
    if (!BindWeldToWeave())
    {
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    LogInfo("工艺数据读取成功，控制单元: %s, 摆动类型数: %d, 工艺数: %d",
        tContralUnitInfo.sUnitName.c_str(), m_nAllWeaveTypeNum, m_nAllWeldParaNum);
    return true;
}

const std::string& WeldProcessFile::GetWeaveIniFilePath() const
{
    return m_sWeaveIniFilePath;
}

const std::string& WeldProcessFile::GetWeldIniFilePath() const
{
    return m_sWeldIniFilePath;
}

std::string WeldProcessFile::GetLastError() const
{
    return m_sLastError;
}

const std::vector<T_WeaveDate>& WeldProcessFile::GetWeaveTypeList() const
{
    return m_vtWeaveTypeList;
}

const std::vector<T_WELD_PARA>& WeldProcessFile::GetWeldParaList() const
{
    return m_vtWeldParaList;
}

int WeldProcessFile::GetAllWeaveTypeNum() const
{
    return m_nAllWeaveTypeNum;
}

int WeldProcessFile::GetUseWeaveTypeNo() const
{
    return m_nUseWeaveTypeNo;
}

int WeldProcessFile::GetAllWeldParaNum() const
{
    return m_nAllWeldParaNum;
}

int WeldProcessFile::GetUseWeldParaNo() const
{
    return m_nUseWeldParaNo;
}

const T_WeaveDate* WeldProcessFile::GetUseWeaveType() const
{
    if (m_nUseWeaveTypeNo < 0 || m_nUseWeaveTypeNo >= static_cast<int>(m_vtWeaveTypeList.size()))
    {
        return nullptr;
    }
    return &m_vtWeaveTypeList[m_nUseWeaveTypeNo];
}

const T_WELD_PARA* WeldProcessFile::GetUseWeldPara() const
{
    if (m_nUseWeldParaNo < 0 || m_nUseWeldParaNo >= static_cast<int>(m_vtWeldParaList.size()))
    {
        return nullptr;
    }
    return &m_vtWeldParaList[m_nUseWeldParaNo];
}

bool WeldProcessFile::UpdateUseWeaveTypeNo(int nUseWeaveTypeNo)
{
    if (nUseWeaveTypeNo < 0 || nUseWeaveTypeNo >= m_nAllWeaveTypeNum)
    {
        m_sLastError = "UseWeaveTypeNo 超出范围。";
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    m_nUseWeaveTypeNo = nUseWeaveTypeNo;
    if (!SaveWeaveTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::UpdateUseWeldParaNo(int nUseWeldParaNo)
{
    if (nUseWeldParaNo < 0 || nUseWeldParaNo >= m_nAllWeldParaNum)
    {
        m_sLastError = "UseWeldParaNo 超出范围。";
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    m_nUseWeldParaNo = nUseWeldParaNo;
    if (!SaveWeldTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::UpdateWeaveType(int nTypeNo, const T_WeaveDate& tWeaveDate)
{
    if (nTypeNo < 0 || nTypeNo >= m_nAllWeaveTypeNum)
    {
        m_sLastError = "摆动类型号超出范围。";
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    m_vtWeaveTypeList[nTypeNo] = tWeaveDate;
    if (!SaveWeaveTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::UpdateWeldPara(int nParaNo, const T_WELD_PARA& tWeldPara)
{
    if (nParaNo < 0 || nParaNo >= m_nAllWeldParaNum)
    {
        m_sLastError = "焊接工艺号超出范围。";
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    T_WELD_PARA item = tWeldPara;
    item.nWeaveEnable = item.nWeaveEnable != 0 ? 1 : 0;
    item.nTrackEnable = item.nTrackEnable != 0 ? 1 : 0;
    item.nArcMode = NormalizeArcMode(item.nArcMode);
    if (item.nWeaveTypeNo < 0 || item.nWeaveTypeNo >= static_cast<int>(m_vtWeaveTypeList.size()))
    {
        item.nWeaveTypeNo = 0;
    }
    item.tWeaveParam = item.nWeaveEnable != 0 && !m_vtWeaveTypeList.empty()
        ? m_vtWeaveTypeList[item.nWeaveTypeNo]
        : T_WeaveDate {};
    m_vtWeldParaList[nParaNo] = item;
    NormalizeWeldOrderKeepGroupOrder();
    if (!SaveWeldTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::AddWeldPara(const T_WELD_PARA& tWeldPara, int& newIndex)
{
    if (m_sWeaveIniFilePath.empty() || m_sWeldIniFilePath.empty())
    {
        if (!PrepareForRecreate())
        {
            return false;
        }
    }

    if (m_vtWeaveTypeList.empty())
    {
        m_vtWeaveTypeList.push_back(T_WeaveDate {});
        m_nAllWeaveTypeNum = 1;
        m_nUseWeaveTypeNo = 0;
        if (!SaveWeaveTxt())
        {
            LogError("%s", m_sLastError.c_str());
            ShowError(m_sLastError);
            return false;
        }
    }

    T_WELD_PARA item = tWeldPara;
    item.nWeaveEnable = item.nWeaveEnable != 0 ? 1 : 0;
    item.nTrackEnable = item.nTrackEnable != 0 ? 1 : 0;
    item.nArcMode = NormalizeArcMode(item.nArcMode);
    if (item.nWeaveTypeNo < 0 || item.nWeaveTypeNo >= static_cast<int>(m_vtWeaveTypeList.size()))
    {
        item.nWeaveTypeNo = 0;
    }
    item.tWeaveParam = item.nWeaveEnable != 0 ? m_vtWeaveTypeList[item.nWeaveTypeNo] : T_WeaveDate {};
    m_vtWeldParaList.push_back(item);
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    m_nUseWeldParaNo = static_cast<int>(m_vtWeldParaList.size()) - 1;
    newIndex = m_nUseWeldParaNo;
    if (!SaveWeldTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::RemoveWeldPara(int nParaNo)
{
    if (nParaNo < 0 || nParaNo >= static_cast<int>(m_vtWeldParaList.size()))
    {
        m_sLastError = "焊接工艺号超出范围。";
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    m_vtWeldParaList.erase(m_vtWeldParaList.begin() + nParaNo);
    NormalizeWeldOrderKeepGroupOrder();
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    if (m_nAllWeldParaNum <= 0)
    {
        m_nUseWeldParaNo = 0;
    }
    else
    {
        m_nUseWeldParaNo = qBound(0, m_nUseWeldParaNo, m_nAllWeldParaNum - 1);
    }

    if (!SaveWeldTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::ReorderWeldGroups(const std::vector<std::string>& orderedGroupKeys)
{
    if (orderedGroupKeys.empty())
    {
        m_sLastError = "工艺分组顺序为空。";
        return false;
    }

    T_WELD_PARA selected {};
    bool hasSelected = false;
    if (m_nUseWeldParaNo >= 0 && m_nUseWeldParaNo < static_cast<int>(m_vtWeldParaList.size()))
    {
        selected = m_vtWeldParaList[m_nUseWeldParaNo];
        hasSelected = true;
    }

    std::unordered_map<std::string, std::vector<T_WELD_PARA>> grouped;
    std::vector<std::string> existingOrder;
    for (const auto& item : m_vtWeldParaList)
    {
        const std::string key = BuildGroupKey(item);
        if (grouped.find(key) == grouped.end())
        {
            existingOrder.push_back(key);
        }
        grouped[key].push_back(item);
    }

    std::vector<T_WELD_PARA> reordered;
    for (const auto& key : orderedGroupKeys)
    {
        auto it = grouped.find(key);
        if (it == grouped.end())
        {
            continue;
        }
        auto& rows = it->second;
        std::sort(rows.begin(), rows.end(), [](const T_WELD_PARA& a, const T_WELD_PARA& b)
            {
                return a.nLayerNo < b.nLayerNo;
            });
        for (int i = 0; i < static_cast<int>(rows.size()); ++i)
        {
            rows[i].nLayerNo = i + 1;
        }
        reordered.insert(reordered.end(), rows.begin(), rows.end());
        grouped.erase(it);
    }

    for (const auto& key : existingOrder)
    {
        auto it = grouped.find(key);
        if (it == grouped.end())
        {
            continue;
        }
        auto& rows = it->second;
        std::sort(rows.begin(), rows.end(), [](const T_WELD_PARA& a, const T_WELD_PARA& b)
            {
                return a.nLayerNo < b.nLayerNo;
            });
        for (int i = 0; i < static_cast<int>(rows.size()); ++i)
        {
            rows[i].nLayerNo = i + 1;
        }
        reordered.insert(reordered.end(), rows.begin(), rows.end());
    }

    m_vtWeldParaList = reordered;
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());

    if (hasSelected)
    {
        for (int i = 0; i < static_cast<int>(m_vtWeldParaList.size()); ++i)
        {
            const auto& item = m_vtWeldParaList[i];
            if (item.strWorkPeace == selected.strWorkPeace
                && item.strWeldType == selected.strWeldType
                && std::abs(item.dWeldAngleSize - selected.dWeldAngleSize) <= 1e-6
                && item.nLayerNo == selected.nLayerNo)
            {
                m_nUseWeldParaNo = i;
                break;
            }
        }
    }

    if (!SaveWeldTxt())
    {
        return false;
    }
    return true;
}

void WeldProcessFile::EnsureGlobalStorage(int nUnitNo)
{
    (void)nUnitNo;
}

std::string WeldProcessFile::BuildWeaveIniPath(const std::string& unitName) const
{
    return "Data/" + unitName + "/WeaveDate.txt";
}

std::string WeldProcessFile::BuildWeldIniPath(const std::string& unitName) const
{
    return "Data/" + unitName + "/WeldPara.txt";
}

bool WeldProcessFile::LoadWeaveTxt()
{
    std::string content;
    if (!ConfigDatabase::ReadTextFile(m_sWeaveIniFilePath, &content))
    {
        m_sLastError = "从配置库读取摆动数据失败。";
        return false;
    }
    std::istringstream in(content);

    m_vtWeaveTypeList.clear();
    m_nUseWeaveTypeNo = 0;

    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        const auto fields = SplitLine(line, kDelimiter);
        if (fields.empty())
        {
            continue;
        }

        if (fields[0] == "USE")
        {
            if (fields.size() < 2 || !TryParseInt(fields[1], m_nUseWeaveTypeNo))
            {
                m_sLastError = "读取摆动数据 USE 行失败。";
                return false;
            }
            continue;
        }

        T_WeaveDate weave {};
        if (!ParseWeaveLine(fields, weave))
        {
            if (m_sLastError.empty())
            {
                m_sLastError = "摆动数据行格式错误。";
            }
            return false;
        }
        m_vtWeaveTypeList.push_back(weave);
    }

    m_nAllWeaveTypeNum = static_cast<int>(m_vtWeaveTypeList.size());
    if (m_nAllWeaveTypeNum <= 0)
    {
        m_sLastError = "摆动数据中没有有效数据。";
        return false;
    }
    m_nUseWeaveTypeNo = qBound(0, m_nUseWeaveTypeNo, m_nAllWeaveTypeNum - 1);
    return true;
}

bool WeldProcessFile::LoadWeldTxt()
{
    std::string content;
    if (!ConfigDatabase::ReadTextFile(m_sWeldIniFilePath, &content))
    {
        m_sLastError = "从配置库读取焊接工艺数据失败。";
        return false;
    }
    std::istringstream in(content);

    m_vtWeldParaList.clear();
    m_nUseWeldParaNo = 0;

    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        const auto fields = SplitLine(line, kDelimiter);
        if (fields.empty())
        {
            continue;
        }

        if (fields[0] == "USE")
        {
            if (fields.size() < 2 || !TryParseInt(fields[1], m_nUseWeldParaNo))
            {
                m_sLastError = "读取焊接工艺数据 USE 行失败。";
                return false;
            }
            continue;
        }

        T_WELD_PARA weld {};
        if (!ParseWeldLine(fields, weld))
        {
            if (m_sLastError.empty())
            {
                m_sLastError = "焊接工艺数据行格式错误。";
            }
            return false;
        }
        m_vtWeldParaList.push_back(weld);
    }

    NormalizeWeldOrderKeepGroupOrder();
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    if (m_nAllWeldParaNum <= 0)
    {
        m_sLastError = "焊接工艺数据中没有有效数据。";
        return false;
    }
    m_nUseWeldParaNo = qBound(0, m_nUseWeldParaNo, m_nAllWeldParaNum - 1);
    return true;
}

bool WeldProcessFile::SaveWeaveTxt() const
{
    std::ostringstream out;
    out << "# WeaveData\n";
    out << "USE" << kDelimiter << m_nUseWeaveTypeNo << "\n";
    for (const auto& item : m_vtWeaveTypeList)
    {
        out << JoinLine(BuildWeaveFields(item), kDelimiter) << "\n";
    }
    if (!ConfigDatabase::WriteTextFile(m_sWeaveIniFilePath, out.str()))
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "写入配置库摆动数据失败。";
        return false;
    }
    return true;
}

bool WeldProcessFile::SaveWeldTxt() const
{
    std::ostringstream out;
    out << "# WeldParameters\n";
    out << "USE" << kDelimiter << m_nUseWeldParaNo << "\n";
    for (const auto& item : m_vtWeldParaList)
    {
        out << JoinLine(BuildWeldFields(item), kDelimiter) << "\n";
    }
    if (!ConfigDatabase::WriteTextFile(m_sWeldIniFilePath, out.str()))
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "写入配置库焊接工艺数据失败。";
        return false;
    }
    return true;
}

void WeldProcessFile::EnsureDefaultLayerRows()
{
    std::vector<T_WELD_PARA> extraRows;
    for (const auto& item : m_vtWeldParaList)
    {
        if (std::abs(item.dWeldAngleSize - 10.0) > 1e-6)
        {
            continue;
        }

        for (int layer = 1; layer <= 3; ++layer)
        {
            const bool exists = std::any_of(m_vtWeldParaList.begin(), m_vtWeldParaList.end(),
                [&](const T_WELD_PARA& row)
                {
                    return row.strWorkPeace == item.strWorkPeace
                        && std::abs(row.dWeldAngleSize - item.dWeldAngleSize) <= 1e-6
                        && row.nLayerNo == layer;
                });
            if (!exists)
            {
                T_WELD_PARA clone = item;
                clone.nLayerNo = layer;
                extraRows.push_back(clone);
            }
        }
    }

    for (const auto& row : extraRows)
    {
        m_vtWeldParaList.push_back(row);
    }
}

void WeldProcessFile::NormalizeWeldOrderKeepGroupOrder()
{
    T_WELD_PARA selected {};
    bool hasSelected = false;
    if (m_nUseWeldParaNo >= 0 && m_nUseWeldParaNo < static_cast<int>(m_vtWeldParaList.size()))
    {
        selected = m_vtWeldParaList[m_nUseWeldParaNo];
        hasSelected = true;
    }

    std::unordered_map<std::string, std::vector<T_WELD_PARA>> grouped;
    std::vector<std::string> order;
    for (const auto& item : m_vtWeldParaList)
    {
        const std::string key = BuildGroupKey(item);
        if (grouped.find(key) == grouped.end())
        {
            order.push_back(key);
        }
        grouped[key].push_back(item);
    }

    std::vector<T_WELD_PARA> normalized;
    for (const auto& key : order)
    {
        auto& rows = grouped[key];
        std::sort(rows.begin(), rows.end(), [](const T_WELD_PARA& a, const T_WELD_PARA& b)
            {
                return a.nLayerNo < b.nLayerNo;
            });
        for (int i = 0; i < static_cast<int>(rows.size()); ++i)
        {
            rows[i].nLayerNo = i + 1;
        }
        normalized.insert(normalized.end(), rows.begin(), rows.end());
    }
    m_vtWeldParaList = normalized;

    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    if (!hasSelected)
    {
        m_nUseWeldParaNo = 0;
        return;
    }

    for (int i = 0; i < static_cast<int>(m_vtWeldParaList.size()); ++i)
    {
        const auto& item = m_vtWeldParaList[i];
        if (item.strWorkPeace == selected.strWorkPeace
            && item.strWeldType == selected.strWeldType
            && std::abs(item.dWeldAngleSize - selected.dWeldAngleSize) <= 1e-6
            && item.nLayerNo == selected.nLayerNo)
        {
            m_nUseWeldParaNo = i;
            return;
        }
    }
    m_nUseWeldParaNo = 0;
}

std::string WeldProcessFile::BuildGroupKey(const T_WELD_PARA& item) const
{
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    out << item.strWorkPeace << "|" << item.dWeldAngleSize;
    return out.str();
}

bool WeldProcessFile::BindWeldToWeave()
{
    for (auto& weldPara : m_vtWeldParaList)
    {
        weldPara.nWeaveEnable = weldPara.nWeaveEnable != 0 ? 1 : 0;
        weldPara.nTrackEnable = weldPara.nTrackEnable != 0 ? 1 : 0;
        weldPara.nArcMode = NormalizeArcMode(weldPara.nArcMode);
        if (weldPara.nWeaveEnable == 0)
        {
            weldPara.tWeaveParam = {};
            continue;
        }
        const int weaveIndex = weldPara.nWeaveTypeNo;
        if (weaveIndex < 0 || weaveIndex >= static_cast<int>(m_vtWeaveTypeList.size()))
        {
            m_sLastError = "焊接工艺关联的摆动参数索引越界。";
            return false;
        }
        weldPara.tWeaveParam = m_vtWeaveTypeList[weaveIndex];
    }
    return true;
}

std::vector<std::string> WeldProcessFile::SplitLine(const std::string& line, char delimiter) const
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, delimiter))
    {
        fields.push_back(field);
    }
    return fields;
}

std::string WeldProcessFile::JoinLine(const std::vector<std::string>& fields, char delimiter) const
{
    std::ostringstream out;
    for (size_t i = 0; i < fields.size(); ++i)
    {
        if (i > 0)
        {
            out << delimiter;
        }
        out << fields[i];
    }
    return out.str();
}

bool WeldProcessFile::ParseWeaveLine(const std::vector<std::string>& fields, T_WeaveDate& tWeaveDate) const
{
    const int fieldCount = static_cast<int>(fields.size());
    if (fieldCount != kWeaveFieldCount)
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "摆动参数格式已升级，请重新创建工艺内容。";
        return false;
    }
    return TryParseInt(fields[0], tWeaveDate.nWeaveType)
        && TryParseInt(fields[1], tWeaveDate.nWeaveShape)
        && TryParseDouble(fields[2], tWeaveDate.dWeaveFrequencyHz)
        && TryParseDouble(fields[3], tWeaveDate.dWeaveAmplitudeMm)
        && TryParseDouble(fields[4], tWeaveDate.dSwingDirectionDeg)
        && TryParseDouble(fields[5], tWeaveDate.dWeavePlaneAngleDeg)
        && TryParseDouble(fields[6], tWeaveDate.dSpaceAngleDeg)
        && TryParseInt(fields[7], tWeaveDate.nPauseTime1Ms)
        && TryParseInt(fields[8], tWeaveDate.nPauseTime2Ms)
        && TryParseInt(fields[9], tWeaveDate.nPauseTime3Ms)
        && TryParseInt(fields[10], tWeaveDate.nPauseTime4Ms)
        && TryParseInt(fields[11], tWeaveDate.nPauseContinue)
        && TryParseDouble(fields[12], tWeaveDate.dEndLengthMm)
        && TryParseDouble(fields[13], tWeaveDate.dEndWidthMm)
        && TryParseDouble(fields[14], tWeaveDate.dCenterHeightMm);
}

bool WeldProcessFile::ParseWeldLine(const std::vector<std::string>& fields, T_WELD_PARA& tWeldPara) const
{
    if (static_cast<int>(fields.size()) != kWeldFieldCount)
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "焊接工艺参数格式已升级，请重新创建工艺内容。";
        return false;
    }

    tWeldPara.strWorkPeace = fields[0];
    tWeldPara.strWeldType = fields[1];
    bool ok = TryParseDouble(fields[2], tWeldPara.dWeldAngleSize)
        && TryParseInt(fields[3], tWeldPara.nLayerNo)
        && TryParseDouble(fields[4], tWeldPara.dStartArcCurrent)
        && TryParseDouble(fields[5], tWeldPara.dStartArcVoltage)
        && TryParseDouble(fields[6], tWeldPara.dStartWaitTime)
        && TryParseDouble(fields[7], tWeldPara.dTrackCurrent)
        && TryParseDouble(fields[8], tWeldPara.dTrackVoltage)
        && TryParseDouble(fields[9], tWeldPara.WeldVelocity)
        && TryParseDouble(fields[10], tWeldPara.dStopArcCurrent)
        && TryParseDouble(fields[11], tWeldPara.dStopArcVoltage)
        && TryParseDouble(fields[12], tWeldPara.dStopWaitTime)
        && TryParseDouble(fields[13], tWeldPara.dWrapCurrentt1)
        && TryParseDouble(fields[14], tWeldPara.dWrapVoltage1)
        && TryParseDouble(fields[15], tWeldPara.dWrapWaitTime1)
        && TryParseDouble(fields[16], tWeldPara.dWrapCurrentt2)
        && TryParseDouble(fields[17], tWeldPara.dWrapVoltage2)
        && TryParseDouble(fields[18], tWeldPara.dWrapWaitTime2)
        && TryParseDouble(fields[19], tWeldPara.dWrapCurrentt3)
        && TryParseDouble(fields[20], tWeldPara.dWrapVoltage3)
        && TryParseDouble(fields[21], tWeldPara.dWrapWaitTime3)
        && TryParseDouble(fields[22], tWeldPara.CrosswiseOffset)
        && TryParseDouble(fields[23], tWeldPara.verticalOffset)
        && TryParseInt(fields[24], tWeldPara.nWrapConditionNo)
        && TryParseDouble(fields[25], tWeldPara.dWeldAngle)
        && TryParseDouble(fields[26], tWeldPara.dWeldDipAngle)
        && TryParseInt(fields[27], tWeldPara.nStandWeldDir)
        && TryParseInt(fields[28], tWeldPara.nWeaveTypeNo)
        && TryParseInt(fields[29], tWeldPara.nWeldMethod);
    if (!ok)
    {
        return false;
    }

    const bool parsed = TryParseInt(fields[30], tWeldPara.nWrapCurrent1Enable)
        && TryParseInt(fields[31], tWeldPara.nWrapVoltage1Enable)
        && TryParseInt(fields[32], tWeldPara.nWrapWaitTime1Enable)
        && TryParseInt(fields[33], tWeldPara.nWrapCurrent2Enable)
        && TryParseInt(fields[34], tWeldPara.nWrapVoltage2Enable)
        && TryParseInt(fields[35], tWeldPara.nWrapWaitTime2Enable)
        && TryParseInt(fields[36], tWeldPara.nWrapCurrent3Enable)
        && TryParseInt(fields[37], tWeldPara.nWrapVoltage3Enable)
        && TryParseInt(fields[38], tWeldPara.nWrapWaitTime3Enable)
        && TryParseDouble(fields[39], tWeldPara.dCornerArcTransitionRadius)
        && TryParseDouble(fields[40], tWeldPara.dCornerArcTransitionSpeed)
        && TryParseDouble(fields[41], tWeldPara.dCornerArcTransitionCurrent)
        && TryParseDouble(fields[42], tWeldPara.dCornerArcTransitionVoltage)
        && TryParseInt(fields[43], tWeldPara.nCornerArcTransitionRadiusEnable)
        && TryParseInt(fields[44], tWeldPara.nCornerArcTransitionSpeedEnable)
        && TryParseInt(fields[45], tWeldPara.nCornerArcTransitionCurrentEnable)
        && TryParseInt(fields[46], tWeldPara.nCornerArcTransitionVoltageEnable)
        && TryParseInt(fields[47], tWeldPara.nCornerArcTransitionApplyScope)
        && TryParseInt(fields[48], tWeldPara.tTrackParam.nLateralBeginCycle)
        && TryParseDouble(fields[49], tWeldPara.tTrackParam.dLateralGain)
        && TryParseDouble(fields[50], tWeldPara.tTrackParam.dLeftAreaCoefficient)
        && TryParseDouble(fields[51], tWeldPara.tTrackParam.dRightAreaCoefficient)
        && TryParseInt(fields[52], tWeldPara.tTrackParam.nVerticalModeFlag)
        && TryParseDouble(fields[53], tWeldPara.tTrackParam.dVerticalReferenceCurrent)
        && TryParseInt(fields[54], tWeldPara.tTrackParam.nVerticalBeginCycle)
        && TryParseInt(fields[55], tWeldPara.tTrackParam.nVerticalSustainCycle)
        && TryParseDouble(fields[56], tWeldPara.tTrackParam.dVerticalCycleLength)
        && TryParseDouble(fields[57], tWeldPara.tTrackParam.dVerticalGain)
        && TryParseInt(fields[58], tWeldPara.tTrackParam.nTimeOrDistanceMode)
        && TryParseInt(fields[59], tWeldPara.tTrackParam.nTimeIntervalMs)
        && TryParseInt(fields[60], tWeldPara.tTrackParam.nDistanceIntervalMm)
        && TryParseDouble(fields[61], tWeldPara.tTrackParam.dLateralMinCompPerCycle)
        && TryParseDouble(fields[62], tWeldPara.tTrackParam.dLateralMaxCompPerCycle)
        && TryParseDouble(fields[63], tWeldPara.tTrackParam.dLateralMaxCompTotal)
        && TryParseDouble(fields[64], tWeldPara.tTrackParam.dLateralAsymmetryCoefficient)
        && TryParseDouble(fields[65], tWeldPara.tTrackParam.dLateralReserved6)
        && TryParseDouble(fields[66], tWeldPara.tTrackParam.dLateralReserved5)
        && TryParseDouble(fields[67], tWeldPara.tTrackParam.dLateralReserved4)
        && TryParseDouble(fields[68], tWeldPara.tTrackParam.dLateralReserved3)
        && TryParseDouble(fields[69], tWeldPara.tTrackParam.dLateralReserved2)
        && TryParseDouble(fields[70], tWeldPara.tTrackParam.dLateralReserved1)
        && TryParseDouble(fields[71], tWeldPara.tTrackParam.dVerticalMinCompPerCycle)
        && TryParseDouble(fields[72], tWeldPara.tTrackParam.dVerticalMaxCompPerCycle)
        && TryParseDouble(fields[73], tWeldPara.tTrackParam.dVerticalMaxCompTotal)
        && TryParseDouble(fields[74], tWeldPara.tTrackParam.dVerticalAsymmetryCoefficient)
        && TryParseDouble(fields[75], tWeldPara.tTrackParam.dVerticalReserved6)
        && TryParseDouble(fields[76], tWeldPara.tTrackParam.dVerticalReserved5)
        && TryParseDouble(fields[77], tWeldPara.tTrackParam.dVerticalReserved4)
        && TryParseDouble(fields[78], tWeldPara.tTrackParam.dVerticalReserved3)
        && TryParseDouble(fields[79], tWeldPara.tTrackParam.dVerticalReserved2)
        && TryParseDouble(fields[80], tWeldPara.tTrackParam.dVerticalReserved1)
        && TryParseInt(fields[81], tWeldPara.nWeaveEnable)
        && TryParseInt(fields[82], tWeldPara.nTrackEnable)
        && TryParseInt(fields[83], tWeldPara.nArcMode);
    if (parsed)
    {
        tWeldPara.nWeaveEnable = tWeldPara.nWeaveEnable != 0 ? 1 : 0;
        tWeldPara.nTrackEnable = tWeldPara.nTrackEnable != 0 ? 1 : 0;
        tWeldPara.nArcMode = NormalizeArcMode(tWeldPara.nArcMode);
    }
    return parsed;
}

std::vector<std::string> WeldProcessFile::BuildWeaveFields(const T_WeaveDate& tWeaveDate) const
{
    return {
        std::to_string(tWeaveDate.nWeaveType),
        std::to_string(tWeaveDate.nWeaveShape),
        ToText(tWeaveDate.dWeaveFrequencyHz),
        ToText(tWeaveDate.dWeaveAmplitudeMm),
        ToText(tWeaveDate.dSwingDirectionDeg),
        ToText(tWeaveDate.dWeavePlaneAngleDeg),
        ToText(tWeaveDate.dSpaceAngleDeg),
        std::to_string(tWeaveDate.nPauseTime1Ms),
        std::to_string(tWeaveDate.nPauseTime2Ms),
        std::to_string(tWeaveDate.nPauseTime3Ms),
        std::to_string(tWeaveDate.nPauseTime4Ms),
        std::to_string(tWeaveDate.nPauseContinue),
        ToText(tWeaveDate.dEndLengthMm),
        ToText(tWeaveDate.dEndWidthMm),
        ToText(tWeaveDate.dCenterHeightMm)
    };
}

std::vector<std::string> WeldProcessFile::BuildWeldFields(const T_WELD_PARA& tWeldPara) const
{
    return {
        tWeldPara.strWorkPeace,
        tWeldPara.strWeldType,
        ToText(tWeldPara.dWeldAngleSize),
        std::to_string(tWeldPara.nLayerNo),
        ToText(tWeldPara.dStartArcCurrent),
        ToText(tWeldPara.dStartArcVoltage),
        ToText(tWeldPara.dStartWaitTime),
        ToText(tWeldPara.dTrackCurrent),
        ToText(tWeldPara.dTrackVoltage),
        ToText(tWeldPara.WeldVelocity),
        ToText(tWeldPara.dStopArcCurrent),
        ToText(tWeldPara.dStopArcVoltage),
        ToText(tWeldPara.dStopWaitTime),
        ToText(tWeldPara.dWrapCurrentt1),
        ToText(tWeldPara.dWrapVoltage1),
        ToText(tWeldPara.dWrapWaitTime1),
        ToText(tWeldPara.dWrapCurrentt2),
        ToText(tWeldPara.dWrapVoltage2),
        ToText(tWeldPara.dWrapWaitTime2),
        ToText(tWeldPara.dWrapCurrentt3),
        ToText(tWeldPara.dWrapVoltage3),
        ToText(tWeldPara.dWrapWaitTime3),
        ToText(tWeldPara.CrosswiseOffset),
        ToText(tWeldPara.verticalOffset),
        std::to_string(tWeldPara.nWrapConditionNo),
        ToText(tWeldPara.dWeldAngle),
        ToText(tWeldPara.dWeldDipAngle),
        std::to_string(tWeldPara.nStandWeldDir),
        std::to_string(tWeldPara.nWeaveTypeNo),
        std::to_string(tWeldPara.nWeldMethod),
        std::to_string(tWeldPara.nWrapCurrent1Enable),
        std::to_string(tWeldPara.nWrapVoltage1Enable),
        std::to_string(tWeldPara.nWrapWaitTime1Enable),
        std::to_string(tWeldPara.nWrapCurrent2Enable),
        std::to_string(tWeldPara.nWrapVoltage2Enable),
        std::to_string(tWeldPara.nWrapWaitTime2Enable),
        std::to_string(tWeldPara.nWrapCurrent3Enable),
        std::to_string(tWeldPara.nWrapVoltage3Enable),
        std::to_string(tWeldPara.nWrapWaitTime3Enable),
        ToText(tWeldPara.dCornerArcTransitionRadius),
        ToText(tWeldPara.dCornerArcTransitionSpeed),
        ToText(tWeldPara.dCornerArcTransitionCurrent),
        ToText(tWeldPara.dCornerArcTransitionVoltage),
        std::to_string(tWeldPara.nCornerArcTransitionRadiusEnable),
        std::to_string(tWeldPara.nCornerArcTransitionSpeedEnable),
        std::to_string(tWeldPara.nCornerArcTransitionCurrentEnable),
        std::to_string(tWeldPara.nCornerArcTransitionVoltageEnable),
        std::to_string(tWeldPara.nCornerArcTransitionApplyScope),
        std::to_string(tWeldPara.tTrackParam.nLateralBeginCycle),
        ToText(tWeldPara.tTrackParam.dLateralGain),
        ToText(tWeldPara.tTrackParam.dLeftAreaCoefficient),
        ToText(tWeldPara.tTrackParam.dRightAreaCoefficient),
        std::to_string(tWeldPara.tTrackParam.nVerticalModeFlag),
        ToText(tWeldPara.tTrackParam.dVerticalReferenceCurrent),
        std::to_string(tWeldPara.tTrackParam.nVerticalBeginCycle),
        std::to_string(tWeldPara.tTrackParam.nVerticalSustainCycle),
        ToText(tWeldPara.tTrackParam.dVerticalCycleLength),
        ToText(tWeldPara.tTrackParam.dVerticalGain),
        std::to_string(tWeldPara.tTrackParam.nTimeOrDistanceMode),
        std::to_string(tWeldPara.tTrackParam.nTimeIntervalMs),
        std::to_string(tWeldPara.tTrackParam.nDistanceIntervalMm),
        ToText(tWeldPara.tTrackParam.dLateralMinCompPerCycle),
        ToText(tWeldPara.tTrackParam.dLateralMaxCompPerCycle),
        ToText(tWeldPara.tTrackParam.dLateralMaxCompTotal),
        ToText(tWeldPara.tTrackParam.dLateralAsymmetryCoefficient),
        ToText(tWeldPara.tTrackParam.dLateralReserved6),
        ToText(tWeldPara.tTrackParam.dLateralReserved5),
        ToText(tWeldPara.tTrackParam.dLateralReserved4),
        ToText(tWeldPara.tTrackParam.dLateralReserved3),
        ToText(tWeldPara.tTrackParam.dLateralReserved2),
        ToText(tWeldPara.tTrackParam.dLateralReserved1),
        ToText(tWeldPara.tTrackParam.dVerticalMinCompPerCycle),
        ToText(tWeldPara.tTrackParam.dVerticalMaxCompPerCycle),
        ToText(tWeldPara.tTrackParam.dVerticalMaxCompTotal),
        ToText(tWeldPara.tTrackParam.dVerticalAsymmetryCoefficient),
        ToText(tWeldPara.tTrackParam.dVerticalReserved6),
        ToText(tWeldPara.tTrackParam.dVerticalReserved5),
        ToText(tWeldPara.tTrackParam.dVerticalReserved4),
        ToText(tWeldPara.tTrackParam.dVerticalReserved3),
        ToText(tWeldPara.tTrackParam.dVerticalReserved2),
        ToText(tWeldPara.tTrackParam.dVerticalReserved1),
        std::to_string(tWeldPara.nWeaveEnable != 0 ? 1 : 0),
        std::to_string(tWeldPara.nTrackEnable != 0 ? 1 : 0),
        std::to_string(NormalizeArcMode(tWeldPara.nArcMode))
    };
}

std::string WeldProcessFile::ToText(double value) const
{
    std::ostringstream out;
    out << value;
    return out.str();
}

bool WeldProcessFile::TryParseInt(const std::string& text, int& value) const
{
    try
    {
        size_t pos = 0;
        value = std::stoi(text, &pos);
        return pos == text.size();
    }
    catch (...)
    {
        return false;
    }
}

bool WeldProcessFile::TryParseDouble(const std::string& text, double& value) const
{
    try
    {
        size_t pos = 0;
        value = std::stod(text, &pos);
        return pos == text.size();
    }
    catch (...)
    {
        return false;
    }
}

void WeldProcessFile::LogInfo(const char* format, ...) const
{
    va_list args;
    va_start(args, format);
    const int size = vsnprintf(nullptr, 0, format, args) + 1;
    va_end(args);

    if (size <= 0)
    {
        return;
    }

    std::vector<char> buffer(size);
    va_start(args, format);
    vsnprintf(buffer.data(), static_cast<size_t>(size), format, args);
    va_end(args);

    WeldProcessLogger().write(LogColor::SUCCESS, "%s", buffer.data());
}

void WeldProcessFile::LogError(const char* format, ...) const
{
    va_list args;
    va_start(args, format);
    const int size = vsnprintf(nullptr, 0, format, args) + 1;
    va_end(args);

    if (size <= 0)
    {
        return;
    }

    std::vector<char> buffer(size);
    va_start(args, format);
    vsnprintf(buffer.data(), static_cast<size_t>(size), format, args);
    va_end(args);

    WeldProcessLogger().write(LogColor::ERR, "%s", buffer.data());
}

void WeldProcessFile::ShowError(const std::string& message) const
{
    showErrorMessage("工艺文件", "%s", message.c_str());
}
