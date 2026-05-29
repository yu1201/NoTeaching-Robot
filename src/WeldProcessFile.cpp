#include "WeldProcessFile.h"

#include "ConfigDatabase.h"
#include "RobotLog.h"
#include "RobotMessage.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
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

namespace fs = std::filesystem;
constexpr char kDelimiter = '\t';
constexpr int kWeaveFieldCount = 14;
constexpr int kWeldBaseFieldCount = 30;
constexpr int kWeldFieldCount = 48;
}

WeldProcessFile::WeldProcessFile(const T_CONTRAL_UNIT& tContralUnitInfo)
    : m_tContralUnitInfo(tContralUnitInfo)
{
    Init();
}

WeldProcessFile::WeldProcessFile(const ContralUnit& contralUnit, int nUnitIndex)
{
    LoadFromControlUnit(contralUnit, nUnitIndex);
}

WeldProcessFile::~WeldProcessFile()
{
}

bool WeldProcessFile::Init()
{
    return LoadFromControlUnit(m_tContralUnitInfo);
}

bool WeldProcessFile::LoadFromControlUnit(const ContralUnit& contralUnit, int nUnitIndex)
{
    if (nUnitIndex < 0 || nUnitIndex >= static_cast<int>(contralUnit.m_vtContralUnitInfo.size()))
    {
        m_sLastError = "控制单元索引越界，无法读取工艺文件。";
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
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
        ShowError(m_sLastError);
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
        ShowError(m_sLastError);
        return false;
    }

    if (!LoadWeaveTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    if (!LoadWeldTxt())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    if (!BindWeldToWeave())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    LogInfo("工艺 txt 读取成功，控制单元: %s, 摆动类型数: %d, 工艺数: %d",
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

    m_vtWeldParaList[nParaNo] = tWeldPara;
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
    m_vtWeldParaList.push_back(tWeldPara);
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
    return (fs::current_path() / "Data" / unitName / "WeaveDate.txt").string();
}

std::string WeldProcessFile::BuildWeldIniPath(const std::string& unitName) const
{
    return (fs::current_path() / "Data" / unitName / "WeldPara.txt").string();
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
                m_sLastError = "读取摆动 txt 的 USE 行失败。";
                return false;
            }
            continue;
        }

        T_WeaveDate weave {};
        if (!ParseWeaveLine(fields, weave))
        {
            m_sLastError = "摆动 txt 数据行格式错误。";
            return false;
        }
        m_vtWeaveTypeList.push_back(weave);
    }

    std::sort(m_vtWeaveTypeList.begin(), m_vtWeaveTypeList.end(), [](const T_WeaveDate& a, const T_WeaveDate& b)
        {
            return a.Type < b.Type;
        });

    m_nAllWeaveTypeNum = static_cast<int>(m_vtWeaveTypeList.size());
    if (m_nAllWeaveTypeNum <= 0)
    {
        m_sLastError = "摆动 txt 中没有有效数据。";
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
                m_sLastError = "读取焊接工艺 txt 的 USE 行失败。";
                return false;
            }
            continue;
        }

        T_WELD_PARA weld {};
        if (!ParseWeldLine(fields, weld))
        {
            m_sLastError = "焊接工艺 txt 数据行格式错误。";
            return false;
        }
        m_vtWeldParaList.push_back(weld);
    }

    NormalizeWeldOrderKeepGroupOrder();
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    if (m_nAllWeldParaNum <= 0)
    {
        m_sLastError = "焊接工艺 txt 中没有有效数据。";
        return false;
    }
    m_nUseWeldParaNo = qBound(0, m_nUseWeldParaNo, m_nAllWeldParaNum - 1);
    return true;
}

bool WeldProcessFile::SaveWeaveTxt() const
{
    std::ostringstream out;
    out << "# WeaveDate.txt\n";
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
    out << "# WeldPara.txt\n";
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
    if (static_cast<int>(fields.size()) != kWeaveFieldCount)
    {
        return false;
    }
    return TryParseInt(fields[0], tWeaveDate.Type)
        && TryParseDouble(fields[1], tWeaveDate.Freq)
        && TryParseDouble(fields[2], tWeaveDate.Amp_L)
        && TryParseDouble(fields[3], tWeaveDate.Amp_R)
        && TryParseInt(fields[4], tWeaveDate.StopTime_L)
        && TryParseInt(fields[5], tWeaveDate.StopTime_C)
        && TryParseInt(fields[6], tWeaveDate.StopTime_R)
        && TryParseDouble(fields[7], tWeaveDate.RotAngle_X)
        && TryParseDouble(fields[8], tWeaveDate.RotAngle_Z)
        && TryParseInt(fields[9], tWeaveDate.DelayType_L)
        && TryParseInt(fields[10], tWeaveDate.DelayType_C)
        && TryParseInt(fields[11], tWeaveDate.DelayType_R)
        && TryParseDouble(fields[12], tWeaveDate.RotAngle_L)
        && TryParseDouble(fields[13], tWeaveDate.RotAngle_R);
}

bool WeldProcessFile::ParseWeldLine(const std::vector<std::string>& fields, T_WELD_PARA& tWeldPara) const
{
    if (static_cast<int>(fields.size()) < kWeldBaseFieldCount
        || static_cast<int>(fields.size()) > kWeldFieldCount)
    {
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

    const bool oldFormat = static_cast<int>(fields.size()) == kWeldBaseFieldCount;
    const int defaultWrapEnable = oldFormat ? 1 : 0;
    auto parseOptionalInt = [&](int index, int& target, int defaultValue)
    {
        target = defaultValue;
        return index >= static_cast<int>(fields.size()) || TryParseInt(fields[index], target);
    };
    auto parseOptionalDouble = [&](int index, double& target, double defaultValue)
    {
        target = defaultValue;
        return index >= static_cast<int>(fields.size()) || TryParseDouble(fields[index], target);
    };

    return parseOptionalInt(30, tWeldPara.nWrapCurrent1Enable, defaultWrapEnable)
        && parseOptionalInt(31, tWeldPara.nWrapVoltage1Enable, defaultWrapEnable)
        && parseOptionalInt(32, tWeldPara.nWrapWaitTime1Enable, defaultWrapEnable)
        && parseOptionalInt(33, tWeldPara.nWrapCurrent2Enable, defaultWrapEnable)
        && parseOptionalInt(34, tWeldPara.nWrapVoltage2Enable, defaultWrapEnable)
        && parseOptionalInt(35, tWeldPara.nWrapWaitTime2Enable, defaultWrapEnable)
        && parseOptionalInt(36, tWeldPara.nWrapCurrent3Enable, defaultWrapEnable)
        && parseOptionalInt(37, tWeldPara.nWrapVoltage3Enable, defaultWrapEnable)
        && parseOptionalInt(38, tWeldPara.nWrapWaitTime3Enable, defaultWrapEnable)
        && parseOptionalDouble(39, tWeldPara.dCornerArcTransitionRadius, 0.0)
        && parseOptionalDouble(40, tWeldPara.dCornerArcTransitionSpeed, 0.0)
        && parseOptionalDouble(41, tWeldPara.dCornerArcTransitionCurrent, 0.0)
        && parseOptionalDouble(42, tWeldPara.dCornerArcTransitionVoltage, 0.0)
        && parseOptionalInt(43, tWeldPara.nCornerArcTransitionRadiusEnable, 0)
        && parseOptionalInt(44, tWeldPara.nCornerArcTransitionSpeedEnable, 0)
        && parseOptionalInt(45, tWeldPara.nCornerArcTransitionCurrentEnable, 0)
        && parseOptionalInt(46, tWeldPara.nCornerArcTransitionVoltageEnable, 0)
        && parseOptionalInt(47, tWeldPara.nCornerArcTransitionApplyScope, 2);
}

std::vector<std::string> WeldProcessFile::BuildWeaveFields(const T_WeaveDate& tWeaveDate) const
{
    return {
        std::to_string(tWeaveDate.Type),
        ToText(tWeaveDate.Freq),
        ToText(tWeaveDate.Amp_L),
        ToText(tWeaveDate.Amp_R),
        std::to_string(tWeaveDate.StopTime_L),
        std::to_string(tWeaveDate.StopTime_C),
        std::to_string(tWeaveDate.StopTime_R),
        ToText(tWeaveDate.RotAngle_X),
        ToText(tWeaveDate.RotAngle_Z),
        std::to_string(tWeaveDate.DelayType_L),
        std::to_string(tWeaveDate.DelayType_C),
        std::to_string(tWeaveDate.DelayType_R),
        ToText(tWeaveDate.RotAngle_L),
        ToText(tWeaveDate.RotAngle_R)
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
        std::to_string(tWeldPara.nCornerArcTransitionApplyScope)
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
