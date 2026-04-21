#include "WeldProcessFile.h"

#include "OPini.h"
#include "RobotLog.h"
#include "RobotMessage.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
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
constexpr int kWeldFieldCount = 30;
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

    if (!fs::exists(m_sWeaveIniFilePath) || !fs::exists(m_sWeldIniFilePath))
    {
        if (!TryMigrateFromIni())
        {
            LogError("%s", m_sLastError.c_str());
            ShowError(m_sLastError);
            return false;
        }
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
    std::ifstream in(m_sWeaveIniFilePath);
    if (!in)
    {
        m_sLastError = "打开摆动 txt 文件失败。";
        return false;
    }

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
    std::ifstream in(m_sWeldIniFilePath);
    if (!in)
    {
        m_sLastError = "打开焊接工艺 txt 文件失败。";
        return false;
    }

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
    std::ofstream out(m_sWeaveIniFilePath, std::ios::trunc);
    if (!out)
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "写入摆动 txt 文件失败。";
        return false;
    }

    out << "# WeaveDate.txt\n";
    out << "USE" << kDelimiter << m_nUseWeaveTypeNo << "\n";
    for (const auto& item : m_vtWeaveTypeList)
    {
        out << JoinLine(BuildWeaveFields(item), kDelimiter) << "\n";
    }
    return true;
}

bool WeldProcessFile::SaveWeldTxt() const
{
    std::ofstream out(m_sWeldIniFilePath, std::ios::trunc);
    if (!out)
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "写入焊接工艺 txt 文件失败。";
        return false;
    }

    out << "# WeldPara.txt\n";
    out << "USE" << kDelimiter << m_nUseWeldParaNo << "\n";
    for (const auto& item : m_vtWeldParaList)
    {
        out << JoinLine(BuildWeldFields(item), kDelimiter) << "\n";
    }
    return true;
}

bool WeldProcessFile::TryMigrateFromIni()
{
    const fs::path txtDir = fs::path(m_sWeldIniFilePath).parent_path();
    fs::create_directories(txtDir);

    const fs::path oldWeaveIni = txtDir / "WeaveDate.ini";
    const fs::path oldWeldIni = txtDir / "WeldPara.ini";

    if (!fs::exists(oldWeaveIni) || !fs::exists(oldWeldIni))
    {
        m_sLastError = "未找到 txt 文件，也未找到可迁移的 ini 文件。";
        return false;
    }

    if (!ImportWeaveFromIni(oldWeaveIni.string()))
    {
        return false;
    }
    if (!ImportWeldFromIni(oldWeldIni.string()))
    {
        return false;
    }
    EnsureDefaultLayerRows();
    NormalizeWeldOrderKeepGroupOrder();

    if (!SaveWeaveTxt() || !SaveWeldTxt())
    {
        return false;
    }
    return true;
}

bool WeldProcessFile::ImportWeaveFromIni(const std::string& iniPath)
{
    COPini reader;
    reader.SetFileName(iniPath);
    reader.SetSectionName("ALLWeaveType");

    if (reader.ReadString("ALLWeaveTypeNum", &m_nAllWeaveTypeNum) <= 0 || m_nAllWeaveTypeNum <= 0)
    {
        m_sLastError = "迁移 ini 时读取 ALLWeaveTypeNum 失败。";
        return false;
    }
    if (reader.ReadString("UseWeaveTypeNo", &m_nUseWeaveTypeNo) <= 0)
    {
        m_nUseWeaveTypeNo = 0;
    }

    m_vtWeaveTypeList.clear();
    for (int i = 0; i < m_nAllWeaveTypeNum; ++i)
    {
        T_WeaveDate item {};
        const std::string sectionName = GetStr("WeaveType%d", i);
        reader.SetSectionName(sectionName);
        if (reader.ReadString("Type", &item.Type) <= 0) return false;
        if (reader.ReadString("Freq", &item.Freq) <= 0) return false;
        if (reader.ReadString("Amp_L", &item.Amp_L) <= 0) return false;
        if (reader.ReadString("Amp_R", &item.Amp_R) <= 0) return false;
        if (reader.ReadString("StopTime_L", &item.StopTime_L) <= 0) return false;
        if (reader.ReadString("StopTime_C", &item.StopTime_C) <= 0) return false;
        if (reader.ReadString("StopTime_R", &item.StopTime_R) <= 0) return false;
        if (reader.ReadString("RotAngle_X", &item.RotAngle_X) <= 0) return false;
        if (reader.ReadString("RotAngle_Z", &item.RotAngle_Z) <= 0) return false;
        if (reader.ReadString("DelayType_L", &item.DelayType_L) <= 0) return false;
        if (reader.ReadString("DelayType_C", &item.DelayType_C) <= 0) return false;
        if (reader.ReadString("DelayType_R", &item.DelayType_R) <= 0) return false;
        if (reader.ReadString("RotAngle_L", &item.RotAngle_L) <= 0) return false;
        if (reader.ReadString("RotAngle_R", &item.RotAngle_R) <= 0) return false;
        m_vtWeaveTypeList.push_back(item);
    }
    std::sort(m_vtWeaveTypeList.begin(), m_vtWeaveTypeList.end(), [](const T_WeaveDate& a, const T_WeaveDate& b)
        {
            return a.Type < b.Type;
        });
    m_nAllWeaveTypeNum = static_cast<int>(m_vtWeaveTypeList.size());
    m_nUseWeaveTypeNo = qBound(0, m_nUseWeaveTypeNo, qMax(0, m_nAllWeaveTypeNum - 1));
    return true;
}

bool WeldProcessFile::ImportWeldFromIni(const std::string& iniPath)
{
    COPini reader;
    reader.SetFileName(iniPath);
    reader.SetSectionName("ALLWeldPara");

    if (reader.ReadString("ALLWeldParaNum", &m_nAllWeldParaNum) <= 0 || m_nAllWeldParaNum <= 0)
    {
        m_sLastError = "迁移 ini 时读取 ALLWeldParaNum 失败。";
        return false;
    }
    if (reader.ReadString("UseWeldParaNo", &m_nUseWeldParaNo) <= 0)
    {
        m_nUseWeldParaNo = 0;
    }

    m_vtWeldParaList.clear();
    for (int i = 0; i < m_nAllWeldParaNum; ++i)
    {
        T_WELD_PARA item {};
        const std::string sectionName = GetStr("WeldPara%d", i);
        reader.SetSectionName(sectionName);
        if (reader.ReadString("strWorkPeace", item.strWorkPeace) <= 0) return false;
        if (reader.ReadString("strWeldType", item.strWeldType) <= 0) return false;
        if (reader.ReadString("dWeldAngleSize", &item.dWeldAngleSize) <= 0) return false;
        if (reader.ReadString("nLayerNo", &item.nLayerNo) <= 0) return false;
        if (reader.ReadString("dStartArcCurrent", &item.dStartArcCurrent) <= 0) return false;
        if (reader.ReadString("dStartArcVoltage", &item.dStartArcVoltage) <= 0) return false;
        if (reader.ReadString("dStartWaitTime", &item.dStartWaitTime) <= 0) return false;
        if (reader.ReadString("dTrackCurrent", &item.dTrackCurrent) <= 0) return false;
        if (reader.ReadString("dTrackVoltage", &item.dTrackVoltage) <= 0) return false;
        if (reader.ReadString("WeldVelocity", &item.WeldVelocity) <= 0) return false;
        if (reader.ReadString("dStopArcCurrent", &item.dStopArcCurrent) <= 0) return false;
        if (reader.ReadString("dStopArcVoltage", &item.dStopArcVoltage) <= 0) return false;
        if (reader.ReadString("dStopWaitTime", &item.dStopWaitTime) <= 0) return false;
        if (reader.ReadString("dWrapCurrentt1", &item.dWrapCurrentt1) <= 0) return false;
        if (reader.ReadString("dWrapVoltage1", &item.dWrapVoltage1) <= 0) return false;
        if (reader.ReadString("dWrapWaitTime1", &item.dWrapWaitTime1) <= 0) return false;
        if (reader.ReadString("dWrapCurrentt2", &item.dWrapCurrentt2) <= 0) return false;
        if (reader.ReadString("dWrapVoltage2", &item.dWrapVoltage2) <= 0) return false;
        if (reader.ReadString("dWrapWaitTime2", &item.dWrapWaitTime2) <= 0) return false;
        if (reader.ReadString("dWrapCurrentt3", &item.dWrapCurrentt3) <= 0) return false;
        if (reader.ReadString("dWrapVoltage3", &item.dWrapVoltage3) <= 0) return false;
        if (reader.ReadString("dWrapWaitTime3", &item.dWrapWaitTime3) <= 0) return false;
        if (reader.ReadString("CrosswiseOffset", &item.CrosswiseOffset) <= 0) return false;
        if (reader.ReadString("verticalOffset", &item.verticalOffset) <= 0) return false;
        if (reader.ReadString("nWrapConditionNo", &item.nWrapConditionNo) <= 0) return false;
        if (reader.ReadString("dWeldAngle", &item.dWeldAngle) <= 0) return false;
        if (reader.ReadString("dWeldDipAngle", &item.dWeldDipAngle) <= 0) return false;
        if (reader.ReadString("nStandWeldDir", &item.nStandWeldDir) <= 0) return false;
        if (reader.ReadString("WeaveTypeNo", &item.nWeaveTypeNo) <= 0) return false;
        if (reader.ReadString("nWeldMethod", &item.nWeldMethod) <= 0) return false;
        m_vtWeldParaList.push_back(item);
    }
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    m_nUseWeldParaNo = qBound(0, m_nUseWeldParaNo, qMax(0, m_nAllWeldParaNum - 1));
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
    if (static_cast<int>(fields.size()) != kWeldFieldCount)
    {
        return false;
    }

    tWeldPara.strWorkPeace = fields[0];
    tWeldPara.strWeldType = fields[1];
    return TryParseDouble(fields[2], tWeldPara.dWeldAngleSize)
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
        std::to_string(tWeldPara.nWeldMethod)
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
