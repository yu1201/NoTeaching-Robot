#include "LineCoarseScan.h"

#include "OPini.h"
#include "RobotLog.h"
#include "RobotMessage.h"

#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <vector>

namespace
{
RobotLog& LineCoarseScanLogger()
{
    static RobotLog logger(".//Log//LineCoarseScan.txt");
    return logger;
}

namespace fs = std::filesystem;

void ShowLineCoarseScanError(const std::string& message)
{
    showErrorMessage("线扫粗定位", "%s", message.c_str());
}
}

LineCoarseScan::LineCoarseScan(const T_CONTRAL_UNIT& tContralUnitInfo)
    : m_tContralUnitInfo(tContralUnitInfo)
{
    Init();
}

LineCoarseScan::LineCoarseScan(const ContralUnit& contralUnit)
{
    LoadParamByControlUnit(contralUnit);
}

LineCoarseScan::~LineCoarseScan()
{
}

bool LineCoarseScan::Init()
{
    return LoadParamByControlUnit(m_tContralUnitInfo);
}

bool LineCoarseScan::LoadParamByControlUnit(const ContralUnit& contralUnit, int nUnitIndex)
{
    if (nUnitIndex < 0 || nUnitIndex >= static_cast<int>(contralUnit.m_vtContralUnitInfo.size()))
    {
        m_sLastError = "控制单元索引越界，无法读取 LineCoarseScanParam.ini。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    return LoadParamByControlUnit(contralUnit.m_vtContralUnitInfo[nUnitIndex]);
}

bool LineCoarseScan::LoadParamByControlUnit(const T_CONTRAL_UNIT& tContralUnitInfo)
{
    m_sLastError.clear();
    m_tContralUnitInfo = tContralUnitInfo;
    m_nALLTableNum = 0;
    m_nCurUseTableNo = 0;

    if (tContralUnitInfo.sUnitName.empty())
    {
        m_sLastError = "控制单元名称为空，无法定位 LineCoarseScanParam.ini。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    m_sIniFilePath = BuildIniFilePath(tContralUnitInfo.sUnitName);
    if (m_sIniFilePath.empty())
    {
        m_sLastError = "未找到 LineCoarseScanParam.ini 文件。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    EnsureGlobalStorage(tContralUnitInfo.nUnitNo);

    COPini iniReader;
    iniReader.SetFileName(m_sIniFilePath);

    int nAllTableNum = 0;
    int nTableNo = 0;
    // 文件分为两段：料台总数、当前料台号，以及所有 Table 参数
    const bool bAllTableOk = ReadAllTableNum(iniReader, nAllTableNum);
    const bool bCurTableOk = ReadCurUseTableNo(iniReader, nTableNo);
    bool bTableOk = bAllTableOk;
    if (bAllTableOk)
    {
        m_vvtCoarseScanParam[tContralUnitInfo.nUnitNo].clear();
        for (int i = 0; i < nAllTableNum; ++i)
        {
            T_COARSE_SCAN_PARAM tTableParam;
            tTableParam.nTableNo = i;
            tTableParam.sUnitName = tContralUnitInfo.sUnitName;
            if (!ReadTableParam(iniReader, i, tTableParam))
            {
                bTableOk = false;
                break;
            }
            m_vvtCoarseScanParam[tContralUnitInfo.nUnitNo].push_back(tTableParam);
        }
    }

    if (!bAllTableOk || !bCurTableOk || !bTableOk)
    {
        if (m_sLastError.empty())
        {
            m_sLastError = "读取 LineCoarseScanParam.ini 参数失败。";
        }
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    if (nTableNo < 0 || nTableNo >= static_cast<int>(m_vvtCoarseScanParam[tContralUnitInfo.nUnitNo].size()))
    {
        m_sLastError = "CurUseTableNo 超出 ALLTableNum 范围。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    LogInfo("线扫粗定位参数读取成功，控制单元: %s, 料台总数: %d, 当前料台: Table%d",
        tContralUnitInfo.sUnitName.c_str(),
        m_nALLTableNum,
        m_nCurUseTableNo);
    return true;
}

const std::string& LineCoarseScan::GetIniFilePath() const
{
    return m_sIniFilePath;
}

std::string LineCoarseScan::GetLastError() const
{
    return m_sLastError;
}

const std::vector<std::vector<T_COARSE_SCAN_PARAM>>& LineCoarseScan::GetCoarseScanParam() const
{
    return m_vvtCoarseScanParam;
}

int LineCoarseScan::GetAllTableNum() const
{
    return m_nALLTableNum;
}

int LineCoarseScan::GetCurUseTableNo() const
{
    return m_nCurUseTableNo;
}

const T_COARSE_SCAN_PARAM* LineCoarseScan::GetCurTableParam() const
{
    if (m_tContralUnitInfo.nUnitNo < 0 ||
        m_tContralUnitInfo.nUnitNo >= static_cast<int>(m_vvtCoarseScanParam.size()) ||
        m_nCurUseTableNo < 0 ||
        m_nCurUseTableNo >= static_cast<int>(m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo].size()))
    {
        return nullptr;
    }

    return &m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo][m_nCurUseTableNo];
}

bool LineCoarseScan::UpdateTableParam(int nTableNo, const T_COARSE_SCAN_PARAM& tTableParam)
{
    if (m_sIniFilePath.empty())
    {
        m_sLastError = "参数文件路径为空，无法修改 Table 参数。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    if (!ValidateTableIndex(nTableNo))
    {
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    COPini iniWriter;
    iniWriter.SetFileName(m_sIniFilePath);
    if (!WriteTableParam(iniWriter, nTableNo, tTableParam))
    {
        if (m_sLastError.empty())
        {
            m_sLastError = "写入 Table 参数失败。";
        }
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    T_COARSE_SCAN_PARAM tNewParam = tTableParam;
    tNewParam.nTableNo = nTableNo;
    if (tNewParam.sUnitName.empty())
    {
        tNewParam.sUnitName = m_tContralUnitInfo.sUnitName;
    }
    m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo][nTableNo] = tNewParam;

    LogInfo("Table%d 参数修改成功。", nTableNo);
    return true;
}

bool LineCoarseScan::UpdateTableDoubleValue(int nTableNo, const std::string& key, double dValue)
{
    if (!ValidateTableIndex(nTableNo))
    {
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    T_COARSE_SCAN_PARAM tTableParam = m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo][nTableNo];
    bool bMatched = true;

    if (key == "YMaxCar") tTableParam.dYMaxCar = dValue;
    else if (key == "YMinCar") tTableParam.dYMinCar = dValue;
    else if (key == "YMaxRobot") tTableParam.dYMaxRobot = dValue;
    else if (key == "YMinRobot") tTableParam.dYMinRobot = dValue;
    else if (key == "XMax") tTableParam.dXMax = dValue;
    else if (key == "XMin") tTableParam.dXMin = dValue;
    else if (key == "ZMax") tTableParam.dZMax = dValue;
    else if (key == "ZMin") tTableParam.dZMin = dValue;
    else if (key == "ScanSpeed") tTableParam.dScanSpeed = dValue;
    else if (key == "RunSpeed") tTableParam.dRunSpeed = dValue;
    else if (key == "dAcc") tTableParam.dAcc = dValue;
    else if (key == "dDec") tTableParam.dDec = dValue;
    else if (key == "TableY") tTableParam.dTableY = dValue;
    else if (key == "TableZ") tTableParam.dTableZ = dValue;
    else if (key == "ScanStartCarLoction") tTableParam.dScanStartCarLoction = dValue;
    else if (key == "ScanEndtCarLoction") tTableParam.dScanEndtCarLoction = dValue;
    else if (key == "Scanlength") tTableParam.dScanLength = dValue;
    else if (key == "Range_XMax") tTableParam.dRangeXMax = dValue;
    else if (key == "Range_XMin") tTableParam.dRangeXMin = dValue;
    else if (key == "Range_YMax") tTableParam.dRangeYMax = dValue;
    else if (key == "Range_YMin") tTableParam.dRangeYMin = dValue;
    else if (key == "Range_ZMax") tTableParam.dRangeZMax = dValue;
    else if (key == "Range_ZMin") tTableParam.dRangeZMin = dValue;
    else bMatched = false;

    if (!bMatched)
    {
        m_sLastError = "不支持的 double 参数名: " + key;
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    return UpdateTableParam(nTableNo, tTableParam);
}

bool LineCoarseScan::UpdateTableIntValue(int nTableNo, const std::string& key, int nValue)
{
    if (!ValidateTableIndex(nTableNo))
    {
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    T_COARSE_SCAN_PARAM tTableParam = m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo][nTableNo];
    bool bMatched = true;

    if (key == "ScanDir") tTableParam.nTableScanDir = nValue;
    else if (key == "ExAxisEnable") tTableParam.nExAxisEnable = nValue;
    else if (key == "ImgStart_x") tTableParam.nTableImgStartX = nValue;
    else if (key == "ImgEnd_x") tTableParam.nTableImgEndX = nValue;
    else bMatched = false;

    if (!bMatched)
    {
        m_sLastError = "不支持的 int 参数名: " + key;
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    return UpdateTableParam(nTableNo, tTableParam);
}

bool LineCoarseScan::UpdateTablePulseValue(int nTableNo, const std::string& key, const T_ANGLE_PULSE& tPulse)
{
    if (!ValidateTableIndex(nTableNo))
    {
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    T_COARSE_SCAN_PARAM tTableParam = m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo][nTableNo];
    if (key == "StartPulse")
    {
        tTableParam.tStartPulse = tPulse;
    }
    else if (key == "EndPulse")
    {
        tTableParam.tEndPulse = tPulse;
    }
    else
    {
        m_sLastError = "不支持的脉冲参数名: " + key;
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    return UpdateTableParam(nTableNo, tTableParam);
}

bool LineCoarseScan::UpdateCurUseTableNo(int nTableNo)
{
    if (m_sIniFilePath.empty())
    {
        m_sLastError = "参数文件路径为空，无法修改 CurUseTableNo。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    if (nTableNo < 0 || nTableNo >= m_nALLTableNum)
    {
        m_sLastError = "CurUseTableNo 超出 ALLTableNum 范围，无法修改。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    COPini iniWriter;
    iniWriter.SetFileName(m_sIniFilePath);
    iniWriter.SetSectionName("CurUseTableNo");
    if (!iniWriter.WriteString("CurUseTableNo", nTableNo))
    {
        m_sLastError = "写入 CurUseTableNo 失败。";
        LogError("%s", m_sLastError.c_str());
        ShowLineCoarseScanError(m_sLastError);
        return false;
    }

    m_nCurUseTableNo = nTableNo;
    LogInfo("CurUseTableNo 修改成功，当前料台: Table%d", nTableNo);
    return true;
}

void LineCoarseScan::EnsureGlobalStorage(int nUnitNo)
{
    if (nUnitNo < 0)
    {
        return;
    }

    if (nUnitNo >= static_cast<int>(m_vvtCoarseScanParam.size()))
    {
        m_vvtCoarseScanParam.resize(static_cast<size_t>(nUnitNo + 1));
    }
}

std::string LineCoarseScan::BuildIniFilePath(const std::string& unitName) const
{
    const fs::path iniPath = fs::current_path() / "Data" / unitName / "LineCoarseScanParam.ini";
    if (fs::exists(iniPath))
    {
        return iniPath.string();
    }

    return "";
}

bool LineCoarseScan::ReadAllTableNum(COPini& iniReader, int& nTableNum)
{
    iniReader.SetSectionName("CurUseTableNo");
    if (iniReader.ReadString("ALLTableNum", &nTableNum) <= 0 || nTableNum <= 0)
    {
        m_sLastError = "读取 ALLTableNum 失败。";
        return false;
    }

    m_nALLTableNum = nTableNum;
    return true;
}

bool LineCoarseScan::ReadCurUseTableNo(COPini& iniReader, int& nTableNo)
{
    iniReader.SetSectionName("CurUseTableNo");
    if (iniReader.ReadString("CurUseTableNo", &nTableNo) <= 0 || nTableNo < 0)
    {
        m_sLastError = "读取 CurUseTableNo 失败。";
        return false;
    }

    m_nCurUseTableNo = nTableNo;
    return true;
}

bool LineCoarseScan::ReadTableParam(COPini& iniReader, int nTableNo, T_COARSE_SCAN_PARAM& tTableParam)
{
    // 根据 ALLTableNum 循环切换到 Table0 / Table1 ... 分组
    const std::string sectionName = GetStr("Table%d", nTableNo);
    iniReader.SetSectionName(sectionName);

    int bRtn = 1;
    bRtn = (bRtn && iniReader.ReadString("YMaxCar", &tTableParam.dYMaxCar) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("YMinCar", &tTableParam.dYMinCar) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("YMaxRobot", &tTableParam.dYMaxRobot) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("YMinRobot", &tTableParam.dYMinRobot) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("XMax", &tTableParam.dXMax) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("XMin", &tTableParam.dXMin) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ZMax", &tTableParam.dZMax) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ZMin", &tTableParam.dZMin) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ScanSpeed", &tTableParam.dScanSpeed) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("RunSpeed", &tTableParam.dRunSpeed) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("dAcc", &tTableParam.dAcc) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("dDec", &tTableParam.dDec) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("TableY", &tTableParam.dTableY) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("TableZ", &tTableParam.dTableZ) > 0) ? 1 : 0;
    bRtn = (bRtn && ReadPulse(iniReader, "StartPulse", tTableParam.tStartPulse)) ? 1 : 0;
    bRtn = (bRtn && ReadPulse(iniReader, "EndPulse", tTableParam.tEndPulse)) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ScanStartCarLoction", &tTableParam.dScanStartCarLoction) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ScanEndtCarLoction", &tTableParam.dScanEndtCarLoction) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Scanlength", &tTableParam.dScanLength) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ScanDir", &tTableParam.nTableScanDir) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ExAxisEnable", &tTableParam.nExAxisEnable) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Range_XMax", &tTableParam.dRangeXMax) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Range_XMin", &tTableParam.dRangeXMin) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Range_YMax", &tTableParam.dRangeYMax) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Range_YMin", &tTableParam.dRangeYMin) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Range_ZMax", &tTableParam.dRangeZMax) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("Range_ZMin", &tTableParam.dRangeZMin) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ImgStart_x", &tTableParam.nTableImgStartX) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString("ImgEnd_x", &tTableParam.nTableImgEndX) > 0) ? 1 : 0;

    if (bRtn == 0)
    {
        m_sLastError = "读取 [" + sectionName + "] 参数失败。";
        return false;
    }

    return true;
}

bool LineCoarseScan::ReadPulse(COPini& iniReader, const std::string& prefix, T_ANGLE_PULSE& tPulse)
{
    int bRtn = 1;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".nS", &tPulse.nSPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".nL", &tPulse.nLPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".nU", &tPulse.nUPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".nR", &tPulse.nRPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".nB", &tPulse.nBPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".nT", &tPulse.nTPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".lBX", &tPulse.lBXPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".lBY", &tPulse.lBYPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && iniReader.ReadString(prefix + ".lBZ", &tPulse.lBZPulse) > 0) ? 1 : 0;
    return (bRtn > 0);
}

bool LineCoarseScan::ValidateTableIndex(int nTableNo)
{
    if (m_tContralUnitInfo.nUnitNo < 0 ||
        m_tContralUnitInfo.nUnitNo >= static_cast<int>(m_vvtCoarseScanParam.size()) ||
        nTableNo < 0 ||
        nTableNo >= static_cast<int>(m_vvtCoarseScanParam[m_tContralUnitInfo.nUnitNo].size()))
    {
        m_sLastError = "Table 索引越界，无法修改参数。";
        return false;
    }

    return true;
}

bool LineCoarseScan::WriteTableParam(COPini& iniWriter, int nTableNo, const T_COARSE_SCAN_PARAM& tTableParam)
{
    iniWriter.SetSectionName(GetStr("Table%d", nTableNo));

    bool bRtn = true;
    bRtn = bRtn && iniWriter.WriteString("YMaxCar", tTableParam.dYMaxCar);
    bRtn = bRtn && iniWriter.WriteString("YMinCar", tTableParam.dYMinCar);
    bRtn = bRtn && iniWriter.WriteString("YMaxRobot", tTableParam.dYMaxRobot);
    bRtn = bRtn && iniWriter.WriteString("YMinRobot", tTableParam.dYMinRobot);
    bRtn = bRtn && iniWriter.WriteString("XMax", tTableParam.dXMax);
    bRtn = bRtn && iniWriter.WriteString("XMin", tTableParam.dXMin);
    bRtn = bRtn && iniWriter.WriteString("ZMax", tTableParam.dZMax);
    bRtn = bRtn && iniWriter.WriteString("ZMin", tTableParam.dZMin);
    bRtn = bRtn && iniWriter.WriteString("ScanSpeed", tTableParam.dScanSpeed);
    bRtn = bRtn && iniWriter.WriteString("RunSpeed", tTableParam.dRunSpeed);
    bRtn = bRtn && iniWriter.WriteString("dAcc", tTableParam.dAcc);
    bRtn = bRtn && iniWriter.WriteString("dDec", tTableParam.dDec);
    bRtn = bRtn && iniWriter.WriteString("TableY", tTableParam.dTableY);
    bRtn = bRtn && iniWriter.WriteString("TableZ", tTableParam.dTableZ);
    bRtn = bRtn && WritePulse(iniWriter, "StartPulse", tTableParam.tStartPulse);
    bRtn = bRtn && WritePulse(iniWriter, "EndPulse", tTableParam.tEndPulse);
    bRtn = bRtn && iniWriter.WriteString("ScanStartCarLoction", tTableParam.dScanStartCarLoction);
    bRtn = bRtn && iniWriter.WriteString("ScanEndtCarLoction", tTableParam.dScanEndtCarLoction);
    bRtn = bRtn && iniWriter.WriteString("Scanlength", tTableParam.dScanLength);
    bRtn = bRtn && iniWriter.WriteString("ScanDir", tTableParam.nTableScanDir);
    bRtn = bRtn && iniWriter.WriteString("ExAxisEnable", tTableParam.nExAxisEnable);
    bRtn = bRtn && iniWriter.WriteString("Range_XMax", tTableParam.dRangeXMax);
    bRtn = bRtn && iniWriter.WriteString("Range_XMin", tTableParam.dRangeXMin);
    bRtn = bRtn && iniWriter.WriteString("Range_YMax", tTableParam.dRangeYMax);
    bRtn = bRtn && iniWriter.WriteString("Range_YMin", tTableParam.dRangeYMin);
    bRtn = bRtn && iniWriter.WriteString("Range_ZMax", tTableParam.dRangeZMax);
    bRtn = bRtn && iniWriter.WriteString("Range_ZMin", tTableParam.dRangeZMin);
    bRtn = bRtn && iniWriter.WriteString("ImgStart_x", tTableParam.nTableImgStartX);
    bRtn = bRtn && iniWriter.WriteString("ImgEnd_x", tTableParam.nTableImgEndX);

    if (!bRtn)
    {
        m_sLastError = GetStr("写入 [Table%d] 参数失败。", nTableNo);
    }

    return bRtn;
}

bool LineCoarseScan::WritePulse(COPini& iniWriter, const std::string& prefix, const T_ANGLE_PULSE& tPulse)
{
    bool bRtn = true;
    bRtn = bRtn && iniWriter.WriteString(prefix + ".nS", static_cast<long>(tPulse.nSPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".nL", static_cast<long>(tPulse.nLPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".nU", static_cast<long>(tPulse.nUPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".nR", static_cast<long>(tPulse.nRPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".nB", static_cast<long>(tPulse.nBPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".nT", static_cast<long>(tPulse.nTPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".lBX", static_cast<long>(tPulse.lBXPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".lBY", static_cast<long>(tPulse.lBYPulse));
    bRtn = bRtn && iniWriter.WriteString(prefix + ".lBZ", static_cast<long>(tPulse.lBZPulse));
    return bRtn;
}

void LineCoarseScan::LogInfo(const char* format, ...) const
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

    LineCoarseScanLogger().write(LogColor::SUCCESS, "%s", buffer.data());
}

void LineCoarseScan::LogError(const char* format, ...) const
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

    LineCoarseScanLogger().write(LogColor::ERR, "%s", buffer.data());
}
