#pragma once

#include "Const.h"
#include "ContralUnit.h"

#include <string>

class COPini;

// 根据控制单元名称读取 LineCoarseScanParam.ini，并将参数写入类内缓存
class LineCoarseScan
{
public:
    explicit LineCoarseScan(const T_CONTRAL_UNIT& tContralUnitInfo);
    explicit LineCoarseScan(const ContralUnit& contralUnit);
    ~LineCoarseScan();

    bool Init();
    bool LoadParamByControlUnit(const T_CONTRAL_UNIT& tContralUnitInfo);
    bool LoadParamByControlUnit(const ContralUnit& contralUnit, int nUnitIndex = 0);

    const std::string& GetIniFilePath() const;
    std::string GetLastError() const;
    const std::vector<std::vector<T_COARSE_SCAN_PARAM>>& GetCoarseScanParam() const;
    int GetAllTableNum() const;
    int GetCurUseTableNo() const;
    const T_COARSE_SCAN_PARAM* GetCurTableParam() const;
    bool UpdateTableParam(int nTableNo, const T_COARSE_SCAN_PARAM& tTableParam);
    bool UpdateCurUseTableNo(int nTableNo);
    bool UpdateTableDoubleValue(int nTableNo, const std::string& key, double dValue);
    bool UpdateTableIntValue(int nTableNo, const std::string& key, int nValue);
    bool UpdateTablePulseValue(int nTableNo, const std::string& key, const T_ANGLE_PULSE& tPulse);

private:
    void EnsureGlobalStorage(int nUnitNo);
    // 生成参数文件路径：Data/<控制单元名>/LineCoarseScanParam.ini
    std::string BuildIniFilePath(const std::string& unitName) const;
    // 读取料台总数 [CurUseTableNo]/ALLTableNum
    bool ReadAllTableNum(COPini& iniReader, int& nTableNum);
    // 读取当前启用的料台编号 [CurUseTableNo]/CurUseTableNo
    bool ReadCurUseTableNo(COPini& iniReader, int& nTableNo);
    // 读取指定料台参数 [TableN]
    bool ReadTableParam(COPini& iniReader, int nTableNo, T_COARSE_SCAN_PARAM& tTableParam);
    // 读取起点/终点脉冲姿态
    bool ReadPulse(COPini& iniReader, const std::string& prefix, T_ANGLE_PULSE& tPulse);
    bool ValidateTableIndex(int nTableNo);
    bool WriteTableParam(COPini& iniWriter, int nTableNo, const T_COARSE_SCAN_PARAM& tTableParam);
    bool WritePulse(COPini& iniWriter, const std::string& prefix, const T_ANGLE_PULSE& tPulse);

    void LogInfo(const char* format, ...) const;
    void LogError(const char* format, ...) const;

private:
    T_CONTRAL_UNIT m_tContralUnitInfo;
    std::vector<std::vector<T_COARSE_SCAN_PARAM>> m_vvtCoarseScanParam;
    int m_nALLTableNum = 0;
    int m_nCurUseTableNo = 0;
    std::string m_sIniFilePath;
    std::string m_sLastError;
};
