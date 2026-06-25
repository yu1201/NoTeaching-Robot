#pragma once

#include "Const.h"
#include "ContralUnit.h"

#include <string>
#include <unordered_map>
#include <vector>

// 配置库中的工艺参数读写层。
// 主数据为配置库多键格式（scope=robot，module=WeldProcess/Entry<i> 每字段一键），
// 旧的 84/15 字段 TAB 文本块（逻辑键 Data/<unit>/WeldPara.txt、WeaveDate.txt）作为
// 兼容镜像双写保留；读取时多键优先，仅有旧文本块时自动惰性迁移成多键。
// 读取后按“工件名 + 焊脚尺寸 + 层号”排序，让同组记录排在一起。
class WeldProcessFile
{
public:
    explicit WeldProcessFile(const T_CONTRAL_UNIT& tContralUnitInfo);
    explicit WeldProcessFile(const ContralUnit& contralUnit, int nUnitIndex = 0);
    explicit WeldProcessFile(const std::string& unitName);
    ~WeldProcessFile();

    bool Init();
    bool PrepareForRecreate();
    bool LoadFromControlUnit(const T_CONTRAL_UNIT& tContralUnitInfo);
    bool LoadFromControlUnit(const ContralUnit& contralUnit, int nUnitIndex = 0);

    const std::string& GetWeaveIniFilePath() const;
    const std::string& GetWeldIniFilePath() const;
    std::string GetLastError() const;

    const std::vector<T_WeaveDate>& GetWeaveTypeList() const;
    const std::vector<T_WELD_PARA>& GetWeldParaList() const;

    int GetAllWeaveTypeNum() const;
    int GetUseWeaveTypeNo() const;
    int GetAllWeldParaNum() const;
    int GetUseWeldParaNo() const;

    const T_WeaveDate* GetUseWeaveType() const;
    const T_WELD_PARA* GetUseWeldPara() const;
    bool UpdateUseWeaveTypeNo(int nUseWeaveTypeNo);
    bool UpdateUseWeldParaNo(int nUseWeldParaNo);
    bool UpdateWeaveType(int nTypeNo, const T_WeaveDate& tWeaveDate);
    bool UpdateWeldPara(int nParaNo, const T_WELD_PARA& tWeldPara);
    bool AddWeldPara(const T_WELD_PARA& tWeldPara, int& newIndex);
    bool RemoveWeldPara(int nParaNo);
    bool ReorderWeldGroups(const std::vector<std::string>& orderedGroupKeys);

private:
    void EnsureGlobalStorage(int nUnitNo);
    std::string BuildWeaveIniPath(const std::string& unitName) const;
    std::string BuildWeldIniPath(const std::string& unitName) const;

    bool LoadWeaveTxt();
    bool LoadWeldTxt();
    bool SaveWeaveTxt() const;
    bool SaveWeldTxt() const;
    // 配置库多键格式（主数据）：每个工艺/摆动条目的每个字段一键。
    bool TryLoadWeaveFromSettings();
    bool TryLoadWeldFromSettings();
    bool SaveWeaveToSettings() const;
    bool SaveWeldToSettings() const;
    void EnsureDefaultLayerRows();
    void NormalizeWeldOrderKeepGroupOrder();
    // 保证 m_vtWeaveTypeList 与 m_vtWeldParaList 一一平行：每个工艺自带一份独立摆动数据，
    // nWeaveTypeNo 恒等于工艺下标。修复历史"所有工艺共享 WeaveData[0]、改一个工艺摆动其余跟着变"的缺陷。
    void NormalizeWeaveTypeParallel();
    bool BindWeldToWeave();
    std::string BuildGroupKey(const T_WELD_PARA& item) const;

    std::vector<std::string> SplitLine(const std::string& line, char delimiter) const;
    std::string JoinLine(const std::vector<std::string>& fields, char delimiter) const;
    bool ParseWeaveLine(const std::vector<std::string>& fields, T_WeaveDate& tWeaveDate) const;
    bool ParseWeldLine(const std::vector<std::string>& fields, T_WELD_PARA& tWeldPara) const;
    std::vector<std::string> BuildWeaveFields(const T_WeaveDate& tWeaveDate) const;
    std::vector<std::string> BuildWeldFields(const T_WELD_PARA& tWeldPara) const;
    std::string ToText(double value) const;
    bool TryParseInt(const std::string& text, int& value) const;
    bool TryParseDouble(const std::string& text, double& value) const;

    void LogInfo(const char* format, ...) const;
    void LogError(const char* format, ...) const;
    void ShowError(const std::string& message) const;

private:
    T_CONTRAL_UNIT m_tContralUnitInfo;
    std::string m_sWeaveIniFilePath;
    std::string m_sWeldIniFilePath;
    std::string m_sLastError;

    int m_nAllWeaveTypeNum = 0;
    int m_nUseWeaveTypeNo = 0;
    int m_nAllWeldParaNum = 0;
    int m_nUseWeldParaNo = 0;

    std::vector<T_WeaveDate> m_vtWeaveTypeList;
    std::vector<T_WELD_PARA> m_vtWeldParaList;
};
