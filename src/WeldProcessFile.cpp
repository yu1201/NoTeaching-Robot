#include "WeldProcessFile.h"

#include "ConfigDatabase.h"
#include "RobotLog.h"
#include "RobotMessage.h"
#include "WeldProcessValidation.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <exception>
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

constexpr int kWeaveFieldCount = 15;
constexpr int kWeldCoreFieldCount = 48;
constexpr int kWeldTrackFieldCount = 33;
constexpr int kWeldSwitchFieldCount = 2;
constexpr int kWeldArcModeFieldCount = 1;
constexpr int kWeldLegacyFieldCount =
    kWeldCoreFieldCount + kWeldTrackFieldCount + kWeldSwitchFieldCount + kWeldArcModeFieldCount;
constexpr int kWeldExtraFieldCount = 2;
constexpr int kWeldFieldCount = kWeldLegacyFieldCount + kWeldExtraFieldCount;

int NormalizeArcMode(int mode)
{
    return mode >= 0 && mode <= 7 ? mode : 4;
}

// ===== 配置库多键格式（主数据） =====
// scope=robot, scopeId=单元名；module：WeldProcess(UseIndex/EntryCount) +
// WeldProcess/Entry<i>（每字段一键）；摆动为 WeaveData(同构)。
// 字段表用 visitor 一处定义、读写共用，避免读写不对称。
constexpr char kScopeRobot[] = "robot";
constexpr char kWeldModule[] = "WeldProcess";
constexpr char kWeaveModule[] = "WeaveData";
constexpr char kUseIndexKey[] = "UseIndex";
constexpr char kEntryCountKey[] = "EntryCount";

QString WeldEntryModule(int index)
{
    return QStringLiteral("WeldProcess/Entry%1").arg(index);
}

QString WeaveEntryModule(int index)
{
    return QStringLiteral("WeaveData/Entry%1").arg(index);
}

template <typename Visitor>
void VisitWeaveFields(T_WeaveDate& item, Visitor&& visit)
{
    visit("WeaveType", item.nWeaveType);
    visit("WeaveShape", item.nWeaveShape);
    visit("WeaveFrequencyHz", item.dWeaveFrequencyHz);
    visit("WeaveAmplitudeMm", item.dWeaveAmplitudeMm);
    visit("SwingDirectionDeg", item.dSwingDirectionDeg);
    visit("WeavePlaneAngleDeg", item.dWeavePlaneAngleDeg);
    visit("SpaceAngleDeg", item.dSpaceAngleDeg);
    visit("PauseTime1Ms", item.nPauseTime1Ms);
    visit("PauseTime2Ms", item.nPauseTime2Ms);
    visit("PauseTime3Ms", item.nPauseTime3Ms);
    visit("PauseTime4Ms", item.nPauseTime4Ms);
    visit("PauseContinue", item.nPauseContinue);
    visit("EndLengthMm", item.dEndLengthMm);
    visit("EndWidthMm", item.dEndWidthMm);
    visit("CenterHeightMm", item.dCenterHeightMm);
}

template <typename Visitor>
void VisitWeldParaFields(T_WELD_PARA& item, Visitor&& visit)
{
    visit("WorkPeace", item.strWorkPeace);
    visit("WeldType", item.strWeldType);
    visit("WeldAngleSize", item.dWeldAngleSize);
    visit("LayerNo", item.nLayerNo);
    visit("StartArcCurrent", item.dStartArcCurrent);
    visit("StartArcVoltage", item.dStartArcVoltage);
    visit("StartWaitTime", item.dStartWaitTime);
    visit("TrackCurrent", item.dTrackCurrent);
    visit("TrackVoltage", item.dTrackVoltage);
    visit("WeldVelocity", item.WeldVelocity);
    visit("StopArcCurrent", item.dStopArcCurrent);
    visit("StopArcVoltage", item.dStopArcVoltage);
    visit("StopWaitTime", item.dStopWaitTime);
    visit("WrapCurrent1", item.dWrapCurrentt1);
    visit("WrapVoltage1", item.dWrapVoltage1);
    visit("WrapWaitTime1", item.dWrapWaitTime1);
    visit("WrapCurrent2", item.dWrapCurrentt2);
    visit("WrapVoltage2", item.dWrapVoltage2);
    visit("WrapWaitTime2", item.dWrapWaitTime2);
    visit("WrapCurrent3", item.dWrapCurrentt3);
    visit("WrapVoltage3", item.dWrapVoltage3);
    visit("WrapWaitTime3", item.dWrapWaitTime3);
    visit("CrosswiseOffset", item.CrosswiseOffset);
    visit("VerticalOffset", item.verticalOffset);
    visit("WrapConditionNo", item.nWrapConditionNo);
    visit("WeldAngle", item.dWeldAngle);
    visit("WeldDipAngle", item.dWeldDipAngle);
    visit("StandWeldDir", item.nStandWeldDir);
    visit("WeaveTypeNo", item.nWeaveTypeNo);
    visit("WeldMethod", item.nWeldMethod);
    visit("WrapCurrent1Enable", item.nWrapCurrent1Enable);
    visit("WrapVoltage1Enable", item.nWrapVoltage1Enable);
    visit("WrapWaitTime1Enable", item.nWrapWaitTime1Enable);
    visit("WrapCurrent2Enable", item.nWrapCurrent2Enable);
    visit("WrapVoltage2Enable", item.nWrapVoltage2Enable);
    visit("WrapWaitTime2Enable", item.nWrapWaitTime2Enable);
    visit("WrapCurrent3Enable", item.nWrapCurrent3Enable);
    visit("WrapVoltage3Enable", item.nWrapVoltage3Enable);
    visit("WrapWaitTime3Enable", item.nWrapWaitTime3Enable);
    visit("CornerArcTransitionRadius", item.dCornerArcTransitionRadius);
    visit("CornerArcTransitionSpeed", item.dCornerArcTransitionSpeed);
    visit("CornerArcTransitionCurrent", item.dCornerArcTransitionCurrent);
    visit("CornerArcTransitionVoltage", item.dCornerArcTransitionVoltage);
    visit("CornerArcTransitionRadiusEnable", item.nCornerArcTransitionRadiusEnable);
    visit("CornerArcTransitionSpeedEnable", item.nCornerArcTransitionSpeedEnable);
    visit("CornerArcTransitionCurrentEnable", item.nCornerArcTransitionCurrentEnable);
    visit("CornerArcTransitionVoltageEnable", item.nCornerArcTransitionVoltageEnable);
    visit("CornerArcTransitionApplyScope", item.nCornerArcTransitionApplyScope);
    visit("WeldPostureType", item.nWeldPostureType);
    visit("WeldOverlapRel", item.dWeldOverlapRel);
    visit("Track.LateralBeginCycle", item.tTrackParam.nLateralBeginCycle);
    visit("Track.LateralGain", item.tTrackParam.dLateralGain);
    visit("Track.LeftAreaCoefficient", item.tTrackParam.dLeftAreaCoefficient);
    visit("Track.RightAreaCoefficient", item.tTrackParam.dRightAreaCoefficient);
    visit("Track.VerticalModeFlag", item.tTrackParam.nVerticalModeFlag);
    visit("Track.VerticalReferenceCurrent", item.tTrackParam.dVerticalReferenceCurrent);
    visit("Track.VerticalBeginCycle", item.tTrackParam.nVerticalBeginCycle);
    visit("Track.VerticalSustainCycle", item.tTrackParam.nVerticalSustainCycle);
    visit("Track.VerticalCycleLength", item.tTrackParam.dVerticalCycleLength);
    visit("Track.VerticalGain", item.tTrackParam.dVerticalGain);
    visit("Track.TimeOrDistanceMode", item.tTrackParam.nTimeOrDistanceMode);
    visit("Track.TimeIntervalMs", item.tTrackParam.nTimeIntervalMs);
    visit("Track.DistanceIntervalMm", item.tTrackParam.nDistanceIntervalMm);
    visit("Track.LateralMinCompPerCycle", item.tTrackParam.dLateralMinCompPerCycle);
    visit("Track.LateralMaxCompPerCycle", item.tTrackParam.dLateralMaxCompPerCycle);
    visit("Track.LateralMaxCompTotal", item.tTrackParam.dLateralMaxCompTotal);
    visit("Track.LateralAsymmetryCoefficient", item.tTrackParam.dLateralAsymmetryCoefficient);
    visit("Track.LateralReserved6", item.tTrackParam.dLateralReserved6);
    visit("Track.LateralReserved5", item.tTrackParam.dLateralReserved5);
    visit("Track.LateralReserved4", item.tTrackParam.dLateralReserved4);
    visit("Track.LateralReserved3", item.tTrackParam.dLateralReserved3);
    visit("Track.LateralReserved2", item.tTrackParam.dLateralReserved2);
    visit("Track.LateralReserved1", item.tTrackParam.dLateralReserved1);
    visit("Track.VerticalMinCompPerCycle", item.tTrackParam.dVerticalMinCompPerCycle);
    visit("Track.VerticalMaxCompPerCycle", item.tTrackParam.dVerticalMaxCompPerCycle);
    visit("Track.VerticalMaxCompTotal", item.tTrackParam.dVerticalMaxCompTotal);
    visit("Track.VerticalAsymmetryCoefficient", item.tTrackParam.dVerticalAsymmetryCoefficient);
    visit("Track.VerticalReserved6", item.tTrackParam.dVerticalReserved6);
    visit("Track.VerticalReserved5", item.tTrackParam.dVerticalReserved5);
    visit("Track.VerticalReserved4", item.tTrackParam.dVerticalReserved4);
    visit("Track.VerticalReserved3", item.tTrackParam.dVerticalReserved3);
    visit("Track.VerticalReserved2", item.tTrackParam.dVerticalReserved2);
    visit("Track.VerticalReserved1", item.tTrackParam.dVerticalReserved1);
    visit("WeaveEnable", item.nWeaveEnable);
    visit("TrackEnable", item.nTrackEnable);
    visit("ArcMode", item.nArcMode);
    // 多键格式独有字段（不进 84 字段文本镜像）：
    visit("FinalWeldTrajectoryStepMm", item.dFinalWeldTrajectoryStepMm);
    visit("WeldDirection", item.nWeldDirection);
}

// 写 visitor：把字段写成 scoped setting（double 用 'g',17 全精度往返）。
struct ScopedSettingsWriter
{
    QString scopeId;
    QString module;
    bool ok = true;

    void operator()(const char* key, int& value)
    {
        Write(key, QString::number(value), QStringLiteral("int"));
    }
    void operator()(const char* key, double& value)
    {
        Write(key, QString::number(value, 'g', 17), QStringLiteral("double"));
    }
    void operator()(const char* key, std::string& value)
    {
        Write(key, QString::fromUtf8(value.c_str()), QStringLiteral("string"));
    }
    void Write(const char* key, const QString& value, const QString& type)
    {
        ok = ConfigDatabase::WriteScopedSetting(
            QLatin1String(kScopeRobot), scopeId, module, QLatin1String(key), value, type) && ok;
    }
};

// 读 visitor：缺键或解析失败时保留字段默认值（向后兼容新字段）。
struct ScopedSettingsReader
{
    QString scopeId;
    QString module;
    bool ok = true;
    std::string error;

    void Fail(const char* key, const QString& text, const char* expectedType)
    {
        if (!ok)
        {
            return;
        }
        ok = false;
        error = QStringLiteral("配置字段 %1=%2 不是有效的%3。")
            .arg(QLatin1String(key), text, QString::fromLatin1(expectedType))
            .toUtf8().constData();
    }

    void operator()(const char* key, int& value)
    {
        QString text;
        if (ConfigDatabase::ReadScopedSetting(QLatin1String(kScopeRobot), scopeId, module, QLatin1String(key), &text))
        {
            bool parsed = false;
            const int parsedValue = text.trimmed().toInt(&parsed);
            if (parsed)
            {
                value = parsedValue;
            }
            else
            {
                Fail(key, text, "整数");
            }
        }
    }
    void operator()(const char* key, double& value)
    {
        QString text;
        if (ConfigDatabase::ReadScopedSetting(QLatin1String(kScopeRobot), scopeId, module, QLatin1String(key), &text))
        {
            bool parsed = false;
            const double parsedValue = text.trimmed().toDouble(&parsed);
            if (parsed)
            {
                value = parsedValue;
            }
            else
            {
                Fail(key, text, "浮点数");
            }
        }
    }
    void operator()(const char* key, std::string& value)
    {
        QString text;
        if (ConfigDatabase::ReadScopedSetting(QLatin1String(kScopeRobot), scopeId, module, QLatin1String(key), &text))
        {
            value = text.toUtf8().constData();
        }
    }
};

bool ReadScopedInt(const QString& scopeId, const char* module, const char* key, int& value)
{
    QString text;
    if (!ConfigDatabase::ReadScopedSetting(
        QLatin1String(kScopeRobot), scopeId, QLatin1String(module), QLatin1String(key), &text))
    {
        return false;
    }
    bool parsed = false;
    const int parsedValue = text.trimmed().toInt(&parsed);
    if (!parsed)
    {
        return false;
    }
    value = parsedValue;
    return true;
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

WeldProcessFile::WeldProcessFile(const std::string& unitName)
{
    // 仅按单元名定位工艺数据（供管线侧只有机器人名时复用本类，消除第二套解析器）。
    m_tContralUnitInfo.sUnitName = unitName;
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

    return true;
}

bool WeldProcessFile::LoadFromControlUnit(const ContralUnit& contralUnit, int nUnitIndex)
{
    if (nUnitIndex < 0 || nUnitIndex >= static_cast<int>(contralUnit.m_vtContralUnitInfo.size()))
    {
        m_sLastError = "控制单元索引越界，无法读取工艺配置。";
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
        m_sLastError = "控制单元名称为空，无法定位工艺配置。";
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    if (!TryLoadWeaveFromSettings() || !TryLoadWeldFromSettings())
    {
        if (m_sLastError.empty())
        {
            m_sLastError = "配置库中未找到完整的焊接工艺与摆动数据。";
        }
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    // 迁移/校正：把历史"所有工艺共享 WeaveData[0]"的数据展开成每工艺一份独立摆动（幂等）。
    // 两条加载路径末尾的 NormalizeWeldOrderKeepGroupOrder 已各调一次，这里再显式调一次以明确
    // BindWeldToWeave 依赖"nWeaveTypeNo=工艺下标、摆动列表与工艺一一平行"这个不变量。
    NormalizeWeaveTypeParallel();
    if (!BindWeldToWeave())
    {
        LogError("%s", m_sLastError.c_str());
        return false;
    }

    LogInfo("工艺数据读取成功(数据库多键)，控制单元: %s, 摆动类型数: %d, 工艺数: %d",
        tContralUnitInfo.sUnitName.c_str(), m_nAllWeaveTypeNum, m_nAllWeldParaNum);
    return true;
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
    if (!SaveWeaveToSettings())
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
    if (!SaveWeldToSettings())
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

    std::string validationError;
    if (!WeldProcessValidation::ValidateWeave(tWeaveDate, validationError))
    {
        m_sLastError = "摆动参数安全校验失败：" + validationError;
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }

    m_vtWeaveTypeList[nTypeNo] = tWeaveDate;
    if (!SaveWeaveToSettings())
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
    std::string validationError;
    if (!WeldProcessValidation::ValidateStoredWeldProcess(item, validationError))
    {
        m_sLastError = "焊接工艺安全校验失败：" + validationError;
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
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
    NormalizeWeldOrderKeepGroupOrder();  // 内部含 NormalizeWeaveTypeParallel：摆动随工艺同步重排
    if (!SaveWeaveToSettings() || !SaveWeldToSettings())
    {
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    return true;
}

bool WeldProcessFile::AddWeldPara(const T_WELD_PARA& tWeldPara, int& newIndex)
{
    if (m_tContralUnitInfo.sUnitName.empty())
    {
        if (!PrepareForRecreate())
        {
            return false;
        }
    }

    T_WELD_PARA item = tWeldPara;
    std::string validationError;
    if (!WeldProcessValidation::ValidateStoredWeldProcess(item, validationError))
    {
        m_sLastError = "焊接工艺安全校验失败：" + validationError;
        LogError("%s", m_sLastError.c_str());
        ShowError(m_sLastError);
        return false;
    }
    item.nWeaveEnable = item.nWeaveEnable != 0 ? 1 : 0;
    item.nTrackEnable = item.nTrackEnable != 0 ? 1 : 0;
    item.nArcMode = NormalizeArcMode(item.nArcMode);

    // 每个新工艺自带一份独立摆动数据（不再共享 WeaveData[0]）：先压入一份默认摆动，
    // 让新工艺指向它，随后 NormalizeWeaveTypeParallel 把 nWeaveTypeNo 重排成工艺下标、保持一一平行。
    m_vtWeaveTypeList.push_back(T_WeaveDate {});
    item.nWeaveTypeNo = static_cast<int>(m_vtWeaveTypeList.size()) - 1;
    item.tWeaveParam = item.nWeaveEnable != 0 ? m_vtWeaveTypeList[item.nWeaveTypeNo] : T_WeaveDate {};
    m_vtWeldParaList.push_back(item);
    NormalizeWeaveTypeParallel();
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    m_nUseWeldParaNo = static_cast<int>(m_vtWeldParaList.size()) - 1;
    newIndex = m_nUseWeldParaNo;
    if (!SaveWeaveToSettings() || !SaveWeldToSettings())
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

    if (!SaveWeaveToSettings() || !SaveWeldToSettings())
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

    // 分组重排后让摆动随工艺一起重排，保持每工艺一份独立摆动（一一平行）。
    NormalizeWeaveTypeParallel();
    if (!SaveWeaveToSettings() || !SaveWeldToSettings())
    {
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

void WeldProcessFile::NormalizeWeaveTypeParallel()
{
    // 不变量：每个工艺自带一份独立摆动数据，nWeaveTypeNo 恒等于工艺在列表中的下标，
    // m_vtWeaveTypeList 与 m_vtWeldParaList 一一平行。历史数据里所有工艺都把 nWeaveTypeNo 记成 0、
    // 共享同一份 WeaveData[0]，导致"改一个工艺的摆动其余工艺跟着变"。本函数按各工艺当前引用的那份
    // 摆动复制出独立副本（初值相同，之后各自独立），并把 nWeaveTypeNo 重排成工艺下标。幂等：已平行
    // 的数据再 normalize 结果不变；工艺增删/重排后调用即可保持平行（被删工艺那份摆动随之丢弃）。
    const size_t weldCount = m_vtWeldParaList.size();
    if (weldCount == 0)
    {
        if (m_vtWeaveTypeList.empty())
        {
            m_vtWeaveTypeList.push_back(T_WeaveDate {});  // 至少留一份模板供新建工艺取默认
        }
        m_nAllWeaveTypeNum = static_cast<int>(m_vtWeaveTypeList.size());
        m_nUseWeaveTypeNo = qBound(0, m_nUseWeaveTypeNo, m_nAllWeaveTypeNum - 1);
        return;
    }

    std::vector<T_WeaveDate> rebuilt;
    rebuilt.reserve(weldCount);
    for (size_t i = 0; i < weldCount; ++i)
    {
        const int ref = m_vtWeldParaList[i].nWeaveTypeNo;
        const T_WeaveDate weave = (ref >= 0 && ref < static_cast<int>(m_vtWeaveTypeList.size()))
            ? m_vtWeaveTypeList[ref]
            : T_WeaveDate {};
        rebuilt.push_back(weave);
        m_vtWeldParaList[i].nWeaveTypeNo = static_cast<int>(i);
        m_vtWeldParaList[i].tWeaveParam = m_vtWeldParaList[i].nWeaveEnable != 0 ? weave : T_WeaveDate {};
    }
    m_vtWeaveTypeList = std::move(rebuilt);
    m_nAllWeaveTypeNum = static_cast<int>(m_vtWeaveTypeList.size());
    m_nUseWeaveTypeNo = qBound(0, m_nUseWeldParaNo, m_nAllWeaveTypeNum - 1);
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
    // 工艺顺序定稿后，让摆动列表随工艺一起重排并保持每工艺一份独立（摆动跟着工艺走）。
    NormalizeWeaveTypeParallel();

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

bool WeldProcessFile::TryLoadWeaveFromSettings()
{
    const QString scopeId = QString::fromUtf8(m_tContralUnitInfo.sUnitName.c_str()).trimmed();
    if (scopeId.isEmpty())
    {
        return false;
    }
    int entryCount = 0;
    if (!ReadScopedInt(scopeId, kWeaveModule, kEntryCountKey, entryCount) || entryCount <= 0)
    {
        return false;
    }
    std::string entryCountError;
    if (!WeldProcessValidation::ValidateStoredEntryCount(entryCount, entryCountError))
    {
        m_sLastError = "摆动配置条目数安全校验失败：" + entryCountError;
        return false;
    }

    m_vtWeaveTypeList.clear();
    m_nUseWeaveTypeNo = 0;
    ReadScopedInt(scopeId, kWeaveModule, kUseIndexKey, m_nUseWeaveTypeNo);

    try
    {
        m_vtWeaveTypeList.reserve(static_cast<size_t>(entryCount));
    }
    catch (const std::exception& e)
    {
        m_sLastError = "摆动配置预分配失败：" + std::string(e.what());
        return false;
    }
    for (int index = 0; index < entryCount; ++index)
    {
        T_WeaveDate item {};
        ScopedSettingsReader reader { scopeId, WeaveEntryModule(index) };
        VisitWeaveFields(item, reader);
        if (!reader.ok)
        {
            m_sLastError = "读取摆动配置失败，Entry" + std::to_string(index)
                + "：" + reader.error;
            return false;
        }
        std::string validationError;
        if (!WeldProcessValidation::ValidateWeave(item, validationError))
        {
            m_sLastError = "摆动配置安全校验失败，Entry" + std::to_string(index)
                + "：" + validationError;
            return false;
        }
        m_vtWeaveTypeList.push_back(item);
    }

    m_nAllWeaveTypeNum = static_cast<int>(m_vtWeaveTypeList.size());
    m_nUseWeaveTypeNo = qBound(0, m_nUseWeaveTypeNo, m_nAllWeaveTypeNum - 1);
    return true;
}

bool WeldProcessFile::TryLoadWeldFromSettings()
{
    const QString scopeId = QString::fromUtf8(m_tContralUnitInfo.sUnitName.c_str()).trimmed();
    if (scopeId.isEmpty())
    {
        return false;
    }
    int entryCount = 0;
    if (!ReadScopedInt(scopeId, kWeldModule, kEntryCountKey, entryCount) || entryCount <= 0)
    {
        return false;
    }
    std::string entryCountError;
    if (!WeldProcessValidation::ValidateStoredEntryCount(entryCount, entryCountError))
    {
        m_sLastError = "焊接工艺条目数安全校验失败：" + entryCountError;
        return false;
    }

    m_vtWeldParaList.clear();
    m_nUseWeldParaNo = 0;
    ReadScopedInt(scopeId, kWeldModule, kUseIndexKey, m_nUseWeldParaNo);

    try
    {
        m_vtWeldParaList.reserve(static_cast<size_t>(entryCount));
    }
    catch (const std::exception& e)
    {
        m_sLastError = "焊接工艺预分配失败：" + std::string(e.what());
        return false;
    }
    for (int index = 0; index < entryCount; ++index)
    {
        T_WELD_PARA item {};
        ScopedSettingsReader reader { scopeId, WeldEntryModule(index) };
        VisitWeldParaFields(item, reader);
        if (!reader.ok)
        {
            m_sLastError = "读取焊接工艺配置失败，Entry" + std::to_string(index)
                + "：" + reader.error;
            return false;
        }
        std::string validationError;
        if (!WeldProcessValidation::ValidateStoredWeldProcess(item, validationError))
        {
            m_sLastError = "焊接工艺配置安全校验失败，Entry" + std::to_string(index)
                + "：" + validationError;
            return false;
        }
        item.nWeaveEnable = item.nWeaveEnable != 0 ? 1 : 0;
        item.nTrackEnable = item.nTrackEnable != 0 ? 1 : 0;
        item.nArcMode = NormalizeArcMode(item.nArcMode);
        m_vtWeldParaList.push_back(item);
    }

    NormalizeWeldOrderKeepGroupOrder();
    m_nAllWeldParaNum = static_cast<int>(m_vtWeldParaList.size());
    m_nUseWeldParaNo = qBound(0, m_nUseWeldParaNo, m_nAllWeldParaNum - 1);
    return true;
}

bool WeldProcessFile::SaveWeaveToSettings() const
{
    const QString scopeId = QString::fromUtf8(m_tContralUnitInfo.sUnitName.c_str()).trimmed();
    if (scopeId.isEmpty())
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "控制单元名称为空，无法写入配置库摆动数据。";
        return false;
    }
    for (size_t index = 0; index < m_vtWeaveTypeList.size(); ++index)
    {
        std::string validationError;
        if (!WeldProcessValidation::ValidateWeave(m_vtWeaveTypeList[index], validationError))
        {
            const_cast<WeldProcessFile*>(this)->m_sLastError =
                "拒绝写入非法摆动参数，Entry" + std::to_string(index) + "：" + validationError;
            return false;
        }
    }

    // 缩减时清理多余条目模块（整模块删除）。
    int previousCount = 0;
    ReadScopedInt(scopeId, kWeaveModule, kEntryCountKey, previousCount);
    const int count = static_cast<int>(m_vtWeaveTypeList.size());
    for (int index = count; index < previousCount; ++index)
    {
        ConfigDatabase::RemoveScopedSettings(QLatin1String(kScopeRobot), scopeId, WeaveEntryModule(index));
    }

    bool ok = ConfigDatabase::WriteScopedSetting(
        QLatin1String(kScopeRobot), scopeId, QLatin1String(kWeaveModule),
        QLatin1String(kEntryCountKey), QString::number(count), QStringLiteral("int"));
    ok = ConfigDatabase::WriteScopedSetting(
        QLatin1String(kScopeRobot), scopeId, QLatin1String(kWeaveModule),
        QLatin1String(kUseIndexKey), QString::number(m_nUseWeaveTypeNo), QStringLiteral("int")) && ok;
    for (int index = 0; index < count; ++index)
    {
        T_WeaveDate item = m_vtWeaveTypeList[static_cast<size_t>(index)];
        ScopedSettingsWriter writer { scopeId, WeaveEntryModule(index) };
        VisitWeaveFields(item, writer);
        ok = writer.ok && ok;
    }
    if (!ok)
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "写入配置库摆动数据(多键)失败。";
    }
    return ok;
}

bool WeldProcessFile::SaveWeldToSettings() const
{
    const QString scopeId = QString::fromUtf8(m_tContralUnitInfo.sUnitName.c_str()).trimmed();
    if (scopeId.isEmpty())
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "控制单元名称为空，无法写入配置库焊接工艺数据。";
        return false;
    }
    for (size_t index = 0; index < m_vtWeldParaList.size(); ++index)
    {
        std::string validationError;
        if (!WeldProcessValidation::ValidateStoredWeldProcess(
                m_vtWeldParaList[index], validationError))
        {
            const_cast<WeldProcessFile*>(this)->m_sLastError =
                "拒绝写入非法焊接工艺，Entry" + std::to_string(index) + "：" + validationError;
            return false;
        }
    }

    int previousCount = 0;
    ReadScopedInt(scopeId, kWeldModule, kEntryCountKey, previousCount);
    const int count = static_cast<int>(m_vtWeldParaList.size());
    for (int index = count; index < previousCount; ++index)
    {
        ConfigDatabase::RemoveScopedSettings(QLatin1String(kScopeRobot), scopeId, WeldEntryModule(index));
    }

    bool ok = ConfigDatabase::WriteScopedSetting(
        QLatin1String(kScopeRobot), scopeId, QLatin1String(kWeldModule),
        QLatin1String(kEntryCountKey), QString::number(count), QStringLiteral("int"));
    ok = ConfigDatabase::WriteScopedSetting(
        QLatin1String(kScopeRobot), scopeId, QLatin1String(kWeldModule),
        QLatin1String(kUseIndexKey), QString::number(m_nUseWeldParaNo), QStringLiteral("int")) && ok;
    for (int index = 0; index < count; ++index)
    {
        T_WELD_PARA item = m_vtWeldParaList[static_cast<size_t>(index)];
        ScopedSettingsWriter writer { scopeId, WeldEntryModule(index) };
        VisitWeldParaFields(item, writer);
        ok = writer.ok && ok;
    }
    if (!ok)
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError = "写入配置库焊接工艺数据(多键)失败。";
    }
    return ok;
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
    const bool parsed = TryParseInt(fields[0], tWeaveDate.nWeaveType)
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
    if (!parsed)
    {
        return false;
    }
    std::string validationError;
    if (!WeldProcessValidation::ValidateWeave(tWeaveDate, validationError))
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError =
            "摆动参数安全校验失败：" + validationError;
        return false;
    }
    return true;
}

bool WeldProcessFile::ParseWeldLine(const std::vector<std::string>& fields, T_WELD_PARA& tWeldPara) const
{
    const int fieldCount = static_cast<int>(fields.size());
    // 兼容旧 84 字段与新 86 字段(末尾姿态+圆滑)。旧文件读取时新字段用默认，下次保存补全到 86。
    if (fieldCount != kWeldFieldCount && fieldCount != kWeldLegacyFieldCount)
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
        // 扩展字段(86)：姿态 + 圆滑。旧 84 字段缺省→默认(可变/20)，下次保存自动补全。
        tWeldPara.nWeldPostureType = 1;
        tWeldPara.dWeldOverlapRel = 20.0;
        if (fieldCount >= kWeldFieldCount)
        {
            int posture = 1;
            double overlap = 20.0;
            if (!TryParseInt(fields[84], posture) || posture < 0 || posture > 3)
            {
                const_cast<WeldProcessFile*>(this)->m_sLastError =
                    "焊接工艺字段 WeldPostureType=" + fields[84] + " 无效，允许范围 [0,3]。";
                return false;
            }
            if (!TryParseDouble(fields[85], overlap))
            {
                const_cast<WeldProcessFile*>(this)->m_sLastError =
                    "焊接工艺字段 WeldOverlapRel=" + fields[85] + " 不是有效浮点数。";
                return false;
            }
            tWeldPara.nWeldPostureType = posture;
            tWeldPara.dWeldOverlapRel = overlap;
        }
    }
    if (!parsed)
    {
        return false;
    }
    std::string validationError;
    if (!WeldProcessValidation::ValidateStoredWeldProcess(tWeldPara, validationError))
    {
        const_cast<WeldProcessFile*>(this)->m_sLastError =
            "焊接工艺参数安全校验失败：" + validationError;
        return false;
    }
    tWeldPara.nWeaveEnable = tWeldPara.nWeaveEnable != 0 ? 1 : 0;
    tWeldPara.nTrackEnable = tWeldPara.nTrackEnable != 0 ? 1 : 0;
    tWeldPara.nArcMode = NormalizeArcMode(tWeldPara.nArcMode);
    return true;
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
        std::to_string(NormalizeArcMode(tWeldPara.nArcMode)),
        std::to_string(tWeldPara.nWeldPostureType),
        ToText(tWeldPara.dWeldOverlapRel)
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
