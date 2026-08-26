#include "ContralUnit.h"
#include "Const.h"
#include "RobotDriverAdaptor.h"
#include "RobotDriverRegistry.h"
#include "RobotLog.h"

namespace
{
    void ReleaseRobotDrivers(std::vector<T_CONTRAL_UNIT>& units)
    {
        for (T_CONTRAL_UNIT& unit : units)
        {
            if (unit.pUnitDriver != nullptr)
            {
                RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unit.pUnitDriver);
                driver->StopStateMonitor();
                delete driver;
                unit.pUnitDriver = nullptr;
            }
        }
    }
}

ContralUnit::ContralUnit()
{
    InitContralUnit();
}

ContralUnit::~ContralUnit()
{
    ReleaseRobotDrivers(m_vtContralUnitInfo);
}

bool ContralUnit::InitContralUnit()
{
    bool bRtn = true;
    ConfigSection opini;
    RobotLog* ContralUnitLog = new RobotLog(".//Log//ContralUnit.txt");
    if (!opini.SetLocation(ConfigLocation::Global(QStringLiteral("ControlUnits"))))
    {
        ContralUnitLog->write(LogColor::ERR, "控制单元数据库位置无效");
        return false;
    }

    // 读取单元数量
    opini.SetSectionName("UnitNum");
    int nNum = 0;
    if (!opini.ReadString("UnitNum", &nNum) || nNum <= 0)
    {
        ContralUnitLog->write(LogColor::ERR, "读取UnitNum失败");
        return false;
    }

    ReleaseRobotDrivers(m_vtContralUnitInfo);
    m_vtContralUnitInfo.clear();

    // 循环读取每个单元
    for (int i = 0; i < nNum; i++)
    {
        T_CONTRAL_UNIT info;
        info.nUnitNo = i;
        info.pUnitDriver = nullptr;

        // 读取 UnitName
        opini.SetSectionName("UnitName");
        if (!opini.ReadString(GetStr("Unit%d", i), info.sUnitName))
        {
            ContralUnitLog->write(LogColor::ERR, "Unit%d 读取UnitName失败", i);
            bRtn = false;
        }

        // 读取 ChineseName
        opini.SetSectionName("ChineseName");
        if (!opini.ReadString(GetStr("Unit%d", i), info.sChineseName))
        {
            ContralUnitLog->write(LogColor::ERR, "Unit%d 读取ChineseName失败", i);
            bRtn = false;
        }

        // 读取 ContralType
        opini.SetSectionName("ContralType");
        if (!opini.ReadString(GetStr("Unit%d", i), info.sContralUnitType))
        {
            ContralUnitLog->write(LogColor::ERR, "Unit%d 读取ContralType失败", i);
            bRtn = false;
        }

        // 读取 UnitType
        opini.SetSectionName("UnitType");
        if (!opini.ReadString(GetStr("Unit%d", i), &info.nsUnitType))
        {
            ContralUnitLog->write(LogColor::ERR, "Unit%d 读取UnitType失败", i);
            bRtn = false;
        }

        // 机器人驱动
        if (info.sContralUnitType == "R")
        {
            std::string sLogPth = GetStr(".//Log//%sLog.txt", info.sUnitName.c_str());
            RobotLog* pRobotLog = new RobotLog(sLogPth);

            ConfigSection robotIni;
            int nRobotType = 0;
            if (!robotIni.SetLocation(ConfigLocation::Robot(
                    QString::fromUtf8(info.sUnitName.c_str()), QStringLiteral("RobotPara"))))
            {
                ContralUnitLog->write(LogColor::ERR, "机器人参数数据库位置无效：%s", info.sUnitName.c_str());
                bRtn = false;
            }
            else
            {
                robotIni.SetSectionName("BaseParam");
                if (!robotIni.ReadString("RobotType", &nRobotType))
                {
                    ContralUnitLog->write(LogColor::ERR, "%s 读取 RobotType 失败", info.sUnitName.c_str());
                    bRtn = false;
                }
                else
                {
                    std::string createError;
                    info.pUnitDriver = RobotDriverRegistry::Create(
                        nRobotType, info.sUnitName, pRobotLog, &createError);
                    if (info.pUnitDriver == nullptr)
                    {
                        ContralUnitLog->write(LogColor::WARNING,
                            "%s 的 RobotType=%d 未创建驱动：%s",
                            info.sUnitName.c_str(), nRobotType, createError.c_str());
                        delete pRobotLog;
                        pRobotLog = nullptr;
                        bRtn = false;
                    }
                    else
                    {
                        ContralUnitLog->write(LogColor::SUCCESS,
                            "通过注册表创建机器人底层成功: unit=%s type=%d brand=%s",
                            info.sUnitName.c_str(), nRobotType,
                            RobotDriverRegistry::DisplayName(nRobotType).c_str());
                    }
                }
            }
        }

        if (info.pUnitDriver != nullptr)
        {
            RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(info.pUnitDriver);
            if (!driver->StartStateMonitor(50))
            {
                ContralUnitLog->write(LogColor::WARNING, "%s 状态监控线程启动失败", info.sUnitName.c_str());
            }
        }

        m_vtContralUnitInfo.push_back(info);
    }

    ContralUnitLog->write(LogColor::SUCCESS, "InitContralUnit 初始化完成");
    return bRtn;
}
