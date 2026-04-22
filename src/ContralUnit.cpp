#include "ContralUnit.h"
#include "Const.h"

ContralUnit::ContralUnit()
{
    InitContralUnit();
}

ContralUnit::~ContralUnit()
{
}

bool ContralUnit::InitContralUnit()
{
    bool bRtn = true;
    COPini opini;
    RobotLog* ContralUnitLog = new RobotLog(".//Log//ContralUnit.txt");
    // 打开INI文件
    if (!opini.SetFileName(CONTRAL_UNIT_INFO_INI))
    {
        ContralUnitLog->write(LogColor::ERR, "读取配置文件失败：%s", CONTRAL_UNIT_INFO_INI);
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

        // 创建目录
        std::string path = DATA_PATH + info.sUnitName;
        opini.CheckAndCreateDir(path);

        // 机器人驱动
        if (info.sContralUnitType == "R")
        {
            std::string sLogPth = GetStr(".//Log//%sLog.txt", info.sUnitName.c_str());
            RobotLog* pRobotLog = new RobotLog(sLogPth);

            COPini robotIni;
            const std::string robotIniPath = DATA_PATH + info.sUnitName + ROBOT_PARA_INI;
            int nRobotType = 0;
            if (!robotIni.SetFileName(robotIniPath))
            {
                ContralUnitLog->write(LogColor::ERR, "读取机器人参数文件失败：%s", robotIniPath.c_str());
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
                else if (nRobotType == ROBOT_TYPE_STEP)
                {
                    info.pUnitDriver = new STEPRobotCtrl(info.sUnitName, pRobotLog);
                    ContralUnitLog->write(LogColor::SUCCESS, "创建新时达驱动成功: %s", info.sUnitName.c_str());
                }
                else if (nRobotType == ROBOT_TYPE_FANUC)
                {
                    info.pUnitDriver = new FANUCRobotCtrl(info.sUnitName, pRobotLog);
                    ContralUnitLog->write(LogColor::SUCCESS, "创建FANUC驱动成功: %s", info.sUnitName.c_str());
                }
                else
                {
                    ContralUnitLog->write(LogColor::WARNING,
                        "%s 的 RobotType=%d 暂未支持，当前未创建驱动",
                        info.sUnitName.c_str(), nRobotType);
                    bRtn = false;
                }
            }
        }

        m_vtContralUnitInfo.push_back(info);
    }

    ContralUnitLog->write(LogColor::SUCCESS, "InitContralUnit 初始化完成");
    return bRtn;
}
