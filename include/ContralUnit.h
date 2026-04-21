#pragma once
#include "FANUCRobotDriver.h"
#include "STEPRobotDriver.h"
#include "RobotDriverAdaptor.h"
#include "RobotLog.h"


class ContralUnit
{
public:
	ContralUnit();
	~ContralUnit();

	bool InitContralUnit();

	//T_ROBOT_COORS get

	std::vector<T_CONTRAL_UNIT>  m_vtContralUnitInfo;

private:

};
