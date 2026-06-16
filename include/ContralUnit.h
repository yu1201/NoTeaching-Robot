#pragma once
// 本头只需要 T_CONTRAL_UNIT(Const.h) 与 std::vector；两套机器人驱动头(连带 KDL/socket/FTP)
// 此前在这里被灌进全部 14 个消费 TU，是仅次于 Const.h 的重编放大器——已下沉到 ContralUnit.cpp。
#include "Const.h"

#include <vector>


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
