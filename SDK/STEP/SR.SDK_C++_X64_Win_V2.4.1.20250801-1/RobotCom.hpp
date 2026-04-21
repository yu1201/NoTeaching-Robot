#pragma once
#include <vector>
#include <chrono>
#include "ControlMsgData.hpp"
#include "EasyTcpClient.hpp"
//#include "profiler.h"

#ifndef _WIN32
#define  __declspec(dllexport)
#include <math.h>
#endif

#include <memory>  // ???????????? std::unique_ptr
#include <atomic>  // ???????????? std::atomic
constexpr bool IsDefaultBlocked = false;

namespace STEPROBOTSDK{
class __declspec(dllexport) RobotComClient {
public:
  RobotComClient();
  ~RobotComClient();

  /* init socket client */
  int init(const char *ip, u_short port);

  /* close com port */
  int close();

  /* unit test */
  int test(const char*);

  std::string getInfo() const;

  void setInterfaceWaitTime(int wait_ms);

#pragma region /* Get sdk struct  */
  const PlatformToSDKStruct *getSDKStruct();
  std::string getTeachRefSysNam();
  std::string getProjectName();
  std::string getProgramName();
  std::string getToolName();
  MessageData  getMessageData();
  RobotDI getRobotDI();
  RobotDO getRobotDO();
  RobotAI getRobotAI();
  RobotAO getRobotAO();
  CartDynState getActCartDynState();
  OPERATIONMODE getOperationMode();
  int getMotorEnableState();
  REFSYS getJogRefSys();
  double getOverride();
  PROGRAMSTATE getProgramState();
  PROGRAMMODE getProgramMode();
  int getCurrentLine();

  //CARTPOS getCartPos();//替换成以下三个，这个不再使用
  //增加mode，20230427
  RobotCartPos getCartPosWorld();
  RobotCartPos getCartPosRobotBase();
  RobotCartPos getCartPosUserRef();
  HomoMatrix getCartPosR();

  AXISPOS getAxisPos();
  unsigned int getRobotRunningTime();
  int getLifeTime();

  //新增获取焊接状态接口，20231215
  ARCWELDINGMODE getWeldingMode();

#pragma endregion

  /**************  RDK - FUNCTION  ******************/
  std::string SDKVersion();
  int ConnectStatus();

#pragma region SDK-CMD
  /* CMD Key */
  int SDKKeyMarkCmd();
  /* 4.4.2 request robotinfo */
  int RequestRobotInfoCmd(RobotInfoStruct &robotinfo);
  /* 4.4.3 request elecboxinfo */
  int RequestElecBoxInfoCmd(ElecBoxInfoStruct &elecboxinfo);
  /* 4.4.4 set override */
  int OverrideSetCmd(double ori);
  /* 4.4.5 set mode (jog robot) */
  int SetModeCmd (MODEKEY mode,bool is_blocked = IsDefaultBlocked);
  /* 4.4.6 set current operation mode */
  int ProgramRunModeCmd(int mode);
  /* 4.4.7 enable motor on */
  int EnableMotorCmd();
  /* 4.4.5 set mode (jog robot) */
  int CyclicKeyCmd(int scanKey);
  /* 4.4.8 set jogref */
  int JogRefSystemCmd(std::string refName);
  /* 4.4.12 confirm all alarm */
  int AllAlarmConfirmCmd();
  /* 4.4.13 set pc */
  int SetpcCmd(int linNum);
  /* 4.4.14 set io */
  int DigitalOutputEnforceCmd(int port, int data);
  int DigitalOutputEnforceRemovalCmd(int port);
  int AnalogOutputEnforceCmd(int port, short data);
  int AnalogOutputEnforceRemovalCmd(int port);
  int DigitalInputSimulationCmd(int port, int data);
  int DigitalInputSimulationRemovalCmd(int port);
  int AnalogInputSimulationCmd(int port, short data);
  int AnalogInputSimulationRemovalCmd(int port);
  /* 4.4.15 set refsys */
  int ReferenceSystemSetCmd(std::string refName);
  /* 4.4.16 set tool */
  int ToolSetCmd(std::string tool);
  /* 4.4.17 load program */
  int ProgramLoadCmd(std::string projectInfo,std::string programInfo, 
      std::vector<int> &errorInfo, bool is_blocked = IsDefaultBlocked);//工程，程序，错误行号
  /* 4.4.18 kill program */
  //TODO： Add block mode
  int ProgramKillCmd(std::string projectInfo,std::string programInfo, 
      bool is_blocked = IsDefaultBlocked);//工程，程序
#pragma endregion

#pragma region /* 4.4.19 modify variable */ //工程名.sr\\程序名,变量名,变量类型 变量名:={…}
  //bool,int,real,string,axispos,cartpos,robotaxispos,robotcartpos
  int VariableModifyCmd(std::string varInfo);//基础

  int VariableBoolModifyCmd(std::string projectInfo, std::string programInfo,
     std::string variable, bool value);
  int VariableIntModifyCmd(std::string projectInfo, std::string programInfo,
     std::string variable, int value);
  int VariableRealModifyCmd(std::string projectInfo, std::string programInfo,
     std::string variable, double value);
  int VariableStringModifyCmd(std::string projectInfo, std::string programInfo,
     std::string variable, std::string value);
  int VariableAxisposModifyCmd(std::string projectInfo, std::string programInfo,
     std::string variable, const AXISPOS& value);
  int VariableCartposModifyCmd(std::string projectInfo, std::string programInfo,
     std::string variable, CARTPOS value);
  int VariableRobotAxisposModifyCmd(std::string projectInfo,std::string programInfo,
     std::string variable, JointsPos value);
  int VariableRobotCartposModifyCmd(std::string projectInfo,std::string programInfo,
     std::string variable, RobotCartPos value);
  int VariableDynamicModifyCmd(std::string projectInfo,std::string programInfo,
     std::string variable, SDynamicPercent value);

  //新增修改工具坐标和用户坐标，20230427
  int VariableToolModifyCmd(std::string variable, Tool value);
  int VariableRefModifyCmd(std::string variable, CartRefSys value);
#pragma endregion

#pragma region //新增焊接相关变量修改，20240304  
  //ARCONDATA,ARCOFFDATA,ARCDATA,ARCRETRYDATA,ARCSEGDATA,ARCCONFIGDATA,
  int VariableArcOnDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcOnData value);
  int VariableArcOffDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcOffData value);
  int VariableArcDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcData value);
  int VariableArcRetryDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcRetryData value);
  int VariableArcSegDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcSegData value);
  int VariableArcConfigDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcConfigData value);
  //WEAVEDATA,CD,PAT,TRACKDATA,LASERTRACKDATA,PLASERFILTER,LASERARCONDATA,LASERARCDATA,LASERARCOFFDATA
  int VariableWeaveDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, WeaveData value);
  int VariableCDModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CDData value);

  int VariableCDSModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcCDSData value);

  int VariablePATModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PAT value);
  int VariableTrackDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, TrackData value);
  int VariableLaserTrackDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserTrackData value);
  int VariablePLaserFilterModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PLaserFilter value);
  int VariableLaserArcOnDataModifyCmd(std::string projectInfo, std::string programInfo, 
      std::string variable, LaserArcOnNewData value);
  int VariableLaserArcDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcData value);
  int VariableLaserArcOffDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcOffData value);

  //新增，补充位置变量修改，20240307
  //AXISDIST,CARTDIST,AUXAXISPOS,PCARTPOS
  int VariableAxisDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AxisDist value);
  int VariableCartDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CartDist value);//mode默认0
  int VariableAuxAxisPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AuxAxisPos value);
  int VariablePCartPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PalletPosition value);
  //增加动态圆滑变量修改，20240307
  //OVERLAPREL,OVERLAPBAS,RAMPTYPE
  int VariableOverlapRELModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, int value);//0~100
  int VariableOverlapABSModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, double value);//0~200 //未使用OverlapABS
  int VariableRampTypeModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, RampType value);
  //增加折弯,打磨,动力学变量修改，20240307
  //BENDDATA,BENDSYNDATA,POLISHDATA
  int VariableBendDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendTargetData value);
  int VariableBendSynDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendSynTargetData value);
  int VariablePolishDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PolishData value);
  //TOOLLOAD,PAYLOAD
  int VariableToolLoadModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PayloadData value);
  int VariablePayLoadModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PayloadData value);

#pragma endregion

#pragma region /* 4.4.20 read variable */
  int VariableBoolReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, bool &Value);
  int VariableIntReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, int &Value);
  int VariableRealReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, double&Value);
  int VariableStringReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, std::string &Value);
  int VariableAxisposReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, AXISPOS &Value);
  int VariableCartposReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, CARTPOS &Value);
  int VariableRobotAxisposReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, JointsPos &Value);
  int VariableRobotCartposReadCmd(std::string projectInfo, std::string programInfo, 
    std::string variable, RobotCartPos &Value);
  int VariableDynamicReadCmd(std::string projectInfo, std::string programInfo,
    std::string variable, SDynamicPercent &Value);
  // 新增获取工具坐标和用户坐标，20230427
  int VariableToolReadCmd(std::string variable, Tool &Value);
  int VariableRefReadCmd(std::string variable, CartRefSys &Value);
#pragma endregion
  
#pragma region //新增焊接相关变量获取，20240305
  //ARCONDATA,ARCOFFDATA,ARCDATA,ARCRETRYDATA,ARCSEGDATA,ARCCONFIGDATA,
  int VariableArcOnDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcOnData&Value);
  int VariableArcOffDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcOffData&Value);
  int VariableArcDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcData&Value);
  int VariableArcRetryDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcRetryData&Value);
  int VariableArcSegDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcSegData&Value);
  int VariableArcConfigDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcConfigData&Value);
  //WEAVEDATA,CD,PAT,TRACKDATA,LASERTRACKDATA,PLASERFILTER,LASERARCONDATA,LASERARCDATA,LASERARCOFFDATA
  int VariableWeaveDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, WeaveData& Value);
  int VariableCDReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CDData& Value);
  int VariableCDSReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcCDSData& Value);
  int VariablePATReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PAT& Value);
  int VariableTrackDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, TrackData& Value);
  int VariableLaserTrackDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserTrackData& Value);
  int VariablePLaserFilterReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PLaserFilter& Value);
  int VariableLaserArcOnDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcOnNewData& Value);
  int VariableLaserArcDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcData& Value);
  int VariableLaserArcOffDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcOffData& Value);

  //新增，补充位置变量获取，20240307
//AXISDIST,CARTDIST,AUXAXISPOS,PCARTPOS
  int VariableAxisDistReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AxisDist& Value);
  int VariableCartDistReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CartDist& Value);//mode默认0
  int VariableAuxAxisPosReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AuxAxisPos& Value);
  int VariablePCartPosReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PalletPosition& Value);
  //增加动态圆滑变量获取，20240307
  //OVERLAPREL,OVERLAPBAS,RAMPTYPE
  int VariableOverlapRELReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, int& Value);//0~100
  int VariableOverlapABSReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, double& Value);//0~200
  int VariableRampTypeReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, RampType& Value);
  //增加折弯,打磨,动力学变量获取，20240307
  //BENDDATA,BENDSYNDATA,POLISHDATA
  int VariableBendDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendTargetData& Value);
  int VariableBendSynDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendSynTargetData& Value);
  int VariablePolishDataReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PolishData& Value);
  //TOOLLOAD,PAYLOAD
  int VariableToolLoadReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PayloadData& Value);
  int VariablePayLoadReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PayloadData& Value);

#pragma endregion

#pragma region  // 新增过程变量在线修改，20230717
  int ProcessVariableModifyCmd(std::string varInfo); // 基础

  int ProcessVariableBoolModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, bool value);
  int ProcessVariableIntModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, int value);
  int ProcessVariableRealModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, double value);
  int ProcessVariableStringModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, std::string value);
  int ProcessVariableAxisposModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, const AXISPOS &value);
  int ProcessVariableCartposModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, CARTPOS value);
  int ProcessVariableRobotAxisposModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, JointsPos value);
  int ProcessVariableRobotCartposModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, RobotCartPos value);
  int ProcessVariableDynamicModifyCmd(std::string projectInfo, std::string programInfo,
    std::string variable, SDynamicPercent value);
#pragma endregion

#pragma region //新增焊接相关变量在线修改，20240307 
 //ARCONDATA,ARCOFFDATA,ARCDATA,ARCRETRYDATA,ARCSEGDATA,ARCCONFIGDATA,
  int ProcessVariableArcOnDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcOnData value);
  int ProcessVariableArcOffDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcOffData value);
  int ProcessVariableArcDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcData value);
  int ProcessVariableArcRetryDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcRetryData value);
  int ProcessVariableArcSegDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcSegData value);
  int ProcessVariableArcConfigDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, ArcConfigData value);
  //WEAVEDATA,CD,PAT,TRACKDATA,LASERTRACKDATA,PLASERFILTER,LASERARCONDATA,LASERARCDATA,LASERARCOFFDATA
  int ProcessVariableWeaveDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, WeaveData value);
  int ProcessVariableCDModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CDData value);
  int ProcessVariablePATModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PAT value);
  int ProcessVariableTrackDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, TrackData value);
  int ProcessVariableLaserTrackDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserTrackData value);
  int ProcessVariablePLaserFilterModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PLaserFilter value);
  int ProcessVariableLaserArcOnDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcOnNewData value);
  int ProcessVariableLaserArcDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcData value);
  int ProcessVariableLaserArcOffDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, LaserArcOffData value);

  //新增，补充位置变量在线修改，20240308
//AXISDIST,CARTDIST,AUXAXISPOS,PCARTPOS
  int ProcessVariableAxisDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AxisDist value);
  int ProcessVariableCartDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CartDist value);//mode默认0
  int ProcessVariableAuxAxisPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AuxAxisPos value);
  int ProcessVariablePCartPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PalletPosition value);
  //增加动态圆滑变量在线修改，20240308
  //OVERLAPREL,OVERLAPBAS,RAMPTYPE
  int ProcessVariableOverlapRELModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, int value);//0~100
  int ProcessVariableOverlapABSModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, double value);//0~200
  int ProcessVariableRampTypeModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, RampType value);
  //增加折弯,打磨变量在线修改，20240308
  //BENDDATA,BENDSYNDATA,POLISHDATA
  int ProcessVariableBendDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendTargetData value);
  int ProcessVariableBendSynDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendSynTargetData value);
  int ProcessVariablePolishDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PolishData value);

#pragma endregion

  // 绝对位置点动功能增加，20231103
  int JogToRobotAxisposCmd(JogPositionType type, JointsPos value,
      std::string toolname, std::string refname);
  int JogToRobotCartposCmd(JogPositionType type, CartPos value,
      std::string toolname, std::string refname);
  int JogToPositionExitCmd();

#pragma region //SDK模拟运动语句，新增，20231115
  int NumOfPro = 0;//SDK仿真程序数量
  int SDKSimulateProgramLoad(bool is_blocked = IsDefaultBlocked);//模拟加载
  int SDKSimulateProgramKill(bool is_blocked = IsDefaultBlocked);//模拟卸载
  int SDKSimulatePTPProCmd(HMIPos endpos, int dyn,
      SDKOverlap ovl, std::string toolname,
      std::string refname,int linenum = 1); // 模拟PTP
  int SDKSimulateLinProCmd(HMIPos endpos, int dyn, SDKOverlap ovl, ORITYPE ori,
      std::string toolname,std::string refname, int linenum = 1); // 模拟Lin
  int SDKSimulateCircProCmd(HMIPos endpos, HMIPos helppos, int dyn,
      SDKOverlap ovl, ORITYPE ori,std::string toolname,
      std::string refname, int linenum = 1); // 模拟Circ
  int SDKSimulateDynProCmd(SDynamicPercent m_DynPerc, int linenum = 1); // 模拟DYN
  int SDKSimulateWaitTimeProCmd(int waittime, int linenum = 1);         // 模拟WaitTime
  int SDKSimulateArcOnProCmd(SDKArcOnVauleStruct arconvalue,
      SDKArcSetVauleStruct arcsetvalue, int decaytime,
      SDKArcRetryData retrydata,double backdistance, int linenum = 1); // 模拟ArcOn
  int SDKSimulateArcOffProCmd(SDKInstructArcOff arcoffdata, int linenum = 1); // 模拟ArcOff
  int SDKSimulateArcSetProCmd(SDKInstructArcSet arcsetdata, int linenum = 1); // 模拟ArcSet
  int SDKSimulateAWLinProCmd(HMIPos endpos, int dyn,SDKOverlap ovl, ORITYPE ori, WeaveParam weave,
      SeamTrackParam seamtrack,std::string toolname,std::string refname, int linenum = 1); // 模拟WLin
  int SDKSimulateAWCircProCmd(HMIPos endpos, HMIPos helppos, int dyn,
      SDKOverlap ovl, ORITYPE ori, WeaveParam weave,
      SeamTrackParam seamtrack, std::string toolname,
      std::string refname, int linenum = 1); // 模拟WCirc
  int SDKSimulateArcModeProCmd(SDKInstructArcMode mode, int linenum = 1); // 模拟Arcmode
  int SDKSimulateEOFCmd(int linenum = 1);     // 模拟EOF语句
  int SDKSimulateProgramSend(int &line_number);//模拟语句发送

  //增加焊接功能中的语句 WLineRel WLinSeg 20250328
  int SDKSimulateWLinRelProCmd(SDKInstructWLinRel wlinrel, int linenum = 1); // 模拟WLinRel
  int SDKSimulateWLinSeg1ProCmd(SDKInstructWLinSeg1 wlinseg1, int linenum = 1); // 模拟WLinSeg1
  int SDKSimulateWLinSeg2ProCmd(SDKInstructWLinSeg2 wlinseg2, int linenum = 1); // 模拟WLinSeg2
  //新增设置工具、用户语句20231215
  int SDKSimulateSetToolCmd(std::string toolname, int linenum = 1);
  int SDKSimulateSetRefSysCmd(std::string refname, int linenum = 1);

#pragma endregion

#pragma region //SDK模拟运动语句，补充，20240726

  int SDKSimulateDOSetProCmd(SDKInstructDOSet dostruct, int linenum = 1);
  int SDKSimulateDOSetSyncPathProCmd(SDKInstructDOSetSyncPath dospstruct, int linenum = 1);
  int SDKSimulateDOSetSyncTimeProCmd(SDKInstructDOSetSyncTime doststruct, int linenum = 1);
  int SDKSimulateAOSetProCmd(SDKInstructAOSet aostruct, int linenum = 1);
  int SDKSimulateAOSetSyncPathProCmd(SDKInstructAOSetSyncPath aospstruct, int linenum = 1);
  int SDKSimulateAOSetSyncTimeProCmd(SDKInstructAOSetSyncTime aoststruct, int linenum = 1);
  int SDKSimulateDOPulseProCmd(SDKInstructDOPulse dopulse, int linenum = 1);
  int SDKSimulateDOPulseSyncPathProCmd(SDKInstructDOPulseSyncPath dopulsesp, int linenum = 1);
  int SDKSimulateDOPulseSyncTimeProCmd(SDKInstructDOPulseSyncTime dopulsest, int linenum = 1);
  int SDKSimulateGDOSetProCmd(SDKInstructGDOSet gdostruct, int linenum = 1);
  int SDKSimulateGDOSetSyncPathProCmd(SDKInstructGDOSetSyncPath gdospstruct, int linenum = 1);
  int SDKSimulateGDOSetSyncTimeProCmd(SDKInstructGDOSetSyncTime gdoststruct, int linenum = 1);
  int SDKSimulateDIWaitProCmd(SDKInstructDIWait diwait, int linenum = 1);
  int SDKSimulateDIWaitSyncPathProCmd(SDKInstructDIWaitSyncPath diwaitsp, int linenum = 1);
  int SDKSimulateDIWaitSyncTimeProCmd(SDKInstructDIWaitSyncTime diwaitst, int linenum = 1);
  int SDKSimulateAIWaitProCmd(SDKInstructAIWait aiwait, int linenum = 1);
  int SDKSimulateAIWaitSyncPathProCmd(SDKInstructAIWaitSyncPath aiwaitsp, int linenum = 1);
  int SDKSimulateAIWaitSyncTimeProCmd(SDKInstructAIWaitSyncTime aiwaitst, int linenum = 1);
  int SDKSimulateGDIWaitProCmd(SDKInstructGDIWait gdiwait, int linenum = 1);
  int SDKSimulateGDIWaitSyncPathProCmd(SDKInstructGDIWaitSyncPath gdiwaitsp, int linenum = 1);
  int SDKSimulateGDIWaitSyncTimeProCmd(SDKInstructGDIWaitSyncTime gdiwaitst, int linenum = 1);
  int SDKSimulateCORR_ToolProCmd(SDKInstructCORR_Tool corrtool, int linenum = 1);
  int SDKSimulateCORR_RefProCmd(SDKInstructCORR_Ref corrref, int linenum = 1);
  int SDKSimulateCORR_OffProCmd(int linenum = 1);
  int SDKSimulateOvlProCmd(SDKInstructOvl ovl, int linenum = 1);
  int SDKSimulateAxisSpaceActivateProCmd(SDKInstructAxisSpaceActivate axisspaceact, int linenum = 1);
  int SDKSimulateCartSpaceActivateProCmd(SDKInstructCartSpaceActivate cartspaceact, int linenum = 1);
  int SDKSimulateArcConfigProCmd(ArcConfigData arcconfig, int linenum = 1);
  int SDKSimulateArcJobProCmd(int arcjog, int linenum = 1);
  int SDKSimulateWireSetProCmd(SDKInstructWireSet wireset, int linenum = 1);

#pragma endregion

  // 新增焊接功能开关，20231211
  int SetArcWeldingModeCmd(int value);
  // 新增获取焊机当前电流电压，20231211
  int GetWeldingInfoCmd(WeldingParamFeedBack &Value);
  //附加轴值转换，20231221
  int SDKGetRailInfoCmd(SDKInstructRailInfo inpailinfo,RailInfo &outpailinfo,int &errorid);

  //新增运动学正逆解接口20240228,暂不对外开放,20241217对外开放
  //逆解
  int TransSDKGetInvKinematics(SDKInstructInvKinematics cartpos, JointsPos& jointpos);
  //正解
  int TransSDKGetKinematics(SDKInstructKinematics jointpos, KinematicsInfo& cartpos);





private:
  /* File transfer */
  int sendFile(const std::string &fileBuf);
  int recvFile(std::string &fileBuf);

  /* Cmd helper */
  int CmdWithNoParam(COMMUNICATIONCOMMAND commCmd, EasyTcpClient *client);
  int CmdWithNoParam(COMMUNICATIONCOMMAND commCmd);
  int CmdWithSendIntParam(COMMUNICATIONCOMMAND commCmd, int intParam);
  int CmdWithRecvIntParam(COMMUNICATIONCOMMAND commCmd, int &intParam);
  int CmdWithSendQStringParam(COMMUNICATIONCOMMAND commCmd,
      const std::string &stringParam);
  int CmdWithRecvQStringParam(COMMUNICATIONCOMMAND commCmd,
      std::string &stringParam);
  int CmdWithRecvStructParam(COMMUNICATIONCOMMAND commCmd, char *structParam);
  // int CmdWithSendQStringQByteArrayParam(COMMUNICATIONCOMMAND commCmd, const
  // std::string& stringParam, char* qByteArrayParam); int
  // CmdWithSendQStringRecvQByteArrayParam(COMMUNICATIONCOMMAND commCmd, const
  // std::string& stringParam, char* qByteArrayParam); int
  // CmdWithSendQStringRecvQByteArrayParam1(COMMUNICATIONCOMMAND commCmd, const
  // std::string& stringParam, char* qByteArrayParam);
  int CmdWithSendStructParam(COMMUNICATIONCOMMAND commCmd, char *structParam, int length);
  int CmdWithSendStructsParam(COMMUNICATIONCOMMAND commCmd, char *structsParam,
      int structLength, int num);
  int CmdWithSendIntStructParam(COMMUNICATIONCOMMAND commCmd, int intParam,
      char *structParam, int length);

  // 绝对位置点动功能增加，20231103，基础接口，不用开放
  int JogToPositionCmd(JogPositionType type, HMIPos m_EndPos,ToolRefName toolrefname);

  /* background cycle thread function recv */
  int backgroundCycleHandler();

  void BackendStateNotify();

  /* Currnet ip */
  std::string m_ip;

  /* Currnet port */
  u_short m_port;

  /* Socket client */
  EasyTcpClient *m_client;

  /* Socket client background cycle */
  EasyTcpClient *m_backClient;

  /* Date recv buffer */
  char m_recvBuffer[k_recvBufferLenth] = {0};

  /* stop flag */
  bool m_stopFlag = false;

  /* stop flag */
  bool m_backstopFlag = true;

  /* SDK Struct  */
  PlatformToSDKStruct m_sdkStruct;

  /* Data recv success flag */
  bool m_dataRecvFlag = false;

  /* erroe message */
  MessageData ErrorMessage;

/* Simulate program 20231115 */
  SDKProgramBuffer SimStructBuffer[MAX_PRO_NUM];

  int SDKSimulateProgramSendmiddle(int &line_number); // 模拟语句发送中间过渡

  ////动力学变量在线修改 TOOLLOAD,PAYLOAD.20240313暂不开放，使用不合理
  //int ProcessVariableToolLoadModifyCmd(std::string projectInfo, std::string programInfo,
  //    std::string variable, PAYLOADDATA value);
  //int ProcessVariablePayLoadModifyCmd(std::string projectInfo, std::string programInfo,
  //    std::string variable, PAYLOADDATA value);




  CARTPOS getCartPos();//替换成以下三个，这个不再使用,2024.7.12更换到私有


  class Impl;
  Impl* m_impl;

  int longRunningTask(bool delay);
  int executeWithTimeout(bool delay);
  std::atomic<bool> stopRequested; // 用于请求取消任务

  //用于解决获取数据乱码问题
  PlatformToSDKStruct buffers[2];
  std::atomic<int> active_buffer{0};
   //std::atomic<int> active_buffer = 0;
  int write_buffer = 0;

  PlatformToSDKStruct& GetReadStruct();
  PlatformToSDKStruct& GetWriteStruct();
  void ExchangeActiveBuffer();

  //新增坐标转换接口20251013
  int SDKRefConvertCMD(SDKInstructRefConvert m_OriPos, HMIPos& m_TransPos);

};
} // namespace STEPROBOTSDK
