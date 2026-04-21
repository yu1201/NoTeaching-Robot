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

  //CARTPOS getCartPos();//�滻�������������������ʹ��
  //����mode��20230427
  RobotCartPos getCartPosWorld();
  RobotCartPos getCartPosRobotBase();
  RobotCartPos getCartPosUserRef();
  HomoMatrix getCartPosR();

  AXISPOS getAxisPos();
  unsigned int getRobotRunningTime();
  int getLifeTime();

  //������ȡ����״̬�ӿڣ�20231215
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
      std::vector<int> &errorInfo, bool is_blocked = IsDefaultBlocked);//���̣����򣬴����к�
  /* 4.4.18 kill program */
  //TODO�� Add block mode
  int ProgramKillCmd(std::string projectInfo,std::string programInfo, 
      bool is_blocked = IsDefaultBlocked);//���̣�����
#pragma endregion

#pragma region /* 4.4.19 modify variable */ //������.sr\\������,������,�������� ������:={��}
  //bool,int,real,string,axispos,cartpos,robotaxispos,robotcartpos
  int VariableModifyCmd(std::string varInfo);//����

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

  //�����޸Ĺ���������û����꣬20230427
  int VariableToolModifyCmd(std::string variable, Tool value);
  int VariableRefModifyCmd(std::string variable, CartRefSys value);
#pragma endregion

#pragma region //����������ر����޸ģ�20240304  
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

  //����������λ�ñ����޸ģ�20240307
  //AXISDIST,CARTDIST,AUXAXISPOS,PCARTPOS
  int VariableAxisDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AxisDist value);
  int VariableCartDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CartDist value);//modeĬ��0
  int VariableAuxAxisPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AuxAxisPos value);
  int VariablePCartPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PalletPosition value);
  //���Ӷ�̬Բ�������޸ģ�20240307
  //OVERLAPREL,OVERLAPBAS,RAMPTYPE
  int VariableOverlapRELModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, int value);//0~100
  int VariableOverlapABSModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, double value);//0~200 //δʹ��OverlapABS
  int VariableRampTypeModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, RampType value);
  //��������,��ĥ,����ѧ�����޸ģ�20240307
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
  // ������ȡ����������û����꣬20230427
  int VariableToolReadCmd(std::string variable, Tool &Value);
  int VariableRefReadCmd(std::string variable, CartRefSys &Value);
#pragma endregion
  
#pragma region //����������ر�����ȡ��20240305
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

  //����������λ�ñ�����ȡ��20240307
//AXISDIST,CARTDIST,AUXAXISPOS,PCARTPOS
  int VariableAxisDistReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AxisDist& Value);
  int VariableCartDistReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CartDist& Value);//modeĬ��0
  int VariableAuxAxisPosReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AuxAxisPos& Value);
  int VariablePCartPosReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PalletPosition& Value);
  //���Ӷ�̬Բ��������ȡ��20240307
  //OVERLAPREL,OVERLAPBAS,RAMPTYPE
  int VariableOverlapRELReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, int& Value);//0~100
  int VariableOverlapABSReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, double& Value);//0~200
  int VariableRampTypeReadCmd(std::string projectInfo, std::string programInfo,
      std::string variable, RampType& Value);
  //��������,��ĥ,����ѧ������ȡ��20240307
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

#pragma region  // �������̱��������޸ģ�20230717
  int ProcessVariableModifyCmd(std::string varInfo); // ����

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

#pragma region //����������ر��������޸ģ�20240307 
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

  //����������λ�ñ��������޸ģ�20240308
//AXISDIST,CARTDIST,AUXAXISPOS,PCARTPOS
  int ProcessVariableAxisDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AxisDist value);
  int ProcessVariableCartDistModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, CartDist value);//modeĬ��0
  int ProcessVariableAuxAxisPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, AuxAxisPos value);
  int ProcessVariablePCartPosModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PalletPosition value);
  //���Ӷ�̬Բ�����������޸ģ�20240308
  //OVERLAPREL,OVERLAPBAS,RAMPTYPE
  int ProcessVariableOverlapRELModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, int value);//0~100
  int ProcessVariableOverlapABSModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, double value);//0~200
  int ProcessVariableRampTypeModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, RampType value);
  //��������,��ĥ���������޸ģ�20240308
  //BENDDATA,BENDSYNDATA,POLISHDATA
  int ProcessVariableBendDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendTargetData value);
  int ProcessVariableBendSynDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, BendSynTargetData value);
  int ProcessVariablePolishDataModifyCmd(std::string projectInfo, std::string programInfo,
      std::string variable, PolishData value);

#pragma endregion

  // ����λ�õ㶯�������ӣ�20231103
  int JogToRobotAxisposCmd(JogPositionType type, JointsPos value,
      std::string toolname, std::string refname);
  int JogToRobotCartposCmd(JogPositionType type, CartPos value,
      std::string toolname, std::string refname);
  int JogToPositionExitCmd();

#pragma region //SDKģ���˶���䣬������20231115
  int NumOfPro = 0;//SDK�����������
  int SDKSimulateProgramLoad(bool is_blocked = IsDefaultBlocked);//ģ�����
  int SDKSimulateProgramKill(bool is_blocked = IsDefaultBlocked);//ģ��ж��
  int SDKSimulatePTPProCmd(HMIPos endpos, int dyn,
      SDKOverlap ovl, std::string toolname,
      std::string refname,int linenum = 1); // ģ��PTP
  int SDKSimulateLinProCmd(HMIPos endpos, int dyn, SDKOverlap ovl, ORITYPE ori,
      std::string toolname,std::string refname, int linenum = 1); // ģ��Lin
  int SDKSimulateCircProCmd(HMIPos endpos, HMIPos helppos, int dyn,
      SDKOverlap ovl, ORITYPE ori,std::string toolname,
      std::string refname, int linenum = 1); // ģ��Circ
  int SDKSimulateDynProCmd(SDynamicPercent m_DynPerc, int linenum = 1); // ģ��DYN
  int SDKSimulateWaitTimeProCmd(int waittime, int linenum = 1);         // ģ��WaitTime
  int SDKSimulateArcOnProCmd(SDKArcOnVauleStruct arconvalue,
      SDKArcSetVauleStruct arcsetvalue, int decaytime,
      SDKArcRetryData retrydata,double backdistance, int linenum = 1); // ģ��ArcOn
  int SDKSimulateArcOffProCmd(SDKInstructArcOff arcoffdata, int linenum = 1); // ģ��ArcOff
  int SDKSimulateArcSetProCmd(SDKInstructArcSet arcsetdata, int linenum = 1); // ģ��ArcSet
  int SDKSimulateAWLinProCmd(HMIPos endpos, int dyn,SDKOverlap ovl, ORITYPE ori, WeaveParam weave,
      SeamTrackParam seamtrack,std::string toolname,std::string refname, int linenum = 1); // ģ��WLin
  int SDKSimulateAWCircProCmd(HMIPos endpos, HMIPos helppos, int dyn,
      SDKOverlap ovl, ORITYPE ori, WeaveParam weave,
      SeamTrackParam seamtrack, std::string toolname,
      std::string refname, int linenum = 1); // ģ��WCirc
  int SDKSimulateArcModeProCmd(SDKInstructArcMode mode, int linenum = 1); // ģ��Arcmode
  int SDKSimulateEOFCmd(int linenum = 1);     // ģ��EOF���
  int SDKSimulateProgramSend(int &line_number);//ģ����䷢��

  //���Ӻ��ӹ����е���� WLineRel WLinSeg 20250328
  int SDKSimulateWLinRelProCmd(SDKInstructWLinRel wlinrel, int linenum = 1); // ģ��WLinRel
  int SDKSimulateWLinSeg1ProCmd(SDKInstructWLinSeg1 wlinseg1, int linenum = 1); // ģ��WLinSeg1
  int SDKSimulateWLinSeg2ProCmd(SDKInstructWLinSeg2 wlinseg2, int linenum = 1); // ģ��WLinSeg2
  //�������ù��ߡ��û����20231215
  int SDKSimulateSetToolCmd(std::string toolname, int linenum = 1);
  int SDKSimulateSetRefSysCmd(std::string refname, int linenum = 1);

#pragma endregion

#pragma region //SDKģ���˶���䣬���䣬20240726

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

  // �������ӹ��ܿ��أ�20231211
  int SetArcWeldingModeCmd(int value);
  // ������ȡ������ǰ������ѹ��20231211
  int GetWeldingInfoCmd(WeldingParamFeedBack &Value);
  //������ֵת����20231221
  int SDKGetRailInfoCmd(SDKInstructRailInfo inpailinfo,RailInfo &outpailinfo,int &errorid);

  //�����˶�ѧ�����ӿ�20240228,�ݲ����⿪��,20241217���⿪��
  //���
  int TransSDKGetInvKinematics(SDKInstructInvKinematics cartpos, JointsPos& jointpos);
  //����
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

  // ����λ�õ㶯�������ӣ�20231103�������ӿڣ����ÿ���
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

  int SDKSimulateProgramSendmiddle(int &line_number); // ģ����䷢���м����

  ////����ѧ���������޸� TOOLLOAD,PAYLOAD.20240313�ݲ����ţ�ʹ�ò�����
  //int ProcessVariableToolLoadModifyCmd(std::string projectInfo, std::string programInfo,
  //    std::string variable, PAYLOADDATA value);
  //int ProcessVariablePayLoadModifyCmd(std::string projectInfo, std::string programInfo,
  //    std::string variable, PAYLOADDATA value);




  CARTPOS getCartPos();//�滻�������������������ʹ��,2024.7.12������˽��


  class Impl;
  Impl* m_impl;

  int longRunningTask(bool delay);
  int executeWithTimeout(bool delay);
  std::atomic<bool> stopRequested; // ��������ȡ������

  //���ڽ����ȡ������������
  PlatformToSDKStruct buffers[2];
  std::atomic<int> active_buffer{0};
   //std::atomic<int> active_buffer = 0;
  int write_buffer = 0;

  PlatformToSDKStruct& GetReadStruct();
  PlatformToSDKStruct& GetWriteStruct();
  void ExchangeActiveBuffer();

  //��������ת���ӿ�20251013
  int SDKRefConvertCMD(SDKInstructRefConvert m_OriPos, HMIPos& m_TransPos);

};
} // namespace STEPROBOTSDK
