#pragma once
namespace STEPROBOTSDK {
    typedef enum {
        eOK = 1,
        eErr = 2,
        eRealTimeSystem = 10,
        eHMI = 11,
        eSDK = 30,
        eSDKKey = 31,
        eHMIKey = 50,
        eCyclicKey = 51,
        eCyclicCommand = 100,
        eCyclicStatus = 101,
        eHMIFileSendTo = 103,  // HMI发送文件给非实时
        eHMIFileReceive = 104, // HMI接收非实时发送的文件
        eFunctionNumSeq = 105, // HMI接收功能码
        eHMILanguageChange = 107,
        eHMIType = 108, // 模式切换，0表示示教器控制，1表示控制柜控制
        eGetVersionInfo = 110,
        eDirectoryStructureQuery, // 请求控制器指定文件夹下的目录结构
        eBackupToController,      // 把当前数据整体备份到控制器
        eBackupToUDisk,           // 把当前数据整体备份到U盘
        eRecoveryFromController,  // 恢复指定控制器中的压缩文件
        eRecoveryFromUDisk,       // 恢复指定U盘中的压缩文件
        eFileConversionFromDos = 116, // change file format from windows to linux
        eArcDataVarNameUpdate = 118,  // 焊接监视界面更新arcdata变量

        eEnableTimeClear = 119,       // 系统上使能时间清零
        eJogConfigSet = 120,          // 点动坐标系配置
        eDateTimeSet = 121,           // 时间设置
        eCommonCmdConfig = 122,       // 常用命令设置
        eSpeedIncrementSet = 123,     // 速度增量设置
        eSpeedGearSet = 124,          // 速度档位设置
        eButtonFuncSet = 125,         // 按键功能设置
        eSystemTimeClear = 126,       // 系统开机时间清零
        eLogoutTimeSet = 127,         // 自动注销登录时间
        eRobotRunningTimeClear = 128, // 程序运行时间清零

        eDirectoryRebuilding = 130,
        eProgramFileImportZip = 131, // 用户程序导入控制器
        eProgramFileExportZip = 132, // 用户程序导出控制器
        eConfigFileImportZip = 133,  // 配置文件导入控制器
        eConfigFileExportZip = 134,  // 配置文件导出控制器
        eHardwareInfoRequest = 135,
        eSoftwareTimeSet = 140,
        eSoftwareTimeRequest,
        eSoftwareTimePwdSet,
        eSoftwareTimePwdRequest,
        eHMIOfflineSet = 144,                // 脱机命令
        eBackUpForceRecoveryFromCtrl = 145,  // 从控制器强制恢复
        eBackUpForceRecoveryFromUdisk = 146, // 从U盘强制恢复
        eRelocation = 147,                   // 移机命令
        eAuxAxisSystemSave = 151,
        eAuxAxisConfigSave,
        eSoftInit = 153,
        eSoftAxisSetting,
        eSoftCartSetting,
        ePowerOffSetting = 156, // 断电设置
        eIPSet = 160,
        eIPGet,
        eFunctionInfoGet =
            162, // 获取函数信息汇总 如“project.sr,INT userAdd,INT a,AXISPOS
                // b;project.sr,INT userAdd,INT a,AXISPOS b...”
        eIsVarUsed = 163, // 查询指定变量是否被使用
        eCalibrationOpen = 165,
        eAuxAxisSync = 166,
        eSingularProtect = 167,
        eCmdDefaultConLParaSave = 168,
        eControllerStatusRequest = 171,
        eExtendTrialTime =
            175, // 延长试用期时间---数据部分是文件里的密文。非实时的反馈可能是ok，或者err。

        /*  SDK新增  */
        eRequestRobotInfo = 200,
        eRequestElecBoxInfo = 201,
        eVarRead = 202, // 单个变量获取
        eProcessVarModify = 203, // 新增过程变量修改，20230717 新增
        eDHParamSet = 204,       // DH参数设置 新增，20230726，非标版本

        //新增运动指令命令 20231115
        eSDKSimulateLoad = 205,  // SDK模拟加载程序
        eSDKSimulateKill = 206,  // SDK模拟卸载程序
        eSDKInstructSend = 207,  // SDK语句发送

        // 新增焊机当前电流电压值获取。20231211
        eSDKGetWeldingInfo = 208,

        //新增附加轴转换接口，20231221
        eSDKGetRailInfo = 209,

        //新增正逆解转换接口，20240228
        eSDKGetKinematics = 210,
        eSDKGetInvKinematics = 211,
        //新增坐标转换接口，20251013
        eSDKRefConvert = 213,

        // 1000开始是命令
        eSingleAlarmConfirm = 1000,       // 单个错误报警信息确认
        eSingleAlarmConfirmWithParameter, // 带参数的错误报警信息
        eAllAlarmConfirm,                 // 确认所有报警信息
        eAlarmFileTranfer,                // 日志文件传输
        eReferenceSystemSet,              // 参考坐标系设置
        eToolSet,                         // 工具设置
        eOverrideSet,                     // override设置
        eJogRefSystemSet,                 // 点动坐标系设置命令
        eToolAlignReady,                  // 工具手对齐准备命令
        eToolAlignExit,                   // 工具手对齐退出命令
        eAnalogInputSimulation,           // 模拟输入仿真命令
        eAnalogInputSimulationRemoval,    // 模拟输入去除仿真命令
        eAnalogOutputEnforce,             // 模拟输出强制命令
        eAnalogOutputEnforceRemoval,      // 模拟输出去除强制命令
        eDigitalInputSimulation,          // 数字输入仿真命令
        eDigitalInputSimulationRemoval,   // 数字输入去除仿真命令
        eDigitalOutputEnforce,            // 数字输出强制命令
        eDigitalOutputEnforceRemoval,     // 数字输出去除强制命令
        eControlAuthority,                // 控制权限命令
        eWriteAuthority,                  // 写权限命令
        eControllerTimeSet,               // 控制器时间设置命令
        eUserLogin,                       // 用户登录命令
        eUserExit,                        // 用户退出命令
        eUserNew,                         // 用户新建命令
        eUserDelete,                      // 用户删除命令
        eUserModify,                      // 用户修改命令
        eRestart,                         // 重启命令
        eGenerateSystemReport,            // 生成系统报告命令
        eVariableNew,                     // 变量新建命令
        eVariableModify,                  // 变量修改命令
        eVariableMove,                    // 变量移动命令
        eVariableRename,                  // 变量重命名命令
        eVariableDelete,                  // 变量删除命令
        eJogToPositionOpen,               // jogToPosition打开命令 1033
        eJogToPositionExit,               // jogToPosition关闭命令
        eRefresh,                         // 工程界面刷新命令
        eProjectClose,                    // 关闭命令
        eProjectNew,                      // 工程新建命令
        eProjectRename,                   // 工程重命名命令
        eProjectDelete,                   // 工程删除命令
        eProgramLoad,                     // 程序载入命令 1040
        eProgramKill,                     // 程序关闭命令
        eProgramNew,                      // 程序新建命令
        eProgramDelete,                   // 程序删除命令
        eProgramRename,                   // 程序重命名命令
        eProgramModify,      // 程序修改命令（引发文件传输）
        eProgramSaveAndLoad, // 程序修改并load
        eProgramCopy,        // 程序复制
        eProjectCopy,        // 程序粘贴
        eSetpc,              // setpc命令
        eProgramStop,        // 程序停止命令
        eEnableMotor,        // 使能命令
        eProgramRunMode,     // 程序运行模式
        eEnableRobot,        // 2013.3.29 启动机器人
        eDisableRobot,       // 2013.3.29 关闭机器人
        eVariableUpdates,    // 临时变量更新（控制器发送给HMI）
        eMessageTransfer,
        eMessageRefresh,
        eDigitalOutput = 1065,
        eDigitalOutputGroup,
        eBackwardModeEnable = 1200,
        eIndForceOn = 1202,        // 界面开启独立轴
        eIndForceOff = 1203,       // 界面关闭独立轴
        eMultiLayerVarNew = 1500,  // 多层多道变量新建
        eMultiLayerVarDelete,      // 多层多道变量删除
        eMultiLayerVarUpdate,      // 多层多道变量更新
        eAxisSpaceSetting = 1600,  // HMI发送关节工作空间设置命令
        eCartSpaceSetting = 1601,  // HMI发送笛卡尔工作空间设置命令
        eCartSpaceActivate = 1602, // 工作空间是否被监视界面激活
        eAxisSpaceActivate = 1603, // 工作空间是否被监视界面激活
        eWorkSpaceInit = 1604,     // 工作空间数据请求命令
        eWorkSpaceOpr = 1605,      // Override按钮操作命令
        eAutoRunStatusSet = 1610, // 0标示未启用一键启动，1表示启用一键启动
        eAutoRunStatusRequest,

        //2000开始为新增运动命令，20231115新增
        eProgPTP = 2001,        //SDK模拟点到点运动语句
        eProgLin = 2002,        //SDK模拟直线运行语句
        eProgCirc = 2003,       //SDK模拟圆弧运行语句
        eSDKDyn = 2009,         //SDK模拟设置Dyn语句

        eProgOvl = 2011,          //SDK模拟设置圆滑语句

        eProgRefsys = 2013,     //设置用户坐标
        eProgTool = 2014,     //设置工具坐标

        eProgSetDO = 2019,       //SDK模拟设置DO语句 
        eProgSetAO = 2020,        //SDK模拟设置AO语句

        eProgEOF = 2024,      // SDK模拟EOF语句
        eProgWaitTime = 2025, // SDK模拟设置等待时间语句
        
        eProgSetGDO = 2031,                // SDK模拟设置GDO语句
                                           
        eProgDOPulse = 2037,               // SDK模拟语句
        eProgDOSetSyncTime = 2038,         // SDK模拟语句
        eProgDOSetSyncPath = 2039,         // SDK模拟语句
        eProgDOPulseSyncTime = 2040,       // SDK模拟语句
        eProgDOPulseSyncPath = 2041,       // SDK模拟语句
                                           
        eProgDIWaitSyncTime = 2048,        // SDK模拟语句
        eProgDIWaitSyncPath = 2049,        // SDK模拟语句
                                           
        eProgGDIWaitSyncTime = 2052,       // SDK模拟语句
        eProgGDIWaitSyncPath = 2053,       // SDK模拟语句
        eProgDIWaitSimple = 2054,          // SDK模拟语句
        eProgGDIWaitSimple = 2055,         // SDK模拟语句
        eProgAOSetSyncPath = 2056,         // SDK模拟语句
        eProgAOSetSyncTime = 2057,         // SDK模拟语句
        eProgGDOSetSyncPath = 2058,        // SDK模拟语句
        eProgGDOSetSyncTime = 2059,        // SDK模拟语句
        eProgAIWaitSyncPath = 2060,        // SDK模拟语句
        eProgAIWaitSyncTime = 2061,        // SDK模拟语句
                                           
        eProgAIWait = 2066,                // SDK模拟语句
                                           
        eProgCartSpaceActivate = 2601,     // SDK模拟语句
        eProgAxisSpaceActivate = 2602,     // SDK模拟语句

        eWeldMode = 3000,      // 焊接模式命令

        eArcProgCorrectOff = 3008,         // SDK模拟语句

        //新增焊接运动命令，20231115新增
        eArcProgArcOn = 3012,     //SDK模拟起弧语句
        eArcProgArcOff = 3013,    //SDK模拟灭弧语句
        eArcProgArcSet = 3014,    //SDK模拟焊接参数设置语句
        eArcProgWLin = 3015,      //SDK模拟WLin语句
        eArcProgWCirc = 3016,     //SDK模拟WCirc语句

        //新增焊接运动命令，2024.7.10新增
        eArcProgArcMode = 3018,//3018
        eArcProgArcJob = 3019,              // SDK模拟语句

        eArcIUFeedBack = 3026, // 焊接监视界面更新焊接实际值
        eArcProgWLinSeg = 3027, // SDK模拟WLinSeg语句
        eArcProgWireSet = 3032,             // SDK模拟语句
                                            
        eArcProgWLinRel = 3037, // SDK模拟WLinRel语句
        eArcProgArcConfig = 3038,           // SDK模拟语句

        eBendSimu = 3100,      // 设置仿真模式
        eBendRefDis = 3107,    // 示教电子尺的零位
        eMessageLine = 3200,   //

        ePalletizingParameter = 3300, // 读取码垛初始化参数：存储的产品、模板名称
        // 产品XML文件处理
        ePalletizingProductCreate, // 产品数据新建
        ePalletizingProductModify, // 产品数据修改
        ePalletizingProductDelete, // 产品数据删除
        ePalletizingProductRename, // 产品数据重命名
        // 模板XML文件处理
        ePalletizingTemplateCreate, // 模板数据新建
        ePalletizingTemplateModify, // 模板数据修改
        ePalletizingTemplateDelete, // 模板数据删除
        ePalletizingTemplateRename, // 模板数据重命名
        // 托盘XML文件处理
        ePalletizingStationCreate, // 托盘数据新建
        ePalletizingStationModify, // 托盘数据修改
        ePalletizingStationDelete, // 托盘数据删除
        ePalletizingStationRename, // 托盘数据重命名
        // 标准平面XML文件处理
        ePalletizingStandardPlaneCreate, // 平面数据新建
        ePalletizingStandardPlaneModify, // 平面数据修改
        ePalletizingStandardPlaneDelete, // 平面数据删除
        ePalletizingStandardPlaneRename, // 平面数据重命名
        // 自定义平面XML文件处理
        ePalletizingCustomPlaneCreate, // 平面数据新建
        ePalletizingCustomPlaneModify, // 平面数据修改
        ePalletizingCustomPlaneDelete, // 平面数据删除
        ePalletizingCustomPlaneRename, // 平面数据重命名
        // 基本垛堆XML文件处理
        ePalletizingBasicStkCreate, // 垛堆数据新建
        ePalletizingBasicStkModify, // 垛堆数据修改
        ePalletizingBasicStkDelete, // 垛堆数据删除
        ePalletizingBasicStkRename, // 垛堆数据重命名
        // 高级垛堆XML文件处理
        ePalletizingSeniorStkCreate, // 垛堆数据新建
        ePalletizingSeniorStkModify, // 垛堆数据修改
        ePalletizingSeniorStkDelete, // 垛堆数据删除
        ePalletizingSeniorStkRename, // 垛堆数据重命名

        eProgPalletizingEntryPoint,   // 码垛进入点
        eProgPalletizingStatusRead,   // 码垛状态读取
        eProgPalletizingStatusUpdate, // 码垛状态更新
        eProgPalletizingStatusSet,    // 码垛状态设置
        eProgPalletizingUpdateProduct, // 更新码垛产品         此段命令没有使用
        ePalletizingModify,            // 监视码垛数据修改
        ePalletNumberUpdate,           // 码垛总数改变
        ePalletizingUpdate,            // 单个码垛变量改变
        ePalletizingACK,               // 码垛命令回复    此命令没有使用
        ePalletizingTest,              // 码垛测试指令
        ePalletizingProgGenerate = 3341,
        ePalletizingProgDelete,
        ePalletizingProgListCheck,
        ePalletizingVarCheck,
        ePalletizingPointUpdateDisable,
        ePalletPointUpdate = 3349,
        ePalletAxis4ConfigModify = 3400,

        eHMIDisconnect = 3817, // 未被使用
        ePalletTest = 4000,    // 码垛测试命令
        eCameraData = 4200,    // 激光示教
        eLaserOpen,            // 激光打开关闭
        eLaserInit = 4212,     // 激光初始化
        eHmiRequestStdPlank = 4300,
        eHmiSendStdPlank,
        eHmiSendMeasurePlank,
        eLineLaserAccuracyCompensationSet = 4400,
        eHome = 5000,
        eOffsetHome,
        eOffsetTuningHome,
        eHomeDataGet,
        eHomeDataSet,
        eHomeDataBackup,
        eOffsetAngleHome = 5007,
        eIOModuleSet = 5100,           // IO模块设置
        eIOConfigSet = 5101,           // IO变量设置
        eIOSystemSignalSet = 5102,     // IO系统信号设置
        eIOWeldSignalSet = 5103,       // IO焊接信号设置
        eIOWeldSignalSetEx = 5104,     // IO模拟信号设置
        eIOModuleRequest = 5105,       // IO模块请求
        eIOConfigRequest = 5106,       // IO变量请求
        eIOSystemSignalRequst = 5107,  // IO系统信号请求
        eIOWeldSignalRequest = 5108,   // IO焊接信号请求
        eIOWeldSignalRequestEx = 5109, // IO焊接模拟信号请求
        eIOBendSignalSet = 5110,       // IO折弯信号设置
        eIOCartSpaceSet = 5111,        // IO笛卡尔空间设置
        eIOAxisSpaceSet = 5112,        // IO关节空间设置
        eIOBendSignalRequest = 5113,   // IO折弯信号请求
        eIOCartSpaceRequest = 5114,    // IO笛卡尔信号请求
        eIOAxisSpaceRequest = 5115,    // IO关节空间信号请求
        eIOPortStatusRequest = 5116,
        ePLaserDataRequest = 5117, // 点激光配置请求
        ePLaserDataSet = 5118,     // 点激光配置设置

        eIOGProgSignalRequst = 5120, // 开机HMI请求组程序配置命令
        eIOGProgSignalSet = 5121,    // 组程序配置设置命令

        eProgramAppointmentSingleSet = 5300, // 信号和程序关联匹配，涂胶非标
        eProgramAppointmentSingleRequst = 5301,
        eProgramAppointmentState = 5302,
        eProgramAppointmentSetting = 5303, // 预约功能设置，涂胶非标
        eAppointmentSettingRequst = 5304,
        eProgramAppointmentSetCount = 5306,
        eProgramAppointmentCountUpdate = 5307,

        eManualPositionReset = 5400,
        eManualPositionData = 5401,
        eDefaultSpeedSet = 5500,
        eSetAxisVirtual = 5510,

        eBelowMoldRequest = 5600,      // HMI请求下模数据
        eBelowMoldSet,                 // HMI设置下模数据
        eBenderRefRequest,             // HMI请求折弯坐标系数据
        eBenderRefSet,                 // HMI设置折弯坐标系数据
        eBenderIORequest,              // HMI请求折弯IO数据
        eBenderIOSet,                  // HMI设置折弯IO数据
        eBenderTuningRequest,          // HMI请求折弯校正数据
        eBenderTuningSet,              // HMI设置折弯校正数据
        eBenderGetMaterialRequest,     // HMI请求折弯取料数据
        eBenderGetMaterialSet,         // HMI设置折弯取料数据
        eBenderPutMaterialRequest,     // HMI请求折弯放料数据
        eBenderPutMaterialSet,         // HMI设置折弯放料数据
        eBenderMessageRequest,         // HMI请求折弯信息数据
        eBenderMessageRefresh,         // HMI请求刷新折弯信息数据
        eBenderMessageClear,           // HMI请求清除折弯信息
        eBenderTest,                   // HMI请求测试折弯机信号
        eBenderGetRefresh,             // HMI请求测试折弯机信号
        eBenderPutRefresh,             // HMI请求测试折弯机信号
        eBenderGetReset = 5618,        // HMI请求测试折弯机信号
        eBenderPutReset,               // HMI请求测试折弯机信号
        eBenderIPSet,                  // 折弯机IP地址保存
        eBenderIPConnect,              // 折弯机tcp通讯连接
        eBenderIPClose,                // 折弯机tcp通讯关闭
        eBenderMouldDataSync,          // 下模参数请求
        eBenderIPRequest,              // 请求IP信息
        eBenderProgNameAndIndexUpdate, // 更新折弯机程序

        // 视觉和Socket
        //    eOpenCamera = 5700,
        //    eTakePhoto,
        //    eGetPos,
        //    eGetMatch,
        //    eCloseCamera,
        //    eSend,
        //    eRecv,
        eProtocolNew = 5707, // 新建协议
        eProtocolCopy,       // 复制协议
        eProtocolRename,     // 重命名协议
        eProtocolDelete,     // 重命名协议
        eProtocolRequest,
        eSocketNumUpdata,
        eSocketStatusUpdata,
        eSocketOpen = 5714,
        eSocketClose,
        eSocketSendString,
        eSocketSendByte,
        eSocketReadReal,
        eSocketSendReal,
        eSocketSendInt,
        eSocketReadInt,
        eSocketReceiveString,

        eServoDataRequest = 5725, // 上传
        eServoDataSet,            // 下载
        eEncoderStudy,
        eEncoderStudyResult,
        ePIDStudyTest,
        ePIDStudy,
        ePIDStudyResult,
        eServoStudyCancel,

        eConveyorRequest = 5750,   // 流水线跟踪数据请求
        eConveyorSet,              // 流水线跟踪数据设置
        eConveyorEncoderRequest,   // 编码器跟踪数据请求
        eConveyorStatusRequest,    // 流水线状态数据请求
        eConveyorActive,           // 流水线激活
        eConveyorGo,               // 运动到第一个可被跟踪物品位置
        eConveyorResetWaitObjects, // 清空待跟踪队列
        eConveyorResetStatistics,  // 清空统计信息
        eConveyorRefRequset,       // 坐标系数据请求命令

        // 动力学
        eStallTypeSet = 5800,   // 安装方式保存
        eDynModelBasicSet,      // 基本参数设定
        eDynModelMechanicalSet, // 动力学模型设定
        eIdentifyTry,           // 辨识试运行
        eIdentifyTryComplate, // 辨识试运行完成，完成信号是非实时发过来的
        eIdentifyFull,        // 辨识完整模型
        eIdentifyFullComplate,             // 辨识完整模型完成
        eIdentifyFriction,                 // 辨识摩擦力
        eIdentifyFrictionComplate,         // 辨识摩擦力完成
        eIdentifyFullSave,                 // 保存完整模型  5809
        eDynModelSetThreshold,             // 机械参数摩擦力阈值保存
        eIdentifyFrictionSave,             // 保存摩擦力数据
        eIdentifyHighSpeedPayLoad,         // 高速辨识负载   5812
        eIdentifyLowSpeedPayLoad,          // 低速负载辨识  5813
        eIdentifyHighSpeedPayLoadComplate, // 高速辨识负载完成
        eIdentifyLowSpeedPayLoadComplate,  // 低速负载辨识完成
        eBodyLoadSet,                      // 本体负载设置  5816
        eFeedforwardSet,                   // 前馈补偿
        eCollisionDetectionSet,            // 碰撞检测设置
        eDirectTeachSet,                   // 拖动示教设置
        eMonitorCurrent,                   // 开始监视电流
        eMonitorStop,                      // 停止监视电流
        eDragTeach,
        eIdentifyCalculateSet,     // 负载辨识手动辨识设置
        eIdentifyLowSpeedComplete, // 低速辨识完成
        eIdentifyFailed,           // 辨识失败
        eIdentifyCancel,           // 辨识取消
        eVibrationDamp,            // 振动抑制
        eMidPointUpdate,           // 对称点更新。数据部分是12个double。
        eIdentifyNopayLoad = 5829, // 空载辨识

        eSamplingStart = 5900, // 采集开始
        eSamplingStop,         // 采集停止
        eSamplingDataRecv,     // 采集数据传输

        eExternalVarMonitor = 5910, // 监视外部变量
        eExternalVarReceive = 5911, // 接收外部变量
        eExternalVarModify = 5912,  // 修改外部变量

        eAnyBusStatus = 7102,        // Anybus状态
        eAnyBusConfigRequest = 7103, // Anybus配置请求
        eAnyBusConfigSet,            // Anybus配置设置

        eProgCORR_Tool = 7110,        // SDK模拟语句
        eProgCORR_Ref = 7111,         // SDK模拟语句

    } COMMUNICATIONCOMMAND;

    /* 最大关节数 */
    #define MAX_JOINT_NUM 6

    /* 最大辅助轴数 */
    #define MAX_AUX_NUM 6

    /* 开根号内是负数 */
    #define PM_SQRT_ERR -5

    #define SQRT_FUZZ (-0.000001)

    #define sq(x) ((x) * (x)) // 平方

    #define PAI 3.1415926

    /* SDK最大语句数 20231122 */
    #define MAX_PRO_NUM 10
    #define MAX_SIZE_CHAR 1500

    /* 三维矩阵 Matrix3 */
    typedef double Matrix3[3][3];
    /* 三维向量 Vector3 */
    typedef double Vector3[3];
    /* 3*3旋转矩阵 Rotation */
    typedef Matrix3 Rotation;

    /* 关节位置 JointsPos */
    typedef double JointsPos[MAX_JOINT_NUM];
    /* 附加轴关节位置 AuxPos */
    typedef double AuxPos[MAX_AUX_NUM];

    typedef struct {
        #ifdef _WIN32
            unsigned long int m_DI[64]; //  DIO_NUM 除以 32 = 64
            unsigned long int m_DIMask[64];
        #else
            unsigned int m_DI[64]; //  DIO_NUM 除以 32 = 64
            unsigned int m_DIMask[64];
        #endif
    } RobotDI;
    typedef struct {
        #ifdef _WIN32
            unsigned long int m_DO[64];
            unsigned long int m_DOMask[64];
        #else
            unsigned int m_DO[64];
            unsigned int m_DOMask[64];
        #endif
    } RobotDO;
    typedef struct {
      short m_AI[32];
        #ifdef _WIN32
            unsigned long int m_AIMask;
        #else
            unsigned int m_AIMask;
        #endif
    } RobotAI;

    typedef struct {
      short m_AO[32];
        #ifdef _WIN32
            unsigned long int m_AOMask;
        #else
            unsigned int m_AOMask;;
        #endif

    } RobotAO;

    typedef enum { eInfo, eWarning, eError } MESSAGETYPE;

    /* 一般动态参数 */
    typedef struct
    {
      double m_Vel;   // 速度
      double m_Acc;   // 加速度
      double m_Dec;   // 减速度
      double m_Jerk;  // 加加速度
      double m_Tjolt; // 时间
    } SubDynamic;
    /* 运动动态状态 DynamicState */
    typedef SubDynamic DynamicState;

    /* 笛卡尔运动动态 CartDynState */
    typedef DynamicState CartDynState;

    typedef DynamicState JointDynState[MAX_JOINT_NUM];  //每个关节都有动态参数

    // 2013.4.22 带关节百分比的完整动态
    typedef struct {
      SubDynamic m_SegmentDynamic;   // 路径动态
      SubDynamic m_OriDynamic;     // TCP姿态动态
      SubDynamic m_JointPercent;     // 关节动态
    } SDynamicPercent;


    /* MessageDate */
    typedef struct {
      MESSAGETYPE m_MessageType;
      int m_MessageID;
      char m_MessageString[96];
      double m_MessageTime; // time_t
      char m_MessageSource[96];
    } MessageData;

    typedef enum {
      eManual,           // 手动模式
      eManualHigh,       // 高速手动模式
      eAutomatic,        // 自动模式
      eAutomaticExternal // 外部自动，用于有上层控制系统的情况
    } OPERATIONMODE;

    /* 点动坐标系类型*/
    typedef enum {
      eJoints, // 关节坐标系
      eWorld,  // 世界坐标系
      eBase,   // 基坐标系
      eTool,   // 工具坐标系
      eCustom, // 工作台坐标系/工件坐标系/用户坐标系
      eGo,
      eTTS // 焊炬点动坐标系，无法在该坐标系下进行姿态点动
    } REFSYS;

    typedef enum {
      eRun,   // 0
      ePause, // 1
      eStop,  // 2
      eReturn
    } PROGRAMSTATE;

    typedef enum { eContinue, eStep, eMotionStep } PROGRAMMODE;

    typedef struct {

      Rotation rot;

      Vector3 xyz;

    } HomoMatrix;

    /* 附加轴系统类型 */
    typedef enum {
      // eNull = 0,
      eERegular = 1, // 不变的类型
      eEAsys,        // 变位机系统
      eEBsys,
      eECsys,
      eEDsys,
      eEEsys,
      eEFsys,
      eERsys // 导轨类系统
    } AUXSYSTYPE;

    /* 位姿矩阵	CartFrame */
    typedef HomoMatrix CartFrame;

    typedef struct {
      // Vector3 m_V_c;     // 坐标系速度
      // Vector3 m_A_c;     // 坐标系加速度
      Vector3 m_Posit;      // 跟踪坐标系的位置
      double m_synvel;      // 坐标系跟踪速度
      double m_synprogress; // 坐标系跟踪路程量
      double remainpath;    // 坐标系跟踪剩余路程
    } Track_Refsys;

    /* 坐标系更新类型 */
    typedef enum

    { eUpdateNot = 1, // 坐标系不更新，用在静坐标系上
      eUpdateLink,    // 通过link 更新，用在变位机坐标系上
      eUpdateSMemSimu, // 通过共享内存仿真更新，可能在调试中可用到
      eUpdateSMemIntern, // 通过内部共享内存更新，暂时未用到
      eUpdateSMemExtern // 通过外部共享内存更新，用在RefsysVar坐标系跟踪上。
    } UPDATETYPE;

    /* 参考坐标系 */
    typedef struct // 动坐标系 静坐标系
    {
      char m_Name[32]; // 坐标系名称

      AUXSYSTYPE m_AuxType;     // 附加轴系统类型
      CartFrame m_FrameCurrent; // To World

      int m_TrackInited; // 跟踪坐标系是否初始化完成
      CartFrame m_BaseRef; // 动坐标系参考的静坐标系，其值为相对world的齐次矩阵
      CartFrame m_Frame; // 端口更新  相对 BaseRef 的当前坐标系值。
      CartFrame frame_tmp;

      int m_SynAcc; // 同步加速中,这个量要在开始的时候设置为1
      CartFrame m_FrameI;      // 初始的Frame值
      CartFrame m_FramePre;    // 保存上一时刻的m_frame
      Track_Refsys m_TrackRef; // 跟踪坐标系的信息
      UPDATETYPE m_UpdType; // 更新类型TMcuUpdateTyp;  // tracking update type,
                            // normal: McuUpdateSMemIntern
      // cyy
      // DynamicCoordinateTrack m_dynamic_coordinate;  //动态坐标系跟踪相关数据
      // Track m_Track;
      // cyy
      int m_IsHolding; // 是否是法兰坐标系
    } RefsysElement;

    typedef enum {
      eJoint = 1,   // 关节位置
      ePosCart,     // cart 位置
      ePosComplete, // 有joint 和 cart 部分都有效
      ePosMix // 混合型  部分为关节,部分为joint 主要用在四关节当中。
              // ePosJC   ,      // joint 和 cart 一致有效
              // ePosJM   ,      // joint 和 Mix 一致有效
              // 其他类型？
    } POSTYPE;

    /* 完整的位置表示：包括关节和笛卡尔的位置 PosFull */
    typedef struct {
      HomoMatrix m_Cart;   // 笛卡尔位置
      JointsPos m_Joint;   // 关节位置
      AuxPos m_AuxJoint;   // 附加轴位置
      unsigned int m_mode; // 关节模式
      //RefsysElement *m_ref; // 参考坐标系：内部处理时，进行关节到Matrix的转换
      unsigned int m_ref;
      POSTYPE m_postype; // posful保存的类型
    } PosFull;

    typedef struct {
      double x;
      double y;
      double z;
      double a;
      double b;
      double c;
    } CoordinateSystem;

    //typedef struct {
    //  ARCWELDINGMODE m_WeldMode;
    //} ArcWeldingState;


    typedef enum { eSimuWelding, eHotWelding, eCheckPath } ARCWELDINGMODE;

    /* Platform sdk struct */
    struct PlatformToSDKStruct {
      char m_TeachRefSysName[32]; //
      char m_ProjectName[32];     //
      char m_ProgramName[32];     //
      char m_ToolName[32];        //
      MessageData m_Message;      //
      RobotDI m_RobotDI;
      RobotDO m_RobotDO;
      CartDynState m_ActCartDynState; //
      OPERATIONMODE m_OperationMode;  //
      int m_MotorEnable;
      REFSYS m_JogRefSys; //
      double m_Override;
      PROGRAMSTATE m_ProgramState; // 机器人状态
      PROGRAMMODE m_ProgramMode;   // 程序运行状态
      int m_CurrentLine;           //
      PosFull m_ActPos;            //
      RobotAI m_RobotAI;
      RobotAO m_RobotAO;
      unsigned int m_RobotRunningTime; // 运行时间
      int m_LifeTime;                  // 开机时间
      //新增，将原本的世界坐标分类，世界、基、用户
      CoordinateSystem Cartpos_World;
      CoordinateSystem Cartpos_Robotbase;
      CoordinateSystem Cartpos_UserRef;
      //新增焊接模式状态
      ARCWELDINGMODE m_weldState;
      //新增关节动态信息
      JointDynState m_JointDynState;
      int m_LoadProgRunTime;
      int m_TotalRunTime;
      int m_AuxAxisSyncOpen;//同步轴 第8位为实时加载标志
    };

    typedef struct {
      JointsPos m_Joint; // 关节位置
      AuxPos m_AuxJoint; // 附加轴位置
    } AXISPOS;

    /* 笛卡尔位置 CartPos */
    typedef struct {
      double cart[6];
      unsigned int m_Mode;
    } RobotCartPos;

    typedef struct {
      RobotCartPos m_CartPos; // xyzabc+mode
      AuxPos m_AuxPos;
    } CARTPOS;

    typedef struct {
      char m_Name[32];     /* 机器人名称 */
      char m_RobotSNo[32]; /* 机器人序列号 */
    } RobotInfoStruct;

    typedef struct {
      char m_ElecBoxName[32]; /* 控制箱型号 */
      char m_ElecBoxSNo[32];  /* 控制箱序列号 */
    } ElecBoxInfoStruct;

    typedef enum {
      MANUAL = 1,
      AUTO,
      AUTO_EXT,
      START, // 4
      V_INCREASE,
      V_DECREASE,
      POSITIVE1,
      POSITIVE2,
      POSITIVE3,
      POSITIVE4,
      POSITIVE5,
      POSITIVE6,
      NEGATIVE1,
      NEGATIVE2,
      NEGATIVE3,
      NEGATIVE4,
      NEGATIVE5,
      NEGATIVE6,
      ND2,
      WIREOUT,//送丝
      WIREBACK,//抽丝
      WIREGAS,//送气
      STOP,       // 23
      MSTOP = 100 // 100,点动停止

    } MODEKEY;
    typedef struct {
      double m_X;
      double m_Y;
      double m_Z;
      double m_A;
      double m_B;
      double m_C;
      unsigned int m_Mode;
    } CartPos;

    /* 固定角 */
    typedef Vector3 Rpy;

    /*新增工具坐标和用户坐标*/
    typedef struct {
      bool Fixed;
      double X;
      double Y;
      double Z;
      double A;
      double B;
      double C;
    } Tool;

    typedef struct {
      double m_X;
      double m_Y;
      double m_Z;
      double m_A;
      double m_B;
      double m_C;
      char m_BaseRef[32];

    } CartRefSys;

     // DH参数设置 新增，20230726，非标版本
    // 定义关节类型
    typedef enum {
      eLinear = 1, // 移动关节
      eRotary      // 转动关节
    } JOINTTYPE;

    // 定义DH参数
    typedef struct {
      JOINTTYPE m_JointType; // 关节类型
      double m_Alpha;        // 连杆转角
      double m_A;            // 连杆两端关节轴的公垂线长度
      double m_D;     // 连杆偏距（相邻连杆公共轴线方向的距离）
      double m_Theta; // 关节角
    } RobotDHParam;
    //RobotDHParam cmdDHParam[MAX_JOINT_NUM + 1]; // DH参数

    //绝对位置点动结构体增加，20231103
    typedef enum {
      eAxisPos = 1,
      eCartPos,
      eRobotAxisPos,
      eRobotCartPos,
      //eAuxAxisPos,
      //eAxisPosExt,
      //eCartPosExt,
      //eRobotAxisPosExt,
      //eRobotCartPosExt,
      //eAuxAxisPosExt,
      //eArcMultiPos,
      //eCartPosEE // 2014.10.29 新增外部扩展点类型
    } HMIPOSTYPE;

    typedef enum {
      ePTP = 0,
      eLine = 1
    } JogPositionType;

    typedef struct {
      CartPos m_CartPos; // xyzabc+mode
      AuxPos m_AuxPos;   // 附加轴信息
      JointsPos m_Joint; // 关节部分
      POSTYPE m_postyp;  // 位置类型
    } RcPos;

    // 用户程序位置数据
    typedef struct {
      HMIPOSTYPE m_PosType;
      int m_Port;
      RcPos m_Pos;
    } HMIPos;

    typedef struct {
      char m_ToolName[32];
      char m_RefName[32];
    } ToolRefName;

    //新增SDK模拟运动命令中使用数据类型 20231115
    //圆滑
    typedef enum {
      eOVLABS = 1, // 绝对值给定
      eOVLREL      // 百分比给定
    } OVERLAPTYPE;

    typedef struct {
      double m_Zone_tcp;
      double m_Zone_ori;
      double m_Zone_leax;
      double m_Zone_reax;
      double m_Percent;     // %百分比
      OVERLAPTYPE m_OveTyp; // 圆滑给定方式
    } SOverlap;

    //SDK专用圆滑结构体
    typedef struct {
      double m_tcp_distance;
      double m_Percent;     // %百分比
      OVERLAPTYPE m_OveTyp; // 圆滑给定方式
    } SDKOverlap;

    //姿态参数
    typedef enum {
      eOriNULL = 0,
      eVar = 1, // 根据示教的姿态信息进行四元数姿态插补
      eConst,   // 始终保持起始位置的姿态
      eCircle,  // 与圆平面的法相保持固定姿态进行圆的插补
      eWrist,   // 腕关节
      eCirPass  // 过圆弧中间点姿态
    } ORITYPE;

    //PTP结构体
    typedef struct {
      HMIPos m_EndPos;
      int m_Dyn; // 只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
      SOverlap m_Ovl; // NULL参数值都为-1，类型枚举为0
      char m_ToolName[32];
      char m_RefName[32];
    } SDKInstructPTP;

    //Lin结构体
    typedef struct {
      HMIPos m_EndPos;
      int m_Dyn; // 只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
      SOverlap m_Ovl; // NULL参数值都为-1，类型枚举为0
      ORITYPE m_Ori;
      char m_ToolName[32];
      char m_RefName[32];
    } SDKInstructLin;

    //Circ结构体
    typedef struct {
      HMIPos m_StartPos;      // 起点，上一行运动指令的目标点
      char m_StartTool[32];   // 上一行运动指令的工具
      char m_StartRefSys[32]; // 上一行运动指令的坐标系
      HMIPos m_EndPos;
      HMIPos m_HelpPos;
      int m_Dyn; // 只可以设置位置速度，其他参数利用Dyn语句设置
      SOverlap m_Ovl; // NULL参数值都为-1，类型枚举为0
      ORITYPE m_Ori;
      char m_ToolName[32];
      char m_RefName[32];
      double m_Angle;
    } SDKInstructCirc;

    //Dyn结构体
    typedef struct {
      SDynamicPercent m_DynPerc; // 三个时间默认值为-1
    } SDKInstructDyn;

    //Waittime结构体
    typedef struct {
      int m_WaitTime;
    } SDKInstructWaitTime;

    //Arcon结构体
    typedef struct {
      int m_ScratchArcFlag;        // 擦线起弧标志，1是启用或x，2是Y
      double m_ScratchMaxDistance; // 擦线距离
      double m_ScratchStepLength;  // 擦线步长，每次移动距离
      double m_ReturnSpeed;        // 擦线返回速度

      int m_ScratchSegType; // 1是Lin，2是Circ，CircAngle
      HMIPos m_EndPos;      // 擦线方向上末端点
      HMIPos m_HelpPos;
      char m_ToolName[32];      // 工具
      char m_RefSysName[32];   // 坐标系
    } ScratchSDKArcData;
    typedef struct {
      int m_Valid;                     // 是否有再起弧
      int m_DetectTime;                // 起弧失败检测时间
      int m_RetryTimes;                // 起弧失败后，重试的次数
      int m_RetractTime;               // 焊丝回抽时间
      int m_RetractWaitTime;           // 回抽等待时间
      double m_CurrentInc;             // 再起弧电流增量
      double m_VoltageInc;             // 再起弧电压增量
      ScratchSDKArcData m_ScratchArcData; // 擦线起弧设置参数
    } SDKArcRetryData;
    typedef struct {
      int m_PreflowTime;     // 预送气时间
      int m_PreArcingTime;   // 预起弧时间
      int m_PreWireFeedTime; // 预送丝时间
      double m_ArcOnCurrent; // 起弧电流
      double m_ArcOnVoltage; // 起弧电压
      int m_ArcOnWaitTime;   // 等待焊机起弧成功的时间//起弧时间

      //新增2025.04.07
      int m_ArcMode;               //焊接模式
      int m_ArcArgonFlag;          //氩弧焊标志
      int m_WeaveSyncFlag;         //摆弧同步输出电流送丝标志
      int m_ArcWeldCurrentBase;    //焊接电流基值
      int m_ArcDutyCycle;          //占空比
      int m_ArcFrequency;          //脉冲频率
      double m_ArcWireSpeed;       //峰值送丝速度
      double m_ArcWireSpeedBase;   //基值送丝速度
      int m_ArcRampTime;           //缓升时间
      int m_ArcDescentTime;        //缓降时间
	  //新增2025.05.16
      int m_ArcOnDelayWireTime;    //延迟送丝时间
      double m_ArcWireRetractLength;//焊丝回退距离

      int m_DecayTime;         // 渐变时间
      double m_ArcSetCurrent;  // 焊接电流
      double m_ArcSetVoltage;  // 焊接电压
      double m_ArcSetSpeed;    // 机器人焊接速度 2016.5.25
      double m_ArcSetEndSpeed; // 机器人焊接结束速度2021.9.23

      SDKArcRetryData m_ArcRetryData; // 再起弧参数
      double m_BackDistance;      //起弧回退距离 //新增2024.7.8

    } SDKInstructArcOn;

    //将ArcOn结构体分组给接口使用，按照示教器规则
    //起弧变量
    typedef struct {
      int m_PreflowTime;     // 预送气时间
      int m_PreArcingTime;   // 预起弧时间
      int m_PreWireFeedTime; // 预送丝时间
      double m_ArcOnCurrent; // 起弧电流
      double m_ArcOnVoltage; // 起弧电压
      int m_ArcOnWaitTime;   // 等待焊机起弧成功的时间//起弧时间
    } SDKArcOnVauleStruct;
    //焊接变量
    typedef struct {
      double m_ArcSetCurrent;  // 焊接电流
      double m_ArcSetVoltage;  // 焊接电压
      double m_ArcSetSpeed;    // 机器人焊接速度 2016.5.25
      double m_ArcSetEndSpeed; // 机器人焊接结束速度2021.9.23

      //新增2025.04.07
      int m_ArcMode;               //焊接模式
      int m_ArcArgonFlag;          //氩弧焊标志
      int m_WeaveSyncFlag;         //摆弧同步输出电流送丝标志
      int m_ArcWeldCurrentBase;    //焊接电流基值
      int m_ArcDutyCycle;          //占空比
      int m_ArcFrequency;          //脉冲频率
      double m_ArcWireSpeed;       //峰值送丝速度
      double m_ArcWireSpeedBase;   //基值送丝速度
      int m_ArcRampTime;           //缓升时间
      int m_ArcDescentTime;        //缓降时间
	  //新增2025.05.16
      int m_ArcOnDelayWireTime;    //延迟送丝时间
      double m_ArcWireRetractLength;//焊丝回退距离
    } SDKArcSetVauleStruct;

    //Arcoff结构体
    typedef struct {
      int m_BurnbackTime;        // 回烧时间
      int m_PostflowTime;        // 滞后送气时间
      double m_ArcOffCurrent;    // 收弧电流
      double m_ArcOffVoltage;    // 收弧电压
      int m_ArcOffWaitTime;      // 等待焊机息弧成功的时间
      int m_DecayTime;           // 渐变时间
      int m_StickCheckDelayTime; // 默认粘丝检测延迟时间，单位秒
      //新增2025.04.07
      int m_ArcMode;               //焊接模式
    } SDKInstructArcOff;
    // Arcset结构体
    typedef struct {
      double m_WeldCurrent; // 焊接电流
      double m_WeldVoltage; // 焊接电压
      double m_WeldSpeed;   // 机器人焊接速度
      double m_EndSpeed;    // 机器人焊接结束速度2021.9.23
      int m_StartEnd;       // 2015.11.6 渐变，START:0;END:1;NULL:-1
      double m_Length;      // 2015.11.6 渐变
      //新增2025.04.07
      int m_ArcMode;               //焊接模式
      int m_ArcArgonFlag;          //氩弧焊标志
      int m_WeaveSyncFlag;         //摆弧同步输出电流送丝标志
      int m_ArcWeldCurrentBase;    //焊接电流基值
      int m_ArcDutyCycle;          //占空比
      int m_ArcFrequency;          //脉冲频率
      double m_ArcWireSpeed;       //峰值送丝速度
      double m_ArcWireSpeedBase;   //基值送丝速度
      int m_ArcRampTime;           //缓升时间
      int m_ArcDescentTime;        //缓降时间
	  //新增2025.05.16
      int m_ArcOnDelayWireTime;    //延迟送丝时间
      double m_ArcWireRetractLength;//焊丝回退距离

    } SDKInstructArcSet;

    //WLin
    typedef enum {
      eTCPWeave = 0,     // 6,原eGeometricWeave，更改名字
      eWristWeave,         // 56关节参与的摆动   // 取消 2015.11.10
      e45JointWeave,      // 456关节参与的摆动  // 取消 2015.11.10
      //e123JointWeave,      // 123关节参与的摆动  // 取消 2015.11.10
    } WeaveType;
    typedef enum {
      eNoWeave = 0,     //
      //eTriangle,        // qi yong；示教器上无该类型20240316
      //eDoubleTriangle,  //
      //eTrapezoid,       //
      //eDoubleTrapezoid, //
      eSin = 5,             //
      eSinFreq,
      eSpiral,         //新增
      eObliqueTriangle,  // 斜椭圆摆
      eSpaceTriangle, //新增
      eLTriangle,        //新增
      eBackForward,
      eConstPoint,  //20220701新增
      eSpiralFreq,               // 给定频率的螺旋线摆    2024.03.14
      eObliqueTriangleFreq,      // 给定频率的斜三角摆    2024.03.14
      eSpaceTriangleFreq,        // 给定频率的空间三角摆  2024.03.14
      eLTriangleFreq,            // 给定频率的空间L型摆   2024.03.14
      eBackForwordFreq,          // 给定频率的前后摆      2024.03.14
      eHalfSin,                  // 涂敷用半边sin摆       2025.06.23
    } WeaveShape;
    typedef struct {
      int m_WeaveNULL; //示教器给的摆弧变量是否为NULL，1为NULL
      WeaveType m_WeaveType;   // 摆动类型：平面摆，关节摆等
      WeaveShape m_WeaveShape; // 摆动形状：无摆动，三角摆等
      double m_WeaveLength;    // 摆动长度
      double m_WeaveAmplitude; // 摆动幅值
      double m_Weavehigh;      // 平面摆动的高度
      int m_QPauseTime;        // 摆弧1/4处停留时间，需界面传入
      int m_ThreeQPauseTime;   // 摆弧3/4处停留时间，需界面传入
      double m_SwingDirection; // 新增，摆弧倾斜角（对sin摆无效）
      double m_WeaveAngle;     // 新增，摆弧平面倾斜角
      double m_SpaceAngle; // 新增，空间摆弧夹角（对L型摆和空间三角摆有效）
      int m_PauseTime_1; // 新增，1/4处停留时间
      int m_PauseTime_2; // 新增，2/4处停留时间
      int m_PauseTime_3; // 新增，3/4处停留时间
      int m_PauseTime_4; // 新增，4/4处停留时间
      int m_PauseContinue; ////新增，2021.9.23，摆弧连续，0-完全停止，1-移动模式
      double m_EndLength;  // 新增，结束长度（sin摆有效）
      double m_EndWidth;   // 新增，结束长度（sin摆有效）
      double m_CenterHigh; // 新增，结束长度（sin摆有效）
      //2025.04.24 新增
      double m_StartWeaveAmplitude;//新增，起始摆弧幅值，2025.04.24
      double m_AStrLength;//新增，起始摆弧渐变长度，默认值给0，
      double m_AEndLength;//新增，结束摆弧渐变长度，默认值给0，
      double m_RightAmplitude; // 新增，右侧摆弧宽度，2025.07.02
    } WeaveParam;//WEAVEDATA

    // 电弧跟踪新增参数20240311
    typedef struct
    {
        //横向
        double m_L_MinComp;     //横向最小补偿（单周期）
        double m_L_MaxComp;     //横向最大补偿（单周期）
        double m_L_TotalMaxComp;//横向最大补偿（总）
        double m_L_RightComp;   //横向不对称调整系数
        double m_L_Comp6;
        double m_L_Comp5;
        double m_L_Comp4;
        double m_L_Comp3;
        double m_L_Comp2;
        double m_L_Comp1;

        //纵向
        double m_V_MinComp;     //纵向最小补偿（单周期）
        double m_V_MaxComp;     //纵向最大补偿（单周期）
        double m_V_TotalMaxComp;//纵向最大补偿（总）
        double m_V_TopComp;     //纵向不对称调整系数
        double m_V_Comp6;       //是否保持跟踪
        double m_V_Comp5;
        double m_V_Comp4;
        double m_V_Comp3;
        double m_V_Comp2;
        double m_V_Comp1;

    } SeamTrackComp;


    typedef struct {
      int m_TrackNULL;
      int m_flag;               // 是否做电弧跟踪
      double Kpl;               // 横向比例增益  不是比例增益百分比
      double m_LeftCoefficient; // add 2016.2.17;
      double m_RightCoefficient;
      double Kpv;              // 纵向比例增益  不是比例增益百分比
      double Iref;             // 电流参考值
      int carry;               // 是否加到下一个周期中
                               /*
                                       int m_Flag;            // 是否做电弧跟踪----保留
                                       double m_Iref;			// 电流参考值  ---弃用
                                       int m_Carry;              // 是否加到下一个周期中---保留 */
                               // 新增
      int m_LateralBeginCycle; // 横向纠偏开始周期（>=3）
      // double m_Kpl;				// 横向比例增益
      // 不是比例增益百分比----保留
      int m_VerticalModeFlag; // 为0则是常数模式,为1是反馈模式//CONST,SAMPLE
      double m_Iref_Vertical;     // 纵向纠偏基准电流（常数模式）
      int m_VerticalBeginCycle;   // 纵向取样开始周期1~999
      int m_VerticalSustainCycle; // 纵向取样持续周期1~999
      double m_VerticalLength;    // 纵向周期长度
      // double m_Kpv;				// 纵向比例增益
      // 不是比例增益百分比---保留

      int m_TypeofTimeOrDist;      // 1 == time,0 == distance
      unsigned int m_TimeInterval; // 100-500ms，默认200ms
      unsigned int m_DistInterval; // 距离 1-30mm,默认：2mm

      double m_YBias; // only for lasertrack,20162.25
      double m_ZBias;

      SeamTrackComp m_SeamTrackComp;

    } SeamTrackParam;

    typedef struct {
      HMIPos m_EndPos;
      int m_Dyn; // 只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
      SOverlap m_Ovl; // NULL参数值都为-1，类型枚举为0
      ORITYPE m_Ori;
      WeaveParam m_Weave;          // 摆弧参数
      SeamTrackParam m_SeamTrack;  // 电弧跟踪
      //SDKInstructArcOn m_ArcOnSet; // 起弧和焊接参数 2016.5.25  2024.7.1作废
      char m_ToolName[32];
      char m_RefName[32];
    } SDKInstructWLin;

    //WCirc
    typedef struct {
      HMIPos m_StartPos;      // 起点 2023.08.15
      char m_StartTool[32];   /// 工具
      char m_StartRefSys[32]; /// 坐标系
      HMIPos m_EndPos;
      HMIPos m_HelpPos;
      int m_Dyn; // 只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
      SOverlap m_Ovl; // NULL参数值都为-1，类型枚举为0
      ORITYPE m_Ori;
      WeaveParam m_Weave;          // 摆弧参数
      SeamTrackParam m_SeamTrack;  // 电弧跟踪
      //SDKInstructArcOn m_ArcOnSet; // 起弧和焊接参数 2016.5.25   2024.7.1作废
      char m_ToolName[32];
      char m_RefName[32];
      double m_Angle;
    } SDKInstructWCirc;

    //ARCMODE，2024.7.11新增
    typedef struct
    {
        int m_Mode;
    }SDKInstructArcMode;


    typedef struct {
      COMMUNICATIONCOMMAND cmd; // 命令
      char buffer[MAX_SIZE_CHAR];
    } SDKProgramBuffer;


    typedef struct {
      double m_SetCurrent;
      double m_SetVoltage;
      double m_RealCurrent;
      double m_RealVoltage;
    } WeldingParamFeedBack;

    //新增导轨转换输入20231221
    typedef struct
    {
        CartPos m_CartPos;
        char m_ToolName[32];
    }SDKInstructRailInfo;


    //新增导轨转换输出20231221
    typedef struct
    {
        unsigned int mode;
        double aux0;
    }RailInfo;


    //新增运动学正逆解结构体20240229
    typedef struct
    {
        double m_X;
        double m_Y;
        double m_Z;
        double m_A;
        double m_B;
        double m_C;
    }CartSys;

    typedef struct
    {
        CartSys xyzabc;
        unsigned int modetmp;         // 模式，0或4
        AuxPos m_AuxPos;
        char m_ToolName[32];
        char m_RefName[32];
    }SDKInstructInvKinematics;

    typedef struct
    {
        CartSys xyzabc;
        unsigned int modetmp;         // 模式，0或4
    }KinematicsInfo;

    typedef struct
    {
        JointsPos joint; // 关节部分
        char m_ToolName[32];
        char m_RefName[32];
    }SDKInstructKinematics;


    //20240304添加焊接相关变量
    //工具类型
    typedef enum
    {
        eInternTool,
        eExternTool
    }EXTERNTOOLTYPE;

    typedef struct
    {
        int m_Indepent;// 0 = No,1 = yes
        int m_LineNum;                                //行号
        char m_ToolName[32];                //工具名称
        EXTERNTOOLTYPE m_ToolType;       //内部工具还是外部工具
        HomoMatrix m_Tool;                    //工具转换矩阵
        int m_Port;                                        //外部工具端口号
        int m_id;                                        //语句的序号
    }CmdTool;

    //坐标系类型定义
    typedef enum
    {
        eRefTypeJoints = 1,
        eRefTypeFixed,
        eRefTypeWorld,
        eRefTypeBase,
        eRefTypeExt,
        eRefTypeVar,
        eRefTypeExSys,
        eRefTypeTool,
        eRefTypeFlange,
        eRefTypeTTS
    }REFSYSTYPE;
    typedef struct
    {
        int m_LineNum;             //行号
        char m_RefSysName[32];    //坐标系名称
        REFSYSTYPE m_RefSysType;  //坐标系类型
        CartFrame m_BaseRef;      //坐标系的参考坐标系值相对于world
        CartFrame m_RefSys;       //坐标系相对于参考坐标系的值
        double m_FollowDistance;  //坐标系跟踪距离
        int m_FollowMode;         //坐标系跟踪模式
        double m_ExitTime;        //坐标系退出跟踪时间
        int m_Port;               //可变坐标系的索引
        int m_id;                 //语句的序号
        double m_TrackAct;        //新增:跟踪加速度，默认值1008，流水线增加模式4，s型速度跟踪:2025.04.25
    }CmdRefSys;

    typedef struct
    {
        int m_ScratchArcFlag;  //擦线起弧标志，1是启用或X，2是Y
        double m_ScratchMaxDistance;  //擦线距离
        double m_ScratchStepLength;   //擦线步长，每次移动距离
        double m_ReturnSpeed;  //擦线返回速度

        int m_ScratchSegType;  //1是Lin，2是Circ，CircAngle
        HMIPos m_EndPos;  //擦线方向上末端点
        HMIPos m_HelpPos;
        CmdTool m_Tool;             //工具
        CmdRefSys m_RefSys;         //坐标系

    }ScratchArcData;

    typedef struct
    {
        int m_Valid;        // 是否有再起弧
        int m_DetectTime;   // 起弧失败检测时间
        int m_RetryTimes;   // 起弧失败后，重试的次数
        int m_RetractTime;  // 焊丝回抽时间
        int m_RetractWaitTime;  // 回抽等待时间
        double m_CurrentInc;    // 再起弧电流增量
        double m_VoltageInc;    // 再起弧电压增量
        ScratchArcData m_ScratchArcData;  //擦线起弧设置参数
    }HMIArcRetryData;  // 2016.5.25

    typedef struct
    {
        int m_DetectTime;   // 起弧失败检测时间
        int m_RetryTimes;   // 起弧失败后，重试的次数
        int m_RetractTime;  // 焊丝回抽时间
        int m_RetractWaitTime;  // 回抽等待时间
        double m_CurrentInc;    // 再起弧电流增量
        double m_VoltageInc;    // 再起弧电压增量

        //int m_ScratchArcFlag;  //擦线起弧标志，1是启用
        char m_ScratchArcFlag[32];//示教器接收需要为字符型，上面禁用
        double m_ScratchMaxDistance;  //擦线距离
        double m_ScratchStepLength;   //擦线步长，每次移动距离
        double m_ReturnSpeed;  //擦线返回速度

    }ArcRetryData;  // 20240304，根据示教器显示新增

    //间断焊变量seg
    typedef struct
    {
        double m_StartArcOnCurrent;//开始起弧电流
        double m_StartArcOnVoltage;//开始起弧电压
        int m_DecaySeg;//渐变段数
        int m_PreflowTime;//预送气时间
        double m_ArcOnCurrent;		// 起弧电流
        double m_ArcOnVoltage;		// 起弧电压
        int m_ArcOnWaitTime;		// 等待焊机起弧成功的时间
        int m_ArcOnDecayTime;              //起弧渐变时间
        //
        //int m_Valid;        // 是否有再起弧
        int m_FailDetectTime;   // 起弧失败检测时间
        int m_RetryTimes;   // 起弧失败后，重试的次数
        int m_RetractTime;  // 焊丝回抽时间
        int m_RetractWaitTime;  // 回抽等待时间
        double m_CurrentInc;    // 再起弧电流增量
        double m_VoltageInc;    // 再起弧电压增量
        //
        int m_BurnbackTime;          // 回烧时间
        int m_PostflowTime;          // 滞后送气时间
        double m_ArcOffCurrent;      // 收弧电流
        double m_ArcOffVoltage;      // 收弧电压
        int m_ArcOffWaitTime;	// 等待焊机息弧成功的时间
        int m_ArcOffDecayTime;        //渐变时间
        int m_StickCheckDelayTime; //粘丝检测延时时间
    }ArcSegData;

    typedef double ArcConfigData[14];

    typedef struct
    {
        WeaveShape m_waveShape;
        WeaveType m_waveType;
        float m_weaveLength;    //
        float m_weaveWidth; // =m_WeaveAmplitude//2014.2.25
        /*float m_weaveAngel;
        unsigned int m_pauseTime_L;
        unsigned int m_pauseTime_R;*/
        double m_SwingDirection; //新增，摆弧倾斜角（对sin摆无效）
        double m_WeaveAngle; //新增，摆弧平面倾斜角
        double m_SpaceAngle;//新增，空间摆弧夹角（对L型摆和空间三角摆有效）
        unsigned int m_PauseTime_1;//新增，1/4处停留时间
        unsigned int m_PauseTime_2;//新增，2/4处停留时间
        unsigned int m_PauseTime_3;//新增，3/4处停留时间
        unsigned int m_PauseTime_4;//新增，4/4处停留时间
        int m_PauseContinue;    //停留连续 //add 2021.9.23
        double m_EndLength;     //结束长度
        double m_EndWidth;      //结束宽度
        double m_CenterHigh;    //中心高度
        //2025.04.24 新增
        double m_StartWeaveAmplitude;//新增，起始摆弧幅值，2025.04.24
        double m_AStrLength;//新增，起始摆弧渐变长度，默认值给0，
        double m_AEndLength;//新增，结束摆弧渐变长度，默认值给0，
        double m_RightAmplitude; // 新增，右侧摆弧宽度，2025.07.02
    }WeaveData;


    //CD,寻位
    typedef enum
    {
        eCD_X_Positive = 1,//to X+ direction
        eCD_X_Negative,//to X- direction
        eCD_Y_Positive,
        eCD_Y_Negative,
        eCD_Z_Positive,
        eCD_Z_Negative
    }CDDirection;
    typedef struct
    {
        int m_Calibrate;            //if have been Calibrated ,value is true; otherwise false
        CartPos m_CartPos;          // xyzabc
        AuxPos m_AuxPos;            //
        CartPos m_CurrentPos;
        CDDirection m_Direction;
        char m_RefName[32];
    }CDData;

    //新增2024.7.2
    typedef enum
    {
        e1D_CR = 1,
        e2D_CR = 2,
        e3D_CR = 3,
        e6D_CR = 6,
        e2D1R_CR = 7,
        e3D1R_CR = 8,
        eCircle_CR = 9,
        e3D3R_CR = 10
    } CORRECTTYPE;

    //CDS新增2024.7.3
    typedef struct
    {
        CORRECTTYPE m_Dimension;
        int m_temp1;
        int m_temp2;
        double m_temp3;
        double m_temp4;
        double m_temp5;
        CDData m_CD[7];
    } ArcCDSData;

    //PAT，接触寻位
    typedef enum
    {
        eSingleTouch,
        eDoubleTouch
    }TouchMode;

    typedef struct
    {
        TouchMode m_TouchMode;
        double m_Tolerance;
        double m_SearchDistance;
        double m_SearchSpeed;
        unsigned int m_ReturnAcc;
        unsigned int m_ReturnSpeed;
        double m_ReturnDistance;//2024.7.12
    }PAT;
    //TRACKDATA，跟踪变量
    typedef struct
    {
        /*float m_horizontalGain;    // Kpl
        float m_verticalGain; // Kpv
        float m_iref;//
        bool m_carryon;      //*/
        int m_LateralBeginCycle;	//横向纠偏开始周期（>=3）
        double Kpl;				// 横向比例增益  不是比例增益百分比----保留
        double m_LeftCoefficient;//add 2016.2.17;zuo bian mian ji xi shu
        double m_RightCoefficient;
        int m_VerticalModeFlag; //为0则是常数模式,为1是反馈模式
        double m_Iref_Vertical;		//纵向纠偏基准电流（常数模式）
        int m_VerticalBeginCycle;	//纵向取样开始周期1~999
        int m_VerticalSustainCycle;	//纵向取样持续周期1~999
        double m_VerticalLength;		//纵向周期长度
        double Kpv;				// 纵向比例增益  不是比例增益百分比---保留

        int m_TypeofTimeOrDist;//1 == time,0 == distance
        unsigned int m_TimeInterval;//100-500ms，默认200ms
        unsigned int m_DistInterval;//距离 1-30mm,默认：2mm

        double m_YBias;//only for lasertrack,20162.25
        double m_ZBias;

        //2024.7.8新增
        double L_MinComp;//横向最小补偿
        double L_MaxComp;//横向最大补偿
        double L_TotalMaxComp;//横向最大补偿总
        double L_RightComp;//横向不对称调整系数
        double Temp_L[6];
        double V_MinComp;//横向最小补偿
        double V_MaxComp;//横向最大补偿
        double V_TotalMaxComp;//横向最大补偿总
        double V_TopComp;//横向不对称调整系数
        double Temp_V[6];

    }TrackData;

    typedef struct
    {
        double m_EndSpeed;				//最大速度
        double m_EndCurrent;			//最大电流
        double m_YBias;                 //Y方向偏置距离
        double m_ZBias;                 //Z方向偏置距离
        double m_MaxWidth;		        //最大坡口宽度
        char m_AdapLength[32];          //宽度自适应开关
        double m_StandardWidth;         //标准坡口宽度 20240708
        double m_WidthRatio;            //宽度比例
    }LaserTrackData;//根据示教器显示新增20240305

    //plaserfilter
    typedef enum
    {
        eNoFilter,
        eMedianFilter,              //中值滤波（默认）
        eMeanFilter,                //均值滤波
        eGaussFilter                //高斯滤波
    }FilterType;

    //点激光滤波方式（返回点的类型）
    typedef enum
    {
        eMax,                 //第一处极大值
        eMin,                 //第一处极小值
        eUpLeft,              //第一处上升沿左侧
        eUpRight,             //第一处上升沿右侧
        eDownLeft,            //第一处下降沿左侧
        eDownRight,           //第一处下降沿右侧
    }PointType;

    //点激光滤波参数
    typedef struct
    {
        FilterType m_FilterType;                                //滤波器类型
        PointType m_PointType;                                  //目标点类型
        int m_FilterParam;                                      //滤波器参数
        int m_FilterLength;             //原Width，20240316     //滤波长度
        double m_Featurethreshold;                              //特征点阈值
    } PLaserFilter;

    //laserarcondata
    typedef struct
    {
        int m_PreFlowTime;   //预送气时间
        int m_PreWireTime;   //预送丝时间
        int m_DelayLightTime;//延迟开光时间
        double m_WireSpeed;     //送丝速度
        double m_Temp4;
        double m_Temp3;
        double m_Temp2;
        double m_Temp1;
    } LaserArcOnData;
    typedef struct
    {
        double m_LaserPower;    //激光功率
        int m_PWM;              //占空比
        int m_Frequency;        //频率
        double m_WireSpeed;     //送丝速度
        double m_Temp4;
        double m_Temp3;
        double m_Temp2;
        double m_Temp1;
    }LaserArcData;

    typedef struct
    {
        LaserArcOnData m_LaserOnData;
        LaserArcData m_LaserData;
    } LaserArcOnNewData;

    typedef struct
    {
        int m_DelayFlowTime;        //滞后送气时间
        int m_PreCloseWireTime;     //提前关丝时间
        int m_WireRetractTime;      //退丝时间
        int m_WireReFeedTime;       //退丝再进时间
        double m_WireSpeed;         //送丝速度
        double m_Temp5;
        double m_Temp4;
        double m_Temp3;
        double m_Temp2;
        double m_Temp1;
    }LaserArcOffData;

    //ArcOnData，起弧变量
    typedef struct
    {
        unsigned int m_preFlowtime;//预送气时间
        float m_arconCurrent;     //起弧电流
        float m_arconVoltage;    //起弧电压
        unsigned int m_arconTime;  //起弧时间
        unsigned int m_BeforeArcOn;//add 2021.1.27，预起弧时间
        unsigned int m_BeforeWireOut;//add 2021.1.27，预送丝时间
    }ArcOnData;
    //ArcOffData，收弧变量
    typedef struct
    {
        unsigned int m_burnBacktime;//回烧时间
        unsigned int m_postFlowtime;//滞后送气时间
        float m_arcoffCurrent;//收弧电流
        float m_arcoffVoltage;//收弧电压
        unsigned int m_arcoffTime;//收弧时间
		int m_ArcMode;//焊接模式 20250407
    }ArcOffData;
    //ArcData
    typedef struct
    {
        float m_weldVoltage;//焊接电压
        float m_weldCurrent;//焊接电流
        float m_weldSpeed;//add 2015.4.2，焊接速度
        double m_endSpeed;//add 2021.9.23，焊接结束速度
        //新增2025.04.07
        int m_ArcMode;               //焊接模式
        int m_ArcArgonFlag;          //氩弧焊标志
        int m_WeaveSyncFlag;         //摆弧同步输出电流送丝标志
        int m_ArcWeldCurrentBase;    //焊接电流基值
        int m_ArcDutyCycle;          //占空比
        int m_ArcFrequency;          //脉冲频率
        double m_ArcWireSpeed;       //峰值送丝速度
        double m_ArcWireSpeedBase;   //基值送丝速度
        int m_ArcRampTime;           //缓升时间
        int m_ArcDescentTime;        //缓降时间
        //新增2025.05.16
        int m_ArcOnDelayWireTime;    //延迟送丝时间
        double m_ArcWireRetractLength;//焊丝回退距离
    }ArcData;

    typedef struct
    {
        JointsPos m_JointsPos;
        AuxPos m_AuxPos;
    }AxisPos;
    typedef AxisPos AxisDist;

    typedef struct
    {
        double m_X;
        double m_Y;
        double m_Z;
        double m_A;
        double m_B;
        double m_C;
        AuxPos m_AuxPos;
        int m_Mode;
    }CartDist;

    typedef AuxPos AuxAxisPos;

    typedef struct
    {
        //double m_PosDist;
        //double m_OriDist;
        //double m_LinAxDist;
        //double m_RotAxDist;
        double m_Zone_tcp;//add 2018.4.24
        double m_Zone_ori;
        double m_Zone_leax;
        double m_Zone_reax;
    }OverlapABS;

    //加速度类型
    typedef enum
    {
        eTran,		   // T型加速度
        eSShape,       // S型速度
        eSine,         // Sin型速度
        eSShape_new    // 新的7段s型速度规划
    } RampType;

    typedef struct
    {
        char			m_PalName[48];	// 码垛变量名称
        unsigned int 	m_LayerNum;		// 层号
        unsigned int	m_PartNum;		// 件号
        CartPos	 		m_CartPos;   	// xyzabc+mode
        AuxPos 			m_AuxPos;     	// joint
    }PalletPosition;

    typedef struct
    {
        double m_BendAngle;           //折弯角度
        double m_BendVel;             //折弯速度
        double m_BendAccTime;         //折弯加速度
        double m_BendDelayTime;       //折弯延迟时间
        double m_BendThickness;       //板厚
    }BendTargetData;

    typedef struct
    {
        double m_SynDistance;      //同步距离
        double m_SynVel;           //同步速度
        double m_SynAccTime;       //同步加速时间
        double m_SynDelayTime;     //同步延迟时间
    }BendSynTargetData;
    typedef enum
    {
        ePolishCircle = 0,      //
        ePolishEllipse,
        ePolishSin          //
    }PolishType;

    typedef struct
    {
        PolishType m_PolishShape;  //打磨类型，枚举类型eCircle, eEllipse, eSin
        unsigned int m_PolishNumber;  //打磨个数，>=1
        double m_Radius;  //圆半径
        double m_SemimajorAxle;  //椭圆长半轴
        double m_SemiminorAxle;  //椭圆短半轴
        double m_Angle;  //椭圆倾斜角 0-180
        double m_Amplitude;  //sin幅值
        double m_PressAmount;  //打磨下压量 默认0
    }PolishData;

    typedef struct
    {
        //质量 Weight
        double Weight;
        //质心 x y z
        double x;
        double y;
        double z;
        //转动惯量
        double ixx;
        double iyy;
        double izz;
        double ixy;
        double ixz;
        double iyz;
    }PayloadData;

    //语句控制增加，补充新增语句结构体
    typedef struct
    {
        int m_DOPort;     //端口号
        bool m_DOValue;   //值
    }SDKInstructDOSet;

    typedef struct
    {
        int m_DOPort;           //端口号
        bool m_DOValue;         //值
        int m_StartEnd;         //方向,0为start，1为end
        double m_Distance;      //距离
        int m_MemberType;       //轴/XYZ，0为null，1~6为A1~A6
    }SDKInstructDOSetSyncPath;

    typedef struct
    {
        int m_DOPort;           //端口号
        bool m_DOValue;         //值
        int m_StartEnd;         //方向,0为start，1为end
        int m_Time;             //时间
    }SDKInstructDOSetSyncTime;

    typedef struct
    {
        int m_AOPort;
        int m_AOValue;
    }SDKInstructAOSet;

    typedef struct
    {
        int m_AOPort;
        int m_AOValue;
        int m_StartEnd;
        double m_Distance;
        int m_MemberType;
    }SDKInstructAOSetSyncPath;

    typedef struct
    {
        int m_AOPort;
        int m_AOValue;
        int m_StartEnd;
        int m_Time;
    }SDKInstructAOSetSyncTime;

    typedef struct
    {
        int m_DOPort;
        bool m_DOValue;
        int m_PulseTime;
    }SDKInstructDOPulse;

    typedef struct
    {
        int m_DOPort;
        bool m_DOValue;
        int m_PulseTime;
        int m_StartEnd;
        double m_Distance;
    }SDKInstructDOPulseSyncPath;

    typedef struct
    {
        int m_DOPort;
        bool m_DOValue;
        int m_PulseTime;
        int m_StartEnd;
        int m_Time;
    }SDKInstructDOPulseSyncTime;

    typedef struct
    {
        int m_GDOStartPort;
        int m_GDOEndPort;
        int m_GDOValue;
    }SDKInstructGDOSet;

    typedef struct
    {
        int m_GDOStartPort;
        int m_GDOEndPort;
        int m_GDOValue;
        int m_StartEnd;
        double m_Distance;
        int m_MemberType;
    }SDKInstructGDOSetSyncPath;

    typedef struct
    {
        int m_GDOStartPort;
        int m_GDOEndPort;
        int m_GDOValue;
        int m_StartEnd;
        int m_Time;
    }SDKInstructGDOSetSyncTime;

    typedef struct
    {
        int m_DIPort;
        bool m_DIValue;
    }SDKInstructDIWait;

    typedef struct
    {
        int m_DIPort;
        bool m_DIValue;
        int m_StartEnd;
        double m_Distance;
    }SDKInstructDIWaitSyncPath;

    typedef struct
    {
        int m_DIPort;
        bool m_DIValue;
        int m_StartEnd;
        int m_Time;
    }SDKInstructDIWaitSyncTime;

    typedef struct
    {
        int m_AIPort;
        int m_AIValue;
    }SDKInstructAIWait;

    typedef struct
    {
        int m_AIPort;
        int m_AIValue;
        int m_StartEnd;
        double m_Distance;
    }SDKInstructAIWaitSyncPath;

    typedef struct
    {
        int m_AIPort;
        int m_AIValue;
        int m_StartEnd;
        int m_Time;
    }SDKInstructAIWaitSyncTime;

    typedef struct
    {
        int m_GDIStartPort;
        int m_GDIEndPort;
        int m_GDIValue;
    }SDKInstructGDIWait;

    typedef struct
    {
        int m_GDIStartPort;
        int m_GDIEndPort;
        int m_GDIValue;
        int m_StartEnd;
        double m_Distance;
    }SDKInstructGDIWaitSyncPath;

    typedef struct
    {
        int m_GDIStartPort;
        int m_GDIEndPort;
        int m_GDIValue;
        int m_StartEnd;
        int m_Time;
    }SDKInstructGDIWaitSyncTime;

    typedef struct
    {
        CartSys m_CartDist;
    }SDKInstructCORR_Tool;

    typedef struct
    {
        CartSys m_CartDist;
        char m_RefName[32];
    }SDKInstructCORR_Ref;

    typedef struct
    {
        int m_OvlRel;
        OverlapABS m_OvlABS;
        OVERLAPTYPE m_OveTyp;
    }SDKInstructOvl;

    typedef struct
    {
        int m_SpaceNum;
        bool m_Active;
    }SDKInstructAxisSpaceActivate;

    typedef struct
    {
        int m_SpaceNum;
        bool m_Active;
    }SDKInstructCartSpaceActivate;

    typedef struct
    {
        double m_ArcConfig[14];
    }SDKInstructArcConfig;//未使用，用的 ArcConfigData


    typedef struct
    {
        int m_WireSetType;
        int m_WireTime;
    }SDKInstructWireSet;

    typedef enum
    {
        eJointDist = 1,
        eCartDist
    }DISTANCETYPE;

    typedef struct
    {
        CartPos m_CartDist;        // xyzabc+mode        
        JointsPos m_JointDist;     // 关节部分
        AuxPos m_AuxPosDist;       // 附加轴信息
        DISTANCETYPE m_DistTyp;    // 位置类型
    }RcDistance;
    typedef struct
    {
        RcDistance m_RcDistance;
        int m_Dyn;//只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
        SOverlap m_Ovl;//NULL参数值都为-1，类型枚举为0
        ORITYPE m_Ori;
        WeaveParam m_Weave;                        // 摆弧参数
        SeamTrackParam m_SeamTrack;        // 电弧跟踪
        char m_ToolName[32];
        char m_RefName[32];
    }SDKInstructWLinRel;
    typedef struct
    {
        float m_weldVoltage;
        float m_weldCurrent;
        float m_weldSpeed;//add 2015.4.2
        double m_endSpeed;//add 2021.9.23
        //新增2025.05.16
        int m_ArcMode;               //焊接模式
        int m_ArcArgonFlag;          //氩弧焊标志
        int m_WeaveSyncFlag;         //摆弧同步输出电流送丝标志
        int m_ArcWeldCurrentBase;    //焊接电流基值
        int m_ArcDutyCycle;          //占空比
        int m_ArcFrequency;          //脉冲频率
        double m_ArcWireSpeed;       //峰值送丝速度
        double m_ArcWireSpeedBase;   //基值送丝速度
        int m_ArcRampTime;           //缓升时间
        int m_ArcDescentTime;        //缓降时间
        int m_ArcOnDelayWireTime;    //延迟送丝时间
        double m_ArcWireRetractLength;//焊丝回退距离
    }ARCDATA;//与ArcData同步，弃用。2025.05.16
    typedef struct
    {
        HMIPos m_StartPos;
        HMIPos m_EndPos;
        int m_LinSegType;     //1, WLinSeg1; 2, WLinSeg2, 3,WLinSegJob
        int m_SegNumber;
        double m_IntervalLength;
        double m_SegLength;
        ArcSegData m_ArcSegmentData;
        ArcData m_ArcData;
        int m_Dyn;//只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
        ORITYPE m_Ori;
        char m_ToolName[32];
        char m_RefName[32];
    }SDKInstructWLinSeg;
    typedef struct
    {
        HMIPos m_StartPos;
        HMIPos m_EndPos;
        int m_SegNumber;
        double m_SegLength;
        ArcSegData m_ArcSegmentData;
        ArcData m_ArcData;
        int m_Dyn;//只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
        ORITYPE m_Ori;
        char m_ToolName[32];
        char m_RefName[32];
    }SDKInstructWLinSeg1;
    typedef struct
    {
        HMIPos m_StartPos;
        HMIPos m_EndPos;
        double m_IntervalLength;
        double m_SegLength;
        ArcSegData m_ArcSegmentData;
        ArcData m_ArcData;
        int m_Dyn;//只可以设置位置速度，其他参数利用Dyn语句设置;NULL可以设置-1
        ORITYPE m_Ori;
        char m_ToolName[32];
        char m_RefName[32];
    }SDKInstructWLinSeg2;

    //新增2025.10.13 坐标转换结构体
    typedef struct 
    {
        HMIPos m_Pos;
        char m_RefSysName_Res[32];           // 源坐标系名称
        char m_RefSysName_Des[32];           // 目标坐标系名称
    }SDKInstructRefConvert;
}//不可以少，一定要在这个命名空间内