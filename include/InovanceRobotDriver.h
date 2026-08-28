#pragma once

#include "RobotDriverAdaptor.h"

#include <atomic>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <vector>

// Inovance/汇川控制器远程以太网驱动。
// 品牌协议、许可、数据流模式、姿态 A/B/C 与通用 RX/RY/RZ 的映射全部封装在本类；
// 业务层只能持有 RobotDriverAdaptor。
class InovanceRobotCtrl final : public RobotDriverAdaptor
{
public:
    InovanceRobotCtrl(std::string unitName, RobotLog* log);
    ~InovanceRobotCtrl() override;

    bool InitRobotDriver(std::string unitName) override;
    RobotDriverDescriptor DriverDescriptor() const override;
    std::uint64_t DriverCapabilities() const override;
    RobotConnectionEndpoint ControlEndpoint() const override;
    bool Connect() override;
    bool Disconnect() override;
    void EnsureConnectionForMonitor() override;

    RobotFileTransferProfile FileTransferProfile() const override;
    std::shared_ptr<RobotFileTransferSession> CreateFileTransferSession(
        std::string* error = nullptr) const override;

    bool ValidateLinearSpeedMmPerMin(
        double speedMmPerMin, std::string* error = nullptr) const override;
    bool MoveLinearMmPerMin(
        const T_ROBOT_COORS& target,
        double speedMmPerMin,
        int externalAxleType,
        const int* configuration = nullptr) override;
    bool MoveJointPercent(
        const T_ANGLE_PULSE& target,
        double speedPercent,
        int externalAxleType) override;
    RobotMotionStatus ReadMotionStatus() override;
    RobotMotionStatus ReadMotionStatusPassive(
        long long* robotMs = nullptr, long long* pcRecvMs = nullptr) override;

    bool ReserveTrajectory(
        RobotTrajectoryPurpose purpose, RobotTrajectoryHandle& handle) override;
    bool DownlinkTrajectory(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        RobotTrajectoryHandle& handle) override;
    bool ExportTrajectoryProgramFiles(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        const std::string& outputDirectory,
        RobotTrajectoryHandle& handle,
        std::string* error = nullptr) override;
    bool StartTrajectory(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        RobotTrajectoryHandle& handle) override;
    bool WaitTrajectory(
        const RobotTrajectoryHandle& handle,
        int pollDelayMs,
        int runTimeoutMs,
        RobotMotionStatus* terminalStatus = nullptr) override;
    bool GetTrackedMotionIdentity(
        std::string& projectName,
        std::string& programName,
        bool* alreadyStopped = nullptr) override;
    bool PauseTrackedMotion(
        const std::string& expectedProgramName,
        int& programLine,
        T_ROBOT_COORS& pausedPose,
        std::string* projectName = nullptr,
        std::string* programName = nullptr) override;
    bool ResumeTrackedMotion(
        const std::string& expectedProgramName,
        const T_ROBOT_COORS& checkpointPose,
        double maxPositionDeviationMm,
        double maxAngleDeviationDeg,
        double* positionDeviationMm = nullptr,
        double* angleDeviationDeg = nullptr) override;
    RobotPersistentRecoveryStrategy PersistentRecoveryStrategy() const override;
    bool AbortPersistedMotion(const std::string& expectedProgramName) override;

    bool SetOperationMode(RobotOperationMode mode) override;
    bool InitializeAfterConnect(std::string* summary = nullptr) override;
    bool ShutdownBeforeDisconnect() override;
    void ReloadRuntimeConfiguration() override;
    bool PrepareNativeProgramUpload() override;

    bool StartContinuousJog(int moveType, double canonicalSpeed) override;
    bool PushContinuousJogPoint(
        const T_ROBOT_COORS& target, double speedMmPerMin) override;
    bool PushContinuousJogPoint(
        const T_ANGLE_PULSE& target, double speedPercent) override;
    void RequestEndContinuousJog() override;
    void EndContinuousJog() override;
    bool IsContinuousJogRunning() const override;

    int UploadNativeProgramSource(
        const std::string& localPath,
        const std::string& remoteDirectory = std::string()) override;
    std::string SendDiagnosticCommand(const std::string& command) override;
    bool WriteCartesianRegister(
        int index, const double pose[8], int config[7]) override;
    bool RunProgramAndWait(
        const std::string& programName,
        int startTimeoutMs,
        int finishTimeoutMs,
        int pollDelayMs,
        RobotMotionStatus* terminalStatus = nullptr) override;
    bool InstallHandEyeSupportPrograms(std::string* summary = nullptr) override;
    bool RunHandEyeValidation(
        const T_ROBOT_COORS& robotPose,
        T_ROBOT_COORS& robotCalculatedPoint) override;

    bool IsConnected() override;
    bool cleanAlarm() override;
    bool ServoOn() override;
    std::string GetRobotStatusText() override;
    std::string GetStateMonitorSourceText() const override;
    double GetCurrentPos(int axisNo) override;
    T_ROBOT_COORS GetCurrentPos() override;
    bool TryGetCurrentPos(T_ROBOT_COORS& pos) override;
    double GetCurrentPulse(int axisNo) override;
    T_ANGLE_PULSE GetCurrentPulse() override;
    bool TryGetCurrentPulse(T_ANGLE_PULSE& pulse) override;
    T_ROBOT_COORS GetCurrentPosPassive(
        long long* robotMs = nullptr, long long* pcRecvMs = nullptr) override;
    T_ANGLE_PULSE GetCurrentPulsePassive(
        long long* robotMs = nullptr, long long* pcRecvMs = nullptr) override;
    int CheckDonePassive(
        long long* robotMs = nullptr, long long* pcRecvMs = nullptr) override;
    int CheckDone() override;
    int CheckRobotDone(int delayMs = 200, int runTimeoutMs = 1800000) override;
    bool AbortCurrentProgramSafely() override;
    bool SetTpSpeed(int speed) override;
    bool GetToolData(int toolNo, T_ROBOT_COORS& robotToolData) override;
    bool TryGetIntVar(
        int index, int& value, const char* prefix = "INT") override;
    int GetIntVar(int index, const char* prefix = "INT") override;
    bool SetIntVar(
        int index, int value, int scope = 2, const char* prefix = "INT") override;
    bool SetIntVar(const char* name, int value, int scope = 2) override;
    bool SetRealVar(
        int index, double value, const char* prefix = "REAL", int scope = 1) override;
    int GetPosVar(
        long index,
        double array[6],
        int config[7] = { 0 },
        int moveType = POSVAR) override;
    bool GetHandEyeMatrixVariable(
        const char* variableName,
        double rotation[9],
        double translation[3],
        std::string* error = nullptr) override;

private:
    bool CloseSocketLocked();
    bool SendCommand(
        const std::string& command,
        std::string& response,
        int timeoutMs = 3000);
    bool QueryInt(const std::string& command, int& value);
    bool QueryDoubles(
        const std::string& command,
        std::vector<double>& values,
        std::size_t minimumCount = 1);
    bool EnsureControlPermit();
    bool EnsureMotionReady();
    bool SetDataStreamMode(const char* action, int expectedMode);
    bool WaitForCommandDone(int commandId, int pollDelayMs, int timeoutMs);
    bool ValidateMoveInfos(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        std::string& error) const;
    bool HasVerifiedWeldJobContract(std::string* error = nullptr) const;
    bool WriteTrajectoryJobFile(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        const std::string& outputDirectory,
        RobotTrajectoryHandle& handle,
        std::string& error);
    bool UploadTrajectoryJob(
        RobotTrajectoryHandle& handle,
        std::string& error);
    bool VerifyTrajectoryJobRemoteIdentity(
        const RobotTrajectoryHandle& handle,
        std::string& error) const;
    bool PrepareWeldJobHardware(std::string& error);
    bool ConfirmWeldArcOutputOff(std::string& error);
    std::uint64_t FingerprintMoveInfos(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose) const;
    bool SendCartesianMove(
        const T_ROBOT_COORS& target,
        double speedMmPerMin,
        int zone,
        const int* configuration,
        int* commandId = nullptr);
    bool SendJointMove(
        const T_ANGLE_PULSE& target,
        double speedPercent,
        int zone,
        int* commandId = nullptr);
    bool ReadCartesianPosition(T_ROBOT_COORS& pos, int armConfig[4] = nullptr);
    bool ReadPositionRegister(int index, T_ROBOT_COORS& pos, int armConfig[4]);
    void StorePassivePose(const T_ROBOT_COORS& pose, long long pcRecvMs);
    void StorePassivePulse(const T_ANGLE_PULSE& pulse, long long pcRecvMs);
    void StorePassiveMotion(const RobotMotionStatus& status, long long pcRecvMs);
    static long long SteadyMs();
    static std::string ProtocolErrorText(const std::string& response);

    std::string m_socketIp;
    int m_socketPort = 2222;
    std::string m_ftpIp;
    int m_ftpPort = 7777;
    std::string m_ftpUser;
    std::string m_ftpPassword;
    std::uintptr_t m_socketHandle = static_cast<std::uintptr_t>(~0ULL);
    std::atomic_bool m_connected{ false };
    bool m_wsaStarted = false;
    mutable std::mutex m_socketMutex;
    std::atomic<long long> m_lastConnectAttemptMs{ 0 };

    int m_toolNo = 0;
    int m_wobjNo = 0;
    int m_maxBufferedCommands = 8;
    bool m_forceControlPermit = false;
    int m_apiUserLevel = 0;
    std::string m_apiPassword;

    // 汇川没有独立焊接协议。实际焊接由底层生成的原生PRO通过RC所属IO/DA控制；
    // 只有数据库中的现场映射完整且自洽时才声明 ActualArcWeld。
    bool m_weldJobEnabled = false;
    int m_weldArcEnableDo = -1;
    int m_weldArcEnableActiveValue = 1;
    int m_weldReadyDi = -1;
    int m_weldReadyActiveValue = 1;
    int m_weldArcEstablishedDi = -1;
    int m_weldArcEstablishedActiveValue = 1;
    int m_weldCurrentDa = -1;
    double m_weldCurrentDaGain = 0.0;
    double m_weldCurrentDaOffset = 0.0;
    double m_weldCurrentDaMin = 0.0;
    double m_weldCurrentDaMax = 0.0;
    int m_weldVoltageDa = -1;
    double m_weldVoltageDaGain = 0.0;
    double m_weldVoltageDaOffset = 0.0;
    double m_weldVoltageDaMin = 0.0;
    double m_weldVoltageDaMax = 0.0;
    int m_weldReadyTimeoutMs = 10000;
    int m_weldArcStartTimeoutMs = 10000;
    int m_weldArcEndTimeoutMs = 10000;
    int m_weldAlarmIndex = 0;
    int m_weldArcInterruptId = -1;
    std::atomic_bool m_permitOwned{ false };
    std::atomic_bool m_userLoggedIn{ false };
    std::atomic_bool m_dataStreamEnabled{ false };

    // 原生程序执行与数据流轨迹是两个独立底层。原生程序互斥只保护同一控制器的
    // main.pro 调度器更新/启动/等待；安全 STOP 不取得此锁，避免等待线程阻塞急停。
    mutable std::mutex m_nativeProgramMutex;
    std::atomic_bool m_nativeProgramRunning{ false };

    mutable std::mutex m_passiveMutex;
    T_ROBOT_COORS m_passivePose;
    T_ANGLE_PULSE m_passivePulse;
    RobotMotionStatus m_passiveMotion;
    long long m_passivePosePcMs = 0;
    long long m_passivePulsePcMs = 0;
    long long m_passiveMotionPcMs = 0;
    bool m_passivePoseValid = false;
    bool m_passivePulseValid = false;
    bool m_passiveMotionValid = false;
    int m_armConfig[4] = { 0, 0, 0, 1 };
    double m_externalValues[6] = {};

    mutable std::mutex m_trajectoryMutex;
    std::vector<T_ROBOT_MOVE_INFO> m_preparedMoveInfos;
    RobotTrajectoryPurpose m_preparedPurpose = RobotTrajectoryPurpose::ScanDryRun;
    RobotTrajectoryHandle m_activeHandle;
    std::uint64_t m_preparedFingerprint = 0;
    std::uint64_t m_trajectoryCounter = 0;
    int m_finalCommandId = -1;
    std::atomic_bool m_trajectoryRunning{ false };
    std::atomic_bool m_trajectoryPaused{ false };

    struct NativeTrajectoryResult
    {
        bool success = false;
        RobotMotionStatus terminalStatus;
        std::string error;
    };
    std::future<NativeTrajectoryResult> m_nativeTrajectoryFuture;
    bool m_nativeTrajectoryResultCached = false;
    NativeTrajectoryResult m_nativeTrajectoryCachedResult;

    std::atomic_bool m_continuousJogRunning{ false };
    std::atomic_bool m_continuousJogStopRequested{ false };
    int m_continuousJogMoveType = MOVL;
    double m_continuousJogSpeed = 0.0;
};
