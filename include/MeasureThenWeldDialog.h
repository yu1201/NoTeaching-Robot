#pragma once

#include "WeldExecutionSafety.h"

#include "Const.h"
#include "ContralUnit.h"

#include <QDialog>
#include <QMutex>
#include <QString>

#include <atomic>
#include <functional>
#include <vector>

class MeasureThenWeldService;
class CameraFrameCache;
class RobotDriverAdaptor;
class COPini;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QPlainTextEdit;
class QTimer;
class RobotDriverAdaptor;

struct T_PRECISE_MEASURE_PARAM
{
    std::string sRobotName;
    std::string sIniFilePath;
    std::string sSectionName = "MeasureGroup0.Scan";
    std::string sWeldParamFilePath;
    std::string sWeldSectionName = "MeasureGroup0.Weld";
    int nParamGroupIndex = 0;
    QString sParamGroupName = "参数组1";

    // 配置库当前测量焊接参数组中的运行速度、扫描速度和相机时间补偿参数。
    // 注：相机读取帧率已迁出测量参数，改由相机参数(CameraParam.ini 的 CameraReadFps)维护。
    double dScanSpeed = 0.0;
    double dRunSpeed = 0.0;
    double dCameraTimeOffsetMs = 0.0;
    bool bUseStatTimeAlign = true;  // 统计时间对齐开关：false=首帧对齐(旧算法，对照测试相机时间戳用)
    double dAcc = 0.0;
    double dDec = 0.0;

    // 焊接段运行参数：界面可切换实际焊接/空跑，不焊接时使用空跑速度。
    bool bDoActualWeld = true;
    double dWeldSpeedMmPerMin = 400.0;
    double dDryRunSpeedMmPerMin = 1000.0;
    double dWeldSafeMoveSpeedMmPerMin = 1000.0;
    // STEP连续运动过渡比例，对应生成SRD文件中的OVERLAPREL变量。
    double dStepOverlapRel = 20.0;
    // 最终下发/生成SRP时的焊接轨迹抽样间距，只作用于最后运动点，不影响前序拐点和姿态生成。
    double dFinalWeldTrajectoryStepMm = 4.0;
    // 断点续焊搭接回退距离（毫米弧长，不再按固定点数估算）。
    double dResumeBacktrackMm = 5.0;
    // 焊接方向：1 从姿态文件起点焊到终点，-1 从终点焊回起点。
    int nWeldDirection = 1;
    // 焊接轨迹下枪/收枪安全位相对首尾焊点的回退距离，对应配置库中的 GunDownBackSafeDis。
    double dGunDownBackSafeDis = 70.0;
    double dWeldRzGainDeg = 0.0;
    // 焊接平台标准姿态示教：启用后 RX/RY 使用示教值，RZ 以平台示教值为基准修正坡道姿态。
    bool bUseTaughtWeldPose = false;
    double dTaughtWeldPoseRxDeg = 0.0;
    double dTaughtWeldPoseRyDeg = 0.0;
    double dTaughtWeldPoseRzDeg = 0.0;
    // 爬坡/下坡段按波峰波谷趋势生成姿态时，RZ 相对测量参考 RZ 的夹紧范围。
    double dSlopeRzMinDeg = -20.0;
    double dSlopeRzMaxDeg = 20.0;

    // 预设流程动作点：下枪安全姿态 -> 扫描起点 -> 扫描终点 -> 收枪安全姿态。
    // 安全姿态保留脉冲点，扫描起终点改为直角坐标点。
    std::vector<T_ANGLE_PULSE> vtStartSafePulse;
    bool bHasStartPulse = false;
    T_ANGLE_PULSE tStartPulse;
    T_ROBOT_COORS tStartPos;
    T_ROBOT_COORS tEndPos;
    std::vector<T_ANGLE_PULSE> vtEndSafePulse;

    // 扫描安全位推算参数：由扫描起点/终点直角位姿按枪方向偏移得到安全位。
    bool bUseComputedScanSafe = true;
    double dScanSafeOffsetDistanceMm = 150.0;
    double dScanSafeGunAngleDeg = 30.0;
    int nScanSafeXDirection = -1;
    double dScanSafeLiftHeightMm = 150.0;
    double dScanSafeFlipWarnThresholdDeg = 90.0;
};

// 先测后焊入口界面：预设参数负责局部精测采集和自动处理；线扫处理保留给整体大范围粗定位。
class MeasureThenWeldDialog : public QDialog
{
    Q_OBJECT

public:
    using StartCameraFunc = std::function<bool(int, QString&)>;
    using StopCameraFunc = std::function<void()>;
    using CameraCacheFunc = std::function<CameraFrameCache*(int)>;

    MeasureThenWeldDialog(ContralUnit* pContralUnit, int unitIndex, StartCameraFunc startCamera, StopCameraFunc stopCamera, CameraCacheFunc cameraCacheForUnit, QWidget* parent = nullptr);
    bool IsRunning() const;
    void ReloadSelectors();
    // 流程成功后回调（UI 线程，参数=Result 案例目录），主窗口接在线上传。
    void SetScanDataUploadHook(std::function<void(const QString&)> hook);
    // 开始前置守卫：返回 false 则拦下先测后焊流程并把原因写进 reason（供主窗口互锁流程测试）。
    void SetPreStartGuard(std::function<bool(QString& reason)> guard) { m_preStartGuard = std::move(guard); }
    // 实时账号会话守卫：阻塞选择器/确认框返回后及每个危险动作检查点必须再次通过。
    void SetLiveSessionGuard(std::function<bool()> guard) { m_liveSessionGuard = std::move(guard); }

signals:
    void FlowStepChanged(const QString& text);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void LoadRobotList();
    void LoadParamGroups();
    void LoadWeldProcessList();
    void LoadCompGroupLists();
    void OnRobotChanged(int index);
    void OnParamGroupChanged(int index);
    void OnWeldProcessChanged(int index);
    QString CurrentRobotName() const;
    int CurrentParamGroupIndex() const;
    int CurrentPoseCompGroupIndex() const;
    int CurrentSeamCompGroupIndex() const;
    bool SaveCurrentParamGroupSelection(QString& error) const;
    bool SaveCurrentWeldProcessSelection(QString& error) const;
    bool SaveCurrentCompGroupSelections(QString& error) const;
    CameraFrameCache* ResolveCameraCacheForUnit(int unitIndex);
    RobotDriverAdaptor* GetRobotDriver();

    // 读取配置库中当前启用的测量焊接参数组。
    bool LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error);
    bool ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const;

    // 单段/多段脉冲运动，内部会阻塞等待机器人运动结束。
    bool MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name);
    bool MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name);
    bool MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name);
    bool MoveScanStartSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed);
    bool MoveScanEndSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed);

    // 扫描段：机器人从 StartPos 运动到 EndPos，同时按配置帧率读取相机缓存点。
    bool ScanMoveAndCollect(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath);
    QString BuildResultDir(const std::string& robotName) const;
    bool SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const;

    // 每个危险动作前弹窗确认；取消会退出当前流程。
    bool ConfirmContinue(const QString& actionName);
    bool ShowCheckpointDialog(const QString& title, const QString& detail);
    bool HasLiveSession(const QString& actionName);
    // 流程线程安全：从姿态文件路径推导案例目录，经队列回 UI 线程调用上传钩子。
    void NotifyFlowResultForUpload(const QString& poseFilePath);

    // 互锁前置检查：被前置守卫拦下时弹 title 提示并返回 true（表示应当中止启动）。
    bool BlockedByOtherFlow(const QString& title);
    // 预设参数流程入口和整体大线扫粗定位占位入口。
    void RunPresetParamFlow();
    void RunSkipScanWeldFlow();
    void RunLineScanProcess();
    // 相机时间补偿自动标定：同一工件自动正/反向各扫一次，按拐点分裂量解算相机链路固有延迟并写回补偿参数。
    void RunCameraTimeOffsetCalibrationFlow();
    // 运行监控窗口的暂停/继续（STEP: SetModeCmd STOP/START，含回位前置）与断点续焊入口。
    void OnPauseResumeClicked();
    void OnResumeWeldClicked();
    // 断点续焊独立流程：只认V2落盘记录绑定的实际 FinalSampled 轨迹，校验案例/端点/参数指纹/SHA256 后按毫米弧长续焊。
    void RunResumeWeldFlow();
    void RunSafeRetreatRecoveryFlow();
    bool PrepareActiveWeldCheckpoint(
        RobotDriverAdaptor* pRobotDriver,
        const T_PRECISE_MEASURE_PARAM& param,
        const QString& sourcePosePath,
        const QString& sourcePoseSha256,
        const QString& sampledPosePath,
        const QString& sampledPoseSha256,
        qint64 sampledPoseSize,
        const QString& programName,
        int sampledPointCount,
        double effectiveFinalStepMm,
        const QString& parameterFingerprint,
        bool resumeCheckpointSupported,
        const QString& resumeUnsupportedReason,
        const T_ROBOT_COORS& requiredEndSafePose,
        double safeMoveSpeedMmPerMin,
        QString& error);
    QString ActiveWeldCheckpointRecord() const;
    void ClearActiveWeldCheckpoint();
    bool FinishActiveWeldExecution(
        const WeldExecutionTerminalResult& terminal,
        QString& error);
    void SetWeldPauseAvailable(bool available);
    void RefreshWeldModeFromParam();
    void SaveWeldModeToParam(bool doActualWeld);
    bool IsActualWeldModeChecked() const;
    void AppendLog(const QString& text);
    void SetFlowStep(const QString& text);
    void SetRunning(bool running);
    void ResetProgress(const QString& text);
    void SetProgress(int value, const QString& text);
    void SetProgressBusy(int baseValue, const QString& text);
    void FinishProgress(bool ok, const QString& text);
    void UpdateProgressAnimation();

private:
    ContralUnit* m_pContralUnit = nullptr;
    int m_unitIndex = 0;
    MeasureThenWeldService* m_pService = nullptr;
    StartCameraFunc m_startCamera;
    StopCameraFunc m_stopCamera;
    CameraCacheFunc m_cameraCacheForUnit;
    CameraFrameCache* m_pCameraCache = nullptr;

    QComboBox* m_pRobotCombo = nullptr;
    QComboBox* m_pParamGroupCombo = nullptr;
    QComboBox* m_pWeldProcessCombo = nullptr;
    QComboBox* m_pPoseCompGroupCombo = nullptr;
    QComboBox* m_pSeamCompGroupCombo = nullptr;
    QPushButton* m_pPresetParamBtn = nullptr;
    QPushButton* m_pSkipScanWeldBtn = nullptr;
    QPushButton* m_pLineScanProcessBtn = nullptr;
    QPushButton* m_pTimeOffsetCalibBtn = nullptr;
    // 运行监控最大化窗口(类型在 cpp 匿名空间定义，照 GroovePointCloudDialog 先例)：流程启动时弹出，
    // 内含流程步骤/进度、实时激光线点云+相机图像、日志、暂停/继续与断点续焊按钮。
    QDialog* m_pRunMonitor = nullptr;
    QTimer* m_pLiveViewTimer = nullptr;
    // 暂停断点记录：暂停那一刻的程序行号与机器人位姿（回位前置与断点续焊用）。
    int m_pauseProgramLine = -1;
    T_ROBOT_COORS m_pausePose{};
    bool m_hasPausePose = false;
    mutable QMutex m_activeWeldCheckpointMutex;
    QString m_activeWeldCheckpointRecord;
    QString m_activeWeldPriorStoredRecord;
    bool m_activeWeldResumeFlow = false;
    // 扫描数据在线上传钩子：流程成功后携案例目录回 UI 线程调用（主窗口接 ScanDataUploader）。
    std::function<void(const QString&)> m_scanDataUploadHook;
    // 互锁：流程测试在跑时拦下本流程，避免两条流程指挥同一台机器人。
    std::function<bool(QString&)> m_preStartGuard;
    std::function<bool()> m_liveSessionGuard;
    QCheckBox* m_pActualWeldCheck = nullptr;
    // 流程免确认：勾选后跳过中间步骤/信息类确认弹窗；首次运动、进入焊接、
    // 翻转风险告警、历史目录核对始终弹出。状态缓存在原子成员供流程线程读取。
    QCheckBox* m_pSkipConfirmCheck = nullptr;
    std::atomic<bool> m_bSkipFlowConfirms{ false };
    QLabel* m_pProgressLabel = nullptr;
    QProgressBar* m_pProgressBar = nullptr;
    QTimer* m_pProgressAnimationTimer = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;

    bool m_bRunning = false;
    bool m_bLoadingSelectors = false;
    bool m_bProgressBusy = false;
    int m_nProgressValue = 0;
    QString m_sProgressText;
};
