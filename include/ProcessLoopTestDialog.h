#pragma once

#include <QDialog>
#include <QString>
#include <QList>
#include <QPair>

#include <atomic>
#include <functional>
#include <thread>

class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;
class QPlainTextEdit;

// 流程循环测试设置：默认取先测后焊「测量焊接参数」里已设好的预设值；勾选「覆盖」的项
// 才用界面上填的值替换（其余仍用预设）。当前框架只做「仅扫描」循环（不自动焊接）。
struct ProcessLoopTestSettings
{
    int unitIndex = -1;             // 目标机器人单元号（-1=当前单元）
    int repeatCount = 10;           // 循环次数（infinite=true 时忽略）
    bool infinite = false;          // 持续循环直到手动停止
    bool stopOnFailure = true;      // 某次失败即停止（机器人状态异常时别继续硬跑）
    int scanIntervalSeconds = 0;    // 定时扫描：两次扫描之间等待秒数（0=连续不等待）
    bool doWeld = false;            // 是否跑焊接段（默认否=仅扫描）；实际焊接/空跑仍按预设 bDoActualWeld

    bool overrideScanSpeed = false;    double scanSpeedMmPerMin = 0.0;
    bool overrideRunSpeed = false;     double runSpeedMmPerMin = 0.0;
    bool overrideCameraOffset = false; double cameraTimeOffsetMs = 0.0;
};

// 某单元先测后焊预设默认值，用于界面预填/展示。
struct ProcessLoopTestDefaults
{
    bool ok = false;
    double scanSpeedMmPerMin = 0.0;
    double runSpeedMmPerMin = 0.0;
    double cameraTimeOffsetMs = 0.0;
    QString paramGroupName;
    QString issue;                  // ok=false 时的原因（无驱动/未示教等）
};

// 管理页「流程测试」：自动循环跑先测后焊（仅扫描）流程，用于重复性/稳定性验证。
// 界面负责设置采集与开/停；实际循环由主窗口注入的 runner 在后台线程执行，进度经回调回传。
class ProcessLoopTestDialog : public QDialog
{
    Q_OBJECT

public:
    // 进度回调（后台线程调用）：iteration=当前第几次(1-based)，total=总次数(<=0 表示持续)，
    // okCount/failCount 累计，step=当前步骤描述，finished=整个测试结束。
    using ProgressCallback = std::function<void(int iteration, int total, int okCount, int failCount,
        const QString& step, bool finished)>;
    using LogCallback = std::function<void(const QString&)>;
    // 跑循环：在调用线程（本对话框的后台线程）同步执行，按 stopFlag 中止，经回调汇报。
    using RunnerFunc = std::function<void(const ProcessLoopTestSettings&, std::atomic<bool>* stopFlag,
        const ProgressCallback&, const LogCallback&)>;
    // 读某单元的先测后焊预设默认（扫描/运行速度、相机偏移），用于界面预填。
    using LoadDefaultsFunc = std::function<ProcessLoopTestDefaults(int unitIndex)>;

    ProcessLoopTestDialog(const QList<QPair<int, QString>>& units,
        int defaultUnitIndex,
        LoadDefaultsFunc loadDefaults,
        RunnerFunc runner,
        QWidget* parent = nullptr);
    ~ProcessLoopTestDialog() override;

protected:
    // 关界面/切走时若正在测试：提示并停止（安全优先，不留后台机器人运动）。
    void closeEvent(QCloseEvent* event) override;

private:
    void BuildUi();
    void ReloadDefaultsForCurrentUnit();
    ProcessLoopTestSettings CollectSettings() const;
    void OnStart();
    void OnStop();
    void JoinWorker();
    void SetRunningUi(bool running);
    void AppendLog(const QString& text);
    void UpdateProgressUi(int iteration, int total, int okCount, int failCount,
        const QString& step, bool finished);

    LoadDefaultsFunc m_loadDefaults;
    RunnerFunc m_runner;
    QList<QPair<int, QString>> m_units;

    // 采集区
    QComboBox* m_unitCombo = nullptr;
    QSpinBox* m_repeatSpin = nullptr;
    QCheckBox* m_infiniteCheck = nullptr;
    QCheckBox* m_stopOnFailureCheck = nullptr;
    QCheckBox* m_doWeldCheck = nullptr;         // 是否包含焊接段（否则仅扫描）
    QSpinBox* m_intervalSpin = nullptr;         // 定时扫描间隔值
    QComboBox* m_intervalUnitCombo = nullptr;   // 间隔单位：秒/分钟

    // 参数覆盖区（默认取先测后焊预设，勾选才覆盖）
    QLabel* m_defaultsLabel = nullptr;
    QCheckBox* m_ovScanSpeedCheck = nullptr;   QDoubleSpinBox* m_scanSpeedSpin = nullptr;
    QCheckBox* m_ovRunSpeedCheck = nullptr;    QDoubleSpinBox* m_runSpeedSpin = nullptr;
    QCheckBox* m_ovCameraOffsetCheck = nullptr; QDoubleSpinBox* m_cameraOffsetSpin = nullptr;

    // 控制/进度区
    QPushButton* m_startBtn = nullptr;
    QPushButton* m_stopBtn = nullptr;
    QLabel* m_progressLabel = nullptr;
    QLabel* m_statLabel = nullptr;
    QPlainTextEdit* m_logText = nullptr;

    // 运行状态
    std::thread m_worker;
    std::atomic<bool> m_stopRequested{ false };
    std::atomic<bool> m_running{ false };
};
