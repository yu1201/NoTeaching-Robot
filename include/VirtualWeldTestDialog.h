#pragma once

#include <QDialog>
#include <QString>

#include "Const.h"  // T_ROBOT_COORS

class ContralUnit;
class RobotDriverAdaptor;
class QDoubleSpinBox;
class QComboBox;
class QCheckBox;
class QPushButton;
class QPlainTextEdit;
class QLabel;
class QCloseEvent;

// 调试用「虚拟焊道测试」页签（管理界面 → 调试）：
// 取机器人当前位姿 → 沿 ±Y 造一条长度/点间距可调的虚拟直线焊道 →
// 走正常轨迹生成（读保存的工艺：姿态/圆滑/摆动）产出 _WeldPose_2mm / _SeamComp / srp / srd →
// 可一键下发并运行（实焊含摆动，带焊前/焊后确认），主要用于测试机器人密集点的摆动。
class VirtualWeldTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit VirtualWeldTestDialog(ContralUnit* pContralUnit, int unitIndex = 0, QWidget* parent = nullptr);
    ~VirtualWeldTestDialog() override;

    // 运行态查询（供主窗口在切换机器人/关闭程序前拦截，避免删除正在驱动机器人的页面或并发下发）。
    bool IsRunning() const { return m_running; }

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void OnReadCurrentPos();
    void OnGenerate();
    void OnRunOnRobot();
    void OnRobotChanged(int index);
    void OnWeldProcessChanged(int index);

private:
    RobotDriverAdaptor* ResolveDriver();
    QString ResolveRobotName(RobotDriverAdaptor* driver) const;
    QString CurrentRobotName() const;
    void LoadRobotList();
    void LoadWeldProcessList();
    void AppendLog(const QString& text);
    void SetRunning(bool running);
    void InvalidateGeneratedTrajectory();

    ContralUnit* m_pContralUnit = nullptr;
    int m_unitIndex = 0;
    bool m_running = false;
    bool m_loadingSelectors = false;

    bool m_hasStartCoors = false;
    T_ROBOT_COORS m_startCoors{};
    QString m_lastWeldPosePath;  // 干净直线 pose 文件，既供查看也作下发执行文件
    double m_lastPointStepMm = 4.0;

    QComboBox* m_pRobotCombo = nullptr;
    QComboBox* m_pWeldProcessCombo = nullptr;
    QDoubleSpinBox* m_lengthSpin = nullptr;
    QDoubleSpinBox* m_stepSpin = nullptr;
    QComboBox* m_directionCombo = nullptr;
    QCheckBox* m_actualWeldCheck = nullptr;
    QLabel* m_posLabel = nullptr;
    QPushButton* m_readPosBtn = nullptr;
    QPushButton* m_generateBtn = nullptr;
    QPushButton* m_runBtn = nullptr;
    QPlainTextEdit* m_logEdit = nullptr;
};
