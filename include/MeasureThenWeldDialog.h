#pragma once

#include "Const.h"
#include "ContralUnit.h"

#include <QDialog>
#include <QString>

#include <functional>
#include <vector>

class FANUCRobotCtrl;
class MeasureThenWeldService;
class QPushButton;
class QPlainTextEdit;

struct T_PRECISE_MEASURE_PARAM
{
    std::string sRobotName;
    std::string sIniFilePath;
    std::string sSectionName = "Postion0";

    // PreciseMeasureParam.ini 中的运行速度和扫描速度。
    double dScanSpeed = 0.0;
    double dRunSpeed = 0.0;
    double dAcc = 0.0;
    double dDec = 0.0;

    // 预设流程动作点：下枪安全姿态 -> 扫描起点 -> 扫描终点 -> 收枪安全姿态。
    // 安全姿态保留脉冲点，扫描起终点改为直角坐标点。
    std::vector<T_ANGLE_PULSE> vtStartSafePulse;
    T_ROBOT_COORS tStartPos;
    T_ROBOT_COORS tEndPos;
    std::vector<T_ANGLE_PULSE> vtEndSafePulse;
};

// 先测后焊入口界面：当前实现“预设参数线扫采集”，线扫处理按钮先保留空函数。
class MeasureThenWeldDialog : public QDialog
{
    Q_OBJECT

public:
    using StartCameraFunc = std::function<bool(QString&)>;
    using StopCameraFunc = std::function<void()>;

    MeasureThenWeldDialog(ContralUnit* pContralUnit, StartCameraFunc startCamera, StopCameraFunc stopCamera, QWidget* parent = nullptr);

signals:
    void FlowStepChanged(const QString& text);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    FANUCRobotCtrl* GetFirstFanucDriver();

    // 读取 Data/<RobotName>/PreciseMeasureParam.ini 中当前启用的 PostionN 分组。
    bool LoadPresetParam(FANUCRobotCtrl* pFanucDriver, T_PRECISE_MEASURE_PARAM& param, QString& error);
    bool ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const;

    // 单段/多段脉冲运动，内部会阻塞等待机器人运动结束。
    bool MovePulseAndWait(FANUCRobotCtrl* pFanucDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name);
    bool MovePulseListAndWait(FANUCRobotCtrl* pFanucDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name);
    bool MoveCoorsAndWait(FANUCRobotCtrl* pFanucDriver, const T_ROBOT_COORS& coors, double speed, const QString& name);

    // 扫描段：机器人从 StartPos 运动到 EndPos，同时每 10ms 读取相机缓存点。
    bool ScanMoveAndCollect(FANUCRobotCtrl* pFanucDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath);
    QString BuildResultDir(const std::string& robotName) const;
    bool SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const;

    // 每个危险动作前弹窗确认；取消会退出当前流程。
    bool ConfirmContinue(const QString& actionName);

    // 预设参数流程入口和线扫处理占位入口。
    void RunPresetParamFlow();
    void RunLineScanProcess();
    void AppendLog(const QString& text);
    void SetFlowStep(const QString& text);
    void SetRunning(bool running);

private:
    ContralUnit* m_pContralUnit = nullptr;
    MeasureThenWeldService* m_pService = nullptr;
    StartCameraFunc m_startCamera;
    StopCameraFunc m_stopCamera;

    QPushButton* m_pPresetParamBtn = nullptr;
    QPushButton* m_pLineScanProcessBtn = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;

    bool m_bRunning = false;
};
