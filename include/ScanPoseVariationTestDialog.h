#pragma once

#include "Const.h"
#include "MeasureThenWeldService.h"

#include <QString>
#include <QWidget>

#include <functional>

class CameraFrameCache;
class ContralUnit;
class QComboBox;
class QLabel;
class QDoubleSpinBox;
class QPlainTextEdit;
class QPushButton;
class QTimer;
class RobotDriverAdaptor;

// 管理页面“调试”菜单中的内嵌“扫描变姿态精度测试”：保存独立示教点，生成固定空间直线、
// 四段周期变姿态轨迹，并复用先测后焊的相机/机器人时间对齐、手眼变换和结果落盘。
class ScanPoseVariationTestDialog : public QWidget
{
public:
    using StartCameraFunc = std::function<bool(int, const QString&, QString&)>;
    using CameraCacheFunc = std::function<CameraFrameCache*(int)>;

    ScanPoseVariationTestDialog(
        ContralUnit* controlUnit,
        int unitIndex,
        CameraFrameCache* cameraCache,
        StartCameraFunc startCamera,
        CameraCacheFunc cameraCacheForUnit,
        QWidget* parent = nullptr);
    ~ScanPoseVariationTestDialog() override;

    bool IsRunning() const { return m_running; }

private:
    RobotDriverAdaptor* ResolveDriver(bool showMessage = true) const;
    QString RobotName(RobotDriverAdaptor* driver = nullptr) const;
    QString ConfigPath() const;
    QString SelectionConfigPath() const;
    QString CurrentCameraSection() const;
    void LoadRobotList(int initialUnitIndex);
    void LoadCameraList(const QString& preferredSection = QString());
    void ChangeRobot(int comboIndex);
    void RefreshLiveImage();
    bool SaveSelection(QString* error = nullptr) const;
    bool LoadConfiguration(QString* error = nullptr);
    bool SaveConfiguration(QString* error = nullptr) const;
    bool ReadCurrentPose(T_ROBOT_COORS& pose, QString& error) const;
    bool ReadCurrentPulse(T_ANGLE_PULSE& pulse, QString& error) const;
    void TeachBasePose();
    void TeachStartPose();
    void TeachEndPose();
    void GeneratePreview();
    void RunScan();
    void AppendLog(const QString& text);
    void UpdateStatusLabels();
    void SetRunning(bool running);
    MeasureThenWeldService::ScanPoseVariationParams CurrentParams() const;

    ContralUnit* m_controlUnit = nullptr;
    int m_unitIndex = 0;
    CameraFrameCache* m_cameraCache = nullptr;
    StartCameraFunc m_startCamera;
    CameraCacheFunc m_cameraCacheForUnit;
    bool m_running = false;
    bool m_loadingSelectors = false;
    qint64 m_lastImageTimestamp = 0;

    bool m_hasBasePose = false;
    bool m_hasStartPose = false;
    bool m_hasEndPose = false;
    bool m_hasStartPulse = false;
    T_ROBOT_COORS m_basePose{};
    T_ROBOT_COORS m_startPose{};
    T_ROBOT_COORS m_endPose{};
    T_ANGLE_PULSE m_startPulse{};

    QLabel* m_baseLabel = nullptr;
    QLabel* m_startLabel = nullptr;
    QLabel* m_endLabel = nullptr;
    QComboBox* m_robotCombo = nullptr;
    QComboBox* m_cameraCombo = nullptr;
    QDoubleSpinBox* m_scanSpeedSpin = nullptr;
    QDoubleSpinBox* m_lowPlatformSpin = nullptr;
    QDoubleSpinBox* m_risingSpin = nullptr;
    QDoubleSpinBox* m_highPlatformSpin = nullptr;
    QDoubleSpinBox* m_fallingSpin = nullptr;
    QDoubleSpinBox* m_leftAngleSpin = nullptr;
    QDoubleSpinBox* m_rightAngleSpin = nullptr;
    QDoubleSpinBox* m_transitionSpin = nullptr;
    QDoubleSpinBox* m_pointStepSpin = nullptr;
    QPushButton* m_teachBaseButton = nullptr;
    QPushButton* m_teachStartButton = nullptr;
    QPushButton* m_teachEndButton = nullptr;
    QPushButton* m_generateButton = nullptr;
    QPushButton* m_runButton = nullptr;
    QLabel* m_liveImageLabel = nullptr;
    QLabel* m_liveImageStatusLabel = nullptr;
    QTimer* m_liveImageTimer = nullptr;
    QPlainTextEdit* m_logEdit = nullptr;
};
