#pragma once

#include "ContralUnit.h"
#include "HandEyeMatrixConfig.h"

#include <QDialog>
#include <QVector>

#include <Eigen/Dense>
#include <atomic>
#include <functional>

class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class FANUCRobotCtrl;

class HandEyeCalibrationDialog : public QDialog
{
public:
    using StartCameraFunc = std::function<bool(QString&)>;
    using StopCameraFunc = std::function<void()>;

    explicit HandEyeCalibrationDialog(
        ContralUnit* pContralUnit,
        const QString& robotName,
        const QString& cameraSection,
        StartCameraFunc startCamera,
        StopCameraFunc stopCamera,
        QWidget* parent = nullptr);

private:
    struct SampleWidgets
    {
        QLabel* pStateLabel = nullptr;
        QVector<QLineEdit*> robotEdits;
        QVector<QLineEdit*> cameraEdits;
    };

    bool LoadConfig();
    bool SaveConfig();
    bool SaveConfigSilently(QString* error = nullptr);
    bool CaptureTcpPoint();
    bool CaptureSample(int index);
    bool ComputeAndSaveMatrix();
    bool TestHandEyeMatrix();
    bool UploadRobotHandEyeCheckProgram(QString* error = nullptr);
    bool UploadAutoCalibrationProgram();
    bool StartAutoCalibration();
    bool EnsureCameraReadyForAutoCalibration(QString* error = nullptr);
    void OpenMatrixDialog();

    bool ReadLatestCameraPoint(Eigen::Vector3d& cameraPoint, QString* error = nullptr) const;
    RobotDriverAdaptor* CurrentDriver(QString* error = nullptr) const;
    FANUCRobotCtrl* CurrentFanucDriver(QString* error = nullptr) const;

    bool ApplyCapturedTargetPoint(const T_ROBOT_COORS& pose, QString* error = nullptr);
    bool ApplyCapturedSample(int index, const T_ROBOT_COORS& pose, const Eigen::Vector3d& cameraPoint, QString* error = nullptr);
    void SetAutoCalibrationUiRunning(bool running);
    void SetAutoCalibrationStateText(const QString& text);
    bool ExportCalibrationReport(const HandEyeMatrixConfig& matrix, QString* reportPathOut = nullptr, QString* error = nullptr) const;

    void SetRobotPoseEditors(QVector<QLineEdit*>& edits, const T_ROBOT_COORS& pose);
    T_ROBOT_COORS ReadRobotPoseEditors(const QVector<QLineEdit*>& edits, bool* ok = nullptr, QString* error = nullptr) const;
    void SetVectorEditors(QVector<QLineEdit*>& edits, const Eigen::Vector3d& point);
    Eigen::Vector3d ReadVectorEditors(const QVector<QLineEdit*>& edits, bool* ok = nullptr, QString* error = nullptr) const;

    void UpdatePathLabels();
    void UpdateSampleStates();
    void SelectSampleTab(int index);
    void AppendLog(const QString& text);

    ContralUnit* m_pContralUnit = nullptr;
    QString m_robotName;
    QString m_cameraSection;
    HandEyeCalibrationConfig m_config;
    StartCameraFunc m_startCamera;
    StopCameraFunc m_stopCamera;

    QLabel* m_pAutoStateLabel = nullptr;
    QLabel* m_pCalibrationPathLabel = nullptr;
    QLabel* m_pMatrixPathLabel = nullptr;
    QLabel* m_pReportPathLabel = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QPushButton* m_pUploadAutoProgramBtn = nullptr;
    QPushButton* m_pAutoCalibrationBtn = nullptr;
    QTabWidget* m_pSampleTabWidget = nullptr;
    QVector<QLineEdit*> m_tcpEdits;
    QVector<SampleWidgets> m_sampleWidgets;
    std::atomic_bool m_bAutoCalibrationRunning = false;
};
