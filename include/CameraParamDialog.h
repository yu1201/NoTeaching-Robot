#pragma once

#include "ContralUnit.h"

#include <QDialog>

#include <functional>

class QComboBox;
class CameraFrameCache;
class QLabel;
class QPlainTextEdit;
class QWidget;

class CameraParamDialog : public QDialog
{
public:
    using StartCameraFunc = std::function<bool(QString&)>;
    using StopCameraFunc = std::function<void()>;
    using SetupStatusChangedFunc = std::function<void()>;

    explicit CameraParamDialog(
        ContralUnit* pContralUnit,
        const QString& robotName,
        StartCameraFunc startCamera,
        StopCameraFunc stopCamera,
        CameraFrameCache* cameraCache,
        SetupStatusChangedFunc setupStatusChanged = nullptr,
        QWidget* parent = nullptr);

private:
    void LoadCameraList();
    void UpdateCurrentCameraInfo();
    void OpenCameraBasicParamDialog();
    void OpenHandEyeDialog();
    void OpenHandEyeCalibrationDialog();
    QString CurrentRobotName() const;
    QString CurrentCameraSection() const;
    void AppendLog(const QString& text);

    ContralUnit* m_pContralUnit = nullptr;
    QString m_robotName;
    StartCameraFunc m_startCamera;
    StopCameraFunc m_stopCamera;
    SetupStatusChangedFunc m_setupStatusChanged;
    CameraFrameCache* m_pCameraCache = nullptr;
    QComboBox* m_pCameraCombo = nullptr;
    QLabel* m_pPathLabel = nullptr;
    QLabel* m_pCameraPathLabel = nullptr;
    QLabel* m_pImagePlaceholder = nullptr;
    QLabel* m_pCameraSectionLabel = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
};
