#pragma once

#include "ContralUnit.h"

#include <QDialog>

#include <functional>

class QComboBox;
class QCloseEvent;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QWidget;

class CameraParamDialog : public QDialog
{
public:
    using StartCameraFunc = std::function<bool(QString&)>;
    using StopCameraFunc = std::function<void()>;

    explicit CameraParamDialog(
        ContralUnit* pContralUnit,
        StartCameraFunc startCamera,
        StopCameraFunc stopCamera,
        QWidget* parent = nullptr);

private:
    void closeEvent(QCloseEvent* event) override;
    void LoadRobotList();
    void LoadCameraList();
    void UpdateCurrentCameraInfo();
    void OpenHandEyeDialog();
    void OpenHandEyeCalibrationDialog();
    QString CurrentRobotName() const;
    QString CurrentCameraSection() const;
    QString CurrentCameraIniPath() const;
    bool LoadCameraParam();
    bool SaveCameraParam();
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);

    ContralUnit* m_pContralUnit = nullptr;
    StartCameraFunc m_startCamera;
    StopCameraFunc m_stopCamera;
    QComboBox* m_pRobotCombo = nullptr;
    QComboBox* m_pCameraCombo = nullptr;
    QLabel* m_pPathLabel = nullptr;
    QLabel* m_pCameraPathLabel = nullptr;
    QLabel* m_pImagePlaceholder = nullptr;
    QLabel* m_pCameraSectionLabel = nullptr;
    QLineEdit* m_pDeviceAddressEdit = nullptr;
    QLineEdit* m_pDevicePortEdit = nullptr;
    QLineEdit* m_pExposureTimeEdit = nullptr;
    QLineEdit* m_pGainLevelEdit = nullptr;
    QLineEdit* m_pCameraTypeEdit = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QString m_cleanSnapshot;
};
