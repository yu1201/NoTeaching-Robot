#pragma once

#include <QDialog>

#include <functional>

class QCheckBox;
class QCloseEvent;
class CameraIpAddressEdit;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QWidget;

class CameraBasicParamDialog : public QDialog
{
public:
    using SetupStatusChangedFunc = std::function<void()>;

    explicit CameraBasicParamDialog(
        const QString& robotName,
        const QString& cameraSection,
        SetupStatusChangedFunc setupStatusChanged = nullptr,
        QWidget* parent = nullptr);

    bool SavedThisSession() const { return m_savedThisSession; }

private:
    void closeEvent(QCloseEvent* event) override;
    bool LoadCameraParam();
    bool SaveCameraParam();
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);

    QString m_robotName;
    QString m_cameraSection;
    SetupStatusChangedFunc m_setupStatusChanged;
    QLabel* m_pCameraSectionLabel = nullptr;
    QLabel* m_pCameraPathLabel = nullptr;
    CameraIpAddressEdit* m_pDeviceAddressEdit = nullptr;
    QLineEdit* m_pDevicePortEdit = nullptr;
    QLineEdit* m_pExposureTimeEdit = nullptr;
    QLineEdit* m_pGainLevelEdit = nullptr;
    QLineEdit* m_pCameraTypeEdit = nullptr;
    QLineEdit* m_pReadFpsEdit = nullptr;
    QCheckBox* m_pImageCaptureEnableCheck = nullptr;
    QLineEdit* m_pImageCaptureStrideEdit = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QString m_cleanSnapshot;
    bool m_savedThisSession = false;
};
