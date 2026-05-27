#pragma once

#include <QDialog>
#include <QString>
#include <QVector>

class QLabel;
class QCloseEvent;
class QLineEdit;
class QPlainTextEdit;
class ContralUnit;
class RobotDriverAdaptor;

class HandEyeMatrixDialog : public QDialog
{
public:
    explicit HandEyeMatrixDialog(const QString& robotName, const QString& cameraSection, QWidget* parent = nullptr);
    explicit HandEyeMatrixDialog(ContralUnit* pContralUnit, const QString& robotName, const QString& cameraSection, QWidget* parent = nullptr);
    bool SavedThisSession() const { return m_bSavedThisSession; }

private:
    void closeEvent(QCloseEvent* event) override;
    bool LoadConfig();
    bool SaveConfig();
    bool ReadRobotEyeVariable();
    bool HasUnsavedChanges() const;
    RobotDriverAdaptor* CurrentDriver(QString* error = nullptr) const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);

    QString m_robotName;
    QString m_cameraSection;
    ContralUnit* m_pContralUnit = nullptr;
    QLabel* m_pPathLabel = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QVector<QLineEdit*> m_rotationEdits;
    QVector<QLineEdit*> m_translationEdits;
    bool m_bSavedThisSession = false;
    QString m_cleanSnapshot;
};
