#pragma once

#include <QDialog>
#include <QString>
#include <QVector>

class QLabel;
class QCloseEvent;
class QLineEdit;
class QPlainTextEdit;

class HandEyeMatrixDialog : public QDialog
{
public:
    explicit HandEyeMatrixDialog(const QString& robotName, const QString& cameraSection, QWidget* parent = nullptr);

private:
    void closeEvent(QCloseEvent* event) override;
    bool LoadConfig();
    bool SaveConfig();
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);

    QString m_robotName;
    QString m_cameraSection;
    QLabel* m_pPathLabel = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QVector<QLineEdit*> m_rotationEdits;
    QVector<QLineEdit*> m_translationEdits;
    QString m_cleanSnapshot;
};
