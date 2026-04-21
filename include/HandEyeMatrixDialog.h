#pragma once

#include <QDialog>
#include <QString>
#include <QVector>

class QLabel;
class QLineEdit;
class QPlainTextEdit;

class HandEyeMatrixDialog : public QDialog
{
public:
    explicit HandEyeMatrixDialog(const QString& robotName, const QString& cameraSection, QWidget* parent = nullptr);

private:
    bool LoadConfig();
    bool SaveConfig();
    void AppendLog(const QString& text);

    QString m_robotName;
    QString m_cameraSection;
    QLabel* m_pPathLabel = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QVector<QLineEdit*> m_rotationEdits;
    QVector<QLineEdit*> m_translationEdits;
};
