#include "HandEyeMatrixDialog.h"

#include "HandEyeMatrixConfig.h"
#include "WindowStyleHelper.h"

#include <QCloseEvent>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>

namespace
{
QString FormatDoubleValue(double value)
{
    return QString::number(value, 'f', 12);
}
}

HandEyeMatrixDialog::HandEyeMatrixDialog(const QString& robotName, const QString& cameraSection, QWidget* parent)
    : QDialog(parent)
    , m_robotName(robotName)
    , m_cameraSection(cameraSection)
{
    setWindowTitle(QString("%1 %2 手眼矩阵参数").arg(robotName, cameraSection));
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(760, 620), 0.76, 0.76);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 8px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 8px; min-width: 120px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel(QString("手眼矩阵参数 - %1 / %2").arg(robotName, cameraSection));
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("公式：机器人坐标 = R_opt * 相机坐标 + t_opt。这里先维护 3x3 旋转矩阵和 3x1 平移向量。");
    hintLabel->setWordWrap(true);
    rootLayout->addWidget(hintLabel);

    m_pPathLabel = new QLabel("参数文件：");
    m_pPathLabel->setWordWrap(true);
    rootLayout->addWidget(m_pPathLabel);

    QGroupBox* rotationGroup = new QGroupBox("旋转矩阵 R_opt");
    QGridLayout* rotationLayout = new QGridLayout(rotationGroup);
    rotationLayout->setHorizontalSpacing(10);
    rotationLayout->setVerticalSpacing(10);
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            QLabel* label = new QLabel(QString("R%1%2").arg(row).arg(col));
            QLineEdit* edit = new QLineEdit();
            rotationLayout->addWidget(label, row, col * 2);
            rotationLayout->addWidget(edit, row, col * 2 + 1);
            m_rotationEdits.push_back(edit);
        }
    }
    rootLayout->addWidget(rotationGroup);

    QGroupBox* translationGroup = new QGroupBox("平移向量 t_opt (mm)");
    QGridLayout* translationLayout = new QGridLayout(translationGroup);
    translationLayout->setHorizontalSpacing(10);
    translationLayout->setVerticalSpacing(10);
    for (int index = 0; index < 3; ++index)
    {
        QLabel* label = new QLabel(QString("T%1").arg(index));
        QLineEdit* edit = new QLineEdit();
        translationLayout->addWidget(label, 0, index * 2);
        translationLayout->addWidget(edit, 0, index * 2 + 1);
        m_translationEdits.push_back(edit);
    }
    rootLayout->addWidget(translationGroup);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch(1);
    QPushButton* reloadBtn = new QPushButton("重新读取");
    QPushButton* saveBtn = new QPushButton("保存参数");
    QPushButton* closeBtn = new QPushButton("关闭");
    buttonLayout->addWidget(reloadBtn);
    buttonLayout->addWidget(saveBtn);
    buttonLayout->addWidget(closeBtn);
    rootLayout->addLayout(buttonLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setMinimumHeight(130);
    rootLayout->addWidget(m_pLogText, 1);

    connect(reloadBtn, &QPushButton::clicked, this, [this]() { LoadConfig(); });
    connect(saveBtn, &QPushButton::clicked, this, [this]() { SaveConfig(); });
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::close);

    LoadConfig();
}

void HandEyeMatrixDialog::closeEvent(QCloseEvent* event)
{
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "手眼矩阵参数", [this]() { return SaveConfig(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

bool HandEyeMatrixDialog::LoadConfig()
{
    HandEyeMatrixConfig config;
    QString error;
    QString filePath;
    if (!LoadHandEyeMatrixConfig(m_robotName, m_cameraSection, config, &error, &filePath))
    {
        QMessageBox::warning(this, "手眼矩阵参数", error);
        AppendLog("读取失败：" + error);
        return false;
    }

    m_pPathLabel->setText(QString("参数文件：%1").arg(filePath));
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            m_rotationEdits[row * 3 + col]->setText(FormatDoubleValue(config.rotation(row, col)));
        }
    }
    for (int index = 0; index < 3; ++index)
    {
        m_translationEdits[index]->setText(FormatDoubleValue(config.translation(index)));
    }
    AppendLog("已读取手眼矩阵参数。");
    MarkCleanSnapshot();
    return true;
}

bool HandEyeMatrixDialog::SaveConfig()
{
    HandEyeMatrixConfig config;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            bool ok = false;
            const double value = m_rotationEdits[row * 3 + col]->text().trimmed().toDouble(&ok);
            if (!ok)
            {
                QMessageBox::warning(this, "手眼矩阵参数", QString("旋转矩阵输入无效：R%1%2").arg(row).arg(col));
                return false;
            }
            config.rotation(row, col) = value;
        }
    }

    for (int index = 0; index < 3; ++index)
    {
        bool ok = false;
        const double value = m_translationEdits[index]->text().trimmed().toDouble(&ok);
        if (!ok)
        {
            QMessageBox::warning(this, "手眼矩阵参数", QString("平移向量输入无效：T%1").arg(index));
            return false;
        }
        config.translation(index) = value;
    }

    QString error;
    QString filePath;
    if (!SaveHandEyeMatrixConfig(m_robotName, m_cameraSection, config, &error, &filePath))
    {
        QMessageBox::warning(this, "手眼矩阵参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    m_pPathLabel->setText(QString("参数文件：%1").arg(filePath));
    AppendLog("手眼矩阵参数已保存。");
    QMessageBox::information(this, "手眼矩阵参数", "保存完成。");
    MarkCleanSnapshot();
    return true;
}

bool HandEyeMatrixDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString HandEyeMatrixDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << m_robotName << m_cameraSection;
    for (QLineEdit* edit : m_rotationEdits)
    {
        fields << (edit != nullptr ? edit->text().trimmed() : QString());
    }
    for (QLineEdit* edit : m_translationEdits)
    {
        fields << (edit != nullptr ? edit->text().trimmed() : QString());
    }
    return fields.join('\n');
}

void HandEyeMatrixDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void HandEyeMatrixDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(text);
    }
}
