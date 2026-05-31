#include "CameraBasicParamDialog.h"

#include "OPini.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QCloseEvent>
#include <QDir>
#include <QDoubleValidator>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QKeyEvent>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QList>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStringList>
#include <QTextDocument>
#include <QVBoxLayout>
#include <utility>

class CameraIpAddressEdit final : public QWidget
{
public:
    explicit CameraIpAddressEdit(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        QHBoxLayout* layout = new QHBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(3);
        for (int index = 0; index < 4; ++index)
        {
            QLineEdit* partEdit = new QLineEdit(this);
            partEdit->setFixedSize(48, 36);
            partEdit->setMaxLength(3);
            partEdit->setAlignment(Qt::AlignCenter);
            partEdit->setStyleSheet(
                "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; "
                "border-radius: 0px; padding: 4px 3px; }"
                "QLineEdit:focus { border-color: #72D4DD; }");
            partEdit->setValidator(new QIntValidator(0, 255, partEdit));
            partEdit->installEventFilter(this);
            MarkPartAsNumeric(partEdit);
            connect(partEdit, &QLineEdit::textEdited, this, [this, index](const QString& text)
                {
                    if (text.size() >= 3 && m_partEdits.value(index)->hasAcceptableInput())
                    {
                        FocusPart(index + 1);
                    }
                });
            m_partEdits.push_back(partEdit);
            layout->addWidget(partEdit);
            if (index < 3)
            {
                QLabel* dotLabel = new QLabel(".", this);
                dotLabel->setFixedWidth(6);
                dotLabel->setAlignment(Qt::AlignCenter);
                dotLabel->setStyleSheet("QLabel { color: #CFEFF5; }");
                layout->addWidget(dotLabel);
            }
        }
        setMinimumWidth(228);
        setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    }

    QString text() const
    {
        bool hasAnyPart = false;
        QStringList parts;
        for (const QLineEdit* partEdit : m_partEdits)
        {
            const QString part = partEdit != nullptr ? partEdit->text().trimmed() : QString();
            hasAnyPart = hasAnyPart || !part.isEmpty();
            parts.push_back(part);
        }
        return hasAnyPart ? parts.join(".") : QString();
    }

    void setText(const QString& ip)
    {
        const QStringList parts = ip.trimmed().split('.', Qt::KeepEmptyParts);
        for (int index = 0; index < m_partEdits.size(); ++index)
        {
            m_partEdits[index]->setText(index < parts.size() ? parts.at(index).trimmed().left(3) : QString());
        }
    }

    bool isComplete() const
    {
        for (const QLineEdit* partEdit : m_partEdits)
        {
            if (partEdit == nullptr || partEdit->text().trimmed().isEmpty() || !partEdit->hasAcceptableInput())
            {
                return false;
            }
        }
        return true;
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() != QEvent::KeyPress)
        {
            return QWidget::eventFilter(watched, event);
        }
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
        const int index = m_partEdits.indexOf(qobject_cast<QLineEdit*>(watched));
        if (index < 0)
        {
            return QWidget::eventFilter(watched, event);
        }
        if (keyEvent->key() == Qt::Key_Period || keyEvent->key() == Qt::Key_Comma)
        {
            FocusPart(index + 1);
            return true;
        }
        if (keyEvent->key() == Qt::Key_Backspace && m_partEdits[index]->text().isEmpty())
        {
            FocusPart(index - 1);
            return true;
        }
        return QWidget::eventFilter(watched, event);
    }

private:
    static void MarkPartAsNumeric(QLineEdit* edit)
    {
        if (edit == nullptr)
        {
            return;
        }
        edit->setProperty("touchKeyboardLayout", QStringLiteral("numeric"));
        edit->setInputMethodHints(Qt::ImhFormattedNumbersOnly);
    }

    void FocusPart(int index)
    {
        if (index < 0 || index >= m_partEdits.size() || m_partEdits[index] == nullptr)
        {
            return;
        }
        m_partEdits[index]->setFocus();
        m_partEdits[index]->selectAll();
    }

    QList<QLineEdit*> m_partEdits;
};

namespace
{
    std::string ToIniBytes(const QString& text)
    {
        return text.toUtf8().toStdString();
    }

    bool WriteRobotSetupReadyFlag(const QString& robotName, const QString& key, QString* error = nullptr)
    {
        const QString path = RobotDataHelper::BuildProjectPath(QString("Data/%1/RobotPara.ini").arg(robotName));
        COPini ini;
        if (!ini.SetFileName(false, ToIniBytes(path)))
        {
            if (error != nullptr)
            {
                *error = QString("打开机器人参数文件失败：%1").arg(path);
            }
            return false;
        }
        ini.SetSectionName("SetupStatus");
        if (!ini.WriteString(ToIniBytes(key), 1))
        {
            if (error != nullptr)
            {
                *error = QString("写入设置完成状态失败：%1 [%2]").arg(path, key);
            }
            return false;
        }
        return true;
    }

    void MarkNumericEdit(QLineEdit* edit)
    {
        if (edit == nullptr)
        {
            return;
        }
        edit->setProperty("touchKeyboardLayout", QStringLiteral("numeric"));
        edit->setInputMethodHints(Qt::ImhFormattedNumbersOnly);
    }

    QDoubleValidator* CreatePositiveDoubleValidator(QObject* parent)
    {
        QDoubleValidator* validator = new QDoubleValidator(0.0, 10000000.0, 6, parent);
        validator->setNotation(QDoubleValidator::StandardNotation);
        return validator;
    }
}

CameraBasicParamDialog::CameraBasicParamDialog(
    const QString& robotName,
    const QString& cameraSection,
    SetupStatusChangedFunc setupStatusChanged,
    QWidget* parent)
    : QDialog(parent)
    , m_robotName(robotName.trimmed().isEmpty() ? QString("RobotA") : robotName.trimmed())
    , m_cameraSection(cameraSection.trimmed().isEmpty() ? QString("CAMERA0") : cameraSection.trimmed())
    , m_setupStatusChanged(std::move(setupStatusChanged))
{
    setWindowTitle("相机基础参数");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(760, 520), 0.54, 0.56);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 8px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 9px 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 10px; min-height: 26px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(22, 18, 22, 20);
    rootLayout->setSpacing(12);

    QLabel* titleLabel = new QLabel("测量相机基础参数", this);
    titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("这里只维护相机连接和采集基础参数。保存后会标记该机器人相机参数已完成。", this);
    hintLabel->setWordWrap(true);
    hintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
    rootLayout->addWidget(hintLabel);

    QScrollArea* paramScrollArea = new QScrollArea(this);
    paramScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(paramScrollArea);

    QWidget* paramContent = new QWidget(paramScrollArea);
    paramContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    QVBoxLayout* paramContentLayout = new QVBoxLayout(paramContent);
    paramContentLayout->setContentsMargins(0, 0, 8, 0);
    paramContentLayout->setSpacing(10);
    paramContentLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    QGroupBox* cameraGroup = new QGroupBox("基础参数", this);
    QGridLayout* cameraLayout = new QGridLayout(cameraGroup);
    cameraLayout->setHorizontalSpacing(14);
    cameraLayout->setVerticalSpacing(12);
    cameraLayout->setColumnStretch(1, 1);
    cameraLayout->setColumnStretch(3, 1);

    m_pCameraSectionLabel = new QLabel(this);
    cameraLayout->addWidget(m_pCameraSectionLabel, 0, 0, 1, 4);
    m_pCameraPathLabel = new QLabel(this);
    m_pCameraPathLabel->setWordWrap(true);
    cameraLayout->addWidget(m_pCameraPathLabel, 1, 0, 1, 4);

    cameraLayout->addWidget(new QLabel("设备IP", this), 2, 0);
    m_pDeviceAddressEdit = new CameraIpAddressEdit(this);
    cameraLayout->addWidget(m_pDeviceAddressEdit, 2, 1);

    cameraLayout->addWidget(new QLabel("端口", this), 2, 2);
    m_pDevicePortEdit = new QLineEdit(this);
    m_pDevicePortEdit->setMinimumWidth(150);
    m_pDevicePortEdit->setAlignment(Qt::AlignRight);
    m_pDevicePortEdit->setValidator(new QIntValidator(1, 65535, m_pDevicePortEdit));
    MarkNumericEdit(m_pDevicePortEdit);
    cameraLayout->addWidget(m_pDevicePortEdit, 2, 3);

    cameraLayout->addWidget(new QLabel("曝光", this), 3, 0);
    m_pExposureTimeEdit = new QLineEdit(this);
    m_pExposureTimeEdit->setMinimumWidth(260);
    m_pExposureTimeEdit->setAlignment(Qt::AlignRight);
    m_pExposureTimeEdit->setValidator(CreatePositiveDoubleValidator(m_pExposureTimeEdit));
    MarkNumericEdit(m_pExposureTimeEdit);
    cameraLayout->addWidget(m_pExposureTimeEdit, 3, 1);

    cameraLayout->addWidget(new QLabel("增益", this), 3, 2);
    m_pGainLevelEdit = new QLineEdit(this);
    m_pGainLevelEdit->setMinimumWidth(150);
    m_pGainLevelEdit->setAlignment(Qt::AlignRight);
    m_pGainLevelEdit->setValidator(CreatePositiveDoubleValidator(m_pGainLevelEdit));
    MarkNumericEdit(m_pGainLevelEdit);
    cameraLayout->addWidget(m_pGainLevelEdit, 3, 3);

    cameraLayout->addWidget(new QLabel("相机类型", this), 4, 0);
    m_pCameraTypeEdit = new QLineEdit(this);
    m_pCameraTypeEdit->setMinimumWidth(260);
    m_pCameraTypeEdit->setAlignment(Qt::AlignRight);
    m_pCameraTypeEdit->setValidator(new QIntValidator(0, 9999, m_pCameraTypeEdit));
    MarkNumericEdit(m_pCameraTypeEdit);
    cameraLayout->addWidget(m_pCameraTypeEdit, 4, 1);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch(1);
    QPushButton* reloadBtn = new QPushButton("重新读取相机参数", this);
    QPushButton* saveBtn = new QPushButton("保存相机参数", this);
    reloadBtn->setFixedWidth(180);
    saveBtn->setFixedWidth(180);
    buttonLayout->addWidget(reloadBtn);
    buttonLayout->addWidget(saveBtn);
    cameraLayout->addLayout(buttonLayout, 5, 0, 1, 4);
    paramContentLayout->addWidget(cameraGroup);
    paramContentLayout->addStretch(1);
    paramScrollArea->setWidget(paramContent);
    rootLayout->addWidget(paramScrollArea, 1);

    m_pLogText = new QPlainTextEdit(this);
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(300);
    m_pLogText->setMinimumHeight(86);
    m_pLogText->setMaximumHeight(120);
    rootLayout->addWidget(m_pLogText);

    connect(reloadBtn, &QPushButton::clicked, this, [this]() { LoadCameraParam(); });
    connect(saveBtn, &QPushButton::clicked, this, [this]() { SaveCameraParam(); });

    m_pCameraSectionLabel->setText(QString("当前机器人：%1    当前分组：%2").arg(m_robotName, m_cameraSection));
    m_pCameraPathLabel->setText(QString("相机参数文件：%1").arg(RobotDataHelper::CameraParamPath(m_robotName)));
    LoadCameraParam();
}

void CameraBasicParamDialog::closeEvent(QCloseEvent* event)
{
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "相机基础参数", [this]() { return SaveCameraParam(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

bool CameraBasicParamDialog::LoadCameraParam()
{
    RobotDataHelper::CameraParamData param;
    QString error;
    if (!RobotDataHelper::LoadCameraParam(m_robotName, m_cameraSection, param, &error))
    {
        AppendLog(error);
        return false;
    }

    m_pDeviceAddressEdit->setText(param.deviceAddress);
    m_pDevicePortEdit->setText(param.devicePort);
    m_pExposureTimeEdit->setText(param.exposureTime);
    m_pGainLevelEdit->setText(param.gainLevel);
    m_pCameraTypeEdit->setText(param.cameraType);
    m_pCameraSectionLabel->setText(QString("当前机器人：%1    当前分组：%2").arg(m_robotName, param.sectionName));
    AppendLog(QString("已读取相机参数：%1 [%2]").arg(RobotDataHelper::CameraParamPath(m_robotName), param.sectionName));
    MarkCleanSnapshot();
    return true;
}

bool CameraBasicParamDialog::SaveCameraParam()
{
    RobotDataHelper::CameraParamData param;
    param.sectionName = m_cameraSection;
    param.deviceAddress = m_pDeviceAddressEdit->text().trimmed();
    param.devicePort = m_pDevicePortEdit->text().trimmed();
    param.exposureTime = m_pExposureTimeEdit->text().trimmed();
    param.gainLevel = m_pGainLevelEdit->text().trimmed();
    param.cameraType = m_pCameraTypeEdit->text().trimmed();

    QString error;
    if (m_pDeviceAddressEdit == nullptr || !m_pDeviceAddressEdit->isComplete())
    {
        error = "保存相机参数失败：设备IP必须是 4 段 0-255 的数字。";
        AppendLog(error);
        QMessageBox::warning(this, "相机基础参数", error);
        return false;
    }
    if (!RobotDataHelper::SaveCameraParam(m_robotName, param, &error))
    {
        AppendLog(error);
        return false;
    }

    AppendLog(QString("相机参数已保存：%1 [%2]").arg(RobotDataHelper::CameraParamPath(m_robotName), param.sectionName));
    QString setupError;
    if (WriteRobotSetupReadyFlag(m_robotName, "CameraParamReady", &setupError))
    {
        AppendLog("相机参数设置已完成，相关相机功能入口已允许启用。");
        m_savedThisSession = true;
        if (m_setupStatusChanged)
        {
            m_setupStatusChanged();
        }
    }
    else
    {
        AppendLog(setupError);
        return false;
    }
    MarkCleanSnapshot();
    return true;
}

bool CameraBasicParamDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString CameraBasicParamDialog::BuildSnapshot() const
{
    return QStringList{
        m_robotName,
        m_cameraSection,
        m_pDeviceAddressEdit != nullptr ? m_pDeviceAddressEdit->text().trimmed() : QString(),
        m_pDevicePortEdit != nullptr ? m_pDevicePortEdit->text().trimmed() : QString(),
        m_pExposureTimeEdit != nullptr ? m_pExposureTimeEdit->text().trimmed() : QString(),
        m_pGainLevelEdit != nullptr ? m_pGainLevelEdit->text().trimmed() : QString(),
        m_pCameraTypeEdit != nullptr ? m_pCameraTypeEdit->text().trimmed() : QString()
    }.join('\n');
}

void CameraBasicParamDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void CameraBasicParamDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(text);
    }
}
