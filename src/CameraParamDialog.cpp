#include "CameraParamDialog.h"

#include "CameraBasicParamDialog.h"
#include "HandEyeCalibrationDialog.h"
#include "HandEyeMatrixConfig.h"
#include "HandEyeMatrixDialog.h"
#include "ConfigSection.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QComboBox>
#include <QDir>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QStringList>
#include <QSplitter>
#include <QSizePolicy>
#include <QTextDocument>
#include <QVBoxLayout>
#include <utility>

namespace
{
    std::string ToConfigBytes(const QString& text)
    {
        return text.toUtf8().toStdString();
    }

    bool WriteRobotSetupReadyFlag(const QString& robotName, const QString& key, QString* error = nullptr)
    {
        ConfigSection ini;
        if (!ini.SetLocation(ConfigLocation::Robot(robotName, QStringLiteral("RobotPara"))))
        {
            if (error != nullptr)
            {
                *error = QString("机器人参数数据库位置无效：%1").arg(robotName);
            }
            return false;
        }
        ini.SetSectionName("SetupStatus");
        if (!ini.WriteString(ToConfigBytes(key), 1))
        {
            if (error != nullptr)
            {
                *error = QString("写入设置完成状态失败：%1 [%2]").arg(robotName, key);
            }
            return false;
        }
        return true;
    }
}

CameraParamDialog::CameraParamDialog(
    ContralUnit* pContralUnit,
    const QString& robotName,
    StartCameraFunc startCamera,
    StopCameraFunc stopCamera,
    CameraFrameCache* cameraCache,
    SetupStatusChangedFunc setupStatusChanged,
    QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_robotName(robotName.trimmed().isEmpty() ? QString("RobotA") : robotName.trimmed())
    , m_startCamera(startCamera)
    , m_stopCamera(stopCamera)
    , m_setupStatusChanged(std::move(setupStatusChanged))
    , m_pCameraCache(cameraCache)
{
    setWindowTitle("相机参数");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(1020, 660), 0.82, 0.76);
    setStyleSheet(QString(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 8px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 9px 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 10px; min-height: 26px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }")
        + UnifiedComboBoxStyleSheet());

    QVBoxLayout* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    outerLayout->setSpacing(0);
    QScrollArea* pageScrollArea = new QScrollArea(this);
    pageScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(pageScrollArea);
    outerLayout->addWidget(pageScrollArea);

    QWidget* pageWidget = new QWidget(pageScrollArea);
    pageWidget->setMinimumWidth(760);
    QVBoxLayout* rootLayout = new QVBoxLayout(pageWidget);
    rootLayout->setContentsMargins(18, 16, 18, 18);
    rootLayout->setSpacing(12);
    pageScrollArea->setWidget(pageWidget);

    QLabel* titleLabel = new QLabel("相机参数");
    titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
    titleLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    titleLabel->setMaximumHeight(44);
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("相机参数独立管理。当前支持手眼矩阵和测量相机基础连接参数，右侧预留图像显示区，后面可以直接接实时图像。");
    hintLabel->setWordWrap(true);
    hintLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    hintLabel->setMaximumHeight(56);
    rootLayout->addWidget(hintLabel);

    QGroupBox* baseGroup = new QGroupBox("基础信息");
    baseGroup->setMaximumHeight(176);
    baseGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    QVBoxLayout* baseLayout = new QVBoxLayout(baseGroup);
    QHBoxLayout* robotLayout = new QHBoxLayout();
    robotLayout->addWidget(new QLabel("机器人："));
    QLabel* robotNameLabel = new QLabel(m_robotName);
    robotNameLabel->setStyleSheet("QLabel { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 10px; min-height: 26px; }");
    robotLayout->addWidget(robotNameLabel, 1);
    robotLayout->addWidget(new QLabel("相机："));
    m_pCameraCombo = new QComboBox();
    robotLayout->addWidget(m_pCameraCombo, 1);
    baseLayout->addLayout(robotLayout);
    m_pPathLabel = new QLabel("手眼参数数据：");
    m_pPathLabel->setWordWrap(true);
    m_pPathLabel->setMaximumHeight(40);
    baseLayout->addWidget(m_pPathLabel);
    m_pCameraPathLabel = new QLabel("相机参数数据：");
    m_pCameraPathLabel->setWordWrap(true);
    m_pCameraPathLabel->setMaximumHeight(40);
    baseLayout->addWidget(m_pCameraPathLabel);
    rootLayout->addWidget(baseGroup);

    QSplitter* contentSplitter = new QSplitter(Qt::Horizontal, this);
    contentSplitter->setChildrenCollapsible(false);
    contentSplitter->setHandleWidth(8);

    QWidget* leftPanel = new QWidget(this);
    QVBoxLayout* leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(12);

    QGroupBox* handEyeGroup = new QGroupBox("手眼矩阵");
    handEyeGroup->setMaximumHeight(190);
    handEyeGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    QVBoxLayout* handEyeLayout = new QVBoxLayout(handEyeGroup);
    QPushButton* handEyeBtn = new QPushButton("手眼矩阵参数");
    QPushButton* handEyeCalibrationBtn = new QPushButton("手眼标定");
    handEyeBtn->setMinimumHeight(48);
    handEyeCalibrationBtn->setMinimumHeight(48);
    handEyeLayout->addWidget(handEyeBtn);
    handEyeLayout->addWidget(handEyeCalibrationBtn);
    handEyeLayout->addWidget(new QLabel("独立打开 3x3 旋转矩阵和 3x1 平移向量编辑窗口。"));
    leftLayout->addWidget(handEyeGroup);

    QGroupBox* cameraGroup = new QGroupBox("测量相机基础参数");
    cameraGroup->setMaximumHeight(170);
    cameraGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    QVBoxLayout* cameraLayout = new QVBoxLayout(cameraGroup);
    m_pCameraSectionLabel = new QLabel("当前分组：CAMERA0");
    cameraLayout->addWidget(m_pCameraSectionLabel);
    QLabel* cameraTip = new QLabel("相机 IP、端口、曝光、增益和相机类型现在在独立弹窗中维护，便于相机参数页和新建引导复用。");
    cameraTip->setWordWrap(true);
    cameraTip->setStyleSheet("QLabel { color: #8EB7BF; }");
    cameraLayout->addWidget(cameraTip);
    QPushButton* openCameraBasicBtn = new QPushButton("打开相机基础参数");
    openCameraBasicBtn->setMinimumHeight(46);
    cameraLayout->addWidget(openCameraBasicBtn);
    leftLayout->addWidget(cameraGroup);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(800);
    m_pLogText->setMinimumHeight(96);
    m_pLogText->setMaximumHeight(140);
    m_pLogText->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    leftLayout->addWidget(m_pLogText, 1);

    QGroupBox* previewGroup = new QGroupBox("图像显示预留区");
    QVBoxLayout* previewLayout = new QVBoxLayout(previewGroup);
    m_pImagePlaceholder = new QLabel("这里预留相机图像显示区域\n\n后续可以放：\n1. 实时图像\n2. 当前三维点信息\n3. 采集状态与触发状态");
    m_pImagePlaceholder->setAlignment(Qt::AlignCenter);
    m_pImagePlaceholder->setMinimumSize(240, 280);
    m_pImagePlaceholder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_pImagePlaceholder->setStyleSheet("QLabel { background: #081018; border: 1px dashed #385366; border-radius: 12px; color: #9ED8DB; font-size: 18px; }");
    previewLayout->addWidget(m_pImagePlaceholder, 1);
    previewGroup->setMinimumWidth(280);

    contentSplitter->addWidget(leftPanel);
    contentSplitter->addWidget(previewGroup);
    contentSplitter->setStretchFactor(0, 3);
    contentSplitter->setStretchFactor(1, 2);
    contentSplitter->setSizes({ 620, 360 });
    rootLayout->addWidget(contentSplitter, 1);

    connect(m_pCameraCombo, &QComboBox::currentIndexChanged, this, [this]() { UpdateCurrentCameraInfo(); });
    connect(openCameraBasicBtn, &QPushButton::clicked, this, [this]() { OpenCameraBasicParamDialog(); });
    connect(handEyeBtn, &QPushButton::clicked, this, [this]() { OpenHandEyeDialog(); });
    connect(handEyeCalibrationBtn, &QPushButton::clicked, this, [this]() { OpenHandEyeCalibrationDialog(); });

    LoadCameraList();
    UpdateCurrentCameraInfo();
}

void CameraParamDialog::LoadCameraList()
{
    if (m_pCameraCombo == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(m_pCameraCombo);
    m_pCameraCombo->clear();

    int selectedIndex = 0;
    const QVector<RobotDataHelper::CameraInfo> cameras = RobotDataHelper::LoadCameraList(CurrentRobotName(), &selectedIndex);
    for (const RobotDataHelper::CameraInfo& camera : cameras)
    {
        m_pCameraCombo->addItem(camera.displayName, camera.sectionName);
    }
    m_pCameraCombo->setCurrentIndex(selectedIndex >= 0 ? selectedIndex : 0);
}

void CameraParamDialog::UpdateCurrentCameraInfo()
{
    const QString robotName = CurrentRobotName();
    QString error;
    QString storageLabel;
    if (!EnsureHandEyeMatrixConfig(robotName, CurrentCameraSection(), &error, &storageLabel))
    {
        m_pPathLabel->setText("手眼参数数据：创建失败");
        AppendLog("手眼矩阵参数准备失败：" + error);
        return;
    }

    m_pPathLabel->setText(QString("手眼参数数据库：%1").arg(storageLabel));
    m_pCameraPathLabel->setText(QString("相机参数数据库：robot/%1/CameraParam").arg(robotName));
    if (m_pCameraSectionLabel != nullptr)
    {
        m_pCameraSectionLabel->setText(QString("当前分组：%1").arg(CurrentCameraSection()));
    }
    AppendLog(QString("当前机器人：%1，当前相机：%2").arg(robotName, CurrentCameraSection()));
}

void CameraParamDialog::OpenCameraBasicParamDialog()
{
    PauseExternalPreview();
    CameraBasicParamDialog dialog(
        CurrentRobotName(),
        CurrentCameraSection(),
        m_setupStatusChanged,
        this);
    dialog.exec();
    if (dialog.SavedThisSession())
    {
        AppendLog("相机基础参数已保存。");
    }
    UpdateCurrentCameraInfo();
}

void CameraParamDialog::OpenHandEyeDialog()
{
    PauseExternalPreview();
    HandEyeMatrixDialog dialog(m_pContralUnit, CurrentRobotName(), CurrentCameraSection(), this);
    dialog.exec();
    if (dialog.SavedThisSession())
    {
        QString setupError;
        if (WriteRobotSetupReadyFlag(CurrentRobotName(), "HandEyeReady", &setupError))
        {
            AppendLog("手眼矩阵已保存，手眼标定相关流程入口已允许启用。");
            if (m_setupStatusChanged)
            {
                m_setupStatusChanged();
            }
        }
        else
        {
            AppendLog(setupError);
        }
    }
    UpdateCurrentCameraInfo();
}

void CameraParamDialog::OpenHandEyeCalibrationDialog()
{
    PauseExternalPreview();
    HandEyeCalibrationDialog dialog(
        m_pContralUnit,
        CurrentRobotName(),
        CurrentCameraSection(),
        m_startCamera,
        m_stopCamera,
        m_pCameraCache,
        this);
    dialog.exec();
    if (dialog.MatrixComputedThisSession())
    {
        QString setupError;
        if (WriteRobotSetupReadyFlag(CurrentRobotName(), "HandEyeReady", &setupError))
        {
            AppendLog("手眼标定已完成，相关流程入口已允许启用。");
            if (m_setupStatusChanged)
            {
                m_setupStatusChanged();
            }
        }
        else
        {
            AppendLog(setupError);
        }
    }
    UpdateCurrentCameraInfo();
}

void CameraParamDialog::PauseExternalPreview()
{
    if (m_stopCamera)
    {
        m_stopCamera();
    }
}

QString CameraParamDialog::CurrentRobotName() const
{
    return m_robotName.isEmpty() ? QString("RobotA") : m_robotName;
}

QString CameraParamDialog::CurrentCameraSection() const
{
    return m_pCameraCombo != nullptr && m_pCameraCombo->currentIndex() >= 0
        ? m_pCameraCombo->currentData().toString()
        : QString("CAMERA0");
}

void CameraParamDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(text);
    }
}
