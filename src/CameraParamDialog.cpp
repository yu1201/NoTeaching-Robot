#include "CameraParamDialog.h"

#include "HandEyeMatrixConfig.h"
#include "HandEyeMatrixDialog.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QComboBox>
#include <QDir>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSplitter>
#include <QVBoxLayout>

CameraParamDialog::CameraParamDialog(ContralUnit* pContralUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
{
    setWindowTitle("相机参数");
    ApplyUnifiedWindowChrome(this);
    resize(980, 700);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 8px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 9px 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QComboBox, QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 10px; min-height: 26px; }"
        "QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 24px; border-left: 1px solid #385366; }"
        "QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("相机参数");
    titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("相机参数独立管理。当前支持手眼矩阵和测量相机基础连接参数，右侧预留图像显示区，后面可以直接接实时图像。");
    hintLabel->setWordWrap(true);
    rootLayout->addWidget(hintLabel);

    QGroupBox* baseGroup = new QGroupBox("基础信息");
    QVBoxLayout* baseLayout = new QVBoxLayout(baseGroup);
    QHBoxLayout* robotLayout = new QHBoxLayout();
    robotLayout->addWidget(new QLabel("机器人："));
    m_pRobotCombo = new QComboBox();
    robotLayout->addWidget(m_pRobotCombo, 1);
    robotLayout->addWidget(new QLabel("相机："));
    m_pCameraCombo = new QComboBox();
    robotLayout->addWidget(m_pCameraCombo, 1);
    baseLayout->addLayout(robotLayout);
    m_pPathLabel = new QLabel("手眼参数文件：");
    m_pPathLabel->setWordWrap(true);
    baseLayout->addWidget(m_pPathLabel);
    m_pCameraPathLabel = new QLabel("相机参数文件：");
    m_pCameraPathLabel->setWordWrap(true);
    baseLayout->addWidget(m_pCameraPathLabel);
    rootLayout->addWidget(baseGroup);

    QSplitter* contentSplitter = new QSplitter(Qt::Horizontal, this);
    contentSplitter->setChildrenCollapsible(false);

    QWidget* leftPanel = new QWidget(this);
    QVBoxLayout* leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(12);

    QGroupBox* handEyeGroup = new QGroupBox("手眼矩阵");
    QVBoxLayout* handEyeLayout = new QVBoxLayout(handEyeGroup);
    QPushButton* handEyeBtn = new QPushButton("手眼矩阵参数");
    handEyeBtn->setMinimumHeight(48);
    handEyeLayout->addWidget(handEyeBtn);
    handEyeLayout->addWidget(new QLabel("独立打开 3x3 旋转矩阵和 3x1 平移向量编辑窗口。"));
    leftLayout->addWidget(handEyeGroup);

    QGroupBox* cameraGroup = new QGroupBox("测量相机基础参数");
    QGridLayout* cameraLayout = new QGridLayout(cameraGroup);
    cameraLayout->setHorizontalSpacing(10);
    cameraLayout->setVerticalSpacing(10);
    m_pCameraSectionLabel = new QLabel("当前分组：CAMERA0");
    cameraLayout->addWidget(m_pCameraSectionLabel, 0, 0, 1, 4);
    cameraLayout->addWidget(new QLabel("设备IP"), 1, 0);
    m_pDeviceAddressEdit = new QLineEdit();
    cameraLayout->addWidget(m_pDeviceAddressEdit, 1, 1);
    cameraLayout->addWidget(new QLabel("端口"), 1, 2);
    m_pDevicePortEdit = new QLineEdit();
    cameraLayout->addWidget(m_pDevicePortEdit, 1, 3);
    cameraLayout->addWidget(new QLabel("曝光"), 2, 0);
    m_pExposureTimeEdit = new QLineEdit();
    cameraLayout->addWidget(m_pExposureTimeEdit, 2, 1);
    cameraLayout->addWidget(new QLabel("增益"), 2, 2);
    m_pGainLevelEdit = new QLineEdit();
    cameraLayout->addWidget(m_pGainLevelEdit, 2, 3);
    cameraLayout->addWidget(new QLabel("相机类型"), 3, 0);
    m_pCameraTypeEdit = new QLineEdit();
    cameraLayout->addWidget(m_pCameraTypeEdit, 3, 1);
    QPushButton* reloadBtn = new QPushButton("重新读取相机参数");
    QPushButton* saveBtn = new QPushButton("保存相机参数");
    cameraLayout->addWidget(reloadBtn, 4, 2);
    cameraLayout->addWidget(saveBtn, 4, 3);
    leftLayout->addWidget(cameraGroup);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setMinimumHeight(170);
    leftLayout->addWidget(m_pLogText, 1);

    QGroupBox* previewGroup = new QGroupBox("图像显示预留区");
    QVBoxLayout* previewLayout = new QVBoxLayout(previewGroup);
    m_pImagePlaceholder = new QLabel("这里预留相机图像显示区域\n\n后续可以放：\n1. 实时图像\n2. 当前三维点信息\n3. 采集状态与触发状态");
    m_pImagePlaceholder->setAlignment(Qt::AlignCenter);
    m_pImagePlaceholder->setMinimumSize(320, 420);
    m_pImagePlaceholder->setStyleSheet("QLabel { background: #081018; border: 1px dashed #385366; border-radius: 12px; color: #9ED8DB; font-size: 18px; }");
    previewLayout->addWidget(m_pImagePlaceholder, 1);

    contentSplitter->addWidget(leftPanel);
    contentSplitter->addWidget(previewGroup);
    contentSplitter->setStretchFactor(0, 3);
    contentSplitter->setStretchFactor(1, 2);
    rootLayout->addWidget(contentSplitter, 1);

    connect(m_pRobotCombo, &QComboBox::currentIndexChanged, this, [this]()
        {
            LoadCameraList();
            UpdateCurrentCameraInfo();
        });
    connect(m_pCameraCombo, &QComboBox::currentIndexChanged, this, [this]() { UpdateCurrentCameraInfo(); });
    connect(handEyeBtn, &QPushButton::clicked, this, [this]() { OpenHandEyeDialog(); });
    connect(reloadBtn, &QPushButton::clicked, this, [this]() { LoadCameraParam(); });
    connect(saveBtn, &QPushButton::clicked, this, [this]() { SaveCameraParam(); });

    LoadRobotList();
}

void CameraParamDialog::LoadRobotList()
{
    m_pRobotCombo->clear();
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        if (m_pRobotCombo->findData(info.robotName) < 0)
        {
            m_pRobotCombo->addItem(info.displayName, info.robotName);
        }
    }
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
    QString filePath;
    if (!EnsureHandEyeMatrixIni(robotName, CurrentCameraSection(), &error, &filePath))
    {
        m_pPathLabel->setText("手眼参数文件：创建失败");
        AppendLog("手眼矩阵文件准备失败：" + error);
        return;
    }

    m_pPathLabel->setText(QString("手眼参数文件：%1").arg(filePath));
    m_pCameraPathLabel->setText(QString("相机参数文件：%1").arg(RobotDataHelper::CameraParamPath(robotName)));
    AppendLog(QString("当前机器人：%1，当前相机：%2").arg(robotName, CurrentCameraSection()));
    LoadCameraParam();
}

void CameraParamDialog::OpenHandEyeDialog()
{
    HandEyeMatrixDialog dialog(CurrentRobotName(), CurrentCameraSection(), this);
    dialog.exec();
    UpdateCurrentCameraInfo();
}

QString CameraParamDialog::CurrentRobotName() const
{
    return m_pRobotCombo != nullptr ? m_pRobotCombo->currentData().toString() : QString("RobotA");
}

QString CameraParamDialog::CurrentCameraSection() const
{
    return m_pCameraCombo != nullptr && m_pCameraCombo->currentIndex() >= 0
        ? m_pCameraCombo->currentData().toString()
        : QString("CAMERA0");
}

QString CameraParamDialog::CurrentCameraIniPath() const
{
    return RobotDataHelper::CameraParamPath(CurrentRobotName());
}

bool CameraParamDialog::LoadCameraParam()
{
    RobotDataHelper::CameraParamData param;
    QString error;
    if (!RobotDataHelper::LoadCameraParam(CurrentRobotName(), CurrentCameraSection(), param, &error))
    {
        AppendLog(error);
        return false;
    }

    m_pDeviceAddressEdit->setText(param.deviceAddress);
    m_pDevicePortEdit->setText(param.devicePort);
    m_pExposureTimeEdit->setText(param.exposureTime);
    m_pGainLevelEdit->setText(param.gainLevel);
    m_pCameraTypeEdit->setText(param.cameraType);

    m_pCameraSectionLabel->setText(QString("当前分组：%1").arg(param.sectionName));
    AppendLog(QString("已读取相机参数：%1 [%2]").arg(CurrentCameraIniPath(), param.sectionName));
    return true;
}

bool CameraParamDialog::SaveCameraParam()
{
    RobotDataHelper::CameraParamData param;
    param.sectionName = CurrentCameraSection();
    param.deviceAddress = m_pDeviceAddressEdit->text().trimmed();
    param.devicePort = m_pDevicePortEdit->text().trimmed();
    param.exposureTime = m_pExposureTimeEdit->text().trimmed();
    param.gainLevel = m_pGainLevelEdit->text().trimmed();
    param.cameraType = m_pCameraTypeEdit->text().trimmed();

    QString error;
    if (!RobotDataHelper::SaveCameraParam(CurrentRobotName(), param, &error))
    {
        AppendLog(error);
        return false;
    }

    AppendLog(QString("相机参数已保存：%1 [%2]").arg(CurrentCameraIniPath(), param.sectionName));
    return true;
}

void CameraParamDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(text);
    }
}
