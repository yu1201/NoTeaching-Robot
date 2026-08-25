#include "ScanPoseVariationTestDialog.h"

#include "CameraFrameCache.h"
#include "ContralUnit.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "RobotDriverAdaptor.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"
#include "WindowStyleHelper.h"

#include <QApplication>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <thread>
#include <utility>

namespace
{
constexpr auto kConfigSection = "ScanPoseVariationTest";
constexpr auto kSelectionSection = "ScanPoseVariationSelection";

QString PoseText(const T_ROBOT_COORS& pose)
{
    return QStringLiteral("X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6 BX=%7 BY=%8 BZ=%9")
        .arg(pose.dX, 0, 'f', 3)
        .arg(pose.dY, 0, 'f', 3)
        .arg(pose.dZ, 0, 'f', 3)
        .arg(pose.dRX, 0, 'f', 3)
        .arg(pose.dRY, 0, 'f', 3)
        .arg(pose.dRZ, 0, 'f', 3)
        .arg(pose.dBX, 0, 'f', 3)
        .arg(pose.dBY, 0, 'f', 3)
        .arg(pose.dBZ, 0, 'f', 3);
}

QDoubleSpinBox* AddLengthEditor(
    QFormLayout* layout,
    const QString& label,
    double value,
    double minimum,
    double maximum,
    QWidget* parent)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(minimum, maximum);
    spin->setDecimals(2);
    spin->setSingleStep(5.0);
    spin->setValue(value);
    layout->addRow(label, CreateExternalUnitEditor(spin, QStringLiteral("mm"), parent));
    return spin;
}

QDoubleSpinBox* AddAngleEditor(
    QFormLayout* layout,
    const QString& label,
    double value,
    QWidget* parent)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(0.0, 60.0);
    spin->setDecimals(2);
    spin->setSingleStep(1.0);
    spin->setValue(value);
    layout->addRow(label, CreateExternalUnitEditor(spin, QStringLiteral("deg"), parent));
    return spin;
}
}

ScanPoseVariationTestDialog::ScanPoseVariationTestDialog(
    ContralUnit* controlUnit,
    int unitIndex,
    CameraFrameCache* cameraCache,
    StartCameraFunc startCamera,
    CameraCacheFunc cameraCacheForUnit,
    QWidget* parent)
    : QWidget(parent)
    , m_controlUnit(controlUnit)
    , m_unitIndex(unitIndex)
    , m_cameraCache(cameraCache)
    , m_startCamera(std::move(startCamera))
    , m_cameraCacheForUnit(std::move(cameraCacheForUnit))
{
    setObjectName(QStringLiteral("scanPoseVariationTestPage"));
    setWindowTitle(QStringLiteral("扫描变姿态精度测试"));
    setWindowFlags(Qt::Widget);
    setWindowModality(Qt::NonModal);
    setAttribute(Qt::WA_DeleteOnClose, false);
    setStyleSheet(
        "QWidget#scanPoseVariationTestPage { background: #101820; color: #E8F1F2; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 10px; margin-top: 15px; padding: 10px; color: #BFE8EC; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 9px; padding: 9px 13px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:disabled { background: #27323A; color: #7D8B91; }"
        "QDoubleSpinBox { background: #071017; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 8px; padding: 7px; }"
        "QFrame#scanInputSourceCard { background: #0B151D; border: 1px solid #294454; border-radius: 8px; }"
        "QLabel { color: #BACBD1; }");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    auto* title = new QLabel(QStringLiteral("扫描变姿态精度测试"), this);
    title->setStyleSheet(QStringLiteral(
        "font-size: 20px; font-weight: 700; color: #9ED8DB;"));
    root->addWidget(title);

    auto* scroll = new QScrollArea(this);
    scroll->setObjectName(QStringLiteral("AdaptiveWindowScrollArea"));
    ConfigureResponsiveScrollArea(scroll);
    auto* content = new QWidget(scroll);
    content->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    content->setMinimumWidth(900);
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(0, 0, 8, 0);
    contentLayout->setSpacing(12);
    contentLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    auto* explanation = new QLabel(
        QStringLiteral("测试目标：位置始终沿示教起点到终点的空间直线，只有机器人姿态按四段周期变化。"
            "变姿态合成与正式焊接示教姿态一致；扫描仍使用先测后焊的时间对齐、手眼变换、点云质量证明和结果目录。"
            "示教扫描起点和终点时都必须保持刚示教的基础姿态，软件会校验物理旋转差并保存起点关节脉冲用于翻腕风险判断。"),
        content);
    explanation->setWordWrap(true);
    contentLayout->addWidget(explanation);

    auto* targetGroup = new QGroupBox(QStringLiteral("测试对象与扫描输入"), content);
    auto* targetLayout = new QGridLayout(targetGroup);
    targetLayout->setHorizontalSpacing(24);
    targetLayout->setVerticalSpacing(8);
    m_robotCombo = new QComboBox(targetGroup);
    m_cameraCombo = new QComboBox(targetGroup);
    m_scanSpeedSpin = new QDoubleSpinBox(targetGroup);
    m_robotCombo->setMinimumWidth(280);
    m_cameraCombo->setMinimumWidth(280);
    m_scanSpeedSpin->setMinimumWidth(180);
    m_scanSpeedSpin->setRange(1.0, 30000.0);
    m_scanSpeedSpin->setDecimals(1);
    m_scanSpeedSpin->setSingleStep(60.0);
    m_scanSpeedSpin->setSuffix(QStringLiteral(" mm/min"));

    targetLayout->addWidget(new QLabel(QStringLiteral("使用机器人"), targetGroup), 0, 0, Qt::AlignRight | Qt::AlignVCenter);
    targetLayout->addWidget(m_robotCombo, 0, 1);
    targetLayout->addWidget(new QLabel(QStringLiteral("扫描相机"), targetGroup), 1, 0, Qt::AlignRight | Qt::AlignVCenter);
    targetLayout->addWidget(m_cameraCombo, 1, 1);
    targetLayout->addWidget(new QLabel(QStringLiteral("扫描速度"), targetGroup), 2, 0, Qt::AlignRight | Qt::AlignVCenter);
    targetLayout->addWidget(m_scanSpeedSpin, 2, 1);

    auto* sourceCard = new QFrame(targetGroup);
    sourceCard->setObjectName(QStringLiteral("scanInputSourceCard"));
    sourceCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    sourceCard->setMinimumWidth(380);
    auto* sourceLayout = new QVBoxLayout(sourceCard);
    sourceLayout->setContentsMargins(14, 10, 14, 10);
    sourceLayout->setSpacing(5);
    auto* sourceTitle = new QLabel(QStringLiteral("参数来源（自动继承）"), sourceCard);
    sourceTitle->setStyleSheet(QStringLiteral("color: #9ED8DB; font-weight: 600;"));
    auto* inheritedHint = new QLabel(
        QStringLiteral("机器人运动与时序：安全移动速度、安全位、加速度、减速度和相机时间补偿，"
            "继承所选机器人的当前先测后焊预设。"), sourceCard);
    inheritedHint->setWordWrap(true);
    auto* cameraHint = new QLabel(
        QStringLiteral("相机与空间坐标：扫描图像来自所选相机；运行前校验该相机对应且已验证的手眼矩阵。"),
        sourceCard);
    cameraHint->setWordWrap(true);
    sourceLayout->addWidget(sourceTitle);
    sourceLayout->addWidget(inheritedHint);
    sourceLayout->addWidget(cameraHint);
    sourceLayout->addStretch(1);

    targetLayout->addWidget(sourceCard, 0, 2, 3, 1);
    targetLayout->setColumnStretch(2, 1);
    contentLayout->addWidget(targetGroup);

    auto* teachGroup = new QGroupBox(QStringLiteral("示教并保存"), content);
    auto* teachLayout = new QGridLayout(teachGroup);
    m_teachBaseButton = new QPushButton(QStringLiteral("示教基础姿态并保存"), teachGroup);
    m_teachStartButton = new QPushButton(QStringLiteral("示教扫描起点并保存"), teachGroup);
    m_teachEndButton = new QPushButton(QStringLiteral("示教扫描终点并保存"), teachGroup);
    m_baseLabel = new QLabel(teachGroup);
    m_startLabel = new QLabel(teachGroup);
    m_endLabel = new QLabel(teachGroup);
    for (QLabel* label : { m_baseLabel, m_startLabel, m_endLabel })
    {
        label->setWordWrap(true);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    }
    teachLayout->addWidget(m_teachBaseButton, 0, 0);
    teachLayout->addWidget(m_baseLabel, 0, 1);
    teachLayout->addWidget(m_teachStartButton, 1, 0);
    teachLayout->addWidget(m_startLabel, 1, 1);
    teachLayout->addWidget(m_teachEndButton, 2, 0);
    teachLayout->addWidget(m_endLabel, 2, 1);
    teachLayout->setColumnStretch(1, 1);
    contentLayout->addWidget(teachGroup);

    auto* paramGroup = new QGroupBox(QStringLiteral("四段周期变姿态参数"), content);
    auto* paramColumns = new QHBoxLayout(paramGroup);
    auto* leftForm = new QFormLayout();
    auto* rightForm = new QFormLayout();
    m_lowPlatformSpin = AddLengthEditor(leftForm, QStringLiteral("下平台长度"), 30.0, 1.0, 2000.0, paramGroup);
    m_risingSpin = AddLengthEditor(leftForm, QStringLiteral("上坡长度"), 30.0, 1.0, 2000.0, paramGroup);
    m_highPlatformSpin = AddLengthEditor(leftForm, QStringLiteral("上平台长度"), 30.0, 1.0, 2000.0, paramGroup);
    m_fallingSpin = AddLengthEditor(leftForm, QStringLiteral("下坡长度"), 30.0, 1.0, 2000.0, paramGroup);
    m_leftAngleSpin = AddAngleEditor(rightForm, QStringLiteral("上坡左旋角度"), 10.0, paramGroup);
    m_rightAngleSpin = AddAngleEditor(rightForm, QStringLiteral("下坡右旋角度"), 10.0, paramGroup);
    m_transitionSpin = AddLengthEditor(rightForm, QStringLiteral("段尾姿态过渡长度"), 10.0, 0.1, 500.0, paramGroup);
    m_pointStepSpin = AddLengthEditor(rightForm, QStringLiteral("轨迹名义点距"), 2.0, 0.2, 20.0, paramGroup);
    paramColumns->addLayout(leftForm, 1);
    paramColumns->addLayout(rightForm, 1);
    contentLayout->addWidget(paramGroup);

    auto* actionRow = new QHBoxLayout();
    m_generateButton = new QPushButton(QStringLiteral("生成并保存扫描轨迹"), content);
    m_runButton = new QPushButton(QStringLiteral("运行扫描并保存数据"), content);
    actionRow->addWidget(m_generateButton);
    actionRow->addWidget(m_runButton);
    contentLayout->addLayout(actionRow);

    auto* imageGroup = new QGroupBox(QStringLiteral("扫描实时相机图像"), content);
    auto* imageLayout = new QVBoxLayout(imageGroup);
    m_liveImageStatusLabel = new QLabel(QStringLiteral("等待扫描启动并取得图像帧。"), imageGroup);
    m_liveImageLabel = new QLabel(
        QStringLiteral("相机图像：等待图像帧...\n（需所选相机图像传输口支持且已开启）"), imageGroup);
    m_liveImageLabel->setAlignment(Qt::AlignCenter);
    m_liveImageLabel->setMinimumHeight(320);
    m_liveImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    m_liveImageLabel->setStyleSheet(QStringLiteral(
        "QLabel { background: #071017; color: #6E8894; border: 1px solid #2B4552; border-radius: 8px; }"));
    imageLayout->addWidget(m_liveImageStatusLabel);
    imageLayout->addWidget(m_liveImageLabel, 1);
    contentLayout->addWidget(imageGroup);
    contentLayout->addStretch(1);
    scroll->setWidget(content);
    root->addWidget(scroll, 1);

    m_logEdit = new QPlainTextEdit(this);
    m_logEdit->setReadOnly(true);
    m_logEdit->setMinimumHeight(170);
    m_logEdit->document()->setMaximumBlockCount(2000);
    root->addWidget(m_logEdit);

    connect(m_teachBaseButton, &QPushButton::clicked, this, [this]() { TeachBasePose(); });
    connect(m_teachStartButton, &QPushButton::clicked, this, [this]() { TeachStartPose(); });
    connect(m_teachEndButton, &QPushButton::clicked, this, [this]() { TeachEndPose(); });
    connect(m_generateButton, &QPushButton::clicked, this, [this]() { GeneratePreview(); });
    connect(m_runButton, &QPushButton::clicked, this, [this]() { RunScan(); });

    m_liveImageTimer = new QTimer(this);
    m_liveImageTimer->setInterval(100);
    connect(m_liveImageTimer, &QTimer::timeout, this, [this]() { RefreshLiveImage(); });

    LoadRobotList(unitIndex);
    LoadCameraList();

    QString loadError;
    if (!LoadConfiguration(&loadError) && !loadError.isEmpty())
    {
        AppendLog(QStringLiteral("读取已保存示教/参数失败：") + loadError);
    }
    connect(m_cameraCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_loadingSelectors || m_running) return;
            QString error;
            if (!SaveConfiguration(&error))
                AppendLog(QStringLiteral("扫描相机选择保存失败：") + error);
        });
    connect(m_scanSpeedSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double)
        {
            if (m_loadingSelectors || m_running) return;
            QString error;
            if (!SaveConfiguration(&error))
                AppendLog(QStringLiteral("扫描速度保存失败：") + error);
        });
    UpdateStatusLabels();
    AppendLog(QStringLiteral("等待示教。运行前会再次显示基础姿态、空间起终点、周期参数和预计控制点，默认拒绝执行。"));
}

ScanPoseVariationTestDialog::~ScanPoseVariationTestDialog()
{
    if (m_liveImageTimer != nullptr)
    {
        m_liveImageTimer->stop();
    }
    if (m_cameraCache != nullptr)
    {
        m_cameraCache->SetLiveImageEnabled(false);
    }
}

void ScanPoseVariationTestDialog::LoadRobotList(int initialUnitIndex)
{
    if (m_robotCombo == nullptr) return;
    m_loadingSelectors = true;
    const QSignalBlocker blocker(m_robotCombo);
    m_robotCombo->clear();

    QString selectedRobotName;
    COPini selectionIni;
    std::string selectedRobotNameRaw;
    if (selectionIni.SetFileName(SelectionConfigPath().toUtf8().constData())
        && selectionIni.SetSectionName(kSelectionSection)
        && selectionIni.ReadString(false, "RobotName", selectedRobotNameRaw) > 0)
    {
        selectedRobotName = QString::fromUtf8(selectedRobotNameRaw.c_str()).trimmed();
    }

    int selectedRow = -1;
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_controlUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        if (info.unitIndex < 0) continue;
        const int row = m_robotCombo->count();
        m_robotCombo->addItem(info.displayName, info.unitIndex);
        m_robotCombo->setItemData(row, info.robotName, Qt::UserRole + 1);
        if ((!selectedRobotName.isEmpty() && info.robotName == selectedRobotName)
            || (selectedRobotName.isEmpty() && info.unitIndex == initialUnitIndex))
        {
            selectedRow = row;
        }
    }
    if (selectedRow < 0 && m_robotCombo->count() > 0) selectedRow = 0;
    if (selectedRow >= 0)
    {
        m_robotCombo->setCurrentIndex(selectedRow);
        m_unitIndex = m_robotCombo->currentData().toInt();
        m_cameraCache = m_cameraCacheForUnit ? m_cameraCacheForUnit(m_unitIndex) : nullptr;
    }
    m_robotCombo->setEnabled(m_robotCombo->count() > 0);
    m_loadingSelectors = false;
    connect(m_robotCombo, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int index) { ChangeRobot(index); });
}

void ScanPoseVariationTestDialog::LoadCameraList(const QString& preferredSection)
{
    if (m_cameraCombo == nullptr) return;
    const QSignalBlocker blocker(m_cameraCombo);
    m_cameraCombo->clear();
    int defaultIndex = 0;
    const QVector<RobotDataHelper::CameraInfo> cameras =
        RobotDataHelper::LoadCameraList(RobotName(), &defaultIndex);
    int selectedRow = -1;
    for (const RobotDataHelper::CameraInfo& camera : cameras)
    {
        const int row = m_cameraCombo->count();
        m_cameraCombo->addItem(camera.displayName, camera.sectionName);
        RobotDataHelper::CameraParamData cameraParam;
        RobotDataHelper::LoadCameraParam(RobotName(), camera.sectionName, cameraParam, nullptr);
        m_cameraCombo->setItemData(row,
            QStringLiteral("%1，IP=%2，图像保存=%3")
                .arg(camera.sectionName,
                    cameraParam.deviceAddress.trimmed().isEmpty()
                        ? QStringLiteral("未配置") : cameraParam.deviceAddress.trimmed(),
                    cameraParam.imageCaptureEnable.trimmed() == QStringLiteral("0")
                        ? QStringLiteral("关闭") : QStringLiteral("开启")),
            Qt::ToolTipRole);
        if (!preferredSection.trimmed().isEmpty()
            && camera.sectionName == preferredSection.trimmed())
        {
            selectedRow = row;
        }
    }
    if (selectedRow < 0) selectedRow = std::clamp(defaultIndex, 0, m_cameraCombo->count() - 1);
    if (selectedRow >= 0) m_cameraCombo->setCurrentIndex(selectedRow);
}

void ScanPoseVariationTestDialog::ChangeRobot(int comboIndex)
{
    if (m_loadingSelectors || m_running || comboIndex < 0) return;
    if (m_cameraCache != nullptr) m_cameraCache->SetLiveImageEnabled(false);
    m_unitIndex = m_robotCombo->itemData(comboIndex).toInt();
    m_cameraCache = m_cameraCacheForUnit ? m_cameraCacheForUnit(m_unitIndex) : nullptr;
    m_lastImageTimestamp = 0;
    LoadCameraList();
    QString error;
    if (!LoadConfiguration(&error) && !error.isEmpty())
    {
        AppendLog(QStringLiteral("切换机器人后读取测试配置失败：") + error);
    }
    if (!SaveSelection(&error))
    {
        AppendLog(QStringLiteral("机器人选择保存失败：") + error);
    }
    UpdateStatusLabels();
    if (m_liveImageLabel != nullptr)
    {
        m_liveImageLabel->setPixmap(QPixmap());
        m_liveImageLabel->setText(QStringLiteral("相机图像：等待扫描启动..."));
    }
    AppendLog(QStringLiteral("测试机器人已切换为：%1；已读取该机器人的独立示教与扫描参数。")
        .arg(RobotName()));
}

QString ScanPoseVariationTestDialog::SelectionConfigPath() const
{
    return RobotDataHelper::BuildProjectPath(QStringLiteral("Data/ScanPoseVariationTestSelection.ini"));
}

QString ScanPoseVariationTestDialog::CurrentCameraSection() const
{
    return m_cameraCombo != nullptr && m_cameraCombo->currentIndex() >= 0
        ? m_cameraCombo->currentData().toString().trimmed()
        : RobotDataHelper::MeasureCameraSection(RobotName());
}

bool ScanPoseVariationTestDialog::SaveSelection(QString* error) const
{
    if (error != nullptr) error->clear();
    const QString path = SelectionConfigPath();
    QDir().mkpath(QFileInfo(path).absolutePath());
    return RobotDataHelper::WriteParamValue(
        path, kSelectionSection, QStringLiteral("RobotName"), RobotName(), error);
}

void ScanPoseVariationTestDialog::RefreshLiveImage()
{
    if (!m_running || m_cameraCache == nullptr || m_liveImageLabel == nullptr) return;
    qint64 imageTimestamp = 0;
    const QImage image = m_cameraCache->LatestImage(&imageTimestamp);
    if (image.isNull() || imageTimestamp <= 0)
    {
        if (m_liveImageStatusLabel != nullptr)
        {
            m_liveImageStatusLabel->setText(
                QStringLiteral("正在等待 %1 / %2 的图像帧；点云扫描与图像口状态相互独立。")
                    .arg(RobotName(), CurrentCameraSection()));
        }
        return;
    }
    if (imageTimestamp == m_lastImageTimestamp) return;
    m_lastImageTimestamp = imageTimestamp;
    const QSize targetSize = m_liveImageLabel->contentsRect().size();
    m_liveImageLabel->setText(QString());
    m_liveImageLabel->setPixmap(QPixmap::fromImage(image).scaled(
        targetSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    if (m_liveImageStatusLabel != nullptr)
    {
        m_liveImageStatusLabel->setText(QStringLiteral("实时显示：%1 / %2，图像时间戳=%3")
            .arg(RobotName(), CurrentCameraSection()).arg(imageTimestamp));
    }
}

RobotDriverAdaptor* ScanPoseVariationTestDialog::ResolveDriver(bool showMessage) const
{
    RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_controlUnit, m_unitIndex);
    if (driver == nullptr && showMessage)
    {
        QMessageBox::warning(const_cast<ScanPoseVariationTestDialog*>(this),
            QStringLiteral("扫描变姿态精度测试"), QStringLiteral("当前控制单元未创建机器人驱动。"));
    }
    return driver;
}

QString ScanPoseVariationTestDialog::RobotName(RobotDriverAdaptor* driver) const
{
    if (driver == nullptr)
    {
        driver = ResolveDriver(false);
    }
    return driver != nullptr && !driver->RobotName().empty()
        ? QString::fromStdString(driver->RobotName())
        : QStringLiteral("RobotA");
}

QString ScanPoseVariationTestDialog::ConfigPath() const
{
    return RobotDataHelper::BuildProjectPath(
        QStringLiteral("Data/%1/FunctionTestScanPoseVariation.ini").arg(RobotName()));
}

bool ScanPoseVariationTestDialog::LoadConfiguration(QString* error)
{
    if (error != nullptr) error->clear();
    m_hasBasePose = false;
    m_hasStartPose = false;
    m_hasEndPose = false;
    m_hasStartPulse = false;

    QString savedCameraSection;
    const QString path = ConfigPath();
    COPini ini;
    if (ini.SetFileName(path.toUtf8().constData())
        && ini.SetSectionName(kConfigSection))
    {
        std::string cameraSectionRaw;
        if (ini.ReadString(false, "CameraSection", cameraSectionRaw) > 0)
        {
            savedCameraSection = QString::fromUtf8(cameraSectionRaw.c_str()).trimmed();
        }
    }
    LoadCameraList(savedCameraSection);

    T_PRECISE_MEASURE_PARAM presetParam;
    QString presetError;
    MeasureThenWeldService service;
    if (RobotDriverAdaptor* driver = ResolveDriver(false);
        driver != nullptr && service.LoadPresetParam(driver, presetParam, presetError)
        && std::isfinite(presetParam.dScanSpeed) && presetParam.dScanSpeed > 0.0)
    {
        m_scanSpeedSpin->setValue(presetParam.dScanSpeed);
    }

    if (!ini.SetFileName(path.toUtf8().constData())
        || !ini.SetSectionName(kConfigSection))
    {
        if (error != nullptr) *error = QStringLiteral("无法打开配置：") + path;
        return false;
    }

    int hasBase = 0;
    int hasStart = 0;
    int hasEnd = 0;
    int hasStartPulse = 0;
    ini.ReadString(false, "HasBasePose", &hasBase);
    ini.ReadString(false, "HasStartPose", &hasStart);
    ini.ReadString(false, "HasEndPose", &hasEnd);
    ini.ReadString(false, "HasStartPulse", &hasStartPulse);
    m_hasBasePose = hasBase != 0
        && RobotDataHelper::ReadCoors(path, kConfigSection, QStringLiteral("BasePose"), m_basePose, nullptr);
    m_hasStartPose = hasStart != 0
        && RobotDataHelper::ReadCoors(path, kConfigSection, QStringLiteral("StartPose"), m_startPose, nullptr);
    m_hasEndPose = hasEnd != 0
        && RobotDataHelper::ReadCoors(path, kConfigSection, QStringLiteral("EndPose"), m_endPose, nullptr);
    m_hasStartPulse = hasStartPulse != 0
        && RobotDataHelper::ReadPulse(path, kConfigSection, QStringLiteral("StartPulse"), m_startPulse, nullptr);

    auto readSpin = [&ini](const char* key, QDoubleSpinBox* spin)
        {
            double value = spin->value();
            if (ini.ReadString(false, key, &value) > 0 && std::isfinite(value))
            {
                spin->setValue(value);
            }
        };
    readSpin("LowPlatformLengthMm", m_lowPlatformSpin);
    readSpin("RisingLengthMm", m_risingSpin);
    readSpin("HighPlatformLengthMm", m_highPlatformSpin);
    readSpin("FallingLengthMm", m_fallingSpin);
    readSpin("LeftRotationDeg", m_leftAngleSpin);
    readSpin("RightRotationDeg", m_rightAngleSpin);
    readSpin("TransitionLengthMm", m_transitionSpin);
    readSpin("PointStepMm", m_pointStepSpin);
    readSpin("ScanSpeedMmPerMin", m_scanSpeedSpin);

    return true;
}

bool ScanPoseVariationTestDialog::SaveConfiguration(QString* error) const
{
    if (error != nullptr) error->clear();
    const QString path = ConfigPath();
    QDir().mkpath(QFileInfo(path).absolutePath());
    const auto params = CurrentParams();
    const QList<QPair<QString, QString>> values = {
        { QStringLiteral("Schema"), QStringLiteral("1") },
        { QStringLiteral("RobotName"), RobotName() },
        { QStringLiteral("CameraSection"), CurrentCameraSection() },
        { QStringLiteral("ScanSpeedMmPerMin"), QString::number(m_scanSpeedSpin->value(), 'f', 6) },
        { QStringLiteral("HasBasePose"), m_hasBasePose ? QStringLiteral("1") : QStringLiteral("0") },
        { QStringLiteral("HasStartPose"), m_hasStartPose ? QStringLiteral("1") : QStringLiteral("0") },
        { QStringLiteral("HasEndPose"), m_hasEndPose ? QStringLiteral("1") : QStringLiteral("0") },
        { QStringLiteral("HasStartPulse"), m_hasStartPulse ? QStringLiteral("1") : QStringLiteral("0") },
        { QStringLiteral("LowPlatformLengthMm"), QString::number(params.lowPlatformLengthMm, 'f', 6) },
        { QStringLiteral("RisingLengthMm"), QString::number(params.risingLengthMm, 'f', 6) },
        { QStringLiteral("HighPlatformLengthMm"), QString::number(params.highPlatformLengthMm, 'f', 6) },
        { QStringLiteral("FallingLengthMm"), QString::number(params.fallingLengthMm, 'f', 6) },
        { QStringLiteral("LeftRotationDeg"), QString::number(params.leftRotationDeg, 'f', 6) },
        { QStringLiteral("RightRotationDeg"), QString::number(params.rightRotationDeg, 'f', 6) },
        { QStringLiteral("TransitionLengthMm"), QString::number(params.transitionLengthMm, 'f', 6) },
        { QStringLiteral("PointStepMm"), QString::number(params.pointStepMm, 'f', 6) }
    };
    for (const auto& value : values)
    {
        if (!RobotDataHelper::WriteParamValue(path, kConfigSection, value.first, value.second, error))
        {
            return false;
        }
    }
    if (m_hasBasePose
        && !RobotDataHelper::WriteCoors(path, kConfigSection, QStringLiteral("BasePose"), m_basePose, error))
    {
        return false;
    }
    if (m_hasStartPose
        && !RobotDataHelper::WriteCoors(path, kConfigSection, QStringLiteral("StartPose"), m_startPose, error))
    {
        return false;
    }
    if (m_hasEndPose
        && !RobotDataHelper::WriteCoors(path, kConfigSection, QStringLiteral("EndPose"), m_endPose, error))
    {
        return false;
    }
    if (m_hasStartPulse
        && !RobotDataHelper::WritePulse(path, kConfigSection, QStringLiteral("StartPulse"), m_startPulse, error))
    {
        return false;
    }
    return true;
}

bool ScanPoseVariationTestDialog::ReadCurrentPose(T_ROBOT_COORS& pose, QString& error) const
{
    RobotDriverAdaptor* driver = ResolveDriver(false);
    if (driver != nullptr && !driver->Supports(RobotDriverCapability::PassiveState))
    {
        error = QStringLiteral(
            "当前机器人品牌底层缺少“机器人状态读取”适配能力，示教功能已限制。");
        return false;
    }
    if (driver == nullptr || !driver->IsConnected())
    {
        error = QStringLiteral("机器人未连接。");
        return false;
    }
    if (!driver->TryGetCurrentPos(pose))
    {
        error = DecodeRobotMessageText(driver->GetLastRobotError());
        if (error.isEmpty()) error = QStringLiteral("机器人未返回可验证的直角位姿。");
        return false;
    }
    return true;
}

bool ScanPoseVariationTestDialog::ReadCurrentPulse(T_ANGLE_PULSE& pulse, QString& error) const
{
    RobotDriverAdaptor* driver = ResolveDriver(false);
    if (driver != nullptr && !driver->Supports(RobotDriverCapability::PassiveState))
    {
        error = QStringLiteral(
            "当前机器人品牌底层缺少“机器人状态读取”适配能力，示教功能已限制。");
        return false;
    }
    if (driver == nullptr || !driver->TryGetCurrentPulse(pulse))
    {
        error = driver == nullptr
            ? QStringLiteral("机器人驱动不可用。")
            : DecodeRobotMessageText(driver->GetLastRobotError());
        return false;
    }
    return true;
}

void ScanPoseVariationTestDialog::TeachBasePose()
{
    QString error;
    T_ROBOT_COORS pose;
    if (!ReadCurrentPose(pose, error))
    {
        QMessageBox::warning(this, QStringLiteral("示教基础姿态"), error);
        return;
    }
    m_basePose = pose;
    m_hasBasePose = true;
    if (!SaveConfiguration(&error))
    {
        m_hasBasePose = false;
        QMessageBox::warning(this, QStringLiteral("示教基础姿态"), QStringLiteral("保存失败：") + error);
        return;
    }
    UpdateStatusLabels();
    AppendLog(QStringLiteral("基础姿态已示教并保存：") + PoseText(pose));
}

void ScanPoseVariationTestDialog::TeachStartPose()
{
    QString error;
    T_ROBOT_COORS pose;
    T_ANGLE_PULSE pulse;
    if (!ReadCurrentPose(pose, error) || !ReadCurrentPulse(pulse, error))
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"), error);
        return;
    }
    m_startPose = pose;
    m_startPulse = pulse;
    m_hasStartPose = true;
    m_hasStartPulse = true;
    if (!SaveConfiguration(&error))
    {
        m_hasStartPose = false;
        m_hasStartPulse = false;
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"), QStringLiteral("保存失败：") + error);
        return;
    }
    UpdateStatusLabels();
    AppendLog(QStringLiteral("扫描起点（含关节脉冲）已示教并保存：") + PoseText(pose));
}

void ScanPoseVariationTestDialog::TeachEndPose()
{
    QString error;
    T_ROBOT_COORS pose;
    if (!ReadCurrentPose(pose, error))
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"), error);
        return;
    }
    m_endPose = pose;
    m_hasEndPose = true;
    if (!SaveConfiguration(&error))
    {
        m_hasEndPose = false;
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"), QStringLiteral("保存失败：") + error);
        return;
    }
    UpdateStatusLabels();
    AppendLog(QStringLiteral("扫描终点已示教并保存：") + PoseText(pose));
}

MeasureThenWeldService::ScanPoseVariationParams ScanPoseVariationTestDialog::CurrentParams() const
{
    MeasureThenWeldService::ScanPoseVariationParams params;
    params.lowPlatformLengthMm = m_lowPlatformSpin->value();
    params.risingLengthMm = m_risingSpin->value();
    params.highPlatformLengthMm = m_highPlatformSpin->value();
    params.fallingLengthMm = m_fallingSpin->value();
    params.leftRotationDeg = m_leftAngleSpin->value();
    params.rightRotationDeg = m_rightAngleSpin->value();
    params.transitionLengthMm = m_transitionSpin->value();
    params.pointStepMm = m_pointStepSpin->value();
    return params;
}

void ScanPoseVariationTestDialog::GeneratePreview()
{
    if (!m_hasBasePose || !m_hasStartPose || !m_hasEndPose)
    {
        QMessageBox::information(this, QStringLiteral("生成扫描轨迹"),
            QStringLiteral("请先完成基础姿态、扫描起点和扫描终点示教。"));
        return;
    }
    RobotDriverAdaptor* driver = ResolveDriver();
    if (driver == nullptr) return;
    QString error;
    if (!SaveSelection(&error))
    {
        QMessageBox::warning(this, QStringLiteral("生成扫描轨迹"), QStringLiteral("保存机器人选择失败：") + error);
        return;
    }
    const int robotType = driver->DriverDescriptor().poseConventionType;
    MeasureThenWeldService service;
    QVector<MeasureThenWeldService::ScanPoseVariationPoint> trajectory;
    QString summary;
    if (!service.GenerateScanPoseVariationTrajectory(
            m_basePose, m_startPose, m_endPose, robotType,
            CurrentParams(), trajectory, summary, error))
    {
        AppendLog(QStringLiteral("轨迹生成失败：") + error);
        QMessageBox::warning(this, QStringLiteral("生成扫描轨迹"), error);
        return;
    }
    if (!SaveConfiguration(&error))
    {
        QMessageBox::warning(this, QStringLiteral("生成扫描轨迹"), QStringLiteral("保存参数失败：") + error);
        return;
    }
    const QString dir = RobotDataHelper::BuildProjectPath(
        QStringLiteral("Result/%1/ScanPoseVariationPlans").arg(RobotName(driver)));
    QDir().mkpath(dir);
    const QString path = QDir(dir).filePath(QStringLiteral("CommandedTrajectory_%1.csv")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    if (!service.SaveScanPoseVariationTrajectory(path, trajectory, error))
    {
        QMessageBox::warning(this, QStringLiteral("生成扫描轨迹"), error);
        return;
    }
    AppendLog(summary);
    AppendLog(QStringLiteral("命令轨迹已保存：") + QDir::toNativeSeparators(path));
    QMessageBox::information(this, QStringLiteral("生成扫描轨迹"),
        summary + QStringLiteral("\n\n文件：") + QDir::toNativeSeparators(path));
}

void ScanPoseVariationTestDialog::RunScan()
{
    if (m_running) return;
    if (!m_hasBasePose || !m_hasStartPose || !m_hasEndPose || !m_hasStartPulse)
    {
        QMessageBox::information(this, QStringLiteral("运行扫描"),
            QStringLiteral("请先完成基础姿态、扫描起点（含关节脉冲）和扫描终点示教。"));
        return;
    }
    RobotDriverAdaptor* driver = ResolveDriver();
    if (driver == nullptr || !driver->IsConnected())
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), QStringLiteral("机器人未连接。"));
        return;
    }
    if (!driver->SupportsAll(
        { RobotDriverCapability::JointMotion,
          RobotDriverCapability::LinearMotion,
          RobotDriverCapability::PassiveState,
          RobotDriverCapability::ContinuousTrajectory,
          RobotDriverCapability::VerifiedProgramCompletion,
          RobotDriverCapability::VerifiedSafeAbort }))
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"),
            QStringLiteral("当前机器人品牌底层无法执行扫描变姿态测试，缺少适配能力：%1；功能已限制。")
            .arg(QString::fromUtf8(driver->MissingCapabilitiesText(
                { RobotDriverCapability::JointMotion,
                  RobotDriverCapability::LinearMotion,
                  RobotDriverCapability::PassiveState,
                  RobotDriverCapability::ContinuousTrajectory,
                  RobotDriverCapability::VerifiedProgramCompletion,
                  RobotDriverCapability::VerifiedSafeAbort }).c_str())));
        return;
    }

    const QString cameraSection = CurrentCameraSection();
    if (cameraSection.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), QStringLiteral("请选择扫描相机。"));
        return;
    }
    RobotDataHelper::CameraParamData cameraParam;
    if (!RobotDataHelper::LoadCameraParam(RobotName(driver), cameraSection, cameraParam, nullptr)
        || cameraParam.deviceAddress.trimmed().isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"),
            QStringLiteral("所选相机 %1 未配置有效 DeviceAddress。").arg(cameraSection));
        return;
    }

    QString cameraIp;
    if (m_startCamera && !m_startCamera(m_unitIndex, cameraSection, cameraIp))
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"),
            QStringLiteral("扫描相机 %1（%2）未能连接并取得新帧。")
                .arg(cameraSection, cameraParam.deviceAddress));
        return;
    }
    if (m_cameraCacheForUnit)
    {
        m_cameraCache = m_cameraCacheForUnit(m_unitIndex);
    }
    if (m_cameraCache == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), QStringLiteral("当前机器人没有可用的专属扫描相机缓存。"));
        return;
    }

    const int robotType = driver->DriverDescriptor().poseConventionType;
    MeasureThenWeldService service;
    QVector<MeasureThenWeldService::ScanPoseVariationPoint> generated;
    QString summary;
    QString error;
    if (!service.GenerateScanPoseVariationTrajectory(
            m_basePose, m_startPose, m_endPose, robotType,
            CurrentParams(), generated, summary, error))
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), error);
        return;
    }
    if (!SaveConfiguration(&error))
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), QStringLiteral("保存示教/参数失败：") + error);
        return;
    }
    if (!SaveSelection(&error))
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), QStringLiteral("保存机器人选择失败：") + error);
        return;
    }

    const QString confirmation = QStringLiteral(
        "即将进行真实机器人扫描运动。\n\n机器人：%1\n扫描相机：%2（%3）\n扫描速度：%4 mm/min\n"
        "基础姿态：%5\n扫描起点：%6\n扫描终点：%7\n%8\n\n"
        "流程：到扫描下枪安全位 -> 扫描起点 -> 连续变姿态扫描并采集 -> 扫描收枪安全位。\n"
        "软件停止不能替代控制柜/示教器急停；请确认机器人周围安全、相机和激光已准备好。是否继续？")
        .arg(RobotName(driver), cameraSection, cameraIp)
        .arg(m_scanSpeedSpin->value(), 0, 'f', 1)
        .arg(PoseText(m_basePose), PoseText(m_startPose), PoseText(m_endPose), summary);
    if (QMessageBox::question(this, QStringLiteral("扫描变姿态运行前确认"), confirmation,
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("管理调试-扫描变姿态精度测试"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, QStringLiteral("运行扫描"), leaseError);
        return;
    }

    std::vector<T_ROBOT_COORS> trajectory;
    trajectory.reserve(generated.size());
    for (const auto& point : generated)
    {
        trajectory.push_back(point.pose);
    }
    const auto params = CurrentParams();
    const T_ROBOT_COORS basePose = m_basePose;
    const T_ROBOT_COORS startPose = m_startPose;
    const T_ROBOT_COORS endPose = m_endPose;
    const T_ANGLE_PULSE startPulse = m_startPulse;
    const QString robotName = RobotName(driver);
    const QString selectedCameraSection = cameraSection;
    const QString selectedCameraIp = cameraIp;
    const double scanSpeedMmPerMin = m_scanSpeedSpin->value();
    CameraFrameCache* cameraCache = m_cameraCache;
    SetRunning(true);
    AppendLog(QStringLiteral("———— 开始扫描变姿态精度测试 ————"));
    AppendLog(summary);

    QPointer<ScanPoseVariationTestDialog> self(this);
    std::thread([self, driver, cameraCache, operationLease, trajectory, generated,
                 params, basePose, startPose, endPose, startPulse, robotType,
                 robotName, selectedCameraSection, selectedCameraIp, scanSpeedMmPerMin]()
        {
            MeasureThenWeldService service;
            T_PRECISE_MEASURE_PARAM param;
            QString error;
            if (!service.LoadPresetParam(driver, param, error))
            {
                QMetaObject::invokeMethod(qApp, [self, error]()
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QStringLiteral("读取当前扫描预设失败：") + error);
                            self->SetRunning(false);
                        }
                    }, Qt::QueuedConnection);
                return;
            }
            // 安全位和进度口径都必须使用机器人真正执行的首末姿态，而不是仅含位置的示教点。
            param.tStartPos = trajectory.front();
            param.tEndPos = trajectory.back();
            param.tStartPulse = startPulse;
            param.bHasStartPulse = true;
            param.dScanSpeed = scanSpeedMmPerMin;

            auto appendLog = [self](const QString& text)
                {
                    QMetaObject::invokeMethod(qApp, [self, text]()
                        {
                            if (self != nullptr) self->AppendLog(text);
                    }, Qt::QueuedConnection);
                };
            appendLog(QStringLiteral(
                "本轮锁定输入：机器人=%1，相机=%2（%3），扫描速度=%4 mm/min；"
                "继承预设=%5，安全移动速度=%6 mm/min，安全位模式=%7，加速=%8，减速=%9，"
                "相机时间补偿=%10 ms，时间对齐=%11。")
                .arg(robotName, selectedCameraSection, selectedCameraIp)
                .arg(scanSpeedMmPerMin, 0, 'f', 3)
                .arg(param.sParamGroupName)
                .arg(param.dRunSpeed, 0, 'f', 3)
                .arg(param.bUseComputedScanSafe ? QStringLiteral("自动计算") : QStringLiteral("示教脉冲"))
                .arg(param.dAcc, 0, 'f', 6)
                .arg(param.dDec, 0, 'f', 6)
                .arg(param.dCameraTimeOffsetMs, 0, 'f', 3)
                .arg(param.bUseStatTimeAlign ? QStringLiteral("统计") : QStringLiteral("首帧")));
            auto setFlowStep = [self](const QString& text)
                {
                    QMetaObject::invokeMethod(qApp, [self, text]()
                        {
                            if (self != nullptr) self->AppendLog(QStringLiteral("步骤：") + text);
                        }, Qt::QueuedConnection);
                };
            auto checkpoint = [self](const QString& title, const QString& detail) -> bool
                {
                    bool approved = false;
                    QMetaObject::invokeMethod(qApp, [self, title, detail, &approved]()
                        {
                            if (self != nullptr)
                            {
                                approved = QMessageBox::question(self, title, detail,
                                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes;
                            }
                        }, Qt::BlockingQueuedConnection);
                    return approved;
                };

            MeasureThenWeldService::ScanCycleResult result;
            const bool ok = service.RunScanCycle(
                driver,
                param,
                param.dRunSpeed,
                cameraCache,
                result,
                appendLog,
                setFlowStep,
                checkpoint,
                MeasureThenWeldService::BeforeActionCallback(),
                [driver]() { return RobotOperationLease::IsCancellationRequested(driver); },
                MeasureThenWeldService::ScanProgressCallback(),
                MeasureThenWeldService::ScanPauseAvailabilityCallback(),
                &trajectory,
                selectedCameraSection);

            QString commandedPath;
            QString saveError;
            if (!result.caseDir.isEmpty())
            {
                commandedPath = QDir(result.caseDir).filePath(
                    QStringLiteral("ScanPoseVariation_CommandedTrajectory.csv"));
                service.SaveScanPoseVariationTrajectory(commandedPath, generated, saveError);
                const QString metadataPath = QDir(result.caseDir).filePath(
                    QStringLiteral("ScanPoseVariation_TestSummary.txt"));
                std::vector<QString> metadata = {
                    QStringLiteral("mode=scan_pose_variation_accuracy_test"),
                    QStringLiteral("robot=%1").arg(robotName),
                    QStringLiteral("camera_section=%1").arg(selectedCameraSection),
                    QStringLiteral("camera_ip=%1").arg(selectedCameraIp),
                    QStringLiteral("scan_speed_mm_per_min=%1").arg(scanSpeedMmPerMin, 0, 'f', 6),
                    QStringLiteral("preset_group=%1").arg(param.sParamGroupName),
                    QStringLiteral("safe_move_speed_mm_per_min=%1").arg(param.dRunSpeed, 0, 'f', 6),
                    QStringLiteral("scan_safe_mode=%1").arg(
                        param.bUseComputedScanSafe ? QStringLiteral("computed") : QStringLiteral("taught_pulse")),
                    QStringLiteral("acceleration=%1").arg(param.dAcc, 0, 'f', 6),
                    QStringLiteral("deceleration=%1").arg(param.dDec, 0, 'f', 6),
                    QStringLiteral("camera_time_offset_ms=%1").arg(param.dCameraTimeOffsetMs, 0, 'f', 6),
                    QStringLiteral("time_alignment=%1").arg(
                        param.bUseStatTimeAlign ? QStringLiteral("statistical") : QStringLiteral("first_frame")),
                    QStringLiteral("base_pose=%1").arg(PoseText(basePose)),
                    QStringLiteral("start_pose=%1").arg(PoseText(startPose)),
                    QStringLiteral("end_pose=%1").arg(PoseText(endPose)),
                    QStringLiteral("low_platform_mm=%1").arg(params.lowPlatformLengthMm, 0, 'f', 6),
                    QStringLiteral("rising_mm=%1").arg(params.risingLengthMm, 0, 'f', 6),
                    QStringLiteral("high_platform_mm=%1").arg(params.highPlatformLengthMm, 0, 'f', 6),
                    QStringLiteral("falling_mm=%1").arg(params.fallingLengthMm, 0, 'f', 6),
                    QStringLiteral("left_rotation_deg=%1").arg(params.leftRotationDeg, 0, 'f', 6),
                    QStringLiteral("right_rotation_deg=%1").arg(params.rightRotationDeg, 0, 'f', 6),
                    QStringLiteral("transition_mm=%1").arg(params.transitionLengthMm, 0, 'f', 6),
                    QStringLiteral("point_step_mm=%1").arg(params.pointStepMm, 0, 'f', 6),
                    QStringLiteral("robot_type=%1").arg(robotType),
                    QStringLiteral("scan_status=%1").arg(ok ? QStringLiteral("success") : QStringLiteral("failed")),
                    QStringLiteral("scan_error=%1").arg(result.error)
                };
                QString metadataError;
                service.SaveTextLines(metadataPath, metadata, metadataError);
                if (!metadataError.isEmpty())
                {
                    saveError += (saveError.isEmpty() ? QString() : QStringLiteral("；")) + metadataError;
                }
            }

            QMetaObject::invokeMethod(qApp,
                [self, ok, result, commandedPath, saveError]()
                {
                    if (self == nullptr) return;
                    if (ok)
                    {
                        self->AppendLog(QStringLiteral("扫描变姿态测试完成并已安全收枪。结果目录：")
                            + QDir::toNativeSeparators(result.caseDir));
                    }
                    else
                    {
                        self->AppendLog(QStringLiteral("扫描变姿态测试失败/中止：") + result.error);
                    }
                    if (!commandedPath.isEmpty())
                    {
                        self->AppendLog(QStringLiteral("本轮命令姿态轨迹：")
                            + QDir::toNativeSeparators(commandedPath));
                    }
                    if (!saveError.isEmpty())
                    {
                        self->AppendLog(QStringLiteral("测试元数据保存告警：") + saveError);
                    }
                    self->SetRunning(false);
                }, Qt::QueuedConnection);
        }).detach();
}

void ScanPoseVariationTestDialog::AppendLog(const QString& text)
{
    if (m_logEdit != nullptr) m_logEdit->appendPlainText(text);
}

void ScanPoseVariationTestDialog::UpdateStatusLabels()
{
    m_baseLabel->setText(m_hasBasePose
        ? QStringLiteral("基础姿态（仅使用RX/RY/RZ）：") + PoseText(m_basePose)
        : QStringLiteral("基础姿态：未示教"));
    m_startLabel->setText(m_hasStartPose
        ? QStringLiteral("扫描空间起点：") + PoseText(m_startPose)
            + (m_hasStartPulse ? QStringLiteral("（关节脉冲已保存）") : QStringLiteral("（缺少关节脉冲）"))
        : QStringLiteral("扫描起点：未示教"));
    m_endLabel->setText(m_hasEndPose
        ? QStringLiteral("扫描终点：") + PoseText(m_endPose)
        : QStringLiteral("扫描终点：未示教"));
}

void ScanPoseVariationTestDialog::SetRunning(bool running)
{
    m_running = running;
    if (running)
    {
        m_lastImageTimestamp = 0;
        if (m_cameraCache != nullptr) m_cameraCache->SetLiveImageEnabled(true);
        if (m_liveImageTimer != nullptr) m_liveImageTimer->start();
        if (m_liveImageStatusLabel != nullptr)
        {
            m_liveImageStatusLabel->setText(
                QStringLiteral("正在启动 %1 / %2 实时图像...").arg(RobotName(), CurrentCameraSection()));
        }
    }
    else
    {
        if (m_liveImageTimer != nullptr) m_liveImageTimer->stop();
        if (m_cameraCache != nullptr) m_cameraCache->SetLiveImageEnabled(false);
        if (m_liveImageStatusLabel != nullptr)
        {
            m_liveImageStatusLabel->setText(QStringLiteral("扫描已结束；本轮图像按相机保存参数归档到结果目录。"));
        }
    }
    for (QPushButton* button : {
            m_teachBaseButton, m_teachStartButton, m_teachEndButton,
            m_generateButton, m_runButton })
    {
        if (button != nullptr) button->setEnabled(!running);
    }
    for (QDoubleSpinBox* spin : {
            m_scanSpeedSpin,
            m_lowPlatformSpin, m_risingSpin, m_highPlatformSpin, m_fallingSpin,
            m_leftAngleSpin, m_rightAngleSpin, m_transitionSpin, m_pointStepSpin })
    {
        if (spin != nullptr) spin->setEnabled(!running);
    }
    if (m_robotCombo != nullptr) m_robotCombo->setEnabled(!running && m_robotCombo->count() > 0);
    if (m_cameraCombo != nullptr) m_cameraCombo->setEnabled(!running && m_cameraCombo->count() > 0);
}
