#include "PreciseMeasureEditDialog.h"

#include "FANUCRobotDriver.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QComboBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSplitter>
#include <QStringList>
#include <QVariant>
#include <QVBoxLayout>

namespace
{
QString ProjectRootPathForPreciseEdit()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        if (QFileInfo::exists(dir.filePath("QtWidgetsApplication4.sln")))
        {
            return QDir::toNativeSeparators(dir.absolutePath());
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return QDir::toNativeSeparators(QDir::currentPath());
}

QString AxisKey(const QString& group, const QString& axis)
{
    return group + "." + axis;
}

QLineEdit* CreateValueEdit(int minWidth = 76, int maxWidth = 110)
{
    QLineEdit* edit = new QLineEdit();
    edit->setMinimumWidth(minWidth);
    edit->setMaximumWidth(maxWidth);
    edit->setAlignment(Qt::AlignRight);
    return edit;
}

bool IsDedicatedPulseKey(const QString& key)
{
    return key.startsWith("StartPulse.", Qt::CaseInsensitive)
        || key.startsWith("StartPos.", Qt::CaseInsensitive)
        || key.startsWith("EndPulse.", Qt::CaseInsensitive)
        || key.startsWith("EndPos.", Qt::CaseInsensitive)
        || key.startsWith("StartSafePulse0.", Qt::CaseInsensitive)
        || key.startsWith("EndSafePulse0.", Qt::CaseInsensitive)
        || key.compare("StartSafePulseNum", Qt::CaseInsensitive) == 0
        || key.compare("EndSafePulseNum", Qt::CaseInsensitive) == 0;
}

constexpr auto CAMERA_READ_FPS_KEY = "CameraReadFps";
constexpr auto CAMERA_TIME_OFFSET_MS_KEY = "CameraTimeOffsetMs";
constexpr double DEFAULT_CAMERA_READ_FPS = 100.0;
constexpr double DEFAULT_CAMERA_TIME_OFFSET_MS = -300.0;

QString PreciseParamDisplayName(const QString& key)
{
    static const QMap<QString, QString> names = {
        { "ALLPostionNum", "位置总数" },
        { "UsePostionNo", "当前使用位置号" },
        { "StartSafePulseNum", "下枪安全位置数量" },
        { "EndSafePulseNum", "收枪安全位置数量" },
        { "ScanStartCarLoction", "大车扫描起始位置" },
        { "ScanEndtCarLoction", "大车扫描结束位置" },
        { "ScanLength", "扫描长度" },
        { "ScanSpeed", "扫描速度" },
        { "RunSpeed", "运行速度" },
        { "Acc", "加速度" },
        { "Dec", "减速度" },
        { "dAcc", "加速度系数" },
        { "dDec", "减速度系数" },
        { "TableY", "料台Y坐标" },
        { "TableZ", "料台Z坐标" },
        { "Range_XMax", "X最大范围" },
        { "Range_XMin", "X最小范围" },
        { "Range_YMax", "Y最大范围" },
        { "Range_YMin", "Y最小范围" },
        { "Range_ZMax", "Z最大范围" },
        { "Range_ZMin", "Z最小范围" },
        { "Range_XMAX", "X最大范围" },
        { "Range_XMIN", "X最小范围" },
        { "Range_YMAX", "Y最大范围" },
        { "Range_YMIN", "Y最小范围" },
        { "Range_ZMAX", "Z最大范围" },
        { "Range_ZMIN", "Z最小范围" },
        { "XMax", "X最大值" },
        { "XMin", "X最小值" },
        { "YMax", "Y最大值" },
        { "YMin", "Y最小值" },
        { "ZMax", "Z最大值" },
        { "ZMin", "Z最小值" },
        { "YMaxCar", "大车Y最大值" },
        { "YMinCar", "大车Y最小值" },
        { "YMaxRobot", "机器人Y最大值" },
        { "YMinRobot", "机器人Y最小值" },
        { "ImgStart_x", "图像起始X" },
        { "ImgEnd_x", "图像结束X" },
        { "Scanlength", "扫描长度" },
        { "ScanDir", "扫描方向" },
        { CAMERA_READ_FPS_KEY, "相机读取帧率" },
        { CAMERA_TIME_OFFSET_MS_KEY, "相机时间补偿(ms)" },
        { "ImgStartX", "图像起始X" },
        { "ImgEndX", "图像结束X" },
        { "TableScanDir", "料台扫描方向" },
        { "ExAxisEnable", "外部轴使能" },
    };

    const auto it = names.find(key);
    if (it != names.end())
    {
        return it.value();
    }
    return key;
}

QString PreciseCommentText(const QString& line)
{
    QString text = line.trimmed();
    while (text.startsWith('#'))
    {
        text.remove(0, 1);
        text = text.trimmed();
    }
    return text;
}

void AddOtherParamEditor(QGridLayout* layout, QMap<QString, QLineEdit*>& editors, int& row, int& colInGroup, const QString& key, const QString& value)
{
    if (layout == nullptr || editors.contains(key))
    {
        return;
    }

    QLabel* label = new QLabel(PreciseParamDisplayName(key));
    QLineEdit* edit = new QLineEdit(value);
    label->setToolTip(key);
    edit->setToolTip(key);
    edit->setProperty("paramKey", key);
    edit->setMinimumWidth(90);
    edit->setMaximumWidth(130);
    editors.insert(key, edit);

    const int uiCol = colInGroup * 2;
    layout->addWidget(label, row, uiCol);
    layout->addWidget(edit, row, uiCol + 1);
    ++colInGroup;
    if (colInGroup >= 2)
    {
        ++row;
        colInGroup = 0;
    }
}

}

PreciseMeasureEditDialog::PreciseMeasureEditDialog(ContralUnit* pContralUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
{
    BuildUi();
    LoadRobotList();
    LoadCurrentParam();
}

void PreciseMeasureEditDialog::BuildUi()
{
    setWindowTitle("精测量数据修改");
    setObjectName("PreciseMeasureEditDialog");
    setWindowFlags(windowFlags() | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
    setMinimumSize(720, 520);
    setStyleSheet(QString(
        "QDialog#PreciseMeasureEditDialog { background: #111820; color: #ECF3F4; }"
        "QWidget#PreciseMeasurePage { background: #111820; color: #ECF3F4; }"
        "QWidget#PrecisePulsePanel, QWidget#PreciseOtherPanel { background: #111820; color: #ECF3F4; }"
        "QWidget#OtherParamWidget { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #9ED8DB; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 7px; padding: 4px 6px; }"
        "QScrollArea#PrecisePageScroll { background: #111820; border: none; }"
        "QScrollArea#PrecisePageScroll > QWidget > QWidget { background: #111820; }"
        "QScrollBar:vertical { background: #111820; width: 12px; margin: 0; }"
        "QScrollBar::handle:vertical { background: #385366; border-radius: 6px; min-height: 24px; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }")
        + UnifiedComboBoxStyleSheet());

    QVBoxLayout* outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);
    QScrollArea* pageScrollArea = new QScrollArea();
    pageScrollArea->setObjectName("PrecisePageScroll");
    pageScrollArea->setWidgetResizable(true);
    pageScrollArea->setFrameShape(QFrame::NoFrame);
    outerLayout->addWidget(pageScrollArea);

    QWidget* pageWidget = new QWidget();
    pageWidget->setObjectName("PreciseMeasurePage");
    QVBoxLayout* rootLayout = new QVBoxLayout(pageWidget);
    rootLayout->setContentsMargins(22, 18, 22, 18);
    pageScrollArea->setWidget(pageWidget);

    QLabel* titleLabel = new QLabel("精测量数据修改");
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QHBoxLayout* robotLayout = new QHBoxLayout();
    robotLayout->addWidget(new QLabel("读取机器人："));
    m_pRobotCombo = new QComboBox();
    m_pRobotCombo->setMinimumWidth(220);
    m_pRobotCombo->setMaximumWidth(320);
    m_pRobotCombo->setFixedHeight(28);
    robotLayout->addWidget(m_pRobotCombo);
    robotLayout->addStretch(1);
    rootLayout->addLayout(robotLayout);

    m_pContentSplitter = new QSplitter(Qt::Horizontal);
    m_pContentSplitter->setChildrenCollapsible(false);
    rootLayout->addWidget(m_pContentSplitter, 1);

    m_pPulsePanel = new QWidget();
    m_pPulsePanel->setObjectName("PrecisePulsePanel");
    m_pPulseGroupsLayout = new QGridLayout(m_pPulsePanel);
    m_pPulseGroupsLayout->setContentsMargins(0, 0, 0, 0);
    m_pulseGroupWidgets
        << CreatePulseGroup("下枪安全位置（脉冲）", "StartSafePulse0", "示教下枪安全位置", &PreciseMeasureEditDialog::TeachStartSafePulse, &PreciseMeasureEditDialog::SaveManualStartSafePulse)
        << CreateCoorsGroup("扫描起点（直角）", "StartPos", "示教起点位置", &PreciseMeasureEditDialog::TeachStartPos, &PreciseMeasureEditDialog::SaveManualStartPos)
        << CreateCoorsGroup("扫描终点（直角）", "EndPos", "示教终点位置", &PreciseMeasureEditDialog::TeachEndPos, &PreciseMeasureEditDialog::SaveManualEndPos)
        << CreatePulseGroup("收枪安全位置（脉冲）", "EndSafePulse0", "示教收枪安全位置", &PreciseMeasureEditDialog::TeachEndSafePulse, &PreciseMeasureEditDialog::SaveManualEndSafePulse);

    m_pOtherPanel = new QWidget();
    m_pOtherPanel->setObjectName("PreciseOtherPanel");
    QVBoxLayout* otherPanelLayout = new QVBoxLayout(m_pOtherPanel);
    otherPanelLayout->setContentsMargins(0, 0, 0, 0);
    QGroupBox* otherGroup = new QGroupBox("其它参数");
    QVBoxLayout* otherLayout = new QVBoxLayout(otherGroup);
    QLabel* otherHint = new QLabel("除四类点位外，本分组其它参数会显示为编辑框；修改后点击“保存参数”统一写回参数文件。");
    otherLayout->addWidget(otherHint);

    QWidget* otherWidget = new QWidget();
    otherWidget->setObjectName("OtherParamWidget");
    otherWidget->setAutoFillBackground(true);
    m_pOtherParamLayout = new QGridLayout(otherWidget);
    m_pOtherParamLayout->setContentsMargins(8, 8, 8, 8);
    m_pOtherParamLayout->setColumnStretch(1, 1);
    otherLayout->addWidget(otherWidget);

    otherPanelLayout->addWidget(otherGroup);

    m_pContentSplitter->addWidget(m_pPulsePanel);
    m_pContentSplitter->addWidget(m_pOtherPanel);
    m_pContentSplitter->setStretchFactor(0, 3);
    m_pContentSplitter->setStretchFactor(1, 2);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch(1);
    QPushButton* reloadBtn = new QPushButton("重新读取");
    QPushButton* saveBtn = new QPushButton("保存参数");
    reloadBtn->setMinimumWidth(120);
    saveBtn->setMinimumWidth(120);
    actionLayout->addWidget(reloadBtn);
    actionLayout->addWidget(saveBtn);
    rootLayout->addLayout(actionLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setMaximumHeight(100);
    m_pLogText->setPlainText("日志：等待读取参数...");
    rootLayout->addWidget(m_pLogText);

    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &PreciseMeasureEditDialog::OnRobotChanged);
    connect(reloadBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::ReloadCurrentParam);
    connect(saveBtn, &QPushButton::clicked, this, &PreciseMeasureEditDialog::SaveAllParamEdits);
    UpdateAdaptiveLayout();
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(980, 680), 0.84, 0.78);
}

void PreciseMeasureEditDialog::resizeEvent(QResizeEvent* event)
{
    QDialog::resizeEvent(event);
    UpdateAdaptiveLayout();
}

void PreciseMeasureEditDialog::LoadRobotList()
{
    m_bLoading = true;
    m_pRobotCombo->clear();

    if (m_pContralUnit == nullptr)
    {
        AppendLog("控制单元为空，无法加载机器人列表。");
        m_bLoading = false;
        return;
    }

    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        const QString paramPath = RobotDataHelper::PreciseMeasureParamPath(info.robotName);
        if (QFileInfo::exists(paramPath))
        {
            m_pRobotCombo->addItem(info.displayName, info.unitIndex);
        }
        else
        {
            AppendLog(QString("跳过 %1：未找到 %2").arg(info.displayName, paramPath));
        }
    }

    if (m_pRobotCombo->count() <= 0)
    {
        AppendLog("控制单元中没有可用于精测量示教的机器人驱动。");
    }
    m_bLoading = false;
}

void PreciseMeasureEditDialog::OnRobotChanged(int index)
{
    (void)index;
    if (!m_bLoading)
    {
        LoadCurrentParam();
    }
}

void PreciseMeasureEditDialog::TeachStartPos()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ROBOT_COORS coors = driver->GetCurrentPos();
    SetCoorsEditors("StartPos", coors);
    AppendLog("已读取当前机器人直角坐标到扫描起点，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::TeachStartSafePulse()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ANGLE_PULSE pulse = driver->GetCurrentPulse();
    SetPulseEditors("StartSafePulse0", pulse);
    AppendLog("已读取当前机器人脉冲到下枪安全位置，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::TeachEndPos()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ROBOT_COORS coors = driver->GetCurrentPos();
    SetCoorsEditors("EndPos", coors);
    AppendLog("已读取当前机器人直角坐标到扫描终点，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::TeachEndSafePulse()
{
    RobotDriverAdaptor* driver = GetSelectedRobotDriver();
    if (driver == nullptr)
    {
        return;
    }

    const T_ANGLE_PULSE pulse = driver->GetCurrentPulse();
    SetPulseEditors("EndSafePulse0", pulse);
    AppendLog("已读取当前机器人脉冲到收枪安全位置，点击“保存参数”后写入文件。");
}

void PreciseMeasureEditDialog::ReloadCurrentParam()
{
    LoadCurrentParam();
}

void PreciseMeasureEditDialog::closeEvent(QCloseEvent* event)
{
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "精测量数据修改", [this]() { return SaveAllParamEdits(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

bool PreciseMeasureEditDialog::SaveAllParamEdits()
{
    if (m_bLoading)
    {
        return false;
    }

    T_ANGLE_PULSE startSafePulse;
    T_ROBOT_COORS startPos;
    T_ROBOT_COORS endPos;
    T_ANGLE_PULSE endSafePulse;
    QString error;
    if (!GetPulseFromEditors("StartSafePulse0", startSafePulse, error)
        || !GetCoorsFromEditors("StartPos", startPos, error)
        || !GetCoorsFromEditors("EndPos", endPos, error)
        || !GetPulseFromEditors("EndSafePulse0", endSafePulse, error))
    {
        QMessageBox::warning(this, "保存参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    if (!WritePulse("StartSafePulse0", startSafePulse, error)
        || !WriteCoors("StartPos", startPos, error)
        || !WriteCoors("EndPos", endPos, error)
        || !WritePulse("EndSafePulse0", endSafePulse, error))
    {
        QMessageBox::warning(this, "保存参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    for (auto it = m_otherParamEditors.cbegin(); it != m_otherParamEditors.cend(); ++it)
    {
        QLineEdit* edit = it.value();
        if (edit == nullptr)
        {
            continue;
        }

        if (!WriteParamValue(it.key(), edit->text().trimmed(), error))
        {
            QMessageBox::warning(this, "保存参数", error);
            AppendLog("其它参数保存失败：" + error);
            return false;
        }
    }

    MarkCleanSnapshot();
    AppendLog("精测量参数已统一保存。");
    QMessageBox::information(this, "保存参数", "精测量参数保存完成。");
    return true;
}

void PreciseMeasureEditDialog::SaveManualStartSafePulse()
{
    if (m_bLoading)
    {
        return;
    }

    T_ANGLE_PULSE pulse;
    QString error;
    if (!GetPulseFromEditors("StartSafePulse0", pulse, error) || !WritePulse("StartSafePulse0", pulse, error))
    {
        AppendLog("下枪安全位置手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog("下枪安全位置手动修改已保存。");
}

void PreciseMeasureEditDialog::SaveManualStartPos()
{
    if (m_bLoading)
    {
        return;
    }

    T_ROBOT_COORS coors;
    QString error;
    if (!GetCoorsFromEditors("StartPos", coors, error) || !WriteCoors("StartPos", coors, error))
    {
        AppendLog("起点手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog("起点手动修改已保存。");
}

void PreciseMeasureEditDialog::SaveManualEndPos()
{
    if (m_bLoading)
    {
        return;
    }

    T_ROBOT_COORS coors;
    QString error;
    if (!GetCoorsFromEditors("EndPos", coors, error) || !WriteCoors("EndPos", coors, error))
    {
        AppendLog("终点手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog("终点手动修改已保存。");
}

void PreciseMeasureEditDialog::SaveManualEndSafePulse()
{
    if (m_bLoading)
    {
        return;
    }

    T_ANGLE_PULSE pulse;
    QString error;
    if (!GetPulseFromEditors("EndSafePulse0", pulse, error) || !WritePulse("EndSafePulse0", pulse, error))
    {
        AppendLog("收枪安全位置手动保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog("收枪安全位置手动修改已保存。");
}

RobotDriverAdaptor* PreciseMeasureEditDialog::GetSelectedRobotDriver()
{
    if (m_pContralUnit == nullptr || m_pRobotCombo == nullptr || m_pRobotCombo->currentIndex() < 0)
    {
        QMessageBox::warning(this, "精测量数据修改", "未选择可用机器人。");
        return nullptr;
    }

    const int unitIndex = m_pRobotCombo->currentData().toInt();
    RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, unitIndex);
    if (driver == nullptr)
    {
        QMessageBox::warning(this, "精测量数据修改", "当前选择的机器人没有可用驱动。");
        return nullptr;
    }
    return driver;
}

QString PreciseMeasureEditDialog::CurrentRobotName() const
{
    if (m_pContralUnit == nullptr || m_pRobotCombo == nullptr || m_pRobotCombo->currentIndex() < 0)
    {
        return QString();
    }

    const int unitIndex = m_pRobotCombo->currentData().toInt();
    if (unitIndex < 0 || unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return QString();
    }

    const T_CONTRAL_UNIT& unitInfo = m_pContralUnit->m_vtContralUnitInfo[unitIndex];
    RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
    if (driver == nullptr)
    {
        return QString();
    }

    return QString::fromStdString(driver->m_sRobotName.empty() ? unitInfo.sUnitName : driver->m_sRobotName);
}

QString PreciseMeasureEditDialog::CurrentParamFilePath() const
{
    if (CurrentRobotName().isEmpty())
    {
        return QString();
    }
    return RobotDataHelper::PreciseMeasureParamPath(CurrentRobotName());
}

QString PreciseMeasureEditDialog::CurrentSectionName(QString* error) const
{
    if (CurrentRobotName().isEmpty())
    {
        if (error != nullptr)
        {
            *error = "未选择机器人。";
        }
        return QString();
    }
    return RobotDataHelper::PreciseMeasureSectionName(CurrentRobotName(), error);
}

bool PreciseMeasureEditDialog::LoadCurrentParam()
{
    QString error;
    T_ANGLE_PULSE startSafePulse;
    T_ANGLE_PULSE endSafePulse;
    T_ROBOT_COORS startPos;
    T_ROBOT_COORS endPos;
    if (!ReadPulse("StartSafePulse0", startSafePulse, error)
        || !ReadPulse("EndSafePulse0", endSafePulse, error))
    {
        AppendLog("读取失败：" + error);
        return false;
    }

    QString startPosError;
    QString endPosError;
    const bool hasStartPos = ReadCoors("StartPos", startPos, startPosError);
    const bool hasEndPos = ReadCoors("EndPos", endPos, endPosError);
    if (!hasStartPos)
    {
        startPos = T_ROBOT_COORS();
    }
    if (!hasEndPos)
    {
        endPos = T_ROBOT_COORS();
    }

    SetEditorsBlocked(true);
    SetPulseEditors("StartSafePulse0", startSafePulse);
    SetCoorsEditors("StartPos", startPos);
    SetCoorsEditors("EndPos", endPos);
    SetPulseEditors("EndSafePulse0", endSafePulse);
    SetEditorsBlocked(false);
    LoadOtherParams();
    if (!hasStartPos)
    {
        AppendLog("未读取到扫描起点直角参数，请重新示教并保存。" + startPosError);
    }
    if (!hasEndPos)
    {
        AppendLog("未读取到扫描终点直角参数，请重新示教并保存。" + endPosError);
    }
    AppendLog(QString("已读取 %1 的精测量点位和其它参数。").arg(CurrentRobotName()));
    MarkCleanSnapshot();
    return true;
}

bool PreciseMeasureEditDialog::ReadPulse(const QString& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::ReadPrecisePulse(CurrentRobotName(), prefix, pulse, &error);
}

bool PreciseMeasureEditDialog::WritePulse(const QString& prefix, const T_ANGLE_PULSE& pulse, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WritePrecisePulse(CurrentRobotName(), prefix, pulse, &error);
}

bool PreciseMeasureEditDialog::ReadCoors(const QString& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::ReadPreciseCoors(CurrentRobotName(), prefix, coors, &error);
}

bool PreciseMeasureEditDialog::WriteCoors(const QString& prefix, const T_ROBOT_COORS& coors, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WritePreciseCoors(CurrentRobotName(), prefix, coors, &error);
}

bool PreciseMeasureEditDialog::LoadOtherParams()
{
    if (m_pOtherParamLayout == nullptr)
    {
        return false;
    }

    QString error;
    const QString path = CurrentParamFilePath();
    const QString section = CurrentSectionName(&error);
    if (path.isEmpty() || section.isEmpty())
    {
        AppendLog("读取其它参数失败：" + error);
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        AppendLog("读取其它参数失败：打开参数文件失败：" + path);
        return false;
    }

    m_bLoading = true;
    ClearOtherParamEditors();

    QString content = QString::fromLocal8Bit(file.readAll());
    content.replace("\r\n", "\n");
    content.replace('\r', '\n');

    bool inSection = false;
    bool hasOtherParam = false;
    int row = 0;
    int colInGroup = 0;
    QStringList pendingComments;
    const QStringList lines = content.split('\n');
    for (const QString& line : lines)
    {
        const QString trimmed = line.trimmed();
        if (trimmed.startsWith('[') && trimmed.endsWith(']'))
        {
            const QString currentSection = trimmed.mid(1, trimmed.size() - 2).trimmed();
            if (inSection && currentSection.compare(section, Qt::CaseInsensitive) != 0)
            {
                break;
            }
            inSection = currentSection.compare(section, Qt::CaseInsensitive) == 0;
            pendingComments.clear();
            colInGroup = 0;
            continue;
        }

        if (!inSection || trimmed.isEmpty())
        {
            continue;
        }

        if (trimmed.startsWith('#'))
        {
            const QString comment = PreciseCommentText(trimmed);
            if (!comment.isEmpty())
            {
                pendingComments << comment;
            }
            continue;
        }

        const int pos = line.indexOf('=');
        if (pos > 0)
        {
            const QString key = line.left(pos).trimmed();
            const QString value = line.mid(pos + 1).trimmed();
            if (IsDedicatedPulseKey(key))
            {
                pendingComments.clear();
                colInGroup = 0;
                continue;
            }

            if (!pendingComments.isEmpty())
            {
                if (colInGroup != 0)
                {
                    ++row;
                    colInGroup = 0;
                }
                QLabel* commentLabel = new QLabel(pendingComments.join("  "));
                commentLabel->setWordWrap(true);
                commentLabel->setStyleSheet("QLabel { color: #9EEBF0; background: #132532; border: 1px solid #2D5262; border-radius: 8px; padding: 6px 10px; font-weight: bold; }");
                m_pOtherParamLayout->addWidget(commentLabel, row, 0, 1, 4);
                ++row;
                pendingComments.clear();
            }

            AddOtherParamEditor(m_pOtherParamLayout, m_otherParamEditors, row, colInGroup, key, value);
            hasOtherParam = true;
        }
    }

    if (!m_otherParamEditors.contains(CAMERA_READ_FPS_KEY))
    {
        AddOtherParamEditor(
            m_pOtherParamLayout,
            m_otherParamEditors,
            row,
            colInGroup,
            CAMERA_READ_FPS_KEY,
            QString::number(DEFAULT_CAMERA_READ_FPS, 'f', 0));
        hasOtherParam = true;
    }

    if (!m_otherParamEditors.contains(CAMERA_TIME_OFFSET_MS_KEY))
    {
        AddOtherParamEditor(
            m_pOtherParamLayout,
            m_otherParamEditors,
            row,
            colInGroup,
            CAMERA_TIME_OFFSET_MS_KEY,
            QString::number(DEFAULT_CAMERA_TIME_OFFSET_MS, 'f', 0));
        hasOtherParam = true;
    }

    if (!hasOtherParam)
    {
        AppendLog(QString("当前分组没有其它参数：%1").arg(section));
    }

    m_bLoading = false;
    return true;
}

bool PreciseMeasureEditDialog::WriteParamValue(const QString& key, const QString& value, QString& error) const
{
    if (CurrentRobotName().isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WritePreciseParamValue(CurrentRobotName(), key, value, &error);
}

QGroupBox* PreciseMeasureEditDialog::CreatePulseGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)())
{
    Q_UNUSED(saveSlot);
    const QStringList axes = { "nS", "nL", "nU", "nR", "nB", "nT", "lBX", "lBY", "lBZ" };
    QGroupBox* groupBox = new QGroupBox(title);
    groupBox->setMinimumHeight(248);
    QGridLayout* layout = new QGridLayout(groupBox);
    layout->setContentsMargins(36, 28, 36, 16);
    layout->setVerticalSpacing(10);
    layout->setHorizontalSpacing(8);
    for (int i = 0; i < axes.size(); ++i)
    {
        QLabel* label = new QLabel(axes[i]);
        QLineEdit* edit = CreateValueEdit();
        m_editors.insert(AxisKey(groupName, axes[i]), edit);
        layout->addWidget(label, i / 3, (i % 3) * 2);
        layout->addWidget(edit, i / 3, (i % 3) * 2 + 1);
    }
    QPushButton* teachBtn = new QPushButton(teachText);
    teachBtn->setMinimumHeight(34);
    teachBtn->setMaximumHeight(40);
    layout->addWidget(teachBtn, 3, 0, 1, 6);
    connect(teachBtn, &QPushButton::clicked, this, teachSlot);
    return groupBox;
}

QGroupBox* PreciseMeasureEditDialog::CreateCoorsGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)())
{
    Q_UNUSED(saveSlot);
    const QStringList axes = { "X", "Y", "Z", "RX", "RY", "RZ", "BX", "BY", "BZ" };
    QGroupBox* groupBox = new QGroupBox(title);
    groupBox->setMinimumHeight(248);
    QGridLayout* layout = new QGridLayout(groupBox);
    layout->setContentsMargins(36, 28, 36, 16);
    layout->setVerticalSpacing(10);
    layout->setHorizontalSpacing(8);
    for (int i = 0; i < axes.size(); ++i)
    {
        QLabel* label = new QLabel(axes[i]);
        QLineEdit* edit = CreateValueEdit(88, 118);
        m_editors.insert(AxisKey(groupName, axes[i]), edit);
        layout->addWidget(label, i / 3, (i % 3) * 2);
        layout->addWidget(edit, i / 3, (i % 3) * 2 + 1);
    }
    QPushButton* teachBtn = new QPushButton(teachText);
    teachBtn->setMinimumHeight(34);
    teachBtn->setMaximumHeight(40);
    layout->addWidget(teachBtn, 3, 0, 1, 6);
    connect(teachBtn, &QPushButton::clicked, this, teachSlot);
    return groupBox;
}

void PreciseMeasureEditDialog::SetPulseEditors(const QString& groupName, const T_ANGLE_PULSE& pulse)
{
    const QMap<QString, long> values = {
        { "nS", pulse.nSPulse },
        { "nL", pulse.nLPulse },
        { "nU", pulse.nUPulse },
        { "nR", pulse.nRPulse },
        { "nB", pulse.nBPulse },
        { "nT", pulse.nTPulse },
        { "lBX", pulse.lBXPulse },
        { "lBY", pulse.lBYPulse },
        { "lBZ", pulse.lBZPulse },
    };

    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        QLineEdit* edit = m_editors.value(AxisKey(groupName, it.key()), nullptr);
        if (edit != nullptr)
        {
            edit->setText(QString::number(it.value()));
        }
    }
}

void PreciseMeasureEditDialog::SetCoorsEditors(const QString& groupName, const T_ROBOT_COORS& coors)
{
    const QMap<QString, double> values = {
        { "X", coors.dX },
        { "Y", coors.dY },
        { "Z", coors.dZ },
        { "RX", coors.dRX },
        { "RY", coors.dRY },
        { "RZ", coors.dRZ },
        { "BX", coors.dBX },
        { "BY", coors.dBY },
        { "BZ", coors.dBZ },
    };

    for (auto it = values.cbegin(); it != values.cend(); ++it)
    {
        QLineEdit* edit = m_editors.value(AxisKey(groupName, it.key()), nullptr);
        if (edit != nullptr)
        {
            edit->setText(QString::number(it.value(), 'f', 6));
        }
    }
}

bool PreciseMeasureEditDialog::GetPulseFromEditors(const QString& groupName, T_ANGLE_PULSE& pulse, QString& error) const
{
    auto readValue = [this, groupName, &error](const QString& axis, long& value) -> bool
        {
            QLineEdit* edit = m_editors.value(AxisKey(groupName, axis), nullptr);
            if (edit == nullptr)
            {
                error = "找不到输入框：" + groupName + "." + axis;
                return false;
            }
            bool ok = false;
            const long parsed = edit->text().trimmed().toLong(&ok);
            if (!ok)
            {
                error = "输入不是整数：" + groupName + "." + axis;
                return false;
            }
            value = parsed;
            return true;
        };

    return readValue("nS", pulse.nSPulse)
        && readValue("nL", pulse.nLPulse)
        && readValue("nU", pulse.nUPulse)
        && readValue("nR", pulse.nRPulse)
        && readValue("nB", pulse.nBPulse)
        && readValue("nT", pulse.nTPulse)
        && readValue("lBX", pulse.lBXPulse)
        && readValue("lBY", pulse.lBYPulse)
        && readValue("lBZ", pulse.lBZPulse);
}

bool PreciseMeasureEditDialog::GetCoorsFromEditors(const QString& groupName, T_ROBOT_COORS& coors, QString& error) const
{
    auto readValue = [this, groupName, &error](const QString& axis, double& value) -> bool
        {
            QLineEdit* edit = m_editors.value(AxisKey(groupName, axis), nullptr);
            if (edit == nullptr)
            {
                error = "找不到输入框：" + groupName + "." + axis;
                return false;
            }
            bool ok = false;
            const double parsed = edit->text().trimmed().toDouble(&ok);
            if (!ok)
            {
                error = "输入不是数字：" + groupName + "." + axis;
                return false;
            }
            value = parsed;
            return true;
        };

    return readValue("X", coors.dX)
        && readValue("Y", coors.dY)
        && readValue("Z", coors.dZ)
        && readValue("RX", coors.dRX)
        && readValue("RY", coors.dRY)
        && readValue("RZ", coors.dRZ)
        && readValue("BX", coors.dBX)
        && readValue("BY", coors.dBY)
        && readValue("BZ", coors.dBZ);
}

void PreciseMeasureEditDialog::SetEditorsBlocked(bool blocked)
{
    m_bLoading = blocked;
    for (QLineEdit* edit : m_editors)
    {
        if (edit != nullptr)
        {
            edit->blockSignals(blocked);
        }
    }
}

void PreciseMeasureEditDialog::ClearOtherParamEditors()
{
    m_otherParamEditors.clear();
    if (m_pOtherParamLayout == nullptr)
    {
        return;
    }

    while (QLayoutItem* item = m_pOtherParamLayout->takeAt(0))
    {
        if (QWidget* widget = item->widget())
        {
            widget->deleteLater();
        }
        delete item;
    }
}

void PreciseMeasureEditDialog::UpdateAdaptiveLayout()
{
    if (m_pContentSplitter == nullptr)
    {
        return;
    }

    const bool wide = width() >= 1050;
    RebuildPulseGroupLayout(wide);

    const Qt::Orientation targetOrientation = wide ? Qt::Horizontal : Qt::Vertical;
    if (m_pContentSplitter->orientation() != targetOrientation)
    {
        m_pContentSplitter->setOrientation(targetOrientation);
    }

    if (wide)
    {
        m_pContentSplitter->setSizes(QList<int>() << width() * 3 / 5 << width() * 2 / 5);
    }
    else
    {
        m_pContentSplitter->setSizes(QList<int>() << height() * 2 / 3 << height() / 3);
    }
}

void PreciseMeasureEditDialog::RebuildPulseGroupLayout(bool wide)
{
    if (m_pPulseGroupsLayout == nullptr || m_bWideAdaptiveLayout == wide)
    {
        return;
    }

    while (QLayoutItem* item = m_pPulseGroupsLayout->takeAt(0))
    {
        delete item;
    }

    for (int i = 0; i < m_pulseGroupWidgets.size(); ++i)
    {
        QWidget* widget = m_pulseGroupWidgets[i];
        if (wide)
        {
            m_pPulseGroupsLayout->addWidget(widget, i / 2, i % 2);
        }
        else
        {
            m_pPulseGroupsLayout->addWidget(widget, i, 0);
        }
    }

    m_pPulseGroupsLayout->setColumnStretch(0, 1);
    m_pPulseGroupsLayout->setColumnStretch(1, wide ? 1 : 0);
    m_bWideAdaptiveLayout = wide;
}

void PreciseMeasureEditDialog::SaveOtherParamEdit()
{
    if (m_bLoading)
    {
        return;
    }

    QLineEdit* edit = qobject_cast<QLineEdit*>(sender());
    if (edit == nullptr)
    {
        return;
    }

    QString error;
    const QString key = edit->property("paramKey").toString();
    const QString value = edit->text().trimmed();
    if (!WriteParamValue(key, value, error))
    {
        AppendLog("其它参数保存失败：" + error);
        return;
    }
    MarkCleanSnapshot();
    AppendLog(QString("其它参数已保存：%1=%2").arg(PreciseParamDisplayName(key), value));
}

bool PreciseMeasureEditDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString PreciseMeasureEditDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << CurrentRobotName();
    for (auto it = m_editors.cbegin(); it != m_editors.cend(); ++it)
    {
        fields << it.key() << (it.value() != nullptr ? it.value()->text().trimmed() : QString());
    }
    for (auto it = m_otherParamEditors.cbegin(); it != m_otherParamEditors.cend(); ++it)
    {
        fields << it.key() << (it.value() != nullptr ? it.value()->text().trimmed() : QString());
    }
    return fields.join('\n');
}

void PreciseMeasureEditDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void PreciseMeasureEditDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }
}
