#include "WeldSeamCompDialog.h"

#include "OPini.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QButtonGroup>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleValidator>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStringList>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
constexpr int EDITOR_FIELD_MAX_WIDTH = 240;

QVector<WeldSeamCompDialog::CompType> BuildDefaultPoseTypes()
{
    return {
        { "姿态0 / 低平台", "low_platform" },
        { "姿态1 / 上升边", "rising_edge" },
        { "姿态2 / 高平台", "high_platform" },
        { "姿态3 / 下降边", "falling_edge" }
    };
}

QVector<WeldSeamCompDialog::SeamCompRow> BuildDefaultSeamRows()
{
    return {
        { "WeldSeamComp0", "波纹板", "CorrugatedPlate", 0.0, 0.0, 0.0 },
        { "WeldSeamComp1", "焊道类型1", "SeamType1", 0.0, 0.0, 0.0 },
        { "WeldSeamComp2", "焊道类型2", "SeamType2", 0.0, 0.0, 0.0 },
        { "WeldSeamComp3", "焊道类型3", "SeamType3", 0.0, 0.0, 0.0 }
    };
}

QString WeldPoseCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldPoseCompParam.ini").arg(robotName));
}

QString WeldSeamCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldSeamCompParam.ini").arg(robotName));
}

QString NativePathText(const QString& title, const QString& path)
{
    return QString("%1：%2").arg(title, QDir::toNativeSeparators(path));
}

QString FormatNumber(double value)
{
    return QString::number(value, 'f', 3);
}

bool ReadIniDouble(COPini& ini, const std::string& key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

int FindCompTypeIndex(const QVector<WeldSeamCompDialog::CompType>& compTypes, const QString& segmentKind)
{
    for (int index = 0; index < compTypes.size(); ++index)
    {
        if (compTypes[index].segmentKind.compare(segmentKind, Qt::CaseInsensitive) == 0)
        {
            return index;
        }
    }
    return -1;
}

std::string LocalString(const QString& text)
{
    const QByteArray bytes = text.toLocal8Bit();
    return std::string(bytes.constData());
}
}

WeldSeamCompDialog::WeldSeamCompDialog(ContralUnit* pContralUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_poseTypes(BuildDefaultPoseTypes())
{
    BuildUi();
    LoadRobotList();
    LoadCurrentParam();
}

void WeldSeamCompDialog::closeEvent(QCloseEvent* event)
{
    QString error;
    StoreEditorValues(false, error);
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "补偿参数", [this]() { return SaveCurrentParam(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

void WeldSeamCompDialog::BuildUi()
{
    setWindowTitle("补偿参数");
    setObjectName("WeldSeamCompDialog");
    setWindowFlags(windowFlags() | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);
    setMinimumSize(620, 520);
    setStyleSheet(QString(
        "QDialog#WeldSeamCompDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:checked { background: #2F6F7A; border-color: #8EE7EC; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 7px; padding: 4px 6px; }"
        "QLineEdit[readOnly=\"true\"] { color: #A7C9CF; background: #101923; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }")
        + UnifiedComboBoxStyleSheet());

    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(22, 18, 22, 18);
    rootLayout->setSpacing(12);

    QLabel* titleLabel = new QLabel("补偿参数");
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QHBoxLayout* topLayout = new QHBoxLayout();
    topLayout->addWidget(new QLabel("机器人："));
    m_pRobotCombo = new QComboBox();
    m_pRobotCombo->setMinimumWidth(180);
    topLayout->addWidget(m_pRobotCombo);
    topLayout->addSpacing(18);

    m_pPoseModeBtn = new QPushButton("姿态补偿");
    m_pSeamModeBtn = new QPushButton("焊道补偿");
    m_pPoseModeBtn->setCheckable(true);
    m_pSeamModeBtn->setCheckable(true);
    m_pModeGroup = new QButtonGroup(this);
    m_pModeGroup->setExclusive(true);
    m_pModeGroup->addButton(m_pPoseModeBtn, 0);
    m_pModeGroup->addButton(m_pSeamModeBtn, 1);
    m_pPoseModeBtn->setChecked(true);
    topLayout->addWidget(m_pPoseModeBtn);
    topLayout->addWidget(m_pSeamModeBtn);
    topLayout->addStretch(1);
    rootLayout->addLayout(topLayout);

    QGroupBox* editorGroup = new QGroupBox("补偿编辑");
    QGridLayout* editorLayout = new QGridLayout(editorGroup);
    editorLayout->setHorizontalSpacing(12);
    editorLayout->setVerticalSpacing(10);

    editorLayout->addWidget(new QLabel("补偿类型："), 0, 0);
    m_pTypeCombo = new QComboBox();
    m_pTypeCombo->setMinimumWidth(240);
    m_pTypeCombo->setMaximumWidth(EDITOR_FIELD_MAX_WIDTH);
    editorLayout->addWidget(m_pTypeCombo, 0, 1, 1, 3);

    m_pHintLabel = new QLabel();
    m_pHintLabel->setWordWrap(true);
    editorLayout->addWidget(m_pHintLabel, 1, 0, 1, 4);

    m_pPoseDisplayWidget = new QWidget();
    QGridLayout* poseLayout = new QGridLayout(m_pPoseDisplayWidget);
    poseLayout->setContentsMargins(0, 0, 0, 0);
    const char* poseLabels[3] = { "姿态RX：", "姿态RY：", "姿态RZ：" };
    for (int index = 0; index < 3; ++index)
    {
        poseLayout->addWidget(new QLabel(QString::fromUtf8(poseLabels[index])), 0, index * 2);
        m_pPoseValues[index] = new QLineEdit();
        m_pPoseValues[index]->setReadOnly(true);
        m_pPoseValues[index]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_pPoseValues[index]->setMinimumWidth(90);
        poseLayout->addWidget(m_pPoseValues[index], 0, index * 2 + 1);
    }
    editorLayout->addWidget(m_pPoseDisplayWidget, 2, 0, 1, 4);

    QDoubleValidator* validator = new QDoubleValidator(-100000.0, 100000.0, 6, this);
    validator->setNotation(QDoubleValidator::StandardNotation);
    for (int index = 0; index < 3; ++index)
    {
        m_pEditLabels[index] = new QLabel();
        m_pEditValues[index] = new QLineEdit();
        m_pEditValues[index]->setValidator(validator);
        m_pEditValues[index]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_pEditValues[index]->setMinimumWidth(130);
        m_pEditValues[index]->setMaximumWidth(150);
        editorLayout->addWidget(m_pEditLabels[index], 3 + index, 0);
        editorLayout->addWidget(m_pEditValues[index], 3 + index, 1, 1, 3);
    }
    rootLayout->addWidget(editorGroup);

    m_pPathLabel = new QLabel();
    m_pPathLabel->setWordWrap(true);
    rootLayout->addWidget(m_pPathLabel);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->addStretch(1);
    m_pReloadBtn = new QPushButton("重新读取");
    m_pSaveBtn = new QPushButton("保存");
    m_pReloadBtn->setMinimumWidth(120);
    m_pSaveBtn->setMinimumWidth(120);
    actionLayout->addWidget(m_pReloadBtn);
    actionLayout->addWidget(m_pSaveBtn);
    rootLayout->addLayout(actionLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setMaximumHeight(110);
    m_pLogText->setPlainText("日志：等待读取补偿参数...");
    rootLayout->addWidget(m_pLogText, 1);

    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (!m_bLoading)
            {
                LoadCurrentParam();
            }
        });
    connect(m_pModeGroup, qOverload<int>(&QButtonGroup::idClicked), this, [this](int id)
        {
            SetMode(id == 0 ? CompMode::Pose : CompMode::Seam);
        });
    connect(m_pTypeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index)
        {
            if (m_bLoading)
            {
                return;
            }
            QString error;
            StoreEditorValues(false, error);
            m_currentTypeIndex = std::max(0, index);
            RefreshEditor();
        });
    connect(m_pReloadBtn, &QPushButton::clicked, this, [this]() { LoadCurrentParam(); });
    connect(m_pSaveBtn, &QPushButton::clicked, this, [this]() { SaveCurrentParam(); });

    RefreshTypeCombo();
    RefreshEditor();

    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(740, 560), 0.72, 0.72);
}

void WeldSeamCompDialog::LoadRobotList()
{
    if (m_pRobotCombo == nullptr)
    {
        return;
    }

    m_bLoading = true;
    m_pRobotCombo->clear();
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        m_pRobotCombo->addItem(info.displayName, info.robotName);
    }
    m_bLoading = false;
}

QString WeldSeamCompDialog::CurrentRobotName() const
{
    if (m_pRobotCombo == nullptr || m_pRobotCombo->currentIndex() < 0)
    {
        return QString();
    }
    return m_pRobotCombo->currentData().toString();
}

QString WeldSeamCompDialog::CurrentPoseParamPath() const
{
    const QString robotName = CurrentRobotName();
    return robotName.isEmpty() ? QString() : WeldPoseCompParamPath(robotName);
}

QString WeldSeamCompDialog::CurrentSeamParamPath() const
{
    const QString robotName = CurrentRobotName();
    return robotName.isEmpty() ? QString() : WeldSeamCompParamPath(robotName);
}

QString WeldSeamCompDialog::CurrentParamPath() const
{
    return m_mode == CompMode::Pose ? CurrentPoseParamPath() : CurrentSeamParamPath();
}

int WeldSeamCompDialog::CurrentTypeIndex() const
{
    const int rowCount = CurrentRowCount();
    if (rowCount <= 0)
    {
        return -1;
    }
    return std::clamp(m_currentTypeIndex, 0, rowCount - 1);
}

int WeldSeamCompDialog::CurrentRowCount() const
{
    return m_mode == CompMode::Pose ? m_poseRows.size() : m_seamRows.size();
}

void WeldSeamCompDialog::LoadCurrentParam()
{
    const QString posePath = CurrentPoseParamPath();
    const QString seamPath = CurrentSeamParamPath();
    if (posePath.isEmpty() || seamPath.isEmpty())
    {
        AppendLog("未选择机器人，无法读取补偿参数。");
        return;
    }

    m_bLoading = true;
    LoadPoseParam(posePath);
    LoadSeamParam(seamPath);
    RefreshTypeCombo();
    m_bLoading = false;
    RefreshEditor();

    AppendLog(QString("已读取补偿参数：姿态槽=%1，焊道槽=%2。")
        .arg(m_poseRows.size())
        .arg(m_seamRows.size()));
    MarkCleanSnapshot();
}

void WeldSeamCompDialog::LoadPoseParam(const QString& path)
{
    m_poseRows.clear();
    m_poseRows.reserve(m_poseTypes.size());
    for (int index = 0; index < m_poseTypes.size(); ++index)
    {
        PoseCompRow row;
        row.sectionName = QString("WeldPoseComp%1").arg(index);
        row.name = m_poseTypes[index].displayName;
        row.segmentKind = m_poseTypes[index].segmentKind;
        m_poseRows.push_back(row);
    }

    if (path.isEmpty() || !QFileInfo::exists(path))
    {
        AppendLog("姿态补偿参数不存在，使用默认槽位：" + QDir::toNativeSeparators(path));
        return;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldPoseComp");
    ReadIniDouble(ini, "PoseMatchMaxErrorDeg", m_poseMatchMaxErrorDeg);

    int count = m_poseRows.size();
    ini.ReadString(false, "PoseCompCount", &count);
    count = std::max(0, count);

    for (int sectionIndex = 0; sectionIndex < count; ++sectionIndex)
    {
        PoseCompRow row;
        const int defaultIndex = sectionIndex < m_poseTypes.size() ? sectionIndex : 0;
        row.sectionName = QString("WeldPoseComp%1").arg(sectionIndex);
        row.name = sectionIndex < m_poseTypes.size()
            ? m_poseTypes[sectionIndex].displayName
            : QString("姿态补偿%1").arg(sectionIndex);
        row.segmentKind = sectionIndex < m_poseTypes.size()
            ? m_poseTypes[sectionIndex].segmentKind
            : QString();

        std::string text;
        ini.SetSectionName(row.sectionName.toStdString());
        if (ini.ReadString(false, "Name", text) > 0)
        {
            row.name = QString::fromLocal8Bit(text.c_str());
        }
        if (ini.ReadString(false, "SegmentKind", text) > 0)
        {
            row.segmentKind = QString::fromStdString(text);
        }
        if (row.segmentKind.isEmpty() && defaultIndex < m_poseTypes.size())
        {
            row.segmentKind = m_poseTypes[defaultIndex].segmentKind;
        }

        ReadIniDouble(ini, "Rx", row.poseRx);
        ReadIniDouble(ini, "Ry", row.poseRy);
        ReadIniDouble(ini, "Rz", row.poseRz);
        ReadIniDouble(ini, "CompX", row.compX);
        ReadIniDouble(ini, "CompY", row.compY);
        ReadIniDouble(ini, "CompZ", row.compZ);

        const int targetIndex = FindCompTypeIndex(m_poseTypes, row.segmentKind);
        if (targetIndex >= 0 && targetIndex < m_poseRows.size())
        {
            row.sectionName = QString("WeldPoseComp%1").arg(targetIndex);
            row.name = m_poseTypes[targetIndex].displayName;
            row.segmentKind = m_poseTypes[targetIndex].segmentKind;
            m_poseRows[targetIndex] = row;
        }
    }
}

void WeldSeamCompDialog::LoadSeamParam(const QString& path)
{
    m_seamRows = BuildDefaultSeamRows();

    if (path.isEmpty() || !QFileInfo::exists(path))
    {
        AppendLog("焊道补偿参数不存在，使用默认槽位：" + QDir::toNativeSeparators(path));
        return;
    }

    m_seamRows.clear();
    COPini ini;
    ini.SetFileName(LocalString(path));
    int count = static_cast<int>(BuildDefaultSeamRows().size());
    ini.SetSectionName("ALLWeldSeamComp");
    ini.ReadString(false, "SeamCompCount", &count);
    count = std::max(0, count);

    for (int sectionIndex = 0; sectionIndex < count; ++sectionIndex)
    {
        SeamCompRow row;
        row.sectionName = QString("WeldSeamComp%1").arg(sectionIndex);
        row.name = QString("焊道类型%1").arg(sectionIndex);
        row.segmentKind = QString("SeamType%1").arg(sectionIndex);

        std::string text;
        ini.SetSectionName(row.sectionName.toStdString());
        if (ini.ReadString(false, "Name", text) > 0)
        {
            row.name = QString::fromLocal8Bit(text.c_str());
        }
        if (ini.ReadString(false, "SegmentKind", text) > 0)
        {
            row.segmentKind = QString::fromStdString(text);
        }

        ReadIniDouble(ini, "WeldZComp", row.weldZComp);
        ReadIniDouble(ini, "WeldGunDirComp", row.weldGunDirComp);
        ReadIniDouble(ini, "WeldSeamDirComp", row.weldSeamDirComp);
        m_seamRows.push_back(row);
    }

    if (m_seamRows.isEmpty())
    {
        m_seamRows = BuildDefaultSeamRows();
    }
}

void WeldSeamCompDialog::SetMode(CompMode mode)
{
    if (m_mode == mode)
    {
        RefreshEditor();
        return;
    }

    QString error;
    StoreEditorValues(false, error);
    m_mode = mode;
    if (m_pPoseModeBtn != nullptr)
    {
        m_pPoseModeBtn->setChecked(m_mode == CompMode::Pose);
    }
    if (m_pSeamModeBtn != nullptr)
    {
        m_pSeamModeBtn->setChecked(m_mode == CompMode::Seam);
    }
    RefreshTypeCombo();
    RefreshEditor();
}

void WeldSeamCompDialog::RefreshTypeCombo()
{
    if (m_pTypeCombo == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(m_pTypeCombo);
    const int rowCount = CurrentRowCount();
    const int previousIndex = rowCount > 0 ? std::clamp(m_currentTypeIndex, 0, rowCount - 1) : -1;
    m_pTypeCombo->clear();
    if (m_mode == CompMode::Pose)
    {
        for (const PoseCompRow& row : m_poseRows)
        {
            m_pTypeCombo->addItem(row.name, row.segmentKind);
        }
    }
    else
    {
        for (const SeamCompRow& row : m_seamRows)
        {
            const QString displayName = row.segmentKind.isEmpty()
                ? row.name
                : QString("%1 (%2)").arg(row.name, row.segmentKind);
            m_pTypeCombo->addItem(displayName, row.segmentKind);
        }
    }

    if (m_pTypeCombo->count() > 0)
    {
        m_currentTypeIndex = std::clamp(previousIndex, 0, m_pTypeCombo->count() - 1);
        m_pTypeCombo->setCurrentIndex(m_currentTypeIndex);
    }
    else
    {
        m_currentTypeIndex = -1;
    }
}

void WeldSeamCompDialog::RefreshEditor()
{
    const int index = CurrentTypeIndex();
    m_currentTypeIndex = index;

    if (m_pPoseDisplayWidget != nullptr)
    {
        m_pPoseDisplayWidget->setVisible(m_mode == CompMode::Pose);
    }

    if (m_pPathLabel != nullptr)
    {
        m_pPathLabel->setText(m_mode == CompMode::Pose
            ? NativePathText("姿态补偿配置", CurrentPoseParamPath())
            : NativePathText("焊道补偿配置", CurrentSeamParamPath()));
    }

    if (m_pHintLabel != nullptr)
    {
        m_pHintLabel->setText(m_mode == CompMode::Pose
            ? QString("姿态补偿：按当前点姿态匹配下方姿态值，匹配成功后将 X/Y/Z 补偿按当前姿态旋转到世界坐标后叠加。")
            : QString("焊道补偿：Z 为世界 Z 向；枪反向为垂直 Z 轴和焊道方向的侧向；焊道方向为沿点序切线方向。"));
    }

    if (m_mode == CompMode::Pose)
    {
        if (index >= 0 && index < m_poseRows.size())
        {
            const PoseCompRow& row = m_poseRows[index];
            if (m_pPoseValues[0] != nullptr) m_pPoseValues[0]->setText(FormatNumber(row.poseRx));
            if (m_pPoseValues[1] != nullptr) m_pPoseValues[1]->setText(FormatNumber(row.poseRy));
            if (m_pPoseValues[2] != nullptr) m_pPoseValues[2]->setText(FormatNumber(row.poseRz));
            if (m_pEditLabels[0] != nullptr) m_pEditLabels[0]->setText("X补偿(mm)：");
            if (m_pEditLabels[1] != nullptr) m_pEditLabels[1]->setText("Y补偿(mm)：");
            if (m_pEditLabels[2] != nullptr) m_pEditLabels[2]->setText("Z补偿(mm)：");
            if (m_pEditValues[0] != nullptr) m_pEditValues[0]->setText(FormatNumber(row.compX));
            if (m_pEditValues[1] != nullptr) m_pEditValues[1]->setText(FormatNumber(row.compY));
            if (m_pEditValues[2] != nullptr) m_pEditValues[2]->setText(FormatNumber(row.compZ));
        }
    }
    else
    {
        if (index >= 0 && index < m_seamRows.size())
        {
            const SeamCompRow& row = m_seamRows[index];
            if (m_pEditLabels[0] != nullptr) m_pEditLabels[0]->setText("Z向补偿(mm)：");
            if (m_pEditLabels[1] != nullptr) m_pEditLabels[1]->setText("枪反向补偿(mm)：");
            if (m_pEditLabels[2] != nullptr) m_pEditLabels[2]->setText("焊道方向补偿(mm)：");
            if (m_pEditValues[0] != nullptr) m_pEditValues[0]->setText(FormatNumber(row.weldZComp));
            if (m_pEditValues[1] != nullptr) m_pEditValues[1]->setText(FormatNumber(row.weldGunDirComp));
            if (m_pEditValues[2] != nullptr) m_pEditValues[2]->setText(FormatNumber(row.weldSeamDirComp));
        }
    }
}

bool WeldSeamCompDialog::StoreEditorValues(bool validate, QString& error)
{
    const int index = CurrentTypeIndex();
    if (index < 0)
    {
        return true;
    }

    double values[3] = { 0.0, 0.0, 0.0 };
    for (int valueIndex = 0; valueIndex < 3; ++valueIndex)
    {
        if (m_pEditValues[valueIndex] == nullptr)
        {
            continue;
        }
        bool ok = false;
        values[valueIndex] = m_pEditValues[valueIndex]->text().trimmed().toDouble(&ok);
        if (!ok || !std::isfinite(values[valueIndex]))
        {
            if (validate)
            {
                error = QString("第 %1 个补偿值不是有效数字。").arg(valueIndex + 1);
                return false;
            }
            return true;
        }
    }

    if (m_mode == CompMode::Pose)
    {
        if (index >= m_poseRows.size())
        {
            return true;
        }
        m_poseRows[index].compX = values[0];
        m_poseRows[index].compY = values[1];
        m_poseRows[index].compZ = values[2];
    }
    else
    {
        if (index >= m_seamRows.size())
        {
            return true;
        }
        m_seamRows[index].weldZComp = values[0];
        m_seamRows[index].weldGunDirComp = values[1];
        m_seamRows[index].weldSeamDirComp = values[2];
    }

    return true;
}

bool WeldSeamCompDialog::SaveCurrentParam()
{
    QString error;
    if (!StoreEditorValues(true, error))
    {
        QMessageBox::warning(this, "保存补偿参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    const QString path = CurrentParamPath();
    const bool ok = m_mode == CompMode::Pose
        ? SavePoseParam(path, error)
        : SaveSeamParam(path, error);
    if (!ok)
    {
        QMessageBox::warning(this, "保存补偿参数", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    MarkCleanSnapshot();
    AppendLog(QString("%1已保存。").arg(m_mode == CompMode::Pose ? "姿态补偿参数" : "焊道补偿参数"));
    QMessageBox::information(this, "保存补偿参数", "补偿参数保存完成。");
    return true;
}

bool WeldSeamCompDialog::SavePoseParam(const QString& path, QString& error) const
{
    if (path.isEmpty())
    {
        error = "姿态补偿参数路径为空。";
        return false;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldPoseComp");
    if (!ini.WriteString("PoseCompCount", static_cast<int>(m_poseRows.size()))
        || !ini.WriteString("PoseMatchMaxErrorDeg", m_poseMatchMaxErrorDeg, 6))
    {
        error = "写入姿态补偿总配置失败：" + path;
        return false;
    }

    for (int index = 0; index < m_poseRows.size(); ++index)
    {
        const PoseCompRow& row = m_poseRows[index];
        ini.SetSectionName(QString("WeldPoseComp%1").arg(index).toStdString());
        if (!ini.WriteString("Name", LocalString(row.name))
            || !ini.WriteString("SegmentKind", row.segmentKind.toStdString())
            || !ini.WriteString("Rx", row.poseRx, 6)
            || !ini.WriteString("Ry", row.poseRy, 6)
            || !ini.WriteString("Rz", row.poseRz, 6)
            || !ini.WriteString("CompX", row.compX, 6)
            || !ini.WriteString("CompY", row.compY, 6)
            || !ini.WriteString("CompZ", row.compZ, 6))
        {
            error = "写入姿态补偿槽失败：" + row.sectionName;
            return false;
        }
    }
    return true;
}

bool WeldSeamCompDialog::SaveSeamParam(const QString& path, QString& error) const
{
    if (path.isEmpty())
    {
        error = "焊道补偿参数路径为空。";
        return false;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldSeamComp");
    if (!ini.WriteString("SeamCompCount", static_cast<int>(m_seamRows.size())))
    {
        error = "写入焊道补偿总配置失败：" + path;
        return false;
    }

    for (int index = 0; index < m_seamRows.size(); ++index)
    {
        const SeamCompRow& row = m_seamRows[index];
        ini.SetSectionName(QString("WeldSeamComp%1").arg(index).toStdString());
        if (!ini.WriteString("Name", LocalString(row.name))
            || !ini.WriteString("SegmentKind", row.segmentKind.toStdString())
            || !ini.WriteString("WeldZComp", row.weldZComp, 6)
            || !ini.WriteString("WeldGunDirComp", row.weldGunDirComp, 6)
            || !ini.WriteString("WeldSeamDirComp", row.weldSeamDirComp, 6))
        {
            error = "写入焊道补偿槽失败：" + row.sectionName;
            return false;
        }
    }
    return true;
}

bool WeldSeamCompDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString WeldSeamCompDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << CurrentRobotName()
           << QString::number(m_poseMatchMaxErrorDeg, 'f', 6);

    for (const PoseCompRow& row : m_poseRows)
    {
        fields << row.sectionName << row.name << row.segmentKind
               << QString::number(row.poseRx, 'f', 6)
               << QString::number(row.poseRy, 'f', 6)
               << QString::number(row.poseRz, 'f', 6)
               << QString::number(row.compX, 'f', 6)
               << QString::number(row.compY, 'f', 6)
               << QString::number(row.compZ, 'f', 6);
    }
    for (const SeamCompRow& row : m_seamRows)
    {
        fields << row.sectionName << row.name << row.segmentKind
               << QString::number(row.weldZComp, 'f', 6)
               << QString::number(row.weldGunDirComp, 'f', 6)
               << QString::number(row.weldSeamDirComp, 'f', 6);
    }
    return fields.join('\n');
}

void WeldSeamCompDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void WeldSeamCompDialog::AppendLog(const QString& text)
{
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }
}
