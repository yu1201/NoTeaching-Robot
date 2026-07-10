#include "VirtualWeldTestDialog.h"

#include "ContralUnit.h"
#include "RobotDriverAdaptor.h"
#include "MeasureThenWeldService.h"
#include "RobotDataHelper.h"
#include "WeldProcessFile.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>
#include <thread>

VirtualWeldTestDialog::VirtualWeldTestDialog(ContralUnit* pContralUnit, int unitIndex, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_unitIndex(unitIndex)
{
    setWindowTitle(QStringLiteral("虚拟焊道测试"));
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; margin-top: 12px; padding: 8px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 12px; padding: 12px 18px; font-size: 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:pressed { background: #18303B; }"
        "QPushButton:disabled { background: #27323A; color: #7D8B91; border-color: #364650; }"
        "QCheckBox { color: #BFE8EC; spacing: 8px; font-size: 15px; }"
        "QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #5C7A8B; background: #0B1117; }"
        "QCheckBox::indicator:checked { background: #1E8AA0; border-color: #72D4DD; }"
        "QDoubleSpinBox { background: #05090D; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QComboBox { background: #05090D; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QComboBox::drop-down { width: 34px; border-left: 1px solid #36586A; background: #05090D; }"
        "QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
        "QComboBox QAbstractItemView { background: #081018; color: #F5FAFA; selection-background-color: #2D7D8C; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }");

    auto* rootLayout = new QVBoxLayout(this);

    // ===== 机器人 / 工艺 选择区 =====
    auto* selectorLayout = new QGridLayout();
    selectorLayout->setHorizontalSpacing(14);
    selectorLayout->setVerticalSpacing(8);
    selectorLayout->addWidget(new QLabel(QStringLiteral("机器人："), this), 0, 0);
    m_pRobotCombo = new QComboBox(this);
    m_pRobotCombo->setMinimumWidth(220);
    selectorLayout->addWidget(m_pRobotCombo, 0, 1);
    selectorLayout->addWidget(new QLabel(QStringLiteral("焊接工艺："), this), 1, 0);
    m_pWeldProcessCombo = new QComboBox(this);
    m_pWeldProcessCombo->setMinimumWidth(360);
    selectorLayout->addWidget(m_pWeldProcessCombo, 1, 1, 1, 3);
    selectorLayout->setColumnStretch(4, 1);
    rootLayout->addLayout(selectorLayout);

    // ===== 参数区 =====
    auto* paramGroup = new QGroupBox(QStringLiteral("虚拟焊道参数"), this);
    auto* paramForm = new QFormLayout(paramGroup);

    m_lengthSpin = new QDoubleSpinBox(paramGroup);
    m_lengthSpin->setRange(1.0, 5000.0);
    m_lengthSpin->setDecimals(1);
    m_lengthSpin->setSingleStep(10.0);
    m_lengthSpin->setValue(200.0);
    m_lengthSpin->setSuffix(QStringLiteral(" mm"));
    paramForm->addRow(QStringLiteral("焊道长度"), m_lengthSpin);

    m_stepSpin = new QDoubleSpinBox(paramGroup);
    m_stepSpin->setRange(0.5, 100.0);
    m_stepSpin->setDecimals(2);
    m_stepSpin->setSingleStep(0.5);
    m_stepSpin->setValue(4.0);
    m_stepSpin->setSuffix(QStringLiteral(" mm"));
    paramForm->addRow(QStringLiteral("点间距(机器人逐点)"), m_stepSpin);

    m_directionCombo = new QComboBox(paramGroup);
    m_directionCombo->addItem(QStringLiteral("+Y (基坐标)"), 1);
    m_directionCombo->addItem(QStringLiteral("-Y (基坐标)"), -1);
    paramForm->addRow(QStringLiteral("方向"), m_directionCombo);

    m_actualWeldCheck = new QCheckBox(QStringLiteral("实焊模式（含摆动 WEAVEDATA / 引弧）"), paramGroup);
    m_actualWeldCheck->setChecked(true);
    paramForm->addRow(QStringLiteral("焊接模式"), m_actualWeldCheck);

    rootLayout->addWidget(paramGroup);

    // ===== 当前位置区 =====
    auto* posRow = new QHBoxLayout();
    m_readPosBtn = new QPushButton(QStringLiteral("读取当前机器人位置"), this);
    m_posLabel = new QLabel(QStringLiteral("当前起点：未读取"), this);
    m_posLabel->setWordWrap(true);
    posRow->addWidget(m_readPosBtn);
    posRow->addWidget(m_posLabel, 1);
    rootLayout->addLayout(posRow);

    // ===== 操作区 =====
    auto* actionRow = new QHBoxLayout();
    m_generateBtn = new QPushButton(QStringLiteral("生成虚拟焊道"), this);
    m_runBtn = new QPushButton(QStringLiteral("下发并运行(实焊含摆动)"), this);
    m_runBtn->setEnabled(false);
    actionRow->addWidget(m_generateBtn);
    actionRow->addWidget(m_runBtn);
    rootLayout->addLayout(actionRow);

    // ===== 日志区 =====
    m_logEdit = new QPlainTextEdit(this);
    m_logEdit->setReadOnly(true);
    m_logEdit->setMinimumHeight(220);
    rootLayout->addWidget(m_logEdit, 1);

    connect(m_readPosBtn, &QPushButton::clicked, this, &VirtualWeldTestDialog::OnReadCurrentPos);
    connect(m_generateBtn, &QPushButton::clicked, this, &VirtualWeldTestDialog::OnGenerate);
    connect(m_runBtn, &QPushButton::clicked, this, &VirtualWeldTestDialog::OnRunOnRobot);
    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &VirtualWeldTestDialog::OnRobotChanged);
    connect(m_pWeldProcessCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &VirtualWeldTestDialog::OnWeldProcessChanged);

    AppendLog(QStringLiteral("说明：取机器人当前位姿沿 Y 方向造一条虚拟直线焊道，复用正常轨迹生成（读取所选机器人/工艺已保存的参数：姿态/圆滑/摆动），"
                             "产出 _WeldPose_2mm / _SeamComp / srp / srd；点间距即机器人最终逐点执行间距，便于测试密集点摆动。"));
    AppendLog(QStringLiteral("提示：实焊模式下机器人运行会真实引弧，如仅观察摆动请现场自行断送丝/断气。"));

    LoadRobotList();
}

void VirtualWeldTestDialog::closeEvent(QCloseEvent* event)
{
    // 嵌入页 close 只隐藏，后台对象不会销毁；运行中允许回主页使用固定安全停止。
    QDialog::closeEvent(event);
}

VirtualWeldTestDialog::~VirtualWeldTestDialog() = default;

namespace
{
// 与「先测后焊」页一致的工艺分组键：同一工件 + 焊脚为一组（多层共一组）。
QString BuildWeldProcessGroupKey(const T_WELD_PARA& weld)
{
    return QString("%1|%2")
        .arg(DecodeRobotMessageText(weld.strWorkPeace))
        .arg(weld.dWeldAngleSize, 0, 'f', 3);
}
}  // namespace

QString VirtualWeldTestDialog::CurrentRobotName() const
{
    if (m_pRobotCombo != nullptr && m_pRobotCombo->currentIndex() >= 0)
    {
        const QString robotName = m_pRobotCombo->currentData(Qt::UserRole + 1).toString().trimmed();
        if (!robotName.isEmpty())
        {
            return robotName;
        }
    }
    RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, m_unitIndex);
    if (driver != nullptr && !driver->m_sRobotName.empty())
    {
        return QString::fromStdString(driver->m_sRobotName);
    }
    return QString();
}

void VirtualWeldTestDialog::LoadRobotList()
{
    if (m_pRobotCombo == nullptr)
    {
        return;
    }

    m_loadingSelectors = true;
    const QSignalBlocker blocker(m_pRobotCombo);
    m_pRobotCombo->clear();

    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    int selectedIndex = -1;
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        const int row = m_pRobotCombo->count();
        m_pRobotCombo->addItem(info.displayName, info.unitIndex);
        m_pRobotCombo->setItemData(row, info.robotName, Qt::UserRole + 1);
        if (info.unitIndex == m_unitIndex)
        {
            selectedIndex = row;
        }
    }
    if (selectedIndex < 0 && m_pRobotCombo->count() > 0)
    {
        selectedIndex = 0;
    }
    if (selectedIndex >= 0)
    {
        m_pRobotCombo->setCurrentIndex(selectedIndex);
        m_unitIndex = m_pRobotCombo->currentData().toInt();
    }
    m_pRobotCombo->setEnabled(m_pRobotCombo->count() > 0);

    m_loadingSelectors = false;
    LoadWeldProcessList();
}

void VirtualWeldTestDialog::LoadWeldProcessList()
{
    if (m_pWeldProcessCombo == nullptr)
    {
        return;
    }

    m_loadingSelectors = true;
    const QSignalBlocker blocker(m_pWeldProcessCombo);
    m_pWeldProcessCombo->clear();
    m_pWeldProcessCombo->setEnabled(false);

    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        m_pWeldProcessCombo->addItem(QStringLiteral("未选择有效机器人"), -1);
        m_loadingSelectors = false;
        return;
    }

    WeldProcessFile processFile(*m_pContralUnit, m_unitIndex);
    if (!processFile.Init())
    {
        m_pWeldProcessCombo->addItem(QStringLiteral("请先在工艺界面重新创建工艺"), -1);
        m_loadingSelectors = false;
        return;
    }

    const std::vector<T_WELD_PARA>& weldList = processFile.GetWeldParaList();
    QString selectedKey;
    if (!weldList.empty())
    {
        const int useIndex = std::clamp(processFile.GetUseWeldParaNo(), 0, static_cast<int>(weldList.size()) - 1);
        selectedKey = BuildWeldProcessGroupKey(weldList[useIndex]);
    }
    int selectedComboIndex = 0;
    QStringList seenKeys;
    for (int index = 0; index < static_cast<int>(weldList.size()); ++index)
    {
        const T_WELD_PARA& weld = weldList[index];
        const QString groupKey = BuildWeldProcessGroupKey(weld);
        if (seenKeys.contains(groupKey))
        {
            continue;
        }
        seenKeys.append(groupKey);
        const QString workpiece = DecodeRobotMessageText(weld.strWorkPeace);
        const QString displayName = QString("%1. %2 | 焊脚%3")
            .arg(m_pWeldProcessCombo->count() + 1)
            .arg(workpiece.isEmpty() ? QStringLiteral("未命名工艺") : workpiece)
            .arg(weld.dWeldAngleSize, 0, 'f', 1);
        m_pWeldProcessCombo->addItem(displayName, index);
        if (groupKey == selectedKey)
        {
            selectedComboIndex = m_pWeldProcessCombo->count() - 1;
        }
    }

    if (m_pWeldProcessCombo->count() > 0)
    {
        m_pWeldProcessCombo->setEnabled(true);
        m_pWeldProcessCombo->setCurrentIndex(std::clamp(selectedComboIndex, 0, m_pWeldProcessCombo->count() - 1));
    }
    else
    {
        m_pWeldProcessCombo->addItem(QStringLiteral("未读取到焊接工艺"), -1);
    }

    m_loadingSelectors = false;
}

void VirtualWeldTestDialog::OnRobotChanged(int index)
{
    Q_UNUSED(index);
    if (m_loadingSelectors || m_running || m_pRobotCombo == nullptr)
    {
        return;
    }
    m_unitIndex = m_pRobotCombo->currentData().toInt();
    // 切换机器人后，之前读到的起点/已生成文件不再适用，需重新读取与生成。
    m_hasStartCoors = false;
    m_lastWeldPosePath.clear();
    if (m_posLabel != nullptr) m_posLabel->setText(QStringLiteral("当前起点：未读取"));
    if (m_runBtn != nullptr) m_runBtn->setEnabled(false);
    LoadWeldProcessList();
    AppendLog(QString(QStringLiteral("当前机器人已切换为：%1")).arg(m_pRobotCombo->currentText()));
}

void VirtualWeldTestDialog::OnWeldProcessChanged(int index)
{
    Q_UNUSED(index);
    if (m_loadingSelectors || m_running || m_pWeldProcessCombo == nullptr)
    {
        return;
    }
    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return;
    }
    const int weldProcessIndex = m_pWeldProcessCombo->currentData().toInt();
    if (weldProcessIndex < 0)
    {
        return;
    }
    WeldProcessFile processFile(*m_pContralUnit, m_unitIndex);
    if (!processFile.Init() || !processFile.UpdateUseWeldParaNo(weldProcessIndex))
    {
        AppendLog(QStringLiteral("切换焊接工艺失败，请先在工艺界面重新创建工艺。"));
        return;
    }
    AppendLog(QString(QStringLiteral("当前焊接工艺已切换为：%1")).arg(m_pWeldProcessCombo->currentText()));
}

RobotDriverAdaptor* VirtualWeldTestDialog::ResolveDriver()
{
    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, QStringLiteral("虚拟焊道测试"), QStringLiteral("未找到可用的控制单元。"));
        return nullptr;
    }
    auto* driver = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    if (driver == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("虚拟焊道测试"), QStringLiteral("当前控制单元未创建驱动。"));
        return nullptr;
    }
    return driver;
}

QString VirtualWeldTestDialog::ResolveRobotName(RobotDriverAdaptor* driver) const
{
    if (driver != nullptr && !driver->m_sRobotName.empty())
    {
        return QString::fromStdString(driver->m_sRobotName);
    }
    if (m_pContralUnit != nullptr
        && m_unitIndex >= 0
        && m_unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        const QString name = m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].sUnitName.c_str();
        if (!name.trimmed().isEmpty())
        {
            return name;
        }
    }
    return QStringLiteral("RobotA");
}

void VirtualWeldTestDialog::AppendLog(const QString& text)
{
    if (m_logEdit != nullptr)
    {
        m_logEdit->appendPlainText(text);
    }
}

void VirtualWeldTestDialog::SetRunning(bool running)
{
    m_running = running;
    if (m_readPosBtn != nullptr) m_readPosBtn->setEnabled(!running);
    if (m_generateBtn != nullptr) m_generateBtn->setEnabled(!running);
    if (m_runBtn != nullptr) m_runBtn->setEnabled(!running && !m_lastWeldPosePath.isEmpty());
    if (m_pRobotCombo != nullptr) m_pRobotCombo->setEnabled(!running && m_pRobotCombo->count() > 0);
    if (m_pWeldProcessCombo != nullptr) m_pWeldProcessCombo->setEnabled(!running && m_pWeldProcessCombo->count() > 0);
    if (m_lengthSpin != nullptr) m_lengthSpin->setEnabled(!running);
    if (m_stepSpin != nullptr) m_stepSpin->setEnabled(!running);
    if (m_directionCombo != nullptr) m_directionCombo->setEnabled(!running);
    if (m_actualWeldCheck != nullptr) m_actualWeldCheck->setEnabled(!running);
}

void VirtualWeldTestDialog::OnReadCurrentPos()
{
    RobotDriverAdaptor* driver = ResolveDriver();
    if (driver == nullptr)
    {
        return;
    }
    if (!driver->IsConnected())
    {
        QMessageBox::warning(this, QStringLiteral("读取当前位置"), QStringLiteral("机器人未连接，无法读取当前位置。"));
        return;
    }

    const T_ROBOT_COORS coors = driver->GetCurrentPos();
    m_startCoors = coors;
    m_hasStartCoors = true;
    const QString text = QString("当前起点：X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6")
        .arg(coors.dX, 0, 'f', 3)
        .arg(coors.dY, 0, 'f', 3)
        .arg(coors.dZ, 0, 'f', 3)
        .arg(coors.dRX, 0, 'f', 3)
        .arg(coors.dRY, 0, 'f', 3)
        .arg(coors.dRZ, 0, 'f', 3);
    if (m_posLabel != nullptr)
    {
        m_posLabel->setText(text);
    }
    AppendLog(QStringLiteral("已读取机器人当前位置作为虚拟焊道起点。"));
    AppendLog(text);
}

void VirtualWeldTestDialog::OnGenerate()
{
    if (m_running)
    {
        return;
    }
    if (!m_hasStartCoors)
    {
        QMessageBox::information(this, QStringLiteral("生成虚拟焊道"), QStringLiteral("请先点击“读取当前机器人位置”。"));
        return;
    }

    QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        robotName = ResolveRobotName(ResolveDriver());
    }
    if (robotName.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("生成虚拟焊道"), QStringLiteral("未选择有效机器人。"));
        return;
    }
    const double lengthMm = m_lengthSpin->value();
    const double stepMm = m_stepSpin->value();
    const int dir = m_directionCombo->currentData().toInt();
    const bool actualWeld = m_actualWeldCheck->isChecked();

    MeasureThenWeldService service;
    QString outputDir;
    QString weldPosePath;
    QString srpPath;
    QString srdPath;
    QString programName;
    QString summary;
    QString error;
    auto appendLog = [this](const QString& s) { AppendLog(s); };

    AppendLog(QStringLiteral("———— 开始生成虚拟焊道 ————"));
    const bool ok = service.GenerateVirtualStraightWeldFiles(
        robotName, m_startCoors, lengthMm, stepMm, dir, actualWeld,
        outputDir, weldPosePath, srpPath, srdPath, programName, summary, error, appendLog);
    if (!ok)
    {
        AppendLog(QStringLiteral("生成失败：") + error);
        QMessageBox::warning(this, QStringLiteral("生成虚拟焊道"), error);
        return;
    }

    m_lastWeldPosePath = weldPosePath;
    m_lastPointStepMm = stepMm;
    AppendLog(summary);
    AppendLog(QStringLiteral("srp=") + srpPath);
    AppendLog(QStringLiteral("srd=") + srdPath);
    AppendLog(QStringLiteral("生成完成，可点击“下发并运行”执行（实焊模式将真实引弧，请确认现场安全）。"));
    if (m_runBtn != nullptr)
    {
        m_runBtn->setEnabled(true);
    }
}

void VirtualWeldTestDialog::OnRunOnRobot()
{
    if (m_running)
    {
        return;
    }
    if (m_lastWeldPosePath.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("下发并运行"), QStringLiteral("请先生成虚拟焊道。"));
        return;
    }

    RobotDriverAdaptor* driver = ResolveDriver();
    if (driver == nullptr)
    {
        return;
    }
    if (!driver->IsConnected())
    {
        QMessageBox::warning(this, QStringLiteral("下发并运行"), QStringLiteral("机器人未连接，无法下发运行。"));
        return;
    }

    const bool actualWeld = m_actualWeldCheck->isChecked();
    const QString confirmText = actualWeld
        ? QStringLiteral("即将下发并运行虚拟焊道（实焊模式，含摆动，会真实引弧）。\n"
                         "机器人将先移动到安全位再执行焊道，请确认现场安全（如仅测摆动请断送丝/断气）。\n是否继续？")
        : QStringLiteral("即将下发并运行虚拟焊道（空跑模式，不引弧、不摆动）。\n机器人将移动执行轨迹，请确认现场安全。\n是否继续？");
    if (QMessageBox::question(this, QStringLiteral("下发并运行"), confirmText,
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("虚拟焊道测试"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, QStringLiteral("下发并运行"), leaseError);
        return;
    }

    const QString execPath = m_lastWeldPosePath;
    const double pointStepMm = m_lastPointStepMm;

    SetRunning(true);
    AppendLog(QStringLiteral("———— 开始下发并运行虚拟焊道 ————"));

    QPointer<VirtualWeldTestDialog> self(this);
    std::thread([self, driver, execPath, pointStepMm, actualWeld, operationLease]()
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
                            self->AppendLog(QStringLiteral("读取预设参数失败：") + error);
                            self->SetRunning(false);
                        }
                    }, Qt::QueuedConnection);
                return;
            }
            param.bDoActualWeld = actualWeld;
            // 与生成侧一致：虚拟焊道按几何方向(±Y)前向执行，避免被工艺反向焊接翻转导致方向不一致。
            param.nWeldDirection = 1;

            auto appendLog = [self](const QString& s)
                {
                    QMetaObject::invokeMethod(qApp, [self, s]()
                        {
                            if (self != nullptr) self->AppendLog(s);
                        }, Qt::QueuedConnection);
                };
            auto setFlowStep = [self](const QString& s)
                {
                    QMetaObject::invokeMethod(qApp, [self, s]()
                        {
                            if (self != nullptr) self->AppendLog(QStringLiteral("步骤：") + s);
                        }, Qt::QueuedConnection);
                };
            auto checkpoint = [self](const QString& title, const QString& message) -> bool
                {
                    bool proceed = false;
                    QMetaObject::invokeMethod(qApp, [self, &title, &message, &proceed]()
                        {
                            if (self != nullptr)
                            {
                                proceed = QMessageBox::question(self, title, message,
                                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes;
                            }
                        }, Qt::BlockingQueuedConnection);
                    return proceed;
                };

            QString summary;
            QString execError;
            const bool ok = service.ExecuteWeldPoseFileWithSafePos(
                driver, execPath, param, summary, execError,
                nullptr, nullptr, appendLog, setFlowStep, checkpoint, pointStepMm, /*allowPointwiseWeave=*/true);

            QMetaObject::invokeMethod(qApp, [self, ok, summary, execError]()
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(ok
                            ? (QStringLiteral("执行完成：") + summary)
                            : (QStringLiteral("执行失败/中止：") + execError));
                        self->SetRunning(false);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}
