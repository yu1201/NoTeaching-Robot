#include "WeldProcessDialog.h"
#include "ui_WeldProcessDialog.h"
#include "ContralUnit.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "WindowStyleHelper.h"

#include <QAbstractSpinBox>
#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QBoxLayout>
#include <QCheckBox>
#include <QCloseEvent>
#include <QShowEvent>
#include <QComboBox>
#include <QStandardItemModel>
#include <QBrush>
#include <QColor>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QListWidgetItem>
#include <QLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QSpinBox>
#include <QStyleOptionSpinBox>
#include <QStringList>
#include <QVBoxLayout>

namespace
{
constexpr double kDoubleMin = -999999.0;
constexpr double kDoubleMax = 999999.0;
constexpr int kGridFieldWidth = 136;
constexpr int kSingleFieldWidth = 164;
constexpr int kSpinFieldHeight = 32;

class PlusMinusDoubleSpinBox : public QDoubleSpinBox
{
public:
    explicit PlusMinusDoubleSpinBox(QWidget* parent = nullptr)
        : QDoubleSpinBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QDoubleSpinBox::paintEvent(event);
        if (buttonSymbols() != QAbstractSpinBox::NoButtons)
        {
            DrawPlusMinus();
        }
    }

private:
    void DrawPlusMinus()
    {
        if (buttonSymbols() == QAbstractSpinBox::NoButtons)
        {
            return;
        }

        QStyleOptionSpinBox option;
        initStyleOption(&option);

        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(QColor("#eef2f8"));
        QFont font = painter.font();
        font.setBold(true);
        font.setPixelSize(11);
        painter.setFont(font);

        const QRect upRect = style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxUp, this);
        const QRect downRect = style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxDown, this);
        painter.drawText(upRect, Qt::AlignCenter, "+");
        painter.drawText(downRect, Qt::AlignCenter, "-");
    }
};

class PlusMinusSpinBox : public QSpinBox
{
public:
    explicit PlusMinusSpinBox(QWidget* parent = nullptr)
        : QSpinBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QSpinBox::paintEvent(event);
        if (buttonSymbols() != QAbstractSpinBox::NoButtons)
        {
            DrawPlusMinus();
        }
    }

private:
    void DrawPlusMinus()
    {
        if (buttonSymbols() == QAbstractSpinBox::NoButtons)
        {
            return;
        }

        QStyleOptionSpinBox option;
        initStyleOption(&option);

        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(QColor("#eef2f8"));
        QFont font = painter.font();
        font.setBold(true);
        font.setPixelSize(11);
        painter.setFont(font);

        const QRect upRect = style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxUp, this);
        const QRect downRect = style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxDown, this);
        painter.drawText(upRect, Qt::AlignCenter, "+");
        painter.drawText(downRect, Qt::AlignCenter, "-");
    }
};

QGridLayout* CreateTwoColumnForm(QWidget* parent)
{
    auto* layout = new QGridLayout(parent);
    layout->setContentsMargins(12, 8, 12, 12);
    layout->setHorizontalSpacing(10);
    layout->setVerticalSpacing(8);
    layout->setColumnMinimumWidth(0, 150);
    layout->setColumnMinimumWidth(2, 150);
    layout->setColumnStretch(0, 0);
    layout->setColumnStretch(1, 1);
    layout->setColumnStretch(2, 0);
    layout->setColumnStretch(3, 1);
    return layout;
}

void AddTwoColumnField(QGridLayout* layout, const QString& labelText, QWidget* field)
{
    if (layout == nullptr || field == nullptr)
    {
        return;
    }

    const int fieldIndex = layout->property("fieldIndex").toInt();
    const int row = fieldIndex / 2;
    const int col = (fieldIndex % 2) * 2;
    layout->setProperty("fieldIndex", fieldIndex + 1);

    QLabel* label = new QLabel(labelText, layout->parentWidget());
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    label->setMinimumWidth(150);
    label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    layout->addWidget(label, row, col);
    layout->addWidget(field, row, col + 1);
}

int UnitLabelWidth(const QString& unit)
{
    const QString trimmed = unit.trimmed();
    if (trimmed.isEmpty())
    {
        return 0;
    }
    if (trimmed == "mm/min")
    {
        return 72;
    }
    return 36;
}

void LockMinimumContentSize(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    if (QLayout* layout = widget->layout())
    {
        layout->setSizeConstraint(QLayout::SetMinimumSize);
    }

    const QSize hint = widget->minimumSizeHint().expandedTo(widget->sizeHint());
    widget->setMinimumSize(hint);
}

QLabel* CreateUnitLabel(QWidget* parent, const QString& unit)
{
    auto* label = new QLabel(unit, parent);
    label->setObjectName("fieldUnitLabel");
    label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    label->setFixedWidth(UnitLabelWidth(unit));
    return label;
}

QWidget* WrapFieldWithUnit(QWidget* parent, QWidget* field, const QString& unit)
{
    if (unit.trimmed().isEmpty())
    {
        return field;
    }

    auto* host = new QWidget(parent);
    host->setObjectName("fieldUnitHost");
    auto* layout = new QHBoxLayout(host);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(5);
    layout->addWidget(field, 1);
    layout->addWidget(CreateUnitLabel(host, unit));
    layout->addStretch(1);
    host->setMinimumWidth(kGridFieldWidth + UnitLabelWidth(unit) + 5);
    host->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    return host;
}

void PrepareFlatSpinBox(QAbstractSpinBox* spin, int width)
{
    if (spin == nullptr)
    {
        return;
    }

    spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    spin->setFrame(false);
    spin->setMinimumWidth(width > 2 ? width - 2 : width);
    spin->setFixedHeight(kSpinFieldHeight > 2 ? kSpinFieldHeight - 2 : kSpinFieldHeight);
    spin->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    if (QLineEdit* editor = spin->findChild<QLineEdit*>())
    {
        editor->setFrame(false);
    }
}

QFrame* WrapSpinFieldFrame(QWidget* parent, QAbstractSpinBox* spin, int width)
{
    auto* frame = new QFrame(parent);
    frame->setObjectName("spinFieldFrame");
    frame->setMinimumSize(width, kSpinFieldHeight);
    frame->setFixedHeight(kSpinFieldHeight);
    frame->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);

    PrepareFlatSpinBox(spin, width);

    auto* layout = new QHBoxLayout(frame);
    layout->setContentsMargins(1, 1, 1, 1);
    layout->setSpacing(0);
    layout->addWidget(spin, 1);
    return frame;
}

QFrame* CreateTabConnectorLine(QWidget* parent, int fixedWidth = -1)
{
    auto* line = new QFrame(parent);
    line->setObjectName("tabConnectorLine");
    line->setFrameShape(QFrame::NoFrame);
    line->setFixedHeight(1);
    if (fixedWidth >= 0)
    {
        line->setFixedWidth(fixedWidth);
        line->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    }
    else
    {
        line->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }
    return line;
}

bool IsChecked(const QCheckBox* check)
{
    return check != nullptr && check->isChecked();
}

void SetChecked(QCheckBox* check, int value)
{
    if (check != nullptr)
    {
        check->setChecked(value != 0);
    }
}

void SetComboByData(QComboBox* combo, int value)
{
    if (combo == nullptr)
    {
        return;
    }
    const int index = combo->findData(value);
    combo->setCurrentIndex(index >= 0 ? index : 0);
}

int ComboData(const QComboBox* combo, int fallback = 0)
{
    if (combo == nullptr || combo->currentIndex() < 0)
    {
        return fallback;
    }
    return combo->currentData().toInt();
}

void PopulateArcModeCombo(QComboBox* combo)
{
    if (combo == nullptr)
    {
        return;
    }

    combo->clear();
    combo->addItem("0 直流一元", 0);
    combo->addItem("1 脉冲一元", 1);
    combo->addItem("2 JOB模式", 2);
    combo->addItem("3 近控模式", 3);
    combo->addItem("4 分别", 4);
    combo->addItem("5 CC/CV", 5);
    combo->addItem("6 TIG", 6);
    combo->addItem("7 CMT", 7);
}
}

WeldProcessDialog::WeldProcessDialog(const T_CONTRAL_UNIT& unitInfo, ContralUnit* pContralUnit, QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::WeldProcessDialog())
    , m_file(unitInfo)
    , m_unitName(QString::fromUtf8(unitInfo.sUnitName.c_str()))
{
    m_pContralUnit = pContralUnit;
    ui->setupUi(this);
    ApplyUnifiedWindowChrome(this);
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint);
    ui->contentWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    if (QBoxLayout* dialogLayout = qobject_cast<QBoxLayout*>(layout()))
    {
        dialogLayout->setStretchFactor(ui->contentWidget, 1);
    }
    BuildEditorUi();
    ApplyDialogStyle();

    connect(ui->reloadBtn, &QPushButton::clicked, this, &WeldProcessDialog::ReloadData);
    connect(ui->saveBtn, &QPushButton::clicked, this, &WeldProcessDialog::SaveData);
    connect(ui->closeBtn, &QPushButton::clicked, this, &QDialog::close);
    connect(m_weldListWidget, &QListWidget::currentRowChanged, this, &WeldProcessDialog::OnWeldSelectionChanged);
    m_weldListWidget->setSortingEnabled(false);

    LoadToUi();
}

WeldProcessDialog::~WeldProcessDialog()
{
    delete ui;
}

void WeldProcessDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    // 工艺数据也可在补偿界面修改（圆弧过渡/点间距/选用项）；页面缓存复用，
    // 每次显示时无未保存编辑则重载，保证两个界面看到同一份数据。
    if (!HasUnsavedChanges())
    {
        LoadToUi();
    }
}

void WeldProcessDialog::closeEvent(QCloseEvent* event)
{
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "工艺参数", [this]() { return SaveData(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

void WeldProcessDialog::BuildEditorUi()
{
    if (auto* oldLayout = ui->contentWidget->layout())
    {
        QLayoutItem* item = nullptr;
        while ((item = oldLayout->takeAt(0)) != nullptr)
        {
            if (item->widget() != nullptr)
            {
                item->widget()->deleteLater();
            }
            delete item;
        }
        delete oldLayout;
    }

    auto* splitterLayout = new QHBoxLayout(ui->contentWidget);
    splitterLayout->setContentsMargins(0, 0, 0, 0);
    splitterLayout->setSpacing(12);

    auto* leftPanel = new QFrame(ui->contentWidget);
    leftPanel->setObjectName("leftPanel");
    leftPanel->setMinimumWidth(380);
    leftPanel->setMaximumWidth(520);
    auto* leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(12, 12, 12, 12);
    leftLayout->setSpacing(8);

    auto* robotLabel = new QLabel("机器人", leftPanel);
    robotLabel->setObjectName("sectionTitle");
    leftLayout->addWidget(robotLabel);
    m_robotCombo = new QComboBox(leftPanel);
    if (m_pContralUnit != nullptr)
    {
        for (const T_CONTRAL_UNIT& unit : m_pContralUnit->m_vtContralUnitInfo)
        {
            const QString uname = QString::fromUtf8(unit.sUnitName.c_str());
            m_robotCombo->addItem(uname, uname);
        }
        const int curIdx = m_robotCombo->findData(m_unitName);
        if (curIdx >= 0) { m_robotCombo->setCurrentIndex(curIdx); }
    }
    leftLayout->addWidget(m_robotCombo);
    connect(m_robotCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WeldProcessDialog::OnRobotChanged);

    auto* listTitle = new QLabel("工艺列表", leftPanel);
    listTitle->setObjectName("sectionTitle");
    leftLayout->addWidget(listTitle);

    auto* weldButtonRow = new QHBoxLayout();
    weldButtonRow->setSpacing(6);
    m_addWeldButton = new QPushButton("+", leftPanel);
    m_removeWeldButton = new QPushButton("-", leftPanel);
    m_addWeldButton->setObjectName("roundActionButton");
    m_removeWeldButton->setObjectName("roundActionButton");
    m_addWeldButton->setFixedSize(28, 28);
    m_removeWeldButton->setFixedSize(28, 28);
    weldButtonRow->addWidget(m_addWeldButton);
    weldButtonRow->addWidget(m_removeWeldButton);
    weldButtonRow->addStretch();
    leftLayout->addLayout(weldButtonRow);

    auto* listRow = new QHBoxLayout();
    listRow->setSpacing(10);

    m_weldListWidget = new QListWidget(leftPanel);
    m_weldListWidget->setObjectName("weldListWidget");
    m_weldListWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_weldListWidget->setDefaultDropAction(Qt::MoveAction);
    m_weldListWidget->setDragEnabled(true);
    m_weldListWidget->setAcceptDrops(true);
    m_weldListWidget->setDropIndicatorShown(true);
    m_weldListWidget->setDragDropOverwriteMode(false);
    listRow->addWidget(m_weldListWidget, 1);

    auto* beadPanel = new QFrame(leftPanel);
    beadPanel->setObjectName("beadPanel");
    beadPanel->setFixedWidth(92);
    auto* beadLayout = new QVBoxLayout(beadPanel);
    beadLayout->setContentsMargins(6, 6, 6, 6);
    beadLayout->setSpacing(8);

    auto* beadButtonRow = new QHBoxLayout();
    beadButtonRow->setSpacing(6);
    m_addBeadButton = new QPushButton("+", beadPanel);
    m_removeBeadButton = new QPushButton("-", beadPanel);
    m_addBeadButton->setObjectName("roundActionButton");
    m_removeBeadButton->setObjectName("roundActionButton");
    m_addBeadButton->setFixedSize(28, 28);
    m_removeBeadButton->setFixedSize(28, 28);
    beadButtonRow->addWidget(m_addBeadButton);
    beadButtonRow->addWidget(m_removeBeadButton);
    beadButtonRow->addStretch();
    beadLayout->addLayout(beadButtonRow);

    m_beadListWidget = new QListWidget(beadPanel);
    m_beadListWidget->setObjectName("beadListWidget");
    beadLayout->addWidget(m_beadListWidget, 1);
    listRow->addWidget(beadPanel);

    leftLayout->addLayout(listRow, 1);
    splitterLayout->addWidget(leftPanel, 1);

    auto* editorRoot = new QWidget(ui->contentWidget);
    editorRoot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* editorLayout = new QVBoxLayout(editorRoot);
    editorLayout->setContentsMargins(0, 0, 0, 0);
    editorLayout->setSpacing(0);

    auto* tabHostContainer = new QWidget(editorRoot);
    tabHostContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* tabHostLayout = new QVBoxLayout(tabHostContainer);
    tabHostLayout->setContentsMargins(6, 0, 6, 8);
    tabHostLayout->setSpacing(0);

    auto* switchRow = new QHBoxLayout();
    switchRow->setContentsMargins(0, 0, 0, 0);
    switchRow->setSpacing(0);
    m_normalPageButton = new QPushButton("常规工艺参数", editorRoot);
    m_weavePageButton = new QPushButton("摆动参数", editorRoot);
    m_trackPageButton = new QPushButton("跟踪参数", editorRoot);
    switchRow->addWidget(CreateTabConnectorLine(tabHostContainer, 18), 0, Qt::AlignBottom);
    for (QPushButton* button : { m_normalPageButton, m_weavePageButton, m_trackPageButton })
    {
        button->setObjectName("tabStripButton");
        button->setCheckable(true);
        button->setMinimumHeight(42);
        switchRow->addWidget(button);
    }
    m_normalPageButton->setChecked(true);
    switchRow->addWidget(CreateTabConnectorLine(tabHostContainer), 1, Qt::AlignBottom);
    tabHostLayout->addLayout(switchRow);

    auto* editorFrame = new QFrame(editorRoot);
    editorFrame->setObjectName("tabHostPanel");
    editorFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* editorFrameLayout = new QVBoxLayout(editorFrame);
    editorFrameLayout->setContentsMargins(8, 8, 8, 8);
    editorFrameLayout->setSpacing(0);

    auto* basicGroup = new QGroupBox("基础工艺参数", editorRoot);
    auto* basicLayout = new QFormLayout(basicGroup);
    basicLayout->setContentsMargins(14, 18, 14, 12);
    basicLayout->setLabelAlignment(Qt::AlignRight);
    basicLayout->setHorizontalSpacing(12);
    basicLayout->setVerticalSpacing(9);
    basicLayout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    m_workPeaceEdit = AddSingleTextField(basicLayout, "工件名称");
    m_weldTypeEdit = AddSingleTextField(basicLayout, "焊接类型");
    m_arcModeCombo = AddSingleComboField(basicLayout, "焊接模式");
    PopulateArcModeCombo(m_arcModeCombo);
    SetComboByData(m_arcModeCombo, 4);
    m_weldAngleSizeSpin = AddSingleDoubleFieldWithUnit(basicLayout, "焊脚尺寸", "mm");
    m_weaveEnableCheck = AddSingleSwitchField(basicLayout, "是否启用摆动");
    m_trackEnableCheck = AddSingleSwitchField(basicLayout, "是否启用跟踪");
    m_weldPostureCombo = AddSingleComboField(basicLayout, "姿态参数");
    m_weldPostureCombo->addItem("NULL(不指定)", 0);
    m_weldPostureCombo->addItem("可变", 1);
    m_weldPostureCombo->addItem("恒定", 2);
    m_weldPostureCombo->addItem("腕关节", 3);
    SetComboByData(m_weldPostureCombo, 1);
    m_dynamicModeCombo = AddSingleComboField(basicLayout, "动态特性");
    m_dynamicModeCombo->addItem("NULL(机器人默认动态)", 0);
    m_dynamicModeCombo->addItem("ntdyn0(程序速度)", 1);
    SetComboByData(m_dynamicModeCombo, 0);
    m_weldOverlapSpin = AddSingleDoubleFieldWithUnit(basicLayout, "圆滑(过渡比例)", "%");
    if (m_weldOverlapSpin != nullptr) { m_weldOverlapSpin->setRange(0.0, 200.0); m_weldOverlapSpin->setValue(20.0); }
    basicGroup->setMinimumWidth(420);

    auto* startGroup = new QGroupBox("引弧参数", editorRoot);
    auto* startLayout = new QFormLayout(startGroup);
    startLayout->setContentsMargins(14, 18, 14, 12);
    startLayout->setLabelAlignment(Qt::AlignRight);
    startLayout->setHorizontalSpacing(10);
    startLayout->setVerticalSpacing(9);
    startLayout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    m_startArcCurrentSpin = AddSingleDoubleFieldWithUnit(startLayout, "引弧电流", "A");
    m_startArcVoltageSpin = AddSingleDoubleFieldWithUnit(startLayout, "引弧电压", "V");
    m_startWaitTimeSpin = AddSingleDoubleFieldWithUnit(startLayout, "引弧时间", "s");
    startGroup->setMinimumWidth(340);

    auto* stopGroup = new QGroupBox("收弧参数", editorRoot);
    auto* stopLayout = new QFormLayout(stopGroup);
    stopLayout->setContentsMargins(14, 18, 14, 12);
    stopLayout->setLabelAlignment(Qt::AlignRight);
    stopLayout->setHorizontalSpacing(10);
    stopLayout->setVerticalSpacing(9);
    stopLayout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    m_stopArcCurrentSpin = AddSingleDoubleFieldWithUnit(stopLayout, "收弧电流", "A");
    m_stopArcVoltageSpin = AddSingleDoubleFieldWithUnit(stopLayout, "收弧电压", "V");
    m_stopWaitTimeSpin = AddSingleDoubleFieldWithUnit(stopLayout, "收弧时间", "s");
    stopGroup->setMinimumWidth(340);

    auto* weldGroup = new QGroupBox("焊接与运动参数", editorRoot);
    auto* weldLayout = CreateTwoColumnForm(weldGroup);
    m_trackCurrentSpin = AddDoubleFieldWithUnit(weldLayout, "焊接电流", "A");
    m_trackVoltageSpin = AddDoubleFieldWithUnit(weldLayout, "焊接电压", "V");
    m_weldVelocitySpin = AddDoubleFieldWithUnit(weldLayout, "焊接速度", "mm/min");
    m_crosswiseOffsetSpin = AddDoubleFieldWithUnit(weldLayout, "横向补偿", "mm");
    m_verticalOffsetSpin = AddDoubleFieldWithUnit(weldLayout, "竖向补偿", "mm");
    m_weldAngleSpin = AddDoubleFieldWithUnit(weldLayout, "焊接角度", "deg");
    m_weldDipAngleSpin = AddDoubleFieldWithUnit(weldLayout, "焊接倾角", "deg");
    for (QWidget* readonlyWeldField : { static_cast<QWidget*>(m_crosswiseOffsetSpin),
        static_cast<QWidget*>(m_verticalOffsetSpin),
        static_cast<QWidget*>(m_weldAngleSpin),
        static_cast<QWidget*>(m_weldDipAngleSpin) })
    {
        if (readonlyWeldField != nullptr)
        {
            readonlyWeldField->setEnabled(false);
        }
    }
    weldGroup->setMinimumWidth(720);

    auto* specialGroup = new QFrame(editorRoot);
    specialGroup->setObjectName("inlineTabHost");
    auto* specialLayout = new QVBoxLayout(specialGroup);
    specialLayout->setContentsMargins(0, 0, 0, 0);
    specialLayout->setSpacing(0);

    auto* specialSwitchRow = new QHBoxLayout();
    specialSwitchRow->setContentsMargins(0, 0, 0, 0);
    specialSwitchRow->setSpacing(0);
    m_wrapParamButton = new QPushButton("包角参数", specialGroup);
    m_cornerTransitionParamButton = new QPushButton("拐点过渡参数", specialGroup);
    specialSwitchRow->addWidget(CreateTabConnectorLine(specialGroup, 18), 0, Qt::AlignBottom);
    for (QPushButton* button : { m_wrapParamButton, m_cornerTransitionParamButton })
    {
        button->setObjectName("tabStripButton");
        button->setCheckable(true);
        button->setMinimumHeight(40);
        specialSwitchRow->addWidget(button);
    }
    m_wrapParamButton->setChecked(false);
    m_wrapParamButton->setEnabled(false);
    m_wrapParamButton->setToolTip("暂未开放");
    m_cornerTransitionParamButton->setChecked(true);
    specialSwitchRow->addWidget(CreateTabConnectorLine(specialGroup), 1, Qt::AlignBottom);
    specialLayout->addLayout(specialSwitchRow);

    auto* wrapPage = new QWidget(specialGroup);
    wrapPage->setEnabled(false);
    auto* wrapLayout = CreateTwoColumnForm(wrapPage);
    m_wrapCurrent1Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段1电流", &m_wrapCurrent1EnableCheck, "A");
    m_wrapVoltage1Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段1电压", &m_wrapVoltage1EnableCheck, "V");
    m_wrapWaitTime1Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段1时间", &m_wrapWaitTime1EnableCheck, "s");
    m_wrapCurrent2Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段2电流", &m_wrapCurrent2EnableCheck, "A");
    m_wrapVoltage2Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段2电压", &m_wrapVoltage2EnableCheck, "V");
    m_wrapWaitTime2Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段2时间", &m_wrapWaitTime2EnableCheck, "s");
    m_wrapCurrent3Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段3电流", &m_wrapCurrent3EnableCheck, "A");
    m_wrapVoltage3Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段3电压", &m_wrapVoltage3EnableCheck, "V");
    m_wrapWaitTime3Spin = AddEnabledDoubleFieldWithUnit(wrapLayout, "段3时间", &m_wrapWaitTime3EnableCheck, "s");

    auto* cornerPage = new QWidget(specialGroup);
    auto* cornerLayout = CreateTwoColumnForm(cornerPage);
    m_cornerTransitionScopeCombo = new QComboBox(cornerPage);
    m_cornerTransitionScopeCombo->addItem("圆弧");
    m_cornerTransitionScopeCombo->addItem("过渡");
    m_cornerTransitionScopeCombo->addItem("圆弧+过渡");
    m_cornerTransitionScopeCombo->setMinimumWidth(kSingleFieldWidth);
    m_cornerTransitionScopeCombo->setFixedHeight(kSpinFieldHeight);
    m_cornerTransitionScopeCombo->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    m_cornerTransitionScopeCombo->setCurrentIndex(2);
    AddTwoColumnField(cornerLayout, "作用范围", m_cornerTransitionScopeCombo);
    m_cornerTransitionRadiusSpin = AddEnabledDoubleFieldWithUnit(cornerLayout, "圆弧过渡半径", &m_cornerTransitionRadiusEnableCheck, "mm");
    m_cornerTransitionRadiusSpin->setMinimum(2.0);
    m_cornerTransitionSpeedSpin = AddEnabledDoubleFieldWithUnit(cornerLayout, "过渡速度", &m_cornerTransitionSpeedEnableCheck, "mm/min");
    m_cornerTransitionCurrentSpin = AddEnabledDoubleFieldWithUnit(cornerLayout, "过渡电流", &m_cornerTransitionCurrentEnableCheck, "A");
    m_cornerTransitionVoltageSpin = AddEnabledDoubleFieldWithUnit(cornerLayout, "过渡电压", &m_cornerTransitionVoltageEnableCheck, "V");
    m_finalWeldStepSpin = AddDoubleFieldWithUnit(cornerLayout, "实际焊道点间距", "mm");
    m_finalWeldStepSpin->setRange(2.0, 100.0);
    m_finalWeldStepSpin->setValue(4.0);
    m_weldDirectionCombo = new QComboBox(cornerPage);
    m_weldDirectionCombo->addItem("起点到终点", 1);
    m_weldDirectionCombo->addItem("终点到起点", -1);
    m_weldDirectionCombo->setMinimumWidth(kSingleFieldWidth);
    m_weldDirectionCombo->setFixedHeight(kSpinFieldHeight);
    m_weldDirectionCombo->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    AddTwoColumnField(cornerLayout, "焊接方向", m_weldDirectionCombo);

    m_specialParamStack = new QStackedWidget(specialGroup);
    m_specialParamStack->addWidget(wrapPage);
    m_specialParamStack->addWidget(cornerPage);
    m_specialParamStack->setCurrentIndex(1);
    auto* specialPanel = new QFrame(specialGroup);
    specialPanel->setObjectName("inlineTabPanel");
    auto* specialPanelLayout = new QVBoxLayout(specialPanel);
    specialPanelLayout->setContentsMargins(8, 8, 8, 8);
    specialPanelLayout->setSpacing(0);
    specialPanelLayout->addWidget(m_specialParamStack);
    specialLayout->addWidget(specialPanel);
    specialGroup->setMinimumWidth(720);

    auto addComboField = [this](QGridLayout* layout, const QString& label)
        {
            auto* combo = new QComboBox(this);
            combo->setMinimumWidth(kGridFieldWidth);
            combo->setFixedHeight(kSpinFieldHeight);
            combo->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
            AddTwoColumnField(layout, label, combo);
            return combo;
        };

    auto* weaveGroup = new QGroupBox("摆动参数", editorRoot);
    m_weaveParamGroup = weaveGroup;
    auto* weaveLayout = CreateTwoColumnForm(weaveGroup);
    m_weaveTypeCombo = addComboField(weaveLayout, "摆弧类型");
    m_weaveTypeCombo->addItem("eTCPWeave", 0);
    m_weaveTypeCombo->addItem("eWristWeave", 1);
    m_weaveTypeCombo->addItem("e45JointWeave", 2);
    m_weaveImplCombo = addComboField(weaveLayout, "摆动实现");
    m_weaveImplCombo->addItem("机器人原生", 0);
    m_weaveImplCombo->addItem("上位机自建(pointwise)", 1);
    m_weaveShapeCombo = addComboField(weaveLayout, "摆弧形状");
    m_weaveShapeCombo->addItem("eNoWeave", 0);
    m_weaveShapeCombo->addItem("eSin", 5);
    m_weaveShapeCombo->addItem("eSinFreq", 6);
    m_weaveShapeCombo->addItem("eSpiral", 7);
    m_weaveShapeCombo->addItem("eObliqueTriangle", 8);
    m_weaveShapeCombo->addItem("eSpaceTriangle", 9);
    m_weaveShapeCombo->addItem("eLTriangle", 10);
    m_weaveShapeCombo->addItem("eBackForward", 11);
    m_weaveShapeCombo->addItem("eConstPoint", 12);
    m_weaveShapeCombo->addItem("eSpiralFreq", 13);
    m_weaveShapeCombo->addItem("eObliqueTriangleFreq", 14);
    m_weaveShapeCombo->addItem("eSpaceTriangleFreq", 15);
    m_weaveShapeCombo->addItem("eLTriangleFreq", 16);
    m_weaveShapeCombo->addItem("eBackForwordFreq", 17);
    m_weaveShapeCombo->addItem("eHalfSin", 18);
    connect(m_weaveImplCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) { UpdateWeaveShapeAvailability(); });
    UpdateWeaveShapeAvailability();  // 初始化：按当前摆动实现设置摆弧形状可用性
    m_weaveFrequencySpin = AddDoubleFieldWithUnit(weaveLayout, "摆弧频率", "hz");
    m_weaveAmplitudeSpin = AddDoubleFieldWithUnit(weaveLayout, "摆动幅值", "mm");
    m_swingDirectionSpin = AddDoubleFieldWithUnit(weaveLayout, "摆弧倾斜角", "°");
    m_weavePointsCombo = addComboField(weaveLayout, "每周期点数");
    for (int pts : {4, 8, 12, 16, 24, 32}) { m_weavePointsCombo->addItem(QString::number(pts), pts); }  // 4的倍数(保证停留落相位)，仅 pointwise 用
    m_weavePlaneAngleSpin = AddDoubleFieldWithUnit(weaveLayout, "摆弧平面倾斜角", "°");
    m_spaceAngleSpin = AddDoubleFieldWithUnit(weaveLayout, "空间摆弧夹角", "°");
    m_pauseTime1Spin = AddIntFieldWithUnit(weaveLayout, "1/4处停留时间", "ms", 0, 999999);
    m_pauseTime2Spin = AddIntFieldWithUnit(weaveLayout, "2/4处停留时间", "ms", 0, 999999);
    m_pauseTime3Spin = AddIntFieldWithUnit(weaveLayout, "3/4处停留时间", "ms", 0, 999999);
    m_pauseTime4Spin = AddIntFieldWithUnit(weaveLayout, "4/4处停留时间", "ms", 0, 999999);
    m_pauseContinueCombo = addComboField(weaveLayout, "摆弧停留连续");
    m_pauseContinueCombo->addItem("FALSE", 0);
    m_pauseContinueCombo->addItem("TRUE", 1);
    m_endLengthSpin = AddDoubleFieldWithUnit(weaveLayout, "摆弧结束长度", "mm");
    m_endWidthSpin = AddDoubleFieldWithUnit(weaveLayout, "摆弧结束宽度", "mm");
    m_centerHeightSpin = AddDoubleFieldWithUnit(weaveLayout, "摆弧中心高度", "mm");
    // 物理量不允许负值：摆幅/频率/结束长度/结束宽度下限设 0；摆弧倾斜角等角度类保留可正可负(±倒向)
    m_weaveFrequencySpin->setMinimum(0.0);
    m_weaveAmplitudeSpin->setMinimum(0.0);
    m_endLengthSpin->setMinimum(0.0);
    m_endWidthSpin->setMinimum(0.0);
    weaveGroup->setMinimumWidth(720);

    auto* trackGroup = new QGroupBox("跟踪参数", editorRoot);
    m_trackParamGroup = trackGroup;
    auto* trackLayout = CreateTwoColumnForm(trackGroup);
    m_lateralBeginCycleSpin = AddIntField(trackLayout, "横向纠偏开始周期", 0, 999999);
    m_lateralGainSpin = AddDoubleField(trackLayout, "横向比例增益", 3);
    m_leftAreaCoefficientSpin = AddDoubleField(trackLayout, "横向左边面积系数", 3);
    m_rightAreaCoefficientSpin = AddDoubleField(trackLayout, "横向右边面积系数", 3);
    m_verticalModeCombo = addComboField(trackLayout, "纵向模式标志位");
    m_verticalModeCombo->addItem("eConst", 0);
    m_verticalModeCombo->addItem("eSample", 1);
    m_verticalReferenceCurrentSpin = AddDoubleFieldWithUnit(trackLayout, "纵向纠偏基准电流", "A", 3);
    m_verticalBeginCycleSpin = AddIntField(trackLayout, "纵向取样开始周期", 1, 999999);
    m_verticalSustainCycleSpin = AddIntField(trackLayout, "纵向取样持续周期", 1, 999999);
    m_verticalCycleLengthSpin = AddDoubleFieldWithUnit(trackLayout, "纵向周期长度", "mm", 3);
    m_verticalGainSpin = AddDoubleField(trackLayout, "纵向比例增益", 3);
    m_trackIntervalModeCombo = addComboField(trackLayout, "纠偏间隔类型");
    m_trackIntervalModeCombo->addItem("eDistance", 0);
    m_trackIntervalModeCombo->addItem("eTime", 1);
    m_timeIntervalSpin = AddIntFieldWithUnit(trackLayout, "时间间隔", "ms", 100, 500);
    m_distanceIntervalSpin = AddIntFieldWithUnit(trackLayout, "距离间隔", "mm", 1, 30);
    m_lateralMinCompSpin = AddDoubleFieldWithUnit(trackLayout, "横向最小补偿（单周期）", "mm", 3);
    m_lateralMaxCompSpin = AddDoubleFieldWithUnit(trackLayout, "横向最大补偿（单周期）", "mm", 3);
    m_lateralTotalMaxCompSpin = AddDoubleFieldWithUnit(trackLayout, "横向最大补偿（总）", "mm", 3);
    m_lateralAsymmetrySpin = AddDoubleField(trackLayout, "横向不对称调整系数", 3);
    m_verticalMinCompSpin = AddDoubleFieldWithUnit(trackLayout, "纵向最小补偿（单周期）", "mm", 3);
    m_verticalMaxCompSpin = AddDoubleFieldWithUnit(trackLayout, "纵向最大补偿（单周期）", "mm", 3);
    m_verticalTotalMaxCompSpin = AddDoubleFieldWithUnit(trackLayout, "纵向最大补偿（总）", "mm", 3);
    m_verticalAsymmetrySpin = AddDoubleField(trackLayout, "纵向不对称调整系数", 3);
    trackGroup->setMinimumWidth(900);

    auto* topRow = new QHBoxLayout();
    topRow->setSpacing(10);
    topRow->addWidget(basicGroup, 4);
    topRow->addWidget(startGroup, 3);
    topRow->addWidget(stopGroup, 3);

    auto* bottomRow = new QHBoxLayout();
    bottomRow->setSpacing(10);
    bottomRow->addWidget(weldGroup, 1);
    bottomRow->addWidget(specialGroup, 1);

    auto* normalPage = new QWidget(editorRoot);
    auto* normalPageOuterLayout = new QVBoxLayout(normalPage);
    normalPageOuterLayout->setContentsMargins(0, 0, 0, 0);
    normalPageOuterLayout->setSpacing(0);
    auto* normalPanel = new QFrame(normalPage);
    normalPanel->setObjectName("editorPagePanel");
    auto* normalLayout = new QVBoxLayout(normalPanel);
    normalLayout->setContentsMargins(10, 10, 10, 10);
    normalLayout->setSpacing(10);
    normalLayout->addLayout(topRow);
    normalLayout->addLayout(bottomRow);
    normalLayout->addStretch();
    normalPageOuterLayout->addWidget(normalPanel, 1);
    LockMinimumContentSize(basicGroup);
    LockMinimumContentSize(startGroup);
    LockMinimumContentSize(stopGroup);
    LockMinimumContentSize(weldGroup);
    LockMinimumContentSize(specialGroup);
    LockMinimumContentSize(normalPanel);
    LockMinimumContentSize(normalPage);

    auto* weavePage = new QWidget(editorRoot);
    auto* weavePageOuterLayout = new QVBoxLayout(weavePage);
    weavePageOuterLayout->setContentsMargins(0, 0, 0, 0);
    weavePageOuterLayout->setSpacing(0);
    auto* weavePanel = new QFrame(weavePage);
    weavePanel->setObjectName("editorPagePanel");
    auto* weavePageLayout = new QHBoxLayout(weavePanel);
    weavePageLayout->setContentsMargins(16, 16, 16, 16);
    weavePageLayout->setSpacing(12);
    weavePageLayout->addWidget(weaveGroup);
    weavePageLayout->addStretch();
    weavePageOuterLayout->addWidget(weavePanel, 1);
    LockMinimumContentSize(weaveGroup);
    LockMinimumContentSize(weavePanel);
    LockMinimumContentSize(weavePage);

    auto* trackPage = new QWidget(editorRoot);
    auto* trackPageOuterLayout = new QVBoxLayout(trackPage);
    trackPageOuterLayout->setContentsMargins(0, 0, 0, 0);
    trackPageOuterLayout->setSpacing(0);
    auto* trackPanel = new QFrame(trackPage);
    trackPanel->setObjectName("editorPagePanel");
    auto* trackPageLayout = new QHBoxLayout(trackPanel);
    trackPageLayout->setContentsMargins(16, 16, 16, 16);
    trackPageLayout->setSpacing(12);
    trackPageLayout->addWidget(trackGroup);
    trackPageLayout->addStretch();
    trackPageOuterLayout->addWidget(trackPanel, 1);
    LockMinimumContentSize(trackGroup);
    LockMinimumContentSize(trackPanel);
    LockMinimumContentSize(trackPage);

    m_editorStack = new QStackedWidget(editorRoot);
    m_editorStack->addWidget(normalPage);
    m_editorStack->addWidget(weavePage);
    m_editorStack->addWidget(trackPage);
    m_editorStack->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    LockMinimumContentSize(m_editorStack);

    auto* editorScrollArea = new QScrollArea(editorFrame);
    editorScrollArea->setObjectName("weldProcessEditorScroll");
    editorScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    editorScrollArea->setWidget(m_editorStack);
    ConfigureResponsiveScrollArea(editorScrollArea);
    editorFrameLayout->addWidget(editorScrollArea, 1);
    tabHostLayout->addWidget(editorFrame, 1);
    editorLayout->addWidget(tabHostContainer, 1);

    splitterLayout->addWidget(editorRoot, 3);

    connect(m_normalPageButton, &QPushButton::clicked, this, [this]()
        {
            m_normalPageButton->setChecked(true);
            m_weavePageButton->setChecked(false);
            m_trackPageButton->setChecked(false);
            m_editorStack->setCurrentIndex(0);
        });
    connect(m_weavePageButton, &QPushButton::clicked, this, [this]()
        {
            m_normalPageButton->setChecked(false);
            m_weavePageButton->setChecked(true);
            m_trackPageButton->setChecked(false);
            m_editorStack->setCurrentIndex(1);
        });
    connect(m_trackPageButton, &QPushButton::clicked, this, [this]()
        {
            m_normalPageButton->setChecked(false);
            m_weavePageButton->setChecked(false);
            m_trackPageButton->setChecked(true);
            m_editorStack->setCurrentIndex(2);
        });
    connect(m_wrapParamButton, &QPushButton::clicked, this, [this]()
        {
            m_wrapParamButton->setChecked(true);
            m_cornerTransitionParamButton->setChecked(false);
            m_specialParamStack->setCurrentIndex(0);
        });
    connect(m_cornerTransitionParamButton, &QPushButton::clicked, this, [this]()
        {
            m_wrapParamButton->setChecked(false);
            m_cornerTransitionParamButton->setChecked(true);
            m_specialParamStack->setCurrentIndex(1);
        });
    connect(m_addBeadButton, &QPushButton::clicked, this, &WeldProcessDialog::AddBead);
    connect(m_removeBeadButton, &QPushButton::clicked, this, &WeldProcessDialog::RemoveBead);
    connect(m_addWeldButton, &QPushButton::clicked, this, &WeldProcessDialog::AddWeldGroup);
    connect(m_removeWeldButton, &QPushButton::clicked, this, &WeldProcessDialog::RemoveWeldGroup);
    connect(m_weaveEnableCheck, &QCheckBox::toggled, this, &WeldProcessDialog::UpdateFeaturePageEnabled);
    connect(m_trackEnableCheck, &QCheckBox::toggled, this, &WeldProcessDialog::UpdateFeaturePageEnabled);
    connect(m_beadListWidget, &QListWidget::currentRowChanged, this, &WeldProcessDialog::OnBeadSelectionChanged);
    connect(m_weldListWidget->model(), &QAbstractItemModel::rowsMoved, this, [this]()
        {
            if (!m_isLoading)
            {
                OnWeldGroupsReordered();
            }
        });
    UpdateFeaturePageEnabled();
}

void WeldProcessDialog::ApplyDialogStyle()
{
    setStyleSheet(
        "QDialog{background:#111820;color:#ECF3F4;}"
        "QFrame#leftPanel,QFrame#beadPanel,QGroupBox,QFrame#editorPagePanel,QFrame#tabHostPanel,QFrame#inlineTabPanel{background:#111820;border:1px solid #2E4656;border-radius:14px;}"
        "QFrame#inlineTabHost{background:transparent;border:none;}"
        "QWidget#topPanel{background:#111820;border:none;}"
        "QStackedWidget{background:transparent;}"
        "QFrame#tabHostPanel,QFrame#inlineTabPanel{padding-top:0px;border-radius:0px;margin-top:0px;border-top:0px;}"
        "QFrame#tabConnectorLine{background:#2E4656;border:none;min-height:1px;max-height:1px;}"
        "QFrame#editorPagePanel{border:none;background:transparent;border-radius:0px;}"
        "QGroupBox{margin-top:0px;padding:30px 14px 14px 14px;font-weight:600;}"
        "QGroupBox::title{subcontrol-origin:padding;subcontrol-position:top left;left:12px;top:7px;padding:0;color:#9ED8DB;background:transparent;}"
        "QLabel#titleLabel{font-size:22px;font-weight:700;color:#F7FCFC;letter-spacing:1px;}"
        "QLabel#descLabel,QLabel#statusLabel{color:#BACBD1;}"
        "QLabel#sectionTitle{font-size:16px;font-weight:600;color:#9ED8DB;}"
        "QLabel{color:#BACBD1;}"
        "QLineEdit,QListWidget{background:#0B1117;color:#F5FAFA;border:1px solid #385366;border-radius:8px;padding:4px 8px;selection-background-color:#2D5465;selection-color:#F5FAFA;}"
        "QComboBox{background:#000000;color:#F5FAFA;border:1px solid #385366;border-radius:0px;padding:0px 8px;selection-background-color:#2D5465;selection-color:#F5FAFA;min-height:26px;}"
        "QComboBox::drop-down{subcontrol-origin:border;subcontrol-position:top right;width:24px;border-left:1px solid #385366;background:#000000;}"
        "QComboBox::down-arrow{image:url(:/QtWidgetsApplication4/icons/chevron-down.svg);width:12px;height:8px;}"
        "QComboBox QAbstractItemView{background:#0B1117;color:#F5FAFA;border:1px solid #385366;selection-background-color:#2D5465;selection-color:#F5FAFA;}"
        "QFrame#spinFieldFrame{background:#000000;border:1px solid #385366;border-radius:0px;}"
        "QSpinBox,QDoubleSpinBox{background:transparent;color:#F5FAFA;border:none;border-radius:0px;padding:0px 8px;selection-background-color:#2D5465;selection-color:#F5FAFA;min-height:26px;}"
        "QSpinBox QLineEdit,QDoubleSpinBox QLineEdit{background:transparent;border:none;border-radius:0px;padding:0px;color:#F5FAFA;}"
        "QLabel#fieldUnitLabel{color:#9ED8DB;}"
        "QSpinBox::up-button,QDoubleSpinBox::up-button{"
            "subcontrol-origin:border;"
            "subcontrol-position:top right;"
            "width:24px;"
            "border-left:1px solid #385366;"
            "border-bottom:1px solid #385366;"
            "background:#000000;"
            "border-radius:0px;"
        "}"
        "QSpinBox::down-button,QDoubleSpinBox::down-button{"
            "subcontrol-origin:border;"
            "subcontrol-position:bottom right;"
            "width:24px;"
            "border-left:1px solid #385366;"
            "background:#000000;"
            "border-radius:0px;"
        "}"
        "QSpinBox::up-arrow,QDoubleSpinBox::up-arrow{image:url(:/QtWidgetsApplication4/icons/chevron-up.svg);width:10px;height:7px;}"
        "QSpinBox::down-arrow,QDoubleSpinBox::down-arrow{image:url(:/QtWidgetsApplication4/icons/chevron-down.svg);width:10px;height:7px;}"
        "QSpinBox::up-button:hover,QDoubleSpinBox::up-button:hover,QSpinBox::down-button:hover,QDoubleSpinBox::down-button:hover{background:#101820;}"
        "QCheckBox{color:#BACBD1;spacing:5px;}"
        "QCheckBox::indicator{width:16px;height:16px;border:1px solid #3C6173;background:#0B1117;border-radius:3px;}"
        "QCheckBox::indicator:checked{background:#2D8F7D;border-color:#7BD8B3;}"
        "QCheckBox::indicator:hover{border-color:#72D4DD;}"
        "QCheckBox#switchCheckBox{spacing:8px;font-weight:600;color:#9ED8DB;}"
        "QCheckBox#switchCheckBox::indicator{width:42px;height:22px;border-radius:11px;background:#263742;border:1px solid #3C6173;}"
        "QCheckBox#switchCheckBox::indicator:checked{background:#2D8F7D;border-color:#7BD8B3;}"
        "QCheckBox#switchCheckBox::indicator:disabled{background:#080C10;border-color:#253947;}"
        "QPushButton{background:#233645;color:#F5FAFA;border:1px solid #3C6173;border-radius:10px;padding:7px 14px;}"
        "QPushButton:hover{background:#2D5465;border-color:#72D4DD;}"
        "QPushButton:pressed{background:#18303B;}"
        "QPushButton#saveBtn{background:#2D5465;border-color:#72D4DD;color:#FFFFFF;font-weight:600;}"
        "QPushButton#saveBtn:hover{background:#347084;}"
        "QPushButton:checked{background:#305F55;border-color:#7BD8B3;color:#F5FAFA;}"
        "QPushButton#tabStripButton{"
            "background:#172530;"
            "color:#BACBD1;"
            "border:1px solid #2E4656;"
            "border-top-left-radius:0px;"
            "border-top-right-radius:0px;"
            "border-bottom-left-radius:0px;"
            "border-bottom-right-radius:0px;"
            "border-bottom:0px;"
            "padding:8px 24px;"
            "margin-right:-1px;"
            "min-width:152px;"
            "font-weight:600;"
        "}"
        "QPushButton#tabStripButton:hover{background:#233645;color:#F5FAFA;border-color:#72D4DD;}"
        "QPushButton#tabStripButton:checked{"
            "background:#233645;"
            "color:#9ED8DB;"
            "border-color:#72D4DD;"
            "border-bottom:0px;"
        "}"
        "QPushButton#tabStripButton:disabled{background:#101820;color:#61727A;border-color:#263742;}"
        "QFrame#spinFieldFrame:disabled{background:#080C10;border-color:#253947;}"
        "QSpinBox:disabled,QDoubleSpinBox:disabled,QLineEdit:disabled{color:#61727A;background:#080C10;border-color:#253947;}"
        "QComboBox:disabled{color:#61727A;background:#080C10;border-color:#253947;}"
        "QComboBox::drop-down:disabled{background:#080C10;border-color:#253947;}"
        "QComboBox QAbstractItemView::item:disabled{color:#55656E;background:#0B1117;}"
        "QCheckBox:disabled{color:#61727A;}"
        "QCheckBox::indicator:disabled{background:#080C10;border-color:#253947;}"
        "QPushButton#roundActionButton{border-radius:12px;padding:0;font-size:16px;font-weight:700;min-width:24px;max-width:24px;min-height:24px;max-height:24px;}"
        "QListWidget::item{padding:8px 10px;border-bottom:1px solid #1E303B;border-radius:6px;margin:1px;}"
        "QListWidget::item:hover{background:#172530;}"
        "QListWidget::item:selected{background:#2D5465;color:#FFFFFF;border:1px solid #72D4DD;}"
        "QListWidget#beadListWidget::item{padding:8px 6px;color:#9ED8DB;text-align:center;}"
        "QListWidget#beadListWidget::item:selected{background:#233645;color:#FFFFFF;border:1px solid #72D4DD;}"
        "QScrollBar:vertical{background:#111820;width:12px;margin:0;}"
        "QScrollBar::handle:vertical{background:#385366;border-radius:6px;min-height:24px;}"
        "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
    );
}

QLineEdit* WeldProcessDialog::AddTextField(QGridLayout* layout, const QString& label)
{
    auto* edit = new QLineEdit(this);
    edit->setMinimumWidth(kGridFieldWidth);
    edit->setFixedHeight(28);
    edit->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    AddTwoColumnField(layout, label, edit);
    return edit;
}

QDoubleSpinBox* WeldProcessDialog::AddDoubleField(QGridLayout* layout, const QString& label, int decimals)
{
    return AddDoubleFieldWithUnit(layout, label, QString(), decimals);
}

QDoubleSpinBox* WeldProcessDialog::AddDoubleFieldWithUnit(QGridLayout* layout, const QString& label, const QString& unit, int decimals)
{
    auto* spin = new PlusMinusDoubleSpinBox(this);
    spin->setDecimals(decimals);
    spin->setRange(kDoubleMin, kDoubleMax);
    AddTwoColumnField(layout, label, WrapFieldWithUnit(this, WrapSpinFieldFrame(this, spin, kGridFieldWidth), unit));
    return spin;
}

QDoubleSpinBox* WeldProcessDialog::AddEnabledDoubleField(QGridLayout* layout, const QString& label, QCheckBox** enableCheck, int decimals)
{
    return AddEnabledDoubleFieldWithUnit(layout, label, enableCheck, QString(), decimals);
}

QDoubleSpinBox* WeldProcessDialog::AddEnabledDoubleFieldWithUnit(QGridLayout* layout, const QString& label, QCheckBox** enableCheck, const QString& unit, int decimals)
{
    auto* spin = new PlusMinusDoubleSpinBox(this);
    spin->setDecimals(decimals);
    spin->setRange(kDoubleMin, kDoubleMax);
    auto* spinFrame = WrapSpinFieldFrame(this, spin, kGridFieldWidth);

    auto* check = new QCheckBox("启用", this);
    check->setObjectName("fieldEnableCheck");
    check->setFixedWidth(50);
    spin->setEnabled(false);
    connect(check, &QCheckBox::toggled, spin, &QWidget::setEnabled);

    auto* fieldHost = new QWidget(this);
    auto* fieldLayout = new QHBoxLayout(fieldHost);
    fieldLayout->setContentsMargins(0, 0, 0, 0);
    fieldLayout->setSpacing(5);
    fieldLayout->addWidget(check);
    fieldLayout->addWidget(spinFrame, 1);
    if (!unit.trimmed().isEmpty())
    {
        fieldLayout->addWidget(CreateUnitLabel(fieldHost, unit));
    }
    fieldLayout->addStretch(1);
    fieldHost->setMinimumWidth(50 + kGridFieldWidth + UnitLabelWidth(unit) + 10);
    fieldHost->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);

    if (enableCheck != nullptr)
    {
        *enableCheck = check;
    }
    AddTwoColumnField(layout, label, fieldHost);
    return spin;
}

QSpinBox* WeldProcessDialog::AddIntField(QGridLayout* layout, const QString& label, int minimum, int maximum)
{
    return AddIntFieldWithUnit(layout, label, QString(), minimum, maximum);
}

QSpinBox* WeldProcessDialog::AddIntFieldWithUnit(QGridLayout* layout, const QString& label, const QString& unit, int minimum, int maximum)
{
    auto* spin = new PlusMinusSpinBox(this);
    spin->setRange(minimum, maximum);
    AddTwoColumnField(layout, label, WrapFieldWithUnit(this, WrapSpinFieldFrame(this, spin, kGridFieldWidth), unit));
    return spin;
}

QLineEdit* WeldProcessDialog::AddSingleTextField(QFormLayout* layout, const QString& label)
{
    auto* edit = new QLineEdit(this);
    edit->setMinimumWidth(kSingleFieldWidth);
    edit->setFixedHeight(28);
    edit->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    layout->addRow(label, edit);
    return edit;
}

QDoubleSpinBox* WeldProcessDialog::AddSingleDoubleField(QFormLayout* layout, const QString& label, int decimals)
{
    return AddSingleDoubleFieldWithUnit(layout, label, QString(), decimals);
}

QDoubleSpinBox* WeldProcessDialog::AddSingleDoubleFieldWithUnit(QFormLayout* layout, const QString& label, const QString& unit, int decimals)
{
    auto* spin = new PlusMinusDoubleSpinBox(this);
    spin->setDecimals(decimals);
    spin->setRange(kDoubleMin, kDoubleMax);
    layout->addRow(label, WrapFieldWithUnit(this, WrapSpinFieldFrame(this, spin, kSingleFieldWidth), unit));
    return spin;
}

QSpinBox* WeldProcessDialog::AddSingleIntField(QFormLayout* layout, const QString& label, int minimum, int maximum)
{
    return AddSingleIntFieldWithUnit(layout, label, QString(), minimum, maximum);
}

QSpinBox* WeldProcessDialog::AddSingleIntFieldWithUnit(QFormLayout* layout, const QString& label, const QString& unit, int minimum, int maximum)
{
    auto* spin = new PlusMinusSpinBox(this);
    spin->setRange(minimum, maximum);
    layout->addRow(label, WrapFieldWithUnit(this, WrapSpinFieldFrame(this, spin, kSingleFieldWidth), unit));
    return spin;
}

QComboBox* WeldProcessDialog::AddSingleComboField(QFormLayout* layout, const QString& label)
{
    auto* combo = new QComboBox(this);
    combo->setMinimumWidth(kSingleFieldWidth);
    combo->setFixedHeight(kSpinFieldHeight);
    combo->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    layout->addRow(label, combo);
    return combo;
}

QCheckBox* WeldProcessDialog::AddSingleSwitchField(QFormLayout* layout, const QString& label)
{
    auto* check = new QCheckBox("启用", this);
    check->setObjectName("switchCheckBox");
    check->setChecked(true);
    check->setFixedHeight(kSpinFieldHeight);
    check->setMinimumWidth(kSingleFieldWidth);
    check->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
    layout->addRow(label, check);
    return check;
}

void WeldProcessDialog::UpdateFeaturePageEnabled()
{
    if (m_weaveParamGroup != nullptr)
    {
        m_weaveParamGroup->setEnabled(IsChecked(m_weaveEnableCheck));
    }
    if (m_trackParamGroup != nullptr)
    {
        m_trackParamGroup->setEnabled(IsChecked(m_trackEnableCheck));
    }
}

void WeldProcessDialog::OnRobotChanged(int index)
{
    if (m_pContralUnit == nullptr || index < 0
        || index >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return;
    }
    const T_CONTRAL_UNIT& unit = m_pContralUnit->m_vtContralUnitInfo[index];
    if (!m_file.LoadFromControlUnit(unit))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return;
    }
    m_unitName = QString::fromUtf8(unit.sUnitName.c_str());
    LoadToUi();  // 内部 m_file.Init() 用新机器人路径重载 + 刷新工艺列表/参数
}

void WeldProcessDialog::UpdateWeaveShapeAvailability()
{
    if (m_weaveImplCombo == nullptr || m_weaveShapeCombo == nullptr) { return; }
    // 摆动实现=上位机自建(pointwise) 时，pointwise 仅支持 eSin(5)正弦 / eObliqueTriangle(8)三角 / eBackForward(11)纵向往复；
    // eLTriangle(10)L摆为占位先置灰，其余原生波形 pointwise 不支持也置灰。机器人原生模式则全部可选。
    const bool pointwise = (ComboData(m_weaveImplCombo, 0) != 0);
    if (auto* model = qobject_cast<QStandardItemModel*>(m_weaveShapeCombo->model()))
    {
        for (int i = 0; i < m_weaveShapeCombo->count(); ++i)
        {
            const int shapeVal = m_weaveShapeCombo->itemData(i).toInt();
            const bool enabled = !pointwise || (shapeVal == 5 || shapeVal == 8 || shapeVal == 11);
            if (auto* item = model->item(i))
            {
                item->setEnabled(enabled);
                // 深色主题下 disabled 项默认色与底色糊在一起，显式给禁用项明显暗灰、启用项亮白
                item->setForeground(QBrush(QColor(enabled ? "#F5FAFA" : "#55656E")));
            }
        }
    }

    // pointwise 只用 摆弧形状/频率/幅值/摆弧倾斜角；其余摆动参数 pointwise 不读，整控件置灰(机器人原生模式恢复)
    // pointwise 现在也用停留时间(nPauseTime1-4，srp WaitTime 实现)，故 1/4~4/4 停留时间不再置灰；
    // 停留连续(nPauseContinue：0完全停/1移动模式)仍置灰——pointwise 的 sleep 只能完全停，给不了移动模式。
    QWidget* const nativeOnlyWidgets[] = {
        m_weaveTypeCombo, m_weavePlaneAngleSpin, m_spaceAngleSpin,
        m_pauseContinueCombo, m_endLengthSpin, m_endWidthSpin, m_centerHeightSpin
    };
    for (QWidget* w : nativeOnlyWidgets)
    {
        if (w == nullptr) { continue; }
        w->setEnabled(!pointwise);
        // spinbox 外面包了 QFrame#spinFieldFrame，连父框一起 disable 背景才整体变暗
        if (QWidget* parent = w->parentWidget())
        {
            if (parent->objectName() == "spinFieldFrame") { parent->setEnabled(!pointwise); }
        }
    }
    // 每周期点数只 pointwise 用：pointwise 时启用、机器人原生时置灰(与上面 nativeOnly 相反)
    if (m_weavePointsCombo != nullptr) { m_weavePointsCombo->setEnabled(pointwise); }
}

void WeldProcessDialog::LoadToUi(int preferredGroupRow, int preferredBeadRow)
{
    m_isLoading = true;
    if (!m_file.Init())
    {
        const QString err = DecodeRobotMessageText(m_file.GetLastError());
        if (m_weldListWidget != nullptr)
        {
            m_weldListWidget->clear();
        }
        if (m_beadListWidget != nullptr)
        {
            m_beadListWidget->clear();
        }
        ui->statusLabel->setText("读取失败: " + err + " 请点击“+”重新创建工艺内容。");
        m_isLoading = false;
        MarkCleanSnapshot();
        return;
    }

    PopulateWeldList(preferredGroupRow);
    PopulateBeadList(preferredBeadRow);
    ui->statusLabel->setText(QStringLiteral("读取完成: 已从配置数据库读取摆动参数和焊接工艺。"));
    m_isLoading = false;
    MarkCleanSnapshot();
}

void WeldProcessDialog::ReloadData()
{
    LoadToUi(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1,
        m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : -1);
}

void WeldProcessDialog::PopulateWeldList(int preferredGroupRow)
{
    m_weldListWidget->clear();
    const auto& list = m_file.GetWeldParaList();
    QStringList seenKeys;
    int selectedGroupRow = 0;
    QString selectedKey;
    if (const auto* selected = m_file.GetUseWeldPara())
    {
        selectedKey = BuildGroupKey(*selected);
    }

    for (int i = 0; i < static_cast<int>(list.size()); ++i)
    {
        const QString key = BuildGroupKey(list[i]);
        if (seenKeys.contains(key))
        {
            continue;
        }
        seenKeys.append(key);
        auto* itemWidget = new QListWidgetItem(BuildWeldDisplayName(m_weldListWidget->count(), list[i]));
        itemWidget->setData(Qt::UserRole, key);
        m_weldListWidget->addItem(itemWidget);
        if (key == selectedKey)
        {
            selectedGroupRow = m_weldListWidget->count() - 1;
        }
    }

    int rowToSelect = preferredGroupRow;
    if (rowToSelect < 0 || rowToSelect >= m_weldListWidget->count())
    {
        rowToSelect = qBound(0, selectedGroupRow, qMax(0, m_weldListWidget->count() - 1));
    }

    if (m_weldListWidget->count() > 0)
    {
        m_weldListWidget->setCurrentRow(rowToSelect);
    }
}

QString WeldProcessDialog::BuildWeldDisplayName(int index, const T_WELD_PARA& item) const
{
    return QString("%1. %2 | 焊脚%3")
        .arg(index + 1)
        .arg(DecodeRobotMessageText(item.strWorkPeace))
        .arg(item.dWeldAngleSize, 0, 'f', 1);
}

int WeldProcessDialog::GroupCount() const
{
    QStringList seenKeys;
    const auto& list = m_file.GetWeldParaList();
    for (const auto& item : list)
    {
        const QString key = BuildGroupKey(item);
        if (!seenKeys.contains(key))
        {
            seenKeys.append(key);
        }
    }
    return seenKeys.size();
}

QString WeldProcessDialog::BuildGroupKey(const T_WELD_PARA& item) const
{
    return QString("%1|%2")
        .arg(DecodeRobotMessageText(item.strWorkPeace))
        .arg(item.dWeldAngleSize, 0, 'f', 3);
}

std::vector<int> WeldProcessDialog::GetGroupIndices(int groupRow) const
{
    std::vector<int> indices;
    if (groupRow < 0)
    {
        return indices;
    }

    QStringList seenKeys;
    const auto& list = m_file.GetWeldParaList();
    QString targetKey;
    for (int i = 0; i < static_cast<int>(list.size()); ++i)
    {
        const QString key = BuildGroupKey(list[i]);
        if (!seenKeys.contains(key))
        {
            if (seenKeys.size() == groupRow)
            {
                targetKey = key;
                break;
            }
            seenKeys.append(key);
        }
    }

    if (targetKey.isEmpty())
    {
        return indices;
    }

    for (int i = 0; i < static_cast<int>(list.size()); ++i)
    {
        if (BuildGroupKey(list[i]) == targetKey)
        {
            indices.push_back(i);
        }
    }
    return indices;
}

void WeldProcessDialog::PopulateBeadList(int preferredBeadRow)
{
    if (m_beadListWidget == nullptr)
    {
        return;
    }

    m_beadListWidget->clear();
    const auto indices = GetGroupIndices(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1);
    int selectedBeadRow = 0;
    const int currentWeldIndex = m_file.GetUseWeldParaNo();

    for (int i = 0; i < static_cast<int>(indices.size()); ++i)
    {
        const auto& item = m_file.GetWeldParaList()[indices[i]];
        m_beadListWidget->addItem(QString("焊道%1").arg(item.nLayerNo));
        if (indices[i] == currentWeldIndex)
        {
            selectedBeadRow = i;
        }
    }

    int rowToSelect = preferredBeadRow;
    if (rowToSelect < 0 || rowToSelect >= m_beadListWidget->count())
    {
        rowToSelect = qBound(0, selectedBeadRow, qMax(0, m_beadListWidget->count() - 1));
    }

    if (m_beadListWidget->count() > 0)
    {
        m_beadListWidget->setCurrentRow(rowToSelect);
        ApplySelectionToUi(indices[rowToSelect]);
    }
}

int WeldProcessDialog::CurrentWeldIndex() const
{
    const auto indices = GetGroupIndices(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1);
    const int beadRow = m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : -1;
    if (beadRow < 0 || beadRow >= static_cast<int>(indices.size()))
    {
        return -1;
    }
    return indices[beadRow];
}

void WeldProcessDialog::ApplySelectionToUi(int row)
{
    const auto& weldList = m_file.GetWeldParaList();
    if (row < 0 || row >= static_cast<int>(weldList.size()))
    {
        return;
    }

    m_isLoading = true;
    const auto& weld = weldList[row];

    m_workPeaceEdit->setText(DecodeRobotMessageText(weld.strWorkPeace));
    m_weldTypeEdit->setText(DecodeRobotMessageText(weld.strWeldType));
    SetComboByData(m_arcModeCombo, weld.nArcMode);
    m_weldAngleSizeSpin->setValue(weld.dWeldAngleSize);
    SetChecked(m_weaveEnableCheck, weld.nWeaveEnable);
    SetComboByData(m_weaveImplCombo, weld.nWrapConditionNo);  // 摆动实现回填(0=原生 1=pointwise)
    SetComboByData(m_weavePointsCombo, weld.nStandWeldDir >= 4 ? weld.nStandWeldDir : 16);  // 每周期点数回填(旧工艺<4回退16)
    SetChecked(m_trackEnableCheck, weld.nTrackEnable);

    m_startArcCurrentSpin->setValue(weld.dStartArcCurrent);
    m_startArcVoltageSpin->setValue(weld.dStartArcVoltage);
    m_startWaitTimeSpin->setValue(weld.dStartWaitTime);

    m_trackCurrentSpin->setValue(weld.dTrackCurrent);
    m_trackVoltageSpin->setValue(weld.dTrackVoltage);
    m_weldVelocitySpin->setValue(weld.WeldVelocity);
    m_crosswiseOffsetSpin->setValue(weld.CrosswiseOffset);
    m_verticalOffsetSpin->setValue(weld.verticalOffset);
    m_weldAngleSpin->setValue(weld.dWeldAngle);
    m_weldDipAngleSpin->setValue(weld.dWeldDipAngle);

    m_stopArcCurrentSpin->setValue(weld.dStopArcCurrent);
    m_stopArcVoltageSpin->setValue(weld.dStopArcVoltage);
    m_stopWaitTimeSpin->setValue(weld.dStopWaitTime);

    m_wrapCurrent1Spin->setValue(weld.dWrapCurrentt1);
    m_wrapVoltage1Spin->setValue(weld.dWrapVoltage1);
    m_wrapWaitTime1Spin->setValue(weld.dWrapWaitTime1);
    m_wrapCurrent2Spin->setValue(weld.dWrapCurrentt2);
    m_wrapVoltage2Spin->setValue(weld.dWrapVoltage2);
    m_wrapWaitTime2Spin->setValue(weld.dWrapWaitTime2);
    m_wrapCurrent3Spin->setValue(weld.dWrapCurrentt3);
    m_wrapVoltage3Spin->setValue(weld.dWrapVoltage3);
    m_wrapWaitTime3Spin->setValue(weld.dWrapWaitTime3);
    SetChecked(m_wrapCurrent1EnableCheck, weld.nWrapCurrent1Enable);
    SetChecked(m_wrapVoltage1EnableCheck, weld.nWrapVoltage1Enable);
    SetChecked(m_wrapWaitTime1EnableCheck, weld.nWrapWaitTime1Enable);
    SetChecked(m_wrapCurrent2EnableCheck, weld.nWrapCurrent2Enable);
    SetChecked(m_wrapVoltage2EnableCheck, weld.nWrapVoltage2Enable);
    SetChecked(m_wrapWaitTime2EnableCheck, weld.nWrapWaitTime2Enable);
    SetChecked(m_wrapCurrent3EnableCheck, weld.nWrapCurrent3Enable);
    SetChecked(m_wrapVoltage3EnableCheck, weld.nWrapVoltage3Enable);
    SetChecked(m_wrapWaitTime3EnableCheck, weld.nWrapWaitTime3Enable);

    SetComboByData(m_weldPostureCombo, weld.nWeldPostureType);
    SetComboByData(m_dynamicModeCombo, weld.nWeldMethod);  // 动态特性回填(复用 nWeldMethod)
    if (m_weldOverlapSpin != nullptr) { m_weldOverlapSpin->setValue(weld.dWeldOverlapRel); }
    m_cornerTransitionRadiusSpin->setValue(weld.dCornerArcTransitionRadius);
    m_cornerTransitionSpeedSpin->setValue(weld.dCornerArcTransitionSpeed);
    m_cornerTransitionCurrentSpin->setValue(weld.dCornerArcTransitionCurrent);
    m_cornerTransitionVoltageSpin->setValue(weld.dCornerArcTransitionVoltage);
    SetChecked(m_cornerTransitionRadiusEnableCheck, weld.nCornerArcTransitionRadiusEnable);
    SetChecked(m_cornerTransitionSpeedEnableCheck, weld.nCornerArcTransitionSpeedEnable);
    SetChecked(m_cornerTransitionCurrentEnableCheck, weld.nCornerArcTransitionCurrentEnable);
    SetChecked(m_cornerTransitionVoltageEnableCheck, weld.nCornerArcTransitionVoltageEnable);
    if (m_cornerTransitionScopeCombo != nullptr)
    {
        m_cornerTransitionScopeCombo->setCurrentIndex(qBound(0, weld.nCornerArcTransitionApplyScope, 2));
    }
    if (m_finalWeldStepSpin != nullptr)
    {
        // 老工艺还没存过点间距(<=0)时预填当前实际生效值（测量页），保存即固化进工艺。
        m_finalWeldStepSpin->setValue(weld.dFinalWeldTrajectoryStepMm > 0.0
            ? weld.dFinalWeldTrajectoryStepMm
            : RobotDataHelper::ReadActiveFinalWeldStepFallbackMm(m_unitName));
    }
    if (m_weldDirectionCombo != nullptr)
    {
        // 老工艺还没存过焊接方向(0)时预填当前实际生效值（测量页），保存即固化进工艺。
        const int direction = weld.nWeldDirection != 0
            ? (weld.nWeldDirection < 0 ? -1 : 1)
            : RobotDataHelper::ReadActiveWeldDirectionFallback(m_unitName);
        const int comboIndex = m_weldDirectionCombo->findData(direction);
        m_weldDirectionCombo->setCurrentIndex(comboIndex >= 0 ? comboIndex : 0);
    }

    const auto& weaveList = m_file.GetWeaveTypeList();
    if (weld.nWeaveTypeNo >= 0 && weld.nWeaveTypeNo < static_cast<int>(weaveList.size()))
    {
        const auto& weave = weaveList[weld.nWeaveTypeNo];
        SetComboByData(m_weaveTypeCombo, weave.nWeaveType);
        SetComboByData(m_weaveShapeCombo, weave.nWeaveShape);
        m_weaveFrequencySpin->setValue(weave.dWeaveFrequencyHz);
        m_weaveAmplitudeSpin->setValue(weave.dWeaveAmplitudeMm);
        m_swingDirectionSpin->setValue(weave.dSwingDirectionDeg);
        m_weavePlaneAngleSpin->setValue(weave.dWeavePlaneAngleDeg);
        m_spaceAngleSpin->setValue(weave.dSpaceAngleDeg);
        m_pauseTime1Spin->setValue(weave.nPauseTime1Ms);
        m_pauseTime2Spin->setValue(weave.nPauseTime2Ms);
        m_pauseTime3Spin->setValue(weave.nPauseTime3Ms);
        m_pauseTime4Spin->setValue(weave.nPauseTime4Ms);
        SetComboByData(m_pauseContinueCombo, weave.nPauseContinue);
        m_endLengthSpin->setValue(weave.dEndLengthMm);
        m_endWidthSpin->setValue(weave.dEndWidthMm);
        m_centerHeightSpin->setValue(weave.dCenterHeightMm);
    }

    const T_TrackData& track = weld.tTrackParam;
    m_lateralBeginCycleSpin->setValue(track.nLateralBeginCycle);
    m_lateralGainSpin->setValue(track.dLateralGain);
    m_leftAreaCoefficientSpin->setValue(track.dLeftAreaCoefficient);
    m_rightAreaCoefficientSpin->setValue(track.dRightAreaCoefficient);
    SetComboByData(m_verticalModeCombo, track.nVerticalModeFlag);
    m_verticalReferenceCurrentSpin->setValue(track.dVerticalReferenceCurrent);
    m_verticalBeginCycleSpin->setValue(track.nVerticalBeginCycle);
    m_verticalSustainCycleSpin->setValue(track.nVerticalSustainCycle);
    m_verticalCycleLengthSpin->setValue(track.dVerticalCycleLength);
    m_verticalGainSpin->setValue(track.dVerticalGain);
    SetComboByData(m_trackIntervalModeCombo, track.nTimeOrDistanceMode);
    m_timeIntervalSpin->setValue(track.nTimeIntervalMs);
    m_distanceIntervalSpin->setValue(track.nDistanceIntervalMm);
    m_lateralMinCompSpin->setValue(track.dLateralMinCompPerCycle);
    m_lateralMaxCompSpin->setValue(track.dLateralMaxCompPerCycle);
    m_lateralTotalMaxCompSpin->setValue(track.dLateralMaxCompTotal);
    m_lateralAsymmetrySpin->setValue(track.dLateralAsymmetryCoefficient);
    m_verticalMinCompSpin->setValue(track.dVerticalMinCompPerCycle);
    m_verticalMaxCompSpin->setValue(track.dVerticalMaxCompPerCycle);
    m_verticalTotalMaxCompSpin->setValue(track.dVerticalMaxCompTotal);
    m_verticalAsymmetrySpin->setValue(track.dVerticalAsymmetryCoefficient);
    UpdateFeaturePageEnabled();
    m_isLoading = false;
}

bool WeldProcessDialog::SaveData()
{
    T_WELD_PARA weldItem;
    if (!CollectWeldFromUi(weldItem))
    {
        ShowError("当前焊接工艺参数存在非法值。");
        return false;
    }

    T_WeaveDate weaveItem;
    if (!CollectWeaveFromUi(weaveItem))
    {
        ShowError("当前摆动参数存在非法值。");
        return false;
    }

    int currentRow = CurrentWeldIndex();
    if (currentRow < 0 || currentRow >= static_cast<int>(m_file.GetWeldParaList().size()))
    {
        if (!m_file.GetWeldParaList().empty())
        {
            ShowError("请先选择工艺和焊道。");
            return false;
        }

        if (!m_file.PrepareForRecreate())
        {
            ShowError(DecodeRobotMessageText(m_file.GetLastError()));
            return false;
        }
        weldItem.nLayerNo = 1;
        weldItem.nWeaveTypeNo = 0;

        int newIndex = -1;
        if (!m_file.AddWeldPara(weldItem, newIndex))
        {
            ShowError(DecodeRobotMessageText(m_file.GetLastError()));
            return false;
        }
        currentRow = newIndex;
    }

    weldItem.nWeaveTypeNo = qBound(0, weldItem.nWeaveTypeNo, qMax(0, m_file.GetAllWeaveTypeNum() - 1));

    if (!m_file.UpdateWeldPara(currentRow, weldItem))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return false;
    }
    if (!m_file.UpdateWeaveType(weldItem.nWeaveTypeNo, weaveItem))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return false;
    }

    ui->statusLabel->setText("保存成功");
    QMessageBox::information(this, "工艺参数", "参数已保存。");
    LoadToUi(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1,
        m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : -1);
    return true;
}

bool WeldProcessDialog::CollectWeaveFromUi(T_WeaveDate& out) const
{
    out.nWeaveType = ComboData(m_weaveTypeCombo, 0);
    out.nWeaveShape = ComboData(m_weaveShapeCombo, 6);
    out.dWeaveFrequencyHz = m_weaveFrequencySpin->value();
    out.dWeaveAmplitudeMm = m_weaveAmplitudeSpin->value();
    out.dSwingDirectionDeg = m_swingDirectionSpin->value();
    out.dWeavePlaneAngleDeg = m_weavePlaneAngleSpin->value();
    out.dSpaceAngleDeg = m_spaceAngleSpin->value();
    out.nPauseTime1Ms = m_pauseTime1Spin->value();
    out.nPauseTime2Ms = m_pauseTime2Spin->value();
    out.nPauseTime3Ms = m_pauseTime3Spin->value();
    out.nPauseTime4Ms = m_pauseTime4Spin->value();
    out.nPauseContinue = ComboData(m_pauseContinueCombo, 0);
    out.dEndLengthMm = m_endLengthSpin->value();
    out.dEndWidthMm = m_endWidthSpin->value();
    out.dCenterHeightMm = m_centerHeightSpin->value();
    return true;
}

bool WeldProcessDialog::CollectWeldFromUi(T_WELD_PARA& out) const
{
    out = {};
    out.strWorkPeace = m_workPeaceEdit->text().trimmed().toUtf8().constData();
    out.strWeldType = m_weldTypeEdit->text().trimmed().toUtf8().constData();
    out.nArcMode = qBound(0, ComboData(m_arcModeCombo, 4), 7);
    out.dWeldAngleSize = m_weldAngleSizeSpin->value();
    out.nLayerNo = m_beadListWidget != nullptr ? (m_beadListWidget->currentRow() + 1) : 1;
    out.dStartArcCurrent = m_startArcCurrentSpin->value();
    out.dStartArcVoltage = m_startArcVoltageSpin->value();
    out.dStartWaitTime = m_startWaitTimeSpin->value();
    out.dTrackCurrent = m_trackCurrentSpin->value();
    out.dTrackVoltage = m_trackVoltageSpin->value();
    out.WeldVelocity = m_weldVelocitySpin->value();
    out.dStopArcCurrent = m_stopArcCurrentSpin->value();
    out.dStopArcVoltage = m_stopArcVoltageSpin->value();
    out.dStopWaitTime = m_stopWaitTimeSpin->value();
    out.dWrapCurrentt1 = m_wrapCurrent1Spin->value();
    out.dWrapVoltage1 = m_wrapVoltage1Spin->value();
    out.dWrapWaitTime1 = m_wrapWaitTime1Spin->value();
    out.dWrapCurrentt2 = m_wrapCurrent2Spin->value();
    out.dWrapVoltage2 = m_wrapVoltage2Spin->value();
    out.dWrapWaitTime2 = m_wrapWaitTime2Spin->value();
    out.dWrapCurrentt3 = m_wrapCurrent3Spin->value();
    out.dWrapVoltage3 = m_wrapVoltage3Spin->value();
    out.dWrapWaitTime3 = m_wrapWaitTime3Spin->value();
    out.nWrapCurrent1Enable = IsChecked(m_wrapCurrent1EnableCheck) ? 1 : 0;
    out.nWrapVoltage1Enable = IsChecked(m_wrapVoltage1EnableCheck) ? 1 : 0;
    out.nWrapWaitTime1Enable = IsChecked(m_wrapWaitTime1EnableCheck) ? 1 : 0;
    out.nWrapCurrent2Enable = IsChecked(m_wrapCurrent2EnableCheck) ? 1 : 0;
    out.nWrapVoltage2Enable = IsChecked(m_wrapVoltage2EnableCheck) ? 1 : 0;
    out.nWrapWaitTime2Enable = IsChecked(m_wrapWaitTime2EnableCheck) ? 1 : 0;
    out.nWrapCurrent3Enable = IsChecked(m_wrapCurrent3EnableCheck) ? 1 : 0;
    out.nWrapVoltage3Enable = IsChecked(m_wrapVoltage3EnableCheck) ? 1 : 0;
    out.nWrapWaitTime3Enable = IsChecked(m_wrapWaitTime3EnableCheck) ? 1 : 0;
    out.dCornerArcTransitionRadius = m_cornerTransitionRadiusSpin->value();
    out.dCornerArcTransitionSpeed = m_cornerTransitionSpeedSpin->value();
    out.dCornerArcTransitionCurrent = m_cornerTransitionCurrentSpin->value();
    out.dCornerArcTransitionVoltage = m_cornerTransitionVoltageSpin->value();
    out.nCornerArcTransitionRadiusEnable = IsChecked(m_cornerTransitionRadiusEnableCheck) ? 1 : 0;
    out.nCornerArcTransitionSpeedEnable = IsChecked(m_cornerTransitionSpeedEnableCheck) ? 1 : 0;
    out.nCornerArcTransitionCurrentEnable = IsChecked(m_cornerTransitionCurrentEnableCheck) ? 1 : 0;
    out.nCornerArcTransitionVoltageEnable = IsChecked(m_cornerTransitionVoltageEnableCheck) ? 1 : 0;
    out.nCornerArcTransitionApplyScope = m_cornerTransitionScopeCombo != nullptr
        ? qBound(0, m_cornerTransitionScopeCombo->currentIndex(), 2)
        : 2;
    out.nWeldPostureType = qBound(0, ComboData(m_weldPostureCombo, 1), 3);
    out.dWeldOverlapRel = m_weldOverlapSpin != nullptr ? m_weldOverlapSpin->value() : 20.0;
    out.dFinalWeldTrajectoryStepMm = m_finalWeldStepSpin != nullptr ? m_finalWeldStepSpin->value() : 0.0;
    out.nWeldDirection = m_weldDirectionCombo != nullptr
        ? (m_weldDirectionCombo->currentData().toInt() < 0 ? -1 : 1)
        : 0;
    if (out.nCornerArcTransitionCurrentEnable != out.nCornerArcTransitionVoltageEnable)
    {
        QMessageBox::warning(
            const_cast<WeldProcessDialog*>(this),
            "参数错误",
            "拐点过渡电流和过渡电压必须同时启用或同时关闭。");
        return false;
    }
    out.CrosswiseOffset = m_crosswiseOffsetSpin->value();
    out.verticalOffset = m_verticalOffsetSpin->value();
    out.nWrapConditionNo = ComboData(m_weaveImplCombo, 0);  // 摆动实现：0=机器人原生 1=上位机自建pointwise
    out.dWeldAngle = m_weldAngleSpin->value();
    out.dWeldDipAngle = m_weldDipAngleSpin->value();
    out.nStandWeldDir = (m_weavePointsCombo != nullptr) ? ComboData(m_weavePointsCombo, 16) : 16;  // 复用 nStandWeldDir 存 pointwise 每周期点数
    out.nWeaveTypeNo = 0;
    out.nWeldMethod = ComboData(m_dynamicModeCombo, 0);  // 复用 nWeldMethod 存动态特性：0=NULL 1=ntdyn0
    out.nWeaveEnable = IsChecked(m_weaveEnableCheck) ? 1 : 0;
    out.nTrackEnable = IsChecked(m_trackEnableCheck) ? 1 : 0;
    out.tWeaveParam = {};
    out.tTrackParam.nLateralBeginCycle = m_lateralBeginCycleSpin->value();
    out.tTrackParam.dLateralGain = m_lateralGainSpin->value();
    out.tTrackParam.dLeftAreaCoefficient = m_leftAreaCoefficientSpin->value();
    out.tTrackParam.dRightAreaCoefficient = m_rightAreaCoefficientSpin->value();
    out.tTrackParam.nVerticalModeFlag = ComboData(m_verticalModeCombo, 1);
    out.tTrackParam.dVerticalReferenceCurrent = m_verticalReferenceCurrentSpin->value();
    out.tTrackParam.nVerticalBeginCycle = m_verticalBeginCycleSpin->value();
    out.tTrackParam.nVerticalSustainCycle = m_verticalSustainCycleSpin->value();
    out.tTrackParam.dVerticalCycleLength = m_verticalCycleLengthSpin->value();
    out.tTrackParam.dVerticalGain = m_verticalGainSpin->value();
    out.tTrackParam.nTimeOrDistanceMode = ComboData(m_trackIntervalModeCombo, 0);
    out.tTrackParam.nTimeIntervalMs = m_timeIntervalSpin->value();
    out.tTrackParam.nDistanceIntervalMm = m_distanceIntervalSpin->value();
    out.tTrackParam.dLateralMinCompPerCycle = m_lateralMinCompSpin->value();
    out.tTrackParam.dLateralMaxCompPerCycle = m_lateralMaxCompSpin->value();
    out.tTrackParam.dLateralMaxCompTotal = m_lateralTotalMaxCompSpin->value();
    out.tTrackParam.dLateralAsymmetryCoefficient = m_lateralAsymmetrySpin->value();
    out.tTrackParam.dVerticalMinCompPerCycle = m_verticalMinCompSpin->value();
    out.tTrackParam.dVerticalMaxCompPerCycle = m_verticalMaxCompSpin->value();
    out.tTrackParam.dVerticalMaxCompTotal = m_verticalTotalMaxCompSpin->value();
    out.tTrackParam.dVerticalAsymmetryCoefficient = m_verticalAsymmetrySpin->value();
    return true;
}

void WeldProcessDialog::OnWeldSelectionChanged(int row)
{
    if (m_isLoading)
    {
        return;
    }
    PopulateBeadList(0);
}

void WeldProcessDialog::OnWeldGroupsReordered()
{
    std::vector<std::string> orderedKeys;
    orderedKeys.reserve(static_cast<size_t>(m_weldListWidget->count()));
    const int currentRow = m_weldListWidget->currentRow();
    const int currentBeadRow = m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : 0;
    for (int i = 0; i < m_weldListWidget->count(); ++i)
    {
        auto* item = m_weldListWidget->item(i);
        if (item == nullptr)
        {
            continue;
        }
        orderedKeys.push_back(item->data(Qt::UserRole).toString().toStdString());
    }

    if (!m_file.ReorderWeldGroups(orderedKeys))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return;
    }
    LoadToUi(currentRow, currentBeadRow);
}

void WeldProcessDialog::OnBeadSelectionChanged(int row)
{
    if (m_isLoading)
    {
        return;
    }

    const auto indices = GetGroupIndices(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1);
    if (row < 0 || row >= static_cast<int>(indices.size()))
    {
        return;
    }
    ApplySelectionToUi(indices[row]);
}

void WeldProcessDialog::AddBead()
{
    const int weldIndex = CurrentWeldIndex();
    if (weldIndex < 0 || weldIndex >= static_cast<int>(m_file.GetWeldParaList().size()))
    {
        ShowError("请先选择一个工艺组，再添加焊道。");
        return;
    }

    T_WELD_PARA item = m_file.GetWeldParaList()[weldIndex];
    const auto indices = GetGroupIndices(m_weldListWidget->currentRow());
    int maxLayer = 0;
    for (int index : indices)
    {
        maxLayer = qMax(maxLayer, m_file.GetWeldParaList()[index].nLayerNo);
    }
    item.nLayerNo = maxLayer + 1;
    int newIndex = -1;
    if (!m_file.AddWeldPara(item, newIndex))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return;
    }
    LoadToUi(m_weldListWidget->currentRow(), static_cast<int>(indices.size()));
}

void WeldProcessDialog::AddWeldGroup()
{
    T_WELD_PARA item {};
    const int currentIndex = CurrentWeldIndex();
    if (currentIndex >= 0 && currentIndex < static_cast<int>(m_file.GetWeldParaList().size()))
    {
        item = m_file.GetWeldParaList()[currentIndex];
    }
    else
    {
        item.nCornerArcTransitionApplyScope = 2;
        item.nArcMode = 4;
        item.nWeldPostureType = 1;
        item.dWeldOverlapRel = 20.0;
        if (!m_file.PrepareForRecreate())
        {
            ShowError(DecodeRobotMessageText(m_file.GetLastError()));
            return;
        }
    }
    item.nCornerArcTransitionApplyScope = qBound(0, item.nCornerArcTransitionApplyScope, 2);

    const int newGroupNo = GroupCount() + 1;
    item.strWorkPeace = GetStr("新工艺%d", newGroupNo);
    item.strWeldType = std::to_string(newGroupNo);
    item.dWeldAngleSize = item.dWeldAngleSize > 0.0 ? item.dWeldAngleSize : static_cast<double>(newGroupNo);
    item.nLayerNo = 1;

    int newIndex = -1;
    if (!m_file.AddWeldPara(item, newIndex))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return;
    }
    LoadToUi(GroupCount() - 1, 0);
}

void WeldProcessDialog::RemoveWeldGroup()
{
    const int groupRow = m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1;
    const auto indices = GetGroupIndices(groupRow);
    if (indices.empty())
    {
        ShowError("请先选择要删除的工艺组。");
        return;
    }
    if (GroupCount() <= 1)
    {
        ShowError("至少需要保留一组工艺。");
        return;
    }

    for (int i = static_cast<int>(indices.size()) - 1; i >= 0; --i)
    {
        if (!m_file.RemoveWeldPara(indices[i]))
        {
            ShowError(DecodeRobotMessageText(m_file.GetLastError()));
            return;
        }
    }
    LoadToUi(qMax(0, groupRow - 1), 0);
}

void WeldProcessDialog::RemoveBead()
{
    const int weldIndex = CurrentWeldIndex();
    if (weldIndex < 0 || weldIndex >= static_cast<int>(m_file.GetWeldParaList().size()))
    {
        ShowError("请先选择要删除的焊道。");
        return;
    }

    const auto indices = GetGroupIndices(m_weldListWidget->currentRow());
    if (indices.size() <= 1)
    {
        ShowError("当前工艺组至少需要保留一条焊道。");
        return;
    }

    const int beadRow = m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : 0;
    if (!m_file.RemoveWeldPara(weldIndex))
    {
        ShowError(DecodeRobotMessageText(m_file.GetLastError()));
        return;
    }
    LoadToUi(m_weldListWidget->currentRow(), qMax(0, beadRow - 1));
}

void WeldProcessDialog::ShowError(const QString& message) const
{
    QMessageBox::critical(const_cast<WeldProcessDialog*>(this), "工艺参数", message);
}

bool WeldProcessDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString WeldProcessDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << QString::number(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1)
           << QString::number(m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : -1)
           << (m_workPeaceEdit != nullptr ? m_workPeaceEdit->text().trimmed() : QString())
           << (m_weldTypeEdit != nullptr ? m_weldTypeEdit->text().trimmed() : QString())
           << QString::number(ComboData(m_arcModeCombo, 4))
           << QString::number(m_weldAngleSizeSpin != nullptr ? m_weldAngleSizeSpin->value() : 0.0, 'f', 6)
           << QString::number(IsChecked(m_weaveEnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_trackEnableCheck) ? 1 : 0)
           << QString::number(m_startArcCurrentSpin != nullptr ? m_startArcCurrentSpin->value() : 0.0, 'f', 6)
           << QString::number(m_startArcVoltageSpin != nullptr ? m_startArcVoltageSpin->value() : 0.0, 'f', 6)
           << QString::number(m_startWaitTimeSpin != nullptr ? m_startWaitTimeSpin->value() : 0.0, 'f', 6)
           << QString::number(m_trackCurrentSpin != nullptr ? m_trackCurrentSpin->value() : 0.0, 'f', 6)
           << QString::number(m_trackVoltageSpin != nullptr ? m_trackVoltageSpin->value() : 0.0, 'f', 6)
           << QString::number(m_weldVelocitySpin != nullptr ? m_weldVelocitySpin->value() : 0.0, 'f', 6)
           << QString::number(m_crosswiseOffsetSpin != nullptr ? m_crosswiseOffsetSpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalOffsetSpin != nullptr ? m_verticalOffsetSpin->value() : 0.0, 'f', 6)
           << QString::number(m_weldAngleSpin != nullptr ? m_weldAngleSpin->value() : 0.0, 'f', 6)
           << QString::number(m_weldDipAngleSpin != nullptr ? m_weldDipAngleSpin->value() : 0.0, 'f', 6)
           << QString::number(m_stopArcCurrentSpin != nullptr ? m_stopArcCurrentSpin->value() : 0.0, 'f', 6)
           << QString::number(m_stopArcVoltageSpin != nullptr ? m_stopArcVoltageSpin->value() : 0.0, 'f', 6)
           << QString::number(m_stopWaitTimeSpin != nullptr ? m_stopWaitTimeSpin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapCurrent1Spin != nullptr ? m_wrapCurrent1Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapVoltage1Spin != nullptr ? m_wrapVoltage1Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapWaitTime1Spin != nullptr ? m_wrapWaitTime1Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapCurrent2Spin != nullptr ? m_wrapCurrent2Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapVoltage2Spin != nullptr ? m_wrapVoltage2Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapWaitTime2Spin != nullptr ? m_wrapWaitTime2Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapCurrent3Spin != nullptr ? m_wrapCurrent3Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapVoltage3Spin != nullptr ? m_wrapVoltage3Spin->value() : 0.0, 'f', 6)
           << QString::number(m_wrapWaitTime3Spin != nullptr ? m_wrapWaitTime3Spin->value() : 0.0, 'f', 6)
           << QString::number(IsChecked(m_wrapCurrent1EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapVoltage1EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapWaitTime1EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapCurrent2EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapVoltage2EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapWaitTime2EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapCurrent3EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapVoltage3EnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_wrapWaitTime3EnableCheck) ? 1 : 0)
           << QString::number(m_cornerTransitionRadiusSpin != nullptr ? m_cornerTransitionRadiusSpin->value() : 0.0, 'f', 6)
           << QString::number(m_cornerTransitionSpeedSpin != nullptr ? m_cornerTransitionSpeedSpin->value() : 0.0, 'f', 6)
           << QString::number(m_cornerTransitionCurrentSpin != nullptr ? m_cornerTransitionCurrentSpin->value() : 0.0, 'f', 6)
           << QString::number(m_cornerTransitionVoltageSpin != nullptr ? m_cornerTransitionVoltageSpin->value() : 0.0, 'f', 6)
           << QString::number(IsChecked(m_cornerTransitionRadiusEnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_cornerTransitionSpeedEnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_cornerTransitionCurrentEnableCheck) ? 1 : 0)
           << QString::number(IsChecked(m_cornerTransitionVoltageEnableCheck) ? 1 : 0)
           << QString::number(m_cornerTransitionScopeCombo != nullptr ? m_cornerTransitionScopeCombo->currentIndex() : 2)
           << QString::number(m_finalWeldStepSpin != nullptr ? m_finalWeldStepSpin->value() : 0.0, 'f', 6)
           << QString::number(m_weldDirectionCombo != nullptr ? m_weldDirectionCombo->currentData().toInt() : 0)
           << QString::number(ComboData(m_weaveTypeCombo, 0))
           << QString::number(ComboData(m_weaveShapeCombo, 6))
           << QString::number(m_weaveFrequencySpin != nullptr ? m_weaveFrequencySpin->value() : 0.0, 'f', 6)
           << QString::number(m_weaveAmplitudeSpin != nullptr ? m_weaveAmplitudeSpin->value() : 0.0, 'f', 6)
           << QString::number(m_swingDirectionSpin != nullptr ? m_swingDirectionSpin->value() : 0.0, 'f', 6)
           << QString::number(m_weavePlaneAngleSpin != nullptr ? m_weavePlaneAngleSpin->value() : 0.0, 'f', 6)
           << QString::number(m_spaceAngleSpin != nullptr ? m_spaceAngleSpin->value() : 0.0, 'f', 6)
           << QString::number(m_pauseTime1Spin != nullptr ? m_pauseTime1Spin->value() : 0)
           << QString::number(m_pauseTime2Spin != nullptr ? m_pauseTime2Spin->value() : 0)
           << QString::number(m_pauseTime3Spin != nullptr ? m_pauseTime3Spin->value() : 0)
           << QString::number(m_pauseTime4Spin != nullptr ? m_pauseTime4Spin->value() : 0)
           << QString::number(ComboData(m_pauseContinueCombo, 0))
           << QString::number(m_endLengthSpin != nullptr ? m_endLengthSpin->value() : 0.0, 'f', 6)
           << QString::number(m_endWidthSpin != nullptr ? m_endWidthSpin->value() : 0.0, 'f', 6)
           << QString::number(m_centerHeightSpin != nullptr ? m_centerHeightSpin->value() : 0.0, 'f', 6)
           << QString::number(m_lateralBeginCycleSpin != nullptr ? m_lateralBeginCycleSpin->value() : 0)
           << QString::number(m_lateralGainSpin != nullptr ? m_lateralGainSpin->value() : 0.0, 'f', 6)
           << QString::number(m_leftAreaCoefficientSpin != nullptr ? m_leftAreaCoefficientSpin->value() : 0.0, 'f', 6)
           << QString::number(m_rightAreaCoefficientSpin != nullptr ? m_rightAreaCoefficientSpin->value() : 0.0, 'f', 6)
           << QString::number(ComboData(m_verticalModeCombo, 1))
           << QString::number(m_verticalReferenceCurrentSpin != nullptr ? m_verticalReferenceCurrentSpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalBeginCycleSpin != nullptr ? m_verticalBeginCycleSpin->value() : 0)
           << QString::number(m_verticalSustainCycleSpin != nullptr ? m_verticalSustainCycleSpin->value() : 0)
           << QString::number(m_verticalCycleLengthSpin != nullptr ? m_verticalCycleLengthSpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalGainSpin != nullptr ? m_verticalGainSpin->value() : 0.0, 'f', 6)
           << QString::number(ComboData(m_trackIntervalModeCombo, 0))
           << QString::number(m_timeIntervalSpin != nullptr ? m_timeIntervalSpin->value() : 0)
           << QString::number(m_distanceIntervalSpin != nullptr ? m_distanceIntervalSpin->value() : 0)
           << QString::number(m_lateralMinCompSpin != nullptr ? m_lateralMinCompSpin->value() : 0.0, 'f', 6)
           << QString::number(m_lateralMaxCompSpin != nullptr ? m_lateralMaxCompSpin->value() : 0.0, 'f', 6)
           << QString::number(m_lateralTotalMaxCompSpin != nullptr ? m_lateralTotalMaxCompSpin->value() : 0.0, 'f', 6)
           << QString::number(m_lateralAsymmetrySpin != nullptr ? m_lateralAsymmetrySpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalMinCompSpin != nullptr ? m_verticalMinCompSpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalMaxCompSpin != nullptr ? m_verticalMaxCompSpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalTotalMaxCompSpin != nullptr ? m_verticalTotalMaxCompSpin->value() : 0.0, 'f', 6)
           << QString::number(m_verticalAsymmetrySpin != nullptr ? m_verticalAsymmetrySpin->value() : 0.0, 'f', 6);
    return fields.join('\n');
}

void WeldProcessDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}
