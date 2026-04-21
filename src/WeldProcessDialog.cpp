#include "WeldProcessDialog.h"
#include "ui_WeldProcessDialog.h"
#include "WindowStyleHelper.h"

#include <QAbstractSpinBox>
#include <QAbstractItemModel>
#include <QAbstractItemView>
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
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QStackedWidget>
#include <QSpinBox>
#include <QStyleOptionSpinBox>
#include <QStringList>
#include <QVBoxLayout>

namespace
{
constexpr double kDoubleMin = -999999.0;
constexpr double kDoubleMax = 999999.0;

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
        DrawPlusMinus();
    }

private:
    void DrawPlusMinus()
    {
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
        DrawPlusMinus();
    }

private:
    void DrawPlusMinus()
    {
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

QGridLayout* CreateTwoColumnForm(QGroupBox* group)
{
    auto* layout = new QGridLayout(group);
    layout->setContentsMargins(14, 8, 14, 12);
    layout->setHorizontalSpacing(10);
    layout->setVerticalSpacing(8);
    layout->setColumnStretch(1, 1);
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
    layout->addWidget(label, row, col);
    layout->addWidget(field, row, col + 1);
}
}

WeldProcessDialog::WeldProcessDialog(const T_CONTRAL_UNIT& unitInfo, QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::WeldProcessDialog())
    , m_file(unitInfo)
{
    ui->setupUi(this);
    ApplyUnifiedWindowChrome(this);
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint);
    BuildEditorUi();
    ApplyDialogStyle();

    connect(ui->reloadBtn, &QPushButton::clicked, this, &WeldProcessDialog::ReloadData);
    connect(ui->saveBtn, &QPushButton::clicked, this, &WeldProcessDialog::SaveData);
    connect(ui->closeBtn, &QPushButton::clicked, this, &WeldProcessDialog::reject);
    connect(m_weldListWidget, &QListWidget::currentRowChanged, this, &WeldProcessDialog::OnWeldSelectionChanged);
    ui->useWeldSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    ui->useWeaveSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    ui->useWeldSpin->setReadOnly(true);
    ui->useWeaveSpin->setReadOnly(true);
    m_weldListWidget->setSortingEnabled(false);

    LoadToUi();
}

WeldProcessDialog::~WeldProcessDialog()
{
    delete ui;
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
    leftPanel->setMinimumWidth(420);
    auto* leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(12, 12, 12, 12);
    leftLayout->setSpacing(8);

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
    auto* editorLayout = new QVBoxLayout(editorRoot);
    editorLayout->setContentsMargins(0, 0, 0, 0);
    editorLayout->setSpacing(0);

    auto* tabHostContainer = new QWidget(editorRoot);
    auto* tabHostLayout = new QVBoxLayout(tabHostContainer);
    tabHostLayout->setContentsMargins(10, 0, 10, 10);
    tabHostLayout->setSpacing(0);

    auto* switchRow = new QHBoxLayout();
    switchRow->setContentsMargins(18, 8, 0, 0);
    switchRow->setSpacing(0);
    m_normalPageButton = new QPushButton("常规工艺参数", editorRoot);
    m_weavePageButton = new QPushButton("摆动参数", editorRoot);
    m_normalPageButton->setObjectName("topTabButton");
    m_weavePageButton->setObjectName("topTabButton");
    m_normalPageButton->setCheckable(true);
    m_weavePageButton->setCheckable(true);
    m_normalPageButton->setChecked(true);
    m_normalPageButton->setMinimumHeight(40);
    m_weavePageButton->setMinimumHeight(40);
    switchRow->addWidget(m_normalPageButton);
    switchRow->addWidget(m_weavePageButton);
    switchRow->addStretch();
    tabHostLayout->addLayout(switchRow);

    auto* editorFrame = new QFrame(editorRoot);
    editorFrame->setObjectName("tabHostPanel");
    auto* editorFrameLayout = new QVBoxLayout(editorFrame);
    editorFrameLayout->setContentsMargins(12, 12, 12, 12);
    editorFrameLayout->setSpacing(0);

    auto* basicGroup = new QGroupBox("基础工艺参数", editorRoot);
    auto* basicLayout = new QFormLayout(basicGroup);
    basicLayout->setLabelAlignment(Qt::AlignRight);
    basicLayout->setHorizontalSpacing(10);
    basicLayout->setVerticalSpacing(8);
    m_workPeaceEdit = AddSingleTextField(basicLayout, "工件名称");
    m_weldTypeEdit = AddSingleTextField(basicLayout, "焊接类型");
    m_weldAngleSizeSpin = AddSingleDoubleField(basicLayout, "焊脚尺寸");
    m_standWeldDirSpin = AddSingleIntField(basicLayout, "立焊方向", 0, 1);
    m_weldMethodSpin = AddSingleIntField(basicLayout, "焊接方法", 0, 999);
    m_weaveTypeNoSpin = AddSingleIntField(basicLayout, "关联摆动号", 0, 9999);
    basicGroup->setMinimumWidth(440);

    auto* startGroup = new QGroupBox("引弧参数", editorRoot);
    auto* startLayout = new QFormLayout(startGroup);
    startLayout->setLabelAlignment(Qt::AlignRight);
    startLayout->setHorizontalSpacing(10);
    startLayout->setVerticalSpacing(8);
    m_startArcCurrentSpin = AddSingleDoubleField(startLayout, "引弧电流");
    m_startArcVoltageSpin = AddSingleDoubleField(startLayout, "引弧电压");
    m_startWaitTimeSpin = AddSingleDoubleField(startLayout, "引弧时间");
    startGroup->setMinimumWidth(360);

    auto* stopGroup = new QGroupBox("收弧参数", editorRoot);
    auto* stopLayout = new QFormLayout(stopGroup);
    stopLayout->setLabelAlignment(Qt::AlignRight);
    stopLayout->setHorizontalSpacing(10);
    stopLayout->setVerticalSpacing(8);
    m_stopArcCurrentSpin = AddSingleDoubleField(stopLayout, "收弧电流");
    m_stopArcVoltageSpin = AddSingleDoubleField(stopLayout, "收弧电压");
    m_stopWaitTimeSpin = AddSingleDoubleField(stopLayout, "收弧时间");
    stopGroup->setMinimumWidth(360);

    auto* weldGroup = new QGroupBox("焊接与运动参数", editorRoot);
    auto* weldLayout = CreateTwoColumnForm(weldGroup);
    m_trackCurrentSpin = AddDoubleField(weldLayout, "焊接电流");
    m_trackVoltageSpin = AddDoubleField(weldLayout, "焊接电压");
    m_weldVelocitySpin = AddDoubleField(weldLayout, "焊接速度");
    m_crosswiseOffsetSpin = AddDoubleField(weldLayout, "横向补偿");
    m_verticalOffsetSpin = AddDoubleField(weldLayout, "竖向补偿");
    m_wrapConditionNoSpin = AddIntField(weldLayout, "摆动条件号", 0, 9999);
    m_weldAngleSpin = AddDoubleField(weldLayout, "焊接角度");
    m_weldDipAngleSpin = AddDoubleField(weldLayout, "焊接倾角");
    weldGroup->setMinimumWidth(520);

    auto* wrapGroup = new QGroupBox("搭接/包角参数", editorRoot);
    auto* wrapLayout = CreateTwoColumnForm(wrapGroup);
    m_wrapCurrent1Spin = AddDoubleField(wrapLayout, "段1电流");
    m_wrapVoltage1Spin = AddDoubleField(wrapLayout, "段1电压");
    m_wrapWaitTime1Spin = AddDoubleField(wrapLayout, "段1时间");
    m_wrapCurrent2Spin = AddDoubleField(wrapLayout, "段2电流");
    m_wrapVoltage2Spin = AddDoubleField(wrapLayout, "段2电压");
    m_wrapWaitTime2Spin = AddDoubleField(wrapLayout, "段2时间");
    m_wrapCurrent3Spin = AddDoubleField(wrapLayout, "段3电流");
    m_wrapVoltage3Spin = AddDoubleField(wrapLayout, "段3电压");
    m_wrapWaitTime3Spin = AddDoubleField(wrapLayout, "段3时间");
    wrapGroup->setMinimumWidth(520);

    auto* weaveGroup = new QGroupBox("摆动参数", editorRoot);
    auto* weaveLayout = CreateTwoColumnForm(weaveGroup);
    m_weaveTypeSpin = AddIntField(weaveLayout, "摆动类型", 0, 9999);
    m_freqSpin = AddDoubleField(weaveLayout, "频率");
    m_ampLeftSpin = AddDoubleField(weaveLayout, "左摆幅");
    m_ampRightSpin = AddDoubleField(weaveLayout, "右摆幅");
    m_stopTimeLeftSpin = AddIntField(weaveLayout, "左停留", 0, 999999);
    m_stopTimeCenterSpin = AddIntField(weaveLayout, "中停留", 0, 999999);
    m_stopTimeRightSpin = AddIntField(weaveLayout, "右停留", 0, 999999);
    m_rotAngleXSpin = AddDoubleField(weaveLayout, "X旋转角");
    m_rotAngleZSpin = AddDoubleField(weaveLayout, "Z旋转角");
    m_delayTypeLeftSpin = AddIntField(weaveLayout, "左延时类型", 0, 999);
    m_delayTypeCenterSpin = AddIntField(weaveLayout, "中延时类型", 0, 999);
    m_delayTypeRightSpin = AddIntField(weaveLayout, "右延时类型", 0, 999);
    m_rotAngleLeftSpin = AddDoubleField(weaveLayout, "左摆角");
    m_rotAngleRightSpin = AddDoubleField(weaveLayout, "右摆角");
    weaveGroup->setMinimumWidth(560);

    auto* topRow = new QHBoxLayout();
    topRow->setSpacing(12);
    topRow->addWidget(basicGroup);
    topRow->addWidget(startGroup);
    topRow->addWidget(stopGroup);
    topRow->addStretch();

    auto* bottomRow = new QHBoxLayout();
    bottomRow->setSpacing(12);
    bottomRow->addWidget(weldGroup);
    bottomRow->addWidget(wrapGroup);
    bottomRow->addStretch();

    auto* normalPage = new QWidget(editorRoot);
    auto* normalPageOuterLayout = new QVBoxLayout(normalPage);
    normalPageOuterLayout->setContentsMargins(0, 0, 0, 0);
    normalPageOuterLayout->setSpacing(0);
    auto* normalPanel = new QFrame(normalPage);
    normalPanel->setObjectName("editorPagePanel");
    auto* normalLayout = new QVBoxLayout(normalPanel);
    normalLayout->setContentsMargins(16, 16, 16, 16);
    normalLayout->setSpacing(12);
    normalLayout->addLayout(topRow);
    normalLayout->addLayout(bottomRow);
    normalLayout->addStretch();
    normalPageOuterLayout->addWidget(normalPanel);

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
    weavePageOuterLayout->addWidget(weavePanel);

    m_editorStack = new QStackedWidget(editorRoot);
    m_editorStack->addWidget(normalPage);
    m_editorStack->addWidget(weavePage);
    editorFrameLayout->addWidget(m_editorStack);
    tabHostLayout->addWidget(editorFrame);
    editorLayout->addWidget(tabHostContainer);
    editorLayout->addStretch();

    splitterLayout->addWidget(editorRoot, 3);

    connect(m_normalPageButton, &QPushButton::clicked, this, [this]()
        {
            m_normalPageButton->setChecked(true);
            m_weavePageButton->setChecked(false);
            m_editorStack->setCurrentIndex(0);
        });
    connect(m_weavePageButton, &QPushButton::clicked, this, [this]()
        {
            m_normalPageButton->setChecked(false);
            m_weavePageButton->setChecked(true);
            m_editorStack->setCurrentIndex(1);
        });
    connect(m_addBeadButton, &QPushButton::clicked, this, &WeldProcessDialog::AddBead);
    connect(m_removeBeadButton, &QPushButton::clicked, this, &WeldProcessDialog::RemoveBead);
    connect(m_addWeldButton, &QPushButton::clicked, this, &WeldProcessDialog::AddWeldGroup);
    connect(m_removeWeldButton, &QPushButton::clicked, this, &WeldProcessDialog::RemoveWeldGroup);
    connect(m_beadListWidget, &QListWidget::currentRowChanged, this, &WeldProcessDialog::OnBeadSelectionChanged);
    connect(m_weldListWidget->model(), &QAbstractItemModel::rowsMoved, this, [this]()
        {
            if (!m_isLoading)
            {
                OnWeldGroupsReordered();
            }
        });
}

void WeldProcessDialog::ApplyDialogStyle()
{
    setStyleSheet(
        "QDialog{background:#111820;color:#ECF3F4;}"
        "QFrame#leftPanel,QFrame#beadPanel,QGroupBox,QFrame#editorPagePanel,QFrame#tabHostPanel{background:#111820;border:1px solid #2E4656;border-radius:14px;}"
        "QWidget#topPanel{background:#111820;border:none;}"
        "QStackedWidget{background:transparent;}"
        "QFrame#tabHostPanel{padding-top:0px;border-radius:14px;margin-top:8px;}"
        "QFrame#editorPagePanel{border:none;background:transparent;border-radius:0px;}"
        "QGroupBox{margin-top:0px;padding:30px 14px 14px 14px;font-weight:600;}"
        "QGroupBox::title{subcontrol-origin:padding;subcontrol-position:top left;left:12px;top:7px;padding:0;color:#9ED8DB;background:transparent;}"
        "QLabel#titleLabel{font-size:22px;font-weight:700;color:#F7FCFC;letter-spacing:1px;}"
        "QLabel#descLabel,QLabel#statusLabel{color:#BACBD1;}"
        "QLabel#sectionTitle{font-size:16px;font-weight:600;color:#9ED8DB;}"
        "QLabel{color:#BACBD1;}"
        "QLineEdit,QSpinBox,QDoubleSpinBox,QListWidget{background:#0B1117;color:#F5FAFA;border:1px solid #385366;border-radius:8px;padding:4px 8px;selection-background-color:#2D5465;selection-color:#F5FAFA;}"
        "QSpinBox,QDoubleSpinBox{padding-right:36px;min-height:24px;}"
        "QSpinBox::up-button,QDoubleSpinBox::up-button{"
            "subcontrol-origin:border;"
            "subcontrol-position:top right;"
            "width:18px;"
            "border-left:1px solid #385366;"
            "border-bottom:1px solid #385366;"
            "background:#233645;"
            "border-top-right-radius:8px;"
        "}"
        "QSpinBox::down-button,QDoubleSpinBox::down-button{"
            "subcontrol-origin:border;"
            "subcontrol-position:bottom right;"
            "width:18px;"
            "border-left:1px solid #385366;"
            "background:#233645;"
            "border-bottom-right-radius:8px;"
        "}"
        "QSpinBox::up-button:hover,QDoubleSpinBox::up-button:hover,QSpinBox::down-button:hover,QDoubleSpinBox::down-button:hover{background:#2D5465;}"
        "QPushButton{background:#233645;color:#F5FAFA;border:1px solid #3C6173;border-radius:10px;padding:7px 14px;}"
        "QPushButton:hover{background:#2D5465;border-color:#72D4DD;}"
        "QPushButton:pressed{background:#18303B;}"
        "QPushButton#saveBtn{background:#2D5465;border-color:#72D4DD;color:#FFFFFF;font-weight:600;}"
        "QPushButton#saveBtn:hover{background:#347084;}"
        "QPushButton:checked{background:#305F55;border-color:#7BD8B3;color:#F5FAFA;}"
        "QPushButton#topTabButton{"
            "background:#172530;"
            "color:#BACBD1;"
            "border:1px solid #2E4656;"
            "border-top-left-radius:10px;"
            "border-top-right-radius:10px;"
            "border-bottom-left-radius:10px;"
            "border-bottom-right-radius:10px;"
            "padding:8px 24px;"
            "margin-right:4px;"
            "min-width:152px;"
            "max-width:190px;"
            "font-weight:600;"
        "}"
        "QPushButton#topTabButton:hover{background:#233645;color:#F5FAFA;border-color:#72D4DD;}"
        "QPushButton#topTabButton:checked{"
            "background:#233645;"
            "color:#9ED8DB;"
            "border-color:#72D4DD;"
        "}"
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
    edit->setFixedWidth(150);
    edit->setFixedHeight(28);
    AddTwoColumnField(layout, label, edit);
    return edit;
}

QDoubleSpinBox* WeldProcessDialog::AddDoubleField(QGridLayout* layout, const QString& label, int decimals)
{
    auto* spin = new PlusMinusDoubleSpinBox(this);
    spin->setDecimals(decimals);
    spin->setRange(kDoubleMin, kDoubleMax);
    spin->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
    spin->setFixedWidth(150);
    spin->setFixedHeight(28);
    AddTwoColumnField(layout, label, spin);
    return spin;
}

QSpinBox* WeldProcessDialog::AddIntField(QGridLayout* layout, const QString& label, int minimum, int maximum)
{
    auto* spin = new PlusMinusSpinBox(this);
    spin->setRange(minimum, maximum);
    spin->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
    spin->setFixedWidth(150);
    spin->setFixedHeight(28);
    AddTwoColumnField(layout, label, spin);
    return spin;
}

QLineEdit* WeldProcessDialog::AddSingleTextField(QFormLayout* layout, const QString& label)
{
    auto* edit = new QLineEdit(this);
    edit->setFixedWidth(150);
    edit->setFixedHeight(28);
    layout->addRow(label, edit);
    return edit;
}

QDoubleSpinBox* WeldProcessDialog::AddSingleDoubleField(QFormLayout* layout, const QString& label, int decimals)
{
    auto* spin = new PlusMinusDoubleSpinBox(this);
    spin->setDecimals(decimals);
    spin->setRange(kDoubleMin, kDoubleMax);
    spin->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
    spin->setFixedWidth(150);
    spin->setFixedHeight(28);
    layout->addRow(label, spin);
    return spin;
}

QSpinBox* WeldProcessDialog::AddSingleIntField(QFormLayout* layout, const QString& label, int minimum, int maximum)
{
    auto* spin = new PlusMinusSpinBox(this);
    spin->setRange(minimum, maximum);
    spin->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
    spin->setFixedWidth(150);
    spin->setFixedHeight(28);
    layout->addRow(label, spin);
    return spin;
}

void WeldProcessDialog::LoadToUi(int preferredGroupRow, int preferredBeadRow)
{
    m_isLoading = true;
    if (!m_file.Init())
    {
        const QString err = QString::fromLocal8Bit(m_file.GetLastError().c_str());
        ui->statusLabel->setText("读取失败: " + err);
        m_isLoading = false;
        ShowError(err);
        return;
    }

    ui->useWeldSpin->setMinimumWidth(84);
    ui->useWeaveSpin->setMinimumWidth(84);
    ui->useWeldSpin->setRange(1, qMax(1, GroupCount()));
    ui->useWeaveSpin->setRange(0, qMax(0, m_file.GetAllWeaveTypeNum() - 1));

    PopulateWeldList(preferredGroupRow);
    PopulateBeadList(preferredBeadRow);
    ui->statusLabel->setText(QString("读取完成: %1 / %2")
        .arg(QString::fromLocal8Bit(m_file.GetWeaveIniFilePath().c_str()))
        .arg(QString::fromLocal8Bit(m_file.GetWeldIniFilePath().c_str())));
    m_isLoading = false;
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
        .arg(QString::fromLocal8Bit(item.strWorkPeace.c_str()))
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
        .arg(QString::fromLocal8Bit(item.strWorkPeace.c_str()))
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
    const auto indices = GetGroupIndices(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1);
    int groupDisplayIndex = 1;
    for (int i = 0; i < static_cast<int>(indices.size()); ++i)
    {
        if (indices[i] == row)
        {
            groupDisplayIndex = qMax(1, (m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : 0) + 1);
            break;
        }
    }
    ui->useWeldSpin->setValue(groupDisplayIndex);
    ui->useWeaveSpin->setValue(weld.nWeaveTypeNo);

    m_workPeaceEdit->setText(QString::fromLocal8Bit(weld.strWorkPeace.c_str()));
    m_weldTypeEdit->setText(QString::fromLocal8Bit(weld.strWeldType.c_str()));
    m_weldAngleSizeSpin->setValue(weld.dWeldAngleSize);
    m_standWeldDirSpin->setValue(weld.nStandWeldDir);
    m_weldMethodSpin->setValue(weld.nWeldMethod);
    m_weaveTypeNoSpin->setRange(0, qMax(0, m_file.GetAllWeaveTypeNum() - 1));
    m_weaveTypeNoSpin->setValue(weld.nWeaveTypeNo);

    m_startArcCurrentSpin->setValue(weld.dStartArcCurrent);
    m_startArcVoltageSpin->setValue(weld.dStartArcVoltage);
    m_startWaitTimeSpin->setValue(weld.dStartWaitTime);

    m_trackCurrentSpin->setValue(weld.dTrackCurrent);
    m_trackVoltageSpin->setValue(weld.dTrackVoltage);
    m_weldVelocitySpin->setValue(weld.WeldVelocity);
    m_crosswiseOffsetSpin->setValue(weld.CrosswiseOffset);
    m_verticalOffsetSpin->setValue(weld.verticalOffset);
    m_wrapConditionNoSpin->setValue(weld.nWrapConditionNo);
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

    const auto& weaveList = m_file.GetWeaveTypeList();
    if (weld.nWeaveTypeNo >= 0 && weld.nWeaveTypeNo < static_cast<int>(weaveList.size()))
    {
        const auto& weave = weaveList[weld.nWeaveTypeNo];
        m_weaveTypeSpin->setValue(weave.Type);
        m_freqSpin->setValue(weave.Freq);
        m_ampLeftSpin->setValue(weave.Amp_L);
        m_ampRightSpin->setValue(weave.Amp_R);
        m_stopTimeLeftSpin->setValue(weave.StopTime_L);
        m_stopTimeCenterSpin->setValue(weave.StopTime_C);
        m_stopTimeRightSpin->setValue(weave.StopTime_R);
        m_rotAngleXSpin->setValue(weave.RotAngle_X);
        m_rotAngleZSpin->setValue(weave.RotAngle_Z);
        m_delayTypeLeftSpin->setValue(weave.DelayType_L);
        m_delayTypeCenterSpin->setValue(weave.DelayType_C);
        m_delayTypeRightSpin->setValue(weave.DelayType_R);
        m_rotAngleLeftSpin->setValue(weave.RotAngle_L);
        m_rotAngleRightSpin->setValue(weave.RotAngle_R);
    }
    m_isLoading = false;
}

void WeldProcessDialog::SaveData()
{
    if (!m_file.Init())
    {
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
        return;
    }

    const int currentRow = CurrentWeldIndex();
    if (currentRow < 0 || currentRow >= static_cast<int>(m_file.GetWeldParaList().size()))
    {
        ShowError("请先选择工艺和焊道。");
        return;
    }

    if (!m_file.UpdateUseWeaveTypeNo(ui->useWeaveSpin->value()))
    {
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
        return;
    }
    if (!m_file.UpdateUseWeldParaNo(currentRow))
    {
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
        return;
    }

    T_WELD_PARA weldItem;
    if (!CollectWeldFromUi(weldItem))
    {
        ShowError("当前焊接工艺参数存在非法值。");
        return;
    }

    T_WeaveDate weaveItem;
    if (!CollectWeaveFromUi(weaveItem))
    {
        ShowError("当前摆动参数存在非法值。");
        return;
    }

    if (!m_file.UpdateWeldPara(currentRow, weldItem))
    {
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
        return;
    }
    if (!m_file.UpdateWeaveType(weldItem.nWeaveTypeNo, weaveItem))
    {
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
        return;
    }

    ui->statusLabel->setText("保存成功");
    QMessageBox::information(this, "工艺参数", "参数已保存。");
    LoadToUi(m_weldListWidget != nullptr ? m_weldListWidget->currentRow() : -1,
        m_beadListWidget != nullptr ? m_beadListWidget->currentRow() : -1);
}

bool WeldProcessDialog::CollectWeaveFromUi(T_WeaveDate& out) const
{
    out.Type = m_weaveTypeSpin->value();
    out.Freq = m_freqSpin->value();
    out.Amp_L = m_ampLeftSpin->value();
    out.Amp_R = m_ampRightSpin->value();
    out.StopTime_L = m_stopTimeLeftSpin->value();
    out.StopTime_C = m_stopTimeCenterSpin->value();
    out.StopTime_R = m_stopTimeRightSpin->value();
    out.RotAngle_X = m_rotAngleXSpin->value();
    out.RotAngle_Z = m_rotAngleZSpin->value();
    out.DelayType_L = m_delayTypeLeftSpin->value();
    out.DelayType_C = m_delayTypeCenterSpin->value();
    out.DelayType_R = m_delayTypeRightSpin->value();
    out.RotAngle_L = m_rotAngleLeftSpin->value();
    out.RotAngle_R = m_rotAngleRightSpin->value();
    return true;
}

bool WeldProcessDialog::CollectWeldFromUi(T_WELD_PARA& out) const
{
    out.strWorkPeace = m_workPeaceEdit->text().trimmed().toLocal8Bit().constData();
    out.strWeldType = m_weldTypeEdit->text().trimmed().toLocal8Bit().constData();
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
    out.CrosswiseOffset = m_crosswiseOffsetSpin->value();
    out.verticalOffset = m_verticalOffsetSpin->value();
    out.nWrapConditionNo = m_wrapConditionNoSpin->value();
    out.dWeldAngle = m_weldAngleSpin->value();
    out.dWeldDipAngle = m_weldDipAngleSpin->value();
    out.nStandWeldDir = m_standWeldDirSpin->value();
    out.nWeaveTypeNo = m_weaveTypeNoSpin->value();
    out.nWeldMethod = m_weldMethodSpin->value();
    out.tWeaveParam = {};
    return true;
}

void WeldProcessDialog::OnWeldSelectionChanged(int row)
{
    if (m_isLoading)
    {
        return;
    }
    ui->useWeldSpin->setValue(qMax(1, row + 1));
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
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
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
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
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

    const int newGroupNo = GroupCount() + 1;
    item.strWorkPeace = GetStr("新工艺%d", newGroupNo);
    item.strWeldType = std::to_string(newGroupNo);
    item.dWeldAngleSize = item.dWeldAngleSize > 0.0 ? item.dWeldAngleSize : static_cast<double>(newGroupNo);
    item.nLayerNo = 1;

    int newIndex = -1;
    if (!m_file.AddWeldPara(item, newIndex))
    {
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
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
            ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
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
        ShowError(QString::fromLocal8Bit(m_file.GetLastError().c_str()));
        return;
    }
    LoadToUi(m_weldListWidget->currentRow(), qMax(0, beadRow - 1));
}

void WeldProcessDialog::ShowError(const QString& message) const
{
    QMessageBox::critical(const_cast<WeldProcessDialog*>(this), "工艺参数", message);
}
