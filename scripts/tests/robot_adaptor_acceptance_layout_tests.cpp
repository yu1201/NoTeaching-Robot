#include "RobotAdaptorAcceptanceLayout.h"
#include "InovanceModeSequence.h"
#include "BrandingConfig.h"
#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QSplitter>
#include <QStackedWidget>
#include <QStyleFactory>
#include <iostream>
#include <stdexcept>

// Only branding is stubbed. Use the production scroll/compact-form code, but
// do not link any robot driver, database, camera or main application code.
QIcon BrandingConfig::WindowIcon() { return {}; }
static void Check(bool value, const char* reason)
{ if (!value) { throw std::runtime_error(reason); } }
static void Pump()
{ for (int pass = 0; pass < 6; ++pass) QApplication::processEvents(); }
static QLabel* Note(const QString& text, QVBoxLayout* layout)
{
    auto* label = new QLabel(text);
    label->setWordWrap(true);
    layout->addWidget(label);
    return label;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));
    ConfigureApplicationFontFallback();
    try
    {
        Check(argc == 2, "expected screenshot output directory");
        QDir output(QString::fromLocal8Bit(argv[1]));
        Check(output.mkpath("."), "create screenshot directory");
        int tested = 0;
        for (const QSize size : {QSize(2048, 1095), QSize(1536, 864), QSize(1280, 720)})
        for (int fontSize : {10, 12, 14, 16})
        {
            auto font = app.font(); font.setPointSize(fontSize); app.setFont(font);
            QWidget window;
            window.setAttribute(Qt::WA_DontShowOnScreen, true);
            window.setFont(font);
            window.setStyleSheet(
                "QWidget {background:#101820; color:#B8C7CC;}"
                "QGroupBox {border:1px solid #2E4656; border-radius:12px; margin-top:18px; padding:14px;}"
                "QPushButton {background:#1F3542; color:#F4FAFA; border:1px solid #3C6475; border-radius:10px; padding:8px 14px;}"
                "QPlainTextEdit {background:#0B1117; border:1px solid #2E4656; padding:8px;}");
            window.setFixedSize(size);
            auto* outer = new QVBoxLayout(&window);
            Note(QStringLiteral("机器人适配测试 — RobotC（离线布局测试，不连接机器人）"), outer);
            Note(QStringLiteral("所有机器人品牌使用同一套分阶段验收流程，自动证据和人工结论按轮次保存，可导出报告。"), outer);
            auto* target = new QGroupBox(QStringLiteral("测试机器人"));
            auto* targetLayout = new QVBoxLayout(target);
            Note(QStringLiteral("选择机器人：RobotC — 汇川 Inovance"), targetLayout);
            Note(QStringLiteral("验收记录：20260904_104327_279 ｜ 数据库记录保留"), targetLayout);
            outer->addWidget(target);
            Note(QStringLiteral("统一验收按阶段执行。低速运动、扫描和实际焊接分别确认；无位移组合测试会切模式和上下电。"), outer);

            auto* splitter = new QSplitter(Qt::Horizontal);
            auto* navigation = new QListWidget(splitter);
            for (int stage = 0; stage < 11; ++stage)
            { navigation->addItem(QStringLiteral("%1 验收阶段\n    未测试").arg(stage)); }
            auto* detail = new QWidget(splitter);
            auto* detailLayout = new QVBoxLayout(detail);
            detailLayout->setSpacing(8);
            Note(QStringLiteral("4 低速直线单步移动并返回原位"), detailLayout);
            Note(QStringLiteral("先验证模式组合，再单独确认低速移动。完成后恢复状态并记录接口结果。"), detailLayout);
            Note(QStringLiteral("适配能力门禁：机器人状态读取、直线运动、运行模式切换、程序完成见证、安全中止见证、伺服上下电"), detailLayout);
            Note(QStringLiteral("当前状态：未测试"), detailLayout);
            auto* stack = new QStackedWidget(detail);
            RobotAdaptorAcceptanceLayout::ModeControls controls;
            std::vector<QPushButton*> jointButtons;
            QWidget* motionPage = nullptr;
            for (int stage = 0; stage < 11; ++stage)
            {
                auto* page = new QWidget();
                auto* pageLayout = new QVBoxLayout(page);
                pageLayout->setContentsMargins(0, 4, 0, 4);
                Note(QStringLiteral("必须确认机器人周围安全、实体急停已解除并握住示教器。按需复位报警并回读，随后由品牌底层准备，测试后恢复原状态。"), pageLayout);
                auto* form = new QFormLayout();
                form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
                pageLayout->addLayout(form);
                if (stage == 4)
                {
                    auto* distance = new QDoubleSpinBox(); distance->setValue(10); distance->setSuffix(QStringLiteral(" mm（基坐标+Y）"));
                    auto* speed = new QDoubleSpinBox(); speed->setValue(60); speed->setSuffix(" mm/min");
                    form->addRow(QStringLiteral("低速位移："), distance);
                    form->addRow(QStringLiteral("线速度："), speed);
                    auto* joint = new QPushButton(QStringLiteral("关节专项：J1 +0.5°（1%）"));
                    auto* cancel = new QPushButton(QStringLiteral("结束关节专项（不自动返回）"));
                    joint->setMinimumHeight(44); cancel->setMinimumHeight(44);
                    auto* jointStatus = new QLabel(QStringLiteral("未测试。独立于直线验收；两段单独确认，回读及恢复成功后自动判定。"));
                    jointStatus->setWordWrap(true);
                    form->addRow(QStringLiteral("独立关节往返验收："), joint);
                    form->addRow(QStringLiteral("关节结论："), jointStatus);
                    form->addRow(QStringLiteral("放弃本次往返："), cancel);
                    jointButtons = {joint, cancel};
                    controls = RobotAdaptorAcceptanceLayout::CreateModeControls(page);
                    pageLayout->addWidget(controls.panel);
                    motionPage = page;
                }
                pageLayout->addStretch(1);
                stack->addWidget(page);
            }
            stack->setCurrentIndex(4);
            detailLayout->addWidget(stack, 1);
            auto* evidence = new QGroupBox(QStringLiteral("当前阶段证据"));
            auto* evidenceLayout = new QVBoxLayout(evidence);
            auto* editor = new QPlainTextEdit(); editor->setMinimumHeight(115);
            editor->setPlainText(QStringLiteral("离线界面检查\n按钮、下拉框不得被此证据区裁剪。"));
            evidenceLayout->addWidget(editor);
            detailLayout->addWidget(evidence);
            auto* actions = new QHBoxLayout();
            for (const auto& text : {QStringLiteral("开始低速外移"), QStringLiteral("人工确认通过"), QStringLiteral("标记失败"), QStringLiteral("标记跳过")})
            { auto* button = new QPushButton(text); button->setMinimumSize(150, 44); actions->addWidget(button); }
            detailLayout->addLayout(actions);
            auto* report = new QPushButton(QStringLiteral("导出当前测试报告")); report->setMinimumHeight(44);
            detailLayout->addWidget(report);
            auto* scroll = RobotAdaptorAcceptanceLayout::WrapDetailPanel(splitter, detail);
            splitter->addWidget(navigation); splitter->addWidget(scroll);
            splitter->setChildrenCollapsible(false);
            splitter->setStretchFactor(0, 3); splitter->setStretchFactor(1, 7);
            splitter->setSizes({360, 900});
            outer->addWidget(splitter, 1);
            std::vector<RobotModePreparationTestCase> cases;
            for (const auto& plan : InovanceModeSequence::Plans()) cases.push_back({plan.id, plan.name + "【未验证】"});
            RobotAdaptorAcceptanceLayout::PopulateModeOptions(controls.combo, "RobotC", cases);
            window.show();
            ApplyResponsivePageDefaults(&window);
            Pump();
            ConfigureResponsiveScrollArea(scroll);
            Pump();
            const QString name = QStringLiteral("acceptance-%1x%2-font%3.png").arg(size.width()).arg(size.height()).arg(fontSize);
            Check(window.grab().save(output.filePath(name)), "save initial render");
            std::cout << "CHECK " << size.width() << 'x' << size.height() << " font=" << fontSize
                << " combo=" << controls.combo->width() << 'x' << controls.combo->height()
                << " panel=" << controls.panel->width() << 'x' << controls.panel->height()
                << " hscroll=" << scroll->horizontalScrollBar()->maximum() << std::endl;
            Check(controls.combo->count() == 8 && !controls.combo->currentText().isEmpty(), "eight mode options not visible");
            Check(controls.combo->height() >= 40, "combo collapsed vertically");
            Check(controls.combo->font().pointSize() == fontSize, "test font did not reach the actual combo");
            Check(controls.combo->width() >= controls.panel->width() - 2, "compact defaults narrowed full-width selector");
            Check(scroll->horizontalScrollBar()->maximum() == 0, "detail panel has horizontal overflow");
            for (auto* button : controls.buttons)
            {
                Check(button->height() >= 44 && button->height() >= button->sizeHint().height(), "button height clipped");
                Check(motionPage->rect().contains(QRect(button->mapTo(motionPage, QPoint()), button->size())), "button clipped by stacked page");
                scroll->ensureWidgetVisible(button, 0, 0); Pump();
                Check(scroll->viewport()->rect().contains(QRect(button->mapTo(scroll->viewport(), QPoint()), button->size())), "button not reachable through scroll");
            }
            for (auto* button : jointButtons)
            {
                Check(button->height() >= button->sizeHint().height(), "joint button height clipped");
                Check(motionPage->rect().contains(QRect(button->mapTo(motionPage, QPoint()), button->size())), "joint button clipped by stacked page");
                scroll->ensureWidgetVisible(button, 0, 0); Pump();
                Check(scroll->viewport()->rect().contains(QRect(button->mapTo(scroll->viewport(), QPoint()), button->size())), "joint button not reachable through scroll");
            }
            Check(stack->geometry().bottom() < evidence->geometry().top(), "evidence overlaps parameter stack");
            Check(controls.buttons[0]->y() == controls.buttons[1]->y()
                && controls.buttons[2]->y() > controls.buttons[0]->y(), "mode buttons not arranged two by two");
            controls.combo->setCurrentIndex(6);
            cases[6].name += "【通过】";
            RobotAdaptorAcceptanceLayout::PopulateModeOptions(controls.combo, "RobotC", cases);
            Check(controls.combo->currentIndex() == 6 && controls.combo->currentText().contains(QStringLiteral("通过")), "refresh loses selection or text");
            for (int stage : {0, 3, 4, 10, 4}) { stack->setCurrentIndex(stage); Pump(); }
            scroll->ensureWidgetVisible(controls.panel, 0, 0); Pump();
            Check(window.grab().save(output.filePath(name)), "save render");
            RobotAdaptorAcceptanceLayout::PopulateModeOptions(controls.combo, "RobotA", {});
            Check(controls.combo->count() == 0 && !controls.combo->placeholderText().isEmpty(), "unsupported brand should display reason");
            RobotAdaptorAcceptanceLayout::PopulateModeOptions(controls.combo, "RobotC", cases);
            Check(controls.combo->count() == 8 && controls.combo->currentIndex() == 0, "robot switch fails to initialize selected text");
            ++tested;
            std::cout << "PASS " << size.width() << 'x' << size.height() << " font=" << fontSize
                << " combo=" << controls.combo->width() << 'x' << controls.combo->height()
                << " vscroll=" << scroll->verticalScrollBar()->maximum() << '\n';
        }
        std::cout << "PASS " << tested << " offline layout/render cases; no controller/database access\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}
