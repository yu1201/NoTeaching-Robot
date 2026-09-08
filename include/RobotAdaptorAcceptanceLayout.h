#pragma once

#include "RobotModePreparationTest.h"
#include "WindowStyleHelper.h"
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <array>
#include <vector>

// UI-only helpers, shared with the offline geometry/render regression test.
// No robot, database, or network operation is performed here.
namespace RobotAdaptorAcceptanceLayout
{
struct ModeControls
{
    QWidget* panel = nullptr;
    QComboBox* combo = nullptr;
    std::array<QPushButton*, 4> buttons{};
};
inline ModeControls CreateModeControls(QWidget* parent)
{
    ModeControls controls;
    controls.panel = new QWidget(parent);
    controls.panel->setObjectName("AdaptorModeControls");
    auto* layout = new QVBoxLayout(controls.panel);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    layout->addWidget(new QLabel(QStringLiteral("无位移模式组合："), controls.panel));
    controls.combo = new QComboBox(controls.panel);
    controls.combo->setObjectName("AdaptorModeCombinationCombo");
    controls.combo->setProperty("_keep_wide_control", true);
    controls.combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    controls.combo->setMinimumContentsLength(25);
    controls.combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    controls.combo->setMinimumHeight(40);
    controls.combo->setStyleSheet(UnifiedComboBoxStyleSheet()
        + QStringLiteral("QComboBox { min-height:30px; }"));
    layout->addWidget(controls.combo);
    auto* buttons = new QGridLayout();
    buttons->setContentsMargins(0, 0, 0, 0);
    buttons->setSpacing(8);
    const std::array<QString, 4> texts = {QStringLiteral("测试所选组合"), QStringLiteral("一次测试全部组合"),
        QStringLiteral("选用已通过组合"), QStringLiteral("停止组合测试")};
    for (int index = 0; index < 4; ++index)
    {
        auto* button = new QPushButton(texts[index], controls.panel);
        button->setObjectName(QStringLiteral("AdaptorModeButton%1").arg(index));
        button->setMinimumSize(150, 44);
        button->setProperty("_keep_wide_control", true);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        buttons->addWidget(button, index / 2, index % 2);
        controls.buttons[index] = button;
    }
    buttons->setColumnStretch(0, 1);
    buttons->setColumnStretch(1, 1);
    layout->addLayout(buttons);
    return controls;
}

inline void PopulateModeOptions(QComboBox* combo, const QString& robot,
    const std::vector<RobotModePreparationTestCase>& cases)
{
    const QSignalBlocker blocker(combo);
    const bool sameRobot = combo->property("robotScope").toString() == robot;
    const QString selected = sameRobot ? combo->currentData().toString() : QString();
    bool rebuild = !sameRobot || combo->count() != static_cast<int>(cases.size());
    for (int index = 0; !rebuild && index < static_cast<int>(cases.size()); ++index)
    { rebuild = combo->itemData(index).toString() != QString::fromStdString(cases[index].id); }
    if (rebuild)
    {
        combo->clear();
        for (const auto& item : cases)
        { combo->addItem(QString::fromUtf8(item.name.c_str()), QString::fromStdString(item.id)); }
        combo->setProperty("robotScope", robot);
        const int selectedIndex = combo->findData(selected);
        combo->setCurrentIndex(selectedIndex >= 0 ? selectedIndex : (cases.empty() ? -1 : 0));
    }
    for (int index = 0; index < static_cast<int>(cases.size()); ++index)
    { combo->setItemText(index, QString::fromUtf8(cases[index].name.c_str())); }
    combo->setPlaceholderText(cases.empty() ? QStringLiteral("当前品牌未提供模式组合测试") : QString());
    if (!cases.empty() && combo->currentIndex() < 0) { combo->setCurrentIndex(0); }
}

inline QScrollArea* WrapDetailPanel(QWidget* parent, QWidget* content)
{
    auto* scroll = new QScrollArea(parent);
    scroll->setObjectName("AdaptorAcceptanceDetailScroll");
    scroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    scroll->setMinimumSize(0, 0);
    scroll->setWidget(content);
    // Preserve the content's layout minimum; viewport overflow scrolls instead
    // of forcing QStackedWidget to clip its child controls under the evidence.
    ConfigureResponsiveScrollArea(scroll);
    return scroll;
}
}
