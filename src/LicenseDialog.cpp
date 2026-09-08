#include "LicenseDialog.h"
#include "LicenseManager.h"
#ifndef HK_LICENSE_TEST_BUILD
#include "WindowStyleHelper.h"
#endif

#include <QFileDialog>
#include <QCloseEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>

QString LicenseDialog::ThemeStyleSheet()
{
    // Same palette and control treatment as the main AuthPage/AuthCard.
    // Font sizes inherit QApplication so enlarged operator fonts remain usable.
    return QStringLiteral(
        "QDialog { background: qlineargradient(x1:0,y1:0,x2:1,y2:1,"
        "stop:0 #0C121A,stop:0.55 #132033,stop:1 #0F1824); color:#ECF3F4; }"
        "QLabel { color:#AFC8CE; background:transparent; }"
        "QLabel#licenseTitle { color:#F5FAFA; }"
        "QFrame#licenseSummaryCard { background:#101923; border:1px solid #2E4256; border-radius:14px; }"
        "QLabel#licenseStatus { color:#ECF3F4; background:transparent; border:none; }"
        "QLineEdit { background:#0F1720; color:#F7FCFC; border:1px solid #2E4256;"
        "border-radius:11px; padding:10px 14px; selection-background-color:#2D5465; }"
        "QLineEdit:focus { border-color:#72D4DD; }"
        "QPushButton { background:#233645; color:#F5FAFA; border:1px solid #3C6173;"
        "border-radius:11px; padding:9px 14px; }"
        "QPushButton:hover { background:#2D5465; border-color:#72D4DD; }"
        "QPushButton:pressed { background:#18303B; }"
        "QPushButton:disabled { background:#18242F; color:#78909A; border-color:#2E4256; }"
        "QPushButton#licenseActivate { background:#67B5FF; color:#FFFFFF; border:none; font-weight:700; }"
        "QPushButton#licenseActivate:hover { background:#5AA8F3; }"
        "QPushButton#licenseActivate:pressed { background:#4B97EA; }"
        "QPushButton#licenseActivate:disabled { background:#29445D; color:#78909A; }"
        "QToolButton { background:transparent; color:#9ED8DB; border:none; padding:4px 0; }"
        "QToolButton:hover { color:#72D4DD; }"
        "QScrollArea#licenseDetailsPanel { background:#081018; border:1px solid #2C4653; border-radius:10px; }"
        "QScrollArea#licenseDetailsPanel QWidget { background:#081018; color:#BFE8EC; }"
        "QScrollBar:vertical { background:#101923; width:10px; margin:0; }"
        "QScrollBar::handle:vertical { background:#3C6173; border-radius:5px; min-height:20px; }"
        "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical { height:0; }"
        "QScrollBar:horizontal { background:#101923; height:10px; margin:0; }"
        "QScrollBar::handle:horizontal { background:#3C6173; border-radius:5px; min-width:20px; }"
        "QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal { width:0; }"
        "QScrollBar::add-page,QScrollBar::sub-page { background:#101923; }"
        "QProgressBar { background:#0F1720; color:#ECF3F4; border:1px solid #2E4256; border-radius:6px; }"
        "QProgressBar::chunk { background:#67B5FF; border-radius:5px; }");
}

LicenseDialog::LicenseDialog(QWidget* parent, std::function<void()> safetyRecovery,
    std::function<void()> safetyStop, std::function<bool()> safetyBusy, bool acknowledgeBlockedGate)
    : QDialog(parent), safetyBusy_(std::move(safetyBusy))
{
    setObjectName(QStringLiteral("LicenseDialog"));
    setProperty("_license_dialog", true);
    setStyleSheet(ThemeStyleSheet());
    auto& manager = LicenseManager::Instance();
    const bool initiallyBlocked = !manager.CanStartProtectedOperation();
    setWindowTitle(initiallyBlocked ? QStringLiteral("授权已过期 / 软件激活") : QStringLiteral("软件授权"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setMinimumSize(610, 440);
#ifndef HK_LICENSE_TEST_BUILD
    ApplyUnifiedWindowChrome(this);
#endif
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(28, 24, 28, 24);
    layout->setSpacing(12);
    title_ = new QLabel(this);
    title_->setObjectName(QStringLiteral("licenseTitle"));
    QFont titleFont = title_->font();
    titleFont.setPointSize(titleFont.pointSize() + 7);
    titleFont.setBold(true);
    title_->setFont(titleFont);
    layout->addWidget(title_);
    auto* explanation = new QLabel(initiallyBlocked
        ? QStringLiteral("激活或续期成功后才能进入软件。联网后会自动同步管理员设置的最新状态。")
        : QStringLiteral("查看授权期限、同步管理设置，或导入管理员签发的离线授权。"), this);
    explanation->setWordWrap(true);
    layout->addWidget(explanation);
    auto* summaryCard = new QFrame(this);
    summaryCard->setObjectName(QStringLiteral("licenseSummaryCard"));
    auto* summaryLayout = new QVBoxLayout(summaryCard);
    summaryLayout->setContentsMargins(18, 14, 18, 14);
    status_ = new QLabel(summaryCard);
    status_->setObjectName(QStringLiteral("licenseStatus"));
    status_->setWordWrap(true);
    status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    status_->setTextFormat(Qt::PlainText);
    status_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    summaryLayout->addWidget(status_);
    layout->addWidget(summaryCard);
    auto* detailsToggle = new QToolButton(this);
    detailsToggle->setObjectName(QStringLiteral("licenseDetailsToggle"));
    detailsToggle->setText(QStringLiteral("授权详情"));
    detailsToggle->setCheckable(true);
    detailsToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    detailsToggle->setArrowType(Qt::RightArrow);
    layout->addWidget(detailsToggle, 0, Qt::AlignLeft);
    auto* detailsPanel = new QScrollArea(this);
    detailsPanel->setObjectName(QStringLiteral("licenseDetailsPanel"));
    detailsPanel->setWidgetResizable(true);
    detailsPanel->setMinimumHeight(130);
    detailsPanel->setMaximumHeight(170);
    details_ = new QLabel(detailsPanel);
    details_->setObjectName(QStringLiteral("licenseDetails"));
    details_->setMargin(12);
    details_->setTextFormat(Qt::PlainText);
    details_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    details_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    detailsPanel->setWidget(details_);
    detailsPanel->hide();
    layout->addWidget(detailsPanel);
    connect(detailsToggle, &QToolButton::toggled, this, [this, detailsToggle, detailsPanel](bool expanded) {
        detailsToggle->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
        detailsPanel->setVisible(expanded);
        this->layout()->invalidate();
        this->layout()->activate();
        const QSize minimum = this->layout()->minimumSize();
        const QSize preferred = this->layout()->sizeHint();
        setMinimumSize(std::max(610, minimum.width()), std::max(440, minimum.height()));
        resize(std::max(width(), preferred.width()), std::max(minimumHeight(), preferred.height()));
    });
    layout->addStretch(1);
    code_ = new QLineEdit(this);
    code_->setObjectName(QStringLiteral("activationCode"));
    code_->setPlaceholderText(QStringLiteral("输入激活码（由管理员提供）"));
    code_->setMaxLength(256);
    layout->addWidget(code_);
    auto* onlineRow = new QHBoxLayout;
    auto* activate = new QPushButton(QStringLiteral("激活 / 续期"), this);
    activate->setObjectName(QStringLiteral("licenseActivate"));
    auto* trial = new QPushButton(QStringLiteral("申请 30 天试用"), this);
    auto* sync = new QPushButton(QStringLiteral("立即同步"), this);
    onlineRow->addWidget(activate);
    onlineRow->addWidget(trial);
    onlineRow->addWidget(sync);
    layout->addLayout(onlineRow);
    auto* offlineRow = new QHBoxLayout;
    auto* request = new QPushButton(QStringLiteral("导出离线申请"), this);
    auto* import = new QPushButton(QStringLiteral("导入离线授权"), this);
    auto* close = new QPushButton(initiallyBlocked ? QStringLiteral("退出软件") : QStringLiteral("关闭"), this);
    offlineRow->addWidget(request);
    offlineRow->addWidget(import);
    offlineRow->addStretch();
    offlineRow->addWidget(close);
    layout->addLayout(offlineRow);
    QPushButton* recoveryButton = nullptr;
    if (initiallyBlocked && (safetyRecovery || safetyStop)) {
        auto* safetyRow = new QHBoxLayout;
        auto* safetyNote = new QLabel(QStringLiteral("现场安全处理"), this);
        safetyRow->addWidget(safetyNote);
        safetyRow->addStretch();
        if (safetyRecovery) {
            recoveryButton = new QPushButton(QStringLiteral("安全回撤恢复"), this);
            safetyRow->addWidget(recoveryButton);
            connect(recoveryButton, &QPushButton::clicked, this, [this, safetyRecovery] {
                if (!safetyBusy_ || !safetyBusy_()) safetyRecovery();
            });
        }
        if (safetyStop) {
            auto* stopButton = new QPushButton(QStringLiteral("安全停止"), this);
            safetyRow->addWidget(stopButton);
            connect(stopButton, &QPushButton::clicked, this, safetyStop);
        }
        layout->addLayout(safetyRow);
    }
    connect(activate, &QPushButton::clicked, this, [this] {
        if (code_->text().trimmed().isEmpty()) {
            QMessageBox::information(this, QStringLiteral("软件激活"), QStringLiteral("请输入管理员提供的激活码。"));
            return;
        }
        LicenseManager::Instance().Register(code_->text());
        code_->clear();
        Refresh();
    });
    connect(trial, &QPushButton::clicked, this, [this] {
        const auto answer = QMessageBox::question(this, QStringLiteral("申请试用"),
            QStringLiteral("试用期从首次联网登记开始，持续 30 天；同一设备仅可申请一次。\n"
                "将上传机器标识摘要、设备名、软件版本和运行状态用于授权与设备管理。是否继续？"));
        if (answer == QMessageBox::Yes) LicenseManager::Instance().Register();
        Refresh();
    });
    connect(sync, &QPushButton::clicked, this, [this] {
        LicenseManager::Instance().SyncNow();
        Refresh();
    });
    connect(request, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出离线申请"),
            QStringLiteral("license-request.json"), QStringLiteral("授权申请 (*.json)"));
        if (path.isEmpty()) return;
        QString error;
        if (!LicenseManager::Instance().ExportOfflineRequest(path, &error))
            QMessageBox::warning(this, QStringLiteral("导出失败"), error);
        else QMessageBox::information(this, QStringLiteral("已导出"),
            QStringLiteral("请把申请文件交给管理员，并在本机导入管理员返回的授权文件。\n重新导出申请会使上一次申请失效。"));
        Refresh();
    });
    connect(import, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入离线授权"),
            QString(), QStringLiteral("离线授权 (*.json)"));
        if (path.isEmpty()) return;
        QString error;
        if (!LicenseManager::Instance().ImportOfflineResponse(path, &error))
            QMessageBox::warning(this, QStringLiteral("导入失败"), error);
        Refresh();
    });
    connect(close, &QPushButton::clicked, this, &QDialog::reject);
    const bool enabled = manager.Mode() != LicenseManager::LicenseMode::Off;
    auto* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this,
        [this, initiallyBlocked, acknowledgeBlockedGate, enabled, activate, trial, sync, request, import, close, recoveryButton] {
        const bool safetyBusy = safetyBusy_ && safetyBusy_();
        close->setEnabled(!safetyBusy);
        if (recoveryButton) recoveryButton->setEnabled(!safetyBusy);
        // A manually opened licensing page does not prove that robot work has
        // stopped. Only the startup/runtime gate may describe or acknowledge it.
        if (acknowledgeBlockedGate && initiallyBlocked
            && !LicenseManager::Instance().CanStartProtectedOperation()) {
            auto& license = LicenseManager::Instance();
            license.SetRuntimeState(safetyBusy ? QStringLiteral("safeRecovery") : QStringLiteral("idle"));
            license.SetEffectStatus(safetyBusy ? QStringLiteral("pendingSafeStop") : QStringLiteral("locked"));
            if (!safetyBusy) license.AcknowledgeAppliedPolicy();
        }
        const bool available = enabled && !LicenseManager::Instance().IsNetworkBusy();
        for (auto* button : {activate, trial, sync, request, import}) button->setEnabled(available);
        code_->setEnabled(available);
        Refresh(); // Show the actual acknowledgement made above, not the previous tick.
        if (initiallyBlocked && !safetyBusy && LicenseManager::Instance().CanStartProtectedOperation()) accept();
    });
    timer->start(250);
    for (auto* button : {activate, trial, sync, request, import})
        button->setEnabled(enabled && !manager.IsNetworkBusy());
    code_->setEnabled(enabled && !manager.IsNetworkBusy());
    Refresh();
}

void LicenseDialog::Refresh()
{
    auto& manager = LicenseManager::Instance();
    QString reason;
    manager.CanStartProtectedOperation(&reason);
    QString title = QStringLiteral("软件授权有效");
    if (manager.Mode() == LicenseManager::LicenseMode::Off) title = QStringLiteral("软件授权");
    else if (reason.contains(QStringLiteral("到期")) || reason.contains(QStringLiteral("结束")))
        title = QStringLiteral("软件已过期");
    else if (reason.contains(QStringLiteral("锁定")) || reason.contains(QStringLiteral("撤销")))
        title = QStringLiteral("软件已锁定");
    else if (reason.contains(QStringLiteral("未激活"))) title = QStringLiteral("请激活软件");
    else if (!reason.isEmpty()) title = QStringLiteral("授权暂不可用");
    title_->setText(title);
    setWindowTitle(title);

    const QString fullStatus = manager.StatusText();
    QStringList concise;
    if (manager.Mode() == LicenseManager::LicenseMode::Off)
        concise << QStringLiteral("此版本默认未启用授权管理。");
    else concise << (reason.isEmpty() ? QStringLiteral("授权有效，可正常使用。") : reason);
    bool afterInstallation = false;
    for (const QString& line : fullStatus.split('\n')) {
        if (line.startsWith(QStringLiteral("授权类型：")) || line.startsWith(QStringLiteral("到期时间：")))
            concise << line;
        if (afterInstallation && !line.isEmpty()) {
            if (line.startsWith(QStringLiteral("已验证管理端策略")))
                concise << QStringLiteral("已同步最新授权状态。");
            else if (line.startsWith(QStringLiteral("管理设置已在本机应用")))
                concise << QStringLiteral("已应用管理员设置。");
            else concise << line; // Network/verification errors stay directly visible.
        }
        if (line.startsWith(QStringLiteral("安装标识："))) afterInstallation = true;
    }
    status_->setText(concise.join('\n'));
    const int summaryWidth = std::max(200, width() - 96);
    status_->setMinimumHeight(status_->heightForWidth(summaryWidth));
    details_->setText(fullStatus);
}

void LicenseDialog::reject()
{
    if (safetyBusy_ && safetyBusy_()) return;
    QDialog::reject();
}

void LicenseDialog::closeEvent(QCloseEvent* event)
{
    if (safetyBusy_ && safetyBusy_()) {
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}
