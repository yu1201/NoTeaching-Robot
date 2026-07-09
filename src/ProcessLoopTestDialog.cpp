#include "ProcessLoopTestDialog.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDateTime>
#include <QVBoxLayout>

ProcessLoopTestDialog::ProcessLoopTestDialog(const QList<QPair<int, QString>>& units,
    int defaultUnitIndex,
    LoadDefaultsFunc loadDefaults,
    RunnerFunc runner,
    QWidget* parent)
    : QDialog(parent)
    , m_loadDefaults(std::move(loadDefaults))
    , m_runner(std::move(runner))
    , m_units(units)
{
    setWindowTitle(QStringLiteral("流程测试"));
    BuildUi();

    // 预选默认单元。
    if (m_unitCombo != nullptr)
    {
        int sel = 0;
        for (int i = 0; i < m_units.size(); ++i)
        {
            if (m_units.at(i).first == defaultUnitIndex) { sel = i; break; }
        }
        m_unitCombo->setCurrentIndex(sel);
    }
    ReloadDefaultsForCurrentUnit();
}

ProcessLoopTestDialog::~ProcessLoopTestDialog()
{
    m_stopRequested.store(true);
    JoinWorker();
}

void ProcessLoopTestDialog::BuildUi()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    auto* title = new QLabel(QStringLiteral("流程测试（自动循环先测后焊）"), this);
    title->setStyleSheet("font-size: 18px; font-weight: bold; color: #9ED8DB;");
    root->addWidget(title);

    auto* hint = new QLabel(QStringLiteral(
        "循环执行先测后焊流程（下枪安全位 → 扫描起点 → 扫描采集 → 收枪安全位 →〔可选〕焊接），用于重复性/稳定性验证。"
        "参数默认取「测量焊接参数」里已设好的预设值，勾选下方「覆盖」的项才用界面值替换。"
        "默认仅扫描；勾选「包含焊接段」才执行焊接。"), this);
    hint->setWordWrap(true);
    hint->setStyleSheet("color: #7E9AA6; font-size: 12px;");
    root->addWidget(hint);

    // —— 采集设置 ——
    auto* collectBox = new QGroupBox(QStringLiteral("测试设置"), this);
    auto* collectForm = new QFormLayout(collectBox);
    collectForm->setLabelAlignment(Qt::AlignRight);

    m_unitCombo = new QComboBox(collectBox);
    for (const auto& u : m_units)
    {
        m_unitCombo->addItem(QStringLiteral("%1（单元%2）").arg(u.second).arg(u.first), u.first);
    }
    connect(m_unitCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this](int) { ReloadDefaultsForCurrentUnit(); });
    collectForm->addRow(QStringLiteral("目标机器人："), m_unitCombo);

    auto* countRow = new QHBoxLayout();
    m_repeatSpin = new QSpinBox(collectBox);
    m_repeatSpin->setRange(1, 100000);
    m_repeatSpin->setValue(10);
    m_repeatSpin->setSuffix(QStringLiteral(" 次"));
    countRow->addWidget(m_repeatSpin);
    m_infiniteCheck = new QCheckBox(QStringLiteral("持续循环（手动停止）"), collectBox);
    connect(m_infiniteCheck, &QCheckBox::toggled, this, [this](bool on) { m_repeatSpin->setDisabled(on); });
    countRow->addWidget(m_infiniteCheck);
    countRow->addStretch();
    collectForm->addRow(QStringLiteral("循环次数："), countRow);

    m_stopOnFailureCheck = new QCheckBox(QStringLiteral("某次失败即停止（推荐：机器人异常时不硬跑）"), collectBox);
    m_stopOnFailureCheck->setChecked(true);
    collectForm->addRow(QString(), m_stopOnFailureCheck);

    m_doWeldCheck = new QCheckBox(QStringLiteral("包含焊接段（不勾=仅扫描；勾选后每次扫描完成即执行焊接，实际焊接/空跑按预设）"), collectBox);
    m_doWeldCheck->setChecked(false);
    collectForm->addRow(QString(), m_doWeldCheck);

    // 定时扫描：两次扫描之间的等待间隔。0=连续；配合「持续循环」= 每隔 X 分钟自动扫一次。
    auto* intervalRow = new QHBoxLayout();
    m_intervalSpin = new QSpinBox(collectBox);
    m_intervalSpin->setRange(0, 100000);
    m_intervalSpin->setValue(0);
    intervalRow->addWidget(m_intervalSpin);
    m_intervalUnitCombo = new QComboBox(collectBox);
    m_intervalUnitCombo->addItem(QStringLiteral("秒"), 1);
    m_intervalUnitCombo->addItem(QStringLiteral("分钟"), 60);
    intervalRow->addWidget(m_intervalUnitCombo);
    auto* intervalHint = new QLabel(QStringLiteral("（0=连续不等待；每次扫描完成后等待此时间再开始下一次）"), collectBox);
    intervalHint->setStyleSheet("color:#7E9AA6; font-size:11px;");
    intervalRow->addWidget(intervalHint);
    intervalRow->addStretch();
    collectForm->addRow(QStringLiteral("扫描间隔："), intervalRow);
    root->addWidget(collectBox);

    // —— 参数覆盖 ——
    auto* paramBox = new QGroupBox(QStringLiteral("参数（默认取先测后焊预设，勾选「覆盖」才改）"), this);
    auto* paramLayout = new QVBoxLayout(paramBox);

    m_defaultsLabel = new QLabel(QStringLiteral("预设：加载中…"), paramBox);
    m_defaultsLabel->setStyleSheet("color: #8FB0BC; font-size: 12px;");
    paramLayout->addWidget(m_defaultsLabel);

    auto addOverrideRow = [paramBox, paramLayout](const QString& label, QCheckBox*& check,
        QDoubleSpinBox*& spin, double minV, double maxV, int decimals, const QString& suffix)
        {
            auto* row = new QHBoxLayout();
            check = new QCheckBox(QStringLiteral("覆盖"), paramBox);
            row->addWidget(check);
            auto* lab = new QLabel(label, paramBox);
            lab->setMinimumWidth(120);
            row->addWidget(lab);
            spin = new QDoubleSpinBox(paramBox);
            spin->setRange(minV, maxV);
            spin->setDecimals(decimals);
            spin->setSuffix(suffix);
            spin->setEnabled(false);
            row->addWidget(spin);
            row->addStretch();
            QObject::connect(check, &QCheckBox::toggled, spin, &QWidget::setEnabled);
            paramLayout->addLayout(row);
        };
    addOverrideRow(QStringLiteral("扫描速度"), m_ovScanSpeedCheck, m_scanSpeedSpin, 1.0, 100000.0, 1, QStringLiteral(" mm/min"));
    addOverrideRow(QStringLiteral("运行速度"), m_ovRunSpeedCheck, m_runSpeedSpin, 1.0, 100000.0, 1, QStringLiteral(" mm/min"));
    addOverrideRow(QStringLiteral("相机时间偏移"), m_ovCameraOffsetCheck, m_cameraOffsetSpin, -10000.0, 10000.0, 1, QStringLiteral(" ms"));
    root->addWidget(paramBox);

    // —— 控制/进度 ——
    auto* ctrlRow = new QHBoxLayout();
    m_startBtn = new QPushButton(QStringLiteral("开始测试"), this);
    m_startBtn->setMinimumHeight(42);
    m_startBtn->setStyleSheet("QPushButton { background:#1F5A46; color:#EAFBF3; border:1px solid #3E8E70;"
        "border-radius:8px; font-size:15px; font-weight:600; padding:6px 18px; }"
        "QPushButton:hover { background:#2A7358; } QPushButton:disabled { background:#2A3A36; color:#6E8880; }");
    connect(m_startBtn, &QPushButton::clicked, this, &ProcessLoopTestDialog::OnStart);
    ctrlRow->addWidget(m_startBtn);

    m_stopBtn = new QPushButton(QStringLiteral("停止"), this);
    m_stopBtn->setMinimumHeight(42);
    m_stopBtn->setEnabled(false);
    m_stopBtn->setStyleSheet("QPushButton { background:#5A2323; color:#FBEAEA; border:1px solid #8E3E3E;"
        "border-radius:8px; font-size:15px; font-weight:600; padding:6px 18px; }"
        "QPushButton:hover { background:#733030; } QPushButton:disabled { background:#3A2A2A; color:#886E6E; }");
    connect(m_stopBtn, &QPushButton::clicked, this, &ProcessLoopTestDialog::OnStop);
    ctrlRow->addWidget(m_stopBtn);
    ctrlRow->addStretch();
    root->addLayout(ctrlRow);

    m_progressLabel = new QLabel(QStringLiteral("就绪。"), this);
    m_progressLabel->setStyleSheet("font-size: 14px; color: #D6E7EA; font-weight: 600;");
    root->addWidget(m_progressLabel);

    m_statLabel = new QLabel(QStringLiteral("成功 0 · 失败 0"), this);
    m_statLabel->setStyleSheet("color: #8FB0BC; font-size: 12px;");
    root->addWidget(m_statLabel);

    m_logText = new QPlainTextEdit(this);
    m_logText->setReadOnly(true);
    m_logText->setMinimumHeight(160);
    m_logText->setStyleSheet("QPlainTextEdit { background:#0E1A1F; color:#CFE3E6; border:1px solid #294049;"
        "border-radius:6px; font-family: Consolas, monospace; font-size: 12px; }");
    root->addWidget(m_logText, 1);

    resize(720, 640);
}

void ProcessLoopTestDialog::ReloadDefaultsForCurrentUnit()
{
    if (m_unitCombo == nullptr || !m_loadDefaults)
    {
        return;
    }
    const int unitIndex = m_unitCombo->currentData().toInt();
    const ProcessLoopTestDefaults d = m_loadDefaults(unitIndex);
    if (!d.ok)
    {
        m_defaultsLabel->setText(QStringLiteral("预设：读取失败（%1）。开始前请确认该单元已示教并设置好测量焊接参数。")
            .arg(d.issue.isEmpty() ? QStringLiteral("未知原因") : d.issue));
        return;
    }
    m_defaultsLabel->setText(QStringLiteral("预设「%1」：扫描速度 %2 mm/min · 运行速度 %3 mm/min · 相机偏移 %4 ms")
        .arg(d.paramGroupName.isEmpty() ? QStringLiteral("参数组") : d.paramGroupName)
        .arg(d.scanSpeedMmPerMin, 0, 'f', 1)
        .arg(d.runSpeedMmPerMin, 0, 'f', 1)
        .arg(d.cameraTimeOffsetMs, 0, 'f', 1));
    // 覆盖框未勾选时用预设值预填，方便用户在此基础上微调。
    if (m_scanSpeedSpin != nullptr && !m_ovScanSpeedCheck->isChecked()) { m_scanSpeedSpin->setValue(d.scanSpeedMmPerMin); }
    if (m_runSpeedSpin != nullptr && !m_ovRunSpeedCheck->isChecked()) { m_runSpeedSpin->setValue(d.runSpeedMmPerMin); }
    if (m_cameraOffsetSpin != nullptr && !m_ovCameraOffsetCheck->isChecked()) { m_cameraOffsetSpin->setValue(d.cameraTimeOffsetMs); }
}

ProcessLoopTestSettings ProcessLoopTestDialog::CollectSettings() const
{
    ProcessLoopTestSettings s;
    s.unitIndex = m_unitCombo != nullptr ? m_unitCombo->currentData().toInt() : -1;
    s.infinite = m_infiniteCheck != nullptr && m_infiniteCheck->isChecked();
    s.repeatCount = m_repeatSpin != nullptr ? m_repeatSpin->value() : 1;
    s.stopOnFailure = m_stopOnFailureCheck == nullptr || m_stopOnFailureCheck->isChecked();
    s.doWeld = m_doWeldCheck != nullptr && m_doWeldCheck->isChecked();
    const int intervalUnitMul = m_intervalUnitCombo != nullptr ? m_intervalUnitCombo->currentData().toInt() : 1;
    s.scanIntervalSeconds = (m_intervalSpin != nullptr ? m_intervalSpin->value() : 0) * intervalUnitMul;
    s.overrideScanSpeed = m_ovScanSpeedCheck != nullptr && m_ovScanSpeedCheck->isChecked();
    s.scanSpeedMmPerMin = m_scanSpeedSpin != nullptr ? m_scanSpeedSpin->value() : 0.0;
    s.overrideRunSpeed = m_ovRunSpeedCheck != nullptr && m_ovRunSpeedCheck->isChecked();
    s.runSpeedMmPerMin = m_runSpeedSpin != nullptr ? m_runSpeedSpin->value() : 0.0;
    s.overrideCameraOffset = m_ovCameraOffsetCheck != nullptr && m_ovCameraOffsetCheck->isChecked();
    s.cameraTimeOffsetMs = m_cameraOffsetSpin != nullptr ? m_cameraOffsetSpin->value() : 0.0;
    return s;
}

void ProcessLoopTestDialog::OnStart()
{
    if (m_running.load() || !m_runner)
    {
        return;
    }
    // 焊接段是物理动作、循环里会反复执行：开始前明确二次确认现场安全。
    if (m_doWeldCheck != nullptr && m_doWeldCheck->isChecked())
    {
        const auto ret = QMessageBox::warning(this, QStringLiteral("流程测试 · 包含焊接段"),
            QStringLiteral("已勾选「包含焊接段」：每次循环将在扫描后执行焊接动作"
                "（实际焊接或空跑按预设设置）。\n\n请确认现场安全、机器人工作范围内无人。\n\n确认开始？"),
            QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
        if (ret != QMessageBox::Ok)
        {
            return;
        }
    }
    JoinWorker();  // 回收上一轮已结束的线程

    const ProcessLoopTestSettings settings = CollectSettings();
    m_logText->clear();
    UpdateProgressUi(0, settings.infinite ? 0 : settings.repeatCount, 0, 0, QStringLiteral("准备中…"), false);
    SetRunningUi(true);
    m_stopRequested.store(false);
    m_running.store(true);

    // 进度/日志回调：后台线程调用 → 编列到 UI 线程执行。
    auto progressCb = [this](int it, int total, int ok, int fail, const QString& step, bool finished)
        {
            QMetaObject::invokeMethod(this, [this, it, total, ok, fail, step, finished]()
                { UpdateProgressUi(it, total, ok, fail, step, finished); }, Qt::QueuedConnection);
        };
    auto logCb = [this](const QString& text)
        {
            QMetaObject::invokeMethod(this, [this, text]() { AppendLog(text); }, Qt::QueuedConnection);
        };

    m_worker = std::thread([this, settings, progressCb, logCb]()
        {
            m_runner(settings, &m_stopRequested, progressCb, logCb);
            m_running.store(false);
        });
}

void ProcessLoopTestDialog::OnStop()
{
    if (!m_running.load())
    {
        return;
    }
    m_stopRequested.store(true);
    if (m_stopBtn != nullptr) { m_stopBtn->setEnabled(false); }
    AppendLog(QStringLiteral("已请求停止，将在当前步骤结束后停下…"));
}

void ProcessLoopTestDialog::JoinWorker()
{
    if (m_worker.joinable())
    {
        m_worker.join();
    }
}

void ProcessLoopTestDialog::SetRunningUi(bool running)
{
    if (m_startBtn != nullptr) { m_startBtn->setEnabled(!running); }
    if (m_stopBtn != nullptr) { m_stopBtn->setEnabled(running); }
    if (m_unitCombo != nullptr) { m_unitCombo->setEnabled(!running); }
    if (m_repeatSpin != nullptr) { m_repeatSpin->setEnabled(!running && !(m_infiniteCheck && m_infiniteCheck->isChecked())); }
    if (m_infiniteCheck != nullptr) { m_infiniteCheck->setEnabled(!running); }
    if (m_stopOnFailureCheck != nullptr) { m_stopOnFailureCheck->setEnabled(!running); }
    if (m_doWeldCheck != nullptr) { m_doWeldCheck->setEnabled(!running); }
    if (m_intervalSpin != nullptr) { m_intervalSpin->setEnabled(!running); }
    if (m_intervalUnitCombo != nullptr) { m_intervalUnitCombo->setEnabled(!running); }
    for (QCheckBox* c : { m_ovScanSpeedCheck, m_ovRunSpeedCheck, m_ovCameraOffsetCheck })
    {
        if (c != nullptr) { c->setEnabled(!running); }
    }
}

void ProcessLoopTestDialog::UpdateProgressUi(int iteration, int total, int okCount, int failCount,
    const QString& step, bool finished)
{
    if (m_progressLabel != nullptr)
    {
        const QString countText = total > 0
            ? QStringLiteral("第 %1 / %2 次").arg(iteration).arg(total)
            : QStringLiteral("第 %1 次（持续）").arg(iteration);
        m_progressLabel->setText(finished
            ? QStringLiteral("测试结束。%1").arg(step)
            : QStringLiteral("%1 · %2").arg(countText, step));
    }
    if (m_statLabel != nullptr)
    {
        m_statLabel->setText(QStringLiteral("成功 %1 · 失败 %2").arg(okCount).arg(failCount));
    }
    if (finished)
    {
        m_running.store(false);
        SetRunningUi(false);
    }
}

void ProcessLoopTestDialog::AppendLog(const QString& text)
{
    if (m_logText != nullptr)
    {
        m_logText->appendPlainText(QStringLiteral("[%1] %2")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss"), text));
    }
}

void ProcessLoopTestDialog::closeEvent(QCloseEvent* event)
{
    if (m_running.load())
    {
        // 安全优先：正在跑就先停下并等其收尾，避免留后台机器人运动。
        m_stopRequested.store(true);
        AppendLog(QStringLiteral("界面关闭：正在停止测试…"));
    }
    JoinWorker();
    QDialog::closeEvent(event);
}
