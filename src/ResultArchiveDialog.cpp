#include "ResultArchiveDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QComboBox>
#include <QLineEdit>
#include <QProgressBar>
#include <QLabel>
#include <QPushButton>
#include <QHeaderView>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <QSet>
#include <QCloseEvent>
#include <QDesktopServices>
#include <QUrl>

#include <QtCore/private/qzipwriter_p.h>

namespace
{
// 从案例目录名前缀解析 yyyyMMdd（结果目录命名 yyyyMMdd_NNN）。解析不出返回空串。
QString CaseDate(const QString& caseName)
{
    if (caseName.size() >= 8)
    {
        const QString head = caseName.left(8);
        bool ok = false;
        head.toLongLong(&ok);
        if (ok)
        {
            return head;
        }
    }
    return QString();
}

QString HumanSize(qint64 bytes)
{
    const double kb = 1024.0;
    if (bytes < kb) { return QString("%1 B").arg(bytes); }
    if (bytes < kb * kb) { return QString("%1 KB").arg(bytes / kb, 0, 'f', 1); }
    if (bytes < kb * kb * kb) { return QString("%1 MB").arg(bytes / (kb * kb), 0, 'f', 1); }
    return QString("%1 GB").arg(bytes / (kb * kb * kb), 0, 'f', 2);
}
}

ResultArchiveDialog::ResultArchiveDialog(const QString& resultRootDir, QWidget* parent)
    : QDialog(parent)
    , m_resultRoot(QDir(resultRootDir).absolutePath())
{
    setWindowTitle(QStringLiteral("结果打包压缩"));
    resize(720, 560);

    auto* root = new QVBoxLayout(this);

    // 顶部：按日期选择 + 全选/全不选/刷新
    auto* topRow = new QHBoxLayout();
    topRow->addWidget(new QLabel(QStringLiteral("按日期："), this));
    m_dateCombo = new QComboBox(this);
    m_dateCombo->setMinimumWidth(140);
    topRow->addWidget(m_dateCombo);
    auto* byDateBtn = new QPushButton(QStringLiteral("勾选该日期"), this);
    topRow->addWidget(byDateBtn);
    topRow->addStretch(1);
    auto* selAllBtn = new QPushButton(QStringLiteral("全选"), this);
    auto* selNoneBtn = new QPushButton(QStringLiteral("全不选"), this);
    auto* refreshBtn = new QPushButton(QStringLiteral("刷新"), this);
    topRow->addWidget(selAllBtn);
    topRow->addWidget(selNoneBtn);
    topRow->addWidget(refreshBtn);
    root->addLayout(topRow);

    // 中部：机器人 / 案例 勾选树
    m_tree = new QTreeWidget(this);
    m_tree->setColumnCount(2);
    m_tree->setHeaderLabels({ QStringLiteral("机器人 / 案例"), QStringLiteral("日期") });
    m_tree->header()->setStretchLastSection(false);
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    root->addWidget(m_tree, 1);

    // 输出位置
    auto* outRow = new QHBoxLayout();
    outRow->addWidget(new QLabel(QStringLiteral("保存为："), this));
    m_outputEdit = new QLineEdit(this);
    outRow->addWidget(m_outputEdit, 1);
    auto* chooseBtn = new QPushButton(QStringLiteral("自定义…"), this);
    outRow->addWidget(chooseBtn);
    root->addLayout(outRow);

    // 进度
    m_progress = new QProgressBar(this);
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    root->addWidget(m_progress);
    m_status = new QLabel(QStringLiteral("就绪。勾选要打包的案例后点「开始打包」。"), this);
    m_status->setWordWrap(true);
    root->addWidget(m_status);

    // 按钮
    auto* btnRow = new QHBoxLayout();
    btnRow->addStretch(1);
    m_startBtn = new QPushButton(QStringLiteral("开始打包"), this);
    m_cancelBtn = new QPushButton(QStringLiteral("取消"), this);
    m_cancelBtn->setEnabled(false);
    auto* closeBtn = new QPushButton(QStringLiteral("关闭"), this);
    btnRow->addWidget(m_startBtn);
    btnRow->addWidget(m_cancelBtn);
    btnRow->addWidget(closeBtn);
    root->addLayout(btnRow);

    m_outputEdit->setText(DefaultOutputPath());

    connect(m_tree, &QTreeWidget::itemChanged, this, &ResultArchiveDialog::OnItemChanged);
    connect(byDateBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnSelectByDate);
    connect(selAllBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnSelectAll);
    connect(selNoneBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnSelectNone);
    connect(refreshBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnRefresh);
    connect(chooseBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnChooseOutput);
    connect(m_startBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnStart);
    connect(m_cancelBtn, &QPushButton::clicked, this, &ResultArchiveDialog::OnCancel);
    connect(closeBtn, &QPushButton::clicked, this, &ResultArchiveDialog::close);

    // 后台线程发来的进度/结束信号，队列连接回主线程更新 UI。
    connect(this, &ResultArchiveDialog::progressChanged, this, &ResultArchiveDialog::OnProgress, Qt::QueuedConnection);
    connect(this, &ResultArchiveDialog::archiveFinished, this, &ResultArchiveDialog::OnFinished, Qt::QueuedConnection);

    BuildTree();
}

ResultArchiveDialog::~ResultArchiveDialog()
{
    m_cancelFlag.store(true);
    JoinWorker();
}

QString ResultArchiveDialog::DefaultOutputPath() const
{
    const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    return QDir(m_resultRoot).filePath(QStringLiteral("Archives/result_%1.zip").arg(ts));
}

void ResultArchiveDialog::BuildTree()
{
    m_updatingChecks = true;
    m_tree->clear();
    QSet<QString> dates;

    QDir resultDir(m_resultRoot);
    const QStringList robots = resultDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QString& robot : robots)
    {
        if (robot.compare(QStringLiteral("Archives"), Qt::CaseInsensitive) == 0)
        {
            continue;  // 输出归档目录自身不列入
        }
        QDir robotDir(resultDir.filePath(robot));
        const QStringList cases = robotDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
        if (cases.isEmpty())
        {
            continue;
        }

        auto* robotItem = new QTreeWidgetItem(m_tree);
        robotItem->setText(0, robot);
        robotItem->setFlags((robotItem->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsSelectable);
        robotItem->setCheckState(0, Qt::Unchecked);

        for (const QString& caseName : cases)
        {
            auto* caseItem = new QTreeWidgetItem(robotItem);
            caseItem->setText(0, caseName);
            const QString date = CaseDate(caseName);
            caseItem->setText(1, date);
            caseItem->setFlags(caseItem->flags() | Qt::ItemIsUserCheckable);
            caseItem->setCheckState(0, Qt::Unchecked);
            caseItem->setData(0, Qt::UserRole, robotDir.filePath(caseName));
            if (!date.isEmpty())
            {
                dates.insert(date);
            }
        }
        robotItem->setExpanded(false);
    }

    QStringList dateList = dates.values();
    dateList.sort();
    m_dateCombo->clear();
    m_dateCombo->addItems(dateList);

    m_updatingChecks = false;
    m_status->setText(QStringLiteral("共 %1 个机器人目录。勾选要打包的案例后点「开始打包」。")
        .arg(m_tree->topLevelItemCount()));
}

void ResultArchiveDialog::OnItemChanged(QTreeWidgetItem* item, int column)
{
    if (column != 0 || m_updatingChecks || item == nullptr)
    {
        return;
    }
    m_updatingChecks = true;
    if (item->childCount() > 0)
    {
        // 机器人节点：级联到所有案例
        const Qt::CheckState st = item->checkState(0);
        if (st != Qt::PartiallyChecked)
        {
            for (int i = 0; i < item->childCount(); ++i)
            {
                item->child(i)->setCheckState(0, st);
            }
        }
    }
    else if (item->parent() != nullptr)
    {
        // 案例节点：回算父节点三态
        QTreeWidgetItem* p = item->parent();
        int checked = 0;
        for (int i = 0; i < p->childCount(); ++i)
        {
            if (p->child(i)->checkState(0) == Qt::Checked) { ++checked; }
        }
        p->setCheckState(0, checked == 0 ? Qt::Unchecked
            : (checked == p->childCount() ? Qt::Checked : Qt::PartiallyChecked));
    }
    m_updatingChecks = false;
}

void ResultArchiveDialog::OnSelectByDate()
{
    const QString date = m_dateCombo->currentText().trimmed();
    if (date.isEmpty())
    {
        return;
    }
    m_updatingChecks = true;
    for (int r = 0; r < m_tree->topLevelItemCount(); ++r)
    {
        QTreeWidgetItem* robot = m_tree->topLevelItem(r);
        for (int c = 0; c < robot->childCount(); ++c)
        {
            QTreeWidgetItem* item = robot->child(c);
            if (CaseDate(item->text(0)) == date)
            {
                item->setCheckState(0, Qt::Checked);
                robot->setExpanded(true);
            }
        }
        int checked = 0;
        for (int c = 0; c < robot->childCount(); ++c)
        {
            if (robot->child(c)->checkState(0) == Qt::Checked) { ++checked; }
        }
        robot->setCheckState(0, checked == 0 ? Qt::Unchecked
            : (checked == robot->childCount() ? Qt::Checked : Qt::PartiallyChecked));
    }
    m_updatingChecks = false;
    m_status->setText(QStringLiteral("已勾选日期 %1 的全部案例。").arg(date));
}

void ResultArchiveDialog::OnSelectAll()
{
    m_updatingChecks = true;
    for (int r = 0; r < m_tree->topLevelItemCount(); ++r)
    {
        QTreeWidgetItem* robot = m_tree->topLevelItem(r);
        robot->setCheckState(0, Qt::Checked);
        for (int c = 0; c < robot->childCount(); ++c)
        {
            robot->child(c)->setCheckState(0, Qt::Checked);
        }
    }
    m_updatingChecks = false;
}

void ResultArchiveDialog::OnSelectNone()
{
    m_updatingChecks = true;
    for (int r = 0; r < m_tree->topLevelItemCount(); ++r)
    {
        QTreeWidgetItem* robot = m_tree->topLevelItem(r);
        robot->setCheckState(0, Qt::Unchecked);
        for (int c = 0; c < robot->childCount(); ++c)
        {
            robot->child(c)->setCheckState(0, Qt::Unchecked);
        }
    }
    m_updatingChecks = false;
}

void ResultArchiveDialog::OnRefresh()
{
    if (m_running.load())
    {
        return;
    }
    BuildTree();
}

QStringList ResultArchiveDialog::CollectSelectedCaseDirs() const
{
    QStringList dirs;
    for (int r = 0; r < m_tree->topLevelItemCount(); ++r)
    {
        QTreeWidgetItem* robot = m_tree->topLevelItem(r);
        for (int c = 0; c < robot->childCount(); ++c)
        {
            QTreeWidgetItem* item = robot->child(c);
            if (item->checkState(0) == Qt::Checked)
            {
                const QString path = item->data(0, Qt::UserRole).toString();
                if (!path.isEmpty())
                {
                    dirs << path;
                }
            }
        }
    }
    return dirs;
}

void ResultArchiveDialog::OnChooseOutput()
{
    const QString start = m_outputEdit->text().trimmed().isEmpty() ? DefaultOutputPath() : m_outputEdit->text().trimmed();
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("保存压缩包"), start,
        QStringLiteral("Zip 压缩包 (*.zip)"));
    if (!path.isEmpty())
    {
        QString p = path;
        if (!p.endsWith(QStringLiteral(".zip"), Qt::CaseInsensitive))
        {
            p += QStringLiteral(".zip");
        }
        m_outputEdit->setText(QDir::toNativeSeparators(p));
    }
}

void ResultArchiveDialog::SetBusy(bool busy)
{
    m_startBtn->setEnabled(!busy);
    m_cancelBtn->setEnabled(busy);
    m_tree->setEnabled(!busy);
    m_outputEdit->setEnabled(!busy);
}

void ResultArchiveDialog::JoinWorker()
{
    if (m_worker.joinable())
    {
        m_worker.join();
    }
}

void ResultArchiveDialog::OnStart()
{
    if (m_running.load())
    {
        return;
    }
    const QStringList cases = CollectSelectedCaseDirs();
    if (cases.isEmpty())
    {
        QMessageBox::information(this, windowTitle(), QStringLiteral("请先勾选要打包的案例。"));
        return;
    }
    QString out = m_outputEdit->text().trimmed();
    if (out.isEmpty())
    {
        out = DefaultOutputPath();
        m_outputEdit->setText(out);
    }
    const QString outDir = QFileInfo(out).absolutePath();
    if (!QDir().mkpath(outDir))
    {
        QMessageBox::warning(this, windowTitle(), QStringLiteral("无法创建输出目录：%1").arg(outDir));
        return;
    }
    if (QFileInfo::exists(out)
        && QMessageBox::question(this, windowTitle(),
            QStringLiteral("文件已存在，覆盖？\n%1").arg(out)) != QMessageBox::Yes)
    {
        return;
    }

    JoinWorker();  // 回收上一次(若有)
    m_cancelFlag.store(false);
    m_running.store(true);
    SetBusy(true);
    m_progress->setValue(0);
    m_status->setText(QStringLiteral("正在统计大小…"));

    const QString root = m_resultRoot;
    m_worker = std::thread([this, cases, out, root]()
    {
        // 1) 统计总字节(只 stat，不读内容)用于进度
        qint64 total = 0;
        for (const QString& caseDir : cases)
        {
            QDirIterator it(caseDir, QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
            while (it.hasNext()) { it.next(); total += it.fileInfo().size(); }
        }

        QZipWriter zw(out);
        zw.setCompressionPolicy(QZipWriter::AlwaysCompress);
        if (zw.status() != QZipWriter::NoError)
        {
            emit archiveFinished(false, QStringLiteral("无法创建压缩包(status=%1)。").arg(int(zw.status())));
            return;
        }

        const QDir rootDir(root);
        qint64 done = 0;
        bool canceled = false;
        for (const QString& caseDir : cases)
        {
            QDirIterator it(caseDir, QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
            while (it.hasNext())
            {
                if (m_cancelFlag.load()) { canceled = true; break; }
                it.next();
                const QFileInfo fi = it.fileInfo();
                const QString rel = rootDir.relativeFilePath(fi.absoluteFilePath());
                QFile f(fi.absoluteFilePath());
                if (f.open(QIODevice::ReadOnly))
                {
                    zw.addFile(rel, &f);
                    f.close();
                }
                done += fi.size();
                const int pct = total > 0 ? static_cast<int>(done * 100 / total) : 100;
                emit progressChanged(pct, rel);
            }
            if (canceled) { break; }
        }
        zw.close();

        if (canceled)
        {
            QFile::remove(out);
            emit archiveFinished(false, QStringLiteral("已取消，未完成的压缩包已删除。"));
            return;
        }
        if (zw.status() != QZipWriter::NoError)
        {
            emit archiveFinished(false, QStringLiteral("压缩过程出错(status=%1)。").arg(int(zw.status())));
            return;
        }
        emit archiveFinished(true, out);
    });
}

void ResultArchiveDialog::OnCancel()
{
    if (m_running.load())
    {
        m_cancelFlag.store(true);
        m_status->setText(QStringLiteral("正在取消…"));
        m_cancelBtn->setEnabled(false);
    }
}

void ResultArchiveDialog::OnProgress(int percent, const QString& entry)
{
    m_progress->setValue(percent);
    m_status->setText(QStringLiteral("正在压缩(%1%)：%2").arg(percent).arg(entry));
}

void ResultArchiveDialog::OnFinished(bool ok, const QString& message)
{
    JoinWorker();
    m_running.store(false);
    SetBusy(false);
    if (ok)
    {
        m_progress->setValue(100);
        const QFileInfo fi(message);
        m_status->setText(QStringLiteral("打包完成：%1（%2）").arg(QDir::toNativeSeparators(message), HumanSize(fi.size())));
        if (QMessageBox::information(this, windowTitle(),
            QStringLiteral("打包完成：\n%1\n（%2）\n\n是否打开所在文件夹？").arg(QDir::toNativeSeparators(message), HumanSize(fi.size())),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) == QMessageBox::Yes)
        {
            QDesktopServices::openUrl(QUrl::fromLocalFile(fi.absolutePath()));
        }
    }
    else
    {
        m_status->setText(message);
        QMessageBox::warning(this, windowTitle(), message);
    }
}

void ResultArchiveDialog::closeEvent(QCloseEvent* event)
{
    if (m_running.load())
    {
        if (QMessageBox::question(this, windowTitle(),
            QStringLiteral("正在打包，确定取消并关闭？")) != QMessageBox::Yes)
        {
            event->ignore();
            return;
        }
        m_cancelFlag.store(true);
        JoinWorker();
    }
    QDialog::closeEvent(event);
}
