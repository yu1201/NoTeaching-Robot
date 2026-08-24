#pragma once

#include <QString>
#include <QStringList>
#include <QWidget>
#include <atomic>
#include <thread>

class QTreeWidget;
class QTreeWidgetItem;
class QComboBox;
class QCloseEvent;
class QLineEdit;
class QProgressBar;
class QLabel;
class QPushButton;

// 调试功能：把 Result/ 下的扫描结果按案例(机器人/案例目录)多选或按日期打包成 zip。
// 压缩在后台线程进行(QZipWriter 逐文件添加、按字节报进度)，不阻塞界面、可取消。
// 输出默认放 Result/Archives/result_<时间戳>.zip，也可自定义保存位置。
class ResultArchiveDialog : public QWidget
{
    Q_OBJECT

public:
    explicit ResultArchiveDialog(const QString& resultRootDir, QWidget* parent = nullptr);
    ~ResultArchiveDialog() override;
    bool IsRunning() const { return m_running.load(); }

signals:
    // 由后台线程发出、队列连接回主线程更新 UI。
    void progressChanged(int percent, const QString& currentEntry);
    void archiveFinished(bool ok, const QString& message);

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void OnItemChanged(QTreeWidgetItem* item, int column);
    void OnSelectByDate();
    void OnSelectAll();
    void OnSelectNone();
    void OnRefresh();
    void OnChooseOutput();
    void OnStart();
    void OnCancel();
    void OnProgress(int percent, const QString& entry);
    void OnFinished(bool ok, const QString& message);

private:
    void BuildTree();
    QStringList CollectSelectedCaseDirs() const;
    void SetBusy(bool busy);
    QString DefaultOutputPath() const;
    void JoinWorker();

    QString m_resultRoot;
    QTreeWidget* m_tree = nullptr;
    QComboBox* m_dateCombo = nullptr;
    QLineEdit* m_outputEdit = nullptr;
    QProgressBar* m_progress = nullptr;
    QLabel* m_status = nullptr;
    QPushButton* m_startBtn = nullptr;
    QPushButton* m_cancelBtn = nullptr;

    bool m_updatingChecks = false;        // OnItemChanged 级联时防重入
    std::thread m_worker;
    std::atomic<bool> m_cancelFlag{ false };
    std::atomic<bool> m_running{ false };
};
