#include "RobotModelManagerDialog.h"

#include "Const.h"
#include "RobotCadAssemblyLoader.h"
#include "RobotCollisionEnvelopeStore.h"
#include "TheoreticalRobotModelStore.h"
#include "WindowStyleHelper.h"

#include <Eigen/Core>

#include <QAbstractItemView>
#include <QCloseEvent>
#include <QDialogButtonBox>
#include <QEventLoop>
#include <QFileDialog>
#include <QColor>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>

namespace
{
constexpr char kModelId[] = "step.sa10-2000h";
constexpr char kAdapterId[] = "step.sa10-2000h.scene-v1";
constexpr char kVerifiedSa10AssemblySha256[] =
    "f8299f6deabc7b6f6ae799592f1f93e2366311d552bf91ac431eccec14bfcaa8";

class NonClosableProgressDialog final : public QProgressDialog
{
public:
    using QProgressDialog::QProgressDialog;

    void reject() override
    {
        // STEP 解析不能安全中断；Esc 不应放开父窗口或制造半完成状态。
    }

protected:
    void closeEvent(QCloseEvent* event) override
    {
        event->ignore();
    }
};

QString ShortSha(const QString& sha256)
{
    return sha256.size() == 64
        ? sha256.left(12) + QStringLiteral("…")
        : QStringLiteral("无效");
}

QString SizeText(qint64 bytes)
{
    if (bytes < 0) return QStringLiteral("未知");
    const double mib = static_cast<double>(bytes) / (1024.0 * 1024.0);
    return mib >= 0.1
        ? QStringLiteral("%1 MiB").arg(mib, 0, 'f', 2)
        : QStringLiteral("%1 字节").arg(bytes);
}

QString MarginText(qint64 micrometres)
{
    if (micrometres < 0) return QStringLiteral("未知");
    return QStringLiteral("%1 mm").arg(
        static_cast<double>(micrometres) / 1000.0, 0, 'f', 3);
}

QString RobotTypeText(int robotType)
{
    if (robotType == ROBOT_TYPE_STEP) return QStringLiteral("新时达 / STEP");
    if (robotType == ROBOT_TYPE_FANUC) return QStringLiteral("FANUC");
    return QStringLiteral("类型 %1").arg(robotType);
}

bool ValidateVerifiedSa10(
    const RobotCadAssemblyLoader::Result& loaded,
    QString& error)
{
    if (loaded.statistics.sourceSha256
        != QString::fromLatin1(kVerifiedSa10AssemblySha256))
    {
        error = QStringLiteral(
            "当前型号库只允许导入已核验的新时达 SA10-2000H 总装 STEP；"
            "不能把其它 J0-J6 模型登记为该型号。");
        return false;
    }
    if (!loaded.base.valid || !loaded.base.j0BoundsMm.valid
        || !loaded.statistics.assemblyBoundsMm.valid
        || (loaded.base.sourceUp - Eigen::Vector3d::UnitY()).norm() > 1.0e-12
        || loaded.statistics.jointComponentCount != 7
        || loaded.statistics.includedComponentCount != 7
        || loaded.statistics.includedPipelineCount != 0)
    {
        error = QStringLiteral(
            "已核验 SA10 总装的 J0-J6、源坐标方向、安装面或装配边界语义不完整。");
        return false;
    }
    std::array<bool, 7> joints{};
    for (const auto& component : loaded.statistics.components)
    {
        if (component.jointIndex < 0 || component.jointIndex > 6) continue;
        const size_t index = static_cast<size_t>(component.jointIndex);
        if (joints[index] || !component.included || !component.boundsMm.valid
            || component.faceCount <= 0)
        {
            error = QStringLiteral("机器人组件 J%1 重复或缺少有效 B-Rep 边界。")
                .arg(component.jointIndex);
            return false;
        }
        joints[index] = true;
    }
    for (int index = 0; index <= 6; ++index)
    {
        if (!joints[static_cast<size_t>(index)])
        {
            error = QStringLiteral("机器人总装缺少唯一有效的 J%1 组件。").arg(index);
            return false;
        }
    }
    if (loaded.statistics.detailedPresentationBuilt
        || loaded.statistics.displayTriangulationPrepared
        || loaded.statistics.displayBlockCount != 0
        || loaded.assemblyShape || loaded.j0Shape || !loaded.displayBlocks.empty())
    {
        error = QStringLiteral("型号库导入意外生成了详细机器人 B-Rep 显示资产。");
        return false;
    }
    error.clear();
    return true;
}

void ConfigureReadOnlyItem(QTableWidgetItem* item)
{
    if (item == nullptr) return;
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
}
}

RobotModelManagerDialog::RobotModelManagerDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("机器人模型库管理"));
    setModal(true);
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(1120, 720), 0.92, 0.90);

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(10);

    QLabel* boundary = new QLabel(QStringLiteral(
        "这里只管理经过验证的机器人型号资产，不绑定具体 RobotA/RobotB，也不修改控制单元。"
        "当前仅支持型号 step.sa10-2000h（新时达 SA10-2000H）；控制单元中的机器人型号是流程唯一选择来源。"));
    boundary->setWordWrap(true);
    boundary->setStyleSheet(QStringLiteral(
        "QLabel { background:#173247; color:#b3e5fc; border:1px solid #39789d; "
        "padding:8px; border-radius:4px; }"));
    root->addWidget(boundary);

    QHBoxLayout* actions = new QHBoxLayout();
    m_importButton = new QPushButton(QStringLiteral("导入 SA10 总装并生成简模"));
    m_refreshButton = new QPushButton(QStringLiteral("刷新型号库"));
    m_importButton->setObjectName(QStringLiteral("RobotModelCatalogImportButton"));
    m_refreshButton->setObjectName(QStringLiteral("RobotModelCatalogRefreshButton"));
    actions->addWidget(m_importButton);
    actions->addWidget(m_refreshButton);
    actions->addStretch(1);
    root->addLayout(actions);

    m_modelTable = new QTableWidget(0, 6);
    m_modelTable->setObjectName(QStringLiteral("RobotModelCatalogTable"));
    m_modelTable->setHorizontalHeaderLabels({
        QStringLiteral("型号ID"),
        QStringLiteral("显示名称"),
        QStringLiteral("驱动类型"),
        QStringLiteral("STEP SHA"),
        QStringLiteral("碰撞简模"),
        QStringLiteral("登记时间")
    });
    m_modelTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_modelTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_modelTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_modelTable->verticalHeader()->setVisible(false);
    m_modelTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_modelTable->horizontalHeader()->setStretchLastSection(true);
    m_modelTable->setMinimumHeight(260);
    root->addWidget(m_modelTable, 1);

    QGroupBox* detailsGroup = new QGroupBox(QStringLiteral("选中型号详情与完整性状态"));
    QVBoxLayout* detailsLayout = new QVBoxLayout(detailsGroup);
    m_detailsEdit = new QTextEdit();
    m_detailsEdit->setObjectName(QStringLiteral("RobotModelCatalogDetails"));
    m_detailsEdit->setReadOnly(true);
    m_detailsEdit->setMinimumHeight(150);
    m_detailsEdit->setPlaceholderText(QStringLiteral("型号库为空。"));
    detailsLayout->addWidget(m_detailsEdit);
    root->addWidget(detailsGroup);

    m_statusLabel = new QLabel(QStringLiteral("正在读取型号库…"));
    m_statusLabel->setObjectName(QStringLiteral("RobotModelCatalogStatus"));
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setMinimumHeight(36);
    root->addWidget(m_statusLabel);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    root->addWidget(buttons);

    connect(m_importButton, &QPushButton::clicked,
        this, [this]() { ImportVerifiedModel(); });
    connect(m_refreshButton, &QPushButton::clicked,
        this, [this]() { BootstrapLegacyCatalog(); });
    connect(m_modelTable, &QTableWidget::currentCellChanged,
        this, [this](int, int, int, int) { RefreshSelectionDetails(); });

    QTimer::singleShot(0, this, [this]() { BootstrapLegacyCatalog(); });
}

void RobotModelManagerDialog::reject()
{
    if (m_busy) return;
    QDialog::reject();
}

void RobotModelManagerDialog::SetBusy(bool busy)
{
    m_busy = busy;
    if (m_importButton != nullptr) m_importButton->setEnabled(!busy);
    if (m_refreshButton != nullptr) m_refreshButton->setEnabled(!busy);
    if (m_modelTable != nullptr) m_modelTable->setEnabled(!busy);
}

void RobotModelManagerDialog::SetStatus(const QString& text, bool error)
{
    if (m_statusLabel == nullptr) return;
    m_statusLabel->setText(text);
    m_statusLabel->setStyleSheet(error
        ? QStringLiteral("QLabel { color:#ff8a80; padding:5px; }")
        : QStringLiteral("QLabel { color:#a5d6a7; padding:5px; }"));
}

void RobotModelManagerDialog::BootstrapLegacyCatalog()
{
    if (m_busy) return;
    SetBusy(true);
    SetStatus(QStringLiteral("正在安全检查旧版当前 SA10 资产并刷新型号库…"));

    RobotModelCatalogStore::ModelRecord bootstrapModel;
    bool availableInCatalog = false;
    bool ok = false;
    QString workerError;
    QEventLoop waitLoop;
    QThread* worker = QThread::create([&]()
        {
            try
            {
                ok = RobotModelCatalogStore::BootstrapVerifiedSa10FromLegacy(
                    bootstrapModel, availableInCatalog, workerError);
            }
            catch (const std::exception& exception)
            {
                workerError = QStringLiteral("刷新机器人型号库线程发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
                ok = false;
            }
            catch (...)
            {
                workerError = QStringLiteral("刷新机器人型号库线程发生未知异常。");
                ok = false;
            }
        });
    NonClosableProgressDialog progress(
        QStringLiteral("正在检查旧版 SA10 资产并读取型号库…"),
        QString(), 0, 0, this);
    progress.setWindowTitle(QStringLiteral("刷新机器人型号库"));
    progress.setWindowModality(Qt::WindowModal);
    progress.setCancelButton(nullptr);
    progress.setMinimumDuration(300);
    progress.setAutoClose(false);
    progress.setAutoReset(false);
    progress.setWindowFlag(Qt::WindowCloseButtonHint, false);
    QTimer progressDelay;
    progressDelay.setSingleShot(true);
    connect(&progressDelay, &QTimer::timeout, &progress, &QProgressDialog::show);
    connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
    connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
    progressDelay.start(300);
    worker->start();
    waitLoop.exec();
    progressDelay.stop();
    worker->wait();
    delete worker;
    progress.accept();
    SetBusy(false);

    if (!ok)
    {
        m_models.clear();
        m_modelTable->setRowCount(0);
        m_detailsEdit->clear();
        SetStatus(QStringLiteral("机器人型号库不可用：%1").arg(workerError), true);
        QMessageBox::warning(this, QStringLiteral("刷新机器人型号库"), workerError);
        return;
    }
    RefreshCatalog(availableInCatalog ? bootstrapModel.modelId : QString());
    if (availableInCatalog)
    {
        SetStatus(QStringLiteral(
            "已确认旧版当前 SA10 资产安全登记为型号 %1；未绑定任何具体机器人。")
            .arg(bootstrapModel.modelId));
    }
}

void RobotModelManagerDialog::RefreshCatalog(const QString& preferredModelId)
{
    if (m_busy) return;
    QString previousId = preferredModelId;
    if (previousId.isEmpty() && m_modelTable->currentRow() >= 0)
    {
        QTableWidgetItem* currentIdItem =
            m_modelTable->item(m_modelTable->currentRow(), 0);
        if (currentIdItem != nullptr)
            previousId = currentIdItem->data(Qt::UserRole).toString();
    }

    QList<RobotModelCatalogStore::ModelRecord> models;
    QString error;
    if (!RobotModelCatalogStore::ListModels(models, error))
    {
        m_models.clear();
        m_modelTable->setRowCount(0);
        m_detailsEdit->clear();
        SetStatus(QStringLiteral("读取机器人型号库失败：%1").arg(error), true);
        return;
    }
    std::sort(models.begin(), models.end(),
        [](const auto& left, const auto& right)
        {
            const int displayOrder = QString::compare(
                left.displayName, right.displayName, Qt::CaseInsensitive);
            return displayOrder != 0 ? displayOrder < 0 : left.modelId < right.modelId;
        });
    m_models = models;
    m_modelTable->setRowCount(m_models.size());

    int selectedRow = -1;
    int unavailableCount = 0;
    QString firstUnavailableReason;
    for (qsizetype index = 0; index < m_models.size(); ++index)
    {
        const auto& model = m_models.at(index);
        const int row = static_cast<int>(index);
        RobotModelCatalogStore::Eligibility eligibility;
        QString eligibilityError;
        const bool eligibilityRead = RobotModelCatalogStore::ResolveModelEligibility(
            model.modelId, model.sourceRobotType, eligibility, eligibilityError);
        const bool available = eligibilityRead && eligibility.eligible;
        const QString state = available
            ? QStringLiteral("可用（%1）").arg(MarginText(model.collision.safetyMarginMicrometres))
            : eligibilityRead
                ? QStringLiteral("不可用：%1").arg(eligibility.reason)
                : QStringLiteral("校验失败：%1").arg(eligibilityError);
        if (!available)
        {
            ++unavailableCount;
            if (firstUnavailableReason.isEmpty()) firstUnavailableReason = state;
        }

        QList<QTableWidgetItem*> items = {
            new QTableWidgetItem(model.modelId),
            new QTableWidgetItem(model.displayName),
            new QTableWidgetItem(RobotTypeText(model.sourceRobotType)),
            new QTableWidgetItem(ShortSha(model.sourceStep.sha256)),
            new QTableWidgetItem(state),
            new QTableWidgetItem(model.registeredUtc)
        };
        items[0]->setData(Qt::UserRole, model.modelId);
        for (int column = 0; column < items.size(); ++column)
        {
            ConfigureReadOnlyItem(items.at(column));
            items.at(column)->setToolTip(column == 4 ? state : items.at(column)->text());
            if (column == 4)
                items.at(column)->setForeground(available ? QColor(102, 187, 106)
                                                         : QColor(239, 83, 80));
            m_modelTable->setItem(row, column, items.at(column));
        }
        if (model.modelId == previousId) selectedRow = row;
    }

    if (selectedRow < 0 && !m_models.isEmpty()) selectedRow = 0;
    if (selectedRow >= 0)
    {
        m_modelTable->selectRow(selectedRow);
        m_modelTable->setCurrentCell(selectedRow, 0);
    }
    else
    {
        m_detailsEdit->clear();
    }
    RefreshSelectionDetails();
    if (m_models.isEmpty())
    {
        SetStatus(QStringLiteral(
            "型号库为空。可导入已核验的 SA10-2000H 总装并生成简模。"));
    }
    else if (unavailableCount > 0)
    {
        SetStatus(QStringLiteral(
            "已读取 %1 个型号，其中 %2 个当前不可用；首个问题：%3")
            .arg(m_models.size()).arg(unavailableCount).arg(firstUnavailableReason), true);
    }
    else
    {
        SetStatus(QStringLiteral(
            "已读取 %1 个机器人型号；列表刷新不会解析大型 STEP。")
            .arg(m_models.size()));
    }
}

void RobotModelManagerDialog::RefreshSelectionDetails()
{
    if (m_detailsEdit == nullptr || m_modelTable == nullptr) return;
    const int row = m_modelTable->currentRow();
    if (row < 0 || row >= m_models.size())
    {
        m_detailsEdit->clear();
        return;
    }
    const auto& model = m_models.at(row);
    QString state = m_modelTable->item(row, 4) != nullptr
        ? m_modelTable->item(row, 4)->text() : QStringLiteral("未知");
    const QString text = QStringLiteral(
        "型号ID：%1\n"
        "显示名称：%2\n"
        "场景适配器：%3\n"
        "驱动类型：%4\n"
        "源文件：%5（%6）\n"
        "STEP SHA-256：%7\n"
        "碰撞简模 profile：%8\n"
        "碰撞 payload SHA-256：%9\n"
        "安全余量：%10\n"
        "登记时间：%11\n"
        "当前状态：%12\n\n"
        "说明：碰撞简模为 J0-J6 静态保守 AABB，未标定且不代表实时关节姿态。")
        .arg(model.modelId)
        .arg(model.displayName)
        .arg(model.adapterId)
        .arg(RobotTypeText(model.sourceRobotType))
        .arg(model.sourceStep.originalDisplayName)
        .arg(SizeText(model.sourceStep.sizeBytes))
        .arg(model.sourceStep.sha256)
        .arg(model.collision.profileKeySha256)
        .arg(model.collisionPayloadSha256)
        .arg(MarginText(model.collision.safetyMarginMicrometres))
        .arg(model.registeredUtc)
        .arg(state);
    m_detailsEdit->setPlainText(text);
}

void RobotModelManagerDialog::ImportVerifiedModel()
{
    if (m_busy) return;
    const QString sourcePath = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("导入新时达 SA10-2000H 总装 STEP"),
        QString(),
        QStringLiteral("STEP模型 (*.step *.stp)"));
    if (sourcePath.isEmpty()) return;
    if (QMessageBox::question(
            this,
            QStringLiteral("确认机器人型号"),
            QStringLiteral(
                "所选文件将按固定型号 step.sa10-2000h（新时达 SA10-2000H）严格校验。\n\n"
                "只有已核验总装的精确 SHA-256、J0-J6、毫米单位、坐标方向和安装面全部通过，"
                "才会在最后一步登记到型号库。继续吗？"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    SetBusy(true);
    SetStatus(QStringLiteral(
        "正在以 bounds-only 模式解析总装、生成 30 mm 碰撞简模并做登记前回读…"));

    RobotModelCatalogStore::ModelRecord registeredModel;
    QString workerError;
    bool workerOk = false;
    QEventLoop waitLoop;
    QThread* worker = QThread::create([&]()
        {
            try
            {
                RobotCadAssemblyLoader::Options options;
                options.buildDetailedPresentation = false;
                options.prepareDisplayTriangulation = false;
                RobotCadAssemblyLoader::Result loaded;
                TheoreticalRobotModelStore::Asset sourceAsset;
                RobotCollisionEnvelopeStore::EnvelopeSet envelope;
                RobotCollisionEnvelopeStore::StoredAsset collisionAsset;
                RobotCollisionEnvelopeStore::GenerationParameters generation;

                workerOk = RobotCadAssemblyLoader::LoadFile(
                        sourcePath, loaded, workerError, &options)
                    && ValidateVerifiedSa10(loaded, workerError)
                    && TheoreticalRobotModelStore::ImportStepFile(
                        sourcePath, sourceAsset, workerError, false);
                if (workerOk && sourceAsset.sha256 != loaded.statistics.sourceSha256)
                {
                    workerError = QStringLiteral(
                        "机器人 STEP 在语义解析和受控复制之间发生变化，未登记型号。");
                    workerOk = false;
                }
                if (workerOk)
                {
                    workerOk = RobotCollisionEnvelopeStore::Generate(
                            loaded, generation, envelope, workerError)
                        && RobotCollisionEnvelopeStore::Persist(
                            envelope, collisionAsset, workerError);
                    loaded = RobotCadAssemblyLoader::Result();
                }
                if (workerOk)
                {
                    workerOk = RobotModelCatalogStore::RegisterValidatedModel(
                        QString::fromLatin1(kModelId),
                        QStringLiteral("新时达 SA10-2000H"),
                        QString::fromLatin1(kAdapterId),
                        ROBOT_TYPE_STEP,
                        sourceAsset,
                        collisionAsset,
                        registeredModel,
                        workerError);
                }
            }
            catch (const std::exception& exception)
            {
                workerError = QStringLiteral("导入机器人型号线程发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
                workerOk = false;
            }
            catch (...)
            {
                workerError = QStringLiteral("导入机器人型号线程发生未知异常。");
                workerOk = false;
            }
        });

    NonClosableProgressDialog progress(
        QStringLiteral(
            "正在校验 SA10 总装并生成 J0-J6 碰撞简模；首次可能需要 1–2 分钟…"),
        QString(), 0, 0, this);
    progress.setWindowTitle(QStringLiteral("导入机器人型号"));
    progress.setWindowModality(Qt::WindowModal);
    progress.setCancelButton(nullptr);
    progress.setMinimumDuration(0);
    progress.setAutoClose(false);
    progress.setAutoReset(false);
    progress.setMinimumWidth(600);
    progress.setWindowFlag(Qt::WindowCloseButtonHint, false);
    connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
    connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
    progress.show();
    worker->start();
    waitLoop.exec();
    worker->wait();
    delete worker;
    progress.accept();
    SetBusy(false);

    if (!workerOk)
    {
        SetStatus(QStringLiteral("机器人型号导入失败，目录未发布新记录：%1")
            .arg(workerError), true);
        QMessageBox::warning(this, QStringLiteral("导入机器人型号"), workerError);
        return;
    }
    RefreshCatalog(registeredModel.modelId);
    SetStatus(QStringLiteral(
        "型号 %1 已完整回读并登记；未绑定或修改任何控制单元机器人。")
        .arg(registeredModel.modelId));
}
