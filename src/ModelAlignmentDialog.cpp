#include "ModelAlignmentDialog.h"

#include "AppPaths.h"
#include "BcpdModelAligner.h"
#include "PointCloud3DView.h"
#include "PointCloudModelDenoiser.h"
#include "ReferenceModelLibrary.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <exception>
#include <new>
#include <thread>

ModelAlignmentDialog::ModelAlignmentDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("模型配准 / 点云去噪"));
    resize(1120, 780);
    BuildUi();
    ApplyStyle();
    RefreshModelList();
    RefreshGroupList(ModelAlignmentConfig::CurrentGroupIndex());
}

ModelAlignmentDialog::~ModelAlignmentDialog()
{
    m_destroying.store(true, std::memory_order_release);
    m_stopRequested.store(true, std::memory_order_release);
    if (m_worker.joinable())
    {
        m_worker.join();
    }
}

void ModelAlignmentDialog::BuildUi()
{
    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    // —— 基准模型行 ——
    QHBoxLayout* modelRow = new QHBoxLayout();
    modelRow->setSpacing(6);
    modelRow->addWidget(new QLabel(QStringLiteral("基准模型：")));
    m_pModelCombo = new QComboBox();
    m_pModelCombo->setMinimumWidth(220);
    QPushButton* importResultBtn = new QPushButton(QStringLiteral("从扫描结果导入"));
    importResultBtn->setObjectName(QStringLiteral("PrimaryButton"));
    importResultBtn->setToolTip(QStringLiteral("选一个干净工件的扫描结果目录，把它的网格模型存入基准库。"));
    QPushButton* importFileBtn = new QPushButton(QStringLiteral("导入 .ply"));
    QPushButton* renameModelBtn = new QPushButton(QStringLiteral("重命名"));
    QPushButton* deleteModelBtn = new QPushButton(QStringLiteral("删除"));
    deleteModelBtn->setObjectName(QStringLiteral("DangerButton"));
    modelRow->addWidget(m_pModelCombo, 1);
    modelRow->addWidget(importResultBtn);
    modelRow->addWidget(importFileBtn);
    modelRow->addWidget(renameModelBtn);
    modelRow->addWidget(deleteModelBtn);
    root->addLayout(modelRow);

    // —— 待处理点云行 ——
    QHBoxLayout* cloudRow = new QHBoxLayout();
    cloudRow->setSpacing(6);
    cloudRow->addWidget(new QLabel(QStringLiteral("待处理点云：")));
    m_pCloudEdit = new QLineEdit();
    m_pCloudEdit->setReadOnly(true);
    m_pCloudEdit->setPlaceholderText(QStringLiteral("选择有噪点云文件（index x y z 文本）…"));
    QPushButton* browseCloudBtn = new QPushButton(QStringLiteral("选择点云…"));
    cloudRow->addWidget(m_pCloudEdit, 1);
    cloudRow->addWidget(browseCloudBtn);
    root->addLayout(cloudRow);

    // —— 参数组行 ——
    QHBoxLayout* groupRow = new QHBoxLayout();
    groupRow->setSpacing(6);
    groupRow->addWidget(new QLabel(QStringLiteral("参数组：")));
    m_pGroupCombo = new QComboBox();
    m_pGroupCombo->setMinimumWidth(180);
    QPushButton* saveGroupBtn = new QPushButton(QStringLiteral("保存"));
    QPushButton* saveAsGroupBtn = new QPushButton(QStringLiteral("另存为组"));
    QPushButton* deleteGroupBtn = new QPushButton(QStringLiteral("删除组"));
    deleteGroupBtn->setObjectName(QStringLiteral("DangerButton"));
    groupRow->addWidget(m_pGroupCombo, 1);
    groupRow->addWidget(saveGroupBtn);
    groupRow->addWidget(saveAsGroupBtn);
    groupRow->addWidget(deleteGroupBtn);
    root->addLayout(groupRow);

    // —— 参数区（左：BCPD 配准；右：去噪）——
    QHBoxLayout* paramRow = new QHBoxLayout();
    paramRow->setSpacing(12);

    QGroupBox* bcpdBox = new QGroupBox(QStringLiteral("BCPD 非刚性配准"));
    QFormLayout* bcpdForm = new QFormLayout(bcpdBox);
    m_pOmega = new QDoubleSpinBox();
    m_pOmega->setRange(0.001, 0.9);
    m_pOmega->setSingleStep(0.01);
    m_pOmega->setDecimals(3);
    m_pOmega->setToolTip(QStringLiteral("-w 外点比例：点云噪声多则调大"));
    m_pLambda = new QDoubleSpinBox();
    m_pLambda->setRange(0.0, 100.0);
    m_pLambda->setSingleStep(0.5);
    m_pLambda->setToolTip(QStringLiteral("-l 位移期望长度正则；0 = bcpd 默认"));
    m_pBeta = new QDoubleSpinBox();
    m_pBeta->setRange(0.0, 100.0);
    m_pBeta->setSingleStep(0.5);
    m_pBeta->setToolTip(QStringLiteral("-b 平滑核宽：越大形变越平滑、越不会拟合噪声；0 = bcpd 默认"));
    m_pSampleCap = new QSpinBox();
    m_pSampleCap->setRange(0, 500000);
    m_pSampleCap->setSingleStep(1000);
    m_pSampleCap->setToolTip(QStringLiteral("配准时点云降采样上限（0 = 不降）；去噪仍用全分辨率"));
    m_pAccel = new QCheckBox(QStringLiteral("启用 -A 加速（模型顶点 > 70 时生效）"));
    bcpdForm->addRow(QStringLiteral("外点比例 ω"), m_pOmega);
    bcpdForm->addRow(QStringLiteral("位移正则 λ"), m_pLambda);
    bcpdForm->addRow(QStringLiteral("平滑核宽 β"), m_pBeta);
    bcpdForm->addRow(QStringLiteral("点云降采样上限"), m_pSampleCap);
    bcpdForm->addRow(QString(), m_pAccel);

    QGroupBox* denoiseBox = new QGroupBox(QStringLiteral("点到模型去噪"));
    QFormLayout* denoiseForm = new QFormLayout(denoiseBox);
    m_pDistThresh = new QDoubleSpinBox();
    m_pDistThresh->setRange(0.05, 100.0);
    m_pDistThresh->setSingleStep(0.5);
    m_pDistThresh->setDecimals(2);
    m_pDistThresh->setToolTip(QStringLiteral("点到形变后模型表面距离 > 此阈值判为噪声剔除"));
    m_pKeepNoSurface = new QCheckBox(QStringLiteral("模型未覆盖区域的点保留（保守，避免误删）"));
    denoiseForm->addRow(
        QStringLiteral("距离阈值"),
        CreateExternalUnitEditor(m_pDistThresh, QStringLiteral("mm"), denoiseBox));
    denoiseForm->addRow(QString(), m_pKeepNoSurface);

    paramRow->addWidget(bcpdBox, 1);
    paramRow->addWidget(denoiseBox, 1);
    root->addLayout(paramRow);

    // —— 运行行 ——
    QHBoxLayout* runRow = new QHBoxLayout();
    runRow->setSpacing(8);
    m_pRunBtn = new QPushButton(QStringLiteral("运行：配准 + 去噪"));
    m_pRunBtn->setObjectName(QStringLiteral("PrimaryButton"));
    m_pRunBtn->setMinimumHeight(34);
    m_pSaveResultBtn = new QPushButton(QStringLiteral("保存去噪点云…"));
    m_pSaveResultBtn->setEnabled(false);
    runRow->addWidget(m_pRunBtn, 1);
    runRow->addWidget(m_pSaveResultBtn);
    root->addLayout(runRow);

    // —— 预览 ——
    m_pPreview = new pcview::PointCloud3DView(this);
    root->addWidget(m_pPreview, 1);

    m_pInfoLabel = new QLabel(QStringLiteral("选基准模型 + 待处理点云 + 参数组，点「运行」做模型引导去噪。"));
    m_pInfoLabel->setWordWrap(true);
    root->addWidget(m_pInfoLabel);

    // —— 连接 ——
    connect(importResultBtn, &QPushButton::clicked, this, [this]() { OnImportModelFromResult(); });
    connect(importFileBtn, &QPushButton::clicked, this, [this]() { OnImportModelFromFile(); });
    connect(renameModelBtn, &QPushButton::clicked, this, [this]() { OnRenameModel(); });
    connect(deleteModelBtn, &QPushButton::clicked, this, [this]() { OnDeleteModel(); });
    connect(browseCloudBtn, &QPushButton::clicked, this, [this]() { OnBrowseCloud(); });
    connect(saveGroupBtn, &QPushButton::clicked, this, [this]() { OnSaveGroup(); });
    connect(saveAsGroupBtn, &QPushButton::clicked, this, [this]() { OnSaveAsGroup(); });
    connect(deleteGroupBtn, &QPushButton::clicked, this, [this]() { OnDeleteGroup(); });
    connect(m_pGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int index) { OnGroupChanged(index); });
    connect(m_pRunBtn, &QPushButton::clicked, this, [this]() { OnRun(); });
    connect(m_pSaveResultBtn, &QPushButton::clicked, this, [this]() { OnSaveResult(); });
}

void ModelAlignmentDialog::ApplyStyle()
{
    setStyleSheet(QString(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QLabel { color: #B8C7CC; }"
        "QGroupBox { color: #9ED8DB; border: 1px solid #2E4656; border-radius: 8px; margin-top: 10px; padding: 8px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
        "QLineEdit { background: #0D161D; color: #9ED8DB; border: 1px solid #2E4656; border-radius: 8px; padding: 6px 10px; }"
        "QComboBox, QSpinBox, QDoubleSpinBox { background: #0D161D; color: #ECF3F4; border: 1px solid #2E4656; border-radius: 6px; padding: 4px 8px; }"
        "QComboBox QAbstractItemView { background: #0D161D; color: #ECF3F4; selection-background-color: #2D5465; }"
        "QCheckBox { color: #B8C7CC; }"
        "QPushButton { background: #1B2B36; color: #D9E8EA; border: 1px solid #31505F; border-radius: 8px; padding: 7px 14px; min-height: 22px; }"
        "QPushButton:hover { background: #24465A; border-color: #63C7D1; }"
        "QPushButton:pressed { background: #152530; }"
        "QPushButton:disabled { background: #141F27; color: #546770; border-color: #243640; }"
        "QPushButton#PrimaryButton { border-color: #3F7E8C; color: #A8E8EE; }"
        "QPushButton#PrimaryButton:hover { background: #1F5160; border-color: #79D6DE; }"
        "QPushButton#DangerButton { border-color: #6B3642; color: #F0B9C2; }"
        "QPushButton#DangerButton:hover { background: #54222E; border-color: #E07A8A; }"));
}

void ModelAlignmentDialog::Info(const QString& text)
{
    if (m_pInfoLabel) m_pInfoLabel->setText(text);
}

// ===================== 模型库 =====================

void ModelAlignmentDialog::RefreshModelList(const QString& selectName)
{
    const QString keep = selectName.isEmpty() ? m_pModelCombo->currentText() : selectName;
    m_pModelCombo->blockSignals(true);
    m_pModelCombo->clear();
    m_pModelCombo->addItems(ReferenceModelLibrary::ListModels());
    const int idx = m_pModelCombo->findText(keep);
    if (idx >= 0) m_pModelCombo->setCurrentIndex(idx);
    m_pModelCombo->blockSignals(false);
}

void ModelAlignmentDialog::OnImportModelFromResult()
{
    const QString startDir = AppPaths::WritablePath(QStringLiteral("Result"));
    const QString dir = QFileDialog::getExistingDirectory(this, QStringLiteral("选择干净工件的扫描结果目录"), startDir);
    if (dir.isEmpty()) return;

    // 解析 LaserPoint 目录与缓存路径，缓存无效则现场生成。
    QString laserDir = dir;
    if (!QFileInfo::exists(WorkpieceMeshBuilder::MeshCachePath(laserDir))
        && QFileInfo::exists(QDir(dir).filePath(QStringLiteral("LaserPoint"))))
    {
        laserDir = QDir(dir).filePath(QStringLiteral("LaserPoint"));
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    WorkpieceMeshBuilder::Mesh mesh;
    QString error;
    bool ok = false;
    if (QFileInfo::exists(WorkpieceMeshBuilder::MeshCachePath(laserDir)))
    {
        ok = WorkpieceMeshBuilder::LoadMeshPly(WorkpieceMeshBuilder::MeshCachePath(laserDir), mesh, error);
    }
    else
    {
        const QString cloudPath = WorkpieceMeshBuilder::ResolveCloudSourcePath(laserDir);
        if (WorkpieceMeshBuilder::EnsureMeshCache(laserDir, cloudPath, error))
            ok = WorkpieceMeshBuilder::LoadMeshPly(WorkpieceMeshBuilder::MeshCachePath(laserDir), mesh, error);
    }
    QApplication::restoreOverrideCursor();
    if (!ok || !mesh.IsValid())
    {
        QMessageBox::warning(this, QStringLiteral("导入基准模型"), QStringLiteral("该目录未能得到有效网格：%1").arg(error));
        return;
    }

    bool inputOk = false;
    const QString name = QInputDialog::getText(this, QStringLiteral("命名基准模型"),
        QStringLiteral("基准模型名称："), QLineEdit::Normal, QFileInfo(dir).fileName(), &inputOk).trimmed();
    if (!inputOk || name.isEmpty()) return;
    if (ReferenceModelLibrary::Exists(name)
        && QMessageBox::question(this, QStringLiteral("覆盖"), QStringLiteral("已存在同名基准，覆盖？")) != QMessageBox::Yes)
        return;

    QString err2;
    if (!ReferenceModelLibrary::ImportFromMesh(name, mesh, err2))
    {
        QMessageBox::warning(this, QStringLiteral("导入基准模型"), err2);
        return;
    }
    RefreshModelList(name);
    Info(QStringLiteral("已导入基准模型「%1」（顶点 %2、三角形 %3）").arg(name).arg(mesh.vertices.size()).arg(mesh.indices.size() / 3));
}

void ModelAlignmentDialog::OnImportModelFromFile()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("选择 .ply 网格文件"),
        QString(), QStringLiteral("网格 (*.ply);;所有文件 (*)"));
    if (path.isEmpty()) return;
    bool inputOk = false;
    const QString name = QInputDialog::getText(this, QStringLiteral("命名基准模型"),
        QStringLiteral("基准模型名称："), QLineEdit::Normal, QFileInfo(path).completeBaseName(), &inputOk).trimmed();
    if (!inputOk || name.isEmpty()) return;
    QString err;
    if (!ReferenceModelLibrary::ImportFromFile(name, path, err))
    {
        QMessageBox::warning(this, QStringLiteral("导入基准模型"), err);
        return;
    }
    RefreshModelList(name);
    Info(QStringLiteral("已从文件导入基准模型「%1」").arg(name));
}

void ModelAlignmentDialog::OnRenameModel()
{
    const QString from = m_pModelCombo->currentText();
    if (from.isEmpty()) return;
    bool inputOk = false;
    const QString to = QInputDialog::getText(this, QStringLiteral("重命名基准模型"),
        QStringLiteral("新名称："), QLineEdit::Normal, from, &inputOk).trimmed();
    if (!inputOk || to.isEmpty() || to == from) return;
    QString err;
    if (!ReferenceModelLibrary::RenameModel(from, to, err))
    {
        QMessageBox::warning(this, QStringLiteral("重命名"), err);
        return;
    }
    RefreshModelList(to);
}

void ModelAlignmentDialog::OnDeleteModel()
{
    const QString name = m_pModelCombo->currentText();
    if (name.isEmpty()) return;
    if (QMessageBox::question(this, QStringLiteral("删除基准模型"),
        QStringLiteral("确认删除基准模型「%1」？").arg(name)) != QMessageBox::Yes)
        return;
    QString err;
    if (!ReferenceModelLibrary::DeleteModel(name, err))
    {
        QMessageBox::warning(this, QStringLiteral("删除"), err);
        return;
    }
    RefreshModelList();
}

// ===================== 参数组 =====================

void ModelAlignmentDialog::RefreshGroupList(int selectIndex)
{
    const int count = ModelAlignmentConfig::GroupCount();
    m_pGroupCombo->blockSignals(true);
    m_pGroupCombo->clear();
    for (int i = 0; i < count; ++i) m_pGroupCombo->addItem(ModelAlignmentConfig::GroupName(i));
    int idx = selectIndex < 0 ? ModelAlignmentConfig::CurrentGroupIndex() : selectIndex;
    idx = qBound(0, idx, count - 1);
    m_pGroupCombo->setCurrentIndex(idx);
    m_pGroupCombo->blockSignals(false);
    ModelAlignmentConfig::SetCurrentGroupIndex(idx);
    LoadParamsToUi(ModelAlignmentConfig::LoadGroup(idx));
}

void ModelAlignmentDialog::LoadParamsToUi(const ModelAlignmentParams& p)
{
    if (!p.referenceModel.isEmpty())
    {
        const int mi = m_pModelCombo->findText(p.referenceModel);
        if (mi >= 0) m_pModelCombo->setCurrentIndex(mi);
    }
    m_pOmega->setValue(p.omega);
    m_pLambda->setValue(p.lambda);
    m_pBeta->setValue(p.beta);
    m_pSampleCap->setValue(p.targetSampleCap);
    m_pAccel->setChecked(p.useAcceleration);
    m_pDistThresh->setValue(p.distanceThresholdMm);
    m_pKeepNoSurface->setChecked(p.keepPointsWithoutSurface);
}

ModelAlignmentParams ModelAlignmentDialog::ParamsFromUi() const
{
    ModelAlignmentParams p;
    p.referenceModel = m_pModelCombo->currentText();
    p.omega = m_pOmega->value();
    p.lambda = m_pLambda->value();
    p.beta = m_pBeta->value();
    p.targetSampleCap = m_pSampleCap->value();
    p.useAcceleration = m_pAccel->isChecked();
    p.distanceThresholdMm = m_pDistThresh->value();
    p.keepPointsWithoutSurface = m_pKeepNoSurface->isChecked();
    return p;
}

void ModelAlignmentDialog::OnGroupChanged(int index)
{
    if (index < 0) return;
    ModelAlignmentConfig::SetCurrentGroupIndex(index);
    LoadParamsToUi(ModelAlignmentConfig::LoadGroup(index));
}

void ModelAlignmentDialog::OnSaveGroup()
{
    const int idx = m_pGroupCombo->currentIndex();
    if (idx < 0) return;
    ModelAlignmentConfig::SaveGroup(idx, m_pGroupCombo->currentText(), ParamsFromUi());
    Info(QStringLiteral("已保存参数组「%1」").arg(m_pGroupCombo->currentText()));
}

void ModelAlignmentDialog::OnSaveAsGroup()
{
    bool ok = false;
    const QString name = QInputDialog::getText(this, QStringLiteral("另存为参数组"),
        QStringLiteral("新组名称："), QLineEdit::Normal, QStringLiteral("参数组"), &ok).trimmed();
    if (!ok || name.isEmpty()) return;
    const int newIdx = ModelAlignmentConfig::AddGroup(name, ParamsFromUi());
    RefreshGroupList(newIdx);
    Info(QStringLiteral("已新建参数组「%1」").arg(name));
}

void ModelAlignmentDialog::OnDeleteGroup()
{
    const int idx = m_pGroupCombo->currentIndex();
    if (idx < 0) return;
    if (!ModelAlignmentConfig::DeleteGroup(idx))
    {
        QMessageBox::information(this, QStringLiteral("删除参数组"), QStringLiteral("至少保留一个参数组。"));
        return;
    }
    RefreshGroupList();
}

// ===================== 点云 + 运行 =====================

void ModelAlignmentDialog::OnBrowseCloud()
{
    if (m_busy) return;
    const QString startDir = AppPaths::WritablePath(QStringLiteral("Result"));
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("选择待处理点云"),
        startDir, QStringLiteral("点云文本 (*.txt);;所有文件 (*)"));
    if (path.isEmpty()) return;

    SetBusy(true);
    EnsureProgressDialog(QStringLiteral("正在安全加载点云…"), true);
    try
    {
        if (m_worker.joinable()) m_worker.join();
        m_stopRequested.store(false, std::memory_order_release);
        m_worker = std::thread([this, path]() noexcept
        {
            try
            {
                bool ok = false;
                QString error;
                QVector<RobotCalculation::IndexedPoint3D> points;
                try
                {
                    int lastPostedPercent = -1;
                    const WorkpieceMeshBuilder::ProgressCallback progress =
                        [this, &lastPostedPercent](int percent, const QString& stage)
                        {
                            if (m_stopRequested.load(std::memory_order_acquire)) return false;
                            percent = std::clamp(percent, 0, 100);
                            if (percent != lastPostedPercent
                                && !m_destroying.load(std::memory_order_acquire))
                            {
                                lastPostedPercent = percent;
                                QMetaObject::invokeMethod(this, [this, percent, stage]()
                                {
                                    if (m_stopRequested.load(std::memory_order_acquire)
                                        || m_destroying.load(std::memory_order_acquire)) return;
                                    if (m_pProgress)
                                    {
                                        m_pProgress->setLabelText(stage);
                                        m_pProgress->setValue(percent);
                                    }
                                }, Qt::QueuedConnection);
                            }
                            return !m_stopRequested.load(std::memory_order_acquire);
                        };
                    ok = WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
                        path, points, error, progress);
                    if (ok && points.isEmpty())
                    {
                        ok = false;
                        error = QStringLiteral("点云加载结果为空。");
                    }
                }
                catch (const std::bad_alloc&)
                {
                    error = QStringLiteral("加载点云时内存不足。");
                }
                catch (const std::exception& ex)
                {
                    error = QStringLiteral("加载点云异常：%1")
                        .arg(QString::fromLocal8Bit(ex.what()).left(256));
                }
                catch (...)
                {
                    error = QStringLiteral("加载点云发生未知异常。");
                }

                if (!m_destroying.load(std::memory_order_acquire))
                {
                    QMetaObject::invokeMethod(this,
                        [this, path, ok, error, points = std::move(points)]() mutable
                        {
                            OnCloudLoadFinished(path, ok, error, std::move(points));
                        }, Qt::QueuedConnection);
                }
            }
            catch (...)
            {
                // 点云工作线程与配准线程共享同一条 no-throw 边界，
                // 极端 OOM 也不得逃逸到 std::thread 导致进程 terminate。
            }
        });
    }
    catch (const std::bad_alloc&)
    {
        OnCloudLoadFinished(path, false, QStringLiteral("启动点云加载失败：内存不足。"), {});
    }
    catch (const std::exception& ex)
    {
        OnCloudLoadFinished(path, false,
            QStringLiteral("启动点云加载失败：%1")
                .arg(QString::fromLocal8Bit(ex.what()).left(256)), {});
    }
    catch (...)
    {
        OnCloudLoadFinished(path, false, QStringLiteral("启动点云加载失败。"), {});
    }
}

void ModelAlignmentDialog::OnCloudLoadFinished(
    const QString& path,
    bool ok,
    const QString& error,
    QVector<RobotCalculation::IndexedPoint3D> points)
{
    if (m_worker.joinable()) m_worker.join();
    bool accepted = ok;
    QString finalError = error;
    if (m_stopRequested.load(std::memory_order_acquire))
    {
        // 取消与“worker 已读完、UI 尚未处理完成回调”竞争时，
        // 用户取消优先，禁止延迟回调发布新 m_inputCloud。
        accepted = false;
        finalError = QStringLiteral("已取消");
        points.clear();
    }
    if (m_pProgress) m_pProgress->reset();
    SetBusy(false);

    if (!accepted || points.isEmpty())
    {
        if (finalError.contains(QStringLiteral("取消")))
        {
            Info(QStringLiteral("已取消加载点云，已有输入/去噪结果未改变。"));
        }
        else
        {
            Info(QStringLiteral("点云加载失败，已有输入/去噪结果未改变。"));
            QMessageBox::warning(this, QStringLiteral("加载点云"),
                QStringLiteral("加载失败：%1").arg(finalError));
        }
        return;
    }

    m_cloudPath = path;
    m_inputCloud = std::move(points);  // 只在 worker 完整成功后发布
    m_denoisedCloud.clear();
    m_pCloudEdit->setText(QDir::toNativeSeparators(path));
    m_pSaveResultBtn->setEnabled(false);
    UpdatePreview();
    Info(QStringLiteral("已加载点云 %1 点。选基准模型后点「运行」。")
        .arg(m_inputCloud.size()));
}

void ModelAlignmentDialog::OnRun()
{
    if (m_busy) return;
    if (m_inputCloud.isEmpty()) { Info(QStringLiteral("请先选择待处理点云。")); return; }
    const ModelAlignmentParams p = ParamsFromUi();
    if (p.referenceModel.isEmpty()) { Info(QStringLiteral("请先选择/导入基准模型。")); return; }
    if (!ReferenceModelLibrary::Exists(p.referenceModel)) { Info(QStringLiteral("基准模型不存在：%1").arg(p.referenceModel)); return; }

    // 运行前把当前参数写回当前组（顺手持久化）。
    ModelAlignmentConfig::SaveGroup(m_pGroupCombo->currentIndex(), m_pGroupCombo->currentText(), p);

    SetBusy(true);
    EnsureProgressDialog(QStringLiteral("模型配准 + 去噪中…"));

    try
    {
        if (m_worker.joinable())
        {
            m_worker.join();
        }
        m_stopRequested.store(false, std::memory_order_release);
        QVector<RobotCalculation::IndexedPoint3D> cloud = m_inputCloud;  // 复制给线程

        // noexcept + 最外层 catch(...) 是 std::thread 的硬边界：任何模型读取、
        // QVector/std::vector 分配或第三方异常都不得逃逸到线程入口而调用 terminate。
        m_worker = std::thread([this, cloud = std::move(cloud), p]() noexcept
        {
            try
            {
                bool ok = false;
                QString msg;
                QVector<RobotCalculation::IndexedPoint3D> result;
                try
                {
                    do
                    {
                        if (m_stopRequested.load(std::memory_order_acquire))
                        {
                            msg = QStringLiteral("模型配准任务已取消。");
                            break;
                        }
                        WorkpieceMeshBuilder::Mesh model;
                        QString err;
                        const auto stopRequested = [this]()
                        {
                            return m_stopRequested.load(std::memory_order_acquire);
                        };
                        if (!ReferenceModelLibrary::LoadModel(
                                p.referenceModel, model, err, stopRequested))
                        {
                            msg = stopRequested()
                                ? QStringLiteral("模型配准任务已取消。")
                                : QStringLiteral("加载基准模型失败：%1").arg(err);
                            break;
                        }
                        BcpdModelAligner::Options bo;
                        bo.omega = p.omega;
                        bo.lambda = p.lambda;
                        bo.beta = p.beta;
                        bo.targetSampleCap = p.targetSampleCap;
                        bo.useAcceleration = p.useAcceleration;
                        bo.cancelRequested = stopRequested;
                        WorkpieceMeshBuilder::Mesh deformed;
                        if (!BcpdModelAligner::DeformModelToCloud(cloud, model, deformed, bo))
                        {
                            msg = stopRequested()
                                ? QStringLiteral("模型配准任务已取消。")
                                : QStringLiteral("BCPD 配准失败（bcpd.exe 缺失/超时/异常），未去噪。");
                            break;
                        }
                        PointCloudModelDenoiser::Options dopt;
                        dopt.distanceThresholdMm = p.distanceThresholdMm;
                        dopt.keepPointsWithoutSurface = p.keepPointsWithoutSurface;
                        dopt.cancelRequested = stopRequested;
                        PointCloudModelDenoiser::Stats st;
                        result = PointCloudModelDenoiser::DenoiseByModelDistance(
                            cloud, deformed, dopt, &st);
                        if (st.cancelled || stopRequested())
                        {
                            result.clear();
                            msg = QStringLiteral("模型配准任务已取消。");
                            break;
                        }
                        if (st.resourceLimitExceeded)
                        {
                            result.clear();
                            msg = QStringLiteral("模型空间索引超出安全限制，未产生去噪结果。");
                            break;
                        }
                        msg = QStringLiteral("去噪完成：%1 → %2 点（远离剔除 %3，保留点最大距离 %4mm）")
                                  .arg(cloud.size()).arg(result.size()).arg(st.removedFar)
                                  .arg(st.maxKeptDistMm, 0, 'f', 2);
                        ok = true;
                    } while (false);
                }
                catch (const std::bad_alloc&)
                {
                    result.clear();
                    msg = QStringLiteral("模型配准内存不足，任务已安全终止。");
                }
                catch (const std::exception& ex)
                {
                    result.clear();
                    msg = QStringLiteral("模型配准发生异常，任务已安全终止：%1")
                        .arg(QString::fromLocal8Bit(ex.what()).left(256));
                }
                catch (...)
                {
                    result.clear();
                    msg = QStringLiteral("模型配准发生未知异常，任务已安全终止。");
                }

                // 用户点“取消”仍需要回到非 busy UI；只有真正析构时才禁止投递。
                if (!m_destroying.load(std::memory_order_acquire))
                {
                    QMetaObject::invokeMethod(this, [this, ok, msg, result]()
                    {
                        m_denoisedCloud = result;
                        OnRunFinished(ok, msg);
                    }, Qt::QueuedConnection);
                }
            }
            catch (...)
            {
                // 最后防线：即使极端 OOM 让异常文案/队列投递也失败，
                // std::thread 入口仍不允许异常逃逸导致整进程 terminate。
            }
        });
    }
    catch (const std::bad_alloc&)
    {
        OnRunFinished(false, QStringLiteral("启动模型配准失败：内存不足。"));
    }
    catch (const std::exception& ex)
    {
        OnRunFinished(false, QStringLiteral("启动模型配准失败：%1")
            .arg(QString::fromLocal8Bit(ex.what()).left(256)));
    }
    catch (...)
    {
        OnRunFinished(false, QStringLiteral("启动模型配准失败：未知异常。"));
    }
}

void ModelAlignmentDialog::OnRunFinished(bool ok, const QString& message)
{
    if (m_worker.joinable()) m_worker.join();
    if (m_pProgress) m_pProgress->reset();
    SetBusy(false);
    Info(message);
    if (ok)
    {
        UpdatePreview();
        m_pSaveResultBtn->setEnabled(!m_denoisedCloud.isEmpty());
    }
}

void ModelAlignmentDialog::OnSaveResult()
{
    if (m_denoisedCloud.isEmpty()) return;
    QString defaultPath = m_cloudPath;
    if (!defaultPath.isEmpty())
    {
        const QFileInfo fi(m_cloudPath);
        defaultPath = fi.dir().filePath(fi.completeBaseName() + QStringLiteral("_Denoised.txt"));
    }
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("保存去噪点云"),
        defaultPath, QStringLiteral("点云文本 (*.txt)"));
    if (path.isEmpty()) return;
    QString error;
    if (!WorkpieceMeshBuilder::SaveCloudPointsTxt(path, m_denoisedCloud, error))
    {
        QMessageBox::warning(this, QStringLiteral("保存"), error);
        return;
    }
    Info(QStringLiteral("已保存去噪点云：%1（%2 点）").arg(QDir::toNativeSeparators(path)).arg(m_denoisedCloud.size()));
}

void ModelAlignmentDialog::UpdatePreview()
{
    if (!m_pPreview) return;
    auto toLayer = [](const QString& name, const QVector<RobotCalculation::IndexedPoint3D>& pts, const QColor& c)
    {
        pcview::PointCloud3DView::Layer layer;
        layer.name = name;
        layer.color = c;
        layer.visible = true;
        layer.pointSize = 2.0;
        layer.points.reserve(pts.size());
        for (const RobotCalculation::IndexedPoint3D& ip : pts)
            layer.points.push_back(pcview::PointCloudVec3{ ip.point.x(), ip.point.y(), ip.point.z() });
        return layer;
    };
    QVector<pcview::PointCloud3DView::Layer> layers;
    if (!m_denoisedCloud.isEmpty())
    {
        // 去噪后：灰底显示输入（半透明感由点色体现），绿色显示保留点。
        layers << toLayer(QStringLiteral("输入点云"), m_inputCloud, QColor(96, 96, 104));
        layers << toLayer(QStringLiteral("去噪结果"), m_denoisedCloud, QColor(90, 210, 150));
    }
    else if (!m_inputCloud.isEmpty())
    {
        layers << toLayer(QStringLiteral("输入点云"), m_inputCloud, QColor(150, 190, 200));
    }
    m_pPreview->SetLayers(layers);
}

void ModelAlignmentDialog::SetBusy(bool busy)
{
    m_busy = busy;
    if (m_pRunBtn) m_pRunBtn->setEnabled(!busy);
    if (m_pSaveResultBtn)
        m_pSaveResultBtn->setEnabled(!busy && !m_denoisedCloud.isEmpty());
}

void ModelAlignmentDialog::EnsureProgressDialog(const QString& label, bool determinate)
{
    if (!m_pProgress)
    {
        m_pProgress = new QProgressDialog(this);
        m_pProgress->setWindowModality(Qt::WindowModal);
        m_pProgress->setMinimumDuration(0);
        m_pProgress->setCancelButtonText(QStringLiteral("取消"));
        m_pProgress->setAutoReset(false);
        m_pProgress->setAutoClose(false);
        connect(m_pProgress, &QProgressDialog::canceled, this, [this]()
        {
            if (!m_busy) return;
            m_stopRequested.store(true, std::memory_order_release);
            Info(QStringLiteral("正在取消模型配准，并等待已启动的子进程/读取安全退出…"));
        });
    }
    m_pProgress->reset();  // 清除上一轮 wasCanceled/进度，同一对话框可安全复用
    m_pProgress->setRange(0, determinate ? 100 : 0);
    if (determinate) m_pProgress->setValue(0);
    m_pProgress->setLabelText(label);
    m_pProgress->show();
}
