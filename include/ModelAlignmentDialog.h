#pragma once

#include "ModelAlignmentConfig.h"
#include "RobotCalculation.h"
#include "WorkpieceMeshBuilder.h"

#include <QDialog>
#include <QString>
#include <QVector>

#include <atomic>
#include <thread>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QProgressDialog;
class QPushButton;
class QSpinBox;
namespace pcview { class PointCloud3DView; }

// 「模型配准 / 点云去噪」独立功能界面（嵌入管理栈）。
//
// 流程：选基准模型（来自 ReferenceModelLibrary）+ 选待处理点云 + 选/编辑参数组 →
// BCPD 把基准模型非刚性形变贴合到点云（吸收尺寸/波形差异）→ 点到形变后模型距离去噪 →
// 预览去噪前后 + 保存去噪点云。参数按组存 ConfigStore.db，基准模型存专用库目录。
class ModelAlignmentDialog : public QDialog
{
public:
    explicit ModelAlignmentDialog(QWidget* parent = nullptr);
    ~ModelAlignmentDialog() override;

private:
    void BuildUi();
    void ApplyStyle();

    // 模型库
    void RefreshModelList(const QString& selectName = QString());
    void OnImportModelFromResult();
    void OnImportModelFromFile();
    void OnRenameModel();
    void OnDeleteModel();

    // 参数组
    void RefreshGroupList(int selectIndex = -1);
    void LoadParamsToUi(const ModelAlignmentParams& p);
    ModelAlignmentParams ParamsFromUi() const;
    void OnGroupChanged(int index);
    void OnSaveGroup();
    void OnSaveAsGroup();
    void OnDeleteGroup();

    // 点云 + 运行
    void OnBrowseCloud();
    void OnCloudLoadFinished(
        const QString& path,
        bool ok,
        const QString& error,
        QVector<RobotCalculation::IndexedPoint3D> points);
    void OnRun();
    void OnRunFinished(bool ok, const QString& message);
    void OnSaveResult();
    void UpdatePreview();
    void SetBusy(bool busy);
    void EnsureProgressDialog(const QString& label, bool determinate = false);
    void Info(const QString& text);

    // 模型库 / 参数组控件
    QComboBox* m_pModelCombo = nullptr;
    QComboBox* m_pGroupCombo = nullptr;
    QLineEdit* m_pCloudEdit = nullptr;

    // 参数控件
    QDoubleSpinBox* m_pOmega = nullptr;
    QDoubleSpinBox* m_pLambda = nullptr;
    QDoubleSpinBox* m_pBeta = nullptr;
    QSpinBox* m_pSampleCap = nullptr;
    QCheckBox* m_pAccel = nullptr;
    QDoubleSpinBox* m_pDistThresh = nullptr;
    QCheckBox* m_pKeepNoSurface = nullptr;

    QPushButton* m_pRunBtn = nullptr;
    QPushButton* m_pSaveResultBtn = nullptr;
    pcview::PointCloud3DView* m_pPreview = nullptr;
    QLabel* m_pInfoLabel = nullptr;
    QProgressDialog* m_pProgress = nullptr;

    // 数据
    QString m_cloudPath;
    QVector<RobotCalculation::IndexedPoint3D> m_inputCloud;    // 待处理点云
    QVector<RobotCalculation::IndexedPoint3D> m_denoisedCloud; // 去噪结果
    bool m_busy = false;
    bool m_suppressGroupSignal = false;  // 切组时抑制控件信号回写

    // 单一 owned worker：析构先请求取消再 join，禁止 detached 线程访问已销毁窗口。
    std::atomic_bool m_stopRequested{ false };
    std::atomic_bool m_destroying{ false };
    std::thread m_worker;
};
