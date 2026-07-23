#pragma once

#include "RobotModelCatalogStore.h"

#include <QDialog>
#include <QList>

class QLabel;
class QPushButton;
class QTableWidget;
class QTextEdit;

// 机器人型号资产库管理。
//
// 本窗口只登记经过适配器完整验证的型号 STEP 与碰撞简模，不选择控制单元、
// 不写 RobotModelId，也不会根据 RobotA/RobotB 等名称猜测型号绑定。
class RobotModelManagerDialog final : public QDialog
{
public:
    explicit RobotModelManagerDialog(QWidget* parent = nullptr);

    void reject() override;

private:
    void BootstrapLegacyCatalog();
    void RefreshCatalog(const QString& preferredModelId = QString());
    void RefreshSelectionDetails();
    void ImportVerifiedModel();
    void SetBusy(bool busy);
    void SetStatus(const QString& text, bool error = false);

    bool m_busy = false;
    QList<RobotModelCatalogStore::ModelRecord> m_models;
    QTableWidget* m_modelTable = nullptr;
    QTextEdit* m_detailsEdit = nullptr;
    QLabel* m_statusLabel = nullptr;
    QPushButton* m_importButton = nullptr;
    QPushButton* m_refreshButton = nullptr;
};
