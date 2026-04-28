#pragma once

#include <QDialog>
#include <QString>

#include "WeldProcessFile.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
class WeldProcessDialog;
}
QT_END_NAMESPACE

class QDoubleSpinBox;
class QCloseEvent;
class QFormLayout;
class QGridLayout;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QStackedWidget;

class WeldProcessDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WeldProcessDialog(const T_CONTRAL_UNIT& unitInfo, QWidget* parent = nullptr);
    ~WeldProcessDialog();

private slots:
    void ReloadData();
    bool SaveData();
    void OnWeldSelectionChanged(int row);
    void OnBeadSelectionChanged(int row);
    void OnWeldGroupsReordered();
    void AddWeldGroup();
    void RemoveWeldGroup();
    void AddBead();
    void RemoveBead();

private:
    void closeEvent(QCloseEvent* event) override;
    void BuildEditorUi();
    void ApplyDialogStyle();
    void LoadToUi(int preferredGroupRow = -1, int preferredBeadRow = -1);
    void PopulateWeldList(int preferredGroupRow);
    void PopulateBeadList(int preferredBeadRow);
    void ApplySelectionToUi(int weldIndex);
    int GroupCount() const;
    QString BuildWeldDisplayName(int index, const T_WELD_PARA& item) const;
    QString BuildGroupKey(const T_WELD_PARA& item) const;
    std::vector<int> GetGroupIndices(int groupRow) const;
    int CurrentWeldIndex() const;
    QLineEdit* AddTextField(QGridLayout* layout, const QString& label);
    QDoubleSpinBox* AddDoubleField(QGridLayout* layout, const QString& label, int decimals = 3);
    QSpinBox* AddIntField(QGridLayout* layout, const QString& label, int minimum = -999999, int maximum = 999999);
    bool CollectWeaveFromUi(T_WeaveDate& out) const;
    bool CollectWeldFromUi(T_WELD_PARA& out) const;
    void ShowError(const QString& message) const;
    QLineEdit* AddSingleTextField(QFormLayout* layout, const QString& label);
    QDoubleSpinBox* AddSingleDoubleField(QFormLayout* layout, const QString& label, int decimals = 3);
    QSpinBox* AddSingleIntField(QFormLayout* layout, const QString& label, int minimum = -999999, int maximum = 999999);
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();

private:
    Ui::WeldProcessDialog* ui = nullptr;
    WeldProcessFile m_file;
    bool m_isLoading = false;

    QListWidget* m_weldListWidget = nullptr;
    QListWidget* m_beadListWidget = nullptr;
    QPushButton* m_addWeldButton = nullptr;
    QPushButton* m_removeWeldButton = nullptr;
    QPushButton* m_addBeadButton = nullptr;
    QPushButton* m_removeBeadButton = nullptr;
    QPushButton* m_normalPageButton = nullptr;
    QPushButton* m_weavePageButton = nullptr;
    QStackedWidget* m_editorStack = nullptr;

    QLineEdit* m_workPeaceEdit = nullptr;
    QLineEdit* m_weldTypeEdit = nullptr;
    QDoubleSpinBox* m_weldAngleSizeSpin = nullptr;
    QSpinBox* m_standWeldDirSpin = nullptr;
    QSpinBox* m_weldMethodSpin = nullptr;
    QSpinBox* m_weaveTypeNoSpin = nullptr;

    QDoubleSpinBox* m_startArcCurrentSpin = nullptr;
    QDoubleSpinBox* m_startArcVoltageSpin = nullptr;
    QDoubleSpinBox* m_startWaitTimeSpin = nullptr;

    QDoubleSpinBox* m_trackCurrentSpin = nullptr;
    QDoubleSpinBox* m_trackVoltageSpin = nullptr;
    QDoubleSpinBox* m_weldVelocitySpin = nullptr;
    QDoubleSpinBox* m_crosswiseOffsetSpin = nullptr;
    QDoubleSpinBox* m_verticalOffsetSpin = nullptr;
    QSpinBox* m_wrapConditionNoSpin = nullptr;
    QDoubleSpinBox* m_weldAngleSpin = nullptr;
    QDoubleSpinBox* m_weldDipAngleSpin = nullptr;

    QDoubleSpinBox* m_stopArcCurrentSpin = nullptr;
    QDoubleSpinBox* m_stopArcVoltageSpin = nullptr;
    QDoubleSpinBox* m_stopWaitTimeSpin = nullptr;

    QDoubleSpinBox* m_wrapCurrent1Spin = nullptr;
    QDoubleSpinBox* m_wrapVoltage1Spin = nullptr;
    QDoubleSpinBox* m_wrapWaitTime1Spin = nullptr;
    QDoubleSpinBox* m_wrapCurrent2Spin = nullptr;
    QDoubleSpinBox* m_wrapVoltage2Spin = nullptr;
    QDoubleSpinBox* m_wrapWaitTime2Spin = nullptr;
    QDoubleSpinBox* m_wrapCurrent3Spin = nullptr;
    QDoubleSpinBox* m_wrapVoltage3Spin = nullptr;
    QDoubleSpinBox* m_wrapWaitTime3Spin = nullptr;

    QSpinBox* m_weaveTypeSpin = nullptr;
    QDoubleSpinBox* m_freqSpin = nullptr;
    QDoubleSpinBox* m_ampLeftSpin = nullptr;
    QDoubleSpinBox* m_ampRightSpin = nullptr;
    QSpinBox* m_stopTimeLeftSpin = nullptr;
    QSpinBox* m_stopTimeCenterSpin = nullptr;
    QSpinBox* m_stopTimeRightSpin = nullptr;
    QDoubleSpinBox* m_rotAngleXSpin = nullptr;
    QDoubleSpinBox* m_rotAngleZSpin = nullptr;
    QSpinBox* m_delayTypeLeftSpin = nullptr;
    QSpinBox* m_delayTypeCenterSpin = nullptr;
    QSpinBox* m_delayTypeRightSpin = nullptr;
    QDoubleSpinBox* m_rotAngleLeftSpin = nullptr;
    QDoubleSpinBox* m_rotAngleRightSpin = nullptr;
    QString m_cleanSnapshot;
};
