#pragma once

#include "CadSeamCandidateExtractor.h"
#include "ModelWeldingWorkflow.h"

#include <QDialog>

class ContralUnit;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class RobotDriverAdaptor;

namespace cadview
{
class CadModel3DView;
}

// “模型焊接流程”的配置与现场示教页。
//
// 首版刻意只完成模型模板、粗放置引导、编号特征站、人工扫描起终点、运行绑定和
// 刚体定位离线验证。它不会绕过 MeasureThenWeldDialog 的生产互锁直接控制机器人运动；
// 多站扫描执行接入后仍由 MeasureThenWeldDialog 持有整段机器人租约。
class ModelWeldingFlowDialog final : public QDialog
{
public:
    explicit ModelWeldingFlowDialog(
        ContralUnit* contralUnit,
        int initialUnitIndex = 0,
        QWidget* parent = nullptr);

    void reject() override;

private:
    void LoadRobots();
    void LoadModels();
    bool LoadTemplates();
    void LoadCameras();
    void LoadSelectedTemplate();
    bool LoadSelectedModel(
        WorkpieceMeshBuilder::Mesh& mesh,
        QString& modelSha256,
        QString& error) const;
    void LoadTeachingForCurrentRobot();

    void CreateDraftTemplate();
    void InheritSimilarTemplate();
    void ImportReferenceModel();
    void ClearTheoreticalRobotModel(const QString& detail = QString());
    void LoadCurrentRobotCatalogModel(bool reportErrors = false);
    void RefreshTheoreticalRobotStatus(const QString& detail = QString());
    void SaveTemplate();
    void SaveTeaching();
    void BindRuntimeIdentity();
    void CheckProductionReadiness();
    void OpenOfflineRigidFit();
    bool ConfirmDiscardChanges(
        const QString& action,
        bool templateWillChange,
        bool teachingWillChange);

    QString CurrentRobotName() const;
    QString CurrentRobotModelId() const;
    int CurrentConfiguredRobotType() const;
    QString CurrentRobotEndpoint() const;
    QString CurrentCameraSection() const;
    QString CurrentStationId() const;
    RobotDriverAdaptor* CurrentDriver() const;
    bool ReadCurrentRobotModelIdentity(
        QString& robotModelId,
        QString& sourceStepSha256,
        QString& collisionProfileSha256,
        QString& error) const;
    bool ReadCurrentRuntimeIdentity(
        QString& robotEndpoint,
        QString& handEyeSha256,
        QString& tool1Sha256,
        QString& error) const;
    ModelWeldingScanTeaching* EnsureTeaching(const QString& stationId);
    const ModelWeldingScanTeaching* FindTeaching(const QString& stationId) const;
    void TeachStart();
    void TeachEnd();
    void ExtractCadSeamCandidates();
    void AddSelectedCadSeamCandidate();
    void ProjectReverseMeshSeedFile();
    void RemoveSelectedSeam();
    void ClearSeamCandidates();

    void RefreshStationTable();
    void RefreshSeamTable();
    void RefreshStationDetails();
    void RefreshPreview(bool preserveView = false);
    bool HasActiveSimilarityInheritance() const;
    void RefreshVSlotPositionLabel();
    void RefreshWorkpieceOrientationLabel();
    void RefreshGroundFaceCandidates();
    void ApplySelectedGroundFace();
    void RotateWorkpieceAroundGroundZ(double degrees);
    void RotateVSlotAroundGroundZ(double degrees);
    void InvalidatePlacementDependentState(const QString& reason);
    void RefreshIdentityStatus();
    void SetStatus(const QString& text, bool error = false);

    ContralUnit* m_contralUnit = nullptr;
    int m_initialUnitIndex = 0;
    int m_activeRobotComboIndex = -1;
    bool m_loading = false;
    bool m_templatePersisted = false;
    bool m_teachingPersisted = false;
    bool m_templateDirty = false;
    bool m_teachingDirty = false;
    bool m_previewInitialized = false;
    bool m_modelIdentityValid = false;
    bool m_groundFaceSatisfied = false;
    bool m_vSlotWorkpieceSnapped = false;
    bool m_theoreticalRobotLoadInProgress = false;
    QString m_theoreticalRobotModelId;
    QString m_theoreticalRobotDisplayName;
    QString m_theoreticalRobotSha256;
    QString m_theoreticalRobotProfileKeySha256;
    qint64 m_theoreticalRobotSafetyMarginMicrometres = -1;
    QString m_teachingLoadNotice;
    QString m_previewModelName;
    QString m_previewPlySha256;
    QString m_previewSourcePath;
    QString m_previewSourceSha256;
    QString m_seamCandidateSourcePath;
    QString m_seamCandidateSourceSha256;

    WorkpieceMeshBuilder::Mesh m_mesh;
    QVector<CadSeamCandidateExtractor::Candidate> m_seamCandidates;
    QVector<ModelWeldingFlowTemplate> m_templates;
    QVector<ModelWeldingRobotTeaching> m_teachings;
    QVector<Eigen::Vector3d> m_groundFaceCentersModel;
    QVector<Eigen::Vector3d> m_groundFaceNormalsModel;
    QVector<double> m_groundFaceAreasMm2;
    QVector<double> m_groundFaceLargestAreasMm2;
    QVector<double> m_groundFaceTolerancesMm;
    QVector<int> m_groundFaceSourceIndices;
    ModelWeldingFlowTemplate m_template;
    ModelWeldingRobotTeaching m_teaching;

    QComboBox* m_templateCombo = nullptr;
    QComboBox* m_modelCombo = nullptr;
    QComboBox* m_robotCombo = nullptr;
    QComboBox* m_cameraCombo = nullptr;
    QComboBox* m_groundFaceCombo = nullptr;
    QComboBox* m_seamCandidateCombo = nullptr;
    QLineEdit* m_templateNameEdit = nullptr;
    QDoubleSpinBox* m_longLengthSpin = nullptr;
    QDoubleSpinBox* m_shortLengthSpin = nullptr;
    QDoubleSpinBox* m_modelRotationStepSpin = nullptr;
    QDoubleSpinBox* m_seamMinimumLengthSpin = nullptr;
    QDoubleSpinBox* m_runSpeedSpin = nullptr;
    QDoubleSpinBox* m_scanSpeedSpin = nullptr;
    QCheckBox* m_datumChecked = nullptr;
    QCheckBox* m_collisionChecked = nullptr;
    QCheckBox* m_showTheoreticalRobotChecked = nullptr;
    QLabel* m_inheritanceLabel = nullptr;
    QLabel* m_vSlotPositionLabel = nullptr;
    QLabel* m_workpieceOrientationLabel = nullptr;
    QLabel* m_theoreticalRobotStatusLabel = nullptr;
    QLabel* m_scanRegionHintLabel = nullptr;
    QLabel* m_startPoseLabel = nullptr;
    QLabel* m_endPoseLabel = nullptr;
    QLabel* m_identityLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    QTableWidget* m_seamTable = nullptr;
    QTableWidget* m_stationTable = nullptr;
    cadview::CadModel3DView* m_preview = nullptr;
};
