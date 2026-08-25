#include "ModelWeldingFlowDialog.h"

#include "CadModel3DView.h"
#include "HandEyeMatrixConfig.h"
#include "PointCloud3DView.h"
#include "ReferenceModelLibrary.h"
#include "ReverseMeshSeamProjector.h"
#include "RobotDataHelper.h"
#include "RobotDriverAdaptor.h"
#include "RobotMessage.h"
#include "RobotModelCatalogStore.h"
#include "RobotOperationLease.h"
#include "RobotCollisionEnvelopeStore.h"
#include "WindowStyleHelper.h"

#include <Eigen/Geometry>

#include <QCheckBox>
#include <QCloseEvent>
#include <QColor>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QProgressDialog>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QStringList>
#include <QTableWidget>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

namespace
{
constexpr int kRobotNameRole = Qt::UserRole + 1;
constexpr int kRobotModelIdRole = Qt::UserRole + 2;
constexpr int kRobotTypeRole = Qt::UserRole + 3;

double NormalizeSignedDegrees(double degrees)
{
    if (!std::isfinite(degrees)) return 0.0;
    double normalized = std::fmod(degrees + 180.0, 360.0);
    if (normalized < 0.0) normalized += 360.0;
    normalized -= 180.0;
    return std::abs(normalized) <= 1.0e-12 ? 0.0 : normalized;
}

QString CollisionMarginText(qint64 micrometres)
{
    if (micrometres < 0) return QStringLiteral("未知");
    if (micrometres % 1000 == 0)
        return QStringLiteral("%1 mm").arg(micrometres / 1000);
    return QStringLiteral("%1 mm").arg(
        static_cast<double>(micrometres) / 1000.0, 0, 'f', 3);
}

QString StationRoleText(ModelWeldingStationRole role)
{
    switch (role)
    {
    case ModelWeldingStationRole::Verify:
        return QStringLiteral("验证");
    case ModelWeldingStationRole::Backup:
        return QStringLiteral("备用");
    default:
        return QStringLiteral("求解");
    }
}

QString PoseText(const T_ROBOT_COORS& pose, bool taught)
{
    if (!taught)
    {
        return QStringLiteral("未示教");
    }
    return QStringLiteral("X %1  Y %2  Z %3\nRX %4  RY %5  RZ %6")
        .arg(pose.dX, 0, 'f', 3)
        .arg(pose.dY, 0, 'f', 3)
        .arg(pose.dZ, 0, 'f', 3)
        .arg(pose.dRX, 0, 'f', 3)
        .arg(pose.dRY, 0, 'f', 3)
        .arg(pose.dRZ, 0, 'f', 3);
}

bool Tool1IsFiniteAndReasonable(const T_ROBOT_COORS& tool, QString& error)
{
    const double values[6] = {
        tool.dX, tool.dY, tool.dZ, tool.dRX, tool.dRY, tool.dRZ
    };
    for (int index = 0; index < 6; ++index)
    {
        const double limit = index < 3 ? 5000.0 : 3600.0;
        if (!std::isfinite(values[index]) || std::abs(values[index]) > limit)
        {
            error = QStringLiteral("控制器Tool1第%1项不是有限合理值。").arg(index + 1);
            return false;
        }
    }
    return true;
}

cadview::Vec3d ViewPoint(const Eigen::Vector3d& point)
{
    return { point.x(), point.y(), point.z() };
}

QColor StationColor(const ModelWeldingFeatureStation& station, bool selected)
{
    if (selected)
    {
        return QColor(255, 214, 64);
    }
    if (!station.candidateConfirmed)
    {
        return QColor(239, 83, 80);
    }
    if (station.role == ModelWeldingStationRole::Verify)
    {
        return QColor(66, 165, 245);
    }
    if (station.role == ModelWeldingStationRole::Backup)
    {
        return QColor(158, 158, 158);
    }
    return QColor(102, 187, 106);
}

QString SeamSourceText(ModelWeldingSeamSource source)
{
    switch (source)
    {
    case ModelWeldingSeamSource::CadShapeIntersection:
        return QStringLiteral("实体截交");
    case ModelWeldingSeamSource::CadCorrugatedButtJoint:
        return QStringLiteral("板间对接立焊");
    case ModelWeldingSeamSource::CadCorrugatedBaseJoint:
        return QStringLiteral("下端波纹平焊");
    case ModelWeldingSeamSource::ReverseMeshSeedProjection:
        return QStringLiteral("逆向网格种子投影");
    default:
        return QStringLiteral("B-Rep共边");
    }
}

QString CandidateSourceText(CadSeamCandidateExtractor::SourceKind source)
{
    switch (source)
    {
    case CadSeamCandidateExtractor::SourceKind::ShapeIntersection:
        return QStringLiteral("实体截交");
    case CadSeamCandidateExtractor::SourceKind::CorrugatedButtJoint:
        return QStringLiteral("板间对接立焊");
    case CadSeamCandidateExtractor::SourceKind::CorrugatedBaseJoint:
        return QStringLiteral("下端波纹平焊");
    default:
        return QStringLiteral("B-Rep共边");
    }
}

double PolylineLength(const QVector<Eigen::Vector3d>& points)
{
    double length = 0.0;
    for (int index = 1; index < points.size(); ++index)
        length += (points.at(index) - points.at(index - 1)).norm();
    return length;
}

void AppendPolylineGuides(
    const QVector<Eigen::Vector3d>& points,
    const QColor& color,
    double width,
    QVector<cadview::GuideSegment>& guides)
{
    if (points.size() < 2) return;
    constexpr int kMaximumPreviewSegments = 400;
    const int stride = std::max(1,
        static_cast<int>(std::ceil(
            static_cast<double>(points.size() - 1) / kMaximumPreviewSegments)));
    for (int first = 0; first < points.size() - 1; first += stride)
    {
        const int second = std::min(first + stride, static_cast<int>(points.size()) - 1);
        cadview::GuideSegment segment;
        segment.start = ViewPoint(points.at(first));
        segment.end = ViewPoint(points.at(second));
        segment.color = color;
        segment.width = width;
        guides.push_back(segment);
    }
}

class NonClosableProgressDialog final : public QProgressDialog
{
public:
    using QProgressDialog::QProgressDialog;

    void reject() override
    {
        // STEP 导入当前不支持安全的中途取消；Esc 不能隐藏模态窗并放开父页面。
    }

protected:
    void closeEvent(QCloseEvent* event) override
    {
        event->ignore();
    }
};
}

ModelWeldingFlowDialog::ModelWeldingFlowDialog(
    ContralUnit* contralUnit,
    int initialUnitIndex,
    QWidget* parent)
    : QDialog(parent)
    , m_contralUnit(contralUnit)
    , m_initialUnitIndex(initialUnitIndex)
{
    setWindowTitle(QStringLiteral("模型焊接流程"));
    ApplyUnifiedWindowChrome(this);
    setModal(true);
    resize(1540, 940);

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(10);

    QLabel* boundary = new QLabel(
        QStringLiteral("本页用于模型模板与现场扫描示教。自动候选点必须人工确认；"
                       "当前版本不会从本页直接启动机器人运动。"));
    boundary->setWordWrap(true);
    boundary->setStyleSheet(QStringLiteral(
        "QLabel { background:#3b2f16; color:#ffd180; border:1px solid #8d6e3b; "
        "padding:8px; border-radius:4px; }"));
    root->addWidget(boundary);

    QGroupBox* selectorGroup = new QGroupBox(QStringLiteral("模板、模型与设备"));
    QGridLayout* selector = new QGridLayout(selectorGroup);
    m_templateCombo = new QComboBox();
    m_modelCombo = new QComboBox();
    m_robotCombo = new QComboBox();
    m_cameraCombo = new QComboBox();
    m_templateNameEdit = new QLineEdit();
    QPushButton* reloadButton = new QPushButton(QStringLiteral("刷新"));
    QPushButton* importButton = new QPushButton(QStringLiteral("导入模型"));
    QPushButton* draftButton = new QPushButton(QStringLiteral("新建/重新生成"));
    QPushButton* inheritButton = new QPushButton(QStringLiteral("继承相似模板"));
    selector->addWidget(new QLabel(QStringLiteral("流程模板")), 0, 0);
    selector->addWidget(m_templateCombo, 0, 1, 1, 2);
    selector->addWidget(reloadButton, 0, 3);
    selector->addWidget(new QLabel(QStringLiteral("模型库")), 1, 0);
    selector->addWidget(m_modelCombo, 1, 1);
    selector->addWidget(importButton, 1, 2);
    selector->addWidget(draftButton, 1, 3);
    selector->addWidget(new QLabel(QStringLiteral("模板名称")), 2, 0);
    selector->addWidget(m_templateNameEdit, 2, 1, 1, 2);
    selector->addWidget(inheritButton, 2, 3);
    selector->addWidget(new QLabel(QStringLiteral("机器人")), 0, 4);
    selector->addWidget(m_robotCombo, 0, 5);
    selector->addWidget(new QLabel(QStringLiteral("定位相机")), 1, 4);
    selector->addWidget(m_cameraCombo, 1, 5);
    selector->setColumnStretch(1, 1);
    selector->setColumnStretch(5, 1);
    root->addWidget(selectorGroup);

    QSplitter* splitter = new QSplitter(Qt::Horizontal);
    splitter->setChildrenCollapsible(false);
    // CadModel3DView 绑定原生 HWND；必须从一开始就以 splitter 为最终父对象，
    // 避免初始化 OCCT 后再次 reparent 导致原生窗口失效。
    m_preview = new cadview::CadModel3DView(splitter);
    m_preview->setMinimumSize(720, 520);
    m_preview->SetVSlotPoseChangedCallback(
        [this](const cadview::VSlotFixture& fixture, bool snapped)
        {
            if (m_loading || m_template.templateId.isEmpty()) return;
            const Eigen::Vector3d nextAnchor(
                fixture.anchor.x, fixture.anchor.y, fixture.anchor.z);
            Eigen::Vector3d nextAxisX(
                fixture.axisX.x, fixture.axisX.y, fixture.axisX.z);
            Eigen::Vector3d nextAxisZ(
                fixture.axisZ.x, fixture.axisZ.y, fixture.axisZ.z);
            if (!nextAnchor.allFinite() || !nextAxisX.allFinite()
                || !nextAxisZ.allFinite()
                || nextAxisX.norm() <= 1.0e-9 || nextAxisZ.norm() <= 1.0e-9)
            {
                return;
            }
            nextAxisX.normalize();
            nextAxisZ.normalize();
            const Eigen::Matrix3d& workpieceFrame = m_template.placement.axesModel;
            if (nextAxisZ.dot(workpieceFrame.col(2)) < 0.99999) return;
            const Eigen::Vector3d localX = workpieceFrame.transpose() * nextAxisX;
            if (!localX.allFinite() || std::hypot(localX.x(), localX.y()) <= 1.0e-9)
                return;
            const double nextYaw = NormalizeSignedDegrees(
                std::atan2(localX.y(), localX.x()) * 180.0 / std::acos(-1.0));
            const bool anchorChanged =
                (nextAnchor - m_template.placement.anchorModelMm).norm() > 1.0e-5;
            const bool yawChanged = std::abs(NormalizeSignedDegrees(
                nextYaw - m_template.placement.vSlotYawDegrees)) > 1.0e-5;
            if (!anchorChanged && !yawChanged) return;
            m_template.placement.anchorModelMm = nextAnchor;
            m_template.placement.vSlotYawDegrees = nextYaw;
            m_vSlotWorkpieceSnapped = snapped;
            InvalidatePlacementDependentState(
                snapped
                    ? QStringLiteral("V槽位置和方向已自动对齐并吸附到工件落地外轮廓")
                    : QStringLiteral("V槽位置或方向已修改，但尚未吸附到工件"));
            RefreshVSlotPositionLabel();
        });
    m_preview->SetVSlotSnapStateCallback([this](bool snapped)
        {
            m_vSlotWorkpieceSnapped = snapped;
            const bool snapRequired = !HasActiveSimilarityInheritance();
            if (m_datumChecked != nullptr)
            {
                m_datumChecked->setEnabled(m_groundFaceSatisfied
                    && m_modelIdentityValid
                    && (!snapRequired || m_vSlotWorkpieceSnapped));
            }
            RefreshVSlotPositionLabel();
        });
    m_preview->SetWorkpieceRotationRequestedCallback([this](double degrees)
        {
            RotateWorkpieceAroundGroundZ(degrees);
        });
    m_preview->SetVSlotRotationRequestedCallback([this](double degrees)
        {
            RotateVSlotAroundGroundZ(degrees);
        });
    splitter->addWidget(m_preview);

    QWidget* editor = new QWidget();
    QVBoxLayout* editorLayout = new QVBoxLayout(editor);
    editorLayout->setContentsMargins(6, 0, 0, 0);
    editorLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
    editor->setMinimumWidth(500);
    editor->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    QGroupBox* placementGroup = new QGroupBox(QStringLiteral("V型槽粗放置引导"));
    QGridLayout* placementLayout = new QGridLayout(placementGroup);
    m_longLengthSpin = new QDoubleSpinBox();
    m_shortLengthSpin = new QDoubleSpinBox();
    for (QDoubleSpinBox* spin : { m_longLengthSpin, m_shortLengthSpin })
    {
        spin->setRange(10.0, 10000.0);
        spin->setDecimals(1);
    }
    m_longLengthSpin->setValue(300.0);
    m_shortLengthSpin->setValue(180.0);
    m_datumChecked = new QCheckBox(QStringLiteral("已人工确认图中长边+X、短边+Y及地面+Z方向正确"));
    m_collisionChecked = new QCheckBox(QStringLiteral("已人工确认工装周围及扫描路径无碰撞"));
    m_inheritanceLabel = new QLabel(QStringLiteral("普通模板：首次按图放入V型槽建立粗基准。"));
    m_inheritanceLabel->setWordWrap(true);
    m_vSlotPositionLabel = new QLabel();
    m_vSlotPositionLabel->setWordWrap(true);
    m_vSlotPositionLabel->setStyleSheet(QStringLiteral("color:#ffb74d; font-weight:600;"));
    placementLayout->addWidget(new QLabel(QStringLiteral("长边 +X")), 0, 0);
    placementLayout->addWidget(
        CreateExternalUnitEditor(
            m_longLengthSpin, QStringLiteral("mm"), placementGroup),
        0,
        1);
    placementLayout->addWidget(new QLabel(QStringLiteral("短边 +Y")), 0, 2);
    placementLayout->addWidget(
        CreateExternalUnitEditor(
            m_shortLengthSpin, QStringLiteral("mm"), placementGroup),
        0,
        3);
    placementLayout->addWidget(m_vSlotPositionLabel, 1, 0, 1, 4);
    placementLayout->addWidget(new QLabel(
        QStringLiteral("先吸附工件落地大面；左侧视图顶部可分别旋转工件和V槽。"
                       "拖动橙色V槽靠近工件外角时，方向和位置会一起磁吸，成功后保持绿色。")),
        2, 0, 1, 4);
    placementLayout->addWidget(m_datumChecked, 3, 0, 1, 4);
    placementLayout->addWidget(m_collisionChecked, 4, 0, 1, 4);
    placementLayout->addWidget(m_inheritanceLabel, 5, 0, 1, 4);
    editorLayout->addWidget(placementGroup);

    QGroupBox* orientationGroup = new QGroupBox(QStringLiteral("工件落地姿态"));
    QGridLayout* orientationLayout = new QGridLayout(orientationGroup);
    m_groundFaceCombo = new QComboBox();
    QPushButton* snapGroundFaceButton = new QPushButton(QStringLiteral("吸附选中大面"));
    orientationLayout->addWidget(new QLabel(QStringLiteral("落地大面")), 0, 0);
    orientationLayout->addWidget(m_groundFaceCombo, 0, 1, 1, 2);
    orientationLayout->addWidget(snapGroundFaceButton, 0, 3);
    m_modelRotationStepSpin = new QDoubleSpinBox();
    m_modelRotationStepSpin->setRange(0.1, 180.0);
    m_modelRotationStepSpin->setDecimals(1);
    m_modelRotationStepSpin->setValue(15.0);
    m_preview->SetRotationStepDegrees(m_modelRotationStepSpin->value());
    QPushButton* rotateGroundMinus = new QPushButton(QStringLiteral("地面内旋转 −"));
    QPushButton* rotateGroundPlus = new QPushButton(QStringLiteral("地面内旋转 +"));
    orientationLayout->addWidget(new QLabel(QStringLiteral("工件每次旋转")), 1, 0);
    orientationLayout->addWidget(
        CreateExternalUnitEditor(
            m_modelRotationStepSpin, QStringLiteral("°"), orientationGroup),
        1,
        1);
    orientationLayout->addWidget(rotateGroundMinus, 1, 2);
    orientationLayout->addWidget(rotateGroundPlus, 1, 3);
    QPushButton* rotateVSlotMinus = new QPushButton(QStringLiteral("V槽旋转 −"));
    QPushButton* rotateVSlotPlus = new QPushButton(QStringLiteral("V槽旋转 +"));
    orientationLayout->addWidget(new QLabel(QStringLiteral("V槽独立朝向")), 2, 0, 1, 2);
    orientationLayout->addWidget(rotateVSlotMinus, 2, 2);
    orientationLayout->addWidget(rotateVSlotPlus, 2, 3);
    m_workpieceOrientationLabel = new QLabel();
    m_workpieceOrientationLabel->setWordWrap(true);
    m_workpieceOrientationLabel->setStyleSheet(QStringLiteral("color:#90caf9;"));
    orientationLayout->addWidget(m_workpieceOrientationLabel, 3, 0, 1, 4);
    orientationLayout->addWidget(new QLabel(QStringLiteral(
        "蓝色面为候选落地面；工件与V槽只绕地面Z旋转。左侧悬浮按钮可直接逐步调整，"
        "V槽拖近外角后会自动对齐边方向并吸附。空白拖动仅改变观察视角。")),
        4, 0, 1, 4);
    connect(m_groundFaceCombo, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int index)
        {
            if (m_preview == nullptr) return;
            const int sourceIndex = index >= 0 && index < m_groundFaceSourceIndices.size()
                ? m_groundFaceSourceIndices.at(index) : -1;
            m_preview->SetGroundFaceCandidateHighlight(sourceIndex);
        });
    connect(snapGroundFaceButton, &QPushButton::clicked,
        this, [this]() { ApplySelectedGroundFace(); });
    connect(rotateGroundMinus, &QPushButton::clicked, this, [this]()
        {
            RotateWorkpieceAroundGroundZ(-m_modelRotationStepSpin->value());
        });
    connect(rotateGroundPlus, &QPushButton::clicked, this, [this]()
        {
            RotateWorkpieceAroundGroundZ(m_modelRotationStepSpin->value());
        });
    connect(rotateVSlotMinus, &QPushButton::clicked, this, [this]()
        {
            RotateVSlotAroundGroundZ(-m_modelRotationStepSpin->value());
        });
    connect(rotateVSlotPlus, &QPushButton::clicked, this, [this]()
        {
            RotateVSlotAroundGroundZ(m_modelRotationStepSpin->value());
        });
    connect(m_modelRotationStepSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
        this, [this](double degrees)
        {
            if (m_preview != nullptr) m_preview->SetRotationStepDegrees(degrees);
        });
    editorLayout->addWidget(orientationGroup);

    QGroupBox* theoreticalRobotGroup = new QGroupBox(QStringLiteral("机器人碰撞简模"));
    QGridLayout* theoreticalRobotLayout = new QGridLayout(theoreticalRobotGroup);
    m_showTheoreticalRobotChecked = new QCheckBox(QStringLiteral("显示碰撞包络"));
    m_showTheoreticalRobotChecked->setChecked(true);
    m_showTheoreticalRobotChecked->setEnabled(false);
    m_theoreticalRobotStatusLabel = new QLabel();
    m_theoreticalRobotStatusLabel->setWordWrap(true);
    m_theoreticalRobotStatusLabel->setStyleSheet(QStringLiteral("color:#90caf9;"));
    theoreticalRobotLayout->addWidget(m_showTheoreticalRobotChecked, 0, 0, 1, 2);
    theoreticalRobotLayout->addWidget(m_theoreticalRobotStatusLabel, 1, 0, 1, 2);
    QLabel* theoreticalRobotHint = new QLabel(QStringLiteral(
        "机器人型号来自控制单元配置；本页只读取型号库中预生成并通过校验的 "
        "J0-J6 碰撞包络，不会读取或解析机器人总装 STEP。"
        "新增或更新适配模型请前往“控制单元 → 机器人模型库管理”。"
        "工件仍保留原始 STEP/B-Rep，供焊缝提取、扫描区域和其它模型功能使用。"
        "当前简模是静态未标定位置，不代表实时关节姿态或最终碰撞结论。"));
    theoreticalRobotHint->setWordWrap(true);
    theoreticalRobotLayout->addWidget(theoreticalRobotHint, 2, 0, 1, 2);
    connect(m_showTheoreticalRobotChecked, &QCheckBox::toggled,
        this, [this](bool visible)
        {
            if (m_loading || m_preview == nullptr) return;
            m_preview->SetTheoreticalRobotVisible(visible);
            RefreshTheoreticalRobotStatus();
        });
    editorLayout->addWidget(theoreticalRobotGroup);

    QGroupBox* seamGroup = new QGroupBox(QStringLiteral("焊缝候选（源 STEP / B-Rep）"));
    QGridLayout* seamLayout = new QGridLayout(seamGroup);
    m_seamMinimumLengthSpin = new QDoubleSpinBox();
    m_seamMinimumLengthSpin->setRange(5.0, 10000.0);
    m_seamMinimumLengthSpin->setDecimals(1);
    m_seamMinimumLengthSpin->setValue(100.0);
    m_seamCandidateCombo = new QComboBox();
    QPushButton* extractSeamsButton = new QPushButton(QStringLiteral("提取候选"));
    QPushButton* addSeamButton = new QPushButton(QStringLiteral("加入模板"));
    QPushButton* projectSeedButton = new QPushButton(QStringLiteral("种子投影"));
    QPushButton* removeSeamButton = new QPushButton(QStringLiteral("移除选中"));
    seamLayout->addWidget(new QLabel(QStringLiteral("最短长度")), 0, 0);
    seamLayout->addWidget(
        CreateExternalUnitEditor(
            m_seamMinimumLengthSpin, QStringLiteral("mm"), seamGroup),
        0, 1);
    seamLayout->addWidget(extractSeamsButton, 0, 2);
    seamLayout->addWidget(m_seamCandidateCombo, 1, 0, 1, 2);
    seamLayout->addWidget(addSeamButton, 1, 2);
    m_seamTable = new QTableWidget(0, 5);
    m_seamTable->setHorizontalHeaderLabels({
        QStringLiteral("编号"), QStringLiteral("来源"), QStringLiteral("长度"),
        QStringLiteral("点数"), QStringLiteral("确认")
    });
    m_seamTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_seamTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_seamTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_seamTable->horizontalHeader()->setStretchLastSection(true);
    m_seamTable->setMinimumHeight(150);
    seamLayout->addWidget(m_seamTable, 2, 0, 1, 3);
    QHBoxLayout* seamActions = new QHBoxLayout();
    seamActions->addWidget(projectSeedButton);
    seamActions->addWidget(removeSeamButton);
    seamLayout->addLayout(seamActions, 3, 2);
    QLabel* seamHint = new QLabel(QStringLiteral(
        "提取只读源 STEP：优先按装配产品语义识别板间对接立焊和下端波纹平焊，"
        "同一板件的折弯棱不作为焊缝；旧模型才回退实体截交。候选加入模板后仍为未确认；"
        "必须在三维视图核对并勾选确认，且本页不会生成机器人姿态或启动运动。"));
    seamHint->setWordWrap(true);
    seamHint->setStyleSheet(QStringLiteral("color:#80cbc4;"));
    seamLayout->addWidget(seamHint, 3, 0, 1, 2);
    editorLayout->addWidget(seamGroup);

    m_stationTable = new QTableWidget(0, 7);
    m_stationTable->setHorizontalHeaderLabels({
        QStringLiteral("编号"), QStringLiteral("角色"), QStringLiteral("模型X"),
        QStringLiteral("模型Y"), QStringLiteral("模型Z"), QStringLiteral("确认"),
        QStringLiteral("示教")
    });
    m_stationTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_stationTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_stationTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_stationTable->horizontalHeader()->setStretchLastSection(true);
    m_stationTable->setMinimumHeight(260);
    editorLayout->addWidget(m_stationTable, 1);
    m_scanRegionHintLabel = new QLabel(QStringLiteral(
        "彩色半透明区域 = 建议人工扫描并采集点云的模型范围；黄色为当前选中站点。"));
    m_scanRegionHintLabel->setWordWrap(true);
    m_scanRegionHintLabel->setStyleSheet(QStringLiteral("color:#80cbc4;"));
    editorLayout->addWidget(m_scanRegionHintLabel);

    QGroupBox* teachingGroup = new QGroupBox(QStringLiteral("选中站点：人工现场示教"));
    QGridLayout* teachingLayout = new QGridLayout(teachingGroup);
    m_startPoseLabel = new QLabel(QStringLiteral("未示教"));
    m_endPoseLabel = new QLabel(QStringLiteral("未示教"));
    m_startPoseLabel->setMinimumHeight(48);
    m_endPoseLabel->setMinimumHeight(48);
    m_runSpeedSpin = new QDoubleSpinBox();
    m_scanSpeedSpin = new QDoubleSpinBox();
    for (QDoubleSpinBox* spin : { m_runSpeedSpin, m_scanSpeedSpin })
    {
        spin->setRange(1.0, 100000.0);
        spin->setDecimals(1);
    }
    m_runSpeedSpin->setValue(300.0);
    m_scanSpeedSpin->setValue(100.0);
    QPushButton* teachStartButton = new QPushButton(QStringLiteral("读取当前为扫描起点"));
    QPushButton* teachEndButton = new QPushButton(QStringLiteral("读取当前为扫描终点"));
    teachingLayout->addWidget(new QLabel(QStringLiteral("起点")), 0, 0);
    teachingLayout->addWidget(m_startPoseLabel, 0, 1, 1, 3);
    teachingLayout->addWidget(new QLabel(QStringLiteral("终点")), 1, 0);
    teachingLayout->addWidget(m_endPoseLabel, 1, 1, 1, 3);
    teachingLayout->addWidget(new QLabel(QStringLiteral("运行速度")), 2, 0);
    teachingLayout->addWidget(
        CreateExternalUnitEditor(
            m_runSpeedSpin, QStringLiteral("mm/min"), teachingGroup),
        2,
        1);
    teachingLayout->addWidget(new QLabel(QStringLiteral("扫描速度")), 2, 2);
    teachingLayout->addWidget(
        CreateExternalUnitEditor(
            m_scanSpeedSpin, QStringLiteral("mm/min"), teachingGroup),
        2,
        3);
    teachingLayout->addWidget(teachStartButton, 3, 0, 1, 2);
    teachingLayout->addWidget(teachEndButton, 3, 2, 1, 2);
    editorLayout->addWidget(teachingGroup);

    m_identityLabel = new QLabel(QStringLiteral("运行绑定：尚未读取手眼参数与控制器Tool1"));
    m_identityLabel->setWordWrap(true);
    editorLayout->addWidget(m_identityLabel);

    QGridLayout* actionLayout = new QGridLayout();
    QPushButton* saveTemplateButton = new QPushButton(QStringLiteral("保存模型模板"));
    QPushButton* saveTeachingButton = new QPushButton(QStringLiteral("保存机器人示教"));
    QPushButton* bindButton = new QPushButton(QStringLiteral("读取并绑定手眼 / Tool1"));
    QPushButton* fitButton = new QPushButton(QStringLiteral("离线验证定位矩阵"));
    QPushButton* readyButton = new QPushButton(QStringLiteral("配置生产就绪检查"));
    actionLayout->addWidget(saveTemplateButton, 0, 0);
    actionLayout->addWidget(saveTeachingButton, 0, 1);
    actionLayout->addWidget(bindButton, 1, 0);
    actionLayout->addWidget(fitButton, 1, 1);
    actionLayout->addWidget(readyButton, 2, 0, 1, 2);
    editorLayout->addLayout(actionLayout);

    m_statusLabel = new QLabel(QStringLiteral("等待选择模型或模板。"));
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setMinimumHeight(44);
    editorLayout->addWidget(m_statusLabel);
    QScrollArea* editorScroll = new QScrollArea(splitter);
    editorScroll->setObjectName(QStringLiteral("AdaptiveWindowScrollArea"));
    editorScroll->setMinimumWidth(520);
    editorScroll->setWidget(editor);
    ConfigureResponsiveScrollArea(editorScroll);
    splitter->addWidget(editorScroll);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    splitter->setSizes({ 1050, 650 });
    root->addWidget(splitter, 1);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);

    connect(reloadButton, &QPushButton::clicked, this, [this]()
        {
            if (!ConfirmDiscardChanges(QStringLiteral("刷新模型库和模板"), true, true))
            {
                return;
            }
            LoadRobots();
            LoadModels();
            LoadTemplates();
            LoadCurrentRobotCatalogModel(false);
        });
    connect(importButton, &QPushButton::clicked, this, [this]() { ImportReferenceModel(); });
    connect(draftButton, &QPushButton::clicked, this, [this]() { CreateDraftTemplate(); });
    connect(inheritButton, &QPushButton::clicked, this, [this]() { InheritSimilarTemplate(); });
    connect(saveTemplateButton, &QPushButton::clicked, this, [this]() { SaveTemplate(); });
    connect(saveTeachingButton, &QPushButton::clicked, this, [this]() { SaveTeaching(); });
    connect(bindButton, &QPushButton::clicked, this, [this]() { BindRuntimeIdentity(); });
    connect(fitButton, &QPushButton::clicked, this, [this]() { OpenOfflineRigidFit(); });
    connect(readyButton, &QPushButton::clicked, this, [this]() { CheckProductionReadiness(); });
    connect(teachStartButton, &QPushButton::clicked, this, [this]() { TeachStart(); });
    connect(teachEndButton, &QPushButton::clicked, this, [this]() { TeachEnd(); });
    connect(extractSeamsButton, &QPushButton::clicked,
        this, [this]() { ExtractCadSeamCandidates(); });
    connect(addSeamButton, &QPushButton::clicked,
        this, [this]() { AddSelectedCadSeamCandidate(); });
    connect(projectSeedButton, &QPushButton::clicked,
        this, [this]() { ProjectReverseMeshSeedFile(); });
    connect(removeSeamButton, &QPushButton::clicked,
        this, [this]() { RemoveSelectedSeam(); });
    connect(m_seamCandidateCombo, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int)
        {
            if (!m_loading) RefreshPreview(true);
        });
    connect(m_seamTable, &QTableWidget::currentCellChanged,
        this, [this](int, int, int, int)
        {
            if (!m_loading) RefreshPreview(true);
        });
    connect(m_seamTable, &QTableWidget::itemChanged, this,
        [this](QTableWidgetItem* item)
        {
            if (m_loading || item == nullptr || item->column() != 4
                || item->row() < 0 || item->row() >= m_template.seams.size())
            {
                return;
            }
            ModelWeldingSeamDefinition& seam = m_template.seams[item->row()];
            const bool checked = item->checkState() == Qt::Checked;
            const bool reverseMesh = seam.source
                == ModelWeldingSeamSource::ReverseMeshSeedProjection;
            if (reverseMesh && checked && !seam.humanConfirmed)
            {
                const QSignalBlocker blocker(m_seamTable);
                item->setCheckState(Qt::Unchecked);
                SetStatus(QStringLiteral(
                    "逆向 PLY 焊缝必须通过“种子投影”的点云叠加预览确认，"
                    "不能在仅显示 STEP 的表格中直接勾选。"), true);
                return;
            }
            const bool sourceMatches = m_modelIdentityValid
                && (reverseMesh
                    ? seam.sourceGeometrySha256 == m_template.modelSha256
                    : (!m_previewSourceSha256.isEmpty()
                        && seam.sourceGeometrySha256 == m_previewSourceSha256));
            if (checked && !sourceMatches)
            {
                const QSignalBlocker blocker(m_seamTable);
                item->setCheckState(seam.humanConfirmed ? Qt::Checked : Qt::Unchecked);
                SetStatus(QStringLiteral(
                    "源 STEP 身份未通过当前追溯复核，禁止确认焊缝。请重新加载并提取候选。"), true);
                return;
            }
            seam.humanConfirmed = checked;
            m_templateDirty = true;
            RefreshPreview(true);
        });

    connect(m_templateCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (!m_loading)
            {
                if (!ConfirmDiscardChanges(QStringLiteral("切换模型流程模板"), true, true))
                {
                    const QSignalBlocker blocker(m_templateCombo);
                    const int restoreIndex = m_templateCombo->findData(m_template.templateId);
                    m_templateCombo->setCurrentIndex(restoreIndex >= 0 ? restoreIndex : 0);
                    return;
                }
                LoadSelectedTemplate();
            }
        });
    connect(m_robotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (!m_loading)
            {
                if (!ConfirmDiscardChanges(QStringLiteral("切换机器人"), false, true))
                {
                    const QSignalBlocker blocker(m_robotCombo);
                    int restoreIndex = m_activeRobotComboIndex >= 0
                        && m_activeRobotComboIndex < m_robotCombo->count()
                        ? m_activeRobotComboIndex : -1;
                    if (restoreIndex < 0)
                    {
                        const QString restoreEndpoint = m_teaching.robotEndpoint;
                        for (int index = 0; index < m_robotCombo->count(); ++index)
                        {
                            if (m_robotCombo->itemData(index, kRobotNameRole).toString()
                                != m_teaching.robotName)
                            {
                                continue;
                            }
                            RobotDriverAdaptor* candidateDriver = RobotDataHelper::GetRobotDriver(
                                m_contralUnit, m_robotCombo->itemData(index).toInt());
                            const QString candidateEndpoint = candidateDriver != nullptr
                                ? RobotOperationLease::PersistentEndpointIdentity(candidateDriver).trimmed()
                                : QString();
                            if (restoreEndpoint.isEmpty() || candidateEndpoint == restoreEndpoint)
                            {
                                restoreIndex = index;
                                break;
                            }
                        }
                    }
                    if (restoreIndex >= 0)
                    {
                        m_robotCombo->setCurrentIndex(restoreIndex);
                    }
                    return;
                }
                m_activeRobotComboIndex = m_robotCombo->currentIndex();
                ClearTheoreticalRobotModel();
                LoadCameras();
                LoadTeachingForCurrentRobot();
                LoadCurrentRobotCatalogModel(true);
            }
        });
    connect(m_cameraCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (!m_loading && !m_teaching.teachingId.isEmpty())
            {
                m_teaching.cameraSection = CurrentCameraSection();
                m_teaching.handEyeSha256.clear();
                m_teaching.tool1Sha256.clear();
                m_teachingDirty = true;
                RefreshIdentityStatus();
            }
        });
    connect(m_stationTable, &QTableWidget::currentCellChanged, this,
        [this](int, int, int, int)
        {
            if (!m_loading)
            {
                RefreshStationDetails();
                RefreshPreview(true);
            }
        });
    connect(m_stationTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item)
        {
            if (m_loading || item == nullptr || item->column() != 5
                || item->row() < 0 || item->row() >= m_template.stations.size())
            {
                return;
            }
            if (!m_modelIdentityValid)
            {
                const QSignalBlocker blocker(m_stationTable);
                item->setCheckState(m_template.stations.at(item->row()).candidateConfirmed
                    ? Qt::Checked : Qt::Unchecked);
                SetStatus(QStringLiteral("绑定模型缺失或SHA-256不匹配，禁止确认特征站。请恢复正确模型后重新载入模板。"), true);
                return;
            }
            m_template.stations[item->row()].candidateConfirmed =
                item->checkState() == Qt::Checked;
            m_templateDirty = true;
            RefreshPreview(true);
        });
    auto updateSpeed = [this]()
        {
            if (m_loading)
            {
                return;
            }
            ModelWeldingScanTeaching* scan = EnsureTeaching(CurrentStationId());
            if (scan != nullptr)
            {
                scan->runSpeedMmPerMin = m_runSpeedSpin->value();
                scan->scanSpeedMmPerMin = m_scanSpeedSpin->value();
                m_teachingDirty = true;
            }
        };
    connect(m_runSpeedSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [updateSpeed](double) { updateSpeed(); });
    connect(m_scanSpeedSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
        [updateSpeed](double) { updateSpeed(); });
    connect(m_collisionChecked, &QCheckBox::toggled, this, [this](bool checked)
        {
            if (!m_loading)
            {
                if (!m_modelIdentityValid && !m_template.templateId.isEmpty())
                {
                    const QSignalBlocker blocker(m_collisionChecked);
                    m_collisionChecked->setChecked(m_template.humanCollisionChecked);
                    SetStatus(QStringLiteral("绑定模型身份无效，禁止修改碰撞确认。"), true);
                    return;
                }
                m_template.humanCollisionChecked = checked;
                m_templateDirty = true;
            }
        });
    connect(m_datumChecked, &QCheckBox::toggled, this, [this](bool checked)
        {
            if (!m_loading)
            {
                if (checked && !m_groundFaceSatisfied)
                {
                    const QSignalBlocker blocker(m_datumChecked);
                    m_datumChecked->setChecked(false);
                    m_template.humanDatumConfirmed = false;
                    SetStatus(QStringLiteral(
                        "必须先选择外包络大平面并吸附到地面，才能确认模型粗基准。"), true);
                    return;
                }
                if (checked && !HasActiveSimilarityInheritance()
                    && !m_vSlotWorkpieceSnapped)
                {
                    const QSignalBlocker blocker(m_datumChecked);
                    m_datumChecked->setChecked(false);
                    m_template.humanDatumConfirmed = false;
                    SetStatus(QStringLiteral(
                        "普通模板必须先把V槽拖到工件外角，绿色表示已吸附。"), true);
                    return;
                }
                if (!m_modelIdentityValid && !m_template.templateId.isEmpty())
                {
                    const QSignalBlocker blocker(m_datumChecked);
                    m_datumChecked->setChecked(m_template.humanDatumConfirmed);
                    SetStatus(QStringLiteral("绑定模型身份无效，禁止修改基准确认。"), true);
                    return;
                }
                m_template.humanDatumConfirmed = checked;
                m_templateDirty = true;
            }
        });
    connect(m_templateNameEdit, &QLineEdit::textChanged, this, [this](const QString&)
        {
            if (!m_loading && !m_template.templateId.isEmpty())
            {
                m_templateDirty = true;
            }
        });
    auto updatePlacementGeometry = [this](double)
        {
            if (!m_loading && !m_template.templateId.isEmpty())
            {
                const double longLength = m_longLengthSpin->value();
                const double shortLength = m_shortLengthSpin->value();
                if (std::abs(m_template.placement.longLengthMm - longLength) <= 1.0e-9
                    && std::abs(m_template.placement.shortLengthMm - shortLength) <= 1.0e-9)
                {
                    return;
                }
                m_template.placement.longLengthMm = longLength;
                m_template.placement.shortLengthMm = shortLength;
                InvalidatePlacementDependentState(QStringLiteral("V槽实体尺寸已修改"));
                RefreshPreview(true);
            }
        };
    connect(m_longLengthSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
        this, updatePlacementGeometry);
    connect(m_shortLengthSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
        this, updatePlacementGeometry);

    RefreshVSlotPositionLabel();
    RefreshWorkpieceOrientationLabel();
    RefreshTheoreticalRobotStatus();
    LoadRobots();
    LoadModels();
    LoadTemplates();
    QTimer::singleShot(0, this, [this]()
        {
            LoadCurrentRobotCatalogModel(false);
        });
}

void ModelWeldingFlowDialog::reject()
{
    if ((m_templateDirty || m_teachingDirty)
        && QMessageBox::question(
            this,
            QStringLiteral("关闭模型焊接流程"),
            QStringLiteral("当前有未保存的模型模板或机器人示教修改。确定放弃这些修改吗？"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }
    QDialog::reject();
}

bool ModelWeldingFlowDialog::ConfirmDiscardChanges(
    const QString& action,
    bool templateWillChange,
    bool teachingWillChange)
{
    const bool losesTemplate = templateWillChange && m_templateDirty;
    const bool losesTeaching = teachingWillChange && m_teachingDirty;
    if (!losesTemplate && !losesTeaching)
    {
        return true;
    }
    QStringList affected;
    if (losesTemplate)
    {
        affected << QStringLiteral("模型模板");
    }
    if (losesTeaching)
    {
        affected << QStringLiteral("机器人示教");
    }
    return QMessageBox::question(
        this,
        action,
        QStringLiteral("此操作会放弃未保存的%1修改。确定继续吗？")
            .arg(affected.join(QStringLiteral("和"))),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No) == QMessageBox::Yes;
}

void ModelWeldingFlowDialog::LoadRobots()
{
    ClearTheoreticalRobotModel();
    m_loading = true;
    m_robotCombo->clear();
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_contralUnit);
    int initialIndex = -1;
    bool initialUnitSeen = false;
    QString initialUnitRejection;
    QStringList rejectionReasons;
    for (const RobotDataHelper::RobotInfo& robot : robots)
    {
        const bool isInitialUnit = robot.unitIndex == m_initialUnitIndex;
        initialUnitSeen = initialUnitSeen || isInitialUnit;
        RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(
            m_contralUnit, robot.unitIndex);
        if (robot.unitIndex < 0 || driver == nullptr)
        {
            const QString reason = QStringLiteral("%1：没有真实机器人驱动")
                .arg(robot.displayName);
            rejectionReasons.push_back(reason);
            if (isInitialUnit) initialUnitRejection = reason;
            continue;
        }
        if (robot.robotType < 0 || robot.robotType != driver->RobotType())
        {
            const QString reason = QStringLiteral("%1：控制单元机器人类型与驱动不一致")
                .arg(robot.displayName);
            rejectionReasons.push_back(reason);
            if (isInitialUnit) initialUnitRejection = reason;
            continue;
        }
        const QString modelId = robot.robotModelId.trimmed().toLower();
        if (modelId.isEmpty())
        {
            const QString reason = QStringLiteral("%1：控制单元未设置机器人型号")
                .arg(robot.displayName);
            rejectionReasons.push_back(reason);
            if (isInitialUnit) initialUnitRejection = reason;
            continue;
        }
        RobotModelCatalogStore::Eligibility eligibility;
        QString eligibilityError;
        if (!RobotModelCatalogStore::ResolveModelEligibility(
                modelId, driver->RobotType(), eligibility, eligibilityError)
            || !eligibility.eligible)
        {
            const QString reason = !eligibilityError.trimmed().isEmpty()
                ? eligibilityError.trimmed() : eligibility.reason.trimmed();
            const QString rejection = QStringLiteral("%1（%2）：%3")
                .arg(robot.displayName, modelId,
                    reason.isEmpty() ? QStringLiteral("型号未适配") : reason);
            rejectionReasons.push_back(rejection);
            if (isInitialUnit) initialUnitRejection = rejection;
            continue;
        }
        const QString modelDisplayName = eligibility.model.displayName.trimmed().isEmpty()
            ? modelId : eligibility.model.displayName.trimmed();
        m_robotCombo->addItem(
            QStringLiteral("%1 — 型号: %2 (%3)")
                .arg(robot.displayName, modelDisplayName, modelId),
            robot.unitIndex);
        const int row = m_robotCombo->count() - 1;
        m_robotCombo->setItemData(row, robot.robotName, kRobotNameRole);
        m_robotCombo->setItemData(row, modelId, kRobotModelIdRole);
        m_robotCombo->setItemData(row, robot.robotType, kRobotTypeRole);
        if (isInitialUnit)
        {
            initialIndex = row;
        }
    }
    if (initialIndex >= 0)
    {
        m_robotCombo->setCurrentIndex(initialIndex);
    }
    else
    {
        // QComboBox 添加第一项时会自动选中 0；入口指定机器人失效时必须显式
        // 保持无选择，不能静默切换到另一台合格机器人。
        m_robotCombo->setCurrentIndex(-1);
    }
    m_activeRobotComboIndex = m_robotCombo->currentIndex();
    m_robotCombo->setEnabled(m_robotCombo->count() > 0);
    m_loading = false;
    LoadCameras();
    if (m_robotCombo->count() == 0)
    {
        const QString detail = rejectionReasons.isEmpty()
            ? QStringLiteral("控制单元没有真实且已适配的机器人。")
            : QStringLiteral("没有可用于模型焊接的机器人：%1")
                .arg(rejectionReasons.join(QStringLiteral("；")));
        ClearTheoreticalRobotModel(detail);
        SetStatus(detail + QStringLiteral(
            " 请先在控制单元设置机器人型号，并在机器人模型库管理中准备简模。"), true);
    }
    else if (initialIndex < 0)
    {
        const QString detail = m_initialUnitIndex < 0
            ? QStringLiteral("入口没有指定机器人，当前保持未选择；请人工选择要使用的机器人。")
            : (initialUnitSeen
                ? QStringLiteral("入口指定机器人不满足模型焊接条件：%1")
                    .arg(initialUnitRejection.isEmpty()
                        ? QStringLiteral("型号或资产资格已失效") : initialUnitRejection)
                : QStringLiteral("入口指定的控制单元机器人已不存在。"));
        ClearTheoreticalRobotModel(detail);
        SetStatus(detail + QStringLiteral(" 不会自动切换到列表中的其他机器人。"), true);
    }
}

void ModelWeldingFlowDialog::LoadModels()
{
    const QString selected = m_modelCombo->currentText();
    const QSignalBlocker blocker(m_modelCombo);
    m_modelCombo->clear();
    m_modelCombo->addItems(ReferenceModelLibrary::ListModels());
    const int index = m_modelCombo->findText(selected);
    if (index >= 0)
    {
        m_modelCombo->setCurrentIndex(index);
    }
}

bool ModelWeldingFlowDialog::LoadTemplates()
{
    QString error;
    QVector<ModelWeldingFlowTemplate> templates;
    if (!ModelWeldingWorkflow::ListTemplates(templates, error))
    {
        SetStatus(QStringLiteral("读取模型流程模板失败：%1").arg(error), true);
        return false;
    }
    QVector<ModelWeldingRobotTeaching> teachings;
    if (!ModelWeldingWorkflow::ListTeachings(teachings, error))
    {
        SetStatus(QStringLiteral("读取机器人示教记录失败：%1").arg(error), true);
        return false;
    }

    const QString selectedId = m_templateCombo->currentData().toString();
    m_templates = templates;
    m_teachings = teachings;
    m_loading = true;
    m_templateCombo->clear();
    m_templateCombo->addItem(QStringLiteral("请选择模板"), QString());
    for (const ModelWeldingFlowTemplate& value : m_templates)
    {
        m_templateCombo->addItem(
            QStringLiteral("%1  [r%2]").arg(value.displayName).arg(value.revision),
            value.templateId);
    }
    const int index = m_templateCombo->findData(selectedId);
    if (index >= 0)
    {
        m_templateCombo->setCurrentIndex(index);
    }
    m_loading = false;
    LoadSelectedTemplate();
    return true;
}

void ModelWeldingFlowDialog::LoadCameras()
{
    const QString previous = m_cameraCombo->currentData().toString();
    const QSignalBlocker blocker(m_cameraCombo);
    m_cameraCombo->clear();
    int selectedIndex = -1;
    const QVector<RobotDataHelper::CameraInfo> cameras =
        RobotDataHelper::LoadCameraList(CurrentRobotName(), &selectedIndex);
    for (const RobotDataHelper::CameraInfo& camera : cameras)
    {
        m_cameraCombo->addItem(camera.displayName, camera.sectionName);
    }
    const int previousIndex = m_cameraCombo->findData(previous);
    if (previousIndex >= 0)
    {
        m_cameraCombo->setCurrentIndex(previousIndex);
    }
    else if (selectedIndex >= 0 && selectedIndex < m_cameraCombo->count())
    {
        m_cameraCombo->setCurrentIndex(selectedIndex);
    }
}

void ModelWeldingFlowDialog::LoadSelectedTemplate()
{
    const QString id = m_templateCombo->currentData().toString();
    if (id.isEmpty())
    {
        ClearSeamCandidates();
        m_template = ModelWeldingFlowTemplate();
        m_teaching = ModelWeldingRobotTeaching();
        m_mesh = WorkpieceMeshBuilder::Mesh();
        m_templatePersisted = false;
        m_teachingPersisted = false;
        m_templateDirty = false;
        m_teachingDirty = false;
        m_modelIdentityValid = false;
        m_teachingLoadNotice.clear();
        m_loading = true;
        m_templateNameEdit->clear();
        m_datumChecked->setChecked(false);
        m_collisionChecked->setChecked(false);
        m_loading = false;
        RefreshSeamTable();
        RefreshStationTable();
        RefreshPreview(false);
        RefreshIdentityStatus();
        SetStatus(QStringLiteral("尚未选择模型流程模板。"));
        return;
    }
    const auto it = std::find_if(m_templates.cbegin(), m_templates.cend(),
        [&id](const ModelWeldingFlowTemplate& value) { return value.templateId == id; });
    if (it == m_templates.cend())
    {
        ClearSeamCandidates();
        m_template = ModelWeldingFlowTemplate();
        m_teaching = ModelWeldingRobotTeaching();
        m_mesh = WorkpieceMeshBuilder::Mesh();
        m_templatePersisted = false;
        m_teachingPersisted = false;
        m_templateDirty = false;
        m_teachingDirty = false;
        m_modelIdentityValid = false;
        m_teachingLoadNotice.clear();
        RefreshSeamTable();
        RefreshStationTable();
        RefreshPreview(false);
        RefreshIdentityStatus();
        SetStatus(QStringLiteral("所选模板已不存在，请刷新。"), true);
        return;
    }

    ClearSeamCandidates();
    m_template = *it;
    m_templatePersisted = true;
    const bool convertedInactiveInheritance =
        !m_template.inheritedFromTemplateId.isEmpty()
        && !HasActiveSimilarityInheritance();
    if (convertedInactiveInheritance)
    {
        // 兼容旧版可能已留下“有继承来源，但同工装/扫描区确认已失效”
        // 的不可恢复记录。现有界面无法对原继承快照重新审核，因此安全地
        // 转为普通模板，并要求用户重新完成 V 槽、站点和碰撞确认。
        m_template.inheritedFromTemplateId.clear();
        m_template.inheritedFromRevision = 0;
        m_template.inheritedFromRecordSha256.clear();
        m_template.humanSameFixtureConfirmed = false;
        m_template.humanDatumConfirmed = false;
        m_template.humanScanAreaConfirmed = false;
        m_template.humanCollisionChecked = false;
        for (ModelWeldingFeatureStation& station : m_template.stations)
            station.candidateConfirmed = false;
    }
    m_templateDirty = convertedInactiveInheritance;
    m_loading = true;
    m_templateNameEdit->setText(m_template.displayName);
    m_longLengthSpin->setValue(m_template.placement.longLengthMm);
    m_shortLengthSpin->setValue(m_template.placement.shortLengthMm);
    m_datumChecked->setChecked(m_template.humanDatumConfirmed);
    m_collisionChecked->setChecked(m_template.humanCollisionChecked);
    const int modelIndex = m_modelCombo->findText(m_template.modelLibraryName);
    if (modelIndex >= 0)
    {
        m_modelCombo->setCurrentIndex(modelIndex);
    }
    m_loading = false;

    QString error;
    WorkpieceMeshBuilder::Mesh loadedMesh;
    const bool modelLoaded = modelIndex >= 0
        && ReferenceModelLibrary::LoadModel(
            m_template.modelLibraryName, loadedMesh, error);
    bool modelIdentityMatches = false;
    if (modelLoaded)
    {
        QString hashError;
        const QString currentHash = ModelWeldingWorkflow::ComputeFileSha256(
            ReferenceModelLibrary::ModelPath(m_template.modelLibraryName), &hashError);
        modelIdentityMatches = !currentHash.isEmpty()
            && currentHash == m_template.modelSha256.toLower();
        if (!modelIdentityMatches)
        {
            error = currentHash.isEmpty()
                ? hashError
                : QStringLiteral("模型文件SHA-256已变化，模板身份失效。");
        }
        else
        {
            m_mesh = std::move(loadedMesh);
        }
    }
    else
    {
        m_mesh = WorkpieceMeshBuilder::Mesh();
        if (error.isEmpty())
        {
            error = QStringLiteral("模型库中不存在“%1”。").arg(m_template.modelLibraryName);
        }
    }
    if (!modelIdentityMatches)
    {
        m_mesh = WorkpieceMeshBuilder::Mesh();
    }
    m_modelIdentityValid = modelIdentityMatches;
    m_inheritanceLabel->setText(!HasActiveSimilarityInheritance()
        ? QStringLiteral("普通模板：首次按图放入V型槽建立粗基准。")
        : QStringLiteral("相似模板快照继承：同一工装放置可跳过V型槽实物校准；站点仍须逐项人工确认。"));
    LoadTeachingForCurrentRobot();
    RefreshSeamTable();
    RefreshStationTable();
    RefreshPreview(false);
    if (!modelLoaded || !modelIdentityMatches)
    {
        SetStatus(QStringLiteral("模板已载入，但绑定模型不可用或身份不匹配：%1").arg(error), true);
    }
    else if (convertedInactiveInheritance)
    {
        SetStatus(QStringLiteral(
            "已读取到旧版失效的相似继承记录，已在内存中转为普通模板。"
            "请重新完成V槽吸附、站点和碰撞确认后保存新修订。"), true);
    }
    else if (!m_teachingLoadNotice.isEmpty())
    {
        SetStatus(m_teachingLoadNotice, true);
    }
    else if (m_teachingPersisted && CurrentCameraSection() != m_teaching.cameraSection)
    {
        SetStatus(QStringLiteral("模板已载入，但示教绑定的相机已不存在；运行指纹已失效。"), true);
    }
    else
    {
        SetStatus(QStringLiteral("已载入模板“%1”，修订 %2。")
            .arg(m_template.displayName).arg(m_template.revision));
    }
}

bool ModelWeldingFlowDialog::LoadSelectedModel(
    WorkpieceMeshBuilder::Mesh& mesh,
    QString& modelSha256,
    QString& error) const
{
    modelSha256.clear();
    const QString modelName = m_modelCombo->currentText().trimmed();
    if (modelName.isEmpty())
    {
        error = QStringLiteral("模型库为空或尚未选择模型。");
        return false;
    }
    mesh = WorkpieceMeshBuilder::Mesh();
    const QString modelPath = ReferenceModelLibrary::ModelPath(modelName);
    const QString firstSha256 = ModelWeldingWorkflow::ComputeFileSha256(modelPath, &error);
    if (firstSha256.isEmpty())
    {
        return false;
    }
    if (!ReferenceModelLibrary::LoadModel(modelName, mesh, error))
    {
        return false;
    }
    const QString secondSha256 = ModelWeldingWorkflow::ComputeFileSha256(modelPath, &error);
    if (secondSha256.isEmpty() || firstSha256 != secondSha256)
    {
        mesh = WorkpieceMeshBuilder::Mesh();
        error = secondSha256.isEmpty()
            ? error
            : QStringLiteral("读取期间模型文件发生变化，请等待文件稳定后重试。");
        return false;
    }
    modelSha256 = firstSha256;
    return true;
}

void ModelWeldingFlowDialog::LoadTeachingForCurrentRobot()
{
    if (m_template.templateId.isEmpty())
    {
        return;
    }
    const QString robotName = CurrentRobotName();
    const QString robotEndpoint = CurrentRobotEndpoint();
    m_teachingLoadNotice.clear();
    if (robotName.isEmpty() || robotEndpoint.isEmpty())
    {
        m_teaching = ModelWeldingRobotTeaching();
        m_teachingPersisted = false;
        m_teachingDirty = false;
        m_teachingLoadNotice = QStringLiteral(
            "当前未选择入口机器人；已清空机器人示教，不会自动套用其他机器人的记录。");
        RefreshStationTable();
        RefreshStationDetails();
        RefreshIdentityStatus();
        SetStatus(m_teachingLoadNotice, true);
        return;
    }
    QString currentRobotModelId;
    QString currentSourceStepSha256;
    QString currentCollisionProfileSha256;
    QString robotModelError;
    if (!ReadCurrentRobotModelIdentity(
            currentRobotModelId,
            currentSourceStepSha256,
            currentCollisionProfileSha256,
            robotModelError))
    {
        m_teaching = ModelWeldingRobotTeaching();
        m_teachingPersisted = false;
        m_teachingDirty = false;
        m_teachingLoadNotice = QStringLiteral("当前机器人型号资产身份无效：%1")
            .arg(robotModelError);
        RefreshStationTable();
        RefreshStationDetails();
        RefreshIdentityStatus();
        SetStatus(m_teachingLoadNotice, true);
        return;
    }
    const ModelWeldingRobotTeaching* newest = nullptr;
    const ModelWeldingRobotTeaching* migration = nullptr;
    bool migrationAmbiguous = false;
    bool incompatibleRobotModelIdentity = false;
    for (const ModelWeldingRobotTeaching& value : m_teachings)
    {
        if (value.templateId != m_template.templateId
            || value.templateRevision != m_template.revision
            || value.robotName != robotName)
        {
            continue;
        }
        const bool endpointMatches = value.robotEndpoint == robotEndpoint;
        const bool endpointNeedsMigration = value.robotEndpoint.isEmpty();
        if (!endpointMatches && !endpointNeedsMigration)
        {
            continue;
        }
        const bool modelIdentityMissing = value.robotModelId.isEmpty()
            && value.sourceStepSha256.isEmpty()
            && value.collisionProfileSha256.isEmpty();
        const bool modelIdentityMatches = value.robotModelId == currentRobotModelId
            && value.sourceStepSha256 == currentSourceStepSha256
            && value.collisionProfileSha256 == currentCollisionProfileSha256;
        if (endpointMatches && modelIdentityMatches)
        {
            if (newest == nullptr || value.revision > newest->revision)
            {
                newest = &value;
            }
            continue;
        }
        if (modelIdentityMissing || modelIdentityMatches)
        {
            if (migration == nullptr)
            {
                migration = &value;
            }
            else if (migration->teachingId != value.teachingId)
            {
                migrationAmbiguous = true;
            }
            continue;
        }
        incompatibleRobotModelIdentity = true;
    }
    const bool migrateLegacy = newest == nullptr && migration != nullptr
        && !migrationAmbiguous;
    if (migrateLegacy)
    {
        newest = migration;
    }
    if (newest != nullptr)
    {
        m_teaching = *newest;
        m_teachingPersisted = true;
        m_teachingDirty = migrateLegacy;
        if (migrateLegacy)
        {
            m_teaching.robotEndpoint = robotEndpoint;
            m_teaching.robotModelId = currentRobotModelId;
            m_teaching.sourceStepSha256 = currentSourceStepSha256;
            m_teaching.collisionProfileSha256 = currentCollisionProfileSha256;
            m_teaching.handEyeSha256.clear();
            m_teaching.tool1Sha256.clear();
            for (ModelWeldingScanTeaching& scan : m_teaching.scans)
            {
                // 旧记录没有机器人 CAD/简模身份，数值只能作为现场参考；清除
                // 所有生产示教见证，强制逐站重新读取起点、终点和起点脉冲。
                scan.startTaught = false;
                scan.endTaught = false;
                scan.startPulseTaught = false;
            }
            m_teachingLoadNotice = QStringLiteral(
                "已载入唯一的旧版示教作为迁移草稿；旧位姿数值仅供参考，"
                "所有示教完成标志均已清除，必须逐站重新示教起点/终点，"
                "并重新绑定机器人型号资产、手眼/Tool1后保存。"
                "旧记录不会自动通过生产检查。");
        }
        const int cameraIndex = m_cameraCombo->findData(m_teaching.cameraSection);
        const QSignalBlocker cameraBlocker(m_cameraCombo);
        if (cameraIndex >= 0)
        {
            m_cameraCombo->setCurrentIndex(cameraIndex);
        }
        else
        {
            m_cameraCombo->setCurrentIndex(-1);
            m_teaching.handEyeSha256.clear();
            m_teaching.tool1Sha256.clear();
            m_teachingDirty = true;
        }
    }
    else
    {
        m_teaching = ModelWeldingRobotTeaching();
        m_teaching.teachingId = ModelWeldingWorkflow::CreateStableId();
        m_teaching.revision = 1;
        m_teaching.templateId = m_template.templateId;
        m_teaching.templateRevision = m_template.revision;
        QString hashError;
        m_teaching.templateRecordSha256 =
            ModelWeldingWorkflow::TemplateRecordSha256(m_template, hashError);
        m_teaching.robotName = robotName;
        m_teaching.robotEndpoint = robotEndpoint;
        m_teaching.robotModelId = currentRobotModelId;
        m_teaching.sourceStepSha256 = currentSourceStepSha256;
        m_teaching.collisionProfileSha256 = currentCollisionProfileSha256;
        m_teaching.cameraSection = CurrentCameraSection();
        const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
        m_teaching.createdAtUtc = now;
        m_teaching.updatedAtUtc = now;
        for (const ModelWeldingFeatureStation& station : m_template.stations)
        {
            ModelWeldingScanTeaching scan;
            scan.stationId = station.stationId;
            m_teaching.scans.push_back(scan);
        }
        m_teachingPersisted = false;
        m_teachingDirty = true;
        if (migrationAmbiguous)
        {
            m_teachingLoadNotice = QStringLiteral(
                "发现多条需要迁移的旧版示教，无法安全自动选择；已新建空白示教草稿，必须重新示教。");
        }
        else if (incompatibleRobotModelIdentity)
        {
            m_teachingLoadNotice = QStringLiteral(
                "已有示教绑定了不同的机器人型号、源STEP或碰撞简模；旧记录已保留，"
                "当前已新建空白示教草稿，禁止跨型号自动套用。");
        }
    }
    RefreshStationTable();
    RefreshStationDetails();
    RefreshIdentityStatus();
    if (!m_teachingLoadNotice.isEmpty())
    {
        SetStatus(m_teachingLoadNotice, true);
    }
}

void ModelWeldingFlowDialog::CreateDraftTemplate()
{
    if (!ConfirmDiscardChanges(QStringLiteral("新建或重新生成模型模板"), true, true))
    {
        return;
    }
    QString error;
    WorkpieceMeshBuilder::Mesh candidateMesh;
    QString modelHash;
    if (!LoadSelectedModel(candidateMesh, modelHash, error))
    {
        QMessageBox::warning(this, QStringLiteral("新建模型模板"), error);
        return;
    }
    if (m_longLengthSpin->value() <= m_shortLengthSpin->value())
    {
        QMessageBox::warning(this, QStringLiteral("新建模型模板"),
            QStringLiteral("V型槽长边必须大于短边。"));
        return;
    }

    ModelWeldingPlacementGuide guide;
    if (!ModelWeldingWorkflow::GeneratePlacementGuide(
        candidateMesh, m_longLengthSpin->value(), m_shortLengthSpin->value(), guide, error))
    {
        QMessageBox::warning(this, QStringLiteral("生成放置引导"), error);
        return;
    }
    const QVector<ModelWeldingFeatureStation> stations =
        ModelWeldingWorkflow::GenerateDraftStations(candidateMesh, 3, 1);
    if (stations.size() < 4)
    {
        QMessageBox::warning(this, QStringLiteral("生成扫描站"),
            QStringLiteral("模型有效顶点不足，无法生成3个求解候选站和1个验证候选站。"));
        return;
    }

    const QString modelName = m_modelCombo->currentText().trimmed();
    ModelWeldingFlowTemplate value;
    value.templateId = ModelWeldingWorkflow::CreateStableId();
    value.displayName = m_templateNameEdit->text().trimmed().isEmpty()
        ? modelName + QStringLiteral(" 模型焊接")
        : m_templateNameEdit->text().trimmed();
    value.modelLibraryName = modelName;
    value.modelSha256 = modelHash;
    value.placement = guide;
    value.stations = stations;
    value.humanDatumConfirmed = false;
    value.humanCollisionChecked = false;
    const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    value.createdAtUtc = now;
    value.updatedAtUtc = now;
    if (!ModelWeldingWorkflow::ValidateTemplateStructure(value, error, false))
    {
        QMessageBox::warning(this, QStringLiteral("新建模型模板"),
            QStringLiteral("生成的模板结构无效：%1").arg(error));
        return;
    }
    m_mesh = std::move(candidateMesh);
    m_modelIdentityValid = true;
    m_template = value;
    ClearSeamCandidates();
    m_templatePersisted = false;
    m_templateDirty = true;
    {
        const QSignalBlocker blocker(m_templateCombo);
        m_templateCombo->setCurrentIndex(0);
    }
    m_templateNameEdit->setText(value.displayName);
    m_datumChecked->setChecked(false);
    m_collisionChecked->setChecked(false);
    m_inheritanceLabel->setText(QStringLiteral(
        "普通模板：首次按图把工件指定位置卡入V型槽；长边为+X，短边为+Y。"));
    LoadTeachingForCurrentRobot();
    RefreshSeamTable();
    RefreshStationTable();
    RefreshPreview(false);
    SetStatus(QStringLiteral("已生成4个候选站。候选位置仅用于辅助选点，必须逐项人工确认后才可生产。"));
}

void ModelWeldingFlowDialog::InheritSimilarTemplate()
{
    if (!ConfirmDiscardChanges(QStringLiteral("继承相似模板"), true, true))
    {
        return;
    }
    if (m_templates.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("继承相似模板"),
            QStringLiteral("当前没有可继承的已保存模型模板。"));
        return;
    }
    QStringList labels;
    for (const ModelWeldingFlowTemplate& value : m_templates)
    {
        labels << QStringLiteral("%1 [r%2] {%3}")
            .arg(value.displayName)
            .arg(value.revision)
            .arg(value.templateId);
    }
    bool ok = false;
    const QString selected = QInputDialog::getItem(
        this, QStringLiteral("继承相似模板"), QStringLiteral("选择来源模板"), labels, 0, false, &ok);
    if (!ok)
    {
        return;
    }
    const int sourceIndex = labels.indexOf(selected);
    if (sourceIndex < 0 || sourceIndex >= m_templates.size())
    {
        return;
    }
    const ModelWeldingFlowTemplate source = m_templates.at(sourceIndex);

    QString error;
    WorkpieceMeshBuilder::Mesh candidateMesh;
    QString modelHash;
    if (!LoadSelectedModel(candidateMesh, modelHash, error))
    {
        QMessageBox::warning(this, QStringLiteral("继承相似模板"), error);
        return;
    }
    ModelWeldingPlacementGuide targetGuide;
    if (!ModelWeldingWorkflow::GeneratePlacementGuide(
        candidateMesh, m_longLengthSpin->value(), m_shortLengthSpin->value(), targetGuide, error))
    {
        QMessageBox::warning(this, QStringLiteral("继承相似模板"), error);
        return;
    }
    const Eigen::Matrix3d targetFromSource =
        targetGuide.axesModel * source.placement.axesModel.transpose();
    // 相似模型继承同一物理工装朝向；工件模型坐标映射仍只使用各自
    // workpiece frame，V 槽相对偏航单独继承。
    targetGuide.vSlotYawDegrees = source.placement.vSlotYawDegrees;

    QDialog audit(this);
    audit.setWindowTitle(QStringLiteral("相似模型人工判定"));
    QVBoxLayout* layout = new QVBoxLayout(&audit);
    QLabel* hint = new QLabel(QStringLiteral(
        "继承会保留同一工装下的机器人扫描路径，并用两个模型的粗放置坐标系修复模型坐标。"
        "外观相似不等于可继承，请逐项现场核对。\n\n"
        "本次模型坐标轴映射 R_target_source：\n"
        "[%1  %2  %3]\n[%4  %5  %6]\n[%7  %8  %9]\n"
        "若映射后的三维编号位置与实体不符，必须取消继承。")
        .arg(targetFromSource(0, 0), 0, 'f', 4)
        .arg(targetFromSource(0, 1), 0, 'f', 4)
        .arg(targetFromSource(0, 2), 0, 'f', 4)
        .arg(targetFromSource(1, 0), 0, 'f', 4)
        .arg(targetFromSource(1, 1), 0, 'f', 4)
        .arg(targetFromSource(1, 2), 0, 'f', 4)
        .arg(targetFromSource(2, 0), 0, 'f', 4)
        .arg(targetFromSource(2, 1), 0, 'f', 4)
        .arg(targetFromSource(2, 2), 0, 'f', 4));
    hint->setWordWrap(true);
    layout->addWidget(hint);
    QCheckBox* sameFixture = new QCheckBox(QStringLiteral("放置工装和工件朝向与来源模板相同"));
    QCheckBox* sameDatum = new QCheckBox(QStringLiteral("长边+X、短边+Y对应的实体基准一致"));
    QCheckBox* scanArea = new QCheckBox(QStringLiteral("每个编号站的实际特征和可扫描区域仍对应"));
    QCheckBox* collision = new QCheckBox(QStringLiteral("已检查新模型外形、焊枪和扫描路径无碰撞"));
    layout->addWidget(sameFixture);
    layout->addWidget(sameDatum);
    layout->addWidget(scanArea);
    layout->addWidget(collision);
    QDialogButtonBox* auditButtons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QPushButton* okButton = auditButtons->button(QDialogButtonBox::Ok);
    okButton->setEnabled(false);
    auto refreshOk = [=]()
        {
            okButton->setEnabled(sameFixture->isChecked() && sameDatum->isChecked()
                && scanArea->isChecked() && collision->isChecked());
        };
    connect(sameFixture, &QCheckBox::toggled, &audit, [refreshOk](bool) { refreshOk(); });
    connect(sameDatum, &QCheckBox::toggled, &audit, [refreshOk](bool) { refreshOk(); });
    connect(scanArea, &QCheckBox::toggled, &audit, [refreshOk](bool) { refreshOk(); });
    connect(collision, &QCheckBox::toggled, &audit, [refreshOk](bool) { refreshOk(); });
    connect(auditButtons, &QDialogButtonBox::accepted, &audit, &QDialog::accept);
    connect(auditButtons, &QDialogButtonBox::rejected, &audit, &QDialog::reject);
    layout->addWidget(auditButtons);
    if (audit.exec() != QDialog::Accepted)
    {
        return;
    }

    const QString modelName = m_modelCombo->currentText().trimmed();
    QString sourceHashError;
    const QString sourceHash = ModelWeldingWorkflow::TemplateRecordSha256(source, sourceHashError);
    if (sourceHash.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("继承相似模板"),
            sourceHashError);
        return;
    }

    ModelWeldingFlowTemplate inherited;
    inherited.templateId = ModelWeldingWorkflow::CreateStableId();
    inherited.displayName = QStringLiteral("%1（继承 %2）").arg(modelName, source.displayName);
    inherited.modelLibraryName = modelName;
    inherited.modelSha256 = modelHash;
    inherited.placement = targetGuide;
    inherited.inheritedFromTemplateId = source.templateId;
    inherited.inheritedFromRevision = source.revision;
    inherited.inheritedFromRecordSha256 = sourceHash;
    inherited.humanSameFixtureConfirmed = true;
    inherited.humanDatumConfirmed = true;
    inherited.humanScanAreaConfirmed = true;
    inherited.humanCollisionChecked = true;
    for (ModelWeldingFeatureStation station : source.stations)
    {
        station.anchorModelMm = targetGuide.anchorModelMm
            + targetFromSource * (station.anchorModelMm - source.placement.anchorModelMm);
        station.scanDirectionModel = targetFromSource * station.scanDirectionModel;
        station.candidateConfirmed = false;
        inherited.stations.push_back(station);
    }
    const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    inherited.createdAtUtc = now;
    inherited.updatedAtUtc = now;
    if (!ModelWeldingWorkflow::ValidateTemplateStructure(inherited, error, false))
    {
        QMessageBox::warning(this, QStringLiteral("继承相似模板"),
            QStringLiteral("生成的继承模板结构无效：%1").arg(error));
        return;
    }
    const QString inheritedRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(inherited, error);
    if (inheritedRecordSha256.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("继承相似模板"), error);
        return;
    }
    m_mesh = std::move(candidateMesh);
    m_modelIdentityValid = true;
    m_template = inherited;
    ClearSeamCandidates();
    m_templatePersisted = false;
    m_templateDirty = true;
    {
        const QSignalBlocker blocker(m_templateCombo);
        m_templateCombo->setCurrentIndex(0);
    }

    const QString robotName = CurrentRobotName();
    const QString robotEndpoint = CurrentRobotEndpoint();
    QString robotModelId;
    QString sourceStepSha256;
    QString collisionProfileSha256;
    QString robotModelIdentityError;
    const bool robotModelIdentityValid = ReadCurrentRobotModelIdentity(
        robotModelId,
        sourceStepSha256,
        collisionProfileSha256,
        robotModelIdentityError);
    const ModelWeldingRobotTeaching* sourceTeaching = nullptr;
    for (const ModelWeldingRobotTeaching& value : m_teachings)
    {
        if (value.templateId == source.templateId
            && value.templateRevision == source.revision
            && value.robotName == robotName
            && value.robotEndpoint == robotEndpoint
            && robotModelIdentityValid
            && value.robotModelId == robotModelId
            && value.sourceStepSha256 == sourceStepSha256
            && value.collisionProfileSha256 == collisionProfileSha256
            && (sourceTeaching == nullptr || value.revision > sourceTeaching->revision))
        {
            sourceTeaching = &value;
        }
    }
    m_teaching = ModelWeldingRobotTeaching();
    m_teaching.teachingId = ModelWeldingWorkflow::CreateStableId();
    m_teaching.templateId = inherited.templateId;
    m_teaching.templateRevision = inherited.revision;
    m_teaching.templateRecordSha256 = inheritedRecordSha256;
    m_teaching.robotName = robotName;
    m_teaching.robotEndpoint = robotEndpoint;
    if (robotModelIdentityValid)
    {
        m_teaching.robotModelId = robotModelId;
        m_teaching.sourceStepSha256 = sourceStepSha256;
        m_teaching.collisionProfileSha256 = collisionProfileSha256;
    }
    m_teaching.cameraSection = CurrentCameraSection();
    m_teaching.createdAtUtc = now;
    m_teaching.updatedAtUtc = now;
    if (sourceTeaching != nullptr)
    {
        m_teaching.scans = sourceTeaching->scans;
    }
    else
    {
        for (const ModelWeldingFeatureStation& station : inherited.stations)
        {
            ModelWeldingScanTeaching scan;
            scan.stationId = station.stationId;
            m_teaching.scans.push_back(scan);
        }
    }
    m_teachingPersisted = false;
    m_teachingDirty = true;

    m_loading = true;
    m_templateNameEdit->setText(inherited.displayName);
    m_datumChecked->setChecked(true);
    m_collisionChecked->setChecked(true);
    m_loading = false;
    m_inheritanceLabel->setText(QStringLiteral(
        "已创建相似模型快照：运行时可跳过V型槽实物粗校准，但所有映射后的编号站必须重新确认。"));
    RefreshSeamTable();
    RefreshStationTable();
    RefreshStationDetails();
    RefreshPreview(false);
    RefreshIdentityStatus();
    SetStatus(sourceTeaching == nullptr
        ? QStringLiteral("相似模型坐标已粗修复；未找到当前机器人来源示教，请重新示教各站。")
        : QStringLiteral("相似模型坐标已粗修复，并复制当前机器人扫描路径快照；请逐站核对后确认。"));
}

void ModelWeldingFlowDialog::ImportReferenceModel()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("导入参考模型"),
        QString(),
        QStringLiteral("模型文件 (*.ply *.step *.stp);;本工程网格 (*.ply);;STEP模型 (*.step *.stp)"));
    if (path.isEmpty())
    {
        return;
    }
    const QString suffix = QFileInfo(path).suffix().toLower();
    const bool isStep = suffix == QStringLiteral("step") || suffix == QStringLiteral("stp");
    bool ok = false;
    const QString defaultName = QFileInfo(path).completeBaseName();
    const QString name = QInputDialog::getText(
        this, QStringLiteral("模型名称"), QStringLiteral("模型库名称"),
        QLineEdit::Normal, defaultName, &ok).trimmed();
    if (!ok)
    {
        return;
    }
    if (!ReferenceModelLibrary::IsValidName(name))
    {
        QMessageBox::warning(this, QStringLiteral("导入模型"), QStringLiteral("模型名称无效。"));
        return;
    }
    if (ReferenceModelLibrary::Exists(name))
    {
        QMessageBox::warning(this, QStringLiteral("导入模型"),
            QStringLiteral("模型库中已存在“%1”。为避免使已绑定模板失效，"
                           "模型焊接流程不允许同名覆盖；请使用新名称导入。")
                .arg(name));
        return;
    }
    if (isStep)
    {
        bool pending = false;
        bool canConfirm = false;
        QString pendingSummary;
        QString pendingToken;
        QString pendingError;
        if (!ReferenceModelLibrary::InspectPendingStepImport(
                name, pending, canConfirm, pendingSummary, pendingToken, pendingError))
        {
            QMessageBox::warning(this, QStringLiteral("检查待确认STEP"), pendingError);
            return;
        }
        if (pending)
        {
            if (canConfirm)
            {
                const QMessageBox::StandardButton recover = QMessageBox::question(
                    this,
                    QStringLiteral("发现待确认STEP模型"),
                    QStringLiteral(
                        "检测到同名模型上次已完成转换，但尚未人工确认。\n\n%1\n\n"
                        "是否恢复并确认上次保存的结果？这不是本次刚选择的新文件。")
                        .arg(pendingSummary),
                    QMessageBox::Yes | QMessageBox::No,
                    QMessageBox::No);
                if (recover == QMessageBox::Yes)
                {
                    QString recoverError;
                    if (!ReferenceModelLibrary::ConfirmStepImport(name, pendingToken, recoverError))
                    {
                        QMessageBox::warning(this, QStringLiteral("恢复STEP模型失败"), recoverError);
                        return;
                    }
                    LoadModels();
                    const int recoveredIndex = m_modelCombo->findText(name);
                    if (recoveredIndex >= 0) m_modelCombo->setCurrentIndex(recoveredIndex);
                    SetStatus(QStringLiteral("已恢复并确认上次待处理的STEP模型“%1”：%2")
                        .arg(name, pendingSummary));
                    return;
                }
            }

            const QMessageBox::StandardButton discard = QMessageBox::warning(
                this,
                QStringLiteral("撤销上次待确认STEP"),
                pendingSummary + QStringLiteral(
                    "\n\n若继续，将永久删除上次隐藏的待确认/中断记录，然后导入本次选择的文件。"
                    "当前新文件尚未完成解析验证。是否明确撤销上次记录？"),
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No);
            if (discard != QMessageBox::Yes) return;
            QString discardError;
            if (!ReferenceModelLibrary::DiscardPendingStepImport(name, pendingToken, discardError))
            {
                QMessageBox::warning(this, QStringLiteral("撤销待确认STEP失败"), discardError);
                return;
            }
        }
    }
    QString error;
    QString importSummary;
    QString confirmationToken;
    bool imported = false;
    if (isStep)
    {
        // OCCT 解析和三角化可能耗时，放到工作线程；进度窗保留 UI 消息循环，
        // 当前导入是原子事务，因此完成前不提供无法安全中断的假取消按钮。
        NonClosableProgressDialog progress(
            QStringLiteral("正在读取 STEP、验证单位并生成模型网格…"),
            QString(), 0, 0, this);
        progress.setWindowTitle(QStringLiteral("STEP模型导入"));
        progress.setWindowModality(Qt::WindowModal);
        progress.setCancelButton(nullptr);
        progress.setMinimumDuration(0);
        progress.setAutoClose(false);
        progress.setAutoReset(false);
        progress.setMinimumWidth(520);
        progress.setWindowFlag(Qt::WindowCloseButtonHint, false);

        QEventLoop waitLoop;
        QThread* worker = QThread::create([&]()
            {
                try
                {
                    imported = ReferenceModelLibrary::ImportFromStepFile(
                        name, path, error, &importSummary, &confirmationToken);
                }
                catch (const std::exception& exception)
                {
                    error = QStringLiteral("STEP导入线程发生异常：%1")
                        .arg(QString::fromLocal8Bit(exception.what()).simplified());
                    imported = false;
                }
                catch (...)
                {
                    error = QStringLiteral("STEP导入线程发生未知异常。");
                    imported = false;
                }
            });
        connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
        connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
        progress.show();
        worker->start();
        waitLoop.exec();
        worker->wait();
        delete worker;
        progress.accept();
    }
    else
    {
        imported = ReferenceModelLibrary::ImportFromFile(name, path, error);
    }
    if (!imported)
    {
        QMessageBox::warning(this, QStringLiteral("导入模型"), error);
        return;
    }
    if (isStep)
    {
        const QMessageBox::StandardButton confirmation = QMessageBox::question(
            this,
            QStringLiteral("确认STEP模型尺寸"),
            importSummary + QStringLiteral(
                "\n\n请核对CAD声明单位和毫米尺寸是否符合实物。选择“否”将立即撤销本次导入。"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if (confirmation != QMessageBox::Yes)
        {
            QString deleteError;
            if (!ReferenceModelLibrary::DiscardPendingStepImport(
                    name, confirmationToken, deleteError))
            {
                QMessageBox::warning(this, QStringLiteral("撤销STEP导入"), deleteError);
            }
            else
            {
                SetStatus(QStringLiteral("STEP模型“%1”因尺寸未确认，已撤销导入。").arg(name));
            }
            LoadModels();
            return;
        }
        QString confirmationError;
        if (!ReferenceModelLibrary::ConfirmStepImport(name, confirmationToken, confirmationError))
        {
            QMessageBox::warning(
                this,
                QStringLiteral("确认STEP模型失败"),
                confirmationError + QStringLiteral(
                    "\n待确认文件已保留；重新导入同名模型时可以恢复确认或显式撤销。"));
            LoadModels();
            return;
        }
    }
    LoadModels();
    const int index = m_modelCombo->findText(name);
    if (index >= 0)
    {
        m_modelCombo->setCurrentIndex(index);
    }
    if (isStep)
    {
        SetStatus(QStringLiteral("STEP模型“%1”已导入：%2").arg(name, importSummary));
    }
    else
    {
        SetStatus(QStringLiteral("模型“%1”已导入参考模型库。").arg(name));
    }
}

void ModelWeldingFlowDialog::ClearTheoreticalRobotModel(const QString& detail)
{
    if (m_preview != nullptr)
    {
        m_preview->ClearTheoreticalRobot();
    }
    m_theoreticalRobotLoadInProgress = false;
    m_theoreticalRobotModelId.clear();
    m_theoreticalRobotDisplayName.clear();
    m_theoreticalRobotSha256.clear();
    m_theoreticalRobotProfileKeySha256.clear();
    m_theoreticalRobotSafetyMarginMicrometres = -1;
    if (m_showTheoreticalRobotChecked != nullptr)
    {
        m_showTheoreticalRobotChecked->setEnabled(false);
    }
    RefreshTheoreticalRobotStatus(detail);
}

void ModelWeldingFlowDialog::LoadCurrentRobotCatalogModel(bool reportErrors)
{
    if (m_theoreticalRobotLoadInProgress || m_preview == nullptr)
    {
        return;
    }

    // 型号切换必须先清掉上一台机器人的显示与身份；任何后续失败都保持空场景。
    ClearTheoreticalRobotModel();

    if (m_robotCombo == nullptr || m_robotCombo->currentIndex() < 0)
    {
        const QString detail = QStringLiteral(
            "当前未选择入口机器人；机器人场景保持清空，不会自动切换到其他机器人。");
        ClearTheoreticalRobotModel(detail);
        SetStatus(detail, true);
        return;
    }

    const QString modelId = CurrentRobotModelId();
    const int configuredRobotType = CurrentConfiguredRobotType();
    RobotDriverAdaptor* driver = CurrentDriver();
    QString error;
    if (driver == nullptr)
    {
        error = QStringLiteral("当前列表项没有真实机器人驱动。");
    }
    else if (modelId.isEmpty())
    {
        error = QStringLiteral("当前控制单元未设置机器人型号。");
    }
    else if (configuredRobotType < 0 || configuredRobotType != driver->RobotType())
    {
        error = QStringLiteral("当前控制单元机器人类型与实际驱动不一致。");
    }

    RobotModelCatalogStore::Eligibility eligibility;
    if (error.isEmpty()
        && !RobotModelCatalogStore::ResolveModelEligibility(
            modelId, driver->RobotType(), eligibility, error))
    {
        if (error.trimmed().isEmpty())
        {
            error = QStringLiteral("机器人模型目录无法可信读取。");
        }
    }
    if (error.isEmpty() && !eligibility.eligible)
    {
        error = eligibility.reason.trimmed().isEmpty()
            ? QStringLiteral("当前机器人型号没有可用的适配模型和碰撞简模。")
            : eligibility.reason.trimmed();
    }

    if (error.isEmpty())
    {
        const QString resolvedModelId =
            eligibility.model.modelId.trimmed().toLower();
        const QString sourceSha256 =
            eligibility.model.sourceStep.sha256.trimmed().toLower();
        const QString profileKeySha256 =
            eligibility.model.collision.profileKeySha256.trimmed().toLower();
        const QString payloadSha256 =
            eligibility.model.collisionPayloadSha256.trimmed().toLower();
        const RobotCollisionEnvelopeStore::EnvelopeSet& envelope =
            eligibility.collisionEnvelope;
        if (resolvedModelId != modelId
            || eligibility.model.sourceRobotType != configuredRobotType
            || sourceSha256.isEmpty()
            || sourceSha256 != envelope.sourceStepSha256.trimmed().toLower()
            || profileKeySha256.isEmpty()
            || profileKeySha256 != envelope.profileKeySha256.trimmed().toLower()
            || payloadSha256.isEmpty()
            || payloadSha256 != envelope.payloadSha256.trimmed().toLower())
        {
            error = QStringLiteral(
                "机器人型号目录记录、源 STEP 身份与碰撞简模身份不一致。");
        }
    }

    if (!error.isEmpty())
    {
        ClearTheoreticalRobotModel(error);
        SetStatus(QStringLiteral("机器人碰撞简模加载失败：%1").arg(error), true);
        if (reportErrors)
        {
            QMessageBox::warning(
                this, QStringLiteral("加载机器人碰撞简模"), error);
        }
        return;
    }

    QString displayError;
    if (!m_preview->SetRobotCollisionEnvelope(
            eligibility.collisionEnvelope, displayError, false))
    {
        ClearTheoreticalRobotModel(displayError);
        SetStatus(QStringLiteral("机器人碰撞简模显示失败：%1").arg(displayError), true);
        if (reportErrors)
        {
            QMessageBox::warning(
                this, QStringLiteral("显示机器人碰撞简模"), displayError);
        }
        return;
    }

    m_theoreticalRobotModelId = eligibility.model.modelId.trimmed().toLower();
    m_theoreticalRobotDisplayName = eligibility.model.displayName.trimmed();
    m_theoreticalRobotSha256 =
        eligibility.model.sourceStep.sha256.trimmed().toLower();
    m_theoreticalRobotProfileKeySha256 =
        eligibility.model.collision.profileKeySha256.trimmed().toLower();
    m_theoreticalRobotSafetyMarginMicrometres =
        eligibility.collisionEnvelope.safetyMarginMicrometres;
    m_showTheoreticalRobotChecked->setEnabled(true);
    m_preview->SetTheoreticalRobotVisible(
        m_showTheoreticalRobotChecked->isChecked());
    RefreshTheoreticalRobotStatus(
        QStringLiteral("型号来自当前控制单元，简模来自机器人模型库。"));
}

void ModelWeldingFlowDialog::RefreshTheoreticalRobotStatus(const QString& detail)
{
    if (m_theoreticalRobotStatusLabel == nullptr) return;
    if (m_theoreticalRobotLoadInProgress)
    {
        m_theoreticalRobotStatusLabel->setText(detail.isEmpty()
            ? QStringLiteral("正在加载机器人碰撞简模…") : detail);
        return;
    }
    if (m_preview != nullptr && m_preview->IsRobotCollisionEnvelope())
    {
        const bool visible = m_preview->HasDisplayedTheoreticalRobot();
        QString status = QStringLiteral(
            "%1（%2）；J0-J6 碰撞包络；安全余量 %3；源SHA %4；简模配置 %5；"
            "静态未标定；%6")
            .arg(m_theoreticalRobotDisplayName.isEmpty()
                    ? QStringLiteral("机器人简模") : m_theoreticalRobotDisplayName)
            .arg(m_theoreticalRobotModelId)
            .arg(CollisionMarginText(
                m_theoreticalRobotSafetyMarginMicrometres))
            .arg(m_theoreticalRobotSha256.left(12) + QStringLiteral("…"))
            .arg(m_theoreticalRobotProfileKeySha256.left(12) + QStringLiteral("…"))
            .arg(visible
                ? QStringLiteral("已贴地放在工件 -X 侧")
                : QStringLiteral("已加载，吸附工件大面后显示"));
        if (!detail.trimmed().isEmpty())
            status += QStringLiteral("；%1").arg(detail.trimmed());
        m_theoreticalRobotStatusLabel->setText(status + QStringLiteral("。"));
        return;
    }
    m_theoreticalRobotStatusLabel->setText(detail.isEmpty()
        ? QStringLiteral("当前控制单元没有可用的已适配机器人型号；请到机器人模型库管理。")
        : QStringLiteral("机器人碰撞简模不可用：%1").arg(detail));
}

void ModelWeldingFlowDialog::SaveTemplate()
{
    if (m_template.templateId.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"),
            QStringLiteral("请先新建模板或继承相似模板。"));
        return;
    }
    if (!m_modelIdentityValid)
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"),
            QStringLiteral("绑定模型缺失或身份无效，请恢复正确模型并重新载入模板。"));
        return;
    }
    QString identityError;
    const QString currentModelHash = ModelWeldingWorkflow::ComputeFileSha256(
        ReferenceModelLibrary::ModelPath(m_template.modelLibraryName), &identityError);
    if (currentModelHash.isEmpty() || currentModelHash != m_template.modelSha256.toLower())
    {
        m_modelIdentityValid = false;
        QMessageBox::warning(this, QStringLiteral("保存模型模板"),
            currentModelHash.isEmpty() ? identityError
                : QStringLiteral("模型文件SHA-256已变化，禁止保存当前人工确认。"));
        return;
    }
    // 保存门禁不能只信任预览时缓存的 B-Rep 候选。强制重新解析当前追溯元数据和
    // 原始 STEP；文件被替换、删除或元数据变化时，RefreshPreview 会清空旧候选。
    RefreshPreview(false);
    if (!m_previewInitialized || m_preview == nullptr || !m_preview->HasCadShape())
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"),
            QStringLiteral("原始 STEP 当前无法通过身份校验和 B-Rep 复核，禁止使用旧预览候选保存。"));
        return;
    }
    RefreshGroundFaceCandidates();
    if (!m_groundFaceSatisfied)
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"),
            QStringLiteral("工件尚未用外包络大平面吸附到地面。请先在三维界面选择候选大面并执行吸附。"));
        return;
    }
    QString currentSourcePath;
    QString currentSourceSha256;
    QString seamSourceError;
    if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            m_template.modelLibraryName,
            m_template.modelSha256,
            currentSourcePath,
            currentSourceSha256,
            seamSourceError))
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"), seamSourceError);
        return;
    }
    for (const ModelWeldingSeamDefinition& seam : m_template.seams)
    {
        const QString expectedSha = seam.source
            == ModelWeldingSeamSource::ReverseMeshSeedProjection
            ? m_template.modelSha256 : currentSourceSha256;
        if (seam.sourceGeometrySha256 != expectedSha)
        {
            QMessageBox::warning(this, QStringLiteral("保存模型模板"),
                QStringLiteral("焊缝 %1 的来源几何 SHA-256 与当前模型不一致，禁止保存旧候选。")
                    .arg(seam.seamId));
            return;
        }
    }
    if (!HasActiveSimilarityInheritance()
        && !m_vSlotWorkpieceSnapped)
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"),
            QStringLiteral("普通模板的V型槽尚未吸附到工件外角。请在三维界面拖动至显示绿色后再保存。"));
        return;
    }
    if (m_templatePersisted && !m_templateDirty)
    {
        QMessageBox::information(this, QStringLiteral("保存模型模板"),
            QStringLiteral("当前模型模板没有未保存的修改。"));
        return;
    }
    m_template.displayName = m_templateNameEdit->text().trimmed();
    m_template.placement.longLengthMm = m_longLengthSpin->value();
    m_template.placement.shortLengthMm = m_shortLengthSpin->value();
    m_template.humanDatumConfirmed = m_datumChecked->isChecked();
    m_template.humanCollisionChecked = m_collisionChecked->isChecked();
    const quint64 expectedRevision = m_templatePersisted ? m_template.revision : 0;
    ModelWeldingFlowTemplate candidate = m_template;
    candidate.revision = expectedRevision + 1;
    const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    if (candidate.createdAtUtc.isEmpty())
    {
        candidate.createdAtUtc = now;
    }
    candidate.updatedAtUtc = now;
    QString error;
    if (!ModelWeldingWorkflow::SaveTemplate(candidate, expectedRevision, error))
    {
        QMessageBox::warning(this, QStringLiteral("保存模型模板"), error);
        return;
    }
    m_template = candidate;
    m_templatePersisted = true;
    m_templateDirty = false;
    m_teaching.templateId = candidate.templateId;
    m_teaching.templateRevision = candidate.revision;
    m_teaching.templateRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(candidate, error);
    m_teachingDirty = true;

    const auto it = std::find_if(m_templates.begin(), m_templates.end(),
        [this](const ModelWeldingFlowTemplate& value)
        {
            return value.templateId == m_template.templateId;
        });
    if (it == m_templates.end())
    {
        m_templates.push_back(m_template);
        m_loading = true;
        m_templateCombo->addItem(
            QStringLiteral("%1  [r%2]").arg(m_template.displayName).arg(m_template.revision),
            m_template.templateId);
        m_templateCombo->setCurrentIndex(m_templateCombo->count() - 1);
        m_loading = false;
    }
    else
    {
        *it = m_template;
        const int comboIndex = m_templateCombo->findData(m_template.templateId);
        if (comboIndex >= 0)
        {
            m_templateCombo->setItemText(comboIndex,
                QStringLiteral("%1  [r%2]").arg(m_template.displayName).arg(m_template.revision));
        }
    }
    RefreshIdentityStatus();
    SetStatus(QStringLiteral("模型模板已保存并写后回读，当前修订 %1；当前机器人示教需按此修订另行保存。"
                             "其他机器人的旧修订示教会保留但不会自动迁移，必须逐台复核。")
        .arg(m_template.revision));
}

void ModelWeldingFlowDialog::SaveTeaching()
{
    if (!m_templatePersisted)
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("请先保存模型模板，再保存与该模板修订绑定的机器人示教。"));
        return;
    }
    if (!m_modelIdentityValid)
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("绑定模型缺失或身份无效，禁止保存示教。"));
        return;
    }
    QString modelIdentityError;
    const QString currentModelHash = ModelWeldingWorkflow::ComputeFileSha256(
        ReferenceModelLibrary::ModelPath(m_template.modelLibraryName), &modelIdentityError);
    if (currentModelHash.isEmpty() || currentModelHash != m_template.modelSha256.toLower())
    {
        m_modelIdentityValid = false;
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            currentModelHash.isEmpty() ? modelIdentityError
                : QStringLiteral("模型文件SHA-256已变化，禁止保存示教。"));
        return;
    }
    m_teaching.templateId = m_template.templateId;
    m_teaching.templateRevision = m_template.revision;
    QString error;
    m_teaching.templateRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(m_template, error);
    m_teaching.robotName = CurrentRobotName();
    QString currentRobotModelId;
    QString currentSourceStepSha256;
    QString currentCollisionProfileSha256;
    if (!ReadCurrentRobotModelIdentity(
            currentRobotModelId,
            currentSourceStepSha256,
            currentCollisionProfileSha256,
            error))
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("当前机器人型号资产身份无法核验：%1").arg(error));
        return;
    }
    const bool teachingHasRobotModelIdentity = !m_teaching.robotModelId.isEmpty()
        || !m_teaching.sourceStepSha256.isEmpty()
        || !m_teaching.collisionProfileSha256.isEmpty();
    if (teachingHasRobotModelIdentity
        && (m_teaching.robotModelId != currentRobotModelId
            || m_teaching.sourceStepSha256 != currentSourceStepSha256
            || m_teaching.collisionProfileSha256 != currentCollisionProfileSha256))
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("当前机器人型号、源STEP或碰撞简模已与示教草稿不同，"
                           "禁止覆盖保存；请重新载入并重新示教。"));
        return;
    }
    if (m_teachingPersisted && !m_teachingDirty)
    {
        QMessageBox::information(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("当前机器人示教没有未保存的修改，且机器人型号资产身份仍一致。"));
        return;
    }
    // v1/v2 记录只允许以迁移草稿读入；每次保存都必须写入当前目录解析出的
    // 完整型号资产身份，绝不沿用缺失或调用方猜测的值。
    m_teaching.robotModelId = currentRobotModelId;
    m_teaching.sourceStepSha256 = currentSourceStepSha256;
    m_teaching.collisionProfileSha256 = currentCollisionProfileSha256;
    const QString currentEndpoint = CurrentRobotEndpoint();
    if (currentEndpoint.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("当前机器人缺少可持久化端点身份，拒绝保存。"));
        return;
    }
    if (!m_teaching.robotEndpoint.isEmpty()
        && m_teaching.robotEndpoint != currentEndpoint)
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"),
            QStringLiteral("当前机器人端点已与示教记录不同，请重新绑定手眼和Tool1。"));
        return;
    }
    m_teaching.robotEndpoint = currentEndpoint;
    m_teaching.cameraSection = CurrentCameraSection();
    const quint64 expectedRevision = m_teachingPersisted ? m_teaching.revision : 0;
    ModelWeldingRobotTeaching candidate = m_teaching;
    candidate.revision = expectedRevision + 1;
    const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    if (candidate.createdAtUtc.isEmpty())
    {
        candidate.createdAtUtc = now;
    }
    candidate.updatedAtUtc = now;
    if (!ModelWeldingWorkflow::SaveTeaching(candidate, expectedRevision, error))
    {
        QMessageBox::warning(this, QStringLiteral("保存机器人示教"), error);
        return;
    }
    m_teaching = candidate;
    m_teachingPersisted = true;
    m_teachingDirty = false;
    const auto it = std::find_if(m_teachings.begin(), m_teachings.end(),
        [this](const ModelWeldingRobotTeaching& value)
        {
            return value.teachingId == m_teaching.teachingId;
        });
    if (it == m_teachings.end())
    {
        m_teachings.push_back(m_teaching);
    }
    else
    {
        *it = m_teaching;
    }
    SetStatus(QStringLiteral("当前机器人示教已保存并写后回读，修订 %1。")
        .arg(m_teaching.revision));
}

void ModelWeldingFlowDialog::BindRuntimeIdentity()
{
    if (m_template.templateId.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("运行绑定"),
            QStringLiteral("请先新建或选择模型流程模板。"));
        return;
    }
    QString robotEndpoint;
    QString handEyeSha256;
    QString tool1Sha256;
    QString robotModelId;
    QString sourceStepSha256;
    QString collisionProfileSha256;
    QString error;
    if (!ReadCurrentRobotModelIdentity(
            robotModelId,
            sourceStepSha256,
            collisionProfileSha256,
            error))
    {
        QMessageBox::warning(this, QStringLiteral("运行绑定"),
            QStringLiteral("当前机器人型号资产身份无法核验：%1").arg(error));
        return;
    }
    if (!ReadCurrentRuntimeIdentity(robotEndpoint, handEyeSha256, tool1Sha256, error))
    {
        QMessageBox::warning(this, QStringLiteral("运行绑定"), error);
        return;
    }
    m_teaching.robotName = CurrentRobotName();
    m_teaching.robotEndpoint = robotEndpoint;
    m_teaching.robotModelId = robotModelId;
    m_teaching.sourceStepSha256 = sourceStepSha256;
    m_teaching.collisionProfileSha256 = collisionProfileSha256;
    m_teaching.cameraSection = CurrentCameraSection();
    m_teaching.handEyeSha256 = handEyeSha256;
    m_teaching.tool1Sha256 = tool1Sha256;
    m_teachingDirty = true;
    RefreshIdentityStatus();
    SetStatus(QStringLiteral(
        "已只读绑定当前机器人型号资产、端点、手眼参数和控制器Tool1；保存机器人示教后才会持久化。"));
}

void ModelWeldingFlowDialog::CheckProductionReadiness()
{
    if (!m_templatePersisted || !m_teachingPersisted
        || m_templateDirty || m_teachingDirty)
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
            QStringLiteral("模型模板和当前机器人示教都必须保存，且不能存在未保存修改。"));
        return;
    }
    QString currentRobotModelId = CurrentRobotModelId();
    int configuredRobotType = CurrentConfiguredRobotType();
    const int selectedUnitIndex = m_robotCombo != nullptr
        && m_robotCombo->currentIndex() >= 0
        ? m_robotCombo->currentData().toInt() : -1;
    RobotDriverAdaptor* driver = CurrentDriver();
    RobotModelCatalogStore::Eligibility robotEligibility;
    QString robotModelError;
    const QVector<RobotDataHelper::RobotInfo> currentRobots =
        RobotDataHelper::LoadRobotList(m_contralUnit);
    const auto currentRobot = std::find_if(
        currentRobots.cbegin(), currentRobots.cend(),
        [selectedUnitIndex](const RobotDataHelper::RobotInfo& robot)
        {
            return robot.unitIndex == selectedUnitIndex;
        });
    if (selectedUnitIndex < 0 || currentRobot == currentRobots.cend())
    {
        robotModelError = QStringLiteral("当前机器人已不在控制单元列表中。");
    }
    else if (currentRobot->robotName != CurrentRobotName()
        || currentRobot->robotModelId.trimmed().toLower() != currentRobotModelId
        || currentRobot->robotType != configuredRobotType)
    {
        robotModelError = QStringLiteral(
            "控制单元的机器人身份、类型或型号已变化，请关闭并重新进入模型焊接流程。");
    }
    else
    {
        currentRobotModelId = currentRobot->robotModelId.trimmed().toLower();
        configuredRobotType = currentRobot->robotType;
    }
    if (robotModelError.isEmpty() && driver == nullptr)
    {
        robotModelError = QStringLiteral("当前机器人没有真实驱动。");
    }
    else if (robotModelError.isEmpty()
        && !driver->SupportsAll(
            { RobotDriverCapability::JointMotion,
              RobotDriverCapability::LinearMotion,
              RobotDriverCapability::PassiveState,
              RobotDriverCapability::ContinuousTrajectory,
              RobotDriverCapability::VerifiedProgramCompletion,
              RobotDriverCapability::VerifiedSafeAbort }))
    {
        robotModelError = QStringLiteral(
            "当前机器人品牌底层不能满足模型先测后焊，缺少适配能力：%1；生产入口已限制。")
            .arg(QString::fromUtf8(driver->MissingCapabilitiesText(
                { RobotDriverCapability::JointMotion,
                  RobotDriverCapability::LinearMotion,
                  RobotDriverCapability::PassiveState,
                  RobotDriverCapability::ContinuousTrajectory,
                  RobotDriverCapability::VerifiedProgramCompletion,
                  RobotDriverCapability::VerifiedSafeAbort }).c_str()));
    }
    else if (robotModelError.isEmpty() && currentRobotModelId.isEmpty())
    {
        robotModelError = QStringLiteral("当前控制单元未设置机器人型号。");
    }
    else if (robotModelError.isEmpty()
        && (configuredRobotType < 0 || configuredRobotType != driver->RobotType()))
    {
        robotModelError = QStringLiteral("当前控制单元机器人类型与实际驱动不一致。");
    }
    else if (robotModelError.isEmpty()
        && !RobotModelCatalogStore::ResolveModelEligibility(
                 currentRobotModelId,
                 driver->RobotType(),
                 robotEligibility,
                 robotModelError))
    {
        if (robotModelError.trimmed().isEmpty())
            robotModelError = QStringLiteral("机器人模型目录无法可信读取。");
    }
    else if (robotModelError.isEmpty() && !robotEligibility.eligible)
    {
        robotModelError = robotEligibility.reason.trimmed().isEmpty()
            ? QStringLiteral("当前机器人型号已不具备模型焊接资格。")
            : robotEligibility.reason.trimmed();
    }

    if (robotModelError.isEmpty())
    {
        const QString resolvedModelId =
            robotEligibility.model.modelId.trimmed().toLower();
        const QString resolvedSourceSha256 =
            robotEligibility.model.sourceStep.sha256.trimmed().toLower();
        const QString resolvedProfileKeySha256 =
            robotEligibility.model.collision.profileKeySha256.trimmed().toLower();
        const RobotCollisionEnvelopeStore::EnvelopeSet& envelope =
            robotEligibility.collisionEnvelope;
        if (resolvedModelId != currentRobotModelId
            || robotEligibility.model.sourceRobotType != configuredRobotType
            || resolvedSourceSha256 != envelope.sourceStepSha256.trimmed().toLower()
            || resolvedProfileKeySha256 != envelope.profileKeySha256.trimmed().toLower()
            || m_theoreticalRobotModelId != currentRobotModelId
            || m_theoreticalRobotSha256 != resolvedSourceSha256
            || m_theoreticalRobotProfileKeySha256 != resolvedProfileKeySha256
            || m_preview == nullptr
            || !m_preview->IsRobotCollisionEnvelope())
        {
            robotModelError = QStringLiteral(
                "当前加载的机器人型号、源 STEP SHA 或碰撞简模配置与控制单元不一致。");
        }
    }
    if (robotModelError.isEmpty()
        && (m_teaching.robotModelId != robotEligibility.model.modelId.trimmed().toLower()
            || m_teaching.sourceStepSha256
                != robotEligibility.model.sourceStep.sha256.trimmed().toLower()
            || m_teaching.collisionProfileSha256
                != robotEligibility.model.collision.profileKeySha256.trimmed().toLower()))
    {
        robotModelError = QStringLiteral(
            "保存示教绑定的机器人型号、源STEP或碰撞简模配置与当前控制单元不一致；"
            "旧版或失配记录必须重新示教/确认、运行绑定并保存。");
    }
    if (!robotModelError.isEmpty())
    {
        ClearTheoreticalRobotModel(robotModelError);
        QMessageBox::warning(
            this, QStringLiteral("配置生产就绪检查"), robotModelError);
        return;
    }
    // 生产门禁同样必须重新核验当前源 STEP，不能沿用可能已经过期的内存候选。
    RefreshPreview(false);
    if (!m_previewInitialized || m_preview == nullptr || !m_preview->HasCadShape())
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
            QStringLiteral("原始 STEP 当前无法通过身份校验和 B-Rep 复核，拒绝生产就绪。"));
        return;
    }
    RefreshGroundFaceCandidates();
    if (!m_groundFaceSatisfied)
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
            QStringLiteral("工件落地姿态已失效：必须有一个经 STEP 外包络验证的大平面与地面重合。"));
        return;
    }
    if (!HasActiveSimilarityInheritance()
        && !m_vSlotWorkpieceSnapped)
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
            QStringLiteral("普通模板的V型槽与工件外角已脱离，拒绝生产就绪。"));
        return;
    }
    QString error;
    const QString currentModelHash = ModelWeldingWorkflow::ComputeFileSha256(
        ReferenceModelLibrary::ModelPath(m_template.modelLibraryName), &error);
    if (currentModelHash.isEmpty() || currentModelHash != m_template.modelSha256.toLower())
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
            currentModelHash.isEmpty() ? error : QStringLiteral("模型文件SHA-256已变化，模板失效。"));
        return;
    }
    QString currentSourcePath;
    QString currentSourceSha256;
    if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            m_template.modelLibraryName,
            m_template.modelSha256,
            currentSourcePath,
            currentSourceSha256,
            error))
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"), error);
        return;
    }
    for (const ModelWeldingSeamDefinition& seam : m_template.seams)
    {
        const QString expectedSha = seam.source
            == ModelWeldingSeamSource::ReverseMeshSeedProjection
            ? currentModelHash : currentSourceSha256;
        if (seam.sourceGeometrySha256 != expectedSha)
        {
            QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
                QStringLiteral("焊缝 %1 的来源几何 SHA-256 与当前模型不一致。")
                    .arg(seam.seamId));
            return;
        }
    }
    if (!ModelWeldingWorkflow::ValidateTeachingStructure(
        m_teaching, &m_template, error, true))
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"), error);
        return;
    }
    QString currentRobotEndpoint;
    QString currentHandEyeSha256;
    QString currentTool1Sha256;
    if (!ReadCurrentRuntimeIdentity(
        currentRobotEndpoint, currentHandEyeSha256, currentTool1Sha256, error))
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"), error);
        return;
    }
    if (currentRobotEndpoint != m_teaching.robotEndpoint
        || currentHandEyeSha256 != m_teaching.handEyeSha256.toLower()
        || currentTool1Sha256 != m_teaching.tool1Sha256.toLower())
    {
        QMessageBox::warning(this, QStringLiteral("配置生产就绪检查"),
            QStringLiteral("当前机器人端点、手眼参数或控制器Tool1已与保存示教时不同，请重新绑定并保存。"));
        return;
    }
    QMessageBox::information(this, QStringLiteral("配置生产就绪检查"),
        QStringLiteral("模板、模型身份、已确认焊缝、3个求解站、1个验证站、人工示教及运行绑定均已通过配置检查。\n\n"
                       "注意：多站点云纯采集、特征拟合、生产证明和自动焊接执行尚未接通，"
                       "因此本版本仍不会启动机器人。"));
}

QString ModelWeldingFlowDialog::CurrentRobotName() const
{
    return m_robotCombo != nullptr && m_robotCombo->currentIndex() >= 0
        ? m_robotCombo->currentData(kRobotNameRole).toString().trimmed()
        : QString();
}

QString ModelWeldingFlowDialog::CurrentRobotModelId() const
{
    return m_robotCombo != nullptr && m_robotCombo->currentIndex() >= 0
        ? m_robotCombo->currentData(kRobotModelIdRole).toString().trimmed().toLower()
        : QString();
}

int ModelWeldingFlowDialog::CurrentConfiguredRobotType() const
{
    if (m_robotCombo == nullptr || m_robotCombo->currentIndex() < 0)
    {
        return -1;
    }
    bool ok = false;
    const int robotType =
        m_robotCombo->currentData(kRobotTypeRole).toInt(&ok);
    return ok ? robotType : -1;
}

QString ModelWeldingFlowDialog::CurrentRobotEndpoint() const
{
    RobotDriverAdaptor* driver = CurrentDriver();
    return driver != nullptr
        ? RobotOperationLease::PersistentEndpointIdentity(driver).trimmed()
        : QString();
}

QString ModelWeldingFlowDialog::CurrentCameraSection() const
{
    return m_cameraCombo != nullptr && m_cameraCombo->currentIndex() >= 0
        ? m_cameraCombo->currentData().toString().trimmed()
        : QString();
}

QString ModelWeldingFlowDialog::CurrentStationId() const
{
    const int row = m_stationTable != nullptr ? m_stationTable->currentRow() : -1;
    return row >= 0 && row < m_template.stations.size()
        ? m_template.stations.at(row).stationId
        : QString();
}

RobotDriverAdaptor* ModelWeldingFlowDialog::CurrentDriver() const
{
    if (m_contralUnit == nullptr || m_robotCombo == nullptr || m_robotCombo->currentIndex() < 0)
    {
        return nullptr;
    }
    return RobotDataHelper::GetRobotDriver(
        m_contralUnit, m_robotCombo->currentData().toInt());
}

bool ModelWeldingFlowDialog::ReadCurrentRobotModelIdentity(
    QString& robotModelId,
    QString& sourceStepSha256,
    QString& collisionProfileSha256,
    QString& error) const
{
    robotModelId.clear();
    sourceStepSha256.clear();
    collisionProfileSha256.clear();
    error.clear();
    if (m_robotCombo == nullptr || m_robotCombo->currentIndex() < 0)
    {
        error = QStringLiteral("当前未选择机器人。");
        return false;
    }
    bool unitIndexOk = false;
    const int selectedUnitIndex = m_robotCombo->currentData().toInt(&unitIndexOk);
    if (!unitIndexOk || selectedUnitIndex < 0)
    {
        error = QStringLiteral("当前机器人控制单元索引无效。");
        return false;
    }
    const QVector<RobotDataHelper::RobotInfo> currentRobots =
        RobotDataHelper::LoadRobotList(m_contralUnit);
    const auto currentRobot = std::find_if(
        currentRobots.cbegin(), currentRobots.cend(),
        [selectedUnitIndex](const RobotDataHelper::RobotInfo& robot)
        {
            return robot.unitIndex == selectedUnitIndex;
        });
    if (currentRobot == currentRobots.cend()
        || currentRobot->robotName != CurrentRobotName())
    {
        error = QStringLiteral("当前机器人已不在控制单元列表中或身份已变化。");
        return false;
    }
    RobotDriverAdaptor* driver = CurrentDriver();
    if (driver == nullptr)
    {
        error = QStringLiteral("当前机器人没有真实驱动。");
        return false;
    }
    const QString configuredModelId = currentRobot->robotModelId.trimmed().toLower();
    const int configuredRobotType = currentRobot->robotType;
    if (configuredModelId.isEmpty())
    {
        error = QStringLiteral("当前控制单元未设置机器人型号。");
        return false;
    }
    if (configuredRobotType < 0 || configuredRobotType != driver->RobotType())
    {
        error = QStringLiteral("当前控制单元机器人类型与实际驱动不一致。");
        return false;
    }
    if (configuredModelId != CurrentRobotModelId()
        || configuredRobotType != CurrentConfiguredRobotType())
    {
        error = QStringLiteral("控制单元机器人型号或类型已变化，请关闭并重新进入流程。");
        return false;
    }

    RobotModelCatalogStore::Eligibility eligibility;
    if (!RobotModelCatalogStore::ResolveModelEligibility(
            configuredModelId, driver->RobotType(), eligibility, error))
    {
        if (error.trimmed().isEmpty())
        {
            error = QStringLiteral("机器人模型目录无法可信读取。");
        }
        return false;
    }
    if (!eligibility.eligible)
    {
        error = eligibility.reason.trimmed().isEmpty()
            ? QStringLiteral("当前机器人型号已不具备模型焊接资格。")
            : eligibility.reason.trimmed();
        return false;
    }
    const QString resolvedModelId = eligibility.model.modelId.trimmed().toLower();
    const QString resolvedSourceStepSha256 =
        eligibility.model.sourceStep.sha256.trimmed().toLower();
    const QString resolvedCollisionProfileSha256 =
        eligibility.model.collision.profileKeySha256.trimmed().toLower();
    if (resolvedModelId != configuredModelId
        || eligibility.model.sourceRobotType != configuredRobotType
        || resolvedSourceStepSha256.isEmpty()
        || resolvedSourceStepSha256
            != eligibility.collisionEnvelope.sourceStepSha256.trimmed().toLower()
        || resolvedCollisionProfileSha256.isEmpty()
        || resolvedCollisionProfileSha256
            != eligibility.collisionEnvelope.profileKeySha256.trimmed().toLower())
    {
        error = QStringLiteral("机器人型号目录、源STEP与碰撞简模身份不一致。");
        return false;
    }
    robotModelId = resolvedModelId;
    sourceStepSha256 = resolvedSourceStepSha256;
    collisionProfileSha256 = resolvedCollisionProfileSha256;
    return true;
}

bool ModelWeldingFlowDialog::ReadCurrentRuntimeIdentity(
    QString& robotEndpoint,
    QString& handEyeSha256,
    QString& tool1Sha256,
    QString& error) const
{
    robotEndpoint.clear();
    handEyeSha256.clear();
    tool1Sha256.clear();
    error.clear();
    RobotDriverAdaptor* driver = CurrentDriver();
    if (driver == nullptr)
    {
        error = QStringLiteral("当前机器人没有可用驱动。");
        return false;
    }
    if (!driver->Supports(RobotDriverCapability::ToolDataRead))
    {
        error = QStringLiteral(
            "当前机器人品牌底层缺少“工具数据读取”适配能力，无法绑定控制器Tool1，功能已限制。");
        return false;
    }
    const QString firstEndpoint =
        RobotOperationLease::PersistentEndpointIdentity(driver).trimmed();
    if (firstEndpoint.isEmpty())
    {
        error = QStringLiteral("当前机器人缺少可持久化端点身份。");
        return false;
    }
    const QString robotName = CurrentRobotName();
    const QString cameraSection = CurrentCameraSection();
    if (cameraSection.isEmpty())
    {
        error = QStringLiteral("当前机器人没有可用定位相机。");
        return false;
    }

    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("模型焊接流程-只读核验手眼和Tool1"), &leaseError);
    if (!lease)
    {
        error = leaseError;
        return false;
    }

    HandEyeMatrixConfig firstHandEye;
    if (!LoadExistingValidatedHandEyeMatrixConfig(
        robotName, cameraSection, firstHandEye, &error, nullptr))
    {
        error = QStringLiteral("手眼参数未达到运动使用条件：%1").arg(error);
        return false;
    }
    const QString firstHandEyeSha256 =
        ModelWeldingWorkflow::ComputeHandEyeSha256(firstHandEye);

    T_ROBOT_COORS firstTool1;
    if (!driver->GetToolData(1, firstTool1))
    {
        const QString detail = DecodeRobotMessageText(driver->GetLastRobotError()).trimmed();
        error = detail.isEmpty()
            ? QStringLiteral("读取控制器Tool1失败，请确认机器人连接和SDK接口。")
            : QStringLiteral("读取控制器Tool1失败：%1").arg(detail);
        return false;
    }
    if (!Tool1IsFiniteAndReasonable(firstTool1, error))
    {
        return false;
    }

    HandEyeMatrixConfig secondHandEye;
    QString secondError;
    if (!LoadExistingValidatedHandEyeMatrixConfig(
        robotName, cameraSection, secondHandEye, &secondError, nullptr))
    {
        error = QStringLiteral("运行绑定期间手眼参数失效：%1").arg(secondError);
        return false;
    }
    const QString secondHandEyeSha256 =
        ModelWeldingWorkflow::ComputeHandEyeSha256(secondHandEye);
    if (firstHandEyeSha256 != secondHandEyeSha256)
    {
        error = QStringLiteral("读取期间手眼参数发生变化，请等待参数稳定后重试。");
        return false;
    }
    T_ROBOT_COORS secondTool1;
    if (!driver->GetToolData(1, secondTool1))
    {
        const QString detail = DecodeRobotMessageText(driver->GetLastRobotError()).trimmed();
        error = detail.isEmpty()
            ? QStringLiteral("二次读取控制器Tool1失败，请确认机器人连接和SDK接口。")
            : QStringLiteral("二次读取控制器Tool1失败：%1").arg(detail);
        return false;
    }
    if (!Tool1IsFiniteAndReasonable(secondTool1, error))
    {
        return false;
    }
    const QString firstTool1Sha256 = ModelWeldingWorkflow::ComputeTool1Sha256(firstTool1);
    const QString secondTool1Sha256 = ModelWeldingWorkflow::ComputeTool1Sha256(secondTool1);
    if (firstTool1Sha256 != secondTool1Sha256)
    {
        error = QStringLiteral("读取期间控制器Tool1发生变化，请等待参数稳定后重试。");
        return false;
    }
    const QString secondEndpoint =
        RobotOperationLease::PersistentEndpointIdentity(driver).trimmed();
    if (firstEndpoint != secondEndpoint)
    {
        error = QStringLiteral("读取期间机器人端点身份发生变化，请重试。");
        return false;
    }
    robotEndpoint = firstEndpoint;
    handEyeSha256 = firstHandEyeSha256;
    tool1Sha256 = firstTool1Sha256;
    return true;
}

ModelWeldingScanTeaching* ModelWeldingFlowDialog::EnsureTeaching(const QString& stationId)
{
    if (stationId.isEmpty())
    {
        return nullptr;
    }
    for (ModelWeldingScanTeaching& scan : m_teaching.scans)
    {
        if (scan.stationId == stationId)
        {
            return &scan;
        }
    }
    ModelWeldingScanTeaching scan;
    scan.stationId = stationId;
    m_teaching.scans.push_back(scan);
    return &m_teaching.scans.last();
}

const ModelWeldingScanTeaching* ModelWeldingFlowDialog::FindTeaching(const QString& stationId) const
{
    for (const ModelWeldingScanTeaching& scan : m_teaching.scans)
    {
        if (scan.stationId == stationId)
        {
            return &scan;
        }
    }
    return nullptr;
}

void ModelWeldingFlowDialog::TeachStart()
{
    if (!m_modelIdentityValid)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"),
            QStringLiteral("绑定模型缺失或身份无效，禁止在错误几何上示教。"));
        return;
    }
    const QString stationId = CurrentStationId();
    RobotDriverAdaptor* driver = CurrentDriver();
    if (stationId.isEmpty() || driver == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"),
            QStringLiteral("请先选择扫描站，并确认当前机器人驱动可用。"));
        return;
    }
    if (!driver->Supports(RobotDriverCapability::PassiveState))
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"),
            QStringLiteral("当前机器人品牌底层缺少“机器人状态读取”适配能力，示教功能已限制。"));
        return;
    }
    const QString endpoint = RobotOperationLease::PersistentEndpointIdentity(driver).trimmed();
    if (endpoint.isEmpty()
        || (!m_teaching.robotEndpoint.isEmpty() && m_teaching.robotEndpoint != endpoint))
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"),
            QStringLiteral("当前机器人端点与示教记录不匹配，请重新选择或绑定。"));
        return;
    }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("模型焊接流程-读取示教起点"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"), leaseError);
        return;
    }
    T_ROBOT_COORS pose;
    T_ANGLE_PULSE pulse;
    if (!driver->TryGetCurrentPos(pose) || !driver->TryGetCurrentPulse(pulse))
    {
        const QString detail = DecodeRobotMessageText(driver->GetLastRobotError()).trimmed();
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"), detail.isEmpty()
            ? QStringLiteral("机器人未返回可验证的当前位置和关节脉冲。") : detail);
        return;
    }
    if (RobotOperationLease::PersistentEndpointIdentity(driver).trimmed() != endpoint)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描起点"),
            QStringLiteral("读取姿态期间机器人端点身份发生变化，本次示教已丢弃。"));
        return;
    }
    m_teaching.robotEndpoint = endpoint;
    ModelWeldingScanTeaching* scan = EnsureTeaching(stationId);
    scan->startPose = pose;
    scan->startTaught = true;
    scan->startPulse = pulse;
    scan->startPulseTaught = true;
    scan->runSpeedMmPerMin = m_runSpeedSpin->value();
    scan->scanSpeedMmPerMin = m_scanSpeedSpin->value();
    m_teachingDirty = true;
    RefreshStationTable();
    RefreshStationDetails();
    SetStatus(QStringLiteral("站点 %1 的扫描起点和起点关节脉冲已读取，尚未保存。")
        .arg(stationId));
}

void ModelWeldingFlowDialog::TeachEnd()
{
    if (!m_modelIdentityValid)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"),
            QStringLiteral("绑定模型缺失或身份无效，禁止在错误几何上示教。"));
        return;
    }
    const QString stationId = CurrentStationId();
    RobotDriverAdaptor* driver = CurrentDriver();
    if (stationId.isEmpty() || driver == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"),
            QStringLiteral("请先选择扫描站，并确认当前机器人驱动可用。"));
        return;
    }
    if (!driver->Supports(RobotDriverCapability::PassiveState))
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"),
            QStringLiteral("当前机器人品牌底层缺少“机器人状态读取”适配能力，示教功能已限制。"));
        return;
    }
    const QString endpoint = RobotOperationLease::PersistentEndpointIdentity(driver).trimmed();
    if (endpoint.isEmpty()
        || (!m_teaching.robotEndpoint.isEmpty() && m_teaching.robotEndpoint != endpoint))
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"),
            QStringLiteral("当前机器人端点与示教记录不匹配，请重新选择或绑定。"));
        return;
    }
    QString leaseError;
    const auto lease = RobotOperationLease::TryAcquire(
        driver, QStringLiteral("模型焊接流程-读取示教终点"), &leaseError);
    if (!lease)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"), leaseError);
        return;
    }
    T_ROBOT_COORS pose;
    if (!driver->TryGetCurrentPos(pose))
    {
        const QString detail = DecodeRobotMessageText(driver->GetLastRobotError()).trimmed();
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"), detail.isEmpty()
            ? QStringLiteral("机器人未返回可验证的当前位置。") : detail);
        return;
    }
    if (RobotOperationLease::PersistentEndpointIdentity(driver).trimmed() != endpoint)
    {
        QMessageBox::warning(this, QStringLiteral("示教扫描终点"),
            QStringLiteral("读取姿态期间机器人端点身份发生变化，本次示教已丢弃。"));
        return;
    }
    m_teaching.robotEndpoint = endpoint;
    ModelWeldingScanTeaching* scan = EnsureTeaching(stationId);
    scan->endPose = pose;
    scan->endTaught = true;
    scan->runSpeedMmPerMin = m_runSpeedSpin->value();
    scan->scanSpeedMmPerMin = m_scanSpeedSpin->value();
    m_teachingDirty = true;
    RefreshStationTable();
    RefreshStationDetails();
    SetStatus(QStringLiteral("站点 %1 的扫描终点已读取，尚未保存。")
        .arg(stationId));
}

void ModelWeldingFlowDialog::ClearSeamCandidates()
{
    m_seamCandidates.clear();
    m_seamCandidateSourcePath.clear();
    m_seamCandidateSourceSha256.clear();
    if (m_seamCandidateCombo != nullptr)
    {
        const QSignalBlocker blocker(m_seamCandidateCombo);
        m_seamCandidateCombo->clear();
    }
}

void ModelWeldingFlowDialog::ExtractCadSeamCandidates()
{
    if (m_template.templateId.isEmpty() || !m_modelIdentityValid)
    {
        QMessageBox::warning(this, QStringLiteral("提取焊缝候选"),
            QStringLiteral("请先新建或载入身份有效的模型模板。"));
        return;
    }
    QString sourcePath;
    QString sourceSha256;
    QString error;
    if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            m_template.modelLibraryName,
            m_template.modelSha256,
            sourcePath,
            sourceSha256,
            error))
    {
        ClearSeamCandidates();
        QMessageBox::warning(this, QStringLiteral("提取焊缝候选"), error);
        return;
    }

    QVector<CadSeamCandidateExtractor::Candidate> extracted;
    CadSeamCandidateExtractor::Statistics statistics;
    CadSeamCandidateExtractor::Options options;
    options.minimumLengthMm = m_seamMinimumLengthSpin->value();
    options.samplingStepMm = 2.0;
    bool success = false;
    NonClosableProgressDialog progress(
        QStringLiteral("正在只读解析源 STEP，并识别板间对接立焊与下端波纹平焊…"),
        QString(), 0, 0, this);
    progress.setWindowTitle(QStringLiteral("提取焊缝候选"));
    progress.setWindowModality(Qt::WindowModal);
    progress.setCancelButton(nullptr);
    progress.setMinimumDuration(0);
    progress.setAutoClose(false);
    progress.setAutoReset(false);
    progress.setMinimumWidth(560);
    progress.setWindowFlag(Qt::WindowCloseButtonHint, false);

    QEventLoop waitLoop;
    QThread* worker = QThread::create([&]()
        {
            try
            {
                success = CadSeamCandidateExtractor::ExtractFromStepFile(
                    sourcePath, extracted, error, &statistics, &options);
            }
            catch (const std::exception& exception)
            {
                error = QStringLiteral("焊缝候选提取线程发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
                success = false;
            }
            catch (...)
            {
                error = QStringLiteral("焊缝候选提取线程发生未知异常。");
                success = false;
            }
        });
    connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
    connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
    progress.show();
    worker->start();
    waitLoop.exec();
    worker->wait();
    delete worker;
    progress.accept();

    if (!success)
    {
        ClearSeamCandidates();
        QMessageBox::warning(this, QStringLiteral("提取焊缝候选"), error);
        return;
    }
    QString verifiedPath;
    QString verifiedSha256;
    if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            m_template.modelLibraryName,
            m_template.modelSha256,
            verifiedPath,
            verifiedSha256,
            error)
        || verifiedPath != sourcePath || verifiedSha256 != sourceSha256)
    {
        ClearSeamCandidates();
        QMessageBox::warning(this, QStringLiteral("提取焊缝候选"),
            error.isEmpty()
                ? QStringLiteral("提取期间源 STEP 身份发生变化，候选已丢弃。") : error);
        return;
    }

    m_seamCandidates = std::move(extracted);
    m_seamCandidateSourcePath = sourcePath;
    m_seamCandidateSourceSha256 = sourceSha256;
    m_loading = true;
    m_seamCandidateCombo->clear();
    for (int index = 0; index < m_seamCandidates.size(); ++index)
    {
        const auto& candidate = m_seamCandidates.at(index);
        QString description = QStringLiteral("%1 | %2 | %3 mm")
            .arg(candidate.candidateId, CandidateSourceText(candidate.sourceKind))
            .arg(candidate.lengthMm, 0, 'f', 1);
        if (candidate.sourceKind == CadSeamCandidateExtractor::SourceKind::SharedEdge)
            description += QStringLiteral(" | %1°").arg(
                candidate.dihedralDegrees, 0, 'f', 1);
        m_seamCandidateCombo->addItem(description, index);
    }
    m_loading = false;
    RefreshPreview(true);
    SetStatus(QStringLiteral(
        "已从源 STEP 提取 %1 条候选：板间对接立焊 %2 条、下端波纹平焊 %3 条；"
        "装配语义=%4，共边候选=%5。当前只预览，不会自动确认或生成机器人轨迹。")
        .arg(m_seamCandidates.size())
        .arg(statistics.corrugatedButtJointCount)
        .arg(statistics.corrugatedBaseJointCount)
        .arg(statistics.assemblySemanticsUsed
            ? QStringLiteral("已采用") : QStringLiteral("未识别/已回退"))
        .arg(statistics.sharedEdgeCount));
}

void ModelWeldingFlowDialog::AddSelectedCadSeamCandidate()
{
    const int comboIndex = m_seamCandidateCombo != nullptr
        ? m_seamCandidateCombo->currentIndex() : -1;
    const int candidateIndex = comboIndex >= 0
        ? m_seamCandidateCombo->itemData(comboIndex).toInt() : -1;
    if (candidateIndex < 0 || candidateIndex >= m_seamCandidates.size())
    {
        QMessageBox::information(this, QStringLiteral("加入焊缝模板"),
            QStringLiteral("请先提取并选择一条候选。"));
        return;
    }
    QString sourcePath;
    QString sourceSha256;
    QString error;
    if (!m_modelIdentityValid
        || !ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            m_template.modelLibraryName,
            m_template.modelSha256,
            sourcePath,
            sourceSha256,
            error)
        || sourcePath != m_seamCandidateSourcePath
        || sourceSha256 != m_seamCandidateSourceSha256)
    {
        ClearSeamCandidates();
        QMessageBox::warning(this, QStringLiteral("加入焊缝模板"),
            error.isEmpty()
                ? QStringLiteral("源 STEP 身份已变化，请重新提取候选。") : error);
        RefreshPreview(true);
        return;
    }
    const auto& candidate = m_seamCandidates.at(candidateIndex);
    const QString seamId = candidate.candidateId.toLower();
    const auto duplicate = std::find_if(
        m_template.seams.cbegin(), m_template.seams.cend(),
        [&seamId](const ModelWeldingSeamDefinition& seam)
        {
            return seam.seamId == seamId;
        });
    if (duplicate != m_template.seams.cend())
    {
        QMessageBox::information(this, QStringLiteral("加入焊缝模板"),
            QStringLiteral("该候选已在当前模板中。"));
        return;
    }

    ModelWeldingSeamDefinition seam;
    seam.seamId = seamId;
    switch (candidate.sourceKind)
    {
    case CadSeamCandidateExtractor::SourceKind::ShapeIntersection:
        seam.source = ModelWeldingSeamSource::CadShapeIntersection;
        break;
    case CadSeamCandidateExtractor::SourceKind::CorrugatedButtJoint:
        seam.source = ModelWeldingSeamSource::CadCorrugatedButtJoint;
        break;
    case CadSeamCandidateExtractor::SourceKind::CorrugatedBaseJoint:
        seam.source = ModelWeldingSeamSource::CadCorrugatedBaseJoint;
        break;
    default:
        seam.source = ModelWeldingSeamSource::CadSharedEdge;
        break;
    }
    seam.sourceGeometrySha256 = sourceSha256;
    seam.pathModelMm = candidate.pointsModelMm;
    seam.lengthMm = PolylineLength(seam.pathModelMm);
    seam.humanConfirmed = false;
    ModelWeldingFlowTemplate candidateTemplate = m_template;
    candidateTemplate.seams.push_back(seam);
    if (!ModelWeldingWorkflow::ValidateTemplateStructure(candidateTemplate, error, false)
        || ModelWeldingWorkflow::EncodeTemplate(candidateTemplate, error).isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("加入焊缝模板"), error);
        return;
    }
    m_template = std::move(candidateTemplate);
    m_templateDirty = true;
    RefreshSeamTable();
    SetStatus(QStringLiteral(
        "候选 %1 已加入模板，当前仍未确认。请在三维视图核对后勾选“确认”。")
        .arg(candidate.candidateId));
}

void ModelWeldingFlowDialog::ProjectReverseMeshSeedFile()
{
    bool addToTemplate = !m_template.templateId.isEmpty()
        && m_modelIdentityValid && m_mesh.IsValid();
    QString projectionModelName;
    QString currentModelHash;
    WorkpieceMeshBuilder::Mesh projectionMesh;
    QString error;
    if (addToTemplate)
    {
        projectionModelName = m_template.modelLibraryName;
        currentModelHash = m_template.modelSha256;
        projectionMesh = m_mesh;
        QString confirmedStepPath;
        QString confirmedStepSha256;
        QString confirmedStepError;
        if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
                projectionModelName,
                currentModelHash,
                confirmedStepPath,
                confirmedStepSha256,
                confirmedStepError))
        {
            // 现有模型焊接模板的地面/V槽复核必须使用源 STEP；纯 PLY 不伪装成
            // 可保存模板，只走同页的预览确认与带双哈希导出。
            addToTemplate = false;
        }
    }
    else
    {
        projectionModelName = m_modelCombo != nullptr
            ? m_modelCombo->currentText().trimmed() : QString();
        if (projectionModelName.isEmpty()
            || !ReferenceModelLibrary::LoadModel(
                projectionModelName, projectionMesh, error))
        {
            QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"),
                error.isEmpty()
                    ? QStringLiteral("请先在模型库选择一个有效的逆向 PLY。") : error);
            return;
        }
        currentModelHash = ModelWeldingWorkflow::ComputeFileSha256(
            ReferenceModelLibrary::ModelPath(projectionModelName), &error);
        if (currentModelHash.isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"), error);
            return;
        }
    }
    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("选择模型坐标种子点列"),
        QString(),
        QStringLiteral("点列文本 (*.txt *.csv);;所有文件 (*)"));
    if (path.isEmpty()) return;

    QVector<RobotCalculation::IndexedPoint3D> indexedSeed;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(path, indexedSeed, &error))
    {
        QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"), error);
        return;
    }
    QVector<Eigen::Vector3d> seed;
    seed.reserve(indexedSeed.size());
    for (const RobotCalculation::IndexedPoint3D& point : indexedSeed)
        seed.push_back(point.point);
    const QString seedSha256 = ModelWeldingWorkflow::ComputeFileSha256(path, &error);
    if (seedSha256.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"), error);
        return;
    }
    const QString checkedModelHash = ModelWeldingWorkflow::ComputeFileSha256(
        ReferenceModelLibrary::ModelPath(projectionModelName), &error);
    if (checkedModelHash.isEmpty() || checkedModelHash != currentModelHash)
    {
        QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"),
            currentModelHash.isEmpty() ? error
                : QStringLiteral("计算 PLY 身份已变化，禁止使用缓存网格投影。"));
        return;
    }

    ReverseMeshSeamProjector::Options projectionOptions;
    double minimumSeedX = seed.front().x();
    double maximumSeedX = minimumSeedX;
    double minimumSeedY = seed.front().y();
    double maximumSeedY = minimumSeedY;
    for (const Eigen::Vector3d& point : seed)
    {
        minimumSeedX = std::min(minimumSeedX, point.x());
        maximumSeedX = std::max(maximumSeedX, point.x());
        minimumSeedY = std::min(minimumSeedY, point.y());
        maximumSeedY = std::max(maximumSeedY, point.y());
    }
    projectionOptions.sampleAxis = maximumSeedX - minimumSeedX
        >= maximumSeedY - minimumSeedY
        ? RobotCalculation::SampleAxis::AxisX
        : RobotCalculation::SampleAxis::AxisY;
    ReverseMeshSeamProjector::Result projected;
    bool success = false;
    NonClosableProgressDialog progress(
        QStringLiteral("正在把人工粗种子投影到逆向 PLY 表面…"),
        QString(), 0, 0, this);
    progress.setWindowTitle(QStringLiteral("逆向网格种子投影"));
    progress.setWindowModality(Qt::WindowModal);
    progress.setCancelButton(nullptr);
    progress.setMinimumDuration(0);
    progress.setAutoClose(false);
    progress.setAutoReset(false);
    progress.setWindowFlag(Qt::WindowCloseButtonHint, false);
    QEventLoop waitLoop;
    QThread* worker = QThread::create([&]()
        {
            try
            {
                success = ReverseMeshSeamProjector::Project(
                    projectionMesh, seed, projected, error, &projectionOptions);
            }
            catch (const std::exception& exception)
            {
                error = QStringLiteral("逆向网格投影线程发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
                success = false;
            }
            catch (...)
            {
                error = QStringLiteral("逆向网格投影线程发生未知异常。");
                success = false;
            }
        });
    connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
    connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
    progress.show();
    worker->start();
    waitLoop.exec();
    worker->wait();
    delete worker;
    progress.accept();
    if (!success)
    {
        QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"), error);
        return;
    }

    QDialog review(this);
    review.setWindowTitle(QStringLiteral("核对逆向网格投影焊缝"));
    review.resize(1100, 760);
    QVBoxLayout* reviewLayout = new QVBoxLayout(&review);
    QLabel* reviewHint = new QLabel(QStringLiteral(
        "灰色 = 逆向 PLY 顶点抽样；蓝色 = 输入粗种子；黄色 = 投影结果。"
        "请旋转、缩放核对整条焊缝。确认只写入模型坐标轨迹，不生成姿态、不启动机器人。"));
    reviewHint->setWordWrap(true);
    reviewLayout->addWidget(reviewHint);
    pcview::PointCloud3DView* reviewView = new pcview::PointCloud3DView(&review);
    QVector<pcview::PointCloud3DView::Layer> layers;
    pcview::PointCloud3DView::Layer meshLayer;
    meshLayer.name = QStringLiteral("逆向 PLY 顶点抽样");
    meshLayer.color = QColor(144, 164, 174);
    meshLayer.pointSize = 1.0;
    constexpr int kMaximumPreviewMeshPoints = 60000;
    const int previewStride = std::max(1,
        static_cast<int>(std::ceil(
            static_cast<double>(projectionMesh.vertices.size()) / kMaximumPreviewMeshPoints)));
    meshLayer.points.reserve(std::min(
        static_cast<int>(projectionMesh.vertices.size()), kMaximumPreviewMeshPoints));
    for (int index = 0; index < projectionMesh.vertices.size(); index += previewStride)
    {
        const Eigen::Vector3f& point = projectionMesh.vertices.at(index);
        meshLayer.points.push_back({ point.x(), point.y(), point.z() });
    }
    layers.push_back(std::move(meshLayer));
    pcview::PointCloud3DView::Layer seedLayer;
    seedLayer.name = QStringLiteral("输入粗种子");
    seedLayer.color = QColor(66, 165, 245);
    seedLayer.connectLines = true;
    seedLayer.pointSize = 2.6;
    for (const Eigen::Vector3d& point : seed)
        seedLayer.points.push_back({ point.x(), point.y(), point.z() });
    layers.push_back(std::move(seedLayer));
    pcview::PointCloud3DView::Layer projectedLayer;
    projectedLayer.name = QStringLiteral("投影焊缝");
    projectedLayer.color = QColor(255, 214, 64);
    projectedLayer.connectLines = true;
    projectedLayer.pointSize = 3.0;
    for (const Eigen::Vector3d& point : projected.pathModelMm)
        projectedLayer.points.push_back({ point.x(), point.y(), point.z() });
    layers.push_back(std::move(projectedLayer));
    reviewView->SetLayers(layers);
    reviewLayout->addWidget(reviewView, 1);
    QDialogButtonBox* reviewButtons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &review);
    reviewButtons->button(QDialogButtonBox::Ok)->setText(addToTemplate
        ? QStringLiteral("已核对，加入并确认")
        : QStringLiteral("已核对，导出点列"));
    reviewButtons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    connect(reviewButtons, &QDialogButtonBox::accepted, &review, &QDialog::accept);
    connect(reviewButtons, &QDialogButtonBox::rejected, &review, &QDialog::reject);
    reviewLayout->addWidget(reviewButtons);
    if (review.exec() != QDialog::Accepted) return;

    const QString verifiedModelHash = ModelWeldingWorkflow::ComputeFileSha256(
        ReferenceModelLibrary::ModelPath(projectionModelName), &error);
    const QString verifiedSeedHash = ModelWeldingWorkflow::ComputeFileSha256(path, &error);
    if (verifiedModelHash != currentModelHash || verifiedSeedHash != seedSha256)
    {
        QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"),
            QStringLiteral("投影期间模型或种子文件发生变化，结果已丢弃。"));
        return;
    }

    if (!addToTemplate)
    {
        const QString outputPath = QFileDialog::getSaveFileName(
            this,
            QStringLiteral("导出已确认的逆向网格焊缝点列"),
            projectionModelName + QStringLiteral("_ReverseMeshSeam.txt"),
            QStringLiteral("点列文本 (*.txt)"));
        if (outputPath.isEmpty()) return;
        QStringList lines;
        lines.reserve(projected.pathModelMm.size() + 4);
        lines.push_back(QStringLiteral("# source_model_sha256=%1").arg(currentModelHash));
        lines.push_back(QStringLiteral("# seed_path_sha256=%1").arg(seedSha256));
        lines.push_back(QStringLiteral("index x y z"));
        for (int index = 0; index < projected.pathModelMm.size(); ++index)
        {
            const Eigen::Vector3d& point = projected.pathModelMm.at(index);
            lines.push_back(QStringLiteral("%1 %2 %3 %4")
                .arg(index + 1)
                .arg(point.x(), 0, 'f', 6)
                .arg(point.y(), 0, 'f', 6)
                .arg(point.z(), 0, 'f', 6));
        }
        if (!RobotDataHelper::SaveTextFileLines(outputPath, lines, &error))
        {
            QMessageBox::warning(this, QStringLiteral("导出逆向网格焊缝"), error);
            return;
        }
        SetStatus(QStringLiteral(
            "已导出人工核对后的逆向网格焊缝：%1 点；文件包含模型和种子 SHA-256 证明。")
            .arg(projected.pathModelMm.size()));
        return;
    }

    const QByteArray identityInput = currentModelHash.toLatin1()
        + ':' + seedSha256.toLatin1();
    const QString seamId = QStringLiteral("mesh-%1").arg(
        QString::fromLatin1(QCryptographicHash::hash(
            identityInput, QCryptographicHash::Sha256).toHex().left(16)));
    const auto duplicate = std::find_if(
        m_template.seams.cbegin(), m_template.seams.cend(),
        [&seamId](const ModelWeldingSeamDefinition& seam)
        {
            return seam.seamId == seamId;
        });
    if (duplicate != m_template.seams.cend())
    {
        QMessageBox::information(this, QStringLiteral("逆向网格种子投影"),
            QStringLiteral("相同模型与种子文件的投影结果已在模板中。"));
        return;
    }
    ModelWeldingSeamDefinition seam;
    seam.seamId = seamId;
    seam.source = ModelWeldingSeamSource::ReverseMeshSeedProjection;
    seam.sourceGeometrySha256 = currentModelHash;
    seam.seedPathSha256 = seedSha256;
    seam.pathModelMm = std::move(projected.pathModelMm);
    seam.lengthMm = PolylineLength(seam.pathModelMm);
    seam.humanConfirmed = true;
    const int outputPointCount = seam.pathModelMm.size();
    ModelWeldingFlowTemplate candidateTemplate = m_template;
    candidateTemplate.seams.push_back(seam);
    if (!ModelWeldingWorkflow::ValidateTemplateStructure(candidateTemplate, error, false)
        || ModelWeldingWorkflow::EncodeTemplate(candidateTemplate, error).isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("逆向网格种子投影"), error);
        return;
    }
    m_template = std::move(candidateTemplate);
    m_templateDirty = true;
    RefreshSeamTable();
    SetStatus(QStringLiteral(
        "逆向网格种子投影已加入模板：网格 %1 点、种子 %2 点、输出 %3 点、"
        "回退 %4 点；已在点云叠加视图中完成人工确认。")
        .arg(projected.meshVertexCount)
        .arg(projected.seedPointCount)
        .arg(outputPointCount)
        .arg(projected.fallbackPointCount));
}

void ModelWeldingFlowDialog::RemoveSelectedSeam()
{
    const int row = m_seamTable != nullptr ? m_seamTable->currentRow() : -1;
    if (row < 0 || row >= m_template.seams.size())
    {
        QMessageBox::information(this, QStringLiteral("移除焊缝"),
            QStringLiteral("请先选择模板中的一条焊缝。"));
        return;
    }
    const QString seamId = m_template.seams.at(row).seamId;
    m_template.seams.removeAt(row);
    m_templateDirty = true;
    RefreshSeamTable();
    SetStatus(QStringLiteral("已从当前模板移除焊缝 %1，尚未保存。").arg(seamId));
}

void ModelWeldingFlowDialog::RefreshSeamTable()
{
    if (m_seamTable == nullptr) return;
    QString selectedId;
    const int oldRow = m_seamTable->currentRow();
    if (oldRow >= 0 && oldRow < m_template.seams.size())
        selectedId = m_template.seams.at(oldRow).seamId;
    m_loading = true;
    m_seamTable->setRowCount(m_template.seams.size());
    int selectedRow = -1;
    for (int row = 0; row < m_template.seams.size(); ++row)
    {
        const ModelWeldingSeamDefinition& seam = m_template.seams.at(row);
        const QStringList values = {
            seam.seamId,
            SeamSourceText(seam.source),
            QString::number(seam.lengthMm, 'f', 1),
            QString::number(seam.pathModelMm.size())
        };
        for (int column = 0; column < values.size(); ++column)
        {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            m_seamTable->setItem(row, column, item);
        }
        QTableWidgetItem* confirmed = new QTableWidgetItem();
        confirmed->setFlags((confirmed->flags() | Qt::ItemIsUserCheckable)
            & ~Qt::ItemIsEditable);
        confirmed->setCheckState(seam.humanConfirmed ? Qt::Checked : Qt::Unchecked);
        m_seamTable->setItem(row, 4, confirmed);
        if (seam.seamId == selectedId) selectedRow = row;
    }
    if (selectedRow < 0 && !m_template.seams.isEmpty()) selectedRow = 0;
    if (selectedRow >= 0)
    {
        m_seamTable->selectRow(selectedRow);
        m_seamTable->setCurrentCell(selectedRow, 0);
    }
    m_loading = false;
    RefreshPreview(true);
}

void ModelWeldingFlowDialog::RefreshStationTable()
{
    const QString selectedId = CurrentStationId();
    m_loading = true;
    m_stationTable->setRowCount(m_template.stations.size());
    int selectedRow = -1;
    for (int row = 0; row < m_template.stations.size(); ++row)
    {
        const ModelWeldingFeatureStation& station = m_template.stations.at(row);
        const ModelWeldingScanTeaching* scan = FindTeaching(station.stationId);
        const QStringList values = {
            station.stationId,
            StationRoleText(station.role),
            QString::number(station.anchorModelMm.x(), 'f', 2),
            QString::number(station.anchorModelMm.y(), 'f', 2),
            QString::number(station.anchorModelMm.z(), 'f', 2)
        };
        for (int column = 0; column < values.size(); ++column)
        {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            m_stationTable->setItem(row, column, item);
        }
        QTableWidgetItem* confirmed = new QTableWidgetItem();
        confirmed->setFlags((confirmed->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
        confirmed->setCheckState(station.candidateConfirmed ? Qt::Checked : Qt::Unchecked);
        m_stationTable->setItem(row, 5, confirmed);
        const bool taught = scan != nullptr && scan->startTaught && scan->endTaught
            && scan->startPulseTaught;
        QTableWidgetItem* taughtItem = new QTableWidgetItem(
            taught ? QStringLiteral("完整") : QStringLiteral("未完成"));
        taughtItem->setFlags(taughtItem->flags() & ~Qt::ItemIsEditable);
        taughtItem->setForeground(taught ? QColor(76, 175, 80) : QColor(239, 83, 80));
        m_stationTable->setItem(row, 6, taughtItem);
        if (station.stationId == selectedId)
        {
            selectedRow = row;
        }
    }
    if (selectedRow < 0 && !m_template.stations.isEmpty())
    {
        selectedRow = 0;
    }
    if (selectedRow >= 0)
    {
        m_stationTable->selectRow(selectedRow);
        m_stationTable->setCurrentCell(selectedRow, 0);
    }
    m_loading = false;
    RefreshStationDetails();
}

void ModelWeldingFlowDialog::RefreshStationDetails()
{
    const ModelWeldingScanTeaching* scan = FindTeaching(CurrentStationId());
    m_loading = true;
    if (scan == nullptr)
    {
        m_startPoseLabel->setText(QStringLiteral("未示教"));
        m_endPoseLabel->setText(QStringLiteral("未示教"));
        m_runSpeedSpin->setValue(300.0);
        m_scanSpeedSpin->setValue(100.0);
    }
    else
    {
        m_startPoseLabel->setText(PoseText(scan->startPose, scan->startTaught));
        m_endPoseLabel->setText(PoseText(scan->endPose, scan->endTaught));
        m_runSpeedSpin->setValue(scan->runSpeedMmPerMin);
        m_scanSpeedSpin->setValue(scan->scanSpeedMmPerMin);
    }
    m_loading = false;
}

bool ModelWeldingFlowDialog::HasActiveSimilarityInheritance() const
{
    return !m_template.inheritedFromTemplateId.isEmpty()
        && m_template.humanSameFixtureConfirmed
        && m_template.humanScanAreaConfirmed;
}

void ModelWeldingFlowDialog::RefreshVSlotPositionLabel()
{
    if (m_vSlotPositionLabel == nullptr) return;
    if (m_template.templateId.isEmpty())
    {
        m_vSlotPositionLabel->setText(QStringLiteral(
            "V槽模型坐标：新建或载入模板后，可在左侧三维视图中拖动定位。"));
        return;
    }
    const Eigen::Vector3d& anchor = m_template.placement.anchorModelMm;
    const bool inherited = HasActiveSimilarityInheritance();
    QString stateText;
    if (!m_groundFaceSatisfied)
        stateText = QStringLiteral("未完成大面吸附，拖动锁定");
    else if (m_vSlotWorkpieceSnapped)
        stateText = QStringLiteral("已吸附工件落地外轮廓，V槽显示绿色（上部外形仍需人工碰撞确认）");
    else if (inherited)
        stateText = QStringLiteral("相似模板已继承，可跳过首次V槽校准");
    else
        stateText = QStringLiteral("已贴地但未吸附工件，请拖近外角");
    m_vSlotPositionLabel->setText(QStringLiteral(
        "V槽模型坐标：X %1　Y %2　Z %3 mm；相对工件偏航 %4°（%5；调整后需保存模板）")
        .arg(anchor.x(), 0, 'f', 2)
        .arg(anchor.y(), 0, 'f', 2)
        .arg(anchor.z(), 0, 'f', 2)
        .arg(m_template.placement.vSlotYawDegrees, 0, 'f', 1)
        .arg(stateText));
}

void ModelWeldingFlowDialog::RefreshWorkpieceOrientationLabel()
{
    if (m_workpieceOrientationLabel == nullptr) return;
    if (m_template.templateId.isEmpty())
    {
        m_workpieceOrientationLabel->setText(QStringLiteral(
            "载入模板后，先选择一个大平面吸附到地面，再调整平面内朝向。"));
        return;
    }
    const Eigen::Matrix3d& axes = m_template.placement.axesModel;
    const auto vectorText = [](const Eigen::Vector3d& value)
        {
            return QStringLiteral("(%1, %2, %3)")
                .arg(value.x(), 0, 'f', 3)
                .arg(value.y(), 0, 'f', 3)
                .arg(value.z(), 0, 'f', 3);
        };
    m_workpieceOrientationLabel->setText(QStringLiteral(
        "%1；大面候选 %2 个。工件地面轴：+X %3　+Y %4　+Z %5；V槽相对偏航 %6°")
        .arg(m_groundFaceSatisfied
            ? QStringLiteral("已吸附大面")
            : QStringLiteral("未吸附合格大面，禁止确认/保存"))
        .arg(m_groundFaceNormalsModel.size())
        .arg(vectorText(axes.col(0)))
        .arg(vectorText(axes.col(1)))
        .arg(vectorText(axes.col(2)))
        .arg(m_template.placement.vSlotYawDegrees, 0, 'f', 1));
}

void ModelWeldingFlowDialog::RefreshGroundFaceCandidates()
{
    if (m_groundFaceCombo == nullptr || m_preview == nullptr) return;
    const QVector<cadview::GroundFaceCandidate> candidates =
        m_preview->GroundFaceCandidates();
    m_groundFaceCentersModel.clear();
    m_groundFaceNormalsModel.clear();
    m_groundFaceAreasMm2.clear();
    m_groundFaceLargestAreasMm2.clear();
    m_groundFaceTolerancesMm.clear();
    m_groundFaceSourceIndices.clear();
    const QSignalBlocker blocker(m_groundFaceCombo);
    m_groundFaceCombo->clear();
    int nearestIndex = -1;
    int nearestSourceIndex = -1;
    double nearestDot = -1.0;
    double nearestPlaneDistance = (std::numeric_limits<double>::max)();
    Eigen::Vector3d currentGroundUp = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d currentAnchor = Eigen::Vector3d::Zero();
    if (!m_template.templateId.isEmpty())
    {
        currentGroundUp = m_template.placement.axesModel.col(2);
        currentAnchor = m_template.placement.anchorModelMm;
    }
    for (int index = 0; index < candidates.size(); ++index)
    {
        const cadview::GroundFaceCandidate& candidate = candidates.at(index);
        const Eigen::Vector3d center(
            candidate.centerModel.x,
            candidate.centerModel.y,
            candidate.centerModel.z);
        Eigen::Vector3d normal(
            candidate.groundUpModel.x,
            candidate.groundUpModel.y,
            candidate.groundUpModel.z);
        const double largestArea = candidate.largestFaceAreaMm2 > 0.0
            ? candidate.largestFaceAreaMm2 : candidate.areaMm2;
        const double tolerance = std::clamp(
            candidate.supportToleranceMm, 0.05, 1.0);
        if (!center.allFinite() || !normal.allFinite() || normal.norm() <= 1.0e-9
            || !std::isfinite(candidate.areaMm2) || candidate.areaMm2 <= 0.0
            || !std::isfinite(largestArea) || largestArea <= 0.0
            || !std::isfinite(tolerance))
        {
            continue;
        }
        normal.normalize();
        m_groundFaceCentersModel.push_back(center);
        m_groundFaceNormalsModel.push_back(normal);
        m_groundFaceAreasMm2.push_back(candidate.areaMm2);
        m_groundFaceLargestAreasMm2.push_back(largestArea);
        m_groundFaceTolerancesMm.push_back(tolerance);
        m_groundFaceSourceIndices.push_back(index);
        const QString areaText = largestArea >= 100000.0
            ? QStringLiteral("%1 m²").arg(largestArea / 1000000.0, 0, 'f', 3)
            : QStringLiteral("%1 cm²").arg(largestArea / 100.0, 0, 'f', 1);
        m_groundFaceCombo->addItem(QStringLiteral("候选 %1　主大面 %2")
            .arg(m_groundFaceNormalsModel.size()).arg(areaText), index);
        const double alignment = currentGroundUp.dot(normal);
        const double planeDistance = std::abs((currentAnchor - center).dot(normal));
        if (alignment > nearestDot + 1.0e-12
            || (std::abs(alignment - nearestDot) <= 1.0e-12
                && planeDistance < nearestPlaneDistance))
        {
            nearestDot = alignment;
            nearestPlaneDistance = planeDistance;
            nearestIndex = m_groundFaceNormalsModel.size() - 1;
            nearestSourceIndex = index;
        }
    }
    const double nearestTolerance = nearestIndex >= 0
        ? m_groundFaceTolerancesMm.at(nearestIndex) : 0.0;
    m_groundFaceSatisfied = nearestIndex >= 0
        && nearestDot >= std::cos(0.1 * std::acos(-1.0) / 180.0)
        && nearestPlaneDistance <= nearestTolerance;
    if (!m_groundFaceSatisfied) m_vSlotWorkpieceSnapped = false;
    if (nearestIndex >= 0) m_groundFaceCombo->setCurrentIndex(nearestIndex);
    m_preview->SetGroundFaceCandidateHighlight(nearestSourceIndex);
    if (m_datumChecked != nullptr)
    {
        const bool snapRequired = !HasActiveSimilarityInheritance();
        m_datumChecked->setEnabled(m_groundFaceSatisfied && m_modelIdentityValid
            && (!snapRequired || m_vSlotWorkpieceSnapped));
    }
    if (!m_groundFaceSatisfied && m_modelIdentityValid && m_preview->HasCadShape()
        && !m_template.templateId.isEmpty())
    {
        bool hasDependentState = m_template.humanDatumConfirmed
            || m_template.humanCollisionChecked
            || m_template.humanSameFixtureConfirmed
            || m_template.humanScanAreaConfirmed;
        for (const ModelWeldingScanTeaching& scan : m_teaching.scans)
        {
            hasDependentState = hasDependentState || scan.startTaught
                || scan.endTaught || scan.startPulseTaught;
        }
        if (hasDependentState)
        {
            InvalidatePlacementDependentState(QStringLiteral(
                "保存的工件姿态未与任何外包络大平面重合"));
        }
    }
    RefreshWorkpieceOrientationLabel();
}

void ModelWeldingFlowDialog::ApplySelectedGroundFace()
{
    if (m_template.templateId.isEmpty())
    {
        SetStatus(QStringLiteral("请先新建或载入模型焊接模板。"), true);
        return;
    }
    if (!m_modelIdentityValid)
    {
        SetStatus(QStringLiteral("绑定模型身份无效，禁止修改工件落地姿态。"), true);
        return;
    }
    const int index = m_groundFaceCombo == nullptr ? -1 : m_groundFaceCombo->currentIndex();
    if (index < 0 || index >= m_groundFaceNormalsModel.size()
        || index >= m_groundFaceCentersModel.size())
    {
        SetStatus(QStringLiteral("原始 STEP 中没有可用的大平面落地候选。"), true);
        return;
    }
    const Eigen::Vector3d groundUp = m_groundFaceNormalsModel.at(index).normalized();
    const Eigen::Vector3d groundCenter = m_groundFaceCentersModel.at(index);
    Eigen::Vector3d groundX = m_template.placement.axesModel.col(0)
        - groundUp * groundUp.dot(m_template.placement.axesModel.col(0));
    if (groundX.norm() <= 1.0e-6)
    {
        groundX = m_template.placement.axesModel.col(1)
            - groundUp * groundUp.dot(m_template.placement.axesModel.col(1));
    }
    if (groundX.norm() <= 1.0e-6)
    {
        const Eigen::Vector3d fallback = std::abs(groundUp.x()) < 0.8
            ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        groundX = fallback - groundUp * groundUp.dot(fallback);
    }
    groundX.normalize();
    const Eigen::Vector3d groundY = groundUp.cross(groundX).normalized();
    Eigen::Matrix3d axes;
    axes.col(0) = groundX;
    axes.col(1) = groundY;
    axes.col(2) = groundUp;
    m_template.placement.axesModel = axes;
    // anchor 定义 V 槽地面平面。只改方向会让大面平行但仍悬空/穿地；
    // 因此必须同步把地面平面沿法向投影到所选候选面。
    Eigen::Vector3d anchor = m_template.placement.anchorModelMm;
    anchor += groundUp * ((groundCenter - anchor).dot(groundUp));
    m_template.placement.anchorModelMm = anchor;
    m_groundFaceSatisfied = false;
    m_vSlotWorkpieceSnapped = false;
    InvalidatePlacementDependentState(QStringLiteral(
        "工件已吸附到落地大面候选 %1").arg(index + 1));
    RefreshPreview(true);
}

void ModelWeldingFlowDialog::RotateWorkpieceAroundGroundZ(double degrees)
{
    if (m_template.templateId.isEmpty() || !m_modelIdentityValid)
    {
        SetStatus(QStringLiteral("请先载入身份有效的模型模板。"), true);
        return;
    }
    if (!m_groundFaceSatisfied)
    {
        SetStatus(QStringLiteral("请先选择一个大平面并吸附到地面。"), true);
        return;
    }
    if (!std::isfinite(degrees) || std::abs(degrees) <= 1.0e-9)
    {
        SetStatus(QStringLiteral("地面内旋转角度无效。"), true);
        return;
    }
    const double radians = degrees * std::acos(-1.0) / 180.0;
    const Eigen::Matrix3d displayRotation =
        Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d previousAxes = m_template.placement.axesModel;
    Eigen::Matrix3d nextAxes =
        previousAxes * displayRotation.transpose();
    Eigen::Quaterniond normalized(nextAxes);
    normalized.normalize();
    nextAxes = normalized.toRotationMatrix();
    if (!nextAxes.allFinite() || nextAxes.determinant() < 0.999999)
    {
        SetStatus(QStringLiteral("工件姿态旋转结果无效，已拒绝修改。"), true);
        return;
    }
    m_template.placement.axesModel = nextAxes;
    // anchor 是模型上的工装基准点，旋转工件不应为稳定视图而篡改它。
    // CadModel3DView 会在显示变换中增加枢轴平移，使该点的屏幕位置保持不变。
    m_vSlotWorkpieceSnapped = false;
    InvalidatePlacementDependentState(
        QStringLiteral("工件已在落地大面内旋转 %1°").arg(degrees, 0, 'f', 1));
    RefreshWorkpieceOrientationLabel();
    RefreshPreview(true);
}

void ModelWeldingFlowDialog::RotateVSlotAroundGroundZ(double degrees)
{
    if (m_template.templateId.isEmpty() || !m_modelIdentityValid)
    {
        SetStatus(QStringLiteral("请先载入身份有效的模型模板。"), true);
        return;
    }
    if (!m_groundFaceSatisfied)
    {
        SetStatus(QStringLiteral("请先选择一个大平面并吸附到地面。"), true);
        return;
    }
    if (HasActiveSimilarityInheritance())
    {
        SetStatus(QStringLiteral(
            "当前相似模板的工装位姿处于审核继承锁定状态；如需改动，请先改变工件姿态转为普通模板。"),
            true);
        return;
    }
    if (!std::isfinite(degrees) || std::abs(degrees) <= 1.0e-9)
    {
        SetStatus(QStringLiteral("V槽旋转角度无效。"), true);
        return;
    }
    m_template.placement.vSlotYawDegrees = NormalizeSignedDegrees(
        m_template.placement.vSlotYawDegrees + degrees);
    m_vSlotWorkpieceSnapped = false;
    InvalidatePlacementDependentState(
        QStringLiteral("V槽已绕地面Z独立旋转 %1°，请拖近目标外角重新吸附")
            .arg(degrees, 0, 'f', 1));
    RefreshWorkpieceOrientationLabel();
    RefreshPreview(true);
}

void ModelWeldingFlowDialog::InvalidatePlacementDependentState(const QString& reason)
{
    if (m_template.templateId.isEmpty()) return;
    const bool convertedFromInheritance =
        !m_template.inheritedFromTemplateId.isEmpty();
    if (convertedFromInheritance)
    {
        // 相似模板的跳过校准结论仅对创建时审核的工装位置有效。
        // 一旦人工改变落地面、旋转、槽尺寸或 anchor，就转为普通模板，
        // 否则已失效的继承记录无法通过现有界面重新审核。
        m_template.inheritedFromTemplateId.clear();
        m_template.inheritedFromRevision = 0;
        m_template.inheritedFromRecordSha256.clear();
        if (m_inheritanceLabel != nullptr)
        {
            m_inheritanceLabel->setText(QStringLiteral(
                "已修改继承工装位置，当前模板已转为普通模板；需重新完成V槽吸附和人工确认。"));
        }
    }
    m_template.humanDatumConfirmed = false;
    m_template.humanCollisionChecked = false;
    m_template.humanSameFixtureConfirmed = false;
    m_template.humanScanAreaConfirmed = false;
    if (m_datumChecked != nullptr)
    {
        const QSignalBlocker blocker(m_datumChecked);
        m_datumChecked->setChecked(false);
    }
    if (m_collisionChecked != nullptr)
    {
        const QSignalBlocker blocker(m_collisionChecked);
        m_collisionChecked->setChecked(false);
    }

    bool teachingInvalidated = false;
    for (ModelWeldingScanTeaching& scan : m_teaching.scans)
    {
        if (scan.startTaught || scan.endTaught || scan.startPulseTaught)
        {
            scan.startTaught = false;
            scan.endTaught = false;
            scan.startPulseTaught = false;
            teachingInvalidated = true;
        }
    }
    m_templateDirty = true;
    if (teachingInvalidated) m_teachingDirty = true;
    RefreshStationTable();
    RefreshVSlotPositionLabel();
    RefreshWorkpieceOrientationLabel();
    SetStatus(QStringLiteral(
        "%1%2；原基准、碰撞确认及已有扫描起终点示教已失效。请按新位置重新确认并示教。")
        .arg(reason,
            convertedFromInheritance
                ? QStringLiteral("；已转为普通模板") : QString()), true);
}

void ModelWeldingFlowDialog::RefreshPreview(bool preserveView)
{
    RefreshVSlotPositionLabel();
    RefreshWorkpieceOrientationLabel();
    const auto clearPreviewSourceIdentity = [this]()
    {
        m_previewModelName.clear();
        m_previewPlySha256.clear();
        m_previewSourcePath.clear();
        m_previewSourceSha256.clear();
    };
    cadview::VSlotFixture fixture;
    cadview::GroundSurface ground;
    QVector<cadview::ScanRegion> scanRegions;
    QVector<cadview::GuideSegment> guides;
    QVector<cadview::StationMarker> markers;
    if (!m_template.templateId.isEmpty())
    {
        const Eigen::Matrix3d& workpieceAxes = m_template.placement.axesModel;
        const double fixtureYawRadians =
            m_template.placement.vSlotYawDegrees * std::acos(-1.0) / 180.0;
        const Eigen::Matrix3d fixtureYaw = Eigen::AngleAxisd(
            fixtureYawRadians, workpieceAxes.col(2).normalized()).toRotationMatrix();
        fixture.visible = true;
        fixture.draggable = !HasActiveSimilarityInheritance();
        fixture.anchor = ViewPoint(m_template.placement.anchorModelMm);
        fixture.axisX = ViewPoint(fixtureYaw * workpieceAxes.col(0));
        fixture.axisY = ViewPoint(fixtureYaw * workpieceAxes.col(1));
        fixture.axisZ = ViewPoint(workpieceAxes.col(2));
        fixture.longLengthMm = m_template.placement.longLengthMm;
        fixture.shortLengthMm = m_template.placement.shortLengthMm;
        const double referenceLength = std::max(10.0,
            std::min(fixture.longLengthMm, fixture.shortLengthMm));
        fixture.wallThicknessMm = std::clamp(referenceLength * 0.075, 10.0, 28.0);
        fixture.wallHeightMm = std::clamp(referenceLength * 0.20, 28.0, 72.0);
        ground.axisX = ViewPoint(workpieceAxes.col(0));
        ground.axisY = ViewPoint(workpieceAxes.col(1));
        ground.axisZ = ViewPoint(workpieceAxes.col(2));
        ground.workpieceRotatable = m_groundFaceSatisfied && m_modelIdentityValid;
    }

    const QString selectedId = CurrentStationId();
    for (const ModelWeldingFeatureStation& station : m_template.stations)
    {
        const bool selected = station.stationId == selectedId;
        const QColor color = StationColor(station, selected);
        cadview::ScanRegion region;
        region.center = ViewPoint(station.anchorModelMm);
        region.outwardNormal = ViewPoint(station.scanDirectionModel);
        region.label = station.stationId;
        region.color = color;
        region.radiusMm = std::clamp(
            station.roiHalfExtentMm.maxCoeff(), 10.0, 250.0);
        region.thicknessMm = selected ? 3.2 : 2.0;
        region.selected = selected;
        scanRegions.push_back(region);

        cadview::StationMarker marker;
        marker.position = ViewPoint(station.anchorModelMm);
        marker.label = QStringLiteral("%1 %2")
            .arg(station.stationId, StationRoleText(station.role));
        marker.color = color;
        marker.size = station.stationId == selectedId ? 10.0 : 7.0;
        markers.push_back(marker);

        Eigen::Vector3d direction = station.scanDirectionModel;
        if (direction.norm() > 1.0e-9)
        {
            direction.normalize();
            cadview::GuideSegment arrow;
            arrow.start = ViewPoint(station.anchorModelMm);
            arrow.end = ViewPoint(station.anchorModelMm + direction * 30.0);
            arrow.color = color;
            arrow.width = station.stationId == selectedId ? 3.0 : 2.0;
            arrow.arrowHead = true;
            guides.push_back(arrow);
        }
    }

    if (m_template.templateId.isEmpty())
    {
        m_preview->ClearSceneOverlays();
        m_preview->ClearModel(QStringLiteral("请选择或新建模型焊接模板。"));
        RefreshGroundFaceCandidates();
        m_previewInitialized = false;
        clearPreviewSourceIdentity();
        RefreshTheoreticalRobotStatus();
        return;
    }
    if (!m_modelIdentityValid)
    {
        m_preview->ClearSceneOverlays();
        m_preview->ClearModel(QStringLiteral(
            "绑定的计算模型缺失或身份校验失败，已禁止显示可能不匹配的原始 STEP。"));
        RefreshGroundFaceCandidates();
        m_previewInitialized = false;
        clearPreviewSourceIdentity();
        RefreshTheoreticalRobotStatus();
        return;
    }

    QString sourcePath;
    QString expectedSourceSha256;
    QString sourceError;
    const QString modelName = m_template.modelLibraryName;
    const QString plySha256 = m_template.modelSha256.trimmed().toLower();
    const bool useResolvedIdentity = preserveView && m_previewInitialized
        && m_previewModelName == modelName
        && m_previewPlySha256 == plySha256
        && !m_previewSourcePath.isEmpty()
        && m_previewSourceSha256.size() == 64;
    if (useResolvedIdentity)
    {
        sourcePath = m_previewSourcePath;
        expectedSourceSha256 = m_previewSourceSha256;
    }
    else if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
                 modelName,
                 plySha256,
                 sourcePath,
                 expectedSourceSha256,
                 sourceError))
    {
        m_preview->ClearSceneOverlays();
        m_preview->ClearModel(QStringLiteral("原始 STEP 外显源校验失败：%1").arg(sourceError));
        RefreshGroundFaceCandidates();
        m_previewInitialized = false;
        clearPreviewSourceIdentity();
        RefreshTheoreticalRobotStatus();
        return;
    }

    const int selectedSeamRow = m_seamTable != nullptr ? m_seamTable->currentRow() : -1;
    for (int seamIndex = 0; seamIndex < m_template.seams.size(); ++seamIndex)
    {
        const ModelWeldingSeamDefinition& seam = m_template.seams.at(seamIndex);
        const bool sourceMatches = seam.source
            == ModelWeldingSeamSource::ReverseMeshSeedProjection
            ? seam.sourceGeometrySha256 == m_template.modelSha256
            : seam.sourceGeometrySha256 == expectedSourceSha256;
        if (!sourceMatches) continue;
        const bool selected = seamIndex == selectedSeamRow;
        const QColor color = selected
            ? QColor(255, 214, 64)
            : (seam.humanConfirmed ? QColor(102, 187, 106) : QColor(239, 83, 80));
        AppendPolylineGuides(
            seam.pathModelMm, color, selected ? 4.2 : 3.0, guides);
    }
    const int candidateComboIndex = m_seamCandidateCombo != nullptr
        ? m_seamCandidateCombo->currentIndex() : -1;
    const int selectedCandidateIndex = candidateComboIndex >= 0
        ? m_seamCandidateCombo->itemData(candidateComboIndex).toInt() : -1;
    if (selectedCandidateIndex >= 0
        && selectedCandidateIndex < m_seamCandidates.size()
        && m_seamCandidateSourcePath == sourcePath
        && m_seamCandidateSourceSha256 == expectedSourceSha256)
    {
        AppendPolylineGuides(
            m_seamCandidates.at(selectedCandidateIndex).pointsModelMm,
            QColor(38, 198, 218), 3.4, guides);
    }

    QString displayError;
    const bool loaded = m_preview->SetStepFile(
        sourcePath,
        expectedSourceSha256,
        displayError,
        preserveView && m_previewInitialized);
    if (loaded)
    {
        RefreshGroundFaceCandidates();
        RefreshVSlotPositionLabel();
        ground.workpieceRotatable = m_groundFaceSatisfied && m_modelIdentityValid;
        const int groundIndex = m_groundFaceCombo != nullptr
            ? m_groundFaceCombo->currentIndex() : -1;
        if (m_groundFaceSatisfied
            && groundIndex >= 0
            && groundIndex < m_groundFaceCentersModel.size()
            && groundIndex < m_groundFaceSourceIndices.size())
        {
            ground.visible = true;
            ground.planePoint = ViewPoint(m_groundFaceCentersModel.at(groundIndex));
            ground.supportCandidateIndex = m_groundFaceSourceIndices.at(groundIndex);
            ground.thicknessMm = 12.0;
            ground.snapDistanceMm = std::clamp(
                std::min(fixture.longLengthMm, fixture.shortLengthMm) * 0.25,
                30.0,
                120.0);
        }
        m_preview->SetSceneOverlays(ground, fixture, scanRegions, guides, markers);
        m_vSlotWorkpieceSnapped = m_preview->IsVSlotSnappedToWorkpiece();
        if (m_datumChecked != nullptr)
        {
            const bool snapRequired = !HasActiveSimilarityInheritance();
            m_datumChecked->setEnabled(m_groundFaceSatisfied && m_modelIdentityValid
                && (!snapRequired || m_vSlotWorkpieceSnapped));
        }
        RefreshVSlotPositionLabel();
    }
    else
    {
        m_preview->ClearSceneOverlays();
        m_preview->ClearModel(QStringLiteral("原始 STEP 外显失败：%1").arg(displayError));
        RefreshGroundFaceCandidates();
    }
    m_previewInitialized = loaded;
    if (loaded)
    {
        if (!m_seamCandidateSourceSha256.isEmpty()
            && (m_seamCandidateSourcePath != sourcePath
                || m_seamCandidateSourceSha256 != expectedSourceSha256))
        {
            ClearSeamCandidates();
        }
        m_previewModelName = modelName;
        m_previewPlySha256 = plySha256;
        m_previewSourcePath = sourcePath;
        m_previewSourceSha256 = expectedSourceSha256;
    }
    else
    {
        clearPreviewSourceIdentity();
    }
    RefreshTheoreticalRobotStatus();
}

void ModelWeldingFlowDialog::RefreshIdentityStatus()
{
    const QString endpoint = m_teaching.robotEndpoint.trimmed();
    const bool handEyeBound = m_teaching.handEyeSha256.size() == 64;
    const bool toolBound = m_teaching.tool1Sha256.size() == 64;
    m_identityLabel->setText(QStringLiteral(
        "运行绑定：机器人 %1；端点 %2；相机 %3；手眼 %4；Tool1 %5")
        .arg(CurrentRobotName().isEmpty() ? QStringLiteral("未选择") : CurrentRobotName())
        .arg(endpoint.isEmpty() ? QStringLiteral("未绑定") : endpoint)
        .arg(CurrentCameraSection().isEmpty() ? QStringLiteral("未选择") : CurrentCameraSection())
        .arg(handEyeBound ? m_teaching.handEyeSha256.left(12) + QStringLiteral("…") : QStringLiteral("未绑定"))
        .arg(toolBound ? m_teaching.tool1Sha256.left(12) + QStringLiteral("…") : QStringLiteral("未绑定")));
}

void ModelWeldingFlowDialog::SetStatus(const QString& text, bool error)
{
    m_statusLabel->setText(text);
    m_statusLabel->setStyleSheet(error
        ? QStringLiteral("QLabel { color:#ff8a80; padding:5px; }")
        : QStringLiteral("QLabel { color:#a5d6a7; padding:5px; }"));
}

void ModelWeldingFlowDialog::OpenOfflineRigidFit()
{
    if (!m_modelIdentityValid)
    {
        QMessageBox::warning(this, QStringLiteral("离线验证定位矩阵"),
            QStringLiteral("绑定模型缺失或SHA-256不匹配，禁止在错误几何上验证。"));
        return;
    }
    QVector<ModelWeldingFeatureStation> stations;
    int solveCount = 0;
    int verifyCount = 0;
    for (const ModelWeldingFeatureStation& station : m_template.stations)
    {
        if (station.candidateConfirmed && station.role != ModelWeldingStationRole::Backup)
        {
            stations.push_back(station);
            solveCount += station.role == ModelWeldingStationRole::Solve ? 1 : 0;
            verifyCount += station.role == ModelWeldingStationRole::Verify ? 1 : 0;
        }
    }
    if (solveCount < 3 || verifyCount < 1)
    {
        QMessageBox::information(this, QStringLiteral("离线验证定位矩阵"),
            QStringLiteral("请先确认至少3个求解站和1个验证站。"));
        return;
    }

    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("刚性定位矩阵离线验证"));
    dialog.resize(1050, 620);
    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QLabel* hint = new QLabel(QStringLiteral(
        "此处只验证模型点→机器人Base点的纯刚体SE(3)算法，不控制机器人。"
        "后续自动流程将由点云特征提取器填入实测Base坐标。"));
    hint->setWordWrap(true);
    layout->addWidget(hint);
    QTableWidget* table = new QTableWidget(stations.size(), 8);
    table->setHorizontalHeaderLabels({
        QStringLiteral("编号"), QStringLiteral("角色"), QStringLiteral("模型X (mm)"),
        QStringLiteral("模型Y (mm)"), QStringLiteral("模型Z (mm)"),
        QStringLiteral("实测Base X (mm)"), QStringLiteral("实测Base Y (mm)"),
        QStringLiteral("实测Base Z (mm)")
    });
    for (int row = 0; row < stations.size(); ++row)
    {
        const ModelWeldingFeatureStation& station = stations.at(row);
        const QStringList fixed = {
            station.stationId,
            StationRoleText(station.role),
            QString::number(station.anchorModelMm.x(), 'f', 4),
            QString::number(station.anchorModelMm.y(), 'f', 4),
            QString::number(station.anchorModelMm.z(), 'f', 4)
        };
        for (int column = 0; column < fixed.size(); ++column)
        {
            QTableWidgetItem* item = new QTableWidgetItem(fixed.at(column));
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            table->setItem(row, column, item);
        }
        for (int column = 5; column < 8; ++column)
        {
            QLineEdit* edit = new QLineEdit();
            edit->setPlaceholderText(QStringLiteral("输入"));
            table->setCellWidget(row, column, edit);
        }
    }
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    layout->addWidget(table, 1);
    QPushButton* calculate = new QPushButton(QStringLiteral("计算并执行质量门禁"));
    QTextEdit* output = new QTextEdit();
    output->setReadOnly(true);
    output->setMinimumHeight(180);
    layout->addWidget(calculate);
    layout->addWidget(output);
    QDialogButtonBox* closeButtons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(closeButtons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(closeButtons);

    connect(calculate, &QPushButton::clicked, &dialog, [table, output, stations]()
        {
            QVector<ModelWeldingRigidPointPair> pairs;
            for (int row = 0; row < stations.size(); ++row)
            {
                Eigen::Vector3d measured;
                for (int axis = 0; axis < 3; ++axis)
                {
                    QLineEdit* edit = qobject_cast<QLineEdit*>(table->cellWidget(row, 5 + axis));
                    bool ok = false;
                    const double value = edit != nullptr
                        ? edit->text().trimmed().toDouble(&ok) : 0.0;
                    if (!ok || !std::isfinite(value))
                    {
                        output->setPlainText(QStringLiteral("第 %1 行实测Base坐标无效。").arg(row + 1));
                        return;
                    }
                    measured[axis] = value;
                }
                ModelWeldingRigidPointPair pair;
                pair.featureId = stations.at(row).stationId;
                pair.modelPointMm = stations.at(row).anchorModelMm;
                pair.measuredBasePointMm = measured;
                pair.useForSolve = stations.at(row).role == ModelWeldingStationRole::Solve;
                pairs.push_back(pair);
            }
            const ModelWeldingRigidFitResult fit =
                ModelWeldingWorkflow::SolveRigidPointPairs(pairs);
            QString text = QStringLiteral("求解：%1\n质量门禁：%2\nRMSE：%3 mm\n"
                                          "求解最大残差：%4 mm\n验证最大残差：%5 mm\n"
                                          "点对最大距离差：%6 mm\n检测比例：%7\n")
                .arg(fit.solved ? QStringLiteral("成功") : QStringLiteral("失败"))
                .arg(fit.accepted ? QStringLiteral("通过") : QStringLiteral("拒绝"))
                .arg(fit.solveRmseMm, 0, 'f', 4)
                .arg(fit.maximumSolveResidualMm, 0, 'f', 4)
                .arg(fit.maximumVerifyResidualMm, 0, 'f', 4)
                .arg(fit.maximumPairDistanceErrorMm, 0, 'f', 4)
                .arg(fit.observedScaleRatio, 0, 'f', 8);
            if (!fit.rejectionReason.isEmpty())
            {
                text += QStringLiteral("拒绝原因：%1\n").arg(fit.rejectionReason);
            }
            if (fit.solved)
            {
                text += QStringLiteral("\nT_Base_Model：\n");
                for (int row = 0; row < 4; ++row)
                {
                    text += QStringLiteral("[%1  %2  %3  %4]\n")
                        .arg(fit.baseFromModel(row, 0), 0, 'f', 9)
                        .arg(fit.baseFromModel(row, 1), 0, 'f', 9)
                        .arg(fit.baseFromModel(row, 2), 0, 'f', 9)
                        .arg(fit.baseFromModel(row, 3), 0, 'f', 4);
                }
            }
            output->setPlainText(text);
        });
    dialog.exec();
}
