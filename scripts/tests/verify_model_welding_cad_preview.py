from pathlib import Path


repo = Path(__file__).resolve().parents[2]
dialog = (repo / "src" / "ModelWeldingFlowDialog.cpp").read_text(encoding="utf-8")
dialog_header = (repo / "include" / "ModelWeldingFlowDialog.h").read_text(encoding="utf-8")
project = (repo / "QtWidgetsApplication4.vcxproj").read_text(encoding="utf-8")
cad = (repo / "src" / "CadModel3DView.cpp").read_text(encoding="utf-8")
cad_header = (repo / "include" / "CadModel3DView.h").read_text(encoding="utf-8")
cad_smoke = (repo / "scripts" / "tests" / "cad_model_3d_view_smoke_tests.cpp").read_text(
    encoding="utf-8"
)


def function_body(source: str, signature: str, next_signature: str) -> str:
    start_index = source.index(signature)
    end_index = source.index(next_signature, start_index)
    return source[start_index:end_index]

preview = function_body(
    dialog,
    "void ModelWeldingFlowDialog::RefreshPreview(bool preserveView)",
    "void ModelWeldingFlowDialog::RefreshIdentityStatus()",
)

required_preview_tokens = (
    "ResolveConfirmedStepDisplaySource",
    "SetStepFile(",
    "SetSceneOverlays(",
    "VSlotFixture",
    "ScanRegion",
    "roiHalfExtentMm",
    "expectedSourceSha256",
)
for token in required_preview_tokens:
    assert token in preview, f"missing CAD preview contract token: {token}"

for forbidden in ("PointCloud3DView", "m_mesh.vertices", "SetLayers(", "SetDirectionArrows("):
    assert forbidden not in preview, f"STEP preview regressed to point rendering: {forbidden}"

for dependency in ("TKPrim.lib", "TKService.lib", "TKV3d.lib", "TKOpenGl.lib", "TKOpenGl.dll"):
    assert dependency in project, f"missing OCCT viewer dependency: {dependency}"

assert "Quantity_Color(1.0, 1.0, 1.0, Quantity_TOC_RGB)" in cad
assert "StableFileSha256" in cad and "loadedSourceSha256" in cad
assert "AIS_Shaded" in cad and "AIS_Shape" in cad
assert "BuildVSlotShape" in cad and "BRepPrimAPI_MakePrism" in cad
assert "BuildScanRegionShape" in cad and "BRepPrimAPI_MakeCylinder" in cad
assert "DetectedInteractive() == vSlotObject" in cad
assert "ConvertWithProj" in cad and "SetLocation(vSlotObject" in cad
assert "ClosestWorkpieceSnapCorner" in cad and "AnchorMatchesWorkpieceSnapCorner" in cad
assert "SetVSlotSnapStateCallback" in cad and "IsVSlotSnappedToWorkpiece" in cad
assert "vSlotSnappedToWorkpiece" in cad and "Quantity_Color(0.16, 0.68, 0.34" in cad
for token in (
    "minimumU",
    "minimumV",
    "contactsLongXWall",
    "contactsShortYWall",
    "minimumIndependentContact",
    "overlapEnd - overlapStart > contactTolerance",
    "displayedVSlotFixture.longLengthMm",
    "displayedVSlotFixture.shortLengthMm",
):
    assert token in cad, f"missing two-wall workpiece snap proof: {token}"
assert "BRepAdaptor_Curve" in cad and "record.snapHull = ExtractGroundSnapPoints" in cad
assert "record.snapSegments = ExtractGroundSnapSegments" in cad
assert "m_vSlotWorkpieceSnapped" in dialog
assert "HasActiveSimilarityInheritance" in dialog
assert "fixture.draggable = !HasActiveSimilarityInheritance()" in dialog
assert "convertedInactiveInheritance" in dialog
assert "m_template.inheritedFromTemplateId.clear()" in dialog

# Rotation must be available where the operator is looking: controls remain hidden until
# a verified ground exists, emit signed configurable steps, and respect the inherited
# fixture's draggable lock. V-slot snapping is a complete pose operation and may capture a
# nearby workpiece-edge direction, not just translate an incorrectly oriented fixture.
for token in (
    "bool workpieceRotatable = false",
    "SetVSlotPoseChangedCallback",
    "SetWorkpieceRotationRequestedCallback",
    "SetVSlotRotationRequestedCallback",
    "SetRotationStepDegrees",
):
    assert token in cad_header, f"missing in-view rotation contract token: {token}"

for token in (
    'QStringLiteral("workpieceRotateMinusButton")',
    'QStringLiteral("workpieceRotatePlusButton")',
    'QStringLiteral("vSlotRotateMinusButton")',
    'QStringLiteral("vSlotRotatePlusButton")',
    "button->setAutoRepeat(false)",
    "workpieceRotationRequestedCallback(-rotationStepDegrees)",
    "workpieceRotationRequestedCallback(rotationStepDegrees)",
    "vSlotRotationRequestedCallback(-rotationStepDegrees)",
    "vSlotRotationRequestedCallback(rotationStepDegrees)",
):
    assert token in cad, f"missing in-view rotation implementation token: {token}"

rotation_controls = function_body(
    cad,
    "void RefreshRotationControls()",
    "void ResizeStatus()",
)
for token in (
    "displayedGroundSurface.visible",
    "displayedGroundSurface.workpieceRotatable",
    "VSlotDragEnabled()",
    "setEnabled(workpieceReady)",
    "setEnabled(vSlotReady)",
    "rotationPanel->setVisible(workpieceReady || vSlotReady)",
):
    assert token in rotation_controls, f"missing rotation-control safety gate: {token}"

display_frame = function_body(
    cad,
    "Vec3d ModelDirectionToDisplay(const Vec3d& value) const",
    "bool ShapeBoundsAtLocation(",
)
for token in (
    "ModelDirectionToDisplay",
    "DisplayDirectionToModel",
    "displayFrameTranslation",
    "return Add(ModelDirectionToDisplay(value), displayFrameTranslation)",
    "DisplayDirectionToModel(Subtract(value, displayFrameTranslation))",
    "const Vec3d fixedDisplayPivot = ModelToDisplay(pivotModel)",
    "fixedDisplayPivot, ModelDirectionToDisplay(pivotModel)",
):
    assert token in display_frame, f"missing fixed-pivot display-frame token: {token}"
for token in (
    "display.anchor = ModelToDisplay(vSlotFixture.anchor)",
    "display.axisX = ModelDirectionToDisplay(vSlotFixture.axisX)",
    "display.axisY = ModelDirectionToDisplay(vSlotFixture.axisY)",
    "display.axisZ = ModelDirectionToDisplay(vSlotFixture.axisZ)",
):
    assert token in cad, f"fixture point/direction transform was conflated: {token}"

snap_contact = function_body(
    cad,
    "bool EvaluateWorkpieceSnapCorner(",
    "bool ClosestWorkpieceSnapCorner(",
)
for token in (
    "const QVector<GroundSnapSegment> realSegments",
    "ExtractGroundSnapSegments(",
    "for (const GroundSnapSegment& segment : realSegments)",
    "segment.startModel",
    "segment.endModel",
):
    assert token in snap_contact, f"missing real B-Rep contact segment proof: {token}"
assert "(index + 1) % fixturePoints.size()" not in snap_contact, (
    "convex-hull neighbors were reused as fictitious material segments"
)

snap_pose = function_body(
    cad,
    "bool ClosestWorkpieceSnapPose(",
    "bool AnchorMatchesWorkpieceSnapCorner(",
)
for token in (
    "kOrientationCaptureDegrees = 12.0",
    "ExtractGroundSnapPoints",
    "angleDegrees <= kOrientationCaptureDegrees",
    "best.axisX = candidateX",
    "best.axisY = candidateY",
    "best.axisZ = normal",
):
    assert token in snap_pose, f"missing V-slot pose-capture token: {token}"

for token in (
    "RotateInPlane(",
    "workpieceRotateMinusButton",
    "rotation controls were visible before verified ground",
    "in-view rotation controls emitted the wrong direction or step",
    "locked V-slot retained active in-view rotation controls",
    "V-slot within 12 degrees was not aligned to the workpiece edge",
    "orthonormal right-handed frame",
    "reapplying the committed V-slot pose lost the green snap state",
    "workpiece rotation moved the V-slot anchor screen pivot",
    "workpiece rotation carried the independent V-slot around the workpiece",
    "workpiece B-Rep did not visibly rotate around the fixed V-slot pivot",
):
    assert token in cad_smoke, f"missing rotation/snap smoke coverage: {token}"

# A legal floor pose must come from a sufficiently large planar B-Rep face that is also
# an extrema support plane of the complete STEP shape. Internal planes and an aggregate of
# many small feet must never become a floor candidate.
ground_analysis = function_body(
    cad,
    "std::vector<GroundFaceRecord> AnalyzeGroundFaceCandidates(const TopoDS_Shape& shape)",
    "struct OverlayLabel",
)
for token in (
    "surface.GetType() != GeomAbs_Plane",
    "BRepBndLib::AddOptimal",
    "largestFaceAreaMm2",
    "diagonal * diagonal * 1.0e-3",
    "records.front().summary.largestFaceAreaMm2 * 0.05",
    "std::abs(planeZ - projectedZMin) <= supportTolerance",
    "std::abs(planeZ - projectedZMax) <= supportTolerance",
    "record.snapHull = ExtractGroundSnapPoints(record)",
    "record.snapSegments = ExtractGroundSnapSegments(record)",
):
    assert token in ground_analysis, f"missing outer-envelope ground-face gate: {token}"

for token in (
    "struct GroundFaceCandidate",
    "largestFaceAreaMm2",
    "GroundFaceCandidates() const",
    "SetGroundFaceCandidateHighlight(int candidateIndex)",
):
    assert token in cad_header, f"missing ground-face CAD contract token: {token}"

# Snapping is a coupled orientation/translation operation: it aligns +Z and projects the
# V-slot anchor onto the selected support plane before any dependent confirmation survives.
ground_snap = function_body(
    dialog,
    "void ModelWeldingFlowDialog::ApplySelectedGroundFace()",
    "void ModelWeldingFlowDialog::RotateWorkpieceAroundGroundZ(double degrees)",
)
for token in (
    "axes.col(2) = groundUp",
    "anchor += groundUp * ((groundCenter - anchor).dot(groundUp))",
    "m_template.placement.anchorModelMm = anchor",
    "m_groundFaceSatisfied = false",
    "InvalidatePlacementDependentState",
):
    assert token in ground_snap, f"missing coupled ground-face snap token: {token}"

# Once snapped, the only model-orientation adjustment is yaw about the displayed ground Z.
ground_rotation = function_body(
    dialog,
    "void ModelWeldingFlowDialog::RotateWorkpieceAroundGroundZ(double degrees)",
    "void ModelWeldingFlowDialog::InvalidatePlacementDependentState(const QString& reason)",
)
for token in (
    "if (!m_groundFaceSatisfied)",
    "Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ())",
    "InvalidatePlacementDependentState",
):
    assert token in ground_rotation, f"missing ground-Z-only rotation token: {token}"
assert "m_template.placement.anchorModelMm =" not in ground_rotation, (
    "workpiece display rotation mutated the fixture model anchor"
)
assert "RotateWorkpieceSetup(" not in dialog
assert "RotateWorkpieceSetup(" not in dialog_header
for forbidden in ("绕地面 X", "绕地面 Y"):
    assert forbidden not in dialog, f"arbitrary floor-normal rotation returned: {forbidden}"

# Fail closed at all three user-visible gates: datum confirmation, template persistence,
# and the production-readiness check.
datum_gate = function_body(
    dialog,
    "connect(m_datumChecked, &QCheckBox::toggled",
    "connect(m_templateNameEdit, &QLineEdit::textChanged",
)
for token in (
    "checked && !m_groundFaceSatisfied",
    "m_datumChecked->setChecked(false)",
    "m_template.humanDatumConfirmed = false",
):
    assert token in datum_gate, f"missing datum-confirmation ground gate: {token}"

save_gate = function_body(
    dialog,
    "void ModelWeldingFlowDialog::SaveTemplate()",
    "void ModelWeldingFlowDialog::SaveTeaching()",
)
assert "RefreshGroundFaceCandidates();" in save_gate
assert "RefreshPreview(false);" in save_gate
assert "!m_previewInitialized" in save_gate and "!m_preview->HasCadShape()" in save_gate
assert "if (!m_groundFaceSatisfied)" in save_gate
assert save_gate.index("RefreshPreview(false);") < save_gate.index(
    "RefreshGroundFaceCandidates();"
) < save_gate.index(
    "if (!m_groundFaceSatisfied)"
) < save_gate.index("ModelWeldingWorkflow::SaveTemplate")

production_gate = function_body(
    dialog,
    "void ModelWeldingFlowDialog::CheckProductionReadiness()",
    "QString ModelWeldingFlowDialog::CurrentRobotName() const",
)
assert "RefreshGroundFaceCandidates();" in production_gate
assert "RefreshPreview(false);" in production_gate
assert "!m_previewInitialized" in production_gate and "!m_preview->HasCadShape()" in production_gate
assert "if (!m_groundFaceSatisfied)" in production_gate
assert production_gate.index("RefreshPreview(false);") < production_gate.index(
    "RefreshGroundFaceCandidates();"
) < production_gate.index(
    "if (!m_groundFaceSatisfied)"
) < production_gate.index("ModelWeldingWorkflow::ValidateTeachingStructure")

for safety_token in (
    "humanDatumConfirmed = false",
    "humanCollisionChecked = false",
    "scan.startTaught = false",
    "scan.endTaught = false",
):
    assert safety_token in dialog, f"missing V-slot move invalidation: {safety_token}"

print("MODEL_WELDING_CAD_PREVIEW_OK")
