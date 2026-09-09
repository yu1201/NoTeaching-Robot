#include "AppPaths.h"
#include "ConfigDatabase.h"
#include "PointCloudProcessingConfig.h"
#include "RobotDataHelper.h"
#include "ScanSafetyGateDialog.h"
#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <iostream>
#include <cstdlib>

QString RobotDataHelper::FindProjectFilePath(const QString& path) { return AppPaths::ResourcePath(path); }
QString RobotDataHelper::BuildProjectPath(const QString& path) { return AppPaths::WritablePath(path); }
void Check(bool ok, const char* message)
{
    if (!ok) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QTemporaryDir temp(QDir::currentPath() + "/InterlockConfig-XXXXXX");
    Check(temp.isValid(), "isolated test root creation failed");
    QString error;
    Check(AppPaths::Initialize({app.applicationFilePath(), "--data-root", temp.path()}, &error), "path init failed");
    Check(ConfigDatabase::IsAvailable(), "database init failed");
    // Regression: these exact grouped keys were previously rejected before SQLite transaction start.
    Check(ConfigDatabase::WriteScopedSettings("global", {}, "PointCloudProcessing",
        QMap<QString,QString>{{"General/ProcessingMode", "legacy"}, {"Validation/CoverageEnabled", "0"}}),
        "existing field database grouped keys rejected");
    QString value;
    Check(ConfigDatabase::ReadScopedSetting("global", {}, "PointCloudProcessing", "Validation/CoverageEnabled", &value)
        && value == "0", "grouped key point read failed");
    Check(!ConfigDatabase::WriteScopedSettings("global", {}, "PointCloudProcessing",
        QMap<QString,QString>{{"General/ProcessingMode", "uncommitted"}, {"../escape", "bad"}}),
        "invalid key accepted");
    Check(ConfigDatabase::ReadScopedSetting("global", {}, "PointCloudProcessing", "General/ProcessingMode", &value)
        && value == "legacy", "invalid batch left a partial write");
    Check(!ConfigDatabase::WriteScopedSetting("global", {}, "OtherModule", "file.ini/Value", "bad"),
        "compatibility widened to unrelated path-based settings");

    Check(ConfigDatabase::WriteScopedSetting("global", {}, "PointCloudProcessing",
        "SafetyGates/MandatorySystemInterlocksEnabled", "0"), "legacy mode seed failed");
    auto settings = PointCloudProcessingConfig::Load();
    for (bool enabled : settings.systemInterlocks.enabled) Check(enabled, "old total-off mode silently disabled an item");
    settings.validationCoverageEnabled = true;
    settings.validationSdkBaseIntegrityEnabled = true;
    settings.validationContinuityEnabled = true;
    settings.validationDenoiseRatioEnabled = true;
    settings.validationResidualEnabled = true;
    settings.validationKeyPointEnabled = true;
    settings.validationOutputEnabled = true;
    settings.validationMinFinitePointCount = 12;
    settings.validationMinProjectedSpanMm = 45.0;
    settings.validationMinSdkBaseCloudCoverageRatio = 0.20;
    settings.validationMaxSdkBaseEndpointDeviationRatio = 0.80;
    settings.validationMinStationCoverageRatio = 0.25;
    settings.validationMinLongestContinuousRatio = 0.50;
    settings.validationMaxRejectedRatio = 0.90;
    settings.validationMaxMedianResidualMm = 30.0;
    settings.validationMaxP95ResidualMm = 40.0;
    settings.validationResidualInlierThresholdMm = 20.0;
    settings.validationMinResidualInlierRatio = 0.10;
    settings.validationMinKeyPointCount = 2;
    settings.validationMinCornerCount = 1;
    settings.validationMinOutputPointCount = 3;
    settings.validationMinOutputLengthRatio = 0.20;
    Check(PointCloudProcessingConfig::Save(settings, &error), "operator threshold save failed");
    const auto controlled = PointCloudProcessingConfig::Load();
    Check(controlled.validationMinFinitePointCount == 12, "finite-point threshold was overridden");
    Check(controlled.validationMinProjectedSpanMm == 45.0, "span threshold was overridden");
    Check(controlled.validationMinSdkBaseCloudCoverageRatio == 0.20, "SDK coverage threshold was overridden");
    Check(controlled.validationMaxSdkBaseEndpointDeviationRatio == 0.80, "SDK endpoint threshold was overridden");
    Check(controlled.validationMinStationCoverageRatio == 0.25, "station coverage threshold was overridden");
    Check(controlled.validationMinLongestContinuousRatio == 0.50, "longest-continuous threshold did not preserve 50 percent");
    Check(controlled.validationMaxRejectedRatio == 0.90, "rejected-ratio threshold was overridden");
    Check(controlled.validationMaxMedianResidualMm == 30.0, "median-residual threshold was overridden");
    Check(controlled.validationMaxP95ResidualMm == 40.0, "P95-residual threshold was overridden");
    Check(controlled.validationResidualInlierThresholdMm == 20.0, "residual inlier threshold was overridden");
    Check(controlled.validationMinResidualInlierRatio == 0.10, "residual inlier ratio was overridden");
    Check(controlled.validationMinKeyPointCount == 2, "key-point threshold was overridden");
    Check(controlled.validationMinCornerCount == 1, "corner threshold was overridden");
    Check(controlled.validationMinOutputPointCount == 3, "output-point threshold was overridden");
    Check(controlled.validationMinOutputLengthRatio == 0.20, "output-length threshold was overridden");
    for (std::size_t off = 0; off < SystemInterlockCount; ++off)
    {
        settings.systemInterlocks = {};
        settings.systemInterlocks.enabled[off] = false;
        Check(PointCloudProcessingConfig::Save(settings, &error), "actual processing Save failed");
        const auto loaded = PointCloudProcessingConfig::Load();
        Check(loaded.systemInterlocks.enabled == settings.systemInterlocks.enabled, "single-item roundtrip changed other items");
        Check(loaded.validationMinLongestContinuousRatio == 0.50, "numeric threshold changed");
    }
    Check(ConfigDatabase::WriteScopedSetting("global", {}, "PointCloudProcessing",
        SystemInterlockPolicy::keys[0], "invalid"), "invalid bool seed failed");
    Check(PointCloudProcessingConfig::Load().systemInterlocks.enabled[0], "malformed bool did not default enabled");

    const auto frozen = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    settings.systemInterlocks = {};
    settings.systemInterlocks.SetEnabled(SystemInterlock::SafeRetreatPending, false);
    settings.systemInterlocks.SetEnabled(SystemInterlock::RecoveryIdentity, false);
    Check(PointCloudProcessingConfig::Save(settings, &error), "reset save failed");
    const auto applied = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    Check(!applied.IsEnabled(SystemInterlock::SafeRetreatPending)
        && !applied.IsEnabled(SystemInterlock::RecoveryIdentity),
        "saved runtime switches did not take effect immediately");
    Check(applied.IsEnabled(SystemInterlock::SingleProcess)
        == frozen.IsEnabled(SystemInterlock::SingleProcess),
        "single-process lock changed inside a running process");

    ScanSafetyGateDialog dialog([] { return true; });
    auto* table = dialog.findChild<QTableWidget*>("mandatoryGateTable");
    Check(table && table->rowCount() == 10, "missing independent rows");
    Check(!dialog.findChild<QCheckBox*>("mandatorySystemInterlocksEnabledCheckBox"), "total switch remains");
    for (int row = 0; row < 10; ++row)
    {
        auto* check = qobject_cast<QCheckBox*>(table->cellWidget(row, 0));
        Check(check && check->isEnabled(), "row is not editable");
        check->setChecked(row % 2 == 0);
    }
    QTimer confirm;
    QObject::connect(&confirm, &QTimer::timeout, [&] {
        if (auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget()))
        {
            const auto choice = box->standardButtons().testFlag(QMessageBox::Yes) ? QMessageBox::Yes : QMessageBox::Ok;
            box->button(choice)->click();
        }
    });
    confirm.start(10);
    dialog.findChild<QPushButton*>("saveSafetyGateButton")->click();
    confirm.stop();
    Check(!dialog.HasUnsavedChanges(), "UI save did not complete");
    const auto loaded = PointCloudProcessingConfig::Load();
    for (std::size_t i = 0; i < SystemInterlockCount; ++i)
        Check(loaded.systemInterlocks.enabled[i] == (i % 2 == 0), "mixed UI switches did not persist independently");
    Check(loaded.validationMinLongestContinuousRatio == 0.50, "UI save changed unrelated numeric threshold");
    const auto uiApplied = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    for (std::size_t i = 1; i < SystemInterlockCount; ++i)
        Check(uiApplied.enabled[i] == (i % 2 == 0), "mixed UI switch did not apply immediately");
    std::cout << "PASS: grouped-key persistence, operator threshold control, ten independent switches, immediate runtime apply, UI save\n";
}
