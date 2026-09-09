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
    settings.validationMinProjectedSpanMm = 456.0;
    for (std::size_t off = 0; off < SystemInterlockCount; ++off)
    {
        settings.systemInterlocks = {};
        settings.systemInterlocks.enabled[off] = false;
        Check(PointCloudProcessingConfig::Save(settings, &error), "actual processing Save failed");
        const auto loaded = PointCloudProcessingConfig::Load();
        Check(loaded.systemInterlocks.enabled == settings.systemInterlocks.enabled, "single-item roundtrip changed other items");
        Check(loaded.validationMinProjectedSpanMm == 456.0, "numeric threshold changed");
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
    Check(loaded.validationMinProjectedSpanMm == 456.0, "UI save changed unrelated numeric threshold");
    const auto uiApplied = PointCloudProcessingConfig::RuntimeSystemInterlocks();
    for (std::size_t i = 1; i < SystemInterlockCount; ++i)
        Check(uiApplied.enabled[i] == (i % 2 == 0), "mixed UI switch did not apply immediately");
    std::cout << "PASS: grouped-key persistence, atomic rejection, ten independent switches, immediate runtime apply, UI save\n";
}
