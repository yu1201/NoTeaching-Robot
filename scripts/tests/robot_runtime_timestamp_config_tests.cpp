#include "AppPaths.h"
#include "MeasureThenWeldRuntimeConfig.h"
#include <QCoreApplication>
#include <QDir>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <cstdlib>
#include <iostream>

// Deliberately no robot driver/SDK linkage: only registry capability declarations.
const RobotDriverSetupProfile* RobotDriverRegistry::SetupProfile(int type)
{
    static RobotDriverSetupProfile native = [] { RobotDriverSetupProfile p; p.supportsRobotTimestamp=true; return p; }();
    static RobotDriverSetupProfile step = [] { auto p=native; p.usesStepTimestampInterface=true; return p; }();
    static RobotDriverSetupProfile pcOnly;
    return type==11 ? &native : type==22 ? &step : type==33 ? &pcOnly : nullptr;
}
using namespace MeasureThenWeldRuntimeConfig;
static void Check(bool ok, const char* why)
{ if (!ok) { std::cerr << "FAIL " << why << '\n'; std::exit(1); } }
static bool WriteType(const QString& robot, int type)
{ return ConfigDatabase::WriteScopedSetting("robot",robot,RobotSettingsGroup(),"RobotType",QString::number(type)); }
int main(int argc, char** argv)
{
    QCoreApplication app(argc,argv);
    const auto args=app.arguments(); Check(args.size()==3,"isolated root and phase required");
    QString error;
    Check(AppPaths::Initialize({args[0],"--data-root",args[1]},&error),"initialize isolated test root");
    Check(ConfigDatabase::DatabasePath().startsWith(QDir::fromNativeSeparators(args[1])),"not production database");
    if (args[2]=="seed")
    {
        Check(WriteType("Native",11) && WriteType("StepA",22) && WriteType("StepB",22) && WriteType("PcOnly",33),"seed identities");
        Check(LoadScanTimestampSource("Native")==ScanTimestampSource::Robot,"native default robot time");
        Check(LoadScanTimestampSource("PcOnly")==ScanTimestampSource::Pc,"unknown native support defaults PC");
        Check(LoadScanTimestampSource("Unknown")==ScanTimestampSource::Pc,"unknown profile PC");
        Check(LoadScanTimestampSource("")==ScanTimestampSource::Pc,"missing robot never global runtime");
        Check(LoadStepSdkInterfaceMode("StepA")==StepSdkInterfaceMode::Timestamp,"default STEP timestamp SDK");
        Check(ConfigDatabase::WriteScopedSetting("global",QString(),SettingsGroup(),TimestampSourceKey(),"pc"),"seed migration source");
        Check(LoadScanTimestampSource("Native")==ScanTimestampSource::Pc,"legacy read compatibility");
        QString value;
        Check(ConfigDatabase::ReadScopedSettingStatus("robot","Native",RobotSettingsGroup(),TimestampSourceKey(),&value)
            ==ConfigDatabase::ReadStatus::NotFound,"migration load is read only");
        Check(SaveScanTimestampSource("Native",ScanTimestampSource::Robot),"save selected robot");
        Check(LoadScanTimestampSource("Native")==ScanTimestampSource::Robot && LoadScanTimestampSource("StepA")==ScanTimestampSource::Pc,
            "saving one robot does not change another fallback");
        Check(ConfigDatabase::ReadScopedSetting("global",QString(),SettingsGroup(),TimestampSourceKey(),&value) && value=="pc","new save never writes global");
        Check(SaveScanTimestampSource("StepA",ScanTimestampSource::Robot) && SaveScanTimestampSource("StepB",ScanTimestampSource::Robot),"seed scoped timestamp preferences");
        Check(SaveStepSdkInterfaceMode("StepA",StepSdkInterfaceMode::Legacy),"save STEP A legacy");
        Check(LoadScanTimestampSource("StepA")==ScanTimestampSource::Pc && LoadScanTimestampSource("StepB")==ScanTimestampSource::Robot,
            "STEP SDK choice isolated by robot");
        Check(LoadStepSdkInterfaceMode("StepB")==StepSdkInterfaceMode::Timestamp,"STEP B interface unchanged");
        Check(SaveScanTimestampSource("PcOnly",ScanTimestampSource::Robot),"retain explicit preference for diagnostic");
        Check(LoadConfiguredScanTimestampSource("PcOnly")==ScanTimestampSource::Robot && LoadScanTimestampSource("PcOnly")==ScanTimestampSource::Pc,
            "no native brand logs preference but effective PC");
        Check(EffectiveScanTimestampSource(ScanTimestampSource::Robot,false)==ScanTimestampSource::Pc
            && EffectiveScanTimestampSource(ScanTimestampSource::Robot,true)==ScanTimestampSource::Robot
            && EffectiveScanTimestampSource(ScanTimestampSource::Pc,true)==ScanTimestampSource::Pc,"actual-driver final capability gate");
        Check(!SaveScanTimestampSource("",ScanTimestampSource::Robot) && !SaveStepSdkInterfaceMode("",StepSdkInterfaceMode::Timestamp),"empty identity cannot write");
        QSqlDatabase db=QSqlDatabase::addDatabase("QSQLITE","timestamp_failure");
        db.setDatabaseName(ConfigDatabase::DatabasePath()); Check(db.open(),"open isolated trigger connection");
        QSqlQuery q(db);
        Check(q.exec("CREATE TRIGGER reject_runtime_write BEFORE INSERT ON settings WHEN (NEW.scope_id='Native' AND NEW.key_name='ScanTimestampSource') OR (NEW.scope_id='StepB' AND NEW.key_name='StepSdkInterfaceMode') BEGIN SELECT RAISE(ABORT,'test-only'); END"),"install failure trigger");
        Check(!SaveScanTimestampSource("Native",ScanTimestampSource::Pc),"save failure propagated");
        Check(LoadScanTimestampSource("Native")==ScanTimestampSource::Robot,"failed save preserves previous value");
        Check(!SaveStepSdkInterfaceMode("StepB",StepSdkInterfaceMode::Legacy)
            && LoadStepSdkInterfaceMode("StepB")==StepSdkInterfaceMode::Timestamp,"STEP save failure preserves interface selection");
        Check(q.exec("DROP TRIGGER reject_runtime_write"),"remove isolated trigger");
    }
    else
    {
        Check(LoadScanTimestampSource("Native")==ScanTimestampSource::Robot,"robot preference persisted");
        Check(LoadStepSdkInterfaceMode("StepA")==StepSdkInterfaceMode::Legacy
            && LoadStepSdkInterfaceMode("StepB")==StepSdkInterfaceMode::Timestamp,"independent interface modes persisted");
        Check(LoadScanTimestampSource("StepA")==ScanTimestampSource::Pc && LoadScanTimestampSource("StepB")==ScanTimestampSource::Robot,"effective modes persist across process");
        Check(LoadScanTimestampSource("PcOnly")==ScanTimestampSource::Pc,"PC-only brand still gated after restart");
    }
    std::cout << "PASS robot runtime timestamp config " << args[2].toStdString() << '\n';
}
