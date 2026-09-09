#include "AppPaths.h"
#include "RobotModePreparationStore.h"
#include <QCoreApplication>
#include <QDir>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <cstdlib>
#include <iostream>

using namespace RobotModePreparationStore;
static int checks = 0;
static void Check(bool ok, const char* why)
{ ++checks; if (!ok) { std::cerr << "FAIL " << why << '\n'; std::exit(1); } }
static Binding Sample(const QString& robot = "RobotC")
{ return {robot,"Inovance","192.0.2.25","test-firmware-1","test-model-A",2222,1}; }
static Record Proof(const QString& plan="1_motor_ds_off_ds_motor")
{ return {plan,"2026-09-07T12:34:56.123Z",QStringLiteral("自动测试通过；恢复模式和伺服回读通过。")}; }
static QString Raw(const Binding& binding)
{
    QString value;
    Check(ConfigDatabase::ReadScopedSetting("robot",binding.robotName,Module(),Key(),&value),"read raw isolated record");
    return value;
}
int main(int argc,char** argv)
{
    QCoreApplication app(argc,argv);
    const auto args=app.arguments(); Check(args.size()==3,"isolated root and phase required");
    QString error;
    Check(AppPaths::Initialize({args[0],"--data-root",args[1]},&error),"initialize isolated test root");
    Check(ConfigDatabase::DatabasePath().startsWith(QDir::fromNativeSeparators(args[1])),"never production database");
    const auto binding=Sample(); Record result;
    if(args[2]=="seed")
    {
        Check(Load(binding,result,error)==LoadStatus::NotFound && result.planId.isEmpty(),"missing record");
        Check(Revoke(Sample("EmptyRevoked"),error)
            && Load(Sample("EmptyRevoked"),result,error)==LoadStatus::NotFound,"revoking absent record still leaves a blocking tombstone");
        Check(ConfigDatabase::WriteScopedSetting("robot",binding.robotName,"RobotAdaptorAcceptance/test","ModeCombinationEvidence","all combinations PASS"),"seed unrelated acceptance text");
        Check(Load(binding,result,error)==LoadStatus::NotFound,"free-form acceptance text never migrates to strategy");
        Check(SaveVerified(binding,Proof(),error),"save verified strategy");
        Check(Load(binding,result,error)==LoadStatus::Found && result.planId==Proof().planId && result.evidence==Proof().evidence,"exact roundtrip");
        Check(SaveVerified(Sample("RobotB"),Proof("2_motor_ds_off_ds_motor"),error),"save independent robot");
        Check(Load(binding,result,error)==LoadStatus::Found && result.planId==Proof().planId,"second robot never overwrites first");
        auto tombstone=Sample("Revoked");
        Check(SaveVerified(tombstone,Proof(),error) && Revoke(tombstone,error),"revoke writes tombstone");
        Check(Load(tombstone,result,error)==LoadStatus::NotFound && !error.isEmpty() && result.planId.isEmpty(),"revoked strategy not reusable");
        auto revokedJson=QJsonDocument::fromJson(Raw(tombstone).toUtf8()).object();
        Check(revokedJson.value("revoked").toBool() && !revokedJson.value("passed").toBool(),"persistent tombstone not deletion");
        QString unused;
        Check(ConfigDatabase::ReadScopedSettingStatus("global",QString(),Module(),Key(),&unused)==ConfigDatabase::ReadStatus::NotFound,"no global writes");
    }
    else if(args[2]=="restart")
    {
        Check(Load(binding,result,error)==LoadStatus::Found && result.planId==Proof().planId,"restore exact strategy in new process");
        Check(Load(Sample("RobotB"),result,error)==LoadStatus::Found && result.planId=="2_motor_ds_off_ds_motor","robot isolation survives restart");
        Check(Load(Sample("Revoked"),result,error)==LoadStatus::NotFound,"revocation survives restart");
        for(int field=0;field<6;++field)
        {
            auto changed=binding;
            switch(field) { case 0: changed.driver="OtherDriver";break; case 1:changed.host="192.0.2.26";break;
                case 2:changed.port=3333;break;case 3:changed.firmware="test-firmware-2";break;
                case 4:changed.controllerModel="test-model-B";break;case 5:changed.revision=2;break; }
            result=Proof();
            Check(Load(changed,result,error)==LoadStatus::Mismatch && result.planId.isEmpty() && !error.isEmpty(),"each changed binding rejects old strategy");
        }
        auto incomplete=binding;incomplete.firmware.clear();
        Check(Load(incomplete,result,error)==LoadStatus::Error && !SaveVerified(incomplete,Proof(),error) && !Revoke(incomplete,error),"incomplete binding rejected");
        incomplete=binding;incomplete.port=0;
        Check(!SaveVerified(incomplete,Proof(),error),"invalid endpoint rejected");
        incomplete=binding;incomplete.robotName="../RobotC";
        Check(!SaveVerified(incomplete,Proof(),error),"invalid robot scope rejected");
        auto invalidProof=Proof();invalidProof.verifiedAtUtc="2026-09-07T20:34:56+08:00";
        Check(!SaveVerified(binding,invalidProof,error),"timestamp must explicitly be UTC");
        invalidProof=Proof();invalidProof.evidence.clear();
        Check(!SaveVerified(binding,invalidProof,error),"missing machine evidence rejected");
        invalidProof=Proof("Move();");Check(!SaveVerified(binding,invalidProof,error),"plan id is data not executable text");
        invalidProof=Proof("valid_plan\n");Check(!SaveVerified(binding,invalidProof,error),"plan id cannot contain a trailing control character");
        auto corrupt=Sample("Corrupt");
        Check(ConfigDatabase::WriteScopedSetting("robot",corrupt.robotName,Module(),Key(),"{broken"),"seed JSON corruption");
        Check(Load(corrupt,result,error)==LoadStatus::Error,"corrupt JSON cannot restore");
        Check(SaveVerified(corrupt,Proof(),error),"explicit new verified record can replace parse corruption");
        auto goodJson=QJsonDocument::fromJson(Raw(corrupt).toUtf8()).object();
        for(int tamper=0;tamper<6;++tamper)
        {
            auto json=goodJson;
            switch(tamper) {
                case 0:json["passed"]=false;break;
                case 1:json["restoreVerified"]=false;break;
                case 2:json["schema"]=1.5;break;
                case 3:json["passed"]="true";break;
                case 4:{auto identity=json["binding"].toObject();identity["port"]=2222.5;json["binding"]=identity;break;}
                case 5:json["revoked"]=true;break;
            }
            Check(ConfigDatabase::WriteScopedSetting("robot",corrupt.robotName,Module(),Key(),Serialize(json)),"seed structurally invalid record");
            Check(Load(corrupt,result,error)==LoadStatus::Error && result.planId.isEmpty(),"invalid flags/schema/binding reject reuse");
        }
        auto stolen=goodJson;auto identity=stolen["binding"].toObject();identity["robotName"]="AnotherRobot";stolen["binding"]=identity;
        Check(ConfigDatabase::WriteScopedSetting("robot",corrupt.robotName,Module(),Key(),Serialize(stolen)),"seed wrong robot binding");
        Check(Load(corrupt,result,error)==LoadStatus::Mismatch,"copied strategy cannot cross robot identity");

        const QString before=Raw(binding);
        {
            QSqlDatabase db=QSqlDatabase::addDatabase("QSQLITE","mode_store_injection");db.setDatabaseName(ConfigDatabase::DatabasePath());
            Check(db.open(),"open isolated test injector");QSqlQuery q(db);
            const QString predicate="NEW.scope_id='RobotC' AND NEW.module='RobotPara/ModePreparation' AND NEW.key_name='SelectedStrategy'";
            Check(q.exec("CREATE TRIGGER mode_write_fail BEFORE INSERT ON settings WHEN "+predicate+" BEGIN SELECT RAISE(ABORT,'test-only'); END"),"install write failure");
            Check(!SaveVerified(binding,Proof("different_plan"),error) && !error.isEmpty(),"write failure propagated");
            Check(Raw(binding)==before,"failed write rolled back");
            Check(!Revoke(binding,error),"failed revoke reported, never falsely acknowledged");
            Check(q.exec("DROP TRIGGER mode_write_fail"),"drop write trigger");
            Check(q.exec("CREATE TRIGGER mode_readback_mismatch AFTER INSERT ON settings WHEN "+predicate+" BEGIN UPDATE settings SET value_text='corrupted',encrypted=0 WHERE scope_id=NEW.scope_id AND module=NEW.module AND key_name=NEW.key_name; END"),"install readback mismatch");
            Check(!SaveVerified(binding,Proof("different_plan"),error) && !error.isEmpty(),"readback mismatch propagated");
            Check(Raw(binding)==before,"readback mismatch rolled back entire transaction");
            Check(q.exec("DROP TRIGGER mode_readback_mismatch"),"drop mismatch trigger");
            Check(q.exec("CREATE TRIGGER mode_readback_error AFTER INSERT ON settings WHEN "+predicate+" BEGIN UPDATE settings SET value_text='not-an-encrypted-value',encrypted=1 WHERE scope_id=NEW.scope_id AND module=NEW.module AND key_name=NEW.key_name; END"),"install readback decode failure");
            Check(!SaveVerified(binding,Proof("different_plan"),error) && !error.isEmpty(),"readback decode failure propagated");
            Check(Raw(binding)==before,"readback decode failure rolled back");
            Check(q.exec("DROP TRIGGER mode_readback_error"),"drop readback trigger");
            Check(q.exec("UPDATE settings SET value_text='broken-encoding',encrypted=1 WHERE scope_id='RobotC' AND module='RobotPara/ModePreparation' AND key_name='SelectedStrategy'"),"seed persistent decode error");
            Check(Load(binding,result,error)==LoadStatus::Error && error.contains(QStringLiteral("数据库或记录解码")),"database decode error distinguished from missing record and JSON corruption");
            Check(q.exec("DELETE FROM settings WHERE scope_id='RobotC' AND module='RobotPara/ModePreparation' AND key_name='SelectedStrategy'"),"remove corrupted isolated row only");
        }
        QSqlDatabase::removeDatabase("mode_store_injection");
        Check(SaveVerified(binding,Proof(),error) && Revoke(binding,error),"persist final revocation");
    }
    else
    {
        Check(Load(binding,result,error)==LoadStatus::NotFound && result.planId.isEmpty(),"failed/revoked strategy never resurrects after another process restart");
        Check(Load(Sample("RobotB"),result,error)==LoadStatus::Found,"other robot strategy remains valid");
    }
    std::cout<<"PASS mode preparation store "<<args[2].toStdString()<<" ("<<checks<<" checks)\n";
}
