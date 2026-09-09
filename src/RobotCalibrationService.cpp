#include "RobotCalibrationService.h"
#include "RobotDriverAdaptor.h"
#include "RobotOperationLease.h"
#include "ConfigDatabase.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QDateTime>
#include <QUuid>
#include <QThread>
#include <random>
#include <set>
#include <sstream>

namespace
{
using namespace RobotKinematicsModel;
QString S(const std::string& v) {return QString::fromStdString(v);}
QJsonArray Matrix(const Eigen::MatrixXd& m)
{
    QJsonArray a;for(int r=0;r<m.rows();++r) {QJsonArray row;for(int c=0;c<m.cols();++c)row.append(m(r,c));a.append(row);}return a;
}
bool ReadMatrix(const QJsonValue& value,int rows,int cols,Eigen::MatrixXd& m)
{
    if(!value.isArray()||value.toArray().size()!=rows)return false;
    const auto a=value.toArray();m.resize(rows,cols);
    for(int r=0;r<rows;++r) {const auto row=a[r].toArray();if(row.size()!=cols)return false;
        for(int c=0;c<cols;++c) {if(!row[c].isDouble()||!std::isfinite(row[c].toDouble()))return false;m(r,c)=row[c].toDouble();}}
    return true;
}
QJsonObject Point(const RobotKinematicsPoint& p)
{return {{"joints_deg",Matrix(p.joints)},{"tcp_in_work_mm",Matrix(p.pose)},
    {"request",S(p.request)},{"response",S(p.response)}};}
bool Store(const RobotDriverAdaptor& driver,const QString& runId,const QJsonObject& document,const QString& report)
{
    // One transaction for each checkpoint. History and active candidate are separate.
    return ConfigDatabase::WriteScopedSettings("robot",S(driver.RobotName()),"CalibrationRuns/"+runId,
        QMap<QString,QString>{{"Document",QString::fromUtf8(QJsonDocument(document).toJson(QJsonDocument::Compact))},
            {"Report",report},{"UpdatedAt",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)}})
        && ConfigDatabase::WriteScopedSetting("robot",S(driver.RobotName()),"CalibrationRuns","LatestRun",runId);
}
QString RunId() {return QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz")+"_"+QUuid::createUuid().toString(QUuid::WithoutBraces);}
}
namespace RobotCalibrationService
{
QJsonObject EncodeModel(const Model& m)
{
    QJsonArray links,mappings,compliance;
    for(int i=0;i<6;++i) {links.append(Matrix(m.nominal[i]));mappings.append(Matrix(m.mappings[i]));compliance.append(Matrix(m.compliance[i]));}
    return {{"schema","equivalent-gravity-6r-v1"},{"nominal",links},{"geometry",Matrix(m.geometry)},
        {"featureGeometry",Matrix(m.featureGeometry)},{"mappings",mappings},{"compliance",compliance}};
}
bool DecodeModel(const QJsonObject& j,Model& m)
{
    if(j["schema"]!="equivalent-gravity-6r-v1")return false;
    auto links=j["nominal"].toArray(),maps=j["mappings"].toArray(),coeff=j["compliance"].toArray();
    if(links.size()!=6||maps.size()!=6||coeff.size()!=6)return false;
    Eigen::MatrixXd v;
    if(!ReadMatrix(j["geometry"],42,1,v))return false;m.geometry=v;
    if(!ReadMatrix(j["featureGeometry"],42,1,v))return false;m.featureGeometry=v;
    if(m.geometry.cwiseAbs().maxCoeff()>=20||m.featureGeometry.cwiseAbs().maxCoeff()>=20)return false;
    for(int i=0;i<6;++i)
    {
        if(!ReadMatrix(links[i],4,4,v)||!Rigid(v))return false;m.nominal[i]=v;
        const int rank=coeff[i].toArray().size();if(rank<0||rank>(6-i)*4)return false;
        if(!ReadMatrix(maps[i],(6-i)*4,rank,v))return false;m.mappings[i]=v;
        if(!ReadMatrix(coeff[i],rank,1,v))return false;m.compliance[i]=v;
        if(rank && m.compliance[i].cwiseAbs().maxCoeff()>=20)return false;
    }
    return true;
}
QString LatestReport(const std::string& robot)
{
    QString id,report;
    if(!ConfigDatabase::ReadScopedSetting("robot",S(robot),"CalibrationRuns","LatestRun",&id))return "尚无资产/优化记录。";
    if(!ConfigDatabase::ReadScopedSetting("robot",S(robot),"CalibrationRuns/"+id,"Report",&report))return "记录读取失败。";
    return "记录："+id+"\n"+report;
}
}

bool RobotDriverAdaptor::ReadKinematicsReference(const RobotKinematicsProfile&,RobotKinematicsReference&,std::string& error)
{error="品牌底层尚未接入控制器计算上下文；不能用本地正解冒充控制器参考值。";return false;}
bool RobotDriverAdaptor::ReadControllerHandEye(int,RobotControllerHandEye&,std::string& error)
{error="品牌尚未定义带传感器、工具和来源绑定的手眼标定读取链路。";return false;}
bool RobotDriverAdaptor::ValidateControllerHandEyeContext(const RobotControllerHandEye&,std::string& error)
{error="品牌尚未定义控制器手眼上下文验证。";return false;}
bool RobotDriverAdaptor::CalculateControllerForward(const RobotKinematicsReference&,const Joint&,RobotKinematicsPoint&,std::string& error)
{error="品牌未接入控制器正解计算接口。";return false;}
bool RobotDriverAdaptor::CalculateControllerInverse(const RobotKinematicsReference&,const RobotKinematicsPoint&,RobotKinematicsPoint&,std::string& error)
{error="品牌未接入控制器逆解计算接口。";return false;}
bool RobotDriverAdaptor::DiscoverCalibrationAssets(RobotCalibrationDiscovery& r,std::atomic_bool& cancel,std::string& error)
{
    r={};r.recipeRevision="adaptor-declared-assets-v1";
    const auto endpoint=ControlEndpoint();r.identity=endpoint.host+":"+std::to_string(endpoint.port)+"|"+DriverDescriptor().typeName;
    if(Supports(RobotDriverCapability::ToolDataRead))
    {
        for(int index=0;index<16&&!cancel.load();++index)
        {
            T_ROBOT_COORS tool;const bool ok=GetToolData(index,tool);
            QJsonArray pose;pose.append(tool.dX);pose.append(tool.dY);pose.append(tool.dZ);pose.append(tool.dRX);pose.append(tool.dRY);pose.append(tool.dRZ);
            r.assets.push_back({"tool","Tool"+std::to_string(index),"adaptor:GetToolData","",ok?"acquired":"missing",
                ok?"已读取工具；未擅自绑定为相机工具。":GetLastRobotError(),
                ok?QJsonDocument(pose).toJson(QJsonDocument::Compact).toStdString():""});
        }
    }
    if(Supports(RobotDriverCapability::HandEyeMatrixRead)&&!cancel.load())
    {
        double rotation[9]{},translation[3]{};std::string why;
        const bool ok=GetHandEyeMatrixVariable("eye",rotation,translation,&why);
        QJsonArray matrix;for(double v:rotation)matrix.append(v);for(double v:translation)matrix.append(v);
        r.assets.push_back({"handeye","eye","adaptor:GetHandEyeMatrixVariable(eye)","",ok?"candidate":"missing",
            ok?"已获取品牌定义的R+T；仍需确认相机归属和安装参考系。":why,
            ok?QJsonDocument(matrix).toJson(QJsonDocument::Compact).toStdString():""});
    }
    if(!Supports(RobotDriverCapability::CalibrationAssetDiscovery))
        r.assets.push_back({"recipe",DriverDescriptor().typeName,"brand backend","","unsupported",
            "本品牌仅检查已声明适配接口；厂商FTP标定目录尚未固化，不猜测路径或变量。",""});
    if(cancel.load()){error="已取消。";return false;}return true;
}
RobotCalibrationRunResult RobotDriverAdaptor::AcquireCalibrationAssets(std::atomic_bool& cancel,const RobotCalibrationProgress& progress)
{
    RobotCalibrationRunResult result;const QString id=RunId();result.runId=id.toStdString();
    QString leaseError;const auto lease=RobotOperationLease::TryAcquire(this,"只读发现标定资产",&leaseError);
    if(!lease){result.report=leaseError.toStdString();return result;}
    QJsonObject doc{{"schema",1},{"kind","asset-discovery"},{"state","reading"},{"robot",S(RobotName())}};
    if(!Store(*this,id,doc,"正在只读搜索；未完成。")){result.report="数据库写入失败，未开始搜索。";return result;}
    RobotCalibrationDiscovery discovery;std::string error;
    if(progress)progress("按品牌固定来源只读获取工具、运动学与相机标定资产……");
    bool ok=false;
    try {ok=DiscoverCalibrationAssets(discovery,cancel,error);}catch(const std::exception& e){error=e.what();}
    QJsonArray assets;std::ostringstream report;
    report<<"来源链路："<<discovery.recipeRevision<<"\n设备："<<discovery.identity<<"\n";
    for(const auto& a:discovery.assets)
    {
        assets.append(QJsonObject{{"kind",S(a.kind)},{"name",S(a.name)},{"source",S(a.source)},
            {"sha256",S(a.fingerprint)},{"status",S(a.status)},{"detail",S(a.detail)},{"raw",S(a.raw)}});
        report<<a.name<<" ["<<a.status<<"] "<<a.source<<"\n"<<a.detail<<"\n";
    }
    report<<"\n搜索完成不等于标定通过；未覆盖生产矩阵或机械参数。\n"<<error;
    doc["assets"]=assets;doc["identity"]=S(discovery.identity);doc["recipe"]=S(discovery.recipeRevision);doc["state"]=ok?"complete":"incomplete";
    result.report=report.str();result.passed=ok&&Store(*this,id,doc,S(result.report));
    if(!ok)Store(*this,id,doc,S(result.report));
    if(ok&&!result.passed)result.report+="\n数据库保存失败。";
    return result;
}

RobotCalibrationRunResult RobotDriverAdaptor::OptimizeKinematicsModel(const RobotKinematicsOptimizationOptions& options,
    std::atomic_bool& cancel,const RobotCalibrationProgress& progress)
{
    RobotCalibrationRunResult result;const QString id=RunId();result.runId=id.toStdString();
    if(!Supports(RobotDriverCapability::ControllerKinematicsCalculate))
    {result.report="此品牌尚未声明控制器正逆解计算能力，优化受限；不会使用本地模型生成自证样本。";return result;}
    QString why;const auto lease=RobotOperationLease::TryAcquire(this,"运动学模型优化（不运动）",&why);
    if(!lease){result.report=why.toStdString();return result;}
    const unsigned seed=options.seed?options.seed:std::random_device{}();
    QJsonObject doc{{"schema",1},{"kind","kinematics-optimization"},{"state","preparing"},
        {"robot",S(RobotName())},{"seed",double(seed)},{"production_enabled",false}};
    QJsonArray records,ikEvidence;std::string error;
    auto save=[&](const QString& text){doc["samples"]=records;doc["inverse_validation"]=ikEvidence;return Store(*this,id,doc,text);};
    auto fail=[&](const std::string& text){result.report=text;doc["state"]=cancel.load()?"cancelled":"failed";save(S(text));return result;};
    if(options.trainingCount<80||options.trainingCount>1000||options.validationCount<40||options.validationCount>300
        ||!std::isfinite(options.maxPositionErrorMm)||options.maxPositionErrorMm<=0||options.maxPositionErrorMm>.1
        ||!std::isfinite(options.maxOrientationErrorDeg)||options.maxOrientationErrorDeg<=0||options.maxOrientationErrorDeg>.1)
        return fail("采样数或验收阈值无效。阈值在采样前固定，不能用测试集调参。");
    if(!save("开始新轮次；仅调用计算接口，不运动。"))return fail("数据库不可写，未开始采样。");
    try
    {
        RobotKinematicsReference reference;
        if(!ReadKinematicsReference(options.profile,reference,error))return fail(error);
        doc["identity"]=S(reference.identity);doc["source"]=S(reference.source);
        doc["profile"]=QJsonArray{options.profile.tool,options.profile.workobject,options.profile.load};
        doc["tool"]=Matrix(reference.tool);doc["work"]=Matrix(reference.work);
        doc["max_position_mm"]=options.maxPositionErrorMm;doc["max_orientation_deg"]=options.maxOrientationErrorDeg;
        Eigen::Matrix<double,6,2> domain;
        for(int i=0;i<6;++i)
        { domain(i,0)=std::max(reference.limits(i,0)+2.,reference.current[i]-60.);
          domain(i,1)=std::min(reference.limits(i,1)-2.,reference.current[i]+60.);
          if(!std::isfinite(domain(i,0))||!std::isfinite(domain(i,1))||domain(i,1)-domain(i,0)<20)return fail("当前关节附近没有足够的六维计算采样域。"); }
        doc["domain_deg"]=Matrix(domain);doc["state"]="sampling";
        if(!save("采样计划与独立验证域已保存。"))return fail("数据库保存采样计划失败。");
        auto check=[&](){
            if(cancel.load()||lease->CancellationRequested()){cancel.store(true);error="已取消；没有发送停止/运动命令。";return false;}
            RobotKinematicsReference now;
            if(!ReadKinematicsReference(options.profile,now,error))return false;
            if(now.identity!=reference.identity){error="机械参数、工具/工件、模式上下文或来源指纹变化，拒绝复用。";return false;}return true;
        };
        Model model;model.nominal=reference.nominal;
        std::vector<Sample> train,test;std::vector<RobotKinematicsPoint> testPoints;
        std::mt19937 rng(seed);std::uniform_real_distribution<double> random(0.,1.);std::set<std::string> unique;
        const int total=options.trainingCount+options.validationCount;
        for(int n=0;n<total;++n)
        {
            if(n%16==0&&!check())return fail(error);
            if(cancel.load())return fail("已取消采样。");
            Joint q;for(int i=0;i<6;++i)q[i]=domain(i,0)+random(rng)*(domain(i,1)-domain(i,0));
            RobotKinematicsPoint point;
            QThread::msleep(200);
            if(!CalculateControllerForward(reference,q,point,error))return fail("控制器正解样本失败："+error);
            const auto sampleKey=QJsonDocument(Matrix(point.joints)).toJson(QJsonDocument::Compact).toStdString();
            if(!unique.insert(sampleKey).second)return fail("控制器正解返回重复关节样本，拒绝调整采样计划重试。");
            Sample s{point.joints,reference.work*point.pose*reference.tool.inverse()};
            if(!Rigid(s.flange))return fail("控制器样本含无效变换。");
            QJsonObject record=Point(point);record["split"]=n<options.trainingCount?"train":"heldout";records.append(record);
            if(n<options.trainingCount)train.push_back(s);else {test.push_back(s);testPoints.push_back(point);}
            if(progress)progress("控制器计算采样 "+std::to_string(n+1)+"/"+std::to_string(total)+"；未发送运动。");
            if(n%16==15&&!save("正在采样，保留未完成证据。"))return fail("采样记录保存失败。");
        }
        doc["state"]="fitting";if(!save("采样完成，开始仅训练集拟合。"))return fail("样本保存失败。");
        if(!Fit(train,model,cancel,progress,error))return fail(error);
        if(!check())return fail(error);
        const auto training=Evaluate(model,train), heldout=Evaluate(model,test);
        doc["model"]=RobotCalibrationService::EncodeModel(model);
        doc["training_max_mm"]=training.positionMax;doc["heldout_max_mm"]=heldout.positionMax;
        doc["heldout_rms_mm"]=heldout.positionRms;doc["heldout_max_deg"]=heldout.angleMax;
        if(heldout.positionMax>options.maxPositionErrorMm||heldout.angleMax>options.maxOrientationErrorDeg)
            return fail("候选模型独立验证超差：最大位置="+std::to_string(heldout.positionMax)+" mm；姿态="+std::to_string(heldout.angleMax)+" deg。候选未启用。");
        // Novel offset targets, not just FK -> IK of training input. Both local
        // and native IK are checked against independent native FK, no movement.
        double inverseMax=0,inverseAngle=0;int inverseCount=0;
        for(int n=0;n<12;++n)
        {
            if(!check())return fail(error);
            RobotKinematicsPoint target=testPoints[n],native,verified,localVerified;
            target.pose(0,3)+=.35;target.pose(1,3)-=.25;target.pose(2,3)+=.20;
            target.pose.topLeftCorner<3,3>()=(Eigen::AngleAxisd(.0003,Eigen::Vector3d::UnitZ())*target.pose.topLeftCorner<3,3>()).eval();
            if(!CalculateControllerInverse(reference,target,native,error))return fail("控制器逆解独立验证失败："+error);
            Joint local=testPoints[n].joints;const Eigen::Matrix4d flange=reference.work*target.pose*reference.tool.inverse();
            if(!Inverse(model,flange,local,domain))return fail("候选模型本地逆解未收敛或超出验证域。");
            QThread::msleep(200);
            if(!CalculateControllerForward(reference,local,localVerified,error)
                ||!CalculateControllerForward(reference,native.joints,verified,error))return fail("逆解回送控制器正解验证失败："+error);
            const double nativeMm=(verified.pose.topRightCorner<3,1>()-target.pose.topRightCorner<3,1>()).norm();
            const double nativeDeg=Eigen::AngleAxisd(verified.pose.topLeftCorner<3,3>()
                *target.pose.topLeftCorner<3,3>().transpose()).angle()*180./3.14159265358979323846;
            if(nativeMm>options.maxPositionErrorMm||nativeDeg>options.maxOrientationErrorDeg)
                return fail("控制器原生逆解→原生正解闭环超差，不能作为候选模型验收基准。");
            const double mm=(localVerified.pose.topRightCorner<3,1>()-target.pose.topRightCorner<3,1>()).norm();
            const double deg=Eigen::AngleAxisd(localVerified.pose.topLeftCorner<3,3>()*target.pose.topLeftCorner<3,3>().transpose()).angle()*180./3.14159265358979323846;
            inverseMax=std::max(inverseMax,mm);inverseAngle=std::max(inverseAngle,deg);++inverseCount;
            ikEvidence.append(QJsonObject{{"target",Matrix(target.pose)},{"native_inverse",Point(native)},
                {"native_forward",Point(verified)},{"native_error_mm",nativeMm},{"native_error_deg",nativeDeg},
                {"local_inverse_forward",Point(localVerified)},{"error_mm",mm},{"error_deg",deg}});
            if(mm>options.maxPositionErrorMm||deg>options.maxOrientationErrorDeg)return fail("新目标逆解独立闭环超差，候选未启用。");
        }
        if(!check())return fail(error);
        doc["inverse_max_mm"]=inverseMax;doc["inverse_max_deg"]=inverseAngle;doc["inverse_count"]=inverseCount;doc["state"]="validated-candidate";
        result.report="控制器计算模型优化通过（不是实体精度标定）。\n独立FK最大误差="+std::to_string(heldout.positionMax)
            +" mm，姿态="+std::to_string(heldout.angleMax)+" deg\n新目标IK最大误差="+std::to_string(inverseMax)
            +" mm，姿态="+std::to_string(inverseAngle)+" deg\n来源="+reference.source
            +"\n模型、训练/独立验证样本、计算请求/应答、来源指纹已入库。候选仅在本轮关节域与工具/工件/负载条件下有效。\n未改控制器；未替换生产运动模型。";
        result.passed=save(S(result.report));if(!result.passed)return fail("验证完成但数据库保存失败，候选不可用。");
        return result;
    }
    catch(const std::exception& e){return fail(std::string("优化异常，已保留未完成证据：")+e.what());}
}
