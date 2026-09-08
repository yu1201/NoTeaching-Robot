#include "InovanceRobotDriver.h"
#include "RobotKinematicsModel.h"
#include "InovanceWeldCalibration.h"
#include "AppPaths.h"
#include "FtpClient.h"
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <algorithm>
#include <iomanip>
#include <locale>
#include <set>

namespace
{
using Joint = Eigen::Matrix<double,6,1>;
constexpr double pi = 3.14159265358979323846;
constexpr auto recipe = "inovance-v4-calibration-assets-v2";
std::string Hash(const std::string& bytes)
{ return QCryptographicHash::hash(QByteArray::fromStdString(bytes),QCryptographicHash::Sha256).toHex().toStdString(); }
Eigen::Matrix4d WirePose(const std::vector<double>& p,size_t at=0)
{
    Eigen::Matrix4d f=Eigen::Matrix4d::Identity();
    f.topRightCorner<3,1>()=Eigen::Vector3d(p[at],p[at+1],p[at+2]);
    f.topLeftCorner<3,3>()=(Eigen::AngleAxisd(p[at+3]*pi/180.,Eigen::Vector3d::UnitZ())
        *Eigen::AngleAxisd(p[at+4]*pi/180.,Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd(p[at+5]*pi/180.,Eigen::Vector3d::UnitX())).toRotationMatrix();
    return f;
}
Eigen::Matrix4d DH(double a,double alpha,double d,double theta)
{
    const double t=theta*pi/180.,al=alpha*pi/180.;
    Eigen::Matrix4d f;
    f<<cos(t),-sin(t)*cos(al),sin(t)*sin(al),a*cos(t),
       sin(t),cos(t)*cos(al),-cos(t)*sin(al),a*sin(t),
       0,sin(al),cos(al),d,0,0,0,1;
    return f;
}
bool Array(const QJsonObject& o,const char* key,int count,std::vector<double>& v)
{
    const auto a=o.value(QLatin1String(key)).toArray();v.clear();
    if(a.size()<count) return false;
    for(int i=0;i<count;++i) { if(!a[i].isDouble()||!std::isfinite(a[i].toDouble())) return false;v.push_back(a[i].toDouble()); }
    return true;
}
std::string Profile(const RobotKinematicsProfile& p)
{ return std::to_string(p.tool)+","+std::to_string(p.workobject)+","+std::to_string(p.load); }
bool ProfileValid(const RobotKinematicsProfile& p)
{ return p.tool>=0 && p.tool<=15 && p.workobject>=0 && p.workobject<=15 && p.load==0; }
}

bool InovanceRobotCtrl::ReadCalibrationSource(const std::string& path,std::string& bytes,
    std::atomic_bool& cancel,std::string& error)
{
    bytes.clear();
    if(path.empty()||path.front()!='/'||path.find("..")!=std::string::npos
        ||path.find_first_of("\r\n\\")!=std::string::npos)
    { error="只读资产路径无效。";return false; }
    const auto slash=path.find_last_of('/');const auto directory=path.substr(0,slash), name=path.substr(slash+1);
    if(cancel.load()) { error="已取消。";return false; }
    FtpClient ftp(m_pRobotLog,m_ftpIp,m_ftpPort,m_ftpUser,m_ftpPassword);ftp.setMessageBoxesEnabled(false);
    std::vector<FtpRemoteFileInfo> files;
    if(!ftp.listFiles(directory,files,nullptr,512)) { error="FTP目录读取失败："+directory;return false; }
    const auto found=std::find_if(files.begin(),files.end(),[&](const auto& x){return x.name==name&&!x.isDirectory;});
    if(found==files.end()||found->size==0||found->size>1024*1024)
    { error="文件不存在、为空或超过1MiB："+path;return false; }
    const auto root=AppPaths::WritablePath("Temp/CalibrationRead");
    if(!QDir().mkpath(root)) { error="无法创建只读下载暂存目录。";return false; }
    QTemporaryDir temp(root+"/read-XXXXXX");
    if(!temp.isValid()) { error="无法创建独立暂存目录。";return false; }
    const auto local=temp.filePath("source.bin");
    if(!ftp.downloadFileBounded(path,local.toStdString(),found->size,1024*1024,&cancel))
    { error="只读下载失败："+path;return false; }
    QFile file(local);if(!file.open(QIODevice::ReadOnly)) { error="暂存文件读取失败。";return false; }
    const QByteArray data=file.readAll();
    if(static_cast<quint64>(data.size())!=found->size) { error="文件大小变化："+path;return false; }
    bytes=data.toStdString();return true;
}

bool InovanceRobotCtrl::ReadKinematicsReference(const RobotKinematicsProfile& profile,
    RobotKinematicsReference& result,std::string& error)
{
    result={};
    auto fail=[&](const std::string& detail){error=detail.empty()?GetLastRobotError():detail;return false;};
    if(!m_connectionReady.load()) return fail("请先完成连接；标定计算不会自动登录或申请控制许可。");
    if(!ProfileValid(profile)) return fail("汇川计算支持Tool/Wobj 0..15；本模型版本仅支持附加载荷号0（工具自带负载计入来源）。");
    int motion=-1;
    if(!QueryInt("Get_MotionSts",motion)||motion!=0) return fail("控制器未确认停止，拒绝采集计算样本。");
    std::string model,firmware;
    if(!SendCommand("Get_RobotType",model)||!SendCommand("Get_FwVersion",firmware)) return fail("");
    std::atomic_bool cancel{false};std::string bytes;
    if(!ReadCalibrationSource("/RobotParams/MachineParams.json",bytes,cancel,error)) return false;
    const auto root=QJsonDocument::fromJson(QByteArray::fromStdString(bytes)).object();
    const auto body=root["stRobotBody"].toObject();const auto joint=root["stJoint"].toObject();
    const auto install=root["stMotion"].toObject()["stSpace"].toObject()["stInstall"].toObject()["stInstallMode"].toObject();
    if(model!="="+body["cRobotName"].toString().toStdString()||!body["cRobotName"].toString().startsWith("IR-R")
        ||body["RobotType"].toInt()!=6||body["stBase"].toObject()["i32AxisNum"].toInt()!=6)
        return fail("TCP/FTP型号不一致或不是已支持的IR-R六轴机构。");
    std::vector<double> structure,negative,positive;
    if(!Array(body["stKinematics"].toObject(),"dRobotStructureParam",6,structure)
        ||!Array(joint,"dNegLimit",6,negative)||!Array(joint,"dPosLimit",6,positive)) return fail("机械参数/限位不完整。");
    const char* keys[]={"alpha1","alpha2","alpha3","alpha4","alpha5","beta2","d3","d5","a4","a5"};
    std::vector<double> comp;
    for(const auto key:keys)
    { if(!install[key].isDouble()||!std::isfinite(install[key].toDouble())) return fail("缺少长度/角度补偿。");comp.push_back(install[key].toDouble()); }
    for(const auto& item:std::vector<std::pair<std::string,std::vector<double>>>{
        {"Get_StrPara",structure},{"Get_StrParaComp",{comp.begin(),comp.begin()+6}},
        {"Get_SupplementaryStrParamComp",{comp.begin()+6,comp.end()}}})
    {
        std::vector<double> v;
        if(!QueryDoubles(item.first,v,item.second.size())||v.size()!=item.second.size()) return fail("");
        for(size_t i=0;i<v.size();++i) if(std::abs(v[i]-item.second[i])>.002) return fail("TCP/FTP参数不一致："+item.first);
    }
    for(int i=0;i<6;++i)
    {
        std::vector<double> lo,hi;
        if(!QueryDoubles("Get_AxisNLim J"+std::to_string(i+1),lo,1)
            ||!QueryDoubles("Get_AxisPLim J"+std::to_string(i+1),hi,1)) return fail("");
        if(lo.size()!=1||hi.size()!=1||std::abs(lo[0]-negative[i])>.002||std::abs(hi[0]-positive[i])>.002
            ||positive[i]-negative[i]<20) return fail("关节限位不一致或范围不足。");
        result.limits(i,0)=negative[i];result.limits(i,1)=positive[i];
    }
    const double a[]={structure[0],structure[1],structure[2],comp[8],comp[9],0};
    const double d[]={structure[5],0,comp[6],structure[3],comp[7],structure[4]};
    for(int i=0;i<6;++i) result.nominal[i]=DH(a[i],comp[i],d[i],i==1?90:0);
    std::vector<double> tool,work,current;
    if(!QueryDoubles("Get_ToolData "+std::to_string(profile.tool),tool,17)
        ||!QueryDoubles("Get_WobjData "+std::to_string(profile.workobject),work,14)
        ||!QueryDoubles("Get_RobJPHere",current,14)) return fail("");
    if(tool.size()!=17||work.size()!=14||current.size()!=14||tool[0]!=1||work[0]!=0||work[1]!=1)
        return fail("仅支持机器人持工具、固定工件；回包维度/安装方式不符合模型契约。");
    for(int i=6;i<14;++i) if(current[i]!=0) return fail("检测到外部轴，本六轴模型不适用。");
    result.profile=profile;result.tool=WirePose(tool,1);result.work=WirePose(work,2)*WirePose(work,8);
    for(int i=0;i<6;++i) result.current[i]=current[i];
    int activeTool=-1,activeWork=-1;
    if(!QueryInt("Get_ToolCNum",activeTool)||!QueryInt("Get_WobjNum",activeWork)
        ||!QueryInt("Get_MotionSts",motion)||motion!=0) return fail("读取上下文时控制器状态变化。");
    std::ostringstream identity;identity.imbue(std::locale::classic());identity<<std::setprecision(17)
        <<recipe<<'|'<<m_socketIp<<':'<<m_socketPort<<'|'<<model<<'|'<<firmware<<'|'<<Hash(bytes)<<'|'<<Profile(profile)
        <<'|'<<activeTool<<','<<activeWork;
    for(double v:tool) identity<<'|'<<v;for(double v:work) identity<<'|'<<v;
    result.identity=Hash(identity.str());
    result.source=identity.str().substr(0,identity.str().find('|',identity.str().find('|')+1))
        +"; "+model+"; "+firmware+"; FTP:/RobotParams/MachineParams.json sha256="+Hash(bytes)+"; profile="+Profile(profile);
    return true;
}

bool InovanceRobotCtrl::CalculateControllerForward(const RobotKinematicsReference& reference,
    const Joint& joints,RobotKinematicsPoint& result,std::string& error)
{
    result={};
    if(!m_connectionReady.load()||!ProfileValid(reference.profile)||!joints.allFinite())
    { error="连接或计算参数无效。";return false; }
    if((joints.array()<reference.limits.col(0).array()).any()||(joints.array()>reference.limits.col(1).array()).any())
    { error="计算关节超出已核对限位。";return false; }
    std::ostringstream cmd;cmd.imbue(std::locale::classic());cmd<<std::fixed<<std::setprecision(3)<<"Get_RobJToRobP ";
    for(int i=0;i<6;++i) { result.joints[i]=std::round(joints[i]*1000)/1000.;if(i)cmd<<',';cmd<<result.joints[i]; }
    cmd<<",0.000,0.000;0.000,0.000,0.000,0.000,0.000,0.000 "<<Profile(reference.profile);
    std::vector<double> values;
    if(!QueryDoubles(cmd.str(),values,16)||values.size()!=16) { error=GetLastRobotError();return false; }
    for(int i=6;i<10;++i) if(values[i]!=std::round(values[i])) { error="正解构型回包无效。";return false; }
    for(int i=10;i<16;++i) if(values[i]!=0) { error="正解回包存在外部轴。";return false; }
    result.pose=WirePose(values);for(int i=0;i<4;++i)result.configuration[i]=static_cast<int>(values[i+6]);
    result.request=cmd.str();std::ostringstream raw;raw.imbue(std::locale::classic());raw<<std::setprecision(17);
    for(double v:values)raw<<v<<',';result.response=raw.str();
    return RobotKinematicsModel::Rigid(result.pose);
}
bool InovanceRobotCtrl::CalculateControllerInverse(const RobotKinematicsReference& reference,
    const RobotKinematicsPoint& target,RobotKinematicsPoint& result,std::string& error)
{
    result={};
    if(!m_connectionReady.load()||!ProfileValid(reference.profile)||!RobotKinematicsModel::Rigid(target.pose))
    { error="逆解计算上下文/目标无效。";return false; }
    const auto r=target.pose.topLeftCorner<3,3>();
    const double b=atan2(-r(2,0),std::hypot(r(0,0),r(1,0)));
    if(std::abs(cos(b))<1e-6) { error="逆解目标欧拉角奇异，请使用其它验证目标。";return false; }
    const double a=atan2(r(1,0),r(0,0)),c=atan2(r(2,1),r(2,2));
    std::ostringstream cmd;cmd.imbue(std::locale::classic());cmd<<std::fixed<<std::setprecision(3)<<"Get_RobPToRobJ "
        <<target.pose(0,3)<<','<<target.pose(1,3)<<','<<target.pose(2,3)<<','<<a*180/pi<<','<<b*180/pi<<','<<c*180/pi<<';';
    for(int i=0;i<4;++i) { if(i)cmd<<',';cmd<<target.configuration[i]; }
    cmd<<";0.000,0.000,0.000,0.000,0.000,0.000 "<<Profile(reference.profile);
    std::vector<double> values;
    if(!QueryDoubles(cmd.str(),values,14)||values.size()!=14) { error=GetLastRobotError();return false; }
    for(int i=6;i<14;++i) if(values[i]!=0) { error="逆解回包存在外部轴。";return false; }
    for(int i=0;i<6;++i)result.joints[i]=values[i];result.request=cmd.str();
    std::ostringstream raw;raw.imbue(std::locale::classic());raw<<std::setprecision(17);for(double v:values)raw<<v<<',';
    result.response=raw.str();return true;
}

bool InovanceRobotCtrl::DiscoverCalibrationAssets(RobotCalibrationDiscovery& result,
    std::atomic_bool& cancel,std::string& error)
{
    result={};result.recipeRevision=recipe;
    if(!m_connectionReady.load()) { error="请先连接控制器。";return false; }
    std::string model,firmware;
    if(!SendCommand("Get_RobotType",model)||!SendCommand("Get_FwVersion",firmware))
    { error=GetLastRobotError();return false; }
    result.identity=m_socketIp+":"+std::to_string(m_socketPort)+"|"+model+"|"+firmware;
    for(const auto& source:std::vector<std::pair<std::string,std::string>>{
        {"kinematics","/RobotParams/MachineParams.json"},
        {"tool-calibration","/robotUserParameter/ToolCalibData.dat"},
        {"workobject-calibration","/robotUserParameter/WobjCalibData.dat"},
        {"vision","/TecParameter/VisionCraftCaliCfgV22.jsn"}})
    {
        if(cancel.load()) { error="已取消；未修改控制器。";return false; }
        RobotCalibrationAsset asset;asset.kind=source.first;asset.source="FTP:"+source.second;asset.name=source.first;
        std::string why;
        if(!ReadCalibrationSource(source.second,asset.raw,cancel,why))
        { asset.status="missing";asset.detail=why;result.assets.push_back(asset);continue; }
        asset.fingerprint=Hash(asset.raw);asset.status="acquired";
        if(source.first=="vision")
        {
            const auto object=QJsonDocument::fromJson(QByteArray::fromStdString(asset.raw)).object();
            const auto calibrationSlots=object["CoorCfgList"].toArray();int nonzero=0;
            for(const auto& v:calibrationSlots) { const auto m=v.toObject()["CaliMatrix"].toArray();
                if(std::any_of(m.begin(),m.end(),[](const auto& n){return n.isDouble()&&std::abs(n.toDouble())>1e-12;}))++nonzero; }
            asset.status=nonzero?"candidate":"invalid";
            asset.detail="视觉标定槽="+std::to_string(calibrationSlots.size())+"，非零CaliMatrix="+std::to_string(nonzero)
                +"。此厂商文件是9元素视觉映射，不能充当3D手眼R(9)+T(3)；不自动标记已标定。";
            if(model!="="+object["RobotName"].toString().toStdString())
            {asset.status="invalid";asset.detail="视觉文件机器人型号与TCP不一致。";}
        }
        else asset.detail="已读取并保存来源原文/哈希；不覆盖业务参数，不写控制器。";
        result.assets.push_back(asset);
    }
    // Fixed bounded discovery roots, not guessed field names or an arbitrary FTP crawl.
    // User-defined project matrices are evidence candidates until their frame/units are verified.
    FtpClient ftp(m_pRobotLog,m_ftpIp,m_ftpPort,m_ftpUser,m_ftpPassword);ftp.setMessageBoxesEnabled(false);
    std::vector<std::pair<std::string,int>> queue={{"/TeachProgram",0},{"/GlobalVarInfo",0}};
    size_t bytesRead=0;int fileCount=0;
    for(size_t at=0;at<queue.size() && at<128;++at)
    {
        if(cancel.load()) {error="已取消。";return false;}
        std::vector<FtpRemoteFileInfo> files;
        if(!ftp.listFiles(queue[at].first,files,nullptr,512))
        {result.assets.push_back({"search",queue[at].first,"FTP:"+queue[at].first,"","missing","目录读取失败。",""});continue;}
        for(const auto& f:files)
        {
            if(f.name=="."||f.name==".."||f.name.find_first_of("/\\\r\n")!=std::string::npos)continue;
            const auto path=queue[at].first+"/"+f.name;
            if(f.isDirectory) {if(queue[at].second<3&&queue.size()<128)queue.push_back({path,queue[at].second+1});continue;}
            const auto name=QString::fromStdString(f.name).toLower();
            if(!(name.endsWith(".pro")||name.endsWith(".pts")||name.endsWith(".jsn")||name.endsWith(".dat")||name.endsWith(".txt")))continue;
            if(++fileCount>512||f.size==0||f.size>256*1024||bytesRead+f.size>4*1024*1024)continue;
            std::string raw,why;if(!ReadCalibrationSource(path,raw,cancel,why))continue;bytesRead+=raw.size();
            const auto text=QString::fromUtf8(raw.data(),int(raw.size())).toLower();
            if(text.contains("eye")||text.contains("matrix")||text.contains("camera")||text.contains("矩阵")||text.contains("相机"))
                result.assets.push_back({"handeye-candidate",f.name,"FTP:"+path,Hash(raw),"candidate",
                    "匹配到标定相关名称；需核对变量、相机归属、R/T维度、单位和坐标方向，禁止按名称自动启用。",raw});
        }
    }
    for(int i=0;i<16&&!cancel.load();++i)
    {
        std::vector<double> values;RobotCalibrationAsset asset;asset.kind="tool";asset.name="Tool"+std::to_string(i);
        asset.source="TCP:Get_ToolData "+std::to_string(i);
        if(QueryDoubles("Get_ToolData "+std::to_string(i),values,17)&&values.size()==17)
        {std::ostringstream raw;raw.imbue(std::locale::classic());raw<<std::setprecision(17);for(double v:values)raw<<v<<',';
            asset.raw=raw.str();asset.fingerprint=Hash(asset.raw);asset.status="acquired";asset.detail="RobHold+TFrame(mm/deg)+负载；读取成功不等同于已指定相机工具。";}
        else {asset.status="missing";asset.detail=GetLastRobotError();}
        result.assets.push_back(asset);
    }
    if(cancel.load()) {error="已取消。";return false;}
    RobotControllerHandEye laser;std::string laserError;
    if(ReadControllerHandEye(0,laser,laserError))
        result.assets.push_back({"handeye","激光传感器0",laser.source,laser.sourceFingerprint,"acquired",
            "已验证camera→Tool"+std::to_string(laser.toolIndex)+"；相机="+laser.cameraAddress+"；可在本页核对后导入。",laser.evidence});
    else result.assets.push_back({"handeye","激光传感器0","FTP:/RCFamily/Controller/Teachology/InoRobPluginWeld/config.xml","","missing",laserError,""});
    return true;
}

bool InovanceRobotCtrl::ReadControllerHandEye(int sensor,RobotControllerHandEye& result,std::string& error)
{
    result={};if(!m_connectionReady.load()){error="控制器未连接。";return false;}
    std::atomic_bool cancel{false};std::string bytes;
    if(!ReadCalibrationSource("/RCFamily/Controller/Teachology/InoRobPluginWeld/config.xml",bytes,cancel,error))return false;
    if(!InovanceWeldCalibration::Parse(QByteArray::fromStdString(bytes),sensor,result,error))return false;
    std::string model,fw;std::vector<double> tool;
    if(!SendCommand("Get_RobotType",model)||!SendCommand("Get_FwVersion",fw)
        ||!QueryDoubles("Get_ToolData "+std::to_string(result.toolIndex),tool,17)||tool.size()!=17||tool[0]!=1)
    {error="无法核对控制器身份或绑定工具（必须为机器人持工具）。";return false;}
    std::ostringstream fingerprint;fingerprint.imbue(std::locale::classic());fingerprint<<std::setprecision(17);
    for(double v:tool)fingerprint<<v<<',';result.toolFingerprint=Hash(fingerprint.str());
    result.controllerIdentity=m_socketIp+":"+std::to_string(m_socketPort)+"|"+model+"|"+fw;
    return true;
}
bool InovanceRobotCtrl::ValidateControllerHandEyeContext(const RobotControllerHandEye& expected,std::string& error)
{
    RobotControllerHandEye actual;
    if(!ReadControllerHandEye(expected.sensorIndex,actual,error))return false;
    int tool=-1;
    if(!QueryInt("Get_ToolCNum",tool)) {error=GetLastRobotError();return false;}
    if(actual.sourceFingerprint!=expected.sourceFingerprint||actual.controllerIdentity!=expected.controllerIdentity
        ||actual.toolFingerprint!=expected.toolFingerprint||actual.toolIndex!=expected.toolIndex||tool!=expected.toolIndex
        ||actual.cameraAddress!=expected.cameraAddress
        ||(actual.cameraToTool-expected.cameraToTool).cwiseAbs().maxCoeff()>1e-9)
    {error="控制器、传感器标定、工具数据或活动工具号变化；重新获取并验证矩阵，禁止复用旧绑定。";return false;}
    return true;
}
