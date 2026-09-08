#pragma once
#include "RobotCalibrationTypes.h"
#include "RobotKinematicsModel.h"
#include <QCryptographicHash>
#include <QRegularExpression>
#include <QXmlStreamReader>
#include <QStringList>

// Firmware plugin schema from its shipped typedef.h: LaserHeadInform (4),
// 6 RobPos (16 each), 5 laser points (3 each), coord/error/compensation (6 each), IP/port.
// Never parse /Plugins/.../config.xml (factory defaults) as installed calibration.
namespace InovanceWeldCalibration
{
inline Eigen::Matrix4d Pose(const std::vector<double>& v,int at)
{
    constexpr double scale=3.14159265358979323846/180.;
    Eigen::Matrix4d f=Eigen::Matrix4d::Identity();
    f.topRightCorner<3,1>()=Eigen::Vector3d(v[at],v[at+1],v[at+2]);
    f.topLeftCorner<3,3>()=(Eigen::AngleAxisd(v[at+3]*scale,Eigen::Vector3d::UnitZ())
        *Eigen::AngleAxisd(v[at+4]*scale,Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd(v[at+5]*scale,Eigen::Vector3d::UnitX())).toRotationMatrix();
    return f;
}
inline bool Parse(const QByteArray& bytes,int sensor,RobotControllerHandEye& result,std::string& error)
{
    result={};
    if(sensor<0||sensor>15||bytes.isEmpty()||bytes.size()>1024*1024){error="传感器编号/文件大小无效。";return false;}
    // Vendor file contains several top-level XML elements. Isolate exactly one
    // named Struct then use an XML parser, without executing entities/DTD.
    const auto text=QString::fromUtf8(bytes);
    if(text.contains("<!DOCTYPE",Qt::CaseInsensitive)||text.contains("<!ENTITY",Qt::CaseInsensitive))
    {error="拒绝带DTD/实体的标定配置。";return false;}
    const QRegularExpression re("<Struct\\s+name=\"WELD_LaserConfig\"[^>]*>.*?</Struct>",QRegularExpression::DotMatchesEverythingOption);
    auto matches=re.globalMatch(text);
    if(!matches.hasNext()){error="未找到WELD_LaserConfig。";return false;}
    const auto fragment=matches.next().captured();if(matches.hasNext()){error="标定结构重复。";return false;}
    const auto wanted=QString("E%1").arg(sensor,2,10,QLatin1Char('0'));
    QXmlStreamReader xml(fragment);QString payload;int count=0;
    while(!xml.atEnd())
    {
        xml.readNext();if(!xml.isStartElement()||xml.name()!=QLatin1String("Variable"))continue;
        QString name,type,value;
        while(xml.readNextStartElement())
        {
            if(xml.name()==QLatin1String("name"))name=xml.readElementText();
            else if(xml.name()==QLatin1String("type"))type=xml.readElementText();
            else if(xml.name()==QLatin1String("value"))value=xml.readElementText();
            else xml.skipCurrentElement();
        }
        if(name==wanted){++count;if(type!="S_LASERCONFIG"){error="传感器结构类型错误。";return false;}payload=value;}
    }
    if(xml.hasError()||count!=1){error="标定XML解析失败或传感器记录不唯一。";return false;}
    const auto fields=payload.split(',');if(fields.size()!=135){error="激光标定结构字段数变化，拒绝猜测偏移。";return false;}
    std::vector<double> numbers(133,0);
    for(int i=1;i<133;++i){bool ok=false;numbers[i]=fields[i].toDouble(&ok);if(!ok||!std::isfinite(numbers[i])){error="激光标定字段不是有限数值。";return false;}}
    if(numbers[2]!=sensor||numbers[3]!=std::round(numbers[3])||numbers[3]<0||numbers[3]>15)
    {error="传感器编号或绑定工具号无效。";return false;}
    for(int i=127;i<133;++i)if(std::abs(numbers[i])>1e-9){error="存在非零激光补偿参数，尚未确认其合成顺序，暂不导入。";return false;}
    for(int i=0;i<6;++i)if(numbers[121+i]<0||!std::isfinite(numbers[121+i])){error="标定误差无效。";return false;}
    result.cameraToTool=Pose(numbers,115);result.sensorIndex=sensor;result.toolIndex=int(numbers[3]);
    result.cameraAddress=fields[133].trimmed().toStdString();
    if(result.cameraAddress.empty()||result.cameraAddress=="0.0.0.0"){error="标定缺少实际相机地址。";return false;}
    Eigen::Matrix<double,3,5> laser;
    const Eigen::Vector3d reference(numbers[4],numbers[5],numbers[6]);
    for(int i=0;i<5;++i)
    {
        laser.col(i)=Eigen::Vector3d(numbers[100+i*3],numbers[101+i*3],numbers[102+i*3]);
        const auto f=Pose(numbers,20+i*16)*result.cameraToTool;
        const Eigen::Vector3d actual=f.topLeftCorner<3,3>()*laser.col(i)+f.topRightCorner<3,1>();
        result.sampleResidualMaxMm=std::max(result.sampleResidualMaxMm,(actual-reference).norm());
    }
    const Eigen::Matrix<double,3,5> centred=laser.colwise()-laser.rowwise().mean();
    Eigen::JacobiSVD<Eigen::Matrix<double,3,5>> svd(centred);
    if(svd.singularValues()[1]<1 || result.sampleResidualMaxMm>2.0)
    {error="五点标定样本无效/退化，或相机到绑定工具的样本闭环大于2mm。";return false;}
    for(int i=0;i<6;++i)result.reportedError[i]=numbers[121+i];
    result.source="FTP:/RCFamily/Controller/Teachology/InoRobPluginWeld/config.xml#WELD_LaserConfig/"+wanted.toStdString();
    result.sourceFingerprint=QCryptographicHash::hash(payload.toUtf8(),QCryptographicHash::Sha256).toHex().toStdString();
    result.evidence="S_LASERCONFIG/135字段；标定结果XYZABC(mm/deg)="+fields.mid(115,6).join(',').toStdString()
        +"；camera→绑定Tool的TCP；五点闭环最大="
        +std::to_string(result.sampleResidualMaxMm)+"mm；控制器报告误差="+fields.mid(121,6).join(',').toStdString()
        +"；原始标定数据="+payload.toStdString();
    return true;
}
}
