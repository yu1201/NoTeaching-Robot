#include "RobotKinematicsModel.h"
#include <cmath>
#include <algorithm>

namespace RobotKinematicsModel
{
namespace
{
constexpr double pi = 3.14159265358979323846;
Eigen::Matrix4d Rz(double degrees)
{
    Eigen::Matrix4d r = Eigen::Matrix4d::Identity();
    r.topLeftCorner<3,3>() = Eigen::AngleAxisd(degrees*pi/180., Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return r;
}
Eigen::Matrix4d Correction(const Eigen::VectorXd& p, int index)
{
    Eigen::Matrix4d c = Eigen::Matrix4d::Identity();
    c.topRightCorner<3,1>() = p.segment<3>(index*6);
    Eigen::Vector3d v = p.segment<3>(index*6+3)/1000.;
    if (v.norm()>1e-15) c.topLeftCorner<3,3>() = Eigen::AngleAxisd(v.norm(),v.normalized()).toRotationMatrix();
    return c;
}
Eigen::Matrix4d GeometryForward(const Model& m, const Joint& q, const Eigen::VectorXd& p)
{
    Eigen::Matrix4d f = Correction(p,0);
    for (int i=0;i<6;++i) f = (f*Rz(q[i])*m.nominal[i]*Correction(p,i+1)).eval();
    return f;
}
std::array<Eigen::VectorXd,6> Features(const Model& m, const Joint& q)
{
    std::array<Eigen::Vector3d,6> origins, axes;
    std::array<Eigen::Matrix4d,6> frames;
    Eigen::Matrix4d f=Correction(m.featureGeometry,0);
    for(int i=0;i<6;++i)
    {
        origins[i]=f.topRightCorner<3,1>()/1000.; axes[i]=f.block<3,1>(0,2);
        f=(f*Rz(q[i])*m.nominal[i]*Correction(m.featureGeometry,i+1)).eval(); frames[i]=f;
    }
    const Eigen::Vector3d gravity(0,0,-1);
    std::array<Eigen::VectorXd,6> values;
    for(int j=0;j<6;++j)
    {
        values[j].resize((6-j)*4); int at=0;
        for(int i=j;i<6;++i)
        {
            Eigen::Vector3d lever=frames[i].topRightCorner<3,1>()/1000.-origins[j];
            values[j][at++]=axes[j].dot(lever.cross(gravity));
            for(int k=0;k<3;++k) values[j][at++]=axes[j].dot(frames[i].block<3,1>(0,k).cross(gravity));
        }
    }
    return values;
}
Eigen::Matrix<double,6,1> Residual(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    Eigen::Matrix<double,6,1> r;
    r.head<3>()=a.topRightCorner<3,1>()-b.topRightCorner<3,1>();
    Eigen::AngleAxisd angle(a.topLeftCorner<3,3>()*b.topLeftCorner<3,3>().transpose());
    r.tail<3>()=angle.axis()*angle.angle()*500.;
    return r;
}
bool Solve(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& residual,
    Eigen::VectorXd& p, int count, std::atomic_bool& cancel, const RobotCalibrationProgress& progress)
{
    double damping=.01; const double ridge=1e-8;
    Eigen::VectorXd r=residual(p);
    for(int iteration=0;iteration<40;++iteration)
    {
        if(cancel.load()) return false;
        if(progress) progress("拟合迭代 "+std::to_string(iteration+1)+"/40（只计算，不运动）");
        Eigen::MatrixXd jac(r.size(),p.size());
        for(int k=0;k<p.size();++k)
        {
            if(cancel.load()) return false;
            Eigen::VectorXd pp=p; pp[k]+=.001; jac.col(k)=(residual(pp)-r)/.001;
        }
        Eigen::MatrixXd h=jac.transpose()*jac/count;
        Eigen::VectorXd g=jac.transpose()*r/count+ridge*p;
        const double old=r.squaredNorm()/count+ridge*p.squaredNorm();
        bool accepted=false; double cost=old; Eigen::VectorXd delta;
        for(int attempt=0;attempt<10;++attempt)
        {
            Eigen::MatrixXd hd=h; hd.diagonal().array()+=damping+ridge;
            delta=hd.ldlt().solve(-g);
            if(!delta.allFinite()) return false;
            Eigen::VectorXd pp=(p+delta).cwiseMax(-20).cwiseMin(20), rr=residual(pp);
            cost=rr.squaredNorm()/count+ridge*pp.squaredNorm();
            if(cost<old) { p=pp;r=rr;damping=std::max(damping/3.,1e-10);accepted=true;break; }
            damping*=10;
        }
        if(!accepted || delta.norm()<1e-7 || old-cost<1e-13) break;
    }
    return p.allFinite() && p.cwiseAbs().maxCoeff()<19.999;
}
}
bool Rigid(const Eigen::Matrix4d& m)
{
    return m.allFinite() && (m.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-8
        && (m.topLeftCorner<3,3>().transpose()*m.topLeftCorner<3,3>()-Eigen::Matrix3d::Identity()).norm()<1e-5
        && std::abs(m.topLeftCorner<3,3>().determinant()-1)<1e-5;
}
Eigen::Matrix4d Forward(const Model& m, const Joint& q)
{
    Joint adjusted=q;
    const auto values=Features(m,q);
    for(int j=0;j<6;++j)
        if(m.compliance[j].size()) adjusted[j]+=values[j].dot(m.mappings[j]*m.compliance[j])/1000.*180./pi;
    return GeometryForward(m,adjusted,m.geometry);
}
bool Inverse(const Model& m, const Eigen::Matrix4d& target, Joint& q,
    const Eigen::Matrix<double,6,2>& limits)
{
    if(!Rigid(target)||!q.allFinite()) return false;
    for(int it=0;it<60;++it)
    {
        const auto r=Residual(Forward(m,q),target);
        if(r.head<3>().norm()<1e-5 && r.tail<3>().norm()<1e-5) return true;
        Eigen::Matrix<double,6,6> jac;
        for(int j=0;j<6;++j) { Joint qp=q;qp[j]+=.0001;jac.col(j)=(Residual(Forward(m,qp),target)-r)/.0001; }
        Eigen::Matrix<double,6,6> h=jac.transpose()*jac;h.diagonal().array()+=1e-6;
        Joint delta=h.ldlt().solve(-jac.transpose()*r);
        if(!delta.allFinite()) return false;
        if(delta.cwiseAbs().maxCoeff()>5) delta*=5/delta.cwiseAbs().maxCoeff();
        q=(q+delta).cwiseMax(limits.col(0)).cwiseMin(limits.col(1));
    }
    return false;
}
bool Fit(const std::vector<Sample>& samples, Model& m, std::atomic_bool& cancel,
    const RobotCalibrationProgress& progress, std::string& error)
{
    if(samples.size()<80) { error="训练样本不足80组。";return false; }
    for(const auto& s:samples) if(!s.q.allFinite()||!Rigid(s.flange)) { error="训练数据无效。";return false; }
    for(const auto& link:m.nominal) if(!Rigid(link)) { error="初始6R模型无效。";return false; }
    auto residual=[&](const Eigen::VectorXd& p)->Eigen::VectorXd {
        Eigen::VectorXd r(samples.size()*6);
        for(size_t i=0;i<samples.size();++i) r.segment<6>(i*6)=Residual(GeometryForward(m,samples[i].q,p),samples[i].flange);
        return r;
    };
    m.geometry=Eigen::VectorXd::Zero(42);
    if(!Solve(residual,m.geometry,int(samples.size()),cancel,progress)) { error="刚性拟合取消、数值失败或达到修正边界。";return false; }
    m.featureGeometry=m.geometry;
    std::array<Eigen::MatrixXd,6> feature;
    for(int j=0;j<6;++j) feature[j].resize(samples.size(),(6-j)*4);
    for(size_t i=0;i<samples.size();++i)
    { const auto values=Features(m,samples[i].q);for(int j=0;j<6;++j) feature[j].row(i)=values[j].transpose(); }
    int size=42;
    for(int j=0;j<6;++j)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(feature[j],Eigen::ComputeThinV);
        const auto s=svd.singularValues();int rank=0;
        while(rank<s.size() && s[rank]>1e-6 && s[rank]>s[0]*1e-4) ++rank;
        m.mappings[j]=svd.matrixV().leftCols(rank);
        for(int k=0;k<rank;++k) m.mappings[j].col(k)*=std::sqrt(double(samples.size()))/s[k];
        feature[j]=(feature[j]*m.mappings[j]).eval();
        m.compliance[j]=Eigen::VectorXd::Zero(rank);size+=rank;
    }
    Eigen::VectorXd p=Eigen::VectorXd::Zero(size);p.head(42)=m.geometry;
    auto flexible=[&](const Eigen::VectorXd& x)->Eigen::VectorXd {
        Eigen::MatrixXd offsets=Eigen::MatrixXd::Zero(samples.size(),6);int at=42;
        for(int j=0;j<6;++j) { int n=int(m.compliance[j].size());offsets.col(j)=feature[j]*x.segment(at,n);at+=n; }
        Eigen::VectorXd r(samples.size()*6);const Eigen::VectorXd geometry=x.head(42);
        for(size_t i=0;i<samples.size();++i)
        { Joint q=samples[i].q+offsets.row(i).transpose()/1000.*180./pi;r.segment<6>(i*6)=Residual(GeometryForward(m,q,geometry),samples[i].flange); }
        return r;
    };
    if(!Solve(flexible,p,int(samples.size()),cancel,progress)) { error="柔顺拟合取消、数值失败或达到修正边界。";return false; }
    m.geometry=p.head(42);int at=42;
    for(int j=0;j<6;++j) { int n=int(m.compliance[j].size());m.compliance[j]=p.segment(at,n);at+=n; }
    return true;
}
Metrics Evaluate(const Model& model,const std::vector<Sample>& samples)
{
    Metrics m;
    for(const auto& s:samples)
    {
        const auto r=Residual(Forward(model,s.q),s.flange);
        m.positionMax=std::max(m.positionMax,r.head<3>().norm());m.positionRms+=r.head<3>().squaredNorm();
        m.angleMax=std::max(m.angleMax,r.tail<3>().norm()/500.*180./pi);
    }
    if(!samples.empty()) m.positionRms=std::sqrt(m.positionRms/samples.size());
    return m;
}
}
