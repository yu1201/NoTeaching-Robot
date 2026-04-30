#include "SkDataClass.h"
#include "SkFunction.h"
#include "math.h"

SkMatrix4d::SkMatrix4d()
{
    mT = Eigen::Matrix4d::Identity();
}

SkMatrix4d::SkMatrix4d(const SkMatrix4d &mTrans)
{
    mT = mTrans.mT;
}

void SkMatrix4d::init()
{
    mT = Eigen::Matrix4d::Identity();
}

int SkMatrix4d::isZeroT()
{
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            if(abs(mT(i, j)) > EPS)
                return TRUE;
        }
    }
    return FALSE;
}

int SkMatrix4d::isOneT()
{
    Eigen::Isometry3d temp = Eigen::Isometry3d::Identity();
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            if(abs(mT(i, j) - temp(i, j)) > EPS)
            {
                return FALSE;
            }
        }
    }
    return TRUE;
}

SkMatrix4d SkMatrix4d::unit()
{
    SkMatrix4d unitM;
    unitM.mT = Eigen::Matrix4d::Identity();
    return unitM;
}

SkMatrix4d SkMatrix4d::inv()
{
    SkMatrix4d tInv;
    tInv.nx = (oy *az - oz *ay) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.ny = -(ny *az - nz *ay) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.nz = (ny *oz - nz *oy) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.ox = -(ox *az - oz *ax) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.oy = (nx *az - nz *ax) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.oz = -(nx *oz - nz *ox) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.ax = (ox *ay - oy *ax) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.ay = -(nx *ay - ny *ax) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.az = (nx *oy - ny *ox) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.px = -(ox *ay *pz - ox *py *az - oy *ax *pz + oy *px *az + oz *ax *py - oz *px *ay) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.py = (nx *ay *pz - nx *py *az - ny *ax *pz + ny *px *az + nz *ax *py - nz *px *ay) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.pz = -(nx *oy *pz - nx *py *oz - ny *ox *pz + ny *px *oz + nz *ox *py - nz *px *oy) / (nx *oy *az - nx *oz *ay - ny *ox *az + ny *oz *ax + nz *ox *ay - nz *oy *ax);
    tInv.n3 = 0;
    tInv.o3 = 0;
    tInv.a3 = 0;
    tInv.p3 = 1;
    return tInv;
}

void SkMatrix4d::print()
{
    qDebug() << "[" << nx << " " << ox << " " << ax << " " << px << \
             ny << " " << oy << " " << ay << " " << py << \
             nz << " " << oz << " " << az << " " << pz << \
             n3 << " " << o3 << " " << a3 << " " << p3 << "]";
}

SkMatrix4d SkMatrix4d::operator*(SkMatrix4d T)
{
    SkMatrix4d tOut;
    tOut.mT = mT *T.mT;
    return tOut;
}

SkMatrix4d SkMatrix4d::operator+(SkVector6d jA)
{
    SkMatrix4d tOut;
    tOut.mT = mT;
    for(int i = 0; i < 3; i++)
    {
        tOut.mT(i, 3) = mT(i, 3) + jA.v[i];
    }
    return tOut;
}

SkMatrix4d SkMatrix4d::operator-(SkVector6d jA)
{
    SkMatrix4d tOut;
    tOut.mT = mT;
    for(int i = 0; i < 3; i++)
    {
        tOut.mT(i, 3) = mT(i, 3) - jA.v[i];
    }
    return tOut;
}

SkVector6d SkMatrix4d::operator*(SkVector6d jA)
{
    SkVector6d jOut;
    for(int i = 0; i < 3; i++)
    {
        jOut.v[i]  = mT(i, 0) * jA.v[0] + mT(i, 1) * jA.v[1] + mT(i, 2) * jA.v[2] + mT(i, 3);
        jOut.v[i + 3] = mT(i, 0) * jA.v[3] + mT(i, 1) * jA.v[4] + mT(i, 2) * jA.v[5];
    }
    return jOut;
}

SkVector9d SkMatrix4d::operator*(SkVector9d jA)
{
    SkVector9d jOut;
    for(int i = 0; i < 3; i++)
    {
        jOut.v[i]  = mT(i, 0) * jA.v[0] + mT(i, 1) * jA.v[1] + mT(i, 2) * jA.v[2] + mT(i, 3);
        jOut.v[i + 3] = mT(i, 0) * jA.v[3] + mT(i, 1) * jA.v[4] + mT(i, 2) * jA.v[5];
        jOut.v[i + 6] = mT(i, 0) * jA.v[6] + mT(i, 1) * jA.v[7] + mT(i, 2) * jA.v[8] + mT(i, 3);
    }
    return jOut;
}

SkVector3d SkMatrix4d::operator*(SkVector3d vA)
{
    SkVector3d vOut;
    vOut.mV = mT.block<3, 3>(0, 0) * vA.mV;
    return vOut;
}

SkPoint3D  SkMatrix4d::operator*(SkPoint3D pA)
{
    SkPoint3D pOut;
    pOut.x = mT(0, 0) * pA.x + mT(0, 1) * pA.y + mT(0, 2) * pA.z + mT(0, 3);
    pOut.y = mT(1, 0) * pA.x + mT(1, 1) * pA.y + mT(1, 2) * pA.z + mT(1, 3);
    pOut.z = mT(2, 0) * pA.x + mT(2, 1) * pA.y + mT(2, 2) * pA.z + mT(2, 3);
    return pOut;
}

SkMatrix4d SkMatrix4d::operator=(SkMatrix4d T)
{
    mT = T.mT;
    return *this;
}

SkVector6d::SkVector6d()
{
    x = 0;
    y = 0;
    z = 0;
    a = 0;
    b = 0;
    c = 0;
}

SkVector6d::SkVector6d(const SkVector6d &mJoint)
{
    for(int i = 0; i < 6; i++)
    {
        v[i] = mJoint.v[i];
    }
}

void SkVector6d::init()
{
    x = 0;
    y = 0;
    z = 0;
    a = 0;
    b = 0;
    c = 0;
}

int SkVector6d::isZeroJ()
{
    for(int i = 0; i < 6; i++)
    {
        if(abs(v[i]) > EPS)
            return FALSE;
    }
    return TRUE;
}

void SkVector6d::print()
{
    qDebug() << "[" << x << " " << y << " " << z << " " << a << " " << b << " " << c << "]";
}

SkVector6d SkVector6d::operator+(SkVector6d jA)
{
    SkVector6d jOut;
    for(int i = 0; i < 6; i++)
    {
        jOut.v[i] = v[i] + jA.v[i];
    }
    return jOut;
}

SkVector6d SkVector6d::operator-(SkVector6d jA)
{
    SkVector6d jOut;
    for(int i = 0; i < 6; i++)
    {
        jOut.v[i] = v[i] - jA.v[i];
    }
    return jOut;
}

SkVector6d SkVector6d::operator*(SkVector6d jA)
{
    SkVector6d jOut;
    SkMatrix4d ownT = joint2trans(*this);
    SkMatrix4d tA = joint2trans(jA);
    SkMatrix4d tOut = ownT *tA;
    jOut = trans2joint(tOut);
    return jOut;
}

SkVector6d SkVector6d::operator%(SkVector6d jA)
{
    SkVector6d jOut = *this;
    jOut.x = x *jA.x;
    jOut.y = y *jA.y;
    jOut.z = z *jA.z;
    return jOut;
}

SkVector6d SkVector6d::operator*(int n)
{
    SkVector6d jOut;
    for(int i = 0; i < 6; i++)
    {
        jOut.v[i] = v[i] * n;
    }
    return jOut;
}

SkVector6d SkVector6d::operator*(double k)
{
    SkVector6d jOut;
    for(int i = 0; i < 6; i++)
    {
        jOut.v[i] = v[i] * k;
    }
    return jOut;
}

SkVector6d SkVector6d::operator/(int n)
{
    SkVector6d jOut;
    for(int i = 0; i < 6; i++)
    {
        jOut.v[i] = v[i] / n;
    }
    return jOut;
}

SkVector6d SkVector6d::operator/(double k)
{
    SkVector6d jOut;
    for(int i = 0; i < 6; i++)
    {
        jOut.v[i] = v[i] / k;
    }
    return jOut;
}

SkVector6d SkVector6d::operator=(SkVector6d jA)
{
    for(int i = 0; i < 6; i++)
    {
        v[i] = jA.v[i];
    }
    return *this;
}

SkVector3d::SkVector3d()
{
    mV = Eigen::Vector3d(0, 0, 0);
}

SkVector3d::SkVector3d(const SkVector3d &mVector)
{
    mV = mVector.mV;
}

void SkVector3d::init()
{
    mV = Eigen::Vector3d(0, 0, 0);
}

int SkVector3d::isZeroV()
{
    for(int i = 0; i < 3; i++)
    {
        if(abs(v[i]) > EPS)
            return FALSE;
    }
    return TRUE;
}

SkVector3d SkVector3d::unit()
{
    SkVector3d vOut;
    vOut.mV /= mV.norm();
    return vOut;
}

void SkVector3d::print()
{
    qDebug() << "[" << x << " " << y << " " << z << "]";
}

SkVector3d SkVector3d::operator+(SkVector3d vA)
{
    SkVector3d vOut;
    vOut.mV = mV + vA.mV;
    return vOut;
}

SkVector3d SkVector3d::operator-(SkVector3d vA)
{
    SkVector3d vOut;
    vOut.mV = mV - vA.mV;
    return vOut;
}

//vP * vQ
double SkVector3d::operator*(SkVector3d vA)
{
    double dOut = 0;
    dOut = mV.dot(vA.mV);
    return dOut;
}

//vP X vQ
SkVector3d SkVector3d::operator%(SkVector3d vA)
{
    SkVector3d vOut;
    vOut.mV = mV.cross(vA.mV);
    return vOut;
}

SkVector3d SkVector3d::operator*(double k)
{
    SkVector3d vOut;
    vOut.mV = mV *k;
    return vOut;
}

SkVector3d SkVector3d::operator*(int n)
{
    SkVector3d vOut;
    vOut.mV = mV *n;
    return vOut;
}

SkVector3d SkVector3d::operator/(double k)
{
    SkVector3d vOut;
    vOut.mV = mV / k;
    return vOut;
}

SkVector3d SkVector3d::operator/(int n)
{
    SkVector3d vOut;
    vOut.mV = mV / n;
    return vOut;
}

SkVector3d SkVector3d::operator=(SkVector3d vA)
{
    mV = vA.mV;
    return *this;
}

SkVector9d::SkVector9d()
{
    x = 0;
    y = 0;
    z = 0;
    a = 0;
    b = 0;
    c = 0;
    e1 = 0;
    e2 = 0;
    e3 = 0;
}

SkVector9d::SkVector9d(const SkVector9d &mExtendJoint)
{
    for(int i = 0; i < 9; i++)
    {
        v[i] = mExtendJoint.v[i];
    }
}

void SkVector9d::init()
{
    x = 0;
    y = 0;
    z = 0;
    a = 0;
    b = 0;
    c = 0;
    e1 = 0;
    e2 = 0;
    e3 = 0;
}

int SkVector9d::isZeroJ()
{
    for(int i = 0; i < 9; i++)
    {
        if(abs(v[i]) > EPS)
            return FALSE;
    }
    return TRUE;
}

void SkVector9d::print()
{
    qDebug() << "[" << x << " " << y << " " << z << " " << a << " " << b << " " << c << "," << e1 << " " << e2 << " " << e3 << "]";
}

SkVector9d SkVector9d::operator+(SkVector9d jA)
{
    SkVector9d jOut = jA;
    for(int i = 0; i < 9; i++)
    {
        jOut.v[i] = v[i] + jA.v[i];
    }
    return jOut;
}

SkVector9d SkVector9d::operator-(SkVector9d jA)
{
    SkVector9d jOut = jA;
    for(int i = 0; i < 9; i++)
    {
        jOut.v[i] = v[i] - jA.v[i];
    }
    return jOut;
}

SkVector9d SkVector9d::operator*(SkVector9d jA)
{
    SkVector9d jOut = jA;
    SkMatrix4d ownT = extendJ2trans(*this);
    SkMatrix4d tA = extendJ2trans(jA);
    SkMatrix4d tOut = ownT *tA;
    jOut = trans2extendJ(tOut);
    return jOut;
}

SkVector9d SkVector9d::operator%(SkVector9d jA)
{
    SkVector9d jOut = *this;
    jOut.x = x *jA.x;
    jOut.y = y *jA.y;
    jOut.z = z *jA.z;
    return jOut;
}

SkVector9d SkVector9d::operator/(int n)
{
    SkVector9d jOut;
    for(int i = 0; i < 9; i++)
    {
        jOut.v[i] = v[i] / n;
    }
    return jOut;
}

SkVector9d SkVector9d::operator/(double k)
{
    SkVector9d jOut;
    for(int i = 0; i < 9; i++)
    {
        jOut.v[i] = v[i] / k;
    }
    return jOut;
}

SkVector9d SkVector9d::operator=(SkVector9d jA)
{
    for(int i = 0; i < 9; i++)
    {
        v[i] = jA.v[i];
    }
    return *this;
}

SkPoint3D::SkPoint3D()
{
    for(int i = 0; i < 3; i++)
    {
        p[i] = 0;
    }
    order = 0;
    dis = 0;
    angle = 0;
}

SkPoint3D::SkPoint3D(const SkPoint3D &pA)
{
    for(int i = 0; i < 3; i++)
    {
        p[i] = pA.p[i];
    }
    order = pA.order;
    dis = pA.dis;
    angle = pA.angle;
}

SkPoint3D::SkPoint3D(double xA, double yA, double zA)
{
    x = xA;
    y = yA;
    z = zA;
    order = 0;
    dis = 0;
    angle = 0;
}

SkPoint3D::SkPoint3D(double xA, double yA, double zA, int orderA)
{
    x = xA;
    y = yA;
    z = zA;
    order = orderA;
    dis = 0;
    angle = 0;
}

SkPoint3D::SkPoint3D(double xA, double yA, double zA, int orderA, double disA, double angleA)
{
    x = xA;
    y = yA;
    z = zA;
    order = orderA;
    dis = disA;
    angle = angleA;
}

void SkPoint3D::init()
{
    for(int i = 0; i < 3; i++)
    {
        p[i] = 0;
    }
    dis = 0;
    angle = 0;
    order = 0;
}

int SkPoint3D::isZero()
{
    for(int i = 0; i < 3; i++)
    {
        if(abs(p[i]) > EPS)
            return FALSE;
    }
    if(abs(dis) > EPS)
        return FALSE;
    if(abs(angle) > EPS)
        return FALSE;
    if(abs(order) > EPS)
        return FALSE;
    return TRUE;
}

void SkPoint3D::print()
{
    qDebug() << "[" << x << " " << y << " " << z << "," << dis << " " << angle << " " << order << "]";
}

SkPoint3D SkPoint3D::operator+(SkPoint3D pA)
{
    SkPoint3D pOut;
    for(int i = 0; i < 3; i++)
    {
        pOut.p[i] = p[i] + pA.p[i];
    }
    pOut.dis = dis + pA.dis;
    pOut.angle = angle + pA.angle;
    pOut.order = order + pA.order;
    return pOut;
}

SkPoint3D SkPoint3D::operator-(SkPoint3D pA)
{
    SkPoint3D pOut = pA;
    for(int i = 0; i < 3; i++)
    {
        pOut.p[i] = p[i] - pA.p[i];
    }
    pOut.dis = dis - pA.dis;
    pOut.angle = angle - pA.angle;
    pOut.order = order - pA.order;
    return pOut;
}

SkPoint3D SkPoint3D::operator*(int n)
{
    SkPoint3D pOut;
    for(int i = 0; i < 3; i++)
    {
        pOut.p[i] = p[i] * n;
    }
    pOut.dis = dis *n;
    pOut.angle = angle *n;
    pOut.order = order *n;
    return pOut;
}

SkPoint3D SkPoint3D::operator*(double k)
{
    SkPoint3D pOut;
    for(int i = 0; i < 3; i++)
    {
        pOut.p[i] = p[i] * k;
    }
    pOut.dis = dis *k;
    pOut.angle = angle *k;
    pOut.order = order *k;
    return pOut;
}

SkPoint3D SkPoint3D::operator/(int n)
{
    SkPoint3D pOut;
    for(int i = 0; i < 3; i++)
    {
        pOut.p[i] = p[i] / n;
    }
    pOut.dis = dis / n;
    pOut.angle = angle / n;
    pOut.order = order / n;
    return pOut;
}

SkPoint3D SkPoint3D::operator/(double k)
{
    SkPoint3D pOut;
    for(int i = 0; i < 3; i++)
    {
        pOut.p[i] = p[i] / k;
    }
    pOut.dis = dis / k;
    pOut.angle = angle / k;
    pOut.order = order / k;
    return pOut;
}

SkPoint3D SkPoint3D::operator=(SkPoint3D pA)
{
    for(int i = 0; i < 3; i++)
    {
        p[i] = pA.p[i];
    }
    order = pA.order;
    dis = pA.dis;
    angle = pA.angle;
    return *this;
}

SkPoint2D::SkPoint2D()
{
    for(int i = 0; i < 2; i++)
    {
        p[i] = 0;
    }
    order = 0;
    dis = 0;
    angle = 0;
}

SkPoint2D::SkPoint2D(const SkPoint2D &pA)
{
    for(int i = 0; i < 2; i++)
    {
        p[i] = pA.p[i];
    }
    order = pA.order;
    dis = pA.dis;
    angle = pA.angle;
}

SkPoint2D::SkPoint2D(double yA, double zA)
{
    y = yA;
    z = zA;
    order = 0;
    dis = 0;
    angle = 0;
}

SkPoint2D::SkPoint2D(double yA, double zA, int orderA)
{
    y = yA;
    z = zA;
    order = orderA;
    dis = 0;
    angle = 0;
}

SkPoint2D::SkPoint2D(double yA, double zA, int orderA, double disA, double angleA)
{
    y = yA;
    z = zA;
    order = orderA;
    dis = disA;
    angle = angleA;
}

void SkPoint2D::init()
{
    for(int i = 0; i < 2; i++)
    {
        p[i] = 0;
    }
    dis = 0;
    angle = 0;
    order = 0;
}

int SkPoint2D::isZero()
{
    for(int i = 0; i < 2; i++)
    {
        if(abs(p[i]) > EPS)
            return FALSE;
    }
    if(abs(dis) > EPS)
        return FALSE;
    if(abs(angle) > EPS)
        return FALSE;
    if(abs(order) > EPS)
        return FALSE;
    return TRUE;
}

void SkPoint2D::print()
{
    qDebug() << "[" << y << " " << z << "," << dis << " " << angle << " " << order << "]";
}

SkPoint2D SkPoint2D::operator+(SkPoint2D pA)
{
    SkPoint2D pOut;
    for(int i = 0; i < 2; i++)
    {
        pOut.p[i] = p[i] + pA.p[i];
    }
    pOut.dis = dis + pA.dis;
    pOut.angle = angle + pA.angle;
    pOut.order = order + pA.order;
    return pOut;
}

SkPoint2D SkPoint2D::operator-(SkPoint2D pA)
{
    SkPoint2D pOut = pA;
    for(int i = 0; i < 2; i++)
    {
        pOut.p[i] = p[i] - pA.p[i];
    }
    pOut.dis = dis - pA.dis;
    pOut.angle = angle - pA.angle;
    pOut.order = order - pA.order;
    return pOut;
}

SkPoint2D SkPoint2D::operator*(int n)
{
    SkPoint2D pOut;
    for(int i = 0; i < 2; i++)
    {
        pOut.p[i] = p[i] * n;
    }
    return pOut;
}

SkPoint2D SkPoint2D::operator*(double k)
{
    SkPoint2D pOut;
    for(int i = 0; i < 2; i++)
    {
        pOut.p[i] = p[i] * k;
    }
    return pOut;
}

SkPoint2D SkPoint2D::operator/(int n)
{
    SkPoint2D pOut;
    for(int i = 0; i < 2; i++)
    {
        pOut.p[i] = p[i] / n;
    }
    return pOut;
}

SkPoint2D SkPoint2D::operator/(double k)
{
    SkPoint2D pOut;
    for(int i = 0; i < 2; i++)
    {
        pOut.p[i] = p[i] / k;
    }
    return pOut;
}

SkPoint2D SkPoint2D::operator=(SkPoint2D pA)
{
    for(int i = 0; i < 2; i++)
    {
        p[i] = pA.p[i];
    }
    order = pA.order;
    dis = pA.dis;
    angle = pA.angle;
    return *this;
}

SkLine::SkLine()
{
    k = 0;
    b = 0;
}

SkLine::SkLine(double m, double n)
{
    k = m;
    b = n;
}

SkContourInfo::SkContourInfo()
{
    stage = ReadyStage;
    coordinate = 0;
    order = 0;
    timestamp = 0;
    area = 0;
    gap = 0;
    mismatch = 0;
    for(int i = 0; i < 150; i++)
    {
        errInfo[i] = 0;
    }
    errId = 0;
    result = 0;
    mp.init();
    op.init();
    bmp.erase(bmp.begin(), bmp.begin());
    bop.erase(bop.begin(), bop.begin());
    line.erase(line.begin(), line.begin());
    iPara.erase(iPara.begin(), iPara.begin());
    dPara.erase(dPara.begin(), dPara.begin());
}

void SkContourInfo::init()
{
    stage = ReadyStage;
    coordinate = 0;
    order = 0;
    timestamp = 0;
    area = 0;
    gap = 0;
    mismatch = 0;
    for(int i = 0; i < 150; i++)
    {
        errInfo[i] = 0;
    }
    errId = 0;
    result = 0;
    mp.init();
    op.init();
    bmp.erase(bmp.begin(), bmp.begin());
    bop.erase(bop.begin(), bop.begin());
    line.erase(line.begin(), line.begin());
    iPara.erase(iPara.begin(), iPara.begin());
    dPara.erase(dPara.begin(), dPara.begin());
}

SkContourInfo SkContourInfo::operator+(SkContourInfo cA)
{
    SkContourInfo infoOut = cA;
    infoOut.gap = gap + cA.gap;
    infoOut.mismatch = mismatch + cA.mismatch;
    infoOut.area = area + cA.area;
    infoOut.mp = mp + cA.mp;
    infoOut.op = op + cA.op;
    return infoOut;
}

SkContourInfo SkContourInfo::operator-(SkContourInfo cA)
{
    SkContourInfo infoOut = cA;
    infoOut.gap = gap - cA.gap;
    infoOut.mismatch = mismatch - cA.mismatch;
    infoOut.area = area - cA.area;
    infoOut.mp = mp - cA.mp;
    infoOut.op = op - cA.op;
    return infoOut;
}

SkContourInfo SkContourInfo::operator*(int n)
{
    SkContourInfo infoOut = *this;
    infoOut.gap = gap *n;
    infoOut.mismatch = mismatch *n;
    infoOut.area = area *n;
    infoOut.mp = mp *n;
    infoOut.op = op *n;
    return infoOut;
}

SkContourInfo SkContourInfo::operator*(double k)
{
    SkContourInfo infoOut = *this;
    infoOut.gap = gap *k;
    infoOut.mismatch = mismatch *k;
    infoOut.area = area *k;
    infoOut.mp = mp *k;
    infoOut.op = op *k;
    return infoOut;
}

SkContourInfo SkContourInfo::operator/(int n)
{
    SkContourInfo infoOut = *this;
    infoOut.gap = gap / n;
    infoOut.mismatch = mismatch / n;
    infoOut.area = area / n;
    infoOut.mp = mp / n;
    infoOut.op = op / n;
    return infoOut;
}

SkContourInfo SkContourInfo::operator/(double k)
{
    SkContourInfo infoOut = *this;
    infoOut.gap = gap / k;
    infoOut.mismatch = mismatch / k;
    infoOut.area = area / k;
    infoOut.mp = mp / k;
    infoOut.op = op / k;
    return infoOut;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CornerGroove::CornerGroove()
{
    leftMatchThre = 0.5;
    rightMatchThre = 0.5;
    leftSlopeAngleMin = -80;
    leftSlopeAngleMax = -10;
    rightSlopeAngleMin = 10;
    rightSlopeAngleMax = 80;
    includedAngleMin = 30;
    includedAngleMax = 150;
    leftLengthMin = 10;
    leftLengthMax = 200;
    rightLengthMin = 10;
    rightLengthMax = 200;
    leftHeightMin = 5;
    leftHeightMax = 200;
    rightHeightMin = 5;
    rightHeightMax = 200;
    plateType = 0;
    priority = 0;
    trackPos = 0;
    gapMaxBot = 30;
    gapMaxInPlane = 10;
    disNumMax = 500;
    planeNumMin = 50;
    normalMethod = 0;
    breakFootEnable = false;
}

void CornerGroove::init()
{
    CornerGroove CornerPara;
    *this = CornerPara;
}

VeeFormGroove::VeeFormGroove()
{
    planeMatchThre = 0.5;
    slopeMatchThre = 0.5;
    planeDisMatchThre = 0.5;
    planeAngleMin = -20;
    planeAngleMax = 20;
    leftSlopeAngleMin = -80;
    leftSlopeAngleMax = -30;
    rightSlopeAngleMin = 30;
    rightSlopeAngleMax = 80;
    leftHeightMin = 5;
    leftHeightMax = 200;
    rightHeightMin = 5;
    rightHeightMax = 200;
    widthMin = 5;
    widthMax = 100;
    mismatchMax = 10;
    trackPos = 0;
    disNumMax = 500;
    planeNumMin = 50;
    includedAngleMin = 30;
    includedAngleMax = 150;
    gapMaxInPlane = 10;
    plateType = 0;
    slopeAngleEnable = 0;
    leftSlopeAngleIn = -60;
    rightSlopeAngleIn = 60;
    thickEnable = 0;
    leftThicknessIn = 10;
    rightThicknessIn = 10;
    breakFootEnable = false;
}

void VeeFormGroove::init()
{
    VeeFormGroove VeeFormPara;
    *this = VeeFormPara;
}

SemiVeeGroove::SemiVeeGroove()
{
    matchThre = 0.5;
    smallSlopeMin = -85;
    smallSlopeMax = -30;
    bigSlopeMin = 30;
    bigSlopeMax = 85;
    includedAngleMin = 30;
    includedAngleMax = 90;
    topGapMin = 3;
    topGapMax = 50;
    thicknessMin = 5;
    thicknessMax = 30;
    planeAngleMin = -20;
    planeAngleMax = 20;
    gapMaxInPlane = 10;
    disNumMax = 500;
    planeNumMin = 50;
    plateType = 0;
    trackPos = 0;
    normalMethod = 0;
    gapMethod = 0;
    breakFootEnable = false;
}

void SemiVeeGroove::init()
{
    SemiVeeGroove SemiVeePara;
    *this = SemiVeePara;
}

LapGroove::LapGroove()
{
    matchThre = 0.5;
    planeAngleMin = -50;
    planeAngleMax = 50;
    lapAngleMin = 50;
    lapAngleMax = 150;
    thicknessMin = 5;
    thicknessMax = 30;
    gapMaxInPlane = 5;
    sepMaxDis = 5;
    planeNumMin = 50;
    disNumMax = 500;
    plateType = 1;
    trackPos = 0;
}

void LapGroove::init()
{
    LapGroove LapPara;
    *this = LapPara;
}

ButtGroove::ButtGroove()
{
    planeMatchThre = 0.5;
    interestMatchThre = 1;
    planeAngleMin = -89.0;
    planeAngleMax = 89.0;
    gapMin = 0.5;
    gapMax = 10;
    mismatchMin = 0.5;
    mismatchMax = 10;
    dPlainPointsNum = 100;
    dPlainMaxDifHeight = 0.8;
    planeNumMin = 100;
    sheetType = 0;
    angleMethod = 0;
}

void ButtGroove::init()
{
    ButtGroove ButtPara;
    *this = ButtPara;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SkComplexVar::SkComplexVar()
{
    enable = false;
    checkable = false;
    property = "";
    varType = 0;
    iVal1 = 0;
    iMin1 = -2000;
    iMax1 = 2000;
    iVal2 = 0;
    iMin2 = -2000;
    iMax2 = 2000;
    dVal1 = 0;
    dMin1 = -2000.0;
    dMax1 = 2000.0;
    dVal2 = 0;
    dMin2 = -2000.0;
    dMax2 = 2000.0;
    cVal = 0;
    qVal.erase(qVal.begin(), qVal.end());
}

void SkComplexVar::init()
{
    SkComplexVar complexVarPara;
    *this = complexVarPara;
}

void SkComplexVar::print()
{
    if(varType == 0) //整形1
    {
        qDebug() << "checkable=" << checkable << "  eanble=" << enable << "  property=" << property << "  iVal1=" << QString::number(iVal1) << "  iMin1=" << iMin1 << "  iMax1=" << iMax1;
    }
    else if(varType == 1) //整形2
    {
        qDebug() << "checkable=" << checkable << "  eanble=" << enable << "  property=" << property << "  iVal1=" << QString::number(iVal1) << "  iMin1=" << iMin1 << "  iMax1=" << iMax1 << \
                 "  iVal2=" << QString::number(iVal2) << "  iMin2=" << iMin2 << "  iMax2=" << iMax2;
    }
    else if(varType == 2) //浮点型1
    {
        qDebug() << "checkable=" << checkable << "  eanble=" << enable << "  property=" << property << "  dVal1=" << QString::number(dVal1, 'f', 3) << "  dMin1=" << dMin1 << "  dMax1=" << dMax1;
    }
    else if(varType == 3) //浮点型1
    {
        qDebug() << "checkable=" << checkable << "  eanble=" << enable << "  property=" << property << "  dVal1=" << QString::number(dVal1, 'f', 3) << "  dMin1=" << dMin1 << "  dMax1=" << dMax1 << \
                 "  dVal2=" << QString::number(dVal2, 'f', 3) << "  dMin2=" << dMin2 << "  dMax2=" << dMax2;
    }
    else if(varType == 4) //选择性
    {
        QString str;
        for(int i = 0; i < (int)qVal.size(); i++)
        {
            str = str + "  qVal[" + QString::number(i) + "]=" + qVal[i];
        }
        qDebug() << "checkable=" << checkable << "  eanble=" << enable << "  property=" << property << "  cVal=" << QString::number(cVal) << str;
    }
}

SkSmartImageAI::SkSmartImageAI()
{
    para.erase(para.begin(), para.end());
}

void SkSmartImageAI::init()
{
    para.erase(para.begin(), para.end());
}

SkField::SkField()
{
    cYmin = -30;
    cYmax = 30;
    fYmin = -100;
    fYmax = 100;
    cZ = -50;
    fZ = -400;
}

void SkField::init()
{
    SkField fieldPara;
    *this = fieldPara;
}

SkGroove::SkGroove()
{
    SkPoint3D pointPara;
    SkLine linePara;
    grooveType = 0;
    grooveP = pointPara;
    leftEndP = pointPara;
    rightEndP = pointPara;
    leftLine = linePara;
    rightLine = linePara;
    dataIn.erase(dataIn.begin(), dataIn.end());
    leftData.erase(leftData.begin(), leftData.end());
    rightData.erase(rightData.begin(), rightData.end());
}

void SkGroove::init()
{
    SkGroove groovePara;
    *this = groovePara;
}

SkSensor::SkSensor()
{
    mode = 0;
    frame = 0;
    ratio = 0;
    height = 0;
    intercept = 0;
}

SkPlaneProperty::SkPlaneProperty()
{
    iPlainExist = FALSE;
    iOrder = 0;
    linePos.init();
    leftP.init();
    rightP.init();
    dPlainTolerance = 0;
}

void SkPlaneProperty::init()
{
    SkPlaneProperty property;
    *this = property;
}

void SkPlaneProperty::reverse()
{
    SkPoint3D tempP;
    linePos.y = -linePos.y;
    tempP = leftP;
    leftP = rightP;
    rightP = tempP;
    leftP.y = -leftP.y;
    rightP.y = -rightP.y;
}

SkScanResult::SkScanResult()
{
    iGotResult = FALSE;
    iOrder = 0;
    algoFlag = 0;

    linePos.init();
    dLowY = 0;
    dHighY = 0;
    dLowHeight = 0;
    dHighHeight = 0;

    leftPlaneLeftP.init();
    leftPlaneRightP.init();
    rightPlaneLeftP.init();
    rightPlaneRightP.init();
    midP.init();
}

void SkScanResult::init()
{
    SkScanResult scanResult;
    *this = scanResult;
}

void SkScanResult::reverse()
{
    linePos.y = -linePos.y;
    dLowY = -dLowY;
    dHighY = -dHighY;
    dLowHeight = dLowHeight;
    dHighHeight = dHighHeight;

    SkPoint3D tempP;
    tempP = leftPlaneRightP;
    leftPlaneRightP = rightPlaneLeftP;
    rightPlaneLeftP = tempP;
    leftPlaneRightP.y = -leftPlaneRightP.y;
    rightPlaneLeftP.y = -rightPlaneLeftP.y;

    tempP = leftPlaneLeftP;
    leftPlaneLeftP = rightPlaneRightP;
    rightPlaneRightP = tempP;
    leftPlaneLeftP.y = -leftPlaneLeftP.y;
    rightPlaneRightP.y = -rightPlaneRightP.y;

    midP.y = -midP.y;
}


SkScanAlgCmdDet::SkScanAlgCmdDet()
{
    scanResultVector.erase(scanResultVector.begin(), scanResultVector.end());
    scanResult.init();
    scanResCount = 0;
    algoCounts.erase(algoCounts.begin(), algoCounts.end());
}

void SkScanAlgCmdDet::init()
{
    SkScanAlgCmdDet scanAlgCmdDet;
    *this = scanAlgCmdDet;
}

void SkScanAlgCmdDet::reverse()
{
    for(int i = 0; i < (int)scanResultVector.size(); i++)
    {
        scanResultVector[i].reverse();
    }
    scanResult.reverse();
}

SkBase::SkBase()
{
    frame = 0;
    base.init();
    tolorence.init();
    threshold.init();
}

SkBase::SkBase(const SkBase &mB)
{
    frame = mB.frame;
    base = mB.base;
    tolorence = mB.tolorence;
    threshold = mB.threshold;
}

void SkBase::init()
{
    frame = 0;
    base.init();
    tolorence.init();
    threshold.init();
}

//数据类型转换
SkMatrix4d joint2trans(SkVector6d jr, int option)
{
    SkMatrix4d tOut;
    tOut = rotz(jr.c, option) * roty(jr.b, option) * rotx(jr.a, option);
    tOut.px = jr.x;
    tOut.py = jr.y;
    tOut.pz = jr.z;
    return tOut;
}

SkVector3d joint2vector(SkVector6d jr)
{
    SkVector3d vOut;
    vOut.x = jr.x;
    vOut.y = jr.y;
    vOut.z = jr.z;
    return vOut;
}

SkPoint3D joint2Point3d(SkVector6d jr)
{
    SkPoint3D pOut;
    pOut.x = jr.x;
    pOut.y = jr.y;
    pOut.z = jr.z;
    return pOut;
}

SkVector9d joint2extendJ(SkVector6d jr)
{
    SkVector9d eOut;
    eOut.x = jr.x;
    eOut.y = jr.y;
    eOut.z = jr.z;
    eOut.a = jr.a;
    eOut.b = jr.b;
    eOut.c = jr.c;
    eOut.e1 = 0;
    eOut.e2 = 0;
    eOut.e3 = 0;
    return eOut;
}

SkMatrix4d vector2trans(SkVector3d jv)
{
    SkMatrix4d tOut;
    tOut.px = jv.x;
    tOut.py = jv.y;
    tOut.pz = jv.z;
    return tOut;
}

SkVector6d vector2joint(SkVector3d jv, int option, int index)
{
    SkVector6d jOut;
    if(index == 0)
    {
        jOut.x = jv.x;
        jOut.y = jv.y;
        jOut.z = jv.z;
    }
    else if(index == 1)
    {
        if(option == 1)
        {
            jOut.a = jv.x *PI / 180.0;
            jOut.b = jv.y *PI / 180.0;
            jOut.c = jv.z *PI / 180.0;
        }
        else
        {
            jOut.a = jv.x;
            jOut.b = jv.y;
            jOut.c = jv.z;
        }
    }
    return jOut;
}

SkPoint3D vector2Point3d(SkVector3d jv)
{
    SkPoint3D pOut;
    pOut.x = jv.x;
    pOut.y = jv.y;
    pOut.z = jv.z;
    return pOut;
}

SkVector9d vector2extendJ(SkVector3d jv, int option, int index)
{
    SkVector9d eOut;
    if(index == 0)
    {
        eOut.x = jv.x;
        eOut.y = jv.y;
        eOut.z = jv.z;
    }
    else if(index == 1)
    {
        if(option == 1)
        {
            eOut.a = jv.x *PI / 180.0;
            eOut.b = jv.y *PI / 180.0;
            eOut.c = jv.z *PI / 180.0;
        }
        else
        {
            eOut.a = jv.x;
            eOut.b = jv.y;
            eOut.c = jv.z;
        }
    }
    else if(index == 2)
    {
        eOut.e1 = jv.x;
        eOut.e2 = jv.y;
        eOut.e3 = jv.z;
    }
    return eOut;
}

SkVector6d trans2joint(SkMatrix4d tr, int option)
{
    SkVector6d jOut;
    jOut.x = tr.px;
    jOut.y = tr.py;
    jOut.z = tr.pz;
    tr2rpy(tr, jOut.a, jOut.b, jOut.c, option);
    return jOut;
}

SkVector3d trans2vector(SkMatrix4d tr)
{
    SkVector3d vOut;
    vOut.x = tr.px;
    vOut.y = tr.py;
    vOut.z = tr.pz;
    return vOut;
}

SkPoint3D trans2Point3d(SkMatrix4d tr)
{
    SkPoint3D pOut;
    pOut.x = tr.px;
    pOut.y = tr.py;
    pOut.z = tr.pz;
    return pOut;
}

SkVector9d trans2extendJ(SkMatrix4d tr, int option)
{
    SkVector9d eOut;
    eOut.x = tr.px;
    eOut.y = tr.py;
    eOut.z = tr.pz;
    tr2rpy(tr, eOut.a, eOut.b, eOut.c, option);
    return eOut;
}

SkVector6d point3d2joint(SkPoint3D pr)
{
    SkVector6d jOut;
    jOut.x = pr.x;
    jOut.y = pr.y;
    jOut.z = pr.z;
    return jOut;
}

SkVector3d point3d2vector(SkPoint3D pr)
{
    SkVector3d vOut;
    vOut.x = pr.x;
    vOut.y = pr.y;
    vOut.z = pr.z;
    return vOut;
}

SkMatrix4d point3d2trans(SkPoint3D pr)
{
    SkMatrix4d tOut;
    tOut.px = pr.x;
    tOut.py = pr.y;
    tOut.pz = pr.z;
    return tOut;
}

SkVector9d point3d2extendJ(SkPoint3D pr)
{
    SkVector9d eOut;
    eOut.x = pr.x;
    eOut.y = pr.y;
    eOut.z = pr.z;
    return eOut;
}

SkVector6d extendJ2joint(SkVector9d extJr)
{
    SkVector6d jOut;
    jOut.x = extJr.x;
    jOut.y = extJr.y;
    jOut.z = extJr.z;
    jOut.a = extJr.a;
    jOut.b = extJr.b;
    jOut.c = extJr.c;
    return jOut;
}

SkVector3d extendJ2vector(SkVector9d extJr)
{
    SkVector3d vOut;
    vOut.x = extJr.x;
    vOut.y = extJr.y;
    vOut.z = extJr.z;
    return vOut;
}

SkMatrix4d extendJ2trans(SkVector9d extJr, int option)
{
    SkMatrix4d tOut;
    tOut = rotz(extJr.c, option) * roty(extJr.b, option) * rotx(extJr.a, option);
    tOut.px = extJr.x;
    tOut.py = extJr.y;
    tOut.pz = extJr.z;
    return tOut;
}

SkPoint3D extendJ2point3d(SkVector9d extJr)
{
    SkPoint3D pOut;
    pOut.x = extJr.x;
    pOut.y = extJr.y;
    pOut.z = extJr.z;
    return pOut;
}
