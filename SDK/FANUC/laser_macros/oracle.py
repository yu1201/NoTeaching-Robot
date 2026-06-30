# -*- coding: utf-8 -*-
"""
激光传感器 FANUC 宏 —— 数学参考实现 (ground-truth oracle)
用途：在没有机器人的情况下，定义每个宏"应该"算出什么，给出可核对的数值标准答案。
机器人上的 KAREL/TP 实现必须与本文件逐数值一致。

约定（与 CLAUDE.md / FANUC 一致）：
- 点 = (x,y,z 单位 mm, w,p,r 单位度)；w绕X p绕Y r绕Z；姿态矩阵 R = Rz(r)*Ry(p)*Rx(w)
- 结果点的位置 = 几何计算所得；结果点的姿态(wpr) = "姿态点"原样拷贝（手册：使用点N的姿态）
- 标志 R[flag]: 1=成功, 0=失败(退化/平行/无解)
"""
import math

EPS = 1e-9

# ---------- 向量小工具 ----------
def sub(a, b):   return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def add(a, b):   return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def scale(a, s): return (a[0]*s, a[1]*s, a[2]*s)
def dot(a, b):   return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def norm(a):     return math.sqrt(dot(a, a))
def unit(a):
    n = norm(a)
    return scale(a, 1.0/n) if n > EPS else (0.0, 0.0, 0.0)

def solve3(M, b):
    """Cramer 法解 3x3 线性方程 M x = b；无解返回 None。KAREL 端同样手算行列式。"""
    def det3(m):
        return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
              - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
              + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))
    D = det3(M)
    if abs(D) < EPS:
        return None
    x = []
    for c in range(3):
        Mc = [[M[r][cc] if cc != c else b[r] for cc in range(3)] for r in range(3)]
        x.append(det3(Mc)/D)
    return tuple(x)

# ---------- FANUC 姿态 <-> 旋转矩阵 ----------
def wpr_to_R(w, p, r):
    cw, sw = math.cos(math.radians(w)), math.sin(math.radians(w))
    cp, sp = math.cos(math.radians(p)), math.sin(math.radians(p))
    cr, sr = math.cos(math.radians(r)), math.sin(math.radians(r))
    Rx = [[1,0,0],[0,cw,-sw],[0,sw,cw]]
    Ry = [[cp,0,sp],[0,1,0],[-sp,0,cp]]
    Rz = [[cr,-sr,0],[sr,cr,0],[0,0,1]]
    def mm(A,B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mm(mm(Rz, Ry), Rx)

def R_to_wpr(R):
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    if sy > EPS:
        w = math.degrees(math.atan2(R[2][1], R[2][2]))
        p = math.degrees(math.atan2(-R[2][0], sy))
        r = math.degrees(math.atan2(R[1][0], R[0][0]))
    else:  # 万向锁
        w = math.degrees(math.atan2(-R[1][2], R[1][1]))
        p = math.degrees(math.atan2(-R[2][0], sy))
        r = 0.0
    return (w, p, r)

# 一个点用 6 元组 (x,y,z,w,p,r) 表示
def P(x, y, z, w=0.0, p=0.0, r=0.0): return (x, y, z, w, p, r)
def xyz(pt): return (pt[0], pt[1], pt[2])
def wpr(pt): return (pt[3], pt[4], pt[5])
def make(pos, posture_pt): return (pos[0], pos[1], pos[2], posture_pt[3], posture_pt[4], posture_pt[5])

# ================= 七个宏的几何 =================

def AIH_CZ(p1, p2, p3, posture):
    """垂足：p3 到直线(p1,p2)的投影点。"""
    a = sub(xyz(p2), xyz(p1))
    if norm(a) < EPS:
        return None, 0
    t = dot(sub(xyz(p3), xyz(p1)), a) / dot(a, a)
    foot = add(xyz(p1), scale(a, t))
    return make(foot, posture), 1

def AIH_YC(p1, p2, posture, dist):
    """延长线：沿 p1->p2 方向，从 p2 再外延 dist。"""
    d = unit(sub(xyz(p2), xyz(p1)))
    if norm(d) < EPS:
        return None, 0
    ext = add(xyz(p2), scale(d, dist))
    return make(ext, posture), 1

def AIH_JD(p1, p2, p3, p4, posture):
    """交点：直线(p1,p2) 与 直线(p3,p4) 的最近点中点（3D 公垂线中点）。"""
    u = sub(xyz(p2), xyz(p1)); v = sub(xyz(p4), xyz(p3)); w0 = sub(xyz(p1), xyz(p3))
    a = dot(u, u); b = dot(u, v); c = dot(v, v); d = dot(u, w0); e = dot(v, w0)
    denom = a*c - b*b
    if abs(denom) < EPS:        # 平行
        return None, 0
    s = (b*e - c*d) / denom
    t = (a*e - b*d) / denom
    cp1 = add(xyz(p1), scale(u, s))
    cp2 = add(xyz(p3), scale(v, t))
    inter = scale(add(cp1, cp2), 0.5)
    return make(inter, posture), 1

def _circle_center(pts):
    """3D 共面点拟合圆心：拟合平面->投影2D->代数圆拟合(精确3点/最小二乘4点)->回映3D。"""
    n = len(pts)
    cen = scale((sum(q[0] for q in pts), sum(q[1] for q in pts), sum(q[2] for q in pts)), 1.0/n)
    # 平面法向：用前三点叉积（足够3/4点用）
    nz = unit(cross(sub(pts[1], pts[0]), sub(pts[2], pts[0])))
    if norm(nz) < EPS:
        return None
    ex = unit(sub(pts[0], cen)); ey = cross(nz, ex)
    uv = [(dot(sub(q, cen), ex), dot(sub(q, cen), ey)) for q in pts]
    # 代数圆拟合: u^2+v^2 + D u + E v + F = 0  ->  线性最小二乘
    Suu=Suv=Svv=Su=Sv=Sn=0.0; Suuu=Svvv=Suvv=Svuu=0.0
    for (u, v) in uv:
        Suu+=u*u; Suv+=u*v; Svv+=v*v; Su+=u; Sv+=v; Sn+=1
        Suuu+=u*u*u; Svvv+=v*v*v; Suvv+=u*v*v; Svuu+=v*u*u
    M=[[Suu,Suv,Su],[Suv,Svv,Sv],[Su,Sv,Sn]]
    rhs=[-(Suuu+Suvv), -(Svvv+Svuu), -(Suu+Svv)]
    sol=solve3(M, rhs)
    if sol is None:
        return None
    Dc,Ec,_=sol
    cu,cv=-Dc/2.0,-Ec/2.0
    center3=add(add(cen, scale(ex, cu)), scale(ey, cv))
    return center3

def AIH_YX(p1, p2, p3, p4, posture, method):
    """圆心：method=1 三点法(p1,p2,p3)；method=2 四点法(p1..p4 最小二乘)。"""
    pts = [xyz(p1), xyz(p2), xyz(p3)] if method == 1 else [xyz(p1), xyz(p2), xyz(p3), xyz(p4)]
    c = _circle_center(pts)
    if c is None:
        return None, 0
    return make(c, posture), 1

def _frame_to_xyzwpr(origin, ex, ey, ez):
    R = [[ex[0], ey[0], ez[0]],
         [ex[1], ey[1], ez[1]],
         [ex[2], ey[2], ez[2]]]
    w, p, r = R_to_wpr(R)
    return (origin[0], origin[1], origin[2], w, p, r)

def AIH_YH(p1, p2, p3, p4, method):
    """用户坐标系（返回 origin+wpr）。三种求法按手册示意图的最常见解释，待现场确认。"""
    o = xyz(p1)
    if method == 1:      # 三点法：原点P1, +X朝P2, P3定+XY平面(+Y侧)
        ex = unit(sub(xyz(p2), xyz(p1)))
        ez = unit(cross(ex, sub(xyz(p3), xyz(p1))))
        ey = cross(ez, ex)
    elif method == 2:    # 四点法：原点P1, +X朝P2, +Y方向由P3->P4
        ex = unit(sub(xyz(p2), xyz(p1)))
        ez = unit(cross(ex, sub(xyz(p4), xyz(p3))))
        ey = cross(ez, ex)
    else:                # method=3：原点P1, +X朝P2, +Y朝P3
        ex = unit(sub(xyz(p2), xyz(p1)))
        ey0 = sub(xyz(p3), xyz(p1))
        ez = unit(cross(ex, ey0)); ey = cross(ez, ex)
    if norm(ex) < EPS or norm(ez) < EPS:
        return None, 0
    return _frame_to_xyzwpr(o, ex, ey, ez), 1

# ================= 自检测试（已知答案） =================
def approx(a, b, tol=1e-6): return all(abs(x-y) < tol for x, y in zip(a, b))

def run_tests():
    ok = 0; tot = 0
    def check(name, got, exp, tol=1e-6):
        nonlocal ok, tot; tot += 1
        good = got is not None and approx(got[:3], exp, tol)
        ok += good
        g = tuple(round(v, 4) for v in (got or ())[:3])
        print(f"[{'PASS' if good else 'FAIL'}] {name}: got={g} exp={tuple(round(v,4) for v in exp)}")

    posture = P(0, 0, 0, 30, 0, 90)  # 任意姿态点

    # AIH_CZ: 点(2,3,0) 投到 X轴(0,0,0)-(10,0,0) -> 垂足(2,0,0)
    r, f = AIH_CZ(P(0,0,0), P(10,0,0), P(2,3,0), posture); check("AIH_CZ 垂足", r, (2,0,0))
    # 姿态应拷贝自 posture
    print("        AIH_CZ 姿态拷贝:", tuple(round(v,3) for v in r[3:]), "应=", (30.0,0.0,90.0))

    # AIH_YC: p1(0,0,0)->p2(0,10,0) 再外延 5 -> (0,15,0)
    r, f = AIH_YC(P(0,0,0), P(0,10,0), posture, 5); check("AIH_YC 延长", r, (0,15,0))

    # AIH_JD: 直线 X轴 (y=0) 与 直线 (x=4 的竖线) 交于 (4,0,0)
    r, f = AIH_JD(P(-5,0,0), P(5,0,0), P(4,-3,0), P(4,7,0), posture); check("AIH_JD 交点", r, (4,0,0))

    # AIH_YX 三点法: 单位圆上三点 -> 圆心(0,0,0)
    r, f = AIH_YX(P(1,0,0), P(0,1,0), P(-1,0,0), P(0,0,0), posture, 1); check("AIH_YX 三点圆心", r, (0,0,0))
    # AIH_YX 四点法: 圆心(2,3,0) 半径5 上四点
    c=(2,3,0); R5=5
    pp=[P(c[0]+R5,c[1],0),P(c[0],c[1]+R5,0),P(c[0]-R5,c[1],0),P(c[0],c[1]-R5,0)]
    r, f = AIH_YX(pp[0],pp[1],pp[2],pp[3], posture, 2); check("AIH_YX 四点圆心", r, c, 1e-5)

    # AIH_YH 三点法: 原点(0,0,0), P2 在 +X, P3 在 +Y -> 单位阵姿态(0,0,0)
    r, f = AIH_YH(P(0,0,0), P(10,0,0), P(0,10,0), P(0,0,0), 1)
    check("AIH_YH 原点", r, (0,0,0)); print("        AIH_YH 姿态wpr:", tuple(round(v,3) for v in r[3:]), "应≈(0,0,0)")
    # P2 在 +X, P3 在 +Y => ex=(1,0,0) ey=(0,1,0) ez=(0,0,1) => wpr=(0,0,0)

    # AIH_YH 绕Z转90°: X轴指向+Y -> r=90
    r, f = AIH_YH(P(0,0,0), P(0,10,0), P(-10,0,0), P(0,0,0), 1)
    print("        AIH_YH 绕Z90 wpr:", tuple(round(v,3) for v in r[3:]), "应≈(0,0,90)")

    print(f"\n通过 {ok}/{tot}")

if __name__ == "__main__":
    run_tests()
