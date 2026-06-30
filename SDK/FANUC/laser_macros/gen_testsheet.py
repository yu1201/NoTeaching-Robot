# -*- coding: utf-8 -*-
"""用 oracle 生成各宏的"输入→标准答案"测试表（mm 量级真实数值），供上机核对。"""
import oracle as o
from oracle import P

def fmt(pt):
    if pt is None: return "FAIL"
    return "(" + ", ".join(f"{v:.3f}" for v in pt) + ")"

print("="*78)
print("FANUC 数学宏 —— 标准答案测试表（位置 mm，姿态 deg）")
print("="*78)

pose = P(0,0,0, 180, 0, 45)  # 通用姿态点：wpr=(180,0,45)

print("\n## AIH_CZ 垂足  调用 AIH_CZ(p1,p2,p3,姿态,垂足PR,标志)")
p1,p2,p3 = P(100,200,300), P(300,200,300), P(250,260,300)
print("  PR入: P1",fmt(p1),"P2",fmt(p2),"P3",fmt(p3),"姿态",fmt(pose))
r,f = o.AIH_CZ(p1,p2,p3,pose); print("  PR出: 垂足 =", fmt(r), " 标志 =", f)

print("\n## AIH_YC 延长线  调用 AIH_YC(p1,p2,姿态,距离,延长PR,标志)")
p1,p2 = P(100,100,50), P(100,300,50); dist=80
print("  PR入: P1",fmt(p1),"P2",fmt(p2),"距离 =",dist,"姿态",fmt(pose))
r,f = o.AIH_YC(p1,p2,pose,dist); print("  PR出: 延长点 =", fmt(r), " 标志 =", f)

print("\n## AIH_JD 交点  调用 AIH_JD(p1,p2,p3,p4,姿态,交点PR,标志)")
p1,p2,p3,p4 = P(0,0,0), P(200,0,0), P(120,-50,0), P(120,80,0)
print("  PR入: P1",fmt(p1),"P2",fmt(p2),"P3",fmt(p3),"P4",fmt(p4))
r,f = o.AIH_JD(p1,p2,p3,p4,pose); print("  PR出: 交点 =", fmt(r), " 标志 =", f)

print("\n## AIH_YX 圆心(三点法,求法=1)  调用 AIH_YX(p1,p2,p3,p4,姿态,1,圆心PR,标志)")
p1,p2,p3 = P(210,150,0), P(150,210,0), P(90,150,0)
print("  PR入: P1",fmt(p1),"P2",fmt(p2),"P3",fmt(p3))
r,f = o.AIH_YX(p1,p2,p3,P(0,0,0),pose,1); print("  PR出: 圆心 =", fmt(r), " 标志 =", f, " (真值圆心 150,150,0 r=60)")

print("\n## AIH_YX 圆心(四点法,求法=2)  调用 AIH_YX(p1,p2,p3,p4,姿态,2,圆心PR,标志)")
c=(150,150,0); R=60
p1,p2,p3,p4 = P(c[0]+R,c[1],0), P(c[0],c[1]+R,0), P(c[0]-R,c[1],0), P(c[0],c[1]-R,0)
print("  PR入: P1",fmt(p1),"P2",fmt(p2),"P3",fmt(p3),"P4",fmt(p4))
r,f = o.AIH_YX(p1,p2,p3,p4,pose,2); print("  PR出: 圆心 =", fmt(r), " 标志 =", f, " (真值圆心 150,150,0)")

print("\n## AIH_YH 用户坐标系(求法=1 三点法)  调用 AIH_YH(p1,p2,p3,p4,1,坐标系号,标志)")
p1,p2,p3 = P(500,0,400), P(600,0,400), P(500,100,400)
print("  入: P1(原点)",fmt(p1),"P2(+X)",fmt(p2),"P3(+Y面)",fmt(p3))
r,f = o.AIH_YH(p1,p2,p3,P(0,0,0),1); print("  出: 坐标系 origin+wpr =", fmt(r), " 标志 =", f, " (应≈ 500,0,400, 0,0,0)")

print("\n## AIH_YH(求法=1, 带姿态：X朝+Y、绕Z转90°)")
p1,p2,p3 = P(500,0,400), P(500,100,400), P(400,0,400)
print("  入: P1(原点)",fmt(p1),"P2(+X)",fmt(p2),"P3(+Y面)",fmt(p3))
r,f = o.AIH_YH(p1,p2,p3,P(0,0,0),1); print("  出: origin+wpr =", fmt(r), " (应≈ 500,0,400, 0,0,90)")
