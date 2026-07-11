#include "RobotDriverAdaptor.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{
    // ——— pointwise 摆动几何 helper（三维向量，仅用 T_ROBOT_COORS 的位置分量 dX/dY/dZ）———
    struct WeaveVec3 { double x = 0.0, y = 0.0, z = 0.0; };
    inline double WeaveDot(const WeaveVec3& a, const WeaveVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline double WeaveNorm(const WeaveVec3& v) { return std::sqrt(WeaveDot(v, v)); }
    inline WeaveVec3 WeaveNormalize(const WeaveVec3& v)
    {
        const double n = WeaveNorm(v);
        return n > 1e-9 ? WeaveVec3{ v.x / n, v.y / n, v.z / n } : WeaveVec3{ 0.0, 0.0, 0.0 };
    }
    inline WeaveVec3 WeaveCross(const WeaveVec3& a, const WeaveVec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    // 一个周期内的横向系数[-1,1]
    inline double WeaveLateralFactor(int shapeValue, double phi)
    {
        constexpr double kPi = 3.14159265358979323846;
        switch (static_cast<EWeaveShape>(shapeValue))
        {
        case EWeaveShape::eBackForward:                 // 纵向往复：横向不动(摆动只沿切向)
            return 0.0;
        case EWeaveShape::eObliqueTriangle:             // 三角 / L摆：横向走三角波 (2/π)·asin(sin)
        case EWeaveShape::eSpaceTriangle:
        case EWeaveShape::eLTriangle:
            return (2.0 / kPi) * std::asin(std::sin(phi));
        case EWeaveShape::eSin:                          // 正弦
        case EWeaveShape::eSinFreq:
        default:
            return std::sin(phi);
        }
    }

    // 角度最近等价插值：先把 to 折到离 from 最近的等价角(差值落 [-180,180])再线性插值，输出夹到 [-180,180)(+180→-180)。
    // 防 RZ 等姿态角跨 ±180 边界被朴素线性插值绕远路——如 -178→+178 实际只差 4°，朴素插值会走经 0 的 356° 长路径
    // (焊枪在该段假性翻转近一圈)，这里改走经 ±180 的 4° 短路径。等价于 NormalizeAngleNear + NormalizeRobotRzOutputRange。
    inline double WeaveLerpAngleDeg(double from, double to, double t)
    {
        double delta = to - from;
        while (delta > 180.0) { delta -= 360.0; }
        while (delta < -180.0) { delta += 360.0; }
        double result = from + delta * t;
        while (result >= 180.0) { result -= 360.0; }
        while (result < -180.0) { result += 360.0; }
        return result;
    }

    // pointwise 每周期采样点数规范化：取 4 的倍数(让停留落 1/4·2/4·3/4·4/4 相位)、最小4；0/无效(旧工艺无此字段)回退默认16
    inline int NormalizeWeavePointsPerCycle(int n)
    {
        if (n < 4) { return 16; }
        const int rounded = (n / 4) * 4;
        return rounded < 4 ? 4 : rounded;
    }

    long long RobotDriverSteadyMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void FillPcPassiveTimestamp(long long* pRobotMs, long long* pPcRecvMs, long long pcMs)
    {
        // Non-monitor drivers use PC steady time as the temporary robot timeline.
        if (pRobotMs != nullptr)
        {
            *pRobotMs = pcMs;
        }
        if (pPcRecvMs != nullptr)
        {
            *pPcRecvMs = pcMs;
        }
    }
}


RobotDriverAdaptor::RobotDriverAdaptor(std::string sRobotName,RobotLog* pRobotLog)
    : m_nExternalAxleType(0),
    m_nRobotAxisCount(6),
    m_pRobotLog(pRobotLog), // 初始化日志：指定路径+控制台输出
    m_pFTP(nullptr),
    m_stateMonitorRunning(false),
    m_stateMonitorNextSequence(0)
{
    LoadRobotKinematicsPara(sRobotName, m_tKinematics, m_tAxisUnit, m_tAxisLimitAngle);
    LoadRobotExternalAxlePara(sRobotName);
    CreateFanucChain();
    m_pRobotLog->write(LogColor::SUCCESS, "RobotDriverAdaptor 初始化完成，机器人链创建成功");

}

std::vector<T_ROBOT_MOVE_INFO> RobotDriverAdaptor::ExpandMoveInfosByPointwiseWeave(
    const std::vector<T_ROBOT_MOVE_INFO>& centerline, std::string* error)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinPitchMm = 1.0;      // 节距下限
    const auto setError = [error](const std::string& msg) { if (error != nullptr) { *error = msg; } };

    // 1) 找第一个 pointwise 摆动焊接点(锚点)；非 pointwise 原样返回(不报错)
    const T_ROBOT_MOVE_INFO* anchor = nullptr;
    for (const auto& mi : centerline)
    {
        if (mi.bWeldProcessEnabled && mi.bHasWeaveParam && mi.bAppPointwiseWeave)
        {
            anchor = &mi;
            break;
        }
    }
    if (anchor == nullptr)
    {
        return centerline;
    }
    if (centerline.size() < 2)
    {
        setError("pointwise 摆动失败：中心线点数不足(需>=2)");
        return {};
    }

    const T_WeaveDate& weave = anchor->tWeaveParam;
    const int kPointsPerCycle = NormalizeWeavePointsPerCycle(anchor->nWeavePointsPerCycle);  // 每周期点数(工艺可配,4倍数,默认16)
    const double amplitude = weave.dWeaveAmplitudeMm;   // 横向半摆幅 mm
    const double freqHz = weave.dWeaveFrequencyHz;
    const int    shape = weave.nWeaveShape;
    const double longAmp = amplitude;                   // 纵向往复/L摆的纵向幅度复用摆幅 dWeaveAmplitudeMm（不借用语义不清的原生 dEndLengthMm）
    // 摆弧倾斜角：横向摆动方向绕焊道切向旋转 swingDeg。0=水平摆；45=斜面摆(直角内角焊缝把摆动面摆到45°平分方向，与工件两面成等腰直角三角形)；90=竖直摆
    const double swingRad = weave.dSwingDirectionDeg * kPi / 180.0;
    const double swingCos = std::cos(swingRad);
    const double swingSin = std::sin(swingRad);
    // 摆动相位停留(srp WaitTime 完全停实现)：相位每跨过 k·π/2 设对应停留——1/4波峰→nPauseTime1，2/4中→2，3/4波谷→3，4/4中→4(ms)，0=不停
    const int pauseTimesMs[4] = {
        static_cast<int>(weave.nPauseTime1Ms), static_cast<int>(weave.nPauseTime2Ms),
        static_cast<int>(weave.nPauseTime3Ms), static_cast<int>(weave.nPauseTime4Ms) };

    // 2) 频率/速度无效直接报错(节距 λ=v/f 依赖焊接速度，不退化为"不摆")
    if (freqHz <= 1e-6)
    {
        setError("pointwise 摆动失败：摆动频率无效(dWeaveFrequencyHz<=0)");
        return {};
    }
    const double anchorVMmPerSec = anchor->dWeldSpeedMmPerMin / 60.0;
    if (anchorVMmPerSec <= 1e-6)
    {
        setError("pointwise 摆动失败：焊接速度无效(dWeldSpeedMmPerMin<=0)，无法按 λ=v/f 算摆动节距");
        return {};
    }

    const WeaveVec3 worldZ{ 0.0, 0.0, 1.0 };

    std::vector<T_ROBOT_MOVE_INFO> out;
    out.reserve(centerline.size() * kPointsPerCycle);

    // 整条焊缝共用一个连续相位时钟：phiBase 累计到上一段末尾的相位，跨段不清零。
    // 否则每段都从相位0重启，会在中心线锚点处出现"相位重置/波形回弹"(如 595 593 595 593 漏掉波峰 596)。
    double phiBase = 0.0;
    for (size_t i = 0; i + 1 < centerline.size(); ++i)
    {
        const T_ROBOT_MOVE_INFO& a = centerline[i];
        const T_ROBOT_MOVE_INFO& b = centerline[i + 1];
        const WeaveVec3 seg{ b.tCoord.dX - a.tCoord.dX, b.tCoord.dY - a.tCoord.dY, b.tCoord.dZ - a.tCoord.dZ };
        const double segLen = WeaveNorm(seg);
        if (segLen < 1e-6) { continue; }

        const WeaveVec3 tangent = WeaveNormalize(seg);                  // 切向(前进)
        WeaveVec3 lateral = WeaveNormalize(WeaveCross(worldZ, tangent));// 横向 N0 = worldZ × T (水平横向，品牌无关)
        if (WeaveNorm(lateral) < 1e-6) { lateral = WeaveVec3{ 1.0, 0.0, 0.0 }; }  // 竖直段兜底
        // 摆弧倾斜角：N0 绕切向 T 旋转 swingDeg。binormal B = T×N0(水平焊道时朝正上方)，N' = N0·cosθ + B·sinθ
        if (swingSin > 1e-9 || swingSin < -1e-9)
        {
            const WeaveVec3 binormal = WeaveNormalize(WeaveCross(tangent, lateral));
            lateral = WeaveVec3{
                lateral.x * swingCos + binormal.x * swingSin,
                lateral.y * swingCos + binormal.y * swingSin,
                lateral.z * swingCos + binormal.z * swingSin };
        }

        // 节距 λ = v/f；本段速度优先，无效用锚点速度(已校验有效)
        // TODO(S4)：速度补偿 k 在挂钩处统一处理；此处仅决定空间节距
        const double segVMmPerSec = a.dWeldSpeedMmPerMin > 1e-6 ? a.dWeldSpeedMmPerMin / 60.0 : anchorVMmPerSec;
        double pitchMm = segVMmPerSec / freqHz;
        if (pitchMm < kMinPitchMm) { pitchMm = kMinPitchMm; }

        const double dsMm = pitchMm / kPointsPerCycle;                  // 采样步长
        const int nSteps = std::max(1, static_cast<int>(std::ceil(segLen / dsMm)));

        int lastQuarter = static_cast<int>(phiBase / (kPi / 2.0));   // 1/4 相位计数(跨段连续，承接 phiBase)，用于在摆动相位点设停留
        for (int s = 0; s < nSteps; ++s)
        {
            const double dist = static_cast<double>(s) * dsMm;
            if (dist > segLen) { break; }
            const double u = dist / segLen;                            // 段内插值参数 [0,1)
            const double phi = phiBase + 2.0 * kPi * dist / pitchMm;  // 相位(全局连续：基相位 + 段内弧长相位)

            T_ROBOT_MOVE_INFO p = a;                                   // 复制全套工艺字段(电流/电压/速度/段类)
            // 中心点：位置 + 姿态都从中心线插值(中心线点自带姿态)
            p.tCoord.dX = a.tCoord.dX + (b.tCoord.dX - a.tCoord.dX) * u;
            p.tCoord.dY = a.tCoord.dY + (b.tCoord.dY - a.tCoord.dY) * u;
            p.tCoord.dZ = a.tCoord.dZ + (b.tCoord.dZ - a.tCoord.dZ) * u;
            p.tCoord.dRX = WeaveLerpAngleDeg(a.tCoord.dRX, b.tCoord.dRX, u);
            p.tCoord.dRY = WeaveLerpAngleDeg(a.tCoord.dRY, b.tCoord.dRY, u);
            p.tCoord.dRZ = WeaveLerpAngleDeg(a.tCoord.dRZ, b.tCoord.dRZ, u);  // 最近等价插值：防 RZ 跨±180 边界绕长路径(焊枪假翻转)

            const double lat = amplitude * WeaveLateralFactor(shape, phi);
            // 纵向分量(沿切向)。TODO：L摆精确形态待安川/新时达手册确认
            double lon = 0.0;
            if (static_cast<EWeaveShape>(shape) == EWeaveShape::eBackForward)
            {
                // 纵向往复(挑弧)：沿切向前后往返折线，横向不动 → 焊枪在焊缝线上前送/回退
                lon = longAmp * (2.0 / kPi) * std::asin(std::sin(phi));
            }
            else if (static_cast<EWeaveShape>(shape) == EWeaveShape::eLTriangle && std::sin(phi) > 0.0)
            {
                lon = longAmp * std::sin(phi);          // L摆：横向到一侧时沿切向挑出一段(占位)
            }

            p.tCoord.dX += lateral.x * lat + tangent.x * lon;
            p.tCoord.dY += lateral.y * lat + tangent.y * lon;
            p.tCoord.dZ += lateral.z * lat + tangent.z * lon;

            // 跨段共享锚点去重：段长恰为采样步长整数倍时，上一段末点与本段首点(s==0)位置重合，
            // 跳过以免 srd 出现零位移重复点(下游会当成停顿)。仅 i>0 的段首可能重合。
            if (s == 0 && i > 0 && !out.empty())
            {
                const auto& q = out.back().tCoord;
                if (std::abs(q.dX - p.tCoord.dX) < 1e-6 && std::abs(q.dY - p.tCoord.dY) < 1e-6
                    && std::abs(q.dZ - p.tCoord.dZ) < 1e-6)
                {
                    continue;
                }
            }

            p.bHasWeaveParam = false;   // 关原生 WEAVEDATA：靠点位本身摆，不再由控制器二次摆动
            // 圆滑(OVERLAPREL)沿用工艺填的值(p 从 a 复制已带)，不强制 0、不做限制。
            // 注意：圆滑越大越会把密集摆动点之间磨成圆弧、削掉摆幅，100% 会直接抹平摆动，合适值由现场测试定。
            // 摆动相位停留：相位每跨过一个 k·π/2(1/4波峰/2/4中/3/4波谷/4/4中)，在该点设停留时间(下游 srp 插 WaitTime 完全停)
            const int curQuarter = static_cast<int>(phi / (kPi / 2.0));
            if (curQuarter > lastQuarter)
            {
                const int dwellMs = pauseTimesMs[(curQuarter - 1) % 4];
                if (dwellMs > 0) { p.nDwellMs = dwellMs; }
                lastQuarter = curQuarter;
            }
            out.push_back(p);
        }
        // 段末把整段相位累加进基相位(用整段长 segLen，使下一段锚点相位严格接续)，实现跨段连续摆动
        phiBase += 2.0 * kPi * segLen / pitchMm;
    }

    // 末点：终点中心原样保留(收回中心线)，关 weave；圆滑沿用工艺值(last 从中心线末点复制已带)
    T_ROBOT_MOVE_INFO last = centerline.back();
    last.bHasWeaveParam = false;
    out.push_back(last);

    return out;
}

std::vector<T_ROBOT_MOVE_INFO> RobotDriverAdaptor::ApplyWeaveSpeedCompensation(
    const std::vector<T_ROBOT_MOVE_INFO>& centerline,
    const std::vector<T_ROBOT_MOVE_INFO>& weaveMoveInfo,
    double maxLinearSpeedMmPerSec,
    std::string* info)
{
    const auto setInfo = [info](const std::string& msg) { if (info != nullptr) { *info = msg; } };

    // 两序列各算路径总长(只用位置分量 dX/dY/dZ)
    const auto pathLength = [](const std::vector<T_ROBOT_MOVE_INFO>& pts) {
        double sum = 0.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i)
        {
            const WeaveVec3 d{
                pts[i + 1].tCoord.dX - pts[i].tCoord.dX,
                pts[i + 1].tCoord.dY - pts[i].tCoord.dY,
                pts[i + 1].tCoord.dZ - pts[i].tCoord.dZ };
            sum += WeaveNorm(d);
        }
        return sum;
    };

    const double centerLen = pathLength(centerline);
    const double weaveLen = pathLength(weaveMoveInfo);
    if (centerLen < 1e-6 || weaveLen < 1e-6)
    {
        return weaveMoveInfo;  // 无法算 k，原样返回
    }
    const double k = weaveLen / centerLen;
    if (k <= 1.0 + 1e-6)
    {
        return weaveMoveInfo;  // 未摆动(路径未变长)，无需补偿
    }

    // 限幅：保证最大点速度 × k 不超机器人/焊机上限(mm/s)
    double appliedK = k;
    if (maxLinearSpeedMmPerSec > 1e-6)
    {
        double maxBaseVMmPerSec = 0.0;
        for (const auto& p : weaveMoveInfo)
        {
            maxBaseVMmPerSec = std::max(maxBaseVMmPerSec, p.tSpeed.dSpeed);
            maxBaseVMmPerSec = std::max(maxBaseVMmPerSec, p.dWeldSpeedMmPerMin / 60.0);
        }
        if (maxBaseVMmPerSec > 1e-6 && maxBaseVMmPerSec * appliedK > maxLinearSpeedMmPerSec)
        {
            appliedK = maxLinearSpeedMmPerSec / maxBaseVMmPerSec;
            setInfo("摆动速度补偿：k=" + std::to_string(k) + " 超速度上限，已限幅到 k=" + std::to_string(appliedK));
        }
    }
    if (info != nullptr && info->empty())
    {
        setInfo("摆动速度补偿：k=" + std::to_string(appliedK)
            + "（摆动路径 " + std::to_string(weaveLen) + "mm / 中心线 " + std::to_string(centerLen) + "mm）");
    }

    // 两个运动速度字段都乘 k：STEP 用 dWeldSpeedMmPerMin(进 ARCDATA)，FANUC 用 tSpeed.dSpeed
    std::vector<T_ROBOT_MOVE_INFO> out = weaveMoveInfo;
    for (auto& p : out)
    {
        p.dWeldSpeedMmPerMin *= appliedK;
        p.tSpeed.dSpeed *= appliedK;
    }
    return out;
}

RobotDriverAdaptor::~RobotDriverAdaptor()
{
    StopStateMonitor();
    if (m_pFTP != nullptr)
    {
        delete m_pFTP;
        m_pFTP = nullptr;
    }
    m_pRobotLog->write(LogColor::DEFAULT, "RobotDriverAdaptor 析构完成");
}

bool RobotDriverAdaptor::InitRobotDriver(std::string strUnitName)
{
    return false;
}
// ===================== 核心函数：创建 FANUC 6 轴机器人链 =====================
void RobotDriverAdaptor::CreateFanucChain()
{
    m_pFanucChain = KDL::Chain();

    struct DhRow
    {
        double a_mm;
        double alpha_deg;
        double d_mm;
        double theta_deg;
    };

    const DhRow dh_rows[6] = {
        {m_tKinematics.dA1, m_tKinematics.dAL1, m_tKinematics.dD1, m_tKinematics.dTH1},
        {m_tKinematics.dA2, m_tKinematics.dAL2, m_tKinematics.dD2, m_tKinematics.dTH2},
        {m_tKinematics.dA3, m_tKinematics.dAL3, m_tKinematics.dD3, m_tKinematics.dTH3},
        {m_tKinematics.dA4, m_tKinematics.dAL4, m_tKinematics.dD4, m_tKinematics.dTH4},
        {m_tKinematics.dA5, m_tKinematics.dAL5, m_tKinematics.dD5, m_tKinematics.dTH5},
        {m_tKinematics.dA6, m_tKinematics.dAL6, m_tKinematics.dD6, m_tKinematics.dTH6},
    };

    for (int i = 0; i < 6; ++i) {
        const double a_m = dh_rows[i].a_mm / 1000.0;
        const double alpha_rad = dh_rows[i].alpha_deg * M_PI / 180.0;
        const double d_m = dh_rows[i].d_mm / 1000.0;
        const double theta_rad = dh_rows[i].theta_deg * M_PI / 180.0;

        m_pFanucChain.addSegment(
            KDL::Segment(
                KDL::Joint(KDL::Joint::RotZ),
                KDL::Frame::DH(a_m, alpha_rad, d_m, theta_rad)));
    }

    m_pRobotLog->write(LogColor::DEFAULT,
        "机器人链创建完成，共%d个关节段 | DH1(a=%.3f, alpha=%.3f, d=%.3f, th=%.3f)",
        m_pFanucChain.getNrOfSegments(),
        dh_rows[0].a_mm, dh_rows[0].alpha_deg, dh_rows[0].d_mm, dh_rows[0].theta_deg);
}



// ===================== 新增：旋转矩阵转 RX/RY/RZ（欧拉角，XYZ顺序） =====================
void RobotDriverAdaptor::rotationMatrixToRPY(const KDL::Rotation& rot, double& rx, double& ry, double& rz)
{
    // KDL 内置函数：旋转矩阵 → 滚转(Roll-X)、俯仰(Pitch-Y)、偏航(Yaw-Z)
    // 输出：rx=绕X轴旋转角(Roll)，ry=绕Y轴旋转角(Pitch)，rz=绕Z轴旋转角(Yaw)
    rot.GetRPY(rx, ry, rz);
}

// ===================== 正运动学求解 =====================
bool RobotDriverAdaptor::RobotKinematics(T_ANGLE_PULSE tRobotPulse, T_ROBOT_COORS tToolCoors, T_ROBOT_COORS& tRobotCoors)
{
    // 日志：正解开始，输入脉冲信息
    m_pRobotLog->write(LogColor::DEFAULT, "正运动学求解开始 - 输入脉冲：S=%ld, L=%ld, U=%ld, R=%ld, B=%ld, T=%ld",
        tRobotPulse.nSPulse, tRobotPulse.nLPulse, tRobotPulse.nUPulse,
        tRobotPulse.nRPulse, tRobotPulse.nBPulse, tRobotPulse.nTPulse);

    // 步骤1：初始化输出位姿为全0
    tRobotCoors = T_ROBOT_COORS();

    // 步骤2：脉冲→关节角度（度）转换（S/L/U/R/B/T轴）
    std::vector<double> joint_angles_deg(6);
    joint_angles_deg[0] = tRobotPulse.nSPulse * m_tAxisUnit.dSPulseUnit;  // S轴（J1）
    joint_angles_deg[1] = tRobotPulse.nLPulse * m_tAxisUnit.dLPulseUnit;  // L轴（J2）
    joint_angles_deg[2] = tRobotPulse.nUPulse * m_tAxisUnit.dUPulseUnit;  // U轴（J3）
    joint_angles_deg[3] = tRobotPulse.nRPulse * m_tAxisUnit.dRPulseUnit;  // R轴（J4）
    joint_angles_deg[4] = tRobotPulse.nBPulse * m_tAxisUnit.dBPulseUnit;  // B轴（J5）
    joint_angles_deg[5] = tRobotPulse.nTPulse * m_tAxisUnit.dTPulseUnit;  // T轴（J6）

    // 日志：脉冲转角度结果
    m_pRobotLog->write(LogColor::DEFAULT, "脉冲转关节角度(度)：J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f",
        joint_angles_deg[0], joint_angles_deg[1], joint_angles_deg[2],
        joint_angles_deg[3], joint_angles_deg[4], joint_angles_deg[5]);

    // 步骤3：角度→弧度转换（适配KDL）
    KDL::JntArray jnt_angles(6);
    for (int i = 0; i < 6; ++i) {
        jnt_angles(i) = joint_angles_deg[i] * M_PI / 180.0;
    }

    // 步骤4：创建机器人KDL模型（和逆解共用同一个DH模型）
    KDL::Chain robot_chain = m_pFanucChain;  // 复用之前的机器人链创建函数

    // 步骤5：正解求解器（递归法，KDL默认）
    KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
    KDL::Frame flange_frame;  // 法兰盘位姿（机器人末端法兰）
    // 执行正解：关节角度 → 法兰盘位姿
    int ret = fk_solver.JntToCart(jnt_angles, flange_frame);
    if (ret < 0) {
        m_pRobotLog->write(LogColor::ERR, "正解失败！错误码：%d", ret);
        return false;
    }
    m_pRobotLog->write(LogColor::SUCCESS, "法兰盘位姿正解成功 - 位置(米)：X=%.4f, Y=%.4f, Z=%.4f",
        flange_frame.p.x(), flange_frame.p.y(), flange_frame.p.z());

    // 步骤6：工具坐标系补偿（法兰盘位姿 + 工具偏移 = 工具末端位姿）
    // 6.1 转换工具坐标系到KDL格式
    KDL::Frame tool_frame;
    CoorsToKDLFrame(tToolCoors, tool_frame);  // 复用之前的位姿转换函数
    // 6.2 计算工具末端位姿：工具末端 = 法兰盘位姿 × 工具坐标系位姿
    KDL::Frame tcp_frame = flange_frame * tool_frame;

    // 步骤7：KDL位姿→T_ROBOT_COORS（米→毫米，弧度→度）
    tRobotCoors.dX = tcp_frame.p.x() * 1000.0;
    tRobotCoors.dY = tcp_frame.p.y() * 1000.0;
    tRobotCoors.dZ = tcp_frame.p.z() * 1000.0;
    // 提取RPY姿态角（弧度→度）
    double rx, ry, rz;
    tcp_frame.M.GetRPY(rx, ry, rz);
    tRobotCoors.dRX = rx * 180.0 / M_PI;
    tRobotCoors.dRY = ry * 180.0 / M_PI;
    tRobotCoors.dRZ = rz * 180.0 / M_PI;

    // 日志：正解最终结果
    m_pRobotLog->write(LogColor::SUCCESS, "正运动学求解完成 - 工具末端位姿(mm/度)：X=%.2f, Y=%.2f, Z=%.2f, RX=%.2f, RY=%.2f, RZ=%.2f",
        tRobotCoors.dX, tRobotCoors.dY, tRobotCoors.dZ,
        tRobotCoors.dRX, tRobotCoors.dRY, tRobotCoors.dRZ);

    return (ret >= 0); // 返回是否成功
}

// 最终逆解函数（匹配你的声明：RobotInverseKinematics）
bool RobotDriverAdaptor::RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ROBOT_COORS tToolCoors, std::vector<T_ANGLE_PULSE>& vtResultPulse)
{
    // 日志：逆解开始，输入目标位姿
    m_pRobotLog->write(LogColor::DEFAULT, "逆运动学求解开始 - 目标位姿(mm/度)：X=%.2f, Y=%.2f, Z=%.2f, RX=%.2f, RY=%.2f, RZ=%.2f",
        tRobotCoors.dX, tRobotCoors.dY, tRobotCoors.dZ,
        tRobotCoors.dRX, tRobotCoors.dRY, tRobotCoors.dRZ);

    // 1. 清空输出
    vtResultPulse.clear();

    // 3. 计算法兰盘目标位姿（工具补偿）
    KDL::Frame flange_frame = CalculateFlangeFrame(tRobotCoors, tToolCoors);
    m_pRobotLog->write(LogColor::DEFAULT, "法兰盘目标位姿(米)：X=%.4f, Y=%.4f, Z=%.4f",
        flange_frame.p.x(), flange_frame.p.y(), flange_frame.p.z());

    // 4. 创建机器人模型
    KDL::Chain robot_chain = m_pFanucChain;

    // 5. 求解所有有效逆解（关节角度，度）
    std::vector<std::vector<double>> all_joint_angles = SolveAllValidIK(robot_chain, flange_frame);
    if (all_joint_angles.empty()) {
        m_pRobotLog->write(LogColor::ERR, "逆解失败：无有效关节解！");
        return false;
    }
    m_pRobotLog->write(LogColor::SUCCESS, "逆解成功，共求解到%d组有效关节解", all_joint_angles.size());

    // 6. 关节角度→脉冲转换，填充输出
    for (size_t i = 0; i < all_joint_angles.size(); ++i) {
        const auto& angles = all_joint_angles[i];
        T_ANGLE_PULSE pulse;
        JointAngleToPulse(angles, pulse);
        vtResultPulse.push_back(pulse);

        // 日志：每组解的关节角度和脉冲
        m_pRobotLog->write(LogColor::DEFAULT, "第%d组解 - 关节角度(度)：J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f | 脉冲：S=%ld, L=%ld, U=%ld, R=%ld, B=%ld, T=%ld",
            i + 1, angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
            pulse.nSPulse, pulse.nLPulse, pulse.nUPulse, pulse.nRPulse, pulse.nBPulse, pulse.nTPulse);
    }

    return true;
}

bool RobotDriverAdaptor::RunKinematicsSelfTest(const T_ANGLE_PULSE& inputPulse, const T_ROBOT_COORS& toolCoors, T_ANGLE_PULSE* pBestResult)
{
    m_pRobotLog->write(LogColor::DEFAULT,
        "开始执行运动学自检(FK -> IK -> FK) | 输入脉冲: S=%ld, L=%ld, U=%ld, R=%ld, B=%ld, T=%ld",
        inputPulse.nSPulse, inputPulse.nLPulse, inputPulse.nUPulse,
        inputPulse.nRPulse, inputPulse.nBPulse, inputPulse.nTPulse);

    T_ROBOT_COORS tcpPose;
    if (!RobotKinematics(inputPulse, toolCoors, tcpPose)) {
        m_pRobotLog->write(LogColor::ERR, "运动学自检失败：FK 阶段求解失败");
        return false;
    }

    std::vector<T_ANGLE_PULSE> ikResults;
    if (!RobotInverseKinematics(tcpPose, toolCoors, ikResults) || ikResults.empty()) {
        m_pRobotLog->write(LogColor::ERR, "运动学自检失败：IK 阶段无有效解");
        return false;
    }

    auto pulseDistance = [](const T_ANGLE_PULSE& lhs, const T_ANGLE_PULSE& rhs) -> long long
        {
            return
                llabs(static_cast<long long>(lhs.nSPulse) - static_cast<long long>(rhs.nSPulse)) +
                llabs(static_cast<long long>(lhs.nLPulse) - static_cast<long long>(rhs.nLPulse)) +
                llabs(static_cast<long long>(lhs.nUPulse) - static_cast<long long>(rhs.nUPulse)) +
                llabs(static_cast<long long>(lhs.nRPulse) - static_cast<long long>(rhs.nRPulse)) +
                llabs(static_cast<long long>(lhs.nBPulse) - static_cast<long long>(rhs.nBPulse)) +
                llabs(static_cast<long long>(lhs.nTPulse) - static_cast<long long>(rhs.nTPulse));
        };

    size_t bestIndex = 0;
    long long bestDistance = pulseDistance(inputPulse, ikResults[0]);
    for (size_t i = 1; i < ikResults.size(); ++i) {
        const long long currentDistance = pulseDistance(inputPulse, ikResults[i]);
        if (currentDistance < bestDistance) {
            bestDistance = currentDistance;
            bestIndex = i;
        }
    }

    const T_ANGLE_PULSE& bestPulse = ikResults[bestIndex];
    if (pBestResult != nullptr) {
        *pBestResult = bestPulse;
    }

    T_ROBOT_COORS recoveredPose;
    if (!RobotKinematics(bestPulse, toolCoors, recoveredPose)) {
        m_pRobotLog->write(LogColor::ERR, "运动学自检失败：回代 FK 阶段求解失败");
        return false;
    }

    const double posErrX = recoveredPose.dX - tcpPose.dX;
    const double posErrY = recoveredPose.dY - tcpPose.dY;
    const double posErrZ = recoveredPose.dZ - tcpPose.dZ;
    const double rotErrX = recoveredPose.dRX - tcpPose.dRX;
    const double rotErrY = recoveredPose.dRY - tcpPose.dRY;
    const double rotErrZ = recoveredPose.dRZ - tcpPose.dRZ;

    m_pRobotLog->write(LogColor::SUCCESS,
        "运动学自检完成 | 选择第%d组逆解 | 脉冲差=%lld",
        static_cast<int>(bestIndex + 1), bestDistance);
    m_pRobotLog->write(LogColor::DEFAULT,
        "自检结果 | 回代脉冲: S=%ld, L=%ld, U=%ld, R=%ld, B=%ld, T=%ld",
        bestPulse.nSPulse, bestPulse.nLPulse, bestPulse.nUPulse,
        bestPulse.nRPulse, bestPulse.nBPulse, bestPulse.nTPulse);
    m_pRobotLog->write(LogColor::DEFAULT,
        "自检误差 | dX=%.3f mm, dY=%.3f mm, dZ=%.3f mm, dRX=%.3f deg, dRY=%.3f deg, dRZ=%.3f deg",
        posErrX, posErrY, posErrZ, rotErrX, rotErrY, rotErrZ);

    return true;
}

// 1. T_ROBOT_COORS → KDL::Frame（适配工具坐标系/法兰盘坐标系）
void RobotDriverAdaptor::CoorsToKDLFrame(const T_ROBOT_COORS& tRobotCoors, KDL::Frame& frame)
{
    // 位置：mm → 米；姿态：度 → 弧度
    double x = tRobotCoors.dX / 1000;
    double y = tRobotCoors.dY / 1000;
    double z = tRobotCoors.dZ / 1000;
    double rx = tRobotCoors.dRX * M_PI / 180.0;
    double ry = tRobotCoors.dRY * M_PI / 180.0;
    double rz = tRobotCoors.dRZ * M_PI / 180.0;

    // 构造KDL位姿（RPY旋转矩阵 + 位置向量）
    frame = KDL::Frame(KDL::Rotation::RPY(rx, ry, rz), KDL::Vector(x, y, z));
}

// 2. 计算法兰盘目标位姿（工具末端位姿 - 工具坐标系偏移）
KDL::Frame RobotDriverAdaptor::CalculateFlangeFrame(const T_ROBOT_COORS& tcp_target, const T_ROBOT_COORS& tool_coors)
{
    KDL::Frame tcp_frame, tool_frame;
    CoorsToKDLFrame(tcp_target, tcp_frame);   // 工具末端目标位姿
    CoorsToKDLFrame(tool_coors, tool_frame);  // 工具坐标系偏移（TCP相对于法兰盘）

    // 法兰盘位姿 = 工具末端位姿 × 工具坐标系位姿的逆（核心补偿）
    return tcp_frame * tool_frame.Inverse();
}

// 单组初始值求解逆解（返回关节角度，度）
bool RobotDriverAdaptor::SolveSingleIK(const KDL::Chain& chain, const KDL::Frame& flange_frame,
    const std::vector<double>& init_angles_deg, std::vector<double>& joint_angles_deg)
{
    // 日志：单组初始值逆解开始
    m_pRobotLog->write(LogColor::DEFAULT, "单组逆解求解 - 初始角度(度)：J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f",
        init_angles_deg[0], init_angles_deg[1], init_angles_deg[2],
        init_angles_deg[3], init_angles_deg[4], init_angles_deg[5]);

    // 1. 初始值转换：度→弧度，初始化KDL关节数组
    KDL::JntArray jnt_init(6), jnt_result(6);
    for (int i = 0; i < 6; ++i) {
        jnt_init(i) = init_angles_deg[i] * M_PI / 180.0;
    }

    // 2. 创建正解/速度逆解/位置逆解器（KDL 1.5.3 要求传入正解器）
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv vel_solver(chain);
    const unsigned int MAX_ITER = 200;  // 最大迭代次数
    const double EPS = 1e-6;           // 收敛精度
    KDL::ChainIkSolverPos_NR pos_solver(chain, fk_solver, vel_solver, MAX_ITER, EPS);

    // 3. 执行逆解
    int ret = pos_solver.CartToJnt(jnt_init, flange_frame, jnt_result);
    if (ret < 0) {
        m_pRobotLog->write(LogColor::WARNING, "单组逆解失败 - 初始角度求解返回错误码：%d", ret);
        return false; // 逆解失败
    }

    // 4. 结果转换：弧度→度
    joint_angles_deg.clear();
    for (int i = 0; i < 6; ++i) {
        joint_angles_deg.push_back(jnt_result(i) * 180.0 / M_PI);
    }

    m_pRobotLog->write(LogColor::DEFAULT, "单组逆解成功 - 结果角度(度)：J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f",
        joint_angles_deg[0], joint_angles_deg[1], joint_angles_deg[2],
        joint_angles_deg[3], joint_angles_deg[4], joint_angles_deg[5]);

    return true;
}

// 求解所有有效逆解（返回关节角度列表，度）
std::vector<std::vector<double>> RobotDriverAdaptor::SolveAllValidIK(const KDL::Chain& chain, const KDL::Frame& flange_frame)
{
    // 通用初始值列表（覆盖S/L/U/R/B/T轴核心姿态）
    std::vector<std::vector<double>> init_angles_list = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},          // 零位
        {0.0, 30.0, -30.0, 0.0, 0.0, 0.0},       // 肘上
        {0.0, -30.0, 30.0, 0.0, 0.0, 0.0},       // 肘下
        {90.0, 0.0, 0.0, 0.0, 0.0, 0.0},         // S轴+90°
        {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0},        // S轴-90°
        {0.0, 0.0, 0.0, 180.0, 0.0, 0.0},        // R轴+180°
        {0.0, 0.0, 0.0, 0.0, 30.0, 0.0},         // B轴+30°
        {90.0, 30.0, -30.0, 180.0, 0.0, 0.0}     // 组合姿态
    };

    // 遍历求解+筛选有效解
    std::vector<std::vector<double>> all_valid_sols;
    for (size_t i = 0; i < init_angles_list.size(); ++i) {
        const auto& init_angles = init_angles_list[i];
        std::vector<double> joint_angles;
        if (SolveSingleIK(chain, flange_frame, init_angles, joint_angles)) {
            if (IsJointAngleValid(joint_angles)) {
                // 去重后添加
                if (!IsDuplicateSolution(joint_angles, all_valid_sols)) {
                    all_valid_sols.push_back(joint_angles);
                    m_pRobotLog->write(LogColor::DEFAULT, "第%d组初始值求解到有效解，已加入结果集", i + 1);
                }
                else {
                    m_pRobotLog->write(LogColor::DEFAULT, "第%d组初始值求解结果为重复解，已过滤", i + 1);
                }
            }
            else {
                m_pRobotLog->write(LogColor::WARNING, "第%d组初始值求解结果超出关节限位，已过滤", i + 1);
            }
        }
        else {
            m_pRobotLog->write(LogColor::WARNING, "第%d组初始值求解失败", i + 1);
        }
    }

    return all_valid_sols;
}

// 5. 筛选有效关节角（在限位范围内）
bool RobotDriverAdaptor::IsJointAngleValid(const std::vector<double>& joint_angles_deg)
{
    if (joint_angles_deg.size() != 6) {
		m_pRobotLog->write(LogColor::ERR, "关节角度数量异常，期望6个，实际%d个", joint_angles_deg.size());
		return false;
	}
	for (int i = 0; i < 6; i++)
	{
		double min_limit = m_tAxisLimitAngle.GetMinAngleByIndex(i); // 弧度转度
		double max_limit = m_tAxisLimitAngle.GetMaxAngleByIndex(i);
		if (joint_angles_deg[i] < min_limit || joint_angles_deg[i] > max_limit) {
			m_pRobotLog->write(LogColor::WARNING, "J%d角度超限：%.2f° (范围：%.2f° ~ %.2f°)",
				i + 1, joint_angles_deg[i], min_limit, max_limit);
			return false;
		}
	}

    return true;
}

// 6. 逆解去重（误差<0.1度视为同一解）
bool RobotDriverAdaptor::IsDuplicateSolution(const std::vector<double>& new_sol, const std::vector<std::vector<double>>& exist_sols)
{
    for (const auto& sol : exist_sols) {
        double diff_sum = 0.0;
        for (int i = 0; i < 6; ++i) {
            diff_sum += fabs(new_sol[i] - sol[i]);
        }
        if (diff_sum < 0.1) { // 总误差<0.1度
            return true;
        }
    }
    return false;
}

// 4. 关节角度→脉冲转换（适配T_ANGLE_PULSE）
void RobotDriverAdaptor::JointAngleToPulse(const std::vector<double>& joint_angles_deg, T_ANGLE_PULSE& pulse)
{
    // 6轴机械臂脉冲转换（S/L/U/R/B/T）
    pulse.nSPulse = static_cast<long>(joint_angles_deg[0] / m_tAxisUnit.dSPulseUnit);
    pulse.nLPulse = static_cast<long>(joint_angles_deg[1] / m_tAxisUnit.dLPulseUnit);
    pulse.nUPulse = static_cast<long>(joint_angles_deg[2] / m_tAxisUnit.dUPulseUnit);
    pulse.nRPulse = static_cast<long>(joint_angles_deg[3] / m_tAxisUnit.dRPulseUnit);
    pulse.nBPulse = static_cast<long>(joint_angles_deg[4] / m_tAxisUnit.dBPulseUnit);
    pulse.nTPulse = static_cast<long>(joint_angles_deg[5] / m_tAxisUnit.dTPulseUnit);
    // 外部轴脉冲暂设为0（可根据实际扩展）
    pulse.lBXPulse = 0;
    pulse.lBYPulse = 0;
    pulse.lBZPulse = 0;
}

void RobotDriverAdaptor::LoadRobotKinematicsPara(std::string strRobotName, T_KINEMATICS& tKinematics, T_AXISUNIT& tAxisUnit, T_AXISLIMITANGLE& tAxisLimitAngle)
{
    COPini opini;
    opini.SetFileName(DATA_PATH + strRobotName + ROBOT_PARA_INI);
    opini.SetSectionName("Kinematics");
    opini.ReadString("dA1", &tKinematics.dA1);
    opini.ReadString("dAL1", &tKinematics.dAL1);
    opini.ReadString("dD1", &tKinematics.dD1);
    opini.ReadString("dTH1", &tKinematics.dTH1);

    opini.ReadString("dA2", &tKinematics.dA2);
    opini.ReadString("dAL2", &tKinematics.dAL2);
    opini.ReadString("dD2", &tKinematics.dD2);
    opini.ReadString("dTH2", &tKinematics.dTH2);

    opini.ReadString("dA3", &tKinematics.dA3);
    opini.ReadString("dAL3", &tKinematics.dAL3);
    opini.ReadString("dD3", &tKinematics.dD3);
    opini.ReadString("dTH3", &tKinematics.dTH3);

    opini.ReadString("dA4", &tKinematics.dA4);
    opini.ReadString("dAL4", &tKinematics.dAL4);
    opini.ReadString("dD4", &tKinematics.dD4);
    opini.ReadString("dTH4", &tKinematics.dTH4);

    opini.ReadString("dA5", &tKinematics.dA5);
    opini.ReadString("dAL5", &tKinematics.dAL5);
    opini.ReadString("dD5", &tKinematics.dD5);
    opini.ReadString("dTH5", &tKinematics.dTH5);

    opini.ReadString("dA6", &tKinematics.dA6);
    opini.ReadString("dAL6", &tKinematics.dAL6);
    opini.ReadString("dD6", &tKinematics.dD6);
    opini.ReadString("dTH6", &tKinematics.dTH6);

    double dAngle = 0;
    double dPulse = 0;
    opini.ReadString("dSAngle", &dAngle);
    opini.ReadString("dSPulse", &dPulse);
    tAxisUnit.dSPulseUnit = dAngle / dPulse;
    opini.ReadString("dLAngle", &dAngle);
    opini.ReadString("dLPulse", &dPulse);
    tAxisUnit.dLPulseUnit = dAngle / dPulse;
    opini.ReadString("dUAngle", &dAngle);
    opini.ReadString("dUPulse", &dPulse);
    tAxisUnit.dUPulseUnit = dAngle / dPulse;
    opini.ReadString("dRAngle", &dAngle);
    opini.ReadString("dRPulse", &dPulse);
    tAxisUnit.dRPulseUnit = dAngle / dPulse;
    opini.ReadString("dBAngle", &dAngle);
    opini.ReadString("dBPulse", &dPulse);
    tAxisUnit.dBPulseUnit = dAngle / dPulse;
    opini.ReadString("dTAngle", &dAngle);
    opini.ReadString("dTPulse", &dPulse);
    tAxisUnit.dTPulseUnit = dAngle / dPulse;

    auto readLimitPair = [&opini](const char* maxKey, const char* minKey,
        const char* altMaxKey, const char* altMinKey,
        double& maxValue, double& minValue)
        {
            maxValue = 0.0;
            minValue = 0.0;
            opini.ReadString(maxKey, &maxValue);
            opini.ReadString(minKey, &minValue);
            if (maxValue == 0.0 && minValue == 0.0) {
                opini.ReadString(altMaxKey, &maxValue);
                opini.ReadString(altMinKey, &minValue);
            }
        };

    readLimitPair("dMaxSAngle", "dMinSAngle", "dMaxPosSAngle", "dMaxNegSAngle", tAxisLimitAngle.dMaxSAngle, tAxisLimitAngle.dMinSAngle);
    readLimitPair("dMaxLAngle", "dMinLAngle", "dMaxPosLAngle", "dMaxNegLAngle", tAxisLimitAngle.dMaxLAngle, tAxisLimitAngle.dMinLAngle);
    readLimitPair("dMaxUAngle", "dMinUAngle", "dMaxPosUAngle", "dMaxNegUAngle", tAxisLimitAngle.dMaxUAngle, tAxisLimitAngle.dMinUAngle);
    readLimitPair("dMaxRAngle", "dMinRAngle", "dMaxPosRAngle", "dMaxNegRAngle", tAxisLimitAngle.dMaxRAngle, tAxisLimitAngle.dMinRAngle);
    readLimitPair("dMaxBAngle", "dMinBAngle", "dMaxPosBAngle", "dMaxNegBAngle", tAxisLimitAngle.dMaxBAngle, tAxisLimitAngle.dMinBAngle);
    readLimitPair("dMaxTAngle", "dMinTAngle", "dMaxPosTAngle", "dMaxNegTAngle", tAxisLimitAngle.dMaxTAngle, tAxisLimitAngle.dMinTAngle);
}

void RobotDriverAdaptor::LoadRobotExternalAxlePara(std::string strRobotName)
{
    COPini opini;
    opini.SetFileName(DATA_PATH + strRobotName + ROBOT_PARA_INI);
    opini.SetSectionName("ExternalAxle");

    m_nExternalAxleType = 0;
    opini.ReadString(false, "ExternalAxleType", &m_nExternalAxleType);
    m_nRobotAxisCount = CalculateRobotAxisCountByExternalAxleType(m_nExternalAxleType);

    m_tAxisUnit.dBXPulseUnit = 0.0;
    m_tAxisUnit.dBYPulseUnit = 0.0;
    m_tAxisUnit.dBZPulseUnit = 0.0;
    opini.ReadString(false, "BXPulse", &m_tAxisUnit.dBXPulseUnit);
    opini.ReadString(false, "BYPulse", &m_tAxisUnit.dBYPulseUnit);
    opini.ReadString(false, "BZPulse", &m_tAxisUnit.dBZPulseUnit);
}

int RobotDriverAdaptor::CalculateRobotAxisCountByExternalAxleType(int externalAxleType) const
{
    int externalAxisCount = 0;
    for (int bit = 0; bit < 3; ++bit)
    {
        if ((externalAxleType & (1 << bit)) != 0)
        {
            ++externalAxisCount;
        }
    }
    return 6 + externalAxisCount;
}

bool RobotDriverAdaptor::InitSocket(const char* ip, unsigned short Port, bool ifRecord)
{
    return false;
}

bool RobotDriverAdaptor::CloseSocket()
{
    return true;
}

bool RobotDriverAdaptor::IsConnected()
{
    return false;
}

bool RobotDriverAdaptor::cleanAlarm()
{
    return false;
}

bool RobotDriverAdaptor::ServoOn()
{
    return false;
}

void RobotDriverAdaptor::ClearLastRobotError()
{
    std::lock_guard<std::mutex> lock(m_lastRobotErrorMutex);
    m_sLastRobotError.clear();
}

void RobotDriverAdaptor::SetLastRobotError(const std::string& error)
{
    std::lock_guard<std::mutex> lock(m_lastRobotErrorMutex);
    m_sLastRobotError = error;
}

std::string RobotDriverAdaptor::GetLastRobotError() const
{
    std::lock_guard<std::mutex> lock(m_lastRobotErrorMutex);
    return m_sLastRobotError;
}

std::string RobotDriverAdaptor::GetRobotStatusText()
{
    return GetLastRobotError();
}

std::string RobotDriverAdaptor::GetStateMonitorSourceText() const
{
    return "主动状态接口(GetCurrentPos/GetCurrentPulse/CheckDone)，时间轴=PC steady ms";
}

double RobotDriverAdaptor::GetCurrentPos(int nAxisNo)
{
    return 0;
}

T_ROBOT_COORS RobotDriverAdaptor::GetCurrentPos()
{
    return T_ROBOT_COORS();
}

bool RobotDriverAdaptor::TryGetCurrentPos(T_ROBOT_COORS& pos)
{
    pos = GetCurrentPos();
    const bool finite = std::isfinite(pos.dX) && std::isfinite(pos.dY) && std::isfinite(pos.dZ)
        && std::isfinite(pos.dRX) && std::isfinite(pos.dRY) && std::isfinite(pos.dRZ)
        && std::isfinite(pos.dBX) && std::isfinite(pos.dBY) && std::isfinite(pos.dBZ);
    if (!finite || !IsConnected())
    {
        SetLastRobotError("机器人当前位置读取失败或返回非有限值。");
        pos = T_ROBOT_COORS();
        return false;
    }
    return true;
}

double RobotDriverAdaptor::GetCurrentPulse(int nAxisNo)
{
    return 0;
}
T_ANGLE_PULSE RobotDriverAdaptor::GetCurrentPulse()
{
    return T_ANGLE_PULSE();
}

bool RobotDriverAdaptor::TryGetCurrentPulse(T_ANGLE_PULSE& pulse)
{
    pulse = GetCurrentPulse();
    if (!IsConnected())
    {
        SetLastRobotError("机器人当前关节/脉冲读取失败：机器人未连接。");
        pulse = T_ANGLE_PULSE();
        return false;
    }
    return true;
}

T_ROBOT_COORS RobotDriverAdaptor::GetCurrentPosPassive(long long* pRobotMs, long long* pPcRecvMs)
{
    const long long pcMs = RobotDriverSteadyMs();
    T_ROBOT_COORS pose = GetCurrentPos();
    FillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
    return pose;
}

T_ANGLE_PULSE RobotDriverAdaptor::GetCurrentPulsePassive(long long* pRobotMs, long long* pPcRecvMs)
{
    const long long pcMs = RobotDriverSteadyMs();
    T_ANGLE_PULSE pulse = GetCurrentPulse();
    FillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
    return pulse;
}

int RobotDriverAdaptor::CheckDonePassive(long long* pRobotMs, long long* pPcRecvMs)
{
    const long long pcMs = RobotDriverSteadyMs();
    const int done = CheckDone();
    FillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
    return done;
}

void RobotDriverAdaptor::PrepareStateMonitor()
{
}

bool RobotDriverAdaptor::StartStateMonitor(int intervalMs)
{
    if (intervalMs <= 0)
    {
        intervalMs = 50;
    }

    bool expected = false;
    if (!m_stateMonitorRunning.compare_exchange_strong(expected, true))
    {
        return true;
    }

    PrepareStateMonitor();
    try
    {
        m_stateMonitorThread = std::thread(&RobotDriverAdaptor::StateMonitorWorker, this, intervalMs);
    }
    catch (...)
    {
        m_stateMonitorRunning.store(false);
        if (m_pRobotLog != nullptr)
        {
            m_pRobotLog->write(LogColor::ERR, "机器人状态监控线程启动失败");
        }
        return false;
    }

    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::SUCCESS,
            "机器人状态监控线程已启动，采样间隔=%dms，缓存=%zu帧",
            intervalMs,
            kStateMonitorMaxFrames);
    }
    return true;
}

void RobotDriverAdaptor::StopStateMonitor()
{
    m_stateMonitorRunning.store(false);
    if (m_stateMonitorThread.joinable())
    {
        m_stateMonitorThread.join();
    }
}

bool RobotDriverAdaptor::IsStateMonitorRunning() const
{
    return m_stateMonitorRunning.load();
}

bool RobotDriverAdaptor::LatestStateSnapshot(StateSnapshot& snapshot) const
{
    std::lock_guard<std::mutex> lock(m_stateMonitorMutex);
    if (m_stateMonitorFrames.empty())
    {
        return false;
    }
    snapshot = m_stateMonitorFrames.back();
    return true;
}

std::vector<RobotDriverAdaptor::StateSnapshot> RobotDriverAdaptor::StateSnapshotsBetween(
    std::uint64_t beginExclusive,
    std::uint64_t endInclusive) const
{
    std::vector<StateSnapshot> result;
    std::lock_guard<std::mutex> lock(m_stateMonitorMutex);
    for (const StateSnapshot& frame : m_stateMonitorFrames)
    {
        if (frame.sequence > beginExclusive && frame.sequence <= endInclusive)
        {
            result.push_back(frame);
        }
    }
    return result;
}

std::uint64_t RobotDriverAdaptor::StateMonitorMark() const
{
    std::lock_guard<std::mutex> lock(m_stateMonitorMutex);
    return m_stateMonitorFrames.empty() ? 0 : m_stateMonitorFrames.back().sequence;
}

int RobotDriverAdaptor::StateMonitorCachedCount() const
{
    std::lock_guard<std::mutex> lock(m_stateMonitorMutex);
    return static_cast<int>(m_stateMonitorFrames.size());
}

void RobotDriverAdaptor::StoreStateSnapshot(const StateSnapshot& snapshot)
{
    StateSnapshot stored = snapshot;
    std::lock_guard<std::mutex> lock(m_stateMonitorMutex);
    stored.sequence = ++m_stateMonitorNextSequence;
    m_stateMonitorFrames.push_back(stored);
    while (m_stateMonitorFrames.size() > kStateMonitorMaxFrames)
    {
        m_stateMonitorFrames.pop_front();
    }
}

std::atomic<bool> RobotDriverAdaptor::s_connectDriversAtConstruct{ true };

void RobotDriverAdaptor::StateMonitorWorker(int intervalMs)
{
    // GUI 启动时驱动构造未连接(为避免阻塞主窗口)，在此后台线程发起首次连接(STEP 重写本钩子；
    // FANUC/基类为空)。连接的阻塞/超时落在本线程，不影响 UI。CLI 已在构造内连上，此处幂等空过。
    EnsureConnectionForMonitor();
    while (m_stateMonitorRunning.load())
    {
		// STEP 等驱动在后台探测并恢复纯连接；具体钩子不得启动程序或运动。
		// 每轮调用可刷新 atomic 连接快照，GUI 无需等待可能阻塞的 SDK mutex。
		// FANUC 基类钩子为空，S4/S5 各自按原有策略管理。
		EnsureConnectionForMonitor();
        const long long nowMs = RobotDriverSteadyMs();
        StateSnapshot snapshot;
        snapshot.robotMs = nowMs;
        snapshot.pcRecvMs = nowMs;
        snapshot.done = -1;
        snapshot.valid = false;

        if (IsConnected())
        {
            long long poseRobotMs = 0;
            long long posePcRecvMs = 0;
            long long pulseRobotMs = 0;
            long long pulsePcRecvMs = 0;
            long long doneRobotMs = 0;
            long long donePcRecvMs = 0;

            snapshot.pose = GetCurrentPosPassive(&poseRobotMs, &posePcRecvMs);
            snapshot.pulse = GetCurrentPulsePassive(&pulseRobotMs, &pulsePcRecvMs);
            snapshot.done = CheckDonePassive(&doneRobotMs, &donePcRecvMs);
			const long long validationNowMs = RobotDriverSteadyMs();

			// 位姿样本的两条时间轴必须来自同一次位姿读取。done或本机now只能描述
			// 状态请求，不能补写成pose的robot_ms/pc_recv_ms，否则会混淆扫描纪元。
            snapshot.robotMs = poseRobotMs;
            snapshot.pcRecvMs = posePcRecvMs;
            const bool finitePose = std::isfinite(snapshot.pose.dX)
                && std::isfinite(snapshot.pose.dY) && std::isfinite(snapshot.pose.dZ)
                && std::isfinite(snapshot.pose.dRX) && std::isfinite(snapshot.pose.dRY)
                && std::isfinite(snapshot.pose.dRZ) && std::isfinite(snapshot.pose.dBX)
                && std::isfinite(snapshot.pose.dBY) && std::isfinite(snapshot.pose.dBZ);
            const bool nonZeroPose = std::abs(snapshot.pose.dX) >= 1e-12
                || std::abs(snapshot.pose.dY) >= 1e-12 || std::abs(snapshot.pose.dZ) >= 1e-12
                || std::abs(snapshot.pose.dRX) >= 1e-12 || std::abs(snapshot.pose.dRY) >= 1e-12
                || std::abs(snapshot.pose.dRZ) >= 1e-12 || std::abs(snapshot.pose.dBX) >= 1e-12
                || std::abs(snapshot.pose.dBY) >= 1e-12 || std::abs(snapshot.pose.dBZ) >= 1e-12;
            const long long maxPoseAgeMs = std::max<long long>(500, intervalMs * 4LL);
			// 被动接口在请求完成时记录pc_recv_ms，因此必须用全部读取结束后的时刻
			// 验证新鲜度；读取前的nowMs会把正常STEP帧误判成“来自未来”。
            const bool freshPose = posePcRecvMs > 0 && posePcRecvMs <= validationNowMs
                && (validationNowMs - posePcRecvMs) <= maxPoseAgeMs;
            // valid 只证明“这一帧位姿本身可用于业务”，不能再由 done 的时间戳或
            // 本机 nowMs 代替失败的位姿读取。扫描插值只接受严格、非零且新鲜的pose。
            snapshot.valid = freshPose && finitePose && nonZeroPose;
        }

        StoreStateSnapshot(snapshot);
        const int sleepMs = snapshot.valid ? intervalMs : std::max(intervalMs, 200);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }
}

int RobotDriverAdaptor::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
    return 0;
}

int RobotDriverAdaptor::CheckDone()
{
    return -1;
}

int RobotDriverAdaptor::CheckRobotDone(int nDelayTime, int runTimeoutMs)
{
    (void)nDelayTime;
    (void)runTimeoutMs;
    return CheckDone();
}

bool RobotDriverAdaptor::AbortCurrentProgramSafely()
{
    SetLastRobotError("当前机器人驱动未实现可验证的不可恢复程序中止。");
    return false;
}

bool RobotDriverAdaptor::CallJob(std::string sJobName)
{
    (void)sJobName;
    return false;
}

int RobotDriverAdaptor::InitFtp()
{
    return -1;
}

int RobotDriverAdaptor::UploadFile(std::string LocalFilePath, std::string RemoteFilePath)
{
    (void)LocalFilePath;
    (void)RemoteFilePath;
    return -1;
}

int RobotDriverAdaptor::DownloadFile(std::string RemoteFilePath, std::string LocalFilePath)
{
    (void)RemoteFilePath;
    (void)LocalFilePath;
    return -1;
}

bool RobotDriverAdaptor::SetTpSpeed(int speed)
{
    (void)speed;
    return false;
}

bool RobotDriverAdaptor::GetToolData(int nToolNo, T_ROBOT_COORS& robotToolData)
{
    (void)nToolNo;
    robotToolData = T_ROBOT_COORS();
    SetLastRobotError("当前机器人驱动不支持读取工具坐标。");
    return false;
}

int RobotDriverAdaptor::GetIntVar(int nIndex, const char* cStrPreFix)
{
    (void)nIndex;
    (void)cStrPreFix;
    return 0;
}

bool RobotDriverAdaptor::SetIntVar(int nIndex, int nValue, int score, const char* cStrPreFix)
{
    (void)nIndex;
    (void)nValue;
    (void)score;
    (void)cStrPreFix;
    return false;
}

bool RobotDriverAdaptor::SetIntVar(const char* name, int value, int score)
{
    (void)name;
    (void)value;
    (void)score;
    return false;
}

bool RobotDriverAdaptor::SetRealVar(int nIndex, double value, const char* cStrPreFix, int score)
{
    (void)nIndex;
    (void)value;
    (void)cStrPreFix;
    (void)score;
    return false;
}

int RobotDriverAdaptor::GetPosVar(long lPvarIndex, double array[6], int config[7], int MoveType)
{
    (void)lPvarIndex;
    (void)array;
    (void)config;
    (void)MoveType;
    return -1;
}

bool RobotDriverAdaptor::GetHandEyeMatrixVariable(const char* variableName, double rotation[9], double translation[3], std::string* error)
{
    (void)variableName;
    (void)rotation;
    (void)translation;
    const std::string message = "当前机器人驱动不支持读取手眼矩阵变量。";
    SetLastRobotError(message);
    if (error != nullptr)
    {
        *error = message;
    }
    return false;
}

bool RobotDriverAdaptor::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName, int isconfig, int config[7])
{
    (void)tRobotJointCoord;
    (void)tPulseMove;
    (void)nExternalAxleType;
    (void)JobName;
    (void)isconfig;
    (void)config;
    return false;
}

bool RobotDriverAdaptor::MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName)
{
    (void)tRobotJointCoord;
    (void)tPulseMove;
    (void)nExternalAxleType;
    (void)JobName;
    return false;
}

bool RobotDriverAdaptor::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, std::string JobName, int config[7])
{
    (void)dRobotJointCoord;
    (void)tPulseMove;
    (void)nExternalAxleType;
    (void)nPVarType;
    (void)JobName;
    (void)config;
    return false;
}
