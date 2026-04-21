#include "RobotDriverAdaptor.h"


RobotDriverAdaptor::RobotDriverAdaptor(std::string sRobotName,RobotLog* pRobotLog)
    : m_nExternalAxleType(0),
    m_nRobotAxisCount(6),
    m_pRobotLog(pRobotLog), // 初始化日志：指定路径+控制台输出
    m_pFTP(nullptr)
{
    LoadRobotKinematicsPara(sRobotName, m_tKinematics, m_tAxisUnit, m_tAxisLimitAngle);
    LoadRobotExternalAxlePara(sRobotName);
    CreateFanucChain();
    m_pRobotLog->write(LogColor::SUCCESS, "RobotDriverAdaptor 初始化完成，机器人链创建成功");

}

RobotDriverAdaptor::~RobotDriverAdaptor()
{
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

bool RobotDriverAdaptor::InitSocket(const char* ip, u_short Port, bool ifRecord)
{
    return false;
}

bool RobotDriverAdaptor::CloseSocket()
{
    return true;
}

double RobotDriverAdaptor::GetCurrentPos(int nAxisNo)
{
    return 0;
}

T_ROBOT_COORS RobotDriverAdaptor::GetCurrentPos()
{
    return T_ROBOT_COORS();
}

double RobotDriverAdaptor::GetCurrentPulse(int nAxisNo)
{
    return 0;
}
T_ANGLE_PULSE RobotDriverAdaptor::GetCurrentPulse()
{
    return T_ANGLE_PULSE();
}

int RobotDriverAdaptor::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
    return 0;
}
