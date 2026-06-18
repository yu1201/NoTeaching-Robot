#pragma once

#include "RobotCalculation.h"
#include "WorkpieceMeshBuilder.h"

#include <QString>
#include <QVector>

#include <functional>

// 用 BCPD（外部 bcpd.exe，相似+非刚性变换 T(y)=sR(y+v)+t）把参考模型非刚性形变贴合到
// 新工件点云：全局尺度 s 吸收「新工件尺寸略大/略小」，逐点位移 v 吸收「波纹波形/相位/
// 幅度差异」。输出形变后的模型网格（顶点更新、三角拓扑不变），供后续「点到面距离去噪」
// 当作贴合新工件的真值表面。
//
// 安全语义：本步只产出「贴合点云的参考表面」，绝不改动点云本身——是否删点、删哪些，
// 完全由后一步 PointCloudModelDenoiser 按距离阈值决定，真实焊缝几何不会被模型带偏。
//
// 集成方式：进程隔离调用 SDK/BCPD/bcpd.exe（MIT，自包含单文件，无 dll 依赖），失败/
// 超时返回 false，调用方据此回退到「不去噪」或「仅统计去噪」，绝不中断主流程。
class BcpdModelAligner
{
public:
    struct Options
    {
        // bcpd.exe 路径；空 = exe 所在目录下的 SDK/BCPD/bcpd.exe。
        QString bcpdExePath;
        // 点云（target）降采样上限，控制 BCPD 规模（0 = 不降采样）。配准只需表面分布，
        // 降采样后仍能定形；去噪用的是全分辨率点云，不受此影响。
        int targetSampleCap = 20000;
        double omega = 0.1;        // -w 外点比例 (0,1)，点云有噪声故留一定容忍。
        double lambda = 0.0;       // -l 位移期望长度正则；0 = 用 bcpd 默认。
        double beta = 0.0;         // -b 平滑核宽，越大形变越平滑、越不会拟合噪声；0 = 默认。
        bool useAcceleration = true;  // -A 加速（仅当模型顶点数 > 70 时实际启用）。
        int timeoutMs = 180000;    // bcpd 进程超时（ms）。
    };

    using LogCallback = std::function<void(const QString&)>;

    // 把 model 非刚性形变贴合到 cloud，结果写入 outDeformedMesh（= model 拓扑 + 形变后顶点）。
    // 成功返回 true；exe 缺失/启动失败/超时/输出异常/顶点数不符均返回 false（调用方应回退）。
    static bool DeformModelToCloud(
        const QVector<RobotCalculation::IndexedPoint3D>& cloud,
        const WorkpieceMeshBuilder::Mesh& model,
        WorkpieceMeshBuilder::Mesh& outDeformedMesh,
        const Options& options = Options(),
        const LogCallback& log = LogCallback());

    // bcpd.exe 是否就位（供调用方提前判断、决定是否启用本功能）。
    static bool IsAvailable(const QString& bcpdExePath = QString());

    // 解析默认 exe 路径（exe 目录/SDK/BCPD/bcpd.exe）。
    static QString DefaultExePath();
};
