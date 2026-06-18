#pragma once

#include <QString>

// 模型配准 + 点到模型去噪的一组参数。
struct ModelAlignmentParams
{
    QString referenceModel;             // 绑定的基准模型名（ReferenceModelLibrary 中的条目）
    double omega = 0.1;                 // BCPD -w 外点比例 (0,1)
    double lambda = 0.0;                // BCPD -l 位移期望长度正则；0 = bcpd 默认
    double beta = 0.0;                  // BCPD -b 平滑核宽；0 = bcpd 默认
    int targetSampleCap = 20000;        // 点云配准降采样上限（去噪仍用全分辨率）
    bool useAcceleration = true;        // BCPD -A（仅模型顶点 > 70 时实际生效）
    double distanceThresholdMm = 3.0;   // 去噪：点到模型面距离 > 此值判噪声
    bool keepPointsWithoutSurface = true;  // 去噪：邻域无模型面参照的点是否保留
};

// 参数组持久化（ConfigStore.db，scope=global）：支持多组保存、切换、增删，记住「当前组」。
// 元信息存 module="ModelAlignment"（GroupCount / CurrentGroup / Group<i>Name）；
// 每组参数存 module="ModelAlignment/Group<i>"。
class ModelAlignmentConfig
{
public:
    static int GroupCount();                  // 至少 1（为空时自动建默认组）
    static int CurrentGroupIndex();           // 夹在 [0, GroupCount-1]
    static void SetCurrentGroupIndex(int index);
    static QString GroupName(int index);
    static ModelAlignmentParams LoadGroup(int index);
    static void SaveGroup(int index, const QString& name, const ModelAlignmentParams& p);
    static int AddGroup(const QString& name, const ModelAlignmentParams& p);  // 返回新组索引
    static bool DeleteGroup(int index);       // 至少保留 1 组，失败返回 false
};
