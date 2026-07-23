#pragma once

#include <Eigen/Core>

#include <QString>
#include <QStringList>
#include <QVector>
#include <QtGlobal>

#include <memory>
#include <vector>

class TopoDS_Shape;

// 机器人 CAD 装配读取器。
//
// 默认详细模式直接保留 STEP/BRep 几何，不做替代网格或点云转换；碰撞简模
// 生成可选用 bounds-only 轻量结果。读取时仅从 XCAF 装配树中选取产品名
//（或实例名）末尾为 J0...J6 的七个关节组件；工作空间、工装等其它组件
// 不会因为体积或排列顺序被误加入。
class RobotCadAssemblyLoader
{
public:
    struct Bounds
    {
        bool valid = false;
        Eigen::Vector3d minimumMm = Eigen::Vector3d::Zero();
        Eigen::Vector3d maximumMm = Eigen::Vector3d::Zero();
    };

    struct ComponentStatistics
    {
        QString assemblyPath;
        QString instanceName;
        QString productName;
        int jointIndex = -1;
        bool isPipeline = false;
        bool included = false;
        qsizetype solidCount = 0;
        qsizetype faceCount = 0;
        Bounds boundsMm;
    };

    struct BaseGeometry
    {
        bool valid = false;

        // 当前支持的厂商机器人 STEP 约定：+Y 从 J0 基座指向机器人上方。
        // 场景层可据此把 +Y 旋转到应用的 +Z，不在读取器中改写源模型坐标。
        Eigen::Vector3d sourceUp = Eigen::Vector3d::UnitY();
        Bounds j0BoundsMm;

        // 基于 J0 包围盒下沿的保守落地点。它不是精加工安装面的拟合结果；
        // 用于首次显示和防止模型穿过地面，后续可由场景层替换为标定安装面。
        double minimumYmm = 0.0;
        Eigen::Vector3d conservativeBaseCenterMm = Eigen::Vector3d::Zero();
    };

    struct Statistics
    {
        QString sourceSha256;
        QStringList sourceLengthUnits;
        QString productNameEncoding;
        QString occtVersion;
        qsizetype freeShapeCount = 0;
        qsizetype discoveredComponentCount = 0;
        qsizetype includedComponentCount = 0;
        qsizetype jointComponentCount = 0;
        qsizetype includedPipelineCount = 0;
        qsizetype solidCount = 0;
        qsizetype faceCount = 0;
        // true 表示 Result 同时保留了可供详细外显使用的原始 B-Rep 总装、
        // J0 和渐进显示块；false 表示本次仅生成碰撞简模所需的轻量统计。
        bool detailedPresentationBuilt = false;
        bool displayTriangulationPrepared = false;
        qsizetype displayBlockCount = 0;
        Bounds assemblyBoundsMm;
        QVector<ComponentStatistics> components;
    };

    struct Options
    {
        bool includePipeline = false;
        // false 时只读取 J0...J6 的身份、单位、SHA、组件/总装边界和基座，
        // 不构造或保留用于详细外显的总装 B-Rep、J0 B-Rep 和显示块，也不网格化。
        bool buildDetailedPresentation = true;
        // 仍然返回原始 B-Rep；这里只在后台预生成 OCCT 显示缓存，避免 2.6 万
        // 曲面的首次 AIS 显示阻塞 GUI 线程。
        bool prepareDisplayTriangulation = true;
        // GUI 逐块建立 AIS presentation；块内仍是 STEP 读取所得的原始
        // B-Rep face，不生成或保存替代网格文件。
        qsizetype displayBlockTargetFaceCount = 200;
        qint64 maximumFileBytes = 512LL * 1024LL * 1024LL;
    };

    struct Result
    {
        // J0...J6（以及按选项识别出的管线包）的合并 BRep compound。
        // 使用 shared_ptr 仅为让公共头保持 OCCT 无关；对象本身仍是原始 B-Rep，
        // 不会在接口层转换成点云或持久化网格。buildDetailedPresentation=false
        // 时这三个详细外显字段有意保持为空。
        std::shared_ptr<TopoDS_Shape> assemblyShape;
        std::shared_ptr<TopoDS_Shape> j0Shape;
        std::vector<std::shared_ptr<TopoDS_Shape>> displayBlocks;
        BaseGeometry base;
        Statistics statistics;
    };

    // 成功时 result 完整替换；失败时 result 被清空，并返回可直接显示的中文错误。
    // 该函数与 StepModelImporter/CadModel3DView 共用 OCCT 操作锁。
    static bool LoadFile(
        const QString& stepFilePath,
        Result& result,
        QString& error,
        const Options* options = nullptr);
};
