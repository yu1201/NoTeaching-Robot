#pragma once

#include "RobotCalculation.h"

#include <Eigen/Dense>

#include <QImage>
#include <QString>
#include <QVector>

#include <functional>

// 工件完整点云的轻量逆向建模：有序扫描线点云 → 三角网格模型。
//
// 数据前提：_WorkpieceCloud.txt 是逐帧激光扫描线（帧内横向有序、帧间沿焊缝推进）的
// 近似 2.5D 表面。利用有序性做相邻扫描线条带三角化（O(N) 索引拼接，参考
// PCL OrganizedFastMesh 的算法本质），不需要散乱点云的通用重建（KD树/法向估计/全局求解）。
//
// 模型缓存：生成一次二进制 PLY（顶点+法线+三角索引），之后查看器/高度图/CloudCompare
// 直接读缓存秒开，不再解析 150MB 级文本点云。
class WorkpieceMeshBuilder
{
public:
    struct Mesh
    {
        // 顶点与法线一一对应；indices 每 3 个为一个三角形（统一逆时针绕序，法线朝上）。
        QVector<Eigen::Vector3f> vertices;
        QVector<Eigen::Vector3f> normals;
        QVector<quint32> indices;

        bool IsValid() const { return vertices.size() >= 3 && indices.size() >= 3; }
    };

    // 工件网格模型缓存文件名（LaserPoint 目录下，二进制 PLY，可直接拖入 CloudCompare）。
    static QString MeshCacheFileName();
    static QString MeshCachePath(const QString& laserDir);
    // 缓存是否存在且为当前网格化算法版本生成（旧版本缓存视为无效，需重建）。
    static bool IsMeshCacheValid(const QString& filePath);

    // 有序扫描线点云 → 三角网格。targetMaxTriangles 控制行列抽稀步长（保持波纹细节的
    // 前提下限制规模，0 表示不限制满分辨率）。
    static bool BuildFromScanlineCloud(
        const QVector<RobotCalculation::IndexedPoint3D>& cloudPoints,
        Mesh& mesh,
        QString& error,
        int targetMaxTriangles = 1500000);

    // 二进制 PLY 读写（little-endian；写出格式同时兼容 CloudCompare/MeshLab）。
    static bool SaveMeshPly(const QString& filePath, const Mesh& mesh, QString& error);
    static bool LoadMeshPly(const QString& filePath, Mesh& mesh, QString& error);

    // 二进制 STL 导出（面法线；SolidWorks/UG 等 CAD 软件可直接导入）。
    static bool SaveMeshStl(const QString& filePath, const Mesh& mesh, QString& error);

    // 进度回调：percent 0~100，stage 阶段文案；返回 false 表示用户取消。
    // 可在任意线程调用（生成流程设计为后台线程跑，回调内自行投递回 UI）。
    using ProgressCallback = std::function<bool(int percent, const QString& stage)>;

    // 带进度的完整点云读取（index x y z 文本，手写解析，比正则快一个量级），
    // 进度按已读字节/总字节计。取消时返回 false 且 error 为"已取消"。
    static bool LoadCloudPointsWithProgress(
        const QString& cloudFilePath,
        QVector<RobotCalculation::IndexedPoint3D>& points,
        QString& error,
        const ProgressCallback& progress = ProgressCallback());

    // 缓存保障：缓存有效直接返回 true；否则从完整点云文件生成并落盘。
    // cloudFilePath 为 _WorkpieceCloud.txt 全路径；生成失败/取消返回 false（error 带原因）。
    // 阶段划分：0~80% 读取点云，80~95% 网格化，95~100% 写出缓存。
    static bool EnsureMeshCache(
        const QString& laserDir,
        const QString& cloudFilePath,
        QString& error,
        const ProgressCallback& progress = ProgressCallback());

    // 2.5D 高度图渲染：网格顶点 XY 栅格化高度场 → 梯度光照 + 伪彩（工业线扫标准俯视图）。
    // maxImageWidth 限制输出图宽（高按 XY 纵横比定）。
    static QImage RenderHeightMap(const Mesh& mesh, int maxImageWidth = 1600);
};
