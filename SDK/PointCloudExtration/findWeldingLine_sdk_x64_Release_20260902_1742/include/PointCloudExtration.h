#pragma once

// DLL 导入/导出宏：
// - 编译 PointCloudExtration.dll 时，在 CMake 中定义 POINTCLOUDEXTRATION_EXPORTS，
//   函数符号使用 __declspec(dllexport) 导出。
// - 外部工程包含本头文件调用 DLL 时，定义 POINTCLOUDEXTRATION_IMPORTS，函数符号使用
//   __declspec(dllimport) 导入。
// - 非 Windows 平台不需要 __declspec，宏展开为空。
#if defined(_WIN32) && defined(POINTCLOUDEXTRATION_EXPORTS)
#define POINTCLOUDEXTRATION_API __declspec(dllexport)
#elif defined(_WIN32) && defined(POINTCLOUDEXTRATION_IMPORTS)
#define POINTCLOUDEXTRATION_API __declspec(dllimport)
#else
#define POINTCLOUDEXTRATION_API
#endif

// 原 DLL ABI 使用的三维点结构。
// 注意：这里必须保持 3 个 double 的内存布局，调用方和 DLL 之间直接传递数组。
typedef struct Point_3D
{
    double x;  // X 坐标，单位沿用输入点云文件/调用方坐标系。
    double y;  // Y 坐标，单位沿用输入点云文件/调用方坐标系。
    double z;  // Z 坐标，单位沿用输入点云文件/调用方坐标系。
} Point_3D;

// 轨迹点类型。
// 原始 IDA 伪码最后输出数组的第 4 个字段是 int 类型标记，这里显式固定枚举底层
// 类型为 int，保证 TrackPointsPosition 的二进制布局稳定。
enum class PointType : int
{
    Start = 0,   // 起点。
    End = 1,     // 终点。
    Corner = 2,  // 中间拐点/关键点。
};

// CorrugatedSheetPointCloudExtration 返回的轨迹点结构。
// 每个元素包含一个三维坐标和一个点类型。调用方使用完数组后必须调用
// ReleaseTrackPoints 释放，不能用 free 或跨模块 delete[] 自行释放。
typedef struct TrackPointsPosition
{
    Point_3D pt;      // 轨迹点坐标。
    PointType type;   // 点类型：Start / End / Corner。
} TrackPointsPosition;

// 原头文件中的焊缝信息结构，当前重构主流程主要返回 TrackPointsPosition。
// 保留该结构是为了兼容旧调用方和 ReleaseWeldingLines 接口。
typedef struct WeldingLine
{
    int IsArc;                    // 焊缝类型：原注释中 0=直线，1/2=圆弧，10/666/120 等为特殊状态。
    bool isClockwise;             // 圆弧方向标记，true 表示顺时针。
    double ZSide;                 // Z 侧补偿/方向值，具体含义由原调用方定义。
    bool isLeft;                  // 左右侧标记。
    Point_3D CenterPoint;         // 圆弧中心或局部几何中心。
    Point_3D StartPoint;          // 焊缝起点。
    Point_3D EndPoint;            // 焊缝终点。
    Point_3D StartNormalVector;   // 起点法向量。
    Point_3D EndNormalVector;     // 终点法向量。
    int StartPointType;           // 起点坡口/端点类型。
    int EndPointType;             // 终点坡口/端点类型。
    double BoardThickness;        // 板厚。
    double UpperDegeWidth;        // 上边缘宽度，字段名保持原头文件拼写。
} WeldingLine;

// 主提取接口：
// 输入点云 -> 过滤/聚类/法向筛选 -> 局部坐标对齐 -> 骨架提取 -> 输出轨迹点。
//
// 返回值：
// - 成功：返回 new[] 分配的 TrackPointsPosition 数组，并通过 TrackPointsNumber
//   返回元素个数。
// - 失败：返回 nullptr，TrackPointsNumber 为 0 或 -1。
//
// 内存约定：
// - 返回数组必须用 ReleaseTrackPoints(&ptr) 释放。
// - terminal 输出轨迹最后一个点，便于调用方直接获取终点。
POINTCLOUDEXTRATION_API TrackPointsPosition* CorrugatedSheetPointCloudExtration(
    Point_3D* Input_pointcloud,   // 输入点云数组。
    int Input_pointsNumber,       // 输入点数量。
    int* TrackPointsNumber,       // 输出轨迹点数量。
    Point_3D& terminal,           // 输出终点。
    double Z_Truncation_Value,    // Z 截断兜底值；优先使用配置文件 Remove_Floor_ZValue。
    Point_3D ScanDirection,       // 扫描/行进方向，用于统一轨迹顺序。
    const char* config_path);     // 配置文件路径，可为空。

// 释放由 DLL 分配的 Point_3D 数组，保留为兼容旧接口。
POINTCLOUDEXTRATION_API void Release3DPoints(Point_3D** Input_Pointer);

// 释放 CorrugatedSheetPointCloudExtration 返回的轨迹点数组。
POINTCLOUDEXTRATION_API void ReleaseTrackPoints(TrackPointsPosition** Input_Pointer);

// 释放由旧焊缝接口分配的 WeldingLine 数组，当前保留为 ABI 兼容。
POINTCLOUDEXTRATION_API void ReleaseWeldingLines(WeldingLine** Input_Pointer);
