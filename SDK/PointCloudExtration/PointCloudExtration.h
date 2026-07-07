#pragma once

//定义表示空间三维点的结构体
typedef struct Point_3D
{
	double x;
	double y;
	double z;
}Point_3D;

enum class PointType
{
	Start,      // 起点
	End,        // 终点
	Corner,     // 拐点
};

//定义表示波纹板轨迹点的结构体
typedef struct TrackPointsPosition
{
	Point_3D pt; //轨迹点三维坐标
	PointType type; //轨迹点属性
}TrackPointsPosition;


//定义表示焊缝信息的结构体
typedef struct WeldingLine
{
	int IsArc; //0表示直板，1表示不闭合弧板，2表示闭合弧板,10表示斜板焊缝,666表示檩托焊缝,120表示未提取出的檩托位置焊缝
	bool isClockwise;
	double ZSide;
	bool isLeft;
	Point_3D CenterPoint;
	Point_3D StartPoint; //起点
	Point_3D EndPoint;  //终点
	Point_3D StartNormalVector; //起点法向
	Point_3D EndNormalVector; //终点法向
	int StartPointType; //起点端头类型
	int EndPointType;  //终点端头类型
	double BoardThickness; //所属立板厚度
	double UpperDegeWidth; //上沿宽度
}WeldingLine;


//定义提取波纹板点云中轨迹点信息结构体的函数
__declspec(dllexport) TrackPointsPosition* CorrugatedSheetPointCloudExtration(Point_3D* Input_pointcloud, int Input_pointsNumber, int* TrackPointsNumber, Point_3D& terminal, double Z_Truncation_Value, Point_3D ScanDirection, const char* config_path);


// 定义释放三维点的函数
__declspec(dllexport) void Release3DPoints(Point_3D** Input_Pointer);


// 定义释放轨迹点的函数
__declspec(dllexport) void ReleaseTrackPoints(TrackPointsPosition** Input_Pointer);


// 定义释放焊缝信息结构体的函数
__declspec(dllexport) void ReleaseWeldingLines(WeldingLine** Input_Pointer);