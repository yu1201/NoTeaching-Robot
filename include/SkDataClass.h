#ifndef SKDATACLASS_H
#define SKDATACLASS_H


#include "qglobal.h"
#include "vector"
#include "deque"
#include "QDebug"
#include "Eigen/Dense"
#include "SkGrooveRecog_global.h"

using namespace Eigen;
using namespace std;

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef TRUE
    #define TRUE 1
#endif

enum {API_FALSE, API_TRUE};

#define ReadyStage      0        //准备状态
#define SearchStage     1        //搜索状态
#define TrackStage      2        //跟踪状态
#define ScanStage       3        //扫描状态

#define RealTrackMode   0        //实时跟踪
#define LagTrackMode    1        //滞后跟踪

#define OffsetOutMode   0        //偏差输出模式
#define PosOutMode      1        //位置输出模式

#define OnLineTrack     0        //在线跟踪
#define ReplayTrack     1        //回放跟踪

#define Manipulator     0        //操作工
#define Supervisor      1        //管理者
#define Factory         2        //厂家

#define MPFrame         0        //相机机械坐标系
#define OPFrame         1        //相机激光系

const double PI = 3.14159265358979323846;
const double EPS = 0.0000001;

#define MIN_LASER_VAL  -1000
#define MAX_LASER_VAL   1000

class SkVector6d;
class SkVector9d;
class SkVector3d;
class SkPoint3D;
//变换矩阵类
class SKGROOVERECOG_EXPORT SkMatrix4d
{
public:
    union
    {
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> mT;
        double m[4][4];
        struct
        {
            double nx, ox, ax, px, ny, oy, ay, py, nz, oz, az, pz, n3, o3, a3, p3;
        };
    };
    SkMatrix4d();  //单位化
    SkMatrix4d(const SkMatrix4d &mTrans);
    void init();
    int isZeroT();
    int isOneT();
    SkMatrix4d unit();
    SkMatrix4d inv();
    void print();
    SkMatrix4d operator*(SkMatrix4d T);
    SkMatrix4d operator+(SkVector6d jA);
    SkMatrix4d operator-(SkVector6d jA);
    SkVector6d operator*(SkVector6d jA);
    SkVector9d operator*(SkVector9d jA);
    SkVector3d operator*(SkVector3d vA);
    SkPoint3D  operator*(SkPoint3D pA);
    SkMatrix4d operator=(SkMatrix4d T);
};

//关节类
class SKGROOVERECOG_EXPORT SkVector6d
{
public:
    union
    {
        Eigen::Matrix<double, 4, 1> mJ;
        double v[6];
        struct
        {
            double x, y, z, a, b, c;
        };
    };
    SkVector6d();
    SkVector6d(const SkVector6d &mJoint);
    void init();
    int isZeroJ();
    void print();
    SkVector6d operator+(SkVector6d jA);
    SkVector6d operator-(SkVector6d jA);
    SkVector6d operator*(SkVector6d jA);
    SkVector6d operator%(SkVector6d jA);
    SkVector6d operator*(int n);
    SkVector6d operator*(double k);
    SkVector6d operator/(int n);
    SkVector6d operator/(double k);
    //friend SkVector6d operator*(SkMatrix4d mT, SkVector6d jA);
    SkVector6d operator=(SkVector6d jA);

};

//三维坐标类
class SKGROOVERECOG_EXPORT SkVector3d
{
public:
    union
    {
        Eigen::Vector3d mV;
        double v[3];
        struct
        {
            double x, y, z;
        };
    };
    SkVector3d();
    SkVector3d(const SkVector3d &mVector);
    void init();
    int isZeroV();
    SkVector3d unit();
    void print();
    SkVector3d operator+(SkVector3d vA);
    SkVector3d operator-(SkVector3d vA);
    double   operator*(SkVector3d vA);
    SkVector3d operator*(double k);
    SkVector3d operator*(int n);
    SkVector3d operator/(double k);
    SkVector3d operator/(int n);
    SkVector3d operator%(SkVector3d vA);
    friend SkVector3d SkMatrix4d::operator*(SkVector3d vA);
    //friend SkVector3d SkMatrix4d::operator%(SkVector3d vA);
    SkVector3d operator=(SkVector3d vA);
};

//扩展的关节类
class SKGROOVERECOG_EXPORT SkVector9d
{
public:
    union
    {
        Eigen::Matrix<double, 9, 1> mJ;
        double v[9];
        struct
        {
            double x, y, z, a, b, c, e1, e2, e3;
        };
    };
    SkVector9d();
    SkVector9d(const SkVector9d &mExtebdJoint);
    void init();
    int isZeroJ();
    void print();
    SkVector9d operator+(SkVector9d jA);
    SkVector9d operator-(SkVector9d jA);
    SkVector9d operator*(SkVector9d jA);
    SkVector9d operator%(SkVector9d jA);
    SkVector9d operator/(int n);
    SkVector9d operator/(double k);
    //friend SkVector9d operator*(SkMatrix4d T, SkVector9d jA);
    SkVector9d operator=(SkVector9d jA);
};

//三维点类
class SKGROOVERECOG_EXPORT SkPoint3D
{
public:
    union
    {
        Eigen::Vector3d mP;
        double p[3];
        struct
        {
            double x, y, z;
        };
    };
    int order;
    double dis;
    double angle;

    SkPoint3D();
    SkPoint3D(const SkPoint3D &pA);
    SkPoint3D(double xA, double yA, double zA);
    SkPoint3D(double xA, double yA, double zA, int orderA);
    SkPoint3D(double xA, double yA, double zA, int orderA, double disA, double angleA);
    void init();
    int isZero();
    void print();
    SkPoint3D operator+(SkPoint3D pA);
    SkPoint3D operator-(SkPoint3D pA);
    SkPoint3D operator*(int n);
    SkPoint3D operator*(double k);
    SkPoint3D operator/(int n);
    SkPoint3D operator/(double k);
    SkPoint3D operator=(SkPoint3D pA);
};

//二维点类
class SKGROOVERECOG_EXPORT SkPoint2D
{
public:
    union
    {
        Eigen::Vector2d mP;
        double p[2];
        struct
        {
            double y, z;
        };
    };
    int order;
    double dis;
    double angle;
    SkPoint2D();
    SkPoint2D(const SkPoint2D &pA);
    SkPoint2D(double yA, double zA);
    SkPoint2D(double yA, double zA, int orderA);
    SkPoint2D(double yA, double zA, int orderA, double disA, double angleA);
    void init();
    int isZero();
    void print();
    SkPoint2D operator+(SkPoint2D pA);
    SkPoint2D operator-(SkPoint2D pA);
    SkPoint2D operator*(int n);
    SkPoint2D operator*(double k);
    SkPoint2D operator/(int n);
    SkPoint2D operator/(double k);
    SkPoint2D operator=(SkPoint2D pA);
};

//直线类
class SKGROOVERECOG_EXPORT SkLine
{
public:
    SkLine();
    SkLine(double m, double n);
    double k;
    double b;
};

//焊缝输出类
class SKGROOVERECOG_EXPORT SkContourInfo
{
public:
    SkContourInfo();
    void init();
    SkContourInfo operator+(SkContourInfo cA);
    SkContourInfo operator-(SkContourInfo cA);
    SkContourInfo operator*(int n);
    SkContourInfo operator*(double k);
    SkContourInfo operator/(int n);
    SkContourInfo operator/(double k);
public:
    int stage;                          //传感器所处状态
    int coordinate;                     //坐标系:       0=相机系   1=激光系
    int order;                          //帧序号
    quint64 timestamp;                   //数据时间戳
    double area;                         //坡口面积
    double gap;                          //坡口间隙
    double mismatch;                     //坡口错边
    char errInfo[150];                  //错误信息
    short errId;                        //错误码
    int result;                         //识别结果        0:无法识别  1：识别
    SkVector6d mp;                      //坡口相机系坐标
    SkVector6d op;                      //坡口激光系坐标
    std::vector<SkPoint3D> bmp;         //分割点相机系坐标
    std::vector<SkPoint3D> bop;         //分割点激光系坐标
    std::vector<SkLine> line;           //坡口平面
    std::vector<int> iPara;             //备用的整形参数输出
    std::vector<double> dPara;          //备用的浮点型参数输出
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//内角接坡口参数类
class SKGROOVERECOG_EXPORT CornerGroove
{
public:
    CornerGroove();
    void init();
    double leftMatchThre;              //配准阈值               默认=0.5
    double rightMatchThre;             //配准阈值               默认=0.5
    double leftSlopeAngleMin;          //左坡口最小角度(度)        默认=-80
    double leftSlopeAngleMax;          //左坡口最大角度(度)        默认=-10
    double rightSlopeAngleMin;         //右坡口最小角度(度)        默认=10
    double rightSlopeAngleMax;         //右坡口最大角度(度)        默认=80
    double includedAngleMin;           //最小夹角(度)            默认=30
    double includedAngleMax;           //最大夹角(度)            默认=150
    double leftLengthMin;              //左板最小长度(mm)         默认=10
    double leftLengthMax;              //右板最大长度(mm)         默认=200
    double rightLengthMin;             //右板最小长度(mm)         默认=10
    double rightLengthMax;             //右板最大长度(mm)         默认=200
    double leftHeightMin;              //左板最小高度(mm)         默认=5
    double leftHeightMax;              //左板最大高度(mm)         默认=200
    double rightHeightMin;             //右板最小高度(mm)         默认=5
    double rightHeightMax;             //右板最大高度(mm)         默认=200
    int plateType;                     //板材左右高低类型         0:左在右上  1:右在左上           默认=0
    int priority;                      //拟合优先级              0:左侧优先  1：右侧优先          默认=0
    int trackPos;                      //跟踪位置                0:交点     1:间隙中心           默认=0
    double gapMaxBot;                  //底部最大间隙             默认=30
    double gapMaxInPlane;              //面内最大间隙             默认=10
    int disNumMax;                     //最大干扰点               默认=500
    int planeNumMin;                   //面最小点数               默认=50
    int normalMethod;                  //法线方向                0:角平分线 1:左板  2:右板        默认=0
    int breakFootEnable;               //分割点投影              0:投影   1:不投影
};

//V型坡口参数类
class SKGROOVERECOG_EXPORT VeeFormGroove
{
public:
    VeeFormGroove();
    void init();
    double planeMatchThre;              //平面配准阈值(mm)        默认=0.5
    double slopeMatchThre;              //斜面配准阈值(mm)        默认=0.5
    double planeDisMatchThre;           //平面偏离阈值(mm)        默认=0.5
    double planeAngleMin;              //板材最小角度(度)         默认=-20
    double planeAngleMax;              //板材最大角度(度)         默认=20
    double leftSlopeAngleMin;          //左斜面最小角度(度)       默认=-80
    double leftSlopeAngleMax;           //右坡口最大角度(度)      默认=-30
    double rightSlopeAngleMin;          //左坡口最小角度(度)      默认=30
    double rightSlopeAngleMax;          //右坡口最大角度(度)      默认=80
    double leftHeightMin;              //左板最小高度(mm)         默认=5
    double leftHeightMax;              //左板最大高度(mm)         默认=200
    double rightHeightMin;             //右板最小高度(mm)         默认=5
    double rightHeightMax;             //右板最大高度(mm)         默认=200
    double widthMin;                   //坡口最小宽度(mm)         默认=5
    double widthMax;                   //坡口最大宽度(mm)         默认=100
    double mismatchMax;                //最大错边(mm)            默认=10
    int trackPos;                       //跟踪位置               0:斜面交点   1:顶部中心  2:底部中心        默认=0
    int disNumMax;                      //最大干扰点              默认=500
    int planeNumMin;                    //面最小点数              默认=50
    double includedAngleMin;            //最小夹角(度)            默认=30
    double includedAngleMax;            //最大夹角(度)            默认=150
    double gapMaxInPlane;               //面内最大间隙(mm)         默认=10
    int plateType;                     //板材左右高低类型          0:左在右上  1:右在左上                 默认=0
    int slopeAngleEnable;               //斜面角度使能             0:禁止   1:启用                       默认=0
    double leftSlopeAngleIn;            //输入左斜面角度(度)        默认=-60
    double rightSlopeAngleIn;           //输入右斜面角度(度)        默认=60
    int thickEnable;                   //输入高度使能             0:禁止   1:启用                       默认=0
    double leftThicknessIn;               //输入左板厚度(mm)         默认=10
    double rightThicknessIn;              //输入右板厚度(mm)         默认=10
    int breakFootEnable;               //分割点投影              0:投影   1:不投影
};

//半V型坡口参数类
class SKGROOVERECOG_EXPORT SemiVeeGroove
{
public:
    SemiVeeGroove();
    void init();
    double matchThre;                //平面配准阈值(mm)             默认=0.5
    double smallSlopeMin;            //小斜面最小角度               默认=-85
    double smallSlopeMax;            //小斜面最大角度               默认=-30
    double bigSlopeMin;              //大斜面最小角度               默认=30
    double bigSlopeMax;              //大斜面最大角度               默认=85
    double includedAngleMin;         //最小夹角                    默认=30
    double includedAngleMax;         //最大夹角                    默认=90
    double topGapMin;              //坡口最小宽度(mm)              默认=3
    double topGapMax;              //坡口最大宽度(mm)              默认=50
    double thicknessMin;             //板材最小厚度(mm)             默认=5
    double thicknessMax;             //板材最大厚度(mm)             默认=30
    double planeAngleMin;            //板材平面最小角度(度)          默认=-20
    double planeAngleMax;            //板材平面最大角度(度)          默认=20
    double gapMaxInPlane;             //面内最大间隙(mm)            默认=10
    int disNumMax;                   //最大干扰点                  默认=500
    int planeNumMin;                 //面最小点数                  默认=50
    int plateType;                   //板材类型                    0=左V右斜面  1=右V左斜面                                       默认=0
    int trackPos;                    //跟踪位置                    0:斜面交点 1:顶部中点 2:底部中点(理论厚度) 3:实际轮廓底部           默认=0
    int normalMethod;                //法线                       0:上表面 1:小斜面  2:斜面平分线   3:大斜面                       默认=0
    int gapMethod;                   //间隙                       0:顶部间隙  1:底部间隙  2:底部间隙(理论厚度)                      默认=0
    int breakFootEnable;               //分割点投影              0:投影   1:不投影
};

//搭接坡口参数类
class SKGROOVERECOG_EXPORT LapGroove
{
public:
    LapGroove();
    void init();
    double matchThre;                    //平面配准阈值(mm)          默认=0.5
    double planeAngleMin;                //板材最小角度(度)          默认=-30
    double planeAngleMax;                //板材最大角度(度)          默认=30
    double lapAngleMin;                  //最小搭接角度(度)          默认=50
    double lapAngleMax;                  //最大搭接角度(度)          默认=130
    double thicknessMin;                 //最小厚度(mm)             默认=5
    double thicknessMax;                 //最大厚度(mm)             默认=15
    double gapMaxInPlane;                //面内最大间隙(mm)           默认=10默认=0
    double sepMaxDis;                    //面间最大间隔(mm)           默认=20
    int planeNumMin;                     //面最小点数                默认=50
    int disNumMax;                       //最大干扰点                默认=500
    int plateType;                       //板材左右高低类型           0:左在右上   1：右在左上 默认=0
    int trackPos;
};

//对接坡口参数类
class SKGROOVERECOG_EXPORT ButtGroove
{
public:
    ButtGroove();
    void init();
    double planeMatchThre;          //平面配准阈值(mm)        默认=0.5
    double interestMatchThre;       //感兴趣阈值(mm)        默认=1
    double planeAngleMin;           //底板最小角度(度)    默认=-30
    double planeAngleMax;           //底板最大角度(度)    默认=30
    double gapMin;                  //坡口最小间隙      默认=0.1
    double gapMax;                  //坡口最大间隙      默认=2
    double mismatchMin;             //坡口最小错边      默认=0.2
    double mismatchMax;             //坡口最大错边      默认=10
    int dPlainPointsNum;            //平台点数
    double dPlainMaxDifHeight;      //平台高度阶差阈值      默认=0.8
    int planeNumMin;                //面最小点数          默认=50
    int sheetType;                  //跳边类型   0：左侧    1：右侧
    int angleMethod;                //角度计算   0：整段    1：左半段   2：右半段
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//定制坡口参数类
class SKGROOVERECOG_EXPORT SkComplexVar
{
public:
    SkComplexVar();
    void init();
    void print();
    bool enable;                    //使能                 0:禁止 1:使能               默认=1
    bool checkable;                 //用户决定使用与否       0:禁止 1:使用               默认=0
    QString property;               //属性
    int varType;                    //类型                 0:浮点型 1:整形 2:枚举型     默认=0
    int iVal1;                    //整形时启用,设定值
    int iMin1;                       //整形时启用,最小值
    int iMax1;                       //整形时启用,最大值
    int iVal2;                    //整形时启用,设定值
    int iMin2;                       //整形时启用,最小值
    int iMax2;                       //整形时启用,最大值
    double dVal1;                    //浮点型时启用,设定值
    double dMin1;                    //浮点型时启用,最小值
    double dMax1;                    //浮点型时启用,最大值
    double dVal2;                    //浮点型时启用,设定值
    double dMin2;                    //浮点型时启用,最小值
    double dMax2;                    //浮点型时启用,最大值
    int cVal;                       //枚举型时启用,设定值
    std::vector<QString> qVal;      //枚举型时启用,可供选择的枚举列表
};

//定制类
class SKGROOVERECOG_EXPORT SkSmartImageAI
{
public:
    SkSmartImageAI();
    void init();
    std::vector<SkComplexVar> para;
};

//坐标范围类
class SKGROOVERECOG_EXPORT SkField
{
public:
    SkField();
    void init();
    double cYmin;                       //近端Y下限              默认=-30
    double cYmax;                       //近端Y上限              默认=30
    double fYmin;                       //远端Y下限              默认=-100
    double fYmax;                       //远端Y上限              默认=100
    double cZ;                          //近端Z值               默认=-50
    double fZ;                          //远端Z值               默认=-400
};

//坡口分段类
class SKGROOVERECOG_EXPORT SkGroove
{
public:
    SkGroove();
    void init();
    int grooveType; //坡口类型  0:宽度方式； 1:错边方式  2:凹陷方式;
    SkPoint3D grooveP;
    SkPoint3D leftEndP;
    SkPoint3D rightEndP;
    SkLine leftLine;
    SkLine rightLine;
    std::vector <SkPoint3D> dataIn;
    std::vector <SkPoint3D> leftData;
    std::vector <SkPoint3D> rightData;
};

class SKGROOVERECOG_EXPORT SkSensor
{
public:
    SkSensor();
    int mode;
    int frame;
    double ratio;
    double height;
    double intercept;
};

class SKGROOVERECOG_EXPORT SkPlaneProperty
{
public:
    SkPlaneProperty();
    void init();
    void reverse();
    int iPlainExist;
    int iOrder;
    SkPoint3D linePos;
    SkPoint3D leftP;
    SkPoint3D rightP;
    double dPlainTolerance;
};

class SKGROOVERECOG_EXPORT SkScanResult
{
public:
    SkScanResult();
    void init();
    void reverse();
    int iGotResult;
    int iOrder;
    int algoFlag;
    SkPoint3D linePos;
    double dLowY;
    double dHighY;
    double dLowHeight;
    double dHighHeight;

    SkPoint3D leftPlaneLeftP;
    SkPoint3D leftPlaneRightP;
    SkPoint3D rightPlaneLeftP;
    SkPoint3D rightPlaneRightP;
    SkPoint3D midP;
};

class SKGROOVERECOG_EXPORT SkScanAlgCmdDet
{
public:
    SkScanAlgCmdDet();
    void init();
    void reverse();
    std::vector<SkScanResult> scanResultVector;
    SkScanResult scanResult;
    int scanResCount;
    std::vector<int> algoCounts;
};

//纠正输出类
class SKGROOVERECOG_EXPORT SkBase
{
public:
    SkBase();
    SkBase(const SkBase &mB);
    void init();
    int frame;
    SkVector6d base;
    SkVector6d tolorence;
    SkVector6d threshold;
};

//数据类型转换
SkMatrix4d joint2trans(SkVector6d jr, int option = 0);
SkVector3d joint2vector(SkVector6d jr);
SkPoint3D joint2Point3d(SkVector6d jr);
SkVector9d joint2extendJ(SkVector6d jr);

SkMatrix4d vector2trans(SkVector3d jv);
SkVector6d vector2joint(SkVector3d jv, int option = 0, int index = 0);
SkPoint3D vector2Point3d(SkVector3d jv);
SkVector9d vector2extendJ(SkVector3d jv, int option = 0, int index = 0);

SkVector6d trans2joint(SkMatrix4d tr, int option = 0);
SkVector3d trans2vector(SkMatrix4d tr);
SkPoint3D trans2Point3d(SkMatrix4d tr);
SkVector9d trans2extendJ(SkMatrix4d tr, int option = 0);

SkVector6d point3d2joint(SkPoint3D pr);
SkVector3d point3d2vector(SkPoint3D pr);
SkMatrix4d point3d2trans(SkPoint3D pr);
SkVector9d point3d2extendJ(SkPoint3D pr);

SkVector6d extendJ2joint(SkVector9d extJr);
SkVector3d extendJ2vector(SkVector9d extJr);
SkMatrix4d extendJ2trans(SkVector9d extJr, int option = 0);
SkPoint3D extendJ2point3d(SkVector9d extJr);

#endif // SKDATACLASS_H
