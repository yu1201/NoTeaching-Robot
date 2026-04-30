#ifndef SKFUNCTION_H
#define SKFUNCTION_H

#include "SkDataClass.h"
#include "QFileInfo"

extern QString gGrooveVersion;
extern SkSensor gSensorPara;
extern SkMatrix4d gInterMatrix;
extern int gCurrentLanguage;

SkMatrix4d rotx(double radx, int option = 0);
SkMatrix4d roty(double rady, int option = 0);
SkMatrix4d rotz(double radz, int option = 0);
SkMatrix4d rpy2tr(double rx, double ry, double rz, int option = 0);
void tr2rpy(SkMatrix4d T, double &rx, double &ry, double &rz, int option = 0);

bool sortByX(SkVector6d m, SkVector6d n);//按X升序排列
bool sortByY(SkVector6d m, SkVector6d n);//按Y升序排列
bool sortByZ(SkVector6d m, SkVector6d n);//按Z升序排列
bool sortByA(SkVector6d m, SkVector6d n);//按A升序排列
bool sortByB(SkVector6d m, SkVector6d n);//按B升序排列
bool sortByC(SkVector6d m, SkVector6d n);//按C升序排列
bool sortByX(SkContourInfo m, SkContourInfo n);//按X升序排列
bool sortByY(SkContourInfo m, SkContourInfo n);//按Y升序排列
bool sortByZ(SkContourInfo m, SkContourInfo n);//按Z升序排列
bool sortByA(SkContourInfo m, SkContourInfo n);//按A升序排列
bool sortByB(SkContourInfo m, SkContourInfo n);//按B升序排列
bool sortByC(SkContourInfo m, SkContourInfo n);//按C升序排列

double getAhead(double height);
int getFullCoord(SkPoint3D &point);

//检查长度符合与否
int checkLength(vector<SkPoint3D> data, int minLength, int maxLength, int &passFlag);

//检查总点数符合与否
int checkTotalPointNum(vector<SkPoint3D> data, SkContourInfo &info, int minValue, int maxValue, int &passFlag);

//修改基准
int getAdjInfo(SkContourInfo info, SkVector6d adjBase, SkContourInfo &adjInfo, int adjFrame);

//圆弧分段
int getArcsByGapAndAngleAndLength(vector<SkPoint3D> dataIn, SkPoint3D arcCenter, vector<vector <SkPoint3D>> &dataListOut, double angleMin, double angleMax, double lengthMin, double lengthMax, double gapThre, int planeNumMin);
//圆弧分段
int getArcsByGapAndLength(vector<SkPoint3D> dataIn, SkPoint3D arcCenter, vector<vector <SkPoint3D>> &dataListOut, double radius, double gapThre, double lengthMin, double lengthMax, int planeNumMin);

int getBestDataByGapAndAngleAndLength(vector<SkPoint3D> dataIn, SkPoint3D circleP, vector<SkPoint3D> &dataRes, SkContourInfo &info, double gapThre,
                                      int planeNumMin, double angleMin = 0, double angleMax = 360, double lengthMin = 0, double lengthMax = 1000);

int getBestDataByGapAndLength(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataRes, SkContourInfo &info, double gapThre,
                              int planeNumMin, double lengthMin = 0, double lengthMax = 1000);
//根据间隙、长度提取点云
int getBestDataByGapAndLength(vector<SkPoint3D> dataIn, SkLine line, vector<SkPoint3D> &dataRes, SkContourInfo &info, double gapThre,
                              int planeNumMin, double lengthMin = 0, double lengthMax = 1000);

int getDataByOrder(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataOut, int startOrder, int endOrder);
int getDataByCoordY(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataOut, double yMin, double yMax, int type = 0);
int getDataByCoordZ(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataOut, double zMin, double zMax, int type = 0);
//根据间隙提取点云
int getDataByGapAndDir(vector<SkPoint3D>data, SkLine line, vector<SkPoint3D> &dataRes, int dir, double gapThre, int planeNumMin);

//根据间隙、长度、方向提取第一点云
int getDataByGapAndLengthAndDir(vector<SkPoint3D> data, SkLine line, vector<SkPoint3D> &dataRes, SkContourInfo &info, int dir, double gapThre,
                                int planeNumMin, double lengthMin = 0, double lengthMax = 1000);

//根据直线获取直线上的点
//type:0/两侧 1/点在直线上侧  2/点在直线下侧
int  getDataByMatchThre(vector<SkPoint3D> data, SkLine line, vector<SkPoint3D> &dataRes, int type, double minMatchThre, double maxMatchThre);

//点到直线的距离
double getDisFromPointToLine(SkPoint3D data, SkLine line);

//点到直线的距离(带正负,正数表示点在直线上方)
double getDisFromPointToLine2(SkPoint3D data, SkLine line, int lineUp = false);

//提取到平面符合条件的点云  0/点在直线上侧  1/点在直线下侧
std::vector<SkPoint3D> getDataByDis(std::vector<SkPoint3D> data, SkLine line, int type, double disThre);

//点与点的距离 type: 0/二维距离  1/三维距离
double getDisFromPointToPoint(SkPoint3D data1, SkPoint3D data2, int type = 0);

//计算边界点  type:0/两侧 1/点在直线上侧  2/点在直线下侧
int getEdgeByDisMatch(vector<SkPoint3D>data, SkLine line, SkPoint3D &dataOut, int type, int dir, double disMatch, int times);

//计算边界点
int getEdgeByGap(vector<SkPoint3D>data, SkLine line, SkPoint3D &dataOut, int dir, double gapThre);

//计算边界点 type:0/两侧 1/点在直线上侧  2/点在直线下侧
int getEdgeByLineAndVertHAndDisMatch(vector<SkPoint3D>data, SkLine line, SkPoint3D &dataOut, int type, int dir, double disMatch, double minVertH, double maxVertH, int times);

//计算点在圆弧上的投影
int getFootPointToArc(SkPoint3D data, SkPoint3D arcCenter, SkPoint3D &projection, double radius);

//计算点在直线上的投影
int getFootPointToLine(SkPoint3D data, SkLine line, SkPoint3D &footPoint);

//计算两向量之间的夹角
int getIntersectAngle(SkPoint3D vector1, SkPoint3D vector2, double &angleOut);

//计算两直线的夹角
int getIntersectAngle(SkLine line1, SkLine line2, double &minAngleOut, double &maxAngleOut);

//计算两直线的夹角
//type:0/ line1:x分量正数、line2:x分量正数;  1/ line1:x分量正数、line2:x分量负数;
//     2/ line1:x分量负数、line2:x分量正数;  3/ line1:x分量负数、line2:x分量负数;
int getIntersectAngle(SkLine line1, SkLine line2, int type, double &angleOut);

//计算两直线交点
int getIntersectPoint(SkLine line1, SkLine line2, SkPoint3D &dataOut);

//计算两圆交点
//type:0/上侧  1/下侧   2/左侧   3/右侧
int getIntersectPoint(SkPoint3D arcCenter1, SkPoint3D arcCenter2, SkPoint3D &pOut, int type, double radius1, double radius2);

//计算直线和圆交点
//type:0/左侧点  1/右侧点   2/上侧点   3/下侧点
int getIntersectPoint(SkLine line, SkPoint3D arcCenter, SkPoint3D &dataOut, int type, double radius);

//计算直线
int getLine(SkPoint3D data1, SkPoint3D data2, SkLine &lineOut);

//计算直线
int getLine(vector<SkPoint3D> dataIn, SkLine &lineOut, int times);

//计算直线
int getLine(SkPoint3D dataIn, SkLine &lineOut, double k);

//直线分段
int getLinesByGapAndLength(vector<SkPoint3D> dataIn, SkLine line, vector<vector <SkPoint3D>> &dataListOut, double gapThre, double lengthMin, double lengthMax, int planeNumMin);
int getGrooveTurningPoints(std::vector<SkPoint3D> outPoints, std::vector<SkPoint3D>& TurningPoint);

//最小二乘法计算直线
//r:相关系数   ->1:相关性大  ->0:相关性小
int getLineByLeastSquares(vector<SkPoint3D> dataIn, SkLine &lineOut, double &r);

//根据输入的点数和
bool getLineParaByPcl(SkPoint3D *sourceData, SkLine line, SkPoint3D &midP, SkLine &fitLine, int numOfData, double matchThre);

//计算点到直线的最大距离
//type:0/两侧 1/点在直线上侧  2/点在直线下侧
int getMaxDisFromPointToLine(vector<SkPoint3D> dataIn, SkLine line, SkPoint3D &dataOut, int type, double &maxDis);

//计算点到圆弧的最大距离
//type:0/两侧  1/点在圆弧里侧  2/点在圆弧外侧
int getMaxDisFromPointToArc(vector<SkPoint3D> dataIn, SkPoint3D arcCenter, SkPoint3D &dataOut, int type, double radius, double &maxDis);

//计算点离圆弧表面的最大距离
double getMaxDisFromPointToArc(SkPoint3D data, SkPoint3D arcCenter, SkPoint3D &projection, double radius);

//计算相邻点的最大间隔   p1:左侧点  p2：右侧点  maxDis ： 最大相邻间隔
int getMaxDisFromPointByPoint(vector<SkPoint3D> dataIn, SkLine line, SkPoint3D &p1, SkPoint3D &p2, double &maxDis);

//计算索引范围相邻点的最大间隔   p1:左侧点  p2：右侧点  maxDis ： 最大相邻间隔
int getMaxDisFromPointByPoint(vector<SkPoint3D> dataIn, SkLine line, int startIndex, int endIndex, SkPoint3D &p1, SkPoint3D &p2, double &maxDis);

//计算均值
int getMean(vector<SkPoint3D> dataIn, SkPoint3D &dataOut);

//计算点离圆弧表面的最小距离
double getMinDisFromPointToArc(SkPoint3D data, SkPoint3D arcCenter, SkPoint3D &projection, double radius);

//根据坐标值值截取点云   dimension:0-x值  1:y值  2:z值
int getPartOfDataByCorrd(vector<SkPoint3D> data, vector<SkPoint3D> &dataRes, double minValue, double maxValue, int dimension);

//根据索引号截取点云
int getPartOfDataByIndex(vector<SkPoint3D> data, vector<SkPoint3D> &dataRes, int startIndex = -1, int endIndex = -1, int dir = 0);

//计算line2上的且与line1的距离为vertH的点的坐标
//type:0/点在交点上方  1/点在交点下方  2/点在交点左侧  3/点在交点右侧
int getPointByVertHAndIntersection(SkLine line1, SkLine line2, SkPoint3D &dataOut, int type, double vertH);

//根据高度差、平面、垂足点求取点
//type:0/求的点在直线上方  1/求的点在直线下方
int getPointByVertPAndLineAndDis(SkPoint3D vertP, SkPoint3D &pOut, SkLine line, double dis, int type);

//计算两点在直线上的投影距离
double getPointsFootDisToLine(SkPoint3D data1, SkPoint3D data2, SkLine line);

//计算两点在直线上的mismatch距离
double getPointsMismatchDisToLine(SkPoint3D data1, SkPoint3D data2, SkLine line);

//计算垂直线
int getVerticalLine(SkPoint3D dataIn, SkLine lineIn, SkLine &lineOut);

//拟合直线
int SmartLine(vector<SkPoint3D> sourceData, vector<SkPoint3D> &fitData, vector<SkPoint3D> &unFitData, SkContourInfo &infoOut,
              SkLine &fitLine, SkPoint3D &midP, double matchThre, double ratioMin, double ratioMax, int planeNumMin, int outlinerNumMax = 10000);

//拟合直线
int SmartLine2(vector<SkPoint3D> sourceData, vector<SkPoint3D> &fitData, vector<SkPoint3D> &unFitData, SkContourInfo &infoOut,
               SkLine &fitLine, SkPoint3D &midP, double matchThre, double ratioMinusMax, double ratioPlusMin, int planeNumMin, int outlinerNumMax);
//拟合圆弧
int SmartCircle(vector<SkPoint3D> sourceData, vector<SkPoint3D> &fitData, vector<SkPoint3D> &unFitData, SkContourInfo &infoOut,
                SkPoint3D &fitCircle, double &radius, double matchThre, double radiusMin, double radiusMax, int planeNumMin, int outlinerNumMax);
bool getCircleParaByThreePoints(SkPoint3D firstP, SkPoint3D secondP, SkPoint3D thirdP, SkPoint3D &circleOut, double &radius);

bool getCircleParaByPcl(SkPoint3D *sourceData, SkPoint3D circle, double radius, SkPoint3D &fitCircle, double &fitRadius, int numOfData,  double matchThre);


int getConcaveGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double gapMin, double gapMax, double sunkenMin, int maxGroove = 3);
int getBulgeGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double gapMin, double gapMax, double bulgeMin, int maxGroove = 3);
int getGapGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double gapMin, double gapMax, int maxGroove = 3);
int getMismatchGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double ratioMin, double ratioMax, double mismatchMin, double mismatchMax, double matchThre, int planeNumMin, int outlinerNumMax = 2000);
int getMismatchGroove2(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double ratioMin, double ratioMax, double gapMaxInPlane, double matchThre, double disMatchThre, int planeNumMin, int outlinerNumMax = 2000);
int getLineEndPoint(vector<SkPoint3D> sourceData, SkLine line, SkPoint3D &endPoint, int start, int finish, double gapMaxInPlane);

int getMaxDistanceFromPointsToLine(vector<SkPoint3D> sourceData, SkLine line, SkPoint3D &maxDisP, int ptUp, double &maxDis);

//target:输出点   lineA：输出点到该线的距离为dis, lineB:输出点在该线上
//type:0-输出点在交点的上侧，1-输出点在交点第的下侧，2-输出点在交点的左侧，3-输出点在交点的右侧
int getPointByHeightAndLine(SkLine lineA, SkLine lineB, SkPoint3D &target, int type, double dis);

int getDataByRotate(vector<SkPoint3D> sourceData, vector<SkPoint3D> &resultData, double rotateAngle, int option = 0);
int getDataByRotate2(vector<SkPoint3D> sourceData, vector<SkPoint3D> &resultData, double rotateAngle, SkPoint3D centerP, int option = 0);

int getDataByRotate(SkPoint3D sourceData, SkPoint3D &resultData, double rotateAngle, int option = 0);
int getDataByRotate2(SkPoint3D sourceData, SkPoint3D &resultData, double rotateAngle, SkPoint3D centerP, int option = 0);
int getDataByRotate(SkVector6d sourceData, SkVector6d &resultData, double rotateAngle, int option = 0);
void reOrderPCL(std::vector<SkPoint3D> &mPCL, int reOrder);
std::vector<SkPoint3D> reOrderPCL2(std::vector<SkPoint3D> mPCL, int reOrder);
SkPlaneProperty getPlainZProperty(SkPoint3D *origin, int length, double tolerance);
SkPlaneProperty getPlainYProperty(SkPoint3D *origin, int length, double tolerance);
SkPlaneProperty getPlainZProperty(SkPoint3D *origin, int length, double tolerance, double gapMax);
int getButtGroove(int sheetType, SkPoint3D *origin, int pSize, double dPlainMaxDifHeight, double gapMin, double gapMax, double heightMin, double heightMax, int plainPointsNum, SkScanAlgCmdDet *scanAlgCmdDet, double angleBottom, SkContourInfo &info);
int getLapGroove(int sheetType, SkPoint3D *origin, int pSize, double dPlainMaxDifHeight, double gapMax, double heightMin, double heightMax, int plainPointsNum, SkScanAlgCmdDet *scanAlgCmdDet, double angleBottom, SkContourInfo &info);
void LogRecord(QFile *fileData, QTextStream *out, QString str, bool replace = FALSE);

int getArea(SkPoint3D linePa, SkPoint3D linePb, SkPoint3D pc, double &area);
int getPointByValue(std::vector<SkPoint3D> data, SkPoint3D &targetP, int type = 0);
SkContourInfo getExportByBase(SkContourInfo infoIn, SkBase basePara);
SkScanResult getWeldLineNewHLSmallDif2(SkPoint3D *origin, int length, double dLineHeightDif, double dPlainMaxDifL, int dPlainLenL, double sepMaxDis, SkScanAlgCmdDet *scanAlgCmdDet, int sectionYPointNum, double sectionYThre, double angleBottom, SkContourInfo &info);

#endif // SKFUNCTION_H
