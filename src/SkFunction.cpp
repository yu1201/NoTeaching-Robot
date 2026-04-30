#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include "SkFunction.h"
QString gGrooveVersion = "AdvancedV1.31C";
SkSensor gSensorPara;
SkMatrix4d gInterMatrix;
int gCurrentLanguage = 0;

SkMatrix4d rotx(double a, int option)
{
    SkMatrix4d tOut;
    if(option == 1)
        a = a *PI / 180.0;
    tOut.m[0][0] = 1;
    tOut.m[0][1] = 0;
    tOut.m[0][2] = 0;
    tOut.m[0][3] = 0;
    tOut.m[1][0] = 0;
    tOut.m[1][1] = cos(a);
    tOut.m[1][2] = -sin(a);
    tOut.m[1][3] = 0;
    tOut.m[2][0] = 0;
    tOut.m[2][1] = sin(a);
    tOut.m[2][2] = cos(a);
    tOut.m[2][3] = 0;
    tOut.m[3][0] = 0;
    tOut.m[3][1] = 0;
    tOut.m[3][2] = 0;
    tOut.m[3][3] = 1;
    return tOut;
}
SkMatrix4d roty(double b, int option)
{
    SkMatrix4d tOut;
    if(option == 1)
        b = b *PI / 180.0;
    tOut.m[0][0] = cos(b);
    tOut.m[0][1] = 0;
    tOut.m[0][2] = sin(b);
    tOut.m[0][3] = 0;
    tOut.m[1][0] = 0;
    tOut.m[1][1] = 1;
    tOut.m[1][2] = 0;
    tOut.m[1][3] = 0;
    tOut.m[2][0] = -sin(b);
    tOut.m[2][1] = 0;
    tOut.m[2][2] = cos(b);
    tOut.m[2][3] = 0;
    tOut.m[3][0] = 0;
    tOut.m[3][1] = 0;
    tOut.m[3][2] = 0;
    tOut.m[3][2] = 1;
    return tOut;
}
SkMatrix4d rotz(double c, int option)
{
    SkMatrix4d tOut;
    if(option == 1)
        c = c *PI / 180.0;
    tOut.m[0][0] = cos(c);
    tOut.m[0][1] = -sin(c);
    tOut.m[0][2] = 0;
    tOut.m[0][3] = 0;
    tOut.m[1][0] = sin(c);
    tOut.m[1][1] = cos(c);
    tOut.m[1][2] = 0;
    tOut.m[1][3] = 0;
    tOut.m[2][0] = 0;
    tOut.m[2][1] = 0;
    tOut.m[2][2] = 1;
    tOut.m[2][3] = 0;
    tOut.m[3][0] = 0;
    tOut.m[3][1] = 0;
    tOut.m[3][2] = 0;
    tOut.m[3][3] = 1;
    return tOut;
}
SkMatrix4d rpy2tr(double rx, double ry, double rz, int option)
{
    SkMatrix4d tOut;
    if(option == 1)
    {
        rx = rx *PI / 180.0;
        ry = ry *PI / 180.0;
        rz = rz *PI / 180.0;
    }
    tOut.m[0][0] = cos(rz) * cos(ry);
    tOut.m[0][1] = cos(rz) * sin(ry) * sin(rx) - sin(rz) * cos(rx);
    tOut.m[0][2] = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);
    tOut.m[0][3] = 0;

    tOut.m[1][0] = sin(rz) * cos(ry);
    tOut.m[1][1] = sin(rz) * sin(ry) * sin(rx) + cos(rz) * cos(rx);
    tOut.m[1][2] = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);
    tOut.m[1][3] = 0;

    tOut.m[2][0] = -sin(ry);
    tOut.m[2][1] = cos(ry) * sin(rx);
    tOut.m[2][2] = cos(ry) * cos(rx);
    tOut.m[2][3] = 0;

    tOut.m[3][0] = 0;
    tOut.m[3][1] = 0;
    tOut.m[3][2] = 0;
    tOut.m[3][3] = 1;
    return tOut;
}
void tr2rpy(SkMatrix4d T, double &rx, double &ry, double &rz, int option)
{
    Eigen::Vector3d eulerAngle;
    Eigen::Matrix3d rotateT = T.mT.block<3, 3>(0, 0);
    eulerAngle = rotateT.eulerAngles(2, 1, 0);
    rx = eulerAngle(2);
    ry = eulerAngle(1);
    rz = eulerAngle(0);
    if(option == 1)
    {
        rx = rx * 180.0 / PI;
        ry = ry * 180.0 / PI;
        rz = rz * 180.0 / PI;
    }
}

bool sortByX(SkVector6d m, SkVector6d n)
{
    return (m.x < n.x);    //按X升序排列
}
bool sortByY(SkVector6d m, SkVector6d n)
{
    return (m.y < n.y);    //按Y升序排列
}
bool sortByZ(SkVector6d m, SkVector6d n)
{
    return (m.z < n.z);    //按Z升序排列
}
bool sortByA(SkVector6d m, SkVector6d n)
{
    return (m.a < n.a);    //按A升序排列
}
bool sortByB(SkVector6d m, SkVector6d n)
{
    return (m.b < n.b);    //按B升序排列
}
bool sortByC(SkVector6d m, SkVector6d n)
{
    return (m.c < n.c);    //按C升序排列
}
bool sortByX(SkContourInfo m, SkContourInfo n)
{
    return (m.mp.x < n.mp.x);    //按X升序排列
}
bool sortByY(SkContourInfo m, SkContourInfo n)
{
    return (m.mp.y < n.mp.y);    //按Y升序排列
}
bool sortByZ(SkContourInfo m, SkContourInfo n)
{
    return (m.mp.z < n.mp.z);    //按Z升序排列
}
bool sortByA(SkContourInfo m, SkContourInfo n)
{
    return (m.mp.a < n.mp.a);    //按A升序排列
}
bool sortByB(SkContourInfo m, SkContourInfo n)
{
    return (m.mp.b < n.mp.b);    //按B升序排列
}
bool sortByC(SkContourInfo m, SkContourInfo n)
{
    return (m.mp.c < n.mp.c);    //按C升序排列
}

double getAhead(double height)
{
    return (gSensorPara.ratio *height + gSensorPara.ratio *gSensorPara.intercept);
}

int getFullCoord(SkPoint3D &point)
{
    point.x = gSensorPara.ratio *point.z + gSensorPara.ratio *gSensorPara.intercept;
    return TRUE;
}

//检查长度符合与否
int checkLength(vector<SkPoint3D> data, int minLength, int maxLength, int &passFlag)
{
    if(data.empty())
        return FALSE;
    double length = getDisFromPointToPoint(data.front(), data.back());
    if(length >= minLength && length <= maxLength)
    {
        passFlag = 1;
    }
    else
    {
        passFlag = 0;
    }
    return TRUE;
}

//检查总点数符合与否
int checkTotalPointNum(vector<SkPoint3D> data, SkContourInfo &info, int minValue, int maxValue, int &passFlag)
{
    if(data.size() < minValue)
    {
        info.errId = -20;
        sprintf(info.errInfo, "总点数太少(%d)", data.size());
        return FALSE;
    }
    else if(data.size() > maxValue)
    {
        info.errId = -21;
        sprintf(info.errInfo, "总点数太多(%d)", data.size());
        return FALSE;
    }
    return TRUE;
}

//修改基准
int getAdjInfo(SkContourInfo info, SkVector6d adjBase, SkContourInfo &adjInfo, int adjFrame)
{
    int rtn = TRUE;
    adjInfo = info;
    if(info.result == 1)
    {
        if(adjFrame == 0)
        {
            for(int i = 0; i < 3; i++)
            {
                adjInfo.mp.v[i] = info.mp.v[i] - adjBase.v[i];
                adjInfo.op.v[i] = info.op.v[i] - adjBase.v[i];
            }
        }
        else if(adjFrame == 1)
        {
            SkMatrix4d transform = rotx(-atan2(info.mp.b, info.mp.c));
            SkVector6d adjV = transform *adjBase;
            for(int i = 0; i < 3; i++)
            {
                adjInfo.mp.v[i] = info.mp.v[i] - adjV.v[i];
                adjInfo.op.v[i] = info.op.v[i] - adjV.v[i];
            }
        }
    }
    return rtn;
}

//圆弧分段
int getArcsByGapAndAngleAndLength(vector<SkPoint3D> dataIn, SkPoint3D arcCenter, vector<vector <SkPoint3D>> &dataListOut, double angleMin, double angleMax, double lengthMin, double lengthMax, double gapThre, int planeNumMin)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    int startIndex = 0, endIndex = 0;
    double angle = 0;
    vector<SkPoint3D> dataArray;
    for(int i = 0; i < int(dataIn.size() - 1); i++)
    {
        double dis = getDisFromPointToPoint(dataIn[i], dataIn[i + 1]);
        if(dis > gapThre)
        {
            SkPoint3D p1(dataIn[startIndex] - arcCenter);
            SkPoint3D p2(dataIn[i] - arcCenter);
            getIntersectAngle(p1, p2, angle);
            if(angle >= angleMin && angle <= angleMax)
            {
                double dis = getDisFromPointToPoint(dataIn[startIndex], dataIn[i]);
                if(dis >= lengthMin && dis <= lengthMax && (i - startIndex > planeNumMin))
                {
                    endIndex = i;
                    dataArray.assign(dataIn.begin() + startIndex, dataIn.begin() + endIndex + 1);
                    dataListOut.push_back(dataArray);
                }
                startIndex = i + 1;
            }
        }
        else if(i == int(dataIn.size() - 2))
        {
            SkPoint3D p1(dataIn[startIndex] - arcCenter);
            SkPoint3D p2(dataIn.back() - arcCenter);
            getIntersectAngle(p1, p2, angle);
            if(angle >= angleMin && angle <= angleMax)
            {
                double dis = getDisFromPointToPoint(dataIn[startIndex], dataIn.back());
                if(dis >= lengthMin && dis <= lengthMax && (int(dataIn.size()) - 1 - startIndex > planeNumMin))
                {
                    dataArray.assign(dataIn.begin() + startIndex, dataIn.end());
                    dataListOut.push_back(dataArray);
                }
            }
        }
    }
    if(dataListOut.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//圆弧分段
int getArcsByGapAndLength(vector<SkPoint3D> dataIn, vector<vector <SkPoint3D>> &dataListOut, double gapThre, double lengthMin, double lengthMax, int planeNumMin)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    int startIndex = 0, endIndex = 0;
    vector<SkPoint3D> dataArray;
    for(int i = 0; i < int(dataIn.size() - 1); i++)
    {
        double dis = getDisFromPointToPoint(dataIn[i], dataIn[i + 1]);
        if(dis > gapThre)
        {
            double disAll = getDisFromPointToPoint(dataIn[startIndex], dataIn[i]);
            if((i - startIndex > planeNumMin) && disAll >= lengthMin && disAll <= lengthMax)
            {
                endIndex = i;
                dataArray.assign(dataIn.begin() + startIndex, dataIn.begin() + endIndex + 1);
                dataListOut.push_back(dataArray);
            }
            startIndex = i + 1;
        }
        else if(i == int(dataIn.size() - 2))
        {
            double disAll = getDisFromPointToPoint(dataIn[startIndex], dataIn.back());
            if((int(dataIn.size()) - 1 - startIndex > planeNumMin) && disAll >= lengthMin && disAll <= lengthMax)
            {
                dataArray.assign(dataIn.begin() + startIndex, dataIn.end());
                dataListOut.push_back(dataArray);
            }
        }
    }
    if(dataListOut.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//根据间隙、长度提取点云
int getBestDataByGapAndLength(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataRes, SkContourInfo &info, double gapThre,
                              int planeNumMin, double lengthMin, double lengthMax)
{
    int rtn = TRUE;
    int maxNum = 0;
    vector<SkPoint3D> bestData;
    vector<vector <SkPoint3D>>  dataList;
    rtn = getArcsByGapAndLength(dataIn, dataList, gapThre, lengthMin, lengthMax, planeNumMin);
    if(rtn == FALSE)
    {
        return rtn;
    }
    for(int i = 0; i < dataList.size(); i++)
    {
        if(dataList[i].size() > maxNum)
        {
            bestData.assign(dataList[i].begin(), dataList[i].end());
            maxNum = dataList[i].size();
        }
    }
    if(bestData.empty())
    {
        return FALSE;
    }
    dataRes.assign(bestData.begin(), bestData.end());
    if(dataRes.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//根据间隙、长度提取点云
int getBestDataByGapAndLength(vector<SkPoint3D> dataIn, SkLine line, vector<SkPoint3D> &dataRes, SkContourInfo &info, double gapThre,
                              int planeNumMin, double lengthMin, double lengthMax)
{
    int rtn = TRUE;
    int maxNum = 0;
    vector<SkPoint3D> bestData;
    vector<vector <SkPoint3D>>  dataList;
    rtn = getLinesByGapAndLength(dataIn, line, dataList, gapThre, lengthMin, lengthMax, planeNumMin);
    if(rtn == FALSE)
    {
        return rtn;
    }
    for(int i = 0; i < dataList.size(); i++)
    {
        if(dataList[i].size() > maxNum)
        {
            bestData.assign(dataList[i].begin(), dataList[i].end());
            maxNum = dataList[i].size();
        }
    }
    if(bestData.empty())
    {
        return FALSE;
    }
    dataRes.assign(bestData.begin(), bestData.end());
    if(dataRes.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//根据间隙、长度提取点云
int getBestDataByGapAndAngleAndLength(vector<SkPoint3D> dataIn, SkPoint3D circleP, vector<SkPoint3D> &dataRes, SkContourInfo &info, double gapThre,
                                      int planeNumMin, double angleMin, double angleMax, double lengthMin, double lengthMax)
{
    int rtn = TRUE;
    int maxNum = 0;
    vector<SkPoint3D> bestData;
    vector<vector <SkPoint3D>>  dataList;
    //rtn=getArcsByGapAndLength(dataIn,dataList,gapThre,lengthMin,lengthMax,planeNumMin);
    rtn = getArcsByGapAndAngleAndLength(dataIn, circleP, dataList, angleMin, angleMax, lengthMin, lengthMax, gapThre, planeNumMin);
    if(rtn == FALSE)
    {
        return rtn;
    }
    for(int i = 0; i < dataList.size(); i++)
    {
        if(dataList[i].size() > maxNum)
        {
            bestData.assign(dataList[i].begin(), dataList[i].end());
            maxNum = dataList[i].size();
        }
    }
    if(bestData.empty())
    {
        return FALSE;
    }
    dataRes.assign(bestData.begin(), bestData.end());
    if(dataRes.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//根据间隙提取点云
int getDataByGapAndDir(vector<SkPoint3D>data, SkLine line, vector<SkPoint3D> &dataRes, int dir, double gapThre, int planeNumMin)
{
    if(data.empty())
    {
        return FALSE;
    }

    vector<SkPoint3D> dataArray;
    if(dir == 0)
    {
        int startIndex = 0, endIndex = 0;
        for(int i = 0; i < int(data.size() - 1); i++)
        {
            double dis = getPointsFootDisToLine(data[i], data[i + 1], line);
            if(dis > gapThre)
            {
                if((i - startIndex > planeNumMin))
                {
                    endIndex = i;
                    dataRes.assign(data.begin() + startIndex, data.begin() + endIndex + 1);
                    break;
                }
                startIndex = i + 1;
            }
            else if(i == int(data.size() - 2))
            {
                if((int(data.size()) - 1 - startIndex > planeNumMin))
                {
                    dataRes.assign(data.begin() + startIndex, data.end());
                    break;
                }
            }
        }
    }
    else
    {
        int startIndex = data.size() - 1, endIndex = 0;
        for(int i = int(data.size() - 1); i > 0; i--)
        {
            double dis = getPointsFootDisToLine(data[i], data[i - 1], line);
            if(dis > gapThre)
            {
                if((startIndex - i > planeNumMin))
                {
                    endIndex = i;
                    dataRes.assign(data.begin() + endIndex, data.begin() + startIndex + 1);
                    break;
                }
                startIndex = i - 1;
            }
            else if(i == 1)
            {
                if((startIndex > planeNumMin))
                {
                    dataRes.assign(data.begin(), data.begin() + startIndex + 1);
                    break;
                }
            }
        }
    }
    if(dataRes.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//根据间隙、长度、方向提取第一点云
int getDataByGapAndLengthAndDir(vector<SkPoint3D> data, SkLine line, vector<SkPoint3D> &dataRes, SkContourInfo &info, int dir, double gapThre,
                                int planeNumMin, double lengthMin, double lengthMax)
{
    if(data.empty())
    {
        return FALSE;
    }

    vector<SkPoint3D> dataArray;
    if(dir == 0)
    {
        int startIndex = 0, endIndex = 0;
        for(int i = 0; i < int(data.size() - 1); i++)
        {
            double dis = getPointsFootDisToLine(data[i], data[i + 1], line);
            if(dis > gapThre)
            {
                double length = getDisFromPointToPoint(data[startIndex], data[i]);
                if((i - startIndex > planeNumMin) && length >= lengthMin && length <= lengthMax)
                {
                    endIndex = i;
                    dataRes.assign(data.begin() + startIndex, data.begin() + endIndex + 1);
                    break;
                }
                startIndex = i + 1;
            }
            else if(i == int(data.size() - 2))
            {
                double length = getDisFromPointToPoint(data[startIndex], data.back());
                if((int(data.size()) - 1 - startIndex > planeNumMin) && length >= lengthMin && length <= lengthMax)
                {
                    dataRes.assign(data.begin() + startIndex, data.end());
                    break;
                }
            }
        }
    }
    else
    {
        int startIndex = data.size() - 1, endIndex = 0;
        for(int i = int(data.size() - 1); i > 0; i--)
        {
            double dis = getPointsFootDisToLine(data[i], data[i - 1], line);
            if(dis > gapThre)
            {
                double length = getDisFromPointToPoint(data[i], data[startIndex]);
                if((startIndex - i > planeNumMin) && length >= lengthMin && length <= lengthMax)
                {
                    endIndex = i;
                    dataRes.assign(data.begin() + endIndex, data.begin() + startIndex + 1);
                    break;
                }
                startIndex = i - 1;
            }
            else if(i == 1)
            {
                double length = getDisFromPointToPoint(data.front(), data[startIndex]);
                if((startIndex > planeNumMin) && length >= lengthMin && length <= lengthMax)
                {
                    endIndex = i;
                    dataRes.assign(data.begin(), data.begin() + startIndex + 1);
                    break;
                }
            }
        }
    }
    if(dataRes.empty())
    {
        return FALSE;
    }
    return TRUE;
}

//根据直线获取直线上的点
//type:0/两侧 1/点在直线上侧  2/点在直线下侧
int  getDataByMatchThre(vector<SkPoint3D> data, SkLine line, vector<SkPoint3D> &dataRes, int type, double minMatchThre, double maxMatchThre)
{
    double matchThre = 0;
    if(data.empty())
    {
        return FALSE;
    }
    for(int i = 0; i < data.size(); i++)
    {
        if(type == 0)
        {
            matchThre = getDisFromPointToLine(data[i], line);
        }
        else if(type == 1)
        {
            matchThre = (data[i].z - line.k *data[i].y - line.b) / sqrt(line.k *line.k + 1);
        }
        else
        {
            matchThre = (line.k *data[i].y + line.b - data[i].z) / sqrt(line.k *line.k + 1);
        }
        if(matchThre >= minMatchThre && matchThre <= maxMatchThre)
        {
            dataRes.push_back(data[i]);
        }
    }
    if(dataRes.empty())
    {
        return FALSE;
    }
    return TRUE;
}


int getDataByOrder(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataOut, int startOrder, int endOrder)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    int numOfP = dataIn.size();
    for(int i = 0; i < numOfP; i++)
    {
        if(dataIn[i].order >= startOrder && dataIn[i].order <= endOrder)
        {
            dataOut.push_back(dataIn[i]);
        }
    }
    return TRUE;
}

int getDataByCoordY(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataOut, double yMin, double yMax, int type)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    int numOfP = dataIn.size();
    if(type == 1) //限制最大值
    {
        for(int i = 0; i < numOfP; i++)
        {
            if(dataIn[i].y <= yMax)
            {
                dataOut.push_back(dataIn[i]);
            }
        }
    }
    else if(type == 2) //限制最小值
    {
        for(int i = 0; i < numOfP; i++)
        {
            if(dataIn[i].y >= yMin)
            {
                dataOut.push_back(dataIn[i]);
            }
        }
    }
    else
    {
        for(int i = 0; i < numOfP; i++)
        {
            if(dataIn[i].y >= yMin && dataIn[i].y <= yMax)
            {
                dataOut.push_back(dataIn[i]);
            }
        }
    }
    return TRUE;
}

int getDataByCoordZ(vector<SkPoint3D> dataIn, vector<SkPoint3D> &dataOut, double zMin, double zMax, int type)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    int numOfP = dataIn.size();
    if(type == 1) //限制最大值
    {
        for(int i = 0; i < numOfP; i++)
        {
            if(dataIn[i].z <= zMax)
            {
                dataOut.push_back(dataIn[i]);
            }
        }
    }
    else if(type == 2) //限制最小值
    {
        for(int i = 0; i < numOfP; i++)
        {
            if(dataIn[i].z >= zMin)
            {
                dataOut.push_back(dataIn[i]);
            }
        }
    }
    else
    {
        for(int i = 0; i < numOfP; i++)
        {
            if(dataIn[i].z >= zMin && dataIn[i].z <= zMax)
            {
                dataOut.push_back(dataIn[i]);
            }
        }
    }
    return TRUE;
}

//点到直线的距离
double getDisFromPointToLine(SkPoint3D data, SkLine line)
{
    return fabs((data.z - line.k *data.y - line.b) / sqrt(line.k *line.k + 1));
}

//点到直线的距离(带正负)
double getDisFromPointToLine2(SkPoint3D data, SkLine line, int lineUp)
{
    if(lineUp == false)
    {
        return (data.z - line.k *data.y - line.b) / sqrt(line.k *line.k + 1);
    }
    else
    {
        return (line.k *data.y + line.b - data.z) / sqrt(line.k *line.k + 1);
    }
}

//提取到平面符合条件的点云  0/点在直线上侧  1/点在直线下侧
std::vector<SkPoint3D> getDataByDis(std::vector<SkPoint3D> data, SkLine line, int type, double disThre)
{
    std::vector<SkPoint3D> dataOut;
    if(type == 0)
    {
        for(int i = 0; i < data.size(); i++)
        {
            double dis = (data[i].z - line.k *data[i].y - line.b) / sqrt(line.k *line.k + 1);
            if(dis > disThre)
            {
                dataOut.push_back(data[i]);
            }
        }
    }
    else
    {
        for(int i = 0; i < data.size(); i++)
        {
            double dis = (data[i].z - line.k *data[i].y - line.b) / sqrt(line.k *line.k + 1);
            if(dis < disThre)
            {
                dataOut.push_back(data[i]);
            }
        }
    }
    return dataOut;
}

//点与点的距离 type: 0/二维距离  1/三维距离
double getDisFromPointToPoint(SkPoint3D data1, SkPoint3D data2, int type)
{
    if(type == 0)
    {
        return sqrt((data1.y - data2.y) * (data1.y - data2.y) + (data1.z - data2.z) * (data1.z - data2.z));
    }
    else
    {
        return sqrt((data1.x - data2.x) * (data1.x - data2.x) + (data1.y - data2.y) * (data1.y - data2.y) + (data1.z - data2.z) * (data1.z - data2.z));
    }
}

//计算边界点  //type:0/两侧 1/点在直线上侧  2/点在直线下侧
int getEdgeByDisMatch(vector<SkPoint3D>data, SkLine line, SkPoint3D &dataOut, int type, int dir, double disMatch, int times)
{
    if(data.empty())
    {
        return FALSE;
    }
    int counter = 0;
    int findFlag = FALSE;
    if(dir == 0)
    {
        dataOut = data.back();
        for(int i = 0; i < data.size(); i++)
        {
            double dis = getDisFromPointToLine2(data[i], line);
            if(((type == 0) && (fabs(dis) > disMatch)) || \
                    ((type == 1) && (dis > disMatch)) || \
                    ((type == 2) && (dis < -disMatch)) )
            {
                counter++;
                if(counter > times)
                {
                    findFlag = TRUE;
                    dataOut = data[i - times];
                    break;
                }
            }
            else
            {
                counter = 0;
            }
        }
    }
    else
    {
        dataOut = data.front();
        for(int i = int(data.size() - 1); i >= 0; i--)
        {
            double dis = getDisFromPointToLine2(data[i], line);
            if(((type == 0) && (fabs(dis) > disMatch)) || \
                    ((type == 1) && (dis > disMatch)) || \
                    ((type == 2) && (dis < -disMatch)) )
            {
                counter++;
                if(counter > times)
                {
                    findFlag = TRUE;
                    dataOut = data[i + times];
                    break;
                }
            }
            else
            {
                counter = 0;
            }
        }
    }
    return findFlag;
}

//计算边界点
int getEdgeByGap(vector<SkPoint3D>data, SkLine line, SkPoint3D &dataOut, int dir, double gapThre)
{
    if(data.empty())
    {
        return FALSE;
    }
    int findFlag = FALSE;
    if(dir == 0)
    {
        dataOut = data.back();
        for(int i = 0; i < int(data.size() - 1); i++)
        {
            double gap = getPointsFootDisToLine(data[i], data[i + 1], line);
            if(gap > gapThre)
            {
                findFlag = TRUE;
                dataOut = data[i];
            }
        }
    }
    else
    {
        dataOut = data.front();
        for(int i = int(data.size() - 1); i > 0; i++)
        {
            double gap = getPointsFootDisToLine(data[i], data[i - 1], line);
            if(gap > gapThre)
            {
                findFlag = TRUE;
                dataOut = data[i];
            }
        }
    }
    return findFlag;
}

//计算边界点 type:0/两侧 1/点在直线上侧  2/点在直线下侧
int getEdgeByLineAndVertHAndDisMatch(vector<SkPoint3D>data, SkLine line, SkPoint3D &dataOut, int type, int dir, double disMatch, double minVertH, double maxVertH, int times)
{
    if(data.empty())
    {
        return FALSE;
    }
    int counter = 0;
    int findFlag = FALSE;
    if(dir == 0)
    {
        dataOut = data.back();
        for(int i = 0; i < data.size(); i++)
        {
            double dis = getDisFromPointToLine2(data[i], line);
            if(fabs(dis) < disMatch)
                dataOut = data[i];
            if(((type == 0) && (fabs(dis) > minVertH) && (fabs(dis) < maxVertH)) || \
                    ((type == 1) && (dis > minVertH) && (dis < maxVertH)) || \
                    ((type == 2) && (dis > -maxVertH) && (dis < -minVertH)) )
            {
                counter++;
                if(counter > times)
                {
                    findFlag = TRUE;
                    break;
                }
            }
            else
            {
                counter = 0;
            }
        }
    }
    else
    {
        dataOut = data.front();
        for(int i = int(data.size() - 1); i >= 0; i--)
        {
            double dis = getDisFromPointToLine2(data[i], line);
            if(fabs(dis) < disMatch)
                dataOut = data[i];
            if(((type == 0) && (fabs(dis) > minVertH) && (fabs(dis) < maxVertH)) || \
                    ((type == 1) && (dis > minVertH) && (dis < maxVertH)) || \
                    ((type == 2) && (dis > -maxVertH) && (dis < -minVertH)) )
            {
                counter++;
                if(counter > times)
                {
                    findFlag = TRUE;
                    break;
                }
            }
            else
            {
                counter = 0;
            }
        }
    }
    return findFlag;
}

//计算点在圆弧上的投影
int getFootPointToArc(SkPoint3D data, SkPoint3D arcCenter, SkPoint3D &projection, double radius)
{
    SkPoint3D p1, p2;
    SkLine line;
    SkPoint3D projection1, projection2;
    getLine(data, arcCenter, line);
    getIntersectPoint(line, arcCenter, projection1, 0, radius);
    getIntersectPoint(line, arcCenter, projection2, 1, radius);
    double dis1 = getDisFromPointToPoint(projection1, data);
    double dis2 = getDisFromPointToPoint(projection2, data);
    if(dis1 < dis2)
    {
        projection = projection1;
        return TRUE;
    }
    else
    {
        projection = projection2;
        return TRUE;
    }
}

//计算点在直线上的投影
int getFootPointToLine(SkPoint3D data, SkLine line, SkPoint3D &footPoint)
{
    double a = line.k;
    double b = -1;
    double c = line.b;
    SkPoint3D point;
    point.y = ( b *b *data.y - a *b *data.z - a *c ) / ( a *a + b *b );
    point.z = ( -a *b *data.y + a *a *data.z - b *c ) / ( a *a + b *b );
    point.x = getAhead(point.z);
    point.order = data.order;
    footPoint = point;
    return TRUE;
}

//计算两向量之间的夹角
int getIntersectAngle(SkPoint3D vector1, SkPoint3D vector2, double &angleOut)
{
    vector1.x = 0;
    vector2.x = 0;
    double cosThea = (vector1.mP.dot(vector2.mP)) / (vector1.mP.norm() * vector2.mP.norm());
    angleOut = acos(cosThea) * 180.0 / PI;
    return TRUE;
}

//计算两直线的夹角
int getIntersectAngle(SkLine line1, SkLine line2, double &minAngleOut, double &maxAngleOut)
{
    SkPoint3D vector1(0, -1, -line1.k);
    SkPoint3D vector2(0, 1, line2.k);
    double angle = 0;
    getIntersectAngle(vector1, vector2, angle);
    if(angle > 90)
    {
        minAngleOut = 180 - angle;
        maxAngleOut = angle;
    }
    else
    {
        minAngleOut = angle;
        maxAngleOut = 180 - angle;
    }
    return TRUE;
}

//计算两直线的夹角
//type:0/ line1:x分量正数、line2:x分量正数;  1/ line1:x分量正数、line2:x分量负数;
//     2/ line1:x分量负数、line2:x分量正数;  3/ line1:x分量负数、line2:x分量负数;
int getIntersectAngle(SkLine line1, SkLine line2, int type, double &angleOut)
{
    SkPoint3D vector1, vector2;
    if(type == 0)
    {
        vector1.y = 1;
        vector1.z = line1.k;
        vector2.y = 1;
        vector2.z = line2.k;
    }
    else if(type == 1)
    {
        vector1.y = 1;
        vector1.z = line1.k;
        vector2.y = -1;
        vector2.z = -line2.k;
    }
    else if(type == 2)
    {
        vector1.y = -1;
        vector1.z = -line1.k;
        vector2.y = 1;
        vector2.z = line2.k;
    }
    else
    {
        vector1.y = -1;
        vector1.z = -line1.k;
        vector2.y = -1;
        vector2.z = -line2.k;
    }
    getIntersectAngle(vector1, vector2, angleOut);
    return TRUE;
}

//计算两直线交点
int getIntersectPoint(SkLine line1, SkLine line2, SkPoint3D &dataOut)
{
    if(fabs(line1.k - line2.k) < EPS)
    {
        return FALSE;
    }
    else
    {
        SkPoint3D intersectP;
        intersectP.y = (line1.b - line2.b) / (line2.k - line1.k);
        intersectP.z = line1.k *intersectP.y + line1.b;
        intersectP.x = getAhead(intersectP.z);
        dataOut = intersectP;
    }
    return TRUE;
}

//计算两圆交点
//type:0/上侧  1/下侧   2/左侧   3/右侧
int getIntersectPoint(SkPoint3D arcCenter1, SkPoint3D arcCenter2, SkPoint3D &pOut, int type, double radius1, double radius2)
{
    int rtn = TRUE;
    SkPoint3D intersectP1, intersectP2;
    double a1, b1, R1, a2, b2, R2;
    a1 = arcCenter1.x;
    b1 = arcCenter1.y;
    R1 = radius1;

    a2 = arcCenter2.x;
    b2 = arcCenter2.y;
    R2 = radius2;

    double R1R1 = R1 *R1;
    double a1a1 = a1 *a1;
    double b1b1 = b1 *b1;

    double a2a2 = a2 *a2;
    double b2b2 = b2 *b2;
    double R2R2 = R2 *R2;

    double subs1 = a1a1 - 2 * a1 *a2 + a2a2 + b1b1 - 2 * b1 *b2 + b2b2;
    double subs2 = -R1R1 *a1 + R1R1 *a2 + R2R2 *a1 - R2R2 *a2 + a1a1 *a1 - a1a1 *a2 - a1 *a2a2 + a1 *b1b1 - 2 * a1 *b1 *b2 + a1 *b2b2 + a2a2 *a2 + a2 *b1b1 - 2 * a2 *b1 *b2 + a2 *b2b2;
    double subs3 = -R1R1 *b1 + R1R1 *b2 + R2R2 *b1 - R2R2 *b2 + a1a1 *b1 + a1a1 *b2 - 2 * a1 *a2 *b1 - 2 * a1 *a2 *b2 + a2a2 *b1 + a2a2 *b2 + b1b1 *b1 - b1b1 *b2 - b1 *b2b2 + b2b2 *b2;
    double sigma = sqrt((R1R1 + 2 * R1 *R2 + R2R2 - a1a1 + 2 * a1 *a2 - a2a2 - b1b1 + 2 * b1 *b2 - b2b2) * (-R1R1 + 2 * R1 *R2 - R2R2 + subs1));
    if(abs(subs1) > 0.0000001)
    {
        intersectP1.y = (subs2 - sigma *b1 + sigma *b2) / (2 * subs1);
        intersectP2.y = (subs2 + sigma *b1 - sigma *b2) / (2 * subs1);
        intersectP1.z = (subs3 + sigma *a1 - sigma *a2) / (2 * subs1);
        intersectP2.z = (subs3 - sigma *a1 + sigma *a2) / (2 * subs1);
        getFullCoord(intersectP1);
        getFullCoord(intersectP2);
        rtn = TRUE;
    }
    else
    {
        rtn = FALSE;
    }

    if(type == 0) //0:上侧
        pOut = (intersectP1.z > intersectP2.z) ? intersectP1 : intersectP2;
    else if(type == 1) //1:下侧
        pOut = (intersectP1.z > intersectP2.z) ? intersectP2 : intersectP1;
    else if(type == 2) //2:左侧
        pOut = (intersectP1.y < intersectP2.y) ? intersectP1 : intersectP2;
    else  //3:右侧
        pOut = (intersectP1.y < intersectP2.y) ? intersectP2 : intersectP1;

    return rtn;
}

//计算直线和圆交点
//type:0/左侧点  1/右侧点   2/上侧点   3/下侧点
int getIntersectPoint(SkLine line, SkPoint3D arcCenter, SkPoint3D &dataOut, int type, double radius)
{
    SkPoint3D p1, p2;
    double A = (line.k *line.k + 1);
    double B = 2 * line.k*(line.b - arcCenter.z) - 2 * arcCenter.y;
    double C = arcCenter.y *arcCenter.y + (line.b - arcCenter.z) * (line.b - arcCenter.z) - radius *radius;
    double temp = B *B - 4 * A *C;
    if(temp >= 0)
    {
        p1.y = (-B - sqrt(temp)) / (2 * A);
        p1.z = line.k *p1.y + line.b;
        getFullCoord(p1);
        p2.y = (-B + sqrt(temp)) / (2 * A);
        p2.z = line.k *p2.y + line.b;
        getFullCoord(p2);
    }
    else
    {
        return FALSE;
    }
    if(type == 0)
    {
        dataOut = p1;
    }
    else if(type == 1)
    {
        dataOut = p2;
    }
    else if(type == 2)
    {
        dataOut = (p1.z > p2.z) ? p1 : p2;
    }
    else
    {
        dataOut = (p1.z > p2.z) ? p2 : p1;
    }
    return TRUE;
}

//计算直线
int getLine(SkPoint3D data1, SkPoint3D data2, SkLine &lineOut)
{
    SkLine line;
    if(fabs(data1.y - data2.y) < EPS)
    {
        line.k = 100000;
    }
    else
    {
        line.k = (data1.z - data2.z) / (data1.y - data2.y);
    }
    line.b = data1.z - line.k *data1.y;
    lineOut = line;
    return TRUE;
}

//计算直线
int getLine(vector<SkPoint3D> dataIn, SkLine &lineOut, int times)
{
    int rtn = TRUE;
    if(dataIn.size() < 2 * times)
    {
        return FALSE;
    }
    else
    {
        SkPoint3D p1, p2;
        for(int i = 0; i < times; i++)
        {
            p1 = p1 + dataIn[i];
            p2 = p2 + dataIn[int(dataIn.size()) - 1 - i];
        }
        p1 = p1 / times;
        p2 = p2 / times;
        rtn = getLine(p1, p2, lineOut);
        return rtn;
    }
}

//计算直线
int getLine(SkPoint3D dataIn, SkLine &lineOut, double k)
{
    int rtn = TRUE;
    lineOut.k = k;
    lineOut.b = dataIn.z - lineOut.k *dataIn.y;
    return rtn;
}

//直线分段
int getLinesByGapAndLength(vector<SkPoint3D> dataIn, SkLine line, vector<vector <SkPoint3D>> &dataListOut, double gapThre, double lengthMin, double lengthMax, int planeNumMin)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    int startIndex = 0, endIndex = 0;
    vector<SkPoint3D> dataArray;
    for(int i = 0; i < int(dataIn.size() - 1); i++)
    {
        double dis = getPointsFootDisToLine(dataIn[i], dataIn[i + 1], line);
        if(dis > gapThre)
        {
            double disAll = getPointsFootDisToLine(dataIn[startIndex], dataIn[i], line);
            if((i - startIndex > planeNumMin) && disAll >= lengthMin && disAll <= lengthMax)
            {
                endIndex = i;
                dataArray.assign(dataIn.begin() + startIndex, dataIn.begin() + endIndex + 1);
                dataListOut.push_back(dataArray);
            }
            startIndex = i + 1;
        }
        else if(i == int(dataIn.size() - 2))
        {
            double disAll = getPointsFootDisToLine(dataIn[startIndex], dataIn.back(), line);
            if((int(dataIn.size()) - 1 - startIndex > planeNumMin) && disAll >= lengthMin && disAll <= lengthMax)
            {
                dataArray.assign(dataIn.begin() + startIndex, dataIn.end());
                dataListOut.push_back(dataArray);
            }
        }
    }
  if(dataListOut.empty())
  {
      return FALSE;
  }
  return TRUE;
}

int getGrooveTurningPoints(std::vector<SkPoint3D> outPoints, std::vector<SkPoint3D>& TurningPoint)
{
    TurningPoint.clear();
    if (outPoints.empty() || outPoints.size() < 50)
    {
        std::cout << "点数不足" << std::endl;
        return FALSE;
    }

    const int smoothWindow = 5;
    const double threshold = 0.2;
    if (outPoints.size() <= static_cast<size_t>(smoothWindow))
    {
        std::cout << "点数不足" << std::endl;
        return FALSE;
    }

    const bool isFrontFirst = (outPoints.front().order < outPoints.back().order);
    (void)isFrontFirst;

    std::vector<SkPoint3D> smoothPoints;
    int posX = 0, negX = 0;
    int posY = 0, negY = 0;
    int posZ = 0, negZ = 0;
    double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;
    double x_var = 0.0, y_var = 0.0, z_var = 0.0;
    for (int i = smoothWindow; i < static_cast<int>(outPoints.size()); ++i)
    {
        double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        for (int j = i - smoothWindow; j < i; ++j)
        {
            sumX += outPoints[j].x;
            sumY += outPoints[j].y;
            sumZ += outPoints[j].z;
        }
        SkPoint3D currPt(sumX / smoothWindow, sumY / smoothWindow, sumZ / smoothWindow);

        if (!smoothPoints.empty())
        {
            const SkPoint3D& prevPt = smoothPoints.back();
            double dx = currPt.x - prevPt.x;
            double dy = currPt.y - prevPt.y;
            double dz = currPt.z - prevPt.z;

            if (fabs(dx) > threshold)
            {
                dx > 0 ? ++posX : ++negX;
            }
            if (fabs(dy) > threshold)
            {
                dy > 0 ? ++posY : ++negY;
            }
            if (fabs(dz) > threshold)
            {
                dz > 0 ? ++posZ : ++negZ;
            }
        }
        x_sum += outPoints[i].x;
        y_sum += outPoints[i].y;
        z_sum += outPoints[i].z;
        smoothPoints.push_back(currPt);
    }

    const int n = static_cast<int>(outPoints.size()) - smoothWindow;
    if (n <= 0)
    {
        std::cout << "点数不足" << std::endl;
        return FALSE;
    }
    for (int i = smoothWindow; i < static_cast<int>(outPoints.size()); ++i)
    {
        x_var += (outPoints[i].x - x_sum / n) * (outPoints[i].x - x_sum / n);
        y_var += (outPoints[i].y - y_sum / n) * (outPoints[i].y - y_sum / n);
        z_var += (outPoints[i].z - z_sum / n) * (outPoints[i].z - z_sum / n);
    }

    enum Axis { AXIS_X, AXIS_Y, AXIS_Z };
    Axis mainAxis;
    Axis secondaryAxis;
    if (abs(posY - negY) >= abs(posX - negX) && abs(posY - negY) >= abs(posZ - negZ))
    {
        mainAxis = AXIS_Y;
        secondaryAxis = x_var > z_var ? AXIS_X : AXIS_Z;
    }
    else if (abs(posX - negX) >= abs(posY - negY) && abs(posX - negX) >= abs(posZ - negZ))
    {
        mainAxis = AXIS_X;
        secondaryAxis = y_var > z_var ? AXIS_Y : AXIS_Z;
    }
    else
    {
        mainAxis = AXIS_Z;
        secondaryAxis = x_var > y_var ? AXIS_X : AXIS_Y;
    }

    std::vector<SkPoint3D> inPoints;
    inPoints.reserve(outPoints.size());
    for (const auto& pt : outPoints)
    {
        if (mainAxis == AXIS_X && secondaryAxis == AXIS_Y)
        {
            inPoints.push_back(SkPoint3D(pt.z, pt.x, pt.y, pt.order));
        }
        else if (mainAxis == AXIS_X && secondaryAxis == AXIS_Z)
        {
            inPoints.push_back(SkPoint3D(pt.y, pt.x, pt.z, pt.order));
        }
        else if (mainAxis == AXIS_Y && secondaryAxis == AXIS_X)
        {
            inPoints.push_back(SkPoint3D(pt.z, pt.y, pt.x, pt.order));
        }
        else if (mainAxis == AXIS_Y && secondaryAxis == AXIS_Z)
        {
            inPoints.push_back(SkPoint3D(pt.x, pt.y, pt.z, pt.order));
        }
        else if (mainAxis == AXIS_Z && secondaryAxis == AXIS_X)
        {
            inPoints.push_back(SkPoint3D(pt.y, pt.z, pt.x, pt.order));
        }
        else if (mainAxis == AXIS_Z && secondaryAxis == AXIS_Y)
        {
            inPoints.push_back(SkPoint3D(pt.x, pt.z, pt.y, pt.order));
        }
        else
        {
            inPoints.push_back(SkPoint3D(pt.x, pt.y, pt.z, pt.order));
        }
    }

    std::vector<SkPoint3D> dataInPlane1, dataOutPlane1;
    std::vector<SkPoint3D> dataInPlane2, dataOutPlane2;
    SkLine fitLine1, fitLine2;
    SkPoint3D midP;
    SkContourInfo Info;
    std::vector<std::vector<SkPoint3D>> dataListOut1, dataListOut2;
    std::vector<SkPoint3D> Splitpoint, Pointcloud;
    std::vector<std::vector<SkPoint3D>> Segmentpointcloud;
    std::vector<SkGroove> Grooves;

    const double planeRatioMin = tan(-85 * PI / 180.0);
    const double planeRatioMax = tan(85 * PI / 180.0);
    int Obtain = SmartLine(inPoints, dataInPlane1, dataOutPlane1, Info,
        fitLine1, midP, 1, planeRatioMin, planeRatioMax, 50, 10000);
    if (Obtain != TRUE)
    {
        std::cout << "第一次获取波峰波谷失败" << std::endl;
        return FALSE;
    }
    Obtain = SmartLine(dataOutPlane1, dataInPlane2, dataOutPlane2, Info,
        fitLine2, midP, 1, planeRatioMin, planeRatioMax, 50, 10000);
    if (Obtain != TRUE)
    {
        std::cout << "第二次获取波峰波谷失败" << std::endl;
        return FALSE;
    }
    getLinesByGapAndLength(dataInPlane1, fitLine1, dataListOut1, 50, 5, 200, 20);
    getLinesByGapAndLength(dataInPlane2, fitLine2, dataListOut2, 50, 5, 200, 20);

    if (!dataListOut1.empty())
    {
        for (const auto& ListOut : dataListOut1)
        {
            Splitpoint.push_back(ListOut.front());
            Splitpoint.push_back(ListOut.back());
        }

        for (const auto& ListOut : dataListOut2)
        {
            Splitpoint.push_back(ListOut.front());
            Splitpoint.push_back(ListOut.back());
        }
    }
    else
    {
        std::cout << "第一次/第二次分段失败" << std::endl;
        return FALSE;
    }

    std::sort(Splitpoint.begin(), Splitpoint.end(), [](const SkPoint3D& a, const SkPoint3D& b) {
        return a.y < b.y;
        });
    for (int i = 0; i < static_cast<int>(Splitpoint.size()) - 1; i++)
    {
        Pointcloud.clear();
        getDataByCoordY(inPoints, Pointcloud, Splitpoint[i].y, Splitpoint[i + 1].y);
        Segmentpointcloud.push_back(Pointcloud);
    }

    for (int i = 0; i < static_cast<int>(Segmentpointcloud.size()); i++)
    {
        std::vector<SkPoint3D> dataIn, dataOut;
        SkLine Linesegment;
        SkPoint3D Midpoint;
        const double minRatio = tan(-85 * PI / 180.0);
        const double maxRatio = tan(85 * PI / 180.0);

        int rtn = SmartLine(Segmentpointcloud[i], dataIn, dataOut, Info,
            Linesegment, Midpoint, 0.5, minRatio, maxRatio, 15, 10000);
        if (rtn == TRUE)
        {
            SkGroove newBorder;
            newBorder.dataIn.resize(dataIn.size());
            std::copy(dataIn.begin(), dataIn.end(), newBorder.dataIn.begin());

            const auto& pLeft = (dataIn.front().y < dataIn.back().y) ? dataIn.front() : dataIn.back();
            const auto& pRight = (dataIn.front().y > dataIn.back().y) ? dataIn.front() : dataIn.back();
            newBorder.leftEndP = pLeft;
            newBorder.rightEndP = pRight;
            newBorder.leftLine = Linesegment;
            newBorder.grooveP = Midpoint;
            newBorder.grooveType = 0;
            Grooves.push_back(newBorder);
        }
    }

    if (Grooves.empty())
    {
        std::cout << "未识别到有效焊道段" << std::endl;
        return FALSE;
    }

    std::vector<SkPoint3D> sortedTurningPoints;
    sortedTurningPoints.push_back(Grooves.front().leftEndP);
    for (int i = 0; i < static_cast<int>(Grooves.size()) - 1; i++)
    {
        SkPoint3D intersectP;
        getIntersectPoint(Grooves[i].leftLine, Grooves[i + 1].leftLine, intersectP);

        SkPoint3D realTurnPoint = intersectP;
        double minDist = 1e9;
        Pointcloud.clear();
        getDataByCoordY(inPoints, Pointcloud, Grooves[i].leftEndP.y, Grooves[i + 1].rightEndP.y);
        if (!Pointcloud.empty())
        {
            for (const auto& p : Pointcloud)
            {
                const double dy = p.y - intersectP.y;
                const double dz = p.z - intersectP.z;
                const double dist = dy * dy + dz * dz;
                if (dist < minDist)
                {
                    minDist = dist;
                    realTurnPoint = p;
                }
            }
        }
        sortedTurningPoints.push_back(realTurnPoint);
    }
    sortedTurningPoints.push_back(Grooves.back().rightEndP);

    for (const auto& pt : sortedTurningPoints)
    {
        if (mainAxis == AXIS_X && secondaryAxis == AXIS_Y)
        {
            TurningPoint.push_back(SkPoint3D(pt.y, pt.z, pt.x, pt.order));
        }
        else if (mainAxis == AXIS_X && secondaryAxis == AXIS_Z)
        {
            TurningPoint.push_back(SkPoint3D(pt.y, pt.x, pt.z, pt.order));
        }
        else if (mainAxis == AXIS_Y && secondaryAxis == AXIS_X)
        {
            TurningPoint.push_back(SkPoint3D(pt.z, pt.y, pt.x, pt.order));
        }
        else if (mainAxis == AXIS_Y && secondaryAxis == AXIS_Z)
        {
            TurningPoint.push_back(SkPoint3D(pt.x, pt.y, pt.z, pt.order));
        }
        else if (mainAxis == AXIS_Z && secondaryAxis == AXIS_X)
        {
            TurningPoint.push_back(SkPoint3D(pt.z, pt.x, pt.y, pt.order));
        }
        else if (mainAxis == AXIS_Z && secondaryAxis == AXIS_Y)
        {
            TurningPoint.push_back(SkPoint3D(pt.x, pt.z, pt.y, pt.order));
        }
        else
        {
            TurningPoint.push_back(SkPoint3D(pt.x, pt.y, pt.z, pt.order));
        }
    }

    std::sort(TurningPoint.begin(), TurningPoint.end(), [](const SkPoint3D& a, const SkPoint3D& b) {
        return a.order < b.order;
        });
    return TRUE;
}

//最小二乘法计算直线
//r:相关系数   ->1:相关性大  ->0:相关性小
int getLineByLeastSquares(vector<SkPoint3D> dataIn, SkLine &lineOut, double &r)
{
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;
    double E = 0.0;
    double F = 0.0;
    double temp = 0;

    for (int i = 0; i < dataIn.size(); i++)
    {
        A += dataIn[i].y *dataIn[i].y;
        B += dataIn[i].y;
        C += dataIn[i].y *dataIn[i].z;
        D += dataIn[i].z;
    }
    temp = dataIn.size() * A - B *B;

    if( fabs(temp) > EPS)
    {
        lineOut.k = (dataIn.size() * C - B *D) / temp;
        lineOut.b = (A *D - B *C) / temp;
    }
    else
    {
        lineOut.k = 1;
        lineOut.b = 0;
        return FALSE;
    }

    // 计算相关系数r
    double yMean = 0, zMean = 0;
    yMean = B / dataIn.size();
    zMean = D / dataIn.size();

    float tempSumXX = 0.0, tempSumYY = 0.0;
    for (int i = 0; i < dataIn.size(); i++)
    {
        tempSumXX += (dataIn[i].y - yMean) * (dataIn[i].y - yMean);
        tempSumYY += (dataIn[i].z - zMean) * (dataIn[i].z - zMean);
        E += (dataIn[i].y - yMean) * (dataIn[i].z - zMean);
    }
    F = sqrt(tempSumXX) * sqrt(tempSumYY);
    r = E / F;
    return TRUE;
}


bool getLineParaByPcl(SkPoint3D *sourceData, SkLine line, SkPoint3D &midP, SkLine &fitLine, int numOfData,  double matchThre)
{
    int numOfFit = 0;
    double ySum = 0, zSum = 0, yySum = 0, yzSum = 0, yMean = 0, zMean = 0;
    double limitDz = matchThre *sqrt(1 + line.k *line.k);
    for(int i = 0; i < numOfData; i++)
    {
        double mismatchZ = sourceData[i].z - line.b - line.k *sourceData[i].y;
        if(fabs(mismatchZ) < limitDz)
        {
            numOfFit++;
            ySum = ySum + sourceData[i].y;
            zSum = zSum + sourceData[i].z;
            yySum = yySum + sourceData[i].y *sourceData[i].y;
            yzSum = yzSum + sourceData[i].y *sourceData[i].z;
        }
    }
    if(numOfFit == 0)
        return FALSE;
    midP.y = ySum / numOfFit;
    midP.z = zSum / numOfFit;
    getFullCoord(midP);
    fitLine.k = (yzSum - ySum *zSum / numOfFit) / (yySum - ySum *ySum / numOfFit);
    fitLine.b = (midP.z - fitLine.k *midP.y);
    return TRUE;
}

bool getCircleParaByThreePoints(SkPoint3D firstP, SkPoint3D secondP, SkPoint3D thirdP, SkPoint3D &circleOut, double &radius)
{
    SkPoint3D centerP;
    double y2 = secondP.y - firstP.y;
    double z2 = secondP.z - firstP.z;
    double y3 = thirdP.y - firstP.y;
    double z3 = thirdP.z - firstP.z;
    double temp = 2 * (y2 *z3 - z2 *y3);
    if(temp < EPS)
    {
        return FALSE;
    }
    centerP.y = (z3 *y2 *y2 + z3 *z2 *z2 - z2 *y3 *y3 - z2 *z3 *z3) / temp;
    centerP.z = (-y3 *y2 *y2 - y3 *z2 *z2 + y2 *y3 *y3 + y2 *z3 *z3) / temp;
    radius = sqrt(centerP.y *centerP.y + centerP.z *centerP.z);
    circleOut.y = centerP.y + firstP.y;
    circleOut.z = centerP.z + firstP.z;
    getFullCoord(circleOut);
    return TRUE;
}

bool getCircleParaByPcl(SkPoint3D *sourceData, SkPoint3D circle, double radius, SkPoint3D &fitCircle, double &fitRadius, int numOfData,  double matchThre)
{
    if(numOfData < 3)
    {
        return false;
    }
    int N = 0;
    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    for (int i = 0; i < numOfData; i++)
    {
        double delta = getDisFromPointToPoint(sourceData[i], circle) - radius;
        if(fabs(delta) < matchThre)
        {
            double x = sourceData[i].y;
            double y = sourceData[i].z;
            double x2 = x *x;
            double y2 = y *y;
            sum_x += x;
            sum_y += y;
            sum_x2 += x2;
            sum_y2 += y2;
            sum_x3 += x2 *x;
            sum_y3 += y2 *y;
            sum_xy += x *y;
            sum_x1y2 += x *y2;
            sum_x2y1 += x2 *y;
            N++;
        }
    }
    double C, D, E, G, H;
    double a, b, c;

    C = N *sum_x2 - sum_x *sum_x;
    D = N *sum_xy - sum_x *sum_y;
    E = N *sum_x3 + N *sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N *sum_y2 - sum_y *sum_y;
    H = N *sum_x2y1 + N *sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H *D - E *G) / (C *G - D *D);
    b = (H *C - E *D) / (D *D - G *C);
    c = -(a *sum_x + b *sum_y + sum_x2 + sum_y2) / N;

    fitCircle.y = a / (-2);
    fitCircle.z = b / (-2);
    getFullCoord(fitCircle);
    fitRadius = sqrt(a *a + b *b - 4 * c) / 2;
    return true;
}
//1个点24，你要想靠这个赚上一百个
//计算点到直线的最大距离
//type:0/两侧 1/点在直线上侧  2/点在直线下侧
int getMaxDisFromPointToLine(vector<SkPoint3D> dataIn, SkLine line, SkPoint3D &dataOut, int type, double &maxDis)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    double dis = -100;
    maxDis = -100;
    if(type == 0)
    {
        for(int i = 0; i < dataIn.size(); i++)
        {
            dis = getDisFromPointToLine(dataIn[i], line);
            if(dis > maxDis)
            {
                dataOut = dataIn[i];
                maxDis = dis;
            }
        }
    }
    else if(type == 1)
    {
        for(int i = 0; i < dataIn.size(); i++)
        {
            dis = getDisFromPointToLine2(dataIn[i], line);
            if(dis > maxDis)
            {
                dataOut = dataIn[i];
                maxDis = dis;
            }
        }
    }
    else
    {
        for(int i = 0; i < dataIn.size(); i++)
        {
            dis = -getDisFromPointToLine2(dataIn[i], line);
            if(dis > maxDis)
            {
                dataOut = dataIn[i];
                maxDis = dis;
            }
        }
    }
    return TRUE;
}

//计算点到圆弧的最大距离
//type:0/两侧 1/点在圆弧内  2/点在圆弧外
int getMaxDisFromPointToArc(vector<SkPoint3D> dataIn, SkPoint3D arcCenter, SkPoint3D &dataOut, int type, double radius, double &maxDis)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    double dis = -100;
    maxDis = -100;
    if(type == 0)
    {
        for(int i = 0; i < dataIn.size(); i++)
        {
            dis = fabs(getDisFromPointToPoint(dataIn[i], arcCenter) - radius);
            if(dis > maxDis)
            {
                dataOut = dataIn[i];
                maxDis = dis;
            }
        }
    }
    else if(type == 1)
    {
        for(int i = 0; i < dataIn.size(); i++)
        {
            dis = radius - getDisFromPointToPoint(dataIn[i], arcCenter);
            if(dis > maxDis)
            {
                dataOut = dataIn[i];
                maxDis = dis;
            }
        }
    }
    else
    {
        for(int i = 0; i < dataIn.size(); i++)
        {
            dis = getDisFromPointToPoint(dataIn[i], arcCenter) - radius;
            if(dis > maxDis)
            {
                dataOut = dataIn[i];
                maxDis = dis;
            }
        }
    }
    return TRUE;
}

//计算点离圆弧表面的最大距离
double getMaxDisFromPointToArc(SkPoint3D data, SkPoint3D arcCenter, SkPoint3D &projection, double radius)
{
    SkPoint3D p1, p2;
    SkLine line;
    SkPoint3D projection1, projection2;
    getLine(data, arcCenter, line);
    getIntersectPoint(line, arcCenter, projection1, 0, radius);
    getIntersectPoint(line, arcCenter, projection2, 1, radius);
    double dis1 = getDisFromPointToPoint(projection1, data);
    double dis2 = getDisFromPointToPoint(projection2, data);
    if(dis1 > dis2)
    {
        projection = projection1;
        return dis1;
    }
    else
    {
        projection = projection2;
        return dis2;
    }
}

int getMaxDisFromPointByPoint(vector<SkPoint3D> dataIn, SkLine line, SkPoint3D &p1, SkPoint3D &p2, double &maxDis)
{
    if(dataIn.empty())
    {
        return FALSE;
    }
    double dis = 0, temp = 0;
    int index1 = 0, index2 = 0;
    for(int i = 0; i < int(dataIn.size() - 1); i++)
    {
        temp = getPointsFootDisToLine(dataIn[i], dataIn[i + 1], line);
        if(temp > dis)
        {
            dis = temp;
            index1 = i;
            index2 = i + 1;
        }
    }
    p1 = dataIn[index1];
    p2 = dataIn[index2];
    maxDis = dis;
    return TRUE;
}

int getMaxDisFromPointByPoint(vector<SkPoint3D> dataIn, SkLine line, int startIndex, int endIndex, SkPoint3D &p1, SkPoint3D &p2, double &maxDis)
{
    double dis = 0, temp = 0;
    int index1 = 0, index2 = 0;
    if(dataIn.empty())
    {
        return FALSE;
    }
    if(startIndex == endIndex)
    {
        return FALSE;
    }
    if(startIndex < 0 || startIndex > int(dataIn.size() - 1))
    {
        return FALSE;
    }
    if(endIndex < 0 || endIndex > int(dataIn.size() - 1))
    {
        return FALSE;
    }
    if(endIndex > startIndex)
    {
        for(int i = startIndex; i < endIndex; i++)
        {
            temp = getPointsFootDisToLine(dataIn[i], dataIn[i + 1], line);
            if(temp > dis)
            {
                dis = temp;
                index1 = i;
                index2 = i + 1;
            }
        }
        p1 = dataIn[index1];
        p2 = dataIn[index2];
        maxDis = dis;
    }
    else
    {
        for(int i = startIndex; i > endIndex; i--)
        {
            temp = getPointsFootDisToLine(dataIn[i], dataIn[i - 1], line);
            if(temp > dis)
            {
                dis = temp;
                index1 = i - 1;
                index2 = i;
            }
        }
        p1 = dataIn[index1];
        p2 = dataIn[index2];
        maxDis = dis;
    }
    return TRUE;
}

//计算均值
int getMean(vector<SkPoint3D> dataIn, SkPoint3D &dataOut)
{
    SkPoint3D sum;
    for(int i = 0; i < dataIn.size(); i++)
    {
        sum = sum + dataIn[i];
    }
    dataOut = sum / int(dataIn.size());
    return TRUE;
}

//计算点离圆弧表面的最小距离
double getMinDisFromPointToArc(SkPoint3D data, SkPoint3D arcCenter, SkPoint3D &projection, double radius)
{
    SkPoint3D p1, p2;
    SkLine line;
    SkPoint3D projection1, projection2;
    getLine(data, arcCenter, line);
    getIntersectPoint(line, arcCenter, projection1, 0, radius);
    getIntersectPoint(line, arcCenter, projection2, 1, radius);
    double dis1 = getDisFromPointToPoint(projection1, data);
    double dis2 = getDisFromPointToPoint(projection2, data);
    if(dis1 > dis2)
    {
        projection = projection2;
        return dis2;
    }
    else
    {
        projection = projection1;
        return dis1;
    }
}

//根据坐标值值截取点云   dimension:0-x值  1:y值  2:z值
int getPartOfDataByCorrd(vector<SkPoint3D> data, vector<SkPoint3D> &dataRes, double minValue, double maxValue, int dimension)
{
    if(data.empty())
    {
        return FALSE;
    }
    for(int i = 0; i < data.size(); i++)
    {
        double value;
        value = data[i].p[dimension];
        if(value > minValue && value <= maxValue)
        {
            dataRes.push_back(data[i]);
        }
    }
    if(dataRes.size() == 0)
    {
        return FALSE;
    }
    return TRUE;
}

//根据索引号截取点云
int getPartOfDataByIndex(vector<SkPoint3D> data, vector<SkPoint3D> &dataRes, int startIndex, int endIndex, int dir)
{
    if(data.empty())
    {
        return FALSE;
    }
    vector<SkPoint3D>::iterator startIt, endIt;
    if(startIndex == -1)
    {
        startIt = data.begin();
    }
    else
    {
        startIt = data.begin() + startIndex;
    }
    if(endIndex == -1)
    {
        endIt = data.end();
    }
    else
    {
        endIt = data.begin() + endIndex;
    }
    dataRes.assign(startIt, endIt);
    if(dir != 0)
    {
        std::reverse(dataRes.begin(), dataRes.end());
    }
    return TRUE;
}

//计算line2上的且与line1的距离为vertH的点的坐标
//type:0/点在交点上方  1/点在交点下方  2/点在交点左侧  3/点在交点右侧
int getPointByVertHAndIntersection(SkLine line1, SkLine line2, SkPoint3D &dataOut, int type, double vertH)
{
    int rtn = TRUE;
    SkPoint3D intersectP;
    rtn = getIntersectPoint(line1, line2, intersectP);
    if(rtn == FALSE)
    {
        return FALSE;
    }
    double cosThea = (1 + line1.k *line2.k) / sqrt((1 + line1.k *line1.k) * (1 + line2.k *line2.k));
    double length = vertH / sqrt(1 - cosThea *cosThea);
    if(fabs(line2.k) < EPS)
    {
        if(type == 0 || type == 1)
        {
            return FALSE;
        }
        else if(type == 2)
        {
            dataOut.y = intersectP.y - length / sqrt(1 + line2.k *line2.k);
            dataOut.z = line2.k *dataOut.y + line2.b;
        }
        else
        {
            dataOut.y = intersectP.y + length / sqrt(1 + line2.k *line2.k);
            dataOut.z = line2.k *dataOut.y + line2.b;
        }
    }
    else
    {
        if(type == 0)
        {
            dataOut.z = intersectP.z + fabs(line2.k *length / sqrt(1 + line2.k *line2.k));
            dataOut.y = (dataOut.z - line2.b) / line2.k;
        }
        else if(type == 1)
        {
            dataOut.z = intersectP.z - fabs(line2.k *length / sqrt(1 + line2.k *line2.k));
            dataOut.y = (dataOut.z - line2.b) / line2.k;
        }
        else if(type == 2)
        {
            dataOut.y = intersectP.y - length / sqrt(1 + line2.k *line2.k);
            dataOut.z = line2.k *dataOut.y + line2.b;
        }
        else if(type == 3)
        {
            dataOut.y = intersectP.y + length / sqrt(1 + line2.k *line2.k);
            dataOut.z = line2.k *dataOut.y + line2.b;
        }
    }
    getFullCoord(dataOut);
    return TRUE;

}

//根据高度差、平面、垂足点求取点
//type:0/求的点在直线上方  1/求的点在直线下方
int getPointByVertPAndLineAndDis(SkPoint3D vertP, SkPoint3D &pOut, SkLine line, double dis, int type)
{
    if(fabs(line.k) < EPS)
    {
        if(type == 0)
        {
            pOut = vertP;
            pOut.z += dis;
        }
        else
        {
            pOut = vertP;
            pOut.z -= dis;
        }
    }
    else
    {
        double k = -1.0 / line.k;
        double b = vertP.z - k *vertP.y;
        SkPoint3D p1, p2;
        p1.y = vertP.y + dis / sqrt(1 + k *k);
        p1.z = vertP.z + k *dis / sqrt(1 + k *k);
        p2.y = vertP.y - dis / sqrt(1 + k *k);
        p2.z = vertP.z - k *dis / sqrt(1 + k *k);
        if(p1.z > p2.z)
        {
            pOut = (type == 0) ? p1 : p2;
        }
        else
        {
            pOut = (type == 0) ? p2 : p1;
        }
    }
    return TRUE;
}

//计算两点在直线上的投影距离
double getPointsFootDisToLine(SkPoint3D data1, SkPoint3D data2, SkLine line)
{
    SkPoint3D delta = data1 - data2;
    double dis = (delta.y + delta.z *line.k) / sqrt(1 + line.k *line.k);
    return fabs(dis);
}

double getPointsMismatchDisToLine(SkPoint3D data1, SkPoint3D data2, SkLine line)
{
    double footDis = getPointsFootDisToLine(data1, data2, line);
    double totalDis = getDisFromPointToPoint(data1, data2);
    double mismatchDis = sqrt(totalDis *totalDis - footDis *footDis);
    return mismatchDis;
}

//计算垂直线
int getVerticalLine(SkPoint3D dataIn, SkLine lineIn, SkLine &lineOut)
{
    int rtn = TRUE;
    if(lineIn.k == 0)
    {
        lineOut.k = 10000;
        rtn = FALSE;
    }
    else
    {
        lineOut.k = -1 / lineIn.k;
        rtn = TRUE;
    }
    lineOut.b = dataIn.z - lineOut.k *dataIn.y;
    return rtn;
}

//拟合直线
int SmartLine(vector<SkPoint3D> sourceData, vector<SkPoint3D> &fitData, vector<SkPoint3D> &unFitData, SkContourInfo &infoOut,
              SkLine &fitLine, SkPoint3D &midP, double matchThre, double ratioMin, double ratioMax, int planeNumMin, int outlinerNumMax)
{
    int rtn = FALSE;
    if(sourceData.empty())
    {
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        infoOut.errId = -1;
        sprintf(infoOut.errInfo, "总点数为0,请确认设定的焊缝类型与实际是否相符!");
        return -1;
    }
    if(sourceData.size() < planeNumMin)
    {
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        infoOut.errId = -2;
        sprintf(infoOut.errInfo, QObject::tr("总点数为(%d),请确认设定的焊缝类型与实际是否相符!").toStdString().c_str(), sourceData.size());
        return -2;
    }
    fitData.erase(fitData.begin(), fitData.end());
    unFitData.erase(unFitData.begin(), unFitData.end());
    int recycleNum = 100;
    int index0BestOK = -1, index1BestOK = -1, index0BestNG = -1, index1BestNG = -1;
    int numBestOK = -1, numBestNG = -1;
    int numOfSourceData = sourceData.size();
    int indexMax = numOfSourceData;
    double ratio = 0;
    SkPoint3D *pSourceData = (SkPoint3D *)malloc(sizeof (SkPoint3D) * numOfSourceData);
    for(int i = 0; i < numOfSourceData; i++)
    {
        pSourceData[i].y = sourceData[i].y;
        pSourceData[i].z = sourceData[i].z;
    }
    srand((unsigned)time(NULL));
    for(int i = 0; i < recycleNum; i++)
    {
        int numFit = 0;
        int index0 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        int index1 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        if(index0 >= indexMax)
            index0 = indexMax - 1;
        if(index1 >= indexMax)
            index1 = indexMax - 1;
        index1 = (index0 == index1) ? (index0 + 20) % indexMax : index1;
        if(fabs(pSourceData[index1].y - pSourceData[index0].y) > EPS)
            ratio = (pSourceData[index1].z - pSourceData[index0].z) / (pSourceData[index1].y - pSourceData[index0].y);
        else
            ratio = 100000;
        double interupt = pSourceData[index0].z - ratio *pSourceData[index0].y;
        double limitZ = sqrt(1 + ratio *ratio) * matchThre;
        for(int j = 0; j < numOfSourceData; j++)
        {
            double deltaZ = pSourceData[j].z - ratio *pSourceData[j].y - interupt;
            if(fabs(deltaZ) < limitZ)
                numFit++;
        }
        if(numFit > numBestOK)
        {
            if(ratio >= ratioMin && ratio <= ratioMax)
            {
                numBestOK = numFit;
                index0BestOK = index0;
                index1BestOK = index1;
            }
        }
        if(numFit > numBestNG)
        {
            numBestNG = numFit;
            index0BestNG = index0;
            index1BestNG = index1;
        }
    }
    SkPoint3D rtnMidP1, rtnMidP2, rtnMidP3;
    SkLine rtnLine1, rtnLine2, rtnLine3;
    if(numBestOK > 0)
    {
        SkLine checkLine;
        checkLine.k = (pSourceData[index1BestOK].z - pSourceData[index0BestOK].z) / (pSourceData[index1BestOK].y - pSourceData[index0BestOK].y);
        checkLine.b = pSourceData[index0BestOK].z - checkLine.k*(pSourceData[index0BestOK].y);
        rtn = getLineParaByPcl(pSourceData, checkLine, rtnMidP1, rtnLine1, numOfSourceData, matchThre);
        if(rtn == FALSE)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -3;
            sprintf(infoOut.errInfo, "检测第一次校验出现错误!");
            return -3;
        }
        rtn = getLineParaByPcl(pSourceData, rtnLine1, rtnMidP2, rtnLine2, numOfSourceData, matchThre);
        if(rtn == FALSE)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -4;
            sprintf(infoOut.errInfo, "检测第二次校验出现错误!");
            return -4;
        }
        double limitZ = sqrt(1 + rtnLine2.k *rtnLine2.k) * matchThre;
        for(int i = 0; i < numOfSourceData; i++)
        {
            double deltaZ = pSourceData[i].z - rtnLine2.k *pSourceData[i].y - rtnLine2.b;
            if(fabs(deltaZ) < limitZ)
                fitData.push_back(sourceData[i]);
            else
                unFitData.push_back(sourceData[i]);
        }
        if(fitData.size() < planeNumMin)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -5;
            sprintf(infoOut.errInfo, QObject::tr("检测到的点数太少(%d)!").toStdString().c_str(), fitData.size());
            return -5;
        }
        if(unFitData.size() > outlinerNumMax)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -6;
            sprintf(infoOut.errInfo, QObject::tr("检测到不符合的点数太多(%d)!").toStdString().c_str(), fitData.size());
            return -6;
        }
        if(rtnLine2.k < ratioMin || rtnLine2.k > ratioMax)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -8;
            sprintf(infoOut.errInfo, QObject::tr("检测到的角度不在设定范围内(%.3f)!").toStdString().c_str(), atan(rtnLine2.k) * 180.0 / PI);
            return -8;
        }
        else
        {
            midP = rtnMidP2;
            fitLine = rtnLine2;
            free(pSourceData);
            return TRUE;
        }
    }
    if(index0BestNG == -1 || index1BestNG == -1)
    {
        sprintf(infoOut.errInfo, QObject::tr("index0BestNG==0/index1BestNG==0!").toStdString().c_str());
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        free(pSourceData);
        infoOut.errId = -9;
        return -9;
    }
    SkLine checkLineNG;
    checkLineNG.k = (pSourceData[index1BestNG].z - pSourceData[index0BestNG].z) / (pSourceData[index1BestNG].y - pSourceData[index0BestNG].y);
    checkLineNG.b = pSourceData[index0BestNG].z - checkLineNG.k*(pSourceData[index0BestNG].y);
    rtn = getLineParaByPcl(pSourceData, checkLineNG, rtnMidP3, rtnLine3, numOfSourceData, matchThre);
    midP = rtnMidP3;
    fitLine = rtnLine3;
    fitData.erase(fitData.begin(), fitData.end());
    unFitData.assign(sourceData.begin(), sourceData.end());
    free(pSourceData);
    infoOut.errId = -7;
    sprintf(infoOut.errInfo, QObject::tr("检测到的角度不在设定范围内(%.3f)!").toStdString().c_str(), atan(fitLine.k) * 180.0 / PI);
    return -7;
}

//拟合直线
int SmartLine2(vector<SkPoint3D> sourceData, vector<SkPoint3D> &fitData, vector<SkPoint3D> &unFitData, SkContourInfo &infoOut,
               SkLine &fitLine, SkPoint3D &midP, double matchThre, double ratioMinusMax, double ratioPlusMin, int planeNumMin, int outlinerNumMax)
{
    int rtn = FALSE;
    if(sourceData.empty())
    {
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        infoOut.errId = -1;
        sprintf(infoOut.errInfo, "总点数为0,请确认设定的焊缝类型与实际是否相符!");
        return -1;
    }
    if(sourceData.size() < planeNumMin)
    {
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        infoOut.errId = -2;
        sprintf(infoOut.errInfo, QObject::tr("总点数为(%d),请确认设定的焊缝类型与实际是否相符!").toStdString().c_str(), sourceData.size());
        return -2;
    }
    fitData.erase(fitData.begin(), fitData.end());
    unFitData.erase(unFitData.begin(), unFitData.end());
    int recycleNum = 100;
    int index0BestOK = -1, index1BestOK = -1, index0BestNG = -1, index1BestNG = -1;
    int numBestOK = -1, numBestNG = -1;
    int numOfSourceData = sourceData.size();
    int indexMax = numOfSourceData;
    double ratio = 0;
    SkPoint3D *pSourceData = (SkPoint3D *)malloc(sizeof (SkPoint3D) * numOfSourceData);
    for(int i = 0; i < numOfSourceData; i++)
    {
        pSourceData[i].y = sourceData[i].y;
        pSourceData[i].z = sourceData[i].z;
    }
    srand((unsigned)time(NULL));
    for(int i = 0; i < recycleNum; i++)
    {
        int numFit = 0;
        int index0 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        int index1 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        if(index0 >= indexMax)
            index0 = indexMax - 1;
        if(index1 >= indexMax)
            index1 = indexMax - 1;
        index1 = (index0 == index1) ? (index0 + 20) % indexMax : index1;
        if(fabs(pSourceData[index1].y - pSourceData[index0].y) > EPS)
            ratio = (pSourceData[index1].z - pSourceData[index0].z) / (pSourceData[index1].y - pSourceData[index0].y);
        else
            ratio = 100000;
        double interupt = pSourceData[index0].z - ratio *pSourceData[index0].y;
        double limitZ = sqrt(1 + ratio *ratio) * matchThre;
        for(int j = 0; j < numOfSourceData; j++)
        {
            double deltaZ = pSourceData[j].z - ratio *pSourceData[j].y - interupt;
            if(fabs(deltaZ) < limitZ)
                numFit++;
        }
        if(numFit > numBestOK)
        {
            if(((ratio <= ratioMinusMax) && (ratio < 0)) ||
                    ((ratio >= ratioPlusMin) && (ratio > 0)))
            {
                numBestOK = numFit;
                index0BestOK = index0;
                index1BestOK = index1;
            }
        }
        if(numFit > numBestNG)
        {
            numBestNG = numFit;
            index0BestNG = index0;
            index1BestNG = index1;
        }
    }
    SkPoint3D rtnMidP1, rtnMidP2, rtnMidP3;
    SkLine rtnLine1, rtnLine2, rtnLine3;
    if(numBestOK > 0)
    {
        SkLine checkLine;
        checkLine.k = (pSourceData[index1BestOK].z - pSourceData[index0BestOK].z) / (pSourceData[index1BestOK].y - pSourceData[index0BestOK].y);
        checkLine.b = pSourceData[index0BestOK].z - checkLine.k*(pSourceData[index0BestOK].y);
        rtn = getLineParaByPcl(pSourceData, checkLine, rtnMidP1, rtnLine1, numOfSourceData, matchThre);
        if(rtn == FALSE)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -3;
            sprintf(infoOut.errInfo, "检测第一次校验出现错误!");
            return -3;
        }
        rtn = getLineParaByPcl(pSourceData, rtnLine1, rtnMidP2, rtnLine2, numOfSourceData, matchThre);
        if(rtn == FALSE)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -4;
            sprintf(infoOut.errInfo, "检测第二次校验出现错误!");
            return -4;
        }
        double limitZ = sqrt(1 + rtnLine2.k *rtnLine2.k) * matchThre;
        for(int i = 0; i < numOfSourceData; i++)
        {
            double deltaZ = pSourceData[i].z - rtnLine2.k *pSourceData[i].y - rtnLine2.b;
            if(fabs(deltaZ) < limitZ)
                fitData.push_back(sourceData[i]);
            else
                unFitData.push_back(sourceData[i]);
        }
        if(fitData.size() < planeNumMin)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -5;
            sprintf(infoOut.errInfo, QObject::tr("检测到的点数太少(%d)!").toStdString().c_str(), fitData.size());
            return -5;
        }
        if(unFitData.size() > outlinerNumMax)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -6;
            sprintf(infoOut.errInfo, QObject::tr("检测到不符合的点数太多(%d)!").toStdString().c_str(), fitData.size());
            return -6;
        }
        if(((rtnLine2.k > ratioMinusMax) && (ratio < 0)) ||
                ((rtnLine2.k < ratioPlusMin) && (ratio > 0)))
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -8;
            sprintf(infoOut.errInfo, QObject::tr("检测到的角度不在设定范围内(%.3f)!").toStdString().c_str(), atan(rtnLine2.k) * 180.0 / PI);
            return -8;
        }
        else
        {
            midP = rtnMidP2;
            fitLine = rtnLine2;
            free(pSourceData);
            return TRUE;
        }
    }
    if(index0BestNG == -1 || index1BestNG == -1)
    {
        sprintf(infoOut.errInfo, QObject::tr("index0BestNG==0/index1BestNG==0!").toStdString().c_str());
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        free(pSourceData);
        infoOut.errId = -9;
        return -9;
    }
    SkLine checkLineNG;
    checkLineNG.k = (pSourceData[index1BestNG].z - pSourceData[index0BestNG].z) / (pSourceData[index1BestNG].y - pSourceData[index0BestNG].y);
    checkLineNG.b = pSourceData[index0BestNG].z - checkLineNG.k*(pSourceData[index0BestNG].y);
    rtn = getLineParaByPcl(pSourceData, checkLineNG, rtnMidP3, rtnLine3, numOfSourceData, matchThre);
    midP = rtnMidP3;
    fitLine = rtnLine3;
    fitData.erase(fitData.begin(), fitData.end());
    unFitData.assign(sourceData.begin(), sourceData.end());
    free(pSourceData);
    infoOut.errId = -7;
    sprintf(infoOut.errInfo, QObject::tr("检测到的角度不在设定范围内(%.3f)!").toStdString().c_str(), atan(fitLine.k) * 180.0 / PI);
    return -7;
}

//拟合圆弧
int SmartCircle(vector<SkPoint3D> sourceData, vector<SkPoint3D> &fitData, vector<SkPoint3D> &unFitData, SkContourInfo &infoOut,
                SkPoint3D &fitCircle, double &fitRadius, double matchThre, double radiusMin, double radiusMax, int planeNumMin, int outlinerNumMax)
{
    int rtn = FALSE;
    SkPoint3D threeP[3];
    SkPoint3D fitCircle1, fitCircle2;
    double fitRadius1, fitRadius2;
    if(sourceData.empty())
    {
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        infoOut.errId = -1;
        sprintf(infoOut.errInfo, "总点数为0,请确认设定的焊缝类型与实际是否相符!");
        return -1;
    }
    if(sourceData.size() < planeNumMin)
    {
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        infoOut.errId = -2;
        sprintf(infoOut.errInfo, QObject::tr("总点数为(%d),请确认设定的焊缝类型与实际是否相符!").toStdString().c_str(), sourceData.size());
        return -2;
    }
    fitData.erase(fitData.begin(), fitData.end());
    unFitData.erase(unFitData.begin(), unFitData.end());
    int recycleNum = 100;
    int index0BestOK = -1, index1BestOK = -1, index2BestOK = -1, index0BestNG = -1, index1BestNG = -1, index2BestNG = -1;
    int numBestOK = -1, numBestNG = -1;
    int numOfSourceData = sourceData.size();
    int indexMax = numOfSourceData;
    SkPoint3D *pSourceData = (SkPoint3D *)malloc(sizeof (SkPoint3D) * numOfSourceData);
    for(int i = 0; i < numOfSourceData; i++)
    {
        pSourceData[i].y = sourceData[i].y;
        pSourceData[i].z = sourceData[i].z;
    }
    srand((unsigned)time(NULL));
    for(int i = 0; i < recycleNum; i++)
    {
        int numFit = 0;
        int index0 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        int index1 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        int index2 = (int)(((double)rand() / (double)RAND_MAX) * indexMax + 0.5);
        if(index1 == index0)
            index1 = (index0 + 15) % indexMax;
        if(index1 > index0)
        {
            if(index2 == index0)
                index2 = (index0 + 15) % indexMax;
            if(index2 == index1)
                index2 = (index1 + 15) % indexMax;
        }
        else
        {
            if(index2 == index1)
                index2 = (index1 + 15) % indexMax;
            if(index2 == index0)
                index2 = (index0 + 15) % indexMax;
        }
        if(index0 == index1 || index0 == index2 || index1 == index2)
            continue;


        rtn = getCircleParaByThreePoints(pSourceData[index0], pSourceData[index1], pSourceData[index2], fitCircle1, fitRadius1);
        rtn = getCircleParaByPcl(pSourceData, fitCircle1, fitRadius1, fitCircle2, fitRadius2, numOfSourceData, matchThre);
        for(int j = 0; j < numOfSourceData; j++)
        {
            double delta = getDisFromPointToPoint(pSourceData[j], fitCircle2) - fitRadius2;
            if(fabs(delta) < matchThre)
                numFit++;
        }
        if(numFit > numBestOK)
        {
            if(fitRadius2 >= radiusMin && fitRadius2 <= radiusMax)
            {
                numBestOK = numFit;
                index0BestOK = index0;
                index1BestOK = index1;
                index2BestOK = index2;
            }
        }
        if(numFit > numBestNG)
        {
            numBestNG = numFit;
            index0BestNG = index0;
            index1BestNG = index1;
            index2BestNG = index2;
        }
    }
    SkPoint3D rtnCircle1, rtnCircle2, rtnCircle3;
    double rtnRadius1, rtnRadius2, rtnRadius3;
    if(numBestOK > 0)
    {
        SkPoint3D checkCircle;
        double checkRadius = 0;
        rtn = getCircleParaByThreePoints(pSourceData[index0BestOK], pSourceData[index1BestOK], pSourceData[index2BestOK], checkCircle, checkRadius);
        rtn = getCircleParaByPcl(pSourceData, checkCircle, checkRadius, rtnCircle1, rtnRadius1, numOfSourceData, matchThre);
        if(rtn == FALSE)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -3;
            sprintf(infoOut.errInfo, "检测第一次校验出现错误!");
            return -3;
        }
        rtn = getCircleParaByPcl(pSourceData, rtnCircle1, rtnRadius1, rtnCircle2, rtnRadius2, numOfSourceData, matchThre);
        if(rtn == FALSE)
        {
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            infoOut.errId = -4;
            sprintf(infoOut.errInfo, "检测第二次校验出现错误!");
            return -4;
        }
        for(int i = 0; i < numOfSourceData; i++)
        {
            double delta = getDisFromPointToPoint(pSourceData[i], rtnCircle2) - rtnRadius2;
            if(fabs(delta) < matchThre)
                fitData.push_back(sourceData[i]);
            else
                unFitData.push_back(sourceData[i]);
        }
        if(rtnRadius2 < radiusMin || rtnRadius2 > radiusMax)
        {
            infoOut.errId = -10;
            sprintf(infoOut.errInfo, QObject::tr("检测到的半径不在设定范围内(%.3f)!").toStdString().c_str(), fitRadius);
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            return -10;
        }
        if(fitData.size() < planeNumMin)
        {
            infoOut.errId = -5;
            sprintf(infoOut.errInfo, QObject::tr("检测到的点数太少(%d)!").toStdString().c_str(), fitData.size());
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            return -5;
        }
        if(unFitData.size() > outlinerNumMax)
        {
            infoOut.errId = -6;
            sprintf(infoOut.errInfo, QObject::tr("检测到不符合的点数太多(%d)!").toStdString().c_str(), fitData.size());
            fitData.erase(fitData.begin(), fitData.end());
            unFitData.assign(sourceData.begin(), sourceData.end());
            free(pSourceData);
            return -6;
        }
        fitCircle = rtnCircle2;
        fitRadius = rtnRadius2;
        free(pSourceData);
        return TRUE;
    }
    SkPoint3D checkCircleNG;
    double checkRadiusNG;
    getCircleParaByThreePoints(pSourceData[index0BestNG], pSourceData[index1BestNG], pSourceData[index2BestNG], checkCircleNG, checkRadiusNG);
    fitCircle = checkCircleNG;
    fitRadius = checkRadiusNG;
    if(fitRadius < radiusMin || fitRadius > radiusMax)
    {
        infoOut.errId = -7;
        sprintf(infoOut.errInfo, QObject::tr("检测到的半径不在设定范围内(%.3f)!").toStdString().c_str(), fitRadius);
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        free(pSourceData);
        return -7;
    }
    for(int i = 0; i < numOfSourceData; i++)
    {
        double delta = getDisFromPointToPoint(pSourceData[i], checkCircleNG) - checkRadiusNG;
        if(fabs(delta) < matchThre)
            fitData.push_back(sourceData[i]);
        else
            unFitData.push_back(sourceData[i]);
    }
    if(fitData.size() < planeNumMin)
    {
        infoOut.errId = -8;
        sprintf(infoOut.errInfo, QObject::tr("检测到的圆点数不在设定范围内(%d)!").toStdString().c_str(), fitData.size());
        fitData.erase(fitData.begin(), fitData.end());
        unFitData.assign(sourceData.begin(), sourceData.end());
        free(pSourceData);
        return -8;
    }
    fitData.erase(fitData.begin(), fitData.end());
    unFitData.assign(sourceData.begin(), sourceData.end());
    free(pSourceData);
    infoOut.errId = -9;
    sprintf(infoOut.errInfo, "出现其他错误!");
    return -9;
}


int getConcaveGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double gapMin, double gapMax, double sunkenMin, int maxGroove)
{
    int rtn = TRUE;
    SkGroove groove;
    bool inGrooveFlag = false;
    if(sourceData.empty())
    {
        return FALSE;
    }
    int numOfData = sourceData.size();
    for(int i = 0; i < numOfData; i++)
    {
        double dis = getDisFromPointToLine2(sourceData[i], line, true);
        if(dis >= sunkenMin && inGrooveFlag == false)
        {
            if(i == 0)
            {
                groove.leftEndP = sourceData[i];
            }
            else
            {
                groove.leftEndP = sourceData[i - 1];
            }
            inGrooveFlag = TRUE;
        }
        else if((dis < sunkenMin) && (inGrooveFlag == TRUE))
        {
            inGrooveFlag = FALSE;
            double disOnLine = getPointsFootDisToLine(sourceData[i], groove.leftEndP, line);
            if(disOnLine > gapMin && disOnLine < gapMax)
            {
                groove.rightEndP = sourceData[i];
                groove.grooveType = 1;

                if(grooveOut.size() > maxGroove)
                {
                    info.errId = -15;
                    sprintf(info.errInfo, QObject::tr("满足凹陷条件的坡口太多(%d)!").toStdString().c_str(), grooveOut.size());
                    return FALSE;
                }
//                qDebug()<<"getConcaveGroove leftEndP=";
//                groove.leftEndP.print();
//                qDebug()<<"getConcaveGroove rightEndP=";
//                groove.rightEndP.print();
                grooveOut.push_back(groove);
            }
        }
    }
    return TRUE;
}

int getBulgeGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double gapMin, double gapMax, double bulgeMin, int maxGroove)
{
    int rtn = TRUE;
    SkGroove groove;
    bool inGrooveFlag = false;
    if(sourceData.empty())
    {
        return FALSE;
    }
    int numOfData = sourceData.size();
    for(int i = 0; i < numOfData; i++)
    {
        double dis = getDisFromPointToLine2(sourceData[i], line, false);
        if(dis >= bulgeMin && inGrooveFlag == false)
        {
            if(i == 0)
            {
                groove.leftEndP = sourceData[i];
            }
            else
            {
                groove.leftEndP = sourceData[i - 1];
            }
            inGrooveFlag = TRUE;
        }
        else if((dis < bulgeMin) && (inGrooveFlag == TRUE))
        {
            inGrooveFlag = FALSE;
            double disOnLine = getPointsFootDisToLine(sourceData[i], groove.leftEndP, line);
            if(disOnLine > gapMin && disOnLine < gapMax)
            {
                groove.rightEndP = sourceData[i];
                groove.grooveType = 1;

                if(grooveOut.size() > maxGroove)
                {
                    info.errId = -15;
                    sprintf(info.errInfo, QObject::tr("满足凸起条件的坡口太多(%d)!").toStdString().c_str(), grooveOut.size());
                    return FALSE;
                }
                grooveOut.push_back(groove);
            }
        }
    }
    return TRUE;
}

int getGapGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double gapMin, double gapMax, int maxGroove)
{
    int rtn = TRUE;
    SkGroove groove;
    if(sourceData.empty())
    {
        return FALSE;
    }
    int numOfData = sourceData.size();
    for(int i = 1; i < numOfData; i++)
    {
        double disOnLine = getPointsFootDisToLine(sourceData[i], sourceData[i - 1], line);
        if(disOnLine > gapMin && disOnLine < gapMax)
        {
//            qDebug()<<"getGapGroove data1=";
//            sourceData[i].print();
//            qDebug()<<"getGapGroove data2=";
//            sourceData[i-1].print();
            groove.leftEndP = sourceData[i - 1];
            groove.rightEndP = sourceData[i];
            groove.grooveType = 2;

            if(grooveOut.size() > maxGroove)
            {
                info.errId = -16;
                sprintf(info.errInfo, QObject::tr("满足间隙条件的坡口太多(%d)!").toStdString().c_str(), grooveOut.size());
                return FALSE;
            }
            grooveOut.push_back(groove);
        }
    }
    return TRUE;
}

int getMismatchGroove(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double ratioMin, double ratioMax, double mismatchMin, double mismatchMax, double matchThre, double disMatchThre, int planeNumMin, int outlinerNumMax)
{
    int rtn = TRUE;
    SkGroove groove;
    vector<SkPoint3D> fitData1, unfitData1, fitData2, unfitData2;
    vector<SkPoint3D> leftPlaneData, rightPlaneData;
    SkLine line1, line2;
    SkPoint3D midPoint1, midPoint2;
    SkPoint3D leftEndPoint, rightEndPoint;
    if(sourceData.empty())
    {
        return FALSE;
    }
    rtn = SmartLine(sourceData, fitData1, unfitData1, info, line1, midPoint1, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
    if(rtn != TRUE)
    {
        info.errId = -100 + info.errId;
        sprintf(info.errInfo, (QObject::tr("错边第一个表面") + info.errInfo).toStdString().c_str());
        return FALSE;
    }
    rtn = SmartLine(unfitData1, fitData2, unfitData2, info, line2, midPoint2, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
    if(rtn != TRUE)
    {
        info.errId = -110 + info.errId;
        sprintf(info.errInfo, (QObject::tr("错边第二个表面") + info.errInfo).toStdString().c_str());
        return FALSE;
    }
    if(midPoint1.y < midPoint2.y)
    {
        int findFlag = FALSE;
        findFlag = getEdgeByLineAndVertHAndDisMatch(sourceData, line1, leftEndPoint, 0, 0, disMatchThre, mismatchMin, mismatchMax, 10);
        if(findFlag == FALSE)
        {
            info.errId = -111 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边左边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        if(fitData1.back().order != leftEndPoint.order)
        {
            getDataByOrder(sourceData, leftPlaneData, sourceData.front().order, leftEndPoint.order);
            rtn = SmartLine(leftPlaneData, fitData1, unfitData1, info, line1, midPoint1, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
            if(rtn != TRUE)
            {
                info.errId = -130 + info.errId;
                sprintf(info.errInfo, (QObject::tr("错边左表面") + info.errInfo).toStdString().c_str());
                return FALSE;
            }
        }
        getDataByOrder(sourceData, rightPlaneData, leftEndPoint.order + 1, sourceData.back().order);
        rtn = SmartLine(rightPlaneData, fitData2, unfitData2, info, line2, midPoint2, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
        if(rtn != TRUE)
        {
            info.errId = -140 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边右表面") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        findFlag = getEdgeByLineAndVertHAndDisMatch(sourceData, line2, rightEndPoint, 0, 1, disMatchThre, mismatchMin, mismatchMax, 10);
        if(findFlag == FALSE)
        {
            info.errId = -112 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边右边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        groove.leftLine = line1;
        groove.rightLine = line2;
        groove.leftEndP = leftEndPoint;
        groove.leftEndP.z = line1.k *groove.leftEndP.y + line1.b;
        groove.rightEndP = rightEndPoint;
        groove.rightEndP.z = line2.k *groove.rightEndP.y + line2.b;
        groove.grooveType = 3;
    }
    else
    {
        int findFlag = FALSE;
        findFlag = getEdgeByLineAndVertHAndDisMatch(sourceData, line2, leftEndPoint, 0, 0, disMatchThre, mismatchMin, mismatchMax, 10);
        if(findFlag == FALSE)
        {
            info.errId = -115 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边左边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        if(fitData2.back().order != leftEndPoint.order)
        {
            getDataByOrder(sourceData, leftPlaneData, sourceData.front().order, leftEndPoint.order);
            rtn = SmartLine(leftPlaneData, fitData2, unfitData2, info, line2, midPoint2, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
            if(rtn != TRUE)
            {
                info.errId = -150 + info.errId;
                sprintf(info.errInfo, (QObject::tr("错边左表面") + info.errInfo).toStdString().c_str());
                return FALSE;
            }
        }
        getDataByOrder(sourceData, rightPlaneData, leftEndPoint.order + 1, sourceData.back().order);
        rtn = SmartLine(rightPlaneData, fitData1, unfitData1, info, line1, midPoint1, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
        if(rtn != TRUE)
        {
            info.errId = -160 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边右表面") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        findFlag = getEdgeByLineAndVertHAndDisMatch(sourceData, line1, rightEndPoint, 0, 1, disMatchThre, mismatchMin, mismatchMax, 10);
        if(findFlag == FALSE)
        {
            info.errId = -116 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边右边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        groove.leftLine = line2;
        groove.rightLine = line1;
        groove.leftEndP = leftEndPoint;
        groove.leftEndP.z = line2.k *groove.leftEndP.y + line2.b;
        groove.rightEndP = rightEndPoint;
        groove.rightEndP.z = line1.k *groove.rightEndP.y + line1.b;
        groove.grooveType = 3;
    }
    grooveOut.push_back(groove);
    return TRUE;
}

int getMismatchGroove2(vector<SkPoint3D> sourceData, SkLine line, SkContourInfo &info, vector<SkGroove> &grooveOut, double ratioMin, double ratioMax, double gapMaxInPlane, double matchThre, double disMatchThre, int planeNumMin, int outlinerNumMax)
{
    int rtn = TRUE;
    SkGroove groove;
    vector<SkPoint3D> fitData1, unfitData1, fitData2, unfitData2;
    vector<SkPoint3D> leftPlaneData, rightPlaneData;
    SkLine line1, line2;
    SkPoint3D midPoint1, midPoint2;
    SkPoint3D leftEndPoint, rightEndPoint;
    if(sourceData.empty())
    {
        return FALSE;
    }
    rtn = SmartLine(sourceData, fitData1, unfitData1, info, line1, midPoint1, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
    if(rtn != TRUE)
    {
        info.errId = -100 + info.errId;
        sprintf(info.errInfo, (QObject::tr("错边第一个表面") + info.errInfo).toStdString().c_str());
        return FALSE;
    }
    rtn = SmartLine(unfitData1, fitData2, unfitData2, info, line2, midPoint2, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
    if(rtn != TRUE)
    {
        info.errId = -110 + info.errId;
        sprintf(info.errInfo, (QObject::tr("错边第二个表面") + info.errInfo).toStdString().c_str());
        return FALSE;
    }
    if(midPoint1.y < midPoint2.y)
    {
        rtn = getLineEndPoint(fitData1, line1, leftEndPoint, 0, int(fitData1.size() - 1), gapMaxInPlane);
        if(rtn != TRUE)
        {
            info.errId = -121;
            sprintf(info.errInfo, (QObject::tr("错边左边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        if(fitData1.back().order != leftEndPoint.order)
        {
            getDataByOrder(sourceData, leftPlaneData, sourceData.front().order, leftEndPoint.order);
            rtn = SmartLine(leftPlaneData, fitData1, unfitData1, info, line1, midPoint1, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
            if(rtn != TRUE)
            {
                info.errId = -130 + info.errId;
                sprintf(info.errInfo, (QObject::tr("错边左表面") + info.errInfo).toStdString().c_str());
                return FALSE;
            }
        }
        getDataByOrder(sourceData, rightPlaneData, leftEndPoint.order + 1, sourceData.back().order);
        rtn = SmartLine(rightPlaneData, fitData2, unfitData2, info, line2, midPoint2, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
        if(rtn != TRUE)
        {
            info.errId = -140 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边右表面") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        rtn = getLineEndPoint(fitData2, line2, rightEndPoint, int(fitData2.size() - 1), 0, gapMaxInPlane);
        if(rtn != TRUE)
        {
            info.errId = -122;
            sprintf(info.errInfo, (QObject::tr("错边右边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        groove.leftLine = line1;
        groove.rightLine = line2;
        groove.leftEndP = leftEndPoint;
        groove.leftEndP.z = line1.k *groove.leftEndP.y + line1.b;
        groove.rightEndP = rightEndPoint;
        groove.rightEndP.z = line2.k *groove.rightEndP.y + line2.b;
        groove.grooveType = 3;
    }
    else
    {
        int findFlag = FALSE;
        rtn = getLineEndPoint(fitData2, line2, leftEndPoint, 0, int(fitData2.size() - 1), gapMaxInPlane);
        if(rtn != TRUE)
        {
            info.errId = -123;
            sprintf(info.errInfo, (QObject::tr("错边左边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        if(fitData2.back().order != leftEndPoint.order)
        {
            getDataByOrder(sourceData, leftPlaneData, sourceData.front().order, leftEndPoint.order);
            rtn = SmartLine(leftPlaneData, fitData2, unfitData2, info, line2, midPoint2, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
            if(rtn != TRUE)
            {
                info.errId = -150 + info.errId;
                sprintf(info.errInfo, (QObject::tr("错边左表面") + info.errInfo).toStdString().c_str());
                return FALSE;
            }
        }
        getDataByOrder(sourceData, rightPlaneData, leftEndPoint.order + 1, sourceData.back().order);
        rtn = SmartLine(rightPlaneData, fitData1, unfitData1, info, line1, midPoint1, matchThre, ratioMin, ratioMax, planeNumMin, outlinerNumMax);
        if(rtn != TRUE)
        {
            info.errId = -160 + info.errId;
            sprintf(info.errInfo, (QObject::tr("错边右表面") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        rtn = getLineEndPoint(fitData1, line1, rightEndPoint, int(fitData1.size() - 1), 0, gapMaxInPlane);
        if(rtn != TRUE)
        {
            info.errId = -124;
            sprintf(info.errInfo, (QObject::tr("错边右边界查找失败") + info.errInfo).toStdString().c_str());
            return FALSE;
        }
        groove.leftLine = line2;
        groove.rightLine = line1;
        groove.leftEndP = leftEndPoint;
        groove.leftEndP.z = line2.k *groove.leftEndP.y + line2.b;
        groove.rightEndP = rightEndPoint;
        groove.rightEndP.z = line1.k *groove.rightEndP.y + line1.b;
        groove.grooveType = 3;
    }
    grooveOut.push_back(groove);
    return TRUE;
}

int getLineEndPoint(vector<SkPoint3D> sourceData, SkLine line, SkPoint3D &endPoint, int start, int finish, double gapMaxInPlane)
{
    if(sourceData.empty())
    {
        return FALSE;
    }
    if(int(sourceData.size() - 1) < start || int(sourceData.size() - 1) < finish)
    {
        return FALSE;
    }
    endPoint = sourceData[finish];
    if(start < finish)
    {
        for(int i = start; i < finish; i++)
        {
            double disOnLine = getPointsFootDisToLine(sourceData[i + 1], sourceData[i], line);
            if(disOnLine > gapMaxInPlane)
            {
                endPoint = sourceData[i];
            }
        }
    }
    else
    {
        for(int i = start; i > finish; i--)
        {
            double disOnLine = getPointsFootDisToLine(sourceData[i], sourceData[i - 1], line);
            if(disOnLine > gapMaxInPlane)
            {
                endPoint = sourceData[i];
            }
        }
    }
    return TRUE;
}

int getMaxDistanceFromPointsToLine(vector<SkPoint3D> sourceData, SkLine line, SkPoint3D &maxDisP, int ptUp, double &maxDis)
{
    double temp = -2000;
    SkPoint3D tempP = maxDisP;
    int maxDisIndex = 0;
    int findFlag = FALSE;
    if(sourceData.empty())
    {
        return FALSE;
    }
    int numOfPoint = (int)sourceData.size();
    for(int i = 0; i < numOfPoint; i++)
    {
        temp = getDisFromPointToLine2(sourceData[i], line, !ptUp);
        if(temp > maxDis)
        {
            maxDis = temp;
            maxDisIndex = i;
            findFlag = TRUE;
        }
    }
    if(findFlag == TRUE)
    {
        maxDisP = sourceData[maxDisIndex];
    }
    return findFlag;
}

//target:输出点   lineA：输出点到该线的距离为dis, lineB:输出点在该线上
//type:0-输出点在交点的上侧，1-输出点在交点第的下侧，2-输出点在交点的左侧，3-输出点在交点的右侧
int getPointByHeightAndLine(SkLine lineA, SkLine lineB, SkPoint3D &target, int type, double dis)
{
    int rtn = TRUE;
    SkPoint3D intersectP;
    getIntersectPoint(lineA, lineB, intersectP);
    double cosThre = (lineA.k *lineB.k + 1) / sqrt((1 + lineA.k *lineA.k) * (1 + lineB.k *lineB.k));
    double disOnB = dis / sqrt(1 - cosThre *cosThre);
    double ratioLength = sqrt(1 + lineB.k *lineB.k);
    double disY = fabs(disOnB / ratioLength);
    double disZ = fabs(disOnB *lineB.k / ratioLength);
    if(type == 0)
    {
        target.z = intersectP.z + disZ;
        target.y = (target.z - lineB.b) / lineB.k;
    }
    else if(type == 1)
    {
        target.z = intersectP.z - disZ;
        target.y = (target.z - lineB.b) / lineB.k;
    }
    else if(type == 2)
    {
        target.y = intersectP.y - disY;
        target.z = lineB.k *target.y + lineB.b;
    }
    else
    {
        target.y = intersectP.y + disY;
        target.z = lineB.k *target.y + lineB.b;
    }
    getFullCoord(target);
    return rtn;
}

int getDataByRotate(vector<SkPoint3D> sourceData, vector<SkPoint3D> &resultData, double rotateAngle, int option)
{
    if(sourceData.empty())
        return FALSE;
    if(option == 1)
        rotateAngle = rotateAngle *PI / 180.0;
    float sinAngle = sin(rotateAngle);
    float cosAngle = cos(rotateAngle);
    resultData.assign(sourceData.begin(), sourceData.end());
    for(int i = 0; i < sourceData.size(); i++)
    {
        resultData[i].y = sourceData[i].y *cosAngle - sourceData[i].z *sinAngle;
        resultData[i].z = sourceData[i].y *sinAngle + sourceData[i].z *cosAngle;
    }
    return TRUE;
}

int getDataByRotate2(vector<SkPoint3D> sourceData, vector<SkPoint3D> &resultData, double rotateAngle, SkPoint3D centerP, int option)
{
    if(sourceData.empty())
        return FALSE;
    if(option == 1)
        rotateAngle = rotateAngle *PI / 180.0;
    float sinAngle = sin(rotateAngle);
    float cosAngle = cos(rotateAngle);
    resultData.assign(sourceData.begin(), sourceData.end());
    for(int i = 0; i < sourceData.size(); i++)
    {
        resultData[i].y = (sourceData[i].y - centerP.y) * cosAngle - (sourceData[i].z - centerP.z) * sinAngle + centerP.y;
        resultData[i].z = (sourceData[i].y - centerP.y) * sinAngle + (sourceData[i].z - centerP.z) * cosAngle + centerP.z;
    }
    return TRUE;
}

int getDataByRotate(SkVector6d sourceData, SkVector6d &resultData, double rotateAngle, int option)
{
    if(option == 1)
        rotateAngle = rotateAngle *PI / 180.0;
    float sinAngle = sin(rotateAngle);
    float cosAngle = cos(rotateAngle);
    resultData = sourceData;
    resultData.y = sourceData.y *cosAngle - sourceData.z *sinAngle;
    resultData.z = sourceData.y *sinAngle + sourceData.z *cosAngle;
    return TRUE;
}

int getDataByRotate(SkPoint3D sourceData, SkPoint3D &resultData, double rotateAngle, int option)
{
    if(option == 1)
        rotateAngle = rotateAngle *PI / 180.0;
    float sinAngle = sin(rotateAngle);
    float cosAngle = cos(rotateAngle);
    resultData = sourceData;
    resultData.y = sourceData.y *cosAngle - sourceData.z *sinAngle;
    resultData.z = sourceData.y *sinAngle + sourceData.z *cosAngle;
    return TRUE;
}

int getDataByRotate2(SkPoint3D sourceData, SkPoint3D &resultData, double rotateAngle, SkPoint3D centerP, int option)
{
    if(option == 1)
        rotateAngle = rotateAngle *PI / 180.0;
    float sinAngle = sin(rotateAngle);
    float cosAngle = cos(rotateAngle);
    resultData = sourceData;
    resultData.y = (sourceData.y - centerP.y) * cosAngle - (sourceData.z - centerP.z) * sinAngle + centerP.y;
    resultData.z = (sourceData.y - centerP.y) * sinAngle + (sourceData.z - centerP.z) * cosAngle + centerP.z;
    return TRUE;
}

void reOrderPCL(std::vector<SkPoint3D> &mPCL, int reOrder)
{
    if(reOrder == TRUE)
    {
        std::reverse(mPCL.begin(), mPCL.end());
        for(int i = 0; i < mPCL.size(); i++)
        {
            mPCL[i].y = -mPCL[i].y;
            mPCL[i].order = i;
        }
    }
}

std::vector<SkPoint3D> reOrderPCL2(std::vector<SkPoint3D> mPCL, int reOrder)
{
    std::vector<SkPoint3D> pclOut;
    pclOut.assign(mPCL.begin(), mPCL.end());
    if(reOrder == TRUE)
    {
        std::reverse(pclOut.begin(), pclOut.end());
        for(int i = 0; i < pclOut.size(); i++)
        {
            pclOut[i].y = -pclOut[i].y;
            pclOut[i].order = i;
        }
    }
    return pclOut;
}

SkPlaneProperty getPlainZProperty(SkPoint3D *origin, int length, double tolerance)
{
    SkPlaneProperty property;
    int isPlainFlag = FALSE;
    double sumY = 0, sumZ = 0, minValue = 1000, maxValue = -1000;
    for(int i = 0; i < length; i++)
    {
        sumY = sumY + origin[i].y;
        sumZ = sumZ + origin[i].z;
        if(minValue > origin[i].z)
            minValue = origin[i].z;
        if(maxValue < origin[i].z)
            maxValue = origin[i].z;
    }
    property.linePos.y = sumY / length;
    property.linePos.z = sumZ / length;
    property.leftP = origin[0];
    property.rightP = origin[length - 1];
    property.dPlainTolerance = maxValue - minValue;
    if(property.dPlainTolerance > tolerance)
    {
        isPlainFlag = FALSE;
    }
    else
    {
        isPlainFlag = TRUE;
    }
    property.iPlainExist = isPlainFlag;
    return property;
}

SkPlaneProperty getPlainYProperty(SkPoint3D *origin, int length, double tolerance)
{
    SkPlaneProperty property;
    int isPlainFlag = FALSE;
    double sumY = 0, sumZ = 0, minValue = 1000, maxValue = -1000;
    for(int i = 0; i < length; i++)
    {
        sumY = sumY + origin[i].y;
        sumZ = sumZ + origin[i].z;
        if(minValue > origin[i].y)
            minValue = origin[i].y;
        if(maxValue < origin[i].y)
            maxValue = origin[i].y;
    }
    property.linePos.y = sumY / length;
    property.linePos.z = sumZ / length;
    property.leftP = origin[0];
    property.rightP = origin[length - 1];
    property.dPlainTolerance = maxValue - minValue;
    if(property.dPlainTolerance > tolerance)
    {
        isPlainFlag = FALSE;
    }
    else
    {
        isPlainFlag = TRUE;
    }
    property.iPlainExist = isPlainFlag;
    return property;
}

SkPlaneProperty getPlainZProperty(SkPoint3D *origin, int length, double tolerance, double gapMax)
{
    SkPlaneProperty property;
    int isPlainFlag = FALSE;
    double sumY = 0, sumZ = 0, minValue = 1000, maxValue = -1000, realMaxGap = -1000;
    for(int i = 0; i < length; i++)
    {
        sumY = sumY + origin[i].y;
        sumZ = sumZ + origin[i].z;
        if(minValue > origin[i].z)
            minValue = origin[i].z;
        if(maxValue < origin[i].z)
            maxValue = origin[i].z;
    }
    for(int i = 0; i < length - 1; i++)
    {
        double realGap = fabs(origin[i + 1].y - origin[i].y);
        if(realMaxGap < realGap)
        {
            realMaxGap = realGap;
        }
    }
    property.linePos.y = sumY / length;
    property.linePos.z = sumZ / length;
    property.leftP = origin[0];
    property.rightP = origin[length - 1];
    property.dPlainTolerance = maxValue - minValue;
    if(property.dPlainTolerance > tolerance || realMaxGap > gapMax)
    {
        isPlainFlag = FALSE;
    }
    else
    {
        isPlainFlag = TRUE;
    }
    property.iPlainExist = isPlainFlag;
    return property;
}

int getButtGroove(int sheetType, SkPoint3D *origin, int pSize, double dPlainMaxDifHeight, double gapMin, double gapMax, double heightMin, double heightMax, int plainPointsNum, SkScanAlgCmdDet *scanAlgCmdDet, double angleBottom, SkContourInfo &info)
{
    int rtn = FALSE;
    int  plainPointsNum43 = plainPointsNum / 3 * 4;
    SkPlaneProperty planeProperty;
    int iMinPlainPos = 0;
    double dMinHeight = 0;
    int iMaxPlainPos = 0;
    double dMaxHeight = 0;
    SkPoint3D minPlain3D;
    SkPoint3D maxPlain3D;
    bool bLowestPlainFound = false;
    bool bFoundMaxPlain = false;
    SkScanResult scanResult;
    int iPlainCount = 0;
    std::vector<SkPlaneProperty> plainVector;
    std::vector<double>  dPlainHeightArray;//保存平台的高度
    std::vector<double>  dPlainPosArray;//保存平台的位置
    std::vector<int>  dPlainOrderArray;//保存平台的order
    double dTempHeightDif = 0;
    double dTempPosDif = 0;

    SkPlaneProperty planeYProperty;
    std::vector<SkPlaneProperty> planeYPropertyVector;

    SkPoint3D lowestP, lowestP2, highestP, highestP2;
    //qDebug()<<QString("getButtGroove:gapMin=%1,gapMax=%2").arg(gapMin).arg(gapMax)<<" plainPointsNum="<<plainPointsNum<<" dPlainMaxDifHeight="<<dPlainMaxDifHeight;
    for(int i = 2; i < pSize - plainPointsNum; i++)
    {
        planeProperty = getPlainZProperty(origin + i, plainPointsNum, dPlainMaxDifHeight, gapMin);
        if(planeProperty.iPlainExist)
        {
            plainVector.push_back(planeProperty);
        }
    }
    //qDebug()<<"plainVector.size()="<<plainVector.size();
    for(int i = 0; i < int(plainVector.size() - 1); i++)
    {
        double realGap = plainVector[i + 1].leftP.y - plainVector[i].rightP.y;
        double realMismatch = fabs(plainVector[i + 1].linePos.z - plainVector[i].linePos.z);
        if(realGap > gapMin && realGap < gapMax && realMismatch > heightMin && realMismatch < heightMax) //对接搭接全符合
        {
            rtn = true;
            scanResult.iGotResult = TRUE;
            scanResult.leftPlaneLeftP = plainVector[i].leftP;
            scanResult.leftPlaneRightP = plainVector[i].rightP;
            scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
            scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
            scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z; //是否添加
            scanResult.leftPlaneRightP.z = plainVector[i].linePos.z; //是否添加
            scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z; //是否添加
            scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z; //是否添加
            scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
            scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
            scanResult.iOrder = i;
            scanResult.algoFlag = 1;
            scanAlgCmdDet->algoCounts[0]++;
            scanAlgCmdDet->scanResultVector.push_back(scanResult);
            scanAlgCmdDet->scanResCount++;
            if(scanAlgCmdDet->scanResultVector.size() >= 10)
            {
                info.errId = -55;
                sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                return FALSE;
            }
        }
        else if(realGap > gapMin && realGap < gapMax && realMismatch < heightMax) //符合对接
        {
            rtn = true;
            scanResult.iGotResult = TRUE;
            scanResult.leftPlaneLeftP = plainVector[i].leftP;
            scanResult.leftPlaneRightP = plainVector[i].rightP;
            scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
            scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
            scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z; //是否添加
            scanResult.leftPlaneRightP.z = plainVector[i].linePos.z; //是否添加
            scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z; //是否添加
            scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z; //是否添加
            scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
            scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
            scanResult.iOrder = i;
            scanResult.algoFlag = 2;
            scanAlgCmdDet->algoCounts[1]++;
            scanAlgCmdDet->scanResultVector.push_back(scanResult);
            scanAlgCmdDet->scanResCount++;
            if(scanAlgCmdDet->scanResultVector.size() >= 10)
            {
                info.errId = -56;
                sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                return FALSE;
            }
        }
        else if(realGap < gapMax && realMismatch > heightMin && realMismatch < heightMax) //符合搭接
        {
            rtn = true;
            scanResult.iGotResult = TRUE;
            scanResult.leftPlaneLeftP = plainVector[i].leftP;
            scanResult.leftPlaneRightP = plainVector[i].rightP;
            scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
            scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
            scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z; //是否添加
            scanResult.leftPlaneRightP.z = plainVector[i].linePos.z; //是否添加
            scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z; //是否添加
            scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z; //是否添加
            scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
            scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
            scanResult.iOrder = i;
            scanResult.algoFlag = 3;
            scanAlgCmdDet->algoCounts[2]++;
            scanAlgCmdDet->scanResultVector.push_back(scanResult);
            scanAlgCmdDet->scanResCount++;
            if(scanAlgCmdDet->scanResultVector.size() >= 10)
            {
                info.errId = -57;
                sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                return FALSE;
            }
        }
    }
    return rtn;
}

int getLapGroove(int sheetType, SkPoint3D *origin, int pSize, double dPlainMaxDifHeight, double gapMax, double heightMin, double heightMax, int plainPointsNum, SkScanAlgCmdDet *scanAlgCmdDet, double angleBottom, SkContourInfo &info)
{
    int rtn = FALSE;
    int  plainPointsNum43 = plainPointsNum / 3 * 4;
    SkPlaneProperty planeProperty;
    int iMinPlainPos = 0;
    double dMinHeight = 0;
    int iMaxPlainPos = 0;
    double dMaxHeight = 0;
    SkPoint3D minPlain3D;
    SkPoint3D maxPlain3D;
    bool bLowestPlainFound = false;
    bool bFoundMaxPlain = false;
    SkScanResult scanResult;
    int iPlainCount = 0;
    std::vector<SkPlaneProperty> plainVector;
    std::vector<double>  dPlainHeightArray;//保存平台的高度
    std::vector<double>  dPlainPosArray;//保存平台的位置
    std::vector<int>  dPlainOrderArray;//保存平台的order
    double dTempHeightDif = 0;
    double dTempPosDif = 0;

    SkPlaneProperty planeYProperty;
    std::vector<SkPlaneProperty> planeYPropertyVector;

    SkPoint3D lowestP, lowestP2, highestP, highestP2;
    //qDebug()<<QString("getButtGroove:gapMin=%1,gapMax=%2").arg(gapMin).arg(gapMax)<<" plainPointsNum="<<plainPointsNum<<" dPlainMaxDifHeight="<<dPlainMaxDifHeight;
    for(int i = 2; i < pSize - plainPointsNum; i++)
    {
        planeProperty = getPlainZProperty(origin + i, plainPointsNum, dPlainMaxDifHeight);
        if(planeProperty.iPlainExist)
        {
            plainVector.push_back(planeProperty);
        }
    }
    for(int i = 0; i < int(plainVector.size() - 1); i++)
    {
        double realGap = plainVector[i + 1].leftP.y - plainVector[i].rightP.y;
        double realMismatch = plainVector[i + 1].linePos.z - plainVector[i].linePos.z;;
        if(sheetType == 0 && realGap < gapMax && realMismatch > heightMin && realMismatch < heightMax) //右在左上 符合搭接
        {
            rtn = true;
            scanResult.iGotResult = TRUE;
            scanResult.leftPlaneLeftP = plainVector[i].leftP;
            scanResult.leftPlaneRightP = plainVector[i].rightP;
            scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
            scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
            scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z;
            scanResult.leftPlaneRightP.z = plainVector[i].linePos.z;
            scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z;
            scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z;
            scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
            scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
            scanResult.iOrder = i;
            scanResult.algoFlag = 1;
            scanAlgCmdDet->algoCounts[2]++;
            scanAlgCmdDet->scanResultVector.push_back(scanResult);
            scanAlgCmdDet->scanResCount++;
            if(scanAlgCmdDet->scanResultVector.size() >= 10)
            {
                info.errId = -67;
                sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                return FALSE;
            }
        }
        else if(sheetType == 1 && realGap < gapMax && (-realMismatch) > heightMin && (-realMismatch) < heightMax) //左在右上 符合搭接
        {
            rtn = true;
            scanResult.iGotResult = TRUE;
            scanResult.leftPlaneLeftP = plainVector[i].leftP;
            scanResult.leftPlaneRightP = plainVector[i].rightP;
            scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
            scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
            scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z;
            scanResult.leftPlaneRightP.z = plainVector[i].linePos.z;
            scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z;
            scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z;
            scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
            scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
            scanResult.iOrder = i;
            scanResult.algoFlag = 2;
            scanAlgCmdDet->algoCounts[2]++;
            scanAlgCmdDet->scanResultVector.push_back(scanResult);
            scanAlgCmdDet->scanResCount++;
            if(scanAlgCmdDet->scanResultVector.size() >= 10)
            {
                info.errId = -68;
                sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                return FALSE;
            }
        }
        else if(sheetType == 2)
        {
            if(realGap < gapMax && realMismatch > heightMin && realMismatch < heightMax) //右在左上 符合搭接
            {
                rtn = true;
                scanResult.iGotResult = TRUE;
                scanResult.leftPlaneLeftP = plainVector[i].leftP;
                scanResult.leftPlaneRightP = plainVector[i].rightP;
                scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
                scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
                scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z;
                scanResult.leftPlaneRightP.z = plainVector[i].linePos.z;
                scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z;
                scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z;
                scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
                scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
                scanResult.iOrder = i;
                scanResult.algoFlag = 1;
                scanAlgCmdDet->algoCounts[2]++;
                scanAlgCmdDet->scanResultVector.push_back(scanResult);
                scanAlgCmdDet->scanResCount++;
                if(scanAlgCmdDet->scanResultVector.size() >= 10)
                {
                    info.errId = -69;
                    sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                    return FALSE;
                }
            }
            else if(realGap < gapMax && (-realMismatch) > heightMin && (-realMismatch) < heightMax) //左在右上 符合搭接
            {
                rtn = true;
                scanResult.iGotResult = TRUE;
                scanResult.leftPlaneLeftP = plainVector[i].leftP;
                scanResult.leftPlaneRightP = plainVector[i].rightP;
                scanResult.rightPlaneLeftP = plainVector[i + 1].leftP;
                scanResult.rightPlaneRightP = plainVector[i + 1].rightP;
                scanResult.leftPlaneLeftP.z = plainVector[i].linePos.z;
                scanResult.leftPlaneRightP.z = plainVector[i].linePos.z;
                scanResult.rightPlaneLeftP.z = plainVector[i + 1].linePos.z;
                scanResult.rightPlaneRightP.z = plainVector[i + 1].linePos.z;
                scanResult.midP = (scanResult.leftPlaneRightP + scanResult.rightPlaneLeftP) / 2;
                scanResult.midP.order = (scanResult.leftPlaneRightP.order + scanResult.rightPlaneLeftP.order) / 2;
                scanResult.iOrder = i;
                scanResult.algoFlag = 2;
                scanAlgCmdDet->algoCounts[2]++;
                scanAlgCmdDet->scanResultVector.push_back(scanResult);
                scanAlgCmdDet->scanResCount++;
                if(scanAlgCmdDet->scanResultVector.size() >= 10)
                {
                    info.errId = -70;
                    sprintf(info.errInfo, QObject::tr("符合条件的坡口太多").toStdString().c_str());
                    return FALSE;
                }
            }
        }
    }
    return rtn;
}

void LogRecord(QFile *fileData, QTextStream *out, QString str, bool replace)
{
    if(replace == true)
    {
        fileData->open(QFile::WriteOnly);
        out->setDevice(fileData);
        *out << str << "\r\n";
        fileData->flush();
        fileData->close();
    }
    else
    {
        fileData->open(QFile::WriteOnly | QFile::Append);
        out->setDevice(fileData);
        *out << str << "\r\n";
        fileData->flush();
        fileData->close();
    }
}

int getArea(SkPoint3D linePa, SkPoint3D linePb, SkPoint3D pc, double &area)
{
    SkLine line;
    getLine(linePa, linePb, line);
    double disAB = getDisFromPointToPoint(linePa, linePb);
    double disH = getDisFromPointToLine(pc, line);
    area = disAB *disH / 2.0;
    return TRUE;
}

int getPointByValue(std::vector<SkPoint3D> data, SkPoint3D &targetP, int type)
{
    int rtn = FALSE;
    double temp = 0;
    int targetIndex = 0;
    int findFlag = FALSE;
    if(data.empty())
    {
        return FALSE;
    }
    if(type == 0) //最大值
    {
        temp = data[0].z;
        for(int i = 0; i < data.size(); i++)
        {
            if(temp < data[i].z)
            {
                temp = data[i].z;
                targetIndex = i;
                findFlag = TRUE;
            }
        }
    }
    else  //最小值
    {
        temp = data[0].z;
        for(int i = 0; i < data.size(); i++)
        {
            if(temp > data[i].z)
            {
                temp = data[i].z;
                targetIndex = i;
                findFlag = TRUE;
            }
        }
    }
    if(findFlag == FALSE)
    {
        return FALSE;
    }
    targetP = data[targetIndex];
    return TRUE;
}

SkContourInfo getExportByBase(SkContourInfo rawData, SkBase basePara)
{
    SkContourInfo exportData = rawData;
    if(basePara.frame == 0)
    {
        exportData.mp = rawData.mp - basePara.base;
        exportData.op = rawData.op - basePara.base;
    }
    else
    {
        double rotX = -atan2(rawData.mp.b, rawData.mp.c);
        SkVector6d getBase = rotx(rotX) * basePara.base;
        exportData.mp = rawData.mp - getBase;
        exportData.op = rawData.op - getBase;
    }
    return exportData;
}

SkScanResult getWeldLineNewHLSmallDif2(SkPoint3D *origin, int length, double dLineHeightDif, double dPlainMaxDifL, int dPlainLenL, double sepMaxDis, SkScanAlgCmdDet *scanAlgCmdDet, int sectionYPointNum, double sectionYThre, double angleBottom, SkContourInfo &info)
{
    double dMinHeightL = dLineHeightDif / 3;
    double dMaxHeightL = dLineHeightDif * 20;
    //double dPlainMaxDif=0.8;
    //int dPlainLen=20;  //平台点数
    int  dPlainLen3 = dPlainLenL / 3 * 4;
    SkPlaneProperty planeProperty;
    int iMinPlainPos = 0;
    double dMinHeight = 0;
    int iMaxPlainPos = 0;
    double dMaxHeight = 0;
    SkPoint3D minPlain3D;
    SkPoint3D maxPlain3D;
    bool bLowestPlainFound = false;
    bool bFoundMaxPlain = false;
    SkScanResult scanResult;
    int iPlainCount = 0;
    double  dPlainHeightArray[300];//保存平台的高度，最多不超过300个
    double  dPlainPosArray[300];//保存平台的位置，最多不超过300个
    int  dPlainOrderArray[300];//保存平台的order,最多不超过300个
    double dTempHeightDif = 0;
    double dTempPosDif = 0;

    SkPlaneProperty planeYProperty;
    std::vector<SkPlaneProperty> planeYPropertyVector;

    SkPoint3D lowestP, lowestP2, highestP, highestP2;

    //step1:找到最低的一段平台
    dMinHeight = 20000;
    for(int i = 2; i < length * 3 / 4; i++)
    {
        planeProperty = getPlainZProperty(origin + i, dPlainLenL, dPlainMaxDifL);
        if(planeProperty.iPlainExist)
        {
            if(dMinHeight > planeProperty.linePos.z &&
                    planeProperty.linePos.z > MIN_LASER_VAL &&
                    planeProperty.linePos.z < MAX_LASER_VAL)
            {
                dMinHeight = planeProperty.linePos.z;
                iMinPlainPos = i;
                bLowestPlainFound = true;
                lowestP.y = planeProperty.linePos.y;
                lowestP.z = planeProperty.linePos.z;
            }
        }
    }
    if(!bLowestPlainFound)
    {
        return scanResult;
    }
    //step2:找到最高的一段平台:
    dMaxHeight = -20000;
    for(int i = iMinPlainPos; i < length - dPlainLen3; i++)
    {
        planeProperty = getPlainZProperty(origin + i, dPlainLenL, dPlainMaxDifL);
        if(planeProperty.iPlainExist)
        {
            if(dMaxHeight < planeProperty.linePos.z &&
                    planeProperty.linePos.z < dMinHeight + dMaxHeightL &&
                    planeProperty.linePos.z > dMinHeight + dMinHeightL)
            {
                dMaxHeight = planeProperty.linePos.z;
                iMaxPlainPos = i;
                bFoundMaxPlain = true;

                highestP.y = planeProperty.linePos.y;
                highestP.z = planeProperty.linePos.z;
            }
        }
    }
    if(!bFoundMaxPlain)
    {
        return scanResult;
    }
    //step3:在最高平台与最低平台之间，寻找最靠近焊缝的低平台
    for(int i = iMaxPlainPos; i >= iMinPlainPos; i--)
    {
        planeProperty = getPlainZProperty(origin + i, dPlainLenL, dPlainMaxDifL);
        if(planeProperty.iPlainExist)
        {
            if(planeProperty.linePos.z < dMinHeight + dPlainMaxDifL && planeProperty.linePos.z >= dMinHeight)
            {
                iMinPlainPos = i;
                lowestP2.y = planeProperty.linePos.y;
                lowestP2.z = planeProperty.linePos.z;
                break;
            }
        }
    }

    //step4:在最高平台与最低平台之间，寻找最靠近焊缝的高平台
    for(int i = iMinPlainPos; i <= iMaxPlainPos; i ++)
    {
        planeProperty = getPlainZProperty(origin + i, dPlainLenL, dPlainMaxDifL);
        if(planeProperty.iPlainExist)//2mm长，1mm波动的平台
        {
            if(planeProperty.linePos.z + dMinHeightL >= dMaxHeight && planeProperty.linePos.z <= dMaxHeight )
            {
                iMaxPlainPos = i;
                dMaxHeight = planeProperty.linePos.z;

                highestP2.y = planeProperty.linePos.y;
                highestP2.z = planeProperty.linePos.z;
                break;
            }
        }
    }

    //step5:为了保证能找到多个平台差，把两个位置后退3个点
    if(iMinPlainPos > 3)
    {
        iMinPlainPos -= 3;
    }
    if(iMaxPlainPos < length - dPlainLen3 - 3)
    {
        iMaxPlainPos += 3;
    }
    //step6:在最高点和最低点内，找到所有的平台，并记录下高度差和位置差
    for(int i = iMinPlainPos; i <= iMaxPlainPos; i ++)
    {
        planeProperty = getPlainZProperty(origin + i, dPlainLenL, dPlainMaxDifL);
        if(planeProperty.iPlainExist)//2mm长，1mm波动的平台
        {
            if(planeProperty.linePos.z < MAX_LASER_VAL && planeProperty.linePos.z > MIN_LASER_VAL )
            {
                dPlainHeightArray[iPlainCount] = planeProperty.linePos.z;
                dPlainPosArray[iPlainCount] = origin[i + dPlainLenL - 1].y;
                dPlainOrderArray[iPlainCount] = i;
                iPlainCount ++;
                if(iPlainCount >= 300)
                {
                    break;
                }
            }
        }
    }
    //step7:查找符合阶梯差的跳变平台
    for(int i = 1; i < iPlainCount; i++)
    {
        dTempHeightDif = dPlainHeightArray[i] - dPlainHeightArray[i - 1];//相邻台阶的高度差
        dTempPosDif = dPlainPosArray[i] - dPlainPosArray[i - 1];//相邻台阶的位置差

        if(dTempHeightDif > dMinHeightL &&
                dTempHeightDif < dMaxHeightL &&
                dTempPosDif < sepMaxDis)
        {
            scanResult.iGotResult = TRUE;
            scanResult.linePos.y = dPlainPosArray[i - 1];
            scanResult.linePos.z = dPlainHeightArray[i - 1];
            scanResult.linePos.x = dPlainOrderArray[i - 1];
            scanResult.linePos.order = dPlainOrderArray[i - 1];
            scanResult.dLowY = dPlainPosArray[i - 1];
            scanResult.dHighY = dPlainPosArray[i];
            scanResult.dLowHeight = dPlainHeightArray[i - 1];
            scanResult.dHighHeight = dPlainHeightArray[i];
            scanAlgCmdDet->scanResultVector.push_back(scanResult);
            scanAlgCmdDet->scanResCount++;
            if(scanAlgCmdDet->scanResultVector.size() >= 10)
            {
                break;
            }
        }
    }

    if(scanAlgCmdDet->scanResultVector.size() >= 1)
    {
        scanAlgCmdDet->scanResult = scanAlgCmdDet->scanResultVector.front();

        int orderP = scanAlgCmdDet->scanResult.linePos.x;

        for(int i = orderP - 40; i > 0; i--)
        {
            planeProperty = getPlainZProperty(origin + i, dPlainLenL, dPlainMaxDifL);
            if(planeProperty.iPlainExist)//2mm长，1mm波动的平台
            {
                scanAlgCmdDet->scanResult.linePos.z = planeProperty.linePos.z;
                break;
            }
        }

        scanAlgCmdDet->scanResCount = 1;

        for(int i = 50; i < length - 50; i++)
        {
            planeYProperty = getPlainYProperty(&origin[i], sectionYPointNum * 2 / 3, sectionYThre);
            if(planeYProperty.iPlainExist)
            {
                planeYPropertyVector.push_back(planeYProperty);
            }
        }

        if(planeYPropertyVector.size() != 0)
        {
            SkPoint3D lineLowPos, lineHighPos, planePos;
            lineLowPos = scanAlgCmdDet->scanResult.linePos;
            planePos.y = planeYPropertyVector.front().linePos.y;
            planePos.z = lineLowPos.z;
            lineHighPos.y = scanAlgCmdDet->scanResult.dHighY;
            lineHighPos.z = scanAlgCmdDet->scanResult.dHighHeight;
            double dis = planeYPropertyVector.front().linePos.y - lineLowPos.y;

            if(dis > 0 && dis < 3)
            {

                scanAlgCmdDet->scanResult.linePos.y = planeYPropertyVector.front().linePos.y;

                getDataByRotate(lowestP, lowestP, angleBottom);
                getDataByRotate(lowestP2, lowestP2, angleBottom);
                getDataByRotate(highestP2, highestP2, angleBottom);
                getDataByRotate(highestP, highestP, angleBottom);
                getDataByRotate(lineLowPos, lineLowPos, angleBottom);
                getDataByRotate(planePos, planePos, angleBottom);
                getDataByRotate(lineHighPos, lineHighPos, angleBottom);
            }
            else
            {
                getDataByRotate(lowestP, lowestP, angleBottom);
                getDataByRotate(lowestP2, lowestP2, angleBottom);
                getDataByRotate(highestP2, highestP2, angleBottom);
                getDataByRotate(highestP, highestP, angleBottom);
            }
        }
        else
        {
            getDataByRotate(lowestP, lowestP, angleBottom);
            getDataByRotate(lowestP2, lowestP2, angleBottom);
            getDataByRotate(highestP2, highestP2, angleBottom);
            getDataByRotate(highestP, highestP, angleBottom);
        }
    }
    return scanAlgCmdDet->scanResult;
}
