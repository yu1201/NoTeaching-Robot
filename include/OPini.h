#pragma once
#include "Const.h"

#include "RobotLog.h"
#include <string>
#include <climits>  // 引入INT_MAX/INT_MIN定义
#include <fstream>
//#include <cstring>

#include <windows.h>
#include <sstream>  // 字符串拼接

#if !defined(AFX_OPINI_H__7449A144_FBE5_4D4B_BD76_520FC78545A4__INCLUDED_)
#define AFX_OPINI_H__7449A144_FBE5_4D4B_BD76_520FC78545A4__INCLUDED_
// 文本编码类型枚举
typedef enum tagTextCodeType
{
    TextUnkonw = -1,
    TextANSI = 0,
    TextUTF8 = 1,
    TextUNICODE = 2,
    TextUNICODE_BIG = 3
} TextCodeType;

class COPini
{
public:
    COPini();
    virtual ~COPini();

    // 核心：DWORD(unsigned int)转int，带溢出防护和日志报错
    int DWORDToInt(unsigned int dwValue, const std::string& context);

    // 原有接口：所有返回unsigned int的改为int，BOOL改为bool
    bool  CheckExists(std::string fileName, std::string sectionName, std::string key);
    bool  CheckExists(std::string key);

    bool  SetFileName(std::string fileName);
    bool  SetFileName(bool bCheck, std::string fileName);
    bool  SetSectionName(std::string sectionName);

    bool  WriteString(std::string key, std::string value);
    bool  WriteString(std::string key, double value, unsigned int unDecimalDigits = 6);
    bool  WriteString(std::string key, int value);
    bool  WriteString(std::string key, long value);
    bool  WriteString(std::string key, bool value);
    bool  WriteString(std::string key1, std::string key2, T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tIsRead = T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, 1, 1, 1));
    bool  WriteString(std::string key1, std::string key2, T_ROBOT_COORS tCoord, T_ROBOT_COORS tIsRead = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1));

    // 所有ReadString返回值改为int
    int ReadString(std::string key, char value[]);
    int ReadString(std::string key, double* value);
    int ReadString(std::string key, float* value);
    int ReadString(std::string key, int* value);
    int ReadString(std::string key, long* value);
    int ReadString(std::string key, long long* value);
    int ReadString(std::string key, unsigned long* value);
    int ReadString(std::string key, std::string& value);
    int ReadString(std::string key, bool* value, bool bCheck = true);
    int ReadString(std::string key1, std::string key2, T_ANGLE_PULSE& tPulse, T_ANGLE_PULSE tIsRead = T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, 1, 1, 1));
    int ReadString(std::string key1, std::string key2, T_ROBOT_COORS& tCoord, T_ROBOT_COORS tIsRead = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1));

    int ReadString(bool bCheck, std::string key, bool* value);
    int ReadString(bool bCheck, std::string key, int* value);
    int ReadString(bool bCheck, std::string key, long* value);
    int ReadString(bool bCheck, std::string key, double* value);
    int ReadString(bool bCheck, std::string key, std::string& value);
    int ReadString(bool bCheck, std::string key, std::string* value);

    int ReadAddString(std::string key, bool* value, bool init_value);
    int ReadAddString(std::string key, int* value, int init_value);
    int ReadAddString(std::string key, int* value, long init_value);
    int ReadAddString(std::string key, double* value, double init_value);
    int ReadAddString(std::string key, std::string& value, std::string init_value);

    std::string m_fileName;
    std::string m_sectionName;
    bool  WriteString(std::string fileName, std::string sectionName, std::string key, std::string value);
    int ReadString(std::string fileName, std::string sectionName, std::string key, char value[]);
    void CheckRead(std::string key, int nReturnValue, bool bCheck = true);  // 参数改为int
    void CheckFileEncodeType(std::string fileName);

    bool CheckFileExists(const std::string& fileName);
    bool CheckAndCreateDir(const std::string& dirPath);
    RobotLog m_pIniLog;
};

#endif // !defined(AFX_OPINI_H__7449A144_FBE5_4D4B_BD76_520FC78545A4__INCLUDED_)