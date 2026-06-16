#include "OPini.h"

#include "ConfigDatabase.h"

#include <cstring>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

namespace
{
unsigned int ReadPrivateProfileStringUtf8(
    const std::string& sectionName,
    const std::string& key,
    char* value,
    unsigned int valueSize,
    const std::string& fileName)
{
    if (value == nullptr || valueSize == 0)
    {
        return 0;
    }
    value[0] = '\0';

    std::string storedValue;
    if (!ConfigDatabase::ReadIniValue(fileName, sectionName, key, &storedValue))
    {
        return 0;
    }

    strncpy_s(value, valueSize, storedValue.c_str(), _TRUNCATE);
    return static_cast<unsigned int>(strlen(value));
}

bool WritePrivateProfileStringUtf8(
    const std::string& sectionName,
    const std::string& key,
    const std::string& value,
    const std::string& fileName)
{
    return ConfigDatabase::WriteIniValue(fileName, sectionName, key, value);
}

bool FileExistsUtf8OrAcp(const std::string& fileName)
{
    return ConfigDatabase::HasIniFile(fileName);
}
}

COPini::COPini()
    : m_pIniLog("Log/RobotRunLog.txt", true)
{
}

COPini::~COPini()
{
}

// 核心：DWORD(unsigned int)转int，带溢出防护和日志报错
int COPini::DWORDToInt(unsigned int dwValue, const std::string& context)
{
    // 溢出判断：DWORD值超过int最大值（INT_MAX=2147483647）
    if (dwValue > static_cast<unsigned int>(INT_MAX))
    {
        // 构造详细错误日志
        std::ostringstream oss;
        oss << "[OVERFLOW ERROR] "
            << "Context: " << context << ", "
            << "DWORD Value: " << dwValue << ", "
            << "INT_MAX: " << INT_MAX << ", "
            << "File: " << m_fileName << ", "
            << "Section: " << m_sectionName;
        // 写入错误日志
        m_pIniLog.write(LogColor::ERR, "%s", oss.str().c_str());
        // 返回-1表示溢出（也可返回INT_MAX，根据业务需求调整）
        return -1;
    }
    // 无溢出，安全转换
    return static_cast<int>(dwValue);
}

bool COPini::CheckExists(std::string fileName, std::string sectionName, std::string key)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(sectionName, key, ch, sizeof(ch), fileName);
    return (dwNum != 0);
}

bool COPini::CheckExists(std::string key)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    return (dwNum != 0);
}

// ReadString返回值改为int，调用转换函数处理溢出
int COPini::ReadString(std::string fileName, std::string sectionName, std::string key, char value[])
{
    unsigned int dwResult = ReadPrivateProfileStringUtf8(sectionName, key, value, 255, fileName);
    // 转换并检测溢出，上下文描述便于定位问题
    return DWORDToInt(dwResult, "ReadString(fileName, sectionName, key, value[])");
}

int COPini::ReadString(std::string key, bool* value, bool bCheck)
{
    char ch[255] = { 0 };
    int nValue;
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    // 转换并检测溢出
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, bool*)");

    if (nReturnValue > 0)  // 仅在无溢出且有返回值时处理
    {
        nValue = atoi(ch);
        *value = (nValue == 1);
    }
    CheckRead(key, nReturnValue, bCheck);
    return nReturnValue;
}

int COPini::ReadString(std::string key, unsigned long* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, unsigned long*)");

    if (nReturnValue > 0)
    {
        *value = atol(ch);
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

int COPini::ReadString(std::string key, long long* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, long long*)");

    if (nReturnValue > 0)
    {
        *value = atol(ch);
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

int COPini::ReadString(std::string key1, std::string key2, T_ANGLE_PULSE& tPulse, T_ANGLE_PULSE tIsRead)
{
    int bRtn = 1;  // 改为int
    if (tIsRead.nSPulse > 0)
    {
        int nRet = ReadString(key1 + "S" + key2, &tPulse.nSPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;  // -1表示溢出
    }
    if (tIsRead.nLPulse > 0)
    {
        int nRet = ReadString(key1 + "L" + key2, &tPulse.nLPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.nUPulse > 0)
    {
        int nRet = ReadString(key1 + "U" + key2, &tPulse.nUPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.nRPulse > 0)
    {
        int nRet = ReadString(key1 + "R" + key2, &tPulse.nRPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.nBPulse > 0)
    {
        int nRet = ReadString(key1 + "B" + key2, &tPulse.nBPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.nTPulse > 0)
    {
        int nRet = ReadString(key1 + "T" + key2, &tPulse.nTPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.lBXPulse > 0)
    {
        int nRet = ReadString(key1 + "BX" + key2, &tPulse.lBXPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.lBYPulse > 0)
    {
        int nRet = ReadString(key1 + "BY" + key2, &tPulse.lBYPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.lBZPulse > 0)
    {
        int nRet = ReadString(key1 + "BZ" + key2, &tPulse.lBZPulse);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }

    return bRtn;
}

int COPini::ReadString(std::string key1, std::string key2, T_ROBOT_COORS& tCoord, T_ROBOT_COORS tIsRead)
{
    int bRtn = 1;  // 改为int
    if (tIsRead.dX > 0)
    {
        int nRet = ReadString(key1 + "X" + key2, &tCoord.dX);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dY > 0)
    {
        int nRet = ReadString(key1 + "Y" + key2, &tCoord.dY);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dZ > 0)
    {
        int nRet = ReadString(key1 + "Z" + key2, &tCoord.dZ);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dRX > 0)
    {
        int nRet = ReadString(key1 + "RX" + key2, &tCoord.dRX);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dRY > 0)
    {
        int nRet = ReadString(key1 + "RY" + key2, &tCoord.dRY);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dRZ > 0)
    {
        int nRet = ReadString(key1 + "RZ" + key2, &tCoord.dRZ);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dBX > 0)
    {
        int nRet = ReadString(key1 + "BX" + key2, &tCoord.dBX);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dBY > 0)
    {
        int nRet = ReadString(key1 + "BY" + key2, &tCoord.dBY);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }
    if (tIsRead.dBZ > 0)
    {
        int nRet = ReadString(key1 + "BZ" + key2, &tCoord.dBZ);
        bRtn = (bRtn && (nRet != 0 && nRet != -1)) ? 1 : 0;
    }

    return bRtn;
}

int COPini::ReadString(bool bCheck, std::string key, long* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(bCheck, key, long*)");

    if (nReturnValue > 0)
    {
        *value = atol(ch);
    }
    CheckRead(key, nReturnValue, bCheck);

    return nReturnValue;
}

void COPini::CheckRead(std::string key, int nReturnValue, bool bCheck)
{
    if (!bCheck)
    {
        return;
    }
    // 0=读取失败，-1=溢出
    if (nReturnValue == 0 || nReturnValue == -1)
    {
        std::string str = "Error: config database path " + m_fileName
            + " section " + m_sectionName
            + " key " + key
            + " read failed! Return value: " + std::to_string(nReturnValue);
        m_pIniLog.write(LogColor::ERR, "%s", str.c_str());
    }
}

void COPini::CheckFileEncodeType(std::string fileName)
{
    (void)fileName;
    // 配置已统一放入 ConfigStore.db，主程序不再探测或改写旧 ini 文件编码。
}

bool COPini::WriteString(std::string fileName, std::string sectionName, std::string key, std::string value)
{
    return WritePrivateProfileStringUtf8(sectionName, key, value, fileName);
}

bool COPini::WriteString(std::string key, bool value)
{
    std::string str;
    int nValue = value ? 1 : 0;
    char buf[32];
    sprintf_s(buf, sizeof(buf), "%ld", nValue);
    str = buf;
    return WritePrivateProfileStringUtf8(m_sectionName, key, str, m_fileName);
}

bool COPini::WriteString(std::string key, long value)
{
    std::string str;
    char buf[32];
    sprintf_s(buf, sizeof(buf), "%ld", value);
    str = buf;
    return WritePrivateProfileStringUtf8(m_sectionName, key, str, m_fileName);
}

bool COPini::WriteString(std::string key1, std::string key2, T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tIsRead)
{
    bool bRtn = true;
    if (tIsRead.nSPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "S" + key2, tPulse.nSPulse);
    }
    if (tIsRead.nLPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "L" + key2, tPulse.nLPulse);
    }
    if (tIsRead.nUPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "U" + key2, tPulse.nUPulse);
    }
    if (tIsRead.nRPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "R" + key2, tPulse.nRPulse);
    }
    if (tIsRead.nBPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "B" + key2, tPulse.nBPulse);
    }
    if (tIsRead.nTPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "T" + key2, tPulse.nTPulse);
    }
    if (tIsRead.lBXPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "BX" + key2, tPulse.lBXPulse);
    }
    if (tIsRead.lBYPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "BY" + key2, tPulse.lBYPulse);
    }
    if (tIsRead.lBZPulse > 0)
    {
        bRtn = bRtn && WriteString(key1 + "BZ" + key2, tPulse.lBZPulse);
    }
    return bRtn;
}

bool COPini::WriteString(std::string key1, std::string key2, T_ROBOT_COORS tCoord, T_ROBOT_COORS tIsRead)
{
    bool bRtn = true;
    if (tIsRead.dX > 0)
    {
        bRtn = bRtn && WriteString(key1 + "X" + key2, tCoord.dX);
    }
    if (tIsRead.dY > 0)
    {
        bRtn = bRtn && WriteString(key1 + "Y" + key2, tCoord.dY);
    }
    if (tIsRead.dZ > 0)
    {
        bRtn = bRtn && WriteString(key1 + "Z" + key2, tCoord.dZ);
    }
    if (tIsRead.dRX > 0)
    {
        bRtn = bRtn && WriteString(key1 + "RX" + key2, tCoord.dRX);
    }
    if (tIsRead.dRY > 0)
    {
        bRtn = bRtn && WriteString(key1 + "RY" + key2, tCoord.dRY);
    }
    if (tIsRead.dRZ > 0)
    {
        bRtn = bRtn && WriteString(key1 + "RZ" + key2, tCoord.dRZ);
    }
    if (tIsRead.dBX > 0)
    {
        bRtn = bRtn && WriteString(key1 + "BX" + key2, tCoord.dBX);
    }
    if (tIsRead.dBY > 0)
    {
        bRtn = bRtn && WriteString(key1 + "BY" + key2, tCoord.dBY);
    }
    if (tIsRead.dBZ > 0)
    {
        bRtn = bRtn && WriteString(key1 + "BZ" + key2, tCoord.dBZ);
    }
    return bRtn;
}

bool COPini::SetFileName(std::string fileName)
{
    m_fileName = fileName;
    if (!CheckFileExists(fileName))
    {
        std::string str = "Error: config database path not found: " + fileName;
        m_pIniLog.write(LogColor::WARNING, "%s", str.c_str());
    }
    CheckFileEncodeType(m_fileName);
    return true;
}

bool COPini::SetFileName(bool bCheck, std::string fileName)
{
    m_fileName = fileName;
    if (!bCheck)
    {
        return true;
    }
    if (!CheckFileExists(fileName))
    {
        std::string str = "Error: config database path not found: " + fileName;
        m_pIniLog.write(LogColor::WARNING, "%s", str.c_str());
    }
    CheckFileEncodeType(m_fileName);
    return true;
}

bool COPini::SetSectionName(std::string sectionName)
{
    m_sectionName = sectionName;
    return true;
}

int COPini::ReadString(std::string key, char value[])
{
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, value, 255, m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, char[])");
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

bool COPini::WriteString(std::string key, std::string value)
{
    return WritePrivateProfileStringUtf8(m_sectionName, key, value, m_fileName);
}

int COPini::ReadString(std::string key, double* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, double*)");

    if (nReturnValue > 0)
    {
        *value = atof(ch);
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

int COPini::ReadString(std::string key, float* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, float*)");

    if (nReturnValue > 0)
    {
        *value = static_cast<float>(atof(ch));
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

bool COPini::WriteString(std::string key, double value, unsigned int unDecimalDigits)
{
    std::string str;
    char format[16];
    sprintf_s(format, sizeof(format), "%%.%uLf", unDecimalDigits);
    char buf[256];
    sprintf_s(buf, sizeof(buf), format, value);
    str = buf;
    return WritePrivateProfileStringUtf8(m_sectionName, key, str, m_fileName);
}

int COPini::ReadString(std::string key, int* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, int*)");

    if (nReturnValue > 0)
    {
        *value = atoi(ch);
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

int COPini::ReadString(std::string key, long* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, long*)");

    if (nReturnValue > 0)
    {
        *value = atol(ch);
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

int COPini::ReadString(std::string key, std::string& value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, std::string&)");

    if (nReturnValue > 0)
    {
        value = ch;
    }
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

bool COPini::WriteString(std::string key, int value)
{
    std::string str;
    char buf[32];
    sprintf_s(buf, sizeof(buf), "%ld", value);
    str = buf;
    return WritePrivateProfileStringUtf8(m_sectionName, key, str, m_fileName);
}

int COPini::ReadString(bool bCheck, std::string key, bool* value)
{
    char ch[255] = { 0 };
    int nValue;
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(bCheck, key, bool*)");

    if (nReturnValue > 0)
    {
        nValue = atoi(ch);
        *value = (nValue == 1);
    }
    CheckRead(key, nReturnValue, bCheck);
    return nReturnValue;
}

int COPini::ReadString(bool bCheck, std::string key, std::string& value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(bCheck, key, std::string&)");

    if (nReturnValue > 0)
    {
        value = ch;
    }
    CheckRead(key, nReturnValue, bCheck);
    return nReturnValue;
}

int COPini::ReadString(bool bCheck, std::string key, std::string* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(bCheck, key, std::string*)");

    if (nReturnValue > 0)
    {
        (*value) = ch;
    }
    CheckRead(key, nReturnValue, bCheck);
    return nReturnValue;
}

int COPini::ReadString(bool bCheck, std::string key, double* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(bCheck, key, double*)");

    if (nReturnValue > 0)
    {
        *value = atof(ch);
    }
    CheckRead(key, nReturnValue, bCheck);
    return nReturnValue;
}

int COPini::ReadString(bool bCheck, std::string key, int* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = ReadPrivateProfileStringUtf8(m_sectionName, key, ch, sizeof(ch), m_fileName);
    int nReturnValue = DWORDToInt(dwNum, "ReadString(bCheck, key, int*)");

    if (nReturnValue > 0)
    {
        *value = atoi(ch);
    }
    CheckRead(key, nReturnValue, bCheck);
    return nReturnValue;
}

int COPini::ReadAddString(std::string key, bool* value, bool init_value)
{
    int nRet = ReadString(false, key, value);
    if (nRet == 0 || nRet == -1)
    {
        *value = init_value;
        WriteString(key, init_value);
        return 0;
    }
    return 1;
}

int COPini::ReadAddString(std::string key, int* value, int init_value)
{
    int nRet = ReadString(false, key, value);
    if (nRet == 0 || nRet == -1)
    {
        *value = init_value;
        WriteString(key, init_value);
        return 0;
    }
    return 1;
}

int COPini::ReadAddString(std::string key, int* value, long init_value)
{
    int nRet = ReadString(false, key, value);
    if (nRet == 0 || nRet == -1)
    {
        *value = static_cast<int>(init_value);
        WriteString(key, static_cast<int>(init_value));
        return 0;
    }
    return 1;
}

int COPini::ReadAddString(std::string key, std::string& value, std::string init_value)
{
    int nRet = ReadString(false, key, value);
    if (nRet == 0 || nRet == -1)
    {
        value = init_value;
        WriteString(key, init_value);
        return 0;
    }
    return 1;
}

int COPini::ReadAddString(std::string key, double* value, double init_value)
{
    int nRet = ReadString(false, key, value);
    if (nRet == 0 || nRet == -1)
    {
        *value = init_value;
        WriteString(key, init_value);
        return 0;
    }
    return 1;
}

bool COPini::CheckFileExists(const std::string& fileName)
{
    return FileExistsUtf8OrAcp(fileName);
}
