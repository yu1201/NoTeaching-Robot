#include "OPini.h"


#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

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
        m_pIniLog.write(LogColor::ERR, oss.str().c_str());
        // 返回-1表示溢出（也可返回INT_MAX，根据业务需求调整）
        return -1;
    }
    // 无溢出，安全转换
    return static_cast<int>(dwValue);
}

bool COPini::CheckExists(std::string fileName, std::string sectionName, std::string key)
{
    char ch[255] = { 0 };
    unsigned int dwNum = GetPrivateProfileStringA(sectionName.c_str(), key.c_str(), NULL, ch, 255, fileName.c_str());
    return (dwNum != 0);
}

bool COPini::CheckExists(std::string key)
{
    char ch[255] = { 0 };
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
    return (dwNum != 0);
}

// ReadString返回值改为int，调用转换函数处理溢出
int COPini::ReadString(std::string fileName, std::string sectionName, std::string key, char value[])
{
    unsigned int dwResult = static_cast<unsigned int>(::GetPrivateProfileStringA(
        sectionName.c_str(), key.c_str(), NULL, value, 255, fileName.c_str()));
    // 转换并检测溢出，上下文描述便于定位问题
    return DWORDToInt(dwResult, "ReadString(fileName, sectionName, key, value[])");
}

int COPini::ReadString(std::string key, bool* value, bool bCheck)
{
    char ch[255] = { 0 };
    int nValue;
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
        std::string str = "Error: " + m_fileName + " file " + m_sectionName + " section " + key + " read failed! Return value: " + std::to_string(nReturnValue);
        m_pIniLog.write(LogColor::ERR, str.c_str());
    }
}

void COPini::CheckFileEncodeType(std::string fileName)
{
    unsigned char headBuf[3] = { 0 };
    TextCodeType type = TextUnkonw;

    FILE* file = nullptr;
    errno_t err = fopen_s(&file, fileName.c_str(), "rb+");
    if (err != 0 || file == NULL)
    {
        std::string str = "Error: " + fileName + " file open failed!";
        m_pIniLog.write(LogColor::ERR, str.c_str());
        return;
    }

    fseek(file, 0, SEEK_SET);
    fread(headBuf, 1, 3, file);

    if (headBuf[0] == 0xEF && headBuf[1] == 0xBB && headBuf[2] == 0xBF)
    {
        type = TextUTF8;
        fseek(file, 0L, SEEK_END);
        long len = ftell(file);
        unsigned char* Buf = new unsigned char[len];
        fseek(file, 0L, SEEK_SET);
        int nNo = 0;
        for (int i = 0; i < len; i++)
        {
            fread(&(Buf[i]), 1, 1, file);
            if (Buf[i] == '\n')
            {
                nNo++;
            }
        }
        fclose(file);

        // 原代码：file = fopen(fileName.c_str(), "wb+");
        FILE* newFile = nullptr;
        errno_t err = fopen_s(&newFile, fileName.c_str(), "wb+");
        if (err == 0 && newFile != NULL)
        {
            fwrite(&(Buf[3]), 1, len - nNo - 3, newFile);
            fclose(newFile);
        }

        delete[] Buf;
    }
    else if (headBuf[0] == 0xFF && headBuf[1] == 0xFE)
    {
        type = TextUNICODE;
        std::string str = "Error: " + fileName + " format error!";
        m_pIniLog.write(LogColor::ERR, str.c_str());
        fclose(file);
    }
    else if (headBuf[0] == 0xFE && headBuf[1] == 0xFF)
    {
        type = TextUNICODE_BIG;
        std::string str = "Error: " + fileName + " format error!";
        m_pIniLog.write(LogColor::ERR, str.c_str());
        fclose(file);
    }
    else
    {
        type = TextANSI;
        fclose(file);
    }
}

bool COPini::WriteString(std::string fileName, std::string sectionName, std::string key, std::string value)
{
    return ::WritePrivateProfileStringA(sectionName.c_str(), key.c_str(), value.c_str(), fileName.c_str()) != 0;
}

bool COPini::WriteString(std::string key, bool value)
{
    std::string str;
    int nValue = value ? 1 : 0;
    char buf[32];
    sprintf_s(buf, sizeof(buf), "%ld", nValue);
    str = buf;
    return ::WritePrivateProfileStringA(m_sectionName.c_str(), key.c_str(), str.c_str(), m_fileName.c_str()) != 0;
}

bool COPini::WriteString(std::string key, long value)
{
    std::string str;
    char buf[32];
    sprintf_s(buf, sizeof(buf), "%ld", value);
    str = buf;
    return ::WritePrivateProfileStringA(m_sectionName.c_str(), key.c_str(), str.c_str(), m_fileName.c_str()) != 0;
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
        std::string str = "Error: " + fileName + " file does not exist!";
        m_pIniLog.write(LogColor::WARNING, str.c_str());
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
        std::string str = "Error: " + fileName + " file does not exist!";
        m_pIniLog.write(LogColor::WARNING, str.c_str());
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
    unsigned int dwNum = static_cast<unsigned int>(::GetPrivateProfileStringA(
        m_sectionName.c_str(), key.c_str(), NULL, value, 255, m_fileName.c_str()));
    int nReturnValue = DWORDToInt(dwNum, "ReadString(key, char[])");
    CheckRead(key, nReturnValue);
    return nReturnValue;
}

bool COPini::WriteString(std::string key, std::string value)
{
    return ::WritePrivateProfileStringA(m_sectionName.c_str(), key.c_str(), value.c_str(), m_fileName.c_str()) != 0;
}

int COPini::ReadString(std::string key, double* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    return ::WritePrivateProfileStringA(m_sectionName.c_str(), key.c_str(), str.c_str(), m_fileName.c_str()) != 0;
}

int COPini::ReadString(std::string key, int* value)
{
    char ch[255] = { 0 };
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    return ::WritePrivateProfileStringA(m_sectionName.c_str(), key.c_str(), str.c_str(), m_fileName.c_str()) != 0;
}

int COPini::ReadString(bool bCheck, std::string key, bool* value)
{
    char ch[255] = { 0 };
    int nValue;
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    unsigned int dwNum = GetPrivateProfileStringA(m_sectionName.c_str(), key.c_str(), NULL, ch, 255, m_fileName.c_str());
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
    WIN32_FIND_DATAA findData;
    HANDLE hFind = FindFirstFileA(fileName.c_str(), &findData);
    bool exists = (hFind != INVALID_HANDLE_VALUE);
    if (exists) FindClose(hFind);
    return exists;
}

bool COPini::CheckAndCreateDir(const std::string& dirPath)
{
    // 1. 检查文件夹是否存在（Windows API：GetFileAttributes）
    DWORD attr = GetFileAttributesA(dirPath.c_str());
    if (attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY))
    {
        m_pIniLog.write(LogColor::SUCCESS, "文件夹已存在：%s", dirPath.c_str());
        return true;
    }

    // 2. 不存在则创建文件夹（Windows API：CreateDirectory）
    BOOL ret = CreateDirectoryA(dirPath.c_str(), NULL);
    if (ret)
    {
        m_pIniLog.write(LogColor::SUCCESS, "文件夹创建成功：%s", dirPath.c_str());
        return true;
    }
    else
    {
        DWORD err = GetLastError();
        m_pIniLog.write(LogColor::ERR, "文件夹创建失败：%s（错误码：%d）", dirPath.c_str(), err);
        return false;
    }
}
