#include "Const.h"

// --------------------- T_ROBOT_COORS 函数实现 ---------------------
T_ROBOT_COORS::T_ROBOT_COORS()
    : dX(0.0), dY(0.0), dZ(0.0), dRX(0.0), dRY(0.0), dRZ(0.0), dBX(0.0), dBY(0.0), dBZ(0.0) {
}

T_ROBOT_COORS::T_ROBOT_COORS(double dX, double dY, double dZ, double dRX, double dRY, double dRZ, double dBX, double dBY, double dBZ)
    : dX(dX), dY(dY), dZ(dZ), dRX(dRX), dRY(dRY), dRZ(dRZ), dBX(dBX), dBY(dBY), dBZ(dBZ) {
}

T_ROBOT_COORS T_ROBOT_COORS::operator+(const T_ROBOT_COORS& tPoint) {
    return T_ROBOT_COORS(
        dX + tPoint.dX, dY + tPoint.dY, dZ + tPoint.dZ,
        dRX + tPoint.dRX, dRY + tPoint.dRY, dRZ + tPoint.dRZ,
        dBX + tPoint.dBX, dBY + tPoint.dBY, dBZ + tPoint.dBZ
    );
}

T_ROBOT_COORS T_ROBOT_COORS::operator-(const T_ROBOT_COORS& tPoint) {
    return T_ROBOT_COORS(
        dX - tPoint.dX, dY - tPoint.dY, dZ - tPoint.dZ,
        dRX - tPoint.dRX, dRY - tPoint.dRY, dRZ - tPoint.dRZ,
        dBX - tPoint.dBX, dBY - tPoint.dBY, dBZ - tPoint.dBZ
    );
}

T_ROBOT_COORS T_ROBOT_COORS::operator*(double dNum) {
    return T_ROBOT_COORS(
        dX * dNum, dY * dNum, dZ * dNum,
        dRX * dNum, dRY * dNum, dRZ * dNum,
        dBX * dNum, dBY * dNum, dBZ * dNum
    );
}

void T_ROBOT_COORS::TransToWorld() {
    dX += dBX;
    dY += dBY;
    dZ += dBZ;
    dBX = 0.0;
    dBY = 0.0;
    dBZ = 0.0;
}

int T_ROBOT_COORS::fprintf(FILE* file, const char* sFormat) {
    if (!file) return -1;
    return ::fprintf(file, sFormat, dX, dY, dZ, dRX, dRY, dRZ, dBX, dBY, dBZ);
}

int T_ROBOT_COORS::fprintf(FILE* file, int nIndex, const char* sFormat) {
    if (!file) return -1;
    return ::fprintf(file, sFormat, nIndex, dX, dY, dZ, dRX, dRY, dRZ, dBX, dBY, dBZ);
}

bool T_ROBOT_COORS::Compare(const T_ROBOT_COORS& tCompare, const T_ROBOT_COORS& tLimit) {
    return (fabs(dX - tCompare.dX) <= tLimit.dX) &&
        (fabs(dY - tCompare.dY) <= tLimit.dY) &&
        (fabs(dZ - tCompare.dZ) <= tLimit.dZ) &&
        (fabs(dRX - tCompare.dRX) <= tLimit.dRX) &&
        (fabs(dRY - tCompare.dRY) <= tLimit.dRY) &&
        (fabs(dRZ - tCompare.dRZ) <= tLimit.dRZ) &&
        (fabs(dBX - tCompare.dBX) <= tLimit.dBX) &&
        (fabs(dBY - tCompare.dBY) <= tLimit.dBY) &&
        (fabs(dBZ - tCompare.dBZ) <= tLimit.dBZ);
}

// --------------------- T_ANGLE_PULSE 函数实现 ---------------------
T_ANGLE_PULSE::T_ANGLE_PULSE()
    : nSPulse(0), nLPulse(0), nUPulse(0), nRPulse(0), nBPulse(0), nTPulse(0), lBXPulse(0), lBYPulse(0), lBZPulse(0) {
}

T_ANGLE_PULSE::T_ANGLE_PULSE(long nSPulse, long nLPulse, long nUPulse, long nRPulse, long nBPulse, long nTPulse, long lBXPulse, long lBYPulse, long lBZPulse)
    : nSPulse(nSPulse), nLPulse(nLPulse), nUPulse(nUPulse), nRPulse(nRPulse), nBPulse(nBPulse), nTPulse(nTPulse),
    lBXPulse(lBXPulse), lBYPulse(lBYPulse), lBZPulse(lBZPulse) {
}

int T_ANGLE_PULSE::fprintf(FILE* file, const char* sFormat) {
    if (!file) return -1;
    return ::fprintf(file, sFormat, nSPulse, nLPulse, nUPulse, nRPulse, nBPulse, nTPulse, lBXPulse, lBYPulse, lBZPulse);
}

int T_ANGLE_PULSE::fprintf(FILE* file, int nIndex, const char* sFormat) {
    if (!file) return -1;
    return ::fprintf(file, sFormat, nIndex, nSPulse, nLPulse, nUPulse, nRPulse, nBPulse, nTPulse, lBXPulse, lBYPulse, lBZPulse);
}

bool T_ANGLE_PULSE::Compare(const T_ANGLE_PULSE& tCompare, const T_ANGLE_PULSE& tLimit) {
    return (labs(nSPulse - tCompare.nSPulse) <= tLimit.nSPulse) &&
        (labs(nLPulse - tCompare.nLPulse) <= tLimit.nLPulse) &&
        (labs(nUPulse - tCompare.nUPulse) <= tLimit.nUPulse) &&
        (labs(nRPulse - tCompare.nRPulse) <= tLimit.nRPulse) &&
        (labs(nBPulse - tCompare.nBPulse) <= tLimit.nBPulse) &&
        (labs(nTPulse - tCompare.nTPulse) <= tLimit.nTPulse) &&
        (labs(lBXPulse - tCompare.lBXPulse) <= tLimit.lBXPulse) &&
        (labs(lBYPulse - tCompare.lBYPulse) <= tLimit.lBYPulse) &&
        (labs(lBZPulse - tCompare.lBZPulse) <= tLimit.lBZPulse);
}

std::string GetStr(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    char s[1000];
    vsprintf_s(s, 1000, format, args); // 安全格式化输出到字符数组
    va_end(args);

    std::string str(s); // 直接用字符数组初始化string（替代错误的Format）
    return str;
}
