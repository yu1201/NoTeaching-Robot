#include "InovanceWeldCalibration.h"

#include <QCoreApplication>
#include <QStringList>
#include <cmath>
#include <iostream>

namespace
{
QString Xml(QStringList values)
{
    return "<Struct name=\"WELD_LaserConfig\"><Variable><name>E00</name>"
        "<type>S_LASERCONFIG</type><value>" + values.join(',')
        + "</value></Variable></Struct><Struct name=\"Other\"/>";
}

QStringList ValidValues()
{
    QStringList f;
    for (int i = 0; i < 135; ++i) { f << "0"; }
    f[0] = "AIH";
    f[2] = "0";
    f[3] = "1";
    f[4] = "934.858";
    f[5] = "-90.049";
    f[6] = "550.391";
    const double xyzabc[] = {97.3294, 67.8006, -134.2267, 76.7707, -1.8085, 146.2318};
    for (int i = 0; i < 6; ++i) { f[115 + i] = QString::number(xyzabc[i], 'f', 4); }
    const double errors[] = {.0733, .1065, .1342, 0, 0, 0};
    for (int i = 0; i < 6; ++i) { f[121 + i] = QString::number(errors[i], 'f', 4); }
    const Eigen::Matrix4d cameraToTool = InovanceWeldCalibration::Pose(
        std::vector<double>(xyzabc, xyzabc + 6), 0);
    const Eigen::Vector3d reference(934.858, -90.049, 550.391);
    const Eigen::Vector3d points[] = {
        {-20, -30, 0}, {20, -30, 0}, {-20, 30, 0}, {20, 30, 0}, {0, 0, 20}
    };
    for (int i = 0; i < 5; ++i)
    {
        const Eigen::Vector3d toolPoint =
            cameraToTool.topLeftCorner<3, 3>() * points[i] + cameraToTool.topRightCorner<3, 1>();
        const Eigen::Vector3d robotTranslation = reference - toolPoint;
        const int poseAt = 20 + i * 16;
        for (int axis = 0; axis < 3; ++axis)
        {
            f[poseAt + axis] = QString::number(robotTranslation[axis], 'f', 9);
            f[100 + i * 3 + axis] = QString::number(points[i][axis], 'f', 9);
        }
    }
    f[133] = "192.168.39.5";
    f[134] = "5020";
    return f;
}

bool Near(double a, double b) { return std::abs(a - b) < 1e-6; }
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    RobotControllerHandEye result;
    std::string error;
    QStringList valid = ValidValues();
    if (!InovanceWeldCalibration::Parse(Xml(valid).toUtf8(), 0, result, error))
    {
        std::cerr << error << '\n';
        return 1;
    }
    if (result.toolIndex != 1 || result.cameraAddress != "192.168.39.5"
        || !Near(result.cameraToTool(0, 3), 97.3294)
        || !Near(result.cameraToTool(1, 3), 67.8006)
        || !Near(result.cameraToTool(2, 3), -134.2267)
        || result.sampleResidualMaxMm > 1e-5
        || result.evidence.find("97.3294,67.8006,-134.2267,76.7707,-1.8085,146.2318") == std::string::npos)
    {
        std::cerr << "parsed calibration mismatch\n";
        return 2;
    }
    QStringList factory = valid;
    for (int i = 20; i < 133; ++i) { factory[i] = "0"; }
    if (InovanceWeldCalibration::Parse(Xml(factory).toUtf8(), 0, result, error)) { return 3; }
    QStringList compensated = valid;
    compensated[127] = "0.1";
    if (InovanceWeldCalibration::Parse(Xml(compensated).toUtf8(), 0, result, error)) { return 4; }
    QStringList wrongCount = valid;
    wrongCount.removeLast();
    if (InovanceWeldCalibration::Parse(Xml(wrongCount).toUtf8(), 0, result, error)) { return 5; }
    std::cout << "PASS: Inovance live weld calibration parser\n";
    return 0;
}
