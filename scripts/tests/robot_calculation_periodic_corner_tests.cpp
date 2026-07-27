#include "../../src/RobotCalculation.cpp"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
struct KeyRow
{
    int sourceIndex;
    double x;
    double y;
    double z;
};

QVector<GeometryProjectedPoint> BuildRobotCPeriodicKeys()
{
    // RobotC/20260723_001 的拟合关键点。最后一个完整下平台被错误拆成
    // 1409->1432 的坡和 1432->1440 的 5.13 mm 同类双角短段。
    const KeyRow rows[] = {
        { 21, -472.225000, 1185.200000, -219.535996 },
        { 49, -458.385857, 1185.024241, -219.506233 },
        { 73, -451.463701, 1171.475406, -219.257063 },
        { 267, -354.674313, 1171.287222, -219.067093 },
        { 292, -347.028870, 1184.589606, -219.284061 },
        { 501, -242.453704, 1181.239127, -219.023777 },
        { 527, -236.112668, 1166.718757, -218.758842 },
        { 722, -138.788099, 1164.383261, -218.530230 },
        { 749, -131.135086, 1178.536874, -218.762008 },
        { 955, -27.651312, 1179.188336, -218.573545 },
        { 980, -21.169795, 1165.359090, -218.320386 },
        { 1175, 76.370082, 1160.602288, -218.049436 },
        { 1197, 83.973767, 1173.919143, -218.266676 },
        { 1409, 188.300839, 1172.107516, -218.033788 },
        { 1432, 193.579223, 1159.766916, -217.808724 },
        { 1440, 198.284310, 1157.720886, -217.764294 },
        { 1547, 251.028181, 1157.460651, -217.657929 },
    };

    QVector<GeometryProjectedPoint> projected;
    projected.reserve(static_cast<int>(sizeof(rows) / sizeof(rows[0])));
    for (const KeyRow& row : rows)
    {
        GeometryProjectedPoint point;
        point.inputIndex = row.sourceIndex;
        point.point = Eigen::Vector3d(row.x, row.y, row.z);
        point.s = row.x;
        point.h = row.y;
        point.smoothH = row.y;
        point.n = row.z;
        point.smoothN = row.z;
        projected.push_back(point);
    }
    return projected;
}

QVector<GeometryProjectedPoint> BuildRobotC009PeriodicKeys()
{
    // RobotC/20260724_009 反向扫描现场关键点：首尾各有一个真实外拐点被拆成
    // OO 短段，分别为 5.319 mm 和 4.879 mm。排除候选后每类只有两个完整
    // 平台/同方向坡参考段，覆盖短工件的两参考段回归。
    const KeyRow rows[] = {
        { 21, -412.927672, 1170.920028, -218.363208 },
        { 149, -349.843657, 1170.891420, -218.251591 },
        { 157, -344.986638, 1173.060334, -218.287460 },
        { 179, -339.118701, 1184.164345, -218.504888 },
        { 391, -234.410387, 1180.863940, -218.252808 },
        { 415, -228.044600, 1166.305678, -217.942915 },
        { 609, -130.646847, 1163.944511, -217.723038 },
        { 635, -123.128378, 1178.052321, -217.999119 },
        { 841, -19.748357, 1178.919615, -217.834964 },
        { 867, -12.950016, 1164.911979, -217.535688 },
        { 1061, 84.348156, 1160.026419, -217.264245 },
        { 1082, 91.928683, 1173.372690, -217.524636 },
        { 1294, 195.981796, 1171.388210, -217.300704 },
        { 1318, 201.425638, 1159.043225, -217.037799 },
        { 1326, 205.887569, 1157.069873, -216.989447 },
        { 1454, 269.390085, 1156.628253, -216.868619 },
    };

    QVector<GeometryProjectedPoint> projected;
    projected.reserve(static_cast<int>(sizeof(rows) / sizeof(rows[0])));
    for (const KeyRow& row : rows)
    {
        GeometryProjectedPoint point;
        point.inputIndex = row.sourceIndex;
        point.point = Eigen::Vector3d(row.x, row.y, row.z);
        point.s = row.x;
        point.h = row.y;
        point.smoothH = row.y;
        point.n = row.z;
        point.smoothN = row.z;
        projected.push_back(point);
    }
    return projected;
}

QVector<int> SequentialIndexes(int count)
{
    QVector<int> indexes;
    indexes.reserve(count);
    for (int index = 0; index < count; ++index)
    {
        indexes.push_back(index);
    }
    return indexes;
}

bool CheckNear(double actual, double expected, double tolerance, const char* label)
{
    if (std::abs(actual - expected) <= tolerance)
    {
        return true;
    }
    std::cerr << label << " mismatch: actual=" << actual
              << " expected=" << expected << "\n";
    return false;
}

bool ContainsSourceIndex(
    const QVector<int>& keyIndexes,
    const QVector<GeometryProjectedPoint>& projected,
    int sourceIndex)
{
    for (int keyIndex : keyIndexes)
    {
        if (projected[keyIndex].inputIndex == sourceIndex)
        {
            return true;
        }
    }
    return false;
}

bool VerifyFourClassStatistics()
{
    const QVector<GeometryProjectedPoint> projected = BuildRobotCPeriodicKeys();
    const QVector<int> keys = SequentialIndexes(projected.size());
    const QVector<char> lapKeys(keys.size(), 0);
    constexpr double flatSlopeThreshold = 0.15;
    constexpr int minimumReferences = 3;

    return CheckNear(
               GeometryTypicalWaveSegmentLength(
                   projected, keys, lapKeys,
                   GeometryWaveSegmentClass::InnerToOuter,
                   -1, flatSlopeThreshold, minimumReferences),
               15.2458155, 0.001, "inner-to-outer")
        && CheckNear(
               GeometryTypicalWaveSegmentLength(
                   projected, keys, lapKeys,
                   GeometryWaveSegmentClass::OuterToInner,
                   -1, flatSlopeThreshold, minimumReferences),
               15.344487, 0.001, "outer-to-inner")
        && CheckNear(
               GeometryTypicalWaveSegmentLength(
                   projected, keys, lapKeys,
                   GeometryWaveSegmentClass::InnerPlatform,
                   -1, flatSlopeThreshold, minimumReferences),
               104.343060, 0.001, "inner-platform")
        && CheckNear(
               GeometryTypicalWaveSegmentLength(
                   projected, keys, lapKeys,
                   GeometryWaveSegmentClass::OuterPlatform,
                   14, flatSlopeThreshold, minimumReferences),
               97.352856, 0.001, "outer-platform");
}

bool VerifyRobotCFalseDoubleCornerIsMerged()
{
    const QVector<GeometryProjectedPoint> projected = BuildRobotCPeriodicKeys();
    const QVector<int> keys = SequentialIndexes(projected.size());
    const QVector<char> lapKeys(keys.size(), 0);
    int removedCount = 0;
    const QVector<int> merged = MergeTooCloseSameTypeCorners(
        projected, keys, lapKeys, 0.40, 0.15, 3, &removedCount);

    if (removedCount != 1
        || merged.size() != keys.size() - 1
        || ContainsSourceIndex(merged, projected, 1432)
        || !ContainsSourceIndex(merged, projected, 1440))
    {
        std::cerr << "RobotC false double corner was not merged as expected\n";
        return false;
    }
    return true;
}

bool VerifyReverseDirectionIsEquivalent()
{
    QVector<GeometryProjectedPoint> projected = BuildRobotCPeriodicKeys();
    std::reverse(projected.begin(), projected.end());
    const QVector<int> keys = SequentialIndexes(projected.size());
    const QVector<char> lapKeys(keys.size(), 0);
    int removedCount = 0;
    const QVector<int> merged = MergeTooCloseSameTypeCorners(
        projected, keys, lapKeys, 0.40, 0.15, 3, &removedCount);

    if (removedCount != 1
        || ContainsSourceIndex(merged, projected, 1432)
        || !ContainsSourceIndex(merged, projected, 1440))
    {
        std::cerr << "reverse-direction false double corner handling changed\n";
        return false;
    }
    return true;
}

bool VerifyLapAndTrueFlatPlatformAreProtected()
{
    const QVector<GeometryProjectedPoint> projected = BuildRobotCPeriodicKeys();
    const QVector<int> keys = SequentialIndexes(projected.size());

    QVector<char> lapKeys(keys.size(), 0);
    lapKeys[14] = 1;
    lapKeys[15] = 1;
    int removedCount = 0;
    const QVector<int> lapMerged = MergeTooCloseSameTypeCorners(
        projected, keys, lapKeys, 0.40, 0.15, 3, &removedCount);
    if (removedCount != 0 || lapMerged != keys)
    {
        std::cerr << "lap-step pair was incorrectly merged\n";
        return false;
    }

    QVector<GeometryProjectedPoint> flatProjected = projected;
    const double flatHeight = flatProjected[15].smoothH;
    for (int position : { 14, 15 })
    {
        flatProjected[position].h = flatHeight;
        flatProjected[position].smoothH = flatHeight;
        flatProjected[position].point.y() = flatHeight;
    }
    // 保持 1440 为 outer：这里只改变被排除的末端不完整段，不参与四类长度统计。
    flatProjected[16].h = 1172.0;
    flatProjected[16].smoothH = 1172.0;
    flatProjected[16].point.y() = 1172.0;

    removedCount = 0;
    const QVector<char> noLapKeys(keys.size(), 0);
    const QVector<int> flatMerged = MergeTooCloseSameTypeCorners(
        flatProjected, keys, noLapKeys, 0.40, 0.15, 3, &removedCount);
    if (removedCount != 0 || flatMerged != keys)
    {
        std::cerr << "true short flat platform was incorrectly merged\n";
        return false;
    }
    return true;
}

bool VerifyAmbiguousRecoveryIsConservative()
{
    QVector<GeometryProjectedPoint> projected = BuildRobotCPeriodicKeys();
    auto setPoint = [&](int position, double station, double height, double normal)
    {
        projected[position].point = Eigen::Vector3d(station, height, normal);
        projected[position].s = station;
        projected[position].h = height;
        projected[position].smoothH = height;
        projected[position].n = normal;
        projected[position].smoothN = normal;
    };

    // 构造 I-O-O-I：试删任意一个 O 都会恢复一条约 18.44 mm 的正常坡，
    // 两个方案相对各自 IO/OI 中位数的评分近乎相同，必须保守不删。
    setPoint(13, 188.3, 1170.0, -218.0);
    setPoint(14, 198.3, 1160.0, -218.0);
    setPoint(15, 202.3, 1158.0, -218.0);
    setPoint(16, 212.3, 1172.0, -218.0);
    projected[16].inputIndex = 1500;
    GeometryProjectedPoint endPoint = projected[16];
    endPoint.inputIndex = 1600;
    endPoint.point = Eigen::Vector3d(228.3, 1172.0, -218.0);
    endPoint.s = 228.3;
    projected.push_back(endPoint);

    auto verifyDirection = [](const QVector<GeometryProjectedPoint>& candidate, const char* label)
    {
        const QVector<int> keys = SequentialIndexes(candidate.size());
        const QVector<char> lapKeys(keys.size(), 0);
        int removedCount = 0;
        const QVector<int> merged = MergeTooCloseSameTypeCorners(
            candidate, keys, lapKeys, 0.40, 0.15, 3, &removedCount);
        if (removedCount != 0 || merged != keys)
        {
            std::cerr << label << " ambiguous recovery was not conservative\n";
            return false;
        }
        return true;
    };

    if (!verifyDirection(projected, "forward"))
    {
        return false;
    }
    std::reverse(projected.begin(), projected.end());
    return verifyDirection(projected, "reverse");
}

bool VerifyRobotC009TwoReferenceDoubleCornersAreMerged()
{
    auto verifyDirection = [](QVector<GeometryProjectedPoint> projected, const char* label)
    {
        const QVector<int> keys = SequentialIndexes(projected.size());
        const QVector<char> lapKeys(keys.size(), 0);
        int removedCount = 0;
        const QVector<int> merged = MergeTooCloseSameTypeCorners(
            projected, keys, lapKeys, 0.40, 0.15, 2, &removedCount);
        if (removedCount != 2
            || ContainsSourceIndex(merged, projected, 157)
            || ContainsSourceIndex(merged, projected, 1318)
            || !ContainsSourceIndex(merged, projected, 149)
            || !ContainsSourceIndex(merged, projected, 1326))
        {
            std::cerr << label
                      << " RobotC/20260724_009 two-reference double corners were not merged\n";
            return false;
        }
        return true;
    };

    QVector<GeometryProjectedPoint> projected = BuildRobotC009PeriodicKeys();
    {
        const QVector<int> keys = SequentialIndexes(projected.size());
        const QVector<char> lapKeys(keys.size(), 0);
        int removedCount = 0;
        const QVector<int> conservative = MergeTooCloseSameTypeCorners(
            projected, keys, lapKeys, 0.40, 0.15, 3, &removedCount);
        if (removedCount != 0 || conservative != keys)
        {
            std::cerr << "minimum reference segment parameter was not honored\n";
            return false;
        }
    }
    if (!verifyDirection(projected, "forward"))
    {
        return false;
    }
    std::reverse(projected.begin(), projected.end());
    return verifyDirection(projected, "reverse");
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    if (!VerifyFourClassStatistics()) return 1;
    if (!VerifyRobotCFalseDoubleCornerIsMerged()) return 2;
    if (!VerifyReverseDirectionIsEquivalent()) return 3;
    if (!VerifyLapAndTrueFlatPlatformAreProtected()) return 4;
    if (!VerifyAmbiguousRecoveryIsConservative()) return 5;
    if (!VerifyRobotC009TwoReferenceDoubleCornersAreMerged()) return 6;
    std::cout << "PASS: four-class periodic corner pairing regression\n";
    return 0;
}
