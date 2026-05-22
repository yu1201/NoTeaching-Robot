#include "LaserFramePoint3DFilter.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

std::vector<cv::Point3d> FilterSingleFrameLaserPoint3D(
    const std::vector<cv::Point3d>& inputPoints,
    const LaserFramePoint3DFilterOptions& options)
{
    std::vector<cv::Point3d> finitePoints;
    finitePoints.reserve(inputPoints.size());
    for (const cv::Point3d& point : inputPoints)
    {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        {
            finitePoints.push_back(point);
        }
    }

    const int minPointCount = std::max(1, options.minPointCount);
    if (finitePoints.size() < static_cast<size_t>(minPointCount))
    {
        return finitePoints;
    }

    const auto medianValue = [](std::vector<double> values) -> double
    {
        if (values.empty())
        {
            return 0.0;
        }

        std::sort(values.begin(), values.end());
        const size_t mid = values.size() / 2;
        if ((values.size() % 2) == 0)
        {
            return (values[mid - 1] + values[mid]) * 0.5;
        }
        return values[mid];
    };

    const auto distance3D = [](const cv::Point3d& a, const cv::Point3d& b) -> double
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    const auto clampProfileAxis = [](const int axis) -> int
    {
        return (axis >= 0 && axis <= 2) ? axis : 0;
    };

    const int profileAxis0 = clampProfileAxis(options.profileAxis0);
    int profileAxis1 = clampProfileAxis(options.profileAxis1);
    if (profileAxis1 == profileAxis0)
    {
        profileAxis1 = (profileAxis0 == 2) ? 1 : 2;
    }

    const auto profileCoordinate = [](const cv::Point3d& point, const int axis) -> double
    {
        switch (axis)
        {
        case 1:
            return point.y;
        case 2:
            return point.z;
        default:
            return point.x;
        }
    };

    const auto distanceProfile2D = [&](const cv::Point3d& a, const cv::Point3d& b) -> double
    {
        const double da = profileCoordinate(a, profileAxis0) - profileCoordinate(b, profileAxis0);
        const double db = profileCoordinate(a, profileAxis1) - profileCoordinate(b, profileAxis1);
        return std::sqrt(da * da + db * db);
    };

    const auto pointSegmentDistanceProfile2D = [&](const cv::Point3d& point,
                                                    const cv::Point3d& segmentStart,
                                                    const cv::Point3d& segmentEnd) -> double
    {
        const double sx = profileCoordinate(segmentStart, profileAxis0);
        const double sy = profileCoordinate(segmentStart, profileAxis1);
        const double ex = profileCoordinate(segmentEnd, profileAxis0);
        const double ey = profileCoordinate(segmentEnd, profileAxis1);
        const double px = profileCoordinate(point, profileAxis0);
        const double py = profileCoordinate(point, profileAxis1);
        const double vx = ex - sx;
        const double vy = ey - sy;
        const double wx = px - sx;
        const double wy = py - sy;
        const double lengthSquared = vx * vx + vy * vy;
        if (lengthSquared <= 1.0e-12)
        {
            const double dx = px - sx;
            const double dy = py - sy;
            return std::sqrt(dx * dx + dy * dy);
        }

        const double tRaw = (wx * vx + wy * vy) / lengthSquared;
        const double t = std::max(0.0, std::min(1.0, tRaw));
        const double projectionX = sx + vx * t;
        const double projectionY = sy + vy * t;
        const double dx = px - projectionX;
        const double dy = py - projectionY;
        return std::sqrt(dx * dx + dy * dy);
    };

    const auto pointSegmentDistance3D = [&distance3D](const cv::Point3d& point,
                                                       const cv::Point3d& segmentStart,
                                                       const cv::Point3d& segmentEnd) -> double
    {
        const double vx = segmentEnd.x - segmentStart.x;
        const double vy = segmentEnd.y - segmentStart.y;
        const double vz = segmentEnd.z - segmentStart.z;
        const double wx = point.x - segmentStart.x;
        const double wy = point.y - segmentStart.y;
        const double wz = point.z - segmentStart.z;
        const double lengthSquared = vx * vx + vy * vy + vz * vz;
        if (lengthSquared <= 1.0e-12)
        {
            return distance3D(point, segmentStart);
        }

        const double tRaw = (wx * vx + wy * vy + wz * vz) / lengthSquared;
        const double t = std::max(0.0, std::min(1.0, tRaw));
        cv::Point3d projection;
        projection.x = segmentStart.x + vx * t;
        projection.y = segmentStart.y + vy * t;
        projection.z = segmentStart.z + vz * t;
        return distance3D(point, projection);
    };

    const auto extractDominantLineSegments = [&](const std::vector<cv::Point3d>& sourcePoints) -> std::vector<cv::Point3d>
    {
        const int minSegmentCount = std::max(1, options.dominantLineMinSegmentCount);
        const int maxSegmentCount = std::max(minSegmentCount, options.dominantLineMaxSegmentCount);
        const int minPointCountPerSegment = std::max(2, options.dominantLineMinPointCount);
        if (sourcePoints.size() < static_cast<size_t>(minSegmentCount * minPointCountPerSegment))
        {
            return sourcePoints;
        }

        std::vector<double> profileStepDistances;
        profileStepDistances.reserve(sourcePoints.size() - 1);
        for (int i = 1; i < static_cast<int>(sourcePoints.size()); ++i)
        {
            const double step = distanceProfile2D(sourcePoints[i - 1], sourcePoints[i]);
            if (step > 1.0e-9)
            {
                profileStepDistances.push_back(step);
            }
        }

        const double profileMedianStep = profileStepDistances.empty()
            ? 0.0
            : medianValue(profileStepDistances);
        const double splitGap = std::max(options.dominantLineSplitGapMinMm,
                                         profileMedianStep * options.dominantLineSplitGapStepScale);
        const double lineDistanceThreshold = std::max(options.dominantLineDistanceThresholdMinMm,
                                                      profileMedianStep * options.dominantLineDistanceThresholdStepScale);
        const double rhoBinSize = std::max(0.1, lineDistanceThreshold);
        const double angleStepDegrees = 0.5;
        const int angleBucketCount = static_cast<int>(180.0 / angleStepDegrees);
        constexpr double kPi = 3.14159265358979323846;

        std::vector<double> profileA(sourcePoints.size(), 0.0);
        std::vector<double> profileB(sourcePoints.size(), 0.0);
        for (int i = 0; i < static_cast<int>(sourcePoints.size()); ++i)
        {
            profileA[i] = profileCoordinate(sourcePoints[i], profileAxis0);
            profileB[i] = profileCoordinate(sourcePoints[i], profileAxis1);
        }

        struct SegmentCandidate
        {
            int begin = 0;
            int end = 0;
            int count = 0;
            double span = 0.0;
            double score = 0.0;
            double normalA = 0.0;
            double normalB = 0.0;
            double directionA = 0.0;
            double directionB = 0.0;
            double rho = 0.0;
            double projectionMin = 0.0;
            double projectionMax = 0.0;
            std::vector<int> indices;
        };

        struct RecoveryInterval
        {
            double projectionMin = 0.0;
            double projectionMax = 0.0;
        };

        struct LineIntersection
        {
            bool valid = false;
            int segmentA = -1;
            int segmentB = -1;
            double pointA = 0.0;
            double pointB = 0.0;
            double projectionA = 0.0;
            double projectionB = 0.0;
        };

        std::vector<int> remainingIndices;
        remainingIndices.reserve(sourcePoints.size());
        for (int i = 0; i < static_cast<int>(sourcePoints.size()); ++i)
        {
            remainingIndices.push_back(i);
        }

        std::vector<SegmentCandidate> selectedSegments;
        const auto selectFastPairCandidate = [&](const std::vector<int>& activeIndices,
                                                 SegmentCandidate& bestCandidate) -> bool
        {
            if (activeIndices.size() < static_cast<size_t>(minPointCountPerSegment))
            {
                return false;
            }

            struct RoughCandidate
            {
                double normalA = 0.0;
                double normalB = 0.0;
                double directionA = 0.0;
                double directionB = 0.0;
                double rho = 0.0;
                double score = 0.0;
                int count = 0;
            };

            struct ProjectedPoint
            {
                int index = 0;
                double projection = 0.0;
            };

            const double roughDistanceThreshold = std::max(
                lineDistanceThreshold * 2.2,
                lineDistanceThreshold + 0.8);
            const int maxSampleCount = std::max(2, options.dominantLineFastSampleCount);
            const int sampleStep = std::max(
                1,
                static_cast<int>(std::ceil(static_cast<double>(activeIndices.size())
                    / static_cast<double>(maxSampleCount))));
            std::vector<int> sampleIndices;
            sampleIndices.reserve(std::min(static_cast<int>(activeIndices.size()), maxSampleCount + 1));
            for (int localIndex = 0; localIndex < static_cast<int>(activeIndices.size()); localIndex += sampleStep)
            {
                sampleIndices.push_back(activeIndices[localIndex]);
            }
            if (sampleIndices.empty() || sampleIndices.back() != activeIndices.back())
            {
                sampleIndices.push_back(activeIndices.back());
            }

            std::vector<RoughCandidate> roughCandidates;
            roughCandidates.reserve(sampleIndices.size() * 4);
            for (int leftSample = 0; leftSample < static_cast<int>(sampleIndices.size()); ++leftSample)
            {
                const int leftIndex = sampleIndices[leftSample];
                const double leftA = profileA[leftIndex];
                const double leftB = profileB[leftIndex];
                for (int rightSample = leftSample + 1; rightSample < static_cast<int>(sampleIndices.size()); ++rightSample)
                {
                    const int rightIndex = sampleIndices[rightSample];
                    const double deltaA = profileA[rightIndex] - leftA;
                    const double deltaB = profileB[rightIndex] - leftB;
                    const double lengthSquared = deltaA * deltaA + deltaB * deltaB;
                    if (lengthSquared < options.dominantLineMinSpanMm * options.dominantLineMinSpanMm)
                    {
                        continue;
                    }

                    const double length = std::sqrt(lengthSquared);
                    const double directionA = deltaA / length;
                    const double directionB = deltaB / length;
                    const double normalA = -directionB;
                    const double normalB = directionA;
                    const double rho = leftA * normalA + leftB * normalB;

                    int roughCount = 0;
                    double projectionMin = std::numeric_limits<double>::max();
                    double projectionMax = -std::numeric_limits<double>::max();
                    for (int pointIndex : activeIndices)
                    {
                        const double pointRho = profileA[pointIndex] * normalA
                            + profileB[pointIndex] * normalB;
                        if (std::abs(pointRho - rho) > roughDistanceThreshold)
                        {
                            continue;
                        }

                        const double projection = profileA[pointIndex] * directionA
                            + profileB[pointIndex] * directionB;
                        projectionMin = std::min(projectionMin, projection);
                        projectionMax = std::max(projectionMax, projection);
                        ++roughCount;
                    }

                    if (roughCount < minPointCountPerSegment)
                    {
                        continue;
                    }

                    const double roughSpan = projectionMax - projectionMin;
                    if (roughSpan < options.dominantLineMinSpanMm)
                    {
                        continue;
                    }

                    RoughCandidate roughCandidate;
                    roughCandidate.normalA = normalA;
                    roughCandidate.normalB = normalB;
                    roughCandidate.directionA = directionA;
                    roughCandidate.directionB = directionB;
                    roughCandidate.rho = rho;
                    roughCandidate.count = roughCount;
                    roughCandidate.score = roughSpan * static_cast<double>(roughCount);
                    roughCandidates.push_back(roughCandidate);
                }
            }

            if (roughCandidates.empty())
            {
                return false;
            }

            const int requestedCandidateCount = std::max(1, options.dominantLineFastCandidateCount);
            const int fullCandidateCount = std::min(requestedCandidateCount, static_cast<int>(roughCandidates.size()));
            std::partial_sort(
                roughCandidates.begin(),
                roughCandidates.begin() + fullCandidateCount,
                roughCandidates.end(),
                [](const RoughCandidate& left, const RoughCandidate& right)
            {
                if (left.score != right.score)
                {
                    return left.score > right.score;
                }
                return left.count > right.count;
            });

            bool hasBestCandidate = false;
            for (int candidateIndex = 0; candidateIndex < fullCandidateCount; ++candidateIndex)
            {
                const RoughCandidate& roughCandidate = roughCandidates[candidateIndex];
                std::vector<int> roughInlierIndices;
                roughInlierIndices.reserve(roughCandidate.count);
                double rhoSum = 0.0;
                for (int pointIndex : activeIndices)
                {
                    const double rho = profileA[pointIndex] * roughCandidate.normalA
                        + profileB[pointIndex] * roughCandidate.normalB;
                    if (std::abs(rho - roughCandidate.rho) > roughDistanceThreshold)
                    {
                        continue;
                    }

                    roughInlierIndices.push_back(pointIndex);
                    rhoSum += rho;
                }

                if (roughInlierIndices.size() < static_cast<size_t>(minPointCountPerSegment))
                {
                    continue;
                }

                const double averageRho = rhoSum / static_cast<double>(roughInlierIndices.size());
                std::vector<ProjectedPoint> projectedPoints;
                projectedPoints.reserve(roughInlierIndices.size());
                for (int pointIndex : roughInlierIndices)
                {
                    const double rho = profileA[pointIndex] * roughCandidate.normalA
                        + profileB[pointIndex] * roughCandidate.normalB;
                    if (std::abs(rho - averageRho) > lineDistanceThreshold)
                    {
                        continue;
                    }

                    ProjectedPoint projectedPoint;
                    projectedPoint.index = pointIndex;
                    projectedPoint.projection = profileA[pointIndex] * roughCandidate.directionA
                        + profileB[pointIndex] * roughCandidate.directionB;
                    projectedPoints.push_back(projectedPoint);
                }

                if (projectedPoints.size() < static_cast<size_t>(minPointCountPerSegment))
                {
                    continue;
                }

                std::sort(projectedPoints.begin(), projectedPoints.end(), [](const ProjectedPoint& left,
                                                                             const ProjectedPoint& right)
                {
                    return left.projection < right.projection;
                });

                int runBegin = 0;
                for (int runEnd = 1; runEnd <= static_cast<int>(projectedPoints.size()); ++runEnd)
                {
                    if (runEnd < static_cast<int>(projectedPoints.size())
                        && splitGap > 1.0e-9
                        && (projectedPoints[runEnd].projection - projectedPoints[runEnd - 1].projection) <= splitGap)
                    {
                        continue;
                    }

                    const int runCount = runEnd - runBegin;
                    if (runCount >= minPointCountPerSegment)
                    {
                        const double span = projectedPoints[runEnd - 1].projection
                            - projectedPoints[runBegin].projection;
                        if (span >= options.dominantLineMinSpanMm)
                        {
                            SegmentCandidate candidate;
                            candidate.begin = std::numeric_limits<int>::max();
                            candidate.end = -1;
                            candidate.count = runCount;
                            candidate.span = span;
                            candidate.score = span * static_cast<double>(runCount);
                            candidate.normalA = roughCandidate.normalA;
                            candidate.normalB = roughCandidate.normalB;
                            candidate.directionA = roughCandidate.directionA;
                            candidate.directionB = roughCandidate.directionB;
                            candidate.rho = averageRho;
                            candidate.projectionMin = projectedPoints[runBegin].projection;
                            candidate.projectionMax = projectedPoints[runEnd - 1].projection;
                            candidate.indices.reserve(runCount);
                            for (int i = runBegin; i < runEnd; ++i)
                            {
                                candidate.indices.push_back(projectedPoints[i].index);
                                candidate.begin = std::min(candidate.begin, projectedPoints[i].index);
                                candidate.end = std::max(candidate.end, projectedPoints[i].index);
                            }

                            if (!hasBestCandidate
                                || candidate.score > bestCandidate.score
                                || (candidate.score == bestCandidate.score && candidate.count > bestCandidate.count))
                            {
                                bestCandidate = candidate;
                                hasBestCandidate = true;
                            }
                        }
                    }

                    runBegin = runEnd;
                }
            }

            return hasBestCandidate;
        };

        for (int segmentIndex = 0; segmentIndex < maxSegmentCount; ++segmentIndex)
        {
            SegmentCandidate bestCandidate;
            if (!selectFastPairCandidate(remainingIndices, bestCandidate))
            {
                break;
            }

            selectedSegments.push_back(bestCandidate);
            std::vector<unsigned char> removePoint(sourcePoints.size(), 0);
            for (int pointIndex : bestCandidate.indices)
            {
                removePoint[pointIndex] = 1;
            }

            std::vector<int> nextRemainingIndices;
            nextRemainingIndices.reserve(remainingIndices.size());
            for (int pointIndex : remainingIndices)
            {
                if (!removePoint[pointIndex])
                {
                    nextRemainingIndices.push_back(pointIndex);
                }
            }
            remainingIndices.swap(nextRemainingIndices);
        }

        if (selectedSegments.size() < static_cast<size_t>(minSegmentCount))
        {
            selectedSegments.clear();
            remainingIndices.clear();
            remainingIndices.reserve(sourcePoints.size());
            for (int i = 0; i < static_cast<int>(sourcePoints.size()); ++i)
            {
                remainingIndices.push_back(i);
            }

            for (int segmentIndex = 0; segmentIndex < maxSegmentCount; ++segmentIndex)
        {
            if (remainingIndices.size() < static_cast<size_t>(minPointCountPerSegment))
            {
                break;
            }

            SegmentCandidate bestCandidate;
            bool hasBestCandidate = false;
            for (int angleIndex = 0; angleIndex < angleBucketCount; ++angleIndex)
            {
                const double angleRadians = (static_cast<double>(angleIndex) * angleStepDegrees) * kPi / 180.0;
                const double normalA = std::cos(angleRadians);
                const double normalB = std::sin(angleRadians);

                double minRho = std::numeric_limits<double>::max();
                double maxRho = -std::numeric_limits<double>::max();
                std::vector<double> rhos;
                rhos.reserve(remainingIndices.size());
                for (int pointIndex : remainingIndices)
                {
                    const cv::Point3d& point = sourcePoints[pointIndex];
                    const double rho = profileCoordinate(point, profileAxis0) * normalA
                        + profileCoordinate(point, profileAxis1) * normalB;
                    rhos.push_back(rho);
                    minRho = std::min(minRho, rho);
                    maxRho = std::max(maxRho, rho);
                }

                const int bucketCount = std::max(1, static_cast<int>(std::ceil((maxRho - minRho) / rhoBinSize)) + 1);
                std::vector<std::vector<int>> rhoBuckets(bucketCount);
                for (int localIndex = 0; localIndex < static_cast<int>(remainingIndices.size()); ++localIndex)
                {
                    const int bucketIndex = std::max(0, std::min(
                        bucketCount - 1,
                        static_cast<int>(std::floor((rhos[localIndex] - minRho) / rhoBinSize))));
                    rhoBuckets[bucketIndex].push_back(remainingIndices[localIndex]);
                }

                for (int bucketIndex = 0; bucketIndex < bucketCount; ++bucketIndex)
                {
                    int roughCount = static_cast<int>(rhoBuckets[bucketIndex].size());
                    if (bucketIndex > 0)
                    {
                        roughCount += static_cast<int>(rhoBuckets[bucketIndex - 1].size());
                    }
                    if (bucketIndex + 1 < bucketCount)
                    {
                        roughCount += static_cast<int>(rhoBuckets[bucketIndex + 1].size());
                    }
                    if (roughCount < minPointCountPerSegment)
                    {
                        continue;
                    }

                    std::vector<int> bucketPointIndices;
                    bucketPointIndices.reserve(roughCount);
                    const int bucketBegin = std::max(0, bucketIndex - 1);
                    const int bucketEnd = std::min(bucketCount - 1, bucketIndex + 1);
                    for (int mergeBucket = bucketBegin; mergeBucket <= bucketEnd; ++mergeBucket)
                    {
                        bucketPointIndices.insert(bucketPointIndices.end(),
                                                  rhoBuckets[mergeBucket].begin(),
                                                  rhoBuckets[mergeBucket].end());
                    }

                    double rhoSum = 0.0;
                    for (int pointIndex : bucketPointIndices)
                    {
                        const cv::Point3d& point = sourcePoints[pointIndex];
                        rhoSum += profileCoordinate(point, profileAxis0) * normalA
                            + profileCoordinate(point, profileAxis1) * normalB;
                    }
                    const double averageRho = rhoSum / static_cast<double>(bucketPointIndices.size());

                    struct ProjectedPoint
                    {
                        int index = 0;
                        double projection = 0.0;
                    };

                    std::vector<ProjectedPoint> projectedPoints;
                    projectedPoints.reserve(bucketPointIndices.size());
                    const double directionA = -normalB;
                    const double directionB = normalA;
                    for (int pointIndex : bucketPointIndices)
                    {
                        const cv::Point3d& point = sourcePoints[pointIndex];
                        const double rho = profileCoordinate(point, profileAxis0) * normalA
                            + profileCoordinate(point, profileAxis1) * normalB;
                        if (std::abs(rho - averageRho) > lineDistanceThreshold)
                        {
                            continue;
                        }

                        ProjectedPoint projectedPoint;
                        projectedPoint.index = pointIndex;
                        projectedPoint.projection = profileCoordinate(point, profileAxis0) * directionA
                            + profileCoordinate(point, profileAxis1) * directionB;
                        projectedPoints.push_back(projectedPoint);
                    }

                    if (projectedPoints.size() < static_cast<size_t>(minPointCountPerSegment))
                    {
                        continue;
                    }

                    std::sort(projectedPoints.begin(), projectedPoints.end(), [](const ProjectedPoint& left,
                                                                                 const ProjectedPoint& right)
                    {
                        return left.projection < right.projection;
                    });

                    int runBegin = 0;
                    for (int runEnd = 1; runEnd <= static_cast<int>(projectedPoints.size()); ++runEnd)
                    {
                        if (runEnd < static_cast<int>(projectedPoints.size())
                            && splitGap > 1.0e-9
                            && (projectedPoints[runEnd].projection - projectedPoints[runEnd - 1].projection) <= splitGap)
                        {
                            continue;
                        }

                        const int runCount = runEnd - runBegin;
                        if (runCount >= minPointCountPerSegment)
                        {
                            const double span = projectedPoints[runEnd - 1].projection
                                - projectedPoints[runBegin].projection;
                            if (span >= options.dominantLineMinSpanMm)
                            {
                                SegmentCandidate candidate;
                                candidate.begin = std::numeric_limits<int>::max();
                                candidate.end = -1;
                                candidate.count = runCount;
                                candidate.span = span;
                                candidate.score = span * static_cast<double>(runCount);
                                candidate.normalA = normalA;
                                candidate.normalB = normalB;
                                candidate.directionA = directionA;
                                candidate.directionB = directionB;
                                candidate.rho = averageRho;
                                candidate.projectionMin = projectedPoints[runBegin].projection;
                                candidate.projectionMax = projectedPoints[runEnd - 1].projection;
                                candidate.indices.reserve(runCount);
                                for (int i = runBegin; i < runEnd; ++i)
                                {
                                    candidate.indices.push_back(projectedPoints[i].index);
                                    candidate.begin = std::min(candidate.begin, projectedPoints[i].index);
                                    candidate.end = std::max(candidate.end, projectedPoints[i].index);
                                }

                                if (!hasBestCandidate
                                    || candidate.score > bestCandidate.score
                                    || (candidate.score == bestCandidate.score && candidate.count > bestCandidate.count))
                                {
                                    bestCandidate = candidate;
                                    hasBestCandidate = true;
                                }
                            }
                        }

                        runBegin = runEnd;
                    }
                }
            }

            if (!hasBestCandidate)
            {
                break;
            }

            selectedSegments.push_back(bestCandidate);
            std::vector<unsigned char> removePoint(sourcePoints.size(), 0);
            for (int pointIndex : bestCandidate.indices)
            {
                removePoint[pointIndex] = 1;
            }

            std::vector<int> nextRemainingIndices;
            nextRemainingIndices.reserve(remainingIndices.size());
            for (int pointIndex : remainingIndices)
            {
                if (!removePoint[pointIndex])
                {
                    nextRemainingIndices.push_back(pointIndex);
                }
            }
            remainingIndices.swap(nextRemainingIndices);
        }
        }

        if (selectedSegments.size() < static_cast<size_t>(minSegmentCount))
        {
            return sourcePoints;
        }

        std::vector<unsigned char> keepPoint(sourcePoints.size(), 0);
        for (const SegmentCandidate& segment : selectedSegments)
        {
            for (int pointIndex : segment.indices)
            {
                keepPoint[pointIndex] = 1;
            }
        }

        if (options.enableDominantLineTrendRecovery)
        {
            const double recoverDistanceThreshold = std::max(
                options.dominantLineTrendRecoverDistanceMinMm,
                profileMedianStep * options.dominantLineTrendRecoverDistanceStepScale);
            const double endpointTolerance = std::max(0.0, options.dominantLineTrendRecoverEndpointToleranceMm);
            std::vector<RecoveryInterval> recoveryIntervals(selectedSegments.size());
            for (int segmentIndex = 0; segmentIndex < static_cast<int>(selectedSegments.size()); ++segmentIndex)
            {
                recoveryIntervals[segmentIndex].projectionMin = selectedSegments[segmentIndex].projectionMin;
                recoveryIntervals[segmentIndex].projectionMax = selectedSegments[segmentIndex].projectionMax;
            }

            const auto projectionOutsideDistance = [](const double projection,
                                                      const SegmentCandidate& segment) -> double
            {
                if (projection < segment.projectionMin)
                {
                    return segment.projectionMin - projection;
                }
                if (projection > segment.projectionMax)
                {
                    return projection - segment.projectionMax;
                }
                return 0.0;
            };

            const auto intersectSegments = [&](const int leftIndex,
                                               const int rightIndex) -> LineIntersection
            {
                LineIntersection intersection;
                intersection.segmentA = leftIndex;
                intersection.segmentB = rightIndex;
                const SegmentCandidate& left = selectedSegments[leftIndex];
                const SegmentCandidate& right = selectedSegments[rightIndex];
                const double determinant = left.normalA * right.normalB
                    - right.normalA * left.normalB;
                if (std::abs(determinant) <= 1.0e-6)
                {
                    return intersection;
                }

                intersection.valid = true;
                intersection.pointA = (left.rho * right.normalB - right.rho * left.normalB) / determinant;
                intersection.pointB = (left.normalA * right.rho - right.normalA * left.rho) / determinant;
                intersection.projectionA = intersection.pointA * left.directionA
                    + intersection.pointB * left.directionB;
                intersection.projectionB = intersection.pointA * right.directionA
                    + intersection.pointB * right.directionB;
                return intersection;
            };

            std::vector<LineIntersection> polylineIntersections;
            if (selectedSegments.size() >= 3)
            {
                int bestMiddleIndex = -1;
                double bestMiddleScore = std::numeric_limits<double>::max();
                std::vector<LineIntersection> bestMiddleIntersections;
                for (int middleIndex = 0; middleIndex < static_cast<int>(selectedSegments.size()); ++middleIndex)
                {
                    std::vector<LineIntersection> intersections;
                    double score = 0.0;
                    for (int otherIndex = 0; otherIndex < static_cast<int>(selectedSegments.size()); ++otherIndex)
                    {
                        if (otherIndex == middleIndex)
                        {
                            continue;
                        }

                        LineIntersection intersection = intersectSegments(middleIndex, otherIndex);
                        if (!intersection.valid)
                        {
                            continue;
                        }

                        score += projectionOutsideDistance(intersection.projectionA, selectedSegments[middleIndex]);
                        score += projectionOutsideDistance(intersection.projectionB, selectedSegments[otherIndex]);
                        intersections.push_back(intersection);
                    }

                    if (intersections.size() >= 2 && score < bestMiddleScore)
                    {
                        bestMiddleIndex = middleIndex;
                        bestMiddleScore = score;
                        bestMiddleIntersections = intersections;
                    }
                }

                if (bestMiddleIndex >= 0 && bestMiddleIntersections.size() >= 2)
                {
                    std::sort(bestMiddleIntersections.begin(),
                              bestMiddleIntersections.end(),
                              [](const LineIntersection& left,
                                 const LineIntersection& right)
                    {
                        return left.projectionA < right.projectionA;
                    });

                    const LineIntersection& first = bestMiddleIntersections.front();
                    const LineIntersection& last = bestMiddleIntersections.back();
                    recoveryIntervals[bestMiddleIndex].projectionMin = std::min(first.projectionA, last.projectionA);
                    recoveryIntervals[bestMiddleIndex].projectionMax = std::max(first.projectionA, last.projectionA);
                    polylineIntersections.push_back(first);
                    polylineIntersections.push_back(last);
                }
            }
            else if (selectedSegments.size() == 2)
            {
                LineIntersection intersection = intersectSegments(0, 1);
                if (intersection.valid)
                {
                    polylineIntersections.push_back(intersection);
                }
            }

            for (const LineIntersection& intersection : polylineIntersections)
            {
                recoveryIntervals[intersection.segmentA].projectionMin = std::min(
                    recoveryIntervals[intersection.segmentA].projectionMin,
                    intersection.projectionA);
                recoveryIntervals[intersection.segmentA].projectionMax = std::max(
                    recoveryIntervals[intersection.segmentA].projectionMax,
                    intersection.projectionA);
                recoveryIntervals[intersection.segmentB].projectionMin = std::min(
                    recoveryIntervals[intersection.segmentB].projectionMin,
                    intersection.projectionB);
                recoveryIntervals[intersection.segmentB].projectionMax = std::max(
                    recoveryIntervals[intersection.segmentB].projectionMax,
                    intersection.projectionB);
            }

            for (int pointIndex : remainingIndices)
            {
                const cv::Point3d& point = sourcePoints[pointIndex];
                const double pointA = profileCoordinate(point, profileAxis0);
                const double pointB = profileCoordinate(point, profileAxis1);
                for (int segmentIndex = 0; segmentIndex < static_cast<int>(selectedSegments.size()); ++segmentIndex)
                {
                    const SegmentCandidate& segment = selectedSegments[segmentIndex];
                    const RecoveryInterval& interval = recoveryIntervals[segmentIndex];
                    const double rho = pointA * segment.normalA + pointB * segment.normalB;
                    if (std::abs(rho - segment.rho) > recoverDistanceThreshold)
                    {
                        continue;
                    }

                    const double projection = pointA * segment.directionA + pointB * segment.directionB;
                    if (projection < interval.projectionMin - endpointTolerance
                        || projection > interval.projectionMax + endpointTolerance)
                    {
                        continue;
                    }

                    keepPoint[pointIndex] = 1;
                    break;
                }
            }
        }

        std::vector<cv::Point3d> dominantPoints;
        dominantPoints.reserve(sourcePoints.size());
        for (int i = 0; i < static_cast<int>(sourcePoints.size()); ++i)
        {
            if (keepPoint[i])
            {
                dominantPoints.push_back(sourcePoints[i]);
            }
        }

        return dominantPoints.empty() ? sourcePoints : dominantPoints;
    };

    if (options.enableDominantLineSegmentFilter)
    {
        const std::vector<cv::Point3d> dominantRawPoints = extractDominantLineSegments(finitePoints);
        if (dominantRawPoints.size() < finitePoints.size())
        {
            return dominantRawPoints;
        }
    }

    std::vector<double> stepDistances;
    stepDistances.reserve(finitePoints.size() - 1);
    for (int i = 1; i < static_cast<int>(finitePoints.size()); ++i)
    {
        const double step = distance3D(finitePoints[i - 1], finitePoints[i]);
        if (step > 1.0e-9)
        {
            stepDistances.push_back(step);
        }
    }

    const double medianStep = stepDistances.empty() ? 0.0 : medianValue(stepDistances);
    const double supportRadius = std::max(options.supportRadiusMinMm, medianStep * options.supportRadiusStepScale);

    const int medianWindowRadius = std::max(1, options.medianWindowRadius);
    std::vector<double> residuals;
    residuals.reserve(finitePoints.size());

    for (int i = 0; i < static_cast<int>(finitePoints.size()); ++i)
    {
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        xs.reserve(medianWindowRadius * 2 + 1);
        ys.reserve(medianWindowRadius * 2 + 1);
        zs.reserve(medianWindowRadius * 2 + 1);

        const int begin = std::max(0, i - medianWindowRadius);
        const int end = std::min(static_cast<int>(finitePoints.size()) - 1, i + medianWindowRadius);
        for (int j = begin; j <= end; ++j)
        {
            xs.push_back(finitePoints[j].x);
            ys.push_back(finitePoints[j].y);
            zs.push_back(finitePoints[j].z);
        }

        cv::Point3d localMedian;
        localMedian.x = medianValue(xs);
        localMedian.y = medianValue(ys);
        localMedian.z = medianValue(zs);
        residuals.push_back(distance3D(finitePoints[i], localMedian));
    }

    const double residualMedian = medianValue(residuals);
    std::vector<double> absoluteDeviation;
    absoluteDeviation.reserve(residuals.size());
    for (double residual : residuals)
    {
        absoluteDeviation.push_back(std::abs(residual - residualMedian));
    }

    const double mad = medianValue(absoluteDeviation);
    const double robustSigma = 1.4826 * mad;
    const double threshold = std::max(options.residualThresholdMinMm,
                                      residualMedian + robustSigma * options.residualSigmaScale);
    const double bridgeDistanceThreshold = std::max(options.bridgeDistanceThresholdMinMm,
                                                    medianStep * options.bridgeDistanceStepScale);
    const double bridgeMaxLength = std::max(options.bridgeMaxLengthMinMm,
                                            medianStep * options.bridgeMaxLengthStepScale);

    std::vector<char> keep(finitePoints.size(), 1);
    const int supportIndexRadius = std::max(1, options.supportIndexRadius);
    const int endpointPreserveCount = std::max(0, options.endpointPreserveCount);
    const int endpointMinSupportCount = std::max(0, options.endpointMinSupportCount);
    const int middleMinSupportCount = std::max(0, options.middleMinSupportCount);
    const int localSpikeSupportMaxCount = std::max(0, options.localSpikeSupportMaxCount);
    for (int i = 0; i < static_cast<int>(finitePoints.size()); ++i)
    {
        int supportCount = 0;
        const int supportBegin = std::max(0, i - supportIndexRadius);
        const int supportEnd = std::min(static_cast<int>(finitePoints.size()) - 1, i + supportIndexRadius);
        for (int j = supportBegin; j <= supportEnd; ++j)
        {
            if (j == i)
            {
                continue;
            }

            if (distance3D(finitePoints[i], finitePoints[j]) <= supportRadius)
            {
                ++supportCount;
            }
        }

        const bool isEndpoint = (i < endpointPreserveCount
            || i >= static_cast<int>(finitePoints.size()) - endpointPreserveCount);
        const int minSupportCount = isEndpoint ? endpointMinSupportCount : middleMinSupportCount;
        const bool hasLocalSupport = supportCount >= minSupportCount;
        const bool isLocalSpike = supportCount <= localSpikeSupportMaxCount
            && residuals[i] > std::max(options.localSpikeThresholdMinMm,
                                       threshold * options.localSpikeThresholdScale);

        // Do not use residual as a hard delete condition here: real corners and
        // short platforms can deviate from the local median but still be valid.
        // Residual is only used together with weak support to remove isolated spikes.
        keep[i] = hasLocalSupport && !isLocalSpike;
    }

    if (options.enableBridgeSpikeFilter)
    {
        const int bridgeInnerGap = std::max(1, options.bridgeInnerGap);
        const int bridgeOuterRadius = std::max(bridgeInnerGap + 1, options.bridgeOuterRadius);
        const int bridgeMinNeighborCount = std::max(1, options.bridgeMinNeighborCount);

        for (int i = bridgeInnerGap; i < static_cast<int>(finitePoints.size()) - bridgeInnerGap; ++i)
        {
            std::vector<double> prevXs;
            std::vector<double> prevYs;
            std::vector<double> prevZs;
            std::vector<double> nextXs;
            std::vector<double> nextYs;
            std::vector<double> nextZs;
            prevXs.reserve(bridgeOuterRadius - bridgeInnerGap + 1);
            prevYs.reserve(bridgeOuterRadius - bridgeInnerGap + 1);
            prevZs.reserve(bridgeOuterRadius - bridgeInnerGap + 1);
            nextXs.reserve(bridgeOuterRadius - bridgeInnerGap + 1);
            nextYs.reserve(bridgeOuterRadius - bridgeInnerGap + 1);
            nextZs.reserve(bridgeOuterRadius - bridgeInnerGap + 1);

            const int prevBegin = std::max(0, i - bridgeOuterRadius);
            const int prevEnd = std::max(0, i - bridgeInnerGap);
            const int nextBegin = std::min(static_cast<int>(finitePoints.size()) - 1, i + bridgeInnerGap);
            const int nextEnd = std::min(static_cast<int>(finitePoints.size()) - 1, i + bridgeOuterRadius);
            for (int j = prevBegin; j <= prevEnd; ++j)
            {
                if (!keep[j])
                {
                    continue;
                }
                prevXs.push_back(finitePoints[j].x);
                prevYs.push_back(finitePoints[j].y);
                prevZs.push_back(finitePoints[j].z);
            }
            for (int j = nextBegin; j <= nextEnd; ++j)
            {
                if (!keep[j])
                {
                    continue;
                }
                nextXs.push_back(finitePoints[j].x);
                nextYs.push_back(finitePoints[j].y);
                nextZs.push_back(finitePoints[j].z);
            }

            if (prevXs.size() < static_cast<size_t>(bridgeMinNeighborCount)
                || nextXs.size() < static_cast<size_t>(bridgeMinNeighborCount))
            {
                continue;
            }

            cv::Point3d prevMedian;
            prevMedian.x = medianValue(prevXs);
            prevMedian.y = medianValue(prevYs);
            prevMedian.z = medianValue(prevZs);

            cv::Point3d nextMedian;
            nextMedian.x = medianValue(nextXs);
            nextMedian.y = medianValue(nextYs);
            nextMedian.z = medianValue(nextZs);

            const double bridgeLength = distance3D(prevMedian, nextMedian);
            const double bridgeDistance = pointSegmentDistance3D(finitePoints[i], prevMedian, nextMedian);
            if (bridgeLength <= bridgeMaxLength && bridgeDistance > bridgeDistanceThreshold)
            {
                keep[i] = 0;
            }
        }
    }

    if (options.enableDeletedSequenceBridgeRestore)
    {
        const int restoreMinPointCount = std::max(2, options.deletedBridgeMinPointCount);
        const int restoreMaxIndexGap = std::max(1, options.deletedBridgeMaxIndexGap);
        const double restoreEndpointGap = std::max(options.deletedBridgeEndpointGapMinMm,
                                                  medianStep * options.deletedBridgeEndpointGapStepScale);

        const auto runSpan = [&](int begin, int end) -> double
        {
            double minX = std::numeric_limits<double>::max();
            double maxX = -std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();
            double maxY = -std::numeric_limits<double>::max();
            double minZ = std::numeric_limits<double>::max();
            double maxZ = -std::numeric_limits<double>::max();
            for (int i = begin; i <= end; ++i)
            {
                const cv::Point3d& point = finitePoints[i];
                minX = std::min(minX, point.x);
                maxX = std::max(maxX, point.x);
                minY = std::min(minY, point.y);
                maxY = std::max(maxY, point.y);
                minZ = std::min(minZ, point.z);
                maxZ = std::max(maxZ, point.z);
            }

            const double spanXY = std::max(maxX - minX, maxY - minY);
            return std::max(spanXY, maxZ - minZ);
        };

        const auto runLineResidualAverage = [&](int begin, int end, double* maxResidual) -> double
        {
            if (maxResidual != nullptr)
            {
                *maxResidual = std::numeric_limits<double>::max();
            }
            if (end <= begin || distance3D(finitePoints[begin], finitePoints[end]) <= 1.0e-9)
            {
                return std::numeric_limits<double>::max();
            }

            double residualSum = 0.0;
            double residualMax = 0.0;
            for (int i = begin; i <= end; ++i)
            {
                const double residual = pointSegmentDistance3D(finitePoints[i], finitePoints[begin], finitePoints[end]);
                residualSum += residual;
                residualMax = std::max(residualMax, residual);
            }

            if (maxResidual != nullptr)
            {
                *maxResidual = residualMax;
            }
            return residualSum / static_cast<double>(end - begin + 1);
        };

        const auto findPreviousKept = [&](int begin) -> int
        {
            const int searchBegin = std::max(0, begin - restoreMaxIndexGap);
            for (int i = begin - 1; i >= searchBegin; --i)
            {
                if (keep[i])
                {
                    return i;
                }
            }
            return -1;
        };

        const auto findNextKept = [&](int end) -> int
        {
            const int searchEnd = std::min(static_cast<int>(finitePoints.size()) - 1, end + restoreMaxIndexGap);
            for (int i = end + 1; i <= searchEnd; ++i)
            {
                if (keep[i])
                {
                    return i;
                }
            }
            return -1;
        };

        int runBegin = 0;
        while (runBegin < static_cast<int>(finitePoints.size()))
        {
            if (keep[runBegin])
            {
                ++runBegin;
                continue;
            }

            int runEnd = runBegin;
            while (runEnd + 1 < static_cast<int>(finitePoints.size()) && !keep[runEnd + 1])
            {
                // Large physical gaps usually mean two unrelated noise islands; split them.
                if (distance3D(finitePoints[runEnd], finitePoints[runEnd + 1]) > restoreEndpointGap)
                {
                    break;
                }
                ++runEnd;
            }

            const int runCount = runEnd - runBegin + 1;
            if (runCount >= restoreMinPointCount && runSpan(runBegin, runEnd) >= options.deletedBridgeMinSpanMm)
            {
                const int previousKept = findPreviousKept(runBegin);
                const int nextKept = findNextKept(runEnd);
                if (previousKept >= 0 && nextKept >= 0
                    && distance3D(finitePoints[previousKept], finitePoints[runBegin]) <= restoreEndpointGap
                    && distance3D(finitePoints[runEnd], finitePoints[nextKept]) <= restoreEndpointGap)
                {
                    double maxLineResidual = std::numeric_limits<double>::max();
                    const double averageLineResidual = runLineResidualAverage(runBegin, runEnd, &maxLineResidual);
                    const bool isShortLineBetweenKeptSegments =
                        averageLineResidual <= options.deletedBridgeMaxAverageLineResidualMm
                        && maxLineResidual <= options.deletedBridgeMaxLineResidualMm;
                    if (isShortLineBetweenKeptSegments)
                    {
                        for (int i = runBegin; i <= runEnd; ++i)
                        {
                            keep[i] = 1;
                        }
                    }
                }
            }

            runBegin = runEnd + 1;
        }
    }

    if (options.enableSmallClusterFilter)
    {
        std::vector<int> keptIndices;
        keptIndices.reserve(finitePoints.size());
        for (int i = 0; i < static_cast<int>(finitePoints.size()); ++i)
        {
            if (keep[i])
            {
                keptIndices.push_back(i);
            }
        }

        if (keptIndices.size() >= static_cast<size_t>(minPointCount))
        {
            struct ClusterStat
            {
                int count = 0;
                int minPointIndex = std::numeric_limits<int>::max();
                int maxPointIndex = -1;
                double minX = std::numeric_limits<double>::max();
                double maxX = -std::numeric_limits<double>::max();
                double minY = std::numeric_limits<double>::max();
                double maxY = -std::numeric_limits<double>::max();
                double minZ = std::numeric_limits<double>::max();
                double maxZ = -std::numeric_limits<double>::max();
                std::vector<int> localIndices;
            };

            const double clusterLinkRadius = std::max(options.clusterLinkRadiusMinMm,
                                                      medianStep * options.clusterLinkRadiusStepScale);
            std::vector<int> clusterOfPoint(keptIndices.size(), -1);
            std::vector<ClusterStat> clusters;
            for (int seed = 0; seed < static_cast<int>(keptIndices.size()); ++seed)
            {
                if (clusterOfPoint[seed] >= 0)
                {
                    continue;
                }

                const int clusterIndex = static_cast<int>(clusters.size());
                clusters.push_back(ClusterStat());
                std::vector<int> stack;
                stack.push_back(seed);
                clusterOfPoint[seed] = clusterIndex;

                while (!stack.empty())
                {
                    const int current = stack.back();
                    stack.pop_back();
                    const cv::Point3d& currentPoint = finitePoints[keptIndices[current]];
                    ClusterStat& cluster = clusters[clusterIndex];
                    ++cluster.count;
                    cluster.minPointIndex = std::min(cluster.minPointIndex, keptIndices[current]);
                    cluster.maxPointIndex = std::max(cluster.maxPointIndex, keptIndices[current]);
                    cluster.minX = std::min(cluster.minX, currentPoint.x);
                    cluster.maxX = std::max(cluster.maxX, currentPoint.x);
                    cluster.minY = std::min(cluster.minY, currentPoint.y);
                    cluster.maxY = std::max(cluster.maxY, currentPoint.y);
                    cluster.minZ = std::min(cluster.minZ, currentPoint.z);
                    cluster.maxZ = std::max(cluster.maxZ, currentPoint.z);
                    cluster.localIndices.push_back(current);

                    for (int candidate = 0; candidate < static_cast<int>(keptIndices.size()); ++candidate)
                    {
                        if (clusterOfPoint[candidate] >= 0)
                        {
                            continue;
                        }
                        if (distance3D(currentPoint, finitePoints[keptIndices[candidate]]) <= clusterLinkRadius)
                        {
                            clusterOfPoint[candidate] = clusterIndex;
                            stack.push_back(candidate);
                        }
                    }
                }
            }

            int largestClusterCount = 0;
            for (const ClusterStat& cluster : clusters)
            {
                largestClusterCount = std::max(largestClusterCount, cluster.count);
            }

            const int minClusterCount = std::max(1, options.clusterMinPointCount);
            const int ratioClusterCount = std::max(1, static_cast<int>(
                std::ceil(largestClusterCount * std::max(0.0, std::min(1.0, options.clusterMinSizeRatioToLargest)))));
            const int spanMinPointCount = std::max(3, minClusterCount / 2);
            const auto clusterSpan = [](const ClusterStat& cluster) -> double
            {
                const double spanXY = std::max(cluster.maxX - cluster.minX, cluster.maxY - cluster.minY);
                return std::max(spanXY, cluster.maxZ - cluster.minZ);
            };

            const auto clusterLineResidualAverage = [&](const ClusterStat& cluster,
                                                        double* maxResidual) -> double
            {
                if (maxResidual != nullptr)
                {
                    *maxResidual = std::numeric_limits<double>::max();
                }
                if (cluster.localIndices.size() < 2)
                {
                    return std::numeric_limits<double>::max();
                }

                int startLocal = cluster.localIndices.front();
                int endLocal = cluster.localIndices.front();
                for (int localIndex : cluster.localIndices)
                {
                    if (keptIndices[localIndex] < keptIndices[startLocal])
                    {
                        startLocal = localIndex;
                    }
                    if (keptIndices[localIndex] > keptIndices[endLocal])
                    {
                        endLocal = localIndex;
                    }
                }

                const cv::Point3d& startPoint = finitePoints[keptIndices[startLocal]];
                const cv::Point3d& endPoint = finitePoints[keptIndices[endLocal]];
                if (distance3D(startPoint, endPoint) <= 1.0e-9)
                {
                    return std::numeric_limits<double>::max();
                }

                double residualSum = 0.0;
                double residualMax = 0.0;
                for (int localIndex : cluster.localIndices)
                {
                    const double residual = pointSegmentDistance3D(finitePoints[keptIndices[localIndex]],
                                                                    startPoint,
                                                                    endPoint);
                    residualSum += residual;
                    residualMax = std::max(residualMax, residual);
                }

                if (maxResidual != nullptr)
                {
                    *maxResidual = residualMax;
                }
                return residualSum / static_cast<double>(cluster.localIndices.size());
            };

            const int bridgeMaxIndexGap = std::max(1, options.clusterBridgeMaxIndexGap);
            const double clusterContinuityEndpointGap = std::max(options.clusterContinuityEndpointGapMinMm,
                                                                 medianStep * options.clusterContinuityEndpointGapStepScale);
            const auto endpointGapToKeptCluster = [&](int clusterIndex,
                                                      int otherIndex) -> double
            {
                const ClusterStat& current = clusters[clusterIndex];
                const ClusterStat& other = clusters[otherIndex];
                if (other.maxPointIndex < current.minPointIndex)
                {
                    return distance3D(finitePoints[other.maxPointIndex],
                                      finitePoints[current.minPointIndex]);
                }
                if (other.minPointIndex > current.maxPointIndex)
                {
                    return distance3D(finitePoints[current.maxPointIndex],
                                      finitePoints[other.minPointIndex]);
                }
                return 0.0;
            };

            const auto hasKeptClusterNeighbor = [&](int clusterIndex,
                                                    const std::vector<unsigned char>& keepCluster) -> bool
            {
                const ClusterStat& current = clusters[clusterIndex];
                int prevGap = std::numeric_limits<int>::max();
                int nextGap = std::numeric_limits<int>::max();
                double prevEndpointGap = std::numeric_limits<double>::max();
                double nextEndpointGap = std::numeric_limits<double>::max();
                for (int otherIndex = 0; otherIndex < static_cast<int>(clusters.size()); ++otherIndex)
                {
                    if (otherIndex == clusterIndex || !keepCluster[otherIndex])
                    {
                        continue;
                    }

                    const ClusterStat& other = clusters[otherIndex];
                    if (other.maxPointIndex < current.minPointIndex)
                    {
                        const int candidateGap = current.minPointIndex - other.maxPointIndex;
                        const double candidateEndpointGap = endpointGapToKeptCluster(clusterIndex, otherIndex);
                        if (candidateGap < prevGap)
                        {
                            prevGap = candidateGap;
                            prevEndpointGap = candidateEndpointGap;
                        }
                    }
                    if (other.minPointIndex > current.maxPointIndex)
                    {
                        const int candidateGap = other.minPointIndex - current.maxPointIndex;
                        const double candidateEndpointGap = endpointGapToKeptCluster(clusterIndex, otherIndex);
                        if (candidateGap < nextGap)
                        {
                            nextGap = candidateGap;
                            nextEndpointGap = candidateEndpointGap;
                        }
                    }
                }

                const bool hasPrev = prevGap <= bridgeMaxIndexGap
                    && prevEndpointGap <= clusterContinuityEndpointGap;
                const bool hasNext = nextGap <= bridgeMaxIndexGap
                    && nextEndpointGap <= clusterContinuityEndpointGap;
                return hasPrev || hasNext;
            };

            const auto hasKeptClusterOnBothSides = [&](int clusterIndex,
                                                       const std::vector<unsigned char>& keepCluster) -> bool
            {
                const ClusterStat& current = clusters[clusterIndex];
                int prevGap = std::numeric_limits<int>::max();
                int nextGap = std::numeric_limits<int>::max();
                double prevEndpointGap = std::numeric_limits<double>::max();
                double nextEndpointGap = std::numeric_limits<double>::max();
                for (int otherIndex = 0; otherIndex < static_cast<int>(clusters.size()); ++otherIndex)
                {
                    if (otherIndex == clusterIndex || !keepCluster[otherIndex])
                    {
                        continue;
                    }

                    const ClusterStat& other = clusters[otherIndex];
                    if (other.maxPointIndex < current.minPointIndex)
                    {
                        const int candidateGap = current.minPointIndex - other.maxPointIndex;
                        const double candidateEndpointGap = endpointGapToKeptCluster(clusterIndex, otherIndex);
                        if (candidateGap < prevGap)
                        {
                            prevGap = candidateGap;
                            prevEndpointGap = candidateEndpointGap;
                        }
                    }
                    if (other.minPointIndex > current.maxPointIndex)
                    {
                        const int candidateGap = other.minPointIndex - current.maxPointIndex;
                        const double candidateEndpointGap = endpointGapToKeptCluster(clusterIndex, otherIndex);
                        if (candidateGap < nextGap)
                        {
                            nextGap = candidateGap;
                            nextEndpointGap = candidateEndpointGap;
                        }
                    }
                }

                return prevGap <= bridgeMaxIndexGap
                    && nextGap <= bridgeMaxIndexGap
                    && prevEndpointGap <= clusterContinuityEndpointGap
                    && nextEndpointGap <= clusterContinuityEndpointGap;
            };

            std::vector<unsigned char> keepCluster(clusters.size(), 0);
            for (int clusterIndex = 0; clusterIndex < static_cast<int>(clusters.size()); ++clusterIndex)
            {
                if (clusters[clusterIndex].count == largestClusterCount)
                {
                    keepCluster[clusterIndex] = 1;
                }
            }

            bool addedConnectedCluster = true;
            while (addedConnectedCluster)
            {
                addedConnectedCluster = false;
                for (int clusterIndex = 0; clusterIndex < static_cast<int>(clusters.size()); ++clusterIndex)
                {
                    if (keepCluster[clusterIndex])
                    {
                        continue;
                    }

                    const ClusterStat& cluster = clusters[clusterIndex];
                    const double span = clusterSpan(cluster);
                    const bool isValidSecondaryCluster = cluster.count >= minClusterCount
                        && cluster.count >= ratioClusterCount
                        && cluster.count >= spanMinPointCount
                        && span >= options.clusterMinSpanMm
                        && hasKeptClusterNeighbor(clusterIndex, keepCluster);
                    // Secondary clusters must be physically connected to an already kept cluster.
                    // This prevents isolated laser ghosts from surviving only because their point
                    // indices happen to sit between two real laser-line sections.
                    if (isValidSecondaryCluster)
                    {
                        keepCluster[clusterIndex] = 1;
                        addedConnectedCluster = true;
                    }
                }
            }

            if (options.enableSequenceBridgeClusterPreserve)
            {
                const int bridgeMinPointCount = std::max(2, options.clusterBridgeMinPointCount);
                bool changed = true;
                while (changed)
                {
                    changed = false;
                    for (int clusterIndex = 0; clusterIndex < static_cast<int>(clusters.size()); ++clusterIndex)
                    {
                        if (keepCluster[clusterIndex])
                        {
                            continue;
                        }

                        const ClusterStat& cluster = clusters[clusterIndex];
                        double maxLineResidual = std::numeric_limits<double>::max();
                        const double averageLineResidual = clusterLineResidualAverage(cluster, &maxLineResidual);
                        const bool isSequenceBridgeCluster = cluster.count >= bridgeMinPointCount
                            && clusterSpan(cluster) >= options.clusterBridgeMinSpanMm
                            && averageLineResidual <= options.clusterBridgeMaxAverageLineResidualMm
                            && maxLineResidual <= options.clusterBridgeMaxLineResidualMm
                            && hasKeptClusterOnBothSides(clusterIndex, keepCluster);
                        if (isSequenceBridgeCluster)
                        {
                            keepCluster[clusterIndex] = 1;
                            changed = true;
                        }
                    }
                }
            }

            for (int localIndex = 0; localIndex < static_cast<int>(keptIndices.size()); ++localIndex)
            {
                if (!keepCluster[clusterOfPoint[localIndex]])
                {
                    keep[keptIndices[localIndex]] = 0;
                }
            }
        }
    }

    std::vector<cv::Point3d> filteredPoints;
    filteredPoints.reserve(finitePoints.size());
    for (int i = 0; i < static_cast<int>(finitePoints.size()); ++i)
    {
        if (keep[i])
        {
            filteredPoints.push_back(finitePoints[i]);
        }
    }

    if (options.enableProfileConnectedComponentFilter
        && !options.enableDominantLineSegmentFilter
        && filteredPoints.size() >= static_cast<std::size_t>(minPointCount))
    {
        std::vector<double> profileStepDistances;
        profileStepDistances.reserve(filteredPoints.size() - 1);
        for (int i = 1; i < static_cast<int>(filteredPoints.size()); ++i)
        {
            const double step = distanceProfile2D(filteredPoints[i - 1], filteredPoints[i]);
            if (step > 1.0e-9)
            {
                profileStepDistances.push_back(step);
            }
        }

        const double profileMedianStep = profileStepDistances.empty()
            ? medianStep
            : medianValue(profileStepDistances);
        const double profileLinkRadius = std::max(options.profileComponentLinkRadiusMinMm,
                                                  profileMedianStep * options.profileComponentLinkRadiusStepScale);
        if (profileLinkRadius > 1.0e-9)
        {
            struct ProfileClusterStat
            {
                int count = 0;
                double minA = std::numeric_limits<double>::max();
                double maxA = -std::numeric_limits<double>::max();
                double minB = std::numeric_limits<double>::max();
                double maxB = -std::numeric_limits<double>::max();
            };

            std::vector<int> componentOfPoint(filteredPoints.size(), -1);
            std::vector<ProfileClusterStat> profileClusters;
            std::vector<int> queue;
            queue.reserve(filteredPoints.size());

            for (int seed = 0; seed < static_cast<int>(filteredPoints.size()); ++seed)
            {
                if (componentOfPoint[seed] >= 0)
                {
                    continue;
                }

                const int componentIndex = static_cast<int>(profileClusters.size());
                profileClusters.push_back(ProfileClusterStat());
                componentOfPoint[seed] = componentIndex;
                queue.clear();
                queue.push_back(seed);

                for (std::size_t head = 0; head < queue.size(); ++head)
                {
                    const int current = queue[head];
                    ProfileClusterStat& stat = profileClusters[componentIndex];
                    const cv::Point3d& currentPoint = filteredPoints[current];
                    const double a = profileCoordinate(currentPoint, profileAxis0);
                    const double b = profileCoordinate(currentPoint, profileAxis1);
                    ++stat.count;
                    stat.minA = std::min(stat.minA, a);
                    stat.maxA = std::max(stat.maxA, a);
                    stat.minB = std::min(stat.minB, b);
                    stat.maxB = std::max(stat.maxB, b);

                    for (int candidate = 0; candidate < static_cast<int>(filteredPoints.size()); ++candidate)
                    {
                        if (componentOfPoint[candidate] >= 0)
                        {
                            continue;
                        }

                        if (distanceProfile2D(currentPoint, filteredPoints[candidate]) <= profileLinkRadius)
                        {
                            componentOfPoint[candidate] = componentIndex;
                            queue.push_back(candidate);
                        }
                    }
                }
            }

            int largestComponentIndex = -1;
            int largestComponentCount = 0;
            for (int componentIndex = 0; componentIndex < static_cast<int>(profileClusters.size()); ++componentIndex)
            {
                if (profileClusters[componentIndex].count > largestComponentCount)
                {
                    largestComponentCount = profileClusters[componentIndex].count;
                    largestComponentIndex = componentIndex;
                }
            }

            if (largestComponentIndex >= 0)
            {
                const double ratio = std::max(
                    0.0,
                    std::min(1.0, options.profileComponentStandaloneMinSizeRatioToLargest));
                const int ratioMinCount = static_cast<int>(std::ceil(largestComponentCount * ratio));
                const int standaloneMinCount = std::max(options.profileComponentStandaloneMinPointCount,
                                                        ratioMinCount);

                std::vector<unsigned char> keepProfileCluster(profileClusters.size(), 0);
                keepProfileCluster[largestComponentIndex] = 1;
                for (int componentIndex = 0; componentIndex < static_cast<int>(profileClusters.size()); ++componentIndex)
                {
                    if (componentIndex == largestComponentIndex)
                    {
                        continue;
                    }

                    const ProfileClusterStat& cluster = profileClusters[componentIndex];
                    const double span = std::max(cluster.maxA - cluster.minA, cluster.maxB - cluster.minB);
                    if (options.profileComponentKeepStandalone
                        && cluster.count >= standaloneMinCount
                        && span >= options.profileComponentStandaloneMinSpanMm)
                    {
                        keepProfileCluster[componentIndex] = 1;
                    }
                }

                std::vector<cv::Point3d> profileFilteredPoints;
                profileFilteredPoints.reserve(filteredPoints.size());
                for (int i = 0; i < static_cast<int>(filteredPoints.size()); ++i)
                {
                    if (keepProfileCluster[componentOfPoint[i]])
                    {
                        profileFilteredPoints.push_back(filteredPoints[i]);
                    }
                }
                filteredPoints.swap(profileFilteredPoints);
            }
        }
    }

    if (options.enableProfileDominantChainFilter
        && !options.enableDominantLineSegmentFilter
        && filteredPoints.size() >= static_cast<std::size_t>(minPointCount))
    {
        std::vector<double> profileStepDistances;
        profileStepDistances.reserve(filteredPoints.size() - 1);
        for (int i = 1; i < static_cast<int>(filteredPoints.size()); ++i)
        {
            const double step = distanceProfile2D(filteredPoints[i - 1], filteredPoints[i]);
            if (step > 1.0e-9)
            {
                profileStepDistances.push_back(step);
            }
        }

        const double profileMedianStep = profileStepDistances.empty()
            ? medianStep
            : medianValue(profileStepDistances);
        const double splitGap = std::max(options.profileRunSplitGapMinMm,
                                         profileMedianStep * options.profileRunSplitGapStepScale);
        const double chainGap = std::max(options.profileRunChainEndpointGapMinMm,
                                         profileMedianStep * options.profileRunChainEndpointGapStepScale);

        if (splitGap > 1.0e-9 && chainGap > 1.0e-9)
        {
            struct ProfileRun
            {
                int begin = 0;
                int end = 0;
                int count = 0;
                double minA = std::numeric_limits<double>::max();
                double maxA = -std::numeric_limits<double>::max();
                double minB = std::numeric_limits<double>::max();
                double maxB = -std::numeric_limits<double>::max();
            };

            const auto makeRun = [&](const int begin, const int end) -> ProfileRun
            {
                ProfileRun run;
                run.begin = begin;
                run.end = end;
                run.count = end - begin + 1;
                for (int i = begin; i <= end; ++i)
                {
                    const cv::Point3d& point = filteredPoints[i];
                    const double a = profileCoordinate(point, profileAxis0);
                    const double b = profileCoordinate(point, profileAxis1);
                    run.minA = std::min(run.minA, a);
                    run.maxA = std::max(run.maxA, a);
                    run.minB = std::min(run.minB, b);
                    run.maxB = std::max(run.maxB, b);
                }
                return run;
            };

            std::vector<ProfileRun> runs;
            int runBegin = 0;
            for (int i = 1; i < static_cast<int>(filteredPoints.size()); ++i)
            {
                if (distanceProfile2D(filteredPoints[i - 1], filteredPoints[i]) > splitGap)
                {
                    runs.push_back(makeRun(runBegin, i - 1));
                    runBegin = i;
                }
            }
            runs.push_back(makeRun(runBegin, static_cast<int>(filteredPoints.size()) - 1));

            if (runs.size() > 1)
            {
                struct RunComponentStat
                {
                    int count = 0;
                    double minA = std::numeric_limits<double>::max();
                    double maxA = -std::numeric_limits<double>::max();
                    double minB = std::numeric_limits<double>::max();
                    double maxB = -std::numeric_limits<double>::max();
                };

                const auto runEndpointDistance = [&](const ProfileRun& a, const ProfileRun& b) -> double
                {
                    const cv::Point3d& aBegin = filteredPoints[a.begin];
                    const cv::Point3d& aEnd = filteredPoints[a.end];
                    const cv::Point3d& bBegin = filteredPoints[b.begin];
                    const cv::Point3d& bEnd = filteredPoints[b.end];
                    return std::min(
                        std::min(distanceProfile2D(aBegin, bBegin), distanceProfile2D(aBegin, bEnd)),
                        std::min(distanceProfile2D(aEnd, bBegin), distanceProfile2D(aEnd, bEnd)));
                };

                std::vector<int> componentOfRun(runs.size(), -1);
                std::vector<RunComponentStat> components;
                std::vector<int> queue;
                queue.reserve(runs.size());

                for (int seed = 0; seed < static_cast<int>(runs.size()); ++seed)
                {
                    if (componentOfRun[seed] >= 0)
                    {
                        continue;
                    }

                    const int componentIndex = static_cast<int>(components.size());
                    components.push_back(RunComponentStat());
                    componentOfRun[seed] = componentIndex;
                    queue.clear();
                    queue.push_back(seed);

                    for (std::size_t head = 0; head < queue.size(); ++head)
                    {
                        const int current = queue[head];
                        RunComponentStat& stat = components[componentIndex];
                        const ProfileRun& currentRun = runs[current];
                        stat.count += currentRun.count;
                        stat.minA = std::min(stat.minA, currentRun.minA);
                        stat.maxA = std::max(stat.maxA, currentRun.maxA);
                        stat.minB = std::min(stat.minB, currentRun.minB);
                        stat.maxB = std::max(stat.maxB, currentRun.maxB);

                        for (int candidate = 0; candidate < static_cast<int>(runs.size()); ++candidate)
                        {
                            if (componentOfRun[candidate] >= 0)
                            {
                                continue;
                            }

                            if (runEndpointDistance(currentRun, runs[candidate]) <= chainGap)
                            {
                                componentOfRun[candidate] = componentIndex;
                                queue.push_back(candidate);
                            }
                        }
                    }
                }

                int largestComponentIndex = -1;
                int largestComponentCount = 0;
                for (int componentIndex = 0; componentIndex < static_cast<int>(components.size()); ++componentIndex)
                {
                    if (components[componentIndex].count > largestComponentCount)
                    {
                        largestComponentCount = components[componentIndex].count;
                        largestComponentIndex = componentIndex;
                    }
                }

                if (largestComponentIndex >= 0)
                {
                    std::vector<unsigned char> keepRunComponent(components.size(), 0);
                    keepRunComponent[largestComponentIndex] = 1;

                    for (int componentIndex = 0; componentIndex < static_cast<int>(components.size()); ++componentIndex)
                    {
                        if (componentIndex == largestComponentIndex)
                        {
                            continue;
                        }

                        const RunComponentStat& component = components[componentIndex];
                        const double span = std::max(component.maxA - component.minA,
                                                     component.maxB - component.minB);
                        if (options.profileRunKeepStandalone
                            && component.count >= options.profileRunStandaloneMinPointCount
                            && span >= options.profileRunStandaloneMinSpanMm)
                        {
                            keepRunComponent[componentIndex] = 1;
                        }
                    }

                    std::vector<cv::Point3d> chainFilteredPoints;
                    chainFilteredPoints.reserve(filteredPoints.size());
                    for (int runIndex = 0; runIndex < static_cast<int>(runs.size()); ++runIndex)
                    {
                        if (!keepRunComponent[componentOfRun[runIndex]])
                        {
                            continue;
                        }

                        const ProfileRun& run = runs[runIndex];
                        for (int pointIndex = run.begin; pointIndex <= run.end; ++pointIndex)
                        {
                            chainFilteredPoints.push_back(filteredPoints[pointIndex]);
                        }
                    }
                    filteredPoints.swap(chainFilteredPoints);
                }
            }
        }
    }

    if (options.enableDominantLineSegmentFilter)
    {
        const std::vector<cv::Point3d> dominantFilteredPoints = extractDominantLineSegments(filteredPoints);
        if (dominantFilteredPoints.size() < filteredPoints.size())
        {
            return dominantFilteredPoints;
        }
    }

    return filteredPoints;
}
