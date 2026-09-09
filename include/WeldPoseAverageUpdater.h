#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

class WeldPoseAverageUpdater
{
public:
    struct GroupReport
    {
        QString displayName;
        QString segmentKind;
        int sourceCount = 0;
        int usedCount = 0;
        int filteredCount = 0;
        double outlierThresholdDeg = 0.0;
        bool hasAverage = false;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
        int poseCompIndex = -1;
        bool addedNewSlot = false;
        bool reusedExistingSlot = false;
        bool wrotePoseToSlot = false;
        double matchedDistanceDeg = 0.0;
        QString previousName;
        QString previousSegmentKind;
        double previousRx = 0.0;
        double previousRy = 0.0;
        double previousRz = 0.0;
    };

    struct UpdateResult
    {
        QString inputPoseFilePath;
        QString robotName;
        QString poseCompStorageLabel;
        QString reportPath;
        int totalRecordCount = 0;
        int addedSlotCount = 0;
        int reusedSlotCount = 0;
        bool anyOutlierFiltered = false;
        QVector<GroupReport> groups;
        QStringList reportLines;
    };

    static bool UpdateFromInput(
        const QString& inputPathOrResultDir,
        const QString& robotNameHint,
        UpdateResult& result,
        QString& error);
};
