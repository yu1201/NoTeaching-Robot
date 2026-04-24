#pragma once

#include "MeasureThenWeldDialog.h"
#include "RobotCalculation.h"

#include <functional>
#include <vector>

class FANUCRobotCtrl;

class MeasureThenWeldService
{
public:
    using LogCallback = std::function<void(const QString&)>;
    using StepCallback = std::function<void(const QString&)>;
    using CheckpointCallback = std::function<bool(const QString&, const QString&)>;

    bool LoadPresetParam(FANUCRobotCtrl* pFanucDriver, T_PRECISE_MEASURE_PARAM& param, QString& error) const;
    bool MovePulseAndWait(FANUCRobotCtrl* pFanucDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MovePulseListAndWait(FANUCRobotCtrl* pFanucDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MoveCoorsAndWait(FANUCRobotCtrl* pFanucDriver, const T_ROBOT_COORS& coors, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool ScanMoveAndCollect(FANUCRobotCtrl* pFanucDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath, const LogCallback& appendLog, const StepCallback& setFlowStep) const;

    QString BuildResultDir(const std::string& robotName) const;
    bool SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const;
    bool ApplyWeldSeamCompToPoseFile(
        const QString& robotName,
        const QString& inputPath,
        const QString& outputPath,
        QString& summary,
        QString& error) const;
    bool DownlinkWeldPoseFile(
        FANUCRobotCtrl* pFanucDriver,
        const QString& poseFilePath,
        double linearSpeedConfigMmPerMin,
        QString& summary,
        QString& error) const;
    bool ExecuteWeldPoseFileWithSafePos(
        FANUCRobotCtrl* pFanucDriver,
        const QString& poseFilePath,
        QString& summary,
        QString& error,
        T_ROBOT_COORS* pStartSafeCoors = nullptr,
        T_ROBOT_COORS* pEndSafeCoors = nullptr,
        const LogCallback& appendLog = LogCallback(),
        const StepCallback& setFlowStep = StepCallback(),
        const CheckpointCallback& checkpoint = CheckpointCallback()) const;
    bool ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const;

private:
    static double SafeSpeed(double value, double fallback);
};
