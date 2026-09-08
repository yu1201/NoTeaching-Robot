#!/usr/bin/env python3
"""Read-only source guards for robot-scoped and connection-frozen time axes."""
from pathlib import Path
root = Path(__file__).resolve().parents[2]
config = (root / 'include/MeasureThenWeldRuntimeConfig.h').read_text(encoding='utf-8')
service = (root / 'src/MeasureThenWeldService.cpp').read_text(encoding='utf-8')
step = (root / 'src/StepRobotDriver.cpp').read_text(encoding='utf-8')
header = (root / 'include/STEPRobotDriver.h').read_text(encoding='utf-8')
def require(ok, why):
    if not ok:
        raise SystemExit('FAIL: ' + why)
for name in ('LoadScanTimestampSource', 'LoadStepSdkInterfaceMode'):
    require(f'{name}(const QString& robotName)' in config, f'{name} requires robot identity')
    require(f'{name}()' not in config + service + step, f'{name} has no global runtime overload')
for name in ('SaveScanTimestampSource', 'SaveStepSdkInterfaceMode'):
    body = config.split(f'inline bool {name}(', 1)[1].split('\n\t}', 1)[0]
    require('"robot", robotName, RobotSettingsGroup()' in body and '"global"' not in body,
            f'{name} writes only selected robot and reports failure')
require('if (status != ConfigDatabase::ReadStatus::NotFound) { return status; }' in config,
        'legacy global fallback only for missing scoped value, never read errors')
for token in ('LoadConfiguredScanTimestampSource(timestampRobotName)',
              'LoadScanTimestampSource(timestampRobotName)',
              'pRobotDriver->Supports(RobotDriverCapability::RobotTimestamp)',
              'EffectiveScanTimestampSource(', 'PC时间不会标为robot_ms'):
    require(token in service, 'scan freezes explicit effective source: ' + token)
require('g_stepSdkInterfaceModeCache' not in step and 'InvalidateStepSdkInterfaceModeCache' not in step + header,
        'STEP has no process-global SDK mode cache')
require('std::atomic<bool> m_stepUseTimestampSdkInterface' in header,
        'SDK mode is an instance snapshot')
reload = step.split('void STEPRobotCtrl::ReloadRuntimeConfiguration()', 1)[1].split('\nbool ', 1)[0]
require(reload.index('m_bSocketConnected.load()') < reload.index('return;') < reload.index('LoadStepSdkInterfaceModeSnapshot();'),
        'connected runtime reload does not replace sampling mode')
passive = step.split('T_ROBOT_COORS STEPRobotCtrl::GetCurrentPosPassive(', 1)[1].split('\ndouble STEPRobotCtrl::GetCurrentPulse', 1)[0]
require('LoadStepSdkInterfaceMode' not in passive and 'StepUseTimestampSdkInterface()' in passive,
        'passive polling uses frozen instance mode and performs no database lookup')
pc_fill = step.split('void StepFillPcPassiveTimestamp(', 1)[1].split('\n\tT_ROBOT_COORS', 1)[0]
require('*pRobotMs = 0;' in pc_fill and '*pPcRecvMs = pcMs;' in pc_fill,
        'legacy PC timestamp is not relabelled as robot time')
require('robotMs = pcRecvMs;' not in passive and 'robotMs = 0;' in passive,
        'new-SDK PC fallback also leaves robot_ms unavailable')
require('StepUseTimestampSdkInterface() && m_nTimestampAxisLatch.load() == 1' in step,
        'native capability requires a verified timestamp in this connection')
require(step.count('LoadStepSdkInterfaceModeSnapshot();') >= 5,
        'constructor, disconnected reload, normal and safety reconnects use per-robot snapshot')
print('PASS: robot-scoped settings, connection-frozen STEP SDK mode and truthful effective scan time axis')
