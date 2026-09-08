#pragma once

#include "ConfigDatabase.h"
#include "RobotDriverRegistry.h"

#include <QString>

namespace MeasureThenWeldRuntimeConfig
{
	enum class ScanTimestampSource
	{
		Robot,
		Pc
	};

	enum class StepSdkInterfaceMode
	{
		Timestamp,
		Legacy
	};

	inline QString SettingsGroup()
	{
		return QStringLiteral("MeasureThenWeld/Runtime");
	}

	inline QString TimestampSourceKey()
	{
		return QStringLiteral("ScanTimestampSource");
	}

	inline QString StepSdkInterfaceModeKey()
	{
		return QStringLiteral("StepSdkInterfaceMode");
	}

	inline QString ToStorageString(ScanTimestampSource source)
	{
		return source == ScanTimestampSource::Pc
			? QStringLiteral("pc")
			: QStringLiteral("robot");
	}

	inline QString ToStorageString(StepSdkInterfaceMode mode)
	{
		return mode == StepSdkInterfaceMode::Legacy
			? QStringLiteral("legacy")
			: QStringLiteral("timestamp");
	}

	inline ScanTimestampSource FromStorageString(const QString& value)
	{
		const QString normalized = value.trimmed().toLower();
		return normalized == QStringLiteral("pc")
			|| normalized == QStringLiteral("pc_recv")
			|| normalized == QStringLiteral("pcrecv")
			|| normalized == QStringLiteral("pc_recv_ms")
			? ScanTimestampSource::Pc
			: ScanTimestampSource::Robot;
	}

	inline StepSdkInterfaceMode StepSdkInterfaceModeFromStorageString(const QString& value)
	{
		const QString normalized = value.trimmed().toLower();
		return normalized == QStringLiteral("legacy")
			|| normalized == QStringLiteral("old")
			|| normalized == QStringLiteral("classic")
			? StepSdkInterfaceMode::Legacy
			: StepSdkInterfaceMode::Timestamp;
	}

	inline QString DisplayName(ScanTimestampSource source)
	{
		return source == ScanTimestampSource::Pc
			? QStringLiteral("PC接收时间")
			: QStringLiteral("机器人时间戳");
	}

	inline QString DisplayName(StepSdkInterfaceMode mode)
	{
		return mode == StepSdkInterfaceMode::Legacy
			? QStringLiteral("旧版SDK接口")
			: QStringLiteral("新版时间戳接口");
	}

	inline QString FieldName(ScanTimestampSource source)
	{
		return source == ScanTimestampSource::Pc
			? QStringLiteral("pc_recv_ms")
			: QStringLiteral("robot_ms");
	}

	inline QString RobotSettingsGroup()
	{
		return QStringLiteral("RobotPara/BaseParam");
	}

	// The old global scope is only a read migration source. New writes are
	// always scoped to the selected robot, never inherited by another unit.
	inline ConfigDatabase::ReadStatus ReadRobotRuntimeValue(
		const QString& robotName, const QString& key, QString* value)
	{
		if (robotName.trimmed().isEmpty()) { return ConfigDatabase::ReadStatus::Error; }
		const auto status = ConfigDatabase::ReadScopedSettingStatus(
			QStringLiteral("robot"), robotName, RobotSettingsGroup(), key, value);
		if (status != ConfigDatabase::ReadStatus::NotFound) { return status; }
		return ConfigDatabase::ReadScopedSettingStatus(
			QStringLiteral("global"), QString(), SettingsGroup(), key, value);
	}

	inline const RobotDriverSetupProfile* ConfiguredRobotProfile(const QString& robotName)
	{
		QString value;
		if (robotName.trimmed().isEmpty() || !ConfigDatabase::ReadScopedSetting(
			"robot", robotName, RobotSettingsGroup(), "RobotType", &value)) { return nullptr; }
		bool ok = false;
		const int type = value.toInt(&ok);
		return ok ? RobotDriverRegistry::SetupProfile(type) : nullptr;
	}

	inline ScanTimestampSource EffectiveScanTimestampSource(ScanTimestampSource configured, bool nativeTimestampAvailable)
	{
		return nativeTimestampAvailable ? configured : ScanTimestampSource::Pc;
	}

	inline ScanTimestampSource LoadConfiguredScanTimestampSource(const QString& robotName)
	{
		QString value;
		const auto status = ReadRobotRuntimeValue(robotName, TimestampSourceKey(), &value);
		if (status == ConfigDatabase::ReadStatus::Error) { return ScanTimestampSource::Pc; }
		return status == ConfigDatabase::ReadStatus::Found ? FromStorageString(value) : ScanTimestampSource::Robot;
	}

	inline StepSdkInterfaceMode LoadStepSdkInterfaceMode(const QString& robotName)
	{
		QString value;
		const auto status = ReadRobotRuntimeValue(robotName, StepSdkInterfaceModeKey(), &value);
		if (status == ConfigDatabase::ReadStatus::Error) { return StepSdkInterfaceMode::Legacy; }
		return status == ConfigDatabase::ReadStatus::Found
			? StepSdkInterfaceModeFromStorageString(value) : StepSdkInterfaceMode::Timestamp;
	}

	inline ScanTimestampSource LoadScanTimestampSource(const QString& robotName)
	{
		const auto* profile = ConfiguredRobotProfile(robotName);
		const bool nativeAvailable = profile != nullptr && profile->supportsRobotTimestamp
			&& (!profile->usesStepTimestampInterface || LoadStepSdkInterfaceMode(robotName) == StepSdkInterfaceMode::Timestamp);
		return EffectiveScanTimestampSource(LoadConfiguredScanTimestampSource(robotName), nativeAvailable);
	}

	inline bool SaveScanTimestampSource(const QString& robotName, ScanTimestampSource source)
	{
		if (robotName.trimmed().isEmpty()) { return false; }
		return ConfigDatabase::WriteScopedSetting("robot", robotName, RobotSettingsGroup(), TimestampSourceKey(), ToStorageString(source));
	}

	inline bool SaveStepSdkInterfaceMode(const QString& robotName, StepSdkInterfaceMode mode)
	{
		if (robotName.trimmed().isEmpty()) { return false; }
		return ConfigDatabase::WriteScopedSetting("robot", robotName, RobotSettingsGroup(), StepSdkInterfaceModeKey(), ToStorageString(mode));
	}

	inline QString SkipFlowConfirmsKey()
	{
		return QStringLiteral("SkipFlowConfirms");
	}

	inline QString SkjFrameBufferCountKey()
	{
		return QStringLiteral("SkjFrameBufferCount");
	}

	inline QString CameraLinePointMirrorZKey()
	{
		return QStringLiteral("CameraLinePointMirrorZ");
	}

	// SKJ 相机客户端接收 FIFO 深度（v1.2.0 SetFrameBufferCount，2~16，SDK 默认 2，
	// 本程序默认 8）。全局生效，界面改动即存即用。
	inline int ClampSkjFrameBufferCount(int count)
	{
		return count < 2 ? 2 : (count > 16 ? 16 : count);
	}

	inline int LoadSkjFrameBufferCount()
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), SkjFrameBufferCountKey(), &value))
		{
			return 8;
		}
		bool ok = false;
		const int count = value.trimmed().toInt(&ok);
		return ok ? ClampSkjFrameBufferCount(count) : 8;
	}

	inline void SaveSkjFrameBufferCount(int count)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), SkjFrameBufferCountKey(),
			QString::number(ClampSkjFrameBufferCount(count)));
	}

	// 完整点云进入手眼变换前的显式诊断/兼容开关。默认关闭，保持相机底层已经
	// 规范化的 TargetDeviceXYZ；开启时仅对生产 cameraLinePoint.z 取反。
	// 预览窗口有独立的显示开关；两个开关都不修改缓存原始帧。
	inline bool LoadCameraLinePointMirrorZ()
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(
			QStringLiteral("global"), QString(), SettingsGroup(), CameraLinePointMirrorZKey(), &value))
		{
			return false;
		}
		return value.trimmed() == QStringLiteral("1");
	}

	inline bool SaveCameraLinePointMirrorZ(bool enabled)
	{
		return ConfigDatabase::WriteScopedSetting(
			QStringLiteral("global"), QString(), SettingsGroup(), CameraLinePointMirrorZKey(),
			enabled ? QStringLiteral("1") : QStringLiteral("0"));
	}

	// 流程免确认（默认关）：跳过流程中间步骤与信息类确认弹窗；
	// 首次运动、进入焊接、翻转风险告警、历史目录核对四类确认不受此开关影响，始终弹出。
	inline bool LoadSkipFlowConfirms()
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), SkipFlowConfirmsKey(), &value))
		{
			return false;
		}
		return value.trimmed() == QStringLiteral("1");
	}

	inline void SaveSkipFlowConfirms(bool skip)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), SkipFlowConfirmsKey(),
			skip ? QStringLiteral("1") : QStringLiteral("0"));
	}
}
