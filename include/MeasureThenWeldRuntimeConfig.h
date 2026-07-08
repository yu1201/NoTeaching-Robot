#pragma once

#include "ConfigDatabase.h"

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

	inline ScanTimestampSource LoadScanTimestampSource()
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), TimestampSourceKey(), &value))
		{
			return ScanTimestampSource::Robot;
		}
		return FromStorageString(value);
	}

	inline StepSdkInterfaceMode LoadStepSdkInterfaceMode()
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), StepSdkInterfaceModeKey(), &value))
		{
			return StepSdkInterfaceMode::Timestamp;
		}
		return StepSdkInterfaceModeFromStorageString(value);
	}

	inline void SaveScanTimestampSource(ScanTimestampSource source)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), TimestampSourceKey(), ToStorageString(source));
	}

	inline void SaveStepSdkInterfaceMode(StepSdkInterfaceMode mode)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), StepSdkInterfaceModeKey(), ToStorageString(mode));
	}

	inline QString SkipFlowConfirmsKey()
	{
		return QStringLiteral("SkipFlowConfirms");
	}

	inline QString SkjFrameBufferCountKey()
	{
		return QStringLiteral("SkjFrameBufferCount");
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
