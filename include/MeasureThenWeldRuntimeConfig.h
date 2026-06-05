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
}
