#pragma once

#include "ConfigDatabase.h"

#include <QString>
#include <QRegularExpression>
#include <QSysInfo>
#include <QUrl>

// 在线服务配置（OTA 升级 + 扫描数据上传）：全局键，管理页「在线服务」界面编辑。
// 服务器为自建（xiaomomoyun.cn），OTA 走 HTTP 静态目录
// （latest.json v2 兼容端点 + latest-v3.json 当前端点 + 安装包），
// 数据上传走 FTP（复用 FtpClient/WinINet）。可恢复凭据由 ConfigDatabase 使用
// Windows DPAPI CurrentUser 保护；安装包不携带服务器密码。
namespace OnlineServicesConfig
{
	enum class AccessLevel
	{
		Upload = 0,
		Ftp = 1,
		Full = 2
	};

	inline QString SettingsGroup()
	{
		return QStringLiteral("OnlineServices");
	}

	inline QString ReadValue(const QString& key, const QString& defaultValue)
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), key, &value)
			|| value.trimmed().isEmpty())
		{
			return defaultValue;
		}
		return value.trimmed();
	}

	inline void WriteValue(const QString& key, const QString& value)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), SettingsGroup(), key, value);
	}

	inline QString ReadSecretValue(const QString& key)
	{
		QString value;
		if (!ConfigDatabase::ReadScopedSetting(
			QStringLiteral("global"), QString(), SettingsGroup(), key, &value))
		{
			return QString();
		}
		return value;  // 密码不可 trim；服务器允许的首尾空格也是凭据的一部分。
	}

	inline QString ServerHost()
	{
		QString host = ReadValue(QStringLiteral("ServerHost"), QString());
		if (host.isEmpty())
		{
			// 兼容旧数据库：优先沿用原 FTP 主机，其次从完整升级源 URL 提取主机。
			host = ReadValue(QStringLiteral("FtpHost"), QString());
		}
		if (host.isEmpty())
		{
			host = QUrl(ReadValue(QStringLiteral("UpdateBaseUrl"), QString())).host();
		}
		return host.isEmpty() ? QStringLiteral("103.217.203.52") : host.trimmed();
	}

	inline void SetServerHost(const QString& host)
	{
		WriteValue(QStringLiteral("ServerHost"), host.trimmed());
	}

	// 服务器配置只保存一个主机/IP，其余固定端口与路径由程序派生。
	inline QString UpdateBaseUrl()
	{
		return QStringLiteral("http://%1:8090/ota").arg(ServerHost());
	}

	inline QString FtpHost()
	{
		return ServerHost();
	}

	inline int FtpPort()
	{
		return 21;
	}

	inline QString FullAccessAccount()
	{
		return QStringLiteral("devicedata");
	}

	inline QString FtpAccessAccount()
	{
		return QStringLiteral("ftpoperator");
	}

	inline QString UploadOnlyAccount()
	{
		return QStringLiteral("uploader");
	}

	inline bool IsDefaultFtpAccount(const QString& account)
	{
		const QString normalized = account.trimmed();
		return normalized == FullAccessAccount()
			|| normalized == FtpAccessAccount()
			|| normalized == UploadOnlyAccount();
	}

	inline bool IsFullAccessAccount(const QString& account)
	{
		return account.trimmed() == FullAccessAccount();
	}

	inline bool IsUploadOnlyAccount(const QString& account)
	{
		return account.trimmed() == UploadOnlyAccount();
	}

	inline AccessLevel AccessLevelForAccount(const QString& account)
	{
		if (IsFullAccessAccount(account))
		{
			return AccessLevel::Full;
		}
		if (account.trimmed() == FtpAccessAccount())
		{
			return AccessLevel::Ftp;
		}
		return AccessLevel::Upload;
	}

	inline bool HasFtpAccess(AccessLevel level)
	{
		return static_cast<int>(level) >= static_cast<int>(AccessLevel::Ftp);
	}

	inline bool HasFullAccess(AccessLevel level)
	{
		return level == AccessLevel::Full;
	}

	// 只保存最近一次已经通过服务器 FTP 登录验证的默认账号与密码。
	// 密码由 ConfigDatabase 以 Windows DPAPI CurrentUser 保护，不在服务器配置页显示。
	inline QString FtpUser()
	{
		return ReadValue(QStringLiteral("FtpUser"), UploadOnlyAccount());
	}

	inline void SetFtpUser(const QString& user)
	{
		WriteValue(QStringLiteral("FtpUser"), user);
	}

	inline QString FtpPassword()
	{
		return ReadSecretValue(QStringLiteral("FtpPassword"));
	}

	inline void SetFtpPassword(const QString& password)
	{
		WriteValue(QStringLiteral("FtpPassword"), password);
	}

	// —— 上传行为 ——
	// 默认关：服务器端部署完成、现场确认网络可达后再在管理页打开。
	inline bool AutoUploadEnabled()
	{
		return ReadValue(QStringLiteral("AutoUploadEnabled"), QStringLiteral("0")) == QStringLiteral("1");
	}

	inline void SetAutoUploadEnabled(bool enabled)
	{
		WriteValue(QStringLiteral("AutoUploadEnabled"), enabled ? QStringLiteral("1") : QStringLiteral("0"));
	}

	inline bool IsServerAccountName(const QString& value)
	{
		static const QRegularExpression pattern(
			QStringLiteral(R"(^[a-z][a-z0-9_-]{2,31}$)"));
		return pattern.match(value).hasMatch();
	}

	inline QString DefaultDeviceName()
	{
		const QString value = QSysInfo::machineHostName().trimmed();
		return value.isEmpty() ? QStringLiteral("device-unknown") : value;
	}

	// 设备名只用于服务器 /data/<设备名>/ 目录；登录账号由三级固定账号独立决定。
	inline QString DeviceName()
	{
		return ReadValue(QStringLiteral("DeviceName"), DefaultDeviceName());
	}

	inline void SetDeviceName(const QString& name)
	{
		WriteValue(QStringLiteral("DeviceName"), name);
	}

	// 失败重试队列（JSON 数组字符串，元素为 Result 案例目录绝对路径）。
	inline QString PendingUploads()
	{
		return ReadValue(QStringLiteral("PendingUploads"), QStringLiteral("[]"));
	}

	inline void SetPendingUploads(const QString& jsonArray)
	{
		WriteValue(QStringLiteral("PendingUploads"), jsonArray);
	}

	// 服务器管理接口令牌（nginx /admin/ 反代 + X-Admin-Token 鉴权）：账号增删/改权限与磁盘统计用。
	// 不在界面显示或编辑；由部署时写入本机数据库，并仅在全权限会话中自动使用。
	inline QString AdminToken()
	{
		return ReadValue(QStringLiteral("AdminApiToken"), QString());
	}

	inline void SetAdminToken(const QString& token)
	{
		WriteValue(QStringLiteral("AdminApiToken"), token);
	}

	// 升级（增量或全量）后的目标版本：安装前记下，重启后主窗口自检「实际版本 == 目标?」，
	// 不符即告警（补丁/安装没生效，如误挂了含旧 exe 的补丁）。空=无待验证升级。
	inline QString PendingUpdateTargetVersion()
	{
		return ReadValue(QStringLiteral("PendingUpdateTargetVersion"), QString());
	}

	inline void SetPendingUpdateTargetVersion(const QString& version)
	{
		WriteValue(QStringLiteral("PendingUpdateTargetVersion"), version);
	}

	// 每个 OTA 通道持久化本机见过的最高签名版本。HTTP 传输下用于拒绝已见版本回放；
	// 这是 TOFU 水位，不保护首次检查/配置被清空的设备。首次设备仍依赖 7 天清单
	// 有效期限制回放窗口；它不能替代 HTTPS 或证书固定。
	inline QString HighestSeenUpdateVersion(const QString& channel)
	{
		return ReadValue(QStringLiteral("HighestSeenUpdateVersion_%1").arg(channel), QString());
	}

	inline void SetHighestSeenUpdateVersion(const QString& channel, const QString& version)
	{
		WriteValue(QStringLiteral("HighestSeenUpdateVersion_%1").arg(channel), version);
	}
}
