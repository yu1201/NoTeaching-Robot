#pragma once

#include "ConfigDatabase.h"

#include <QString>
#include <QRegularExpression>
#include <QSysInfo>

// 在线服务配置（OTA 升级 + 扫描数据上传）：全局键，管理页「在线服务」界面编辑。
// 服务器为自建（xiaomomoyun.cn），OTA 走 HTTP 静态目录
// （latest.json v2 兼容端点 + latest-v3.json 当前端点 + 安装包），
// 数据上传走 FTP（复用 FtpClient/WinINet）。可恢复凭据由 ConfigDatabase 使用
// Windows DPAPI CurrentUser 保护；安装包不携带服务器密码。
namespace OnlineServicesConfig
{
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

	// —— OTA 更新源 ——
	// 当前清单地址 = UpdateBaseUrl + "/" + channel + "/latest-v3.json"；
	// latest.json 保留给已经部署、只理解 schema v2 的旧客户端。
	// 默认直连服务器 IP（156.239.225.105，:8090，8080 被机上 qqbot 占用）；域名解析就绪后可在界面改回域名。
	inline QString UpdateBaseUrl()
	{
		return ReadValue(QStringLiteral("UpdateBaseUrl"), QStringLiteral("http://103.217.203.52:8090/ota"));
	}

	inline void SetUpdateBaseUrl(const QString& url)
	{
		WriteValue(QStringLiteral("UpdateBaseUrl"), url);
	}

	// —— 数据上传（FTP）——
	inline QString FtpHost()
	{
		return ReadValue(QStringLiteral("FtpHost"), QStringLiteral("103.217.203.52"));
	}

	inline void SetFtpHost(const QString& host)
	{
		WriteValue(QStringLiteral("FtpHost"), host);
	}

	inline int FtpPort()
	{
		bool ok = false;
		const int port = ReadValue(QStringLiteral("FtpPort"), QStringLiteral("21")).toInt(&ok);
		return (ok && port > 0 && port <= 65535) ? port : 21;
	}

	inline void SetFtpPort(int port)
	{
		WriteValue(QStringLiteral("FtpPort"), QString::number(port));
	}

	// 共享 uploader 已永久退役。账号与密码都必须由管理员在现场显式配置。
	inline QString FtpUser()
	{
		return ReadValue(QStringLiteral("FtpUser"), QString());
	}

	inline void SetFtpUser(const QString& user)
	{
		WriteValue(QStringLiteral("FtpUser"), user);
	}

	inline QString FtpPassword()
	{
		return ReadValue(QStringLiteral("FtpPassword"), QString());
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
		QString value = QSysInfo::machineHostName().trimmed().toLower();
		value.replace(QRegularExpression(QStringLiteral("[^a-z0-9_-]+")), QStringLiteral("-"));
		while (value.startsWith(QLatin1Char('-')) || value.startsWith(QLatin1Char('_')))
		{
			value.remove(0, 1);
		}
		if (value.isEmpty() || value.front() < QLatin1Char('a') || value.front() > QLatin1Char('z'))
		{
			value.prepend(QStringLiteral("device-"));
		}
		value = value.left(32);
		while (value.size() < 3)
		{
			value.append(QLatin1Char('0'));
		}
		return IsServerAccountName(value) ? value : QStringLiteral("device-unknown");
	}

	// 设备名同时也是服务端普通 FTP 账号名；必须匹配服务端账号正则。
	inline QString DeviceName()
	{
		return ReadValue(QStringLiteral("DeviceName"), DefaultDeviceName());
	}

	inline void SetDeviceName(const QString& name)
	{
		WriteValue(QStringLiteral("DeviceName"), name);
	}

	// 自动扫描数据上传不允许使用已退役 uploader 或唯一全权限 devicedata。客户端配置本身不能证明
	// 服务端 ACL；这里只实施可执行的失败关闭前置条件：必须由服务端为设备配置
	// 与设备名完全一致的独立 FTP 账号，并在服务端限制该账号只能写 /data/<device>。
	// 即使通过此函数，真正的跨设备隔离仍由服务端 ACL 完成，不把本地字符串校验
	// 伪装成服务端身份证明。
	inline bool IsDeviceBoundUploadIdentity(
		const QString& userValue,
		const QString& deviceValue,
		QString* error = nullptr)
	{
		const QString user = userValue.trimmed();
		const QString device = deviceValue.trimmed();
		QString reason;
		if (user.isEmpty() || device.isEmpty())
		{
			reason = QStringLiteral("设备专用 FTP 账号或设备名为空");
		}
		else if (!IsServerAccountName(user) || !IsServerAccountName(device))
		{
			reason = QStringLiteral("账号和设备名必须匹配 ^[a-z][a-z0-9_-]{2,31}$");
		}
		else if (user == QStringLiteral("uploader"))
		{
			reason = QStringLiteral("共享 uploader 账号已永久退役");
		}
		else if (user == QStringLiteral("devicedata"))
		{
			reason = QStringLiteral("devicedata 是唯一全权限管理账号，不能作为设备上传身份");
		}
		else if (user.compare(device, Qt::CaseSensitive) != 0)
		{
			reason = QStringLiteral("设备专用 FTP 用户名必须与设备名完全一致");
		}
		if (error != nullptr)
		{
			*error = reason;
		}
		return reason.isEmpty();
	}

	inline bool HasDeviceBoundUploadIdentity(QString* error = nullptr)
	{
		return IsDeviceBoundUploadIdentity(FtpUser(), DeviceName(), error);
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
	// 不随包分发，由管理员在「在线服务 → 服务器配置」手填；本地使用 DPAPI CurrentUser 保护。
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
