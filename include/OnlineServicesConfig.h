#pragma once

#include "ConfigDatabase.h"

#include <QString>
#include <QSysInfo>

// 在线服务配置（OTA 升级 + 扫描数据上传）：全局键，管理页「在线服务」界面编辑。
// 服务器为自建（xiaomomoyun.cn），OTA 走 HTTP 静态目录（latest.json + 安装包），
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
	// 完整清单地址 = UpdateBaseUrl + "/" + channel + "/latest.json"，channel 由品牌自动决定（neutral/brand）。
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

	// 默认仅预填低权限账号名，密码必须由管理员在现场配置，安装包不再分发共享密码。
	inline QString FtpUser()
	{
		return ReadValue(QStringLiteral("FtpUser"), QStringLiteral("uploader"));
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

	// 设备名：服务器上按 /data/<DeviceName>/ 分目录，多台设备互不覆盖。默认取主机名。
	inline QString DeviceName()
	{
		return ReadValue(QStringLiteral("DeviceName"), QSysInfo::machineHostName());
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
}
