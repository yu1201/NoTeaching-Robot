#pragma once

#include "ConfigDatabase.h"

#include <QString>
#include <QSysInfo>

// 在线服务配置（OTA 升级 + 扫描数据上传）：全局键，管理页「在线服务」界面编辑。
// 服务器为自建（xiaomomoyun.cn），OTA 走 HTTP 静态目录（latest.json + 安装包），
// 数据上传走 FTP（复用 FtpClient/WinINet）。密码键名含 password，ConfigDatabase 自动混淆存储。
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
		return ReadValue(QStringLiteral("UpdateBaseUrl"), QStringLiteral("http://xiaomomoyun.cn:8090/ota"));
	}

	inline void SetUpdateBaseUrl(const QString& url)
	{
		WriteValue(QStringLiteral("UpdateBaseUrl"), url);
	}

	// —— 数据上传（FTP）——
	inline QString FtpHost()
	{
		return ReadValue(QStringLiteral("FtpHost"), QStringLiteral("xiaomomoyun.cn"));
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

	// 默认随包分发「上传专用」账号 uploader：服务器端仅允许上传(STOR/建目录)、禁止下载(RETR)，
	// 现场设备开箱即可自动上传扫描数据，且下载不到别的设备数据。管理员要远程浏览/下载数据时，
	// 在界面手动改成「全权限」账号 devicedata（不随包分发）。
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
		// uploader 的密码（低权限、仅上传，随包默认；全权限账号密码不写死、由管理员手填）。
		return ReadValue(QStringLiteral("FtpPassword"), QStringLiteral("UprOLDgeLOmM1wjN"));
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
}
