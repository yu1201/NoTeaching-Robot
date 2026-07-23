#pragma once

#include "OnlineServicesConfig.h"

#include <QDialog>

#include <atomic>
#include <thread>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;

// 在线服务独立登录。账号密码由服务器 FTP 登录实时校验，程序只根据固定账号名
// 映射界面权限，不在客户端保存或比较默认账号的明文密码。
class OnlineServicesLoginDialog final : public QDialog
{
public:
	explicit OnlineServicesLoginDialog(QWidget* parent = nullptr);
	~OnlineServicesLoginDialog() override;

	QString Account() const;
	QString Password() const;
	OnlineServicesConfig::AccessLevel AccessLevel() const;

protected:
	void reject() override;

private:
	void StartVerification();
	void FinishVerification(bool ok);
	void JoinWorker();

	QComboBox* m_accountCombo = nullptr;
	QLineEdit* m_passwordEdit = nullptr;
	QLabel* m_statusLabel = nullptr;
	QPushButton* m_loginButton = nullptr;
	QPushButton* m_cancelButton = nullptr;
	QString m_verifiedAccount;
	QString m_verifiedPassword;
	QString m_savedAccount;
	QString m_savedPassword;
	std::atomic<bool> m_verifying{ false };
	std::thread m_worker;
};
