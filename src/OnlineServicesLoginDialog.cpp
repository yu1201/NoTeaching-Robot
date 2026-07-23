#include "OnlineServicesLoginDialog.h"

#include "AppPaths.h"
#include "FTPClient.h"
#include "RobotLog.h"

#include <QComboBox>
#include <QDir>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QVBoxLayout>

namespace
{
	std::string OnlineServicesLogPath()
	{
		return QDir::toNativeSeparators(
			AppPaths::WritablePath(QStringLiteral("Log/OnlineServices.txt")))
			.toLocal8Bit().toStdString();
	}
}

OnlineServicesLoginDialog::OnlineServicesLoginDialog(QWidget* parent)
	: QDialog(parent)
	, m_savedAccount(OnlineServicesConfig::FtpUser().trimmed())
	, m_savedPassword(OnlineServicesConfig::FtpPassword())
{
	setWindowTitle(QStringLiteral("登录在线服务"));
	setModal(true);
	setMinimumWidth(470);

	auto* layout = new QVBoxLayout(this);
	auto* hint = new QLabel(QStringLiteral("请选择服务器账号。登录成功后，程序按账号等级开放对应功能。"), this);
	hint->setWordWrap(true);
	layout->addWidget(hint);

	auto* form = new QFormLayout();
	m_accountCombo = new QComboBox(this);
	m_accountCombo->addItem(QStringLiteral("全权限"), OnlineServicesConfig::FullAccessAccount());
	m_accountCombo->addItem(QStringLiteral("FTP 权限"), OnlineServicesConfig::FtpAccessAccount());
	m_accountCombo->addItem(QStringLiteral("上传权限"), OnlineServicesConfig::UploadOnlyAccount());
	m_passwordEdit = new QLineEdit(this);
	m_passwordEdit->setEchoMode(QLineEdit::Password);
	m_passwordEdit->setPlaceholderText(QStringLiteral("服务器密码"));
	form->addRow(QStringLiteral("账号等级"), m_accountCombo);
	form->addRow(QStringLiteral("密码"), m_passwordEdit);
	layout->addLayout(form);

	m_statusLabel = new QLabel(this);
	m_statusLabel->setWordWrap(true);
	m_statusLabel->setStyleSheet(QStringLiteral("color: #8FB0BC;"));
	layout->addWidget(m_statusLabel);

	auto* buttons = new QHBoxLayout();
	buttons->addStretch(1);
	m_cancelButton = new QPushButton(QStringLiteral("取消"), this);
	m_loginButton = new QPushButton(QStringLiteral("登录"), this);
	m_loginButton->setDefault(true);
	m_loginButton->setProperty("kind", "primary");
	buttons->addWidget(m_cancelButton);
	buttons->addWidget(m_loginButton);
	layout->addLayout(buttons);

	int selected = m_accountCombo->findData(m_savedAccount);
	if (selected < 0)
	{
		selected = m_accountCombo->findData(OnlineServicesConfig::UploadOnlyAccount());
	}
	m_accountCombo->setCurrentIndex(selected);
	if (m_savedAccount == m_accountCombo->currentData().toString())
	{
		m_passwordEdit->setText(m_savedPassword);
	}

	connect(m_accountCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]()
		{
			const QString account = m_accountCombo->currentData().toString();
			m_passwordEdit->setText(account == m_savedAccount ? m_savedPassword : QString());
			m_statusLabel->clear();
		});
	connect(m_passwordEdit, &QLineEdit::returnPressed, this, [this]() { StartVerification(); });
	connect(m_loginButton, &QPushButton::clicked, this, [this]() { StartVerification(); });
	connect(m_cancelButton, &QPushButton::clicked, this, [this]() { reject(); });
}

OnlineServicesLoginDialog::~OnlineServicesLoginDialog()
{
	JoinWorker();
}

QString OnlineServicesLoginDialog::Account() const
{
	return m_verifiedAccount;
}

QString OnlineServicesLoginDialog::Password() const
{
	return m_verifiedPassword;
}

OnlineServicesConfig::AccessLevel OnlineServicesLoginDialog::AccessLevel() const
{
	return OnlineServicesConfig::AccessLevelForAccount(m_verifiedAccount);
}

void OnlineServicesLoginDialog::reject()
{
	if (m_verifying.load())
	{
		m_statusLabel->setText(QStringLiteral("正在向服务器验证，请稍候…"));
		return;
	}
	QDialog::reject();
}

void OnlineServicesLoginDialog::StartVerification()
{
	if (m_verifying.exchange(true))
	{
		return;
	}
	JoinWorker();
	const QString account = m_accountCombo->currentData().toString().trimmed();
	const QString password = m_passwordEdit->text();
	if (!OnlineServicesConfig::IsDefaultFtpAccount(account) || password.isEmpty())
	{
		m_verifying.store(false);
		m_statusLabel->setText(QStringLiteral("请输入服务器密码。"));
		return;
	}

	m_accountCombo->setEnabled(false);
	m_passwordEdit->setEnabled(false);
	m_loginButton->setEnabled(false);
	m_cancelButton->setEnabled(false);
	m_statusLabel->setText(QStringLiteral("正在连接服务器并验证账号…"));
	const std::string host = OnlineServicesConfig::FtpHost().toStdString();
	const int port = OnlineServicesConfig::FtpPort();
	m_worker = std::thread([this, account, password, host, port]()
		{
			RobotLog log(OnlineServicesLogPath(), false);
			FtpClient ftp(&log, host, port, account.toStdString(), password.toStdString());
			ftp.setMessageBoxesEnabled(false);
			const bool ok = ftp.connect();
			QMetaObject::invokeMethod(this, [this, account, password, ok]()
				{
					if (ok)
					{
						m_verifiedAccount = account;
						m_verifiedPassword = password;
					}
					FinishVerification(ok);
				}, Qt::QueuedConnection);
		});
}

void OnlineServicesLoginDialog::FinishVerification(bool ok)
{
	JoinWorker();
	m_verifying.store(false);
	if (ok)
	{
		accept();
		return;
	}
	m_accountCombo->setEnabled(true);
	m_passwordEdit->setEnabled(true);
	m_loginButton->setEnabled(true);
	m_cancelButton->setEnabled(true);
	m_statusLabel->setText(QStringLiteral("登录失败：密码不正确、账号未在服务器启用，或服务器暂时不可达。"));
	m_passwordEdit->selectAll();
	m_passwordEdit->setFocus();
}

void OnlineServicesLoginDialog::JoinWorker()
{
	if (m_worker.joinable())
	{
		m_worker.join();
	}
}
