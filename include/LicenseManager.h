#pragma once

#include <QObject>
#include <QJsonObject>
#include <QString>
#include <functional>
#include <memory>

class QWidget;

class LicenseManager final : public QObject
{
public:
    enum class LicenseMode { Off = 0, Audit = 1, Enforce = 2 };
    static LicenseManager& Instance();
    ~LicenseManager() override;

    // Initialize and Start must be called from the application GUI thread.
    void Initialize(bool readOnly = false);
    void Start();
    LicenseMode Mode() const;
    bool IsEnforced() const;
    bool CanStartProtectedOperation(QString* reason = nullptr) const;
    bool IsLockRequested() const;
    bool IsNetworkBusy() const;
    QString StatusText() const;
    void ShowDialog(QWidget* parent = nullptr,
        std::function<void()> safetyRecovery = {}, std::function<void()> safetyStop = {},
        std::function<bool()> safetyBusy = {}, bool acknowledgeBlockedGate = false);

    // Runtime state/acknowledgement describes the actual robot state, rather
    // than assuming that a delivered lock has already taken physical effect.
    void SetRuntimeState(const QString& state);
    void SetEffectStatus(const QString& status);
    void AcknowledgeAppliedPolicy();
    void Register(const QString& activationCode = QString());
    void SyncNow();
    bool ExportOfflineRequest(const QString& path, QString* error = nullptr);
    bool ImportOfflineResponse(const QString& path, QString* error = nullptr);

    // Called on the GUI thread and always outside the internal mutex.
    std::function<void()> onPolicyChanged;

#ifdef HK_LICENSE_TEST_BUILD
    void ResetForTest(const QString& statePath, const QJsonObject& fingerprint,
        const QByteArray& publicBlob, const QString& installationId, qint64 now,
        const QString& deviceToken = QString());
    QJsonObject LastRequestForTest() const;
    QString LastRequestEndpointForTest() const;
    void SetNowForTest(qint64 now);
    bool ApplyPolicyForTest(const QJsonObject& policy, const QString& nonce,
        QString* error = nullptr, bool online = true);
    bool ReloadForTest(QString* error = nullptr);
    qint64 HighestRevisionForTest() const;
    qint64 AppliedRevisionForTest() const;
#endif

private:
    LicenseManager();
    struct Data;
    std::unique_ptr<Data> d;
    void NotifyChanged();
    void Post(const QString& endpoint, QJsonObject request, bool registration);
};
