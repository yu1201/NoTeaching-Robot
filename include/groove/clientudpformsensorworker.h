#ifndef CLIENTUDPFORMSENSORWORKER_H
#define CLIENTUDPFORMSENSORWORKER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

class ClientUDPFormSensorWorker : public QObject
{
    Q_OBJECT
public:
    explicit ClientUDPFormSensorWorker(QObject* parent = nullptr);
    ~ClientUDPFormSensorWorker() override;

public slots:
    void startReceive(const QString& serverIP);
    void stopReceive();

private slots:
    void sendHeartbeat();
    void readPendingDatagrams();

private:
    void resetReceiveState();

private:
    QByteArray      m_receiveBuffer;
    qint32          m_expectedSize;

    QUdpSocket* m_udp;
    QTimer* m_heartbeatTimer;

    QString         m_serverIP;
    quint16         m_serverPort;
    bool            m_running;
};

#endif
