#ifndef TCPSENSORCLIENTWORKER_H
#define TCPSENSORCLIENTWORKER_H

#include <QObject>
#include <QTcpSocket>
#include <QTimer>

#include <opencv2/opencv.hpp>

class TcpSensorClientWorker : public QObject
{
    Q_OBJECT
public:
    explicit TcpSensorClientWorker(QObject* parent = nullptr);
    ~TcpSensorClientWorker() override;

public slots:
    void startClient(const QString& serverIP);
    void stopClient();

private slots:
    void sendHeartBeat();
    void onDataReceived();
    void onSocketConnected();
    void onSocketDisconnected();
    void onSocketErrorOccur(QAbstractSocket::SocketError err);

private:
    void resetReceiveState();

private:
    QByteArray      m_recvBuffer;
    qint32          m_frameTotalSize;

    QTcpSocket* m_tcpSocket;
    QTimer* m_heartBeatTimer;

    QString         m_serverIP;
    quint16         m_serverPort;
    bool            m_isRunning;

    qulonglong      m_lastTimestamp;
};

#endif // TCPSENSORCLIENTWORKER_H
