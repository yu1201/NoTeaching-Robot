#ifndef TCPSENSORCLIENTWORKER_H
#define TCPSENSORCLIENTWORKER_H

#include <QObject>
#include <QTcpSocket>
#include <QTimer>

#include <opencv2/opencv.hpp>

class CameraFrameCache;

class TcpSensorClientWorker : public QObject
{
    Q_OBJECT
public:
    explicit TcpSensorClientWorker(CameraFrameCache* frameCache = nullptr,QObject* parent = nullptr);
    ~TcpSensorClientWorker() override;

    void setFrameCache(CameraFrameCache* frameCache);

public slots:
    void startClient(const QString& serverIP);
    void startClient(const QString& serverIP, int serverPort);
    void stopClient();
    // 兼容旧 UDP worker 的调用入口；端口参数会作为 TCP 目标端口使用。
    void startReceive(const QString& serverIP, int serverPort);
    void stopReceive();

signals:
    void diagnosticChanged(
        qint64 datagramCount,
        qint64 filteredDatagramCount,
        qint64 decodedFrameCount,
        qint64 decodeFailedCount,
        qint64 appendedFrameCount,
        const QString& statusText);
    void targetDiagnosticChanged(
        const QString& targetIP,
        qint64 datagramCount,
        qint64 filteredDatagramCount,
        qint64 decodedFrameCount,
        qint64 decodeFailedCount,
        qint64 appendedFrameCount,
        const QString& statusText);

private slots:
    void sendHeartBeat();
    void onDataReceived();
    void onSocketConnected();
    void onSocketDisconnected();
    void onSocketErrorOccur(QAbstractSocket::SocketError err);

private:
    void openSocket();
    void cleanupSocket();
    void scheduleReconnect(const QString& reason);
    void resetReceiveState();
    void resetCounters();

private:
    QByteArray      m_recvBuffer;
    qint32          m_frameTotalSize;

    QTcpSocket* m_tcpSocket;
    QTimer* m_heartBeatTimer;

    QString         m_serverIP;
    quint16         m_serverPort;
    bool            m_isRunning;

    CameraFrameCache* m_frameCache;
    qint64          m_datagramCount;
    qint64          m_filteredDatagramCount;
    qint64          m_decodedFrameCount;
    qint64          m_decodeFailedCount;
    qint64          m_appendedFrameCount;
    bool            m_loggedFirstDecodedFrame;
    bool            m_loggedFirstDecodeFailure;
    bool            m_loggedFirstAppendedFrame;

    qulonglong      m_lastTimestamp;
};

#endif // TCPSENSORCLIENTWORKER_H
