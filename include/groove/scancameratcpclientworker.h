#ifndef SCANCAMERATCPCLIENTWORKER_H
#define SCANCAMERATCPCLIENTWORKER_H

#include <QObject>
#include <QTcpSocket>

class CameraFrameCache;
class QTimer;

class ScanCameraTcpClientWorker : public QObject
{
    Q_OBJECT
public:
    explicit ScanCameraTcpClientWorker(CameraFrameCache* frameCache = nullptr, QObject* parent = nullptr);
    ~ScanCameraTcpClientWorker() override;

    void setFrameCache(CameraFrameCache* frameCache);

public slots:
    void startClient(const QString& serverIP);
    void startClient(const QString& serverIP, int serverPort);
    void stopClient();
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
    QByteArray m_recvBuffer;
    qint32 m_frameTotalSize = 0;

    QTcpSocket* m_tcpSocket = nullptr;
    QTimer* m_heartBeatTimer = nullptr;

    QString m_serverIP;
    quint16 m_serverPort = 0;
    bool m_isRunning = false;

    CameraFrameCache* m_frameCache = nullptr;
    qint64 m_datagramCount = 0;
    qint64 m_filteredDatagramCount = 0;
    qint64 m_decodedFrameCount = 0;
    qint64 m_decodeFailedCount = 0;
    qint64 m_appendedFrameCount = 0;
    bool m_loggedFirstDecodedFrame = false;
    bool m_loggedFirstDecodeFailure = false;
    bool m_loggedFirstAppendedFrame = false;

    qulonglong m_lastTimestamp = 0;
};

#endif // SCANCAMERATCPCLIENTWORKER_H
