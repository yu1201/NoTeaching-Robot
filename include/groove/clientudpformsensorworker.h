#ifndef CLIENTUDPFORMSENSORWORKER_H
#define CLIENTUDPFORMSENSORWORKER_H

#include <QObject>
#include <QHash>
#include <QUdpSocket>
#include <QTimer>

class CameraFrameCache;

class ClientUDPFormSensorWorker : public QObject
{
    Q_OBJECT
public:
    explicit ClientUDPFormSensorWorker(CameraFrameCache* frameCache = nullptr, QObject* parent = nullptr);
    ~ClientUDPFormSensorWorker() override;

    void setFrameCache(CameraFrameCache* frameCache);
    void setDispatchTargets(const QHash<QString, CameraFrameCache*>& targets);

public slots:
    void startReceive(const QString& serverIP, int serverPort);
    void startReceiveDemux(int serverPort);
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
    void sendHeartbeat();
    void readPendingDatagrams();

private:
    void resetReceiveState();
    void resetDispatchTargetState();
    void processDemuxDatagram(
        const QByteArray& buf,
        const QHostAddress& senderAddress,
        quint16 senderPort);

private:
    static constexpr int MAX_FRAME_PACKET_SIZE = 16 * 1024 * 1024;
    static constexpr int MAX_RECEIVE_BUFFER_SIZE = 2 * MAX_FRAME_PACKET_SIZE;

    struct DispatchTarget
    {
        CameraFrameCache* frameCache = nullptr;
        QByteArray receiveBuffer;
        qint32 expectedSize = 0;
        qint64 datagramCount = 0;
        qint64 filteredDatagramCount = 0;
        qint64 decodedFrameCount = 0;
        qint64 decodeFailedCount = 0;
        qint64 appendedFrameCount = 0;
        bool loggedFirstDatagram = false;
        bool loggedFirstDecodedFrame = false;
        bool loggedFirstDecodeFailure = false;
        bool loggedFirstAppendedFrame = false;
    };

    QByteArray      m_receiveBuffer;
    qint32          m_expectedSize;

    QUdpSocket* m_udp;
    QTimer* m_heartbeatTimer;

    QString         m_serverIP;
    quint16         m_serverPort;
    bool            m_running;
    bool            m_demuxMode = false;
    CameraFrameCache* m_frameCache;
    QHash<QString, DispatchTarget> m_dispatchTargets;
    qint64          m_datagramCount = 0;
    qint64          m_filteredDatagramCount = 0;
    qint64          m_decodedFrameCount = 0;
    qint64          m_decodeFailedCount = 0;
    qint64          m_appendedFrameCount = 0;
    bool            m_loggedFirstDatagram = false;
    bool            m_loggedFirstFilteredDatagram = false;
    bool            m_loggedFirstDecodedFrame = false;
    bool            m_loggedFirstDecodeFailure = false;
    bool            m_loggedFirstAppendedFrame = false;
};

#endif
