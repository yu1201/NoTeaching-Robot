#include "groove/tcpsensorclientworker.h"

#include <QDataStream>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QIODevice>
#include <QNetworkProxy>
#include <QTextStream>
#include <QtEndian>

#include <limits>
#include <mutex>

#include "CameraFrameCache.h"
#include "groove/framebuffer.h"
#include "groove/pointcloundresultframe.h"

// 服务器端口（和 PointCloundTcpServer 对应）。
#define TCP_SERVER_PORT       50006
#define HEARTBEAT_INTERVAL_MS 300
#define RECONNECT_DELAY_MS    1000
#define TCP_MAX_PACKET_SIZE   (32 * 1024 * 1024)
#define MAX_RECEIVE_BUFFER_SIZE (2 * TCP_MAX_PACKET_SIZE)

namespace
{
void WriteCameraTcpLog(const QString& text)
{
    static std::mutex logMutex;
    std::lock_guard<std::mutex> lock(logMutex);

    QDir dir(QDir::currentPath());
    if (!dir.exists("Log"))
    {
        dir.mkpath("Log");
    }

    QFile file(dir.filePath("Log/CameraTcpClient.txt"));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        return;
    }

    QTextStream stream(&file);
    stream << '[' << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz") << "] "
        << text << '\n';
}

udpDataShow BuildUdpFrame(const PointCloundResultFrame& frame)
{
    QVector<double> pointCloundX, pointCloundY;
    QVector<double> fitLineX, fitLineY;
    QVector<double> targetX, targetY;

    for (const cv::Point3d& p : frame.dataPoints3D)
    {
        pointCloundX.append(p.y);
        pointCloundY.append(p.z);
    }
    for (const cv::Point2d& p : frame.calcPoints2D)
    {
        fitLineX.append(p.x);
        fitLineY.append(p.y);
    }

    cv::Point3d targetPoint = frame.resultPoints3D;
    if (targetPoint == cv::Point3d(0, 0, 0))
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        targetPoint = cv::Point3d(nan, nan, nan);
    }

    targetX << targetPoint.y;
    targetY << -targetPoint.z;

    udpDataShow udpFrame;
    udpFrame.XData = pointCloundX;
    udpFrame.YData = pointCloundY;
    udpFrame.fitLineX = fitLineX;
    udpFrame.fitLineY = fitLineY;
    udpFrame.targetX = targetX;
    udpFrame.targetY = targetY;
    udpFrame.errorMessage = frame.errorMsg;
    udpFrame.targetPoint = targetPoint;
    udpFrame.allResultPoint = frame.dataPoints3D;
    udpFrame.mFps = frame.calcFrameRate;
    udpFrame.timestamp = frame.timestamp;
    return udpFrame;
}

bool DecodeTcpFrame(const QByteArray& frameData, PointCloundResultFrame& frame)
{
    QDataStream s(frameData);
    s.setByteOrder(QDataStream::BigEndian);

    quint32 magic = 0;
    quint32 hdrSz = 0;
    quint32 total = 0;
    qint64 ts = 0;
    qint32 ch = 0;
    QByteArray body;

    s >> magic >> hdrSz >> total >> ts >> ch >> body;
    if (s.status() != QDataStream::Ok
        || magic != PointCloundResultFrame::MAGIC_NUMBER
        || hdrSz != PointCloundResultFrame::HEADER_SIZE
        || total < PointCloundResultFrame::HEADER_SIZE)
    {
        return false;
    }

    quint32 checksum = 0;
    s >> checksum;

    frame.magicNumber = magic;
    frame.headerSize = hdrSz;
    frame.totalSize = total;
    frame.timestamp = ts;
    frame.channel = ch;
    frame.checksum = checksum;

    return frame.deserializeBody(body);
}

int FindTcpFrameHeader(const QByteArray& buffer)
{
    const int sz = buffer.size();
    if (sz < static_cast<int>(PointCloundResultFrame::HEADER_SIZE))
    {
        return -1;
    }

    for (int i = 0; i <= sz - static_cast<int>(PointCloundResultFrame::HEADER_SIZE); ++i)
    {
        quint32 magic = 0;
        memcpy(&magic, buffer.constData() + i, sizeof(magic));
        magic = qFromBigEndian(magic);
        if (magic != PointCloundResultFrame::MAGIC_NUMBER)
        {
            continue;
        }

        quint32 hdrSz = 0;
        memcpy(&hdrSz, buffer.constData() + i + 4, sizeof(hdrSz));
        hdrSz = qFromBigEndian(hdrSz);
        if (hdrSz != PointCloundResultFrame::HEADER_SIZE)
        {
            continue;
        }

        quint32 totalSz = 0;
        memcpy(&totalSz, buffer.constData() + i + 8, sizeof(totalSz));
        totalSz = qFromBigEndian(totalSz);
        if (totalSz < PointCloundResultFrame::HEADER_SIZE + sizeof(quint32)
            || totalSz > TCP_MAX_PACKET_SIZE)
        {
            continue;
        }

        return i;
    }
    return -1;
}

int ResolveTcpFrameSize(const QByteArray& buffer, quint32 headerTotalSize)
{
    int frameSize = static_cast<int>(headerTotalSize);
    if (buffer.size() < static_cast<int>(PointCloundResultFrame::HEADER_SIZE + sizeof(quint32)))
    {
        return frameSize;
    }

    quint32 bodySize = 0;
    memcpy(&bodySize, buffer.constData() + PointCloundResultFrame::HEADER_SIZE, sizeof(bodySize));
    bodySize = qFromBigEndian(bodySize);
    if (bodySize > TCP_MAX_PACKET_SIZE)
    {
        return frameSize;
    }

    const quint64 qDataStreamPacketSize =
        static_cast<quint64>(PointCloundResultFrame::HEADER_SIZE)
        + sizeof(quint32)
        + bodySize
        + sizeof(quint32);
    const quint64 legacyHeaderTotalSize =
        static_cast<quint64>(PointCloundResultFrame::HEADER_SIZE)
        + bodySize
        + sizeof(quint32);

    // 发送端使用 QDataStream << QByteArray 时会额外写入4字节body长度；
    // 老totalSize没有计入这4字节，TCP流按老长度截帧会少读校验和。
    if (headerTotalSize == legacyHeaderTotalSize || headerTotalSize == qDataStreamPacketSize)
    {
        frameSize = static_cast<int>(qDataStreamPacketSize);
    }
    return frameSize;
}
}

TcpSensorClientWorker::TcpSensorClientWorker(CameraFrameCache* frameCache, QObject* parent)
    : QObject(parent)
    , m_frameTotalSize(0)
    , m_tcpSocket(nullptr)
    , m_heartBeatTimer(nullptr)
    , m_serverPort(0)
    , m_isRunning(false)
    , m_frameCache(frameCache)
    , m_datagramCount(0)
    , m_filteredDatagramCount(0)
    , m_decodedFrameCount(0)
    , m_decodeFailedCount(0)
    , m_appendedFrameCount(0)
    , m_loggedFirstDecodedFrame(false)
    , m_loggedFirstDecodeFailure(false)
    , m_loggedFirstAppendedFrame(false)
    , m_lastTimestamp(0)
{
}

TcpSensorClientWorker::~TcpSensorClientWorker()
{
    stopClient();
}

void TcpSensorClientWorker::setFrameCache(CameraFrameCache* frameCache)
{
    m_frameCache = frameCache;
}

void TcpSensorClientWorker::startReceive(const QString& serverIP, int serverPort)
{
    startClient(serverIP, serverPort);
}

void TcpSensorClientWorker::stopReceive()
{
    stopClient();
}

void TcpSensorClientWorker::startClient(const QString& serverIP)
{
    startClient(serverIP, TCP_SERVER_PORT);
}

void TcpSensorClientWorker::startClient(const QString& serverIP, int serverPort)
{
    stopClient();

    m_serverIP = serverIP;
    m_serverPort = (serverPort > 0 && serverPort <= 65535)
        ? static_cast<quint16>(serverPort)
        : static_cast<quint16>(TCP_SERVER_PORT);
    resetCounters();
    WriteCameraTcpLog(QString("start target=%1:%2 cache=%3")
        .arg(m_serverIP)
        .arg(m_serverPort)
        .arg(m_frameCache != nullptr ? "ok" : "null"));

    openSocket();
}

void TcpSensorClientWorker::openSocket()
{
    cleanupSocket();
    if (m_serverIP.isEmpty() || m_serverPort == 0)
    {
        return;
    }

    m_tcpSocket = new QTcpSocket(this);
    // 相机是内网裸 TCP 连接，避免 Qt 继承系统代理导致 SocketError=10。
    m_tcpSocket->setProxy(QNetworkProxy::NoProxy);
    m_tcpSocket->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::connected, this, &TcpSensorClientWorker::onSocketConnected);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &TcpSensorClientWorker::onSocketDisconnected);
    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &TcpSensorClientWorker::onDataReceived);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    connect(m_tcpSocket, &QTcpSocket::errorOccurred, this, &TcpSensorClientWorker::onSocketErrorOccur);
#else
    connect(m_tcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
        this, &TcpSensorClientWorker::onSocketErrorOccur);
#endif

    m_tcpSocket->connectToHost(m_serverIP, m_serverPort);

    resetReceiveState();
    m_isRunning = true;
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("TCP连接中：目标相机 %1:%2").arg(m_serverIP).arg(m_serverPort));
}

void TcpSensorClientWorker::cleanupSocket()
{
    if (m_tcpSocket) {
        QTcpSocket* socket = m_tcpSocket;
        m_tcpSocket = nullptr;
        QObject::disconnect(socket, nullptr, this, nullptr);
        socket->deleteLater();
    }
    resetReceiveState();
}

void TcpSensorClientWorker::scheduleReconnect(const QString& reason)
{
    if (!m_isRunning || m_serverIP.isEmpty() || m_serverPort == 0)
    {
        return;
    }

    const QString targetIP = m_serverIP;
    const quint16 targetPort = m_serverPort;
    WriteCameraTcpLog(QString("[CameraTCP] reconnect scheduled target=%1:%2 reason=%3 delayMs=%4")
        .arg(targetIP)
        .arg(targetPort)
        .arg(reason)
        .arg(RECONNECT_DELAY_MS));

    QTimer::singleShot(RECONNECT_DELAY_MS, this, [this, targetIP, targetPort]()
        {
            if (!m_isRunning
                || m_tcpSocket != nullptr
                || m_serverIP != targetIP
                || m_serverPort != targetPort)
            {
                return;
            }
            WriteCameraTcpLog(QString("[CameraTCP] reconnecting target=%1:%2")
                .arg(targetIP)
                .arg(targetPort));
            openSocket();
        });
}

void TcpSensorClientWorker::stopClient()
{
    const bool wasRunning = m_isRunning || m_tcpSocket != nullptr || m_heartBeatTimer != nullptr;
    const QString stoppedIP = m_serverIP;
    const quint16 stoppedPort = m_serverPort;
    m_isRunning = false;

    if (m_heartBeatTimer) {
        QTimer* timer = m_heartBeatTimer;
        m_heartBeatTimer = nullptr;
        timer->stop();
        timer->deleteLater();
    }

    cleanupSocket();
    m_serverIP.clear();
    m_serverPort = 0;
    if (wasRunning)
    {
        const QString logText = QString("[CameraTCP] stopped target=%1:%2")
            .arg(stoppedIP)
            .arg(stoppedPort);
        qInfo().noquote() << logText;
        WriteCameraTcpLog(logText);
    }
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        "TCP已停止");
}

void TcpSensorClientWorker::resetReceiveState()
{
    m_recvBuffer.clear();
    m_frameTotalSize = 0;
}

void TcpSensorClientWorker::resetCounters()
{
    m_datagramCount = 0;
    m_filteredDatagramCount = 0;
    m_decodedFrameCount = 0;
    m_decodeFailedCount = 0;
    m_appendedFrameCount = 0;
    m_loggedFirstDecodedFrame = false;
    m_loggedFirstDecodeFailure = false;
    m_loggedFirstAppendedFrame = false;
}

void TcpSensorClientWorker::sendHeartBeat()
{
    if (!m_isRunning || !m_tcpSocket || m_serverIP.isEmpty())
        return;

    m_tcpSocket->write("HEARTBEAT");
}

void TcpSensorClientWorker::onDataReceived()
{
    if (!m_isRunning || !m_tcpSocket)
    {
        return;
    }

    ++m_datagramCount;
    const QByteArray newBytes = m_tcpSocket->readAll();
    m_recvBuffer.append(newBytes);
    if (m_datagramCount == 1)
    {
        WriteCameraTcpLog(QString("first read target=%1:%2 bytes=%3 buffer=%4")
            .arg(m_serverIP)
            .arg(m_serverPort)
            .arg(newBytes.size())
            .arg(m_recvBuffer.size()));
    }
    if (m_recvBuffer.size() > MAX_RECEIVE_BUFFER_SIZE)
    {
        const QString logText = QString("[CameraTCP] receive buffer overflow target=%1:%2 bytes=%3")
            .arg(m_serverIP)
            .arg(m_serverPort)
            .arg(m_recvBuffer.size());
        qWarning().noquote() << logText;
        WriteCameraTcpLog(logText);
        resetReceiveState();
        ++m_filteredDatagramCount;
    }

    while (true)
    {
        const int framePos = FindTcpFrameHeader(m_recvBuffer);
        if (framePos < 0)
        {
            break;
        }

        if (framePos > 0)
        {
            m_recvBuffer.remove(0, framePos);
            ++m_filteredDatagramCount;
        }

        if (m_recvBuffer.size() < static_cast<int>(PointCloundResultFrame::HEADER_SIZE))
        {
            break;
        }

        quint32 totalSz = 0;
        memcpy(&totalSz, m_recvBuffer.constData() + 8, sizeof(totalSz));
        totalSz = qFromBigEndian(totalSz);

        if (totalSz < PointCloundResultFrame::HEADER_SIZE
            || totalSz > TCP_MAX_PACKET_SIZE)
        {
            m_recvBuffer.remove(0, 1);
            ++m_filteredDatagramCount;
            continue;
        }

        const int frameSize = ResolveTcpFrameSize(m_recvBuffer, totalSz);
        if (frameSize < static_cast<int>(PointCloundResultFrame::HEADER_SIZE)
            || frameSize > static_cast<int>(TCP_MAX_PACKET_SIZE + sizeof(quint32)))
        {
            m_recvBuffer.remove(0, 1);
            ++m_filteredDatagramCount;
            continue;
        }

        if (m_recvBuffer.size() < frameSize)
        {
            break;
        }

        const QByteArray frameData = m_recvBuffer.left(frameSize);
        m_recvBuffer.remove(0, frameSize);

        PointCloundResultFrame frame;
        bool ok = frame.fromByteArray(frameData);
        if (!ok)
        {
            ok = DecodeTcpFrame(frameData, frame);
        }
        if (ok)
        {
            ++m_decodedFrameCount;
            if (!m_loggedFirstDecodedFrame)
            {
                m_loggedFirstDecodedFrame = true;
                const QString logText = QString("[CameraTCP] first decoded frame target=%1:%2 timestamp=%3 fps=%4 points=%5 fitPoints=%6 targetPoint=(%7,%8,%9)")
                    .arg(m_serverIP)
                    .arg(m_serverPort)
                    .arg(frame.timestamp)
                    .arg(frame.calcFrameRate)
                    .arg(frame.dataPoints3D.size())
                    .arg(frame.calcPoints2D.size())
                    .arg(frame.resultPoints3D.x, 0, 'f', 3)
                    .arg(frame.resultPoints3D.y, 0, 'f', 3)
                    .arg(frame.resultPoints3D.z, 0, 'f', 3);
                qInfo().noquote() << logText;
                WriteCameraTcpLog(logText);
            }

            const udpDataShow udpFrame = BuildUdpFrame(frame);
            if (m_frameCache != nullptr)
            {
                m_frameCache->AppendFrame(udpFrame);
                ++m_appendedFrameCount;
                if (!m_loggedFirstAppendedFrame)
                {
                    m_loggedFirstAppendedFrame = true;
                    const QString logText = QString("[CameraTCP] first frame appended target=%1:%2 timestamp=%3 cached=%4")
                        .arg(m_serverIP)
                        .arg(m_serverPort)
                        .arg(udpFrame.timestamp)
                        .arg(m_frameCache->CachedCount());
                    qInfo().noquote() << logText;
                    WriteCameraTcpLog(logText);
                }
            }
            else if (!m_loggedFirstAppendedFrame)
            {
                m_loggedFirstAppendedFrame = true;
                WriteCameraTcpLog(QString("[CameraTCP] decoded but cache is null target=%1:%2 timestamp=%3")
                    .arg(m_serverIP)
                    .arg(m_serverPort)
                    .arg(frame.timestamp));
            }
        }
        else
        {
            ++m_decodeFailedCount;
            if (!m_loggedFirstDecodeFailure)
            {
                m_loggedFirstDecodeFailure = true;
                const QString logText = QString("[CameraTCP] frame decode failed target=%1:%2 bufferBytes=%3 expectedBytes=%4")
                    .arg(m_serverIP)
                    .arg(m_serverPort)
                    .arg(frameData.size())
                    .arg(totalSz);
                qWarning().noquote() << logText;
                WriteCameraTcpLog(logText);
            }
        }
    }

    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("TCP收包中：目标相机 %1:%2").arg(m_serverIP).arg(m_serverPort));
}

void TcpSensorClientWorker::onSocketConnected()
{
    const QString logText = QString("[CameraTCP] connected target=%1:%2")
        .arg(m_serverIP)
        .arg(m_serverPort);
    qInfo().noquote() << logText;
    WriteCameraTcpLog(logText);
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("TCP已连接：目标相机 %1:%2").arg(m_serverIP).arg(m_serverPort));
}

void TcpSensorClientWorker::onSocketDisconnected()
{
    const QString logText = QString("[CameraTCP] disconnected target=%1:%2")
        .arg(m_serverIP)
        .arg(m_serverPort);
    qInfo().noquote() << logText;
    WriteCameraTcpLog(logText);
    cleanupSocket();
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("TCP已断开，准备重连：目标相机 %1:%2").arg(m_serverIP).arg(m_serverPort));
    scheduleReconnect("disconnected");
}

void TcpSensorClientWorker::onSocketErrorOccur(QAbstractSocket::SocketError err)
{
    const QString errorText = m_tcpSocket != nullptr ? m_tcpSocket->errorString() : QString();
    const QString logText = QString("[CameraTCP] error target=%1:%2 code=%3 text=%4")
        .arg(m_serverIP)
        .arg(m_serverPort)
        .arg(static_cast<int>(err))
        .arg(errorText);
    qWarning().noquote() << logText;
    WriteCameraTcpLog(logText);
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("TCP错误，准备重连：%1").arg(errorText));
    cleanupSocket();
    scheduleReconnect(errorText);
}
