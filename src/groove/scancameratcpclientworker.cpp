#include "groove/scancameratcpclientworker.h"

#include <QDataStream>
#include <QDateTime>
#include <QDebug>
#include <QNetworkProxy>
#include <QTimer>
#include <QtEndian>

#include <cstring>
#include <limits>

#include "CameraFrameCache.h"
#include "RobotLog.h"
#include "groove/framebuffer.h"
#include "groove/pointcloundresultframe.h"

namespace
{
constexpr quint16 kDefaultTcpServerPort = 50006;
constexpr int kHeartbeatIntervalMs = 300;
constexpr int kReconnectDelayMs = 1000;
constexpr int kTcpMaxPacketSize = 32 * 1024 * 1024;
constexpr int kMaxReceiveBufferSize = 2 * kTcpMaxPacketSize;

void WriteCameraTcpLog(const QString& text)
{
    // 统一走 RobotLog：按天归档 Log/<日期>/、自带写入锁、自动毫秒时间戳。
    static RobotLog logger("Log/CameraTcpClient.txt", false);
    logger.writeLine(text.toStdString());
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
    // 旧 TCP 协议的线点云 Z 与 resultPoints3D 相反；在相机底层统一为 targetPoint 坐标。
    udpFrame.allResultPoint = CanonicalizeCameraPointCloud(
        frame.dataPoints3D,
        CameraNativePointCloudConvention::LegacyCloudZOppositeTarget);
    udpFrame.allResultPointCanonical = true;
    udpFrame.mFps = frame.calcFrameRate;
    udpFrame.timestamp = frame.timestamp;
    return udpFrame;
}

bool DecodeTcpFrame(const QByteArray& frameData, PointCloundResultFrame& frame)
{
    QDataStream stream(frameData);
    stream.setByteOrder(QDataStream::BigEndian);

    quint32 magic = 0;
    quint32 headerSize = 0;
    quint32 totalSize = 0;
    qint64 timestamp = 0;
    qint32 channel = 0;
    QByteArray body;

    stream >> magic >> headerSize >> totalSize >> timestamp >> channel >> body;
    if (stream.status() != QDataStream::Ok
        || magic != PointCloundResultFrame::MAGIC_NUMBER
        || headerSize != PointCloundResultFrame::HEADER_SIZE
        || totalSize < PointCloundResultFrame::HEADER_SIZE)
    {
        return false;
    }

    quint32 checksum = 0;
    stream >> checksum;

    frame.magicNumber = magic;
    frame.headerSize = headerSize;
    frame.totalSize = totalSize;
    frame.timestamp = timestamp;
    frame.channel = channel;
    frame.checksum = checksum;

    return frame.deserializeBody(body);
}

int FindTcpFrameHeader(const QByteArray& buffer)
{
    const int size = buffer.size();
    if (size < static_cast<int>(PointCloundResultFrame::HEADER_SIZE))
    {
        return -1;
    }

    for (int i = 0; i <= size - static_cast<int>(PointCloundResultFrame::HEADER_SIZE); ++i)
    {
        quint32 magic = 0;
        memcpy(&magic, buffer.constData() + i, sizeof(magic));
        magic = qFromBigEndian(magic);
        if (magic != PointCloundResultFrame::MAGIC_NUMBER)
        {
            continue;
        }

        quint32 headerSize = 0;
        memcpy(&headerSize, buffer.constData() + i + 4, sizeof(headerSize));
        headerSize = qFromBigEndian(headerSize);
        if (headerSize != PointCloundResultFrame::HEADER_SIZE)
        {
            continue;
        }

        quint32 totalSize = 0;
        memcpy(&totalSize, buffer.constData() + i + 8, sizeof(totalSize));
        totalSize = qFromBigEndian(totalSize);
        if (totalSize < PointCloundResultFrame::HEADER_SIZE + sizeof(quint32)
            || totalSize > kTcpMaxPacketSize)
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
    if (bodySize > kTcpMaxPacketSize)
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

    // The sender serializes QByteArray through QDataStream, which adds a
    // four-byte body length. Some headers do not count that length.
    if (headerTotalSize == legacyHeaderTotalSize || headerTotalSize == qDataStreamPacketSize)
    {
        frameSize = static_cast<int>(qDataStreamPacketSize);
    }
    return frameSize;
}
}

ScanCameraTcpClientWorker::ScanCameraTcpClientWorker(CameraFrameCache* frameCache, QObject* parent)
    : QObject(parent)
    , m_frameCache(frameCache)
{
}

ScanCameraTcpClientWorker::~ScanCameraTcpClientWorker()
{
    stopClient();
}

void ScanCameraTcpClientWorker::setFrameCache(CameraFrameCache* frameCache)
{
    m_frameCache = frameCache;
}

void ScanCameraTcpClientWorker::startReceive(const QString& serverIP, int serverPort)
{
    startClient(serverIP, serverPort);
}

void ScanCameraTcpClientWorker::stopReceive()
{
    stopClient();
}

void ScanCameraTcpClientWorker::startClient(const QString& serverIP)
{
    startClient(serverIP, kDefaultTcpServerPort);
}

void ScanCameraTcpClientWorker::startClient(const QString& serverIP, int serverPort)
{
    stopClient();

    m_serverIP = serverIP;
    m_serverPort = (serverPort > 0 && serverPort <= 65535)
        ? static_cast<quint16>(serverPort)
        : kDefaultTcpServerPort;
    resetCounters();
    WriteCameraTcpLog(QString("start target=%1:%2 cache=%3")
        .arg(m_serverIP)
        .arg(m_serverPort)
        .arg(m_frameCache != nullptr ? "ok" : "null"));

    openSocket();
}

void ScanCameraTcpClientWorker::openSocket()
{
    cleanupSocket();
    if (m_serverIP.isEmpty() || m_serverPort == 0)
    {
        return;
    }

    m_tcpSocket = new QTcpSocket(this);
    // The camera is a direct LAN TCP endpoint; bypass system proxy settings.
    m_tcpSocket->setProxy(QNetworkProxy::NoProxy);
    m_tcpSocket->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::connected, this, &ScanCameraTcpClientWorker::onSocketConnected);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &ScanCameraTcpClientWorker::onSocketDisconnected);
    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &ScanCameraTcpClientWorker::onDataReceived);
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    connect(m_tcpSocket, &QTcpSocket::errorOccurred, this, &ScanCameraTcpClientWorker::onSocketErrorOccur);
#else
    connect(m_tcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
        this, &ScanCameraTcpClientWorker::onSocketErrorOccur);
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

void ScanCameraTcpClientWorker::cleanupSocket()
{
    if (m_tcpSocket)
    {
        QTcpSocket* socket = m_tcpSocket;
        m_tcpSocket = nullptr;
        QObject::disconnect(socket, nullptr, this, nullptr);
        socket->deleteLater();
    }
    resetReceiveState();
}

void ScanCameraTcpClientWorker::scheduleReconnect(const QString& reason)
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
        .arg(kReconnectDelayMs));

    QTimer::singleShot(kReconnectDelayMs, this, [this, targetIP, targetPort]()
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

void ScanCameraTcpClientWorker::stopClient()
{
    const bool wasRunning = m_isRunning || m_tcpSocket != nullptr || m_heartBeatTimer != nullptr;
    const QString stoppedIP = m_serverIP;
    const quint16 stoppedPort = m_serverPort;
    m_isRunning = false;

    if (m_heartBeatTimer)
    {
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

void ScanCameraTcpClientWorker::resetReceiveState()
{
    m_recvBuffer.clear();
    m_frameTotalSize = 0;
}

void ScanCameraTcpClientWorker::resetCounters()
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

void ScanCameraTcpClientWorker::sendHeartBeat()
{
    if (!m_isRunning || !m_tcpSocket || m_serverIP.isEmpty())
    {
        return;
    }

    m_tcpSocket->write("HEARTBEAT");
}

void ScanCameraTcpClientWorker::onDataReceived()
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
    if (m_recvBuffer.size() > kMaxReceiveBufferSize)
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

        quint32 totalSize = 0;
        memcpy(&totalSize, m_recvBuffer.constData() + 8, sizeof(totalSize));
        totalSize = qFromBigEndian(totalSize);

        if (totalSize < PointCloundResultFrame::HEADER_SIZE
            || totalSize > kTcpMaxPacketSize)
        {
            m_recvBuffer.remove(0, 1);
            ++m_filteredDatagramCount;
            continue;
        }

        const int frameSize = ResolveTcpFrameSize(m_recvBuffer, totalSize);
        if (frameSize < static_cast<int>(PointCloundResultFrame::HEADER_SIZE)
            || frameSize > static_cast<int>(kTcpMaxPacketSize + sizeof(quint32)))
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
                    .arg(totalSize);
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

void ScanCameraTcpClientWorker::onSocketConnected()
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

void ScanCameraTcpClientWorker::onSocketDisconnected()
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

void ScanCameraTcpClientWorker::onSocketErrorOccur(QAbstractSocket::SocketError err)
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
