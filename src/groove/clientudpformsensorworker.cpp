#include "groove/clientudpformsensorworker.h"
#include <QDebug>
#include <QDataStream>
#include <QIODevice>
#include <QVariant>
#include <QtEndian>
#include <cstring>
#include "CameraFrameCache.h"
#include "groove/pointcloundresultframe.h"
#include "groove/framebuffer.h"

namespace
{
constexpr quint16 kDefaultUdpPort = 50004;
constexpr int kMaxFramePacketSize = 16 * 1024 * 1024;

bool IsExpectedSender(const QHostAddress& senderAddress, const QString& expectedIP)
{
    if (expectedIP.trimmed().isEmpty())
    {
        return true;
    }

    const QHostAddress expectedAddress(expectedIP.trimmed());
    if (!expectedAddress.isNull())
    {
        return senderAddress.toIPv4Address() == expectedAddress.toIPv4Address();
    }

    return senderAddress.toString() == expectedIP.trimmed();
}

QString NormalizeAddressKey(const QString& ipText)
{
    const QString trimmed = ipText.trimmed();
    const QHostAddress address(trimmed);
    if (!address.isNull())
    {
        return QHostAddress(address.toIPv4Address()).toString();
    }
    return trimmed;
}

QString NormalizeAddressKey(const QHostAddress& address)
{
    return QHostAddress(address.toIPv4Address()).toString();
}

quint16 NormalizeUdpPort(int serverPort)
{
    if (serverPort <= 0 || serverPort > 65535)
    {
        return kDefaultUdpPort;
    }
    return static_cast<quint16>(serverPort);
}

quint32 ReadBigEndianUInt32(const QByteArray& buffer, int offset)
{
    quint32 value = 0;
    if (offset < 0 || offset + static_cast<int>(sizeof(value)) > buffer.size())
    {
        return 0;
    }
    std::memcpy(&value, buffer.constData() + offset, sizeof(value));
    return qFromBigEndian(value);
}

int ResolveFrameByteCount(const QByteArray& buffer, quint32 headerTotalSize)
{
    const int headerSize = static_cast<int>(PointCloundResultFrame::HEADER_SIZE);
    if (buffer.size() < headerSize + static_cast<int>(sizeof(quint32)))
    {
        return static_cast<int>(headerTotalSize);
    }

    const quint32 bodySize = ReadBigEndianUInt32(buffer, headerSize);
    if (bodySize > static_cast<quint32>(kMaxFramePacketSize))
    {
        return static_cast<int>(headerTotalSize);
    }

    const quint64 legacyTotalSize =
        static_cast<quint64>(PointCloundResultFrame::HEADER_SIZE)
        + static_cast<quint64>(bodySize)
        + sizeof(quint32);
    const quint64 qDataStreamPacketSize =
        static_cast<quint64>(PointCloundResultFrame::HEADER_SIZE)
        + sizeof(quint32)
        + static_cast<quint64>(bodySize)
        + sizeof(quint32);
    if (headerTotalSize == legacyTotalSize || headerTotalSize == qDataStreamPacketSize)
    {
        return static_cast<int>(qDataStreamPacketSize);
    }
    return static_cast<int>(headerTotalSize);
}

bool TryTakeCompleteFrame(QByteArray& receiveBuffer, qint32& expectedSize, QByteArray& frameBytes)
{
    frameBytes.clear();
    const int headerSize = static_cast<int>(PointCloundResultFrame::HEADER_SIZE);

    while (receiveBuffer.size() >= headerSize)
    {
        const int headerIndex = PointCloundResultFrame::findFrameHeader(receiveBuffer);
        if (headerIndex < 0)
        {
            const int keepBytes = headerSize > 1 ? headerSize - 1 : 0;
            if (receiveBuffer.size() > keepBytes)
            {
                receiveBuffer.remove(0, receiveBuffer.size() - keepBytes);
            }
            expectedSize = 0;
            return false;
        }
        if (headerIndex > 0)
        {
            receiveBuffer.remove(0, headerIndex);
        }
        if (receiveBuffer.size() < headerSize)
        {
            expectedSize = 0;
            return false;
        }

        const quint32 headerTotalSize = ReadBigEndianUInt32(receiveBuffer, 8);
        if (headerTotalSize < PointCloundResultFrame::HEADER_SIZE + sizeof(quint32)
            || headerTotalSize > static_cast<quint32>(kMaxFramePacketSize))
        {
            receiveBuffer.remove(0, 1);
            expectedSize = 0;
            continue;
        }

        const int frameSize = ResolveFrameByteCount(receiveBuffer, headerTotalSize);
        expectedSize = static_cast<qint32>(frameSize);
        if (frameSize <= 0 || frameSize > kMaxFramePacketSize)
        {
            receiveBuffer.remove(0, 1);
            expectedSize = 0;
            continue;
        }
        if (receiveBuffer.size() < frameSize)
        {
            return false;
        }

        frameBytes = receiveBuffer.left(frameSize);
        receiveBuffer.remove(0, frameSize);
        expectedSize = 0;
        return true;
    }

    expectedSize = 0;
    return false;
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

    targetX << frame.resultPoints3D.y;
    targetY << -frame.resultPoints3D.z;

    udpDataShow udpFrame;
    udpFrame.XData = pointCloundX;
    udpFrame.YData = pointCloundY;
    udpFrame.fitLineX = fitLineX;
    udpFrame.fitLineY = fitLineY;
    udpFrame.targetX = targetX;
    udpFrame.targetY = targetY;
    udpFrame.errorMessage = frame.errorMsg;
    udpFrame.targetPoint = frame.resultPoints3D;
    udpFrame.allResultPoint = frame.dataPoints3D;
    udpFrame.mFps = frame.calcFrameRate;
    udpFrame.timestamp = frame.timestamp;
    return udpFrame;
}
}

ClientUDPFormSensorWorker::ClientUDPFormSensorWorker(CameraFrameCache* frameCache, QObject* parent)
    : QObject(parent)
    , m_udp(nullptr)
    , m_heartbeatTimer(nullptr)
    , m_expectedSize(0)
    , m_serverPort(0)
    , m_running(false)
    , m_frameCache(frameCache)
{
}

ClientUDPFormSensorWorker::~ClientUDPFormSensorWorker()
{
    stopReceive();
}

void ClientUDPFormSensorWorker::setFrameCache(CameraFrameCache* frameCache)
{
    m_frameCache = frameCache;
}

void ClientUDPFormSensorWorker::setDispatchTargets(const QHash<QString, CameraFrameCache*>& targets)
{
    m_dispatchTargets.clear();
    for (auto it = targets.constBegin(); it != targets.constEnd(); ++it)
    {
        const QString key = NormalizeAddressKey(it.key());
        if (key.isEmpty() || it.value() == nullptr)
        {
            continue;
        }
        DispatchTarget target;
        target.frameCache = it.value();
        m_dispatchTargets.insert(key, target);
    }
}

// ===============================
// 全新启动（无锁）
// ===============================
void ClientUDPFormSensorWorker::startReceive(const QString& serverIP, int serverPort)
{
    // 先完全清理
    stopReceive();

    m_serverIP = serverIP;
    m_serverPort = NormalizeUdpPort(serverPort);
    m_demuxMode = false;
    m_datagramCount = 0;
    m_filteredDatagramCount = 0;
    m_decodedFrameCount = 0;
    m_decodeFailedCount = 0;
    m_appendedFrameCount = 0;
    m_loggedFirstDatagram = false;
    m_loggedFirstFilteredDatagram = false;
    m_loggedFirstDecodedFrame = false;
    m_loggedFirstDecodeFailure = false;
    m_loggedFirstAppendedFrame = false;

    // 新建UDP
    m_udp = new QUdpSocket(this);
    m_udp->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    m_udp->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    if (!m_udp->bind(
        QHostAddress::AnyIPv4,
        m_serverPort,
        QAbstractSocket::ShareAddress | QAbstractSocket::ReuseAddressHint)) {
        qWarning().noquote() << QString("[CameraUDP] bind failed localPort=%1 target=%2 error=%3")
            .arg(m_serverPort)
            .arg(m_serverIP)
            .arg(m_udp->errorString());
        emit diagnosticChanged(
            m_datagramCount,
            m_filteredDatagramCount,
            m_decodedFrameCount,
            m_decodeFailedCount,
            m_appendedFrameCount,
            QString("UDP绑定失败：本地端口 %1").arg(m_serverPort));
        delete m_udp;
        m_udp = nullptr;
        return;
    }

    // 新建心跳定时器
    m_heartbeatTimer = new QTimer(this);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &ClientUDPFormSensorWorker::sendHeartbeat);

    // 接收信号
    connect(m_udp, &QUdpSocket::readyRead, this, &ClientUDPFormSensorWorker::readPendingDatagrams);

    // 启动
    resetReceiveState();
    m_running = true;
    m_heartbeatTimer->start(300);
    qInfo().noquote() << QString("[CameraUDP] started localPort=%1 target=%2")
        .arg(m_serverPort)
        .arg(m_serverIP);
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("UDP已启动：监听端口 %1，目标相机 %2").arg(m_serverPort).arg(m_serverIP));
}

void ClientUDPFormSensorWorker::startReceiveDemux(int serverPort)
{
    stopReceive();

    m_serverIP = "按IP分发";
    m_serverPort = NormalizeUdpPort(serverPort);
    m_demuxMode = true;
    m_datagramCount = 0;
    m_filteredDatagramCount = 0;
    m_decodedFrameCount = 0;
    m_decodeFailedCount = 0;
    m_appendedFrameCount = 0;
    m_loggedFirstDatagram = false;
    m_loggedFirstFilteredDatagram = false;
    m_loggedFirstDecodedFrame = false;
    m_loggedFirstDecodeFailure = false;
    m_loggedFirstAppendedFrame = false;
    resetDispatchTargetState();

    m_udp = new QUdpSocket(this);
    m_udp->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    m_udp->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    if (!m_udp->bind(
        QHostAddress::AnyIPv4,
        m_serverPort,
        QAbstractSocket::ShareAddress | QAbstractSocket::ReuseAddressHint)) {
        qWarning().noquote() << QString("[CameraUDP] demux bind failed localPort=%1 error=%2")
            .arg(m_serverPort)
            .arg(m_udp->errorString());
        emit diagnosticChanged(
            m_datagramCount,
            m_filteredDatagramCount,
            m_decodedFrameCount,
            m_decodeFailedCount,
            m_appendedFrameCount,
            QString("UDP共享接收绑定失败：本地端口 %1").arg(m_serverPort));
        delete m_udp;
        m_udp = nullptr;
        return;
    }

    m_heartbeatTimer = new QTimer(this);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &ClientUDPFormSensorWorker::sendHeartbeat);
    connect(m_udp, &QUdpSocket::readyRead, this, &ClientUDPFormSensorWorker::readPendingDatagrams);

    resetReceiveState();
    m_running = true;
    m_heartbeatTimer->start(300);
    qInfo().noquote() << QString("[CameraUDP] demux started localPort=%1 targets=%2")
        .arg(m_serverPort)
        .arg(QStringList(m_dispatchTargets.keys()).join(','));
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("UDP共享接收已启动：监听端口 %1，按相机IP分发").arg(m_serverPort));

    for (auto it = m_dispatchTargets.constBegin(); it != m_dispatchTargets.constEnd(); ++it)
    {
        emit targetDiagnosticChanged(
            it.key(),
            0,
            0,
            0,
            0,
            0,
            QString("等待共享UDP端口 %1 的相机数据").arg(m_serverPort));
    }
}

// ===============================
// 完全清理（无锁）
// ===============================
void ClientUDPFormSensorWorker::stopReceive()
{
    const bool wasRunning = m_running || m_udp != nullptr || m_heartbeatTimer != nullptr;
    const QString stoppedIP = m_serverIP;
    const quint16 stoppedPort = m_serverPort;
    m_running = false;

    // 清理定时器
    if (m_heartbeatTimer) {
        m_heartbeatTimer->stop();
        m_heartbeatTimer->deleteLater();
        m_heartbeatTimer = nullptr;
    }

    // 清理UDP
    if (m_udp) {
        m_udp->close();
        m_udp->deleteLater();
        m_udp = nullptr;
    }

    // 清空状态
    resetReceiveState();
    m_serverIP.clear();
    m_serverPort = 0;
    m_demuxMode = false;
    if (wasRunning)
    {
        qInfo().noquote() << QString("[CameraUDP] stopped localPort=%1 target=%2")
            .arg(stoppedPort)
            .arg(stoppedIP);
    }
    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        "UDP已停止");
}

// ===============================
// 重置接收状态
// ===============================
void ClientUDPFormSensorWorker::resetReceiveState()
{
    m_receiveBuffer.clear();
    m_expectedSize = 0;
}

void ClientUDPFormSensorWorker::resetDispatchTargetState()
{
    for (auto it = m_dispatchTargets.begin(); it != m_dispatchTargets.end(); ++it)
    {
        it->receiveBuffer.clear();
        it->expectedSize = 0;
        it->datagramCount = 0;
        it->filteredDatagramCount = 0;
        it->decodedFrameCount = 0;
        it->decodeFailedCount = 0;
        it->appendedFrameCount = 0;
        it->loggedFirstDatagram = false;
        it->loggedFirstDecodedFrame = false;
        it->loggedFirstDecodeFailure = false;
        it->loggedFirstAppendedFrame = false;
    }
}

// ===============================
// 心跳（无锁）
// ===============================
void ClientUDPFormSensorWorker::sendHeartbeat()
{
    if (!m_running || !m_udp || m_serverPort == 0)
        return;

    if (m_demuxMode)
    {
        for (auto it = m_dispatchTargets.constBegin(); it != m_dispatchTargets.constEnd(); ++it)
        {
            m_udp->writeDatagram("HEARTBEAT", QHostAddress(it.key()), m_serverPort);
        }
        return;
    }

    if (m_serverIP.isEmpty())
    {
        return;
    }
    m_udp->writeDatagram("HEARTBEAT", QHostAddress(m_serverIP), m_serverPort);
}

void ClientUDPFormSensorWorker::processDemuxDatagram(
    const QByteArray& buf,
    const QHostAddress& senderAddress,
    quint16 senderPort)
{
    const QString senderKey = NormalizeAddressKey(senderAddress);
    auto targetIt = m_dispatchTargets.find(senderKey);
    if (targetIt == m_dispatchTargets.end())
    {
        ++m_filteredDatagramCount;
        if (!m_loggedFirstFilteredDatagram)
        {
            m_loggedFirstFilteredDatagram = true;
            qWarning().noquote() << QString("[CameraUDP] demux unknown sender localPort=%1 sender=%2:%3 bytes=%4 targets=%5")
                .arg(m_serverPort)
                .arg(senderAddress.toString())
                .arg(senderPort)
                .arg(buf.size())
                .arg(QStringList(m_dispatchTargets.keys()).join(','));
        }
        return;
    }

    DispatchTarget& target = targetIt.value();
    ++target.datagramCount;
    if (!target.loggedFirstDatagram)
    {
        target.loggedFirstDatagram = true;
        qInfo().noquote() << QString("[CameraUDP] demux first datagram localPort=%1 target=%2 sender=%3:%4 bytes=%5")
            .arg(m_serverPort)
            .arg(targetIt.key())
            .arg(senderAddress.toString())
            .arg(senderPort)
            .arg(buf.size());
    }

    target.receiveBuffer.append(buf);
    if (target.receiveBuffer.size() > MAX_RECEIVE_BUFFER_SIZE)
    {
        qWarning().noquote() << QString("[CameraUDP] demux receive buffer overflow target=%1 bytes=%2")
            .arg(targetIt.key())
            .arg(target.receiveBuffer.size());
        target.receiveBuffer.clear();
        target.expectedSize = 0;
        emit targetDiagnosticChanged(
            targetIt.key(),
            target.datagramCount,
            target.filteredDatagramCount,
            target.decodedFrameCount,
            target.decodeFailedCount,
            target.appendedFrameCount,
            QString("共享接收缓存溢出，已重置：%1").arg(targetIt.key()));
        return;
    }

    QByteArray frameBytes;
    bool decodedAnyFrame = false;
    while (TryTakeCompleteFrame(target.receiveBuffer, target.expectedSize, frameBytes))
    {
        decodedAnyFrame = true;
        PointCloundResultFrame frame;
        const bool ok = frame.fromByteArray(frameBytes);
        if (ok)
        {
            ++target.decodedFrameCount;
            if (!target.loggedFirstDecodedFrame)
            {
                target.loggedFirstDecodedFrame = true;
                qInfo().noquote() << QString("[CameraUDP] demux first decoded frame target=%1 localPort=%2 timestamp=%3 fps=%4 points=%5")
                    .arg(targetIt.key())
                    .arg(m_serverPort)
                    .arg(frame.timestamp)
                    .arg(frame.calcFrameRate)
                    .arg(frame.dataPoints3D.size());
            }

            if (target.frameCache != nullptr)
            {
                const udpDataShow udpFrame = BuildUdpFrame(frame);
                target.frameCache->AppendFrame(udpFrame);
                ++target.appendedFrameCount;
                if (!target.loggedFirstAppendedFrame)
                {
                    target.loggedFirstAppendedFrame = true;
                    qInfo().noquote() << QString("[CameraUDP] demux first frame appended target=%1 localPort=%2 timestamp=%3")
                        .arg(targetIt.key())
                        .arg(m_serverPort)
                        .arg(udpFrame.timestamp);
                }
            }
        }
        else
        {
            ++target.decodeFailedCount;
            if (!target.loggedFirstDecodeFailure)
            {
                target.loggedFirstDecodeFailure = true;
                qWarning().noquote() << QString("[CameraUDP] demux frame decode failed target=%1 localPort=%2 frameBytes=%3")
                    .arg(targetIt.key())
                    .arg(m_serverPort)
                    .arg(frameBytes.size());
            }
        }
    }

    if (!decodedAnyFrame)
    {
        emit targetDiagnosticChanged(
            targetIt.key(),
            target.datagramCount,
            target.filteredDatagramCount,
            target.decodedFrameCount,
            target.decodeFailedCount,
            target.appendedFrameCount,
            QString("共享接收中：等待完整帧 %1").arg(targetIt.key()));
        return;
    }
    emit targetDiagnosticChanged(
        targetIt.key(),
        target.datagramCount,
        target.filteredDatagramCount,
        target.decodedFrameCount,
        target.decodeFailedCount,
        target.appendedFrameCount,
        QString("共享UDP收包中：端口 %1，相机 %2").arg(m_serverPort).arg(targetIt.key()));
}

// ===============================
// 接收数据（无锁）
// ===============================
void ClientUDPFormSensorWorker::readPendingDatagrams()
{
    if (!m_running || !m_udp)
        return;

    while (m_udp->hasPendingDatagrams())
    {
        QByteArray buf;
        buf.resize(m_udp->pendingDatagramSize());
        QHostAddress senderAddress;
        quint16 senderPort = 0;
        m_udp->readDatagram(buf.data(), buf.size(), &senderAddress, &senderPort);
        ++m_datagramCount;
        if (m_demuxMode)
        {
            if (!m_loggedFirstDatagram)
            {
                m_loggedFirstDatagram = true;
                qInfo().noquote() << QString("[CameraUDP] demux first datagram localPort=%1 sender=%2:%3 bytes=%4")
                    .arg(m_serverPort)
                    .arg(senderAddress.toString())
                    .arg(senderPort)
                    .arg(buf.size());
            }
            processDemuxDatagram(buf, senderAddress, senderPort);
            continue;
        }
        if (!m_loggedFirstDatagram)
        {
            m_loggedFirstDatagram = true;
            qInfo().noquote() << QString("[CameraUDP] first datagram localPort=%1 target=%2 sender=%3:%4 bytes=%5")
                .arg(m_serverPort)
                .arg(m_serverIP)
                .arg(senderAddress.toString())
                .arg(senderPort)
                .arg(buf.size());
        }
        if (!IsExpectedSender(senderAddress, m_serverIP))
        {
            ++m_filteredDatagramCount;
            if (!m_loggedFirstFilteredDatagram)
            {
                m_loggedFirstFilteredDatagram = true;
                qWarning().noquote() << QString("[CameraUDP] datagram filtered localPort=%1 target=%2 sender=%3:%4 bytes=%5")
                    .arg(m_serverPort)
                    .arg(m_serverIP)
                    .arg(senderAddress.toString())
                    .arg(senderPort)
                    .arg(buf.size());
            }
            continue;
        }
        m_receiveBuffer.append(buf);
        if (m_receiveBuffer.size() > MAX_RECEIVE_BUFFER_SIZE)
        {
            qWarning() << "UDP receive buffer overflow, reset:" << m_receiveBuffer.size();
            resetReceiveState();
            continue;
        }

        QByteArray frameBytes;
        while (TryTakeCompleteFrame(m_receiveBuffer, m_expectedSize, frameBytes))
        {
            PointCloundResultFrame frame;
            const bool ok = frame.fromByteArray(frameBytes);

            if (ok)
            {
                ++m_decodedFrameCount;
                if (!m_loggedFirstDecodedFrame)
                {
                    m_loggedFirstDecodedFrame = true;
                    qInfo().noquote() << QString("[CameraUDP] first decoded frame target=%1 localPort=%2 timestamp=%3 fps=%4 points=%5 targetPoint=(%6,%7,%8)")
                        .arg(m_serverIP)
                        .arg(m_serverPort)
                        .arg(frame.timestamp)
                        .arg(frame.calcFrameRate)
                        .arg(frame.dataPoints3D.size())
                        .arg(frame.resultPoints3D.x, 0, 'f', 3)
                        .arg(frame.resultPoints3D.y, 0, 'f', 3)
                        .arg(frame.resultPoints3D.z, 0, 'f', 3);
                }
                const udpDataShow udpFrame = BuildUdpFrame(frame);

                if (m_frameCache != nullptr)
                {
                    m_frameCache->AppendFrame(udpFrame);
                    ++m_appendedFrameCount;
                    if (!m_loggedFirstAppendedFrame)
                    {
                        m_loggedFirstAppendedFrame = true;
                        qInfo().noquote() << QString("[CameraUDP] first frame appended target=%1 localPort=%2 timestamp=%3")
                            .arg(m_serverIP)
                            .arg(m_serverPort)
                            .arg(udpFrame.timestamp);
                    }
                }
            }
            else
            {
                ++m_decodeFailedCount;
                if (!m_loggedFirstDecodeFailure)
                {
                    m_loggedFirstDecodeFailure = true;
                    qWarning().noquote() << QString("[CameraUDP] frame decode failed target=%1 localPort=%2 frameBytes=%3")
                        .arg(m_serverIP)
                        .arg(m_serverPort)
                        .arg(frameBytes.size());
                }
            }
        }
    }

    emit diagnosticChanged(
        m_datagramCount,
        m_filteredDatagramCount,
        m_decodedFrameCount,
        m_decodeFailedCount,
        m_appendedFrameCount,
        QString("UDP收包中：监听端口 %1，目标相机 %2").arg(m_serverPort).arg(m_serverIP));
}
