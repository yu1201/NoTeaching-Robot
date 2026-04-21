#include "groove/clientudpformsensorworker.h"
#include <QDebug>
#include <QDataStream>
#include <QVariant>
#include "groove/pointcloundresultframe.h"
#include "groove/threadsafebuffer.h"
#include "groove/framebuffer.h"

#define UDP_SENDER_PORT 50005

ClientUDPFormSensorWorker::ClientUDPFormSensorWorker(QObject* parent)
    : QObject(parent)
    , m_udp(nullptr)
    , m_heartbeatTimer(nullptr)
    , m_expectedSize(0)
    , m_serverPort(0)
    , m_running(false)
{
}

ClientUDPFormSensorWorker::~ClientUDPFormSensorWorker()
{
    stopReceive();
}

// ===============================
// 全新启动（无锁）
// ===============================
void ClientUDPFormSensorWorker::startReceive(const QString& serverIP)
{
    // 先完全清理
    stopReceive();

    m_serverIP = serverIP;
    m_serverPort = UDP_SENDER_PORT;

    // 新建UDP
    m_udp = new QUdpSocket(this);
    m_udp->setSocketOption(QAbstractSocket::KeepAliveOption, QVariant(1));
    m_udp->setSocketOption(QAbstractSocket::LowDelayOption, QVariant(1));

    if (!m_udp->bind(QHostAddress::AnyIPv4, UDP_SENDER_PORT)) {
        qDebug() << "UDP 绑定失败:" << UDP_SENDER_PORT;
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
}

// ===============================
// 完全清理（无锁）
// ===============================
void ClientUDPFormSensorWorker::stopReceive()
{
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
}

// ===============================
// 重置接收状态
// ===============================
void ClientUDPFormSensorWorker::resetReceiveState()
{
    m_receiveBuffer.clear();
    m_expectedSize = 0;
}

// ===============================
// 心跳（无锁）
// ===============================
void ClientUDPFormSensorWorker::sendHeartbeat()
{
    if (!m_running || !m_udp || m_serverIP.isEmpty() || m_serverPort == 0)
        return;

    m_udp->writeDatagram("HEARTBEAT", QHostAddress(m_serverIP), m_serverPort);
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
        m_udp->readDatagram(buf.data(), buf.size());
        m_receiveBuffer.append(buf);

        if (m_expectedSize <= 0)
        {
            if (m_receiveBuffer.size() >= (int)PointCloundResultFrame::HEADER_SIZE)
            {
                QDataStream s(m_receiveBuffer);
                s.setByteOrder(QDataStream::BigEndian);

                quint32 magic, hdrSz, totalSz;
                s >> magic >> hdrSz >> totalSz;

                if (magic == PointCloundResultFrame::MAGIC_NUMBER &&
                    hdrSz == PointCloundResultFrame::HEADER_SIZE)
                {
                    m_expectedSize = totalSz;
                }
                else
                {
                    m_receiveBuffer.remove(0, 1);
                    continue;
                }
            }
            else
            {
                continue;
            }
        }

        if (m_receiveBuffer.size() < m_expectedSize)
            continue;

        PointCloundResultFrame frame;
        bool ok = frame.fromByteArray(m_receiveBuffer);

        if (ok)
        {
            QVector<double> pointCloundX, pointCloundY;
            QVector<double> fitLineX, fitLineY;
            QVector<double> targetX, targetY;

            for (cv::Point3d p : frame.dataPoints3D) {
                pointCloundX.append(p.y);
                pointCloundY.append(p.z);
            }
            for (cv::Point2d p : frame.calcPoints2D) {
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

            ThreadSafeBuffer<udpDataShow>::Instance().enqueue(udpFrame);
        }

        m_receiveBuffer.clear();
        m_expectedSize = 0;
    }
}
