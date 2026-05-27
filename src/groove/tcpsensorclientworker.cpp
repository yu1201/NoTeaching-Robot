#include "tcpsensorclientworker.h"

#include <QDebug>
#include <QDataStream>
#include "src/groove/pointcloundresultframe.h"
#include "src/groove/threadsafebuffer.h"
#include "src/groove/framebuffer.h"

// 服务器端口（和你的 PointCloundTcpServer 对应）
#define TCP_SERVER_PORT       50006
#define HEARTBEAT_INTERVAL_MS 300

TcpSensorClientWorker::TcpSensorClientWorker(QObject* parent)
    : QObject(parent)
    , m_tcpSocket(nullptr)
    , m_heartBeatTimer(nullptr)
    , m_frameTotalSize(0)
    , m_serverPort(0)
    , m_isRunning(false)
    , m_lastTimestamp(0)
{
}

TcpSensorClientWorker::~TcpSensorClientWorker()
{
    stopClient();
}

void TcpSensorClientWorker::startClient(const QString& serverIP)
{
    stopClient();

    m_serverIP = serverIP;
    m_serverPort = TCP_SERVER_PORT;

    m_tcpSocket = new QTcpSocket(this);
    m_tcpSocket->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
    m_tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);

    connect(m_tcpSocket, &QTcpSocket::connected, this, &TcpSensorClientWorker::onSocketConnected);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &TcpSensorClientWorker::onSocketDisconnected);
    connect(m_tcpSocket, &QTcpSocket::readyRead, this, &TcpSensorClientWorker::onDataReceived);
    connect(m_tcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
        this, &TcpSensorClientWorker::onSocketErrorOccur);

    m_tcpSocket->connectToHost(m_serverIP, m_serverPort);

    //m_heartBeatTimer = new QTimer(this);
    //connect(m_heartBeatTimer, &QTimer::timeout, this, &TcpSensorClientWorker::sendHeartBeat);
    //m_heartBeatTimer->start(HEARTBEAT_INTERVAL_MS);

    resetReceiveState();
    m_isRunning = true;
}

void TcpSensorClientWorker::stopClient()
{
    m_isRunning = false;

    if (m_heartBeatTimer) {
        m_heartBeatTimer->stop();
        m_heartBeatTimer->deleteLater();
        m_heartBeatTimer = nullptr;
    }

    if (m_tcpSocket) {
        if (m_tcpSocket->state() != QAbstractSocket::UnconnectedState) {
            m_tcpSocket->disconnectFromHost();
            //m_tcpSocket->waitForDisconnected(1000);
        }
        m_tcpSocket->deleteLater();
        m_tcpSocket = nullptr;
    }

    resetReceiveState();
    m_serverIP.clear();
    m_serverPort = 0;
}

void TcpSensorClientWorker::resetReceiveState()
{
    m_recvBuffer.clear();
    m_frameTotalSize = 0;
}

void TcpSensorClientWorker::sendHeartBeat()
{
    if (!m_isRunning || !m_tcpSocket || m_serverIP.isEmpty())
        return;

    m_tcpSocket->write("HEARTBEAT");
}

void TcpSensorClientWorker::onDataReceived()
{
    if (!m_isRunning || !m_tcpSocket) return;

    m_recvBuffer.append(m_tcpSocket->readAll());

    while (true)
    {
        // 找帧头
        int framePos = PointCloundResultFrame::findFrameHeader(m_recvBuffer);
        if (framePos < 0) break;

        if (framePos > 0)
            m_recvBuffer.remove(0, framePos);

        if (m_recvBuffer.size() < 24) break;

        // 读取 QDataStream 格式的总长度
        quint32 totalSz;
        memcpy(&totalSz, m_recvBuffer.constData() + 8, 4);
        totalSz = qFromBigEndian(totalSz);

        if (totalSz < 24 || totalSz > 10 * 1024 * 1024 || m_recvBuffer.size() < (int)totalSz)
            break;

        QByteArray frameData = m_recvBuffer.left(totalSz);
        m_recvBuffer.remove(0, totalSz);

        // ==============================================
        // 这里是关键！！！
        // 跳过 QDataStream 自动加的 4 字节长度头！！！
        // ==============================================
        QByteArray realData = frameData;
        QDataStream s(realData);
        s.setByteOrder(QDataStream::BigEndian);

        quint32 magic, hdrSz, total;
        qint64 ts;
        qint32 ch;
        QByteArray body;
        quint32 cs;

        s >> magic >> hdrSz >> total >> ts >> ch >> body >> cs;

        // 直接构造正确数据
        PointCloundResultFrame frame;
        frame.magicNumber = magic;
        frame.headerSize = hdrSz;
        frame.totalSize = total;
        frame.timestamp = ts;
        frame.channel = ch;
        frame.checksum = cs;

        if (frame.deserializeBody(body))
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


            if (frame.resultPoints3D == (cv::Point3d(0, 0, 0))) {
                frame.resultPoints3D = cv::Point3d(NAN, NAN, NAN);
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

            ThreadSafeBuffer<udpDataShow>::Instance().enqueue(udpFrame);
            //qDebug() << "=====================================";
            qDebug() << "✅ 解析成功！点云数量：" << frame.dataPoints3D.size();
        }
        else
        {
            qDebug() << "❌ 解析失败！";
        }
    }
}

void TcpSensorClientWorker::onSocketConnected()
{
    qDebug() << "TCP 传感器客户端已连接：" << m_serverIP;
}

void TcpSensorClientWorker::onSocketDisconnected()
{
    qDebug() << "TCP 传感器客户端断开连接";
    stopClient();
}

void TcpSensorClientWorker::onSocketErrorOccur(QAbstractSocket::SocketError err)
{
    qDebug() << "TCP 错误：" << m_tcpSocket->errorString() << " 错误码：" << err;
    stopClient();
}