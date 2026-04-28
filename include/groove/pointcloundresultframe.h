#ifndef POINTCLOUNDRESULTFRAME_H
#define POINTCLOUNDRESULTFRAME_H

#include <QByteArray>
#include <QDataStream>
#include <QDebug>
#include <QtEndian>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

struct PointCloundResultFrame
{
    static const quint32 HEADER_SIZE = 24;
    static const quint32 MAX_PACKET_SIZE = 10 * 1024 * 1024;
    static const quint32 MAGIC_NUMBER = 0xABCDEF12;

    // 头部
    quint32 magicNumber = MAGIC_NUMBER;
    quint32 headerSize = HEADER_SIZE;
    quint32 totalSize = HEADER_SIZE + sizeof(quint32);
    qint64  timestamp = 0;
    qint32  channel = 0;

    // 业务数据
    cv::Mat                 binaryImage;
    std::vector<cv::Point3d> dataPoints3D;
    std::vector<cv::Point2d> calcPoints2D;
    cv::Point3d             resultPoints3D;
    QString                 errorMsg;
    float                   calcFrameRate = 0.0f;

    quint32 checksum = 0;

    // ------------------------------
    // 极速内存拷贝
    // ------------------------------
    QByteArray compressMat(const cv::Mat& mat) const
    {
        if (mat.empty())
            return {};

        QByteArray bytes;
        QDataStream s(&bytes, QIODevice::WriteOnly);
        s.setByteOrder(QDataStream::BigEndian);

        s << (qint32)mat.cols;
        s << (qint32)mat.rows;
        s << (qint32)mat.type();
        s << (qint32)mat.step;

        bytes.append((const char*)mat.data, mat.total() * mat.elemSize());
        return bytes;
    }

    // ------------------------------
    // 极速解压缩
    // ------------------------------
    cv::Mat decompressMat(const QByteArray& bytes) const
    {
        if (bytes.isEmpty())
            return {};

        QDataStream s(bytes);
        s.setByteOrder(QDataStream::BigEndian);

        qint32 w, h, type, step;
        s >> w >> h >> type >> step;

        QByteArray pixData = bytes.mid(4 * sizeof(qint32));
        cv::Mat mat(h, w, type, (void*)pixData.data(), step);

        return mat.clone();
    }

    // ------------------------------
    // CRC32 校验
    // ------------------------------
    quint32 calculateChecksum(const QByteArray& data) const
    {
        const quint32 polynomial = 0x04C11DB7;
        static quint32 table[256];
        static bool inited = false;

        if (!inited)
        {
            for (quint32 i = 0; i < 256; ++i)
            {
                quint32 crc = i << 24;
                for (int j = 0; j < 8; ++j)
                {
                    crc = (crc << 1) ^ ((crc & 0x80000000) ? polynomial : 0);
                }
                table[i] = crc;
            }
            inited = true;
        }

        quint32 crc = 0xFFFFFFFF;
        for (char ch : data)
        {
            quint8 byte = static_cast<quint8>(ch);
            crc = (crc << 8) ^ table[((crc >> 24) ^ byte) & 0xFF];
        }
        return crc ^ 0xFFFFFFFF;
    }

    // ------------------------------
    // 序列化主体
    // ------------------------------
    QByteArray serializeBody() const
    {
        QByteArray body;
        QDataStream s(&body, QIODevice::WriteOnly);
        s.setByteOrder(QDataStream::BigEndian);

        QByteArray imgBytes = compressMat(binaryImage);
        s << imgBytes;

        s << (quint32)dataPoints3D.size();
        for (const auto& p : dataPoints3D)
        {
            s << p.x << p.y << p.z;
        }

        s << (quint32)calcPoints2D.size();
        for (const auto& p : calcPoints2D)
        {
            s << p.x << p.y;
        }

        s << resultPoints3D.x << resultPoints3D.y << resultPoints3D.z;
        s << errorMsg;
        s << calcFrameRate;

        return body;
    }

    // ------------------------------
    // 反序列化主体
    // ------------------------------
    bool deserializeBody(const QByteArray& body)
    {
        QDataStream s(body);
        s.setByteOrder(QDataStream::BigEndian);

        QByteArray imgBytes;
        s >> imgBytes;
        binaryImage = decompressMat(imgBytes);

        quint32 cnt;
        s >> cnt;
        dataPoints3D.resize(cnt);
        for (quint32 i = 0; i < cnt; ++i)
        {
            s >> dataPoints3D[i].x >> dataPoints3D[i].y >> dataPoints3D[i].z;
        }

        s >> cnt;
        calcPoints2D.resize(cnt);
        for (quint32 i = 0; i < cnt; ++i)
        {
            s >> calcPoints2D[i].x >> calcPoints2D[i].y;
        }

        s >> resultPoints3D.x >> resultPoints3D.y >> resultPoints3D.z;
        s >> errorMsg;
        s >> calcFrameRate;

        return true;
    }

    // ------------------------------
    // 打包发送
    // ------------------------------
    QByteArray toByteArray()
    {
        QByteArray body = serializeBody();
        checksum = calculateChecksum(body);
        totalSize = headerSize + body.size() + sizeof(quint32);

        QByteArray packet;
        QDataStream s(&packet, QIODevice::WriteOnly);
        s.setByteOrder(QDataStream::BigEndian);

        s << magicNumber;
        s << headerSize;
        s << totalSize;
        s << timestamp;
        s << channel;
        s << body;
        s << checksum;

        return packet;
    }

    // ------------------------------
    // 从字节流解析
    // ------------------------------
    bool fromByteArray(const QByteArray& buffer)
    {
        if (buffer.size() < (int)HEADER_SIZE)
        {
            return false;
        }

        QDataStream s(buffer);
        s.setByteOrder(QDataStream::BigEndian);

        s >> magicNumber;
        if (magicNumber != MAGIC_NUMBER)
        {
            return false;
        }

        s >> headerSize;
        s >> totalSize;
        s >> timestamp;
        s >> channel;

        if (headerSize != HEADER_SIZE || totalSize > (quint32)buffer.size())
        {
            return false;
        }

        QByteArray body;
        s >> body;
        s >> checksum;

        if (calculateChecksum(body) != checksum)
        {
            return false;
        }

        return deserializeBody(body);
    }

    // ------------------------------
    // 查找帧头
    // ------------------------------
    static int findFrameHeader(const QByteArray& buffer)
    {
        const int sz = buffer.size();
        if (sz < (int)HEADER_SIZE)
        {
            return -1;
        }

        for (int i = 0; i <= sz - (int)HEADER_SIZE; ++i)
        {
            quint32 magic;
            memcpy(&magic, buffer.constData() + i, 4);
            magic = qFromBigEndian(magic);
            if (magic != MAGIC_NUMBER)
            {
                continue;
            }

            quint32 hdrSz;
            memcpy(&hdrSz, buffer.constData() + i + 4, 4);
            hdrSz = qFromBigEndian(hdrSz);
            if (hdrSz != HEADER_SIZE)
            {
                continue;
            }

            quint32 totalSz;
            memcpy(&totalSz, buffer.constData() + i + 8, 4);
            totalSz = qFromBigEndian(totalSz);
            if (totalSz < HEADER_SIZE + 4 || totalSz > MAX_PACKET_SIZE)
            {
                continue;
            }

            return i;
        }
        return -1;
    }
};

#endif // POINTCLOUNDRESULTFRAME_H
