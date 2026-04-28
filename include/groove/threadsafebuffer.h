#ifndef THREADSAFEBUFFER_H
#define THREADSAFEBUFFER_H

#include <QMutex>
#include <QMutexLocker>
#include <QQueue>
#include <QWaitCondition>

template <typename T>
class ThreadSafeBuffer
{
public:
    // 全局单例入口
    static ThreadSafeBuffer<T>& Instance() {
        static ThreadSafeBuffer<T> ins(2);  // 默认长度2
        return ins;
    }

    // 禁止拷贝、禁止赋值
    ThreadSafeBuffer(const ThreadSafeBuffer&) = delete;
    ThreadSafeBuffer& operator=(const ThreadSafeBuffer&) = delete;

    // 设置最大缓冲长度
    void setMaxSize(int maxSize) {
        QMutexLocker locker(&m_mutex);
        if (maxSize >= 1) {
            m_maxSize = maxSize;

            // 如果当前队列超过新长度，自动移除旧数据
            while (m_queue.size() > m_maxSize) {
                m_queue.dequeue();
            }
        }
    }

    // 添加帧到缓冲区
    void enqueue(const T& frame) {
        QMutexLocker locker(&m_mutex);

        // 如果缓冲区已满，移除最早的帧
        while (m_queue.size() >= m_maxSize) {
            m_queue.dequeue();
        }

        m_queue.enqueue(frame);
        m_notEmpty.wakeOne();
    }

    bool front(T& outFrame) const {
        QMutexLocker locker(&m_mutex);
        if (m_queue.isEmpty()) {
            return false;
        }
        outFrame = m_queue.head();
        return true;
    }

    // 从缓冲区获取帧（出队）
    bool dequeue(T& frame, int timeout = -1) {
        QMutexLocker locker(&m_mutex);

        // 等待有数据可用
        if (m_queue.isEmpty()) {
            if (!m_notEmpty.wait(&m_mutex, timeout)) {
                return false;
            }
        }

        if (!m_queue.isEmpty()) {
            frame = m_queue.dequeue();
            return true;
        }

        return false;
    }

    // 清空缓冲区
    void clear() {
        QMutexLocker locker(&m_mutex);
        m_queue.clear();
    }

    // 获取缓冲区当前大小
    int size() const {
        QMutexLocker locker(&m_mutex);
        return m_queue.size();
    }

private:
    // 私有化构造（单例）
    explicit ThreadSafeBuffer(int maxSize)
        : m_maxSize(maxSize) {
    }

private:
    QQueue<T> m_queue;
    mutable QMutex m_mutex;
    QWaitCondition m_notEmpty;
    int m_maxSize;
};

#endif // THREADSAFEBUFFER_H
