#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "protocol.h"
#include "SKJCamera_global.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file tcp_client.h
 * @brief SKJCamera SDK 内部 TCP 客户端工具。
 *
 * 普通业务程序不需要直接包含本头文件，应优先使用 skjcamera.h 中的 SKJCamera_* 接口。
 * 本头文件保留给 SDK 内部实现、底层诊断和确实需要直接访问 socket 的高级集成。
 *
 * 该实现同时支持 Windows Winsock 和 POSIX socket。
 */

/** TCP 连接状态。 */
typedef enum
{
    TCP_STATE_IDLE,       /**< 空闲状态，尚未初始化或已经销毁 */
    TCP_STATE_CONNECTING, /**< 正在连接 */
    TCP_STATE_CONNECTED,  /**< 已连接 */
    TCP_STATE_ERROR       /**< 错误状态 */
} TcpState;

/**
 * @brief TCP 客户端上下文。
 *
 * 调用方不要直接修改字段。使用顺序为：
 * tcp_client_init() -> tcp_client_connect() -> tcp_client_send_all()/tcp_client_recv() -> tcp_client_destroy()。
 */
typedef struct
{
    void     *sock_fd;       /**< 内部 socket 句柄，使用 void* 屏蔽平台差异 */
    TcpState  state;         /**< 当前连接状态 */
    char      server_ip[64]; /**< 服务器 IP 地址字符串 */
    uint16_t  server_port;   /**< 服务器端口号 */
} TcpClient;

/**
 * @brief 初始化 TCP 客户端上下文。
 *
 * @param client TCP 客户端上下文指针。
 * @param ip     服务器 IPv4 地址字符串。
 * @param port   服务器端口号。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口必须在 tcp_client_connect() 前调用。
 * @note 在 Windows 上，首次初始化时会同时初始化 Winsock。
 */
SKJCAMERA_API int SKJCAMERA_CALL tcp_client_init(TcpClient *client, const char *ip, uint16_t port);

/**
 * @brief 建立 TCP 连接。
 *
 * 连接成功后会启用 TCP_NODELAY，降低命令交互延迟。
 *
 * @param client 已通过 tcp_client_init() 初始化的 TCP 客户端上下文。
 * @param timeout_ms 连接超时时间，单位毫秒；必须大于 0。
 * @return SKJ_OK 表示连接成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 tcp_client_destroy() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL tcp_client_connect(TcpClient *client, int timeout_ms);

/**
 * @brief 发送完整缓冲区数据。
 *
 * 本函数会循环发送，直到 len 字节全部发送完成或发生错误。
 *
 * @param client 已连接的 TCP 客户端上下文。
 * @param data   待发送数据指针。
 * @param len    待发送数据长度，单位字节。
 * @return SKJ_OK 表示全部发送成功；失败时返回 SKJ_ERR_*。
 */
SKJCAMERA_API int SKJCAMERA_CALL tcp_client_send_all(TcpClient *client, const uint8_t *data, size_t len);

/**
 * @brief 按超时时间接收 TCP 数据。
 *
 * @param client     已连接的 TCP 客户端上下文。
 * @param buf        调用方提供的接收缓冲区。
 * @param buf_size   接收缓冲区大小，单位字节。
 * @param timeout_ms 接收超时时间，单位毫秒。
 * @return 正数表示实际接收字节数；0 表示本次等待超时；负数为 SKJ_ERR_*。
 */
SKJCAMERA_API int SKJCAMERA_CALL tcp_client_recv(TcpClient *client, uint8_t *buf, size_t buf_size, int timeout_ms);

/**
 * @brief 关闭 socket 并重置 TCP 客户端上下文。
 *
 * @param client TCP 客户端上下文指针；允许重复调用。
 *
 * @note 本接口与 tcp_client_connect() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL tcp_client_destroy(TcpClient *client);

#ifdef __cplusplus
}
#endif

#endif // TCP_CLIENT_H
