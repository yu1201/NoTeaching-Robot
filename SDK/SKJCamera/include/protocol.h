#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "SKJCamera_global.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file protocol.h
 * @brief SKJCamera 底层通信协议定义和解析工具。
 *
 * 普通业务程序建议包含 skjcamera.h，并通过 SKJCamera_* 接口访问设备。本头文件主要供
 * SDK 内部实现、协议诊断工具、自动化测试工具或需要直接处理协议帧的高级集成使用。
 *
 * 协议帧格式（小端字节序）：
 * - 4 字节帧头："SKJF"（十六进制 53 4B 4A 46）。
 * - 4 字节消息类型，uint32。
 * - 4 字节数据段长度，uint32。
 * - N 字节数据段。
 *
 * 长度字段只表示数据段字节数，不包含帧头、消息类型和长度字段本身。
 */

/* ===== SDK 通用返回码 ===== */
#define SKJ_OK                   0     /**< 成功 */
#define SKJ_ERR_INVALID_PARAM   (-100) /**< 参数无效，例如 NULL 指针、非法数值或输出缓冲区不可写 */
#define SKJ_ERR_NOT_CONNECTED   (-101) /**< 尚未连接设备或连接已经断开 */
#define SKJ_ERR_CONNECT_FAIL    (-102) /**< 连接设备失败 */
#define SKJ_ERR_INIT_FAIL       (-103) /**< SDK、socket、线程或其他内部资源初始化失败 */
#define SKJ_ERR_SEND_FAIL       (-104) /**< 数据发送失败 */
#define SKJ_ERR_TIMEOUT         (-105) /**< 等待设备回复或接收数据超时 */
#define SKJ_ERR_NO_DATA         (-106) /**< 暂无可读取的新数据 */
#define SKJ_ERR_DATA_DUPLICATE  (-107) /**< 当前最新帧已经返回过，尚未收到更新的数据帧 */
#define SKJ_ERR_PARSE_FAIL      (-108) /**< 协议帧、点云数据或命令回复解析失败 */
#define SKJ_ERR_REPLY_INVALID   (-109) /**< 命令回复格式无效或缺少必要字段 */
#define SKJ_ERR_REPLY_MISMATCH  (-110) /**< 收到的回复命令码与当前等待的命令不一致 */
#define SKJ_ERR_REPLY_FAIL      (-111) /**< 设备返回命令执行失败 */
#define SKJ_ERR_RECV_FAIL       (-112) /**< 数据接收失败 */
#define SKJ_ERR_BUFFER_OVERFLOW (-113) /**< 调用方缓冲区或 SDK 内部接收缓冲区容量不足 */

/* ===== 协议帧常量 ===== */
#define FRAME_HEADER_0   0x53  /**< 帧头字节 0：'S' */
#define FRAME_HEADER_1   0x4B  /**< 帧头字节 1：'K' */
#define FRAME_HEADER_2   0x4A  /**< 帧头字节 2：'J' */
#define FRAME_HEADER_3   0x46  /**< 帧头字节 3：'F' */
#define FRAME_HEADER_SIZE  4   /**< 帧头长度，单位字节 */
#define FRAME_TYPE_SIZE    4   /**< 消息类型字段长度，单位字节 */
#define FRAME_LENGTH_SIZE  4   /**< 数据段长度字段长度，单位字节 */
#define FRAME_HEAD_TOTAL   (FRAME_HEADER_SIZE + FRAME_TYPE_SIZE + FRAME_LENGTH_SIZE) /**< 固定帧头总长度，12 字节 */

/* ===== 协议消息类型 ===== */
#define MSG_TYPE_POINTCLOUD  0x01  /**< 点云数据帧 */
#define MSG_TYPE_CMD         0x02  /**< 命令请求帧 */
#define MSG_TYPE_CMD_REPLY   0x03  /**< 命令回复帧 */
#define MSG_TYPE_HEARTBEAT   0x04  /**< 心跳帧 */
#define MSG_TYPE_ERROR       0x05  /**< 设备错误消息帧 */

/* ===== 设备命令码 ===== */
#define CMD_GET_BINARIZE     0x10  /**< 获取二值化阈值 */
#define CMD_SET_BINARIZE     0x11  /**< 设置二值化阈值 */
#define CMD_GET_EXPOSURE     0x20  /**< 获取曝光值 */
#define CMD_SET_EXPOSURE     0x21  /**< 设置曝光值 */
#define CMD_GET_GAIN         0x30  /**< 获取增益值 */
#define CMD_SET_GAIN         0x31  /**< 设置增益值 */
#define CMD_LASER_ON         0x40  /**< 打开激光 */
#define CMD_LASER_OFF        0x41  /**< 关闭激光 */

/* ===== 命令回复结果码 ===== */
#define CMD_RESULT_OK        0x00  /**< 命令执行成功 */
#define CMD_RESULT_FAIL      0x01  /**< 命令执行失败 */

/** 设备坐标系下的 3D 点。 */
typedef struct
{
    double x; /**< X 坐标 */
    double y; /**< Y 坐标 */
    double z; /**< Z 坐标 */
} Point3d;

/** 图像坐标系下的 2D 点。 */
typedef struct
{
    double x; /**< X 坐标 */
    double y; /**< Y 坐标 */
} Point2d;

/**
 * @brief 图像坐标系下的 ROI 矩形框，单位为像素。
 *
 * ROI 由设备随点云数据帧上报，SDK 不提供单独的 ROI 获取或设置命令。
 * 调用方通过 SKJCamera_GetLatestPointCloud() 获得 PointCloudFrame 后读取 frame.roi。
 */
typedef struct
{
    int32_t x;      /**< ROI 左上角 X 坐标，单位像素 */
    int32_t y;      /**< ROI 左上角 Y 坐标，单位像素 */
    int32_t width;  /**< ROI 宽度，单位像素 */
    int32_t height; /**< ROI 高度，单位像素 */
} SKJCameraROI;

/**
 * @brief 点云帧的 C 结构体表示。
 *
 * points3d、points2d 和 err 均由 SDK 按需分配。释放方式取决于填充该结构体的接口：
 * - 由 SKJCamera_GetLatestPointCloud() 填充时，使用 SKJCamera_FreeFrame() 释放。
 * - 由 protocol_unpack_pointcloud() 填充时，使用 protocol_free_frame() 释放。
 *
 * 跨语言调用建议使用 skjcamera.h 中的 SKJCamera_GetLatestFrame() 和 SKJFrame_* 访问器，
 * 避免在不同语言运行时之间传递需要手动释放的结构体字段。
 */
typedef struct
{
    int32_t  channel;          /**< 通道号 */
    int64_t  timestamp;        /**< 时间戳，单位毫秒 */
    Point3d *points3d;         /**< 3D 点数组，由 SDK 分配；数量见 point3d_count */
    int32_t  point3d_count;    /**< 3D 点数量 */
    Point2d *points2d;         /**< 2D 点数组，由 SDK 分配；数量见 point2d_count */
    int32_t  point2d_count;    /**< 2D 点数量 */
    Point3d  result_point;     /**< 设备或算法输出的结果点 */
    float    fps_image;        /**< 图像帧率 */
    float    fps_pointcloud;   /**< 点云处理帧率 */
    SKJCameraROI roi;          /**< 设备随点云帧上报的 ROI 框，不需要单独发送 ROI 指令 */
    char    *err;              /**< UTF-8 错误文本，由 SDK 分配；无错误时可为 NULL */
    int32_t  err_len;          /**< 错误文本长度，单位字节，不包含字符串结尾 '\0' */
} PointCloudFrame;

/**
 * @brief 命令回复的数据结构。
 *
 * data 由 SDK 按需分配，使用 protocol_unpack_cmd_reply() 成功解析后，
 * 必须调用 protocol_free_cmd_reply() 释放。
 */
typedef struct
{
    uint8_t  cmd;          /**< 回复对应的命令码 */
    int      result_code;  /**< 命令执行结果，见 CMD_RESULT_* */
    uint8_t *data;         /**< 回复数据，由 SDK 分配；无数据时可为 NULL */
    int32_t  data_len;     /**< 回复数据长度，单位字节 */
} CmdReply;

/**
 * @brief 从接收缓冲区解析一帧完整协议数据。
 *
 * 解析成功后，已解析的帧会从 buf 中移除，剩余未解析数据会前移。body 输出的是新分配的数据段副本，
 * 调用方必须使用 protocol_free_buffer() 释放。
 *
 * @param buf      接收缓冲区；函数可能会原地移动其中的数据。
 * @param buf_len  输入/输出参数，传入 buf 中已有字节数，返回剩余未解析字节数。
 * @param msg_type 输出消息类型，见 MSG_TYPE_*。
 * @param body     输出数据段副本；如果调用方不需要数据段，可以传 NULL。
 * @param body_len 输出数据段长度，单位字节。
 * @return SKJ_OK 表示成功解析一帧；返回 1 表示数据不足需要继续接收；返回 SKJ_ERR_* 表示解析失败。
 *
 * @note 本接口与 protocol_free_buffer() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL protocol_parse_frame(uint8_t *buf, int32_t *buf_len,
        uint32_t *msg_type, uint8_t **body, int32_t *body_len);

/**
 * @brief 释放 protocol_parse_frame() 通过 body 返回的数据段缓冲区。
 *
 * @param ptr protocol_parse_frame() 返回的 body 指针；允许传入 NULL。
 *
 * @note 本接口与 protocol_parse_frame() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL protocol_free_buffer(void *ptr);

/**
 * @brief 将点云消息的数据段解析为 PointCloudFrame。
 *
 * 解析失败时，函数会自动清理已经分配的中间内存。解析成功后，out 内部动态字段由 SDK 分配，
 * 调用方必须使用 protocol_free_frame() 释放。
 *
 * @param body     点云消息数据段指针。
 * @param body_len 数据段长度，单位字节。
 * @param out      输出点云帧结构体。
 * @return SKJ_OK 表示解析成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 protocol_free_frame() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL protocol_unpack_pointcloud(const uint8_t *body, int32_t body_len,
        PointCloudFrame *out);

/**
 * @brief 释放 PointCloudFrame 内部的动态内存并将字段清零。
 *
 * 本函数释放 points3d、points2d、err 等由协议解析器分配的字段，不释放 frame 指针本身。
 *
 * @param frame 点云帧结构体指针；允许传入 NULL。
 *
 * @note 本接口与 protocol_unpack_pointcloud() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL protocol_free_frame(PointCloudFrame *frame);

/**
 * @brief 将设备命令打包为完整协议帧。
 *
 * 生成内容包括帧头、MSG_TYPE_CMD、数据段长度和数据段。data 只表示命令参数，
 * 不包含命令码本身；函数会自动把 cmd 写入数据段起始位置。
 *
 * @param cmd      命令码。
 * @param data     命令参数数据，不含命令码；无参数命令可传 NULL。
 * @param data_len 命令参数长度，单位字节。
 * @param out      调用方提供的输出缓冲区。
 * @param out_size 输出缓冲区大小，单位字节。
 * @return 返回非负数表示打包后的总字节数；失败时返回 SKJ_ERR_*。
 */
SKJCAMERA_API int SKJCAMERA_CALL protocol_pack_command(uint8_t cmd, const uint8_t *data, int data_len,
        uint8_t *out, int out_size);

/**
 * @brief 解析命令回复消息的数据段。
 *
 * 解析成功后，out->data 由 SDK 分配，调用方必须使用 protocol_free_cmd_reply() 释放。
 *
 * @param body     命令回复数据段指针。
 * @param body_len 数据段长度，单位字节。
 * @param out      输出命令回复结构体。
 * @return SKJ_OK 表示解析成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 protocol_free_cmd_reply() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL protocol_unpack_cmd_reply(const uint8_t *body, int32_t body_len,
        CmdReply *out);

/**
 * @brief 释放 CmdReply 内部的动态内存并将字段清零。
 *
 * @param reply 命令回复结构体指针；允许传入 NULL。
 *
 * @note 本接口与 protocol_unpack_cmd_reply() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL protocol_free_cmd_reply(CmdReply *reply);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
