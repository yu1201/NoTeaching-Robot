#ifndef SKJCAMERA_H
#define SKJCAMERA_H

#include "SKJCamera_global.h"
#include "protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file skjcamera.h
 * @brief SKJCamera 点云相机 SDK 的公开 C 接口。
 *
 * 普通业务程序只需要包含本头文件。接口使用稳定的 C ABI、固定调用约定和不透明句柄，
 * 便于 C/C++、Python ctypes/cffi、C# P/Invoke、Java JNA/JNI 等语言直接调用。
 *
 * 线程模型：
 * - SKJCamera_Connect() 成功后会启动 SDK 内部接收线程。
 * - SKJCamera_GetLatestFrame() 和 SKJCamera_GetLatestPointCloud() 从内部接收线程缓存中读取最新点云帧。
 * - 同一时间戳的帧只会成功返回一次；如果新帧尚未到达，重复读取会返回 SKJ_ERR_DATA_DUPLICATE。
 *
 * 内存和句柄所有权：
 * - SKJCamera_Create() 与 SKJCamera_Destroy() 成对使用。
 * - SKJCamera_Connect() 与 SKJCamera_Disconnect() 成对使用；SKJCamera_Destroy() 会自动断开连接。
 * - SKJCamera_GetLatestFrame() 与 SKJFrame_Release() 成对使用。
 * - SKJCamera_GetLatestPointCloud() 与 SKJCamera_FreeFrame() 成对使用。
 * - 不要使用 free()、delete、Marshal.FreeHGlobal() 或其他语言运行时内存接口释放 SDK 返回的指针。
 *
 * 跨语言调用建议：
 * - Python/C#/Java 等语言优先使用 SKJCamera_GetLatestFrame() 返回的 SKJFrameHandle。
 * - 通过 SKJFrame_GetPoint3DData()、SKJFrame_GetPoint2DData() 等访问数据。
 * - 所有由 SKJFrame_* 返回的只读指针仅在调用 SKJFrame_Release() 前有效。
 *
 * 典型流程：
 * @code
 * SKJCameraHandle cam = SKJCamera_Create();
 * int ret = SKJCamera_Connect(cam, "192.168.1.100", 50006);
 *
 * SKJFrameHandle frame = NULL;
 * ret = SKJCamera_GetLatestFrame(cam, &frame);
 * if (ret == SKJ_OK) {
 *     int32_t count = SKJFrame_GetPoint3DCount(frame);
 *     const Point3d *points = SKJFrame_GetPoint3DData(frame);
 *     // 在 SKJFrame_Release(frame) 前读取 points[0..count-1]。
 *     SKJFrame_Release(frame);
 * }
 *
 * SKJCamera_Destroy(cam);
 * @endcode
 */

/** 相机实例不透明结构体。调用方不要直接分配、释放或访问内部字段。 */
typedef struct SKJCamera SKJCamera;

/** 点云帧不透明结构体。由 SKJCamera_GetLatestFrame() 创建，由 SKJFrame_Release() 释放。 */
typedef struct SKJFrame SKJFrame;

/** 相机句柄类型，供 C 接口和跨语言绑定使用。 */
typedef SKJCamera *SKJCameraHandle;

/** 点云帧句柄类型，供 C 接口和跨语言绑定使用。 */
typedef SKJFrame  *SKJFrameHandle;

/**
 * @brief SDK 日志回调函数类型。
 *
 * message 为 UTF-8 字符串，仅在回调函数执行期间有效；如需异步保存，请在回调内自行复制。
 * 回调可能由 SDK 内部接收线程触发，建议回调函数只做轻量处理，避免在回调内调用阻塞式 SDK 接口。
 */
typedef void (SKJCAMERA_CALL *SKJCameraLogCallback)(int level,
        const char *message, void *user_data);

#define SKJ_LOG_ERROR   1 /**< 错误日志。无论是否启用日志或设置回调，SDK 都会立即打印到控制台。 */
#define SKJ_LOG_WARNING 2 /**< 警告日志。无论是否启用日志或设置回调，SDK 都会立即打印到控制台。 */
#define SKJ_LOG_INFO    3 /**< 普通信息日志。仅在启用默认日志或设置日志回调后输出。 */
#define SKJ_LOG_DEBUG   4 /**< 调试日志。仅在启用默认日志或设置日志回调后输出。 */

/**
 * @brief 搜索本地网络中的 SKJCamera 设备。
 *
 * 本函数监听 SDK 设备发现端口上的 UDP 广播包，不需要先创建相机句柄。
 *
 * @param ip_list    调用方提供的输出数组，每个元素必须至少能容纳 64 字节。
 * @param max_count  ip_list 最多可写入的 IP 数量。
 * @param out_count  输出实际发现的设备数量。
 * @param timeout_ms 搜索超时时间，单位毫秒；传入 0 或负数表示使用默认超时。
 * @return SKJ_OK 表示成功发现设备；SKJ_ERR_NO_DATA 表示超时内未发现设备；其他返回值见 SKJ_ERR_*。
 *
 * @note 跨语言调用时推荐使用 SKJCamera_SearchDevicesFlat()，该接口更容易声明输出缓冲区。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SearchDevices(char ip_list[][64],
        int max_count, int *out_count, int timeout_ms);

/**
 * @brief 使用一段连续缓冲区搜索设备，便于 Python/C#/Java 等语言调用。
 *
 * 第 index 个 IP 会写入 ip_buffer + index * ip_stride，内容为以 '\0' 结尾的 UTF-8 字符串。
 *
 * @param ip_buffer   调用方提供的输出缓冲区。
 * @param buffer_size ip_buffer 的总字节数。
 * @param max_count   最多写入的 IP 数量。
 * @param ip_stride   每个 IP 字符串预留的字节数，建议传 64。
 * @param out_count   输出实际发现的设备数量。
 * @param timeout_ms  搜索超时时间，单位毫秒；传入 0 或负数表示使用默认超时。
 * @return SKJ_OK 表示成功；SKJ_ERR_BUFFER_OVERFLOW 表示缓冲区容量不足；其他返回值见 SKJ_ERR_*。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SearchDevicesFlat(char *ip_buffer,
        int buffer_size, int max_count, int ip_stride, int *out_count, int timeout_ms);

/* ===== 生命周期接口 ===== */

/**
 * @brief 创建相机实例。
 *
 * @return 成功返回相机句柄；内存分配失败时返回 NULL。
 *
 * @note 本接口必须与 SKJCamera_Destroy() 成对使用。
 */
SKJCAMERA_API SKJCamera *SKJCAMERA_CALL SKJCamera_Create(void);

/**
 * @brief 销毁相机实例并释放内部资源。
 *
 * 如果相机仍处于连接状态，本函数会先自动调用断开逻辑。调用后 cam 不得再被使用。
 *
 * @param cam 相机句柄；允许传入 NULL。
 *
 * @note 本接口必须与 SKJCamera_Create() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL SKJCamera_Destroy(SKJCamera *cam);

/* ===== 连接接口 ===== */

/**
 * @brief 连接到相机服务器。
 *
 * 若当前句柄已经连接，本函数会先断开旧连接，再建立新连接。连接成功后 SDK 内部接收线程会立即启动。
 *
 * @param cam  相机句柄。
 * @param ip   相机或服务器 IPv4 地址，例如 "192.168.1.100"。
 * @param port 相机或服务器 TCP 端口。
 * @return SKJ_OK 表示连接成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口通常与 SKJCamera_Disconnect() 成对使用；SKJCamera_Destroy() 也会自动断开连接。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_Connect(SKJCamera *cam, const char *ip, int port);

/**
 * @brief 断开相机连接。
 *
 * 本函数会停止内部接收线程，清空缓存帧和等待中的命令回复，并关闭 socket。
 *
 * @param cam 相机句柄；允许传入 NULL。
 *
 * @note 本接口通常与 SKJCamera_Connect() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL SKJCamera_Disconnect(SKJCamera *cam);

/**
 * @brief 查询当前连接状态。
 *
 * @param cam 相机句柄。
 * @return 已连接返回 SKJ_OK；未连接返回 SKJ_ERR_NOT_CONNECTED；参数无效返回 SKJ_ERR_INVALID_PARAM。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_IsConnected(SKJCamera *cam);

/* ===== 点云帧接口 ===== */

/**
 * @brief 将最新点云帧深拷贝到调用方提供的 PointCloudFrame 结构体中。
 *
 * 本接口适合 C/C++ 直接使用。Python/C#/Java 等跨语言场景更推荐 SKJCamera_GetLatestFrame()，
 * 因为后者通过不透明句柄管理点云帧生命周期。
 *
 * @param cam 相机句柄。
 * @param out 输出点云帧；函数会先清零该结构体，再写入数据。
 * @return SKJ_OK 表示成功；
 *         SKJ_ERR_NO_DATA 表示尚未收到点云帧；
 *         SKJ_ERR_DATA_DUPLICATE 表示最新帧已经被取走；
 *         其他返回值见 SKJ_ERR_*。
 *
 * @note SKJ_OK 返回后，out 内部的 points3d、points2d、err 必须用 SKJCamera_FreeFrame() 释放。
 * @note 本接口与 SKJCamera_FreeFrame() 成对使用；出错后也可以安全调用 SKJCamera_FreeFrame(out)。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetLatestPointCloud(SKJCamera *cam, PointCloudFrame *out);

/**
 * @brief 释放 PointCloudFrame 内部由 SDK 分配的动态内存。
 *
 * 本函数只用于释放 SKJCamera_GetLatestPointCloud() 填充的结构体内部字段，不会释放 frame 指针本身。
 *
 * @param frame 点云帧结构体指针；允许传入 NULL。
 *
 * @note 本接口与 SKJCamera_GetLatestPointCloud() 成对使用。
 */
SKJCAMERA_API void SKJCAMERA_CALL SKJCamera_FreeFrame(PointCloudFrame *frame);

/**
 * @brief 获取最新点云帧的不透明句柄。
 *
 * 本接口是跨语言绑定推荐使用的点云读取方式。通过 SKJFrame_GetPoint3DData()、
 * SKJFrame_GetPoint2DData()、SKJFrame_GetErrorText() 获取的指针均为只读，
 * 且只在调用 SKJFrame_Release() 前有效。
 *
 * @param cam       相机句柄。
 * @param out_frame 输出点云帧句柄；成功时写入有效句柄，失败时写入 NULL。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJFrame_Release() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetLatestFrame(SKJCamera *cam,
        SKJFrame **out_frame);

/**
 * @brief 释放 SKJCamera_GetLatestFrame() 返回的点云帧句柄。
 *
 * @param frame 点云帧句柄；允许传入 NULL。
 *
 * @note 本接口与 SKJCamera_GetLatestFrame() 成对使用。释放后所有由该 frame 取得的数据指针立即失效。
 */
SKJCAMERA_API void SKJCAMERA_CALL SKJFrame_Release(SKJFrame *frame);

/** 获取点云帧通道号；frame 为 NULL 时返回 0。 */
SKJCAMERA_API int32_t SKJCAMERA_CALL SKJFrame_GetChannel(const SKJFrame *frame);

/** 获取点云帧时间戳，单位毫秒；frame 为 NULL 时返回 0。 */
SKJCAMERA_API int64_t SKJCAMERA_CALL SKJFrame_GetTimestamp(const SKJFrame *frame);

/** 获取 3D 点数量；frame 为 NULL 时返回 0。 */
SKJCAMERA_API int32_t SKJCAMERA_CALL SKJFrame_GetPoint3DCount(const SKJFrame *frame);

/**
 * @brief 获取只读 3D 点数组。
 *
 * 返回指针由 SDK 持有，不要释放或写入。该指针只在 SKJFrame_Release(frame) 前有效。
 */
SKJCAMERA_API const Point3d *SKJCAMERA_CALL SKJFrame_GetPoint3DData(const SKJFrame *frame);

/** 获取 2D 点数量；frame 为 NULL 时返回 0。 */
SKJCAMERA_API int32_t SKJCAMERA_CALL SKJFrame_GetPoint2DCount(const SKJFrame *frame);

/**
 * @brief 获取只读 2D 点数组。
 *
 * 返回指针由 SDK 持有，不要释放或写入。该指针只在 SKJFrame_Release(frame) 前有效。
 */
SKJCAMERA_API const Point2d *SKJCAMERA_CALL SKJFrame_GetPoint2DData(const SKJFrame *frame);

/**
 * @brief 复制点云帧中的计算结果点。
 *
 * @param frame     点云帧句柄。
 * @param out_point 输出结果点。
 * @return SKJ_OK 表示成功；参数无效返回 SKJ_ERR_INVALID_PARAM。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJFrame_GetResultPointValue(const SKJFrame *frame,
        Point3d *out_point);

/** 获取设备上报的图像帧率；frame 为 NULL 时返回 0.0。 */
SKJCAMERA_API float SKJCAMERA_CALL SKJFrame_GetFpsImage(const SKJFrame *frame);

/** 获取设备上报的点云处理帧率；frame 为 NULL 时返回 0.0。 */
SKJCAMERA_API float SKJCAMERA_CALL SKJFrame_GetFpsPointCloud(const SKJFrame *frame);

/**
 * @brief 获取点云帧中的错误文本。
 *
 * 返回 UTF-8 字符串指针；frame 为空或没有错误文本时返回空字符串。返回指针只在
 * SKJFrame_Release(frame) 前有效。
 */
SKJCAMERA_API const char *SKJCAMERA_CALL SKJFrame_GetErrorText(const SKJFrame *frame);

/** 获取错误文本长度，单位字节；frame 为 NULL 时返回 0。 */
SKJCAMERA_API int32_t SKJCAMERA_CALL SKJFrame_GetErrorLength(const SKJFrame *frame);

/* ===== 配置和诊断接口 ===== */

/**
 * @brief 获取 SDK 版本字符串。
 *
 * 返回静态 UTF-8 字符串，格式为 "major.minor.patch"。返回指针由 SDK 持有，调用方不要释放。
 * 该接口不需要创建相机句柄，可用于程序启动时记录 SDK 版本或跨语言运行时检查。
 */
SKJCAMERA_API const char *SKJCAMERA_CALL SKJCamera_GetVersion(void);

/**
 * @brief 设置 TCP 连接超时时间。
 *
 * 影响 SKJCamera_Connect() 建立 TCP 连接的最长等待时间。默认值为 3000 毫秒。
 *
 * @note 本接口与 SKJCamera_GetConnectTimeout() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SetConnectTimeout(SKJCamera *cam, int timeout_ms);

/**
 * @brief 读取当前 TCP 连接超时时间，单位毫秒。
 *
 * @note 本接口与 SKJCamera_SetConnectTimeout() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetConnectTimeout(SKJCamera *cam, int *out_timeout_ms);

/**
 * @brief 设置命令等待超时时间。
 *
 * 影响参数读写和激光开关等需要等待设备回复的接口。
 *
 * @note 本接口与 SKJCamera_GetCommandTimeout() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SetCommandTimeout(SKJCamera *cam, int timeout_ms);

/**
 * @brief 读取当前命令等待超时时间，单位毫秒。
 *
 * @note 本接口与 SKJCamera_SetCommandTimeout() 成对使用。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetCommandTimeout(SKJCamera *cam, int *out_timeout_ms);

/**
 * @brief 设置进程级 SDK 日志回调。
 *
 * 传入 NULL 可取消日志回调。错误和警告无论是否设置回调、是否启用默认日志，都会实时打印到控制台；
 * 设置回调后，错误和警告会同时进入回调。
 * 轮询状态类返回值（例如 SKJ_ERR_NO_DATA、SKJ_ERR_DATA_DUPLICATE）不会打印日志，避免高频读取时刷屏。
 *
 * @note 本接口设置的是进程级回调，不属于单个相机句柄。
 */
SKJCAMERA_API void SKJCAMERA_CALL SKJCamera_SetLogCallback(SKJCameraLogCallback callback,
        void *user_data);

/**
 * @brief 启用或关闭 SDK 默认日志输出。
 *
 * 默认日志关闭时，普通信息和调试日志不会输出到控制台；错误和警告仍会无条件输出到控制台。
 * 如果已经设置日志回调，回调仍会收到日志消息。
 * 日志策略为：主动操作失败和运行时异常打印 ERROR/WARNING；轮询状态返回值不打印。
 */
SKJCAMERA_API void SKJCAMERA_CALL SKJCamera_SetLogEnabled(int enabled);

/** 获取 SDK 错误码的静态说明字符串；返回指针由 SDK 持有，调用方不要释放。 */
SKJCAMERA_API const char *SKJCAMERA_CALL SKJCamera_GetErrorString(int code);

/* ===== 设备参数接口 ===== */

/**
 * @brief 读取当前二值化阈值。
 *
 * @param cam       相机句柄。
 * @param out_value 输出二值化阈值。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_SetBinarize() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetBinarize(SKJCamera *cam, int32_t *out_value);

/**
 * @brief 设置二值化阈值。
 *
 * @param cam   相机句柄。
 * @param value 要设置的二值化阈值。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_GetBinarize() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SetBinarize(SKJCamera *cam, int32_t value);

/**
 * @brief 读取当前曝光值。
 *
 * @param cam       相机句柄。
 * @param out_value 输出曝光值。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_SetExposure() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetExposure(SKJCamera *cam, int32_t *out_value);

/**
 * @brief 设置曝光值。
 *
 * @param cam   相机句柄。
 * @param value 要设置的曝光值。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_GetExposure() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SetExposure(SKJCamera *cam, int32_t value);

/**
 * @brief 读取当前增益值。
 *
 * @param cam       相机句柄。
 * @param out_value 输出增益值。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_SetGain() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_GetGain(SKJCamera *cam, int32_t *out_value);

/**
 * @brief 设置增益值。
 *
 * @param cam   相机句柄。
 * @param value 要设置的增益值。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_GetGain() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_SetGain(SKJCamera *cam, int32_t value);

/**
 * @brief 打开激光。
 *
 * @param cam 相机句柄。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_LaserOff() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_LaserOn(SKJCamera *cam);

/**
 * @brief 关闭激光。
 *
 * @param cam 相机句柄。
 * @return SKJ_OK 表示成功；失败时返回 SKJ_ERR_*。
 *
 * @note 本接口与 SKJCamera_LaserOn() 对应。
 */
SKJCAMERA_API int SKJCAMERA_CALL SKJCamera_LaserOff(SKJCamera *cam);

#ifdef __cplusplus
}
#endif

#endif // SKJCAMERA_H
