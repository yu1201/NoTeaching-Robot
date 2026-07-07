# 图像获取与解码使用说明

SDK 内部接管了图像的接收与解码：连接图像服务器后，内部图像线程持续接收压缩图像
（JPEG/H264/H265），自动识别编码并用 FFmpeg 解码为 **BGR24**，缓存最新一帧。
业务程序只需轮询 `SKJCamera_GetLatestImage()` 拿到解码后的裸数据即可。

像素格式为 BGR24（8 位三通道、BGR 顺序、行连续），与 OpenCV 默认的 `CV_8UC3` 一致，
因此上层可以**零拷贝**地把它包装成 `cv::Mat` 或 `QImage` 继续处理。

## 1. 基本轮询流程

```cpp
#include "skjcamera.h"
#include <thread>
#include <chrono>

SKJCameraHandle cam = SKJCamera_Create();

// 图像连接独立于点云连接，可单独建立；端口通常为 50001。
int ret = SKJCamera_ConnectImage(cam, "192.168.1.100", 50001);
if (ret != SKJ_OK) {
    // 处理错误：SKJCamera_GetErrorString(ret)
}

for (;;) {
    SKJImageHandle img = NULL;
    ret = SKJCamera_GetLatestImage(cam, &img);

    if (ret == SKJ_OK) {
        int      w      = SKJImage_GetWidth(img);
        int      h      = SKJImage_GetHeight(img);
        int      stride = SKJImage_GetStride(img);   // BGR24 下为 w*3
        const uint8_t *data = SKJImage_GetData(img); // 只读，Release 前有效

        // ... 在此使用 data（见下方 cv::Mat / QImage 包装）...

        SKJImage_Release(img); // 用完必须释放；释放后 data 立即失效
    } else if (ret == SKJ_ERR_NO_DATA || ret == SKJ_ERR_DATA_DUPLICATE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 暂无新帧
    } else {
        break; // 其他错误
    }
}

SKJCamera_DisconnectImage(cam);
SKJCamera_Destroy(cam); // Destroy 也会自动断开图像连接
```

> **生命周期**：`SKJImage_GetData()` 返回的指针由该 `SKJImage` 句柄持有，仅在调用
> `SKJImage_Release()` 之前有效。若要在释放后继续使用，请自行拷贝，或对包装出的
> `cv::Mat` / `QImage` 调用 `.clone()` / `.copy()`。

## 2. 包装成 cv::Mat

`cv::Mat` 支持用外部缓冲区构造，不复制数据：

```cpp
#include <opencv2/opencv.hpp>

// 零拷贝：直接引用 SDK 的数据指针，注意 mat 不可在 SKJImage_Release 后使用
cv::Mat wrapToMat(SKJImageHandle img) {
    int w      = SKJImage_GetWidth(img);
    int h      = SKJImage_GetHeight(img);
    int stride = SKJImage_GetStride(img);
    void *data = const_cast<uint8_t *>(SKJImage_GetData(img));
    return cv::Mat(h, w, CV_8UC3, data, stride); // BGR24 == CV_8UC3(BGR)
}

// 用法：
SKJImageHandle img = NULL;
if (SKJCamera_GetLatestImage(cam, &img) == SKJ_OK) {
    cv::Mat view = wrapToMat(img);   // 零拷贝视图
    // 若需在 Release 后保留，请拷贝：
    cv::Mat owned = view.clone();
    SKJImage_Release(img);
    // owned 仍然可用
}
```

OpenCV 默认就是 BGR 通道顺序，无需做 `cvtColor`。

## 3. 包装成 QImage

`QImage` 同样支持引用外部缓冲区（Qt 5.14+ 提供 `Format_BGR888`，正好匹配 BGR24）：

```cpp
#include <QImage>

// 零拷贝：QImage 引用 SDK 数据指针，img 必须在 QImage 使用期间保持有效
QImage wrapToQImage(SKJImageHandle img) {
    int w      = SKJImage_GetWidth(img);
    int h      = SKJImage_GetHeight(img);
    int stride = SKJImage_GetStride(img);
    const uchar *data = SKJImage_GetData(img);
    return QImage(data, w, h, stride, QImage::Format_BGR888);
}

// 用法：
SKJImageHandle img = NULL;
if (SKJCamera_GetLatestImage(cam, &img) == SKJ_OK) {
    QImage view = wrapToQImage(img);
    // QImage 隐式共享，若要在 Release 后保留，请深拷贝：
    QImage owned = view.copy();
    SKJImage_Release(img);
    // owned 仍然可用（如用于 QPixmap::fromImage 显示）
}
```

> 若使用的 Qt 版本早于 5.14 没有 `Format_BGR888`，可改用
> `QImage::Format_RGB888` 并先做一次 BGR→RGB 交换（例如对 `cv::Mat` 调用
> `cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB)` 后再包装），或使用
> `rgbSwapped()`。

## 4. 错误码

| 返回码 | 含义 |
| --- | --- |
| `SKJ_OK` | 成功取到一帧新图像 |
| `SKJ_ERR_NO_DATA` | 尚未收到可用图像帧 |
| `SKJ_ERR_DATA_DUPLICATE` | 最新帧已被取走，暂无更新帧 |
| `SKJ_ERR_NOT_CONNECTED` | 图像连接未建立 |
| `SKJ_ERR_INVALID_PARAM` | 参数无效（如句柄为空） |

`SKJ_ERR_NO_DATA` 与 `SKJ_ERR_DATA_DUPLICATE` 属于轮询的正常状态，不会刷屏打印日志，
轮询时遇到这两个返回码 sleep 后重试即可。
