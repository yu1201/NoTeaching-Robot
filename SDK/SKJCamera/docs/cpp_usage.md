# SKJCamera C++ 调用说明

本文档面向使用 C++ 调用 SKJCamera SDK 的业务程序。SDK 对外提供稳定的 C ABI，C++ 项目可以直接包含 `skjcamera.h` 调用；如果后续需要给 C#、Python 等语言封装，也建议以本文件中的句柄式调用流程为准。

## 1. 文件和库

发布给用户时，建议至少提供以下文件：

```text
include/
  skjcamera.h
  protocol.h
  SKJCamera_global.h
  tcp_client.h

bin/<arch>/<Debug|Release>/
  SKJCamera.dll

lib/<arch>/<Debug|Release>/
  SKJCamera.lib        # MSVC import library，若使用 MSVC 构建
  libSKJCamera.a       # MinGW import/static library，若使用 MinGW 构建
```

业务程序通常只需要包含：

```cpp
#include "skjcamera.h"
```

`protocol.h` 和 `tcp_client.h` 主要用于 SDK 内部、协议诊断或底层测试，普通业务代码不建议直接调用其中的接口。

## 2. 基本流程

典型调用顺序如下：

```text
SKJCamera_Create
  -> 可选：SKJCamera_SetLogCallback / SKJCamera_SetLogEnabled
  -> 可选：SKJCamera_SetConnectTimeout / SKJCamera_SetCommandTimeout
  -> SKJCamera_Connect
  -> 循环读取点云帧、读写参数、控制激光
  -> SKJCamera_Disconnect
  -> SKJCamera_Destroy
```

成对使用的接口：

| 创建/获取接口 | 释放/关闭接口 | 说明 |
| --- | --- | --- |
| `SKJCamera_Create()` | `SKJCamera_Destroy()` | 创建和销毁相机实例 |
| `SKJCamera_Connect()` | `SKJCamera_Disconnect()` | 建立和断开 TCP 连接 |
| `SKJCamera_GetLatestFrame()` | `SKJFrame_Release()` | 推荐的点云帧句柄接口 |
| `SKJCamera_GetLatestPointCloud()` | `SKJCamera_FreeFrame()` | C 风格结构体接口 |
| `SKJCamera_SetConnectTimeout()` | `SKJCamera_GetConnectTimeout()` | 连接超时配置 |
| `SKJCamera_SetCommandTimeout()` | `SKJCamera_GetCommandTimeout()` | 命令超时配置 |
| `SKJCamera_LaserOn()` | `SKJCamera_LaserOff()` | 激光开关 |

公开 API 速查：

| 分类 | 接口 |
| --- | --- |
| 设备搜索 | `SKJCamera_SearchDevices()`、`SKJCamera_SearchDevicesFlat()` |
| 生命周期 | `SKJCamera_Create()`、`SKJCamera_Destroy()` |
| 连接 | `SKJCamera_Connect()`、`SKJCamera_Disconnect()`、`SKJCamera_IsConnected()` |
| 点云帧句柄 | `SKJCamera_GetLatestFrame()`、`SKJFrame_Release()` |
| 点云帧数据 | `SKJFrame_GetChannel()`、`SKJFrame_GetTimestamp()`、`SKJFrame_GetPoint3DCount()`、`SKJFrame_GetPoint3DData()`、`SKJFrame_GetPoint2DCount()`、`SKJFrame_GetPoint2DData()`、`SKJFrame_GetResultPointValue()`、`SKJFrame_GetFpsImage()`、`SKJFrame_GetFpsPointCloud()`、`SKJFrame_GetErrorText()`、`SKJFrame_GetErrorLength()` |
| C 风格点云帧 | `SKJCamera_GetLatestPointCloud()`、`SKJCamera_FreeFrame()`、`PointCloudFrame.roi` |
| 超时配置 | `SKJCamera_SetConnectTimeout()`、`SKJCamera_GetConnectTimeout()`、`SKJCamera_SetCommandTimeout()`、`SKJCamera_GetCommandTimeout()` |
| 版本、日志和错误 | `SKJCamera_GetVersion()`、`SKJCamera_SetLogCallback()`、`SKJCamera_SetLogEnabled()`、`SKJCamera_GetErrorString()` |
| 参数和控制 | `SKJCamera_GetBinarize()`、`SKJCamera_SetBinarize()`、`SKJCamera_GetExposure()`、`SKJCamera_SetExposure()`、`SKJCamera_GetGain()`、`SKJCamera_SetGain()`、`SKJCamera_LaserOn()`、`SKJCamera_LaserOff()` |

## 3. 版本号

SDK 提供运行时版本函数：

```cpp
std::cout << "SKJCamera SDK version: " << SKJCamera_GetVersion() << std::endl;
```

`SKJCamera_GetVersion()` 返回静态 UTF-8 字符串，不需要创建相机句柄，调用方不要释放返回指针。
版本值保存在 DLL 内部实现中，因此用户只替换新版 DLL 时，不需要因为版本号变化重新编译 EXE。

## 4. 错误码处理

所有返回 `int` 的 SDK 接口通常返回：

- `SKJ_OK`：调用成功。
- `SKJ_ERR_INVALID_PARAM`：参数无效。
- `SKJ_ERR_NOT_CONNECTED`：未连接或连接已断开。
- `SKJ_ERR_CONNECT_FAIL`：连接失败。
- `SKJ_ERR_TIMEOUT`：等待超时。
- `SKJ_ERR_NO_DATA`：暂时没有点云帧。
- `SKJ_ERR_DATA_DUPLICATE`：最新帧已经返回过，尚未收到新帧。
- 其他 `SKJ_ERR_*`：详见 `protocol.h`。

可以使用 `SKJCamera_GetErrorString()` 获取错误码说明：

```cpp
int ret = SKJCamera_Connect(cam, "192.168.1.100", 50006);
if (ret != SKJ_OK) {
    std::cerr << "Connect failed: " << SKJCamera_GetErrorString(ret) << std::endl;
}
```

注意：`SKJ_ERR_NO_DATA` 和 `SKJ_ERR_DATA_DUPLICATE` 是读帧轮询时的正常状态，不一定表示异常。

## 5. 日志

SDK 日志分为：

- `SKJ_LOG_ERROR`：错误。无论是否启用默认日志，都会实时打印到控制台。
- `SKJ_LOG_WARNING`：警告。无论是否启用默认日志，都会实时打印到控制台。
- `SKJ_LOG_INFO`：普通信息。需要启用默认日志或设置日志回调。
- `SKJ_LOG_DEBUG`：调试信息。需要启用默认日志或设置日志回调。

日志策略：

- 主动操作失败或运行时异常会打印 `ERROR/WARNING`，例如连接失败、连接超时、命令超时、协议解析失败、缓冲区溢出。
- 轮询状态不会刷屏，例如 `SKJ_ERR_NO_DATA`、`SKJ_ERR_DATA_DUPLICATE`。

启用默认日志：

```cpp
SKJCamera_SetLogEnabled(1);
```

设置日志回调：

```cpp
void SKJCAMERA_CALL OnSdkLog(int level, const char *message, void *user_data)
{
    (void)user_data;
    std::cout << "[SDK][" << level << "] " << message << std::endl;
}

SKJCamera_SetLogCallback(OnSdkLog, nullptr);
```

建议在连接相机前设置日志回调，并在程序退出或不再需要时传入 `NULL` 清除：

```cpp
SKJCamera_SetLogCallback(NULL, NULL);
```

## 6. 搜索设备

### 6.1 C/C++ 数组接口

```cpp
char ips[16][64] = {};
int count = 0;

int ret = SKJCamera_SearchDevices(ips, 16, &count, 1000);
if (ret == SKJ_OK) {
    for (int i = 0; i < count; ++i) {
        std::cout << "Device " << i << ": " << ips[i] << std::endl;
    }
}
```

参数说明：

- `ip_list`：输出数组，每个元素至少 64 字节。
- `max_count`：最多写入多少个 IP。
- `out_count`：实际发现数量。
- `timeout_ms`：搜索超时时间，单位毫秒；传入 0 或负数使用默认值。

### 6.2 连续缓冲区接口

这个接口主要给 C#、Python 等语言绑定使用，C++ 中也可以调用：

```cpp
char buffer[16 * 64] = {};
int count = 0;

int ret = SKJCamera_SearchDevicesFlat(buffer, sizeof(buffer), 16, 64, &count, 1000);
if (ret == SKJ_OK) {
    for (int i = 0; i < count; ++i) {
        const char *ip = buffer + i * 64;
        std::cout << "Device " << i << ": " << ip << std::endl;
    }
}
```

## 7. 创建实例和连接相机

```cpp
SKJCameraHandle cam = SKJCamera_Create();
if (!cam) {
    std::cerr << "Create camera failed" << std::endl;
    return 1;
}

SKJCamera_SetConnectTimeout(cam, 3000);  // TCP 连接超时，默认 3000 ms
SKJCamera_SetCommandTimeout(cam, 1000);  // 参数/激光命令超时，默认 1000 ms

int ret = SKJCamera_Connect(cam, "192.168.1.100", 50006);
if (ret != SKJ_OK) {
    std::cerr << "Connect failed: " << SKJCamera_GetErrorString(ret) << std::endl;
    SKJCamera_Destroy(cam);
    return 1;
}
```

连接成功后，SDK 内部会启动接收线程并持续接收点云帧。

查询连接状态：

```cpp
if (SKJCamera_IsConnected(cam) == SKJ_OK) {
    std::cout << "camera connected" << std::endl;
}
```

读取当前超时配置：

```cpp
int connect_timeout = 0;
int command_timeout = 0;

SKJCamera_GetConnectTimeout(cam, &connect_timeout);
SKJCamera_GetCommandTimeout(cam, &command_timeout);

std::cout << "connect timeout=" << connect_timeout
          << ", command timeout=" << command_timeout << std::endl;
```

断开和销毁：

```cpp
SKJCamera_Disconnect(cam);
SKJCamera_Destroy(cam);
cam = nullptr;
```

如果忘记调用 `SKJCamera_Disconnect()`，`SKJCamera_Destroy()` 也会自动断开连接，但推荐业务代码显式断开。

## 8. 读取点云帧

### 8.1 推荐方式：句柄接口

推荐使用 `SKJCamera_GetLatestFrame()`。它返回不透明帧句柄，所有内部内存由 SDK 管理，调用方只需要在使用完后调用 `SKJFrame_Release()`。

```cpp
SKJFrameHandle frame = NULL;
int ret = SKJCamera_GetLatestFrame(cam, &frame);

if (ret == SKJ_OK) {
    int32_t channel = SKJFrame_GetChannel(frame);
    int64_t timestamp = SKJFrame_GetTimestamp(frame);
    int32_t count3d = SKJFrame_GetPoint3DCount(frame);
    const Point3d *points3d = SKJFrame_GetPoint3DData(frame);
    int32_t count2d = SKJFrame_GetPoint2DCount(frame);
    const Point2d *points2d = SKJFrame_GetPoint2DData(frame);

    std::cout << "channel=" << channel
              << ", timestamp=" << timestamp
              << ", point count=" << count3d << std::endl;

    for (int32_t i = 0; i < count3d; ++i) {
        const Point3d &p = points3d[i];
        std::cout << p.x << ", " << p.y << ", " << p.z << std::endl;
    }

    if (count2d > 0 && points2d) {
        std::cout << "first 2D point: "
                  << points2d[0].x << ", " << points2d[0].y << std::endl;
    }

    Point3d result{};
    if (SKJFrame_GetResultPointValue(frame, &result) == SKJ_OK) {
        std::cout << "result point: "
                  << result.x << ", " << result.y << ", " << result.z << std::endl;
    }

    std::cout << "fps image=" << SKJFrame_GetFpsImage(frame)
              << ", fps pointcloud=" << SKJFrame_GetFpsPointCloud(frame) << std::endl;

    const char *err = SKJFrame_GetErrorText(frame);
    int32_t err_len = SKJFrame_GetErrorLength(frame);
    if (err && err_len > 0) {
        std::cout << "frame error(" << err_len << " bytes): " << err << std::endl;
    }

    SKJFrame_Release(frame);
    frame = NULL;
} else if (ret == SKJ_ERR_NO_DATA || ret == SKJ_ERR_DATA_DUPLICATE) {
    // 正常轮询状态，稍后再读。
} else {
    std::cerr << "Get frame failed: " << SKJCamera_GetErrorString(ret) << std::endl;
}
```

重要注意事项：

- `SKJFrame_GetPoint3DData()`、`SKJFrame_GetPoint2DData()`、`SKJFrame_GetErrorText()` 返回的是只读指针。
- 这些指针只在 `SKJFrame_Release(frame)` 前有效。
- 不要对这些指针调用 `free()`、`delete` 或其他释放函数。

### 8.2 C 风格结构体接口

如果需要直接获得 `PointCloudFrame` 结构体，可以使用：

```cpp
PointCloudFrame frame;
int ret = SKJCamera_GetLatestPointCloud(cam, &frame);
if (ret == SKJ_OK) {
    for (int32_t i = 0; i < frame.point3d_count; ++i) {
        const Point3d &p = frame.points3d[i];
        std::cout << p.x << ", " << p.y << ", " << p.z << std::endl;
    }

    const SKJCameraROI &roi = frame.roi;
    std::cout << "ROI: x=" << roi.x
              << ", y=" << roi.y
              << ", width=" << roi.width
              << ", height=" << roi.height << std::endl;

    SKJCamera_FreeFrame(&frame);
} else if (ret == SKJ_ERR_NO_DATA || ret == SKJ_ERR_DATA_DUPLICATE) {
    // 正常轮询状态。
} else {
    std::cerr << "Get point cloud failed: " << SKJCamera_GetErrorString(ret) << std::endl;
    SKJCamera_FreeFrame(&frame);  // 出错后也可以安全调用。
}
```

该方式会深拷贝点云帧，适合纯 C/C++ 使用；跨语言绑定更推荐句柄接口。

## 9. 参数读写

### 9.1 二值化阈值

```cpp
int32_t binarize = 0;
int ret = SKJCamera_GetBinarize(cam, &binarize);
if (ret == SKJ_OK) {
    std::cout << "binarize=" << binarize << std::endl;
}

ret = SKJCamera_SetBinarize(cam, 120);
if (ret != SKJ_OK) {
    std::cerr << "Set binarize failed: " << SKJCamera_GetErrorString(ret) << std::endl;
}
```

### 9.2 曝光

```cpp
int32_t exposure = 0;
int ret = SKJCamera_GetExposure(cam, &exposure);
if (ret == SKJ_OK) {
    std::cout << "exposure=" << exposure << std::endl;
}

ret = SKJCamera_SetExposure(cam, 5000);
```

### 9.3 增益

```cpp
int32_t gain = 0;
int ret = SKJCamera_GetGain(cam, &gain);
if (ret == SKJ_OK) {
    std::cout << "gain=" << gain << std::endl;
}

ret = SKJCamera_SetGain(cam, 10);
```

参数接口都需要设备回复，受 `SKJCamera_SetCommandTimeout()` 控制。

## 10. 激光控制

```cpp
int ret = SKJCamera_LaserOn(cam);
if (ret != SKJ_OK) {
    std::cerr << "Laser on failed: " << SKJCamera_GetErrorString(ret) << std::endl;
}

// ...

ret = SKJCamera_LaserOff(cam);
if (ret != SKJ_OK) {
    std::cerr << "Laser off failed: " << SKJCamera_GetErrorString(ret) << std::endl;
}
```

建议在程序退出或相机断开前关闭激光，具体是否必须关闭取决于设备固件策略。

## 11. 完整示例

下面示例展示：搜索设备、创建实例、连接相机、读取 5 秒点云帧、关闭激光并释放资源。

```cpp
#include "skjcamera.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

static void SKJCAMERA_CALL OnSdkLog(int level, const char *message, void *user_data)
{
    (void)user_data;
    std::cerr << "[SKJCamera][" << level << "] " << message << std::endl;
}

static void PrintError(const char *action, int code)
{
    std::cerr << action << " failed: "
              << code << " (" << SKJCamera_GetErrorString(code) << ")"
              << std::endl;
}

int main()
{
    std::cout << "SKJCamera SDK version: " << SKJCamera_GetVersion() << std::endl;

    SKJCamera_SetLogCallback(OnSdkLog, nullptr);
    SKJCamera_SetLogEnabled(1);

    char ips[8][64] = {};
    int device_count = 0;
    int ret = SKJCamera_SearchDevices(ips, 8, &device_count, 1000);

    const char *ip = nullptr;
    if (ret == SKJ_OK && device_count > 0) {
        ip = ips[0];
        std::cout << "Use discovered device: " << ip << std::endl;
    } else {
        ip = "192.168.1.100";
        std::cout << "No device discovered, use manual IP: " << ip << std::endl;
    }

    SKJCameraHandle cam = SKJCamera_Create();
    if (!cam) {
        std::cerr << "Create camera failed" << std::endl;
        return 1;
    }

    SKJCamera_SetConnectTimeout(cam, 3000);
    SKJCamera_SetCommandTimeout(cam, 1000);

    ret = SKJCamera_Connect(cam, ip, 50006);
    if (ret != SKJ_OK) {
        PrintError("Connect", ret);
        SKJCamera_Destroy(cam);
        SKJCamera_SetLogCallback(NULL, NULL);
        return 1;
    }

    ret = SKJCamera_LaserOn(cam);
    if (ret != SKJ_OK) {
        PrintError("LaserOn", ret);
    }

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        SKJFrameHandle frame = NULL;
        ret = SKJCamera_GetLatestFrame(cam, &frame);

        if (ret == SKJ_OK) {
            int64_t timestamp = SKJFrame_GetTimestamp(frame);
            int32_t count = SKJFrame_GetPoint3DCount(frame);
            const Point3d *points = SKJFrame_GetPoint3DData(frame);

            std::cout << "Frame timestamp=" << timestamp
                      << ", 3D count=" << count;

            if (count > 0 && points) {
                std::cout << ", first point=("
                          << points[0].x << ", "
                          << points[0].y << ", "
                          << points[0].z << ")";
            }

            std::cout << std::endl;
            SKJFrame_Release(frame);
        } else if (ret == SKJ_ERR_NO_DATA || ret == SKJ_ERR_DATA_DUPLICATE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            PrintError("GetLatestFrame", ret);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    ret = SKJCamera_LaserOff(cam);
    if (ret != SKJ_OK) {
        PrintError("LaserOff", ret);
    }

    SKJCamera_Disconnect(cam);
    SKJCamera_Destroy(cam);
    SKJCamera_SetLogCallback(NULL, NULL);
    return 0;
}
```

## 12. 编译示例

### 12.1 MSVC

假设目录结构为：

```text
example/
  main.cpp
include/
  skjcamera.h ...
lib/x64/Release/
  SKJCamera.lib
bin/x64/Release/
  SKJCamera.dll
```

命令行示例：

```bat
cl /EHsc /Iinclude example\main.cpp /link /LIBPATH:lib\x64\Release SKJCamera.lib
```

运行前确保 `SKJCamera.dll` 在可执行文件同目录，或已加入 `PATH`。

### 12.2 MinGW

```bat
g++ -std=c++11 -Iinclude example\main.cpp -Llib\x64\Release -lSKJCamera -lws2_32 -o example.exe
```

运行前确保 `SKJCamera.dll` 在可执行文件同目录，或已加入 `PATH`。

## 13. 使用注意事项

- 一个 `SKJCameraHandle` 对应一个相机连接。多相机场景请为每台相机创建独立句柄。
- `SKJCamera_Connect()` 成功后 SDK 内部会启动接收线程。
- `SKJCamera_Disconnect()` 会停止接收线程、清空缓存帧和等待中的命令回复。
- 不要跨线程同时销毁同一个 `SKJCameraHandle`，建议由业务层统一管理生命周期。
- 日志回调建议在连接前设置，不建议在多个线程中频繁切换回调。
- 点云帧数据指针只在对应帧句柄释放前有效。
- 高频读取点云时，遇到 `SKJ_ERR_NO_DATA` 或 `SKJ_ERR_DATA_DUPLICATE` 后建议 sleep 5 到 10 ms，避免 CPU 空转。
- 如果业务程序是 C# 或 Python 调用 DLL，应优先绑定 `SKJCamera_GetLatestFrame()` 和 `SKJFrame_*` 访问器，不建议直接跨语言释放 `PointCloudFrame` 内部指针。
