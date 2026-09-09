# findWeldingLine SDK（x64 Release）

构建环境：Visual Studio 2022 / MSVC v143 / C++17，动态运行库 `/MD`。

## 目录

- `bin/findWeldingLine.dll`：核心动态库。
- `lib/findWeldingLine.lib`：MSVC 导入库。
- `include/PointCloudExtration.h`：公开 C++ 接口及数据结构。
- `config/CorrugatedSheetPointCloudEctration.ini`：示例算法配置。
- `symbols/findWeldingLine.pdb`：Release 调试符号。
- `examples/smoke_test.cpp`、`bin/smoke_test.exe`：最小链接与 DLL 加载验证程序。
- `bin/` 中其余 DLL：当前库所需的 PCL、OpenCV 与 MSVC 运行时依赖。

## 接入

1. 调用方包含 `include/PointCloudExtration.h`，并定义 `POINTCLOUDEXTRATION_IMPORTS`；MSVC 请使用 `/utf-8` 编译选项。
2. 链接 `lib/findWeldingLine.lib`。
3. 将 `bin/` 内全部 DLL 放到调用程序 EXE 同目录，或将该目录加入 DLL 搜索路径。
4. 使用 `CorrugatedSheetPointCloudExtration` 返回的数组后，必须调用 `ReleaseTrackPoints` 释放。

该接口是 MSVC C++ ABI（导出名会修饰），调用方应使用兼容的 64 位 MSVC 工具链。

## 校验值

- `findWeldingLine.dll` SHA-256：`925AC6BF19762F76CF249C96A0FE873ABFA94151AC6636989C10759CE4A27432`
- `findWeldingLine.lib` SHA-256：`3D21729455915A10AC3EDD11ACB5C5449D81B1CB2F3C5AEA1881D599FEBC2C77`
