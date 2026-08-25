# QtWidgetsApplication4 代码环境搭建说明书

- 适用项目：`NoTeaching-Robot / QtWidgetsApplication4`
- 适用平台：Windows 10/11 x64
- 工程配置：`Debug|x64`、`Release|x64`
- 整理日期：`2026-08-19`

## 1. 目标与边界

本说明用于在一台新的 Windows 开发机上完成源码克隆、第三方工具链安装、路径配置、环境检查和本地编译。

本说明和配套包不执行以下操作：

- 不修改服务器、机器人、相机、FTP、OTA 或防火墙；
- 不保存或分发密码、令牌、SSH 私钥、签名密钥和现场数据库；
- 不代替正式安装包、OTA、GitHub/GitLab 发布门禁；
- 轻量源码环境配套包不分发大型依赖；如目标机器没有 Qt、OpenCV、Eigen、Orocos KDL 或 OpenCASCADE，可另用[第三方依赖离线包](third-party-offline-package.md)。Visual Studio 和 Git 仍需单独安装。

主工程需要的 STEP、点云、相机和 BCPD 项目 SDK 文件大部分已在 Git 仓库中。第三方通用工具链需按本文安装。

## 2. 已验证的工具链基线

| 组件 | 固定版本/要求 | 当前验证路径 | 用途 |
| --- | --- | --- | --- |
| Visual Studio | 2022，MSVC v143，x64 C++，Windows SDK | `C:\Program Files\Microsoft Visual Studio\2022\Professional` | C++ 编译与链接 |
| MSBuild | 17.14 | VS 2022 内置 | 命令行构建 |
| Qt | 6.7.3，`msvc2022_64`，Debug+Release | `E:\workspace\soft\QT\6.7.3\msvc2022_64` | GUI、Network、SQL、OpenGL |
| Qt VS Tools/QtMsBuild | 与 VS 2022 配套 | `%LOCALAPPDATA%\QtMsBuild` | moc/uic/rcc 与 VS 工程集成 |
| OpenCV | 4.6.0，`opencv_world460[d]` | `E:\OpenCV4.6.0\build` | 图像与几何计算 |
| Eigen | 3.4.0 | `E:\Eigen3.4\eigen-3.4.0` | 头文件数学库 |
| Orocos KDL | 1.5.3，Debug+Release 静态库 | `C:\Program Files\orocos_kdl` | 机器人运动学 |
| vcpkg | 已验证程序版本 `2026-03-04-4b3e4c...` | `E:\vcpkg` | OpenCASCADE 安装根 |
| OpenCASCADE | **必须精确为 7.9.3**，`x64-windows`，Debug+Release | `E:\vcpkg\installed\x64-windows` | STEP/B-Rep/网格读取 |
| C++ 标准 | C++17 / C17 | 工程文件固定 | 编译标准 |

不要在未验证的情况下升级任一 ABI 相关组件。尤其是 Qt、OpenCV、KDL、OpenCASCADE 和 MSVC 工具集；版本或编译器不一致可能在链接阶段失败，或产生运行期 ABI 问题。

## 3. 仓库中已经包含的文件

完整克隆后，至少应存在：

- `SDK/STEP/Robot-SDK.lib`、`SDK/STEP/Robot-SDKd.lib` 和三个 STEP 头文件；
- `SDK/PointCloudExtration/PointCloudExtration.dll/.lib`、配置及其运行时依赖；
- `SDK/SKJCamera/include`、`lib/x64`、`bin/x64`；
- `SDK/BCPD/bcpd.exe` 与 `LICENSE.md`；
- `SDK/FANUC` 下 Git 跟踪的 KAREL/LS 源文件。

注意：

- `SDK/STEP/versions/*/*.ARM.zip` 是 STEP 控制器系统升级包，不是 Windows 编译依赖；
- `SDK/FANUC/*.tp`、`*.pc` 是被 `.gitignore` 排除的现场/正式打包运行文件，普通源码编译不依赖它们；
- 正式安装包构建必须另行核对权威 `.tp/.pc` 文件，不能把本配套包当成正式发版输入。

## 4. 安装顺序

### 4.1 Visual Studio 2022

通过 Visual Studio Installer 安装“使用 C++ 的桌面开发”，至少包含：

- MSVC v143 x64/x86 build tools；
- Windows 10 或 Windows 11 SDK；
- C++ ATL/MFC 并非主工程硬要求，但已有团队环境可一并保留；
- Qt Visual Studio Tools 扩展。

安装后应能找到：

```text
C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe
<VS安装目录>\MSBuild\Current\Bin\MSBuild.exe
```

### 4.2 Git 与源码

建议克隆到无中文、无特殊字符、路径不过长的目录，例如：

```powershell
git clone <项目仓库地址> E:\WorkFile\bowen\QtWidgetsApplication4
Set-Location E:\WorkFile\bowen\QtWidgetsApplication4
git status --short --branch
```

仓库目前没有 `.gitmodules`，不需要额外初始化 Git submodule。

### 4.3 Qt 6.7.3

安装 Qt `6.7.3 / MSVC 2022 64-bit`，并确认同时具有 Debug 和 Release 库。必需模块为：

```text
Core, Gui, Widgets, Network, Sql, OpenGL, OpenGLWidgets
```

关键文件包括：

```text
bin\qmake.exe
bin\windeployqt.exe
lib\Qt6Core.lib
lib\Qt6Cored.lib
plugins\sqldrivers\qsqlite.dll
plugins\tls\qschannelbackend.dll
```

使用 Visual Studio 图形界面时，可在 Qt VS Tools 中把该安装登记为 `qt6.7`。本项目的命令行构建脚本会直接传入 Qt 根目录，不依赖注册名。

### 4.4 OpenCV 4.6.0

项目当前固定链接：

```text
x64\vc15\lib\opencv_world460.lib
x64\vc15\lib\opencv_world460d.lib
```

并在构建后复制对应 DLL。若使用自编译 OpenCV，必须保持同名的 Debug/Release x64 库以及兼容的 MSVC ABI。

### 4.5 Eigen 3.4.0

Eigen 为头文件库。配置根目录下应直接存在：

```text
Eigen\Core
Eigen\src\Core\util\Macros.h
```

### 4.6 Orocos KDL 1.5.3

必须提供：

```text
include\kdl\config.h
lib\orocos-kdl.lib
lib\orocos-kdld.lib
```

Debug 工程链接 `orocos-kdld.lib`，Release 工程链接 `orocos-kdl.lib`。

### 4.7 vcpkg 与 OpenCASCADE 7.9.3

工程要求 OpenCASCADE 版本**精确为 7.9.3**。已验证的安装布局是：

```text
E:\vcpkg\installed\x64-windows\include\opencascade
E:\vcpkg\installed\x64-windows\lib
E:\vcpkg\installed\x64-windows\debug\lib
E:\vcpkg\installed\x64-windows\bin
E:\vcpkg\installed\x64-windows\debug\bin
E:\vcpkg\installed\x64-windows\share\opencascade\copyright
```

安装命令形式为：

```powershell
E:\vcpkg\vcpkg.exe install "opencascade[core]:x64-windows"
```

由于 vcpkg 端口版本会变化，安装后必须运行配套检查脚本确认得到的是 7.9.3；不是 7.9.3 时应使用团队已验证的 vcpkg 快照或二进制缓存，不要修改工程绕过版本门禁。

## 5. 配置本机路径

工程支持两种覆盖方式，优先推荐本机私有属性文件。

### 5.1 Visual Studio 与 MSBuild 通用配置

在仓库根目录执行：

```powershell
Copy-Item tools\source_environment\environment.local.props.example environment.local.props
notepad environment.local.props
```

按本机实际路径修改。`environment.local.props` 已被 `.gitignore` 排除，不应提交。

工程保留当前机器路径作为兼容默认值；存在 `environment.local.props` 时会在工程加载最前面读取，并覆盖默认路径。

### 5.2 配套脚本配置

```powershell
Copy-Item tools\source_environment\environment.example.psd1 tools\source_environment\environment.local.psd1
notepad tools\source_environment\environment.local.psd1
```

该文件同样已被 `.gitignore` 排除。`MSBuildExecutable`、`QtMsBuild`、`OcctRoot` 留空时，脚本会按 VS Installer、`%LOCALAPPDATA%` 和 `VcpkgRoot` 自动推导。

## 6. 环境检查

在仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\check_environment.ps1
```

脚本只读检查：

- VS/MSBuild、Windows SDK、Git；
- Qt/QtMsBuild、OpenCV、Eigen、KDL 版本与关键文件；
- OpenCASCADE 7.9.3 的 Debug/Release 头文件、库、DLL 和许可证；
- 工程自带 STEP、点云、SKJCamera、BCPD 文件；
- Git 工作区是否有未提交改动；
- 正式发版所需 FANUC `.tp/.pc` 是否缺失（仅警告，不阻止普通源码编译）。

结果中存在 `FAIL` 时不要继续构建。`WARN` 通常表示可选工具、脏工作区或仅正式发版需要的文件缺失。

## 7. 本地构建

### 7.1 命令行构建

首次建议分别执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\build_local.ps1 -Configuration Debug -Target Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\build_local.ps1 -Configuration Release -Target Rebuild
```

后续增量构建可把 `-Target Rebuild` 改为 `-Target Build`。

输出通常位于：

```text
x64\Debug\QtWidgetsApplication4.exe
x64\Release\QtWidgetsApplication4.exe
```

工程构建后会运行 `windeployqt`，并复制 OpenCV、SKJCamera、OpenCASCADE DLL、Qt 中文翻译和 OpenCASCADE 许可证。

### 7.2 Visual Studio 构建

1. 确认仓库根目录已有正确的 `environment.local.props`；
2. 打开 `QtWidgetsApplication4.sln`；
3. 选择 `Debug|x64` 或 `Release|x64`；
4. 执行“生成解决方案”或“重新生成解决方案”。

不要选择 Win32/x86；工程只定义了 x64。

## 8. 构建后检查

至少确认：

```powershell
Test-Path x64\Debug\QtWidgetsApplication4.exe
Test-Path x64\Release\QtWidgetsApplication4.exe
Get-FileHash x64\Release\QtWidgetsApplication4.exe -Algorithm SHA256
```

本地启动前，先确认不会连接生产机器人、相机或在线服务。代码编译成功不等于真机运动、焊接、正式安装包或 OTA 发布已通过验证。

## 9. 常见故障

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 找不到 `qt.targets`/`Qt.props` | Qt VS Tools 或 QtMsBuild 路径错误 | 修正 `QtMsBuild`，确认 `%LOCALAPPDATA%\QtMsBuild` 完整 |
| 找不到 `Qt6*.lib` | Qt 根目录、版本或 Debug 库不完整 | 安装 Qt 6.7.3 `msvc2022_64` Debug+Release |
| `LNK1104 opencv_world460[d].lib` | OpenCV 根或 `vc15` 子目录不匹配 | 使用 4.6.0 x64 包并修正 `OpenCvRoot` |
| `orocos-kdl[d].lib` 缺失 | KDL 只装了单一配置或路径错误 | 准备 1.5.3 Debug+Release 库 |
| OpenCASCADE 版本门禁失败 | 不是 7.9.3 或只装了单一配置 | 使用验证过的 vcpkg 快照重新安装 x64-windows |
| `Robot-SDK[d].lib` 缺失 | 克隆不完整或文件被安全软件隔离 | 用 Git 核对并恢复 `SDK/STEP` 跟踪文件 |
| 编译成功但启动缺 DLL | 跳过了 AfterBuild/windeployqt 或复制失败 | 用本配套构建脚本重新构建，查看首个失败项 |
| 正式打包门禁缺 `.tp/.pc` | 这些文件不在 Git 中 | 从权威现场/发版输入恢复并执行正式发布门禁 |

## 10. 正式发版和服务器部署边界

`build_local.ps1` 只用于本地源码构建。正式安装包、品牌包、OTA、GitHub/GitLab 发布必须继续使用仓库已有的发布脚本和门禁，并在本地测试后等待明确发布确认。

在线服务 Ubuntu 部署另见 [`server-deployment.md`](server-deployment.md)。它与 Windows 源码环境搭建是两条独立流程。

## 11. 配套包校验

轻量配套 ZIP 内含本说明、配置模板、环境检查脚本、构建脚本、依赖锁定清单和 SHA-256 清单，不含大型第三方安装介质和秘密。另行生成的第三方依赖离线包包含经过审计的 Qt、QtMsBuild、OpenCV、Eigen、KDL 和 OpenCASCADE 文件，详见[第三方依赖离线包说明](third-party-offline-package.md)。

校验 ZIP：

```powershell
Get-FileHash <配套包.zip> -Algorithm SHA256
```

解压后可再核对根目录下的 `MANIFEST.sha256`。

如果配套包和源码仓库分开放置，可从配套包目录显式指定源码与配置：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\check_environment.ps1 `
  -ProjectRoot E:\WorkFile\bowen\QtWidgetsApplication4 `
  -SettingsPath config\environment.local.psd1
```
