# QtWidgetsApplication4 第三方依赖离线包说明

- 适用平台：Windows 10/11 x64
- 工程配置：`Debug|x64`、`Release|x64`
- 依赖基线：Qt 6.7.3、OpenCV 4.6.0、Eigen 3.4.0、Orocos KDL 1.5.3、OpenCASCADE 7.9.3
- 整理日期：`2026-08-19`

## 1. 这个包解决什么问题

另一台 Windows 机器没有 Qt、OpenCV、Eigen、Orocos KDL 或 OpenCASCADE 时，不需要逐项重新下载和猜测版本。解压本包后，可直接让 `QtWidgetsApplication4` 源码工程引用包内的固定依赖树。

本包面向**源码编译**，包含 Debug 和 Release 所需的头文件、导入库、运行 DLL、Qt 构建工具、Qt 插件、许可证与 SHA-256 清单。

本包不包含：

- Visual Studio 2022、MSVC v143 和 Windows SDK；
- Git；
- 项目源码、现场数据、机器人/相机配置、账号或密码；
- FANUC 正式发布所需的现场 `.tp/.pc` 文件；
- 安装包签名材料、OTA/GitHub/GitLab 凭据。

Visual Studio 2022 和 Git 应通过各自官方安装程序单独安装。Visual Studio 至少选择“使用 C++ 的桌面开发”、MSVC v143 x64/x86 工具和 Windows 10/11 SDK。

## 2. 包内组件

| 目录 | 版本与内容 | 打包策略 |
| --- | --- | --- |
| `dependencies/Qt/6.7.3/msvc2022_64` | Qt 头文件、mkspecs、qmake/moc/uic/rcc/windeployqt、Debug/Release 库与运行 DLL、必需插件和中文翻译 | 按本工程真实链接及部署闭包裁剪 |
| `dependencies/Qt/Licenses` | Qt 随安装器提供的许可证文本 | 完整复制 |
| `dependencies/QtMsBuild` | Visual Studio Qt 工程集成文件 | 完整复制 |
| `dependencies/OpenCV/4.6.0/build` | `opencv_world460[d]` 及头文件、DLL、许可证 | 完整复制已验证 build 树 |
| `dependencies/Eigen/3.4.0/eigen-3.4.0` | Eigen 头文件与许可证 | 完整复制 |
| `dependencies/OrocosKDL/1.5.3/orocos_kdl` | KDL 头文件、Debug/Release 库及许可证 | 完整复制已验证安装树 |
| `dependencies/vcpkg/installed/x64-windows` | OpenCASCADE 7.9.3 及 vcpkg 安装树中的配套库、DLL、许可证 | 完整复制固定 installed 树 |

`bundle-source.json` 记录每个组件的源文件数、源字节数、包内文件数、包内字节数和裁剪说明。`MANIFEST.sha256` 记录包内每个文件的 SHA-256 与长度。

### 为什么 Qt 没有整树打包

原始 Qt 6.7.3 安装树约 15.24 GiB，其中约 10 GiB 来自本项目不使用的 WebEngine、QML、其他模块和 PDB。直接整包会显著增加传输、解压和审计成本。

本包保留：

- 工程实际使用的 Core、Gui、Widgets、Network、Sql、OpenGL、OpenGLWidgets；
- SVG 图标/图片插件所需的 Svg/SvgWidgets；
- Debug/Release 导入库和运行 DLL；
- qmake、moc、uic、rcc、windeployqt、qtpaths；
- Windows 平台、SQLite、TLS、常用图像格式、现代 Windows 样式插件；
- 完整 Qt 头文件、mkspecs、metatypes、modules 和许可证。

若以后源码新增 Qt WebEngine、QML、PDF、Multimedia 等模块，必须重新生成并验证依赖包，不能假定当前裁剪包包含它们。

## 3. 新机器使用步骤

假设：

```text
源码目录：D:\Work\QtWidgetsApplication4
依赖包目录：D:\DevKits\QtWidgetsApplication4-third-party-dependencies-20260819
```

### 3.1 安装基础工具

先安装：

1. Visual Studio 2022，含 MSVC v143、x64 C++ 和 Windows SDK；
2. Git for Windows；
3. Windows PowerShell 5.1 或 PowerShell 7。

无需另行安装 Qt、OpenCV、Eigen、KDL 或 OpenCASCADE。

### 3.2 校验离线包

在 PowerShell 中执行：

```powershell
Set-Location D:\DevKits\QtWidgetsApplication4-third-party-dependencies-20260819
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\verify_third_party_bundle.ps1
```

完整校验会读取 `MANIFEST.sha256` 并重新计算每个文件的 SHA-256。看到 `BUNDLE_VERIFY_OK` 才继续。

### 3.3 为源码生成本机配置

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File D:\DevKits\QtWidgetsApplication4-third-party-dependencies-20260819\scripts\configure_project_from_bundle.ps1 `
  -BundleRoot D:\DevKits\QtWidgetsApplication4-third-party-dependencies-20260819 `
  -ProjectRoot D:\Work\QtWidgetsApplication4
```

脚本在源码目录生成两个被 Git 忽略的本机文件：

```text
environment.local.props
tools\source_environment\environment.local.psd1
```

如果这两个文件已存在，脚本默认拒绝覆盖。确认旧配置可以替换后，再显式加 `-Force`。

### 3.4 环境检查与构建

```powershell
Set-Location D:\Work\QtWidgetsApplication4
powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\check_environment.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\build_local.ps1 -Configuration Debug -Target Rebuild
powershell -NoProfile -ExecutionPolicy Bypass -File tools\source_environment\build_local.ps1 -Configuration Release -Target Rebuild
```

预期输出：

```text
x64\Debug\QtWidgetsApplication4.exe
x64\Release\QtWidgetsApplication4.exe
```

构建脚本会运行 `windeployqt`，并把 OpenCV、SKJCamera、OpenCASCADE 等运行 DLL 复制到对应输出目录。

## 4. 压缩包本身的校验

传输前记录发布方提供的 ZIP SHA-256；收到文件后执行：

```powershell
Get-FileHash D:\Transfer\QtWidgetsApplication4-third-party-dependencies-20260819.zip -Algorithm SHA256
```

ZIP 哈希一致只能证明压缩包整体未改变；解压后还应执行 `verify_third_party_bundle.ps1` 校验每个文件。

## 5. 许可证与再分发边界

包内保留了各依赖安装树已有的许可证文件。原 KDL 安装目录没有许可证文本，生成脚本从 Orocos KDL 官方仓库补入 `COPYING`，并验证 LGPL 2.1 标记。

本包仅用于项目团队在授权范围内的开发、构建和内部移交。向团队外部再分发前，应由项目负责人或法务重新确认 Qt 许可模式、第三方许可证义务和源代码/归属声明要求。

## 6. 重新生成离线包

在已验证的源机器上，从仓库根目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File tools\source_environment\new_third_party_dependencies_kit.ps1
```

脚本将：

1. 检查所有固定依赖路径和输出盘剩余空间；
2. 复制完整依赖树并裁剪 Qt；
3. 补齐 KDL 官方许可证；
4. 扫描私钥/凭据样式文件名；
5. 生成 `bundle-source.json` 和逐文件 SHA-256 清单；
6. 使用 Windows `tar.exe` 生成支持大文件的 ZIP。

为避免误覆盖，若同日 staging 目录或 ZIP 已存在，脚本会直接停止。旧产物应先人工审计、归档或移动后再重跑。

## 7. 安全和发布边界

离线依赖包验证通过，只表示源码编译所需的第三方环境完整，不表示以下事项已经通过：

- 真机连接、运动或焊接安全验证；
- 正式安装包、品牌包或 OTA 构建；
- 版本号、签名、GitHub/GitLab 发布和远端回读；
- 生产账号、证书、数据库或现场配置迁移。

这些步骤必须继续遵守项目现有的独立发布门禁。
