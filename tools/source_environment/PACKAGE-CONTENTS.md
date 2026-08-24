# 配套包内容

本配套包用于搭建和检查 `QtWidgetsApplication4` 的 Windows 源码环境。

包含：

- `README.md`：完整代码环境搭建说明书；
- `scripts/check_environment.ps1`：只读环境检查；
- `scripts/build_local.ps1`：Debug/Release x64 本地构建；
- `scripts/new_environment_kit.ps1`：从完整仓库重新生成配套包；
- `config/environment.local.props.example`：Visual Studio/MSBuild 本机路径模板；
- `config/environment.example.psd1`：配套脚本路径模板；
- `config/dependency-lock.json`：已验证版本锁定清单；
- `project-sdk-sha256.txt`：完整仓库中关键项目 SDK 文件的大小与 SHA-256；
- `package-source.json`：生成日期、Git HEAD 和工作区状态摘要；
- `MANIFEST.sha256`：包内文件 SHA-256。

不包含：

- Visual Studio、Qt、OpenCV、Eigen、Orocos KDL、vcpkg、OpenCASCADE 安装介质；
- 仓库源码或完整 SDK 二进制副本；
- FANUC 现场/正式发版 `.tp`、`.pc` 文件；
- 密码、令牌、SSH/签名密钥、现场数据库、服务器配置或品牌私有文件。

配套包不是正式安装包，也不授权发布、OTA 上传或真机运动。

只拿到解压后的配套包时，在 `config/environment.example.psd1` 中配置路径，或复制为
`config/environment.local.psd1` 后修改，然后显式指定源码目录：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\check_environment.ps1 `
  -ProjectRoot E:\WorkFile\bowen\QtWidgetsApplication4 `
  -SettingsPath config\environment.local.psd1
```
