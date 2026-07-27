#define MyAppName "HK-Pathlynx-CORPLA"
; AppId/MyAppGuid 与中性通道共用 A5A7E2A0（历史值，所有存量装机的注册表键都挂在它下面）。
; AppId 一经发布永不可改；全量升级必须继续识别既有安装目录。
#define MyAppGuid "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
#ifndef MyAppVersion
  #define MyAppVersion "2026.07.27.0930"
#endif
#ifndef MyOutputBaseFilename
  #define MyOutputBaseFilename "HK-Pathlynx-CORPLA-Setup-v2026.07.27.0930"
#endif
#define MyAppPublisher "海瞰智焊"
#define MyAppExeName "HK-Pathlynx-CORPLA.exe"
#define MySourceDir "..\dist\QtWidgetsApplication4"

[Setup]
AppId={{A5A7E2A0-8226-40BB-B126-94C5D298B3CF}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL=https://github.com/yu1201/NoTeaching-Robot
DefaultDirName={localappdata}\{#MyAppName}
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
PrivilegesRequired=admin
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
CloseApplications=force
CloseApplicationsFilter={#MyAppExeName}
RestartApplications=no
OutputDir=..\dist\installer
OutputBaseFilename={#MyOutputBaseFilename}
SetupIconFile=..\icons\app.ico
UninstallDisplayIcon={app}\{#MyAppExeName}
UninstallDisplayName={#MyAppName} version {#MyAppVersion}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Dirs]
Name: "{app}\Log"
Name: "{app}\Result"
Name: "{app}\Temp"
Name: "{app}\Data"

[Files]
; 程序文件正常覆盖，Data 目录由现场运行数据拥有，安装包不覆盖、不删除，也不执行数据库迁移。
; SDK\STEP\versions（.lib 链接期产物与 SRS 机器人系统升级包）不随安装包分发，升级包按需另发现场。
Source: "{#MySourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "Data\*,SDK\STEP\versions\*"
; 修复工具仅随程序安装，供程序检测到数据库不兼容后由用户点击“自动修复”调用；安装器本身绝不调用。
Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion
Source: "..\dist\tools\ConfigMigrate_Run.cmd"; DestDir: "{app}\tools"; Flags: ignoreversion
Source: "..\dist\tools\ConfigMigrate_Install.ps1"; DestDir: "{app}\tools"; Flags: ignoreversion

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\branding\app_nobg.ico"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\branding\app_nobg.ico"; Tasks: desktopicon

[Run]
Filename: "{app}\Prerequisites\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Microsoft Visual C++ Runtime..."; Flags: waituntilterminated runhidden; Check: VcRedistInstallerExists and VcRedistNeedsInstall
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent

[Code]
function VcRedistInstallerExists: Boolean;
begin
  Result := FileExists(ExpandConstant('{app}\Prerequisites\vc_redist.x64.exe'));
end;

function VcRedistNeedsInstall: Boolean;
var
  Installed: Cardinal;
begin
  if RegQueryDWordValue(HKLM64, 'SOFTWARE\Microsoft\VisualStudio\14.0\VC\Runtimes\x64', 'Installed', Installed) then
    Result := Installed <> 1
  else
    Result := True;
end;
