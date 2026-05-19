#define MyAppName "NoTeaching-Robot"
#define MyAppGuid "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
#ifndef MyAppVersion
  #define MyAppVersion "2026.05.19.2"
#endif
#define MyAppPublisher "yu1201"
#define MyAppExeName "QtWidgetsApplication4.exe"
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
OutputDir=..\dist\installer
OutputBaseFilename=NoTeaching-Robot-Setup
SetupIconFile=..\icons\app.ico
UninstallDisplayIcon={app}\{#MyAppExeName}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Dirs]
Name: "{app}\Log"
Name: "{app}\Result"
Name: "{app}\Temp"

[InstallDelete]
; 模版允许随安装包更新，避免旧模版残留；现场机器人参数不在这里删除。
Type: filesandordirs; Name: "{app}\Data\WorkpieceTemplates"

[Files]
; 程序文件正常覆盖，但 Data 目录单独处理，避免升级时覆盖现场保存的机器人参数。
Source: "{#MySourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "Data\*"
; 工件默认模版可以覆盖，供新建控制单元复制最新默认值。
Source: "{#MySourceDir}\Data\WorkpieceTemplates\*"; DestDir: "{app}\Data\WorkpieceTemplates"; Flags: ignoreversion recursesubdirs createallsubdirs skipifsourcedoesntexist
; 除模版外，Data 只补齐不存在的文件：新增文件会安装，已有机器人/账号/参数文件不会被覆盖。
Source: "{#MySourceDir}\Data\*"; DestDir: "{app}\Data"; Flags: ignoreversion recursesubdirs createallsubdirs onlyifdoesntexist; Excludes: "WorkpieceTemplates\*"

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\Prerequisites\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Microsoft Visual C++ Runtime..."; Flags: waituntilterminated runhidden; Check: VcRedistInstallerExists and VcRedistNeedsInstall
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent

[Code]
function GetInstalledAppVersion(var Version: string): Boolean;
var
  UninstallKey: string;
begin
  UninstallKey := 'Software\Microsoft\Windows\CurrentVersion\Uninstall\{' + '{#MyAppGuid}' + '}_is1';
  Result :=
    RegQueryStringValue(HKLM64, UninstallKey, 'DisplayVersion', Version) or
    RegQueryStringValue(HKLM, UninstallKey, 'DisplayVersion', Version) or
    RegQueryStringValue(HKCU, UninstallKey, 'DisplayVersion', Version);
end;

function InitializeSetup(): Boolean;
var
  InstalledVersion: string;
begin
  Result := True;
  if GetInstalledAppVersion(InstalledVersion) and (InstalledVersion = '{#MyAppVersion}') then
  begin
    MsgBox(
      ExpandConstant('{#MyAppName}') + ' ' + InstalledVersion +
      ' 已经安装，本次不再重复安装。' + #13#10 +
      '如果需要重新安装，请先卸载旧版本或重新生成新版本安装包。',
      mbInformation,
      MB_OK);
    Result := False;
  end;
end;

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
