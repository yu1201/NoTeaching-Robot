#define MyAppName "NoTeaching-Robot"
#define MyAppGuid "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
#ifndef MyAppVersion
  #define MyAppVersion "2026.04.28"
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

[Files]
Source: "{#MySourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

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
