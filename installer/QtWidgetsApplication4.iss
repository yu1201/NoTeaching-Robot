#define MyAppName "NoTeaching-Robot"
#ifndef MyAppVersion
  #define MyAppVersion "2026.04.21"
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
PrivilegesRequired=lowest
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
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

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
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent
