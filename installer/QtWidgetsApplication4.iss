#define MyAppName "HK-Pathlynx-CORPLA"
; AppId/MyAppGuid 与中性通道共用 A5A7E2A0（历史值，所有存量装机的注册表键都挂在它下面）。
; ⚠️ AppId 一经发布永不可改：v1643 曾改成 F3E0296A，Inno 认不出既有安装，全量升级落到
; {localappdata} 在别的盘装出第二份（2026-07-10 事故）。一台设备只装一个通道，共用无害。
#define MyAppGuid "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
#ifndef MyAppVersion
  #define MyAppVersion "2026.07.16.1236"
#endif
#ifndef MyOutputBaseFilename
  #define MyOutputBaseFilename "HK-Pathlynx-CORPLA-Setup-v2026.07.16.1236"
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
; 程序文件正常覆盖，Data 目录由现场运行数据拥有，安装包不覆盖也不删除。
; SDK\STEP\versions（.lib 链接期产物与 SRS 机器人系统升级包）不随安装包分发，升级包按需另发现场。
Source: "{#MySourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "Data\*,SDK\STEP\versions\*"
; 旧 INI/TXT 参数迁移工具，现场机器无需安装 Python。
Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{app}\tools"; Flags: ignoreversion
Source: "..\dist\tools\ConfigMigrate_Run.cmd"; DestDir: "{app}\tools"; Flags: ignoreversion
Source: "..\dist\tools\ConfigMigrate_Install.ps1"; DestDir: "{app}\tools"; Flags: ignoreversion
; 覆盖安装迁移必须在任何新版文件落盘前运行，因此从安装包的受校验临时载荷执行。
Source: "..\dist\tools\ConfigMigrate.exe"; DestDir: "{tmp}"; DestName: "ConfigMigrate_PreInstall.exe"; Flags: dontcopy
Source: "..\dist\tools\ConfigMigrate_Install.ps1"; DestDir: "{tmp}"; DestName: "ConfigMigrate_PreInstall.ps1"; Flags: dontcopy

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\branding\app_nobg.ico"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\branding\app_nobg.ico"; Tasks: desktopicon

[Run]
Filename: "{app}\Prerequisites\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Microsoft Visual C++ Runtime..."; Flags: waituntilterminated runhidden; Check: VcRedistInstallerExists and VcRedistNeedsInstall
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent; Check: DatabaseMigrationSucceeded

[Code]
var
  DatabaseMigrationReady: Boolean;
  DatabaseMigrationAttempted: Boolean;
  DatabaseMigrationHelperReady: Boolean;
  DatabaseTransactionReconciled: Boolean;
  InstallationCompleted: Boolean;
  MigrationDataPath: string;
  MigrationExePath: string;
  MigrationScriptPath: string;
  MigrationStatusPath: string;
  MigrationBackupName: string;
  MigrationBackupSha256: string;
  MigrationCreatedDatabaseSha256: string;
  MigrationNeedsFinalize: Boolean;
  DatabaseFinalizeFailed: Boolean;
  InstallationInstanceLeaseHandle: LongWord;
  InstallationInstanceLeaseHeld: Boolean;
  SameVersionRecoveryRequired: Boolean;
  SameVersionRecoveryRegistryRoot: Integer;
  SameVersionRecoveryUninstallKey: string;
  SameVersionRecoveryDataPath: string;
  SameVersionRecoveryRecordPath: string;
  SameVersionRecoveryRecordSha256: string;
  DatabaseClosingFailed: Boolean;
  DatabaseClosingExpectedRecordSha256: string;

const
  ErrorAlreadyExists = 183;
  ApplicationInstanceMutexName = 'Global\NoTeaching-Robot-Hardware-Control-v1-1659b29cef5475429cd49311046dad5f450d375d8468d89e5caf3a85bc97e0f3';
  SameVersionRecoveryFormat = 'NoTeaching-Robot-Same-Version-Recovery-v1';
  RecoveryReadyValueName = 'NoTeachingRobotRecoveryReady';
  RecoveryFormatValueName = 'NoTeachingRobotRecoveryFormat';
  RecoveryVersionValueName = 'NoTeachingRobotRecoveryVersion';
  RecoveryDisplayNameValueName = 'NoTeachingRobotRecoveryDisplayName';
  RecoveryExecutableValueName = 'NoTeachingRobotRecoveryExecutable';
  RecoveryInstallDirectoryValueName = 'NoTeachingRobotRecoveryInstallDirectory';
  RecoveryRecordPathValueName = 'NoTeachingRobotRecoveryRecordPath';
  RecoveryRecordSha256ValueName = 'NoTeachingRobotRecoveryRecordSha256';

function CreateMutex(SecurityAttributes: LongWord; InitialOwner: Boolean;
  Name: string): LongWord;
  external 'CreateMutexW@kernel32.dll stdcall';
function CloseHandle(Handle: LongWord): Boolean;
  external 'CloseHandle@kernel32.dll stdcall';
function GetLastError(): LongWord;
  external 'GetLastError@kernel32.dll stdcall';
procedure SetLastError(ErrorCode: LongWord);
  external 'SetLastError@kernel32.dll stdcall';

function AcquireInstallationInstanceLease(var FailureReason: string): Boolean;
var
  LastError: LongWord;
  CandidateHandle: LongWord;
begin
  Result := InstallationInstanceLeaseHeld;
  if Result then
    Exit;

  SetLastError(0);
  CandidateHandle := CreateMutex(0, False, ApplicationInstanceMutexName);
  LastError := GetLastError();
  if CandidateHandle = 0 then
  begin
    FailureReason :=
      '无法取得程序的机器级安装互斥锁，安装已停止。Windows 错误：' +
      IntToStr(LastError);
    Exit;
  end;
  if LastError = ErrorAlreadyExists then
  begin
    CloseHandle(CandidateHandle);
    FailureReason :=
      '检测到程序或另一个安装进程仍持有机器级运行锁。' + #13#10 +
      '请关闭所有 NoTeaching-Robot/QtWidgetsApplication4 实例后重试。';
    Exit;
  end;

  InstallationInstanceLeaseHandle := CandidateHandle;
  InstallationInstanceLeaseHeld := True;
  Result := True;
  Log('Acquired machine-wide application lease for the complete installation transaction.');
end;

function ReleaseInstallationInstanceLease: Boolean;
begin
  Result := True;
  if not InstallationInstanceLeaseHeld then
    Exit;

  if not CloseHandle(InstallationInstanceLeaseHandle) then
  begin
    Result := False;
    Log('CRITICAL: failed to release the machine-wide application installation lease.');
    Exit;
  end;
  InstallationInstanceLeaseHandle := 0;
  InstallationInstanceLeaseHeld := False;
  Log('Released machine-wide application lease after the installation transaction.');
end;

function QuoteProcessArgument(const Value: string): string;
begin
  Result := '"' + Value + '"';
end;

function ReadMigrationStatus(const StatusPath: string): string;
var
  RawStatus: AnsiString;
begin
  Result := '状态文件不可读';
  if LoadStringFromFile(StatusPath, RawStatus) then
  begin
    Result := Trim(string(RawStatus));
    if Result = '' then
      Result := '状态文件为空';
  end;
end;

function GetMigrationStatusValue(const StatusText: string; const Name: string): string;
var
  StartPos: Integer;
  EndPos: Integer;
begin
  Result := '';
  StartPos := Pos(Name + '=', StatusText);
  if StartPos = 0 then
    Exit;
  Result := Copy(StatusText, StartPos + Length(Name) + 1, Length(StatusText));
  EndPos := Pos(#13, Result);
  if EndPos > 0 then
    Result := Copy(Result, 1, EndPos - 1);
  EndPos := Pos(#10, Result);
  if EndPos > 0 then
    Result := Copy(Result, 1, EndPos - 1);
  Result := Trim(Result);
end;

function NormalizeMigrationStatus(const StatusText: string): string;
begin
  Result := Trim(StatusText);
  StringChangeEx(Result, #13#10, #10, True);
  StringChangeEx(Result, #13, #10, True);
end;

function IsLowerHexSha256(const Value: string): Boolean;
var
  I: Integer;
begin
  Result := Length(Value) = 64;
  if not Result then
    Exit;
  for I := 1 to Length(Value) do
  begin
    if not (((Value[I] >= '0') and (Value[I] <= '9')) or
            ((Value[I] >= 'a') and (Value[I] <= 'f'))) then
    begin
      Result := False;
      Exit;
    end;
  end;
end;

function GetApplicationUninstallKey: string;
begin
  Result := 'Software\Microsoft\Windows\CurrentVersion\Uninstall\{' +
    '{#MyAppGuid}' + '}_is1';
end;

function TryReadInstalledAppRegistration(
  RootKey: Integer;
  const UninstallKey: string;
  var Version: string;
  var InstallLocation: string;
  var DisplayName: string;
  var DisplayIcon: string): Boolean;
begin
  Result :=
    RegQueryStringValue(RootKey, UninstallKey, 'DisplayVersion', Version) and
    RegQueryStringValue(RootKey, UninstallKey, 'InstallLocation', InstallLocation) and
    RegQueryStringValue(RootKey, UninstallKey, 'DisplayName', DisplayName) and
    RegQueryStringValue(RootKey, UninstallKey, 'DisplayIcon', DisplayIcon);
end;

function GetInstalledAppRegistration(
  var RootKey: Integer;
  var UninstallKey: string;
  var Version: string;
  var InstallLocation: string;
  var DisplayName: string;
  var DisplayIcon: string): Boolean;
begin
  UninstallKey := GetApplicationUninstallKey;
  RootKey := HKLM64;
  Result := TryReadInstalledAppRegistration(
    RootKey, UninstallKey, Version, InstallLocation, DisplayName, DisplayIcon);
  if not Result then
  begin
    RootKey := HKLM;
    Result := TryReadInstalledAppRegistration(
      RootKey, UninstallKey, Version, InstallLocation, DisplayName, DisplayIcon);
  end;
  if not Result then
  begin
    RootKey := HKCU;
    Result := TryReadInstalledAppRegistration(
      RootKey, UninstallKey, Version, InstallLocation, DisplayName, DisplayIcon);
  end;
end;

function NormalizeAbsoluteLocalPath(const Value: string; var Normalized: string): Boolean;
var
  Candidate: string;
begin
  Candidate := Trim(Value);
  Result :=
    (Length(Candidate) >= 3) and
    (((Candidate[1] >= 'A') and (Candidate[1] <= 'Z')) or
     ((Candidate[1] >= 'a') and (Candidate[1] <= 'z'))) and
    (Candidate[2] = ':') and
    (Candidate[3] = '\');
  if not Result then
    Exit;
  Normalized := RemoveBackslashUnlessRoot(ExpandFileName(Candidate));
end;

function DeleteRecoveryValueIfPresent(
  RootKey: Integer;
  const UninstallKey: string;
  const ValueName: string): Boolean;
begin
  Result := True;
  if RegValueExists(RootKey, UninstallKey, ValueName) then
    Result := RegDeleteValue(RootKey, UninstallKey, ValueName);
end;

function ClearSameVersionRecoveryMarker: Boolean;
var
  UninstallKey: string;
begin
  UninstallKey := GetApplicationUninstallKey;
  Result := DeleteRecoveryValueIfPresent(
    HKLM64, UninstallKey, RecoveryReadyValueName);
  if not Result then
  begin
    Log('CRITICAL: could not invalidate the protected same-version recovery marker.');
    Exit;
  end;

  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryFormatValueName) then
    Result := False;
  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryVersionValueName) then
    Result := False;
  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryDisplayNameValueName) then
    Result := False;
  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryExecutableValueName) then
    Result := False;
  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryInstallDirectoryValueName) then
    Result := False;
  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryRecordPathValueName) then
    Result := False;
  if not DeleteRecoveryValueIfPresent(HKLM64, UninstallKey, RecoveryRecordSha256ValueName) then
    Result := False;
  if not Result then
    Log('CRITICAL: the invalidated same-version recovery marker could not be fully removed.');
end;

function TryReadSameVersionRecoveryMarker(
  RootKey: Integer;
  const UninstallKey: string;
  var Version: string;
  var DisplayName: string;
  var ExecutableName: string;
  var InstallDirectory: string;
  var RecordPath: string;
  var RecordSha256: string): Boolean;
var
  Ready: string;
  MarkerFormat: string;
begin
  Result :=
    RegQueryStringValue(RootKey, UninstallKey, RecoveryReadyValueName, Ready) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryFormatValueName, MarkerFormat) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryVersionValueName, Version) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryDisplayNameValueName, DisplayName) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryExecutableValueName, ExecutableName) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryInstallDirectoryValueName, InstallDirectory) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryRecordPathValueName, RecordPath) and
    RegQueryStringValue(RootKey, UninstallKey, RecoveryRecordSha256ValueName, RecordSha256) and
    (Ready = '1') and
    (MarkerFormat = SameVersionRecoveryFormat);
end;

function ValidateSameVersionRecoveryMarker(
  RootKey: Integer;
  const UninstallKey: string;
  const InstalledVersion: string;
  const InstalledLocation: string;
  const InstalledDisplayName: string;
  const InstalledDisplayIcon: string;
  var DataPath: string;
  var RecordPath: string;
  var RecordSha256: string): Boolean;
var
  MarkerVersion: string;
  MarkerDisplayName: string;
  MarkerExecutableName: string;
  MarkerInstallDirectory: string;
  MarkerRecordPath: string;
  MarkerRecordSha256: string;
  NormalizedInstallLocation: string;
  NormalizedMarkerInstallDirectory: string;
  NormalizedDisplayIcon: string;
  NormalizedMarkerRecordPath: string;
  ExpectedDisplayIcon: string;
  ExpectedRecordPath: string;
  ActualRecordSha256: string;
begin
  Result := False;
  if (RootKey <> HKLM64) or
     (UninstallKey <> GetApplicationUninstallKey) or
     (InstalledVersion <> '{#MyAppVersion}') or
     (InstalledDisplayName <> '{#MyAppName} version {#MyAppVersion}') then
    Exit;
  if not NormalizeAbsoluteLocalPath(InstalledLocation, NormalizedInstallLocation) then
    Exit;
  ExpectedDisplayIcon := AddBackslash(NormalizedInstallLocation) + '{#MyAppExeName}';
  if not NormalizeAbsoluteLocalPath(InstalledDisplayIcon, NormalizedDisplayIcon) or
     (CompareText(NormalizedDisplayIcon, ExpectedDisplayIcon) <> 0) or
     (not FileExists(ExpectedDisplayIcon)) then
    Exit;

  if not TryReadSameVersionRecoveryMarker(
       RootKey,
       UninstallKey,
       MarkerVersion,
       MarkerDisplayName,
       MarkerExecutableName,
       MarkerInstallDirectory,
       MarkerRecordPath,
       MarkerRecordSha256) then
    Exit;
  if (MarkerVersion <> InstalledVersion) or
     (MarkerDisplayName <> InstalledDisplayName) or
     (MarkerExecutableName <> '{#MyAppExeName}') or
     (not IsLowerHexSha256(MarkerRecordSha256)) then
    Exit;
  if not NormalizeAbsoluteLocalPath(
       MarkerInstallDirectory, NormalizedMarkerInstallDirectory) or
     (CompareText(NormalizedMarkerInstallDirectory, NormalizedInstallLocation) <> 0) then
    Exit;

  DataPath := AddBackslash(NormalizedInstallLocation) + 'Data';
  ExpectedRecordPath := AddBackslash(DataPath) +
    'ConfigStore.db.install-transaction-v1';
  if not NormalizeAbsoluteLocalPath(MarkerRecordPath, NormalizedMarkerRecordPath) or
     (CompareText(NormalizedMarkerRecordPath, ExpectedRecordPath) <> 0) or
     (not FileExists(ExpectedRecordPath)) then
    Exit;
  try
    ActualRecordSha256 := Lowercase(GetSHA256OfFile(ExpectedRecordPath));
  except
    Exit;
  end;
  if ActualRecordSha256 <> MarkerRecordSha256 then
    Exit;

  RecordPath := ExpectedRecordPath;
  RecordSha256 := MarkerRecordSha256;
  Result := True;
end;

function TryGetPendingTransactionRecordSha256(
  const DataPath: string;
  var RecordSha256: string): Boolean;
var
  NormalizedDataPath: string;
  RecordPath: string;
begin
  Result := False;
  RecordSha256 := '';
  if not NormalizeAbsoluteLocalPath(DataPath, NormalizedDataPath) then
    Exit;
  RecordPath := AddBackslash(NormalizedDataPath) +
    'ConfigStore.db.install-transaction-v1';
  if not FileExists(RecordPath) then
    Exit;
  try
    RecordSha256 := Lowercase(GetSHA256OfFile(RecordPath));
  except
    RecordSha256 := '';
    Exit;
  end;
  Result := IsLowerHexSha256(RecordSha256);
end;

function PersistSameVersionRecoveryMarker: Boolean;
var
  UninstallKey: string;
  InstalledVersion: string;
  InstalledLocation: string;
  InstalledDisplayName: string;
  InstalledDisplayIcon: string;
  NormalizedInstallLocation: string;
  NormalizedCurrentInstallLocation: string;
  RecordPath: string;
  RecordSha256: string;
  VerifiedDataPath: string;
  VerifiedRecordPath: string;
  VerifiedRecordSha256: string;
begin
  Result := False;
  UninstallKey := GetApplicationUninstallKey;
  if not TryReadInstalledAppRegistration(
       HKLM64,
       UninstallKey,
       InstalledVersion,
       InstalledLocation,
       InstalledDisplayName,
       InstalledDisplayIcon) then
    Exit;
  if (InstalledVersion <> '{#MyAppVersion}') or
     (InstalledDisplayName <> '{#MyAppName} version {#MyAppVersion}') or
     (not NormalizeAbsoluteLocalPath(InstalledLocation, NormalizedInstallLocation)) or
     (not NormalizeAbsoluteLocalPath(
       ExpandConstant('{app}'), NormalizedCurrentInstallLocation)) or
     (CompareText(NormalizedInstallLocation, NormalizedCurrentInstallLocation) <> 0) then
    Exit;

  RecordPath := AddBackslash(NormalizedInstallLocation) +
    'Data\ConfigStore.db.install-transaction-v1';
  if not FileExists(RecordPath) then
    Exit;
  try
    RecordSha256 := Lowercase(GetSHA256OfFile(RecordPath));
  except
    Exit;
  end;
  if not IsLowerHexSha256(RecordSha256) or
     (RecordSha256 <> DatabaseClosingExpectedRecordSha256) then
    Exit;
  if not ClearSameVersionRecoveryMarker then
    Exit;

  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryFormatValueName, SameVersionRecoveryFormat) then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryVersionValueName, InstalledVersion) then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryDisplayNameValueName, InstalledDisplayName) then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryExecutableValueName, '{#MyAppExeName}') then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  if not RegWriteStringValue(
       HKLM64,
       UninstallKey,
       RecoveryInstallDirectoryValueName,
       NormalizedInstallLocation) then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryRecordPathValueName, RecordPath) then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryRecordSha256ValueName, RecordSha256) then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  { Ready is the commit point.  It is written last and deleted first. }
  if not RegWriteStringValue(
       HKLM64, UninstallKey, RecoveryReadyValueName, '1') then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;

  Result := ValidateSameVersionRecoveryMarker(
    HKLM64,
    UninstallKey,
    InstalledVersion,
    InstalledLocation,
    InstalledDisplayName,
    InstalledDisplayIcon,
    VerifiedDataPath,
    VerifiedRecordPath,
    VerifiedRecordSha256);
  Result := Result and
    (CompareText(VerifiedRecordPath, RecordPath) = 0) and
    (VerifiedRecordSha256 = RecordSha256);
  if not Result then
  begin
    ClearSameVersionRecoveryMarker;
    Exit;
  end;
  Log('Persisted and read-back verified the HKLM-bound same-version recovery marker.');
end;

function IsCanonicalUpgradeBackupName(const Value: string): Boolean;
var
  Prefix: string;
  Suffix: string;
  GuidText: string;
  I: Integer;
begin
  Prefix := '.ConfigStore.db.install-upgrade-';
  Suffix := '.tmp.install-backup.dpapi.bak';
  Result :=
    (Length(Value) = Length(Prefix) + 32 + Length(Suffix)) and
    (Copy(Value, 1, Length(Prefix)) = Prefix) and
    (Copy(Value, Length(Prefix) + 33, Length(Suffix)) = Suffix) and
    (ExtractFileName(Value) = Value);
  if not Result then
    Exit;
  GuidText := Copy(Value, Length(Prefix) + 1, 32);
  for I := 1 to Length(GuidText) do
  begin
    if not (
      ((GuidText[I] >= '0') and (GuidText[I] <= '9')) or
      ((GuidText[I] >= 'a') and (GuidText[I] <= 'f'))
    ) then
    begin
      Result := False;
      Exit;
    end;
  end;
end;

function DatabaseMigrationSucceeded: Boolean;
begin
  Result := DatabaseMigrationReady;
end;

function RollbackPendingDatabaseTransaction(var RollbackStatus: string): Boolean;
var
  PowerShellPath: string;
  Parameters: string;
  ResultCode: Integer;
begin
  Result := False;
  RollbackStatus := '状态文件不可读';
  DeleteFile(MigrationStatusPath);
  PowerShellPath := ExpandConstant('{sys}\WindowsPowerShell\v1.0\powershell.exe');
  Parameters :=
    '-NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -File ' +
    QuoteProcessArgument(MigrationScriptPath) +
    ' -DataDirectory ' + QuoteProcessArgument(MigrationDataPath) +
    ' -MigrateExecutable ' + QuoteProcessArgument(MigrationExePath) +
    ' -ApplicationExecutable ' + QuoteProcessArgument(ExpandConstant('{app}\{#MyAppExeName}')) +
    ' -StatusFile ' + QuoteProcessArgument(MigrationStatusPath) +
    ' -RollbackPendingTransaction -InstallerHoldsInstanceLease';
  ResultCode := -1;
  if Exec(PowerShellPath, Parameters, '', SW_HIDE, ewWaitUntilTerminated, ResultCode) then
  begin
    RollbackStatus := NormalizeMigrationStatus(ReadMigrationStatus(MigrationStatusPath));
    Result :=
      (ResultCode = 0) and
      ((RollbackStatus = 'OK:PENDING_UPGRADE_ROLLED_BACK') or
       (RollbackStatus = 'OK:PENDING_CREATE_REMOVED') or
       (RollbackStatus = 'OK:PENDING_PUBLISHED_PRESERVED') or
       (RollbackStatus = 'OK:PENDING_NOOP_CLEARED'));
  end;
end;

function CommitPendingDatabaseTransaction(var CommitStatus: string): Boolean;
var
  PowerShellPath: string;
  Parameters: string;
  ResultCode: Integer;
begin
  Result := False;
  CommitStatus := '状态文件不可读';
  DeleteFile(MigrationStatusPath);
  PowerShellPath := ExpandConstant('{sys}\WindowsPowerShell\v1.0\powershell.exe');
  Parameters :=
    '-NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -File ' +
    QuoteProcessArgument(MigrationScriptPath) +
    ' -DataDirectory ' + QuoteProcessArgument(MigrationDataPath) +
    ' -MigrateExecutable ' + QuoteProcessArgument(MigrationExePath) +
    ' -ApplicationExecutable ' + QuoteProcessArgument(ExpandConstant('{app}\{#MyAppExeName}')) +
    ' -StatusFile ' + QuoteProcessArgument(MigrationStatusPath) +
    ' -CommitPendingTransaction -InstallerHoldsInstanceLease';

  ResultCode := -1;
  if Exec(PowerShellPath, Parameters, '', SW_HIDE, ewWaitUntilTerminated, ResultCode) then
  begin
    CommitStatus := NormalizeMigrationStatus(ReadMigrationStatus(MigrationStatusPath));
    if (ResultCode = 0) and
       (CommitStatus = 'OK:PENDING_TRANSACTION_COMMITTED') then
    begin
      MigrationNeedsFinalize := False;
      Result := True;
    end
    else if (ResultCode = 0) and
            (CommitStatus = 'OK:PENDING_TRANSACTION_COMMITTED_FINALIZE_REQUIRED') then
    begin
      MigrationNeedsFinalize := True;
      Result := True;
    end;
  end;
end;

procedure AppendPreparationRollbackOutcome(var FailureMessage: string);
var
  RollbackStatus: string;
begin
  if RollbackPendingDatabaseTransaction(RollbackStatus) then
  begin
    DatabaseTransactionReconciled := True;
    FailureMessage := FailureMessage + #13#10 +
      '安装器已按独立事务记录完成数据库回滚。状态：' + RollbackStatus;
    if (RollbackStatus <> 'OK:PENDING_PUBLISHED_PRESERVED') and
       (not ClearSameVersionRecoveryMarker) then
      FailureMessage := FailureMessage + #13#10 +
        '严重错误：数据库已回滚，但 HKLM 同版本恢复凭证未能完整清除。';
  end
  else
    FailureMessage := FailureMessage + #13#10 +
      '严重错误：安装器无法按独立事务记录确认数据库回滚。请勿启动程序。状态：' +
      RollbackStatus;
end;

function PrepareToInstall(var NeedsRestart: Boolean): string;
var
  PowerShellPath: string;
  Parameters: string;
  StatusText: string;
  ExpectedStatusText: string;
  RecoveryInstalledVersion: string;
  RecoveryInstalledLocation: string;
  RecoveryInstalledDisplayName: string;
  RecoveryInstalledDisplayIcon: string;
  RevalidatedRecoveryDataPath: string;
  RevalidatedRecoveryRecordPath: string;
  RevalidatedRecoveryRecordSha256: string;
  ResultCode: Integer;
begin
  Result := '';
  if DatabaseMigrationAttempted then
  begin
    if not DatabaseMigrationReady then
      Result := '账号和配置数据库尚未安全迁移，安装已停止。';
    Exit;
  end;

  DatabaseMigrationAttempted := True;
  DatabaseMigrationReady := False;
  DatabaseClosingExpectedRecordSha256 := '';
  MigrationDataPath := ExpandConstant('{app}\Data');
  if SameVersionRecoveryRequired then
  begin
    if CompareText(
         RemoveBackslashUnlessRoot(ExpandFileName(MigrationDataPath)),
         RemoveBackslashUnlessRoot(ExpandFileName(SameVersionRecoveryDataPath))) <> 0 then
    begin
      Result :=
        '同版本恢复安装的目标目录与原安装事务不一致，安装已在写入任何文件前停止。' + #13#10 +
        '必须从原安装目录恢复：' + SameVersionRecoveryDataPath;
      Exit;
    end;
    if not TryReadInstalledAppRegistration(
         SameVersionRecoveryRegistryRoot,
         SameVersionRecoveryUninstallKey,
         RecoveryInstalledVersion,
         RecoveryInstalledLocation,
         RecoveryInstalledDisplayName,
         RecoveryInstalledDisplayIcon) or
       not ValidateSameVersionRecoveryMarker(
         SameVersionRecoveryRegistryRoot,
         SameVersionRecoveryUninstallKey,
         RecoveryInstalledVersion,
         RecoveryInstalledLocation,
         RecoveryInstalledDisplayName,
         RecoveryInstalledDisplayIcon,
         RevalidatedRecoveryDataPath,
         RevalidatedRecoveryRecordPath,
         RevalidatedRecoveryRecordSha256) or
       (CompareText(RevalidatedRecoveryDataPath, SameVersionRecoveryDataPath) <> 0) or
       (CompareText(RevalidatedRecoveryRecordPath, SameVersionRecoveryRecordPath) <> 0) or
       (RevalidatedRecoveryRecordSha256 <> SameVersionRecoveryRecordSha256) then
    begin
      Result :=
        '同版本恢复所需的 HKLM 凭证或数据库事务绑定已改变，安装已在写入任何文件前停止。' + #13#10 +
        '请勿绕过恢复门禁、修改注册表或手工创建事务记录。';
      Exit;
    end;
  end;
  if not AcquireInstallationInstanceLease(Result) then
    Exit;
  ExtractTemporaryFile('ConfigMigrate_PreInstall.exe');
  ExtractTemporaryFile('ConfigMigrate_PreInstall.ps1');

  PowerShellPath := ExpandConstant('{sys}\WindowsPowerShell\v1.0\powershell.exe');
  MigrationExePath := ExpandConstant('{tmp}\ConfigMigrate_PreInstall.exe');
  MigrationScriptPath := ExpandConstant('{tmp}\ConfigMigrate_PreInstall.ps1');
  MigrationStatusPath := ExpandConstant('{tmp}\ConfigMigrate_PreInstall.status');
  DatabaseMigrationHelperReady := True;
  DeleteFile(MigrationStatusPath);

  Parameters :=
    '-NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -File ' +
    QuoteProcessArgument(MigrationScriptPath) +
    ' -DataDirectory ' + QuoteProcessArgument(MigrationDataPath) +
    ' -MigrateExecutable ' + QuoteProcessArgument(MigrationExePath) +
    ' -ApplicationExecutable ' + QuoteProcessArgument(ExpandConstant('{app}\{#MyAppExeName}')) +
    ' -StatusFile ' + QuoteProcessArgument(MigrationStatusPath) +
    ' -InstallerHoldsInstanceLease';

  ResultCode := -1;
  if not Exec(PowerShellPath, Parameters, '', SW_HIDE, ewWaitUntilTerminated, ResultCode) then
  begin
    Result :=
      '无法启动账号和配置数据库升级工具，安装已在写入新版程序前停止。' + #13#10 +
      '原安装目录和数据库未由安装器覆盖。';
    DatabaseTransactionReconciled := True;
    Exit;
  end;

  StatusText := NormalizeMigrationStatus(ReadMigrationStatus(MigrationStatusPath));
  if ResultCode <> 0 then
  begin
    Result :=
      '账号和配置数据库安全升级失败，安装已在启动新版程序前停止。' + #13#10 +
      '迁移工具退出码：' + IntToStr(ResultCode) + #13#10 +
      '迁移状态：' + StatusText + #13#10 +
      '请保留 Data 目录和 *.dpapi.bak 文件并联系维护人员；不要删除数据库或使用 --overwrite。';
    AppendPreparationRollbackOutcome(Result);
    Exit;
  end;

  if StatusText = 'OK:NO_DATABASE' then
  begin
    { Exact no-field status. }
  end
  else if StatusText = 'OK:CURRENT_AND_VERIFIED' then
  begin
    { Exact no-field status. }
  end
  else if StatusText = 'OK:CURRENT_AND_VERIFIED' + #10 + 'FINALIZE=1' then
  begin
    MigrationNeedsFinalize := True;
  end
  else if Pos('OK:MIGRATED_AND_VERIFIED' + #10, StatusText) = 1 then
  begin
    MigrationBackupName := GetMigrationStatusValue(StatusText, 'BACKUP_NAME');
    MigrationBackupSha256 := GetMigrationStatusValue(StatusText, 'BACKUP_SHA256');
    ExpectedStatusText :=
      'OK:MIGRATED_AND_VERIFIED' + #10 +
      'BACKUP_NAME=' + MigrationBackupName + #10 +
      'BACKUP_SHA256=' + MigrationBackupSha256;
    if ((StatusText <> ExpectedStatusText) and
        (StatusText <> ExpectedStatusText + #10 + 'FINALIZE=1')) or
       (not IsCanonicalUpgradeBackupName(MigrationBackupName)) or
       (not IsLowerHexSha256(MigrationBackupSha256)) or
       (ExtractFileName(MigrationBackupName) <> MigrationBackupName) then
    begin
      Result :=
        '账号和配置数据库已升级，但受保护回滚备份状态不完整或含未知字段，安装已停止。';
      AppendPreparationRollbackOutcome(Result);
      Exit;
    end;
    MigrationNeedsFinalize :=
      StatusText = ExpectedStatusText + #10 + 'FINALIZE=1';
  end
  else if Pos('OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED' + #10, StatusText) = 1 then
  begin
    MigrationCreatedDatabaseSha256 := GetMigrationStatusValue(StatusText, 'CREATED_SHA256');
    if (StatusText <>
          'OK:LEGACY_DATABASE_CREATED_DEFERRED_AND_VERIFIED' + #10 +
          'DATABASE_CREATED=1' + #10 +
          'CREATED_SHA256=' + MigrationCreatedDatabaseSha256 + #10 +
          'FINALIZE=1') or
       (not IsLowerHexSha256(MigrationCreatedDatabaseSha256)) then
    begin
      Result :=
        '旧版配置数据库已创建，但安装回滚或凭据清理状态不完整或含未知字段，安装已停止。';
      AppendPreparationRollbackOutcome(Result);
      Exit;
    end;
    MigrationNeedsFinalize := True;
  end
  else if Pos('OK:LEGACY_DATABASE_CREATED_AND_VERIFIED' + #10, StatusText) = 1 then
  begin
    MigrationCreatedDatabaseSha256 := GetMigrationStatusValue(StatusText, 'CREATED_SHA256');
    if (StatusText <>
          'OK:LEGACY_DATABASE_CREATED_AND_VERIFIED' + #10 +
          'DATABASE_CREATED=1' + #10 +
          'CREATED_SHA256=' + MigrationCreatedDatabaseSha256) or
       (not IsLowerHexSha256(MigrationCreatedDatabaseSha256)) then
    begin
      Result :=
        '配置数据库已创建，但完整状态或哈希含未知字段，安装已停止。';
      AppendPreparationRollbackOutcome(Result);
      Exit;
    end;
  end
  else
  begin
    Result :=
      '账号和配置数据库升级工具返回了未知或含多余字段的成功状态，安装已停止。' + #13#10 +
      '迁移状态：' + StatusText;
    AppendPreparationRollbackOutcome(Result);
    Exit;
  end;

  if not TryGetPendingTransactionRecordSha256(
       MigrationDataPath, DatabaseClosingExpectedRecordSha256) then
  begin
    Result :=
      '账号和配置数据库升级状态缺少可回读的事务记录，安装已停止。';
    AppendPreparationRollbackOutcome(Result);
    Exit;
  end;
  DatabaseMigrationReady := True;
end;

function FinalizeDeferredCredentialScrub: Boolean;
var
  PowerShellPath: string;
  Parameters: string;
  FinalizeStatus: string;
  FailureLogPath: string;
  ResultCode: Integer;
begin
  Result := False;
  DeleteFile(MigrationStatusPath);
  PowerShellPath := ExpandConstant('{sys}\WindowsPowerShell\v1.0\powershell.exe');
  Parameters :=
    '-NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -File ' +
    QuoteProcessArgument(MigrationScriptPath) +
    ' -DataDirectory ' + QuoteProcessArgument(MigrationDataPath) +
    ' -MigrateExecutable ' + QuoteProcessArgument(MigrationExePath) +
    ' -ApplicationExecutable ' + QuoteProcessArgument(ExpandConstant('{app}\{#MyAppExeName}')) +
    ' -StatusFile ' + QuoteProcessArgument(MigrationStatusPath) +
    ' -FinalizeDeferredScrub -InstallerHoldsInstanceLease';

  ResultCode := -1;
  if Exec(PowerShellPath, Parameters, '', SW_HIDE, ewWaitUntilTerminated, ResultCode) and
     (ResultCode = 0) then
  begin
    FinalizeStatus := NormalizeMigrationStatus(ReadMigrationStatus(MigrationStatusPath));
    if FinalizeStatus = 'OK:DEFERRED_SCRUB_FINALIZED_AND_VERIFIED' then
    begin
      Result := True;
      Exit;
    end;
  end;

  FinalizeStatus := NormalizeMigrationStatus(ReadMigrationStatus(MigrationStatusPath));
  FailureLogPath := ExpandConstant('{app}\Log\ConfigMigrate_Install_Failure.txt');
  ForceDirectories(ExtractFileDir(FailureLogPath));
  SaveStringToFile(
    FailureLogPath,
    'Deferred credential cleanup failed. Status=' + FinalizeStatus + #13#10,
    False);
  Log('Deferred credential cleanup failed; automatic application launch is disabled. Status=' + FinalizeStatus);
  if not WizardSilent then
    MsgBox(
      '旧版账号凭据的安装后安全清理未能通过回读验证。' + #13#10 +
      '新版程序不会自动启动，数据库和旧配置已保留为失败关闭状态。' + #13#10 +
      '请联系维护人员并提供：' + FailureLogPath + #13#10 +
      '状态：' + FinalizeStatus,
      mbCriticalError,
      MB_OK);
end;

procedure CurStepChanged(CurStep: TSetupStep);
var
  CommitStatus: string;
begin
  if CurStep = ssPostInstall then
  begin
    if not CommitPendingDatabaseTransaction(CommitStatus) then
    begin
      DatabaseMigrationReady := False;
      DatabaseFinalizeFailed := True;
      DatabaseClosingFailed := True;
      Log('CRITICAL: pending database transaction commit failed. Status=' + CommitStatus);
      if not WizardSilent then
        MsgBox(
          '新版程序文件已安装，但数据库安装事务记录无法安全提交。' + #13#10 +
          '程序不会自动启动；请勿手工删除 Data 目录中的事务记录。' + #13#10 +
          '状态：' + CommitStatus,
          mbCriticalError,
          MB_OK);
    end;
    if DatabaseMigrationReady and MigrationNeedsFinalize and
       (not TryGetPendingTransactionRecordSha256(
         MigrationDataPath, DatabaseClosingExpectedRecordSha256)) then
    begin
      DatabaseMigrationReady := False;
      DatabaseFinalizeFailed := True;
      DatabaseClosingFailed := True;
      Log('CRITICAL: the committed finalize-pending transaction record could not be hash-bound.');
    end;
    if DatabaseMigrationReady and MigrationNeedsFinalize and
       (not FinalizeDeferredCredentialScrub) then
    begin
      DatabaseMigrationReady := False;
      DatabaseFinalizeFailed := True;
      DatabaseClosingFailed := True;
    end;
    if DatabaseClosingFailed then
    begin
      if not PersistSameVersionRecoveryMarker then
        Log('CRITICAL: database closing failed and no protected same-version recovery marker could be persisted.');
    end
    else if not ClearSameVersionRecoveryMarker then
    begin
      DatabaseMigrationReady := False;
      DatabaseFinalizeFailed := True;
      Log('CRITICAL: successful database closing could not clear the protected same-version recovery marker.');
    end;
    { ssPostInstall means the new file set is complete.  From this point onward
      the safe state is the new program plus its v5 database; never roll it back
      to an old executable after deferred cleanup may have started. }
    InstallationCompleted := True;
    if DatabaseMigrationReady and (not ReleaseInstallationInstanceLease) then
    begin
      DatabaseMigrationReady := False;
      DatabaseFinalizeFailed := True;
    end;
  end;
  if CurStep = ssDone then
    InstallationCompleted := True;
end;

function GetCustomSetupExitCode: Integer;
begin
  if DatabaseFinalizeFailed then
    Result := 20
  else
    Result := 0;
end;

procedure DeinitializeSetup;
var
  RollbackStatus: string;
begin
  try
    if (not DatabaseMigrationAttempted) or (not DatabaseMigrationHelperReady) or
       DatabaseTransactionReconciled or InstallationCompleted then
      Exit;

    if RollbackPendingDatabaseTransaction(RollbackStatus) then
    begin
      Log('Pending database transaction was reconciled after installation did not complete. Status=' +
        RollbackStatus);
      DatabaseMigrationReady := False;
      if (RollbackStatus <> 'OK:PENDING_PUBLISHED_PRESERVED') and
         (not ClearSameVersionRecoveryMarker) then
        Log('CRITICAL: rollback succeeded but the protected same-version recovery marker was not fully cleared.');
      Exit;
    end;

    Log(
      'CRITICAL: pending pre-install database transaction rollback failed. Status=' + RollbackStatus);
    if not WizardSilent then
      MsgBox(
        '安装未完成，而且账号和配置数据库自动回滚失败。' + #13#10 +
        '请勿启动旧版或新版程序；请保留 Data 目录、事务记录和受保护备份并联系维护人员。' + #13#10 +
        '回滚状态：' + RollbackStatus,
        mbCriticalError,
        MB_OK);
  finally
    if not ReleaseInstallationInstanceLease then
      Log('CRITICAL: the installation process will retain the instance lease until it exits.');
  end;
end;

function InitializeSetup(): Boolean;
var
  InstalledRootKey: Integer;
  InstalledUninstallKey: string;
  InstalledVersion: string;
  InstalledLocation: string;
  InstalledDisplayName: string;
  InstalledDisplayIcon: string;
  ValidatedDataPath: string;
  ValidatedRecordPath: string;
  ValidatedRecordSha256: string;
begin
  Result := True;
  SameVersionRecoveryRequired := False;
  SameVersionRecoveryRegistryRoot := 0;
  SameVersionRecoveryUninstallKey := '';
  SameVersionRecoveryDataPath := '';
  SameVersionRecoveryRecordPath := '';
  SameVersionRecoveryRecordSha256 := '';
  if GetInstalledAppRegistration(
       InstalledRootKey,
       InstalledUninstallKey,
       InstalledVersion,
       InstalledLocation,
       InstalledDisplayName,
       InstalledDisplayIcon) and
     (InstalledVersion = '{#MyAppVersion}') then
  begin
    if ValidateSameVersionRecoveryMarker(
         InstalledRootKey,
         InstalledUninstallKey,
         InstalledVersion,
         InstalledLocation,
         InstalledDisplayName,
         InstalledDisplayIcon,
         ValidatedDataPath,
         ValidatedRecordPath,
         ValidatedRecordSha256) then
    begin
      SameVersionRecoveryRequired := True;
      SameVersionRecoveryRegistryRoot := InstalledRootKey;
      SameVersionRecoveryUninstallKey := InstalledUninstallKey;
      SameVersionRecoveryDataPath := ValidatedDataPath;
      SameVersionRecoveryRecordPath := ValidatedRecordPath;
      SameVersionRecoveryRecordSha256 := ValidatedRecordSha256;
      Log('Same-version installation is allowed only by a read-back verified HKLM recovery marker.');
      Exit;
    end;
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
