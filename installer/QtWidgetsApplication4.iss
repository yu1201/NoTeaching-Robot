#define MyAppName "HK-Pathlynx-CORPLA"
; AppId/MyAppGuid 与中性通道共用 A5A7E2A0（历史值，所有存量装机的注册表键都挂在它下面）。
; ⚠️ AppId 一经发布永不可改：v1643 曾改成 F3E0296A，Inno 认不出既有安装，全量升级落到
; {localappdata} 在别的盘装出第二份（2026-07-10 事故）。一台设备只装一个通道，共用无害。
#define MyAppGuid "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
#ifndef MyAppVersion
  #define MyAppVersion "2026.07.13.2115"
#endif
#ifndef MyOutputBaseFilename
  #define MyOutputBaseFilename "HK-Pathlynx-CORPLA-Setup-v2026.07.13.2115"
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

const
  ErrorAlreadyExists = 183;
  ApplicationInstanceMutexName = 'Global\NoTeaching-Robot-Hardware-Control-v1-1659b29cef5475429cd49311046dad5f450d375d8468d89e5caf3a85bc97e0f3';

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
  if not AcquireInstallationInstanceLease(Result) then
    Exit;
  ExtractTemporaryFile('ConfigMigrate_PreInstall.exe');
  ExtractTemporaryFile('ConfigMigrate_PreInstall.ps1');

  PowerShellPath := ExpandConstant('{sys}\WindowsPowerShell\v1.0\powershell.exe');
  MigrationDataPath := ExpandConstant('{app}\Data');
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
       (not FinalizeDeferredCredentialScrub) then
    begin
      DatabaseMigrationReady := False;
      DatabaseFinalizeFailed := True;
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
