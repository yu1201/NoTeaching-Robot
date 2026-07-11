@echo off
setlocal EnableExtensions DisableDelayedExpansion
set "TOOL_DIR=%~dp0"
set "APP_DIR=%TOOL_DIR%.."
if not exist "%APP_DIR%\Data" if exist "%TOOL_DIR%..\QtWidgetsApplication4\Data" set "APP_DIR=%TOOL_DIR%..\QtWidgetsApplication4"

set "DATA_ROOT=%QTWIDGETSAPP4_DATA_ROOT%"
set "SOURCE_DIR="
set "PAUSE_AT_END=1"
set "OVERWRITE_ARG="

:parse_args
if "%~1"=="" goto args_done
if /I "%~1"=="--data-root" (
    if "%~2"=="" goto missing_data_root
    set "DATA_ROOT=%~2"
    shift /1
    shift /1
    goto parse_args
)
if /I "%~1"=="--source" (
    if "%~2"=="" goto missing_source
    set "SOURCE_DIR=%~2"
    shift /1
    shift /1
    goto parse_args
)
if /I "%~1"=="--no-pause" (
    set "PAUSE_AT_END=0"
    shift /1
    goto parse_args
)
if /I "%~1"=="--overwrite" (
    set "OVERWRITE_ARG=--overwrite"
    shift /1
    goto parse_args
)
echo Unknown argument: %~1
goto usage_error

:args_done
if not defined DATA_ROOT set "DATA_ROOT=%APP_DIR%"
if not defined SOURCE_DIR set "SOURCE_DIR=%APP_DIR%\Data"
for %%I in ("%DATA_ROOT%") do set "DATA_ROOT=%%~fI"
for %%I in ("%SOURCE_DIR%") do set "SOURCE_DIR=%%~fI"
set "DB_PATH=%DATA_ROOT%\Data\ConfigStore.db"

if not exist "%TOOL_DIR%ConfigMigrate.exe" (
    echo ConfigMigrate.exe not found: %TOOL_DIR%ConfigMigrate.exe
    goto failed
)
if not exist "%SOURCE_DIR%" (
    echo Legacy Data directory not found: %SOURCE_DIR%
    goto failed
)

echo Resolved data root: %DATA_ROOT%
echo Resolved source Data: %SOURCE_DIR%
echo Resolved target database: %DB_PATH%
echo.
"%TOOL_DIR%ConfigMigrate.exe" --source "%SOURCE_DIR%" --db "%DB_PATH%" --encrypt --scrub-legacy-credentials %OVERWRITE_ARG%
set "EXIT_CODE=%ERRORLEVEL%"
echo.
if not "%EXIT_CODE%"=="0" (
    echo Migration failed with exit code %EXIT_CODE%.
) else (
    echo Migration finished successfully. Review messages above.
)
if "%PAUSE_AT_END%"=="1" pause
exit /b %EXIT_CODE%

:missing_data_root
echo --data-root requires a directory.
goto usage_error

:missing_source
echo --source requires a directory.
goto usage_error

:usage_error
echo Usage: ConfigMigrate_Run.cmd [--data-root DIR] [--source LEGACY_DATA_DIR] [--overwrite] [--no-pause]
if "%PAUSE_AT_END%"=="1" pause
exit /b 2

:failed
if "%PAUSE_AT_END%"=="1" pause
exit /b 1
