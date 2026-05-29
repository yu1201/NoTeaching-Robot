@echo off
setlocal
set "APP_DIR=%~dp0.."
if not exist "%APP_DIR%\Data" if exist "%~dp0..\QtWidgetsApplication4\Data" set "APP_DIR=%~dp0..\QtWidgetsApplication4"
cd /d "%APP_DIR%"
echo Migrating legacy Data to Data\ConfigStore.db ...
echo.
"%~dp0ConfigMigrate.exe" --source "Data" --encrypt
echo.
echo Finished. Review messages above.
pause
