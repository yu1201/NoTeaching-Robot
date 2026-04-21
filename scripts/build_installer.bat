@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "PS_SCRIPT=%SCRIPT_DIR%build_installer.ps1"

if not exist "%PS_SCRIPT%" (
    echo [ERROR] Can not find PowerShell script:
    echo         %PS_SCRIPT%
    echo.
    pause
    exit /b 1
)

echo [INFO] Building installer...
echo.
powershell -ExecutionPolicy Bypass -File "%PS_SCRIPT%" %*
set "EXIT_CODE=%ERRORLEVEL%"
echo.

if not "%EXIT_CODE%"=="0" (
    echo [ERROR] Installer build failed. ExitCode=%EXIT_CODE%
    echo.
    pause
    exit /b %EXIT_CODE%
)

echo [OK] Installer build finished.
echo [OK] Output: E:\WorkFile\bowen\QtWidgetsApplication4\dist\installer\NoTeaching-Robot-Setup.exe
echo.
pause
exit /b 0
