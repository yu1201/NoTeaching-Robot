@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "PS_SCRIPT=%SCRIPT_DIR%build_release_package.ps1"

if not exist "%PS_SCRIPT%" (
    echo [ERROR] Can not find PowerShell script:
    echo         %PS_SCRIPT%
    echo.
    pause
    exit /b 1
)

echo [INFO] Building release package...
echo.
powershell -ExecutionPolicy Bypass -File "%PS_SCRIPT%" %*
set "EXIT_CODE=%ERRORLEVEL%"
echo.

if not "%EXIT_CODE%"=="0" (
    echo [ERROR] Release package build failed. ExitCode=%EXIT_CODE%
    echo.
    pause
    exit /b %EXIT_CODE%
)

echo [OK] Release package build finished.
echo [OK] Output folder: E:\WorkFile\bowen\QtWidgetsApplication4\dist\QtWidgetsApplication4
echo.
pause
exit /b 0
