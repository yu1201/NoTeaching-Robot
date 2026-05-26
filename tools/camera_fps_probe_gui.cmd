@echo off
cd /d "%~dp0.."
powershell -ExecutionPolicy Bypass -File "%~dp0camera_fps_probe.ps1" -ShowWindow -HostAddress 192.168.39.18 -Seconds 10 %*
