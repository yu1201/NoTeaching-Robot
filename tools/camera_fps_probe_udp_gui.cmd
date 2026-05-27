@echo off
cd /d "%~dp0.."
powershell -ExecutionPolicy Bypass -File "%~dp0camera_fps_probe.ps1" -ShowWindow -Protocol Udp -HostAddress 192.168.39.18 -Port 50004 -Seconds 10 %*
