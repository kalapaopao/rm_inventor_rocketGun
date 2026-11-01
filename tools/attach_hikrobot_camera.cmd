@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem One-click USB/IP attach for the Hikrobot USB3 Vision camera.
rem This script only maps the camera to WSL; it does not start the project.

net session >nul 2>&1
if not "%errorlevel%"=="0" (
    echo Requesting administrator permission for usbipd bind...
    powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
        "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b 0
)

set "BUSID="
for /f "tokens=1,2" %%A in ('usbipd.exe list ^| findstr /i "2bdf:0001"') do (
    if not defined BUSID set "BUSID=%%A"
)

if not defined BUSID (
    echo Hikrobot camera 2bdf:0001 was not found in usbipd list.
    echo Check the USB cable and run usbipd.exe list manually.
    pause
    exit /b 1
)

echo Camera BUSID: %BUSID%
usbipd.exe bind --busid %BUSID%
if errorlevel 1 (
    echo usbipd bind failed.
    pause
    exit /b 1
)

usbipd.exe attach --wsl --busid %BUSID%
if errorlevel 1 (
    echo usbipd attach failed.
    pause
    exit /b 1
)

echo.
echo Hikrobot camera attached to WSL successfully.
echo In WSL, verify with: lsusb
pause
