@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"
title Carla-Air 仿真器一键启动

rem ============================================================
rem  Carla-Air (CarlaUE4) 一键启动
rem  用法：双击本文件即可；也可带地图名：
rem        启动仿真器.bat Town03
rem  地图优先级：命令行参数 > 上次使用的地图 > Town10HD
rem  Carla-Air 路径来源：应用配置里的 Carla-Air 路径 > D:\Carla-Air
rem ============================================================

echo ============================================================
echo   Carla-Air 仿真器  ·  一键启动
echo ============================================================
echo.

set "MAP=%~1"
if not defined MAP (
    if exist "%~dp0.carla_last_map" set /p MAP=<"%~dp0.carla_last_map"
)
if not defined MAP set "MAP=Town10HD"

rem ---------- 定位 Carla-Air ----------
set "CFG=%APPDATA%\drone-dataset-platform\config.json"
set "CARLA_DIR="
if exist "%CFG%" (
    for /f "usebackq delims=" %%i in (`powershell -NoProfile -Command "try{(Get-Content -Raw '%CFG%' ^| ConvertFrom-Json).carlaAirPath}catch{}"`) do set "CARLA_DIR=%%i"
)
if not defined CARLA_DIR if exist "D:\Carla-Air\StartCarlaAir.bat" set "CARLA_DIR=D:\Carla-Air"
if not defined CARLA_DIR if exist "%~dp0..\Carla-Air\StartCarlaAir.bat" set "CARLA_DIR=%~dp0..\Carla-Air"

if not defined CARLA_DIR (
    echo   [X] 没找到 Carla-Air 路径。
    echo       解决方式（任选其一）：
    echo         1. 打开平台，在「仿真器配置」页填写 Carla-Air 文件夹路径并保存
    echo         2. 把 Carla-Air 解压到 D:\Carla-Air
    goto :fail
)
if not exist "%CARLA_DIR%\StartCarlaAir.bat" (
    echo   [X] 目录里没有 StartCarlaAir.bat: %CARLA_DIR%
    goto :fail
)

echo   地图   : %MAP%
echo   仿真器 : %CARLA_DIR%
echo.

rem ---------- 已经在运行就不重复启动 ----------
call :sim_up
if "%SIM_UP%"=="0" (
    echo   [OK] 仿真器已经在运行（端口 2000 已监听），不重复启动。
    echo        要换地图请先运行 "%CARLA_DIR%\StopCarlaAir.bat"
    echo.
    ping -n 7 127.0.0.1 >nul
    exit /b 0
)

echo   [..] 正在启动 Carla-Air（会弹出 UE4 窗口，首次加载约 20-60 秒）...
start "Carla-Air 仿真器" cmd /c ""%CARLA_DIR%\StartCarlaAir.bat" %MAP%"

echo   [..] 等待 CARLA(端口 2000) 与 AirSim(端口 41451) 就绪 ...
call :wait_sim
if not "%SIM_UP%"=="0" (
    echo   [注意] 120 秒内没等到端口就绪。请查看弹出的 Carla-Air 窗口里的提示。
    goto :fail
)

echo.
echo   [OK] 仿真器已就绪：地图 %MAP%，CARLA 2000 / AirSim 41451
echo   %MAP%>"%~dp0.carla_last_map"
echo.
echo   提示：平台界面里点「🔗 连接已有仿真器」即可附着到它（不会重启仿真器）。
ping -n 9 127.0.0.1 >nul
exit /b 0

rem ==================== 子过程 ====================

:sim_up
rem 端口 2000 在监听则 SIM_UP=0（视为已运行）
set "SIM_UP=1"
netstat -ano -p tcp 2>nul | findstr /i "LISTENING" | findstr /c:":2000" >nul 2>nul
if not errorlevel 1 set "SIM_UP=0"
goto :eof

:wait_sim
set "SIM_UP=1"
for /l %%i in (1,1,40) do (
    netstat -ano -p tcp 2>nul | findstr /i "LISTENING" | findstr /c:":2000" >nul 2>nul
    if not errorlevel 1 (
        netstat -ano -p tcp 2>nul | findstr /i "LISTENING" | findstr /c:":41451" >nul 2>nul
        if not errorlevel 1 (
            set "SIM_UP=0"
            goto :eof
        )
    )
    ping -n 4 127.0.0.1 >nul
)
goto :eof

:fail
echo.
echo ------------------------------------------------------------
echo  启动未完成，请按上面的提示处理后重试。
echo ------------------------------------------------------------
echo.
pause
exit /b 1
