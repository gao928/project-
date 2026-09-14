@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"
title 无人机数据集生成平台

rem ============================================================
rem  无人机数据集生成平台 · 一键启动
rem  用法：直接双击本文件
rem       启动平台.bat check    —— 只检查环境，不启动（排错用）
rem ============================================================

set "CHECKONLY="
if /i "%~1"=="check" set "CHECKONLY=1"

echo ============================================================
echo   无人机数据集生成平台  ·  一键启动
echo ============================================================
echo.

rem ---------------- 1/4 Node.js ----------------
echo [1/4] 检查 Node.js ...
where node >nul 2>nul
if errorlevel 1 (
    echo   [X] 没找到 Node.js
    echo       请先安装 LTS 版: https://nodejs.org/
    echo       安装后重新双击本脚本即可。
    goto :fail
)
for /f "delims=" %%v in ('node -v 2^>nul') do echo   [OK] Node %%v
where npm >nul 2>nul
if errorlevel 1 (
    echo   [X] 找到 node 但没找到 npm，请重新安装 Node.js 并勾选 npm。
    goto :fail
)

rem ---------------- 2/4 前端依赖 ----------------
echo.
echo [2/4] 检查前端依赖 ...
if exist "node_modules\electron\dist\electron.exe" (
    echo   [OK] 依赖已就绪
) else (
    echo   [..] 首次运行，正在安装依赖，约 400MB，需要几分钟 ...
    call npm install
    if errorlevel 1 (
        echo   [X] npm install 失败。请检查网络（国内可能需要换镜像源）后重试。
        goto :fail
    )
    echo   [OK] 依赖安装完成
)

rem ---------------- 3/4 查找 Python ----------------
echo.
echo [3/4] 查找 Python 环境（需要 flask、flask_cors）...
set "PY="
set "PY_PREVIEW="
set "PATH_PY="
set "PY_LAUNCHER="

rem 3.1 PATH 上的 python（跳过微软商店的 0 字节占位程序）
for /f "delims=" %%i in ('where python 2^>nul') do (
    echo %%i | findstr /i "WindowsApps" >nul
    if errorlevel 1 if not defined PATH_PY set "PATH_PY=%%i"
)

rem 3.2 py 启动器（Python 装了但没加进 PATH 的情况）
for /f "delims=" %%p in ('py -3 -c "import sys;print(sys.executable)" 2^>nul') do set "PY_LAUNCHER=%%p"

rem 3.3 依次探测候选环境；优先挑带 carla/airsim 的（能采集、能巡航）
for %%c in (
    "%USERPROFILE%\miniconda3\envs\carlaAir\python.exe"
    "%USERPROFILE%\anaconda3\envs\carlaAir\python.exe"
    "%USERPROFILE%\AppData\Local\miniconda3\envs\carlaAir\python.exe"
    "C:\ProgramData\miniconda3\envs\carlaAir\python.exe"
    "C:\ProgramData\Anaconda3\envs\carlaAir\python.exe"
    "C:\Anaconda\envs\carlaAir\python.exe"
    "D:\miniconda3\envs\carlaAir\python.exe"
    "D:\anaconda3\envs\carlaAir\python.exe"
    "E:\miniconda3\envs\carlaAir\python.exe"
    "%~dp0.venv\Scripts\python.exe"
) do call :probe "%%~c"
if not defined PY if defined PATH_PY call :probe "%PATH_PY%"
if not defined PY if defined PY_LAUNCHER call :probe "%PY_LAUNCHER%"

if defined PY goto :py_full
if defined PY_PREVIEW (
    set "PY=%PY_PREVIEW%"
    echo   [注意] 使用: %PY_PREVIEW%
    echo       注意：这个环境没有 carla / airsim，只能预览地图；
    echo             采集数据与自动巡航需要在装了 Carla-Air 的机器上运行。
    goto :py_done
)

rem 3.4 有 python 但缺 flask —— 自动装一下
set "BASEPY="
if defined PATH_PY set "BASEPY=%PATH_PY%"
if not defined BASEPY if defined PY_LAUNCHER set "BASEPY=%PY_LAUNCHER%"
if not defined BASEPY (
    echo   [X] 没找到可用的 Python。
    echo       请安装 Miniconda / Anaconda，然后执行:
    echo           conda create -n carlaAir python=3.10
    echo           conda activate carlaAir
    echo           pip install flask flask-cors
    goto :fail
)
echo   [..] %BASEPY% 缺少 flask，正在安装 flask / flask-cors ...
"%BASEPY%" -m pip install flask flask-cors
"%BASEPY%" -c "import flask, flask_cors" >nul 2>nul
if errorlevel 1 (
    echo   [X] 安装失败。请手动执行:  "%BASEPY%" -m pip install flask flask-cors
    goto :fail
)
set "PY=%BASEPY%"
echo   [OK] 使用: %PY%

:py_full
echo   [OK] 使用: %PY%
echo       支持 Carla-Air：可以采集数据与自动巡航

:py_done

rem ---------------- 4/4 启动 ----------------
echo.
echo [4/4] 启动服务 ...
if defined CHECKONLY (
    echo   [OK] 环境检查通过，未启动服务（check 模式）。
    echo.
    pause
    exit /b 0
)

call :backend_up
if "%BACKEND_UP%"=="0" (
    echo   [OK] 后端已在运行（:5000），直接复用
    goto :backend_ready
)
echo   [..] 启动后端（会弹出一个"平台后端"窗口，别关它）...
start "平台后端 - 运行中请勿关闭" cmd /k ""%PY%" backend.py"
echo   [..] 等待后端就绪 ...
call :wait_backend
if "%BACKEND_UP%"=="0" goto :backend_started
echo   [注意] 后端还没就绪。请查看"平台后端"窗口里的提示。
echo       常见原因：端口 5000 被占用 / Carla-Air 路径未配置。
echo       界面仍会打开，"地图与路线"页的预览需要后端可用。
goto :backend_ready

:backend_started
echo   [OK] 后端就绪

:backend_ready

echo   [..] 打开界面 ...
echo.
call npm start

echo.
echo 界面已关闭。
echo 提示：后端在单独的"平台后端"窗口里运行，用完可以直接关掉那个窗口。
pause
exit /b 0

rem ==================== 子过程 ====================

:probe
rem %~1 = python.exe 路径；带 flask 记为候选，再带 carla+airsim 记为最优
if not exist "%~1" goto :eof
"%~1" -c "import flask, flask_cors" >nul 2>nul
if errorlevel 1 goto :eof
"%~1" -c "import carla, airsim" >nul 2>nul
if errorlevel 1 (
    if not defined PY_PREVIEW set "PY_PREVIEW=%~1"
) else (
    if not defined PY set "PY=%~1"
)
goto :eof

:backend_up
rem 用 netstat 判断 5000 端口是否已在监听 -> BACKEND_UP=0 表示已运行
set "BACKEND_UP=1"
netstat -ano -p tcp 2>nul | findstr /i "LISTENING" | findstr /c:":5000" >nul 2>nul
if not errorlevel 1 set "BACKEND_UP=0"
goto :eof

:wait_backend
set "BACKEND_UP=1"
for /l %%i in (1,1,30) do (
    netstat -ano -p tcp 2>nul | findstr /i "LISTENING" | findstr /c:":5000" >nul 2>nul
    if not errorlevel 1 (
        set "BACKEND_UP=0"
        goto :eof
    )
    ping -n 2 127.0.0.1 >nul
)
goto :eof

:fail
echo.
echo ------------------------------------------------------------
echo  启动未完成，请按上面的提示处理后重新双击本脚本。
echo ------------------------------------------------------------
echo.
pause
exit /b 1
