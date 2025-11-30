@echo off
echo ==========================================
echo 機械手臂運動學控制系統啟動器
echo ==========================================
echo.
echo 請選擇要啟動的程式：
echo.
echo [1] 基本機械手臂控制器 (robot_arm_controller.py)
echo [2] 運動學控制系統 (kinematics_control.py)  
echo [3] 整合控制系統 (integrated_controller.py)
echo [4] 安裝所需套件
echo [5] 退出
echo.

set /p choice=請輸入選項 (1-5): 

if "%choice%"=="1" (
    echo.
    echo 啟動基本機械手臂控制器...
    python robot_arm_controller.py
) else if "%choice%"=="2" (
    echo.
    echo 啟動運動學控制系統...
    python kinematics_control.py
) else if "%choice%"=="3" (
    echo.
    echo 啟動整合控制系統...
    python integrated_controller.py
) else if "%choice%"=="4" (
    echo.
    echo 安裝基本控制器套件...
    pip install -r requirements.txt
    echo.
    echo 安裝運動學控制套件...
    pip install -r kinematics_requirements.txt
    echo.
    echo 套件安裝完成！
) else if "%choice%"=="5" (
    echo 退出...
    exit
) else (
    echo 無效選項，請重新執行
)

echo.
pause