@echo off
REM Create a virtual environment for the BLE Beacon Scanner
REM This script creates a Python virtual environment and installs 
REM the required packages

echo Setting up virtual environment for BLE Beacon Scanner...

REM Get script directory
set SCRIPT_DIR=%~dp0
set VENV_DIR=%SCRIPT_DIR%venv

REM Check if Python 3 is installed
where python >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Python 3 is required but not found. Please install Python 3.
    exit /b 1
)

REM Create virtual environment if it doesn't exist
if not exist "%VENV_DIR%" (
    echo Creating virtual environment in %VENV_DIR%...
    python -m venv "%VENV_DIR%"
) else (
    echo Virtual environment already exists at %VENV_DIR%
)

REM Activate virtual environment
echo Activating virtual environment...
call "%VENV_DIR%\Scripts\activate.bat"

REM Install required packages
echo Installing required packages...
pip install --upgrade pip
pip install bleak colorama

echo Virtual environment setup complete!
echo.
echo To activate the virtual environment, run:
echo   %VENV_DIR%\Scripts\activate.bat
echo.
echo To run the BLE beacon scanner, run:
echo   python %SCRIPT_DIR%ble_beacon_scanner.py
echo.

REM Keep console window open
pause 