@echo off
REM Script to run the Mudra Ring application and visualization with a virtual environment

REM Check if mudra_ring executable exists
if not exist "mudra_ring.exe" (
    echo Error: mudra_ring executable not found in current directory.
    echo Please build the application first.
    exit /b 1
)

REM Check for visualization script
set VIZ_SCRIPT=stream.py
if not exist "%VIZ_SCRIPT%" (
    echo Error: Visualization script not found.
    echo Please ensure stream.py exists in the current directory.
    exit /b 1
)

REM Create the named pipes if they don't exist
set BMI323_PIPE=\\.\pipe\bmi323_stream
set AFE4950_PIPE=\\.\pipe\afe4950_stream

REM Set up Python virtual environment
if not exist "mudra_ring_venv" (
    echo Setting up Python virtual environment...
    python -m venv mudra_ring_venv
    if errorlevel 1 (
        echo Failed to create virtual environment.
        exit /b 1
    )
)

REM Activate virtual environment and install requirements
call mudra_ring_venv\Scripts\activate.bat
pip install matplotlib numpy
if errorlevel 1 (
    echo Failed to install required packages.
    exit /b 1
)

REM Start the visualization script in a new window
start cmd /k "mudra_ring_venv\Scripts\python.exe %VIZ_SCRIPT%"

REM Start the mudra_ring program
echo Starting Mudra Ring application...
mudra_ring.exe

REM Cleanup
echo Stopping all processes...
taskkill /F /IM python.exe /FI "WINDOWTITLE eq %VIZ_SCRIPT%*" >nul 2>&1

REM Deactivate virtual environment
call mudra_ring_venv\Scripts\deactivate.bat

echo All processes terminated. 