#!/bin/bash
# Script to run the Mudra Ring application and visualization with a virtual environment
# For MinGW on Windows

# Check if mudra_ring executable exists
if [ ! -x "./mudra_ring" ]; then
    echo "Error: mudra_ring executable not found in current directory."
    echo "Please build the application first using 'make'."
    exit 1
fi

# Check for visualization script
VIZ_SCRIPT="stream.py"
if [ ! -f "$VIZ_SCRIPT" ]; then
    echo "Error: Visualization script not found."
    echo "Please ensure stream.py exists in the current directory."
    exit 1
fi

# Create the named pipes if they don't exist
BMI323_PIPE="/tmp/bmi323_stream"
AFE4950_PIPE="/tmp/afe4950_stream"

# For MinGW, we need to create the /tmp directory if it doesn't exist
if [ ! -d "/tmp" ]; then
    mkdir -p /tmp
fi

if [ ! -p "$BMI323_PIPE" ]; then
    echo "Creating named pipe at $BMI323_PIPE"
    mkfifo "$BMI323_PIPE" || {
        echo "Error: Failed to create named pipe."
        exit 1
    }
fi

if [ ! -p "$AFE4950_PIPE" ]; then
    echo "Creating named pipe at $AFE4950_PIPE"
    mkfifo "$AFE4950_PIPE" || {
        echo "Error: Failed to create named pipe."
        exit 1
    }
fi

# Set up Python virtual environment
if [ ! -d "mudra_ring_venv" ]; then
    echo "Setting up Python virtual environment..."
    python -m venv mudra_ring_venv || {
        echo "Failed to create virtual environment."
        exit 1
    }
fi

# Activate virtual environment and install requirements
source mudra_ring_venv/Scripts/activate || {
    echo "Failed to activate virtual environment."
    exit 1
}

pip install matplotlib numpy || {
    echo "Failed to install required packages."
    exit 1
}

# Start the visualization script in a new window
start cmd /k "mudra_ring_venv/Scripts/python.exe $VIZ_SCRIPT"

# Start the mudra_ring program
echo "Starting Mudra Ring application..."
./mudra_ring

# Cleanup
echo "Stopping all processes..."
taskkill //F //IM python.exe //FI "WINDOWTITLE eq $VIZ_SCRIPT*" >nul 2>&1

# Deactivate virtual environment
deactivate

echo "All processes terminated."