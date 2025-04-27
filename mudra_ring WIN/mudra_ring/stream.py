#!/usr/bin/env python3
"""
Real-time plotter for Mudra Ring sensors (BMI323 IMU and AFE4950 PPG)
Reads from two named pipes and displays the data in a 6x1 grid layout:
- 1st: Accelerometer data
- 2nd: Gyroscope data
- 3rd: TIA1-3 PPG data
- 4th: TIA1-5 PPG data
- 5th: TIA1-8 PPG data
- 6th: TIA1-10 PPG data

With dynamic y-axis scaling for PPG channels
"""

import os
import time
import signal
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import platform
import sys

# Configuration
if platform.system() == 'Windows':
    BMI323_PIPE_PATH = r'\\.\pipe\bmi323_stream'
    AFE4950_PIPE_PATH = r'\\.\pipe\afe4950_stream'
else:
    BMI323_PIPE_PATH = "/tmp/bmi323_stream"
    AFE4950_PIPE_PATH = "/tmp/afe4950_stream"

WINDOW_SIZE = 18000  # Maximum number of points to store
DISPLAY_WINDOW = 10.0  # Display window in seconds (fixed to 10 seconds)
UPDATE_INTERVAL = 20  # Update interval in ms
DATA_TIMEOUT = 2.0  # Seconds without new data before reset

# Fixed y-axis limits for IMU
ACC_Y_MIN = -2.0
ACC_Y_MAX = 2.0
GYRO_Y_MIN = -250.0
GYRO_Y_MAX = 250.0

# Margin percentage for PPG plots (adds this percentage of the range above/below min/max)
PPG_MARGIN_PERCENT = 0.15

# Initialize tracking variables for PPG min/max values
tia3_min, tia3_max = float('inf'), float('-inf')
tia5_min, tia5_max = float('inf'), float('-inf')
tia8_min, tia8_max = float('inf'), float('-inf')
tia10_min, tia10_max = float('inf'), float('-inf')

# Initialize data storage with de-ques (efficient for appending/popping)
# BMI323 IMU data
imu_timestamps = deque(maxlen=WINDOW_SIZE)
acc_x_data = deque(maxlen=WINDOW_SIZE)
acc_y_data = deque(maxlen=WINDOW_SIZE)
acc_z_data = deque(maxlen=WINDOW_SIZE)
gyr_x_data = deque(maxlen=WINDOW_SIZE)
gyr_y_data = deque(maxlen=WINDOW_SIZE)
gyr_z_data = deque(maxlen=WINDOW_SIZE)
imu_last_update = 0  # Track when we last got IMU data

# AFE4950 PPG data
ppg_timestamps = deque(maxlen=WINDOW_SIZE)
tia1_3_data = deque(maxlen=WINDOW_SIZE)
tia1_5_data = deque(maxlen=WINDOW_SIZE)
tia1_8_data = deque(maxlen=WINDOW_SIZE)
tia1_10_data = deque(maxlen=WINDOW_SIZE)
ppg_last_update = 0  # Track when we last got PPG data

# Synchronization locks
imu_lock = threading.Lock()
ppg_lock = threading.Lock()

# Flag to control the animation
running = True

fig = plt.figure(figsize=(12, 8))
fig.suptitle("Mudra Ring Sensor Data", fontsize=12)  # Reduced title size

# Top-left subplot: Accelerometer
ax_acc = fig.add_subplot(6, 1, 1)
ax_acc.set_ylabel("g", fontsize=8)
ax_acc.tick_params(labelsize=8)
ax_acc.grid(True, alpha=0.4)
ax_acc.set_ylim(ACC_Y_MIN, ACC_Y_MAX)
ax_acc.set_xlim(0, DISPLAY_WINDOW)
ax_acc.text(0.01, 0.83, "ACC", transform=ax_acc.transAxes,
            fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.7))

# Top-right subplot: Gyroscope
ax_gyr = fig.add_subplot(6, 1, 2)
ax_gyr.set_ylabel("dps", fontsize=8)
ax_gyr.tick_params(labelsize=8)
ax_gyr.grid(True, alpha=0.4)
ax_gyr.set_ylim(GYRO_Y_MIN, GYRO_Y_MAX)
ax_gyr.set_xlim(0, DISPLAY_WINDOW)
ax_gyr.text(0.01, 0.83, "GYR", transform=ax_gyr.transAxes,
            fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.7))

# Bottom-left subplot: TIA1-3
ax_tia_1 = fig.add_subplot(6, 1, 3)
ax_tia_1.set_ylabel("Volt", fontsize=8)
ax_tia_1.tick_params(labelsize=8)
ax_tia_1.grid(True, alpha=0.4)
ax_tia_1.set_xlim(0, DISPLAY_WINDOW)
ax_tia_1.text(0.01, 0.83, "TIA1-3 520nm", transform=ax_tia_1.transAxes,
              fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.7))

# Bottom-right subplot: TIA1-5
ax_tia_2 = fig.add_subplot(6, 1, 4)
ax_tia_2.set_ylabel("Volt", fontsize=8)
ax_tia_2.tick_params(labelsize=8)
ax_tia_2.grid(True, alpha=0.4)
ax_tia_2.set_xlim(0, DISPLAY_WINDOW)
ax_tia_2.text(0.01, 0.83, "TIA1-5 940nm", transform=ax_tia_2.transAxes,
              fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.7))

# Bottom-right subplot: TIA1-8
ax_tia_3 = fig.add_subplot(6, 1, 5)
ax_tia_3.set_ylabel("Volt", fontsize=8)
ax_tia_3.tick_params(labelsize=8)
ax_tia_3.grid(True, alpha=0.4)
ax_tia_3.set_xlim(0, DISPLAY_WINDOW)
ax_tia_3.text(0.01, 0.83, "TIA1-8 810nm", transform=ax_tia_3.transAxes,
              fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.7))

# Bottom-right subplot: TIA1-10
ax_tia_4 = fig.add_subplot(6, 1, 6)
ax_tia_4.set_ylabel("Volt", fontsize=8)
ax_tia_4.set_xlabel("Seconds", fontsize=10)
ax_tia_4.tick_params(labelsize=8)
ax_tia_4.grid(True, alpha=0.4)
ax_tia_4.set_xlim(0, DISPLAY_WINDOW)
ax_tia_4.text(0.01, 0.83, "TIA1-10 660nm", transform=ax_tia_4.transAxes,
              fontsize=8, color='black', bbox=dict(facecolor='white', alpha=0.7))

# Initialize line objects with distinctive colors
acc_x_line, = ax_acc.plot([], [], '#FF5555', label='X', linewidth=1.0)
acc_y_line, = ax_acc.plot([], [], '#55FF55', label='Y', linewidth=1.0)
acc_z_line, = ax_acc.plot([], [], '#5555FF', label='Z', linewidth=1.0)

gyr_x_line, = ax_gyr.plot([], [], '#FF5555', label='X', linewidth=1.0)
gyr_y_line, = ax_gyr.plot([], [], '#55FF55', label='Y', linewidth=1.0)
gyr_z_line, = ax_gyr.plot([], [], '#5555FF', label='Z', linewidth=1.0)

tia3_line, = ax_tia_1.plot([], [], '#FF8888', label='TIA1-3', linewidth=1.0)
tia5_line, = ax_tia_2.plot([], [], '#8888FF', label='TIA1-5', linewidth=1.0)
tia8_line, = ax_tia_3.plot([], [], '#88FF88', label='TIA1-8', linewidth=1.0)
tia10_line, = ax_tia_4.plot([], [], '#FF88FF', label='TIA1-10', linewidth=1.0)

ax_acc.legend(loc='upper right', fontsize=6)
ax_gyr.legend(loc='upper right', fontsize=6)


def signal_handler(sig, frame):
    _, _ = sig, frame
    """Handle Ctrl+C gracefully"""
    global running
    print("\nStopping data visualization...")
    running = False


signal.signal(signal.SIGINT, signal_handler)


def reset_imu_data():
    """Reset IMU data when new stream starts or after timeout"""
    global imu_timestamps, acc_x_data, acc_y_data, acc_z_data, gyr_x_data, gyr_y_data, gyr_z_data

    imu_timestamps.clear()
    acc_x_data.clear()
    acc_y_data.clear()
    acc_z_data.clear()
    gyr_x_data.clear()
    gyr_y_data.clear()
    gyr_z_data.clear()

    # Reset the lines
    acc_x_line.set_data([], [])
    acc_y_line.set_data([], [])
    acc_z_line.set_data([], [])
    gyr_x_line.set_data([], [])
    gyr_y_line.set_data([], [])
    gyr_z_line.set_data([], [])


def reset_ppg_data():
    """Reset PPG data when new stream starts or after timeout"""
    global ppg_timestamps, tia1_3_data, tia1_5_data, tia1_8_data, tia1_10_data
    global tia3_min, tia3_max, tia5_min, tia5_max, tia8_min, tia8_max, tia10_min, tia10_max

    ppg_timestamps.clear()
    tia1_3_data.clear()
    tia1_5_data.clear()
    tia1_8_data.clear()
    tia1_10_data.clear()

    # Reset min/max values
    tia3_min, tia3_max = float('inf'), float('-inf')
    tia5_min, tia5_max = float('inf'), float('-inf')
    tia8_min, tia8_max = float('inf'), float('-inf')
    tia10_min, tia10_max = float('inf'), float('-inf')

    # Reset the lines
    tia3_line.set_data([], [])
    tia5_line.set_data([], [])
    tia8_line.set_data([], [])
    tia10_line.set_data([], [])


def init():
    """Initialize the animation"""
    # Initialize with empty data
    acc_x_line.set_data([], [])
    acc_y_line.set_data([], [])
    acc_z_line.set_data([], [])

    gyr_x_line.set_data([], [])
    gyr_y_line.set_data([], [])
    gyr_z_line.set_data([], [])

    tia3_line.set_data([], [])
    tia5_line.set_data([], [])
    tia8_line.set_data([], [])
    tia10_line.set_data([], [])

    return (acc_x_line, acc_y_line, acc_z_line,
            gyr_x_line, gyr_y_line, gyr_z_line,
            tia3_line, tia5_line, tia8_line, tia10_line)


def calculate_axis_limits(min_val, max_val, margin_percent=0.15, min_range=0.0001):
    """
    Calculate appropriate y-axis limits with a margin

    Parameters:
    min_val (float): Minimum data value
    max_val (float): Maximum data value
    margin_percent (float): Percentage of data range to add as margin
    min_range (float): Minimum range to enforce when min_val and max_val are close

    Returns:
    tuple: (y_min, y_max) limits for the y-axis
    """
    # Calculate data range
    data_range = max_val - min_val

    # Enforce a minimum range to prevent issues with nearly flat signals
    if data_range < min_range:
        mid_point = (min_val + max_val) / 2
        min_val = mid_point - min_range / 2
        max_val = mid_point + min_range / 2
        data_range = min_range

    # Calculate margin
    margin = data_range * margin_percent

    # Add margins
    y_min = min_val - margin
    y_max = max_val + margin

    return y_min, y_max


def check_timestamp_reset(timestamps, new_timestamp, threshold=1000000):
    """Check if timestamps have reset or jumped backward"""
    if len(timestamps) > 0:
        last_timestamp = timestamps[-1]
        # If new timestamp is significantly less than the last one, it's a reset
        if new_timestamp < last_timestamp - threshold:
            return True
    return False


def update(frame):
    _ = frame
    """Update function for animation"""
    global running, tia3_min, tia3_max, tia5_min, tia5_max, tia8_min, tia8_max, tia10_min, tia10_max
    global imu_last_update, ppg_last_update

    if not running:
        plt.close(fig)
        return (acc_x_line, acc_y_line, acc_z_line,
                gyr_x_line, gyr_y_line, gyr_z_line,
                tia3_line, tia5_line, tia8_line, tia10_line)

    current_time = time.time()

    # Check for IMU data timeout
    with imu_lock:
        # Check if we've received any IMU data
        if len(imu_timestamps) > 0:
            # If it's been too long since the last update, reset the data
            if current_time - imu_last_update > DATA_TIMEOUT:
                reset_imu_data()

    # Check for PPG data timeout
    with ppg_lock:
        # Check if we've received any PPG data
        if len(ppg_timestamps) > 0:
            # If it's been too long since the last update, reset the data
            if current_time - ppg_last_update > DATA_TIMEOUT:
                reset_ppg_data()

    # Update IMU data
    with imu_lock:
        try:
            if len(imu_timestamps) > 0:
                imu_time_data = [(t - imu_timestamps[0]) / 1_000_000 for t in imu_timestamps]

                # Calculate the relative time (always referenced to the latest timestamp)
                latest_time = imu_timestamps[-1]

                # Use a fixed window from 0 to DISPLAY_WINDOW seconds
                imu_window_start = 0
                imu_window_end = DISPLAY_WINDOW

                # Adjust the data time values to always be within 0-10 second window
                # by subtracting an offset that keeps the latest point at the appropriate position
                offset = (latest_time - imu_timestamps[0]) / 1_000_000 - DISPLAY_WINDOW
                if offset < 0:
                    offset = 0  # During the initial growing phase

                # Adjust time values to fit the fixed window
                imu_time_data = [t - offset for t in imu_time_data]

                # Find indices for the windowed data
                imu_window_indices = [i for i, t in enumerate(imu_time_data) if t >= imu_window_start]

                if imu_window_indices:
                    imu_windowed_time = [imu_time_data[i] for i in imu_window_indices]

                    # Window the data for each sensor
                    windowed_acc_x = [acc_x_data[i] for i in imu_window_indices]
                    windowed_acc_y = [acc_y_data[i] for i in imu_window_indices]
                    windowed_acc_z = [acc_z_data[i] for i in imu_window_indices]
                    windowed_gyr_x = [gyr_x_data[i] for i in imu_window_indices]
                    windowed_gyr_y = [gyr_y_data[i] for i in imu_window_indices]
                    windowed_gyr_z = [gyr_z_data[i] for i in imu_window_indices]

                    # Update accelerometer lines
                    acc_x_line.set_data(imu_windowed_time, windowed_acc_x)
                    acc_y_line.set_data(imu_windowed_time, windowed_acc_y)
                    acc_z_line.set_data(imu_windowed_time, windowed_acc_z)

                    # Update gyroscope lines
                    gyr_x_line.set_data(imu_windowed_time, windowed_gyr_x)
                    gyr_y_line.set_data(imu_windowed_time, windowed_gyr_y)
                    gyr_z_line.set_data(imu_windowed_time, windowed_gyr_z)

                # Keep the fixed y-axis limits
                ax_acc.set_ylim(ACC_Y_MIN, ACC_Y_MAX)
                ax_gyr.set_ylim(GYRO_Y_MIN, GYRO_Y_MAX)

        except Exception as e:
            print(f"Error updating IMU plot: {e}")

    # Update PPG data
    with ppg_lock:
        try:
            if len(ppg_timestamps) > 0:
                ppg_time_data = [(t - ppg_timestamps[0]) / 9_700_0 for t in ppg_timestamps]

                # Calculate the relative time (always referenced to the latest timestamp)
                latest_time = ppg_timestamps[-1]

                # Use a fixed window from 0 to DISPLAY_WINDOW seconds
                ppg_window_start = 0
                ppg_window_end = DISPLAY_WINDOW

                # Adjust the data time values to always be within 0-10 second window
                # by subtracting an offset that keeps the latest point at the appropriate position
                offset = (latest_time - ppg_timestamps[0]) / 9_700_0 - DISPLAY_WINDOW
                if offset < 0:
                    offset = 0  # During the initial growing phase

                # Adjust time values to fit the fixed window
                ppg_time_data = [t - offset for t in ppg_time_data]

                # Find indices for the windowed data
                ppg_window_indices = [i for i, t in enumerate(ppg_time_data) if t >= ppg_window_start]

                if ppg_window_indices:
                    ppg_windowed_time = [ppg_time_data[i] for i in ppg_window_indices]

                    # Window the data for each channel
                    windowed_tia3 = [tia1_3_data[i] for i in ppg_window_indices]
                    windowed_tia5 = [tia1_5_data[i] for i in ppg_window_indices]
                    windowed_tia8 = [tia1_8_data[i] for i in ppg_window_indices]
                    windowed_tia10 = [tia1_10_data[i] for i in ppg_window_indices]

                    # Update channel lines
                    tia3_line.set_data(ppg_windowed_time, windowed_tia3)
                    tia5_line.set_data(ppg_windowed_time, windowed_tia5)
                    tia8_line.set_data(ppg_windowed_time, windowed_tia8)
                    tia10_line.set_data(ppg_windowed_time, windowed_tia10)

                    # Get the min/max values for the CURRENT window and update y-axis limits and ticks
                    if windowed_tia3:
                        window_tia3_min = min(windowed_tia3)
                        window_tia3_max = max(windowed_tia3)
                        if window_tia3_min != window_tia3_max:  # Avoid flat lines causing issues
                            y_min, y_max = calculate_axis_limits(window_tia3_min, window_tia3_max, PPG_MARGIN_PERCENT)
                            ax_tia_1.set_ylim(y_min, y_max)

                    if windowed_tia5:
                        window_tia5_min = min(windowed_tia5)
                        window_tia5_max = max(windowed_tia5)
                        if window_tia5_min != window_tia5_max:
                            y_min, y_max = calculate_axis_limits(window_tia5_min, window_tia5_max, PPG_MARGIN_PERCENT)
                            ax_tia_2.set_ylim(y_min, y_max)

                    if windowed_tia8:
                        window_tia8_min = min(windowed_tia8)
                        window_tia8_max = max(windowed_tia8)
                        if window_tia8_min != window_tia8_max:
                            y_min, y_max = calculate_axis_limits(window_tia8_min, window_tia8_max, PPG_MARGIN_PERCENT)
                            ax_tia_3.set_ylim(y_min, y_max)

                    if windowed_tia10:
                        window_tia10_min = min(windowed_tia10)
                        window_tia10_max = max(windowed_tia10)
                        if window_tia10_min != window_tia10_max:
                            y_min, y_max = calculate_axis_limits(window_tia10_min, window_tia10_max, PPG_MARGIN_PERCENT)
                            ax_tia_4.set_ylim(y_min, y_max)

        except Exception as e:
            print(f"Error updating PPG plot: {e}")

    # Always ensure x-axis limits are correct
    ax_acc.set_xlim(0, DISPLAY_WINDOW)
    ax_gyr.set_xlim(0, DISPLAY_WINDOW)
    ax_tia_1.set_xlim(0, DISPLAY_WINDOW)
    ax_tia_2.set_xlim(0, DISPLAY_WINDOW)
    ax_tia_3.set_xlim(0, DISPLAY_WINDOW)
    ax_tia_4.set_xlim(0, DISPLAY_WINDOW)

    return (acc_x_line, acc_y_line, acc_z_line,
            gyr_x_line, gyr_y_line, gyr_z_line,
            tia3_line, tia5_line, tia8_line, tia10_line)


def read_imu_pipe_data():
    """Read data from the IMU named pipe"""
    global imu_timestamps, acc_x_data, acc_y_data, acc_z_data, gyr_x_data, gyr_y_data, gyr_z_data, imu_last_update
    
    try:
        if platform.system() == 'Windows':
            # Windows named pipe handling
            while running:
                try:
                    pipe = open(BMI323_PIPE_PATH, 'r')
                    print(f"Connected to BMI323 pipe: {BMI323_PIPE_PATH}")
                    
                    while running:
                        try:
                            line = pipe.readline().strip()
                            if line:
                                values = line.split(',')
                                if len(values) == 7:
                                    with imu_lock:
                                        timestamp = float(values[0])
                                        imu_timestamps.append(timestamp)
                                        acc_x_data.append(float(values[1]))
                                        acc_y_data.append(float(values[2]))
                                        acc_z_data.append(float(values[3]))
                                        gyr_x_data.append(float(values[4]))
                                        gyr_y_data.append(float(values[5]))
                                        gyr_z_data.append(float(values[6]))
                                        imu_last_update = time.time()
                        except Exception as e:
                            print(f"Error reading IMU line: {e}")
                            break
                            
                    pipe.close()
                    
                except Exception as e:
                    print(f"Error opening BMI323 pipe: {e}")
                    time.sleep(1)  # Wait before trying to reconnect
        else:
            # Unix named pipe handling (original code)
            with open(BMI323_PIPE_PATH, 'r') as pipe:
                while running:
                    try:
                        line = pipe.readline().strip()
                        if line:
                            values = [float(x) for x in line.split()]
                            if len(values) == 6:
                                with imu_lock:
                                    current_time = time.time()
                                    imu_timestamps.append(current_time)
                                    acc_x_data.append(values[0])
                                    acc_y_data.append(values[1])
                                    acc_z_data.append(values[2])
                                    gyr_x_data.append(values[3])
                                    gyr_y_data.append(values[4])
                                    gyr_z_data.append(values[5])
                                    imu_last_update = current_time
                    except Exception as e:
                        print(f"Error reading IMU data: {e}")
                        time.sleep(0.1)
    except Exception as e:
        print(f"Error opening IMU pipe: {e}")


def read_ppg_pipe_data():
    """Read data from the PPG named pipe"""
    global ppg_timestamps, tia1_3_data, tia1_5_data, tia1_8_data, tia1_10_data, ppg_last_update
    
    try:
        if platform.system() == 'Windows':
            # Windows named pipe handling
            while running:
                try:
                    pipe = open(AFE4950_PIPE_PATH, 'r')
                    print(f"Connected to AFE4950 pipe: {AFE4950_PIPE_PATH}")
                    
                    while running:
                        try:
                            line = pipe.readline().strip()
                            if line:
                                values = line.split(',')
                                if len(values) == 5:
                                    with ppg_lock:
                                        timestamp = float(values[0])
                                        ppg_timestamps.append(timestamp)
                                        tia1_3_data.append(float(values[1]))
                                        tia1_5_data.append(float(values[2]))
                                        tia1_8_data.append(float(values[3]))
                                        tia1_10_data.append(float(values[4]))
                                        ppg_last_update = time.time()
                        except Exception as e:
                            print(f"Error reading PPG line: {e}")
                            break
                            
                    pipe.close()
                    
                except Exception as e:
                    print(f"Error opening AFE4950 pipe: {e}")
                    time.sleep(1)  # Wait before trying to reconnect
        else:
            # Unix named pipe handling (original code)
            with open(AFE4950_PIPE_PATH, 'r') as pipe:
                while running:
                    try:
                        line = pipe.readline().strip()
                        if line:
                            values = [float(x) for x in line.split()]
                            if len(values) == 4:
                                with ppg_lock:
                                    current_time = time.time()
                                    ppg_timestamps.append(current_time)
                                    tia1_3_data.append(values[0])
                                    tia1_5_data.append(values[1])
                                    tia1_8_data.append(values[2])
                                    tia1_10_data.append(values[3])
                                    ppg_last_update = current_time
                    except Exception as e:
                        print(f"Error reading PPG data: {e}")
                        time.sleep(0.1)
    except Exception as e:
        print(f"Error opening PPG pipe: {e}")


if __name__ == "__main__":
    # Start the data reading threads
    imu_thread = threading.Thread(target=read_imu_pipe_data)
    imu_thread.daemon = True
    imu_thread.start()

    ppg_thread = threading.Thread(target=read_ppg_pipe_data)
    ppg_thread.daemon = True
    ppg_thread.start()

    # Start the animation
    ani = FuncAnimation(fig, update, init_func=init, interval=UPDATE_INTERVAL, blit=False)

    print("Starting visualization... (Press Ctrl+C to exit)")
    try:
        plt.tight_layout()
        plt.subplots_adjust(top=0.92, hspace=0.25, wspace=0.25)  # Optimized spacing
        plt.show()
    except KeyboardInterrupt:
        pass

    print("Visualization stopped.")
    running = False