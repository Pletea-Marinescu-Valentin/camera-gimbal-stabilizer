import matplotlib.pyplot as plt
import numpy as np
import re
import os

# Function to parse the data from the console log
def parse_data(log_text):
    # Regular expressions to extract data
    raw_accel_pattern = r"Raw Accel X:\s+(-?\d+), Y:\s+(-?\d+), Z:\s+(-?\d+)"
    raw_gyro_pattern = r"Raw Gyro\s+X:\s+(-?\d+), Y:\s+(-?\d+), Z:\s+(-?\d+)"
    filtered_pattern = r"Filtered X: (-?\d+\.\d+), Y: (-?\d+\.\d+)"
    temp_pattern = r"Temp: (\d+\.\d+) C"
    
    # Extract data using regex
    raw_accel = re.findall(raw_accel_pattern, log_text)
    raw_gyro = re.findall(raw_gyro_pattern, log_text)
    filtered = re.findall(filtered_pattern, log_text)
    temps = re.findall(temp_pattern, log_text)
    
    # Convert to appropriate data types
    raw_accel_x = [int(x[0]) for x in raw_accel]
    raw_accel_y = [int(x[1]) for x in raw_accel]
    raw_accel_z = [int(x[2]) for x in raw_accel]
    
    raw_gyro_x = [int(x[0]) for x in raw_gyro]
    raw_gyro_y = [int(x[1]) for x in raw_gyro]
    raw_gyro_z = [int(x[2]) for x in raw_gyro]
    
    filtered_x = [float(x[0]) for x in filtered]
    filtered_y = [float(x[1]) for x in filtered]
    
    temperatures = [float(x) for x in temps]
    
    # Calculate raw angles from accelerometer data (for comparison)
    # This mimics how the angles are calculated in the firmware
    accel_angle_x = [np.arctan2(y, z) * 180 / np.pi for y, z in zip(raw_accel_y, raw_accel_z)]
    accel_angle_y = [np.arctan2(-x, np.sqrt(y*y + z*z)) * 180 / np.pi 
                    for x, y, z in zip(raw_accel_x, raw_accel_y, raw_accel_z)]
    
    return {
        'raw_accel': {
            'x': raw_accel_x,
            'y': raw_accel_y,
            'z': raw_accel_z
        },
        'raw_gyro': {
            'x': raw_gyro_x,
            'y': raw_gyro_y,
            'z': raw_gyro_z
        },
        'filtered': {
            'x': filtered_x,
            'y': filtered_y
        },
        'accel_angle': {
            'x': accel_angle_x,
            'y': accel_angle_y
        },
        'temperature': temperatures,
        'sample_count': len(filtered_x)
    }

def create_kalman_stability_analysis(data):
    # Calculate the difference between raw accel angles and filtered angles
    diff_x = np.array(data['accel_angle']['x']) - np.array(data['filtered']['x'])
    diff_y = np.array(data['accel_angle']['y']) - np.array(data['filtered']['y'])
    
    # Calculate moving average of differences
    window_size = 5
    diff_x_smooth = np.convolve(diff_x, np.ones(window_size)/window_size, mode='valid')
    diff_y_smooth = np.convolve(diff_y, np.ones(window_size)/window_size, mode='valid')
    
    # Calculate standard deviation of differences (noise reduction measure)
    std_x = np.std(diff_x)
    std_y = np.std(diff_y)
    
    time = np.arange(len(diff_x))
    time_smooth = np.arange(len(diff_x_smooth))
    
    # Create figure for stability analysis
    plt.figure(figsize=(12, 10))
    
    # Plot difference between raw and filtered for X axis
    plt.subplot(2, 1, 1)
    plt.plot(time, diff_x, 'r-', alpha=0.5, label='Instantaneous Difference')
    plt.plot(time_smooth, diff_x_smooth, 'b-', linewidth=2, label='Smoothed Difference')
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.title(f'Kalman Filter Noise Reduction - X Axis (StdDev: {std_x:.2f}°)', fontsize=14)
    plt.xlabel('Sample Number')
    plt.ylabel('Angle Difference (degrees)')
    plt.legend()
    plt.grid(True)
    
    # Plot difference between raw and filtered for Y axis
    plt.subplot(2, 1, 2)
    plt.plot(time, diff_y, 'r-', alpha=0.5, label='Instantaneous Difference')
    plt.plot(time_smooth, diff_y_smooth, 'b-', linewidth=2, label='Smoothed Difference')
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.title(f'Kalman Filter Noise Reduction - Y Axis (StdDev: {std_y:.2f}°)', fontsize=14)
    plt.xlabel('Sample Number')
    plt.ylabel('Angle Difference (degrees)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    
    # Save figure to Images folder
    plt.savefig(os.path.join('Images', 'kalman_filter_stability.png'), dpi=300)
    plt.show()

def main():
    # Read the console log data (replace with your actual file path)
    with open('sensor_data.txt', 'r') as file:
        log_text = file.read()
    
    # Parse the data
    data = parse_data(log_text)
    
    create_kalman_stability_analysis(data)
    
    print(f"Analysis complete! {data['sample_count']} samples processed.")
    print(f"X-axis range: {min(data['filtered']['x']):.2f}° to {max(data['filtered']['x']):.2f}°")
    print(f"Y-axis range: {min(data['filtered']['y']):.2f}° to {max(data['filtered']['y']):.2f}°")

if __name__ == "__main__":
    main()