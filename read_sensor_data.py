import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt

# Load the sensor data
data = pkl.load(open("sensor_data1.pkl", "rb"))

# Extract the 0th data point for normalization
timestamp0, sensor0, mocap0 = data[0]

# Normalize all data by subtracting the initial values
normalized_data = []
for timestamp, sensor_array, mocap_array in data:
    # Normalize timestamp (make it relative to start, starting at 0)
    normalized_timestamp = timestamp  # - timestamp0

    # Normalize sensor data (subtract initial sensor values)
    normalized_sensor = sensor_array - sensor0

    # Normalize mocap data (subtract initial mocap values)
    normalized_mocap = mocap_array - mocap0

    normalized_data.append((normalized_timestamp, normalized_sensor, normalized_mocap))

# Replace original data with normalized data
data = normalized_data

print(f"Loaded {len(data)} data points")
print(f"First data point (should be all zeros): {data[0]}")
print(f"Second data point: {data[1]}")

# Extract data for plotting
# sensor_array: [height, roll, pitch, yaw, motor_1, motor_2, servo_angle_deg]
# mocap_array: [pos_x, pos_y, pos_z, rot_roll, rot_pitch, rot_yaw]
timestamps = np.array([entry[0] for entry in data])
# Raw sensor pitch (assumed radians); convert to degrees.
sensor_pitch_rad = np.array([entry[1][2] for entry in data])  # pitch is at index 2
sensor_pitch = np.degrees(sensor_pitch_rad)  # now in degrees
mocap_pitch = np.array(
    [entry[2][4] for entry in data]
)  # pitch is at index 4 (assumed already degrees)

# Extract motor data (motor_1 at index 4, motor_2 at index 5)
motor_1 = np.array([entry[1][4] for entry in data])
motor_2 = np.array([entry[1][5] for entry in data])

# Filter out invalid angle values: anything outside [-180, 180] degrees or non-finite values
# Both `sensor_pitch` and `mocap_pitch` are treated as degrees here. If your sensor
# pitch were still in radians, you'd compare to np.pi instead.
valid_mask = (
    np.isfinite(sensor_pitch)
    & np.isfinite(mocap_pitch)
    & (np.abs(sensor_pitch) <= 180.0)
    & (np.abs(mocap_pitch) <= 180.0)
)
num_filtered = np.count_nonzero(~valid_mask)
if num_filtered > 0:
    print(
        f"Filtered out {num_filtered} data points with out-of-range angles (>180° or <-180° or NaN)"
    )

# Apply filter to timestamps and angle arrays used for plotting
timestamps = timestamps[valid_mask]
sensor_pitch = sensor_pitch[valid_mask]
mocap_pitch = mocap_pitch[valid_mask]

# Create two plots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Plot 1: Pitch angles vs time
ax1.plot(timestamps, sensor_pitch, label="Normalized Sensor Pitch (deg)", linewidth=2)
ax1.plot(timestamps, mocap_pitch, label="Normalized Mocap Pitch (GT)", linewidth=2)
ax1.set_xlabel("Timestamp", fontsize=12)
ax1.set_ylabel("Pitch Angle (degrees)", fontsize=12)
ax1.set_title("Pitch Angle vs Time", fontsize=14, fontweight="bold")
ax1.legend(fontsize=11)
ax1.grid(True, alpha=0.3)

# Plot 2: Sin of pitch angles vs time
ax2.plot(
    timestamps, np.sin(np.deg2rad(sensor_pitch)), label="sin(Sensor Pitch)", linewidth=2
)
ax2.plot(
    timestamps, np.sin(np.deg2rad(mocap_pitch)), label="sin(Mocap Pitch)", linewidth=2
)
ax2.set_xlabel("Timestamp", fontsize=12)
ax2.set_ylabel("sin(Pitch Angle)", fontsize=12)
ax2.set_title("Sin of Pitch Angles vs Time", fontsize=14, fontweight="bold")
ax2.legend(fontsize=11)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Create distribution plots for motor data
fig2, (ax3, ax4, ax5) = plt.subplots(1, 3, figsize=(15, 5))

# Histogram for motor_1
ax3.hist(motor_1, bins=50, alpha=0.7, color='blue', edgecolor='black')
ax3.set_xlabel("Motor 1 Value", fontsize=12)
ax3.set_ylabel("Frequency", fontsize=12)
ax3.set_title("Motor 1 Distribution", fontsize=14, fontweight="bold")
ax3.grid(True, alpha=0.3)
ax3.axvline(np.mean(motor_1), color='red', linestyle='--', linewidth=2, label=f'Mean: {np.mean(motor_1):.3f}')
ax3.axvline(np.median(motor_1), color='green', linestyle='--', linewidth=2, label=f'Median: {np.median(motor_1):.3f}')
ax3.legend(fontsize=10)
ax3.text(0.05, 0.95, f'Std: {np.std(motor_1):.3f}\nMin: {np.min(motor_1):.3f}\nMax: {np.max(motor_1):.3f}', 
         transform=ax3.transAxes, fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# Histogram for motor_2
ax4.hist(motor_2, bins=50, alpha=0.7, color='orange', edgecolor='black')
ax4.set_xlabel("Motor 2 Value", fontsize=12)
ax4.set_ylabel("Frequency", fontsize=12)
ax4.set_title("Motor 2 Distribution", fontsize=14, fontweight="bold")
ax4.grid(True, alpha=0.3)
ax4.axvline(np.mean(motor_2), color='red', linestyle='--', linewidth=2, label=f'Mean: {np.mean(motor_2):.3f}')
ax4.axvline(np.median(motor_2), color='green', linestyle='--', linewidth=2, label=f'Median: {np.median(motor_2):.3f}')
ax4.legend(fontsize=10)
ax4.text(0.05, 0.95, f'Std: {np.std(motor_2):.3f}\nMin: {np.min(motor_2):.3f}\nMax: {np.max(motor_2):.3f}', 
         transform=ax4.transAxes, fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# Scatter plot comparing motor_1 vs motor_2
ax5.scatter(motor_1, motor_2, alpha=0.5, s=10)
ax5.set_xlabel("Motor 1 Value", fontsize=12)
ax5.set_ylabel("Motor 2 Value", fontsize=12)
ax5.set_title("Motor 1 vs Motor 2", fontsize=14, fontweight="bold")
ax5.grid(True, alpha=0.3)
# Add correlation coefficient
correlation = np.corrcoef(motor_1, motor_2)[0, 1]
ax5.text(0.05, 0.95, f'Correlation: {correlation:.3f}', 
         transform=ax5.transAxes, fontsize=12, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
plt.show()

# Print summary statistics
print("\n" + "="*60)
print("Motor Data Summary Statistics")
print("="*60)
print(f"Motor 1 - Mean: {np.mean(motor_1):.4f}, Std: {np.std(motor_1):.4f}, Min: {np.min(motor_1):.4f}, Max: {np.max(motor_1):.4f}")
print(f"Motor 2 - Mean: {np.mean(motor_2):.4f}, Std: {np.std(motor_2):.4f}, Min: {np.min(motor_2):.4f}, Max: {np.max(motor_2):.4f}")
print(f"Correlation between Motor 1 and Motor 2: {correlation:.4f}")
print("="*60)
