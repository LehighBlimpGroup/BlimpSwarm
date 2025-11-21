import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt

# Load the sensor data
data = pkl.load(open("sensor_data.pkl", "rb"))

# Extract the 0th data point for normalization
timestamp0, sensor0, mocap0 = data[0]

# Normalize all data by subtracting the initial values
normalized_data = []
for timestamp, sensor_array, mocap_array in data:
    # Normalize timestamp (make it relative to start, starting at 0)
    normalized_timestamp = timestamp #- timestamp0
    
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
sensor_pitch = np.array([entry[1][2] for entry in data])  # pitch is at index 2
mocap_pitch = np.array([entry[2][4] for entry in data])   # pitch is at index 4

# Create two plots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Plot 1: Pitch angles vs time
ax1.plot(timestamps, sensor_pitch, label='Normalized Sensor Pitch', linewidth=2)
ax1.plot(timestamps, mocap_pitch, label='Normalized Mocap Pitch (GT)', linewidth=2)
ax1.set_xlabel('Timestamp', fontsize=12)
ax1.set_ylabel('Pitch Angle (degrees)', fontsize=12)
ax1.set_title('Pitch Angle vs Time', fontsize=14, fontweight='bold')
ax1.legend(fontsize=11)
ax1.grid(True, alpha=0.3)

# Plot 2: Sin of pitch angles vs time
ax2.plot(timestamps, np.sin(np.deg2rad(sensor_pitch)), label='sin(Sensor Pitch)', linewidth=2)
ax2.plot(timestamps, np.sin(np.deg2rad(mocap_pitch)), label='sin(Mocap Pitch)', linewidth=2)
ax2.set_xlabel('Timestamp', fontsize=12)
ax2.set_ylabel('sin(Pitch Angle)', fontsize=12)
ax2.set_title('Sin of Pitch Angles vs Time', fontsize=14, fontweight='bold')
ax2.legend(fontsize=11)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()


