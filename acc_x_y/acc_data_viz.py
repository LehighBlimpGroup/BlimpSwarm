import pickle as pkl
import matplotlib.pyplot as plt
import numpy as np

data = pkl.load(open("1_d_robot_acc_data_1765229795.6465828.pkl", "rb"))
print(len(data))
timestamps = []
x_acceleration = []
y_acceleration = []
for timestamp, acceleration in data:
    timestamps.append(timestamp)
    x_acceleration.append(acceleration[0])
    y_acceleration.append(acceleration[1])

# Calculate differences between consecutive timestamps
timestamps = np.array(timestamps)
timestamp_diffs = np.diff(timestamps)

# Calculate and print the average timestamp difference
avg_time_diff = np.mean(timestamp_diffs)
print(f"Average timestamp difference between consecutive data points: {avg_time_diff:.6f} seconds")

# Plot timestamp differences
plt.figure(figsize=(10, 6))
plt.plot(timestamp_diffs)
plt.xlabel("Data Point Index")
plt.ylabel("Timestamp Difference (seconds)")
plt.title("Timestamp Differences Between Consecutive Data Points")
plt.grid(True, alpha=0.3)
plt.axhline(y=avg_time_diff, color='r', linestyle='--', label=f'Average: {avg_time_diff:.6f} s')
plt.legend()
plt.show()


# plt.plot(timestamps, x_acceleration, label="X Acceleration")
# plt.plot(timestamps, y_acceleration, label="Y Acceleration")
# plt.xlabel("Time (s)")
# plt.ylabel("Acceleration")
# plt.legend()
# plt.show()