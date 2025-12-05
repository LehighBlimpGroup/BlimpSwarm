import pickle as pkl
import matplotlib.pyplot as plt

data = pkl.load(open("acc_data_3.pkl", "rb"))
timestamps = []
x_acceleration = []
y_acceleration = []
for timestamp, acceleration in data:
    timestamps.append(timestamp)
    x_acceleration.append(acceleration[0])
    y_acceleration.append(acceleration[1])

plt.plot(timestamps, x_acceleration, label="X Acceleration")
plt.plot(timestamps, y_acceleration, label="Y Acceleration")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration")
plt.legend()
plt.show()