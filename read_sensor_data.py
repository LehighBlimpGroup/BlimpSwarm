import pickle as pkl
import numpy as np

data = pkl.load(open("sensor_data.pkl", "rb"))
# imu0, input0, mocap0 = data[0]
# print(data)
# # print(data[1])
# # print(data[2])
# # print(imu0)
# # imu0 = imu0.copy()
# new_data = []
# for imu, input, mocap in data:
#     imu_rel = tuple(x - y for x, y in zip(imu, imu0))
#     input_rel = tuple(x - y for x, y in zip(input, input0))
#     mocap_rel = tuple(x - y for x, y in zip(mocap, mocap0))
#     new_data.append((imu_rel, input_rel, mocap_rel))


# data = new_data


print(data)
