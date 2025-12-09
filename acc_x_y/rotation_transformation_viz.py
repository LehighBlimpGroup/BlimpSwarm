import numpy as np

# ---------- Low-level helpers ----------

def _Rx(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0],
                     [0, ca, -sa],
                     [0, sa,  ca]])

def _Ry(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ ca, 0, sa],
                     [  0, 1,  0],
                     [-sa, 0, ca]])

def _Rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0],
                     [sa,  ca, 0],
                     [ 0,   0, 1]])

def _R_mocap(roll_gY, pitch_gX, yaw_gZ):
    """
    Build rotation from mocap global angles:
    - roll about global Y
    - pitch about global X
    - yaw about global Z

    Applied as extrinsic rotations in order: Y (roll), X (pitch), Z (yaw).
    Result: v_world = R @ v_body
    """
    return _Rz(yaw_gZ) @ _Rx(pitch_gX) @ _Ry(roll_gY)

def _wrap_angle(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi

def _euler_zyx_from_R(R):
    """
    Extract intrinsic ZYX Euler angles (yaw, pitch, roll) from rotation
    assuming: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    # clamp for numerical safety
    R20 = np.clip(-R[2, 0], -1.0, 1.0)

    yaw   = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arcsin(R20)
    roll  = np.arctan2(R[2, 1], R[2, 2])
    return yaw, pitch, roll


# ---------- Main conversion function ----------

def mocap_global_rpy_to_body_frame(mocap_data, t0_index=0, angles_in_degrees=True):
    """
    Convert mocap global RPY (roll about global Y, pitch about global X,
    yaw about global Z) to *body-frame/local* RPY relative to pose at t0.

    Parameters
    ----------
    mocap_data : array-like, shape (T, 6)
        Each row: [pos_x, pos_y, pos_z, roll_gY, pitch_gX, yaw_gZ]
    t0_index : int
        Index of the reference sample (IMU "zero" time).
    angles_in_degrees : bool
        True if mocap angles are in degrees.

    Returns
    -------
    body_rpy_mocap : ndarray, shape (T, 3)
        For each timestep t:
        [pitch_body_from_mocap, roll_body_from_mocap, yaw_body_from_mocap]
        in the same convention/order as your sensor_array.
        Units: degrees if angles_in_degrees=True, else radians.
    """
    mocap_data = np.asarray(mocap_data)
    assert mocap_data.shape[1] == 6, "mocap_data must have shape (T, 6)"

    # Extract angle columns
    roll_gY  = mocap_data[:, 3]
    pitch_gX = mocap_data[:, 4]
    yaw_gZ   = mocap_data[:, 5]

    # Convert to radians internally
    if angles_in_degrees:
        deg2rad = np.pi / 180.0
        roll_gY_rad  = roll_gY  * deg2rad
        pitch_gX_rad = pitch_gX * deg2rad
        yaw_gZ_rad   = yaw_gZ   * deg2rad
    else:
        roll_gY_rad  = roll_gY
        pitch_gX_rad = pitch_gX
        yaw_gZ_rad   = yaw_gZ

    # Reference orientation at t0 (IMU "zero")
    roll0  = roll_gY_rad[t0_index]
    pitch0 = pitch_gX_rad[t0_index]
    yaw0   = yaw_gZ_rad[t0_index]
    R0 = _R_mocap(roll0, pitch0, yaw0)

    T = mocap_data.shape[0]
    body_rpy = np.zeros((T, 3))  # [pitch_body, roll_body, yaw_body]

    for t in range(T):
        R_t = _R_mocap(roll_gY_rad[t], pitch_gX_rad[t], yaw_gZ_rad[t])

        # Relative rotation since t0
        R_rel = R0.T @ R_t

        # Get yaw, pitch, roll (body frame, intrinsic ZYX)
        yaw_b, pitch_b, roll_b = _euler_zyx_from_R(R_rel)

        # Wrap to (-pi, pi]
        yaw_b   = _wrap_angle(yaw_b)
        pitch_b = _wrap_angle(pitch_b)
        roll_b  = _wrap_angle(roll_b)

        # Store in your sensor ordering: [pitch_body, roll_body, yaw_body]
        body_rpy[t, 0] = pitch_b
        body_rpy[t, 1] = roll_b
        body_rpy[t, 2] = yaw_b

    # Convert back to degrees if needed
    if angles_in_degrees:
        rad2deg = 180.0 / np.pi
        body_rpy *= rad2deg

    return body_rpy


# ---------- Integrated "align and overwrite" helper ----------

def align_mocap_with_sensor(sensor_data, mocap_data,
                            t0_index=0, angles_in_degrees=True):
    """
    Given sensor_data and mocap_data, use mocap at t0 to define the
    local/body frame, convert global mocap RPY to body-frame RPY,
    and overwrite sensor_data[:, 1:4] with those angles.

    Parameters
    ----------
    sensor_data : array-like, shape (T, 7)
        [height, pitch_body_imu, roll_body_imu, yaw_body_imu,
         motor_1, motor_2, servo_angle_deg]
    mocap_data : array-like, shape (T, 6)
        [pos_x, pos_y, pos_z, roll_gY, pitch_gX, yaw_gZ]
    t0_index : int
        Reference index for defining the local frame.
    angles_in_degrees : bool
        Units of mocap angles.

    Returns
    -------
    sensor_data_mocap : ndarray, shape (T, 7)
        Copy of sensor_data where columns 1:4
        are replaced by mocap-derived body RPY.
    body_rpy_from_mocap : ndarray, shape (T, 3)
        The computed [pitch_body, roll_body, yaw_body] from mocap.
    """
    sensor_data = np.asarray(sensor_data)
    mocap_data  = np.asarray(mocap_data)

    assert sensor_data.shape[0] == mocap_data.shape[0], \
        "sensor_data and mocap_data must have same length along time axis"
    assert sensor_data.shape[1] == 7, "sensor_data must have shape (T, 7)"

    body_rpy_from_mocap = mocap_global_rpy_to_body_frame(
        mocap_data,
        t0_index=t0_index,
        angles_in_degrees=angles_in_degrees
    )

    sensor_data_mocap = sensor_data.copy()
    sensor_data_mocap[:, 1:4] = body_rpy_from_mocap

    return sensor_data_mocap, body_rpy_from_mocap


import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt

# Load the sensor data
data = pkl.load(open("sensor_data1.pkl", "rb")) 
sensor_data_arr = []
mocap_data_arr = []
for timestamp, sensor_array, mocap_array in data:
    sensor_data_arr.append(sensor_array)
    mocap_array.append(mocap_array)

sensor_data_mocap = align_mocap_with_sensor(sensor_data_arr,mocap_array)
