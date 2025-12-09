from comm.Serial import DataType_Boolean
from input.JoystickManager import JoystickManager
from user_parameters import ROBOT_MACS, TENSILE_MASTER, TENSILE_FOLLOWERS, DEFENDER_MACS
from robot.RobotMaster import RobotMaster
import Preferences
import time
import importlib
import pickle as pkl
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler

PRINT_JOYSTICK = False

positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)


def startAutonomousBall(serial, robot, args):
    serial.send_control_params(robot, (3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.2)
    serial.send_control_params(robot, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.2)
    return 2


def startAutonomousGoal(serial, robot, args):
    serial.send_control_params(robot, (4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.2)
    serial.send_control_params(robot, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.2)
    return 2


def stopOne(serial, robot, args):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.05)
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.05)
    return 0


def sendCalibrate(serial, robot, args):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(0.05)
    serial.send_preference(robot, DataType_Boolean, "calibrate", True)
    time.sleep(0.05)
    return -1


def controlTensileFollowers(serial, robot, args):
    serial.send_control_params(
        robot, (args[0], args[0], args[1], args[1], 0, 0, 0, 0, 0, 0, 0, 0, 0)
    )
    time.sleep(0.05)
    return 1


def sendPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["ff:ff:ff:ff:ff:ff"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1


def sendOrangePreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["orange"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1


def sendYellowPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["yellow"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1


def sendDefenderPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["defender"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1


def sendAttackerPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["attacker"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1


def main():
    try:
        robot_master = RobotMaster(0.3)
        joystick = JoystickManager()
        macs = ROBOT_MACS + DEFENDER_MACS + TENSILE_MASTER + TENSILE_FOLLOWERS
        robot_master.setup(macs, "nicla")
        followers = [
            i + 1 for i in list(range(len(macs) - len(TENSILE_FOLLOWERS), len(macs)))
        ]

        robot_master.functionFactory("s", stopOne, "Stop")
        robot_master.functionFactory("b", startAutonomousBall, "Auto Ball")
        robot_master.functionFactory("g", startAutonomousGoal, "Auto Goal")
        robot_master.functionFactory("c", sendCalibrate, "Calibrate")
        robot_master.functionFactory("p", sendPreferences, "Send Preferences")
        robot_master.functionFactory(
            "a", sendAttackerPreferences, "Send Attacker Preferences"
        )
        robot_master.functionFactory(
            "d", sendDefenderPreferences, "Send Defender Preferences"
        )
        robot_master.functionFactory(
            "f", controlTensileFollowers, "Controlling Follower"
        )
        robot_master.functionFactory(
            "y", sendYellowPreferences, "Send Yellow Preferences"
        )
        robot_master.functionFactory(
            "o", sendOrangePreferences, "Send Orange Preferences"
        )

        index = "0"
        power = 0.45
        angle = 45
        # Store data as list of tuples: (timestamp, sensor_array, mocap_array)
        # sensor_array shape: (7,) - [height, roll, pitch, yaw, motor_1, motor_2, servo_angle_deg]
        # mocap_array shape: (6,) - [pos_x, pos_y, pos_z, rot_roll, rot_pitch, rot_yaw]
        sensor_data_all = []

        # Mocap data

        clientAddress = "192.168.0.21"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 525

        # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()

        time_collected = time.time()
        sensor_data_old = None
        sensor_data = None
        time_start = time.time()

        while True:
            time.sleep(0.01)
            keys = robot_master.get_last_n_keys(1)
            axis, buttons = joystick.getJoystickInputs()
            sensor_data = robot_master.processManual(axis, buttons, print_vals=False)
            mocap = positions[robot_id] + rotations[robot_id]
            if (
                True
            ):  # or (sensor_data != sensor_data_old and sensor_data is not None) : #and time.time() - time_collected >= 0.05:
                time_curr = time.time() - time_start

                # Store data as numpy arrays for efficient numerical operations
                # Format: (timestamp, sensor_array, mocap_array)
                # sensor_array: [height, pitch_body, - roll_body, yaw_body, pitch rate, - roll rate, yaw rate, y acceleration, - x acceleration, z acceleration, motor_1, motor_2, servo_angle_deg]
                # mocap_array: [pos_x, pos_y, pos_z, rot_roll(abt global y), rot_pitch(abt global x), rot_yaw(about global z)]
                data_entry = (
                    time_curr,
                    np.array(sensor_data, dtype=np.float32),
                    np.array(mocap, dtype=np.float32),
                )
                time_collected = time.time()
                # acc x
                # print("sensor pitch", sensor_data[1], "sensor roll", sensor_data[2], "sensor yaw", sensor_data[3],
                # "sensor roll rate", sensor_data[4], "sensor pitch rate", sensor_data[5], "sensor yaw rate", sensor_data[6],
                # "mocap pitch", mocap[3], "mocap roll", mocap[4], "mocap yaw", mocap[5], "Time: ", time_curr)  # Print pitch values for comparison

                # print("sensor acc x", sensor_data[7], "sensor acc y", sensor_data[8], "sensor acc z", sensor_data[9])
                # print("sensor pitch", sensor_data[1], "sensor roll", sensor_data[2], "sensor yaw", sensor_data[3], "pitch rate", sensor_data[4], "roll rate", sensor_data[5], "yaw rate", sensor_data[6])
                # print("mocap pitch", mocap[3], "mocap roll", mocap[4], "mocap yaw", mocap[5])
                # print("Time: ", time_curr)
                sensor_data_all.append(data_entry)
                # time_old = time.time()
            sensor_data_old = sensor_data
            power = min(1, max(0, power))
            angle = min(180, max(-180, angle))

            # Display joystick values if enabled
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            if len(keys) == 0:
                continue

            key_pressed = keys[0]

            if key_pressed == "enter":
                i = int(index)
                robot_master.switchRobot(i)
                index = "0"
            elif len(key_pressed) == 1 and 32 <= ord(key_pressed) <= 127:
                if "0" <= key_pressed <= "9":
                    index += key_pressed
                elif key_pressed == "[":
                    for i in followers:
                        robot_master.runFunction("f", i, 0, 0)
                elif key_pressed == "]":
                    for i in followers:
                        robot_master.runFunction("f", i, power, angle)
                elif key_pressed == "=":
                    angle += 5
                    for i in followers:
                        robot_master.runFunction("f", i, power, angle)
                elif key_pressed == "-":
                    angle -= 5
                    for i in followers:
                        robot_master.runFunction("f", i, power, angle)
                elif key_pressed == ";":
                    power -= 0.05
                    for i in followers:
                        robot_master.runFunction("f", i, power, angle)
                elif key_pressed == "'":
                    power += 0.05
                    for i in followers:
                        robot_master.runFunction("f", i, power, angle)
                elif key_pressed == "s":
                    power = 0
                    angle = 45
                    i = int(index)
                    robot_master.runFunction(key_pressed, i)
                    index = "0"
                else:
                    i = int(index)
                    robot_master.runFunction(key_pressed, i)
                    index = "0"
            else:
                print("Invalid button.")
    except KeyboardInterrupt:
        print("Stopping!")
        timestamp = time.time()
        pkl.dump(
            sensor_data_all,
            open(f"training_data_dupes/dupes_9_sleep_0.05_{timestamp}.pkl", "wb"),
        )
        return
    except Exception as e:
        print(e)
        return


if __name__ == "__main__":
    main()
