from comm.Serial import DataType_Boolean
from input.JoystickManager import JoystickManager
from user_parameters import ROBOT_MACS
from robot.RobotMaster import RobotMaster
import Preferences
import time
import importlib

PRINT_JOYSTICK = False


def startAutonomous(serial, robot):
    serial.send_control_params(robot, (3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_control_params(robot, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 2


def stopOne(serial, robot):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 0


def startG1(serial, robot):
    serial.send_control_params(robot, (1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 2


def sendConstant(serial, robot):
    serial.send_control_params(robot, (1, .2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 1


def sendCalibrate(serial, robot):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_preference(robot, DataType_Boolean, "calibrate", True)
    time.sleep(.05)
    return -1


def sendPreferences(serial, robot):
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


def main():
    try:
        robot_master = RobotMaster(0.3)
        joystick = JoystickManager()
        robot_master.setup(ROBOT_MACS, "openmv")

        robot_master.functionFactory('s', stopOne, "Stop")
        index = "0"
        current_speed = 0
        current_angle = 0
        d_speed = 0.02
        d_angle = 10
        robot_master.switchRobot(3)

        while True:
            print(current_speed, current_angle)
            time.sleep(0.2)
            keys = robot_master.get_last_n_keys(1)
            axis, buttons = joystick.getJoystickInputs()
            robot_master.processManual(axis, buttons, print=False)

            # Display joystick values if enabled
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            if len(keys) == 0:
                continue

            key_pressed = keys[0]

            if len(key_pressed) == 1 and 32 <= ord(key_pressed) <= 127:
                if key_pressed == '2':
                    current_speed += d_speed

                    def temp(serial, robot):
                        serial.send_control_params(robot, (
                        current_speed, current_speed, current_angle, current_angle, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        time.sleep(0.05)
                        return 1

                    robot_master.functionFactory('g', temp, "run")
                    robot_master.runFunction('g', 1)
                    robot_master.runFunction('g', 2)
                elif key_pressed == '1':
                    current_speed -= d_speed

                    def temp(serial, robot):
                        serial.send_control_params(robot, (
                        current_speed, current_speed, current_angle, current_angle, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        time.sleep(0.05)
                        return 1

                    robot_master.functionFactory('g', temp, "run")
                    robot_master.runFunction('g', 1)
                    robot_master.runFunction('g', 2)
                if key_pressed == 'w':
                    current_angle += d_angle

                    def temp(serial, robot):
                        serial.send_control_params(robot, (
                        current_speed, current_speed, current_angle, current_angle, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        time.sleep(0.05)
                        return 1

                    robot_master.functionFactory('g', temp, "run")
                    robot_master.runFunction('g', 1)
                    robot_master.runFunction('g', 2)
                elif key_pressed == 'q':
                    current_angle -= d_angle

                    def temp(serial, robot):
                        serial.send_control_params(robot, (
                        current_speed, current_speed, current_angle, current_angle, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                        time.sleep(0.05)
                        return 1

                    robot_master.functionFactory('g', temp, "run")
                    robot_master.runFunction('g', 1)
                    robot_master.runFunction('g', 2)
                else:
                    robot_master.runFunction(key_pressed, 1)
                    robot_master.runFunction(key_pressed, 2)
            else:
                print("Invalid button.")

    except KeyboardInterrupt:
        print("Stopping!")
        return
    except Exception as e:
        print(e)
        return
    finally:
        robot_master.runFunction('s')


if __name__ == "__main__":
    main()