from comm.Serial import DataType_Boolean
from input.JoystickManager import JoystickManager
from user_parameters import ROBOT_MACS
from robot.RobotMaster import RobotMaster
import Preferences
import time
import importlib

PRINT_JOYSTICK = False
        
def startAutonomousBall(serial, robot, args):
    serial.send_control_params(robot, (3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.2)
    serial.send_control_params(robot, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.2)
    return 2

def stopOne(serial, robot, args):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_control_params(robot, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 0

def startOne(serial, robot, args):
    serial.send_control_params(robot, (4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_control_params(robot, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.2)
    return 2

def sendCalibrate(serial, robot, args):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_preference(robot, DataType_Boolean, "calibrate", True)
    time.sleep(.05)
    return -1

def setHeight(serial, robot, args):
    print(args[0])
    serial.send_control_params(robot, (args[0], args[0], args[1], args[1], 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 1


def sendPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["ff:ff:ff:ff:ff:ff"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1

def sendOrangePreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["orange"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1

def sendDefenderPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["defender"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1

def sendAttackerPreferences(serial, robot, args):
    importlib.reload(Preferences)

    for pref in Preferences.PREFERENCES["attacker"]:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    if robot not in Preferences.PREFERENCES:
        serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
        return -1
    prefer = Preferences.PREFERENCES[robot]
    for pref in prefer:
        serial.send_preference(robot, pref["data_type"], pref["key"], pref["value"])
    serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    return -1

def main():
    try:
        robot_master = RobotMaster(0.3)
        joystick = JoystickManager()
        robot_master.setup(ROBOT_MACS, "nicla")
        followers = []

        robot_master.functionFactory('s', stopOne, "Stop")
        robot_master.functionFactory('a', startAutonomousBall, "Auto")
        robot_master.functionFactory('c', sendCalibrate, "Calibrate")
        robot_master.functionFactory('p', sendPreferences, "Send Preferences")
        robot_master.functionFactory('m', sendAttackerPreferences, "Send Attacker Preferences")
        robot_master.functionFactory(',', sendDefenderPreferences, "Send Defender Preferences")
        robot_master.functionFactory('h', setHeight, "Set Height")
        robot_master.functionFactory('g', startOne, "Start goal")
        index = "0"
        power = 0
        angle = 0
        dt_p = 0.1
        dt_a = 5

        while True:
            time.sleep(0.2)
            keys = robot_master.get_last_n_keys(1)
            axis, buttons = joystick.getJoystickInputs()
            robot_master.processManual(axis, buttons, print=True)

            power = min(1, max(0, power))
            angle = min(180, max(-180, angle))
            

            # Display joystick values if enabled
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            if len(keys) == 0:
                continue
            
            key_pressed = keys[0]
            
            if key_pressed == 'enter':
                i = int(index)
                robot_master.switchRobot(i)
                index = "0"
            elif len(key_pressed) == 1 and 32 <= ord(key_pressed) <= 127:
                if '0' <= key_pressed <= '9':
                    index += key_pressed
                elif key_pressed == 'd':
                    index = "0"
                    print("Cleared index")
                elif key_pressed == 'q':
                    break
                elif key_pressed == '[':
                    angle = 0
                    power = 0
                    for i in followers:
                        robot_master.runFunction('h', i, power, angle)
                    print(power, angle, "close")
                elif key_pressed == ']':
                    angle = 47
                    power = .3
                    for i in followers:
                        robot_master.runFunction('h', i, power, angle)
                    print(power, angle, "open")
                elif key_pressed == ';':
                    power -= 0.05
                    for i in followers:
                        robot_master.runFunction('h', i, power, angle)
                    print(power, angle)
                elif key_pressed == "'":
                    power += 0.05
                    for i in followers:
                        robot_master.runFunction('h', i, power, angle)
                    print(power, angle)
                elif key_pressed == 's':
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
        return
    except Exception as e:
        print(e)
        return
    

if __name__ == "__main__":
    main()