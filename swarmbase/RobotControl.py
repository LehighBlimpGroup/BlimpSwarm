from comm.Serial import DataType_Boolean
from input.JoystickManager import JoystickManager
from user_parameters import ROBOT_MACS
from robot.RobotMaster import RobotMaster
from input.KeyLogger import KeyLogger
from Preferences import PREFERENCES
import time

PRINT_JOYSTICK = False

# Function to send preferences to a specific robot
def send_preferences(serial, robot):
    serial.send_preference(robot, PREFERENCES["data_type"], PREFERENCES["key"], PREFERENCES["value"])
        
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

def startOne(serial, robot):
    serial.send_control_params(robot, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return 2

def sendCalibrate(serial, robot):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    serial.send_preference(robot, DataType_Boolean, "calibrate", True)
    time.sleep(.05)
    return -1

def stopCommunication(serial, robot):
    serial.send_control_params(robot, (5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(.05)
    return -1

def main():
    try:
        robot_master = RobotMaster(0.3)
        joystick = JoystickManager()
        robot_master.setup(ROBOT_MACS, "openmv")

        robot_master.functionFactory('s', stopOne, "Stop")
        robot_master.functionFactory('g', startOne, "Start")
        robot_master.functionFactory('a', startAutonomous, "Auto")
        robot_master.functionFactory('c', sendCalibrate, "Calibrate")
        robot_master.functionFactory('l', stopCommunication, "Stop Communication")
        index = "0"

        while True:
            time.sleep(0.2)
            keys =  robot_master.get_last_n_keys(1)
            axis, buttons = joystick.getJoystickInputs()
            robot_master.processManual(axis, buttons, print=True)

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
    finally:
        robot_master.runFunction('s')
    

if __name__ == "__main__":
    main()