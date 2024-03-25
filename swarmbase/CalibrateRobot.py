'''
Calibrate your robot!
In order to calibrate, 
    1) make sure your drone is on and your mac/port are correct
    2) run this python file
    3) unplug your robot, then replug in with a battery
    4) Listen to the motors making "music"
    5) when the music stops, your robot has finished calibrating
After calibration, your robot will function as normal, 
and you will probobly not need to run this program again.
'''



from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
import time
from user_parameters import ROBOT_MAC, SERIAL_PORT




BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address



if __name__ == "__main__":
    # Communication
    serial = SerialController(SERIAL_PORT, timeout=.1)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "calibrate", True)
    serial.close()