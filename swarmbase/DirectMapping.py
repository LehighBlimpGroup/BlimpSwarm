

from comm.Serial import SerialController, DataType_Int, DataType_Float
from joystick.JoystickManager import JoystickManager
import time

ROBOT_MAC = "48:27:E2:E6:E0:1C"
SERIAL_PORT = "COM9"
PRINT_JOYSTICK = False

if __name__ == "__main__":
    # Communication
    serial = SerialController(SERIAL_PORT, timeout=.1)
    serial.manage_peer("A", ROBOT_MAC) 
    serial.manage_peer("G", ROBOT_MAC)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", .6)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", .8)
    serial.send_control_params(ROBOT_MAC, (0,0,90,90, 0, 0, 0, 0, 0, 0, 0, 1, 0)) #refresh parameters
    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()


    try:
        while True:
            # Axis input: [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()


            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)
            

            #### CONTROL INPUTS to the robot here #########
            m1 = axis[2]  # Motor 1: a value between 0-1
            m2 = axis[5]  # Motor 2: a value between 0-1
            s1 = axis[3]  # Servo 1: a value between 0-180
            s2 = axis[0]  # Servo 2: a value between 0-180
            ready = 1
            
            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (m1, m2, s1, s2, ready, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(.03)
    finally:
        print("Stopping!")
        serial.send_control_params(ROBOT_MAC, (-1, -1, 90, 90, 1, 0, 0, 0, 0, 0, 0, 0, 0)) # zero out the motors
        serial.close()
