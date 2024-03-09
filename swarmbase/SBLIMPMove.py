

from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
from gui.simpleGUI import SimpleGUI
import time

##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##
ROBOT_MAC = "34:85:18:91:B3:44"#"34:85:18:AB:FE:68" # "DC:54:75:D7:B3:E8"
### Insert your SERIAL PORT here ###
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
PORT = "COM6"


# For debug purposes
PRINT_JOYSTICK = False


BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address



if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.5)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    time.sleep(.05)
    
    serial.send_control_params(ROBOT_MAC, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()
    mygui = SimpleGUI()
    ready = True
    old_b = 0
    dt = .1
    height = 0
    servos = 75
    tz = 0
    try:
        while True:
            # Axis input: [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()
            
            if buttons[3] == 1: # y stops the program
                break
            if buttons[1] == 1 and old_b == 0: # b pauses the control
                ready = not ready
            old_b = buttons[1]
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            # #### CONTROL INPUTS to the robot here #########
            # if abs(axis[0]) < .15:
            #     axis[0] = 0
            # height += -axis[0] * dt 
            # if height > 15:
            #     height = 15
            # elif height < -3:
            #     height = -3
                
            # if abs(axis[4]) < .15:
            #     axis[4] = 0
            # tz += -axis[4] *2.2 * dt
            # tz = -axis[4] * .1
            
            # fx = axis[2] - axis[5]
            # #print(tz, ":", height)
            # #print(height)
            
            # led = -buttons[2]
            ############# End CONTROL INPUTS ###############
            sensors = serial.getSensorData()
            # print(sensors)
            if (sensors):
                mygui.update(
                    cur_yaw=sensors[1],
                    des_yaw=tz,
                    cur_height=sensors[0],
                    des_height=height,
                    battery=sensors[2],
                    distance=0,
                    connection_status=True,
                )
            # fx = 0
            fz = height
            tx = 0
            # tz = 0
            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (fz, fz, fz, fz, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("Stopping!")
        # Send zero input
serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
