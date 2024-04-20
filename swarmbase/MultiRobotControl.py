import pygame
from comm.Serial import SerialController
from joystick.JoystickManager import JoystickManager
from gui.simpleGUI import SimpleGUI
from gui.niclaGUI import NiclaBox
import time
from user_parameters import ROBOT_MACS,  SERIAL_PORT, PRINT_JOYSTICK
ROBOT_MAC = None
def main():
    serial = SerialController(SERIAL_PORT, timeout=0.5)
    joystick = JoystickManager()
    mygui = SimpleGUI()
    niclaGUI = NiclaBox(max_x=240, max_y=160, x=120, y=80, width=120, height=80)
    pygame.init()
    # Get all Robot Mac addresses
    robots = ROBOT_MACS
    current_robot_index = 0
    ROBOT_MAC = robots[current_robot_index]

    # Setup communication with robot
    for robot_mac in robots:
        serial.manage_peer("A", robot_mac)
        #serial.send_control_params(ROBOT_MAC, (2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) send nicla autonomous to all robots or optionally whatever constants
        time.sleep(0.05)
    
    sensors = serial.getSensorData()
    # Initialize control variables
    height, tz =  (0, 0)
    ready = 0
    old_buttons = [0] * 4  # A, B, X, Y
    fx_ave = 0
    dt = 0.1

    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif pygame.K_0 <= event.key <= pygame.K_9:
                        index = event.key - pygame.K_0 - 1
                        if 0 <= index < len(robots):
                            current_robot_index = index
                            print("index ", current_robot_index)
                            if ready == 5:
                                serial.send_control_params(ROBOT_MAC, (6, fx_ave, height, 0, tz, -buttons[2], 0, 0, 0, 0, 0, 0, 0))
                            else:
                                serial.send_control_params(ROBOT_MAC, (ready, fx_ave, height, 0, tz, -buttons[2], 0, 0, 0, 0, 0, 0, 0))

                            ROBOT_MAC = robots[current_robot_index]
                            sensors = serial.getSensorData()
                            ready = 5
                            serial.send_control_params(ROBOT_MAC, (5, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0))
                            
                            time.sleep(.3)
                            sensor_update = sensors == serial.getSensorData() 
                            counter = 0
                            while (sensor_update and counter < 5):
                                serial.send_control_params(ROBOT_MAC, (5, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0))
                                counter += 1
                                
                                time.sleep(.3)
                                sensor_update = sensors == serial.getSensorData() 
                            if not sensor_update:
                                sensors = serial.getSensorData()
                                ready = 1
                                height, tz = (sensors[0], sensors[1])
                            else:
                                ready = 5 
                            
                            print(f"Switched to robot {ROBOT_MAC}")
            axis, buttons = joystick.getJoystickInputs()
            
            # Button press logic
            if buttons[3]:  # Y stops the program
                break
            if buttons[1] and not old_buttons[1]:  # B toggles pause
                ready = 0 if ready else 1
            if buttons[2] and not old_buttons[2]:  # X toggles special mode
                ready = 3 if ready != 3 else 4
            if buttons[0] and not old_buttons[0]:  # A sets specific ready state
                ready = 2

            # Update old button states
            old_buttons = buttons[:]
            
            # Display joystick values if enabled
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            if sensors != serial.getSensorData() and ready == 5:
                sensors = serial.getSensorData()
                ready = 1
                height, tz = (sensors[0], sensors[1])
            if ready != 5:
                # Control inputs to the robot
                height += -axis[0] * dt if abs(axis[0]) >= 0.15 else 0
                height = max(min(height, 15), -3)
                tz += -axis[4] * 1.2 * dt if abs(axis[4]) >= 0.15 else 0
            else:
                # Control inputs to the robot
                height = -axis[0] * .5 * dt if abs(axis[0]) >= 0.15 else 0
                tz = -axis[4] * .5 * dt if abs(axis[4]) >= 0.15 else 0
            fx = (-axis[2] + axis[5]) * 0.2
            fx_ave = fx_ave * 0.8 + fx * 0.2

            sensors = serial.getSensorData()
            if sensors:
                if sensors[2] < 300:
                    niclaGUI.update(x=sensors[2], y=sensors[3], width=sensors[4], height=sensors[4])
                mygui.update(cur_yaw=sensors[1], des_yaw=tz, cur_height=sensors[0], des_height=height, battery=sensors[5], distance=0, connection_status=True)
            # send control parameters
            serial.send_control_params(ROBOT_MAC, (ready, fx_ave, height, 0, tz, -buttons[2], 1, 0, 0, 0, 0, 0, 0))
            
            time.sleep(dt)
    except KeyboardInterrupt:
        print("Stopping!")
    
    for robot_mac in robots:
        serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

if __name__ == "__main__":
    main()
