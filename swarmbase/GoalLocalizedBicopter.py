from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
import pygame, time
import numpy as np
from gui.robotSimulator import NonHolonomicRobotSimulator
from user_parameters import ROBOT_MAC, SERIAL_PORT, PRINT_JOYSTICK

# Initialize the simulator
scale = 50
verbose = False
simulator = NonHolonomicRobotSimulator(verbose, scale)


def main():
    serial = SerialController(SERIAL_PORT, timeout=0.02)
    joystick = JoystickManager()
    
    # Setup communication with robot
    serial.manage_peer("A", ROBOT_MAC)
    time.sleep(0.05)

    # Initialize control variables
    sensors = serial.getSensorData()
    height, tz = (sensors[0], sensors[1]) if sensors else (0, 0)
    ready = 5
    old_buttons = [0] * 4  # A, B, X, Y
    fx_ave = 0
    dt = 0.1
    now = time.time()

    # Simulate robot state
    x = np.array(([15], [6.24], [0], [0]))
    ell = (10 / scale, 10 / scale, 0)
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            axis, buttons = joystick.getJoystickInputs()
            
            # Button press logic
            if buttons[3]:  # Y stops the program
                break
            if buttons[1] and not old_buttons[1]:  # B toggles pause
                ready = 0 if ready else 5
            # if buttons[2] and not old_buttons[2]:  # X toggles special mode
            #     ready = 3 if ready != 3 else 4
            # if buttons[0] and not old_buttons[0]:  # A sets specific ready state
            #     ready = 2

            # Update old button states
            old_buttons = buttons[:]
            
            # Display joystick values if enabled
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            # Control inputs to the robot
            height += -axis[0] * dt if abs(axis[0]) >= 0.15 else 0
            height = max(min(height, 15), -3)
            tz += -axis[4] * 1.2 * dt if abs(axis[4]) >= 0.15 else 0
            fx = (-axis[2] + axis[5]) * 0.2
            fx_ave = fx_ave * 0.8 + fx * 0.2

            # Retrieve sensor data
            sensors = serial.getSensorData()
            if sensors:
                if sensors[2] < 300:
                    robot_height = sensors[0]
                    robot_yaw = sensors[1] * 180 / 3.14159
                    kalman_x = sensors[2]
                    kalman_y = sensors[3]
                    ell_angle = sensors[4]
                    ell_major = 100
                    ell_minor = 100
                    battery_v = sensors[5]
                    print(sensors)

                    # Simulate robot state
                    x = np.array(([kalman_x], [kalman_y], [0], [0]))
                    ell = (ell_major , ell_minor , ell_angle)
                    simulator.robot_x = kalman_x* scale
                    simulator.robot_y = kalman_y * scale
                    simulator.robot_angle = robot_yaw
            simulator.simulate(x=x * scale, ell=ell)

            # Update GUI and send control parameters
            serial.send_control_params(ROBOT_MAC, (ready, fx_ave, height, 0, tz, -buttons[2], 1, 0, 0, 0, 0, 0, 0))

            dt = time.time() - now
            now = time.time()

            simulator.clock.tick(11)

    except KeyboardInterrupt:
        print("Stopping!")
        serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        pygame.quit()

if __name__ == "__main__":
    main()
