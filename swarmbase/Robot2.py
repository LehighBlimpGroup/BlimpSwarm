from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
import time

ROBOT_MAC = "DC:DA:0C:57:AE:2C"
SERIAL_PORT = "COM16"
PRINT_JOYSTICK = True

if __name__ == "__main__":
    # Communication
    serial = SerialController(SERIAL_PORT, timeout=.5)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    time.sleep(.05)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "zEn", False)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "rollEn", False)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "rotateEn", False)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "pitchEn", False)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "yawEn", True)
    serial.send_preference(ROBOT_MAC, DataType_Boolean, "calibrate", False)

    # // PID terms
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpyaw", 2)  # 2
    serial.send_preference(ROBOT_MAC, DataType_Float, "kppyaw", 0.05)  # .1
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdyaw", .05)  # .1
    serial.send_preference(ROBOT_MAC, DataType_Float, "kddyaw", 0.05)  # .1
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiyaw", 0)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiyawrate", 0)

    serial.send_preference(ROBOT_MAC, DataType_Float, "yawrate_gamma", 0.5)
    serial.send_preference(ROBOT_MAC, DataType_Float, "rollrate_gamma", 0.85)
    serial.send_preference(ROBOT_MAC, DataType_Float, "pitchrate_gamma", 0.7)

    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", 0.3)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", 0.8)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiz", 0.1)

    serial.send_preference(ROBOT_MAC, DataType_Float, "kproll", 0)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdroll", 0.2)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kppitch", 0)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdpitch", .4)

    # // Range terms for the integrals
    serial.send_preference(ROBOT_MAC, DataType_Float, "z_int_low", 0.0)
    serial.send_preference(ROBOT_MAC, DataType_Float, "z_int_high", 0.15)
    serial.send_preference(ROBOT_MAC, DataType_Float, "yawRateIntRange", 0.1)

    # // radius of the blimp
    serial.send_preference(ROBOT_MAC, DataType_Float, "lx", 0.15)

    serial.send_preference(ROBOT_MAC, DataType_Float, "servoRange", 260)  # degrees
    serial.send_preference(ROBOT_MAC, DataType_Float, "servoBeta", 90)  # degrees
    serial.send_preference(ROBOT_MAC, DataType_Float, "servo_move_min", 0)  # degrees

    serial.send_preference(ROBOT_MAC, DataType_Float, "botZlim", -1)
    serial.send_preference(ROBOT_MAC, DataType_Float, "pitchOffset", 0)  # degrees
    serial.send_preference(ROBOT_MAC, DataType_Float, "pitchInvert", -1)  # degrees

    # nicla parameters
    serial.send_preference(ROBOT_MAC, DataType_Int, "state_flag", 0x80)
    serial.send_preference(ROBOT_MAC, DataType_Int, "num_captures", 4)  # number of ball captures before going to goal
    serial.send_preference(ROBOT_MAC, DataType_Int, "time_in_ball", 30)  # in seconds
    serial.send_preference(ROBOT_MAC, DataType_Float, "goal_height", 5)  # in meters

    # goals
    serial.send_preference(ROBOT_MAC, DataType_Float, "y_thresh", 0.55)
    serial.send_preference(ROBOT_MAC, DataType_Float, "y_strength", 3)
    serial.send_preference(ROBOT_MAC, DataType_Float, "x_strength", 1)

    serial.send_preference(ROBOT_MAC, DataType_Float, "fx_togoal", 0.2)
    serial.send_preference(ROBOT_MAC, DataType_Float, "fx_charge", 0.35)
    serial.send_preference(ROBOT_MAC, DataType_Float, "fx_levy", 0.2)

    serial.send_preference(ROBOT_MAC, DataType_Int, "n_max_x", 240)
    serial.send_preference(ROBOT_MAC, DataType_Int, "n_max_y", 160)
    serial.send_preference(ROBOT_MAC, DataType_Float, "h_ratio", 0.8)
    serial.send_preference(ROBOT_MAC, DataType_Float, "range_for_forward", 0.16)

    # balloons
    serial.send_preference(ROBOT_MAC, DataType_Float, "by_thresh", 0.42)
    serial.send_preference(ROBOT_MAC, DataType_Float, "by_strength", 5)
    serial.send_preference(ROBOT_MAC, DataType_Float, "bx_strength", 2)

    serial.send_preference(ROBOT_MAC, DataType_Float, "bfx_togoal", .15)
    serial.send_preference(ROBOT_MAC, DataType_Float, "bfx_charge", 0.05)
    serial.send_preference(ROBOT_MAC, DataType_Float, "bfx_levy", 0.1)

    serial.send_preference(ROBOT_MAC, DataType_Int, "bn_max_x", 240)
    serial.send_preference(ROBOT_MAC, DataType_Int, "bn_max_y", 160)
    serial.send_preference(ROBOT_MAC, DataType_Float, "bh_ratio", 0.8)

    serial.send_preference(ROBOT_MAC, DataType_Float, "brange_for_forward", 0.16)

    serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0))
    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()

    sensors = serial.getSensorData()
    height = 0
    tz = 0
    if (sensors):
        tz = sensors[1]
        height = sensors[0]
    ready = 0
    old_a = 0
    old_b = 0
    old_x = 0
    fx_ave = 0
    dt = .1

    servos = 75

    try:
        while True:
            # Axis input: [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()

            if buttons[3] == 1:  # y stops the program
                break
            if buttons[1] == 1 and old_b == 0:  # b pauses the control
                if (sensors):
                    tz = sensors[1]
                    height = sensors[0]
                if ready != 0:
                    ready = 0
                else:
                    ready = 1
            if buttons[2] == 1 and old_x == 0:
                if ready != 3:
                    ready = 3
                else:
                    ready = 4
            if buttons[0] == 1 and old_a == 0:
                ready = 2
            old_x = buttons[2]
            old_b = buttons[1]
            old_a = buttons[0]
            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            #### CONTROL INPUTS to the robot here #########
            if abs(axis[0]) < .15:
                axis[0] = 0
            height += -axis[0] * dt
            if height > 15:
                height = 15
            elif height < -3:
                height = -3

            if abs(axis[1]) < .15:
                axis[1] = 0
            tx = axis[1] * .2

            if abs(axis[4]) < .15:
                axis[4] = 0
            tz += -axis[4] * 1.2 * dt
            # tz = -axis[4] * .1

            fx = - axis[2] + axis[5]
            if (fx < 0):
                fx = fx * .5

            # print(tz, ":", height)
            # print(height)

            led = -buttons[2]
            ############# End CONTROL INPUTS ###############
            # fx = 0
            fz = height
            fx_ave = fx  # fx_ave * .8 + fx * .2
            # tx = 0
            # tz = 0
            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (ready, fx_ave, fz, tx, tz, led, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(dt)

    except KeyboardInterrupt:
        print("Stopping!")
    finally:
        serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
