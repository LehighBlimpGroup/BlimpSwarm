from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean


from user_parameters import ROBOT_MACS, SERIAL_PORT, PRINT_JOYSTICK

# Function to send preferences to a specific robot
def send_preferences(serial, robot_mac, preferences):
    for pref in preferences:
        serial.send_preference(robot_mac, pref["data_type"], pref["key"], pref["value"])

# Main execution block
if __name__ == "__main__":
    # Communication setup
    serial = SerialController(SERIAL_PORT, timeout=0.5)

    # List of robots and their preferences
    robots = [
        # # blue robot with skates and no net
        # {"mac": "30:30:F9:34:66:FC", "preferences": [
        #     {"data_type": DataType_Boolean, "key": "zEn", "value": True},
        #     {"data_type": DataType_Float, "key": "kpyaw", "value": 0},
        #     # Add more preferences here
        # ]},
        # {"mac": "MAC2", "preferences": [
        #     {"data_type": DataType_Boolean, "key": "zEn", "value": True},
        #     {"data_type": DataType_Float, "key": "kpyaw", "value": 1.5},
        #     # Custom preferences for MAC2
        # ]}
    ]

    # Common preferences that apply to all robots if any
    common_preferences = [
        {"data_type": DataType_Boolean, "key": "zEn", "value": True},
        {"data_type": DataType_Boolean, "key": "rollEn", "value": False},
        {"data_type": DataType_Boolean, "key": "rotateEn", "value": False},
        {"data_type": DataType_Boolean, "key": "pitchEn", "value": True},
        {"data_type": DataType_Boolean, "key": "yawEn", "value": True},

        {"data_type": DataType_Float, "key": "kpyaw", "value": 2},
        {"data_type": DataType_Float, "key": "kppyaw", "value": 0.04},
        {"data_type": DataType_Float, "key": "kdyaw", "value": 0.07},
        {"data_type": DataType_Float, "key": "kddyaw", "value": 0.04},
        {"data_type": DataType_Float, "key": "kiyaw", "value": 0},
        {"data_type": DataType_Float, "key": "kiyawrate", "value": 0},

        {"data_type": DataType_Float, "key": "yawrate_gamma", "value": 0.5},
        {"data_type": DataType_Float, "key": "rollrate_gamma", "value": 0.85},
        {"data_type": DataType_Float, "key": "pitchrate_gamma", "value": 0.7},

        {"data_type": DataType_Float, "key": "kpz", "value": 0.3},
        {"data_type": DataType_Float, "key": "kdz", "value": 0.6},
        {"data_type": DataType_Float, "key": "kiz", "value": 0},

        {"data_type": DataType_Float, "key": "kproll", "value": 0},
        {"data_type": DataType_Float, "key": "kdroll", "value": 0},
        {"data_type": DataType_Float, "key": "kppitch", "value": 0},
        {"data_type": DataType_Float, "key": "kdpitch", "value": 0},

        {"data_type": DataType_Float, "key": "z_int_low", "value": 0},
        {"data_type": DataType_Float, "key": "z_int_high", "value": 0.15},
        {"data_type": DataType_Float, "key": "yawRateIntRange", "value": 0},
        {"data_type": DataType_Float, "key": "lx", "value": 0.15},
        {"data_type": DataType_Float, "key": "servoRange", "value": 260},
        {"data_type": DataType_Float, "key": "servoBeta", "value": 90},
        {"data_type": DataType_Float, "key": "servo_move_min", "value": 0},
        {"data_type": DataType_Float, "key": "botZlim", "value": -1},
        {"data_type": DataType_Float, "key": "pitchOffset", "value": 0},
        {"data_type": DataType_Float, "key": "pitchInvert", "value": -1},

        {"data_type": DataType_Int, "key": "state_flag", "value": 0x40},
        {"data_type": DataType_Int, "key": "num_captures", "value": 4},
        {"data_type": DataType_Int, "key": "time_in_ball", "value": 60},
        {"data_type": DataType_Float, "key": "goal_height", "value": 8.0},

        {"data_type": DataType_Float, "key": "y_thresh", "value": 0.6},
        {"data_type": DataType_Float, "key": "y_strength", "value": 2.5},
        {"data_type": DataType_Float, "key": "x_strength", "value": 1},
        {"data_type": DataType_Float, "key": "fx_togoal", "value": 0.15},
        {"data_type": DataType_Float, "key": "fx_charge", "value": 0.35},
        {"data_type": DataType_Float, "key": "fx_levy", "value": 0.15},
        {"data_type": DataType_Int, "key": "n_max_x", "value": 240},
        {"data_type": DataType_Int, "key": "n_max_y", "value": 160},
        {"data_type": DataType_Float, "key": "h_ratio", "value": 0.8},
        {"data_type": DataType_Float, "key": "range_for_forward", "value": 0.16},

        {"data_type": DataType_Float, "key": "by_thresh", "value": 0.3},
        {"data_type": DataType_Float, "key": "by_strength", "value": 1.5},
        {"data_type": DataType_Float, "key": "bx_strength", "value": 1},
        {"data_type": DataType_Float, "key": "bfx_togoal", "value": 0.2},
        {"data_type": DataType_Float, "key": "bfx_charge", "value": 0.4},
        {"data_type": DataType_Float, "key": "bfx_levy", "value": 0.13},
        {"data_type": DataType_Int, "key": "bn_max_x", "value": 240},
        {"data_type": DataType_Int, "key": "bn_max_y", "value": 160},
        {"data_type": DataType_Float, "key": "bh_ratio", "value": 0.1},
        {"data_type": DataType_Float, "key": "brange_for_forward", "value": 0.35}
    ]

    # Sending common preferences to all robots
    for robot in ROBOT_MACS:
        serial.manage_peer("A", robot)
        serial.manage_peer("G", robot)
        # Send common preferences
        send_preferences(serial, robot, common_preferences)
        # Send specific preferences
        for prefs in robots:
            if robot == prefs["mac"]:
                send_preferences(serial, robot, prefs["preferences"])
                break
        serial.send_control_params(robot, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))

