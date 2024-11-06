from comm.Serial import DataType_Int, DataType_Float, DataType_Boolean

PREFERENCES = {
        "FF:FF:FF:FF:FF:FF" : [
            {"data_type": DataType_Boolean, "key": "zEn", "value": True},
            {"data_type": DataType_Boolean, "key": "rollEn", "value": False},
            {"data_type": DataType_Boolean, "key": "rotateEn", "value": False},
            {"data_type": DataType_Boolean, "key": "pitchEn", "value": False},
            {"data_type": DataType_Boolean, "key": "yawEn", "value": True},

            {"data_type": DataType_Float, "key": "kpyaw", "value": 2},
            {"data_type": DataType_Float, "key": "kppyaw", "value": 0.03},
            {"data_type": DataType_Float, "key": "kdyaw", "value": 0.035},
            {"data_type": DataType_Float, "key": "kddyaw", "value": 0.03},
            {"data_type": DataType_Float, "key": "kiyaw", "value": 0},
            {"data_type": DataType_Float, "key": "kiyawrate", "value": 0},

            {"data_type": DataType_Float, "key": "yawrate_gamma", "value": 0.5},
            {"data_type": DataType_Float, "key": "rollrate_gamma", "value": 0.85},
            {"data_type": DataType_Float, "key": "pitchrate_gamma", "value": 0.7},

            {"data_type": DataType_Float, "key": "kpz", "value": 0.8},
            {"data_type": DataType_Float, "key": "kdz", "value": 1.2},
            {"data_type": DataType_Float, "key": "kiz", "value": 0.0},

            {"data_type": DataType_Float, "key": "kproll", "value": 0},
            {"data_type": DataType_Float, "key": "kdroll", "value": 0},
            {"data_type": DataType_Float, "key": "kppitch", "value": 0},
            {"data_type": DataType_Float, "key": "kdpitch", "value": -0.3},

            {"data_type": DataType_Float, "key": "z_int_low", "value": 0},
            {"data_type": DataType_Float, "key": "z_int_high", "value": 0.15},
            {"data_type": DataType_Float, "key": "yawRateIntRange", "value": 0},
            {"data_type": DataType_Float, "key": "lx", "value": 0.15},
            {"data_type": DataType_Float, "key": "servoRange", "value": 260},
            {"data_type": DataType_Float, "key": "servoBeta", "value": 0},
            {"data_type": DataType_Float, "key": "servo_move_min", "value": 0},
            {"data_type": DataType_Float, "key": "botZlim", "value": -1},
            {"data_type": DataType_Float, "key": "pitchOffset", "value": 0},
            {"data_type": DataType_Float, "key": "pitchInvert", "value": -1},

            {"data_type": DataType_Int, "key": "state_flag", "value": 0x40},
            {"data_type": DataType_Int, "key": "num_captures", "value": 2},
            {"data_type": DataType_Int, "key": "time_in_ball", "value": 500},
            {"data_type": DataType_Float, "key": "goal_height", "value": 8},

            {"data_type": DataType_Float, "key": "y_thresh", "value": 0.57},
            {"data_type": DataType_Float, "key": "y_strength", "value": 3.5},
            {"data_type": DataType_Float, "key": "x_strength", "value": 2},
            {"data_type": DataType_Float, "key": "fx_togoal", "value": 0.3},
            {"data_type": DataType_Float, "key": "fx_charge", "value": 0.5},
            {"data_type": DataType_Float, "key": "fx_levy", "value": 0.1},
            {"data_type": DataType_Int, "key": "n_max_x", "value": 240},
            {"data_type": DataType_Int, "key": "n_max_y", "value": 160},
            {"data_type": DataType_Float, "key": "h_ratio", "value": 0.8},
            {"data_type": DataType_Float, "key": "range_for_forward", "value": 0.12},

            {"data_type": DataType_Float, "key": "by_thresh", "value": 0.15},
            {"data_type": DataType_Float, "key": "by_strength", "value": 1.5},
            {"data_type": DataType_Float, "key": "bx_strength", "value": 1.1},
            {"data_type": DataType_Float, "key": "bfx_togoal", "value": 0.15}, #0.11
            {"data_type": DataType_Float, "key": "bfx_charge", "value": 0.3}, #0.4
            {"data_type": DataType_Float, "key": "bfx_levy", "value": 0.2}, #0.3
            {"data_type": DataType_Int, "key": "bn_max_x", "value": 320},
            {"data_type": DataType_Int, "key": "bn_max_y", "value": 240},
            {"data_type": DataType_Float, "key": "bh_ratio", "value": 0.36},
            {"data_type": DataType_Float, "key": "brange_for_forward", "value": 0.15}
    ], 
    "48:27:e2:e6:df:a0" : [
            {"data_type": DataType_Float, "key": "servoRange", "value": 360},
            {"data_type": DataType_Float, "key": "servoBeta", "value": 180},
    ]}