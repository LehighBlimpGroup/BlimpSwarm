from comm.Serial import DataType_Int, DataType_Float, DataType_Boolean

preferences = {
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
		{"data_type": DataType_Float, "key": "lx", "value": 0.1},
		{"data_type": DataType_Float, "key": "servoRange", "value": 360},
		{"data_type": DataType_Float, "key": "servoBeta", "value": 180},
		{"data_type": DataType_Float, "key": "servo_move_min", "value": 0},
		{"data_type": DataType_Float, "key": "botZlim", "value": -1},
		{"data_type": DataType_Float, "key": "pitchOffset", "value": 0},
		{"data_type": DataType_Float, "key": "pitchInvert", "value": -1},

		{"data_type": DataType_Int, "key": "state_flag", "value": 0x40},
		
		{"data_type": DataType_Int, "key": "num_charges", "value": 2},
		{"data_type": DataType_Int, "key": "time_in_mode", "value": 600},
		{"data_type": DataType_Int, "key": "charge_time", "value": 15},
		{"data_type": DataType_Float, "key": "default_height", "value": 7},
		{"data_type": DataType_Float, "key": "height_range", "value": 1},
		{"data_type": DataType_Float, "key": "wall_thresh", "value": 250},

		{"data_type": DataType_Float, "key": "y_thresh", "value": 0.45},
		{"data_type": DataType_Float, "key": "y_strength", "value": 1.5},
		{"data_type": DataType_Float, "key": "x_strength", "value": 1.1},
		{"data_type": DataType_Float, "key": "fx_togoal", "value": 0.2},
		{"data_type": DataType_Float, "key": "fx_charge", "value": 0.5},
		{"data_type": DataType_Float, "key": "fx_levy", "value": 0.3},
		{"data_type": DataType_Float, "key": "fz_levy", "value": 10},
		{"data_type": DataType_Float, "key": "levy_yaw", "value": 0.6},
		{"data_type": DataType_Int, "key": "n_max_x", "value": 240},
		{"data_type": DataType_Int, "key": "n_max_y", "value": 160},
		{"data_type": DataType_Float, "key": "h_ratio", "value": 0.8},
		{"data_type": DataType_Float, "key": "range_for_forward", "value": 0.12},
		
		{"data_type": DataType_Int, "key": "bnum_charges", "value": 2},
		{"data_type": DataType_Int, "key": "btime_in_mode", "value": 30},
		{"data_type": DataType_Int, "key": "bcharge_time", "value": 4},
		{"data_type": DataType_Float, "key": "bdefault_height", "value": 3},
		{"data_type": DataType_Float, "key": "bheight_range", "value": 1},
		{"data_type": DataType_Float, "key": "bwall_thresh", "value": 250},
		
		{"data_type": DataType_Float, "key": "by_thresh", "value": 0.35},
		{"data_type": DataType_Float, "key": "by_strength", "value": 1.5},
		{"data_type": DataType_Float, "key": "bx_strength", "value": 1.1},
		{"data_type": DataType_Float, "key": "bfx_togoal", "value": 0.15}, #0.11
		{"data_type": DataType_Float, "key": "bfx_charge", "value": 0.3}, #0.4
		{"data_type": DataType_Float, "key": "bfx_levy", "value": 0.1}, #0.3
		{"data_type": DataType_Float, "key": "bfz_levy", "value": 10}, #0.3
		{"data_type": DataType_Float, "key": "blevy_yaw", "value": 0.6}, #0.3
		{"data_type": DataType_Int, "key": "bn_max_x", "value": 240},
		{"data_type": DataType_Int, "key": "bn_max_y", "value": 160},
		{"data_type": DataType_Float, "key": "bh_ratio", "value": 0.1},
		{"data_type": DataType_Float, "key": "brange_for_forward", "value": 0.15}
    ], 
    "34:85:18:91:49:C0" : [
		{"data_type": DataType_Float, "key": "servoRange", "value": 260},
		{"data_type": DataType_Float, "key": "servoBeta", "value": 0},
		{"data_type": DataType_Float, "key": "bfx_levy", "value": 0.3}, #0.3
		{"data_type": DataType_Float, "key": "fx_levy", "value": 0.6},
		{"data_type": DataType_Float, "key": "fx_togoal", "value": 0.1},
		{"data_type": DataType_Float, "key": "fx_charge", "value": 0.1},

    ],
    "30:30:F9:34:66:FC" : [
		{"data_type": DataType_Boolean, "key": "zEn", "value": False},
		{"data_type": DataType_Boolean, "key": "rollEn", "value": False},
		{"data_type": DataType_Boolean, "key": "rotateEn", "value": False},
		{"data_type": DataType_Boolean, "key": "pitchEn", "value": False},
		{"data_type": DataType_Boolean, "key": "yawEn", "value": True},
		{"data_type": DataType_Float, "key": "servoRange", "value": 360},
		{"data_type": DataType_Float, "key": "servoBeta", "value": 180},
		{"data_type": DataType_Float, "key": "bfx_levy", "value": 0.1}, #0.3
		{"data_type": DataType_Float, "key": "fx_levy", "value": 0.1},
		{"data_type": DataType_Float, "key": "kpyaw", "value": 0},
		{"data_type": DataType_Float, "key": "kppyaw", "value": 0.0},
		{"data_type": DataType_Float, "key": "kdyaw", "value": 0.0},
		{"data_type": DataType_Float, "key": "kddyaw", "value": 0.0},
		{"data_type": DataType_Float, "key": "kiyaw", "value": 0},
		{"data_type": DataType_Float, "key": "kiyawrate", "value": 0},
    ],
    "DC:DA:0C:57:4B:94" : [
		{"data_type": DataType_Boolean, "key": "zEn", "value": False},
		{"data_type": DataType_Boolean, "key": "rollEn", "value": False},
		{"data_type": DataType_Boolean, "key": "rotateEn", "value": False},
		{"data_type": DataType_Boolean, "key": "pitchEn", "value": False},
		{"data_type": DataType_Boolean, "key": "yawEn", "value": True},
		{"data_type": DataType_Float, "key": "servoRange", "value": 360},
		{"data_type": DataType_Float, "key": "servoBeta", "value": 180},
		{"data_type": DataType_Float, "key": "bfx_levy", "value": 0.1}, #0.3
		{"data_type": DataType_Float, "key": "fx_levy", "value": 0.1},
		{"data_type": DataType_Float, "key": "kpyaw", "value": 0},
		{"data_type": DataType_Float, "key": "kppyaw", "value": 0.00},
		{"data_type": DataType_Float, "key": "kdyaw", "value": 0.0},
		{"data_type": DataType_Float, "key": "kddyaw", "value": 0.0},
		{"data_type": DataType_Float, "key": "kiyaw", "value": 0},
		{"data_type": DataType_Float, "key": "kiyawrate", "value": 0},
    ],
    "34:85:18:91:CE:FC" : [
		{"data_type": DataType_Float, "key": "servoRange", "value": 360},
		{"data_type": DataType_Float, "key": "servoBeta", "value": 180},
		{"data_type": DataType_Float, "key": "bfx_levy", "value": 0.1}, #0.3
		{"data_type": DataType_Float, "key": "fx_levy", "value": 0.1},
		{"data_type": DataType_Boolean, "key": "yawEn", "value": True},
    ]}

PREFERENCES = {k.lower(): v for k, v in preferences.items()}