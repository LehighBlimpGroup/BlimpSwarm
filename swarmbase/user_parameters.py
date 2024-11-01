##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##


ROBOT_MAC ="34:85:18:91:b7:4c"#"DC:DA:0C:57:4B:94" # "30:30:F9:34:66:FC" #, "34:85:18:ab:fe:68"
ROBOT_MACS =["48:27:e2:e6:e0:1c"]### Insert your SERIAL PORT here ###
#"34:85:18:91:b7:4c", "34:85:18:ab:fe:68", "34:85:18:91:ce:fc",
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
SERIAL_PORT = "COM6"

PUMP_MAC = "48:27:E2:E6:E6:44"





# For debug purposes
PRINT_JOYSTICK = False


"""
FullBicopterNicla notes:
	Controls:
		Left joystick applies height control (line 128)
		Right joystick applies yaw control (line 144)
		Right trigger applies forward in x (line 150)
		Left trigger applies backward in x (line 150)

		X button toggles nicla control (line 113)
		B button toggles the system on or off (starts in 'off' state) (line 105)
		Y button stops the python program (line 103)

		You may want to change these if you think 
		the controls should be faster or slower

	Parameters/Preferences:
		PID parameters!
		zEn, yawEn : enable of the yaw or height feedback (Should stay true unless you want to disable for unit testing)
		rollEn, pitchEn : enable of the pitch and roll feedback (not recommended to change, (pitch is easier to test than roll))
		kppYaw, kddYaw: the classic PD control in yaw for tuning (you should tune these to your robot)
		kpYaw, kdYaw: A cascading controller in yaw (you can switch to this one if you want)
			The main difference between the two is that the cascading controller can have smoother velocities when closer to the desired yaw position
		yawrate_gamma: creates a weighted average on the yawrate to smooth behavior (not recommended to change)
		rollrate_gamma, pitchrate_gamma: creates a weighted average on rollrate and pitchrate (not recommended to change unless you mess with rollEn and pitchEn)
		kpz, kdz, kiz: PID controls for height (you should tune these to your robot)
		z_int_low, z_int_high: acts as a limiter for the z integral term in height (change low to be higher if you think your robot is heavy)
		kproll, kdroll: kproll keeps your robot level and kdroll reduces sway in roll (not recommended to change unless you mess with rollEn)
		kppitch, kdpitch: kppitch keeps your robot level in pitch and kdpitch reduces sway in pitch (not recommended to change unless you mess with pitchEn)
		
		hardware parameters!
		lx: Radius of your motors in meters (tune to your robot)
		servoRange: Depends on your servos (likely 180 degrees)
		servoBeta: changes the 'centerpoint' of your servo- where it points up (0 means your range is from fowards to backwards, 90 means it goes from up to down)
		servoMoveMin: changes the minimum change for your servos to actually move
		botZlim: if your servos cant point down it should be 0, if they can point down, it can be -1.
		pitchOffset: if your BNO is connected at an angle relative to your servos (tuned to the robot likely 0)
		pitchInvert: if your BNO needs to be inverted  (-1 when the BNO is on top, 1 when the BNO is on the bottom)
			The pitch angle is directly added to the servo angle when pitchEn is enabled in order to reduce effects of sway
		
		Nicla parameters!
		y_thresh: the center point for tuining your height in the frame ( should likely be near .5)
		y_strength: both enables the height controller from nicla, and denotes its strength (0 disables)
		x_strength: applies the 'conversion' from [-.5,.5] in the camera frame to radians relative to yaw (should likely be 1)
		fx_togoal, fx_charge, fxlevy: used in my statemachine (only used for state machine)
		n_max_x, n_max_y: the length and width in pixels of the nicla frame (HQVGA is 240,160)
		h_ratio: used in statemachine to determine when the robot is too close to goal (should be >.7)
"""