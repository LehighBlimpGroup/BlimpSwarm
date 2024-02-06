

import RawBicopterInterface

DroneMacAddress = "48:27:E2:E6:E1:00" #insert your blimp's mac address here (you can get it by running your arduino and looking at the serial monitor for your flying drone)
BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address
port = "COM6" # may look like "COM5" or "/dev/tty.usbmodem14301", look in arduino for the port that your specific transeiver is connected to
#note: make sure that your serial monitor is OFF on your base station in arduino or else you will get "access is denied" error

if __name__ == "__main__":
    RBI = RawBicopterInterface.RawBicopterInterface(DroneMacAddress, port)
    try:
        while True:
            RBI.updateJoy()
            joysticks = RBI.getJoysticks() # [right joystick vertical, right joystick horizontal, left joystick vertical, left joystick horizontal, right trigger, left trigger]
            buttons = RBI.getButtons() #[A, B, X, Y]
            ''' Insert your code here! '''
            #print(joysticks)
            #print(buttons)
            
            RBI.send(0,0,0,0)# motor 1 (value 0-1), motor 2 (value 0-1), servo 1 (degrees), servo 2 (degrees)
            
    except KeyboardInterrupt:
        print("Stopping!")
    RBI.send(0,0,180,180)