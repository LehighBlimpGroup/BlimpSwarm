
import pygame, time
import numpy as np
import ComputerBase as CB


DroneMacAddress = "48:27:E2:E6:E1:00" #insert your blimp's mac address here (you can get it by running your arduino and looking at the serial monitor for your flying drone)
BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address
port = "COM6" # may look like "COM5" or "/dev/tty.usbmodem14301", look in arduino for the port that your specific transeiver is connected to
#note: make sure that your serial monitor is OFF on your base station in arduino or else you will get "access is denied" error

class RawBicopterInterface:
    def __init__(self, blimpMac, portadd):
        
        self.controller = CB.SerialController(portadd, timeout=.1)  # 5-second timeout
        self.controller.manage_peer("A", blimpMac)
        self.mac = blimpMac
        # Initialize Pygame for joystick handling
        pygame.init()
        pygame.joystick.init()  # Initialize the Joystick subsystem

        # Assuming there's at least one joystick connected
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)  # Initialize the first joystick
            self.joystick.init()
        else:
            print("No joystick detected!")
            self.joystick = None

    def updateJoy(self):
        pygame.event.pump()  # Process event queue for the joystick
        time.sleep(.1)

    def getJoysticks(self):
        if self.joystick:
            # Example mapping (adjust based on your joystick)
            right_vert = self.joystick.get_axis(3)  # Right joystick vertical
            right_horz = self.joystick.get_axis(2)  # Right joystick horizontal
            left_vert = self.joystick.get_axis(1)  # Left joystick vertical
            left_horz = self.joystick.get_axis(0)  # Left joystick horizontal
            right_trigger = self.joystick.get_axis(5)  # Right trigger
            left_trigger = self.joystick.get_axis(4)  # Left trigger
            return [right_vert, right_horz, left_vert, left_horz, right_trigger, left_trigger]
        else:
            return [0, 0, 0, 0, 0, 0]

    def getButtons(self):
        if self.joystick:
            # Example button mapping (adjust based on your joystick)
            a = self.joystick.get_button(0)
            b = self.joystick.get_button(1)
            x = self.joystick.get_button(2)
            y = self.joystick.get_button(3)
            return [a, b, x, y]
        else:
            return [0, 0, 0, 0]

    def send(self, m1,m2,s1,s2):
        params = (m1, m2, s1, s2,0, 0,0,0,0,0,0,0,0)
        self.controller.send_control_params(self.mac, params)
        


if __name__ == "__main__":
    RBI = RawBicopterInterface(DroneMacAddress, port)
    try:
        while True:
            RBI.updateJoy()
            joysticks = RBI.getJoysticks() # [right joystick vertical, right joystick horizontal, left joystick vertical, left joystick horizontal, right trigger, left trigger]
            buttons = RBI.getButtons() #[A, B, X, Y]
            ''' Insert your code here! '''
            
            RBI.send(0,0,0,0)# motor 1 (value 0-1), motor 2 (value 0-1), servo 1 (degrees), servo 2 (degrees)
            
    except KeyboardInterrupt:
        print("Stopping!")
    RBI.send(0,0,180,180)