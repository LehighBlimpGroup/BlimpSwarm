import serial
import struct
import time

class SerialController:
    def __init__(self, port, baudrate=115200, timeout=2):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def manage_peer(self, operation, mac_address=None):
        """Manage ESP-NOW peers by adding or removing them."""
        if operation in ['A', 'R'] and mac_address:
            mac_bytes = bytes(int(x, 16) for x in mac_address.split(':'))
            self.serial.write(operation.encode() + mac_bytes)
            print(f"Peer {'added' if operation == 'A' else 'removed'}: {mac_address}")
        elif operation == 'C':  # Indicates a control command
            self.serial.write(operation.encode())
        else:
            print("Invalid operation or MAC address")
        self.wait_for_acknowledgement()

    def send_control_params(self, params):
        """Send control parameters to the Arduino."""
        if len(params) != 13:
            raise ValueError("Expected 13 control parameters")
        data = struct.pack('<13f', *params)  # Packing parameters into binary format
        self.serial.write(b'C' + data)  # Prefix with 'C' for control command
        print("Control parameters sent.")
        self.wait_for_acknowledgement()

    def wait_for_acknowledgement(self):
        """Wait for an acknowledgment or error message from Arduino."""
        try:
            incoming = self.serial.readline().decode().strip()
            if incoming == "":
                print("Timeout or no data received.")
                return False
            print("Received from Arduino:", incoming)
            return True
        except serial.SerialException as e:
            print("Serial communication error:", e)
            return False

    def close(self):
        self.serial.close()

# Example usage
if __name__ == "__main__":
    port = 'COM5'  # Adjust as necessary for your setup
    controller = SerialController(port, timeout=1)  # 5-second timeout

    try:
        # Example: Add a peer
        controller.manage_peer('A', "48:27:E2:E6:E1:00")
        
        # Example: Send control parameters
        # example_params = (1.0, 0.5, -1.2, 2.5, 0.0, 1.1, -0.9, 2.2, 3.3, 4.4, 5.5, -2.2, 0.1)
        # controller.send_control_params(example_params)
        
    finally:
        controller.close()