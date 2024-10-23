# Serial.py
import serial
import struct
import time

class SerialController:
    def __init__(self, port, baudrate=115200, timeout=2):
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            self.serial.reset_input_buffer()
            self.values = None
        except serial.serialutil.SerialException as se:
            self.serial = None
            self.values = None
            print(se)
        
    def manage_peer(self, operation, mac_address=None):
        """Manage ESP-NOW peers by adding or removing them."""
        time.sleep(0.02)
        self.serial.reset_input_buffer()

        mac_bytes = bytes(int(x, 16) for x in mac_address.split(':'))
        if operation in ['A', 'R'] and mac_address:
            self.serial.write(operation.encode() + mac_bytes)
            print(f"Peer {'added' if operation == 'A' else 'removed'}: {mac_address}")
        else:
            print("Invalid operation or MAC address")
            
        self.wait_for_acknowledgement()

    def send_control_params(self, mac_address, params):
        """Send control parameters (e.g., to move a robot) via ESP-NOW."""
        self.serial.reset_input_buffer()
        if len(params) != 13:
            raise ValueError("Expected 13 control parameters")

        mac_bytes = bytes(int(x, 16) for x in mac_address.split(':'))
        data = struct.pack('<6B13f', *mac_bytes, *params)  # Prefix data with MAC address
        self.serial.write(b'C' + data)  # 'C' indicates a control command
        self.wait_for_acknowledgement()

    def getSensorData(self):
        """Retrieve sensor data from the ESP32 via ESP-NOW."""
        self.serial.reset_input_buffer()
        self.serial.write(b'I')  # Request sensor data

        incoming = self.serial.readline().decode('utf-8').strip()
        if incoming.startswith("SENSOR_DATA:"):
            data_str = incoming.replace("SENSOR_DATA:", "")
            data_values = data_str.split(",")
            if len(data_values) == 6:
                try:
                    data = [float(value) for value in data_values]
                    self.values = data  # List of 6 floats
                except ValueError as e:
                    print(f"Error parsing sensor data: {e}")
                    self.values = None
            else:
                print("Received incorrect number of sensor values.")
                self.values = None
        else:
            print("Received unexpected data or no data.")
            self.values = None
        return self.values

    def wait_for_acknowledgement(self):
        """Wait for an acknowledgment or error message from ESP32."""
        time.sleep(0.01)
        try:
            incoming = self.serial.readline().decode('utf-8').strip()
            if not incoming:
                print("Timeout or no data received.")
                return False
            # Optionally process the acknowledgment message
            # For now, assume any message is an acknowledgment
            return True
        except serial.SerialException as e:
            print("Serial communication error:", e)
            return False

    def close(self):
        if self.serial:
            self.serial.close()
