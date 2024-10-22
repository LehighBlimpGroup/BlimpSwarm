import serial
import struct
import time
import codecs

# Message and data type definitions
MessageType_Parameter = 0x68
DataType_Int = 0x01
DataType_Float = 0x02
DataType_String = 0x03
DataType_Boolean = 0x04

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
        time.sleep(.02)
        self.serial.reset_input_buffer()

        mac_bytes = bytes(int(x, 16) for x in mac_address.split(':'))
        if operation in ['A', 'R'] and mac_address:
            self.serial.write(operation.encode() + mac_bytes)
            print(f"Peer {'added' if operation == 'A' else 'removed'}: {mac_address}")
        elif operation == 'G':  # Indicates a control command
            self.serial.write(operation.encode() + mac_bytes)
            print("Ground Station Added!")
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

    def send_preference(self, peer_mac, value_type, key, value):
        """Send preferences such as robot configurations."""
        self.serial.reset_input_buffer()
        time.sleep(.02)
        buffer = bytearray()

        if value_type in [DataType_Int, DataType_Float, DataType_Boolean]:
            buffer.extend(struct.pack('<6B', *[int(x, 16) for x in peer_mac.split(':')]))

        buffer.append(MessageType_Parameter)
        buffer.append(value_type)
        buffer.append(len(key))
        buffer.extend(key.encode('utf-8'))

        if value_type == DataType_Int:
            buffer.extend(struct.pack('<i', value))
        elif value_type == DataType_Float:
            buffer.extend(struct.pack('<f', value))
        elif value_type == DataType_String:
            buffer.extend(value.encode('utf-8'))
        elif value_type == DataType_Boolean:
            buffer.extend(struct.pack('<?', value))

        self.serial.write(b'D' + buffer)
        self.wait_for_acknowledgement()
        print("Sending Preference: ", key, ":", value, ", len:", codecs.encode(buffer, 'hex').decode())

    def getSensorData(self):
        """Retrieve sensor data from the ESP32 via ESP-NOW."""
        self.serial.reset_input_buffer()
        self.serial.write(b'I')  # Request sensor data

        incoming = self.serial.readline().strip()
        
        # Assuming ESP-NOW sends data in structured format (32 bytes for char, int, float, bool)
        expected_length = 32 + 4 + 4 + 1  # 32-char, int (4 bytes), float (4 bytes), bool (1 byte)
        
        if len(incoming) >= expected_length:
            try:
                # Unpack the incoming data
                data = struct.unpack('<32sif?', incoming[:expected_length])  # Char[32], int, float, bool
                string_value = data[0].decode('utf-8').strip('\x00')  # Decode char[32] and remove null padding
                int_value = data[1]
                float_value = data[2]
                bool_value = data[3]
                self.values = (string_value, int_value, float_value, bool_value)
            except struct.error as e:
                print(f"Error unpacking sensor data: {e}")
                self.values = None
        else:
            print("Received data is too short:", len(incoming))
            self.values = None
        return self.values

    def wait_for_acknowledgement(self):
        """Wait for an acknowledgment or error message from ESP32."""
        time.sleep(.01)
        try:
            incoming = self.serial.readline().strip()
            if not incoming:
                print("Timeout or no data received.")
                return False
            self.serial.flush()
            return True
        except serial.SerialException as e:
            print("Serial communication error:", e)
            return False

    def close(self):
        self.serial.close()


# Example usage
if __name__ == "__main__":
    port = 'COM5'  # Adjust as necessary for your setup
    controller = SerialController(port, timeout=.1)  # 5-second timeout

    try:
        # Example: Add a peer
        controller.manage_peer('A', "48:27:E2:E6:E1:00")
        controller.manage_peer('A', 'dc:54:75:d8:40:74')
        
        # Example: Send control parameters
        mac_address = 'dc:54:75:d8:40:74'  # Example target peer MAC address
        params = (1.0, 0.5, -1.2, 2.5, 0.0, 1.1, -0.9, 2.2, 3.3, 4.4, 5.5, -2.2, 0.1)
        controller.send_control_params(mac_address, params)

        # Example: Send preferences
        controller.send_preference(mac_address, DataType_Int, "controls", 25)

    finally:
        controller.close()
