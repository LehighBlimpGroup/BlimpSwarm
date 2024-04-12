

from pyb import UART

uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)

def checksum(arr, initial= 0):
    """ The last pair of byte is the checksum on iBus
    """
    sum = initial
    for a in arr:
        sum += a
    checksum = 0xFFFF - sum
    chA = checksum >> 8
    chB = checksum & 0xFF
    return chA, chB

def IBus_message(message_arr_to_send):
    msg = bytearray(32)
    msg[0] = 0x20
    msg[1] = 0x40
    for i in range(len(message_arr_to_send)):
        msg_byte_tuple = bytearray(message_arr_to_send[i].to_bytes(2, 'little'))
        msg[int(2*i + 2)] = msg_byte_tuple[0]
        msg[int(2*i + 3)] = msg_byte_tuple[1]

    # Perform the checksume
    chA, chB = checksum(msg[:-2], 0)
    msg[-1] = chA
    msg[-2] = chB
    
    uart.write(msg)

def refreshIbusConnection():
    if uart.any():
        uart_input = uart.read()


############### Color detection here ###############

#put color detection and such here



color_is_detected = False # replace this with your method


flag = 0
if (color_is_detected):
    flag = 1 # when a color is detected make this flag 1
pixels_x = 0 # put your x center in pixels
pixels_y = 0 # put your y center in pixels
pixels_w = 0 # put your width in pixels (these have almost no affect on control (for now))
pixels_h = 0 # put your height center in pixels (these have almost no affect on control (for now))
###############################




messageToSend = [flag, pixels_x, pixels_y, pixels_w, pixels_h]

IBus_message(messageToSend)
refreshIbusConnection()