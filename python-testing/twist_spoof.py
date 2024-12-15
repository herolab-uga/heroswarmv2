import serial, struct

#device = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#! pi zero w1 
#device = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
#! pi zero w2
device = serial.Serial('/dev/ttyS0', 115200, timeout=1)

def uart_talker(values):

    print(values)
    # convert to bytes 
    byteList = bytes([0xBE, 0xEF]) + \
                struct.pack("i", 0) + \
                struct.pack('f'*len(values), *values) + \
                bytes('\n'.encode())
    print(byteList)
    device.write(byteList)

def mainloop(): 

    twist_lx = 1.0
    twist_ly = 0.0
    twist_az = 0.0

    uart_talker([twist_lx, twist_ly, twist_az]) 


if __name__ == '__main__':

    mainloop()




