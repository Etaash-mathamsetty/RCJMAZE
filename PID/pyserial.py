import time
import serial

serial_port = '/dev/ttyS0'

ser = serial.Serial(port = serial_port, baudrate = 115200)

values_dict = {}

while True:
    
    data = ser.readline().decode('ascii','ignore').split("::")
    data[1] = data[1][0:-2]
    split_data = [float (x) for x in data[1].split(',')]
    values_dict[data[0]] = split_data

    print(values_dict)
    
    
    #time.sleep(0.1)
