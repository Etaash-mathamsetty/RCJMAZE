import cv2
import Robot as Rb
import serial 

print("Initializing Python subsystem")

simulation = Rb.isSimulation()
serial_port = '/dev/ttyS0'

if not simulation:
    video = cv2.VideoCapture(0)
    video1 = cv2.VideoCapture(1)
    if not video.isOpened():
        print("failed to open video 0")
    if not video1.isOpened():
        print("failed to open video 1")
    ser = serial.Serial(port = serial_port, baudrate = 115200)

def SendSerialCommand(command):
    print("sending command: " + command)
    try:
        b = bytearray()
        b.extend(map(ord, command))
        ser.write(b)
    except:
        return False
    return True

def DataIsAva():
    return ser.in_waiting >= 5


print("py init done: simulation = " + str(simulation))
