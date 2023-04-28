import cv2
import Robot as Rb
import serial 

print("Initializing Python subsystem")

simulation = Rb.isSimulation()
serial_port = '/dev/ttyS0'

if not simulation:
    video = cv2.VideoCapture(0)
    video1 = cv2.VideoCapture(1)
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

print("py init done: simulation = " + str(simulation))
