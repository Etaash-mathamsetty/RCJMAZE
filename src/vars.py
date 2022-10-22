import cv2
import Robot as Rb
import serial 

simulation = Rb.isSimulation()
serial_port = '/dev/ttyS0'

if not simulation:
    video = cv2.VideoCapture(0)
    ser = serial.Serial(port = serial_port, baudrate = 115200)

def SendSerialCommand(command):
    print("sending command: " + command)

print("py init done: simulation = " + str(simulation))