import cv2
import Robot as Rb
import serial
import numpy as np
import math

print("Initializing Python subsystem")

simulation = Rb.isSimulation()
serial_port = '/dev/ttyS0'
camera_num = True
resize_size = 20
label_txt = np.empty((0,1))
feature_txt = np.empty((0, resize_size ** 2))
knn = cv2.ml.KNearest_create()
feature_txt = np.loadtxt("feature.txt", np.float32)
label_txt = np.loadtxt("label.txt", np.float32).reshape((feature_txt.shape[0], 1))
knn.train(feature_txt, cv2.ml.ROW_SAMPLE, label_txt)

if not simulation:
    video = cv2.VideoCapture(0)
    video1 = cv2.VideoCapture(1)
    video.set(3, 320)
    video.set(4, 240)
    video1.set(3, 320)
    video1.set(4, 240)

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
