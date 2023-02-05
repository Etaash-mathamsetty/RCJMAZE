import cv2
import serial 

ser = serial.Serial('/dev/ttyUSB0')

#inp = input("Enter a command: ").strip()

while True:
	ser.write(b'gn')
	#inp = input("Enter a command: ").strip()  
ser.close()   

#print("py init done: simulation = " + str(simulation))
