import cv2
import serial 

ser = serial.Serial('/dev/ttyUSB0')

#inp = input("Enter a command: ").strip()

while True:
	str_bytes = bytes(input("enter a command: ").encode('ascii'))
	ser.write(str_bytes)
	while(ser.in_waiting > 0):
		data = ser.readline()
		print(data)
	print(str_bytes)
	#inp = input("Enter a command: ").strip()  

ser.close()   

#print("py init done: simulation = " + str(simulation))
