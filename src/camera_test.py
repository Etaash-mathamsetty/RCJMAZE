import cv2

video = cv2.VideoCapture(0, cv2.CAP_V4L2)
video1 = cv2.VideoCapture(1, cv2.CAP_V4L2)

if not video.isOpened():
	print("failed to open video 0")
if not video1.isOpened():
	print("failed to open video 1")

while True:
	if video.isOpened():
		video.set(cv2.CAP_PROP_BUFFERSIZE, 1)
		frame = video.read()[1]
		cv2.imshow("frame", frame)
	if video1.isOpened():
		video1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
		frame1 = video1.read()[1]
		cv2.imshow("frame1", frame1)

	key = cv2.waitKey(1) & 0xFF

	if key == ord('q'):
		break


video.release()
video1.release()
cv2.destroyAllWindows()
