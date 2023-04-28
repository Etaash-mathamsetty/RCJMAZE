import cv2

video = cv2.VideoCapture(0)
video1 = cv2.VideoCapture(1)

if not video.isOpened():
	print("failed to open video 0")
if not video1.isOpened():
	print("failed to open video 1")

while True:
	if video.isOpened():
		frame = video.read()[1]
	if video1.isOpened():
		frame1 = video1.read()[1]

	cv2.imshow("frame", frame)
	cv2.imshow("frame1", frame1)

	key = cv2.waitKey(1) & 0xFF

	if key == ord('q'):
		break


video.release()
video1.release()
cv2.destroyAllWindows()
