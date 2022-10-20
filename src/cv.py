#used for testing python integration
import Robot as Rb
import cv2

frame = cv2.flip(video.read()[1], 1)
frame = cv2.cvtColor(frame, cv.COLOR_BGR2GRAY)
frame = cv2.medianBlur(frame, 5)

cv2.imshow("frame", frame)