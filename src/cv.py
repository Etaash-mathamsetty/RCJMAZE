#import Robot as Rb
#import cv2

debug_cv = True

def pre_process_img(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.medianBlur(frame, 5)
    return frame

frame = cv2.flip(video.read()[1], 1)
frame2 = cv2.flip(video1.read()[1], 1)
frame = pre_process_img(frame)
frame2 = pre_process_img(frame2)

#TODO:
Rb.SetDataValue("NRK", 0)

if(debug_cv):
    cv2.imshow("frame", frame)