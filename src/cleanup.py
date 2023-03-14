# variables are defined in vars.py (I did this to avoid multi threading)

if not simulation:
    video.release()
    video1.release()
    
cv2.destroyAllWindows()