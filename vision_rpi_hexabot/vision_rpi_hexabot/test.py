import numpy as np
import cv2 

caper = cv2.VideoCapture(0)

if not caper.isOpened():
    print("error, no camera detect")
    exit()

while True:
    ret, frame = caper.read()
    if not ret:
        print("cannot recieve data from camera (stream), Exiting process started... ")
        break
    print(frame)

caper.release()
cv2.destroyAllWindows()