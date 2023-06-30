import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:
        break