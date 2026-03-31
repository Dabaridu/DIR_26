import numpy as np 
import cv2
import math 

webcam = cv2.VideoCapture(0)
window = "Yaskawa color detection" 
cv2.namedWindow(window) 

while(1): 
    _, frame = webcam.read() 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # set red color range 
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)

    kernal = np.ones((32,32), "uint8")

    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(frame, frame, mask=red_mask)

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour) 

        if (math.fabs((w/h)-1.0) >= 0.2): 
            continue

        if (area < 4000): 
            continue 

        frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 2)

    cv2.imshow(window, frame)
    k = cv2.waitKey(1)

    if k == 27:
        break
