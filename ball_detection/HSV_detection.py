import cv2
import numpy as np
import os
import time
import imutils

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow("Trackbars")

light = False
if light==False:
    cv2.createTrackbar("L – H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L – S", "Trackbars", 88, 255, nothing)
    cv2.createTrackbar("L – V", "Trackbars", 117, 255, nothing)
    cv2.createTrackbar("U – H", "Trackbars", 7, 179, nothing)
    cv2.createTrackbar("U – S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U – V", "Trackbars", 237, 255, nothing)

    cv2.createTrackbar("minval_canny", "Trackbars", 108, 255, nothing)
    cv2.createTrackbar("maxval_canny", "Trackbars", 133, 255, nothing)

if light==True:
    cv2.createTrackbar("L – H", "Trackbars", 172, 179, nothing)
    cv2.createTrackbar("L – S", "Trackbars", 70, 255, nothing)
    cv2.createTrackbar("L – V", "Trackbars", 81, 255, nothing)
    cv2.createTrackbar("U – H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U – S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U – V", "Trackbars", 252, 255, nothing)

    cv2.createTrackbar("minval_canny", "Trackbars", 108, 255, nothing)
    cv2.createTrackbar("maxval_canny", "Trackbars", 133, 255, nothing)

_, frame = cap.read()
h,w = frame.shape[:2]

# initialize video writer
#video = cv2.VideoWriter('ball_golf_detected.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 40, (w,h))

while True:
    _, frame = cap.read()
    blurred = cv2.GaussianBlur(frame, (11,11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #print(hsv[390,340])
    l_h = cv2.getTrackbarPos("L – H", "Trackbars")
    l_s = cv2.getTrackbarPos("L – S", "Trackbars")
    l_v = cv2.getTrackbarPos("L – V", "Trackbars")
    u_h = cv2.getTrackbarPos("U – H", "Trackbars")
    u_s = cv2.getTrackbarPos("U – S", "Trackbars")
    u_v = cv2.getTrackbarPos("U – V", "Trackbars")
    minval = cv2.getTrackbarPos("minval_canny", "Trackbars")
    maxval = cv2.getTrackbarPos("maxval_canny", "Trackbars")

    # Describe the lower and upper boundaries of the 
    # orange ball in the HSV space 
    lower_ball = np.array([l_h, l_s, l_v])
    upper_ball = np.array([u_h, u_s, u_v])

    # Create a mask for the color orange, then perform
    # dilatation and erosion to remove smalls blobs left in the mask
    mask = cv2.inRange(hsv, lower_ball, upper_ball)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the ball's center
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts)>0:
        c = max(cnts, key=cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)

        if radius>5:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,0),2)
            #cv2.circle(frame, center, 5, (255,0,0), -1)
            cv2.circle(frame, (int(x), int(y)), 3, (255,0,0), -1)

    #video.write(frame)
    cv2.imshow("Frame", frame)
    cv2.imshow("mask", mask)

    key = cv2.waitKey(100) & 0xFF
	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

cap.release()
#video.release()
cv2.destroyAllWindows()