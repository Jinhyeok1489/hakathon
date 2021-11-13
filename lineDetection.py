# -*- coding: utf-8 -*-

import sys
import time
import cv2
import numpy as np
import os
from jetbot import Robot



# GStreamer 명령어 파이프라인 작성 및 파라미터 조정

def gstreamer_pipeline(

    capture_width=640,

    capture_height=480,

    display_width=640,

    display_height=480,

    framerate=60,

    flip_method=0,

):

    return (

        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"

        % (

            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )

    )

Kernel_size=15

low_threshold=40
high_threshold=120

rho=10
threshold=15
theta=np.pi/180
minLineLength=10
maxLineGap=1
print(gstreamer_pipeline(flip_method=0))
#Initialize camera
video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
linePos_1 = 300
linePos_2 = 400
lineColorSet = 0

while True:
    # CAPTURE FRAME-BY-FRAME
    ret, frame = video_capture.read()
    time.sleep(0.1)
    #Convert to Grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # Use threshold function for image conversion
    retval, frame_findline = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    # Image erosion 
    frame_findline = cv2.erode(frame_findline, None, iterations = 6)
    colorPos_1 = frame_findline[linePos_1]
    colorPos_2 = frame_findline[linePos_2]  

    try:
        lineColorCount_Pos1 = np.sum(colorPos_1 == lineColorSet)
        lineColorCount_Pos2 = np.sum(colorPos_2 == lineColorSet)
        lineIndex_Pos1 = np.where(colorPos_1 == lineColorSet)
        lineIndex_Pos2 = np.where(colorPos_2 == lineColorSet)

        if lineColorCount_Pos1 == 0:
            lineColorCount_Pos1 = 1
        if lineColorCount_Pos2 == 0:
            lineColorCount_Pos2 = 1
        left_Pos1 = lineIndex_Pos1[0][lineColorCount_Pos1-1]
        right_Pos1 = lineIndex_Pos1[0][0] 
        center_Pos1 = int((left_Pos1+right_Pos1)/2)
        left_Pos2 = lineIndex_Pos2[0][lineColorCount_Pos2-1]
        right_Pos2 = lineIndex_Pos2[0][0]
        center_Pos2 = int((left_Pos2+right_Pos2)/2)
        center = int((center_Pos1 + center_Pos2)/2)
    except:
        center = None
        pass
    print(center)
    try:
        cv2.line(frame, (left_Pos1, (linePos_1+30)), (left_Pos1, (linePos_1-30)), (255, 128, 64), 1)
        cv2.line(frame, (right_Pos1, (linePos_1+30)), (right_Pos1, (linePos_1-30)), (64, 128, 255), 1)
        cv2.line(frame, (0, linePos_1) , (640, linePos_1), (255, 255, 64), 1)
        cv2.line(frame, (left_Pos2, (linePos_2+30)), (left_Pos2, (linePos_2-30)), (255, 128, 64), 1)
        cv2.line(frame, (right_Pos2, (linePos_2+30)), (right_Pos1, (linePos_2-30)), (64, 128, 255), 1)
        cv2.line(frame, (0, linePos_2), (640, linePos_2), (255, 255, 64), 1)
    except:
        pass
    robot = Robot()
    turn_value = 0
    tuning_factor = 0.01
    forward_vel = 0.5
    if center < 300: # If center point is located left to center -20
        print("Turn left")
        turn_value = abs((center-320)*tuning_factor)
        robot.left(turn_value)
        print("Turn value: ", turn_value)
        time.sleep(1.00)
    elif center > 340: # If center point is located right to center +20
        print("Turn right")
        turn_value = abs((center-320)*0.01)
        robot.right(turn_value)
        print("Turn value: ", turn_value)
        time.sleep(1.00)
    else:
        print("Go straight")
        robot.forward(forward_vel)
        print("Forward value: ", forward_vel)
    cv2.imshow("line detect test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):

        break

# When everything is done, release the capture
video_capture.release()

cv2.destroyAllWindows()