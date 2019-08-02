from AutmanRobot import TurtleBot
from AutmanRobot.States.State_FindBall import FindBall
from AutmanRobot.States.State_ObstaceAvoidance import ObstaceAvoidance
import cv2
import threading
import rospy
import smach
from AutmanRobot.Field import Field
import numpy as np
import time
import numpy.linalg as la
# import imutils
import math


##################################################################################################
def find_circ(frame,hsv):
    percent = 50
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    # resized = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
    # frame = resized
    ratio = (frame.shape[0])/height
    # hsv[...,2] = hsv[...,2]*0.4
    image=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    # (B, G, R) = cv2.split(image)
    # Y=np.uint8(np.around(0.3*R + 0.5*G + 0.11*B))
    # MAX=np.maximum(Y,B)
    # R[R<1.0*MAX]=0
    # G[G<1.0*MAX]=0
    # B[B<1.0*MAX]=0
    # filtered = cv2.merge([B,G,R])
    # cv2.imshow("Fil",filtered)
    lower_red = np.array([0,0,0])
    upper_red = np.array([25,255,255])

    # filteredd = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)
    filteredd = hsv
    mask_red = cv2.inRange(filteredd, lower_red, upper_red)
    mask_red = cv2.erode(mask_red, None, iterations=3)

    # s=1
    # if s==1:
    #     kernel = np.ones((5,5),np.uint8)
    #     dilation = cv2.dilate(mask_red,kernel,iterations=2)
    #     kernel = np.ones((15,15),np.uint8)
    #     opening=cv2.morphologyEx(dilation,cv2.MORPH_OPEN,kernel)
    #     mask_red = cv2.morphologyEx(opening,cv2.MORPH_CLOSE,kernel)
    # cv2.imshow("Frame",mask_red)
    _, cnts, _ = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    x=50
    y=50
    error=0
    distance = 50
    gate =1
    ball=0
    for c in cnts:
        M = cv2.moments(c)
        cX = int(((M["m10"] + 0.00001 )/( M["m00"] + 0.00001)) * ratio)
        cY = int(((M["m01"] + 0.00001 )/( M["m00"] + 0.00001)) * ratio)
        # c = c.astype("float")
        c = c.astype("int")
        appr = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True),True)
        if len(appr) >= 10 and len(appr)<=25:
            c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
            (x, y), raduis= cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            raduis= int(raduis)
            cv2.circle(frame, center, raduis, (0, 255, 0), 2)
            area = cv2.contourArea(c)
            mark = cv2.minAreaRect(c)
            KNOWN_DISTANCE = 19.68
            KNOWN_WIDTH = 2.56
            focalLength = ( 50 * KNOWN_DISTANCE) / KNOWN_WIDTH
            inch = int((KNOWN_WIDTH * focalLength) / (mark[1][0]))
            distance = inch * 2.54
            error= int((int(x)-int(frame.shape[1] /2))*6/37.8)
            gate=0
            ball=1

            # ang = int(15 * np.arcsin(z+0.001/cm+0.001))
    return x,y,error,distance,gate,ball
############################################################################
def p(error):
    w=-1*float(error)*10/200
    return w
############################################################################
if __name__ == "__main__":
    robot = TurtleBot.TB3("tayeb",[0,1,2])
    robot.init_node()
    robot.printInfo()
    robot.setGripper(0)
    robot.setCameraPos(90,70)
    t = time.time()
    ang = 0;
    prefered_color = 'red'
    ball_arr = []
    imu = robot.getOdometry()
    init_phi = imu[2]
    tt = time.time()
    temp = 0
    n_temp = 0
    ball_arr_num = []
    rotation_velocity = 0.5
    print("init phi",init_phi)
    while (True):
        robot.setVelocity(0,rotation_velocity)
        frame, hsv = robot.getFrame(color = "hsv")
        t = time.time()
        x, y, error, distance, gate, ball = find_circ(frame,hsv)
        dt = time.time()-t
        imu = robot.getOdometry()
        print(imu[2])
        phi = imu[2]
        if (ball==1):
            print("ball detected")
            if(len(ball_arr)==0 or (abs(ball_arr[-1] - phi) > 0.2 and abs(ball_arr[-1] - phi)<6.1)):
                ball_arr.append(phi)
                ball_arr_num.append(1)
            else:
                ball_arr[-1] = phi
                ball_arr_num[-1] = ball_arr_num[-1] + 1

        if (abs(phi - init_phi)<0.2 and t-tt > 5):
            print("end")
            robot.setVelocity(0,0)
            break

        cv2.imshow("Frame",frame)
        k=cv2.waitKey(1) & 0xFF
    ball_arr = [i-np.sign(rotation_velocity)*0.4 for i in ball_arr] ## corection of detected ball angle
    print(ball_arr)
    print(ball_arr_num)
    ball_detected = None
    if (len(ball_arr)>0 and max(ball_arr_num)>7):
        ball_detected = ball_arr[ball_arr_num.index(max(ball_arr_num))]
        # time.sleep(0.1)
    print(ball_detected)

