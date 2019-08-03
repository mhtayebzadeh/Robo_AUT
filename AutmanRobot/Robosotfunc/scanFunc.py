from AutmanRobot import TurtleBot
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

######################____RESCALE_FRAME___###########################
def rescale_frame (frame ,hsv, scale):
    width = int(frame.shape[1] * scale/100)
    height = int(frame.shape[0] * scale/100)
    dim = (width, height)
    resizedBGR = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
    resizedHSV = cv2.resize(hsv, dim, interpolation =cv2.INTER_AREA)
    return resizedBGR , resizedHSV, scale
#####################################################################

##############___COLORS___##########################################

def color(robot ,hsv):
    # hsv[...,2] = hsv[...,2]*0.4
    #################___RED__##############
    # lower_red = np.array([0,45,0])
    # upper_red = np.array([21,225,255])
    #######################################

    ################___YELLOW___###########
    # lower_yellow = np.array([31,5,100])
    # upper_yellow = np.array([65,255,105])
    #######################################

    ################___BLUE___#############
    # lower_blue = np.array([85,0,102])
    # upper_blue = np.array([100,225,255])
    #######################################

    ################___WHITE___#############
    # lower_white = np.array([250,250,250])
    # upper_white = np.array([255,255,255])
    #######################################

    lower_blue = robot.lower_blue_ball
    upper_blue = robot.upper_blue_ball

    lower_red1 = robot.lower_red1_ball
    upper_red1 = robot.upper_red1_ball
    
    lower_red2 = robot.lower_red2_ball
    upper_red2 = robot.upper_red2_ball

    lower_yellow = robot.lower_yellow_ball
    upper_yellow = robot.upper_yellow_ball

    ##########___MASK___##########

    ###########___REMOVE_WHITE___#####
    # mask_white = cv2.inRange(frame, lower_white, upper_white)
    # maskw = cv2.bitwise_not(mask_white)
    
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    # maskr = cv2.bitwise_not(mask_red)
    # maskrr = cv2.erode(maskr, None, iterations=2)
    # mask_red = cv2.bitwise_not(maskrr)
    mask_red = cv2.bitwise_or(mask_red1 , mask_red2)

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # mask_yellow = cv2.bitwise_and(mask_yellow, mask_yellow ,mask=maskw)

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)


    return  mask_red, mask_yellow, mask_blue
#####################################################################

######################____TOOP_FIND___###############################
def find_circ(frame, scale, cc):
    _, cnts, _ = cv2.findContours(cc, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    x=0
    y=0
    errorx=0
    errory=0
    gate =1
    ball=0
    area = 0
    radius = 0
    for c in cnts:
        M = cv2.moments(c)
        cX = int(((M["m10"] + 0.00001 )/( M["m00"] + 0.00001)) / scale)
        cY = int(((M["m01"] + 0.00001 )/( M["m00"] + 0.00001)) / scale)
        # c = c.astype("float")
        c = c.astype("int")
        appr = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True),True)
        if len(appr) >= 10 and len(appr)<=25:
            area = cv2.contourArea(c)
            if area >= 500 and area <= 11000:
                c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
                area = cv2.contourArea(c)
                (x, y), radius= cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius= int(radius)
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                errorx= int((int(x)-float(frame.shape[1] /2))*30/37.8)
                errory= int((int(y)-float(frame.shape[0] /2))*30/37.8)
                gate=0
                ball=1

    return ball

#####################################################################

#########################___P_CONTROLLER___##########################
def p(errorx):
    w=-1*float(errorx)*10/200/20
    return w
#####################################################################

def doScan(robot,prefered_color = 'red',time_out=25):
    robot.setGripper(90)
    robot.setCameraPos(105,70)
    # t = time.time()
    ang = 0;
    frame, hsv = robot.getFrame(color = "hsv")
    resizedBGR , resizedHSV, scale = rescale_frame(frame ,hsv, 50)
    red, yellow, blue= color(robot , resizedHSV)
    ball_red = find_circ(resizedHSV, scale, red)
    ball_yellow = find_circ(resizedHSV, scale, yellow)
    ball_blue = find_circ(resizedHSV, scale, blue)
    # cv2.imshow("Frame",frame)
    # k=cv2.waitKey(1) & 0xFF

    # prefered_color = 'red'
    ball_red_arr = []
    ball_yellow_arr = []
    ball_blue_arr = []
    rotation_velocity = -0.5
    robot.setVelocity(0,rotation_velocity)
    time.sleep(0.2)
    for i in range(10):
        imu = robot.getOdometry()
        time.sleep(0.05)
    imu = robot.getOdometry()
    init_phi = imu[2]
    tt = time.time()
    temp = 0
    n_temp = 0
    ball_red_arr_num = []
    ball_yellow_arr_num = []
    ball_blue_arr_num = []

    
    print("init phi",init_phi)
    while (time.time() - tt < time_out):
        robot.setVelocity(0,rotation_velocity)
        frame, hsv = robot.getFrame(color = "hsv")
        t = time.time()
        resizedBGR , resizedHSV, scale = rescale_frame(frame ,hsv, 50)
        red, yellow, blue= color(robot , resizedHSV)

        ########################################################
        ball_red = find_circ(resizedHSV, scale, red)
        ball_yellow = find_circ(resizedHSV, scale, yellow)
        ball_blue = find_circ(resizedHSV, scale, blue)
        ########################################################
        dt = time.time()-t
        imu = robot.getOdometry()
        print(imu[2])
        phi = imu[2]
###################################################################
        if (ball_red==1):
            print("ball RED detected")
            if(len(ball_red_arr)==0 or (abs(ball_red_arr[-1] - phi) > 0.2 and abs(ball_red_arr[-1] - phi)<6.1)):
                ball_red_arr.append(phi)
                ball_red_arr_num.append(1)
            else:
                ball_red_arr[-1] = phi
                ball_red_arr_num[-1] = ball_red_arr_num[-1] + 1

        if (ball_yellow==1):
            print("ball YELLOW detected")
            if(len(ball_yellow_arr)==0 or (abs(ball_yellow_arr[-1] - phi) > 0.2 and abs(ball_yellow_arr[-1] - phi)<6.1)):
                ball_yellow_arr.append(phi)
                ball_yellow_arr_num.append(1)
            else:
                ball_yellow_arr[-1] = phi
                ball_yellow_arr_num[-1] = ball_yellow_arr_num[-1] + 1

        if (ball_blue==1):
            print("ball BLUE detected")
            if(len(ball_blue_arr)==0 or (abs(ball_blue_arr[-1] - phi) > 0.2 and abs(ball_blue_arr[-1] - phi)<6.1)):
                ball_blue_arr.append(phi)
                ball_blue_arr_num.append(1)
            else:
                ball_blue_arr[-1] = phi
                ball_blue_arr_num[-1] = ball_blue_arr_num[-1] + 1
            ball_red_arr = [i-np.sign(rotation_velocity)*0.4 for i in ball_red_arr] ## corection of detected ball angle

        arr_dict = {'red':ball_red_arr , 'yellow':ball_yellow_arr , 'blue':ball_blue_arr }
        arr_num_dict = {'red':ball_red_arr_num , 'yellow':ball_yellow_arr_num , 'blue':ball_blue_arr_num }

        if ((abs(phi - init_phi) < 0.15 and t-tt > 5)or (len(arr_dict[prefered_color])>0 and max(arr_num_dict[prefered_color])>7)):
            print("end")
            robot.setVelocity(0,0)
            break

        # cv2.imshow("Frame",frame)
        # k=cv2.waitKey(1) & 0xFF

    ball_red_arr = [i-np.sign(rotation_velocity)*0.4 for i in ball_red_arr] ## corection of detected ball angle
    print(ball_red_arr)
    print(ball_red_arr_num)
    ball_red_detected = None
    if (len(ball_red_arr)>0 and max(ball_red_arr_num)>7):
        ball_red_detected = ball_red_arr[ball_red_arr_num.index(max(ball_red_arr_num))]
        # time.sleep(0.1)
    print(ball_red_detected)

    ball_yellow_arr = [i-np.sign(rotation_velocity)*0.4 for i in ball_yellow_arr] ## corection of detected ball angle
    print(ball_yellow_arr)
    print(ball_yellow_arr_num)
    ball_yellow_detected = None
    if (len(ball_yellow_arr)>0 and max(ball_yellow_arr_num)>7):
        ball_yellow_detected = ball_yellow_arr[ball_yellow_arr_num.index(max(ball_yellow_arr_num))]
        # time.sleep(0.1)
    print(ball_yellow_detected)

    ball_blue_arr = [i-np.sign(rotation_velocity)*0.4 for i in ball_blue_arr] ## corection of detected ball angle
    print(ball_blue_arr)
    print(ball_blue_arr_num)
    ball_blue_detected = None
    if (len(ball_blue_arr)>0 and max(ball_blue_arr_num)>7):
        ball_blue_detected = ball_blue_arr[ball_blue_arr_num.index(max(ball_blue_arr_num))]
        # time.sleep(0.1)
    print(ball_blue_detected)
    color_dict = {'red':ball_red_detected , 'yellow':ball_yellow_detected , 'blue':ball_blue_detected }
    find_color = None
    find_color_phi = None

    if color_dict['red'] != None:
        find_color = 'red'
        find_color_phi = color_dict['red']
    if color_dict['blue'] != None:
        find_color = 'blue'
        find_color_phi = color_dict['blue']
    if color_dict['yellow'] != None:
        find_color = 'yellow'
        find_color_phi = color_dict['yellow']
        
    if color_dict[prefered_color] != None:
        find_color = prefered_color
        find_color_phi = color_dict[prefered_color]

    print("color , phi = ",find_color,find_color_phi) 
    
    robot.find_ball_color = find_color
    robot.find_ball_phi = find_color_phi  

    if find_color == None:
        return "notFound"
    else :
        return "found"

if __name__ == "__main__":
    robot = TurtleBot.TB3("tayeb",[0,1,2])
    robot.init_node()
    robot.printInfo()
    doScan(robot)