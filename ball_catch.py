from AutmanRobot import TurtleBot
from AutmanRobot.States.State_ObstaceAvoidance import ObstaceAvoidance



import cv2
import threading
import rospy
import smach
from AutmanRobot.Field import Field
import numpy as np
import time
import numpy.linalg as la
import math
from operator import xor
import sys


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

def color(hsv):
    lower_red = np.array([0,136,87])
    upper_red = np.array([17,254,255])
    # #######################################

    # ################___YELLOW___###########
    lower_yellow = np.array([20,36,250])
    upper_yellow = np.array([40,255,255])
    # #######################################

    # ################___BLUE___#############
    lower_blue = np.array([84,0,255])
    upper_blue = np.array([108,255,255])
    # ######################################

    ##########___MASK___##########

    ###########___REMOVE_WHITE___#####
    # mask_white = cv2.inRange(frame, lower_white, upper_white)
    # maskw = cv2.bitwise_not(mask_white)
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    maskr = cv2.bitwise_not(mask_red)
    maskrr = cv2.erode(maskr, None, iterations=2)
    mask_red = cv2.bitwise_not(maskrr)

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # mask_yellow = cv2.bitwise_and(mask_yellow, mask_yellow ,mask=maskw)
    masky = cv2.bitwise_not(mask_yellow)
    maskyy = cv2.erode(masky, None, iterations=2)
    mask_yellow = cv2.bitwise_not(maskyy)

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    return  mask_red, mask_yellow, mask_blue
#####################################################################

######################____TOOP_FIND___###############################
def find_circ(frame, scale, cc):
    _, cnts, _ = cv2.findContours(cc, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    x=50
    y=50
    errorx=0
    errory=0
    gate =1
    ball=0
    area = 0
    radius = 0
    offset_errorx=25
    cnts_sort = sorted(cnts, key=cv2.contourArea, reverse=True)
    if len(cnts_sort) > 2 :
        cnts = cnts_sort[0:1]
    else:
        cnts = cnts_sort

    arr = []
    res = []
    for c in cnts:
        # M = cv2.moments(c)
        # cX = int(((M["m10"] + 0.00001 )/( M["m00"] + 0.00001)) / scale)
        # cY = int(((M["m01"] + 0.00001 )/( M["m00"] + 0.00001)) / scale)
        # # c = c.astype("float")
        # c = c.astype("int")
        appr = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True),True)
        if len(appr) >= 10 and len(appr)<=25:
            area = cv2.contourArea(c)
            if area >= 500 and area <= 11000:
                # c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
                (x, y), raduis= cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                raduis= int(raduis)
                cv2.circle(frame, center, raduis, (0, 255, 0), 2)
                # area = cv2.contourArea(c)
                # print(str(area))
                errorx= int((int(x)+offset_errorx-float(frame.shape[1] /2))*15/37.8)
                errory= int((int(y)-float(frame.shape[0] /2))*25/37.8)
                arr.append(errorx)
                gate=0
                ball=1
                res.append([x,y,errorx,errory,gate,ball,area,radius])

    if len(arr) >= 2 and arr[0]<arr[1]:
        x,y,errorx,errory,gate,ball,area,radius = res[0]
        
    elif len(arr) >= 2:
        x,y,errorx,errory,gate,ball,area,radius = res[1]
        print("res1",res[1])
    else:
        pass

    return x,y,errorx,errory,gate,ball,area,radius

#####################################################################

#########################___P_CONTROLLER___##########################
def p(errorx):
    w=-1*float(errorx)*2/200
    return w
#####################################################################

#########################___SAF___##################################
def saf(errorx, ball, had):
    w=p(errorx)
    # print('error:   '+ str(errorx) +'w:      '+ str(w))
    s = 0
    if abs(w)>=had:
        s=0
        ang= 0.7*w
        robot.setVelocity(0,ang)
        time.sleep(0.005) 
    else :
        w = 0  
        s=1
    return s
#####################################################################


#########################____Charkh___###############################
def charkh (angle, phi):
    angle = angle + math.pi
    phi = phi + math.pi

    err = phi - angle
    print ("Angle   :::   "+ str(angle)+ "       Phi  ::    "+ str(phi)+"     Error    :::  " + str(err))
    
    if 2*math.pi - err <=3.14:
        z = err
    else:
        z=-err

    if z > 0.5:
        z=0.5
        
    if z < -0.5:
        z=-0.5
    
    return z,err
#####################################################################


if __name__ == "__main__":
    robot = TurtleBot.TB3("sina",[0,1,2])
    robot.init_node()
    robot.printInfo()
    robot.setGripper(90)
    teta = 70
    offset_pan=105
    robot.setCameraPos(offset_pan,teta)
    prefered_color="blue"
    prefered_angle=1.4
    first_time =  0
    bbox = (100, 100, 200, 200)
    # tracker = cv2.TrackerCSRT_create()
    ####__ANGLE___####
    while (True):
        imu = robot.getOdometry()
        phi = imu[2]
        z,err = charkh (angle, phi)
        robot.setVelocity(0 ,2*z)
        time.sleep(0.05)
        if abs(z) <0.03:
            robot.setVelocity(0,0)
            break
    ###################

    while (True):
        t1=time.time()

        ###___getting_frames_from_webcam___###
        frame, hsv = robot.getFrame(color = "hsv")

        ###___Rescale_the_frame___###
        resizedBGR , resizedhsv, scale = rescale_frame(frame ,hsv, 50)
        

        ###___extract_the_needed_colors_mask___###
        red, yellow, blue= color(resizedhsv)

        ###___get_the_details_of_each_color___###
        dict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=dict[prefered_color]
        x, y, errorx, errory, gate, ball, area , r = find_circ(resizedhsv, scale, prefered_mask)

        print("while 1")
        # if ball == 1 and first_time == 0:
        #     first_time = 1
        #     bbox = (x-r, y-r, x+r, y+r) 
        #     ok = tracker.init(resizedBGR, bbox)
        # elif ball ==1:
        #     ok, bbox = tracker.update(resizedBGR)
        #     p1 = (int(bbox[0]), int(bbox[1]))
        #     p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        #     cv2.rectangle(resizedBGR, p1, p2, (255,0,0), 2, 1)

        s = saf(errorx, ball, 0.06)
        # print ("area :      "+str(vv))
        cv2.imshow("Frame",resizedBGR)
        cv2.imshow(prefered_color,prefered_mask)

        k=cv2.waitKey(1) & 0xFF

         
        if s == 1 and ball==1:
            robot.setVelocity(0 ,0)
            break

        t2=time.time()
    
    robot.setCameraPos(offset_pan,20)
    time.sleep(1)
    robot.getFrame()
    cnt = 0

    while (True):
        robot.setVelocity(0.05,0)
        time.sleep(0.06)

        ###___getting_frames_from_webcam___###
        frame, hsv = robot.getFrame(color = "hsv")

        ###___Rescale_the_frame___###
        resizedBGR , resizedhsv, scale = rescale_frame(frame,hsv, 50)

        ###___extract_the_needed_colors_mask___###
        red, yellow, blue= color(resizedhsv)

        ###___get_the_details_of_each_color___###
        dict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=dict[prefered_color]
        x, y, errorx, errory, gate, ball, area , radius = find_circ(resizedhsv, scale, prefered_mask)
        if y >= 25 and ball==1:
            robot.setGripper(0)
            print ("Seee Seee Seee")
            
        print("while 2")
        s = saf(errorx, ball, 0.06)
        
        
        cv2.imshow(prefered_color,prefered_mask)
        k=cv2.waitKey(1) & 0xFF

        print("y = ",y)
        if ball == 1 and y >= 180 and s==1:
            cnt = cnt + 1
            if cnt > 4:
                robot.setVelocity(0.05,0)
                time.sleep(0.05)
                robot.setGripper(20)
                time.sleep(0.05)
                robot.setGripper(30)
                time.sleep(0.05)
                robot.setGripper(40)
                time.sleep(0.05)
                robot.setGripper(50)
                time.sleep(0.05)
                robot.setGripper(55)
                time.sleep(0.05)
                robot.setVelocity(0,0)
                break
        

    
    
    






