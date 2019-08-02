from AutmanRobot import TurtleBot
# from AutmanRobot.States.State_FindBall import FindBall
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

######################____RESCALE_FRAME___###########################
def rescale_frame (frame, hsv, scale):
    width = int(frame.shape[1] * scale/100)
    height = int(frame.shape[0] * scale/100)
    dim = (width, height)
    resizedBGR = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
    resizedHSV = cv2.resize(hsv, dim, interpolation =cv2.INTER_AREA)
    return resizedBGR, resizedHSV, scale
#####################################################################



#####################___COLOR___###########################################################
def color(hsv):
    # hsv[...,2]=hsv[...,2]*0.4
    # hsv=cv2.medianBlur(hsv,19)

    (h,s,v) = cv2.split(hsv)
    s[s<=255]=0
    hsv = cv2.merge([h,s,v])
    
    # #################___RED__##############
    # lower_red = np.array([0,50,0])
    # upper_red = np.array([30,255,255])
    # #######################################

    # ################___YELLOW___###########
    # lower_yellow = np.array([30,0,0])
    # upper_yellow = np.array([68,255,255])
    # #######################################

    # ################___BLUE___#############
    # lower_blue = np.array([83,0,240])
    # upper_blue = np.array([108,255,255])
    # #######################################

    #################___RED__##############
    lower_red = np.array([0,0,200])
    upper_red = np.array([45,255,255])
    #######################################

    ################___YELLOW___###########
    lower_yellow = np.array([26,0,190])
    upper_yellow = np.array([61,255,255])
    #######################################

    ################___BLUE___#############
    lower_blue = np.array([80,0,255])
    upper_blue = np.array([255,255,255])
    #######################################






    ##########___MASK___##########
    
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    # maskr = cv2.bitwise_not(mask_r)
    # maskrr = cv2.erode(maskr, None, iterations=1)
    # mask_red = cv2.bitwise_not(maskrr)
    
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # masky = cv2.bitwise_not(mask_y)
    # maskyy = cv2.erode(masky, None, iterations=1)
    # mask_yellow = cv2.bitwise_not(maskyy)

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    # maskb = cv2.bitwise_not(mask_b)
    # maskbb = cv2.erode(maskb, None, iterations=1)
    # mask_blue = cv2.bitwise_not(maskbb)
    

    return  mask_red, mask_yellow, mask_blue
##############################################################################################

######################____Gate_FIND___###############################
def find_gate(frame, scale, cc):
    _, cnts, _ = cv2.findContours(cc, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    x = 50
    y = 50
    area = 0
    l = 0
    gate=0
    errorx= 0
    errory= 1000
    offset_errorx=25
    for c in cnts:
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]

        area = cv2.contourArea(c)
        appr = cv2.approxPolyDP(c, 0.05 * cv2.arcLength(c, True),True)
        l=len(appr)
        
        if area > 1000:
            if len(appr) > 3 and len(appr) <= 5:
                x,y,v,h = cv2.boundingRect(c)
                cv2.rectangle(frame,(x,y),(x+v,y+h),(0,255,0),4)
                center = (int(x+(v/2)), int(y+(h/2)))
                x = center[0];
                y = center[1];
                
                # x = appr.ravel()[0] + (appr.ravel()[0])/2
                # y = appr.ravel()[1] + (appr.ravel()[1])/2
                # cv2.drawContours(frame, [appr], 0, (0, 255, 0), 4)

                gate = 1
                errorx= int((int(x)+offset_errorx- float(frame.shape[1] /2))*15/37.8)
                errory= int((int(y)-float(frame.shape[0] /2))*15/37.8)
            


    return x, y, area, l, errorx, errory, gate

#####################################################################

#########################___P_CONTROLLER___##########################
def p(error):
    w=-1*float(error)/200
    return w
#####################################################################

#########################___SAF___##################################
def saf(errorx, gate, had):
    w=p(errorx)
    s = 0
    print ("w   :   "+str(w))
    if abs(w)>=had:
        s=0
        ang= 0.8*w
        robot.setVelocity(0,ang)
        time.sleep(0.005) 
    else :
        w = 0  
        s=1
    return s
#####################################################################


if __name__ == "__main__":
    robot = TurtleBot.TB3("sina",[0,1,2])
    robot.init_node()
    robot.printInfo()
    teta = 80
    offset_pan=105
    robot.setCameraPos(offset_pan,teta)
    prefered_color="yellow"

    while (True):
        t1=time.time()

        ###___getting_frames_from_webcam___###
        frame, hsv = robot.getFrame(color = "hsv")

        ###___Rescale_the_frame___###
        resizedBGR, resizedhsv, scale = rescale_frame(frame, hsv, 50)

        ###___extract_the_needed_colors_mask___###
        red, yellow, blue= color(resizedhsv)

        ###___get_the_details_of_each_color___###
        dict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=dict[prefered_color]


        x, y, area, l, errorx, errory, gate = find_gate(resizedhsv, scale, prefered_mask)
        # cv2.imshow("Frame",resizedhsv)
        # cv2.imshow(prefered_color,prefered_mask)

        k=cv2.waitKey(1) & 0xFF
        print ("While 1")
        s = saf(errorx, gate, 0.04)

        t2=time.time()

        print ("Time 1      ===  "+str(t2-t1))
       
        if s == 1 and gate==1 or k==27:
            robot.setVelocity(0 ,0)
            break

    
    time.sleep(1)
    robot.getFrame()

    while (True):
        robot.setVelocity(0.05,0)
        time.sleep(0.06)
        frame, hsv = robot.getFrame(color = "hsv")
        resizedBGR , resizedhsv, scale = rescale_frame(frame,hsv, 50)
        red, yellow, blue= color(resizedhsv)

        dict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=dict[prefered_color]

        # cv2.imshow("Frame",resizedhsv)
        # cv2.imshow(prefered_color,prefered_mask)

        k=cv2.waitKey(1) & 0xFF
        
        x, y, area, l, errorx, errory, gate = find_gate(resizedhsv, scale, prefered_mask)
        
        print("while 2   :::::::::::::::     "+ str(errory))

        s = saf(errorx, gate, 0.1)

        if abs(errory) < 100:
            robot.setCameraPos(offset_pan,30)
            

        print ("Area   :   " + str (area) + "     gate :   " + str(gate))
        if area > 50000 and gate == 1 or k==27:
            time.sleep(2.5)
            robot.setGripper(0)
            time.sleep(1)
            robot.setVelocity(0,0)
            break

 
        

       

