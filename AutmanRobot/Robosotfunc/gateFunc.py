import sys
sys.path.insert(0, '../..')

from AutmanRobot import TurtleBot
import cv2
import threading
import rospy
import smach
# from AutmanRobot.Field import Field
import numpy as np
import time
import numpy.linalg as la
import math
from operator import xor

######################___FPS___######################################
_tick2_frame=0
_tick2_fps=20000000 # real raw FPS
_tick2_t0=time.time()

def tick(fps=240):
    global _tick2_frame,_tick2_fps,_tick2_t0
    n=_tick2_fps/fps
    _tick2_frame+=n
    while n>0: 
        n-=1
        if time.time()-_tick2_t0>1:
            _tick2_t0=time.time()
            _tick2_fps=_tick2_frame
            _tick2_frame=0

#####################################################################
######################____RESCALE_FRAME___###########################
def rescale_frame (frame, hsv, scale):
    # hsv = cv2.blur(hsv,(11,11))
    # frame = cv2.blur(frame,(11,11))
    width = int(frame.shape[1] * scale/100)
    height = int(frame.shape[0] * scale/100)
    dim = (width, height)
    resizedBGR = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
    resizedHSV = cv2.resize(hsv, dim, interpolation =cv2.INTER_AREA)
    return resizedBGR, resizedHSV, scale
#####################################################################



#####################___COLOR___###########################################################
def color(robot , frame,hsv):
    # hsv[...,2]=hsv[...,2]*0.4
    # hsv=cv2.medianBlur(hsv,19)

    # (h,s,v) = cv2.split(hsv)
    # s[s<=255]=0
    # hsv = cv2.merge([h,s,v])
    
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
    lower_red1 = robot.lower_red1_ball
    upper_red1 = robot.upper_red1_ball
    lower_red2 = robot.lower_red2_ball
    upper_red2 = robot.upper_red2_ball
    #######################################

    ################___YELLOW___###########
    lower_yellow = robot.lower_yellow_ball
    upper_yellow = robot.upper_yellow_ball
    #######################################

    ################___BLUE___#############
    lower_blue = robot.lower_blue_ball
    upper_blue = robot.upper_blue_ball
    #######################################






    ##########___MASK___##########
    
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1 , mask_red2)

    # maskr = cv2.bitwise_not(mask_r)
    # maskrr = cv2.erode(maskr, None, iterations=1)
    # mask_red = cv2.bitwise_not(maskrr)
    
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # masky = cv2.bitwise_not(mask_yellow)
    # maskyy = cv2.erode(masky, None, iterations=1)
    # mask_yellow = cv2.bitwise_not(maskyy)

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    # maskb = cv2.bitwise_not(mask_blue)
    # mask_bb = cv2.erode(maskb, None, iterations=1)
    # mask_blue = cv2.bitwise_not(mask_bb)
    

    return  mask_red, mask_yellow, mask_blue
##############################################################################################

######################____Gate_FIND___###############################
def find_gate(frame, scale, cc):
    _, cnts, _ = cv2.findContours(cc, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    x = 50
    y = 50
    b=0
    d=10
    area = 0
    l = 0
    gate=0
    errorx= 50
    errory= 1000
    offset_errorx=0
    for c in cnts:
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]

        area = cv2.contourArea(c)
        appr = cv2.approxPolyDP(c, 0.05 * cv2.arcLength(c, True),True)
        l=len(appr)
        
        if area > 1000:
            if l >=3 and l <= 5:
                a,b,c,d = cv2.boundingRect(c)
                cv2.rectangle(frame,(a,b),(a+c,b+d),(0,255,0),4)
                center = (int(a+(c/2)), int(b+(d/2)))
                x = center[0];
                y = center[1];
                
                # x = appr.ravel()[0] + (appr.ravel()[0])/2
                # y = appr.ravel()[1] + (appr.ravel()[1])/2
                # cv2.drawContours(frame, [appr], 0, (0, 255, 0), 4)

                gate = 1
                errorx= int((int(x)+offset_errorx- float(frame.shape[1] /2))*15/37.8)
                errory= int((int(y)-float(frame.shape[0] /2))*15/37.8)
            


    return b, d, x, y, area, l, errorx, errory, gate

#####################################################################

#########################___P_CONTROLLER___##########################
def p(error):
    w=-1*float(error)/200
    return w
#####################################################################

#########################___SAF___##################################
def saf(robot,errorx, gate, had):
    print (errorx)
    w=p(errorx)
    s = 0
    print ("w   :   "+str(w))
    if abs(w)>=had:
        s=0
        ang= 0.8*w
        robot.setVelocity(0,ang)
        # time.sleep(0.005) 
    else :
        w = 0  
        s=1
    return s
#####################################################################

def doGateFunc(robot , foundColor ):
    teta = 80
    offset_pan=105
    robot.setCameraPos(offset_pan,teta)
    prefered_color = foundColor
    robot.setGripper(40)
    cnt = 0
    while (True):
        tick(120)
        print ("FPS               ******************               "+str(_tick2_fps))
        t1=time.time()

        ###___getting_frames_from_webcam___###
        frame, hsv = robot.getFrame(color = "hsv")

        ###___Rescale_the_frame___###
        resizedBGR, resizedhsv, scale = rescale_frame(frame, hsv, 50)

        ###___extract_the_needed_colors_mask___###
        red, yellow, blue= color(robot ,resizedBGR,resizedhsv)

        ###___get_the_details_of_each_color___###
        dict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=dict[prefered_color]


        b, d, x, y, area, l, errorx, errory, gate = find_gate(resizedBGR, scale, prefered_mask)
        cv2.imshow("Frame",resizedBGR)
        cv2.imshow(prefered_color,prefered_mask)

        k=cv2.waitKey(1) & 0xFF
        print ("While 1")
        print("errorx  &&&&&&&&&&&&&&&&&&&& $$$$$$$$$$$$$  "+str(errorx))
        s = 0
        if gate == 1:
            cnt = 5
        else:
            cnt -= 1
        
        if cnt > 0:
            s = saf(robot,errorx, gate, 0.04)
        else :
            robot.setVelocity(0,-0.6)

        t2=time.time()

        print ("Time 1      ===  "+str(t2-t1))
       
        if s == 1 and gate==1:
            robot.setVelocity(0 ,0)
            robot.setCameraPos(offset_pan,40)
            time.sleep(0.1)
            robot.getFrame()
            break

    
    time.sleep(0.1)
    robot.getFrame()
    tick(120)
    time.sleep(0.2)

    while (True):
        tick(120)
        print ("FPS               ******************               "+str(_tick2_fps))
        robot.setVelocity(0.08,0)
        # time.sleep(0.06)
        frame, hsv = robot.getFrame(color = "hsv")
        resizedBGR , resizedhsv, scale = rescale_frame(frame,hsv, 50)
        red, yellow, blue= color(robot , resizedBGR,resizedhsv)

        dict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=dict[prefered_color]


        k=cv2.waitKey(1) & 0xFF
        
        b, d, x, y, area, l, errorx, errory, gate = find_gate(resizedBGR, scale, prefered_mask)
        
        print("while 2   :::::::::::::::     "+ str(errory))
        print("errorx  &&&&&&&&&&&&&&&&&&&&   "+str(errorx))
        s = saf(robot,errorx, gate, 0.3)

        # if abs(errory) < 100:
        #     robot.setCameraPos(offset_pan,30)

        cv2.imshow("Frame",resizedBGR)
        cv2.imshow(prefered_color,prefered_mask)
     

        print ("Area   :   " + str (area) + "     gate :   " + str(gate))
        # if b+d>100 and area > 20000 and gate == 1 and s==1:
        if abs(errory)<5 and area > 20000 and gate == 1 and s==1:
            time.sleep(1.8)
            robot.setVelocity(0 ,0)
            time.sleep(0.2)
            ############ khodet ro saaf kon ;) ###################
            time_out = 2
            init_time = rospy.get_time();
            ####__ANGLE___####
            while (rospy.get_time() - init_time < time_out):
                imu = robot.getOdometry()
                phi = imu[2]
                if phi < 0:
                    z = 0.5
                else :
                    z = -0.5
                robot.setVelocity(0 ,z)
                time.sleep(0.01)
                if abs(phi) <0.1:
                    robot.setVelocity(0,0)
                    time.sleep(0.1)
                    break
            ###################
            robot.setVelocity(0 ,0)
            time.sleep(0.2)
            robot.setGripper(20,1)
            time.sleep(1)
            robot.setVelocity(0,0)
            time.sleep(0.1)
            print "finish"
            robot.setVelocity(0,0)
            time.sleep(1.5)
            robot.setVelocity(-0.05 ,0)
            time.sleep(4.5)
            robot.setVelocity(0,0)
            return 'released'


if __name__ == "__main__":
    robot = TurtleBot.TB3("sina",[0,1,2])
    robot.init_node()
    robot.printInfo()
    robot.loadParameter()
    doGateFunc(robot , "blue")
 
        

       


