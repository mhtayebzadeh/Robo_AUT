import sys
sys.path.insert(0, '../..')

from AutmanRobot import TurtleBot
# from AutmanRobot.States.State_ObstaceAvoidance import ObstaceAvoidance

import cv2
import threading
import rospy
import smach
import numpy as np
import time
import numpy.linalg as la
import math
from operator import xor
import sys

######################____RESCALE_FRAME___###########################
def rescale_frame (frame ,hsv, scale):
    hsv = cv2.blur(hsv,(11,11))
    width = int(frame.shape[1] * scale/100)
    height = int(frame.shape[0] * scale/100)
    dim = (width, height)
    resizedBGR = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
    resizedHSV = cv2.resize(hsv, dim, interpolation =cv2.INTER_AREA)
    return resizedBGR , resizedHSV, scale
#####################################################################

########################################
def RGBY(frame):
    # split the image into its BGR components
    kernel = np.ones((5,5),np.float32)/40
    blur = cv2.blur(frame,(11,11))
    (B, G, R) = cv2.split(blur)
    Y= np.uint8(np.around(0.3*R + 0.55*G + 0.11*B))
    MAX=np.maximum(Y,B)
    MAX2=np.maximum(Y,G)
    # MAX3=np.maximum(R,Y)
    R[R < 1 * MAX] = 0
    G[G < 1 * MAX] = 0
    B[B < 1 * MAX2] = 0
   
    # merge the channels back together and return the image
    rgby = cv2.merge([B, G, R])

    return rgby
#########################################

########################################
def RGB(frame):
    # split the image into its BGR components
    kernel = np.ones((5,5),np.float32)/40
    blur = cv2.blur(frame,(11,11))
    (B, G, R) = cv2.split(blur)
    
    MAX=np.maximum(R,G)
    MAX2=np.maximum(MAX,B)
    # MAX3=np.maximum(R,Y)
    R[R < 1 * MAX2] = 0
    G[G < 1 * MAX2] = 0
    B[B < 1 * MAX2] = 0
   
    # merge the channels back together and return the image
    rgb = cv2.merge([B, G, R])

    return rgb
#########################################

########################################
def XYZ(frame):
    # split the image into its BGR components
    frame=cv2.cvtColor(frame, cv2.COLOR_BGR2XYZ)
    kernel = np.ones((5,5),np.float32)/40
    xyz = cv2.blur(frame,(11,11))
    # (B, G, R) = cv2.split(blur)
    
    # MAX=np.maximum(R,G)
    # MAX2=np.maximum(MAX,B)
    # # MAX3=np.maximum(R,Y)
    # R[R < 1 * MAX2] = 0
    # G[G < 1 * MAX2] = 0
    # B[B < 1 * MAX2] = 0
   
    # merge the channels back together and return the image
    # rgby = cv2.merge([B, G, R])

    return xyz
#########################################


##############___COLORS___##########################################

def color(robot , rgb,xyz,rgby,hsv):

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
    
    # mask_red = cv2.inRange(rgb, lower_red, upper_red)
    mask_red1 = cv2.inRange(hsv , lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv , lower_red2, upper_red2)
    # maskr = cv2.bitwise_not(mask_red)
    # maskrr = cv2.erode(maskr, None, iterations=2)
    # mask_red = cv2.bitwise_not(maskrr)
    mask_red = cv2.bitwise_or(mask_red2,mask_red1)

    # mask_yellow = cv2.inRange(rgby, lower_yellow, upper_yellow)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # mask_yellow = cv2.bitwise_and(mask_yellow, mask_yellow ,mask=maskw)
    # masky = cv2.bitwise_not(mask_yellow)
    # maskyy = cv2.erode(masky, None, iterations=2)
    # mask_yellow = cv2.bitwise_not(maskyy)

    # mask_blue = cv2.inRange(xyz, lower_blue, upper_blue)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # mask_green = cv2.inRange(frame, lower_green, upper_green)
    # mask_lblue = cv2.inRange(frame, lower_lblue, upper_lblue)

  


    return  mask_red, mask_yellow, mask_blue
#####################################################################

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
    offset_errorx=20
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
            if area >= 300 and area <= 24000:
                # c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
                (x, y), raduis= cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                raduis= int(raduis)
                cv2.circle(frame, center, raduis, (0, 0, 0), 4)
                # area = cv2.contourArea(c)
                # print(str(area))
                errorx= int((int(x)+offset_errorx-float(frame.shape[1] /2))*25/37.8)
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
    w=-1*float(errorx)/400
    return w
#####################################################################

#########################___SAF___##################################
def saf(robot ,errorx, ball, had):
    w=p(errorx)
    print('error:   '+ str(errorx) +'w:      '+ str(w))
    s = 0
    if abs(w)>=had:
        s=0
        ang= 1*w
        # time.sleep(0.01)
        robot.setVelocity(0,ang)
        # time.sleep(0.0001) 
    else :
        w = 0  
        s=1
        
    return s
#####################################################################

#########################____Charkh___###############################
def charkh(angle, phi):
    angle = angle + math.pi
    phi = phi + math.pi

    err = angle - phi
    print ("Angle   :::   "+ str(angle)+ "       Phi  ::    "+ str(phi)+"     Error    :::  " + str(err))
    
    if abs(2*math.pi - err) > math.pi:
        z = err
    else:
        z = -err

    if z > 0.5:
        z=0.5
    
    if z < -0.5:
        z=-0.5
    
    return z
    
#####################################################################
def doCatchFunc(robot,found_color_,angle,time_out=30):
    t5= time.time()
    robot.setGripper(100)
    robot.getFrame()
    found_color = found_color_ 
    init_time = rospy.get_time();

    ####__ANGLE___####
    while (rospy.get_time() - init_time < time_out):
        imu = robot.getOdometry()
        phi = imu[2]
        z= charkh (angle, phi)
        robot.setVelocity(0 ,z)
        time.sleep(0.001)
        if abs(z) <0.1:
            robot.setVelocity(0,0)
            break
    ###################
    

    teta = 70
    offset_pan=105
    robot.setCameraPos(offset_pan,teta)
    time.sleep(2)
    first_time =  0

    while (rospy.get_time() - init_time < time_out):
        t1=time.time()

        ###___getting_frames_from_webcam___###
        frame, hsv = robot.getFrame(color = "hsv")

        ###___Rescale_the_frame___###
        resizedBGR , resizedhsv, scale = rescale_frame(frame ,hsv, 50)
        

        ###___extract_the_needed_colors_mask___###
        rgb = RGB(resizedBGR)
        xyz=XYZ(resizedBGR)
        rgby = RGBY(resizedBGR)
        red, yellow, blue= color(robot , rgb,xyz,rgby,hsv)

        ###___get_the_details_of_each_color___###
        maskDict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=maskDict[found_color]
        x, y, errorx, errory, gate, ball, area , r = find_circ(resizedBGR, scale, prefered_mask)

        # print("while 1")


        s = saf(robot ,errorx, ball, 0.03)
        # print ("area :      "+str(vv))
        cv2.imshow("Frame",resizedhsv)
        cv2.imshow(found_color,prefered_mask)
        
        k=cv2.waitKey(1) & 0xFF
            
        if s == 1 and ball==1:
            s=saf(robot , errorx, ball, 0.01)
            robot.setVelocity(0 ,0)
            robot.setCameraPos(offset_pan,20)
            time.sleep(0.1)
            robot.getFrame()
            break

        t2=time.time()

    time.sleep(0.1)
    robot.getFrame()
    time.sleep(0.2)
    cnt = 0

    linVel = 0.05
    while (rospy.get_time() - init_time < time_out):
        robot.setVelocity(linVel,0)
        # time.sleep(0.001)

        ###___getting_frames_from_webcam___###
        frame, hsv = robot.getFrame(color = "hsv")

        ###___Rescale_the_frame___###
        resizedBGR , resizedhsv, scale = rescale_frame(frame,hsv, 50)
        
        ###___extract_the_needed_colors_mask___###
        rgb = RGB(resizedBGR)
        xyz=XYZ(resizedBGR)
        rgby = RGBY(resizedBGR)
        red, yellow, blue= color(robot , rgb,xyz,rgby)

        ###___get_the_details_of_each_color___###
        maskDict={"red":red, "yellow":yellow, "blue":blue}
        prefered_mask=maskDict[found_color]
        x, y, errorx, errory, gate, ball, area , radius = find_circ(resizedBGR, scale, prefered_mask)

        s = saf(robot , errorx, ball, 0.2)
        if y > 50 and ball==1:
            linVel = 0.03
            robot.setGripper(0)
            robot.setVelocity(linVel,0)
            print ("Seee Seee Seee")
            
        # print("while 2")
        s = saf(robot , errorx, ball, 0.06)
        
        cv2.imshow("Frame",resizedhsv)
        cv2.imshow(found_color,prefered_mask)
        k=cv2.waitKey(1) & 0xFF

        print("y = ",y)
        if ball == 1 and y >= 100 and s==1:
            cnt = cnt + 1
            if cnt > 4:
                robot.setVelocity(0.06,0)
                time.sleep(0.6)
                # robot.setGripper(20)
                # time.sleep(0.05)
                # robot.setGripper(30)
                # time.sleep(0.05)
                # robot.setGripper(40)
                # time.sleep(0.05)
                robot.setGripper(50)
                time.sleep(0.1)
                robot.setGripper(70)
                time.sleep(0.1)
                robot.setVelocity(0,0)
                time.sleep(0.5)
                robot.setVelocity(-0.05,0)
                time.sleep(2)
                robot.setVelocity(0,0)
                time.sleep(0.5)
                break
        else:
            cnt=0

    robot.setCameraPos(offset_pan,20)
    time.sleep(1)
    frame, hsv = robot.getFrame(color = "hsv")
    resizedBGR , resizedhsv, scale = rescale_frame(frame,hsv, 50)
    rgb = RGB(resizedBGR)
    xyz=XYZ(resizedBGR)
    rgby = RGBY(resizedBGR)
    red, yellow, blue= color(robot , rgb,xyz,rgby,hsv)

    maskDict={"red":red, "yellow":yellow, "blue":blue}
    prefered_mask=maskDict[found_color]
    x, y, errorx, errory, gate, ball, area , radius = find_circ(resizedBGR, scale, prefered_mask)
    s = saf(robot , errorx, ball, 0.06)
    if ball == 1 and y >= 150 and s == 1:
        t6 =time.time()
        print("YEEEEEESSSSSSSSS")
        return 'catched'
    else:
        robot.setCameraPos(offset_pan,50)
        robot.setVelocity(0,0)
        t6 =time.time()
        print("NNNOOOOOOOOOOOOO")
        return 'fail'
    
    

        

if __name__ == "__main__":
    robot = TurtleBot.TB3("sina",[0,1,2])
    robot.init_node()
    robot.printInfo()
    robot.setVelocity(0,0)
    # time.sleep(1)
    doCatchFunc(robot,"blue",0, time_out=30)

    
    
    






