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

# ##############___COLORS___##########################################

def color(hsv):
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


    lower_red = np.array([0,0,147])
    upper_red = np.array([30,225,255])
    # #######################################

    # ################___YELLOW___###########
    lower_yellow = np.array([20,0,0])
    upper_yellow = np.array([40,255,255])
    # #######################################

    # ################___BLUE___#############
    lower_blue = np.array([83,0,255])
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

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)




    return  mask_red, mask_yellow, mask_blue
# #####################################################################



######################___BLUR_COLOR___###########################################################
# def color(hsv):
#     hsv[...,2]=hsv[...,2]*0.4
#     hsv=cv2.medianBlur(hsv,19)
    
#     #################___RED__##############
#     lower_red = np.array([0,73,0])
#     upper_red = np.array([76,225,255])
#     #######################################

#     ################___YELLOW___###########
#     lower_yellow = np.array([0,0,0])
#     upper_yellow = np.array([61,63,255])
#     #######################################

#     ################___BLUE___#############
#     lower_blue = np.array([88,0,75])
#     upper_blue = np.array([255,255,255])
#     #######################################

#     ##########___MASK___##########
    
#     mask_r = cv2.inRange(hsv, lower_red, upper_red)
#     maskr = cv2.bitwise_not(mask_r)
#     maskrr = cv2.erode(maskr, None, iterations=3)
#     mask_red = cv2.bitwise_not(maskrr)
    

#     mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)
#     masky = cv2.bitwise_not(mask_y)
#     maskyy = cv2.erode(masky, None, iterations=3)
#     mask_yellow = cv2.bitwise_not(maskyy)

#     mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
#     maskb = cv2.bitwise_not(mask_b)
#     maskbb = cv2.erode(maskb, None, iterations=3)
#     mask_blue = cv2.bitwise_not(maskbb)
    

#     return  mask_red, mask_yellow, mask_blue
###############################################################################################
#########################___P_CONTROLLER___##########################
def p(errorx):
    w=-1*float(errorx)*10/200/20
    return w
#####################################################################


#########################___SAF___##################################
def saf(errorx, had):
    w=p(errorx)
    s = 1
    if abs(w)>=had:
        s=1
        ang=1*w
        robot.setVelocity(0,ang)
        time.sleep(0.008) 
    else :
        w = 0  
        s=0 
    print "ksrjghjrkglzkrjdfbnlzkjtd,dtjkcv"    
    print w       
    return s
#####################################################################
# def camera():
#     x=20
#     step=1
#     prefered_color="red"
#     gate=0
#     while(True):
#         robot.setCameraPos(x,70)
#         time.sleep(0.05)
#         x+=step
#         if x==160:
#             step=-1
#         if x==20:
#             step=1            

#         frame, hsv = robot.getFrame(color = "hsv")

#         ###___Rescale_the_frame___###
#         resizedBGR , resizedhsv, scale = rescale_frame(frame ,hsv, 50)
        
#         red, yellow, blue= color(resizedhsv)

#         ###___get_the_details_of_each_color___###
#         dict={"red":red, "yellow":yellow, "blue":blue}
#         prefered_mask=dict[prefered_color]
#         x, y, errorx, errory, gate, area = find_gate(resizedhsv, scale, prefered_mask)
#         # cv2.imshow("resizedhsv",resizedhsv)
#         # cv2.imshow(prefered_color,prefered_mask)
#         # cv2.imshow("resizedhsv",resizedhsv)
#         cv2.imshow(prefered_color,prefered_mask) 
#         if gate==1:
#             break


                  


######################################################################

######################____GATE_FIND___###############################
def find_gate():
    x = q00
    y = 100
    errorx = 0
    errory = 0
    gate = 0
    area = 0
    for s in range(10):
        frame, hsv = robot.getFrame(color = "hsv")

    ###___Rescale_the_frame___###
    resizedBGR , resizedhsv, scale = rescale_frame(frame ,hsv, 50)

    red, yellow, blue= color(resizedhsv)
    prefered_color="red"
    ###___get_the_details_of_each_color___###
    dict={"red":red, "yellow":yellow, "blue":blue}
    prefered_mask=dict[prefered_color]
    if gate==1:
        print "found gate"
    #cv2.imshow("resizedhsv",resizedhsv)
    # cv2.imshow(prefered_color,prefered_mask)
    # cv2.imshow(prefered_color,prefered_mask)
    k=cv2.waitKey(1) & 0xFF     
    frame=resizedhsv
    cc=prefered_mask
    _, cnts, _ = cv2.findContours(cc, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts)==0:
        return 0,0,0,0,0,0
    else:
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        _cnts=cnts[0]
        arr = []
        res = []
        M = cv2.moments(_cnts)
        cX = int(((M["m10"] + 0.00001 )/( M["m00"] + 0.00001)) / scale)
        cY = int(((M["m01"] + 0.00001 )/( M["m00"] + 0.00001)) / scale)
        # c = c.astype("float")
        _cnts = _cnts.astype("int")
        appr = cv2.approxPolyDP(_cnts, 0.05 * cv2.arcLength(_cnts, True),True)
        if len(appr) > 2 and len(appr)<=8:
            area = cv2.contourArea(_cnts)
            print("area = ",area)
            if area >= 5000 and area <= 75000:
                # c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
                x,y,w,h = cv2.boundingRect(_cnts)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
                center = (int(x+(w/2)), int(y+(h/2)))
                x = center[0];
                y = center[1];
                # area = cv2.contourArea(c)
                cv2.drawContours(frame, [appr], 0, (0, 255, 0), 4)
                print("center = " , center)
                errorx= int((int(x)-25-float(frame.shape[1] /2))*25/37.8)
                errory= int((int(y)-float(frame.shape[0] /2))*25/37.8)
                arr.append(errorx)
                gate = 1
            else:
                print "too small area"   
        cv2.imshow('resizedhsv',frame)
 
        return x,y,errorx,errory,gate,area

#####################################################################



if __name__ == "__main__":
    robot = TurtleBot.TB3("sina",[0,1,2])
    robot.init_node()
    robot.printInfo()

    # teta = 70
    # robot.setCameraPos(90,teta)
    prefered_color="red"
    first_time =  0
    find_gatee=0
    while (True):
        robot.setCameraPos(75,70)
        for h in range(10):
            x, y, errorx, errory, gate, area = find_gate()
            if gate==1:
                find_gatee=1
                break 
        if find_gatee==1:
            break       
        robot.setCameraPos(105,70)
        for h in range(10):
            x, y, errorx, errory, gate, area = find_gate()
            if gate==1:
                find_gatee=1
                break  
        if find_gatee==1:
            break                       
        robot.setCameraPos(135,70)
        for h in range(10):
            x, y, errorx, errory, gate, area = find_gate()
            if gate==1:
                find_gatee=1
                break
        if find_gatee==1:
            break                 
    print "gate founded"
    time.sleep(3)
    print "errorx=%d , errory=%d"% (errorx, errory)
    while(True):
        x,y,errorx,errory,gate,area=find_gate()
        s = saf(errorx, 0.05)
        if s==0:
            break
    robot.setVelocity(0,0)     
    robot.setCameraPos(105,70)
    time.sleep(1)
    while(True):
        x,y,errorx,errory,gate,area=find_gate()
        s = saf(errorx, 0.05)
        if s==0:
            break
    print "saf"        
    robot.setVelocity(0.1,0)
    robot.setCameraPos(105,10)
    time.sleep(1)   
    while(True):        
        x,y,errorx,errory,gate,area=find_gate()
        if y<100:
            break
    robot.setVelocity(0,0)        
    print "residimo residim kashki nemiresidim"        
       

    while(True):
        pass        
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
        x, y, errorx, errory, gate, area = find_gate(resizedhsv, scale, prefered_mask)
        print 'x=%d y=%d errorx=%d errory=%d' % (x, y, errorx, errory)
        camera()

        # s = saf(errorx, ball, 0.03)
        # print ("area :      "+str(vv))
        # cv2.imshow("resizedhsv",resizedhsv)
        # cv2.imshow(prefered_color,prefered_mask)
        
        k=cv2.waitKey(1) & 0xFF

         
        # if s == 0:
        #     robot.setVelocity(0 ,0)
        #     break

        t2=time.time()

    
    # robot.setCameraPos(90,35)
    # time.sleep(1)
    # robot.getFrame()
    # cnt = 0
    # while (True):
    #     robot.setVelocity(0.05,0)
    #     time.sleep(0.08)

    #     ###_        s = saf(errorx, ball, 0.05)
###__getting_frames_from_webcam___###
    #     frame, hsv = robot.getFrame(color = "hsv")

    #     ###___Rescale_the_frame___###
    #     resizedBGR , resizedhsv, scale = rescale_frame(frame,hsv, 50)

    #     ###___extract_the_needed_colors_mask___###
    #     red, yellow, blue= color(resizedhsv)

    #     ###___get_the_details_of_each_color___###
    #     dict={"red":red, "yellow":yellow, "blue":blue}
    #     prefered_mask=dict[prefered_color]
    #     x, y, errorx, errory, gate, ball, area , radius = find_circ(resizedhsv, scale, prefered_mask)

    #     print("while 2")
    #     s = saf(errorx, ball, 0.05)
        
    #     cv2.imshow(prefered_color,prefered_mask)
    #     k=cv2.waitKey(1) & 0xFF

    #     print("y = ",y)
    #     if ball == 1 and y >= 180 and s==0:
    #         cnt = cnt + 1
    #         if cnt > 4:
    #             break
        

    
    robot.setVelocity(0.05,0)
    time.sleep(0.5)
    robot.setVelocity(0,0)
    robot.setGripper(20)
    time.sleep(0.05)
    robot.setGripper(30)
    time.sleep(0.05)
    robot.setGripper(40)
    time.sleep(0.05)
    robot.setGripper(50)
    time.sleep(0.05)
    robot.setGripper(55)
    






