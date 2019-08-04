import cv2
import numpy as np
import pickle
import copy






def nothing(x):
    pass
cap = cv2.VideoCapture(0)
# Create a black image, a window
img = np.zeros((480,640,3), np.uint8)
cv2.namedWindow('image') 
# create trackbars for color change
cv2.createTrackbar('Hmin','image',0,255,nothing)
cv2.createTrackbar('Hmax','image',255,255,nothing)
cv2.createTrackbar('Smin','image',0,255,nothing)
cv2.createTrackbar('Smax','image',255,255,nothing)
cv2.createTrackbar('Vmin','image',0,255,nothing)
cv2.createTrackbar('Vmax','image',255,255,nothing)

# create switch for ON/OFF functionality
BLUEswitch = ' Blue 0 : OFF \n1 : ON'
REDswitch =  ' Red 0 : OFF \n1 : ON'
YELLOWswitch =  'Yellow 0 : OFF \n1 : ON'
cv2.createTrackbar(BLUEswitch, 'image',0,1,nothing)
cv2.createTrackbar(REDswitch, 'image',0,1,nothing)
cv2.createTrackbar(YELLOWswitch, 'image',0,1,nothing)

while(1):
    # get current positions of four trackbars
    hmin = cv2.getTrackbarPos('Hmin','image')
    hmax = cv2.getTrackbarPos('Hmax','image')
    smin = cv2.getTrackbarPos('Smin','image')
    smax = cv2.getTrackbarPos('Smax','image')
    vmin = cv2.getTrackbarPos('Vmin','image')
    vmax = cv2.getTrackbarPos('Vmax','image')
    BLUE = cv2.getTrackbarPos(BLUEswitch,'image')
    RED = cv2.getTrackbarPos(REDswitch,'image')
    YELLOW = cv2.getTrackbarPos(YELLOWswitch,'image')
    if BLUE==0 and RED==0 and YELLOW==0 :
        img[:] =0
    elif BLUE==1: 
        RED=0
        YELLOW=0
        #img[:]=[hmin,hmax,smin,smax,vmin,vmax]
        with open ('./AutmanRobot/color/blue.pickle','wb') as tuned_param:
            num=[hmin,hmax,smin,smax,vmin,vmax]
            pickle.dump(num,tuned_param)
    elif RED ==1:
        cv2.createTrackbar(BLUEswitch, 'image',0,0,nothing)
        cv2.createTrackbar(YELLOWswitch, 'image',0,0,nothing)
        with open ('./AutmanRobot/color/red.pickle','wb') as tuned_param:
            num=[hmin,hmax,smin,smax,vmin,vmax]
            pickle.dump(num,tuned_param)
    elif YELLOW ==1:
        BLUE=0
        RED=0
        with open ('./AutmanRobot/color/yellow.pickle','wb') as tuned_param:
            num=[hmin,hmax,smin,smax,vmin,vmax]
            pickle.dump(num,tuned_param)
       # Take each frame
    _, frame = cap.read()
    img = frame.copy()
    # Convert BGR to HSV
    scale=50
    width = int(frame.shape[1] * scale/100)
    height = int(frame.shape[0] * scale/100)
    dim = (width, height)
    resizedBGR = cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
    hsv = cv2.cvtColor(resizedBGR, cv2.COLOR_BGR2HSV)
    ##########################################3
    lower_range = np.array([hmin, smin, vmin])
    upper_range = np.array([hmax, smax, vmax])
    mask = cv2.inRange(hsv, lower_range, upper_range)
    
    cv2.waitKey(1)
    cv2.imshow("image",img)
    cv2.imshow("mask",mask)

cap.release()
cv2destroyAllWindows()    