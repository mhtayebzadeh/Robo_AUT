import numpy as np 
import cv2
import pickle



def nothing(x):
    pass

cap = cv2.VideoCapture(1)

# Create a black image, a window
img = np.zeros((480,640,3), np.uint8)
#cv2.namedWindow('image')

while(1):
    # Take each frame
    _, frame = cap.read()
    
    hmin=0
    hmax=0
    smin=0
    smax=0
    vmin=0
    vmax=0
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    with open ('red.pickle','rb') as tuned_param:
        red=[hmin,hmax,smin,smax,vmin,vmax]
        red=pickle.load(tuned_param)
        #print('num',num)
        [hmin,hmax,smin,smax,vmin,vmax]=red
    with open ('blue.pickle','rb') as tuned_param:
        blue=[hmin,hmax,smin,smax,vmin,vmax]
        blue=pickle.load(tuned_param)
        [hmin,hmax,smin,smax,vmin,vmax]=blue
    with open ('yellow.pickle','rb') as tuned_param:
        yellow=[hmin,hmax,smin,smax,vmin,vmax]
        yellow=pickle.load(tuned_param)
        [hmin,hmax,smin,smax,vmin,vmax]=yellow
    # define range of blue color in HSV
    lower_blue = np.array([hmin, smin, vmin])
    upper_blue = np.array([hmax, smax, vmax])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    cv2.imshow('mask',mask)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break


cv2.destroyAllWindows()
cap.release()
