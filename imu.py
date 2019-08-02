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

if __name__ == "__main__":
    robot = TurtleBot.TB3("tayeb",[0,1,2])
    robot.init_node()
    robot.printInfo()
    rotationVel = 0.5;
    init_t = time.time()
    while (True):
        robot.setVelocity(0,rotationVel)
        time.sleep(0.05)    
        imu = robot.getOdometry()
        phi = imu[2]
        print("phi = " , phi)
        if ((time.time() - init_t)>20):
            robot.setVelocity(0,0)
            time.sleep(0.1)
            break
