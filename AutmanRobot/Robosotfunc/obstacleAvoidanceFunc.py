import sys
sys.path.insert(0, '../..')

from AutmanRobot import TurtleBot
# from AutmanRobot.States.State_ObstaceAvoidance import ObstaceAvoidance
import cv2
import threading
import rospy
import smach
from AutmanRobot.Field import Field
from turtlebot3_msgs.msg import Sound
import numpy as np
import time
import numpy.linalg as la
from std_msgs.msg import Float32 , Int32 , Empty

def obstacleAvoidance(robot , pos_d_ = [1,1],time_out = 45 , isGoal = False):
        
    pos_d = pos_d_
    dist_goal = 100
    robot.setRobotTime(0)
    time_ = 0
    pos_d = np.array(pos_d)
    init_time = rospy.get_time();
    while(dist_goal > 0.1 and (rospy.get_time() - init_time) < time_out ):
        Q_goal = robot.getRosParam("Q_goal")
        Q_tatal_obstacles = robot.getRosParam("Q_total_obstacles")
        a = robot.getOdometry()
        pos = np.array( [a[0] , a[1]] )
        phi = a[2]
        diff = pos_d - pos
        dist = la.norm(diff) + 0.0001
        dist_goal = float(dist)
        F_goal = (diff/dist) * (Q_goal/((dist - 0.1)**2))

        obstacles = robot.getObstacles(minRange = 0.1 , maxRange = 0.44 , minAngle = 300 , maxAngle = 60)
        # obstacles = robot.getObstacles(minRange = 0.1 , maxRange = 0.44 , minAngle = 275 , maxAngle = 85)

        N_obs = len(obstacles)
        if N_obs == 0:
            N_obs = 1

        F_obs = np.array([0.0,0.0])
        front_obstacles = 0

        for obs in obstacles:
            # coef = 1
            theta = obs[0]
            diff = np.array([obs[1] * np.cos(theta*np.pi/180 + phi) , obs[1] * np.sin(theta*np.pi/180 + phi)])
            dist = obs[1]
            if dist < 0.2 and (theta > 310  or theta < 40):
                front_obstacles += 1
            # if dist > .45 and dist <.66 :
            #     coef = 0.2
            F = - (diff/dist) * (Q_tatal_obstacles/((dist - 0.15)**2))
            F_obs += F

            # if (isGoal == False):
            #     ### check obstacle on the next point
            #     landa = theta*180/np.pi - phi;
            #     x_estimate_obs = pos[0] + dist*np.cos(landa)  
            #     y_estimate_obs = pos[1] - dist*np.sin(landa)

            #     diff_obs_2_desire = np.sqrt((pos_d[0] - x_estimate_obs)**2 + (pos_d[1] - y_estimate_obs)**2)
            #     if diff_obs_2_desire < 0.2:
            #         pos_d[0] = pos_d_[0] + 0.8*(np.random.random() - 0.5)
            #         pos_d[1] = pos_d_[1] + 0.8*(np.random.random() - 0.5)
            #         #TODO: 
            #     ### end of check obstacle

        F_obs = F_obs/N_obs
        print("pos_d = " + str(pos_d))
        rotation_theta = -phi
        R = np.array([[np.cos(rotation_theta) , -np.sin(rotation_theta)],[np.sin(rotation_theta) ,np.cos(rotation_theta)]])
        # F_obs = R.dot(F_obs)

        F_total = F_goal + F_obs


        # ang = -a[2] + np.arctan2(F_total[1] , F_total[0])
        robot_vec = np.array([np.cos(phi) , np.sin(phi)] )
        ang = robot.vec_ang(F_total , robot_vec)
        print("dist = " + str(dist_goal) + " ang = " + str(int(robot.rad2deg(ang))) + " , phi = " + str(int(phi*180/np.pi)) + " , obs phi = " + str(int(np.arctan2(F_obs[1] , F_obs[0])*180/np.pi)) )
        # print("F_total = " + str(F_total) + " , F_goal = " + str(F_goal) + " , F_obs = " + str(F_obs))
        a = np.append(F_total , 0)
        b = np.append(robot_vec , 0)

        sign = np.sign(np.cross(a,b))[2]
        theta_err = sign * ang
        
        w = robot.getRosParam("Kw") * theta_err
        v = 0.0
        if abs(theta_err) > 5:
            v = 0
        else: 
            v = robot.getRosParam("Kv") * (1 - abs(theta_err)/10)
        
        if front_obstacles > 1:
            v = 0.0

        
        robot.setVelocity(v , w)
        time.sleep(0.05)
        if dist_goal <= 0.1:
            return "arrived"

    if dist_goal <= 0.1:
            return "arrived"
    return "fail"

if __name__ == "__main__":
    robot = TurtleBot.TB3("tayeb",[0,1,2])
    robot.init_node()
    robot.printInfo()

    robot.setVelocity(0,0)    
    robot.setGripper(0)

    obstacleAvoidance(robot , [1,1])
    #obstacleAvoidance(robot , [1,0])
    # obstacleAvoidance(robot , [2,1])
    # obstacleAvoidance(robot , [2.7,0])
    # obstacleAvoidance(robot , [0,0])

    robot.setVelocity(0 , 0)