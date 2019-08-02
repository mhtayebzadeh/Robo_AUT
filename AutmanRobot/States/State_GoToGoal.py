#!/usr/bin/env python
from AutmanRobot.Robosotfunc.obstacleAvoidanceFunc import *
from AutmanRobot import TurtleBot
import rospy
import smach
import time
from AutmanRobot import TurtleBot

class GoToGoal(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['arrived','fail'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state GoToGoal')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)

        result = obstacleAvoidance(self.robot , self.robot.goalPoint , 40)
        if result == "arrived" :
            angle = 0
            time_out = 7
            init_time = rospy.get_time();
            ####__ANGLE___####
            while (rospy.get_time() - init_time < time_out):
                imu = self.robot.getOdometry()
                phi = imu[2]
                if phi < 0:
                    z = 0.5
                else :
                    z = -0.5
                self.robot.setVelocity(0 ,z)
                time.sleep(0.001)
                if abs(z) <0.2:
                    self.robot.setVelocity(0,0)
                    break
            ###################
            self.robot.setVelocity(0,0)
            return "arrived"
        return 'fail'