#!/usr/bin/env python
from AutmanRobot.Robosotfunc.obstacleAvoidanceFunc import *
from AutmanRobot import TurtleBot
import rospy
import smach
import time

class ObstaceAvoidance(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['arrived','fail'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state ObstaceAvoidance')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)

        result = obstacleAvoidance(self.robot , self.robot.nextPoint , time_out = 45)
        if result == "arrived" :
            return "arrived"

        return 'fail'