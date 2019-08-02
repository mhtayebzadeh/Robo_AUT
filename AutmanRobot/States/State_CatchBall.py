#!/usr/bin/env python
from AutmanRobot.Robosotfunc.catchFunc import *
import rospy
import smach
import time
from AutmanRobot import TurtleBot




class CatchBall(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['catched','fail'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state CatchBall')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)
        result = doCatchFunc(self.robot ,self.robot.find_ball_color, self.robot.find_ball_phi , time_out = 30)
        if result == 'catched':
            return 'catched'
        return 'fail'