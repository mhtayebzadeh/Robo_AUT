#!/usr/bin/env python
from AutmanRobot.Robosotfunc.scanFunc import *
from AutmanRobot import TurtleBot
import rospy
import smach
import time

class ScanBall(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['found','notFound'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state ScanBall')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)

        result = doScan(self.robot , self.robot.prefered_color , 25)
        time.sleep(5)
        if result == "found":
            return "found"
        else:
            return 'notFound'