#!/usr/bin/env python
from AutmanRobot import TurtleBot
import rospy
import smach
import time

class FindNextPoint(smach.State):
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['noNextPoint','ok'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state FindNextPoint')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)


        nextPoint = self.robot.findNextPoint()
        if nextPoint == None:
            return 'noNextPoint'
        else:
            return 'ok'