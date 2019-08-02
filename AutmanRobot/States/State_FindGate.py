#!/usr/bin/env python
import rospy
import smach
import time
from AutmanRobot import TurtleBot

class FindGate(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['ok','fail'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state FindGate')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)

        ##
        
        return 'fail'