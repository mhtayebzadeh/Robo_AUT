#!/usr/bin/env python
from AutmanRobot import TurtleBot
import rospy
import smach
import time
from AutmanRobot import TurtleBot

class GoToNextGoal(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['ok','fail'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state GoToNextGoal')

        ## just should change goal point
        
        
        return 'fail'