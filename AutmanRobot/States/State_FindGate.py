#!/usr/bin/env python
from AutmanRobot.Robosotfunc.gateFunc import *
import rospy
import smach
import time
from AutmanRobot import TurtleBot

class FindGate(smach.State):
    robot = None
    def __init__(self,robot):
        smach.State.__init__(self, outcomes=['released','fail'])
        self.robot = robot

    def execute(self, userdata):
        print('Executing state FindGate')
        self.robot.setVelocity(0,0)
        time.sleep(0.1)
        result = doGateFunc(self.robot ,self.robot.find_ball_color)
        if result == "released":
            return "released"
        return 'fail'