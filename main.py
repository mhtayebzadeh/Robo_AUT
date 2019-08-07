from AutmanRobot import TurtleBot
from AutmanRobot.States.State_ObstaceAvoidance import ObstaceAvoidance
from AutmanRobot.States.State_FindNextPoint import FindNextPoint
from AutmanRobot.States.State_ScanBall import ScanBall
from AutmanRobot.States.State_CatchBall import CatchBall
from AutmanRobot.States.State_GoToGoal import GoToGoal
from AutmanRobot.States.State_FindGate import FindGate
from AutmanRobot.States.State_GoToNextGoal import GoToNextGoal

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

def start_states(robot):

    sm = smach.StateMachine(outcomes=['outcome4', 'END'])
    # Open the container
    with sm:
        # Add states to the container


        smach.StateMachine.add('FindNextPoint', FindNextPoint(robot),
                               transitions={'ok':'ObstaceAvoidance',
                                            'noNextPoint':'END'})

        smach.StateMachine.add('ObstaceAvoidance', ObstaceAvoidance(robot),
                               transitions={'arrived':'ScanBall',
                                            'fail':'FindNextPoint'})

        smach.StateMachine.add('ScanBall', ScanBall(robot),
                               transitions={'found':'CatchBall',
                                            'notFound':'FindNextPoint'})


        smach.StateMachine.add('CatchBall', CatchBall(robot),
                               transitions={'catched':'GoToGoal',
                                            'fail':'ScanBall'})



        smach.StateMachine.add('GoToGoal', GoToGoal(robot),
                                transitions={'arrived':'FindGate',
                                            'fail':'GoToNextGoal'})

        smach.StateMachine.add('FindGate', FindGate(robot),
                                transitions={'released':'FindNextPoint',
                                            'fail':'GoToNextGoal'})

        smach.StateMachine.add('GoToNextGoal', GoToNextGoal(robot),
                                transitions={'ok':'GoToGoal',
                                            'fail':'GoToNextGoal'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == "__main__":
    robot = TurtleBot.TB3("tayeb",[0,1,2])
    robot.init_node()
    robot.printInfo()
    robot.loadParameter()
    print(robot.lower_yellow_ball)
    time.sleep(0.5)
    # robot.resetRobot()
    # time.sleep(5)
    robot.setVelocity(0,0)    
    robot.setGripper(0)
    time.sleep(1)
    start_states(robot)
    robot.setVelocity(0,0) 
    # del robot