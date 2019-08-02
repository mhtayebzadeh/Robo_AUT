from AutmanRobot import TurtleBot
from AutmanRobot.States.State_FindBall import FindBall
from AutmanRobot.States.State_FindGoal import FindGoal

from AutmanRobot.States.State_ObstaceAvoidance import ObstaceAvoidance
import cv2
import threading
import rospy
import smach
import time
from AutmanRobot.Field import Field
#

# for i in range(100):
#     f,_ = robot.getFrame("BGR")
#     robot.setVelocity(i*0.01,2);
#     cv2.imshow("ff" , f)
#     cv2.waitKey(50)
#     # print(robot.getLidarRanges())

# print(robot.getRobotTime())
#

def start_states(robot):
    # a = FindBall(robot)
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FindGoal', FindGoal(robot),
                               transitions={'arrived':'FindBall',
                                            'fail':'outcome4'})

        smach.StateMachine.add('FindBall', FindBall(robot),
                               transitions={'catched_ball':'ObstaceAvoidance',
                                            'no_ball_found':'FindBall'})

        smach.StateMachine.add('ObstaceAvoidance', ObstaceAvoidance(robot),
                               transitions={'arrived':'FindBall',
                                            'fail':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == "__main__":
    robot = TurtleBot.TB3("tayeb",[0,1,2])
    robot.init_node()
    robot.printInfo()
    time.sleep(2)
    print ("1")
    robot.setVelocity(0.1,0) 
    time.sleep(2)
    print ("2")

    # f = Field()

    # start_states(robot)
