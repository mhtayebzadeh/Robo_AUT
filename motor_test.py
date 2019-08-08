
from AutmanRobot import TurtleBot
import time

if __name__ == "__main__":
    robot = TurtleBot.TB3("sina",[0,1,2])
    robot.init_node()
    robot.printInfo()
    robot.setVelocity(0.15,0)
    time.sleep(1)
    robot.setVelocity(0,0)
    time.sleep(1)
    robot.setVelocity(-0.15,0)
    time.sleep(1)
    robot.setVelocity(0,1)
    time.sleep(2)
    robot.setVelocity(0,0)
    time.sleep(1)
    robot.setGripper(0 , 2)
    time.sleep(1)
    robot.setGripper(40 , 2)
    time.sleep(1)
    robot.setCameraPos(0, 20)
    time.sleep(1)
    robot.setCameraPos(0, 150)
    time.sleep(1)
    


       


