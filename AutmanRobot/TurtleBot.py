#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32 , Int32 , Empty , String
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_from_euler
import cv2
import time
import subprocess
import threading
import numpy as np
import numpy.linalg as la
from .Field import Field

class TB3:
    robot_namespace = ""
    cap = None
    webcam = "None"
    velocityPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    init_time = 0
    max_angularVel = 1
    max_linearVelocity = 0.22
    lidar_ranges = []
    odometery_pose_orientation_phi = 0
    odometery_pose_position_x = 0
    odometery_pose_position_y = 0
    odometery_twist_linear_x = 0
    odometery_twist_angular_z = 0
    
    resetPub = rospy.Publisher('reset', Empty, queue_size=2)
    cameraPosPanPub = rospy.Publisher('camera_pos_pan', Int32, queue_size=2)
    cameraPosTiltPub = rospy.Publisher('camera_pos_tilt', Int32, queue_size=2)
    gripperPub = rospy.Publisher('gripper_pos', Int32, queue_size=2)

    gripper = 0
    camera_pos_tilt = 0
    camera_pos_pan = 0
    field = None ## not used

    prefered_color = "yellow"
    fieldPoints = [ [1,1] , [1 ,0] , [2, -1]]; ## olaviat az ziad be kam
    nextPoint = [0,0]
    goalPoint = [2.7,0]
    find_ball_color = None
    find_ball_phi = None

    lower_blue_ball = np.array([87 , 137 , 99])
    upper_blue_ball = np.array([112 , 225 ,255])

    lower_red1_ball = np.array([160 , 145 , 0])
    upper_red1_ball = np.array([200 , 225 ,255])

    lower_red2_ball = np.array([0 , 51 , 0])
    upper_red2_ball = np.array([5 , 225 ,255])

    lower_yellow_ball = np.array([21 , 68 , 120])
    upper_yellow_ball = np.array([45 , 225 ,255])

    def __init__(self, name, cameraID , namespace = ""):
        print("robot init ...")
        self.robot_namespace = namespace
        self.name = name
        self.field = Field()
        self.setRosParams()
        if cameraID != None:
            if str(type(cameraID)) == "<type 'list'>":
                p = subprocess.Popen("ls /dev | grep video", stdout=subprocess.PIPE, shell=True)
                stdout, stderr = p.communicate()
                for i in cameraID:
                    if str(i) in stdout:
                        self.webcam = "/dev/video" + str(i)
                        print(self.webcam + " open ...")
                        self.cap = cv2.VideoCapture(int(i))
                        return
            else:
                self.cap = cv2.VideoCapture(cameraID)
        
    def __del__(self):
        try:
            for i in range(10):
                self.setVelocity(0,0)
            self.cap.release()
            print("camera released ...")
            cv2.destroyAllWindows()
        except:
            print "Exception"

    def findNextPoint(self):
        if len(self.fieldPoints) < 1:
            self.nextPoint = None
            return None
        else:
            self.nextPoint = self.fieldPoints[0]
            self.fieldPoints.remove(self.fieldPoints[0])
            return self.nextPoint
        
    def getNextPoint(self):
        return self.nextPoint

    def getRosTime(self):
        return rospy.get_time()

    def getRobotTime(self):
        return rospy.get_time() - self.init_time
    
    def setRobotTime(self,t = 0):
        self.init_time = rospy.get_time() - t;

    def resetRobot(self):
        self.resetPub = rospy.Publisher('reset', Empty, queue_size=10)
        for i in range(2):
            msg = Empty()
            self.resetPub.publish(msg)
            time.sleep(0.1)
        time.sleep(1.5)

    def getFrame(self , color = "BGR"):
        '''
            return (frame , 0) for color = "BGR" (default) 
            return (frame , gray) for color = "gray"
            return (frame , hsv) for color = "hsv"
        '''
        _,frame = self.cap.read()
        if color == "gray":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return frame , gray
        if color == "hsv":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            return frame , hsv
        return frame , 0

    def getObstacles(self , minRange = 0, maxRange = 3.5 , minAngle = 0 , maxAngle = 360):
        '''
        return abstacle list [degree , distance]
        '''
        obstacles = []
        if(maxAngle > 360):
            maxAngle = 360
        if(minAngle < 0):
            minAngle = 0
            
        if(minAngle < maxAngle):
            for i in range(minAngle , maxAngle):
                if self.lidar_ranges[i] > minRange and self.lidar_ranges[i] < maxRange:
                    obstacles.append([i,self.lidar_ranges[i]])
        else:
            a = range(minAngle , 360) + range(maxAngle)
            for i in a:
                if self.lidar_ranges[i] > minRange and self.lidar_ranges[i] < maxRange:
                    obstacles.append([i,self.lidar_ranges[i]])

        return obstacles
        
    def printInfo(self):
        print("")
        print("<<< == == == == ROBOT INFO == == == == >>>")
        print ("\trobot name = " + self.name)
        print ("\tCamera : " + self.webcam)
        print("<<< == == == == ********** == == == == >>>")
        print("")

    def init_node(self):
        rospy.init_node(self.name)
        print("node \'"+self.name+"\' started...")
        self.init_time = rospy.get_time()
        rosThread = threading.Thread(target=self.__init_ROS)
        rosThread.daemon = True
        rosThread.start()
        time.sleep(0.1)
        self.resetRobot()
        self.setVelocity(0,0)
        time.sleep(1.5)


    def __init_ROS(self):
        self.velocityPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cameraPosPanPub = rospy.Publisher(
            'camera_pos_pan', Int32, queue_size=10)
        self.cameraPosTiltPub = rospy.Publisher(
            'camera_pos_tilt', Int32, queue_size=10)
             
        self.monitorPub = rospy.Publisher(
            'monitor', String, queue_size=10)

        self.gripperPub = rospy.Publisher('gripper_pos', Int32, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.__odometry_callback)
        rospy.Subscriber("scan", LaserScan, self.__lidar_callback)
        rospy.Subscriber("imu", Imu, self.__IMU_callback)

        self.setCameraPos(105 , 70)
        self.setGripper(0)
        self.setOdometry(0,0,0)

        rospy.spin()

    def __odometry_callback(self,msg):
        q = msg.pose.pose.orientation
        e = euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.odometery_pose_orientation_phi = e[2]
        self.odometery_pose_position_x = msg.pose.pose.position.x
        self.odometery_pose_position_y = msg.pose.pose.position.y

        self.odometery_twist_linear_x = msg.twist.twist.linear.x
        self.odometery_twist_angular_z = msg.twist.twist.angular.z

    def __IMU_callback(self,msg):
        # TODO :
        pass

    def __lidar_callback(self,msg):
        self.lidar_ranges = msg.ranges


    def monitor(self , text):
        msg = String()
        msg.data = text
        self.monitorPub.publish(msg)
        
        
    def getIMU(self):
        # TODO: 
        pass

    def loadParameter():
        pass
        # TODO: 

    def getOdometry(self):
        '''
            return [x,y,phi(rad) , lin_vel , ang_vel]
        '''
        phi = self.odometery_pose_orientation_phi;
        x = self.odometery_pose_position_x
        y = self.odometery_pose_position_y
        lin_vel = self.odometery_twist_linear_x
        ang_vel = self.odometery_twist_angular_z
        return [x,y,phi, lin_vel , ang_vel]


    def getLidarRanges(self):
        return self.lidar_ranges

    def setOdometry(self,x,y,theta):
        pass
        #TODO: 

    def setCameraPos(self, pan , tilt):
        msg = Int32()
        msg.data = int(pan)
        self.camera_pos_pan = pan
        self.cameraPosPanPub.publish(msg)
        msg = Int32()
        msg.data = int(tilt)
        self.camera_pos_tilt = tilt
        self.cameraPosTiltPub.publish(msg)
        
    def getCameraPos():
        return self.camera_pos_pan , self.camera_pos_tilt

    def setGripper(self, status):
        '''
        status = 100 ==> Gripper completely close
        status = 0 ==> Gripper completely open
        '''
        msg = Int32()
        msg.data = int(status)
        self.gripper = status
        self.gripperPub.publish(msg)
        
    def vec_ang(self ,v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'    """
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def rad2deg(self ,ang):
        return ang*180/np.pi

    def deg2rad(self ,ang):
        return ang*np.pi/180


    def getGripperStatus():
        return self.gripper

    def setVelocity(self , lin , ang , _time = 0 , thread = False):
        if lin > self.max_linearVelocity:
            lin = self.max_linearVelocity
        if ang > self.max_angularVel:
            ang = self.max_angularVel
            
        if thread == True:
            myThread = threading.Thread(
                target=self.setVelocity, args=(lin, ang, _time, False))
            myThread.daemon = True
            myThread.start();
        else: 
            twist = Twist()
            twist.linear.x = lin; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = ang;
            self.velocityPub.publish(twist)
            if _time > 0:
                time.sleep(_time)
                twist = Twist()
                twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
                self.velocityPub.publish(twist)
            
    def getRosParam(self,param):
        return rospy.get_param(param)
    
    def setRosParams(self):
        rospy.set_param("Kw" , -1)
        rospy.set_param("Kv" , 0.15)
        rospy.set_param("Q_total_obstacles" , 0.12)
        rospy.set_param("Q_goal" , 5)




if __name__ == "__main__":
    print("start ...")
