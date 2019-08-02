#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time

counter = 0

def monitor_Callback(msg):
    global counter
    counter += 1
    text = msg.data
    print("")
    print("<<==== Msg " + str(counter) + " ====>>")
    print(text)
    print("")

if __name__=="__main__":
    rospy.init_node("monitor")
    rospy.Subscriber("monitor" , String , monitor_Callback)

    rospy.spin()
