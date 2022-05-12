#!/usr/bin/env python2

import time
import rospy
from geometry_msgs.msg import Twist
import pandas as pd
from nav_msgs.msg import Odometry

class TestFeedForwardControl:
    def __init__(self):
        self.velocity_publisher_ = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.count = 0

    def velCallback(self,odom_data):
        current_time = odom_data.header.stamp
        pose = odom_data.pose.pose
        #TO BE SOLVED Count prob
        #self.count += 1
        # rospy.loginfo(self.count)

    def execute(self, a):
        ref_ = pd.read_csv('/home/syseng402/Straight.csv')
        velocity = ref_.values.tolist()
        vel_msg_ = Twist()
        if  self.count == 501 :
            vel_msg_.linear.x = 0
            vel_msg_.angular.z = 0
            self.velocity_publisher_.publish(vel_msg_)
        else:
            vel_msg_.linear.x = velocity[self.count][0]
            vel_msg_.angular.z = velocity[self.count][1]
            self.velocity_publisher_.publish(vel_msg_)

            self.count += 1

if __name__ == '__main__':

    rospy.init_node('control_node', anonymous = True)
   
    controller = TestFeedForwardControl()
    
    rospy.Timer(rospy.Duration(0.05), controller.execute)
    
    rospy.Subscriber('odom', Odometry, controller.velCallback)

    rospy.spin()
