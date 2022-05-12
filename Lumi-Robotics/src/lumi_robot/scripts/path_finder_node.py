#!/usr/bin/env python3
# coding: utf-8
"""

Abstract::
    -
History::
    - Ver.      Date            Author        History
    - 1.0
"""
# 標準ライブラリ
import os
import time

import rospy
import json
import geometry_msgs.msg as geometry_msgs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped

from tf.transformations import euler_from_quaternion

# 現在のディレクトリ情報
current_directory = os.path.dirname(os.path.abspath(__file__))

parent_directory = os.path.join(current_directory, "..")

data_directory = os.path.join(parent_directory, "data")



class PathFinder():
    PI = 3.14
    FORWARD = 1
    BACKWARD = 2
    END_POINT = 3
    START_POINT = 4
    
    def __init__(self):
        self.point = 0
        self.pose_subscriber = rospy.Subscriber('tracked_pose', PoseStamped,
                                                self.pose_callback)
        self.current_point = {}
        self.pose = None

    def pose_callback(self, pose):
        self.pose = pose

        self.current_point = {
            "x": self.pose.pose.position.x,
            "y": self.pose.pose.position.y,
            "yaw": euler_from_quaternion([
                self.pose.pose.orientation.x,
                self.pose.pose.orientation.y,
                self.pose.pose.orientation.z,
                self.pose.pose.orientation.w,
            ])[2]
        }

    def create_goal(self, data, i):
        goal_send = MoveBaseGoal()
        goal_send.target_pose.header.frame_id = "map"#goal.header.frame_id
        goal_send.target_pose.header.stamp = rospy.Time.now()#goal.header.stamp
        goal_send.target_pose.pose.position.x = data[i]["x"]#goal.pose.position.x
        goal_send.target_pose.pose.position.y = data[i]["y"]#goal.pose.position.y
        goal_send.target_pose.pose.orientation.w = data[i]["yaw"]#goal.pose.orientation.z
        rospy.loginfo(goal_send.target_pose.pose.position.x)
        rospy.loginfo(goal_send.target_pose.pose.position.y)
        return goal_send    
    
    def move(self, direction):
        if direction == PathFinder.FORWARD:
            f = open(os.path.join(
                        data_directory, "forward_point.json"
                    ))
        elif direction == PathFinder.BACKWARD:
            f = open(os.path.join(
                data_directory, "backward_point.json"
            ))
            
        data = json.load(f)

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        for i in range(0, len(data)):
            print("point %s", i)

            goal_send = self.create_goal(data, i)
            rospy.loginfo("Send goal")
            client.send_goal(goal_send)
            wait = client.wait_for_result()
            rospy.loginfo("After waiting for result")
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("SHUT")
            else:
                result = client.get_result()
                rospy.loginfo(result)

    def rotate(self, target_angle):
        velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        speed = 50
        # Converting from angles to radians
        angular_speed = speed*2*PathFinder.PI/360

        # We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
  
        vel_msg.angular.z = -abs(angular_speed)
        # Setting the current time for distance calculus

        pre_angle = self.current_point["yaw"] % PathFinder.PI
        count = 0
        while True:
            current_angle = self.current_point["yaw"] % PathFinder.PI
            if abs(current_angle - pre_angle) > 2.5:
                vel_msg.angular.z = abs(angular_speed)
                velocity_publisher.publish(vel_msg)
                time.sleep(0.5)
                break
            count += 1
            if count > 40:
                break
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            print("current_angle = {}".format(current_angle))
            print("target_angle = {}".format(target_angle))
            pre_angle = current_angle
            time.sleep(0.5)
        #
        for _ in range(4):
            vel_msg.angular.z = 0
            vel_msg.linear.x = -0.2
            velocity_publisher.publish(vel_msg)
            time.sleep(0.5)

        vel_msg.angular.z = 0
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)
        time.sleep(3)


if __name__ == '__main__':
    rospy.init_node("path_finder_node", anonymous = True)
    try:
        path_client = PathFinder()
        path_client.move(PathFinder.FORWARD)
        path_client.rotate(0)
        path_client.move(PathFinder.BACKWARD)
        path_client.rotate(0)
        # path_client.move(PathFinder.RETURN_START_POINT)
        rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



