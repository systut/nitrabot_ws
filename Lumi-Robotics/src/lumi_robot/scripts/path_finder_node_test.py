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

import rospy
import json
import geometry_msgs.msg as geometry_msgs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 現在のディレクトリ情報
current_directory = os.path.dirname(os.path.abspath(__file__))

parent_directory = os.path.join(current_directory, "..")

data_directory = os.path.join(parent_directory, "data")

class PathFinder():
    def __init__(self):
        self.point = 0

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
    
    def move(self):
        f = open(os.path.join(
                data_directory, "waypoint.json"
            ))
        data = json.load(f)
	
	global roll, pitch, yaw
		
	
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        for i in range(0,len(data)):
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
            i += 1
       #Rotate at last point    
        rotate_point = create_goal(data, len(data)) 
        rotate_point_orientation = rotate_point.target_pose.pose.orientation
  	orientation_list = [rotate_point_orientation.x, rotate_point_orientation.y, rotate_point_orientation.z, rotate_point_orientation.w]
  	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
  	yaw = yaw + 3.1415
  	rotate_point_quat = quaternion_from_euler (roll, pitch, yaw)
	client.send_goal(rotate_point_quat)
  	wait = client.wait_for_result()
            rospy.loginfo("After waiting for result")
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("SHUT")
            else:
                result = client.get_result()
                rospy.loginfo(result)
                

if __name__ == '__main__':
    rospy.init_node("path_finder_node", anonymous = True)
    try:
        path_client = PathFinder()
        path_client.move()
        rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

