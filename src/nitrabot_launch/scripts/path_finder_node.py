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

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 現在のディレクトリ情報
current_directory = os.path.dirname(os.path.abspath(__file__))

parent_directory = os.path.join(current_directory, "..")

data_directory = os.path.join(parent_directory, "data")



class PathFinder():
    PI = 3.14
    
    def __init__(self):
        """
        :param :
        :return : 
        """
        self.point = 0


    def create_goal(self, data, trajectory_index):
        """
        Create a Move Base goal for navigation 
        :param data: trajectory data contains (x, y, yaw)
        :param trajectory_index: index of a trajectory point 
        :return 
        """
        goal_send = MoveBaseGoal()
        
        goal_send.target_pose.header.frame_id = "map"#goal.header.frame_id

        goal_send.target_pose.header.stamp = rospy.Time.now()#goal.header.stamp

        goal_send.target_pose.pose.position.x = data[trajectory_index]["x"]        
        
        goal_send.target_pose.pose.position.y = data[trajectory_index]["y"]
        
        heading_in_quaternion = quaternion_from_euler(0.0, 0.0, data[trajectory_index ]["yaw"])

        goal_send.target_pose.pose.orientation.w = data[trajectory_index]["yaw"]
       
        rospy.loginfo(goal_send.target_pose.pose.position.x)
        
        rospy.loginfo(goal_send.target_pose.pose.position.y)

        return goal_send    
    
    def move(self):
        """
        
        :param : 
        :return:
        """ 
        trajectory_file = open(os.path.join(
            data_directory, "waypoint.json"
        ))
            
        data = json.load(trajectory_file)

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        client.wait_for_server()

        for i in range(0, len(data)):

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



if __name__ == '__main__':
    rospy.init_node("path_finder_node", anonymous = True)
    try:
        path_client = PathFinder()
        path_client.move()
        rospy.loginfo("Goal execution done!")
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")



