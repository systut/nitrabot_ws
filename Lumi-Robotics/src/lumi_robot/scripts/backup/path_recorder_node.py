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
import json

# 外部ライブラリ
from tf.transformations import euler_from_quaternion
import rospy
from geometry_msgs.msg import Twist, PoseStamped

# 現在のディレクトリ情報
current_directory = os.path.dirname(os.path.abspath(__file__))

parent_directory = os.path.join(current_directory, "..")

data_directory = os.path.join(parent_directory, "data")


class PathNode:
    def __init__(self):
        self.velocity_subscriber = rospy.Subscriber('cmd_vel', Twist,
                                                    self.velocity_callback)

        self.pose_subscriber = rospy.Subscriber('tracked_pose', PoseStamped,
                                                self.pose_callback)

        self.pose = PoseStamped()

        self.previous_velocity = 0

    def pose_callback(self, pose: PoseStamped):
        """

        :param pose:
        :return:
        """
        self.pose = pose

    def velocity_callback(self, velocity: Twist):
        """

        :param velocity:
        :return:
        """
        angular_velocity = velocity.angular.z

        if angular_velocity != 0 and self.previous_velocity != 0:

            point = {
                "x": self.pose.pose.position.x,
                "y": self.pose.pose.position.y,
                "yaw": euler_from_quaternion([
                    self.pose.pose.orientation.x,
                    self.pose.pose.orientation.y,
                    self.pose.pose.orientation.z,
                    self.pose.pose.orientation.w,
                ])[2]
            }

            self.add_waypoint(point, file_path=os.path.join(
                data_directory, "waypoint.json"
            ))

        self.previous_velocity = angular_velocity

    @staticmethod
    def add_waypoint(new_point: dict, file_path):
        """

        :param new_point:
        :param file_path:
        :return:
        """

        try:
            file = open(file_path, "r+")

        except FileNotFoundError as exception:
            file = open(file_path, "w")
            points = [new_point]
            json.dump(points, file, indent=4)
            file.close()

        else:
            points = json.load(file)

            points.append(new_point)
            file.seek(0)
            json.dump(points, file, indent=4)
            file.truncate()
            file.close()


if __name__ == '__main__':
    rospy.init_node('path_recorder_node', anonymous=True)

    path_node = PathNode()

    try:
        rospy.spin()
        rospy.loginfo("Finish recording path!")

    except KeyboardInterrupt:
        print("Shutting down")
