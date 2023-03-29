#!usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import geonav_transform.geonav_conversions as gc


class gps_transform_node():

    def __init__(self) -> None:
        
        rospy.init_node("gps_transform_node")

        self.gps_subscriber = rospy.Subscriber("/fix", NavSatFix, self._gps_callback)

        self.odom_publisher = rospy.Publisher("/gps/odom", Odometry, queue_size=1)

        self.origiin_lat = rospy.get_param("/original_latitude")

        self.origiin_lon = rospy.get_param("/original_longitude")

        self.gps_data = NavSatFix()

        self.gps_odom = Odometry()

        self._initial_positions = []

        self.lat = 0

        self.lon = 0

    def _get_x_y(self):
        x, y = gc.ll2xy(self.lat, self.lon, self.origiin_lat, self.origiin_lon)

        return x, y

    def _gps_callback(self, msg: NavSatFix) -> None:
        
        self.gps_data = msg

        self.lat = self.gps_data.latitude

        self.lon = self.gps_data.longitude

        current_x, current_y = self._get_x_y()

        rospy.loginfo("Current pose ({}, {})".format(current_x, current_y))

        self._publish_gps_odom(current_x, current_y)

        # self._initial_positions.append(msg)

    def _publish_gps_odom(self, x ,y):
        
        self.gps_odom = Odometry()

        self.gps_odom.header.frame_id = "gps_odometry"

        self.gps_odom.pose.pose.position.x = x

        self.gps_odom.pose.pose.position.y = y

        self.odom_publisher.publish(self.gps_odom)

    def _execute(self):

        rospy.spin()