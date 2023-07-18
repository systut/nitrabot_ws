#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
#
# Distributed under terms of the MIT license.

# Standard library
import json

# External library
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from rospy_message_converter import json_message_converter

# Internal library
from nitrabot_remote.websocket_client import WebsocketClient


class NitrabotRemoteNode(object):
    """!
    @brief This class privide a ROS node to convert cmd_vel to velocities and calculate P
    """
    # ==========================================================================
    # PUBLICH FUNCTION
    # ==========================================================================
    def __init__(self) -> None:
        """! Class constructor
        """
        # Initialization of ROS related variable
        rospy.init_node("nitra_control_node", anonymous=True)

        self._velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self._gps_subscriber = rospy.Subscriber('fix', NavSatFix, self._callback)

        self._client = WebsocketClient(host_address="http://192.168.86.122:5000")

        self._client.connect(self._on_message)
    
    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin()
    
    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================
    def _callback(self, msg):
        """! Callback for GPS data
        @param msg<NavSatFix>
        """
        gps_data = json_message_converter.convert_ros_message_to_json(msg)

        self._client._send_message(gps_data)

    def _on_message(self, msg):
        """! Callback for receiving message from websocket server
        @param msg<str> message from websocket server
        """
        message = json.loads(msg)

        message_type = "geometry_msgs/Twist"

        velocity_msg = json_message_converter.convert_json_to_ros_message(message_type, message["velocity"])

        self._velocity_publisher.publish(velocity_msg)



