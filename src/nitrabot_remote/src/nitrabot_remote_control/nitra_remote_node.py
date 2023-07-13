#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
#
# Distributed under terms of the MIT license.

# Standard library

# External library
import rospy


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
    
    def run(self) -> None:
        """! Start ros node
        """
        rospy.spin()
    
    # ==========================================================================
    # PRIVATE FUNCTION
    # ==========================================================================