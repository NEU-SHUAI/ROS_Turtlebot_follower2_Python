#!/usr/bin/env python
from __future__ import print_function

import rospy
import numpy as np
from std_msgs.msg import String, Int8, Int8MultiArray, MultiArrayLayout, MultiArrayDimension
from nav_msgs.msg import GridCells
from dynamic_reconfigure.parameter_generator import *
from dynamic_reconfigure.server import Server as ParamSrv


class ProfilerServer:

    '''
    Generates a server for dynamic reconfiguration of the profile.
    '''

    NODE_NAME = "profiler_server"
    # LOWER_LAYOUT = MultiArrayDimension(label="lower", size=3, stride=3 * 3)
    # UPPER_LAYOUT = MultiArrayDimension(label="upper", size=3, stride=3)

    def __init__(self, name=NODE_NAME):
        # Get the ~private namespace parameters from command line or launch
        # file.
        # initializes ROS node
        rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
        rate = float(rospy.get_param('~rate', '1.0'))
        topic = rospy.get_param('~topic', name)
        rospy.loginfo("rate = {}".format(rate))
        rospy.loginfo("topic = {}".format(topic))
        # create the server
        rospy.logdebug(dir(ParamSrv))
        self.server = ParamSrv(GridCells, self.update_cb)

        self.gen = ParameterGenerator()
        gen.add("lower", str_t, 0, "The data", 0)

        # Create a publisher for our custom message.
        self.pub = rospy.Publisher(
            "/turtlebot/profiler", GridCells, queue_size=2)
        # Initialize message variables.
        self.msg = GridCells()
        rospy.logdebug(self.msg)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish our custom message.
            self.pub.publish(msg)
            # Sleep for a while before publishing new messages. Division is so
            # rate != period.
            if rate:
                rospy.sleep(1 / rate)
            else:
                rospy.sleep(1.0)

    def update_cb(self, config):
        # rospy.logdebug("""Reconfiugre Request: {int_param}, {double_param}, {str_param}, {bool_param}, {size}""".format(**config))
        rospy.logedbug(dir(config))
        return config


if __name__ == "__main__":
    try:
        # never returns
        srv = ProfilerServer()
        rospy.spin()
    except Exception as e:
        rospy.logerr("{}".format(e))
