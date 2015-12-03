#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('turtlebot_gt')
import rospy
import sys

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as dynamic_reconfig

# Import custom message data and dynamic reconfigure variables.
from depth.msg import depth_data
from depth.cfg import depth_paramsConfig as ConfigType


# Node example class.
class DepthParams():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        init_message = rospy.get_param('~x', 1.0)
        rate = float(rospy.get_param('~rate', '1.0'))
        topic = rospy.get_param('~topic', 'simple_sub')
        rospy.loginfo('rate = %d', rate)
        rospy.loginfo('topic = %s', topic)
        # Create a dynamic reconfigure server.
        self.server = dynamic_reconfig(ConfigType, self.reconfigure)
        # Create a publisher for our custom message.
        pub = rospy.Publisher(topic, depth_data)
        # Set the message to publish as our custom message.
        msg = depth_data()
        # Initialize message variables.
        msg.x = 0.8
        msg.y = 0.7
        msg.z = 0.6
        # Main while loop.
        while not rospy.is_shutdown():
            # Fill in custom message variables with values from dynamic reconfigure server.
            msg.x = self.x
            msg.y = self.y
            msg.z = self.z
            # Publish our custom message.
            pub.publish(msg)
            # Sleep for a while before publishing new messages. Division is so rate != period.
            if rate:
                rospy.sleep(1 / rate)
            else:
                rospy.sleep(1.0)

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.x = config["x"]
        self.y = config["y"]
        self.z = config["z"]
        # Return the new variables.
        return config

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('simple_sub')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = DepthParams()
    except rospy.ROSInterruptException:
        pass
