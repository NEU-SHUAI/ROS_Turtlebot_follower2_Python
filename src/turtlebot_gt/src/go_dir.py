#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist


class GoDir():

    def __init__(self):
        rospy.init_node('Gocrazy', anonymous=False)
        self.distance = 0
        self.angle = 0

        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if
        # you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)

        # TurtleBot will stop if we don't keep telling it to move.  How often
        # should we tell it to move? 10 HZ
        r = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = self.distance
        move_cmd.angular.y = self.angle
        while not rospy.is_shutdown():
            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop
        # TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to
        # shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        GoDir()
        GoDir.distance = 0.5
        GoDir.angle = 90
    except:
        rospy.loginfo("GoForward node terminated.")
