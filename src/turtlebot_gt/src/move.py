#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)


def move(distance, angle):
    move_cmd = Twist()
    move_cmd.linear.x = distance
    move_cmd.angular.z = angle
    cmd_vel.publish(move_cmd)
