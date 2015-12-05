#!/usr/bin/env python
from __future__ import print_function

import os
import rospy
import cv2
import numpy as np
import __main__ as main
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Profiler:

    '''
    Generates a profile of a tennis ball.
    '''

    NODE_NAME = "profiler"
    RADIUS = 20

    def __init__(self, name=NODE_NAME):
        # initializes ROS node
        rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
        # lower range of HSV values to be tracked
        self.lower = np.array([0, 0, 0], np.uint8)
        # upper range of HSV values to be tracked
        self.upper = np.array([0, 0, 0], np.uint8)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_color", Image, self.track_cb)

    # callback function for tracking the object
    def track_cb(self, data):
        try:
            img = np.array(self.bridge.imgmsg_to_cv2(data))
            self.profile_img(img)
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))

    #  generate a profile of from an image and write it to a file
    def profile_img(self, img):
        img = cv2.flip(img, 1)
        dim = img.shape
        centerX, centerY = dim[1]/2, dim[0]/2
        cv2.circle(img, (centerX, centerY), self.RADIUS, (0, 0, 255))
        cv2.imshow("Image", img)
        cv2.moveWindow("Image", 0, 0)
        key = cv2.waitKey(10) & 0xff
        if key == ord('q'):
            self.profile_write(img, [centerX, centerY])

    def profile_write(self, img, center):
        pixels = []
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        rr = self.RADIUS
        for dx in range(-rr, rr):
            for dy in range(-rr, rr):
                if (dx*dx + dy*dy <= rr*rr):
                    pixels.append(hsv[center[1]+dy, center[0]+dx, :])
        pixels = np.array(pixels)
        self.lower = np.array(
            [min(pixels[:, 0]), min(pixels[:, 1]), min(pixels[:, 2])])
        self.upper = np.array(
            [max(pixels[:, 0]), max(pixels[:, 1]), max(pixels[:, 2])])
        rospy.loginfo("Lower" + str(self.lower) + " Upper" + str(self.upper))
        with open(os.path.dirname(main.__file__)+'/tennisball.py', 'w') as f:
            s = 'lower=(%d, %d, %d)\n' % (
                self.lower[0], self.lower[1], self.lower[2])
            f.write(s)
            s = 'upper=(%d, %d, %d)\n' % (
                self.upper[0], self.upper[1], self.upper[2])
            f.write(s)


if __name__ == "__main__":
    try:
        tr = Profiler()
        rospy.loginfo("Started {}".format(main.__file__))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down {}".format(main.__file__))
    cv2.destroyAllWindows()
