#!/usr/bin/env python
from __future__ import print_function

import rospy
import cv2
import time
import tennisball
import profiler
import imutils
import numpy as np
import __main__ as main
from math import isnan
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Tracker:

    '''
    Tracks a tennis ball and publishes it's location.
    '''

    NODE_NAME = "tracker"

    def __init__(self, name=NODE_NAME):
        # initializes ROS node
        rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
        # lower range of HSV values to be tracked
        self.lower = np.array(tennisball.lower)
        # upper range of HSV values to be tracked
        self.upper = np.array(tennisball.upper)
        # tracked_X/Y/Z = coordinates of tracked object
        self.tracked = [None, None, None]
        self.trackedZhist = [0, 0, 0, 0, 0]
        self.trackedZidx = 0
        self.bridge = CvBridge()
        self.start = time.time()
        self.frames = 0
        # need 2 subscribers - image and depth
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_color", Image, self.image_cb)
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_cb)
        # publisher for depth location
        self.coord_pub = rospy.Publisher('coord', String, queue_size=10)

    def image_cb(self, data):
        '''
        Callback function for tracking the object.
        '''
        try:
            img = np.array(self.bridge.imgmsg_to_cv2(data))
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))

        # subsample
        img = cv2.flip(img, 1)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # IMPORTANT: generate binary array with the image: 1 = pixel is between
        # [lower,upper]
        mask = cv2.inRange(hsv, self.lower, self.upper)

        # dilate/erode to fill in holes in the mask
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)

        # find contour - ideally there should only be one, the one for the
        # tracked object
        # TODO: try to make sure the above holds
        if imutils.is_cv2():
            contours, hier = cv2.findContours(
                mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        elif imutils.is_cv3():
            _, contours, hier = cv2.findContours(
                mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) < 1:
            return
        # find the centroid of the contour of the object
        largest_area = cv2.contourArea(contours[0])
        largest_contour = contours[0]
        for cont in contours:
            area = cv2.contourArea(cont)
            if area > largest_area:
                largest_area = area
                largest_contour = cont
        cv2.drawContours(img, [largest_contour], 0, (0, 255, 0), 3)
        m = cv2.moments(largest_contour)
        if m['m00'] == 0:
            return
        bx = int(m['m10']/m['m00'])
        by = int(m['m01']/m['m00'])
        self.tracked[:2] = bx, by

    def depth_cb(self, data):
        '''
        Callback for working with depth data.
        '''
        try:
            img = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))
        arr = np.array(img, dtype=np.float32)
        arr = cv2.flip(arr, 1)
        arr = cv2.normalize(arr, arr, 0, 1, cv2.NORM_MINMAX)
        # use the tracked object's (X,Y) location to index into the depth array
        # TODO: fix
        if any(x is None for x in self.tracked[:2]):
            return
        else:
            zz = []
            r2 = profiler.Profiler.RADIUS/2
            for dx in range(-r2, r2):
                for dy in range(-r2, r2):
                    try:
                        z = arr[self.tracked[1]+dy, self.tracked[0]+dx]
                    except:
                        continue
                    if not(isnan(z) or z == 0.0):
                        zz.append(z)
            if len(zz) < 1:
                return
            self.trackedZhist[self.trackedZidx] = np.mean(zz)
            self.tracked[2] = np.mean(self.trackedZhist)
            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
            cv2.circle(arr, self.tracked[:2], 10, (0, 0, 255))
            cv2.imshow("Depth", arr)
            cv2.moveWindow("Depth", 400, 0)
            cv2.waitKey(10)

    def broadcast(self):
        r = rospy.Rate(10)
        while True:
            if all(x is not None for x in self.tracked):
                msg = "{},{},{}".format(*self.tracked)
                self.coord_pub.publish(msg)
                rospy.logdebug(msg)
            r.sleep()

if __name__ == "__main__":
    try:
        rospy.loginfo("Starting {}".format(main.__file__))
        tr = Tracker()
        tr.broadcast()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down {}".format(main.__file__))
    cv2.destroyAllWindows()
