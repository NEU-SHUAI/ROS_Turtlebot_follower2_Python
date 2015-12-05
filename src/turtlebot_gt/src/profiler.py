#!/usr/bin/env python

import sys
import rospy
import roslib
import cv2
import numpy as np
import __main__ as main
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
RADIUS = 20

class profiler:
    def __init__(self):
        rospy.init_node("profiler", anonymous=True, log_level=rospy.DEBUG) # initializes ROS node
        self.lower = np.array([0,0,0], np.uint8) # lower range of HSV values to be tracked
        self.upper = np.array([0,0,0], np.uint8) # upper range of HSV values to be tracked
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.track_cb)

    # callback function for tracking the object
    def track_cb(self, data):
        try:
            img = np.array(self.bridge.imgmsg_to_cv2(data))
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))
        img = cv2.flip(img, 1)
        dim = img.shape
        centerX, centerY = dim[1]/2, dim[0]/2
        cv2.circle(img, (centerX, centerY), RADIUS, (0, 0, 255))	
        cv2.imshow("Image", img)
        cv2.moveWindow("Image", 0, 0)
        #cv2.imshow("Mask", mask)
        #cv2.moveWindow("Mask", 400, 0)
        key = cv2.waitKey(10)
        if key == 113:
            pixels = []
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            for dx in range(-RADIUS, RADIUS):
                for dy in range(-RADIUS, RADIUS):
                    if (dx*dx + dy*dy <= RADIUS*RADIUS):
                        pixels.append(hsv[centerY+dy, centerX+dx,:])
            pixels = np.array(pixels)
            self.lower = np.array([min(pixels[:,0]), min(pixels[:,1]), min(pixels[:,2])])
            self.upper = np.array([max(pixels[:,0]), max(pixels[:,1]), max(pixels[:,2])])
            rospy.loginfo("Lower" + str(self.lower) + " Upper" + str(self.upper))      
            f = open(os.path.dirname(__file__)+'/tennisball.py', 'w')
            s = 'lower=(%d, %d, %d)\n'%(self.lower[0], self.lower[1], self.lower[2])
            f.write(s)
            s = 'upper=(%d, %d, %d)\n'%(self.upper[0], self.upper[1], self.upper[2])
            f.write(s)
            f.close()
            #print self.lower, self.upper
            
if __name__ == "__main__":
    try:
        tr = profiler()
        rospy.loginfo("Started {}".format(main.__file__))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down {}".format(main.__file__))
    cv2.destroyAllWindows()
