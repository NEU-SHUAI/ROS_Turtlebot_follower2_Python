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
# our defined message for depth
# roslib.load_manifest("turtlebot_gt")
# from turtlebot_gt import turtlebot_gt
from turtlebot_gt.msg import Depth

RADIUS = 20

class tracker:
    def __init__(self):
        rospy.init_node("tracker", anonymous=True, log_level=rospy.DEBUG) # initializes ROS node
        self.lower = np.array([0,0,0], np.uint8) # lower range of HSV values to be tracked
        self.upper = np.array([0,0,0], np.uint8) # upper range of HSV values to be tracked
        self.trackedX = None # tracked_X/Y/Z = coordinates of tracked object 
        self.trackedY = None
        self.trackedZ = None
        self.bridge = CvBridge()
        # need 2 subscribers - image and depth
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.profile_cb) 
        self.depth_sub = None
        # publisher for depth location
        self.depth_msg = Depth()
        self.depth_pub = rospy.Publisher('depth', Depth, queue_size=10)


    # one callback function to capture data about the tracked object
    def profile_cb(self, data):
        img = np.array(self.bridge.imgmsg_to_cv2(data))
        img = np.array(img[::2, ::2, :])
        img = cv2.flip(img, 1)
        dim = img.shape
        centerX, centerY = dim[1]/2, dim[0]/2
        # place a circle in the center of image
        cv2.circle(img, (centerX, centerY), RADIUS, (0, 0, 255))
        cv2.imshow("Image", img)
        key = cv2.waitKey(1)
        # waits for a specific keypress to begin capturing data
        if key != 113:
            return
        # acculumate pixel values in the circle
        pixels = []
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for dx in range(-RADIUS, RADIUS):
            for dy in range(-RADIUS, RADIUS):
                if (dx*dx+dy*dy) <= RADIUS*RADIUS:
                    pixels.append(hsv[centerY+dy, centerX+dx,:])

        pixels = np.array(pixels)
        self.lower[0] = min(pixels[:, 0])
        self.upper[0] = max(pixels[:, 0])
        self.lower[1] = min(pixels[:, 1])
        self.upper[1] = max(pixels[:, 1])
        self.lower[2] = min(pixels[:, 2])
        self.upper[2] = max(pixels[:, 2])
        cv2.destroyAllWindows()
        # register new callbacks for tracking
        self.image_sub.unregister()
        ros.loginfo("Changing callbacks")
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.track_cb)
        self.depth_sub = rospy.Subscriber("/camera/depth/image", Image, self.depth_cb)


    # callback function for tracking the object
    def track_cb(self, data):
        try:
            img = np.array(self.bridge.imgmsg_to_cv2(data))
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))
    	# subsample
        img = img[::2,::2,:]
        img = cv2.flip(img, 1)
        dim = img.shape
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # IMPORTANT: generate binary array with the image: 1 = pixel is between [lower,upper]
        mask = cv2.inRange(hsv, self.lower, self.upper)
        # dilate/erode to fill in holes in the mask
        mask = cv2.bitwise_not(mask)
        mask = cv2.dilate(mask, np.ones((5,5), np.uint8), iterations=1)
        mask = cv2.erode(mask, np.ones((5,5), np.uint8), iterations=1)
        mask = cv2.bitwise_not(mask)
        # find contour - ideally there should only be one, the one for the tracked object
        # TODO: make sure the above holds
        contour, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        if len(contour) < 1:
            return
        # find the centroid of the contour of the object
        cont = contour[0]
        M = cv2.moments(cont)
        if M['m00'] == 0:
            return
        bx = int(M['m10']/M['m00'])
        by = int(M['m01']/M['m00'])
        self.trackedX, self.trackedY = bx, by
        # draw a circle and line from the object to the "center of the screen", or some other reference point
        cv2.circle(mask, (bx, by), 10, (0, 0, 255))
        cx, cy = dim[1]/2, dim[0]/2
        cv2.line(mask, (cx, cy), (bx, by), (255, 0, 0))
        cv2.imshow("Image", img)
        cv2.moveWindow("Image", 0, 0)
        cv2.imshow("Mask", mask)
        cv2.moveWindow("Mask", 400, 0)
        cv2.waitKey(10)


    # callback for working with depth data
    def depth_cb(self, data):
        try:
            img = np.array(self.bridge.imgmsg_to_cv2(data, '32FC1'))
            img = img[::2, ::2]
            #img = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))
        # use the tracked object's (X,Y) location to index into the depth array
        # TODO: fix
        if self.trackedX is None or self.trackedY is None:
            return
        else:
            self.trackedZ = img[self.trackedY, self.trackedX]
            if self.trackedZ is None:
                return

        self.depth_msg.x = self.trackedX
        self.depth_msg.y = self.trackedY
        self.depth_msg.y = self.trackedZ
        pub.publish(self.depth_msg)
        ros.logdebug("Ball is at ({},{},{})".format(self.trackdX, self.trackedY, self.trackedZ))
        #cv2.circle(img, (self.trackedX, self.trackedY), 10, (0, 0, 255))
        cv2.imshow("Depth", img)
        cv2.moveWindow("Depth", 0, 400)
        cv2.waitKey(10)
        

if __name__ == "__main__":
    try:
        tr = tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down {}".format(main.__file__))

cv2.destroyAllWindows()
