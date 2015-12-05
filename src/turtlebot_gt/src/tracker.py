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
import tennisball
from math import isnan

import time

RADIUS = 20

class tracker:
    def __init__(self):
        rospy.init_node("tracker", anonymous=True, log_level=rospy.DEBUG) # initializes ROS node
        self.lower = np.array(tennisball.lower) # lower range of HSV values to be tracked
        self.upper = np.array(tennisball.upper) # upper range of HSV values to be tracked
        self.trackedX = None # tracked_X/Y/Z = coordinates of tracked object
        self.trackedY = None
        self.trackedZ = None
        self.bridge = CvBridge()
        self.start = time.time()
        self.frames = 0
        # need 2 subscribers - image and depth
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_cb)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb)
        # publisher for depth location
        # self.depth_pub = rospy.Publisher('depth', Depth, queue_size=10)
        
    # callback function for tracking the object
    def image_cb(self, data):
        try:
            img = np.array(self.bridge.imgmsg_to_cv2(data))
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))
        # subsample
        img = cv2.flip(img, 1)
        dim = img.shape
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # IMPORTANT: generate binary array with the image: 1 = pixel is between [lower,upper]
        mask = cv2.inRange(hsv, self.lower*0.9, self.upper*1.1)

        # dilate/erode to fill in holes in the mask
        mask = cv2.erode(mask,np.ones((5,5),np.uint8),iterations=1)
        mask = cv2.dilate(mask,np.ones((5,5),np.uint8),iterations=1)
        
        #find contour - ideally there should only be one, the one for the tracked object
        # TODO: try to make sure the above holds
        contours, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        mask = cv2.cvtColor(mask*255, cv2.COLOR_GRAY2RGB)
        if len(contours) < 1:
            return
        # find the centroid of the contour of the object
        largestArea = cv2.contourArea(contours[0])
        largestContour = contours[0]
        for cont in contours:
            area = cv2.contourArea(cont)
            if area > largestArea:
                largestArea = area
                largestContour = cont
        cv2.drawContours(img, [largestContour], 0, (0, 255, 0), 3)
        M = cv2.moments(largestContour)
        if M['m00'] == 0:
            return
        bx = int(M['m10']/M['m00'])
        by = int(M['m01']/M['m00'])
        self.trackedX, self.trackedY = int(bx), int(by)
        # draw a circle and line from the object to the "center of the screen", or some other reference point
        cv2.circle(mask, (bx, by), 10, (0, 0, 255))
        cx, cy = dim[1]/2, dim[0]/2
        cv2.line(mask, (cx, cy), (bx, by), (255, 0, 0))
        #cv2.imshow("Image", img)
        #cv2.moveWindow("Image", 0, 0)
        #cv2.imshow("Mask", mask)
        #cv2.moveWindow("Mask", 800, 0)
        #cv2.waitKey(10)
        #self.frames += 1
        #t = time.time()-self.start
        #print ("fps:%f"+str(self.frames/t))
        #rospy.loginfo("fps:%f"+str(self.frames/t))


    # callback for working with depth data
    def depth_cb(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError, e:
            rospy.logerr("{}".format(e))
        arr = np.array(img, dtype=np.float32)
        arr = cv2.flip(arr, 1)
        dim = arr.shape
        arr = cv2.normalize(arr, arr, 0, 1, cv2.NORM_MINMAX) 
        # use the tracked object's (X,Y) location to index into the depth array
        # TODO: fix
        if self.trackedX is None or self.trackedY is None:
            return
        else:
            depth_z = sum_z = mean_z = 0
            good_count = 0
            for dx in range(-RADIUS/2, RADIUS/2):
                for dy in range(-RADIUS/2, RADIUS/2):
                    try:
                        z = arr[self.trackedY/2+dy, self.trackedX/2+dx]
                    except:
                        continue
                    if not(isnan(z) or z == 0.0):
                        sum_z += z
                        good_count += 1    
            if good_count == 0:
                return
            mean_z = sum_z/good_count
            self.trackedZ = mean_z
            print self.trackedX, self.trackedY, self.trackedZ
        # FIXME:
        # self.depth_msg.x = self.trackedX
        # self.depth_msg.y = self.trackedY
        # self.depth_msg.y = self.trackedZ
        # pub.publish(self.depth_msg)
        #rospy.logdebug("Ball is at ({},{},{})".format(self.trackedX, self.trackedY, self.trackedZ))
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
        cv2.circle(arr, (int(self.trackedX)/2, int(self.trackedY)/2), 10, (0, 0, 255))
        cv2.imshow("Depth", arr)
        cv2.moveWindow("Depth", 400, 0)
        cv2.waitKey(10)

if __name__ == "__main__":
    try:
        tr = tracker()
        rospy.loginfo("Started {}".format(main.__file__))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down {}".format(main.__file__))
    cv2.destroyAllWindows()
