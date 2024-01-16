#!/usr/bin/env python3
from __future__ import print_function

import roslib
roslib.load_manifest('robutler_detection')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    #self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw2",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    # my code here----------------------------------

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    vid_thresh = hsv_image.copy()

    # h s v value to segment purple, to obtain other value use color_segmenter.py and a photo 
    lower_bound = np.array([130,150,0])
    upper_bound = np.array([150,255,150])
    
    # TODO lower_bound,upper_bound = color_segmenter()
    # # masking the image using in.range function
    vid_mask=cv2.inRange(vid_thresh,lower_bound, upper_bound)

    #find all the contours of the segmented mask
    cnts,_ = cv2.findContours(vid_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    #checking if any countor is detected then ru the following statements
    if len(cnts) > 0:
        # sorting the contours to find biggest contour
        cnt = sorted(cnts, key = cv2.contourArea, reverse = True)[0]
        
        
        # Calculating the center of the detected contour
        M = cv2.moments(cnt)
        # add 1e-5 to avoid zero division
        center = (int(M['m10'] / M['m00']+1e-5), int(M['m01'] / M['m00']+1e-5))

        cv2.line(cv_image, (int(center[0]) - 10, int(center[1]) + 10), (int(center[0]) + 10, int(center[1]) - 10), (0, 0, 255), 1)
        cv2.line(cv_image, (int(center[0]) + 10, int(center[1]) + 10), (int(center[0]) - 10, int(center[1]) - 10), (0, 0, 255), 1)

    #-----------------------------------------------
    
    # cv2.imshow('Image', cv_image)
    # cv2.imshow("Mask window", vid_mask)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    

def main(args):
  rospy.init_node('detect', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)