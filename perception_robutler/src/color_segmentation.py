#!/usr/bin/env python3

import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def parse_arguments():

    limits = {'color_lower':[0,0,0],'color_upper':[255,255,255]}
    parser = argparse.ArgumentParser(description="Color Segmentation ROS Node")
    parser.add_argument("-c","--color", type=str, default="all", help="Color to look for: all, red, blue, violet.", required=False)
    args = parser.parse_args()

    #This arguments are in hsv form
    if args.color == "all":
        limits['color_lower']=[0,0,0]
        limits['color_upper']=[255,255,255]
    
    elif args.color == "red":
        limits['color_lower'] = [0, 100, 100]  
        limits['color_upper'] = [10, 255, 255] 
    
    elif args.color == "blue":
        limits['color_lower'] = [90, 50, 50] 
        limits['color_upper'] = [110, 255, 255]

    elif args.color == "violet":
        limits['color_lower'] = [100, 0, 100]  # Limite inferior para a cor violeta em RGB
        limits['color_upper'] = [200, 100, 255]  # Limite superior para a cor violeta em RGB

    else:
        print(f"Invalid color specified: {args.color}. Using default color range.")


    return limits

class ColorSegmentationNode:
    def __init__(self, color_lower, color_upper):
        rospy.init_node('image_manipulation_node', anonymous=False)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to the camera feed
        rospy.Subscriber('/camera2/rgb/image_raw2', Image, self.camera_callback)

        # Publisher for downsampled image
        self.segmented_pub = rospy.Publisher('/manipulated_image', Image, queue_size=1)

        self.lower_color = np.array(color_lower)
        self.upper_color = np.array(color_upper)

    def camera_callback(self, camera_image_msg):
        try:
            # Convert camera image message to OpenCV format
            camera_image = self.bridge.imgmsg_to_cv2(camera_image_msg, 'bgr8')
            hsv_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2HSV)            

            mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)
            segmented_image = cv2.bitwise_and(camera_image, camera_image, mask=mask)

            # Blend the segmented image with the original image
            alpha = 0.95  # Adjust the alpha value for blending
            blended_image = cv2.addWeighted(camera_image, 1 - alpha, segmented_image, alpha, 0)

            # Convert the segmented image back to ROS format and publish
            blended_image_msg = self.bridge.cv2_to_imgmsg(blended_image, 'bgr8')
            self.segmented_pub.publish(blended_image_msg)

        except Exception as e:
            rospy.logerr(f"Error processing camera image: {str(e)}")

    def run(self):
        rospy.spin()

# -------------------------------
# MAIN
# -------------------------------

if __name__ == '__main__':
    limits = parse_arguments()
    segmentation_node = ColorSegmentationNode(limits['color_lower'], limits['color_upper'])
    try:
        segmentation_node.run()
    except rospy.ROSInterruptException:
        pass
