#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SphereDetector:
    def __init__(self):
        rospy.init_node("sphere_detector_node", anonymous=True)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/camera_topic", Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print(e)
            return

        # Process the image to detect spheres
        num_spheres = self.detect_spheres(cv_image)

        # Publish the number of spheres
        rospy.loginfo("Number of spheres: %d" % num_spheres)

    def detect_spheres(self, image):
        # Convert to grayscale.
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Blur using 3 * 3 kernel.
        gray_blurred = cv2.blur(gray, (3, 3))

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(
            gray_blurred,
            cv2.HOUGH_GRADIENT,
            1,
            20,
            param1=50,
            param2=30,
            minRadius=1,
            maxRadius=40,
        )

        # Draw circles that are detected.
        if detected_circles is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))

            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                cv2.circle(image, (a, b), r, (0, 255, 0), 2)

                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(image, (a, b), 1, (0, 0, 255), 3)
                cv2.imshow("Detected Circle", image)
                cv2.waitKey(0)

        return 0  # Placeholder, replace with actual detection logic


if __name__ == "__main__":
    try:
        sphere_detector = SphereDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
