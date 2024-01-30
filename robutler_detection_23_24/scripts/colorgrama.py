#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from robutler_bringup_23_24.msg import SphereDetection


class SphereDetector:
    def __init__(self):
        

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera2/rgb/image_raw2", Image, self.image_callback
        )

        self.sphere_pub = rospy.Publisher(
            "sphere_detector_node/sphere_detection_results",
            SphereDetection,
            queue_size=10,
        )
        self.prev_detections = []

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        # Process the image to detect spheres
        num_spheres, sphere_colors, current_detections = self.detect_spheres(cv_image)

        filtered_detections = self.apply_position_filter(
            current_detections, sphere_colors
        )

        # Publish the detection results
        detection_msg = SphereDetection()
        detection_msg.num_spheres = len(filtered_detections)
        detection_msg.sphere_colors = ",".join(sphere_colors)
        rospy.loginfo(
            f"num_spheres {len(filtered_detections)} , sphere_colors {sphere_colors}"
        )
        self.sphere_pub.publish(detection_msg)

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
            minRadius=10,
            maxRadius=40,
        )

        num_spheres = 0  # Initialize counter
        total_color_name = []
        current_detections = []
        # Draw circles that are detected.
        if detected_circles is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))

            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]

                current_detections.append((a, b))
                # Crop the circle region
                circle_roi = image[b - r : b + r, a - r : a + r]

                # Convert the cropped image to HSV color space
                hsv_roi = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2HSV)

                # Define the color ranges for detection
                color_ranges = {
                    "red": ([0, 100, 100], [10, 255, 255]),  # Adjusted red range
                    "blue": ([100, 100, 100], [150, 255, 255]),  # Adjusted blue range
                    "violet": ([270, 100, 100], [310, 255, 255]),  # Adjusted violet range
                }
                # Check for each color range
                for color_name, (lower_color, upper_color) in color_ranges.items():
                    # Create a mask to isolate the color within the specified range
                    mask = cv2.inRange(
                        hsv_roi, np.array(lower_color), np.array(upper_color)
                    )

                    # If the mask has non-zero pixels, consider it as detected color
                    if cv2.countNonZero(mask) > 0:
                        # Increment the sphere counter
                        num_spheres += 1
                        total_color_name.append(color_name)

                        # Draw the circumference of the circle.
                        cv2.circle(image, (a, b), r, (0, 255, 0), 2)

                        # Draw a small circle (of radius 1) to show the center.
                        # cv2.circle(image, (a, b), 1, (0, 0, 255), 3)
        else:
            rospy.loginfo("No Spheres Detected")
            num_spheres = 0
            total_color_name = " "

        # Display the image (optional)
        cv2.imshow("Detected Circle", image)
        cv2.waitKey(1)  # Adjust the delay as needed

        return num_spheres, total_color_name, current_detections

    def apply_position_filter(self, current_detections, sphere_colors):
        filtered_detections = []
        filtered_colors = []

        # Iterate over the current detections and compare with the previous ones
        for detection, color in zip(current_detections, sphere_colors):
            if all(
                self.calculate_distance(detection, prev_detection) > 10
                for prev_detection in self.prev_detections
            ):
                filtered_detections.append(detection)
                filtered_colors.append(color)

        return filtered_detections, filtered_colors

    def calculate_distance(self, detection1, detection2):
        # Calculate Euclidean distance between two detections
        return np.linalg.norm(np.array(detection1) - np.array(detection2))


if __name__ == "__main__":
    # try:
    #     sphere_detector = SphereDetector()
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass
    try:
        rospy.init_node("sphere_detector_node", anonymous=True)
        sphere_detector = SphereDetector()

        # Define the rate at which the image callback should be called (e.g., 10 Hz)
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Call the image callback function
            # sphere_detector.image_callback()

            # Sleep to maintain the specified rate
            rate.sleep()

    except rospy.ROSInterruptException:
        pass