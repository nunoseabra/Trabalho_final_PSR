#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from robutler_bringup_23_24.msg import DetectionControl


class ObjectDetectorNode:
    def __init__(self):
        self.reading_enabled = False
        self.percentage_threshold = 0.0
        self.detected_objects = []
        self.prev_detected_objects = []
        self.instante = False
        self.detected_objects_publisher = rospy.Publisher(
            "/mission_manager/detected_objects", String, queue_size=10
        )
        rospy.Subscriber(
            "/mission_manager/detection_control",
            DetectionControl,
            self.detection_control_callback,
        )
        rospy.Subscriber(
            "/darknet_ros/bounding_boxes", BoundingBoxes, self.listening_to_objects
        )

    def detection_control_callback(self, msg):
        self.instante = msg.instant
        if not msg.enable_reading and self.reading_enabled:
            self.publish_detected_objects()
        self.reading_enabled = msg.enable_reading
        self.percentage_threshold = msg.percentage_threshold
        rospy.loginfo(f"Detection enabled!")
        rospy.loginfo(f"Percentage Threshold: {self.percentage_threshold}%")

    def listening_to_objects(self, msg):
        if self.reading_enabled:
            rospy.loginfo(f"Darknet message received!")
            for bbox in msg.bounding_boxes:
                if bbox.probability >= self.percentage_threshold:
                    if bbox.Class not in self.prev_detected_objects:
                        self.detected_objects.append(bbox.Class)

            self.prev_detected_objects = [
                bbox.Class
                for bbox in msg.bounding_boxes
                if bbox.probability >= self.percentage_threshold
            ]
            # rospy.logwarn("HERE")
        if self.instante:
            self.publish_detected_objects()
            # rospy.logwarn("HERE")

    def publish_detected_objects(self):
        # rospy.logwarn("HERE")
        if self.detected_objects:
            detected_objects_msg = "Objects Detected: " + ", ".join(
                self.detected_objects
            )
            rospy.loginfo(f"Sending: {detected_objects_msg}")
            self.detected_objects_publisher.publish(detected_objects_msg)
            self.detected_objects = []
            # rospy.logwarn("HERE")

    def run(self):
        rospy.init_node("objects_detector", anonymous=True)
        rospy.loginfo(f"Objects Detector node initialized")
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ObjectDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
