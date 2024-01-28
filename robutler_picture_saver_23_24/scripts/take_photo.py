#!/usr/bin/env python3

from __future__ import print_function
import rospy
import rospkg
import uuid
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robutler_picture_saver_23_24.srv import TakePhoto


class TakePhotoServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False

        package_path = rospkg.RosPack().get_path("robutler_picture_saver_23_24")
        folder_path = package_path + "/images"
        self.save_path = folder_path

        self.take_photo_service = rospy.Service(
            "take_photo", TakePhoto, self.take_photo_callback
        )

    def callback(self, data):
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self):
        # Connect image topic
        img_topic = "/camera2/rgb/image_raw2"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        rospy.wait_for_message(img_topic, Image)

        uuid_str = str(uuid.uuid4())
        uuid_str = uuid_str[0:6]
        img_title = "picture_" + uuid_str + ".jpg"

        if self.image_received:
            # Salva a imagem no caminho configurado
            full_path = f"{self.save_path}/{img_title}"
            cv2.imwrite(full_path, self.image)
            # # Display Picture (waits for user input(used for testing)) #TODO remove user input block
            # cv2.imshow("Picture Taken", self.image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            rospy.loginfo(f"Picture saved! Path: {full_path}")
            self.image_sub.unregister()
        else:
            rospy.loginfo("No images received")

    def take_photo_callback(self, req):
        self.take_picture()
        return []


if __name__ == "__main__":
    rospy.init_node("take_photo_server", anonymous=False)
    server = TakePhotoServer()

    rospy.spin()  # Keep the node running
