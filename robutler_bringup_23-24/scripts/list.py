
import rospy
from darknet_ros_msgs.msg import BoundingBoxes  # Substitua "darknet_ros_msgs" pelo pacote real

class FoundObjectListener:
    def __init__(self):
        rospy.init_node('found_object_listener', anonymous=True)
        self.count_value = None
        self.topic_name = "/darknet_ros/bounding_boxes"  # Substitua pelo tópico real
        rospy.Subscriber(self.topic_name, BoundingBoxes, self.callback)

    def callback(self, data):
        self.count_value = data.bounding_boxes
        for c in self.count_value:
            print(c.Class)
        #print(self.count_value)
        #rospy.loginfo("Count: %d", self.count_value)
        

    def listen(self):
        rospy.spin()

if __name__ == '__main__':
    listener = FoundObjectListener()
    listener.listen()
