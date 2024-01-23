
import rospy
from darknet_ros_msgs.msg import ObjectCount  # Substitua "darknet_ros_msgs" pelo pacote real

class FoundObjectListener:
    def __init__(self):
        rospy.init_node('found_object_listener', anonymous=True)
        self.count_value = None
        self.topic_name = "/darknet_ros/found_object"  # Substitua pelo tópico real
        rospy.Subscriber(self.topic_name, ObjectCount, self.callback)

    def callback(self, data):
        self.count_value = data.count
        rospy.loginfo("Count: %d", self.count_value)

    def listen(self):
        rospy.spin()

if __name__ == '__main__':
    listener = FoundObjectListener()
    listener.listen()
