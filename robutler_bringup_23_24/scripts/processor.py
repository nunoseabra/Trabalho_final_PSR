#!/usr/bin/env python3

import rospy
from object_spawner_23_24.srv import SpawnObject, DeleteObject
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from robutler_bringup_23_24.msg import DetectionControl
from tf.transformations import quaternion_from_euler
import random
import time
from std_msgs.msg import String
import rospkg
import json


class Processor:
    def __init__(self):
        rospy.init_node("/processor")
        rospy.Subscriber("/mission_manager/moveTo", String, self.moveTo)
        rospy.Subscriber("/mission_manager/spawn_move_to_find")
        self.goal_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=5
        )
        pkg_path = rospkg.RosPack().get_path("robutler_bringup_23_24")
        fullpath = pkg_path + "/dictionary/robutler_check_loc.txt"
        with open(fullpath, "r") as dictionary_file:
            self.robutler_loc_dict = json.load(dictionary_file)

    def moveTo(self,location):
        rospy.loginfo(f"Called moving to {location}")

        for _, division_data in self.robutler_loc_dict.items():
            for section_name, section_data in division_data.items():
                if section_name == location:
                    coordinates = section_data.get("Coords")
                    if coordinates:
                        rospy.loginfo(f"Coordinates for {location}: {coordinates}")
                        break
                    else:
                        rospy.loginfo(f"No coordinates found for section {location}")
        (x, y, Y) = coordinates

        p = Pose()
        p.position = Point(x=x, y=y, z=0)
        q = quaternion_from_euler(0, 0, Y)  # From euler angles (rpy) to quaternion
        p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        ps = PoseStamped()
        ps.pose = p
        ps.header = Header(frame_id="map", stamp=rospy.Time.now())

        rospy.loginfo(f"Sending Goal move to {location}")
        self.goal_publisher.publish(ps)

    def delete_objs(object_ns, delete_object_client):
        try:
            resp = delete_object_client(object_ns)
            rospy.loginfo("Object {object_ns} deleted")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def spawn_move_to_find(
        self,
        division,
        sp_object,
        object_size,
        goal_publisher,
        spawn_object_client,
        delete_object_client,
        detection_control_publisher,
        num_finds=1,
    ):
        global active_objects, robutler_loc_dict, count_obj, current_object, flag_still_moving, flag_interrupt
        # Delete all active objects that where spawned by SPAWNER
        current_object = sp_object
        for object_ns in active_objects:
            self.delete_objs(
                object_ns=object_ns,
                delete_object_client=delete_object_client,
            )
        flag_interrupt = False
        # Spawn new objects
        random_num_finds = random.randint(1, num_finds)
        for _ in range(random_num_finds):
            if any(random.getrandbits(1) for _ in range(3)):
                rospy.loginfo(f"Spawning object ({sp_object}) in the {division}")
                try:
                    resp = spawn_object_client(
                        division=division,
                        object_name=sp_object,
                        object_size=object_size,
                    )
                    active_objects.append(resp.object_namespace)
                    rospy.loginfo(
                        "Object spawned in {division}, robot namespace: %s"
                        % resp.object_namespace
                    )
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s" % e)
            else:
                rospy.loginfo(
                    f"It will not spawn the object ({sp_object}) in {division}"
                )

        for index, (section_name, section_data) in enumerate(
            robutler_loc_dict[division].items()
        ):
            if index > 0:
                control_msg = DetectionControl()
                control_msg.enable_reading = True
                control_msg.percentage_threshold = 0.70
                detection_control_publisher.publish(control_msg)

            moveTo(feedback, section_name, goal_publisher)
            while flag_still_moving:
                if flag_interrupt:
                    rospy.loginfo(f"Goals interrupted!")
                    return
                time.sleep(0.5)
            rospy.loginfo(f"Target Location Reached")
            time.sleep(2)

            if index > 0:
                control_msg.enable_reading = False
                detection_control_publisher.publish(control_msg)

            if count_obj >= random_num_finds:
                rospy.loginfo(f"Robutler found {sp_object} in {division}")
                break
            else:
                if index == (len(robutler_loc_dict[division].items()) - 1):
                    rospy.loginfo(
                        f"Robutler didn't find all the {sp_object} in {division}"
                    )
                else:
                    rospy.loginfo(f"There is still {sp_object} to find in {division}")


if __name__ == "__main__":
    movement = Processor()
    rospy.spin()
