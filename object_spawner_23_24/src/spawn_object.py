#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import random
import os
import json
import rospkg
import time
from robutler_bringup_23_24.msg import Obj_Spawner_msg


class ObjectSpawner:
    def __init__(self):
        rospy.init_node("spawner")
        self.service_name = "/gazebo/spawn_sdf_model"
        rospy.wait_for_service(self.service_name)
        self.service_client = rospy.ServiceProxy(self.service_name, SpawnModel)
        self.package_path = (
            rospkg.RosPack().get_path("robutler_description_23_24") + "/models/"
        )
        self.objects = self.load_objects()
        self.divisions = self.define_divisions()
        self.spawned_objects = {}  # Dictionary to keep track of spawned objects
        self.object_ns = None

        # Subscribe to the topic for object spawning
        # TODO: change this to the msg file
        self.spawn_subscriber = rospy.Subscriber(
            "/mission_manager/obj_spawner",
            Obj_Spawner_msg,
            self.spawn_object_callback,
        )

    def load_objects(self):
        objects = {}
        for dirname in os.listdir(self.package_path):
            dir_path = os.path.join(self.package_path, dirname)
            if os.path.isdir(dir_path):
                sdf_filename = None
                for filename in os.listdir(dir_path):
                    if filename.endswith(".sdf"):
                        sdf_filename = filename
                        # rospy.loginfo(f"Found {dirname}")
                        file_path = os.path.join(dir_path, sdf_filename)
                        with open(file_path, "r") as f:
                            sdf_content = f.read()
                        break
                if sdf_filename:
                    objects[dirname] = {
                        "name": dirname,
                        "sdf": sdf_content,
                    }
        return objects

    def define_divisions(self):
        divisions = {}
        pkg_path = rospkg.RosPack().get_path("robutler_bringup_23_24")
        fullpath = pkg_path + "/dictionary/object_spawn_loc.txt"
        with open(fullpath, "r") as dictionary_file:
            small_house_dict = json.load(dictionary_file)
        for division_name, division_data in small_house_dict.items():
            divisions[division_name] = division_data
        return divisions

    def spawn_object_callback(self, msg):
        # Callback function for spawning objects
        if msg.spawn_object:
            rospy.loginfo(
                f"Spawner received to spawn the {msg.object_name} in {msg.division}"
            )
            self.spawn_object(msg.division, msg.object_name)
        else:
            rospy.loginfo(f"Spawner received to delete all objects in {msg.division}")
            self.delete_object(msg.division)

    def spawn_object(self, division, object_name):
        # Spawn object logic
        if division not in self.divisions:
            rospy.logerr(
                f"Division '{division}' is unknown. Available locations are {list(self.divisions.keys())}"
            )
            return

        if object_name not in self.objects:
            rospy.logerr(
                f"Object '{object_name}' is unknown. Available objects are {list(self.objects.keys())}"
            )
            return

        temp_sections = self.divisions[division]
        num_sec = list(temp_sections.keys())
        random_sec = random.choice(num_sec)
        random_sec_coords = temp_sections.get(random_sec, {}).get("Coords", [])
        (x, y, z, roll, pitch, yaw) = random_sec_coords

        pose = Pose()
        pose.position = Point(x, y, z)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(*q)

        uuid_str = str(uuid.uuid4())

        self.object_ns = self.objects[object_name]["name"] + "_" + uuid_str
        try:
            self.service_client(
                model_name=self.objects[object_name]["name"] + "_" + uuid_str,
                model_xml=self.objects[object_name]["sdf"],
                robot_namespace=self.objects[object_name]["name"] + "_" + uuid_str,
                initial_pose=pose,
                reference_frame="world",
            )
            rospy.loginfo(f"Spawned object '{object_name}' at location '{division}'")
            # Save the spawned object
            self.spawned_objects[self.object_ns] = (division, object_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def delete_object(self, division):
        if division not in self.divisions:
            rospy.logerr(
                f"Division '{division}' is unknown. Available locations are {list(self.divisions.keys())}"
            )
            return
        for object_ns, (div, _) in self.spawned_objects.items():
            if div == division:
                try:
                    delete_service_name = "/gazebo/delete_model"
                    rospy.wait_for_service(delete_service_name)
                    delete_service = rospy.ServiceProxy(
                        delete_service_name, DeleteModel
                    )
                    delete_service(object_ns)
                    rospy.loginfo(f"Deleted object '{object_ns}'")
                    del self.spawned_objects[object_ns]
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    object = ObjectSpawner()
    rospy.spin()
