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
from object_spawner_23_24.srv import SpawnObject, SpawnObjectResponse, DeleteObject


class ObjectSpawner:
    def __init__(self):
        rospy.init_node("spawner")
        self.spawn_service_client = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel
        )
        self.delete_service_client = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel
        )

        self.package_path = (
            rospkg.RosPack().get_path("robutler_description_23_24") + "/models/"
        )
        self.objects = self.load_objects()
        self.divisions = self.define_divisions()

        # Setup services for mission_manager
        self.spawn_object_service = rospy.Service(
            "/spawner/spawn_object", SpawnObject, self.spawn_object
        )
        self.delete_object_service = rospy.Service(
            "/spawner/delete_object", DeleteObject, self.delete_object
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
                        file_path = os.path.join(dir_path, sdf_filename)
                        with open(file_path, "r") as f:
                            sdf_content = f.read()
                        break
                if sdf_filename:
                    prefix_name = dirname.split("_")[0]
                    objects[dirname] = {
                        "name": prefix_name,
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

    def spawn_object(self, req):
        division = req.division
        object_name = req.object_name.lower()
        object_size = req.object_size

        if division not in self.divisions:
            rospy.logerr(
                f"Division '{division}' is unknown. Available locations are {list(self.divisions.keys())}"
            )
            return

        object_names = {obj["name"] for obj in self.objects.values()}
        if object_name not in object_names:
            rospy.logerr(
                f"Object '{object_name}' is unknown. Available objects are {list(object_names)}"
            )
            return

        temp_sections = self.divisions[division]
        valid_sections = []
        if object_size == "Big":
            for section_name, section_data in temp_sections.items():
                if "Size" in section_data and section_data["Size"] == object_size:
                    valid_sections.append(section_name)
        else:
            valid_sections = list(temp_sections.keys())
        random_sec = random.choice(valid_sections)
        random_sec_coords = temp_sections.get(random_sec, {}).get("Coords", [])
        (x, y, z, roll, pitch, yaw) = random_sec_coords

        pose = Pose()
        pose.position = Point(x, y, z)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(*q)

        uuid_str = str(uuid.uuid4())

        matching_objects = [
            obj for obj in self.objects.values() if obj["name"] == object_name
        ]

        if not matching_objects:
            rospy.logerr(f"No objects found with the prefix '{object_name}'")
            return

        selected_object = random.choice(matching_objects)
        uuid_str = str(uuid.uuid4())

        try:
            self.spawn_service_client(
                model_name=selected_object["name"] + "_" + uuid_str,
                model_xml=selected_object["sdf"],
                robot_namespace=selected_object["name"] + "_" + uuid_str,
                initial_pose=pose,
                reference_frame="world",
            )
            rospy.loginfo(f"Spawned object '{object_name}' in '{division}'")
            return SpawnObjectResponse(
                object_namespace=selected_object["name"] + "_" + uuid_str
            )

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return SpawnObjectResponse(object_namespace="")

    def delete_object(self, req):
        object_ns = req.object_ns
        try:
            self.delete_service_client(object_ns)
            rospy.loginfo(f"Deleted object '{object_ns}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    object = ObjectSpawner()
    rospy.spin()
