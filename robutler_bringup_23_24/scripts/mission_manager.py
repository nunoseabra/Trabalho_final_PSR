#!/usr/bin/env python3

from functools import partial
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
from darknet_ros_msgs.msg import BoundingBoxes
import time
import pandas as pd
import json

from object_spawner_23_24.srv import SpawnObject, DeleteObject
from robutler_picture_saver_23_24.srv import TakePhoto

from std_msgs.msg import String, Bool
from robutler_bringup_23_24.msg import DetectionControl

import random

server = None
marker_pos = 1

menu_handler = MenuHandler()
robutler_loc_dict = {}
h_first_entry = 0
h_mode_last = 0
rospack = rospkg.RosPack()

count_obj = 0
active_objects = []

#  -------------------------HANDLER------------------------------------------------


def enableCb(feedback):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible(h_first_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible(h_first_entry, True)

    menu_handler.reApply(server)
    rospy.loginfo("update")
    server.applyChanges()


def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

    rospy.loginfo(f"Switching to menu entry # {h_mode_last}")
    menu_handler.reApply(server)
    print("DONE")
    server.applyChanges()


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.2

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def makeEmptyMarker(dummyBox=True):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.z = marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker


def makeMenuMarker(name):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append(makeBox(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker)


def deepCb(feedback):
    rospy.loginfo("The deep sub-menu has been found.")


#  -------------------------MOVEMENT------------------------------------------------


def moveTo(feedback, location, goal_publisher):
    global robutler_loc_dict
    rospy.loginfo(f"Called moving to {location}")

    for division_name, division_data in robutler_loc_dict.items():
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
    goal_publisher.publish(ps)

    rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
    # rospy.loginfo(result_msg.status.status)
    # TODO: change this While to maybe threads, One do the main while the others triggers a flag when result_msg.status.status value = 3
    #                                                                           or lissens to MoveBaseActionStatus  status_msg.status.status = 3
    rospy.loginfo(f"Target Location ({location}) Reached")


#  -------------------------PICTURE------------------------------------------------


def take_picture(feedback, take_photo_client):
    rospy.wait_for_service("take_photo_server/take_photo")
    try:
        response = take_photo_client()
        rospy.logwarn(response)
        rospy.loginfo("Photo taken successfully")

    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


#  -------------------------DETECTION------------------------------------------------


def update_objs_Class(msg):
    global count_obj, current_object
    objs_Class = msg.data.split(", ")
    count_obj = objs_Class.count(current_object)


def check(feedback, location, object, goal_publisher):
    return None


def find_in_house(feedback, object, goal_publisher):
    return None


#  -------------------------SPAWNER------------------------------------------------


def delete_objs(feedback, object_ns, delete_object_client):
    req = DeleteObject()
    req.object_ns = object_ns

    try:
        resp = delete_object_client(req)
        rospy.loginfo("Object {object_ns} deleted")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def spawn_move_to_find(
    feedback,
    division,
    sp_object,
    object_size,
    goal_publisher,
    spawn_object_client,
    delete_object_client,
    detection_control_publisher,
    num_finds,
):
    global active_objects, robutler_loc_dict, count_obj

    for object_ns in active_objects:
        partial(
            delete_objs, object_ns=object_ns, delete_object_client=delete_object_client
        )
        rospy.logwarn("TESTS")

    random_num_finds = random.randint(1, num_finds)
    for _ in range(random_num_finds):
        if any(random.getrandbits(1) for _ in range(3)):
            rospy.loginfo(f"Spawning object ({sp_object}) in the {division}")

            try:
                resp = spawn_object_client(
                    division=division, object_name=sp_object, object_size=object_size
                )
                active_objects.append(resp.object_namespace)
                rospy.loginfo(
                    "Object spawned in {division}, robot namespace: %s"
                    % resp.object_namespace
                )
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        else:
            rospy.loginfo(f"It will not spawn the object ({sp_object}) in {division}")

    for index, (section_name, section_data) in enumerate(
        robutler_loc_dict[division].items()
    ):
        if index > 0:
            control_msg = DetectionControl()
            control_msg.enable_reading = True
            control_msg.percentage_threshold = 0.85
            detection_control_publisher.publish(control_msg)

        moveTo(feedback, section_name, goal_publisher)

        if index > 0:
            control_msg.enable_reading = False
            detection_control_publisher.publish(control_msg)

        if count_obj >= random_num_finds:
            rospy.loginfo(f"Robutler found {sp_object} in {division}")
            break
        else:
            if index == len(robutler_loc_dict[division].items() - 1):
                rospy.loginfo(f"Robutler didn't find all the {sp_object} in {division}")
            else:
                rospy.loginfo(f"There is still {sp_object} to find in {division}")



#  -------------------------MAIN------------------------------------------------

# TODO: change dictionary to also be a different pkg of mgs Division_Section_Coords
def main():
    global server, h_first_entry, h_mode_last, robutler_loc_dict

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    # Create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    spawn_object_client = rospy.ServiceProxy("spawner/spawn_object", SpawnObject)
    delete_object_client = rospy.ServiceProxy("spawner/delete_object", DeleteObject)
    take_photo_client = rospy.ServiceProxy("take_photo_server/take_photo", TakePhoto)
    detected_objects_subscriber = rospy.Subscriber(
        "/mission_manager/detected_objects", String, update_objs_Class
    )
    detection_control_publisher = rospy.Publisher(
        "/mission_manager/detection_control", DetectionControl, queue_size=10
    )

    server = InteractiveMarkerServer("mission")
    print(server)

    pkg_path = rospack.get_path("robutler_bringup_23_24")
    fullpath = pkg_path + "/dictionary/robutler_check_loc.txt"
    with open(fullpath, "r") as dictionary_file:
        robutler_loc_dict = json.load(dictionary_file)

    obj_names = ["bottle", "laptop", "person", "sphere", "vase"]
    object_sizes = ["Small", "Small", "Big", "Small", "Small"]

    h_move_entry = menu_handler.insert("Move to")

    for division_name, division_data in robutler_loc_dict.items():
        h_move_division = menu_handler.insert(division_name, parent=h_move_entry)
        for section_name, section_data in division_data.items():
            menu_handler.insert(
                section_name,
                parent=h_move_division,
                callback=partial(
                    moveTo, location=section_name, goal_publisher=goal_publisher
                ),
            )

    h_find_entry = menu_handler.insert("Find")

    for index_object, spawn_obj in enumerate(obj_names):
        h_find_obj_entry = menu_handler.insert(spawn_obj, parent=h_find_entry)
        for division_name, division_data in robutler_loc_dict.items():
            menu_handler.insert(
                division_name,
                parent=h_find_obj_entry,
                callback=partial(
                    spawn_move_to_find,
                    spawn_object_client=spawn_object_client,
                    delete_object_client=delete_object_client,
                    division=division_name,
                    object_size=object_sizes[index_object],
                    sp_object=spawn_obj,
                    goal_publisher=goal_publisher,
                    detection_control_publisher=detection_control_publisher,
                ),
            )

    h_photo_entry = menu_handler.insert(
        "Take picture",
        callback=partial(take_picture, take_photo_client=take_photo_client),
    )

    # --------------------------------------------------------------------------------------------------------------------

    h_fourth_entry = menu_handler.insert("Check if")

    entry = menu_handler.insert(
        "Pc is on the table",
        parent=h_fourth_entry,
        callback=partial(
            move_and_find,
            location="bed_table_on_top",
            object="pc",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "Can of coke is on the table",
        parent=h_fourth_entry,
        callback=partial(
            move_and_find,
            location="top_dining_table",
            object="can_coke",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "Bootle of wine is on the table",
        parent=h_fourth_entry,
        callback=partial(
            move_and_find,
            location="top_dining_table",
            object="bottle",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "Diner table is cleared",
        parent=h_fourth_entry,
        callback=partial(
            check,
            location="top_dining_table",
            object="diningtable",
            goal_publisher=goal_publisher,
        ),
    )
    entry = menu_handler.insert(
        "Is someone home",
        parent=h_fourth_entry,
        callback=partial(find_in_house, object="person", goal_publisher=goal_publisher),
    )

    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    rospy.spin()


if __name__ == "__main__":
    main()
