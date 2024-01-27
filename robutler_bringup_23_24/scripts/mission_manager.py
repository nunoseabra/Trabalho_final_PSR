#!/usr/bin/env python3

from functools import partial
import subprocess
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import uuid
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Bool
import cv2
import time
import pandas as pd
import json
from robutler_bringup_23_24.msg import Obj_Spawner_msg
import os
import random

server = None
marker_pos = 1

menu_handler = MenuHandler()
robutler_loc_dict = {}
h_first_entry = 0
h_mode_last = 0
rospack = rospkg.RosPack()
found_object_listener = None
objs_Percent = []
objs_Class = []
count_obj = 0
active_objects = []


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


def listening_to_objects(msg):
    global found_object_listener, objs_Percent, objs_Class
    rospy.loginfo("Darknet message received!")
    objs_Class = []
    objs_Percent = []

    for bbox in msg.bounding_boxes:
        objs_Class.append(bbox.Class)
        objs_Percent.append(bbox.probability)

    found_object_listener.unregister()
    rospy.loginfo("Bounding Boxes subscriber unregistered")
    # found_object_listener = False


def move_and_find(feedback, location, object, goal_publisher):
    global found_object_listener, objs_Class, objs_Percent, count_obj

    moveTo(feedback, location, goal_publisher)
    rospy.loginfo(f"Finding {object} in the {location}")
    rospy.loginfo("Creating Bounding Boxes subscriber...")
    time.sleep(2)  # Do to the delay betweeen movement and darknet image aquisition
    found_object_listener = rospy.Subscriber(
        "/darknet_ros/bounding_boxes", BoundingBoxes, listening_to_objects
    )
    timer = 0
    try:
        while found_object_listener.get_num_connections() >= 0:
            time.sleep(0.5)
            timer += 0.5
            if timer > 2:
                rospy.logwarn(f"Darknet didn't detect any object here or Darknet is off")
                break
    except:
        rospy.loginfo(f"Robutler has found: {objs_Class} - {objs_Percent}")

    received_data = {"Objects": objs_Class, "Percentage": objs_Percent}
    df = pd.DataFrame(received_data)
    # TODO: this needs to be fixed so that Objects from yolo and Objects from models/ have the same name
    count_obj = (df["Objects"][df["Percentage"] > 0.5] == object).sum()

    rospy.loginfo(
        f"Robutler found ({count_obj}) ({object}), with a certainty above 50%."
    )


def take_picture(feedback, take_photo_pub):
    rospy.loginfo("Sending Flag to take_photo_pub")
    take_photo_pub.publish(Bool(True))


def check(feedback, location, object, goal_publisher):
    global objs_Class, objs_Percent
    spawn_move_to(feedback, location, object, goal_publisher)
    for index_objs, obj_Class in enumerate(objs_Class):
        rospy.loginfo(f"{obj_Class}")
        # TODO: ADD to check if inside the area of bounding_boxes there is another obj
        if str(obj_Class) != "diningtable" or str(obj_Class) != "chair":
            rospy.loginfo(
                f"The table is not cleared, with a certainty of {objs_Percent[index_objs]}%"
            )
            break
        else:
            rospy.loginfo("The table is cleared")


def find_in_house(feedback, object, goal_publisher):
    global found_object_listener, count_obj, robutler_loc_dict
    total_objs = 0
    for division_name, division_data in robutler_loc_dict.items():
        rospy.loginfo(f"Going to Division: {division_name}")
        for section_name, section_data in division_data.items():
            coords = section_data.get("Coords")
            rospy.loginfo(f"Going to Section: {section_name}")
            if coords:
                move_and_find(
                    feedback=feedback,
                    location=division_name,
                    object=object,
                    goal_publisher=goal_publisher,
                )

            total_objs += count_obj
    rospy.loginfo(f"Robutler found {total_objs} {object} in the house")


def delete_objs(feedback, divisions, spawner_publisher):
    for division in divisions:
        rospy.loginfo(f"Removing objects in the {division}")
        spawn_msg = Obj_Spawner_msg()
        spawn_msg.object_name = "All"
        spawn_msg.division = division
        spawn_msg.spawn_object = False
        spawner_publisher.publish(spawn_msg)


# TODO: change the spawn_object to be in it's own package and be a node with pub(obj id) and sub(comand) communication with mission_manager
def spawn_move_to(feedback, division, sp_object, goal_publisher, spawner_publisher):
    global active_objects, robutler_loc_dict, count_obj
    partial(delete_objs, divisions=division, spawner_publisher=spawner_publisher)
    if random.getrandbits(1):
        rospy.loginfo(f"Spawning object ({sp_object}) in the {division}")
        spawn_msg = Obj_Spawner_msg()
        spawn_msg.object_name = sp_object
        spawn_msg.division = division
        spawn_msg.spawn_object = True
        spawner_publisher.publish(spawn_msg)
        rospy.loginfo(f"Object ({sp_object}) spawned in {division}")
    else:
        rospy.loginfo(f"It will not spawn the object ({sp_object}) in {division}")

    for section_name, section_data in robutler_loc_dict[division].items():
        move_and_find(
            feedback=feedback,
            location=section_name,
            object=sp_object,
            goal_publisher=goal_publisher,
        )
        if count_obj >= 1:
            rospy.loginfo(f"Robutler found {sp_object} in {division} {section_name}")
            break


# TODO: change dictionary to also be a different pkg of mgs Division_Section_Coords
def main():
    global server, h_first_entry, h_mode_last, robutler_loc_dict

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    # Create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    take_photo_pub = rospy.Publisher(
        "/mission_manager/take_photo_flag", Bool, queue_size=10
    )
    spawner_publisher = rospy.Publisher(
        "/mission_manager/obj_spawner", Obj_Spawner_msg, queue_size=2
    )

    server = InteractiveMarkerServer("mission")
    print(server)

    pkg_path = rospack.get_path("robutler_bringup_23_24")
    fullpath = pkg_path + "/dictionary/robutler_check_loc.txt"
    with open(fullpath, "r") as dictionary_file:
        robutler_loc_dict = json.load(dictionary_file)

    obj_path = rospkg.RosPack().get_path("robutler_description_23_24") + "/models/"
    obj_names = []
    for item in os.listdir(obj_path):
        if os.path.isdir(os.path.join(obj_path, item)):
            obj_names.append(item)

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

    for spawn_obj in obj_names:
        h_find_obj_entry = menu_handler.insert(spawn_obj, parent=h_find_entry)
        for division_name, division_data in robutler_loc_dict.items():
            menu_handler.insert(
                division_name,
                parent=h_find_obj_entry,
                callback=partial(
                    spawn_move_to,
                    spawner_publisher=spawner_publisher,
                    division=division_name,
                    sp_object=spawn_obj,
                    goal_publisher=goal_publisher,
                ),
            )

    h_photo_entry = menu_handler.insert(
        "Take picture", callback=partial(take_picture, take_photo_pub=take_photo_pub)
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
