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
import time
import json
from actionlib_msgs.msg import GoalID
from robutler_object_spawner_23_24.srv import SpawnObject, DeleteObject
from robutler_picture_saver_23_24.srv import TakePhoto

from std_msgs.msg import String
from robutler_bringup_23_24.msg import DetectionControl, SphereDetection

import random

detected_objects_subscriber = None
goal_publisher = None
spawn_object_client = None
delete_object_client = None
detection_control_publisher = None
server = None
marker_pos = 1
flag_still_moving = False
menu_handler = MenuHandler()
robutler_loc_dict = {}
h_move_entry = 0
h_interrupt_entry = 0
rospack = rospkg.RosPack()
objs_Class = []
count_obj = 0
active_objects = []
current_object = []

#  -------------------------HANDLER------------------------------------------------


def enableCb(feedback):
    global h_move_entry
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible(h_move_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible(h_move_entry, True)

    menu_handler.reApply(server)
    rospy.loginfo("update")
    server.applyChanges()


def modeCb(feedback):
    global h_interrupt_entry
    menu_handler.setCheckState(h_interrupt_entry, MenuHandler.UNCHECKED)
    h_interrupt_entry = feedback.menu_entry_id
    menu_handler.setCheckState(h_interrupt_entry, MenuHandler.CHECKED)

    rospy.loginfo(f"Switching to menu entry # {h_interrupt_entry}")
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


def moveTo(feedback, location):
    global robutler_loc_dict, flag_still_moving, goal_publisher
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
    flag_still_moving = True
    # rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
    # rospy.loginfo(result_msg.status.status)
    # TODO: change this While to maybe threads, One do the main while the others triggers a flag when result_msg.status.status value = 3
    #                                                                           or lissens to MoveBaseActionStatus  status_msg.status.status = 3
    # rospy.loginfo(f"Target Location ({location}) Reached")


def interruption(feedback, cancel_pub):
    rospy.loginfo("Cancelling the current goals...")
    cancel_msg = GoalID()
    cancel_pub.publish(cancel_msg)
    rospy.loginfo("Goals Cancelled")


def sign_of_arrival(msg):
    global flag_still_moving
    flag_still_moving = False


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
    global count_obj, current_object, objs_Class
    objs_Class = msg.data.split(", ")
    rospy.loginfo(f"Received: {msg.data}")
    count_obj += objs_Class.count(current_object)
    # rospy.logwarn("HERE")
    # rospy.loginfo(f"For a total of {count_obj} {current_object} found")


#  -------------------------SPAWNER-&-Mission----------------------------------------------


def delete_objs(feedback, object_ns):
    global delete_object_client
    try:
        resp = delete_object_client(object_ns)
        rospy.loginfo("Object {object_ns} deleted")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def spawn_move_to_find(
    feedback,
    division,
    sp_object,
    object_size,
    num_finds=1,
    location=" ",
    check=False,
    deletion=True,
    num=4,
    percentage=0.75,
    instant=True,
):
    global active_objects, robutler_loc_dict, count_obj, current_object, flag_still_moving, goal_publisher, spawn_object_client, detection_control_publisher, objs_Class, detected_objects_subscriber
    # Delete all active objects that where spawned by SPAWNER
    current_object = sp_object
    if deletion:
        for object_ns in active_objects:
            delete_objs(
                feedback=feedback,
                object_ns=object_ns,
            )

    control_msg = DetectionControl()
    # Spawn new objects
    random_num_finds = random.randint(1, num_finds)
    for _ in range(random_num_finds):
        if any(random.getrandbits(1) for _ in range(num)):
            rospy.loginfo(f"Spawning object ({sp_object}) in the {division}")
            try:
                resp = spawn_object_client(
                    division=division,
                    object_name=sp_object,
                    object_size=object_size,
                    location=location,
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

    if sp_object != "sphere":
        if location == " ":
            for index, (section_name, section_data) in enumerate(
                robutler_loc_dict[division].items()
            ):
                if index > 0 and not instant:
                    control_msg.enable_reading = True
                    control_msg.percentage_threshold = percentage
                    control_msg.instant = instant
                    detection_control_publisher.publish(control_msg)

                moveTo(feedback, section_name)
                while flag_still_moving:
                    time.sleep(0.5)
                rospy.loginfo(f"Target Location Reached")
                time.sleep(2)

                if instant:
                    control_msg.enable_reading = True
                    control_msg.percentage_threshold = percentage
                    control_msg.instant = instant
                    time.sleep(1)
                    detection_control_publisher.publish(control_msg)

                if index > 0 and not instant:
                    control_msg.enable_reading = False
                    detection_control_publisher.publish(control_msg)

                # rospy.logwarn("HERE")
                # rospy.wait_for_message("/mission_manager/detected_objects", String)
                # rospy.logwarn("HERE")
                rospy.loginfo(f"Count: {count_obj}")
                rospy.loginfo(f"Objects: {objs_Class}")

                if count_obj >= random_num_finds:
                    rospy.loginfo(
                        f"Robutler found {sp_object} in {division}, with a certainty above {percentage*100}%"
                    )
                    break
                else:
                    if index == (len(robutler_loc_dict[division].items()) - 1):
                        rospy.loginfo(
                            f"Robutler didn't find all the {sp_object} in {division}"
                        )
                    else:
                        rospy.loginfo(
                            f"There is still {sp_object} to find in {division}"
                        )
        else:
            moveTo(feedback, location)
            while flag_still_moving:
                time.sleep(0.5)
            rospy.loginfo(f"Target Location Reached")
            control_msg.enable_reading = True
            control_msg.percentage_threshold = percentage
            control_msg.instant = instant
            detection_control_publisher.publish(control_msg)
            time.sleep(4)
            control_msg.enable_reading = False
            detection_control_publisher.publish(control_msg)
            if not check:
                if count_obj >= random_num_finds:
                    rospy.loginfo(
                        f"Robutler found {sp_object} with a certainty above {percentage}"
                    )
                else:
                    rospy.loginfo(f"Robutler didn't find {sp_object}")
            else:
                return
    else:
        for index, (section_name, section_data) in enumerate(
            robutler_loc_dict[division].items()
        ):
            moveTo(feedback, section_name)
            while flag_still_moving:
                time.sleep(0.5)
            detected_objects_subscriber = rospy.Subscriber(
                "sphere_detector_node/sphere_detection_results",
                SphereDetection,
                checkSpheres,
            )


def checkSpheres(msg):
    global detected_objects_subscriber
    num_spheres = msg.num_spheres
    sphere_colors = msg.sphere_colors.split(",") if msg.sphere_colors else []

    if sphere_colors:
        rospy.loginfo(
            f"Robutler detected {len(sphere_colors)} with the colours {sphere_colors}"
        )
        detected_objects_subscriber.unregister()


#  -------------------------MISSIONS------------------------------------------------


def check(feedback, location, division, object, num_finds, instant):
    spawn_move_to_find(
        feedback,
        division=division,
        sp_object=object,
        object_size="Small",
        num_finds=num_finds,
        location=location,
        instant=instant,
    )
    if any(obj != "dinnertable" and obj != "chair" for obj in objs_Class):
        rospy.loginfo("The table is not cleared!")
    else:
        rospy.loginfo("The table is cleared!")


def find_in_house(feedback, object, object_size, instant):
    global robutler_loc_dict, count_obj
    percentage = 0.85
    for division_name, division_data in robutler_loc_dict.items():
        spawn_move_to_find(
            feedback=feedback,
            division=division_name,
            sp_object=object,
            object_size=object_size,
            deletion=False,
            num=1,
            percentage=percentage,
            instant=instant,
        )
        if count_obj > 1 and instant:
            rospy.loginfo(f"There is SOMEONE HOME!!!!!(certanty of {percentage*100}%)")
        if count_obj > 1 and not instant:
            rospy.loginfo(f"Robutler has found {count_obj} {object} in the House")
        else:
            rospy.loginfo(f"Robutler didn't find anyone")


#  -------------------------MAIN------------------------------------------------


# TODO: change dictionary to also be a different pkg of mgs Division_Section_Coords
def main():
    global server, h_move_entry, h_interrupt_entry, robutler_loc_dict, goal_publisher, spawn_object_client, delete_object_client, detection_control_publisher

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")
    server = InteractiveMarkerServer("mission")
    print(server)

    # ---------------------------------------Publishers---------------------------------------------------------------------

    goal_publisher = rospy.Publisher(
        "/move_base_simple/goal", PoseStamped, queue_size=5
    )
    detection_control_publisher = rospy.Publisher(
        "/mission_manager/detection_control", DetectionControl, queue_size=10
    )
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=3)

    # ---------------------------------------Clients---------------------------------------------------------------------

    spawn_object_client = rospy.ServiceProxy("/spawner/spawn_object", SpawnObject)
    delete_object_client = rospy.ServiceProxy("/spawner/delete_object", DeleteObject)
    take_photo_client = rospy.ServiceProxy("take_photo_server/take_photo", TakePhoto)

    # ---------------------------------------Subscribers---------------------------------------------------------------------

    detected_objects_subscriber = rospy.Subscriber(
        "/mission_manager/detected_objects", String, update_objs_Class
    )
    arrival_sub = rospy.Subscriber(
        "/move_base/result", MoveBaseActionResult, sign_of_arrival
    )

    # ---------------------------------------Dictionary---------------------------------------------------------------------

    pkg_path = rospack.get_path("robutler_bringup_23_24")
    fullpath = pkg_path + "/dictionary/robutler_check_loc.txt"
    with open(fullpath, "r") as dictionary_file:
        robutler_loc_dict = json.load(dictionary_file)

    obj_names = ["bottle", "laptop", "person", "sphere", "vase", "tvmonitor"]
    object_sizes = ["Small", "Small", "Big", "Small", "Small", "Small"]

    # ----------------------------------------Missions----------------------------------------------------------------------

    h_move_entry = menu_handler.insert("Move to")

    for division_name, division_data in robutler_loc_dict.items():
        h_move_division = menu_handler.insert(division_name, parent=h_move_entry)
        for section_name, section_data in division_data.items():
            menu_handler.insert(
                section_name,
                parent=h_move_division,
                callback=partial(moveTo, location=section_name),
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
                    division=division_name,
                    object_size=object_sizes[index_object],
                    sp_object=spawn_obj,
                    instant=False,
                ),
            )

    entry = menu_handler.insert(
        "Total people in the House",
        parent=h_find_entry,
        callback=partial(
            find_in_house, object="person", object_size="Big", instant=False
        ),
    )
    # --------------------------------------------------------------------------------------------------------------------

    h_checkh_entry = menu_handler.insert("Check if")

    entry = menu_handler.insert(
        "Pc is on the table",
        parent=h_checkh_entry,
        callback=partial(
            spawn_move_to_find,
            division="bedroom",
            object_size="Small",
            sp_object="laptop",
            location="bed_table_on_top",
            instant=True,
        ),
    )

    entry = menu_handler.insert(
        "Bootle of wine is on the table",
        parent=h_checkh_entry,
        callback=partial(
            spawn_move_to_find,
            division="dining_room",
            object_size="Small",
            sp_object="bottle",
            location="top_dining_table",
            instant=True,
        ),
    )

    entry = menu_handler.insert(
        "See if the stove is on",
        parent=h_checkh_entry,
        callback=partial(
            spawn_move_to_find,
            division="kitchen",
            object_size="Small",
            sp_object="sphere",
            location="stove",
            instant=True,
        ),
    )

    entry = menu_handler.insert(
        "Diner table is cleared",
        parent=h_checkh_entry,
        callback=partial(
            check,
            location="top_dining_table",
            division="kitchen",
            object="bottle",
            num_finds=3,
            instant=True,
        ),
    )
    entry = menu_handler.insert(
        "Is someone home",
        parent=h_checkh_entry,
        callback=partial(
            find_in_house,
            object="person",
            object_size="Big",
            instant=False,
        ),
    )

    # --------------------------------------------------------------------------------------------------------------------

    h_photo_entry = menu_handler.insert(
        "Take picture",
        callback=partial(take_picture, take_photo_client=take_photo_client),
    )

    h_interrupt_entry = menu_handler.insert(
        "STOP",
        callback=partial(interruption, cancel_pub=cancel_pub),
    )
    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    rospy.spin()


if __name__ == "__main__":
    main()
