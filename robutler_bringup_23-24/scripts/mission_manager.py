#!/usr/bin/env python3

from functools import partial
import subprocess
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import uuid
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
import cv2

server = None
marker_pos = 1

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0


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

    rospy.loginfo("Switching to menu entry #" + str(h_mode_last))
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


def spawn_object(loc, object):
    # could be simplified by including spawn_object.py
    # -> making it a class -> using a dictionary -> small_house{name,division{name,sections{name,area(placement)}}}
    bashCommand = (
        "rosrun robutler_bringup_23-24 spawn_object.py -l "
        + str(loc)
        + " -o "
        + str(object)
    )
    spawn_process = subprocess.Popen(bashCommand.split())
    output, error = spawn_process.communicate()


def moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher):
    print("Called moving to " + location)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    q = quaternion_from_euler(R, P, Y)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id="map", stamp=rospy.Time.now())

    print("Sending Goal move to " + location)
    goal_publisher.publish(ps)

    result_msg = rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
    # rospy.loginfo(result_msg.status.status)
    # TODO: change this While to maybe threads, One do the main while the others triggers a flag when result_msg.status.status value = 3
    #                                                                           or lissens to MoveBaseActionStatus  status_msg.status.status = 3
    rospy.loginfo("Target Location (" + location + ") Reached")
    # print("move base completed goal with result " + str(result_msg))


def find(location, color, object):
    print(
        "Finding " + object + " in the " + location + " turning on color_segmentation!"
    )
    if object == "ball":

        bashCommand = "rosrun perception_robutler color_segmentation.py - c " + str(color)
        find_process = subprocess.Popen(bashCommand.split())

        find_process.kill()

    elif object == "person":

        bashCommand = "roslaunch darknet_ros darknet_ros.launch"
        find_process = subprocess.Popen(bashCommand.split())




def move_and_find(feedback, x, y, z, R, P, Y, location, color, object, goal_publisher):
    room_locations = {"on_bed", "on_bed_side_table", "on_corner_chair"}
    gym_locations = {"on_table", "on_exercise_bench", "on_corner"}

    if location == "bedroom":
        for loc in room_locations:
            spawn_object(loc, object)
    elif location == "gym":
        for loc in gym_locations:
            spawn_object(loc, object)

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)
    find(location, color, object)

    # bashCommand = "roslaunch darknet_ros darknet_ros.launch"
    # find_process = subprocess.Popen(bashCommand.split())
    # output, error = find_process.communicate()


def take_picture(feedback):
    bashCommand = "rosrun perception_robutler take_picture.py"
    picture_process = subprocess.Popen(bashCommand.split())
    output, error = picture_process.communicate()


def check(feedback, x, y, z, R, P, Y, location, object, goal_publisher):
    if object == "pc":
        bashCommand = (
            "rosrun robutler_bringup_23-24 spawn_object.py -l "
            + str(location)
            + " -o "
            + str(object)
        )
        check_process = subprocess.Popen(bashCommand.split())
        output, error = check_process.communicate()

    elif object == "bottle":
        bashCommand = (
            "rosrun robutler_bringup_23-24 spawn_object.py -l "
            + str(location)
            + " -o "
            + str(object)
        )
        check_process = subprocess.Popen(bashCommand.split())
        output, error = check_process.communicate()

    moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher)
    bashCommand = "roslaunch darknet_ros darknet_ros.launch"
    check_process = subprocess.Popen(bashCommand.split())

    while True:
        key = cv2.waitKey(1)

        if key == ord("w"):
            check_process.kill()
        break
    print("Object found!")
    # if sucess:
    #   bashCommand = "roslaunch darknet_ros darknet_ros.launch"
    #   find_process = subprocess.Popen(bashCommand.split())
    #   check_process.kill()


def main():
    global server

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    # Create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher(
        "/move_base_simple/goal", PoseStamped, queue_size=1
    )

    server = InteractiveMarkerServer("mission")
    print(server)

    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert("Move to")

    entry = menu_handler.insert(
        "kitchen",
        parent=h_first_entry,
        callback=partial(
            moveTo,
            x=6.568593,
            y=-1.788789,
            z=0,
            R=0,
            P=0,
            Y=-1.504141,
            location="kitchen",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "bedroom",
        parent=h_first_entry,
        callback=partial(
            moveTo,
            x=-4.409525,
            y=-0.182006,
            z=0,
            R=-0.000007,
            P=0.003198,
            Y=1.980398,
            location="bedroom",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "livingroom",
        parent=h_first_entry,
        callback=partial(
            moveTo,
            x=0.987,
            y=0.039,
            z=0,
            R=0,
            P=0,
            Y=0,
            location="livingroom",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "gym",
        parent=h_first_entry,
        callback=partial(
            moveTo,
            x=1.344,
            y=2.1515,
            z=0,
            R=0,
            P=0.003175,
            Y=0.706,
            location="gym",
            goal_publisher=goal_publisher,
        ),
    )
    # entry = menu_handler.insert("living room", parent=h_first_entry, callback=moveToLivingRoom)

    h_second_entry = menu_handler.insert("Find")

    sub_handler1 = menu_handler.insert("Violet ball", parent=h_second_entry)

    entry = menu_handler.insert(
        "In the bedroom",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=-4.409525,
            y=-0.182006,
            z=0,
            R=-0.000007,
            P=0.003198,
            Y=1.980398,
            location="bedroom",
            color="violet",
            object="violet_ball",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "In the gym",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=1.344,
            y=2.1515,
            z=0,
            R=0,
            P=0.003175,
            Y=0.706,
            location="gym",
            color="violet",
            object="violet_ball",
            goal_publisher=goal_publisher,
        ),
    )

    sub_handler1 = menu_handler.insert("Red ball", parent=h_second_entry)

    entry = menu_handler.insert(
        "In the bedroom",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=-4.409525,
            y=-0.182006,
            z=0,
            R=-0.000007,
            P=0.003198,
            Y=1.980398,
            location="bedroom",
            color="red",
            object="red_ball",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "In the gym",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=1.344,
            y=2.1515,
            z=0,
            R=0,
            P=0.003175,
            Y=0.706,
            location="gym",
            color="red",
            object="red_ball",
            goal_publisher=goal_publisher,
        ),
    )

    sub_handler1 = menu_handler.insert("Blue ball", parent=h_second_entry)

    entry = menu_handler.insert(
        "In the bedroom",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=-4.409525,
            y=-0.182006,
            z=0,
            R=-0.000007,
            P=0.003198,
            Y=1.980398,
            location="bedroom",
            color="blue",
            object="blue_ball",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "In the gym",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=1.344,
            y=2.1515,
            z=0,
            R=0,
            P=0.003175,
            Y=0.706,
            location="gym",
            color="blue",
            object="blue_ball",
            goal_publisher=goal_publisher,
        ),
    )

    sub_handler1 = menu_handler.insert("person", parent=h_second_entry)

    entry = menu_handler.insert(
        "In the room",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=-4.409525,
            y=-0.182006,
            z=0,
            R=-0.000007,
            P=0.003198,
            Y=1.980398,
            location="bedroom",
            color="violet",
            object="person",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "In the gym",
        parent=sub_handler1,
        callback=partial(
            move_and_find,
            x=1.344,
            y=2.1515,
            z=0,
            R=0,
            P=0.003175,
            Y=0.706,
            location="gym",
            color="violet",
            object="person",
            goal_publisher=goal_publisher,
        ),
    )

    h_third_entry = menu_handler.insert("Take picture", callback=take_picture)

    h_fourth_entry = menu_handler.insert("Check if")

    entry = menu_handler.insert(
        "Pc is on the table",
        parent=h_fourth_entry,
        callback=partial(
            check,
            x=-7.622083,
            y=0.526304,
            z=-0.001006,
            R=-0.000007,
            P=0.003169,
            Y=2.379986,
            location="table_bedroom",
            object="pc",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "Can of coke is on the table",
        parent=h_fourth_entry,
        callback=partial(
            check,
            x=-7.622083,
            y=0.526304,
            z=-0.001006,
            R=-0.000007,
            P=0.003169,
            Y=2.379986,
            location="table_bedroom",
            object="can_coke",
            goal_publisher=goal_publisher,
        ),
    )

    entry = menu_handler.insert(
        "Bootle of wine is on the table",
        parent=h_fourth_entry,
        callback=partial(
            check,
            x=-7.622083,
            y=0.526304,
            z=-0.001006,
            R=-0.000007,
            P=0.003169,
            Y=2.379986,
            location="table_bedroom",
            object="bottle",
            goal_publisher=goal_publisher,
        ),
    )

    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    rospy.spin()


if __name__ == "__main__":
    main()
