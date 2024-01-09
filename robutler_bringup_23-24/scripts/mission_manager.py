#!/usr/bin/env python3

from functools import partial
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


def moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher):

    print('Called moving to ' + location)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    q = quaternion_from_euler(R, P, Y)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id='map', stamp=rospy.Time.now())

    print('Sending Goal move to ' + location)
    goal_publisher.publish(ps)

    # TODO know when move is finished

    try:
        result_msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=60)
    except:
        print('Timeout waiting for moveto')
        # TODO
        return

    print('move base completed goal with result ' + str(result_msg))

def find(feedback, x, y, z, R, P, Y,location, object, goal_publisher):
    print('Finding ' + object + ' in the ' + location)
    
    if location=='bedroom':
        partial(moveTo,x, y, z, R, P, Y, location, goal_publisher)
        spawn(object,location)

    elif location=='gym':
        partial(moveTo,x, y, z, R, P, Y, location, goal_publisher)
        spawn(object,location)

    
    
def spawn(object,location):

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_description_23-24') + '/models/'

    poses = {}

    # on bed pose
    p = Pose()
    p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['bedroom'] = {'pose': p}
    
    p.position = Point(x=1.344, y=2.1515, z=0)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['gym'] = {'pose': p}
    
    objects = {}

    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['ball'] = {'name': 'sphere_v', 'sdf': f.read()}

    # add object person_standing
    f = open(package_path + 'person_standing/model.sdf', 'r')
    objects['person'] = {'name': 'person_standing', 'sdf': f.read()}

    service_name = 'gazebo/spawn_sdf_model'

    service_client = rospy.ServiceProxy(service_name, SpawnModel)

    uuid_str = str(uuid.uuid4())
    service_client(objects[object]['name'] + '_' + uuid_str,
                    objects[object]['sdf'],
                    objects[object]['name'] + '_' + uuid_str,
                    poses[location]['pose'],
                    'world')

def main():

    global server

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    # Create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    server = InteractiveMarkerServer("mission")
    print(server)

    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert("Move to")

    entry = menu_handler.insert("kitchen", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=6.568593, y=-1.788789, z=0,
                                                 R=0, P=0, Y=-1.504141,
                                                 location='kitchen',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("bedroom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("livingroom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=0.987, y=0.039, z=0,
                                                 R=0, P=0, Y=0,
                                                 location='livingroom',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("gym", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=1.344, y=2.1515, z=0,
                                                 R=0, P=0.003175, Y=0.706,
                                                 location='gym',
                                                 goal_publisher=goal_publisher))
    # entry = menu_handler.insert("living room", parent=h_first_entry, callback=moveToLivingRoom)
    
    h_second_entry = menu_handler.insert("Find")

    sub_handler1 = menu_handler.insert("red ball", parent=h_second_entry)

    entry = menu_handler.insert("in the bedroom", parent=sub_handler1,
                                callback=partial(find,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='bedroom',object='ball',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("in the gym", parent=sub_handler1,
                                callback=partial(find,
                                                 x=1.344, y=2.1515, z=0,
                                                 R=0, P=0.003175, Y=0.706,
                                                 location='gym',object='ball',
                                                 goal_publisher=goal_publisher))

    sub_handler1 = menu_handler.insert("person", parent=h_second_entry)
    
    entry = menu_handler.insert("in the room", parent=sub_handler1,
                                callback=partial(moveTo,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='bedroom', color='violet',object='person',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("in the gym", parent=sub_handler1,
                                callback=partial(moveTo,
                                                 x=1.344, y=2.1515, z=0,
                                                 R=0, P=0.003175, Y=0.706,
                                                 location='gym', color='violet',object='person',
                                                 goal_publisher=goal_publisher))

    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    rospy.spin()


if __name__ == '__main__':
    main()
