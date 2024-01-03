#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import argparse


def main():

    # -------------------------------
    # Initialization
    # -------------------------------
    parser = argparse.ArgumentParser(description='Spawn de objetos em determinadas localizacões')
    parser.add_argument('-l', '--location', type=str, help='', required=False,
                        default='on_bed_side_table')
    parser.add_argument('-o', '--object', type=str, help='', required=False,
                        default='sphere_v')

    args = vars(parser.parse_args())  # creates a dictionary
    print(args)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_description_23-24') + '/models/'

    # Defines poses where to put objects
    poses = {}

    # on bed pose
    p = Pose()
    p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed'] = {'pose': p}

    # on bed-side-table pose
    p = Pose()
    p.position = Point(x=-4.489786, y=2.867268, z=0.679033)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table'] = {'pose': p}

    # on bed-side-table pose
    p = Pose()
    p.position = Point(x=-8.213487, y=-4.440661, z=0.351335)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_corner_chair'] = {'pose': p}

    # define objects
    objects = {}

    # add object sphere_v
    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['sphere_v'] = {'name': 'sphere_v', 'sdf': f.read()}

    # add object person_standing
    f = open(package_path + 'person_standing/model.sdf', 'r')
    objects['person_standing'] = {'name': 'person_standing', 'sdf': f.read()}

    # Check if given object and location are valid

    if not args['location'] in poses.keys():
        print('Location ' + args['location'] +
              ' is unknown. Available locations are ' + str(list(poses.keys())))

    elif not args['object'] in objects.keys():
        print('Object ' + args['object'] +
              ' is unknown. Available objects are ' + str(list(objects.keys())))

    # -------------------------------
    # ROS
    # -------------------------------
    
    else:
        rospy.init_node('insert_object', log_level=rospy.INFO)

        service_name = 'gazebo/spawn_sdf_model'
        print('waiting for service ' + service_name + ' ... ', end='')
        rospy.wait_for_service(service_name)
        print('Found')

        service_client = rospy.ServiceProxy(service_name, SpawnModel)

        location = args['location']
        object_name = args['object']

        uuid_str = str(uuid.uuid4())
        service_client(objects[object_name]['name'] + '_' + uuid_str,
                    objects[object_name]['sdf'],
                    objects[object_name]['name'] + '_' + uuid_str,
                    poses[location]['pose'],
                    'world')
        print('Done')


if __name__ == '__main__':
    main()
