#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class ArmManipulator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("search_command")

        # MoveIt Commander
        self.robot_commander = RobotCommander()
        self.arm_group = MoveGroupCommander("robutler_arm")  # "arm" is the name of the move group for the robot arm
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")

        # Subscriber to check result
        self.result_subscriber = rospy.Subscriber('/result_topic', Bool, self.result_callback)
        self.check_result = None

        # Wait for the MoveIt interface to come up
        self.arm_group.set_planning_time(5)
        rospy.sleep(2)

    def result_callback(self, msg):
        self.check_result = msg.data

    def move_arm_to_search_pose(self):
        # Move the arm to the predefined "search" pose
        self.arm_group.set_named_target("searching")
        self.arm_group.go(wait=True)

    def rotate_first_joint_slowly(self):
        # Slowly rotate the first joint while checking the result
        rate = rospy.Rate(1)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            if self.check_result:
                rospy.loginfo("Object found. Stopping arm rotation.")
                break

            current_joint_values = self.arm_group.get_current_joint_values()
            current_joint_values[0] += 0.05  # Adjust the rotation increment as needed
            self.arm_group.set_joint_value_target(current_joint_values)
            self.arm_group.go(wait=True)

            rate.sleep()

    def execute_manipulation_sequence(self):
        rospy.loginfo("Executing manipulation sequence.")

        # Move the arm to the search pose
        self.move_arm_to_search_pose()

        # Rotate the first joint slowly while looking for the object
        self.rotate_first_joint_slowly()

        rospy.loginfo("Manipulation sequence completed.")

if __name__ == "__main__":
    arm_manipulator = ArmManipulator()
    arm_manipulator.execute_manipulation_sequence()
    rospy.spin()
