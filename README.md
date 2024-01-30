Trabalho final da unidade curricular PSR - Universidade de Aveiro

launch codes:

roscore


<---------- Gazebo and robutler ----------->

roslaunch robutler_bringup_23_24 gazebo.launch

roslaunch robutler_bringup_23_24 bringup.launch use_yolo:=false use_arm:=true update_map_dict:=true

roslaunch robutler_detection_23_24 detection.launch


<---------- Navigation ----------->

roslaunch robutler_navigation_23_24 navigation.launch

roslaunch robutler_navigation_23_24 localization.launch


<---------- Mapping----------->

rosrun gmapping slam_gmapping scan:=/scan _base_frame:=base_footprint

rosrun map_server map_saver -f my_map


<---------- Tele control ----------->

rosrun robutler_bringup_23_24 teleop.py

rosrun robutler_bringup_23_24 mission_manager.py

rosrun robutler_bringup_23_24 spawn_object.py - l (Location) -o (Object)

<---------- Manipulation ----------->

roslaunch robutler_bringup_23_24 gazebo.launch

Em um outro terminal Crtl+Shift+t:

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

Em um outro terminal Crtl+Shift+t

roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch




<!-- TODO: Change the dictionary from robutler_bringup to robutler_description (Change the launch and rospgk) -->