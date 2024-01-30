Trabalho final da unidade curricular PSR - Universidade de Aveiro

Requiriments:

<---------- Darknet_ROS ----------->
https://github.com/leggedrobotics/darknet_ros

<---------- Turtlebot3 ----------->
https://github.com/ROBOTIS-GIT/turtlebot3

launch codes:

roscore

<---------- Gazebo and robutler ----------->

roslaunch robutler_bringup_23_24 gazebo.launch

roslaunch robutler_bringup_23_24 bringup.launch use_yolo:=false use_arm:=true

roslaunch robutler_detection_23_24 detection.launch use_yolo:=true use_spheres:=true


<---------- Navigation ----------->

roslaunch robutler_navigation_23_24 navigation.launch

roslaunch robutler_navigation_23_24 localization.launch


<---------- Mapping----------->

rosrun gmapping slam_gmapping scan:=/scan _base_frame:=base_footprint

rosrun map_server map_saver -f my_map


<---------- Tele control ----------->

rosrun robutler_bringup_23_24 teleop.py

<---------- Services ----------->

roslaunch robutler_picture_saver_23_24 take_photo.launch

roslaunch robutler_object_spawner_23_24 object_spawner.launch