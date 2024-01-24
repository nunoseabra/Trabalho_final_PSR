Trabalho final da unidade curricular PSR - Universidade de Aveiro

launch codes:

roscore


<---------- Gazebo and robutler ----------->

roslaunch robutler_bringup_23_24 gazebo.launch

roslaunch robutler_bringup_23_24 bringup.launch

roslaunch robutler_bringup_23_24 yolo_v3.launch


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


