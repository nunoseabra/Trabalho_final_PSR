Trabalho final da unidade curricular PSR - Universidade de Aveiro

launch codes:

roscore


<---------- Gazebo and robutler ----------->
roslaunch robutler_bringup_23-24 gazebo.launch

roslaunch robutler_bringup_23-24 bringup.launch


<---------- Navigation ----------->
roslaunch robutler_navigation_23-24 navigation.launch

roslaunch robutler_navigation_23-24 localization.launch


<---------- Mapping----------->
rosrun gmapping slam_gmapping scan:=/scan _base_frame:=base_footprint

rosrun map_server map_saver -f my_map





<---------- Tele control ----------->
rosrun robutler_bringup_23-24 teleop.py

rosrun robutler_bringup_23-24 mission_manager.py

rosrun robutler_bringup_23-24 spawn_object.py - l Location -o Object


