# WORK IN PROGRESS .... 

First step is to read the jupyter notebook provided in the docs folder.

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 launch turtlebot3_cartographer cartographer.launch.py

ros2 launch turtlebot3_cartographer cartographer_rviz.launch.py

ros2 run nav2_map_server map_saver_cli -f ./map

rviz should open before running the map server with correct confifurationp 

the map_server and amcl neededs to launched with nav2_lifecycle node
 
ros2 launch turtlebot3_localization nav2_amcl.launch.py # to launch the amcl with mapserver and rviz 