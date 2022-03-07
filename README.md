# Technex'22 Labyrinth

Solution node for Labyrinth event under Technex'22

## Steps to run Autonomous navigation

### Basic Steps

1. Launch Gazebo
2. Launch Rviz


### Generating/saving map

1. rosrun gmapping slam_gmapping scan:scan odam_frame:odom
3. run teleop node
2. rosrun map_server map_saver -f map_name


### Autonomous navigation

1. Publish aruco ( rostopic pub /aruco std_msgs/Int8 "data: 0" )
2. run spy node ( rosrun spy spy.py )
3. run solution node (roscd solution/src/ -> python3 solution.py )