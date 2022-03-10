# Technex'22 Labyrinth

Solution node for Labyrinth event under Technex'22

## Steps to run Autonomous navigation

### Basic Steps

1. Launch Gazebo  [`roslaunch labyrinth_qual labyrinth_husky_arena.launch`]
2. Launch Rviz  [`rosrun rviz rviz -d custom.rviz`]


### Generating/saving map

1. Gmapping Node  [`roslaunch solution gmapping.launch`]
3. Teleop Node  [`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`]
2. Saving Map  [`rosrun map_server map_saver -f map_name`]


### Qualification Round

1. Install Dependencies  [`python3 -m pip install -r requirements.txt`]
2. Run quals.py  [`roscd solution/src/  -> python3 quals.py`]


### Autonomous navigation

1. Publish aruco ( rostopic pub /aruco std_msgs/Int8 "data: 0" )
2. run spy node ( rosrun spy spy.py )
3. run solution node (roscd solution/src/ -> python3 solution.py )