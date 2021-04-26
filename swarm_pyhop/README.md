## Requirement

- Python 3.8.5
- ROS Noectic
- Turtlebot 3

## How to run

1. paste project folder in src in catkin workspace
2. open terminal in catkin workspace
3. source devel/setup.bash
5. export TURTLEBOT3_MODEL=burger
4. roslaunch swarm_pyhop program.launch x_pos:=1 y_pos:=3 theta:=0 x_pos_f:=4 y_pos_f:=4 clearance:=0 rpm1:=5 rpm2:=10

## Notes
As the code starts the exploration and visualization starts.Once the path is found u will see 2nd image poping up with path in dark blue press q on the image so that the turtlebot path following will start

if u want to use it in ros melodic u can change the interpreter from #!/usr/bin/python3 to #!/usr/bin/python in the file main.py

it is recommended to use in  ros noectic

for non ros file named withou_ros_main execute it 
