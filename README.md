# Assignment 1

First, install dependencies:
```bash
sudo apt install ros-<ros2-distro>-turtlebot3*
```

Set key environment variables:
```bash
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models
```
In the same terminal, run:
```bash
ros2 launch  turtlebot3_assignment_1 turtlebot3_assignment_1.launch.py 
```

Then, open another terminal and run to enable random walk:
```bash
source /opt/ros/<ros2-distro>/setup.bash
ros2 service call /enable std_srvs/srv/SetBool '{data: False}'
```
