# Assignment 1

First, install dependencies:
```bash
sudo apt install ros-<ros2-distro>-turtlebot3*
```

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/luizcarloscf/turtlebot3_assignment_1.git
cd ~/ros_ws

source /opt/ros/<ros2-distro>/setup.bash
colcon build
source install/setup.bash
```
Set key environment variables:
```bash
export TURTLEBOT3_MODEL=waffle 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models:$(pwd)/src/turtlebot3_assignment_1/turtlebot3_assignment_1/models
```

In the same terminal, run:
```bash
export RANDOM_POSE=$(ros2 run turtlebot3_assignment_1 random_pose.py -g 4 -w 2)
ros2 launch turtlebot3_assignment_1 turtlebot3_assignment_1.launch.py world_group:=4 world_number:=2 $RANDOM_POSE
```

Then, open another terminal and run to enable random walk:
```bash
source /opt/ros/<ros2-distro>/setup.bash
ros2 service call /enable std_srvs/srv/SetBool '{data: True}'
```

Then, in terminal run to start classifier:
```bash
ros2 run turtlebot3_object_classifier object_classifier
```
