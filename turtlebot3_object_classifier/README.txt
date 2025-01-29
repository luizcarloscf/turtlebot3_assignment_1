source /opt/ros/humble/setup.bash
cd ros_ws/
source install/setup.bash


export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/huble/share/turtlebot3_gazebo/models:$(pwd)/src/turtlebot3_assignment_1/turtlebot3_assignment_1/models



ros2 launch turtlebot3_assignment_1 turtlebot3_assignment_1.launch.py
ros2 service call /enable std_srvs/srv/SetBool '{data: True}'
ros2 run turtlebot3_object_classifier object_classifier



