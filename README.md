# Assignment 1

This repository includes a solution for the first assignment of a course presented in the [Electrical Engineering] department at the [UFES], by [PhD Ricardo Carminati de Mello] using ROS. 

> The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers and state-of-the-art algorithms to powerful developer tools, ROS has the open source tools you need for your next robotics project. 

This assigment consists of using a mobile robot with a set of sensors in a partially known environment to:

1. identify obstacles in the environment;
2. position and classify the obstacles;
3. report the total number of obstacles in each class within the environment.

To do so, we developed to following ROS2 packages:
* [turtlebot3_assignment_1](./turtlebot3_assignment_1/): Meta package responsible for packing the entire solution;
* [turtlebot3_random_walk](./turtlebot3_random_walk/): Package the contains a node `turtlebot3_random_wakl` responsible for explore a world randomly;
* [turtlebot3_object_classifier](./turtlebot3_object_classifier/): Package the contains a node `turtlebot3_object_classifier` with the object containing and classification logic.


## Installation

First, install the required ROS 2 packages:
```bash
sudo apt install ros-<ros2-distro>-turtlebot3*
```

Then, clone the repository and build the workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/luizcarloscf/turtlebot3_assignment_1.git
cd ~/ros_ws

source /opt/ros/<ros2-distro>/setup.bash
colcon build
source install/setup.bash
```
Ensure you have all necessary Python dependencies installed:
```bash
pip3 install -r src/turtlebot3_assigment_1/requirements.txt
```

## Configuration

Before running the simulation, set the necessary environment variables:
```bash
export TURTLEBOT3_MODEL=waffle 
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models:$(pwd)/src/turtlebot3_assignment_1/turtlebot3_assignment_1/models
```

## Running 
In the same terminal, execute the following command to generate a random pose and launch the simulation:
```bash
export RANDOM_POSE=$(ros2 run turtlebot3_assignment_1 random_pose.py -g 4 -w 2)
ros2 launch turtlebot3_assignment_1 turtlebot3_assignment_1.launch.py world_group:=4 world_number:=2 $RANDOM_POSE
```
## Notes

* Ensure that you replace `<ros2-distro>` with your installed ROS 2 distribution (e.g., `humble`, `foxy`).
* If you encounter any dependency issues, verify that all required packages are installed and sourced properly.
* The `random_pose.py` script generates a random starting position for the TurtleBot3 in the simulation.

# Troubleshooting

* If Gazebo does not launch properly, check if all environment variables are correctly set.
* If the build fails, ensure that ROS 2 is correctly installed and sourced.

For additional support, refer to the official [ROS 2 documentation](https://docs.ros.org/en/humble/index.html).


[Electrical Engineering]: https://ele.ufes.br/
[UFES]: https://www.ufes.br/
[ROS]: https://docs.ros.org/en/humble/index.html
[PhD Ricardo Carminati de Mello]: http://lattes.cnpq.br/1569638571582691