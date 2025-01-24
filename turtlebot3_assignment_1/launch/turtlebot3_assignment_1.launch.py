import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    turtlebot3_random_walk_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_random_walk'),
        'launch',
    )
    turtlebot3_gazebo_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    turtlebot3_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_launch_dir,
                'turtlebot3_world.launch.py',
            ),
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    turtlebot3_random_walk_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_random_walk_launch_dir,
                'turtlebot3_random_walk.launch.py',
            ),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_world_cmd)
    ld.add_action(turtlebot3_random_walk_cmd)
    return ld