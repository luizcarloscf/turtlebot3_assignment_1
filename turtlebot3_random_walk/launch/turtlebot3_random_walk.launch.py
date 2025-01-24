import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_random_walk'),
            'param',
            'turtlebot3_random_walk.yaml',
        ),
    )
    
    turtlebot3_random_walk_node = Node(
        package='turtlebot3_random_walk',
        namespace='',
        executable='turtlebot3_random_walk',
        name='turtlebot3_random_walk',
        parameters=[tb3_param_dir],
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_random_walk_node)
    return ld