from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    turtlebot3_object_classifier_node = Node(
        package="turtlebot3_object_classifier",
        namespace="",
        executable="object_classifier",
        name="turtlebot3_object_classifier",
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_object_classifier_node)
    return ld
