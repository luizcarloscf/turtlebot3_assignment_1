import rclpy
from rclpy.executors import ExternalShutdownException

from turtlebot3_random_walk.turtlebot3_random_walk \
    import Turtlebot3RandomWalk


def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = Turtlebot3RandomWalk()
    try:
        rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()