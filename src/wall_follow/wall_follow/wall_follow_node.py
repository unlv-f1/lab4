#!/usr/bin/env python3

from typing import Any

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class WallFollow(Node):
    """
    Implement Wall Following on the car
    """

    def __init__(self):
        super().__init__("wall_follow_node")

        # TODO: Declare parameters

        # TODO: Read parameters
        # self.speed_max =
        # self.distance_setpoint =
        # self.kp =
        # self.kd =
        # self.ki =

        # TODO: Create subscriptions and publishers

        # TODO: Initialize attributes related to PID
        # self.integral =
        # self.prev_error =

        self.get_logger().info("Initialized.")

    def get_parameter_value_checked(
        self, name: str, expected_type: type, check_none: bool = True
    ) -> Any:
        """Helper method to get the value of a parameter, check that it's not
        None, then check that its type is given"""
        value = self.get_parameter(name).value
        if check_none and value is None:
            raise ValueError(f"No value given for parameter {name!r}")
        if not isinstance(value, expected_type):
            raise TypeError(
                f"Given value for parameter {name!r} is not of type "
                f"{expected_type.__name__!r}"
            )
        return value

    def get_error(self, ranges: np.ndarray) -> float:
        """
        Calculates the error to the wall. Remember, you are following the wall
        to your left. (Going counterclockwise in the Levine loop!)

        Args:
            ranges: Range array

        Returns:
            error: calculated error
        """
        # TODO implement
        return 0.0

    def get_speed(self, steering_angle: float) -> float:
        """
        Calculates the speed for the drive message.

        Args:
            steering_angle: Steering angle for the drive message
        Returns:
            speed: Speed of the car
        """
        # TODO: Implement
        return 0.0

    def scan_callback(self, scan_msg: LaserScan) -> None:
        """Called whenever a scan message is received.

        Args:
            scan_msg: Incoming LaserScan message

        Returns:
            None
        """

        ranges = np.array(scan_msg.ranges)

        # TODO: Calculate error
        # TODO: Use PID formula and derive steering_angle
        # TODO: Calculate speed
        # TODO: Publish drive message


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
