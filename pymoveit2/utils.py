import time
from typing import Type

import rclpy
from rclpy.node import Node


def spin_or_wait(node: Node, timeout_sec: float = 1.0) -> None:
    """Advance ``node``'s callbacks while blocking inside a wait loop.

    CRITICAL: if an external executor is already actively spinning this node (e.g.
    GUIDE's ROS2Robot spins it in a background MultiThreadedExecutor), we must NOT
    call ``rclpy.spin_once`` here. ``spin_once`` hijacks the node onto the global
    executor, so two executors then call ``rclpy_wait`` on the same node's handles
    concurrently. That corrupts the wait set ("IndexError: wait set index too big")
    and afterwards the node silently drops all service-client replies -- e.g.
    /PoseRequest is served by the server but the client future never completes.
    When an external executor is spinning, just yield and let it run the callbacks;
    only self-spin when the node is otherwise unmanaged (standalone pymoveit2 use).
    """
    executor = node.executor
    if executor is not None and getattr(executor, "_is_spinning", False):
        time.sleep(0.005)
    else:
        rclpy.spin_once(node, timeout_sec=timeout_sec)


def enum_to_str(enum_class: Type, value: int) -> str:
    """Converts a ROS2 enum value to its string name.

    Args:
        enum_class: The ROS2 message class containing the enum constants.
        value: The integer value of the enum.

    Returns:
        str: The name of the enum constant, or the value with "UNKNOWN NAME" if not found.
    """
    mapping = {}
    # Iterate over all attributes in the enum class
    for attr_name in dir(enum_class):
        # Consider only uppercase attributes (common convention for enums)
        if attr_name.isupper():
            attr_value = getattr(enum_class, attr_name)
            # Check if the attribute is an integer (enum values are typically int)
            if isinstance(attr_value, int):
                mapping[attr_value] = attr_name
    return mapping.get(value, f"{value} :UNKNOWN NAME")
