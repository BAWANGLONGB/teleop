"""ROS2 PICO source and raw data collection support."""


def spin_until_stopped(executor, context, stop_event):
    """Handle context shutdown racing a background executor's wait-set creation."""
    from rclpy.executors import ExternalShutdownException, ShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation

    try:
        while not stop_event.is_set() and context.ok():
            executor.spin_once(timeout_sec=0.02)
    except (ExternalShutdownException, ShutdownException):
        return
    except rclpy_implementation.RCLError:
        if context.ok():
            raise
