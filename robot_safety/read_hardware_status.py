"""Read one complete hardware status message without publishing commands."""

import json
import sys
import time


def read_status(timeout_secs=3.0):
    """Subscribe once using the ROS Foxy API and a monotonic deadline."""
    import rclpy
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("hardware_status_reader")
        messages = []
        node.create_subscription(String, "/safety/hardware_status", messages.append, 1)
        deadline = time.monotonic() + timeout_secs
        while not messages:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("No hardware status received within %.1f seconds" % timeout_secs)
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
        status = json.loads(messages[0].data)
        if not isinstance(status, dict):
            raise ValueError("Hardware status must be a JSON object")
        return status
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main():
    try:
        print(json.dumps(read_status(), allow_nan=False))
    except Exception as error:
        print("Cannot read safety-service status: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
