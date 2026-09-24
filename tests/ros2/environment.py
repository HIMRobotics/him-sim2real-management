"""Keep automated ROS tests off the manual robot-control domain."""

import os


def require_test_domain():
    """Reject test execution unless the reserved localhost test domain is selected."""
    if os.environ.get("ROS_DOMAIN_ID") != "191":
        raise RuntimeError("ROS tests require ROS_DOMAIN_ID=191, never manual domain 97")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("ROS tests require ROS_LOCALHOST_ONLY=1")
