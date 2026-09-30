FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir numpy==1.24.4 onnxruntime==1.18.1 PyYAML==6.0.2
WORKDIR /example
COPY interfaces /example/interfaces
RUN source /opt/ros/foxy/setup.bash && colcon build --base-paths interfaces \
    --packages-select him_safety_interfaces
COPY controllers.py client.py entrypoint.sh /example/
COPY robot_safety /example/robot_safety
COPY policy_safety /example/policy_safety
COPY project /example/project
ENTRYPOINT ["/bin/bash", "/example/entrypoint.sh"]
