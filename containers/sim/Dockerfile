FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
COPY --from=ghcr.io/astral-sh/uv:0.8.22 /uv /usr/local/bin/uv
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
COPY ros2 /tmp/ros2
RUN source /opt/ros/foxy/setup.bash \
    && colcon build --packages-select him_safety_interfaces \
       --base-paths /tmp/ros2 \
       --build-base /opt/him_safety/build \
       --install-base /opt/him_safety/install \
    && rm -rf /tmp/ros2
RUN uv python install 3.12 \
    && uv venv /opt/plant --python 3.12 \
    && uv pip install --python /opt/plant/bin/python --no-cache \
       "numpy>=1.26,<3" "mujoco>=3.3.2,<3.4" "onnxruntime>=1.22,<2" \
       "onnx>=1.17,<2" "pyyaml>=6,<7"
COPY containers/sim/desktop_entry.sh /usr/local/bin/desktop_entry.sh
RUN chmod +x /usr/local/bin/desktop_entry.sh
WORKDIR /workspace
ENV G1_SIM_PYTHON=/opt/plant/bin/python
ENTRYPOINT ["/usr/local/bin/desktop_entry.sh"]
