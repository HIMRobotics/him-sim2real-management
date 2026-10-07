# Shared clients for simulation and explicitly enabled hardware execution.
FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp ros-foxy-std-srvs \
    && rm -rf /var/lib/apt/lists/*
# Match the Python 3.8 ARM64 runtime used by the walker submission image.
RUN python3 -m pip install --no-cache-dir numpy==1.24.4 onnxruntime==1.16.3 PyYAML==6.0.2
WORKDIR /example
COPY interfaces /example/interfaces
RUN source /opt/ros/foxy/setup.bash && colcon build --base-paths interfaces \
    --packages-select him_safety_interfaces
COPY controllers.py client.py trial.py entrypoint.sh /example/
COPY robot_safety /example/robot_safety
COPY policy_safety /example/policy_safety
COPY project /example/project
RUN python3 -c "from pathlib import Path; from controllers import ExamplePolicy; ExamplePolicy('walker' if Path('project/walker.onnx').is_file() else 'balance', 'project')"
ENTRYPOINT ["/bin/bash", "/example/entrypoint.sh"]
