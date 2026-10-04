# Walker test submission for the simulator, not a general deployment image.
FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rmw-fastrtps-cpp ros-foxy-rosidl-default-generators ros-foxy-std-srvs \
    && rm -rf /var/lib/apt/lists/*
# 1.16.3 is the last onnxruntime wheel for Python 3.8 on linux/arm64.
# The walker graph is ONNX IR 9 / opset 17, which that runtime loads.
RUN python3 -m pip install --no-cache-dir \
    numpy==1.24.4 onnxruntime==1.16.3 typing_extensions==4.12.2 PyYAML==6.0.2
WORKDIR /opt/walker
COPY interfaces /opt/walker/interfaces
RUN source /opt/ros/foxy/setup.bash \
    && colcon build --base-paths interfaces --packages-select him_safety_interfaces
COPY unitree_sdk2py /opt/walker/unitree_sdk2py
COPY cyclonedds /opt/walker/cyclonedds
COPY ddsc /opt/walker/ddsc
ENV LD_LIBRARY_PATH=/opt/walker/ddsc
COPY walker_node.py policy.py walker.onnx walker.onnx.data walker.yaml stand.yaml circle.yaml entrypoint.sh /opt/walker/
COPY policy_safety /opt/walker/policy_safety
ENV PYTHONPATH=/opt/walker
RUN python3 -c "from policy import WalkerPolicy; p = WalkerPolicy('walker.onnx', 'walker.yaml', 'stand.yaml'); p.targets(p.walker.default, [0.0]*29, [1.0,0.0,0.0,0.0], [0.0]*6)"
ENTRYPOINT ["/bin/bash", "/opt/walker/entrypoint.sh"]
