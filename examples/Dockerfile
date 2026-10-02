FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rmw-fastrtps-cpp ros-foxy-rosidl-default-generators ros-foxy-std-srvs \
    && rm -rf /var/lib/apt/lists/*
# 1.16.3 is the last onnxruntime wheel for Python 3.8 on linux/arm64.
# The walker graph is kept at ONNX IR 8 so this runtime can load it.
RUN python3 -m pip install --no-cache-dir \
    numpy==1.24.4 onnxruntime==1.16.3 typing_extensions==4.12.2
WORKDIR /opt/balance
COPY interfaces /opt/balance/interfaces
RUN source /opt/ros/foxy/setup.bash \
    && colcon build --base-paths interfaces --packages-select him_safety_interfaces
COPY unitree_sdk2py /opt/balance/unitree_sdk2py
COPY cyclonedds /opt/balance/cyclonedds
COPY ddsc /opt/balance/ddsc
ENV LD_LIBRARY_PATH=/opt/balance/ddsc
COPY balance_node.py policy.py walker.onnx walker.onnx.data walker.yaml stand.yaml circle.yaml entrypoint.sh /opt/balance/
ENV PYTHONPATH=/opt/balance
ENTRYPOINT ["/bin/bash", "/opt/balance/entrypoint.sh"]
