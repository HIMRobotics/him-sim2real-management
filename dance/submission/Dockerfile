FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
# OpenBLAS worker threads stall for tens of ms when the cores are shared with
# the safety stack, which misses the 200 ms command deadline. The MLP is tiny.
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp ros-foxy-std-srvs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /submission
COPY interfaces /submission/interfaces
RUN source /opt/ros/foxy/setup.bash && colcon build --base-paths interfaces \
    --packages-select him_safety_interfaces
COPY requirements.txt /submission/
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY assets /submission/assets
COPY policy.py node.py entrypoint.sh /submission/
RUN python3 -c "from pathlib import Path; from policy import Policy; \
q = Policy(Path('assets')).act({'q': [0.0] * 29, 'dq': [0.0] * 29, 'quat': [1.0, 0.0, 0.0, 0.0], \
'gyro': [0.0] * 3, 'accel': [0.0, 0.0, 9.81], 'observation_id': 1}); \
assert len(q) == 29, 'Policy.act must return 29 targets'"
ENTRYPOINT ["/bin/bash", "/submission/entrypoint.sh"]
