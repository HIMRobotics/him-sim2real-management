# Simulation-only clients for sim.sh --example.
FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir numpy==1.24.4 onnxruntime==1.16.3 PyYAML==6.0.2 onnx==1.14.1
WORKDIR /example
COPY interfaces /example/interfaces
RUN source /opt/ros/foxy/setup.bash && colcon build --base-paths interfaces \
    --packages-select him_safety_interfaces
COPY controllers.py client.py entrypoint.sh /example/
COPY robot_safety /example/robot_safety
COPY policy_safety /example/policy_safety
COPY project /example/project
ENV G1_SIM_EXAMPLE=walker
# Foxy's Python 3.8 wheel is onnxruntime 1.16 (max IR 9). Newer PyTorch exports
# can mark IR 10 without using IR 10 features; cap so this image still loads.
RUN python3 - <<'PY'
from pathlib import Path
import onnx
path = Path("/example/project/walker.onnx")
if path.is_file():
    model = onnx.load(str(path), load_external_data=False)
    if model.ir_version > 8:
        model.ir_version = 8
        onnx.save(model, str(path))
PY
ENTRYPOINT ["/bin/bash", "/example/entrypoint.sh"]
