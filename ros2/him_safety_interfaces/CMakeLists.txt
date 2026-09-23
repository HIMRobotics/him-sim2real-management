cmake_minimum_required(VERSION 3.5)
project(him_safety_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/CheckedCommand.msg"
  "msg/RobotFeedback.msg"
  "msg/Abort.msg"
  "msg/DryRunOutput.msg"
  "srv/ArmGate.srv"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
