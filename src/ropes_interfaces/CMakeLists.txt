cmake_minimum_required(VERSION 3.8)
project(ropes_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowCartesianPath.action"
  "msg/CableTensions.msg"
  "msg/HardwareStatus.msg"
  "srv/ComputeForwardKinematics.srv"
  "srv/ComputeInverseKinematics.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
