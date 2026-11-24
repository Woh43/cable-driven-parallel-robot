cmake_minimum_required(VERSION 3.8)
project(ropes_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ropes_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ropes_cable_plugin SHARED src/cable_plugin.cpp)
target_compile_features(ropes_cable_plugin PUBLIC cxx_std_17)
ament_target_dependencies(ropes_cable_plugin gazebo_dev gazebo_ros geometry_msgs rclcpp
  ropes_interfaces sensor_msgs)

ament_python_install_package(${PROJECT_NAME})
install(TARGETS ropes_cable_plugin LIBRARY DESTINATION lib)
install(PROGRAMS scripts/kinematic_simulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world test/test_world.py APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
