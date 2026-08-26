# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(moveit_planning_service)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(absl REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(intrinsic_sdk_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_planning_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(SERVICE_NAME "moveit_planning_service")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)

# Executables
add_executable(moveit_planning_node
  src/moveit_planning_node.cpp
  src/generate_box_grasp_poses.cpp
  src/object_geometry.cpp
)
target_include_directories(moveit_planning_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(moveit_planning_node
  rclcpp::rclcpp
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  ${SERVICE_NAME}_protos
  ${std_srvs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_core_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${moveit_task_constructor_core_TARGETS}
  ${moveit_planning_interfaces_TARGETS}
)

# Status monitor
add_executable(status_monitor src/status_monitor.cpp)
target_link_libraries(status_monitor
 PUBLIC
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_planning_interfaces_TARGETS}
  tf2_ros::tf2_ros
)

# Install
install(TARGETS
  moveit_planning_node
  status_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install proto descriptor to sideload as a Flowstate service
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(object_geometry_test test/object_geometry_test.cpp src/object_geometry.cpp src/generate_box_grasp_poses.cpp)
  target_include_directories(object_geometry_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(object_geometry_test
    Eigen3::Eigen
    ${moveit_msgs_TARGETS}
    ${moveit_core_TARGETS}
    ${moveit_task_constructor_core_TARGETS}
  )

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_grasp_planning_integration.test.py
    ENV
      RMW_IMPLEMENTATION=rmw_zenoh_cpp
    TIMEOUT 120
  )
endif(BUILD_TESTING)



ament_package()
