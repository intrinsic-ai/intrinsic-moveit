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

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find LTTng-UST using PkgConfig to define target for flowstate_ros_bridge linking
find_package(PkgConfig REQUIRED)
pkg_check_modules(LTTNG_UST REQUIRED lttng-ust IMPORTED_TARGET)

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
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(flowstate_ros_bridge REQUIRED)
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
  src/grasp_planning_pipeline.cpp
  src/publish_static_transforms.cpp
  src/moveit_scene_synchronizer.cpp
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
  absl::flags
  absl::log
  absl::time
  absl::strings
  ament_index_cpp::ament_index_cpp
  Eigen3::Eigen
  geometric_shapes::geometric_shapes
  flowstate_ros_bridge::flowstate_ros_builtin_bridges
  flowstate_ros_bridge::flowstate_ros_bridge_component
  flowstate_ros_bridge::world
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  ${SERVICE_NAME}_protos
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_core_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${moveit_task_constructor_core_TARGETS}
  ${moveit_planning_interfaces_TARGETS}
  tf2_ros::tf2_ros
  PkgConfig::LTTNG_UST
)

# Status monitor library & executable
add_library(status_monitor_lib STATIC
  src/status_monitor.cpp
)
target_include_directories(status_monitor_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(status_monitor_lib
 PUBLIC
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  rclcpp::rclcpp
  absl::strings
  ${SERVICE_NAME}_protos
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_planning_interfaces_TARGETS}
  tf2_ros::tf2_ros
)

add_executable(status_monitor src/status_monitor_main.cpp)
target_include_directories(status_monitor PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(status_monitor
 PUBLIC
  status_monitor_lib
)

# Install
install(TARGETS
  moveit_planning_node
  status_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install proto descriptor for sideloading
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(publish_static_transforms_test
    test/publish_static_transforms_test.cpp
    src/publish_static_transforms.cpp
  )
  target_include_directories(publish_static_transforms_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(publish_static_transforms_test
    rclcpp::rclcpp
    absl::log
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
  )

  ament_add_gtest(moveit_scene_synchronizer_test
    test/moveit_scene_synchronizer_test.cpp
    src/moveit_scene_synchronizer.cpp
  )
  target_include_directories(moveit_scene_synchronizer_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(moveit_scene_synchronizer_test
    rclcpp::rclcpp
    absl::log
    absl::strings
    Eigen3::Eigen
    geometric_shapes::geometric_shapes
    flowstate_ros_bridge::flowstate_ros_builtin_bridges
    flowstate_ros_bridge::flowstate_ros_bridge_component
    flowstate_ros_bridge::world
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    ${SERVICE_NAME}_protos
    ${std_srvs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${shape_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    ${moveit_msgs_TARGETS}
    tf2_ros::tf2_ros
    PkgConfig::LTTNG_UST
  )

  ament_add_gtest(object_geometry_test test/object_geometry_test.cpp src/object_geometry.cpp src/generate_box_grasp_poses.cpp)
  target_include_directories(object_geometry_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(object_geometry_test
    Eigen3::Eigen
    ${moveit_msgs_TARGETS}
    ${moveit_core_TARGETS}
    ${moveit_task_constructor_core_TARGETS}
  )

  ament_add_gtest(grasp_planning_pipeline_test
    test/grasp_planning_pipeline_test.cpp
    src/grasp_planning_pipeline.cpp
    src/object_geometry.cpp
    src/generate_box_grasp_poses.cpp
  )
  target_include_directories(grasp_planning_pipeline_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(grasp_planning_pipeline_test
    rclcpp::rclcpp
    Eigen3::Eigen
    ${moveit_msgs_TARGETS}
    ${moveit_core_TARGETS}
    ${moveit_ros_planning_interface_TARGETS}
    ${moveit_task_constructor_core_TARGETS}
    ${moveit_planning_interfaces_TARGETS}
  )

  ament_add_gtest(status_monitor_test
    test/status_monitor_test.cpp
  )
  target_include_directories(status_monitor_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(status_monitor_test
    status_monitor_lib
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
