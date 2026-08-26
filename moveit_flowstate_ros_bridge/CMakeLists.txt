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
project(moveit_flowstate_ros_bridge)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)

# Find LTTng-UST using PkgConfig to define target for flowstate_ros_bridge linking
find_package(PkgConfig REQUIRED)
pkg_check_modules(LTTNG_UST REQUIRED lttng-ust IMPORTED_TARGET)

set(dep_pkgs
  absl
  ament_index_cpp
  flowstate_ros_bridge
  lifecycle_msgs
  pluginlib
  rclcpp
  std_srvs
  moveit_msgs
  geometric_shapes
  shape_msgs
  geometry_msgs
  tf2_msgs
  tf2_ros
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)

#==================================

add_library(moveit_scene_bridge SHARED
  src/moveit_scene_bridge.cpp
)
target_include_directories(moveit_scene_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(moveit_scene_bridge
  absl::log
  absl::time
  ament_index_cpp::ament_index_cpp
  Eigen3::Eigen
  flowstate_ros_bridge::flowstate_ros_builtin_bridges
  pluginlib::pluginlib
  rclcpp::rclcpp
  geometric_shapes::geometric_shapes
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
target_compile_features(moveit_scene_bridge PUBLIC cxx_std_20)

ament_export_libraries(moveit_scene_bridge)

#===============================================================================

pluginlib_export_plugin_description_file(flowstate_ros_bridge plugins.xml)

#===============================================================================

set(SERVICE_NAME "moveit_flowstate_ros_bridge")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)
add_executable(${SERVICE_NAME}_main
 src/${SERVICE_NAME}_main.cpp
)
target_include_directories(${SERVICE_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${SERVICE_NAME}_main
 PUBLIC
  flowstate_ros_bridge::flowstate_ros_bridge_component
  ${SERVICE_NAME}_protos
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  tf2_ros::tf2_ros
)

# Status monitor
add_executable(status_monitor
  src/status_monitor.cpp
)
target_link_libraries(status_monitor
 PUBLIC
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  rclcpp::rclcpp
  ${lifecycle_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

#===============================================================================

install(TARGETS
  moveit_scene_bridge
  ${SERVICE_NAME}_main
  status_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install proto descriptor file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rviz_http_proxy
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
