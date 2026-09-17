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
project(moveit_plan_grasp_skill)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(abseil_cpp_vendor REQUIRED)
find_package(absl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_planning_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# Intrinsic Dependencies
find_package(intrinsic_sdk_cmake REQUIRED)

# Create Intrinsic Skill
intrinsic_sdk_protobuf_generate(
  NAME moveit_plan_grasp_skill
  SOURCES src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.proto
  TARGET moveit_plan_grasp_skill_protos
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_protos.desc"
)

# Generate the skill config file.
intrinsic_sdk_generate_skill_config(
  TARGET moveit_plan_grasp_skill_config
  SKILL_NAME moveit_plan_grasp_skill
  MANIFEST src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.manifest.textproto
  PROTO_DESCRIPTOR_FILE "${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_protos.desc"
  SKILL_CONFIG_FILE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_config.pbbin"
)

# Generate the main function for this skill
intrinsic_sdk_generate_skill_main_cc(
  MANIFEST src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.manifest.textproto
  HEADER_FILES src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.h
  MAIN_FILE_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_main.cc
)

install(
  FILES
    src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.manifest.textproto
    "${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_config.pbbin"
    "${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_protos.desc"
  DESTINATION share/${PROJECT_NAME}
)

add_library(moveit_plan_grasp_skill
  src/moveit_plan_grasp_skill/moveit_plan_grasp_skill.cpp
)

target_include_directories(moveit_plan_grasp_skill
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/moveit_plan_grasp_skill
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/src/moveit_plan_grasp_skill
)

target_link_libraries(moveit_plan_grasp_skill
  PUBLIC
  intrinsic_sdk_cmake::intrinsic_sdk_cmake
  moveit_plan_grasp_skill_protos
  rclcpp::rclcpp
  absl::log
  absl::strings
  ${moveit_msgs_TARGETS}
  ${moveit_planning_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_dependencies(moveit_plan_grasp_skill moveit_plan_grasp_skill_config)

add_executable(moveit_plan_grasp_skill_main
  ${CMAKE_CURRENT_BINARY_DIR}/moveit_plan_grasp_skill_main.cc
)

target_link_libraries(moveit_plan_grasp_skill_main
  moveit_plan_grasp_skill
)

install(
  TARGETS moveit_plan_grasp_skill moveit_plan_grasp_skill_main
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(moveit_plan_grasp_skill_test
    test/moveit_plan_grasp_skill_test.cpp
  )
  target_link_libraries(moveit_plan_grasp_skill_test
    moveit_plan_grasp_skill
  )
endif()

# Export dependencies for downstream
ament_package()
