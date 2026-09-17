// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_planning_interfaces/srv/plan_grasps.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "absl/flags/flag.h"
#include "flowstate_ros_bridge/world.hpp"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "moveit_planning_service.pb.h"
#include "moveit_planning_service/grasp_planning_pipeline.hpp"
#include "moveit_planning_service/moveit_scene_synchronizer.hpp"
#include "moveit_planning_service/publish_static_transforms.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

using PlanGrasps = moveit_planning_interfaces::srv::PlanGrasps;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
using PlanningSceneClient = rclcpp::Client<GetPlanningScene>;

/**
 * @brief Handles motion planning requests.
 */
void handle_motion_plan_request(
    const std::shared_ptr<std::atomic<bool>>& scene_ready,
    const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request>& request,
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response>& response) {
  if (!scene_ready->load()) {
    RCLCPP_WARN(
        rclcpp::get_logger("moveit_planning_service"),
        "Rejecting motion planning request: collision scene is not ready yet.");
    response->motion_plan_response.error_code.val =
        moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Received motion planning request for group: '%s'",
              request->motion_plan_request.group_name.c_str());

  // TODO: implement actual free-space or Cartesian motion planning, replace
  // dummy code
  response->motion_plan_response.group_name =
      request->motion_plan_request.group_name;
  response->motion_plan_response.planning_time =
      0.05;  // 50ms dummy planning time
  response->motion_plan_response.error_code.val =
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Successfully generated dummy motion plan.");
}

double get_double_parameter(const rclcpp::Node::SharedPtr& node,
                            const std::string& name, double default_value) {
  if (!node->has_parameter(name)) {
    return default_value;
  }
  const auto& param = node->get_parameter(name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return param.as_double();
  } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param.as_int());
  }
  return default_value;
}

/**
 * @brief Coordinates in-process collision scene synchronization and verifies
 * MoveIt's planning scene until populated.
 */
std::thread orchestrate_scene_synchronization(
    const rclcpp::Node::SharedPtr& node,
    const PlanningSceneClient::SharedPtr& planning_scene_client,
    const std::shared_ptr<moveit_planning_service::MoveitSceneSynchronizer>&
        scene_synchronizer,
    const std::shared_ptr<std::atomic<bool>>& scene_ready) {
  const double ros_service_call_timeout_sec =
      get_double_parameter(node, "ros_service_call_timeout_sec", 5.0);
  const double retry_interval_sec =
      get_double_parameter(node, "add_collision_retry_interval_sec", 2.0);
  const bool expect_collision_objects =
      node->get_parameter("expect_collision_objects").as_bool();
  const bool use_mock_hardware =
      node->get_parameter("use_mock_hardware").as_bool();

  const auto service_call_timeout = std::chrono::milliseconds(
      static_cast<long long>(ros_service_call_timeout_sec * 1000.0));
  const auto retry_interval = std::chrono::milliseconds(
      static_cast<long long>(retry_interval_sec * 1000.0));

  return std::thread([node, planning_scene_client, scene_synchronizer,
                      scene_ready, service_call_timeout, retry_interval,
                      expect_collision_objects, use_mock_hardware]() {
    if (use_mock_hardware) {
      RCLCPP_INFO(
          node->get_logger(),
          "Running in mock hardware mode. Skipping collision scene wait.");
      scene_ready->store(true);
      return;
    }

    while (rclcpp::ok() && !scene_ready->load()) {
      // Step 1: Ensure MoveIt's move_group node and planning scene service are
      // up
      if (!planning_scene_client->wait_for_service(retry_interval)) {
        RCLCPP_INFO(
            node->get_logger(),
            "Waiting for MoveIt '/get_planning_scene' service (move_group)...");
        continue;
      }

      // Step 2: Trigger in-process scene synchronization if synchronizer is
      // active
      if (scene_synchronizer) {
        RCLCPP_INFO(node->get_logger(),
                    "Synchronizing collision objects to "
                    "MoveIt planning scene in-process...");
        const auto status =
            scene_synchronizer->fetchAndSynchronizeCollisionObjects();
        if (!status.ok()) {
          RCLCPP_WARN(
              node->get_logger(),
              "Failed to fetch collision objects from World service: %s",
              status.ToString().c_str());
        }
      }

      if (!expect_collision_objects) {
        RCLCPP_INFO(node->get_logger(),
                    "'expect_collision_objects' is false. Proceeding without "
                    "collision scene population verification.");
        scene_ready->store(true);
        break;
      }

      // Step 3: Give ROS 2 transport a moment to process collision objects
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // Step 4: Query MoveIt's planning scene to verify collision objects and
      // robot state
      auto scene_req =
          std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      scene_req->components.components =
          moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
          moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;

      auto scene_future = planning_scene_client->async_send_request(scene_req);
      if (scene_future.wait_for(service_call_timeout) ==
          std::future_status::ready) {
        try {
          auto scene_resp = scene_future.get();
          const auto& objects = scene_resp->scene.world.collision_objects;
          const auto& robot_joints =
              scene_resp->scene.robot_state.joint_state.name;

          const bool collision_ok =
              !expect_collision_objects || !objects.empty();
          const bool robot_state_ok =
              use_mock_hardware || !robot_joints.empty();

          if (collision_ok && robot_state_ok) {
            RCLCPP_INFO(node->get_logger(),
                        "Successfully verified MoveIt planning scene ready "
                        "(%zu collision objects, %zu robot state joints)!",
                        objects.size(), robot_joints.size());
            scene_ready->store(true);
            break;
          } else {
            if (!robot_state_ok) {
              RCLCPP_WARN(node->get_logger(),
                          "MoveIt planning scene robot state is not yet "
                          "initialized with joint states. Waiting for "
                          "/joint_states from flowstate_ros_bridge...");
            }
            if (!collision_ok) {
              if (!scene_synchronizer) {
                RCLCPP_WARN(
                    node->get_logger(),
                    "MoveIt planning scene is empty and Scene "
                    "Synchronizer is offline. Retrying in %.1fs... "
                    "(Tip: pass 'use_mock_hardware:=true "
                    "expect_collision_objects:=false' for standalone testing "
                    "without platform services)",
                    retry_interval.count() / 1000.0);
              } else {
                RCLCPP_WARN(node->get_logger(),
                            "MoveIt planning scene is still waiting for "
                            "collision objects. Retrying in %.1fs...",
                            retry_interval.count() / 1000.0);
              }
            }
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(),
                       "Exception querying MoveIt planning scene: %s. Retrying "
                       "in %.1fs...",
                       e.what(), retry_interval.count() / 1000.0);
        }
      } else {
        planning_scene_client->remove_pending_request(scene_future);
        RCLCPP_WARN(
            node->get_logger(),
            "Timeout querying MoveIt planning scene. Retrying in %.1fs...",
            retry_interval.count() / 1000.0);
      }

      std::this_thread::sleep_for(retry_interval);
    }
  });
}

intrinsic_proto::config::RuntimeContext GetRuntimeContext() {
  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    // Return default context for running locally
    std::cerr << "Warning: using default RuntimeContext\n";
  }
  return runtime_context;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto runtime_context = GetRuntimeContext();
  intrinsic::MoveItPlanningServiceConfig config;
  const bool has_runtime_config = runtime_context.config().UnpackTo(&config);
  if (!has_runtime_config) {
    RCLCPP_WARN(
        rclcpp::get_logger("moveit_planning_service"),
        "Failed to unpack MoveItPlanningServiceConfig from runtime_config.pb; "
        "using default parameters or CLI arguments.");
  }

  // Determine Zenoh router address
  std::string router_address = config.zenoh_router_address();
  if (router_address.empty()) {
    router_address = "tcp/localhost:7447";
  }
  absl::SetFlag(&FLAGS_zenoh_router, router_address);
  std::string zenoh_config_override =
      "connect/endpoints=[\"" + router_address + "\"]";
  setenv("ZENOH_CONFIG_OVERRIDE", zenoh_config_override.c_str(), 1);
  RCLCPP_INFO(rclcpp::get_logger("moveit_planning_service"),
              "Zenoh router endpoint configured to: %s",
              router_address.c_str());

  auto node = rclcpp::Node::make_shared("moveit_planning_node");

  rcl_interfaces::msg::ParameterDescriptor double_desc;
  double_desc.dynamic_typing = true;

  node->declare_parameter<std::string>("intrinsic_core_ingress_address",
                                       "localhost:17080");
  node->declare_parameter<std::string>("zenoh_router_address",
                                       "tcp/localhost:7447");
  node->declare_parameter("ros_service_call_timeout_sec",
                          rclcpp::ParameterValue(5.0), double_desc);
  node->declare_parameter("add_collision_retry_interval_sec",
                          rclcpp::ParameterValue(2.0), double_desc);
  node->declare_parameter<bool>("expect_collision_objects", true);
  node->declare_parameter<bool>("use_mock_hardware", false);
  node->declare_parameter<bool>("publish_world_root_tf", true);
  node->declare_parameter<bool>("publish_robot_base_tf", true);
  node->declare_parameter<std::string>("world_tf_prefix", "world");
  node->declare_parameter<std::string>("robot_base_frame_id",
                                       "ur_module/base_link");
  node->declare_parameter<std::string>("tf_prefix", "");
  node->declare_parameter<std::vector<std::string>>(
      "strip_tf_prefixes", std::vector<std::string>{""});
  node->declare_parameter<std::vector<std::string>>(
      "excluded_collision_namespaces",
      std::vector<std::string>{"ur_module", "gripper", "camera_mount",
                               "orbbec_camera",
                               "ecat_ft_hal_module_with_adapters_with_sim"});
  node->declare_parameter<std::vector<std::string>>(
      "override_joint_names",
      std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint",
                               "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                               "wrist_3_joint"});
  node->declare_parameter("collision_objects_update_rate_hz",
                          rclcpp::ParameterValue(10.0), double_desc);
  node->declare_parameter("collision_objects_min_position_delta",
                          rclcpp::ParameterValue(0.001), double_desc);
  node->declare_parameter("collision_objects_min_rotation_delta",
                          rclcpp::ParameterValue(0.01), double_desc);
  node->declare_parameter("world_sync_interval_sec",
                          rclcpp::ParameterValue(2.0), double_desc);

  if (has_runtime_config) {
    if (!config.intrinsic_core_ingress_address().empty()) {
      node->set_parameter(
          rclcpp::Parameter("intrinsic_core_ingress_address",
                            config.intrinsic_core_ingress_address()));
    }
    if (!config.zenoh_router_address().empty()) {
      node->set_parameter(rclcpp::Parameter("zenoh_router_address",
                                            config.zenoh_router_address()));
    }
    if (config.has_ros_service_call_timeout_sec()) {
      node->set_parameter(
          rclcpp::Parameter("ros_service_call_timeout_sec",
                            config.ros_service_call_timeout_sec()));
    }
    if (config.has_add_collision_retry_interval_sec()) {
      node->set_parameter(
          rclcpp::Parameter("add_collision_retry_interval_sec",
                            config.add_collision_retry_interval_sec()));
    }
    if (config.has_expect_collision_objects()) {
      node->set_parameter(rclcpp::Parameter("expect_collision_objects",
                                            config.expect_collision_objects()));
    }
    if (config.has_use_mock_hardware()) {
      node->set_parameter(
          rclcpp::Parameter("use_mock_hardware", config.use_mock_hardware()));
    }
    if (config.has_publish_world_root_tf()) {
      node->set_parameter(rclcpp::Parameter("publish_world_root_tf",
                                            config.publish_world_root_tf()));
    }
    if (config.has_publish_robot_base_tf()) {
      node->set_parameter(rclcpp::Parameter("publish_robot_base_tf",
                                            config.publish_robot_base_tf()));
    }
    if (config.has_world_tf_prefix()) {
      node->set_parameter(
          rclcpp::Parameter("world_tf_prefix", config.world_tf_prefix()));
    }
    if (!config.robot_base_frame_id().empty()) {
      node->set_parameter(rclcpp::Parameter("robot_base_frame_id",
                                            config.robot_base_frame_id()));
    }
    if (config.strip_tf_prefix_size() > 0) {
      std::vector<std::string> prefixes(config.strip_tf_prefix().begin(),
                                        config.strip_tf_prefix().end());
      node->set_parameter(rclcpp::Parameter("strip_tf_prefixes", prefixes));
    }
    if (config.excluded_collision_namespaces_size() > 0) {
      std::vector<std::string> excluded(
          config.excluded_collision_namespaces().begin(),
          config.excluded_collision_namespaces().end());
      node->set_parameter(
          rclcpp::Parameter("excluded_collision_namespaces", excluded));
    }
    if (config.has_collision_objects_update_rate_hz()) {
      node->set_parameter(
          rclcpp::Parameter("collision_objects_update_rate_hz",
                            config.collision_objects_update_rate_hz()));
    }
    if (config.has_collision_objects_min_position_delta()) {
      node->set_parameter(
          rclcpp::Parameter("collision_objects_min_position_delta",
                            config.collision_objects_min_position_delta()));
    }
    if (config.has_collision_objects_min_rotation_delta()) {
      node->set_parameter(
          rclcpp::Parameter("collision_objects_min_rotation_delta",
                            config.collision_objects_min_rotation_delta()));
    }
    if (config.has_world_sync_interval_sec()) {
      node->set_parameter(rclcpp::Parameter("world_sync_interval_sec",
                                            config.world_sync_interval_sec()));
    }
  }

  // Populate strongly-typed config structs from resolved node parameters
  moveit_planning_service::StaticTransformsConfig tf_config;
  tf_config.publish_world_root_tf =
      node->get_parameter("publish_world_root_tf").as_bool();
  tf_config.publish_robot_base_tf =
      node->get_parameter("publish_robot_base_tf").as_bool();
  const std::string world_prefix =
      node->get_parameter("world_tf_prefix").as_string();
  tf_config.world_frame = world_prefix.empty() ? "world" : world_prefix;
  tf_config.robot_base_frame =
      node->get_parameter("robot_base_frame_id").as_string();

  moveit_planning_service::MoveitSceneSynchronizerConfig sync_config;
  sync_config.tf_prefix = node->get_parameter("tf_prefix").as_string();
  sync_config.strip_tf_prefixes =
      node->get_parameter("strip_tf_prefixes").as_string_array();
  sync_config.excluded_collision_namespaces =
      node->get_parameter("excluded_collision_namespaces").as_string_array();
  sync_config.collision_objects_update_rate_hz =
      get_double_parameter(node, "collision_objects_update_rate_hz", 10.0);
  sync_config.collision_objects_min_position_delta =
      get_double_parameter(node, "collision_objects_min_position_delta", 0.001);
  sync_config.collision_objects_min_rotation_delta =
      get_double_parameter(node, "collision_objects_min_rotation_delta", 0.01);
  sync_config.world_sync_interval_sec =
      get_double_parameter(node, "world_sync_interval_sec", 2.0);

  const bool use_mock_hardware =
      node->get_parameter("use_mock_hardware").as_bool();

  // Broadcast static transforms when not running in mock hardware
  // mode. (In mock hardware mode, static_virtual_joint_tfs publishes world ->
  // base_link).
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster =
      nullptr;
  if (!use_mock_hardware) {
    static_broadcaster =
        moveit_planning_service::PublishStaticTransforms(node, tf_config);
  }

  // Initialize Scene Synchronizer if not using mock hardware
  std::shared_ptr<moveit_planning_service::MoveitSceneSynchronizer>
      scene_synchronizer = nullptr;
  if (!use_mock_hardware) {
    std::string ingress_addr =
        node->get_parameter("intrinsic_core_ingress_address").as_string();
    if (ingress_addr.empty()) {
      ingress_addr = "localhost:17080";
    }

    std::string zenoh_router =
        node->get_parameter("zenoh_router_address").as_string();
    if (zenoh_router.empty()) {
      zenoh_router = "tcp/localhost:7447";
    }
    absl::SetFlag(&FLAGS_zenoh_router, zenoh_router);
    std::string zenoh_override = "connect/endpoints=[\"" + zenoh_router + "\"]";
    setenv("ZENOH_CONFIG_OVERRIDE", zenoh_override.c_str(), 1);

    RCLCPP_INFO(node->get_logger(),
                "Connecting to Ingress at '%s' (zenoh: '%s')...",
                ingress_addr.c_str(), zenoh_router.c_str());

    auto pubsub = std::make_shared<intrinsic::PubSub>(node->get_name());
    auto world = std::make_shared<flowstate_ros_bridge::World>(
        pubsub, ingress_addr, ingress_addr);

    auto world_status = world->connect();
    if (!world_status.ok()) {
      RCLCPP_WARN(node->get_logger(),
                  "Failed to connect to Ingress at '%s': %s. "
                  "Operating in offline/standalone mode.",
                  ingress_addr.c_str(), world_status.ToString().c_str());
      RCLCPP_WARN(node->get_logger(),
                  "Platform connection unavailable. For standalone testing "
                  "without platform services, launch with "
                  "'use_mock_hardware:=true expect_collision_objects:=false'.");
    } else {
      scene_synchronizer =
          std::make_shared<moveit_planning_service::MoveitSceneSynchronizer>();
      if (!scene_synchronizer->initialize(node, world, sync_config)) {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to initialize MoveIt Scene Synchronizer!");
      }
    }
  }

  auto scene_ready = std::make_shared<std::atomic<bool>>(false);

  // Dedicated callback group & executor for /get_planning_scene queries
  auto planning_scene_callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

  auto planning_scene_client =
      node->create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene", rclcpp::ServicesQoS(),
          planning_scene_callback_group);

  auto planning_scene_executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  planning_scene_executor->add_callback_group(planning_scene_callback_group,
                                              node->get_node_base_interface());

  std::thread planning_scene_executor_thread(
      [planning_scene_executor]() { planning_scene_executor->spin(); });

  // Dedicated callback group for planning services (automatically added to main
  // executor)
  auto planning_services_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto motion_service = node->create_service<moveit_msgs::srv::GetMotionPlan>(
      "motion_planning/get_motion_plan",
      [scene_synchronizer, scene_ready](
          const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request>
              request,
          std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> response) {
        if (scene_synchronizer) {
          scene_synchronizer->fetchAndSynchronizeCollisionObjects(std::nullopt);
        }
        handle_motion_plan_request(scene_ready, request, response);
      },
      rclcpp::ServicesQoS(), planning_services_callback_group);

  auto grasp_pipeline =
      std::make_shared<moveit_planning_service::GraspPlanningPipeline>(
          node, planning_scene_client);

  auto grasp_service = node->create_service<PlanGrasps>(
      "grasp_planning/plan_grasps",
      [node, grasp_pipeline, scene_synchronizer, scene_ready](
          const PlanGrasps::Request::SharedPtr request,
          PlanGrasps::Response::SharedPtr response) {
        if (!scene_ready->load()) {
          RCLCPP_WARN(node->get_logger(),
                      "Rejecting grasp planning request: "
                      "collision scene is not ready yet.");
          response->error_code.val =
              moveit_msgs::msg::MoveItErrorCodes::FAILURE;
          return;
        }
        if (scene_synchronizer) {
          scene_synchronizer->fetchAndSynchronizeCollisionObjects(std::nullopt);
        }
        double timeout_sec = 5.0;
        node->get_parameter("ros_service_call_timeout_sec", timeout_sec);
        const auto service_call_timeout = std::chrono::milliseconds(
            static_cast<int64_t>(timeout_sec * 1000.0));
        grasp_pipeline->PlanGrasps(request, response, service_call_timeout);
      },
      rclcpp::ServicesQoS(), planning_services_callback_group);

  RCLCPP_INFO(node->get_logger(), "MoveIt Planning Service started.");
  RCLCPP_INFO(node->get_logger(), "Ready to receive requests at:");
  RCLCPP_INFO(node->get_logger(), " - motion_planning/get_motion_plan");
  RCLCPP_INFO(node->get_logger(), " - grasp_planning/plan_grasps");
  RCLCPP_INFO(node->get_logger(), " - ~/sync_collision_objects");

  auto sync_thread = orchestrate_scene_synchronization(
      node, planning_scene_client, scene_synchronizer, scene_ready);

  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Signal shutdown to synchronizer loop and cancel secondary executor
  scene_ready->store(true);
  planning_scene_executor->cancel();

  if (sync_thread.joinable()) {
    sync_thread.join();
  }
  if (planning_scene_executor_thread.joinable()) {
    planning_scene_executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
