// Copyright 2026 Intrinsic Innovation LLC

#ifndef MOVEIT_PLANNING_SERVICE_GRASP_PLANNING_PIPELINE_HPP_
#define MOVEIT_PLANNING_SERVICE_GRASP_PLANNING_PIPELINE_HPP_

#include <chrono>
#include <memory>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_planning_interfaces/srv/plan_grasps.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace moveit_planning_service {

class GraspPlanningPipeline {
 public:
  using PlanGraspsSrv = moveit_planning_interfaces::srv::PlanGrasps;
  using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;
  using PlanningSceneClient = rclcpp::Client<GetPlanningScene>;

  explicit GraspPlanningPipeline(
      rclcpp::Node::SharedPtr node,
      PlanningSceneClient::SharedPtr planning_scene_client,
      moveit::core::RobotModelConstPtr robot_model = nullptr);

  /**
   * @brief Returns the cached robot model, loading it if not yet initialized.
   */
  moveit::core::RobotModelConstPtr GetRobotModel();

  /**
   * @brief Executes MTC-based grasp candidate generation and motion planning.
   *
   * @param request Grasp planning service request.
   * @param response Grasp planning service response to populate.
   * @param service_call_timeout Timeout for internal ROS service queries (e.g.
   * GetPlanningScene).
   */
  void PlanGrasps(const std::shared_ptr<PlanGraspsSrv::Request>& request,
                  std::shared_ptr<PlanGraspsSrv::Response>& response,
                  std::chrono::milliseconds service_call_timeout =
                      std::chrono::milliseconds(5000));

  /**
   * @brief Resolves an incoming target object ID against existing collision
   * objects in the MoveIt planning scene.
   *
   * Resolves target IDs through the following priority order:
   * 1. Exact match against existing collision object IDs.
   * 2. Leading slash stripped.
   * 3. Canonical entity suffix (<clean_id>/whole).
   * 4. Prefix match against any <clean_id>/<entity>.
   */
  static std::string ResolveTargetObjectId(
      const std::vector<std::string>& existing_object_ids,
      const std::string& target_id);

  /**
   * @brief Queries MoveIt's planning scene service for the list of existing
   * world collision object names.
   */
  std::vector<std::string> GetPlanningSceneObjectIds(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) const;

  /**
   * @brief Retrieves a collision object and its geometry from the planning
   * scene.
   */
  std::optional<moveit_msgs::msg::CollisionObject>
  GetPlanningSceneCollisionObject(
      const std::string& object_id,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) const;

 private:
  rclcpp::Node::SharedPtr node_;
  PlanningSceneClient::SharedPtr planning_scene_client_;
  mutable std::mutex robot_model_mutex_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
};

}  // namespace moveit_planning_service

#endif  // MOVEIT_PLANNING_SERVICE_GRASP_PLANNING_PIPELINE_HPP_
