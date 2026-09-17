// Copyright 2026 Intrinsic Innovation LLC

#include "moveit_planning_service/moveit_scene_synchronizer.hpp"

#include <gtest/gtest.h>

#include <algorithm>

#include "rclcpp/rclcpp.hpp"

namespace moveit_planning_service {
namespace {

class MoveitSceneSynchronizerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(MoveitSceneSynchronizerTest,
       StripTfPrefixesMatchesFirstApplicablePrefix) {
  const std::vector<std::string> prefixes = {"/flowstate/", "/intrinsic/", ""};

  EXPECT_EQ(MoveitSceneSynchronizer::StripTfPrefixes(
                "/flowstate/ur_module/base_link", prefixes),
            "ur_module/base_link");
  EXPECT_EQ(MoveitSceneSynchronizer::StripTfPrefixes("/intrinsic/table/surface",
                                                     prefixes),
            "table/surface");
  EXPECT_EQ(
      MoveitSceneSynchronizer::StripTfPrefixes("ur_module/base_link", prefixes),
      "ur_module/base_link");
  EXPECT_EQ(MoveitSceneSynchronizer::StripTfPrefixes("", prefixes), "");
}

TEST_F(MoveitSceneSynchronizerTest, StripTfPrefixesWithEmptyPrefixList) {
  const std::vector<std::string> empty_prefixes = {};
  EXPECT_EQ(
      MoveitSceneSynchronizer::StripTfPrefixes("some_frame", empty_prefixes),
      "some_frame");
}

TEST_F(MoveitSceneSynchronizerTest, UninitializedStateSafeOperations) {
  MoveitSceneSynchronizer synchronizer;

  EXPECT_EQ(synchronizer.getTrackedCollisionObjectsCount(), 0u);

  const auto status = synchronizer.fetchAndSynchronizeCollisionObjects();
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), absl::StatusCode::kFailedPrecondition);

  // Calling modifying methods before initialization should not throw or crash
  EXPECT_NO_THROW(synchronizer.addCollisionObjectsToScene());
  EXPECT_NO_THROW(synchronizer.removeCollisionObject("non_existent_frame"));
  EXPECT_NO_THROW(synchronizer.updateCollisionObjects({}));
}

TEST_F(MoveitSceneSynchronizerTest, ConfigDefaultValues) {
  MoveitSceneSynchronizerConfig config;
  EXPECT_EQ(config.tf_prefix, "");
  EXPECT_EQ(config.strip_tf_prefixes.size(), 1u);
  EXPECT_EQ(config.strip_tf_prefixes[0], "");
  EXPECT_GE(config.excluded_collision_namespaces.size(), 3u);
  EXPECT_TRUE(std::find(config.excluded_collision_namespaces.begin(),
                        config.excluded_collision_namespaces.end(),
                        "gripper") !=
              config.excluded_collision_namespaces.end());
  EXPECT_TRUE(std::find(config.excluded_collision_namespaces.begin(),
                        config.excluded_collision_namespaces.end(),
                        "camera_mount") !=
              config.excluded_collision_namespaces.end());
  EXPECT_TRUE(std::find(config.excluded_collision_namespaces.begin(),
                        config.excluded_collision_namespaces.end(),
                        "orbbec_camera") !=
              config.excluded_collision_namespaces.end());
  EXPECT_DOUBLE_EQ(config.collision_objects_update_rate_hz, 10.0);
  EXPECT_DOUBLE_EQ(config.collision_objects_min_position_delta, 0.001);
  EXPECT_DOUBLE_EQ(config.collision_objects_min_rotation_delta, 0.01);
}

TEST_F(MoveitSceneSynchronizerTest, InitializeFailsWithNullNodeOrWorldClient) {
  MoveitSceneSynchronizer synchronizer;
  MoveitSceneSynchronizerConfig config;

  // Null node and null world client
  EXPECT_FALSE(synchronizer.initialize(nullptr, nullptr, config));

  // Valid node, null world client
  auto node = std::make_shared<rclcpp::Node>("test_synchronizer_node");
  EXPECT_FALSE(synchronizer.initialize(node, nullptr, config));
}

}  // namespace
}  // namespace moveit_planning_service
