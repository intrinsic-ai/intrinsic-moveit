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

import os
from pathlib import Path
import sys

# Set default RMW implementation before rclpy is imported
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')

_TEST_DIR = str(Path(__file__).resolve().parent)
if _TEST_DIR not in sys.path:
    sys.path.insert(0, _TEST_DIR)

from grasp_planning_integration_common import GraspPlanningIntegrationTestBase  # noqa: E402
from grasp_planning_integration_common import (  # noqa: E402
    build_grasp_planning_test_description,
)
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return build_grasp_planning_test_description(robot_config='lab-bb-01')


generate_test_description.__ready_to_test_action_timeout__ = 60.0


class TestGraspPlanningIntegrationLabBb01(GraspPlanningIntegrationTestBase):
    EXPECTED_ROBOT_CONFIG = 'lab_bb_01'
    EXPECTED_ROBOT_NAME = 'lab_bb_01'
    EXPECTED_MESH_SUBDIR = 'package://robot_hardware_descriptions/urdf/ur3e/'
    BOX_1_XYZ = (0.24, -0.14, 0.015)
    BOX_2_XYZ = (0.24, 0.14, 0.015)

    def test_plan_grasps_for_collision_box(self, **kwargs):
        self.run_grasp_planning_for_collision_boxes()
