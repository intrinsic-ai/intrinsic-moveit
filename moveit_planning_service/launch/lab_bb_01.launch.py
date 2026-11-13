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

"""Launch MoveIt planning service for the lab_bb_01 (UR3e) workcell."""

from pathlib import Path
import sys

_LAUNCH_DIR = str(Path(__file__).resolve().parent)
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from planning_service_common import (  # noqa: E402
    generate_robot_service_launch_description,
)


def generate_launch_description():
    return generate_robot_service_launch_description(
        robot_name="lab_bb_01",
        moveit_config_package="lab_bb_01_moveit_config",
    )
