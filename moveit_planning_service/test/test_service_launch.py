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

"""Tests for per-workcell MoveIt planning service launch files and configs."""

import importlib.util
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import pytest


def _load_launch_module(filename: str):
    launch_path = (
        Path(get_package_share_directory("moveit_planning_service"))
        / "launch"
        / filename
    )
    spec = importlib.util.spec_from_file_location(launch_path.stem, launch_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _walk_entities(context: LaunchContext, entities: list) -> list:
    """Expand declarative launch entities without spawning Node processes."""
    collected = []
    for entity in entities:
        collected.append(entity)
        if isinstance(
            entity,
            (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction),
        ):
            sub_entities = entity.visit(context)
            if sub_entities:
                expanded = []
                for item in sub_entities:
                    if isinstance(item, LaunchDescription):
                        expanded.extend(item.entities)
                    else:
                        expanded.append(item)
                collected.extend(_walk_entities(context, expanded))
    return collected


def _moveit_include_locations(
    context: LaunchContext, entities: list, pkg_name: str
) -> list[str]:
    del context
    pkg_share = str(Path(get_package_share_directory(pkg_name)).resolve())
    pkg_share_raw = get_package_share_directory(pkg_name)
    matched = []
    for entity in entities:
        if isinstance(entity, IncludeLaunchDescription):
            loc = entity.launch_description_source.location
            if isinstance(loc, str) and (
                loc.startswith(pkg_share) or loc.startswith(pkg_share_raw)
            ):
                matched.append(loc)
    return matched


@pytest.mark.parametrize(
    ("launch_file", "expected_pkg"),
    [
        ("omts.launch.py", "omts_moveit_config"),
        ("lab_bb_01.launch.py", "lab_bb_01_moveit_config"),
    ],
)
def test_per_workcell_launch_files(launch_file: str, expected_pkg: str):
    mod = _load_launch_module(launch_file)
    ctx = LaunchContext()
    entities = _walk_entities(ctx, mod.generate_launch_description().entities)

    includes = _moveit_include_locations(ctx, entities, expected_pkg)
    assert len(includes) == 2
    planning_nodes = [
        e
        for e in entities
        if isinstance(e, Node) and e.node_executable == "moveit_planning_node"
    ]
    assert len(planning_nodes) == 1

    # Enabling mock hardware + RViz expands all 5 MoveIt launch includes.
    full_ctx = LaunchContext()
    full_ctx.launch_configurations["headless"] = "false"
    full_ctx.launch_configurations["use_mock_hardware"] = "true"
    full_entities = _walk_entities(
        full_ctx, mod.generate_launch_description().entities
    )
    assert len(_moveit_include_locations(full_ctx, full_entities, expected_pkg)) == 5


@pytest.mark.parametrize(
    ("launch_args", "expected_pkg", "other_pkg"),
    [
        ({}, "omts_moveit_config", "lab_bb_01_moveit_config"),
        (
            {"robot_config": "omts"},
            "omts_moveit_config",
            "lab_bb_01_moveit_config",
        ),
        (
            {"robot_config": "lab_bb_01"},
            "lab_bb_01_moveit_config",
            "omts_moveit_config",
        ),
    ],
)
def test_service_launch_dispatches_workcell_launch(
    launch_args: dict[str, str], expected_pkg: str, other_pkg: str
):
    service_launch = _load_launch_module("service.launch.py")
    ctx = LaunchContext()
    for k, v in launch_args.items():
        ctx.launch_configurations[k] = v

    entities = _walk_entities(
        ctx, service_launch.generate_launch_description().entities
    )
    assert len(_moveit_include_locations(ctx, entities, expected_pkg)) == 2
    assert len(_moveit_include_locations(ctx, entities, other_pkg)) == 0
    planning_nodes = [
        e
        for e in entities
        if isinstance(e, Node) and e.node_executable == "moveit_planning_node"
    ]
    assert len(planning_nodes) == 1


@pytest.mark.parametrize(
    "bad_cfg", ["", "lab-bb-01", "LAB_BB_01", "invalid_cell", "../omts"]
)
def test_service_and_test_launches_reject_invalid_robot_config(bad_cfg: str):
    for launch_file in ("service.launch.py", "grasp_planning_test.launch.py"):
        mod = _load_launch_module(launch_file)
        ctx = LaunchContext()
        ctx.launch_configurations["robot_config"] = bad_cfg
        with pytest.raises(RuntimeError, match="robot_config"):
            _walk_entities(ctx, mod.generate_launch_description().entities)


def test_grasp_planning_test_launch_selects_expected_moveit_config():
    demo_launch = _load_launch_module("grasp_planning_test.launch.py")
    for launch_args, expected_pkg in (
        ({}, "omts_moveit_config"),
        ({"robot_config": "lab_bb_01"}, "lab_bb_01_moveit_config"),
    ):
        ctx = LaunchContext()
        for k, v in launch_args.items():
            ctx.launch_configurations[k] = v
        entities = _walk_entities(
            ctx, demo_launch.generate_launch_description().entities
        )
        assert len(_moveit_include_locations(ctx, entities, expected_pkg)) >= 1


def test_moveit_configs_joint_limits_match_workcells():
    omts_cfg = MoveItConfigsBuilder(
        "omts", package_name="omts_moveit_config"
    ).to_moveit_configs()
    omts_limits = omts_cfg.joint_limits["robot_description_planning"][
        "joint_limits"
    ]
    for wrist in ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint"):
        assert omts_limits[wrist]["max_velocity"] == pytest.approx(
            math.pi, rel=1e-6
        )

    lab_cfg = MoveItConfigsBuilder(
        "lab_bb_01", package_name="lab_bb_01_moveit_config"
    ).to_moveit_configs()
    lab_limits = lab_cfg.joint_limits["robot_description_planning"][
        "joint_limits"
    ]
    for wrist in ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint"):
        assert lab_limits[wrist]["max_velocity"] == pytest.approx(
            2.0 * math.pi, rel=1e-6
        )
    assert lab_limits["wrist_3_joint"]["has_position_limits"] is True
    assert lab_limits["wrist_3_joint"]["min_position"] == pytest.approx(
        -2.0 * math.pi, rel=1e-6
    )
    assert lab_limits["wrist_3_joint"]["max_position"] == pytest.approx(
        2.0 * math.pi, rel=1e-6
    )
