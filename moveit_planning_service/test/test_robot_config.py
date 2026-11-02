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

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
import pytest

from moveit_planning_service.robot_config import DEFAULT_ROBOT_CONFIG
from moveit_planning_service.robot_config import SUPPORTED_ROBOT_CONFIGS
from moveit_planning_service.robot_config import default_descriptor_paths
from moveit_planning_service.robot_config import (
    load_runtime_config_message_classes,
)
from moveit_planning_service.robot_config import normalize_robot_config
from moveit_planning_service.robot_config import read_robot_config_from_runtime_pb
from moveit_planning_service.robot_config import resolve_robot_config


def _write_runtime_context_pb(
    target_path: Path,
    robot_config_value: str | None = None,
    raw_any_type_url: str | None = None,
    raw_any_value: bytes = b"",
) -> Path:
    desc_paths = default_descriptor_paths()
    assert len(desc_paths) == 2, f"Expected 2 .desc files, got {desc_paths}"
    runtime_ctx_cls, service_cfg_cls = load_runtime_config_message_classes(
        desc_paths
    )

    ctx_msg = runtime_ctx_cls()
    if raw_any_type_url is not None:
        ctx_msg.config.type_url = raw_any_type_url
        ctx_msg.config.value = raw_any_value
    else:
        cfg_msg = service_cfg_cls()
        if robot_config_value is not None:
            cfg_msg.robot_config = robot_config_value
        ctx_msg.config.Pack(cfg_msg)

    target_path.write_bytes(ctx_msg.SerializeToString())
    return target_path


def _load_launch_module(launch_filename: str):
    share_dir = Path(get_package_share_directory("moveit_planning_service"))
    module_path = share_dir / "launch" / launch_filename
    spec = importlib.util.spec_from_file_location(
        launch_filename.replace(".", "_"), module_path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _evaluate_launch_description(launch_description, overrides: dict[str, str]):
    context = LaunchContext()
    for key, val in overrides.items():
        context.launch_configurations[key] = val

    produced_actions = []
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
        elif isinstance(entity, OpaqueFunction):
            result = entity.execute(context)
            if result:
                produced_actions.extend(result)
        else:
            produced_actions.append(entity)
    return context, produced_actions


def test_supported_constants():
    assert DEFAULT_ROBOT_CONFIG == "omts"
    assert SUPPORTED_ROBOT_CONFIGS == ("omts", "lab_bb_01")


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("omts", "omts"),
        ("OMTS", "omts"),
        ("lab_bb_01", "lab_bb_01"),
        ("lab-bb-01", "lab_bb_01"),
        ("  LAB-BB-01  ", "lab_bb_01"),
    ],
)
def test_normalize_robot_config_valid(raw: str, expected: str):
    assert normalize_robot_config(raw) == expected


@pytest.mark.parametrize(
    "invalid_value",
    ["", "   ", "ur10e", "../omts", "bad/name", "lab_bb_02"],
)
def test_normalize_robot_config_invalid(invalid_value: str):
    with pytest.raises(ValueError) as exc_info:
        normalize_robot_config(invalid_value)
    msg = str(exc_info.value)
    assert "omts" in msg
    assert "lab_bb_01" in msg


def test_resolve_robot_config_from_runtime_pb(tmp_path: Path):
    pb_path = _write_runtime_context_pb(
        tmp_path / "runtime_config.pb", robot_config_value="lab_bb_01"
    )
    assert read_robot_config_from_runtime_pb(str(pb_path)) == "lab_bb_01"
    assert resolve_robot_config("", str(pb_path)) == "lab_bb_01"


def test_resolve_robot_config_hyphenated_in_runtime_pb(tmp_path: Path):
    pb_path = _write_runtime_context_pb(
        tmp_path / "runtime_config.pb", robot_config_value="lab-bb-01"
    )
    assert resolve_robot_config("", str(pb_path)) == "lab_bb_01"


def test_resolve_robot_config_cli_overrides_runtime_pb(tmp_path: Path):
    pb_lab = _write_runtime_context_pb(
        tmp_path / "lab.pb", robot_config_value="lab_bb_01"
    )
    assert resolve_robot_config("omts", str(pb_lab)) == "omts"

    pb_omts = _write_runtime_context_pb(
        tmp_path / "omts.pb", robot_config_value="omts"
    )
    assert resolve_robot_config("lab-bb-01", str(pb_omts)) == "lab_bb_01"


def test_resolve_robot_config_fallbacks_to_default(tmp_path: Path):
    # 1. Missing file
    missing = tmp_path / "does_not_exist.pb"
    assert resolve_robot_config("", str(missing)) == "omts"

    # 2. Unset robot_config field in MoveItPlanningServiceConfig
    unset_pb = _write_runtime_context_pb(
        tmp_path / "unset.pb", robot_config_value=None
    )
    assert resolve_robot_config("", str(unset_pb)) == "omts"

    # 3. Empty/whitespace robot_config in MoveItPlanningServiceConfig
    blank_pb = _write_runtime_context_pb(
        tmp_path / "blank.pb", robot_config_value="   "
    )
    assert resolve_robot_config("", str(blank_pb)) == "omts"

    # 4. Corrupt protobuf bytes
    corrupt_pb = tmp_path / "corrupt.pb"
    corrupt_pb.write_bytes(b"\xff\x00\xfe\x01not-valid-protobuf")
    assert resolve_robot_config("", str(corrupt_pb)) == "omts"

    # 5. RuntimeContext packing an unrelated Any type_url
    unrelated_pb = _write_runtime_context_pb(
        tmp_path / "unrelated.pb",
        raw_any_type_url="type.googleapis.com/other.ServiceConfig",
        raw_any_value=b"",
    )
    assert resolve_robot_config("", str(unrelated_pb)) == "omts"


def test_resolve_robot_config_invalid_in_runtime_pb_raises(tmp_path: Path):
    bad_pb = _write_runtime_context_pb(
        tmp_path / "bad.pb", robot_config_value="ur10e"
    )
    with pytest.raises(ValueError) as exc_info:
        resolve_robot_config("", str(bad_pb))
    assert "omts" in str(exc_info.value)
    assert "lab_bb_01" in str(exc_info.value)

    # Explicit valid CLI argument takes precedence without failing on runtime_config.pb
    assert resolve_robot_config("omts", str(bad_pb)) == "omts"


def _resolved_include_locations(
    context: LaunchContext, actions: list
) -> list[str]:
    locations: list[str] = []
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            action.launch_description_source.get_launch_description(context)
            locations.append(
                str(Path(action.launch_description_source.location).resolve())
            )
    return locations


def _matches_pkg(location: str, pkg_name: str) -> bool:
    share_dir = Path(get_package_share_directory(pkg_name))
    candidates = {
        str(share_dir),
        str(share_dir.resolve()),
        str((share_dir / "package.xml").resolve().parent),
    }
    return any(location.startswith(prefix) for prefix in candidates)


def test_service_launch_selects_expected_moveit_config(tmp_path: Path):
    service_launch = _load_launch_module("service.launch.py")

    # Default (no CLI override, no /etc/intrinsic/runtime_config.pb) -> omts_moveit_config
    default_ctx, default_actions = _evaluate_launch_description(
        service_launch.generate_launch_description(), {}
    )
    default_includes = _resolved_include_locations(default_ctx, default_actions)
    assert len(default_includes) >= 4
    assert all(
        _matches_pkg(loc, "omts_moveit_config") for loc in default_includes
    )

    # CLI override robot_config:="lab-bb-01" -> lab_bb_01_moveit_config
    lab_ctx, lab_actions = _evaluate_launch_description(
        service_launch.generate_launch_description(),
        {"robot_config": "lab-bb-01"},
    )
    lab_includes = _resolved_include_locations(lab_ctx, lab_actions)
    assert len(lab_includes) >= 4
    assert all(
        _matches_pkg(loc, "lab_bb_01_moveit_config") for loc in lab_includes
    )

    # Runtime protobuf selection when CLI robot_config is empty
    pb_lab = _write_runtime_context_pb(
        tmp_path / "runtime_config.pb", robot_config_value="lab-bb-01"
    )
    ctx = LaunchContext()
    for entity in service_launch.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
    runtime_actions = service_launch.build_service_launch_actions(
        ctx, runtime_config_path=str(pb_lab)
    )
    runtime_includes = _resolved_include_locations(ctx, runtime_actions)
    assert len(runtime_includes) >= 4
    assert all(
        _matches_pkg(loc, "lab_bb_01_moveit_config")
        for loc in runtime_includes
    )

    # Invalid CLI robot_config -> ValueError
    with pytest.raises(ValueError):
        _evaluate_launch_description(
            service_launch.generate_launch_description(),
            {"robot_config": "invalid_cell"},
        )


def test_grasp_planning_test_launch_selects_expected_moveit_config():
    demo_launch = _load_launch_module("grasp_planning_test.launch.py")

    default_ctx, default_actions = _evaluate_launch_description(
        demo_launch.generate_launch_description(), {}
    )
    default_includes = _resolved_include_locations(default_ctx, default_actions)
    assert len(default_includes) >= 1
    assert all(
        _matches_pkg(loc, "omts_moveit_config") for loc in default_includes
    )

    lab_ctx, lab_actions = _evaluate_launch_description(
        demo_launch.generate_launch_description(),
        {"robot_config": "lab-bb-01"},
    )
    lab_includes = _resolved_include_locations(lab_ctx, lab_actions)
    assert len(lab_includes) >= 1
    assert all(
        _matches_pkg(loc, "lab_bb_01_moveit_config") for loc in lab_includes
    )

    with pytest.raises(ValueError):
        _evaluate_launch_description(
            demo_launch.generate_launch_description(),
            {"robot_config": "nonexistent"},
        )
