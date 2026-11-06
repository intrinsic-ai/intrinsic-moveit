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
import math
import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from google.protobuf import descriptor_pb2
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from moveit_configs_utils import MoveItConfigsBuilder
import pytest

import moveit_planning_service.robot_config as robot_config_mod
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


def _load_launch_module(
    launch_filename: str, pkg_name: str = "moveit_planning_service"
):
    share_dir = Path(get_package_share_directory(pkg_name))
    module_path = share_dir / "launch" / launch_filename
    spec = importlib.util.spec_from_file_location(
        f"{pkg_name}_{launch_filename.replace('.', '_')}", module_path
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
    display_launch = _load_launch_module(
        "display.launch.py", pkg_name="robot_hardware_descriptions"
    )
    assert tuple(display_launch.SUPPORTED_ROBOT_CONFIGS) == (
        SUPPORTED_ROBOT_CONFIGS
    )


def test_find_package_file_falls_back_to_env_prefixes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    def _always_missing(pkg_name: str) -> str:
        raise PackageNotFoundError(pkg_name)

    monkeypatch.setattr(
        robot_config_mod, "get_package_share_directory", _always_missing
    )

    ament_prefix = tmp_path / "ament_ws"
    ament_desc = ament_prefix / "share" / "custom_pkg_a" / "test_a.desc"
    ament_desc.parent.mkdir(parents=True)
    ament_desc.write_bytes(b"a")

    cmake_prefix = tmp_path / "cmake_ws"
    cmake_desc = (
        cmake_prefix / "custom_pkg_b" / "share" / "custom_pkg_b" / "test_b.desc"
    )
    cmake_desc.parent.mkdir(parents=True)
    cmake_desc.write_bytes(b"b")

    monkeypatch.setenv(
        "AMENT_PREFIX_PATH", f"{tmp_path / 'empty'}{os.pathsep}{ament_prefix}"
    )
    monkeypatch.setenv("CMAKE_PREFIX_PATH", str(cmake_prefix))

    assert (
        robot_config_mod._find_package_file("custom_pkg_a", "test_a.desc")
        == ament_desc
    )
    assert (
        robot_config_mod._find_package_file("custom_pkg_b", "test_b.desc")
        == cmake_desc
    )
    assert (
        robot_config_mod._find_package_file("custom_pkg_c", "missing.desc")
        is None
    )


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
    # 1. Empty/whitespace path or missing file -> silent fallback
    assert resolve_robot_config("", "") == "omts"
    assert resolve_robot_config("", "   ") == "omts"
    missing = tmp_path / "does_not_exist.pb"
    assert resolve_robot_config("", str(missing)) == "omts"

    # 2. 0-byte runtime_config.pb -> silent fallback
    empty_pb = tmp_path / "empty.pb"
    empty_pb.write_bytes(b"")
    assert resolve_robot_config("", str(empty_pb)) == "omts"

    # 3. Unset robot_config field in MoveItPlanningServiceConfig -> silent fallback
    unset_pb = _write_runtime_context_pb(
        tmp_path / "unset.pb", robot_config_value=None
    )
    assert resolve_robot_config("", str(unset_pb)) == "omts"

    # 4. Empty/whitespace robot_config in MoveItPlanningServiceConfig -> silent fallback
    blank_pb = _write_runtime_context_pb(
        tmp_path / "blank.pb", robot_config_value="   "
    )
    assert resolve_robot_config("", str(blank_pb)) == "omts"

    # 5. RuntimeContext packing an unrelated Any type_url -> silent fallback
    unrelated_pb = _write_runtime_context_pb(
        tmp_path / "unrelated.pb",
        raw_any_type_url="type.googleapis.com/other.ServiceConfig",
        raw_any_value=b"",
    )
    assert resolve_robot_config("", str(unrelated_pb)) == "omts"


def test_read_runtime_pb_fails_closed_on_read_or_decode_errors(tmp_path: Path):
    # 1. OSError when reading path (e.g. directory instead of file) -> RuntimeError
    dir_path = tmp_path / "not_a_file.pb"
    dir_path.mkdir()
    with pytest.raises(RuntimeError, match="Failed to read runtime config"):
        resolve_robot_config("", str(dir_path))

    # 2. Corrupt RuntimeContext bytes -> RuntimeError
    corrupt_pb = tmp_path / "corrupt.pb"
    corrupt_pb.write_bytes(b"\xff\x00\xfe\x01not-valid-protobuf")
    with pytest.raises(RuntimeError, match="Failed to decode RuntimeContext"):
        resolve_robot_config("", str(corrupt_pb))

    # 3. Corrupt MoveItPlanningServiceConfig inside Any -> RuntimeError
    corrupt_any_pb = _write_runtime_context_pb(
        tmp_path / "corrupt_any.pb",
        raw_any_type_url=(
            "type.googleapis.com/intrinsic.MoveItPlanningServiceConfig"
        ),
        raw_any_value=b"\xff\x00\xfe\x01not-valid-protobuf",
    )
    with pytest.raises(
        RuntimeError, match="Failed to unpack MoveItPlanningServiceConfig"
    ):
        resolve_robot_config("", str(corrupt_any_pb))

    # 4. Missing or unreadable descriptor files with non-empty runtime_config.pb -> RuntimeError
    valid_pb = _write_runtime_context_pb(
        tmp_path / "valid_lab.pb", robot_config_value="lab_bb_01"
    )
    with pytest.raises(
        RuntimeError, match="Failed to load protobuf descriptors"
    ):
        read_robot_config_from_runtime_pb(str(valid_pb), descriptor_paths=[])
    with pytest.raises(
        RuntimeError, match="Failed to load protobuf descriptors"
    ):
        read_robot_config_from_runtime_pb(
            str(valid_pb), descriptor_paths=[dir_path]
        )

    # 5. Descriptor bundle missing an imported .proto dependency -> KeyError / RuntimeError
    desc_paths = default_descriptor_paths()
    stripped_sdk_desc = tmp_path / "stripped_intrinsic_proto.desc"
    sdk_fds = descriptor_pb2.FileDescriptorSet.FromString(
        desc_paths[0].read_bytes()
    )
    kept_files = [
        f for f in sdk_fds.file if f.name != "google/protobuf/any.proto"
    ]
    del sdk_fds.file[:]
    sdk_fds.file.extend(kept_files)
    stripped_sdk_desc.write_bytes(sdk_fds.SerializeToString())

    with pytest.raises(KeyError, match="google/protobuf/any.proto"):
        load_runtime_config_message_classes([stripped_sdk_desc, desc_paths[1]])
    with pytest.raises(RuntimeError, match="google/protobuf/any.proto"):
        read_robot_config_from_runtime_pb(
            str(valid_pb), descriptor_paths=[stripped_sdk_desc, desc_paths[1]]
        )

    # Explicit CLI override does not touch corrupt runtime_config.pb
    assert resolve_robot_config("omts", str(corrupt_pb)) == "omts"


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


def _planning_node_robot_config(
    context: LaunchContext, actions: list
) -> str:
    planning_nodes = [
        action
        for action in actions
        if isinstance(action, Node)
        and action.node_executable == "moveit_planning_node"
    ]
    assert len(planning_nodes) == 1
    evaluated_params = evaluate_parameters(
        context, planning_nodes[0]._Node__parameters
    )
    merged: dict[str, object] = {}
    for entry in evaluated_params:
        if isinstance(entry, dict):
            merged.update(entry)
    assert "robot_config" in merged
    return str(merged["robot_config"])


def test_service_launch_selects_expected_moveit_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    service_launch = _load_launch_module("service.launch.py")
    monkeypatch.setattr(
        service_launch,
        "DEFAULT_RUNTIME_CONFIG_PATH",
        str(tmp_path / "isolated_missing_runtime_config.pb"),
    )

    # Default (no CLI override, isolated missing runtime_config.pb) -> omts_moveit_config
    default_ctx, default_actions = _evaluate_launch_description(
        service_launch.generate_launch_description(), {}
    )
    default_includes = _resolved_include_locations(default_ctx, default_actions)
    assert len(default_includes) >= 4
    assert all(
        _matches_pkg(loc, "omts_moveit_config") for loc in default_includes
    )
    assert _planning_node_robot_config(default_ctx, default_actions) == "omts"

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
    assert _planning_node_robot_config(lab_ctx, lab_actions) == "lab_bb_01"

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
    assert _planning_node_robot_config(ctx, runtime_actions) == "lab_bb_01"

    # CLI robot_config:="omts" overrides runtime_config.pb ("lab-bb-01")
    override_ctx = LaunchContext()
    override_ctx.launch_configurations["robot_config"] = "omts"
    for entity in service_launch.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(override_ctx)
    override_actions = service_launch.build_service_launch_actions(
        override_ctx, runtime_config_path=str(pb_lab)
    )
    override_includes = _resolved_include_locations(
        override_ctx, override_actions
    )
    assert len(override_includes) >= 4
    assert all(
        _matches_pkg(loc, "omts_moveit_config") for loc in override_includes
    )
    assert (
        _planning_node_robot_config(override_ctx, override_actions) == "omts"
    )

    # Corrupt runtime_config.pb fails closed during launch instead of loading omts
    corrupt_pb = tmp_path / "corrupt_runtime.pb"
    corrupt_pb.write_bytes(b"\xff\x00\xfe\x01not-valid-protobuf")
    with pytest.raises(RuntimeError, match="Failed to decode RuntimeContext"):
        service_launch.build_service_launch_actions(
            ctx, runtime_config_path=str(corrupt_pb)
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
