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

"""Workcell robot configuration resolution for moveit_planning_service."""

from collections.abc import Sequence
import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from google.protobuf import descriptor_pb2
from google.protobuf import message_factory
from google.protobuf.message import DecodeError

SUPPORTED_ROBOT_CONFIGS: tuple[str, ...] = ("omts", "lab_bb_01")
DEFAULT_ROBOT_CONFIG: str = "omts"
DEFAULT_RUNTIME_CONFIG_PATH: str = "/etc/intrinsic/runtime_config.pb"

_RUNTIME_CONTEXT_FULL_NAME = "intrinsic_proto.config.RuntimeContext"
_SERVICE_CONFIG_FULL_NAME = "intrinsic.MoveItPlanningServiceConfig"


def normalize_robot_config(value: str) -> str:
    """Normalize and validate a robot configuration name."""
    normalized = value.strip().lower().replace("-", "_")
    if normalized not in SUPPORTED_ROBOT_CONFIGS:
        supported = ", ".join(SUPPORTED_ROBOT_CONFIGS)
        raise ValueError(
            f"Unsupported robot_config {value!r}. "
            f"Supported configurations: {supported}"
        )
    return normalized


def _find_package_file(pkg_name: str, filename: str) -> Path | None:
    try:
        candidate = Path(get_package_share_directory(pkg_name)) / filename
        if candidate.is_file():
            return candidate
    except PackageNotFoundError:
        pass

    search_prefixes: list[str] = []
    for env_var in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        raw = os.environ.get(env_var, "")
        if raw:
            search_prefixes.extend(p for p in raw.split(os.pathsep) if p)
    search_prefixes.extend(
        [
            "/opt/ros/overlay/install",
            "/opt/ros/underlay/install",
            "/opt/intrinsic/intrinsic_sdk_cmake/install",
        ]
    )

    for prefix in search_prefixes:
        for rel in (
            Path(prefix) / "share" / pkg_name / filename,
            Path(prefix) / pkg_name / "share" / pkg_name / filename,
        ):
            if rel.is_file():
                return rel
    return None


def default_descriptor_paths() -> list[Path]:
    """Locate installed protobuf FileDescriptorSet (`.desc`) files."""
    paths: list[Path] = []
    for pkg_name, filename in (
        ("intrinsic_sdk_cmake", "intrinsic_proto.desc"),
        ("moveit_planning_service", "moveit_planning_service_protos.desc"),
    ):
        found = _find_package_file(pkg_name, filename)
        if found is not None:
            paths.append(found)
    return paths


def load_runtime_config_message_classes(
    descriptor_paths: Sequence[str | Path] | None = None,
) -> tuple[type, type]:
    """Load `(RuntimeContext, MoveItPlanningServiceConfig)` protobuf classes."""
    resolved_paths = (
        [Path(p) for p in descriptor_paths]
        if descriptor_paths is not None
        else default_descriptor_paths()
    )
    file_map: dict[str, descriptor_pb2.FileDescriptorProto] = {}
    msg_to_file: dict[str, str] = {}

    for path in resolved_paths:
        try:
            desc_bytes = path.read_bytes()
        except FileNotFoundError:
            continue
        fds = descriptor_pb2.FileDescriptorSet.FromString(desc_bytes)
        for file_proto in fds.file:
            if file_proto.name not in file_map:
                file_map[file_proto.name] = file_proto
                pkg = file_proto.package
                for msg_proto in file_proto.message_type:
                    full_name = (
                        f"{pkg}.{msg_proto.name}" if pkg else msg_proto.name
                    )
                    msg_to_file[full_name] = file_proto.name

    root_files: list[str] = []
    for full_name in (_RUNTIME_CONTEXT_FULL_NAME, _SERVICE_CONFIG_FULL_NAME):
        if full_name not in msg_to_file:
            raise KeyError(
                f"Required protobuf message {full_name!r} not found in "
                f"descriptor sets: {[str(p) for p in resolved_paths]}"
            )
        root_files.append(msg_to_file[full_name])

    sorted_protos: list[descriptor_pb2.FileDescriptorProto] = []
    visited: set[str] = set()

    def _visit(name: str) -> None:
        if name in visited:
            return
        visited.add(name)
        fp = file_map[name]
        for dep in fp.dependency:
            if dep not in file_map:
                raise KeyError(
                    f"Missing required protobuf dependency descriptor {dep!r} "
                    f"(imported by {name!r})"
                )
            _visit(dep)
        sorted_protos.append(fp)

    for fname in root_files:
        _visit(fname)

    messages = message_factory.GetMessages(sorted_protos)
    return (
        messages[_RUNTIME_CONTEXT_FULL_NAME],
        messages[_SERVICE_CONFIG_FULL_NAME],
    )


def read_robot_config_from_runtime_pb(
    runtime_config_path: str = DEFAULT_RUNTIME_CONFIG_PATH,
    descriptor_paths: Sequence[str | Path] | None = None,
) -> str:
    """Read and normalize `robot_config` from a serialized RuntimeContext file.

    Returns `""` strictly when the file is absent or empty (0 bytes), unpacks to
    an unrelated service configuration `Any` type, or leaves `robot_config`
    unset/blank. Raises `RuntimeError` if a present, non-empty runtime config
    cannot be read or decoded, and `ValueError` if `robot_config` is set to an
    unsupported value, so launch fails closed rather than planning with the
    wrong robot model.
    """
    if not runtime_config_path or not runtime_config_path.strip():
        return ""
    pb_path = Path(runtime_config_path)

    try:
        raw_bytes = pb_path.read_bytes()
    except FileNotFoundError:
        return ""
    except OSError as exc:
        raise RuntimeError(
            f"Failed to read runtime config at {pb_path}: {exc}"
        ) from exc
    if not raw_bytes:
        return ""

    try:
        runtime_ctx_cls, service_cfg_cls = load_runtime_config_message_classes(
            descriptor_paths
        )
    except (DecodeError, KeyError, OSError, TypeError) as exc:
        raise RuntimeError(
            f"Failed to load protobuf descriptors while reading {pb_path}: {exc}"
        ) from exc

    ctx_msg = runtime_ctx_cls()
    try:
        ctx_msg.ParseFromString(raw_bytes)
    except (DecodeError, UnicodeDecodeError) as exc:
        raise RuntimeError(
            f"Failed to decode RuntimeContext from {pb_path}: {exc}"
        ) from exc

    if not ctx_msg.HasField("config") or not ctx_msg.config.type_url:
        return ""

    cfg_msg = service_cfg_cls()
    if not ctx_msg.config.Is(cfg_msg.DESCRIPTOR):
        return ""
    try:
        unpacked = ctx_msg.config.Unpack(cfg_msg)
    except (DecodeError, UnicodeDecodeError) as exc:
        raise RuntimeError(
            f"Failed to unpack MoveItPlanningServiceConfig from {pb_path}: {exc}"
        ) from exc
    if not unpacked:
        raise RuntimeError(
            f"Failed to unpack MoveItPlanningServiceConfig from {pb_path}."
        )

    raw_robot_config = cfg_msg.robot_config.strip()
    if not raw_robot_config:
        return ""
    return normalize_robot_config(raw_robot_config)


def resolve_robot_config(
    cli_robot_config: str = "",
    runtime_config_path: str = DEFAULT_RUNTIME_CONFIG_PATH,
    descriptor_paths: Sequence[str | Path] | None = None,
) -> str:
    """Resolve active robot configuration (`CLI > runtime_config.pb > 'omts'`)."""
    if cli_robot_config and cli_robot_config.strip():
        return normalize_robot_config(cli_robot_config)

    from_runtime = read_robot_config_from_runtime_pb(
        runtime_config_path=runtime_config_path,
        descriptor_paths=descriptor_paths,
    )
    if from_runtime:
        return from_runtime

    return DEFAULT_ROBOT_CONFIG
