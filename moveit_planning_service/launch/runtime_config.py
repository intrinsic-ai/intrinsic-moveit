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

"""Resolve MoveIt launch defaults from /etc/intrinsic/runtime_config.pb."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from google.protobuf import any_pb2
from google.protobuf import descriptor_pb2
from google.protobuf import descriptor_pool
from google.protobuf import message_factory
from google.protobuf.message import DecodeError

DEFAULT_RUNTIME_CONFIG_PATH = "/etc/intrinsic/runtime_config.pb"
RUNTIME_CONFIG_ENV_VAR = "INTRINSIC_RUNTIME_CONFIG"
_CONFIG_TYPE_SUFFIX = "/intrinsic.MoveItPlanningServiceConfig"


def _add_file_with_deps(
    pool: descriptor_pool.DescriptorPool,
    fds_by_name: dict[str, descriptor_pb2.FileDescriptorProto],
    filename: str,
) -> None:
    try:
        pool.FindFileByName(filename)
        return
    except KeyError:
        pass
    fd = fds_by_name[filename]
    for dep in fd.dependency:
        _add_file_with_deps(pool, fds_by_name, dep)
    pool.Add(fd)


def _get_message_class(pool: descriptor_pool.DescriptorPool, descriptor):
    if hasattr(message_factory, "GetMessageClass"):
        return message_factory.GetMessageClass(descriptor)
    return message_factory.MessageFactory(pool).GetPrototype(descriptor)


def _build_config_classes():
    pool = descriptor_pool.DescriptorPool()
    pool.AddSerializedFile(any_pb2.DESCRIPTOR.serialized_pb)

    # Minimal wire-compatible RuntimeContext descriptor (field 6: Any config)
    # so launch does not depend on non-ament intrinsic_sdk_cmake search paths.
    rc_file = descriptor_pb2.FileDescriptorProto(
        name="intrinsic/resources/proto/runtime_context_minimal.proto",
        package="intrinsic_proto.config",
        syntax="proto3",
        dependency=[any_pb2.DESCRIPTOR.name],
    )
    rc_msg = rc_file.message_type.add(name="RuntimeContext")
    rc_msg.field.add(
        name="config",
        number=6,
        label=descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL,
        type=descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE,
        type_name=".google.protobuf.Any",
    )
    pool.Add(rc_file)

    desc_path = (
        Path(get_package_share_directory("moveit_planning_service"))
        / "moveit_planning_service_protos.desc"
    )
    svc_fds = descriptor_pb2.FileDescriptorSet.FromString(
        desc_path.read_bytes()
    )
    svc_by_name = {fd.name: fd for fd in svc_fds.file}
    for fd in svc_fds.file:
        _add_file_with_deps(pool, svc_by_name, fd.name)

    rc_cls = _get_message_class(
        pool,
        pool.FindMessageTypeByName("intrinsic_proto.config.RuntimeContext"),
    )
    cfg_cls = _get_message_class(
        pool,
        pool.FindMessageTypeByName("intrinsic.MoveItPlanningServiceConfig"),
    )
    return rc_cls, cfg_cls


def resolve_default_robot_config(
    supported_configs: tuple[str, ...],
    default: str = "omts",
) -> str:
    """Return robot_config from runtime_config.pb, or default if unset."""
    config_path = Path(
        os.environ.get(RUNTIME_CONFIG_ENV_VAR, DEFAULT_RUNTIME_CONFIG_PATH)
    )
    if not config_path.is_file():
        return default

    raw = config_path.read_bytes()
    if not raw:
        return default

    rc_cls, cfg_cls = _build_config_classes()
    rc = rc_cls()
    try:
        rc.ParseFromString(raw)
    except DecodeError as exc:
        raise ValueError(
            f"Failed to parse RuntimeContext in {config_path}"
        ) from exc

    if not rc.config.type_url.endswith(_CONFIG_TYPE_SUFFIX):
        return default

    cfg = cfg_cls()
    try:
        cfg.ParseFromString(rc.config.value)
    except DecodeError as exc:
        raise ValueError(
            f"Failed to parse MoveItPlanningServiceConfig in {config_path}"
        ) from exc

    if not cfg.HasField("robot_config") or not cfg.robot_config:
        return default

    if cfg.robot_config not in supported_configs:
        raise ValueError(
            f"Unsupported robot_config '{cfg.robot_config}' in {config_path}; "
            f"expected one of {list(supported_configs)}."
        )
    return cfg.robot_config
