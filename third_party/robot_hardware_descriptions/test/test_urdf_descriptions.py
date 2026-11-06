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

"""Verification tests for omts and lab_bb_01 robot hardware descriptions."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import pytest
import xacro


PKG_NAME = 'robot_hardware_descriptions'
PACKAGE_URI_PREFIX = f'package://{PKG_NAME}/'

REQUIRED_LINKS = {
    'base_link',
    'shoulder_link',
    'upper_arm_link',
    'forearm_link',
    'wrist_1_link',
    'wrist_2_link',
    'wrist_3_link',
    'flange',
    'tool0',
    'hande_hande_base_link',
    'gripper_base_link',
    'hande_tcp',
    'hande_tool_frame',
    'tool_frame',
    'hande_hande_finger_link_l',
    'hande_hande_finger_link_r',
    'orbbec_camera_link',
    'orbbec_camera_mesh_centroid_frame',
}

REQUIRED_JOINTS = {
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
    'hande_left_finger_joint',
    'hande_right_finger_joint',
    'flange_to_hande',
    'flange_to_orbbec_camera',
}


def _render_urdf(config_name: str) -> tuple[str, ET.Element]:
    share_dir = Path(get_package_share_directory(PKG_NAME))
    xacro_path = share_dir / 'urdf' / f'{config_name}.urdf.xacro'
    assert xacro_path.is_file(), f'Missing xacro file: {xacro_path}'
    xml_str = xacro.process_file(str(xacro_path)).toxml()
    return xml_str, ET.fromstring(xml_str)


def _parse_floats(attr_value: str) -> list[float]:
    return [float(x) for x in attr_value.split()]


@pytest.mark.parametrize('config_name', ['omts', 'lab_bb_01'])
def test_xacro_renders_expected_links_joints_and_meshes(config_name: str):
    """Both cell xacros render cleanly with valid mesh URIs and shared semantic links."""
    share_dir = Path(get_package_share_directory(PKG_NAME))
    xml_str, root = _render_urdf(config_name)

    assert root.attrib.get('name') == config_name
    assert 'robot_hardware_description/' not in xml_str

    link_names = {el.attrib['name'] for el in root.findall('link')}
    joint_names = {el.attrib['name'] for el in root.findall('joint')}
    assert REQUIRED_LINKS.issubset(link_names)
    assert REQUIRED_JOINTS.issubset(joint_names)

    meshes = root.findall('.//mesh')
    assert len(meshes) > 0
    for mesh in meshes:
        uri = mesh.attrib['filename']
        assert uri.startswith(PACKAGE_URI_PREFIX), f'Unexpected mesh URI: {uri}'
        rel_path = uri[len(PACKAGE_URI_PREFIX):]
        mesh_file = share_dir / rel_path
        assert mesh_file.is_file(), f'Mesh file does not exist: {mesh_file}'
        assert mesh_file.stat().st_size > 0, f'Empty mesh file: {mesh_file}'


def test_omts_preserves_existing_offsets_and_ur5e_kinematics():
    """OMTS URDF keeps existing flange (z=0.0), TCP (z=0.14), and UR5e dimensions."""
    _, root = _render_urdf('omts')
    joints = {j.attrib['name']: j for j in root.findall('joint')}

    elbow_xyz = _parse_floats(joints['elbow_joint'].find('origin').attrib['xyz'])
    assert elbow_xyz == pytest.approx([0.425, 0.0, 0.0])

    flange_xyz = _parse_floats(joints['flange_to_hande'].find('origin').attrib['xyz'])
    assert flange_xyz == pytest.approx([0.0, 0.0, 0.0])

    tcp_xyz = _parse_floats(
        joints['hande_attach_tool_frame'].find('origin').attrib['xyz']
    )
    assert tcp_xyz == pytest.approx([0.0, 0.0, 0.14])


def test_lab_bb_01_matches_intrinsic_omts_ur3e_and_wrist_3_bounds():
    """lab_bb_01 URDF uses vendored UR3e, z=0.128 TCP, -Y Orbbec camera, and +-2pi wrist_3."""
    _, root = _render_urdf('lab_bb_01')
    joints = {j.attrib['name']: j for j in root.findall('joint')}

    shoulder_xyz = _parse_floats(
        joints['shoulder_pan_joint'].find('origin').attrib['xyz']
    )
    elbow_xyz = _parse_floats(joints['elbow_joint'].find('origin').attrib['xyz'])
    wrist1_xyz = _parse_floats(joints['wrist_1_joint'].find('origin').attrib['xyz'])
    assert shoulder_xyz == pytest.approx([0.0, 0.0, 0.15185])
    assert elbow_xyz == pytest.approx([0.24355, 0.0, 0.0])
    assert wrist1_xyz == pytest.approx([0.2132, 0.0, 0.0])

    wrist3_limit = joints['wrist_3_joint'].find('limit')
    assert float(wrist3_limit.attrib['lower']) == pytest.approx(
        -2.0 * math.pi, abs=1e-5
    )
    assert float(wrist3_limit.attrib['upper']) == pytest.approx(
        2.0 * math.pi, abs=1e-5
    )
    assert float(wrist3_limit.attrib['velocity']) == pytest.approx(
        2.0 * math.pi, abs=1e-5
    )

    flange_xyz = _parse_floats(joints['flange_to_hande'].find('origin').attrib['xyz'])
    assert flange_xyz == pytest.approx([0.0, 0.0, 0.0])

    tcp_xyz = _parse_floats(
        joints['hande_attach_tool_frame'].find('origin').attrib['xyz']
    )
    assert tcp_xyz == pytest.approx([0.0, 0.0, 0.128])

    cam_origin = joints['flange_to_orbbec_camera'].find('origin')
    cam_xyz = _parse_floats(cam_origin.attrib['xyz'])
    cam_rpy = _parse_floats(cam_origin.attrib['rpy'])
    assert cam_xyz == pytest.approx(
        [-0.02384257, -0.06852203, 0.03192175], abs=1e-6
    )
    assert cam_rpy == pytest.approx(
        [0.00361469, 0.00089883, -0.00419457], abs=1e-6
    )


@pytest.mark.parametrize('ur_model', ['ur5e', 'ur3e'])
def test_vendored_ur_docs_and_licenses_installed(ur_model: str):
    """Vendored UR directories include README and upstream BSD-3-Clause LICENSE files."""
    share_dir = Path(get_package_share_directory(PKG_NAME))
    ur_dir = share_dir / 'urdf' / ur_model
    assert (ur_dir / 'README.md').is_file()
    assert (ur_dir / 'visual' / 'LICENSE').is_file()
    assert (ur_dir / 'collision' / 'LICENSE').is_file()


def test_display_launch_normalizes_and_validates_robot_config():
    """display.launch.py supports omts, lab_bb_01, lab-bb-01, and rejects invalid names."""
    import importlib.util

    share_dir = Path(get_package_share_directory(PKG_NAME))
    spec = importlib.util.spec_from_file_location(
        'display_launch', share_dir / 'launch' / 'display.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert hasattr(module, 'resolve_display_xacro_path')
    assert module.resolve_display_xacro_path('omts').endswith('omts.urdf.xacro')
    assert module.resolve_display_xacro_path('lab_bb_01').endswith(
        'lab_bb_01.urdf.xacro'
    )
    assert module.resolve_display_xacro_path('lab-bb-01').endswith(
        'lab_bb_01.urdf.xacro'
    )
    assert module.resolve_display_xacro_path(' LAB-BB-01 ').endswith(
        'lab_bb_01.urdf.xacro'
    )
    with pytest.raises(ValueError, match='omts'):
        module.resolve_display_xacro_path('unknown_robot')
    with pytest.raises(ValueError, match='lab_bb_01'):
        module.resolve_display_xacro_path('../omts')
    assert module.generate_launch_description() is not None
    _ = LaunchContext()
