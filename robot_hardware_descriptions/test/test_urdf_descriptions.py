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

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import pytest
import xacro


PKG_NAME = 'robot_hardware_descriptions'
VENDOR_PKG_NAME = 'robot_meshes_vendor'
ALLOWED_MESH_PACKAGES = {PKG_NAME, VENDOR_PKG_NAME}

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
        assert uri.startswith('package://'), f'Unexpected mesh URI: {uri}'
        pkg_and_rel = uri[len('package://'):]
        pkg_name, sep, rel_path = pkg_and_rel.partition('/')
        assert sep == '/' and pkg_name in ALLOWED_MESH_PACKAGES, (
            f'Unexpected mesh package in URI: {uri}'
        )
        pkg_share = Path(get_package_share_directory(pkg_name))
        mesh_file = pkg_share / rel_path
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
    """UR macro directories include README, and robot_meshes_vendor installs BSD licenses."""
    share_dir = Path(get_package_share_directory(PKG_NAME))
    vendor_share_dir = Path(get_package_share_directory(VENDOR_PKG_NAME))
    assert (share_dir / 'urdf' / ur_model / 'README.md').is_file()
    vendor_ur_dir = vendor_share_dir / 'meshes' / ur_model
    assert (vendor_ur_dir / 'visual' / 'LICENSE').is_file()
    assert (vendor_ur_dir / 'collision' / 'LICENSE').is_file()


def test_display_launch_validates_robot_config_and_builds_nodes():
    """display.launch.py supports omts/lab_bb_01, builds all 3 nodes, and rejects bad configs."""
    share_dir = Path(get_package_share_directory(PKG_NAME))
    assert (share_dir / 'rviz' / 'display.rviz').is_file()

    spec = importlib.util.spec_from_file_location(
        'display_launch', share_dir / 'launch' / 'display.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    for override, expected_name in ((None, 'omts'), ('lab_bb_01', 'lab_bb_01')):
        ld = module.generate_launch_description()
        assert len(ld.entities) == 4
        declare_arg = ld.entities[0]
        nodes = ld.entities[1:]
        assert isinstance(declare_arg, DeclareLaunchArgument)
        assert all(isinstance(n, Node) for n in nodes)
        assert [n.node_package for n in nodes] == [
            'robot_state_publisher',
            'joint_state_publisher_gui',
            'rviz2',
        ]

        context = LaunchContext()
        if override is not None:
            context.launch_configurations['robot_config'] = override
        declare_arg.visit(context)
        rsp_params = evaluate_parameters(
            context,
            normalize_parameters(
                [{'robot_description': module.build_robot_description_param()}]
            ),
        )
        assert f'<robot name="{expected_name}">' in rsp_params[0]['robot_description']

    for invalid_config in ('', 'lab-bb-01', 'LAB_BB_01', 'unknown_robot', '../omts'):
        ld = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations['robot_config'] = invalid_config
        with pytest.raises(RuntimeError, match='robot_config'):
            ld.entities[0].visit(context)
