"""Tests for fusion_addin.generators.urdf.generate_urdf_xacro.

Must run with plain `python3 -m pytest` — no Fusion, no live ROS needed
(the one exception, test_check_urdf_accepts_generated_xacro, shells out to
the `check_urdf` binary and skips itself if that binary isn't on PATH).
"""

import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.urdf import generate_urdf_xacro
from robot_model import (
    Geometry,
    Inertial,
    Joint,
    JointType,
    Link,
    Material,
    Pose,
    Robot,
    ValidationError,
)


def make_two_link_arm() -> Robot:
    """Same shape as tests/robot_model/test_schema.py's fixture, extended
    with visual/collision geometry and material so geometry rendering is
    exercised too."""
    base = Link(
        name="base_link",
        visual_geometry=Geometry(kind="box", size=(0.2, 0.2, 0.05)),
        collision_geometry=Geometry(kind="box", size=(0.2, 0.2, 0.05)),
        material=Material(name="grey", rgba=(0.5, 0.5, 0.5, 1.0)),
        inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01),
    )
    link1 = Link(
        name="link1",
        parent="base_link",
        visual_geometry=Geometry(
            kind="mesh", mesh_path="package://two_link_arm/meshes/link1.stl", scale=(1.0, 1.0, 1.0)
        ),
        collision_geometry=Geometry(kind="cylinder", radius=0.03, length=0.3),
        inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001),
    )
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        origin=Pose(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.57,
        upper_limit=1.57,
        velocity_limit=2.0,
        effort_limit=10.0,
    )
    return Robot(name="two_link_arm", links=[base, link1], joints=[joint1])


def make_three_link_robot_with_fixed_joint() -> Robot:
    """Two-link arm plus a third link attached via a fixed joint, and a
    sphere-geometry link, to broaden geometry-kind + joint-type coverage."""
    robot = make_two_link_arm()
    sensor_mount = Link(
        name="sensor_mount",
        parent="link1",
        visual_geometry=Geometry(kind="sphere", radius=0.02),
        inertial=Inertial(mass=0.05, ixx=0.0001, iyy=0.0001, izz=0.0001),
    )
    fixed_joint = Joint(
        name="sensor_mount_joint",
        type=JointType.FIXED,
        parent="link1",
        child="sensor_mount",
        origin=Pose(xyz=(0.0, 0.0, 0.3)),
    )
    robot.links.append(sensor_mount)
    robot.joints.append(fixed_joint)
    return robot


# --- basic well-formedness -------------------------------------------------


def test_output_parses_as_xml():
    robot = make_two_link_arm()
    xml_text = generate_urdf_xacro(robot)
    root = ET.fromstring(xml_text)
    assert root.tag == "robot"
    assert root.attrib["name"] == "two_link_arm"
    # xmlns:xacro is a namespace declaration, so a namespace-aware parser
    # (ElementTree included) consumes it during parsing rather than exposing
    # it in .attrib — assert on the raw serialized text instead, which is
    # what a xacro/URDF consumer that isn't namespace-aware actually reads.
    assert 'xmlns:xacro="http://www.ros.org/wiki/xacro"' in xml_text


def test_correct_link_and_joint_counts():
    robot = make_three_link_robot_with_fixed_joint()
    xml_text = generate_urdf_xacro(robot)
    root = ET.fromstring(xml_text)
    links = root.findall("link")
    joints = root.findall("joint")
    assert len(links) == 3
    assert len(joints) == 2
    assert {l.attrib["name"] for l in links} == {"base_link", "link1", "sensor_mount"}
    assert {j.attrib["name"] for j in joints} == {"joint1", "sensor_mount_joint"}


# --- joint type handling ----------------------------------------------


def test_revolute_joint_gets_limit_and_axis():
    robot = make_two_link_arm()
    root = ET.fromstring(generate_urdf_xacro(robot))
    joint1 = next(j for j in root.findall("joint") if j.attrib["name"] == "joint1")
    assert joint1.attrib["type"] == "revolute"
    axis = joint1.find("axis")
    limit = joint1.find("limit")
    assert axis is not None and axis.attrib["xyz"] == "0.0 0.0 1.0"
    assert limit is not None
    assert limit.attrib["lower"] == "-1.57"
    assert limit.attrib["upper"] == "1.57"
    assert limit.attrib["velocity"] == "2.0"
    assert limit.attrib["effort"] == "10.0"


def test_fixed_joint_has_no_axis_or_limit():
    robot = make_three_link_robot_with_fixed_joint()
    root = ET.fromstring(generate_urdf_xacro(robot))
    fixed_joint = next(j for j in root.findall("joint") if j.attrib["name"] == "sensor_mount_joint")
    assert fixed_joint.attrib["type"] == "fixed"
    assert fixed_joint.find("axis") is None
    assert fixed_joint.find("limit") is None
    # parent/child/origin are still present
    assert fixed_joint.find("parent").attrib["link"] == "link1"
    assert fixed_joint.find("child").attrib["link"] == "sensor_mount"
    assert fixed_joint.find("origin") is not None


def test_continuous_joint_without_velocity_or_effort_omits_limit():
    base = Link(name="base_link")
    wheel = Link(name="wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="wheel",
        axis=(0.0, 1.0, 0.0),
    )
    robot = Robot(name="cont_test", links=[base, wheel], joints=[joint])
    root = ET.fromstring(generate_urdf_xacro(robot))
    wheel_joint = root.find("joint")
    assert wheel_joint.find("axis") is not None
    assert wheel_joint.find("limit") is None


def test_continuous_joint_with_both_velocity_and_effort_gets_limit():
    base = Link(name="base_link")
    wheel = Link(name="wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="wheel",
        axis=(0.0, 1.0, 0.0),
        velocity_limit=5.0,
        effort_limit=1.0,
    )
    robot = Robot(name="cont_test", links=[base, wheel], joints=[joint])
    root = ET.fromstring(generate_urdf_xacro(robot))
    limit = root.find("joint").find("limit")
    assert limit is not None
    assert limit.attrib["velocity"] == "5.0"
    assert limit.attrib["effort"] == "1.0"
    assert "lower" not in limit.attrib
    assert "upper" not in limit.attrib


def test_continuous_joint_with_only_one_of_velocity_effort_raises():
    base = Link(name="base_link")
    wheel = Link(name="wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="wheel",
        axis=(0.0, 1.0, 0.0),
        velocity_limit=5.0,
        # effort_limit intentionally left None
    )
    robot = Robot(name="cont_test", links=[base, wheel], joints=[joint])
    with pytest.raises(ValueError, match="only one of"):
        generate_urdf_xacro(robot)


# --- geometry kinds -----------------------------------------------------


def test_geometry_kinds_emit_correct_tags():
    robot = make_three_link_robot_with_fixed_joint()
    root = ET.fromstring(generate_urdf_xacro(robot))
    links_by_name = {l.attrib["name"]: l for l in root.findall("link")}

    base_visual_geom = links_by_name["base_link"].find("visual").find("geometry")
    assert base_visual_geom.find("box") is not None
    assert base_visual_geom.find("box").attrib["size"] == "0.2 0.2 0.05"

    link1_visual_geom = links_by_name["link1"].find("visual").find("geometry")
    mesh = link1_visual_geom.find("mesh")
    assert mesh is not None
    assert mesh.attrib["filename"] == "package://two_link_arm/meshes/link1.stl"
    assert mesh.attrib["scale"] == "1.0 1.0 1.0"

    link1_collision_geom = links_by_name["link1"].find("collision").find("geometry")
    cylinder = link1_collision_geom.find("cylinder")
    assert cylinder is not None
    assert cylinder.attrib["radius"] == "0.03"
    assert cylinder.attrib["length"] == "0.3"

    sensor_visual_geom = links_by_name["sensor_mount"].find("visual").find("geometry")
    sphere = sensor_visual_geom.find("sphere")
    assert sphere is not None
    assert sphere.attrib["radius"] == "0.02"


def test_material_and_color_rendered_on_visual_only():
    robot = make_two_link_arm()
    root = ET.fromstring(generate_urdf_xacro(robot))
    base_link = next(l for l in root.findall("link") if l.attrib["name"] == "base_link")
    material = base_link.find("visual").find("material")
    assert material is not None
    assert material.attrib["name"] == "grey"
    color = material.find("color")
    assert color is not None
    assert color.attrib["rgba"] == "0.5 0.5 0.5 1.0"
    # collision never gets a material per URDF spec
    assert base_link.find("collision").find("material") is None


def test_link_with_no_visual_collision_or_inertial_is_minimal_and_valid():
    base = Link(name="base_link")
    leaf = Link(name="leaf", parent="base_link")
    joint = Joint(name="j", type=JointType.FIXED, parent="base_link", child="leaf")
    robot = Robot(name="minimal", links=[base, leaf], joints=[joint])
    root = ET.fromstring(generate_urdf_xacro(robot))
    leaf_elem = next(l for l in root.findall("link") if l.attrib["name"] == "leaf")
    assert leaf_elem.find("visual") is None
    assert leaf_elem.find("collision") is None
    assert leaf_elem.find("inertial") is None


# --- determinism ----------------------------------------------------------


def test_generation_is_deterministic():
    robot = make_three_link_robot_with_fixed_joint()
    first = generate_urdf_xacro(robot)
    second = generate_urdf_xacro(robot)
    assert first == second


# --- error propagation ---------------------------------------------------


def test_invalid_robot_raises_validation_error():
    base = Link(name="base_link")
    orphan = Link(name="orphan", parent="ghost")
    robot = Robot(name="broken", links=[base, orphan], joints=[])
    with pytest.raises(ValidationError):
        generate_urdf_xacro(robot)


def test_revolute_joint_missing_velocity_limit_raises_value_error():
    base = Link(name="base_link")
    link1 = Link(name="link1", parent="base_link")
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        effort_limit=10.0,
        # velocity_limit intentionally left None
    )
    robot = Robot(name="incomplete_arm", links=[base, link1], joints=[joint])
    with pytest.raises(ValueError, match="velocity_limit"):
        generate_urdf_xacro(robot)


def test_revolute_joint_missing_effort_limit_raises_value_error():
    base = Link(name="base_link")
    link1 = Link(name="link1", parent="base_link")
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0,
        # effort_limit intentionally left None
    )
    robot = Robot(name="incomplete_arm", links=[base, link1], joints=[joint])
    with pytest.raises(ValueError, match="effort_limit"):
        generate_urdf_xacro(robot)


# --- real ROS parser check --------------------------------------------

CHECK_URDF = shutil.which("check_urdf") or "/opt/ros/lyrical/bin/check_urdf"


@pytest.mark.skipif(
    shutil.which("check_urdf") is None and not Path("/opt/ros/lyrical/bin/check_urdf").exists(),
    reason="check_urdf binary not available on this machine",
)
def test_check_urdf_accepts_generated_xacro(tmp_path):
    robot = make_three_link_robot_with_fixed_joint()
    xml_text = generate_urdf_xacro(robot)
    urdf_path = tmp_path / "two_link_arm.urdf"
    urdf_path.write_text(xml_text)

    result = subprocess.run(
        [CHECK_URDF, str(urdf_path)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, (
        f"check_urdf rejected generated URDF.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    assert "Successfully Parsed XML" in result.stdout
