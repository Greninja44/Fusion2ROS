"""Tests for fusion_addin.generators.moveit.

Must run with plain `python3 -m pytest` -- no Fusion, no MoveIt 2
installation needed. `yaml` (PyYAML) is used here only to check the
*generator's output* is well-formed YAML; fusion_addin/generators/moveit.py
itself never imports it (see that module's docstring -- stdlib + robot_model
only).
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.moveit import (
    detect_moveit_suitability,
    generate_joint_limits_yaml,
    generate_kinematics_yaml,
    generate_moveit_controllers_yaml,
    generate_moveit_demo_launch,
    generate_srdf,
)
from robot_model import Actuator, Geometry, Inertial, Joint, JointType, Link, Material, Pose, Robot


# --- fixtures ----------------------------------------------------------------


def make_sample_arm() -> Robot:
    """Simple 3-link, 2-joint arm -- base_link -> upper_arm -> forearm,
    the shape described in ARCHITECTURE.md / examples/sample_arm.py."""
    base = Link(
        name="base_link",
        visual_geometry=Geometry(kind="cylinder", radius=0.08, length=0.05),
        material=Material(name="grey", rgba=(0.4, 0.4, 0.4, 1.0)),
        inertial=Inertial(mass=1.5, ixx=0.004, iyy=0.004, izz=0.006),
    )
    upper_arm = Link(
        name="upper_arm",
        parent="base_link",
        visual_geometry=Geometry(kind="box", size=(0.05, 0.05, 0.3)),
        inertial=Inertial(mass=0.8, center_of_mass=(0.0, 0.0, 0.15), ixx=0.006, iyy=0.006, izz=0.0007),
    )
    forearm = Link(
        name="forearm",
        parent="upper_arm",
        visual_geometry=Geometry(kind="box", size=(0.04, 0.04, 0.25)),
        inertial=Inertial(mass=0.4, center_of_mass=(0.0, 0.0, 0.125), ixx=0.002, iyy=0.002, izz=0.0003),
    )
    shoulder = Joint(
        name="shoulder_joint",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="upper_arm",
        origin=Pose(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-math.pi,
        upper_limit=math.pi,
        velocity_limit=2.0,
        effort_limit=20.0,
    )
    elbow = Joint(
        name="elbow_joint",
        type=JointType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Pose(xyz=(0.0, 0.0, 0.3)),
        axis=(0.0, 1.0, 0.0),
        # lower/upper straddle a non-zero-inclusive range so the "home"
        # group_state midpoint-fallback branch is exercised too.
        lower_limit=0.2,
        upper_limit=2.0,
        velocity_limit=1.5,
        effort_limit=10.0,
    )
    robot = Robot(
        name="sample_arm",
        links=[base, upper_arm, forearm],
        joints=[shoulder, elbow],
        actuators=[
            Actuator(name="shoulder_motor", type="electric_motor", joint="shoulder_joint"),
            Actuator(name="elbow_motor", type="electric_motor", joint="elbow_joint"),
        ],
    )
    robot.validate()
    return robot


def make_branchy_robot() -> Robot:
    """base_link -> upper_arm -> {forearm, camera_mount}: upper_arm has two
    children, so the tree is not a single chain to one leaf."""
    robot = make_sample_arm()
    camera_mount = Link(name="camera_mount", parent="upper_arm", inertial=Inertial(mass=0.05, ixx=1e-4, iyy=1e-4, izz=1e-4))
    camera_joint = Joint(
        name="camera_joint",
        type=JointType.FIXED,
        parent="upper_arm",
        child="camera_mount",
        origin=Pose(xyz=(0.02, 0.0, 0.1)),
    )
    robot.links.append(camera_mount)
    robot.joints.append(camera_joint)
    robot.validate()
    return robot


def make_drivetrain_robot() -> Robot:
    """A robot flagged as a mobile/drivetrain robot via metadata -- should
    be refused regardless of its chain shape."""
    base = Link(name="base_link", inertial=Inertial(mass=5.0, ixx=0.05, iyy=0.05, izz=0.08))
    wheel = Link(name="left_wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=1e-4, iyy=1e-4, izz=1e-4))
    wheel_joint = Joint(
        name="left_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="left_wheel",
        axis=(0.0, 1.0, 0.0),
        velocity_limit=10.0,
        effort_limit=5.0,
    )
    robot = Robot(
        name="rover",
        links=[base, wheel],
        joints=[wheel_joint],
        metadata={"drivetrain": {"type": "differential"}},
    )
    robot.validate()
    return robot


def make_no_moving_joints_robot() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    plate = Link(name="plate", parent="base_link", inertial=Inertial(mass=0.1, ixx=1e-4, iyy=1e-4, izz=1e-4))
    fixed = Joint(name="plate_joint", type=JointType.FIXED, parent="base_link", child="plate")
    robot = Robot(name="static_thing", links=[base, plate], joints=[fixed])
    robot.validate()
    return robot


def make_arm_missing_velocity_limit() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    link1 = Link(name="link1", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.57,
        upper_limit=1.57,
        velocity_limit=None,
        effort_limit=10.0,
    )
    robot = Robot(name="no_vel_arm", links=[base, link1], joints=[joint1])
    robot.validate()
    return robot


# --- detect_moveit_suitability -----------------------------------------------


def test_sample_arm_is_suitable():
    assert detect_moveit_suitability(make_sample_arm()) == []


def test_branchy_robot_is_unsuitable_with_clear_message():
    problems = detect_moveit_suitability(make_branchy_robot())
    assert len(problems) == 1
    assert "branch" in problems[0].lower()
    assert "upper_arm" in problems[0]


def test_drivetrain_robot_is_unsuitable_with_clear_message():
    problems = detect_moveit_suitability(make_drivetrain_robot())
    assert any("drivetrain" in p.lower() for p in problems)


def test_no_moving_joints_is_unsuitable():
    problems = detect_moveit_suitability(make_no_moving_joints_robot())
    assert any("non-fixed" in p.lower() for p in problems)


def test_invalid_robot_reported_via_validate_first():
    # Two roots -> robot.validate() itself fails; detect_moveit_suitability
    # should surface that rather than crash trying to walk the chain.
    base1 = Link(name="a", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    base2 = Link(name="b", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    robot = Robot(name="broken", links=[base1, base2], joints=[])
    problems = detect_moveit_suitability(robot)
    assert len(problems) == 1
    assert "validation" in problems[0].lower()


# --- generate_srdf ------------------------------------------------------------


def test_srdf_parses_and_has_expected_chain():
    robot = make_sample_arm()
    text = generate_srdf(robot)
    root = ET.fromstring(text)
    assert root.tag == "robot"
    assert root.attrib["name"] == "sample_arm"

    group = root.find("group")
    assert group.attrib["name"] == "arm"
    chain = group.find("chain")
    assert chain.attrib["base_link"] == "base_link"
    assert chain.attrib["tip_link"] == "forearm"


def test_srdf_home_group_state_has_exactly_expected_joints():
    robot = make_sample_arm()
    text = generate_srdf(robot)
    root = ET.fromstring(text)

    state = root.find("group_state")
    assert state.attrib["name"] == "home"
    assert state.attrib["group"] == "arm"

    joint_values = {j.attrib["name"]: float(j.attrib["value"]) for j in state.findall("joint")}
    assert set(joint_values.keys()) == {"shoulder_joint", "elbow_joint"}
    # shoulder: 0.0 is within [-pi, pi] -> home value 0.0
    assert joint_values["shoulder_joint"] == pytest.approx(0.0)
    # elbow: [0.2, 2.0] does not include 0.0 -> home value is the midpoint
    assert joint_values["elbow_joint"] == pytest.approx(1.1)


def test_srdf_disables_collisions_for_every_adjacent_pair():
    robot = make_sample_arm()
    text = generate_srdf(robot)
    root = ET.fromstring(text)

    pairs = {(e.attrib["link1"], e.attrib["link2"]) for e in root.findall("disable_collisions")}
    expected = {(j.parent, j.child) for j in robot.joints}
    assert pairs == expected
    assert len(pairs) == len(robot.joints)


def test_srdf_explicit_base_and_tip_link_for_branchy_robot():
    robot = make_branchy_robot()
    # detect_moveit_suitability refuses this robot as a whole, but a caller
    # can still target one branch explicitly.
    text = generate_srdf(robot, group_name="arm", base_link="base_link", tip_link="forearm")
    root = ET.fromstring(text)
    chain = root.find("group").find("chain")
    assert chain.attrib["tip_link"] == "forearm"

    text2 = generate_srdf(robot, group_name="camera", base_link="upper_arm", tip_link="camera_mount")
    root2 = ET.fromstring(text2)
    chain2 = root2.find("group").find("chain")
    assert chain2.attrib["base_link"] == "upper_arm"
    assert chain2.attrib["tip_link"] == "camera_mount"


# --- generate_joint_limits_yaml -----------------------------------------------


def test_joint_limits_yaml_parses_with_expected_entries():
    robot = make_sample_arm()
    text = generate_joint_limits_yaml(robot)
    data = yaml.safe_load(text)
    assert set(data["joint_limits"].keys()) == {"shoulder_joint", "elbow_joint"}
    shoulder = data["joint_limits"]["shoulder_joint"]
    assert shoulder["has_velocity_limits"] is True
    assert shoulder["max_velocity"] == pytest.approx(2.0)
    assert shoulder["has_acceleration_limits"] is False


def test_joint_limits_yaml_raises_on_missing_velocity_limit():
    robot = make_arm_missing_velocity_limit()
    with pytest.raises(ValueError, match="joint1"):
        generate_joint_limits_yaml(robot)


# --- generate_kinematics_yaml --------------------------------------------------


def test_kinematics_yaml_parses_with_kdl_plugin_and_defaults():
    robot = make_sample_arm()
    text = generate_kinematics_yaml(robot, group_name="arm")
    data = yaml.safe_load(text)
    arm_cfg = data["arm"]
    assert arm_cfg["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
    assert arm_cfg["kinematics_solver_search_resolution"] == pytest.approx(0.005)
    assert arm_cfg["kinematics_solver_timeout"] == pytest.approx(0.005)


# --- generate_moveit_controllers_yaml ------------------------------------------


def test_moveit_controllers_yaml_parses_with_expected_shape():
    robot = make_sample_arm()
    text = generate_moveit_controllers_yaml(robot, group_name="arm")
    data = yaml.safe_load(text)
    mgr = data["moveit_simple_controller_manager"]
    assert mgr["controller_names"] == ["arm_controller"]
    controller = mgr["arm_controller"]
    assert controller["action_ns"] == "follow_joint_trajectory"
    assert controller["type"] == "FollowJointTrajectory"
    assert controller["default"] is True
    assert controller["joints"] == ["shoulder_joint", "elbow_joint"]


# --- generate_moveit_demo_launch -----------------------------------------------


def test_demo_launch_is_syntactically_valid_python():
    robot = make_sample_arm()
    text = generate_moveit_demo_launch(robot)
    compile(text, "<generated moveit demo launch>", "exec")


def test_demo_launch_references_expected_names():
    robot = make_sample_arm()
    text = generate_moveit_demo_launch(robot, group_name="arm")
    assert "move_group" in text
    assert "rviz2" in text
    assert "sample_arm_moveit_config" in text
    assert "sample_arm.srdf" in text
    assert "generate_launch_description" in text


def test_demo_launch_moveit_config_package_override():
    # fusion_addin/app.py's integration puts every generator's output into
    # ONE combined package rather than a separate "<robot>_moveit_config"
    # package -- confirmed for real that move_group fails to find the
    # default name in that setup, hence this override parameter.
    robot = make_sample_arm()
    text = generate_moveit_demo_launch(robot, group_name="arm", moveit_config_package="sample_arm")
    assert 'MOVEIT_CONFIG_PACKAGE = "sample_arm"' in text
    assert "sample_arm_moveit_config" not in text
