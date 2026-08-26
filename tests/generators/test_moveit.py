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
    detect_moveit_groups,
    detect_moveit_suitability,
    generate_joint_limits_yaml,
    generate_joint_limits_yaml_multi_group,
    generate_kinematics_yaml,
    generate_kinematics_yaml_multi_group,
    generate_moveit_controllers_yaml,
    generate_moveit_controllers_yaml_multi_group,
    generate_moveit_demo_launch,
    generate_moveit_demo_launch_multi_group,
    generate_ompl_planning_yaml,
    generate_srdf,
    generate_srdf_multi_group,
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


def make_arm_with_gripper() -> Robot:
    """base_link -> upper_arm -> forearm -> wrist_link, then wrist_link
    branches: {wrist_link -> tool0 -> tcp_link} (2 fixed joints, the arm's
    own nominal tool frame -- the LONGER branch) and
    {wrist_link -> gripper_link} (1 prismatic joint -- the SHORTER
    branch). Exactly one 2-way branch point (wrist_link) -- the simple
    "arm with a gripper" shape detect_moveit_groups is built to handle:
    the longer branch (base_link -> tcp_link) should become the "arm"
    group, the shorter branch (wrist_link -> gripper_link) the "gripper"
    group."""
    robot = make_sample_arm()
    wrist_link = Link(
        name="wrist_link",
        parent="forearm",
        inertial=Inertial(mass=0.2, ixx=1e-4, iyy=1e-4, izz=1e-4),
    )
    tool0 = Link(name="tool0", parent="wrist_link", inertial=Inertial(mass=0.01, ixx=1e-5, iyy=1e-5, izz=1e-5))
    tcp_link = Link(name="tcp_link", parent="tool0", inertial=Inertial(mass=0.01, ixx=1e-5, iyy=1e-5, izz=1e-5))
    gripper_link = Link(
        name="gripper_link", parent="wrist_link", inertial=Inertial(mass=0.05, ixx=1e-4, iyy=1e-4, izz=1e-4)
    )

    wrist_joint = Joint(
        name="wrist_joint",
        type=JointType.REVOLUTE,
        parent="forearm",
        child="wrist_link",
        origin=Pose(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-math.pi,
        upper_limit=math.pi,
        velocity_limit=2.0,
        effort_limit=10.0,
    )
    tool_joint = Joint(name="tool_joint", type=JointType.FIXED, parent="wrist_link", child="tool0")
    tcp_joint = Joint(name="tcp_joint", type=JointType.FIXED, parent="tool0", child="tcp_link")
    gripper_joint = Joint(
        name="gripper_joint",
        type=JointType.PRISMATIC,
        parent="wrist_link",
        child="gripper_link",
        axis=(1.0, 0.0, 0.0),
        lower_limit=0.0,
        upper_limit=0.04,
        velocity_limit=0.5,
        effort_limit=50.0,
    )

    robot.links.extend([wrist_link, tool0, tcp_link, gripper_link])
    robot.joints.extend([wrist_joint, tool_joint, tcp_joint, gripper_joint])
    robot.actuators.append(Actuator(name="wrist_motor", type="electric_motor", joint="wrist_joint"))
    robot.actuators.append(Actuator(name="gripper_motor", type="electric_motor", joint="gripper_joint"))
    robot.validate()
    return robot


def make_two_branch_point_robot() -> Robot:
    """Same as make_arm_with_gripper, but with a SECOND branch point
    (forearm also grows a fixed camera_mount child) -- more than one fork,
    which detect_moveit_groups deliberately refuses rather than guess."""
    robot = make_arm_with_gripper()
    camera_mount = Link(name="camera_mount", parent="forearm", inertial=Inertial(mass=0.05, ixx=1e-4, iyy=1e-4, izz=1e-4))
    camera_joint = Joint(name="camera_joint", type=JointType.FIXED, parent="forearm", child="camera_mount")
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


def test_arm_with_gripper_still_refused_by_single_chain_suitability():
    # detect_moveit_suitability is unchanged: it still refuses ANY branchy
    # robot, even the "arm with a gripper" shape detect_moveit_groups is
    # built to handle -- only detect_moveit_groups implements that heuristic.
    problems = detect_moveit_suitability(make_arm_with_gripper())
    assert len(problems) == 1
    assert "branch" in problems[0].lower()
    assert "wrist_link" in problems[0]


# --- detect_moveit_groups -----------------------------------------------------


def test_detect_moveit_groups_zero_branch_matches_single_chain_detection():
    problems, groups = detect_moveit_groups(make_sample_arm())
    assert problems == []
    assert groups == [("arm", "base_link", "forearm")]


def test_detect_moveit_groups_arm_with_gripper():
    problems, groups = detect_moveit_groups(make_arm_with_gripper())
    assert problems == []
    assert len(groups) == 2

    by_name = {name: (base, tip) for name, base, tip in groups}
    assert set(by_name.keys()) == {"arm", "gripper"}
    # Longer branch (base_link -> tcp_link, 5 joints) is "arm"; shorter
    # branch (wrist_link -> gripper_link, 1 joint) is "gripper".
    assert by_name["arm"] == ("base_link", "tcp_link")
    assert by_name["gripper"] == ("wrist_link", "gripper_link")


def test_detect_moveit_groups_refuses_multiple_branch_points():
    problems, groups = detect_moveit_groups(make_two_branch_point_robot())
    assert groups == []
    assert len(problems) == 1
    assert "branch" in problems[0].lower()
    assert "forearm" in problems[0]
    assert "wrist_link" in problems[0]


def test_detect_moveit_groups_refuses_drivetrain_robot():
    problems, groups = detect_moveit_groups(make_drivetrain_robot())
    assert groups == []
    assert any("drivetrain" in p.lower() for p in problems)


def test_detect_moveit_groups_refuses_invalid_robot_via_validate_first():
    base1 = Link(name="a", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    base2 = Link(name="b", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    robot = Robot(name="broken", links=[base1, base2], joints=[])
    problems, groups = detect_moveit_groups(robot)
    assert groups == []
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


# --- generate_srdf_multi_group -------------------------------------------------


def test_srdf_multi_group_single_group_matches_generate_srdf():
    robot = make_sample_arm()
    multi_text = generate_srdf_multi_group(robot, [("arm", "base_link", "forearm")])
    single_text = generate_srdf(robot, group_name="arm")

    multi_root = ET.fromstring(multi_text)
    single_root = ET.fromstring(single_text)

    # Same group/chain.
    assert ET.tostring(multi_root.find("group")) == ET.tostring(single_root.find("group"))
    # Same home group_state joints/values.
    multi_state = multi_root.find("group_state")
    single_state = single_root.find("group_state")
    assert multi_state.attrib == single_state.attrib
    multi_joints = {j.attrib["name"]: j.attrib["value"] for j in multi_state.findall("joint")}
    single_joints = {j.attrib["name"]: j.attrib["value"] for j in single_state.findall("joint")}
    assert multi_joints == single_joints
    # Same disable_collisions pairs.
    multi_pairs = {(e.attrib["link1"], e.attrib["link2"]) for e in multi_root.findall("disable_collisions")}
    single_pairs = {(e.attrib["link1"], e.attrib["link2"]) for e in single_root.findall("disable_collisions")}
    assert multi_pairs == single_pairs


def test_srdf_multi_group_arm_and_gripper():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_srdf_multi_group(robot, groups)
    root = ET.fromstring(text)

    top_level_groups = {g.attrib["name"]: g for g in root.findall("group")}
    # "arm", "gripper", plus the composite "arm_gripper" group.
    assert set(top_level_groups.keys()) == {"arm", "gripper", "arm_gripper"}

    arm_chain = top_level_groups["arm"].find("chain")
    assert arm_chain.attrib == {"base_link": "base_link", "tip_link": "tcp_link"}
    gripper_chain = top_level_groups["gripper"].find("chain")
    assert gripper_chain.attrib == {"base_link": "wrist_link", "tip_link": "gripper_link"}

    # Composite group references both subgroups by name, no <chain> of its own.
    composite = top_level_groups["arm_gripper"]
    assert composite.find("chain") is None
    subgroup_names = {g.attrib["name"] for g in composite.findall("group")}
    assert subgroup_names == {"arm", "gripper"}

    # Exactly one combined home group_state, covering every non-fixed joint
    # from both chains.
    states = root.findall("group_state")
    assert len(states) == 1
    assert states[0].attrib["group"] == "arm_gripper"
    joint_names = {j.attrib["name"] for j in states[0].findall("joint")}
    assert joint_names == {"shoulder_joint", "elbow_joint", "wrist_joint", "gripper_joint"}

    # disable_collisions covers every adjacent pair in the WHOLE robot, not
    # just the two groups' chains.
    pairs = {(e.attrib["link1"], e.attrib["link2"]) for e in root.findall("disable_collisions")}
    assert pairs == {(j.parent, j.child) for j in robot.joints}


def test_srdf_multi_group_rejects_empty_groups_list():
    with pytest.raises(ValueError, match="at least one"):
        generate_srdf_multi_group(make_sample_arm(), [])


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


# --- generate_joint_limits_yaml_multi_group ------------------------------------


def test_joint_limits_yaml_multi_group_covers_both_groups():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_joint_limits_yaml_multi_group(robot, groups)
    data = yaml.safe_load(text)
    assert set(data["joint_limits"].keys()) == {"shoulder_joint", "elbow_joint", "wrist_joint", "gripper_joint"}
    gripper = data["joint_limits"]["gripper_joint"]
    assert gripper["has_velocity_limits"] is True
    assert gripper["max_velocity"] == pytest.approx(0.5)


def test_joint_limits_yaml_multi_group_raises_on_missing_velocity_limit():
    robot = make_arm_missing_velocity_limit()
    with pytest.raises(ValueError, match="joint1"):
        generate_joint_limits_yaml_multi_group(robot, [("arm", "base_link", "link1")])


def test_joint_limits_yaml_multi_group_rejects_empty_groups_list():
    with pytest.raises(ValueError, match="at least one"):
        generate_joint_limits_yaml_multi_group(make_sample_arm(), [])


# --- generate_kinematics_yaml --------------------------------------------------


def test_kinematics_yaml_parses_with_kdl_plugin_and_defaults():
    robot = make_sample_arm()
    text = generate_kinematics_yaml(robot, group_name="arm")
    data = yaml.safe_load(text)
    arm_cfg = data["arm"]
    assert arm_cfg["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
    assert arm_cfg["kinematics_solver_search_resolution"] == pytest.approx(0.005)
    assert arm_cfg["kinematics_solver_timeout"] == pytest.approx(0.005)


# --- generate_kinematics_yaml_multi_group --------------------------------------


def test_kinematics_yaml_multi_group_has_one_entry_per_group():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_kinematics_yaml_multi_group(robot, groups)
    data = yaml.safe_load(text)
    assert set(data.keys()) == {"arm", "gripper"}
    for group_name in ("arm", "gripper"):
        cfg = data[group_name]
        assert cfg["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
        assert cfg["kinematics_solver_search_resolution"] == pytest.approx(0.005)
        assert cfg["kinematics_solver_timeout"] == pytest.approx(0.005)


def test_kinematics_yaml_multi_group_rejects_empty_groups_list():
    with pytest.raises(ValueError, match="at least one"):
        generate_kinematics_yaml_multi_group(make_sample_arm(), [])


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


# --- generate_moveit_controllers_yaml_multi_group ------------------------------


def test_moveit_controllers_yaml_multi_group_arm_gets_follow_joint_trajectory_gripper_gets_gripper_command():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_moveit_controllers_yaml_multi_group(robot, groups)
    data = yaml.safe_load(text)
    mgr = data["moveit_simple_controller_manager"]
    assert set(mgr["controller_names"]) == {"arm_controller", "gripper_controller"}

    arm_controller = mgr["arm_controller"]
    assert arm_controller["action_ns"] == "follow_joint_trajectory"
    assert arm_controller["type"] == "FollowJointTrajectory"
    assert arm_controller["default"] is True
    assert arm_controller["joints"] == ["shoulder_joint", "elbow_joint", "wrist_joint"]

    gripper_controller = mgr["gripper_controller"]
    assert gripper_controller["action_ns"] == "gripper_cmd"
    assert gripper_controller["type"] == "GripperCommand"
    assert gripper_controller["default"] is True
    assert gripper_controller["joints"] == ["gripper_joint"]


def test_moveit_controllers_yaml_multi_group_single_group_has_no_gripper_controller():
    robot = make_sample_arm()
    text = generate_moveit_controllers_yaml_multi_group(robot, [("arm", "base_link", "forearm")])
    data = yaml.safe_load(text)
    mgr = data["moveit_simple_controller_manager"]
    assert mgr["controller_names"] == ["arm_controller"]
    assert mgr["arm_controller"]["type"] == "FollowJointTrajectory"


def test_moveit_controllers_yaml_multi_group_custom_gripper_group_names():
    # A caller can override which group name(s) count as "gripper" instead
    # of relying on the literal name "gripper".
    robot = make_arm_with_gripper()
    groups = [("main_arm", "base_link", "tcp_link"), ("end_effector", "wrist_link", "gripper_link")]
    text = generate_moveit_controllers_yaml_multi_group(robot, groups, gripper_group_names=["end_effector"])
    data = yaml.safe_load(text)
    mgr = data["moveit_simple_controller_manager"]
    assert mgr["main_arm_controller"]["type"] == "FollowJointTrajectory"
    assert mgr["end_effector_controller"]["type"] == "GripperCommand"
    assert mgr["end_effector_controller"]["action_ns"] == "gripper_cmd"


def test_moveit_controllers_yaml_multi_group_rejects_empty_groups_list():
    with pytest.raises(ValueError, match="at least one"):
        generate_moveit_controllers_yaml_multi_group(make_sample_arm(), [])


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


def test_demo_launch_loads_ompl_planning_pipeline():
    # Regression: move_group throws std::runtime_error and terminates
    # immediately ("Planning plugin name is empty or not defined in
    # namespace 'move_group'") with no planning pipeline registered at
    # all -- confirmed for real against a live move_group. The launch file
    # must load config/ompl_planning.yaml and pass planning_pipelines/
    # default_planning_pipeline/ompl to move_group_node.
    robot = make_sample_arm()
    text = generate_moveit_demo_launch(robot)
    assert "ompl_planning.yaml" in text
    assert "default_planning_pipeline" in text
    assert "planning_pipeline_config" in text


# --- generate_moveit_demo_launch_multi_group -----------------------------------


def test_demo_launch_multi_group_is_syntactically_valid_python():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_moveit_demo_launch_multi_group(robot, groups)
    compile(text, "<generated multi-group moveit demo launch>", "exec")


def test_demo_launch_multi_group_references_expected_names():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_moveit_demo_launch_multi_group(robot, groups, moveit_config_package=robot.name)
    assert "move_group" in text
    assert "rviz2" in text
    assert f'MOVEIT_CONFIG_PACKAGE = "{robot.name}"' in text
    assert f"{robot.name}.srdf" in text
    assert "generate_launch_description" in text
    assert "PLANNING_GROUPS" in text
    assert "'arm'" in text or '"arm"' in text
    assert "'gripper'" in text or '"gripper"' in text


def test_demo_launch_multi_group_loads_ompl_planning_pipeline():
    robot = make_arm_with_gripper()
    _, groups = detect_moveit_groups(robot)
    text = generate_moveit_demo_launch_multi_group(robot, groups)
    assert "ompl_planning.yaml" in text
    assert "default_planning_pipeline" in text
    assert "planning_pipeline_config" in text


def test_demo_launch_multi_group_rejects_empty_groups_list():
    with pytest.raises(ValueError, match="at least one"):
        generate_moveit_demo_launch_multi_group(make_sample_arm(), [])


def test_ompl_planning_yaml_parses_and_has_required_plugin():
    text = generate_ompl_planning_yaml()
    data = yaml.safe_load(text)
    assert data["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert "request_adapters" in data
    assert "response_adapters" in data


def test_ompl_planning_yaml_is_robot_independent():
    # No `robot` parameter -- must return byte-identical content regardless
    # of which robot it's generated for.
    assert generate_ompl_planning_yaml() == generate_ompl_planning_yaml()
