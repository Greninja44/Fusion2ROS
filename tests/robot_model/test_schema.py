"""Tests for robot_model.schema. Must run with plain `python3 -m pytest` —
no Fusion, no ROS, no network. That's the whole point of this package."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_model import (
    Actuator,
    Geometry,
    Inertial,
    Joint,
    JointType,
    Link,
    Pose,
    Robot,
    Sensor,
    ValidationError,
)


def make_two_link_arm() -> Robot:
    base = Link(
        name="base_link",
        inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01),
    )
    link1 = Link(
        name="link1",
        parent="base_link",
        inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001),
    )
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.57,
        upper_limit=1.57,
        velocity_limit=2.0,
        effort_limit=10.0,
    )
    return Robot(name="two_link_arm", links=[base, link1], joints=[joint1])


def test_valid_robot_passes_validation():
    robot = make_two_link_arm()
    assert robot.validate() == []


def test_root_link_lookup():
    robot = make_two_link_arm()
    assert robot.root_link().name == "base_link"


def test_missing_root_link_detected():
    a = Link(name="a", parent="b")
    b = Link(name="b", parent="a")
    joint = Joint(name="j", type=JointType.FIXED, parent="a", child="b")
    robot = Robot(name="cyclic", links=[a, b], joints=[joint])
    problems = robot.validate(raise_on_error=False)
    assert any("root link" in p for p in problems)


def test_multiple_roots_detected():
    a = Link(name="a")
    b = Link(name="b")
    robot = Robot(name="two_roots", links=[a, b], joints=[])
    problems = robot.validate(raise_on_error=False)
    assert any("multiple root links" in p for p in problems)


def test_disconnected_link_detected():
    base = Link(name="base_link")
    orphan = Link(name="orphan", parent="ghost")
    robot = Robot(name="broken", links=[base, orphan], joints=[])
    problems = robot.validate(raise_on_error=False)
    assert any("no joint has it as child" in p for p in problems)


def test_cycle_detected():
    base = Link(name="base_link")
    a = Link(name="a", parent="base_link")
    b = Link(name="b", parent="a")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="base_link", child="a")
    j2 = Joint(name="j2", type=JointType.FIXED, parent="a", child="b")
    j3 = Joint(name="j3", type=JointType.FIXED, parent="b", child="a")  # cycle back to a
    robot = Robot(name="cyclic", links=[base, a, b], joints=[j1, j2, j3])
    problems = robot.validate(raise_on_error=False)
    assert any("Cycle detected" in p for p in problems)


def test_duplicate_link_names_detected():
    a = Link(name="dup")
    b = Link(name="dup", parent="dup")
    robot = Robot(name="dup_test", links=[a, b], joints=[])
    problems = robot.validate(raise_on_error=False)
    assert any("Duplicate link name" in p for p in problems)


def test_joint_references_unknown_link():
    base = Link(name="base_link")
    j = Joint(name="j", type=JointType.FIXED, parent="base_link", child="nonexistent")
    robot = Robot(name="dangling", links=[base], joints=[j])
    problems = robot.validate(raise_on_error=False)
    assert any("unknown child link" in p for p in problems)


def test_validate_raises_by_default():
    base = Link(name="a")
    orphan = Link(name="b", parent="ghost")
    robot = Robot(name="broken", links=[base, orphan], joints=[])
    with pytest.raises(ValidationError):
        robot.validate()


def test_revolute_joint_requires_axis():
    with pytest.raises(ValueError):
        Joint(name="j", type=JointType.REVOLUTE, parent="a", child="b", lower_limit=0, upper_limit=1)


def test_revolute_joint_requires_limits():
    with pytest.raises(ValueError):
        Joint(name="j", type=JointType.REVOLUTE, parent="a", child="b", axis=(0, 0, 1))


def test_continuous_joint_rejects_limits():
    with pytest.raises(ValueError):
        Joint(
            name="j",
            type=JointType.CONTINUOUS,
            parent="a",
            child="b",
            axis=(0, 0, 1),
            lower_limit=-1,
            upper_limit=1,
        )


def test_fixed_joint_needs_no_axis_or_limits():
    j = Joint(name="j", type=JointType.FIXED, parent="a", child="b")
    assert j.axis is None


def test_lower_limit_exceeding_upper_rejected():
    with pytest.raises(ValueError):
        Joint(
            name="j",
            type=JointType.PRISMATIC,
            parent="a",
            child="b",
            axis=(1, 0, 0),
            lower_limit=1.0,
            upper_limit=-1.0,
        )


def test_negative_mass_rejected():
    with pytest.raises(ValueError):
        Inertial(mass=-1.0)


def test_zero_mass_rejected():
    with pytest.raises(ValueError):
        Inertial(mass=0.0)


def test_negative_effort_limit_rejected():
    with pytest.raises(ValueError):
        Joint(
            name="j",
            type=JointType.REVOLUTE,
            parent="a",
            child="b",
            axis=(0, 0, 1),
            lower_limit=-1,
            upper_limit=1,
            effort_limit=-5.0,
        )


def test_mesh_geometry_requires_path():
    with pytest.raises(ValueError):
        Geometry(kind="mesh")


def test_box_geometry_requires_size():
    with pytest.raises(ValueError):
        Geometry(kind="box")


def test_box_geometry_valid():
    g = Geometry(kind="box", size=(0.1, 0.2, 0.3))
    assert g.size == (0.1, 0.2, 0.3)


def test_pose_identity_default():
    p = Pose()
    assert p.xyz == (0.0, 0.0, 0.0)
    assert p.rpy == (0.0, 0.0, 0.0)


def test_pose_rejects_wrong_length():
    with pytest.raises(ValueError):
        Pose(xyz=(1.0, 2.0))


def test_sensor_and_actuator_reference_validation():
    base = Link(name="base_link", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", parent="base_link", inertial=Inertial(mass=0.5))
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0, 0, 1),
        lower_limit=-1,
        upper_limit=1,
    )
    good_sensor = Sensor(name="cam", type="camera", parent_link="link1")
    bad_sensor = Sensor(name="lidar", type="lidar", parent_link="nonexistent")
    good_actuator = Actuator(name="motor1", type="electric_motor", joint="joint1")
    bad_actuator = Actuator(name="motor2", type="electric_motor", joint="nonexistent")

    robot = Robot(
        name="sensored",
        links=[base, link1],
        joints=[joint1],
        sensors=[good_sensor, bad_sensor],
        actuators=[good_actuator, bad_actuator],
    )
    problems = robot.validate(raise_on_error=False)
    assert any("Sensor 'lidar'" in p for p in problems)
    assert any("Actuator 'motor2'" in p for p in problems)
    assert not any("cam" in p for p in problems)
    assert not any("motor1" in p for p in problems)
